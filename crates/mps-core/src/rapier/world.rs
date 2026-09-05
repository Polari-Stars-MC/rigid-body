use rapier3d::math::Rotation;
use rapier3d::prelude::fluid::FluidWorld;
use rapier3d::prelude::granular::GranularWorld;
use rapier3d::prelude::soft_body::{SoftBodyId, SoftBodySet};
use rapier3d::prelude::{
    ActiveHooks, BroadPhaseBvh, CCDSolver, ColliderHandle, ColliderSet, ImpulseJointSet,
    IntegrationParameters, IslandManager, MultibodyJointSet, NarrowPhase, PhysicsPipeline,
    RigidBodyHandle, RigidBodySet, Vector,
};
use std::sync::Arc;

#[cfg(feature = "relative-force")]
use dashmap::DashMap;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, ERR_UNSUPPORTED,
    clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    Bool, ColliderHandleRaw, MAX_OUTPUT_CAPACITY, Quat, RigidBodyHandleRaw, Vec3, WorldHandle,
    force_law_type_from_u32, isometry_from_parts, pack_rigid_body_handle, quat_finite,
    unpack_collider_handle, unpack_rigid_body_handle, vec3_finite, vec3_to_rapier,
};
use crate::rapier::forces::{BodyForceLog, ForceFacade, ForceRegistry};
use crate::rapier::registry::IdRegistry;
use crate::rapier::terrain_gravity::TerrainGravitySource;

const MAX_STEP_SECONDS: f64 = 1.0;

/// Preallocated working storage reused each frame to avoid per-step heap allocations.
pub(crate) struct FrameWorkBuffers {
    /// Per-body force log: indexed by handle index for O(1) access without hashing.
    /// Index = RigidBodyHandle::into_raw_parts().0 (arena index portion).
    /// Auto-expands when new bodies are inserted beyond current capacity.
    pub(crate) body_log: Vec<Option<BodyForceLog>>,
    /// Scratch buffer for Coulomb friction pairs (avoid per-frame Vec::new()).
    pub(crate) friction_work: Vec<(
        rapier3d::prelude::RigidBodyHandle,
        rapier3d::prelude::RigidBodyHandle,
        Vector,
    )>,
    /// Scratch buffer for legacy external force computation.
    pub(crate) pending_forces: smallvec::SmallVec<[crate::rapier::events::PendingForce; 128]>,
    /// Scratch buffer for arena command → handle mapping.
    pub(crate) arena_idx_map: Vec<Option<rapier3d::prelude::RigidBodyHandle>>,
    /// Reusable `(handle, force)` accumulator shared across ForceLaw::apply() calls.
    /// Cleared before each law runs; avoids per-law-per-frame SmallVec::new().
    pub(crate) scratch_force_pairs:
        smallvec::SmallVec<[(rapier3d::prelude::RigidBodyHandle, Vector); 64]>,
    /// Secondary `(handle, force)` scratch (e.g. PulsarMagneticDipole fallback path).
    pub(crate) scratch_force_pairs_alt:
        smallvec::SmallVec<[(rapier3d::prelude::RigidBodyHandle, Vector); 64]>,
    /// Reusable `(handle, mass, position)` buffer for pairwise gravity pre-collection.
    /// Avoids per-frame SmallVec allocation in NewtonianGravityForceLaw::apply().
    pub(crate) scratch_body_data:
        smallvec::SmallVec<[(rapier3d::prelude::RigidBodyHandle, f64, Vector); 64]>,
    /// P1.8: Coulomb hook 同步跟踪。`true` 时 `world_step` 末尾需要对一遍 collider
    /// 把 `MODIFY_SOLVER_CONTACTS` bit 设上；扫描完成后清零、记录当时的 collider 数量，
    /// 下次只在数量变化（新增/移除 collider）或 Coulomb law 切换时再扫。
    pub(crate) coulomb_hook_dirty: bool,
    /// P1.8: 上次完成 Coulomb hook 同步时的 `colliders.len()`，用于在 step 入口
    /// 廉价判定是否有新插入/移除的 collider 破坏了既有 hook 状态。
    pub(crate) coulomb_hook_last_collider_count: usize,
    /// P1.9: arena handle 映射的「当前正文长度」——上一次 `arena_idx_map` rebuild
    /// 时 `bodies.len()`。step 入口比较 `bodies.len()` 与此值，相等则跳过 clear+rebuild。
    pub(crate) arena_idx_map_body_count: usize,
}

impl Default for FrameWorkBuffers {
    fn default() -> Self {
        Self {
            body_log: Vec::with_capacity(256),
            friction_work: Vec::with_capacity(512),
            pending_forces: smallvec::SmallVec::new(),
            arena_idx_map: Vec::with_capacity(256),
            scratch_force_pairs: smallvec::SmallVec::new(),
            scratch_force_pairs_alt: smallvec::SmallVec::new(),
            scratch_body_data: smallvec::SmallVec::new(),
            coulomb_hook_dirty: true,
            coulomb_hook_last_collider_count: 0,
            arena_idx_map_body_count: 0,
        }
    }
}

pub struct PhysicsWorld {
    pub(crate) pipeline: PhysicsPipeline,
    pub(crate) gravity: Vector,
    pub(crate) integration_parameters: IntegrationParameters,
    pub(crate) islands: IslandManager,
    pub(crate) broad_phase: BroadPhaseBvh,
    pub(crate) narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub(crate) impulse_joints: ImpulseJointSet,
    pub(crate) multibody_joints: MultibodyJointSet,
    pub(crate) ccd_solver: CCDSolver,
    /// All soft bodies (deformable / point-mass + spring structures). Phase 0b
    /// wiring: stepped independently after the rigid-body pipeline each frame,
    /// mirroring the rapier `PhysicsWorld::soft_bodies` field. Particle gravity
    /// is applied inside `SoftBody::step`; bound particles route their spring
    /// forces into the rigid-body `force_containers` via `write_spring_forces`.
    pub soft_bodies: SoftBodySet,
    /// All SPH fluid bodies (particle clouds). Stepped independently after the
    /// rigid-body pipeline, mirroring `soft_bodies`. See
    /// `.hermes/plans/2026-08-30_fluid-sph-roadmap.md`.
    pub fluids: Vec<FluidWorld>,
    /// DEM granular bodies (Phase 36): particle clouds advanced independently
    /// after the rigid pipeline, mirroring `fluids`.
    pub granular_bodies: Vec<GranularWorld>,
    /// Phase 37: voxel-dig → grain spawn link. When set, digging a solid cell
    /// out of any voxel collider spawns one grain at the cell's world centre
    /// into the linked granular body (`id, grain_mass, grain_radius`).
    pub(crate) granular_dig_spawn: Option<(u32, f64, f64)>,
    /// Phase 38 (granular DEM ↔ rigid): per-body collision proxies, mirroring
    /// `fluid_proxies`. When enabled via `granular_enable_collision`, each
    /// particle is backed by a dynamic `RigidBody` (gravity_scale 0 — the DEM
    /// integrator applies gravity itself) + `Ball` collider. Poses sync into
    /// the proxies before the rigid step; the contacted poses are read back
    /// after it and before the DEM step, so collision response enters the
    /// cloud and the DEM integrator runs on the corrected particles.
    pub(crate) granular_proxies:
        std::collections::HashMap<u32, Vec<Option<rapier3d::prelude::RigidBodyHandle>>>,
    /// Phase 39: articulated chains (revolute-motor serial arms), keyed by
    /// articulation id. See `articulation.rs`.
    pub(crate) articulations: IdRegistry<crate::rapier::articulation::ArticulationBody>,
    /// Phase 2 (fluid SPH ↔ rigid): per-fluid collision proxies. When a fluid has
    /// collision coupling enabled (via `fluid_enable_collision`), each particle is
    /// backed by a dynamic `RigidBody` + `Ball` collider, parallel to
    /// `FluidWorld.particles`. The integration layer syncs particle poses into these
    /// proxies before the rigid step and reads the contacted poses back afterwards.
    /// Keyed by fluid id (its `Vec` index in `fluids`). Unlike soft-body proxies,
    /// fluid proxies keep the default (all-groups) collision filter so that fluid
    /// particles collide with each other (maintaining incompressibility) and with
    /// rigid terrain/entities.
    pub(crate) fluid_proxies:
        std::collections::HashMap<u32, Vec<Option<rapier3d::prelude::RigidBodyHandle>>>,
    /// Phase 3 (skinned soft body): skeleton bindings for linear-blend skinning.
    /// Keyed by soft-body id. Each entry maps a set of bone rigid bodies to
    /// per-particle weight bindings; `world_step` applies the skinning (driving
    /// particle positions from the live bone transforms) after the soft-body
    /// integration step.
    pub(crate) skin_bindings:
        std::collections::HashMap<u32, crate::rapier::soft_body::SkeletonBinding>,
    /// Character bodies (kinematic rigid body + `KinematicCharacterController`).
    /// Keyed by a stable id assigned at creation.
    pub(crate) character_bodies: IdRegistry<crate::rapier::character_body::CharacterBody>,
    /// Sensor trigger zones (sensor collider + overlap tracking). Keyed by a
    /// stable id assigned at creation.
    pub(crate) sensor_zones: IdRegistry<crate::rapier::sensor::SensorZone>,
    /// Ray-cast vehicle controllers. Keyed by a stable id assigned at
    /// creation.
    pub(crate) vehicle_controllers: IdRegistry<crate::rapier::vehicle::VehicleController>,
    /// Tire model controllers (Pacejka-style tire physics for vehicles).
    /// Keyed by a stable id assigned at creation.
    pub(crate) tire_models: IdRegistry<crate::rapier::tire_model::TireModel>,
    /// PD/PID servo bodies (dynamic rigid body + velocity-level servo
    /// controller). Keyed by a stable id assigned at creation.
    pub(crate) servo_bodies: IdRegistry<crate::rapier::servo_body::ServoBody>,
    /// Fracture mesh bodies (composite rigid bodies that can fracture).
    /// Keyed by a stable id assigned at creation.
    pub(crate) fracture_mesh_bodies: IdRegistry<crate::rapier::fracture_mesh::FractureMeshBody>,
    /// Hair/fur systems (hair strands attached to rigid bodies). Keyed by a
    /// stable id assigned at creation.
    pub(crate) hair_systems: IdRegistry<crate::rapier::hair::HairSystem>,
    /// Rope knot/weaving systems (per-strand soft bodies with collision
    /// proxies). Keyed by a stable id assigned at creation.
    pub(crate) rope_knots: IdRegistry<crate::rapier::rope_knot::RopeKnotSystem>,
    /// Phase 5d: per-soft-body voxel→particle mapping so a dug-out voxel cell can
    /// be mapped back to the exact particle index to remove via `soft_body_voxel_dig`.
    /// Keyed by `SoftBodyId.0`; populated only by `soft_body_voxel_build`.
    pub(crate) voxel_soft_meta:
        std::collections::HashMap<u32, crate::rapier::soft_body::VoxelSoftMeta>,
    /// Phase 5f: per-soft-body collision proxies. When a soft body has collision
    /// coupling enabled (`SoftBody.collide == true`), each free particle is backed
    /// by a dynamic `RigidBody` + `Ball` collider (keyed by `SoftBodyId.0`, parallel
    /// to `SoftBody.particles`). The integration layer syncs particle forces/poses
    /// into these proxies before the rigid-body step and reads the contacted poses
    /// back afterwards. Pinned particles (inv_mass == 0) have no proxy (`None`).
    pub(crate) soft_body_proxies:
        std::collections::HashMap<u32, Vec<Option<rapier3d::prelude::RigidBodyHandle>>>,
    pub(crate) hooks: crate::rapier::events::CallbackPhysicsHooks,
    pub(crate) events: Arc<crate::rapier::events::CollectingEventHandler>,
    /// World query lock. Acquired (read) by the synchronous query entry points
    /// (`query.rs`/`controller.rs`/`joints.rs`) so a caller inspecting the world
    /// mid-frame does not race the `world_step` integration. Restored after a
    /// refactor dropped the field while leaving its (no-op) acquire sites in place.
    pub(crate) query_lock: parking_lot::RwLock<()>,
    pub(crate) force_registry: ForceRegistry,
    /// Active terrain-gravity source (polyhedron / DEM / lunar-mascon), if any.
    /// Mirrors the registered `TerrainGravity` force law so the character
    /// controller can sample local gravity per-frame without re-parsing the law.
    pub(crate) terrain_gravity_source: Option<TerrainGravitySource>,
    pub(crate) shared_arena: Option<Box<crate::rapier::shared_arena::SharedPhysicsArena>>,
    /// Per-collider voxel source grid for in-place voxel edits. Keyed by the
    /// `ColliderHandleRaw` returned at insert time; populated only for
    /// colliders built from a voxel builder. Empty for non-voxel colliders.
    pub(crate) voxel_grids: std::collections::HashMap<
        crate::rapier::ffi::ColliderHandleRaw,
        crate::rapier::voxel::VoxelCache,
    >,
    /// Persistent per-frame work buffers — cleared and reused each `world_step`.
    pub(crate) buffers: FrameWorkBuffers,
    /// Relative force feature: per-body enabled state and local attachment point.
    #[cfg(feature = "relative-force")]
    pub(crate) relative_force: DashMap<RigidBodyHandleRaw, (bool, Vec3)>,
}

impl PhysicsWorld {
    pub(crate) fn new(gravity: Vec3) -> Self {
        let integration_parameters = IntegrationParameters {
            dt: 1.0 / 60.0,
            num_solver_iterations: 4,
            max_ccd_substeps: 4,
            normalized_prediction_distance: 0.005,
            num_internal_stabilization_iterations: 4,
            normalized_max_corrective_velocity: 1.0,
            warmstart_coefficient: 0.5,
            ..IntegrationParameters::default()
        };

        let events = Arc::new(crate::rapier::events::CollectingEventHandler::default());
        Self {
            pipeline: PhysicsPipeline::new(),
            gravity: vec3_to_rapier(gravity),
            integration_parameters,
            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            soft_bodies: SoftBodySet::new(),
            fluids: Vec::new(),
            granular_bodies: Vec::new(),
            granular_proxies: std::collections::HashMap::new(),
            articulations: IdRegistry::new(),
            granular_dig_spawn: None,
            fluid_proxies: std::collections::HashMap::new(),
            skin_bindings: std::collections::HashMap::new(),
            character_bodies: IdRegistry::new(),
            sensor_zones: IdRegistry::new(),
            vehicle_controllers: IdRegistry::new(),
            tire_models: IdRegistry::new(),
            hair_systems: IdRegistry::new(),
            rope_knots: IdRegistry::new(),
            servo_bodies: IdRegistry::new(),
            fracture_mesh_bodies: IdRegistry::new(),
            voxel_soft_meta: std::collections::HashMap::new(),
            soft_body_proxies: std::collections::HashMap::new(),
            hooks: crate::rapier::events::CallbackPhysicsHooks::new(events.clone()),
            events,
            query_lock: parking_lot::RwLock::new(()),
            force_registry: ForceRegistry::new(),
            terrain_gravity_source: None,
            shared_arena: None,
            voxel_grids: std::collections::HashMap::new(),
            buffers: FrameWorkBuffers::default(),
            #[cfg(feature = "relative-force")]
            relative_force: DashMap::new(),
        }
    }

    /// Enable or disable collision detection between two specific colliders,
    /// regardless of their collision groups, solver hooks, or whether they are
    /// connected by a joint.  Forwards to the narrow-phase's per-pair filter.
    pub(crate) fn set_collision_enabled(
        &mut self,
        c1: ColliderHandle,
        c2: ColliderHandle,
        enabled: bool,
    ) {
        if enabled {
            self.narrow_phase.enable_collision(c1, c2);
        } else {
            self.narrow_phase.disable_collision(c1, c2);
        }
    }
}

/// Create a new physics world.  Non-finite gravity components fall back to zero.
///
/// The returned pointer is owned by Rust; release it with `world_destroy`.
///
/// # Safety
/// No pointer arguments are dereferenced.  The returned pointer is owned by
/// Rust and must be released exactly once with `world_destroy`.
#[unsafe(no_mangle)]
pub extern "C" fn world_create(gravity: Vec3) -> *mut WorldHandle {
    ffi_guard(std::ptr::null_mut(), || {
        let gravity = if vec3_finite(gravity) {
            gravity
        } else {
            Vec3::default()
        };

        Box::into_raw(Box::new(WorldHandle {
            inner: PhysicsWorld::new(gravity),
        }))
    })
}

/// Destroy a physics world created by `world_create`.  Null is a no-op.
///
/// # Safety
/// `world` must be a pointer returned by `world_create` (or null) and must not
/// be used again after this call.
#[unsafe(no_mangle)]
pub extern "C" fn world_destroy(world: *mut WorldHandle) {
    ffi_guard((), || {
        if world.is_null() {
            return;
        }

        unsafe {
            drop(Box::from_raw(world));
        }
    })
}

/// Advance the simulation by `delta_seconds` (clamped to (0, 1]).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn world_step(world: *mut WorldHandle, delta_seconds: f64) {
    ffi_guard((), || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            return;
        };
        if !delta_seconds.is_finite() || delta_seconds <= 0.0 || delta_seconds > MAX_STEP_SECONDS {
            return;
        }

        // Thread-contract guard (see the `events` module docs): refuse to step
        // while an init-time event call holds the producer cache — stepping
        // would race its exclusive access. The Arc is cloned to a local so the
        // guard does not pin a borrow of `world` for the whole step body
        // (fracture-mesh auto-fracturing below needs `&mut WorldHandle`).
        let events = std::sync::Arc::clone(&world.inner.events);
        let Some(_step_guard) = events.step_guard() else {
            set_error(
                ERR_UNSUPPORTED,
                "world_step during event ring/callback init",
            );
            return;
        };

        world.inner.integration_parameters.dt = delta_seconds;

        // --- Arena: drain Java commands before applying forces ---
        // Java writes forces/set-poses/impulses via shared memory, Rust reads them here.
        if let Some(ref arena) = world.inner.shared_arena {
            let commands = arena.drain_commands();
            if !commands.is_empty() {
                // P1.9: arena_idx_map 增量更新——只在 `bodies.len()` 与上次不一致时
                // clear+rebuild；相等时 arena handle 顺序未变（rapier 维持插入次序），
                // 直接复用既有内容，消除每帧 O(n) rebuild。
                let n_bodies = world.inner.bodies.len();
                if n_bodies != world.inner.buffers.arena_idx_map_body_count {
                    let idx = &mut world.inner.buffers.arena_idx_map;
                    idx.clear();
                    for (h, _) in world.inner.bodies.iter() {
                        idx.push(Some(h));
                    }
                    world.inner.buffers.arena_idx_map_body_count = n_bodies;
                }
                let idx = &world.inner.buffers.arena_idx_map;
                for (cmd_type, body_idx, a0, a1, a2) in commands {
                    if let Some(Some(h)) = idx.get(body_idx as usize)
                        && let Some(body) = world.inner.bodies.get_mut(*h)
                    {
                        match cmd_type {
                            0 => {
                                // AddForce
                                body.add_force(rapier3d::prelude::Vector::new(a0, a1, a2), true);
                            }
                            1 => {
                                // AddTorque
                                body.add_torque(rapier3d::prelude::Vector::new(a0, a1, a2), true);
                            }
                            2 => {
                                // SetPose
                                // a0..a2 = position, rest packed into user_data via cmd encoding
                                let pos = rapier3d::prelude::Pose::from_parts(
                                    rapier3d::prelude::Vector::new(a0, a1, a2),
                                    *body.rotation(),
                                );
                                body.set_position(pos, true);
                            }
                            3 => {
                                // SetVelocity
                                body.set_linvel(rapier3d::prelude::Vector::new(a0, a1, a2), true);
                            }
                            4 => {
                                // ApplyImpulse
                                body.apply_impulse(
                                    rapier3d::prelude::Vector::new(a0, a1, a2),
                                    true,
                                );
                            }
                            5 => {
                                // ApplyTorqueImpulse
                                body.apply_torque_impulse(
                                    rapier3d::prelude::Vector::new(a0, a1, a2),
                                    true,
                                );
                            }
                            6 => {
                                // WakeUp
                                body.wake_up(true);
                            }
                            7 => {
                                // Sleep
                                body.sleep();
                            }
                            8 => {
                                // SetRotation — a0..a2 = rotation vector (axis-angle)
                                let axis_angle = rapier3d::prelude::Vector::new(a0, a1, a2);
                                let angle = axis_angle.length();
                                if angle > 1e-12 {
                                    let unit_axis = axis_angle / angle;
                                    body.set_rotation(
                                        rapier3d::prelude::Rotation::from_axis_angle(
                                            unit_axis, angle,
                                        ),
                                        true,
                                    );
                                }
                            }
                            9 => {
                                // SetGravityScale — a0 = scale
                                body.set_gravity_scale(a0, true);
                            }
                            10 => {
                                // SetLinearDamping — a0 = damping
                                body.set_linear_damping(a0);
                            }
                            11 => {
                                // SetAngularDamping — a0 = damping
                                body.set_angular_damping(a0);
                            }
                            12 => {
                                // AddForceAtPoint — a0..a2 = force, need point from next cmd or use COM
                                body.add_force(rapier3d::prelude::Vector::new(a0, a1, a2), true);
                            }
                            _ => {}
                        }
                    }
                }
            }
        }

        // --- Coulomb hook setup (P1.8: dirty flag + collider count guard) ---
        // 稳态下整段退化为 O(1)（`coulomb_hook_dirty=false` 且 collider 数量不变
        // 时跳过遍历），只在以下触发重扫：首帧 / Coulomb law 切换（setter 把
        // dirty 标 true）/ collider 数量变化（新增/移除）。rebuild 完毕后清零
        // dirty、记下当次 collider 数量，下次只在结构变化时再扫。
        let custom = world.inner.events.custom_physics();
        let coulomb_active = custom
            .coulomb_friction
            .is_some_and(|law| law.enabled.0 != 0);

        if !coulomb_active {
            // Coulomb 没启用：下次启用时强制扫一次（dirty 已是此处不再设回
            // false）。last_collider_count 同步清 0，保证重启用时即便 collider
            // 数量恰好与上次记下的相等，dirty 也会驱动扫描。
            world.inner.buffers.coulomb_hook_dirty = true;
            world.inner.buffers.coulomb_hook_last_collider_count = 0;
        } else if world.inner.buffers.coulomb_hook_dirty
            || world.inner.colliders.len() != world.inner.buffers.coulomb_hook_last_collider_count
        {
            let hook_bit = ActiveHooks::MODIFY_SOLVER_CONTACTS;
            for (_, collider) in world.inner.colliders.iter_mut() {
                let current = collider.active_hooks();
                if !current.contains(hook_bit) {
                    collider.set_active_hooks(current | hook_bit);
                }
            }
            world.inner.buffers.coulomb_hook_last_collider_count = world.inner.colliders.len();
            world.inner.buffers.coulomb_hook_dirty = false;
        }

        // --- Force facade: the single entry-point for all force application ---
        // O1 fix: reuse persistent body_log (Vec-indexed by handle) instead of HashMap.
        // Take ownership of the buffers, use them, then put them back.
        let mut body_log = std::mem::take(&mut world.inner.buffers.body_log);
        let mut pending_forces = std::mem::take(&mut world.inner.buffers.pending_forces);
        let mut friction_work = std::mem::take(&mut world.inner.buffers.friction_work);
        let mut scratch_force_pairs = std::mem::take(&mut world.inner.buffers.scratch_force_pairs);
        let mut scratch_force_pairs_alt =
            std::mem::take(&mut world.inner.buffers.scratch_force_pairs_alt);
        let mut scratch_body_data = std::mem::take(&mut world.inner.buffers.scratch_body_data);
        let mut facade = ForceFacade::new(
            &mut world.inner.bodies,
            &mut world.inner.colliders,
            &world.inner.narrow_phase,
            world.inner.integration_parameters.dt,
            &mut body_log,
            &mut pending_forces,
            &mut friction_work,
            &mut scratch_force_pairs,
            &mut scratch_force_pairs_alt,
            &mut scratch_body_data,
        );

        // 1. Registered ForceLaw list (from new system)
        world.inner.force_registry.apply_all(&mut facade);

        // 2. Backward-compat: old unregistered external-force law setter path
        //   Work around borrowck by copying body handles/positions, then replaying forces through facade.
        crate::rapier::events::apply_custom_external_forces_with_facade(&custom, &mut facade);

        // 3. Backward-compat: old unregistered body-interaction path
        //   Same approach: compute forces first (immutable reads), then replay.
        crate::rapier::interaction::apply_body_interactions_with_facade(
            &world.inner.force_registry,
            &custom,
            &mut facade,
        );

        // 4. Drain the facade frame-log into a report and write it to events
        let force_report = facade.drain_report();
        // P1+P5 fix: put drained buffers back for next frame reuse
        let empty_log = std::mem::take(facade.body_log);
        world.inner.buffers.body_log = empty_log;
        world.inner.buffers.pending_forces = std::mem::take(facade.pending_forces);
        world.inner.buffers.friction_work = std::mem::take(facade.friction_work);
        world.inner.buffers.scratch_force_pairs = std::mem::take(facade.scratch_force_pairs);
        world.inner.buffers.scratch_force_pairs_alt =
            std::mem::take(facade.scratch_force_pairs_alt);
        world.inner.buffers.scratch_body_data = std::mem::take(facade.scratch_body_data);
        if force_report
            .contributions
            .values()
            .any(|c| c.body_count > 0)
        {
            world
                .inner
                .events
                .set_last_custom_physics_report(force_report.to_legacy_report());
        }

        // Phase 0b/2 wiring: route bound-particle spring forces into the rigid-body
        // `force_containers`, then advance the soft-body point masses (gravity +
        // Hookean springs) independently. Mirrors rapier's own `PhysicsWorld` step
        // order. Sleeping soft bodies are skipped inside `SoftBodySet::step`.
        // Phase 5f: collision-coupled soft bodies are driven by proxy rigid bodies,
        // so their particle forces/poses are pushed into the proxies *before* the
        // rigid-body step (narrow-phase/contact then sees the latest positions).
        for (sid_u32, proxies) in world.inner.soft_body_proxies.iter_mut() {
            let sid = SoftBodyId(*sid_u32);
            let Some(soft) = world.inner.soft_bodies.get_mut(sid) else {
                continue;
            };
            if !soft.collide {
                continue;
            }
            soft.compute_forces();
            for (i, p) in soft.particles.iter().enumerate() {
                let Some(Some(rb_h)) = proxies.get(i) else {
                    continue; // pinned particle has no proxy
                };
                let Some(rb) = world.inner.bodies.get_mut(*rb_h) else {
                    continue;
                };
                rb.set_translation(p.pos, false);
                rb.set_linvel(p.vel, false);
                rb.reset_forces(false);
                rb.add_force(p.force, false);
            }
        }
        world
            .inner
            .soft_bodies
            .write_spring_forces(&mut world.inner.bodies);
        world
            .inner
            .soft_bodies
            .step(world.inner.integration_parameters.dt);
        // Phase 3 (skinned soft body): drive bound particles from live bone
        // transforms via linear-blend skinning, after the soft-body integration.
        for (sb_id, skin) in &world.inner.skin_bindings {
            // Snapshot live bone transforms.
            let mut bone_trans: Vec<Vector> = Vec::with_capacity(skin.bones.len());
            let mut bone_rot: Vec<Rotation> = Vec::with_capacity(skin.bones.len());
            let mut ok = true;
            for h in &skin.bones {
                match world.inner.bodies.get(*h) {
                    Some(rb) => {
                        bone_trans.push(rb.translation());
                        bone_rot.push(*rb.rotation());
                    }
                    None => {
                        ok = false;
                        break;
                    }
                }
            }
            if !ok {
                continue; // a bone handle disappeared; skip this body this step
            }
            if let Some(sb) = world.inner.soft_bodies.get_mut(SoftBodyId(*sb_id)) {
                for (pi, vw) in &skin.vertices {
                    let mut acc = Vector::ZERO;
                    for k in 0..4 {
                        if vw.weight[k] > 0.0 {
                            let bi = vw.bone_idx[k] as usize;
                            if bi < bone_trans.len() {
                                let skinned = bone_rot[bi] * vw.local[k] + bone_trans[bi];
                                acc += vw.weight[k] * skinned;
                            }
                        }
                    }
                    if let Some(p) = sb.particles.get_mut(*pi as usize) {
                        p.pos = acc;
                    }
                }
            }
        }
        // Phase 14: world-level soft-soft (cross-body) collision. Runs after every
        // body has stepped; only bodies with `cross_collision` set collide with each
        // other (see `solve_cross_body_collisions`).
        rapier3d::dynamics::soft_body::solve_cross_body_collisions(
            &mut world.inner.soft_bodies,
            world.inner.integration_parameters.dt,
        );
        // Phase 17: world-level cohesion (adhesion / breakable glue) between bodies
        // with `cohesion` set (see `solve_cohesion`). Runs after cross-collision so the
        // two composes: bodies first repel on overlap, then glue together at contact.
        rapier3d::dynamics::soft_body::solve_cohesion(
            &mut world.inner.soft_bodies,
            world.inner.integration_parameters.dt,
        );

        // Phase 2 (fluid SPH ↔ rigid): sync each coupled fluid's particle poses into
        // its collision proxies *before* the rigid step, so the narrow-phase sees the
        // current particle positions/velocities and resolves contacts against terrain.
        for (fi, fluid) in world.inner.fluids.iter().enumerate() {
            #[allow(clippy::collapsible_if)]
            if let Some(proxies) = world.inner.fluid_proxies.get(&(fi as u32)) {
                for (pi, ph) in proxies.iter().enumerate() {
                    if let (Some(h), Some(p)) = (ph, fluid.particles.get(pi)) {
                        if let Some(rb) = world.inner.bodies.get_mut(*h) {
                            rb.set_translation(p.pos, true);
                            rb.set_linvel(p.vel, true);
                        }
                    }
                }
            }
        }

        // Phase 38 (granular DEM ↔ rigid): sync each granular particle's pose
        // into its collision proxy before the rigid step, mirroring fluids.
        for (gi, proxies) in world.inner.granular_proxies.iter() {
            if let Some(g) = world.inner.granular_bodies.get(*gi as usize) {
                for (pi, ph) in proxies.iter().enumerate() {
                    if let Some(p) = g.particles.get(pi)
                        && let Some(h) = ph
                        && let Some(rb) = world.inner.bodies.get_mut(*h)
                    {
                        rb.set_translation(p.pos, true);
                        rb.set_linvel(p.vel, true);
                    }
                }
            }
        }

        world.inner.pipeline.step(
            world.inner.gravity,
            &world.inner.integration_parameters,
            &mut world.inner.islands,
            &mut world.inner.broad_phase,
            &mut world.inner.narrow_phase,
            &mut world.inner.bodies,
            &mut world.inner.colliders,
            &mut world.inner.impulse_joints,
            &mut world.inner.multibody_joints,
            &mut world.inner.ccd_solver,
            &world.inner.hooks,
            &*world.inner.events,
        );

        // Phase 5f: read the contacted proxy poses back into the soft-body particles
        // so collision response propagates into the soft body (free particles only;
        // `SoftBody::step` is a no-op for `collide` bodies).
        for (sid_u32, proxies) in &world.inner.soft_body_proxies {
            let sid = SoftBodyId(*sid_u32);
            let Some(soft) = world.inner.soft_bodies.get_mut(sid) else {
                continue;
            };
            if !soft.collide {
                continue;
            }
            for (i, p) in soft.particles.iter_mut().enumerate() {
                let Some(Some(rb_h)) = proxies.get(i) else {
                    continue;
                };
                let Some(rb) = world.inner.bodies.get(*rb_h) else {
                    continue;
                };
                #[allow(clippy::clone_on_copy)]
                {
                    p.pos = rb.translation().clone();
                    p.vel = rb.linvel().clone();
                }
            }
        }

        // Phase 8: after the rigid pipeline has integrated, snap bound soft-body
        // particles to their rigid bodies' new world transforms (so an anchored
        // flag/cloth follows a moving object). Runs after `pipeline.step` so the
        // followers see the post-integration poses.
        world
            .inner
            .soft_bodies
            .follow_rigid_bodies(&world.inner.bodies);

        // Fracture mesh auto impact damage: accumulate this step's solver
        // contact impulses into every enabled fracture mesh body and
        // auto-fracture the ones past threshold. O(1) when none enabled.
        crate::rapier::fracture_mesh::accumulate_impact_damage(world);

        // Phase 0 (fluid SPH): advance every fluid particle cloud independently,
        // after the rigid-body pipeline. No rigid coupling yet.
        let dt = world.inner.integration_parameters.dt;
        for fluid in &mut world.inner.fluids {
            fluid.step(dt);
        }
        // Phase 2 (fluid SPH ↔ rigid): read the contacted proxy poses back into the
        // particles so collision response (against terrain/entities) propagates into
        // the fluid. Runs after `fluid.step` so the SPH integration happens first.
        for (fi, fluid) in world.inner.fluids.iter_mut().enumerate() {
            #[allow(clippy::collapsible_if)]
            if let Some(proxies) = world.inner.fluid_proxies.get(&(fi as u32)) {
                for (pi, ph) in proxies.iter().enumerate() {
                    if let Some(h) = ph {
                        if let Some(rb) = world.inner.bodies.get(*h) {
                            if let Some(p) = fluid.particles.get_mut(pi) {
                                p.pos = rb.translation();
                                p.vel = rb.linvel();
                            }
                        }
                    }
                }
            }
        }

        // Phase 38 (granular DEM ↔ rigid): read the contacted proxy poses back
        // into the particles (collision response), then run the DEM integrator
        // on the corrected cloud — gravity + inter-particle repulsion apply on
        // top of the collision response, and nothing is overwritten.
        for (gi, proxies) in world.inner.granular_proxies.iter() {
            if let Some(g) = world.inner.granular_bodies.get_mut(*gi as usize) {
                for (pi, ph) in proxies.iter().enumerate() {
                    if let Some(p) = g.particles.get_mut(pi)
                        && let Some(h) = ph
                        && let Some(rb) = world.inner.bodies.get(*h)
                    {
                        p.pos = rb.translation();
                        p.vel = rb.linvel();
                    }
                }
            }
        }
        for granular in &mut world.inner.granular_bodies {
            granular.step(dt);
        }

        // 4b. Clear the persistent user force/torque on every dynamic body.
        // Rapier's `add_force` is a *persistent* force that the step does NOT
        // clear, so a law's force (or a one-shot FFI force) keeps acting on every
        // subsequent frame until explicitly reset.  We clear it *after* the step,
        // once it has already been integrated into velocity — so clearing here is
        // harmless for the frame just simulated, but stops an unregistered law (or
        // a spent one-shot force) from acting forever.  Registered laws re-apply
        // their force each frame inside `apply_all` above, so they stay correct.
        for (_, body) in world.inner.bodies.iter_mut() {
            if body.is_dynamic() {
                body.reset_forces(false);
            }
        }

        // 5. Flush shared arena body/collider state → Java zero-JNI read
        if let Some(ref arena) = world.inner.shared_arena {
            arena.flush_all_bodies(&world.inner.bodies);
            arena.flush_all_colliders(&world.inner.colliders);
            arena.flush_integration_params(
                world.inner.integration_parameters.dt,
                world.inner.integration_parameters.num_solver_iterations as u32,
                world.inner.integration_parameters.max_ccd_substeps as u32,
                &world.inner.gravity,
            );
            let legacy = &force_report.to_legacy_report();
            arena.flush_force_report(
                force_report.max_reynolds_number,
                &legacy.total_external_force,
                &legacy.total_drag_force,
                legacy.drag_body_count,
                legacy.external_force_body_count,
            );
            // Per-type breakdown (zero-JNI for Java to inspect)
            arena.flush_force_breakdown(&force_report);
            arena.flush_events_from_handler(&world.inner.events);
        }
    })
}

/// Set integration parameters (dt, solver iterations, CCD substeps).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn world_set_integration_parameters(
    world: *mut WorldHandle,
    dt: f64,
    solver_iterations: u32,
    ccd_substeps: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return crate::rapier::ffi::Bool::FALSE;
        };
        if !dt.is_finite()
            || dt <= 0.0
            || dt > MAX_STEP_SECONDS
            || solver_iterations == 0
            || solver_iterations > 255
            || ccd_substeps > 255
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid integration parameters");
            return crate::rapier::ffi::Bool::FALSE;
        }

        world.inner.integration_parameters.dt = dt;
        world.inner.integration_parameters.num_solver_iterations = solver_iterations as usize;
        world.inner.integration_parameters.max_ccd_substeps = ccd_substeps as usize;
        clear_error();
        crate::rapier::ffi::Bool::TRUE
    })
}

/// Read integration parameters into `out_values` (dt, iterations, CCD substeps).
///
/// # Safety
/// `world` must be a valid world pointer (or null); `out_values` must point to
/// writable memory for at least `capacity` f64 values.
#[unsafe(no_mangle)]
pub extern "C" fn world_get_integration_parameters(
    world: *const WorldHandle,
    out_values: *mut f64,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if out_values.is_null() {
            set_error(ERR_NULL_POINTER, "integration parameter output is null");
            return 0;
        }
        if capacity < 3 {
            set_error(
                ERR_CAPACITY,
                "integration parameter output capacity must be at least 3",
            );
            return 0;
        }

        let out = unsafe { std::slice::from_raw_parts_mut(out_values, capacity as usize) };
        out[0] = world.inner.integration_parameters.dt;
        out[1] = world.inner.integration_parameters.num_solver_iterations as f64;
        out[2] = world.inner.integration_parameters.max_ccd_substeps as f64;
        clear_error();
        3
    })
}

/// Set the world gravity vector.  Non-finite input is ignored.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn world_set_gravity(world: *mut WorldHandle, gravity: Vec3) {
    ffi_guard((), || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            return;
        };
        if !vec3_finite(gravity) {
            return;
        }

        world.inner.gravity = vec3_to_rapier(gravity);
    })
}

/// Get the world gravity vector.
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn world_get_gravity(world: *const WorldHandle) -> Vec3 {
    ffi_guard(Vec3::default(), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            return Vec3::default();
        };

        crate::rapier::ffi::vec3_from_rapier(world.inner.gravity)
    })
}

/// Number of rigid bodies in the world (-1 on null world).
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn world_get_rigid_body_set_size(world: *const WorldHandle) -> i32 {
    ffi_guard(-1, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            return -1;
        };

        world.inner.bodies.len() as i32
    })
}

/// Number of colliders in the world (-1 on null world).
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn world_get_collider_set_size(world: *const WorldHandle) -> i32 {
    ffi_guard(-1, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            return -1;
        };

        world.inner.colliders.len() as i32
    })
}

/// Write the world gravity into `out_gravity`.
///
/// # Safety
/// `out_gravity` must point to a writable `Vec3` (or be null); `world` must be
/// a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn world_get_gravity_out(world: *const WorldHandle, out_gravity: *mut Vec3) {
    ffi_guard((), || {
        let Some(out_gravity) = (unsafe { out_gravity.as_mut() }) else {
            return;
        };

        *out_gravity = world_get_gravity(world);
    })
}

/// Count of dynamic bodies (for sizing a `world_dynamic_body_snapshot` call).
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn world_dynamic_body_snapshot_count(world: *const WorldHandle) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            return 0;
        };

        world
            .inner
            .bodies
            .iter()
            .filter(|(_, body)| body.is_dynamic())
            .count() as u32
    })
}

/// Snapshot dynamic body handles + poses (7 f64 per body: pos3 + quat4).
///
/// # Safety
/// `world` must be a valid world pointer (or null); `out_handles` must point to
/// writable memory for `capacity` handles and `out_values` for `capacity * 7`
/// f64 values.
#[unsafe(no_mangle)]
pub extern "C" fn world_dynamic_body_snapshot(
    world: *const WorldHandle,
    out_handles: *mut RigidBodyHandleRaw,
    out_values: *mut f64,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            return 0;
        };
        if out_handles.is_null()
            || out_values.is_null()
            || capacity == 0
            || capacity > MAX_OUTPUT_CAPACITY
        {
            return 0;
        }

        let capacity = capacity as usize;
        let Some(value_capacity) = capacity.checked_mul(7) else {
            return 0;
        };
        let handles_out = unsafe { std::slice::from_raw_parts_mut(out_handles, capacity) };
        let values = unsafe { std::slice::from_raw_parts_mut(out_values, value_capacity) };

        // Two-phase snapshot (see the `parallel` module docs): collect the
        // dynamic handles, compute each body's 7-lane pose in parallel above
        // `PAR_MIN_ITEMS` handles (read-only), then replay into the caller's
        // buffer in handle order — same output as the serial loop.
        let handles: Vec<RigidBodyHandle> = world
            .inner
            .bodies
            .iter()
            .filter(|(_, body)| body.is_dynamic())
            .take(capacity)
            .map(|(handle, _)| handle)
            .collect();
        let snaps =
            crate::rapier::parallel::body_pose_snapshots(&handles, &world.inner.bodies, false);

        let written = snaps.len();
        for (i, snap) in snaps.iter().enumerate() {
            handles_out[i] = pack_rigid_body_handle(handles[i]);
            values[i * 7..i * 7 + 7].copy_from_slice(&snap[..7]);
        }

        written as u32
    })
}

/// Count of all bodies (for sizing a `world_body_snapshot` call).
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn world_body_snapshot_count(world: *const WorldHandle) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            return 0;
        };

        world.inner.bodies.len().min(u32::MAX as usize) as u32
    })
}

/// Snapshot all body handles + poses + velocities (13 f64 per body:
/// pos3 + quat4 + linvel3 + angvel3).
///
/// # Safety
/// `world` must be a valid world pointer (or null); `out_handles` must point to
/// writable memory for `capacity` handles and `out_values` for `capacity * 13`
/// f64 values.
#[unsafe(no_mangle)]
pub extern "C" fn world_body_snapshot(
    world: *const WorldHandle,
    out_handles: *mut RigidBodyHandleRaw,
    out_values: *mut f64,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if out_handles.is_null()
            || out_values.is_null()
            || capacity == 0
            || capacity > MAX_OUTPUT_CAPACITY
        {
            set_error(ERR_CAPACITY, "invalid body snapshot output");
            return 0;
        }

        let capacity = capacity as usize;
        let Some(value_capacity) = capacity.checked_mul(13) else {
            set_error(ERR_CAPACITY, "body snapshot output capacity overflow");
            return 0;
        };
        let handles_out = unsafe { std::slice::from_raw_parts_mut(out_handles, capacity) };
        let values = unsafe { std::slice::from_raw_parts_mut(out_values, value_capacity) };

        // Two-phase snapshot (see the `parallel` module docs): parallel
        // per-body 13-lane computation above `PAR_MIN_ITEMS` handles, then a
        // serial replay into the caller's buffer in handle order.
        let handles: Vec<RigidBodyHandle> = world
            .inner
            .bodies
            .iter()
            .take(capacity)
            .map(|(handle, _)| handle)
            .collect();
        let snaps =
            crate::rapier::parallel::body_pose_snapshots(&handles, &world.inner.bodies, true);

        let written = snaps.len();
        for (i, snap) in snaps.iter().enumerate() {
            handles_out[i] = pack_rigid_body_handle(handles[i]);
            values[i * 13..i * 13 + 13].copy_from_slice(snap);
        }

        clear_error();
        written as u32
    })
}

/// Batch-update body poses (7 f64 per body: pos3 + quat4).  Returns the number
/// of bodies actually updated.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `handles` and
/// `values` must point to readable arrays of `count` handles and `count * 7`
/// f64 values respectively.
#[unsafe(no_mangle)]
pub extern "C" fn world_update_body_poses(
    world: *mut WorldHandle,
    handles: *const RigidBodyHandleRaw,
    values: *const f64,
    count: u32,
    wake_up: crate::rapier::ffi::Bool,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if handles.is_null() || values.is_null() || count == 0 || count > MAX_OUTPUT_CAPACITY {
            set_error(ERR_CAPACITY, "invalid body pose input");
            return 0;
        }

        let count = count as usize;
        let Some(value_count) = count.checked_mul(7) else {
            set_error(ERR_CAPACITY, "body pose input capacity overflow");
            return 0;
        };
        let handles = unsafe { std::slice::from_raw_parts(handles, count) };
        let values = unsafe { std::slice::from_raw_parts(values, value_count) };
        let mut updated = 0u32;

        for (index, handle) in handles.iter().enumerate() {
            let offset = index * 7;
            let translation = Vec3 {
                x: values[offset],
                y: values[offset + 1],
                z: values[offset + 2],
            };
            let rotation = Quat {
                i: values[offset + 3],
                j: values[offset + 4],
                k: values[offset + 5],
                w: values[offset + 6],
            };
            if !vec3_finite(translation) || !quat_finite(rotation) {
                continue;
            }
            if let Some(body) = world
                .inner
                .bodies
                .get_mut(unpack_rigid_body_handle(*handle))
            {
                body.set_position(isometry_from_parts(translation, rotation), wake_up.0 != 0);
                updated += 1;
            }
        }

        if updated == 0 {
            set_error(ERR_NOT_FOUND, "no body poses were updated");
        } else {
            clear_error();
        }
        updated
    })
}

/// Batch-update body velocities (6 f64 per body: linvel3 + angvel3).  Returns
/// the number of bodies actually updated.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `handles` and
/// `values` must point to readable arrays of `count` handles and `count * 6`
/// f64 values respectively.
#[unsafe(no_mangle)]
pub extern "C" fn world_update_body_velocities(
    world: *mut WorldHandle,
    handles: *const RigidBodyHandleRaw,
    values: *const f64,
    count: u32,
    wake_up: crate::rapier::ffi::Bool,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if handles.is_null() || values.is_null() || count == 0 || count > MAX_OUTPUT_CAPACITY {
            set_error(ERR_CAPACITY, "invalid body velocity input");
            return 0;
        }

        let count = count as usize;
        let Some(value_count) = count.checked_mul(6) else {
            set_error(ERR_CAPACITY, "body velocity input capacity overflow");
            return 0;
        };
        let handles = unsafe { std::slice::from_raw_parts(handles, count) };
        let values = unsafe { std::slice::from_raw_parts(values, value_count) };
        let mut updated = 0u32;

        for (index, handle) in handles.iter().enumerate() {
            let offset = index * 6;
            let linvel = Vec3 {
                x: values[offset],
                y: values[offset + 1],
                z: values[offset + 2],
            };
            let angvel = Vec3 {
                x: values[offset + 3],
                y: values[offset + 4],
                z: values[offset + 5],
            };
            if !vec3_finite(linvel) || !vec3_finite(angvel) {
                continue;
            }
            if let Some(body) = world
                .inner
                .bodies
                .get_mut(unpack_rigid_body_handle(*handle))
            {
                body.set_linvel(vec3_to_rapier(linvel), wake_up.0 != 0);
                body.set_angvel(vec3_to_rapier(angvel), wake_up.0 != 0);
                updated += 1;
            }
        }

        if updated == 0 {
            set_error(ERR_NOT_FOUND, "no body velocities were updated");
        } else {
            clear_error();
        }
        updated
    })
}

// ---------------------------------------------------------------------------
// ForceRegistry FFI — generic access for advanced callers
// ---------------------------------------------------------------------------

/// Opaque handle for a force law registered in the world's ForceRegistry.
/// Maps to `ForceLawHandle` in Rust.
pub type ForceLawHandleRaw = u64;

/// Number of force laws registered in the world's ForceRegistry.
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn world_get_force_registry_count(world: *const WorldHandle) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            return 0;
        };
        world.inner.force_registry.len() as u32
    })
}

/// Get count of registered force laws of a specific type.
/// `law_type` is the numeric discriminant of `ForceLawType`.
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn world_get_force_registry_typed_count(
    world: *const WorldHandle,
    law_type: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            return 0;
        };
        let law_type = match force_law_type_from_u32(law_type) {
            Some(lt) => lt,
            None => return 0,
        };
        world.inner.force_registry.find_by_type(law_type).len() as u32
    })
}

// ---------------------------------------------------------------------------
// Tests

// ---------------------------------------------------------------------------
// Shared Arena FFI — zero-JNI physics data access
// ---------------------------------------------------------------------------

/// Create a shared-memory physics arena.
///
/// Returns the arena pointer as a u64 (suitable for `MemorySegment.ofAddress` in Java).
/// The arena persists for the lifetime of the world.
///
/// At most one arena may exist per world. Calling this again while an arena
/// is still live fails with `ERR_INVALID_ARGUMENT` and leaves the existing
/// arena untouched — call `world_destroy_shared_arena` first to recreate one.
///
/// WARNING (Java side): before calling `world_destroy_shared_arena`, the
/// `MemorySegment` mapping the arena must be released/unmapped; destroying
/// the arena frees the underlying memory, and any still-mapped Java segment
/// would become a use-after-free.
///
/// `max_bodies` — max concurrent bodies to mirror
/// `max_events` — max pending collision/contact events
/// `max_commands` — max pending commands (force/set pose etc.)
/// `out_address` — receives the arena base address
/// `out_size` — receives the total arena size in bytes (for Java MemorySegment mapping)
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `out_address`
/// and `out_size` may be null, otherwise each must point to a writable u64.
#[unsafe(no_mangle)]
pub extern "C" fn world_create_shared_arena(
    world: *mut WorldHandle,
    max_bodies: u32,
    max_colliders: u32,
    max_events: u32,
    max_commands: u32,
    out_address: *mut u64,
    out_size: *mut u64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if world.inner.shared_arena.is_some() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "shared arena already exists; destroy it before recreating",
            );
            return Bool::FALSE;
        }
        if max_bodies == 0 || max_colliders == 0 || max_events == 0 || max_commands == 0 {
            set_error(ERR_INVALID_ARGUMENT, "arena capacities must be >0");
            return Bool::FALSE;
        }

        let Some(arena) = crate::rapier::shared_arena::SharedPhysicsArena::new(
            max_bodies,
            max_colliders,
            max_events,
            max_commands,
        ) else {
            set_error(ERR_CAPACITY, "arena capacities exceed limits");
            return Bool::FALSE;
        };
        let addr = arena.address();
        let sz = arena.size() as u64;

        world.inner.shared_arena = Some(Box::new(arena));

        if let Some(p) = unsafe { out_address.as_mut() } {
            *p = addr;
        }
        if let Some(p) = unsafe { out_size.as_mut() } {
            *p = sz;
        }
        clear_error();
        Bool::TRUE
    })
}

/// Destroy the shared arena (if any).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create` (or null).  Any
/// Java `MemorySegment` mapping the arena must be released before this call.
#[unsafe(no_mangle)]
pub extern "C" fn world_destroy_shared_arena(world: *mut WorldHandle) {
    ffi_guard((), || {
        if let Some(world) = unsafe { world.as_mut() } {
            world.inner.shared_arena = None;
        }
    })
}

/// Get the arena address (returns 0 if no arena).
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn world_get_shared_arena_address(world: *const WorldHandle) -> u64 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            return 0;
        };
        world.inner.shared_arena.as_ref().map_or(0, |a| a.address())
    })
}

/// Get the arena size (returns 0 if no arena).
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn world_get_shared_arena_size(world: *const WorldHandle) -> u64 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            return 0;
        };
        world
            .inner
            .shared_arena
            .as_ref()
            .map_or(0, |a| a.size() as u64)
    })
}

/// Reset the event ring (Java calls this after draining events).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create` (or null) and not
/// yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn world_reset_shared_arena_events(world: *mut WorldHandle) {
    ffi_guard((), || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            return;
        };
        if let Some(ref arena) = world.inner.shared_arena {
            arena.reset_event_ring();
        }
    })
}

/// Enable or disable relative force for a rigid body.
/// When enabled, forces applied via `rigid_body_add_force_at_local_point`
/// will be applied at the local attachment point instead of world coordinates.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create` (or null).
#[cfg(feature = "relative-force")]
#[unsafe(no_mangle)]
pub extern "C" fn world_set_relative_force_enabled(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    enabled: Bool,
    local_point: Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(_) = world.inner.bodies.get(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(local_point) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite local point");
            return Bool::FALSE;
        }
        world
            .inner
            .relative_force
            .insert(handle, (enabled.0 != 0, local_point));
        clear_error();
        Bool::TRUE
    })
}

/// Check if relative force is enabled for a rigid body.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create` (or null).
#[cfg(feature = "relative-force")]
#[unsafe(no_mangle)]
pub extern "C" fn world_get_relative_force_enabled(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(_) = world.inner.bodies.get(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        let enabled = world
            .inner
            .relative_force
            .get(&handle)
            .map(|v| v.0)
            .unwrap_or(false);
        clear_error();
        Bool(enabled as u8)
    })
}

/// Get the local attachment point for relative force.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create` (or null).
#[cfg(feature = "relative-force")]
#[unsafe(no_mangle)]
pub extern "C" fn world_get_relative_force_local_point(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Vec3 {
    ffi_guard(Vec3::default(), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Vec3::default();
        };
        let Some(_) = world.inner.bodies.get(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Vec3::default();
        };
        let local_point = world
            .inner
            .relative_force
            .get(&handle)
            .map(|v| v.1)
            .unwrap_or(Vec3::default());
        clear_error();
        local_point
    })
}

/// Set the local attachment point for relative force.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create` (or null).
#[cfg(feature = "relative-force")]
#[unsafe(no_mangle)]
pub extern "C" fn world_set_relative_force_local_point(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    local_point: Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(_) = world.inner.bodies.get(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(local_point) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite local point");
            return Bool::FALSE;
        }
        // Insert-or-update: keep existing enabled state, only replace point.
        world
            .inner
            .relative_force
            .entry(handle)
            .and_modify(|(_, point)| *point = local_point)
            .or_insert((false, local_point));
        clear_error();
        Bool::TRUE
    })
}

/// Remove relative force configuration for a rigid body.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create` (or null).
#[cfg(feature = "relative-force")]
#[unsafe(no_mangle)]
pub extern "C" fn world_remove_relative_force(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let removed = world.inner.relative_force.remove(&handle).is_some();
        if !removed {
            set_error(ERR_NOT_FOUND, "relative force not configured for this body");
            return Bool::FALSE;
        }
        clear_error();
        Bool::TRUE
    })
}

/// Enable or disable collision detection between two specific colliders, regardless
/// of their collision groups, solver hooks, or whether they are connected by a joint.
///
/// This surfaces the per-pair collision filtering exposed by Rapier's `World`
/// (`set_collision_enabled`). Unlike collision groups, the two colliders need not
/// belong to the same body or be jointed; any pair can be disabled. Disabling a
/// pair that was previously disabled (or enabling a pair that was never disabled)
/// is a no-op. The setting persists across `world_step` calls: a disabled pair's
/// existing contact manifolds are cleared on the next step.
///
/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` (or null). `collider1`
/// and `collider2` must be valid `ColliderHandleRaw` values returned at insert time.
#[unsafe(no_mangle)]
pub extern "C" fn world_set_collision_enabled(
    world: *mut WorldHandle,
    collider1: ColliderHandleRaw,
    collider2: ColliderHandleRaw,
    enabled: Bool,
) {
    ffi_guard((), || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return;
        };
        let c1 = unpack_collider_handle(collider1);
        let c2 = unpack_collider_handle(collider2);
        world.inner.set_collision_enabled(c1, c2, enabled.0 != 0);
    })
}
