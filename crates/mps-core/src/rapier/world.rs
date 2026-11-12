use super::body_spatial_index::BodySpatialIndex;
use crate::rapier::soft_data::{SoftBodyId, SoftBodySet};
use rapier3d::prelude::fluid::FluidWorld;
use rapier3d::prelude::granular::GranularWorld;
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

#[derive(Clone, Copy, PartialEq)]
pub(crate) struct WorldRegion {
    pub center: Vector,
    pub radius2: f64,
    pub active: bool,
    pub step_interval: u32,
    pub priority: i32,
}

fn apply_region_state(world: &mut PhysicsWorld, region: WorldRegion, dt: f64) -> u32 {
    let handles = world.region_handles(region.center, region.radius2.sqrt(), dt);
    let mut changed = 0;
    let mut selected: std::collections::HashSet<_> = handles.iter().copied().collect();
    if region.active {
        for pair in world.narrow_phase.contact_pairs() {
            let a = world.colliders.get(pair.collider1).and_then(|c| c.parent());
            let b = world.colliders.get(pair.collider2).and_then(|c| c.parent());
            if let (Some(a), Some(b)) = (a, b) {
                if selected.contains(&a) {
                    selected.insert(b);
                }
                if selected.contains(&b) {
                    selected.insert(a);
                }
            }
        }
    }
    for handle in selected {
        if let Some(body) = world.bodies.get_mut(handle) {
            if region.active && body.is_sleeping() {
                body.wake_up(true);
                changed += 1;
            } else if !region.active && !body.is_sleeping() {
                body.sleep();
                changed += 1;
            }
        }
    }
    changed
}

fn apply_region_states(world: &mut PhysicsWorld, regions: &[WorldRegion], dt: f64) {
    let mut desired = std::collections::HashMap::new();
    for region in regions {
        for handle in world.region_handles(region.center, region.radius2.sqrt(), dt) {
            let entry = desired.entry(handle).or_insert((i32::MIN, region.active));
            if region.priority >= entry.0 {
                *entry = (region.priority, region.active);
            }
        }
    }
    for (handle, (_, active)) in desired {
        if let Some(body) = world.bodies.get_mut(handle) {
            if active && body.is_sleeping() {
                body.wake_up(true);
            } else if !active && !body.is_sleeping() {
                body.sleep();
            }
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct WorldRuntimeSettings {
    configured: bool,
    pub solver_iterations: usize,
    pub ccd_substeps: usize,
    pub collision_events: bool,
    pub contact_force_events: bool,
    pub enable_ccd: bool,
    pub enable_sleeping: bool,
}

impl Default for WorldRuntimeSettings {
    fn default() -> Self {
        Self {
            configured: false,
            solver_iterations: 4,
            ccd_substeps: 4,
            collision_events: false,
            contact_force_events: false,
            enable_ccd: false,
            enable_sleeping: true,
        }
    }
}

impl WorldRuntimeSettings {
    pub(crate) fn apply_body(&self, body: &mut rapier3d::dynamics::RigidBody) {
        if !self.configured {
            return;
        }
        body.enable_ccd(self.enable_ccd);
        if !self.enable_sleeping {
            body.activation_mut().normalized_linear_threshold = -1.0;
            body.activation_mut().angular_threshold = -1.0;
        }
    }
    pub(crate) fn apply_collider(&self, collider: &mut rapier3d::geometry::Collider) {
        if !self.configured {
            return;
        }
        let mut flags = collider.active_events();
        flags.set(
            rapier3d::prelude::ActiveEvents::COLLISION_EVENTS,
            self.collision_events,
        );
        flags.set(
            rapier3d::prelude::ActiveEvents::CONTACT_FORCE_EVENTS,
            self.contact_force_events,
        );
        collider.set_active_events(flags);
    }
}

/// Applies settings to the current world's bodies and colliders, preserving
/// unrelated event flags and custom sleeping thresholds. New objects retain
/// their builder settings; call again after inserting a batch to apply globally.
/// This explicit operation never runs an O(n) configuration pass inside step.
/// Binary arguments accept only 0/1. Disabling events does not erase queued events.
/// Disabling CCD sets effective CCD substeps to zero, including this fork's
/// automatic sweeps against fixed colliders. Re-enable with the desired substeps.
/// # Safety
/// The world must be live and exclusively accessible, including relative to step.
#[unsafe(no_mangle)]
pub extern "C" fn world_apply_runtime_settings(
    world: *mut WorldHandle,
    solver_iterations: u32,
    ccd_substeps: u32,
    collision_events: u32,
    contact_force_events: u32,
    enable_ccd: u32,
    enable_sleeping: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        if solver_iterations == 0
            || solver_iterations > 255
            || ccd_substeps > 255
            || [
                collision_events,
                contact_force_events,
                enable_ccd,
                enable_sleeping,
            ]
            .iter()
            .any(|v| *v > 1)
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid runtime settings");
            return Bool::FALSE;
        }
        let Some(w) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        w.inner.runtime_settings = WorldRuntimeSettings {
            configured: true,
            solver_iterations: solver_iterations as usize,
            ccd_substeps: ccd_substeps as usize,
            collision_events: collision_events != 0,
            contact_force_events: contact_force_events != 0,
            enable_ccd: enable_ccd != 0,
            enable_sleeping: enable_sleeping != 0,
        };
        w.inner.integration_parameters.num_solver_iterations =
            w.inner.runtime_settings.solver_iterations;
        w.inner.integration_parameters.max_ccd_substeps = if enable_ccd != 0 {
            w.inner.runtime_settings.ccd_substeps
        } else {
            0
        };
        for (_, body) in w.inner.bodies.iter_mut() {
            body.enable_ccd(enable_ccd != 0);
            let activation = body.activation_mut();
            if enable_sleeping == 0 {
                activation.normalized_linear_threshold = -1.0;
                activation.angular_threshold = -1.0;
                body.wake_up(true);
            } else {
                if activation.normalized_linear_threshold < 0.0 {
                    activation.normalized_linear_threshold =
                        rapier3d::prelude::RigidBodyActivation::default_normalized_linear_threshold(
                        );
                }
                if activation.angular_threshold < 0.0 {
                    activation.angular_threshold =
                        rapier3d::prelude::RigidBodyActivation::default_angular_threshold();
                }
            }
        }
        for (_, collider) in w.inner.colliders.iter_mut() {
            let mut flags = collider.active_events();
            flags.set(
                rapier3d::prelude::ActiveEvents::COLLISION_EVENTS,
                collision_events != 0,
            );
            flags.set(
                rapier3d::prelude::ActiveEvents::CONTACT_FORCE_EVENTS,
                contact_force_events != 0,
            );
            collider.set_active_events(flags);
        }
        clear_error();
        Bool::TRUE
    })
}

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
    body_spatial_index: parking_lot::Mutex<Option<BodySpatialIndex>>,
    pub(crate) regions: Vec<WorldRegion>,
    /// B1 体级模拟域：domain_id → 成员刚体集合。域被 `world_set_domain_active`
    /// 置为 inactive 时，全体成员 `set_enabled(false)`（从 broad phase/island
    /// 撤出、状态冻结；比 region 的 sleep 更强——disabled 体不参与任何求解，
    /// 且休眠体内能不被扰动）。重新 active 时 `set_enabled(true)` 恢复。
    pub(crate) simulation_domains:
        std::collections::HashMap<u32, std::collections::HashSet<RigidBodyHandle>>,
    /// 当前被停用的域（inactive 域的 id 集合）。
    pub(crate) inactive_domains: std::collections::HashSet<u32>,
    pub(crate) region_tick: u64,
    pub(crate) region_version: u64,
    pub(crate) region_order_version: u64,
    pub(crate) region_order: Vec<usize>,
    pub(crate) runtime_settings: WorldRuntimeSettings,
    pub(crate) default_collision_mode: super::collision_mode::WorldCollisionMode,
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
    /// Upstream (rapier v0.36) soft-body set. This project's soft bodies live in
    /// `soft_bodies` (the fork's own `rapier3d::dynamics::soft_body_fork`
    /// system); upstream's `PhysicsPipeline::step`, `RigidBodySet::remove` and
    /// `ColliderSet::remove` still take a `SoftBodySet`, so this one stays empty
    /// and exists only to satisfy those signatures.
    pub(crate) upstream_soft_bodies: rapier3d::dynamics::SoftBodySet,
    /// Soft-body solver backend: the fork's own integrator (`Fork`, default) or
    /// rapier v0.36's official `SoftBody` (`Upstream`), which integrates soft
    /// bodies inside the pipeline with their deformable colliders. Selected with
    /// `soft_body_set_backend`; the C ABI is identical either way.
    pub(crate) soft_backend: crate::rapier::soft_upstream::SoftBodyBackend,
    /// Upstream-backend mirrors, keyed by the `SoftBodyId` slot index.
    pub(crate) soft_upstream_slots:
        std::collections::HashMap<u32, crate::rapier::soft_upstream::UpstreamSlot>,
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
    /// Debug recording (Jolt-Viewer-style capture): `Some` while frames are
    /// retained; `debug_recording_*` FFI manages the lifecycle. Absent when
    /// not recording so the hot path pays nothing.
    pub(crate) debug_recording: Option<crate::rapier::debug_recording::RecordingBuffer>,
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
    /// Natural-disaster sources (typhoon / tornado / hailstorm) driving the
    /// `disaster_*` FFI. Keyed by a stable id assigned at creation.
    pub(crate) disasters: IdRegistry<crate::rapier::disasters::Disaster>,
    /// Volcanic-eruption sources driving the `volcano_*` FFI. Keyed by a
    /// stable id assigned at creation.
    pub(crate) volcanoes: IdRegistry<crate::rapier::volcano::Volcano>,
    /// Per-body thermal / melt state for the volcano module, keyed by the
    /// `RigidBodyHandleRaw`. Entries for removed bodies are pruned lazily.
    pub(crate) volcano_bodies:
        std::collections::HashMap<RigidBodyHandleRaw, crate::rapier::volcano::VolcanoBody>,
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
    fn region_handles(&self, center: Vector, radius: f64, dt: f64) -> Vec<RigidBodyHandle> {
        let mut cache = self.body_spatial_index.lock();
        let index =
            cache.get_or_insert_with(|| BodySpatialIndex::new(&self.bodies, &self.colliders));
        index.sync(&self.bodies, dt);
        index.query(center, radius, dt)
    }

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
            body_spatial_index: parking_lot::Mutex::new(None),
            regions: Vec::new(),
            simulation_domains: std::collections::HashMap::new(),
            inactive_domains: std::collections::HashSet::new(),
            region_tick: 0,
            region_version: 0,
            region_order_version: u64::MAX,
            region_order: Vec::new(),
            runtime_settings: WorldRuntimeSettings::default(),
            default_collision_mode: super::collision_mode::WorldCollisionMode::Simple,
            pipeline: PhysicsPipeline::new(),
            gravity: vec3_to_rapier(gravity),
            integration_parameters,
            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: {
                let mut bodies = RigidBodySet::new();
                bodies.enable_spatial_change_tracking();
                bodies
            },
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            soft_bodies: SoftBodySet::new(),
            upstream_soft_bodies: rapier3d::dynamics::SoftBodySet::new(),
            soft_backend: crate::rapier::soft_upstream::SoftBodyBackend::Upstream,
            soft_upstream_slots: std::collections::HashMap::new(),
            fluids: Vec::new(),
            granular_bodies: Vec::new(),
            granular_proxies: std::collections::HashMap::new(),
            articulations: IdRegistry::new(),
            granular_dig_spawn: None,
            debug_recording: None,
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
            disasters: IdRegistry::new(),
            volcanoes: IdRegistry::new(),
            volcano_bodies: std::collections::HashMap::new(),
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
            magic: crate::rapier::ffi::types::WORLD_HANDLE_MAGIC,
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
            // Zero the magic first so any stale handle is detectable afterwards.
            (*world).magic = 0;
            drop(Box::from_raw(world));
        }
    })
}

/// Advance the simulation by `delta_seconds`. Valid range is `(0, 1]`
/// (`MAX_STEP_SECONDS`); anything else is rejected with `ERR_INVALID_ARGUMENT`
/// and the world is left untouched.
///
/// `world_step` returns `()` so a rejected call cannot be reported through a
/// sentinel. The error slot is the only channel — it is written here so a host
/// that steps with a NaN / non-positive / too-large `dt` can tell that nothing
/// happened instead of silently losing a frame.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn world_step(world: *mut WorldHandle, delta_seconds: f64) {
    ffi_guard((), || {
        #[cfg(feature = "profiler")]
        let profile = std::env::var_os("MPS_STEP_PROFILE").is_some();
        #[cfg(feature = "profiler")]
        let profile_start = std::time::Instant::now();
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return;
        };
        // Acquire the world write lock through a raw pointer so the guard does
        // not hold a borrow of `world.inner`; the step mutates every subsystem.
        let query_lock = &world.inner.query_lock as *const parking_lot::RwLock<()>;
        let _step_world_lock = unsafe { (&*query_lock).write() };
        if !delta_seconds.is_finite() || delta_seconds <= 0.0 || delta_seconds > MAX_STEP_SECONDS {
            set_error(
                ERR_INVALID_ARGUMENT,
                "world_step: delta_seconds must be finite and in (0, 1]",
            );
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
        world.inner.region_tick = world.inner.region_tick.wrapping_add(1);
        let tick = world.inner.region_tick;
        if world.inner.region_order_version != world.inner.region_version {
            world.inner.region_order = (0..world.inner.regions.len()).collect();
            world
                .inner
                .region_order
                .sort_by_key(|&i| world.inner.regions[i].priority);
            world.inner.region_order_version = world.inner.region_version;
        }
        let due: Vec<_> = world
            .inner
            .region_order
            .iter()
            .filter_map(|&i| world.inner.regions.get(i).copied())
            .filter(|r| tick % r.step_interval as u64 == 0)
            .collect();
        apply_region_states(&mut world.inner, &due, delta_seconds);

        // --- Arena: drain Java commands before applying forces ---
        // Java writes forces/set-poses/impulses via shared memory, Rust reads them here.
        if let Some(ref arena) = world.inner.shared_arena {
            let commands = arena.drain_commands();
            if !commands.is_empty() {
                // Rebuild the arena-index → handle map on every drain.
                //
                // This used to be incremental, keyed off `bodies.len()`: the
                // map was reused whenever the count was unchanged, on the
                // theory that "same count ⇒ same iteration order". That does
                // not hold. Rapier's body set is a slot arena, so a
                // remove-then-insert pair between two steps leaves the length
                // identical while shifting which handle sits at which arena
                // index. The stale map then pointed every command at a dead
                // handle, `bodies.get_mut` returned `None`, and the command
                // was dropped in silence — no error code, no log, just a
                // force that never arrived.
                //
                // Rebuilding is O(n) and only runs when the arena actually
                // delivered commands, so the steady-state cost is one push per
                // body per draining frame — negligible next to a physics step,
                // and correctness beats the micro-optimisation.
                let n_bodies = world.inner.bodies.len();
                {
                    let idx = &mut world.inner.buffers.arena_idx_map;
                    idx.clear();
                    idx.reserve(n_bodies);
                    for (h, _) in world.inner.bodies.iter() {
                        idx.push(Some(h));
                    }
                }
                world.inner.buffers.arena_idx_map_body_count = n_bodies;
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
                                // AddForceAtPoint — a0..a2 = force. NOTE: currently applies at the
                                // COM; the point argument is ignored. The 32-byte CommandSlot has
                                // no room for an application point — a protocol v3 would add it
                                // (Java side would need updating too). Behavior is intentionally
                                // unchanged until then.
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

        #[cfg(feature = "profiler")]
        let before_forces = profile_start.elapsed();
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
        #[cfg(feature = "profiler")]
        let after_forces = profile_start.elapsed();

        // 官方后端：把授权数据镜像成 rapier `SoftBody`，位姿/外力推进官方体；
        // 真正的求解在下面的 `pipeline.step` 里完成，步进后 `collect` 写回。
        crate::rapier::soft_upstream::prepare(
            &mut world.inner.soft_bodies,
            &mut world.inner.upstream_soft_bodies,
            &mut world.inner.soft_upstream_slots,
            &mut world.inner.islands,
            &mut world.inner.bodies,
            &mut world.inner.colliders,
            &mut world.inner.impulse_joints,
            &mut world.inner.multibody_joints,
        );

        // Phase 2 (fluid SPH ↔ rigid): sync each coupled fluid's particle poses into
        // its collision proxies *before* the rigid step, so the narrow-phase sees the
        // current particle positions/velocities and resolves contacts against terrain.
        for (fi, fluid) in world.inner.fluids.iter().enumerate() {
            #[allow(clippy::collapsible_if)]
            if let Some(proxies) = world.inner.fluid_proxies.get(&(fi as u32)) {
                for (pi, ph) in proxies.iter().enumerate() {
                    if let (Some(h), Some(p)) = (ph, fluid.particles.get(pi)) {
                        // `get_mut_internal`: this is an internal pose sync, not a user
                        // change — it must not push the proxy into `modified_bodies` every
                        // step (next-step user-changes handling + CCD fixed-target cache
                        // invalidation). Wake-up comes from `set_translation(_, true)`
                        // below, not from the accessor.
                        if let Some(rb) = world.inner.bodies.get_mut_internal(*h) {
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
                        // `get_mut_internal`: internal pose sync, not a user change — see
                        // the fluid block above for why this must not touch
                        // `modified_bodies`. Wake-up comes from `set_translation(_, true)`.
                        && let Some(rb) = world.inner.bodies.get_mut_internal(*h)
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
            &mut world.inner.upstream_soft_bodies,
            &mut world.inner.ccd_solver,
            &world.inner.hooks,
            &*world.inner.events,
        );
        #[cfg(feature = "profiler")]
        let after_pipeline = profile_start.elapsed();

        // 官方软体后端：求解在 `pipeline.step` 内完成，这里把解写回授权数据，
        // 让 `soft_body_*` 的读接口（粒子/能量/体积）拿到本步结果。
        crate::rapier::soft_upstream::collect(
            &mut world.inner.soft_bodies,
            &world.inner.upstream_soft_bodies,
            &world.inner.soft_upstream_slots,
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

        #[cfg(feature = "profiler")]
        if profile {
            eprintln!(
                "world_step profile: pre_force={before_forces:?}, forces={:?}, pipeline={:?}, post_pipeline={:?}, total={:?}",
                after_forces.saturating_sub(before_forces),
                after_pipeline.saturating_sub(after_forces),
                profile_start.elapsed().saturating_sub(after_pipeline),
                profile_start.elapsed(),
            );
        }

        // 绑定粒子的“跟随刚体”由官方软体自己的 attachment 机制完成：`collect` 已把
        // 官方解写回授权数据，这里不再需要 fork 的手动跟随。

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
        // Ordering asymmetry with granular (next block) is intentional: fluid reads
        // back AFTER `fluid.step` — SPH integrates first and collision response
        // overwrites its result; granular reads back BEFORE `granular.step` — the DEM
        // integrator runs on top of the collision response.
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
        // Ordering asymmetry with fluid (previous block) is intentional: granular
        // reads back BEFORE `granular.step` — the DEM integrator runs on top of the
        // collision response; fluid reads back AFTER `fluid.step` — SPH integrates
        // first and collision response overwrites its result.
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
        let spatial = world.inner.body_spatial_index.get_mut();
        if let Some(index) = spatial {
            index.sync(&world.inner.bodies, world.inner.integration_parameters.dt);
        }
        for (handle, body) in world.inner.bodies.iter_mut() {
            if body.is_dynamic() {
                body.reset_forces(false);
            }
            if let Some(index) = spatial {
                index.update(handle, body, world.inner.integration_parameters.dt);
            }
        }
        world.inner.bodies.clear_spatial_changes();
        if let Some(index) = spatial {
            index.journal_cleared();
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

/// Writes the latest Rapier stage timings in milliseconds:
/// update, broad phase, narrow phase, island construction, solver, CCD, total.
/// Returns the number of values written (7), or 0 on invalid arguments.
/// Timings are zero without the `profiler` feature. Event and hook callback
/// costs are included in their calling stages, not reported independently.
/// # Safety
/// `world` must be live and not concurrently stepped; `out_values` must point
/// to at least `capacity` writable `f64` values.
#[unsafe(no_mangle)]
pub extern "C" fn world_get_pipeline_timings(
    world: *const WorldHandle,
    out_values: *mut f64,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if capacity < 7 || out_values.is_null() {
            set_error(
                ERR_CAPACITY,
                "pipeline timing output capacity must be at least 7",
            );
            return 0;
        }
        let Some(out) = (unsafe {
            crate::rapier::ffi::convert::checked_output_slice(out_values, capacity as usize)
        }) else {
            set_error(ERR_INVALID_ARGUMENT, "invalid pipeline timing output");
            return 0;
        };
        let c = &world.inner.pipeline.counters;
        out[..7].copy_from_slice(&[
            c.update_time_ms(),
            c.broad_phase_time_ms(),
            c.narrow_phase_time_ms(),
            c.island_construction_time_ms(),
            c.solver_time_ms(),
            c.ccd_time_ms(),
            c.step_time_ms(),
        ]);
        clear_error();
        7
    })
}

/// Invalidates the cached body spatial bounds after an in-place collider shape
/// or local-pose change. The next region query rebuilds bounds lazily.
#[unsafe(no_mangle)]
pub extern "C" fn world_invalidate_region_index(world: *mut WorldHandle) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        *world.inner.body_spatial_index.lock() = None;
        clear_error();
        Bool::TRUE
    })
}

pub(crate) fn invalidate_region_index(world: &mut PhysicsWorld) {
    *world.body_spatial_index.lock() = None;
}

#[allow(dead_code)]
pub(crate) fn refresh_region_body(world: &mut PhysicsWorld, handle: ColliderHandle) {
    let Some(parent) = world.colliders.get(handle).and_then(|c| c.parent()) else {
        return;
    };
    let Some(body) = world.bodies.get(parent) else {
        return;
    };
    let mut cache = world.body_spatial_index.lock();
    if let Some(index) = cache.as_mut() {
        index.update_snapshot(
            parent,
            body,
            &world.colliders,
            world.integration_parameters.dt,
        );
    }
}

/// Enables or suspends dynamic bodies inside a spherical spatial region.
/// Suspended bodies are put to sleep and excluded from active islands until
/// they are re-enabled. Returns the number of affected bodies.
#[unsafe(no_mangle)]
pub extern "C" fn world_set_region_active(
    world: *mut WorldHandle,
    center: Vec3,
    radius: f64,
    active: Bool,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if !vec3_finite(center) || !radius.is_finite() || radius < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid region");
            return 0;
        }
        let r2 = radius * radius;
        let center = vec3_to_rapier(center);
        let before = world.inner.regions.clone();
        if let Some(region) = world
            .inner
            .regions
            .iter_mut()
            .find(|r| r.center == center && r.radius2 == r2)
        {
            region.active = active == Bool::TRUE;
        } else {
            world.inner.regions.push(WorldRegion {
                center,
                radius2: r2,
                active: active == Bool::TRUE,
                step_interval: 1,
                priority: 0,
            });
        }
        let query_dt = world.inner.integration_parameters.dt;
        if world.inner.regions != before {
            world.inner.region_version = world.inner.region_version.wrapping_add(1);
        }
        let changed = apply_region_state(
            &mut world.inner,
            WorldRegion {
                center,
                radius2: r2,
                active: active == Bool::TRUE,
                step_interval: 1,
                priority: 0,
            },
            query_dt,
        );
        clear_error();
        changed
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_region_step_interval(
    world: *mut WorldHandle,
    center: Vec3,
    radius: f64,
    interval: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let _query_lock = world.inner.query_lock.write();
        if !vec3_finite(center)
            || !radius.is_finite()
            || radius < 0.0
            || interval == 0
            || interval > 1024
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid region interval");
            return Bool::FALSE;
        }
        let c = vec3_to_rapier(center);
        let r2 = radius * radius;
        let before = world.inner.regions.clone();
        if let Some(region) = world
            .inner
            .regions
            .iter_mut()
            .find(|r| r.center == c && r.radius2 == r2)
        {
            region.step_interval = interval;
        } else {
            world.inner.regions.push(WorldRegion {
                center: c,
                radius2: r2,
                active: true,
                step_interval: interval,
                priority: 0,
            });
        }
        if world.inner.regions != before {
            world.inner.region_version = world.inner.region_version.wrapping_add(1);
        }
        clear_error();
        Bool::TRUE
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn world_wake_region(world: *mut WorldHandle, center: Vec3, radius: f64) -> u32 {
    world_set_region_active(world, center, radius, Bool::TRUE)
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_region_priority(
    world: *mut WorldHandle,
    center: Vec3,
    radius: f64,
    priority: i32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let _lock = world.inner.query_lock.write();
        if !vec3_finite(center) || !radius.is_finite() || radius < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid region");
            return Bool::FALSE;
        }
        let c = vec3_to_rapier(center);
        let r2 = radius * radius;
        let before = world.inner.regions.clone();
        if let Some(region) = world
            .inner
            .regions
            .iter_mut()
            .find(|r| r.center == c && r.radius2 == r2)
        {
            region.priority = priority;
        } else {
            world.inner.regions.push(WorldRegion {
                center: c,
                radius2: r2,
                active: true,
                step_interval: 1,
                priority,
            });
        }
        if world.inner.regions != before {
            world.inner.region_version = world.inner.region_version.wrapping_add(1);
        }
        clear_error();
        Bool::TRUE
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_region_body_count(
    world: *const WorldHandle,
    center: Vec3,
    radius: f64,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let _query_lock = world.inner.query_lock.read();
        if !vec3_finite(center) || !radius.is_finite() || radius < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid region");
            return 0;
        }
        let count = world
            .inner
            .region_handles(
                vec3_to_rapier(center),
                radius,
                world.inner.integration_parameters.dt,
            )
            .len();
        clear_error();
        count as u32
    })
}

// ---- B1 体级模拟域（per-body simulation domains）----
//
// 语义（ADR 见 docs/adr/）：炮弹/远处结构停在模拟边界外 → 宿主把它们划入
// 非活动域。inactive 域的成员被 `set_enabled(false)` 完全冻结（从 broad
// phase/island 撤出、位姿与速度原样保留），active 恢复时 `set_enabled(true)`。
// 比 region 的 sleep 更强的保证：disabled 体不参与任何求解阶段，也不能被
// 唤醒。与 region 的分工：region 是空间谓词（每步重算），domain 是显式
// 成员表（宿主控制成员资格，零每步查询开销）。

/// 把刚体划入模拟域（domain 0 = 无域/默认，调用即从当前域移除并恢复活动）。
/// 域成员资格在切换时立即生效：若目标域当前 inactive，刚体立即冻结。
///
/// # Safety
/// `world` 必须是 `world_create` 返回的有效指针。
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_simulation_domain(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    domain: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let _lock = world.inner.query_lock.write();
        let body = crate::rapier::ffi::convert::unpack_rigid_body_handle(handle);
        // 从旧域移除。
        for members in world.inner.simulation_domains.values_mut() {
            members.remove(&body);
        }
        if domain == 0 {
            // 无域：确保恢复活动。
            if let Some(b) = world.inner.bodies.get_mut(body) {
                b.set_enabled(true);
            }
            clear_error();
            return Bool::TRUE;
        }
        world
            .inner
            .simulation_domains
            .entry(domain)
            .or_default()
            .insert(body);
        // 目标域 inactive → 立即冻结。
        if world.inner.inactive_domains.contains(&domain)
            && let Some(b) = world.inner.bodies.get_mut(body)
        {
            b.set_enabled(false);
        }
        clear_error();
        Bool::TRUE
    })
}

/// 设置域的活动状态。返回状态实际翻转的刚体数。
///
/// # Safety
/// `world` 必须是 `world_create` 返回的有效指针。
#[unsafe(no_mangle)]
pub extern "C" fn world_set_domain_active(
    world: *mut WorldHandle,
    domain: u32,
    active: Bool,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let _lock = world.inner.query_lock.write();
        if domain == 0 {
            set_error(ERR_INVALID_ARGUMENT, "domain 0 is reserved (no domain)");
            return 0;
        }
        let Some(members) = world.inner.simulation_domains.get(&domain) else {
            set_error(ERR_NOT_FOUND, "unknown simulation domain");
            return 0;
        };
        let mut changed = 0u32;
        let activate = active == Bool::TRUE;
        for handle in members {
            if let Some(b) = world.inner.bodies.get_mut(*handle)
                && activate != b.is_enabled()
            {
                b.set_enabled(activate);
                changed += 1;
            }
        }
        if activate {
            world.inner.inactive_domains.remove(&domain);
        } else {
            world.inner.inactive_domains.insert(domain);
        }
        clear_error();
        changed
    })
}

/// 域成员数（含被冻结的成员）。未知域返回 -1。
///
/// # Safety
/// `world` 必须是 `world_create` 返回的有效指针。
#[unsafe(no_mangle)]
pub extern "C" fn world_domain_body_count(world: *const WorldHandle, domain: u32) -> i32 {
    ffi_guard(-1, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return -1;
        };
        let _lock = world.inner.query_lock.read();
        match world.inner.simulation_domains.get(&domain) {
            Some(members) => {
                clear_error();
                members.len() as i32
            }
            None => {
                set_error(ERR_NOT_FOUND, "unknown simulation domain");
                -1
            }
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
/// See also `world_apply_runtime_settings` for body/collider feature switches.
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
        let _query_lock = world.inner.query_lock.read();

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
        let _query_lock = world.inner.query_lock.read();

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
        let _query_lock = world.inner.query_lock.read();

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
        let _query_lock = world.inner.query_lock.read();
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
        let _query_lock = world.inner.query_lock.read();

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
        let _query_lock = world.inner.query_lock.read();
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
