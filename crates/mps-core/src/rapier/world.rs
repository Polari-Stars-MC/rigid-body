use rapier3d::prelude::{
    ActiveHooks, BroadPhaseBvh, CCDSolver, ColliderSet, ImpulseJointSet, IntegrationParameters,
    IslandManager, MultibodyJointSet, NarrowPhase, PhysicsPipeline, RigidBodySet, Vector,
};
use std::sync::Arc;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, MAX_OUTPUT_CAPACITY, Quat, RigidBodyHandleRaw, Vec3, WorldHandle, isometry_from_parts,
    pack_rigid_body_handle, quat_finite, quat_from_rapier, unpack_rigid_body_handle, vec3_finite,
    vec3_from_rapier, vec3_to_rapier,
};
use crate::rapier::forces::{BodyForceLog, ForceFacade, ForceRegistry};
use hashbrown::HashMap;

const MAX_STEP_SECONDS: f64 = 1.0;

/// Preallocated working storage reused each frame to avoid per-step heap allocations.
pub(crate) struct FrameWorkBuffers {
    /// Per-body force log: indexed by handle index for O(1) access without hashing.
    /// Index = RigidBodyHandle::into_raw_parts().0 (arena index portion).
    /// Auto-expands when new bodies are inserted beyond current capacity.
    pub(crate) body_log: Vec<Option<BodyForceLog>>,
    /// Scratch buffer for Coulomb friction pairs (avoid per-frame Vec::new()).
    pub(crate) friction_work: Vec<(rapier3d::prelude::RigidBodyHandle, rapier3d::prelude::RigidBodyHandle, Vector)>,
    /// Scratch buffer for legacy external force computation.
    pub(crate) pending_forces: smallvec::SmallVec<[crate::rapier::events::PendingForce; 128]>,
    /// Scratch buffer for arena command → handle mapping.
    pub(crate) arena_idx_map: Vec<Option<rapier3d::prelude::RigidBodyHandle>>,
}

impl Default for FrameWorkBuffers {
    fn default() -> Self {
        Self {
            body_log: Vec::with_capacity(256),
            friction_work: Vec::with_capacity(512),
            pending_forces: smallvec::SmallVec::new(),
            arena_idx_map: Vec::with_capacity(256),
        }
    }
}

impl FrameWorkBuffers {
    /// Clear all buffers for reuse in the next frame without deallocating.
    fn clear(&mut self) {
        // Clear individual log entries (keep capacity)
        for entry in self.body_log.iter_mut() {
            if let Some(log) = entry {
                log.forces.clear();
                log.torques.clear();
                *entry = None; // mark slot as reusable
            }
        }
        // Truncate to zero but keep capacity
        self.body_log.truncate(0);
        self.friction_work.clear();
        self.pending_forces.clear();
        self.arena_idx_map.clear();
    }

    /// Ensure body_log can hold at least `max_index` entries.
    /// Called whenever a new body is inserted with a higher handle index.
    fn ensure_body_log_capacity(&mut self, max_index: usize) {
        if max_index >= self.body_log.len() {
            self.body_log.resize_with(max_index + 1, || None);
        }
    }
}

pub(crate) struct PhysicsWorld {
    pub(crate) pipeline: PhysicsPipeline,
    pub(crate) gravity: Vector,
    pub(crate) integration_parameters: IntegrationParameters,
    pub(crate) islands: IslandManager,
    pub(crate) broad_phase: BroadPhaseBvh,
    pub(crate) narrow_phase: NarrowPhase,
    pub(crate) bodies: RigidBodySet,
    pub(crate) colliders: ColliderSet,
    pub(crate) impulse_joints: ImpulseJointSet,
    pub(crate) multibody_joints: MultibodyJointSet,
    pub(crate) ccd_solver: CCDSolver,
    pub(crate) hooks: crate::rapier::events::CallbackPhysicsHooks,
    pub(crate) events: Arc<crate::rapier::events::CollectingEventHandler>,
    pub(crate) force_registry: ForceRegistry,
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) shared_arena: Option<Box<crate::rapier::shared_arena::SharedPhysicsArena>>,
    /// Persistent per-frame work buffers — cleared and reused each `world_step`.
    pub(crate) buffers: FrameWorkBuffers,
}

impl PhysicsWorld {
    pub(crate) fn new(gravity: Vec3) -> Self {
        let integration_parameters = IntegrationParameters {
            dt: 1.0 / 60.0,
            num_solver_iterations: 4,
            max_ccd_substeps: 4,
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
            hooks: crate::rapier::events::CallbackPhysicsHooks::new(events.clone()),
            events,
            force_registry: ForceRegistry::new(),
            #[cfg(not(target_arch = "wasm32"))]
            shared_arena: None,
            buffers: FrameWorkBuffers::default(),
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn world_create(gravity: Vec3) -> *mut WorldHandle {
    let gravity = if vec3_finite(gravity) {
        gravity
    } else {
        Vec3::default()
    };

    Box::into_raw(Box::new(WorldHandle {
        inner: PhysicsWorld::new(gravity),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn world_destroy(world: *mut WorldHandle) {
    if world.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(world));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn world_step(world: *mut WorldHandle, delta_seconds: f64) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    if !delta_seconds.is_finite() || delta_seconds <= 0.0 || delta_seconds > MAX_STEP_SECONDS {
        return;
    }

    world.inner.integration_parameters.dt = delta_seconds;

    // --- Arena: drain Java commands before applying forces ---
    // Java writes forces/set-poses/impulses via shared memory, Rust reads them here.
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(ref arena) = world.inner.shared_arena {
        let commands = arena.drain_commands();
        if !commands.is_empty() {
            // Use persistent cached index map (P3 fix: avoid per-frame Vec rebuild)
            let idx = &mut world.inner.buffers.arena_idx_map;
            idx.clear();
            for (h, _) in world.inner.bodies.iter() {
                idx.push(Some(h));
            }
            for (cmd_type, body_idx, a0, a1, a2) in commands {
                if let Some(Some(h)) = idx.get(body_idx as usize) {
                    if let Some(body) = world.inner.bodies.get_mut(*h) {
                        match cmd_type {
                            0 => { // AddForce
                                body.add_force(
                                    rapier3d::prelude::Vector::new(a0, a1, a2), true);
                            }
                            1 => { // AddTorque
                                body.add_torque(
                                    rapier3d::prelude::Vector::new(a0, a1, a2), true);
                            }
                            2 => { // SetPose
                                // a0..a2 = position, rest packed into user_data via cmd encoding
                                let pos = rapier3d::prelude::Pose::from_parts(
                                    rapier3d::prelude::Vector::new(a0, a1, a2),
                                    *body.rotation(),
                                );
                                body.set_position(pos, true);
                            }
                            3 => { // SetVelocity
                                body.set_linvel(
                                    rapier3d::prelude::Vector::new(a0, a1, a2), true);
                            }
                            4 => { // ApplyImpulse
                                body.apply_impulse(
                                    rapier3d::prelude::Vector::new(a0, a1, a2), true);
                            }
                            5 => { // ApplyTorqueImpulse
                                body.apply_torque_impulse(
                                    rapier3d::prelude::Vector::new(a0, a1, a2), true);
                            }
                            6 => { // WakeUp
                                body.wake_up(true);
                            }
                            7 => { // Sleep
                                body.sleep();
                            }
                            8 => { // SetRotation — a0..a2 = rotation vector (axis-angle)
                                let axis_angle = rapier3d::prelude::Vector::new(a0, a1, a2);
                                let angle = axis_angle.length();
                                if angle > 1e-12 {
                                    let unit_axis = axis_angle / angle;
                                    body.set_rotation(
                                        rapier3d::prelude::Rotation::from_axis_angle(
                                            unit_axis, angle), true);
                                }
                            }
                            9 => { // SetGravityScale — a0 = scale
                                body.set_gravity_scale(a0, true);
                            }
                            10 => { // SetLinearDamping — a0 = damping
                                body.set_linear_damping(a0);
                            }
                            11 => { // SetAngularDamping — a0 = damping
                                body.set_angular_damping(a0);
                            }
                            12 => { // AddForceAtPoint — a0..a2 = force, need point from next cmd or use COM
                                body.add_force(
                                    rapier3d::prelude::Vector::new(a0, a1, a2), true);
                            }
                            _ => {}
                        }
                    }
                }
            }
        }
    }

    // --- Coulomb hook setup ---
    let custom = world.inner.events.custom_physics();
    let coulomb_active = custom
        .coulomb_friction
        .is_some_and(|law| law.enabled.0 != 0);

    if coulomb_active {
        let hook_bit = ActiveHooks::MODIFY_SOLVER_CONTACTS;
        for (_, collider) in world.inner.colliders.iter_mut() {
            let current = collider.active_hooks();
            if !current.contains(hook_bit) {
                collider.set_active_hooks(current | hook_bit);
            }
        }
    }

    // --- Force facade: the single entry-point for all force application ---
    // O1 fix: reuse persistent body_log (Vec-indexed by handle) instead of HashMap.
    // Take ownership of the buffers, use them, then put them back.
    let mut body_log = std::mem::take(&mut world.inner.buffers.body_log);
    let mut pending_forces = std::mem::take(&mut world.inner.buffers.pending_forces);
    let mut friction_work = std::mem::take(&mut world.inner.buffers.friction_work);
    let mut facade = ForceFacade::new(
        &mut world.inner.bodies,
        &mut world.inner.colliders,
        &world.inner.narrow_phase,
        &mut body_log,
        &mut pending_forces,
        &mut friction_work,
    );

    // 1. Registered ForceLaw list (from new system)
    world.inner.force_registry.apply_all(&mut facade);

    // 2. Backward-compat: old unregistered external-force law setter path
    //   Work around borrowck by copying body handles/positions, then replaying forces through facade.
    crate::rapier::events::apply_custom_external_forces_with_facade(
        &custom,
        &mut facade,
    );

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

    // 5. Flush shared arena body/collider state → Java zero-JNI read
    #[cfg(not(target_arch = "wasm32"))]
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
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_integration_parameters(
    world: *mut WorldHandle,
    dt: f64,
    solver_iterations: u32,
    ccd_substeps: u32,
) -> Bool {
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
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_integration_parameters(
    world: *const WorldHandle,
    out_values: *mut f64,
    capacity: u32,
) -> u32 {
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
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_gravity(world: *mut WorldHandle, gravity: Vec3) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    if !vec3_finite(gravity) {
        return;
    }

    world.inner.gravity = vec3_to_rapier(gravity);
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_gravity(world: *const WorldHandle) -> Vec3 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return Vec3::default();
    };

    crate::rapier::ffi::vec3_from_rapier(world.inner.gravity)
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_rigid_body_set_size(world: *const WorldHandle) -> i32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return -1;
    };

    world.inner.bodies.len() as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_collider_set_size(world: *const WorldHandle) -> i32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return -1;
    };

    world.inner.colliders.len() as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_gravity_out(world: *const WorldHandle, out_gravity: *mut Vec3) {
    let Some(out_gravity) = (unsafe { out_gravity.as_mut() }) else {
        return;
    };

    *out_gravity = world_get_gravity(world);
}

#[unsafe(no_mangle)]
pub extern "C" fn world_dynamic_body_snapshot_count(world: *const WorldHandle) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };

    world
        .inner
        .bodies
        .iter()
        .filter(|(_, body)| body.is_dynamic())
        .count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn world_dynamic_body_snapshot(
    world: *const WorldHandle,
    out_handles: *mut RigidBodyHandleRaw,
    out_values: *mut f64,
    capacity: u32,
) -> u32 {
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
    let handles = unsafe { std::slice::from_raw_parts_mut(out_handles, capacity) };
    let values = unsafe { std::slice::from_raw_parts_mut(out_values, value_capacity) };
    let mut written = 0usize;

    for (handle, body) in world.inner.bodies.iter() {
        if !body.is_dynamic() {
            continue;
        }
        if written >= capacity {
            break;
        }

        let translation = vec3_from_rapier(body.translation());
        let rotation = quat_from_rapier(*body.rotation());
        handles[written] = pack_rigid_body_handle(handle);
        let offset = written * 7;
        values[offset] = translation.x;
        values[offset + 1] = translation.y;
        values[offset + 2] = translation.z;
        values[offset + 3] = rotation.i;
        values[offset + 4] = rotation.j;
        values[offset + 5] = rotation.k;
        values[offset + 6] = rotation.w;
        written += 1;
    }

    written as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn world_body_snapshot_count(world: *const WorldHandle) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };

    world.inner.bodies.len().min(u32::MAX as usize) as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn world_body_snapshot(
    world: *const WorldHandle,
    out_handles: *mut RigidBodyHandleRaw,
    out_values: *mut f64,
    capacity: u32,
) -> u32 {
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
    let handles = unsafe { std::slice::from_raw_parts_mut(out_handles, capacity) };
    let values = unsafe { std::slice::from_raw_parts_mut(out_values, value_capacity) };
    let mut written = 0usize;

    for (handle, body) in world.inner.bodies.iter() {
        if written >= capacity {
            break;
        }

        let translation = vec3_from_rapier(body.translation());
        let rotation = quat_from_rapier(*body.rotation());
        let linvel = vec3_from_rapier(body.linvel());
        let angvel = vec3_from_rapier(body.angvel());
        handles[written] = pack_rigid_body_handle(handle);
        let offset = written * 13;
        values[offset] = translation.x;
        values[offset + 1] = translation.y;
        values[offset + 2] = translation.z;
        values[offset + 3] = rotation.i;
        values[offset + 4] = rotation.j;
        values[offset + 5] = rotation.k;
        values[offset + 6] = rotation.w;
        values[offset + 7] = linvel.x;
        values[offset + 8] = linvel.y;
        values[offset + 9] = linvel.z;
        values[offset + 10] = angvel.x;
        values[offset + 11] = angvel.y;
        values[offset + 12] = angvel.z;
        written += 1;
    }

    clear_error();
    written as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn world_update_body_poses(
    world: *mut WorldHandle,
    handles: *const RigidBodyHandleRaw,
    values: *const f64,
    count: u32,
    wake_up: crate::rapier::ffi::Bool,
) -> u32 {
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
}

#[unsafe(no_mangle)]
pub extern "C" fn world_update_body_velocities(
    world: *mut WorldHandle,
    handles: *const RigidBodyHandleRaw,
    values: *const f64,
    count: u32,
    wake_up: crate::rapier::ffi::Bool,
) -> u32 {
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
}

// ---------------------------------------------------------------------------
// Convenience: register celestial gravity as a ForceLaw
// ---------------------------------------------------------------------------

/// Register celestial body gravity as a ForceLaw in the world's registry.
///
/// `body_id` maps to `CelestialBodyId` (0=Sun, 3=Earth, 4=Moon, 5=Mars, etc.).
///
/// Returns handle (non-zero) on success, 0 on invalid body_id.
#[unsafe(no_mangle)]
pub extern "C" fn world_register_celestial_gravity(
    world: *mut WorldHandle,
    body_id: u32,
    max_degree: u32,
) -> u64 {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    };
    let id = match body_id {
        0..=9 => unsafe {
            std::mem::transmute::<u32, crate::rapier::celestial_data::CelestialBodyId>(body_id)
        },
        _ => {
            set_error(ERR_INVALID_ARGUMENT, "invalid celestial body ID");
            return 0;
        }
    };
    let body = crate::rapier::celestial_data::get_celestial_body(id);
    let law = crate::rapier::interaction::CelestialGravityForceLaw {
        body,
        max_sh_degree: max_degree.min(body.max_degree),
        enabled: true,
    };

    // P8: single traversal to find + unregister all existing celestial gravity laws
    world.inner.force_registry.unregister_by_type(
        crate::rapier::forces::ForceLawType::CelestialGravity,
    );

    clear_error();
    world.inner.force_registry.register(Box::new(law)).raw()
}

// ---------------------------------------------------------------------------
// ForceRegistry FFI — generic access for advanced callers
// ---------------------------------------------------------------------------

use crate::rapier::forces::ForceLawType;

/// Opaque handle for a force law registered in the world's ForceRegistry.
/// Maps to `ForceLawHandle` in Rust.
pub type ForceLawHandleRaw = u64;

#[unsafe(no_mangle)]
pub extern "C" fn world_get_force_registry_count(world: *const WorldHandle) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    world.inner.force_registry.len() as u32
}

/// Get count of registered force laws of a specific type.
/// `law_type` is the numeric discriminant of `ForceLawType`.
#[unsafe(no_mangle)]
pub extern "C" fn world_get_force_registry_typed_count(
    world: *const WorldHandle,
    law_type: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    let law_type = match force_law_type_from_u32(law_type) {
        Some(lt) => lt,
        None => return 0,
    };
    world.inner.force_registry.find_by_type(law_type).len() as u32
}

/// Convert a u32 tag to `ForceLawType`.  The mapping matches cbindgen's
/// C enum generation: 0 = WorldGravity, 1 = UserForce, ...
fn force_law_type_from_u32(tag: u32) -> Option<ForceLawType> {
    match tag {
        0 => Some(ForceLawType::WorldGravity),
        1 => Some(ForceLawType::UserForce),
        2 => Some(ForceLawType::NewtonianGravity),
        3 => Some(ForceLawType::CoulombFriction),
        4 => Some(ForceLawType::AirDrag),
        5 => Some(ForceLawType::Buoyancy),
        6 => Some(ForceLawType::Electromagnetic),
        7 => Some(ForceLawType::ElasticSpring),
        8 => Some(ForceLawType::PointGravity),
        9 => Some(ForceLawType::AerodynamicSurface),
        10 => Some(ForceLawType::AerodynamicVoxel),
        11 => Some(ForceLawType::FluidAABB),
        12 => Some(ForceLawType::MolecularLennardJones),
        13 => Some(ForceLawType::MolecularCoulomb),
        14 => Some(ForceLawType::SpaceJ2),
        15 => Some(ForceLawType::SpaceCMG),
        16 => Some(ForceLawType::SpaceAtmosphericDrag),
        17 => Some(ForceLawType::SpaceSolarRadiation),
        18 => Some(ForceLawType::SpaceGravityGradient),
        19 => Some(ForceLawType::SpaceMagneticTorquer),
        20 => Some(ForceLawType::TrajectoryCoriolis),
        21 => Some(ForceLawType::TrajectoryCentrifugal),
        22 => Some(ForceLawType::TrajectoryGravity),
        23 => Some(ForceLawType::ControlPID),
        24 => Some(ForceLawType::CelestialGravity),
        25 => Some(ForceLawType::TerrainGravity),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Tests

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::BodyStatus;

    #[test]
    fn integration_parameters_and_body_batch_updates_work() {
        let world = world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        assert!(!world.is_null());
        assert_eq!(
            world_set_integration_parameters(world, 1.0 / 120.0, 8, 2),
            Bool::TRUE
        );

        let mut params = [0.0; 3];
        assert_eq!(
            world_get_integration_parameters(world, params.as_mut_ptr(), params.len() as u32),
            3
        );
        assert_eq!(params[1], 8.0);
        assert_eq!(params[2], 2.0);

        let builder =
            crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        let body = crate::rapier::rigid_body::rigid_body_builder_build(builder);
        let handle = crate::rapier::rigid_body::world_insert_rigid_body(world, body);
        assert_ne!(handle, 0);
        assert_eq!(world_body_snapshot_count(world), 1);

        let handles = [handle];
        let poses = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0];
        assert_eq!(
            world_update_body_poses(world, handles.as_ptr(), poses.as_ptr(), 1, Bool::TRUE),
            1
        );
        let velocities = [4.0, 5.0, 6.0, 0.1, 0.2, 0.3];
        assert_eq!(
            world_update_body_velocities(
                world,
                handles.as_ptr(),
                velocities.as_ptr(),
                1,
                Bool::TRUE
            ),
            1
        );

        let mut out_handles = [0; 1];
        let mut values = [0.0; 13];
        assert_eq!(
            world_body_snapshot(
                world,
                out_handles.as_mut_ptr(),
                values.as_mut_ptr(),
                out_handles.len() as u32,
            ),
            1
        );
        assert_eq!(out_handles[0], handle);
        assert_eq!(&values[..3], &[1.0, 2.0, 3.0]);
        assert_eq!(&values[7..10], &[4.0, 5.0, 6.0]);

        world_destroy(world);
    }
}

// ---------------------------------------------------------------------------

// Shared Arena FFI — zero-JNI physics data access
// ---------------------------------------------------------------------------

/// Create a shared-memory physics arena.
///
/// Returns the arena pointer as a u64 (suitable for `MemorySegment.ofAddress` in Java).
/// The arena persists for the lifetime of the world.
///
/// `max_bodies` — max concurrent bodies to mirror
/// `max_events` — max pending collision/contact events
/// `max_commands` — max pending commands (force/set pose etc.)
/// `out_address` — receives the arena base address
/// `out_size` — receives the total arena size in bytes (for Java MemorySegment mapping)
#[cfg(not(target_arch = "wasm32"))]
#[unsafe(no_mangle)]
#[cfg(not(target_arch = "wasm32"))]
pub extern "C" fn world_create_shared_arena(
    world: *mut WorldHandle,
    max_bodies: u32,
    max_colliders: u32,
    max_events: u32,
    max_commands: u32,
    out_address: *mut u64,
    out_size: *mut u64,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    if max_bodies == 0 || max_events == 0 || max_commands == 0 {
        set_error(ERR_INVALID_ARGUMENT, "arena capacities must be >0");
        return Bool::FALSE;
    }

    let arena = crate::rapier::shared_arena::SharedPhysicsArena::new(
        max_bodies, max_colliders, max_events, max_commands,
    );
    let addr = arena.address();
    let sz = arena.size() as u64;

    world.inner.shared_arena = Some(Box::new(arena));

    if let Some(p) = (unsafe { out_address.as_mut() }) { *p = addr; }
    if let Some(p) = (unsafe { out_size.as_mut() }) { *p = sz; }
    clear_error();
    Bool::TRUE
}

/// Destroy the shared arena (if any).
#[cfg(not(target_arch = "wasm32"))]
#[cfg(not(target_arch = "wasm32"))]
#[unsafe(no_mangle)]
pub extern "C" fn world_destroy_shared_arena(world: *mut WorldHandle) {
    if let Some(world) = (unsafe { world.as_mut() }) {
        world.inner.shared_arena = None;
    }
}
#[cfg(not(target_arch = "wasm32"))]

#[cfg(not(target_arch = "wasm32"))]
/// Get the arena address (returns 0 if no arena).
#[unsafe(no_mangle)]
pub extern "C" fn world_get_shared_arena_address(world: *const WorldHandle) -> u64 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    world.inner.shared_arena.as_ref().map_or(0, |a| a.address())
#[cfg(not(target_arch = "wasm32"))]
}

#[cfg(not(target_arch = "wasm32"))]
/// Get the arena size (returns 0 if no arena).
#[unsafe(no_mangle)]
pub extern "C" fn world_get_shared_arena_size(world: *const WorldHandle) -> u64 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    world.inner.shared_arena.as_ref().map_or(0, |a| a.size() as u64)
}
#[cfg(not(target_arch = "wasm32"))]

#[cfg(not(target_arch = "wasm32"))]
/// Reset the event ring (Java calls this after draining events).
#[unsafe(no_mangle)]
pub extern "C" fn world_reset_shared_arena_events(world: *mut WorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else { return };
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(ref arena) = world.inner.shared_arena {
        arena.reset_event_ring();
    }
}
