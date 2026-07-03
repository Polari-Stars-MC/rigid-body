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

const MAX_STEP_SECONDS: f64 = 1.0;

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

    // Cache custom physics read once to avoid double RwLock acquire.
    // The coulomb hook setup only needs to run when the law is first enabled;
    // we detect this by checking whether MODIFY_SOLVER_CONTACTS is already set
    // on the first collider (all are set uniformly).
    let custom = world.inner.events.custom_physics();
    let coulomb_active = custom
        .coulomb_friction
        .is_some_and(|law| law.enabled.0 != 0);

    if coulomb_active {
        // Only set the hook on colliders that don't already have it.
        // Most colliders get it set on first step and skip thereafter.
        let hook_bit = ActiveHooks::MODIFY_SOLVER_CONTACTS;
        for (_, collider) in world.inner.colliders.iter_mut() {
            let current = collider.active_hooks();
            if !current.contains(hook_bit) {
                collider.set_active_hooks(current | hook_bit);
            }
        }
    }

    crate::rapier::events::apply_custom_external_forces_with_custom(
        &mut world.inner,
        custom.clone(),
    );
    // Run body-body interactions: pairwise gravity, Coulomb friction, air drag
    crate::rapier::interaction::apply_body_interactions(&mut world.inner, &custom);
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
