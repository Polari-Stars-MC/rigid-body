use rapier3d::prelude::{MassProperties, RigidBodyBuilder};

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    BodyStatus, Bool, Quat, RigidBodyBuilderHandle, RigidBodyHandleRaw, RigidBodyMemHandle, Vec3,
    WorldHandle, body_status_from_rapier, body_status_from_raw, body_status_to_rapier,
    body_status_to_raw, isometry_from_parts, pack_rigid_body_handle, quat_finite, quat_from_rapier,
    quat_to_rapier, unpack_rigid_body_handle, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

#[cfg(feature = "relative-force")]
use rapier3d::prelude::RigidBodyType;

fn builder_from_status(status: BodyStatus) -> RigidBodyBuilder {
    match status {
        BodyStatus::Dynamic => RigidBodyBuilder::dynamic(),
        BodyStatus::Fixed => RigidBodyBuilder::fixed(),
        BodyStatus::KinematicPositionBased => RigidBodyBuilder::kinematic_position_based(),
        BodyStatus::KinematicVelocityBased => RigidBodyBuilder::kinematic_velocity_based(),
    }
}

/// Creates a rigid body builder for the given body status.
///
/// # Safety
///
/// Takes no pointers. The returned pointer is owned by the caller and must be released with
/// `rigid_body_builder_build` or `rigid_body_builder_destroy`.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_create(status: u32) -> *mut RigidBodyBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        clear_error();
        Box::into_raw(Box::new(RigidBodyBuilderHandle {
            inner: builder_from_status(body_status_from_raw(status)),
        }))
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create` (or null); ownership
/// is taken and the pointer must not be used afterwards.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_build(
    builder: *mut RigidBodyBuilderHandle,
) -> *mut RigidBodyMemHandle {
    ffi_guard(std::ptr::null_mut(), || {
        if builder.is_null() {
            set_error(ERR_NULL_POINTER, "builder is null");
            return std::ptr::null_mut();
        }

        let builder = unsafe { Box::from_raw(builder) };
        let RigidBodyBuilderHandle { inner } = *builder;
        clear_error();
        Box::into_raw(Box::new(RigidBodyMemHandle {
            inner: inner.build(),
        }))
    })
}

/// # Safety
///
/// `builder` must be a pointer returned by `rigid_body_builder_create` (or null, which is a
/// no-op); ownership is taken and the pointer must not be used afterwards.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_destroy(builder: *mut RigidBodyBuilderHandle) {
    ffi_guard((), || {
        if builder.is_null() {
            return;
        }

        unsafe {
            drop(Box::from_raw(builder));
        }
    })
}

/// # Safety
///
/// `rigid_body` must be a pointer returned by `rigid_body_builder_build` or
/// `world_copy_rigid_body` (or null, which is a no-op); ownership is taken and the pointer must
/// not be used afterwards.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_destroy_raw(rigid_body: *mut RigidBodyMemHandle) {
    ffi_guard((), || {
        if rigid_body.is_null() {
            return;
        }

        unsafe {
            drop(Box::from_raw(rigid_body));
        }
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_translation(
    builder: *mut RigidBodyBuilderHandle,
    translation: Vec3,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !vec3_finite(translation) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite builder translation");
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.translation(vec3_to_rapier(translation));
        clear_error();
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_rotation(
    builder: *mut RigidBodyBuilderHandle,
    rotation_axis_angle: Vec3,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !vec3_finite(rotation_axis_angle) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "non-finite builder rotation axis-angle",
            );
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.rotation(vec3_to_rapier(rotation_axis_angle));
        clear_error();
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_pose(
    builder: *mut RigidBodyBuilderHandle,
    translation: Vec3,
    rotation: Quat,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !vec3_finite(translation) || !quat_finite(rotation) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite builder pose");
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.pose(isometry_from_parts(translation, rotation));
        clear_error();
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_additional_mass_properties(
    builder: *mut RigidBodyBuilderHandle,
    center: Vec3,
    mass: f64,
    inertia: Vec3,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !vec3_finite(center)
            || !vec3_finite(inertia)
            || !mass.is_finite()
            || mass < 0.0
            || inertia.x < 0.0
            || inertia.y < 0.0
            || inertia.z < 0.0
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid additional mass properties");
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.additional_mass_properties(MassProperties::new(
            vec3_to_rapier(center),
            mass,
            vec3_to_rapier(inertia),
        ));
        clear_error();
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_linvel(
    builder: *mut RigidBodyBuilderHandle,
    linvel: Vec3,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !vec3_finite(linvel) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite builder linear velocity");
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.linvel(vec3_to_rapier(linvel));
        clear_error();
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_angvel(
    builder: *mut RigidBodyBuilderHandle,
    angvel: Vec3,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !vec3_finite(angvel) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite builder angular velocity");
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.angvel(vec3_to_rapier(angvel));
        clear_error();
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_gravity_scale(
    builder: *mut RigidBodyBuilderHandle,
    gravity_scale: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !gravity_scale.is_finite() {
            set_error(ERR_INVALID_ARGUMENT, "non-finite builder gravity scale");
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.gravity_scale(gravity_scale);
        clear_error();
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_linear_damping(
    builder: *mut RigidBodyBuilderHandle,
    linear_damping: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !linear_damping.is_finite() || linear_damping < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid builder linear damping");
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.linear_damping(linear_damping);
        clear_error();
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_angular_damping(
    builder: *mut RigidBodyBuilderHandle,
    angular_damping: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !angular_damping.is_finite() || angular_damping < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid builder angular damping");
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.angular_damping(angular_damping);
        clear_error();
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_can_sleep(
    builder: *mut RigidBodyBuilderHandle,
    can_sleep: Bool,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };

        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.can_sleep(can_sleep.0 != 0);
        clear_error();
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_enabled_rotations(
    builder: *mut RigidBodyBuilderHandle,
    allow_x: Bool,
    allow_y: Bool,
    allow_z: Bool,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };

        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.enabled_rotations(allow_x.0 != 0, allow_y.0 != 0, allow_z.0 != 0);
        clear_error();
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_user_data(
    builder: *mut RigidBodyBuilderHandle,
    user_data_low: u64,
    user_data_high: u64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };

        let user_data = (user_data_low as u128) | ((user_data_high as u128) << 64);
        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.user_data(user_data);
        clear_error();
    })
}

/// # Safety
///
/// `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_additional_mass(
    builder: *mut RigidBodyBuilderHandle,
    mass: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !mass.is_finite() || mass < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid builder additional mass");
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
        builder.inner = inner.additional_mass(mass);
        clear_error();
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null. `memory_handle` must be a
/// pointer returned by `rigid_body_builder_build`; ownership is taken and the pointer must not be
/// used afterwards.
#[unsafe(no_mangle)]
pub extern "C" fn world_insert_rigid_body(
    world: *mut WorldHandle,
    memory_handle: *mut RigidBodyMemHandle,
) -> RigidBodyHandleRaw {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let _query_lock = world.inner.query_lock.write();
        if memory_handle.is_null() {
            set_error(ERR_NULL_POINTER, "rigid body pointer is null");
            return 0;
        }

        let mut built = unsafe { Box::from_raw(memory_handle).inner };
        world.inner.runtime_settings.apply_body(&mut built);
        clear_error();
        pack_rigid_body_handle(world.inner.bodies.insert(built))
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn world_remove_rigid_body(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    remove_attached_colliders: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let _query_lock = world.inner.query_lock.write();

        let removed = world
            .inner
            .bodies
            .remove(
                unpack_rigid_body_handle(handle),
                &mut world.inner.islands,
                &mut world.inner.colliders,
                &mut world.inner.impulse_joints,
                &mut world.inner.multibody_joints,
                &mut world.inner.upstream_soft_bodies,
                remove_attached_colliders.0 != 0,
            )
            .is_some();
        if !removed {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        }
        // B1：被移除的体要从所有模拟域成员表里清掉，避免悬空句柄。
        let removed_handle = unpack_rigid_body_handle(handle);
        for members in world.inner.simulation_domains.values_mut() {
            members.remove(&removed_handle);
        }
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null. The returned pointer is
/// owned by the caller and must be released with `rigid_body_destroy_raw`.
#[unsafe(no_mangle)]
pub extern "C" fn world_copy_rigid_body(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
) -> *mut RigidBodyMemHandle {
    ffi_guard(std::ptr::null_mut(), || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return std::ptr::null_mut();
        };
        let _query_lock = world.inner.query_lock.write();

        let Some(rb) = world
            .inner
            .bodies
            .get(unpack_rigid_body_handle(handle))
            .cloned()
        else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return std::ptr::null_mut();
        };

        clear_error();
        Box::into_raw(Box::new(RigidBodyMemHandle { inner: rb }))
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn world_remove_rigid_body_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    remove_attached_colliders: Bool,
) -> u8 {
    ffi_guard(0, || {
        world_remove_rigid_body(world, handle, remove_attached_colliders).0
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_status(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> u32 {
    ffi_guard(body_status_to_raw(BodyStatus::Fixed), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return body_status_to_raw(BodyStatus::Fixed);
        };

        let Some(body) = world.inner.bodies.get(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return body_status_to_raw(BodyStatus::Fixed);
        };
        clear_error();
        body_status_to_raw(body_status_from_rapier(body.body_type()))
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_status(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    status: u32,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };

        body.set_body_type(
            body_status_to_rapier(body_status_from_raw(status)),
            wake_up.0 != 0,
        );
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_translation(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Vec3 {
    ffi_guard(Vec3::default(), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Vec3::default();
        };

        let Some(body) = world.inner.bodies.get(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Vec3::default();
        };
        clear_error();
        vec3_from_rapier(body.translation())
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null. `out_translation` must be
/// a valid writable pointer to a `Vec3`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_translation_out(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
    out_translation: *mut Vec3,
) {
    ffi_guard((), || {
        let Some(out_translation) = (unsafe { out_translation.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "output pointer is null");
            return;
        };

        *out_translation = rigid_body_get_translation(world, handle);
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_rotation(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Quat {
    ffi_guard(Quat::default(), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Quat::default();
        };

        let Some(body) = world.inner.bodies.get(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Quat::default();
        };
        clear_error();
        quat_from_rapier(*body.rotation())
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null. `out_rotation` must be a
/// valid writable pointer to a `Quat`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_rotation_out(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
    out_rotation: *mut Quat,
) {
    ffi_guard((), || {
        let Some(out_rotation) = (unsafe { out_rotation.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "output pointer is null");
            return;
        };

        *out_rotation = rigid_body_get_rotation(world, handle);
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_pose(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    translation: Vec3,
    rotation: Quat,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(translation) || !quat_finite(rotation) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite body pose");
            return Bool::FALSE;
        }

        body.set_position(isometry_from_parts(translation, rotation), wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_translation(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    translation: Vec3,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(translation) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite body translation");
            return Bool::FALSE;
        }

        body.set_translation(vec3_to_rapier(translation), wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_next_kinematic_position(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    translation: Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(translation) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite body translation");
            return Bool::FALSE;
        }

        let rotation = *body.rotation();
        body.set_next_kinematic_position(rapier3d::math::Pose::from_parts(
            vec3_to_rapier(translation),
            rotation,
        ));
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_translation_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    translation: Vec3,
    wake_up: Bool,
) -> u8 {
    ffi_guard(0, || {
        rigid_body_set_translation(world, handle, translation, wake_up).0
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_rotation(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    rotation: Quat,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !quat_finite(rotation) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite body rotation");
            return Bool::FALSE;
        }

        body.set_rotation(quat_to_rapier(rotation), wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_rotation_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    rotation: Quat,
    wake_up: Bool,
) -> u8 {
    ffi_guard(0, || {
        rigid_body_set_rotation(world, handle, rotation, wake_up).0
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_pose_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    translation: Vec3,
    rotation: Quat,
    wake_up: Bool,
) -> u8 {
    ffi_guard(0, || {
        rigid_body_set_pose(world, handle, translation, rotation, wake_up).0
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_mass(world: *mut WorldHandle, handle: RigidBodyHandleRaw) -> f64 {
    ffi_guard(0.0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0.0;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return 0.0;
        };

        clear_error();
        body.mass()
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_force(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Vec3 {
    ffi_guard(Vec3::default(), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Vec3::default();
        };

        let Some(body) = world.inner.bodies.get(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Vec3::default();
        };
        clear_error();
        vec3_from_rapier(body.user_force())
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_linvel(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Vec3 {
    ffi_guard(Vec3::default(), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Vec3::default();
        };

        let Some(body) = world.inner.bodies.get(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Vec3::default();
        };
        clear_error();
        vec3_from_rapier(body.linvel())
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null. `out_linvel` must be a
/// valid writable pointer to a `Vec3`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_linvel_out(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
    out_linvel: *mut Vec3,
) {
    ffi_guard((), || {
        let Some(out_linvel) = (unsafe { out_linvel.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "output pointer is null");
            return;
        };

        *out_linvel = rigid_body_get_linvel(world, handle);
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_linvel(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    linvel: Vec3,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(linvel) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite body linear velocity");
            return Bool::FALSE;
        }

        body.set_linvel(vec3_to_rapier(linvel), wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_linvel_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    linvel: Vec3,
    wake_up: Bool,
) -> u8 {
    ffi_guard(0, || {
        rigid_body_set_linvel(world, handle, linvel, wake_up).0
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_angvel(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Vec3 {
    ffi_guard(Vec3::default(), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Vec3::default();
        };

        let Some(body) = world.inner.bodies.get(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Vec3::default();
        };
        clear_error();
        vec3_from_rapier(body.angvel())
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null. `out_angvel` must be a
/// valid writable pointer to a `Vec3`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_angvel_out(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
    out_angvel: *mut Vec3,
) {
    ffi_guard((), || {
        let Some(out_angvel) = (unsafe { out_angvel.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "output pointer is null");
            return;
        };

        *out_angvel = rigid_body_get_angvel(world, handle);
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_angvel(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    angvel: Vec3,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(angvel) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite body angular velocity");
            return Bool::FALSE;
        }

        body.set_angvel(vec3_to_rapier(angvel), wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_angvel_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    angvel: Vec3,
    wake_up: Bool,
) -> u8 {
    ffi_guard(0, || {
        rigid_body_set_angvel(world, handle, angvel, wake_up).0
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_force(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    force: Vec3,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(force) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite body force");
            return Bool::FALSE;
        }

        body.add_force(vec3_to_rapier(force), wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_force_at_point(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    force: Vec3,
    point: Vec3,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(force) || !vec3_finite(point) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite body force or point");
            return Bool::FALSE;
        }

        body.add_force_at_point(vec3_to_rapier(force), vec3_to_rapier(point), wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[cfg(feature = "relative-force")]
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_force_at_local_point(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    force: Vec3,
    local_point: Vec3,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(force) || !vec3_finite(local_point) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite body force or local point");
            return Bool::FALSE;
        }
        if body.body_type() != RigidBodyType::Dynamic {
            set_error(
                ERR_INVALID_ARGUMENT,
                "relative force only works on dynamic bodies",
            );
            return Bool::FALSE;
        }

        let world_point = body.position().transform_point(vec3_to_rapier(local_point));
        body.add_force_at_point(vec3_to_rapier(force), world_point, wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[cfg(feature = "relative-force")]
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_torque_at_local_point(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    torque: Vec3,
    _local_point: Vec3,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(torque) || !vec3_finite(_local_point) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "non-finite body torque or local point",
            );
            return Bool::FALSE;
        }
        if body.body_type() != RigidBodyType::Dynamic {
            set_error(
                ERR_INVALID_ARGUMENT,
                "relative torque only works on dynamic bodies",
            );
            return Bool::FALSE;
        }

        body.add_torque(vec3_to_rapier(torque), wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[cfg(feature = "relative-force")]
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_force_at_local_point_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    force: Vec3,
    local_point: Vec3,
    wake_up: Bool,
) -> u8 {
    ffi_guard(0, || {
        rigid_body_add_force_at_local_point(world, handle, force, local_point, wake_up).0
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[cfg(feature = "relative-force")]
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_torque_at_local_point_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    torque: Vec3,
    local_point: Vec3,
    wake_up: Bool,
) -> u8 {
    ffi_guard(0, || {
        rigid_body_add_torque_at_local_point(world, handle, torque, local_point, wake_up).0
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_reset_force(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };

        body.reset_forces(wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_force_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    force: Vec3,
    wake_up: Bool,
) -> u8 {
    ffi_guard(0, || rigid_body_add_force(world, handle, force, wake_up).0)
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_torque(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    torque: Vec3,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(torque) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite body torque");
            return Bool::FALSE;
        }

        body.add_torque(vec3_to_rapier(torque), wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_reset_torque(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };

        body.reset_torques(wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_torque_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    torque: Vec3,
    wake_up: Bool,
) -> u8 {
    ffi_guard(0, || {
        rigid_body_add_torque(world, handle, torque, wake_up).0
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_apply_impulse(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    impulse: Vec3,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(impulse) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite body impulse");
            return Bool::FALSE;
        }

        body.apply_impulse(vec3_to_rapier(impulse), wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_apply_impulse_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    impulse: Vec3,
    wake_up: Bool,
) -> u8 {
    ffi_guard(0, || {
        rigid_body_apply_impulse(world, handle, impulse, wake_up).0
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_apply_torque_impulse(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    torque_impulse: Vec3,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        if !vec3_finite(torque_impulse) {
            set_error(ERR_INVALID_ARGUMENT, "non-finite body torque impulse");
            return Bool::FALSE;
        }

        body.apply_torque_impulse(vec3_to_rapier(torque_impulse), wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_apply_torque_impulse_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    torque_impulse: Vec3,
    wake_up: Bool,
) -> u8 {
    ffi_guard(0, || {
        rigid_body_apply_torque_impulse(world, handle, torque_impulse, wake_up).0
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_enable_ccd(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    enabled: Bool,
) -> Bool {
    // Shim (ADR-0016 迁移第 2 步): 转发到通用属性通道。`Bool` 按 0/1 映射成`f64`。
    // 外层保留 `ffi_guard`：`verify_ffi_guard` 要求每个 `extern "C"` 入口
    // 直接包含它，而不是依赖被调用者内部的守卫。
    ffi_guard(Bool::FALSE, || {
        rigid_body_set_param(world, handle, RIGID_BODY_PARAM_ENABLE_CCD, enabled.0 as f64)
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_enable_ccd_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    enabled: Bool,
) -> u8 {
    ffi_guard(0, || rigid_body_enable_ccd(world, handle, enabled).0)
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_sleep(world: *mut WorldHandle, handle: RigidBodyHandleRaw) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };

        body.sleep();
        clear_error();
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_sleep_flag(world: *mut WorldHandle, handle: RigidBodyHandleRaw) -> u8 {
    ffi_guard(0, || rigid_body_sleep(world, handle).0)
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_wake_up(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    strong: Bool,
) -> Bool {
    // Shim (ADR-0016 迁移第 2 步): 转发到通用属性通道。`Bool` 按 0/1 映射成`f64`。
    // 外层保留 `ffi_guard`：`verify_ffi_guard` 要求每个 `extern "C"` 入口
    // 直接包含它，而不是依赖被调用者内部的守卫。
    ffi_guard(Bool::FALSE, || {
        rigid_body_set_param(world, handle, RIGID_BODY_PARAM_WAKE_UP, strong.0 as f64)
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_wake_up_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    strong: Bool,
) -> u8 {
    ffi_guard(0, || rigid_body_wake_up(world, handle, strong).0)
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_is_sleeping(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        let Some(body) = world.inner.bodies.get(unpack_rigid_body_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "body was not found");
            return Bool::FALSE;
        };
        clear_error();
        body.is_sleeping().into()
    })
}

/// # Safety
///
/// `world` must be a live pointer returned by `world_create`, or null.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_is_sleeping_flag(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> u8 {
    ffi_guard(0, || rigid_body_is_sleeping(world, handle).0)
}

// ── 通用属性通道（ADR-0016，rigid_body 批） ──────────────────────────────────
//
// 第三个模块。这里能吸收的入口很少——扫描（docs/abi-merge-scan.md）显示
// rigid 的 34 个未覆盖 setter 里绝大多数是 `Vec3 + Bool` 形态（施加力/冲量）
// 或向量型的速度/位姿设置，都不属于"标量属性"。
//
// ## 为什么只有 3 个属性
//
// * `simulation_domain`（u32）**故意不收**：它要拿查询锁、把体从旧域摘除、
//   在目标域 inactive 时立刻冻结。这些副作用不适合塞进通用通道，留在专用入口
//   更安全。这是"能合并但不该合并"的例子。
// * `Vec3 + Bool` 那一批（`add_force` / `apply_impulse` / `set_linvel` …）
//   需要"向量 + 可选开关"的通道，那是另一个设计，不在本批。
//
// 留下来的三个都是干净的实体级布尔/标量开关。

/// `rigid_body_set_param` 的属性编号。
pub const RIGID_BODY_PARAM_ENABLE_CCD: u32 = 0;
/// `wake_up(strong)`：非零 = 强唤醒（清空睡眠计时器），零 = 普通唤醒。
pub const RIGID_BODY_PARAM_WAKE_UP: u32 = 1;
/// `enabled`：暂停/恢复该体的模拟。
pub const RIGID_BODY_PARAM_ENABLED: u32 = 2;

/// 已分配的 rigid_body 属性个数。
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_param_abi_version() -> u32 {
    ffi_guard(0, || 3)
}

/// 通用 rigid-body 属性写入通道。
///
/// 语义同 `soft_body_set_param` / `collider_set_param`（ADR-0016 三条规则）。
///
/// # Returns
/// `Bool::TRUE` 成功；world 为 null → `ERR_NULL_POINTER`；句柄无效 →
/// `ERR_NOT_FOUND`；`param` 未分配或载荷非有限 → `ERR_INVALID_ARGUMENT`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_param(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    param: u32,
    value: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "rigid_body_set_param: world is null");
            return Bool::FALSE;
        };
        if !value.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "rigid_body_set_param: value must be finite",
            );
            return Bool::FALSE;
        }
        let bh = unpack_rigid_body_handle(handle);
        let Some(body) = world.inner.bodies.get_mut(bh) else {
            set_error(ERR_NOT_FOUND, "rigid_body_set_param: body was not found");
            return Bool::FALSE;
        };
        match param {
            RIGID_BODY_PARAM_ENABLE_CCD => body.enable_ccd(value != 0.0),
            RIGID_BODY_PARAM_WAKE_UP => body.wake_up(value != 0.0),
            RIGID_BODY_PARAM_ENABLED => body.set_enabled(value != 0.0),
            _ => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "rigid_body_set_param: unknown param id \
                     (see rigid_body_param_abi_version)",
                );
                return Bool::FALSE;
            }
        }
        clear_error();
        Bool::TRUE
    })
}

/// `u8` 返回变体，语义同 [`rigid_body_set_param`]。
///
/// # Safety
/// 同 [`rigid_body_set_param`]。
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_param_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    param: u32,
    value: f64,
) -> u8 {
    ffi_guard(0, || rigid_body_set_param(world, handle, param, value).0)
}

// ── 向量 + 可选开关通道（ADR-0017） ───────────────────────────────────────────
//
// 一批入口的形状是 `(world, handle, Vec3, Bool)`：一个向量载荷加一个 wake_up
// 开关。`collider_set_vec3_param` 装不下最后的 `Bool`，所以需要独立通道。
//
// ## 为什么不直接给 vec3 通道加参数
//
// `collider_set_vec3_param(world, handle, param, x, y, z)` 的语义是"把这个向量
// 属性设成 (x,y,z)"。如果再加一个 `wake_up`，通道语义就变成"设一个值 + 可能
// 顺带唤醒"，而 collider 那边根本没有唤醒概念。给不相关的模块加无用参数会污染
// 语义，所以独立成 `rigid_body_set_vec3_flag_param`。
//
// ## 一个必须记清的语义分歧
//
// 这 14 个入口分成两类，**语义完全不同**：
//
// | 类型 | 代表 | 语义 |
// | --- | --- | --- |
// | 累积 | `add_force` / `add_torque` / `apply_impulse` | **加上**一份，可多次调用叠加 |
// | 设定 | `set_linvel` / `set_angvel` / `set_translation` | **设成**这个值，覆盖 |
//
// 两类共用同一个签名形状，所以能进同一个通道，但 `param` 必须区分开 ——
// 把 `ADD_FORCE` 和 `SET_LINVEL` 混成一个编号会让"累积"和"覆盖"变成同一件事，
// 那是灾难性的语义错误。
//
// ## wake_up 的处理
//
// `Bool` 在 C 侧是 0/1，通道用 `u8` 接收（与仓库既有的 `_flag` 约定一致）。
// 非零即真，与专用入口的 `wake_up.0 != 0` 判定一致。

/// `rigid_body_set_vec3_flag_param` 的属性编号。
///
/// **累积型（0~3）与设定型（4~6）必须分开编号**，见上面的语义分歧说明。
pub const RIGID_BODY_VEC3_ADD_FORCE: u32 = 0;
pub const RIGID_BODY_VEC3_ADD_TORQUE: u32 = 1;
pub const RIGID_BODY_VEC3_APPLY_IMPULSE: u32 = 2;
pub const RIGID_BODY_VEC3_APPLY_TORQUE_IMPULSE: u32 = 3;
pub const RIGID_BODY_VEC3_SET_LINVEL: u32 = 4;
pub const RIGID_BODY_VEC3_SET_ANGVEL: u32 = 5;
pub const RIGID_BODY_VEC3_SET_TRANSLATION: u32 = 6;

/// 已分配的"向量 + 开关"属性个数。
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_vec3_flag_param_abi_version() -> u32 {
    ffi_guard(0, || 7)
}

/// 向量 + 可选唤醒通道。
///
/// # Returns
/// `Bool::TRUE` 成功；world 为 null → `ERR_NULL_POINTER`；句柄无效 →
/// `ERR_NOT_FOUND`；`param` 未分配或分量非有限 → `ERR_INVALID_ARGUMENT`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_vec3_flag_param(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    param: u32,
    x: f64,
    y: f64,
    z: f64,
    flag: u8,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(
                ERR_NULL_POINTER,
                "rigid_body_set_vec3_flag_param: world is null",
            );
            return Bool::FALSE;
        };
        let value = Vec3 { x, y, z };
        if !vec3_finite(value) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "rigid_body_set_vec3_flag_param: vector components must be finite",
            );
            return Bool::FALSE;
        }
        let bh = unpack_rigid_body_handle(handle);
        let Some(body) = world.inner.bodies.get_mut(bh) else {
            set_error(
                ERR_NOT_FOUND,
                "rigid_body_set_vec3_flag_param: body was not found",
            );
            return Bool::FALSE;
        };
        // 与专用入口的 `wake_up.0 != 0` 一致：非零即真。
        let wake = flag != 0;
        let v = vec3_to_rapier(value);
        match param {
            // ── 累积型：多次调用叠加 ──
            RIGID_BODY_VEC3_ADD_FORCE => body.add_force(v, wake),
            RIGID_BODY_VEC3_ADD_TORQUE => body.add_torque(v, wake),
            RIGID_BODY_VEC3_APPLY_IMPULSE => body.apply_impulse(v, wake),
            RIGID_BODY_VEC3_APPLY_TORQUE_IMPULSE => body.apply_torque_impulse(v, wake),
            // ── 设定型：覆盖当前值 ──
            RIGID_BODY_VEC3_SET_LINVEL => body.set_linvel(v, wake),
            RIGID_BODY_VEC3_SET_ANGVEL => body.set_angvel(v, wake),
            RIGID_BODY_VEC3_SET_TRANSLATION => body.set_translation(v, wake),
            _ => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "rigid_body_set_vec3_flag_param: unknown param id \
                     (see rigid_body_vec3_flag_param_abi_version)",
                );
                return Bool::FALSE;
            }
        }
        clear_error();
        Bool::TRUE
    })
}
