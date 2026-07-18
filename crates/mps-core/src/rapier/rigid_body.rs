use rapier3d::dynamics::RigidBody;
use rapier3d::prelude::{MassProperties, RigidBodyBuilder};

use crate::rapier::ffi::{
    BodyStatus, Bool, Quat, RigidBodyBuilderHandle, RigidBodyHandleRaw, Vec3, WorldHandle,
    body_status_from_rapier, body_status_from_raw, body_status_to_rapier, body_status_to_raw,
    isometry_from_parts, pack_rigid_body_handle, quat_finite, quat_from_rapier, quat_to_rapier,
    unpack_rigid_body_handle, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

fn builder_from_status(status: BodyStatus) -> RigidBodyBuilder {
    match status {
        BodyStatus::Dynamic => RigidBodyBuilder::dynamic(),
        BodyStatus::Fixed => RigidBodyBuilder::fixed(),
        BodyStatus::KinematicPositionBased => RigidBodyBuilder::kinematic_position_based(),
        BodyStatus::KinematicVelocityBased => RigidBodyBuilder::kinematic_velocity_based(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_create(status: u32) -> *mut RigidBodyBuilderHandle {
    Box::into_raw(Box::new(RigidBodyBuilderHandle {
        inner: builder_from_status(body_status_from_raw(status)),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_build(builder: *mut RigidBodyBuilderHandle) -> *mut RigidBody {
    if builder.is_null() {
        return std::ptr::null_mut();
    }

    let builder = unsafe { Box::from_raw(builder) };
    let RigidBodyBuilderHandle { inner } = *builder;
    Box::into_raw(Box::new(inner.build()))
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_destroy(builder: *mut RigidBodyBuilderHandle) {
    if builder.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(builder));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_destroy_raw(rigid_body: *mut RigidBody) {
    if rigid_body.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(rigid_body));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_translation(
    builder: *mut RigidBodyBuilderHandle,
    translation: Vec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !vec3_finite(translation) {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.translation(vec3_to_rapier(translation));
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_rotation(
    builder: *mut RigidBodyBuilderHandle,
    rotation_axis_angle: Vec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !vec3_finite(rotation_axis_angle) {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.rotation(vec3_to_rapier(rotation_axis_angle));
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_pose(
    builder: *mut RigidBodyBuilderHandle,
    translation: Vec3,
    rotation: Quat,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !vec3_finite(translation) || !quat_finite(rotation) {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.pose(isometry_from_parts(translation, rotation));
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_additional_mass_properties(
    builder: *mut RigidBodyBuilderHandle,
    center: Vec3,
    mass: f64,
    inertia: Vec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
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
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.additional_mass_properties(MassProperties::new(
        vec3_to_rapier(center),
        mass,
        vec3_to_rapier(inertia),
    ));
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_linvel(
    builder: *mut RigidBodyBuilderHandle,
    linvel: Vec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !vec3_finite(linvel) {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.linvel(vec3_to_rapier(linvel));
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_angvel(
    builder: *mut RigidBodyBuilderHandle,
    angvel: Vec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !vec3_finite(angvel) {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.angvel(vec3_to_rapier(angvel));
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_gravity_scale(
    builder: *mut RigidBodyBuilderHandle,
    gravity_scale: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !gravity_scale.is_finite() {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.gravity_scale(gravity_scale);
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_linear_damping(
    builder: *mut RigidBodyBuilderHandle,
    linear_damping: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !linear_damping.is_finite() || linear_damping < 0.0 {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.linear_damping(linear_damping);
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_angular_damping(
    builder: *mut RigidBodyBuilderHandle,
    angular_damping: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !angular_damping.is_finite() || angular_damping < 0.0 {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.angular_damping(angular_damping);
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_can_sleep(
    builder: *mut RigidBodyBuilderHandle,
    can_sleep: Bool,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.can_sleep(can_sleep.0 != 0);
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_enabled_rotations(
    builder: *mut RigidBodyBuilderHandle,
    allow_x: Bool,
    allow_y: Bool,
    allow_z: Bool,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.enabled_rotations(allow_x.0 != 0, allow_y.0 != 0, allow_z.0 != 0);
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_user_data(
    builder: *mut RigidBodyBuilderHandle,
    user_data_low: u64,
    user_data_high: u64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let user_data = (user_data_low as u128) | ((user_data_high as u128) << 64);
    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.user_data(user_data);
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_builder_set_additional_mass(
    builder: *mut RigidBodyBuilderHandle,
    mass: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !mass.is_finite() || mass < 0.0 {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, RigidBodyBuilder::dynamic());
    builder.inner = inner.additional_mass(mass);
}

#[unsafe(no_mangle)]
pub extern "C" fn world_insert_rigid_body(
    world: *mut WorldHandle,
    memory_handle: *mut RigidBody,
) -> RigidBodyHandleRaw {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0;
    };
    if memory_handle.is_null() {
        return 0;
    }

    let built = unsafe { *Box::from_raw(memory_handle) };
    pack_rigid_body_handle(world.inner.bodies.insert(built))
}

#[unsafe(no_mangle)]
pub extern "C" fn world_remove_rigid_body(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    remove_attached_colliders: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };

    world
        .inner
        .bodies
        .remove(
            unpack_rigid_body_handle(handle),
            &mut world.inner.islands,
            &mut world.inner.colliders,
            &mut world.inner.impulse_joints,
            &mut world.inner.multibody_joints,
            remove_attached_colliders.0 != 0,
        )
        .is_some()
        .into()
}

#[unsafe(no_mangle)]
pub extern "C" fn world_copy_rigid_body(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
) -> *mut RigidBody {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return std::ptr::null_mut();
    };

    let Some(rb) = world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .cloned()
    else {
        return std::ptr::null_mut();
    };

    Box::into_raw(Box::new(rb))
}

#[unsafe(no_mangle)]
pub extern "C" fn world_remove_rigid_body_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    remove_attached_colliders: Bool,
) -> u8 {
    world_remove_rigid_body(world, handle, remove_attached_colliders).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_status(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return body_status_to_raw(BodyStatus::Fixed);
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| body_status_to_raw(body_status_from_rapier(body.body_type())))
        .unwrap_or(body_status_to_raw(BodyStatus::Fixed))
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_status(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    status: u32,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };

    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };

    body.set_body_type(
        body_status_to_rapier(body_status_from_raw(status)),
        wake_up.0 != 0,
    );
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_translation(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Vec3 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return Vec3::default();
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| vec3_from_rapier(body.translation()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_translation_out(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
    out_translation: *mut Vec3,
) {
    let Some(out_translation) = (unsafe { out_translation.as_mut() }) else {
        return;
    };

    *out_translation = rigid_body_get_translation(world, handle);
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_rotation(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Quat {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return Quat::default();
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| quat_from_rapier(*body.rotation()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_rotation_out(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
    out_rotation: *mut Quat,
) {
    let Some(out_rotation) = (unsafe { out_rotation.as_mut() }) else {
        return;
    };

    *out_rotation = rigid_body_get_rotation(world, handle);
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_pose(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    translation: Vec3,
    rotation: Quat,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };
    if !vec3_finite(translation) || !quat_finite(rotation) {
        return Bool::FALSE;
    }

    body.set_position(isometry_from_parts(translation, rotation), wake_up.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_translation(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    translation: Vec3,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };
    if !vec3_finite(translation) {
        return Bool::FALSE;
    }

    body.set_translation(vec3_to_rapier(translation), wake_up.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_translation_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    translation: Vec3,
    wake_up: Bool,
) -> u8 {
    rigid_body_set_translation(world, handle, translation, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_rotation(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    rotation: Quat,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };
    if !quat_finite(rotation) {
        return Bool::FALSE;
    }

    body.set_rotation(quat_to_rapier(rotation), wake_up.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_rotation_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    rotation: Quat,
    wake_up: Bool,
) -> u8 {
    rigid_body_set_rotation(world, handle, rotation, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_pose_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    translation: Vec3,
    rotation: Quat,
    wake_up: Bool,
) -> u8 {
    rigid_body_set_pose(world, handle, translation, rotation, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_mass(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
) -> f64 {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0.0;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return 0.0;
    };

    body.mass()
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_force(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Vec3 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return Vec3::default();
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| vec3_from_rapier(body.user_force()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_linvel(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Vec3 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return Vec3::default();
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| vec3_from_rapier(body.linvel()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_linvel_out(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
    out_linvel: *mut Vec3,
) {
    let Some(out_linvel) = (unsafe { out_linvel.as_mut() }) else {
        return;
    };

    *out_linvel = rigid_body_get_linvel(world, handle);
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_linvel(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    linvel: Vec3,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };
    if !vec3_finite(linvel) {
        return Bool::FALSE;
    }

    body.set_linvel(vec3_to_rapier(linvel), wake_up.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_linvel_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    linvel: Vec3,
    wake_up: Bool,
) -> u8 {
    rigid_body_set_linvel(world, handle, linvel, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_angvel(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Vec3 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return Vec3::default();
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| vec3_from_rapier(body.angvel()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_get_angvel_out(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
    out_angvel: *mut Vec3,
) {
    let Some(out_angvel) = (unsafe { out_angvel.as_mut() }) else {
        return;
    };

    *out_angvel = rigid_body_get_angvel(world, handle);
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_angvel(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    angvel: Vec3,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };
    if !vec3_finite(angvel) {
        return Bool::FALSE;
    }

    body.set_angvel(vec3_to_rapier(angvel), wake_up.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_set_angvel_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    angvel: Vec3,
    wake_up: Bool,
) -> u8 {
    rigid_body_set_angvel(world, handle, angvel, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_force(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    force: Vec3,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };
    if !vec3_finite(force) {
        return Bool::FALSE;
    }

    body.add_force(vec3_to_rapier(force), wake_up.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_force_at_point(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    force: Vec3,
    point: Vec3,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };
    if !vec3_finite(force) || !vec3_finite(point) {
        return Bool::FALSE;
    }

    body.add_force_at_point(vec3_to_rapier(force), vec3_to_rapier(point), wake_up.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_reset_force(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };

    body.reset_forces(wake_up.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_force_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    force: Vec3,
    wake_up: Bool,
) -> u8 {
    rigid_body_add_force(world, handle, force, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_torque(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    torque: Vec3,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };
    if !vec3_finite(torque) {
        return Bool::FALSE;
    }

    body.add_torque(vec3_to_rapier(torque), wake_up.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_reset_torque(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };

    body.reset_torques(wake_up.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_add_torque_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    torque: Vec3,
    wake_up: Bool,
) -> u8 {
    rigid_body_add_torque(world, handle, torque, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_apply_impulse(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    impulse: Vec3,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };
    if !vec3_finite(impulse) {
        return Bool::FALSE;
    }

    body.apply_impulse(vec3_to_rapier(impulse), wake_up.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_apply_impulse_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    impulse: Vec3,
    wake_up: Bool,
) -> u8 {
    rigid_body_apply_impulse(world, handle, impulse, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_apply_torque_impulse(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    torque_impulse: Vec3,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };
    if !vec3_finite(torque_impulse) {
        return Bool::FALSE;
    }

    body.apply_torque_impulse(vec3_to_rapier(torque_impulse), wake_up.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_apply_torque_impulse_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    torque_impulse: Vec3,
    wake_up: Bool,
) -> u8 {
    rigid_body_apply_torque_impulse(world, handle, torque_impulse, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_enable_ccd(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    enabled: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };

    body.enable_ccd(enabled.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_enable_ccd_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    enabled: Bool,
) -> u8 {
    rigid_body_enable_ccd(world, handle, enabled).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_sleep(world: *mut WorldHandle, handle: RigidBodyHandleRaw) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };

    body.sleep();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_sleep_flag(world: *mut WorldHandle, handle: RigidBodyHandleRaw) -> u8 {
    rigid_body_sleep(world, handle).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_wake_up(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    strong: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(handle)) else {
        return Bool::FALSE;
    };

    body.wake_up(strong.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_wake_up_flag(
    world: *mut WorldHandle,
    handle: RigidBodyHandleRaw,
    strong: Bool,
) -> u8 {
    rigid_body_wake_up(world, handle, strong).0
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_is_sleeping(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> Bool {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return Bool::FALSE;
    };

    world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(handle))
        .map(|body| body.is_sleeping().into())
        .unwrap_or(Bool::FALSE)
}

#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_is_sleeping_flag(
    world: *const WorldHandle,
    handle: RigidBodyHandleRaw,
) -> u8 {
    rigid_body_is_sleeping(world, handle).0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::world::world_create;
    use crate::rapier::world::world_destroy;

    fn make_world() -> *mut WorldHandle {
        world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        })
    }

    fn make_dynamic_body(world: *mut WorldHandle) -> RigidBodyHandleRaw {
        let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        assert!(!builder.is_null());
        let body = rigid_body_builder_build(builder);
        assert!(!body.is_null());
        let handle = world_insert_rigid_body(world, body);
        assert_ne!(handle, 0);
        handle
    }

    // ---- builder create / build / destroy ----

    #[test]
    fn builder_create_for_all_statuses() {
        for status in [
            BodyStatus::Dynamic,
            BodyStatus::Fixed,
            BodyStatus::KinematicPositionBased,
            BodyStatus::KinematicVelocityBased,
        ] {
            let b = rigid_body_builder_create(status as u32);
            assert!(!b.is_null());
            rigid_body_builder_destroy(b);
        }
    }

    #[test]
    fn builder_destroy_null_is_noop() {
        rigid_body_builder_destroy(std::ptr::null_mut());
    }

    #[test]
    fn build_and_destroy() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        let body = rigid_body_builder_build(b);
        assert!(!body.is_null());
        rigid_body_destroy_raw(body);
    }

    #[test]
    fn destroy_null_body_is_noop() {
        rigid_body_destroy_raw(std::ptr::null_mut());
    }

    // ---- builder setters ----

    #[test]
    fn builder_set_translation_works() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_translation(b, Vec3 { x: 1.0, y: 2.0, z: 3.0 });
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_translation_rejects_nan() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_translation(b, Vec3 { x: f64::NAN, y: 0.0, z: 0.0 });
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_rotation_rejects_nan() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_rotation(b, Vec3 { x: f64::NAN, y: 0.0, z: 0.0 });
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_pose_works() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_pose(
            b,
            Vec3 { x: 1.0, y: 0.0, z: 0.0 },
            Quat { i: 0.0, j: 0.0, k: 0.0, w: 1.0 },
        );
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_additional_mass_properties_rejects_negative_mass() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_additional_mass_properties(
            b,
            Vec3::default(),
            -1.0,
            Vec3 { x: 1.0, y: 1.0, z: 1.0 },
        );
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_linvel_works() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_linvel(b, Vec3 { x: 10.0, y: 0.0, z: 0.0 });
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_angvel_works() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_angvel(b, Vec3 { x: 0.0, y: 1.0, z: 0.0 });
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_gravity_scale_works() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_gravity_scale(b, 0.5);
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_linear_damping_rejects_negative() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_linear_damping(b, -0.1);
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_angular_damping_works() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_angular_damping(b, 0.3);
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_can_sleep_works() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_can_sleep(b, Bool::TRUE);
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_enabled_rotations_works() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_enabled_rotations(b, Bool::TRUE, Bool::FALSE, Bool::TRUE);
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_user_data_works() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_user_data(b, 42, 0);
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_additional_mass_rejects_negative() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_additional_mass(b, -5.0);
        rigid_body_builder_destroy(b);
    }

    // ---- world insert / remove / copy ----

    #[test]
    fn world_insert_rejects_null_world() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        let body = rigid_body_builder_build(b);
        assert_eq!(world_insert_rigid_body(std::ptr::null_mut(), body), 0);
        rigid_body_destroy_raw(body);
    }

    #[test]
    fn world_insert_rejects_null_body() {
        let world = make_world();
        assert_eq!(world_insert_rigid_body(world, std::ptr::null_mut()), 0);
        world_destroy(world);
    }

    #[test]
    fn world_insert_and_remove() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(world_remove_rigid_body(world, handle, Bool::FALSE), Bool::TRUE);
        assert_eq!(world_remove_rigid_body(world, handle, Bool::FALSE), Bool::FALSE);
        world_destroy(world);
    }

    #[test]
    fn copy_rigid_body_works() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        let copy = world_copy_rigid_body(world, handle);
        assert!(!copy.is_null());
        rigid_body_destroy_raw(copy);
        world_destroy(world);
    }

    #[test]
    fn copy_rigid_body_rejects_null_world() {
        assert!(world_copy_rigid_body(std::ptr::null_mut(), 1).is_null());
    }

    #[test]
    fn copy_rigid_body_rejects_invalid_handle() {
        let world = make_world();
        assert!(world_copy_rigid_body(world, 0).is_null());
        world_destroy(world);
    }

    // ---- rigid body status ----

    #[test]
    fn get_status_returns_dynamic() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(rigid_body_get_status(world, handle), BodyStatus::Dynamic as u32);
        world_destroy(world);
    }

    #[test]
    fn get_status_null_world_returns_fixed() {
        assert_eq!(
            rigid_body_get_status(std::ptr::null_mut(), 1),
            BodyStatus::Fixed as u32
        );
    }

    #[test]
    fn get_status_invalid_handle_returns_fixed() {
        let world = make_world();
        assert_eq!(rigid_body_get_status(world, 0), BodyStatus::Fixed as u32);
        world_destroy(world);
    }

    #[test]
    fn set_status_changes_type() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_status(world, handle, BodyStatus::KinematicVelocityBased as u32, Bool::TRUE),
            Bool::TRUE
        );
        assert_eq!(
            rigid_body_get_status(world, handle),
            BodyStatus::KinematicVelocityBased as u32
        );
        world_destroy(world);
    }

    #[test]
    fn set_status_rejects_null_world() {
        assert_eq!(
            rigid_body_set_status(std::ptr::null_mut(), 1, BodyStatus::Dynamic as u32, Bool::TRUE),
            Bool::FALSE
        );
    }

    #[test]
    fn set_status_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(
            rigid_body_set_status(world, 0, BodyStatus::Dynamic as u32, Bool::TRUE),
            Bool::FALSE
        );
        world_destroy(world);
    }

    // ---- get/set translation / rotation / pose ----

    #[test]
    fn get_translation_is_zero_by_default() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        let t = rigid_body_get_translation(world, handle);
        assert!((t.x - 0.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn get_translation_null_world_returns_zero() {
        let t = rigid_body_get_translation(std::ptr::null(), 1);
        assert_eq!(t.x, 0.0);
        assert_eq!(t.y, 0.0);
        assert_eq!(t.z, 0.0);
    }

    #[test]
    fn get_translation_out_writes_value() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        let mut out = Vec3::default();
        rigid_body_get_translation_out(world, handle, &mut out);
        assert!(out.x.is_finite());
        world_destroy(world);
    }

    #[test]
    fn get_translation_out_rejects_null_out() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_get_translation_out(world, handle, std::ptr::null_mut());
        world_destroy(world);
    }

    #[test]
    fn get_rotation_is_identity_by_default() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        let q = rigid_body_get_rotation(world, handle);
        assert!((q.w - 1.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn set_translation_moves_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_translation(world, handle, Vec3 { x: 5.0, y: 10.0, z: 15.0 }, Bool::TRUE),
            Bool::TRUE
        );
        let t = rigid_body_get_translation(world, handle);
        assert!((t.x - 5.0).abs() < 1e-9);
        assert!((t.y - 10.0).abs() < 1e-9);
        assert!((t.z - 15.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn set_translation_rejects_null_world() {
        assert_eq!(
            rigid_body_set_translation(std::ptr::null_mut(), 1, Vec3::default(), Bool::TRUE),
            Bool::FALSE
        );
    }

    #[test]
    fn set_translation_rejects_nan() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_translation(world, handle, Vec3 { x: f64::NAN, y: 0.0, z: 0.0 }, Bool::TRUE),
            Bool::FALSE
        );
        world_destroy(world);
    }

    #[test]
    fn set_rotation_accepts_valid_quat() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        let angle = std::f64::consts::FRAC_PI_2;
        let half = angle * 0.5;
        let q = Quat { i: 0.0, j: 0.0, k: half.sin(), w: half.cos() };
        assert_eq!(rigid_body_set_rotation(world, handle, q, Bool::TRUE), Bool::TRUE);
        world_destroy(world);
    }

    #[test]
    fn set_rotation_rejects_nan() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_rotation(world, handle, Quat { i: f64::NAN, j: 0.0, k: 0.0, w: 1.0 }, Bool::TRUE),
            Bool::FALSE
        );
        world_destroy(world);
    }

    #[test]
    fn set_pose_moves_body_to_position() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_pose(
                world,
                handle,
                Vec3 { x: 1.0, y: 2.0, z: 3.0 },
                Quat { i: 0.0, j: 0.0, k: 0.0, w: 1.0 },
                Bool::TRUE
            ),
            Bool::TRUE
        );
        let t = rigid_body_get_translation(world, handle);
        assert!((t.x - 1.0).abs() < 1e-9);
        assert!((t.y - 2.0).abs() < 1e-9);
        assert!((t.z - 3.0).abs() < 1e-9);
        world_destroy(world);
    }

    // ---- linvel / angvel ----

    #[test]
    fn get_linvel_is_zero_by_default() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        let v = rigid_body_get_linvel(world, handle);
        assert!((v.x - 0.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn set_linvel_updates_velocity() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_linvel(world, handle, Vec3 { x: 1.0, y: 2.0, z: 3.0 }, Bool::TRUE),
            Bool::TRUE
        );
        let v = rigid_body_get_linvel(world, handle);
        assert!((v.x - 1.0).abs() < 1e-9);
        assert!((v.y - 2.0).abs() < 1e-9);
        assert!((v.z - 3.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn set_linvel_rejects_nan() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_linvel(world, handle, Vec3 { x: f64::NAN, y: 0.0, z: 0.0 }, Bool::TRUE),
            Bool::FALSE
        );
        world_destroy(world);
    }

    #[test]
    fn get_angvel_is_zero_by_default() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        let v = rigid_body_get_angvel(world, handle);
        assert!((v.x - 0.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn set_angvel_updates_angular_velocity() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_angvel(world, handle, Vec3 { x: 0.0, y: 1.0, z: 0.0 }, Bool::TRUE),
            Bool::TRUE
        );
        let v = rigid_body_get_angvel(world, handle);
        assert!((v.y - 1.0).abs() < 1e-9);
        world_destroy(world);
    }

    // ---- forces / impulses ----

    #[test]
    fn add_force_on_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_add_force(world, handle, Vec3 { x: 0.0, y: 100.0, z: 0.0 }, Bool::TRUE);
        world_destroy(world);
    }

    #[test]
    fn add_force_rejects_null_world() {
        rigid_body_add_force(std::ptr::null_mut(), 1, Vec3::default(), Bool::TRUE);
    }

    #[test]
    fn add_force_rejects_nan() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_add_force(world, handle, Vec3 { x: f64::NAN, y: 0.0, z: 0.0 }, Bool::TRUE);
        world_destroy(world);
    }

    #[test]
    fn add_torque_on_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_add_torque(world, handle, Vec3 { x: 0.0, y: 0.0, z: 10.0 }, Bool::TRUE);
        world_destroy(world);
    }

    #[test]
    fn apply_impulse_on_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_apply_impulse(world, handle, Vec3 { x: 5.0, y: 0.0, z: 0.0 }, Bool::TRUE);
        world_destroy(world);
    }

    #[test]
    fn apply_torque_impulse_on_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_apply_torque_impulse(world, handle, Vec3 { x: 0.0, y: 1.0, z: 0.0 }, Bool::TRUE);
        world_destroy(world);
    }

    // ---- sleep / wake-up ----

    #[test]
    fn sleep_and_wake_up_roundtrip() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(rigid_body_is_sleeping(world, handle), Bool::FALSE);
        assert_eq!(rigid_body_sleep(world, handle), Bool::TRUE);
        assert_eq!(rigid_body_is_sleeping(world, handle), Bool::TRUE);
        rigid_body_wake_up(world, handle, Bool::TRUE);
        assert_eq!(rigid_body_is_sleeping(world, handle), Bool::FALSE);
        world_destroy(world);
    }

    #[test]
    fn is_sleeping_rejects_null_world() {
        assert_eq!(rigid_body_is_sleeping(std::ptr::null(), 1), Bool::FALSE);
    }

    #[test]
    fn is_sleeping_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(rigid_body_is_sleeping(world, 0), Bool::FALSE);
        world_destroy(world);
    }

    // ---- CCD ----

    #[test]
    fn enable_ccd_on_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_enable_ccd(world, handle, Bool::TRUE);
        world_destroy(world);
    }

    // ---- world_destroy with null is noop ----

    #[test]
    fn world_destroy_null_is_noop() {
        world_destroy(std::ptr::null_mut());
    }
}
