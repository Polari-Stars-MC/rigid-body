use rapier3d::prelude::{
    MultibodyJointHandle as RapierMultibodyJointHandle,
    FixedJointBuilder, GenericJointBuilder, ImpulseJointHandle as RapierImpulseJointHandle,
    JointAxesMask, MotorModel, PrismaticJointBuilder, RevoluteJointBuilder, RopeJointBuilder,
    SphericalJointBuilder, SpringJointBuilder, Vector, WheelJointBuilder,
};

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    Bool, ImpulseJointHandleRaw, JointAxisDesc, JointBuilderHandle, JointTypeDesc,
    RigidBodyHandleRaw, Vec3, WorldHandle, joint_axis_from_raw, joint_axis_to_rapier,
    joint_type_from_raw, pack_impulse_joint_handle, unpack_impulse_joint_handle,
    unpack_rigid_body_handle, vec3_finite, vec3_to_rapier,
};

const EPSILON: f64 = 1.0e-9;

fn valid_axis(axis: Vec3) -> bool {
    vec3_finite(axis) && (axis.x * axis.x + axis.y * axis.y + axis.z * axis.z) > EPSILON
}

pub(crate) enum JointBuilderKind {
    Fixed(FixedJointBuilder),
    Revolute(RevoluteJointBuilder),
    Prismatic(PrismaticJointBuilder),
    Rope(RopeJointBuilder),
    Spring(SpringJointBuilder),
    Spherical(SphericalJointBuilder),
    Wheel(WheelJointBuilder),
}

pub struct GenericJointBuilderHandle {
    inner: GenericJointBuilder,
}

impl JointBuilderKind {
    fn set_contacts_enabled(&mut self, enabled: bool) {
        *self = match std::mem::replace(self, JointBuilderKind::Fixed(FixedJointBuilder::new())) {
            JointBuilderKind::Fixed(builder) => {
                JointBuilderKind::Fixed(builder.contacts_enabled(enabled))
            }
            JointBuilderKind::Revolute(builder) => {
                JointBuilderKind::Revolute(builder.contacts_enabled(enabled))
            }
            JointBuilderKind::Prismatic(builder) => {
                JointBuilderKind::Prismatic(builder.contacts_enabled(enabled))
            }
            JointBuilderKind::Rope(builder) => {
                JointBuilderKind::Rope(builder.contacts_enabled(enabled))
            }
            JointBuilderKind::Spring(builder) => {
                JointBuilderKind::Spring(builder.contacts_enabled(enabled))
            }
            JointBuilderKind::Spherical(builder) => {
                JointBuilderKind::Spherical(builder.contacts_enabled(enabled))
            }
            JointBuilderKind::Wheel(builder) => {
                JointBuilderKind::Wheel(builder.contacts_enabled(enabled))
            }
        };
    }

    fn set_local_anchor1(&mut self, anchor: Vector) {
        *self = match std::mem::replace(self, JointBuilderKind::Fixed(FixedJointBuilder::new())) {
            JointBuilderKind::Fixed(builder) => {
                JointBuilderKind::Fixed(builder.local_anchor1(anchor))
            }
            JointBuilderKind::Revolute(builder) => {
                JointBuilderKind::Revolute(builder.local_anchor1(anchor))
            }
            JointBuilderKind::Prismatic(builder) => {
                JointBuilderKind::Prismatic(builder.local_anchor1(anchor))
            }
            JointBuilderKind::Rope(builder) => {
                JointBuilderKind::Rope(builder.local_anchor1(anchor))
            }
            JointBuilderKind::Spring(builder) => {
                JointBuilderKind::Spring(builder.local_anchor1(anchor))
            }
            JointBuilderKind::Spherical(builder) => {
                JointBuilderKind::Spherical(builder.local_anchor1(anchor))
            }
            JointBuilderKind::Wheel(builder) => {
                JointBuilderKind::Wheel(builder.local_anchor1(anchor))
            }
        };
    }

    fn set_local_anchor2(&mut self, anchor: Vector) {
        *self = match std::mem::replace(self, JointBuilderKind::Fixed(FixedJointBuilder::new())) {
            JointBuilderKind::Fixed(builder) => {
                JointBuilderKind::Fixed(builder.local_anchor2(anchor))
            }
            JointBuilderKind::Revolute(builder) => {
                JointBuilderKind::Revolute(builder.local_anchor2(anchor))
            }
            JointBuilderKind::Prismatic(builder) => {
                JointBuilderKind::Prismatic(builder.local_anchor2(anchor))
            }
            JointBuilderKind::Rope(builder) => {
                JointBuilderKind::Rope(builder.local_anchor2(anchor))
            }
            JointBuilderKind::Spring(builder) => {
                JointBuilderKind::Spring(builder.local_anchor2(anchor))
            }
            JointBuilderKind::Spherical(builder) => {
                JointBuilderKind::Spherical(builder.local_anchor2(anchor))
            }
            JointBuilderKind::Wheel(builder) => {
                JointBuilderKind::Wheel(builder.local_anchor2(anchor))
            }
        };
    }

    fn set_limits(&mut self, axis: JointAxisDesc, min: f64, max: f64) {
        *self = match std::mem::replace(self, JointBuilderKind::Fixed(FixedJointBuilder::new())) {
            JointBuilderKind::Fixed(builder) => JointBuilderKind::Fixed(builder),
            JointBuilderKind::Revolute(builder) => {
                JointBuilderKind::Revolute(builder.limits([min, max]))
            }
            JointBuilderKind::Prismatic(builder) => {
                JointBuilderKind::Prismatic(builder.limits([min, max]))
            }
            JointBuilderKind::Rope(builder) => JointBuilderKind::Rope(builder),
            JointBuilderKind::Spring(builder) => JointBuilderKind::Spring(builder),
            JointBuilderKind::Spherical(builder) => {
                JointBuilderKind::Spherical(builder.limits(joint_axis_to_rapier(axis), [min, max]))
            }
            JointBuilderKind::Wheel(builder) => JointBuilderKind::Wheel(builder),
        };
    }

    fn set_motor_velocity(&mut self, axis: JointAxisDesc, target_vel: f64, factor: f64) {
        *self = match std::mem::replace(self, JointBuilderKind::Fixed(FixedJointBuilder::new())) {
            JointBuilderKind::Fixed(builder) => JointBuilderKind::Fixed(builder),
            JointBuilderKind::Revolute(builder) => {
                JointBuilderKind::Revolute(builder.motor_velocity(target_vel, factor))
            }
            JointBuilderKind::Prismatic(builder) => {
                JointBuilderKind::Prismatic(builder.motor_velocity(target_vel, factor))
            }
            JointBuilderKind::Rope(builder) => {
                JointBuilderKind::Rope(builder.motor_velocity(target_vel, factor))
            }
            JointBuilderKind::Spring(builder) => JointBuilderKind::Spring(builder),
            JointBuilderKind::Spherical(builder) => JointBuilderKind::Spherical(
                builder.motor_velocity(joint_axis_to_rapier(axis), target_vel, factor),
            ),
            JointBuilderKind::Wheel(builder) => JointBuilderKind::Wheel(builder),
        };
    }

    fn set_motor_position(
        &mut self,
        axis: JointAxisDesc,
        target_pos: f64,
        stiffness: f64,
        damping: f64,
    ) {
        *self = match std::mem::replace(self, JointBuilderKind::Fixed(FixedJointBuilder::new())) {
            JointBuilderKind::Fixed(builder) => JointBuilderKind::Fixed(builder),
            JointBuilderKind::Revolute(builder) => {
                JointBuilderKind::Revolute(builder.motor_position(target_pos, stiffness, damping))
            }
            JointBuilderKind::Prismatic(builder) => {
                JointBuilderKind::Prismatic(builder.motor_position(target_pos, stiffness, damping))
            }
            JointBuilderKind::Rope(builder) => {
                JointBuilderKind::Rope(builder.motor_position(target_pos, stiffness, damping))
            }
            JointBuilderKind::Spring(builder) => JointBuilderKind::Spring(builder),
            JointBuilderKind::Spherical(builder) => JointBuilderKind::Spherical(
                builder.motor_position(joint_axis_to_rapier(axis), target_pos, stiffness, damping),
            ),
            JointBuilderKind::Wheel(builder) => JointBuilderKind::Wheel(builder),
        };
    }
}

/// Creates a joint builder of the given type and returns an owned pointer to it.
///
/// # Safety
///
/// No pointers are dereferenced. The returned pointer is owned by the caller and
/// must be released with `joint_builder_destroy` (or consumed by
/// `world_insert_impulse_joint`). Invalid parameters fail with
/// `ERR_INVALID_ARGUMENT` and return null.
#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_create(
    joint_type: u32,
    axis_or_primary: Vec3,
    b: f64,
    c: f64,
) -> *mut JointBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        if !vec3_finite(axis_or_primary) || !b.is_finite() || !c.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "joint builder parameters must be finite",
            );
            return std::ptr::null_mut();
        }
        let joint_type = joint_type_from_raw(joint_type);
        let inner = match joint_type {
            JointTypeDesc::Fixed => JointBuilderKind::Fixed(FixedJointBuilder::new()),
            JointTypeDesc::Revolute => {
                if !valid_axis(axis_or_primary) {
                    set_error(ERR_INVALID_ARGUMENT, "revolute axis must be non-zero");
                    return std::ptr::null_mut();
                }
                JointBuilderKind::Revolute(RevoluteJointBuilder::new(vec3_to_rapier(
                    axis_or_primary,
                )))
            }
            JointTypeDesc::Prismatic => {
                if !valid_axis(axis_or_primary) {
                    set_error(ERR_INVALID_ARGUMENT, "prismatic axis must be non-zero");
                    return std::ptr::null_mut();
                }
                JointBuilderKind::Prismatic(PrismaticJointBuilder::new(vec3_to_rapier(
                    axis_or_primary,
                )))
            }
            JointTypeDesc::Rope => {
                if b < 0.0 {
                    set_error(ERR_INVALID_ARGUMENT, "rope length must be non-negative");
                    return std::ptr::null_mut();
                }
                JointBuilderKind::Rope(RopeJointBuilder::new(b))
            }
            JointTypeDesc::Spring => {
                if b < 0.0 || c < 0.0 {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "spring stiffness and damping must be non-negative",
                    );
                    return std::ptr::null_mut();
                }
                JointBuilderKind::Spring(SpringJointBuilder::new(axis_or_primary.x, b, c))
            }
            JointTypeDesc::Spherical => JointBuilderKind::Spherical(SphericalJointBuilder::new()),
            JointTypeDesc::Wheel => {
                if b < 0.0 || c < 0.0 {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "wheel suspension stiffness and damping must be non-negative",
                    );
                    return std::ptr::null_mut();
                }
                JointBuilderKind::Wheel(WheelJointBuilder::new(axis_or_primary.x, b, c))
            }
        };

        Box::into_raw(Box::new(JointBuilderHandle { inner }))
    })
}

/// # Safety
///
/// `builder` must be a pointer returned by `joint_builder_create` (or null, which is a
/// no-op). Ownership is transferred to Rust and the pointer must not be used after
/// this call.
#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_destroy(builder: *mut JointBuilderHandle) {
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
/// `builder` must be a valid pointer returned by `joint_builder_create` and must
/// remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_set_contacts_enabled(
    builder: *mut JointBuilderHandle,
    enabled: Bool,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "joint builder is null");
            return;
        };
        builder.inner.set_contacts_enabled(enabled.0 != 0);
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by `joint_builder_create` and must
/// remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_set_local_anchor1(builder: *mut JointBuilderHandle, anchor: Vec3) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "joint builder is null");
            return;
        };
        if !vec3_finite(anchor) {
            set_error(ERR_INVALID_ARGUMENT, "anchor must be finite");
            return;
        }
        builder.inner.set_local_anchor1(vec3_to_rapier(anchor));
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by `joint_builder_create` and must
/// remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_set_local_anchor2(builder: *mut JointBuilderHandle, anchor: Vec3) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "joint builder is null");
            return;
        };
        if !vec3_finite(anchor) {
            set_error(ERR_INVALID_ARGUMENT, "anchor must be finite");
            return;
        }
        builder.inner.set_local_anchor2(vec3_to_rapier(anchor));
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by `joint_builder_create` and must
/// remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_set_limits(
    builder: *mut JointBuilderHandle,
    axis: u32,
    min: f64,
    max: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "joint builder is null");
            return;
        };
        if !min.is_finite() || !max.is_finite() || min > max {
            set_error(
                ERR_INVALID_ARGUMENT,
                "limits must be finite with min <= max",
            );
            return;
        }
        builder
            .inner
            .set_limits(joint_axis_from_raw(axis), min, max);
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by `joint_builder_create` and must
/// remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_set_motor_velocity(
    builder: *mut JointBuilderHandle,
    axis: u32,
    target_vel: f64,
    factor: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "joint builder is null");
            return;
        };
        if !target_vel.is_finite() || !factor.is_finite() || factor < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "motor velocity and factor must be finite, factor non-negative",
            );
            return;
        }
        builder
            .inner
            .set_motor_velocity(joint_axis_from_raw(axis), target_vel, factor);
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by `joint_builder_create` and must
/// remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_set_motor_position(
    builder: *mut JointBuilderHandle,
    axis: u32,
    target_pos: f64,
    stiffness: f64,
    damping: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "joint builder is null");
            return;
        };
        if !target_pos.is_finite()
            || !stiffness.is_finite()
            || !damping.is_finite()
            || stiffness < 0.0
            || damping < 0.0
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "motor position parameters must be finite, stiffness/damping non-negative",
            );
            return;
        }
        builder
            .inner
            .set_motor_position(joint_axis_from_raw(axis), target_pos, stiffness, damping);
    })
}

fn build_and_insert(
    world: &mut WorldHandle,
    body1: RigidBodyHandleRaw,
    body2: RigidBodyHandleRaw,
    builder: JointBuilderKind,
    wake_up: bool,
) -> RapierImpulseJointHandle {
    let body1 = unpack_rigid_body_handle(body1);
    let body2 = unpack_rigid_body_handle(body2);
    match builder {
        JointBuilderKind::Fixed(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
        JointBuilderKind::Revolute(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
        JointBuilderKind::Prismatic(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
        JointBuilderKind::Rope(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
        JointBuilderKind::Spring(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
        JointBuilderKind::Spherical(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
        JointBuilderKind::Wheel(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
    }
}

/// # Safety
///
/// `world` must be a valid world pointer. `builder` must be a pointer returned by
/// `joint_builder_create`; on success its ownership is consumed by this call and it
/// must not be used afterwards.
#[unsafe(no_mangle)]
pub extern "C" fn world_insert_impulse_joint(
    world: *mut WorldHandle,
    body1: RigidBodyHandleRaw,
    body2: RigidBodyHandleRaw,
    builder: *mut JointBuilderHandle,
    wake_up: Bool,
) -> ImpulseJointHandleRaw {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        // 裸指针取锁：避免锁守卫的借用与下方 &mut WorldHandle 冲突
        let lock_ptr = &world.inner.query_lock as *const parking_lot::RwLock<()>;
        let _query_lock = unsafe { (*lock_ptr).write() };
        if builder.is_null() {
            set_error(ERR_NULL_POINTER, "joint builder is null");
            return 0;
        }

        let builder = unsafe { Box::from_raw(builder) };
        let JointBuilderHandle { inner: joint } = *builder;
        pack_impulse_joint_handle(build_and_insert(world, body1, body2, joint, wake_up.0 != 0))
    })
}

/// # Safety
///
/// `world` must be a valid world pointer and must remain alive for the duration of
/// the call.
#[unsafe(no_mangle)]
pub extern "C" fn world_remove_impulse_joint(
    world: *mut WorldHandle,
    handle: ImpulseJointHandleRaw,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let _query_lock = world.inner.query_lock.write();

        world
            .inner
            .impulse_joints
            .remove(unpack_impulse_joint_handle(handle), wake_up.0 != 0)
            .is_some()
            .into()
    })
}

/// Create a Rapier GenericJoint with `locked` as a JointAxesMask bitmask.
#[unsafe(no_mangle)]
pub extern "C" fn generic_joint_builder_create(locked: u64) -> *mut GenericJointBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        let mask = JointAxesMask::from_bits_truncate(locked as u8);
        Box::into_raw(Box::new(GenericJointBuilderHandle {
            inner: GenericJointBuilder::new(mask),
        }))
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn generic_joint_builder_destroy(builder: *mut GenericJointBuilderHandle) {
    ffi_guard((), || {
        if !builder.is_null() {
            unsafe {
                drop(Box::from_raw(builder));
            }
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn generic_joint_builder_set_contacts_enabled(
    builder: *mut GenericJointBuilderHandle,
    enabled: Bool,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "generic joint builder is null");
            return;
        };
        builder.inner = builder.inner.contacts_enabled(enabled.0 != 0);
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn generic_joint_builder_set_local_anchor1(
    builder: *mut GenericJointBuilderHandle,
    anchor: Vec3,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "generic joint builder is null");
            return;
        };
        if !vec3_finite(anchor) {
            set_error(ERR_INVALID_ARGUMENT, "anchor must be finite");
            return;
        }
        builder.inner = builder.inner.local_anchor1(vec3_to_rapier(anchor));
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn generic_joint_builder_set_local_anchor2(
    builder: *mut GenericJointBuilderHandle,
    anchor: Vec3,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "generic joint builder is null");
            return;
        };
        if !vec3_finite(anchor) {
            set_error(ERR_INVALID_ARGUMENT, "anchor must be finite");
            return;
        }
        builder.inner = builder.inner.local_anchor2(vec3_to_rapier(anchor));
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn generic_joint_builder_set_limits(
    builder: *mut GenericJointBuilderHandle,
    axis: u32,
    min: f64,
    max: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "generic joint builder is null");
            return;
        };
        if !min.is_finite() || !max.is_finite() || min > max {
            set_error(
                ERR_INVALID_ARGUMENT,
                "limits must be finite with min <= max",
            );
            return;
        }
        let rapier_axis = joint_axis_to_rapier(joint_axis_from_raw(axis));
        builder.inner = builder.inner.limits(rapier_axis, [min, max]);
    })
}

/// ForceBased position motor with stiffness/damping, plus optional max force
/// (call `generic_joint_builder_set_motor_max_force` afterwards).
#[unsafe(no_mangle)]
pub extern "C" fn generic_joint_builder_set_motor_position(
    builder: *mut GenericJointBuilderHandle,
    axis: u32,
    target_pos: f64,
    stiffness: f64,
    damping: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "generic joint builder is null");
            return;
        };
        if !target_pos.is_finite()
            || !stiffness.is_finite()
            || !damping.is_finite()
            || stiffness < 0.0
            || damping < 0.0
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid motor position parameters");
            return;
        }
        let axis = joint_axis_to_rapier(joint_axis_from_raw(axis));
        builder.inner = builder
            .inner
            .motor_position(axis, target_pos, stiffness, damping)
            .motor_model(axis, MotorModel::ForceBased);
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn generic_joint_builder_set_motor_max_force(
    builder: *mut GenericJointBuilderHandle,
    axis: u32,
    max_force: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "generic joint builder is null");
            return;
        };
        if !max_force.is_finite() || max_force < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "max force must be finite and non-negative",
            );
            return;
        }
        builder.inner = builder
            .inner
            .motor_max_force(joint_axis_to_rapier(joint_axis_from_raw(axis)), max_force);
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn world_insert_generic_joint(
    world: *mut WorldHandle,
    body1: RigidBodyHandleRaw,
    body2: RigidBodyHandleRaw,
    builder: *mut GenericJointBuilderHandle,
    wake_up: Bool,
) -> ImpulseJointHandleRaw {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let lock_ptr = &world.inner.query_lock as *const parking_lot::RwLock<()>;
        let _query_lock = unsafe { (*lock_ptr).write() };
        if builder.is_null() {
            set_error(ERR_NULL_POINTER, "generic joint builder is null");
            return 0;
        }
        let builder = unsafe { Box::from_raw(builder) };
        let b1 = unpack_rigid_body_handle(body1);
        let b2 = unpack_rigid_body_handle(body2);
        let handle =
            world
                .inner
                .impulse_joints
                .insert(b1, b2, builder.inner.build(), wake_up.0 != 0);
        pack_impulse_joint_handle(handle)
    })
}

/// Read one joint-space coordinate (rad/m) of an impulse joint.
#[unsafe(no_mangle)]
pub extern "C" fn world_joint_axis_state(
    world: *mut WorldHandle,
    joint: ImpulseJointHandleRaw,
    axis: u32,
    out_angle: *mut f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return crate::rapier::ffi::Bool::FALSE;
        };
        if out_angle.is_null() {
            set_error(ERR_NULL_POINTER, "out_angle is null");
            return crate::rapier::ffi::Bool::FALSE;
        }
        let handle = unpack_impulse_joint_handle(joint);
        let Some(ij) = world.inner.impulse_joints.get(handle) else {
            set_error(ERR_NOT_FOUND, "impulse joint not found");
            return crate::rapier::ffi::Bool::FALSE;
        };
        let b1 = world.inner.bodies.get(ij.body1());
        let b2 = world.inner.bodies.get(ij.body2());
        let (Some(b1), Some(b2)) = (b1, b2) else {
            set_error(ERR_NOT_FOUND, "joint bodies not found");
            return crate::rapier::ffi::Bool::FALSE;
        };
        let rapier_axis = joint_axis_to_rapier(joint_axis_from_raw(axis));
        let value = ij.data.position(b1, b2)[rapier_axis as usize];
        unsafe {
            *out_angle = value;
        }
        clear_error();
        crate::rapier::ffi::Bool::TRUE
    })
}

/// Update a ForceBased position motor at runtime.
#[unsafe(no_mangle)]
pub extern "C" fn world_joint_set_motor(
    world: *mut WorldHandle,
    joint: ImpulseJointHandleRaw,
    axis: u32,
    target: f64,
    stiffness: f64,
    damping: f64,
    max_force: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return crate::rapier::ffi::Bool::FALSE;
        };
        if !target.is_finite()
            || !stiffness.is_finite()
            || !damping.is_finite()
            || !max_force.is_finite()
            || stiffness < 0.0
            || damping < 0.0
            || max_force < 0.0
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid motor parameters");
            return crate::rapier::ffi::Bool::FALSE;
        }
        let handle = unpack_impulse_joint_handle(joint);
        let Some(ij) = world.inner.impulse_joints.get_mut(handle, true) else {
            set_error(ERR_NOT_FOUND, "impulse joint not found");
            return crate::rapier::ffi::Bool::FALSE;
        };
        let rapier_axis = joint_axis_to_rapier(joint_axis_from_raw(axis));
        let i = rapier_axis as usize;
        ij.data.motors[i].model = MotorModel::ForceBased;
        ij.data.motors[i].target_pos = target;
        ij.data.motors[i].stiffness = stiffness;
        ij.data.motors[i].damping = damping;
        ij.data.motors[i].max_force = max_force;
        ij.data.motor_axes |= JointAxesMask::from_bits_truncate(1u8 << i);
        clear_error();
        crate::rapier::ffi::Bool::TRUE
    })
}

/// Batch update ForceBased position motors: one call for many (joint, axis).
#[unsafe(no_mangle)]
pub extern "C" fn world_batch_set_motor(
    world: *mut WorldHandle,
    count: u32,
    joints: *const u64,
    axes: *const u32,
    targets: *const f64,
    stiffness: *const f64,
    damping: *const f64,
    max_forces: *const f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            return Bool::FALSE;
        };
        if count == 0
            || count > 65536
            || joints.is_null()
            || axes.is_null()
            || targets.is_null()
            || stiffness.is_null()
            || damping.is_null()
            || max_forces.is_null()
        {
            return Bool::FALSE;
        }
        let count = count as usize;
        let joints = unsafe { std::slice::from_raw_parts(joints, count) };
        let axes = unsafe { std::slice::from_raw_parts(axes, count) };
        let targets = unsafe { std::slice::from_raw_parts(targets, count) };
        let stiffness = unsafe { std::slice::from_raw_parts(stiffness, count) };
        let damping = unsafe { std::slice::from_raw_parts(damping, count) };
        let max_forces = unsafe { std::slice::from_raw_parts(max_forces, count) };
        for i in 0..count {
            if !targets[i].is_finite()
                || !stiffness[i].is_finite()
                || !damping[i].is_finite()
                || !max_forces[i].is_finite()
                || stiffness[i] < 0.
                || damping[i] < 0.
                || max_forces[i] < 0.
            {
                return Bool::FALSE;
            }
            let handle = unpack_impulse_joint_handle(joints[i]);
            let Some(ij) = world.inner.impulse_joints.get_mut(handle, true) else {
                return Bool::FALSE;
            };
            let rapier_axis = joint_axis_to_rapier(joint_axis_from_raw(axes[i]));
            let idx = rapier_axis as usize;
            ij.data.motors[idx].model = MotorModel::ForceBased;
            ij.data.motors[idx].target_pos = targets[i];
            ij.data.motors[idx].stiffness = stiffness[i];
            ij.data.motors[idx].damping = damping[i];
            ij.data.motors[idx].max_force = max_forces[i];
            ij.data.motor_axes |= JointAxesMask::from_bits_truncate(1u8 << idx);
        }
        Bool::TRUE
    })
}

/// Batch read joint-space coordinates (rad/m) for many (joint, axis) pairs.
#[unsafe(no_mangle)]
pub extern "C" fn world_batch_joint_axis_state(
    world: *const WorldHandle,
    count: u32,
    joints: *const u64,
    axes: *const u32,
    out: *mut f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            return Bool::FALSE;
        };
        if count == 0 || count > 65536 || joints.is_null() || axes.is_null() || out.is_null() {
            return Bool::FALSE;
        }
        let count = count as usize;
        let joints = unsafe { std::slice::from_raw_parts(joints, count) };
        let axes = unsafe { std::slice::from_raw_parts(axes, count) };
        let out = unsafe { std::slice::from_raw_parts_mut(out, count) };
        for i in 0..count {
            let handle = unpack_impulse_joint_handle(joints[i]);
            let Some(ij) = world.inner.impulse_joints.get(handle) else {
                return Bool::FALSE;
            };
            let Some(b1) = world.inner.bodies.get(ij.body1()) else {
                return Bool::FALSE;
            };
            let Some(b2) = world.inner.bodies.get(ij.body2()) else {
                return Bool::FALSE;
            };
            let rapier_axis = joint_axis_to_rapier(joint_axis_from_raw(axes[i]));
            out[i] = ij.data.position(b1, b2)[rapier_axis as usize];
        }
        Bool::TRUE
    })
}

// ======================= 多体关节（multibody joint）=======================
// Rapier 两套关节系统：
//   * impulse joint   —— 迭代约束求解（world_insert_impulse_joint 那一套）
//   * multibody joint —— 多体求解器直接解运动方程（MultibodyJointSet）
// 多体关节对串联机构/马达更稳，这里给 Java 暴露通用入口。

fn pack_multibody_joint_handle(handle: RapierMultibodyJointHandle) -> u64 {
    // +1 偏移：0 留给「失败」（JNI 层的 handle_or_fail 只认 0）
    let (index, generation) = handle.into_raw_parts();
    (((generation as u64) << 32) | (index as u64)) + 1
}

fn unpack_multibody_joint_handle(raw: u64) -> RapierMultibodyJointHandle {
    let value = raw.saturating_sub(1);
    RapierMultibodyJointHandle::from_raw_parts((value & 0xFFFF_FFFF) as u32, (value >> 32) as u32)
}

fn multibody_joint_from_builder(builder: JointBuilderKind) -> rapier3d::prelude::GenericJoint {
    match builder {
        JointBuilderKind::Fixed(b) => b.build().into(),
        JointBuilderKind::Revolute(b) => b.build().into(),
        JointBuilderKind::Prismatic(b) => b.build().into(),
        JointBuilderKind::Rope(b) => b.build().into(),
        JointBuilderKind::Spring(b) => b.build().into(),
        JointBuilderKind::Spherical(b) => b.build().into(),
        JointBuilderKind::Wheel(b) => b.build().into(),
    }
}

/// 用 multibody joint 连接两个刚体（解运动方程那套），返回 handle；`u64::MAX` = 失败。
///
/// # Safety
/// `world` / `builder` 必须是有效指针，调用后 builder 所有权转移给本函数。
#[unsafe(no_mangle)]
pub extern "C" fn world_insert_multibody_joint(
    world: *mut WorldHandle,
    body1: RigidBodyHandleRaw,
    body2: RigidBodyHandleRaw,
    builder: *mut JointBuilderHandle,
    wake_up: Bool,
) -> u64 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let lock_ptr = &world.inner.query_lock as *const parking_lot::RwLock<()>;
        let _query_lock = unsafe { (*lock_ptr).write() };
        if builder.is_null() {
            set_error(ERR_NULL_POINTER, "joint builder is null");
            return 0;
        }
        let builder = unsafe { Box::from_raw(builder) };
        let JointBuilderHandle { inner: joint } = *builder;
        let body1 = unpack_rigid_body_handle(body1);
        let body2 = unpack_rigid_body_handle(body2);
        match world.inner.multibody_joints.insert(
            body1,
            body2,
            multibody_joint_from_builder(joint),
            wake_up.0 != 0,
        ) {
            Some(handle) => {
                clear_error();
                pack_multibody_joint_handle(handle)
            }
            None => {
                set_error(ERR_INVALID_ARGUMENT, "multibody joint rejected");
                0
            }
        }
    })
}

/// 设置 multibody joint 的弹簧马达（目标角度 + 刚度）。
#[unsafe(no_mangle)]
pub extern "C" fn world_multibody_joint_set_motor(
    world: *mut WorldHandle,
    handle: u64,
    axis: u32,
    target: f64,
    stiffness: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !target.is_finite() || !stiffness.is_finite() || stiffness < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid motor parameters");
            return Bool::FALSE;
        }
        let jh = unpack_multibody_joint_handle(handle);
        let Some((mb, link_id)) = world.inner.multibody_joints.get_mut(jh) else {
            set_error(ERR_NOT_FOUND, "multibody joint not found");
            return Bool::FALSE;
        };
        let Some(link) = mb.link_mut(link_id) else {
            set_error(ERR_NOT_FOUND, "multibody link not found");
            return Bool::FALSE;
        };
        let dof = joint_axis_to_rapier(joint_axis_from_raw(axis)) as usize;
        link.joint.set_spring(dof, stiffness, target);
        clear_error();
        Bool::TRUE
    })
}

/// 移除 multibody joint。
#[unsafe(no_mangle)]
pub extern "C" fn world_remove_multibody_joint(
    world: *mut WorldHandle,
    handle: u64,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let jh = unpack_multibody_joint_handle(handle);
        world.inner.multibody_joints.remove(jh, wake_up.0 != 0);
        clear_error();
        Bool::TRUE
    })
}