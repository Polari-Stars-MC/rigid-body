use rapier3d::prelude::{
    FixedJointBuilder, ImpulseJointHandle as RapierImpulseJointHandle, PrismaticJointBuilder,
    RevoluteJointBuilder, RopeJointBuilder, SphericalJointBuilder, SpringJointBuilder, Vector,
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
        };
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_create(
    joint_type: u32,
    axis_or_primary: Vec3,
    b: f64,
    c: f64,
) -> *mut JointBuilderHandle {
    if !vec3_finite(axis_or_primary) || !b.is_finite() || !c.is_finite() {
        return std::ptr::null_mut();
    }
    let joint_type = joint_type_from_raw(joint_type);
    let inner = match joint_type {
        JointTypeDesc::Fixed => JointBuilderKind::Fixed(FixedJointBuilder::new()),
        JointTypeDesc::Revolute => {
            if !valid_axis(axis_or_primary) {
                return std::ptr::null_mut();
            }
            JointBuilderKind::Revolute(RevoluteJointBuilder::new(vec3_to_rapier(axis_or_primary)))
        }
        JointTypeDesc::Prismatic => {
            if !valid_axis(axis_or_primary) {
                return std::ptr::null_mut();
            }
            JointBuilderKind::Prismatic(PrismaticJointBuilder::new(vec3_to_rapier(axis_or_primary)))
        }
        JointTypeDesc::Rope => {
            if b < 0.0 {
                return std::ptr::null_mut();
            }
            JointBuilderKind::Rope(RopeJointBuilder::new(b))
        }
        JointTypeDesc::Spring => {
            if b < 0.0 || c < 0.0 {
                return std::ptr::null_mut();
            }
            JointBuilderKind::Spring(SpringJointBuilder::new(axis_or_primary.x, b, c))
        }
        JointTypeDesc::Spherical => JointBuilderKind::Spherical(SphericalJointBuilder::new()),
    };

    Box::into_raw(Box::new(JointBuilderHandle { inner }))
}

#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_destroy(builder: *mut JointBuilderHandle) {
    if builder.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(builder));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_set_contacts_enabled(
    builder: *mut JointBuilderHandle,
    enabled: Bool,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    builder.inner.set_contacts_enabled(enabled.0 != 0);
}

#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_set_local_anchor1(builder: *mut JointBuilderHandle, anchor: Vec3) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !vec3_finite(anchor) {
        return;
    }
    builder.inner.set_local_anchor1(vec3_to_rapier(anchor));
}

#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_set_local_anchor2(builder: *mut JointBuilderHandle, anchor: Vec3) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !vec3_finite(anchor) {
        return;
    }
    builder.inner.set_local_anchor2(vec3_to_rapier(anchor));
}

#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_set_limits(
    builder: *mut JointBuilderHandle,
    axis: u32,
    min: f64,
    max: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !min.is_finite() || !max.is_finite() || min > max {
        return;
    }
    builder
        .inner
        .set_limits(joint_axis_from_raw(axis), min, max);
}

#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_set_motor_velocity(
    builder: *mut JointBuilderHandle,
    axis: u32,
    target_vel: f64,
    factor: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !target_vel.is_finite() || !factor.is_finite() || factor < 0.0 {
        return;
    }
    builder
        .inner
        .set_motor_velocity(joint_axis_from_raw(axis), target_vel, factor);
}

#[unsafe(no_mangle)]
pub extern "C" fn joint_builder_set_motor_position(
    builder: *mut JointBuilderHandle,
    axis: u32,
    target_pos: f64,
    stiffness: f64,
    damping: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !target_pos.is_finite()
        || !stiffness.is_finite()
        || !damping.is_finite()
        || stiffness < 0.0
        || damping < 0.0
    {
        return;
    }
    builder
        .inner
        .set_motor_position(joint_axis_from_raw(axis), target_pos, stiffness, damping);
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
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn world_insert_impulse_joint(
    world: *mut WorldHandle,
    body1: RigidBodyHandleRaw,
    body2: RigidBodyHandleRaw,
    builder: *mut JointBuilderHandle,
    wake_up: Bool,
) -> ImpulseJointHandleRaw {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0;
    };
    if builder.is_null() {
        return 0;
    }

    let builder = unsafe { Box::from_raw(builder) };
    let JointBuilderHandle { inner: joint } = *builder;
    pack_impulse_joint_handle(build_and_insert(world, body1, body2, joint, wake_up.0 != 0))
}

#[unsafe(no_mangle)]
pub extern "C" fn world_remove_impulse_joint(
    world: *mut WorldHandle,
    handle: ImpulseJointHandleRaw,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };

    world
        .inner
        .impulse_joints
        .remove(unpack_impulse_joint_handle(handle), wake_up.0 != 0)
        .is_some()
        .into()
}


