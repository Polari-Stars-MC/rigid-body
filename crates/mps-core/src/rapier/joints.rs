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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::{BodyStatus, JointTypeDesc};
    use crate::rapier::rigid_body::{
        rigid_body_builder_build, rigid_body_builder_create, world_insert_rigid_body,
    };
    use crate::rapier::world::{world_create, world_destroy};

    fn make_world() -> *mut WorldHandle {
        world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        })
    }

    fn make_dynamic_body(world: *mut WorldHandle) -> RigidBodyHandleRaw {
        let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        let body = rigid_body_builder_build(builder);
        let handle = world_insert_rigid_body(world, body);
        assert_ne!(handle, 0);
        handle
    }

    // ---- builder create / destroy ----

    #[test]
    fn create_fixed_joint() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        assert!(!b.is_null());
        joint_builder_destroy(b);
    }

    #[test]
    fn create_revolute_joint() {
        let b = joint_builder_create(
            JointTypeDesc::Revolute as u32,
            Vec3 { x: 0.0, y: 1.0, z: 0.0 },
            0.0,
            0.0,
        );
        assert!(!b.is_null());
        joint_builder_destroy(b);
    }

    #[test]
    fn create_revolute_rejects_zero_axis() {
        let b = joint_builder_create(JointTypeDesc::Revolute as u32, Vec3::default(), 0.0, 0.0);
        assert!(b.is_null());
    }

    #[test]
    fn create_prismatic_joint() {
        let b = joint_builder_create(
            JointTypeDesc::Prismatic as u32,
            Vec3 { x: 1.0, y: 0.0, z: 0.0 },
            0.0,
            0.0,
        );
        assert!(!b.is_null());
        joint_builder_destroy(b);
    }

    #[test]
    fn create_rope_joint() {
        let b = joint_builder_create(JointTypeDesc::Rope as u32, Vec3::default(), 3.0, 0.0);
        assert!(!b.is_null());
        joint_builder_destroy(b);
    }

    #[test]
    fn create_rope_rejects_negative_length() {
        let b = joint_builder_create(JointTypeDesc::Rope as u32, Vec3::default(), -1.0, 0.0);
        assert!(b.is_null());
    }

    #[test]
    fn create_spring_joint() {
        let b = joint_builder_create(
            JointTypeDesc::Spring as u32,
            Vec3 { x: 10.0, y: 0.0, z: 0.0 },
            1.0,
            0.5,
        );
        assert!(!b.is_null());
        joint_builder_destroy(b);
    }

    #[test]
    fn create_spring_rejects_negative_params() {
        let b = joint_builder_create(
            JointTypeDesc::Spring as u32,
            Vec3::default(),
            -1.0,
            0.5,
        );
        assert!(b.is_null());
    }

    #[test]
    fn create_spherical_joint() {
        let b = joint_builder_create(JointTypeDesc::Spherical as u32, Vec3::default(), 0.0, 0.0);
        assert!(!b.is_null());
        joint_builder_destroy(b);
    }

    #[test]
    fn destroy_null_builder_is_noop() {
        joint_builder_destroy(std::ptr::null_mut());
    }

    // ---- builder options ----

    #[test]
    fn set_contacts_enabled() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        joint_builder_set_contacts_enabled(b, Bool::TRUE);
        joint_builder_set_contacts_enabled(b, Bool::FALSE);
        joint_builder_destroy(b);
    }

    #[test]
    fn set_local_anchors() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        joint_builder_set_local_anchor1(b, Vec3 { x: 0.0, y: 0.5, z: 0.0 });
        joint_builder_set_local_anchor2(b, Vec3 { x: 0.0, y: -0.5, z: 0.0 });
        joint_builder_destroy(b);
    }

    #[test]
    fn set_local_anchor1_rejects_nan() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        joint_builder_set_local_anchor1(b, Vec3 { x: f64::NAN, y: 0.0, z: 0.0 });
        joint_builder_destroy(b);
    }

    #[test]
    fn set_limits() {
        let b = joint_builder_create(
            JointTypeDesc::Revolute as u32,
            Vec3 { x: 0.0, y: 1.0, z: 0.0 },
            0.0,
            0.0,
        );
        joint_builder_set_limits(b, JointAxisDesc::AngX as u32, -1.0, 1.0);
        joint_builder_destroy(b);
    }

    #[test]
    fn set_limits_rejects_inverted() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        joint_builder_set_limits(b, JointAxisDesc::LinX as u32, 1.0, 0.0); // min > max
        joint_builder_destroy(b);
    }

    #[test]
    fn set_motor_velocity() {
        let b = joint_builder_create(
            JointTypeDesc::Revolute as u32,
            Vec3 { x: 0.0, y: 1.0, z: 0.0 },
            0.0,
            0.0,
        );
        joint_builder_set_motor_velocity(b, JointAxisDesc::AngX as u32, 2.0, 0.5);
        joint_builder_destroy(b);
    }

    #[test]
    fn set_motor_velocity_rejects_negative_factor() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        joint_builder_set_motor_velocity(b, JointAxisDesc::LinX as u32, 2.0, -0.5);
        joint_builder_destroy(b);
    }

    #[test]
    fn set_motor_position() {
        let b = joint_builder_create(
            JointTypeDesc::Revolute as u32,
            Vec3 { x: 0.0, y: 1.0, z: 0.0 },
            0.0,
            0.0,
        );
        joint_builder_set_motor_position(b, JointAxisDesc::AngX as u32, 1.0, 100.0, 10.0);
        joint_builder_destroy(b);
    }

    #[test]
    fn set_motor_position_rejects_negative_stiffness() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        joint_builder_set_motor_position(b, JointAxisDesc::LinX as u32, 1.0, -100.0, 10.0);
        joint_builder_destroy(b);
    }

    // ---- world insert / remove ----

    #[test]
    fn insert_and_remove_fixed_joint() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        let builder = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        let handle = world_insert_impulse_joint(world, b1, b2, builder, Bool::TRUE);
        assert_ne!(handle, 0);
        assert_eq!(world_remove_impulse_joint(world, handle, Bool::TRUE), Bool::TRUE);
        world_destroy(world);
    }

    #[test]
    fn insert_and_remove_rope_joint() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        let builder = joint_builder_create(JointTypeDesc::Rope as u32, Vec3::default(), 2.0, 0.0);
        let handle = world_insert_impulse_joint(world, b1, b2, builder, Bool::TRUE);
        assert_ne!(handle, 0);
        assert_eq!(world_remove_impulse_joint(world, handle, Bool::FALSE), Bool::TRUE);
        // second remove fails
        assert_eq!(world_remove_impulse_joint(world, handle, Bool::FALSE), Bool::FALSE);
        world_destroy(world);
    }

    #[test]
    fn insert_rejects_null_world() {
        let builder = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        assert_eq!(
            world_insert_impulse_joint(std::ptr::null_mut(), 1, 2, builder, Bool::TRUE),
            0
        );
        // builder consumed
    }

    #[test]
    fn insert_rejects_null_builder() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        assert_eq!(
            world_insert_impulse_joint(world, b1, b2, std::ptr::null_mut(), Bool::TRUE),
            0
        );
        world_destroy(world);
    }

    #[test]
    fn remove_rejects_null_world() {
        assert_eq!(world_remove_impulse_joint(std::ptr::null_mut(), 1, Bool::TRUE), Bool::FALSE);
    }

    #[test]
    fn remove_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(world_remove_impulse_joint(world, 0, Bool::TRUE), Bool::FALSE);
        world_destroy(world);
    }

    // ---- all joint types round-trip ----

    #[test]
    fn revolute_joint_roundtrip() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        let builder = joint_builder_create(
            JointTypeDesc::Revolute as u32,
            Vec3 { x: 0.0, y: 1.0, z: 0.0 },
            0.0,
            0.0,
        );
        joint_builder_set_local_anchor1(builder, Vec3 { x: 0.0, y: 0.5, z: 0.0 });
        joint_builder_set_local_anchor2(builder, Vec3 { x: 0.0, y: -0.5, z: 0.0 });
        let handle = world_insert_impulse_joint(world, b1, b2, builder, Bool::TRUE);
        assert_ne!(handle, 0);
        assert_eq!(world_remove_impulse_joint(world, handle, Bool::TRUE), Bool::TRUE);
        world_destroy(world);
    }

    #[test]
    fn prismatic_joint_roundtrip() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        let builder = joint_builder_create(
            JointTypeDesc::Prismatic as u32,
            Vec3 { x: 1.0, y: 0.0, z: 0.0 },
            0.0,
            0.0,
        );
        let handle = world_insert_impulse_joint(world, b1, b2, builder, Bool::TRUE);
        assert_ne!(handle, 0);
        assert_eq!(world_remove_impulse_joint(world, handle, Bool::TRUE), Bool::TRUE);
        world_destroy(world);
    }

    #[test]
    fn spherical_joint_roundtrip() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        let builder = joint_builder_create(JointTypeDesc::Spherical as u32, Vec3::default(), 0.0, 0.0);
        let handle = world_insert_impulse_joint(world, b1, b2, builder, Bool::TRUE);
        assert_ne!(handle, 0);
        assert_eq!(world_remove_impulse_joint(world, handle, Bool::TRUE), Bool::TRUE);
        world_destroy(world);
    }

    #[test]
    fn spring_joint_roundtrip() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        let builder = joint_builder_create(
            JointTypeDesc::Spring as u32,
            Vec3 { x: 0.0, y: 50.0, z: 0.0 },
            0.5,
            0.1,
        );
        let handle = world_insert_impulse_joint(world, b1, b2, builder, Bool::TRUE);
        assert_ne!(handle, 0);
        assert_eq!(world_remove_impulse_joint(world, handle, Bool::TRUE), Bool::TRUE);
        world_destroy(world);
    }
}
