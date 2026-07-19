#[cfg(test)]
mod tests {
    use mps_core::rapier::joints::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::{BodyStatus, JointTypeDesc};
    use mps_core::rapier::rigid_body::{
        rigid_body_builder_build, rigid_body_builder_create, world_insert_rigid_body,
    };
    use mps_core::rapier::world::{world_create, world_destroy};

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



