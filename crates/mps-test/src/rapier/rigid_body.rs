#[cfg(test)]
mod tests {
    use smallvec::SmallVec;
    use mps_core::rapier::rigid_body::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::world::world_create;
    use mps_core::rapier::world::world_destroy;

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







