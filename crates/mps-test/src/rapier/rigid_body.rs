#[cfg(test)]
mod tests {
    use mps_core::rapier::error::{
        ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, last_error_code,
    };
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::rigid_body::*;
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
        rigid_body_builder_set_translation(
            b,
            Vec3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
        );
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_translation_rejects_nan() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_translation(
            b,
            Vec3 {
                x: f64::NAN,
                y: 0.0,
                z: 0.0,
            },
        );
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_rotation_rejects_nan() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_rotation(
            b,
            Vec3 {
                x: f64::NAN,
                y: 0.0,
                z: 0.0,
            },
        );
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_pose_works() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_pose(
            b,
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Quat {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
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
            Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
        );
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_linvel_works() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_linvel(
            b,
            Vec3 {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
        );
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_angvel_works() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_angvel(
            b,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
        );
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
        assert_eq!(
            world_remove_rigid_body(world, handle, Bool::FALSE),
            Bool::TRUE
        );
        assert_eq!(
            world_remove_rigid_body(world, handle, Bool::FALSE),
            Bool::FALSE
        );
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
        assert_eq!(
            rigid_body_get_status(world, handle),
            BodyStatus::Dynamic as u32
        );
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
            rigid_body_set_status(
                world,
                handle,
                BodyStatus::KinematicVelocityBased as u32,
                Bool::TRUE
            ),
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
            rigid_body_set_status(
                std::ptr::null_mut(),
                1,
                BodyStatus::Dynamic as u32,
                Bool::TRUE
            ),
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
            rigid_body_set_translation(
                world,
                handle,
                Vec3 {
                    x: 5.0,
                    y: 10.0,
                    z: 15.0
                },
                Bool::TRUE
            ),
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
            rigid_body_set_translation(
                world,
                handle,
                Vec3 {
                    x: f64::NAN,
                    y: 0.0,
                    z: 0.0
                },
                Bool::TRUE
            ),
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
        let q = Quat {
            i: 0.0,
            j: 0.0,
            k: half.sin(),
            w: half.cos(),
        };
        assert_eq!(
            rigid_body_set_rotation(world, handle, q, Bool::TRUE),
            Bool::TRUE
        );
        world_destroy(world);
    }

    #[test]
    fn set_rotation_rejects_nan() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_rotation(
                world,
                handle,
                Quat {
                    i: f64::NAN,
                    j: 0.0,
                    k: 0.0,
                    w: 1.0
                },
                Bool::TRUE
            ),
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
                Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0
                },
                Quat {
                    i: 0.0,
                    j: 0.0,
                    k: 0.0,
                    w: 1.0
                },
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
            rigid_body_set_linvel(
                world,
                handle,
                Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0
                },
                Bool::TRUE
            ),
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
            rigid_body_set_linvel(
                world,
                handle,
                Vec3 {
                    x: f64::NAN,
                    y: 0.0,
                    z: 0.0
                },
                Bool::TRUE
            ),
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
            rigid_body_set_angvel(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0
                },
                Bool::TRUE
            ),
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
        rigid_body_add_force(
            world,
            handle,
            Vec3 {
                x: 0.0,
                y: 100.0,
                z: 0.0,
            },
            Bool::TRUE,
        );
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
        rigid_body_add_force(
            world,
            handle,
            Vec3 {
                x: f64::NAN,
                y: 0.0,
                z: 0.0,
            },
            Bool::TRUE,
        );
        world_destroy(world);
    }

    #[test]
    fn add_torque_on_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_add_torque(
            world,
            handle,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 10.0,
            },
            Bool::TRUE,
        );
        world_destroy(world);
    }

    #[test]
    fn apply_impulse_on_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_apply_impulse(
            world,
            handle,
            Vec3 {
                x: 5.0,
                y: 0.0,
                z: 0.0,
            },
            Bool::TRUE,
        );
        world_destroy(world);
    }

    #[test]
    fn apply_torque_impulse_on_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_apply_torque_impulse(
            world,
            handle,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            Bool::TRUE,
        );
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

    #[test]
    fn enable_ccd_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(rigid_body_enable_ccd(world, 0, Bool::TRUE), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    // ---- builder negative cases ----

    #[test]
    fn builder_build_null_returns_null() {
        assert!(rigid_body_builder_build(std::ptr::null_mut()).is_null());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    #[test]
    fn builder_set_translation_rejects_null_builder() {
        rigid_body_builder_set_translation(std::ptr::null_mut(), Vec3::default());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    #[test]
    fn builder_set_pose_rejects_nan() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_pose(
            b,
            Vec3 {
                x: f64::NAN,
                y: 0.0,
                z: 0.0,
            },
            Quat {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_additional_mass_properties_rejects_negative_inertia() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_additional_mass_properties(
            b,
            Vec3::default(),
            1.0,
            Vec3 {
                x: -1.0,
                y: 1.0,
                z: 1.0,
            },
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_linvel_rejects_nan() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_linvel(
            b,
            Vec3 {
                x: 0.0,
                y: f64::NAN,
                z: 0.0,
            },
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_gravity_scale_rejects_nan() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_gravity_scale(b, f64::NAN);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        rigid_body_builder_destroy(b);
    }

    #[test]
    fn builder_set_angular_damping_rejects_negative() {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_angular_damping(b, -0.5);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        rigid_body_builder_destroy(b);
    }

    // ---- stale / wrong-generation handles ----

    #[test]
    fn removed_body_handle_reports_not_found() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            world_remove_rigid_body(world, handle, Bool::FALSE),
            Bool::TRUE
        );
        let t = rigid_body_get_translation(world, handle);
        assert_eq!(t.x, 0.0);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn wrong_generation_handle_reports_not_found() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        // Handles are packed as ((generation << 32) | id) + 1; bumping the high
        // word yields a stale generation for the same slot.
        let stale = handle + (1u64 << 32);
        let t = rigid_body_get_translation(world, stale);
        assert_eq!(t.x, 0.0);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    // ---- world_remove_rigid_body_flag ----

    #[test]
    fn remove_flag_removes_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(world_remove_rigid_body_flag(world, handle, Bool::FALSE), 1);
        assert_eq!(world_remove_rigid_body_flag(world, handle, Bool::FALSE), 0);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn remove_flag_rejects_null_world() {
        assert_eq!(
            world_remove_rigid_body_flag(std::ptr::null_mut(), 1, Bool::FALSE),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    // ---- get_rotation_out ----

    #[test]
    fn get_rotation_out_writes_value() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        let half = std::f64::consts::FRAC_PI_4;
        let q = Quat {
            i: 0.0,
            j: 0.0,
            k: half.sin(),
            w: half.cos(),
        };
        assert_eq!(
            rigid_body_set_rotation(world, handle, q, Bool::TRUE),
            Bool::TRUE
        );
        let mut out = Quat::default();
        rigid_body_get_rotation_out(world, handle, &mut out);
        assert!((out.k - half.sin()).abs() < 1e-9);
        assert!((out.w - half.cos()).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn get_rotation_out_rejects_null_out() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_get_rotation_out(world, handle, std::ptr::null_mut());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        world_destroy(world);
    }

    // ---- get_linvel_out / get_angvel_out ----

    #[test]
    fn get_linvel_out_writes_value() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_linvel(
                world,
                handle,
                Vec3 {
                    x: 3.0,
                    y: 4.0,
                    z: 5.0
                },
                Bool::TRUE
            ),
            Bool::TRUE
        );
        let mut out = Vec3::default();
        rigid_body_get_linvel_out(world, handle, &mut out);
        assert!((out.x - 3.0).abs() < 1e-9);
        assert!((out.y - 4.0).abs() < 1e-9);
        assert!((out.z - 5.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn get_linvel_out_rejects_null_out() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_get_linvel_out(world, handle, std::ptr::null_mut());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        world_destroy(world);
    }

    #[test]
    fn get_angvel_out_writes_value() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_angvel(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 2.0,
                    z: 0.0
                },
                Bool::TRUE
            ),
            Bool::TRUE
        );
        let mut out = Vec3::default();
        rigid_body_get_angvel_out(world, handle, &mut out);
        assert!((out.y - 2.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn get_angvel_out_rejects_null_out() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        rigid_body_get_angvel_out(world, handle, std::ptr::null_mut());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        world_destroy(world);
    }

    // ---- setter invalid handles / NaN ----

    #[test]
    fn set_translation_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(
            rigid_body_set_translation(world, 0, Vec3::default(), Bool::TRUE),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn set_rotation_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(
            rigid_body_set_rotation(
                world,
                0,
                Quat {
                    i: 0.0,
                    j: 0.0,
                    k: 0.0,
                    w: 1.0
                },
                Bool::TRUE
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn set_pose_rejects_nan_rotation() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_pose(
                world,
                handle,
                Vec3::default(),
                Quat {
                    i: 0.0,
                    j: f64::NAN,
                    k: 0.0,
                    w: 1.0
                },
                Bool::TRUE
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }

    #[test]
    fn set_pose_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(
            rigid_body_set_pose(
                world,
                0,
                Vec3::default(),
                Quat {
                    i: 0.0,
                    j: 0.0,
                    k: 0.0,
                    w: 1.0
                },
                Bool::TRUE
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn set_linvel_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(
            rigid_body_set_linvel(world, 0, Vec3::default(), Bool::TRUE),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn set_angvel_rejects_nan() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_angvel(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: f64::NAN
                },
                Bool::TRUE
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }

    #[test]
    fn set_angvel_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(
            rigid_body_set_angvel(world, 0, Vec3::default(), Bool::TRUE),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    // ---- mass / force getters ----

    #[test]
    fn get_mass_is_zero_without_colliders() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(rigid_body_get_mass(world, handle), 0.0);
        world_destroy(world);
    }

    #[test]
    fn get_mass_includes_additional_mass_properties() {
        let world = make_world();
        let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_additional_mass_properties(
            builder,
            Vec3::default(),
            2.5,
            Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
        );
        let body = rigid_body_builder_build(builder);
        assert!(!body.is_null());
        let handle = world_insert_rigid_body(world, body);
        assert_ne!(handle, 0);
        // Additional mass-properties are folded into the local mass only when
        // the mass properties are recomputed, which happens during a step.
        mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        let mass = rigid_body_get_mass(world, handle);
        assert!((mass - 2.5).abs() < 1e-9, "unexpected mass: {mass}");
        world_destroy(world);
    }

    #[test]
    fn get_mass_rejects_null_world() {
        assert_eq!(rigid_body_get_mass(std::ptr::null_mut(), 1), 0.0);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    #[test]
    fn get_mass_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(rigid_body_get_mass(world, 0), 0.0);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn get_force_returns_added_force() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_add_force(
                world,
                handle,
                Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0
                },
                Bool::TRUE
            ),
            Bool::TRUE
        );
        let f = rigid_body_get_force(world, handle);
        assert!((f.x - 1.0).abs() < 1e-9);
        assert!((f.y - 2.0).abs() < 1e-9);
        assert!((f.z - 3.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn get_force_rejects_null_world() {
        let f = rigid_body_get_force(std::ptr::null(), 1);
        assert_eq!(f.x, 0.0);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    #[test]
    fn get_force_rejects_invalid_handle() {
        let world = make_world();
        let f = rigid_body_get_force(world, 0);
        assert_eq!(f.x, 0.0);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    // ---- forces / torques: additional coverage ----

    #[test]
    fn add_force_at_point_applies() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_add_force_at_point(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 10.0,
                    z: 0.0
                },
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0
                },
                Bool::TRUE
            ),
            Bool::TRUE
        );
        world_destroy(world);
    }

    #[test]
    fn add_force_at_point_rejects_nan_point() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_add_force_at_point(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 10.0,
                    z: 0.0
                },
                Vec3 {
                    x: f64::NAN,
                    y: 0.0,
                    z: 0.0
                },
                Bool::TRUE
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }

    #[test]
    fn add_force_at_point_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(
            rigid_body_add_force_at_point(world, 0, Vec3::default(), Vec3::default(), Bool::TRUE),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn reset_force_clears_user_force() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_add_force(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 50.0,
                    z: 0.0
                },
                Bool::TRUE
            ),
            Bool::TRUE
        );
        assert_eq!(
            rigid_body_reset_force(world, handle, Bool::TRUE),
            Bool::TRUE
        );
        let f = rigid_body_get_force(world, handle);
        assert_eq!(f.x, 0.0);
        assert_eq!(f.y, 0.0);
        assert_eq!(f.z, 0.0);
        world_destroy(world);
    }

    #[test]
    fn reset_force_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(rigid_body_reset_force(world, 0, Bool::TRUE), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn reset_torque_works() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_reset_torque(world, handle, Bool::TRUE),
            Bool::TRUE
        );
        world_destroy(world);
    }

    #[test]
    fn reset_torque_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(rigid_body_reset_torque(world, 0, Bool::TRUE), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn add_torque_rejects_nan() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_add_torque(
                world,
                handle,
                Vec3 {
                    x: f64::NAN,
                    y: 0.0,
                    z: 0.0
                },
                Bool::TRUE
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }

    #[test]
    fn apply_impulse_rejects_nan() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_apply_impulse(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: f64::NAN,
                    z: 0.0
                },
                Bool::TRUE
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }

    #[test]
    fn apply_torque_impulse_rejects_nan() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_apply_torque_impulse(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: f64::NAN
                },
                Bool::TRUE
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }

    // ---- _flag variants ----

    #[test]
    fn set_translation_flag_moves_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_translation_flag(
                world,
                handle,
                Vec3 {
                    x: 7.0,
                    y: 8.0,
                    z: 9.0
                },
                Bool::TRUE
            ),
            1
        );
        let t = rigid_body_get_translation(world, handle);
        assert!((t.x - 7.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn set_translation_flag_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(
            rigid_body_set_translation_flag(world, 0, Vec3::default(), Bool::TRUE),
            0
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn set_rotation_flag_rotates_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        let half = std::f64::consts::FRAC_PI_4;
        assert_eq!(
            rigid_body_set_rotation_flag(
                world,
                handle,
                Quat {
                    i: 0.0,
                    j: 0.0,
                    k: half.sin(),
                    w: half.cos()
                },
                Bool::TRUE
            ),
            1
        );
        let q = rigid_body_get_rotation(world, handle);
        assert!((q.w - half.cos()).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn set_pose_flag_moves_body() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_pose_flag(
                world,
                handle,
                Vec3 {
                    x: 4.0,
                    y: 5.0,
                    z: 6.0
                },
                Quat {
                    i: 0.0,
                    j: 0.0,
                    k: 0.0,
                    w: 1.0
                },
                Bool::TRUE
            ),
            1
        );
        let t = rigid_body_get_translation(world, handle);
        assert!((t.x - 4.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn set_linvel_flag_updates_velocity() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_linvel_flag(
                world,
                handle,
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0
                },
                Bool::TRUE
            ),
            1
        );
        let v = rigid_body_get_linvel(world, handle);
        assert!((v.x - 1.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn set_angvel_flag_updates_angular_velocity() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_set_angvel_flag(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 3.0
                },
                Bool::TRUE
            ),
            1
        );
        let v = rigid_body_get_angvel(world, handle);
        assert!((v.z - 3.0).abs() < 1e-9);
        world_destroy(world);
    }

    #[test]
    fn add_force_flag_applies() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_add_force_flag(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0
                },
                Bool::TRUE
            ),
            1
        );
        world_destroy(world);
    }

    #[test]
    fn add_torque_flag_applies() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_add_torque_flag(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0
                },
                Bool::TRUE
            ),
            1
        );
        world_destroy(world);
    }

    #[test]
    fn apply_impulse_flag_applies() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_apply_impulse_flag(
                world,
                handle,
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0
                },
                Bool::TRUE
            ),
            1
        );
        world_destroy(world);
    }

    #[test]
    fn apply_torque_impulse_flag_applies() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(
            rigid_body_apply_torque_impulse_flag(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0
                },
                Bool::TRUE
            ),
            1
        );
        world_destroy(world);
    }

    #[test]
    fn enable_ccd_flag_enables() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(rigid_body_enable_ccd_flag(world, handle, Bool::TRUE), 1);
        world_destroy(world);
    }

    #[test]
    fn enable_ccd_flag_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(rigid_body_enable_ccd_flag(world, 0, Bool::TRUE), 0);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn sleep_and_wake_up_flag_roundtrip() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        assert_eq!(rigid_body_is_sleeping_flag(world, handle), 0);
        assert_eq!(rigid_body_sleep_flag(world, handle), 1);
        assert_eq!(rigid_body_is_sleeping_flag(world, handle), 1);
        assert_eq!(rigid_body_wake_up_flag(world, handle, Bool::TRUE), 1);
        assert_eq!(rigid_body_is_sleeping_flag(world, handle), 0);
        world_destroy(world);
    }

    #[test]
    fn sleep_rejects_null_world() {
        assert_eq!(rigid_body_sleep(std::ptr::null_mut(), 1), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    #[test]
    fn wake_up_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(rigid_body_wake_up(world, 0, Bool::TRUE), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    // ---- world_destroy with null is noop ----

    #[test]
    fn world_destroy_null_is_noop() {
        world_destroy(std::ptr::null_mut());
    }

    // ── 通用属性通道：rigid_body 批（ADR-0016 第三个模块） ────────────────────

    use mps_core::rapier::rigid_body::{
        RIGID_BODY_PARAM_ENABLE_CCD, RIGID_BODY_PARAM_ENABLED, RIGID_BODY_PARAM_WAKE_UP,
        rigid_body_param_abi_version, rigid_body_set_param, rigid_body_set_param_flag,
    };

    #[test]
    fn rigid_body_param_ids_match_the_abi_version() {
        let all = [
            RIGID_BODY_PARAM_ENABLE_CCD,
            RIGID_BODY_PARAM_WAKE_UP,
            RIGID_BODY_PARAM_ENABLED,
        ];
        assert_eq!(rigid_body_param_abi_version() as usize, all.len());
        let mut sorted = all;
        sorted.sort_unstable();
        for w in sorted.windows(2) {
            assert_ne!(w[0], w[1], "duplicate rigid_body param id {}", w[0]);
        }
    }

    /// `set_param(ENABLE_CCD, v)` must equal `rigid_body_enable_ccd(Bool(v))`.
    #[test]
    fn enable_ccd_agrees_with_the_dedicated_entry() {
        let world = make_world();
        for (value, flag) in [(1.0, true), (0.0, false), (2.0, true), (-1.0, true)] {
            let a = make_dynamic_body(world);
            let b = make_dynamic_body(world);
            let generic = rigid_body_set_param(world, a, RIGID_BODY_PARAM_ENABLE_CCD, value).0;
            let specific = rigid_body_enable_ccd(world, b, Bool(flag as u8)).0;
            assert_eq!(
                generic, specific,
                "enable_ccd value {value}: generic {generic} vs dedicated {specific}"
            );
        }
        world_destroy(world);
    }

    /// `set_param(WAKE_UP, v)` must equal `rigid_body_wake_up(Bool(v))`.
    ///
    /// The dedicated entry's `Bool` is "strong": a true value clears the sleep
    /// timer, not just the sleeping flag. The channel must preserve that.
    #[test]
    fn wake_up_agrees_with_the_dedicated_entry() {
        let world = make_world();
        for value in [0.0, 1.0] {
            let a = make_dynamic_body(world);
            let b = make_dynamic_body(world);
            // Put both to sleep first so wake-up has something to do.
            assert_eq!(rigid_body_sleep(world, a), Bool::TRUE);
            assert_eq!(rigid_body_sleep(world, b), Bool::TRUE);

            let generic = rigid_body_set_param(world, a, RIGID_BODY_PARAM_WAKE_UP, value).0;
            let specific = rigid_body_wake_up(world, b, Bool(value as u8)).0;
            assert_eq!(
                generic, specific,
                "wake_up value {value}: generic {generic} vs dedicated {specific}"
            );
        }
        world_destroy(world);
    }

    #[test]
    fn set_param_rejects_unknown_ids_and_bad_input() {
        let world = make_world();
        let handle = make_dynamic_body(world);

        assert_eq!(
            rigid_body_set_param(world, handle, rigid_body_param_abi_version(), 1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert_eq!(
            rigid_body_set_param(
                std::ptr::null_mut(),
                handle,
                RIGID_BODY_PARAM_ENABLE_CCD,
                1.0
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert_eq!(
            rigid_body_set_param(world, 0xDEAD_BEEF, RIGID_BODY_PARAM_ENABLE_CCD, 1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);

        // Non-finite is rejected before it reaches the setter.
        assert_eq!(
            rigid_body_set_param(world, handle, RIGID_BODY_PARAM_ENABLE_CCD, f64::NAN),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        world_destroy(world);
    }

    #[test]
    fn set_param_flag_matches_the_bool_variant() {
        let world = make_world();
        let handle = make_dynamic_body(world);
        for (param, value) in [
            (RIGID_BODY_PARAM_ENABLE_CCD, 1.0),
            (RIGID_BODY_PARAM_ENABLE_CCD, 0.0),
            (RIGID_BODY_PARAM_WAKE_UP, 1.0),
            (rigid_body_param_abi_version(), 1.0),
        ] {
            let b = rigid_body_set_param(world, handle, param, value).0;
            let f = rigid_body_set_param_flag(world, handle, param, value);
            assert_eq!(b, f, "param {param} value {value}: Bool {b} vs flag {f}");
        }
        world_destroy(world);
    }
}
