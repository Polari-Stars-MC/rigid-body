#[cfg(test)]
mod tests {
    use mps_core::rapier::controller::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::{BodyStatus, RigidBodyHandleRaw, ShapeDesc, ShapeType};
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

    fn cuboid_shape(hx: f64, hy: f64, hz: f64) -> ShapeDesc {
        ShapeDesc {
            shape_type: ShapeType::Cuboid as u32,
            a: hx,
            b: hy,
            c: hz,
            d: 0.0,
        }
    }

    // ---- create / destroy ----

    #[test]
    fn create_and_destroy() {
        let c = character_controller_create();
        assert!(!c.is_null());
        character_controller_destroy(c);
    }

    #[test]
    fn destroy_null_is_noop() {
        character_controller_destroy(std::ptr::null_mut());
    }

    // ---- set_up ----

    #[test]
    fn set_up_rejects_null() {
        character_controller_set_up(std::ptr::null_mut(), Vec3 { x: 0.0, y: 1.0, z: 0.0 });
    }

    #[test]
    fn set_up_rejects_nan() {
        let c = character_controller_create();
        character_controller_set_up(c, Vec3 { x: f64::NAN, y: 0.0, z: 0.0 });
        character_controller_destroy(c);
    }

    // ---- set_offset ----

    #[test]
    fn set_offset_absolute() {
        let c = character_controller_create();
        character_controller_set_offset_absolute(c, 0.5);
        character_controller_destroy(c);
    }

    #[test]
    fn set_offset_absolute_rejects_negative() {
        let c = character_controller_create();
        character_controller_set_offset_absolute(c, -0.1);
        character_controller_destroy(c);
    }

    #[test]
    fn set_offset_relative() {
        let c = character_controller_create();
        character_controller_set_offset_relative(c, 0.2);
        character_controller_destroy(c);
    }

    #[test]
    fn set_offset_relative_rejects_negative() {
        let c = character_controller_create();
        character_controller_set_offset_relative(c, -0.1);
        character_controller_destroy(c);
    }

    // ---- set_slide ----

    #[test]
    fn set_slide() {
        let c = character_controller_create();
        character_controller_set_slide(c, Bool::TRUE);
        character_controller_set_slide(c, Bool::FALSE);
        character_controller_destroy(c);
    }

    // ---- set_autostep ----

    #[test]
    fn set_autostep_enabled() {
        let c = character_controller_create();
        character_controller_set_autostep(c, Bool::TRUE, 0.5, 0.2, Bool::FALSE);
        character_controller_destroy(c);
    }

    #[test]
    fn set_autostep_disabled() {
        let c = character_controller_create();
        character_controller_set_autostep(c, Bool::FALSE, 0.5, 0.2, Bool::FALSE);
        character_controller_destroy(c);
    }

    #[test]
    fn set_autostep_rejects_negative_height() {
        let c = character_controller_create();
        character_controller_set_autostep(c, Bool::TRUE, -0.5, 0.2, Bool::FALSE);
        character_controller_destroy(c);
    }

    // ---- set_snap_to_ground ----

    #[test]
    fn set_snap_to_ground() {
        let c = character_controller_create();
        character_controller_set_snap_to_ground(c, Bool::TRUE, 0.3);
        character_controller_set_snap_to_ground(c, Bool::FALSE, 0.0);
        character_controller_destroy(c);
    }

    #[test]
    fn set_snap_to_ground_rejects_negative() {
        let c = character_controller_create();
        character_controller_set_snap_to_ground(c, Bool::TRUE, -0.3);
        character_controller_destroy(c);
    }

    // ---- set_slope_angles ----

    #[test]
    fn set_slope_angles() {
        let c = character_controller_create();
        character_controller_set_slope_angles(c, 0.8, 0.3);
        character_controller_destroy(c);
    }

    #[test]
    fn set_slope_angles_rejects_nan() {
        let c = character_controller_create();
        character_controller_set_slope_angles(c, f64::NAN, 0.3);
        character_controller_destroy(c);
    }

    // ---- move_shape ----

    #[test]
    fn move_shape_rejects_null_world() {
        let c = character_controller_create();
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        let result = character_controller_move_shape(
            std::ptr::null(),
            c,
            1.0 / 60.0,
            shape,
            Vec3 { x: 0.0, y: 1.0, z: 0.0 },
            Quat { i: 0.0, j: 0.0, k: 0.0, w: 1.0 },
            Vec3 { x: 0.0, y: 0.0, z: 1.0 },
        );
        assert_eq!(result.translation.x, 0.0);
        character_controller_destroy(c);
    }

    #[test]
    fn move_shape_rejects_null_controller() {
        let world = make_world();
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        let result = character_controller_move_shape(
            world,
            std::ptr::null_mut(),
            1.0 / 60.0,
            shape,
            Vec3 { x: 0.0, y: 1.0, z: 0.0 },
            Quat { i: 0.0, j: 0.0, k: 0.0, w: 1.0 },
            Vec3 { x: 0.0, y: 0.0, z: 1.0 },
        );
        assert_eq!(result.translation.x, 0.0);
        world_destroy(world);
    }

    #[test]
    fn move_shape_with_no_colliders() {
        let world = make_world();
        make_dynamic_body(world); // ensure world is non-empty but no colliders
        let c = character_controller_create();
        character_controller_set_up(c, Vec3 { x: 0.0, y: 1.0, z: 0.0 });
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        let result = character_controller_move_shape(
            world,
            c,
            1.0 / 60.0,
            shape,
            Vec3 { x: 0.0, y: 1.0, z: 0.0 },
            Quat { i: 0.0, j: 0.0, k: 0.0, w: 1.0 },
            Vec3 { x: 1.0, y: 0.0, z: 0.0 },
        );
        // translation applied should be close to desired (no obstacles)
        assert!((result.translation.x - 1.0).abs() < 0.1);
        character_controller_destroy(c);
        world_destroy(world);
    }

    #[test]
    fn move_shape_rejects_invalid_dt() {
        let world = make_world();
        let c = character_controller_create();
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        let result = character_controller_move_shape(
            world,
            c,
            0.0, // invalid dt
            shape,
            Vec3::default(),
            Quat { i: 0.0, j: 0.0, k: 0.0, w: 1.0 },
            Vec3 { x: 1.0, y: 0.0, z: 0.0 },
        );
        assert_eq!(result.translation.x, 0.0);
        character_controller_destroy(c);
        world_destroy(world);
    }

    // ---- collision_count ----

    #[test]
    fn collision_count_rejects_null() {
        assert_eq!(character_controller_collision_count(std::ptr::null()), 0);
    }

    // ---- get_collision ----

    #[test]
    fn get_collision_rejects_null() {
        let col = character_controller_get_collision(std::ptr::null(), 0);
        assert_eq!(col.collider, 0);
    }

    // ---- solve_impulses ----

    #[test]
    fn solve_impulses_rejects_null_world() {
        let c = character_controller_create();
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        assert_eq!(
            character_controller_solve_impulses(std::ptr::null_mut(), c, 1.0 / 60.0, shape, 70.0),
            Bool::FALSE
        );
        character_controller_destroy(c);
    }

    #[test]
    fn solve_impulses_rejects_invalid_dt() {
        let world = make_world();
        let c = character_controller_create();
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        assert_eq!(
            character_controller_solve_impulses(world, c, 0.0, shape, 70.0),
            Bool::FALSE
        );
        character_controller_destroy(c);
        world_destroy(world);
    }

    #[test]
    fn solve_impulses_rejects_negative_mass() {
        let world = make_world();
        let c = character_controller_create();
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        assert_eq!(
            character_controller_solve_impulses(world, c, 1.0 / 60.0, shape, -1.0),
            Bool::FALSE
        );
        character_controller_destroy(c);
        world_destroy(world);
    }
}



