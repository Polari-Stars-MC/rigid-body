#[cfg(test)]
mod tests {
    use mps_core::rapier::collider::*;
    use mps_core::rapier::error::{
        ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, ERR_OK, last_error_code,
    };
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::rigid_body::{
        rigid_body_builder_build, rigid_body_builder_create, world_insert_rigid_body,
    };
    use mps_core::rapier::world::{world_create, world_destroy};

    fn aabb(min: f64, max: f64) -> AabbDesc {
        AabbDesc {
            mins: Vec3 {
                x: min,
                y: min,
                z: min,
            },
            maxs: Vec3 {
                x: max,
                y: max,
                z: max,
            },
        }
    }

    fn assert_builder(builder: *mut ColliderBuilderHandle) {
        assert!(!builder.is_null());
        collider_builder_destroy(builder);
    }

    #[test]
    fn convex_hull_builder_accepts_cube_points() {
        let points = [
            -1.0, -1.0, -1.0, //
            -1.0, -1.0, 1.0, //
            -1.0, 1.0, -1.0, //
            -1.0, 1.0, 1.0, //
            1.0, -1.0, -1.0, //
            1.0, -1.0, 1.0, //
            1.0, 1.0, -1.0, //
            1.0, 1.0, 1.0,
        ];

        assert_builder(collider_builder_create_convex_hull(points.as_ptr(), 8));
    }

    #[test]
    fn point_cloud_bounds_builder_accepts_points() {
        let points = [
            -2.0, 1.0, 0.5, //
            3.0, -4.0, 2.0, //
            1.0, 2.0, -6.0,
        ];

        assert_builder(collider_builder_create_point_cloud_bounds(
            points.as_ptr(),
            3,
        ));
    }

    #[test]
    fn broad_volume_builders_accept_valid_inputs() {
        let points = [
            -2.0, 1.0, 0.5, //
            3.0, -4.0, 2.0, //
            1.0, 2.0, -6.0, //
            0.0, 0.0, 0.0,
        ];
        let vertices = [
            0.0, 0.0, 0.0, //
            1.0, 0.0, 0.0, //
            1.0, 1.0, 0.0,
        ];
        let edges = [0u32, 1, 1, 2];
        let spheres = [
            0.0, 0.0, 0.0, 0.5, //
            1.0, 0.0, 0.0, 0.25,
        ];

        assert_builder(collider_builder_create_double_bv(
            aabb(0.0, 1.0),
            aabb(2.0, 3.0),
        ));
        assert_builder(collider_builder_create_skewed_obb(
            Vec3::default(),
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3 {
                x: 0.25,
                y: 1.0,
                z: 0.0,
            },
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ));
        assert_builder(collider_builder_create_discrete_obb(points.as_ptr(), 4, 1));
        assert_builder(collider_builder_create_fused_collapsing_bounds(
            points.as_ptr(),
            4,
            0.1,
        ));
        assert_builder(collider_builder_create_edge_bvh(
            vertices.as_ptr(),
            3,
            edges.as_ptr(),
            2,
            0.05,
        ));
        assert_builder(collider_builder_create_medial_spheres(spheres.as_ptr(), 2));
    }

    // ---- shared helpers ----

    fn v3(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    fn identity_quat() -> Quat {
        Quat {
            i: 0.0,
            j: 0.0,
            k: 0.0,
            w: 1.0,
        }
    }

    fn make_world() -> *mut WorldHandle {
        world_create(v3(0.0, -9.81, 0.0))
    }

    fn ball_desc() -> ShapeDesc {
        ShapeDesc {
            shape_type: 0,
            a: 0.5,
            b: 0.0,
            c: 0.0,
            d: 0.0,
        }
    }

    fn make_ball_builder() -> *mut ColliderBuilderHandle {
        let builder = collider_builder_create_ex(ball_desc());
        assert!(!builder.is_null());
        builder
    }

    fn insert_ball(world: *mut WorldHandle) -> ColliderHandleRaw {
        let collider = collider_builder_build(make_ball_builder());
        assert!(!collider.is_null());
        let handle = world_insert_collider(world, collider);
        assert_ne!(handle, 0);
        handle
    }

    // ---- shape builders: happy paths ----

    #[test]
    fn builder_create_accepts_all_shape_types() {
        let cases = [
            (0, v3(0.5, 0.0, 0.0)),   // ball: radius
            (1, v3(0.5, 0.5, 0.5)),   // cuboid: half extents
            (2, v3(0.5, 0.25, 0.0)),  // capsule_y
            (3, v3(0.5, 0.25, 0.0)),  // capsule_x
            (4, v3(0.5, 0.25, 0.0)),  // capsule_z
            (5, v3(0.5, 0.25, 0.0)),  // cylinder
            (6, v3(0.5, 0.25, 0.05)), // round cylinder
            (7, v3(0.5, 0.25, 0.0)),  // cone
            (8, v3(0.5, 0.25, 0.05)), // round cone
            (9, v3(0.5, 0.5, 0.5)),   // round cuboid (border radius 0)
        ];
        for (shape_type, data) in cases {
            let builder = collider_builder_create(shape_type, data);
            assert!(!builder.is_null(), "shape_type {shape_type}");
            collider_builder_destroy(builder);
        }
    }

    #[test]
    fn builder_create_rejects_invalid_dimensions() {
        assert!(collider_builder_create(0, v3(-1.0, 0.0, 0.0)).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert!(collider_builder_create(1, v3(f64::NAN, 1.0, 1.0)).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn builder_create_ex_accepts_round_cuboid_with_border() {
        let desc = ShapeDesc {
            shape_type: 9,
            a: 1.0,
            b: 1.0,
            c: 1.0,
            d: 0.1,
        };
        assert_builder(collider_builder_create_ex(desc));
    }

    #[test]
    fn builder_create_ex_rejects_negative_border_radius() {
        let desc = ShapeDesc {
            shape_type: 9,
            a: 1.0,
            b: 1.0,
            c: 1.0,
            d: -0.1,
        };
        assert!(collider_builder_create_ex(desc).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn halfspace_builder_accepts_finite_normal() {
        assert_builder(collider_builder_create_halfspace(v3(0.0, 1.0, 0.0)));
    }

    #[test]
    fn halfspace_builder_rejects_non_finite_normal() {
        assert!(collider_builder_create_halfspace(v3(f64::NAN, 0.0, 0.0)).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn obb_builder_accepts_valid_obb() {
        let obb = Obb {
            center: v3(1.0, 2.0, 3.0),
            half_extents: v3(0.5, 0.5, 0.5),
            rotation: identity_quat(),
        };
        assert_builder(collider_builder_create_obb(obb));
    }

    #[test]
    fn obb_builder_rejects_invalid_obb() {
        let degenerate = Obb {
            center: Vec3::default(),
            half_extents: v3(0.0, 1.0, 1.0),
            rotation: identity_quat(),
        };
        assert!(collider_builder_create_obb(degenerate).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let nan_rotation = Obb {
            center: Vec3::default(),
            half_extents: v3(1.0, 1.0, 1.0),
            rotation: Quat {
                i: f64::NAN,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
        };
        assert!(collider_builder_create_obb(nan_rotation).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn sphere_builder_accepts_valid_sphere() {
        let sphere = Sphere {
            center: v3(1.0, 2.0, 3.0),
            radius: 0.75,
        };
        assert_builder(collider_builder_create_sphere(sphere));
    }

    #[test]
    fn sphere_builder_rejects_invalid_sphere() {
        let zero_radius = Sphere {
            center: Vec3::default(),
            radius: 0.0,
        };
        assert!(collider_builder_create_sphere(zero_radius).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let nan_center = Sphere {
            center: v3(f64::NAN, 0.0, 0.0),
            radius: 1.0,
        };
        assert!(collider_builder_create_sphere(nan_center).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn heightmap_builder_accepts_valid_grid() {
        let data = [0.0f64, 0.1, 0.2, 0.3];
        assert_builder(collider_builder_create_heightmap(
            data.as_ptr(),
            2,
            2,
            v3(1.0, 1.0, 1.0),
        ));
    }

    #[test]
    fn heightmap_builder_rejects_invalid_input() {
        let data = [0.0f64; 4];
        assert!(
            collider_builder_create_heightmap(std::ptr::null(), 2, 2, v3(1.0, 1.0, 1.0)).is_null()
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert!(
            collider_builder_create_heightmap(data.as_ptr(), 0, 2, v3(1.0, 1.0, 1.0)).is_null()
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let nan_data = [0.0f64, f64::NAN, 0.0, 0.0];
        assert!(
            collider_builder_create_heightmap(nan_data.as_ptr(), 2, 2, v3(1.0, 1.0, 1.0)).is_null()
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    // ---- point-buffer builders: negative cases ----

    #[test]
    fn convex_hull_builder_rejects_invalid_input() {
        let cube = [
            -1.0, -1.0, -1.0, //
            -1.0, -1.0, 1.0, //
            -1.0, 1.0, -1.0, //
            -1.0, 1.0, 1.0, //
            1.0, -1.0, -1.0, //
            1.0, -1.0, 1.0, //
            1.0, 1.0, -1.0, //
            1.0, 1.0, 1.0,
        ];

        assert!(collider_builder_create_convex_hull(std::ptr::null(), 8).is_null());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert!(collider_builder_create_convex_hull(cube.as_ptr(), 0).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // A hull needs at least 4 points.
        assert!(collider_builder_create_convex_hull(cube.as_ptr(), 3).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let nan_points = [
            0.0,
            0.0,
            0.0, //
            f64::NAN,
            1.0,
            0.0, //
            1.0,
            0.0,
            1.0, //
            0.0,
            1.0,
            1.0,
        ];
        assert!(collider_builder_create_convex_hull(nan_points.as_ptr(), 4).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn point_buffer_builders_reject_null_points() {
        assert!(collider_builder_create_point_cloud_bounds(std::ptr::null(), 1).is_null());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert!(collider_builder_create_discrete_obb(std::ptr::null(), 1, 0).is_null());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert!(
            collider_builder_create_fused_collapsing_bounds(std::ptr::null(), 1, 0.0).is_null()
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    #[test]
    fn fused_collapsing_bounds_rejects_negative_padding() {
        let points = [0.0f64, 0.0, 0.0];
        assert!(
            collider_builder_create_fused_collapsing_bounds(points.as_ptr(), 1, -0.5).is_null()
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn double_bv_rejects_inverted_aabb() {
        assert!(collider_builder_create_double_bv(aabb(1.0, 0.0), aabb(0.0, 1.0)).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn skewed_obb_rejects_zero_axis() {
        let builder = collider_builder_create_skewed_obb(
            Vec3::default(),
            v3(0.0, 0.0, 0.0),
            v3(0.0, 1.0, 0.0),
            v3(0.0, 0.0, 1.0),
        );
        assert!(builder.is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn edge_bvh_rejects_invalid_input() {
        let vertices = [
            0.0, 0.0, 0.0, //
            1.0, 0.0, 0.0, //
            1.0, 1.0, 0.0,
        ];
        let edges = [0u32, 1];

        assert!(
            collider_builder_create_edge_bvh(vertices.as_ptr(), 3, std::ptr::null(), 1, 0.05)
                .is_null()
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert!(
            collider_builder_create_edge_bvh(vertices.as_ptr(), 3, edges.as_ptr(), 0, 0.05)
                .is_null()
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(
            collider_builder_create_edge_bvh(vertices.as_ptr(), 3, edges.as_ptr(), 1, 0.0)
                .is_null()
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let out_of_range = [0u32, 7];
        assert!(
            collider_builder_create_edge_bvh(vertices.as_ptr(), 3, out_of_range.as_ptr(), 1, 0.05)
                .is_null()
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn medial_spheres_rejects_invalid_input() {
        let spheres = [0.0f64, 0.0, 0.0, 0.5];

        assert!(collider_builder_create_medial_spheres(std::ptr::null(), 1).is_null());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert!(collider_builder_create_medial_spheres(spheres.as_ptr(), 0).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let negative_radius = [0.0f64, 0.0, 0.0, -1.0];
        assert!(collider_builder_create_medial_spheres(negative_radius.as_ptr(), 1).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    // ---- build / destroy ----

    #[test]
    fn builder_build_produces_destroyable_collider() {
        let collider = collider_builder_build(make_ball_builder());
        assert!(!collider.is_null());
        collider_destroy_raw(collider);
    }

    #[test]
    fn null_raw_pointers_report_null_pointer() {
        assert!(collider_builder_build(std::ptr::null_mut()).is_null());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        collider_builder_destroy(std::ptr::null_mut());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        collider_destroy_raw(std::ptr::null_mut());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    // ---- builder setters ----

    #[test]
    fn builder_setters_accept_valid_values() {
        let builder = make_ball_builder();
        let groups = InteractionGroupsDesc {
            memberships: 0b1,
            filter: 0b10,
        };
        collider_builder_set_translation(builder, v3(1.0, 2.0, 3.0));
        collider_builder_set_rotation(builder, v3(0.0, 0.5, 0.0));
        collider_builder_set_pose(builder, v3(1.0, 0.0, 0.0), identity_quat());
        collider_builder_set_sensor(builder, Bool::TRUE);
        collider_builder_set_friction(builder, 0.8);
        collider_builder_set_restitution(builder, 0.2);
        collider_builder_set_density(builder, 2.5);
        collider_builder_set_collision_groups(builder, groups);
        collider_builder_set_solver_groups(builder, groups);
        collider_builder_set_active_events(builder, 1);
        collider_builder_set_active_hooks(builder, 1);
        collider_builder_set_contact_force_event_threshold(builder, 10.0);
        assert_eq!(last_error_code(), ERR_OK);

        let collider = collider_builder_build(builder);
        assert!(!collider.is_null());
        collider_destroy_raw(collider);
    }

    #[test]
    fn builder_setters_reject_null_builder() {
        let groups = InteractionGroupsDesc::default();
        let null = std::ptr::null_mut();

        collider_builder_set_translation(null, Vec3::default());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_builder_set_rotation(null, Vec3::default());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_builder_set_pose(null, Vec3::default(), identity_quat());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_builder_set_sensor(null, Bool::TRUE);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_builder_set_friction(null, 0.5);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_builder_set_restitution(null, 0.5);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_builder_set_density(null, 1.0);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_builder_set_collision_groups(null, groups);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_builder_set_solver_groups(null, groups);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_builder_set_active_events(null, 1);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_builder_set_active_hooks(null, 1);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_builder_set_contact_force_event_threshold(null, 1.0);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    #[test]
    fn builder_setters_reject_invalid_values() {
        let builder = make_ball_builder();
        collider_builder_set_friction(builder, -1.0);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        collider_builder_set_friction(builder, f64::NAN);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        collider_builder_set_restitution(builder, -0.5);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        collider_builder_set_density(builder, -1.0);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        collider_builder_set_contact_force_event_threshold(builder, -1.0);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        collider_builder_set_translation(builder, v3(f64::NAN, 0.0, 0.0));
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        collider_builder_set_rotation(builder, v3(f64::NAN, 0.0, 0.0));
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        collider_builder_set_pose(
            builder,
            Vec3::default(),
            Quat {
                i: f64::NAN,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        collider_builder_destroy(builder);
    }

    // ---- world insert / remove / copy ----

    #[test]
    fn insert_and_remove_collider_round_trip() {
        let world = make_world();
        let handle = insert_ball(world);

        assert_eq!(world_remove_collider(world, handle, Bool::TRUE), Bool::TRUE);
        assert_eq!(last_error_code(), ERR_OK);

        // Removing the same handle again reports not found.
        assert_eq!(
            world_remove_collider(world, handle, Bool::FALSE),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn insert_collider_rejects_null_arguments() {
        let world = make_world();
        let collider = collider_builder_build(make_ball_builder());

        assert_eq!(world_insert_collider(std::ptr::null_mut(), collider), 0);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        // The failed insert did not consume the collider.
        collider_destroy_raw(collider);

        assert_eq!(world_insert_collider(world, std::ptr::null_mut()), 0);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        world_destroy(world);
    }

    #[test]
    fn insert_collider_with_parent_attaches_to_body() {
        let world = make_world();
        let body_builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        assert!(!body_builder.is_null());
        let body = rigid_body_builder_build(body_builder);
        assert!(!body.is_null());
        let body_handle = world_insert_rigid_body(world, body);
        assert_ne!(body_handle, 0);

        let collider = collider_builder_build(make_ball_builder());
        let handle = world_insert_collider_with_parent(world, collider, body_handle);
        assert_ne!(handle, 0);
        world_destroy(world);
    }

    #[test]
    fn insert_collider_with_parent_rejects_null_arguments() {
        let world = make_world();
        let collider = collider_builder_build(make_ball_builder());

        assert_eq!(
            world_insert_collider_with_parent(std::ptr::null_mut(), collider, 1),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_destroy_raw(collider);

        assert_eq!(
            world_insert_collider_with_parent(world, std::ptr::null_mut(), 1),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        world_destroy(world);
    }

    #[test]
    fn copy_collider_round_trip() {
        let world = make_world();
        let handle = insert_ball(world);

        let copy = world_copy_collider(world, handle);
        assert!(!copy.is_null());
        collider_destroy_raw(copy);
        world_destroy(world);
    }

    #[test]
    fn copy_collider_rejects_invalid_arguments() {
        let world = make_world();

        assert!(world_copy_collider(std::ptr::null_mut(), 1).is_null());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert!(world_copy_collider(world, 12345).is_null());
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn remove_collider_flag_matches_bool_variant() {
        let world = make_world();
        let handle = insert_ball(world);

        assert_eq!(world_remove_collider_flag(world, handle, Bool::TRUE), 1);
        assert_eq!(world_remove_collider_flag(world, handle, Bool::FALSE), 0);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn remove_collider_rejects_null_world() {
        assert_eq!(
            world_remove_collider(std::ptr::null_mut(), 1, Bool::FALSE),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    // ---- getters ----

    #[test]
    fn translation_and_rotation_round_trip() {
        let world = make_world();
        let builder = make_ball_builder();
        collider_builder_set_translation(builder, v3(1.0, 2.0, 3.0));
        let collider = collider_builder_build(builder);
        let handle = world_insert_collider(world, collider);
        assert_ne!(handle, 0);

        let t = collider_get_translation(world, handle);
        assert_eq!((t.x, t.y, t.z), (1.0, 2.0, 3.0));

        let mut out = Vec3::default();
        collider_get_translation_out(world, handle, &mut out);
        assert_eq!((out.x, out.y, out.z), (1.0, 2.0, 3.0));

        let r = collider_get_rotation(world, handle);
        assert!((r.w - 1.0).abs() < 1.0e-9);

        let mut out_quat = Quat::default();
        collider_get_rotation_out(world, handle, &mut out_quat);
        assert!((out_quat.w - 1.0).abs() < 1.0e-9);
        world_destroy(world);
    }

    #[test]
    fn getters_reject_null_world_and_unknown_handle() {
        let world = make_world();
        let handle = insert_ball(world);

        collider_get_translation(std::ptr::null(), handle);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_get_translation(world, 12345);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);

        collider_get_rotation(std::ptr::null(), handle);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_get_rotation(world, 12345);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);

        assert_eq!(collider_get_shape_count(std::ptr::null(), handle), 0);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(collider_get_shape_count(world, 12345), 0);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);

        assert_eq!(collider_get_density(std::ptr::null(), handle), 0.0);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(collider_get_density(world, 12345), 0.0);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn out_getters_reject_null_output() {
        let world = make_world();
        let handle = insert_ball(world);

        collider_get_translation_out(world, handle, std::ptr::null_mut());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        collider_get_rotation_out(world, handle, std::ptr::null_mut());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        world_destroy(world);
    }

    #[test]
    fn shape_count_reflects_compound_parts() {
        let world = make_world();
        let ball = insert_ball(world);
        assert_eq!(collider_get_shape_count(world, ball), 1);

        let spheres = [
            0.0, 0.0, 0.0, 0.5, //
            1.0, 0.0, 0.0, 0.25,
        ];
        let builder = collider_builder_create_medial_spheres(spheres.as_ptr(), 2);
        assert!(!builder.is_null());
        let collider = collider_builder_build(builder);
        let compound = world_insert_collider(world, collider);
        assert_eq!(collider_get_shape_count(world, compound), 2);
        world_destroy(world);
    }

    #[test]
    fn density_reflects_builder_value() {
        let world = make_world();
        let builder = make_ball_builder();
        collider_builder_set_density(builder, 2.5);
        let collider = collider_builder_build(builder);
        let handle = world_insert_collider(world, collider);

        assert_eq!(collider_get_density(world, handle), 2.5);
        world_destroy(world);
    }

    // ---- world-side setters ----

    #[test]
    fn pose_setters_move_collider() {
        let world = make_world();
        let handle = insert_ball(world);

        assert_eq!(
            collider_set_translation(world, handle, v3(4.0, 5.0, 6.0)),
            Bool::TRUE
        );
        let t = collider_get_translation(world, handle);
        assert_eq!((t.x, t.y, t.z), (4.0, 5.0, 6.0));

        assert_eq!(
            collider_set_rotation(world, handle, identity_quat()),
            Bool::TRUE
        );

        assert_eq!(
            collider_set_pose(world, handle, v3(7.0, 8.0, 9.0), identity_quat()),
            Bool::TRUE
        );
        let t = collider_get_translation(world, handle);
        assert_eq!((t.x, t.y, t.z), (7.0, 8.0, 9.0));

        assert_eq!(
            collider_set_pose_flag(world, handle, Vec3::default(), identity_quat()),
            1
        );
        world_destroy(world);
    }

    #[test]
    fn material_and_group_setters_return_true() {
        let world = make_world();
        let handle = insert_ball(world);
        let groups = InteractionGroupsDesc {
            memberships: 0b1,
            filter: 0b10,
        };

        assert_eq!(collider_set_sensor(world, handle, Bool::TRUE), Bool::TRUE);
        assert_eq!(collider_set_sensor_flag(world, handle, Bool::FALSE), 1);
        assert_eq!(collider_set_friction(world, handle, 0.9), Bool::TRUE);
        assert_eq!(collider_set_friction_flag(world, handle, 0.5), 1);
        assert_eq!(collider_set_restitution(world, handle, 0.3), Bool::TRUE);
        assert_eq!(collider_set_restitution_flag(world, handle, 0.1), 1);
        assert_eq!(
            collider_set_collision_groups(world, handle, groups),
            Bool::TRUE
        );
        assert_eq!(collider_set_collision_groups_flag(world, handle, groups), 1);
        assert_eq!(
            collider_set_solver_groups(world, handle, groups),
            Bool::TRUE
        );
        assert_eq!(collider_set_solver_groups_flag(world, handle, groups), 1);
        assert_eq!(collider_set_active_events(world, handle, 1), Bool::TRUE);
        assert_eq!(collider_set_active_events_flag(world, handle, 1), 1);
        assert_eq!(collider_set_active_hooks(world, handle, 1), Bool::TRUE);
        assert_eq!(collider_set_active_hooks_flag(world, handle, 1), 1);
        assert_eq!(
            collider_set_contact_force_event_threshold(world, handle, 5.0),
            Bool::TRUE
        );
        assert_eq!(
            collider_set_contact_force_event_threshold_flag(world, handle, 5.0),
            1
        );
        assert_eq!(last_error_code(), ERR_OK);
        world_destroy(world);
    }

    #[test]
    fn world_setters_reject_null_world() {
        let groups = InteractionGroupsDesc::default();
        let null = std::ptr::null_mut();

        assert_eq!(
            collider_set_translation(null, 1, Vec3::default()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(collider_set_rotation(null, 1, identity_quat()), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            collider_set_pose(null, 1, Vec3::default(), identity_quat()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(collider_set_sensor(null, 1, Bool::TRUE), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(collider_set_friction(null, 1, 0.5), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(collider_set_restitution(null, 1, 0.5), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(collider_set_collision_groups(null, 1, groups), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(collider_set_solver_groups(null, 1, groups), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(collider_set_active_events(null, 1, 1), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(collider_set_active_hooks(null, 1, 1), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            collider_set_contact_force_event_threshold(null, 1, 1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    #[test]
    fn world_setters_reject_unknown_handle() {
        let world = make_world();
        let groups = InteractionGroupsDesc::default();

        assert_eq!(
            collider_set_translation(world, 12345, Vec3::default()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(
            collider_set_rotation(world, 12345, identity_quat()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(
            collider_set_pose(world, 12345, Vec3::default(), identity_quat()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(collider_set_sensor(world, 12345, Bool::TRUE), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(collider_set_friction(world, 12345, 0.5), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(collider_set_restitution(world, 12345, 0.5), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(
            collider_set_collision_groups(world, 12345, groups),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(
            collider_set_solver_groups(world, 12345, groups),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(collider_set_active_events(world, 12345, 1), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(collider_set_active_hooks(world, 12345, 1), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(
            collider_set_contact_force_event_threshold(world, 12345, 1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn world_setters_reject_invalid_values() {
        let world = make_world();
        let handle = insert_ball(world);

        assert_eq!(collider_set_friction(world, handle, -1.0), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(collider_set_friction(world, handle, f64::NAN), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(collider_set_restitution(world, handle, -0.5), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            collider_set_contact_force_event_threshold(world, handle, -1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            collider_set_translation(world, handle, v3(f64::NAN, 0.0, 0.0)),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let nan_quat = Quat {
            i: f64::NAN,
            j: 0.0,
            k: 0.0,
            w: 1.0,
        };
        assert_eq!(collider_set_rotation(world, handle, nan_quat), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            collider_set_pose(world, handle, Vec3::default(), nan_quat),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }

    // -----------------------------------------------------------------------
    // 滚动阻力（rolling resistance）
    //
    // 这一组测试钉住三件事：
    //   1. setter 的契约（返回值 + 错误码 + get 回读）；
    //   2. `μ_r = 0` 时行为与改动前一致（球一直滚）—— 这是"没有回归"的证据；
    //   3. `μ_r > 0` 时球会停下 —— 这是"缺口真的补上了"的证据。
    // -----------------------------------------------------------------------

    fn ball_on_ground_world(rolling: f64) -> (*mut WorldHandle, u64) {
        use mps_core::rapier::rigid_body::{
            rigid_body_builder_set_translation, rigid_body_enable_ccd,
        };
        use mps_core::rapier::world::world_step;

        let world = world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        assert!(!world.is_null());

        // Static ground: a half-space whose normal points up, so the plane is
        // `y = 0`. A half-space is the cleanest floor for a rolling test: it has
        // no edges the ball could catch on.
        let ground = collider_builder_create_halfspace(Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        });
        assert!(!ground.is_null(), "cuboid builder failed");
        let ground_collider = world_insert_collider(world, collider_builder_build(ground));
        assert_ne!(ground_collider, 0);
        assert!(collider_set_rolling_resistance(world, ground_collider, rolling).0 != 0);
        assert!((collider_get_rolling_resistance(world, ground_collider) - rolling).abs() < 1e-12);
        // Ground must not slide the ball: give it no friction so the only
        // mechanism that can stop the ball is rolling resistance.
        assert!(collider_set_friction(world, ground_collider, 0.0).0 != 0);

        // Dynamic ball sitting just above the ground.
        let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_translation(
            builder,
            Vec3 {
                x: 0.0,
                y: 0.5,
                z: 0.0,
            },
        );
        let body = rigid_body_builder_build(builder);
        let ball = world_insert_rigid_body(world, body);
        assert_ne!(ball, 0);
        rigid_body_enable_ccd(world, ball, Bool::TRUE);

        let sphere = collider_builder_create_sphere(Sphere {
            center: Vec3::default(),
            radius: 0.5,
        });
        let ball_collider =
            world_insert_collider_with_parent(world, collider_builder_build(sphere), ball);
        assert_ne!(ball_collider, 0);
        assert!(collider_set_friction(world, ball_collider, 0.0).0 != 0);
        // The ball itself carries no rolling resistance; only the ground does,
        // which exercises the combine rule path.
        let _ = world_step;

        (world, ball)
    }

    #[test]
    fn rolling_resistance_setter_contract() {
        let world = world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        let sphere = collider_builder_create_sphere(Sphere {
            center: Vec3::default(),
            radius: 0.5,
        });
        let c = world_insert_collider(world, collider_builder_build(sphere));

        // Valid set + read-back.
        assert!(collider_set_rolling_resistance(world, c, 0.02).0 != 0);
        assert!((collider_get_rolling_resistance(world, c) - 0.02).abs() < 1e-12);
        assert_eq!(last_error_code(), ERR_OK);

        // Negative and non-finite are rejected without touching the value.
        assert!(collider_set_rolling_resistance(world, c, -1.0).0 == 0);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert!(collider_set_rolling_resistance(world, c, f64::NAN).0 == 0);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert!(collider_set_rolling_resistance(world, c, f64::INFINITY).0 == 0);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert!((collider_get_rolling_resistance(world, c) - 0.02).abs() < 1e-12);

        // Null world and unknown handle.
        assert!(collider_set_rolling_resistance(std::ptr::null_mut(), c, 0.1).0 == 0);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert!(collider_set_rolling_resistance(world, u64::MAX, 0.1).0 == 0);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(collider_get_rolling_resistance(world, u64::MAX), 0.0);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);

        // The `_flag` variant agrees with the `Bool` one.
        assert_eq!(
            collider_set_rolling_resistance_flag(world, c, 0.03),
            collider_set_rolling_resistance(world, c, 0.03).0
        );

        world_destroy(world);
    }

    #[test]
    fn default_rolling_resistance_is_zero() {
        let world = world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        let sphere = collider_builder_create_sphere(Sphere {
            center: Vec3::default(),
            radius: 0.5,
        });
        let c = world_insert_collider(world, collider_builder_build(sphere));
        assert_eq!(collider_get_rolling_resistance(world, c), 0.0);
        world_destroy(world);
    }

    /// A ball rolling on a frictionless floor: with `μ_r = 0` it must keep its
    /// angular momentum (no mechanism can stop it), with `μ_r > 0` it must slow
    /// down. Both halves are asserted in one test so a change that breaks either
    /// direction shows up here.
    #[test]
    fn rolling_resistance_stops_a_rolling_ball() {
        use mps_core::rapier::rigid_body::{rigid_body_get_angvel, rigid_body_set_angvel};

        fn spin_after(rolling: f64, steps: usize) -> f64 {
            let (world, ball) = ball_on_ground_world(rolling);
            // Give the ball backspin about z so it rolls along x.
            assert!(
                rigid_body_set_angvel(
                    world,
                    ball,
                    Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: -10.0,
                    },
                    Bool::TRUE
                )
                .0 != 0
            );
            for _ in 0..steps {
                mps_core::rapier::world::world_step(world, 1.0 / 60.0);
            }
            let w = rigid_body_get_angvel(world, ball);
            world_destroy(world);
            w.z.abs()
        }

        let without = spin_after(0.0, 120);
        let with = spin_after(0.02, 120);

        // μ_r = 0 must be bit-exactly the pre-existing behaviour: the ball is
        // frictionless and nothing else acts about z, so |ωz| stays at its
        // initial 10 rad/s to the last bit.
        assert!(
            (without - 10.0).abs() < 1e-9,
            "with μ_r = 0 the ball must keep spinning unchanged (no mechanism acts \
             about z); got |ωz| = {without}"
        );
        // μ_r = 0.02 must measurably remove angular momentum. The observed decay
        // is small per step by construction (the budget is μ_r·N·r, and N is only
        // the weight impulse), so assert a floor well above the μ_r = 0 noise and
        // well below a full stop: this fails if the constraint stops being solved,
        // and also if it starts over-braking.
        assert!(
            with < without - 0.1,
            "with μ_r = 0.02 the ball must lose angular momentum: |ωz| {with} vs {without}"
        );
        assert!(
            with > 5.0,
            "μ_r = 0.02 must not brake the ball to a stop within 2 s: |ωz| = {with}"
        );
    }

    // ── 通用属性通道：collider 批（ADR-0016 跨模块验证） ──────────────────────

    use mps_core::rapier::collider::{
        COLLIDER_PARAM_ACTIVE_EVENTS, COLLIDER_PARAM_ACTIVE_HOOKS,
        COLLIDER_PARAM_CONTACT_FORCE_EVENT_THRESHOLD, COLLIDER_PARAM_FRICTION,
        COLLIDER_PARAM_FRICTION_COMBINE_RULE, COLLIDER_PARAM_RESTITUTION,
        COLLIDER_PARAM_RESTITUTION_COMBINE_RULE, COLLIDER_PARAM_ROLLING_RESISTANCE,
        COLLIDER_PARAM_SENSOR, collider_param_abi_version, collider_set_param,
        collider_set_param_flag,
    };

    #[test]
    fn collider_param_ids_match_the_abi_version() {
        let all = [
            COLLIDER_PARAM_FRICTION,
            COLLIDER_PARAM_ROLLING_RESISTANCE,
            COLLIDER_PARAM_RESTITUTION,
            COLLIDER_PARAM_CONTACT_FORCE_EVENT_THRESHOLD,
            COLLIDER_PARAM_FRICTION_COMBINE_RULE,
            COLLIDER_PARAM_RESTITUTION_COMBINE_RULE,
            COLLIDER_PARAM_ACTIVE_EVENTS,
            COLLIDER_PARAM_ACTIVE_HOOKS,
            COLLIDER_PARAM_SENSOR,
        ];
        let version = collider_param_abi_version();
        assert_eq!(
            version as usize,
            all.len(),
            "abi version must equal the number of allocated params"
        );
        let mut sorted = all;
        sorted.sort_unstable();
        for w in sorted.windows(2) {
            assert_ne!(w[0], w[1], "duplicate collider param id {}", w[0]);
        }
    }

    /// The migration premise: every dedicated collider setter and the generic
    /// channel must reach the same conclusion, for accept and reject alike.
    #[test]
    fn collider_setters_agree_with_the_generic_channel() {
        let world = make_world();
        let a = insert_ball(world);
        let b = insert_ball(world);

        // (param, value, dedicated entry) — includes the reject cases.
        let cases: [(
            u32,
            f64,
            fn(*mut WorldHandle, ColliderHandleRaw, f64) -> Bool,
        ); 12] = [
            (COLLIDER_PARAM_FRICTION, 0.7, |w, h, v| {
                collider_set_friction(w, h, v)
            }),
            (COLLIDER_PARAM_FRICTION, -0.5, |w, h, v| {
                collider_set_friction(w, h, v)
            }),
            (COLLIDER_PARAM_ROLLING_RESISTANCE, 0.05, |w, h, v| {
                collider_set_rolling_resistance(w, h, v)
            }),
            (COLLIDER_PARAM_ROLLING_RESISTANCE, -1.0, |w, h, v| {
                collider_set_rolling_resistance(w, h, v)
            }),
            (COLLIDER_PARAM_RESTITUTION, 0.4, |w, h, v| {
                collider_set_restitution(w, h, v)
            }),
            (COLLIDER_PARAM_RESTITUTION, f64::NAN, |w, h, v| {
                collider_set_restitution(w, h, v)
            }),
            (
                COLLIDER_PARAM_CONTACT_FORCE_EVENT_THRESHOLD,
                10.0,
                |w, h, v| collider_set_contact_force_event_threshold(w, h, v),
            ),
            (
                COLLIDER_PARAM_CONTACT_FORCE_EVENT_THRESHOLD,
                -3.0,
                |w, h, v| collider_set_contact_force_event_threshold(w, h, v),
            ),
            // Combine rules take a u32; only whole numbers in 0..=4 reach them.
            (COLLIDER_PARAM_FRICTION_COMBINE_RULE, 2.0, |w, h, v| {
                collider_set_friction_combine_rule(w, h, v as u32)
            }),
            (COLLIDER_PARAM_RESTITUTION_COMBINE_RULE, 4.0, |w, h, v| {
                collider_set_restitution_combine_rule(w, h, v as u32)
            }),
            (COLLIDER_PARAM_ACTIVE_EVENTS, 3.0, |w, h, v| {
                collider_set_active_events(w, h, v as u32)
            }),
            (COLLIDER_PARAM_ACTIVE_HOOKS, 1.0, |w, h, v| {
                collider_set_active_hooks(w, h, v as u32)
            }),
        ];

        for (param, value, dedicated) in cases {
            let generic = collider_set_param(world, a, param, value).0 != 0;
            let specific = dedicated(world, b, value).0 != 0;
            assert_eq!(
                generic, specific,
                "collider param {param} value {value}: generic {generic} vs dedicated {specific}"
            );
        }
        world_destroy(world);
    }

    /// The sensor flag is `Bool` on the dedicated entry, `f64` on the channel.
    #[test]
    fn collider_sensor_flag_agrees_with_the_dedicated_entry() {
        let world = make_world();
        // The return value is "the call succeeded", not "the flag is now true" —
        // `value == 0.0` successfully clears the sensor bit and still returns
        // TRUE. So this test compares the two paths, it does not assert the bit.
        for value in [1.0, 0.0, 2.0, -1.0] {
            let a = insert_ball(world);
            let b = insert_ball(world);
            let generic = collider_set_param(world, a, COLLIDER_PARAM_SENSOR, value).0 != 0;
            let specific = collider_set_sensor(world, b, Bool(value as u8)).0 != 0;
            assert_eq!(
                generic, specific,
                "sensor value {value}: generic {generic} vs dedicated {specific}"
            );
            assert!(generic, "setting a sensor flag is a successful call");
        }
        // NaN is rejected by the channel (the dedicated entry takes a Bool, so
        // there is no equivalent input to compare against).
        let a = insert_ball(world);
        assert_eq!(
            collider_set_param(world, a, COLLIDER_PARAM_SENSOR, f64::NAN),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }

    #[test]
    fn collider_set_param_rejects_unknown_ids_without_touching_state() {
        let world = make_world();
        let handle = insert_ball(world);

        // Known state first.
        assert_eq!(
            collider_set_param(world, handle, COLLIDER_PARAM_FRICTION, 0.42),
            Bool::TRUE
        );

        let bogus = collider_param_abi_version() + 50;
        assert_eq!(collider_set_param(world, handle, bogus, 1.0), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // A rejected set must not disturb the earlier value.
        assert_eq!(
            collider_set_param(world, handle, COLLIDER_PARAM_FRICTION, 0.42),
            Bool::TRUE
        );

        world_destroy(world);
    }

    #[test]
    fn collider_set_param_validates_the_error_channels() {
        let world = make_world();
        let handle = insert_ball(world);

        assert_eq!(
            collider_set_param(std::ptr::null_mut(), handle, COLLIDER_PARAM_FRICTION, 0.5),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // A handle that was never allocated.
        assert_eq!(
            collider_set_param(world, 0xDEAD_BEEF, COLLIDER_PARAM_FRICTION, 0.5),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);

        world_destroy(world);
    }

    /// `_flag` must be the same verdict as the `Bool` variant — the whole point
    /// of not duplicating eight dedicated `_flag` entries.
    #[test]
    fn collider_set_param_flag_matches_the_bool_variant() {
        let world = make_world();
        let handle = insert_ball(world);

        for (param, value) in [
            (COLLIDER_PARAM_FRICTION, 0.3),
            (COLLIDER_PARAM_FRICTION, -0.3),
            (COLLIDER_PARAM_SENSOR, 1.0),
            (collider_param_abi_version() + 1, 1.0),
        ] {
            let b = collider_set_param(world, handle, param, value).0;
            let f = collider_set_param_flag(world, handle, param, value);
            assert_eq!(b, f, "param {param} value {value}: Bool {b} vs flag {f}");
        }
        world_destroy(world);
    }

    /// Combine rules: fractional and out-of-range values are rejected by the
    /// channel even though the dedicated entry silently maps them to `Average`.
    #[test]
    fn collider_combine_rule_rejects_non_integer_and_out_of_range() {
        let world = make_world();
        let handle = insert_ball(world);

        for bad in [1.5, -1.0, 5.0, f64::NAN] {
            assert_eq!(
                collider_set_param(world, handle, COLLIDER_PARAM_FRICTION_COMBINE_RULE, bad),
                Bool::FALSE,
                "combine rule {bad} must be rejected"
            );
            assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        }
        for good in [0.0, 1.0, 2.0, 3.0, 4.0] {
            assert_eq!(
                collider_set_param(world, handle, COLLIDER_PARAM_FRICTION_COMBINE_RULE, good),
                Bool::TRUE,
                "combine rule {good} must be accepted"
            );
        }
        world_destroy(world);
    }

    // ── 向量与复合载荷通道（ADR-0016 第四批，设计上限验证） ──────────────────

    use mps_core::rapier::collider::{
        COLLIDER_QUAT_ROTATION, COLLIDER_U32X2_COLLISION_GROUPS, COLLIDER_U32X2_SOLVER_GROUPS,
        COLLIDER_VEC3_TRANSLATION, collider_quat_param_abi_version, collider_set_quat_param,
        collider_set_u32x2_param, collider_set_vec3_param, collider_u32x2_param_abi_version,
        collider_vec3_param_abi_version,
    };

    #[test]
    fn vec_and_composite_param_ids_match_their_abi_versions() {
        assert_eq!(collider_vec3_param_abi_version(), 1);
        assert_eq!(collider_quat_param_abi_version(), 1);
        assert_eq!(collider_u32x2_param_abi_version(), 2);
        // The single allocated ids must be 0 so a version bump is the only way
        // to add a property.
        assert_eq!(COLLIDER_VEC3_TRANSLATION, 0);
        assert_eq!(COLLIDER_QUAT_ROTATION, 0);
        assert_ne!(
            COLLIDER_U32X2_COLLISION_GROUPS, COLLIDER_U32X2_SOLVER_GROUPS,
            "the two composite properties must not alias"
        );
    }

    /// `set_vec3_param(TRANSLATION, ..)` must equal `collider_set_translation`.
    #[test]
    fn vec3_translation_agrees_with_the_dedicated_entry() {
        let world = make_world();
        for (x, y, z) in [(1.0, 2.0, 3.0), (-0.5, 0.0, 4.25), (0.0, 0.0, 0.0)] {
            let a = insert_ball(world);
            let b = insert_ball(world);
            let generic = collider_set_vec3_param(world, a, COLLIDER_VEC3_TRANSLATION, x, y, z).0;
            let specific = collider_set_translation(world, b, Vec3 { x, y, z }).0;
            assert_eq!(
                generic, specific,
                "translation ({x}, {y}, {z}): generic {generic} vs dedicated {specific}"
            );
        }

        // Non-finite components are rejected.
        let a = insert_ball(world);
        assert_eq!(
            collider_set_vec3_param(world, a, COLLIDER_VEC3_TRANSLATION, f64::NAN, 0.0, 0.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        world_destroy(world);
    }

    /// `set_quat_param(ROTATION, ..)` must equal `collider_set_rotation`.
    ///
    /// Note the component naming: the ABI `Quat` uses `i/j/k/w`, so the channel
    /// parameters `x/y/z/w` map onto `i/j/k/w` in that order.
    #[test]
    fn quat_rotation_agrees_with_the_dedicated_entry() {
        let world = make_world();
        // (x, y, z, w) = (i, j, k, w) in ABI order.
        for (x, y, z, w) in [
            (0.0, 0.0, 0.0, 1.0),                               // identity
            (0.0, 0.7071067811865476, 0.0, 0.7071067811865476), // 90° about Y
            (0.5, 0.5, 0.5, 0.5),                               // unit-ish
        ] {
            let a = insert_ball(world);
            let b = insert_ball(world);
            let generic = collider_set_quat_param(world, a, COLLIDER_QUAT_ROTATION, x, y, z, w).0;
            let specific = collider_set_rotation(
                world,
                b,
                Quat {
                    i: x,
                    j: y,
                    k: z,
                    w,
                },
            )
            .0;
            assert_eq!(
                generic, specific,
                "rotation ({x}, {y}, {z}, {w}): generic {generic} vs dedicated {specific}"
            );
        }

        let a = insert_ball(world);
        assert_eq!(
            collider_set_quat_param(
                world,
                a,
                COLLIDER_QUAT_ROTATION,
                0.0,
                0.0,
                0.0,
                f64::INFINITY
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        world_destroy(world);
    }

    /// The composite channel must equal both `set_collision_groups` and
    /// `set_solver_groups`.
    #[test]
    fn u32x2_groups_agree_with_the_dedicated_entries() {
        let world = make_world();
        let cases: [(u32, u32); 3] = [
            (0x0001_0001, 0xFFFF_FFFF),
            (0, 0),
            (0x00FF_00FF, 0x0000_FFFF),
        ];

        for (memberships, filter) in cases {
            // collision groups
            let a = insert_ball(world);
            let b = insert_ball(world);
            let generic = collider_set_u32x2_param(
                world,
                a,
                COLLIDER_U32X2_COLLISION_GROUPS,
                memberships,
                filter,
            );
            let specific = collider_set_collision_groups(
                world,
                b,
                InteractionGroupsDesc {
                    memberships,
                    filter,
                },
            );
            assert_eq!(
                generic, specific,
                "collision groups ({memberships:#x}, {filter:#x})"
            );

            // solver groups
            let a = insert_ball(world);
            let b = insert_ball(world);
            let generic = collider_set_u32x2_param(
                world,
                a,
                COLLIDER_U32X2_SOLVER_GROUPS,
                memberships,
                filter,
            );
            let specific = collider_set_solver_groups(
                world,
                b,
                InteractionGroupsDesc {
                    memberships,
                    filter,
                },
            );
            assert_eq!(
                generic, specific,
                "solver groups ({memberships:#x}, {filter:#x})"
            );
        }
        world_destroy(world);
    }

    #[test]
    fn new_channels_reject_unknown_ids_and_bad_handles() {
        let world = make_world();
        let handle = insert_ball(world);

        // Unknown param on each channel.
        assert_eq!(
            collider_set_vec3_param(
                world,
                handle,
                collider_vec3_param_abi_version(),
                1.0,
                0.0,
                0.0
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert_eq!(
            collider_set_quat_param(
                world,
                handle,
                collider_quat_param_abi_version(),
                0.0,
                0.0,
                0.0,
                1.0
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert_eq!(
            collider_set_u32x2_param(world, handle, collider_u32x2_param_abi_version(), 0, 0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Null world on each channel.
        assert_eq!(
            collider_set_vec3_param(
                std::ptr::null_mut(),
                handle,
                COLLIDER_VEC3_TRANSLATION,
                0.0,
                0.0,
                0.0
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Unknown handle.
        assert_eq!(
            collider_set_u32x2_param(world, 0xDEAD_BEEF, COLLIDER_U32X2_COLLISION_GROUPS, 1, 1),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);

        world_destroy(world);
    }

    /// Setting translation must not disturb rotation, and vice versa — both
    /// dedicated entries preserve the other half of the pose.
    #[test]
    fn vec_and_quat_channels_preserve_the_other_half_of_the_pose() {
        let world = make_world();
        let handle = insert_ball(world);

        // Establish a known pose.
        let t = Vec3 {
            x: 3.0,
            y: -2.0,
            z: 1.5,
        };
        let q = Quat {
            i: 0.0,
            j: 0.7071067811865476,
            k: 0.0,
            w: 0.7071067811865476,
        };
        assert_eq!(collider_set_pose(world, handle, t, q), Bool::TRUE);

        // Change only the translation; rotation must survive.
        assert_eq!(
            collider_set_vec3_param(world, handle, COLLIDER_VEC3_TRANSLATION, 7.0, 8.0, 9.0),
            Bool::TRUE
        );
        let out_t = collider_get_translation(world, handle);
        let out_q = collider_get_rotation(world, handle);
        assert!((out_t.x - 7.0).abs() < 1e-12);
        assert!(
            (out_q.w - q.w).abs() < 1e-12 && (out_q.j - q.j).abs() < 1e-12,
            "rotation must be preserved by a translation-only set"
        );

        // Change only the rotation; translation must survive.
        assert_eq!(
            collider_set_quat_param(world, handle, COLLIDER_QUAT_ROTATION, 0.0, 0.0, 0.0, 1.0),
            Bool::TRUE
        );
        let out_t = collider_get_translation(world, handle);
        let out_q = collider_get_rotation(world, handle);
        assert!(
            (out_t.x - 7.0).abs() < 1e-12 && (out_t.y - 8.0).abs() < 1e-12,
            "translation must be preserved by a rotation-only set"
        );
        assert!((out_q.w - 1.0).abs() < 1e-12);

        world_destroy(world);
    }
}
