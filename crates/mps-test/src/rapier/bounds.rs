#[cfg(test)]
mod tests {
    use mps_core::rapier::bounds::*;
    use mps_core::rapier::collider::{collider_builder_build, world_insert_collider};
    use mps_core::rapier::error::{
        ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, last_error_code,
    };
    use mps_core::rapier::ffi::ColliderMemHandle;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::{Quat, Vec3};

    fn identity_rotation() -> Quat {
        Quat {
            i: 0.0,
            j: 0.0,
            k: 0.0,
            w: 1.0,
        }
    }

    fn assert_bound_hits(
        builder: *mut ColliderMemHandle,
        count: impl FnOnce(*const WorldHandle) -> u32,
    ) {
        assert!(!builder.is_null());
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let collider = world_insert_collider(world, builder);
        assert_ne!(collider, 0);
        mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        assert_eq!(count(world), 1);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn capsule_and_ssv_build() {
        let capsule = Capsule {
            a: Vec3::default(),
            b: Vec3 {
                x: 0.0,
                y: 2.0,
                z: 0.0,
            },
            radius: 0.5,
        };
        assert_bound_hits(
            collider_builder_build(collider_builder_create_capsule(capsule)),
            |world| query_intersect_capsule_count_all(world, capsule),
        );

        let ssv = Ssv {
            a: capsule.a,
            b: capsule.b,
            radius: capsule.radius,
        };
        assert_bound_hits(
            collider_builder_build(collider_builder_create_ssv(ssv)),
            |world| query_intersect_ssv_count_all(world, ssv),
        );
    }

    #[test]
    fn ellipsoid_prism_cylinder_and_shell_build() {
        let ellipsoid = Ellipsoid {
            center: Vec3::default(),
            radii: Vec3 {
                x: 1.0,
                y: 0.5,
                z: 1.5,
            },
            rotation: identity_rotation(),
            segments: 12,
        };
        assert_bound_hits(
            collider_builder_build(collider_builder_create_ellipsoid(ellipsoid)),
            |world| query_intersect_ellipsoid_count_all(world, ellipsoid),
        );

        let prism = Prism {
            center: Vec3::default(),
            radius: 1.0,
            half_height: 0.5,
            sides: 6,
            rotation: identity_rotation(),
        };
        assert_bound_hits(
            collider_builder_build(collider_builder_create_prism(prism)),
            |world| query_intersect_prism_count_all(world, prism),
        );

        let cylinder = Cylinder {
            center: Vec3::default(),
            radius: 1.0,
            half_height: 0.5,
            rotation: identity_rotation(),
        };
        assert_bound_hits(
            collider_builder_build(collider_builder_create_cylinder(cylinder)),
            |world| query_intersect_cylinder_count_all(world, cylinder),
        );

        let shell = SphericalShell {
            center: Vec3::default(),
            inner_radius: 0.5,
            outer_radius: 1.0,
        };
        assert_bound_hits(
            collider_builder_build(collider_builder_create_spherical_shell(shell)),
            |world| query_intersect_spherical_shell_count_all(world, shell),
        );
    }

    #[test]
    fn capsule_and_ssv_reject_invalid_parameters() {
        let valid = Capsule {
            a: Vec3::default(),
            b: Vec3 {
                x: 0.0,
                y: 2.0,
                z: 0.0,
            },
            radius: 0.5,
        };

        // Degenerate segment (a == b).
        let degenerate = Capsule {
            b: Vec3::default(),
            ..valid
        };
        assert!(collider_builder_create_capsule(degenerate).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Non-positive radius.
        let zero_radius = Capsule {
            radius: 0.0,
            ..valid
        };
        assert!(collider_builder_create_capsule(zero_radius).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // NaN endpoint.
        let nan_a = Capsule {
            a: Vec3 {
                x: f64::NAN,
                y: 0.0,
                z: 0.0,
            },
            ..valid
        };
        assert!(collider_builder_create_capsule(nan_a).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // SSV shares the capsule validation.
        let ssv = Ssv {
            a: Vec3::default(),
            b: Vec3::default(),
            radius: 0.5,
        };
        assert!(collider_builder_create_ssv(ssv).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn cylinder_and_spherical_shell_reject_invalid_parameters() {
        let valid_cylinder = Cylinder {
            center: Vec3::default(),
            radius: 1.0,
            half_height: 0.5,
            rotation: identity_rotation(),
        };

        // Non-positive radius.
        let zero_radius = Cylinder {
            radius: 0.0,
            ..valid_cylinder
        };
        assert!(collider_builder_create_cylinder(zero_radius).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Non-positive half height.
        let negative_height = Cylinder {
            half_height: -0.5,
            ..valid_cylinder
        };
        assert!(collider_builder_create_cylinder(negative_height).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Non-finite rotation.
        let nan_rotation = Cylinder {
            rotation: Quat {
                i: f64::NAN,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
            ..valid_cylinder
        };
        assert!(collider_builder_create_cylinder(nan_rotation).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let valid_shell = SphericalShell {
            center: Vec3::default(),
            inner_radius: 0.5,
            outer_radius: 1.0,
        };

        // Non-positive outer radius.
        let zero_outer = SphericalShell {
            outer_radius: 0.0,
            ..valid_shell
        };
        assert!(collider_builder_create_spherical_shell(zero_outer).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Negative inner radius.
        let negative_inner = SphericalShell {
            inner_radius: -0.1,
            ..valid_shell
        };
        assert!(collider_builder_create_spherical_shell(negative_inner).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Inner radius larger than outer radius.
        let swapped = SphericalShell {
            inner_radius: 2.0,
            ..valid_shell
        };
        assert!(collider_builder_create_spherical_shell(swapped).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn ellipsoid_and_prism_reject_invalid_parameters() {
        let valid_ellipsoid = Ellipsoid {
            center: Vec3::default(),
            radii: Vec3 {
                x: 1.0,
                y: 0.5,
                z: 1.5,
            },
            rotation: identity_rotation(),
            segments: 12,
        };

        // Zero axis radius.
        let zero_radius = Ellipsoid {
            radii: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 1.5,
            },
            ..valid_ellipsoid
        };
        assert!(collider_builder_create_ellipsoid(zero_radius).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // NaN radii.
        let nan_radii = Ellipsoid {
            radii: Vec3 {
                x: f64::NAN,
                y: 0.5,
                z: 1.5,
            },
            ..valid_ellipsoid
        };
        assert!(collider_builder_create_ellipsoid(nan_radii).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let valid_prism = Prism {
            center: Vec3::default(),
            radius: 1.0,
            half_height: 0.5,
            sides: 6,
            rotation: identity_rotation(),
        };

        // Fewer than 3 sides.
        let two_sides = Prism {
            sides: 2,
            ..valid_prism
        };
        assert!(collider_builder_create_prism(two_sides).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Non-positive radius.
        let zero_radius = Prism {
            radius: 0.0,
            ..valid_prism
        };
        assert!(collider_builder_create_prism(zero_radius).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Non-positive half height.
        let zero_height = Prism {
            half_height: 0.0,
            ..valid_prism
        };
        assert!(collider_builder_create_prism(zero_height).is_null());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn intersect_count_rejects_null_world_and_invalid_shape() {
        let capsule = Capsule {
            a: Vec3::default(),
            b: Vec3 {
                x: 0.0,
                y: 2.0,
                z: 0.0,
            },
            radius: 0.5,
        };
        assert_eq!(
            query_intersect_capsule_count_all(std::ptr::null(), capsule),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        let world = mps_core::rapier::world::world_create(Vec3::default());

        // Invalid bound shapes report ERR_INVALID_ARGUMENT on a valid world.
        let degenerate = Capsule {
            a: Vec3::default(),
            b: Vec3::default(),
            radius: 0.5,
        };
        assert_eq!(query_intersect_capsule_count_all(world, degenerate), 0);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let bad_ssv = Ssv {
            a: Vec3::default(),
            b: Vec3::default(),
            radius: 0.5,
        };
        assert_eq!(query_intersect_ssv_count_all(world, bad_ssv), 0);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let bad_cylinder = Cylinder {
            center: Vec3::default(),
            radius: 0.0,
            half_height: 0.5,
            rotation: identity_rotation(),
        };
        assert_eq!(query_intersect_cylinder_count_all(world, bad_cylinder), 0);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let bad_shell = SphericalShell {
            center: Vec3::default(),
            inner_radius: 2.0,
            outer_radius: 1.0,
        };
        assert_eq!(
            query_intersect_spherical_shell_count_all(world, bad_shell),
            0
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let bad_ellipsoid = Ellipsoid {
            center: Vec3::default(),
            radii: Vec3::default(),
            rotation: identity_rotation(),
            segments: 12,
        };
        assert_eq!(query_intersect_ellipsoid_count_all(world, bad_ellipsoid), 0);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let bad_prism = Prism {
            center: Vec3::default(),
            radius: 1.0,
            half_height: 0.5,
            sides: 2,
            rotation: identity_rotation(),
        };
        assert_eq!(query_intersect_prism_count_all(world, bad_prism), 0);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn intersect_bound_rejects_null_output_and_bad_capacity() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let capsule = Capsule {
            a: Vec3::default(),
            b: Vec3 {
                x: 0.0,
                y: 2.0,
                z: 0.0,
            },
            radius: 0.5,
        };
        let mut handles = [0u64; 8];

        // Null world takes precedence over every other validation.
        assert_eq!(
            query_intersect_capsule(
                std::ptr::null(),
                capsule,
                QueryFilterDesc::default(),
                std::ptr::null_mut(),
                0,
            ),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Null output buffer.
        assert_eq!(
            query_intersect_capsule(
                world,
                capsule,
                QueryFilterDesc::default(),
                std::ptr::null_mut(),
                8,
            ),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Zero capacity and capacity above MAX_OUTPUT_CAPACITY (1_000_000).
        assert_eq!(
            query_intersect_capsule(
                world,
                capsule,
                QueryFilterDesc::default(),
                handles.as_mut_ptr(),
                0,
            ),
            0
        );
        assert_eq!(last_error_code(), ERR_CAPACITY);
        assert_eq!(
            query_intersect_capsule(
                world,
                capsule,
                QueryFilterDesc::default(),
                handles.as_mut_ptr(),
                1_000_001,
            ),
            0
        );
        assert_eq!(last_error_code(), ERR_CAPACITY);

        // Invalid shape parameters with an otherwise valid call.
        let degenerate = Capsule {
            a: Vec3::default(),
            b: Vec3::default(),
            radius: 0.5,
        };
        assert_eq!(
            query_intersect_capsule(
                world,
                degenerate,
                QueryFilterDesc::default(),
                handles.as_mut_ptr(),
                8,
            ),
            0
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        mps_core::rapier::world::world_destroy(world);
    }
}
