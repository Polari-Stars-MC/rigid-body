#[cfg(test)]
mod tests {
    use mps_core::rapier::bounds::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::collider::{collider_builder_build, world_insert_collider};
    use mps_core::rapier::ffi::{Quat, Vec3};
    use rapier3d::prelude::Collider;

    fn identity_rotation() -> Quat {
        Quat {
            i: 0.0,
            j: 0.0,
            k: 0.0,
            w: 1.0,
        }
    }

    fn assert_bound_hits(builder: *mut Collider, count: impl FnOnce(*const WorldHandle) -> u32) {
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
}



