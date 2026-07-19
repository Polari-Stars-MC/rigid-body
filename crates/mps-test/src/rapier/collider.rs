#[cfg(test)]
mod tests {
    use smallvec::SmallVec;
    use mps_core::rapier::collider::*;
    use mps_core::rapier::ffi::*;

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
}







