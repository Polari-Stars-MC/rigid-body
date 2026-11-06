#[cfg(test)]
mod tests {
    use mps_core::rapier::compat::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::world::{
        world_create, world_destroy, world_get_rigid_body_set_size, world_step,
    };

    fn make_world() -> *mut WorldHandle {
        world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        })
    }

    fn identity() -> Quat {
        Quat {
            i: 0.0,
            j: 0.0,
            k: 0.0,
            w: 1.0,
        }
    }

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

    /// One cuboid: local offset (0,0,0), half extents (1,1,1).
    fn one_cuboid() -> [f64; 6] {
        [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
    }

    fn insert_one(world: *mut WorldHandle, cuboids: &[f64], count: u32) -> RigidBodyHandleRaw {
        world_insert_dynamic_cuboids(
            world,
            Vec3::default(),
            identity(),
            Vec3::default(),
            cuboids.as_ptr(),
            count,
            1.0,
            0.5,
            0.0,
            InteractionGroupsDesc::default(),
            InteractionGroupsDesc::default(),
        )
    }

    // ---- world_insert_dynamic_cuboids ----

    #[test]
    fn insert_dynamic_cuboids_returns_handle() {
        let world = make_world();
        let cuboids = [
            0.0, 0.0, 0.0, 1.0, 1.0, 1.0, //
            3.0, 0.0, 0.0, 0.5, 0.5, 0.5,
        ];
        let handle = insert_one(world, &cuboids, 2);
        assert_ne!(handle, 0);
        assert_eq!(world_get_rigid_body_set_size(world), 1);
        world_destroy(world);
    }

    #[test]
    fn insert_dynamic_cuboids_skips_invalid_entries() {
        let world = make_world();
        // Second cuboid has degenerate half extents and must be skipped.
        let cuboids = [
            0.0, 0.0, 0.0, 1.0, 1.0, 1.0, //
            3.0, 0.0, 0.0, 0.0, 0.5, 0.5,
        ];
        let handle = insert_one(world, &cuboids, 2);
        assert_ne!(handle, 0);
        world_destroy(world);
    }

    #[test]
    fn insert_dynamic_cuboids_all_invalid_rolls_back_body() {
        let world = make_world();
        let cuboids = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let handle = insert_one(world, &cuboids, 1);
        assert_eq!(handle, 0);
        // The orphaned body must have been removed again.
        assert_eq!(world_get_rigid_body_set_size(world), 0);
        world_destroy(world);
    }

    #[test]
    fn insert_dynamic_cuboids_rejects_null_world() {
        let cuboids = one_cuboid();
        let handle = world_insert_dynamic_cuboids(
            std::ptr::null_mut(),
            Vec3::default(),
            identity(),
            Vec3::default(),
            cuboids.as_ptr(),
            1,
            1.0,
            0.5,
            0.0,
            InteractionGroupsDesc::default(),
            InteractionGroupsDesc::default(),
        );
        assert_eq!(handle, 0);
    }

    #[test]
    fn insert_dynamic_cuboids_rejects_null_data() {
        let world = make_world();
        let handle = world_insert_dynamic_cuboids(
            world,
            Vec3::default(),
            identity(),
            Vec3::default(),
            std::ptr::null(),
            1,
            1.0,
            0.5,
            0.0,
            InteractionGroupsDesc::default(),
            InteractionGroupsDesc::default(),
        );
        assert_eq!(handle, 0);
        world_destroy(world);
    }

    #[test]
    fn insert_dynamic_cuboids_rejects_bad_counts() {
        let world = make_world();
        let cuboids = one_cuboid();
        // Zero count.
        assert_eq!(insert_one(world, &cuboids, 0), 0);
        // Above the hard limit; rejected before reading the buffer.
        assert_eq!(insert_one(world, &cuboids, 100_001), 0);
        assert_eq!(world_get_rigid_body_set_size(world), 0);
        world_destroy(world);
    }

    #[test]
    fn insert_dynamic_cuboids_rejects_non_finite_or_negative_params() {
        let world = make_world();
        let cuboids = one_cuboid();
        let groups = InteractionGroupsDesc::default();

        // NaN translation.
        assert_eq!(
            world_insert_dynamic_cuboids(
                world,
                Vec3 {
                    x: f64::NAN,
                    y: 0.0,
                    z: 0.0
                },
                identity(),
                Vec3::default(),
                cuboids.as_ptr(),
                1,
                1.0,
                0.5,
                0.0,
                groups,
                groups,
            ),
            0
        );
        // Negative density.
        assert_eq!(
            world_insert_dynamic_cuboids(
                world,
                Vec3::default(),
                identity(),
                Vec3::default(),
                cuboids.as_ptr(),
                1,
                -1.0,
                0.5,
                0.0,
                groups,
                groups,
            ),
            0
        );
        // Infinite friction.
        assert_eq!(
            world_insert_dynamic_cuboids(
                world,
                Vec3::default(),
                identity(),
                Vec3::default(),
                cuboids.as_ptr(),
                1,
                1.0,
                f64::INFINITY,
                0.0,
                groups,
                groups,
            ),
            0
        );
        // Negative restitution.
        assert_eq!(
            world_insert_dynamic_cuboids(
                world,
                Vec3::default(),
                identity(),
                Vec3::default(),
                cuboids.as_ptr(),
                1,
                1.0,
                0.5,
                -0.5,
                groups,
                groups,
            ),
            0
        );
        assert_eq!(world_get_rigid_body_set_size(world), 0);
        world_destroy(world);
    }

    // ---- world_insert_static_trimesh ----

    fn triangle_vertices() -> [f64; 9] {
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    }

    #[test]
    fn insert_static_trimesh_returns_handle() {
        let world = make_world();
        let vertices = triangle_vertices();
        let indices = [0u32, 1, 2];
        let handle =
            world_insert_static_trimesh(world, vertices.as_ptr(), 9, indices.as_ptr(), 3, 0.5, 0.0);
        assert_ne!(handle, 0);
        assert_eq!(world_get_rigid_body_set_size(world), 1);
        world_destroy(world);
    }

    #[test]
    fn insert_static_trimesh_rejects_null_pointers() {
        let world = make_world();
        let vertices = triangle_vertices();
        let indices = [0u32, 1, 2];
        assert_eq!(
            world_insert_static_trimesh(world, std::ptr::null(), 9, indices.as_ptr(), 3, 0.5, 0.0),
            0
        );
        assert_eq!(
            world_insert_static_trimesh(world, vertices.as_ptr(), 9, std::ptr::null(), 3, 0.5, 0.0),
            0
        );
        assert_eq!(
            world_insert_static_trimesh(
                std::ptr::null_mut(),
                vertices.as_ptr(),
                9,
                indices.as_ptr(),
                3,
                0.5,
                0.0
            ),
            0
        );
        world_destroy(world);
    }

    #[test]
    fn insert_static_trimesh_rejects_bad_lengths() {
        let world = make_world();
        let vertices = triangle_vertices();
        let indices = [0u32, 1, 2];
        // Fewer than one triangle worth of vertices.
        assert_eq!(
            world_insert_static_trimesh(world, vertices.as_ptr(), 8, indices.as_ptr(), 3, 0.5, 0.0),
            0
        );
        // Vertex length not a multiple of 3.
        assert_eq!(
            world_insert_static_trimesh(
                world,
                vertices.as_ptr(),
                10,
                indices.as_ptr(),
                3,
                0.5,
                0.0
            ),
            0
        );
        // Index length not a multiple of 3.
        assert_eq!(
            world_insert_static_trimesh(world, vertices.as_ptr(), 9, indices.as_ptr(), 2, 0.5, 0.0),
            0
        );
        assert_eq!(world_get_rigid_body_set_size(world), 0);
        world_destroy(world);
    }

    #[test]
    fn insert_static_trimesh_rejects_out_of_range_index() {
        let world = make_world();
        let vertices = triangle_vertices();
        let indices = [0u32, 1, 3]; // only 3 vertices exist
        assert_eq!(
            world_insert_static_trimesh(world, vertices.as_ptr(), 9, indices.as_ptr(), 3, 0.5, 0.0),
            0
        );
        assert_eq!(world_get_rigid_body_set_size(world), 0);
        world_destroy(world);
    }

    #[test]
    fn insert_static_trimesh_rejects_non_finite_vertex() {
        let world = make_world();
        let vertices = [0.0, 0.0, 0.0, 1.0, f64::NAN, 0.0, 0.0, 1.0, 0.0];
        let indices = [0u32, 1, 2];
        assert_eq!(
            world_insert_static_trimesh(world, vertices.as_ptr(), 9, indices.as_ptr(), 3, 0.5, 0.0),
            0
        );
        world_destroy(world);
    }

    #[test]
    fn insert_static_trimesh_rejects_negative_friction() {
        let world = make_world();
        let vertices = triangle_vertices();
        let indices = [0u32, 1, 2];
        assert_eq!(
            world_insert_static_trimesh(
                world,
                vertices.as_ptr(),
                9,
                indices.as_ptr(),
                3,
                -0.5,
                0.0
            ),
            0
        );
        world_destroy(world);
    }

    // ---- query_intersect_aabb_rigid_body_count / query_intersect_aabb_rigid_bodies ----

    fn insert_cuboid_body_at(world: *mut WorldHandle, x: f64) -> RigidBodyHandleRaw {
        let cuboids = one_cuboid();
        let handle = world_insert_dynamic_cuboids(
            world,
            Vec3 { x, y: 0.0, z: 0.0 },
            identity(),
            Vec3::default(),
            cuboids.as_ptr(),
            1,
            1.0,
            0.5,
            0.0,
            InteractionGroupsDesc::default(),
            InteractionGroupsDesc::default(),
        );
        assert_ne!(handle, 0);
        handle
    }

    #[test]
    fn aabb_query_finds_inserted_body() {
        let world = make_world();
        let handle = insert_cuboid_body_at(world, 0.0);
        world_step(world, 1.0 / 60.0);

        let filter = QueryFilterDesc::default();
        assert_eq!(
            query_intersect_aabb_rigid_body_count(world, aabb(-5.0, 5.0), filter),
            1
        );

        let mut out = [0u64; 4];
        let written =
            query_intersect_aabb_rigid_bodies(world, aabb(-5.0, 5.0), filter, out.as_mut_ptr(), 4);
        assert_eq!(written, 1);
        assert_eq!(out[0], handle);
        world_destroy(world);
    }

    #[test]
    fn aabb_query_counts_each_body_once() {
        let world = make_world();
        insert_cuboid_body_at(world, 0.0);
        insert_cuboid_body_at(world, 10.0);
        world_step(world, 1.0 / 60.0);

        let filter = QueryFilterDesc::default();
        assert_eq!(
            query_intersect_aabb_rigid_body_count(world, aabb(-5.0, 15.0), filter),
            2
        );
        // Misses bodies outside the box.
        assert_eq!(
            query_intersect_aabb_rigid_body_count(world, aabb(-5.0, 5.0), filter),
            1
        );

        // Output capacity caps the number of written handles.
        let mut out = [0u64; 1];
        let written =
            query_intersect_aabb_rigid_bodies(world, aabb(-5.0, 15.0), filter, out.as_mut_ptr(), 1);
        assert_eq!(written, 1);
        world_destroy(world);
    }

    #[test]
    fn aabb_query_rejects_invalid_arguments() {
        let world = make_world();
        let filter = QueryFilterDesc::default();
        let mut out = [0u64; 4];

        // Null world.
        assert_eq!(
            query_intersect_aabb_rigid_body_count(std::ptr::null(), aabb(-5.0, 5.0), filter),
            0
        );
        assert_eq!(
            query_intersect_aabb_rigid_bodies(
                std::ptr::null(),
                aabb(-5.0, 5.0),
                filter,
                out.as_mut_ptr(),
                4
            ),
            0
        );
        // Inverted AABB (mins > maxs).
        assert_eq!(
            query_intersect_aabb_rigid_body_count(world, aabb(5.0, -5.0), filter),
            0
        );
        // Null output buffer / zero capacity.
        assert_eq!(
            query_intersect_aabb_rigid_bodies(
                world,
                aabb(-5.0, 5.0),
                filter,
                std::ptr::null_mut(),
                4
            ),
            0
        );
        assert_eq!(
            query_intersect_aabb_rigid_bodies(world, aabb(-5.0, 5.0), filter, out.as_mut_ptr(), 0),
            0
        );
        world_destroy(world);
    }
}
