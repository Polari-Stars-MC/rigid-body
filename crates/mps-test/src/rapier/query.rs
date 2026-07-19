#[cfg(test)]
mod tests {
    use mps_core::rapier::query::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::{Quat, Sphere, Vec3};

    #[test]
    fn obb_query_hits_inserted_obb_collider() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let obb = Obb {
            center: Vec3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            half_extents: Vec3 {
                x: 0.5,
                y: 1.0,
                z: 1.5,
            },
            rotation: Quat {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
        };
        let builder = mps_core::rapier::collider::collider_builder_build(
            mps_core::rapier::collider::collider_builder_create_obb(obb),
        );
        assert!(!builder.is_null());

        let collider = mps_core::rapier::collider::world_insert_collider(world, builder);
        assert_ne!(collider, 0);
        mps_core::rapier::world::world_step(world, 1.0 / 60.0);

        assert_eq!(query_intersect_obb_count_all(world, obb), 1);

        let mut handles = [0; 1];
        assert_eq!(
            query_intersect_obb_all(world, obb, handles.as_mut_ptr(), handles.len() as u32),
            1
        );
        assert_eq!(handles[0], collider);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn sphere_query_hits_inserted_sphere_collider() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let sphere = Sphere {
            center: Vec3 {
                x: 2.0,
                y: 3.0,
                z: 4.0,
            },
            radius: 1.25,
        };
        let builder = mps_core::rapier::collider::collider_builder_build(
            mps_core::rapier::collider::collider_builder_create_sphere(sphere),
        );
        assert!(!builder.is_null());

        let collider = mps_core::rapier::collider::world_insert_collider(world, builder);
        assert_ne!(collider, 0);
        mps_core::rapier::world::world_step(world, 1.0 / 60.0);

        assert_eq!(query_intersect_sphere_count_all(world, sphere), 1);

        let mut handles = [0; 1];
        assert_eq!(
            query_intersect_sphere_all(world, sphere, handles.as_mut_ptr(), handles.len() as u32),
            1
        );
        assert_eq!(handles[0], collider);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn point_projection_and_batch_rays_hit_inserted_sphere() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let sphere = Sphere {
            center: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            radius: 1.0,
        };
        let builder = mps_core::rapier::collider::collider_builder_build(
            mps_core::rapier::collider::collider_builder_create_sphere(sphere),
        );
        let collider = mps_core::rapier::collider::world_insert_collider(world, builder);
        mps_core::rapier::world::world_step(world, 1.0 / 60.0);

        let mut projected_collider = 0;
        let projection = query_project_point(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            10.0,
            Bool::TRUE,
            QueryFilterDesc::default(),
            &mut projected_collider,
        );
        assert_eq!(projected_collider, collider);
        assert_eq!(projection.is_inside, Bool::TRUE);
        assert_eq!(
            query_intersect_point_count(
                world,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                QueryFilterDesc::default()
            ),
            1
        );

        let rays = [0.0, 3.0, 0.0, 0.0, -1.0, 0.0, 3.0, 3.0, 0.0, 0.0, -1.0, 0.0];
        let mut hits = [RayHit::default(); 2];
        assert_eq!(
            query_cast_rays(
                world,
                rays.as_ptr(),
                2,
                10.0,
                Bool::TRUE,
                QueryFilterDesc::default(),
                hits.as_mut_ptr(),
                hits.len() as u32,
            ),
            2
        );
        assert_eq!(hits[0].collider, collider);
        assert_eq!(hits[1].collider, 0);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn batch_intersection_counts_return_per_query_counts() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let sphere = Sphere {
            center: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            radius: 1.0,
        };
        let builder = mps_core::rapier::collider::collider_builder_build(
            mps_core::rapier::collider::collider_builder_create_sphere(sphere),
        );
        let collider = mps_core::rapier::collider::world_insert_collider(world, builder);
        assert_ne!(collider, 0);
        mps_core::rapier::world::world_step(world, 1.0 / 60.0);

        let aabbs = [
            AabbDesc {
                mins: Vec3 {
                    x: -2.0,
                    y: -2.0,
                    z: -2.0,
                },
                maxs: Vec3 {
                    x: 2.0,
                    y: 2.0,
                    z: 2.0,
                },
            },
            AabbDesc {
                mins: Vec3 {
                    x: 10.0,
                    y: 10.0,
                    z: 10.0,
                },
                maxs: Vec3 {
                    x: 11.0,
                    y: 11.0,
                    z: 11.0,
                },
            },
        ];
        let mut counts = [0; 2];
        assert_eq!(
            query_intersect_aabb_counts(
                world,
                aabbs.as_ptr(),
                aabbs.len() as u32,
                QueryFilterDesc::default(),
                counts.as_mut_ptr(),
                counts.len() as u32,
            ),
            2
        );
        assert_eq!(counts, [1, 0]);

        let spheres = [
            sphere,
            Sphere {
                center: Vec3 {
                    x: 10.0,
                    y: 10.0,
                    z: 10.0,
                },
                radius: 1.0,
            },
        ];
        counts = [0; 2];
        assert_eq!(
            query_intersect_sphere_counts(
                world,
                spheres.as_ptr(),
                spheres.len() as u32,
                QueryFilterDesc::default(),
                counts.as_mut_ptr(),
                counts.len() as u32,
            ),
            2
        );
        assert_eq!(counts, [1, 0]);

        let obbs = [
            Obb {
                center: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                half_extents: Vec3 {
                    x: 1.5,
                    y: 1.5,
                    z: 1.5,
                },
                rotation: Quat {
                    i: 0.0,
                    j: 0.0,
                    k: 0.0,
                    w: 1.0,
                },
            },
            Obb {
                center: Vec3 {
                    x: 10.0,
                    y: 10.0,
                    z: 10.0,
                },
                half_extents: Vec3 {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
                rotation: Quat {
                    i: 0.0,
                    j: 0.0,
                    k: 0.0,
                    w: 1.0,
                },
            },
        ];
        counts = [0; 2];
        assert_eq!(
            query_intersect_obb_counts(
                world,
                obbs.as_ptr(),
                obbs.len() as u32,
                QueryFilterDesc::default(),
                counts.as_mut_ptr(),
                counts.len() as u32,
            ),
            2
        );
        assert_eq!(counts, [1, 0]);

        mps_core::rapier::world::world_destroy(world);
    }
}



