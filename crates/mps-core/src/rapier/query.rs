use rapier3d::geometry::{Aabb, Ray};
use rapier3d::parry::shape::FeatureId;
use rapier3d::prelude::SharedShape;

use crate::rapier::error::{ERR_CAPACITY, ERR_NULL_POINTER, clear_error, set_error};
use crate::rapier::ffi::{
    AabbDesc, Bool, ColliderHandleRaw, MAX_OUTPUT_CAPACITY, Obb, PointProjection, QueryFilterDesc,
    RayHit, ShapeCastHit, ShapeCastOptionsDesc, ShapeDesc, Sphere, Vec3, WorldHandle,
    pack_collider_handle, quat_finite, query_filter_from_desc, shape_cast_options_to_rapier,
    shape_desc_valid, shape_from_desc, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

fn aabb_to_rapier(aabb: AabbDesc) -> Aabb {
    Aabb::new(vec3_to_rapier(aabb.mins), vec3_to_rapier(aabb.maxs))
}

fn aabb_valid(aabb: AabbDesc) -> bool {
    vec3_finite(aabb.mins)
        && vec3_finite(aabb.maxs)
        && aabb.mins.x <= aabb.maxs.x
        && aabb.mins.y <= aabb.maxs.y
        && aabb.mins.z <= aabb.maxs.z
}

fn feature_id_to_u32(feature: FeatureId) -> u32 {
    match feature {
        FeatureId::Unknown => 0,
        FeatureId::Vertex(id) => 0x1000_0000 | id,
        FeatureId::Edge(id) => 0x2000_0000 | id,
        FeatureId::Face(id) => 0x3000_0000 | id,
    }
}

fn obb_shape(obb: Obb) -> Option<SharedShape> {
    if !vec3_finite(obb.center)
        || !vec3_finite(obb.half_extents)
        || !quat_finite(obb.rotation)
        || obb.half_extents.x <= 0.0
        || obb.half_extents.y <= 0.0
        || obb.half_extents.z <= 0.0
    {
        return None;
    }

    Some(SharedShape::cuboid(
        obb.half_extents.x,
        obb.half_extents.y,
        obb.half_extents.z,
    ))
}

fn sphere_shape(sphere: Sphere) -> Option<SharedShape> {
    if !vec3_finite(sphere.center) || !sphere.radius.is_finite() || sphere.radius <= 0.0 {
        return None;
    }

    Some(SharedShape::ball(sphere.radius))
}

fn identity_rotation() -> crate::rapier::ffi::Quat {
    crate::rapier::ffi::Quat {
        i: 0.0,
        j: 0.0,
        k: 0.0,
        w: 1.0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn query_cast_ray(
    world: *const WorldHandle,
    origin: Vec3,
    direction: Vec3,
    max_toi: f64,
    solid: Bool,
    filter: QueryFilterDesc,
) -> RayHit {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RayHit::default();
    };
    if !vec3_finite(origin) || !vec3_finite(direction) || !max_toi.is_finite() || max_toi < 0.0 {
        return RayHit::default();
    }

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );
    let ray = Ray::new(vec3_to_rapier(origin), vec3_to_rapier(direction));

    query
        .cast_ray_and_get_normal(&ray, max_toi, solid.0 != 0)
        .map(|(handle, hit)| RayHit {
            collider: pack_collider_handle(handle),
            time_of_impact: hit.time_of_impact,
            normal: vec3_from_rapier(hit.normal),
            feature: feature_id_to_u32(hit.feature),
        })
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn query_cast_ray_out(
    world: *const WorldHandle,
    origin: Vec3,
    direction: Vec3,
    max_toi: f64,
    solid: Bool,
    filter: QueryFilterDesc,
    out_hit: *mut RayHit,
) -> ColliderHandleRaw {
    let hit = query_cast_ray(world, origin, direction, max_toi, solid, filter);
    if let Some(out_hit) = unsafe { out_hit.as_mut() } {
        *out_hit = hit;
    }
    hit.collider
}

#[unsafe(no_mangle)]
pub extern "C" fn query_cast_rays(
    world: *const WorldHandle,
    rays: *const f64,
    ray_count: u32,
    max_toi: f64,
    solid: Bool,
    filter: QueryFilterDesc,
    out_hits: *mut RayHit,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    };
    if rays.is_null() || out_hits.is_null() {
        set_error(ERR_NULL_POINTER, "ray input or output is null");
        return 0;
    }
    if ray_count == 0 || capacity < ray_count || ray_count > MAX_OUTPUT_CAPACITY {
        set_error(ERR_CAPACITY, "invalid ray batch capacity");
        return 0;
    }
    let Some(ray_value_count) = (ray_count as usize).checked_mul(6) else {
        set_error(ERR_CAPACITY, "ray batch input capacity overflow");
        return 0;
    };

    let rays = unsafe { std::slice::from_raw_parts(rays, ray_value_count) };
    let hits = unsafe { std::slice::from_raw_parts_mut(out_hits, capacity as usize) };
    let mut written = 0usize;
    for (index, hit) in hits.iter_mut().enumerate().take(ray_count as usize) {
        let offset = index * 6;
        *hit = query_cast_ray(
            world,
            Vec3 {
                x: rays[offset],
                y: rays[offset + 1],
                z: rays[offset + 2],
            },
            Vec3 {
                x: rays[offset + 3],
                y: rays[offset + 4],
                z: rays[offset + 5],
            },
            max_toi,
            solid,
            filter,
        );
        written += 1;
    }

    clear_error();
    written as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_project_point(
    world: *const WorldHandle,
    point: Vec3,
    max_dist: f64,
    solid: Bool,
    filter: QueryFilterDesc,
    out_collider: *mut ColliderHandleRaw,
) -> PointProjection {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return PointProjection::default();
    };
    if !vec3_finite(point) || !max_dist.is_finite() || max_dist < 0.0 {
        return PointProjection::default();
    }

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    let Some((handle, projection)) =
        query.project_point(vec3_to_rapier(point), max_dist, solid.0 != 0)
    else {
        return PointProjection::default();
    };

    if let Some(out_collider) = unsafe { out_collider.as_mut() } {
        *out_collider = pack_collider_handle(handle);
    }

    PointProjection {
        point: vec3_from_rapier(projection.point),
        is_inside: projection.is_inside.into(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn query_project_point_out(
    world: *const WorldHandle,
    point: Vec3,
    max_dist: f64,
    solid: Bool,
    filter: QueryFilterDesc,
    out_collider: *mut ColliderHandleRaw,
    out_projection: *mut PointProjection,
) -> ColliderHandleRaw {
    let projection = query_project_point(world, point, max_dist, solid, filter, out_collider);
    let collider = if let Some(out_collider) = unsafe { out_collider.as_ref() } {
        *out_collider
    } else {
        0
    };
    if let Some(out_projection) = unsafe { out_projection.as_mut() } {
        *out_projection = projection;
    }
    collider
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_point_count(
    world: *const WorldHandle,
    point: Vec3,
    filter: QueryFilterDesc,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    if !vec3_finite(point) {
        return 0;
    }

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    query.intersect_point(vec3_to_rapier(point)).count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_aabb_count(
    world: *const WorldHandle,
    aabb: AabbDesc,
    filter: QueryFilterDesc,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    if !aabb_valid(aabb) {
        return 0;
    }

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    query
        .intersect_aabb_conservative(aabb_to_rapier(aabb))
        .count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_aabb(
    world: *const WorldHandle,
    aabb: AabbDesc,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    if out_handles.is_null() || capacity == 0 || capacity > MAX_OUTPUT_CAPACITY || !aabb_valid(aabb)
    {
        return 0;
    }

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    let out = unsafe { std::slice::from_raw_parts_mut(out_handles, capacity as usize) };
    let mut written = 0usize;
    for (handle, _) in query.intersect_aabb_conservative(aabb_to_rapier(aabb)) {
        if written >= out.len() {
            break;
        }
        out[written] = pack_collider_handle(handle);
        written += 1;
    }

    written as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_aabb_count_all(world: *const WorldHandle, aabb: AabbDesc) -> u32 {
    query_intersect_aabb_count(world, aabb, QueryFilterDesc::default())
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_aabb_counts(
    world: *const WorldHandle,
    aabbs: *const AabbDesc,
    query_count: u32,
    filter: QueryFilterDesc,
    out_counts: *mut u32,
    capacity: u32,
) -> u32 {
    if world.is_null() {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    }
    if aabbs.is_null() || out_counts.is_null() {
        set_error(ERR_NULL_POINTER, "AABB input or count output is null");
        return 0;
    }
    if query_count == 0 || capacity < query_count || query_count > MAX_OUTPUT_CAPACITY {
        set_error(ERR_CAPACITY, "invalid AABB batch capacity");
        return 0;
    }

    let aabbs = unsafe { std::slice::from_raw_parts(aabbs, query_count as usize) };
    let counts = unsafe { std::slice::from_raw_parts_mut(out_counts, capacity as usize) };
    for index in 0..query_count as usize {
        counts[index] = query_intersect_aabb_count(world, aabbs[index], filter);
    }

    clear_error();
    query_count
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_obb_count(
    world: *const WorldHandle,
    obb: Obb,
    filter: QueryFilterDesc,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    let Some(shape) = obb_shape(obb) else {
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    query
        .intersect_shape(
            crate::rapier::ffi::isometry_from_parts(obb.center, obb.rotation),
            shape.as_ref(),
        )
        .count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_obb_count_all(world: *const WorldHandle, obb: Obb) -> u32 {
    query_intersect_obb_count(world, obb, QueryFilterDesc::default())
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_obb_counts(
    world: *const WorldHandle,
    obbs: *const Obb,
    query_count: u32,
    filter: QueryFilterDesc,
    out_counts: *mut u32,
    capacity: u32,
) -> u32 {
    if world.is_null() {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    }
    if obbs.is_null() || out_counts.is_null() {
        set_error(ERR_NULL_POINTER, "OBB input or count output is null");
        return 0;
    }
    if query_count == 0 || capacity < query_count || query_count > MAX_OUTPUT_CAPACITY {
        set_error(ERR_CAPACITY, "invalid OBB batch capacity");
        return 0;
    }

    let obbs = unsafe { std::slice::from_raw_parts(obbs, query_count as usize) };
    let counts = unsafe { std::slice::from_raw_parts_mut(out_counts, capacity as usize) };
    for index in 0..query_count as usize {
        counts[index] = query_intersect_obb_count(world, obbs[index], filter);
    }

    clear_error();
    query_count
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_obb(
    world: *const WorldHandle,
    obb: Obb,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    if out_handles.is_null() || capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
        return 0;
    }
    let Some(shape) = obb_shape(obb) else {
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    let out = unsafe { std::slice::from_raw_parts_mut(out_handles, capacity as usize) };
    let mut written = 0usize;
    for (handle, _) in query.intersect_shape(
        crate::rapier::ffi::isometry_from_parts(obb.center, obb.rotation),
        shape.as_ref(),
    ) {
        if written >= out.len() {
            break;
        }
        out[written] = pack_collider_handle(handle);
        written += 1;
    }

    written as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_obb_all(
    world: *const WorldHandle,
    obb: Obb,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    query_intersect_obb(
        world,
        obb,
        QueryFilterDesc::default(),
        out_handles,
        capacity,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_sphere_count(
    world: *const WorldHandle,
    sphere: Sphere,
    filter: QueryFilterDesc,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    let Some(shape) = sphere_shape(sphere) else {
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    query
        .intersect_shape(
            crate::rapier::ffi::isometry_from_parts(sphere.center, identity_rotation()),
            shape.as_ref(),
        )
        .count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_sphere_count_all(
    world: *const WorldHandle,
    sphere: Sphere,
) -> u32 {
    query_intersect_sphere_count(world, sphere, QueryFilterDesc::default())
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_sphere_counts(
    world: *const WorldHandle,
    spheres: *const Sphere,
    query_count: u32,
    filter: QueryFilterDesc,
    out_counts: *mut u32,
    capacity: u32,
) -> u32 {
    if world.is_null() {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    }
    if spheres.is_null() || out_counts.is_null() {
        set_error(ERR_NULL_POINTER, "sphere input or count output is null");
        return 0;
    }
    if query_count == 0 || capacity < query_count || query_count > MAX_OUTPUT_CAPACITY {
        set_error(ERR_CAPACITY, "invalid sphere batch capacity");
        return 0;
    }

    let spheres = unsafe { std::slice::from_raw_parts(spheres, query_count as usize) };
    let counts = unsafe { std::slice::from_raw_parts_mut(out_counts, capacity as usize) };
    for index in 0..query_count as usize {
        counts[index] = query_intersect_sphere_count(world, spheres[index], filter);
    }

    clear_error();
    query_count
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_sphere(
    world: *const WorldHandle,
    sphere: Sphere,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    if out_handles.is_null() || capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
        return 0;
    }
    let Some(shape) = sphere_shape(sphere) else {
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    let out = unsafe { std::slice::from_raw_parts_mut(out_handles, capacity as usize) };
    let mut written = 0usize;
    for (handle, _) in query.intersect_shape(
        crate::rapier::ffi::isometry_from_parts(sphere.center, identity_rotation()),
        shape.as_ref(),
    ) {
        if written >= out.len() {
            break;
        }
        out[written] = pack_collider_handle(handle);
        written += 1;
    }

    written as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_sphere_all(
    world: *const WorldHandle,
    sphere: Sphere,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    query_intersect_sphere(
        world,
        sphere,
        QueryFilterDesc::default(),
        out_handles,
        capacity,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_aabb_rigid_body_count_all(
    world: *const WorldHandle,
    aabb: AabbDesc,
) -> u32 {
    crate::rapier::compat::query_intersect_aabb_rigid_body_count(
        world,
        aabb,
        QueryFilterDesc::default(),
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_aabb_rigid_bodies_all(
    world: *const WorldHandle,
    aabb: AabbDesc,
    out_handles: *mut crate::rapier::ffi::RigidBodyHandleRaw,
    capacity: u32,
) -> u32 {
    crate::rapier::compat::query_intersect_aabb_rigid_bodies(
        world,
        aabb,
        QueryFilterDesc::default(),
        out_handles,
        capacity,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn query_cast_shape(
    world: *const WorldHandle,
    shape_desc: ShapeDesc,
    translation: Vec3,
    rotation: crate::rapier::ffi::Quat,
    velocity: Vec3,
    options: ShapeCastOptionsDesc,
    filter: QueryFilterDesc,
) -> ShapeCastHit {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return ShapeCastHit::default();
    };
    if !shape_desc_valid(shape_desc)
        || !vec3_finite(translation)
        || !quat_finite(rotation)
        || !vec3_finite(velocity)
        || !options.max_time_of_impact.is_finite()
        || !options.target_distance.is_finite()
        || options.max_time_of_impact < 0.0
        || options.target_distance < 0.0
    {
        return ShapeCastHit::default();
    }

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );
    let shape = shape_from_desc(shape_desc);

    query
        .cast_shape(
            &crate::rapier::ffi::isometry_from_parts(translation, rotation),
            vec3_to_rapier(velocity),
            shape.as_ref(),
            shape_cast_options_to_rapier(options),
        )
        .map(|(handle, hit)| ShapeCastHit {
            collider: pack_collider_handle(handle),
            time_of_impact: hit.time_of_impact,
            witness1: vec3_from_rapier(hit.witness1),
            witness2: vec3_from_rapier(hit.witness2),
            normal1: vec3_from_rapier(hit.normal1),
            normal2: vec3_from_rapier(hit.normal2),
            status: hit.status as u32,
        })
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn query_cast_shape_out(
    world: *const WorldHandle,
    shape_desc: ShapeDesc,
    translation: Vec3,
    rotation: crate::rapier::ffi::Quat,
    velocity: Vec3,
    options: ShapeCastOptionsDesc,
    filter: QueryFilterDesc,
    out_hit: *mut ShapeCastHit,
) -> ColliderHandleRaw {
    let hit = query_cast_shape(
        world,
        shape_desc,
        translation,
        rotation,
        velocity,
        options,
        filter,
    );
    if let Some(out_hit) = unsafe { out_hit.as_mut() } {
        *out_hit = hit;
    }
    hit.collider
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::{Quat, Sphere, Vec3};

    #[test]
    fn obb_query_hits_inserted_obb_collider() {
        let world = crate::rapier::world::world_create(Vec3::default());
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
        let builder = crate::rapier::collider::collider_builder_build(
            crate::rapier::collider::collider_builder_create_obb(obb),
        );
        assert!(!builder.is_null());

        let collider = crate::rapier::collider::world_insert_collider(world, builder);
        assert_ne!(collider, 0);
        crate::rapier::world::world_step(world, 1.0 / 60.0);

        assert_eq!(query_intersect_obb_count_all(world, obb), 1);

        let mut handles = [0; 1];
        assert_eq!(
            query_intersect_obb_all(world, obb, handles.as_mut_ptr(), handles.len() as u32),
            1
        );
        assert_eq!(handles[0], collider);

        crate::rapier::world::world_destroy(world);
    }

    #[test]
    fn sphere_query_hits_inserted_sphere_collider() {
        let world = crate::rapier::world::world_create(Vec3::default());
        let sphere = Sphere {
            center: Vec3 {
                x: 2.0,
                y: 3.0,
                z: 4.0,
            },
            radius: 1.25,
        };
        let builder = crate::rapier::collider::collider_builder_build(
            crate::rapier::collider::collider_builder_create_sphere(sphere),
        );
        assert!(!builder.is_null());

        let collider = crate::rapier::collider::world_insert_collider(world, builder);
        assert_ne!(collider, 0);
        crate::rapier::world::world_step(world, 1.0 / 60.0);

        assert_eq!(query_intersect_sphere_count_all(world, sphere), 1);

        let mut handles = [0; 1];
        assert_eq!(
            query_intersect_sphere_all(world, sphere, handles.as_mut_ptr(), handles.len() as u32),
            1
        );
        assert_eq!(handles[0], collider);

        crate::rapier::world::world_destroy(world);
    }

    #[test]
    fn point_projection_and_batch_rays_hit_inserted_sphere() {
        let world = crate::rapier::world::world_create(Vec3::default());
        let sphere = Sphere {
            center: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            radius: 1.0,
        };
        let builder = crate::rapier::collider::collider_builder_build(
            crate::rapier::collider::collider_builder_create_sphere(sphere),
        );
        let collider = crate::rapier::collider::world_insert_collider(world, builder);
        crate::rapier::world::world_step(world, 1.0 / 60.0);

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

        crate::rapier::world::world_destroy(world);
    }

    #[test]
    fn batch_intersection_counts_return_per_query_counts() {
        let world = crate::rapier::world::world_create(Vec3::default());
        let sphere = Sphere {
            center: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            radius: 1.0,
        };
        let builder = crate::rapier::collider::collider_builder_build(
            crate::rapier::collider::collider_builder_create_sphere(sphere),
        );
        let collider = crate::rapier::collider::world_insert_collider(world, builder);
        assert_ne!(collider, 0);
        crate::rapier::world::world_step(world, 1.0 / 60.0);

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

        crate::rapier::world::world_destroy(world);
    }
}
