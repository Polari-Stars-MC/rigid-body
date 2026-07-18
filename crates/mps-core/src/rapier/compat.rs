use std::slice;

use hashbrown::HashSet;
use rapier3d::math::Vector;
use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder};

use crate::rapier::ffi::{
    AabbDesc, InteractionGroupsDesc, MAX_OUTPUT_CAPACITY, Quat, QueryFilterDesc,
    RigidBodyHandleRaw, Vec3, WorldHandle, interaction_groups_to_rapier, isometry_from_parts,
    pack_rigid_body_handle, quat_finite, query_filter_from_desc, vec3_finite, vec3_to_rapier,
};

const DYNAMIC_LINEAR_DAMPING: f64 = 0.4;
const DYNAMIC_ANGULAR_DAMPING: f64 = 0.18;
const MAX_DYNAMIC_CUBOIDS: u32 = 100_000;
const MAX_TRIMESH_VERTICES: u32 = 1_000_000;
const MAX_TRIMESH_INDICES: u32 = 3_000_000;

fn valid_aabb(aabb: AabbDesc) -> bool {
    vec3_finite(aabb.mins)
        && vec3_finite(aabb.maxs)
        && aabb.mins.x <= aabb.maxs.x
        && aabb.mins.y <= aabb.maxs.y
        && aabb.mins.z <= aabb.maxs.z
}

#[unsafe(no_mangle)]
pub extern "C" fn world_insert_dynamic_cuboids(
    world: *mut WorldHandle,
    translation: Vec3,
    rotation: Quat,
    linvel: Vec3,
    cuboids: *const f64,
    cuboid_count: u32,
    density: f64,
    friction: f64,
    restitution: f64,
    collision_groups: InteractionGroupsDesc,
    solver_groups: InteractionGroupsDesc,
) -> RigidBodyHandleRaw {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0;
    };
    if cuboids.is_null() || cuboid_count == 0 || cuboid_count > MAX_DYNAMIC_CUBOIDS {
        return 0;
    }
    if !vec3_finite(translation)
        || !quat_finite(rotation)
        || !vec3_finite(linvel)
        || !density.is_finite()
        || !friction.is_finite()
        || !restitution.is_finite()
        || density < 0.0
        || friction < 0.0
        || restitution < 0.0
    {
        return 0;
    }
    let Some(value_count) = (cuboid_count as usize).checked_mul(6) else {
        return 0;
    };

    let cuboids = unsafe { slice::from_raw_parts(cuboids, value_count) };
    let body = RigidBodyBuilder::dynamic()
        .pose(isometry_from_parts(translation, rotation))
        .linvel(vec3_to_rapier(linvel))
        .linear_damping(DYNAMIC_LINEAR_DAMPING)
        .angular_damping(DYNAMIC_ANGULAR_DAMPING)
        .can_sleep(true)
        .ccd_enabled(true)
        .build();
    let body_handle = world.inner.bodies.insert(body);
    let collision_groups = interaction_groups_to_rapier(collision_groups);
    let solver_groups = interaction_groups_to_rapier(solver_groups);
    let mut collider_count = 0usize;

    for cuboid in cuboids.chunks_exact(6) {
        let half_x = cuboid[3];
        let half_y = cuboid[4];
        let half_z = cuboid[5];
        if !cuboid.iter().all(|value| value.is_finite()) {
            continue;
        }
        if half_x <= 1.0E-5 || half_y <= 1.0E-5 || half_z <= 1.0E-5 {
            continue;
        }

        let collider = ColliderBuilder::cuboid(half_x, half_y, half_z)
            .translation(Vector::new(cuboid[0], cuboid[1], cuboid[2]))
            .density(density)
            .friction(friction)
            .restitution(restitution)
            .collision_groups(collision_groups)
            .solver_groups(solver_groups)
            .build();
        world
            .inner
            .colliders
            .insert_with_parent(collider, body_handle, &mut world.inner.bodies);
        collider_count += 1;
    }

    if collider_count == 0 {
        world.inner.bodies.remove(
            body_handle,
            &mut world.inner.islands,
            &mut world.inner.colliders,
            &mut world.inner.impulse_joints,
            &mut world.inner.multibody_joints,
            true,
        );
        return 0;
    }

    pack_rigid_body_handle(body_handle)
}

#[unsafe(no_mangle)]
pub extern "C" fn world_insert_static_trimesh(
    world: *mut WorldHandle,
    vertices_xyz: *const f64,
    vertex_xyz_len: u32,
    indices: *const u32,
    index_len: u32,
    friction: f64,
    restitution: f64,
) -> RigidBodyHandleRaw {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0;
    };
    if vertices_xyz.is_null()
        || indices.is_null()
        || vertex_xyz_len < 9
        || !vertex_xyz_len.is_multiple_of(3)
        || index_len < 3
        || !index_len.is_multiple_of(3)
    {
        return 0;
    }
    let vertex_count = vertex_xyz_len / 3;
    if vertex_count > MAX_TRIMESH_VERTICES || index_len > MAX_TRIMESH_INDICES {
        return 0;
    }
    if !friction.is_finite() || !restitution.is_finite() || friction < 0.0 || restitution < 0.0 {
        return 0;
    }

    let vertices_xyz = unsafe { slice::from_raw_parts(vertices_xyz, vertex_xyz_len as usize) };
    let indices = unsafe { slice::from_raw_parts(indices, index_len as usize) };

    let mut vertices = Vec::with_capacity(vertex_count as usize);
    for chunk in vertices_xyz.chunks_exact(3) {
        if !chunk[0].is_finite() || !chunk[1].is_finite() || !chunk[2].is_finite() {
            return 0;
        }
        vertices.push(Vector::new(chunk[0], chunk[1], chunk[2]));
    }

    let mut triangles = Vec::with_capacity(index_len as usize / 3);
    for chunk in indices.chunks_exact(3) {
        if chunk[0] >= vertex_count || chunk[1] >= vertex_count || chunk[2] >= vertex_count {
            return 0;
        }
        triangles.push([chunk[0], chunk[1], chunk[2]]);
    }

    let body = RigidBodyBuilder::fixed().build();
    let body_handle = world.inner.bodies.insert(body);
    let Ok(collider) = ColliderBuilder::trimesh(vertices, triangles) else {
        world.inner.bodies.remove(
            body_handle,
            &mut world.inner.islands,
            &mut world.inner.colliders,
            &mut world.inner.impulse_joints,
            &mut world.inner.multibody_joints,
            true,
        );
        return 0;
    };
    let collider = collider.friction(friction).restitution(restitution).build();
    world
        .inner
        .colliders
        .insert_with_parent(collider, body_handle, &mut world.inner.bodies);
    pack_rigid_body_handle(body_handle)
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_aabb_rigid_body_count(
    world: *const WorldHandle,
    aabb: AabbDesc,
    filter: QueryFilterDesc,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    if !valid_aabb(aabb) {
        return 0;
    }

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    let mut unique = HashSet::with_capacity(world.inner.bodies.len().min(1024));
    for (collider_handle, _) in query.intersect_aabb_conservative(rapier3d::geometry::Aabb::new(
        vec3_to_rapier(aabb.mins),
        vec3_to_rapier(aabb.maxs),
    )) {
        if let Some(parent) = world
            .inner
            .colliders
            .get(collider_handle)
            .and_then(|collider| collider.parent())
        {
            unique.insert(parent);
        }
    }

    unique.len() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_aabb_rigid_bodies(
    world: *const WorldHandle,
    aabb: AabbDesc,
    filter: QueryFilterDesc,
    out_handles: *mut RigidBodyHandleRaw,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    if out_handles.is_null() || capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
        return 0;
    }
    if !valid_aabb(aabb) {
        return 0;
    }

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    let mut unique = HashSet::with_capacity((capacity as usize).min(world.inner.bodies.len()));
    let mut written = 0usize;
    let out = unsafe { slice::from_raw_parts_mut(out_handles, capacity as usize) };
    for (collider_handle, _) in query.intersect_aabb_conservative(rapier3d::geometry::Aabb::new(
        vec3_to_rapier(aabb.mins),
        vec3_to_rapier(aabb.maxs),
    )) {
        let Some(collider) = world.inner.colliders.get(collider_handle) else {
            continue;
        };
        let Some(parent) = collider.parent() else {
            continue;
        };
        if unique.insert(parent) {
            if written >= out.len() {
                break;
            }
            out[written] = pack_rigid_body_handle(parent);
            written += 1;
        }
    }

    written as u32
}
