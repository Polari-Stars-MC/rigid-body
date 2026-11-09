use std::slice;

use hashbrown::HashSet;
use rapier3d::math::Vector;
use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder};

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
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

/// Insert a dynamic rigid body built from a list of cuboids.
///
/// # Safety
///
/// `world` must be a valid, live world pointer; `cuboids` must point to at
/// least 6×cuboid_count readable f64s (center xyz + half-extents xyz per
/// cuboid).
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
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if cuboids.is_null() {
            set_error(ERR_NULL_POINTER, "cuboid input is null");
            return 0;
        }
        if cuboid_count == 0 || cuboid_count > MAX_DYNAMIC_CUBOIDS {
            set_error(ERR_CAPACITY, "invalid cuboid count");
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
            set_error(ERR_INVALID_ARGUMENT, "invalid dynamic cuboid parameters");
            return 0;
        }
        let Some(value_count) = (cuboid_count as usize).checked_mul(6) else {
            set_error(ERR_CAPACITY, "cuboid count is too large");
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

        for cuboid in cuboids.as_chunks::<6>().0 {
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
            world.inner.colliders.insert_with_parent(
                collider,
                body_handle,
                &mut world.inner.bodies,
            );
            collider_count += 1;
        }

        if collider_count == 0 {
            world.inner.bodies.remove(
                body_handle,
                &mut world.inner.islands,
                &mut world.inner.colliders,
                &mut world.inner.impulse_joints,
                &mut world.inner.multibody_joints,
                &mut world.inner.upstream_soft_bodies,
                true,
            );
            set_error(ERR_INVALID_ARGUMENT, "no valid cuboids");
            return 0;
        }

        clear_error();
        pack_rigid_body_handle(body_handle)
    })
}

/// Insert a fixed rigid body with a trimesh collider.
///
/// # Safety
///
/// `world` must be a valid, live world pointer; `vertices_xyz` must point to
/// at least `vertex_xyz_len` readable f64s and `indices` to at least
/// `index_len` readable u32s.
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
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if vertices_xyz.is_null() || indices.is_null() {
            set_error(ERR_NULL_POINTER, "trimesh input is null");
            return 0;
        }
        if vertex_xyz_len < 9
            || !vertex_xyz_len.is_multiple_of(3)
            || index_len < 3
            || !index_len.is_multiple_of(3)
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid trimesh lengths");
            return 0;
        }
        let vertex_count = vertex_xyz_len / 3;
        if vertex_count > MAX_TRIMESH_VERTICES || index_len > MAX_TRIMESH_INDICES {
            set_error(ERR_CAPACITY, "trimesh exceeds maximum size");
            return 0;
        }
        if !friction.is_finite() || !restitution.is_finite() || friction < 0.0 || restitution < 0.0
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid trimesh material parameters");
            return 0;
        }

        let vertices_xyz = unsafe { slice::from_raw_parts(vertices_xyz, vertex_xyz_len as usize) };
        let indices = unsafe { slice::from_raw_parts(indices, index_len as usize) };

        let mut vertices = Vec::with_capacity(vertex_count as usize);
        for chunk in vertices_xyz.as_chunks::<3>().0 {
            if !chunk[0].is_finite() || !chunk[1].is_finite() || !chunk[2].is_finite() {
                set_error(ERR_INVALID_ARGUMENT, "non-finite trimesh vertex");
                return 0;
            }
            vertices.push(Vector::new(chunk[0], chunk[1], chunk[2]));
        }

        let mut triangles = Vec::with_capacity(index_len as usize / 3);
        for chunk in indices.as_chunks::<3>().0 {
            if chunk[0] >= vertex_count || chunk[1] >= vertex_count || chunk[2] >= vertex_count {
                set_error(ERR_INVALID_ARGUMENT, "trimesh index out of range");
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
                &mut world.inner.upstream_soft_bodies,
                true,
            );
            set_error(ERR_INVALID_ARGUMENT, "failed to build trimesh collider");
            return 0;
        };
        let collider = collider.friction(friction).restitution(restitution).build();
        world
            .inner
            .colliders
            .insert_with_parent(collider, body_handle, &mut world.inner.bodies);
        clear_error();
        pack_rigid_body_handle(body_handle)
    })
}

/// Count the rigid bodies intersecting an AABB.
///
/// # Safety
///
/// `world` must be a valid, live world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_aabb_rigid_body_count(
    world: *const WorldHandle,
    aabb: AabbDesc,
    filter: QueryFilterDesc,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let _query_lock = world.inner.query_lock.read();
        if !valid_aabb(aabb) {
            set_error(ERR_INVALID_ARGUMENT, "invalid AABB");
            return 0;
        }

        let query = world.inner.broad_phase.as_query_pipeline(
            world.inner.narrow_phase.query_dispatcher(),
            &world.inner.bodies,
            &world.inner.colliders,
            query_filter_from_desc(filter),
        );

        let mut unique = HashSet::with_capacity(world.inner.bodies.len().min(1024));
        for (collider_handle, _) in query.intersect_aabb_conservative(
            rapier3d::geometry::Aabb::new(vec3_to_rapier(aabb.mins), vec3_to_rapier(aabb.maxs)),
        ) {
            if let Some(parent) = world
                .inner
                .colliders
                .get(collider_handle)
                .and_then(|collider| collider.parent())
            {
                unique.insert(parent);
            }
        }

        clear_error();
        unique.len() as u32
    })
}

/// Collect the rigid body handles intersecting an AABB.
///
/// # Safety
///
/// `world` must be a valid, live world pointer; `out_handles` must be valid
/// for `capacity` `RigidBodyHandleRaw` writes.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_aabb_rigid_bodies(
    world: *const WorldHandle,
    aabb: AabbDesc,
    filter: QueryFilterDesc,
    out_handles: *mut RigidBodyHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let _query_lock = world.inner.query_lock.read();
        if out_handles.is_null() {
            set_error(ERR_NULL_POINTER, "output buffer is null");
            return 0;
        }
        if capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
            set_error(ERR_CAPACITY, "invalid output capacity");
            return 0;
        }
        if !valid_aabb(aabb) {
            set_error(ERR_INVALID_ARGUMENT, "invalid AABB");
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
        for (collider_handle, _) in query.intersect_aabb_conservative(
            rapier3d::geometry::Aabb::new(vec3_to_rapier(aabb.mins), vec3_to_rapier(aabb.maxs)),
        ) {
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

        clear_error();
        written as u32
    })
}
