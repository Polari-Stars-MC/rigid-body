use rapier3d::math::{Pose, Rotation, Vector};
use rapier3d::prelude::{Array2, Collider, ColliderBuilder, SharedShape, TypedShape};
use smallvec::SmallVec;
use std::slice;
use rapier3d::na::Unit;
use crate::convert::quat_to_rapier;
use crate::rapier::ffi::{
    AabbDesc, Bool, ColliderBuilderHandle, ColliderHandleRaw, InteractionGroupsDesc, Obb, Quat,
    RigidBodyHandleRaw, ShapeDesc, Sphere, Vec3, WorldHandle, active_events_from_bits,
    active_hooks_from_bits, interaction_groups_to_rapier, isometry_from_parts,
    pack_collider_handle, quat_finite, quat_from_rapier, shape_desc_valid, shape_from_desc,
    unpack_collider_handle, unpack_rigid_body_handle, vec3_finite, vec3_from_rapier,
    vec3_to_rapier,
};

const MIN_HALF_EXTENT: f64 = 1.0e-9;
const MAX_RAW_POINTS: u32 = 1_000_000;
const MAX_HEIGHTMAP_CELLS: usize = 4_000_000;
const MAX_EDGE_COUNT: u32 = 1_000_000;
const MAX_SPHERE_COUNT: u32 = 1_000_000;

fn default_builder(shape_desc: ShapeDesc) -> ColliderBuilder {
    ColliderBuilder::new(shape_from_desc(shape_desc))
}

fn builder_from_aabb(mins: Vec3, maxs: Vec3) -> *mut ColliderBuilderHandle {
    if !valid_aabb(mins, maxs) {
        return std::ptr::null_mut();
    }

    let center = Vec3 {
        x: (mins.x + maxs.x) * 0.5,
        y: (mins.y + maxs.y) * 0.5,
        z: (mins.z + maxs.z) * 0.5,
    };
    let half = Vec3 {
        x: ((maxs.x - mins.x) * 0.5).max(MIN_HALF_EXTENT),
        y: ((maxs.y - mins.y) * 0.5).max(MIN_HALF_EXTENT),
        z: ((maxs.z - mins.z) * 0.5).max(MIN_HALF_EXTENT),
    };

    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: ColliderBuilder::cuboid(half.x, half.y, half.z).translation(vec3_to_rapier(center)),
    }))
}

fn valid_aabb(mins: Vec3, maxs: Vec3) -> bool {
    mins.x.is_finite()
        && mins.y.is_finite()
        && mins.z.is_finite()
        && maxs.x.is_finite()
        && maxs.y.is_finite()
        && maxs.z.is_finite()
        && mins.x <= maxs.x
        && mins.y <= maxs.y
        && mins.z <= maxs.z
}

fn points_from_xyz(points_xyz: *const f64, point_count: u32) -> Option<Vec<Vec3>> {
    if points_xyz.is_null() || point_count == 0 || point_count > MAX_RAW_POINTS {
        return None;
    }
    let value_count = (point_count as usize).checked_mul(3)?;
    let values = unsafe { slice::from_raw_parts(points_xyz, value_count) };
    let mut points = Vec::with_capacity(point_count as usize);
    for chunk in values.chunks_exact(3) {
        let point = Vec3 {
            x: chunk[0],
            y: chunk[1],
            z: chunk[2],
        };
        if !vec3_finite(point) {
            return None;
        }
        points.push(point);
    }
    Some(points)
}

fn builder_from_points(points: Vec<Vec3>) -> *mut ColliderBuilderHandle {
    if points.len() < 4 {
        return std::ptr::null_mut();
    }
    let points: Vec<_> = points.into_iter().map(vec3_to_rapier).collect();
    let Some(builder) = ColliderBuilder::convex_hull(&points) else {
        return std::ptr::null_mut();
    };
    Box::into_raw(Box::new(ColliderBuilderHandle { inner: builder }))
}

fn bounds_from_points(points: &[Vec3]) -> Option<(Vec3, Vec3)> {
    let mut iter = points.iter();
    let first = *iter.next()?;
    let mut mins = first;
    let mut maxs = first;
    for point in iter {
        mins.x = mins.x.min(point.x);
        mins.y = mins.y.min(point.y);
        mins.z = mins.z.min(point.z);
        maxs.x = maxs.x.max(point.x);
        maxs.y = maxs.y.max(point.y);
        maxs.z = maxs.z.max(point.z);
    }
    Some((mins, maxs))
}

fn builder_from_compound(parts: Vec<(Pose, SharedShape)>) -> *mut ColliderBuilderHandle {
    if parts.is_empty() {
        return std::ptr::null_mut();
    }
    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: ColliderBuilder::compound(parts),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create(
    shape_type: u32,
    shape_data: Vec3,
) -> *mut ColliderBuilderHandle {
    let shape_desc = ShapeDesc {
        shape_type,
        a: shape_data.x,
        b: shape_data.y,
        c: shape_data.z,
        d: 0.0,
    };
    if !shape_desc_valid(shape_desc) {
        return std::ptr::null_mut();
    }

    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: default_builder(shape_desc),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_halfspace(normal: Vec3) -> *mut ColliderBuilderHandle {
    if !vec3_finite(normal) {
        return std::ptr::null_mut();
    }

    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: ColliderBuilder::halfspace(Unit::new_unchecked(vec3_to_rapier(normal).normalize())),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_ex(shape_desc: ShapeDesc) -> *mut ColliderBuilderHandle {
    if !shape_desc_valid(shape_desc) {
        return std::ptr::null_mut();
    }

    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: default_builder(shape_desc),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_obb(obb: Obb) -> *mut ColliderBuilderHandle {
    if !vec3_finite(obb.center)
        || !vec3_finite(obb.half_extents)
        || !quat_finite(obb.rotation)
        || obb.half_extents.x <= 0.0
        || obb.half_extents.y <= 0.0
        || obb.half_extents.z <= 0.0
    {
        return std::ptr::null_mut();
    }

    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: ColliderBuilder::cuboid(obb.half_extents.x, obb.half_extents.y, obb.half_extents.z)
            .position(isometry_from_parts(obb.center, obb.rotation)),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_sphere(sphere: Sphere) -> *mut ColliderBuilderHandle {
    if !vec3_finite(sphere.center) || !sphere.radius.is_finite() || sphere.radius <= 0.0 {
        return std::ptr::null_mut();
    }

    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: ColliderBuilder::ball(sphere.radius).translation(vec3_to_rapier(sphere.center)),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_heightmap(
    data: *const f64,
    data_x: u32,
    data_y: u32,
    scale: Vec3,
) -> *mut ColliderBuilderHandle {
    let sv = vec3_to_rapier(scale);
    if data.is_null() || data_x == 0 || data_y == 0 || !vec3_finite(scale) || sv.length() <= 0.0 {
        return std::ptr::null_mut();
    }
    let Some(value_count) = (data_x as usize).checked_mul(data_y as usize) else {
        return std::ptr::null_mut();
    };
    if value_count > MAX_HEIGHTMAP_CELLS {
        return std::ptr::null_mut();
    }
    let values = unsafe { slice::from_raw_parts(data, value_count) };
    let mut heightfield = Array2::<f64>::zeros(data_x as usize, data_y as usize);
    for x in 0..data_x as usize {
        for y in 0..data_y as usize {
            let value = values[y * data_x as usize + x];
            if !value.is_finite() {
                return std::ptr::null_mut();
            }
            heightfield[(x, y)] = value;
        }
    }

    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: ColliderBuilder::heightfield(heightfield, sv),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_convex_hull(
    points_xyz: *const f64,
    point_count: u32,
) -> *mut ColliderBuilderHandle {
    let Some(points) = points_from_xyz(points_xyz, point_count) else {
        return std::ptr::null_mut();
    };
    builder_from_points(points)
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_point_cloud_bounds(
    points_xyz: *const f64,
    point_count: u32,
) -> *mut ColliderBuilderHandle {
    let Some(points) = points_from_xyz(points_xyz, point_count) else {
        return std::ptr::null_mut();
    };
    let mut mins = Vec3 {
        x: f64::INFINITY,
        y: f64::INFINITY,
        z: f64::INFINITY,
    };
    let mut maxs = Vec3 {
        x: f64::NEG_INFINITY,
        y: f64::NEG_INFINITY,
        z: f64::NEG_INFINITY,
    };

    for point in points {
        mins.x = mins.x.min(point.x);
        mins.y = mins.y.min(point.y);
        mins.z = mins.z.min(point.z);
        maxs.x = maxs.x.max(point.x);
        maxs.y = maxs.y.max(point.y);
        maxs.z = maxs.z.max(point.z);
    }

    builder_from_aabb(mins, maxs)
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_double_bv(
    first: AabbDesc,
    second: AabbDesc,
) -> *mut ColliderBuilderHandle {
    if !valid_aabb(first.mins, first.maxs) || !valid_aabb(second.mins, second.maxs) {
        return std::ptr::null_mut();
    }

    builder_from_aabb(
        Vec3 {
            x: first.mins.x.min(second.mins.x),
            y: first.mins.y.min(second.mins.y),
            z: first.mins.z.min(second.mins.z),
        },
        Vec3 {
            x: first.maxs.x.max(second.maxs.x),
            y: first.maxs.y.max(second.maxs.y),
            z: first.maxs.z.max(second.maxs.z),
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_skewed_obb(
    center: Vec3,
    axis_x: Vec3,
    axis_y: Vec3,
    axis_z: Vec3,
) -> *mut ColliderBuilderHandle {
    if !vec3_finite(center)
        || !vec3_finite(axis_x)
        || !vec3_finite(axis_y)
        || !vec3_finite(axis_z)
        || axis_x.x * axis_x.x + axis_x.y * axis_x.y + axis_x.z * axis_x.z <= MIN_HALF_EXTENT
        || axis_y.x * axis_y.x + axis_y.y * axis_y.y + axis_y.z * axis_y.z <= MIN_HALF_EXTENT
        || axis_z.x * axis_z.x + axis_z.y * axis_z.y + axis_z.z * axis_z.z <= MIN_HALF_EXTENT
    {
        return std::ptr::null_mut();
    }

    let mut points = SmallVec::<[Vec3; 8]>::with_capacity(8);
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            for sz in [-1.0, 1.0] {
                points.push(Vec3 {
                    x: center.x + axis_x.x * sx + axis_y.x * sy + axis_z.x * sz,
                    y: center.y + axis_x.y * sx + axis_y.y * sy + axis_z.y * sz,
                    z: center.z + axis_x.z * sx + axis_y.z * sy + axis_z.z * sz,
                });
            }
        }
    }
    builder_from_points(points.into_vec())
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_discrete_obb(
    points_xyz: *const f64,
    point_count: u32,
    axis: u32,
) -> *mut ColliderBuilderHandle {
    let Some(mut points) = points_from_xyz(points_xyz, point_count) else {
        return std::ptr::null_mut();
    };
    if axis % 3 == 1 {
        for point in &mut points {
            std::mem::swap(&mut point.x, &mut point.y);
        }
    } else if axis % 3 == 2 {
        for point in &mut points {
            std::mem::swap(&mut point.x, &mut point.z);
        }
    }
    let Some((mins, maxs)) = bounds_from_points(&points) else {
        return std::ptr::null_mut();
    };
    builder_from_aabb(mins, maxs)
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_fused_collapsing_bounds(
    points_xyz: *const f64,
    point_count: u32,
    padding: f64,
) -> *mut ColliderBuilderHandle {
    let Some(points) = points_from_xyz(points_xyz, point_count) else {
        return std::ptr::null_mut();
    };
    if !padding.is_finite() || padding < 0.0 {
        return std::ptr::null_mut();
    }
    let mut mins = Vec3 {
        x: f64::INFINITY,
        y: f64::INFINITY,
        z: f64::INFINITY,
    };
    let mut maxs = Vec3 {
        x: f64::NEG_INFINITY,
        y: f64::NEG_INFINITY,
        z: f64::NEG_INFINITY,
    };
    for point in points {
        mins.x = mins.x.min(point.x);
        mins.y = mins.y.min(point.y);
        mins.z = mins.z.min(point.z);
        maxs.x = maxs.x.max(point.x);
        maxs.y = maxs.y.max(point.y);
        maxs.z = maxs.z.max(point.z);
    }
    builder_from_aabb(
        Vec3 {
            x: mins.x - padding,
            y: mins.y - padding,
            z: mins.z - padding,
        },
        Vec3 {
            x: maxs.x + padding,
            y: maxs.y + padding,
            z: maxs.z + padding,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_edge_bvh(
    vertices_xyz: *const f64,
    vertex_count: u32,
    edges: *const u32,
    edge_count: u32,
    radius: f64,
) -> *mut ColliderBuilderHandle {
    if edges.is_null()
        || edge_count == 0
        || edge_count > MAX_EDGE_COUNT
        || !radius.is_finite()
        || radius <= 0.0
    {
        return std::ptr::null_mut();
    }
    let Some(vertices) = points_from_xyz(vertices_xyz, vertex_count) else {
        return std::ptr::null_mut();
    };
    let Some(index_count) = (edge_count as usize).checked_mul(2) else {
        return std::ptr::null_mut();
    };
    let indices = unsafe { slice::from_raw_parts(edges, index_count) };
    let mut parts = Vec::with_capacity(edge_count as usize);
    for edge in indices.chunks_exact(2) {
        let Some(a) = vertices.get(edge[0] as usize).copied() else {
            return std::ptr::null_mut();
        };
        let Some(b) = vertices.get(edge[1] as usize).copied() else {
            return std::ptr::null_mut();
        };
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let dz = b.z - a.z;
        if dx * dx + dy * dy + dz * dz <= MIN_HALF_EXTENT {
            continue;
        }
        parts.push((
            Pose::from_parts(Vector::ZERO, Rotation::IDENTITY),
            SharedShape::capsule(vec3_to_rapier(a), vec3_to_rapier(b), radius),
        ));
    }
    builder_from_compound(parts)
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_medial_spheres(
    spheres_xyzw: *const f64,
    sphere_count: u32,
) -> *mut ColliderBuilderHandle {
    if spheres_xyzw.is_null() || sphere_count == 0 || sphere_count > MAX_SPHERE_COUNT {
        return std::ptr::null_mut();
    }
    let Some(value_count) = (sphere_count as usize).checked_mul(4) else {
        return std::ptr::null_mut();
    };
    let values = unsafe { slice::from_raw_parts(spheres_xyzw, value_count) };
    let mut parts = Vec::with_capacity(sphere_count as usize);
    for chunk in values.chunks_exact(4) {
        let center = Vec3 {
            x: chunk[0],
            y: chunk[1],
            z: chunk[2],
        };
        let radius = chunk[3];
        if !vec3_finite(center) || !radius.is_finite() || radius <= 0.0 {
            return std::ptr::null_mut();
        }
        parts.push((
            Pose::from_parts(vec3_to_rapier(center), Rotation::IDENTITY),
            SharedShape::ball(radius),
        ));
    }
    builder_from_compound(parts)
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_build(builder: *mut ColliderBuilderHandle) -> *mut Collider {
    if builder.is_null() {
        return std::ptr::null_mut();
    }

    let builder = unsafe { Box::from_raw(builder) };
    let ColliderBuilderHandle { inner } = *builder;
    Box::into_raw(Box::new(inner.build()))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_destroy(builder: *mut ColliderBuilderHandle) {
    if builder.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(builder));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_destroy_raw(collider: *mut Collider) {
    if collider.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(collider));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_translation(
    builder: *mut ColliderBuilderHandle,
    translation: Vec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !vec3_finite(translation) {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.translation(vec3_to_rapier(translation));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_rotation(
    builder: *mut ColliderBuilderHandle,
    rotation_axis_angle: Vec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !vec3_finite(rotation_axis_angle) {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.rotation(vec3_to_rapier(rotation_axis_angle));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_pose(
    builder: *mut ColliderBuilderHandle,
    translation: Vec3,
    rotation: Quat,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !vec3_finite(translation) || !quat_finite(rotation) {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.position(isometry_from_parts(translation, rotation));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_sensor(builder: *mut ColliderBuilderHandle, sensor: Bool) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.sensor(sensor.0 != 0);
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_friction(
    builder: *mut ColliderBuilderHandle,
    friction: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !friction.is_finite() || friction < 0.0 {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.friction(friction);
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_restitution(
    builder: *mut ColliderBuilderHandle,
    restitution: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !restitution.is_finite() || restitution < 0.0 {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.restitution(restitution);
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_density(builder: *mut ColliderBuilderHandle, density: f64) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !density.is_finite() || density < 0.0 {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.density(density);
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_collision_groups(
    builder: *mut ColliderBuilderHandle,
    groups: InteractionGroupsDesc,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.collision_groups(interaction_groups_to_rapier(groups));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_solver_groups(
    builder: *mut ColliderBuilderHandle,
    groups: InteractionGroupsDesc,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.solver_groups(interaction_groups_to_rapier(groups));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_active_events(
    builder: *mut ColliderBuilderHandle,
    active_events_bits: u32,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.active_events(active_events_from_bits(active_events_bits));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_active_hooks(
    builder: *mut ColliderBuilderHandle,
    active_hooks_bits: u32,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.active_hooks(active_hooks_from_bits(active_hooks_bits));
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_contact_force_event_threshold(
    builder: *mut ColliderBuilderHandle,
    threshold: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    if !threshold.is_finite() || threshold < 0.0 {
        return;
    }

    let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
    builder.inner = inner.contact_force_event_threshold(threshold);
}

#[unsafe(no_mangle)]
pub extern "C" fn world_insert_collider(
    world: *mut WorldHandle,
    memory_handle: *mut Collider,
) -> ColliderHandleRaw {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0;
    };
    if memory_handle.is_null() {
        return 0;
    }

    let built = unsafe { *Box::from_raw(memory_handle) };
    pack_collider_handle(world.inner.colliders.insert(built))
}

#[unsafe(no_mangle)]
pub extern "C" fn world_insert_collider_with_parent(
    world: *mut WorldHandle,
    memory_handle: *mut Collider,
    parent: RigidBodyHandleRaw,
) -> ColliderHandleRaw {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0;
    };
    if memory_handle.is_null() {
        return 0;
    }

    let built = unsafe { *Box::from_raw(memory_handle) };
    pack_collider_handle(world.inner.colliders.insert_with_parent(
        built,
        unpack_rigid_body_handle(parent),
        &mut world.inner.bodies,
    ))
}

#[unsafe(no_mangle)]
pub extern "C" fn world_remove_collider(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    wake_up: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };

    world
        .inner
        .colliders
        .remove(
            unpack_collider_handle(handle),
            &mut world.inner.islands,
            &mut world.inner.bodies,
            wake_up.0 != 0,
        )
        .is_some()
        .into()
}

#[unsafe(no_mangle)]
pub extern "C" fn world_copy_collider(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
) -> *mut Collider {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return std::ptr::null_mut();
    };

    let Some(collider) = world
        .inner
        .colliders
        .get(unpack_collider_handle(handle))
        .cloned()
    else {
        return std::ptr::null_mut();
    };

    Box::into_raw(Box::new(collider))
}

#[unsafe(no_mangle)]
pub extern "C" fn world_remove_collider_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    wake_up: Bool,
) -> u8 {
    world_remove_collider(world, handle, wake_up).0
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_get_translation(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
) -> Vec3 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return Vec3::default();
    };

    world
        .inner
        .colliders
        .get(unpack_collider_handle(handle))
        .map(|collider| vec3_from_rapier(collider.translation()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_get_shape_count(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
) -> usize {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };

    match world.inner.colliders.get(unpack_collider_handle(handle)).unwrap().shape().as_typed_shape() {
        TypedShape::Compound(compound) => compound.shapes().len(),
        _ => 1,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_get_translation_out(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
    out_translation: *mut Vec3,
) {
    let Some(out_translation) = (unsafe { out_translation.as_mut() }) else {
        return;
    };

    *out_translation = collider_get_translation(world, handle);
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_get_rotation(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
) -> Quat {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return Quat::default();
    };

    world
        .inner
        .colliders
        .get(unpack_collider_handle(handle))
        .map(|collider| quat_from_rapier(collider.rotation()))
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_get_rotation_out(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
    out_rotation: *mut Quat,
) {
    let Some(out_rotation) = (unsafe { out_rotation.as_mut() }) else {
        return;
    };

    *out_rotation = collider_get_rotation(world, handle);
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_pose(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    translation: Vec3,
    rotation: Quat,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };
    if !vec3_finite(translation) || !quat_finite(rotation) {
        return Bool::FALSE;
    }

    collider.set_position(isometry_from_parts(translation, rotation));
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_translation(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    translation: Vec3,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };
    if !vec3_finite(translation) {
        return Bool::FALSE;
    }

    collider.set_translation(vec3_to_rapier(translation));
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_rotation(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    rotation: Quat,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };
    if !quat_finite(rotation) {
        return Bool::FALSE;
    }

    collider.set_rotation(quat_to_rapier(rotation));
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_pose_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    translation: Vec3,
    rotation: Quat,
) -> u8 {
    collider_set_pose(world, handle, translation, rotation).0
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_sensor(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    sensor: Bool,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_sensor(sensor.0 != 0);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_sensor_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    sensor: Bool,
) -> u8 {
    collider_set_sensor(world, handle, sensor).0
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_friction(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    friction: f64,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };
    if !friction.is_finite() || friction < 0.0 {
        return Bool::FALSE;
    }

    collider.set_friction(friction);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_friction_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    friction: f64,
) -> u8 {
    collider_set_friction(world, handle, friction).0
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_restitution(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    restitution: f64,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };
    if !restitution.is_finite() || restitution < 0.0 {
        return Bool::FALSE;
    }

    collider.set_restitution(restitution);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_restitution_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    restitution: f64,
) -> u8 {
    collider_set_restitution(world, handle, restitution).0
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_collision_groups(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    groups: InteractionGroupsDesc,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_collision_groups(interaction_groups_to_rapier(groups));
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_collision_groups_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    groups: InteractionGroupsDesc,
) -> u8 {
    collider_set_collision_groups(world, handle, groups).0
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_solver_groups(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    groups: InteractionGroupsDesc,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_solver_groups(interaction_groups_to_rapier(groups));
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_solver_groups_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    groups: InteractionGroupsDesc,
) -> u8 {
    collider_set_solver_groups(world, handle, groups).0
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_active_events(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    active_events_bits: u32,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_active_events(active_events_from_bits(active_events_bits));
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_active_events_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    active_events_bits: u32,
) -> u8 {
    collider_set_active_events(world, handle, active_events_bits).0
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_active_hooks(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    active_hooks_bits: u32,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };

    collider.set_active_hooks(active_hooks_from_bits(active_hooks_bits));
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_active_hooks_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    active_hooks_bits: u32,
) -> u8 {
    collider_set_active_hooks(world, handle, active_hooks_bits).0
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_contact_force_event_threshold(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    threshold: f64,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(collider) = world
        .inner
        .colliders
        .get_mut(unpack_collider_handle(handle))
    else {
        return Bool::FALSE;
    };
    if !threshold.is_finite() || threshold < 0.0 {
        return Bool::FALSE;
    }

    collider.set_contact_force_event_threshold(threshold);
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_set_contact_force_event_threshold_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    threshold: f64,
) -> u8 {
    collider_set_contact_force_event_threshold(world, handle, threshold).0
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_get_density(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
) -> f64 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0.0;
    };

    world
        .inner
        .colliders
        .get(unpack_collider_handle(handle))
        .map(|collider| collider.density())
        .unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

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
