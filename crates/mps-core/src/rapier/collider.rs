use crate::convert::quat_to_rapier;
use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    AabbDesc, Bool, ColliderBuilderHandle, ColliderHandleRaw, InteractionGroupsDesc, Obb, Quat,
    RigidBodyHandleRaw, ShapeDesc, Sphere, Vec3, WorldHandle, active_events_from_bits,
    active_hooks_from_bits, interaction_groups_to_rapier, isometry_from_parts,
    pack_collider_handle, quat_finite, quat_from_rapier, shape_desc_valid, shape_from_desc,
    unpack_collider_handle, unpack_rigid_body_handle, vec3_finite, vec3_from_rapier,
    vec3_to_rapier,
};
use rapier3d::math::{Pose, Rotation, Vector};
use rapier3d::na::Unit;
use rapier3d::prelude::{
    Array2, CoefficientCombineRule, Collider, ColliderBuilder, SharedShape, TypedShape,
};

// Side-channel that carries a voxel source grid from `collider_builder_build`
// to the immediately-following `world_insert_collider*` call on the same
// thread. The mod always builds then inserts a builder in one logical step,
// so this is safe: `collider_builder_build` stores the cache (only present
// for voxel builders) and the next insert consumes it. Non-voxel builders
// leave it `None`, so inserts of ordinary colliders are unaffected.
thread_local! {
    #[allow(clippy::missing_const_for_thread_local)]
    static PENDING_VOXEL_CACHE: std::cell::RefCell<Option<crate::rapier::voxel::VoxelCache>> =
        std::cell::RefCell::new(None);
}
use smallvec::SmallVec;
use std::slice;

const MIN_HALF_EXTENT: f64 = 1.0e-9;
const MAX_RAW_POINTS: u32 = 1_000_000;
const MAX_HEIGHTMAP_CELLS: usize = 4_000_000;
const MAX_EDGE_COUNT: u32 = 1_000_000;
const MAX_SPHERE_COUNT: u32 = 1_000_000;
const MAX_COMPOUND_PARTS: u32 = 100_000;

fn default_builder(shape_desc: ShapeDesc) -> ColliderBuilder {
    ColliderBuilder::new(shape_from_desc(shape_desc))
}

fn builder_from_aabb(mins: Vec3, maxs: Vec3) -> *mut ColliderBuilderHandle {
    if !valid_aabb(mins, maxs) {
        set_error(ERR_INVALID_ARGUMENT, "invalid AABB");
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
        voxel_source: None,
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
    if points_xyz.is_null() {
        set_error(ERR_NULL_POINTER, "points buffer is null");
        return None;
    }
    if point_count == 0 || point_count > MAX_RAW_POINTS {
        set_error(ERR_INVALID_ARGUMENT, "invalid point count");
        return None;
    }
    let value_count = (point_count as usize).checked_mul(3)?;
    let values = unsafe { slice::from_raw_parts(points_xyz, value_count) };
    let mut points = Vec::with_capacity(point_count as usize);
    for chunk in values.as_chunks::<3>().0 {
        let point = Vec3 {
            x: chunk[0],
            y: chunk[1],
            z: chunk[2],
        };
        if !vec3_finite(point) {
            set_error(ERR_INVALID_ARGUMENT, "point contains non-finite value");
            return None;
        }
        points.push(point);
    }
    Some(points)
}

fn builder_from_points(points: Vec<Vec3>) -> *mut ColliderBuilderHandle {
    if points.len() < 4 {
        set_error(
            ERR_INVALID_ARGUMENT,
            "convex hull requires at least 4 points",
        );
        return std::ptr::null_mut();
    }
    let points: Vec<_> = points.into_iter().map(vec3_to_rapier).collect();
    let Some(builder) = ColliderBuilder::convex_hull(&points) else {
        set_error(ERR_INVALID_ARGUMENT, "convex hull computation failed");
        return std::ptr::null_mut();
    };
    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: builder,
        voxel_source: None,
    }))
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
        set_error(ERR_INVALID_ARGUMENT, "compound collider has no parts");
        return std::ptr::null_mut();
    }
    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: ColliderBuilder::compound(parts),
        voxel_source: None,
    }))
}

fn boxes_from_minmax(box_data: *const f64, box_count: u32) -> Option<Vec<(Pose, SharedShape)>> {
    if box_data.is_null() {
        set_error(ERR_NULL_POINTER, "box data is null");
        return None;
    }
    if box_count == 0 {
        set_error(ERR_INVALID_ARGUMENT, "compound collider has no boxes");
        return None;
    }
    let count = box_count as usize;
    if count > MAX_COMPOUND_PARTS as usize {
        set_error(ERR_INVALID_ARGUMENT, "too many compound boxes");
        return None;
    }
    let total = count.checked_mul(6)?;
    let data = unsafe { slice::from_raw_parts(box_data, total) };

    let mut parts = Vec::with_capacity(count);
    for chunk in data.as_chunks::<6>().0 {
        let min_x = chunk[0];
        let min_y = chunk[1];
        let min_z = chunk[2];
        let max_x = chunk[3];
        let max_y = chunk[4];
        let max_z = chunk[5];
        if !min_x.is_finite()
            || !min_y.is_finite()
            || !min_z.is_finite()
            || !max_x.is_finite()
            || !max_y.is_finite()
            || !max_z.is_finite()
            || min_x >= max_x
            || min_y >= max_y
            || min_z >= max_z
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid box bounds");
            return None;
        }

        let center = Vec3 {
            x: (min_x + max_x) * 0.5,
            y: (min_y + max_y) * 0.5,
            z: (min_z + max_z) * 0.5,
        };
        let half = Vec3 {
            x: ((max_x - min_x) * 0.5).max(MIN_HALF_EXTENT),
            y: ((max_y - min_y) * 0.5).max(MIN_HALF_EXTENT),
            z: ((max_z - min_z) * 0.5).max(MIN_HALF_EXTENT),
        };
        parts.push((
            Pose::from_parts(vec3_to_rapier(center), Rotation::IDENTITY),
            SharedShape::cuboid(half.x, half.y, half.z),
        ));
    }
    Some(parts)
}

/// Creates a compound collider builder from a packed array of axis-aligned boxes.
///
/// # Safety
///
/// `box_data` must point to at least `box_count * 6` readable `f64` values,
/// each box described as min_x, min_y, min_z, max_x, max_y, max_z.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_compound_boxes(
    box_data: *const f64,
    box_count: u32,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        let Some(parts) = boxes_from_minmax(box_data, box_count) else {
            return std::ptr::null_mut();
        };
        builder_from_compound(parts)
    })
}

/// Creates a collider builder from a generic shape type and packed shape data.
///
/// # Safety
///
/// All parameters are passed by value; no raw pointers are dereferenced.
/// An invalid shape descriptor fails with `ERR_INVALID_ARGUMENT` and returns null.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create(
    shape_type: u32,
    shape_data: Vec3,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        let shape_desc = ShapeDesc {
            shape_type,
            a: shape_data.x,
            b: shape_data.y,
            c: shape_data.z,
            d: 0.0,
        };
        if !shape_desc_valid(shape_desc) {
            set_error(ERR_INVALID_ARGUMENT, "invalid shape descriptor");
            return std::ptr::null_mut();
        }

        Box::into_raw(Box::new(ColliderBuilderHandle {
            inner: default_builder(shape_desc),
            voxel_source: None,
        }))
    })
}

/// Creates a halfspace collider builder with the given plane normal.
///
/// # Safety
///
/// `normal` is passed by value; no raw pointers are dereferenced.
/// A non-finite normal fails with `ERR_INVALID_ARGUMENT` and returns null.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_halfspace(normal: Vec3) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        if !vec3_finite(normal) {
            set_error(ERR_INVALID_ARGUMENT, "halfspace normal must be finite");
            return std::ptr::null_mut();
        }

        Box::into_raw(Box::new(ColliderBuilderHandle {
            inner: ColliderBuilder::halfspace(Unit::new_unchecked(
                vec3_to_rapier(normal).normalize(),
            )),
            voxel_source: None,
        }))
    })
}

/// Creates a collider builder from an extended shape descriptor.
///
/// # Safety
///
/// `shape_desc` is passed by value; no raw pointers are dereferenced.
/// An invalid shape descriptor fails with `ERR_INVALID_ARGUMENT` and returns null.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_ex(shape_desc: ShapeDesc) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        if !shape_desc_valid(shape_desc) {
            set_error(ERR_INVALID_ARGUMENT, "invalid shape descriptor");
            return std::ptr::null_mut();
        }

        Box::into_raw(Box::new(ColliderBuilderHandle {
            inner: default_builder(shape_desc),
            voxel_source: None,
        }))
    })
}

/// Creates an oriented box (cuboid) collider builder from an OBB descriptor.
///
/// # Safety
///
/// `obb` is passed by value; no raw pointers are dereferenced.
/// A non-finite center/rotation or non-positive half extents fail with
/// `ERR_INVALID_ARGUMENT` and return null.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_obb(obb: Obb) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        if !vec3_finite(obb.center)
            || !vec3_finite(obb.half_extents)
            || !quat_finite(obb.rotation)
            || obb.half_extents.x <= 0.0
            || obb.half_extents.y <= 0.0
            || obb.half_extents.z <= 0.0
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid OBB");
            return std::ptr::null_mut();
        }

        Box::into_raw(Box::new(ColliderBuilderHandle {
            inner: ColliderBuilder::cuboid(
                obb.half_extents.x,
                obb.half_extents.y,
                obb.half_extents.z,
            )
            .position(isometry_from_parts(obb.center, obb.rotation)),
            voxel_source: None,
        }))
    })
}

/// Creates a ball collider builder from a sphere descriptor.
///
/// # Safety
///
/// `sphere` is passed by value; no raw pointers are dereferenced.
/// A non-finite center or a non-finite/non-positive radius fails with
/// `ERR_INVALID_ARGUMENT` and returns null.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_sphere(sphere: Sphere) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        if !vec3_finite(sphere.center) || !sphere.radius.is_finite() || sphere.radius <= 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid sphere");
            return std::ptr::null_mut();
        }

        Box::into_raw(Box::new(ColliderBuilderHandle {
            inner: ColliderBuilder::ball(sphere.radius).translation(vec3_to_rapier(sphere.center)),
            voxel_source: None,
        }))
    })
}

/// # Safety
///
/// `data` must point to at least `data_x * data_y` readable `f64` height values.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_heightmap(
    data: *const f64,
    data_x: u32,
    data_y: u32,
    scale: Vec3,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        let sv = vec3_to_rapier(scale);
        if data.is_null() {
            set_error(ERR_NULL_POINTER, "heightmap data is null");
            return std::ptr::null_mut();
        }
        if data_x == 0 || data_y == 0 || !vec3_finite(scale) || sv.length() <= 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "invalid heightmap dimensions or scale",
            );
            return std::ptr::null_mut();
        }
        let Some(value_count) = (data_x as usize).checked_mul(data_y as usize) else {
            set_error(ERR_INVALID_ARGUMENT, "heightmap cell count overflow");
            return std::ptr::null_mut();
        };
        if value_count > MAX_HEIGHTMAP_CELLS {
            set_error(ERR_INVALID_ARGUMENT, "heightmap cell count exceeds limit");
            return std::ptr::null_mut();
        }
        let values = unsafe { slice::from_raw_parts(data, value_count) };
        let mut heightfield = Array2::<f64>::zeros(data_x as usize, data_y as usize);
        for x in 0..data_x as usize {
            for y in 0..data_y as usize {
                let value = values[y * data_x as usize + x];
                if !value.is_finite() {
                    set_error(ERR_INVALID_ARGUMENT, "heightmap contains non-finite value");
                    return std::ptr::null_mut();
                }
                heightfield[(x, y)] = value;
            }
        }

        Box::into_raw(Box::new(ColliderBuilderHandle {
            inner: ColliderBuilder::heightfield(heightfield, sv),
            voxel_source: None,
        }))
    })
}

/// # Safety
///
/// `points_xyz` must point to at least `point_count * 3` readable `f64` values.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_convex_hull(
    points_xyz: *const f64,
    point_count: u32,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        let Some(points) = points_from_xyz(points_xyz, point_count) else {
            return std::ptr::null_mut();
        };
        builder_from_points(points)
    })
}

/// # Safety
///
/// `points_xyz` must point to at least `point_count * 3` readable `f64` values.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_point_cloud_bounds(
    points_xyz: *const f64,
    point_count: u32,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
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
    })
}

/// Creates a collider builder covering the union of two AABBs.
///
/// # Safety
///
/// `first` and `second` are passed by value; no raw pointers are dereferenced.
/// An invalid AABB (non-finite or `mins > maxs`) fails with
/// `ERR_INVALID_ARGUMENT` and returns null.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_double_bv(
    first: AabbDesc,
    second: AabbDesc,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        if !valid_aabb(first.mins, first.maxs) || !valid_aabb(second.mins, second.maxs) {
            set_error(ERR_INVALID_ARGUMENT, "invalid AABB");
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
    })
}

/// Creates a convex-hull collider builder from a skewed box (center + 3 axis vectors).
///
/// # Safety
///
/// All parameters are passed by value; no raw pointers are dereferenced.
/// Non-finite vectors or near-zero-length axes fail with `ERR_INVALID_ARGUMENT`
/// and return null.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_skewed_obb(
    center: Vec3,
    axis_x: Vec3,
    axis_y: Vec3,
    axis_z: Vec3,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        if !vec3_finite(center)
            || !vec3_finite(axis_x)
            || !vec3_finite(axis_y)
            || !vec3_finite(axis_z)
            || axis_x.x * axis_x.x + axis_x.y * axis_x.y + axis_x.z * axis_x.z <= MIN_HALF_EXTENT
            || axis_y.x * axis_y.x + axis_y.y * axis_y.y + axis_y.z * axis_y.z <= MIN_HALF_EXTENT
            || axis_z.x * axis_z.x + axis_z.y * axis_z.y + axis_z.z * axis_z.z <= MIN_HALF_EXTENT
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid skewed OBB axes");
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
    })
}

/// # Safety
///
/// `points_xyz` must point to at least `point_count * 3` readable `f64` values.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_discrete_obb(
    points_xyz: *const f64,
    point_count: u32,
    axis: u32,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
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
    })
}

/// # Safety
///
/// `points_xyz` must point to at least `point_count * 3` readable `f64` values.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_fused_collapsing_bounds(
    points_xyz: *const f64,
    point_count: u32,
    padding: f64,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        let Some(points) = points_from_xyz(points_xyz, point_count) else {
            return std::ptr::null_mut();
        };
        if !padding.is_finite() || padding < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid padding");
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
    })
}

/// # Safety
///
/// `vertices_xyz` must point to at least `vertex_count * 3` readable `f64`
/// values and `edges` to at least `edge_count * 2` readable `u32` indices.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_edge_bvh(
    vertices_xyz: *const f64,
    vertex_count: u32,
    edges: *const u32,
    edge_count: u32,
    radius: f64,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        if edges.is_null() {
            set_error(ERR_NULL_POINTER, "edge index buffer is null");
            return std::ptr::null_mut();
        }
        if edge_count == 0 || edge_count > MAX_EDGE_COUNT {
            set_error(ERR_INVALID_ARGUMENT, "invalid edge count");
            return std::ptr::null_mut();
        }
        if !radius.is_finite() || radius <= 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid radius");
            return std::ptr::null_mut();
        }
        let Some(vertices) = points_from_xyz(vertices_xyz, vertex_count) else {
            return std::ptr::null_mut();
        };
        let Some(index_count) = (edge_count as usize).checked_mul(2) else {
            set_error(ERR_INVALID_ARGUMENT, "edge index count overflow");
            return std::ptr::null_mut();
        };
        let indices = unsafe { slice::from_raw_parts(edges, index_count) };
        let mut parts = Vec::with_capacity(edge_count as usize);
        for edge in indices.as_chunks::<2>().0 {
            let Some(a) = vertices.get(edge[0] as usize).copied() else {
                set_error(ERR_INVALID_ARGUMENT, "edge vertex index out of range");
                return std::ptr::null_mut();
            };
            let Some(b) = vertices.get(edge[1] as usize).copied() else {
                set_error(ERR_INVALID_ARGUMENT, "edge vertex index out of range");
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
    })
}

/// # Safety
///
/// `spheres_xyzw` must point to at least `sphere_count * 4` readable `f64`
/// values (center xyz + radius per sphere).
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_medial_spheres(
    spheres_xyzw: *const f64,
    sphere_count: u32,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        if spheres_xyzw.is_null() {
            set_error(ERR_NULL_POINTER, "sphere buffer is null");
            return std::ptr::null_mut();
        }
        if sphere_count == 0 || sphere_count > MAX_SPHERE_COUNT {
            set_error(ERR_INVALID_ARGUMENT, "invalid sphere count");
            return std::ptr::null_mut();
        }
        let Some(value_count) = (sphere_count as usize).checked_mul(4) else {
            set_error(ERR_INVALID_ARGUMENT, "sphere value count overflow");
            return std::ptr::null_mut();
        };
        let values = unsafe { slice::from_raw_parts(spheres_xyzw, value_count) };
        let mut parts = Vec::with_capacity(sphere_count as usize);
        for chunk in values.as_chunks::<4>().0 {
            let center = Vec3 {
                x: chunk[0],
                y: chunk[1],
                z: chunk[2],
            };
            let radius = chunk[3];
            if !vec3_finite(center) || !radius.is_finite() || radius <= 0.0 {
                set_error(ERR_INVALID_ARGUMENT, "invalid sphere");
                return std::ptr::null_mut();
            }
            parts.push((
                Pose::from_parts(vec3_to_rapier(center), Rotation::IDENTITY),
                SharedShape::ball(radius),
            ));
        }
        builder_from_compound(parts)
    })
}

/// # Safety
///
/// `builder` must be a pointer returned by a `collider_builder_create_*`
/// function. It is consumed by this call and must not be used afterwards.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_build(builder: *mut ColliderBuilderHandle) -> *mut Collider {
    ffi_guard(std::ptr::null_mut(), || {
        if builder.is_null() {
            set_error(ERR_NULL_POINTER, "builder is null");
            return std::ptr::null_mut();
        }

        let builder = unsafe { Box::from_raw(builder) };
        let ColliderBuilderHandle {
            inner,
            voxel_source,
        } = *builder;
        // Hand the voxel source grid (if any) to the next `world_insert_collider*`.
        PENDING_VOXEL_CACHE.with(|slot| *slot.borrow_mut() = voxel_source);
        Box::into_raw(Box::new(inner.build()))
    })
}

/// # Safety
///
/// `builder` must be a pointer returned by a `collider_builder_create_*`
/// function that has not been consumed by `collider_builder_build`.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_destroy(builder: *mut ColliderBuilderHandle) {
    ffi_guard((), || {
        if builder.is_null() {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        }

        unsafe {
            drop(Box::from_raw(builder));
        }
    })
}

/// # Safety
///
/// `collider` must be a pointer returned by `collider_builder_build` or
/// `world_copy_collider` that has not already been destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_destroy_raw(collider: *mut Collider) {
    ffi_guard((), || {
        if collider.is_null() {
            set_error(ERR_NULL_POINTER, "collider is null");
            return;
        }

        unsafe {
            drop(Box::from_raw(collider));
        }
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_translation(
    builder: *mut ColliderBuilderHandle,
    translation: Vec3,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !vec3_finite(translation) {
            set_error(ERR_INVALID_ARGUMENT, "translation must be finite");
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.translation(vec3_to_rapier(translation));
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_rotation(
    builder: *mut ColliderBuilderHandle,
    rotation_axis_angle: Vec3,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !vec3_finite(rotation_axis_angle) {
            set_error(ERR_INVALID_ARGUMENT, "rotation must be finite");
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.rotation(vec3_to_rapier(rotation_axis_angle));
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_pose(
    builder: *mut ColliderBuilderHandle,
    translation: Vec3,
    rotation: Quat,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !vec3_finite(translation) || !quat_finite(rotation) {
            set_error(ERR_INVALID_ARGUMENT, "pose must be finite");
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.position(isometry_from_parts(translation, rotation));
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_sensor(builder: *mut ColliderBuilderHandle, sensor: Bool) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.sensor(sensor.0 != 0);
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_friction(
    builder: *mut ColliderBuilderHandle,
    friction: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !friction.is_finite() || friction < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "friction must be finite and non-negative",
            );
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.friction(friction);
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_restitution(
    builder: *mut ColliderBuilderHandle,
    restitution: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !restitution.is_finite() || restitution < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "restitution must be finite and non-negative",
            );
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.restitution(restitution);
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_contact_skin(
    builder: *mut ColliderBuilderHandle,
    skin: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !skin.is_finite() || skin < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "contact skin must be finite and non-negative",
            );
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.contact_skin(skin);
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_density(builder: *mut ColliderBuilderHandle, density: f64) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !density.is_finite() || density < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "density must be finite and non-negative",
            );
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.density(density);
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_collision_groups(
    builder: *mut ColliderBuilderHandle,
    groups: InteractionGroupsDesc,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.collision_groups(interaction_groups_to_rapier(groups));
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_solver_groups(
    builder: *mut ColliderBuilderHandle,
    groups: InteractionGroupsDesc,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.solver_groups(interaction_groups_to_rapier(groups));
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_active_events(
    builder: *mut ColliderBuilderHandle,
    active_events_bits: u32,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.active_events(active_events_from_bits(active_events_bits));
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_active_hooks(
    builder: *mut ColliderBuilderHandle,
    active_hooks_bits: u32,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.active_hooks(active_hooks_from_bits(active_hooks_bits));
    })
}

/// # Safety
///
/// `builder` must be a valid pointer returned by a `collider_builder_create_*`
/// function and not yet consumed or destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_set_contact_force_event_threshold(
    builder: *mut ColliderBuilderHandle,
    threshold: f64,
) {
    ffi_guard((), || {
        let Some(builder) = (unsafe { builder.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "builder is null");
            return;
        };
        if !threshold.is_finite() || threshold < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "threshold must be finite and non-negative",
            );
            return;
        }

        let inner = std::mem::replace(&mut builder.inner, ColliderBuilder::ball(0.5));
        builder.inner = inner.contact_force_event_threshold(threshold);
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create`. `memory_handle`
/// must be a pointer returned by `collider_builder_build` or
/// `world_copy_collider`; it is consumed by this call.
#[unsafe(no_mangle)]
pub extern "C" fn world_insert_collider(
    world: *mut WorldHandle,
    memory_handle: *mut Collider,
) -> ColliderHandleRaw {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if memory_handle.is_null() {
            set_error(ERR_NULL_POINTER, "collider is null");
            return 0;
        }

        let built = unsafe { *Box::from_raw(memory_handle) };
        let handle = world.inner.colliders.insert(built);
        if let Some(cache) = PENDING_VOXEL_CACHE.with(|slot| slot.borrow_mut().take()) {
            world
                .inner
                .voxel_grids
                .insert(pack_collider_handle(handle), cache);
        }
        pack_collider_handle(handle)
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create`. `memory_handle`
/// must be a pointer returned by `collider_builder_build` or
/// `world_copy_collider`; it is consumed by this call.
#[unsafe(no_mangle)]
pub extern "C" fn world_insert_collider_with_parent(
    world: *mut WorldHandle,
    memory_handle: *mut Collider,
    parent: RigidBodyHandleRaw,
) -> ColliderHandleRaw {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if memory_handle.is_null() {
            set_error(ERR_NULL_POINTER, "collider is null");
            return 0;
        }

        let built = unsafe { *Box::from_raw(memory_handle) };
        let handle = world.inner.colliders.insert_with_parent(
            built,
            unpack_rigid_body_handle(parent),
            &mut world.inner.bodies,
        );
        if let Some(cache) = PENDING_VOXEL_CACHE.with(|slot| slot.borrow_mut().take()) {
            world
                .inner
                .voxel_grids
                .insert(pack_collider_handle(handle), cache);
        }
        pack_collider_handle(handle)
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn world_remove_collider(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        let removed = world
            .inner
            .colliders
            .remove(
                unpack_collider_handle(handle),
                &mut world.inner.islands,
                &mut world.inner.bodies,
                wake_up.0 != 0,
            )
            .is_some();
        if !removed {
            set_error(ERR_NOT_FOUND, "collider not found");
        } else {
            // Drop any stored voxel source grid for the removed collider.
            world.inner.voxel_grids.remove(&handle);
        }
        removed.into()
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn world_copy_collider(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
) -> *mut Collider {
    ffi_guard(std::ptr::null_mut(), || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return std::ptr::null_mut();
        };

        let Some(collider) = world
            .inner
            .colliders
            .get(unpack_collider_handle(handle))
            .cloned()
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return std::ptr::null_mut();
        };

        Box::into_raw(Box::new(collider))
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn world_remove_collider_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    wake_up: Bool,
) -> u8 {
    ffi_guard(0, || world_remove_collider(world, handle, wake_up).0)
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_get_translation(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
) -> Vec3 {
    ffi_guard(Vec3::default(), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Vec3::default();
        };

        let Some(collider) = world.inner.colliders.get(unpack_collider_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Vec3::default();
        };
        vec3_from_rapier(collider.translation())
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_get_shape_count(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
) -> usize {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };

        // Invalid handle: return 0 instead of panicking across the FFI boundary.
        let Some(collider) = world.inner.colliders.get(unpack_collider_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return 0;
        };
        match collider.shape().as_typed_shape() {
            TypedShape::Compound(compound) => compound.shapes().len(),
            _ => 1,
        }
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create`; `out_translation`
/// must point to a writable `Vec3`.
#[unsafe(no_mangle)]
pub extern "C" fn collider_get_translation_out(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
    out_translation: *mut Vec3,
) {
    ffi_guard((), || {
        let Some(out_translation) = (unsafe { out_translation.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "output pointer is null");
            return;
        };

        *out_translation = collider_get_translation(world, handle);
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_get_rotation(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
) -> Quat {
    ffi_guard(Quat::default(), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Quat::default();
        };

        let Some(collider) = world.inner.colliders.get(unpack_collider_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Quat::default();
        };
        quat_from_rapier(collider.rotation())
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create`; `out_rotation`
/// must point to a writable `Quat`.
#[unsafe(no_mangle)]
pub extern "C" fn collider_get_rotation_out(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
    out_rotation: *mut Quat,
) {
    ffi_guard((), || {
        let Some(out_rotation) = (unsafe { out_rotation.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "output pointer is null");
            return;
        };

        *out_rotation = collider_get_rotation(world, handle);
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_pose(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    translation: Vec3,
    rotation: Quat,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };
        if !vec3_finite(translation) || !quat_finite(rotation) {
            set_error(ERR_INVALID_ARGUMENT, "pose must be finite");
            return Bool::FALSE;
        }

        collider.set_position(isometry_from_parts(translation, rotation));
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_translation(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    translation: Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };
        if !vec3_finite(translation) {
            set_error(ERR_INVALID_ARGUMENT, "translation must be finite");
            return Bool::FALSE;
        }

        collider.set_translation(vec3_to_rapier(translation));
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_rotation(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    rotation: Quat,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };
        if !quat_finite(rotation) {
            set_error(ERR_INVALID_ARGUMENT, "rotation must be finite");
            return Bool::FALSE;
        }

        collider.set_rotation(quat_to_rapier(rotation));
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_pose_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    translation: Vec3,
    rotation: Quat,
) -> u8 {
    ffi_guard(0, || {
        collider_set_pose(world, handle, translation, rotation).0
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_sensor(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    sensor: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };

        collider.set_sensor(sensor.0 != 0);
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_sensor_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    sensor: Bool,
) -> u8 {
    ffi_guard(0, || collider_set_sensor(world, handle, sensor).0)
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_friction(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    friction: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };
        if !friction.is_finite() || friction < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "friction must be finite and non-negative",
            );
            return Bool::FALSE;
        }

        collider.set_friction(friction);
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_friction_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    friction: f64,
) -> u8 {
    ffi_guard(0, || collider_set_friction(world, handle, friction).0)
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_restitution(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    restitution: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };
        if !restitution.is_finite() || restitution < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "restitution must be finite and non-negative",
            );
            return Bool::FALSE;
        }

        collider.set_restitution(restitution);
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_friction_combine_rule(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    rule: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };

        let rule = match rule {
            1 => CoefficientCombineRule::Min,
            2 => CoefficientCombineRule::Multiply,
            3 => CoefficientCombineRule::Max,
            4 => CoefficientCombineRule::ClampedSum,
            _ => CoefficientCombineRule::Average,
        };
        collider.set_friction_combine_rule(rule);
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_restitution_combine_rule(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    rule: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };

        let rule = match rule {
            1 => CoefficientCombineRule::Min,
            2 => CoefficientCombineRule::Multiply,
            3 => CoefficientCombineRule::Max,
            4 => CoefficientCombineRule::ClampedSum,
            _ => CoefficientCombineRule::Average,
        };
        collider.set_restitution_combine_rule(rule);
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_restitution_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    restitution: f64,
) -> u8 {
    ffi_guard(0, || collider_set_restitution(world, handle, restitution).0)
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_collision_groups(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    groups: InteractionGroupsDesc,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };

        collider.set_collision_groups(interaction_groups_to_rapier(groups));
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_collision_groups_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    groups: InteractionGroupsDesc,
) -> u8 {
    ffi_guard(0, || collider_set_collision_groups(world, handle, groups).0)
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_solver_groups(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    groups: InteractionGroupsDesc,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };

        collider.set_solver_groups(interaction_groups_to_rapier(groups));
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_solver_groups_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    groups: InteractionGroupsDesc,
) -> u8 {
    ffi_guard(0, || collider_set_solver_groups(world, handle, groups).0)
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_active_events(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    active_events_bits: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };

        collider.set_active_events(active_events_from_bits(active_events_bits));
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_active_events_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    active_events_bits: u32,
) -> u8 {
    ffi_guard(0, || {
        collider_set_active_events(world, handle, active_events_bits).0
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_active_hooks(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    active_hooks_bits: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };

        collider.set_active_hooks(active_hooks_from_bits(active_hooks_bits));
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_active_hooks_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    active_hooks_bits: u32,
) -> u8 {
    ffi_guard(0, || {
        collider_set_active_hooks(world, handle, active_hooks_bits).0
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_contact_force_event_threshold(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    threshold: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(collider) = world
            .inner
            .colliders
            .get_mut(unpack_collider_handle(handle))
        else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return Bool::FALSE;
        };
        if !threshold.is_finite() || threshold < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "threshold must be finite and non-negative",
            );
            return Bool::FALSE;
        }

        collider.set_contact_force_event_threshold(threshold);
        Bool::TRUE
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_contact_force_event_threshold_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    threshold: f64,
) -> u8 {
    ffi_guard(0, || {
        collider_set_contact_force_event_threshold(world, handle, threshold).0
    })
}

/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_get_density(
    world: *const WorldHandle,
    handle: ColliderHandleRaw,
) -> f64 {
    ffi_guard(0.0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0.0;
        };

        let Some(collider) = world.inner.colliders.get(unpack_collider_handle(handle)) else {
            set_error(ERR_NOT_FOUND, "collider not found");
            return 0.0;
        };
        collider.density()
    })
}
