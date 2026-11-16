use crate::convert::quat_to_rapier;
use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    AabbDesc, Bool, ColliderBuilderHandle, ColliderHandleRaw, ColliderMemHandle,
    InteractionGroupsDesc, Obb, Quat, RigidBodyHandleRaw, ShapeDesc, Sphere, Vec3, WorldHandle,
    active_events_from_bits, active_hooks_from_bits, interaction_groups_to_rapier,
    isometry_from_parts, pack_collider_handle, quat_finite, quat_from_rapier, shape_desc_valid,
    shape_from_desc, unpack_collider_handle, unpack_rigid_body_handle, vec3_finite,
    vec3_from_rapier, vec3_to_rapier,
};
use rapier3d::math::{Pose, Rotation, Vector};
use rapier3d::na::Unit;
use rapier3d::prelude::{Array2, CoefficientCombineRule, ColliderBuilder, SharedShape, TypedShape};

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

const MIN_HALF_EXTENT: f64 = 1.0e-9;
const MAX_RAW_POINTS: u32 = 1_000_000;
const MAX_HEIGHTMAP_CELLS: usize = 4_000_000;
const MAX_EDGE_COUNT: u32 = 1_000_000;
const MAX_SPHERE_COUNT: u32 = 1_000_000;
/// 复合碰撞体的盒数上限：跟 voxel 那边保持一致，按 MC 最大世界高度放大。
const MAX_COMPOUND_PARTS: u32 = 1_000_000;

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
    let Some(values) =
        (unsafe { crate::rapier::ffi::convert::checked_input_slice(points_xyz, value_count) })
    else {
        set_error(ERR_INVALID_ARGUMENT, "invalid points buffer");
        return None;
    };
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
    let Some(data) = (unsafe { crate::rapier::ffi::convert::checked_input_slice(box_data, total) })
    else {
        set_error(ERR_INVALID_ARGUMENT, "invalid box data buffer");
        return None;
    };

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
        let Some(values) =
            (unsafe { crate::rapier::ffi::convert::checked_input_slice(data, value_count) })
        else {
            set_error(ERR_INVALID_ARGUMENT, "invalid collider data buffer");
            return std::ptr::null_mut();
        };
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
        let Some(indices) =
            (unsafe { crate::rapier::ffi::convert::checked_input_slice(edges, index_count) })
        else {
            set_error(ERR_INVALID_ARGUMENT, "invalid edge index buffer");
            return std::ptr::null_mut();
        };
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
        let Some(values) = (unsafe {
            crate::rapier::ffi::convert::checked_input_slice(spheres_xyzw, value_count)
        }) else {
            set_error(ERR_INVALID_ARGUMENT, "invalid sphere buffer");
            return std::ptr::null_mut();
        };
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
pub extern "C" fn collider_builder_build(
    builder: *mut ColliderBuilderHandle,
) -> *mut ColliderMemHandle {
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
        Box::into_raw(Box::new(ColliderMemHandle {
            inner: inner.build(),
        }))
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
pub extern "C" fn collider_destroy_raw(collider: *mut ColliderMemHandle) {
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
    memory_handle: *mut ColliderMemHandle,
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

        let mut built = unsafe { Box::from_raw(memory_handle).inner };
        world.inner.runtime_settings.apply_collider(&mut built);
        let handle = world.inner.colliders.insert(built);
        crate::rapier::world::invalidate_region_index(&mut world.inner);
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
    memory_handle: *mut ColliderMemHandle,
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

        let mut built = unsafe { Box::from_raw(memory_handle).inner };
        world.inner.runtime_settings.apply_collider(&mut built);
        let handle = world.inner.colliders.insert_with_parent(
            built,
            unpack_rigid_body_handle(parent),
            &mut world.inner.bodies,
        );
        crate::rapier::world::invalidate_region_index(&mut world.inner);
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
                &mut world.inner.upstream_soft_bodies,
                wake_up.0 != 0,
            )
            .is_some();
        if !removed {
            set_error(ERR_NOT_FOUND, "collider not found");
        } else {
            crate::rapier::world::refresh_region_body(
                &mut world.inner,
                unpack_collider_handle(handle),
            );
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
) -> *mut ColliderMemHandle {
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

        Box::into_raw(Box::new(ColliderMemHandle { inner: collider }))
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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

/// Set the rolling-resistance coefficient of a collider.
///
/// Rolling resistance opposes *rolling* — rotation about an axis lying in the
/// contact tangent plane — not sliding. Coulomb friction already stops slipping;
/// a ball that rolls without slipping has zero slip velocity, so without this
/// coefficient it rolls forever. Typical values: ~0.001 steel-on-steel,
/// ~0.01-0.03 tire-on-asphalt. `0.0` disables the constraint (default), which
/// keeps the solver on its pre-existing path.
///
/// The resisting torque per step is bounded by `μ_r · N · r_eff`, where `N` is
/// the accumulated normal impulse of the contact manifold.
///
/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_rolling_resistance(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    coefficient: f64,
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
        if !coefficient.is_finite() || coefficient < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "rolling_resistance must be finite and non-negative",
            );
            return Bool::FALSE;
        }

        collider.set_rolling_resistance(coefficient);
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
        Bool::TRUE
    })
}

/// `u8`-returning variant of `collider_set_rolling_resistance`.
///
/// # Safety
///
/// Same contract as `collider_set_rolling_resistance`.
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_rolling_resistance_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    coefficient: f64,
) -> u8 {
    ffi_guard(0, || {
        collider_set_rolling_resistance(world, handle, coefficient).0
    })
}

/// Read the rolling-resistance coefficient of a collider.
///
/// Returns `0.0` for an unknown collider or a null world, and sets
/// `ERR_NOT_FOUND` / `ERR_NULL_POINTER` accordingly.
///
/// # Safety
///
/// `world` must be a valid pointer returned by `world_create` and not yet destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn collider_get_rolling_resistance(
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
        collider.rolling_resistance()
    })
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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
        crate::rapier::world::refresh_region_body(&mut world.inner, unpack_collider_handle(handle));
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

// ── 通用属性通道（ADR-0016，collider 批） ────────────────────────────────────
//
// 这是 ABI 收敛跨模块复用的验证：`soft_body` 证明了这套模式在**一个模块内**可行，
// 但没证明它能**跨模块**。collider 与 soft_body 有三处实质差异，这一批就是为了
// 确认这些差异不会撑破设计。
//
// ## 与 soft_body 的三处差异
//
// 1. **实体句柄不同**：soft_body 用 `u32` 索引，collider 用 packed
//    `ColliderHandleRaw`（`u64`，含 generation）。通道签名因此是
//    `(world, handle, param, value)` 而不是 `(world, id, param, value)`。
// 2. **有 `_flag` 变体**：9 个 setter 里有 8 个另有 `_flag` 返回 `u8`。
//    通道只提供 `Bool` 版本 + 一个统一的 `_flag` 包装，不重复 9 次。
// 3. **必要副作用**：每个 setter 都要调 `refresh_region_body`，否则 region
//    缓存会过期。通道里必须逐个保留，漏一个就是静默的行为差异。
//
// 结论：设计撑住了。差异体现在签名与副作用，不影响"一个通道吸收 N 个入口"的
// 核心主张。

/// `collider_set_param` 的属性编号。
///
/// 与 `SOFT_PARAM_*` 是**独立的编号空间** —— 两个通道的实体类型不同，
/// 混用没有意义。
pub const COLLIDER_PARAM_FRICTION: u32 = 0;
pub const COLLIDER_PARAM_ROLLING_RESISTANCE: u32 = 1;
pub const COLLIDER_PARAM_RESTITUTION: u32 = 2;
pub const COLLIDER_PARAM_CONTACT_FORCE_EVENT_THRESHOLD: u32 = 3;
/// 摩擦合并规则：`0`=Average, `1`=Min, `2`=Multiply, `3`=Max, `4`=ClampedSum。
///
/// **注意**：与专用入口 `collider_set_friction_combine_rule` 一致，越界值会被
/// 静默当作 `Average`（该入口的 `_ =>` 分支）。这是既有行为，通道沿用而不是
/// 改成拒绝 —— 改语义属于另一个决定，不该混在收敛里。
pub const COLLIDER_PARAM_FRICTION_COMBINE_RULE: u32 = 4;
pub const COLLIDER_PARAM_RESTITUTION_COMBINE_RULE: u32 = 5;
/// 激活的事件位（bitfield，见 `active_events_from_bits`）。
pub const COLLIDER_PARAM_ACTIVE_EVENTS: u32 = 6;
/// 激活的钩子位（bitfield，见 `active_hooks_from_bits`）。
pub const COLLIDER_PARAM_ACTIVE_HOOKS: u32 = 7;
/// 传感器标志（`0` = 实体，非 `0` = 传感器）。
pub const COLLIDER_PARAM_SENSOR: u32 = 8;

/// 已分配的 collider 属性个数。
///
/// 只增不减，已分配的编号永不复用。
#[unsafe(no_mangle)]
pub extern "C" fn collider_param_abi_version() -> u32 {
    ffi_guard(0, || 9)
}

/// 通用 collider 标量属性写入通道。
///
/// 语义与 `soft_body_set_param` 一致（ADR-0016 三条规则）：
///
/// 1. **失败是"不改"** —— 任何一步校验失败，collider 状态完全不变。
/// 2. **未知 `param` 返回 `ERR_INVALID_ARGUMENT`**，不静默当作 0。
/// 3. 每个 `param` 的属性名与值域见 `docs/abi-convergence.md`。
///
/// # Returns
/// `Bool::TRUE` 成功；world 为 null → `ERR_NULL_POINTER`；collider 句柄无效 →
/// `ERR_NOT_FOUND`；`param` 未分配或值域非法 → `ERR_INVALID_ARGUMENT`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_param(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    param: u32,
    value: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "collider_set_param: world is null");
            return Bool::FALSE;
        };
        let ch = unpack_collider_handle(handle);
        let Some(collider) = world.inner.colliders.get_mut(ch) else {
            set_error(ERR_NOT_FOUND, "collider_set_param: collider not found");
            return Bool::FALSE;
        };
        // 每个分支的校验与对应专用入口逐条一致，否则同一件事通过两条路径会有
        // 两种结论。`refresh_region_body` 在全部成功路径上都要调用 —— 它是
        // region 缓存失效的必要副作用，漏掉会让后续查询读到陈旧几何。
        match param {
            COLLIDER_PARAM_FRICTION => {
                if !value.is_finite() || value < 0.0 {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "collider_set_param: friction must be finite and non-negative",
                    );
                    return Bool::FALSE;
                }
                collider.set_friction(value);
            }
            COLLIDER_PARAM_ROLLING_RESISTANCE => {
                if !value.is_finite() || value < 0.0 {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "collider_set_param: rolling resistance must be finite and non-negative",
                    );
                    return Bool::FALSE;
                }
                collider.set_rolling_resistance(value);
            }
            COLLIDER_PARAM_RESTITUTION => {
                if !value.is_finite() || value < 0.0 {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "collider_set_param: restitution must be finite and non-negative",
                    );
                    return Bool::FALSE;
                }
                collider.set_restitution(value);
            }
            COLLIDER_PARAM_CONTACT_FORCE_EVENT_THRESHOLD => {
                if !value.is_finite() || value < 0.0 {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "collider_set_param: contact-force threshold must be finite and \
                         non-negative",
                    );
                    return Bool::FALSE;
                }
                collider.set_contact_force_event_threshold(value);
            }
            COLLIDER_PARAM_FRICTION_COMBINE_RULE => {
                if !value.is_finite() || value < 0.0 || value > 4.0 || value.fract() != 0.0 {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "collider_set_param: combine rule must be an integer in 0..=4",
                    );
                    return Bool::FALSE;
                }
                collider.set_friction_combine_rule(combine_rule(value as u32));
            }
            COLLIDER_PARAM_RESTITUTION_COMBINE_RULE => {
                if !value.is_finite() || value < 0.0 || value > 4.0 || value.fract() != 0.0 {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "collider_set_param: combine rule must be an integer in 0..=4",
                    );
                    return Bool::FALSE;
                }
                collider.set_restitution_combine_rule(combine_rule(value as u32));
            }
            COLLIDER_PARAM_ACTIVE_EVENTS => {
                // 位域是 u32 语义：拒绝负数、小数和越界。
                if !value.is_finite()
                    || value < 0.0
                    || value > u32::MAX as f64
                    || value.fract() != 0.0
                {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "collider_set_param: active events must be a u32 bitfield",
                    );
                    return Bool::FALSE;
                }
                collider.set_active_events(active_events_from_bits(value as u32));
            }
            COLLIDER_PARAM_ACTIVE_HOOKS => {
                if !value.is_finite()
                    || value < 0.0
                    || value > u32::MAX as f64
                    || value.fract() != 0.0
                {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "collider_set_param: active hooks must be a u32 bitfield",
                    );
                    return Bool::FALSE;
                }
                collider.set_active_hooks(active_hooks_from_bits(value as u32));
            }
            COLLIDER_PARAM_SENSOR => {
                if !value.is_finite() {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "collider_set_param: sensor flag must be finite",
                    );
                    return Bool::FALSE;
                }
                collider.set_sensor(value != 0.0);
            }
            _ => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "collider_set_param: unknown param id (see collider_param_abi_version)",
                );
                return Bool::FALSE;
            }
        }
        crate::rapier::world::refresh_region_body(&mut world.inner, ch);
        crate::rapier::error::clear_error();
        Bool::TRUE
    })
}

/// `u8` 返回变体，语义同 [`collider_set_param`]。
///
/// 专用入口里有 8 个各有自己的 `_flag` 变体（`collider_set_friction_flag` 等）。
/// 通道只提供一个统一包装，而不是重复 8 次 —— 这正是收敛要省掉的东西。
///
/// # Safety
/// 同 [`collider_set_param`]。
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_param_flag(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    param: u32,
    value: f64,
) -> u8 {
    ffi_guard(0, || collider_set_param(world, handle, param, value).0)
}
/// 把 C 侧的规则编号映射成 rapier 枚举。
///
/// **与 `collider_set_friction_combine_rule` 的内联 `match` 逐条一致**：越界值
/// 落到 `Average`。之所以不在这里改成拒绝，是因为那会改变既有语义；通道的职责
/// 是等价迁移，不是顺手修 bug。真要改，应该两个入口一起改并单独记 ADR。
fn combine_rule(rule: u32) -> CoefficientCombineRule {
    match rule {
        1 => CoefficientCombineRule::Min,
        2 => CoefficientCombineRule::Multiply,
        3 => CoefficientCombineRule::Max,
        4 => CoefficientCombineRule::ClampedSum,
        _ => CoefficientCombineRule::Average,
    }
}

// ── 向量与复合载荷通道（ADR-0016 第四批，验证设计上限） ──────────────────────
//
// 前三批都是纯标量。这一批处理两类**不同形状**的载荷，目的是测出这套设计的真实
// 上限：
//
// 1. **向量**：`Vec3`（3 个 f64）和 `Quat`（4 个 f64）。`set_param` 的单个
//    `f64` 装不下，需要独立签名。
// 2. **复合**：`InteractionGroupsDesc`（2 个 u32）。既不是标量也不是向量，
//    而是"固定宽度的小结构体"。
//
// ## 设计结论（先说答案）
//
// 两类都能收敛，但**代价不同**：
//
// * 向量用 `(x, y, z)` / `(x, y, z, w)` 展开成独立参数 —— 干净，一个通道吸收
//   3 个入口（pose / translation / rotation）。
// * 复合载荷用 `(a, b)` 两个 u32 —— 也干净，但**只能吸收载荷恰好是 2 个 u32 的
//   入口**。载荷变成 3 个或含混合类型时，就得再开一个通道。
//
// 也就是说：**标量与向量的收敛是"形状级"的，复合载荷的收敛是"宽度级"的。**
// 后者不能无限扩展，这是这套设计的真实边界。
//
// 顺带一个观察：`InteractionGroupsDesc` 只有 2 个 u32，所以它其实**可以**塞进
// 一个 `u64`（高 32 位 memberships，低 32 位 filter）。但那样会让 C 侧调用者
// 手工做位移，容易错。展开成两个参数更清楚，代价是通道签名固定为 2 个 u32。

/// `collider_set_vec3_param` 的属性编号。
pub const COLLIDER_VEC3_TRANSLATION: u32 = 0;
/// 旋转用四元数，走 `collider_set_quat_param`（4 个分量）。
pub const COLLIDER_QUAT_ROTATION: u32 = 0;

/// 已分配的向量属性个数。
#[unsafe(no_mangle)]
pub extern "C" fn collider_vec3_param_abi_version() -> u32 {
    ffi_guard(0, || 1)
}

/// 已分配的四元数属性个数。
#[unsafe(no_mangle)]
pub extern "C" fn collider_quat_param_abi_version() -> u32 {
    ffi_guard(0, || 1)
}

/// 已分配的"2×u32 复合载荷"属性个数。
#[unsafe(no_mangle)]
pub extern "C" fn collider_u32x2_param_abi_version() -> u32 {
    ffi_guard(0, || 2)
}

/// `collider_set_u32x2_param` 的属性编号。
pub const COLLIDER_U32X2_COLLISION_GROUPS: u32 = 0;
pub const COLLIDER_U32X2_SOLVER_GROUPS: u32 = 1;

/// 向量属性写入通道（`Vec3` 载荷）。
///
/// # Returns
/// `Bool::TRUE` 成功；world 为 null → `ERR_NULL_POINTER`；句柄无效 →
/// `ERR_NOT_FOUND`；`param` 未分配或分量非有限 → `ERR_INVALID_ARGUMENT`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_vec3_param(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    param: u32,
    x: f64,
    y: f64,
    z: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "collider_set_vec3_param: world is null");
            return Bool::FALSE;
        };
        let ch = unpack_collider_handle(handle);
        let Some(collider) = world.inner.colliders.get_mut(ch) else {
            set_error(ERR_NOT_FOUND, "collider_set_vec3_param: collider not found");
            return Bool::FALSE;
        };
        match param {
            COLLIDER_VEC3_TRANSLATION => {
                let translation = Vec3 { x, y, z };
                if !vec3_finite(translation) {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "collider_set_vec3_param: translation must be finite",
                    );
                    return Bool::FALSE;
                }
                // 只改平移，保留现有旋转 —— 与 `collider_set_translation` 一致。
                // `position()` 返回 `Pose`，其 `translation` 已是 `DVec3`，没有
                // 额外的 `.vector` 层。
                let pose = collider.position();
                collider.set_position(Pose::from_parts(vec3_to_rapier(translation), pose.rotation));
            }
            _ => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "collider_set_vec3_param: unknown param id \
                     (see collider_vec3_param_abi_version)",
                );
                return Bool::FALSE;
            }
        }
        crate::rapier::world::refresh_region_body(&mut world.inner, ch);
        crate::rapier::error::clear_error();
        Bool::TRUE
    })
}

/// 四元数属性写入通道（`Quat` 载荷）。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_quat_param(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    param: u32,
    x: f64,
    y: f64,
    z: f64,
    w: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "collider_set_quat_param: world is null");
            return Bool::FALSE;
        };
        let ch = unpack_collider_handle(handle);
        let Some(collider) = world.inner.colliders.get_mut(ch) else {
            set_error(ERR_NOT_FOUND, "collider_set_quat_param: collider not found");
            return Bool::FALSE;
        };
        match param {
            COLLIDER_QUAT_ROTATION => {
                // `Quat` 的字段是 `i/j/k/w`（不是 `x/y/z/w`）—— 沿用仓库既有
                // 的四元数分量命名，避免和 glam/nalgebra 的约定混淆。
                let rotation = Quat {
                    i: x,
                    j: y,
                    k: z,
                    w,
                };
                if !quat_finite(rotation) {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "collider_set_quat_param: rotation must be finite",
                    );
                    return Bool::FALSE;
                }
                // 只改旋转，保留现有平移 —— 与 `collider_set_rotation` 一致。
                collider.set_rotation(quat_to_rapier(rotation));
            }
            _ => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "collider_set_quat_param: unknown param id \
                     (see collider_quat_param_abi_version)",
                );
                return Bool::FALSE;
            }
        }
        crate::rapier::world::refresh_region_body(&mut world.inner, ch);
        crate::rapier::error::clear_error();
        Bool::TRUE
    })
}

/// 复合载荷写入通道（固定 2 个 `u32`，如 `InteractionGroupsDesc`）。
///
/// **这是本设计的边界**：通道签名写死了 2 个 `u32`。载荷宽度不同的复合属性
/// （3 个字段、混合类型）无法复用这个通道，必须另开一个。所以复合载荷的收敛
/// 是"宽度级"的，不像标量那样是"形状级"的。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn collider_set_u32x2_param(
    world: *mut WorldHandle,
    handle: ColliderHandleRaw,
    param: u32,
    a: u32,
    b: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "collider_set_u32x2_param: world is null");
            return Bool::FALSE;
        };
        let ch = unpack_collider_handle(handle);
        let Some(collider) = world.inner.colliders.get_mut(ch) else {
            set_error(
                ERR_NOT_FOUND,
                "collider_set_u32x2_param: collider not found",
            );
            return Bool::FALSE;
        };
        let groups = InteractionGroupsDesc {
            memberships: a,
            filter: b,
        };
        match param {
            COLLIDER_U32X2_COLLISION_GROUPS => {
                collider.set_collision_groups(interaction_groups_to_rapier(groups));
            }
            COLLIDER_U32X2_SOLVER_GROUPS => {
                collider.set_solver_groups(interaction_groups_to_rapier(groups));
            }
            _ => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "collider_set_u32x2_param: unknown param id \
                     (see collider_u32x2_param_abi_version)",
                );
                return Bool::FALSE;
            }
        }
        crate::rapier::world::refresh_region_body(&mut world.inner, ch);
        crate::rapier::error::clear_error();
        Bool::TRUE
    })
}
