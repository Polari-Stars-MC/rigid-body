use std::slice;

use rayon::prelude::*;

use rapier3d::math::{Pose, Rotation, Vector};
use rapier3d::prelude::{ColliderBuilder, SharedShape};

use crate::rapier::ffi::{
    AabbDesc, Bool, ColliderBuilderHandle, ColliderHandleRaw, Obb, QueryFilterDesc,
    RigidBodyHandleRaw, Vec3, VoxelBuildStats, VoxelColliderMode, VoxelColliderOptions,
    WorldHandle, quat_finite, quat_to_rapier, vec3_finite, voxel_collider_mode_from_raw,
};

const MAX_VOXEL_CELLS: usize = 262_144;
const MAX_COMPOUND_PARTS: usize = 100_000;
const MAX_SURFACE_VERTICES: usize = 1_000_000;
const MAX_SURFACE_TRIANGLES: usize = 500_000;

struct VoxelGrid<'a> {
    voxels: &'a [u8],
    size_x: usize,
    size_y: usize,
    size_z: usize,
    voxel_size: f64,
    origin: Vec3,
}

struct OwnedVoxelGrid {
    voxels: Vec<u8>,
    size_x: usize,
    size_y: usize,
    size_z: usize,
    voxel_size: f64,
    origin: Vec3,
}

impl OwnedVoxelGrid {
    fn as_grid(&self) -> VoxelGrid<'_> {
        VoxelGrid {
            voxels: &self.voxels,
            size_x: self.size_x,
            size_y: self.size_y,
            size_z: self.size_z,
            voxel_size: self.voxel_size,
            origin: self.origin,
        }
    }
}

impl VoxelGrid<'_> {
    fn index(&self, x: usize, y: usize, z: usize) -> Option<usize> {
        let plane = self.size_x.checked_mul(self.size_y)?;
        let base = z.checked_mul(plane)?;
        let row = y.checked_mul(self.size_x)?;
        base.checked_add(row)?.checked_add(x)
    }

    fn is_solid(&self, x: usize, y: usize, z: usize) -> bool {
        self.index(x, y, z)
            .and_then(|index| self.voxels.get(index))
            .is_some_and(|voxel| *voxel != 0)
    }

    fn is_solid_checked(&self, x: isize, y: isize, z: isize) -> bool {
        if x < 0
            || y < 0
            || z < 0
            || x as usize >= self.size_x
            || y as usize >= self.size_y
            || z as usize >= self.size_z
        {
            return false;
        }

        self.is_solid(x as usize, y as usize, z as usize)
    }

    fn cell_min(&self, x: usize, y: usize, z: usize) -> Vector {
        Vector::new(
            self.origin.x + x as f64 * self.voxel_size,
            self.origin.y + y as f64 * self.voxel_size,
            self.origin.z + z as f64 * self.voxel_size,
        )
    }
}

fn choose_mode(solid_count: usize, options: VoxelColliderOptions) -> VoxelColliderMode {
    let mode = voxel_collider_mode_from_raw(options.mode);
    if mode != VoxelColliderMode::Auto {
        return mode;
    }
    if solid_count <= options.small_voxel_limit as usize {
        return VoxelColliderMode::Cuboids;
    }
    if options.dynamic_body.0 != 0 {
        return VoxelColliderMode::GreedyCuboids;
    }
    if solid_count >= options.mesh_voxel_limit as usize {
        return VoxelColliderMode::SurfaceMesh;
    }
    VoxelColliderMode::GreedyCuboids
}

#[allow(clippy::too_many_arguments)]
fn push_cuboid(
    grid: &VoxelGrid<'_>,
    parts: &mut Vec<(Pose, SharedShape)>,
    x: usize,
    y: usize,
    z: usize,
    max_x: usize,
    max_y: usize,
    max_z: usize,
) {
    let size_x = (max_x - x) as f64 * grid.voxel_size;
    let size_y = (max_y - y) as f64 * grid.voxel_size;
    let size_z = (max_z - z) as f64 * grid.voxel_size;
    if size_x <= 0.0 || size_y <= 0.0 || size_z <= 0.0 {
        return;
    }

    let center = Vector::new(
        grid.origin.x + (x as f64 + (max_x - x) as f64 * 0.5) * grid.voxel_size,
        grid.origin.y + (y as f64 + (max_y - y) as f64 * 0.5) * grid.voxel_size,
        grid.origin.z + (z as f64 + (max_z - z) as f64 * 0.5) * grid.voxel_size,
    );
    parts.push((
        Pose::from_parts(center, Rotation::IDENTITY),
        SharedShape::cuboid(size_x * 0.5, size_y * 0.5, size_z * 0.5),
    ));
}

fn build_cuboids(grid: &VoxelGrid<'_>, solid_count: usize) -> Option<ColliderBuilder> {
    if solid_count > MAX_COMPOUND_PARTS {
        return None;
    }

    let mut parts = Vec::with_capacity(solid_count);
    for z in 0..grid.size_z {
        for y in 0..grid.size_y {
            for x in 0..grid.size_x {
                if grid.is_solid(x, y, z) {
                    push_cuboid(grid, &mut parts, x, y, z, x + 1, y + 1, z + 1);
                    if parts.len() > MAX_COMPOUND_PARTS {
                        return None;
                    }
                }
            }
        }
    }
    (!parts.is_empty()).then(|| ColliderBuilder::compound(parts))
}

fn build_greedy_cuboids(grid: &VoxelGrid<'_>) -> Option<ColliderBuilder> {
    let chunk_size = 32usize;
    let y_starts: Vec<_> = (0..grid.size_y).step_by(chunk_size).collect();
    let chunk_parts = y_starts
        .into_par_iter()
        .map(|y_start| {
            build_greedy_cuboids_y_range(grid, y_start, (y_start + chunk_size).min(grid.size_y))
        })
        .collect::<Option<Vec<_>>>()?;
    let total_parts = chunk_parts
        .iter()
        .try_fold(0usize, |total, parts| total.checked_add(parts.len()))?;
    if total_parts > MAX_COMPOUND_PARTS {
        return None;
    }

    let mut parts = Vec::with_capacity(total_parts);
    for mut chunk in chunk_parts {
        parts.append(&mut chunk);
    }
    (!parts.is_empty()).then(|| ColliderBuilder::compound(parts))
}

fn build_greedy_cuboids_y_range(
    grid: &VoxelGrid<'_>,
    y_start: usize,
    y_end: usize,
) -> Option<Vec<(Pose, SharedShape)>> {
    // Use generation-counter visited array instead of bool vec — avoids
    // O(n) zeroing per call.  Each invocation bumps `gen`; a cell is
    // "visited" when visited[cell] == gen.
    let cell_count = grid.voxels.len();
    let mut visited: Vec<u32> = vec![0; cell_count];
    let generation = 1u32;
    let mut parts = Vec::new();

    for z in 0..grid.size_z {
        for y in y_start..y_end {
            for x in 0..grid.size_x {
                let start = grid.index(x, y, z)?;
                if visited[start] == generation || !grid.is_solid(x, y, z) {
                    continue;
                }

                let mut max_x = x + 1;
                while max_x < grid.size_x {
                    let i = grid.index(max_x, y, z)?;
                    if visited[i] == generation || !grid.is_solid(max_x, y, z) {
                        break;
                    }
                    max_x += 1;
                }

                let mut max_y = y + 1;
                'expand_y: while max_y < y_end {
                    for xx in x..max_x {
                        let i = grid.index(xx, max_y, z)?;
                        if visited[i] == generation || !grid.is_solid(xx, max_y, z) {
                            break 'expand_y;
                        }
                    }
                    max_y += 1;
                }

                let mut max_z = z + 1;
                'expand_z: while max_z < grid.size_z {
                    for yy in y..max_y {
                        for xx in x..max_x {
                            let i = grid.index(xx, yy, max_z)?;
                            if visited[i] == generation || !grid.is_solid(xx, yy, max_z) {
                                break 'expand_z;
                            }
                        }
                    }
                    max_z += 1;
                }

                for zz in z..max_z {
                    for yy in y..max_y {
                        for xx in x..max_x {
                            let i = grid.index(xx, yy, zz)?;
                            visited[i] = generation;
                        }
                    }
                }

                push_cuboid(grid, &mut parts, x, y, z, max_x, max_y, max_z);
            }
        }
    }

    Some(parts)
}

fn count_greedy_cuboids(grid: &VoxelGrid<'_>) -> Option<usize> {
    let mut visited: Vec<u32> = vec![0; grid.voxels.len()];
    let generation = 1u32;
    let mut count = 0usize;

    for z in 0..grid.size_z {
        for y in 0..grid.size_y {
            for x in 0..grid.size_x {
                let start = grid.index(x, y, z)?;
                if visited[start] == generation || !grid.is_solid(x, y, z) {
                    continue;
                }

                let mut max_x = x + 1;
                while max_x < grid.size_x {
                    let i = grid.index(max_x, y, z)?;
                    if visited[i] == generation || !grid.is_solid(max_x, y, z) {
                        break;
                    }
                    max_x += 1;
                }

                let mut max_y = y + 1;
                'expand_y: while max_y < grid.size_y {
                    for xx in x..max_x {
                        let i = grid.index(xx, max_y, z)?;
                        if visited[i] == generation || !grid.is_solid(xx, max_y, z) {
                            break 'expand_y;
                        }
                    }
                    max_y += 1;
                }

                let mut max_z = z + 1;
                'expand_z: while max_z < grid.size_z {
                    for yy in y..max_y {
                        for xx in x..max_x {
                            let i = grid.index(xx, yy, max_z)?;
                            if visited[i] == generation || !grid.is_solid(xx, yy, max_z) {
                                break 'expand_z;
                            }
                        }
                    }
                    max_z += 1;
                }

                for zz in z..max_z {
                    for yy in y..max_y {
                        for xx in x..max_x {
                            let i = grid.index(xx, yy, zz)?;
                            visited[i] = generation;
                        }
                    }
                }

                count += 1;
            }
        }
    }

    Some(count)
}

fn count_surface_faces(grid: &VoxelGrid<'_>) -> usize {
    let mut faces = 0usize;
    for z in 0..grid.size_z {
        for y in 0..grid.size_y {
            for x in 0..grid.size_x {
                if !grid.is_solid(x, y, z) {
                    continue;
                }
                let x = x as isize;
                let y = y as isize;
                let z = z as isize;
                faces += (!grid.is_solid_checked(x - 1, y, z)) as usize;
                faces += (!grid.is_solid_checked(x + 1, y, z)) as usize;
                faces += (!grid.is_solid_checked(x, y - 1, z)) as usize;
                faces += (!grid.is_solid_checked(x, y + 1, z)) as usize;
                faces += (!grid.is_solid_checked(x, y, z - 1)) as usize;
                faces += (!grid.is_solid_checked(x, y, z + 1)) as usize;
            }
        }
    }
    faces
}

fn push_face(
    vertices: &mut Vec<Vector>,
    indices: &mut Vec<[u32; 3]>,
    corners: [Vector; 4],
) -> Option<()> {
    if vertices.len() + 4 > MAX_SURFACE_VERTICES || indices.len() + 2 > MAX_SURFACE_TRIANGLES {
        return None;
    }

    let base = u32::try_from(vertices.len()).ok()?;
    vertices.extend(corners);
    indices.push([base, base + 1, base + 2]);
    indices.push([base, base + 2, base + 3]);
    Some(())
}

fn build_surface_mesh(grid: &VoxelGrid<'_>, solid_count: usize) -> Option<ColliderBuilder> {
    let face_capacity = solid_count.saturating_mul(6);
    let mut vertices = Vec::with_capacity(face_capacity.saturating_mul(4).min(65_536));
    let mut indices = Vec::with_capacity(face_capacity.saturating_mul(2).min(32_768));
    let s = grid.voxel_size;

    for z in 0..grid.size_z {
        for y in 0..grid.size_y {
            for x in 0..grid.size_x {
                if !grid.is_solid(x, y, z) {
                    continue;
                }

                let min = grid.cell_min(x, y, z);
                let max = min + Vector::new(s, s, s);
                let x = x as isize;
                let y = y as isize;
                let z = z as isize;

                if !grid.is_solid_checked(x - 1, y, z) {
                    push_face(
                        &mut vertices,
                        &mut indices,
                        [
                            Vector::new(min.x, min.y, min.z),
                            Vector::new(min.x, min.y, max.z),
                            Vector::new(min.x, max.y, max.z),
                            Vector::new(min.x, max.y, min.z),
                        ],
                    )?;
                }
                if !grid.is_solid_checked(x + 1, y, z) {
                    push_face(
                        &mut vertices,
                        &mut indices,
                        [
                            Vector::new(max.x, min.y, min.z),
                            Vector::new(max.x, max.y, min.z),
                            Vector::new(max.x, max.y, max.z),
                            Vector::new(max.x, min.y, max.z),
                        ],
                    )?;
                }
                if !grid.is_solid_checked(x, y - 1, z) {
                    push_face(
                        &mut vertices,
                        &mut indices,
                        [
                            Vector::new(min.x, min.y, min.z),
                            Vector::new(max.x, min.y, min.z),
                            Vector::new(max.x, min.y, max.z),
                            Vector::new(min.x, min.y, max.z),
                        ],
                    )?;
                }
                if !grid.is_solid_checked(x, y + 1, z) {
                    push_face(
                        &mut vertices,
                        &mut indices,
                        [
                            Vector::new(min.x, max.y, min.z),
                            Vector::new(min.x, max.y, max.z),
                            Vector::new(max.x, max.y, max.z),
                            Vector::new(max.x, max.y, min.z),
                        ],
                    )?;
                }
                if !grid.is_solid_checked(x, y, z - 1) {
                    push_face(
                        &mut vertices,
                        &mut indices,
                        [
                            Vector::new(min.x, min.y, min.z),
                            Vector::new(min.x, max.y, min.z),
                            Vector::new(max.x, max.y, min.z),
                            Vector::new(max.x, min.y, min.z),
                        ],
                    )?;
                }
                if !grid.is_solid_checked(x, y, z + 1) {
                    push_face(
                        &mut vertices,
                        &mut indices,
                        [
                            Vector::new(min.x, min.y, max.z),
                            Vector::new(max.x, min.y, max.z),
                            Vector::new(max.x, max.y, max.z),
                            Vector::new(min.x, max.y, max.z),
                        ],
                    )?;
                }
            }
        }
    }

    if vertices.is_empty() {
        return None;
    }

    ColliderBuilder::trimesh(vertices, indices).ok()
}

fn build_voxel_collider(
    grid: &VoxelGrid<'_>,
    options: VoxelColliderOptions,
) -> Option<ColliderBuilder> {
    let solid_count = grid.voxels.iter().filter(|voxel| **voxel != 0).count();
    if solid_count == 0 {
        return None;
    }

    match choose_mode(solid_count, options) {
        VoxelColliderMode::Auto => unreachable!(),
        VoxelColliderMode::Cuboids => build_cuboids(grid, solid_count),
        VoxelColliderMode::GreedyCuboids => build_greedy_cuboids(grid),
        VoxelColliderMode::SurfaceMesh => build_surface_mesh(grid, solid_count),
    }
}

fn usize_to_u32(value: usize) -> u32 {
    value.min(u32::MAX as usize) as u32
}

fn compute_voxel_build_stats(
    grid: &VoxelGrid<'_>,
    options: VoxelColliderOptions,
) -> VoxelBuildStats {
    let cell_count = grid.voxels.len();
    let solid_count = grid.voxels.iter().filter(|voxel| **voxel != 0).count();
    if solid_count == 0 {
        return VoxelBuildStats {
            cell_count: usize_to_u32(cell_count),
            size_x: usize_to_u32(grid.size_x),
            size_y: usize_to_u32(grid.size_y),
            size_z: usize_to_u32(grid.size_z),
            ..VoxelBuildStats::default()
        };
    }

    let mode = choose_mode(solid_count, options);
    let mut stats = VoxelBuildStats {
        cell_count: usize_to_u32(cell_count),
        solid_count: usize_to_u32(solid_count),
        selected_mode: mode as u32,
        size_x: usize_to_u32(grid.size_x),
        size_y: usize_to_u32(grid.size_y),
        size_z: usize_to_u32(grid.size_z),
        ..VoxelBuildStats::default()
    };

    match mode {
        VoxelColliderMode::Auto => unreachable!(),
        VoxelColliderMode::Cuboids => {
            stats.estimated_parts = usize_to_u32(solid_count);
        }
        VoxelColliderMode::GreedyCuboids => {
            stats.estimated_parts = count_greedy_cuboids(grid).map(usize_to_u32).unwrap_or(0);
        }
        VoxelColliderMode::SurfaceMesh => {
            let faces = count_surface_faces(grid);
            stats.estimated_vertices = usize_to_u32(faces.saturating_mul(4));
            stats.estimated_triangles = usize_to_u32(faces.saturating_mul(2));
        }
    }

    stats
}

fn ceil_to_usize(value: f64) -> Option<usize> {
    if !value.is_finite() || value < 0.0 {
        return None;
    }
    let value = value.ceil();
    if value > usize::MAX as f64 {
        return None;
    }
    Some(value as usize)
}

fn build_aabb_voxel_grid(aabb: AabbDesc, voxel_size: f64) -> Option<OwnedVoxelGrid> {
    if !vec3_finite(aabb.mins)
        || !vec3_finite(aabb.maxs)
        || !voxel_size.is_finite()
        || voxel_size <= 0.0
        || aabb.mins.x >= aabb.maxs.x
        || aabb.mins.y >= aabb.maxs.y
        || aabb.mins.z >= aabb.maxs.z
    {
        return None;
    }

    let size_x = ceil_to_usize((aabb.maxs.x - aabb.mins.x) / voxel_size)?;
    let size_y = ceil_to_usize((aabb.maxs.y - aabb.mins.y) / voxel_size)?;
    let size_z = ceil_to_usize((aabb.maxs.z - aabb.mins.z) / voxel_size)?;
    if size_x == 0 || size_y == 0 || size_z == 0 {
        return None;
    }
    let len = size_x.checked_mul(size_y)?.checked_mul(size_z)?;
    if len > MAX_VOXEL_CELLS {
        return None;
    }

    Some(OwnedVoxelGrid {
        voxels: vec![1; len],
        size_x,
        size_y,
        size_z,
        voxel_size,
        origin: aabb.mins,
    })
}

fn obb_world_aabb(obb: Obb, rotation: Rotation) -> Option<AabbDesc> {
    if !vec3_finite(obb.center)
        || !vec3_finite(obb.half_extents)
        || !quat_finite(obb.rotation)
        || obb.half_extents.x <= 0.0
        || obb.half_extents.y <= 0.0
        || obb.half_extents.z <= 0.0
    {
        return None;
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
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            for sz in [-1.0, 1.0] {
                let local = Vector::new(
                    obb.half_extents.x * sx,
                    obb.half_extents.y * sy,
                    obb.half_extents.z * sz,
                );
                let point =
                    rotation * local + Vector::new(obb.center.x, obb.center.y, obb.center.z);
                mins.x = mins.x.min(point.x);
                mins.y = mins.y.min(point.y);
                mins.z = mins.z.min(point.z);
                maxs.x = maxs.x.max(point.x);
                maxs.y = maxs.y.max(point.y);
                maxs.z = maxs.z.max(point.z);
            }
        }
    }

    Some(AabbDesc { mins, maxs })
}

fn build_obb_voxel_grid(obb: Obb, voxel_size: f64) -> Option<OwnedVoxelGrid> {
    if !voxel_size.is_finite() || voxel_size <= 0.0 {
        return None;
    }
    let rotation = quat_to_rapier(obb.rotation);
    let bounds = obb_world_aabb(obb, rotation)?;
    let mut grid = build_aabb_voxel_grid(bounds, voxel_size)?;
    let inverse_rotation = rotation.inverse();
    let center = Vector::new(obb.center.x, obb.center.y, obb.center.z);
    let half = Vector::new(obb.half_extents.x, obb.half_extents.y, obb.half_extents.z);

    let mut solid_count = 0usize;
    for z in 0..grid.size_z {
        for y in 0..grid.size_y {
            for x in 0..grid.size_x {
                let world = Vector::new(
                    grid.origin.x + (x as f64 + 0.5) * voxel_size,
                    grid.origin.y + (y as f64 + 0.5) * voxel_size,
                    grid.origin.z + (z as f64 + 0.5) * voxel_size,
                );
                let local = inverse_rotation * (world - center);
                let solid =
                    local.x.abs() <= half.x && local.y.abs() <= half.y && local.z.abs() <= half.z;
                let index = z * grid.size_x * grid.size_y + y * grid.size_x + x;
                grid.voxels[index] = solid as u8;
                solid_count += solid as usize;
            }
        }
    }

    (solid_count > 0).then_some(grid)
}

fn builder_from_owned_grid(
    grid: OwnedVoxelGrid,
    options: VoxelColliderOptions,
) -> *mut ColliderBuilderHandle {
    let grid_ref = grid.as_grid();
    let Some(builder) = build_voxel_collider(&grid_ref, options) else {
        return std::ptr::null_mut();
    };

    Box::into_raw(Box::new(ColliderBuilderHandle { inner: builder }))
}

fn create_voxels_with_options(
    voxels: *const u8,
    size_x: u32,
    size_y: u32,
    size_z: u32,
    voxel_size: f64,
    origin: Vec3,
    options: VoxelColliderOptions,
) -> *mut ColliderBuilderHandle {
    if voxels.is_null()
        || size_x == 0
        || size_y == 0
        || size_z == 0
        || !voxel_size.is_finite()
        || voxel_size <= 0.0
        || !origin.x.is_finite()
        || !origin.y.is_finite()
        || !origin.z.is_finite()
    {
        return std::ptr::null_mut();
    }

    let Some(xy) = (size_x as usize).checked_mul(size_y as usize) else {
        return std::ptr::null_mut();
    };
    let Some(len) = xy.checked_mul(size_z as usize) else {
        return std::ptr::null_mut();
    };
    if len > MAX_VOXEL_CELLS {
        return std::ptr::null_mut();
    }

    let voxels = unsafe { slice::from_raw_parts(voxels, len) };
    let grid = VoxelGrid {
        voxels,
        size_x: size_x as usize,
        size_y: size_y as usize,
        size_z: size_z as usize,
        voxel_size,
        origin,
    };

    let Some(builder) = build_voxel_collider(&grid, options) else {
        return std::ptr::null_mut();
    };

    Box::into_raw(Box::new(ColliderBuilderHandle { inner: builder }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_voxels(
    voxels: *const u8,
    size_x: u32,
    size_y: u32,
    size_z: u32,
    voxel_size: f64,
    origin: Vec3,
    options: VoxelColliderOptions,
) -> *mut ColliderBuilderHandle {
    create_voxels_with_options(voxels, size_x, size_y, size_z, voxel_size, origin, options)
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_voxels_auto(
    voxels: *const u8,
    size_x: u32,
    size_y: u32,
    size_z: u32,
    voxel_size: f64,
    origin: Vec3,
    dynamic_body: crate::rapier::ffi::Bool,
) -> *mut ColliderBuilderHandle {
    create_voxels_with_options(
        voxels,
        size_x,
        size_y,
        size_z,
        voxel_size,
        origin,
        VoxelColliderOptions {
            dynamic_body,
            ..VoxelColliderOptions::default()
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn voxel_build_stats(
    voxels: *const u8,
    size_x: u32,
    size_y: u32,
    size_z: u32,
    voxel_size: f64,
    origin: Vec3,
    options: VoxelColliderOptions,
) -> VoxelBuildStats {
    if voxels.is_null()
        || size_x == 0
        || size_y == 0
        || size_z == 0
        || !voxel_size.is_finite()
        || voxel_size <= 0.0
        || !origin.x.is_finite()
        || !origin.y.is_finite()
        || !origin.z.is_finite()
    {
        return VoxelBuildStats::default();
    }
    let Some(xy) = (size_x as usize).checked_mul(size_y as usize) else {
        return VoxelBuildStats::default();
    };
    let Some(len) = xy.checked_mul(size_z as usize) else {
        return VoxelBuildStats::default();
    };
    if len > MAX_VOXEL_CELLS {
        return VoxelBuildStats::default();
    }

    let voxels = unsafe { slice::from_raw_parts(voxels, len) };
    let grid = VoxelGrid {
        voxels,
        size_x: size_x as usize,
        size_y: size_y as usize,
        size_z: size_z as usize,
        voxel_size,
        origin,
    };
    compute_voxel_build_stats(&grid, options)
}

#[unsafe(no_mangle)]
pub extern "C" fn voxel_aabb_build_stats(
    aabb: AabbDesc,
    voxel_size: f64,
    options: VoxelColliderOptions,
) -> VoxelBuildStats {
    let Some(grid) = build_aabb_voxel_grid(aabb, voxel_size) else {
        return VoxelBuildStats::default();
    };
    compute_voxel_build_stats(&grid.as_grid(), options)
}

#[unsafe(no_mangle)]
pub extern "C" fn voxel_obb_build_stats(
    obb: Obb,
    voxel_size: f64,
    options: VoxelColliderOptions,
) -> VoxelBuildStats {
    let Some(grid) = build_obb_voxel_grid(obb, voxel_size) else {
        return VoxelBuildStats::default();
    };
    compute_voxel_build_stats(&grid.as_grid(), options)
}

#[unsafe(no_mangle)]
pub extern "C" fn voxel_aabb_build_stats_out(
    aabb: AabbDesc,
    voxel_size: f64,
    options: VoxelColliderOptions,
    out_stats: *mut VoxelBuildStats,
) {
    let Some(out_stats) = (unsafe { out_stats.as_mut() }) else {
        return;
    };
    *out_stats = voxel_aabb_build_stats(aabb, voxel_size, options);
}

#[unsafe(no_mangle)]
pub extern "C" fn voxel_obb_build_stats_out(
    obb: Obb,
    voxel_size: f64,
    options: VoxelColliderOptions,
    out_stats: *mut VoxelBuildStats,
) {
    let Some(out_stats) = (unsafe { out_stats.as_mut() }) else {
        return;
    };
    *out_stats = voxel_obb_build_stats(obb, voxel_size, options);
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_voxel_aabb(
    aabb: AabbDesc,
    voxel_size: f64,
    options: VoxelColliderOptions,
) -> *mut ColliderBuilderHandle {
    let Some(grid) = build_aabb_voxel_grid(aabb, voxel_size) else {
        return std::ptr::null_mut();
    };
    builder_from_owned_grid(grid, options)
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_voxel_aabb_auto(
    aabb: AabbDesc,
    voxel_size: f64,
    dynamic_body: crate::rapier::ffi::Bool,
) -> *mut ColliderBuilderHandle {
    collider_builder_create_voxel_aabb(
        aabb,
        voxel_size,
        VoxelColliderOptions {
            dynamic_body,
            ..VoxelColliderOptions::default()
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_voxel_obb(
    obb: Obb,
    voxel_size: f64,
    options: VoxelColliderOptions,
) -> *mut ColliderBuilderHandle {
    let Some(grid) = build_obb_voxel_grid(obb, voxel_size) else {
        return std::ptr::null_mut();
    };
    builder_from_owned_grid(grid, options)
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_voxel_obb_auto(
    obb: Obb,
    voxel_size: f64,
    dynamic_body: crate::rapier::ffi::Bool,
) -> *mut ColliderBuilderHandle {
    collider_builder_create_voxel_obb(
        obb,
        voxel_size,
        VoxelColliderOptions {
            dynamic_body,
            ..VoxelColliderOptions::default()
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_voxel_aabb(
    world: *const WorldHandle,
    aabb: AabbDesc,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    crate::rapier::query::query_intersect_obb(
        world,
        Obb {
            center: Vec3 {
                x: (aabb.mins.x + aabb.maxs.x) * 0.5,
                y: (aabb.mins.y + aabb.maxs.y) * 0.5,
                z: (aabb.mins.z + aabb.maxs.z) * 0.5,
            },
            half_extents: Vec3 {
                x: (aabb.maxs.x - aabb.mins.x) * 0.5,
                y: (aabb.maxs.y - aabb.mins.y) * 0.5,
                z: (aabb.maxs.z - aabb.mins.z) * 0.5,
            },
            rotation: crate::rapier::ffi::Quat {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
        },
        filter,
        out_handles,
        capacity,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_voxel_aabb_count(
    world: *const WorldHandle,
    aabb: AabbDesc,
    filter: QueryFilterDesc,
) -> u32 {
    crate::rapier::query::query_intersect_obb_count(
        world,
        Obb {
            center: Vec3 {
                x: (aabb.mins.x + aabb.maxs.x) * 0.5,
                y: (aabb.mins.y + aabb.maxs.y) * 0.5,
                z: (aabb.mins.z + aabb.maxs.z) * 0.5,
            },
            half_extents: Vec3 {
                x: (aabb.maxs.x - aabb.mins.x) * 0.5,
                y: (aabb.maxs.y - aabb.mins.y) * 0.5,
                z: (aabb.maxs.z - aabb.mins.z) * 0.5,
            },
            rotation: crate::rapier::ffi::Quat {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
        },
        filter,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_voxel_obb(
    world: *const WorldHandle,
    obb: Obb,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    crate::rapier::query::query_intersect_obb(world, obb, filter, out_handles, capacity)
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_voxel_obb_count(
    world: *const WorldHandle,
    obb: Obb,
    filter: QueryFilterDesc,
) -> u32 {
    crate::rapier::query::query_intersect_obb_count(world, obb, filter)
}

#[unsafe(no_mangle)]
pub extern "C" fn world_insert_static_voxel_aabb(
    world: *mut WorldHandle,
    aabb: AabbDesc,
    voxel_size: f64,
    options: VoxelColliderOptions,
    friction: f64,
    restitution: f64,
) -> RigidBodyHandleRaw {
    let body = crate::rapier::rigid_body::rigid_body_builder_build(
        crate::rapier::rigid_body::rigid_body_builder_create(
            crate::rapier::ffi::BodyStatus::Fixed as u32,
        ),
    );
    if body.is_null() {
        return 0;
    }
    let body_handle = crate::rapier::rigid_body::world_insert_rigid_body(world, body);
    if body_handle == 0 {
        return 0;
    }
    let builder = collider_builder_create_voxel_aabb(aabb, voxel_size, options);
    if builder.is_null() {
        crate::rapier::rigid_body::world_remove_rigid_body(world, body_handle, Bool::TRUE);
        return 0;
    }
    crate::rapier::collider::collider_builder_set_friction(builder, friction);
    crate::rapier::collider::collider_builder_set_restitution(builder, restitution);
    let collider = crate::rapier::collider::collider_builder_build(builder);
    if collider.is_null() {
        crate::rapier::rigid_body::world_remove_rigid_body(world, body_handle, Bool::TRUE);
        return 0;
    }
    let collider_handle =
        crate::rapier::collider::world_insert_collider_with_parent(world, collider, body_handle);
    if collider_handle == 0 {
        crate::rapier::rigid_body::world_remove_rigid_body(world, body_handle, Bool::TRUE);
        return 0;
    }
    body_handle
}

#[unsafe(no_mangle)]
pub extern "C" fn world_insert_dynamic_voxel_obb(
    world: *mut WorldHandle,
    obb: Obb,
    voxel_size: f64,
    mut options: VoxelColliderOptions,
    density: f64,
    friction: f64,
    restitution: f64,
) -> RigidBodyHandleRaw {
    options.dynamic_body = Bool::TRUE;
    let body = crate::rapier::rigid_body::rigid_body_builder_build(
        crate::rapier::rigid_body::rigid_body_builder_create(
            crate::rapier::ffi::BodyStatus::Dynamic as u32,
        ),
    );
    if body.is_null() {
        return 0;
    }
    let body_handle = crate::rapier::rigid_body::world_insert_rigid_body(world, body);
    if body_handle == 0 {
        return 0;
    }
    let builder = collider_builder_create_voxel_obb(
        Obb {
            center: Vec3::default(),
            half_extents: obb.half_extents,
            rotation: crate::rapier::ffi::Quat {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
        },
        voxel_size,
        options,
    );
    if builder.is_null() {
        crate::rapier::rigid_body::world_remove_rigid_body(world, body_handle, Bool::TRUE);
        return 0;
    }
    crate::rapier::collider::collider_builder_set_density(builder, density);
    crate::rapier::collider::collider_builder_set_friction(builder, friction);
    crate::rapier::collider::collider_builder_set_restitution(builder, restitution);
    let collider = crate::rapier::collider::collider_builder_build(builder);
    if collider.is_null() {
        crate::rapier::rigid_body::world_remove_rigid_body(world, body_handle, Bool::TRUE);
        return 0;
    }
    let collider_handle =
        crate::rapier::collider::world_insert_collider_with_parent(world, collider, body_handle);
    if collider_handle == 0 {
        crate::rapier::rigid_body::world_remove_rigid_body(world, body_handle, Bool::TRUE);
        return 0;
    }
    crate::rapier::rigid_body::rigid_body_set_pose(
        world,
        body_handle,
        obb.center,
        obb.rotation,
        Bool::TRUE,
    );
    body_handle
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::{Bool, Quat};

    fn options(mode: VoxelColliderMode) -> VoxelColliderOptions {
        VoxelColliderOptions {
            mode: mode as u32,
            dynamic_body: Bool::FALSE,
            small_voxel_limit: 128,
            mesh_voxel_limit: 20_000,
        }
    }

    #[test]
    fn empty_voxels_build_no_collider() {
        let grid = VoxelGrid {
            voxels: &[0; 8],
            size_x: 2,
            size_y: 2,
            size_z: 2,
            voxel_size: 1.0,
            origin: Vec3::default(),
        };

        assert!(build_voxel_collider(&grid, options(VoxelColliderMode::Auto)).is_none());
    }

    #[test]
    fn solid_voxels_build_with_each_mode() {
        let voxels = [1; 8];
        let grid = VoxelGrid {
            voxels: &voxels,
            size_x: 2,
            size_y: 2,
            size_z: 2,
            voxel_size: 1.0,
            origin: Vec3::default(),
        };

        assert!(build_voxel_collider(&grid, options(VoxelColliderMode::Cuboids)).is_some());
        assert!(build_voxel_collider(&grid, options(VoxelColliderMode::GreedyCuboids)).is_some());
        assert!(build_voxel_collider(&grid, options(VoxelColliderMode::SurfaceMesh)).is_some());
    }

    #[test]
    fn voxel_aabb_and_obb_build() {
        let aabb = AabbDesc {
            mins: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            maxs: Vec3 {
                x: 2.0,
                y: 1.0,
                z: 1.0,
            },
        };
        let aabb_builder =
            collider_builder_create_voxel_aabb(aabb, 0.5, options(VoxelColliderMode::Auto));
        assert!(!aabb_builder.is_null());
        crate::rapier::collider::collider_builder_destroy(aabb_builder);

        let obb = Obb {
            center: Vec3::default(),
            half_extents: Vec3 {
                x: 1.0,
                y: 0.5,
                z: 0.5,
            },
            rotation: Quat {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
        };
        let obb_builder =
            collider_builder_create_voxel_obb(obb, 0.5, options(VoxelColliderMode::Auto));
        assert!(!obb_builder.is_null());
        crate::rapier::collider::collider_builder_destroy(obb_builder);
    }
}
