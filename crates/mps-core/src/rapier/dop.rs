use std::slice;

use rapier3d::prelude::{ColliderBuilder, Vector};
use smallvec::{SmallVec, smallvec};

use crate::rapier::ffi::{ColliderBuilderHandle, KdopPreset, kdop_preset_from_raw};

const EPSILON: f64 = 1.0e-9;
const MAX_RAW_POINTS: u32 = 1_000_000;
const MAX_RAW_DIRECTIONS: u32 = 4_096;

#[derive(Clone, Copy)]
struct Slab {
    normal: Vector,
    min: f64,
    max: f64,
}

trait DirectionHull {
    fn directions(&self) -> &[Vector];

    fn build(&self, points: &[Vector]) -> Option<ColliderBuilder> {
        build_direction_hull(points, self.directions())
    }
}

struct KdopHull {
    directions: SmallVec<[Vector; 13]>,
}

impl DirectionHull for KdopHull {
    fn directions(&self) -> &[Vector] {
        &self.directions
    }
}

struct FdhHull<'a> {
    directions: &'a [Vector],
}

impl DirectionHull for FdhHull<'_> {
    fn directions(&self) -> &[Vector] {
        self.directions
    }
}

fn normalize_direction(direction: Vector) -> Option<Vector> {
    let len = direction.length();
    (len > EPSILON).then_some(direction / len)
}

fn read_vectors(values: &[f64]) -> Option<SmallVec<[Vector; 32]>> {
    let mut vectors = SmallVec::with_capacity(values.len() / 3);
    for chunk in values.chunks_exact(3) {
        if !chunk[0].is_finite() || !chunk[1].is_finite() || !chunk[2].is_finite() {
            return None;
        }
        vectors.push(Vector::new(chunk[0], chunk[1], chunk[2]));
    }
    Some(vectors)
}

fn kdop_directions(preset: KdopPreset) -> SmallVec<[Vector; 13]> {
    let mut directions: SmallVec<[Vector; 13]> = smallvec![
        Vector::new(1.0, 0.0, 0.0),
        Vector::new(0.0, 1.0, 0.0),
        Vector::new(0.0, 0.0, 1.0),
    ];

    if matches!(preset, KdopPreset::K14 | KdopPreset::K18 | KdopPreset::K26) {
        directions.extend([
            Vector::new(1.0, 1.0, 1.0),
            Vector::new(1.0, 1.0, -1.0),
            Vector::new(1.0, -1.0, 1.0),
            Vector::new(-1.0, 1.0, 1.0),
        ]);
    }

    if matches!(preset, KdopPreset::K18 | KdopPreset::K26) {
        directions.extend([Vector::new(1.0, 1.0, 0.0), Vector::new(1.0, -1.0, 0.0)]);
    }

    if matches!(preset, KdopPreset::K26) {
        directions.extend([
            Vector::new(1.0, 0.0, 1.0),
            Vector::new(1.0, 0.0, -1.0),
            Vector::new(0.0, 1.0, 1.0),
            Vector::new(0.0, 1.0, -1.0),
        ]);
    }

    directions
        .into_iter()
        .filter_map(normalize_direction)
        .collect()
}

fn slabs_from_points(points: &[Vector], directions: &[Vector]) -> Option<SmallVec<[Slab; 16]>> {
    let mut slabs = SmallVec::<[Slab; 16]>::new();
    for direction in directions {
        let Some(normal) = normalize_direction(*direction) else {
            continue;
        };

        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        for point in points {
            let projection = normal.dot(*point);
            min = min.min(projection);
            max = max.max(projection);
        }

        if min.is_finite() && max.is_finite() {
            slabs.push(Slab { normal, min, max });
        }
    }

    (slabs.len() >= 3).then_some(slabs)
}

fn solve_planes(a: Vector, da: f64, b: Vector, db: f64, c: Vector, dc: f64) -> Option<Vector> {
    let cross_bc = b.cross(c);
    let det = a.dot(cross_bc);
    if det.abs() <= EPSILON {
        return None;
    }

    Some((cross_bc * da + c.cross(a) * db + a.cross(b) * dc) / det)
}

fn contains_point(slabs: &[Slab], point: Vector) -> bool {
    slabs.iter().all(|slab| {
        let projection = slab.normal.dot(point);
        projection >= slab.min - 1.0e-7 && projection <= slab.max + 1.0e-7
    })
}

fn push_unique(points: &mut SmallVec<[Vector; 32]>, point: Vector) {
    if points
        .iter()
        .any(|existing| (*existing - point).length_squared() <= 1.0e-12)
    {
        return;
    }

    points.push(point);
}

fn build_direction_hull(points: &[Vector], directions: &[Vector]) -> Option<ColliderBuilder> {
    if points.len() < 4 {
        return None;
    }

    let slabs = slabs_from_points(points, directions)?;
    let mut planes = SmallVec::<[(Vector, f64); 32]>::with_capacity(slabs.len() * 2);
    for slab in &slabs {
        planes.push((slab.normal, slab.max));
        planes.push((-slab.normal, -slab.min));
    }

    let mut vertices = SmallVec::<[Vector; 32]>::new();
    for i in 0..planes.len() {
        for j in (i + 1)..planes.len() {
            for k in (j + 1)..planes.len() {
                let Some(point) = solve_planes(
                    planes[i].0,
                    planes[i].1,
                    planes[j].0,
                    planes[j].1,
                    planes[k].0,
                    planes[k].1,
                ) else {
                    continue;
                };

                if contains_point(&slabs, point) {
                    push_unique(&mut vertices, point);
                }
            }
        }
    }

    ColliderBuilder::convex_hull(vertices.as_slice())
}

fn builder_from_raw_points(
    points_xyz: *const f64,
    point_count: u32,
) -> Option<SmallVec<[Vector; 32]>> {
    if points_xyz.is_null() || !(4..=MAX_RAW_POINTS).contains(&point_count) {
        return None;
    }
    let value_count = (point_count as usize).checked_mul(3)?;
    let values = unsafe { slice::from_raw_parts(points_xyz, value_count) };
    read_vectors(values)
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_kdop(
    points_xyz: *const f64,
    point_count: u32,
    preset: u32,
) -> *mut ColliderBuilderHandle {
    let Some(points) = builder_from_raw_points(points_xyz, point_count) else {
        return std::ptr::null_mut();
    };

    let hull = KdopHull {
        directions: kdop_directions(kdop_preset_from_raw(preset)),
    };
    let Some(builder) = hull.build(&points) else {
        return std::ptr::null_mut();
    };

    Box::into_raw(Box::new(ColliderBuilderHandle { inner: builder }))
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_fdh(
    points_xyz: *const f64,
    point_count: u32,
    directions_xyz: *const f64,
    direction_count: u32,
) -> *mut ColliderBuilderHandle {
    let Some(points) = builder_from_raw_points(points_xyz, point_count) else {
        return std::ptr::null_mut();
    };
    if directions_xyz.is_null() || !(3..=MAX_RAW_DIRECTIONS).contains(&direction_count) {
        return std::ptr::null_mut();
    }

    let Some(direction_value_count) = (direction_count as usize).checked_mul(3) else {
        return std::ptr::null_mut();
    };
    let direction_values = unsafe { slice::from_raw_parts(directions_xyz, direction_value_count) };
    let Some(directions) = read_vectors(direction_values) else {
        return std::ptr::null_mut();
    };
    let hull = FdhHull {
        directions: &directions,
    };
    let Some(builder) = hull.build(&points) else {
        return std::ptr::null_mut();
    };

    Box::into_raw(Box::new(ColliderBuilderHandle { inner: builder }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube_points() -> SmallVec<[Vector; 8]> {
        let mut points = SmallVec::new();
        for x in [-1.0, 1.0] {
            for y in [-1.0, 1.0] {
                for z in [-1.0, 1.0] {
                    points.push(Vector::new(x, y, z));
                }
            }
        }
        points
    }

    #[test]
    fn kdop_builds_from_cube_points() {
        let hull = KdopHull {
            directions: kdop_directions(KdopPreset::K14),
        };
        assert!(hull.build(&cube_points()).is_some());
    }

    #[test]
    fn fdh_builds_from_custom_directions() {
        let directions = kdop_directions(KdopPreset::K6);
        let hull = FdhHull {
            directions: &directions,
        };
        assert!(hull.build(&cube_points()).is_some());
    }
}
