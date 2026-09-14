//! Body-center BVH, independent of collider presence and broad-phase freshness.
use rapier3d::geometry::Aabb;
use rapier3d::parry::bounding_volume::BoundingVolume;
use rapier3d::parry::partitioning::{Bvh, BvhBuildStrategy};
use rapier3d::prelude::{ColliderSet, RigidBody, RigidBodyHandle, RigidBodySet, Vector};

pub(crate) struct BodySpatialIndex {
    bvh: Bvh,
    entries: Vec<Option<(RigidBodyHandle, Vector, Vector, f64)>>,
    cursor: usize,
}

impl BodySpatialIndex {
    pub fn new(bodies: &RigidBodySet, colliders: &ColliderSet) -> Self {
        let mut entries = Vec::new();
        for (handle, body) in bodies.iter() {
            if body.is_dynamic() && body.translation().is_finite() {
                let index = handle.into_raw_parts().0 as usize;
                entries.resize(entries.len().max(index + 1), None);
                let shape_radius = body
                    .colliders()
                    .iter()
                    .filter_map(|h| colliders.get(*h))
                    .map(|c| c.compute_aabb().half_extents().length())
                    .fold(0.0, f64::max);
                entries[index] = Some((
                    handle,
                    body.translation(),
                    body.linvel(),
                    body.angvel().length() + shape_radius,
                ));
            }
        }
        let bvh = Bvh::from_iter(
            BvhBuildStrategy::default(),
            entries.iter().enumerate().filter_map(|(i, entry)| {
                entry.map(|(_, p, v, w)| {
                    let next = p + v * (1.0 / 60.0);
                    let pad = w * (1.0 / 60.0);
                    (
                        i,
                        Aabb::new(
                            p.min(next) - Vector::splat(pad),
                            p.max(next) + Vector::splat(pad),
                        ),
                    )
                })
            }),
        );
        Self {
            bvh,
            entries,
            cursor: bodies.spatial_changes().len(),
        }
    }

    pub fn update(&mut self, handle: RigidBodyHandle, body: &RigidBody, dt: f64) {
        let index = handle.into_raw_parts().0 as usize;
        let value = (body.is_dynamic() && body.translation().is_finite()).then_some((
            handle,
            body.translation(),
            body.linvel(),
            body.angvel().length(),
        ));
        if index >= self.entries.len() {
            if value.is_none() {
                return;
            }
            self.entries.resize(index + 1, None);
        }
        if self.entries[index] == value {
            return;
        }
        if let Some((_, p, v, w)) = value {
            let next = p + v * dt;
            let pad = w * dt;
            self.bvh.insert(
                Aabb::new(
                    p.min(next) - Vector::splat(pad),
                    p.max(next) + Vector::splat(pad),
                ),
                index as u32,
            );
        } else if self.entries[index].is_some() {
            self.bvh.remove(index as u32);
        }
        self.entries[index] = value;
    }

    #[allow(dead_code)]
    pub fn update_snapshot(
        &mut self,
        handle: RigidBodyHandle,
        body: &RigidBody,
        colliders: &ColliderSet,
        dt: f64,
    ) {
        let shape_radius = body
            .colliders()
            .iter()
            .filter_map(|h| colliders.get(*h))
            .map(|c| c.compute_aabb().half_extents().length())
            .fold(0.0, f64::max);
        self.update_values(
            handle,
            body.translation(),
            body.linvel(),
            body.angvel().length() + shape_radius,
            dt,
            handle.into_raw_parts().0 as usize,
        );
    }

    fn update_values(
        &mut self,
        handle: RigidBodyHandle,
        p: Vector,
        v: Vector,
        w: f64,
        dt: f64,
        index: usize,
    ) {
        if index >= self.entries.len() {
            self.entries.resize(index + 1, None);
        }
        let value = Some((handle, p, v, w));
        if self.entries[index] != value {
            let next = p + v * dt;
            let pad = w * dt;
            self.bvh.insert(
                Aabb::new(
                    p.min(next) - Vector::splat(pad),
                    p.max(next) + Vector::splat(pad),
                ),
                index as u32,
            );
            self.entries[index] = value;
        }
    }

    pub fn sync(&mut self, bodies: &RigidBodySet, dt: f64) {
        for &index in &bodies.spatial_changes()[self.cursor..] {
            // Resolve the latest generation, including remove/reinsert before query.
            if let Some((body, handle)) = bodies.get_unknown_gen(index) {
                self.update(handle, body, dt);
            } else if let Some(entry) = self.entries.get_mut(index as usize)
                && entry.take().is_some()
            {
                self.bvh.remove(index);
            }
        }
        self.cursor = bodies.spatial_changes().len();
    }

    pub fn journal_cleared(&mut self) {
        self.cursor = 0;
    }

    pub fn query(&self, center: Vector, radius: f64, dt: f64) -> Vec<RigidBodyHandle> {
        let ext = Vector::splat(radius);
        let bounds = Aabb::new(center - ext, center + ext);
        self.bvh
            .leaves(|node| node.aabb().intersects(&bounds))
            .filter_map(|index| self.entries[index as usize])
            .filter(|(_, p, v, w)| {
                segment_distance_squared(*p, *p + *v * dt, center) <= (radius + *w * dt).powi(2)
            })
            .map(|(handle, _, _, _)| handle)
            .collect()
    }
}

fn segment_distance_squared(start: Vector, end: Vector, point: Vector) -> f64 {
    let delta = end - start;
    let denom = delta.length_squared();
    if denom <= f64::EPSILON {
        return (point - start).length_squared();
    }
    let t = ((point - start).dot(delta) / denom).clamp(0.0, 1.0);
    (point - (start + delta * t)).length_squared()
}
