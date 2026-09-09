//! Body-center BVH, independent of collider presence and broad-phase freshness.
use rapier3d::geometry::Aabb;
use rapier3d::parry::bounding_volume::BoundingVolume;
use rapier3d::parry::partitioning::{Bvh, BvhBuildStrategy};
use rapier3d::prelude::{RigidBody, RigidBodyHandle, RigidBodySet, Vector};

pub(crate) struct BodySpatialIndex {
    bvh: Bvh,
    entries: Vec<Option<(RigidBodyHandle, Vector)>>,
    cursor: usize,
}

impl BodySpatialIndex {
    pub fn new(bodies: &RigidBodySet) -> Self {
        let mut entries = Vec::new();
        for (handle, body) in bodies.iter() {
            if body.is_dynamic() && body.translation().is_finite() {
                let index = handle.into_raw_parts().0 as usize;
                entries.resize(entries.len().max(index + 1), None);
                entries[index] = Some((handle, body.translation()));
            }
        }
        let bvh = Bvh::from_iter(
            BvhBuildStrategy::default(),
            entries
                .iter()
                .enumerate()
                .filter_map(|(i, entry)| entry.map(|(_, p)| (i, Aabb::new(p, p)))),
        );
        Self {
            bvh,
            entries,
            cursor: bodies.spatial_changes().len(),
        }
    }

    pub fn update(&mut self, handle: RigidBodyHandle, body: &RigidBody) {
        let index = handle.into_raw_parts().0 as usize;
        let value = (body.is_dynamic() && body.translation().is_finite())
            .then_some((handle, body.translation()));
        if index >= self.entries.len() {
            if value.is_none() {
                return;
            }
            self.entries.resize(index + 1, None);
        }
        if self.entries[index] == value {
            return;
        }
        if let Some((_, p)) = value {
            self.bvh.insert(Aabb::new(p, p), index as u32);
        } else if self.entries[index].is_some() {
            self.bvh.remove(index as u32);
        }
        self.entries[index] = value;
    }

    pub fn sync(&mut self, bodies: &RigidBodySet) {
        for &index in &bodies.spatial_changes()[self.cursor..] {
            // Resolve the latest generation, including remove/reinsert before query.
            if let Some((body, handle)) = bodies.get_unknown_gen(index) {
                self.update(handle, body);
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

    pub fn query(&self, center: Vector, radius: f64) -> Vec<RigidBodyHandle> {
        let ext = Vector::splat(radius);
        let bounds = Aabb::new(center - ext, center + ext);
        self.bvh
            .leaves(|node| node.aabb().intersects(&bounds))
            .filter_map(|index| self.entries[index as usize])
            .filter(|(_, p)| (*p - center).length() <= radius)
            .map(|(handle, _)| handle)
            .collect()
    }
}
