#![cfg(test)]

use mps_core::rapier::{
    ffi::{Bool, Vec3, WorldHandle, pack_rigid_body_handle},
    rigid_body::world_remove_rigid_body,
    world::*,
};
use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder, RigidBodyType, Vector};

struct World(*mut WorldHandle);
impl World {
    fn new() -> Self {
        Self(world_create(Vec3::default()))
    }
    fn count(&self, x: f64, radius: f64) -> u32 {
        world_get_region_body_count(self.0, Vec3 { x, y: 0.0, z: 0.0 }, radius)
    }
}
impl Drop for World {
    fn drop(&mut self) {
        world_destroy(self.0);
    }
}

#[test]
#[ignore = "Release spatial query benchmark; run alone"]
fn body_index_query_benchmark() {
    assert!(
        !std::hint::black_box(cfg!(debug_assertions)),
        "use --release"
    );
    for n in [100_000, 1_000_000] {
        let world = World::new();
        for i in 0..n {
            unsafe { &mut (*world.0).inner }.bodies.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(
                    (i % 1000) as f64,
                    (i / 1000) as f64,
                    0.0,
                )),
            );
        }
        let start = std::time::Instant::now();
        let expected = world.count(0.0, 10.0);
        let initial = start.elapsed();
        let start = std::time::Instant::now();
        for _ in 0..10 {
            assert_eq!(std::hint::black_box(world.count(0.0, 10.0)), expected);
        }
        let indexed = start.elapsed() / 10;
        let start = std::time::Instant::now();
        for _ in 0..10 {
            let count = unsafe { &(*world.0).inner }
                .bodies
                .iter()
                .filter(|(_, b)| {
                    b.is_dynamic() && std::hint::black_box(b.translation()).length() <= 10.0
                })
                .count();
            assert_eq!(count as u32, expected);
        }
        let scan = start.elapsed() / 10;
        eprintln!(
            "bodies={n}, first_query={initial:?}, indexed_mean={indexed:?}, scan_mean={scan:?}, hits={expected}"
        );
    }
}

#[test]
fn body_index_tracks_insert_move_remove_and_reused_generation() {
    let world = World::new();
    assert_eq!(world.count(0.0, 0.0), 0);
    let h = unsafe { &mut (*world.0).inner }.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(2.0, 0.0, 0.0))
            .build(),
    );
    assert_eq!(world.count(2.0, 0.0), 1);
    unsafe { &mut (*world.0).inner }.bodies[h].set_translation(Vector::new(5.0, 0.0, 0.0), true);
    assert_eq!(world.count(2.0, 0.0), 0);
    assert_eq!(world.count(5.0, 0.0), 1);
    assert_eq!(
        world_remove_rigid_body(world.0, pack_rigid_body_handle(h), Bool::TRUE),
        Bool::TRUE
    );
    let replacement = unsafe { &mut (*world.0).inner }.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(9.0, 0.0, 0.0))
            .build(),
    );
    assert_ne!(h, replacement);
    assert_eq!(world.count(5.0, 0.0), 0);
    assert_eq!(world.count(9.0, 0.0), 1);
    unsafe { &mut (*world.0).inner }
        .bodies
        .get_mut(replacement)
        .unwrap()
        .set_body_type(RigidBodyType::Fixed, true);
    assert_eq!(world.count(9.0, 0.0), 0);
}

#[test]
fn body_index_is_collider_independent_and_sphere_filtered() {
    let world = World::new();
    let w = unsafe { &mut (*world.0).inner };
    let plain = w
        .bodies
        .insert(RigidBodyBuilder::dynamic().additional_mass(1.0));
    let multiple = w
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(1.0, 0.0, 0.0)));
    for x in [-10.0, 10.0] {
        w.colliders.insert_with_parent(
            ColliderBuilder::ball(0.2).translation(Vector::new(x, 0.0, 0.0)),
            multiple,
            &mut w.bodies,
        );
    }
    let outside = w
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.9, 0.9, 0.0)));
    // Query before the first physics step: no collider BVH has been built yet.
    assert_eq!(world.count(0.0, 1.0), 2);
    assert_eq!(
        world_set_region_active(world.0, Vec3::default(), 1.0, Bool::FALSE),
        2
    );
    assert_eq!(
        world_set_region_active(world.0, Vec3::default(), 1.0, Bool::FALSE),
        0
    );
    let w = unsafe { &(*world.0).inner };
    assert!(w.bodies[plain].is_sleeping() && w.bodies[multiple].is_sleeping());
    assert!(!w.bodies[outside].is_sleeping());
    assert_eq!(world_wake_region(world.0, Vec3::default(), 1.0), 2);
    assert_eq!(world_wake_region(world.0, Vec3::default(), 1.0), 0);
}

#[test]
fn body_index_tracks_step_and_bulk_mutable_access() {
    let world = World::new();
    let h = unsafe { &mut (*world.0).inner }.bodies.insert(
        RigidBodyBuilder::dynamic()
            .additional_mass(1.0)
            .linvel(Vector::X),
    );
    assert_eq!(world.count(0.0, 0.0), 1);
    world_step(world.0, 0.1);
    assert_eq!(world.count(0.0, 0.01), 0);
    let x = unsafe { &(*world.0).inner }.bodies[h].translation().x;
    assert!(x > 0.0);
    assert_eq!(world.count(x, 1e-9), 1);
    for (_, b) in unsafe { &mut (*world.0).inner }.bodies.iter_mut() {
        b.set_translation(Vector::new(7.0, 0.0, 0.0), true);
    }
    assert_eq!(world.count(x, 1e-9), 0);
    assert_eq!(world.count(7.0, 0.0), 1);
    world_step(world.0, f64::NAN);
    assert_eq!(world.count(7.0, 0.0), 1);
}

#[test]
fn body_index_matches_scan_after_repeated_edits() {
    let world = World::new();
    let handles: Vec<_> = (0..256)
        .map(|i| {
            unsafe { &mut (*world.0).inner }
                .bodies
                .insert(RigidBodyBuilder::dynamic().translation(Vector::new(i as f64, 0.0, 0.0)))
        })
        .collect();
    assert_eq!(world.count(0.0, 20.0), 21);
    for round in 0..12 {
        for (i, &h) in handles.iter().enumerate().step_by(3) {
            unsafe { &mut (*world.0).inner }
                .bodies
                .get_mut(h)
                .unwrap()
                .set_translation(
                    Vector::new(((i * 17 + round * 11) % 300) as f64 - 30.0, 0.0, 0.0),
                    true,
                );
        }
        for center in [0.0, 50.0, 200.0] {
            let expected = unsafe { &(*world.0).inner }
                .bodies
                .iter()
                .filter(|(_, b)| (b.translation() - Vector::new(center, 0.0, 0.0)).length() <= 15.0)
                .count();
            assert_eq!(world.count(center, 15.0) as usize, expected);
        }
    }
}
