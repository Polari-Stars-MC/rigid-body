#![cfg(test)]

use mps_core::rapier::error::last_error_code as error_code;
use mps_core::rapier::{collider::*, collision_mode::*, error::*, ffi::*, world::*};
use rapier3d::prelude::RigidBodyBuilder;

struct Fixture {
    world: *mut WorldHandle,
    simple: *mut ColliderBuilderHandle,
    compound: *mut ColliderBuilderHandle,
    body: u64,
}
impl Fixture {
    fn new(mode: WorldCollisionMode) -> Self {
        let world = world_create_with_collision_mode(Vec3::default(), mode as u32);
        assert!(!world.is_null());
        let body = unsafe {
            (*world)
                .inner
                .bodies
                .insert(RigidBodyBuilder::dynamic().additional_mass(1.0).build())
        };
        let simple = collider_builder_create(
            0,
            Vec3 {
                x: 0.5,
                y: 0.0,
                z: 0.0,
            },
        );
        let boxes = [
            -0.5, -0.5, -0.5, 0.0, 0.5, 0.5, 0.0, -0.5, -0.5, 0.5, 0.5, 0.5,
        ];
        let compound = collider_builder_create_compound_boxes(boxes.as_ptr(), 2);
        assert!(!simple.is_null() && !compound.is_null());
        Self {
            world,
            simple,
            compound,
            body: pack_rigid_body_handle(body),
        }
    }
    fn insert(&self) -> u64 {
        world_insert_default_collider(self.world, self.body, self.simple, self.compound)
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        collider_builder_destroy(self.simple);
        collider_builder_destroy(self.compound);
        world_destroy(self.world);
    }
}

#[test]
fn collision_mode_switch_preserves_existing_shapes_and_builders() {
    let f = Fixture::new(WorldCollisionMode::Simple);
    let first = f.insert();
    assert_ne!(first, 0);
    assert_ne!(f.insert(), 0);
    assert_eq!(world_set_default_collision_mode(f.world, 2), Bool::TRUE);
    let compound = f.insert();
    assert_ne!(compound, 0);
    assert_eq!(world_set_default_collision_mode(f.world, 0), Bool::TRUE);
    assert_eq!(
        world_insert_default_collider(f.world, f.body, std::ptr::null(), std::ptr::null()),
        0
    );
    assert_eq!(error_code(), ERR_OK);
    let w = unsafe { &(*f.world).inner };
    assert_eq!(w.colliders.len(), 3);
    assert!(
        w.colliders[unpack_collider_handle(first)]
            .shape()
            .as_ball()
            .is_some()
    );
    assert_eq!(
        w.colliders[unpack_collider_handle(compound)]
            .shape()
            .as_compound()
            .unwrap()
            .shapes()
            .len(),
        2
    );
    world_step(f.world, 0.01);
    assert_eq!(world_get_default_collision_mode(f.world), 0);
}

#[test]
fn adaptive_mode_prefers_compound_and_falls_back_to_simple() {
    let f = Fixture::new(WorldCollisionMode::Adaptive);
    assert!(f.insert() != 0);
    assert_eq!(unsafe { (*f.world).inner.colliders.len() }, 1);
    let simple_only = world_insert_default_collider(f.world, f.body, f.simple, std::ptr::null());
    assert_ne!(simple_only, 0);
    assert_eq!(unsafe { (*f.world).inner.colliders.len() }, 2);
}

#[test]
fn collision_mode_invalid_inputs_leave_world_unchanged() {
    let f = Fixture::new(WorldCollisionMode::Simple);
    assert_eq!(
        world_set_default_collision_mode(f.world, u32::MAX),
        Bool::FALSE
    );
    assert_eq!(error_code(), ERR_INVALID_ARGUMENT);
    assert_eq!(world_get_default_collision_mode(f.world), 1);
    assert_eq!(
        world_insert_default_collider(f.world, f.body, f.compound, f.compound),
        0
    );
    assert_eq!(error_code(), ERR_INVALID_ARGUMENT);
    assert_eq!(
        world_insert_default_collider(f.world, f.body, std::ptr::null(), f.compound),
        0
    );
    assert_eq!(error_code(), ERR_NULL_POINTER);
    assert_eq!(
        world_insert_default_collider(f.world, 0, f.simple, f.compound),
        0
    );
    assert_eq!(error_code(), ERR_NOT_FOUND);
    assert_eq!(world_set_default_collision_mode(f.world, 2), Bool::TRUE);
    assert_eq!(
        world_insert_default_collider(f.world, f.body, f.simple, f.simple),
        0
    );
    assert_eq!(error_code(), ERR_INVALID_ARGUMENT);
    assert_eq!(unsafe { (*f.world).inner.colliders.len() }, 0);
    assert!(world_create_with_collision_mode(Vec3::default(), 4).is_null());
    assert_eq!(error_code(), ERR_INVALID_ARGUMENT);
    assert_eq!(world_get_default_collision_mode(std::ptr::null()), u32::MAX);
    assert_eq!(error_code(), ERR_NULL_POINTER);
}

#[test]
fn collision_mode_legacy_world_and_explicit_insertion_remain_compatible() {
    let world = world_create(Vec3::default());
    assert_eq!(world_get_default_collision_mode(world), 1);
    world_destroy(world);
    let mut f = Fixture::new(WorldCollisionMode::None);
    let collider = collider_builder_build(f.simple);
    f.simple = std::ptr::null_mut();
    assert_ne!(
        world_insert_collider_with_parent(f.world, collider, f.body),
        0
    );
    assert_eq!(unsafe { (*f.world).inner.colliders.len() }, 1);
}
