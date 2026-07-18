use rapier3d::control::{
    CharacterAutostep, CharacterCollision as RapierCharacterCollision, CharacterLength,
    KinematicCharacterController,
};

use crate::rapier::ffi::{
    Bool, CharacterCollision as FfiCharacterCollision, CharacterControllerHandle,
    EffectiveCharacterMovement, Quat, ShapeDesc, Vec3, WorldHandle, pack_collider_handle,
    quat_finite, shape_desc_valid, shape_from_desc, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

#[derive(Default)]
pub(crate) struct CharacterControllerState {
    pub(crate) controller: KinematicCharacterController,
    pub(crate) collisions: Vec<RapierCharacterCollision>,
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_create() -> *mut CharacterControllerHandle {
    Box::into_raw(Box::new(CharacterControllerHandle {
        inner: CharacterControllerState::default(),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_destroy(controller: *mut CharacterControllerHandle) {
    if controller.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(controller));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_up(
    controller: *mut CharacterControllerHandle,
    up: Vec3,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    if !vec3_finite(up) {
        return;
    }
    controller.inner.controller.up = vec3_to_rapier(up);
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_offset_absolute(
    controller: *mut CharacterControllerHandle,
    offset: f64,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    if !offset.is_finite() || offset < 0.0 {
        return;
    }
    controller.inner.controller.offset = CharacterLength::Absolute(offset);
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_offset_relative(
    controller: *mut CharacterControllerHandle,
    offset: f64,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    if !offset.is_finite() || offset < 0.0 {
        return;
    }
    controller.inner.controller.offset = CharacterLength::Relative(offset);
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_slide(
    controller: *mut CharacterControllerHandle,
    slide: Bool,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    controller.inner.controller.slide = slide.0 != 0;
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_autostep(
    controller: *mut CharacterControllerHandle,
    enabled: Bool,
    max_height: f64,
    min_width: f64,
    include_dynamic_bodies: Bool,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    if enabled.0 != 0
        && (!max_height.is_finite()
            || !min_width.is_finite()
            || max_height < 0.0
            || min_width < 0.0)
    {
        return;
    }
    controller.inner.controller.autostep = if enabled.0 != 0 {
        Some(CharacterAutostep {
            max_height: CharacterLength::Absolute(max_height),
            min_width: CharacterLength::Absolute(min_width),
            include_dynamic_bodies: include_dynamic_bodies.0 != 0,
        })
    } else {
        None
    };
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_snap_to_ground(
    controller: *mut CharacterControllerHandle,
    enabled: Bool,
    distance: f64,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    if enabled.0 != 0 && (!distance.is_finite() || distance < 0.0) {
        return;
    }
    controller.inner.controller.snap_to_ground = if enabled.0 != 0 {
        Some(CharacterLength::Absolute(distance))
    } else {
        None
    };
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_slope_angles(
    controller: *mut CharacterControllerHandle,
    max_climb_angle: f64,
    min_slide_angle: f64,
) {
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return;
    };
    if !max_climb_angle.is_finite() || !min_slide_angle.is_finite() {
        return;
    }
    controller.inner.controller.max_slope_climb_angle = max_climb_angle;
    controller.inner.controller.min_slope_slide_angle = min_slide_angle;
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_move_shape(
    world: *const WorldHandle,
    controller: *mut CharacterControllerHandle,
    dt: f64,
    shape_desc: ShapeDesc,
    translation: Vec3,
    rotation: Quat,
    desired_translation: Vec3,
) -> EffectiveCharacterMovement {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return EffectiveCharacterMovement::default();
    };
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return EffectiveCharacterMovement::default();
    };
    if !dt.is_finite()
        || dt <= 0.0
        || !shape_desc_valid(shape_desc)
        || !vec3_finite(translation)
        || !quat_finite(rotation)
        || !vec3_finite(desired_translation)
    {
        return EffectiveCharacterMovement::default();
    }

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        rapier3d::prelude::QueryFilter::default(),
    );
    let shape = shape_from_desc(shape_desc);
    controller.inner.collisions.clear();
    let movement = controller.inner.controller.move_shape(
        dt,
        &query,
        shape.as_ref(),
        &crate::rapier::ffi::isometry_from_parts(translation, rotation),
        vec3_to_rapier(desired_translation),
        |collision| controller.inner.collisions.push(collision),
    );

    EffectiveCharacterMovement {
        translation: vec3_from_rapier(movement.translation),
        grounded: movement.grounded.into(),
        is_sliding_down_slope: movement.is_sliding_down_slope.into(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_collision_count(
    controller: *const CharacterControllerHandle,
) -> u32 {
    let Some(controller) = (unsafe { controller.as_ref() }) else {
        return 0;
    };

    controller.inner.collisions.len() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_get_collision(
    controller: *const CharacterControllerHandle,
    index: u32,
) -> FfiCharacterCollision {
    let Some(controller) = (unsafe { controller.as_ref() }) else {
        return FfiCharacterCollision::default();
    };
    let Some(collision) = controller.inner.collisions.get(index as usize) else {
        return FfiCharacterCollision::default();
    };

    FfiCharacterCollision {
        collider: pack_collider_handle(collision.handle),
        character_translation: vec3_from_rapier(collision.character_pos.translation),
        translation_applied: vec3_from_rapier(collision.translation_applied),
        translation_remaining: vec3_from_rapier(collision.translation_remaining),
        world_witness1: vec3_from_rapier(collision.hit.witness1),
        world_witness2: vec3_from_rapier(collision.hit.witness2),
        normal1: vec3_from_rapier(collision.hit.normal1),
        normal2: vec3_from_rapier(collision.hit.normal2),
        time_of_impact: collision.hit.time_of_impact,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn character_controller_solve_impulses(
    world: *mut WorldHandle,
    controller: *mut CharacterControllerHandle,
    dt: f64,
    shape_desc: ShapeDesc,
    character_mass: f64,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(controller) = (unsafe { controller.as_mut() }) else {
        return Bool::FALSE;
    };
    if !dt.is_finite()
        || dt <= 0.0
        || !character_mass.is_finite()
        || character_mass < 0.0
        || !shape_desc_valid(shape_desc)
    {
        return Bool::FALSE;
    }

    let shape = shape_from_desc(shape_desc);
    let query = world.inner.broad_phase.as_query_pipeline_mut(
        world.inner.narrow_phase.query_dispatcher(),
        &mut world.inner.bodies,
        &mut world.inner.colliders,
        rapier3d::prelude::QueryFilter::default(),
    );

    controller
        .inner
        .controller
        .solve_character_collision_impulses(
            dt,
            &mut { query },
            shape.as_ref(),
            character_mass,
            controller.inner.collisions.iter(),
        );
    Bool::TRUE
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::{BodyStatus, RigidBodyHandleRaw, ShapeDesc, ShapeType};
    use crate::rapier::rigid_body::{
        rigid_body_builder_build, rigid_body_builder_create, world_insert_rigid_body,
    };
    use crate::rapier::world::{world_create, world_destroy};

    fn make_world() -> *mut WorldHandle {
        world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        })
    }

    fn make_dynamic_body(world: *mut WorldHandle) -> RigidBodyHandleRaw {
        let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        let body = rigid_body_builder_build(builder);
        let handle = world_insert_rigid_body(world, body);
        assert_ne!(handle, 0);
        handle
    }

    fn cuboid_shape(hx: f64, hy: f64, hz: f64) -> ShapeDesc {
        ShapeDesc {
            shape_type: ShapeType::Cuboid as u32,
            a: hx,
            b: hy,
            c: hz,
            d: 0.0,
        }
    }

    // ---- create / destroy ----

    #[test]
    fn create_and_destroy() {
        let c = character_controller_create();
        assert!(!c.is_null());
        character_controller_destroy(c);
    }

    #[test]
    fn destroy_null_is_noop() {
        character_controller_destroy(std::ptr::null_mut());
    }

    // ---- set_up ----

    #[test]
    fn set_up_rejects_null() {
        character_controller_set_up(std::ptr::null_mut(), Vec3 { x: 0.0, y: 1.0, z: 0.0 });
    }

    #[test]
    fn set_up_rejects_nan() {
        let c = character_controller_create();
        character_controller_set_up(c, Vec3 { x: f64::NAN, y: 0.0, z: 0.0 });
        character_controller_destroy(c);
    }

    // ---- set_offset ----

    #[test]
    fn set_offset_absolute() {
        let c = character_controller_create();
        character_controller_set_offset_absolute(c, 0.5);
        character_controller_destroy(c);
    }

    #[test]
    fn set_offset_absolute_rejects_negative() {
        let c = character_controller_create();
        character_controller_set_offset_absolute(c, -0.1);
        character_controller_destroy(c);
    }

    #[test]
    fn set_offset_relative() {
        let c = character_controller_create();
        character_controller_set_offset_relative(c, 0.2);
        character_controller_destroy(c);
    }

    #[test]
    fn set_offset_relative_rejects_negative() {
        let c = character_controller_create();
        character_controller_set_offset_relative(c, -0.1);
        character_controller_destroy(c);
    }

    // ---- set_slide ----

    #[test]
    fn set_slide() {
        let c = character_controller_create();
        character_controller_set_slide(c, Bool::TRUE);
        character_controller_set_slide(c, Bool::FALSE);
        character_controller_destroy(c);
    }

    // ---- set_autostep ----

    #[test]
    fn set_autostep_enabled() {
        let c = character_controller_create();
        character_controller_set_autostep(c, Bool::TRUE, 0.5, 0.2, Bool::FALSE);
        character_controller_destroy(c);
    }

    #[test]
    fn set_autostep_disabled() {
        let c = character_controller_create();
        character_controller_set_autostep(c, Bool::FALSE, 0.5, 0.2, Bool::FALSE);
        character_controller_destroy(c);
    }

    #[test]
    fn set_autostep_rejects_negative_height() {
        let c = character_controller_create();
        character_controller_set_autostep(c, Bool::TRUE, -0.5, 0.2, Bool::FALSE);
        character_controller_destroy(c);
    }

    // ---- set_snap_to_ground ----

    #[test]
    fn set_snap_to_ground() {
        let c = character_controller_create();
        character_controller_set_snap_to_ground(c, Bool::TRUE, 0.3);
        character_controller_set_snap_to_ground(c, Bool::FALSE, 0.0);
        character_controller_destroy(c);
    }

    #[test]
    fn set_snap_to_ground_rejects_negative() {
        let c = character_controller_create();
        character_controller_set_snap_to_ground(c, Bool::TRUE, -0.3);
        character_controller_destroy(c);
    }

    // ---- set_slope_angles ----

    #[test]
    fn set_slope_angles() {
        let c = character_controller_create();
        character_controller_set_slope_angles(c, 0.8, 0.3);
        character_controller_destroy(c);
    }

    #[test]
    fn set_slope_angles_rejects_nan() {
        let c = character_controller_create();
        character_controller_set_slope_angles(c, f64::NAN, 0.3);
        character_controller_destroy(c);
    }

    // ---- move_shape ----

    #[test]
    fn move_shape_rejects_null_world() {
        let c = character_controller_create();
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        let result = character_controller_move_shape(
            std::ptr::null(),
            c,
            1.0 / 60.0,
            shape,
            Vec3 { x: 0.0, y: 1.0, z: 0.0 },
            Quat { i: 0.0, j: 0.0, k: 0.0, w: 1.0 },
            Vec3 { x: 0.0, y: 0.0, z: 1.0 },
        );
        assert_eq!(result.translation.x, 0.0);
        character_controller_destroy(c);
    }

    #[test]
    fn move_shape_rejects_null_controller() {
        let world = make_world();
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        let result = character_controller_move_shape(
            world,
            std::ptr::null_mut(),
            1.0 / 60.0,
            shape,
            Vec3 { x: 0.0, y: 1.0, z: 0.0 },
            Quat { i: 0.0, j: 0.0, k: 0.0, w: 1.0 },
            Vec3 { x: 0.0, y: 0.0, z: 1.0 },
        );
        assert_eq!(result.translation.x, 0.0);
        world_destroy(world);
    }

    #[test]
    fn move_shape_with_no_colliders() {
        let world = make_world();
        make_dynamic_body(world); // ensure world is non-empty but no colliders
        let c = character_controller_create();
        character_controller_set_up(c, Vec3 { x: 0.0, y: 1.0, z: 0.0 });
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        let result = character_controller_move_shape(
            world,
            c,
            1.0 / 60.0,
            shape,
            Vec3 { x: 0.0, y: 1.0, z: 0.0 },
            Quat { i: 0.0, j: 0.0, k: 0.0, w: 1.0 },
            Vec3 { x: 1.0, y: 0.0, z: 0.0 },
        );
        // translation applied should be close to desired (no obstacles)
        assert!((result.translation.x - 1.0).abs() < 0.1);
        character_controller_destroy(c);
        world_destroy(world);
    }

    #[test]
    fn move_shape_rejects_invalid_dt() {
        let world = make_world();
        let c = character_controller_create();
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        let result = character_controller_move_shape(
            world,
            c,
            0.0, // invalid dt
            shape,
            Vec3::default(),
            Quat { i: 0.0, j: 0.0, k: 0.0, w: 1.0 },
            Vec3 { x: 1.0, y: 0.0, z: 0.0 },
        );
        assert_eq!(result.translation.x, 0.0);
        character_controller_destroy(c);
        world_destroy(world);
    }

    // ---- collision_count ----

    #[test]
    fn collision_count_rejects_null() {
        assert_eq!(character_controller_collision_count(std::ptr::null()), 0);
    }

    // ---- get_collision ----

    #[test]
    fn get_collision_rejects_null() {
        let col = character_controller_get_collision(std::ptr::null(), 0);
        assert_eq!(col.collider, 0);
    }

    // ---- solve_impulses ----

    #[test]
    fn solve_impulses_rejects_null_world() {
        let c = character_controller_create();
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        assert_eq!(
            character_controller_solve_impulses(std::ptr::null_mut(), c, 1.0 / 60.0, shape, 70.0),
            Bool::FALSE
        );
        character_controller_destroy(c);
    }

    #[test]
    fn solve_impulses_rejects_invalid_dt() {
        let world = make_world();
        let c = character_controller_create();
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        assert_eq!(
            character_controller_solve_impulses(world, c, 0.0, shape, 70.0),
            Bool::FALSE
        );
        character_controller_destroy(c);
        world_destroy(world);
    }

    #[test]
    fn solve_impulses_rejects_negative_mass() {
        let world = make_world();
        let c = character_controller_create();
        let shape = cuboid_shape(0.5, 1.0, 0.5);
        assert_eq!(
            character_controller_solve_impulses(world, c, 1.0 / 60.0, shape, -1.0),
            Bool::FALSE
        );
        character_controller_destroy(c);
        world_destroy(world);
    }
}
