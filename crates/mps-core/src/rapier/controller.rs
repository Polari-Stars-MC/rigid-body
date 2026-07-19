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


