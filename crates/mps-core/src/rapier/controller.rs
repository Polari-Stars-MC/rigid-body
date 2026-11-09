use rapier3d::control::{
    CharacterAutostep, CharacterCollision as RapierCharacterCollision, CharacterLength,
    KinematicCharacterController,
};

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    Bool, CharacterCollision, CharacterControllerHandle, EffectiveCharacterMovement, Quat,
    ShapeDesc, Vec3, WorldHandle, pack_collider_handle, quat_finite, shape_desc_valid,
    shape_from_desc, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};
use crate::rapier::terrain_gravity::terrain_gravity_acceleration;

#[derive(Default)]
pub(crate) struct CharacterControllerState {
    pub(crate) controller: KinematicCharacterController,
    pub(crate) collisions: Vec<RapierCharacterCollision>,
}

/// Creates a new character controller and returns an opaque handle to it.
///
/// # Safety
///
/// The returned pointer is owned by Rust and must be passed to
/// `character_controller_destroy` exactly once. Returns null on internal
/// failure (see `last_error_code`).
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_create() -> *mut CharacterControllerHandle {
    ffi_guard(std::ptr::null_mut(), || {
        Box::into_raw(Box::new(CharacterControllerHandle {
            inner: CharacterControllerState::default(),
        }))
    })
}

/// # Safety
///
/// `controller` must be a pointer returned by `character_controller_create` (or null,
/// which is a no-op). Ownership is transferred to Rust and the pointer must not be
/// used after this call.
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_destroy(controller: *mut CharacterControllerHandle) {
    ffi_guard((), || {
        if controller.is_null() {
            return;
        }

        unsafe {
            drop(Box::from_raw(controller));
        }
    })
}

/// # Safety
///
/// `controller` must be a valid pointer returned by `character_controller_create`
/// and must remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_up(
    controller: *mut CharacterControllerHandle,
    up: Vec3,
) {
    ffi_guard((), || {
        let Some(controller) = (unsafe { controller.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "character controller is null");
            return;
        };
        if !vec3_finite(up) {
            set_error(ERR_INVALID_ARGUMENT, "up vector must be finite");
            return;
        }
        controller.inner.controller.up = vec3_to_rapier(up);
    })
}

/// # Safety
///
/// `controller` must be a valid pointer returned by `character_controller_create`
/// and must remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_offset_absolute(
    controller: *mut CharacterControllerHandle,
    offset: f64,
) {
    ffi_guard((), || {
        let Some(controller) = (unsafe { controller.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "character controller is null");
            return;
        };
        if !offset.is_finite() || offset < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "offset must be finite and non-negative",
            );
            return;
        }
        controller.inner.controller.offset = CharacterLength::Absolute(offset);
    })
}

/// # Safety
///
/// `controller` must be a valid pointer returned by `character_controller_create`
/// and must remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_offset_relative(
    controller: *mut CharacterControllerHandle,
    offset: f64,
) {
    ffi_guard((), || {
        let Some(controller) = (unsafe { controller.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "character controller is null");
            return;
        };
        if !offset.is_finite() || offset < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "offset must be finite and non-negative",
            );
            return;
        }
        controller.inner.controller.offset = CharacterLength::Relative(offset);
    })
}

/// # Safety
///
/// `controller` must be a valid pointer returned by `character_controller_create`
/// and must remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_slide(
    controller: *mut CharacterControllerHandle,
    slide: Bool,
) {
    ffi_guard((), || {
        let Some(controller) = (unsafe { controller.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "character controller is null");
            return;
        };
        controller.inner.controller.slide = slide.0 != 0;
    })
}

/// # Safety
///
/// `controller` must be a valid pointer returned by `character_controller_create`
/// and must remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_autostep(
    controller: *mut CharacterControllerHandle,
    enabled: Bool,
    max_height: f64,
    min_width: f64,
    include_dynamic_bodies: Bool,
) {
    ffi_guard((), || {
        let Some(controller) = (unsafe { controller.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "character controller is null");
            return;
        };
        if enabled.0 != 0
            && (!max_height.is_finite()
                || !min_width.is_finite()
                || max_height < 0.0
                || min_width < 0.0)
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "autostep dimensions must be finite and non-negative",
            );
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
    })
}

/// # Safety
///
/// `controller` must be a valid pointer returned by `character_controller_create`
/// and must remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_snap_to_ground(
    controller: *mut CharacterControllerHandle,
    enabled: Bool,
    distance: f64,
) {
    ffi_guard((), || {
        let Some(controller) = (unsafe { controller.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "character controller is null");
            return;
        };
        if enabled.0 != 0 && (!distance.is_finite() || distance < 0.0) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "snap-to-ground distance must be finite and non-negative",
            );
            return;
        }
        controller.inner.controller.snap_to_ground = if enabled.0 != 0 {
            Some(CharacterLength::Absolute(distance))
        } else {
            None
        };
    })
}

/// # Safety
///
/// `controller` must be a valid pointer returned by `character_controller_create`
/// and must remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_set_slope_angles(
    controller: *mut CharacterControllerHandle,
    max_climb_angle: f64,
    min_slide_angle: f64,
) {
    ffi_guard((), || {
        let Some(controller) = (unsafe { controller.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "character controller is null");
            return;
        };
        if !max_climb_angle.is_finite() || !min_slide_angle.is_finite() {
            set_error(ERR_INVALID_ARGUMENT, "slope angles must be finite");
            return;
        }
        controller.inner.controller.max_slope_climb_angle = max_climb_angle;
        controller.inner.controller.min_slope_slide_angle = min_slide_angle;
    })
}

/// # Safety
///
/// `world` must be a valid world pointer and `controller` a valid pointer returned by
/// `character_controller_create`; both must remain alive for the duration of the call.
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
    ffi_guard(EffectiveCharacterMovement::default(), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return EffectiveCharacterMovement::default();
        };
        // move_shape 浼氭敼涓栫晫鍐呭垰浣撶姸鎬侊紝闇€瑕佸啓閿佷笌鏌ヨ浜掓枼
        let _query_lock = world.inner.query_lock.write();
        let Some(controller) = (unsafe { controller.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "character controller is null");
            return EffectiveCharacterMovement::default();
        };
        if !dt.is_finite()
            || dt <= 0.0
            || !shape_desc_valid(shape_desc)
            || !vec3_finite(translation)
            || !quat_finite(rotation)
            || !vec3_finite(desired_translation)
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid move_shape arguments");
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
    })
}

/// # Safety
///
/// `controller` must be a valid pointer returned by `character_controller_create`
/// and must remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_collision_count(
    controller: *const CharacterControllerHandle,
) -> u32 {
    ffi_guard(0, || {
        let Some(controller) = (unsafe { controller.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "character controller is null");
            return 0;
        };

        controller.inner.collisions.len() as u32
    })
}

/// # Safety
///
/// `controller` must be a valid pointer returned by `character_controller_create`
/// and must remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_get_collision(
    controller: *const CharacterControllerHandle,
    index: u32,
) -> CharacterCollision {
    ffi_guard(CharacterCollision::default(), || {
        let Some(controller) = (unsafe { controller.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "character controller is null");
            return CharacterCollision::default();
        };
        let Some(collision) = controller.inner.collisions.get(index as usize) else {
            set_error(ERR_NOT_FOUND, "collision index out of range");
            return CharacterCollision::default();
        };

        CharacterCollision {
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
    })
}

/// # Safety
///
/// `world` must be a valid world pointer and `controller` a valid pointer returned by
/// `character_controller_create`; both must remain alive for the duration of the call.
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_solve_impulses(
    world: *mut WorldHandle,
    controller: *mut CharacterControllerHandle,
    dt: f64,
    shape_desc: ShapeDesc,
    character_mass: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let _query_lock = world.inner.query_lock.write();
        let Some(controller) = (unsafe { controller.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "character controller is null");
            return Bool::FALSE;
        };
        if !dt.is_finite()
            || dt <= 0.0
            || !character_mass.is_finite()
            || character_mass < 0.0
            || !shape_desc_valid(shape_desc)
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid solve_impulses arguments");
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
    })
}

/// # Safety
///
/// `world` must be a valid world pointer and `controller` a valid pointer returned by
/// `character_controller_create`; both must remain alive for the duration of the call.
///
/// Like [`character_controller_move_shape`] but additionally samples the world's
/// registered terrain gravity (polyhedron / DEM / lunar-mascon) at the character's
/// current `translation` and folds the resulting free-fall displacement
/// (`陆路a路dt虏`, directed along the local terrain-gravity acceleration `a`) into the
/// desired translation.  This lets a kinematic character fall toward and stand on an
/// irregular small-body surface instead of floating.  When no terrain-gravity law is
/// registered the call is identical to `character_controller_move_shape`.
#[unsafe(no_mangle)]
pub extern "C" fn character_controller_move_shape_with_terrain(
    world: *const WorldHandle,
    controller: *mut CharacterControllerHandle,
    dt: f64,
    shape_desc: ShapeDesc,
    translation: Vec3,
    rotation: Quat,
    desired_translation: Vec3,
) -> EffectiveCharacterMovement {
    ffi_guard(EffectiveCharacterMovement::default(), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return EffectiveCharacterMovement::default();
        };
        let Some(controller) = (unsafe { controller.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "character controller is null");
            return EffectiveCharacterMovement::default();
        };
        if !dt.is_finite()
            || dt <= 0.0
            || !shape_desc_valid(shape_desc)
            || !vec3_finite(translation)
            || !quat_finite(rotation)
            || !vec3_finite(desired_translation)
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "invalid move_shape_with_terrain arguments",
            );
            return EffectiveCharacterMovement::default();
        }

        // Sample the world's terrain gravity at the character's current position and
        // add the corresponding free-fall displacement to the desired translation.
        let mut desired = vec3_to_rapier(desired_translation);
        if let Some(source) = &world.inner.terrain_gravity_source {
            let accel = terrain_gravity_acceleration(source, translation);
            desired += vec3_to_rapier(accel) * (0.5 * dt * dt);
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
            desired,
            |collision| controller.inner.collisions.push(collision),
        );

        EffectiveCharacterMovement {
            translation: vec3_from_rapier(movement.translation),
            grounded: movement.grounded.into(),
            is_sliding_down_slope: movement.is_sliding_down_slope.into(),
        }
    })
}
