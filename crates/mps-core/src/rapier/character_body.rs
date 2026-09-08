//! End-to-end **character body** — a kinematic rigid body driven by a
//! [`KinematicCharacterController`]. This is the "third body type" alongside the
//! existing rigid bodies and soft bodies: a capsule/ball collider that walks,
//! slides, autosteps and snaps to the ground, with its resolved position written
//! back to a kinematic rigid body every step so it can push other bodies and be
//! queried like any other body.
//!
//! This layer reuses the collision query construction already in `controller.rs`
//! and just binds a controller to a kinematic body. No fork changes.

use rapier3d::control::{
    CharacterAutostep, CharacterCollision, CharacterLength, KinematicCharacterController,
};
use rapier3d::prelude::{
    ColliderBuilder, ColliderHandle, QueryFilter, RigidBodyBuilder, RigidBodyHandle, RigidBodyType,
    Vector,
};

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    Bool, EffectiveCharacterMovement, Quat, ShapeDesc, Vec3, WorldHandle, isometry_from_parts,
    shape_desc_valid, shape_from_desc, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

/// A character: a controller bound to a kinematic rigid body (which owns the
/// collider built from `shape`). `last_movement` caches the most recent resolve.
#[derive(Default)]
pub(crate) struct CharacterBody {
    pub controller: KinematicCharacterController,
    pub body: RigidBodyHandle,
    /// Collider inserted into the world and parented to `body`. It lets the
    /// kinematic character participate in the dynamic world (so `world_step`
    /// resolves kinematic-vs-dynamic contacts and the character pushes other
    /// bodies). It is excluded from the controller's own shape-cast via a
    /// `QueryFilter` (see `character_body_move`), so the character never catches
    /// itself.
    pub collider: ColliderHandle,
    pub shape: ShapeDesc,
    /// When true (default), `character_body_solve_impulses` transfers the
    /// character's intended momentum to the dynamic bodies it is blocked against.
    /// Set false to make the character "ghost" through dynamic bodies without
    /// shoving them (it still resolves against static geometry).
    pub apply_impulses_to_dynamic_bodies: bool,
    pub last_movement: EffectiveCharacterMovement,
    /// Collisions captured by the most recent `character_body_move` (cleared each
    /// call, populated by the move's collision callback). Read back via
    /// `character_body_collision_count` / `character_body_get_collision` and fed to
    /// `character_body_solve_impulses`.
    pub collisions: Vec<CharacterCollision>,
}

macro_rules! character_mut {
    ($world:expr, $id:expr) => {{
        match $world.inner.character_bodies.get_mut($id) {
            Some(value) => value,
            None => {
                set_error(ERR_NOT_FOUND, "character body handle is no longer valid");
                return Default::default();
            }
        }
    }};
}

/// Create a character body in `world` from a collider shape and an initial
/// translation. Returns a stable id, or `u32::MAX` on bad arguments. The character
/// is a `KinematicPositionBased` rigid body so its position is driven externally
/// by [`character_body_move`]. A world collider is inserted and parented to the
/// body so the character participates in the dynamic world (it can push other
/// bodies during `world_step`); that collider is excluded from the controller's
/// own shape-cast via a `QueryFilter`, so the character never catches itself.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_create(
    world: *mut WorldHandle,
    shape: ShapeDesc,
    translation: Vec3,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return u32::MAX;
        };
        if !shape_desc_valid(shape) || !vec3_finite(translation) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "character_body_create: invalid shape/translation",
            );
            return u32::MAX;
        }

        // Kinematic body we drive from the controller's resolved movement.
        let mut builder = RigidBodyBuilder::new(RigidBodyType::KinematicPositionBased);
        builder = builder.translation(Vector::new(translation.x, translation.y, translation.z));
        let body = world.inner.bodies.insert(builder.build());

        // Insert a collider parented to the kinematic body so the character
        // participates in the dynamic world (kinematic-vs-dynamic contact is
        // resolved by `world_step`, letting the character push other bodies).
        // The controller's own shape-cast excludes this collider (see
        // `character_body_move`), so the character never catches itself.
        let collider_builder = ColliderBuilder::new(shape_from_desc(shape));
        let collider = world.inner.colliders.insert_with_parent(
            collider_builder.build(),
            body,
            &mut world.inner.bodies,
        );

        let id = world.inner.character_bodies.insert(CharacterBody {
            controller: KinematicCharacterController::default(),
            body,
            collider,
            shape,
            apply_impulses_to_dynamic_bodies: true,
            last_movement: EffectiveCharacterMovement::default(),
            collisions: Vec::new(),
        });
        clear_error();
        id
    })
}

/// Change a character body's collision shape after creation. The new shape is
/// used by subsequent `character_body_move` calls (the controller shape-casts the
/// shape directly) and is applied to the world collider in place (so the character
/// keeps pushing other bodies with the new hitbox). Useful for Minecraft style
/// avatars that change hitbox (e.g. sneaking shrinks the box).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `shape` must be a
/// valid [`ShapeDesc`] (finite params).
#[unsafe(no_mangle)]
pub extern "C" fn character_body_set_shape(
    world: *mut WorldHandle,
    id: u32,
    shape: ShapeDesc,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !shape_desc_valid(shape) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "character_body_set_shape: invalid shape",
            );
            return Bool::FALSE;
        }
        match world.inner.character_bodies.get_mut(id) {
            Some(cb) => {
                cb.shape = shape;
                // Update the parented world collider in place so the new hitbox is
                // used by the next `world_step` contact resolution.
                if let Some(collider) = world.inner.colliders.get_mut(cb.collider) {
                    collider.set_shape(shape_from_desc(shape));
                }
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(ERR_NOT_FOUND, "character_body_set_shape: unknown id");
                Bool::FALSE
            }
        }
    })
}

/// Advance the character by `desired` (a desired translation for this step). The
/// controller resolves collisions/slopes/steps and the result is written back to
/// the kinematic body. Returns the effective movement (resolved translation,
/// `grounded`, `is_sliding_down_slope`).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_move(
    world: *mut WorldHandle,
    id: u32,
    desired: Vec3,
    dt: f64,
) -> EffectiveCharacterMovement {
    ffi_guard(EffectiveCharacterMovement::default(), || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return EffectiveCharacterMovement::default();
        };
        if !world.inner.character_bodies.contains_key(id) {
            set_error(ERR_NOT_FOUND, "character_body_move: unknown id");
            return EffectiveCharacterMovement::default();
        }
        if !dt.is_finite() || dt <= 0.0 || !vec3_finite(desired) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "character_body_move: invalid dt/desired",
            );
            return EffectiveCharacterMovement::default();
        }

        // Resolve the desired movement against the world (read-only query). Exclude
        // the character's own collider so its shape-cast never catches itself.
        let Some(character) = world.inner.character_bodies.get(id) else {
            set_error(ERR_NOT_FOUND, "character_body_move: unknown id");
            return EffectiveCharacterMovement::default();
        };
        let body = character.body;
        let self_collider = character.collider;
        let Some(body_ref) = world.inner.bodies.get(body) else {
            set_error(
                ERR_NOT_FOUND,
                "character_body_move: backing rigid body missing",
            );
            return EffectiveCharacterMovement::default();
        };
        let current = body_ref.translation();
        let mut collected: Vec<CharacterCollision> = Vec::new();
        let movement = {
            let Some(cb) = world.inner.character_bodies.get(id) else {
                set_error(ERR_NOT_FOUND, "character_body_move: unknown id");
                return EffectiveCharacterMovement::default();
            };
            let shape = shape_from_desc(cb.shape);
            let query = world.inner.broad_phase.as_query_pipeline(
                world.inner.narrow_phase.query_dispatcher(),
                &world.inner.bodies,
                &world.inner.colliders,
                QueryFilter::new().exclude_collider(self_collider),
            );
            cb.controller.move_shape(
                dt,
                &query,
                shape.as_ref(),
                &isometry_from_parts(vec3_from_rapier(current), Quat::default()),
                vec3_to_rapier(desired),
                |collision| collected.push(collision),
            )
        };
        // Store this step's collisions for read-back / impulse solving.
        character_mut!(world, id).collisions = collected;

        // `movement.translation` is the *delta* to apply this step; add it to the
        // current pose and queue it as the next kinematic translation so the
        // following `world_step` applies it and refreshes the broad phase.
        let new_pos = current + movement.translation;
        if let Some(rb) = world.inner.bodies.get_mut(body) {
            rb.set_next_kinematic_translation(new_pos);
        }

        let result = EffectiveCharacterMovement {
            translation: vec3_from_rapier(movement.translation),
            grounded: movement.grounded.into(),
            is_sliding_down_slope: movement.is_sliding_down_slope.into(),
        };
        character_mut!(world, id).last_movement = result;
        clear_error();
        result
    })
}

/// Set the character's up vector (used for slope/ground semantics). Defaults to
/// world +Y. Mirrors `KinematicCharacterController::setUp`.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_set_up(world: *mut WorldHandle, id: u32, up: Vec3) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !world.inner.character_bodies.contains_key(id) {
            set_error(ERR_NOT_FOUND, "character_body_set_up: unknown id");
            return Bool::FALSE;
        }
        if !vec3_finite(up) {
            set_error(ERR_INVALID_ARGUMENT, "character_body_set_up: non-finite up");
            return Bool::FALSE;
        }
        character_mut!(world, id).controller.up = vec3_to_rapier(up);
        clear_error();
        Bool::TRUE
    })
}

/// Set the character controller's skin/offset (absolute, in metres).
#[unsafe(no_mangle)]
pub extern "C" fn character_body_set_offset_absolute(
    world: *mut WorldHandle,
    id: u32,
    offset: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !world.inner.character_bodies.contains_key(id) {
            set_error(
                ERR_NOT_FOUND,
                "character_body_set_offset_absolute: unknown id",
            );
            return Bool::FALSE;
        }
        if !offset.is_finite() || offset < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "offset must be finite and non-negative",
            );
            return Bool::FALSE;
        }
        character_mut!(world, id).controller.offset = CharacterLength::Absolute(offset);
        clear_error();
        Bool::TRUE
    })
}

/// Set the character controller's skin/offset (relative, as a fraction of the
/// shape's dimensions).
#[unsafe(no_mangle)]
pub extern "C" fn character_body_set_offset_relative(
    world: *mut WorldHandle,
    id: u32,
    offset: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !world.inner.character_bodies.contains_key(id) {
            set_error(
                ERR_NOT_FOUND,
                "character_body_set_offset_relative: unknown id",
            );
            return Bool::FALSE;
        }
        if !offset.is_finite() || offset < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "offset must be finite and non-negative",
            );
            return Bool::FALSE;
        }
        character_mut!(world, id).controller.offset = CharacterLength::Relative(offset);
        clear_error();
        Bool::TRUE
    })
}

/// Enable / disable auto-stepping so the character can climb block-sized ledges
/// (e.g. a 1-metre Minecraft step). `max_height` and `min_width` are absolute
/// metres; `include_dynamic_bodies` lets the step ride on moving platforms.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_set_autostep(
    world: *mut WorldHandle,
    id: u32,
    enabled: Bool,
    max_height: f64,
    min_width: f64,
    include_dynamic_bodies: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !world.inner.character_bodies.contains_key(id) {
            set_error(ERR_NOT_FOUND, "character_body_set_autostep: unknown id");
            return Bool::FALSE;
        }
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
            return Bool::FALSE;
        }
        character_mut!(world, id).controller.autostep = if enabled.0 != 0 {
            Some(CharacterAutostep {
                max_height: CharacterLength::Absolute(max_height),
                min_width: CharacterLength::Absolute(min_width),
                include_dynamic_bodies: include_dynamic_bodies.0 != 0,
            })
        } else {
            None
        };
        clear_error();
        Bool::TRUE
    })
}

/// Enable / disable snap-to-ground so the character sticks to block surfaces
/// instead of floating a hair above them after a step.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_set_snap_to_ground(
    world: *mut WorldHandle,
    id: u32,
    enabled: Bool,
    distance: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !world.inner.character_bodies.contains_key(id) {
            set_error(
                ERR_NOT_FOUND,
                "character_body_set_snap_to_ground: unknown id",
            );
            return Bool::FALSE;
        }
        if enabled.0 != 0 && (!distance.is_finite() || distance < 0.0) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "snap distance must be finite and non-negative",
            );
            return Bool::FALSE;
        }
        character_mut!(world, id).controller.snap_to_ground = if enabled.0 != 0 {
            Some(CharacterLength::Absolute(distance))
        } else {
            None
        };
        clear_error();
        Bool::TRUE
    })
}

/// Set the slope-climb / slope-slide angles (radians). Tune these so the
/// character climbs gentle block ramps but slides down steep ones.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_set_slope_angles(
    world: *mut WorldHandle,
    id: u32,
    max_climb_angle: f64,
    min_slide_angle: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !world.inner.character_bodies.contains_key(id) {
            set_error(ERR_NOT_FOUND, "character_body_set_slope_angles: unknown id");
            return Bool::FALSE;
        }
        if !max_climb_angle.is_finite() || !min_slide_angle.is_finite() {
            set_error(ERR_INVALID_ARGUMENT, "slope angles must be finite");
            return Bool::FALSE;
        }
        let cb = character_mut!(world, id);
        cb.controller.max_slope_climb_angle = max_climb_angle;
        cb.controller.min_slope_slide_angle = min_slide_angle;
        clear_error();
        Bool::TRUE
    })
}

/// Enable / disable sliding along walls/floors when the character is blocked.
/// `slide = true` gives the smooth Minecraft-style "glide along a wall" feel;
/// `slide = false` makes the character stop dead on contact.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_set_slide(world: *mut WorldHandle, id: u32, slide: Bool) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !world.inner.character_bodies.contains_key(id) {
            set_error(ERR_NOT_FOUND, "character_body_set_slide: unknown id");
            return Bool::FALSE;
        }
        character_mut!(world, id).controller.slide = slide.0 != 0;
        clear_error();
        Bool::TRUE
    })
}

/// Whether the character was on the ground during the last `character_body_move`.
/// Essential for Minecraft-style jump logic (only jump when grounded).
#[unsafe(no_mangle)]
pub extern "C" fn character_body_is_grounded(world: *const WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        match world.inner.character_bodies.get(id) {
            Some(cb) => cb.last_movement.grounded,
            None => {
                set_error(ERR_NOT_FOUND, "character_body_is_grounded: unknown id");
                Bool::FALSE
            }
        }
    })
}

/// Whether the character was sliding down a slope during the last
/// `character_body_move`. Useful for Minecraft-style ice/slide behaviour.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_is_sliding_down_slope(world: *const WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        match world.inner.character_bodies.get(id) {
            Some(cb) => cb.last_movement.is_sliding_down_slope,
            None => {
                set_error(
                    ERR_NOT_FOUND,
                    "character_body_is_sliding_down_slope: unknown id",
                );
                Bool::FALSE
            }
        }
    })
}

/// Reliable "is the character standing on something" check for Minecraft-style
/// jump logic. This fork's `is_grounded` classifies a capsule resting on a flat
/// floor as `sliding_down_slope` (see `is_grounded_at_contact_manifold`'s normal
/// convention), so it alone is NOT a good jump gate. This helper ORs `grounded`
/// with `is_sliding_down_slope` and additionally excludes the case where the
/// character is moving strongly upward (i.e. already jumping), giving a stable
/// on-ground signal the caller can gate jumps on.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_is_on_ground(world: *const WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        match world.inner.character_bodies.get(id) {
            Some(cb) => {
                let touching = cb.last_movement.grounded.0 != 0
                    || cb.last_movement.is_sliding_down_slope.0 != 0;
                // Exclude an active upward jump (vertical speed well above 0).
                let rising = cb.last_movement.translation.y > 0.05;
                Bool::from(touching && !rising)
            }
            None => {
                set_error(ERR_NOT_FOUND, "character_body_is_on_ground: unknown id");
                Bool::FALSE
            }
        }
    })
}

/// Number of collisions captured by the most recent `character_body_move`. Use
/// this with [`character_body_get_collision`] to inspect what the character hit
/// (e.g. to apply custom push forces or build a contact-reporting system).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_collision_count(world: *const WorldHandle, id: u32) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        match world.inner.character_bodies.get(id) {
            Some(cb) => cb.collisions.len() as u32,
            None => {
                set_error(ERR_NOT_FOUND, "character_body_collision_count: unknown id");
                0
            }
        }
    })
}

/// Read the `index`-th collision captured by the most recent `character_body_move`.
/// Returns a default (all-zero) collision if `index` is out of range.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_get_collision(
    world: *const WorldHandle,
    id: u32,
    index: u32,
) -> crate::rapier::ffi::CharacterCollision {
    ffi_guard(crate::rapier::ffi::CharacterCollision::default(), || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return crate::rapier::ffi::CharacterCollision::default();
        };
        let Some(cb) = world.inner.character_bodies.get(id) else {
            set_error(ERR_NOT_FOUND, "character_body_get_collision: unknown id");
            return crate::rapier::ffi::CharacterCollision::default();
        };
        match cb.collisions.get(index as usize) {
            Some(c) => crate::rapier::ffi::CharacterCollision {
                collider: crate::rapier::ffi::pack_collider_handle(c.handle),
                character_translation: vec3_from_rapier(c.character_pos.translation),
                translation_applied: vec3_from_rapier(c.translation_applied),
                translation_remaining: vec3_from_rapier(c.translation_remaining),
                world_witness1: vec3_from_rapier(c.hit.witness1),
                world_witness2: vec3_from_rapier(c.hit.witness2),
                normal1: vec3_from_rapier(c.hit.normal1),
                normal2: vec3_from_rapier(c.hit.normal2),
                time_of_impact: c.hit.time_of_impact,
            },
            None => {
                set_error(
                    ERR_NOT_FOUND,
                    "character_body_get_collision: index out of range",
                );
                crate::rapier::ffi::CharacterCollision::default()
            }
        }
    })
}

/// Apply the impulses accumulated from the latest `character_body_move` to the
/// dynamic bodies the character is touching. This is how a kinematic character
/// "pushes" crates/other rigid bodies. For each captured collision that was
/// actually blocked (non-zero `translation_remaining`) against a dynamic body, we
/// apply an impulse of `character_mass * remaining / dt` along the blocked
/// direction — i.e. the momentum the character wanted to carry into the body.
/// Rapier's own `solve_character_collision_impulses` only separates bodies, so the
/// forward push is implemented here, with no fork changes. Call this after each
/// move that reported contacts.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_solve_impulses(
    world: *mut WorldHandle,
    id: u32,
    dt: f64,
    character_mass: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !world.inner.character_bodies.contains_key(id) {
            set_error(ERR_NOT_FOUND, "character_body_solve_impulses: unknown id");
            return Bool::FALSE;
        }
        if !dt.is_finite() || dt <= 0.0 || !character_mass.is_finite() || character_mass < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "character_body_solve_impulses: invalid dt/mass",
            );
            return Bool::FALSE;
        }
        let _query_lock = world.inner.query_lock.write();
        // Transfer the character's intended (blocked) momentum to the dynamic bodies
        // it is pushing into. rapier's own `solve_character_collision_impulses` only
        // applies a *separating* impulse (it zeroes any push along the contact normal
        // via `delta_vel_per_contact.max(0.0)`), so a resting body is never shoved
        // forward by it. Instead we use the captured collision's `translation_remaining`
        // — the displacement the character *wanted* but was blocked from taking — as the
        // push vector, and apply `mass * v` (v = remaining/dt) to the contacted dynamic
        // body. This is the "character pushes crates" behaviour, with no fork changes.
        let Some(cb) = world.inner.character_bodies.get(id) else {
            set_error(ERR_NOT_FOUND, "character_body_solve_impulses: unknown id");
            return Bool::FALSE;
        };
        if !cb.apply_impulses_to_dynamic_bodies {
            clear_error();
            return Bool::TRUE;
        }
        for c in cb.collisions.iter() {
            if let Some(collider) = world.inner.colliders.get(c.handle)
                && let Some(parent) = collider.parent()
                && let Some(body) = world.inner.bodies.get_mut(parent)
                && body.is_dynamic()
            {
                // Drive the contacted body toward the character's intended
                // velocity (the blocked displacement per step), capped to the
                // horizontal plane so gravity keeps working. This makes the body
                // move *with* the character instead of being launched: an impulse
                // of `mass * (desired_vel - current_vel)` only closes the gap each
                // step, so gravity/friction still apply.
                let mut desired_vel = c.translation_remaining / dt;
                desired_vel.y = 0.0;
                let mut delta = desired_vel - body.linvel();
                delta.y = 0.0;
                let mass = body.mass();
                body.apply_impulse(delta * mass, true);
            }
        }
        clear_error();
        Bool::TRUE
    })
}

/// Enable or disable transferring the character's intended momentum to the dynamic
/// bodies it is blocked against (default: enabled). When disabled, the character
/// still resolves against static geometry but does not shove dynamic bodies — it
/// "ghosts" through them. No fork changes.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_set_apply_impulses_to_dynamic_bodies(
    world: *mut WorldHandle,
    id: u32,
    enabled: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(cb) = world.inner.character_bodies.get_mut(id) else {
            set_error(
                ERR_NOT_FOUND,
                "character_body_set_apply_impulses: unknown id",
            );
            return Bool::FALSE;
        };
        cb.apply_impulses_to_dynamic_bodies = enabled.0 != 0;
        clear_error();
        Bool::TRUE
    })
}

/// Like [`character_body_move`] but additionally samples the world's registered
/// terrain gravity (polyhedron / DEM / lunar-mascon) at the character's current
/// position and folds the resulting free-fall displacement (`½·a·dt²`) into the
/// desired translation, so the character falls toward and stands on an irregular
/// small-body surface instead of floating. When no terrain-gravity law is
/// registered this is identical to `character_body_move`.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_move_with_terrain(
    world: *mut WorldHandle,
    id: u32,
    desired: Vec3,
    dt: f64,
) -> EffectiveCharacterMovement {
    ffi_guard(EffectiveCharacterMovement::default(), || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return EffectiveCharacterMovement::default();
        };
        if !world.inner.character_bodies.contains_key(id) {
            set_error(
                ERR_NOT_FOUND,
                "character_body_move_with_terrain: unknown id",
            );
            return EffectiveCharacterMovement::default();
        }
        if !dt.is_finite() || dt <= 0.0 || !vec3_finite(desired) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "character_body_move_with_terrain: invalid dt/desired",
            );
            return EffectiveCharacterMovement::default();
        }

        // Sample terrain gravity and fold the free-fall displacement into desired.
        let mut desired = vec3_to_rapier(desired);
        if let Some(source) = &world.inner.terrain_gravity_source {
            let accel = crate::rapier::terrain_gravity::terrain_gravity_acceleration(source, {
                let Some(character) = world.inner.character_bodies.get(id) else {
                    set_error(
                        ERR_NOT_FOUND,
                        "character_body_move_with_terrain: unknown id",
                    );
                    return EffectiveCharacterMovement::default();
                };
                let Some(body) = world.inner.bodies.get(character.body) else {
                    set_error(
                        ERR_NOT_FOUND,
                        "character_body_move_with_terrain: backing body missing",
                    );
                    return EffectiveCharacterMovement::default();
                };
                vec3_from_rapier(body.translation())
            });
            desired += vec3_to_rapier(accel) * (0.5 * dt * dt);
        }

        // Delegate the resolve + kinematic write-back to the shared move path.
        let Some(character) = world.inner.character_bodies.get(id) else {
            set_error(
                ERR_NOT_FOUND,
                "character_body_move_with_terrain: unknown id",
            );
            return EffectiveCharacterMovement::default();
        };
        let body = character.body;
        let Some(body_ref) = world.inner.bodies.get(body) else {
            set_error(
                ERR_NOT_FOUND,
                "character_body_move_with_terrain: backing body missing",
            );
            return EffectiveCharacterMovement::default();
        };
        let current = body_ref.translation();
        let mut collected: Vec<CharacterCollision> = Vec::new();
        let movement = {
            let Some(cb) = world.inner.character_bodies.get(id) else {
                set_error(
                    ERR_NOT_FOUND,
                    "character_body_move_with_terrain: unknown id",
                );
                return EffectiveCharacterMovement::default();
            };
            let shape = shape_from_desc(cb.shape);
            let query = world.inner.broad_phase.as_query_pipeline(
                world.inner.narrow_phase.query_dispatcher(),
                &world.inner.bodies,
                &world.inner.colliders,
                QueryFilter::default(),
            );
            cb.controller.move_shape(
                dt,
                &query,
                shape.as_ref(),
                &isometry_from_parts(vec3_from_rapier(current), Quat::default()),
                desired,
                |collision| collected.push(collision),
            )
        };
        character_mut!(world, id).collisions = collected;

        let new_pos = current + movement.translation;
        if let Some(rb) = world.inner.bodies.get_mut(body) {
            rb.set_next_kinematic_translation(new_pos);
        }

        let result = EffectiveCharacterMovement {
            translation: vec3_from_rapier(movement.translation),
            grounded: movement.grounded.into(),
            is_sliding_down_slope: movement.is_sliding_down_slope.into(),
        };
        character_mut!(world, id).last_movement = result;
        clear_error();
        result
    })
}

/// Destroy a character body, removing its rigid body, collider and controller
/// state. Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_destroy(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        match world.inner.character_bodies.remove(id) {
            Some(cb) => {
                // Removing the rigid body also frees its parented collider.
                world.inner.bodies.remove(
                    cb.body,
                    &mut world.inner.islands,
                    &mut world.inner.colliders,
                    &mut world.inner.impulse_joints,
                    &mut world.inner.multibody_joints,
                    false,
                );
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(ERR_NOT_FOUND, "character_body_destroy: unknown id");
                Bool::FALSE
            }
        }
    })
}

/// Read the character body's current world-space translation (the kinematic body
/// pose driven by [`character_body_move`]). Writes into `out` when non-null.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `out` may be null.
#[unsafe(no_mangle)]
pub extern "C" fn character_body_get_translation(
    world: *const WorldHandle,
    id: u32,
    out: *mut Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        match world.inner.character_bodies.get(id) {
            Some(cb) => {
                let Some(body) = world.inner.bodies.get(cb.body) else {
                    set_error(
                        ERR_NOT_FOUND,
                        "character_body_get_translation: backing body missing",
                    );
                    return Bool::FALSE;
                };
                let t = body.translation();
                if !out.is_null() {
                    unsafe { *out = vec3_from_rapier(t) };
                }
                Bool::TRUE
            }
            None => {
                set_error(ERR_NOT_FOUND, "character_body_get_translation: unknown id");
                Bool::FALSE
            }
        }
    })
}
