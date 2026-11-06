//! PD/PID servo body — a **sixth body type** alongside rigid bodies, soft bodies,
//! character bodies, sensor zones and vehicles. A servo body is a dynamic rigid
//! body driven by a rapier `PdController` or `PidController` toward a target pose
//! (position + rotation) and/or target velocities. Each `servo_body_update` step
//! computes the velocity-level correction from the current body state vs. the
//! target and writes it back via `set_linvel`/`set_angvel`, so the body is
//! *driven* toward its target by the solver rather than kinematically snapped.
//!
//! This is a pure `mps-core` composition layer — rapier's `control` module
//! already provides `PdController`/`PidController`; no fork changes.
//!
//! Driving model (per step):
//! 1. `world_step(world, dt)` — advances the body and resolves contacts.
//! 2. `servo_body_update(world, id, dt)` — computes the PD/PID correction and
//!    writes the corrected linear/angular velocity back to the body.
//!
//! Typical uses: spacecraft attitude control (RCS/reaction-wheel abstraction),
//! robotic arm joints, active stabilization, hover/seek behaviors.

use rapier3d::control::{PdController, PdController as RapierPdController, PidController};
use rapier3d::math::{Rotation, Vector};
use rapier3d::prelude::{
    AxesMask, ColliderBuilder, RigidBodyBuilder, RigidBodyHandle, RigidBodyType,
};

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    Bool, Quat, RigidBodyHandleRaw, ShapeDesc, Vec3, WorldHandle, pack_rigid_body_handle,
    quat_finite, quat_to_rapier, shape_desc_valid, shape_from_desc, vec3_finite, vec3_from_rapier,
    vec3_to_rapier,
};

/// The controller flavour: pure PD (no integral term) or full PID.
#[derive(Debug)]
pub(crate) enum ServoController {
    Pd(PdController),
    Pid(PidController),
}

/// A servo body: a dynamic rigid body + a PD/PID controller + the target pose
/// and velocities the controller drives toward.
pub(crate) struct ServoBody {
    /// The dynamic rigid body being driven.
    pub body: RigidBodyHandle,
    /// PD or PID controller with caller-configured gains + axes.
    pub controller: ServoController,
    /// Target translation (world space). Defaults to the body's initial position.
    pub target_pos: Vector,
    /// Target rotation (world space). Defaults to identity.
    pub target_rot: Rotation,
    /// Target linear velocity (world space). Defaults to zero.
    pub target_linvel: Vector,
    /// Target angular velocity (world space). Defaults to zero.
    pub target_angvel: rapier3d::math::AngVector,
}

impl Default for ServoBody {
    fn default() -> Self {
        Self {
            body: RigidBodyHandle::default(),
            controller: ServoController::Pd(RapierPdController::default()),
            target_pos: Vector::ZERO,
            target_rot: Rotation::IDENTITY,
            target_linvel: Vector::ZERO,
            target_angvel: rapier3d::math::AngVector::ZERO,
        }
    }
}

/// Build an `AxesMask` from a raw `u8`. Bits: LIN_X=1, LIN_Y=2, LIN_Z=4,
/// ANG_X=8, ANG_Y=16, ANG_Z=32. A mask of 0 defaults to "all axes".
fn axes_from_u8(raw: u8) -> AxesMask {
    let mask = AxesMask::from_bits_truncate(raw);
    if mask.is_empty() {
        AxesMask::all()
    } else {
        mask
    }
}

/// Create a servo body in `world` from a collider shape and an initial
/// translation. The body is dynamic with a collider parented to it. Returns a
/// stable id, or `u32::MAX` on bad arguments.
///
/// - `kp`: proportional gain (applied uniformly to all axes).
/// - `kd`: derivative gain (applied uniformly to all axes).
/// - `ki`: integral gain. When `> 0`, a full `PidController` is used; when
///   `== 0`, a pure `PdController` (no integral term) is used instead.
/// - `axes`: bitfield selecting which axes the controller affects (see
///   `axes_from_u8`). `0` means all axes.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `shape` must be
/// a valid [`ShapeDesc`] (finite params).
#[unsafe(no_mangle)]
pub extern "C" fn servo_body_create(
    world: *mut WorldHandle,
    shape: ShapeDesc,
    translation: Vec3,
    kp: f64,
    kd: f64,
    ki: f64,
    axes: u8,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return u32::MAX;
        };
        if !shape_desc_valid(shape) || !vec3_finite(translation) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "servo_body_create: invalid shape/translation",
            );
            return u32::MAX;
        }
        if !kp.is_finite() || kp < 0.0 || !kd.is_finite() || kd < 0.0 || !ki.is_finite() || ki < 0.0
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "servo_body_create: gains must be finite and non-negative",
            );
            return u32::MAX;
        }
        let axes_mask = axes_from_u8(axes);
        let mut builder = RigidBodyBuilder::new(RigidBodyType::Dynamic);
        builder = builder.translation(vec3_to_rapier(translation));
        let body = world.inner.bodies.insert(builder.build());
        let collider = ColliderBuilder::new(shape_from_desc(shape)).build();
        world
            .inner
            .colliders
            .insert_with_parent(collider, body, &mut world.inner.bodies);
        let controller = if ki > 0.0 {
            ServoController::Pid(PidController::new(kp, ki, kd, axes_mask))
        } else {
            ServoController::Pd(PdController::new(kp, kd, axes_mask))
        };
        let target_pos = vec3_to_rapier(translation);
        let servo = ServoBody {
            body,
            controller,
            target_pos,
            target_rot: Rotation::IDENTITY,
            target_linvel: Vector::ZERO,
            target_angvel: rapier3d::math::AngVector::ZERO,
        };
        let id = world.inner.servo_bodies.insert(servo);
        clear_error();
        id
    })
}

/// Set the target world-space position the servo drives toward.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn servo_body_set_target_position(
    world: *mut WorldHandle,
    id: u32,
    position: Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !vec3_finite(position) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "servo_body_set_target_position: invalid position",
            );
            return Bool::FALSE;
        }
        let Some(servo) = world.inner.servo_bodies.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "servo_body_set_target_position: unknown id");
            return Bool::FALSE;
        };
        servo.target_pos = vec3_to_rapier(position);
        clear_error();
        Bool::TRUE
    })
}

/// Set the target world-space rotation (as a quaternion `i, j, k, w`) the servo
/// drives toward.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn servo_body_set_target_rotation(
    world: *mut WorldHandle,
    id: u32,
    rotation: Quat,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !quat_finite(rotation) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "servo_body_set_target_rotation: invalid rotation",
            );
            return Bool::FALSE;
        }
        let Some(servo) = world.inner.servo_bodies.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "servo_body_set_target_rotation: unknown id");
            return Bool::FALSE;
        };
        let n = (rotation.i * rotation.i
            + rotation.j * rotation.j
            + rotation.k * rotation.k
            + rotation.w * rotation.w)
            .sqrt();
        if n < 1e-12 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "servo_body_set_target_rotation: zero quaternion",
            );
            return Bool::FALSE;
        }
        servo.target_rot = quat_to_rapier(rotation);
        clear_error();
        Bool::TRUE
    })
}

/// Set the target linear velocity (world space) the servo drives toward.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn servo_body_set_target_velocity(
    world: *mut WorldHandle,
    id: u32,
    velocity: Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !vec3_finite(velocity) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "servo_body_set_target_velocity: invalid velocity",
            );
            return Bool::FALSE;
        }
        let Some(servo) = world.inner.servo_bodies.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "servo_body_set_target_velocity: unknown id");
            return Bool::FALSE;
        };
        servo.target_linvel = vec3_to_rapier(velocity);
        clear_error();
        Bool::TRUE
    })
}

/// Set the target angular velocity (world space) the servo drives toward.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn servo_body_set_target_angular_velocity(
    world: *mut WorldHandle,
    id: u32,
    velocity: Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !vec3_finite(velocity) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "servo_body_set_target_angular_velocity: invalid velocity",
            );
            return Bool::FALSE;
        }
        let Some(servo) = world.inner.servo_bodies.get_mut(id) else {
            set_error(
                ERR_NOT_FOUND,
                "servo_body_set_target_angular_velocity: unknown id",
            );
            return Bool::FALSE;
        };
        servo.target_angvel = vec3_to_rapier(velocity);
        clear_error();
        Bool::TRUE
    })
}

/// Advance the servo controller by `dt`: compute the PD/PID velocity-level
/// correction from the body's current pose/velocity vs. the target and write
/// it back via `set_linvel`/`set_angvel`. Call **after** `world_step`.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn servo_body_update(world: *mut WorldHandle, id: u32, dt: f64) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !dt.is_finite() || dt <= 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "servo_body_update: invalid dt");
            return Bool::FALSE;
        }
        let Some(servo) = world.inner.servo_bodies.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "servo_body_update: unknown id");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get_mut(servo.body) else {
            set_error(ERR_NOT_FOUND, "servo_body_update: body missing");
            return Bool::FALSE;
        };
        let target_pose = rapier3d::math::Pose::from_parts(servo.target_pos, servo.target_rot);
        let target_vels = rapier3d::dynamics::RigidBodyVelocity {
            linvel: servo.target_linvel,
            angvel: servo.target_angvel,
        };
        let correction = match &mut servo.controller {
            ServoController::Pd(pd) => pd.rigid_body_correction(body, target_pose, target_vels),
            ServoController::Pid(pid) => {
                pid.rigid_body_correction(dt, body, target_pose, target_vels)
            }
        };
        // Write the corrected velocity back. `rigid_body_correction` returns a
        // velocity *change* (Δv), so we add it to the body's current velocity.
        // wake_up=false avoids unnecessary island wake cascades — the body is
        // already awake (we just stepped it) and the servo runs every frame.
        body.set_linvel(body.linvel() + correction.linvel, false);
        body.set_angvel(body.angvel() + correction.angvel, false);
        clear_error();
        Bool::TRUE
    })
}

/// Read the body's world-space translation.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `out` may be null.
#[unsafe(no_mangle)]
pub extern "C" fn servo_body_get_translation(
    world: *const WorldHandle,
    id: u32,
    out: *mut Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(servo) = world.inner.servo_bodies.get(id) else {
            set_error(ERR_NOT_FOUND, "servo_body_get_translation: unknown id");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get(servo.body) else {
            set_error(ERR_NOT_FOUND, "servo_body_get_translation: body missing");
            return Bool::FALSE;
        };
        if !out.is_null() {
            unsafe { *out = vec3_from_rapier(body.translation()) };
        }
        clear_error();
        Bool::TRUE
    })
}

/// Read the body's world-space linear velocity.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `out` may be null.
#[unsafe(no_mangle)]
pub extern "C" fn servo_body_get_velocity(
    world: *const WorldHandle,
    id: u32,
    out: *mut Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(servo) = world.inner.servo_bodies.get(id) else {
            set_error(ERR_NOT_FOUND, "servo_body_get_velocity: unknown id");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get(servo.body) else {
            set_error(ERR_NOT_FOUND, "servo_body_get_velocity: body missing");
            return Bool::FALSE;
        };
        if !out.is_null() {
            unsafe { *out = vec3_from_rapier(body.linvel()) };
        }
        clear_error();
        Bool::TRUE
    })
}

/// Read the packed rigid-body handle so the caller can use the general
/// `rigid_body_*` FFI (forces, impulses, mass properties, etc.) on the
/// servo's underlying body.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn servo_body_get_rigid_body_handle(
    world: *const WorldHandle,
    id: u32,
) -> RigidBodyHandleRaw {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let Some(servo) = world.inner.servo_bodies.get(id) else {
            set_error(
                ERR_NOT_FOUND,
                "servo_body_get_rigid_body_handle: unknown id",
            );
            return 0;
        };
        clear_error();
        pack_rigid_body_handle(servo.body)
    })
}

/// Destroy a servo body by id. Removes the controller, the rigid body, and its
/// parented collider from the world. Returns `FALSE` if the id is unknown.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn servo_body_destroy(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        match world.inner.servo_bodies.remove(id) {
            Some(servo) => {
                world.inner.bodies.remove(
                    servo.body,
                    &mut world.inner.islands,
                    &mut world.inner.colliders,
                    &mut world.inner.impulse_joints,
                    &mut world.inner.multibody_joints,
                    &mut world.inner.upstream_soft_bodies,
                    true,
                );
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(ERR_NOT_FOUND, "servo_body_destroy: unknown id");
                Bool::FALSE
            }
        }
    })
}
