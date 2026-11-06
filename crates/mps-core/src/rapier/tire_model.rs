//! Tire model — physics-based tire controller for vehicles.
//!
//! A **tire model** provides realistic tire physics beyond the ray-cast suspension
//! used by `vehicle.rs`. This implements a simplified Pacejka-style tire model
//! with:
//!
//! * **Longitudinal slip** — a per-wheel angular-speed state driven by
//!   `engine_force` / `brake` and reacted against by the computed tire force
//!   (so a spinning wheel settles at a realistic slip ratio)
//! * **Lateral slip** — cornering force from the slip angle measured at the
//!   wheel contact point
//! * **Load sensitivity** — tire forces scale with the per-wheel suspension
//!   load (falling back to a static per-wheel estimate when airborne)
//! * **Friction ellipse** — combined slip limits for realistic drifting
//!
//! `tire_model_update` computes and *stores* the per-wheel forces; read them
//! back with `tire_model_get_forces` and apply them to the chassis with the
//! existing rigid-body impulse FFI. Callers that want the model to replace the
//! vehicle controller's built-in friction should keep the wheel
//! `friction_slip` low; the forces reported here are independent of it.

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, ffi_guard,
    set_error,
};
use crate::rapier::ffi::{Bool, WorldHandle};

const MAX_TIRE_COUNT: u32 = 32;
/// Fraction of the chassis mass assumed for each wheel when estimating the
/// wheel spin inertia (typical road car: unsprung mass ≈ 5–10% of total).
const WHEEL_MASS_FRACTION: f64 = 0.06;
/// Floor for the denominator of the slip-ratio / slip-angle normalisation so
/// the model stays finite at standstill and low speed.
const MIN_REF_SPEED: f64 = 1.0;

/// Reference wheel load (N) used to non-dimensionalise load sensitivity.
///
/// Roughly one quarter of a 1600 kg car's weight, i.e. a typical passenger-car
/// wheel at rest. Load sensitivity is defined relative to it, so the friction
/// coefficient equals `peak_mu_*` at this load and the produced force keeps
/// units of newtons.
const REFERENCE_LOAD: f64 = 4_000.0;

/// Friction-curve fraction of the peak that survives at very large slip.
///
/// Real tyres saturate at their sliding-friction level (≈70–90 % of peak), they
/// do not decay towards zero. See [`slip_curve`].
const SLIDE_FRICTION_FRACTION: f64 = 0.8;

/// Normalised longitudinal-force fraction for a given slip value.
///
/// Linear up to `peak`, then a smooth saturation towards
/// [`SLIDE_FRICTION_FRACTION`]:
///
/// ```text
/// |s| <= peak :  s / peak
/// |s| >  peak :  sign(s) · (F + (1 − F) · peak / |s|)
/// ```
///
/// The previous form was `sign(s) · peak / |s|`, which decays as 1/|s|: a fully
/// locked wheel (maximum slip) produced zero braking force and a large slip
/// angle produced zero cornering force, i.e. exactly the opposite of what a tyre
/// does. `f(peak) = 1` in both branches, so the curve is continuous.
fn slip_curve(slip: f64, peak: f64) -> f64 {
    if peak <= 0.0 || !peak.is_finite() {
        return 0.0;
    }
    let magnitude = slip.abs();
    if magnitude <= peak {
        return slip / peak;
    }
    let tail = SLIDE_FRICTION_FRACTION + (1.0 - SLIDE_FRICTION_FRACTION) * (peak / magnitude);
    slip.signum() * tail
}

/// Load-sensitivity multiplier for the peak friction coefficient.
///
/// `(Fz / Fz0)^(n − 1)`: 1.0 at the reference load, and `peak_mu · Fz · factor`
/// stays a force. The old code returned `Fz^n`, which has units of N^0.8 — a
/// 1200 kg car got ≈715 N of peak longitudinal force per wheel where μ·N is
/// ≈3531 N, and grip barely grew with load.
fn load_sensitivity_factor(normal_load: f64, sensitivity: f64) -> f64 {
    let load = normal_load.max(0.0);
    if load <= 0.0 {
        return 0.0;
    }
    (load / REFERENCE_LOAD).powf(sensitivity - 1.0)
}

/// Tire model parameters (simplified Pacejka).
#[derive(Debug, Clone, Copy)]
pub(crate) struct TireParams {
    /// Peak longitudinal friction coefficient (maximum acceleration/braking force).
    pub peak_mu_long: f64,
    /// Peak lateral friction coefficient (maximum cornering force).
    pub peak_mu_lat: f64,
    /// Slip ratio at peak longitudinal force (0.1-0.2 typical).
    pub peak_slip_ratio: f64,
    /// Slip angle at peak lateral force (in radians, 0.1-0.2 typical).
    pub peak_slip_angle: f64,
    /// Load sensitivity exponent (0.7-1.0 typical, <1.0 means force grows slower than load).
    pub load_sensitivity: f64,
    /// Friction ellipse shaping factor (1.0 = circle, >1.0 = combined slip allows more force).
    pub ellipse_factor: f64,
}

impl Default for TireParams {
    fn default() -> Self {
        Self {
            peak_mu_long: 1.2,
            peak_mu_lat: 1.2,
            peak_slip_ratio: 0.15,
            peak_slip_angle: 0.15,
            load_sensitivity: 0.8,
            ellipse_factor: 1.3,
        }
    }
}

/// Tire state (dynamic per-frame).
#[derive(Debug, Clone)]
pub(crate) struct TireState {
    /// Vehicle wheel index this tire is attached to.
    pub wheel_index: u32,
    /// Tire parameters.
    pub params: TireParams,
    /// Wheel angular speed (rad/s, positive = rolling forward).
    pub wheel_omega: f64,
    /// Current slip ratio (negative = braking, positive = acceleration).
    pub slip_ratio: f64,
    /// Current slip angle (lateral slip in radians).
    pub slip_angle: f64,
    /// Current normal load (from suspension).
    pub normal_load: f64,
    /// Last computed longitudinal force.
    pub fx: f64,
    /// Last computed lateral force.
    pub fy: f64,
}

/// Tire model controller.
pub(crate) struct TireModel {
    /// Vehicle controller id this tire model is attached to.
    pub vehicle_id: u32,
    /// Tire states (indexed by wheel index).
    pub tires: Vec<TireState>,
}

/// Create a tire model for a vehicle controller.
///
/// Returns a stable id, or `u32::MAX` on error.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn tire_model_create(
    world: *mut WorldHandle,
    vehicle_id: u32,
    wheel_count: u32,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return u32::MAX;
        };
        if wheel_count == 0 || wheel_count > MAX_TIRE_COUNT {
            set_error(ERR_CAPACITY, "invalid wheel count");
            return u32::MAX;
        }
        if !world.inner.vehicle_controllers.contains_key(vehicle_id) {
            set_error(ERR_NOT_FOUND, "vehicle controller not found");
            return u32::MAX;
        }

        let mut tires = Vec::with_capacity(wheel_count as usize);
        for i in 0..wheel_count {
            tires.push(TireState {
                wheel_index: i,
                params: TireParams::default(),
                wheel_omega: 0.0,
                slip_ratio: 0.0,
                slip_angle: 0.0,
                normal_load: 0.0,
                fx: 0.0,
                fy: 0.0,
            });
        }

        let id = world
            .inner
            .tire_models
            .insert(TireModel { vehicle_id, tires });

        clear_error();
        id
    })
}

/// Set tire parameters for a specific wheel.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn tire_model_set_params(
    world: *mut WorldHandle,
    id: u32,
    wheel_index: u32,
    peak_mu_long: f64,
    peak_mu_lat: f64,
    peak_slip_ratio: f64,
    peak_slip_angle: f64,
    load_sensitivity: f64,
    ellipse_factor: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        let Some(tire_model) = world.inner.tire_models.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "tire model not found");
            return Bool::FALSE;
        };

        if wheel_index as usize >= tire_model.tires.len() {
            set_error(ERR_INVALID_ARGUMENT, "wheel index out of range");
            return Bool::FALSE;
        }

        if !peak_mu_long.is_finite()
            || peak_mu_long <= 0.0
            || !peak_mu_lat.is_finite()
            || peak_mu_lat <= 0.0
            || !peak_slip_ratio.is_finite()
            || peak_slip_ratio <= 0.0
            || !peak_slip_angle.is_finite()
            || peak_slip_angle <= 0.0
            || !load_sensitivity.is_finite()
            || load_sensitivity <= 0.0
            || !ellipse_factor.is_finite()
            || ellipse_factor <= 0.0
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid tire parameters");
            return Bool::FALSE;
        }

        tire_model.tires[wheel_index as usize].params = TireParams {
            peak_mu_long,
            peak_mu_lat,
            peak_slip_ratio,
            peak_slip_angle,
            load_sensitivity,
            ellipse_factor,
        };

        clear_error();
        Bool::TRUE
    })
}

/// Compute tire forces based on current wheel state.
///
/// This should be called each frame **after** `vehicle_controller_update` so
/// the wheel transforms (steering, world-space axle, suspension force) are
/// fresh. The computed forces are stored per wheel; read them with
/// `tire_model_get_forces` and apply them to the chassis via the rigid-body
/// impulse FFI.
///
/// Returns `true` on success.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn tire_model_update(world: *mut WorldHandle, id: u32, dt: f64) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        if !dt.is_finite() || dt <= 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid dt");
            return Bool::FALSE;
        }

        let vehicle_id = match world.inner.tire_models.get(id) {
            Some(tire_model) => tire_model.vehicle_id,
            None => {
                set_error(ERR_NOT_FOUND, "tire model not found");
                return Bool::FALSE;
            }
        };

        let Some(vehicle) = world.inner.vehicle_controllers.get(vehicle_id) else {
            set_error(ERR_NOT_FOUND, "vehicle controller not found");
            return Bool::FALSE;
        };

        let Some(chassis) = world.inner.bodies.get(vehicle.body) else {
            set_error(ERR_NOT_FOUND, "chassis body not found");
            return Bool::FALSE;
        };

        let chassis_mass = chassis.mass();
        let gravity_len = world.inner.gravity.length();
        let wheels = vehicle.controller.wheels();

        let Some(tire_model) = world.inner.tire_models.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "tire model not found");
            return Bool::FALSE;
        };

        let wheel_share = 1.0 / tire_model.tires.len().max(1) as f64;
        for tire in &mut tire_model.tires {
            let Some(wheel) = wheels.get(tire.wheel_index as usize) else {
                continue;
            };
            let rc = wheel.raycast_info();

            // Normal load: the real suspension force while grounded, an equal
            // static share of the chassis weight while airborne.
            tire.normal_load = if rc.is_in_contact && wheel.wheel_suspension_force > 0.0 {
                wheel.wheel_suspension_force
            } else {
                chassis_mass * gravity_len * wheel_share
            };

            // Contact-patch velocity from the chassis rigid-body motion.
            let v = chassis.velocity_at_point(wheel.center());
            // Rolling direction on the contact plane, matching the
            // controller's `update_friction` convention (normal × axle). The
            // axle already includes the steering rotation.
            let up = if rc.is_in_contact {
                rc.contact_normal_ws
            } else {
                -wheel.suspension()
            };
            let fwd = up.cross(wheel.axle()).normalize_or_zero();
            let v_fwd = v.dot(fwd);
            let v_lat = v.dot(wheel.axle());

            let r = wheel.radius.max(1e-6);
            let p = &tire.params;
            // Peak friction scaled to the current load, expressed as a *force*
            // (see `load_sensitivity_factor`).
            let peak_long = p.peak_mu_long
                * tire.normal_load.max(0.0)
                * load_sensitivity_factor(tire.normal_load, p.load_sensitivity);
            let peak_lat = p.peak_mu_lat
                * tire.normal_load.max(0.0)
                * load_sensitivity_factor(tire.normal_load, p.load_sensitivity);

            // ── Longitudinal slip: wheel spin state ──
            // Engine force acts at the contact patch (torque F·r); the ground
            // reaction (last fx) spins the wheel back. The brake is a torque
            // applied over `dt` → Δω = (J·r / I)·dt, clamped so the wheel locks
            // (ω = 0) instead of reversing.
            let wheel_mass = chassis_mass * WHEEL_MASS_FRACTION;
            let inertia = (0.5 * wheel_mass * r * r).max(1e-9);
            let drive_torque = wheel.engine_force * r;
            tire.wheel_omega += (drive_torque - tire.fx * r) / inertia * dt;
            // The `dt` factor used to be missing here while the drive term above
            // had it, so the same `brake` value decelerated the wheel 4× harder
            // at 120 Hz than at 30 Hz — braking strength depended on the host's
            // call rate rather than on time.
            let brake_domega = wheel.brake * r / inertia * dt;
            tire.wheel_omega -=
                tire.wheel_omega.signum() * brake_domega.min(tire.wheel_omega.abs());

            let v_surface = tire.wheel_omega * r;
            tire.slip_ratio = (v_surface - v_fwd) / v_fwd.abs().max(MIN_REF_SPEED);
            // Lateral slip angle: sideways contact-point velocity relative to
            // the rolling speed.
            tire.slip_angle = (v_lat / v_fwd.abs().max(MIN_REF_SPEED)).atan();

            // ── Pacejka-style force curves ──
            // Longitudinal: slip ratio drives the force along `fwd` (a wheel
            // spinning faster than the road pushes the car forward). Lateral:
            // slip angle drives a force along the axle that *opposes* the
            // lateral slide (cornering grip).
            //
            // Past the peak the curve saturates towards the sliding-friction
            // level (`SLIDE_FRICTION_FRACTION` of peak) instead of falling off
            // as 1/|slip|. The 1/x form meant a fully locked wheel (maximum
            // slip) produced *zero* braking force and a big slip angle produced
            // zero cornering force — the exact opposite of the physics, and it
            // made a locked brake and a spin unrecoverable.
            let fx_norm = slip_curve(tire.slip_ratio, p.peak_slip_ratio);
            tire.fx = peak_long * fx_norm;

            let fy_norm = -slip_curve(tire.slip_angle, p.peak_slip_angle);
            tire.fy = peak_lat * fy_norm;

            // Friction ellipse: cap the combined force slightly above the
            // peak (ellipse_factor) for realistic drifting saturation.
            let combined = (tire.fx * tire.fx + tire.fy * tire.fy).sqrt();
            let max_force = peak_long * p.ellipse_factor;
            if combined > max_force {
                let scale = max_force / combined;
                tire.fx *= scale;
                tire.fy *= scale;
            }
        }

        clear_error();
        Bool::TRUE
    })
}

/// Get the computed tire forces for a specific wheel.
///
/// # Safety
///
/// `world` must be a valid world pointer. `out_fx` and `out_fy` must be valid pointers.
#[unsafe(no_mangle)]
pub extern "C" fn tire_model_get_forces(
    world: *mut WorldHandle,
    id: u32,
    wheel_index: u32,
    out_fx: *mut f64,
    out_fy: *mut f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        let Some(tire_model) = world.inner.tire_models.get(id) else {
            set_error(ERR_NOT_FOUND, "tire model not found");
            return Bool::FALSE;
        };

        if wheel_index as usize >= tire_model.tires.len() {
            set_error(ERR_INVALID_ARGUMENT, "wheel index out of range");
            return Bool::FALSE;
        }

        if out_fx.is_null() || out_fy.is_null() {
            set_error(ERR_NULL_POINTER, "output pointers are null");
            return Bool::FALSE;
        }

        let tire = &tire_model.tires[wheel_index as usize];
        unsafe {
            *out_fx = tire.fx;
            *out_fy = tire.fy;
        }

        clear_error();
        Bool::TRUE
    })
}

/// Remove a tire model from the world.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn tire_model_remove(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        world.inner.tire_models.remove(id);
        clear_error();
        Bool::TRUE
    })
}
