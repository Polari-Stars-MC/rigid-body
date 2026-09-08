//! `spaceflight::kepler` submodule — orbital-mechanics primitives (Kepler period/semi-major, elements↔state, Lambert, Hohmann, Tsiolkovsky, decay rate)
//!
//! Split out of the original 2610-line `spaceflight.rs`. See [`super`]
//! for the shared helpers (`finite`, `write_out`, `invalid_nan`, `cross`, `clamp_unit`)
//! and numeric constants (`EPS`, `SIGMA`, `SPEED_OF_LIGHT`, `PI/TAU`).
//! Every `extern "C" fn space_*` in this file retains its
//! `#[unsafe(no_mangle)]` name, signature, and behaviour — the crate-level
//! `pub use` in `super::mod` keeps ABI paths stable.

use super::*;

/// # Safety
/// `out_state` must be null or point to a valid, writable `StateVector`.
#[unsafe(no_mangle)]
pub extern "C" fn space_elements_to_state(
    elements: OrbitalElements,
    mu: f64,
    out_state: *mut StateVector,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        if !finite(&[
            elements.semi_major_axis,
            elements.eccentricity,
            elements.inclination,
            elements.raan,
            elements.argument_of_periapsis,
            elements.true_anomaly,
            mu,
        ]) || mu <= 0.0
            || elements.semi_major_axis <= 0.0
            || elements.eccentricity < 0.0
            || elements.eccentricity >= 1.0
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid orbital elements");
            return Bool::FALSE;
        }

        let a = elements.semi_major_axis;
        let e = elements.eccentricity;
        let i = elements.inclination;
        let raan = elements.raan;
        let argp = elements.argument_of_periapsis;
        let nu = elements.true_anomaly;
        let p = a * (1.0 - e * e);
        if p <= 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid orbital semi-latus rectum");
            return Bool::FALSE;
        }

        let r = p / (1.0 + e * nu.cos());
        let r_pf = Vector::new(r * nu.cos(), r * nu.sin(), 0.0);
        let v_pf = Vector::new(
            -(mu / p).sqrt() * nu.sin(),
            (mu / p).sqrt() * (e + nu.cos()),
            0.0,
        );

        let (so, co) = raan.sin_cos();
        let (si, ci) = i.sin_cos();
        let (sw, cw) = argp.sin_cos();
        let rotate = |v: Vector| -> Vector {
            Vector::new(
                (co * cw - so * sw * ci) * v.x + (-co * sw - so * cw * ci) * v.y,
                (so * cw + co * sw * ci) * v.x + (-so * sw + co * cw * ci) * v.y,
                (sw * si) * v.x + (cw * si) * v.y,
            )
        };

        write_out(
            out_state,
            StateVector {
                position: vec3_from_rapier(rotate(r_pf)),
                velocity: vec3_from_rapier(rotate(v_pf)),
            },
        )
    })
}

/// # Safety
/// `out_transfer` must be null or point to a valid, writable `HohmannTransfer`.
#[unsafe(no_mangle)]
pub extern "C" fn space_hohmann_transfer(
    mu: f64,
    radius1: f64,
    radius2: f64,
    out_transfer: *mut HohmannTransfer,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        if !finite(&[mu, radius1, radius2]) || mu <= 0.0 || radius1 <= 0.0 || radius2 <= 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid Hohmann transfer parameters");
            return Bool::FALSE;
        }
        let transfer_a = 0.5 * (radius1 + radius2);
        let circular1 = (mu / radius1).sqrt();
        let circular2 = (mu / radius2).sqrt();
        let transfer_periapsis = (mu * (2.0 / radius1 - 1.0 / transfer_a)).sqrt();
        let transfer_apoapsis = (mu * (2.0 / radius2 - 1.0 / transfer_a)).sqrt();
        let delta_v1 = transfer_periapsis - circular1;
        let delta_v2 = circular2 - transfer_apoapsis;
        write_out(
            out_transfer,
            HohmannTransfer {
                delta_v1,
                delta_v2,
                total_delta_v: delta_v1.abs() + delta_v2.abs(),
                transfer_time: PI * (transfer_a.powi(3) / mu).sqrt(),
            },
        )
    })
}

/// Computes the orbital period from the gravitational parameter and semi-major axis
/// (Kepler's third law).
///
/// # Safety
/// This function takes no pointers and transfers no ownership; it is safe to call with
/// any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
#[unsafe(no_mangle)]
pub extern "C" fn space_kepler_period(mu: f64, semi_major_axis: f64) -> f64 {
    ffi_guard(0.0, || {
        mps_formula::ffi::formula_result(mps_formula::spaceflight::kepler_period_checked(
            mu,
            semi_major_axis,
        ))
        .unwrap_or(f64::NAN)
    })
}

/// Computes the semi-major axis from the gravitational parameter and orbital period.
///
/// # Safety
/// This function takes no pointers and transfers no ownership; it is safe to call with
/// any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
#[unsafe(no_mangle)]
pub extern "C" fn space_kepler_semi_major_axis(mu: f64, period: f64) -> f64 {
    ffi_guard(0.0, || {
        if !finite(&[mu, period]) || mu <= 0.0 || period <= 0.0 {
            return invalid_nan("invalid Kepler semi-major-axis parameters");
        }
        clear_error();
        (mu * (period / TAU).powi(2)).cbrt()
    })
}

/// Computes the time of flight for an elliptic Lambert arc.
///
/// # Safety
/// This function takes no pointers and transfers no ownership; it is safe to call with
/// any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
#[unsafe(no_mangle)]
pub extern "C" fn space_lambert_time_elliptic(
    mu: f64,
    semi_major_axis: f64,
    alpha: f64,
    beta: f64,
    revolutions: u32,
) -> f64 {
    ffi_guard(0.0, || {
        if !finite(&[mu, semi_major_axis, alpha, beta]) || mu <= 0.0 || semi_major_axis <= 0.0 {
            return invalid_nan("invalid Lambert time parameters");
        }
        clear_error();
        let m = revolutions as f64;
        (semi_major_axis.powi(3) / mu).sqrt()
            * ((alpha - alpha.sin()) - (beta - beta.sin()) + TAU * m)
    })
}

/// Computes the semi-major axis decay rate due to atmospheric drag.
///
/// # Safety
/// This function takes no pointers and transfers no ownership; it is safe to call with
/// any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
#[unsafe(no_mangle)]
pub extern "C" fn space_semi_major_axis_decay_rate(
    semi_major_axis: f64,
    density: f64,
    drag_coefficient: f64,
    area: f64,
    mass: f64,
    mu: f64,
) -> f64 {
    ffi_guard(0.0, || {
        mps_formula::ffi::formula_result(
            mps_formula::spaceflight::semi_major_axis_decay_rate_checked(
                semi_major_axis,
                density,
                drag_coefficient,
                area,
                mass,
                mu,
            ),
        )
        .unwrap_or(f64::NAN)
    })
}

/// # Safety
/// `out_elements` must be null or point to a valid, writable `OrbitalElements`.
#[unsafe(no_mangle)]
pub extern "C" fn space_state_to_elements(
    state: StateVector,
    mu: f64,
    out_elements: *mut OrbitalElements,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        if !vec3_finite(state.position)
            || !vec3_finite(state.velocity)
            || !mu.is_finite()
            || mu <= 0.0
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid state vector");
            return Bool::FALSE;
        }

        let r_vec = vec3_to_rapier(state.position);
        let v_vec = vec3_to_rapier(state.velocity);
        let r = r_vec.length();
        let v2 = v_vec.length_squared();
        if r <= EPS {
            set_error(ERR_INVALID_ARGUMENT, "position magnitude is zero");
            return Bool::FALSE;
        }

        let h_vec = cross(r_vec, v_vec);
        let h = h_vec.length();
        if h <= EPS {
            set_error(ERR_INVALID_ARGUMENT, "angular momentum magnitude is zero");
            return Bool::FALSE;
        }
        let n_vec = cross(Vector::Z, h_vec);
        let n = n_vec.length();
        let e_vec = cross(v_vec, h_vec) / mu - r_vec / r;
        let e = e_vec.length();
        let energy = 0.5 * v2 - mu / r;
        if energy.abs() <= EPS {
            set_error(ERR_INVALID_ARGUMENT, "parabolic orbit is unsupported");
            return Bool::FALSE;
        }

        let a = -mu / (2.0 * energy);
        let inclination = clamp_unit(h_vec.z / h).acos();
        let raan = if n > EPS {
            n_vec.y.atan2(n_vec.x).rem_euclid(TAU)
        } else {
            0.0
        };
        let argument_of_periapsis = if n > EPS && e > EPS {
            let mut value = clamp_unit(n_vec.dot(e_vec) / (n * e)).acos();
            if e_vec.z < 0.0 {
                value = TAU - value;
            }
            value
        } else {
            0.0
        };
        let true_anomaly = if e > EPS {
            let mut value = clamp_unit(e_vec.dot(r_vec) / (e * r)).acos();
            if r_vec.dot(v_vec) < 0.0 {
                value = TAU - value;
            }
            value
        } else if n > EPS {
            let mut value = clamp_unit(n_vec.dot(r_vec) / (n * r)).acos();
            if r_vec.z < 0.0 {
                value = TAU - value;
            }
            value
        } else {
            r_vec.y.atan2(r_vec.x).rem_euclid(TAU)
        };

        write_out(
            out_elements,
            OrbitalElements {
                semi_major_axis: a,
                eccentricity: e,
                inclination,
                raan,
                argument_of_periapsis,
                true_anomaly,
            },
        )
    })
}

/// Computes the Tsiolkovsky rocket equation delta-v.
///
/// # Safety
/// This function takes no pointers and transfers no ownership; it is safe to call with
/// any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
#[unsafe(no_mangle)]
pub extern "C" fn space_tsiolkovsky_delta_v(
    specific_impulse: f64,
    standard_gravity: f64,
    initial_mass: f64,
    final_mass: f64,
) -> f64 {
    ffi_guard(0.0, || {
        mps_formula::ffi::formula_result(mps_formula::spaceflight::tsiolkovsky_delta_v_checked(
            specific_impulse,
            standard_gravity,
            initial_mass,
            final_mass,
        ))
        .unwrap_or(f64::NAN)
    })
}
