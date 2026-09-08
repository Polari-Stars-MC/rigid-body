//! `spaceflight::kepler` submodule — orbital-mechanics primitives (Kepler period/semi-major, elements↔state, Lambert, Hohmann, Tsiolkovsky, decay rate, plane change, bi-elliptic, phasing, ballistic coef, equilibrium glide, rendezvous)
//!
//! Split out of the original 2040-line `spaceflight.rs` per OPTIMIZATION.md §N8.
//! See [`super`] for the shared helpers (`finite`, `clamp_unit`,
//! `stumpff_functions`) and numeric constants (`EPS`, `SIGMA`,
//! `SPEED_OF_LIGHT`, `PI`, `TAU`).
//!
//! All public functions keep their `pub fn` names and signatures
//! unchanged; the crate-level `pub use` in `super::mod` keeps the
//! downstream `mps-core::rapier::spaceflight::*` path stable.

use super::*;
use crate::domain::{self, FormulaError};

/// Ballistic coefficient: beta = m / (Cd * A_ref)
pub fn ballistic_coefficient(mass: f64, drag_coefficient: f64, reference_area: f64) -> Option<f64> {
    ballistic_coefficient_checked(mass, drag_coefficient, reference_area).ok()
}

/// Checked ballistic coefficient. All inputs must be finite and positive;
/// zero drag or area is rejected instead of returning an infinite coefficient.
pub fn ballistic_coefficient_checked(
    mass: f64,
    drag_coefficient: f64,
    reference_area: f64,
) -> Result<f64, FormulaError> {
    domain::positive(mass, "mass")?;
    domain::positive(drag_coefficient, "drag_coefficient")?;
    domain::positive(reference_area, "reference_area")?;
    let denominator = domain::finite_result(drag_coefficient * reference_area, "drag area")?;
    domain::divide(mass, denominator)
}

/// Bi-elliptic transfer total delta-V for triple-impulse maneuver.
/// Efficient when r2/r1 > 11.94 for coplanar transfers.
pub fn bi_elliptic_transfer_delta_v(
    mu: f64,
    r1: f64,
    r2: f64,
    r_intermediate: f64,
) -> Option<(f64, f64, f64, f64)> {
    if !finite(&[mu, r1, r2, r_intermediate])
        || mu <= 0.0
        || r1 <= 0.0
        || r2 <= 0.0
        || r_intermediate <= 0.0
    {
        return None;
    }
    let vc1 = (mu / r1).sqrt();
    let vc2 = (mu / r2).sqrt();
    let a1 = 0.5 * (r1 + r_intermediate);
    let a2 = 0.5 * (r_intermediate + r2);
    let vp1 = (mu * (2.0 / r1 - 1.0 / a1)).sqrt();
    let va1 = (mu * (2.0 / r_intermediate - 1.0 / a1)).sqrt();
    let vp2 = (mu * (2.0 / r_intermediate - 1.0 / a2)).sqrt();
    let va2 = (mu * (2.0 / r2 - 1.0 / a2)).sqrt();
    let dv1 = vp1 - vc1;
    let dv2 = vp2 - va1;
    let dv3 = vc2 - va2;
    Some((dv1, dv2, dv3, dv1.abs() + dv2.abs() + dv3.abs()))
}

pub fn combined_maneuver_delta_v(
    mu: f64,
    r1: f64,
    r2: f64,
    inclination_change: f64,
) -> Option<(f64, f64, f64)> {
    if !finite(&[mu, r1, r2, inclination_change])
        || mu <= 0.0
        || r1 <= 0.0
        || r2 <= 0.0
        || inclination_change < 0.0
    {
        return None;
    }
    let v1 = (mu / r1).sqrt();
    let v2 = (mu / r2).sqrt();
    let at = 0.5 * (r1 + r2);
    let vp = (mu * (2.0 / r1 - 1.0 / at)).sqrt();
    let va = (mu * (2.0 / r2 - 1.0 / at)).sqrt();
    let dv1 = vp - v1;
    let dv_plane = (va * va + v2 * v2 - 2.0 * va * v2 * inclination_change.cos()).sqrt();
    Some((dv1, dv_plane, dv1.abs() + dv_plane))
}

pub fn elements_to_state(elements: OrbitalElements, mu: f64) -> Option<StateVector> {
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
        return None;
    }

    let a = elements.semi_major_axis;
    let e = elements.eccentricity;
    let i = elements.inclination;
    let raan = elements.raan;
    let argp = elements.argument_of_periapsis;
    let nu = elements.true_anomaly;
    let p = a * (1.0 - e * e);
    if p <= 0.0 {
        return None;
    }

    let r = p / (1.0 + e * nu.cos());
    let r_pf = vec3_to_rapier(Vec3 {
        x: r * nu.cos(),
        y: r * nu.sin(),
        z: 0.0,
    });
    let v_pf = vec3_to_rapier(Vec3 {
        x: -(mu / p).sqrt() * nu.sin(),
        y: (mu / p).sqrt() * (e + nu.cos()),
        z: 0.0,
    });

    let (so, co) = raan.sin_cos();
    let (si, ci) = i.sin_cos();
    let (sw, cw) = argp.sin_cos();
    let rotate = |v: crate::math::Vector3f64| -> crate::math::Vector3f64 {
        crate::math::Vector3f64::new(
            (co * cw - so * sw * ci) * v.x + (-co * sw - so * cw * ci) * v.y,
            (so * cw + co * sw * ci) * v.x + (-so * sw + co * cw * ci) * v.y,
            (sw * si) * v.x + (cw * si) * v.y,
        )
    };

    Some(StateVector {
        position: vec3_from_rapier(rotate(r_pf)),
        velocity: vec3_from_rapier(rotate(v_pf)),
    })
}

/// Equilibrium glide condition: L/D ratio needed to maintain altitude.
pub fn equilibrium_glide_ld(mu: f64, radius: f64, velocity: f64) -> Option<f64> {
    if !finite(&[mu, radius, velocity]) || mu <= 0.0 || radius <= 0.0 || velocity <= 0.0 {
        return None;
    }
    let g = mu / (radius * radius);
    let centripetal = velocity * velocity / radius;
    if g <= centripetal {
        return None;
    }
    Some((g - centripetal) / g)
}

pub fn hohmann_transfer(mu: f64, radius1: f64, radius2: f64) -> Option<HohmannTransfer> {
    if !finite(&[mu, radius1, radius2]) || mu <= 0.0 || radius1 <= 0.0 || radius2 <= 0.0 {
        return None;
    }
    let transfer_a = 0.5 * (radius1 + radius2);
    let circular1 = (mu / radius1).sqrt();
    let circular2 = (mu / radius2).sqrt();
    let transfer_periapsis = (mu * (2.0 / radius1 - 1.0 / transfer_a)).sqrt();
    let transfer_apoapsis = (mu * (2.0 / radius2 - 1.0 / transfer_a)).sqrt();
    let delta_v1 = transfer_periapsis - circular1;
    let delta_v2 = circular2 - transfer_apoapsis;
    Some(HohmannTransfer {
        delta_v1,
        delta_v2,
        total_delta_v: delta_v1.abs() + delta_v2.abs(),
        transfer_time: PI * (transfer_a.powi(3) / mu).sqrt(),
    })
}

pub fn kepler_period(mu: f64, semi_major_axis: f64) -> Option<f64> {
    kepler_period_checked(mu, semi_major_axis).ok()
}

/// Checked orbital period for a bound ellipse. Inputs must be finite and
/// positive; non-finite intermediate values or periods are rejected.
pub fn kepler_period_checked(mu: f64, semi_major_axis: f64) -> Result<f64, FormulaError> {
    domain::positive(mu, "mu")?;
    domain::positive(semi_major_axis, "semi_major_axis")?;
    let cube = domain::finite_result(semi_major_axis.powi(3), "semi-major axis cubed")?;
    let period = TAU * domain::sqrt(domain::divide(cube, mu)?)?;
    domain::finite_result(period, "orbital period")
}

pub fn kepler_semi_major_axis(mu: f64, period: f64) -> Option<f64> {
    if !finite(&[mu, period]) || mu <= 0.0 || period <= 0.0 {
        return None;
    }
    Some((mu * (period / TAU).powi(2)).cbrt())
}

pub fn lambert_time_elliptic(
    mu: f64,
    semi_major_axis: f64,
    alpha: f64,
    beta: f64,
    revolutions: u32,
) -> Option<f64> {
    if !finite(&[mu, semi_major_axis, alpha, beta]) || mu <= 0.0 || semi_major_axis <= 0.0 {
        return None;
    }
    let m = revolutions as f64;
    Some(
        (semi_major_axis.powi(3) / mu).sqrt()
            * ((alpha - alpha.sin()) - (beta - beta.sin()) + TAU * m),
    )
}

/// Lambert universal variable solver. Returns (v1, v2) velocity vectors.
pub fn lambert_universal_variable(
    r1: Vec3,
    r2: Vec3,
    delta_t: f64,
    mu: f64,
    prograde: bool,
) -> Option<(Vec3, Vec3)> {
    if !vec3_finite(r1)
        || !vec3_finite(r2)
        || !delta_t.is_finite()
        || delta_t <= 0.0
        || !mu.is_finite()
        || mu <= 0.0
    {
        return None;
    }
    let r1v = vec3_to_rapier(r1);
    let r2v = vec3_to_rapier(r2);
    let r1m = r1v.length();
    let r2m = r2v.length();
    if r1m < EPS || r2m < EPS {
        return None;
    }
    let cos_dnu = (r1v.dot(r2v) / (r1m * r2m)).clamp(-1.0, 1.0);
    let dnu = if prograde {
        cos_dnu.acos()
    } else {
        TAU - cos_dnu.acos()
    };
    let a = dnu.sin() * (r1m * r2m / (1.0 - cos_dnu)).sqrt();
    if a < EPS {
        return None;
    }
    let c = (r1m + r2m) / 2.0;
    let mut x = (1.0 - dnu / TAU) * PI;
    if dnu > PI {
        x = -(1.0 - (TAU - dnu) / TAU) * PI;
    }
    for _ in 0..50 {
        let z = x * x / a;
        let (c2, c3) = stumpff_functions(z);
        let y_new = r1m + r2m + a * (z * c3 - 1.0) / c2.sqrt();
        let x_new = (y_new / c).powf(1.5) * c2 * mu.sqrt() * delta_t / (r1m * r2m * dnu.sin()) + x;
        if (x_new - x).abs() < 1.0e-8 {
            x = x_new;
            break;
        }
        x = x_new;
    }
    let z = x * x / a;
    let (c2, c3) = stumpff_functions(z);
    let yf = r1m + r2m + a * (z * c3 - 1.0) / c2.sqrt();
    let f = 1.0 - yf / r1m;
    let g = a * (yf / mu).sqrt();
    let gdot = 1.0 - yf / r2m;
    let v1 = (r2v - r1v * f) / g;
    let v2 = (r2v * gdot - r1v) / g;
    Some((vec3_from_rapier(v1), vec3_from_rapier(v2)))
}

/// Phasing orbit period for rendezvous wait.
pub fn phasing_orbit_semi_major_axis(mu: f64, target_period: f64, phase_angle: f64) -> Option<f64> {
    if !finite(&[mu, target_period, phase_angle]) || mu <= 0.0 || target_period <= 0.0 {
        return None;
    }
    let _n = TAU / target_period;
    let phasing_period = target_period * (1.0 + phase_angle / TAU);
    Some((mu * (phasing_period / TAU).powi(2)).cbrt())
}

pub fn plane_change_delta_v(circular_velocity: f64, inclination_change: f64) -> Option<f64> {
    if !circular_velocity.is_finite()
        || circular_velocity <= 0.0
        || !inclination_change.is_finite()
        || inclination_change < 0.0
    {
        return None;
    }
    Some(2.0 * circular_velocity * (inclination_change / 2.0).sin())
}

/// Orbital rendezvous phasing: phase angle needed for co-planar Hohmann rendezvous.
pub fn rendezvous_phase_angle(mu: f64, r_chaser: f64, r_target: f64) -> Option<f64> {
    if !finite(&[mu, r_chaser, r_target]) || mu <= 0.0 || r_chaser <= 0.0 || r_target <= 0.0 {
        return None;
    }
    let n_target = (mu / (r_target * r_target * r_target)).sqrt();
    let transfer_a = 0.5 * (r_chaser + r_target);
    let transfer_period = TAU * (transfer_a * transfer_a * transfer_a / mu).sqrt();
    Some(PI - n_target * transfer_period * 0.5)
}

pub fn semi_major_axis_decay_rate(
    semi_major_axis: f64,
    density: f64,
    drag_coefficient: f64,
    area: f64,
    mass: f64,
    mu: f64,
) -> Option<f64> {
    semi_major_axis_decay_rate_checked(semi_major_axis, density, drag_coefficient, area, mass, mu)
        .ok()
}

/// Checked atmospheric decay rate. Density, drag coefficient and area may
/// be zero, but cannot be negative. Axis, mass and mu must be positive.
/// Every input and the computed rate must be finite.
pub fn semi_major_axis_decay_rate_checked(
    semi_major_axis: f64,
    density: f64,
    drag_coefficient: f64,
    area: f64,
    mass: f64,
    mu: f64,
) -> Result<f64, FormulaError> {
    domain::positive(semi_major_axis, "semi_major_axis")?;
    domain::nonnegative(density, "density")?;
    domain::nonnegative(drag_coefficient, "drag_coefficient")?;
    domain::nonnegative(area, "area")?;
    domain::positive(mass, "mass")?;
    domain::positive(mu, "mu")?;
    if density == 0.0 || drag_coefficient == 0.0 || area == 0.0 {
        return Ok(0.0);
    }
    let v = domain::sqrt(domain::divide(mu, semi_major_axis)?)?;
    domain::finite_result(
        -density * drag_coefficient * area / mass * semi_major_axis * v,
        "decay rate",
    )
}

pub fn state_to_elements(state: StateVector, mu: f64) -> Option<OrbitalElements> {
    if !vec3_finite(state.position) || !vec3_finite(state.velocity) || !mu.is_finite() || mu <= 0.0
    {
        return None;
    }

    let r_vec = vec3_to_rapier(state.position);
    let v_vec = vec3_to_rapier(state.velocity);
    let r = r_vec.length();
    let v2 = v_vec.length_squared();
    if r <= EPS {
        return None;
    }

    let h_vec = r_vec.cross(v_vec);
    let h = h_vec.length();
    if h <= EPS {
        return None;
    }
    let n_vec = crate::math::Vector3f64::new(0.0, 0.0, 1.0).cross(h_vec);
    let n = n_vec.length();
    let e_vec = v_vec.cross(h_vec) / mu - r_vec / r;
    let e = e_vec.length();
    let energy = 0.5 * v2 - mu / r;
    if energy.abs() <= EPS {
        return None;
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

    Some(OrbitalElements {
        semi_major_axis: a,
        eccentricity: e,
        inclination,
        raan,
        argument_of_periapsis,
        true_anomaly,
    })
}

pub fn tsiolkovsky_delta_v(
    specific_impulse: f64,
    standard_gravity: f64,
    initial_mass: f64,
    final_mass: f64,
) -> Option<f64> {
    tsiolkovsky_delta_v_checked(specific_impulse, standard_gravity, initial_mass, final_mass).ok()
}

/// Checked rocket equation. Inputs must be finite and positive, and initial
/// mass must be at least final mass. Equal masses yield zero delta-v; zero
/// final mass is rejected rather than treated as an infinite delta-v limit.
pub fn tsiolkovsky_delta_v_checked(
    specific_impulse: f64,
    standard_gravity: f64,
    initial_mass: f64,
    final_mass: f64,
) -> Result<f64, FormulaError> {
    domain::positive(specific_impulse, "specific_impulse")?;
    domain::positive(standard_gravity, "standard_gravity")?;
    domain::positive(initial_mass, "initial_mass")?;
    domain::positive(final_mass, "final_mass")?;
    if initial_mass < final_mass {
        return Err(FormulaError::OutOfDomain {
            parameter: "initial_mass",
            requirement: "at least final_mass",
        });
    }
    if initial_mass == final_mass {
        return Ok(0.0);
    }
    let ratio = domain::divide(initial_mass, final_mass)?;
    domain::finite_result(
        specific_impulse * standard_gravity * domain::ln(ratio)?,
        "delta-v",
    )
}

// ---------------------------------------------------------------------------
// Constellation geometry (PHYSICS_EXPANSION_PLAN.md W3)
// ---------------------------------------------------------------------------

/// Walker delta constellation geometry — total satellites `t`, orbital planes
/// `p`, phasing parameter `f`, returns the `(RAAN_deg, mean_anomaly_deg)` pair
/// for the 0-based `idx`-th satellite in the shell.
///
/// Constraints enforced (return `None` on violation with a prior `set_error`
/// call):
/// - `t > 0`, `p > 0`, `p ≤ t`, `f < p`, `idx < t`
/// - `t` must be exactly divisible by `p` (so each plane holds `s = t/p`
///   satellites evenly)
///
/// Convention: RAAN spacing between planes is `360°/p`; in-plane mean-anomaly
/// spacing is `360°/s`; the phasing offset for a Walker delta scales with the
/// plane index: `plane · f · 360°/t` (the relative geometry between planes is
/// what `f` controls).
///
/// Example: GPS Block II "24/3/2" shell → idx=17 gives (240°, 105°).
pub fn walker_delta_layout(t: u32, p: u32, f: u32, idx: u32) -> Option<(f64, f64)> {
    if t == 0 || p == 0 || p > t || f >= p || idx >= t {
        return None;
    }
    let s = t / p;
    if s * p != t {
        return None; // t must divide p exactly
    }
    let plane = idx / s;
    let pos = idx % s;
    let plane_spacing = 360.0 / p as f64;
    let in_plane_spacing = 360.0 / s as f64;
    let raan = (plane as f64) * plane_spacing;
    // Walker delta phasing: each plane is offset by f·(360°/t) relative to
    // the previous one, so the offset scales with the plane index.
    let phasing_offset = (plane as f64) * (f as f64) * 360.0 / (t as f64);
    let mean_anomaly = (pos as f64) * in_plane_spacing + phasing_offset;
    Some((raan % 360.0, mean_anomaly % 360.0))
}

/// Sun-synchronous orbit (SSO) inclination from orbital altitude — simplified
/// closed-form for near-circular low-Earth orbits.  Returns the inclination in
/// degrees needed for the J2 precession rate
/// `Ω̇ = -(3/2) · n · J2 · (R_E/a)² · cos(i)` to match the requested rate
/// (one full rotation per year is 360°/365.25 d ≈ 0.9856 deg/day ≈ 1.991e-7
/// rad/s).
///
/// Inputs:
/// - `radius_earth_km` — central body's equatorial radius [km]
/// - `altitude_km`     — mean orbital altitude above that radius [km] (≥ 0)
/// - `mu_km3_s2`       — gravitational parameter μ [km³/s²]
/// - `raan_precession_rate_rad_s` — desired RAAN precession rate [rad/s]
///   (+1.991e-7 for "one revolution per year"; negative for dusk-dawn)
///
/// Returns `i_deg` in `[0°, 180°]`.  For typical 600 km LEO SSO, i ≈ 97.9°.
pub fn sun_synchronous_inclination(
    radius_earth_km: f64,
    altitude_km: f64,
    mu_km3_s2: f64,
    raan_precession_rate_rad_s: f64,
) -> Option<f64> {
    const J2: f64 = 1.082626173e-3;
    let a = altitude_km + radius_earth_km;
    if !finite(&[
        radius_earth_km,
        altitude_km,
        mu_km3_s2,
        raan_precession_rate_rad_s,
    ]) || radius_earth_km <= 0.0
        || altitude_km < 0.0
        || mu_km3_s2 <= 0.0
        || raan_precession_rate_rad_s == 0.0
        || a <= 0.0
    {
        return None;
    }
    // mean motion n = sqrt(μ/a³)
    let n = (mu_km3_s2 / (a * a * a)).sqrt();
    // precession rate = -(3/2)·J2·n·(R_E/a)²·cos(i)
    // ⇒ cos(i) = -dot_Ω / ((3/2)·J2·n·(R_E/a)²)
    let denom = 1.5 * J2 * n * (radius_earth_km / a).powi(2);
    if denom == 0.0 {
        return None;
    }
    let cos_i = -raan_precession_rate_rad_s / denom;
    if !(-1.0..=1.0).contains(&cos_i) {
        return None; // no real inclination exists for this combination
    }
    Some(cos_i.acos().to_degrees())
}

/// Molniya-orbit critical argument-of-perigee: for a 12-hour high-eccentricity
/// orbit with 63.4° inclination, the critical ω = 270° places apogee over the
/// northern hemisphere (maximising dwell time over high-latitude coverage).
/// This helper returns the critical inclination in degrees plus the
/// conventional argument-of-perigee (270°) as a tuple; it exists so callers
/// can refer to the design point without magic numbers.
pub fn molniya_critical_elements() -> (f64, f64) {
    (63.4, 270.0)
}
