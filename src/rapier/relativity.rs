//! Relativistic effects: Lorentz transformations, Schwarzschild metric,
//! gravitational time dilation, length contraction, and near-light-speed particle physics.
//!
//! All functions are FFI-exported with C-compatible types, following the
//! error-handling conventions of the mps_rigid_body physics engine.

use rapier3d::prelude::Vector;

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ERR_UNSUPPORTED, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, GravitationalTimeDilation, LengthContraction, LorentzBoost, LorentzTransformedFrame,
    RelativisticParticle, SchwarzschildMetric, Vec3, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};
use crate::rapier::math::mul_add;

use crate::rapier::math::{finite_non_negative, finite_positive};

const SPEED_OF_LIGHT: f64 = 299_792_458.0;
const EPSILON: f64 = 1.0e-12;

fn write_out<T: Copy>(out: *mut T, value: T) -> Bool {
    let Some(out) = (unsafe { out.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return Bool::FALSE;
    };
    *out = value;
    clear_error();
    Bool::TRUE
}

// ---------------------------------------------------------------------------
// A. Lorentz factor and kinematics
// ---------------------------------------------------------------------------

/// Compute the Lorentz factor gamma = 1/sqrt(1 - v^2/c^2).
#[unsafe(no_mangle)]
pub extern "C" fn rel_lorentz_factor(speed: f64, out_gamma: *mut f64) -> Bool {
    if !finite_non_negative(speed) {
        set_error(ERR_INVALID_ARGUMENT, "speed must be finite and non-negative");
        return Bool::FALSE;
    }
    if speed >= SPEED_OF_LIGHT - EPSILON {
        set_error(ERR_INVALID_ARGUMENT, "speed must be less than speed of light");
        return Bool::FALSE;
    }
    let beta = speed / SPEED_OF_LIGHT;
    // Use mul_add to avoid catastrophic cancellation when beta ≈ 1:
    // 1.0 - beta*beta = -(beta*beta - 1.0), computed with single rounding.
    let one_minus_beta_sq = -mul_add(beta, beta, -1.0);
    let gamma = 1.0 / one_minus_beta_sq.max(0.0).sqrt();
    write_out(out_gamma, gamma)
}

/// Build the full 4x4 Lorentz boost matrix for a given velocity 3-vector.
///
/// The matrix acts on column 4-vectors (ct, x, y, z)^T.
#[unsafe(no_mangle)]
pub extern "C" fn rel_lorentz_boost(velocity: Vec3, out_boost: *mut LorentzBoost) -> Bool {
    if !vec3_finite(velocity) {
        set_error(ERR_INVALID_ARGUMENT, "velocity must be finite");
        return Bool::FALSE;
    }
    let v = vec3_to_rapier(velocity);
    let v_sq = v.length_squared();
    let c2 = SPEED_OF_LIGHT * SPEED_OF_LIGHT;

    if v_sq >= c2 - EPSILON {
        set_error(
            ERR_INVALID_ARGUMENT,
            "velocity magnitude must be less than c",
        );
        return Bool::FALSE;
    }
    // Identity for negligible velocity
    if v_sq < EPSILON {
        return write_out(
            out_boost,
            LorentzBoost {
                m00: 1.0, m11: 1.0, m22: 1.0, m33: 1.0,
                ..LorentzBoost::default()
            },
        );
    }
    let beta_x = velocity.x / SPEED_OF_LIGHT;
    let beta_y = velocity.y / SPEED_OF_LIGHT;
    let beta_z = velocity.z / SPEED_OF_LIGHT;
    // Sum beta components with mul_add for better precision near c
    let beta_sq = mul_add(beta_x, beta_x, mul_add(beta_y, beta_y, beta_z * beta_z));
    let one_minus_beta_sq = -mul_add(beta_sq, 1.0_f64, -1.0_f64); // -(β² - 1) = 1 - β²
    let gamma = if one_minus_beta_sq > 0.0 {
        1.0 / one_minus_beta_sq.sqrt()
    } else {
        f64::INFINITY
    };
    let g = gamma;
    let gm1_over_b2 = if beta_sq > 0.0 {
        (gamma - 1.0) / beta_sq
    } else {
        0.5 // limit as beta→0: (γ-1)/β² → 0.5
    };

    write_out(
        out_boost,
        LorentzBoost {
            m00:  g,
            m01: -g * beta_x, m02: -g * beta_y, m03: -g * beta_z,
            m10: -g * beta_x,
            m11: 1.0 + gm1_over_b2 * beta_x * beta_x,
            m12: gm1_over_b2 * beta_x * beta_y,
            m13: gm1_over_b2 * beta_x * beta_z,
            m20: -g * beta_y,
            m21: gm1_over_b2 * beta_y * beta_x,
            m22: 1.0 + gm1_over_b2 * beta_y * beta_y,
            m23: gm1_over_b2 * beta_y * beta_z,
            m30: -g * beta_z,
            m31: gm1_over_b2 * beta_z * beta_x,
            m32: gm1_over_b2 * beta_z * beta_y,
            m33: 1.0 + gm1_over_b2 * beta_z * beta_z,
        },
    )
}

/// Apply a Lorentz boost to a 4-vector (ct, x, y, z).
#[unsafe(no_mangle)]
pub extern "C" fn rel_transform_four_vector(
    boost: LorentzBoost,
    ct: f64,
    x: f64,
    y: f64,
    z: f64,
    out_transformed: *mut LorentzTransformedFrame,
) -> Bool {
    if !ct.is_finite() || ct <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "ct must be finite and non-negative");
        return Bool::FALSE;
    }
    if !x.is_finite() || !y.is_finite() || !z.is_finite() {
        set_error(ERR_INVALID_ARGUMENT, "spatial components must be finite");
        return Bool::FALSE;
    }
    write_out(
        out_transformed,
        LorentzTransformedFrame {
            ct_prime: boost.m00 * ct + boost.m01 * x + boost.m02 * y + boost.m03 * z,
            x_prime:  boost.m10 * ct + boost.m11 * x + boost.m12 * y + boost.m13 * z,
            y_prime:  boost.m20 * ct + boost.m21 * x + boost.m22 * y + boost.m23 * z,
            z_prime:  boost.m30 * ct + boost.m31 * x + boost.m32 * y + boost.m33 * z,
        },
    )
}

/// Relativistic velocity addition (3D general formula).
///
/// w = (u + v_∥ + v_⊥/γ_u) / (1 + u·v/c²)
#[unsafe(no_mangle)]
pub extern "C" fn rel_velocity_addition(
    u: Vec3,
    v: Vec3,
    out_result: *mut Vec3,
) -> Bool {
    if !vec3_finite(u) || !vec3_finite(v) {
        set_error(ERR_INVALID_ARGUMENT, "velocities must be finite");
        return Bool::FALSE;
    }
    let u_vec = vec3_to_rapier(u);
    let v_vec = vec3_to_rapier(v);
    let u_len_sq = u_vec.length_squared();
    let c2 = SPEED_OF_LIGHT * SPEED_OF_LIGHT;
    if u_len_sq >= c2 - EPSILON {
        set_error(ERR_INVALID_ARGUMENT, "velocity u magnitude must be less than c");
        return Bool::FALSE;
    }
    if v_vec.length_squared() >= c2 - EPSILON {
        set_error(ERR_INVALID_ARGUMENT, "velocity v magnitude must be less than c");
        return Bool::FALSE;
    }
    let denom = 1.0 + u_vec.dot(v_vec) / c2;
    if denom.abs() < EPSILON {
        set_error(ERR_INVALID_ARGUMENT, "velocity addition denominator is zero");
        return Bool::FALSE;
    }
    // Decompose v into parallel and perpendicular components relative to u
    let result = if u_len_sq < EPSILON {
        // u ≈ 0 → w ≈ v
        v_vec
    } else {
        let u_len = u_len_sq.sqrt();
        let u_hat = u_vec / u_len;
        let v_parallel = u_hat * u_hat.dot(v_vec);
        let v_perp = v_vec - v_parallel;
        let one_minus_u = -mul_add(u_len_sq / c2, 1.0_f64, -1.0_f64);
        let gamma_u = 1.0 / one_minus_u.max(0.0).sqrt();
        (u_vec + v_parallel + v_perp / gamma_u) / denom
    };
    write_out(out_result, vec3_from_rapier(result))
}

/// Rapidity = arctanh(v/c).
#[unsafe(no_mangle)]
pub extern "C" fn rel_rapidity(speed: f64) -> f64 {
    if !finite_non_negative(speed) {
        set_error(ERR_INVALID_ARGUMENT, "speed must be finite and non-negative");
        return f64::NAN;
    }
    if speed >= SPEED_OF_LIGHT - EPSILON {
        set_error(ERR_INVALID_ARGUMENT, "speed must be less than c");
        return f64::NAN;
    }
    clear_error();
    let beta = speed / SPEED_OF_LIGHT;
    0.5 * ((1.0 + beta) / (1.0 - beta)).ln()
}

/// Beta (v/c) from Lorentz factor: beta = sqrt(1 - 1/gamma^2).
#[unsafe(no_mangle)]
pub extern "C" fn rel_beta_from_gamma(gamma: f64) -> f64 {
    if !gamma.is_finite() || gamma < 1.0 {
        set_error(ERR_INVALID_ARGUMENT, "gamma must be >= 1");
        return f64::NAN;
    }
    clear_error();
    (1.0 - 1.0 / (gamma * gamma)).sqrt()
}

/// Return the speed of light constant.
#[unsafe(no_mangle)]
pub extern "C" fn rel_speed_of_light() -> f64 {
    SPEED_OF_LIGHT
}

// ---------------------------------------------------------------------------
// B. Schwarzschild metric
// ---------------------------------------------------------------------------

/// Compute the Schwarzschild radius rs = 2GM/c^2.
#[unsafe(no_mangle)]
pub extern "C" fn rel_schwarzschild_radius(mass: f64, gravitational_constant: f64) -> f64 {
    if !finite_positive(mass) || !finite_positive(gravitational_constant) {
        set_error(ERR_INVALID_ARGUMENT, "mass and gravitational constant must be positive");
        return f64::NAN;
    }
    clear_error();
    2.0 * gravitational_constant * mass / (SPEED_OF_LIGHT * SPEED_OF_LIGHT)
}

/// Compute the Schwarzschild metric coefficients at a given radius.
#[unsafe(no_mangle)]
pub extern "C" fn rel_schwarzschild_metric(
    radius: f64,
    mass: f64,
    gravitational_constant: f64,
    out_metric: *mut SchwarzschildMetric,
) -> Bool {
    if !finite_positive(radius) || !finite_positive(mass) || !finite_positive(gravitational_constant) {
        set_error(ERR_INVALID_ARGUMENT, "radius, mass, and gravitational constant must be positive");
        return Bool::FALSE;
    }
    let rs = rel_schwarzschild_radius(mass, gravitational_constant);
    if !rs.is_finite() {
        return Bool::FALSE;
    }
    if radius <= rs + EPSILON {
        set_error(
            ERR_INVALID_ARGUMENT,
            "radius must be greater than the Schwarzschild radius",
        );
        return Bool::FALSE;
    }
    // mul_add to avoid cancellation when radius ≈ rs (near horizon)
    let factor = -mul_add(rs / radius, 1.0_f64, -1.0_f64); // -(rs/r - 1) = 1 - rs/r
    let g_tt = -factor;
    let g_rr = 1.0 / factor;
    write_out(
        out_metric,
        SchwarzschildMetric {
            g_tt,
            g_rr,
            schwarzschild_radius: rs,
            radius_over_rs: radius / rs,
        },
    )
}

/// Einstein light deflection angle: delta_phi = 4GM/(b*c^2).
///
/// Returns the deflection angle in radians. Returns ERR_UNSUPPORTED when the
/// impact parameter is close to the photon sphere (b < 2.6 * rs).
#[unsafe(no_mangle)]
pub extern "C" fn rel_light_deflection_angle(
    impact_parameter: f64,
    mass: f64,
    gravitational_constant: f64,
) -> f64 {
    if !finite_positive(impact_parameter) || !finite_positive(mass)
        || !finite_positive(gravitational_constant)
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "impact parameter, mass, and gravitational constant must be positive",
        );
        return f64::NAN;
    }
    let rs = rel_schwarzschild_radius(mass, gravitational_constant);
    if !rs.is_finite() {
        return f64::NAN;
    }
    if impact_parameter < 2.6 * rs {
        set_error(
            ERR_UNSUPPORTED,
            "impact parameter too close to photon sphere (b < 2.6*rs)",
        );
        return f64::NAN;
    }
    clear_error();
    4.0 * gravitational_constant * mass / (impact_parameter * SPEED_OF_LIGHT * SPEED_OF_LIGHT)
}

/// Effective potential for Schwarzschild orbits (per unit mass m of the orbiting body).
///
/// V_eff(r) = -GM/r + L^2/(2*r^2) - G*M*L^2/(c^2*r^3)
///
/// The orbiting body's mass m and angular momentum L are parameters.
#[unsafe(no_mangle)]
pub extern "C" fn rel_effective_potential(
    radius: f64,
    angular_momentum: f64,
    mass: f64,
    gravitational_constant: f64,
    out_potential: *mut f64,
) -> Bool {
    if !finite_positive(radius) || !finite_positive(mass) || !finite_positive(gravitational_constant) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "radius, mass, and gravitational constant must be positive",
        );
        return Bool::FALSE;
    }
    if !finite_non_negative(angular_momentum) {
        set_error(ERR_INVALID_ARGUMENT, "angular momentum must be finite and non-negative");
        return Bool::FALSE;
    }
    let rs = rel_schwarzschild_radius(mass, gravitational_constant);
    if !rs.is_finite() {
        return Bool::FALSE;
    }
    if radius <= rs + EPSILON {
        set_error(
            ERR_INVALID_ARGUMENT,
            "radius must be greater than the Schwarzschild radius",
        );
        return Bool::FALSE;
    }
    let gm = gravitational_constant * mass;
    let newtonian = -gm / radius + 0.5 * angular_momentum * angular_momentum / (radius * radius);
    let gr_correction = -gm * angular_momentum * angular_momentum
        / (SPEED_OF_LIGHT * SPEED_OF_LIGHT * radius.powi(3));
    write_out(out_potential, newtonian + gr_correction)
}

// ---------------------------------------------------------------------------
// C. Gravitational time dilation
// ---------------------------------------------------------------------------

/// Compute gravitational time dilation factors.
///
/// Stationary factor: dtau/dt = sqrt(1 - rs/r)
/// Orbital factor (circular orbit): dtau/dt = sqrt(1 - 3*rs/(2*r))
#[unsafe(no_mangle)]
pub extern "C" fn rel_gravitational_time_dilation(
    radius: f64,
    mass: f64,
    gravitational_constant: f64,
    out_dilation: *mut GravitationalTimeDilation,
) -> Bool {
    if !finite_positive(radius) || !finite_positive(mass) || !finite_positive(gravitational_constant) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "radius, mass, and gravitational constant must be positive",
        );
        return Bool::FALSE;
    }
    let rs = rel_schwarzschild_radius(mass, gravitational_constant);
    if !rs.is_finite() {
        return Bool::FALSE;
    }
    if radius <= rs + EPSILON {
        set_error(
            ERR_INVALID_ARGUMENT,
            "radius must be greater than the Schwarzschild radius",
        );
        return Bool::FALSE;
    }
    let stationary_factor = (1.0 - rs / radius).sqrt();
    // Orbital factor requires r > 1.5*rs (the ISCO for Schwarzschild)
    let min_orbital_radius = 1.5 * rs;
    let (orbital_factor, orbiting_velocity) = if radius > min_orbital_radius + EPSILON {
        let of = (1.0 - 1.5 * rs / radius).sqrt();
        let v = (gravitational_constant * mass / radius).sqrt();
        (of, v)
    } else {
        (f64::NAN, 0.0)
    };
    write_out(
        out_dilation,
        GravitationalTimeDilation {
            stationary_factor,
            orbital_factor,
            orbiting_velocity,
        },
    )
}

/// Lightweight gravitational time dilation: returns sqrt(1 - rs/r) directly.
#[unsafe(no_mangle)]
pub extern "C" fn rel_gravitational_time_dilation_simple(
    radius: f64,
    schwarzschild_radius: f64,
) -> f64 {
    if !finite_positive(radius) || !finite_non_negative(schwarzschild_radius) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "radius and schwarzschild_radius must be positive",
        );
        return f64::NAN;
    }
    if radius <= schwarzschild_radius + EPSILON {
        set_error(
            ERR_INVALID_ARGUMENT,
            "radius must be greater than the Schwarzschild radius",
        );
        return f64::NAN;
    }
    clear_error();
    (1.0 - schwarzschild_radius / radius).sqrt()
}

// ---------------------------------------------------------------------------
// D. Length contraction
// ---------------------------------------------------------------------------

/// Compute length contraction: L = L0 / gamma.
#[unsafe(no_mangle)]
pub extern "C" fn rel_length_contraction(
    proper_length: f64,
    speed: f64,
    out_contraction: *mut LengthContraction,
) -> Bool {
    if !finite_non_negative(proper_length) {
        set_error(ERR_INVALID_ARGUMENT, "proper length must be finite and non-negative");
        return Bool::FALSE;
    }
    if !finite_non_negative(speed) {
        set_error(ERR_INVALID_ARGUMENT, "speed must be finite and non-negative");
        return Bool::FALSE;
    }
    if speed >= SPEED_OF_LIGHT - EPSILON {
        set_error(ERR_INVALID_ARGUMENT, "speed must be less than speed of light");
        return Bool::FALSE;
    }
    let beta = speed / SPEED_OF_LIGHT;
    let one_minus_beta_sq = -mul_add(beta, beta, -1.0);
    let gamma = 1.0 / one_minus_beta_sq.max(0.0).sqrt();
    let contracted = proper_length / gamma;
    write_out(
        out_contraction,
        LengthContraction {
            lorentz_factor: gamma,
            contracted_length: contracted,
            proper_length,
            speed_ratio: beta,
        },
    )
}

// ---------------------------------------------------------------------------
// E. Near-light-speed particle physics
// ---------------------------------------------------------------------------

/// Compute relativistic particle properties.
///
/// For zero mass (photon-like), speed must equal c and the particle has
/// no well-defined gamma from velocity alone — gamma and total energy
/// are returned as INFINITY, and momentum is set to a unit vector scaled
/// by INFINITY (direction only).
#[unsafe(no_mangle)]
pub extern "C" fn rel_particle_properties(
    mass: f64,
    velocity: Vec3,
    out_particle: *mut RelativisticParticle,
) -> Bool {
    if !finite_non_negative(mass) {
        set_error(ERR_INVALID_ARGUMENT, "mass must be finite and non-negative");
        return Bool::FALSE;
    }
    if !vec3_finite(velocity) {
        set_error(ERR_INVALID_ARGUMENT, "velocity must be finite");
        return Bool::FALSE;
    }
    let v_vec = vec3_to_rapier(velocity);
    let speed_sq = v_vec.length_squared();
    let c2 = SPEED_OF_LIGHT * SPEED_OF_LIGHT;

    // Photon / massless particle
    if mass < EPSILON {
        if speed_sq >= c2 - EPSILON {
            let dir = if speed_sq > EPSILON {
                v_vec / speed_sq.sqrt()
            } else {
                v_vec
            };
            return write_out(
                out_particle,
                RelativisticParticle {
                    lorentz_factor: f64::INFINITY,
                    total_energy: f64::INFINITY,
                    kinetic_energy: f64::INFINITY,
                    momentum_magnitude: f64::INFINITY,
                    momentum: vec3_from_rapier(dir * f64::INFINITY),
                    rapidity: f64::INFINITY,
                },
            );
        }
        // Massless but below c → invalid
        set_error(
            ERR_INVALID_ARGUMENT,
            "massless particles must travel at speed c",
        );
        return Bool::FALSE;
    }
    // Massive particle
    if speed_sq >= c2 - EPSILON {
        set_error(
            ERR_INVALID_ARGUMENT,
            "massive particle speed must be less than c",
        );
        return Bool::FALSE;
    }
    let speed = speed_sq.sqrt();
    let beta = speed / SPEED_OF_LIGHT;
    let one_minus_beta_sq = -mul_add(beta, beta, -1.0);
    let gamma = 1.0 / one_minus_beta_sq.max(0.0).sqrt();
    let mc2 = mass * c2;
    let total_energy = gamma * mc2;
    let kinetic_energy = (gamma - 1.0) * mc2;
    let momentum_mag = gamma * mass * speed;
    let momentum = if speed_sq > EPSILON {
        v_vec / speed * momentum_mag
    } else {
        Vector::ZERO
    };
    let rapidity = 0.5 * ((1.0 + beta) / (1.0 - beta)).ln();

    write_out(
        out_particle,
        RelativisticParticle {
            lorentz_factor: gamma,
            total_energy,
            kinetic_energy,
            momentum_magnitude: momentum_mag,
            momentum: vec3_from_rapier(momentum),
            rapidity,
        },
    )
}

/// Compute the invariant (rest) mass from energy and momentum:
///
/// m0 = sqrt(E^2/c^4 - p^2/c^2)
///
/// Returns NAN for tachyonic states (E^2 < p^2 * c^2).
#[unsafe(no_mangle)]
pub extern "C" fn rel_invariant_mass(energy: f64, px: f64, py: f64, pz: f64) -> f64 {
    if !energy.is_finite() || energy < 0.0
        || !px.is_finite()
        || !py.is_finite()
        || !pz.is_finite()
    {
        set_error(ERR_INVALID_ARGUMENT, "energy and momentum must be finite");
        return f64::NAN;
    }
    let c2 = SPEED_OF_LIGHT * SPEED_OF_LIGHT;
    let p_sq = px * px + py * py + pz * pz;
    let e_sq_over_c4 = energy * energy / (c2 * c2);
    let p_sq_over_c2 = p_sq / c2;
    let mass_sq = e_sq_over_c4 - p_sq_over_c2;
    if mass_sq < 0.0 {
        set_error(
            ERR_INVALID_ARGUMENT,
            "tachyonic state: E^2 < p^2 * c^2",
        );
        return f64::NAN;
    }
    clear_error();
    mass_sq.sqrt()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const C: f64 = SPEED_OF_LIGHT;
    const G: f64 = 6.674_30e-11;
    const SOLAR_MASS: f64 = 1.989e30;

    #[test]
    fn lorentz_factor_is_one_at_rest() {
        let mut gamma = 0.0;
        assert_eq!(rel_lorentz_factor(0.0, &mut gamma), Bool::TRUE);
        assert!((gamma - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn lorentz_factor_diverges_near_c() {
        let mut gamma = 0.0;
        let speed = 0.999 * C;
        assert_eq!(rel_lorentz_factor(speed, &mut gamma), Bool::TRUE);
        let expected = 1.0 / (1.0 - 0.999_f64.powi(2)).sqrt();
        assert!((gamma - expected).abs() < 1.0e-6);
        assert!(gamma > 22.0);
    }

    #[test]
    fn lorentz_boost_is_identity_for_zero() {
        let mut boost = LorentzBoost::default();
        assert_eq!(
            rel_lorentz_boost(Vec3::default(), &mut boost),
            Bool::TRUE
        );
        assert!((boost.m00 - 1.0).abs() < 1.0e-12);
        assert!((boost.m11 - 1.0).abs() < 1.0e-12);
        assert!((boost.m22 - 1.0).abs() < 1.0e-12);
        assert!((boost.m33 - 1.0).abs() < 1.0e-12);
        assert!(boost.m01.abs() < 1.0e-12);
        assert!(boost.m10.abs() < 1.0e-12);
    }

    #[test]
    fn transform_four_vector_is_consistent() {
        // Interval invariance: -(ct)^2 + x^2 + y^2 + z^2 = -(ct')^2 + x'^2 + y'^2 + z'^2
        let mut boost = LorentzBoost::default();
        assert_eq!(
            rel_lorentz_boost(
                Vec3 {
                    x: 0.5 * C,
                    y: 0.0,
                    z: 0.0,
                },
                &mut boost,
            ),
            Bool::TRUE
        );
        let mut transformed = LorentzTransformedFrame::default();
        assert_eq!(
            rel_transform_four_vector(boost, 10.0, 3.0, 4.0, 0.0, &mut transformed),
            Bool::TRUE
        );
        let interval = -(10.0 * 10.0) + 3.0 * 3.0 + 4.0 * 4.0 + 0.0;
        let interval_prime = -(transformed.ct_prime * transformed.ct_prime)
            + transformed.x_prime * transformed.x_prime
            + transformed.y_prime * transformed.y_prime
            + transformed.z_prime * transformed.z_prime;
        assert!((interval - interval_prime).abs() < 1.0e-6);
    }

    #[test]
    fn schwarzschild_metric_outside_horizon() {
        let mut metric = SchwarzschildMetric::default();
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        assert!(rs_val.is_finite() && rs_val > 0.0);
        let r = 10.0 * rs_val;
        assert_eq!(
            rel_schwarzschild_metric(r, SOLAR_MASS, G, &mut metric),
            Bool::TRUE
        );
        assert!(metric.g_tt < 0.0);
        assert!(metric.g_rr > 0.0);
        assert!((metric.radius_over_rs - 10.0).abs() < 1.0e-6);
    }

    #[test]
    fn schwarzschild_metric_at_horizon_rejected() {
        let mut metric = SchwarzschildMetric::default();
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        assert_eq!(
            rel_schwarzschild_metric(rs_val, SOLAR_MASS, G, &mut metric),
            Bool::FALSE
        );
        assert_eq!(
            rel_schwarzschild_metric(0.5 * rs_val, SOLAR_MASS, G, &mut metric),
            Bool::FALSE
        );
    }

    #[test]
    fn time_dilation_far_from_mass() {
        let mut dilation = GravitationalTimeDilation::default();
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        let r = 1.0e6 * rs_val; // very far
        assert_eq!(
            rel_gravitational_time_dilation(r, SOLAR_MASS, G, &mut dilation),
            Bool::TRUE
        );
        assert!((dilation.stationary_factor - 1.0).abs() < 1.0e-6);
        assert!((dilation.orbital_factor - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn time_dilation_near_mass() {
        let mut dilation = GravitationalTimeDilation::default();
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        let r = 10.0 * rs_val;
        assert_eq!(
            rel_gravitational_time_dilation(r, SOLAR_MASS, G, &mut dilation),
            Bool::TRUE
        );
        let expected = (1.0 - 1.0_f64 / 10.0).sqrt();
        assert!((dilation.stationary_factor - expected).abs() < 1.0e-12);
    }

    #[test]
    fn length_contraction_at_half_c() {
        let mut contraction = LengthContraction::default();
        assert_eq!(
            rel_length_contraction(10.0, 0.5 * C, &mut contraction),
            Bool::TRUE
        );
        let expected_gamma = 1.0 / (1.0 - 0.5_f64.powi(2)).sqrt();
        assert!((contraction.lorentz_factor - expected_gamma).abs() < 1.0e-12);
        assert!((contraction.speed_ratio - 0.5).abs() < 1.0e-12);
        assert!((contraction.contracted_length - 10.0 / expected_gamma).abs() < 1.0e-12);
    }

    #[test]
    fn length_contraction_zero_length() {
        let mut contraction = LengthContraction::default();
        assert_eq!(
            rel_length_contraction(0.0, 0.8 * C, &mut contraction),
            Bool::TRUE
        );
        assert_eq!(contraction.contracted_length, 0.0);
    }

    #[test]
    fn particle_properties_at_rest() {
        let mut particle = RelativisticParticle::default();
        assert_eq!(
            rel_particle_properties(1.0, Vec3::default(), &mut particle),
            Bool::TRUE
        );
        assert!((particle.lorentz_factor - 1.0).abs() < 1.0e-12);
        assert!((particle.total_energy - C * C).abs() < 1.0);
        assert!(particle.kinetic_energy.abs() < 1.0);
        assert_eq!(particle.momentum_magnitude, 0.0);
        assert!(particle.rapidity.abs() < 1.0e-12);
    }

    #[test]
    fn particle_properties_with_speed() {
        let mut particle = RelativisticParticle::default();
        let speed = 0.6 * C;
        let v = Vec3 {
            x: speed,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            rel_particle_properties(2.0, v, &mut particle),
            Bool::TRUE
        );
        let expected_gamma = 1.0 / (1.0 - 0.6_f64.powi(2)).sqrt();
        assert!((particle.lorentz_factor - expected_gamma).abs() < 1.0e-10);
        assert!(
            (particle.total_energy - expected_gamma * 2.0 * C * C).abs() < 1.0
        );
        assert!(
            (particle.kinetic_energy - (expected_gamma - 1.0) * 2.0 * C * C).abs() < 1.0
        );
        assert!((particle.momentum_magnitude - expected_gamma * 2.0 * speed).abs() < 1.0);
    }

    #[test]
    fn velocity_addition_less_than_c() {
        let u = Vec3 {
            x: 0.6 * C,
            y: 0.0,
            z: 0.0,
        };
        let v = Vec3 {
            x: 0.6 * C,
            y: 0.0,
            z: 0.0,
        };
        let mut result = Vec3::default();
        assert_eq!(rel_velocity_addition(u, v, &mut result), Bool::TRUE);
        // 1D relativistic addition: w = (u+v) / (1 + uv/c^2)
        let expected = (0.6 + 0.6) / (1.0 + 0.36) * C;
        assert!((result.x - expected).abs() < 1.0);
        assert!(result.x < C);
    }

    #[test]
    fn velocity_addition_with_zero() {
        let u = Vec3 {
            x: 0.5 * C,
            y: 0.0,
            z: 0.0,
        };
        let v = Vec3::default();
        let mut result = Vec3::default();
        assert_eq!(rel_velocity_addition(u, v, &mut result), Bool::TRUE);
        assert!((result.x - 0.5 * C).abs() < 1.0);
    }

    #[test]
    fn invariant_mass_conserved() {
        let speed = 0.8 * C;
        let mass = 3.0;
        let mut particle = RelativisticParticle::default();
        assert_eq!(
            rel_particle_properties(
                mass,
                Vec3 {
                    x: speed,
                    y: 0.0,
                    z: 0.0,
                },
                &mut particle,
            ),
            Bool::TRUE
        );
        let m0 = rel_invariant_mass(
            particle.total_energy,
            particle.momentum.x,
            particle.momentum.y,
            particle.momentum.z,
        );
        assert!(m0.is_finite());
        assert!((m0 - mass).abs() < 1.0e-6);
    }

    #[test]
    fn light_deflection_sun() {
        let solar_radius = 6.957e8;
        let angle = rel_light_deflection_angle(solar_radius, SOLAR_MASS, G);
        assert!(angle.is_finite());
        // Classical Einstein deflection ~ 8.48e-6 radians (1.75 arcseconds)
        let expected = 4.0 * G * SOLAR_MASS / (solar_radius * C * C);
        assert!((angle - expected).abs() < 1.0e-12);
        // Convert to arcseconds
        let arcsec = angle * 180.0 / std::f64::consts::PI * 3600.0;
        assert!((arcsec - 1.75).abs() < 0.1);
    }

    #[test]
    fn error_on_superspeed() {
        let mut gamma = 0.0;
        assert_eq!(
            rel_lorentz_factor(C * 1.1, &mut gamma),
            Bool::FALSE
        );
        let mut contraction = LengthContraction::default();
        assert_eq!(
            rel_length_contraction(1.0, C * 1.1, &mut contraction),
            Bool::FALSE
        );
    }

    #[test]
    fn error_on_null_pointer() {
        assert_eq!(
            rel_lorentz_factor(0.0, std::ptr::null_mut()),
            Bool::FALSE
        );
    }

    #[test]
    fn schwarzschild_radius_solar() {
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        assert!(rs_val.is_finite());
        let expected = 2.0 * G * SOLAR_MASS / (C * C);
        assert!((rs_val - expected).abs() / expected < 0.01);
        // Solar Schwarzschild radius ~ 2953 m
        assert!((rs_val - 2953.0).abs() < 100.0);
    }

    #[test]
    fn rapidity_consistent_with_gamma() {
        let speed = 0.8 * C;
        let mut gamma = 0.0;
        assert_eq!(rel_lorentz_factor(speed, &mut gamma), Bool::TRUE);
        let phi = rel_rapidity(speed);
        assert!(phi.is_finite());
        // cosh(phi) = gamma
        assert!((phi.cosh() - gamma).abs() < 1.0e-10);
    }

    #[test]
    fn beta_from_gamma_roundtrip() {
        let mut gamma = 0.0;
        assert_eq!(rel_lorentz_factor(0.9 * C, &mut gamma), Bool::TRUE);
        let beta = rel_beta_from_gamma(gamma);
        assert!(beta.is_finite());
        assert!((beta - 0.9).abs() < 1.0e-6);
    }

    #[test]
    fn effective_potential_outside_horizon() {
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        let r = 10.0 * rs_val;
        let l = 1.0e15; // some angular momentum
        let mut potential = 0.0;
        assert_eq!(
            rel_effective_potential(r, l, 1000.0, G, &mut potential),
            Bool::TRUE
        );
        assert!(potential.is_finite());
    }

    #[test]
    fn gravitational_time_dilation_simple_works() {
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        let r = 4.0 * rs_val;
        let factor = rel_gravitational_time_dilation_simple(r, rs_val);
        assert!(factor.is_finite());
        assert!((factor - (1.0 - 0.25_f64).sqrt()).abs() < 1.0e-12);
    }

    #[test]
    fn photon_particle_returns_infinity() {
        let mut particle = RelativisticParticle::default();
        assert_eq!(
            rel_particle_properties(
                0.0,
                Vec3 {
                    x: C,
                    y: 0.0,
                    z: 0.0,
                },
                &mut particle,
            ),
            Bool::TRUE
        );
        assert!(particle.lorentz_factor.is_infinite());
        assert!(particle.total_energy.is_infinite());
    }

    #[test]
    fn massless_below_c_is_rejected() {
        let mut particle = RelativisticParticle::default();
        assert_eq!(
            rel_particle_properties(
                0.0,
                Vec3 {
                    x: 0.5 * C,
                    y: 0.0,
                    z: 0.0,
                },
                &mut particle,
            ),
            Bool::FALSE
        );
    }
}