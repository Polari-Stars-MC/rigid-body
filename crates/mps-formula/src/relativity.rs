//! Relativistic effects: Lorentz transformations, Schwarzschild metric,
//! gravitational time dilation, length contraction, and near-light-speed particle physics.
//!
//! All functions are FFI-exported with C-compatible types, following the
//! error-handling conventions of the mps_rigid_body physics engine.

use crate::error::{
    ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ERR_UNSUPPORTED, clear_error, ffi_guard, set_error,
};
use crate::ffi::{
    Bool, GravitationalTimeDilation, LengthContraction, LorentzBoost, LorentzTransformedFrame,
    RelativisticParticle, SchwarzschildMetric, Vec3, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};
use crate::math::mul_add;

use crate::math::{EPS_GENERAL as EPSILON, finite_non_negative, finite_positive};

pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

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
///
/// # Safety
///
/// `out_gamma` must be non-null and point to writable memory for one `f64`.
/// No ownership is transferred; a null pointer fails with `ERR_NULL_POINTER`.
#[unsafe(no_mangle)]
pub extern "C" fn rel_lorentz_factor(speed: f64, out_gamma: *mut f64) -> Bool {
    ffi_guard(Bool::FALSE, || {
        if !finite_non_negative(speed) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "speed must be finite and non-negative",
            );
            return Bool::FALSE;
        }
        if speed >= SPEED_OF_LIGHT - EPSILON {
            set_error(
                ERR_INVALID_ARGUMENT,
                "speed must be less than speed of light",
            );
            return Bool::FALSE;
        }
        let beta = speed / SPEED_OF_LIGHT;
        // Use mul_add to avoid catastrophic cancellation when beta ≈ 1:
        // 1.0 - beta*beta = -(beta*beta - 1.0), computed with single rounding.
        let one_minus_beta_sq = -mul_add(beta, beta, -1.0);
        let gamma = 1.0 / one_minus_beta_sq.max(0.0).sqrt();
        write_out(out_gamma, gamma)
    })
}

/// Build the full 4x4 Lorentz boost matrix for a given velocity 3-vector.
///
/// The matrix acts on column 4-vectors (ct, x, y, z)^T.
///
/// # Safety
///
/// `out_boost` must be non-null and point to writable memory for one
/// `LorentzBoost`. No ownership is transferred; a null pointer fails with
/// `ERR_NULL_POINTER`.
#[unsafe(no_mangle)]
pub extern "C" fn rel_lorentz_boost(velocity: Vec3, out_boost: *mut LorentzBoost) -> Bool {
    ffi_guard(Bool::FALSE, || {
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
                    m00: 1.0,
                    m11: 1.0,
                    m22: 1.0,
                    m33: 1.0,
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
        // β² == 1 is the light-speed asymptote: gamma diverges, so there is no
        // finite boost matrix. The old code substituted INFINITY for every matrix
        // entry and returned `TRUE`, which contradicted `rel_lorentz_factor` (same
        // velocity → ERR_INVALID_ARGUMENT) and handed the caller a NaN/Inf matrix
        // it would happily multiply by.
        if one_minus_beta_sq <= 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "velocity magnitude must be less than c (boost is singular at c)",
            );
            return Bool::FALSE;
        }
        let gamma = 1.0 / one_minus_beta_sq.sqrt();
        let g = gamma;
        let gm1_over_b2 = if beta_sq > 0.0 {
            (gamma - 1.0) / beta_sq
        } else {
            0.5 // limit as beta→0: (γ-1)/β² → 0.5
        };

        write_out(
            out_boost,
            LorentzBoost {
                m00: g,
                m01: -g * beta_x,
                m02: -g * beta_y,
                m03: -g * beta_z,
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
    })
}

/// Apply a Lorentz boost to a 4-vector (ct, x, y, z).
///
/// # Safety
///
/// `out_transformed` must be non-null and point to writable memory for one
/// `LorentzTransformedFrame`. No ownership is transferred; a null pointer
/// fails with `ERR_NULL_POINTER`.
#[unsafe(no_mangle)]
pub extern "C" fn rel_transform_four_vector(
    boost: LorentzBoost,
    ct: f64,
    x: f64,
    y: f64,
    z: f64,
    out_transformed: *mut LorentzTransformedFrame,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
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
                x_prime: boost.m10 * ct + boost.m11 * x + boost.m12 * y + boost.m13 * z,
                y_prime: boost.m20 * ct + boost.m21 * x + boost.m22 * y + boost.m23 * z,
                z_prime: boost.m30 * ct + boost.m31 * x + boost.m32 * y + boost.m33 * z,
            },
        )
    })
}

/// Relativistic velocity addition (3D general formula).
///
/// w = (u + v_∥ + v_⊥/γ_u) / (1 + u·v/c²)
///
/// # Safety
///
/// `out_result` must be non-null and point to writable memory for one `Vec3`.
/// No ownership is transferred; a null pointer fails with `ERR_NULL_POINTER`.
#[unsafe(no_mangle)]
pub extern "C" fn rel_velocity_addition(u: Vec3, v: Vec3, out_result: *mut Vec3) -> Bool {
    ffi_guard(Bool::FALSE, || {
        if !vec3_finite(u) || !vec3_finite(v) {
            set_error(ERR_INVALID_ARGUMENT, "velocities must be finite");
            return Bool::FALSE;
        }
        let u_vec = vec3_to_rapier(u);
        let v_vec = vec3_to_rapier(v);
        let u_len_sq = u_vec.length_squared();
        let c2 = SPEED_OF_LIGHT * SPEED_OF_LIGHT;
        if u_len_sq >= c2 - EPSILON {
            set_error(
                ERR_INVALID_ARGUMENT,
                "velocity u magnitude must be less than c",
            );
            return Bool::FALSE;
        }
        if v_vec.length_squared() >= c2 - EPSILON {
            set_error(
                ERR_INVALID_ARGUMENT,
                "velocity v magnitude must be less than c",
            );
            return Bool::FALSE;
        }
        let denom = 1.0 + u_vec.dot(v_vec) / c2;
        if denom.abs() < EPSILON {
            set_error(
                ERR_INVALID_ARGUMENT,
                "velocity addition denominator is zero",
            );
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
    })
}

/// Rapidity = arctanh(v/c).
///
/// # Safety
///
/// Takes only scalar values; no pointers are dereferenced.
#[unsafe(no_mangle)]
pub extern "C" fn rel_rapidity(speed: f64) -> f64 {
    ffi_guard(f64::NAN, || {
        if !finite_non_negative(speed) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "speed must be finite and non-negative",
            );
            return f64::NAN;
        }
        if speed >= SPEED_OF_LIGHT - EPSILON {
            set_error(ERR_INVALID_ARGUMENT, "speed must be less than c");
            return f64::NAN;
        }
        clear_error();
        let beta = speed / SPEED_OF_LIGHT;
        0.5 * ((1.0 + beta) / (1.0 - beta)).ln()
    })
}

/// Beta (v/c) from Lorentz factor: beta = sqrt(1 - 1/gamma^2).
///
/// # Safety
///
/// Takes only scalar values; no pointers are dereferenced.
#[unsafe(no_mangle)]
pub extern "C" fn rel_beta_from_gamma(gamma: f64) -> f64 {
    ffi_guard(f64::NAN, || {
        if !gamma.is_finite() || gamma < 1.0 {
            set_error(ERR_INVALID_ARGUMENT, "gamma must be >= 1");
            return f64::NAN;
        }
        clear_error();
        (1.0 - 1.0 / (gamma * gamma)).sqrt()
    })
}

/// Return the speed of light constant.
///
/// # Safety
///
/// Takes no arguments; no pointers are dereferenced.
#[unsafe(no_mangle)]
pub extern "C" fn rel_speed_of_light() -> f64 {
    ffi_guard(f64::NAN, || SPEED_OF_LIGHT)
}

// ---------------------------------------------------------------------------
// B. Schwarzschild metric
// ---------------------------------------------------------------------------

/// Compute the Schwarzschild radius rs = 2GM/c^2.
///
/// # Safety
///
/// Takes only scalar values; no pointers are dereferenced.
#[unsafe(no_mangle)]
pub extern "C" fn rel_schwarzschild_radius(mass: f64, gravitational_constant: f64) -> f64 {
    ffi_guard(f64::NAN, || {
        if !finite_positive(mass) || !finite_positive(gravitational_constant) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "mass and gravitational constant must be positive",
            );
            return f64::NAN;
        }
        clear_error();
        2.0 * gravitational_constant * mass / (SPEED_OF_LIGHT * SPEED_OF_LIGHT)
    })
}

/// Compute the Schwarzschild metric coefficients at a given radius.
///
/// # Safety
///
/// `out_metric` must be non-null and point to writable memory for one
/// `SchwarzschildMetric`. No ownership is transferred; a null pointer fails
/// with `ERR_NULL_POINTER`.
#[unsafe(no_mangle)]
pub extern "C" fn rel_schwarzschild_metric(
    radius: f64,
    mass: f64,
    gravitational_constant: f64,
    out_metric: *mut SchwarzschildMetric,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        if !finite_positive(radius)
            || !finite_positive(mass)
            || !finite_positive(gravitational_constant)
        {
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
    })
}

/// Einstein light deflection angle: delta_phi = 4GM/(b*c^2).
///
/// Returns the deflection angle in radians. Returns ERR_UNSUPPORTED when the
/// impact parameter is close to the photon sphere (b < 2.6 * rs).
///
/// # Safety
///
/// Takes only scalar values; no pointers are dereferenced.
#[unsafe(no_mangle)]
pub extern "C" fn rel_light_deflection_angle(
    impact_parameter: f64,
    mass: f64,
    gravitational_constant: f64,
) -> f64 {
    ffi_guard(f64::NAN, || {
        if !finite_positive(impact_parameter)
            || !finite_positive(mass)
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
    })
}

/// Effective potential for Schwarzschild orbits (per unit mass m of the orbiting body).
///
/// V_eff(r) = -GM/r + L^2/(2*r^2) - G*M*L^2/(c^2*r^3)
///
/// The orbiting body's mass m and angular momentum L are parameters.
///
/// # Safety
///
/// `out_potential` must be non-null and point to writable memory for one
/// `f64`. No ownership is transferred; a null pointer fails with
/// `ERR_NULL_POINTER`.
#[unsafe(no_mangle)]
pub extern "C" fn rel_effective_potential(
    radius: f64,
    angular_momentum: f64,
    mass: f64,
    gravitational_constant: f64,
    out_potential: *mut f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        if !finite_positive(radius)
            || !finite_positive(mass)
            || !finite_positive(gravitational_constant)
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "radius, mass, and gravitational constant must be positive",
            );
            return Bool::FALSE;
        }
        if !finite_non_negative(angular_momentum) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "angular momentum must be finite and non-negative",
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
        let gm = gravitational_constant * mass;
        let newtonian =
            -gm / radius + 0.5 * angular_momentum * angular_momentum / (radius * radius);
        let gr_correction = -gm * angular_momentum * angular_momentum
            / (SPEED_OF_LIGHT * SPEED_OF_LIGHT * radius.powi(3));
        write_out(out_potential, newtonian + gr_correction)
    })
}

// ---------------------------------------------------------------------------
// C. Gravitational time dilation
// ---------------------------------------------------------------------------

/// Compute gravitational time dilation factors.
///
/// Stationary factor: dtau/dt = sqrt(1 - rs/r)
/// Orbital factor (circular orbit): dtau/dt = sqrt(1 - 3*rs/(2*r))
///
/// # Safety
///
/// `out_dilation` must be non-null and point to writable memory for one
/// `GravitationalTimeDilation`. No ownership is transferred; a null pointer
/// fails with `ERR_NULL_POINTER`.
#[unsafe(no_mangle)]
pub extern "C" fn rel_gravitational_time_dilation(
    radius: f64,
    mass: f64,
    gravitational_constant: f64,
    out_dilation: *mut GravitationalTimeDilation,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        if !finite_positive(radius)
            || !finite_positive(mass)
            || !finite_positive(gravitational_constant)
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "radius, mass, and gravitational constant must be positive",
            );
            return Bool::FALSE;
        }
        let rs = rel_schwarzschild_radius(mass, gravitational_constant);
        if !rs.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "Schwarzschild radius is not finite (mass and G overflow)",
            );
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
        // Orbital factor requires r > 1.5*rs (the ISCO for Schwarzschild). Below it
        // there is no circular orbit, so this is a domain edge, not a NaN result:
        // reporting `orbital_factor = NaN` while returning `TRUE` violated
        // docs/formula-numeric-domains.md ("non-finite intermediate or result must
        // fail") and poisoned every downstream computation with a silent NaN.
        let min_orbital_radius = 1.5 * rs;
        if radius <= min_orbital_radius + EPSILON {
            set_error(
                ERR_INVALID_ARGUMENT,
                "no circular orbit inside 1.5 * Schwarzschild radius (ISCO)",
            );
            return Bool::FALSE;
        }
        let orbital_factor = (1.0 - 1.5 * rs / radius).sqrt();
        let orbiting_velocity = (gravitational_constant * mass / radius).sqrt();
        write_out(
            out_dilation,
            GravitationalTimeDilation {
                stationary_factor,
                orbital_factor,
                orbiting_velocity,
            },
        )
    })
}

/// Lightweight gravitational time dilation: returns sqrt(1 - rs/r) directly.
///
/// # Safety
///
/// Takes only scalar values; no pointers are dereferenced.
#[unsafe(no_mangle)]
pub extern "C" fn rel_gravitational_time_dilation_simple(
    radius: f64,
    schwarzschild_radius: f64,
) -> f64 {
    ffi_guard(f64::NAN, || {
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
    })
}

// ---------------------------------------------------------------------------
// D. Length contraction
// ---------------------------------------------------------------------------

/// Compute length contraction: L = L0 / gamma.
///
/// # Safety
///
/// `out_contraction` must be non-null and point to writable memory for one
/// `LengthContraction`. No ownership is transferred; a null pointer fails
/// with `ERR_NULL_POINTER`.
#[unsafe(no_mangle)]
pub extern "C" fn rel_length_contraction(
    proper_length: f64,
    speed: f64,
    out_contraction: *mut LengthContraction,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        if !finite_non_negative(proper_length) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "proper length must be finite and non-negative",
            );
            return Bool::FALSE;
        }
        if !finite_non_negative(speed) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "speed must be finite and non-negative",
            );
            return Bool::FALSE;
        }
        if speed >= SPEED_OF_LIGHT - EPSILON {
            set_error(
                ERR_INVALID_ARGUMENT,
                "speed must be less than speed of light",
            );
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
    })
}

// ---------------------------------------------------------------------------
// E. Near-light-speed particle physics
// ---------------------------------------------------------------------------

/// Compute relativistic particle properties.
///
/// For zero mass (photon-like), speed must equal c. `gamma`, total / kinetic
/// energy, momentum magnitude and rapidity are then all `INFINITY` — a massless
/// particle at c has no finite rest frame — and `momentum` carries the
/// **direction** as a unit vector. Read `momentum_magnitude` for the (infinite)
/// scalar; do not treat the vector's length as a physical value in that case.
///
/// Earlier this multiplied the direction by `INFINITY`, so any axis-aligned
/// photon got `momentum = (inf, NaN, NaN)` (`0 * inf == NaN`) while the function
/// returned success.
///
/// # Safety
///
/// `out_particle` must be non-null and point to writable memory for one
/// `RelativisticParticle`. No ownership is transferred; a null pointer fails
/// with `ERR_NULL_POINTER`.
#[unsafe(no_mangle)]
pub extern "C" fn rel_particle_properties(
    mass: f64,
    velocity: Vec3,
    out_particle: *mut RelativisticParticle,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
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
                // Unit direction only: multiplying by INFINITY turned every
                // axis-aligned photon into `(inf, NaN, NaN)` via `0 * inf == NaN`.
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
                        momentum: vec3_from_rapier(dir),
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
            crate::math::Vector3f64::zeros()
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
    })
}

/// Compute the invariant (rest) mass from energy and momentum:
///
/// m0 = sqrt(E^2/c^4 - p^2/c^2)
///
/// Returns NAN for tachyonic states (E^2 < p^2 * c^2).
///
/// # Safety
///
/// Takes only scalar values; no pointers are dereferenced.
#[unsafe(no_mangle)]
pub extern "C" fn rel_invariant_mass(energy: f64, px: f64, py: f64, pz: f64) -> f64 {
    ffi_guard(f64::NAN, || {
        if !energy.is_finite()
            || energy < 0.0
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
            set_error(ERR_INVALID_ARGUMENT, "tachyonic state: E^2 < p^2 * c^2");
            return f64::NAN;
        }
        clear_error();
        mass_sq.sqrt()
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Kerr metric basics
// ---------------------------------------------------------------------------

/// Kerr metric horizon radii.
/// Returns (r_outer, r_inner) where r = GM/c^2 +- sqrt((GM/c^2)^2 - a^2)
pub fn kerr_horizon_radii(mass: f64, spin_parameter: f64, g: f64) -> Option<(f64, f64)> {
    let c = 299_792_458.0;
    if !mass.is_finite()
        || mass <= 0.0
        || !spin_parameter.is_finite()
        || spin_parameter < 0.0
        || !g.is_finite()
        || g <= 0.0
    {
        return None;
    }
    let m = g * mass / (c * c); // gravitational radius
    if spin_parameter > m {
        return None;
    } // naked singularity
    let r = (m * m - spin_parameter * spin_parameter).sqrt();
    Some((m + r, m - r))
}

/// Kerr ergosphere radius (outer): r_E = m + sqrt(m^2 - a^2 * cos^2(theta))
pub fn kerr_ergosphere_radius(
    mass: f64,
    spin_parameter: f64,
    polar_angle: f64,
    g: f64,
) -> Option<f64> {
    let c = 299_792_458.0;
    if !mass.is_finite()
        || mass <= 0.0
        || !spin_parameter.is_finite()
        || spin_parameter < 0.0
        || !polar_angle.is_finite()
        || !g.is_finite()
        || g <= 0.0
    {
        return None;
    }
    let m = g * mass / (c * c);
    if spin_parameter > m {
        return None;
    }
    let r = (m * m - spin_parameter * spin_parameter * polar_angle.cos().powi(2)).sqrt();
    Some(m + r)
}

/// Frame-dragging angular velocity at radius r (the Lense-Thirring effect):
/// omega = 2 * m * a * r / Sigma^2  where Sigma = r^2 + a^2 * cos^2(theta)
pub fn kerr_frame_dragging_frequency(
    mass: f64,
    spin_parameter: f64,
    r: f64,
    theta: f64,
    g: f64,
) -> Option<f64> {
    let c = 299_792_458.0;
    if !mass.is_finite()
        || mass <= 0.0
        || !spin_parameter.is_finite()
        || spin_parameter < 0.0
        || !r.is_finite()
        || r <= 0.0
        || !theta.is_finite()
        || !g.is_finite()
        || g <= 0.0
    {
        return None;
    }
    let m = g * mass / (c * c);
    let cos2 = theta.cos().powi(2);
    let sigma = r * r + spin_parameter * spin_parameter * cos2;
    Some(2.0 * m * spin_parameter * r / (sigma * sigma))
}

// ---------------------------------------------------------------------------
// ISCO radius
// ---------------------------------------------------------------------------

/// Innermost Stable Circular Orbit (ISCO) for Schwarzschild: 6 M (3 R_s)
pub fn schwarzschild_isco(mass: f64, g: f64) -> Option<f64> {
    let c = 299_792_458.0;
    if !mass.is_finite() || mass <= 0.0 || !g.is_finite() || g <= 0.0 {
        return None;
    }
    Some(6.0 * g * mass / (c * c))
}

/// ISCO radius for Kerr (prograde orbit): r_isco/M depends on spin
/// r_isco(prograde) = M * (3 + Z2 - sqrt((3 - Z1)(3 + Z1 + 2*Z2)))
/// where Z1 = 1 + (1 - a^2)^(1/3) * ((1+a)^(1/3) + (1-a)^(1/3))
/// Z2 = sqrt(3*a^2 + Z1^2)
pub fn kerr_isco(mass: f64, spin_parameter: f64, g: f64, prograde: bool) -> Option<f64> {
    let c = 299_792_458.0;
    if !mass.is_finite()
        || mass <= 0.0
        || !spin_parameter.is_finite()
        || spin_parameter < 0.0
        || !g.is_finite()
        || g <= 0.0
    {
        return None;
    }
    let m = g * mass / (c * c);
    let a = if prograde {
        spin_parameter.min(m)
    } else {
        -spin_parameter.min(m)
    };
    let a_norm = if m > 0.0 {
        a / m
    } else {
        return None;
    };
    let z1 = 1.0
        + (1.0 - a_norm * a_norm).powf(1.0 / 3.0)
            * ((1.0 + a_norm).powf(1.0 / 3.0) + (1.0 - a_norm).powf(1.0 / 3.0));
    let z2 = (3.0 * a_norm * a_norm + z1 * z1).sqrt();
    let z3 = ((3.0 - z1) * (3.0 + z1 + 2.0 * z2)).sqrt();
    Some(m * (3.0 + z2 - z3))
}

// ---------------------------------------------------------------------------
// Gravitational redshift
// ---------------------------------------------------------------------------

/// Gravitational redshift: z = 1 / sqrt(1 - R_s / r) - 1
pub fn gravitational_redshift(mass: f64, radius: f64, g: f64) -> Option<f64> {
    let c = 299_792_458.0;
    if !mass.is_finite() || mass <= 0.0 || !radius.is_finite() || !g.is_finite() || g <= 0.0 {
        return None;
    }
    let rs = 2.0 * g * mass / (c * c);
    if radius <= rs {
        return None;
    } // inside horizon
    Some(1.0 / (1.0 - rs / radius).sqrt() - 1.0)
}

// ---------------------------------------------------------------------------
// Reissner-Nordstrom metric (charged black hole)
// ---------------------------------------------------------------------------

/// Reissner-Nordstrom horizon radii: r = m +- sqrt(m^2 - Q^2)
pub fn reissner_nordstrom_horizons(mass: f64, charge: f64, g: f64) -> Option<(f64, f64)> {
    let c = 299_792_458.0;
    let k = 8.9875517923e9;
    if !mass.is_finite()
        || mass <= 0.0
        || !charge.is_finite()
        || charge < 0.0
        || !g.is_finite()
        || g <= 0.0
    {
        return None;
    }
    let m = g * mass / (c * c);
    let q2 = k * g * charge * charge / (c * c * c * c);
    let disc = m * m - q2;
    if disc < 0.0 {
        return None;
    }
    let r = disc.sqrt();
    Some((m + r, m - r))
}

// ---------------------------------------------------------------------------
// Gravitational wave strain
// ---------------------------------------------------------------------------

/// Characteristic GW strain amplitude from compact binary.
/// h = (4/d) · (G M_c / c²)^(5/3) · (πf)^(2/3)
pub fn gw_strain_amplitude(
    distance: f64,
    chirp_mass_kg: f64,
    orbital_frequency: f64,
) -> Option<f64> {
    let g = 6.67430e-11;
    let c = 299_792_458.0;
    if !distance.is_finite()
        || distance <= 0.0
        || !chirp_mass_kg.is_finite()
        || chirp_mass_kg <= 0.0
        || !orbital_frequency.is_finite()
        || orbital_frequency <= 0.0
    {
        return None;
    }
    let pi_f = std::f64::consts::PI * orbital_frequency;
    let gm = g * chirp_mass_kg / (c * c);
    Some(4.0 / distance * gm.powf(5.0 / 3.0) * pi_f.powf(2.0 / 3.0))
}

/// Chirp mass: M_c = (m₁·m₂)^(3/5) / (m₁+m₂)^(1/5)
pub fn chirp_mass(mass1: f64, mass2: f64) -> Option<f64> {
    if !mass1.is_finite() || mass1 <= 0.0 || !mass2.is_finite() || mass2 <= 0.0 {
        return None;
    }
    Some((mass1 * mass2).powf(0.6) / (mass1 + mass2).powf(0.2))
}

/// GW frequency evolution: df/dt ∝ f^(11/3)
pub fn gw_frequency_derivative(frequency: f64, chirp_mass_kg: f64) -> Option<f64> {
    let g = 6.67430e-11;
    let c = 299_792_458.0;
    if !frequency.is_finite()
        || frequency <= 0.0
        || !chirp_mass_kg.is_finite()
        || chirp_mass_kg <= 0.0
    {
        return None;
    }
    let mc = g * chirp_mass_kg / (c * c * c);
    Some(
        96.0 / 5.0
            * std::f64::consts::PI.powf(8.0 / 3.0)
            * mc.powf(5.0 / 3.0)
            * frequency.powf(11.0 / 3.0),
    )
}

/// Relativistic longitudinal Doppler shift.
pub fn relativistic_doppler_longitudinal(
    source_frequency: f64,
    relative_velocity: f64,
    approaching: bool,
) -> Option<f64> {
    let c = 299_792_458.0;
    if !source_frequency.is_finite()
        || source_frequency <= 0.0
        || !relative_velocity.is_finite()
        || relative_velocity < 0.0
        || relative_velocity >= c
    {
        return None;
    }
    let beta = relative_velocity / c;
    let shift = ((1.0 - beta) / (1.0 + beta)).sqrt();
    Some(if approaching {
        source_frequency / shift
    } else {
        source_frequency * shift
    })
}

/// Relativistic transverse Doppler: f' = f/γ
pub fn relativistic_doppler_transverse(
    source_frequency: f64,
    relative_velocity: f64,
) -> Option<f64> {
    let c = 299_792_458.0;
    if !source_frequency.is_finite()
        || source_frequency <= 0.0
        || !relative_velocity.is_finite()
        || relative_velocity < 0.0
        || relative_velocity >= c
    {
        return None;
    }
    let gamma = 1.0 / (1.0 - (relative_velocity / c).powi(2)).sqrt();
    Some(source_frequency / gamma)
}

/// Gravitational lensing Einstein radius for point mass.
pub fn einstein_radius(
    mass_kg: f64,
    dist_lens: f64,
    dist_source: f64,
    dist_ls: f64,
) -> Option<f64> {
    let g = 6.67430e-11;
    let c = 299_792_458.0;
    if !mass_kg.is_finite()
        || mass_kg <= 0.0
        || !dist_lens.is_finite()
        || dist_lens <= 0.0
        || !dist_source.is_finite()
        || dist_source <= 0.0
        || !dist_ls.is_finite()
        || dist_ls <= 0.0
    {
        return None;
    }
    Some((4.0 * g * mass_kg / (c * c) * dist_ls / (dist_lens * dist_source)).sqrt())
}

/// Cosmological redshift: z = 1/a - 1
pub fn cosmological_redshift(scale_factor: f64) -> Option<f64> {
    if !scale_factor.is_finite() || scale_factor <= 0.0 {
        return None;
    }
    Some(1.0 / scale_factor - 1.0)
}

/// Redshift from wavelengths: z = (λ_obs - λ_em) / λ_em
pub fn redshift_from_wavelengths(observed: f64, emitted: f64) -> Option<f64> {
    if !observed.is_finite() || !emitted.is_finite() || emitted <= 0.0 {
        return None;
    }
    Some(observed / emitted - 1.0)
}

/// Lense-Thirring frame dragging angular frequency at polar orbit.
pub fn lense_thirring_angular_frequency(
    mass_kg: f64,
    spin_parameter: f64,
    orbital_radius: f64,
) -> Option<f64> {
    let g = 6.67430e-11;
    let c = 299_792_458.0;
    if !mass_kg.is_finite()
        || mass_kg <= 0.0
        || !spin_parameter.is_finite()
        || !orbital_radius.is_finite()
        || orbital_radius <= 0.0
    {
        return None;
    }
    let j = spin_parameter * mass_kg * c;
    Some(2.0 * g * j / (c * c * orbital_radius * orbital_radius * orbital_radius))
}

/// Schwarzschild effective potential: V_eff = (1 - r_s/r)(1 + L²/r²)
pub fn schwarzschild_effective_potential(r: f64, rs: f64, angular_momentum: f64) -> Option<f64> {
    if !r.is_finite()
        || r <= 0.0
        || !rs.is_finite()
        || rs <= 0.0
        || r <= rs
        || !angular_momentum.is_finite()
    {
        return None;
    }
    Some((1.0 - rs / r) * (1.0 + angular_momentum * angular_momentum / (r * r)))
}

// ---------------------------------------------------------------------------
// Gravitational-wave matched filtering (PHYSICS_EXPANSION_PLAN.md W6)
// ---------------------------------------------------------------------------

/// Matched-filter signal-to-noise ratio for a circular compact-binary
/// inspiral against a flat (single-sided) noise PSD.  Order-of-magnitude
/// estimate under the stationary-phase approximation:
///
/// ```text
/// ρ² = 4 · ∫|h̃(f)|² / S_n(f) df
/// ρ ≈ h_rss · sqrt(Δf / S_n)
/// ```
///
/// Inputs:
/// - `strain_rss` — root-sum-square strain amplitude `h_rss` [1/sqrt(Hz)]
/// - `f_min`      — lower band edge [Hz] (e.g. 20 Hz for LIGO O4)
/// - `f_max`      — upper band edge [Hz] (e.g. 400 Hz for post-merger cutoff)
/// - `noise_psd`  — flat single-sided noise PSD `S_n` at the band centre
///   [Hz^-1] (e.g. 1e-46 for early-aLIGO at 100 Hz)
///
/// Returns the matched-filter SNR (dimensionless).  Note: real LIGO uses a
/// shaped PSD curve, not a single number; this is a compact closed-form
/// estimate that generalises naturally when integrated against an actual
/// PSD curve later (see PHYSICS_EXPANSION_PLAN.md W6 follow-up).
pub fn gw_inspiral_snr(strain_rss: f64, f_min: f64, f_max: f64, noise_psd: f64) -> Option<f64> {
    if !finite_positive(strain_rss)
        || !finite_positive(f_min)
        || !finite_positive(f_max)
        || f_max <= f_min
        || !finite_positive(noise_psd)
    {
        set_error(ERR_INVALID_ARGUMENT, "bad GW inspiral SNR args");
        return None;
    }
    let bandwidth = f_max - f_min;
    // ρ ≈ h_rss · sqrt(Δf / S_n) — root-sum-square convention; for
    // root-power spectral density conventions this collapses the integral.
    Some(strain_rss * (bandwidth / noise_psd).sqrt())
}

/// inspiral-time-to-coalescence for a circular binary in the quadrupole
/// approximation (Peters & Mathews 1963, leading order):
///
/// ```text
/// t_c = (5/256) · (c⁵ / G³) · (M_c⁵ / f⁵) · (π · f)^(-8/3)
/// ```
///
/// Simplified (geometric units dropped back to SI): use chirp mass and the
/// gravitational-wave frequency `f_gw` (twice the orbital frequency) to
/// compute remaining inspiral time to coalescence.
///
/// Inputs:
/// - `chirp_mass_kg` — M_c (pulsar-mass-plus-pulsar-mass-derived chirp mass)
/// - `f_gw_hz`       — current gravitational-wave frequency [Hz]
///
/// Returns seconds until coalescence.  For reference, a 1.4 Msun + 1.4 Msun
/// binary at f_gw = 100 Hz has t_c ≈ 2.2 s.
pub fn gw_inspiral_time_to_coalescence(chirp_mass_kg: f64, f_gw_hz: f64) -> Option<f64> {
    const G: f64 = 6.67430e-11;
    if !finite_positive(chirp_mass_kg) || !finite_positive(f_gw_hz) {
        set_error(ERR_INVALID_ARGUMENT, "bad GW t_c args");
        return None;
    }
    // t_c = (5/256) · c^5 / (G^(5/3) · π^(8/3) · f_gw^(8/3) · M_c^(5/3))
    // Standard inspiral formula from leading-order quadrupole radiation.
    let f_pow = f_gw_hz.powf(8.0 / 3.0);
    let m_pow = chirp_mass_kg.powf(5.0 / 3.0);
    let numerator = 5.0 / 256.0 * SPEED_OF_LIGHT.powi(5);
    let denominator = G.powf(5.0 / 3.0) * core::f64::consts::PI.powf(8.0 / 3.0) * f_pow * m_pow;
    Some(numerator / denominator)
}

// ---------------------------------------------------------------------------
// F. Relativistic kinematics relations
// ---------------------------------------------------------------------------

/// Relativistic total energy: E = γ·m·c².
pub fn relativistic_total_energy(rest_mass: f64, lorentz_factor: f64) -> Option<f64> {
    if !rest_mass.is_finite()
        || rest_mass < 0.0
        || !lorentz_factor.is_finite()
        || lorentz_factor < 1.0
    {
        return None;
    }
    Some(lorentz_factor * rest_mass * SPEED_OF_LIGHT * SPEED_OF_LIGHT)
}

/// Relativistic momentum magnitude: p = γ·m·v.
pub fn relativistic_momentum(rest_mass: f64, speed: f64) -> Option<f64> {
    if !rest_mass.is_finite()
        || rest_mass < 0.0
        || !speed.is_finite()
        || !(0.0..SPEED_OF_LIGHT).contains(&speed)
    {
        return None;
    }
    let beta = speed / SPEED_OF_LIGHT;
    let gamma = 1.0 / (1.0 - beta * beta).sqrt();
    Some(gamma * rest_mass * speed)
}

/// Energy–momentum relation (inverse of invariant mass): E = √(m²c⁴ + p²c²).
pub fn relativistic_energy_from_momentum(rest_mass: f64, momentum: f64) -> Option<f64> {
    if !rest_mass.is_finite() || rest_mass < 0.0 || !momentum.is_finite() || momentum < 0.0 {
        return None;
    }
    let c2 = SPEED_OF_LIGHT * SPEED_OF_LIGHT;
    Some((rest_mass * rest_mass * c2 * c2 + momentum * momentum * c2).sqrt())
}

/// Relativistic aberration of light: cos θ' = (cos θ − β) / (1 − β·cos θ).
pub fn relativistic_aberration(cos_theta: f64, beta: f64) -> Option<f64> {
    if !cos_theta.is_finite() || !beta.is_finite() || beta.abs() >= 1.0 {
        return None;
    }
    let denom = 1.0 - beta * cos_theta;
    if denom.abs() < 1.0e-12 {
        return None;
    }
    Some(((cos_theta - beta) / denom).clamp(-1.0, 1.0))
}

/// Relativistic Doppler beaming (boost) factor: δ = 1 / [γ·(1 − β·cos θ)].
pub fn relativistic_doppler_beaming_factor(beta: f64, cos_theta: f64) -> Option<f64> {
    if !beta.is_finite() || !(0.0..1.0).contains(&beta) || !cos_theta.is_finite() {
        return None;
    }
    let gamma = 1.0 / (1.0 - beta * beta).sqrt();
    let denom = gamma * (1.0 - beta * cos_theta);
    if denom.abs() < 1.0e-12 {
        return None;
    }
    Some(1.0 / denom)
}

// ---------------------------------------------------------------------------
// G. Black-hole geometry & thermodynamics
// ---------------------------------------------------------------------------

/// Photon-sphere radius (Schwarzschild): r_ph = 1.5·r_s = 3·G·M/c².
pub fn photon_sphere_radius(mass: f64, g: f64) -> Option<f64> {
    if !mass.is_finite() || mass <= 0.0 || !g.is_finite() || g <= 0.0 {
        return None;
    }
    Some(3.0 * g * mass / (SPEED_OF_LIGHT * SPEED_OF_LIGHT))
}

const HAWKING_HBAR: f64 = 1.054_571_817e-34;
const HAWKING_KB: f64 = 1.380_649e-23;

/// Hawking temperature of a Schwarzschild black hole:
/// T = ħ·c³ / (8·π·G·M·k_B).
pub fn hawking_temperature(mass: f64, g: f64) -> Option<f64> {
    if !mass.is_finite() || mass <= 0.0 || !g.is_finite() || g <= 0.0 {
        return None;
    }
    Some(
        HAWKING_HBAR * SPEED_OF_LIGHT.powi(3)
            / (8.0 * std::f64::consts::PI * g * mass * HAWKING_KB),
    )
}

// ---------------------------------------------------------------------------
// H. Cosmological kinematics
// ---------------------------------------------------------------------------

/// Hubble-law recession velocity: v = H₀·d.
pub fn hubble_recession_velocity(distance: f64, hubble_constant: f64) -> Option<f64> {
    if !distance.is_finite()
        || distance < 0.0
        || !hubble_constant.is_finite()
        || hubble_constant <= 0.0
    {
        return None;
    }
    Some(hubble_constant * distance)
}

/// Hubble-law luminosity distance from redshift (low-z): d = c·z / H₀.
pub fn hubble_distance(redshift: f64, hubble_constant: f64) -> Option<f64> {
    if !redshift.is_finite()
        || redshift < 0.0
        || !hubble_constant.is_finite()
        || hubble_constant <= 0.0
    {
        return None;
    }
    Some(SPEED_OF_LIGHT * redshift / hubble_constant)
}

/// Flat matter-dominated universe lookback time:
/// t_L = (2/3)·t_H·(1 − 1/√(1+z)), where t_H = 1/H₀ is the Hubble time.
pub fn flat_universe_lookback_time(redshift: f64, hubble_time: f64) -> Option<f64> {
    if !redshift.is_finite() || redshift < 0.0 || !hubble_time.is_finite() || hubble_time <= 0.0 {
        return None;
    }
    let factor = 1.0 + redshift;
    Some((2.0 / 3.0) * hubble_time * (1.0 - 1.0 / factor.sqrt()))
}
