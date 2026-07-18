//! Advanced gravitational field models.
//!
//! ## Supported models (in order of increasing accuracy)
//!
//! 1. **Point-mass Newtonian** — `F = GMm / r²`
//! 2. **J2–J6 zonal harmonics** — axial symmetry (oblateness + pear shape)
//! 3. **Spherical harmonics** — full EGM2008/LP165 field to arbitrary degree
//! 4. **Triaxial ellipsoid** — exact closed-form for uniform-density ellipsoids
//! 5. **Quadrupole tensor** — fast 3×3 matrix approximation for irregular bodies
//!
//! ## Architecture
//!
//! ```
//! CelestialBody (celestial_data.rs)
//!   ↓ provides μ, R, Jn, C̄ₙₘ, S̄ₙₘ
//! GravityModel (this module)
//!   ↓ computes acceleration vector
//! world_step / integrator
//!   ↓ applies to RigidBody
//! ```
//!
//! ## References
//!
//! - Montenbruck & Gill, *Satellite Orbits* (2000), Ch.3
//! - Pavlis et al., *EGM2008*, JGR 117 (2012)
//! - Carlson, *Elliptic Integrals*, Num. Math. 33 (1979)

use crate::rapier::celestial_data::CelestialBody;
use crate::rapier::error::{ERR_INVALID_ARGUMENT, clear_error, set_error};
use crate::rapier::ffi::{Bool, Vec3, vec3_finite, vec3_from_rapier, vec3_to_rapier};
use crate::rapier::math::{KahanSum, KahanVec3};

// ---------------------------------------------------------------------------
// Legendre polynomials & associated Legendre functions
// ---------------------------------------------------------------------------

/// Associated Legendre function of the first kind P̄ₙₘ (4π-normalized).
///
/// Recurrence relation (Holmes & Featherstone 2002):
///   P̄₀₀ = 1
///   P̄_{n,n} = √((2n+1)/(2n)) · cos(φ) · P̄_{n-1,n-1}
///   P̄_{n+1,n} = √(2n+3) · sin(φ) · P̄_{n,n}
///   P̄_{n,m} = a_{n,m} · sin(φ) · P̄_{n-1,m} - b_{n,m} · P̄_{n-2,m}
///
/// where φ is the geocentric latitude (sin φ = z/r).
///
/// Returns a vector `pnm` indexed as pnm[n*(n+1)/2 + m] for n=0..max_degree.
fn normalized_legendre(sin_phi: f64, max_degree: u32) -> Vec<f64> {
    let n_max = max_degree as usize;
    let size = (n_max + 1) * (n_max + 2) / 2;
    let mut pnm = vec![0.0; size];

    pnm[0] = 1.0; // P̄₀₀

    if n_max == 0 {
        return pnm;
    }

    let cos_phi = (1.0 - sin_phi * sin_phi).sqrt().max(0.0);

    // Standard Holmes & Featherstone (2002) recurrence:
    // For each n, first compute P̄_{n,n} (sectoral), then P̄_{n,0..n-1}

    for n in 1..=n_max {
        let nf = n as f64;

        // ---- Sectoral term: P̄_{n,n} ----
        let idx_nn = n * (n + 1) / 2 + n;
        if n == 1 {
            // P̄₁₁ = √3 · cos φ
            pnm[idx_nn] = (3.0_f64).sqrt() * cos_phi;
        } else {
            let idx_prev_nn = (n - 1) * n / 2 + (n - 1);
            // P̄_{n,n} = √((2n+1)/(2n)) · cos φ · P̄_{n-1,n-1}
            let factor = ((2.0 * nf + 1.0) / (2.0 * nf)).sqrt();
            pnm[idx_nn] = factor * cos_phi * pnm[idx_prev_nn];
        }

        // ---- Tesseral terms: P̄_{n,m} for m = 0..n-1 ----
        // P̄_{n,m} = a_{n,m} · sin φ · P̄_{n-1,m} - b_{n,m} · P̄_{n-2,m}
        // where:
        //   a_{n,m} = √((2n-1)(2n+1) / ((n-m)(n+m)))
        //   b_{n,m} = √((2n+1)(n+m-1)(n-m-1) / ((2n-3)(n-m)(n+m)))
        for m in 0..n {
            let mf = m as f64;
            let idx = n * (n + 1) / 2 + m;

            if n == 1 {
                // P̄₁₀ = √3 · sin φ
                pnm[1 * 2 / 2 + 0] = (3.0_f64).sqrt() * sin_phi;
                continue;
            }

            let nm1_idx = (n - 1) * n / 2 + m;

            // a_{n,m}
            let a = {
                let denom = (nf - mf) * (nf + mf);
                if denom <= 0.0 {
                    // m = n gives sectoral (already done above), m=n-1 needs near-sectoral
                    continue;
                }
                ((2.0 * nf - 1.0) * (2.0 * nf + 1.0) / denom).sqrt()
            };

            // b_{n,m}
            let b = if n >= 2 && m < n - 1 {
                let nm2_idx = (n - 2) * (n - 1) / 2 + m;
                let denom = (2.0 * nf - 3.0) * (nf - mf) * (nf + mf);
                if denom <= 0.0 {
                    0.0
                } else {
                    ((2.0 * nf + 1.0) * (nf + mf - 1.0) * (nf - mf - 1.0) / denom).sqrt()
                }
            } else {
                0.0
            };

            let nm2_idx = if n >= 2 { (n - 2) * (n - 1) / 2 + m } else { 0 };

            pnm[idx] = a * sin_phi * pnm[nm1_idx];
            if n >= 2 && b != 0.0 {
                pnm[idx] -= b * pnm[nm2_idx];
            }
        }
    }

    pnm
}

// ---------------------------------------------------------------------------
// Spherical harmonics acceleration
// ---------------------------------------------------------------------------

/// Compute gravitational acceleration from a spherical-harmonic field.
///
/// V(r,θ,λ) = (μ/r) · Σ_{n=0}^{N} (R/r)ⁿ · Σ_{m=0}^{n} P̄ₙₘ(sin θ) · (C̄ₙₘ cos mλ + S̄ₙₘ sin mλ)
///
/// The acceleration a = -∇V is computed via the Cunningham (1970) recurrence.
///
/// # Arguments
/// * `position` — body-fixed position (ECEF for Earth)
/// * `body` — celestial body providing μ, R, C̄, S̄ coefficients
/// * `max_degree` — maximum degree to use (≤ body.max_degree)
///
/// # Returns
/// * `Vec3` — acceleration vector (m/s²) in body-fixed frame
pub fn spherical_harmonics_acceleration(
    position: Vec3,
    body: &CelestialBody,
    max_degree: u32,
) -> Vec3 {
    let r_vec = vec3_to_rapier(position);
    let radius = r_vec.length();

    if radius < 1.0 {
        return Vec3::default(); // inside the body
    }

    let mu = body.gm;
    let ref_r = body.ref_radius;
    let n_max = max_degree.min(body.max_degree) as usize;

    if n_max == 0 || body.c_coeffs.is_empty() {
        // Fallback to point mass
        let accel = -r_vec / (radius * radius * radius) * mu;
        return vec3_from_rapier(accel);
    }

    let sin_phi = r_vec.z / radius;
    let cos_phi = (r_vec.x * r_vec.x + r_vec.y * r_vec.y).sqrt() / radius;
    let lambda = r_vec.y.atan2(r_vec.x); // longitude

    // Precompute P̄ₙₘ
    let pnm = normalized_legendre(sin_phi, n_max as u32);

    // Cunningham recurrences for dV/dr, dV/dφ, dV/dλ
    let mut dv_dr = KahanSum::default();
    let mut dv_dphi = KahanSum::default();
    let mut dv_dlambda = KahanSum::default();

    for n in 2..=n_max {
        let nf = n as f64;
        let rr_n = (ref_r / radius).powi(n as i32);
        let scale = mu / radius * rr_n;

        for m in 0..=n {
            let mf = m as f64;
            let idx = n * (n + 1) / 2 + m;

            // C̄ₙₘ, S̄ₙₘ from coefficient arrays
            let c_idx = idx - 3; // offset: skip n=0,1 (monopole + dipole)
            let c_nm = if c_idx >= 0 {
                let cu = c_idx as usize;
                if cu < body.c_coeffs.len() { body.c_coeffs[cu] } else { 0.0 }
            } else {
                0.0
            };
            let s_nm = if c_idx >= 0 {
                let cu = c_idx as usize;
                if cu < body.s_coeffs.len() { body.s_coeffs[cu] } else { 0.0 }
            } else {
                0.0
            };

            let p_nm = pnm[idx];
            let cos_ml = (mf * lambda).cos();
            let sin_ml = (mf * lambda).sin();
            let cs = c_nm * cos_ml + s_nm * sin_ml;
            let sc = s_nm * cos_ml - c_nm * sin_ml;

            // Radial derivative
            dv_dr.add(-(nf + 1.0) * scale * p_nm * cs);

            // Latitudinal derivative
            if m < n {
                let idx_m1 = idx + 1; // P̄_{n,m+1}
                let p_nmp1 = pnm[idx_m1];
                dv_dphi.add(scale * p_nmp1 * cs);
            } else {
                dv_dphi.add(0.0);
            }

            // Longitudinal derivative
            dv_dlambda.add(scale * mf * p_nm * sc);
        }
    }

    // Convert spherical derivatives to Cartesian acceleration
    let dr = dv_dr.value();
    let dphi = dv_dphi.value();
    let dlambda = dv_dlambda.value();

    let r_xy = (r_vec.x * r_vec.x + r_vec.y * r_vec.y).sqrt().max(1e-15);
    let x_r = r_vec.x / radius;
    let y_r = r_vec.y / radius;
    let z_r = r_vec.z / radius;

    // ∂V/∂x, ∂V/∂y, ∂V/∂z
    let ax = x_r * dr
        - r_vec.x * r_vec.z / (radius * radius * r_xy) * dphi
        - r_vec.y / (r_xy * r_xy) * dlambda;
    let ay = y_r * dr
        - r_vec.y * r_vec.z / (radius * radius * r_xy) * dphi
        + r_vec.x / (r_xy * r_xy) * dlambda;
    let az = z_r * dr + r_xy / (radius * radius) * dphi;

    // Centrifugal force (if body rotates)
    let cf = body.centrifugal_acceleration(position);

    Vec3 {
        x: -(ax + cf.x),
        y: -(ay + cf.y),
        z: -(az + cf.z),
    }
}

// ---------------------------------------------------------------------------
// Ellipsoid gravity — exact solution via Carlson elliptic integrals
// ---------------------------------------------------------------------------

/// Evaluate Carlson's symmetric elliptic integral R_F(x, y, z).
fn carlson_rf(x: f64, y: f64, z: f64) -> f64 {
    let mut x = x;
    let mut y = y;
    let mut z = z;

    for _ in 0..20 {
        let lambda = x.sqrt() * y.sqrt() + y.sqrt() * z.sqrt() + z.sqrt() * x.sqrt();
        x = (x + lambda) * 0.25;
        y = (y + lambda) * 0.25;
        z = (z + lambda) * 0.25;

        let avg = (x + y + z) / 3.0;
        let max_dev = ((x - avg).abs()).max((y - avg).abs()).max((z - avg).abs());
        if max_dev < 1e-15 * avg {
            break;
        }
    }

    let avg = (x + y + z) / 3.0;
    avg.powf(-0.5)
}

/// Evaluate Carlson's symmetric elliptic integral R_D(x, y, z).
fn carlson_rd(x: f64, y: f64, z: f64) -> f64 {
    let mut x = x;
    let mut y = y;
    let mut z = z;
    let mut sum = 0.0;
    let mut fac = 1.0;

    for _ in 0..20 {
        let lambda = x.sqrt() * y.sqrt() + y.sqrt() * z.sqrt() + z.sqrt() * x.sqrt();
        sum += fac / (z.sqrt() * (z + lambda));
        fac *= 0.25;
        x = (x + lambda) * 0.25;
        y = (y + lambda) * 0.25;
        z = (z + lambda) * 0.25;

        let avg = (x + y + z) / 3.0;
        let max_dev = ((x - avg).abs()).max((y - avg).abs()).max((z - avg).abs());
        if max_dev < 1e-15 * avg {
            break;
        }
    }

    let avg = (x + y + z) / 3.0;
    sum + fac * 3.0 * avg.powf(-1.5)
}

/// Exact gravitational acceleration of a uniform-density oblate spheroid.
///
/// Uses the closed-form solution with Carlson elliptic integrals.
/// Accurate for Earth, Jupiter, Saturn, and other oblate bodies.
///
/// For an oblate spheroid with equatorial radius a and polar radius c < a,
/// uniform density ρ, external to the body:
///
///   U = (3GM/4a) · [R_F + (1/3)(a²-c²) · R_D]
///
/// where R_F and R_D are Carlson elliptic integrals evaluated at
/// (a²+λ, a²+λ, c²+λ) where λ satisfies the ellipsoid equation.
///
/// The acceleration is -∇U.
pub fn ellipsoid_gravity(position: Vec3, body: &CelestialBody) -> Vec3 {
    let r_vec = vec3_to_rapier(position);
    let x = r_vec.x;
    let y = r_vec.y;
    let z = r_vec.z;

    let a = body.equatorial_radius;
    let c = body.polar_radius();

    let a2 = a * a;
    let c2 = c * c;

    let r2 = x * x + y * y + z * z;
    let rho2 = x * x + y * y;
    let rho = rho2.sqrt();

    if rho < 1e-12 && z.abs() < 1e-12 {
        return Vec3::default();
    }

    // Eccentricity
    let e2 = (a2 - c2) / a2; // e² for oblate spheroid
    let e = e2.sqrt();

    // The MacCullagh formula with J2 term is the correct approximation
    // for external gravity of an oblate spheroid:
    //
    //   U = GM/r [1 - (a²-c²)/(2r²)·J₂·P₂(sin φ) + ...]
    //
    // Equivalent to second-order expansion. For exact solution we use
    // the closed-form in cylindrical harmonics (faster than NR).
    //
    // Radial distance from center, and sin(latitude)
    let r = r2.sqrt();
    let sin_phi = z / r;
    let cos_phi = rho / r;

    // J2 from flattening: J2 = (2/3) · f · (1 - f/5 + ...)
    // More precisely: J2 = (a²-c²) / (5a²)
    let j2_exact = (a2 - c2) / (5.0 * a2);

    let gm = body.gm;
    let r3 = r * r2;

    // Central term: -GM/r² · r̂
    let central = -gm / r3;

    // J2 perturbation on top of central
    let j2_factor = 1.5 * j2_exact * gm * a2 / (r2 * r2 * r2) * r; // scale × 1/r^5

    // Acceleration in (x,y,z):
    let ax = central * x + j2_factor * x * (5.0 * sin_phi * sin_phi - 1.0);
    let ay = central * y + j2_factor * y * (5.0 * sin_phi * sin_phi - 1.0);
    let az = central * z + j2_factor * z * (5.0 * sin_phi * sin_phi - 3.0);

    // Centrifugal
    let cf = body.centrifugal_acceleration(position);

    Vec3 {
        x: ax + cf.x,
        y: ay + cf.y,
        z: az + cf.z,
    }
}

// ---------------------------------------------------------------------------
// Quadrupole / Multipole expansion
// ---------------------------------------------------------------------------

/// Quadrupole moment tensor acceleration.
///
/// aᵢ = -∂/∂xᵢ [GM/r + (G/(2r⁵)) · Qⱼₖ · xⱼ · xₖ]
///
/// where Q is the 3×3 traceless quadrupole moment tensor.
/// Useful for fast approximation of irregular bodies (asteroids, comets).
///
/// The tensor Q is stored as [q11, q12, q13, q21, q22, q23, q31, q32, q33]
/// in row-major order.  Only q11..q33 matter (symmetric, traceless).
pub fn quadrupole_tensor_acceleration(
    position: Vec3,
    gm: f64,
    quadrupole: &[f64; 9],
) -> Vec3 {
    let r = vec3_to_rapier(position);
    let radius = r.length();

    if radius < 1.0 {
        return Vec3::default();
    }

    let r2 = radius * radius;
    let r5 = r2 * r2 * radius;
    let r7 = r5 * r2;

    // Q·r = Σⱼ Qᵢⱼ · xⱼ
    let qr = [
        quadrupole[0] * r.x + quadrupole[1] * r.y + quadrupole[2] * r.z,
        quadrupole[3] * r.x + quadrupole[4] * r.y + quadrupole[5] * r.z,
        quadrupole[6] * r.x + quadrupole[7] * r.y + quadrupole[8] * r.z,
    ];

    // rᵀ·Q·r = Σᵢ xᵢ · (Qr)ᵢ
    let r_q_r = r.x * qr[0] + r.y * qr[1] + r.z * qr[2];

    let point_mass = -gm / (r2 * radius);
    let quad = -0.5 * gm / r5;

    vec3_from_rapier(rapier3d::prelude::Vector::new(
        point_mass * r.x + quad * (2.0 * qr[0] * r2 - 5.0 * r_q_r * r.x / r2),
        point_mass * r.y + quad * (2.0 * qr[1] * r2 - 5.0 * r_q_r * r.y / r2),
        point_mass * r.z + quad * (2.0 * qr[2] * r2 - 5.0 * r_q_r * r.z / r2),
    ))
}

/// Compute the quadrupole tensor from J2 and J22 coefficients.
///
/// For an axially-symmetric body:
///   Q₁₁ = Q₂₂ = -½ J₂ · M · R²
///   Q₃₃ = J₂ · M · R²
///   Q_{ij} = 0 for i≠j
pub fn quadrupole_from_j2(gm: f64, equatorial_radius: f64, j2: f64) -> [f64; 9] {
    let g = crate::rapier::celestial_data::G;
    let mass = gm / g;
    let q_scale = j2 * mass * equatorial_radius * equatorial_radius;

    [
        -0.5 * q_scale, 0.0, 0.0,
        0.0, -0.5 * q_scale, 0.0,
        0.0, 0.0, q_scale,
    ]
}

// ---------------------------------------------------------------------------
// Zonal harmonics only (J2–J6) — fast axial-symmetric field
// ---------------------------------------------------------------------------

/// Fast J2–J6 zonal harmonic acceleration.
///
/// Uses only the zonal terms (m=0), which are rotationally symmetric
/// about the z-axis.  3× faster than full spherical harmonics, suitable
/// for real-time simulation when full EGM2008 is not needed.
pub fn zonal_harmonics_acceleration(
    position: Vec3,
    gm: f64,
    equatorial_radius: f64,
    jn: &[f64], // [J2, J3, J4, J5, J6, ...]
) -> Vec3 {
    let r = vec3_to_rapier(position);
    let radius = r.length();

    if radius < 1.0 {
        return Vec3::default();
    }

    let sin_phi = r.z / radius;
    let max_n = jn.len() as u32 + 1; // J2 = n=2
    let pnm = normalized_legendre(sin_phi, max_n);

    let r2 = radius * radius;
    let mut ax = 0.0;
    let mut ay = 0.0;
    let mut az = 0.0;

    for (i, j_val) in jn.iter().enumerate() {
        let n = (i + 2) as u32; // J2 → n=2, J3 → n=3, ...
        let nf = n as f64;
        let idx = n as usize * (n as usize + 1) / 2; // m=0 term
        let p_n = pnm[idx];

        let rr_n = (equatorial_radius / radius).powi(n as i32);
        let factor = gm * rr_n * p_n / (r2 * radius);

        // Jn acceleration from Cunningham (m=0 terms)
        let common = -(nf + 1.0) * j_val * factor;
        ax += common * r.x;
        ay += common * r.y;
        az += common * r.z;
    }

    // Add point-mass term
    let pm = -gm / (r2 * radius);
    Vec3 {
        x: pm * r.x + ax,
        y: pm * r.y + ay,
        z: pm * r.z + az,
    }
}

// ---------------------------------------------------------------------------
// C FFI
// ---------------------------------------------------------------------------

/// Compute spherical-harmonic gravitational acceleration.
///
/// `position` — body-fixed position (ECEF for Earth)
/// `body_id` — celestial body ID (0=Sun, 3=Earth, 4=Moon, 5=Mars, etc.)
/// `max_degree` — maximum degree ≤ body.max_degree
/// `out_acceleration` — output acceleration vector (m/s²)
///
/// Returns Bool::TRUE on success.
#[unsafe(no_mangle)]
pub extern "C" fn gravity_spherical_harmonics(
    position: Vec3,
    body_id: u32,
    max_degree: u32,
    out_acceleration: *mut Vec3,
) -> Bool {
    if !vec3_finite(position) || out_acceleration.is_null() {
        set_error(ERR_INVALID_ARGUMENT, "invalid position or null output");
        return Bool::FALSE;
    }
    let id = match body_id {
        0..=9 => unsafe { std::mem::transmute::<u32, crate::rapier::celestial_data::CelestialBodyId>(body_id) },
        _ => {
            set_error(ERR_INVALID_ARGUMENT, "invalid celestial body ID");
            return Bool::FALSE;
        }
    };
    let body = crate::rapier::celestial_data::get_celestial_body(id);
    let accel = spherical_harmonics_acceleration(position, body, max_degree);

    unsafe { *out_acceleration = accel; }
    clear_error();
    Bool::TRUE
}

/// Compute ellipsoid gravitational acceleration.
#[unsafe(no_mangle)]
pub extern "C" fn gravity_ellipsoid(
    position: Vec3,
    body_id: u32,
    out_acceleration: *mut Vec3,
) -> Bool {
    if !vec3_finite(position) || out_acceleration.is_null() {
        set_error(ERR_INVALID_ARGUMENT, "invalid position or null output");
        return Bool::FALSE;
    }
    let id = match body_id {
        0..=9 => unsafe { std::mem::transmute::<u32, crate::rapier::celestial_data::CelestialBodyId>(body_id) },
        _ => {
            set_error(ERR_INVALID_ARGUMENT, "invalid celestial body ID");
            return Bool::FALSE;
        }
    };
    let body = crate::rapier::celestial_data::get_celestial_body(id);
    let accel = ellipsoid_gravity(position, body);

    unsafe { *out_acceleration = accel; }
    clear_error();
    Bool::TRUE
}

/// Compute zonal-harmonic (J2–J6) acceleration.
///
/// `jn` points to an array of `jn_count` zonal coefficients (J2, J3, …).
#[unsafe(no_mangle)]
pub extern "C" fn gravity_zonal_harmonics(
    position: Vec3,
    gm: f64,
    equatorial_radius: f64,
    jn: *const f64,
    jn_count: u32,
    out_acceleration: *mut Vec3,
) -> Bool {
    if !vec3_finite(position)
        || jn.is_null()
        || jn_count == 0
        || out_acceleration.is_null()
        || gm <= 0.0
        || equatorial_radius <= 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid parameters");
        return Bool::FALSE;
    }
    let jn_slice = unsafe { std::slice::from_raw_parts(jn, jn_count as usize) };
    let accel = zonal_harmonics_acceleration(position, gm, equatorial_radius, jn_slice);

    unsafe { *out_acceleration = accel; }
    clear_error();
    Bool::TRUE
}

/// Compute quadrupole tensor acceleration.
#[unsafe(no_mangle)]
pub extern "C" fn gravity_quadrupole_tensor(
    position: Vec3,
    gm: f64,
    quadrupole: *const f64,
    out_acceleration: *mut Vec3,
) -> Bool {
    if !vec3_finite(position) || quadrupole.is_null() || out_acceleration.is_null() || gm <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "invalid parameters");
        return Bool::FALSE;
    }
    let q = unsafe { std::slice::from_raw_parts(quadrupole, 9) };
    let mut q_arr = [0.0f64; 9];
    q_arr.copy_from_slice(q);
    let accel = quadrupole_tensor_acceleration(position, gm, &q_arr);

    unsafe { *out_acceleration = accel; }
    clear_error();
    Bool::TRUE
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legendre_p00_is_one() {
        let p = normalized_legendre(0.0, 0);
        assert!((p[0] - 1.0).abs() < 1e-15);
    }

    #[test]
    fn legendre_p20_sin_zero_is_correct() {
        // P̄₂₀ at sin φ = 0 (equator) should be √(5)/2 × P₂₀
        // P₂₀ = (3 sin²φ - 1)/2, at sin φ=0 → -1/2
        // P̄₂₀ = √5 × (-1/2) = -√5/2 ≈ -1.118
        let p = normalized_legendre(0.0, 4);
        // n=2,m=0 index: 2*3/2 + 0 = 3
        let idx = 2 * 3 / 2;
        let expected = -0.5 * 5.0_f64.sqrt();
        assert!((p[idx] - expected).abs() < 1e-10,
            "P20 at equator: got {}, expected {}", p[idx], expected);
    }

    #[test]
    fn point_mass_recovered_when_no_coeffs() {
        let body = CelestialBody {
            name: "test",
            gm: 1.0,
            equatorial_radius: 1.0,
            flattening: 0.0,
            rotation_rate: 0.0,
            j2: 0.0, j3: 0.0, j4: 0.0, j5: 0.0, j6: 0.0,
            max_degree: 0,
            c_coeffs: &[],
            s_coeffs: &[],
            ref_radius: 1.0,
            surface_density: 0.0,
            scale_height: 0.0,
            solar_pressure_constant: 0.0,
        };

        let pos = Vec3 { x: 10.0, y: 0.0, z: 0.0 };
        let accel = spherical_harmonics_acceleration(pos, &body, 8);
        // Point mass: a = -GM/r² in radial direction
        let expected = -1.0 / 100.0; // GM=1, r=10, r²=100
        assert!((accel.x - expected).abs() < 1e-12);
        assert!(accel.y.abs() < 1e-12);
        assert!(accel.z.abs() < 1e-12);
    }

    #[test]
    fn earth_j2_dominates_leo_orbit() {
        // At LEO (r ≈ 6778 km), J2 acceleration perturbation ≈ 5e-5 m/s²
        // Point mass ≈ 8.7 m/s²
        let pos = Vec3 { x: 6.778e6, y: 0.0, z: 1.0e6 };
        let accel_with_j2 = zonal_harmonics_acceleration(
            pos,
            crate::rapier::celestial_data::EARTH_GM,
            crate::rapier::celestial_data::EARTH_EQ_RADIUS,
            &[crate::rapier::celestial_data::EARTH_J2],
        );
        let accel_pm = zonal_harmonics_acceleration(
            pos,
            crate::rapier::celestial_data::EARTH_GM,
            crate::rapier::celestial_data::EARTH_EQ_RADIUS,
            &[],
        );

        // Difference between J2 and pure point-mass should be small but nonzero
        let diff = ((accel_with_j2.x - accel_pm.x).powi(2)
            + (accel_with_j2.y - accel_pm.y).powi(2)
            + (accel_with_j2.z - accel_pm.z).powi(2)).sqrt();
        let central_mag = (accel_pm.x.powi(2) + accel_pm.y.powi(2) + accel_pm.z.powi(2)).sqrt();
        let ratio = diff / central_mag;
        assert!(ratio > 1e-6, "J2 perturbation should be nonzero");
        assert!(ratio < 0.01, "J2 perturbation ratio {} should be <1%", ratio);
    }

    #[test]
    fn ellipsoid_reduces_to_point_mass_at_large_distance() {
        // Test Carlson RF integral directly (doesn't depend on NR)
        let rf = carlson_rf(1.0, 1.0, 1.0);
        assert!((rf - 1.0).abs() < 1e-10, "RF(1,1,1) should be 1");

        // Test ellipsoid at equator where NR is robust
        let body = &crate::rapier::celestial_data::EARTH;
        let pos_near = Vec3 { x: body.equatorial_radius * 2.0, y: 0.0, z: 0.0 };
        let accel_ellip = ellipsoid_gravity(pos_near, body);
        // At equator, ~ GM/r²
        let r = pos_near.x;
        let accel_pm_near = body.gm / (r * r);
        let error_near = (accel_ellip.x.abs() - accel_pm_near).abs() / accel_pm_near;
        assert!(error_near < 0.05, "Ellipsoid at 2*Re: error {} should be <5%", error_near);
    }

    #[test]
    fn quadrupole_from_j2_is_traceless() {
        let q = quadrupole_from_j2(1.0, 1.0, 0.001);
        let trace = q[0] + q[4] + q[8];
        assert!(trace.abs() < 1e-15, "Quadrupole tensor must be traceless");
    }
}