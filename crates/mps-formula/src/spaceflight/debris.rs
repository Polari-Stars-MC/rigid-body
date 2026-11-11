//! `spaceflight::debris` submodule — debris & environment (collision probability, atomic oxygen, radiation dose, Whipple shield, surface charging, airlock, Chandrasekhar/Schwarzschild/Lense-Thirring, eclipse, Lagrange/CR3BP)
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

pub fn airlock_depressurization(
    pressure: f64,
    ambient_pressure: f64,
    volume: f64,
    conductance: f64,
    dt: f64,
) -> Option<AirlockDepressurization> {
    if !finite(&[pressure, ambient_pressure, volume, conductance, dt])
        || volume <= 0.0
        || conductance < 0.0
        || dt < 0.0
    {
        return None;
    }
    let rate = -conductance / volume * (pressure - ambient_pressure);
    Some(AirlockDepressurization {
        pressure: ambient_pressure
            + (pressure - ambient_pressure) * (-conductance * dt / volume).exp(),
        pressure_rate: rate,
    })
}

pub fn atomic_oxygen_erosion(
    fluence: f64,
    erosion_yield: f64,
    area: f64,
    density: f64,
) -> Option<AtomicOxygenErosion> {
    if !finite(&[fluence, erosion_yield, area, density])
        || fluence < 0.0
        || erosion_yield < 0.0
        || area < 0.0
        || density < 0.0
    {
        return None;
    }
    let volume_loss = fluence * erosion_yield * area;
    Some(AtomicOxygenErosion {
        volume_loss,
        mass_loss: volume_loss * density,
    })
}

/// Chandrasekhar mass limit (1.44 solar masses).
pub fn chandrasekhar_mass() -> f64 {
    1.44
}

/// CR3BP equations of motion (3D).
/// Returns acceleration (ax, ay, az) in the rotating frame.
pub fn cr3bp_acceleration(position: Vec3, mu: f64) -> Option<Vec3> {
    if !vec3_finite(position) || !mu.is_finite() || !(0.0..=1.0).contains(&mu) {
        return None;
    }
    let x = position.x;
    let y = position.y;
    let z = position.z;
    let r1 = ((x + mu).powi(2) + y * y + z * z).sqrt();
    let r2 = ((x - 1.0 + mu).powi(2) + y * y + z * z).sqrt();
    if r1 < EPS || r2 < EPS {
        return None;
    }
    let r13 = r1 * r1 * r1;
    let r23 = r2 * r2 * r2;
    let om1 = (1.0 - mu) / r13;
    let om2 = mu / r23;
    Some(Vec3 {
        x: x + 2.0 * y - om1 * (x + mu) - om2 * (x - 1.0 + mu),
        y: y - 2.0 * x - om1 * y - om2 * y,
        z: -om1 * z - om2 * z,
    })
}

/// CR3BP Jacobi constant for a given state and mass ratio.
pub fn cr3bp_jacobi_constant(position: Vec3, velocity: Vec3, mu: f64) -> Option<f64> {
    if !vec3_finite(position)
        || !vec3_finite(velocity)
        || !mu.is_finite()
        || !(0.0..=1.0).contains(&mu)
    {
        return None;
    }
    let x = position.x;
    let y = position.y;
    let z = position.z;
    let r1 = ((x + mu).powi(2) + y * y + z * z).sqrt();
    let r2 = ((x - 1.0 + mu).powi(2) + y * y + z * z).sqrt();
    if r1 < EPS || r2 < EPS {
        return None;
    }
    let omega = (1.0 - mu) / r1 + mu / r2;
    let v2 = velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z;
    Some(x * x + y * y + 2.0 * omega - v2)
}

/// Circular restricted 3-body: L1/L2/L3 position (x coordinate in rotating frame).
pub fn cr3bp_lagrange_x(mu: f64, point: u8) -> Option<f64> {
    let gamma = lagrange_collinear_gamma(mu, point)?;
    match point {
        1 => Some(1.0 - mu - gamma), // L1: between bodies
        2 => Some(1.0 - mu + gamma), // L2: beyond secondary
        3 => Some(-mu - gamma),      // L3: opposite side
        _ => None,
    }
}

pub fn debris_collision_probability(
    miss_distance: f64,
    combined_radius: f64,
    sigma_radial: f64,
    sigma_intrack: f64,
) -> Option<CollisionProbability> {
    if !finite(&[miss_distance, combined_radius, sigma_radial, sigma_intrack])
        || combined_radius < 0.0
        || sigma_radial <= 0.0
        || sigma_intrack <= 0.0
    {
        return None;
    }
    let sigma = (sigma_radial * sigma_intrack).sqrt();
    let probability = (combined_radius * combined_radius / (2.0 * sigma_radial * sigma_intrack))
        * (-0.5 * miss_distance * miss_distance / (sigma * sigma)).exp();
    Some(CollisionProbability {
        probability: probability.clamp(0.0, 1.0),
        combined_sigma: sigma,
    })
}

/// Eclipsing duration for a circular orbit (conical shadow model).
pub fn eclipse_duration_circular(
    semi_major_axis: f64,
    mu: f64,
    planet_radius: f64,
    sun_direction: Vec3,
    orbit_normal: Vec3,
) -> Option<f64> {
    if !finite(&[semi_major_axis, mu, planet_radius])
        || semi_major_axis <= 0.0
        || mu <= 0.0
        || planet_radius <= 0.0
    {
        return None;
    }
    if !vec3_finite(sun_direction) || !vec3_finite(orbit_normal) {
        return None;
    }
    let beta = vec3_to_rapier(sun_direction)
        .dot(vec3_to_rapier(orbit_normal))
        .clamp(-1.0, 1.0)
        .acos(); // angle between sun and orbit plane
    if beta.abs() < EPS {
        return None;
    } // no eclipse possible
    let rho = (planet_radius / semi_major_axis).asin();
    if rho >= beta {
        return None;
    }
    let cos_half = beta.cos() / rho.cos();
    if cos_half.abs() > 1.0 {
        return None;
    }
    let n = (mu / (semi_major_axis * semi_major_axis * semi_major_axis)).sqrt();
    Some(2.0 * cos_half.acos() / n)
}

/// Lagrange point positions (collinear L1, L2, L3) relative to the primary.
/// Returns the distance from the secondary body in units of the orbital separation.
/// mu = m2 / (m1 + m2) where m2 is the smaller body.
pub fn lagrange_collinear_gamma(mu: f64, point: u8) -> Option<f64> {
    if !mu.is_finite() || mu <= 0.0 || mu >= 1.0 || !(1..=3).contains(&point) {
        return None;
    }
    let gamma = match point {
        1 => {
            // L1: between primary and secondary
            let xi = (mu / (3.0 * (1.0 - mu))).cbrt();
            xi - xi * xi / 3.0 - xi * xi * xi / 9.0
        }
        2 => {
            // L2: beyond secondary
            let xi = (mu / (3.0 * (1.0 - mu))).cbrt();
            xi + xi * xi / 3.0 - xi * xi * xi / 9.0
        }
        3 => {
            // L3: opposite side of primary
            let nu = mu / (1.0 - mu);
            1.0 - (7.0 / 12.0) * nu + (7.0 / 12.0) * nu * nu
        }
        _ => return None,
    };
    Some(gamma)
}

/// L4/L5 equilateral Lagrange point coordinates relative to the barycenter.
/// Returns (x, y) in units of the separation distance.
pub fn lagrange_equilateral_coords() -> (f64, f64, f64, f64) {
    // L4: (0.5 - mu, sqrt(3)/2), L5: (0.5 - mu, -sqrt(3)/2)
    let sqrt3_2 = 0.866_025_403_784_438_6;
    (0.5, sqrt3_2, 0.5, -sqrt3_2)
}

/// Lense-Thirring precession rate (rad/s) for a test particle.
/// Omega_LT = (2GJ)/(c² r³)  where J = I * omega_spin is the angular momentum.
pub fn lense_thirring_precession_rate(
    mass_kg: f64,
    radius_m: f64,
    spin_parameter: f64,
) -> Option<f64> {
    let g = 6.67430e-11;
    let c = 299_792_458.0;
    if !finite(&[mass_kg, radius_m, spin_parameter]) || mass_kg <= 0.0 || radius_m <= 0.0 {
        return None;
    }
    let j = spin_parameter * mass_kg * mass_kg * g / c; // a = Jc/(GM²) → J = aGM²/c
    Some(2.0 * g * j / (c * c * radius_m * radius_m * radius_m))
}

/// [`radiation_absorbed_dose`] 的 checked 变体：显式域校验（质量必须为正，
/// 品质因子非负），结果非有限同样报错。
pub fn radiation_absorbed_dose_checked(
    energy_joules: f64,
    mass_kg: f64,
    quality_factor: f64,
) -> Result<f64, crate::FormulaError> {
    use crate::FormulaError;
    if !energy_joules.is_finite() {
        return Err(FormulaError::NonFiniteInput {
            parameter: "energy_joules",
        });
    }
    if mass_kg <= 0.0 || !mass_kg.is_finite() {
        return Err(FormulaError::OutOfDomain {
            parameter: "mass_kg",
            requirement: "> 0 kg",
        });
    }
    if quality_factor < 0.0 || !quality_factor.is_finite() {
        return Err(FormulaError::OutOfDomain {
            parameter: "quality_factor",
            requirement: ">= 0 (dimensionless)",
        });
    }
    crate::domain::finite_result(energy_joules / mass_kg * quality_factor, "absorbed dose")
}

pub fn radiation_absorbed_dose(
    energy_joules: f64,
    mass_kg: f64,
    quality_factor: f64,
) -> Option<f64> {
    radiation_absorbed_dose_checked(energy_joules, mass_kg, quality_factor).ok()
}

/// Schwarzschild ISCO: r_isco = 6GM/c² (already in relativity, keep here for convenience).
pub fn schwarzschild_isco_radius(mass_kg: f64) -> Option<f64> {
    let g = 6.67430e-11;
    let c = 299_792_458.0;
    if !mass_kg.is_finite() || mass_kg <= 0.0 {
        return None;
    }
    Some(6.0 * g * mass_kg / (c * c))
}

pub fn surface_charging_current_balance(
    photo_current: f64,
    secondary_current: f64,
    backscatter_current: f64,
    electron_current: f64,
    ion_current: f64,
) -> Option<f64> {
    if !finite(&[
        photo_current,
        secondary_current,
        backscatter_current,
        electron_current,
        ion_current,
    ]) {
        return None;
    }
    Some(photo_current + secondary_current + backscatter_current + ion_current - electron_current)
}

pub fn whipple_critical_projectile_diameter(
    bumper_thickness: f64,
    bumper_density: f64,
    projectile_density: f64,
    impact_velocity: f64,
    standoff: f64,
) -> Option<f64> {
    if !finite(&[
        bumper_thickness,
        bumper_density,
        projectile_density,
        impact_velocity,
        standoff,
    ]) || bumper_thickness <= 0.0
        || bumper_density <= 0.0
        || projectile_density <= 0.0
        || impact_velocity <= 0.0
        || standoff <= 0.0
    {
        return None;
    }
    Some(
        bumper_thickness
            * (bumper_density / projectile_density).sqrt()
            * (standoff / bumper_thickness).powf(1.0 / 3.0)
            * (7_000.0 / impact_velocity).powf(2.0 / 3.0),
    )
}
