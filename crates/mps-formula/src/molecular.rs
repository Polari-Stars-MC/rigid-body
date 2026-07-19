//! Molecular dynamics formulas — Lennard-Jones, Coulomb, and pairwise interactions.
//!
//! Pure computation only — no access to `WorldHandle`, `RigidBody`, or Rapier state.
//! All input/output types are from `crate::ffi::types`.

use crate::ffi::{
    Bool, MolecularForceLaw, MolecularPairReport, MolecularParticle, Vec3,
    vec3_finite,
};
use crate::math::{finite_non_negative, finite_positive};

const EPSILON: f64 = 1.0e-12;
const VACUUM_COULOMB_CONSTANT: f64 = 8.987_551_792_3e9;

fn particle_valid(particle: MolecularParticle) -> bool {
    vec3_finite(particle.position)
        && vec3_finite(particle.velocity)
        && particle.mass.is_finite()
        && particle.mass >= 0.0
        && particle.charge.is_finite()
        && finite_non_negative(particle.epsilon)
        && finite_positive(particle.sigma)
}

fn force_law_valid(law: MolecularForceLaw) -> bool {
    let coulomb_constant = if law.coulomb_constant == 0.0 {
        VACUUM_COULOMB_CONSTANT
    } else {
        law.coulomb_constant
    };
    coulomb_constant.is_finite()
        && finite_positive(law.relative_permittivity)
        && finite_non_negative(law.cutoff_radius)
        && finite_non_negative(law.softening)
}

fn normalized_force_law(law: MolecularForceLaw) -> MolecularForceLaw {
    MolecularForceLaw {
        coulomb_constant: if law.coulomb_constant == 0.0 {
            VACUUM_COULOMB_CONSTANT
        } else {
            law.coulomb_constant
        },
        relative_permittivity: law.relative_permittivity,
        cutoff_radius: law.cutoff_radius,
        softening: law.softening,
        lennard_jones_enabled: law.lennard_jones_enabled,
        coulomb_enabled: law.coulomb_enabled,
    }
}

fn effective_distance_sq(displacement: &Vec3, softening: f64) -> f64 {
    let dx = displacement.x;
    let dy = displacement.y;
    let dz = displacement.z;
    dx * dx + dy * dy + dz * dz + softening * softening
}

/// Lennard-Jones potential and force between a pair.
/// Returns (potential, force_vector).
fn lennard_jones_pair(
    epsilon: f64,
    sigma: f64,
    displacement: &Vec3,
    distance: f64,
) -> (f64, Vec3) {
    if epsilon == 0.0 || sigma <= 0.0 || distance <= EPSILON {
        return (0.0, Vec3::default());
    }
    let inv_r = 1.0 / distance;
    let sr = sigma * inv_r;
    let sr2 = sr * sr;
    let sr6 = sr2 * sr2 * sr2;
    let sr12 = sr6 * sr6;
    let potential = 4.0 * epsilon * (sr12 - sr6);
    let force_scale = 24.0 * epsilon * (2.0 * sr12 - sr6) * inv_r * inv_r;
    (
        potential,
        Vec3 {
            x: displacement.x * force_scale,
            y: displacement.y * force_scale,
            z: displacement.z * force_scale,
        },
    )
}

/// Coulomb potential and force between two charges.
fn coulomb_pair(
    charge_a: f64,
    charge_b: f64,
    law: &MolecularForceLaw,
    displacement: &Vec3,
    distance: f64,
) -> (f64, Vec3) {
    if distance <= EPSILON || charge_a == 0.0 || charge_b == 0.0 {
        return (0.0, Vec3::default());
    }
    let coefficient = law.coulomb_constant * charge_a * charge_b / law.relative_permittivity;
    let potential = coefficient / distance;
    let inv_r3 = 1.0 / (distance * distance * distance);
    let force = coefficient * inv_r3;
    (
        potential,
        Vec3 {
            x: displacement.x * force,
            y: displacement.y * force,
            z: displacement.z * force,
        },
    )
}

/// Mixed Lennard-Jones parameters using Lorentz-Berthelot rules.
fn mixed_lennard_jones(a: &MolecularParticle, b: &MolecularParticle) -> (f64, f64) {
    let epsilon = (a.epsilon * b.epsilon).sqrt();
    let sigma = 0.5 * (a.sigma + b.sigma);
    (epsilon, sigma)
}

/// Compute the full pairwise interaction between two particles.
pub fn compute_pair_interaction(
    a: MolecularParticle,
    b: MolecularParticle,
    law: MolecularForceLaw,
) -> Option<MolecularPairReport> {
    if !particle_valid(a) || !particle_valid(b) || !force_law_valid(law) {
        return None;
    }
    let law = normalized_force_law(law);
    let displacement = Vec3 {
        x: a.position.x - b.position.x,
        y: a.position.y - b.position.y,
        z: a.position.z - b.position.z,
    };
    let distance = effective_distance_sq(&displacement, law.softening).sqrt();
    if law.cutoff_radius > 0.0 && distance > law.cutoff_radius {
        return Some(MolecularPairReport {
            displacement,
            distance,
            ..MolecularPairReport::default()
        });
    }

    let (epsilon, sigma) = mixed_lennard_jones(&a, &b);
    let (lj_potential, lj_force) = if law.lennard_jones_enabled.0 != 0 {
        lennard_jones_pair(epsilon, sigma, &displacement, distance)
    } else {
        (0.0, Vec3::default())
    };
    let (coulomb_potential, coulomb_force) = if law.coulomb_enabled.0 != 0 {
        coulomb_pair(a.charge, b.charge, &law, &displacement, distance)
    } else {
        (0.0, Vec3::default())
    };

    Some(MolecularPairReport {
        displacement,
        distance,
        lennard_jones_potential: lj_potential,
        coulomb_potential,
        total_potential: lj_potential + coulomb_potential,
        lennard_jones_force: lj_force,
        coulomb_force,
        total_force: Vec3 {
            x: lj_force.x + coulomb_force.x,
            y: lj_force.y + coulomb_force.y,
            z: lj_force.z + coulomb_force.z,
        },
    })
}

/// Lennard-Jones potential for a given distance.
pub fn lennard_jones_potential(distance: f64, epsilon: f64, sigma: f64) -> Option<f64> {
    if !finite_positive(distance) || !finite_non_negative(epsilon) || !finite_positive(sigma) {
        return None;
    }
    let sr = sigma / distance;
    let sr6 = sr.powi(6);
    Some(4.0 * epsilon * (sr6 * sr6 - sr6))
}

/// Lennard-Jones force vector for a given displacement.
pub fn lennard_jones_force(
    displacement: Vec3,
    epsilon: f64,
    sigma: f64,
    softening: f64,
) -> Option<Vec3> {
    if !vec3_finite(displacement) || !finite_non_negative(epsilon) || !finite_positive(sigma) || !finite_non_negative(softening) {
        return None;
    }
    let distance = effective_distance_sq(&displacement, softening).sqrt();
    let (_, force) = lennard_jones_pair(epsilon, sigma, &displacement, distance);
    Some(force)
}

/// Coulomb potential for a given distance.
pub fn coulomb_potential(
    distance: f64,
    charge_a: f64,
    charge_b: f64,
    coulomb_constant: f64,
    relative_permittivity: f64,
) -> Option<f64> {
    let law = MolecularForceLaw {
        coulomb_constant,
        relative_permittivity,
        ..MolecularForceLaw::default()
    };
    if !finite_positive(distance) || !charge_a.is_finite() || !charge_b.is_finite() || !force_law_valid(law) {
        return None;
    }
    let law = normalized_force_law(law);
    Some(law.coulomb_constant * charge_a * charge_b / (relative_permittivity * distance))
}

/// Coulomb force vector for a given displacement.
pub fn coulomb_force(
    displacement: Vec3,
    charge_a: f64,
    charge_b: f64,
    coulomb_constant: f64,
    relative_permittivity: f64,
    softening: f64,
) -> Option<Vec3> {
    let law = MolecularForceLaw {
        coulomb_constant,
        relative_permittivity,
        softening,
        coulomb_enabled: Bool::TRUE,
        ..MolecularForceLaw::default()
    };
    if !vec3_finite(displacement) || !charge_a.is_finite() || !charge_b.is_finite() || !force_law_valid(law) {
        return None;
    }
    let law = normalized_force_law(law);
    let distance = effective_distance_sq(&displacement, law.softening).sqrt();
    let (_, force) = coulomb_pair(charge_a, charge_b, &law, &displacement, distance);
    Some(force)
}

/// Vacuum Coulomb constant.
pub fn vacuum_coulomb_constant() -> f64 {
    VACUUM_COULOMB_CONSTANT
}