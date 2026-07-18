use rapier3d::prelude::Vector;

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, MolecularForceLaw, MolecularPairReport, MolecularParticle, RigidBodyHandleRaw, Vec3,
    WorldHandle, unpack_rigid_body_handle, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

use crate::rapier::math::{finite_non_negative, finite_positive};

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

fn effective_distance(displacement: Vector, softening: f64) -> f64 {
    (displacement.length_squared() + softening * softening).sqrt()
}

fn lennard_jones_pair(
    epsilon: f64,
    sigma: f64,
    displacement: Vector,
    distance: f64,
) -> (f64, Vector) {
    if epsilon == 0.0 || sigma <= 0.0 || distance <= EPSILON {
        return (0.0, Vector::ZERO);
    }
    let inv_r = 1.0 / distance;
    let sr = sigma * inv_r;
    let sr2 = sr * sr;
    let sr6 = sr2 * sr2 * sr2;
    let sr12 = sr6 * sr6;
    let potential = 4.0 * epsilon * (sr12 - sr6);
    let force_scale = 24.0 * epsilon * (2.0 * sr12 - sr6) * inv_r * inv_r;
    (potential, displacement * force_scale)
}

fn coulomb_pair(
    charge_a: f64,
    charge_b: f64,
    law: MolecularForceLaw,
    displacement: Vector,
    distance: f64,
) -> (f64, Vector) {
    if distance <= EPSILON || charge_a == 0.0 || charge_b == 0.0 {
        return (0.0, Vector::ZERO);
    }
    let coefficient = law.coulomb_constant * charge_a * charge_b / law.relative_permittivity;
    let potential = coefficient / distance;
    // Compute 1/r³ as 1/(r² * r) to avoid powi(3) intermediate overflow
    let inv_r3 = 1.0 / (distance * distance * distance);
    let force = displacement * (coefficient * inv_r3);
    (potential, force)
}

fn mixed_lennard_jones(a: MolecularParticle, b: MolecularParticle) -> (f64, f64) {
    let epsilon = (a.epsilon * b.epsilon).sqrt();
    let sigma = 0.5 * (a.sigma + b.sigma);
    (epsilon, sigma)
}

fn compute_pair_interaction(
    a: MolecularParticle,
    b: MolecularParticle,
    law: MolecularForceLaw,
) -> Option<MolecularPairReport> {
    if !particle_valid(a) || !particle_valid(b) || !force_law_valid(law) {
        return None;
    }
    let law = normalized_force_law(law);
    let displacement = vec3_to_rapier(a.position) - vec3_to_rapier(b.position);
    let distance = effective_distance(displacement, law.softening);
    if law.cutoff_radius > 0.0 && distance > law.cutoff_radius {
        return Some(MolecularPairReport {
            displacement: vec3_from_rapier(displacement),
            distance,
            ..MolecularPairReport::default()
        });
    }

    let (epsilon, sigma) = mixed_lennard_jones(a, b);
    let (lj_potential, lj_force) = if law.lennard_jones_enabled.0 != 0 {
        lennard_jones_pair(epsilon, sigma, displacement, distance)
    } else {
        (0.0, Vector::ZERO)
    };
    let (coulomb_potential, coulomb_force) = if law.coulomb_enabled.0 != 0 {
        coulomb_pair(a.charge, b.charge, law, displacement, distance)
    } else {
        (0.0, Vector::ZERO)
    };
    let total_force = lj_force + coulomb_force;

    Some(MolecularPairReport {
        displacement: vec3_from_rapier(displacement),
        distance,
        lennard_jones_potential: lj_potential,
        coulomb_potential,
        total_potential: lj_potential + coulomb_potential,
        lennard_jones_force: vec3_from_rapier(lj_force),
        coulomb_force: vec3_from_rapier(coulomb_force),
        total_force: vec3_from_rapier(total_force),
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn molecular_lennard_jones_potential(
    distance: f64,
    epsilon: f64,
    sigma: f64,
) -> f64 {
    if !finite_positive(distance) || !finite_non_negative(epsilon) || !finite_positive(sigma) {
        return f64::NAN;
    }
    let sr = sigma / distance;
    let sr6 = sr.powi(6);
    4.0 * epsilon * (sr6 * sr6 - sr6)
}

#[unsafe(no_mangle)]
pub extern "C" fn molecular_lennard_jones_force(
    displacement: Vec3,
    epsilon: f64,
    sigma: f64,
    softening: f64,
    out_force: *mut Vec3,
) -> Bool {
    if !vec3_finite(displacement)
        || !finite_non_negative(epsilon)
        || !finite_positive(sigma)
        || !finite_non_negative(softening)
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid Lennard-Jones force parameters",
        );
        return Bool::FALSE;
    }
    let r = vec3_to_rapier(displacement);
    let distance = effective_distance(r, softening);
    let (_, force) = lennard_jones_pair(epsilon, sigma, r, distance);
    let Some(out_force) = (unsafe { out_force.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Lennard-Jones force output is null");
        return Bool::FALSE;
    };
    *out_force = vec3_from_rapier(force);
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn molecular_coulomb_potential(
    distance: f64,
    charge_a: f64,
    charge_b: f64,
    coulomb_constant: f64,
    relative_permittivity: f64,
) -> f64 {
    let law = MolecularForceLaw {
        coulomb_constant,
        relative_permittivity,
        ..MolecularForceLaw::default()
    };
    if !finite_positive(distance)
        || !charge_a.is_finite()
        || !charge_b.is_finite()
        || !force_law_valid(law)
    {
        return f64::NAN;
    }
    normalized_force_law(law).coulomb_constant * charge_a * charge_b
        / (relative_permittivity * distance)
}

#[unsafe(no_mangle)]
pub extern "C" fn molecular_coulomb_force(
    displacement: Vec3,
    charge_a: f64,
    charge_b: f64,
    coulomb_constant: f64,
    relative_permittivity: f64,
    softening: f64,
    out_force: *mut Vec3,
) -> Bool {
    let law = MolecularForceLaw {
        coulomb_constant,
        relative_permittivity,
        softening,
        coulomb_enabled: Bool::TRUE,
        ..MolecularForceLaw::default()
    };
    if !vec3_finite(displacement)
        || !charge_a.is_finite()
        || !charge_b.is_finite()
        || !force_law_valid(law)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Coulomb force parameters");
        return Bool::FALSE;
    }
    let law = normalized_force_law(law);
    let r = vec3_to_rapier(displacement);
    let distance = effective_distance(r, law.softening);
    let (_, force) = coulomb_pair(charge_a, charge_b, law, r, distance);
    let Some(out_force) = (unsafe { out_force.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Coulomb force output is null");
        return Bool::FALSE;
    };
    *out_force = vec3_from_rapier(force);
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn molecular_pair_interaction(
    particle_a: MolecularParticle,
    particle_b: MolecularParticle,
    law: MolecularForceLaw,
    out_report: *mut MolecularPairReport,
) -> Bool {
    let Some(report) = compute_pair_interaction(particle_a, particle_b, law) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid molecular pair parameters");
        return Bool::FALSE;
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "molecular pair output is null");
        return Bool::FALSE;
    };
    *out_report = report;
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn molecular_apply_pair_forces(
    world: *mut WorldHandle,
    body_a: RigidBodyHandleRaw,
    body_b: RigidBodyHandleRaw,
    particle_a: MolecularParticle,
    particle_b: MolecularParticle,
    law: MolecularForceLaw,
    wake_up: Bool,
    out_report: *mut MolecularPairReport,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    if world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(body_a))
        .is_none()
        || world
            .inner
            .bodies
            .get(unpack_rigid_body_handle(body_b))
            .is_none()
    {
        set_error(ERR_NOT_FOUND, "molecular body was not found");
        return Bool::FALSE;
    }
    let Some(report) = compute_pair_interaction(particle_a, particle_b, law) else {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid molecular body force parameters",
        );
        return Bool::FALSE;
    };

    let force_a = vec3_to_rapier(report.total_force);
    if let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(body_a)) {
        body.add_force(force_a, wake_up.0 != 0);
    }
    if let Some(body) = world.inner.bodies.get_mut(unpack_rigid_body_handle(body_b)) {
        body.add_force(-force_a, wake_up.0 != 0);
    }
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = report;
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn molecular_apply_pair_forces_flag(
    world: *mut WorldHandle,
    body_a: RigidBodyHandleRaw,
    body_b: RigidBodyHandleRaw,
    particle_a: MolecularParticle,
    particle_b: MolecularParticle,
    law: MolecularForceLaw,
    wake_up: Bool,
    out_report: *mut MolecularPairReport,
) -> u8 {
    molecular_apply_pair_forces(
        world, body_a, body_b, particle_a, particle_b, law, wake_up, out_report,
    )
    .0
}

#[unsafe(no_mangle)]
pub extern "C" fn molecular_vacuum_coulomb_constant() -> f64 {
    VACUUM_COULOMB_CONSTANT
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::BodyStatus;

    fn particle(x: f64, charge: f64) -> MolecularParticle {
        MolecularParticle {
            position: Vec3 { x, y: 0.0, z: 0.0 },
            velocity: Vec3::default(),
            mass: 1.0,
            charge,
            epsilon: 1.0,
            sigma: 1.0,
        }
    }

    fn law() -> MolecularForceLaw {
        MolecularForceLaw {
            coulomb_constant: 1.0,
            relative_permittivity: 1.0,
            cutoff_radius: 0.0,
            softening: 0.0,
            lennard_jones_enabled: Bool::TRUE,
            coulomb_enabled: Bool::TRUE,
        }
    }

    #[test]
    fn lennard_jones_and_coulomb_formulas_work() {
        let potential = molecular_lennard_jones_potential(2.0_f64.powf(1.0 / 6.0), 1.0, 1.0);
        assert!((potential + 1.0).abs() < 1.0e-12);

        let mut force = Vec3::default();
        assert_eq!(
            molecular_lennard_jones_force(
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0
                },
                1.0,
                1.0,
                0.0,
                &mut force
            ),
            Bool::TRUE
        );
        assert!(force.x > 0.0);

        assert_eq!(
            molecular_coulomb_force(
                Vec3 {
                    x: 2.0,
                    y: 0.0,
                    z: 0.0
                },
                1.0,
                1.0,
                1.0,
                1.0,
                0.0,
                &mut force
            ),
            Bool::TRUE
        );
        assert!((force.x - 0.25).abs() < 1.0e-12);
    }

    #[test]
    fn pair_interaction_reports_force_on_first_particle() {
        let mut report = MolecularPairReport::default();
        assert_eq!(
            molecular_pair_interaction(particle(0.0, 1.0), particle(2.0, -1.0), law(), &mut report),
            Bool::TRUE
        );
        assert!(report.total_potential.is_finite());
        assert!(report.coulomb_force.x > 0.0);
    }

    #[test]
    fn applies_equal_and_opposite_forces_to_bodies() {
        let world = crate::rapier::world::world_create(Vec3::default());
        let builder_a =
            crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        crate::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder_a, 1.0);
        let body_a = crate::rapier::rigid_body::rigid_body_builder_build(builder_a);
        let handle_a = crate::rapier::rigid_body::world_insert_rigid_body(world, body_a);
        let builder_b =
            crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        crate::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder_b, 1.0);
        let body_b = crate::rapier::rigid_body::rigid_body_builder_build(builder_b);
        let handle_b = crate::rapier::rigid_body::world_insert_rigid_body(world, body_b);
        let mut report = MolecularPairReport::default();

        assert_eq!(
            molecular_apply_pair_forces(
                world,
                handle_a,
                handle_b,
                particle(0.0, 1.0),
                particle(2.0, -1.0),
                law(),
                Bool::TRUE,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(report.total_force.x > 0.0);

        crate::rapier::world::world_step(world, 1.0 / 60.0);
        let velocity_a = crate::rapier::rigid_body::rigid_body_get_linvel(world, handle_a);
        let velocity_b = crate::rapier::rigid_body::rigid_body_get_linvel(world, handle_b);
        assert!(velocity_a.x > 0.0);
        assert!(velocity_b.x < 0.0);
        crate::rapier::world::world_destroy(world);
    }
}
