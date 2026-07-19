use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, MolecularForceLaw, MolecularPairReport, MolecularParticle, RigidBodyHandleRaw, Vec3,
    WorldHandle, unpack_rigid_body_handle, vec3_finite, vec3_to_rapier,
};

#[unsafe(no_mangle)]
pub extern "C" fn molecular_lennard_jones_potential(
    distance: f64,
    epsilon: f64,
    sigma: f64,
) -> f64 {
    match mps_formula::molecular::lennard_jones_potential(distance, epsilon, sigma) {
        Some(v) => { clear_error(); v }
        None => { set_error(ERR_INVALID_ARGUMENT, "invalid Lennard-Jones potential parameters"); f64::NAN }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn molecular_lennard_jones_force(
    displacement: Vec3,
    epsilon: f64,
    sigma: f64,
    softening: f64,
    out_force: *mut Vec3,
) -> Bool {
    if !vec3_finite(displacement) {
        set_error(ERR_INVALID_ARGUMENT, "invalid Lennard-Jones force parameters");
        return Bool::FALSE;
    }
    let Some(out_force) = (unsafe { out_force.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Lennard-Jones force output is null");
        return Bool::FALSE;
    };
    match mps_formula::molecular::lennard_jones_force(displacement, epsilon, sigma, softening) {
        Some(f) => { *out_force = f; clear_error(); Bool::TRUE }
        None => { set_error(ERR_INVALID_ARGUMENT, "invalid Lennard-Jones force parameters"); Bool::FALSE }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn molecular_coulomb_potential(
    distance: f64,
    charge_a: f64,
    charge_b: f64,
    coulomb_constant: f64,
    relative_permittivity: f64,
) -> f64 {
    match mps_formula::molecular::coulomb_potential(distance, charge_a, charge_b, coulomb_constant, relative_permittivity) {
        Some(v) => { clear_error(); v }
        None => { set_error(ERR_INVALID_ARGUMENT, "invalid Coulomb potential parameters"); f64::NAN }
    }
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
    if !vec3_finite(displacement) {
        set_error(ERR_INVALID_ARGUMENT, "invalid Coulomb force parameters");
        return Bool::FALSE;
    }
    let Some(out_force) = (unsafe { out_force.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Coulomb force output is null");
        return Bool::FALSE;
    };
    match mps_formula::molecular::coulomb_force(displacement, charge_a, charge_b, coulomb_constant, relative_permittivity, softening) {
        Some(f) => { *out_force = f; clear_error(); Bool::TRUE }
        None => { set_error(ERR_INVALID_ARGUMENT, "invalid Coulomb force parameters"); Bool::FALSE }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn molecular_pair_interaction(
    particle_a: MolecularParticle,
    particle_b: MolecularParticle,
    law: MolecularForceLaw,
    out_report: *mut MolecularPairReport,
) -> Bool {
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "molecular pair output is null");
        return Bool::FALSE;
    };
    match mps_formula::molecular::compute_pair_interaction(particle_a, particle_b, law) {
        Some(report) => { *out_report = report; clear_error(); Bool::TRUE }
        None => { set_error(ERR_INVALID_ARGUMENT, "invalid molecular pair parameters"); Bool::FALSE }
    }
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
    let Some(report) = mps_formula::molecular::compute_pair_interaction(particle_a, particle_b, law) else {
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
    mps_formula::molecular::vacuum_coulomb_constant()
}