use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    BernoulliReport, Bool, FluidForceReport, FluidVolume, NavierStokesReport, RigidBodyHandleRaw,
    SphForceReport, SphParticle, Vec3, WorldHandle, unpack_rigid_body_handle, vec3_finite,
    vec3_from_rapier, vec3_to_rapier,
};
use crate::rapier::ffi::finite_positive;

#[unsafe(no_mangle)]
pub extern "C" fn fluid_estimate_aabb_forces(
    fluid: FluidVolume,
    body_center: Vec3,
    body_half_extents: Vec3,
    body_volume: f64,
    body_linvel: Vec3,
    body_angvel: Vec3,
    out_report: *mut FluidForceReport,
) -> Bool {
    let Some(report) = mps_formula::fluid::compute_fluid_forces(
        fluid,
        body_center,
        body_half_extents,
        body_volume,
        body_linvel,
        body_angvel,
    ) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid fluid force parameters");
        return Bool::FALSE;
    };

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = report;
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_apply_aabb_forces(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    fluid: FluidVolume,
    body_half_extents: Vec3,
    body_volume: f64,
    wake_up: Bool,
    out_report: *mut FluidForceReport,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(body) = world
        .inner
        .bodies
        .get_mut(unpack_rigid_body_handle(body_handle))
    else {
        set_error(ERR_NOT_FOUND, "body was not found");
        return Bool::FALSE;
    };

    let body_center = vec3_from_rapier(body.center_of_mass());
    let body_linvel = vec3_from_rapier(body.linvel());
    let body_angvel = vec3_from_rapier(body.angvel());
    let Some(report) = mps_formula::fluid::compute_fluid_forces(
        fluid,
        body_center,
        body_half_extents,
        body_volume,
        body_linvel,
        body_angvel,
    ) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid fluid force parameters");
        return Bool::FALSE;
    };

    body.add_force(vec3_to_rapier(report.total_force), wake_up.0 != 0);
    body.add_torque(vec3_to_rapier(report.total_torque), wake_up.0 != 0);
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = report;
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_apply_aabb_forces_flag(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    fluid: FluidVolume,
    body_half_extents: Vec3,
    body_volume: f64,
    wake_up: Bool,
    out_report: *mut FluidForceReport,
) -> u8 {
    fluid_apply_aabb_forces(
        world,
        body_handle,
        fluid,
        body_half_extents,
        body_volume,
        wake_up,
        out_report,
    )
    .0
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_navier_stokes_simplified_step(
    velocity: Vec3,
    advection: Vec3,
    pressure_gradient: Vec3,
    laplacian_velocity: Vec3,
    external_acceleration: Vec3,
    density: f64,
    kinematic_viscosity: f64,
    dt: f64,
    out_report: *mut NavierStokesReport,
) -> Bool {
    let Some(report) = mps_formula::fluid::navier_stokes_simplified_step(
        velocity, advection, pressure_gradient, laplacian_velocity,
        external_acceleration, density, kinematic_viscosity, dt,
    ) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid Navier-Stokes parameters");
        return Bool::FALSE;
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Navier-Stokes output is null");
        return Bool::FALSE;
    };
    *out_report = report;
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_sph_poly6_kernel(distance: f64, smoothing_radius: f64) -> f64 {
    mps_formula::fluid::sph_poly6_kernel(distance, smoothing_radius)
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_sph_spiky_gradient(
    offset: Vec3,
    smoothing_radius: f64,
    out_gradient: *mut Vec3,
) -> Bool {
    if !vec3_finite(offset) {
        set_error(ERR_INVALID_ARGUMENT, "invalid SPH spiky gradient parameters");
        return Bool::FALSE;
    }
    let Some(gradient) = mps_formula::fluid::sph_spiky_gradient(offset, smoothing_radius) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid SPH spiky gradient parameters");
        return Bool::FALSE;
    };
    let Some(out_gradient) = (unsafe { out_gradient.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "SPH gradient output is null");
        return Bool::FALSE;
    };
    *out_gradient = gradient;
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_sph_viscosity_laplacian(distance: f64, smoothing_radius: f64) -> f64 {
    mps_formula::fluid::sph_viscosity_laplacian(distance, smoothing_radius)
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_sph_estimate_density(
    position: Vec3,
    particles: *const SphParticle,
    particle_count: u32,
    smoothing_radius: f64,
    out_density: *mut f64,
) -> Bool {
    if !vec3_finite(position) || !finite_positive(smoothing_radius) {
        set_error(ERR_INVALID_ARGUMENT, "invalid SPH density parameters");
        return Bool::FALSE;
    }
    if particles.is_null() && particle_count > 0 {
        set_error(ERR_NULL_POINTER, "SPH particle pointer is null");
        return Bool::FALSE;
    }
    let particles = unsafe { std::slice::from_raw_parts(particles, particle_count as usize) };
    let Some(density) = mps_formula::fluid::sph_estimate_density(position, particles, smoothing_radius) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid SPH particle");
        return Bool::FALSE;
    };
    let Some(out_density) = (unsafe { out_density.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "SPH density output is null");
        return Bool::FALSE;
    };
    *out_density = density;
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_sph_estimate_forces(
    particle: SphParticle,
    particles: *const SphParticle,
    particle_count: u32,
    smoothing_radius: f64,
    gas_constant: f64,
    rest_density: f64,
    viscosity: f64,
    surface_tension: f64,
    out_report: *mut SphForceReport,
) -> Bool {
    if particles.is_null() && particle_count > 0 {
        set_error(ERR_NULL_POINTER, "SPH particle pointer is null");
        return Bool::FALSE;
    }
    let particles = unsafe { std::slice::from_raw_parts(particles, particle_count as usize) };
    let Some(report) = mps_formula::fluid::sph_estimate_forces(
        particle, particles, smoothing_radius, gas_constant, rest_density, viscosity, surface_tension,
    ) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid SPH force parameters");
        return Bool::FALSE;
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "SPH force output is null");
        return Bool::FALSE;
    };
    *out_report = report;
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_bernoulli_pressure(
    total_pressure: f64,
    density: f64,
    velocity: f64,
    gravity: f64,
    elevation: f64,
) -> f64 {
    mps_formula::fluid::bernoulli_pressure(total_pressure, density, velocity, gravity, elevation)
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_bernoulli_report(
    pressure: f64,
    density: f64,
    velocity: f64,
    gravity: f64,
    elevation: f64,
    out_report: *mut BernoulliReport,
) -> Bool {
    let Some(report) = mps_formula::fluid::bernoulli_report(pressure, density, velocity, gravity, elevation) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid Bernoulli parameters");
        return Bool::FALSE;
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Bernoulli output is null");
        return Bool::FALSE;
    };
    *out_report = report;
    clear_error();
    Bool::TRUE
}