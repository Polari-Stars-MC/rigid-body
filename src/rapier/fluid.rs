use rapier3d::prelude::Vector;

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    BernoulliReport, Bool, FluidForceReport, FluidVolume, NavierStokesReport, RigidBodyHandleRaw,
    SphForceReport, SphParticle, Vec3, WorldHandle, unpack_rigid_body_handle, vec3_finite,
    vec3_from_rapier, vec3_to_rapier,
};

use crate::rapier::math::{KahanSum, KahanVec3, finite_non_negative, finite_positive, mul_add};

const EPSILON: f64 = 1.0e-12;
const PI: f64 = std::f64::consts::PI;

fn fluid_valid(fluid: FluidVolume) -> bool {
    vec3_finite(fluid.center)
        && vec3_finite(fluid.half_extents)
        && vec3_finite(fluid.flow_velocity)
        && vec3_finite(fluid.gravity)
        && fluid.half_extents.x >= 0.0
        && fluid.half_extents.y >= 0.0
        && fluid.half_extents.z >= 0.0
        && fluid.density.is_finite()
        && fluid.density >= 0.0
        && fluid.linear_drag.is_finite()
        && fluid.linear_drag >= 0.0
        && fluid.quadratic_drag.is_finite()
        && fluid.quadratic_drag >= 0.0
        && fluid.angular_drag.is_finite()
        && fluid.angular_drag >= 0.0
}

fn clamp01(value: f64) -> f64 {
    value.clamp(0.0, 1.0)
}

fn submerged_fraction_aabb(body_center: Vec3, body_half_extents: Vec3, fluid: FluidVolume) -> f64 {
    let body_min_y = body_center.y - body_half_extents.y;
    let body_max_y = body_center.y + body_half_extents.y;
    let fluid_min_y = fluid.center.y - fluid.half_extents.y;
    let fluid_max_y = fluid.center.y + fluid.half_extents.y;
    let body_height = body_max_y - body_min_y;
    if body_height <= 0.0 {
        return 0.0;
    }
    let overlap = body_max_y.min(fluid_max_y) - body_min_y.max(fluid_min_y);
    clamp01(overlap / body_height)
}

fn compute_fluid_forces(
    fluid: FluidVolume,
    body_center: Vec3,
    body_half_extents: Vec3,
    body_volume: f64,
    body_linvel: Vec3,
    body_angvel: Vec3,
) -> Option<FluidForceReport> {
    if !fluid_valid(fluid)
        || !vec3_finite(body_center)
        || !vec3_finite(body_half_extents)
        || !vec3_finite(body_linvel)
        || !vec3_finite(body_angvel)
        || !finite_positive(body_volume)
        || body_half_extents.x < 0.0
        || body_half_extents.y < 0.0
        || body_half_extents.z < 0.0
    {
        return None;
    }

    let submerged_fraction = submerged_fraction_aabb(body_center, body_half_extents, fluid);
    if submerged_fraction <= 0.0 || fluid.density <= 0.0 {
        return Some(FluidForceReport::default());
    }

    let displaced_volume = body_volume * submerged_fraction;
    let gravity = vec3_to_rapier(fluid.gravity);
    let relative_velocity = vec3_to_rapier(fluid.flow_velocity) - vec3_to_rapier(body_linvel);
    let speed = relative_velocity.length_squared().sqrt();
    let drag_force = if speed > 1.0e-12 {
        let drag_scale =
            submerged_fraction * (fluid.linear_drag * speed + fluid.quadratic_drag * speed * speed);
        relative_velocity / speed * drag_scale
    } else {
        Vector::ZERO
    };
    let buoyancy_force = -gravity * (fluid.density * displaced_volume);
    let angular_damping_torque =
        -vec3_to_rapier(body_angvel) * (fluid.angular_drag * submerged_fraction);
    let total_force = buoyancy_force + drag_force;

    Some(FluidForceReport {
        buoyancy_force: vec3_from_rapier(buoyancy_force),
        drag_force: vec3_from_rapier(drag_force),
        angular_damping_torque: vec3_from_rapier(angular_damping_torque),
        total_force: vec3_from_rapier(total_force),
        total_torque: vec3_from_rapier(angular_damping_torque),
        submerged_fraction,
        displaced_volume,
    })
}

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
    let Some(report) = compute_fluid_forces(
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
    let Some(report) = compute_fluid_forces(
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
    if !vec3_finite(velocity)
        || !vec3_finite(advection)
        || !vec3_finite(pressure_gradient)
        || !vec3_finite(laplacian_velocity)
        || !vec3_finite(external_acceleration)
        || !finite_positive(density)
        || !finite_non_negative(kinematic_viscosity)
        || !finite_non_negative(dt)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Navier-Stokes parameters");
        return Bool::FALSE;
    }

    let adv = vec3_to_rapier(advection);
    let pressure_acceleration = -vec3_to_rapier(pressure_gradient) / density;
    let viscosity_acceleration = vec3_to_rapier(laplacian_velocity) * kinematic_viscosity;
    let external = vec3_to_rapier(external_acceleration);
    let total = -adv + pressure_acceleration + viscosity_acceleration + external;
    let next_velocity = vec3_to_rapier(velocity) + total * dt;

    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Navier-Stokes output is null");
        return Bool::FALSE;
    };
    *out_report = NavierStokesReport {
        advection,
        pressure_acceleration: vec3_from_rapier(pressure_acceleration),
        viscosity_acceleration: vec3_from_rapier(viscosity_acceleration),
        external_acceleration,
        total_acceleration: vec3_from_rapier(total),
        next_velocity: vec3_from_rapier(next_velocity),
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_sph_poly6_kernel(distance: f64, smoothing_radius: f64) -> f64 {
    if !finite_non_negative(distance) || !finite_positive(smoothing_radius) {
        return f64::NAN;
    }
    if distance >= smoothing_radius {
        return 0.0;
    }
    let h2 = smoothing_radius * smoothing_radius;
    let r2 = distance * distance;
    // Use mul_add to preserve precision for h² - r² when r ≈ h
    let diff = -mul_add(r2, 1.0_f64, -h2); // -(r² - h²) = h² - r²
    if diff <= 0.0 {
        return 0.0;
    }
    315.0 / (64.0 * PI * smoothing_radius.powi(9)) * diff.powi(3)
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_sph_spiky_gradient(
    offset: Vec3,
    smoothing_radius: f64,
    out_gradient: *mut Vec3,
) -> Bool {
    if !vec3_finite(offset) || !finite_positive(smoothing_radius) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid SPH spiky gradient parameters",
        );
        return Bool::FALSE;
    }
    let r = vec3_to_rapier(offset);
    let distance = r.length();
    let gradient = if distance <= EPSILON || distance >= smoothing_radius {
        Vector::ZERO
    } else {
        // mul_add to preserve (smoothing_radius - distance)² when distance ≈ smoothing_radius
        let diff = mul_add(-1.0_f64, distance, smoothing_radius); // h - r
        -r / distance * (45.0 / (PI * smoothing_radius.powi(6)) * diff * diff)
    };
    let Some(out_gradient) = (unsafe { out_gradient.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "SPH gradient output is null");
        return Bool::FALSE;
    };
    *out_gradient = vec3_from_rapier(gradient);
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fluid_sph_viscosity_laplacian(distance: f64, smoothing_radius: f64) -> f64 {
    if !finite_non_negative(distance) || !finite_positive(smoothing_radius) {
        return f64::NAN;
    }
    if distance >= smoothing_radius {
        return 0.0;
    }
    45.0 / (PI * smoothing_radius.powi(6)) * (smoothing_radius - distance)
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
    let p = vec3_to_rapier(position);
    let mut density = KahanSum::default();
    for particle in particles {
        if !vec3_finite(particle.position) || !particle.mass.is_finite() || particle.mass < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid SPH particle");
            return Bool::FALSE;
        }
        density.add(
            particle.mass
                * fluid_sph_poly6_kernel(
                    (p - vec3_to_rapier(particle.position)).length(),
                    smoothing_radius,
                ),
        );
    }
    let Some(out_density) = (unsafe { out_density.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "SPH density output is null");
        return Bool::FALSE;
    };
    *out_density = density.value();
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
    if !vec3_finite(particle.position)
        || !vec3_finite(particle.velocity)
        || !finite_positive(particle.mass)
        || !finite_positive(smoothing_radius)
        || !gas_constant.is_finite()
        || !finite_positive(rest_density)
        || !finite_non_negative(viscosity)
        || !finite_non_negative(surface_tension)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid SPH force parameters");
        return Bool::FALSE;
    }
    if particles.is_null() && particle_count > 0 {
        set_error(ERR_NULL_POINTER, "SPH particle pointer is null");
        return Bool::FALSE;
    }
    let particles = unsafe { std::slice::from_raw_parts(particles, particle_count as usize) };
    let p = vec3_to_rapier(particle.position);
    let v = vec3_to_rapier(particle.velocity);
    let density = if particle.density > EPSILON {
        particle.density
    } else {
        let mut density = KahanSum::default();
        for neighbor in particles {
            density.add(
                neighbor.mass
                    * fluid_sph_poly6_kernel(
                        (p - vec3_to_rapier(neighbor.position)).length(),
                        smoothing_radius,
                    ),
            );
        }
        density.value().max(rest_density)
    };
    let pressure = if particle.pressure.is_finite() {
        particle.pressure
    } else {
        gas_constant * (density - rest_density)
    };
    let mut pressure_force = KahanVec3::default();
    let mut viscosity_force = KahanVec3::default();
    let mut color_gradient = KahanVec3::default();

    for neighbor in particles {
        if !vec3_finite(neighbor.position)
            || !vec3_finite(neighbor.velocity)
            || !finite_positive(neighbor.mass)
            || neighbor.density < 0.0
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid SPH neighbor");
            return Bool::FALSE;
        }
        let offset = p - vec3_to_rapier(neighbor.position);
        let distance = offset.length();
        if distance <= EPSILON || distance >= smoothing_radius {
            continue;
        }
        let neighbor_density = neighbor.density.max(rest_density);
        let neighbor_pressure = if neighbor.pressure.is_finite() {
            neighbor.pressure
        } else {
            gas_constant * (neighbor_density - rest_density)
        };
        let diff = mul_add(-1.0_f64, distance, smoothing_radius); // h - r
        let gradient = -offset / distance * (45.0 / (PI * smoothing_radius.powi(6)) * diff * diff);
        pressure_force.add(vec3_from_rapier(
            -neighbor.mass * ((pressure + neighbor_pressure) / (2.0 * neighbor_density)) * gradient,
        ));
        viscosity_force.add(vec3_from_rapier(
            viscosity * neighbor.mass * (vec3_to_rapier(neighbor.velocity) - v)
                / neighbor_density
                * fluid_sph_viscosity_laplacian(distance, smoothing_radius),
        ));
        color_gradient.add(vec3_from_rapier(neighbor.mass / neighbor_density * gradient));
    }

    let color_gradient_vec = vec3_to_rapier(color_gradient.value());
    let surface_tension_force = if color_gradient_vec.length() > EPSILON {
        -color_gradient_vec / color_gradient_vec.length()
            * surface_tension
            * color_gradient_vec.length()
    } else {
        Vector::ZERO
    };
    let total_force = vec3_to_rapier(pressure_force.value())
        + vec3_to_rapier(viscosity_force.value())
        + surface_tension_force;
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "SPH force output is null");
        return Bool::FALSE;
    };
    *out_report = SphForceReport {
        density,
        pressure,
        pressure_force: pressure_force.value(),
        viscosity_force: viscosity_force.value(),
        surface_tension_force: vec3_from_rapier(surface_tension_force),
        total_force: vec3_from_rapier(total_force),
    };
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
    if !total_pressure.is_finite()
        || !finite_positive(density)
        || !finite_non_negative(velocity)
        || !gravity.is_finite()
        || !elevation.is_finite()
    {
        return f64::NAN;
    }
    total_pressure - 0.5 * density * velocity * velocity - density * gravity * elevation
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
    if !pressure.is_finite()
        || !finite_positive(density)
        || !finite_non_negative(velocity)
        || !gravity.is_finite()
        || !elevation.is_finite()
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Bernoulli parameters");
        return Bool::FALSE;
    }
    let dynamic_pressure = 0.5 * density * velocity * velocity;
    let total_pressure = pressure + dynamic_pressure + density * gravity * elevation;
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Bernoulli output is null");
        return Bool::FALSE;
    };
    *out_report = BernoulliReport {
        pressure,
        velocity,
        elevation,
        total_head: total_pressure / (density * gravity),
        dynamic_pressure,
    };
    clear_error();
    Bool::TRUE
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::BodyStatus;

    fn water() -> FluidVolume {
        FluidVolume {
            center: Vec3::default(),
            half_extents: Vec3 {
                x: 10.0,
                y: 10.0,
                z: 10.0,
            },
            density: 1000.0,
            linear_drag: 2.0,
            quadratic_drag: 0.5,
            angular_drag: 0.2,
            flow_velocity: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            gravity: Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        }
    }

    #[test]
    fn estimates_buoyancy_and_drag() {
        let mut report = FluidForceReport::default();
        assert_eq!(
            fluid_estimate_aabb_forces(
                water(),
                Vec3::default(),
                Vec3 {
                    x: 0.5,
                    y: 0.5,
                    z: 0.5,
                },
                1.0,
                Vec3::default(),
                Vec3::default(),
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(report.submerged_fraction, 1.0);
        assert!(report.buoyancy_force.y > 0.0);
        assert!(report.drag_force.x > 0.0);
    }

    #[test]
    fn applies_fluid_force_to_body() {
        let world = crate::rapier::world::world_create(Vec3::default());
        let builder =
            crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        crate::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder, 1.0);
        let body = crate::rapier::rigid_body::rigid_body_builder_build(builder);
        let handle = crate::rapier::rigid_body::world_insert_rigid_body(world, body);
        let mut report = FluidForceReport::default();

        assert_eq!(
            fluid_apply_aabb_forces(
                world,
                handle,
                water(),
                Vec3 {
                    x: 0.5,
                    y: 0.5,
                    z: 0.5,
                },
                1.0,
                Bool::TRUE,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(report.total_force.y > 0.0);
        crate::rapier::world::world_step(world, 1.0 / 60.0);
        let velocity = crate::rapier::rigid_body::rigid_body_get_linvel(world, handle);
        assert!(velocity.y > 0.0);
        crate::rapier::world::world_destroy(world);
    }

    #[test]
    fn navier_stokes_sph_and_bernoulli_formulas_work() {
        let mut ns = NavierStokesReport::default();
        assert_eq!(
            fluid_navier_stokes_simplified_step(
                Vec3::default(),
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: 2.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 3.0,
                },
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
                2.0,
                0.5,
                0.1,
                &mut ns,
            ),
            Bool::TRUE
        );
        assert!(ns.total_acceleration.x < 0.0);
        assert!(ns.total_acceleration.y < 0.0);
        assert!(ns.total_acceleration.z > 0.0);

        let particles = [
            SphParticle {
                position: Vec3::default(),
                velocity: Vec3::default(),
                mass: 1.0,
                density: 1000.0,
                pressure: 10.0,
            },
            SphParticle {
                position: Vec3 {
                    x: 0.25,
                    y: 0.0,
                    z: 0.0,
                },
                velocity: Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                mass: 1.0,
                density: 1000.0,
                pressure: 20.0,
            },
        ];
        let mut density = 0.0;
        assert_eq!(
            fluid_sph_estimate_density(
                Vec3::default(),
                particles.as_ptr(),
                particles.len() as u32,
                1.0,
                &mut density,
            ),
            Bool::TRUE
        );
        assert!(density > 0.0);

        let mut sph = SphForceReport::default();
        assert_eq!(
            fluid_sph_estimate_forces(
                particles[0],
                particles.as_ptr(),
                particles.len() as u32,
                1.0,
                3.0,
                1000.0,
                0.1,
                0.05,
                &mut sph,
            ),
            Bool::TRUE
        );
        assert!(sph.total_force.x.is_finite());

        let pressure = fluid_bernoulli_pressure(200_000.0, 1000.0, 10.0, 9.81, 2.0);
        assert!(pressure < 200_000.0);
        let mut bernoulli = BernoulliReport::default();
        assert_eq!(
            fluid_bernoulli_report(pressure, 1000.0, 10.0, 9.81, 2.0, &mut bernoulli),
            Bool::TRUE
        );
        assert!(bernoulli.dynamic_pressure > 0.0);
        assert!(bernoulli.total_head > 0.0);
    }
}
