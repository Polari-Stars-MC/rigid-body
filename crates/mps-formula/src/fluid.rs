//! Fluid dynamics — AABB buoyancy/drag, SPH, Navier-Stokes, and Bernoulli formulas.
//!
//! Pure computation only — no access to `WorldHandle`, `RigidBody`, or Rapier state.

use crate::ffi::{
    BernoulliReport, FluidForceReport, FluidVolume, NavierStokesReport,
    SphForceReport, SphParticle, Vec3,
    vec3_finite, vec3_to_rapier, vec3_from_rapier,
};
use crate::math::{KahanSum, KahanVec3, finite_non_negative, finite_positive, mul_add};
use rapier3d::prelude::Vector;

const EPSILON: f64 = 1.0e-12;
const PI: f64 = std::f64::consts::PI;

fn fluid_valid(fluid: &FluidVolume) -> bool {
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

fn submerged_fraction_aabb(body_center: Vec3, body_half_extents: Vec3, fluid: &FluidVolume) -> f64 {
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

/// Compute AABB-based fluid buoyancy and drag forces.
pub fn compute_fluid_forces(
    fluid: FluidVolume,
    body_center: Vec3,
    body_half_extents: Vec3,
    body_volume: f64,
    body_linvel: Vec3,
    body_angvel: Vec3,
) -> Option<FluidForceReport> {
    if !fluid_valid(&fluid)
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

    let submerged_fraction = submerged_fraction_aabb(body_center, body_half_extents, &fluid);
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
        rapier3d::prelude::Vector::ZERO
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

/// Simplified Navier-Stokes step.
pub fn navier_stokes_simplified_step(
    velocity: Vec3,
    advection: Vec3,
    pressure_gradient: Vec3,
    laplacian_velocity: Vec3,
    external_acceleration: Vec3,
    density: f64,
    kinematic_viscosity: f64,
    dt: f64,
) -> Option<NavierStokesReport> {
    if !vec3_finite(velocity)
        || !vec3_finite(advection)
        || !vec3_finite(pressure_gradient)
        || !vec3_finite(laplacian_velocity)
        || !vec3_finite(external_acceleration)
        || !finite_positive(density)
        || !finite_non_negative(kinematic_viscosity)
        || !finite_non_negative(dt)
    {
        return None;
    }

    let adv = vec3_to_rapier(advection);
    let pressure_acceleration = -vec3_to_rapier(pressure_gradient) / density;
    let viscosity_acceleration = vec3_to_rapier(laplacian_velocity) * kinematic_viscosity;
    let external = vec3_to_rapier(external_acceleration);
    let total = -adv + pressure_acceleration + viscosity_acceleration + external;
    let next_velocity = vec3_to_rapier(velocity) + total * dt;

    Some(NavierStokesReport {
        advection,
        pressure_acceleration: vec3_from_rapier(pressure_acceleration),
        viscosity_acceleration: vec3_from_rapier(viscosity_acceleration),
        external_acceleration,
        total_acceleration: vec3_from_rapier(total),
        next_velocity: vec3_from_rapier(next_velocity),
    })
}

// ---------------------------------------------------------------------------
// SPH kernels
// ---------------------------------------------------------------------------

/// SPH Poly6 kernel.
pub fn sph_poly6_kernel(distance: f64, smoothing_radius: f64) -> f64 {
    if !finite_non_negative(distance) || !finite_positive(smoothing_radius) {
        return f64::NAN;
    }
    if distance >= smoothing_radius {
        return 0.0;
    }
    let h2 = smoothing_radius * smoothing_radius;
    let r2 = distance * distance;
    let diff = -mul_add(r2, 1.0_f64, -h2);
    if diff <= 0.0 {
        return 0.0;
    }
    315.0 / (64.0 * PI * smoothing_radius.powi(9)) * diff.powi(3)
}

/// SPH Spiky gradient.
pub fn sph_spiky_gradient(offset: Vec3, smoothing_radius: f64) -> Option<Vec3> {
    if !vec3_finite(offset) || !finite_positive(smoothing_radius) {
        return None;
    }
    let r = vec3_to_rapier(offset);
    let distance = r.length();
    let gradient = if distance <= EPSILON || distance >= smoothing_radius {
        rapier3d::prelude::Vector::ZERO
    } else {
        let diff = mul_add(-1.0_f64, distance, smoothing_radius);
        -r / distance * (45.0 / (PI * smoothing_radius.powi(6)) * diff * diff)
    };
    Some(vec3_from_rapier(gradient))
}

/// SPH viscosity Laplacian.
pub fn sph_viscosity_laplacian(distance: f64, smoothing_radius: f64) -> f64 {
    if !finite_non_negative(distance) || !finite_positive(smoothing_radius) {
        return f64::NAN;
    }
    if distance >= smoothing_radius {
        return 0.0;
    }
    45.0 / (PI * smoothing_radius.powi(6)) * (smoothing_radius - distance)
}

/// Estimate density at a position using SPH particles.
pub fn sph_estimate_density(
    position: Vec3,
    particles: &[SphParticle],
    smoothing_radius: f64,
) -> Option<f64> {
    if !vec3_finite(position) || !finite_positive(smoothing_radius) {
        return None;
    }
    let p = vec3_to_rapier(position);
    let mut density = KahanSum::default();
    for particle in particles {
        if !vec3_finite(particle.position) || !particle.mass.is_finite() || particle.mass < 0.0 {
            return None;
        }
        density.add(
            particle.mass
                * sph_poly6_kernel(
                    (p - vec3_to_rapier(particle.position)).length(),
                    smoothing_radius,
                ),
        );
    }
    Some(density.value())
}

/// Estimate SPH forces on a particle from its neighbors.
pub fn sph_estimate_forces(
    particle: SphParticle,
    particles: &[SphParticle],
    smoothing_radius: f64,
    gas_constant: f64,
    rest_density: f64,
    viscosity: f64,
    surface_tension: f64,
) -> Option<SphForceReport> {
    if !vec3_finite(particle.position)
        || !vec3_finite(particle.velocity)
        || !finite_positive(particle.mass)
        || !finite_positive(smoothing_radius)
        || !gas_constant.is_finite()
        || !finite_positive(rest_density)
        || !finite_non_negative(viscosity)
        || !finite_non_negative(surface_tension)
    {
        return None;
    }
    let p = vec3_to_rapier(particle.position);
    let v = vec3_to_rapier(particle.velocity);
    let density = if particle.density > EPSILON {
        particle.density
    } else {
        let mut density = KahanSum::default();
        for neighbor in particles {
            density.add(
                neighbor.mass
                    * sph_poly6_kernel(
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
            return None;
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
        let diff = mul_add(-1.0_f64, distance, smoothing_radius);
        let gradient = -offset / distance * (45.0 / (PI * smoothing_radius.powi(6)) * diff * diff);
        pressure_force.add(vec3_from_rapier(
            -neighbor.mass * ((pressure + neighbor_pressure) / (2.0 * neighbor_density)) * gradient,
        ));
        viscosity_force.add(vec3_from_rapier(
            viscosity * neighbor.mass * (vec3_to_rapier(neighbor.velocity) - v)
                / neighbor_density
                * sph_viscosity_laplacian(distance, smoothing_radius),
        ));
        color_gradient.add(vec3_from_rapier(neighbor.mass / neighbor_density * gradient));
    }

    let color_gradient_vec = vec3_to_rapier(color_gradient.value());
    let surface_tension_force = if color_gradient_vec.length() > EPSILON {
        -color_gradient_vec / color_gradient_vec.length()
            * surface_tension
            * color_gradient_vec.length()
    } else {
        rapier3d::prelude::Vector::ZERO
    };
    let total_force = vec3_to_rapier(pressure_force.value())
        + vec3_to_rapier(viscosity_force.value())
        + surface_tension_force;

    Some(SphForceReport {
        density,
        pressure,
        pressure_force: pressure_force.value(),
        viscosity_force: viscosity_force.value(),
        surface_tension_force: vec3_from_rapier(surface_tension_force),
        total_force: vec3_from_rapier(total_force),
    })
}

// ---------------------------------------------------------------------------
// Bernoulli
// ---------------------------------------------------------------------------

/// Bernoulli static pressure.
pub fn bernoulli_pressure(
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

/// Bernoulli report.
pub fn bernoulli_report(
    pressure: f64,
    density: f64,
    velocity: f64,
    gravity: f64,
    elevation: f64,
) -> Option<BernoulliReport> {
    if !pressure.is_finite()
        || !finite_positive(density)
        || !finite_non_negative(velocity)
        || !gravity.is_finite()
        || !elevation.is_finite()
    {
        return None;
    }
    let dynamic_pressure = 0.5 * density * velocity * velocity;
    let total_pressure = pressure + dynamic_pressure + density * gravity * elevation;
    Some(BernoulliReport {
        pressure,
        velocity,
        elevation,
        total_head: total_pressure / (density * gravity),
        dynamic_pressure,
    })
}