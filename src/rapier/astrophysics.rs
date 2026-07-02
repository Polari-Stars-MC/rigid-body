use std::slice;

use rapier3d::prelude::Vector;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, NBodyForceReport, NBodyParticle, NBodySolverParams, OrbitalResonanceReport,
    RelativisticOrbitReport, RocheLimitReport, Vec3, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};
use crate::rapier::math::mul_add;

use crate::rapier::math::{finite_non_negative, finite_positive};

const MAX_NBODY_PARTICLES: u32 = 100_000;
const SPEED_OF_LIGHT: f64 = 299_792_458.0;
const EPSILON: f64 = 1.0e-12;

#[derive(Clone, Copy)]
struct Bounds {
    center: Vector,
    half_size: f64,
}

#[derive(Clone)]
struct QuadNode {
    bounds: Bounds,
    mass: f64,
    center_of_mass: Vector,
    particle: Option<usize>,
    children: [Option<usize>; 4],
}

fn params_valid(params: NBodySolverParams) -> bool {
    finite_positive(params.gravitational_constant)
        && finite_non_negative(params.softening)
        && finite_positive(params.opening_angle)
}

fn particle_valid(particle: NBodyParticle) -> bool {
    vec3_finite(particle.position)
        && vec3_finite(particle.velocity)
        && finite_non_negative(particle.mass)
}

fn child_index(bounds: Bounds, position: Vector) -> usize {
    usize::from(position.x >= bounds.center.x) + 2 * usize::from(position.y >= bounds.center.y)
}

fn child_bounds(bounds: Bounds, index: usize) -> Bounds {
    let quarter = bounds.half_size * 0.5;
    Bounds {
        center: Vector::new(
            bounds.center.x + if index & 1 == 0 { -quarter } else { quarter },
            bounds.center.y + if index & 2 == 0 { -quarter } else { quarter },
            0.0,
        ),
        half_size: quarter,
    }
}

fn update_mass(node: &mut QuadNode, particle: NBodyParticle) {
    if particle.mass <= 0.0 {
        return;
    }
    let old_mass = node.mass;
    node.mass += particle.mass;
    node.center_of_mass = (node.center_of_mass * old_mass
        + vec3_to_rapier(particle.position) * particle.mass)
        / node.mass;
}

fn insert_particle(
    nodes: &mut Vec<QuadNode>,
    node_index: usize,
    particle_index: usize,
    particles: &[NBodyParticle],
) {
    update_mass(&mut nodes[node_index], particles[particle_index]);
    if nodes[node_index].bounds.half_size <= 1.0e-9 {
        nodes[node_index].particle = Some(particle_index);
        return;
    }
    if nodes[node_index].particle.is_none()
        && nodes[node_index].children.iter().all(Option::is_none)
    {
        nodes[node_index].particle = Some(particle_index);
        return;
    }
    if let Some(existing) = nodes[node_index].particle.take() {
        let child = child_index(
            nodes[node_index].bounds,
            vec3_to_rapier(particles[existing].position),
        );
        let child_node = nodes.len();
        nodes.push(QuadNode {
            bounds: child_bounds(nodes[node_index].bounds, child),
            mass: 0.0,
            center_of_mass: Vector::ZERO,
            particle: None,
            children: [None; 4],
        });
        nodes[node_index].children[child] = Some(child_node);
        insert_particle(nodes, child_node, existing, particles);
    }
    let child = child_index(
        nodes[node_index].bounds,
        vec3_to_rapier(particles[particle_index].position),
    );
    let child_node = if let Some(child_node) = nodes[node_index].children[child] {
        child_node
    } else {
        let child_node = nodes.len();
        nodes.push(QuadNode {
            bounds: child_bounds(nodes[node_index].bounds, child),
            mass: 0.0,
            center_of_mass: Vector::ZERO,
            particle: None,
            children: [None; 4],
        });
        nodes[node_index].children[child] = Some(child_node);
        child_node
    };
    insert_particle(nodes, child_node, particle_index, particles);
}

fn acceleration_from_mass(
    position: Vector,
    center: Vector,
    mass: f64,
    params: NBodySolverParams,
) -> Vector {
    if mass <= 0.0 {
        return Vector::ZERO;
    }
    let offset = center - position;
    // Use mul_add for softened distance: r² + ε² with single rounding
    let r2 = mul_add(params.softening, params.softening, offset.length_squared());
    if r2 <= EPSILON {
        return Vector::ZERO;
    }
    // r2 * sqrt(r2) = r³; compute as r2.sqrt() * r2 to avoid overflow
    let r3 = r2.sqrt() * r2;
    offset * (params.gravitational_constant * mass / r3)
}

fn bh_acceleration(
    nodes: &[QuadNode],
    node_index: usize,
    particle_index: usize,
    particles: &[NBodyParticle],
    params: NBodySolverParams,
    approximate_count: &mut u32,
    direct_count: &mut u32,
) -> Vector {
    let node = &nodes[node_index];
    if node.mass <= 0.0 {
        return Vector::ZERO;
    }
    if node.particle == Some(particle_index) && node.children.iter().all(Option::is_none) {
        return Vector::ZERO;
    }
    let position = vec3_to_rapier(particles[particle_index].position);
    let distance = (node.center_of_mass - position).length();
    let width = node.bounds.half_size * 2.0;
    if node.children.iter().all(Option::is_none)
        || width / distance.max(EPSILON) < params.opening_angle
    {
        *approximate_count += 1;
        return acceleration_from_mass(position, node.center_of_mass, node.mass, params);
    }
    let mut acceleration = Vector::ZERO;
    for child in node.children.into_iter().flatten() {
        if nodes[child].children.iter().all(Option::is_none) {
            *direct_count += 1;
        }
        acceleration += bh_acceleration(
            nodes,
            child,
            particle_index,
            particles,
            params,
            approximate_count,
            direct_count,
        );
    }
    acceleration
}

fn root_bounds(particles: &[NBodyParticle]) -> Bounds {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for particle in particles {
        min_x = f64::min(min_x, particle.position.x);
        max_x = f64::max(max_x, particle.position.x);
        min_y = f64::min(min_y, particle.position.y);
        max_y = f64::max(max_y, particle.position.y);
    }
    let center = Vector::new(0.5 * (min_x + max_x), 0.5 * (min_y + max_y), 0.0);
    let half_size = (0.5 * f64::max(max_x - min_x, max_y - min_y)).max(1.0);
    Bounds { center, half_size }
}

#[unsafe(no_mangle)]
pub extern "C" fn astro_nbody_direct_accelerations(
    particles: *const NBodyParticle,
    particle_count: u32,
    params: NBodySolverParams,
    out_accelerations: *mut Vec3,
    capacity: u32,
    out_report: *mut NBodyForceReport,
) -> Bool {
    if particle_count == 0 || particle_count > MAX_NBODY_PARTICLES || capacity < particle_count {
        set_error(ERR_CAPACITY, "invalid N-body direct capacity");
        return Bool::FALSE;
    }
    if particles.is_null() || out_accelerations.is_null() {
        set_error(ERR_NULL_POINTER, "N-body direct pointers are null");
        return Bool::FALSE;
    }
    if !params_valid(params) {
        set_error(ERR_INVALID_ARGUMENT, "invalid N-body direct parameters");
        return Bool::FALSE;
    }
    let particles = unsafe { slice::from_raw_parts(particles, particle_count as usize) };
    if particles.iter().any(|particle| !particle_valid(*particle)) {
        set_error(ERR_INVALID_ARGUMENT, "invalid N-body particle");
        return Bool::FALSE;
    }
    let out = unsafe { slice::from_raw_parts_mut(out_accelerations, capacity as usize) };
    let mut report = NBodyForceReport {
        body_count: particle_count,
        ..NBodyForceReport::default()
    };
    for i in 0..particles.len() {
        let mut acceleration = Vector::ZERO;
        for j in 0..particles.len() {
            if i == j {
                continue;
            }
            acceleration += acceleration_from_mass(
                vec3_to_rapier(particles[i].position),
                vec3_to_rapier(particles[j].position),
                particles[j].mass,
                params,
            );
            report.direct_pair_count += 1;
        }
        report.max_acceleration = f64::max(report.max_acceleration, acceleration.length());
        out[i] = vec3_from_rapier(acceleration);
    }
    for i in 0..particles.len() {
        for j in i + 1..particles.len() {
            let distance = (vec3_to_rapier(particles[j].position)
                - vec3_to_rapier(particles[i].position))
            .length()
            .max(params.softening);
            report.potential_energy -=
                params.gravitational_constant * particles[i].mass * particles[j].mass / distance;
        }
    }
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = report;
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn astro_nbody_barnes_hut_accelerations(
    particles: *const NBodyParticle,
    particle_count: u32,
    params: NBodySolverParams,
    out_accelerations: *mut Vec3,
    capacity: u32,
    out_report: *mut NBodyForceReport,
) -> Bool {
    if particle_count == 0 || particle_count > MAX_NBODY_PARTICLES || capacity < particle_count {
        set_error(ERR_CAPACITY, "invalid Barnes-Hut capacity");
        return Bool::FALSE;
    }
    if particles.is_null() || out_accelerations.is_null() {
        set_error(ERR_NULL_POINTER, "Barnes-Hut pointers are null");
        return Bool::FALSE;
    }
    if !params_valid(params) {
        set_error(ERR_INVALID_ARGUMENT, "invalid Barnes-Hut parameters");
        return Bool::FALSE;
    }
    let particles = unsafe { slice::from_raw_parts(particles, particle_count as usize) };
    if particles.iter().any(|particle| !particle_valid(*particle)) {
        set_error(ERR_INVALID_ARGUMENT, "invalid Barnes-Hut particle");
        return Bool::FALSE;
    }
    let mut nodes = vec![QuadNode {
        bounds: root_bounds(particles),
        mass: 0.0,
        center_of_mass: Vector::ZERO,
        particle: None,
        children: [None; 4],
    }];
    for index in 0..particles.len() {
        insert_particle(&mut nodes, 0, index, particles);
    }
    let out = unsafe { slice::from_raw_parts_mut(out_accelerations, capacity as usize) };
    let mut report = NBodyForceReport {
        body_count: particle_count,
        ..NBodyForceReport::default()
    };
    for (i, out_item) in out.iter_mut().enumerate().take(particles.len()) {
        let mut approximate = 0;
        let mut direct = 0;
        let acceleration = bh_acceleration(
            &nodes,
            0,
            i,
            particles,
            params,
            &mut approximate,
            &mut direct,
        );
        report.approximate_node_count += approximate;
        report.direct_pair_count += direct;
        report.max_acceleration = f64::max(report.max_acceleration, acceleration.length());
        *out_item = vec3_from_rapier(acceleration);
    }
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = report;
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn astro_fmm_monopole_acceleration(
    position: Vec3,
    cluster_center: Vec3,
    cluster_mass: f64,
    params: NBodySolverParams,
    out_acceleration: *mut Vec3,
) -> Bool {
    if !vec3_finite(position)
        || !vec3_finite(cluster_center)
        || !finite_non_negative(cluster_mass)
        || !params_valid(params)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid FMM monopole parameters");
        return Bool::FALSE;
    }
    let Some(out_acceleration) = (unsafe { out_acceleration.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "FMM monopole output is null");
        return Bool::FALSE;
    };
    *out_acceleration = vec3_from_rapier(acceleration_from_mass(
        vec3_to_rapier(position),
        vec3_to_rapier(cluster_center),
        cluster_mass,
        params,
    ));
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn astro_relativistic_orbit_correction(
    position: Vec3,
    velocity: Vec3,
    central_mass: f64,
    gravitational_constant: f64,
    out_report: *mut RelativisticOrbitReport,
) -> Bool {
    if !vec3_finite(position)
        || !vec3_finite(velocity)
        || !finite_positive(central_mass)
        || !finite_positive(gravitational_constant)
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid relativistic correction parameters",
        );
        return Bool::FALSE;
    }
    let r = vec3_to_rapier(position);
    let v = vec3_to_rapier(velocity);
    let radius = r.length();
    if radius <= EPSILON {
        set_error(
            ERR_INVALID_ARGUMENT,
            "relativistic correction radius is zero",
        );
        return Bool::FALSE;
    }
    let mu = gravitational_constant * central_mass;
    let h = r.cross(v).length();
    let radial_velocity = r.dot(v) / radius;
    let c2 = SPEED_OF_LIGHT * SPEED_OF_LIGHT;
    // Compute r² and r³ safely; prefer r² * r over powi(3)
    let r2 = radius * radius;
    let r3 = r2 * radius;
    let correction = r * (mu / (c2 * r3)) * (4.0 * mu / radius - v.length_squared())
        + v * (4.0 * mu * radial_velocity / (c2 * r2));
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "relativistic correction output is null");
        return Bool::FALSE;
    };
    *out_report = RelativisticOrbitReport {
        schwarzschild_radius: 2.0 * mu / (SPEED_OF_LIGHT * SPEED_OF_LIGHT),
        periapsis_precession_per_orbit: if h > EPSILON {
            6.0 * std::f64::consts::PI * mu * mu / (SPEED_OF_LIGHT * SPEED_OF_LIGHT * h * h)
        } else {
            0.0
        },
        correction_acceleration: vec3_from_rapier(correction),
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn astro_roche_limit(
    primary_radius: f64,
    primary_density: f64,
    secondary_density: f64,
    orbital_distance: f64,
    out_report: *mut RocheLimitReport,
) -> Bool {
    if !finite_positive(primary_radius)
        || !finite_positive(primary_density)
        || !finite_positive(secondary_density)
        || !finite_non_negative(orbital_distance)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Roche limit parameters");
        return Bool::FALSE;
    }
    let ratio = (primary_density / secondary_density).cbrt();
    let fluid = 2.44 * primary_radius * ratio;
    let rigid = 1.26 * primary_radius * ratio;
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Roche limit output is null");
        return Bool::FALSE;
    };
    *out_report = RocheLimitReport {
        fluid_roche_limit: fluid,
        rigid_roche_limit: rigid,
        inside_fluid_limit: Bool::from(orbital_distance > 0.0 && orbital_distance < fluid),
        inside_rigid_limit: Bool::from(orbital_distance > 0.0 && orbital_distance < rigid),
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn astro_orbital_resonance_detect(
    inner_period: f64,
    outer_period: f64,
    max_denominator: u32,
    tolerance: f64,
    out_report: *mut OrbitalResonanceReport,
) -> Bool {
    if !finite_positive(inner_period)
        || !finite_positive(outer_period)
        || max_denominator == 0
        || max_denominator > 128
        || !finite_non_negative(tolerance)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid orbital resonance parameters");
        return Bool::FALSE;
    }
    let actual = outer_period / inner_period;
    let mut best_num = 1;
    let mut best_den = 1;
    let mut best_error = f64::INFINITY;
    for den in 1..=max_denominator {
        let num = (actual * den as f64).round().max(1.0) as u32;
        let target = num as f64 / den as f64;
        let error = ((actual - target) / target).abs();
        if error < best_error {
            best_error = error;
            best_num = num;
            best_den = den;
        }
    }
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "orbital resonance output is null");
        return Bool::FALSE;
    };
    let target = best_num as f64 / best_den as f64;
    *out_report = OrbitalResonanceReport {
        ratio_numerator: best_num,
        ratio_denominator: best_den,
        actual_ratio: actual,
        target_ratio: target,
        relative_error: best_error,
        resonant: Bool::from(best_error <= tolerance),
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn astro_barnes_hut_should_open(
    node_width: f64,
    distance: f64,
    opening_angle: f64,
) -> Bool {
    if !finite_positive(node_width) || !finite_positive(distance) || !finite_positive(opening_angle)
    {
        return Bool::FALSE;
    }
    Bool::from(node_width / distance >= opening_angle)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> NBodySolverParams {
        NBodySolverParams {
            gravitational_constant: 1.0,
            softening: 0.0,
            opening_angle: 0.5,
            multipole_order: 0,
        }
    }

    #[test]
    fn direct_nbody_accelerates_toward_mass() {
        let particles = [
            NBodyParticle {
                position: Vec3::default(),
                velocity: Vec3::default(),
                mass: 1.0,
            },
            NBodyParticle {
                position: Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                velocity: Vec3::default(),
                mass: 2.0,
            },
        ];
        let mut out = [Vec3::default(); 2];
        let mut report = NBodyForceReport::default();
        assert_eq!(
            astro_nbody_direct_accelerations(
                particles.as_ptr(),
                particles.len() as u32,
                params(),
                out.as_mut_ptr(),
                out.len() as u32,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(out[0].x > 0.0);
        assert!(out[1].x < 0.0);
        assert!(report.potential_energy < 0.0);
    }

    #[test]
    fn roche_and_resonance_reports_work() {
        let mut roche = RocheLimitReport::default();
        assert_eq!(
            astro_roche_limit(1.0, 5.0, 1.0, 2.0, &mut roche),
            Bool::TRUE
        );
        assert!(roche.fluid_roche_limit > roche.rigid_roche_limit);
        assert_eq!(roche.inside_fluid_limit, Bool::TRUE);

        let mut resonance = OrbitalResonanceReport::default();
        assert_eq!(
            astro_orbital_resonance_detect(1.0, 2.01, 8, 0.01, &mut resonance),
            Bool::TRUE
        );
        assert_eq!(resonance.ratio_numerator, 2);
        assert_eq!(resonance.ratio_denominator, 1);
        assert_eq!(resonance.resonant, Bool::TRUE);
    }

    #[test]
    fn relativistic_correction_is_finite() {
        let mut report = RelativisticOrbitReport::default();
        assert_eq!(
            astro_relativistic_orbit_correction(
                Vec3 {
                    x: 1.0e7,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: 1.0e4,
                    z: 0.0,
                },
                1.0e30,
                6.67430e-11,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(report.schwarzschild_radius > 0.0);
        assert!(report.correction_acceleration.x.is_finite());
    }
}
