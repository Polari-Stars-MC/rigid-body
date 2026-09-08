use crate::math::Vector3f64;
use std::f64::consts::PI;

use crate::error::{ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error};
use crate::ffi::{
    Bool, NBodyForceReport, NBodyParticle, NBodySolverParams, OrbitalResonanceReport,
    RelativisticOrbitReport, RocheLimitReport, Vec3, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};
use crate::math::mul_add;

use crate::math::{EPS_GENERAL as EPSILON, finite_many, finite_non_negative, finite_positive};

const MAX_NBODY_PARTICLES: u32 = 100_000;
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

#[derive(Clone, Copy)]
struct Bounds {
    center: Vector3f64,
    half_size: f64,
}

#[derive(Clone)]
struct QuadNode {
    bounds: Bounds,
    mass: f64,
    center_of_mass: Vector3f64,
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

fn child_index(bounds: Bounds, position: Vector3f64) -> usize {
    usize::from(position.x >= bounds.center.x) + 2 * usize::from(position.y >= bounds.center.y)
}

fn child_bounds(bounds: Bounds, index: usize) -> Bounds {
    let quarter = bounds.half_size * 0.5;
    Bounds {
        center: Vector3f64::new(
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
            center_of_mass: Vector3f64::zeros(),
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
            center_of_mass: Vector3f64::zeros(),
            particle: None,
            children: [None; 4],
        });
        nodes[node_index].children[child] = Some(child_node);
        child_node
    };
    insert_particle(nodes, child_node, particle_index, particles);
}

fn acceleration_from_mass(
    position: Vector3f64,
    center: Vector3f64,
    gm: f64,
    softening: f64,
) -> Vector3f64 {
    if gm <= 0.0 {
        return Vector3f64::zeros();
    }
    let offset = center - position;
    // Use mul_add for softened distance: r² + ε² with single rounding
    let r2 = mul_add(softening, softening, offset.length_squared());
    if r2 <= EPSILON {
        return Vector3f64::zeros();
    }
    // r2 * sqrt(r2) = r³; compute as r2.sqrt() * r2 to avoid overflow
    let r3 = r2.sqrt() * r2;
    offset * (gm / r3)
}

fn bh_acceleration(
    nodes: &[QuadNode],
    node_index: usize,
    particle_index: usize,
    particles: &[NBodyParticle],
    params: NBodySolverParams,
    approximate_count: &mut u32,
    direct_count: &mut u32,
) -> Vector3f64 {
    let node = &nodes[node_index];
    if node.mass <= 0.0 {
        return Vector3f64::zeros();
    }
    if node.particle == Some(particle_index) && node.children.iter().all(Option::is_none) {
        return Vector3f64::zeros();
    }
    let position = vec3_to_rapier(particles[particle_index].position);
    let distance = (node.center_of_mass - position).length();
    let width = node.bounds.half_size * 2.0;
    if node.children.iter().all(Option::is_none)
        || width / distance.max(EPSILON) < params.opening_angle
    {
        *approximate_count += 1;
        return acceleration_from_mass(
            position,
            node.center_of_mass,
            params.gravitational_constant * node.mass,
            params.softening,
        );
    }
    let mut acceleration = Vector3f64::zeros();
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
    let center = Vector3f64::new(0.5 * (min_x + max_x), 0.5 * (min_y + max_y), 0.0);
    let half_size = (0.5 * f64::max(max_x - min_x, max_y - min_y)).max(1.0);
    Bounds { center, half_size }
}

/// Computes direct all-pairs gravitational accelerations for an N-body system.
///
/// # Safety
///
/// `particles` must point to `particle_count` readable `NBodyParticle` elements
/// (`0 < particle_count <= 100_000`); `out_accelerations` must point to writable
/// memory for `capacity` `Vec3` elements (`capacity >= particle_count`);
/// `out_report` may be null, in which case the report is not written.
/// Null `particles` or `out_accelerations` fail with `ERR_NULL_POINTER`;
/// no ownership is transferred.
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
    let particles = match unsafe {
        crate::ffi::checked_input_slice(particles, particle_count as usize, "particles")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
    if particles.iter().any(|particle| !particle_valid(*particle)) {
        set_error(ERR_INVALID_ARGUMENT, "invalid N-body particle");
        return Bool::FALSE;
    }
    let out = match unsafe {
        crate::ffi::checked_output_slice(out_accelerations, capacity as usize, "out_accelerations")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
    // G×m 对每个天体是常量，预计算一次
    let gm: Vec<f64> = particles
        .iter()
        .map(|particle| particle.mass * params.gravitational_constant)
        .collect();
    let mut report = NBodyForceReport {
        body_count: particle_count,
        ..NBodyForceReport::default()
    };
    for i in 0..particles.len() {
        let mut acceleration = Vector3f64::zeros();
        for j in 0..particles.len() {
            if i == j {
                continue;
            }
            acceleration += acceleration_from_mass(
                vec3_to_rapier(particles[i].position),
                vec3_to_rapier(particles[j].position),
                gm[j],
                params.softening,
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

/// 裸牛顿引力快速路径：无软化、无参数校验、无逐对防护的 unsafe 版本。
///
/// 加速度 a = G·m_j / r³ · (p_j − p_i)，直接 O(n²)，G×m 每体预缓存。
/// 调用方必须保证：无重叠粒子（否则除零产生 inf/NaN）、数据全为有限值。
///
/// # Safety
/// `particles` 必须指向至少 `particle_count` 个可读 `NBodyParticle`；
/// `out_accelerations` 必须指向至少 `capacity` 个可写 `Vec3`，且 `capacity >= particle_count`。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn astro_nbody_direct_accelerations_bare(
    particles: *const NBodyParticle,
    particle_count: u32,
    gravitational_constant: f64,
    out_accelerations: *mut Vec3,
    capacity: u32,
) {
    if capacity < particle_count {
        set_error(ERR_CAPACITY, "insufficient acceleration capacity");
        return;
    }
    if particle_count == 0 {
        clear_error();
        return;
    }
    let Some(particles) = crate::ffi::formula_result(unsafe {
        crate::ffi::checked_input_slice(particles, particle_count as usize, "particles")
    }) else {
        return;
    };
    let Some(out) = crate::ffi::formula_result(unsafe {
        crate::ffi::checked_output_slice(out_accelerations, capacity as usize, "out_accelerations")
    }) else {
        return;
    };

    // G×m 对每个天体是常量，预缓存一次
    let gm: Vec<f64> = particles
        .iter()
        .map(|particle| particle.mass * gravitational_constant)
        .collect();

    for i in 0..particles.len() {
        let mut acceleration = Vector3f64::zeros();
        let pi = vec3_to_rapier(particles[i].position);
        for j in 0..particles.len() {
            if i == j {
                continue;
            }
            let offset = vec3_to_rapier(particles[j].position) - pi;
            let r2 = offset.length_squared();
            let r3 = r2.sqrt() * r2;
            acceleration += offset * (gm[j] / r3);
        }
        out[i] = vec3_from_rapier(acceleration);
    }
}

/// Computes Barnes-Hut tree-approximated gravitational accelerations for an N-body system.
///
/// # Safety
///
/// `particles` must point to `particle_count` readable `NBodyParticle` elements
/// (`0 < particle_count <= 100_000`); `out_accelerations` must point to writable
/// memory for `capacity` `Vec3` elements (`capacity >= particle_count`);
/// `out_report` may be null, in which case the report is not written.
/// Null `particles` or `out_accelerations` fail with `ERR_NULL_POINTER`;
/// no ownership is transferred.
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
    let Some(particles) = crate::ffi::formula_result(unsafe {
        crate::ffi::checked_input_slice(particles, particle_count as usize, "particles")
    }) else {
        return Bool::FALSE;
    };
    if particles.iter().any(|particle| !particle_valid(*particle)) {
        set_error(ERR_INVALID_ARGUMENT, "invalid Barnes-Hut particle");
        return Bool::FALSE;
    }
    let mut nodes = vec![QuadNode {
        bounds: root_bounds(particles),
        mass: 0.0,
        center_of_mass: Vector3f64::zeros(),
        particle: None,
        children: [None; 4],
    }];
    for index in 0..particles.len() {
        insert_particle(&mut nodes, 0, index, particles);
    }
    let Some(out) = crate::ffi::formula_result(unsafe {
        crate::ffi::checked_output_slice(out_accelerations, capacity as usize, "out_accelerations")
    }) else {
        return Bool::FALSE;
    };
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

/// Computes the FMM monopole (single-cluster) gravitational acceleration at a position.
///
/// # Safety
///
/// `out_acceleration` must point to writable memory for one `Vec3` element;
/// a null pointer fails with `ERR_NULL_POINTER`. No ownership is transferred.
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
        params.gravitational_constant * cluster_mass,
        params.softening,
    ));
    clear_error();
    Bool::TRUE
}

/// Computes the first-order post-Newtonian relativistic correction for a two-body orbit.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `RelativisticOrbitReport`
/// element; a null pointer fails with `ERR_NULL_POINTER`.
/// No ownership is transferred.
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

/// Computes the fluid and rigid Roche limits of a primary body and tests an orbital
/// distance against them.
///
/// Returns `None` if any parameter is non-finite or non-positive. `orbital_distance`
/// of 0.0 is allowed (treated as origin: `inside_*` flags become `false`).
///
/// Formula:
///   fluid Roche limit  ≈ 2.44 · R_primary · (ρ_primary / ρ_secondary)^(1/3)
///   rigid  Roche limit ≈ 1.26 · R_primary · (ρ_primary / ρ_secondary)^(1/3)
///
/// Coefficients 2.44 / 1.26 are the conventional values (fluid = Édouard Roche
/// 1848; rigid ~ 1.26 for a rigid satellite held together only by self-gravity).
pub fn roche_limit(
    primary_radius: f64,
    primary_density: f64,
    secondary_density: f64,
) -> Option<(f64, f64)> {
    if !finite_positive(primary_radius)
        || !finite_positive(primary_density)
        || !finite_positive(secondary_density)
    {
        return None;
    }
    let ratio = (primary_density / secondary_density).cbrt();
    Some((2.44 * primary_radius * ratio, 1.26 * primary_radius * ratio))
}

/// Same as [`roche_limit`] but also tests an orbital distance against the
/// computed limits — the convenience analogue of the FFI
/// [`RocheLimitReport`]. Returns `None` on invalid parameters; returns the
/// report with `inside_fluid_limit`/`inside_rigid_limit` both `false` when
/// `orbital_distance == 0.0` (no sensible crossing test at the origin).
pub fn roche_limit_report(
    primary_radius: f64,
    primary_density: f64,
    secondary_density: f64,
    orbital_distance: f64,
) -> Option<RocheLimitReport> {
    if !finite_non_negative(orbital_distance) {
        return None;
    }
    let (fluid, rigid) = roche_limit(primary_radius, primary_density, secondary_density)?;
    Some(RocheLimitReport {
        fluid_roche_limit: fluid,
        rigid_roche_limit: rigid,
        inside_fluid_limit: Bool::from(orbital_distance > 0.0 && orbital_distance < fluid),
        inside_rigid_limit: Bool::from(orbital_distance > 0.0 && orbital_distance < rigid),
    })
}

/// Hill sphere radius — the region around a satellite (mass `m_sec`) within
/// which its gravity dominates over the tidal field of the primary body
/// (mass `m_pri`). The companion criterion to the Roche limit: the Roche
/// limit tells you *when a satellite breaks up under the primary's tide*;
/// the Hill sphere tells you *when a smaller bound object stays bound to
/// the satellite instead of being stripped by the primary*.
///
/// `r_H ≈ a · (1 - e) · (m_sec / (3 · m_pri))^(1/3)`
///
/// Inputs:
/// - `primary_mass`     — M_pri in kg
/// - `secondary_mass`   — m_sec in kg (the embedded body, e.g. a moon)
/// - `semi_major_axis`  — a in metres (orbit of sec about pri)
/// - `eccentricity`     — e, dimensionless, clamped to `0..=1`
///
/// Returns `None` if any mass/axis is non-finite or non-positive.
pub fn hill_sphere_radius(
    primary_mass: f64,
    secondary_mass: f64,
    semi_major_axis: f64,
    eccentricity: f64,
) -> Option<f64> {
    if !finite_positive(primary_mass)
        || !finite_positive(secondary_mass)
        || !finite_positive(semi_major_axis)
        || !eccentricity.is_finite()
    {
        return None;
    }
    let e = eccentricity.clamp(0.0, 1.0);
    Some(semi_major_axis * (1.0 - e) * (secondary_mass / (3.0 * primary_mass)).cbrt())
}

/// Computes the fluid and rigid Roche limits of a primary body and tests an orbital
/// distance against them.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `RocheLimitReport` element;
/// a null pointer fails with `ERR_NULL_POINTER`. No ownership is transferred.
#[unsafe(no_mangle)]
pub extern "C" fn astro_roche_limit(
    primary_radius: f64,
    primary_density: f64,
    secondary_density: f64,
    orbital_distance: f64,
    out_report: *mut RocheLimitReport,
) -> Bool {
    let Some(report) = roche_limit_report(
        primary_radius,
        primary_density,
        secondary_density,
        orbital_distance,
    ) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid Roche limit parameters");
        return Bool::FALSE;
    };
    let Some(out) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Roche limit output is null");
        return Bool::FALSE;
    };
    *out = report;
    clear_error();
    Bool::TRUE
}

/// Finds the best small-integer orbital period ratio and reports whether two orbits
/// are in resonance within a tolerance.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `OrbitalResonanceReport`
/// element; a null pointer fails with `ERR_NULL_POINTER`.
/// No ownership is transferred.
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

/// Applies the Barnes-Hut opening-angle criterion to decide whether a tree node
/// must be opened.
///
/// # Safety
///
/// This function takes no pointers and performs no memory access; it is always
/// safe to call.
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

// ---------------------------------------------------------------------------
// Stellar structure
// ---------------------------------------------------------------------------

/// Lane-Emden equation (polytropic): dimensionless central density for polytropic index n.
/// Returns the dimensionless radius xi_1 where theta becomes zero (surface).
pub fn lane_emden_first_zero(polytropic_index: f64) -> Option<f64> {
    if !polytropic_index.is_finite() || !(0.0..=5.0).contains(&polytropic_index) {
        return None;
    }
    if (polytropic_index - 0.0).abs() < 1e-6 {
        return Some(2.449_489_742_783_178_f64);
    }
    if (polytropic_index - 1.0).abs() < 1e-6 {
        return Some(std::f64::consts::PI);
    }
    if (polytropic_index - 1.5).abs() < 1e-6 {
        return Some(3.653_753_735_236_717_f64);
    }
    if (polytropic_index - 3.0).abs() < 1e-6 {
        return Some(6.896_848_624_348_534_f64);
    }
    // n=4: xi_1 ~ 14.97, n=4.5: xi_1 ~ 31.84, n=5: infinite
    None
}

/// Mass-luminosity relation for main sequence stars: L ~ M^alpha
/// alpha ~ 3.5 for solar-type stars
pub fn mass_luminosity_relation(mass_solar: f64, exponent: f64) -> Option<f64> {
    if !mass_solar.is_finite() || mass_solar <= 0.0 || !exponent.is_finite() || exponent <= 0.0 {
        return None;
    }
    Some(mass_solar.powf(exponent))
}

/// Eddington luminosity: L_Edd = 4*pi*G*M*c / kappa (W)
pub fn eddington_luminosity(mass: f64, opacity: f64) -> Option<f64> {
    let g = 6.67430e-11;
    let c = 299_792_458.0;
    if !mass.is_finite() || mass <= 0.0 || !opacity.is_finite() || opacity <= 0.0 {
        return None;
    }
    Some(4.0 * PI * g * mass * c / opacity)
}

/// Eddington luminosity in solar units (L/L_sun).
pub fn eddington_luminosity_solar(mass_solar: f64, opacity: f64) -> Option<f64> {
    let l_sun = 3.828e26;
    let m = mass_solar * 1.98847e30;
    let l = eddington_luminosity(m, opacity)?;
    Some(l / l_sun)
}

// ---------------------------------------------------------------------------
// Hubble's law and cosmology
// ---------------------------------------------------------------------------

/// Hubble's law: v = H0 * d
pub fn hubble_velocity(hubble_constant: f64, distance: f64) -> Option<f64> {
    if !hubble_constant.is_finite()
        || hubble_constant <= 0.0
        || !distance.is_finite()
        || distance < 0.0
    {
        return None;
    }
    Some(hubble_constant * distance)
}

/// Hubble distance: d = v / H0
pub fn hubble_distance(velocity: f64, hubble_constant: f64) -> Option<f64> {
    if !velocity.is_finite()
        || velocity < 0.0
        || !hubble_constant.is_finite()
        || hubble_constant <= 0.0
    {
        return None;
    }
    Some(velocity / hubble_constant)
}

// ---------------------------------------------------------------------------
// NFW dark matter profile
// ---------------------------------------------------------------------------

/// NFW dark matter density profile: rho(r) = rho_0 / (r/r_s * (1 + r/r_s)^2)
pub fn nfw_density(radius: f64, scale_radius: f64, characteristic_density: f64) -> Option<f64> {
    if !finite_many(&[radius, scale_radius, characteristic_density, 0.0])
        || scale_radius <= 0.0
        || characteristic_density < 0.0
    {
        return None;
    }
    let x = radius / scale_radius;
    if x <= 0.0 {
        return Some(characteristic_density);
    }
    Some(characteristic_density / (x * (1.0 + x) * (1.0 + x)))
}

/// NFW enclosed mass within radius r: M(r) = 4*pi*rho_0*r_s^3 * (ln(1+r/r_s) - r/(r_s+r))
pub fn nfw_enclosed_mass(
    radius: f64,
    scale_radius: f64,
    characteristic_density: f64,
) -> Option<f64> {
    if !finite_many(&[radius, scale_radius, characteristic_density, 0.0])
        || scale_radius <= 0.0
        || characteristic_density < 0.0
    {
        return None;
    }
    let x = radius / scale_radius;
    let term = (1.0 + x).ln() - x / (1.0 + x);
    Some(4.0 * PI * characteristic_density * scale_radius.powi(3) * term)
}

// ---------------------------------------------------------------------------
// Blackbody radiation
// ---------------------------------------------------------------------------

/// Planck blackbody spectral radiance: B_lambda(T) = 2hc^2/lambda^5 * 1/(exp(hc/(lambda*kb*T))-1)
pub fn blackbody_spectral_radiance(wavelength: f64, temperature: f64) -> Option<f64> {
    let h = 6.62607015e-34;
    let c = 299_792_458.0;
    let kb = 1.380649e-23;
    if !wavelength.is_finite()
        || wavelength <= 0.0
        || !temperature.is_finite()
        || temperature <= 0.0
    {
        return None;
    }
    let exponent = h * c / (wavelength * kb * temperature);
    if exponent > 700.0 {
        return Some(0.0);
    }
    Some(2.0 * h * c * c / wavelength.powi(5) / (exponent.exp() - 1.0))
}

/// Wien's displacement law: lambda_max * T = b, where b = 2.898e-3 m·K
pub fn wien_displacement(temperature: f64) -> Option<f64> {
    if !temperature.is_finite() || temperature <= 0.0 {
        return None;
    }
    Some(2.897771955e-3 / temperature)
}

// ---------------------------------------------------------------------------
// Jeans criterion
// ---------------------------------------------------------------------------

/// Jeans mass: M_J = (5*kb*T/(G*mu*mH))^(3/2) * (3/(4*pi*rho))^(1/2)
pub fn jeans_mass(temperature: f64, density: f64, mean_molecular_weight: f64) -> Option<f64> {
    let g = 6.67430e-11;
    let kb = 1.380649e-23;
    let mh = 1.6735575e-27;
    if !finite_many(&[temperature, density, mean_molecular_weight, 0.0])
        || temperature < 0.0
        || density <= 0.0
        || mean_molecular_weight <= 0.0
    {
        return None;
    }
    let cs2 = kb * temperature / (mean_molecular_weight * mh);
    Some((5.0 * cs2 / (2.0 * g)).powf(1.5) * (3.0 / (4.0 * PI * density)).sqrt())
}

/// Jeans length: lambda_J = cs * sqrt(pi / (G * rho))
pub fn jeans_length(temperature: f64, density: f64, mean_molecular_weight: f64) -> Option<f64> {
    let g = 6.67430e-11;
    let kb = 1.380649e-23;
    let mh = 1.6735575e-27;
    if !finite_many(&[temperature, density, mean_molecular_weight, 0.0])
        || temperature < 0.0
        || density <= 0.0
        || mean_molecular_weight <= 0.0
    {
        return None;
    }
    let cs = (kb * temperature / (mean_molecular_weight * mh)).sqrt();
    Some(cs * (PI / (g * density)).sqrt())
}

// ---------------------------------------------------------------------------
// Stellar evolution
// ---------------------------------------------------------------------------

/// Main sequence lifetime: τ ≈ 10¹⁰ · (M/M☉)^(-2.5) years
pub fn main_sequence_lifetime(mass_solar: f64) -> Option<f64> {
    if !mass_solar.is_finite() || mass_solar <= 0.0 {
        return None;
    }
    Some(1.0e10 * mass_solar.powf(-2.5))
}

/// Mass-radius relation for main sequence stars: R/R☉ ≈ (M/M☉)^0.8
pub fn mass_radius_relation(mass_solar: f64) -> Option<f64> {
    if !mass_solar.is_finite() || mass_solar <= 0.0 {
        return None;
    }
    Some(mass_solar.powf(0.8))
}

/// Chandrasekhar mass: M_ch = 1.44 M☉
pub fn chandrasekhar_mass_limit() -> f64 {
    1.44
}

// ---------------------------------------------------------------------------
// Binary star systems
// ---------------------------------------------------------------------------

/// Binary mass function: f(m) = (m₂·sin i)³ / (m₁+m₂)² = P·K₁³/(2πG)
pub fn mass_function(period_seconds: f64, semi_amplitude: f64) -> Option<f64> {
    let g = 6.67430e-11;
    if !period_seconds.is_finite()
        || period_seconds <= 0.0
        || !semi_amplitude.is_finite()
        || semi_amplitude <= 0.0
    {
        return None;
    }
    Some(period_seconds * semi_amplitude.powi(3) / (2.0 * std::f64::consts::PI * g))
}

/// Kepler's third law for binary: a³ = G·(m₁+m₂)·P²/(4π²)
pub fn binary_semi_major_axis(total_mass: f64, period: f64) -> Option<f64> {
    let g = 6.67430e-11;
    if !total_mass.is_finite() || total_mass <= 0.0 || !period.is_finite() || period <= 0.0 {
        return None;
    }
    Some(
        (g * total_mass * period * period / (4.0 * std::f64::consts::PI * std::f64::consts::PI))
            .cbrt(),
    )
}

// ---------------------------------------------------------------------------
// Accretion disk (Shakura-Sunyaev alpha model)
// ---------------------------------------------------------------------------

/// Shakura-Sunyaev effective temperature at radius:
/// T_eff(r) = (3GM·M_dot / (8π·σ·r³))^(1/4) · (1 - (R_in/r)^(1/2))^(1/4)
pub fn ss73_disk_temperature(
    mass_kg: f64,
    accretion_rate: f64,
    radius: f64,
    inner_radius: f64,
) -> Option<f64> {
    let g = 6.67430e-11;
    let sigma = 5.670_374_419e-8;
    if !finite_many(&[mass_kg, accretion_rate, radius, inner_radius])
        || mass_kg <= 0.0
        || accretion_rate < 0.0
        || radius <= 0.0
        || inner_radius <= 0.0
        || radius < inner_radius
    {
        return None;
    }
    let factor =
        3.0 * g * mass_kg * accretion_rate / (8.0 * std::f64::consts::PI * sigma * radius.powi(3));
    let inner = (1.0 - (inner_radius / radius).sqrt()).max(0.0);
    Some((factor * inner).powf(0.25))
}

// ---------------------------------------------------------------------------
// Supernova
// ---------------------------------------------------------------------------

/// Chandrasekhar mass (alternative formulation in kg).
pub fn chandrasekhar_mass_kg() -> f64 {
    2.865e30
}

/// Nickel-56 decay contribution to SN light curve: L(t) = M_Ni · ε_Ni · exp(-t/τ_Ni)
/// τ_Ni ≈ 8.76 days
pub fn nickel56_decay_luminosity(nickel_mass_kg: f64, time_days: f64) -> Option<f64> {
    if !nickel_mass_kg.is_finite()
        || nickel_mass_kg < 0.0
        || !time_days.is_finite()
        || time_days < 0.0
    {
        return None;
    }
    let tau = 8.76 * 86400.0; // 8.76 days in seconds
    let epsilon = 3.90e13; // J/kg
    Some(nickel_mass_kg * epsilon * (-time_days * 86400.0 / tau).exp())
}

// ---------------------------------------------------------------------------
// Exoplanet characterization
// ---------------------------------------------------------------------------

/// Transit depth: δ = (R_p / R_s)²
pub fn transit_depth(planet_radius: f64, star_radius: f64) -> Option<f64> {
    if !finite_many(&[planet_radius, star_radius, 0.0, 0.0])
        || planet_radius < 0.0
        || star_radius <= 0.0
    {
        return None;
    }
    Some((planet_radius / star_radius).powi(2))
}

/// Radial velocity semi-amplitude: K = (2πG/P)^(1/3) · (m_p·sin i)/((m_s+m_p)^(2/3))
pub fn radial_velocity_semi_amplitude(
    planet_mass_kg: f64,
    star_mass_kg: f64,
    period: f64,
    inclination: f64,
) -> Option<f64> {
    let g = 6.67430e-11;
    if !finite_many(&[planet_mass_kg, star_mass_kg, period, inclination])
        || planet_mass_kg <= 0.0
        || star_mass_kg <= 0.0
        || period <= 0.0
    {
        return None;
    }
    let total = star_mass_kg + planet_mass_kg;
    let m_sin_i = planet_mass_kg * inclination.sin();
    Some((2.0 * std::f64::consts::PI * g / period).cbrt() * m_sin_i / total.powf(2.0 / 3.0))
}

/// Habitable zone inner and outer boundaries (simplified).
pub fn habitable_zone_boundaries(star_luminosity_solar: f64) -> Option<(f64, f64)> {
    if !star_luminosity_solar.is_finite() || star_luminosity_solar <= 0.0 {
        return None;
    }
    let inner = (star_luminosity_solar / 1.1).sqrt();
    let outer = (star_luminosity_solar / 0.53).sqrt();
    Some((inner, outer))
}

// ---------------------------------------------------------------------------
// Galaxy rotation curve
// ---------------------------------------------------------------------------

/// Circular velocity for NFW dark matter halo: V_c²(r) = V_c²_r · ln(1+cx) - cx/(1+cx) / (ln(1+c) - c/(1+c))
/// Returns V_c in km/s at radius r in units of the scale radius.
pub fn nfw_circular_velocity(r: f64, v_max: f64, r_scale: f64) -> Option<f64> {
    if !finite_many(&[r, v_max, r_scale, 0.0]) || r < 0.0 || v_max <= 0.0 || r_scale <= 0.0 {
        return None;
    }
    let x = r / r_scale;
    if x <= 0.0 {
        return None;
    }
    let ln1x = (1.0 + x).ln();
    Some(v_max * (ln1x / x - 1.0 / (1.0 + x)).sqrt() / (std::f64::consts::LN_2 - 0.5).sqrt())
}

// ---------------------------------------------------------------------------
// Stellar initial mass function
// ---------------------------------------------------------------------------

/// Salpeter IMF: ξ(m) ∝ m^(-2.35)
pub fn salpeter_imf(mass_solar: f64) -> Option<f64> {
    if !mass_solar.is_finite() || mass_solar <= 0.0 {
        return None;
    }
    Some(mass_solar.powf(-2.35))
}

/// Kroupa IMF (piecewise power-law):
/// ξ(m) ∝ m^(-0.3) for m < 0.08, m^(-1.3) for 0.08 < m < 0.5, m^(-2.3) for m > 0.5
pub fn kroupa_imf(mass_solar: f64) -> Option<f64> {
    if !mass_solar.is_finite() || mass_solar <= 0.0 {
        return None;
    }
    let slope = if mass_solar < 0.08 {
        -0.3
    } else if mass_solar < 0.5 {
        -1.3
    } else {
        -2.3
    };
    Some(mass_solar.powf(slope))
}

// ---------------------------------------------------------------------------
// CMB
// ---------------------------------------------------------------------------

/// CMB temperature at redshift z: T(z) = T_0 · (1+z) where T_0 = 2.725 K
pub fn cmb_temperature(redshift: f64) -> Option<f64> {
    if !redshift.is_finite() || redshift < -1.0 {
        return None;
    }
    Some(2.725 * (1.0 + redshift))
}

/// Sound horizon at recombination (simplified): r_s ≈ 150 Mpc
pub fn sound_horizon_at_recombination() -> f64 {
    150.0
}

// ---------------------------------------------------------------------------
// Cosmology: Friedmann equation
// ---------------------------------------------------------------------------

/// Hubble parameter at redshift z: H(z) = H₀·√(Ω_m·(1+z)³ + Ω_r·(1+z)⁴ + Ω_Λ)
pub fn hubble_parameter_z(
    redshift: f64,
    omega_m: f64,
    omega_r: f64,
    omega_l: f64,
    h0: f64,
) -> Option<f64> {
    if !finite_many(&[redshift, omega_m, omega_r, omega_l]) || !h0.is_finite() {
        return None;
    }
    if omega_m < 0.0 || omega_r < 0.0 || omega_l < 0.0 || h0 <= 0.0 {
        return None;
    }
    let z1 = 1.0 + redshift;
    Some(
        h0 * (omega_m * z1.powi(3)
            + omega_r * z1.powi(4)
            + omega_l
            + (1.0 - omega_m - omega_r - omega_l) * z1 * z1)
            .sqrt(),
    )
}

/// Comoving distance to redshift z (flat universe, matter-dominated approx for low z).
pub fn comoving_distance_z(redshift: f64, h0: f64) -> Option<f64> {
    if !redshift.is_finite() || redshift < 0.0 || !h0.is_finite() || h0 <= 0.0 {
        return None;
    }
    let c = 299_792.458; // km/s
    let h0_si = h0 * 3.240_779_29e-20; // convert to s⁻¹
    let integrator = |z: f64| -> f64 {
        // approximation for flat ΛCDM with Ω_m=0.3, Ω_Λ=0.7
        let mut sum = 0.0;
        let n = 100;
        for i in 0..=n {
            let dz = z / n as f64;
            let zz = i as f64 * dz;
            let e = (0.3 * (1.0 + zz).powi(3) + 0.7).sqrt();
            sum += if i == 0 || i == n {
                1.0 / e
            } else if i % 2 == 0 {
                2.0 / e
            } else {
                4.0 / e
            };
        }
        sum * z / (3.0 * n as f64)
    };
    Some(c / h0_si * integrator(redshift))
}
