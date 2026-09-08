use std::slice;

use crate::math::Vector3f64;

use crate::error::{ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error};
use crate::ffi::{
    Bool, SoftBendingConstraint, SoftBodyStepReport, SoftDistanceConstraint, SoftSphereCollision,
    SoftSpring, SoftVolumeConstraint, Vec3, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

use crate::math::{EPS_GENERAL as EPSILON, KahanSum, finite_non_negative, finite_positive};

const MAX_PARTICLES: u32 = 2_000_000;
const MAX_CONSTRAINTS: u32 = 2_000_000;

fn index_valid(index: u32, count: u32) -> bool {
    index < count
}

fn distance_project(
    positions: &mut [Vec3],
    inverse_masses: &[f64],
    a: u32,
    b: u32,
    rest_length: f64,
    stiffness: f64,
    compliance: f64,
    lambda: &mut f64,
    dt: f64,
) -> Option<f64> {
    let ia = a as usize;
    let ib = b as usize;
    let wa = inverse_masses[ia];
    let wb = inverse_masses[ib];
    if wa + wb <= EPSILON {
        return Some(0.0);
    }
    let pa = vec3_to_rapier(positions[ia]);
    let pb = vec3_to_rapier(positions[ib]);
    let delta = pb - pa;
    let length = delta.length();
    if length <= EPSILON {
        return Some(0.0);
    }
    let normal = delta / length;
    let c = length - rest_length;
    let alpha = if dt > EPSILON {
        compliance / (dt * dt)
    } else {
        0.0
    };
    let delta_lambda = if compliance > 0.0 {
        -(c + alpha * *lambda) / (wa + wb + alpha)
    } else {
        -stiffness.clamp(0.0, 1.0) * c / (wa + wb)
    };
    if compliance > 0.0 {
        *lambda += delta_lambda;
    }
    let correction = normal * delta_lambda;
    positions[ia] = vec3_from_rapier(pa - correction * wa);
    positions[ib] = vec3_from_rapier(pb + correction * wb);
    Some(c.abs())
}

/// Predict soft-body particle positions for one step (apply damping and
/// gravity to velocities, then integrate).
///
/// # Safety
///
/// `positions`, `velocities` and `inverse_masses` must each point to
/// `particle_count` readable elements (`Vec3` / `f64`);
/// `out_predicted_positions` must point to writable memory for `capacity`
/// `Vec3` elements with `particle_count <= capacity`. A null input or
/// output pointer fails with `ERR_NULL_POINTER`; `out_report` may be null.
/// No ownership is transferred.
#[unsafe(no_mangle)]
pub extern "C" fn softbody_predict_positions(
    positions: *const Vec3,
    velocities: *const Vec3,
    inverse_masses: *const f64,
    particle_count: u32,
    gravity: Vec3,
    damping: f64,
    dt: f64,
    out_predicted_positions: *mut Vec3,
    capacity: u32,
    out_report: *mut SoftBodyStepReport,
) -> Bool {
    if particle_count == 0 || particle_count > MAX_PARTICLES || capacity < particle_count {
        set_error(ERR_CAPACITY, "invalid soft-body prediction capacity");
        return Bool::FALSE;
    }
    if positions.is_null()
        || velocities.is_null()
        || inverse_masses.is_null()
        || out_predicted_positions.is_null()
    {
        set_error(ERR_NULL_POINTER, "soft-body prediction pointers are null");
        return Bool::FALSE;
    }
    if !vec3_finite(gravity) || !finite_non_negative(damping) || !finite_non_negative(dt) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid soft-body prediction parameters",
        );
        return Bool::FALSE;
    }

    let count = particle_count as usize;
    let positions = match unsafe { crate::ffi::checked_input_slice(positions, count, "positions") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    let velocities = match unsafe { crate::ffi::checked_input_slice(velocities, count, "velocities") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    let inverse_masses = match unsafe { crate::ffi::checked_input_slice(inverse_masses, count, "inverse_masses") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    let write_count = count.min(capacity as usize);
    let out_positions = match unsafe { crate::ffi::checked_output_slice(out_predicted_positions, write_count, "out_predicted_positions") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    let gravity = vec3_to_rapier(gravity);
    let velocity_scale = (1.0 - damping * dt).max(0.0);
    let mut active_particles = 0;
    let mut max_displacement = 0.0;
    for i in 0..write_count {
        if !vec3_finite(positions[i])
            || !vec3_finite(velocities[i])
            || !finite_non_negative(inverse_masses[i])
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid soft-body particle data");
            return Bool::FALSE;
        }
        let pos = vec3_to_rapier(positions[i]);
        let mut velocity = vec3_to_rapier(velocities[i]) * velocity_scale;
        if inverse_masses[i] > 0.0 {
            velocity += gravity * dt;
            active_particles += 1;
        }
        let predicted = pos + velocity * dt;
        out_positions[i] = vec3_from_rapier(predicted);
        max_displacement = f64::max(max_displacement, (predicted - pos).length());
    }

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = SoftBodyStepReport {
            particle_count,
            constraint_count: 0,
            active_particle_count: active_particles,
            max_correction: max_displacement,
            total_error: 0.0,
        };
    }
    clear_error();
    Bool::TRUE
}

/// Accumulate mass-spring forces (stiffness plus relative-velocity damping)
/// into per-particle force vectors.
///
/// # Safety
///
/// `positions` and `velocities` must each point to `particle_count`
/// readable `Vec3` elements; `out_forces` must point to writable memory for
/// `force_capacity` `Vec3` elements with `particle_count <= force_capacity`.
/// `springs` must point to `spring_count` readable `SoftSpring` elements
/// and be non-null when `spring_count > 0`. Null pointers fail with
/// `ERR_NULL_POINTER`; `out_report` may be null. No ownership is
/// transferred.
#[unsafe(no_mangle)]
pub extern "C" fn softbody_mass_spring_forces(
    positions: *const Vec3,
    velocities: *const Vec3,
    particle_count: u32,
    springs: *const SoftSpring,
    spring_count: u32,
    out_forces: *mut Vec3,
    force_capacity: u32,
    out_report: *mut SoftBodyStepReport,
) -> Bool {
    if particle_count == 0
        || particle_count > MAX_PARTICLES
        || spring_count > MAX_CONSTRAINTS
        || force_capacity < particle_count
    {
        set_error(ERR_CAPACITY, "invalid soft spring capacity");
        return Bool::FALSE;
    }
    if positions.is_null()
        || velocities.is_null()
        || out_forces.is_null()
        || (spring_count > 0 && springs.is_null())
    {
        set_error(ERR_NULL_POINTER, "soft spring pointers are null");
        return Bool::FALSE;
    }

    let count = particle_count as usize;
    let positions = match unsafe { crate::ffi::checked_input_slice(positions, count, "positions") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    let velocities = match unsafe { crate::ffi::checked_input_slice(velocities, count, "velocities") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    let springs = if spring_count == 0 { &[] } else { match unsafe { crate::ffi::checked_input_slice(springs, spring_count as usize, "springs") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } } };
    let write_count = count.min(force_capacity as usize);
    let out_forces = match unsafe { crate::ffi::checked_output_slice(out_forces, write_count, "out_forces") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    out_forces[..write_count].fill(Vec3::default());

    let mut total_error_acc = KahanSum::default();
    let mut max_force = 0.0;
    for spring in springs {
        if !index_valid(spring.particle_a, particle_count)
            || !index_valid(spring.particle_b, particle_count)
            || !finite_non_negative(spring.rest_length)
            || !finite_non_negative(spring.stiffness)
            || !finite_non_negative(spring.damping)
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid soft spring");
            return Bool::FALSE;
        }
        let a = spring.particle_a as usize;
        let b = spring.particle_b as usize;
        let pa = vec3_to_rapier(positions[a]);
        let pb = vec3_to_rapier(positions[b]);
        let delta = pb - pa;
        let length = delta.length();
        if length <= EPSILON {
            continue;
        }
        let normal = delta / length;
        let relative_velocity = vec3_to_rapier(velocities[b]) - vec3_to_rapier(velocities[a]);
        let force = normal
            * (spring.stiffness * (length - spring.rest_length)
                + spring.damping * relative_velocity.dot(normal));
        let fa = vec3_to_rapier(out_forces[a]) + force;
        let fb = vec3_to_rapier(out_forces[b]) - force;
        out_forces[a] = vec3_from_rapier(fa);
        out_forces[b] = vec3_from_rapier(fb);
        total_error_acc.add((length - spring.rest_length).abs());
        max_force = f64::max(max_force, force.length());
    }

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = SoftBodyStepReport {
            particle_count,
            constraint_count: spring_count,
            active_particle_count: particle_count,
            max_correction: max_force,
            total_error: total_error_acc.value(),
        };
    }
    clear_error();
    Bool::TRUE
}

/// Solve XPBD distance constraints, updating particle positions and
/// constraint `lambda` accumulators in place.
///
/// # Safety
///
/// `positions` must point to writable memory for `particle_count` `Vec3`
/// elements and `inverse_masses` to `particle_count` readable `f64`
/// elements; `constraints` must point to writable memory for
/// `constraint_count` `SoftDistanceConstraint` elements and be non-null
/// when `constraint_count > 0`. Null pointers fail with
/// `ERR_NULL_POINTER`; `out_report` may be null. No ownership is
/// transferred.
#[unsafe(no_mangle)]
pub extern "C" fn softbody_solve_xpbd_distance_constraints(
    positions: *mut Vec3,
    inverse_masses: *const f64,
    particle_count: u32,
    constraints: *mut SoftDistanceConstraint,
    constraint_count: u32,
    dt: f64,
    iterations: u32,
    out_report: *mut SoftBodyStepReport,
) -> Bool {
    if particle_count == 0
        || particle_count > MAX_PARTICLES
        || constraint_count > MAX_CONSTRAINTS
        || iterations > 10_000
    {
        set_error(ERR_CAPACITY, "invalid XPBD distance capacity");
        return Bool::FALSE;
    }
    if positions.is_null()
        || inverse_masses.is_null()
        || (constraint_count > 0 && constraints.is_null())
    {
        set_error(ERR_NULL_POINTER, "XPBD distance pointers are null");
        return Bool::FALSE;
    }
    if !finite_non_negative(dt) {
        set_error(ERR_INVALID_ARGUMENT, "invalid XPBD distance timestep");
        return Bool::FALSE;
    }
    let positions = match unsafe { crate::ffi::checked_output_slice(positions, particle_count as usize, "positions") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    let inverse_masses = match unsafe { crate::ffi::checked_input_slice(inverse_masses, particle_count as usize, "inverse_masses") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    let constraints = if constraint_count == 0 { &mut [] } else { match unsafe { crate::ffi::checked_output_slice(constraints, constraint_count as usize, "constraints") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } } };
    let mut total_error_acc = KahanSum::default();
    let mut max_correction = 0.0;
    for _ in 0..iterations.max(1) {
        total_error_acc.reset();
        max_correction = 0.0;
        for constraint in constraints.iter_mut() {
            if !index_valid(constraint.particle_a, particle_count)
                || !index_valid(constraint.particle_b, particle_count)
                || !finite_non_negative(constraint.rest_length)
                || !finite_non_negative(constraint.stiffness)
                || !finite_non_negative(constraint.compliance)
                || !constraint.lambda.is_finite()
            {
                set_error(ERR_INVALID_ARGUMENT, "invalid XPBD distance constraint");
                return Bool::FALSE;
            }
            let Some(error) = distance_project(
                positions,
                inverse_masses,
                constraint.particle_a,
                constraint.particle_b,
                constraint.rest_length,
                constraint.stiffness,
                constraint.compliance,
                &mut constraint.lambda,
                dt,
            ) else {
                set_error(ERR_INVALID_ARGUMENT, "invalid XPBD distance projection");
                return Bool::FALSE;
            };
            total_error_acc.add(error);
            max_correction = f64::max(max_correction, error);
        }
    }
    let total_error = total_error_acc.value();
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = SoftBodyStepReport {
            particle_count,
            constraint_count,
            active_particle_count: inverse_masses.iter().filter(|mass| **mass > 0.0).count() as u32,
            max_correction,
            total_error,
        };
    }
    clear_error();
    Bool::TRUE
}

/// Solve XPBD bending constraints as distance constraints between their
/// particle pairs, updating positions and `lambda` accumulators in place.
///
/// # Safety
///
/// Same contract as `softbody_solve_xpbd_distance_constraints`: `positions`
/// must point to writable memory for `particle_count` `Vec3` elements,
/// `inverse_masses` to `particle_count` readable `f64` elements, and
/// `constraints` to `constraint_count` writable `SoftBendingConstraint`
/// elements, non-null when `constraint_count > 0`. Null pointers fail with
/// `ERR_NULL_POINTER`; `out_report` may be null. No ownership is
/// transferred.
#[unsafe(no_mangle)]
pub extern "C" fn softbody_solve_xpbd_bending_constraints(
    positions: *mut Vec3,
    inverse_masses: *const f64,
    particle_count: u32,
    constraints: *mut SoftBendingConstraint,
    constraint_count: u32,
    dt: f64,
    iterations: u32,
    out_report: *mut SoftBodyStepReport,
) -> Bool {
    if constraints.is_null() && constraint_count > 0 {
        set_error(ERR_NULL_POINTER, "XPBD bending constraints are null");
        return Bool::FALSE;
    }
    let constraints_slice = if constraint_count == 0 {
        &mut []
    } else {
        unsafe { slice::from_raw_parts_mut(constraints, constraint_count as usize) }
    };
    let mut distance_constraints = constraints_slice
        .iter()
        .map(|constraint| SoftDistanceConstraint {
            particle_a: constraint.particle_a,
            particle_b: constraint.particle_b,
            rest_length: constraint.rest_distance,
            stiffness: constraint.stiffness,
            compliance: constraint.compliance,
            lambda: constraint.lambda,
        })
        .collect::<Vec<_>>();
    let result = softbody_solve_xpbd_distance_constraints(
        positions,
        inverse_masses,
        particle_count,
        distance_constraints.as_mut_ptr(),
        constraint_count,
        dt,
        iterations,
        out_report,
    );
    if result == Bool::TRUE {
        for (source, target) in distance_constraints
            .iter()
            .zip(constraints_slice.iter_mut())
        {
            target.lambda = source.lambda;
        }
    }
    result
}

/// Project active particles out of collision spheres, updating positions in
/// place.
///
/// # Safety
///
/// `positions` must point to writable memory for `particle_count` `Vec3`
/// elements and `inverse_masses` to `particle_count` readable `f64`
/// elements; `spheres` must point to `sphere_count` readable
/// `SoftSphereCollision` elements and be non-null when `sphere_count > 0`.
/// Null pointers fail with `ERR_NULL_POINTER`; `out_report` may be null.
/// No ownership is transferred.
#[unsafe(no_mangle)]
pub extern "C" fn softbody_solve_sphere_collision_constraints(
    positions: *mut Vec3,
    inverse_masses: *const f64,
    particle_count: u32,
    spheres: *const SoftSphereCollision,
    sphere_count: u32,
    out_report: *mut SoftBodyStepReport,
) -> Bool {
    if particle_count == 0 || particle_count > MAX_PARTICLES || sphere_count > MAX_CONSTRAINTS {
        set_error(ERR_CAPACITY, "invalid soft collision capacity");
        return Bool::FALSE;
    }
    if positions.is_null() || inverse_masses.is_null() || (sphere_count > 0 && spheres.is_null()) {
        set_error(ERR_NULL_POINTER, "soft collision pointers are null");
        return Bool::FALSE;
    }
    let positions = match unsafe { crate::ffi::checked_output_slice(positions, particle_count as usize, "positions") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    let inverse_masses = match unsafe { crate::ffi::checked_input_slice(inverse_masses, particle_count as usize, "inverse_masses") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    let spheres = if sphere_count == 0 { &[] } else { match unsafe { crate::ffi::checked_input_slice(spheres, sphere_count as usize, "spheres") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } } };
    let mut total_error_acc = KahanSum::default();
    let mut max_correction = 0.0;
    for sphere in spheres {
        if !vec3_finite(sphere.center) || !finite_non_negative(sphere.radius) {
            set_error(ERR_INVALID_ARGUMENT, "invalid soft collision sphere");
            return Bool::FALSE;
        }
        let center = vec3_to_rapier(sphere.center);
        for i in 0..particle_count as usize {
            if inverse_masses[i] <= 0.0 {
                continue;
            }
            let pos = vec3_to_rapier(positions[i]);
            let delta = pos - center;
            let distance = delta.length();
            if distance < sphere.radius {
                let normal = if distance > EPSILON {
                    delta / distance
                } else {
                    crate::math::Vector3f64::y()
                };
                let corrected = center + normal * sphere.radius;
                let correction = (corrected - pos).length();
                positions[i] = vec3_from_rapier(corrected);
                total_error_acc.add(correction);
                max_correction = f64::max(max_correction, correction);
            }
        }
    }
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = SoftBodyStepReport {
            particle_count,
            constraint_count: sphere_count,
            active_particle_count: inverse_masses.iter().filter(|mass| **mass > 0.0).count() as u32,
            max_correction,
            total_error: total_error_acc.value(),
        };
    }
    clear_error();
    Bool::TRUE
}

pub fn tetra_volume(a: Vector3f64, b: Vector3f64, c: Vector3f64, d: Vector3f64) -> f64 {
    (b - a).dot((c - a).cross(d - a)) / 6.0
}

/// Solve XPBD volume constraints on tetrahedra, updating particle positions
/// and constraint `lambda` accumulators in place.
///
/// # Safety
///
/// `positions` must point to writable memory for `particle_count` `Vec3`
/// elements and `inverse_masses` to `particle_count` readable `f64`
/// elements; `constraints` must point to writable memory for
/// `constraint_count` `SoftVolumeConstraint` elements and be non-null when
/// `constraint_count > 0`. Null pointers fail with `ERR_NULL_POINTER`;
/// `out_report` may be null. No ownership is transferred.
#[unsafe(no_mangle)]
pub extern "C" fn softbody_solve_xpbd_volume_constraints(
    positions: *mut Vec3,
    inverse_masses: *const f64,
    particle_count: u32,
    constraints: *mut SoftVolumeConstraint,
    constraint_count: u32,
    dt: f64,
    iterations: u32,
    out_report: *mut SoftBodyStepReport,
) -> Bool {
    if particle_count == 0
        || particle_count > MAX_PARTICLES
        || constraint_count > MAX_CONSTRAINTS
        || iterations > 10_000
    {
        set_error(ERR_CAPACITY, "invalid XPBD volume capacity");
        return Bool::FALSE;
    }
    if positions.is_null()
        || inverse_masses.is_null()
        || (constraint_count > 0 && constraints.is_null())
    {
        set_error(ERR_NULL_POINTER, "XPBD volume pointers are null");
        return Bool::FALSE;
    }
    if !finite_non_negative(dt) {
        set_error(ERR_INVALID_ARGUMENT, "invalid XPBD volume timestep");
        return Bool::FALSE;
    }
    let positions = unsafe { slice::from_raw_parts_mut(positions, particle_count as usize) };
    let inverse_masses = unsafe { slice::from_raw_parts(inverse_masses, particle_count as usize) };
    let constraints = unsafe { slice::from_raw_parts_mut(constraints, constraint_count as usize) };
    let mut total_error_acc = KahanSum::default();
    let mut max_correction = 0.0;
    for _ in 0..iterations.max(1) {
        total_error_acc.reset();
        max_correction = 0.0;
        for constraint in constraints.iter_mut() {
            if !index_valid(constraint.particle_a, particle_count)
                || !index_valid(constraint.particle_b, particle_count)
                || !index_valid(constraint.particle_c, particle_count)
                || !index_valid(constraint.particle_d, particle_count)
                || !constraint.rest_volume.is_finite()
                || !finite_non_negative(constraint.compliance)
                || !constraint.lambda.is_finite()
            {
                set_error(ERR_INVALID_ARGUMENT, "invalid XPBD volume constraint");
                return Bool::FALSE;
            }
            let ids = [
                constraint.particle_a as usize,
                constraint.particle_b as usize,
                constraint.particle_c as usize,
                constraint.particle_d as usize,
            ];
            let p = ids
                .iter()
                .map(|&id| vec3_to_rapier(positions[id]))
                .collect::<Vec<_>>();
            let volume = tetra_volume(p[0], p[1], p[2], p[3]);
            let c = volume - constraint.rest_volume;
            let gradients = [
                (p[3] - p[1]).cross(p[2] - p[1]) / 6.0,
                (p[2] - p[0]).cross(p[3] - p[0]) / 6.0,
                (p[3] - p[0]).cross(p[1] - p[0]) / 6.0,
                (p[1] - p[0]).cross(p[2] - p[0]) / 6.0,
            ];
            let mut denominator = KahanSum::default();
            for i in 0..4 {
                denominator.add(inverse_masses[ids[i]] * gradients[i].length_squared());
            }
            let denom_val = denominator.value();
            let alpha = if dt > EPSILON {
                constraint.compliance / (dt * dt)
            } else {
                0.0
            };
            if denom_val + alpha <= EPSILON {
                continue;
            }
            let delta_lambda = -(c + alpha * constraint.lambda) / (denom_val + alpha);
            constraint.lambda += delta_lambda;
            for i in 0..4 {
                let corrected = p[i] + gradients[i] * (inverse_masses[ids[i]] * delta_lambda);
                positions[ids[i]] = vec3_from_rapier(corrected);
            }
            let error = c.abs();
            total_error_acc.add(error);
            max_correction = f64::max(max_correction, error);
        }
    }
    let total_error = total_error_acc.value();
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = SoftBodyStepReport {
            particle_count,
            constraint_count,
            active_particle_count: inverse_masses.iter().filter(|mass| **mass > 0.0).count() as u32,
            max_correction,
            total_error,
        };
    }
    clear_error();
    Bool::TRUE
}

/// Update particle velocities from the previous and current positions over
/// one timestep.
///
/// # Safety
///
/// `previous_positions` and `current_positions` must each point to
/// `particle_count` readable `Vec3` elements; `out_velocities` must point
/// to writable memory for `capacity` `Vec3` elements with
/// `particle_count <= capacity`. Null pointers fail with
/// `ERR_NULL_POINTER`; `out_report` may be null. No ownership is
/// transferred.
#[unsafe(no_mangle)]
pub extern "C" fn softbody_update_velocities(
    previous_positions: *const Vec3,
    current_positions: *const Vec3,
    particle_count: u32,
    dt: f64,
    out_velocities: *mut Vec3,
    capacity: u32,
    out_report: *mut SoftBodyStepReport,
) -> Bool {
    if particle_count == 0 || particle_count > MAX_PARTICLES || capacity < particle_count {
        set_error(ERR_CAPACITY, "invalid soft velocity update capacity");
        return Bool::FALSE;
    }
    if previous_positions.is_null() || current_positions.is_null() || out_velocities.is_null() {
        set_error(ERR_NULL_POINTER, "soft velocity update pointers are null");
        return Bool::FALSE;
    }
    if !finite_positive(dt) {
        set_error(ERR_INVALID_ARGUMENT, "invalid soft velocity timestep");
        return Bool::FALSE;
    }
    let count = particle_count as usize;
    let previous = unsafe { slice::from_raw_parts(previous_positions, count) };
    let current = unsafe { slice::from_raw_parts(current_positions, count) };
    let write_count = count.min(capacity as usize);
    let velocities = unsafe { slice::from_raw_parts_mut(out_velocities, write_count) };
    let mut max_speed = 0.0;
    for i in 0..write_count {
        if !vec3_finite(previous[i]) || !vec3_finite(current[i]) {
            set_error(ERR_INVALID_ARGUMENT, "invalid soft velocity position data");
            return Bool::FALSE;
        }
        let velocity = (vec3_to_rapier(current[i]) - vec3_to_rapier(previous[i])) / dt;
        velocities[i] = vec3_from_rapier(velocity);
        max_speed = f64::max(max_speed, velocity.length());
    }
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = SoftBodyStepReport {
            particle_count,
            constraint_count: 0,
            active_particle_count: particle_count,
            max_correction: max_speed,
            total_error: 0.0,
        };
    }
    clear_error();
    Bool::TRUE
}
