use std::slice;

use rapier3d::prelude::Vector;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, SoftBendingConstraint, SoftBodyStepReport, SoftDistanceConstraint, SoftSphereCollision,
    SoftSpring, SoftVolumeConstraint, Vec3, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

use crate::rapier::math::{KahanSum, finite_non_negative, finite_positive};

const EPSILON: f64 = 1.0e-12;
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
    let positions = unsafe { slice::from_raw_parts(positions, count) };
    let velocities = unsafe { slice::from_raw_parts(velocities, count) };
    let inverse_masses = unsafe { slice::from_raw_parts(inverse_masses, count) };
    let write_count = count.min(capacity as usize);
    let out_positions =
        unsafe { slice::from_raw_parts_mut(out_predicted_positions, write_count) };
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
    let positions = unsafe { slice::from_raw_parts(positions, count) };
    let velocities = unsafe { slice::from_raw_parts(velocities, count) };
    let springs = unsafe { slice::from_raw_parts(springs, spring_count as usize) };
    let write_count = count.min(force_capacity as usize);
    let out_forces = unsafe { slice::from_raw_parts_mut(out_forces, write_count) };
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
    let positions = unsafe { slice::from_raw_parts_mut(positions, particle_count as usize) };
    let inverse_masses = unsafe { slice::from_raw_parts(inverse_masses, particle_count as usize) };
    let spheres = unsafe { slice::from_raw_parts(spheres, sphere_count as usize) };
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
                    Vector::Y
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

fn tetra_volume(a: Vector, b: Vector, c: Vector, d: Vector) -> f64 {
    (b - a).dot((c - a).cross(d - a)) / 6.0
}

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

#[cfg(test)]
mod tests {
    use super::*;

    fn v3(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    #[test]
    fn prediction_spring_and_distance_constraint_work() {
        let positions = [v3(0.0, 0.0, 0.0), v3(2.0, 0.0, 0.0)];
        let velocities = [v3(0.0, 0.0, 0.0), v3(0.0, 0.0, 0.0)];
        let inverse_masses = [1.0, 1.0];
        let mut predicted = [Vec3::default(); 2];
        assert_eq!(
            softbody_predict_positions(
                positions.as_ptr(),
                velocities.as_ptr(),
                inverse_masses.as_ptr(),
                2,
                v3(0.0, -10.0, 0.0),
                0.0,
                0.1,
                predicted.as_mut_ptr(),
                2,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert!(predicted[0].y < 0.0);

        let spring = [SoftSpring {
            particle_a: 0,
            particle_b: 1,
            rest_length: 1.0,
            stiffness: 10.0,
            damping: 0.0,
        }];
        let mut forces = [Vec3::default(); 2];
        assert_eq!(
            softbody_mass_spring_forces(
                positions.as_ptr(),
                velocities.as_ptr(),
                2,
                spring.as_ptr(),
                1,
                forces.as_mut_ptr(),
                2,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert!(forces[0].x > 0.0);

        let mut projected = positions;
        let mut constraints = [SoftDistanceConstraint {
            particle_a: 0,
            particle_b: 1,
            rest_length: 1.0,
            stiffness: 1.0,
            compliance: 0.0,
            lambda: 0.0,
        }];
        assert_eq!(
            softbody_solve_xpbd_distance_constraints(
                projected.as_mut_ptr(),
                inverse_masses.as_ptr(),
                2,
                constraints.as_mut_ptr(),
                1,
                0.1,
                4,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        let distance = (vec3_to_rapier(projected[1]) - vec3_to_rapier(projected[0])).length();
        assert!((distance - 1.0).abs() < 1.0e-8);
    }

    #[test]
    fn collision_volume_and_velocity_update_work() {
        let inverse_masses = [1.0, 1.0, 1.0, 1.0];
        let mut positions = [
            v3(0.0, 0.0, 0.0),
            v3(1.2, 0.0, 0.0),
            v3(0.0, 1.0, 0.0),
            v3(0.0, 0.0, 1.0),
        ];
        let mut volumes = [SoftVolumeConstraint {
            particle_a: 0,
            particle_b: 1,
            particle_c: 2,
            particle_d: 3,
            rest_volume: 1.0 / 6.0,
            compliance: 0.0,
            lambda: 0.0,
        }];
        assert_eq!(
            softbody_solve_xpbd_volume_constraints(
                positions.as_mut_ptr(),
                inverse_masses.as_ptr(),
                4,
                volumes.as_mut_ptr(),
                1,
                0.1,
                8,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        let volume = tetra_volume(
            vec3_to_rapier(positions[0]),
            vec3_to_rapier(positions[1]),
            vec3_to_rapier(positions[2]),
            vec3_to_rapier(positions[3]),
        );
        assert!((volume - 1.0 / 6.0).abs() < 1.0e-4);

        let mut colliding = [v3(0.0, 0.0, 0.0)];
        let sphere = [SoftSphereCollision {
            center: v3(0.0, 0.0, 0.0),
            radius: 2.0,
        }];
        assert_eq!(
            softbody_solve_sphere_collision_constraints(
                colliding.as_mut_ptr(),
                inverse_masses.as_ptr(),
                1,
                sphere.as_ptr(),
                1,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert!((vec3_to_rapier(colliding[0]).length() - 2.0).abs() < 1.0e-8);

        let mut velocities = [Vec3::default()];
        let previous = [v3(0.0, 0.0, 0.0)];
        assert_eq!(
            softbody_update_velocities(
                previous.as_ptr(),
                colliding.as_ptr(),
                1,
                0.5,
                velocities.as_mut_ptr(),
                1,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert!(vec3_to_rapier(velocities[0]).length() > 0.0);
    }
}
