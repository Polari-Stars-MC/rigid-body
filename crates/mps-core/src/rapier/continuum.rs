use std::slice;

use rapier3d::prelude::{Matrix3, Vector};

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, FemConstitutiveReport, FemShapeFunctionReport, FemTetrahedron, MaterialProperties,
    NewmarkBetaParameters, NewmarkBetaReport, Vec3, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

use crate::rapier::math::finite_positive;

const EPSILON: f64 = 1.0e-12;
const MAX_DOF: u32 = 512;

fn material_valid(material: MaterialProperties) -> bool {
    finite_positive(material.youngs_modulus)
        && material.poisson_ratio.is_finite()
        && material.poisson_ratio > -1.0
        && material.poisson_ratio < 0.5
}

fn tetra_valid(tetra: FemTetrahedron) -> bool {
    vec3_finite(tetra.a) && vec3_finite(tetra.b) && vec3_finite(tetra.c) && vec3_finite(tetra.d)
}

fn signed_tetra_volume(a: Vector, b: Vector, c: Vector, d: Vector) -> f64 {
    (b - a).dot((c - a).cross(d - a)) / 6.0
}

fn tetra_gradients(tetra: FemTetrahedron) -> Option<([Vector; 4], f64)> {
    if !tetra_valid(tetra) {
        return None;
    }
    let a = vec3_to_rapier(tetra.a);
    let b = vec3_to_rapier(tetra.b);
    let c = vec3_to_rapier(tetra.c);
    let d = vec3_to_rapier(tetra.d);
    let volume = signed_tetra_volume(a, b, c, d);
    if volume.abs() <= EPSILON {
        return None;
    }
    let six_v = 6.0 * volume;
    let gradients = [
        (c - b).cross(d - b) / six_v,
        (d - a).cross(c - a) / six_v,
        (b - a).cross(d - a) / six_v,
        (c - a).cross(b - a) / six_v,
    ];
    Some((gradients, volume.abs()))
}

fn barycentric_weights(tetra: FemTetrahedron, point: Vec3) -> Option<([f64; 4], f64)> {
    if !tetra_valid(tetra) || !vec3_finite(point) {
        return None;
    }
    let a = vec3_to_rapier(tetra.a);
    let b = vec3_to_rapier(tetra.b);
    let c = vec3_to_rapier(tetra.c);
    let d = vec3_to_rapier(tetra.d);
    let p = vec3_to_rapier(point);
    let volume = signed_tetra_volume(a, b, c, d);
    if volume.abs() <= EPSILON {
        return None;
    }
    Some((
        [
            signed_tetra_volume(p, b, c, d) / volume,
            signed_tetra_volume(a, p, c, d) / volume,
            signed_tetra_volume(a, b, p, d) / volume,
            signed_tetra_volume(a, b, c, p) / volume,
        ],
        volume.abs(),
    ))
}

fn dense_mat_vec_out(matrix: &[f64], vector: &[f64], n: usize, out: &mut [f64]) {
    for row in 0..n {
        let mut sum = 0.0;
        for col in 0..n {
            sum += matrix[row * n + col] * vector[col];
        }
        out[row] = sum;
    }
}

fn solve_dense_system_in_place(matrix: &mut [f64], rhs: &mut [f64], x: &mut [f64], n: usize) -> bool {
    for pivot in 0..n {
        let mut pivot_row = pivot;
        let mut pivot_abs = matrix[pivot * n + pivot].abs();
        for row in pivot + 1..n {
            let value = matrix[row * n + pivot].abs();
            if value > pivot_abs {
                pivot_abs = value;
                pivot_row = row;
            }
        }
        if pivot_abs <= EPSILON {
            return false;
        }
        if pivot_row != pivot {
            for col in 0..n {
                matrix.swap(pivot * n + col, pivot_row * n + col);
            }
            rhs.swap(pivot, pivot_row);
        }
        let pivot_value = matrix[pivot * n + pivot];
        for row in pivot + 1..n {
            let factor = matrix[row * n + pivot] / pivot_value;
            matrix[row * n + pivot] = 0.0;
            for col in pivot + 1..n {
                matrix[row * n + col] -= factor * matrix[pivot * n + col];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }

    for row in (0..n).rev() {
        let mut sum = rhs[row];
        for col in row + 1..n {
            sum -= matrix[row * n + col] * x[col];
        }
        x[row] = sum / matrix[row * n + row];
    }
    true
}

#[unsafe(no_mangle)]
pub extern "C" fn continuum_tetra_volume(tetra: FemTetrahedron) -> f64 {
    if !tetra_valid(tetra) {
        return f64::NAN;
    }
    let a = vec3_to_rapier(tetra.a);
    let b = vec3_to_rapier(tetra.b);
    let c = vec3_to_rapier(tetra.c);
    let d = vec3_to_rapier(tetra.d);
    signed_tetra_volume(a, b, c, d).abs()
}

#[unsafe(no_mangle)]
pub extern "C" fn continuum_tetra_shape_functions(
    tetra: FemTetrahedron,
    point: Vec3,
    out_report: *mut FemShapeFunctionReport,
) -> Bool {
    let Some((weights, volume)) = barycentric_weights(tetra, point) else {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid tetrahedral shape function input",
        );
        return Bool::FALSE;
    };
    let Some((gradients, _)) = tetra_gradients(tetra) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid tetrahedral gradient input");
        return Bool::FALSE;
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "shape function output is null");
        return Bool::FALSE;
    };
    let inside = weights
        .iter()
        .all(|weight| *weight >= -1.0e-9 && *weight <= 1.0 + 1.0e-9);
    *out_report = FemShapeFunctionReport {
        weights,
        gradients: gradients.map(vec3_from_rapier),
        volume,
        inside: Bool::from(inside),
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn continuum_linear_elastic_constitutive_matrix(
    material: MaterialProperties,
    out_matrix: *mut f64,
    capacity: u32,
    out_report: *mut FemConstitutiveReport,
) -> Bool {
    if capacity < 36 || out_matrix.is_null() {
        set_error(
            ERR_CAPACITY,
            "constitutive matrix capacity must be at least 36",
        );
        return Bool::FALSE;
    }
    if !material_valid(material) {
        set_error(ERR_INVALID_ARGUMENT, "invalid linear elastic material");
        return Bool::FALSE;
    }
    let lambda = material.youngs_modulus * material.poisson_ratio
        / ((1.0 + material.poisson_ratio) * (1.0 - 2.0 * material.poisson_ratio));
    let shear = material.youngs_modulus / (2.0 * (1.0 + material.poisson_ratio));
    let matrix = unsafe { slice::from_raw_parts_mut(out_matrix, capacity as usize) };
    matrix[..36].fill(0.0);
    for row in 0..3 {
        for col in 0..3 {
            matrix[row * 6 + col] = lambda;
        }
        matrix[row * 6 + row] += 2.0 * shear;
    }
    matrix[3 * 6 + 3] = shear;
    matrix[4 * 6 + 4] = shear;
    matrix[5 * 6 + 5] = shear;
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = FemConstitutiveReport {
            lambda,
            shear_modulus: shear,
            bulk_modulus: material.youngs_modulus / (3.0 * (1.0 - 2.0 * material.poisson_ratio)),
            matrix_size: 6,
        };
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn continuum_tetra_strain_displacement_matrix(
    tetra: FemTetrahedron,
    out_matrix: *mut f64,
    capacity: u32,
    out_volume: *mut f64,
) -> Bool {
    if capacity < 72 || out_matrix.is_null() {
        set_error(
            ERR_CAPACITY,
            "strain-displacement matrix capacity must be at least 72",
        );
        return Bool::FALSE;
    }
    let Some((gradients, volume)) = tetra_gradients(tetra) else {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid tetrahedral strain matrix input",
        );
        return Bool::FALSE;
    };
    let matrix = unsafe { slice::from_raw_parts_mut(out_matrix, capacity as usize) };
    matrix[..72].fill(0.0);
    for (node, gradient) in gradients.iter().enumerate() {
        let col = node * 3;
        let gx = gradient.x;
        let gy = gradient.y;
        let gz = gradient.z;
        matrix[col] = gx;
        matrix[6 + col + 1] = gy;
        matrix[12 + col + 2] = gz;
        matrix[18 + col] = gy;
        matrix[18 + col + 1] = gx;
        matrix[24 + col + 1] = gz;
        matrix[24 + col + 2] = gy;
        matrix[30 + col] = gz;
        matrix[30 + col + 2] = gx;
    }
    if let Some(out_volume) = unsafe { out_volume.as_mut() } {
        *out_volume = volume;
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn continuum_newmark_beta_solve(
    mass_matrix: *const f64,
    damping_matrix: *const f64,
    stiffness_matrix: *const f64,
    displacement: *const f64,
    velocity: *const f64,
    acceleration: *const f64,
    external_force: *const f64,
    dof: u32,
    params: NewmarkBetaParameters,
    out_delta_displacement: *mut f64,
    out_next_displacement: *mut f64,
    out_next_velocity: *mut f64,
    out_next_acceleration: *mut f64,
    capacity: u32,
    out_report: *mut NewmarkBetaReport,
) -> Bool {
    if dof == 0 || dof > MAX_DOF || capacity < dof {
        set_error(ERR_CAPACITY, "invalid Newmark-beta system size");
        return Bool::FALSE;
    }
    if mass_matrix.is_null()
        || damping_matrix.is_null()
        || stiffness_matrix.is_null()
        || displacement.is_null()
        || velocity.is_null()
        || acceleration.is_null()
        || external_force.is_null()
        || out_delta_displacement.is_null()
        || out_next_displacement.is_null()
        || out_next_velocity.is_null()
        || out_next_acceleration.is_null()
    {
        set_error(ERR_NULL_POINTER, "Newmark-beta pointers are null");
        return Bool::FALSE;
    }
    if !finite_positive(params.dt) || !finite_positive(params.beta) || !params.gamma.is_finite() {
        set_error(ERR_INVALID_ARGUMENT, "invalid Newmark-beta parameters");
        return Bool::FALSE;
    }

    let n = dof as usize;
    let nn = n * n;
    let mass = unsafe { slice::from_raw_parts(mass_matrix, nn) };
    let damping = unsafe { slice::from_raw_parts(damping_matrix, nn) };
    let stiffness = unsafe { slice::from_raw_parts(stiffness_matrix, nn) };
    let u = unsafe { slice::from_raw_parts(displacement, n) };
    let v = unsafe { slice::from_raw_parts(velocity, n) };
    let a = unsafe { slice::from_raw_parts(acceleration, n) };
    let force = unsafe { slice::from_raw_parts(external_force, n) };
    if mass
        .iter()
        .chain(damping)
        .chain(stiffness)
        .chain(u)
        .chain(v)
        .chain(a)
        .chain(force)
        .any(|value| !value.is_finite())
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "Newmark-beta inputs contain non-finite values",
        );
        return Bool::FALSE;
    }

    let a0 = 1.0 / (params.beta * params.dt * params.dt);
    let a1 = params.gamma / (params.beta * params.dt);
    let mut effective_stiffness = vec![0.0; nn];
    for index in 0..nn {
        effective_stiffness[index] = stiffness[index] + a0 * mass[index] + a1 * damping[index];
    }

    let mut u_pred = vec![0.0; n];
    let mut v_pred = vec![0.0; n];
    for i in 0..n {
        u_pred[i] = u[i] + params.dt * v[i] + params.dt * params.dt * (0.5 - params.beta) * a[i];
        v_pred[i] = v[i] + params.dt * (1.0 - params.gamma) * a[i];
    }
    // Compute k_u_pred and c_v_pred into reusable buffers,
    // then fold into effective_force in-place.
    let mut effective_force = vec![0.0; n];
    dense_mat_vec_out(stiffness, &u_pred, n, &mut effective_force);
    dense_mat_vec_out(damping, &v_pred, n, &mut u_pred);
    for i in 0..n {
        effective_force[i] = force[i] - effective_force[i] - u_pred[i];
    }

    // Solve in-place to avoid the clone() / second allocation for `delta`.
    // effective_stiffness is consumed (modified in-place), effective_force too.
    let mut delta = vec![0.0; n];
    if !solve_dense_system_in_place(&mut effective_stiffness, &mut effective_force, &mut delta, n) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "Newmark-beta effective stiffness is singular",
        );
        return Bool::FALSE;
    };

    let out_delta = unsafe { slice::from_raw_parts_mut(out_delta_displacement, capacity as usize) };
    let out_u = unsafe { slice::from_raw_parts_mut(out_next_displacement, capacity as usize) };
    let out_v = unsafe { slice::from_raw_parts_mut(out_next_velocity, capacity as usize) };
    let out_a = unsafe { slice::from_raw_parts_mut(out_next_acceleration, capacity as usize) };
    let mut max_delta = 0.0;
    for i in 0..n {
        let next_acceleration = a0 * delta[i];
        out_delta[i] = delta[i];
        out_u[i] = u_pred[i] + delta[i];
        out_v[i] = v_pred[i] + params.gamma * params.dt * next_acceleration;
        out_a[i] = next_acceleration;
        max_delta = f64::max(max_delta, delta[i].abs());
    }

    // Reuse u_pred as scratch for residual computation
    let mut solved_force = u_pred; // was u_pred, now reused
    dense_mat_vec_out(&effective_stiffness, &delta, n, &mut solved_force);
    let residual_norm = solved_force
        .iter()
        .zip(effective_force.iter())
        .map(|(lhs, rhs)| (lhs - rhs) * (lhs - rhs))
        .sum::<f64>()
        .sqrt();
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = NewmarkBetaReport {
            dof,
            beta: params.beta,
            gamma: params.gamma,
            dt: params.dt,
            effective_stiffness_scale: a0,
            effective_damping_scale: a1,
            max_delta_displacement: max_delta,
            residual_norm,
        };
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn continuum_linear_tetra_element_stiffness(
    tetra: FemTetrahedron,
    material: MaterialProperties,
    out_stiffness: *mut f64,
    capacity: u32,
    out_volume: *mut f64,
) -> Bool {
    if capacity < 144 || out_stiffness.is_null() {
        set_error(
            ERR_CAPACITY,
            "tetra stiffness capacity must be at least 144",
        );
        return Bool::FALSE;
    }
    if !material_valid(material) {
        set_error(ERR_INVALID_ARGUMENT, "invalid tetra stiffness material");
        return Bool::FALSE;
    }
    let mut b_matrix = [0.0; 72];
    let mut volume = 0.0;
    if continuum_tetra_strain_displacement_matrix(
        tetra,
        b_matrix.as_mut_ptr(),
        b_matrix.len() as u32,
        &mut volume,
    ) != Bool::TRUE
    {
        return Bool::FALSE;
    }
    let mut d_matrix = [0.0; 36];
    if continuum_linear_elastic_constitutive_matrix(
        material,
        d_matrix.as_mut_ptr(),
        d_matrix.len() as u32,
        std::ptr::null_mut(),
    ) != Bool::TRUE
    {
        return Bool::FALSE;
    }

    let stiffness = unsafe { slice::from_raw_parts_mut(out_stiffness, capacity as usize) };
    stiffness[..144].fill(0.0);
    for i in 0..12 {
        for j in 0..12 {
            let mut sum = 0.0;
            for alpha in 0..6 {
                for beta in 0..6 {
                    sum += b_matrix[alpha * 12 + i]
                        * d_matrix[alpha * 6 + beta]
                        * b_matrix[beta * 12 + j];
                }
            }
            stiffness[i * 12 + j] = sum * volume;
        }
    }
    if let Some(out_volume) = unsafe { out_volume.as_mut() } {
        *out_volume = volume;
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn continuum_deformation_gradient(
    reference_tetra: FemTetrahedron,
    deformed_tetra: FemTetrahedron,
    out_matrix: *mut f64,
    capacity: u32,
) -> Bool {
    if capacity < 9 || out_matrix.is_null() {
        set_error(
            ERR_CAPACITY,
            "deformation gradient capacity must be at least 9",
        );
        return Bool::FALSE;
    }
    if !tetra_valid(reference_tetra) || !tetra_valid(deformed_tetra) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid deformation gradient tetrahedron",
        );
        return Bool::FALSE;
    }
    let x0 = vec3_to_rapier(reference_tetra.a);
    let x1 = vec3_to_rapier(reference_tetra.b);
    let x2 = vec3_to_rapier(reference_tetra.c);
    let x3 = vec3_to_rapier(reference_tetra.d);
    let y0 = vec3_to_rapier(deformed_tetra.a);
    let y1 = vec3_to_rapier(deformed_tetra.b);
    let y2 = vec3_to_rapier(deformed_tetra.c);
    let y3 = vec3_to_rapier(deformed_tetra.d);
    let dm = Matrix3::from_cols(x1 - x0, x2 - x0, x3 - x0);
    let Some(dm_inv) = dm.try_inverse() else {
        set_error(ERR_INVALID_ARGUMENT, "reference tetrahedron is degenerate");
        return Bool::FALSE;
    };
    let ds = Matrix3::from_cols(y1 - y0, y2 - y0, y3 - y0);
    let f = ds * dm_inv;
    let out = unsafe { slice::from_raw_parts_mut(out_matrix, capacity as usize) };
    let cols = f.to_cols_array();
    out[0] = cols[0];
    out[1] = cols[3];
    out[2] = cols[6];
    out[3] = cols[1];
    out[4] = cols[4];
    out[5] = cols[7];
    out[6] = cols[2];
    out[7] = cols[5];
    out[8] = cols[8];
    clear_error();
    Bool::TRUE
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tetra() -> FemTetrahedron {
        FemTetrahedron {
            a: Vec3::default(),
            b: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            c: Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            d: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        }
    }

    fn material() -> MaterialProperties {
        MaterialProperties {
            youngs_modulus: 1000.0,
            poisson_ratio: 0.25,
            ..MaterialProperties::default()
        }
    }

    #[test]
    fn tetra_shape_functions_and_volume_work() {
        assert!((continuum_tetra_volume(tetra()) - 1.0 / 6.0).abs() < 1.0e-12);
        let mut report = FemShapeFunctionReport::default();
        assert_eq!(
            continuum_tetra_shape_functions(
                tetra(),
                Vec3 {
                    x: 0.25,
                    y: 0.25,
                    z: 0.25
                },
                &mut report
            ),
            Bool::TRUE
        );
        assert_eq!(report.inside, Bool::TRUE);
        assert!(
            report
                .weights
                .iter()
                .all(|weight| (*weight - 0.25).abs() < 1.0e-12)
        );
    }

    #[test]
    fn constitutive_and_tetra_stiffness_work() {
        let mut d = [0.0; 36];
        let mut report = FemConstitutiveReport::default();
        assert_eq!(
            continuum_linear_elastic_constitutive_matrix(
                material(),
                d.as_mut_ptr(),
                d.len() as u32,
                &mut report
            ),
            Bool::TRUE
        );
        assert!(d[0] > d[3 * 6 + 3]);
        assert!(report.shear_modulus > 0.0);

        let mut k = [0.0; 144];
        let mut volume = 0.0;
        assert_eq!(
            continuum_linear_tetra_element_stiffness(
                tetra(),
                material(),
                k.as_mut_ptr(),
                k.len() as u32,
                &mut volume
            ),
            Bool::TRUE
        );
        assert!(volume > 0.0);
        assert!(k[0] > 0.0);
    }

    #[test]
    fn newmark_beta_solve_advances_sdof() {
        let m = [1.0];
        let c = [0.0];
        let k = [4.0];
        let u = [0.0];
        let v = [0.0];
        let a = [0.0];
        let f = [1.0];
        let mut du = [0.0];
        let mut next_u = [0.0];
        let mut next_v = [0.0];
        let mut next_a = [0.0];
        let mut report = NewmarkBetaReport::default();
        assert_eq!(
            continuum_newmark_beta_solve(
                m.as_ptr(),
                c.as_ptr(),
                k.as_ptr(),
                u.as_ptr(),
                v.as_ptr(),
                a.as_ptr(),
                f.as_ptr(),
                1,
                NewmarkBetaParameters {
                    beta: 0.25,
                    gamma: 0.5,
                    dt: 0.1
                },
                du.as_mut_ptr(),
                next_u.as_mut_ptr(),
                next_v.as_mut_ptr(),
                next_a.as_mut_ptr(),
                1,
                &mut report
            ),
            Bool::TRUE
        );
        assert!(du[0] > 0.0);
        assert!(next_u[0] > 0.0);
        assert!(report.residual_norm < 1.0e-10);
    }
}
