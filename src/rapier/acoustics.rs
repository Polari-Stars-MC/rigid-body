use std::slice;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ERR_UNSUPPORTED, clear_error, set_error,
};
use crate::rapier::ffi::{
    AcousticContactDesc, AcousticExcitationReport, AcousticMaterial, AcousticResonanceReport,
    AcousticWaveReport, Bool, ModalAnalysisReport, ModalSynthesisReport, SpatializedSample,
    StructuralModeReport, Vec3, clamp01, finite_non_negative, finite_positive,
};
use crate::rapier::math::KahanSum;

const EPSILON: f64 = 1.0e-12;
const MAX_MODAL_DOF: u32 = 128;
const MAX_WAVE_CELLS: u32 = 2_000_000;
const MAX_AUDIO_MODES: u32 = 512;

fn finite_vec3(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

fn vec3_sub(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.x - b.x,
        y: a.y - b.y,
        z: a.z - b.z,
    }
}

fn vec3_dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn vec3_norm(value: Vec3) -> f64 {
    vec3_dot(value, value).sqrt()
}

fn vec3_normalize(value: Vec3) -> Vec3 {
    let length = vec3_norm(value);
    if length <= EPSILON {
        Vec3::default()
    } else {
        Vec3 {
            x: value.x / length,
            y: value.y / length,
            z: value.z / length,
        }
    }
}

fn material_valid(material: AcousticMaterial) -> bool {
    finite_positive(material.density)
        && finite_non_negative(material.hardness)
        && finite_non_negative(material.damping)
        && finite_non_negative(material.roughness)
        && finite_non_negative(material.restitution)
        && finite_positive(material.sound_speed)
}

fn matrix_index(row: usize, col: usize, n: usize) -> usize {
    row * n + col
}

fn cholesky_decompose(matrix: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut lower = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut sum = matrix[matrix_index(i, j, n)];
            for k in 0..j {
                sum -= lower[matrix_index(i, k, n)] * lower[matrix_index(j, k, n)];
            }
            if i == j {
                if sum <= EPSILON || !sum.is_finite() {
                    return None;
                }
                lower[matrix_index(i, j, n)] = sum.sqrt();
            } else {
                lower[matrix_index(i, j, n)] = sum / lower[matrix_index(j, j, n)];
            }
        }
    }
    Some(lower)
}

fn solve_lower(lower: &[f64], rhs: &[f64], n: usize) -> Vec<f64> {
    let mut out = vec![0.0; n];
    for i in 0..n {
        let mut sum = rhs[i];
        for k in 0..i {
            sum -= lower[matrix_index(i, k, n)] * out[k];
        }
        out[i] = sum / lower[matrix_index(i, i, n)];
    }
    out
}

fn solve_upper_from_lower(lower: &[f64], rhs: &[f64], n: usize) -> Vec<f64> {
    let mut out = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = rhs[i];
        for k in i + 1..n {
            sum -= lower[matrix_index(k, i, n)] * out[k];
        }
        out[i] = sum / lower[matrix_index(i, i, n)];
    }
    out
}

fn transform_generalized(stiffness: &[f64], mass_lower: &[f64], n: usize) -> Vec<f64> {
    let mut temp = vec![0.0; n * n];
    for col in 0..n {
        let rhs = (0..n)
            .map(|row| stiffness[matrix_index(row, col, n)])
            .collect::<Vec<_>>();
        let solved = solve_lower(mass_lower, &rhs, n);
        for row in 0..n {
            temp[matrix_index(row, col, n)] = solved[row];
        }
    }

    let mut transformed = vec![0.0; n * n];
    for row in 0..n {
        let rhs = (0..n)
            .map(|col| temp[matrix_index(row, col, n)])
            .collect::<Vec<_>>();
        let solved = solve_lower(mass_lower, &rhs, n);
        for col in 0..n {
            transformed[matrix_index(row, col, n)] = solved[col];
        }
    }

    for row in 0..n {
        for col in row + 1..n {
            let average = 0.5
                * (transformed[matrix_index(row, col, n)] + transformed[matrix_index(col, row, n)]);
            transformed[matrix_index(row, col, n)] = average;
            transformed[matrix_index(col, row, n)] = average;
        }
    }
    transformed
}

fn jacobi_eigen_symmetric(mut matrix: Vec<f64>, n: usize) -> Option<(Vec<f64>, Vec<f64>)> {
    if n > 200 {
        // Jacobi O(n⁴) becomes prohibitive beyond 200 DOF.
        // Caller should handle this fallback (e.g. use a different solver).
        return None;
    }
    let mut vectors = vec![0.0; n * n];
    for i in 0..n {
        vectors[matrix_index(i, i, n)] = 1.0;
    }

    for _ in 0..(64 * n * n).max(1) {
        let mut pivot_row = 0;
        let mut pivot_col = 1.min(n.saturating_sub(1));
        let mut max_offdiag = 0.0;
        for row in 0..n {
            for col in row + 1..n {
                let value = matrix[matrix_index(row, col, n)].abs();
                if value > max_offdiag {
                    max_offdiag = value;
                    pivot_row = row;
                    pivot_col = col;
                }
            }
        }
        if max_offdiag <= 1.0e-10 {
            break;
        }

        let app = matrix[matrix_index(pivot_row, pivot_row, n)];
        let aqq = matrix[matrix_index(pivot_col, pivot_col, n)];
        let apq = matrix[matrix_index(pivot_row, pivot_col, n)];
        let tau = (aqq - app) / (2.0 * apq);
        let tangent = if tau >= 0.0 {
            1.0 / (tau + (1.0 + tau * tau).sqrt())
        } else {
            -1.0 / (-tau + (1.0 + tau * tau).sqrt())
        };
        let cosine = 1.0 / (1.0 + tangent * tangent).sqrt();
        let sine = tangent * cosine;

        for k in 0..n {
            if k != pivot_row && k != pivot_col {
                let akp = matrix[matrix_index(k, pivot_row, n)];
                let akq = matrix[matrix_index(k, pivot_col, n)];
                let new_kp = cosine * akp - sine * akq;
                let new_kq = sine * akp + cosine * akq;
                matrix[matrix_index(k, pivot_row, n)] = new_kp;
                matrix[matrix_index(pivot_row, k, n)] = new_kp;
                matrix[matrix_index(k, pivot_col, n)] = new_kq;
                matrix[matrix_index(pivot_col, k, n)] = new_kq;
            }
        }
        matrix[matrix_index(pivot_row, pivot_row, n)] =
            cosine * cosine * app - 2.0 * sine * cosine * apq + sine * sine * aqq;
        matrix[matrix_index(pivot_col, pivot_col, n)] =
            sine * sine * app + 2.0 * sine * cosine * apq + cosine * cosine * aqq;
        matrix[matrix_index(pivot_row, pivot_col, n)] = 0.0;
        matrix[matrix_index(pivot_col, pivot_row, n)] = 0.0;

        for k in 0..n {
            let vkp = vectors[matrix_index(k, pivot_row, n)];
            let vkq = vectors[matrix_index(k, pivot_col, n)];
            vectors[matrix_index(k, pivot_row, n)] = cosine * vkp - sine * vkq;
            vectors[matrix_index(k, pivot_col, n)] = sine * vkp + cosine * vkq;
        }
    }

    let values = (0..n)
        .map(|i| matrix[matrix_index(i, i, n)])
        .collect::<Vec<_>>();
    Some((values, vectors))
}

#[unsafe(no_mangle)]
pub extern "C" fn acoustic_generalized_modal_analysis(
    stiffness_matrix: *const f64,
    mass_matrix: *const f64,
    dof: u32,
    requested_modes: u32,
    out_eigenvalues: *mut f64,
    out_frequencies_hz: *mut f64,
    out_mode_shapes: *mut f64,
    eigen_capacity: u32,
    mode_shape_capacity: u32,
    out_report: *mut ModalAnalysisReport,
) -> Bool {
    if dof == 0 || dof > MAX_MODAL_DOF {
        set_error(
            ERR_CAPACITY,
            "invalid modal analysis degree-of-freedom count",
        );
        return Bool::FALSE;
    }
    let mode_count = requested_modes.min(dof);
    if mode_count == 0 || eigen_capacity < mode_count || mode_shape_capacity < mode_count * dof {
        set_error(ERR_CAPACITY, "invalid modal analysis output capacity");
        return Bool::FALSE;
    }
    if stiffness_matrix.is_null()
        || mass_matrix.is_null()
        || out_eigenvalues.is_null()
        || out_frequencies_hz.is_null()
        || out_mode_shapes.is_null()
    {
        set_error(ERR_NULL_POINTER, "modal analysis pointers are null");
        return Bool::FALSE;
    }

    let n = dof as usize;
    let stiffness = unsafe { slice::from_raw_parts(stiffness_matrix, n * n) };
    let mass = unsafe { slice::from_raw_parts(mass_matrix, n * n) };
    if stiffness.iter().chain(mass).any(|value| !value.is_finite()) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "modal analysis matrices contain non-finite values",
        );
        return Bool::FALSE;
    }
    let Some(mass_lower) = cholesky_decompose(mass, n) else {
        set_error(ERR_INVALID_ARGUMENT, "mass matrix is not positive definite");
        return Bool::FALSE;
    };
    let transformed = transform_generalized(stiffness, &mass_lower, n);
    let Some((eigenvalues, modal_vectors)) = jacobi_eigen_symmetric(transformed, n) else {
        set_error(
            ERR_UNSUPPORTED,
            "Jacobi solver does not support n > 200; use a different eigensolver");
        return Bool::FALSE;
    };
    let mut order = (0..n).collect::<Vec<_>>();
    order.sort_by(|&a, &b| eigenvalues[a].total_cmp(&eigenvalues[b]));

    let out_eigenvalues =
        unsafe { slice::from_raw_parts_mut(out_eigenvalues, eigen_capacity as usize) };
    let out_frequencies =
        unsafe { slice::from_raw_parts_mut(out_frequencies_hz, eigen_capacity as usize) };
    let out_modes =
        unsafe { slice::from_raw_parts_mut(out_mode_shapes, mode_shape_capacity as usize) };

    let mut stable_modes = 0;
    let mut max_frequency_hz = 0.0;
    for mode in 0..mode_count as usize {
        let source = order[mode];
        let eigenvalue = eigenvalues[source].max(0.0);
        let frequency = eigenvalue.sqrt() / (2.0 * std::f64::consts::PI);
        out_eigenvalues[mode] = eigenvalue;
        out_frequencies[mode] = frequency;
        max_frequency_hz = f64::max(max_frequency_hz, frequency);
        if eigenvalues[source] >= 0.0 {
            stable_modes += 1;
        }

        let y = (0..n)
            .map(|row| modal_vectors[matrix_index(row, source, n)])
            .collect::<Vec<_>>();
        let x = solve_upper_from_lower(&mass_lower, &y, n);
        for row in 0..n {
            out_modes[mode * n + row] = x[row];
        }
    }

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = ModalAnalysisReport {
            dof,
            mode_count,
            stable_mode_count: stable_modes,
            max_frequency_hz,
        };
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn acoustic_structural_mode_sdof(
    stiffness: f64,
    mass: f64,
    damping: f64,
    out_report: *mut StructuralModeReport,
) -> Bool {
    if !finite_non_negative(stiffness) || !finite_positive(mass) || !finite_non_negative(damping) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid single-DOF structural mode parameters",
        );
        return Bool::FALSE;
    }
    let angular_frequency = (stiffness / mass).sqrt();
    let frequency_hz = angular_frequency / (2.0 * std::f64::consts::PI);
    let critical_damping = 2.0 * (stiffness * mass).sqrt();
    let damping_ratio = if critical_damping > EPSILON {
        damping / critical_damping
    } else {
        0.0
    };
    let damped_frequency_hz = if damping_ratio < 1.0 {
        frequency_hz * (1.0 - damping_ratio * damping_ratio).sqrt()
    } else {
        0.0
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "structural mode output is null");
        return Bool::FALSE;
    };
    *out_report = StructuralModeReport {
        angular_frequency,
        frequency_hz,
        damping_ratio,
        damped_frequency_hz,
        critical_damping,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn acoustic_wave_equation_step(
    previous_pressure: *const f64,
    current_pressure: *const f64,
    laplacian_pressure: *const f64,
    cell_count: u32,
    sound_speed: f64,
    damping: f64,
    dt: f64,
    out_next_pressure: *mut f64,
    capacity: u32,
    out_report: *mut AcousticWaveReport,
) -> Bool {
    if cell_count == 0 || cell_count > MAX_WAVE_CELLS || capacity < cell_count {
        set_error(ERR_CAPACITY, "invalid acoustic wave capacity");
        return Bool::FALSE;
    }
    if previous_pressure.is_null()
        || current_pressure.is_null()
        || laplacian_pressure.is_null()
        || out_next_pressure.is_null()
    {
        set_error(ERR_NULL_POINTER, "acoustic wave pointers are null");
        return Bool::FALSE;
    }
    if !finite_positive(sound_speed) || !finite_non_negative(damping) || !finite_non_negative(dt) {
        set_error(ERR_INVALID_ARGUMENT, "invalid acoustic wave parameters");
        return Bool::FALSE;
    }

    let count = cell_count as usize;
    let previous = unsafe { slice::from_raw_parts(previous_pressure, count) };
    let current = unsafe { slice::from_raw_parts(current_pressure, count) };
    let laplacian = unsafe { slice::from_raw_parts(laplacian_pressure, count) };
    let next = unsafe { slice::from_raw_parts_mut(out_next_pressure, capacity as usize) };
    let mut max_pressure = 0.0;
    let mut acoustic_energy_acc = KahanSum::default();
    for index in 0..count {
        if !previous[index].is_finite()
            || !current[index].is_finite()
            || !laplacian[index].is_finite()
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "acoustic wave cell contains non-finite values",
            );
            return Bool::FALSE;
        }
        let velocity_term = (current[index] - previous[index]) * (1.0 - damping * dt);
        next[index] =
            current[index] + velocity_term + sound_speed * sound_speed * dt * dt * laplacian[index];
        max_pressure = f64::max(max_pressure, next[index].abs());
        acoustic_energy_acc.add(0.5 * next[index] * next[index]);
    }

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = AcousticWaveReport {
            cell_count,
            max_pressure,
            acoustic_energy: acoustic_energy_acc.value(),
        };
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn acoustic_detect_resonance(
    excitation_frequency_hz: f64,
    modal_frequencies_hz: *const f64,
    damping_ratios: *const f64,
    mode_count: u32,
    tolerance_hz: f64,
    out_report: *mut AcousticResonanceReport,
) -> Bool {
    if mode_count == 0 || mode_count > MAX_MODAL_DOF {
        set_error(ERR_CAPACITY, "invalid resonance mode count");
        return Bool::FALSE;
    }
    if modal_frequencies_hz.is_null() {
        set_error(ERR_NULL_POINTER, "modal frequency pointer is null");
        return Bool::FALSE;
    }
    if !finite_non_negative(excitation_frequency_hz) || !finite_non_negative(tolerance_hz) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid resonance detection parameters",
        );
        return Bool::FALSE;
    }
    let modes = unsafe { slice::from_raw_parts(modal_frequencies_hz, mode_count as usize) };
    let damping = if damping_ratios.is_null() {
        None
    } else {
        Some(unsafe { slice::from_raw_parts(damping_ratios, mode_count as usize) })
    };
    let mut nearest_index = 0;
    let mut nearest_frequency_hz = 0.0;
    let mut frequency_delta_hz = f64::INFINITY;
    let mut amplification_estimate = 0.0;
    for (index, &frequency) in modes.iter().enumerate() {
        if !finite_non_negative(frequency) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "modal frequency contains non-finite values",
            );
            return Bool::FALSE;
        }
        let delta = (frequency - excitation_frequency_hz).abs();
        if delta < frequency_delta_hz {
            nearest_index = index as u32;
            nearest_frequency_hz = frequency;
            frequency_delta_hz = delta;
            let ratio = if frequency > EPSILON {
                excitation_frequency_hz / frequency
            } else {
                0.0
            };
            let zeta = damping
                .map(|values| values[index])
                .filter(|value| value.is_finite() && *value >= 0.0)
                .unwrap_or(0.0);
            amplification_estimate = 1.0
                / ((1.0 - ratio * ratio).powi(2) + (2.0 * zeta * ratio).powi(2))
                    .sqrt()
                    .max(EPSILON);
        }
    }
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "resonance output is null");
        return Bool::FALSE;
    };
    *out_report = AcousticResonanceReport {
        resonant: Bool::from(frequency_delta_hz <= tolerance_hz),
        nearest_mode_index: nearest_index,
        nearest_frequency_hz,
        frequency_delta_hz,
        amplification_estimate,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn acoustic_contact_material_excitation(
    material_a: AcousticMaterial,
    material_b: AcousticMaterial,
    contact: AcousticContactDesc,
    out_report: *mut AcousticExcitationReport,
) -> Bool {
    if !material_valid(material_a) || !material_valid(material_b) {
        set_error(ERR_INVALID_ARGUMENT, "invalid acoustic material parameters");
        return Bool::FALSE;
    }
    if !finite_non_negative(contact.normal_force)
        || !contact.normal_velocity.is_finite()
        || !contact.tangential_velocity.is_finite()
        || !finite_non_negative(contact.contact_area)
        || !finite_positive(contact.dt)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid acoustic contact parameters");
        return Bool::FALSE;
    }
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "acoustic excitation output is null");
        return Bool::FALSE;
    };

    let effective_hardness = 2.0 * material_a.hardness * material_b.hardness
        / (material_a.hardness + material_b.hardness).max(EPSILON);
    let effective_density = 0.5 * (material_a.density + material_b.density);
    let restitution = clamp01(0.5 * (material_a.restitution + material_b.restitution));
    let damping = clamp01(0.5 * (material_a.damping + material_b.damping));
    let roughness = clamp01(0.5 * (material_a.roughness + material_b.roughness));
    let acoustic_impedance =
        effective_density * 0.5 * (material_a.sound_speed + material_b.sound_speed);

    let normal_speed = contact.normal_velocity.abs();
    let tangential_speed = contact.tangential_velocity.abs();
    let impulse = contact.normal_force * contact.dt * (1.0 + restitution);
    let contact_scale = (contact.contact_area + 1.0e-6).sqrt();
    let hardness_scale = (effective_hardness / (effective_hardness + acoustic_impedance)).sqrt();
    let normal_component = impulse * normal_speed * hardness_scale / contact_scale.max(EPSILON);
    let scrape_component = contact.normal_force * tangential_speed * roughness * contact.dt
        / contact_scale.max(EPSILON);
    let brightness = clamp01(
        hardness_scale * (1.0 - 0.5 * damping)
            + roughness * tangential_speed / (normal_speed + tangential_speed + EPSILON),
    );
    let amplitude = (normal_component + scrape_component) * (1.0 - 0.75 * damping).max(0.0);

    *out_report = AcousticExcitationReport {
        impulse,
        normal_component,
        scrape_component,
        brightness,
        damping,
        amplitude,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn acoustic_modal_synthesis_step(
    modal_frequencies_hz: *const f64,
    damping_ratios: *const f64,
    modal_gains: *const f64,
    mode_displacements: *mut f64,
    mode_velocities: *mut f64,
    mode_count: u32,
    excitation: f64,
    dt: f64,
    output_gain: f64,
    out_report: *mut ModalSynthesisReport,
) -> Bool {
    if mode_count == 0 || mode_count > MAX_AUDIO_MODES {
        set_error(ERR_CAPACITY, "invalid modal synthesis mode count");
        return Bool::FALSE;
    }
    if modal_frequencies_hz.is_null()
        || modal_gains.is_null()
        || mode_displacements.is_null()
        || mode_velocities.is_null()
    {
        set_error(ERR_NULL_POINTER, "modal synthesis pointers are null");
        return Bool::FALSE;
    }
    if !excitation.is_finite() || !finite_positive(dt) || !output_gain.is_finite() {
        set_error(ERR_INVALID_ARGUMENT, "invalid modal synthesis parameters");
        return Bool::FALSE;
    }

    let count = mode_count as usize;
    let frequencies = unsafe { slice::from_raw_parts(modal_frequencies_hz, count) };
    let damping = if damping_ratios.is_null() {
        None
    } else {
        Some(unsafe { slice::from_raw_parts(damping_ratios, count) })
    };
    let gains = unsafe { slice::from_raw_parts(modal_gains, count) };
    let displacements = unsafe { slice::from_raw_parts_mut(mode_displacements, count) };
    let velocities = unsafe { slice::from_raw_parts_mut(mode_velocities, count) };

    let mut sample_acc = KahanSum::default();
    let mut peak_modal_displacement: f64 = 0.0;
    let mut modal_energy_acc = KahanSum::default();
    for index in 0..count {
        let frequency = frequencies[index];
        let gain = gains[index];
        let zeta = damping
            .map(|values| values[index])
            .filter(|value| value.is_finite())
            .unwrap_or(0.02)
            .max(0.0);
        if !finite_non_negative(frequency)
            || !gain.is_finite()
            || !displacements[index].is_finite()
            || !velocities[index].is_finite()
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "modal synthesis buffer contains non-finite values",
            );
            return Bool::FALSE;
        }

        let omega = 2.0 * std::f64::consts::PI * frequency;
        let acceleration = gain * excitation
            - 2.0 * zeta * omega * velocities[index]
            - omega * omega * displacements[index];
        velocities[index] += acceleration * dt;
        displacements[index] += velocities[index] * dt;
        if frequency <= EPSILON {
            velocities[index] *= (1.0 - zeta * dt).max(0.0);
        }

        sample_acc.add(gain * displacements[index]);
        peak_modal_displacement = peak_modal_displacement.max(displacements[index].abs());
        modal_energy_acc.add(
            0.5 * (velocities[index] * velocities[index]
                + omega * omega * displacements[index] * displacements[index]),
        );
    }

    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "modal synthesis output is null");
        return Bool::FALSE;
    };
    *out_report = ModalSynthesisReport {
        mode_count,
        sample: sample_acc.value() * output_gain,
        peak_modal_displacement,
        modal_energy: modal_energy_acc.value(),
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn acoustic_spatialize_mono_sample(
    mono_sample: f64,
    source_position: Vec3,
    listener_position: Vec3,
    listener_right: Vec3,
    reference_distance: f64,
    rolloff: f64,
    out_sample: *mut SpatializedSample,
) -> Bool {
    if !mono_sample.is_finite()
        || !finite_vec3(source_position)
        || !finite_vec3(listener_position)
        || !finite_vec3(listener_right)
        || !finite_positive(reference_distance)
        || !finite_non_negative(rolloff)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid spatialization parameters");
        return Bool::FALSE;
    }
    let Some(out_sample) = (unsafe { out_sample.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "spatialized sample output is null");
        return Bool::FALSE;
    };

    let offset = vec3_sub(source_position, listener_position);
    let distance = vec3_norm(offset);
    let direction = vec3_normalize(offset);
    let right = vec3_normalize(listener_right);
    let pan = clamp01(0.5 + 0.5 * vec3_dot(direction, right));
    let attenuation = reference_distance
        / (reference_distance + rolloff * (distance - reference_distance).max(0.0));
    let equal_power_left = (pan * std::f64::consts::FRAC_PI_2).cos();
    let equal_power_right = (pan * std::f64::consts::FRAC_PI_2).sin();
    let scaled = mono_sample * attenuation;

    *out_sample = SpatializedSample {
        left: scaled * equal_power_left,
        right: scaled * equal_power_right,
        distance,
        attenuation,
        pan,
    };
    clear_error();
    Bool::TRUE
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modal_analysis_solves_generalized_eigenproblem() {
        let stiffness = [4.0, 0.0, 0.0, 9.0];
        let mass = [1.0, 0.0, 0.0, 1.0];
        let mut eigenvalues = [0.0; 2];
        let mut frequencies = [0.0; 2];
        let mut modes = [0.0; 4];
        let mut report = ModalAnalysisReport::default();
        assert_eq!(
            acoustic_generalized_modal_analysis(
                stiffness.as_ptr(),
                mass.as_ptr(),
                2,
                2,
                eigenvalues.as_mut_ptr(),
                frequencies.as_mut_ptr(),
                modes.as_mut_ptr(),
                2,
                4,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!((eigenvalues[0] - 4.0).abs() < 1.0e-8);
        assert!((eigenvalues[1] - 9.0).abs() < 1.0e-8);
        assert_eq!(report.mode_count, 2);
    }

    #[test]
    fn wave_step_and_resonance_detection_work() {
        let previous = [0.0, 0.0, 0.0];
        let current = [0.0, 1.0, 0.0];
        let laplacian = [1.0, -2.0, 1.0];
        let mut next = [0.0; 3];
        let mut wave = AcousticWaveReport::default();
        assert_eq!(
            acoustic_wave_equation_step(
                previous.as_ptr(),
                current.as_ptr(),
                laplacian.as_ptr(),
                3,
                1.0,
                0.0,
                0.1,
                next.as_mut_ptr(),
                3,
                &mut wave,
            ),
            Bool::TRUE
        );
        assert!(next[1] < 2.0);
        assert!(wave.acoustic_energy > 0.0);

        let modal = [100.0, 250.0, 500.0];
        let damping = [0.02, 0.05, 0.1];
        let mut resonance = AcousticResonanceReport::default();
        assert_eq!(
            acoustic_detect_resonance(
                248.0,
                modal.as_ptr(),
                damping.as_ptr(),
                3,
                5.0,
                &mut resonance
            ),
            Bool::TRUE
        );
        assert_eq!(resonance.resonant, Bool::TRUE);
        assert_eq!(resonance.nearest_mode_index, 1);
    }

    #[test]
    fn contact_excitation_feeds_modal_synthesis_and_spatialization() {
        let metal = AcousticMaterial {
            density: 7_800.0,
            hardness: 2.0e11,
            damping: 0.03,
            roughness: 0.25,
            restitution: 0.55,
            sound_speed: 5_000.0,
        };
        let wood = AcousticMaterial {
            density: 700.0,
            hardness: 1.0e10,
            damping: 0.12,
            roughness: 0.55,
            restitution: 0.35,
            sound_speed: 3_300.0,
        };
        let contact = AcousticContactDesc {
            normal_force: 120.0,
            normal_velocity: -2.0,
            tangential_velocity: 0.4,
            contact_area: 0.002,
            dt: 1.0 / 60.0,
        };
        let mut excitation = AcousticExcitationReport::default();
        assert_eq!(
            acoustic_contact_material_excitation(metal, wood, contact, &mut excitation),
            Bool::TRUE
        );
        assert!(excitation.amplitude > 0.0);
        assert!(excitation.brightness >= 0.0 && excitation.brightness <= 1.0);

        let frequencies = [180.0, 520.0, 1_200.0];
        let damping = [0.02, 0.04, 0.08];
        let gains = [1.0, 0.45, 0.18];
        let mut displacements = [0.0; 3];
        let mut velocities = [0.0; 3];
        let mut modal = ModalSynthesisReport::default();
        assert_eq!(
            acoustic_modal_synthesis_step(
                frequencies.as_ptr(),
                damping.as_ptr(),
                gains.as_ptr(),
                displacements.as_mut_ptr(),
                velocities.as_mut_ptr(),
                3,
                excitation.amplitude,
                1.0 / 48_000.0,
                0.25,
                &mut modal,
            ),
            Bool::TRUE
        );
        assert_eq!(modal.mode_count, 3);
        assert!(modal.modal_energy > 0.0);

        let mut stereo = SpatializedSample::default();
        assert_eq!(
            acoustic_spatialize_mono_sample(
                modal.sample,
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3::default(),
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                1.0,
                1.0,
                &mut stereo,
            ),
            Bool::TRUE
        );
        assert!(stereo.right.abs() >= stereo.left.abs());
        assert!(stereo.attenuation > 0.0 && stereo.attenuation <= 1.0);
    }
}
