use std::slice;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, DensityFieldStats, SimpMaterialReport, TopologyOptimizationParams,
    TopologyOptimizationReport,
};

use crate::rapier::math::{KahanSum, finite_non_negative, finite_positive};

const MAX_DENSITY_CELLS: u32 = 2_000_000;
const EPSILON: f64 = 1.0e-12;

fn params_valid(params: TopologyOptimizationParams) -> bool {
    params.volume_fraction.is_finite()
        && params.volume_fraction > 0.0
        && params.volume_fraction <= 1.0
        && finite_positive(params.penalization)
        && params.min_density.is_finite()
        && params.min_density >= 0.0
        && params.min_density < params.volume_fraction
        && finite_positive(params.move_limit)
        && finite_non_negative(params.filter_radius)
        && finite_non_negative(params.stiffness_min)
        && finite_positive(params.stiffness_solid)
        && params.stiffness_solid >= params.stiffness_min
}

fn density_valid(value: f64) -> bool {
    value.is_finite() && (0.0..=1.0).contains(&value)
}

fn density_stats(densities: &[f64], threshold: f64) -> DensityFieldStats {
    let mut stats = DensityFieldStats {
        cell_count: densities.len().min(u32::MAX as usize) as u32,
        min_density: f64::INFINITY,
        ..DensityFieldStats::default()
    };
    let mut avg_acc = KahanSum::default();
    for density in densities {
        avg_acc.add(*density);
        stats.min_density = f64::min(stats.min_density, *density);
        stats.max_density = f64::max(stats.max_density, *density);
        if *density >= threshold {
            stats.solid_count += 1;
        }
    }
    if !densities.is_empty() {
        stats.average_density = avg_acc.value() / densities.len() as f64;
    } else {
        stats.min_density = 0.0;
    }
    stats
}

fn apply_oc_update(
    density: f64,
    sensitivity: f64,
    lambda: f64,
    params: TopologyOptimizationParams,
) -> f64 {
    let scale = (-sensitivity / lambda.max(EPSILON)).max(0.0).sqrt();
    (density * scale)
        .clamp(density - params.move_limit, density + params.move_limit)
        .clamp(params.min_density, 1.0)
}

fn average_after_oc(
    densities: &[f64],
    sensitivities: &[f64],
    lambda: f64,
    params: TopologyOptimizationParams,
) -> f64 {
    densities
        .iter()
        .zip(sensitivities)
        .map(|(density, sensitivity)| apply_oc_update(*density, *sensitivity, lambda, params))
        .sum::<f64>()
        / densities.len() as f64
}

#[unsafe(no_mangle)]
pub extern "C" fn topology_simp_material(
    density: f64,
    params: TopologyOptimizationParams,
    out_report: *mut SimpMaterialReport,
) -> Bool {
    if !density_valid(density) || !params_valid(params) {
        set_error(ERR_INVALID_ARGUMENT, "invalid SIMP material parameters");
        return Bool::FALSE;
    }
    let physical_density = density.max(params.min_density);
    let stiffness = params.stiffness_min
        + physical_density.powf(params.penalization)
            * (params.stiffness_solid - params.stiffness_min);
    let derivative = params.penalization
        * physical_density.powf(params.penalization - 1.0)
        * (params.stiffness_solid - params.stiffness_min);
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "SIMP material output is null");
        return Bool::FALSE;
    };
    *out_report = SimpMaterialReport {
        density: physical_density,
        stiffness,
        stiffness_derivative: derivative,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn topology_simp_stiffness(
    density: f64,
    penalization: f64,
    stiffness_min: f64,
    stiffness_solid: f64,
) -> f64 {
    if !density_valid(density)
        || !finite_positive(penalization)
        || !finite_non_negative(stiffness_min)
        || !finite_positive(stiffness_solid)
        || stiffness_solid < stiffness_min
    {
        return f64::NAN;
    }
    stiffness_min + density.powf(penalization) * (stiffness_solid - stiffness_min)
}

#[unsafe(no_mangle)]
pub extern "C" fn topology_compliance_sensitivity(
    density: f64,
    element_energy: f64,
    params: TopologyOptimizationParams,
) -> f64 {
    if !density_valid(density) || !finite_non_negative(element_energy) || !params_valid(params) {
        return f64::NAN;
    }
    let physical_density = density.max(params.min_density);
    -params.penalization
        * physical_density.powf(params.penalization - 1.0)
        * (params.stiffness_solid - params.stiffness_min)
        * element_energy
}

#[unsafe(no_mangle)]
pub extern "C" fn topology_oc_update(
    densities: *const f64,
    sensitivities: *const f64,
    cell_count: u32,
    params: TopologyOptimizationParams,
    out_densities: *mut f64,
    capacity: u32,
    out_report: *mut TopologyOptimizationReport,
) -> Bool {
    if cell_count == 0 || cell_count > MAX_DENSITY_CELLS || capacity < cell_count {
        set_error(ERR_CAPACITY, "invalid topology OC capacity");
        return Bool::FALSE;
    }
    if densities.is_null() || sensitivities.is_null() || out_densities.is_null() {
        set_error(ERR_NULL_POINTER, "topology OC pointers are null");
        return Bool::FALSE;
    }
    if !params_valid(params) {
        set_error(ERR_INVALID_ARGUMENT, "invalid topology OC parameters");
        return Bool::FALSE;
    }
    let densities = unsafe { slice::from_raw_parts(densities, cell_count as usize) };
    let sensitivities = unsafe { slice::from_raw_parts(sensitivities, cell_count as usize) };
    let out = unsafe { slice::from_raw_parts_mut(out_densities, capacity as usize) };
    if densities.iter().any(|density| !density_valid(*density))
        || sensitivities
            .iter()
            .any(|sensitivity| !sensitivity.is_finite())
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid topology OC field values");
        return Bool::FALSE;
    }

    let mut lower = EPSILON;
    let mut upper = 1.0e12;
    for _ in 0..80 {
        let mid = 0.5 * (lower + upper);
        let average = average_after_oc(densities, sensitivities, mid, params);
        if average > params.volume_fraction {
            lower = mid;
        } else {
            upper = mid;
        }
    }

    let lambda = 0.5 * (lower + upper);
    let mut report = TopologyOptimizationReport {
        cell_count,
        min_density: f64::INFINITY,
        ..TopologyOptimizationReport::default()
    };
    let mut avg_acc = KahanSum::default();
    for (index, (density, sensitivity)) in densities.iter().zip(sensitivities).enumerate() {
        let next = apply_oc_update(*density, *sensitivity, lambda, params);
        out[index] = next;
        avg_acc.add(next);
        report.min_density = f64::min(report.min_density, next);
        report.max_density = f64::max(report.max_density, next);
        report.max_density_change = f64::max(report.max_density_change, (next - density).abs());
    }
    report.average_density = avg_acc.value() / cell_count as f64;
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = report;
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn topology_density_filter_2d(
    densities: *const f64,
    width: u32,
    height: u32,
    filter_radius: f64,
    out_densities: *mut f64,
    capacity: u32,
) -> Bool {
    let Some(cell_count) = width.checked_mul(height) else {
        set_error(ERR_CAPACITY, "density filter size overflow");
        return Bool::FALSE;
    };
    if width == 0 || height == 0 || cell_count > MAX_DENSITY_CELLS || capacity < cell_count {
        set_error(ERR_CAPACITY, "invalid density filter capacity");
        return Bool::FALSE;
    }
    if densities.is_null() || out_densities.is_null() {
        set_error(ERR_NULL_POINTER, "density filter pointers are null");
        return Bool::FALSE;
    }
    if !finite_non_negative(filter_radius) {
        set_error(ERR_INVALID_ARGUMENT, "invalid density filter radius");
        return Bool::FALSE;
    }
    let width = width as usize;
    let height = height as usize;
    let densities = unsafe { slice::from_raw_parts(densities, cell_count as usize) };
    let out = unsafe { slice::from_raw_parts_mut(out_densities, capacity as usize) };
    if densities.iter().any(|density| !density_valid(*density)) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "density filter contains invalid values",
        );
        return Bool::FALSE;
    }
    let radius = filter_radius.ceil() as isize;
    for y in 0..height {
        for x in 0..width {
            let mut weighted_sum = 0.0;
            let mut weight_total = 0.0;
            for oy in -radius..=radius {
                for ox in -radius..=radius {
                    let nx = x as isize + ox;
                    let ny = y as isize + oy;
                    if nx < 0 || ny < 0 || nx >= width as isize || ny >= height as isize {
                        continue;
                    }
                    let distance = ((ox * ox + oy * oy) as f64).sqrt();
                    let weight = if filter_radius > 0.0 {
                        (filter_radius - distance).max(0.0)
                    } else if ox == 0 && oy == 0 {
                        1.0
                    } else {
                        0.0
                    };
                    if weight <= 0.0 {
                        continue;
                    }
                    weighted_sum += weight * densities[ny as usize * width + nx as usize];
                    weight_total += weight;
                }
            }
            out[y * width + x] = if weight_total > 0.0 {
                weighted_sum / weight_total
            } else {
                densities[y * width + x]
            };
        }
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn topology_density_to_voxels(
    densities: *const f64,
    cell_count: u32,
    threshold: f64,
    out_voxels: *mut u8,
    capacity: u32,
    out_stats: *mut DensityFieldStats,
) -> Bool {
    if cell_count == 0 || cell_count > MAX_DENSITY_CELLS || capacity < cell_count {
        set_error(ERR_CAPACITY, "invalid density voxel capacity");
        return Bool::FALSE;
    }
    if densities.is_null() || out_voxels.is_null() {
        set_error(ERR_NULL_POINTER, "density voxel pointers are null");
        return Bool::FALSE;
    }
    if !density_valid(threshold) {
        set_error(ERR_INVALID_ARGUMENT, "invalid density voxel threshold");
        return Bool::FALSE;
    }
    let densities = unsafe { slice::from_raw_parts(densities, cell_count as usize) };
    let voxels = unsafe { slice::from_raw_parts_mut(out_voxels, capacity as usize) };
    if densities.iter().any(|density| !density_valid(*density)) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "density voxel field contains invalid values",
        );
        return Bool::FALSE;
    }
    for (index, density) in densities.iter().enumerate() {
        voxels[index] = u8::from(*density >= threshold);
    }
    if let Some(out_stats) = unsafe { out_stats.as_mut() } {
        *out_stats = density_stats(densities, threshold);
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn topology_runtime_shape_density_step(
    densities: *const f64,
    element_energies: *const f64,
    cell_count: u32,
    params: TopologyOptimizationParams,
    out_densities: *mut f64,
    capacity: u32,
    out_report: *mut TopologyOptimizationReport,
) -> Bool {
    if cell_count == 0 || cell_count > MAX_DENSITY_CELLS || capacity < cell_count {
        set_error(ERR_CAPACITY, "invalid runtime topology capacity");
        return Bool::FALSE;
    }
    if densities.is_null() || element_energies.is_null() {
        set_error(ERR_NULL_POINTER, "runtime topology pointers are null");
        return Bool::FALSE;
    }
    if !params_valid(params) {
        set_error(ERR_INVALID_ARGUMENT, "invalid runtime topology parameters");
        return Bool::FALSE;
    }
    let densities_slice = unsafe { slice::from_raw_parts(densities, cell_count as usize) };
    let energies = unsafe { slice::from_raw_parts(element_energies, cell_count as usize) };
    if energies.iter().any(|energy| !finite_non_negative(*energy)) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "runtime topology energies are invalid",
        );
        return Bool::FALSE;
    }
    let sensitivities = densities_slice
        .iter()
        .zip(energies)
        .map(|(density, energy)| topology_compliance_sensitivity(*density, *energy, params))
        .collect::<Vec<_>>();
    if topology_oc_update(
        densities,
        sensitivities.as_ptr(),
        cell_count,
        params,
        out_densities,
        capacity,
        out_report,
    ) != Bool::TRUE
    {
        return Bool::FALSE;
    }
    if let Some(report) = unsafe { out_report.as_mut() } {
        let mut compliance_acc = KahanSum::default();
        for (density, energy) in densities_slice.iter().zip(energies) {
            compliance_acc.add(
                topology_simp_stiffness(
                    density.max(params.min_density),
                    params.penalization,
                    params.stiffness_min,
                    params.stiffness_solid,
                ) * *energy,
            );
        }
        report.total_compliance = compliance_acc.value();
    }
    clear_error();
    Bool::TRUE
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> TopologyOptimizationParams {
        TopologyOptimizationParams {
            volume_fraction: 0.5,
            penalization: 3.0,
            min_density: 0.001,
            move_limit: 0.2,
            filter_radius: 1.5,
            stiffness_min: 1.0e-6,
            stiffness_solid: 1.0,
        }
    }

    #[test]
    fn simp_material_and_sensitivity_work() {
        let mut report = SimpMaterialReport::default();
        assert_eq!(
            topology_simp_material(0.5, params(), &mut report),
            Bool::TRUE
        );
        assert!(report.stiffness > 0.0);
        assert!(report.stiffness_derivative > 0.0);
        let sensitivity = topology_compliance_sensitivity(0.5, 2.0, params());
        assert!(sensitivity < 0.0);
    }

    #[test]
    fn oc_update_preserves_volume_fraction() {
        let densities = [0.5, 0.5, 0.5, 0.5];
        let sensitivities = [-4.0, -1.0, -1.0, -4.0];
        let mut out = [0.0; 4];
        let mut report = TopologyOptimizationReport::default();
        assert_eq!(
            topology_oc_update(
                densities.as_ptr(),
                sensitivities.as_ptr(),
                4,
                params(),
                out.as_mut_ptr(),
                4,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!((report.average_density - 0.5).abs() < 1.0e-3);
        assert!(out[0] > out[1]);
    }

    #[test]
    fn density_filter_and_voxelization_work() {
        let densities = [1.0, 0.0, 0.0, 1.0];
        let mut filtered = [0.0; 4];
        assert_eq!(
            topology_density_filter_2d(densities.as_ptr(), 2, 2, 1.5, filtered.as_mut_ptr(), 4),
            Bool::TRUE
        );
        assert!(filtered[0] < 1.0);
        assert!(filtered[1] > 0.0);

        let mut voxels = [0u8; 4];
        let mut stats = DensityFieldStats::default();
        assert_eq!(
            topology_density_to_voxels(
                filtered.as_ptr(),
                4,
                0.5,
                voxels.as_mut_ptr(),
                4,
                &mut stats,
            ),
            Bool::TRUE
        );
        assert_eq!(stats.cell_count, 4);
    }
}
