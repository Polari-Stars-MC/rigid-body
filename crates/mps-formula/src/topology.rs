use crate::error::{ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error};
use crate::ffi::{
    Bool, DensityFieldStats, SimpMaterialReport, TopologyOptimizationParams,
    TopologyOptimizationReport,
};

use crate::math::{EPS_GENERAL as EPSILON, KahanSum, finite_non_negative, finite_positive};

const MAX_DENSITY_CELLS: u32 = 2_000_000;

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

/// Computes the SIMP-interpolated stiffness and its density derivative for one cell.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `SimpMaterialReport`;
/// a null `out_report` fails with `ERR_NULL_POINTER`. `density` and `params`
/// are passed by value; invalid values fail with `ERR_INVALID_ARGUMENT`.
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

/// Returns the SIMP-interpolated stiffness for a single density value.
///
/// # Safety
///
/// This function takes no pointers and performs no memory access; all
/// parameters are passed by value. Invalid inputs yield `NaN`.
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

/// Returns the compliance sensitivity of one element for the OC update.
///
/// # Safety
///
/// This function takes no pointers and performs no memory access; all
/// parameters are passed by value. Invalid inputs yield `NaN`.
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

/// Performs one optimality-criteria (OC) density update over a density field.
///
/// # Safety
///
/// `densities` and `sensitivities` must each point to `cell_count` readable
/// `f64` elements; `out_densities` must point to writable memory for
/// `capacity` `f64` elements (`cell_count <= capacity`,
/// `0 < cell_count <= 2_000_000`). `out_report` may be null; when non-null it
/// must point to writable memory for one `TopologyOptimizationReport`. Null
/// required pointers fail with `ERR_NULL_POINTER`. No ownership is
/// transferred; the caller keeps ownership of all buffers.
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
    let densities = match unsafe {
        crate::ffi::checked_input_slice(densities, cell_count as usize, "densities")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
    let sensitivities = match unsafe {
        crate::ffi::checked_input_slice(sensitivities, cell_count as usize, "sensitivities")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
    let out = match unsafe {
        crate::ffi::checked_output_slice(out_densities, capacity as usize, "out_densities")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
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

/// Applies a radial weighted-average density filter to a 2D density grid.
///
/// # Safety
///
/// `densities` must point to `width * height` readable `f64` elements in
/// row-major order; `out_densities` must point to writable memory for
/// `capacity` `f64` elements (`width * height <= capacity`,
/// `0 < width * height <= 2_000_000`). Null pointers fail with
/// `ERR_NULL_POINTER`. No ownership is transferred.
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
    let densities = match unsafe {
        crate::ffi::checked_input_slice(densities, cell_count as usize, "densities")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
    let out = match unsafe {
        crate::ffi::checked_output_slice(out_densities, capacity as usize, "out_densities")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
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

/// Thresholds a density field into a binary (0/1) voxel grid.
///
/// # Safety
///
/// `densities` must point to `cell_count` readable `f64` elements;
/// `out_voxels` must point to writable memory for `capacity` `u8` elements
/// (`cell_count <= capacity`, `0 < cell_count <= 2_000_000`). `out_stats` may
/// be null; when non-null it must point to writable memory for one
/// `DensityFieldStats`. Null required pointers fail with `ERR_NULL_POINTER`.
/// No ownership is transferred.
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
    let densities = match unsafe {
        crate::ffi::checked_input_slice(densities, cell_count as usize, "densities")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
    let voxels = match unsafe {
        crate::ffi::checked_output_slice(out_voxels, capacity as usize, "out_voxels")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
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

/// Runs one runtime topology step: derives sensitivities from element
/// energies, then applies an OC density update.
///
/// # Safety
///
/// `densities` and `element_energies` must each point to `cell_count`
/// readable `f64` elements; `out_densities` must point to writable memory for
/// `capacity` `f64` elements (`cell_count <= capacity`,
/// `0 < cell_count <= 2_000_000`). `out_report` may be null; when non-null it
/// must point to writable memory for one `TopologyOptimizationReport`. Null
/// required pointers fail with `ERR_NULL_POINTER`. No ownership is
/// transferred; the caller keeps ownership of all buffers.
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
    let densities_slice = match unsafe {
        crate::ffi::checked_input_slice(densities, cell_count as usize, "densities")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
    let energies = match unsafe {
        crate::ffi::checked_input_slice(element_energies, cell_count as usize, "element_energies")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
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
