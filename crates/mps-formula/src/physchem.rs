use crate::error::{ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error};
use crate::ffi::{
    Bool, CatalystEffect, CatalystReport, ConcentrationBuoyancyReport, GrayScottParams,
    GrayScottReactionReport, ReactionDiffusionReport, Vec3, vec3_finite, vec3_from_rapier,
    vec3_to_rapier,
};

use crate::math::{KahanSum, finite_non_negative, finite_positive};

const MAX_GRID_CELLS: u32 = 2_000_000;

fn params_valid(params: GrayScottParams) -> bool {
    finite_non_negative(params.diffusion_u)
        && finite_non_negative(params.diffusion_v)
        && finite_non_negative(params.feed_rate)
        && finite_non_negative(params.kill_rate)
        && finite_positive(params.dx)
}

fn catalyst_multiplier(catalyst: CatalystEffect) -> Option<f64> {
    if !finite_non_negative(catalyst.concentration)
        || !finite_non_negative(catalyst.strength)
        || !finite_non_negative(catalyst.saturation)
    {
        return None;
    }
    let activity = if catalyst.saturation > 0.0 {
        catalyst.concentration / (catalyst.saturation + catalyst.concentration)
    } else {
        catalyst.concentration
    };
    Some(1.0 + catalyst.strength * activity)
}

fn laplacian_center(values: &[f64], width: usize, height: usize, x: usize, y: usize) -> f64 {
    let center = values[y * width + x];
    let left = values[y * width + if x == 0 { width - 1 } else { x - 1 }];
    let right = values[y * width + if x + 1 == width { 0 } else { x + 1 }];
    let up = values[if y == 0 {
        (height - 1) * width + x
    } else {
        (y - 1) * width + x
    }];
    let down = values[if y + 1 == height {
        x
    } else {
        (y + 1) * width + x
    }];
    left + right + up + down - 4.0 * center
}

/// Computes the catalyzed reaction rate (`base_rate` × catalyst multiplier)
/// and writes the breakdown to `out_report`.
///
/// # Safety
///
/// `out_report` must be non-null and point to writable memory for one
/// `CatalystReport`; a null pointer fails with `ERR_NULL_POINTER`.
/// `base_rate` and `catalyst` are passed by value (no ownership transfer);
/// invalid values fail with `ERR_INVALID_ARGUMENT`.
#[unsafe(no_mangle)]
pub extern "C" fn physchem_catalyst_rate_multiplier(
    base_rate: f64,
    catalyst: CatalystEffect,
    out_report: *mut CatalystReport,
) -> Bool {
    if !finite_non_negative(base_rate) {
        set_error(ERR_INVALID_ARGUMENT, "invalid catalyst base rate");
        return Bool::FALSE;
    }
    let Some(multiplier) = catalyst_multiplier(catalyst) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid catalyst parameters");
        return Bool::FALSE;
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "catalyst output is null");
        return Bool::FALSE;
    };
    *out_report = CatalystReport {
        rate_multiplier: multiplier,
        effective_rate: base_rate * multiplier,
    };
    clear_error();
    Bool::TRUE
}

/// Computes the Gray-Scott reaction and diffusion terms for a single grid cell.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `GrayScottReactionReport`;
/// a null pointer fails with `ERR_NULL_POINTER`. Non-finite or invalid scalar
/// inputs and catalyst fields fail with `ERR_INVALID_ARGUMENT`.
#[unsafe(no_mangle)]
pub extern "C" fn physchem_gray_scott_reaction_terms(
    u: f64,
    v: f64,
    laplacian_u: f64,
    laplacian_v: f64,
    params: GrayScottParams,
    catalyst: CatalystEffect,
    out_report: *mut GrayScottReactionReport,
) -> Bool {
    if !u.is_finite()
        || !v.is_finite()
        || !laplacian_u.is_finite()
        || !laplacian_v.is_finite()
        || !params_valid(params)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Gray-Scott reaction terms");
        return Bool::FALSE;
    }
    let Some(multiplier) = catalyst_multiplier(catalyst) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid Gray-Scott catalyst");
        return Bool::FALSE;
    };
    let reaction_rate = u * v * v * multiplier;
    let diffusion_u_term = params.diffusion_u * laplacian_u / (params.dx * params.dx);
    let diffusion_v_term = params.diffusion_v * laplacian_v / (params.dx * params.dx);
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Gray-Scott reaction output is null");
        return Bool::FALSE;
    };
    *out_report = GrayScottReactionReport {
        reaction_rate,
        diffusion_u_term,
        diffusion_v_term,
        du_dt: diffusion_u_term - reaction_rate + params.feed_rate * (1.0 - u),
        dv_dt: diffusion_v_term + reaction_rate - (params.feed_rate + params.kill_rate) * v,
    };
    clear_error();
    Bool::TRUE
}

/// Advances a 2D Gray-Scott reaction-diffusion grid by one explicit Euler step.
///
/// # Safety
///
/// `u_values` and `v_values` must point to readable arrays of `width * height`
/// `f64` elements; `out_u_values` and `out_v_values` must point to writable
/// memory for `capacity` `f64` elements (`capacity >= width * height`,
/// `width * height <= MAX_GRID_CELLS`). Null grid pointers fail with
/// `ERR_NULL_POINTER`; `out_report` may be null, otherwise it must point to
/// writable memory for one `ReactionDiffusionReport`. No ownership is transferred.
#[unsafe(no_mangle)]
pub extern "C" fn physchem_gray_scott_step_2d(
    u_values: *const f64,
    v_values: *const f64,
    width: u32,
    height: u32,
    params: GrayScottParams,
    catalyst: CatalystEffect,
    dt: f64,
    out_u_values: *mut f64,
    out_v_values: *mut f64,
    capacity: u32,
    out_report: *mut ReactionDiffusionReport,
) -> Bool {
    let Some(cell_count) = width.checked_mul(height) else {
        set_error(ERR_CAPACITY, "Gray-Scott grid size overflow");
        return Bool::FALSE;
    };
    if width == 0 || height == 0 || cell_count > MAX_GRID_CELLS || capacity < cell_count {
        set_error(ERR_CAPACITY, "invalid Gray-Scott grid capacity");
        return Bool::FALSE;
    }
    if u_values.is_null() || v_values.is_null() || out_u_values.is_null() || out_v_values.is_null()
    {
        set_error(ERR_NULL_POINTER, "Gray-Scott grid pointers are null");
        return Bool::FALSE;
    }
    if !params_valid(params) || !finite_non_negative(dt) {
        set_error(ERR_INVALID_ARGUMENT, "invalid Gray-Scott grid parameters");
        return Bool::FALSE;
    }
    let Some(multiplier) = catalyst_multiplier(catalyst) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid Gray-Scott grid catalyst");
        return Bool::FALSE;
    };

    let count = cell_count as usize;
    let width_usize = width as usize;
    let height_usize = height as usize;
    macro_rules! input {
        ($p:expr, $name:expr) => {
            match unsafe { crate::ffi::checked_input_slice($p, count, $name) } {
                Ok(v) => v,
                Err(e) => {
                    crate::ffi::formula_result::<()>(Err(e));
                    return Bool::FALSE;
                }
            }
        };
    }
    let u_values = input!(u_values, "u_values");
    let v_values = input!(v_values, "v_values");
    let out_u = match unsafe {
        crate::ffi::checked_output_slice(out_u_values, capacity as usize, "out_u_values")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
    let out_v = match unsafe {
        crate::ffi::checked_output_slice(out_v_values, capacity as usize, "out_v_values")
    } {
        Ok(v) => v,
        Err(e) => {
            crate::ffi::formula_result::<()>(Err(e));
            return Bool::FALSE;
        }
    };
    if u_values
        .iter()
        .chain(v_values)
        .any(|value| !value.is_finite())
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "Gray-Scott grid contains non-finite values",
        );
        return Bool::FALSE;
    }

    let inv_dx2 = 1.0 / (params.dx * params.dx);
    let mut report = ReactionDiffusionReport {
        cell_count,
        ..ReactionDiffusionReport::default()
    };
    let mut total_u_acc = KahanSum::default();
    let mut total_v_acc = KahanSum::default();
    for y in 0..height_usize {
        for x in 0..width_usize {
            let index = y * width_usize + x;
            let u = u_values[index];
            let v = v_values[index];
            let reaction_rate = u * v * v * multiplier;
            let du_dt = params.diffusion_u
                * laplacian_center(u_values, width_usize, height_usize, x, y)
                * inv_dx2
                - reaction_rate
                + params.feed_rate * (1.0 - u);
            let dv_dt = params.diffusion_v
                * laplacian_center(v_values, width_usize, height_usize, x, y)
                * inv_dx2
                + reaction_rate
                - (params.feed_rate + params.kill_rate) * v;
            let next_u = (u + du_dt * dt).max(0.0);
            let next_v = (v + dv_dt * dt).max(0.0);
            out_u[index] = next_u;
            out_v[index] = next_v;
            report.max_delta_u = f64::max(report.max_delta_u, (next_u - u).abs());
            report.max_delta_v = f64::max(report.max_delta_v, (next_v - v).abs());
            total_u_acc.add(next_u);
            total_v_acc.add(next_v);
            report.max_reaction_rate = f64::max(report.max_reaction_rate, reaction_rate.abs());
        }
    }
    report.total_u = total_u_acc.value();
    report.total_v = total_v_acc.value();
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = report;
    }
    clear_error();
    Bool::TRUE
}

/// Advances a reaction-diffusion concentration by one explicit Euler step.
///
/// # Safety
///
/// Takes only scalar values; no pointers are dereferenced. `concentration`,
/// `laplacian`, `reaction_rate`, and `source` must be finite and
/// `diffusion_coefficient` and `dt` finite and non-negative; invalid inputs
/// return `f64::NAN` instead of an error code.
#[unsafe(no_mangle)]
pub extern "C" fn physchem_reaction_diffusion_explicit(
    concentration: f64,
    laplacian: f64,
    diffusion_coefficient: f64,
    reaction_rate: f64,
    source: f64,
    dt: f64,
) -> f64 {
    if !concentration.is_finite()
        || !laplacian.is_finite()
        || !finite_non_negative(diffusion_coefficient)
        || !reaction_rate.is_finite()
        || !source.is_finite()
        || !finite_non_negative(dt)
    {
        return f64::NAN;
    }
    (concentration + dt * (diffusion_coefficient * laplacian + reaction_rate + source)).max(0.0)
}

/// Computes the buoyancy acceleration and force from a concentration difference.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `ConcentrationBuoyancyReport`;
/// a null pointer fails with `ERR_NULL_POINTER`. Non-finite or negative scalar
/// inputs fail with `ERR_INVALID_ARGUMENT`.
#[unsafe(no_mangle)]
pub extern "C" fn physchem_concentration_buoyancy(
    concentration: f64,
    reference_concentration: f64,
    reference_density: f64,
    expansion_coefficient: f64,
    volume: f64,
    gravity: Vec3,
    out_report: *mut ConcentrationBuoyancyReport,
) -> Bool {
    if !concentration.is_finite()
        || !reference_concentration.is_finite()
        || !finite_non_negative(reference_density)
        || !expansion_coefficient.is_finite()
        || !finite_non_negative(volume)
        || !vec3_finite(gravity)
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid concentration buoyancy parameters",
        );
        return Bool::FALSE;
    }
    let density_delta =
        -reference_density * expansion_coefficient * (concentration - reference_concentration);
    let density = (reference_density + density_delta).max(0.0);
    let acceleration = -vec3_to_rapier(gravity) * (density_delta / reference_density.max(1.0e-12));
    let force = acceleration * (density * volume);
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "concentration buoyancy output is null");
        return Bool::FALSE;
    };
    *out_report = ConcentrationBuoyancyReport {
        density,
        density_delta,
        buoyancy_acceleration: vec3_from_rapier(acceleration),
        buoyancy_force: vec3_from_rapier(force),
    };
    clear_error();
    Bool::TRUE
}
