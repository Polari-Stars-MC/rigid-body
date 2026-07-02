use std::slice;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, CatalystEffect, CatalystReport, ConcentrationBuoyancyReport, GrayScottParams,
    GrayScottReactionReport, ReactionDiffusionReport, Vec3, vec3_finite, vec3_from_rapier,
    vec3_to_rapier,
};

use crate::rapier::math::{KahanSum, finite_non_negative, finite_positive};

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
    let u_values = unsafe { slice::from_raw_parts(u_values, count) };
    let v_values = unsafe { slice::from_raw_parts(v_values, count) };
    let out_u = unsafe { slice::from_raw_parts_mut(out_u_values, capacity as usize) };
    let out_v = unsafe { slice::from_raw_parts_mut(out_v_values, capacity as usize) };
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

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> GrayScottParams {
        GrayScottParams {
            diffusion_u: 0.16,
            diffusion_v: 0.08,
            feed_rate: 0.060,
            kill_rate: 0.062,
            dx: 1.0,
        }
    }

    #[test]
    fn catalyst_and_gray_scott_terms_work() {
        let catalyst = CatalystEffect {
            concentration: 2.0,
            strength: 1.0,
            saturation: 2.0,
        };
        let mut catalyst_report = CatalystReport::default();
        assert_eq!(
            physchem_catalyst_rate_multiplier(4.0, catalyst, &mut catalyst_report),
            Bool::TRUE
        );
        assert_eq!(catalyst_report.rate_multiplier, 1.5);
        assert_eq!(catalyst_report.effective_rate, 6.0);

        let mut terms = GrayScottReactionReport::default();
        assert_eq!(
            physchem_gray_scott_reaction_terms(1.0, 0.5, 0.0, 0.0, params(), catalyst, &mut terms),
            Bool::TRUE
        );
        assert!(terms.reaction_rate > 0.0);
        assert!(terms.du_dt < 0.0);
        assert!(terms.dv_dt > 0.0);
    }

    #[test]
    fn gray_scott_grid_step_updates_concentrations() {
        let u = [1.0, 1.0, 1.0, 1.0];
        let v = [0.0, 0.2, 0.0, 0.0];
        let mut out_u = [0.0; 4];
        let mut out_v = [0.0; 4];
        let mut report = ReactionDiffusionReport::default();
        assert_eq!(
            physchem_gray_scott_step_2d(
                u.as_ptr(),
                v.as_ptr(),
                2,
                2,
                params(),
                CatalystEffect::default(),
                0.1,
                out_u.as_mut_ptr(),
                out_v.as_mut_ptr(),
                4,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(report.cell_count, 4);
        assert!(report.max_delta_v > 0.0);
        assert!(out_u[1] < 1.0);
    }

    #[test]
    fn concentration_buoyancy_maps_density_to_force() {
        let mut report = ConcentrationBuoyancyReport::default();
        assert_eq!(
            physchem_concentration_buoyancy(
                2.0,
                1.0,
                1000.0,
                0.1,
                0.5,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(report.density < 1000.0);
        assert!(report.buoyancy_force.y < 0.0);
    }
}
