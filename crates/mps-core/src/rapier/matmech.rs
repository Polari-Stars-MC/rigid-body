//! Material mechanics C ABI — thin `#[unsafe(no_mangle)]` wrappers around the
//! pure `mps_formula::material_mechanics` helpers (Hooke's law, elastic moduli,
//! yield criteria, fracture mechanics, fatigue, creep, beam theory).
//!
//! Scalar results reuse [`crate::rapier::ffi::ffi_scalar`] (null `out` →
//! `Bool::FALSE`, `None` result → `Bool::FALSE`, otherwise writes and returns
//! `Bool::TRUE`). Multi-valued results (`principal_stresses`, `miners_damage`)
//! are written out explicitly.
//!
//! No `WorldHandle` / Rapier state is touched — these are pure calculators,
//! mirroring the `molecular` / `fracture` FFI modules.
//!
//! Rust module name is `matmech`; the exported C symbols are prefixed
//! `material_mechanics_`.
//!
//! NOTE: functions are written out explicitly (no macro) because cbindgen does
//! not expand declarative macros, so macro-generated `pub extern "C" fn`
//! items are silently omitted from `rigid_body.h`.

use std::slice;

use crate::rapier::error::{ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ffi_guard, set_error};
use crate::rapier::ffi::{Bool, ffi_scalar};
use mps_formula::material_mechanics::*;

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_hookes_law_uniaxial(
    stress: f64,
    youngs_modulus: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || hookes_law_uniaxial(stress, youngs_modulus))
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_stress_from_strain(
    youngs_modulus: f64,
    strain: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || stress_from_strain(youngs_modulus, strain))
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_shear_modulus(
    youngs_modulus: f64,
    poisson_ratio: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || shear_modulus(youngs_modulus, poisson_ratio))
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_bulk_modulus(
    youngs_modulus: f64,
    poisson_ratio: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || bulk_modulus(youngs_modulus, poisson_ratio))
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_lame_lambda(
    youngs_modulus: f64,
    poisson_ratio: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || lame_lambda(youngs_modulus, poisson_ratio))
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_von_mises_stress(
    sx: f64,
    sy: f64,
    sz: f64,
    txy: f64,
    tyz: f64,
    tzx: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || von_mises_stress(sx, sy, sz, txy, tyz, tzx))
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_von_mises_yield_check(
    von_mises_stress: f64,
    yield_stress: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        von_mises_yield_check(von_mises_stress, yield_stress)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_tresca_shear_stress(
    sigma_1: f64,
    sigma_3: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || tresca_shear_stress(sigma_1, sigma_3))
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_tresca_yield_check(
    sigma_1: f64,
    sigma_3: f64,
    yield_stress: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || tresca_yield_check(sigma_1, sigma_3, yield_stress))
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_ki_center_crack(
    stress: f64,
    crack_half_length: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || ki_center_crack(stress, crack_half_length))
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_ki_edge_crack(
    stress: f64,
    crack_length: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || ki_edge_crack(stress, crack_length))
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_fracture_check(
    stress_intensity: f64,
    fracture_toughness: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || fracture_check(stress_intensity, fracture_toughness))
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_critical_crack_length(
    stress: f64,
    fracture_toughness: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || critical_crack_length(stress, fracture_toughness))
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_basquin_stress_amplitude(
    cycles_to_failure: f64,
    fatigue_strength_coefficient: f64,
    fatigue_exponent: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        basquin_stress_amplitude(
            cycles_to_failure,
            fatigue_strength_coefficient,
            fatigue_exponent,
        )
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_basquin_cycles_to_failure(
    stress_amplitude: f64,
    fatigue_strength_coefficient: f64,
    fatigue_exponent: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        basquin_cycles_to_failure(
            stress_amplitude,
            fatigue_strength_coefficient,
            fatigue_exponent,
        )
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_coffin_manson_strain_amplitude(
    cycles_to_failure: f64,
    ductility_coefficient: f64,
    ductility_exponent: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        coffin_manson_strain_amplitude(cycles_to_failure, ductility_coefficient, ductility_exponent)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_goodman_correction(
    stress_amplitude: f64,
    mean_stress: f64,
    ultimate_tensile: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        goodman_correction(stress_amplitude, mean_stress, ultimate_tensile)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_norton_creep_rate(
    stress: f64,
    temperature: f64,
    a: f64,
    n: f64,
    activation_energy: f64,
    gas_constant: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        norton_creep_rate(stress, temperature, a, n, activation_energy, gas_constant)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_beam_bending_stress(
    bending_moment: f64,
    distance_from_neutral_axis: f64,
    area_moment_of_inertia: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        beam_bending_stress(
            bending_moment,
            distance_from_neutral_axis,
            area_moment_of_inertia,
        )
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_beam_deflection_center_point_load(
    load: f64,
    span: f64,
    youngs_modulus: f64,
    moment_of_inertia: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        beam_deflection_center_point_load(load, span, youngs_modulus, moment_of_inertia)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_euler_buckling_load(
    youngs_modulus: f64,
    moment_of_inertia: f64,
    effective_length_factor: f64,
    column_length: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        euler_buckling_load(
            youngs_modulus,
            moment_of_inertia,
            effective_length_factor,
            column_length,
        )
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_slenderness_ratio(
    effective_length_factor: f64,
    column_length: f64,
    radius_of_gyration: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        slenderness_ratio(effective_length_factor, column_length, radius_of_gyration)
    })
}

/// Principal stresses from a 3D stress tensor. Writes (σ₁, σ₂, σ₃) sorted
/// descending into `out` (capacity must be ≥ 3). Returns `Bool::FALSE` on
/// invalid input or null/short `out`.
#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_principal_stresses(
    sx: f64,
    sy: f64,
    sz: f64,
    txy: f64,
    tyz: f64,
    tzx: f64,
    out: *mut f64,
) -> Bool {
    if out.is_null() {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return Bool::FALSE;
    }
    // 三个分量各走一次 ffi_scalar（panic 屏障 + 错误槽语义统一）。
    let s1 = ffi_scalar(out, || {
        principal_stresses(sx, sy, sz, txy, tyz, tzx).map(|(a, _, _)| a)
    });
    let s2 = ffi_scalar(unsafe { out.add(1) }, || {
        principal_stresses(sx, sy, sz, txy, tyz, tzx).map(|(_, b, _)| b)
    });
    let s3 = ffi_scalar(unsafe { out.add(2) }, || {
        principal_stresses(sx, sy, sz, txy, tyz, tzx).map(|(_, _, c)| c)
    });
    if s1 == Bool::TRUE && s2 == Bool::TRUE && s3 == Bool::TRUE {
        Bool::TRUE
    } else {
        Bool::FALSE
    }
}

/// Miner's linear damage rule: D = Σ (nᵢ / N_fᵢ). `ratios` points to
/// `count` `f64` elements (each nᵢ/N_fᵢ). Writes the summed damage into `out`.
/// Returns `Bool::FALSE` on null pointers, empty/short input, or invalid data.
#[unsafe(no_mangle)]
pub extern "C" fn material_mechanics_miners_damage(
    ratios: *const f64,
    count: u32,
    out: *mut f64,
) -> Bool {
    if ratios.is_null() || out.is_null() || count == 0 {
        set_error(ERR_INVALID_ARGUMENT, "ratios/out is null or count is zero");
        return Bool::FALSE;
    }
    ffi_guard(Bool::FALSE, || {
        let ratios = unsafe { slice::from_raw_parts(ratios, count as usize) };
        match miners_damage(ratios) {
            Some(d) => {
                unsafe { *out = d };
                Bool::TRUE
            }
            None => Bool::FALSE,
        }
    })
}
