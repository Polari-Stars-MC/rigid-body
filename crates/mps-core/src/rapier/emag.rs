//! Electromagnetism C ABI — thin `#[unsafe(no_mangle)]` wrappers around the
//! pure `mps_formula::electromagnetism` scalar helpers (Poynting, wave
//! propagation, antennas, impedance matching, coax, scattering, Faraday).
//!
//! Scalar results reuse [`crate::rapier::ffi::ffi_scalar`] (null `out` →
//! `Bool::FALSE`, `None` result → `Bool::FALSE`, otherwise writes and returns
//! `Bool::TRUE`). `transmission_line_input_impedance` writes two `f64`
//! outputs (real, imag).
//!
//! No `WorldHandle` / Rapier state is touched — these are pure calculators.
//! The Vec3-returning helpers (Biot–Savart, Poynting vector) and the existing
//! `em_*` FFI already live in `mps_formula::electromagnetism` and are not
//! re-wrapped here.
//!
//! Rust module name is `emag`; the exported C symbols are prefixed
//! `electromagnetism_`.
//!
//! NOTE: functions are written out explicitly (no macro) because cbindgen does
//! not expand declarative macros, so macro-generated `pub extern "C" fn`
//! items are silently omitted from `rigid_body.h`.

use crate::rapier::ffi::{Bool, ffi_scalar};
use mps_formula::electromagnetism::*;

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_poynting_magnitude_plane_wave(
    e_field_magnitude: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || poynting_magnitude_plane_wave(e_field_magnitude))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_phase_velocity(refractive_index: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || phase_velocity(refractive_index))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_wavelength_in_medium(
    frequency: f64,
    refractive_index: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || wavelength_in_medium(frequency, refractive_index))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_intrinsic_impedance(
    permeability: f64,
    permittivity: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || intrinsic_impedance(permeability, permittivity))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_skin_depth(
    frequency: f64,
    permeability: f64,
    conductivity: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || skin_depth(frequency, permeability, conductivity))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_vacuum_wavelength(frequency: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || vacuum_wavelength(frequency))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_wave_frequency(wavelength: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || wave_frequency(wavelength))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_dipole_radiation_resistance(
    dipole_length: f64,
    wavelength: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        dipole_radiation_resistance(dipole_length, wavelength)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_half_wave_dipole_directivity(out: *mut f64) -> Bool {
    ffi_scalar(out, || Some(half_wave_dipole_directivity()))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_effective_aperture(
    gain_linear: f64,
    wavelength: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || effective_aperture(gain_linear, wavelength))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_far_field_distance(
    antenna_size: f64,
    wavelength: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || far_field_distance(antenna_size, wavelength))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_friis_power_received(
    transmit_power: f64,
    tx_gain: f64,
    rx_gain: f64,
    wavelength: f64,
    range: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        friis_power_received(transmit_power, tx_gain, rx_gain, wavelength, range)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_reflection_coefficient(
    load_impedance: f64,
    characteristic_impedance: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        reflection_coefficient(load_impedance, characteristic_impedance)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_vswr(reflection_coeff: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || vswr(reflection_coeff))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_return_loss(reflection_coeff: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || return_loss(reflection_coeff))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_quarter_wave_transformer(
    z0: f64,
    z_load: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || quarter_wave_transformer(z0, z_load))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_coaxial_impedance(
    inner_diameter: f64,
    outer_diameter: f64,
    relative_permittivity: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        coaxial_impedance(inner_diameter, outer_diameter, relative_permittivity)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_coaxial_cutoff_frequency(
    inner_diameter: f64,
    outer_diameter: f64,
    relative_permittivity: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        coaxial_cutoff_frequency(inner_diameter, outer_diameter, relative_permittivity)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_rayleigh_scattering_cross_section(
    refractive_index: f64,
    diameter: f64,
    wavelength: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        rayleigh_scattering_cross_section(refractive_index, diameter, wavelength)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_faraday_rotation(
    verdet_constant: f64,
    magnetic_field: f64,
    path_length: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        faraday_rotation(verdet_constant, magnetic_field, path_length)
    })
}

/// Transmission-line input impedance (lossless). Writes (real, imag) into
/// `out_real` / `out_imag`. Returns `Bool::FALSE` on invalid input or a null output.
#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_transmission_line_input_impedance(
    z0: f64,
    z_load_real: f64,
    z_load_imag: f64,
    phase_constant: f64,
    length: f64,
    out_real: *mut f64,
    out_imag: *mut f64,
) -> Bool {
    let a = ffi_scalar(out_real, || -> Option<f64> {
        transmission_line_input_impedance(z0, z_load_real, z_load_imag, phase_constant, length)
            .map(|(re, _)| re)
    });
    let b = ffi_scalar(out_imag, || -> Option<f64> {
        transmission_line_input_impedance(z0, z_load_real, z_load_imag, phase_constant, length)
            .map(|(_, im)| im)
    });
    if a == Bool::TRUE && b == Bool::TRUE {
        Bool::TRUE
    } else {
        Bool::FALSE
    }
}
