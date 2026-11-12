//! Quantum mechanics C ABI — thin `#[unsafe(no_mangle)]` wrappers around the
//! pure `mps_formula::quantum` scalar helpers (de Broglie, particle-in-a-box,
//! hydrogen spectra, uncertainty, perturbation theory, fine structure,
//! variational method, coherent states, spherical harmonics).
//!
//! Scalar results reuse [`crate::rapier::ffi::ffi_scalar`] (null `out` →
//! `Bool::FALSE`, `None` result → `Bool::FALSE`, otherwise writes and returns
//! `Bool::TRUE`). The two tuple-valued helpers (`degenerate_perturbation_2x2`,
//! `time_evolution_phase`) write two `f64` outputs. Integer arguments
//! (`u32`/`i32`) are passed through unchanged.
//!
//! No `WorldHandle` / Rapier state is touched — these are pure calculators.
//! The struct/Vec3-returning `quantum_*` FFI already live in
//! `mps_formula::quantum` and are not re-wrapped here.
//!
//! Rust module name is `qphys`; the exported C symbols are prefixed `quantum_`.
//!
//! NOTE: functions are written out explicitly (no macro) because cbindgen does
//! not expand declarative macros, so macro-generated `pub extern "C" fn`
//! items are silently omitted from `rigid_body.h`.

use crate::rapier::ffi::{Bool, ffi_scalar};
use mps_formula::quantum::*;

#[unsafe(no_mangle)]
pub extern "C" fn quantum_free_particle_energy(wave_number: f64, mass: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || free_particle_energy(wave_number, mass))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_de_broglie_wavelength(mass: f64, velocity: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || de_broglie_wavelength(mass, velocity))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_infinite_well_energy(
    quantum_number: u32,
    mass: f64,
    well_width: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        infinite_well_energy(quantum_number, mass, well_width)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_infinite_well_wave_function(
    quantum_number: u32,
    well_width: f64,
    x: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        infinite_well_wave_function(quantum_number, well_width, x)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_bohr_radius(out: *mut f64) -> Bool {
    ffi_scalar(out, || Some(bohr_radius()))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_hydrogen_energy_level(quantum_number: u32, out: *mut f64) -> Bool {
    ffi_scalar(out, || hydrogen_energy_level(quantum_number))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_hydrogen_orbital_radius(quantum_number: u32, out: *mut f64) -> Bool {
    ffi_scalar(out, || hydrogen_orbital_radius(quantum_number))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_hydrogen_transition_wavelength(n1: u32, n2: u32, out: *mut f64) -> Bool {
    ffi_scalar(out, || hydrogen_transition_wavelength(n1, n2))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_minimum_uncertainty_product(out: *mut f64) -> Bool {
    ffi_scalar(out, || Some(minimum_uncertainty_product()))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_fermi_golden_rule_linear(
    matrix_element2: f64,
    density_of_states: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        fermi_golden_rule_linear(matrix_element2, density_of_states)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_spin_orbit_energy(
    n: f64,
    l: f64,
    j: f64,
    atomic_number: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || spin_orbit_energy(n, l, j, atomic_number))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_fine_structure_constant(out: *mut f64) -> Bool {
    ffi_scalar(out, || Some(fine_structure_constant()))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_variational_hydrogen_energy(alpha: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || variational_hydrogen_energy(alpha))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_variational_hydrogen_optimal_alpha(out: *mut f64) -> Bool {
    ffi_scalar(out, || Some(variational_hydrogen_optimal_alpha()))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_coherent_state_photon_probability(
    alpha_squared: f64,
    n: u32,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || coherent_state_photon_probability(alpha_squared, n))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_spherical_harmonic_real(
    l: i32,
    m: i32,
    theta: f64,
    phi: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || spherical_harmonic_real(l, m, theta, phi))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_angular_momentum_squared(j: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || angular_momentum_squared(j))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_photoelectric_threshold(work_function: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || photoelectric_threshold(work_function))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_photoelectric_max_kinetic(
    frequency: f64,
    work_function: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || photoelectric_max_kinetic(frequency, work_function))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_compton_wavelength_shift(scattering_angle: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || compton_wavelength_shift(scattering_angle))
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_compton_scattered_wavelength(
    lambda: f64,
    scattering_angle: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        compton_scattered_wavelength(lambda, scattering_angle)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_rabi_oscillation_probability(
    rabi_frequency: f64,
    detuning: f64,
    time: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        rabi_oscillation_probability(rabi_frequency, detuning, time)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_landau_level(
    quantum_number: i32,
    magnetic_field: f64,
    charge: f64,
    mass: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        landau_level(quantum_number, magnetic_field, charge, mass)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_einstein_a_coefficient(
    transition_frequency: f64,
    dipole_moment: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        einstein_a_coefficient(transition_frequency, dipole_moment)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_clebsch_gordan_allowed(
    j1: f64,
    j2: f64,
    j3: f64,
    m1: f64,
    m2: f64,
    m3: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || clebsch_gordan_allowed(j1, j2, j3, m1, m2, m3))
}

/// Degenerate 2×2 perturbation eigenvalues. Writes (λ₁, λ₂) into `out_e1` /
/// `out_e2`. Returns `Bool::FALSE` on invalid input or a null output.
#[unsafe(no_mangle)]
pub extern "C" fn quantum_degenerate_perturbation_2x2(
    h11: f64,
    h12: f64,
    h22: f64,
    out_e1: *mut f64,
    out_e2: *mut f64,
) -> Bool {
    let a = ffi_scalar(out_e1, || -> Option<f64> {
        degenerate_perturbation_2x2(h11, h12, h22).map(|(e1, _)| e1)
    });
    let b = ffi_scalar(out_e2, || -> Option<f64> {
        degenerate_perturbation_2x2(h11, h12, h22).map(|(_, e2)| e2)
    });
    if a == Bool::TRUE && b == Bool::TRUE {
        Bool::TRUE
    } else {
        Bool::FALSE
    }
}

/// Time-evolution phase factor e^{-iEt/ℏ}. Writes (real, imag) into
/// `out_real` / `out_imag`. Returns `Bool::FALSE` on a null output.
#[unsafe(no_mangle)]
pub extern "C" fn quantum_time_evolution_phase(
    energy: f64,
    time: f64,
    out_real: *mut f64,
    out_imag: *mut f64,
) -> Bool {
    let a = ffi_scalar(out_real, || -> Option<f64> {
        time_evolution_phase(energy, time).map(|(re, _)| re)
    });
    let b = ffi_scalar(out_imag, || -> Option<f64> {
        time_evolution_phase(energy, time).map(|(_, im)| im)
    });
    if a == Bool::TRUE && b == Bool::TRUE {
        Bool::TRUE
    } else {
        Bool::FALSE
    }
}
