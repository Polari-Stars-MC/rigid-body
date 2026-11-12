//! Relativity C ABI — thin `#[unsafe(no_mangle)]` wrappers around the pure
//! `mps_formula::relativity` scalar helpers (Kerr/Schwarzschild horizons &
//! ISCO, gravitational & cosmological redshift, relativistic Doppler, Einstein
//! radius, gravitational-wave strain/chirp/SNR/inspiral time, Schwarzschild
//! effective potential, Lense-Thirring frame dragging).
//!
//! Scalar results reuse [`crate::rapier::ffi::ffi_scalar`] (null `out` →
//! `Bool::FALSE`, `None` result → `Bool::FALSE`, otherwise writes and returns
//! `Bool::TRUE`). The two tuple-valued helpers (`kerr_horizon_radii`,
//! `reissner_nordstrom_horizons`) write two `f64` outputs. `bool` arguments
//! (`prograde`, `approaching`) are passed as `Bool` (C `uint8_t`).
//!
//! No `WorldHandle` / Rapier state is touched — these are pure calculators.
//! The four-vector / report-returning `rel_*` FFI already live in
//! `mps_formula::relativity` and are not re-wrapped here.
//!
//! Rust module name is `rel`; the exported C symbols are prefixed
//! `relativity_`.
//!
//! NOTE: functions are written out explicitly (no macro) because cbindgen does
//! not expand declarative macros, so macro-generated `pub extern "C" fn`
//! items are silently omitted from `rigid_body.h`.

use crate::rapier::ffi::{Bool, ffi_scalar};
use mps_formula::relativity::*;

#[unsafe(no_mangle)]
pub extern "C" fn relativity_kerr_horizon_radii(
    mass: f64,
    spin_parameter: f64,
    g: f64,
    out_event: *mut f64,
    out_cauchy: *mut f64,
) -> Bool {
    let a = ffi_scalar(out_event, || -> Option<f64> {
        kerr_horizon_radii(mass, spin_parameter, g).map(|(e, _)| e)
    });
    let b = ffi_scalar(out_cauchy, || -> Option<f64> {
        kerr_horizon_radii(mass, spin_parameter, g).map(|(_, c)| c)
    });
    if a == Bool::TRUE && b == Bool::TRUE {
        Bool::TRUE
    } else {
        Bool::FALSE
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_kerr_ergosphere_radius(
    mass: f64,
    spin_parameter: f64,
    polar_angle: f64,
    g: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::kerr_ergosphere_radius(mass, spin_parameter, polar_angle, g)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_kerr_frame_dragging_frequency(
    mass: f64,
    spin_parameter: f64,
    r: f64,
    theta: f64,
    g: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::kerr_frame_dragging_frequency(mass, spin_parameter, r, theta, g)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_schwarzschild_isco(mass: f64, g: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || mps_formula::relativity::schwarzschild_isco(mass, g))
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_kerr_isco(
    mass: f64,
    spin_parameter: f64,
    g: f64,
    prograde: Bool,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::kerr_isco(mass, spin_parameter, g, prograde.0 != 0)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_gravitational_redshift(
    mass: f64,
    radius: f64,
    g: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::gravitational_redshift(mass, radius, g)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_reissner_nordstrom_horizons(
    mass: f64,
    charge: f64,
    g: f64,
    out_outer: *mut f64,
    out_inner: *mut f64,
) -> Bool {
    let a = ffi_scalar(out_outer, || -> Option<f64> {
        reissner_nordstrom_horizons(mass, charge, g).map(|(o, _)| o)
    });
    let b = ffi_scalar(out_inner, || -> Option<f64> {
        reissner_nordstrom_horizons(mass, charge, g).map(|(_, i)| i)
    });
    if a == Bool::TRUE && b == Bool::TRUE {
        Bool::TRUE
    } else {
        Bool::FALSE
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_gw_strain_amplitude(
    distance: f64,
    chirp_mass_kg: f64,
    orbital_frequency: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::gw_strain_amplitude(distance, chirp_mass_kg, orbital_frequency)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_chirp_mass(mass1: f64, mass2: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || mps_formula::relativity::chirp_mass(mass1, mass2))
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_gw_frequency_derivative(
    frequency: f64,
    chirp_mass_kg: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::gw_frequency_derivative(frequency, chirp_mass_kg)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_relativistic_doppler_longitudinal(
    source_frequency: f64,
    relative_velocity: f64,
    approaching: Bool,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::relativistic_doppler_longitudinal(
            source_frequency,
            relative_velocity,
            approaching.0 != 0,
        )
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_relativistic_doppler_transverse(
    source_frequency: f64,
    relative_velocity: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::relativistic_doppler_transverse(
            source_frequency,
            relative_velocity,
        )
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_einstein_radius(
    mass_kg: f64,
    dist_lens: f64,
    dist_source: f64,
    dist_ls: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::einstein_radius(mass_kg, dist_lens, dist_source, dist_ls)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_cosmological_redshift(scale_factor: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::cosmological_redshift(scale_factor)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_redshift_from_wavelengths(
    observed: f64,
    emitted: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::redshift_from_wavelengths(observed, emitted)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_lense_thirring_angular_frequency(
    mass_kg: f64,
    spin_parameter: f64,
    orbital_radius: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::lense_thirring_angular_frequency(
            mass_kg,
            spin_parameter,
            orbital_radius,
        )
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_schwarzschild_effective_potential(
    r: f64,
    rs: f64,
    angular_momentum: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::schwarzschild_effective_potential(r, rs, angular_momentum)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_gw_inspiral_snr(
    strain_rss: f64,
    f_min: f64,
    f_max: f64,
    noise_psd: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::gw_inspiral_snr(strain_rss, f_min, f_max, noise_psd)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_gw_inspiral_time_to_coalescence(
    chirp_mass_kg: f64,
    f_gw_hz: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::gw_inspiral_time_to_coalescence(chirp_mass_kg, f_gw_hz)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_relativistic_total_energy(
    rest_mass: f64,
    lorentz_factor: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::relativistic_total_energy(rest_mass, lorentz_factor)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_relativistic_momentum(
    rest_mass: f64,
    speed: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::relativistic_momentum(rest_mass, speed)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_relativistic_energy_from_momentum(
    rest_mass: f64,
    momentum: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::relativistic_energy_from_momentum(rest_mass, momentum)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_relativistic_aberration(
    cos_theta: f64,
    beta: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::relativistic_aberration(cos_theta, beta)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_relativistic_doppler_beaming_factor(
    beta: f64,
    cos_theta: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::relativistic_doppler_beaming_factor(beta, cos_theta)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_photon_sphere_radius(mass: f64, g: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::photon_sphere_radius(mass, g)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_hawking_temperature(mass: f64, g: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::hawking_temperature(mass, g)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_hubble_recession_velocity(
    distance: f64,
    hubble_constant: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::hubble_recession_velocity(distance, hubble_constant)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_hubble_distance(
    redshift: f64,
    hubble_constant: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::hubble_distance(redshift, hubble_constant)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn relativity_flat_universe_lookback_time(
    redshift: f64,
    hubble_time: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::relativity::flat_universe_lookback_time(redshift, hubble_time)
    })
}
