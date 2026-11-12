//! Astrophysics C ABI — thin `#[unsafe(no_mangle)]` wrappers around the pure
//! `mps_formula::astrophysics` scalar helpers (Roche / Hill sphere, Hubble's
//! law, NFW profile, blackbody, Jeans criterion, stellar structure, binaries,
//! exoplanets, galaxies).
//!
//! Scalar results reuse [`crate::rapier::ffi::ffi_scalar`] (null `out` →
//! `Bool::FALSE`, `None` result → `Bool::FALSE`, otherwise writes and returns
//! `Bool::TRUE`). The two tuple-valued helpers (`roche_limit`,
//! `habitable_zone_boundaries`) write two `f64` outputs.
//!
//! No `WorldHandle` / Rapier state is touched — these are pure calculators.
//! The N-body / `astro_*` FFI (Barnes-Hut, FMM, resonance, ...) already live in
//! `mps_formula::astrophysics` itself and are not re-wrapped here.
//!
//! Rust module name is `astrocalc`; the exported C symbols are prefixed
//! `astrophysics_`.
//!
//! NOTE: functions are written out explicitly (no macro) because cbindgen does
//! not expand declarative macros, so macro-generated `pub extern "C" fn`
//! items are silently omitted from `rigid_body.h`.

use crate::rapier::ffi::{Bool, ffi_scalar};
use mps_formula::astrophysics::*;

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_hill_sphere_radius(
    primary_mass: f64,
    secondary_mass: f64,
    semi_major_axis: f64,
    eccentricity: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        hill_sphere_radius(primary_mass, secondary_mass, semi_major_axis, eccentricity)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_lane_emden_first_zero(polytropic_index: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || lane_emden_first_zero(polytropic_index))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_mass_luminosity_relation(
    mass_solar: f64,
    exponent: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || mass_luminosity_relation(mass_solar, exponent))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_eddington_luminosity(
    mass: f64,
    opacity: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || eddington_luminosity(mass, opacity))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_eddington_luminosity_solar(
    mass_solar: f64,
    opacity: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || eddington_luminosity_solar(mass_solar, opacity))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_hubble_velocity(
    hubble_constant: f64,
    distance: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || hubble_velocity(hubble_constant, distance))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_hubble_distance(
    velocity: f64,
    hubble_constant: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || hubble_distance(velocity, hubble_constant))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_nfw_density(
    radius: f64,
    scale_radius: f64,
    characteristic_density: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        nfw_density(radius, scale_radius, characteristic_density)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_nfw_enclosed_mass(
    radius: f64,
    scale_radius: f64,
    characteristic_density: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        nfw_enclosed_mass(radius, scale_radius, characteristic_density)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_blackbody_spectral_radiance(
    wavelength: f64,
    temperature: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || blackbody_spectral_radiance(wavelength, temperature))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_wien_displacement(temperature: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || wien_displacement(temperature))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_jeans_mass(
    temperature: f64,
    density: f64,
    mean_molecular_weight: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        jeans_mass(temperature, density, mean_molecular_weight)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_jeans_length(
    temperature: f64,
    density: f64,
    mean_molecular_weight: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        jeans_length(temperature, density, mean_molecular_weight)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_main_sequence_lifetime(mass_solar: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || main_sequence_lifetime(mass_solar))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_mass_radius_relation(mass_solar: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || mass_radius_relation(mass_solar))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_chandrasekhar_mass_limit(out: *mut f64) -> Bool {
    ffi_scalar(out, || Some(chandrasekhar_mass_limit()))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_chandrasekhar_mass_kg(out: *mut f64) -> Bool {
    ffi_scalar(out, || Some(chandrasekhar_mass_kg()))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_mass_function(
    period_seconds: f64,
    semi_amplitude: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || mass_function(period_seconds, semi_amplitude))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_binary_semi_major_axis(
    total_mass: f64,
    period: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || binary_semi_major_axis(total_mass, period))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_ss73_disk_temperature(
    mass_kg: f64,
    accretion_rate: f64,
    radius: f64,
    inner_radius: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        ss73_disk_temperature(mass_kg, accretion_rate, radius, inner_radius)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_nickel56_decay_luminosity(
    nickel_mass_kg: f64,
    time_days: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || nickel56_decay_luminosity(nickel_mass_kg, time_days))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_transit_depth(
    planet_radius: f64,
    star_radius: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || transit_depth(planet_radius, star_radius))
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_radial_velocity_semi_amplitude(
    planet_mass_kg: f64,
    star_mass_kg: f64,
    period: f64,
    inclination: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        radial_velocity_semi_amplitude(planet_mass_kg, star_mass_kg, period, inclination)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_nfw_circular_velocity(
    r: f64,
    v_max: f64,
    r_scale: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || nfw_circular_velocity(r, v_max, r_scale))
}

/// Roche fluid/rigid limits. Writes (fluid, rigid) into `out_fluid` /
/// `out_rigid`. Returns `Bool::FALSE` on invalid input or a null output.
#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_roche_limit(
    primary_radius: f64,
    primary_density: f64,
    secondary_density: f64,
    out_fluid: *mut f64,
    out_rigid: *mut f64,
) -> Bool {
    let a = ffi_scalar(out_fluid, || -> Option<f64> {
        roche_limit(primary_radius, primary_density, secondary_density).map(|(f, _)| f)
    });
    let b = ffi_scalar(out_rigid, || -> Option<f64> {
        roche_limit(primary_radius, primary_density, secondary_density).map(|(_, r)| r)
    });
    if a == Bool::TRUE && b == Bool::TRUE {
        Bool::TRUE
    } else {
        Bool::FALSE
    }
}

/// Habitable-zone inner/outer radii. Writes (inner, outer) into `out_inner` /
/// `out_outer`. Returns `Bool::FALSE` on invalid input or a null output.
#[unsafe(no_mangle)]
pub extern "C" fn astrophysics_habitable_zone_boundaries(
    star_luminosity_solar: f64,
    out_inner: *mut f64,
    out_outer: *mut f64,
) -> Bool {
    let a = ffi_scalar(out_inner, || -> Option<f64> {
        habitable_zone_boundaries(star_luminosity_solar).map(|(i, _)| i)
    });
    let b = ffi_scalar(out_outer, || -> Option<f64> {
        habitable_zone_boundaries(star_luminosity_solar).map(|(_, o)| o)
    });
    if a == Bool::TRUE && b == Bool::TRUE {
        Bool::TRUE
    } else {
        Bool::FALSE
    }
}
