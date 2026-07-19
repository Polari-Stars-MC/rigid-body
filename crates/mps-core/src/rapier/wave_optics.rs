//! Wave optics and diffraction:
//! - Huygens–Fresnel principle (point-source superposition)
//! - Fresnel diffraction (near-field scalar diffraction)
//! - Fraunhofer diffraction (far-field, Fourier transform regime)
//! - Fresnel–Kirchhoff diffraction integral (obliquity factor)
//! - Young's double-slit interference with single-slit envelope
//! - Thin-film interference (rainbow colours, half-wave loss)
//! - Spherical wave emission and propagation
//! - Fresnel zone analysis
//! - Interference pattern caching (grid sampling)
//!
//! All functions are FFI-exported with C-compatible types.

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    ApertureDesc, Bool, ComplexAmplitude, DiffractionPoint, FresnelZoneReport,
    KirchhoffDiffractionPoint, PlaneWaveParams, PointSource, SphericalWavePoint, ThinFilmInterferenceReport,
    ThinFilmParams, YoungSlitPoint,
};

use crate::rapier::math::{finite, finite_non_negative, finite_positive};

const EPSILON: f64 = 1.0e-14;
pub const PI: f64 = std::f64::consts::PI;
const TWO_PI: f64 = 2.0 * PI;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn write_out<T: Copy>(out: *mut T, value: T) -> Bool {
    let Some(out) = (unsafe { out.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return Bool::FALSE;
    };
    *out = value;
    clear_error();
    Bool::TRUE
}

fn complex_from_polar(amplitude: f64, phase: f64) -> ComplexAmplitude {
    let real = amplitude * phase.cos();
    let imag = amplitude * phase.sin();
    ComplexAmplitude {
        real,
        imag,
        intensity: real * real + imag * imag,
    }
}

fn complex_add(a: ComplexAmplitude, b: ComplexAmplitude) -> ComplexAmplitude {
    let real = a.real + b.real;
    let imag = a.imag + b.imag;
    ComplexAmplitude {
        real,
        imag,
        intensity: real * real + imag * imag,
    }
}

fn wave_params_valid(params: &PlaneWaveParams) -> bool {
    finite_positive(params.wavenumber)
        && finite_positive(params.wavelength)
        && finite_non_negative(params.amplitude)
        && finite(params.phase_offset)
}

// ===========================================================================
// A. Plane wave & wavenumber utilities
// ===========================================================================

/// Compute wavenumber from wavelength: k = 2π / λ.
#[unsafe(no_mangle)]
pub extern "C" fn wo_wavenumber(wavelength: f64) -> f64 {
    if !finite_positive(wavelength) {
        set_error(ERR_INVALID_ARGUMENT, "wavelength must be positive and finite");
        return f64::NAN;
    }
    clear_error();
    TWO_PI / wavelength
}

/// Compute wavelength from wavenumber: λ = 2π / k.
#[unsafe(no_mangle)]
pub extern "C" fn wo_wavelength(wavenumber: f64) -> f64 {
    if !finite_positive(wavenumber) {
        set_error(ERR_INVALID_ARGUMENT, "wavenumber must be positive and finite");
        return f64::NAN;
    }
    clear_error();
    TWO_PI / wavenumber
}

/// Compute the complex amplitude of a plane wave at position (x, y, z):
///   E = A₀ · exp(i (k·r − ωt))
/// where k = (kx, ky, kz) and ωt is a global time phase offset.
///
/// For a wave propagating along the z-axis: E = A₀ · exp(i (k·z − φ₀))
#[unsafe(no_mangle)]
pub extern "C" fn wo_plane_wave(
    params: PlaneWaveParams,
    x: f64,
    y: f64,
    z: f64,
    kx: f64,
    ky: f64,
    kz: f64,
    out_amplitude: *mut ComplexAmplitude,
) -> Bool {
    if !wave_params_valid(&params) {
        return Bool::FALSE;
    }
    if !finite(x) || !finite(y) || !finite(z) || !finite(kx) || !finite(ky) || !finite(kz) {
        set_error(ERR_INVALID_ARGUMENT, "all coordinates and k components must be finite");
        return Bool::FALSE;
    }

    let k_dot_r = kx * x + ky * y + kz * z;
    let total_phase = k_dot_r - params.phase_offset;
    let amp = complex_from_polar(params.amplitude, total_phase);
    write_out(out_amplitude, amp)
}

// ===========================================================================
// B. Spherical wave
// ===========================================================================

/// Compute the complex amplitude of a spherical wave at an observation point.
///
///   E = A₀ · exp(i k r) / r
///
/// where r is the distance from the source to the observation point.
#[unsafe(no_mangle)]
pub extern "C" fn wo_spherical_wave(
    source_x: f64,
    source_y: f64,
    source_z: f64,
    obs_x: f64,
    obs_y: f64,
    obs_z: f64,
    wavenumber: f64,
    amplitude: f64,
    out_wave: *mut SphericalWavePoint,
) -> Bool {
    if !finite(source_x) || !finite(source_y) || !finite(source_z)
        || !finite(obs_x) || !finite(obs_y) || !finite(obs_z)
    {
        set_error(ERR_INVALID_ARGUMENT, "all coordinates must be finite");
        return Bool::FALSE;
    }
    if !finite_positive(wavenumber) {
        set_error(ERR_INVALID_ARGUMENT, "wavenumber must be positive and finite");
        return Bool::FALSE;
    }
    if !finite_non_negative(amplitude) {
        set_error(ERR_INVALID_ARGUMENT, "amplitude must be finite and non-negative");
        return Bool::FALSE;
    }

    let dx = obs_x - source_x;
    let dy = obs_y - source_y;
    let dz = obs_z - source_z;
    let r = (dx * dx + dy * dy + dz * dz).sqrt();

    if r < EPSILON {
        set_error(ERR_INVALID_ARGUMENT, "observation point coincides with source");
        return Bool::FALSE;
    }

    let phase = wavenumber * r;
    let amp = complex_from_polar(amplitude / r, phase);
    write_out(
        out_wave,
        SphericalWavePoint {
            amplitude: amp,
            radius: r,
            decay_factor: 1.0 / r,
        },
    )
}

// ===========================================================================
// C. Huygens–Fresnel principle (point-source superposition)
// ===========================================================================

/// Compute the field at an observation point from N point sources
/// using the Huygens–Fresnel superposition integral.
///
///   E(P) = Σ_j A_j · exp(i k r_j) / r_j
///
/// where r_j is the distance from source j to the observation point.
#[unsafe(no_mangle)]
pub extern "C" fn wo_huygens_fresnel(
    sources: *const PointSource,
    source_count: u32,
    obs_x: f64,
    obs_y: f64,
    obs_z: f64,
    wavenumber: f64,
    out_amplitude: *mut ComplexAmplitude,
) -> Bool {
    if sources.is_null() {
        set_error(ERR_NULL_POINTER, "sources pointer is null");
        return Bool::FALSE;
    }
    if source_count == 0 {
        set_error(ERR_INVALID_ARGUMENT, "source_count must be > 0");
        return Bool::FALSE;
    }
    if !finite(obs_x) || !finite(obs_y) || !finite(obs_z) {
        set_error(ERR_INVALID_ARGUMENT, "observation coordinates must be finite");
        return Bool::FALSE;
    }
    if !finite_positive(wavenumber) {
        set_error(ERR_INVALID_ARGUMENT, "wavenumber must be positive and finite");
        return Bool::FALSE;
    }

    let srcs = unsafe { std::slice::from_raw_parts(sources, source_count as usize) };
    let mut total = ComplexAmplitude::default();

    for src in srcs {
        if !finite(src.x) || !finite(src.y) || !finite(src.z) {
            continue;
        }
        let dx = obs_x - src.x;
        let dy = obs_y - src.y;
        let dz = obs_z - src.z;
        let r = (dx * dx + dy * dy + dz * dz).sqrt();
        if r < EPSILON {
            continue;
        }
        let amp = src.amplitude / r;
        let phase = wavenumber * r + src.phase;
        let contribution = complex_from_polar(amp, phase);
        total = complex_add(total, contribution);
    }

    write_out(out_amplitude, total)
}

// ===========================================================================
// D. Fresnel diffraction (near-field, parabolic approximation)
// ===========================================================================

/// Compute the Fresnel diffraction field at a single observation point from a
/// rectangular aperture, using the Fresnel (paraxial) approximation.
///
/// The Fresnel diffraction integral for a rectangular aperture:
///
///   E(x, y) ∝ ∫∫ A(ξ, η) · exp( i k / (2z) · [(x-ξ)² + (y-η)²] ) dξ dη
///
/// This simplified version assumes uniform illumination (A = 1) over the
/// aperture and performs a numerical Riemann sum over `samples_x × samples_y`
/// sub-divisions of the aperture.
#[unsafe(no_mangle)]
pub extern "C" fn wo_fresnel_diffraction_point(
    aperture: ApertureDesc,
    obs_x: f64,
    obs_y: f64,
    obs_z: f64,
    wavenumber: f64,
    samples_x: u32,
    samples_y: u32,
    out_point: *mut DiffractionPoint,
) -> Bool {
    if aperture.half_width_x <= 0.0 || aperture.half_width_y <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "aperture half-widths must be positive");
        return Bool::FALSE;
    }
    if !finite(aperture.transmission) || aperture.transmission < 0.0 || aperture.transmission > 1.0 {
        set_error(ERR_INVALID_ARGUMENT, "transmission must be in [0, 1]");
        return Bool::FALSE;
    }
    if !finite(obs_x) || !finite(obs_y) || !finite(obs_z) {
        set_error(ERR_INVALID_ARGUMENT, "observation coordinates must be finite");
        return Bool::FALSE;
    }
    if !finite_positive(wavenumber) {
        set_error(ERR_INVALID_ARGUMENT, "wavenumber must be positive and finite");
        return Bool::FALSE;
    }
    if obs_z <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "obs_z (distance) must be positive");
        return Bool::FALSE;
    }
    if samples_x == 0 || samples_y == 0 {
        set_error(ERR_INVALID_ARGUMENT, "samples must be > 0");
        return Bool::FALSE;
    }

    let tx = aperture.transmission;
    let hx = aperture.half_width_x;
    let hy = aperture.half_width_y;
    let cx = aperture.center_x;
    let cy = aperture.center_y;

    let dx = 2.0 * hx / (samples_x as f64);
    let dy = 2.0 * hy / (samples_y as f64);
    let z = obs_z;

    let mut total = ComplexAmplitude::default();

    for iy in 0..samples_y {
        let eta = cy - hy + (iy as f64 + 0.5) * dy;
        for ix in 0..samples_x {
            let xi = cx - hx + (ix as f64 + 0.5) * dx;

            let x_diff = obs_x - xi;
            let y_diff = obs_y - eta;
            let r2 = x_diff * x_diff + y_diff * y_diff + z * z;
            let r = r2.sqrt();

            // Fresnel approximation phase: k * (z + (x-ξ)²/(2z) + (y-η)²/(2z))
            let phase = wavenumber * (z + (x_diff * x_diff + y_diff * y_diff) / (2.0 * z));
            let amplitude = tx * dx * dy / r;
            let contrib = complex_from_polar(amplitude, phase);
            total = complex_add(total, contrib);
        }
    }

    write_out(
        out_point,
        DiffractionPoint {
            x: obs_x,
            y: obs_y,
            amplitude: total,
        },
    )
}

// ===========================================================================
// E. Fresnel–Kirchhoff diffraction integral
// ===========================================================================

/// Compute the Fresnel–Kirchhoff diffraction integral at a single observation
/// point, including the obliquity (inclination) factor.
///
///   E(P) = (1 / iλ) ∫∫ A(ξ,η) · exp(i k r) / r · cosθ dξ dη
///
/// where cosθ = z/r is the obliquity factor for normal incidence.
#[unsafe(no_mangle)]
pub extern "C" fn wo_kirchhoff_diffraction_point(
    aperture: ApertureDesc,
    obs_x: f64,
    obs_y: f64,
    obs_z: f64,
    wavenumber: f64,
    samples_x: u32,
    samples_y: u32,
    out_point: *mut KirchhoffDiffractionPoint,
) -> Bool {
    if aperture.half_width_x <= 0.0 || aperture.half_width_y <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "aperture half-widths must be positive");
        return Bool::FALSE;
    }
    if !finite(obs_x) || !finite(obs_y) || !finite(obs_z) {
        set_error(ERR_INVALID_ARGUMENT, "observation coordinates must be finite");
        return Bool::FALSE;
    }
    if !finite_positive(wavenumber) {
        set_error(ERR_INVALID_ARGUMENT, "wavenumber must be positive and finite");
        return Bool::FALSE;
    }
    if obs_z <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "obs_z must be positive");
        return Bool::FALSE;
    }
    if samples_x == 0 || samples_y == 0 {
        set_error(ERR_INVALID_ARGUMENT, "samples must be > 0");
        return Bool::FALSE;
    }

    let hx = aperture.half_width_x;
    let hy = aperture.half_width_y;
    let cx = aperture.center_x;
    let cy = aperture.center_y;
    let tx = aperture.transmission;

    let dx = 2.0 * hx / (samples_x as f64);
    let dy = 2.0 * hy / (samples_y as f64);
    let z = obs_z;
    let wavelength = TWO_PI / wavenumber;

    let mut total = ComplexAmplitude::default();

    for iy in 0..samples_y {
        let eta = cy - hy + (iy as f64 + 0.5) * dy;
        for ix in 0..samples_x {
            let xi = cx - hx + (ix as f64 + 0.5) * dx;

            let x_diff = obs_x - xi;
            let y_diff = obs_y - eta;
            let r2 = x_diff * x_diff + y_diff * y_diff + z * z;
            let r = r2.sqrt();

            if r < EPSILON {
                continue;
            }

            // Obliquity factor: cosθ = z / r
            let cos_theta = z / r;

            // Kirchhoff formula: E = (1 / iλ) · A · exp(ikr) / r · cosθ
            // 1/i = -i, so prefactor = -i / λ
            let amplitude = tx * dx * dy * cos_theta / r;
            let phase = wavenumber * r;

            // Multiply by -i: rotate phase by -π/2
            let real = amplitude * (phase - PI / 2.0).cos() / wavelength;
            let imag = amplitude * (phase - PI / 2.0).sin() / wavelength;

            total = complex_add(
                total,
                ComplexAmplitude {
                    real,
                    imag,
                    intensity: real * real + imag * imag,
                },
            );
        }
    }

    // Compute the average obliquity factor
    let avg_cos_theta = obs_z / (obs_x * obs_x + obs_y * obs_y + obs_z * obs_z).sqrt().max(EPSILON);

    write_out(
        out_point,
        KirchhoffDiffractionPoint {
            x: obs_x,
            y: obs_y,
            amplitude: total,
            obliquity_factor: avg_cos_theta,
        },
    )
}

// ===========================================================================
// F. Young's double-slit interference
// ===========================================================================

/// Compute the interference pattern from Young's double-slit experiment at a
/// single observation point on a distant screen.
///
/// Slits are at (±d/2, 0) in the aperture plane, screen at distance D.
/// Single-slit envelope (width a) is included.
///
/// Returns the normalised intensity:
///   I = I₀ · cos²(π d x / λ D) · sinc²(π a x / λ D)
#[unsafe(no_mangle)]
pub extern "C" fn wo_young_slit_point(
    slit_separation: f64,
    slit_width: f64,
    screen_distance: f64,
    wavelength: f64,
    obs_x: f64,
    obs_y: f64,
    out_point: *mut YoungSlitPoint,
) -> Bool {
    if !finite_positive(slit_separation) {
        set_error(ERR_INVALID_ARGUMENT, "slit_separation must be positive and finite");
        return Bool::FALSE;
    }
    if !finite_non_negative(slit_width) {
        set_error(ERR_INVALID_ARGUMENT, "slit_width must be non-negative and finite");
        return Bool::FALSE;
    }
    if !finite_positive(screen_distance) {
        set_error(ERR_INVALID_ARGUMENT, "screen_distance must be positive and finite");
        return Bool::FALSE;
    }
    if !finite_positive(wavelength) {
        set_error(ERR_INVALID_ARGUMENT, "wavelength must be positive and finite");
        return Bool::FALSE;
    }
    if !finite(obs_x) || !finite(obs_y) {
        set_error(ERR_INVALID_ARGUMENT, "observation coordinates must be finite");
        return Bool::FALSE;
    }

    let angle = obs_x / screen_distance; // small-angle approximation: sinθ ≈ tanθ ≈ x/D
    let k = TWO_PI / wavelength;

    // Path difference: Δ = d · sinθ ≈ d · x / D
    let path_difference = slit_separation * angle;
    let phase_difference = k * path_difference;

    // Interference term: cos²(δ/2)
    let interference = (phase_difference * 0.5).cos().powi(2);

    // Single-slit envelope: sinc²(β) where β = π a sinθ / λ
    let envelope_factor = if slit_width > EPSILON {
        let beta = PI * slit_width * angle / wavelength;
        if beta.abs() > EPSILON {
            let sinc = beta.sin() / beta;
            sinc * sinc
        } else {
            1.0
        }
    } else {
        1.0
    };

    let intensity = interference * envelope_factor;

    write_out(
        out_point,
        YoungSlitPoint {
            x: obs_x,
            y: obs_y,
            phase_difference,
            path_difference,
            intensity,
            envelope_factor,
        },
    )
}

/// Compute the Young's interference pattern across a 1D array of points
/// (along the x-axis) and write intensities into a pre-allocated buffer.
#[unsafe(no_mangle)]
pub extern "C" fn wo_young_slit_pattern(
    slit_separation: f64,
    slit_width: f64,
    screen_distance: f64,
    wavelength: f64,
    x_min: f64,
    x_max: f64,
    num_points: u32,
    out_intensities: *mut f64,
    out_len: u32,
) -> u32 {
    if !finite_positive(slit_separation)
        || !finite_non_negative(slit_width)
        || !finite_positive(screen_distance)
        || !finite_positive(wavelength)
    {
        return 0;
    }
    if !finite(x_min) || !finite(x_max) || x_min > x_max {
        set_error(ERR_INVALID_ARGUMENT, "x_min <= x_max and both finite");
        return 0;
    }
    if num_points < 2 {
        clear_error();
        return 0;
    }
    if out_intensities.is_null() {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return 0;
    }

    let cap = out_len as usize;
    let count = (num_points as usize).min(cap);
    let buf = unsafe { std::slice::from_raw_parts_mut(out_intensities, count) };

    for (i, buf_item) in buf.iter_mut().enumerate().take(count) {
        let frac = i as f64 / (count - 1).max(1) as f64;
        let x = x_min + frac * (x_max - x_min);
        let mut point = YoungSlitPoint::default();
        let _ = wo_young_slit_point(
            slit_separation, slit_width, screen_distance, wavelength,
            x, 0.0, &mut point,
        );
        *buf_item = point.intensity;
    }

    clear_error();
    count as u32
}

// ===========================================================================
// G. Thin-film interference (rainbow colours)
// ===========================================================================

/// Compute thin-film interference for a single layer.
///
/// Optical path difference (normal incidence): OPD = 2 n_film t cos θ_t
/// where θ_t is the transmission angle (from Snell's law).
///
/// Phase difference: δ = (2π/λ) · OPD + π (if half-wave loss occurs)
///
/// Half-wave loss occurs when n_film > n_incident or n_film > n_substrate
/// (reflection off a higher-index medium).
///
/// Interference intensity: I = I₀ · [1 + cos(δ)] / 2  (simplified)
#[unsafe(no_mangle)]
pub extern "C" fn wo_thin_film_interference(
    params: ThinFilmParams,
    wavelength: f64,
    out_report: *mut ThinFilmInterferenceReport,
) -> Bool {
    if !finite_positive(params.thickness)
        || !finite_positive(params.n_film)
        || !finite_positive(params.n_incident)
        || !finite_positive(params.n_substrate)
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "all thin-film parameters must be positive and finite",
        );
        return Bool::FALSE;
    }
    if !finite(params.incidence_angle) {
        set_error(ERR_INVALID_ARGUMENT, "incidence_angle must be finite");
        return Bool::FALSE;
    }
    if !finite_positive(wavelength) {
        set_error(ERR_INVALID_ARGUMENT, "wavelength must be positive and finite");
        return Bool::FALSE;
    }

    // Snell's law: n_incident · sin(θ_i) = n_film · sin(θ_t)
    let sin_theta_i = params.incidence_angle.sin().abs();
    let sin_theta_t = if params.n_film > EPSILON {
        (params.n_incident * sin_theta_i / params.n_film).min(1.0)
    } else {
        0.0
    };
    let cos_theta_t = (1.0 - sin_theta_t * sin_theta_t).sqrt().max(0.0);

    // Optical path difference: OPD = 2 n_film t cos θ_t
    let opd = 2.0 * params.n_film * params.thickness * cos_theta_t;

    // Half-wave loss: occurs when reflecting off a higher-index medium
    // Top surface: n_film > n_incident → π phase shift
    // Bottom surface: n_substrate > n_film → π phase shift
    let top_shift = params.n_film > params.n_incident;
    let bottom_shift = params.n_substrate > params.n_film;
    let half_wave_loss = top_shift != bottom_shift; // net π shift if only one interface has it

    let extra_phase = if half_wave_loss { PI } else { 0.0 };

    // Total phase difference
    let phase_difference = TWO_PI * opd / wavelength + extra_phase;

    // Reflection coefficient (simplified Fresnel, normal incidence approximation)
    let r1 = (params.n_incident - params.n_film) / (params.n_incident + params.n_film);
    let r2 = (params.n_film - params.n_substrate) / (params.n_film + params.n_substrate);
    let reflection_coefficient = (r1.abs() + r2.abs()) * 0.5;

    // Interference intensity: I = I₀ · (1 + cos δ) / 2
    let intensity = 0.5 * (1.0 + phase_difference.cos());

    write_out(
        out_report,
        ThinFilmInterferenceReport {
            opd,
            phase_difference,
            reflection_coefficient,
            intensity,
            half_wave_loss: Bool::from(half_wave_loss),
            wavelength,
        },
    )
}

/// Compute thin-film interference for multiple wavelengths (rainbow spectrum).
///
/// `wavelengths` — pointer to array of wavelengths (m).
/// `intensities_out` — pre-allocated buffer for output intensities.
/// `count` — number of wavelengths.
///
/// Returns the number of intensities written.
#[unsafe(no_mangle)]
pub extern "C" fn wo_thin_film_spectrum(
    params: ThinFilmParams,
    wavelengths: *const f64,
    intensities_out: *mut f64,
    count: u32,
) -> u32 {
    if wavelengths.is_null() || intensities_out.is_null() {
        set_error(ERR_NULL_POINTER, "pointer is null");
        return 0;
    }
    if count == 0 {
        clear_error();
        return 0;
    }
    if !finite_positive(params.thickness) || !finite_positive(params.n_film)
        || !finite_positive(params.n_incident) || !finite_positive(params.n_substrate)
    {
        set_error(ERR_INVALID_ARGUMENT, "thin-film params must be positive");
        return 0;
    }

    let waves = unsafe { std::slice::from_raw_parts(wavelengths, count as usize) };
    let out = unsafe { std::slice::from_raw_parts_mut(intensities_out, count as usize) };

    let mut written = 0u32;
    for i in 0..count as usize {
        if !finite_positive(waves[i]) {
            out[i] = 0.0;
            continue;
        }
        let mut report = ThinFilmInterferenceReport::default();
        let result = wo_thin_film_interference(params, waves[i], &mut report);
        if result == Bool::TRUE {
            out[i] = report.intensity;
        } else {
            out[i] = 0.0;
        }
        written += 1;
    }

    clear_error();
    written
}

// ===========================================================================
// H. Fresnel zones
// ===========================================================================

/// Compute the radius of the n-th Fresnel zone for a point at distance D
/// from the aperture plane and wavelength λ.
///
///   r_n = √(n λ D)
///
/// Also determines whether the zone contributes constructively.
#[unsafe(no_mangle)]
pub extern "C" fn wo_fresnel_zone(
    zone_index: u32,
    distance: f64,
    wavelength: f64,
    out_zone: *mut FresnelZoneReport,
) -> Bool {
    if zone_index == 0 {
        set_error(ERR_INVALID_ARGUMENT, "zone_index must be >= 1");
        return Bool::FALSE;
    }
    if !finite_positive(distance) {
        set_error(ERR_INVALID_ARGUMENT, "distance must be positive and finite");
        return Bool::FALSE;
    }
    if !finite_positive(wavelength) {
        set_error(ERR_INVALID_ARGUMENT, "wavelength must be positive and finite");
        return Bool::FALSE;
    }

    let n = zone_index as f64;
    let zone_radius = (n * wavelength * distance).sqrt();
    let zone_phase = PI * n; // each successive zone alternates phase by π
    let constructive = (zone_phase % TWO_PI - PI).abs() < PI / 2.0;

    write_out(
        out_zone,
        FresnelZoneReport {
            zone_radius,
            zone_index,
            zone_phase,
            constructive: Bool::from(constructive),
        },
    )
}

/// Compute the sum of contributions from the first N Fresnel zones
/// (simplified phasor sum).
///
/// `num_zones` — number of zones to sum.
/// `out_intensity` — normalised intensity after summing N zones.
#[unsafe(no_mangle)]
pub extern "C" fn wo_fresnel_zone_sum(
    num_zones: u32,
    distance: f64,
    wavelength: f64,
    out_intensity: *mut f64,
) -> Bool {
    if num_zones == 0 {
        set_error(ERR_INVALID_ARGUMENT, "num_zones must be >= 1");
        return Bool::FALSE;
    }
    if !finite_positive(distance) || !finite_positive(wavelength) {
        set_error(ERR_INVALID_ARGUMENT, "distance and wavelength must be positive");
        return Bool::FALSE;
    }

    let mut phasor = ComplexAmplitude::default();

    for n in 1..=num_zones {
        let mut zone = FresnelZoneReport::default();
        let _ = wo_fresnel_zone(n, distance, wavelength, &mut zone);

        // Each zone contributes approximately the same amplitude but alternating sign
        let amp_mag = 1.0 / (num_zones as f64);
        let phase = PI * (n as f64);
        let contrib = complex_from_polar(amp_mag, phase);
        phasor = complex_add(phasor, contrib);
    }

    write_out(out_intensity, phasor.intensity)
}

// ===========================================================================
// I. Interference pattern caching (grid sampling)
// ===========================================================================

/// Sample the Fresnel diffraction pattern on a regular 2D grid in the
/// observation plane.
///
/// `nx` × `ny` points spanning `extent_x` × `extent_y` around the optical axis.
/// Results are written into `out_grid` (array of `DiffractionPoint`, capacity `out_len`).
///
/// Returns the number of points written.
#[unsafe(no_mangle)]
pub extern "C" fn wo_fresnel_grid(
    aperture: ApertureDesc,
    screen_distance: f64,
    wavenumber: f64,
    nx: u32,
    ny: u32,
    extent_x: f64,
    extent_y: f64,
    samples_x: u32,
    samples_y: u32,
    out_grid: *mut DiffractionPoint,
    out_len: u32,
) -> u32 {
    if nx == 0 || ny == 0 {
        clear_error();
        return 0;
    }
    if !finite_positive(extent_x) || !finite_positive(extent_y) {
        return 0;
    }
    if out_grid.is_null() {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return 0;
    }

    let total = (nx as usize).saturating_mul(ny as usize);
    let cap = out_len as usize;
    let count = total.min(cap);
    let buf = unsafe { std::slice::from_raw_parts_mut(out_grid, count) };

    let mut idx = 0usize;
    for iy in 0..ny {
        let fy = (iy as f64) / ((ny - 1).max(1) as f64) - 0.5;
        for ix in 0..nx {
            if idx >= count {
                break;
            }
            let fx = (ix as f64) / ((nx - 1).max(1) as f64) - 0.5;
            let obs_x = fx * extent_x;
            let obs_y = fy * extent_y;

            let mut point = DiffractionPoint::default();
            let _ = wo_fresnel_diffraction_point(
                aperture, obs_x, obs_y, screen_distance, wavenumber,
                samples_x, samples_y, &mut point,
            );
            buf[idx] = point;
            idx += 1;
        }
    }

    clear_error();
    idx as u32
}

