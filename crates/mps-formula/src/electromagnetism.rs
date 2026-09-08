use std::f64::consts::PI;

use crate::error::{ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error};
use crate::ffi::{
    Bool, ElectromagneticField, FaradayInductionReport, FdtdYeeReport, LorentzForceReport,
    MagneticFluxReport, MaxwellPointReport, Vec3, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

use crate::math::{
    EPS_GENERAL as EPSILON, KahanSum, finite_many, finite_non_negative, finite_positive,
};

const VACUUM_PERMITTIVITY: f64 = 8.854_187_812_8e-12;
const VACUUM_PERMEABILITY: f64 = 1.256_637_062_12e-6;
const MAX_FIELD_CELLS: u32 = 2_000_000;

fn field_valid(field: ElectromagneticField) -> bool {
    vec3_finite(field.electric) && vec3_finite(field.magnetic)
}

/// Compute the Lorentz force F = q(E + v x B) and the resulting acceleration.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `LorentzForceReport`;
/// a null pointer fails with `ERR_NULL_POINTER`. No ownership is transferred.
#[unsafe(no_mangle)]
pub extern "C" fn em_lorentz_force(
    charge: f64,
    velocity: Vec3,
    field: ElectromagneticField,
    mass: f64,
    out_report: *mut LorentzForceReport,
) -> Bool {
    if !charge.is_finite()
        || !vec3_finite(velocity)
        || !field_valid(field)
        || !finite_non_negative(mass)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Lorentz force parameters");
        return Bool::FALSE;
    }
    let v = vec3_to_rapier(velocity);
    let electric = vec3_to_rapier(field.electric);
    let magnetic = vec3_to_rapier(field.magnetic);
    let force = (electric + v.cross(magnetic)) * charge;
    let acceleration = if mass > EPSILON {
        force / mass
    } else {
        crate::math::Vector3f64::zeros()
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Lorentz force output is null");
        return Bool::FALSE;
    };
    *out_report = LorentzForceReport {
        electric_force: vec3_from_rapier(electric * charge),
        magnetic_force: vec3_from_rapier(v.cross(magnetic) * charge),
        total_force: vec3_from_rapier(force),
        acceleration: vec3_from_rapier(acceleration),
    };
    clear_error();
    Bool::TRUE
}

/// Compute the magnetic flux Phi = B . n_hat * A through a planar surface.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `MagneticFluxReport`;
/// a null pointer fails with `ERR_NULL_POINTER`. No ownership is transferred.
#[unsafe(no_mangle)]
pub extern "C" fn em_magnetic_flux(
    magnetic_field: Vec3,
    area_normal: Vec3,
    area: f64,
    out_report: *mut MagneticFluxReport,
) -> Bool {
    if !vec3_finite(magnetic_field) || !vec3_finite(area_normal) || !finite_non_negative(area) {
        set_error(ERR_INVALID_ARGUMENT, "invalid magnetic flux parameters");
        return Bool::FALSE;
    }
    let b = vec3_to_rapier(magnetic_field);
    let n = vec3_to_rapier(area_normal);
    let normal_len = n.length();
    if normal_len <= EPSILON {
        set_error(ERR_INVALID_ARGUMENT, "magnetic flux normal is zero");
        return Bool::FALSE;
    }
    let unit_normal = n / normal_len;
    let normal_component = b.dot(unit_normal);
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "magnetic flux output is null");
        return Bool::FALSE;
    };
    *out_report = MagneticFluxReport {
        flux: normal_component * area,
        normal_component,
        area,
    };
    clear_error();
    Bool::TRUE
}

/// Compute Faraday induction (flux rate, induced EMF, induced current).
///
/// # Safety
///
/// `out_report` must point to writable memory for one `FaradayInductionReport`;
/// a null pointer fails with `ERR_NULL_POINTER`. No ownership is transferred.
#[unsafe(no_mangle)]
pub extern "C" fn em_faraday_induction(
    previous_flux: f64,
    current_flux: f64,
    dt: f64,
    turns: f64,
    resistance: f64,
    out_report: *mut FaradayInductionReport,
) -> Bool {
    if !previous_flux.is_finite()
        || !current_flux.is_finite()
        || !finite_positive(dt)
        || !finite_non_negative(turns)
        || !finite_non_negative(resistance)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Faraday induction parameters");
        return Bool::FALSE;
    }
    let flux_rate = (current_flux - previous_flux) / dt;
    let induced_emf = -turns * flux_rate;
    let induced_current = if resistance > EPSILON {
        induced_emf / resistance
    } else {
        0.0
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Faraday induction output is null");
        return Bool::FALSE;
    };
    *out_report = FaradayInductionReport {
        flux_rate,
        induced_emf,
        induced_current,
    };
    clear_error();
    Bool::TRUE
}

/// Advance an electromagnetic field at a single point by `dt` via Maxwell's equations.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `MaxwellPointReport`;
/// a null pointer fails with `ERR_NULL_POINTER`. No ownership is transferred.
#[unsafe(no_mangle)]
pub extern "C" fn em_maxwell_point_update(
    field: ElectromagneticField,
    curl_electric: Vec3,
    curl_magnetic: Vec3,
    current_density: Vec3,
    charge_density: f64,
    divergence_electric: f64,
    divergence_magnetic: f64,
    permittivity: f64,
    permeability: f64,
    dt: f64,
    out_report: *mut MaxwellPointReport,
) -> Bool {
    if !field_valid(field)
        || !vec3_finite(curl_electric)
        || !vec3_finite(curl_magnetic)
        || !vec3_finite(current_density)
        || !charge_density.is_finite()
        || !divergence_electric.is_finite()
        || !divergence_magnetic.is_finite()
        || !finite_positive(permittivity)
        || !finite_positive(permeability)
        || !finite_non_negative(dt)
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid Maxwell point update parameters",
        );
        return Bool::FALSE;
    }

    let e = vec3_to_rapier(field.electric);
    let b = vec3_to_rapier(field.magnetic);
    let curl_e = vec3_to_rapier(curl_electric);
    let curl_b = vec3_to_rapier(curl_magnetic);
    let j = vec3_to_rapier(current_density);
    let electric_derivative = curl_b / (permittivity * permeability) - j / permittivity;
    let magnetic_derivative = -curl_e;
    let next_electric = e + electric_derivative * dt;
    let next_magnetic = b + magnetic_derivative * dt;

    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Maxwell point output is null");
        return Bool::FALSE;
    };
    *out_report = MaxwellPointReport {
        next_field: ElectromagneticField {
            electric: vec3_from_rapier(next_electric),
            magnetic: vec3_from_rapier(next_magnetic),
        },
        electric_derivative: vec3_from_rapier(electric_derivative),
        magnetic_derivative: vec3_from_rapier(magnetic_derivative),
        gauss_electric_residual: divergence_electric - charge_density / permittivity,
        gauss_magnetic_residual: divergence_magnetic,
    };
    clear_error();
    Bool::TRUE
}

/// Advance an FDTD Yee grid of `cell_count` cells by `dt`.
///
/// # Safety
///
/// `electric_fields`, `magnetic_fields`, `curl_electric` and `curl_magnetic`
/// must each point to at least `cell_count` readable `Vec3` elements;
/// `out_electric_fields` and `out_magnetic_fields` must each point to writable
/// memory for `capacity` `Vec3` elements (only the first `cell_count` are
/// written). Requires `0 < cell_count <= MAX_FIELD_CELLS` (2_000_000) and
/// `capacity >= cell_count`, failing with `ERR_CAPACITY` otherwise; any null
/// grid pointer fails with `ERR_NULL_POINTER`. `out_report` may be null, in
/// which case the report is skipped. No ownership is transferred.
#[unsafe(no_mangle)]
pub extern "C" fn em_fdtd_yee_update(
    electric_fields: *const Vec3,
    magnetic_fields: *const Vec3,
    curl_electric: *const Vec3,
    curl_magnetic: *const Vec3,
    cell_count: u32,
    permittivity: f64,
    permeability: f64,
    conductivity: f64,
    dt: f64,
    out_electric_fields: *mut Vec3,
    out_magnetic_fields: *mut Vec3,
    capacity: u32,
    out_report: *mut FdtdYeeReport,
) -> Bool {
    if cell_count == 0 || cell_count > MAX_FIELD_CELLS || capacity < cell_count {
        set_error(ERR_CAPACITY, "invalid FDTD Yee grid capacity");
        return Bool::FALSE;
    }
    if electric_fields.is_null()
        || magnetic_fields.is_null()
        || curl_electric.is_null()
        || curl_magnetic.is_null()
        || out_electric_fields.is_null()
        || out_magnetic_fields.is_null()
    {
        set_error(ERR_NULL_POINTER, "FDTD Yee grid pointers are null");
        return Bool::FALSE;
    }
    if !finite_positive(permittivity)
        || !finite_positive(permeability)
        || !finite_non_negative(conductivity)
        || !finite_non_negative(dt)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid FDTD Yee grid parameters");
        return Bool::FALSE;
    }

    macro_rules! input { ($p:expr, $name:expr) => { match unsafe { crate::ffi::checked_input_slice($p, cell_count as usize, $name) } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } } } }
    let electric_fields = input!(electric_fields, "electric_fields");
    let magnetic_fields = input!(magnetic_fields, "magnetic_fields");
    let curl_electric = input!(curl_electric, "curl_electric");
    let curl_magnetic = input!(curl_magnetic, "curl_magnetic");
    let out_electric = match unsafe { crate::ffi::checked_output_slice(out_electric_fields, capacity as usize, "out_electric_fields") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    let out_magnetic = match unsafe { crate::ffi::checked_output_slice(out_magnetic_fields, capacity as usize, "out_magnetic_fields") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };

    let mut max_electric_delta = 0.0;
    let mut max_magnetic_delta = 0.0;
    let mut total_energy_acc = KahanSum::default();
    for index in 0..cell_count as usize {
        if !vec3_finite(electric_fields[index])
            || !vec3_finite(magnetic_fields[index])
            || !vec3_finite(curl_electric[index])
            || !vec3_finite(curl_magnetic[index])
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid FDTD Yee grid cell");
            return Bool::FALSE;
        }
        let e = vec3_to_rapier(electric_fields[index]);
        let b = vec3_to_rapier(magnetic_fields[index]);
        let e_delta = (vec3_to_rapier(curl_magnetic[index]) / (permittivity * permeability)
            - e * (conductivity / permittivity))
            * dt;
        let b_delta = -vec3_to_rapier(curl_electric[index]) * dt;
        let next_e = e + e_delta;
        let next_b = b + b_delta;
        out_electric[index] = vec3_from_rapier(next_e);
        out_magnetic[index] = vec3_from_rapier(next_b);
        max_electric_delta = f64::max(max_electric_delta, e_delta.length());
        max_magnetic_delta = f64::max(max_magnetic_delta, b_delta.length());
        total_energy_acc.add(
            0.5 * permittivity * next_e.length_squared()
                + 0.5 * next_b.length_squared() / permeability,
        );
    }

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = FdtdYeeReport {
            cell_count,
            max_electric_delta,
            max_magnetic_delta,
            total_energy_density: total_energy_acc.value(),
            courant_number: dt / (permittivity * permeability).sqrt(),
        };
    }
    clear_error();
    Bool::TRUE
}

/// Return the vacuum permittivity epsilon0 (F/m).
///
/// # Safety
///
/// This function takes no pointers and has no safety preconditions.
#[unsafe(no_mangle)]
pub extern "C" fn em_vacuum_permittivity() -> f64 {
    VACUUM_PERMITTIVITY
}

/// Return the vacuum permeability mu0 (H/m).
///
/// # Safety
///
/// This function takes no pointers and has no safety preconditions.
#[unsafe(no_mangle)]
pub extern "C" fn em_vacuum_permeability() -> f64 {
    VACUUM_PERMEABILITY
}

// ---------------------------------------------------------------------------
// Biot-Savart law
// ---------------------------------------------------------------------------

/// Biot-Savart law: dB = (mu0/4pi) * I * dl x r_hat / r^2
/// Returns the magnetic field contribution at `point` from a current element.
pub fn biot_savart_element(current: f64, dl: Vec3, position: Vec3, point: Vec3) -> Option<Vec3> {
    let mu0 = 1.25663706212e-6;
    if !vec3_finite(dl) || !vec3_finite(position) || !vec3_finite(point) || !current.is_finite() {
        return None;
    }
    let r_vec = vec3_to_rapier(point) - vec3_to_rapier(position);
    let r = r_vec.length();
    if r < 1.0e-12 {
        return None;
    }
    let r_hat = r_vec / r;
    let cross = vec3_to_rapier(dl).cross(r_hat);
    let factor = mu0 * current / (4.0 * PI * r * r);
    Some(vec3_from_rapier(cross * factor))
}

/// Biot-Savart law for a finite straight wire segment.
/// Returns B at `point` from wire from `p1` to `p2` carrying current.
pub fn biot_savart_wire_segment(current: f64, p1: Vec3, p2: Vec3, point: Vec3) -> Option<Vec3> {
    let mu0 = 1.25663706212e-6;
    if !finite_many(&[current, p1.x, p1.y, p1.z, p2.x, p2.y]) || !vec3_finite(point) {
        return None;
    }
    let a = vec3_to_rapier(p1);
    let b = vec3_to_rapier(p2);
    let p = vec3_to_rapier(point);
    let l = b - a;
    let l_len = l.length();
    if l_len < 1.0e-12 {
        return None;
    }
    let r1 = p - a;
    let r2 = p - b;
    let r1_len = r1.length();
    let r2_len = r2.length();
    if r1_len < 1.0e-12 || r2_len < 1.0e-12 {
        return None;
    }
    let l_hat = l / l_len;
    let sin_theta1 = l_hat.cross(r1 / r1_len).length();
    let sin_theta2 = l_hat.cross(r2 / r2_len).length();
    let direction = l_hat.cross(r1 / r1_len).try_normalize()?;
    let factor = mu0 * current / (4.0 * PI);
    let term = (1.0 / r1_len + 1.0 / r2_len) * (sin_theta1 + sin_theta2) / 2.0;
    Some(vec3_from_rapier(direction * factor * term))
}

// ---------------------------------------------------------------------------
// Poynting vector
// ---------------------------------------------------------------------------

/// Poynting vector: S = E x H (W/m^2) where H = B / mu0
pub fn poynting_vector(e: Vec3, b: Vec3) -> Option<Vec3> {
    let mu0 = 1.25663706212e-6;
    if !vec3_finite(e) || !vec3_finite(b) {
        return None;
    }
    let e_v = vec3_to_rapier(e);
    let b_v = vec3_to_rapier(b);
    let s = e_v.cross(b_v) / mu0;
    Some(vec3_from_rapier(s))
}

/// Poynting vector magnitude for plane wave: |S| = |E|^2 / (mu0 * c)
pub fn poynting_magnitude_plane_wave(e_field_magnitude: f64) -> Option<f64> {
    let c = 299_792_458.0;
    let mu0 = 1.25663706212e-6;
    if !e_field_magnitude.is_finite() || e_field_magnitude < 0.0 {
        return None;
    }
    Some(e_field_magnitude * e_field_magnitude / (mu0 * c))
}

// ---------------------------------------------------------------------------
// EM wave propagation
// ---------------------------------------------------------------------------

/// Phase velocity in medium: v = c / n
pub fn phase_velocity(refractive_index: f64) -> Option<f64> {
    let c = 299_792_458.0;
    if !refractive_index.is_finite() || refractive_index <= 0.0 {
        return None;
    }
    Some(c / refractive_index)
}

/// Wavelength: lambda = c / (n * f)
pub fn wavelength_in_medium(frequency: f64, refractive_index: f64) -> Option<f64> {
    let c = 299_792_458.0;
    if !frequency.is_finite()
        || frequency <= 0.0
        || !refractive_index.is_finite()
        || refractive_index <= 0.0
    {
        return None;
    }
    Some(c / (refractive_index * frequency))
}

/// Intrinsic impedance of medium: eta = sqrt(mu / epsilon)
pub fn intrinsic_impedance(permeability: f64, permittivity: f64) -> Option<f64> {
    if !permeability.is_finite()
        || permeability <= 0.0
        || !permittivity.is_finite()
        || permittivity <= 0.0
    {
        return None;
    }
    Some((permeability / permittivity).sqrt())
}

/// Skin depth: delta = 1 / sqrt(pi * f * mu * sigma)
pub fn skin_depth(frequency: f64, permeability: f64, conductivity: f64) -> Option<f64> {
    if !frequency.is_finite()
        || frequency <= 0.0
        || !permeability.is_finite()
        || permeability <= 0.0
        || !conductivity.is_finite()
        || conductivity <= 0.0
    {
        return None;
    }
    Some(1.0 / (PI * frequency * permeability * conductivity).sqrt())
}

/// EM wave vacuum wavelength: lambda = c / f
pub fn vacuum_wavelength(frequency: f64) -> Option<f64> {
    let c = 299_792_458.0;
    if !frequency.is_finite() || frequency <= 0.0 {
        return None;
    }
    Some(c / frequency)
}

/// EM wave frequency: f = c / lambda
pub fn wave_frequency(wavelength: f64) -> Option<f64> {
    let c = 299_792_458.0;
    if !wavelength.is_finite() || wavelength <= 0.0 {
        return None;
    }
    Some(c / wavelength)
}

// ---------------------------------------------------------------------------
// Antenna radiation
// ---------------------------------------------------------------------------

/// Radiation resistance of a short dipole: R_r = 80π² (L/λ)²
pub fn dipole_radiation_resistance(dipole_length: f64, wavelength: f64) -> Option<f64> {
    if !dipole_length.is_finite()
        || !wavelength.is_finite()
        || dipole_length < 0.0
        || wavelength <= 0.0
    {
        return None;
    }
    Some(80.0 * std::f64::consts::PI * std::f64::consts::PI * (dipole_length / wavelength).powi(2))
}

/// Half-wave dipole directivity: D = 1.64
pub fn half_wave_dipole_directivity() -> f64 {
    1.64
}

/// Effective aperture from gain: A_e = G · λ² / (4π)
pub fn effective_aperture(gain_linear: f64, wavelength: f64) -> Option<f64> {
    if !gain_linear.is_finite()
        || gain_linear <= 0.0
        || !wavelength.is_finite()
        || wavelength <= 0.0
    {
        return None;
    }
    Some(gain_linear * wavelength * wavelength / (4.0 * std::f64::consts::PI))
}

/// Far-field distance (Fraunhofer): r = 2D²/λ where D is the largest antenna dimension.
pub fn far_field_distance(antenna_size: f64, wavelength: f64) -> Option<f64> {
    if !antenna_size.is_finite()
        || antenna_size <= 0.0
        || !wavelength.is_finite()
        || wavelength <= 0.0
    {
        return None;
    }
    Some(2.0 * antenna_size * antenna_size / wavelength)
}

/// Friis transmission equation (power): P_r = P_t · G_t · G_r · (λ/(4πR))²
pub fn friis_power_received(
    transmit_power: f64,
    tx_gain: f64,
    rx_gain: f64,
    wavelength: f64,
    range: f64,
) -> Option<f64> {
    if !transmit_power.is_finite()
        || !tx_gain.is_finite()
        || !rx_gain.is_finite()
        || !wavelength.is_finite()
        || !range.is_finite()
    {
        return None;
    }
    if transmit_power < 0.0 || tx_gain < 0.0 || rx_gain < 0.0 || wavelength <= 0.0 || range <= 0.0 {
        return None;
    }
    Some(
        transmit_power
            * tx_gain
            * rx_gain
            * (wavelength / (4.0 * std::f64::consts::PI * range)).powi(2),
    )
}

// ---------------------------------------------------------------------------
// Impedance matching and transmission line
// ---------------------------------------------------------------------------

/// Reflection coefficient: Γ = (Z_L - Z_0) / (Z_L + Z_0)
pub fn reflection_coefficient(load_impedance: f64, characteristic_impedance: f64) -> Option<f64> {
    if !load_impedance.is_finite()
        || !characteristic_impedance.is_finite()
        || characteristic_impedance <= 0.0
    {
        return None;
    }
    let gamma =
        (load_impedance - characteristic_impedance) / (load_impedance + characteristic_impedance);
    Some(gamma)
}

/// Voltage standing wave ratio: VSWR = (1+|Γ|)/(1-|Γ|)
pub fn vswr(reflection_coeff: f64) -> Option<f64> {
    if !reflection_coeff.is_finite() || reflection_coeff.abs() >= 1.0 {
        return None;
    }
    Some((1.0 + reflection_coeff.abs()) / (1.0 - reflection_coeff.abs()))
}

/// Return loss: RL = -20 log₁₀ |Γ| (dB)
pub fn return_loss(reflection_coeff: f64) -> Option<f64> {
    if !reflection_coeff.is_finite()
        || reflection_coeff.abs() <= 0.0
        || reflection_coeff.abs() >= 1.0
    {
        return None;
    }
    Some(-20.0 * reflection_coeff.abs().log10())
}

/// Quarter-wave transformer impedance: Z_q = sqrt(Z_0 · Z_L)
pub fn quarter_wave_transformer(z0: f64, z_load: f64) -> Option<f64> {
    if !z0.is_finite() || z0 <= 0.0 || !z_load.is_finite() || z_load <= 0.0 {
        return None;
    }
    Some((z0 * z_load).sqrt())
}

/// Transmission line input impedance: Z_in = Z_0 · (Z_L + j·Z_0·tan(βl)) / (Z_0 + j·Z_L·tan(βl))
/// Returns (real, imag) for lossless case.
pub fn transmission_line_input_impedance(
    z0: f64,
    z_load_real: f64,
    z_load_imag: f64,
    phase_constant: f64,
    length: f64,
) -> Option<(f64, f64)> {
    if !z0.is_finite()
        || z0 <= 0.0
        || !z_load_real.is_finite()
        || !z_load_imag.is_finite()
        || !phase_constant.is_finite()
        || !length.is_finite()
    {
        return None;
    }
    let tan_bl = (phase_constant * length).tan();
    let num_real = z_load_real;
    let num_imag = z_load_imag + z0 * tan_bl;
    let den_real = z0 - z_load_imag * tan_bl;
    let den_imag = z_load_real * tan_bl;
    let den_sq = den_real * den_real + den_imag * den_imag;
    if den_sq <= 0.0 {
        return None;
    }
    let z_in_real = z0 * (num_real * den_real + num_imag * den_imag) / den_sq;
    let z_in_imag = z0 * (num_imag * den_real - num_real * den_imag) / den_sq;
    Some((z_in_real, z_in_imag))
}

// ---------------------------------------------------------------------------
// Coaxial cable
// ---------------------------------------------------------------------------

/// Coaxial cable characteristic impedance: Z_0 = (60/√ε_r) · ln(D/d)
pub fn coaxial_impedance(
    inner_diameter: f64,
    outer_diameter: f64,
    relative_permittivity: f64,
) -> Option<f64> {
    if !inner_diameter.is_finite()
        || !outer_diameter.is_finite()
        || !relative_permittivity.is_finite()
    {
        return None;
    }
    if inner_diameter <= 0.0 || outer_diameter <= inner_diameter || relative_permittivity <= 0.0 {
        return None;
    }
    Some(60.0 / relative_permittivity.sqrt() * (outer_diameter / inner_diameter).ln())
}

/// Coaxial cable cutoff frequency (TE11 mode): f_c ≈ c/(π·(D+d)/2 · √ε_r)
pub fn coaxial_cutoff_frequency(
    inner_diameter: f64,
    outer_diameter: f64,
    relative_permittivity: f64,
) -> Option<f64> {
    if !inner_diameter.is_finite()
        || !outer_diameter.is_finite()
        || !relative_permittivity.is_finite()
    {
        return None;
    }
    if inner_diameter <= 0.0 || outer_diameter <= inner_diameter || relative_permittivity <= 0.0 {
        return None;
    }
    let c = 299_792_458.0;
    let mean_diameter = 0.5 * (inner_diameter + outer_diameter);
    Some(c / (std::f64::consts::PI * mean_diameter * relative_permittivity.sqrt()))
}

// ---------------------------------------------------------------------------
// Scattering
// ---------------------------------------------------------------------------

/// Rayleigh scattering cross-section for a small dielectric sphere.
/// σ_s = (8π³/3) · ((n²-1)/(n²+2))² · (d/2)⁶ / λ⁴
pub fn rayleigh_scattering_cross_section(
    refractive_index: f64,
    diameter: f64,
    wavelength: f64,
) -> Option<f64> {
    if !refractive_index.is_finite() || !diameter.is_finite() || !wavelength.is_finite() {
        return None;
    }
    if refractive_index <= 0.0 || diameter <= 0.0 || wavelength <= 0.0 {
        return None;
    }
    let r = diameter / 2.0;
    let polarizability =
        (refractive_index * refractive_index - 1.0) / (refractive_index * refractive_index + 2.0);
    Some(
        8.0 * std::f64::consts::PI.powi(3) / 3.0 * polarizability.powi(2) * r.powi(6)
            / wavelength.powi(4),
    )
}

/// Faraday rotation angle: θ = V · B · L
/// V = Verdet constant (rad/(T·m)), B = magnetic field along path (T), L = path length (m)
pub fn faraday_rotation(
    verdet_constant: f64,
    magnetic_field: f64,
    path_length: f64,
) -> Option<f64> {
    if !verdet_constant.is_finite() || !magnetic_field.is_finite() || !path_length.is_finite() {
        return None;
    }
    Some(verdet_constant * magnetic_field * path_length)
}
