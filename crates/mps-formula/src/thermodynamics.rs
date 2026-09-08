use std::slice;

use crate::error::{ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error};
use crate::ffi::{
    Bool, FemHeatDiffusionReport, FemHeatEdge, FemHeatNode, HeatConductionReport,
    MaterialProperties, PhaseChangeReport, ThermalRadiationReport, ThermalStressReport,
    ThermoelasticReport,
};

use crate::math::{KahanSum, finite_non_negative, finite_positive};

const STEFAN_BOLTZMANN: f64 = 5.670_374_419e-8;
const MAX_FEM_NODES: u32 = 1_000_000;
const MAX_FEM_EDGES: u32 = 2_000_000;

fn material_valid(material: MaterialProperties) -> bool {
    finite_non_negative(material.density)
        && finite_non_negative(material.friction)
        && finite_non_negative(material.restitution)
        && finite_positive(material.youngs_modulus)
        && material.poisson_ratio.is_finite()
        && material.poisson_ratio > -1.0
        && material.poisson_ratio < 0.5
        && material.thermal_expansion.is_finite()
}

/// Pure Fourier conduction with explicit domain errors.
///
/// Temperatures must be finite (signed values are valid on a Celsius scale).
/// Conductivity and area must be finite and nonnegative; thickness must be
/// finite and positive. All computed quantities must be finite except for
/// thermal resistance: exactly zero conductivity or area models an insulator
/// and intentionally returns positive infinity. Infinite inputs are never accepted.
pub fn fourier_conduction_checked(
    hot_temperature: f64,
    cold_temperature: f64,
    conductivity: f64,
    area: f64,
    thickness: f64,
) -> Result<HeatConductionReport, crate::FormulaError> {
    use crate::domain;
    domain::finite(hot_temperature, "hot_temperature")?;
    domain::finite(cold_temperature, "cold_temperature")?;
    domain::nonnegative(conductivity, "conductivity")?;
    domain::nonnegative(area, "area")?;
    domain::positive(thickness, "thickness")?;
    let temperature_delta =
        domain::finite_result(hot_temperature - cold_temperature, "temperature delta")?;
    let temperature_gradient = domain::divide(temperature_delta, thickness)?;
    let heat_flux = domain::finite_result(conductivity * temperature_gradient, "heat flux")?;
    let heat_rate = domain::finite_result(heat_flux * area, "heat rate")?;
    let thermal_resistance = if conductivity == 0.0 || area == 0.0 {
        f64::INFINITY
    } else {
        let denominator = domain::finite_result(conductivity * area, "conductivity times area")?;
        domain::divide(thickness, denominator)?
    };
    Ok(HeatConductionReport {
        temperature_delta,
        temperature_gradient,
        heat_flux,
        heat_rate,
        thermal_resistance,
    })
}

/// Fourier heat conduction through a slab: flux, heat rate, and thermal resistance.
///
/// Uses [`fourier_conduction_checked`]; domain errors set `ERR_INVALID_ARGUMENT`
/// without writing the output. Zero conductivity or area intentionally produces
/// positive infinite thermal resistance. NaN and infinite inputs are rejected.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `HeatConductionReport`
/// element; a null pointer fails with `ERR_NULL_POINTER`. No ownership is
/// transferred; the caller keeps ownership of `out_report`.
#[unsafe(no_mangle)]
pub extern "C" fn thermal_fourier_conduction(
    hot_temperature: f64,
    cold_temperature: f64,
    conductivity: f64,
    area: f64,
    thickness: f64,
    out_report: *mut HeatConductionReport,
) -> Bool {
    let Some(report) = crate::ffi::formula_result(fourier_conduction_checked(
        hot_temperature,
        cold_temperature,
        conductivity,
        area,
        thickness,
    )) else {
        return Bool::FALSE;
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "heat conduction output is null");
        return Bool::FALSE;
    };
    *out_report = report;
    clear_error();
    Bool::TRUE
}

/// Phase-change heating: splits `heat_input` into sensible and latent heat.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `PhaseChangeReport`
/// element; a null pointer fails with `ERR_NULL_POINTER`. No ownership is
/// transferred; the caller keeps ownership of `out_report`.
#[unsafe(no_mangle)]
pub extern "C" fn thermal_phase_change(
    temperature: f64,
    phase_temperature: f64,
    mass: f64,
    specific_heat: f64,
    latent_heat: f64,
    heat_input: f64,
    out_report: *mut PhaseChangeReport,
) -> Bool {
    if !temperature.is_finite()
        || !phase_temperature.is_finite()
        || !finite_positive(mass)
        || !finite_positive(specific_heat)
        || !finite_non_negative(latent_heat)
        || !heat_input.is_finite()
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid phase-change parameters");
        return Bool::FALSE;
    }

    let mut sensible_heat = heat_input;
    let mut latent_heat_used = 0.0;
    let mut final_temperature = temperature;
    let mut phase_fraction_delta = 0.0;
    let phase_energy = mass * latent_heat;

    if heat_input > 0.0 && temperature < phase_temperature {
        let heat_to_phase = (phase_temperature - temperature) * mass * specific_heat;
        let used = heat_input.min(heat_to_phase);
        final_temperature += used / (mass * specific_heat);
        sensible_heat = used;
        let remaining = heat_input - used;
        if remaining > 0.0 && phase_energy > 0.0 {
            latent_heat_used = remaining.min(phase_energy);
            phase_fraction_delta = latent_heat_used / phase_energy;
            final_temperature = phase_temperature;
        }
    } else if heat_input > 0.0
        && (temperature - phase_temperature).abs() <= f64::EPSILON
        && phase_energy > 0.0
    {
        latent_heat_used = heat_input.min(phase_energy);
        phase_fraction_delta = latent_heat_used / phase_energy;
        sensible_heat = heat_input - latent_heat_used;
        final_temperature = phase_temperature + sensible_heat / (mass * specific_heat);
    } else {
        final_temperature += heat_input / (mass * specific_heat);
    }

    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "phase-change output is null");
        return Bool::FALSE;
    };
    *out_report = PhaseChangeReport {
        final_temperature,
        sensible_heat,
        latent_heat_used,
        phase_fraction_delta,
        phase_changed: Bool::from(phase_fraction_delta > 0.0),
    };
    clear_error();
    Bool::TRUE
}

/// Solid fraction interpolation between solidus and liquidus temperatures.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `PhaseChangeReport`
/// element; a null pointer fails with `ERR_NULL_POINTER`. No ownership is
/// transferred; the caller keeps ownership of `out_report`.
#[unsafe(no_mangle)]
pub extern "C" fn thermal_phase_condition(
    temperature: f64,
    solidus_temperature: f64,
    liquidus_temperature: f64,
    out_report: *mut PhaseChangeReport,
) -> Bool {
    if !temperature.is_finite()
        || !solidus_temperature.is_finite()
        || !liquidus_temperature.is_finite()
        || solidus_temperature > liquidus_temperature
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid phase condition temperatures");
        return Bool::FALSE;
    }
    let fraction = if temperature <= solidus_temperature {
        0.0
    } else if temperature >= liquidus_temperature {
        1.0
    } else {
        (temperature - solidus_temperature) / (liquidus_temperature - solidus_temperature)
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "phase condition output is null");
        return Bool::FALSE;
    };
    *out_report = PhaseChangeReport {
        final_temperature: temperature,
        sensible_heat: 0.0,
        latent_heat_used: 0.0,
        phase_fraction_delta: fraction,
        phase_changed: Bool::from(fraction > 0.0),
    };
    clear_error();
    Bool::TRUE
}

/// Stefan-Boltzmann radiative heat exchange: emitted, absorbed, and net power.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `ThermalRadiationReport`
/// element; a null pointer fails with `ERR_NULL_POINTER`. No ownership is
/// transferred; the caller keeps ownership of `out_report`.
#[unsafe(no_mangle)]
pub extern "C" fn thermal_stefan_boltzmann_radiation(
    temperature: f64,
    ambient_temperature: f64,
    emissivity: f64,
    area: f64,
    out_report: *mut ThermalRadiationReport,
) -> Bool {
    if !finite_non_negative(temperature)
        || !finite_non_negative(ambient_temperature)
        || !emissivity.is_finite()
        || !(0.0..=1.0).contains(&emissivity)
        || !finite_non_negative(area)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid thermal radiation parameters");
        return Bool::FALSE;
    }
    let emitted_power = emissivity * STEFAN_BOLTZMANN * area * temperature.powi(4);
    let absorbed_power = emissivity * STEFAN_BOLTZMANN * area * ambient_temperature.powi(4);
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "thermal radiation output is null");
        return Bool::FALSE;
    };
    *out_report = ThermalRadiationReport {
        emitted_power,
        absorbed_power,
        net_power: emitted_power - absorbed_power,
        radiative_coefficient: emissivity * STEFAN_BOLTZMANN * area,
    };
    clear_error();
    Bool::TRUE
}

/// One explicit FEM heat-diffusion timestep over a node/edge network.
///
/// # Safety
///
/// `nodes` must point to a readable array of `node_count` `FemHeatNode`
/// elements (`0 < node_count <= MAX_FEM_NODES`); `edges` must point to a
/// readable array of `edge_count` `FemHeatEdge` elements
/// (`edge_count <= MAX_FEM_EDGES`) and may be null only when `edge_count == 0`;
/// `out_temperatures` must point to writable memory for `capacity` `f64`
/// elements (`capacity >= node_count`); `out_report` may be null, otherwise it
/// must point to writable memory for one `FemHeatDiffusionReport` element.
/// Null pointers fail with `ERR_NULL_POINTER`, count/capacity violations with
/// `ERR_CAPACITY`. No ownership is transferred; the caller keeps ownership of
/// all buffers.
#[unsafe(no_mangle)]
pub extern "C" fn thermal_fem_diffusion_step(
    nodes: *const FemHeatNode,
    node_count: u32,
    edges: *const FemHeatEdge,
    edge_count: u32,
    dt: f64,
    out_temperatures: *mut f64,
    capacity: u32,
    out_report: *mut FemHeatDiffusionReport,
) -> Bool {
    if node_count == 0
        || node_count > MAX_FEM_NODES
        || edge_count > MAX_FEM_EDGES
        || capacity < node_count
    {
        set_error(ERR_CAPACITY, "invalid FEM heat diffusion capacity");
        return Bool::FALSE;
    }
    if nodes.is_null() || out_temperatures.is_null() || (edge_count > 0 && edges.is_null()) {
        set_error(ERR_NULL_POINTER, "FEM heat diffusion pointers are null");
        return Bool::FALSE;
    }
    if !finite_non_negative(dt) {
        set_error(ERR_INVALID_ARGUMENT, "invalid FEM heat diffusion timestep");
        return Bool::FALSE;
    }

    let nodes = unsafe { slice::from_raw_parts(nodes, node_count as usize) };
    let edges = unsafe { slice::from_raw_parts(edges, edge_count as usize) };
    let out_temperatures =
        unsafe { slice::from_raw_parts_mut(out_temperatures, capacity as usize) };
    // Use out_temperatures as temporary scratch before writing final values:
    // first pass accumulates heat_rates into out_temperatures directly,
    // second pass converts to temperature deltas in-place.
    let heat_rates = &mut out_temperatures[..node_count as usize];
    heat_rates.fill(0.0);

    for (index, node) in nodes.iter().enumerate() {
        if !node.temperature.is_finite()
            || !finite_positive(node.heat_capacity)
            || !node.heat_source.is_finite()
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid FEM heat node");
            return Bool::FALSE;
        }
        heat_rates[index] += node.heat_source;
    }

    for edge in edges {
        if edge.node_a >= node_count
            || edge.node_b >= node_count
            || !finite_non_negative(edge.conductance)
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid FEM heat edge");
            return Bool::FALSE;
        }
        let a = edge.node_a as usize;
        let b = edge.node_b as usize;
        let heat_rate = edge.conductance * (nodes[b].temperature - nodes[a].temperature);
        heat_rates[a] += heat_rate;
        heat_rates[b] -= heat_rate;
    }

    let mut max_temperature_delta = 0.0;
    let mut total_heat_rate_acc = KahanSum::default();
    for (index, node) in nodes.iter().enumerate() {
        let delta = heat_rates[index] * dt / node.heat_capacity;
        heat_rates[index] = node.temperature + delta;
        max_temperature_delta = f64::max(max_temperature_delta, delta.abs());
        total_heat_rate_acc.add(heat_rates[index]);
    }

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = FemHeatDiffusionReport {
            node_count,
            edge_count,
            total_heat_rate: total_heat_rate_acc.value(),
            max_temperature_delta,
        };
    }
    clear_error();
    Bool::TRUE
}

/// Thermal stress from constrained thermal expansion (1D).
///
/// # Safety
///
/// `out_report` must point to writable memory for one `ThermalStressReport`
/// element; a null pointer fails with `ERR_NULL_POINTER`. No ownership is
/// transferred; the caller keeps ownership of `out_report`.
#[unsafe(no_mangle)]
pub extern "C" fn thermal_stress_from_expansion(
    material: MaterialProperties,
    strain: f64,
    delta_temperature: f64,
    out_report: *mut ThermalStressReport,
) -> Bool {
    if !material_valid(material) || !strain.is_finite() || !delta_temperature.is_finite() {
        set_error(ERR_INVALID_ARGUMENT, "invalid thermal stress parameters");
        return Bool::FALSE;
    }
    let thermal_strain = material.thermal_expansion * delta_temperature;
    let mechanical_strain = strain - thermal_strain;
    let stress = material.youngs_modulus * mechanical_strain;
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "thermal stress output is null");
        return Bool::FALSE;
    };
    *out_report = ThermalStressReport {
        free_thermal_strain: thermal_strain,
        mechanical_strain,
        stress,
        deformation: thermal_strain,
        elastic_energy_density: 0.5 * stress * mechanical_strain,
    };
    clear_error();
    Bool::TRUE
}

/// 3D thermoelastic stress/strain for an isotropic material under a
/// temperature change.
///
/// # Safety
///
/// `out_report` must point to writable memory for one `ThermoelasticReport`
/// element; a null pointer fails with `ERR_NULL_POINTER`. No ownership is
/// transferred; the caller keeps ownership of `out_report`.
#[unsafe(no_mangle)]
pub extern "C" fn thermal_thermoelastic_stress_strain(
    material: MaterialProperties,
    strain_x: f64,
    strain_y: f64,
    strain_z: f64,
    delta_temperature: f64,
    out_report: *mut ThermoelasticReport,
) -> Bool {
    if !material_valid(material)
        || !strain_x.is_finite()
        || !strain_y.is_finite()
        || !strain_z.is_finite()
        || !delta_temperature.is_finite()
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid thermoelastic parameters");
        return Bool::FALSE;
    }
    let thermal_strain = material.thermal_expansion * delta_temperature;
    let ex = strain_x - thermal_strain;
    let ey = strain_y - thermal_strain;
    let ez = strain_z - thermal_strain;
    let lambda = material.youngs_modulus * material.poisson_ratio
        / ((1.0 + material.poisson_ratio) * (1.0 - 2.0 * material.poisson_ratio));
    let shear = material.youngs_modulus / (2.0 * (1.0 + material.poisson_ratio));
    let trace = ex + ey + ez;
    let stress_x = lambda * trace + 2.0 * shear * ex;
    let stress_y = lambda * trace + 2.0 * shear * ey;
    let stress_z = lambda * trace + 2.0 * shear * ez;
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "thermoelastic output is null");
        return Bool::FALSE;
    };
    *out_report = ThermoelasticReport {
        thermal_strain,
        mechanical_strain_x: ex,
        mechanical_strain_y: ey,
        mechanical_strain_z: ez,
        stress_x,
        stress_y,
        stress_z,
        bulk_modulus: material.youngs_modulus / (3.0 * (1.0 - 2.0 * material.poisson_ratio)),
        shear_modulus: shear,
    };
    clear_error();
    Bool::TRUE
}

// ---------------------------------------------------------------------------
// Ideal gas law
// ---------------------------------------------------------------------------

/// Ideal gas law: PV = nRT. Returns pressure (Pa).
pub fn ideal_gas_pressure(volume: f64, moles: f64, temperature: f64) -> Option<f64> {
    if !volume.is_finite()
        || volume <= 0.0
        || !moles.is_finite()
        || moles < 0.0
        || !temperature.is_finite()
        || temperature < 0.0
    {
        return None;
    }
    Some(moles * 8.314462618 * temperature / volume)
}

/// Returns volume from ideal gas law.
pub fn ideal_gas_volume(pressure: f64, moles: f64, temperature: f64) -> Option<f64> {
    if !pressure.is_finite()
        || pressure < 0.0
        || !moles.is_finite()
        || moles < 0.0
        || !temperature.is_finite()
        || temperature < 0.0
    {
        return None;
    }
    Some(moles * 8.314462618 * temperature / pressure)
}

/// Returns temperature from ideal gas law.
pub fn ideal_gas_temperature(pressure: f64, volume: f64, moles: f64) -> Option<f64> {
    if !pressure.is_finite()
        || pressure < 0.0
        || !volume.is_finite()
        || volume <= 0.0
        || !moles.is_finite()
        || moles <= 0.0
    {
        return None;
    }
    Some(pressure * volume / (moles * 8.314462618))
}

// ---------------------------------------------------------------------------
// Polytropic process
// ---------------------------------------------------------------------------

/// Polytropic process: P2 = P1 * (V1/V2)^gamma
pub fn polytropic_pressure(p1: f64, v1: f64, v2: f64, gamma: f64) -> Option<f64> {
    if !finite_4(p1, v1, v2, gamma) || p1 < 0.0 || v1 <= 0.0 || v2 <= 0.0 || gamma <= 0.0 {
        return None;
    }
    Some(p1 * (v1 / v2).powf(gamma))
}

/// Polytropic work: W = (P2*V2 - P1*V1) / (1 - gamma)
pub fn polytropic_work(p1: f64, v1: f64, p2: f64, v2: f64, gamma: f64) -> Option<f64> {
    if !finite_4(p1, v1, gamma, 0.0)
        || !finite_4(p2, v2, gamma, 0.0)
        || p1 < 0.0
        || v1 <= 0.0
        || p2 < 0.0
        || v2 <= 0.0
        || gamma <= 0.0
    {
        return None;
    }
    if (gamma - 1.0).abs() < 1.0e-12 {
        return None;
    }
    Some((p2 * v2 - p1 * v1) / (1.0 - gamma))
}

// ---------------------------------------------------------------------------
// Convective heat transfer
// ---------------------------------------------------------------------------

/// Newton's law of cooling: Q = h * A * (T_surface - T_fluid)
pub fn convective_heat_flux(h: f64, area: f64, t_surface: f64, t_fluid: f64) -> Option<f64> {
    if !finite_4(h, area, t_surface, t_fluid) || h < 0.0 || area < 0.0 {
        return None;
    }
    Some(h * area * (t_surface - t_fluid))
}

/// Reynolds number: Re = rho * v * L / mu
pub fn reynolds_number(
    density: f64,
    velocity: f64,
    char_length: f64,
    viscosity: f64,
) -> Option<f64> {
    if !finite_4(density, velocity, char_length, viscosity)
        || density < 0.0
        || velocity < 0.0
        || char_length <= 0.0
        || viscosity <= 0.0
    {
        return None;
    }
    Some(density * velocity * char_length / viscosity)
}

/// Nusselt number (Dittus-Boelter): Nu = 0.023 * Re^0.8 * Pr^n
pub fn dittus_boelter_nusselt(reynolds: f64, prandtl: f64, heating: bool) -> Option<f64> {
    if !reynolds.is_finite() || reynolds < 0.0 || !prandtl.is_finite() || prandtl < 0.0 {
        return None;
    }
    if reynolds < 10000.0 {
        return None;
    }
    let n = if heating { 0.4 } else { 0.3 };
    Some(0.023 * reynolds.powf(0.8) * prandtl.powf(n))
}

/// Prandtl number: Pr = cp * mu / k
pub fn prandtl_number(cp: f64, viscosity: f64, conductivity: f64) -> Option<f64> {
    if !finite_4(cp, viscosity, conductivity, 0.0)
        || cp <= 0.0
        || viscosity <= 0.0
        || conductivity <= 0.0
    {
        return None;
    }
    Some(cp * viscosity / conductivity)
}

/// Heat transfer coefficient from Nusselt: h = Nu * k / L
pub fn htc_from_nusselt(nusselt: f64, conductivity: f64, char_length: f64) -> Option<f64> {
    if !finite_4(nusselt, conductivity, char_length, 0.0)
        || nusselt < 0.0
        || conductivity <= 0.0
        || char_length <= 0.0
    {
        return None;
    }
    Some(nusselt * conductivity / char_length)
}

// ---------------------------------------------------------------------------
// Thermodynamic cycle efficiencies
// ---------------------------------------------------------------------------

/// Carnot efficiency: eta = 1 - T_cold / T_hot
pub fn carnot_efficiency(t_hot: f64, t_cold: f64) -> Option<f64> {
    if !finite_4(t_hot, t_cold, 0.0, 0.0) || t_hot <= 0.0 || t_cold < 0.0 || t_cold >= t_hot {
        return None;
    }
    Some(1.0 - t_cold / t_hot)
}

/// Otto cycle efficiency: eta = 1 - 1 / r^(gamma-1)
pub fn otto_efficiency(compression_ratio: f64, gamma: f64) -> Option<f64> {
    if !compression_ratio.is_finite()
        || compression_ratio <= 1.0
        || !gamma.is_finite()
        || gamma <= 1.0
    {
        return None;
    }
    Some(1.0 - 1.0 / compression_ratio.powf(gamma - 1.0))
}

/// Diesel cycle efficiency
pub fn diesel_efficiency(compression_ratio: f64, cutoff_ratio: f64, gamma: f64) -> Option<f64> {
    if !finite_4(compression_ratio, cutoff_ratio, gamma, 0.0)
        || compression_ratio <= 1.0
        || cutoff_ratio <= 1.0
        || gamma <= 1.0
    {
        return None;
    }
    let term = (cutoff_ratio.powf(gamma) - 1.0) / (gamma * (cutoff_ratio - 1.0));
    Some(1.0 - 1.0 / compression_ratio.powf(gamma - 1.0) * term)
}

/// Brayton cycle efficiency: eta = 1 - 1 / r_p^((gamma-1)/gamma)
pub fn brayton_efficiency(pressure_ratio: f64, gamma: f64) -> Option<f64> {
    if !pressure_ratio.is_finite() || pressure_ratio <= 1.0 || !gamma.is_finite() || gamma <= 1.0 {
        return None;
    }
    Some(1.0 - 1.0 / pressure_ratio.powf((gamma - 1.0) / gamma))
}

// ---------------------------------------------------------------------------
// Clausius-Clapeyron
// ---------------------------------------------------------------------------

/// Clausius-Clapeyron: ln(P2/P1) = -(L/R) * (1/T2 - 1/T1)
pub fn clausius_clapeyron_pressure(p1: f64, t1: f64, t2: f64, latent_heat: f64) -> Option<f64> {
    if !finite_4(p1, t1, t2, latent_heat)
        || p1 <= 0.0
        || t1 <= 0.0
        || t2 <= 0.0
        || latent_heat < 0.0
    {
        return None;
    }
    Some(p1 * (-latent_heat / 8.314462618 * (1.0 / t2 - 1.0 / t1)).exp())
}

// ---------------------------------------------------------------------------
// Entropy change
// ---------------------------------------------------------------------------

pub fn entropy_change_constant_volume(moles: f64, cv: f64, t1: f64, t2: f64) -> Option<f64> {
    if !finite_4(moles, cv, t1, t2) || moles < 0.0 || cv <= 0.0 || t1 <= 0.0 || t2 <= 0.0 {
        return None;
    }
    Some(moles * cv * (t2 / t1).ln())
}

pub fn entropy_change_constant_pressure(moles: f64, cp: f64, t1: f64, t2: f64) -> Option<f64> {
    if !finite_4(moles, cp, t1, t2) || moles < 0.0 || cp <= 0.0 || t1 <= 0.0 || t2 <= 0.0 {
        return None;
    }
    Some(moles * cp * (t2 / t1).ln())
}

fn finite_4(a: f64, b: f64, c: f64, d: f64) -> bool {
    a.is_finite() && b.is_finite() && c.is_finite() && d.is_finite()
}

fn finite_5(a: f64, b: f64, c: f64, d: f64, e: f64) -> bool {
    a.is_finite() && b.is_finite() && c.is_finite() && d.is_finite() && e.is_finite()
}

// ---------------------------------------------------------------------------
// Van der Waals equation of state
// ---------------------------------------------------------------------------

/// Van der Waals pressure: P = RT/(V-b) - a/V²
pub fn van_der_waals_pressure(temperature: f64, molar_volume: f64, a: f64, b: f64) -> Option<f64> {
    if !finite_5(temperature, molar_volume, a, b, 0.0)
        || temperature <= 0.0
        || molar_volume <= 0.0
        || a < 0.0
        || b < 0.0
    {
        return None;
    }
    let r = 8.314462618;
    Some(r * temperature / (molar_volume - b) - a / (molar_volume * molar_volume))
}

/// Van der Waals critical point: Tc = 8a/(27Rb), Pc = a/(27b²), Vc = 3b
pub fn van_der_waals_critical_point(a: f64, b: f64) -> Option<(f64, f64, f64)> {
    if !finite_5(a, b, 0.0, 0.0, 0.0) || a <= 0.0 || b <= 0.0 {
        return None;
    }
    let r = 8.314462618;
    let tc = 8.0 * a / (27.0 * r * b);
    let pc = a / (27.0 * b * b);
    let vc = 3.0 * b;
    Some((tc, pc, vc))
}

// ---------------------------------------------------------------------------
// Maxwell relations (thermodynamic potentials)
// ---------------------------------------------------------------------------

/// Maxwell relation 1: (∂T/∂V)_S = -(∂P/∂S)_V
pub fn maxwell_relation_1(_temperature: f64, _volume: f64, _entropy: f64, _pressure: f64) -> f64 {
    0.0 // stub — analytical form depends on the specific EOS; use as reminder of the identity
}

// ---------------------------------------------------------------------------
// Enthalpy — 实现迁至 `scientists::willard_gibbs::formulas`，
// 此处仅 `pub use` 重导出以保持 `mps_formula::thermodynamics::*` 路径稳定。
// ---------------------------------------------------------------------------
pub use crate::scientists::willard_gibbs::formulas::enthalpy;

// ---------------------------------------------------------------------------
// Helmholtz free energy — 实现迁至 `scientists::hermann_von_helmholtz::formulas`，
// 此处仅 `pub use` 重导出以保持 `mps_formula::thermodynamics::*` 路径稳定。
// ---------------------------------------------------------------------------
pub use crate::scientists::hermann_von_helmholtz::formulas::helmholtz_free_energy;

// ---------------------------------------------------------------------------
// Gibbs free energy — 实现迁至 `scientists::willard_gibbs::formulas`，
// 此处仅 `pub use` 重导出以保持 `mps_formula::thermodynamics::*` 路径稳定。
// ---------------------------------------------------------------------------
pub use crate::scientists::willard_gibbs::formulas::gibbs_free_energy;

// ---------------------------------------------------------------------------
// Joule-Thomson effect — 实现迁至 `scientists::james_thomson::formulas`，
// 此处仅 `pub use` 重导出以保持 `mps_formula::thermodynamics::*` 路径稳定。
// ---------------------------------------------------------------------------
pub use crate::scientists::james_thomson::formulas::joule_thomson_coefficient;
pub use crate::scientists::james_thomson::formulas::joule_thomson_inversion_temperature;

// ---------------------------------------------------------------------------
// Heat capacity
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Debye heat capacity — 实现迁至 `scientists::peter_debye::formulas`，
// 此处仅 `pub use` 重导出以保持 `mps_formula::thermodynamics::*` 路径稳定。
// ---------------------------------------------------------------------------
pub use crate::scientists::peter_debye::formulas::debye_heat_capacity_low_t;

// ---------------------------------------------------------------------------
// Einstein heat capacity — 实现迁至 `scientists::albert_einstein::formulas`，
// 此处仅 `pub use` 重导出以保持 `mps_formula::thermodynamics::*` 路径稳定。
// ---------------------------------------------------------------------------
pub use crate::scientists::albert_einstein::formulas::einstein_heat_capacity;

// ---------------------------------------------------------------------------
// Refrigeration cycle
// ---------------------------------------------------------------------------

/// Carnot refrigeration coefficient of performance: COP = Tc / (Th - Tc)
pub fn carnot_refrigeration_cop(t_cold: f64, t_hot: f64) -> Option<f64> {
    if !finite_5(t_cold, t_hot, 0.0, 0.0, 0.0) || t_cold <= 0.0 || t_hot <= 0.0 || t_hot <= t_cold {
        return None;
    }
    Some(t_cold / (t_hot - t_cold))
}

/// Heat pump COP: COP = Th / (Th - Tc)
pub fn heat_pump_cop(t_cold: f64, t_hot: f64) -> Option<f64> {
    if !finite_5(t_cold, t_hot, 0.0, 0.0, 0.0) || t_cold <= 0.0 || t_hot <= 0.0 || t_hot <= t_cold {
        return None;
    }
    Some(t_hot / (t_hot - t_cold))
}

// ---------------------------------------------------------------------------
// Heat exchanger: LMTD and NTU-epsilon methods
// ---------------------------------------------------------------------------

/// Log-mean temperature difference for counter-flow heat exchanger.
pub fn lmtd_counter_flow(
    t_hot_in: f64,
    t_hot_out: f64,
    t_cold_in: f64,
    t_cold_out: f64,
) -> Option<f64> {
    if !finite_5(t_hot_in, t_hot_out, t_cold_in, t_cold_out, 0.0)
        || t_hot_in < t_cold_out
        || t_hot_out < t_cold_in
    {
        return None;
    }
    let d1 = t_hot_in - t_cold_out;
    let d2 = t_hot_out - t_cold_in;
    if d1 <= 0.0 || d2 <= 0.0 {
        return None;
    }
    Some((d1 - d2) / (d1 / d2).ln())
}

/// Log-mean temperature difference for parallel-flow heat exchanger.
pub fn lmtd_parallel_flow(
    t_hot_in: f64,
    t_hot_out: f64,
    t_cold_in: f64,
    t_cold_out: f64,
) -> Option<f64> {
    if !finite_5(t_hot_in, t_hot_out, t_cold_in, t_cold_out, 0.0) {
        return None;
    }
    let d1 = t_hot_in - t_cold_in;
    let d2 = t_hot_out - t_cold_out;
    if d1 <= 0.0 || d2 <= 0.0 {
        return None;
    }
    Some((d1 - d2) / (d1 / d2).ln())
}

/// NTU-epsilon effectiveness for counter-flow heat exchanger.
pub fn ntu_epsilon_counter_flow(ntu: f64, c_r: f64) -> Option<f64> {
    if !finite_5(ntu, c_r, 0.0, 0.0, 0.0) || ntu < 0.0 || c_r < 0.0 {
        return None;
    }
    let epsilon = if c_r >= 1.0 {
        ntu / (1.0 + ntu) // c_r = 1 limiting case
    } else {
        let exp = (-ntu * (1.0 - c_r)).exp();
        (1.0 - exp) / (1.0 - c_r * exp)
    };
    Some(epsilon)
}

/// Number of transfer units: NTU = UA / C_min
pub fn ntu(overall_htc: f64, area: f64, c_min: f64) -> Option<f64> {
    if !finite_5(overall_htc, area, c_min, 0.0, 0.0)
        || overall_htc <= 0.0
        || area <= 0.0
        || c_min <= 0.0
    {
        return None;
    }
    Some(overall_htc * area / c_min)
}

/// Heat capacity rate: C = m_dot * cp
pub fn heat_capacity_rate(mass_flow: f64, specific_heat: f64) -> Option<f64> {
    if !finite_5(mass_flow, specific_heat, 0.0, 0.0, 0.0)
        || mass_flow <= 0.0
        || specific_heat <= 0.0
    {
        return None;
    }
    Some(mass_flow * specific_heat)
}

// ---------------------------------------------------------------------------
// View factor (simple configurations)
// ---------------------------------------------------------------------------

/// View factor for two parallel coaxial disks.
/// R1 = r1/d, R2 = r2/d where d is the separation distance.
pub fn view_factor_coaxial_disks(radius_ratio_1: f64, radius_ratio_2: f64) -> Option<f64> {
    if !finite_5(radius_ratio_1, radius_ratio_2, 0.0, 0.0, 0.0)
        || radius_ratio_1 < 0.0
        || radius_ratio_2 < 0.0
    {
        return None;
    }
    let x = 1.0 + (1.0 + radius_ratio_2 * radius_ratio_2) / (radius_ratio_1 * radius_ratio_1);
    Some(0.5 * (x - (x * x - 4.0 * (radius_ratio_2 / radius_ratio_1).powi(2)).sqrt()))
}

/// View factor for two parallel, equal rectangles.
/// X = a/d, Y = b/d where a, b are side lengths and d is the separation.
pub fn view_factor_parallel_rectangles(x: f64, y: f64) -> Option<f64> {
    if !finite_5(x, y, 0.0, 0.0, 0.0) || x <= 0.0 || y <= 0.0 {
        return None;
    }
    let f = 2.0 / (std::f64::consts::PI * x * y)
        * ((x * x * (1.0 + y * y) / (1.0 + x * x + y * y)).ln().sqrt()
            + (y * y * (1.0 + x * x) / (1.0 + x * x + y * y)).ln().sqrt()
            + x * (1.0 + y * y).atan() / (x * x + y * y + x * x * y * y).sqrt()
            + y * (1.0 + x * x).atan() / (x * x + y * y + y * y * x * x).sqrt()
            - x * x.atan()
            - y * y.atan());
    Some(f)
}

// ---------------------------------------------------------------------------
// Virial expansion
// ---------------------------------------------------------------------------

/// Second virial coefficient for Lennard-Jones gas (simplified).
/// B(T) = b₀ - a₀/RT, where b₀ = 2πN_A σ³/3, a₀ = 2πN_A² ε σ³
pub fn virial_second_coefficient(temperature: f64, sigma: f64, epsilon: f64) -> Option<f64> {
    if !finite_5(temperature, sigma, epsilon, 0.0, 0.0)
        || temperature <= 0.0
        || sigma <= 0.0
        || epsilon < 0.0
    {
        return None;
    }
    let r = 8.314462618;
    let avogadro = 6.022_140_76e23;
    let b0 = 2.0 * std::f64::consts::PI * avogadro * sigma.powi(3) / 3.0;
    let a0 = 2.0 * std::f64::consts::PI * avogadro * avogadro * epsilon * sigma.powi(3);
    Some(b0 - a0 / (r * temperature))
}

// ---------------------------------------------------------------------------
// Two-phase flow
// ---------------------------------------------------------------------------

/// Quality (vapor mass fraction): x = m_vapor / (m_vapor + m_liquid)
pub fn quality(vapor_mass: f64, liquid_mass: f64) -> Option<f64> {
    if !finite_5(vapor_mass, liquid_mass, 0.0, 0.0, 0.0) || vapor_mass < 0.0 || liquid_mass < 0.0 {
        return None;
    }
    let total = vapor_mass + liquid_mass;
    if total <= 0.0 {
        return None;
    }
    Some(vapor_mass / total)
}

/// Homogeneous void fraction: α = 1 / (1 + (1-x)/x * ρ_v/ρ_l)
pub fn homogeneous_void_fraction(quality: f64, rho_vapor: f64, rho_liquid: f64) -> Option<f64> {
    if !finite_5(quality, rho_vapor, rho_liquid, 0.0, 0.0)
        || !(0.0..=1.0).contains(&quality)
        || rho_vapor <= 0.0
        || rho_liquid <= 0.0
    {
        return None;
    }
    if quality <= 0.0 || quality >= 1.0 {
        return Some(quality);
    }
    Some(1.0 / (1.0 + (1.0 - quality) / quality * rho_vapor / rho_liquid))
}
