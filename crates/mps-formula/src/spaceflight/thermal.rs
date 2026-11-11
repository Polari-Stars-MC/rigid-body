//! `spaceflight::thermal` submodule — thermal control (heat balance, heat pipe, single-phase loop, radiator, reentry peak g-load, Sutton-Graves heat rate)
//!
//! Split out of the original 2040-line `spaceflight.rs` per OPTIMIZATION.md §N8.
//! See [`super`] for the shared helpers (`finite`, `clamp_unit`,
//! `stumpff_functions`) and numeric constants (`EPS`, `SIGMA`,
//! `SPEED_OF_LIGHT`, `PI`, `TAU`).
//!
//! All public functions keep their `pub fn` names and signatures
//! unchanged; the crate-level `pub use` in `super::mod` keeps the
//! downstream `mps-core::rapier::spaceflight::*` path stable.

use super::*;
use crate::domain::{self, FormulaError};

/// `heat_pipe_thermal_resistance` 的 checked 变体：逐项有限性校验，
/// 失败时返回带参数名的 [`FormulaError`]。
pub fn heat_pipe_thermal_resistance_checked(
    evaporator_resistance: f64,
    vapor_resistance: f64,
    condenser_resistance: f64,
    wick_resistance: f64,
) -> Result<f64, FormulaError> {
    domain::finite(evaporator_resistance, "evaporator_resistance")?;
    domain::finite(vapor_resistance, "vapor_resistance")?;
    domain::finite(condenser_resistance, "condenser_resistance")?;
    domain::finite(wick_resistance, "wick_resistance")?;
    domain::finite_result(
        evaporator_resistance + vapor_resistance + condenser_resistance + wick_resistance,
        "total thermal resistance",
    )
}

pub fn heat_pipe_thermal_resistance(
    evaporator_resistance: f64,
    vapor_resistance: f64,
    condenser_resistance: f64,
    wick_resistance: f64,
) -> Option<f64> {
    heat_pipe_thermal_resistance_checked(
        evaporator_resistance,
        vapor_resistance,
        condenser_resistance,
        wick_resistance,
    )
    .ok()
}

/// `radiator_power` 的 checked 变体。
pub fn radiator_power_checked(
    area: f64,
    emissivity: f64,
    temperature: f64,
    sink_temperature: f64,
    absorbed_power: f64,
) -> Result<RadiatorPower, FormulaError> {
    domain::nonnegative(area, "area")?;
    domain::nonnegative(emissivity, "emissivity")?;
    domain::nonnegative(temperature, "temperature")?;
    domain::nonnegative(sink_temperature, "sink_temperature")?;
    domain::finite(absorbed_power, "absorbed_power")?;
    let emitted =
        emissivity * SIGMA * area * (temperature.powi(4) - sink_temperature.powi(4)).max(0.0);
    Ok(RadiatorPower {
        emitted_power: emitted,
        net_power: emitted - absorbed_power,
    })
}

pub fn radiator_power(
    area: f64,
    emissivity: f64,
    temperature: f64,
    sink_temperature: f64,
    absorbed_power: f64,
) -> Option<RadiatorPower> {
    radiator_power_checked(
        area,
        emissivity,
        temperature,
        sink_temperature,
        absorbed_power,
    )
    .ok()
}

/// `reentry_peak_g_load` 的 checked 变体。
pub fn reentry_peak_g_load_checked(
    beta: f64,
    entry_velocity: f64,
    entry_angle: f64,
    scale_height: f64,
) -> Result<f64, FormulaError> {
    domain::positive(beta, "beta")?;
    domain::positive(entry_velocity, "entry_velocity")?;
    domain::finite(entry_angle, "entry_angle")?;
    domain::positive(scale_height, "scale_height")?;
    let sin_gamma = entry_angle.sin().abs();
    if sin_gamma < EPS {
        return Err(FormulaError::OutOfDomain {
            parameter: "entry_angle",
            requirement: "sin(angle) >= EPS (entry must not be near-grazing)",
        });
    }
    domain::finite_result(
        entry_velocity * entry_velocity * sin_gamma
            / (2.0 * std::f64::consts::E * beta * 9.80665 * scale_height),
        "peak g-load",
    )
}

/// Re-entry deceleration (g-load) for ballistic entry.
pub fn reentry_peak_g_load(
    beta: f64,
    entry_velocity: f64,
    entry_angle: f64,
    scale_height: f64,
) -> Option<f64> {
    reentry_peak_g_load_checked(beta, entry_velocity, entry_angle, scale_height).ok()
}

/// `single_phase_loop_heat_transfer` 的 checked 变体。
pub fn single_phase_loop_heat_transfer_checked(
    mass_flow_rate: f64,
    specific_heat: f64,
    inlet_temperature: f64,
    heat_input: f64,
) -> Result<FluidLoopHeatTransfer, FormulaError> {
    domain::positive(mass_flow_rate, "mass_flow_rate")?;
    domain::positive(specific_heat, "specific_heat")?;
    domain::finite(inlet_temperature, "inlet_temperature")?;
    domain::finite(heat_input, "heat_input")?;
    let outlet = domain::finite_result(
        inlet_temperature + heat_input / (mass_flow_rate * specific_heat),
        "outlet_temperature",
    )?;
    Ok(FluidLoopHeatTransfer {
        heat_rate: heat_input,
        outlet_temperature: outlet,
    })
}

pub fn single_phase_loop_heat_transfer(
    mass_flow_rate: f64,
    specific_heat: f64,
    inlet_temperature: f64,
    heat_input: f64,
) -> Option<FluidLoopHeatTransfer> {
    single_phase_loop_heat_transfer_checked(
        mass_flow_rate,
        specific_heat,
        inlet_temperature,
        heat_input,
    )
    .ok()
}

/// `sutton_graves_heat_rate` 的 checked 变体。
pub fn sutton_graves_heat_rate_checked(
    density: f64,
    velocity: f64,
    nose_radius: f64,
    planet_k: f64,
) -> Result<f64, FormulaError> {
    domain::nonnegative(density, "density")?;
    domain::nonnegative(velocity, "velocity")?;
    domain::positive(nose_radius, "nose_radius")?;
    domain::positive(planet_k, "planet_k")?;
    let q = planet_k * (density / nose_radius).sqrt() * velocity.powi(3);
    domain::finite_result(q, "heat rate")
}

/// Sutton-Graves convective heating rate (stagnation point).
/// q_dot = k * sqrt(rho / r_n) * V^3  (W/m²)
/// Earth: k ≈ 1.83e-4 (kg^(1/2)/m)
pub fn sutton_graves_heat_rate(
    density: f64,
    velocity: f64,
    nose_radius: f64,
    planet_k: f64,
) -> Option<f64> {
    sutton_graves_heat_rate_checked(density, velocity, nose_radius, planet_k).ok()
}

/// `thermal_balance` 的 checked 变体。
pub fn thermal_balance_checked(
    absorbed_power: f64,
    internal_power: f64,
    emitted_area: f64,
    emissivity: f64,
) -> Result<ThermalBalance, FormulaError> {
    domain::finite(absorbed_power, "absorbed_power")?;
    domain::finite(internal_power, "internal_power")?;
    domain::positive(emitted_area, "emitted_area")?;
    domain::positive(emissivity, "emissivity")?;
    let net = absorbed_power + internal_power;
    let equilibrium_temperature = if net > 0.0 {
        (net / (emissivity * SIGMA * emitted_area)).powf(0.25)
    } else {
        0.0
    };
    let equilibrium_temperature =
        domain::finite_result(equilibrium_temperature, "equilibrium_temperature")?;
    Ok(ThermalBalance {
        net_power: net,
        equilibrium_temperature,
    })
}

pub fn thermal_balance(
    absorbed_power: f64,
    internal_power: f64,
    emitted_area: f64,
    emissivity: f64,
) -> Option<ThermalBalance> {
    thermal_balance_checked(absorbed_power, internal_power, emitted_area, emissivity).ok()
}
