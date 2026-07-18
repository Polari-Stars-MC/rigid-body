use std::slice;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, FemHeatDiffusionReport, FemHeatEdge, FemHeatNode, HeatConductionReport,
    MaterialProperties, PhaseChangeReport, ThermalRadiationReport, ThermalStressReport,
    ThermoelasticReport,
};

use crate::rapier::math::{KahanSum, finite_non_negative, finite_positive};

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

#[unsafe(no_mangle)]
pub extern "C" fn thermal_fourier_conduction(
    hot_temperature: f64,
    cold_temperature: f64,
    conductivity: f64,
    area: f64,
    thickness: f64,
    out_report: *mut HeatConductionReport,
) -> Bool {
    if !hot_temperature.is_finite()
        || !cold_temperature.is_finite()
        || !finite_non_negative(conductivity)
        || !finite_non_negative(area)
        || !finite_positive(thickness)
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid Fourier conduction parameters",
        );
        return Bool::FALSE;
    }
    let temperature_delta = hot_temperature - cold_temperature;
    let temperature_gradient = temperature_delta / thickness;
    let heat_flux = conductivity * temperature_gradient;
    let heat_rate = heat_flux * area;
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "heat conduction output is null");
        return Bool::FALSE;
    };
    *out_report = HeatConductionReport {
        temperature_delta,
        temperature_gradient,
        heat_flux,
        heat_rate,
        thermal_resistance: if conductivity > 0.0 && area > 0.0 {
            thickness / (conductivity * area)
        } else {
            f64::INFINITY
        },
    };
    clear_error();
    Bool::TRUE
}

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

#[cfg(test)]
mod tests {
    use super::*;

    fn material() -> MaterialProperties {
        MaterialProperties {
            density: 7850.0,
            friction: 0.5,
            restitution: 0.1,
            youngs_modulus: 200.0e9,
            poisson_ratio: 0.3,
            thermal_expansion: 12.0e-6,
        }
    }

    #[test]
    fn thermal_formulas_work() {
        let mut conduction = HeatConductionReport::default();
        assert_eq!(
            thermal_fourier_conduction(400.0, 300.0, 10.0, 2.0, 0.5, &mut conduction),
            Bool::TRUE
        );
        assert_eq!(conduction.heat_rate, 4000.0);

        let mut phase = PhaseChangeReport::default();
        assert_eq!(
            thermal_phase_change(290.0, 300.0, 2.0, 10.0, 100.0, 300.0, &mut phase),
            Bool::TRUE
        );
        assert_eq!(phase.final_temperature, 300.0);
        assert!((phase.phase_fraction_delta - 0.5).abs() < 1.0e-12);

        let mut radiation = ThermalRadiationReport::default();
        assert_eq!(
            thermal_stefan_boltzmann_radiation(400.0, 300.0, 0.8, 2.0, &mut radiation),
            Bool::TRUE
        );
        assert!(radiation.net_power > 0.0);

        let mut stress = ThermalStressReport::default();
        assert_eq!(
            thermal_stress_from_expansion(material(), 0.0, 100.0, &mut stress),
            Bool::TRUE
        );
        assert!(stress.stress < 0.0);
    }

    #[test]
    fn fem_heat_diffusion_moves_heat_between_nodes() {
        let nodes = [
            FemHeatNode {
                temperature: 400.0,
                heat_capacity: 10.0,
                heat_source: 0.0,
            },
            FemHeatNode {
                temperature: 300.0,
                heat_capacity: 10.0,
                heat_source: 0.0,
            },
        ];
        let edges = [FemHeatEdge {
            node_a: 0,
            node_b: 1,
            conductance: 2.0,
        }];
        let mut temperatures = [0.0; 2];
        let mut report = FemHeatDiffusionReport::default();
        assert_eq!(
            thermal_fem_diffusion_step(
                nodes.as_ptr(),
                nodes.len() as u32,
                edges.as_ptr(),
                edges.len() as u32,
                0.1,
                temperatures.as_mut_ptr(),
                temperatures.len() as u32,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(temperatures[0] < 400.0);
        assert!(temperatures[1] > 300.0);
        assert_eq!(report.edge_count, 1);
    }
}
