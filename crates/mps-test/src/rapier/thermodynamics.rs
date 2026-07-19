#[cfg(test)]
mod tests {
    use mps_core::rapier::thermodynamics::*;
    use mps_core::rapier::ffi::*;

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



