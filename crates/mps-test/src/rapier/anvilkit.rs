#[cfg(all(test, feature = "anvilkit-bridge"))]
mod tests {
    use mps_core::rapier::anvilkit::*;
    use mps_core::rapier::ffi::*;

    fn test_material() -> MaterialProperties {
        MaterialProperties {
            density: 2.0,
            friction: 0.6,
            restitution: 0.3,
            youngs_modulus: 2.0e11,
            poisson_ratio: 0.3,
            thermal_expansion: 1.2e-5,
        }
    }

    #[test]
    fn material_formulas_work() {
        let material = test_material();
        let mut stress = StressStrainReport::default();
        assert_eq!(
            material_stress_strain_linear(material, 0.001, 10.0, &mut stress),
            Bool::TRUE
        );
        assert!(stress.stress > 0.0);
        assert!(stress.thermal_strain > 0.0);

        let rebound = material_elastic_collision_relative_speed(-5.0, material.restitution);
        assert!(rebound > 0.0);

        let mut hertz = HertzContactReport::default();
        assert_eq!(
            material_hertz_contact_force(
                material, material, 0.5, 0.5, 0.001, 0.2, 10.0, &mut hertz,
            ),
            Bool::TRUE
        );
        assert!(hertz.normal_force > 0.0);
        assert!(hertz.contact_area > 0.0);
        assert!(hertz.total_force > hertz.normal_force);
    }
}



