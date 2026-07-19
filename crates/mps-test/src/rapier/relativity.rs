#[cfg(test)]
mod tests {
    use mps_core::rapier::relativity::*;
    use mps_core::rapier::ffi::*;

    const C: f64 = SPEED_OF_LIGHT;
    const G: f64 = 6.674_30e-11;
    const SOLAR_MASS: f64 = 1.989e30;

    #[test]
    fn lorentz_factor_is_one_at_rest() {
        let mut gamma = 0.0;
        assert_eq!(rel_lorentz_factor(0.0, &mut gamma), Bool::TRUE);
        assert!((gamma - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn lorentz_factor_diverges_near_c() {
        let mut gamma = 0.0;
        let speed = 0.999 * C;
        assert_eq!(rel_lorentz_factor(speed, &mut gamma), Bool::TRUE);
        let expected = 1.0 / (1.0 - 0.999_f64.powi(2)).sqrt();
        assert!((gamma - expected).abs() < 1.0e-6);
        assert!(gamma > 22.0);
    }

    #[test]
    fn lorentz_boost_is_identity_for_zero() {
        let mut boost = LorentzBoost::default();
        assert_eq!(
            rel_lorentz_boost(Vec3::default(), &mut boost),
            Bool::TRUE
        );
        assert!((boost.m00 - 1.0).abs() < 1.0e-12);
        assert!((boost.m11 - 1.0).abs() < 1.0e-12);
        assert!((boost.m22 - 1.0).abs() < 1.0e-12);
        assert!((boost.m33 - 1.0).abs() < 1.0e-12);
        assert!(boost.m01.abs() < 1.0e-12);
        assert!(boost.m10.abs() < 1.0e-12);
    }

    #[test]
    fn transform_four_vector_is_consistent() {
        // Interval invariance: -(ct)^2 + x^2 + y^2 + z^2 = -(ct')^2 + x'^2 + y'^2 + z'^2
        let mut boost = LorentzBoost::default();
        assert_eq!(
            rel_lorentz_boost(
                Vec3 {
                    x: 0.5 * C,
                    y: 0.0,
                    z: 0.0,
                },
                &mut boost,
            ),
            Bool::TRUE
        );
        let mut transformed = LorentzTransformedFrame::default();
        assert_eq!(
            rel_transform_four_vector(boost, 10.0, 3.0, 4.0, 0.0, &mut transformed),
            Bool::TRUE
        );
        let interval = -(10.0 * 10.0) + 3.0 * 3.0 + 4.0 * 4.0 + 0.0;
        let interval_prime = -(transformed.ct_prime * transformed.ct_prime)
            + transformed.x_prime * transformed.x_prime
            + transformed.y_prime * transformed.y_prime
            + transformed.z_prime * transformed.z_prime;
        assert!((interval - interval_prime).abs() < 1.0e-6);
    }

    #[test]
    fn schwarzschild_metric_outside_horizon() {
        let mut metric = SchwarzschildMetric::default();
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        assert!(rs_val.is_finite() && rs_val > 0.0);
        let r = 10.0 * rs_val;
        assert_eq!(
            rel_schwarzschild_metric(r, SOLAR_MASS, G, &mut metric),
            Bool::TRUE
        );
        assert!(metric.g_tt < 0.0);
        assert!(metric.g_rr > 0.0);
        assert!((metric.radius_over_rs - 10.0).abs() < 1.0e-6);
    }

    #[test]
    fn schwarzschild_metric_at_horizon_rejected() {
        let mut metric = SchwarzschildMetric::default();
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        assert_eq!(
            rel_schwarzschild_metric(rs_val, SOLAR_MASS, G, &mut metric),
            Bool::FALSE
        );
        assert_eq!(
            rel_schwarzschild_metric(0.5 * rs_val, SOLAR_MASS, G, &mut metric),
            Bool::FALSE
        );
    }

    #[test]
    fn time_dilation_far_from_mass() {
        let mut dilation = GravitationalTimeDilation::default();
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        let r = 1.0e6 * rs_val; // very far
        assert_eq!(
            rel_gravitational_time_dilation(r, SOLAR_MASS, G, &mut dilation),
            Bool::TRUE
        );
        assert!((dilation.stationary_factor - 1.0).abs() < 1.0e-6);
        assert!((dilation.orbital_factor - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn time_dilation_near_mass() {
        let mut dilation = GravitationalTimeDilation::default();
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        let r = 10.0 * rs_val;
        assert_eq!(
            rel_gravitational_time_dilation(r, SOLAR_MASS, G, &mut dilation),
            Bool::TRUE
        );
        let expected = (1.0 - 1.0_f64 / 10.0).sqrt();
        assert!((dilation.stationary_factor - expected).abs() < 1.0e-12);
    }

    #[test]
    fn length_contraction_at_half_c() {
        let mut contraction = LengthContraction::default();
        assert_eq!(
            rel_length_contraction(10.0, 0.5 * C, &mut contraction),
            Bool::TRUE
        );
        let expected_gamma = 1.0 / (1.0 - 0.5_f64.powi(2)).sqrt();
        assert!((contraction.lorentz_factor - expected_gamma).abs() < 1.0e-12);
        assert!((contraction.speed_ratio - 0.5).abs() < 1.0e-12);
        assert!((contraction.contracted_length - 10.0 / expected_gamma).abs() < 1.0e-12);
    }

    #[test]
    fn length_contraction_zero_length() {
        let mut contraction = LengthContraction::default();
        assert_eq!(
            rel_length_contraction(0.0, 0.8 * C, &mut contraction),
            Bool::TRUE
        );
        assert_eq!(contraction.contracted_length, 0.0);
    }

    #[test]
    fn particle_properties_at_rest() {
        let mut particle = RelativisticParticle::default();
        assert_eq!(
            rel_particle_properties(1.0, Vec3::default(), &mut particle),
            Bool::TRUE
        );
        assert!((particle.lorentz_factor - 1.0).abs() < 1.0e-12);
        assert!((particle.total_energy - C * C).abs() < 1.0);
        assert!(particle.kinetic_energy.abs() < 1.0);
        assert_eq!(particle.momentum_magnitude, 0.0);
        assert!(particle.rapidity.abs() < 1.0e-12);
    }

    #[test]
    fn particle_properties_with_speed() {
        let mut particle = RelativisticParticle::default();
        let speed = 0.6 * C;
        let v = Vec3 {
            x: speed,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            rel_particle_properties(2.0, v, &mut particle),
            Bool::TRUE
        );
        let expected_gamma = 1.0 / (1.0 - 0.6_f64.powi(2)).sqrt();
        assert!((particle.lorentz_factor - expected_gamma).abs() < 1.0e-10);
        assert!(
            (particle.total_energy - expected_gamma * 2.0 * C * C).abs() < 1.0
        );
        assert!(
            (particle.kinetic_energy - (expected_gamma - 1.0) * 2.0 * C * C).abs() < 1.0
        );
        assert!((particle.momentum_magnitude - expected_gamma * 2.0 * speed).abs() < 1.0);
    }

    #[test]
    fn velocity_addition_less_than_c() {
        let u = Vec3 {
            x: 0.6 * C,
            y: 0.0,
            z: 0.0,
        };
        let v = Vec3 {
            x: 0.6 * C,
            y: 0.0,
            z: 0.0,
        };
        let mut result = Vec3::default();
        assert_eq!(rel_velocity_addition(u, v, &mut result), Bool::TRUE);
        // 1D relativistic addition: w = (u+v) / (1 + uv/c^2)
        let expected = (0.6 + 0.6) / (1.0 + 0.36) * C;
        assert!((result.x - expected).abs() < 1.0);
        assert!(result.x < C);
    }

    #[test]
    fn velocity_addition_with_zero() {
        let u = Vec3 {
            x: 0.5 * C,
            y: 0.0,
            z: 0.0,
        };
        let v = Vec3::default();
        let mut result = Vec3::default();
        assert_eq!(rel_velocity_addition(u, v, &mut result), Bool::TRUE);
        assert!((result.x - 0.5 * C).abs() < 1.0);
    }

    #[test]
    fn invariant_mass_conserved() {
        let speed = 0.8 * C;
        let mass = 3.0;
        let mut particle = RelativisticParticle::default();
        assert_eq!(
            rel_particle_properties(
                mass,
                Vec3 {
                    x: speed,
                    y: 0.0,
                    z: 0.0,
                },
                &mut particle,
            ),
            Bool::TRUE
        );
        let m0 = rel_invariant_mass(
            particle.total_energy,
            particle.momentum.x,
            particle.momentum.y,
            particle.momentum.z,
        );
        assert!(m0.is_finite());
        assert!((m0 - mass).abs() < 1.0e-6);
    }

    #[test]
    fn light_deflection_sun() {
        let solar_radius = 6.957e8;
        let angle = rel_light_deflection_angle(solar_radius, SOLAR_MASS, G);
        assert!(angle.is_finite());
        // Classical Einstein deflection ~ 8.48e-6 radians (1.75 arcseconds)
        let expected = 4.0 * G * SOLAR_MASS / (solar_radius * C * C);
        assert!((angle - expected).abs() < 1.0e-12);
        // Convert to arcseconds
        let arcsec = angle * 180.0 / std::f64::consts::PI * 3600.0;
        assert!((arcsec - 1.75).abs() < 0.1);
    }

    #[test]
    fn error_on_superspeed() {
        let mut gamma = 0.0;
        assert_eq!(
            rel_lorentz_factor(C * 1.1, &mut gamma),
            Bool::FALSE
        );
        let mut contraction = LengthContraction::default();
        assert_eq!(
            rel_length_contraction(1.0, C * 1.1, &mut contraction),
            Bool::FALSE
        );
    }

    #[test]
    fn error_on_null_pointer() {
        assert_eq!(
            rel_lorentz_factor(0.0, std::ptr::null_mut()),
            Bool::FALSE
        );
    }

    #[test]
    fn schwarzschild_radius_solar() {
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        assert!(rs_val.is_finite());
        let expected = 2.0 * G * SOLAR_MASS / (C * C);
        assert!((rs_val - expected).abs() / expected < 0.01);
        // Solar Schwarzschild radius ~ 2953 m
        assert!((rs_val - 2953.0).abs() < 100.0);
    }

    #[test]
    fn rapidity_consistent_with_gamma() {
        let speed = 0.8 * C;
        let mut gamma = 0.0;
        assert_eq!(rel_lorentz_factor(speed, &mut gamma), Bool::TRUE);
        let phi = rel_rapidity(speed);
        assert!(phi.is_finite());
        // cosh(phi) = gamma
        assert!((phi.cosh() - gamma).abs() < 1.0e-10);
    }

    #[test]
    fn beta_from_gamma_roundtrip() {
        let mut gamma = 0.0;
        assert_eq!(rel_lorentz_factor(0.9 * C, &mut gamma), Bool::TRUE);
        let beta = rel_beta_from_gamma(gamma);
        assert!(beta.is_finite());
        assert!((beta - 0.9).abs() < 1.0e-6);
    }

    #[test]
    fn effective_potential_outside_horizon() {
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        let r = 10.0 * rs_val;
        let l = 1.0e15; // some angular momentum
        let mut potential = 0.0;
        assert_eq!(
            rel_effective_potential(r, l, 1000.0, G, &mut potential),
            Bool::TRUE
        );
        assert!(potential.is_finite());
    }

    #[test]
    fn gravitational_time_dilation_simple_works() {
        let rs_val = rel_schwarzschild_radius(SOLAR_MASS, G);
        let r = 4.0 * rs_val;
        let factor = rel_gravitational_time_dilation_simple(r, rs_val);
        assert!(factor.is_finite());
        assert!((factor - (1.0 - 0.25_f64).sqrt()).abs() < 1.0e-12);
    }

    #[test]
    fn photon_particle_returns_infinity() {
        let mut particle = RelativisticParticle::default();
        assert_eq!(
            rel_particle_properties(
                0.0,
                Vec3 {
                    x: C,
                    y: 0.0,
                    z: 0.0,
                },
                &mut particle,
            ),
            Bool::TRUE
        );
        assert!(particle.lorentz_factor.is_infinite());
        assert!(particle.total_energy.is_infinite());
    }

    #[test]
    fn massless_below_c_is_rejected() {
        let mut particle = RelativisticParticle::default();
        assert_eq!(
            rel_particle_properties(
                0.0,
                Vec3 {
                    x: 0.5 * C,
                    y: 0.0,
                    z: 0.0,
                },
                &mut particle,
            ),
            Bool::FALSE
        );
    }
}



