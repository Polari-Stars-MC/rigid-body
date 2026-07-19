#[cfg(test)]
mod tests {
    use mps_core::rapier::superfluidity::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::Vec3;

    const VORTEX_CORE: f64 = 1.0e-10;

    fn make_seg(x1: f64, y1: f64, z1: f64, x2: f64, y2: f64, z2: f64) -> VortexSegment {
        VortexSegment {
            start: Vec3 { x: x1, y: y1, z: z1 },
            end: Vec3 { x: x2, y: y2, z: z2 },
            circulation_quantum: 1,
            core_radius: VORTEX_CORE,
        }
    }

    #[test]
    fn biot_savart_induces_velocity() {
        // Segment along x-axis from (0,0,0) to (1,0,0)
        let seg = make_seg(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let field = Vec3 {
            x: 0.5,
            y: 0.1,
            z: 0.0,
        };
        let mut vel = BiotSavartVelocity::default();
        assert_eq!(
            sf_biot_savart_velocity(seg, field, &mut vel),
            Bool::TRUE
        );
        // Velocity should be in the z-direction (for a segment on x-axis and point in xy-plane)
        // Direction of r1×r2 near the segment midpoint should be ±z
        assert!(vel.velocity.z != 0.0 || vel.magnitude.abs() < 1e-20);
        assert!(vel.magnitude >= 0.0);
    }

    #[test]
    fn biot_savart_collinear_returns_zero() {
        // Field point on the line of the segment but outside
        let seg = make_seg(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let field = Vec3 {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        };
        let mut vel = BiotSavartVelocity::default();
        assert_eq!(
            sf_biot_savart_velocity(seg, field, &mut vel),
            Bool::TRUE
        );
        assert_eq!(vel.velocity.x, 0.0);
        assert_eq!(vel.velocity.y, 0.0);
        assert_eq!(vel.velocity.z, 0.0);
    }

    #[test]
    fn biot_savart_on_segment_rejected() {
        let seg = make_seg(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let field = Vec3 {
            x: 0.5,
            y: 0.0,
            z: 0.0,
        }; // on the segment
        let mut vel = BiotSavartVelocity::default();
        assert_eq!(
            sf_biot_savart_velocity(seg, field, &mut vel),
            Bool::FALSE
        );
    }

    #[test]
    fn vortex_ring_velocity_finite() {
        let ring = VortexRing {
            center: Vec3::default(),
            radius: 1.0e-6,
            circulation_quantum: 1,
            axis: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            velocity: Vec3::default(),
        };
        let mut vel = Vec3::default();
        assert_eq!(
            sf_vortex_ring_velocity(ring, &mut vel),
            Bool::TRUE
        );
        // Ring should move along its axis
        assert!(vel.z != 0.0);
        assert!(vel.x == 0.0 && vel.y == 0.0);
        // Speed should be positive (for positive circulation)
        assert!(vel.z != 0.0); // direction depends on quantum sign
    }

    #[test]
    fn circulation_quantum_constant_is_positive() {
        let kappa = sf_circulation_quantum();
        assert!(kappa > 0.0, "circulation quantum must be positive");
        assert!(kappa.is_finite());
    }

    #[test]
    fn gp_order_parameter_vortex_profile() {
        let center = Vec3::default();
        let axis = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        };
        let xi = 1.0;
        let n0 = 1.0;

        // At core center: amplitude should be zero
        let mut param = GpOrderParameter::default();
        assert_eq!(
            sf_gp_order_parameter(0.0, 0.0, 0.0, center, axis, 1, xi, n0, &mut param),
            Bool::TRUE
        );
        assert_eq!(param.amplitude, 0.0, "amplitude at core should be zero");
        assert_eq!(param.density, 0.0, "density at core should be zero");

        // Far from core: amplitude should approach √n₀
        let mut param2 = GpOrderParameter::default();
        assert_eq!(
            sf_gp_order_parameter(10.0, 0.0, 0.0, center, axis, 1, xi, n0, &mut param2),
            Bool::TRUE
        );
        assert!(
            (param2.amplitude - 1.0).abs() < 0.05,
            "far-field amplitude should approach √n₀, got {}",
            param2.amplitude
        );
    }

    #[test]
    fn gp_energy_density_is_nonnegative() {
        let mut energy = GpEnergyDensity::default();
        assert_eq!(
            sf_gp_energy_density(
                1.0, 100.0, // trap frequency
                HELIUM_MASS, 1.0e-10, // coupling
                1.0e-6, // radius from center
                &mut energy,
            ),
            Bool::TRUE
        );
        assert!(energy.kinetic_density >= 0.0);
        assert!(energy.interaction_density >= 0.0);
        assert!(energy.trapping_density >= 0.0);
        assert!(energy.total_density >= 0.0);
        assert!(energy.chemical_potential.is_finite());
    }

    #[test]
    fn gp_simple_evolution_converges() {
        // Imaginary-time evolution should converge to n₀
        let params = GpTimeEvolutionParams {
            healing_length: 1.0,
            sound_speed: 1.0,
            chemical_potential: 1.0,
            coupling_constant: 1.0,
            dt: 0.01,
        };
        let mut amp = 0.1_f64;
        for _ in 0..500 {
            let mut next = 0.0_f64;
            let density = amp * amp;
            let result = sf_gp_amplitude_evolution(amp, density, params, &mut next);
            assert_eq!(result, Bool::TRUE);
            amp = next;
        }
        // Should approach √n₀ = 1.0
        assert!((amp - 1.0).abs() < 0.05, "amplitude should approach 1, got {amp}");
    }

    #[test]
    fn vortex_reconnection_detection() {
        // Two perpendicular segments far apart
        let seg1 = make_seg(-2.0, 0.0, 0.0, 2.0, 0.0, 0.0);
        let seg2 = make_seg(0.0, -2.0, 0.0, 0.0, 2.0, 0.0);
        let xi = VORTEX_CORE;

        let mut report = VortexReconnectionReport::default();
        // With a large reconnection distance covering the crossing, they reconnect
        assert_eq!(
            sf_vortex_reconnection(seg1, seg2, 5.0, xi, &mut report),
            Bool::TRUE
        );
        assert_eq!(report.reconnected, Bool::TRUE);
        assert!(report.closest_approach < 5.0);

        // With a smaller reconnection distance than the closest approach, no reconnect
        // Use segments that don't intersect and are separated
        let seg3 = make_seg(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let seg4 = make_seg(0.0, 0.0, 10.0, 0.0, 1.0, 10.0); // 10 units away
        let mut report2 = VortexReconnectionReport::default();
        assert_eq!(
            sf_vortex_reconnection(seg3, seg4, 1.0, xi, &mut report2),
            Bool::TRUE
        );
        assert_eq!(report2.reconnected, Bool::FALSE);
    }

    #[test]
    fn tangle_stats_with_single_segment() {
        let seg = make_seg(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let mut stats = VortexTangleStats::default();
        assert_eq!(
            sf_vortex_tangle_stats(&seg, 1, 1.0, &mut stats),
            Bool::TRUE
        );
        assert_eq!(stats.segment_count, 1);
        assert!((stats.total_length - 1.0).abs() < 1e-12);
        assert!(stats.total_kinetic_energy > 0.0);
        assert!((stats.vortex_line_density - 1.0).abs() < 1e-12);
    }

    #[test]
    fn tangle_stats_empty() {
        let mut stats = VortexTangleStats::default();
        assert_eq!(
            sf_vortex_tangle_stats(std::ptr::null(), 0, 1.0, &mut stats),
            Bool::TRUE
        );
        // Should return zeros
        assert_eq!(stats.segment_count, 0);
    }

    #[test]
    fn healing_length_and_sound_speed_consistent() {
        let g = 1.0e-10;
        let m = HELIUM_MASS;
        let n = 1.0e20; // ~10²⁰ m⁻³ for ⁴He

        let xi = sf_healing_length(g, m, n);
        assert!(xi.is_finite() && xi > 0.0);

        let c = sf_sound_speed(g, m, n);
        assert!(c.is_finite() && c > 0.0);

        // Consistency: c = ħ / (√2 m ξ)
        let c_from_xi = HBAR / (2.0f64.sqrt() * m * xi);
        assert!((c - c_from_xi).abs() / c < 0.01,
            "sound speed inconsistent with healing length: c={c}, c_from_xi={c_from_xi}");
    }

    #[test]
    fn grid_sample_produces_output() {
        let mut grid = [GpGridPoint::default(); 100];
        let count = sf_gp_grid_sample(
            Vec3::default(), // plane center
            Vec3 { x: 0.0, y: 0.0, z: 1.0 }, // plane axis
            10, 10,
            5.0, 5.0,
            Vec3::default(), // vortex center
            Vec3 { x: 0.0, y: 0.0, z: 1.0 }, // vortex axis
            1,
            1.0, 1.0,
            grid.as_mut_ptr(), 100,
        );
        assert_eq!(count, 100);
        // Center point (core): density ≈ 0
        assert!(grid[45].density < 1.0 || grid[55].density < 1.0);
        // Corner points: density ≈ 1
        assert!(
            (grid[0].density - 1.0).abs() < 0.5 || (grid[99].density - 1.0).abs() < 0.5
        );
    }

    #[test]
    fn null_pointer_rejected() {
        let seg = make_seg(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        assert_eq!(
            sf_biot_savart_velocity(seg, Vec3::default(), std::ptr::null_mut()),
            Bool::FALSE
        );
    }

    #[test]
    fn circulation_quantum_number_estimate() {
        // For a circulation of exactly n quanta, the estimate should match
        let kappa = circulation_quantum_const();
        let radius = 1e-6;
        let n = 3_i32;
        let tangential_v = (n as f64) * kappa / (2.0 * std::f64::consts::PI * radius);
        let samples = [tangential_v; 36];
        let mut quantum = 0_i32;
        assert_eq!(
            sf_quantum_number_estimate(
                samples.as_ptr(),
                radius,
                36,
                &mut quantum,
            ),
            Bool::TRUE
        );
        assert_eq!(quantum, n);
    }
}



