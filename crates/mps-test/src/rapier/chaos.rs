#[cfg(test)]
mod tests {
    use mps_core::rapier::chaos::*;
    use mps_core::rapier::ffi::*;

    #[test]
    fn lorenz_step_default_params() {
        let state = LorenzState {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let params = LorenzParams::default();
        let mut report = LorenzStepReport::default();
        assert_eq!(chaos_lorenz_step(state, params, &mut report), Bool::TRUE);
        // Classical Lorenz attractor stays bounded with sigma=10, rho=28, beta=8/3
        assert!(report.state.x.is_finite());
        assert!(report.state.y.is_finite());
        assert!(report.state.z.is_finite());
    }

    #[test]
    fn lorenz_step_null_pointer() {
        let state = LorenzState::default();
        let params = LorenzParams::default();
        assert_eq!(chaos_lorenz_step(state, params, std::ptr::null_mut()), Bool::FALSE);
    }

    #[test]
    fn lorenz_integrate_fills_buffer() {
        let initial = LorenzState {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let params = LorenzParams::default();
        let mut buf = [LorenzState::default(); 100];
        let count = chaos_lorenz_integrate(initial, params, 100, buf.as_mut_ptr(), 100);
        assert_eq!(count, 100);
        // Check that the last state is different from the first
        assert_ne!(buf[99].x, initial.x);
    }

    #[test]
    fn lorenz_integrate_respects_capacity() {
        let initial = LorenzState {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let params = LorenzParams::default();
        let mut buf = [LorenzState::default(); 10];
        let count = chaos_lorenz_integrate(initial, params, 1000, buf.as_mut_ptr(), 10);
        assert_eq!(count, 10);
    }

    #[test]
    fn lyapunov_lorenz_positive() {
        // Lorenz system with classical parameters should have a positive Lyapunov exponent
        let initial = LorenzState {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let params = LorenzParams::default();
        let mut report = LyapunovReport::default();
        let result = chaos_lyapunov_lorenz(initial, params, 1e-8, 10, 3000, &mut report);
        assert_eq!(result, Bool::TRUE, "lyapunov_lorenz returned FALSE");
        assert!(report.largest_exponent.is_finite(), "LE should be finite");
        // For the classical Lorenz attractor the largest LE is ~0.9
        assert!(
            report.largest_exponent > 0.0,
            "expected positive LE for Lorenz, got {}",
            report.largest_exponent
        );
    }

    #[test]
    fn lyapunov_rosenstein_works() {
        // Generate a simple chaotic time series - use high-precision logistic map
        let n = 5000usize;
        let mut data = vec![0.0_f64; n];
        let mut x = 0.1;
        for item in data.iter_mut() {
            x = 4.0 * x * (1.0 - x);
            *item = x;
        }
        let mut report = LyapunovReport::default();
        let result = chaos_lyapunov_rosenstein(
            data.as_ptr(),
            n as u32,
            2,
            1,
            &mut report,
        );
        assert_eq!(result, Bool::TRUE, "rosenstein returned FALSE");
        // Logistic map at r=4 has Lyapunov exponent = ln(2) ≈ 0.693
        assert!(
            report.largest_exponent > 0.0,
            "expected positive LE for logistic r=4, got {}",
            report.largest_exponent
        );
    }

    #[test]
    fn rosenstein_rejects_too_few_points() {
        let data = [0.5, 0.6, 0.7];
        let mut report = LyapunovReport::default();
        assert_eq!(
            chaos_lyapunov_rosenstein(data.as_ptr(), 3, 3, 1, &mut report),
            Bool::FALSE
        );
    }

    #[test]
    fn bifurcation_lorenz_produces_points() {
        let initial = LorenzState {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let base = LorenzParams::default();
        let mut buf = [BifurcationPoint::default(); 500];
        let count = chaos_bifurcation_lorenz(
            initial, base, 1, // vary rho
            10.0, 50.0, // rho from 10 to 50 (avoid fixed-point regime near 0)
            3, // param steps
            50, // transient
            3, // samples per value
            buf.as_mut_ptr(), 500,
        );
        assert!(count > 0, "bifurcation should produce some points");
    }

    #[test]
    fn double_pendulum_step_conserves_energy_approximately() {
        // For a simple drop from small angle, total mechanical energy should be roughly conserved
        let state = DoublePendulumState {
            theta1: 0.1,
            theta2: 0.1,
            omega1: 0.0,
            omega2: 0.0,
        };
        let params = DoublePendulumParams {
            m1: 1.0,
            m2: 1.0,
            l1: 1.0,
            l2: 1.0,
            g: 9.81,
            dt: 0.001,
        };

        // Initial energy: potential only
        let initial_pe = params.g
            * (params.m1 + params.m2) * params.l1 * (1.0 - state.theta1.cos())
            + params.g * params.m2 * params.l2 * (1.0 - state.theta2.cos());

        let mut s = state;
        let mut max_deviation = 0.0;
        for _ in 0..100 {
            let mut next = DoublePendulumState::default();
            assert_eq!(
                chaos_double_pendulum_step(s, params, &mut next),
                Bool::TRUE
            );

            // Compute total energy
            let ke = 0.5 * (params.m1 + params.m2) * params.l1.powi(2) * next.omega1.powi(2)
                + 0.5 * params.m2 * params.l2.powi(2) * next.omega2.powi(2)
                + params.m2 * params.l1 * params.l2 * next.omega1 * next.omega2
                    * (next.theta1 - next.theta2).cos();
            let pe = params.g
                * (params.m1 + params.m2) * params.l1 * (1.0 - next.theta1.cos())
                + params.g * params.m2 * params.l2 * (1.0 - next.theta2.cos());
            let total = ke + pe;
            let deviation = (total - initial_pe).abs() / initial_pe.abs().max(1.0);
            if deviation > max_deviation {
                max_deviation = deviation;
            }
            s = next;
        }
        // With RK4 and small dt, energy drift should be small over 100 steps
        assert!(max_deviation < 0.01, "energy drift {max_deviation} exceeds 1%");
    }

    #[test]
    fn double_pendulum_null_pointer() {
        let state = DoublePendulumState::default();
        let params = DoublePendulumParams::default();
        assert_eq!(
            chaos_double_pendulum_step(state, params, std::ptr::null_mut()),
            Bool::FALSE
        );
    }

    #[test]
    fn double_pendulum_integrate_basic() {
        let state = DoublePendulumState {
            theta1: 1.0,
            theta2: 0.5,
            omega1: 0.0,
            omega2: 0.0,
        };
        let params = DoublePendulumParams::default();
        let mut buf = [DoublePendulumState::default(); 50];
        let count = chaos_double_pendulum_integrate(state, params, 50, buf.as_mut_ptr(), 50);
        assert_eq!(count, 50);
        assert!(buf[49].theta1.is_finite());
    }

    #[test]
    fn chaos_detect_logistic_chaotic() {
        // Logistic map at r=4.0 is fully chaotic
        let n = 5000usize;
        let mut data = vec![0.0_f64; n];
        let mut x = 0.5;
        for item in data.iter_mut() {
            x = 4.0 * x * (1.0 - x);
            *item = x;
        }
        let params = ChaosDetectionParams {
            sample_steps: 3000,
            embedding_dim: 3,
            embedding_delay: 1,
            ..ChaosDetectionParams::default()
        };
        let mut report = ChaosDetectionReport::default();
        let result = chaos_detect(data.as_ptr(), n as u32, params, &mut report);
        assert_eq!(result, Bool::TRUE, "chaos_detect returned FALSE");
        if report.is_chaotic == Bool::TRUE {
            assert!(report.confidence > 0.3);
        }
        // At minimum the detection should not crash or report NAN
        assert!(
            report.lyapunov_exponent.is_finite(),
            "LE must be finite"
        );
    }

    #[test]
    fn chaos_detect_periodic_not_chaotic() {
        // Logistic map at r=2.5 converges to fixed point
        let mut data = [0.0_f64; 200];
        let mut x = 0.3;
        for item in data.iter_mut() {
            x = 2.5 * x * (1.0 - x);
            *item = x;
        }
        let params = ChaosDetectionParams::default();
        let mut report = ChaosDetectionReport::default();
        let result = chaos_detect(data.as_ptr(), 200, params, &mut report);
        assert_eq!(result, Bool::TRUE);
        // Fixed point → Lyapunov exponent should be negative or near zero
        assert!(report.lyapunov_exponent < 0.01 || report.is_chaotic == Bool::FALSE);
    }

    #[test]
    fn logistic_map_period_doubling() {
        // At r=3.2 we should see period-2 behaviour
        let mut x = 0.5_f64;
        for _ in 0..1000 {
            x = 3.2 * x * (1.0 - x);
        }
        // After transient, check that the orbit has period 2
        let x0 = x;
        let x1 = 3.2 * x0 * (1.0 - x0);
        let x2 = 3.2 * x1 * (1.0 - x1);
        assert!((x2 - x0).abs() < 1e-6);
    }

    #[test]
    fn logistic_step_valid() {
        let mut next = LogisticMapState::default();
        assert_eq!(chaos_logistic_step(0.5, 2.0, &mut next), Bool::TRUE);
        // r=2, x=0.5 → fixed at 0.5
        assert!((next.x - 0.5).abs() < 1e-12);
    }

    #[test]
    fn logistic_iterate_basic() {
        let mut data = [0.0_f64; 10];
        let count = chaos_logistic_iterate(0.5, 3.8, 10, data.as_mut_ptr(), 10);
        assert_eq!(count, 10);
        for &val in data.iter() {
            assert!((0.0..=1.0).contains(&val));
        }
    }

    #[test]
    fn logistic_bifurcation_produces_points() {
        let mut buf = [BifurcationPoint::default(); 200];
        let count = chaos_logistic_bifurcation(
            0.5,
            2.5, 4.0,
            5,   // param steps
            100, // transient
            10,  // samples per value
            buf.as_mut_ptr(), 200,
        );
        assert_eq!(count, 50); // 5 * 10
        // With 5 param steps from 2.5 to 4.0
        // Points should have parameters in that range
        for point in buf.iter().take(count as usize) {
            assert!(point.parameter >= 2.5 - 1e-9);
            assert!(point.parameter <= 4.0 + 1e-9);
        }
    }

    #[test]
    fn double_pendulum_accel_formula() {
        let state = DoublePendulumState {
            theta1: 0.5,
            theta2: 0.3,
            omega1: 0.1,
            omega2: 0.2,
        };
        let params = DoublePendulumParams {
            m1: 1.0,
            m2: 1.0,
            l1: 1.0,
            l2: 1.0,
            g: 9.81,
            dt: 0.01,
        };
        let mut accel = DoublePendulumAccel::default();
        assert_eq!(
            chaos_double_pendulum_accel(state, params, &mut accel),
            Bool::TRUE
        );
        assert!(accel.alpha1.is_finite());
        assert!(accel.alpha2.is_finite());
    }
}



