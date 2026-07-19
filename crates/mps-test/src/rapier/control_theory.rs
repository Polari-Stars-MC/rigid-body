#[cfg(test)]
mod tests {
    use mps_core::rapier::control_theory::*;
    use mps_core::rapier::ffi::*;

    #[test]
    fn pid_step_updates_integral_and_output() {
        let mut state = PidState::default();
        let mut report = PidReport::default();
        assert_eq!(
            control_pid_step(
                1.0,
                0.25,
                0.1,
                PidGains {
                    kp: 2.0,
                    ki: 0.5,
                    kd: 0.1,
                    output_min: -10.0,
                    output_max: 10.0,
                    integral_min: -1.0,
                    integral_max: 1.0,
                },
                &mut state,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(report.output > 0.0);
        assert!(state.integral > 0.0);
    }

    #[test]
    fn state_space_step_works() {
        let a = [1.0, 0.1, 0.0, 1.0];
        let b = [0.0, 0.1];
        let c = [1.0, 0.0];
        let d = [0.0];
        let x = [0.0, 1.0];
        let u = [2.0];
        let mut next = [0.0; 2];
        let mut y = [0.0; 1];
        let mut report = StateSpaceReport::default();
        assert_eq!(
            control_state_space_step(
                a.as_ptr(),
                b.as_ptr(),
                c.as_ptr(),
                d.as_ptr(),
                x.as_ptr(),
                u.as_ptr(),
                2,
                1,
                1,
                next.as_mut_ptr(),
                y.as_mut_ptr(),
                2,
                1,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(next[0], 0.1);
        assert_eq!(next[1], 1.2);
        assert_eq!(y[0], 0.0);
    }

    #[test]
    fn mpc_reduces_cost_for_integrator() {
        let a = [1.0];
        let b = [1.0];
        let q = [1.0];
        let r = [0.1];
        let x0 = [2.0];
        let target = [0.0];
        let mut control = [0.0];
        let mut report = MpcReport::default();
        assert_eq!(
            control_mpc_solve_box_qp(
                a.as_ptr(),
                b.as_ptr(),
                q.as_ptr(),
                r.as_ptr(),
                x0.as_ptr(),
                target.as_ptr(),
                MpcConfig {
                    state_count: 1,
                    input_count: 1,
                    horizon: 4,
                    dt: 1.0,
                    control_min: -1.0,
                    control_max: 1.0,
                    gradient_iterations: 30,
                    step_size: 0.05,
                },
                control.as_mut_ptr(),
                1,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(control[0] < 0.0);
        assert!(report.final_cost < report.initial_cost);
    }
}



