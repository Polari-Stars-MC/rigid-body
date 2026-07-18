use std::slice;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, MpcConfig, MpcReport, PidGains, PidReport, PidState, StateSpaceReport,
};

use crate::rapier::math::{finite, finite_positive};

const MAX_STATE_COUNT: u32 = 64;
const MAX_INPUT_COUNT: u32 = 32;
const MAX_OUTPUT_COUNT: u32 = 64;
const MAX_HORIZON: u32 = 64;

fn vec_norm(values: &[f64]) -> f64 {
    values.iter().map(|value| value * value).sum::<f64>().sqrt()
}

fn mat_vec(matrix: &[f64], rows: usize, cols: usize, vector: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0; rows];
    for row in 0..rows {
        let mut sum = 0.0;
        for col in 0..cols {
            sum += matrix[row * cols + col] * vector[col];
        }
        out[row] = sum;
    }
    out
}

fn mpc_config_valid(config: MpcConfig) -> bool {
    config.state_count > 0
        && config.state_count <= MAX_STATE_COUNT
        && config.input_count > 0
        && config.input_count <= MAX_INPUT_COUNT
        && config.horizon > 0
        && config.horizon <= MAX_HORIZON
        && finite_positive(config.dt)
        && finite(config.control_min)
        && finite(config.control_max)
        && config.control_min <= config.control_max
        && config.gradient_iterations <= 10_000
        && finite_positive(config.step_size)
}

fn simulate_mpc_cost(
    a: &[f64],
    b: &[f64],
    q_diag: &[f64],
    r_diag: &[f64],
    initial_state: &[f64],
    target_state: &[f64],
    controls: &[f64],
    config: MpcConfig,
) -> f64 {
    let n = config.state_count as usize;
    let m = config.input_count as usize;
    let horizon = config.horizon as usize;
    let mut state = initial_state.to_vec();
    let mut cost = 0.0;
    for step in 0..horizon {
        let control = &controls[step * m..(step + 1) * m];
        for i in 0..n {
            let error = state[i] - target_state[i];
            cost += q_diag[i] * error * error;
        }
        for i in 0..m {
            cost += r_diag[i] * control[i] * control[i];
        }
        let ax = mat_vec(a, n, n, &state);
        let bu = mat_vec(b, n, m, control);
        for i in 0..n {
            state[i] = ax[i] + bu[i];
        }
    }
    for i in 0..n {
        let error = state[i] - target_state[i];
        cost += q_diag[i] * error * error;
    }
    cost
}

#[unsafe(no_mangle)]
pub extern "C" fn control_pid_step(
    setpoint: f64,
    measurement: f64,
    dt: f64,
    gains: PidGains,
    state: *mut PidState,
    out_report: *mut PidReport,
) -> Bool {
    if !finite(setpoint)
        || !finite(measurement)
        || !finite_positive(dt)
        || !finite(gains.kp)
        || !finite(gains.ki)
        || !finite(gains.kd)
        || !finite(gains.output_min)
        || !finite(gains.output_max)
        || !finite(gains.integral_min)
        || !finite(gains.integral_max)
        || gains.output_min > gains.output_max
        || gains.integral_min > gains.integral_max
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid PID parameters");
        return Bool::FALSE;
    }
    let Some(state) = (unsafe { state.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "PID state is null");
        return Bool::FALSE;
    };
    if !finite(state.integral) || !finite(state.previous_error) {
        set_error(ERR_INVALID_ARGUMENT, "invalid PID state");
        return Bool::FALSE;
    }
    let error = setpoint - measurement;
    state.integral = (state.integral + error * dt).clamp(gains.integral_min, gains.integral_max);
    let derivative = (error - state.previous_error) / dt;
    let unclamped_output = gains.kp * error + gains.ki * state.integral + gains.kd * derivative;
    let output = unclamped_output.clamp(gains.output_min, gains.output_max);
    state.previous_error = error;
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = PidReport {
            error,
            integral: state.integral,
            derivative,
            unclamped_output,
            output,
        };
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn control_state_space_step(
    a_matrix: *const f64,
    b_matrix: *const f64,
    c_matrix: *const f64,
    d_matrix: *const f64,
    state: *const f64,
    input: *const f64,
    state_count: u32,
    input_count: u32,
    output_count: u32,
    out_next_state: *mut f64,
    out_output: *mut f64,
    state_capacity: u32,
    output_capacity: u32,
    out_report: *mut StateSpaceReport,
) -> Bool {
    if state_count == 0
        || state_count > MAX_STATE_COUNT
        || input_count == 0
        || input_count > MAX_INPUT_COUNT
        || output_count == 0
        || output_count > MAX_OUTPUT_COUNT
        || state_capacity < state_count
        || output_capacity < output_count
    {
        set_error(ERR_CAPACITY, "invalid state-space dimensions");
        return Bool::FALSE;
    }
    if a_matrix.is_null()
        || b_matrix.is_null()
        || c_matrix.is_null()
        || d_matrix.is_null()
        || state.is_null()
        || input.is_null()
        || out_next_state.is_null()
        || out_output.is_null()
    {
        set_error(ERR_NULL_POINTER, "state-space pointers are null");
        return Bool::FALSE;
    }
    let n = state_count as usize;
    let m = input_count as usize;
    let p = output_count as usize;
    let a = unsafe { slice::from_raw_parts(a_matrix, n * n) };
    let b = unsafe { slice::from_raw_parts(b_matrix, n * m) };
    let c = unsafe { slice::from_raw_parts(c_matrix, p * n) };
    let d = unsafe { slice::from_raw_parts(d_matrix, p * m) };
    let x = unsafe { slice::from_raw_parts(state, n) };
    let u = unsafe { slice::from_raw_parts(input, m) };
    if a.iter()
        .chain(b)
        .chain(c)
        .chain(d)
        .chain(x)
        .chain(u)
        .any(|value| !finite(*value))
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "state-space inputs contain non-finite values",
        );
        return Bool::FALSE;
    }
    let ax = mat_vec(a, n, n, x);
    let bu = mat_vec(b, n, m, u);
    let cx = mat_vec(c, p, n, x);
    let du = mat_vec(d, p, m, u);
    let out_x = unsafe { slice::from_raw_parts_mut(out_next_state, state_capacity as usize) };
    let out_y = unsafe { slice::from_raw_parts_mut(out_output, output_capacity as usize) };
    let mut max_state_delta = 0.0;
    for i in 0..n {
        out_x[i] = ax[i] + bu[i];
        max_state_delta = f64::max(max_state_delta, (out_x[i] - x[i]).abs());
    }
    for i in 0..p {
        out_y[i] = cx[i] + du[i];
    }
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = StateSpaceReport {
            state_count,
            input_count,
            output_count,
            max_state_delta,
            output_norm: vec_norm(&out_y[..p]),
        };
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn control_mpc_solve_box_qp(
    a_matrix: *const f64,
    b_matrix: *const f64,
    q_diag: *const f64,
    r_diag: *const f64,
    initial_state: *const f64,
    target_state: *const f64,
    config: MpcConfig,
    out_first_control: *mut f64,
    control_capacity: u32,
    out_report: *mut MpcReport,
) -> Bool {
    if !mpc_config_valid(config) || control_capacity < config.input_count {
        set_error(ERR_CAPACITY, "invalid MPC configuration");
        return Bool::FALSE;
    }
    if a_matrix.is_null()
        || b_matrix.is_null()
        || q_diag.is_null()
        || r_diag.is_null()
        || initial_state.is_null()
        || target_state.is_null()
        || out_first_control.is_null()
    {
        set_error(ERR_NULL_POINTER, "MPC pointers are null");
        return Bool::FALSE;
    }
    let n = config.state_count as usize;
    let m = config.input_count as usize;
    let horizon = config.horizon as usize;
    let a = unsafe { slice::from_raw_parts(a_matrix, n * n) };
    let b = unsafe { slice::from_raw_parts(b_matrix, n * m) };
    let q = unsafe { slice::from_raw_parts(q_diag, n) };
    let r = unsafe { slice::from_raw_parts(r_diag, m) };
    let x0 = unsafe { slice::from_raw_parts(initial_state, n) };
    let x_target = unsafe { slice::from_raw_parts(target_state, n) };
    if a.iter()
        .chain(b)
        .chain(q)
        .chain(r)
        .chain(x0)
        .chain(x_target)
        .any(|value| !finite(*value))
        || q.iter().any(|value| *value < 0.0)
        || r.iter().any(|value| *value < 0.0)
    {
        set_error(ERR_INVALID_ARGUMENT, "MPC inputs contain invalid values");
        return Bool::FALSE;
    }

    let mut controls = vec![0.0; horizon * m];
    let initial_cost = simulate_mpc_cost(a, b, q, r, x0, x_target, &controls, config);
    let eps = 1.0e-5;
    let iterations = config.gradient_iterations.max(1);
    for _ in 0..iterations {
        for i in 0..controls.len() {
            let original = controls[i];
            controls[i] = (original + eps).clamp(config.control_min, config.control_max);
            let plus_cost = simulate_mpc_cost(a, b, q, r, x0, x_target, &controls, config);
            controls[i] = (original - eps).clamp(config.control_min, config.control_max);
            let minus_cost = simulate_mpc_cost(a, b, q, r, x0, x_target, &controls, config);
            let gradient = (plus_cost - minus_cost) / (2.0 * eps);
            controls[i] = (original - config.step_size.min(0.01) * gradient)
                .clamp(config.control_min, config.control_max);
        }
    }
    let final_cost = simulate_mpc_cost(a, b, q, r, x0, x_target, &controls, config);
    let out = unsafe { slice::from_raw_parts_mut(out_first_control, control_capacity as usize) };
    out[..m].copy_from_slice(&controls[..m]);
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = MpcReport {
            horizon: config.horizon,
            iterations,
            initial_cost,
            final_cost,
            first_control_norm: vec_norm(&controls[..m]),
        };
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn control_lqr_like_stabilizing_input(
    state: *const f64,
    gain_matrix: *const f64,
    state_count: u32,
    input_count: u32,
    control_min: f64,
    control_max: f64,
    out_control: *mut f64,
    capacity: u32,
) -> Bool {
    if state_count == 0
        || state_count > MAX_STATE_COUNT
        || input_count == 0
        || input_count > MAX_INPUT_COUNT
        || capacity < input_count
        || !finite(control_min)
        || !finite(control_max)
        || control_min > control_max
    {
        set_error(ERR_CAPACITY, "invalid stabilizing input dimensions");
        return Bool::FALSE;
    }
    if state.is_null() || gain_matrix.is_null() || out_control.is_null() {
        set_error(ERR_NULL_POINTER, "stabilizing input pointers are null");
        return Bool::FALSE;
    }
    let n = state_count as usize;
    let m = input_count as usize;
    let x = unsafe { slice::from_raw_parts(state, n) };
    let k = unsafe { slice::from_raw_parts(gain_matrix, m * n) };
    if x.iter().chain(k).any(|value| !finite(*value)) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "stabilizing input contains non-finite values",
        );
        return Bool::FALSE;
    }
    let out = unsafe { slice::from_raw_parts_mut(out_control, capacity as usize) };
    for row in 0..m {
        let mut value = 0.0;
        for col in 0..n {
            value -= k[row * n + col] * x[col];
        }
        out[row] = value.clamp(control_min, control_max);
    }
    clear_error();
    Bool::TRUE
}

#[cfg(test)]
mod tests {
    use super::*;

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
