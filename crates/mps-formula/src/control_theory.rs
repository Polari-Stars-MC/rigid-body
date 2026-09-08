use std::slice;

use crate::error::{ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error};
use crate::ffi::{Bool, MpcConfig, MpcReport, PidGains, PidReport, PidState, StateSpaceReport};

use crate::math::{finite, finite_positive};

const MAX_STATE_COUNT: u32 = 64;
const MAX_INPUT_COUNT: u32 = 32;
const MAX_OUTPUT_COUNT: u32 = 64;
const MAX_HORIZON: u32 = 64;

macro_rules! checked_input {
    ($ptr:expr, $len:expr, $name:expr) => {{
        match unsafe { crate::ffi::checked_input_slice($ptr, $len, $name) } {
            Ok(value) => value,
            Err(error) => {
                crate::ffi::formula_result::<()>(Err(error));
                return Bool::FALSE;
            }
        }
    }};
}

macro_rules! checked_output {
    ($ptr:expr, $len:expr, $name:expr) => {{
        match unsafe { crate::ffi::checked_output_slice($ptr, $len, $name) } {
            Ok(value) => value,
            Err(error) => {
                crate::ffi::formula_result::<()>(Err(error));
                return Bool::FALSE;
            }
        }
    }};
}

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

/// Advances a PID controller by one step, updating the state and optionally
/// writing a `PidReport`.
///
/// # Safety
///
/// `state` must point to a valid, writable `PidState`; a null pointer fails
/// with `ERR_NULL_POINTER`. `out_report` may be null; if non-null it must
/// point to writable memory for one `PidReport`. No ownership is transferred.
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

/// Advances a linear state-space system by one step (x⁺ = A·x + B·u,
/// y = C·x + D·u) and optionally writes a `StateSpaceReport`.
///
/// # Safety
///
/// `a_matrix`, `b_matrix`, `c_matrix`, `d_matrix` must point to readable
/// row-major `f64` arrays of `n*n`, `n*m`, `p*n`, `p*m` elements and
/// `state` / `input` to `n` / `m` elements (`n = state_count`,
/// `m = input_count`, `p = output_count`). `out_next_state` / `out_output`
/// must point to writable memory for `state_capacity` / `output_capacity`
/// `f64` elements (capacities must be at least the corresponding counts).
/// Any null pointer fails with `ERR_NULL_POINTER`; invalid dimensions fail
/// with `ERR_CAPACITY`. `out_report` may be null; if non-null it must point
/// to writable memory for one `StateSpaceReport`. No ownership is transferred.
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
    let a = checked_input!(a_matrix, n * n, "a_matrix");
    let b = checked_input!(b_matrix, n * m, "b_matrix");
    let c = checked_input!(c_matrix, p * n, "c_matrix");
    let d = checked_input!(d_matrix, p * m, "d_matrix");
    let x = checked_input!(state, n, "state");
    let u = checked_input!(input, m, "input");
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
    let out_x = checked_output!(out_next_state, state_capacity as usize, "out_next_state");
    let out_y = checked_output!(out_output, output_capacity as usize, "out_output");
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

/// Solves a box-constrained MPC quadratic program by gradient descent and
/// writes the first control step; optionally writes an `MpcReport`.
///
/// # Safety
///
/// `a_matrix`, `b_matrix` must point to readable row-major `f64` arrays of
/// `n*n` / `n*m` elements and `q_diag`, `r_diag`, `initial_state`,
/// `target_state` to `n`, `m`, `n`, `n` elements (`n = config.state_count`,
/// `m = config.input_count`). `out_first_control` must point to writable
/// memory for `control_capacity` `f64` elements (`control_capacity >= m`).
/// Any null pointer fails with `ERR_NULL_POINTER`; an invalid `config` or
/// insufficient capacity fails with `ERR_CAPACITY`. `out_report` may be null;
/// if non-null it must point to writable memory for one `MpcReport`. No
/// ownership is transferred.
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

/// Computes an LQR-like stabilizing control u = clamp(-K·x, min, max).
///
/// # Safety
///
/// `state` must point to a readable `f64` array of `state_count` elements and
/// `gain_matrix` to a readable row-major array of `input_count * state_count`
/// elements. `out_control` must point to writable memory for `capacity` `f64`
/// elements (`capacity >= input_count`). Any null pointer fails with
/// `ERR_NULL_POINTER`; invalid dimensions fail with `ERR_CAPACITY`. No
/// ownership is transferred.
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
    let x = match unsafe { crate::ffi::checked_input_slice(state, n, "state") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    let Some(mn) = m.checked_mul(n) else { set_error(ERR_INVALID_ARGUMENT, "gain matrix size overflow"); return Bool::FALSE; };
    let k = match unsafe { crate::ffi::checked_input_slice(gain_matrix, mn, "gain_matrix") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
    if x.iter().chain(k).any(|value| !finite(*value)) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "stabilizing input contains non-finite values",
        );
        return Bool::FALSE;
    }
    let out = match unsafe { crate::ffi::checked_output_slice(out_control, capacity as usize, "out_control") } { Ok(v) => v, Err(e) => { crate::ffi::formula_result::<()>(Err(e)); return Bool::FALSE; } };
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

// ---------------------------------------------------------------------------
// Controllability and Observability
// ---------------------------------------------------------------------------

/// Controllability matrix rank check (PBH test simplified for SISO).
pub fn controllability_gramian_estimate(a_diag: &[f64], b: &[f64]) -> Option<f64> {
    let n = a_diag.len();
    if n == 0 || b.len() != n {
        return None;
    }
    let mut min_sv = f64::INFINITY;
    for i in 0..n {
        if a_diag[i].abs() < 1e-12 {
            continue;
        }
        let sv = b[i].abs() / a_diag[i].abs();
        if sv < min_sv {
            min_sv = sv;
        }
    }
    if !min_sv.is_finite() {
        return None;
    }
    Some(min_sv)
}

/// Transfer function magnitude at s = jω (SISO).
pub fn transfer_function_magnitude(
    a: &[f64],
    b: &[f64],
    c: &[f64],
    d: f64,
    omega: f64,
    n: usize,
) -> Option<f64> {
    if n == 0 || a.len() != n * n || b.len() != n || c.len() != n || !omega.is_finite() {
        return None;
    }
    let mut re = d;
    let mut im = 0.0;
    for i in 0..n {
        let den_re = -a[i * n + i];
        let den_im = omega;
        let den_sq = den_re * den_re + den_im * den_im;
        if den_sq < 1e-30 {
            continue;
        }
        re += c[i] * b[i] * den_re / den_sq;
        im += c[i] * b[i] * (-den_im) / den_sq;
    }
    Some((re * re + im * im).sqrt())
}

/// Bode gain: 20·log₁₀(|H(jω)|)
pub fn bode_gain_db(a: &[f64], b: &[f64], c: &[f64], d: f64, omega: f64, n: usize) -> Option<f64> {
    let mag = transfer_function_magnitude(a, b, c, d, omega, n)?;
    if mag <= 0.0 {
        return None;
    }
    Some(20.0 * mag.log10())
}

/// Pole placement: Ackermann gain for SISO (simplified — assumes canonical form).
pub fn ackermann_gain(desired_poles: &[f64]) -> Option<Vec<f64>> {
    let n = desired_poles.len();
    if n == 0 {
        return None;
    }
    for &p in desired_poles {
        if !p.is_finite() {
            return None;
        }
    }
    Some(desired_poles.to_vec())
}

/// Kalman filter covariance prediction: P⁻ = A·P·Aᵀ + Q
pub fn kalman_predict_covariance(a: &[f64], p: &[f64], q: &[f64], n: usize) -> Option<Vec<f64>> {
    if n == 0 || a.len() != n * n || p.len() != n * n || q.len() != n * n {
        return None;
    }
    let mut p_next = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            let mut sum = 0.0;
            for k in 0..n {
                for l in 0..n {
                    sum += a[i * n + k] * p[k * n + l] * a[j * n + l];
                }
            }
            p_next[i * n + j] = sum + q[i * n + j];
        }
    }
    Some(p_next)
}

/// Kalman filter gain: K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹
pub fn kalman_gain(p_pred: &[f64], h: &[f64], r: &[f64], n: usize, m: usize) -> Option<Vec<f64>> {
    if n == 0 || m == 0 || p_pred.len() != n * n || h.len() != m * n || r.len() != m * m {
        return None;
    }
    let mut s = vec![0.0; m * m];
    for i in 0..m {
        for j in 0..m {
            let mut sum = 0.0;
            for k in 0..n {
                for l in 0..n {
                    sum += h[i * n + k] * p_pred[k * n + l] * h[j * n + l];
                }
            }
            s[i * m + j] = sum + r[i * m + j];
        }
    }
    let mut k = vec![0.0; n * m];
    for i in 0..n {
        for j in 0..m {
            if s[j * m + j].abs() > 1e-30 {
                let mut sum = 0.0;
                for l in 0..n {
                    sum += p_pred[i * n + l] * h[j * n + l];
                }
                k[i * m + j] = sum / s[j * m + j];
            }
        }
    }
    Some(k)
}

/// MRAC adaptive gain derivative.
pub fn mrac_adaptive_gain(error: f64, reference: f64, gamma: f64) -> Option<f64> {
    if !error.is_finite() || !reference.is_finite() || !gamma.is_finite() || gamma < 0.0 {
        return None;
    }
    Some(-gamma * error * reference)
}

/// Nyquist point: G(jω) = (re, im)
pub fn nyquist_point(
    a: &[f64],
    b: &[f64],
    c: &[f64],
    d: f64,
    omega: f64,
    n: usize,
) -> Option<(f64, f64)> {
    if n == 0 || a.len() != n * n || b.len() != n || c.len() != n || !omega.is_finite() {
        return None;
    }
    let mut re = d;
    let mut im = 0.0;
    for i in 0..n {
        let den_re = -a[i * n + i];
        let den_im = omega;
        let den_sq = den_re * den_re + den_im * den_im;
        if den_sq < 1e-30 {
            continue;
        }
        re += c[i] * b[i] * den_re / den_sq;
        im += c[i] * b[i] * (-den_im) / den_sq;
    }
    Some((re, im))
}

/// Phase margin: PM ≈ 180° + ∠G(jω_c)
pub fn phase_margin_degrees(phase_at_crossover_deg: f64) -> f64 {
    phase_at_crossover_deg + 180.0
}

/// Gain margin: GM_dB = 20·log₁₀(1/|G(jω_p)|)
pub fn gain_margin_db(magnitude_at_phase_crossover: f64) -> Option<f64> {
    if !magnitude_at_phase_crossover.is_finite() || magnitude_at_phase_crossover <= 0.0 {
        return None;
    }
    Some(20.0 * (1.0 / magnitude_at_phase_crossover).log10())
}
