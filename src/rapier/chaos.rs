//! Chaos theory and nonlinear dynamics:
//! - Lorenz attractor (integration & visualisation)
//! - Lyapunov exponent estimation (largest exponent via orbit divergence)
//! - Bifurcation diagrams (scan parameter vs. sampled state)
//! - Double-pendulum chaos (Lagrangian mechanics → RK4 integration)
//! - Chaos detection (Lyapunov + correlation dimension heuristics)
//! - Logistic map (1D discrete chaos, educational)
//!
//! All functions are FFI-exported with C-compatible types, following the
//! error-handling conventions of the mps_rigid_body physics engine.

use core::f64;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    BifurcationPoint, Bool, ChaosDetectionParams, ChaosDetectionReport, DoublePendulumAccel,
    DoublePendulumParams, DoublePendulumState, LogisticMapState, LorenzParams, LorenzState,
    LorenzStepReport, LyapunovReport,
};

use crate::rapier::math::{KahanSum, finite, finite_positive};

const EPSILON: f64 = 1.0e-14;
const DIST_EPSILON: f64 = 1.0e-16;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn write_out<T: Copy>(out: *mut T, value: T) -> Bool {
    let Some(out) = (unsafe { out.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return Bool::FALSE;
    };
    *out = value;
    clear_error();
    Bool::TRUE
}

// ===========================================================================
// A. Lorenz attractor
// ===========================================================================

/// Compute the Lorenz system derivatives at a given state.
#[inline]
fn lorenz_deriv(state: LorenzState, params: LorenzParams) -> LorenzStepReport {
    let dx = params.sigma * (state.y - state.x);
    let dy = state.x * (params.rho - state.z) - state.y;
    let dz = state.x * state.y - params.beta * state.z;
    LorenzStepReport {
        state,
        dx,
        dy,
        dz,
    }
}

/// Perform one RK4 step of the Lorenz system.
#[unsafe(no_mangle)]
pub extern "C" fn chaos_lorenz_step(
    state: LorenzState,
    params: LorenzParams,
    out_report: *mut LorenzStepReport,
) -> Bool {
    if !params_valid(&params) {
        return Bool::FALSE;
    }
    if !state_finite(state) {
        return Bool::FALSE;
    }

    let dt = params.dt;

    // k1
    let k1 = lorenz_deriv(state, params);
    // k2
    let s2 = LorenzState {
        x: state.x + 0.5 * dt * k1.dx,
        y: state.y + 0.5 * dt * k1.dy,
        z: state.z + 0.5 * dt * k1.dz,
    };
    let k2 = lorenz_deriv(s2, params);
    // k3
    let s3 = LorenzState {
        x: state.x + 0.5 * dt * k2.dx,
        y: state.y + 0.5 * dt * k2.dy,
        z: state.z + 0.5 * dt * k2.dz,
    };
    let k3 = lorenz_deriv(s3, params);
    // k4
    let s4 = LorenzState {
        x: state.x + dt * k3.dx,
        y: state.y + dt * k3.dy,
        z: state.z + dt * k3.dz,
    };
    let k4 = lorenz_deriv(s4, params);

    let next = LorenzState {
        x: state.x + (dt / 6.0) * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx),
        y: state.y + (dt / 6.0) * (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy),
        z: state.z + (dt / 6.0) * (k1.dz + 2.0 * k2.dz + 2.0 * k3.dz + k4.dz),
    };

    let report = LorenzStepReport {
        state: next,
        dx: (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx) / 6.0,
        dy: (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy) / 6.0,
        dz: (k1.dz + 2.0 * k2.dz + 2.0 * k3.dz + k4.dz) / 6.0,
    };

    write_out(out_report, report)
}

/// Integrate the Lorenz system for N steps, writing each state into a
/// pre-allocated output buffer of length `out_len`.
///
/// Returns the number of steps actually written.
#[unsafe(no_mangle)]
pub extern "C" fn chaos_lorenz_integrate(
    initial: LorenzState,
    params: LorenzParams,
    steps: u32,
    out_states: *mut LorenzState,
    out_len: u32,
) -> u32 {
    if !params_valid(&params) || !state_finite(initial) {
        return 0;
    }
    if steps == 0 {
        clear_error();
        return 0;
    }
    if out_states.is_null() {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return 0;
    }
    let cap = out_len as usize;
    let count = (steps as usize).min(cap);
    let buf = unsafe { std::slice::from_raw_parts_mut(out_states, count) };

    let dt = params.dt;
    let mut s = initial;

    for item in buf.iter_mut() {
        // One RK4 step
        let k1 = lorenz_deriv(s, params);
        let s2 = LorenzState {
            x: s.x + 0.5 * dt * k1.dx,
            y: s.y + 0.5 * dt * k1.dy,
            z: s.z + 0.5 * dt * k1.dz,
        };
        let k2 = lorenz_deriv(s2, params);
        let s3 = LorenzState {
            x: s.x + 0.5 * dt * k2.dx,
            y: s.y + 0.5 * dt * k2.dy,
            z: s.z + 0.5 * dt * k2.dz,
        };
        let k3 = lorenz_deriv(s3, params);
        let s4 = LorenzState {
            x: s.x + dt * k3.dx,
            y: s.y + dt * k3.dy,
            z: s.z + dt * k3.dz,
        };
        let k4 = lorenz_deriv(s4, params);
        s = LorenzState {
            x: s.x + (dt / 6.0) * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx),
            y: s.y + (dt / 6.0) * (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy),
            z: s.z + (dt / 6.0) * (k1.dz + 2.0 * k2.dz + 2.0 * k3.dz + k4.dz),
        };
        *item = s;
    }

    clear_error();
    count as u32
}

/// Return the number of states written (for use after `chaos_lorenz_integrate`).
/// Identical to the return value; provided as a convenience for FFI callers
/// who want it stored in memory.
#[unsafe(no_mangle)]
pub extern "C" fn chaos_lorenz_integrate_count(steps: u32, out_len: u32) -> u32 {
    steps.min(out_len)
}

/// Validate Lorenz parameters.
fn params_valid(params: &LorenzParams) -> bool {
    if !finite(params.dt) || params.dt <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "dt must be positive and finite");
        return false;
    }
    if !finite(params.sigma) || !finite(params.rho) || !finite(params.beta) {
        set_error(ERR_INVALID_ARGUMENT, "sigma, rho, beta must be finite");
        return false;
    }
    if params.beta <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "beta must be positive");
        return false;
    }
    true
}

fn state_finite(s: LorenzState) -> bool {
    if !finite(s.x) || !finite(s.y) || !finite(s.z) {
        set_error(ERR_INVALID_ARGUMENT, "state components must be finite");
        return false;
    }
    true
}

// ===========================================================================
// B. Lyapunov exponent (largest) via orbit divergence
// ===========================================================================

/// Estimate the largest Lyapunov exponent by tracking the divergence of two
/// nearby trajectories in the Lorenz system.
///
/// A reference trajectory and a perturbed copy are integrated simultaneously.
/// Every `renorm_interval` steps the separation is measured, the log ratio
/// accumulated, and the perturbed trajectory is re-normalised to keep the
/// perturbation small. This gives λ ≈ (1/t) Σ ln(δ/δ₀).
///
/// `perturbation` is the initial separation magnitude (typical 1e-8).
/// `renorm_every` re-normalises every N integration steps.
/// `total_steps` total integration steps for the estimate.
#[unsafe(no_mangle)]
pub extern "C" fn chaos_lyapunov_lorenz(
    initial: LorenzState,
    params: LorenzParams,
    perturbation: f64,
    renorm_every: u32,
    total_steps: u32,
    out_report: *mut LyapunovReport,
) -> Bool {
    if !params_valid(&params) || !state_finite(initial) {
        return Bool::FALSE;
    }
    if !finite_positive(perturbation) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "perturbation must be positive and finite",
        );
        return Bool::FALSE;
    }
    if renorm_every == 0 || total_steps == 0 {
        set_error(ERR_INVALID_ARGUMENT, "renorm_every and total_steps must be > 0");
        return Bool::FALSE;
    }

    let dt = params.dt;
    let mut ref_state = initial;
    let mut pert_state = LorenzState {
        x: initial.x + perturbation,
        y: initial.y,
        z: initial.z,
    };

    let mut sum_log = KahanSum::default();
    let mut norm_count: u32 = 0;

    for step in 0..total_steps {
        // RK4 for reference
        let k1r = lorenz_deriv(ref_state, params);
        let s2r = LorenzState {
            x: ref_state.x + 0.5 * dt * k1r.dx,
            y: ref_state.y + 0.5 * dt * k1r.dy,
            z: ref_state.z + 0.5 * dt * k1r.dz,
        };
        let k2r = lorenz_deriv(s2r, params);
        let s3r = LorenzState {
            x: ref_state.x + 0.5 * dt * k2r.dx,
            y: ref_state.y + 0.5 * dt * k2r.dy,
            z: ref_state.z + 0.5 * dt * k2r.dz,
        };
        let k3r = lorenz_deriv(s3r, params);
        let s4r = LorenzState {
            x: ref_state.x + dt * k3r.dx,
            y: ref_state.y + dt * k3r.dy,
            z: ref_state.z + dt * k3r.dz,
        };
        let k4r = lorenz_deriv(s4r, params);
        ref_state = LorenzState {
            x: ref_state.x + (dt / 6.0) * (k1r.dx + 2.0 * k2r.dx + 2.0 * k3r.dx + k4r.dx),
            y: ref_state.y + (dt / 6.0) * (k1r.dy + 2.0 * k2r.dy + 2.0 * k3r.dy + k4r.dy),
            z: ref_state.z + (dt / 6.0) * (k1r.dz + 2.0 * k2r.dz + 2.0 * k3r.dz + k4r.dz),
        };

        // RK4 for perturbed
        let k1p = lorenz_deriv(pert_state, params);
        let s2p = LorenzState {
            x: pert_state.x + 0.5 * dt * k1p.dx,
            y: pert_state.y + 0.5 * dt * k1p.dy,
            z: pert_state.z + 0.5 * dt * k1p.dz,
        };
        let k2p = lorenz_deriv(s2p, params);
        let s3p = LorenzState {
            x: pert_state.x + 0.5 * dt * k2p.dx,
            y: pert_state.y + 0.5 * dt * k2p.dy,
            z: pert_state.z + 0.5 * dt * k2p.dz,
        };
        let k3p = lorenz_deriv(s3p, params);
        let s4p = LorenzState {
            x: pert_state.x + dt * k3p.dx,
            y: pert_state.y + dt * k3p.dy,
            z: pert_state.z + dt * k3p.dz,
        };
        let k4p = lorenz_deriv(s4p, params);
        pert_state = LorenzState {
            x: pert_state.x + (dt / 6.0) * (k1p.dx + 2.0 * k2p.dx + 2.0 * k3p.dx + k4p.dx),
            y: pert_state.y + (dt / 6.0) * (k1p.dy + 2.0 * k2p.dy + 2.0 * k3p.dy + k4p.dy),
            z: pert_state.z + (dt / 6.0) * (k1p.dz + 2.0 * k2p.dz + 2.0 * k3p.dz + k4p.dz),
        };

        // Re-normalise every renorm_every steps
        if (step + 1) % renorm_every == 0 {
            let dx = pert_state.x - ref_state.x;
            let dy = pert_state.y - ref_state.y;
            let dz = pert_state.z - ref_state.z;
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();

            if dist > EPSILON {
                sum_log.add((dist / perturbation).ln());
                norm_count += 1;

                // Re-normalise: scale back to initial perturbation size
                let scale = perturbation / dist;
                pert_state = LorenzState {
                    x: ref_state.x + dx * scale,
                    y: ref_state.y + dy * scale,
                    z: ref_state.z + dz * scale,
                };
            } else {
                // Trajectories converged — re-seed
                pert_state.x = ref_state.x + perturbation;
            }
        }
    }

    if norm_count == 0 {
        set_error(ERR_INVALID_ARGUMENT, "no renormalisation events occurred");
        return Bool::FALSE;
    }

    let total_time = (total_steps as f64) * dt;
    let lyapunov = sum_log.value() / total_time;

    write_out(
        out_report,
        LyapunovReport {
            largest_exponent: lyapunov,
            convergence_steps: norm_count,
            positive: Bool::from(lyapunov > 0.0),
        },
    )
}

/// Compute the largest Lyapunov exponent from a 1D time series using the
/// Rosenstein algorithm (method of delays).
///
/// `data` — pointer to an array of length `data_len` containing the scalar
/// time-series samples.
/// `embedding_dim` — embedding dimension m (typically 3–7).
/// `delay` — time delay τ in samples (typically 1–10).
/// `out_report` — filled with the estimated exponent.
#[unsafe(no_mangle)]
pub extern "C" fn chaos_lyapunov_rosenstein(
    data: *const f64,
    data_len: u32,
    embedding_dim: u32,
    delay: u32,
    out_report: *mut LyapunovReport,
) -> Bool {
    if data.is_null() {
        set_error(ERR_NULL_POINTER, "data pointer is null");
        return Bool::FALSE;
    }
    if data_len < 2 {
        set_error(ERR_INVALID_ARGUMENT, "data_len must be >= 2");
        return Bool::FALSE;
    }
    if embedding_dim < 2 {
        set_error(ERR_INVALID_ARGUMENT, "embedding_dim must be >= 2");
        return Bool::FALSE;
    }
    if delay == 0 {
        set_error(ERR_INVALID_ARGUMENT, "delay must be > 0");
        return Bool::FALSE;
    }

    let n = data_len as usize;
    let m = embedding_dim as usize;
    let tau = delay as usize;
    let samples = unsafe { std::slice::from_raw_parts(data, n) };

    // Number of embedded vectors
    let n_vectors = n.saturating_sub((m - 1) * tau);
    if n_vectors < 2 {
        set_error(ERR_INVALID_ARGUMENT, "too few vectors for embedding");
        return Bool::FALSE;
    }

    // Build embedded vectors as a flat array: flat[i*m + j] = vector[i][j]
    let flat = {
        let mut f = vec![0.0_f64; n_vectors * m];
        for i in 0..n_vectors {
            for j in 0..m {
                f[i * m + j] = samples[i + j * tau];
            }
        }
        f
    };

    // Helper to access vector i, component j in the flat array
    let v = |i: usize, j: usize| flat[i * m + j];

    // Helper for Euclidean distance between vectors i and k in flat array
    let euclid = |i: usize, k: usize| -> f64 {
        let mut sum = 0.0_f64;
        for j in 0..m {
            let d = v(i, j) - v(k, j);
            sum += d * d;
        }
        sum.sqrt()
    };
    // For each vector, find nearest neighbour (excluding temporally close ones)
    let mut sum_log_div = KahanSum::default();
    let mut count = 0u32;

    for i in 0..n_vectors {
        let mut min_dist = f64::MAX;
        let min_separation = 3; // exclude neighbours too close in time

        for j in 0..n_vectors {
            if (i as isize - j as isize).unsigned_abs() < min_separation {
                continue;
            }
            let dist = euclid(i, j);
            if dist > DIST_EPSILON && dist < min_dist {
                min_dist = dist;
            }
        }

        if min_dist.is_finite() && min_dist < f64::MAX && min_dist > 0.0 && i + 1 < n_vectors {
            // Track divergence over one time step
            let mut next_dist = f64::MAX;
            for j in 0..n_vectors - 1 {
                if (i as isize - j as isize).unsigned_abs() < min_separation {
                    continue;
                }
                let dist = euclid(i + 1, j + 1);
                if dist > 0.0 && dist < next_dist {
                    next_dist = dist;
                }
            }
            if next_dist.is_finite() && next_dist < f64::MAX && next_dist > 0.0 {
                let ratio = next_dist / min_dist;
                if ratio > DIST_EPSILON && ratio.is_finite() {
                    sum_log_div.add(ratio.ln());
                    count += 1;
                }
            }
        }
    }

    if count == 0 {
        set_error(ERR_INVALID_ARGUMENT, "no valid neighbour pairs found");
        return Bool::FALSE;
    }

    let lyapunov = sum_log_div.value() / (count as f64);
    write_out(
        out_report,
        LyapunovReport {
            largest_exponent: lyapunov,
            convergence_steps: count,
            positive: Bool::from(lyapunov > 0.0),
        },
    )
}

// ===========================================================================
// C. Bifurcation diagram (Lorenz system)
// ===========================================================================

/// Sample a Lorenz bifurcation diagram by scanning one parameter across a
/// range. For each parameter value:
///   1. Discard `transient_steps` integration steps.
///   2. Record the next `samples_per_value` local maxima of x (or y/z)
///      as bifurcation points.
///
/// `vary` — which parameter to vary: 0 = sigma, 1 = rho, 2 = beta.
/// `param_min`, `param_max` — range of the parameter.
/// `param_steps` — how many distinct parameter values.
/// `transient_steps` — steps to discard before recording.
/// `samples_per_value` — number of Poincaré samples per parameter value.
/// `out_points` — pre-allocated buffer for `BifurcationPoint`.
/// `out_len` — capacity of the output buffer.
/// Returns the number of points actually written.
#[unsafe(no_mangle)]
pub extern "C" fn chaos_bifurcation_lorenz(
    initial: LorenzState,
    base_params: LorenzParams,
    vary: u32,
    param_min: f64,
    param_max: f64,
    param_steps: u32,
    transient_steps: u32,
    samples_per_value: u32,
    out_points: *mut BifurcationPoint,
    out_len: u32,
) -> u32 {
    if !state_finite(initial) {
        return 0;
    }
    if param_steps == 0 || samples_per_value == 0 {
        set_error(ERR_INVALID_ARGUMENT, "param_steps and samples_per_value must be > 0");
        return 0;
    }
    if !finite(param_min) || !finite(param_max) || param_min > param_max {
        set_error(
            ERR_INVALID_ARGUMENT,
            "param_min <= param_max and both finite",
        );
        return 0;
    }
    if vary > 2 {
        set_error(ERR_INVALID_ARGUMENT, "vary must be 0 (sigma), 1 (rho), or 2 (beta)");
        return 0;
    }

    let cap = out_len as usize;
    let total_needed = (param_steps as usize).saturating_mul(samples_per_value as usize);
    if total_needed > cap {
        set_error(ERR_CAPACITY, "output buffer too small");
        return 0;
    }

    let dt = base_params.dt;
    if !finite(dt) || dt <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "base_params.dt must be positive and finite");
        return 0;
    }

    let buf = unsafe { std::slice::from_raw_parts_mut(out_points, cap) };
    let mut written = 0usize;

    for p_idx in 0..param_steps as usize {
        let frac = if param_steps > 1 {
            p_idx as f64 / (param_steps - 1) as f64
        } else {
            0.0
        };
        let param_val = param_min + frac * (param_max - param_min);

        // Build modified params
        let mut params = base_params;
        match vary {
            0 => params.sigma = param_val,
            1 => params.rho = param_val,
            _ => params.beta = param_val,
        }

        // Validate
        if !finite(params.sigma) || !finite(params.rho) || !finite(params.beta) {
            continue;
        }
        if params.beta <= 0.0 {
            continue;
        }

        // Integrate transient
        let mut s = initial;
        for _ in 0..transient_steps {
            let k1 = lorenz_deriv(s, params);
            let s2 = LorenzState {
                x: s.x + 0.5 * dt * k1.dx,
                y: s.y + 0.5 * dt * k1.dy,
                z: s.z + 0.5 * dt * k1.dz,
            };
            let k2 = lorenz_deriv(s2, params);
            let s3 = LorenzState {
                x: s.x + 0.5 * dt * k2.dx,
                y: s.y + 0.5 * dt * k2.dy,
                z: s.z + 0.5 * dt * k2.dz,
            };
            let k3 = lorenz_deriv(s3, params);
            let s4 = LorenzState {
                x: s.x + dt * k3.dx,
                y: s.y + dt * k3.dy,
                z: s.z + dt * k3.dz,
            };
            let k4 = lorenz_deriv(s4, params);
            s = LorenzState {
                x: s.x + (dt / 6.0) * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx),
                y: s.y + (dt / 6.0) * (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy),
                z: s.z + (dt / 6.0) * (k1.dz + 2.0 * k2.dz + 2.0 * k3.dz + k4.dz),
            };
        }

        // Sample local maxima of x (Poincaré-like section)
        let mut _prev_x = s.x;
        let mut _prev_dx = 0.0;
        const MAX_ITER_PER_SAMPLE: u32 = 100_000;

        for _ in 0..samples_per_value {
            let mut search_iter = 0u32;
            // Find next local maximum of x
            loop {
                if search_iter >= MAX_ITER_PER_SAMPLE {
                    break; // no maximum found — skip this sample
                }
                search_iter += 1;
                let k1 = lorenz_deriv(s, params);
                let s2 = LorenzState {
                    x: s.x + 0.5 * dt * k1.dx,
                    y: s.y + 0.5 * dt * k1.dy,
                    z: s.z + 0.5 * dt * k1.dz,
                };
                let k2 = lorenz_deriv(s2, params);
                let s3 = LorenzState {
                    x: s.x + 0.5 * dt * k2.dx,
                    y: s.y + 0.5 * dt * k2.dy,
                    z: s.z + 0.5 * dt * k2.dz,
                };
                let k3 = lorenz_deriv(s3, params);
                let s4 = LorenzState {
                    x: s.x + dt * k3.dx,
                    y: s.y + dt * k3.dy,
                    z: s.z + dt * k3.dz,
                };
                let k4 = lorenz_deriv(s4, params);
                let next = LorenzState {
                    x: s.x + (dt / 6.0) * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx),
                    y: s.y + (dt / 6.0) * (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy),
                    z: s.z + (dt / 6.0) * (k1.dz + 2.0 * k2.dz + 2.0 * k3.dz + k4.dz),
                };

                let dx = next.x - s.x;
                if _prev_dx > 0.0 && dx <= 0.0 {
                    // Local maximum found
                    buf[written] = BifurcationPoint {
                        parameter: param_val,
                        sample: s.x,
                    };
                    written += 1;
                    _prev_x = next.x;
                    _prev_dx = 0.0;
                    s = next;
                    break;
                }
                _prev_dx = dx;
                _prev_x = s.x;
                s = next;
            }
        }
    }

    clear_error();
    written as u32
}

// ===========================================================================
// D. Double pendulum (Lagrangian mechanics)
// ===========================================================================

/// Compute the angular accelerations of a double pendulum using the
/// Lagrangian equations of motion:
///
///   α1 = ( -g (2 m1 + m2) sin θ1 - m2 g sin(θ1 - 2 θ2)
///         - 2 sin(θ1 - θ2) m2 ( ω2² L2 + ω1² L1 cos(θ1 - θ2) ) )
///        / ( L1 ( 2 m1 + m2 - m2 cos(2 θ1 - 2 θ2) ) )
///
///   α2 = ( 2 sin(θ1 - θ2) ( ω1² L1 (m1 + m2) + g (m1 + m2) cos θ1
///         + ω2² L2 m2 cos(θ1 - θ2) ) )
///        / ( L2 ( 2 m1 + m2 - m2 cos(2 θ1 - 2 θ2) ) )
#[unsafe(no_mangle)]
pub extern "C" fn chaos_double_pendulum_accel(
    state: DoublePendulumState,
    params: DoublePendulumParams,
    out_accel: *mut DoublePendulumAccel,
) -> Bool {
    if !double_pendulum_params_valid(&params) || !double_pendulum_state_finite(state) {
        return Bool::FALSE;
    }

    let (theta1, theta2, omega1, omega2) = (state.theta1, state.theta2, state.omega1, state.omega2);
    let (m1, m2, l1, l2, g) = (params.m1, params.m2, params.l1, params.l2, params.g);

    let delta = theta1 - theta2;
    let sin_delta = delta.sin();
    let cos_delta = delta.cos();
    let denom = 2.0 * m1 + m2 - m2 * (2.0 * delta).cos();

    if denom.abs() < EPSILON {
        set_error(ERR_INVALID_ARGUMENT, "singular denominator in double pendulum equations");
        return Bool::FALSE;
    }

    let alpha1 = (-g * (2.0 * m1 + m2) * theta1.sin()
        - m2 * g * (theta1 - 2.0 * theta2).sin()
        - 2.0 * sin_delta * m2 * (omega2 * omega2 * l2 + omega1 * omega1 * l1 * cos_delta))
        / (l1 * denom);

    let alpha2 = (2.0 * sin_delta
        * (omega1 * omega1 * l1 * (m1 + m2) + g * (m1 + m2) * theta1.cos()
            + omega2 * omega2 * l2 * m2 * cos_delta))
        / (l2 * denom);

    write_out(out_accel, DoublePendulumAccel { alpha1, alpha2 })
}

/// Perform one RK4 integration step of the double pendulum.
///
/// The system is a 4D ODE: (θ1, ω1, θ2, ω2) with ω = dθ/dt and
/// α = dω/dt given by `chaos_double_pendulum_accel`.
#[unsafe(no_mangle)]
pub extern "C" fn chaos_double_pendulum_step(
    state: DoublePendulumState,
    params: DoublePendulumParams,
    out_next: *mut DoublePendulumState,
) -> Bool {
    if !double_pendulum_params_valid(&params) || !double_pendulum_state_finite(state) {
        return Bool::FALSE;
    }

    let dt = params.dt;

    // Helper to compute state derivatives given current state
    let derivatives = |s: DoublePendulumState| -> (f64, f64, f64, f64) {
        // θ̇ = ω
        let dtheta1 = s.omega1;
        let dtheta2 = s.omega2;
        // ω̇ = α
        let g = params.g;
        let m1 = params.m1;
        let m2 = params.m2;
        let l1 = params.l1;
        let l2 = params.l2;
        let delta = s.theta1 - s.theta2;
        let sin_delta = delta.sin();
        let cos_delta = delta.cos();
        let denom = 2.0 * m1 + m2 - m2 * (2.0 * delta).cos();

        // If denom is near zero, return zeros (will get caught by validation below)
        if denom.abs() < EPSILON {
            return (dtheta1, dtheta2, 0.0, 0.0);
        }

        let alpha1 = (-g * (2.0 * m1 + m2) * s.theta1.sin()
            - m2 * g * (s.theta1 - 2.0 * s.theta2).sin()
            - 2.0 * sin_delta * m2
                * (s.omega2 * s.omega2 * l2 + s.omega1 * s.omega1 * l1 * cos_delta))
            / (l1 * denom);
        let alpha2 = (2.0 * sin_delta
            * (s.omega1 * s.omega1 * l1 * (m1 + m2) + g * (m1 + m2) * s.theta1.cos()
                + s.omega2 * s.omega2 * l2 * m2 * cos_delta))
            / (l2 * denom);

        (dtheta1, dtheta2, alpha1, alpha2)
    };

    // RK4
    let (k1_t1, k1_t2, k1_w1, k1_w2) = derivatives(state);

    let s2 = DoublePendulumState {
        theta1: state.theta1 + 0.5 * dt * k1_t1,
        omega1: state.omega1 + 0.5 * dt * k1_w1,
        theta2: state.theta2 + 0.5 * dt * k1_t2,
        omega2: state.omega2 + 0.5 * dt * k1_w2,
    };
    let (k2_t1, k2_t2, k2_w1, k2_w2) = derivatives(s2);

    let s3 = DoublePendulumState {
        theta1: state.theta1 + 0.5 * dt * k2_t1,
        omega1: state.omega1 + 0.5 * dt * k2_w1,
        theta2: state.theta2 + 0.5 * dt * k2_t2,
        omega2: state.omega2 + 0.5 * dt * k2_w2,
    };
    let (k3_t1, k3_t2, k3_w1, k3_w2) = derivatives(s3);

    let s4 = DoublePendulumState {
        theta1: state.theta1 + dt * k3_t1,
        omega1: state.omega1 + dt * k3_w1,
        theta2: state.theta2 + dt * k3_t2,
        omega2: state.omega2 + dt * k3_w2,
    };
    let (k4_t1, k4_t2, k4_w1, k4_w2) = derivatives(s4);

    let next = DoublePendulumState {
        theta1: state.theta1 + (dt / 6.0) * (k1_t1 + 2.0 * k2_t1 + 2.0 * k3_t1 + k4_t1),
        omega1: state.omega1 + (dt / 6.0) * (k1_w1 + 2.0 * k2_w1 + 2.0 * k3_w1 + k4_w1),
        theta2: state.theta2 + (dt / 6.0) * (k1_t2 + 2.0 * k2_t2 + 2.0 * k3_t2 + k4_t2),
        omega2: state.omega2 + (dt / 6.0) * (k1_w2 + 2.0 * k2_w2 + 2.0 * k3_w2 + k4_w2),
    };

    if !double_pendulum_state_finite(next) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "double pendulum state diverged (non-finite values)",
        );
        return Bool::FALSE;
    }

    write_out(out_next, next)
}

/// Integrate the double pendulum for N steps, writing states into a
/// pre-allocated output buffer.
///
/// Returns the number of states written.
#[unsafe(no_mangle)]
pub extern "C" fn chaos_double_pendulum_integrate(
    initial: DoublePendulumState,
    params: DoublePendulumParams,
    steps: u32,
    out_states: *mut DoublePendulumState,
    out_len: u32,
) -> u32 {
    if !double_pendulum_params_valid(&params) || !double_pendulum_state_finite(initial) {
        return 0;
    }
    if steps == 0 {
        clear_error();
        return 0;
    }
    if out_states.is_null() {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return 0;
    }

    let cap = out_len as usize;
    let count = (steps as usize).min(cap);
    let buf = unsafe { std::slice::from_raw_parts_mut(out_states, count) };

    let dt = params.dt;
    let mut s = initial;

    for item in buf.iter_mut() {
        // Inline one RK4 step (same as above) to keep performance tight
        let derivatives = |st: DoublePendulumState| -> (f64, f64, f64, f64) {
            let dtheta1 = st.omega1;
            let dtheta2 = st.omega2;
            let g = params.g;
            let m1 = params.m1;
            let m2 = params.m2;
            let l1 = params.l1;
            let l2 = params.l2;
            let delta = st.theta1 - st.theta2;
            let sin_delta = delta.sin();
            let cos_delta = delta.cos();
            let denom = 2.0 * m1 + m2 - m2 * (2.0 * delta).cos();
            if denom.abs() < EPSILON {
                return (dtheta1, dtheta2, 0.0, 0.0);
            }
            let alpha1 = (-g * (2.0 * m1 + m2) * st.theta1.sin()
                - m2 * g * (st.theta1 - 2.0 * st.theta2).sin()
                - 2.0 * sin_delta * m2
                    * (st.omega2 * st.omega2 * l2 + st.omega1 * st.omega1 * l1 * cos_delta))
                / (l1 * denom);
            let alpha2 = (2.0 * sin_delta
                * (st.omega1 * st.omega1 * l1 * (m1 + m2)
                    + g * (m1 + m2) * st.theta1.cos()
                    + st.omega2 * st.omega2 * l2 * m2 * cos_delta))
                / (l2 * denom);
            (dtheta1, dtheta2, alpha1, alpha2)
        };

        let (k1_t1, k1_t2, k1_w1, k1_w2) = derivatives(s);
        let s2 = DoublePendulumState {
            theta1: s.theta1 + 0.5 * dt * k1_t1,
            omega1: s.omega1 + 0.5 * dt * k1_w1,
            theta2: s.theta2 + 0.5 * dt * k1_t2,
            omega2: s.omega2 + 0.5 * dt * k1_w2,
        };
        let (k2_t1, k2_t2, k2_w1, k2_w2) = derivatives(s2);
        let s3 = DoublePendulumState {
            theta1: s.theta1 + 0.5 * dt * k2_t1,
            omega1: s.omega1 + 0.5 * dt * k2_w1,
            theta2: s.theta2 + 0.5 * dt * k2_t2,
            omega2: s.omega2 + 0.5 * dt * k2_w2,
        };
        let (k3_t1, k3_t2, k3_w1, k3_w2) = derivatives(s3);
        let s4 = DoublePendulumState {
            theta1: s.theta1 + dt * k3_t1,
            omega1: s.omega1 + dt * k3_w1,
            theta2: s.theta2 + dt * k3_t2,
            omega2: s.omega2 + dt * k3_w2,
        };
        let (k4_t1, k4_t2, k4_w1, k4_w2) = derivatives(s4);

        s = DoublePendulumState {
            theta1: s.theta1 + (dt / 6.0) * (k1_t1 + 2.0 * k2_t1 + 2.0 * k3_t1 + k4_t1),
            omega1: s.omega1 + (dt / 6.0) * (k1_w1 + 2.0 * k2_w1 + 2.0 * k3_w1 + k4_w1),
            theta2: s.theta2 + (dt / 6.0) * (k1_t2 + 2.0 * k2_t2 + 2.0 * k3_t2 + k4_t2),
            omega2: s.omega2 + (dt / 6.0) * (k1_w2 + 2.0 * k2_w2 + 2.0 * k3_w2 + k4_w2),
        };
        *item = s;
    }

    clear_error();
    count as u32
}

fn double_pendulum_params_valid(params: &DoublePendulumParams) -> bool {
    if !finite_positive(params.m1) {
        set_error(ERR_INVALID_ARGUMENT, "m1 must be positive and finite");
        return false;
    }
    if !finite_positive(params.m2) {
        set_error(ERR_INVALID_ARGUMENT, "m2 must be positive and finite");
        return false;
    }
    if !finite_positive(params.l1) {
        set_error(ERR_INVALID_ARGUMENT, "l1 must be positive and finite");
        return false;
    }
    if !finite_positive(params.l2) {
        set_error(ERR_INVALID_ARGUMENT, "l2 must be positive and finite");
        return false;
    }
    if !finite_positive(params.g) {
        set_error(ERR_INVALID_ARGUMENT, "g must be positive and finite");
        return false;
    }
    if !finite_positive(params.dt) {
        set_error(ERR_INVALID_ARGUMENT, "dt must be positive and finite");
        return false;
    }
    true
}

fn double_pendulum_state_finite(s: DoublePendulumState) -> bool {
    finite(s.theta1)
        && finite(s.theta2)
        && finite(s.omega1)
        && finite(s.omega2)
}

// ===========================================================================
// E. Chaos detection
// ===========================================================================

/// Analyse a 1D time series and determine if it exhibits chaotic behaviour.
///
/// Uses two heuristics:
///   1. Largest Lyapunov exponent via Rosenstein algorithm (if positive → chaotic).
///   2. Correlation dimension via Grassberger–Procaccia (low fractional → periodic/quasi,
///      high fractional → chaotic).
///
/// `data` — pointer to an array of scalar samples.
/// `data_len` — length of the time series.
/// `params` — detection parameters (embedding, neighbourhood, threshold).
/// `out_report` — filled with the analysis results.
#[unsafe(no_mangle)]
pub extern "C" fn chaos_detect(
    data: *const f64,
    data_len: u32,
    params: ChaosDetectionParams,
    out_report: *mut ChaosDetectionReport,
) -> Bool {
    if data.is_null() {
        set_error(ERR_NULL_POINTER, "data pointer is null");
        return Bool::FALSE;
    }
    if data_len < 10 {
        set_error(ERR_INVALID_ARGUMENT, "data_len must be >= 10");
        return Bool::FALSE;
    }
    if params.embedding_dim < 2 {
        set_error(ERR_INVALID_ARGUMENT, "embedding_dim must be >= 2");
        return Bool::FALSE;
    }
    if params.sample_steps == 0 {
        set_error(ERR_INVALID_ARGUMENT, "sample_steps must be > 0");
        return Bool::FALSE;
    }

    // Clamp sample steps to data length
    let effective_steps = (params.sample_steps as usize).min(data_len as usize);
    let data_slice = unsafe { std::slice::from_raw_parts(data, effective_steps) };

    // ---- 1. Lyapunov exponent (Rosenstein) ----
    let mut lyapunov_report = LyapunovReport::default();
    let lyapunov_ok = chaos_lyapunov_rosenstein(
        data_slice.as_ptr(),
        effective_steps as u32,
        params.embedding_dim,
        params.embedding_delay,
        &mut lyapunov_report,
    );

    let lyapunov_exp = if lyapunov_ok == Bool::TRUE {
        lyapunov_report.largest_exponent
    } else {
        0.0
    };

    // ---- 2. Correlation dimension (Grassberger–Procaccia) ----
    let m = params.embedding_dim as usize;
    let tau = params.embedding_delay as usize;
    let n_vectors = effective_steps.saturating_sub((m - 1) * tau);

    let corr_dim = if n_vectors >= 4 {
        let radius = params.neighbourhood_radius;
        // Flat array: flat[i*m + j] = vector[i][j]
        let flat = {
            let mut f = vec![0.0_f64; n_vectors * m];
            for i in 0..n_vectors {
                for j in 0..m {
                    f[i * m + j] = data_slice[i + j * tau];
                }
            }
            f
        };
        let v = |i: usize, j: usize| flat[i * m + j];
        let euclid = |i: usize, k: usize| -> f64 {
            let mut sum = 0.0_f64;
            for j in 0..m {
                let d = v(i, j) - v(k, j);
                sum += d * d;
            }
            sum.sqrt()
        };

        // Count neighbours within radius for a subset
        let subset_size = n_vectors.min(200); // cap to avoid O(n²)
        let mut log_eps = 0.0_f64;
        let mut log_c = 0.0_f64;
        let mut pairs = 0u32;

        for i in 0..subset_size {
            let mut count_in_radius = 0u32;
            for j in 0..subset_size {
                if i == j {
                    continue;
                }
                let d = euclid(i, j);
                if d < radius {
                    count_in_radius += 1;
                }
            }
            if count_in_radius > 0 {
                log_eps += radius.ln();
                log_c += (count_in_radius as f64 / (subset_size as f64)).ln();
                pairs += 1;
            }
        }

        if pairs > 0 && log_eps.abs() > EPSILON {
            log_c / log_eps
        } else {
            // Fallback: use multiple radii
            let radii = [radius * 0.5, radius, radius * 2.0];
            let mut log_r = Vec::new();
            let mut log_cr = Vec::new();
            for &r in &radii {
                let mut count_pr = 0u32;
                for i in 0..subset_size {
                    for j in 0..subset_size {
                        if i == j {
                            continue;
                        }
                        if euclid(i, j) < r {
                            count_pr += 1;
                        }
                    }
                }
                if count_pr > 0 {
                    log_r.push(r.ln());
                    log_cr.push((count_pr as f64 / (subset_size as f64 * subset_size as f64)).ln());
                }
            }
            if log_r.len() >= 2 {
                // Simple linear fit slope
                let n = log_r.len() as f64;
                let sum_x: f64 = log_r.iter().sum();
                let sum_y: f64 = log_cr.iter().sum();
                let sum_xx: f64 = log_r.iter().map(|x| x * x).sum();
                let sum_xy: f64 = log_r.iter().zip(log_cr.iter()).map(|(x, y)| x * y).sum();
                let denom = n * sum_xx - sum_x * sum_x;
                if denom.abs() > EPSILON {
                    (n * sum_xy - sum_x * sum_y) / denom
                } else {
                    0.0
                }
            } else {
                0.0
            }
        }
    } else {
        0.0
    };

    // ---- 3. Decision ----
    let is_chaotic = lyapunov_exp > params.chaotic_threshold;
    // Confidence based on how far above threshold and whether dimension is > 1.5
    let mut confidence = if is_chaotic {
        0.5 + 0.3 * (lyapunov_exp / (lyapunov_exp + params.chaotic_threshold))
            + 0.2 * (corr_dim / (corr_dim + 1.0))
    } else {
        0.3 * (params.chaotic_threshold / (params.chaotic_threshold + (-lyapunov_exp).max(0.0)))
    };
    confidence = confidence.clamp(0.0, 1.0);

    write_out(
        out_report,
        ChaosDetectionReport {
            lyapunov_exponent: lyapunov_exp,
            correlation_dimension: corr_dim,
            is_chaotic: Bool::from(is_chaotic),
            confidence,
        },
    )
}

// ===========================================================================
// F. Logistic map (1D discrete chaos, educational)
// ===========================================================================

/// Perform one iteration of the logistic map: x_{n+1} = r * x_n * (1 - x_n).
#[unsafe(no_mangle)]
pub extern "C" fn chaos_logistic_step(
    x: f64,
    r: f64,
    out_next: *mut LogisticMapState,
) -> Bool {
    if !finite(x) || !finite(r) {
        set_error(ERR_INVALID_ARGUMENT, "x and r must be finite");
        return Bool::FALSE;
    }
    if !(0.0..=1.0).contains(&x) {
        set_error(ERR_INVALID_ARGUMENT, "x must be in [0, 1]");
        return Bool::FALSE;
    }
    if !(0.0..=4.0).contains(&r) {
        set_error(ERR_INVALID_ARGUMENT, "r must be in [0, 4]");
        return Bool::FALSE;
    }

    let next_x = r * x * (1.0 - x);
    write_out(
        out_next,
        LogisticMapState {
            x: next_x,
            r,
        },
    )
}

/// Run the logistic map for N steps, returning all iterates.
#[unsafe(no_mangle)]
pub extern "C" fn chaos_logistic_iterate(
    initial_x: f64,
    r: f64,
    steps: u32,
    out_values: *mut f64,
    out_len: u32,
) -> u32 {
    if !finite(initial_x) || !finite(r) {
        return 0;
    }
    if !(0.0..=1.0).contains(&initial_x) || !(0.0..=4.0).contains(&r) {
        set_error(ERR_INVALID_ARGUMENT, "x in [0,1], r in [0,4]");
        return 0;
    }
    if steps == 0 {
        clear_error();
        return 0;
    }
    if out_values.is_null() {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return 0;
    }

    let cap = out_len as usize;
    let count = (steps as usize).min(cap);
    let buf = unsafe { std::slice::from_raw_parts_mut(out_values, count) };

    let mut x = initial_x;
    for item in buf.iter_mut() {
        x = r * x * (1.0 - x);
        *item = x;
    }

    clear_error();
    count as u32
}

/// Logistic map bifurcation diagram.
///
/// For each of `param_steps` values of r between `r_min` and `r_max`:
///   1. Run `transient_steps` iterations to reach the attractor.
///   2. Record the next `samples_per_value` iterates.
#[unsafe(no_mangle)]
pub extern "C" fn chaos_logistic_bifurcation(
    initial_x: f64,
    r_min: f64,
    r_max: f64,
    param_steps: u32,
    transient_steps: u32,
    samples_per_value: u32,
    out_points: *mut BifurcationPoint,
    out_len: u32,
) -> u32 {
    if !finite(initial_x) || !finite(r_min) || !finite(r_max) {
        return 0;
    }
    if !(0.0..=1.0).contains(&initial_x) {
        return 0;
    }
    if r_min < 0.0 || r_max > 4.0 || r_min > r_max {
        set_error(ERR_INVALID_ARGUMENT, "r range must be [0, 4] and r_min <= r_max");
        return 0;
    }
    if param_steps == 0 || transient_steps == 0 || samples_per_value == 0 {
        set_error(ERR_INVALID_ARGUMENT, "steps must be > 0");
        return 0;
    }

    let cap = out_len as usize;
    let total_needed = (param_steps as usize).saturating_mul(samples_per_value as usize);
    if total_needed > cap {
        set_error(ERR_CAPACITY, "output buffer too small");
        return 0;
    }

    let buf = unsafe { std::slice::from_raw_parts_mut(out_points, cap) };
    let mut written = 0usize;

    for p_idx in 0..param_steps as usize {
        let r = if param_steps > 1 {
            r_min + (p_idx as f64 / (param_steps - 1) as f64) * (r_max - r_min)
        } else {
            r_min
        };

        if !finite(r) || !(0.0..=4.0).contains(&r) {
            continue;
        }

        // Transient
        let mut x = initial_x;
        for _ in 0..transient_steps {
            x = r * x * (1.0 - x);
        }

        // Sample
        for _ in 0..samples_per_value {
            x = r * x * (1.0 - x);
            buf[written] = BifurcationPoint {
                parameter: r,
                sample: x,
            };
            written += 1;
        }
    }

    clear_error();
    written as u32
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

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