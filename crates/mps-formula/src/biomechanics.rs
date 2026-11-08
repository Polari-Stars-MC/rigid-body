use crate::error::{ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, ffi_guard, set_error};
use crate::ffi::{
    Bool, HillMuscleDesc, HillMuscleReport, HillMuscleState, SkeletalConstraintReport,
    SkeletalJointLimit,
};

use crate::math::{EPS_GENERAL as EPSILON, finite_non_negative, finite_positive};

fn muscle_desc_valid(desc: HillMuscleDesc) -> bool {
    finite_positive(desc.max_isometric_force)
        && finite_positive(desc.optimal_fiber_length)
        && finite_non_negative(desc.tendon_slack_length)
        && finite_positive(desc.max_contraction_velocity)
        && finite_non_negative(desc.parallel_stiffness)
        && finite_non_negative(desc.series_stiffness)
        && finite_non_negative(desc.damping)
        && desc.pennation_angle.is_finite()
        && desc.pennation_angle.abs() < std::f64::consts::FRAC_PI_2
}

fn muscle_state_valid(state: HillMuscleState) -> bool {
    state.activation.is_finite()
        && (0.0..=1.0).contains(&state.activation)
        && finite_positive(state.fiber_length)
        && state.fiber_velocity.is_finite()
        && finite_non_negative(state.tendon_length)
        && state.moment_arm.is_finite()
}

fn joint_limit_valid(limit: SkeletalJointLimit) -> bool {
    limit.min_angle.is_finite()
        && limit.max_angle.is_finite()
        && limit.min_angle <= limit.max_angle
        && finite_non_negative(limit.stiffness)
        && finite_non_negative(limit.damping)
}

/// Computes the Hill muscle force-length factor (Gaussian around the optimal fiber length).
///
/// # Safety
///
/// Takes only scalar values; no pointers are dereferenced. `fiber_length`,
/// `optimal_fiber_length`, and `width` must be finite and positive; invalid
/// inputs return `f64::NAN` instead of an error code.
#[unsafe(no_mangle)]
pub extern "C" fn biomechanics_hill_force_length_factor(
    fiber_length: f64,
    optimal_fiber_length: f64,
    width: f64,
) -> f64 {
    ffi_guard(f64::NAN, || {
        if !finite_positive(fiber_length)
            || !finite_positive(optimal_fiber_length)
            || !finite_positive(width)
        {
            return f64::NAN;
        }
        let normalized = fiber_length / optimal_fiber_length;
        let x = (normalized - 1.0) / width;
        (-x * x).exp()
    })
}

/// Computes the Hill muscle force-velocity factor for a given fiber velocity.
///
/// # Safety
///
/// Takes only scalar values; no pointers are dereferenced. `fiber_velocity`
/// must be finite and `max_contraction_velocity` finite and positive; invalid
/// inputs return `f64::NAN` instead of an error code.
#[unsafe(no_mangle)]
pub extern "C" fn biomechanics_hill_force_velocity_factor(
    fiber_velocity: f64,
    max_contraction_velocity: f64,
) -> f64 {
    ffi_guard(f64::NAN, || {
        if !fiber_velocity.is_finite() || !finite_positive(max_contraction_velocity) {
            return f64::NAN;
        }
        let normalized = fiber_velocity / max_contraction_velocity;
        if normalized < 0.0 {
            ((1.0 + normalized).max(0.0) / (1.0 - normalized / 1.5)).clamp(0.0, 1.5)
        } else {
            (1.0 + 0.3 * normalized).clamp(1.0, 1.5)
        }
    })
}

/// Evaluates a Hill-type muscle model and writes the force breakdown to `out_report`.
///
/// # Safety
///
/// `out_report` must be null or point to writable memory for one
/// `HillMuscleReport`; a null pointer fails with `ERR_NULL_POINTER`. `desc`
/// and `state` are passed by value (no ownership transfer) and must satisfy
/// the finite/positive/range checks in `muscle_desc_valid` /
/// `muscle_state_valid`; invalid values fail with `ERR_INVALID_ARGUMENT`.
#[unsafe(no_mangle)]
pub extern "C" fn biomechanics_hill_muscle_evaluate(
    desc: HillMuscleDesc,
    state: HillMuscleState,
    out_report: *mut HillMuscleReport,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        if !muscle_desc_valid(desc) || !muscle_state_valid(state) {
            set_error(ERR_INVALID_ARGUMENT, "invalid Hill muscle parameters");
            return Bool::FALSE;
        }
        let force_length = biomechanics_hill_force_length_factor(
            state.fiber_length,
            desc.optimal_fiber_length,
            0.45,
        );
        let force_velocity = biomechanics_hill_force_velocity_factor(
            state.fiber_velocity,
            desc.max_contraction_velocity,
        );
        let pennation = desc.pennation_angle.cos().max(EPSILON);
        let active_force =
            state.activation * desc.max_isometric_force * force_length * force_velocity;
        let stretch = (state.fiber_length - desc.optimal_fiber_length).max(0.0);
        let parallel_elastic_force = desc.parallel_stiffness * stretch * stretch;
        let damping_force = -desc.damping * state.fiber_velocity;
        let total_fiber_force = (active_force + parallel_elastic_force + damping_force).max(0.0);
        let tendon_stretch = (state.tendon_length - desc.tendon_slack_length).max(0.0);
        let series_elastic_force = desc.series_stiffness * tendon_stretch;
        let tendon_force = f64::min(total_fiber_force * pennation, series_elastic_force);
        let Some(out_report) = (unsafe { out_report.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "Hill muscle output is null");
            return Bool::FALSE;
        };
        *out_report = HillMuscleReport {
            active_force,
            parallel_elastic_force,
            series_elastic_force,
            damping_force,
            total_fiber_force,
            tendon_force,
            joint_torque: tendon_force * state.moment_arm,
            force_length_factor: force_length,
            force_velocity_factor: force_velocity,
        };
        clear_error();
        Bool::TRUE
    })
}

/// Convenience wrapper returning the tendon force of the three-element Hill muscle model.
///
/// # Safety
///
/// Takes only scalar values and a by-value `HillMuscleDesc` (no pointers are
/// dereferenced, no ownership transfer). Inputs must pass the same validation
/// as `biomechanics_hill_muscle_evaluate`; invalid inputs return `f64::NAN`.
#[unsafe(no_mangle)]
pub extern "C" fn biomechanics_hill_three_element_force(
    activation: f64,
    fiber_length: f64,
    fiber_velocity: f64,
    tendon_length: f64,
    desc: HillMuscleDesc,
) -> f64 {
    ffi_guard(f64::NAN, || {
        let mut report = HillMuscleReport::default();
        let state = HillMuscleState {
            activation,
            fiber_length,
            fiber_velocity,
            tendon_length,
            moment_arm: 0.0,
        };
        if biomechanics_hill_muscle_evaluate(desc, state, &mut report) == Bool::TRUE {
            report.tendon_force
        } else {
            f64::NAN
        }
    })
}

/// Applies a skeletal joint limit, clamping `angle` and computing a corrective torque.
///
/// # Safety
///
/// `out_report` must be null or point to writable memory for one
/// `SkeletalConstraintReport`; a null pointer fails with `ERR_NULL_POINTER`.
/// `angle` and `angular_velocity` must be finite and `limit` (passed by value,
/// no ownership transfer) must pass the checks in `joint_limit_valid`; invalid
/// values fail with `ERR_INVALID_ARGUMENT`.
#[unsafe(no_mangle)]
pub extern "C" fn biomechanics_skeletal_joint_limit(
    angle: f64,
    angular_velocity: f64,
    limit: SkeletalJointLimit,
    out_report: *mut SkeletalConstraintReport,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        if !angle.is_finite() || !angular_velocity.is_finite() || !joint_limit_valid(limit) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "invalid skeletal joint limit parameters",
            );
            return Bool::FALSE;
        }
        let clamped_angle = angle.clamp(limit.min_angle, limit.max_angle);
        let angle_error = clamped_angle - angle;
        let limited = angle_error.abs() > EPSILON;
        let corrective_torque = if limited {
            limit.stiffness * angle_error - limit.damping * angular_velocity
        } else {
            0.0
        };
        let Some(out_report) = (unsafe { out_report.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "skeletal constraint output is null");
            return Bool::FALSE;
        };
        *out_report = SkeletalConstraintReport {
            clamped_angle,
            angle_error,
            corrective_torque,
            limited: Bool::from(limited),
        };
        clear_error();
        Bool::TRUE
    })
}

/// Computes the joint torque produced by a muscle force acting at a moment arm.
///
/// # Safety
///
/// Takes only scalar values; no pointers are dereferenced. `muscle_force` and
/// `moment_arm` must be finite; invalid inputs return `f64::NAN` instead of an
/// error code.
#[unsafe(no_mangle)]
pub extern "C" fn biomechanics_muscle_joint_torque(muscle_force: f64, moment_arm: f64) -> f64 {
    ffi_guard(f64::NAN, || {
        if !muscle_force.is_finite() || !moment_arm.is_finite() {
            return f64::NAN;
        }
        muscle_force * moment_arm
    })
}
