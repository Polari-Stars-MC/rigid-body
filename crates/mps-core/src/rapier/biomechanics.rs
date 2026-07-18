use crate::rapier::error::{ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error};
use crate::rapier::ffi::{
    Bool, HillMuscleDesc, HillMuscleReport, HillMuscleState, SkeletalConstraintReport,
    SkeletalJointLimit,
};

use crate::rapier::math::{finite_non_negative, finite_positive};

const EPSILON: f64 = 1.0e-12;

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

#[unsafe(no_mangle)]
pub extern "C" fn biomechanics_hill_force_length_factor(
    fiber_length: f64,
    optimal_fiber_length: f64,
    width: f64,
) -> f64 {
    if !finite_positive(fiber_length)
        || !finite_positive(optimal_fiber_length)
        || !finite_positive(width)
    {
        return f64::NAN;
    }
    let normalized = fiber_length / optimal_fiber_length;
    let x = (normalized - 1.0) / width;
    (-x * x).exp()
}

#[unsafe(no_mangle)]
pub extern "C" fn biomechanics_hill_force_velocity_factor(
    fiber_velocity: f64,
    max_contraction_velocity: f64,
) -> f64 {
    if !fiber_velocity.is_finite() || !finite_positive(max_contraction_velocity) {
        return f64::NAN;
    }
    let normalized = fiber_velocity / max_contraction_velocity;
    if normalized < 0.0 {
        ((1.0 + normalized).max(0.0) / (1.0 - normalized / 1.5)).clamp(0.0, 1.5)
    } else {
        (1.0 + 0.3 * normalized).clamp(1.0, 1.5)
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn biomechanics_hill_muscle_evaluate(
    desc: HillMuscleDesc,
    state: HillMuscleState,
    out_report: *mut HillMuscleReport,
) -> Bool {
    if !muscle_desc_valid(desc) || !muscle_state_valid(state) {
        set_error(ERR_INVALID_ARGUMENT, "invalid Hill muscle parameters");
        return Bool::FALSE;
    }
    let force_length =
        biomechanics_hill_force_length_factor(state.fiber_length, desc.optimal_fiber_length, 0.45);
    let force_velocity = biomechanics_hill_force_velocity_factor(
        state.fiber_velocity,
        desc.max_contraction_velocity,
    );
    let pennation = desc.pennation_angle.cos().max(EPSILON);
    let active_force = state.activation * desc.max_isometric_force * force_length * force_velocity;
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
}

#[unsafe(no_mangle)]
pub extern "C" fn biomechanics_hill_three_element_force(
    activation: f64,
    fiber_length: f64,
    fiber_velocity: f64,
    tendon_length: f64,
    desc: HillMuscleDesc,
) -> f64 {
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
}

#[unsafe(no_mangle)]
pub extern "C" fn biomechanics_skeletal_joint_limit(
    angle: f64,
    angular_velocity: f64,
    limit: SkeletalJointLimit,
    out_report: *mut SkeletalConstraintReport,
) -> Bool {
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
}

#[unsafe(no_mangle)]
pub extern "C" fn biomechanics_muscle_joint_torque(muscle_force: f64, moment_arm: f64) -> f64 {
    if !muscle_force.is_finite() || !moment_arm.is_finite() {
        return f64::NAN;
    }
    muscle_force * moment_arm
}

#[cfg(test)]
mod tests {
    use super::*;

    fn muscle() -> HillMuscleDesc {
        HillMuscleDesc {
            max_isometric_force: 1500.0,
            optimal_fiber_length: 0.1,
            tendon_slack_length: 0.2,
            max_contraction_velocity: 1.2,
            parallel_stiffness: 20_000.0,
            series_stiffness: 80_000.0,
            damping: 20.0,
            pennation_angle: 0.1,
        }
    }

    #[test]
    fn hill_three_element_model_reports_force_components() {
        let mut report = HillMuscleReport::default();
        assert_eq!(
            biomechanics_hill_muscle_evaluate(
                muscle(),
                HillMuscleState {
                    activation: 0.8,
                    fiber_length: 0.1,
                    fiber_velocity: 0.0,
                    tendon_length: 0.23,
                    moment_arm: 0.04,
                },
                &mut report
            ),
            Bool::TRUE
        );
        assert!(report.active_force > 0.0);
        assert!(report.series_elastic_force > 0.0);
        assert!(report.tendon_force > 0.0);
        assert!(report.joint_torque > 0.0);
    }

    #[test]
    fn force_length_and_velocity_factors_are_finite() {
        let fl = biomechanics_hill_force_length_factor(0.1, 0.1, 0.45);
        let fv = biomechanics_hill_force_velocity_factor(-0.2, 1.2);
        assert!((fl - 1.0).abs() < 1.0e-12);
        assert!(fv > 0.0);
        assert!(fv <= 1.5);
    }

    #[test]
    fn skeletal_joint_limit_generates_corrective_torque() {
        let mut report = SkeletalConstraintReport::default();
        assert_eq!(
            biomechanics_skeletal_joint_limit(
                2.0,
                0.5,
                SkeletalJointLimit {
                    min_angle: -1.0,
                    max_angle: 1.0,
                    stiffness: 100.0,
                    damping: 10.0,
                },
                &mut report
            ),
            Bool::TRUE
        );
        assert_eq!(report.limited, Bool::TRUE);
        assert_eq!(report.clamped_angle, 1.0);
        assert!(report.corrective_torque < 0.0);
    }
}
