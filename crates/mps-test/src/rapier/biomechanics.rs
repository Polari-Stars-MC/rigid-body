#[cfg(test)]
mod tests {
    use mps_core::rapier::biomechanics::*;
    use mps_core::rapier::ffi::*;

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



