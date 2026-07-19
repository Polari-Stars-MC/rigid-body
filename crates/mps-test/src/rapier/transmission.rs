#[cfg(test)]
mod tests {
    use mps_core::rapier::transmission::*;
    use mps_core::rapier::ffi::*;

    #[test]
    fn gear_constraint_supports_opposite_rotation() {
        let mut report = GearConstraintReport::default();
        assert_eq!(
            transmission_gear_evaluate(
                2.0,
                -4.1,
                3.0,
                -6.0,
                GearConstraintDesc {
                    ratio: 2.0,
                    phase: 0.0,
                    backlash: 0.0,
                    opposite_direction: Bool::TRUE,
                },
                &mut report
            ),
            Bool::TRUE
        );
        assert_eq!(report.target_angle, -4.0);
        assert!((report.angle_error - 0.1).abs() < 1.0e-12);
        assert_eq!(report.target_angular_velocity, -6.0);
    }

    #[test]
    fn screw_constraint_maps_rotation_to_translation() {
        let mut report = ScrewConstraintReport::default();
        assert_eq!(
            transmission_screw_evaluate(
                TAU,
                0.08,
                TAU * 2.0,
                0.2,
                ScrewConstraintDesc {
                    lead: 0.1,
                    phase: 0.0,
                    right_handed: Bool::TRUE,
                },
                &mut report
            ),
            Bool::TRUE
        );
        assert!((report.target_translation - 0.1).abs() < 1.0e-12);
        assert!((report.target_linear_velocity - 0.2).abs() < 1.0e-12);
    }

    #[test]
    fn cycloidal_cam_and_spiral_constraints_work() {
        let mut cam = CamConstraintReport::default();
        assert_eq!(
            transmission_cycloidal_cam_evaluate(
                std::f64::consts::FRAC_PI_2,
                0.0,
                2.0,
                CamConstraintDesc {
                    base_radius: 1.0,
                    lift: 0.5,
                    rise_angle: std::f64::consts::PI,
                    return_angle: std::f64::consts::PI,
                    phase: 0.0,
                },
                &mut cam
            ),
            Bool::TRUE
        );
        assert!(cam.follower_displacement > 0.0);
        assert!(cam.radius > 1.0);

        let mut spiral = SpiralConstraintReport::default();
        assert_eq!(
            transmission_archimedean_spiral_evaluate(
                std::f64::consts::FRAC_PI_2,
                1.0,
                3.0,
                SpiralConstraintDesc {
                    initial_radius: 1.0,
                    radial_pitch: 0.2,
                    phase: 0.0,
                },
                &mut spiral
            ),
            Bool::TRUE
        );
        assert!(spiral.radius > 1.0);
        assert!(spiral.radial_velocity > 0.0);
    }
}



