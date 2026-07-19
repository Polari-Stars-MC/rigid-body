#[cfg(test)]
mod tests {
    use mps_core::rapier::trajectory::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::Vec3;

    fn env() -> TrajectoryEnvironment {
        TrajectoryEnvironment {
            gravity: Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
            flow_velocity: Vec3::default(),
            mass: 2.0,
            reference_area: 0.1,
            density: 1.225,
            drag_coefficient: 0.5,
            lift_coefficient: 0.0,
            lift_direction: Vec3::default(),
        }
    }

    #[test]
    fn estimates_gravity_and_drag() {
        let mut report = TrajectoryForceReport::default();
        assert_eq!(
            trajectory_estimate_forces(
                TrajectoryState {
                    position: Vec3::default(),
                    velocity: Vec3 {
                        x: 10.0,
                        y: 0.0,
                        z: 0.0,
                    },
                },
                env(),
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(report.gravity_force.y < 0.0);
        assert!(report.drag_force.x < 0.0);
    }

    #[test]
    fn integrates_state_forward() {
        let mut out = TrajectoryState::default();
        assert_eq!(
            trajectory_integrate_step(
                TrajectoryState {
                    position: Vec3::default(),
                    velocity: Vec3 {
                        x: 10.0,
                        y: 0.0,
                        z: 0.0,
                    },
                },
                env(),
                0.1,
                &mut out,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert!(out.position.x > 0.0);
        assert!(out.velocity.y < 0.0);
    }

    fn glide_env() -> TrajectoryGlideEnvironment {
        TrajectoryGlideEnvironment {
            gravity: 9.80665,
            planet_radius: 6_371_000.0,
            ballistic_coefficient: 2_000.0,
            lift_to_drag: 1.5,
            bank_angle: 0.0,
            reference_density: 1.225,
            scale_height: 7_200.0,
        }
    }

    #[test]
    fn estimates_glide_derivatives() {
        let mut report = TrajectoryGlideReport::default();
        assert_eq!(
            trajectory_glide_estimate(
                TrajectoryGlideState {
                    speed: 3_000.0,
                    flight_path_angle: -0.05,
                    altitude: 40_000.0,
                    downrange: 0.0,
                },
                glide_env(),
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(report.density > 0.0);
        assert!(report.drag_acceleration > 0.0);
        assert!(report.altitude_dot < 0.0);
        assert!(report.downrange_dot > 0.0);
    }

    #[test]
    fn integrates_glide_state_forward() {
        let mut out = TrajectoryGlideState::default();
        assert_eq!(
            trajectory_glide_integrate_step(
                TrajectoryGlideState {
                    speed: 3_000.0,
                    flight_path_angle: -0.05,
                    altitude: 40_000.0,
                    downrange: 0.0,
                },
                glide_env(),
                0.5,
                &mut out,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert!(out.speed < 3_000.0);
        assert!(out.altitude < 40_000.0);
        assert!(out.downrange > 0.0);
    }
}



