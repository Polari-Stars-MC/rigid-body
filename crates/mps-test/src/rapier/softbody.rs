#[cfg(test)]
mod tests {
    use mps_core::rapier::softbody::*;
    use mps_core::rapier::ffi::*;

    fn v3(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    #[test]
    fn prediction_spring_and_distance_constraint_work() {
        let positions = [v3(0.0, 0.0, 0.0), v3(2.0, 0.0, 0.0)];
        let velocities = [v3(0.0, 0.0, 0.0), v3(0.0, 0.0, 0.0)];
        let inverse_masses = [1.0, 1.0];
        let mut predicted = [Vec3::default(); 2];
        assert_eq!(
            softbody_predict_positions(
                positions.as_ptr(),
                velocities.as_ptr(),
                inverse_masses.as_ptr(),
                2,
                v3(0.0, -10.0, 0.0),
                0.0,
                0.1,
                predicted.as_mut_ptr(),
                2,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert!(predicted[0].y < 0.0);

        let spring = [SoftSpring {
            particle_a: 0,
            particle_b: 1,
            rest_length: 1.0,
            stiffness: 10.0,
            damping: 0.0,
        }];
        let mut forces = [Vec3::default(); 2];
        assert_eq!(
            softbody_mass_spring_forces(
                positions.as_ptr(),
                velocities.as_ptr(),
                2,
                spring.as_ptr(),
                1,
                forces.as_mut_ptr(),
                2,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert!(forces[0].x > 0.0);

        let mut projected = positions;
        let mut constraints = [SoftDistanceConstraint {
            particle_a: 0,
            particle_b: 1,
            rest_length: 1.0,
            stiffness: 1.0,
            compliance: 0.0,
            lambda: 0.0,
        }];
        assert_eq!(
            softbody_solve_xpbd_distance_constraints(
                projected.as_mut_ptr(),
                inverse_masses.as_ptr(),
                2,
                constraints.as_mut_ptr(),
                1,
                0.1,
                4,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        let distance = (vec3_to_rapier(projected[1]) - vec3_to_rapier(projected[0])).length();
        assert!((distance - 1.0).abs() < 1.0e-8);
    }

    #[test]
    fn collision_volume_and_velocity_update_work() {
        let inverse_masses = [1.0, 1.0, 1.0, 1.0];
        let mut positions = [
            v3(0.0, 0.0, 0.0),
            v3(1.2, 0.0, 0.0),
            v3(0.0, 1.0, 0.0),
            v3(0.0, 0.0, 1.0),
        ];
        let mut volumes = [SoftVolumeConstraint {
            particle_a: 0,
            particle_b: 1,
            particle_c: 2,
            particle_d: 3,
            rest_volume: 1.0 / 6.0,
            compliance: 0.0,
            lambda: 0.0,
        }];
        assert_eq!(
            softbody_solve_xpbd_volume_constraints(
                positions.as_mut_ptr(),
                inverse_masses.as_ptr(),
                4,
                volumes.as_mut_ptr(),
                1,
                0.1,
                8,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        let volume = tetra_volume(
            vec3_to_rapier(positions[0]),
            vec3_to_rapier(positions[1]),
            vec3_to_rapier(positions[2]),
            vec3_to_rapier(positions[3]),
        );
        assert!((volume - 1.0 / 6.0).abs() < 1.0e-4);

        let mut colliding = [v3(0.0, 0.0, 0.0)];
        let sphere = [SoftSphereCollision {
            center: v3(0.0, 0.0, 0.0),
            radius: 2.0,
        }];
        assert_eq!(
            softbody_solve_sphere_collision_constraints(
                colliding.as_mut_ptr(),
                inverse_masses.as_ptr(),
                1,
                sphere.as_ptr(),
                1,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert!((vec3_to_rapier(colliding[0]).length() - 2.0).abs() < 1.0e-8);

        let mut velocities = [Vec3::default()];
        let previous = [v3(0.0, 0.0, 0.0)];
        assert_eq!(
            softbody_update_velocities(
                previous.as_ptr(),
                colliding.as_ptr(),
                1,
                0.5,
                velocities.as_mut_ptr(),
                1,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert!(vec3_to_rapier(velocities[0]).length() > 0.0);
    }
}



