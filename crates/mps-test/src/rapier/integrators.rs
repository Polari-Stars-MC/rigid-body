#[cfg(test)]
mod tests {
    use mps_core::rapier::integrators::*;
    use mps_core::rapier::ffi::*;

    /// Constant acceleration (uniform field) for testing
    fn const_accel(_pos: Vec3) -> Vec3 { Vec3 { x: 0.0, y: -9.81, z: 0.0 } }
    fn kepler_accel(pos: Vec3) -> Vec3 {
        let r2 = pos.x * pos.x + pos.y * pos.y + pos.z * pos.z;
        let r3 = r2 * r2.sqrt();
        let gm = 3.986004415e14; // Earth GM
        Vec3 {
            x: -gm * pos.x / r3,
            y: -gm * pos.y / r3,
            z: -gm * pos.z / r3,
        }
    }

    #[test]
    fn leapfrog_conserves_energy_better_than_euler() {
        let dt = 60.0; // 1 minute steps
        let steps = 100;

        // Initial LEO orbit
        let mut pos_euler = Vec3 { x: 6.778e6, y: 0.0, z: 0.0 };
        let mut vel_euler = Vec3 { x: 0.0, y: 7.67e3, z: 0.0 };

        let mut pos_lf = pos_euler;
        let mut vel_lf = vel_euler;

        // Euler (semi-implicit)
        for _ in 0..steps {
            let a = kepler_accel(pos_euler);
            vel_euler.x += a.x * dt;
            vel_euler.y += a.y * dt;
            vel_euler.z += a.z * dt;
            pos_euler.x += vel_euler.x * dt;
            pos_euler.y += vel_euler.y * dt;
            pos_euler.z += vel_euler.z * dt;
        }

        // Leapfrog
        for _ in 0..steps {
            leapfrog_step(&mut pos_lf, &mut vel_lf, dt, kepler_accel);
        }

        let e0 = specific_energy(
            Vec3 { x: 6.778e6, y: 0.0, z: 0.0 },
            Vec3 { x: 0.0, y: 7.67e3, z: 0.0 },
            3.986004415e14,
        );
        let e_euler = specific_energy(pos_euler, vel_euler, 3.986004415e14);
        let e_lf = specific_energy(pos_lf, vel_lf, 3.986004415e14);

        let drift_euler = (e_euler - e0).abs() / e0.abs();
        let drift_lf = (e_lf - e0).abs() / e0.abs();

        // Leapfrog should have significantly less energy drift
        assert!(drift_lf < drift_euler * 0.5,
            "Leapfrog drift {:.2e} should be < 50% of Euler drift {:.2e}",
            drift_lf, drift_euler);
    }

    #[test]
    fn yoshida4_conserves_energy_better_than_leapfrog() {
        let dt = 300.0; // 5 minute steps
        let steps = 1000;

        let start = Vec3 { x: 6.778e6, y: 0.0, z: 0.0 };
        let v_start = Vec3 { x: 0.0, y: 7.67e3, z: 0.0 };

        let mut pos_lf = start;
        let mut vel_lf = v_start;
        let mut pos_y4 = start;
        let mut vel_y4 = v_start;

        let e0 = specific_energy(start, v_start, 3.986004415e14);

        for _ in 0..steps {
            leapfrog_step(&mut pos_lf, &mut vel_lf, dt, kepler_accel);
            yoshida4_step(&mut pos_y4, &mut vel_y4, dt, kepler_accel);
        }

        let e_lf = specific_energy(pos_lf, vel_lf, 3.986004415e14);
        let e_y4 = specific_energy(pos_y4, vel_y4, 3.986004415e14);

        let drift_lf = (e_lf - e0).abs() / e0.abs();
        let drift_y4 = (e_y4 - e0).abs() / e0.abs();

        assert!(drift_y4 < drift_lf * 0.1,
            "Yoshida4 drift {:.2e} should be < 10% of Leapfrog drift {:.2e}",
            drift_y4, drift_lf);
    }

    #[test]
    fn keplerian_elements_circular_orbit() {
        let pos = Vec3 { x: 7.0e6, y: 0.0, z: 0.0 };
        let gm = 3.986004415e14;
        let v_circ = (gm / pos.x).sqrt();
        let vel = Vec3 { x: 0.0, y: v_circ, z: 0.0 };

        let (a, e, i, _, _, _) = keplerian_elements(pos, vel, gm);

        assert!((a - pos.x).abs() / pos.x < 1e-10,
            "Semi-major axis should equal radius for circular orbit");
        assert!(e < 1e-12, "Eccentricity should be ~0 for circular orbit");
        assert!(i.abs() < 1e-12, "Inclination should be 0 for equatorial orbit");
    }

    #[test]
    fn post_newtonian_is_small_correction() {
        let pos = Vec3 { x: 7.0e6, y: 0.0, z: 0.0 };
        let gm = 3.986004415e14;
        let v_circ = (gm / pos.x).sqrt();
        let vel = Vec3 { x: 0.0, y: v_circ, z: 0.0 };

        let a_pn = post_newtonian_full(pos, vel, gm);
        let r2 = pos.x * pos.x;
        let a_newton = gm / r2;

        let pn_mag = (a_pn.x * a_pn.x + a_pn.y * a_pn.y + a_pn.z * a_pn.z).sqrt();
        let ratio = pn_mag / a_newton;

        // Post-Newtonian correction at LEO should be nonzero but very small
        assert!(pn_mag > 0.0, "PN correction should be nonzero");
        assert!(ratio < 1e-4, "PN ratio {:.2e} should be < 1e-4 at LEO", ratio);
    }

    #[test]
    fn adaptive_step_size_works() {
        let dt = 1.0;
        let dt_small = adaptive_step_size(dt, 1e-6, 1e-8, 4);
        let dt_large = adaptive_step_size(dt, 1e-10, 1e-8, 4);

        // Large error → smaller step
        assert!(dt_small < dt);
        // Small error → larger step
        assert!(dt_large > dt);
    }
}



