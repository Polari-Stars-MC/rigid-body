//! `flight::stability` — power-iteration eigenvalue + linearization tests.

#[cfg(test)]
mod tests {
    use mps_cosmos::flight::{
        ConstantGravity, FlightControls, RigidBodyState, SeaLevelAtmosphere, linearize,
        longitudinal_modes, longitudinal_submatrix, power_iteration,
    };
    use mps_formula::rotor::RotorParams;
    use rapier3d::prelude::{Rotation, Vector};
    use std::f64::consts::PI;

    fn main_rotor() -> RotorParams {
        RotorParams {
            radius: 5.4,
            n_blades: 2,
            chord: 0.20,
            hinge_offset: 0.05,
            lift_slope: 2.0 * PI,
            zero_lift_alpha: 0.0,
            profile_cd0: 0.012,
            cd_k: 0.05,
        }
    }
    fn tail_rotor() -> RotorParams {
        RotorParams {
            radius: 1.0,
            n_blades: 2,
            chord: 0.08,
            hinge_offset: 0.02,
            lift_slope: 2.0 * PI,
            zero_lift_alpha: 0.0,
            profile_cd0: 0.015,
            cd_k: 0.05,
        }
    }
    fn state() -> RigidBodyState {
        RigidBodyState {
            position: Vector::new(0.0, 0.0, 100.0),
            linvel_world: Vector::ZERO,
            angvel_body: Vector::ZERO,
            rotation: Rotation::IDENTITY,
            mass: 800.0,
        }
    }
    fn controls() -> FlightControls {
        FlightControls {
            collective: 0.10,
            cyclic_lon: 0.0,
            cyclic_lat: 0.0,
            tail_collective: 0.0,
            throttle: 1.0,
        }
    }

    #[test]
    fn linearize_returns_some_on_valid_inputs() {
        let r = main_rotor();
        let t = tail_rotor();
        let a = SeaLevelAtmosphere;
        let g = ConstantGravity { g: 9.81 };
        let derivs = linearize(&state(), &controls(), &r, &t, &a, &g, 40.0, 0.0, 1.0e-3, 60);
        assert!(derivs.is_some(), "linearize should succeed");
        let d = derivs.unwrap();
        // A is 6×6 = 36 entries.
        assert_eq!(d.a.len(), 36);
        assert_eq!(d.b.len(), 30);
        // nonlinearity is finite.
        assert!(d.nonlinearity.is_finite());
    }

    #[test]
    fn linearize_rejects_bad_inputs() {
        let r = main_rotor();
        let t = tail_rotor();
        let a = SeaLevelAtmosphere;
        let g = ConstantGravity { g: 9.81 };
        // Zero mass
        let mut bad = state();
        bad.mass = 0.0;
        assert!(linearize(&bad, &controls(), &r, &t, &a, &g, 40.0, 0.0, 1.0e-3, 60).is_none());
        // h <= 0
        assert!(linearize(&state(), &controls(), &r, &t, &a, &g, 40.0, 0.0, 0.0, 60).is_none());
        // Bad rotor
        let mut bad_rotor = r;
        bad_rotor.radius = 0.0;
        assert!(
            linearize(
                &state(),
                &controls(),
                &bad_rotor,
                &t,
                &a,
                &g,
                40.0,
                0.0,
                1.0e-3,
                60
            )
            .is_none()
        );
    }

    #[test]
    fn power_iteration_converges_on_diagonal_identity() {
        let id = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let r = power_iteration(&id);
        assert!(r.converged, "did not converge; iter={}", r.iterations);
        assert!(
            (r.dominant_eigenvalue - 1.0).abs() < 1.0e-6,
            "dominant={}",
            r.dominant_eigenvalue
        );
    }

    #[test]
    fn power_iteration_converges_on_diagonal_dominant() {
        let m = [
            2.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.05,
        ];
        let r = power_iteration(&m);
        assert!(r.converged);
        assert!(
            (r.dominant_eigenvalue - 2.0).abs() < 1.0e-6,
            "dominant={}",
            r.dominant_eigenvalue
        );
        // Eigenvector should be approximately [1, 0, 0, 0].
        assert!(r.dominant_eigenvector[0].abs() > 0.95);
        assert!(r.dominant_eigenvector[1].abs() < 0.1);
        assert!(r.dominant_eigenvector[2].abs() < 0.1);
        assert!(r.dominant_eigenvector[3].abs() < 0.1);
    }

    #[test]
    fn longitudinal_modes_extracts_two_eigenpairs() {
        let m = [
            2.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.05,
        ];
        let modes = longitudinal_modes(&m);
        assert_eq!(modes.len(), 2);
        // first mode dominant, second mode deflated.
        assert!(modes[0].converged);
        assert!((modes[0].dominant_eigenvalue - 2.0).abs() < 1.0e-4);
    }

    #[test]
    fn longitudinal_submatrix_picks_correct_indices() {
        // Construct a synthetic 6×6 A with marker values at A[0,2,3,5]
        // rows×cols = entries (0,2),(2,2),(3,2),(5,2), etc. — confirm
        // mapping is identity.
        let mut a = [0.0_f64; 36];
        let rows = [0, 2, 3, 5];
        let cols = [0, 2, 3, 5];
        for (i, &ri) in rows.iter().enumerate() {
            for (j, &ci) in cols.iter().enumerate() {
                // Encode (i, j) as i · 10 + j + 1 at (ri, ci)
                a[ri * 6 + ci] = (i as f64) * 10.0 + (j as f64) + 1.0;
            }
        }
        let out = longitudinal_submatrix(&a);
        // out[i*4 + j] should equal i*10 + j + 1.
        for i in 0..4 {
            for j in 0..4 {
                let expected = (i as f64) * 10.0 + (j as f64) + 1.0;
                assert!(
                    (out[i * 4 + j] - expected).abs() < 1.0e-12,
                    "out[{}, {}] = {}",
                    i,
                    j,
                    out[i * 4 + j]
                );
            }
        }
    }
}
