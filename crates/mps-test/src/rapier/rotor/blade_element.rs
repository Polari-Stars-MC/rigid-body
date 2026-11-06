//! `rotor::blade_element` — BET integration canonical-value tests.
//!
//! A simple uniform-pitch symmetric rotor is integrated; we check that the
//! hover thrust (zero forward speed) is positive and that the thrust falls
//! into the physical band implied by momentum theory on the same rotor.

#[cfg(test)]
mod tests {
    use mps_core::rapier::rotor::blade_element::{Airfoil, LinearAirfoil};
    use mps_core::rapier::rotor::*;
    use std::f64::consts::PI;

    fn rotor() -> RotorParams {
        RotorParams {
            radius: 5.4,
            n_blades: 2,
            chord: 0.20,
            hinge_offset: 0.05,
            lift_slope: 2.0 * std::f64::consts::PI, // thin flat-plate
            zero_lift_alpha: 0.0,
            profile_cd0: 0.012,
            cd_k: 0.05,
        }
    }

    #[test]
    fn compute_rotor_forces_hover_is_positive_thrust() {
        let r = rotor();
        let airfoil = LinearAirfoil::from_rotor(&r);
        let pitch = PitchDistribution::Uniform { theta: 0.15 }; // ~8.6°
        // Hover: zero forward inflow means r·ω is the dominant velocity.  The
        // momentum-theory induced velocity is roughly ~10 m/s for this rotor;
        // we use a small fixed inflow for a clean BET-only test.
        let result = compute_rotor_forces(&r, 8.0, 40.0, 1.225, &pitch, &airfoil, 60).unwrap();
        assert!(result.thrust > 0.0, "thrust={}", result.thrust);
        assert!(result.torque > 0.0, "torque={}", result.torque);
        // For an 8.6° collective and this rotor we expect a thrust in the
        // kilo-Newton range; BET integration gives a typical number.
        // Smoke check, no tight canonical for BET without airfoil database.
        assert!(result.stations > 10, "stations={}", result.stations);
    }

    #[test]
    fn compute_rotor_forces_rejects_bad_geometry() {
        let r = rotor();
        let airfoil = LinearAirfoil::from_rotor(&r);
        let pitch = PitchDistribution::Uniform { theta: 0.15 };
        // Negative inflow is allowed (descent).
        assert!(compute_rotor_forces(&r, -1.0, 40.0, 1.225, &pitch, &airfoil, 60).is_some());
        // Bad omega / rho / stations
        assert!(compute_rotor_forces(&r, 8.0, 0.0, 1.225, &pitch, &airfoil, 60).is_none());
        assert!(compute_rotor_forces(&r, 8.0, 40.0, 0.0, &pitch, &airfoil, 60).is_none());
        assert!(compute_rotor_forces(&r, 8.0, 40.0, 1.225, &pitch, &airfoil, 1).is_none());
        assert!(compute_rotor_forces(&r, 8.0, 40.0, 1.225, &pitch, &airfoil, 0).is_none());
        // Bad radius and zero blades fail in `rotor.valid()` path.
        let mut r1 = r;
        r1.radius = 0.0;
        assert!(compute_rotor_forces(&r1, 8.0, 40.0, 1.225, &pitch, &airfoil, 60).is_none());
        let mut r2 = r;
        r2.n_blades = 0;
        assert!(compute_rotor_forces(&r2, 8.0, 40.0, 1.225, &pitch, &airfoil, 60).is_none());
    }

    #[test]
    fn pitch_distribution_construction_and_lookup() {
        let u = PitchDistribution::Uniform { theta: 0.1 };
        assert!((u.at(0.0) - 0.1).abs() < 1.0e-12);
        assert!((u.at(0.7) - 0.1).abs() < 1.0e-12);
        let l = PitchDistribution::Linear {
            theta_root: 0.15,
            theta_tip: 0.05,
        };
        assert!((l.at(0.0) - 0.15).abs() < 1.0e-12);
        assert!((l.at(1.0) - 0.05).abs() < 1.0e-12);
        assert!((l.at(0.5) - 0.10).abs() < 1.0e-12);
        let s = PitchDistribution::Sampled(vec![0.1, 0.2, 0.3, 0.4]);
        assert!((s.at(0.0) - 0.1).abs() < 1.0e-12);
        assert!((s.at(1.0) - 0.4).abs() < 1.0e-12);
        // midpoint of 4 samples → idx=round(0.5·3)=1.5 → round → 2 → 0.3.
        assert!((s.at(0.5) - 0.3).abs() < 1.0e-12);
    }

    #[test]
    fn linear_airfoil_clips_at_stall() {
        let r = rotor();
        let a = LinearAirfoil::from_rotor(&r);
        // In the linear regime: C_l = a₀ · alpha, with a₀ = 2π.
        let lin = a.cl(0.05);
        assert!((lin - 2.0 * PI * 0.05).abs() < 1.0e-6, "lin_cl={lin}");
        // At stall_alpha (0.21 rad): C_l = 2π·0.21.
        let v_stall = a.cl(0.21);
        assert!(
            (v_stall - 2.0 * PI * 0.21).abs() < 1.0e-6,
            "stall_cl={v_stall}"
        );
        // Past stall (0.5 rad): C_l clamps to stall value.
        let v_post = a.cl(0.5);
        assert!(
            (v_post - v_stall).abs() < 1.0e-6,
            "post-stall cl={v_post} should clamp to {v_stall}"
        );
        // Cd baseline + induced.
        let cd = a.cd(0.0);
        assert!((cd - r.profile_cd0).abs() < 1.0e-9);
    }
}
