//! `rotor::momentum` — momentum-theory canonical-value + invalid-input tests.
//!
//! Mirror tests for [`mps_formula::rotor::momentum`] reachable via
//! `mps_core::rapier::rotor::*`.
//!
//! Density 1.225 kg/m³ (ISA sea level), rotor radius 5.4 m (Robinson R44-like),
//! thrust 10 kN.  We hover-check the closed form `v_h = √(T/(2ρA))`.

#[cfg(test)]
mod tests {
    use mps_core::rapier::rotor::*;

    const RHO: f64 = 1.225;
    const R: f64 = 5.4;
    const T: f64 = 10_000.0; // ~10 kN, ~1000 kg-f class rotorcraft

    #[test]
    fn rotor_hover_induced_velocity_is_textbook_value() {
        // v_h = √(T / (2ρA));  A = π(5.4)² = 91.6 m².
        let area = std::f64::consts::PI * R * R;
        let expected = (T / (2.0 * RHO * area)).sqrt();
        let v = rotor_hover_induced_velocity(T, RHO, R).expect("hover induced vel");
        assert!(
            (v - expected).abs() < 1.0e-6,
            "v_h={v}, expected={expected}"
        );
        // Physical sanity: hover induced velocity for a 10 kN / 5.4 m rotor is
        // in the 5-10 m/s range.
        assert!((v - 6.5).abs() < 1.5, "v_h={v} not in expected band");
    }

    #[test]
    fn rotor_hover_induced_velocity_zero_thrust_is_zero() {
        assert!(rotor_hover_induced_velocity(0.0, RHO, R).unwrap().abs() < 1.0e-12);
    }

    #[test]
    fn rotor_hover_induced_velocity_rejects_bad_inputs() {
        assert!(rotor_hover_induced_velocity(-1.0, RHO, R).is_none());
        assert!(rotor_hover_induced_velocity(T, -1.0, R).is_none());
        assert!(rotor_hover_induced_velocity(T, RHO, -1.0).is_none());
        assert!(rotor_hover_induced_velocity(f64::NAN, RHO, R).is_none());
        assert!(rotor_hover_induced_velocity(T, f64::NAN, R).is_none());
    }

    #[test]
    fn rotor_hover_power_is_thrust_times_induced() {
        let v_h = rotor_hover_induced_velocity(T, RHO, R).unwrap();
        let p = rotor_hover_power(T, v_h).unwrap();
        assert!((p - T * v_h).abs() < 1.0e-6);
        // ~ideal hover power for 10 kN at v_h ~ 7 m/s = 70 kW.  Physical.
        assert!(
            p > 50_000.0 && p < 100_000.0,
            "P_i={p} W out of expected band"
        );
    }

    #[test]
    fn rotor_figure_of_merit_is_ideal_to_actual() {
        // FM = ideal / actual; ideal == actual → FM = 1.
        let fm = rotor_figure_of_merit(1000.0_f64, 1000.0).unwrap();
        assert!((fm - 1.0).abs() < 1.0e-12);
        // 80% efficient rotor
        let fm2 = rotor_figure_of_merit(800.0_f64, 1000.0).unwrap();
        assert!((fm2 - 0.8).abs() < 1.0e-12);
        // Invalid
        assert!(rotor_figure_of_merit(-1.0, 1000.0).is_none());
        assert!(rotor_figure_of_merit(500.0, 0.0).is_none());
        assert!(rotor_figure_of_merit(500.0, -1.0).is_none());
    }

    #[test]
    fn rotor_forward_induced_velocity_recovers_hover_at_zero_speed() {
        let v_h = rotor_hover_induced_velocity(T, RHO, R).unwrap();
        let v_i = rotor_forward_induced_velocity(T, RHO, R, 0.0).unwrap();
        assert!(
            (v_i - v_h).abs() < 1.0e-6,
            "v_i(V=0)={v_i} should equal v_h={v_h}"
        );
    }

    #[test]
    fn rotor_forward_induced_velocity_decreases_with_forward_speed() {
        let v_h = rotor_hover_induced_velocity(T, RHO, R).unwrap();
        let v_f = rotor_forward_induced_velocity(T, RHO, R, 30.0).unwrap();
        // Glauert: increasing V_a decreases v_i (forward flight unloads the
        // disk — rotor inflow falls).
        assert!(
            v_f < v_h,
            "forward induced vel {v_f} should be less than hover {v_h}"
        );
        // Always positive in the flight regime.
        assert!(v_f > 0.0);
    }

    #[test]
    fn rotor_climb_induced_velocity_physics() {
        let v_h = rotor_hover_induced_velocity(T, RHO, R).unwrap();
        let v_i_climb = rotor_climb_induced_velocity(T, RHO, R, 3.0).unwrap();
        // climbing aircraft sees v_i < v_h (climb rate adds free-stream
        // momentum through the disk, so induced inflow drops).
        assert!(
            v_i_climb < v_h,
            "climb-induced vel {v_i_climb} should be less than hover {v_h}"
        );
    }

    #[test]
    fn rotor_tip_speed_and_advance_ratio_basic() {
        let omega = 40.0; // rad/s, ~380 RPM
        let v_tip = rotor_tip_speed(omega, R).unwrap();
        assert!((v_tip - omega * R).abs() < 1.0e-6);
        let mu = rotor_advance_ratio(15.0, omega, R).unwrap();
        assert!((mu - 15.0 / (omega * R)).abs() < 1.0e-6);
        // invalid
        assert!(rotor_tip_speed(-1.0, R).is_none());
        assert!(rotor_advance_ratio(15.0, 0.0, R).is_none());
        assert!(rotor_advance_ratio(15.0, omega, 0.0).is_none());
    }
}
