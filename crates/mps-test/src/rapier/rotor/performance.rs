//! `rotor::performance` — power-accounting components and totals tests.

#[cfg(test)]
mod tests {
    use mps_core::rapier::rotor::*;

    fn rotor() -> RotorParams {
        RotorParams {
            radius: 5.4,
            n_blades: 2,
            chord: 0.20,
            hinge_offset: 0.05,
            lift_slope: 2.0 * std::f64::consts::PI,
            zero_lift_alpha: 0.0,
            profile_cd0: 0.012,
            cd_k: 0.05,
        }
    }

    #[test]
    fn profile_power_is_positive_and_scales_with_omega_cube() {
        let r = rotor();
        let p1 = rotor_profile_power(&r, 1.225, 40.0).unwrap();
        let p2 = rotor_profile_power(&r, 1.225, 80.0).unwrap();
        // σ C_d0 ρ A (ΩR)³ / 8 — doubling Ω should give 8× the power.
        assert!(p1 > 0.0 && p2 > 0.0);
        assert!(
            (p2 / p1 - 8.0).abs() < 1.0e-9,
            "p1={}, p2={}, ratio={}",
            p1,
            p2,
            p2 / p1
        );
        // bad inputs
        assert!(rotor_profile_power(&r, -1.0, 40.0).is_none());
        assert!(rotor_profile_power(&r, 1.225, 0.0).is_none());
    }

    #[test]
    fn total_power_sums_four_components() {
        let p = rotor_total_power(100.0, 50.0, 25.0, 10.0).unwrap();
        assert!((p - 185.0).abs() < 1.0e-12, "p={p}");
        // Rejections
        assert!(rotor_total_power(-1.0, 0.0, 0.0, 0.0).is_none());
        assert!(rotor_total_power(0.0, f64::NAN, 0.0, 0.0).is_none());
    }

    #[test]
    fn flat_plate_area_and_parasite_power() {
        let f = rotor_flat_plate_area(2.0, 0.3).unwrap();
        assert!((f - 0.6).abs() < 1.0e-12);
        let p = rotor_parasite_power(1.225, 30.0, f).unwrap();
        // P = ½ ρ V³ f  →  ½·1.225·27000·0.6  ≈  9922.5 W
        let expected = 0.5 * 1.225 * (30.0_f64).powi(3) * f;
        assert!((p - expected).abs() < 1.0e-9, "p={p}, expected={expected}");
        // bad inputs rejected
        assert!(rotor_flat_plate_area(0.0, 0.3).is_none());
        assert!(rotor_flat_plate_area(2.0, -1.0).is_none());
        assert!(rotor_parasite_power(0.0, 30.0, f).is_none());
        assert!(rotor_parasite_power(1.225, -1.0, f).is_none());
    }

    #[test]
    fn climb_power_signs() {
        // Climb: positive power.
        let p_climb = rotor_climb_power(1000.0, 2.0).unwrap();
        assert!((p_climb - 2000.0).abs() < 1.0e-12);
        // Descent: negative.
        // ...but rotor_climb_power rejects negative climb rates currently.
        // Verify zero climb = zero power.
        assert!(rotor_climb_power(1000.0, 0.0).unwrap().abs() < 1.0e-12);
        // Bad thrust
        assert!(rotor_climb_power(-1.0, 1.0).is_none());
    }

    #[test]
    fn hover_efficiency_alias_of_figure_of_merit() {
        let fm = rotor_hover_efficiency(700.0_f64, 1000.0).unwrap();
        assert!((fm - 0.7).abs() < 1.0e-12);
        assert!(rotor_hover_efficiency(-1.0, 1000.0).is_none());
        assert!(rotor_hover_efficiency(700.0, 0.0).is_none());
    }

    #[test]
    fn rotor_params_solidity_disk_area() {
        let r = rotor();
        let area = r.disk_area().unwrap();
        assert!(
            (area - std::f64::consts::PI * 5.4 * 5.4).abs() < 1.0e-9,
            "area={area}"
        );
        let sigma = r.solidity().unwrap();
        // σ = N c / (π R)  = 2 · 0.20 / (π · 5.4) ≈ 0.0236
        let expected = (2.0 * 0.20) / (std::f64::consts::PI * 5.4);
        assert!((sigma - expected).abs() < 1.0e-9, "sigma={sigma}");
        // Bad params → None
        let mut bad = r;
        bad.radius = 0.0;
        assert!(bad.disk_area().is_none());
        assert!(bad.solidity().is_none());
    }
}
