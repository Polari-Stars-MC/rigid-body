//! Tests for [`mps_formula::motors`].
//!
//! The assertions here pin **invariants**, not datasheet numbers: monotonicity,
//! the SI identity `k_t = k_e`, energy balance, and limit behaviour. Every
//! concrete parameter set is a worked example that happens to be physically
//! plausible — none of it is a claim about a real product.

#[cfg(test)]
mod tests {
    use mps_formula::motors::*;

    /// A small permanent-magnet DC gearmotor: 24 V, 2 Ω, k_e = 0.05 V·s/rad.
    /// Plausible for a hobby winch; not a datasheet.
    fn dc() -> DcMotor {
        DcMotor {
            voltage: 24.0,
            resistance: 2.0,
            ke: 0.05,
            no_load_torque: 0.02,
        }
    }

    // ---------------------------------------------------------------- DC motor

    #[test]
    fn dc_stall_and_no_load_speed_match_the_line() {
        let m = dc();
        // τ_stall = k_t·V/R − τ_friction = 0.05·24/2 − 0.02 = 0.58
        assert!((m.stall_torque().unwrap() - 0.58).abs() < 1e-12);
        // I_stall = V/R = 12 A
        assert!((m.stall_current().unwrap() - 12.0).abs() < 1e-12);
        // The ideal (circuit) no-load speed is V/k_e = 480 rad/s.
        assert!((m.ideal_no_load_speed().unwrap() - 480.0).abs() < 1e-9);
        // The real one is slower, because friction must be overcome first:
        // ω = (V − τ_f·R/k_e)/k_e = (24 − 0.02·2/0.05)/0.05 = 464
        assert!((m.no_load_speed().unwrap() - 464.0).abs() < 1e-9);
        // At stall the model must return the stall torque.
        assert!((m.torque_at_speed(0.0).unwrap() - 0.58).abs() < 1e-12);
    }

    #[test]
    fn dc_torque_falls_monotonically_with_speed() {
        let m = dc();
        let mut prev = f64::INFINITY;
        for i in 0..=100 {
            let omega = i as f64 * 5.0;
            let torque = m.torque_at_speed(omega).unwrap();
            assert!(
                torque < prev,
                "torque must decrease with speed: at {omega} rad/s got {torque} after {prev}"
            );
            prev = torque;
        }
    }

    #[test]
    fn dc_passes_through_zero_at_no_load_speed() {
        let m = dc();
        let omega = m.no_load_speed().unwrap();
        // At no-load speed the electromagnetic torque exactly cancels friction.
        assert!(m.torque_at_speed(omega).unwrap().abs() < 1e-12);
    }

    #[test]
    fn dc_speed_at_torque_inverts_torque_at_speed() {
        let m = dc();
        for i in 1..=10 {
            let omega = i as f64 * 30.0;
            let torque = m.torque_at_speed(omega).unwrap();
            if torque <= 0.0 {
                continue;
            }
            let back = m.speed_at_torque(torque).unwrap();
            assert!(
                (back - omega).abs() < 1e-9,
                "round trip failed: {omega} -> {torque} -> {back}"
            );
        }
    }

    #[test]
    fn dc_rejects_impossible_torque() {
        let m = dc();
        assert!(
            m.speed_at_torque(1.0).is_none(),
            "asking beyond stall torque must fail, not clamp"
        );
    }

    #[test]
    fn dc_efficiency_stays_physical() {
        let m = dc();
        for i in 1..=20 {
            let omega = i as f64 * 15.0;
            if let Some(eta) = m.efficiency(omega) {
                assert!(
                    eta > 0.0 && eta <= 1.0,
                    "efficiency out of (0, 1] at {omega} rad/s: {eta}"
                );
            }
        }
        // Peak efficiency of a DC machine sits near (but below) no-load speed.
        let best = (1..=40)
            .filter_map(|i| m.efficiency(i as f64 * 10.0))
            .fold(0.0f64, f64::max);
        assert!(
            best > 0.5,
            "a plausible DC machine should exceed 50%: {best}"
        );
        assert!(best < 1.0, "and never reach 100%: {best}");
    }

    #[test]
    fn dc_regenerates_above_no_load_speed() {
        let m = dc();
        // Above the *friction-limited* no-load speed the shaft torque is
        // negative: the machine is being back-driven.
        let omega = m.no_load_speed().unwrap() * 1.05;
        let torque = m.torque_at_speed(omega).unwrap();
        assert!(torque < 0.0, "above no-load speed it must brake: {torque}");
        assert!(
            m.shaft_power(omega).unwrap() < 0.0,
            "braking means negative shaft power"
        );
    }

    #[test]
    fn dc_rejects_non_physical_parameters() {
        let mut m = dc();
        m.resistance = 0.0;
        assert!(m.torque_at_speed(10.0).is_none());
        let mut m = dc();
        m.ke = -1.0;
        assert!(m.torque_at_speed(10.0).is_none());
        let mut m = dc();
        m.no_load_torque = -1.0;
        assert!(m.torque_at_speed(10.0).is_none());
    }

    #[test]
    fn kt_equals_ke_in_si() {
        // The identity this module documents; if someone "fixes" kt_from_ke to
        // apply a conversion factor, this fails and forces a conversation.
        assert_eq!(kt_from_ke(0.05).unwrap(), 0.05);
    }

    // ------------------------------------------------------------ series motor

    fn series() -> SeriesMotor {
        SeriesMotor {
            torque_constant: 50.0,
            speed_offset: 10.0,
        }
    }

    #[test]
    fn series_torque_falls_but_never_reaches_zero() {
        let m = series();
        let t0 = m.torque_at_speed(0.0).unwrap();
        let t1 = m.torque_at_speed(100.0).unwrap();
        let t2 = m.torque_at_speed(10_000.0).unwrap();
        assert!(t0 > t1 && t1 > t2, "must decrease: {t0} {t1} {t2}");
        assert!(t2 > 0.0, "a series motor never reaches zero torque: {t2}");
    }

    #[test]
    fn series_stall_torque_is_bounded_by_the_speed_offset() {
        let m = series();
        // τ(0) = k/ω_0 — finite only because of the offset, which is the
        // modelling fiction that keeps the hyperbola from diverging.
        assert!((m.torque_at_speed(0.0).unwrap() - 5.0).abs() < 1e-12);
    }

    #[test]
    fn series_speed_limit_keeps_power_finite() {
        let m = series();
        // A 20 W cap must yield a finite speed well below runaway.
        let limit = m.speed_limit_for_power(20.0).unwrap();
        assert!(limit > 0.0 && limit.is_finite(), "limit {limit}");
        // At that speed the model's power equals the cap.
        let power = m.torque_at_speed(limit).unwrap() * limit;
        assert!(
            (power - 20.0).abs() < 1e-6,
            "power at the cap should be 20 W, got {power}"
        );
    }

    #[test]
    fn series_has_no_speed_for_zero_torque() {
        let m = series();
        assert!(
            m.speed_at_torque(0.0).is_none(),
            "runaway: there is no speed where torque hits zero"
        );
    }

    // --------------------------------------------------------- brushless motor

    fn bldc() -> BrushlessMotor {
        BrushlessMotor {
            bus_voltage: 48.0,
            phase_resistance: 0.1,
            ke: 0.02,
            cogging_torque: 0.01,
            cogging_periods: 12,
            base_speed: 0.0,
        }
    }

    #[test]
    fn bldc_torque_falls_with_speed_and_stops_at_no_load() {
        let m = bldc();
        let t0 = m.torque_at_speed(0.0).unwrap();
        let t_mid = m.torque_at_speed(1200.0).unwrap();
        assert!(t0 > t_mid && t_mid > 0.0, "{t0} {t_mid}");
        let no_load = m.no_load_speed().unwrap();
        assert!((no_load - 2400.0).abs() < 1e-9);
        assert!(m.torque_at_speed(no_load).unwrap() < 1e-12);
        assert_eq!(m.torque_at_speed(no_load * 2.0).unwrap(), 0.0);
    }

    #[test]
    fn bldc_cogging_ripple_is_bounded_by_its_amplitude() {
        let m = bldc();
        let omega = 600.0;
        let average = m.torque_at_speed(omega).unwrap();
        for i in 0..64 {
            let theta = i as f64 * std::f64::consts::TAU / 64.0;
            let instant = m.torque_at(omega, theta).unwrap();
            assert!(
                (instant - average).abs() <= m.cogging_torque + 1e-12,
                "ripple exceeded its amplitude at theta {theta}: {instant} vs {average}"
            );
        }
    }

    #[test]
    fn bldc_cogging_has_the_declared_period_count() {
        let m = bldc();
        let omega = 600.0;
        let average = m.torque_at_speed(omega).unwrap();
        // Count sign changes of (instant − average) over one revolution,
        // excluding the wrap point: a sine with n periods changes sign 2n times
        // per revolution. Sampling `0..samples` (not `0..=samples`) avoids
        // counting theta = 0 twice.
        let samples = 2048;
        let mut crossings: usize = 0;
        let mut prev = m.torque_at(omega, 0.0).unwrap() - average;
        for i in 1..samples {
            let theta = i as f64 * std::f64::consts::TAU / samples as f64;
            let cur = m.torque_at(omega, theta).unwrap() - average;
            if (prev > 0.0) != (cur > 0.0) {
                crossings += 1;
            }
            prev = cur;
        }
        let expected = 2 * m.cogging_periods as usize;
        assert!(
            crossings.abs_diff(expected) <= 2,
            "expected ~{expected} sign changes, got {crossings}"
        );
    }

    #[test]
    fn bldc_field_weakening_holds_a_voltage_floor() {
        let mut m = bldc();
        m.base_speed = 1000.0;
        // 48 V bus, k_e = 0.02: back-EMF exceeds the bus above 2400 rad/s.
        assert_eq!(bldc().effective_voltage(2400.0).unwrap(), 0.0);
        assert_eq!(bldc().effective_voltage(3000.0).unwrap(), 0.0);
        // Below that the plain machine still has voltage left.
        assert!((bldc().effective_voltage(2000.0).unwrap() - 8.0).abs() < 1e-9);
        // Field weakening holds a 10%-of-bus floor instead of collapsing to zero.
        let weakened = m.effective_voltage(3000.0).unwrap();
        assert!((weakened - 4.8).abs() < 1e-9, "10% of bus: {weakened}");
        // And such a machine has no finite no-load speed.
        assert!(m.no_load_speed().is_none());
    }

    // ----------------------------------------------------------- stepper motor

    fn stepper() -> StepperMotor {
        StepperMotor {
            holding_torque: 0.4,
            corner_speed: 50.0,
            steps_per_revolution: 200,
            rotor_inertia: 1.0e-5,
        }
    }

    #[test]
    fn stepper_step_angle_matches_steps_per_revolution() {
        let m = stepper();
        let step = m.step_angle().unwrap();
        assert!((step - std::f64::consts::TAU / 200.0).abs() < 1e-12);
        // A 1.8° motor.
        assert!((step.to_degrees() - 1.8).abs() < 1e-12);
    }

    #[test]
    fn stepper_torque_is_holding_at_rest_and_falls_above_corner() {
        let m = stepper();
        assert!((m.torque_at_speed(0.0).unwrap() - 0.4).abs() < 1e-12);
        // At the corner speed torque is holding/√2.
        let at_corner = m.torque_at_speed(50.0).unwrap();
        assert!((at_corner - 0.4 / 2f64.sqrt()).abs() < 1e-12);
        // Well above the corner it falls as 1/ω.
        let fast = m.torque_at_speed(500.0).unwrap();
        let faster = m.torque_at_speed(1000.0).unwrap();
        assert!(
            (fast / faster - 2.0).abs() < 0.05,
            "1/ω tail: {fast} {faster}"
        );
    }

    #[test]
    fn stepper_speed_at_torque_inverts() {
        let m = stepper();
        for i in 1..=8 {
            let torque = 0.4 * i as f64 / 10.0;
            let speed = m.speed_at_torque(torque).unwrap();
            let back = m.torque_at_speed(speed).unwrap();
            assert!(
                (back - torque).abs() < 1e-9,
                "round trip: {torque} -> {speed} -> {back}"
            );
        }
        // Above holding torque there is no such speed.
        assert!(m.speed_at_torque(1.0).is_none());
    }

    #[test]
    fn stepper_start_rate_is_finite_and_needs_inertia() {
        let m = stepper();
        let rate = m.max_start_rate().unwrap();
        assert!(rate > 0.0 && rate.is_finite(), "start rate {rate}");
        // A motor with no rotor inertia has no start-rate limit in this model.
        let mut frictionless = m;
        frictionless.rotor_inertia = 0.0;
        assert!(frictionless.max_start_rate().is_none());
        // Heavier rotor => lower start rate.
        let mut heavy = m;
        heavy.rotor_inertia = 1.0e-4;
        assert!(heavy.max_start_rate().unwrap() < rate);
    }

    #[test]
    fn stepper_position_wraps_into_one_revolution() {
        let m = stepper();
        for steps in [-1000i64, -1, 0, 1, 200, 201, 9999] {
            let pos = m.position_after_steps(steps).unwrap();
            assert!(
                (0.0..std::f64::consts::TAU).contains(&pos),
                "position must be in [0, 2pi) for {steps} steps, got {pos}"
            );
        }
        // One full revolution returns to zero.
        assert!(m.position_after_steps(200).unwrap().abs() < 1e-12);
    }

    // --------------------------------------------------------- induction motor

    fn induction() -> InductionMotor {
        InductionMotor {
            frequency: 50.0,
            pole_pairs: 2,
            breakdown_torque: 100.0,
            breakdown_slip: 0.2,
        }
    }

    #[test]
    fn induction_synchronous_speed_matches_pole_pairs() {
        let m = induction();
        // 50 Hz, 2 pole pairs = 4-pole machine = 1500 rpm = 157.08 rad/s.
        let sync = m.synchronous_speed().unwrap();
        assert!((sync - 2.0 * std::f64::consts::PI * 25.0).abs() < 1e-9);
        let rpm = sync * 60.0 / std::f64::consts::TAU;
        assert!(
            (rpm - 1500.0).abs() < 1e-9,
            "4-pole at 50 Hz is 1500 rpm: {rpm}"
        );
    }

    #[test]
    fn induction_torque_peaks_at_breakdown_slip() {
        let m = induction();
        let peak = m.torque_at_slip(m.breakdown_slip).unwrap();
        assert!((peak - m.breakdown_torque).abs() < 1e-9);
        // Either side of the peak, torque is lower.
        assert!(m.torque_at_slip(0.05).unwrap() < peak);
        assert!(m.torque_at_slip(0.6).unwrap() < peak);
    }

    #[test]
    fn induction_makes_no_torque_at_synchronous_speed() {
        let m = induction();
        let sync = m.synchronous_speed().unwrap();
        assert!(
            m.torque_at_speed(sync).is_none(),
            "at synchronous speed there is no slip and therefore no torque"
        );
        assert!(
            m.torque_at_speed(sync * 1.1).is_none(),
            "above synchronous speed is generating, not covered here"
        );
    }

    #[test]
    fn induction_starting_torque_is_well_below_breakdown() {
        let m = induction();
        let start = m.starting_torque().unwrap();
        assert!(start > 0.0, "must be able to start: {start}");
        assert!(
            start < m.breakdown_torque * 0.7,
            "starting torque is typically well under breakdown: {start}"
        );
    }

    #[test]
    fn induction_speed_at_torque_returns_the_stable_branch() {
        let m = induction();
        for i in 1..=9 {
            let torque = m.breakdown_torque * i as f64 / 10.0;
            let speed = m.speed_at_torque(torque).unwrap();
            let back = m.torque_at_speed(speed).unwrap();
            assert!(
                (back - torque).abs() < 1e-6,
                "round trip: {torque} -> {speed} -> {back}"
            );
            // The stable branch is the high-speed one, i.e. low slip.
            let sync = m.synchronous_speed().unwrap();
            assert!(speed > sync * 0.5, "stable branch should be fast: {speed}");
        }
        assert!(m.speed_at_torque(m.breakdown_torque * 1.5).is_none());
    }

    // ------------------------------------------------------------- servo motor

    fn servo() -> ServoMotor {
        ServoMotor {
            rated_torque: 2.0,
            peak_torque: 6.0,
            rated_speed: 300.0,
            max_speed: 600.0,
        }
    }

    #[test]
    fn servo_is_constant_torque_then_constant_power() {
        let m = servo();
        // Constant torque region.
        assert_eq!(m.torque_at_speed(0.0).unwrap(), 2.0);
        assert_eq!(m.torque_at_speed(300.0).unwrap(), 2.0);
        // Constant power region: τ·ω is invariant.
        let p1 = m.torque_at_speed(400.0).unwrap() * 400.0;
        let p2 = m.torque_at_speed(600.0).unwrap() * 600.0;
        assert!(
            (p1 - p2).abs() < 1e-6,
            "constant power region: {p1} vs {p2}"
        );
        // Beyond max speed the envelope is zero.
        assert_eq!(m.torque_at_speed(700.0).unwrap(), 0.0);
    }

    #[test]
    fn servo_saturates_a_demand_beyond_the_envelope() {
        let m = servo();
        // Demand 100 N·m at rest: peak is 6.
        assert!((m.available_torque(0.0, 100.0).unwrap() - 6.0).abs() < 1e-12);
        // A modest demand passes through untouched.
        assert!((m.available_torque(0.0, 1.5).unwrap() - 1.5).abs() < 1e-12);
        // Negative demand symmetric.
        assert!((m.available_torque(0.0, -100.0).unwrap() + 6.0).abs() < 1e-12);
    }

    #[test]
    fn servo_continuous_rating_is_below_peak() {
        let m = servo();
        assert!(m.within_continuous_rating(0.0, 2.0).unwrap());
        assert!(!m.within_continuous_rating(0.0, 3.0).unwrap());
        // Above base speed the continuous envelope shrinks, so 2 N·m no longer fits.
        assert!(!m.within_continuous_rating(600.0, 2.0).unwrap());
    }

    #[test]
    fn servo_rejects_peak_below_rated() {
        let mut m = servo();
        m.peak_torque = 1.0;
        assert!(m.torque_at_speed(0.0).is_none());
    }

    // ------------------------------------------------------------ linear motor

    #[test]
    fn linear_motor_envelope_matches_its_rotary_twin() {
        let m = LinearMotor {
            rated_force: 500.0,
            peak_force: 1500.0,
            rated_speed: 1.0,
            max_speed: 3.0,
        };
        assert_eq!(m.force_at_speed(0.5).unwrap(), 500.0);
        let p1 = m.force_at_speed(1.5).unwrap() * 1.5;
        let p2 = m.force_at_speed(3.0).unwrap() * 3.0;
        assert!((p1 - p2).abs() < 1e-6, "constant power: {p1} vs {p2}");
        assert_eq!(m.force_at_speed(4.0).unwrap(), 0.0);
        // Saturation uses peak force.
        assert!((m.available_force(0.0, 1e9).unwrap() - 1500.0).abs() < 1e-12);
    }

    // ---------------------------------------------------------------- piezo

    fn piezo() -> PiezoActuator {
        PiezoActuator {
            blocking_force: 100.0,
            free_displacement: 20.0e-6,
            resonance_hz: 0.0,
        }
    }

    #[test]
    fn piezo_force_falls_linearly_to_zero_at_full_stroke() {
        let m = piezo();
        assert!((m.force_at_displacement(0.0).unwrap() - 100.0).abs() < 1e-12);
        let half = m.force_at_displacement(10.0e-6).unwrap();
        assert!((half - 50.0).abs() < 1e-9);
        assert!(m.force_at_displacement(20.0e-6).unwrap().abs() < 1e-12);
        // Beyond the stroke it simply does not push.
        assert_eq!(m.force_at_displacement(30.0e-6).unwrap(), 0.0);
    }

    #[test]
    fn piezo_work_peaks_at_half_the_blocking_force() {
        let m = piezo();
        let matched = m.matched_load_force().unwrap();
        assert!((matched - 50.0).abs() < 1e-12);
        let at_matched = m.work_per_stroke(matched).unwrap();
        // Either side is worse.
        assert!(m.work_per_stroke(25.0).unwrap() < at_matched);
        assert!(m.work_per_stroke(75.0).unwrap() < at_matched);
        // And it equals F·ΔL/4 at the matched load.
        let expected = 50.0 * 10.0e-6;
        assert!(
            (at_matched - expected).abs() < 1e-12,
            "{at_matched} vs {expected}"
        );
        // Beyond blocking force there is no work.
        assert!(m.work_per_stroke(150.0).is_none());
    }

    #[test]
    fn piezo_stiffness_is_blocking_force_over_stroke() {
        let m = piezo();
        let k = m.stiffness().unwrap();
        assert!((k - 100.0 / 20.0e-6).abs() < 1e-3, "{k}");
    }

    #[test]
    fn quasi_static_piezo_has_no_resonant_frequency() {
        let m = piezo();
        assert!(m.max_frequency().is_none());
        let resonant = PiezoActuator {
            resonance_hz: 40_000.0,
            ..m
        };
        assert_eq!(resonant.max_frequency().unwrap(), 40_000.0);
    }

    // -------------------------------------------------------------------- SMA

    fn sma() -> ShapeMemoryActuator {
        ShapeMemoryActuator {
            rest_length: 0.1,
            recoverable_strain: 0.04,
            recovery_stress: 200.0e6,
            cross_section: 1.0e-6,
            austenite_finish: 340.0,
        }
    }

    #[test]
    fn sma_blocking_force_and_stroke_follow_stress_and_strain() {
        let m = sma();
        assert!((m.blocking_force().unwrap() - 200.0).abs() < 1e-9);
        assert!((m.free_stroke().unwrap() - 0.004).abs() < 1e-12);
    }

    #[test]
    fn sma_actuation_is_a_bounded_smooth_step() {
        let m = sma();
        // Cold: no actuation.
        assert!(m.actuation_fraction(300.0).unwrap() < 0.05);
        // At the finish temperature: half way.
        assert!((m.actuation_fraction(340.0).unwrap() - 0.5).abs() < 1e-12);
        // Hot: fully actuated.
        assert!(m.actuation_fraction(380.0).unwrap() > 0.95);
        // Monotonic and always in [0, 1].
        let mut prev = -1.0;
        for i in 0..=100 {
            let t = 250.0 + i as f64;
            let f = m.actuation_fraction(t).unwrap();
            assert!(
                (0.0..=1.0).contains(&f),
                "fraction out of range at {t}: {f}"
            );
            assert!(f >= prev, "must be monotonic at {t}: {f} < {prev}");
            prev = f;
        }
    }

    #[test]
    fn sma_work_peaks_at_half_the_blocking_force() {
        let m = sma();
        let blocking = m.blocking_force().unwrap();
        let at_matched = m.work_per_cycle(blocking * 0.5).unwrap();
        assert!(m.work_per_cycle(blocking * 0.25).unwrap() < at_matched);
        assert!(m.work_per_cycle(blocking * 0.75).unwrap() < at_matched);
        assert!(m.work_per_cycle(blocking * 1.5).is_none());
    }

    #[test]
    fn sma_rejects_unphysical_strain() {
        let mut m = sma();
        m.recoverable_strain = 1.5;
        assert!(m.blocking_force().is_none());
        m.recoverable_strain = 0.0;
        assert!(m.free_stroke().is_none());
    }

    // --------------------------------------------------------- shared helpers

    #[test]
    fn copper_loss_grows_with_the_square_of_current() {
        assert!((copper_loss(2.0, 3.0).unwrap() - 12.0).abs() < 1e-12);
        assert!((copper_loss(4.0, 3.0).unwrap() - 48.0).abs() < 1e-12);
        assert!(copper_loss(1.0, -1.0).is_none());
    }

    #[test]
    fn back_emf_and_power_are_consistent() {
        let e = back_emf(0.05, 100.0).unwrap();
        assert!((e - 5.0).abs() < 1e-12);
        assert!((electrical_power(24.0, 2.0).unwrap() - 48.0).abs() < 1e-12);
        assert!((mechanical_power(2.0, 100.0).unwrap() - 200.0).abs() < 1e-12);
    }

    #[test]
    fn electrical_time_constant_needs_positive_resistance() {
        assert!((electrical_time_constant(0.01, 2.0).unwrap() - 0.005).abs() < 1e-12);
        assert!(electrical_time_constant(0.01, 0.0).is_none());
    }

    #[test]
    fn thermal_model_relaxes_toward_steady_state() {
        // A 10 W loss into a 2 K/W body from 300 K settles at 320 K.
        let steady = thermal_steady_state(10.0, 2.0, 300.0).unwrap();
        assert!((steady - 320.0).abs() < 1e-12);

        // Starting cold and stepping for a long time approaches that value.
        let mut t = 300.0;
        for _ in 0..20_000 {
            t = thermal_step(t, 300.0, 10.0, 100.0, 2.0, 0.1).unwrap();
        }
        assert!(
            (t - steady).abs() < 0.1,
            "should converge to {steady}, got {t}"
        );
    }

    #[test]
    fn thermal_model_cools_when_loss_stops() {
        let hot = 400.0;
        let cooled = thermal_step(hot, 300.0, 0.0, 100.0, 2.0, 1.0).unwrap();
        assert!(cooled < hot, "must cool without a loss: {cooled}");
        assert!(cooled > 300.0, "and not overshoot ambient: {cooled}");
    }

    #[test]
    fn thermal_margin_turns_negative_past_the_limit() {
        assert!((thermal_margin(350.0, 400.0).unwrap() - 50.0).abs() < 1e-12);
        assert!(thermal_margin(450.0, 400.0).unwrap() < 0.0);
    }

    #[test]
    fn non_finite_inputs_are_rejected_everywhere() {
        assert!(mechanical_power(f64::NAN, 1.0).is_none());
        assert!(electrical_power(1.0, f64::INFINITY).is_none());
        assert!(back_emf(0.05, f64::NAN).is_none());
        assert!(dc().torque_at_speed(f64::NAN).is_none());
        assert!(series().torque_at_speed(f64::INFINITY).is_none());
        assert!(bldc().torque_at_speed(f64::NAN).is_none());
        assert!(stepper().torque_at_speed(f64::NAN).is_none());
        assert!(induction().torque_at_speed(f64::NAN).is_none());
        assert!(servo().torque_at_speed(f64::NAN).is_none());
        assert!(piezo().force_at_displacement(f64::NAN).is_none());
        assert!(sma().actuation_fraction(f64::NAN).is_none());
    }
}
