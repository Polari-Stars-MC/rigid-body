//! Numeric-domain contract tests for the pure formula layer and its FFI adapters.

#[cfg(test)]
mod tests {
    use mps_formula::domain::{self, FormulaError};
    use mps_formula::error::{
        ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ERR_OK, clear_error, error_code, set_error,
    };
    use mps_formula::ffi::{Bool, HeatConductionReport};
    use mps_formula::fluid::{bernoulli_pressure_checked, bernoulli_report_checked};
    use mps_formula::spaceflight::{
        ballistic_coefficient_checked, kepler_period_checked, semi_major_axis_decay_rate_checked,
        tsiolkovsky_delta_v_checked,
    };
    use mps_formula::thermodynamics::{fourier_conduction_checked, thermal_fourier_conduction};

    fn reject_nonfinite<const N: usize>(
        valid: [f64; N],
        eval: impl Fn([f64; N]) -> Result<f64, FormulaError>,
    ) {
        assert!(eval(valid).unwrap().is_finite());
        for index in 0..N {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut args = valid;
                args[index] = value;
                assert!(
                    matches!(eval(args), Err(FormulaError::NonFiniteInput { .. })),
                    "index {index}, value {value}"
                );
            }
        }
    }

    #[test]
    fn checked_formulas_reject_nonfinite_in_every_parameter() {
        reject_nonfinite([1.0, 2.0, 3.0], |[m, cd, a]| {
            ballistic_coefficient_checked(m, cd, a)
        });
        reject_nonfinite([3.986e14, 7e6], |[mu, a]| kepler_period_checked(mu, a));
        reject_nonfinite([300.0, 9.81, 1000.0, 500.0], |[isp, g, m0, mf]| {
            tsiolkovsky_delta_v_checked(isp, g, m0, mf)
        });
        reject_nonfinite(
            [7e6, 1e-12, 2.0, 10.0, 1000.0, 3.986e14],
            |[a, rho, cd, area, m, mu]| semi_major_axis_decay_rate_checked(a, rho, cd, area, m, mu),
        );
        reject_nonfinite([100000.0, 1000.0, 10.0, 9.81, 2.0], |[p, rho, v, g, h]| {
            bernoulli_pressure_checked(p, rho, v, g, h)
        });
        reject_nonfinite([100000.0, 1000.0, 10.0, 9.81, 2.0], |[p, rho, v, g, h]| {
            bernoulli_report_checked(p, rho, v, g, h).map(|v| v.total_head)
        });
        reject_nonfinite([400.0, 300.0, 10.0, 2.0, 0.5], |[hot, cold, k, area, d]| {
            fourier_conduction_checked(hot, cold, k, area, d).map(|v| v.heat_rate)
        });
    }

    #[test]
    fn elementary_domains_reject_singularities_and_overflow() {
        assert_eq!(domain::sqrt(0.0), Ok(0.0));
        assert_eq!(domain::sqrt(4.0), Ok(2.0));
        assert!(matches!(
            domain::sqrt(-1.0),
            Err(FormulaError::OutOfDomain { .. })
        ));
        assert_eq!(domain::ln(1.0), Ok(0.0));
        for x in [-1.0, 0.0, -0.0] {
            assert!(matches!(
                domain::ln(x),
                Err(FormulaError::OutOfDomain { .. })
            ));
        }
        for x in [0.0, -0.0] {
            assert_eq!(domain::divide(1.0, x), Err(FormulaError::DivisionByZero));
        }
        assert!(matches!(
            domain::divide(f64::MAX, f64::MIN_POSITIVE),
            Err(FormulaError::NonFiniteResult { .. })
        ));
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(domain::sqrt(value).is_err());
            assert!(domain::ln(value).is_err());
            assert!(domain::divide(value, 1.0).is_err());
            assert!(domain::divide(1.0, value).is_err());
        }
    }

    #[test]
    fn physical_domains_distinguish_zero_limits_from_invalid_inputs() {
        for mass in [-1.0, 0.0, -0.0] {
            assert!(ballistic_coefficient_checked(mass, 1.0, 1.0).is_err());
            assert!(tsiolkovsky_delta_v_checked(300.0, 9.81, 1000.0, mass).is_err());
        }
        assert!(tsiolkovsky_delta_v_checked(300.0, 9.81, 1.0, 2.0).is_err());
        assert_eq!(
            tsiolkovsky_delta_v_checked(f64::MAX, f64::MAX, 1.0, 1.0),
            Ok(0.0)
        );
        assert!(
            semi_major_axis_decay_rate_checked(7e6, -1.0, 2.0, 10.0, 1000.0, 3.986e14).is_err()
        );
        assert_eq!(
            semi_major_axis_decay_rate_checked(7e6, 0.0, 2.0, 10.0, 1000.0, 3.986e14),
            Ok(0.0)
        );
        assert!(bernoulli_pressure_checked(1.0, -1.0, 1.0, 1.0, 1.0).is_err());
        assert_eq!(
            bernoulli_pressure_checked(10.0, 2.0, 1.0, 0.0, 1.0),
            Ok(9.0)
        );
        for gravity in [0.0, -0.0] {
            assert!(matches!(
                bernoulli_report_checked(10.0, 2.0, 1.0, gravity, 1.0),
                Err(FormulaError::DivisionByZero)
            ));
        }
    }

    #[test]
    fn finite_inputs_cannot_silently_produce_nonfinite_success() {
        assert!(matches!(
            kepler_period_checked(1.0, f64::MAX),
            Err(FormulaError::NonFiniteResult { .. })
        ));
        assert!(ballistic_coefficient_checked(1.0, f64::MIN_POSITIVE, f64::MIN_POSITIVE).is_err());
        assert!(tsiolkovsky_delta_v_checked(f64::MAX, 10.0, 2.0, 1.0).is_err());
        assert!(bernoulli_pressure_checked(0.0, f64::MAX, f64::MAX, 1.0, 1.0).is_err());
        assert!(fourier_conduction_checked(f64::MAX, -f64::MAX, 1.0, 1.0, 1.0).is_err());
        assert!(mps_formula::spaceflight::kepler_period(1.0, f64::MAX).is_none());
    }

    #[test]
    fn documented_insulator_limit_is_the_only_infinite_conduction_output() {
        for (k, area) in [(0.0, 2.0), (10.0, 0.0), (0.0, 0.0)] {
            let report = fourier_conduction_checked(400.0, 300.0, k, area, 0.5).unwrap();
            assert_eq!(report.thermal_resistance, f64::INFINITY);
            assert_eq!(report.heat_rate, 0.0);
            assert!(report.heat_flux.is_finite());
            let mut out = HeatConductionReport::default();
            assert_eq!(
                thermal_fourier_conduction(400.0, 300.0, k, area, 0.5, &mut out),
                Bool::TRUE
            );
            assert_eq!(error_code(), ERR_OK);
            assert_eq!(out.thermal_resistance, f64::INFINITY);
        }
    }

    #[test]
    fn ffi_reports_domain_failures_without_writing_and_clears_on_success() {
        use mps_core::rapier::fluid::fluid_bernoulli_pressure;
        use mps_core::rapier::spaceflight::{space_kepler_period, space_tsiolkovsky_delta_v};
        let mut out = HeatConductionReport {
            heat_rate: 123.0,
            ..Default::default()
        };
        clear_error();
        assert_eq!(
            thermal_fourier_conduction(400.0, 300.0, 1.0, 1.0, 0.0, &mut out),
            Bool::FALSE
        );
        assert_eq!(error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(out.heat_rate, 123.0);
        assert_eq!(
            thermal_fourier_conduction(400.0, 300.0, 1.0, 1.0, 1.0, std::ptr::null_mut()),
            Bool::FALSE
        );
        assert_eq!(error_code(), ERR_NULL_POINTER);
        assert!(space_kepler_period(1.0, f64::MAX).is_nan());
        assert_eq!(error_code(), ERR_INVALID_ARGUMENT);
        assert!(space_tsiolkovsky_delta_v(300.0, 9.81, 1000.0, 0.0).is_nan());
        assert_eq!(error_code(), ERR_INVALID_ARGUMENT);
        assert!(fluid_bernoulli_pressure(1.0, -1.0, 1.0, 1.0, 1.0).is_nan());
        assert_eq!(error_code(), ERR_INVALID_ARGUMENT);
        assert!((space_kepler_period(1.0, 1.0) - std::f64::consts::TAU).abs() < 1e-14);
        assert_eq!(error_code(), ERR_OK);
    }

    #[test]
    fn pure_api_leaves_ffi_error_slot_untouched() {
        set_error(ERR_NULL_POINTER, "existing error");
        assert_eq!(ballistic_coefficient_checked(10.0, 2.0, 5.0), Ok(1.0));
        assert!(ballistic_coefficient_checked(-1.0, 2.0, 5.0).is_err());
        assert_eq!(error_code(), ERR_NULL_POINTER);
        let error = domain::positive(-1.0, "mass").unwrap_err();
        assert_eq!(error.to_string(), "mass must be positive");
    }
}
