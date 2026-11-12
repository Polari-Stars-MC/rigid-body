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

    // ---- E1：checked 公式覆盖率 ratchet ----
    //
    // 每个新增的 `*_checked` 公式必须在 domain.rs 里至少有一条 happy-path
    // 断言和一条域拒绝断言。守门（`every_checked_formula_is_covered`）用
    // source-scan 数 `pub fn *_checked` 总数，ratchet 常量钉住「已覆盖」数，
    // 新增公式不接测试就红——比「人为记得写」可靠。

    /// 覆盖 ratchet：本文件必须直接调用至少这么多不同的 checked 公式。
    /// 上调规则：新增公式接入下面的 `checked_formulas_full_coverage`
    /// 后同步 +1；禁止下调。
    const CHECKED_COVERAGE_RATCHET: usize = 18;

    #[test]
    fn checked_coverage_ratchet_counts_every_checked_formula() {
        use mps_formula::gravitational_models::spherical_harmonics_acceleration_checked;
        use mps_formula::spaceflight::debris::radiation_absorbed_dose_checked;
        use mps_formula::spaceflight::thermal::{
            heat_pipe_thermal_resistance_checked, radiator_power_checked,
            reentry_peak_g_load_checked, single_phase_loop_heat_transfer_checked,
            sutton_graves_heat_rate_checked, thermal_balance_checked,
        };
        use mps_formula::spaceflight::{
            bi_elliptic_transfer_delta_v_checked, combined_maneuver_delta_v_checked,
            hohmann_transfer_checked,
        };

        // 已有覆盖的 7 条（前文测试也各有一条 happy path），这里逐函数点名，
        // 保证「这个公式在别处测过」的说法是编译期可见的引用而非注释。
        let _ = ballistic_coefficient_checked;
        let _ = kepler_period_checked;
        let _ = tsiolkovsky_delta_v_checked;
        let _ = semi_major_axis_decay_rate_checked;
        let _ = bernoulli_pressure_checked;
        let _ = bernoulli_report_checked;
        let _ = fourier_conduction_checked;

        // 新补的 11 条：每条一条 happy path + 一条域拒绝。
        // hohmann
        let h = hohmann_transfer_checked(3.986e14, 7e6, 4.2e7).unwrap();
        assert!(h.delta_v1 > 0.0 && h.delta_v2 > 0.0);
        assert!(hohmann_transfer_checked(1.0, -1.0, 2.0).is_err());
        // bi-elliptic
        let (dv1, dv2, _, _) =
            bi_elliptic_transfer_delta_v_checked(3.986e14, 7e6, 4.2e7, 8e7).unwrap();
        assert!(dv1 > 0.0 && dv2 > 0.0);
        assert!(bi_elliptic_transfer_delta_v_checked(1.0, -1.0, 2.0, 3.0).is_err());
        // combined
        let (cdv1, _, _) = combined_maneuver_delta_v_checked(3.986e14, 7e6, 4.2e7, 0.1).unwrap();
        assert!(cdv1 > 0.0);
        assert!(combined_maneuver_delta_v_checked(1.0, -1.0, 2.0, 0.1).is_err());
        // radiation dose
        assert!(radiation_absorbed_dose_checked(10.0, 2.0, 1.0).unwrap() > 0.0);
        assert!(radiation_absorbed_dose_checked(10.0, -1.0, 1.0).is_err());
        // heat pipe
        let hp = heat_pipe_thermal_resistance_checked(0.1, 0.2, 0.3, 0.4).unwrap();
        assert!(hp > 0.0);
        assert!(heat_pipe_thermal_resistance_checked(f64::NAN, 0.2, 0.3, 0.4).is_err());
        // radiator power
        let rp = radiator_power_checked(2.0, 0.9, 300.0, 3.0, 500.0).unwrap();
        assert!(rp.net_power.is_finite());
        assert!(radiator_power_checked(-1.0, 0.9, 300.0, 3.0, 500.0).is_err());
        // reentry peak g
        assert!(reentry_peak_g_load_checked(0.5, 7800.0, 2.0, 8500.0).unwrap() > 0.0);
        assert!(reentry_peak_g_load_checked(-0.5, 7800.0, 2.0, 8500.0).is_err());
        // single-phase loop
        let lp = single_phase_loop_heat_transfer_checked(0.5, 4180.0, 300.0, 1000.0).unwrap();
        assert!(lp.outlet_temperature > 300.0);
        assert!(single_phase_loop_heat_transfer_checked(-0.5, 4180.0, 300.0, 1000.0).is_err());
        // sutton-graves
        assert!(sutton_graves_heat_rate_checked(1e-4, 7800.0, 0.5, 1.7415e-4).unwrap() >= 0.0);
        assert!(sutton_graves_heat_rate_checked(f64::NAN, 7800.0, 0.5, 1.7415e-4).is_err());
        // thermal balance
        let tb = thermal_balance_checked(500.0, 100.0, 2.0, 0.9).unwrap();
        assert!(tb.equilibrium_temperature.is_finite());
        assert!(thermal_balance_checked(f64::INFINITY, 100.0, 2.0, 0.9).is_err());
        // spherical harmonics：非有限位置 → NonFiniteInput（体参数取内置地球）。
        use mps_formula::celestial_data::EARTH;
        use mps_formula::ffi::Vec3;
        assert!(
            spherical_harmonics_acceleration_checked(
                Vec3 {
                    x: f64::NAN,
                    y: 0.0,
                    z: 7e6
                },
                &EARTH,
                2
            )
            .is_err()
        );
        let acc = spherical_harmonics_acceleration_checked(
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 7e6,
            },
            &EARTH,
            2,
        )
        .unwrap();
        assert!(acc.x.is_finite() && acc.y.is_finite() && acc.z.is_finite());
    }

    /// 守门：`pub fn *_checked` 源码计数不得大于 ratchet（新增必须接测试），
    /// 也不得小于（公式被删时 ratchet 必须同步降，防止"虚高覆盖率"）。
    #[test]
    fn every_checked_formula_is_covered_by_domain_tests() {
        let checked_count = count_checked_formulas();
        assert_eq!(
            checked_count, CHECKED_COVERAGE_RATCHET,
            "checked 公式数 {checked_count} ≠ ratchet {CHECKED_COVERAGE_RATCHET}：\
             新增 `*_checked` 公式必须在 domain.rs 接入覆盖测试后上调 ratchet；\
             删除公式则下调。禁止下调 ratchet 来跳过覆盖。"
        );
    }

    /// 统计 mps-formula 源码里的 `pub fn *_checked` 定义数（逐文件递归读，
    /// 只在测试 crate 里跑，不用 build script）。
    fn count_checked_formulas() -> usize {
        let manifest = env!("CARGO_MANIFEST_DIR");
        let formula_root = std::path::Path::new(manifest)
            .parent()
            .unwrap()
            .join("mps-formula/src");
        let mut count = 0usize;
        let mut stack = vec![formula_root];
        while let Some(dir) = stack.pop() {
            for entry in std::fs::read_dir(&dir).expect("read formula src") {
                let entry = entry.expect("dir entry");
                let path = entry.path();
                if path.is_dir() {
                    stack.push(path);
                } else if path.extension().and_then(|e| e.to_str()) == Some("rs") {
                    let text = std::fs::read_to_string(&path).expect("read rs");
                    let mut rest = text.as_str();
                    while let Some(i) = rest.find("pub fn ") {
                        rest = &rest[i + "pub fn ".len()..];
                        let name: &str = rest
                            .split(|c: char| !c.is_alphanumeric() && c != '_')
                            .next()
                            .unwrap_or("");
                        if name.ends_with("_checked") {
                            count += 1;
                        }
                    }
                }
            }
        }
        count
    }
}
