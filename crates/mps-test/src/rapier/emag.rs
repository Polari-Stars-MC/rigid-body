//! FFI wrapper tests for `mps_core::rapier::emag` (the C-ABI surface that
//! wraps `mps_formula::electromagnetism`). Mirrors the `emag` core module.
#[cfg(test)]
mod tests {
    use mps_core::rapier::emag::*;
    use mps_core::rapier::ffi::Bool;
    use std::ptr;

    #[test]
    fn phase_velocity_and_wavelength_round_trip() {
        let n: f64 = 1.5;
        let mut v = 0.0;
        assert_eq!(electromagnetism_phase_velocity(n, &mut v), Bool::TRUE);
        assert!((v - 299_792_458.0 / n).abs() < 1e-3);
        let mut lam = 0.0;
        assert_eq!(
            electromagnetism_vacuum_wavelength(1.0e9, &mut lam),
            Bool::TRUE
        );
        assert!((lam - 299_792_458.0 / 1.0e9).abs() < 1e-3);
    }

    #[test]
    fn reflection_vswr_return_loss_consistent() {
        let zl = 75.0;
        let z0 = 50.0;
        let mut gamma = 0.0;
        assert_eq!(
            electromagnetism_reflection_coefficient(zl, z0, &mut gamma),
            Bool::TRUE
        );
        let expected_gamma = (zl - z0) / (zl + z0);
        assert!((gamma - expected_gamma).abs() < 1e-9);
        let mut vswr = 0.0;
        assert_eq!(electromagnetism_vswr(gamma, &mut vswr), Bool::TRUE);
        let expected_vswr = (1.0 + gamma.abs()) / (1.0 - gamma.abs());
        assert!((vswr - expected_vswr).abs() < 1e-9);
    }

    #[test]
    fn half_wave_dipole_directivity_constant() {
        let mut d = 0.0;
        assert_eq!(
            electromagnetism_half_wave_dipole_directivity(&mut d),
            Bool::TRUE
        );
        assert!((d - 1.64).abs() < 1e-9);
    }

    #[test]
    fn transmission_line_writes_two_outputs() {
        let mut real = 0.0;
        let mut imag = 0.0;
        // z0=50, z_load=50 (real), beta=0, length=any -> Z_in = 50 + j0
        let ok = electromagnetism_transmission_line_input_impedance(
            50.0, 50.0, 0.0, 0.0, 1.0, &mut real, &mut imag,
        );
        assert_eq!(ok, Bool::TRUE);
        assert!((real - 50.0).abs() < 1e-6);
        assert!(imag.abs() < 1e-6);
    }

    #[test]
    fn null_output_returns_false() {
        assert_eq!(
            electromagnetism_wave_frequency(1.0, ptr::null_mut()),
            Bool::FALSE
        );
        assert_eq!(
            electromagnetism_transmission_line_input_impedance(
                50.0,
                50.0,
                0.0,
                0.0,
                1.0,
                ptr::null_mut(),
                &mut 0.0,
            ),
            Bool::FALSE
        );
    }

    #[test]
    fn invalid_input_returns_false() {
        let mut out = 0.0;
        // negative refractive index -> None
        assert_eq!(electromagnetism_phase_velocity(-1.0, &mut out), Bool::FALSE);
        // |Gamma| >= 1 -> None
        assert_eq!(electromagnetism_vswr(1.5, &mut out), Bool::FALSE);
    }
    // ── CIE 加权积分（ADR-0032）────────────────────────────────────────────

    /// 色匹配函数的三输出契约：成功时全部写入，失败时一个都不动。
    #[test]
    fn cie_color_matching_writes_three_outputs() {
        let (mut x, mut y, mut z) = (0.0f64, 0.0f64, 0.0f64);
        assert_eq!(
            tyndall_cie_color_matching(555.0, &mut x, &mut y, &mut z),
            Bool::TRUE
        );
        assert!((y - 1.0).abs() < 0.02, "ȳ(555) = {y}");
        assert!(x > 0.0 && z > 0.0);

        // 非有限波长必须失败且不写。
        let (mut x2, mut y2, mut z2) = (-1.0f64, -1.0f64, -1.0f64);
        assert_eq!(
            tyndall_cie_color_matching(f64::NAN, &mut x2, &mut y2, &mut z2),
            Bool::FALSE
        );
        assert_eq!((x2, y2, z2), (-1.0, -1.0, -1.0), "失败时不得写入");
    }

    #[test]
    fn cie_color_matching_rejects_null_outputs() {
        let mut v = 0.0f64;
        assert_eq!(
            tyndall_cie_color_matching(550.0, ptr::null_mut(), &mut v, &mut v),
            Bool::FALSE
        );
    }

    /// 端到端 ABI：小颗粒烟雾的光柱必须是蓝主导的线性 sRGB。
    #[test]
    fn inscatter_rgb_uniform_end_to_end() {
        let (mut r, mut g, mut b) = (0.0f64, 0.0f64, 0.0f64);
        let ok = tyndall_inscatter_rgb_uniform(
            10e-9, // 10 nm 水滴 → 瑞利区
            1.33, 0.0, 1e18, // 数密度
            1.0,  // phase
            1.0,  // light
            10.0, // ray_length
            64,   // steps
            15,   // samples
            &mut r, &mut g, &mut b,
        );
        assert_eq!(ok, Bool::TRUE);
        assert!(r >= 0.0 && g >= 0.0 && b >= 0.0);
        assert!(r + g + b > 0.0, "光柱必须有亮度");
        assert!(b > r, "细颗粒必须偏蓝：r={r} b={b}");

        // 非法参数必须失败且不写。
        let (mut r2, mut g2, mut b2) = (-1.0f64, -1.0f64, -1.0f64);
        assert_eq!(
            tyndall_inscatter_rgb_uniform(
                0.0, 1.33, 0.0, 1e18, 1.0, 1.0, 10.0, 64, 15, &mut r2, &mut g2, &mut b2
            ),
            Bool::FALSE
        );
        assert_eq!((r2, g2, b2), (-1.0, -1.0, -1.0), "失败时不得写入");
    }

    #[test]
    fn inscatter_rgb_uniform_rejects_null_outputs() {
        let mut v = 0.0f64;
        assert_eq!(
            tyndall_inscatter_rgb_uniform(
                1e-6,
                1.33,
                0.0,
                1e13,
                1.0,
                1.0,
                10.0,
                64,
                15,
                ptr::null_mut(),
                &mut v,
                &mut v
            ),
            Bool::FALSE
        );
    }
}
