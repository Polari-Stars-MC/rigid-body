//! 丁达尔效应数据层的测试。
//!
//! 这些测试钉的是**物理正确性**，不是"函数能跑"。关键的几条：
//!
//! * 米氏散射**波长依赖弱**（丁达尔是白光柱），瑞利是 1/λ⁴（蓝天）
//! * 相函数前倾（`g > 0` 时光柱看起来像一条亮线）
//! * Beer–Lambert 的单调性与光学厚度
//! * 沿视线积分"近光源更亮"
//! * 可见度在光学厚度 ≈ 1 附近最大

use mps_formula::tyndall::{
    beer_lambert_transmittance, henyey_greenstein_phase, inscatter_along_ray,
    mie_scattering_efficiency, optical_thickness, rayleigh_phase, single_scattering_albedo,
    size_parameter, tyndall_visibility,
};

const EPS: f64 = 1e-12;

#[test]
fn size_parameter_is_pi_d_over_lambda() {
    // d = λ 时 x = π。
    let x = size_parameter(1.0, 1.0).unwrap();
    assert!((x - std::f64::consts::PI).abs() < EPS);
    // 尺寸参数随粒径线性增长、随波长反比下降。
    assert!(size_parameter(2.0, 1.0).unwrap() > size_parameter(1.0, 1.0).unwrap());
    assert!(size_parameter(1.0, 2.0).unwrap() < size_parameter(1.0, 1.0).unwrap());
}

#[test]
fn size_parameter_rejects_degenerate_input() {
    assert!(size_parameter(0.0, 1.0).is_none());
    assert!(size_parameter(1.0, 0.0).is_none());
    assert!(size_parameter(-1.0, 1.0).is_none());
    assert!(size_parameter(f64::NAN, 1.0).is_none());
    assert!(size_parameter(1.0, f64::INFINITY).is_none());
}

/// 瑞利相函数前后对称 —— 这是它和丁达尔（前倾）的根本区别。
#[test]
fn rayleigh_phase_is_symmetric() {
    for c in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let fwd = rayleigh_phase(c).unwrap();
        let bwd = rayleigh_phase(-c).unwrap();
        assert!(
            (fwd - bwd).abs() < EPS,
            "rayleigh must be symmetric: p({c})={fwd} p(-{c})={bwd}"
        );
    }
    // 前向与侧向的比值是 (1+1)/1 = 2。
    assert!((rayleigh_phase(1.0).unwrap() / rayleigh_phase(0.0).unwrap() - 2.0).abs() < EPS);
}

#[test]
fn rayleigh_phase_rejects_out_of_range_cosine() {
    assert!(rayleigh_phase(1.5).is_none());
    assert!(rayleigh_phase(-1.5).is_none());
    assert!(rayleigh_phase(f64::NAN).is_none());
}

/// HG 相函数必须**前倾**：`g > 0` 时正前方远亮于背向。
#[test]
fn henyey_greenstein_is_forward_peaked_for_positive_g() {
    let g = 0.8;
    let forward = henyey_greenstein_phase(1.0, g).unwrap();
    let side = henyey_greenstein_phase(0.0, g).unwrap();
    let backward = henyey_greenstein_phase(-1.0, g).unwrap();

    assert!(
        forward > side,
        "g={g}: forward {forward} must exceed side {side}"
    );
    assert!(
        side > backward,
        "g={g}: side {side} must exceed backward {backward}"
    );
    // g = 0.8 时前向/背向应达到两个数量级左右（这是丁达尔"亮线"的来源）。
    assert!(
        forward / backward > 100.0,
        "g={g}: forward/backward = {} should be >> 1",
        forward / backward
    );
}

/// `g = 0` 退化为各向同性（所有方向相同）。
#[test]
fn henyey_greenstein_degenerates_to_isotropic_at_zero_g() {
    for c in [-1.0, -0.5, 0.0, 0.5, 1.0] {
        let p = henyey_greenstein_phase(c, 0.0).unwrap();
        assert!(
            (p - 1.0).abs() < EPS,
            "g=0 must be isotropic, got {p} at {c}"
        );
    }
}

#[test]
fn henyey_greenstein_rejects_invalid_g() {
    // |g| >= 1 是奇异/无意义的。
    assert!(henyey_greenstein_phase(0.0, 1.0).is_none());
    assert!(henyey_greenstein_phase(0.0, -1.0).is_none());
    assert!(henyey_greenstein_phase(0.0, 1.5).is_none());
    assert!(henyey_greenstein_phase(2.0, 0.5).is_none());
}

/// **丁达尔的核心物理断言**：米氏散射的波长依赖远弱于瑞利的 1/λ⁴。
///
/// 瑞利让光柱发蓝（蓝天效应），米氏让光柱接近白色（真正的丁达尔）。
#[test]
fn mie_is_nearly_wavelength_independent_while_rayleigh_is_not() {
    // 在米氏区（x ≈ 5）取两个波长，效率之比应接近 1。
    let q1 = mie_scattering_efficiency(size_parameter(5.0, 1.0).unwrap(), 1.33).unwrap();
    let q2 = mie_scattering_efficiency(size_parameter(5.0, 2.0).unwrap(), 1.33).unwrap();
    let ratio = q1 / q2;
    assert!(
        (0.5..=2.0).contains(&ratio),
        "mie efficiency should be roughly wavelength-flat, got ratio {ratio}"
    );

    // 对照：瑞利区（x ≈ 0.1）波长翻倍时效率应掉 2⁴ = 16 倍。
    let r1 = mie_scattering_efficiency(size_parameter(0.05, 1.0).unwrap(), 1.33).unwrap();
    let r2 = mie_scattering_efficiency(size_parameter(0.05, 2.0).unwrap(), 1.33).unwrap();
    let r_ratio = r1 / r2;
    assert!(
        (r_ratio - 16.0).abs() < 1.0,
        "rayleigh regime must scale as 1/lambda^4 (16x), got {r_ratio}"
    );
}

#[test]
fn mie_efficiency_approaches_two_for_large_particles() {
    // 消光佯谬：x >> 1 时 Q_ext -> 2。我们的散射效率也趋近常数。
    let q_small = mie_scattering_efficiency(25.0, 1.33).unwrap();
    let q_large = mie_scattering_efficiency(250.0, 1.33).unwrap();
    assert!(
        (q_small - q_large).abs() < 0.1,
        "large-particle efficiency should plateau, got {q_small} vs {q_large}"
    );
    assert!(q_large > 0.0 && q_large <= 2.1, "got {q_large}");
}

#[test]
fn mie_efficiency_rejects_degenerate_input() {
    assert!(mie_scattering_efficiency(0.0, 1.33).is_none());
    assert!(mie_scattering_efficiency(-1.0, 1.33).is_none());
    assert!(mie_scattering_efficiency(1.0, 0.0).is_none());
    assert!(mie_scattering_efficiency(f64::NAN, 1.33).is_none());
}

#[test]
fn beer_lambert_is_monotonic_and_bounded() {
    // 无介质时完全透射。
    assert!((beer_lambert_transmittance(1.0, 0.0, 100.0).unwrap() - 1.0).abs() < EPS);
    // 光学厚度为 0 时也是 1。
    assert!((beer_lambert_transmittance(0.0, 1.0, 100.0).unwrap() - 1.0).abs() < EPS);
    // 单调递减。
    let t1 = beer_lambert_transmittance(1e-20, 1e18, 1.0).unwrap();
    let t2 = beer_lambert_transmittance(1e-20, 1e18, 2.0).unwrap();
    assert!(t1 > t2, "transmittance must decrease with path length");
    // 落在 (0, 1]。
    assert!(t1 > 0.0 && t1 <= 1.0);
    assert!(t2 > 0.0 && t2 <= 1.0);
}

#[test]
fn optical_thickness_matches_the_transmittance_exponent() {
    let (sigma, n, l) = (2.5e-13, 1e14, 3.0);
    let tau = optical_thickness(sigma, n, l).unwrap();
    let t = beer_lambert_transmittance(sigma, n, l).unwrap();
    assert!(
        (t - (-tau).exp()).abs() < 1e-15,
        "T must equal exp(-tau), got {t} vs {}",
        (-tau).exp()
    );
}

#[test]
fn beer_lambert_rejects_negative_and_non_finite() {
    assert!(beer_lambert_transmittance(-1.0, 1.0, 1.0).is_none());
    assert!(beer_lambert_transmittance(1.0, -1.0, 1.0).is_none());
    assert!(beer_lambert_transmittance(1.0, 1.0, -1.0).is_none());
    assert!(beer_lambert_transmittance(f64::NAN, 1.0, 1.0).is_none());
}

#[test]
fn single_scattering_albedo_bounds() {
    // 纯散射介质的反照率为 1（光柱最亮）。
    assert!((single_scattering_albedo(1.0, 1.0).unwrap() - 1.0).abs() < EPS);
    // 一半吸收。
    assert!((single_scattering_albedo(0.5, 1.0).unwrap() - 0.5).abs() < EPS);
    // 散射不可能超过消光 —— 这是物理约束，必须拒绝而不是截断。
    assert!(single_scattering_albedo(2.0, 1.0).is_none());
    assert!(single_scattering_albedo(1.0, 0.0).is_none());
    assert!(single_scattering_albedo(-1.0, 1.0).is_none());
}

/// 沿视线积分：**靠近光源的段落贡献更大**（透射率衰减），所以总亮度随长度
/// 增长但边际递减。
#[test]
fn inscatter_grows_but_saturates_with_ray_length() {
    let (ss, se, n, ph, li) = (1e-12, 1e-12, 1e12, 1.0, 1.0);
    let short = inscatter_along_ray(ss, se, n, ph, li, 1.0, 64).unwrap();
    let long = inscatter_along_ray(ss, se, n, ph, li, 4.0, 64).unwrap();
    let longer = inscatter_along_ray(ss, se, n, ph, li, 8.0, 64).unwrap();

    assert!(short > 0.0);
    assert!(long > short, "longer path must scatter more");
    assert!(longer > long);
    // 边际递减：从 4m 到 8m 的增量应小于从 1m 到 4m 的增量（按比例看）。
    let d1 = long - short;
    let d2 = longer - long;
    assert!(d2 / d1 < 2.0, "increment must saturate: {d1} then {d2}");
}

#[test]
fn inscatter_is_zero_without_medium() {
    // 无散射、无数密度、零长度，都应为 0。
    assert_eq!(
        inscatter_along_ray(0.0, 1e-12, 1e12, 1.0, 1.0, 10.0, 16).unwrap(),
        0.0
    );
    assert_eq!(
        inscatter_along_ray(1e-12, 1e-12, 0.0, 1.0, 1.0, 10.0, 16).unwrap(),
        0.0
    );
    assert_eq!(
        inscatter_along_ray(1e-12, 1e-12, 1e12, 1.0, 1.0, 0.0, 16).unwrap(),
        0.0
    );
}

#[test]
fn inscatter_scales_linearly_with_phase_and_light() {
    let base = inscatter_along_ray(1e-12, 1e-12, 1e12, 1.0, 1.0, 5.0, 32).unwrap();
    let double_phase = inscatter_along_ray(1e-12, 1e-12, 1e12, 2.0, 1.0, 5.0, 32).unwrap();
    let double_light = inscatter_along_ray(1e-12, 1e-12, 1e12, 1.0, 2.0, 5.0, 32).unwrap();

    assert!((double_phase - 2.0 * base).abs() / base < 1e-9);
    assert!((double_light - 2.0 * base).abs() / base < 1e-9);
}

#[test]
fn inscatter_converges_as_steps_increase() {
    let coarse = inscatter_along_ray(1e-12, 1e-12, 1e12, 1.0, 1.0, 10.0, 16).unwrap();
    let fine = inscatter_along_ray(1e-12, 1e-12, 1e12, 1.0, 1.0, 10.0, 512).unwrap();
    let finer = inscatter_along_ray(1e-12, 1e-12, 1e12, 1.0, 1.0, 10.0, 4096).unwrap();

    // 中点法二阶收敛：细分后应逐步逼近。
    assert!((fine - coarse).abs() / fine < 0.05);
    assert!((finer - fine).abs() / finer < 0.01);
}

#[test]
fn inscatter_rejects_invalid_arguments() {
    // steps = 0
    assert!(inscatter_along_ray(1e-12, 1e-12, 1e12, 1.0, 1.0, 1.0, 0).is_none());
    // 散射超过消光
    assert!(inscatter_along_ray(2e-12, 1e-12, 1e12, 1.0, 1.0, 1.0, 8).is_none());
    // 负参数
    assert!(inscatter_along_ray(-1e-12, 1e-12, 1e12, 1.0, 1.0, 1.0, 8).is_none());
    assert!(inscatter_along_ray(1e-12, 1e-12, 1e12, -1.0, 1.0, 1.0, 8).is_none());
    // 非有限
    assert!(inscatter_along_ray(f64::NAN, 1e-12, 1e12, 1.0, 1.0, 1.0, 8).is_none());
}

/// 可见度在**光学厚度 ≈ 1**附近最大 —— 太稀看不到（散射不够），太浓也看不到
/// （散射光出不去）。
///
/// 这一项曾经写错过：最初只用 `ω·(1-T)·front_bias`，结果 τ=20 时可见度反而
/// 接近 1.0（因为 `1-T → 1`），与"浓到看不清的烟雾里根本看不到光柱"矛盾。
/// 加上的 `T_out = sqrt(T_in)`（出射路径衰减）修正了它。
#[test]
fn visibility_peaks_at_intermediate_optical_thickness() {
    let (ss, se, g) = (1e-12, 1e-12, 0.8);
    let l = 1.0;
    // 使 tau = se * n * l，所以 n_opt 对应 tau = 1。
    let n_opt = 1.0 / (se * l);

    let thin = tyndall_visibility(ss, se, n_opt * 0.05, l, g).unwrap();
    let optimal = tyndall_visibility(ss, se, n_opt, l, g).unwrap();
    let thick = tyndall_visibility(ss, se, n_opt * 20.0, l, g).unwrap();

    assert!(
        optimal > thin,
        "too thin is less visible: optimal {optimal} vs thin {thin}"
    );
    assert!(
        optimal > thick,
        "too thick must be less visible: optimal {optimal} vs thick {thick}"
    );
    // τ = 20 时应几乎不可见 —— 出射光全被挡住了。
    assert!(
        thick < 0.01,
        "tau=20 should be nearly invisible, got {thick}"
    );
    // 落在 [0, 1]。
    for v in [thin, optimal, thick] {
        assert!((0.0..=1.0).contains(&v), "visibility out of range: {v}");
    }
}

/// 前倾程度越高（g 越大），同样条件下可见度越高。
#[test]
fn visibility_grows_with_forward_bias() {
    let (ss, se, n, l) = (1e-12, 1e-12, 1e12, 1.0);
    let v_iso = tyndall_visibility(ss, se, n, l, 0.0).unwrap();
    let v_mid = tyndall_visibility(ss, se, n, l, 0.5).unwrap();
    let v_high = tyndall_visibility(ss, se, n, l, 0.9).unwrap();
    assert!(v_mid > v_iso, "{v_mid} should exceed {v_iso}");
    assert!(v_high > v_mid, "{v_high} should exceed {v_mid}");
}

#[test]
fn visibility_rejects_invalid_albedo_or_g() {
    // 散射 > 消光
    assert!(tyndall_visibility(2e-12, 1e-12, 1e12, 1.0, 0.5).is_none());
    // 非法 g
    assert!(tyndall_visibility(1e-12, 1e-12, 1e12, 1.0, 1.0).is_none());
    assert!(tyndall_visibility(1e-12, 1e-12, 1e12, 1.0, f64::NAN).is_none());
    // 非法消光
    assert!(tyndall_visibility(1e-12, 0.0, 1e12, 1.0, 0.5).is_none());
}

// ── C ABI 层（emag.rs 的包装）────────────────────────────────────────────────

use mps_core::rapier::emag::{
    tyndall_beer_lambert_transmittance, tyndall_henyey_greenstein_phase,
    tyndall_inscatter_along_ray, tyndall_mie_scattering_efficiency, tyndall_optical_thickness,
    tyndall_rayleigh_phase, tyndall_single_scattering_albedo, tyndall_size_parameter,
    tyndall_visibility_score,
};
use mps_core::rapier::error::{ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, last_error_code};
use mps_core::rapier::ffi::Bool;

#[test]
fn abi_writes_the_scalar_and_returns_true() {
    let mut out = 0.0f64;
    assert_eq!(tyndall_size_parameter(1.0, 1.0, &mut out), Bool::TRUE);
    assert!((out - std::f64::consts::PI).abs() < 1e-12);
    assert_eq!(last_error_code(), 0);
}

/// 失败时**绝不动 out**，且错误码是 `ERR_INVALID_ARGUMENT`。
#[test]
fn abi_never_writes_out_on_failure() {
    let sentinel = -999.0f64;
    let mut out = sentinel;

    // 非法尺寸参数。
    assert_eq!(tyndall_size_parameter(0.0, 1.0, &mut out), Bool::FALSE);
    assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    assert_eq!(out, sentinel, "failed call must not touch out");

    // 非法 cos_theta。
    assert_eq!(tyndall_rayleigh_phase(2.0, &mut out), Bool::FALSE);
    assert_eq!(out, sentinel);

    // 非法 g。
    assert_eq!(
        tyndall_henyey_greenstein_phase(0.0, 1.0, &mut out),
        Bool::FALSE
    );
    assert_eq!(out, sentinel);

    // 散射 > 消光。
    assert_eq!(
        tyndall_single_scattering_albedo(2.0, 1.0, &mut out),
        Bool::FALSE
    );
    assert_eq!(out, sentinel);
}

#[test]
fn abi_rejects_null_out() {
    assert_eq!(
        tyndall_size_parameter(1.0, 1.0, std::ptr::null_mut()),
        Bool::FALSE
    );
    assert_eq!(last_error_code(), ERR_NULL_POINTER);

    assert_eq!(
        tyndall_visibility_score(1.0, 1.0, 1e12, 1.0, 0.5, std::ptr::null_mut()),
        Bool::FALSE
    );
    assert_eq!(last_error_code(), ERR_NULL_POINTER);
}

/// `steps = 0` 必须被拒绝（不是静默返回 0）。
#[test]
fn abi_rejects_zero_steps() {
    let mut out = 0.0f64;
    assert_eq!(
        tyndall_inscatter_along_ray(1e-12, 1e-12, 1e12, 1.0, 1.0, 1.0, 0, &mut out),
        Bool::FALSE
    );
    assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
}

/// 端到端：从颗粒直径与波长算出可见度，走一遍完整的 ABI 链。
#[test]
fn abi_end_to_end_visibility_pipeline() {
    // 1 µm 水滴，550 nm 绿光 → 米氏区。
    let mut x = 0.0f64;
    assert_eq!(tyndall_size_parameter(1e-6, 550e-9, &mut x), Bool::TRUE);
    assert!((x - 5.712).abs() < 0.01, "size parameter = {x}");

    let mut q = 0.0f64;
    assert_eq!(
        tyndall_mie_scattering_efficiency(x, 1.33, &mut q),
        Bool::TRUE
    );
    assert!(q > 0.0, "mie efficiency must be positive, got {q}");

    // 相函数：正前方比侧向亮很多。
    let mut p_fwd = 0.0f64;
    let mut p_side = 0.0f64;
    assert_eq!(
        tyndall_henyey_greenstein_phase(1.0, 0.8, &mut p_fwd),
        Bool::TRUE
    );
    assert_eq!(
        tyndall_henyey_greenstein_phase(0.0, 0.8, &mut p_side),
        Bool::TRUE
    );
    assert!(p_fwd > p_side * 10.0, "forward {p_fwd} vs side {p_side}");

    // 透射率与光学厚度自洽。
    let (sigma, n, l) = (1e-12, 1e12, 1.0);
    let mut tau = 0.0f64;
    let mut t = 0.0f64;
    assert_eq!(tyndall_optical_thickness(sigma, n, l, &mut tau), Bool::TRUE);
    assert_eq!(
        tyndall_beer_lambert_transmittance(sigma, n, l, &mut t),
        Bool::TRUE
    );
    assert!((t - (-tau).exp()).abs() < 1e-15);

    // 最终可见度落在 [0, 1]。
    let mut vis = 0.0f64;
    assert_eq!(
        tyndall_visibility_score(sigma, sigma, n, l, 0.8, &mut vis),
        Bool::TRUE
    );
    assert!((0.0..=1.0).contains(&vis), "visibility = {vis}");
    assert!(vis > 0.0, "a light beam should be visible here, got {vis}");
}
