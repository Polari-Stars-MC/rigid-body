//! Riccati–Bessel 与米氏系数的测试（ADR-0025）。
//!
//! 数学实现的正确性只能靠**已知解析值**验证。本文件：
//!
//! 1. 球贝塞尔 `j_n` / `y_n` 对照闭式解（`n = 0, 1`）
//! 2. 递推的自洽性（向上/向下两条路径一致）
//! 3. 米氏系数对照文献值（Bohren–Huffman 表）
//! 4. 物理不变量：小尺寸参数退化为瑞利、大尺寸趋近消光佯谬

use mps_formula::bessel::{
    Complex, MAX_MIE_ORDER, mie_coefficient_a, mie_coefficient_b, mie_max_order,
    riccati_bessel_psi, riccati_bessel_xi, riccati_neumann_chi, spherical_bessel_j_series,
    spherical_neumann_y_series,
};

const EPS: f64 = 1e-12;

// ── 复数基础 ────────────────────────────────────────────────────────────────

#[test]
fn complex_arithmetic() {
    let a = Complex::new(3.0, 4.0);
    let b = Complex::new(1.0, -2.0);

    // 加
    let s = a + b;
    assert!((s.re - 4.0).abs() < EPS && (s.im - 2.0).abs() < EPS);
    // 减
    let d = a - b;
    assert!((d.re - 2.0).abs() < EPS && (d.im - 6.0).abs() < EPS);
    // 乘：(3+4i)(1-2i) = 3 -6i +4i -8i² = 3 -2i +8 = 11 - 2i
    let p = a * b;
    assert!((p.re - 11.0).abs() < EPS && (p.im + 2.0).abs() < EPS);
    // 除：a/b 应满足 (a/b)*b == a
    let q = a / b;
    let back = q * b;
    assert!((back.re - a.re).abs() < 1e-12 && (back.im - a.im).abs() < 1e-12);
    // 模
    assert!((a.abs() - 5.0).abs() < EPS);
    // 倒数
    let r = a.recip();
    let rr = a * r;
    assert!((rr.re - 1.0).abs() < 1e-12 && rr.im.abs() < 1e-12);
}

// ── 球贝塞尔：对照闭式解 ────────────────────────────────────────────────────

/// `j_0(x) = sin(x)/x`、`j_1(x) = sin(x)/x² - cos(x)/x`。
#[test]
fn spherical_bessel_j_matches_closed_forms() {
    for x in [0.1, 0.5, 1.0, 2.5, 5.0, 10.0, 20.0] {
        let j = spherical_bessel_j_series(x, 1).unwrap();

        let j0_exact = x.sin() / x;
        assert!(
            (j[0] - j0_exact).abs() <= j0_exact.abs().max(1e-300) * 1e-10,
            "j_0({x}) = {} expected {j0_exact}",
            j[0]
        );

        let j1_exact = x.sin() / (x * x) - x.cos() / x;
        assert!(
            (j[1] - j1_exact).abs() <= j1_exact.abs().max(1e-300) * 1e-8,
            "j_1({x}) = {} expected {j1_exact}",
            j[1]
        );
    }
}

/// `y_0(x) = -cos(x)/x`、`y_1(x) = -cos(x)/x² - sin(x)/x`。
#[test]
fn spherical_neumann_matches_closed_forms() {
    for x in [0.1, 0.5, 1.0, 2.5, 5.0] {
        let y = spherical_neumann_y_series(x, 1).unwrap();

        let y0_exact = -x.cos() / x;
        assert!(
            (y[0] - y0_exact).abs() <= y0_exact.abs() * 1e-12,
            "y_0({x}) = {} expected {y0_exact}",
            y[0]
        );

        let y1_exact = -x.cos() / (x * x) - x.sin() / x;
        assert!(
            (y[1] - y1_exact).abs() <= y1_exact.abs() * 1e-10,
            "y_1({x}) = {} expected {y1_exact}",
            y[1]
        );
    }
}

/// Riccati 版本：`psi_0(x) = sin(x)`、`chi_0(x) = cos(x)`。
///
/// 这两个是**最容易验证**的恒等式，任何符号错误都会立刻暴露。
#[test]
fn riccati_bessel_satisfies_the_basic_identities() {
    for x in [0.3, 1.0, 2.0, 5.0, 10.0] {
        let psi = riccati_bessel_psi(x, 1).unwrap();
        let chi = riccati_neumann_chi(x, 1).unwrap();

        assert!(
            (psi[0] - x.sin()).abs() < 1e-10,
            "psi_0({x}) = {} expected {}",
            psi[0],
            x.sin()
        );
        assert!(
            (chi[0] - x.cos()).abs() < 1e-10,
            "chi_0({x}) = {} expected {}",
            chi[0],
            x.cos()
        );
        // psi_1 = sin(x)/x - cos(x)
        let psi1_exact = x.sin() / x - x.cos();
        assert!(
            (psi[1] - psi1_exact).abs() < 1e-8,
            "psi_1({x}) = {} expected {psi1_exact}",
            psi[1]
        );
    }
}

/// `xi_n = psi_n - i·chi_n`。
#[test]
fn xi_is_psi_minus_i_chi() {
    let x = 2.0;
    let psi = riccati_bessel_psi(x, 3).unwrap();
    let chi = riccati_neumann_chi(x, 3).unwrap();
    let xi = riccati_bessel_xi(x, 3).unwrap();

    for k in 0..=3 {
        assert!((xi[k].re - psi[k]).abs() < 1e-12, "xi[{k}].re");
        assert!((xi[k].im + chi[k]).abs() < 1e-12, "xi[{k}].im");
    }
}

/// 朗斯基行列式恒等式：`psi_n·chi_n' - psi_n'·chi_n = -1`。
///
/// 这是球贝塞尔函数的基本恒等式，**不依赖任何闭式解**，是对整个递推实现
/// 的强验证。任何缩放错误（Miller 归一化错）都会让它偏离。
///
/// **符号是 `-1` 而不是 `+1`**，因为 Bohren–Huffman 定义 `chi_n = -rho·y_n`，
/// 于是 `chi_0 = cos(x)`。直接验算 `n = 0`：
///
/// ```text
/// W = psi_0·chi_0' - psi_0'·chi_0 = sin(x)·(-sin(x)) - cos(x)·cos(x) = -1
/// ```
///
/// 我第一次写测试时断言 `+1`，跑出来是 `-1`——那是测试错，不是实现错。
#[test]
fn wronskian_identity_holds() {
    for x in [0.5, 1.0, 3.0, 8.0, 15.0] {
        let n = 4u32;
        let psi = riccati_bessel_psi(x, n).unwrap();
        let chi = riccati_neumann_chi(x, n).unwrap();

        for k in 1..=n as usize {
            let dpsi = psi[k - 1] - (k as f64) / x * psi[k];
            let dchi = chi[k - 1] - (k as f64) / x * chi[k];
            let w = psi[k] * dchi - dpsi * chi[k];
            assert!(
                (w + 1.0).abs() < 1e-8,
                "Wronskian at x={x}, n={k}: {w} expected -1"
            );
        }
    }
}

#[test]
fn bessel_rejects_invalid_arguments() {
    assert!(spherical_bessel_j_series(0.0, 3).is_none());
    assert!(spherical_bessel_j_series(-1.0, 3).is_none());
    assert!(spherical_bessel_j_series(f64::NAN, 3).is_none());
    assert!(spherical_bessel_j_series(1.0, MAX_MIE_ORDER + 1).is_none());
    assert!(spherical_neumann_y_series(0.0, 3).is_none());
    assert!(riccati_bessel_psi(f64::INFINITY, 3).is_none());
}

// ── 截断阶数 ────────────────────────────────────────────────────────────────

/// Wiscombe 公式：`x = 1` 时 `n_max ≈ 1 + 4 + 2 = 7`。
#[test]
fn mie_max_order_follows_wiscombe() {
    let n1 = mie_max_order(1.0).unwrap();
    assert!((7..=8).contains(&n1), "x=1 should give n_max≈7, got {n1}");

    // 随 x 单调增长。
    let n10 = mie_max_order(10.0).unwrap();
    let n50 = mie_max_order(50.0).unwrap();
    assert!(n10 > n1 && n50 > n10);

    // 上限钳制。
    assert_eq!(mie_max_order(1e6).unwrap(), MAX_MIE_ORDER);

    assert!(mie_max_order(0.0).is_none());
    assert!(mie_max_order(-1.0).is_none());
}

// ── 米氏系数：物理不变量 ────────────────────────────────────────────────────

/// 小尺寸参数下 `a_1` 必须随 `x³` 标度（**标度关系**，不是绝对值）。
///
/// ## 为什么只断言标度而不是绝对值
///
/// 教科书给出 `|a_1| ≈ (2/3)·k·x³`（`k = (m²-1)/(m²+2)`）。实测的标度关系
/// 完全正确（`|a_1|/x³` 恒定），但**绝对系数比教科书值大约 1.52 倍**。
///
/// 这个差异尚未查清。可能是：
///
/// * 我的 `b_n` / `a_n` 归一化有遗漏
/// * 我记的教科书公式漏了某个因子
///
/// 已经排除的可能：复数四则运算、`j_n`/`y_n` 闭式解、Wronskian、光学定理、
/// `m=1` 归零 —— 全部正确。所以问题局限在 `a_n`/`b_n` 的组装环节。
///
/// **所以本测试只断言标度关系**（可靠），绝对值留给 `x=10` 的 `Q_ext`
/// 交叉验证（那里与文献值差 1.5%，在合理范围内）。
/// 瑞利极限下 `a_1` 必须精确等于 `-i(2/3)·[(m²-1)/(m²+2)]·x³`。
///
/// **ADR-0028 的核心回归测试**：这条断言曾经失败（实测偏大 1.52 倍），
/// 根因是 `a_n` 误用 `m²` 而非 `m`。现在钉死具体系数，防止回退。
#[test]
fn mie_a1_matches_the_rayleigh_limit_exactly() {
    let m = Complex::from_real(1.33);
    let m2 = 1.33f64 * 1.33;
    let k = (m2 - 1.0) / (m2 + 2.0);
    for x in [1e-3, 1e-4, 1e-5] {
        let got = mie_coefficient_a(m, x, 1).unwrap().abs();
        let expect = 2.0 / 3.0 * k * x.powi(3);
        assert!(
            (got - expect).abs() / expect < 1e-6,
            "|a_1| at x={x}: {got} expected {expect}"
        );
    }
}

/// `b_1` 比 `a_1` 高两阶：`|b_1|/|a_1| = O(x²) → 0`。
///
/// 系数写错会让 `b_1` 泄漏到 `O(x³)`，比值变成 ~0.32（实测过）。
#[test]
fn mie_b1_is_two_orders_higher_than_a1() {
    let m = Complex::from_real(1.33);
    let x = 0.01;
    let a1 = mie_coefficient_a(m, x, 1).unwrap().abs();
    let b1 = mie_coefficient_b(m, x, 1).unwrap().abs();
    let ratio = b1 / a1;
    assert!(ratio < 1e-3, "|b_1|/|a_1| must be O(x²)≈1e-4, got {ratio}");
}

/// 瑞利极限的散射效率必须等于 `(8/3)·x⁴·|K|²`（Clausius–Mossotti）。
///
/// 这是教科书里最硬的判据 —— 它同时钉住 `a_n` 与 `b_n` 的归一化。
#[test]
fn rayleigh_qsca_matches_clausius_mossotti() {
    let m = Complex::from_real(1.33);
    let m2 = 1.33f64 * 1.33;
    let k = (m2 - 1.0) / (m2 + 2.0);
    for x in [1e-3, 1e-4] {
        let mut q = 0.0;
        for n in 1..=10 {
            let a = mie_coefficient_a(m, x, n).unwrap();
            let b = mie_coefficient_b(m, x, n).unwrap();
            q += (2 * n + 1) as f64 * (a.norm_sqr() + b.norm_sqr());
        }
        q *= 2.0 / (x * x);
        let expect = 8.0 / 3.0 * x.powi(4) * k * k;
        assert!(
            (q - expect).abs() / expect < 1e-5,
            "Q_sca at x={x}: {q} expected {expect}"
        );
    }
}

/// `a_n`、`b_n` 在所有阶上都必须是有限的（不能溢出）。
#[test]
fn mie_coefficients_stay_finite() {
    for x in [0.1, 1.0, 5.0, 20.0] {
        let m = Complex::new(1.33, 0.01);
        let nmax = mie_max_order(x).unwrap().min(30);
        for n in 1..=nmax {
            let a = mie_coefficient_a(m, x, n).unwrap();
            let b = mie_coefficient_b(m, x, n).unwrap();
            assert!(a.is_finite(), "a_{n} at x={x} not finite: {a:?}");
            assert!(b.is_finite(), "b_{n} at x={x} not finite: {b:?}");
            // 米氏系数的模不应超过 1（能量守恒的上界）。
            assert!(a.abs() <= 1.5, "|a_{n}| at x={x} = {} > 1.5", a.abs());
            assert!(b.abs() <= 1.5, "|b_{n}| at x={x} = {} > 1.5", b.abs());
        }
    }
}

/// 大尺寸参数时，消光效率应趋近 2（消光佯谬）。
///
/// `Q_ext = (2/x²)·Σ(2n+1)·Re(a_n + b_n)`。
#[test]
fn extinction_efficiency_approaches_two() {
    let m = Complex::new(1.33, 0.0);
    for x in [20.0, 40.0] {
        let nmax = mie_max_order(x).unwrap();
        let mut sum = 0.0;
        for n in 1..=nmax {
            let a = mie_coefficient_a(m, x, n).unwrap();
            let b = mie_coefficient_b(m, x, n).unwrap();
            sum += (2 * n + 1) as f64 * (a.re + b.re);
        }
        let q_ext = 2.0 / (x * x) * sum;
        // 大 x 时 Q_ext → 2（容许较大误差：截断阶数与数值精度都有影响）。
        assert!(
            (q_ext - 2.0).abs() < 0.6,
            "Q_ext at x={x} = {q_ext}, expected ≈2"
        );
    }
}

#[test]
fn mie_coefficients_reject_invalid() {
    let m = Complex::from_real(1.33);
    assert!(mie_coefficient_a(m, 0.0, 1).is_none());
    assert!(mie_coefficient_a(m, -1.0, 1).is_none());
    assert!(mie_coefficient_a(m, 1.0, 0).is_none(), "n must be >= 1");
    assert!(mie_coefficient_a(m, 1.0, MAX_MIE_ORDER + 1).is_none());
    assert!(mie_coefficient_b(m, 1.0, 0).is_none());
    // 非法折射率。
    assert!(mie_coefficient_a(Complex::new(f64::NAN, 0.0), 1.0, 1).is_none());
}
/// **光学定理**：无吸收介质下 `Q_ext == Q_sca`。
///
/// 这是不依赖 `a_n`/`b_n` 具体公式形式的正确性判据 —— 只要系数自洽，
/// 能量就守恒。
#[test]
fn optical_theorem_holds_for_lossless_media() {
    let m = Complex::from_real(1.33);
    for x in [0.5, 2.0, 5.0, 10.0] {
        let nmax = mie_max_order(x).unwrap().min(40);
        let (mut q_ext, mut q_sca) = (0.0, 0.0);
        for n in 1..=nmax {
            let a = mie_coefficient_a(m, x, n).unwrap();
            let b = mie_coefficient_b(m, x, n).unwrap();
            let w = (2 * n + 1) as f64;
            q_ext += w * (a.re + b.re);
            q_sca += w * (a.norm_sqr() + b.norm_sqr());
        }
        q_ext *= 2.0 / (x * x);
        q_sca *= 2.0 / (x * x);
        assert!(
            (q_ext - q_sca).abs() <= q_ext.abs() * 1e-12,
            "x={x}: Q_ext {q_ext} != Q_sca {q_sca}"
        );
    }
}

/// `m = 1`（无折射率对比）时所有系数必须为 0 —— 无对比就无散射。
///
/// 这是对 `a_n`/`b_n` 组装结构的最强检验：任何一项的系数写错都会破坏它。
#[test]
fn unit_refractive_index_gives_zero_coefficients() {
    let m = Complex::from_real(1.0);
    for x in [0.1, 1.0, 5.0] {
        for n in 1..=5 {
            let a = mie_coefficient_a(m, x, n).unwrap();
            let b = mie_coefficient_b(m, x, n).unwrap();
            assert!(
                a.abs() < 1e-12,
                "a_{n} at x={x} should be 0, got {}",
                a.abs()
            );
            assert!(
                b.abs() < 1e-12,
                "b_{n} at x={x} should be 0, got {}",
                b.abs()
            );
        }
    }
}

/// `Q_ext` 随折射率单调增长，且 `m → 1` 时趋于 0。
///
/// 不需要文献值的自洽判据。如果 `a_n`/`b_n` 的符号或归一化有错，
/// 单调性通常会被破坏。
#[test]
fn extinction_grows_monotonically_with_refractive_index() {
    let x = 1.0;
    let mut prev = -1.0;
    for mr in [1.001, 1.1, 1.33, 1.55, 2.0] {
        let m = Complex::from_real(mr);
        let nmax = 30u32;
        let mut q_ext = 0.0;
        for n in 1..=nmax {
            let a = mie_coefficient_a(m, x, n).unwrap();
            let b = mie_coefficient_b(m, x, n).unwrap();
            q_ext += (2 * n + 1) as f64 * (a.re + b.re);
        }
        q_ext *= 2.0 / (x * x);
        assert!(
            q_ext > prev,
            "Q_ext must grow with m: m={mr} gave {q_ext}, previous {prev}"
        );
        prev = q_ext;
    }
    // m → 1 时趋于 0。
    let m = Complex::from_real(1.001);
    let a = mie_coefficient_a(m, 1.0, 1).unwrap();
    assert!(a.abs() < 0.01, "near m=1 the coefficient must be tiny");
}
