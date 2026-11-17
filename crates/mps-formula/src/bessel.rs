//! 最小复数类型与 Riccati–Bessel 函数。
//!
//! 为米氏散射（Mie scattering）的**精确解**提供数学基础。
//!
//! ## 为什么自己实现而不是引入 `num-complex`
//!
//! `mps-formula` 目前**零外部依赖**（只有 `mps-bindgen-macro` 这个本地 proc-macro）。
//! 这是刻意的：公式层要能在任何宿主里编译，包括没有 std 的特殊场景。
//!
//! 米氏散射需要的复数运算只有四种：加、减、乘、除。不需要 `sqrt`/`exp`/`log`/
//! 三角函数（那些要额外的级数实现）。所以 60 行足够，不值得为它开一个依赖。
//!
//! ## 数值稳定性
//!
//! 球贝塞尔函数 `j_n(x)` 的**向上**递推在 `n > x` 时发散（`j_n` 本身在 `n` 增大
//! 时急剧减小）。标准做法是**向下递推 + 归一化**（Miller 算法），本模块采用它。
//!
//! 米氏系数的计算用 Bohren–Huffman 的对数导数形式，避免 `psi_n` 溢出。
//!
//! 参考：Bohren & Huffman, *Absorption and Scattering of Light by Small
//! Particles*, Wiley 1983, §4.4–4.5。

/// 复数（`f64` 精度）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Complex {
    /// 实部。
    pub re: f64,
    /// 虚部。
    pub im: f64,
}

impl Complex {
    /// 零。
    pub const ZERO: Self = Self { re: 0.0, im: 0.0 };
    /// 一。
    pub const ONE: Self = Self { re: 1.0, im: 0.0 };

    /// 由实部与虚部构造。
    pub const fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    /// 纯实部。
    pub const fn from_real(re: f64) -> Self {
        Self { re, im: 0.0 }
    }

    /// 模的平方。
    pub fn norm_sqr(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    /// 模。
    pub fn abs(self) -> f64 {
        self.norm_sqr().sqrt()
    }

    /// 是否两个分量都有限。
    pub fn is_finite(self) -> bool {
        self.re.is_finite() && self.im.is_finite()
    }

    /// 倒数。
    pub fn recip(self) -> Self {
        let d = self.norm_sqr();
        Self {
            re: self.re / d,
            im: -self.im / d,
        }
    }
}

impl core::ops::Add for Complex {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self {
            re: self.re + o.re,
            im: self.im + o.im,
        }
    }
}

impl core::ops::Sub for Complex {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self {
            re: self.re - o.re,
            im: self.im - o.im,
        }
    }
}

impl core::ops::Mul for Complex {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        Self {
            re: self.re * o.re - self.im * o.im,
            im: self.re * o.im + self.im * o.re,
        }
    }
}

impl core::ops::Div for Complex {
    type Output = Self;
    fn div(self, o: Self) -> Self {
        let d = o.norm_sqr();
        Self {
            re: (self.re * o.re + self.im * o.im) / d,
            im: (self.im * o.re - self.re * o.im) / d,
        }
    }
}

impl core::ops::Neg for Complex {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            re: -self.re,
            im: -self.im,
        }
    }
}

/// 米氏计算的阶数上限（防止病态参数下无限循环）。
pub const MAX_MIE_ORDER: u32 = 2000;

/// 用 Miller 向下递推计算球贝塞尔函数 `j_n(x)`（`x > 0`，实数参数）。
///
/// 返回 `j_0..j_n` 的数组，长度 `n + 1`。
///
/// ## 算法
///
/// 向上递推 `j_{n+1} = (2n+1)/x·j_n - j_{n-1}` 在 `n > x` 时数值不稳定
/// （误差放大）。向下递推则稳定，但起点值未知 —— Miller 算法先任意取
/// `j_{N} = 0`、`j_{N-1} = 1` 向下递推到 `j_0`，再用解析的 `j_0 = sin(x)/x`
/// 归一化。
///
/// 起始阶 `N` 取 `max(n, ⌊x⌋) + 20`，多出的余量保证递推已经收敛。
///
/// # Returns
/// `Some(vec)`；`x <= 0`、`n > MAX_MIE_ORDER` 或非有限时 `None`。
pub fn spherical_bessel_j_series(x: f64, n: u32) -> Option<Vec<f64>> {
    if !x.is_finite() || x <= 0.0 || n > MAX_MIE_ORDER {
        return None;
    }
    let nmax = n as usize;
    // 递推起点必须越过转折点 n = x：j_n(x) 在 n < x 时振荡，n > x 时指数衰减。
    // Miller 算法只在衰减区起点有效，所以取 max(n + 20, ceil(x) + 30)。
    let start = (nmax + 20).max((x.ceil() as usize) + 30);

    let mut j = vec![0.0f64; start + 2];
    // 任意起点（Miller）：j[start] = 0, j[start-1] = 1e-30（避免全零）。
    j[start] = 0.0;
    j[start - 1] = 1.0e-30;

    // 向下递推：j_{k-1} = (2k+1)/x · j_k - j_{k+1}
    for k in (1..start).rev() {
        j[k - 1] = (2.0 * k as f64 + 1.0) / x * j[k] - j[k + 1];
    }

    // 用解析的 j_0 = sin(x)/x 归一化。
    let j0_exact = x.sin() / x;
    if j[0].abs() < 1e-300 {
        return None;
    }
    let scale = j0_exact / j[0];
    let out = j
        .iter()
        .take(nmax + 1)
        .map(|v| v * scale)
        .collect::<Vec<f64>>();
    Some(out)
}

/// 球诺伊曼函数 `y_n(x)`（第二类球贝塞尔），向上递推。
///
/// `y_0 = -cos(x)/x`、`y_1 = -cos(x)/x² - sin(x)/x`，然后
/// `y_{n+1} = (2n+1)/x·y_n - y_{n-1}`。
///
/// **向上递推对 `y_n` 是稳定的**（与 `j_n` 相反），因为 `y_n` 随 `n` 增大而增大。
pub fn spherical_neumann_y_series(x: f64, n: u32) -> Option<Vec<f64>> {
    if !x.is_finite() || x <= 0.0 || n > MAX_MIE_ORDER {
        return None;
    }
    let nmax = n as usize;
    let mut y = vec![0.0f64; nmax + 1];
    y[0] = -x.cos() / x;
    if nmax >= 1 {
        y[1] = -x.cos() / (x * x) - x.sin() / x;
        for k in 1..nmax {
            y[k + 1] = (2.0 * k as f64 + 1.0) / x * y[k] - y[k - 1];
        }
    }
    Some(y)
}

/// Riccati–Bessel 函数 `psi_n(x) = x·j_n(x)`。
pub fn riccati_bessel_psi(x: f64, n: u32) -> Option<Vec<f64>> {
    let j = spherical_bessel_j_series(x, n)?;
    Some(j.into_iter().map(|v| x * v).collect())
}

/// Riccati–Neumann 函数 `chi_n(x) = -x·y_n(x)`。
///
/// 注意符号：Bohren–Huffman 用的是 `chi_n = -x·y_n`（正的 `chi_0 = cos(x)`）。
pub fn riccati_neumann_chi(x: f64, n: u32) -> Option<Vec<f64>> {
    let y = spherical_neumann_y_series(x, n)?;
    Some(y.into_iter().map(|v| -x * v).collect())
}

/// Riccati–Bessel 函数 `xi_n(x) = psi_n(x) - i·chi_n(x)`（第一类球汉克尔乘以 x）。
///
/// `xi_n = x·h_n^(1)(x) = x·(j_n - i·y_n) = psi_n - i·chi_n`。
pub fn riccati_bessel_xi(x: f64, n: u32) -> Option<Vec<Complex>> {
    let psi = riccati_bessel_psi(x, n)?;
    let chi = riccati_neumann_chi(x, n)?;
    Some(
        psi.into_iter()
            .zip(chi)
            .map(|(p, c)| Complex::new(p, -c))
            .collect(),
    )
}

/// 米氏级数的截断阶数 `n_max`。
///
/// Wiscombe 的经验公式（Bohren–Huffman §4.5）：
///
/// ```text
/// x < 8:   n_max = x + 4x^(1/3) + 2
/// x >= 8:  n_max = x + 4.05·x^(1/3) + 2
/// ```
///
/// 加 1 并取整。上限 `MAX_MIE_ORDER`。
pub fn mie_max_order(size_parameter: f64) -> Option<u32> {
    if !size_parameter.is_finite() || size_parameter <= 0.0 {
        return None;
    }
    let x = size_parameter;
    let cbrt = x.cbrt();
    let raw = if x < 8.0 {
        x + 4.0 * cbrt + 2.0
    } else {
        x + 4.05 * cbrt + 2.0
    };
    Some((raw.ceil() as u32).clamp(1, MAX_MIE_ORDER))
}

/// 米氏参数 `z` 的**对数导数**序列 `D_n(z) = psi_n'(z)/psi_n(z)`（`n = 0..nmax`）。
///
/// ## 为什么这个函数是米氏计算的核心
///
/// 直接求 `psi_n(z)` 需要复参数球贝塞尔函数。幂级数在 `|z|` 大时精度急剧
/// 下降（相消），所以那条路走不远。
///
/// 但米氏系数其实只需要 `D_n`，它满足**一阶递推**：
///
/// ```text
/// D_{n-1}(z) = n/z - 1/(D_n(z) + n/z)
/// ```
///
/// 这个递推**向下**（`n` 从大到小）求解时对任何 `|z|` 都数值稳定。起始条件
/// 利用 `D_N ≈ 0` 当 `N ≫ |z|`。
///
/// 参考 Bohren & Huffman §4.5，式 (4.89)。
///
/// ## 起始阶的选取
///
/// `start = max(nmax, ⌈|z|⌉) + 25`。多出的 25 阶保证向下递推已经收敛；
/// `⌈|z|⌉` 保证起始阶确实远大于 `|z|`（否则 `D_N ≈ 0` 不成立）。
///
/// # Returns
/// `Some(vec)`（长度 `nmax + 1`）；`z` 为零/非有限时 `None`。
pub fn logarithmic_derivative_series(z: Complex, nmax: u32) -> Option<Vec<Complex>> {
    if !z.is_finite() || z.norm_sqr() <= 0.0 || nmax == 0 || nmax > MAX_MIE_ORDER {
        return None;
    }
    let n_use = nmax as usize;
    let start = (z.abs().ceil() as usize).max(n_use) + 25;

    let mut d = vec![Complex::ZERO; start + 1];
    // D_start ≈ 0（start ≫ |z|）。
    for k in (1..=start).rev() {
        let kz = Complex::from_real(k as f64) / z;
        let denom = d[k] + kz;
        if denom.norm_sqr() <= 0.0 {
            return None;
        }
        d[k - 1] = kz - Complex::ONE / denom;
    }
    d.truncate(n_use + 1);
    Some(d)
}

/// 米氏散射系数 `a_n`（复数）。
///
/// Bohren–Huffman 式 (4.88) 的对数导数形式：
///
/// ```text
///        (D_n(mx)/m + n/x)·psi_n(x) - psi_{n-1}(x)
/// a_n = ─────────────────────────────────────────────
///        (D_n(mx)/m + n/x)·xi_n(x)  - xi_{n-1}(x)
/// ```
///
/// `psi_n(x)` / `xi_n(x)` 是**实参数**的 Riccati–Bessel（用 Miller 递推，
/// 见 [`riccati_bessel_psi`] / [`riccati_bessel_xi`]），`D_n(mx)` 是复参数的
/// 对数导数（见 [`logarithmic_derivative_series`]）。
///
/// ## 系数是 `1/m`，不是 `m²`
///
/// 原始形式用 `m²·j_n(mx)`，但 `j_n(mx) = psi_n(mx)/(mx)`，所以
/// `m²·j_n(mx) = m·psi_n(mx)/x` —— 与分母共有的 `1/x` 约掉后只剩 `m`，
/// 而对数导数形式里是 `D_n/m`。
///
/// **写成 `m²` 会让瑞利极限偏大 1.52 倍**（`m = 1.33` 时）。见 ADR-0028。
///
/// # Parameters
/// * `m` 相对复折射率（颗粒/介质）
/// * `x` 尺寸参数 `πd/λ`
/// * `n` 阶数（`1..=n_max`）
pub fn mie_coefficient_a(m: Complex, x: f64, n: u32) -> Option<Complex> {
    if !m.is_finite() || !x.is_finite() || x <= 0.0 || n == 0 || n > MAX_MIE_ORDER {
        return None;
    }
    let mx = m * Complex::from_real(x);
    if !mx.is_finite() || mx.norm_sqr() <= 0.0 {
        return None;
    }
    let k = n as usize;
    let psi_x = riccati_bessel_psi(x, n)?;
    let xi_x = riccati_bessel_xi(x, n)?;
    let d = logarithmic_derivative_series(mx, n)?;

    let term = d[k] / m + Complex::from_real(n as f64 / x);
    let num = term * Complex::from_real(psi_x[k]) - Complex::from_real(psi_x[k - 1]);
    let den = term * xi_x[k] - xi_x[k - 1];
    if den.norm_sqr() <= 0.0 {
        return None;
    }
    Some(num / den)
}

/// 米氏散射系数 `b_n`（复数）。
///
/// ```text
///        (m·D_n(mx) + n/x)·psi_n(x) - psi_{n-1}(x)
/// b_n = ─────────────────────────────────────────────
///        (m·D_n(mx) + n/x)·xi_n(x)  - xi_{n-1}(x)
/// ```
///
/// 与 [`mie_coefficient_a`] 的唯一区别：`D_n/m` 换成 `m·D_n`。
///
/// **不是完全去掉 `m`** —— 去掉会让 `b_1` 从 `O(x⁵)` 错误地泄漏到 `O(x³)`，
/// 表现为 `|b_1|/|a_1| ≈ 0.32` 而非应有的 `~x²`（见 ADR-0028）。
pub fn mie_coefficient_b(m: Complex, x: f64, n: u32) -> Option<Complex> {
    if !m.is_finite() || !x.is_finite() || x <= 0.0 || n == 0 || n > MAX_MIE_ORDER {
        return None;
    }
    let mx = m * Complex::from_real(x);
    if !mx.is_finite() || mx.norm_sqr() <= 0.0 {
        return None;
    }
    let k = n as usize;
    let psi_x = riccati_bessel_psi(x, n)?;
    let xi_x = riccati_bessel_xi(x, n)?;
    let d = logarithmic_derivative_series(mx, n)?;

    let term = m * d[k] + Complex::from_real(n as f64 / x);
    let num = term * Complex::from_real(psi_x[k]) - Complex::from_real(psi_x[k - 1]);
    let den = term * xi_x[k] - xi_x[k - 1];
    if den.norm_sqr() <= 0.0 {
        return None;
    }
    Some(num / den)
}

/// 一次性算出 `1..=nmax` 全部阶的米氏系数 `a_n` / `b_n`。
///
/// ## 为什么需要批量版
///
/// [`mie_coefficient_a`] / [`mie_coefficient_b`] 每次调用都重新算
/// `psi_n(x)` / `xi_n(x)` 的整个 Miller 递推 —— 那是 `O(start)` 的开销。
/// 循环调用 `n = 1..=nmax` 就变成 `O(nmax²)`。
///
/// 本函数把三个级数（`psi`、`xi`、`D_n`）各算**一次**，然后只做 `O(nmax)`
/// 的组装。总复杂度 `O(nmax)`。
///
/// 对 `x = 10`（`nmax = 21`）差别很小；对 `x = 1000`（`nmax ≈ 1040`）
/// 是 **1000 倍**的差距 —— 这是放开 `x > 200` 上限的前提。
///
/// # Returns
/// `Some((a, b))`，两个 `Vec` 的长度都是 `nmax + 1`，索引 `0` 元素未使用（填零）。
pub fn mie_coefficients_series(
    m: Complex,
    x: f64,
    nmax: u32,
) -> Option<(Vec<Complex>, Vec<Complex>)> {
    if !m.is_finite() || !x.is_finite() || x <= 0.0 || nmax == 0 || nmax > MAX_MIE_ORDER {
        return None;
    }
    let mx = m * Complex::from_real(x);
    if !mx.is_finite() || mx.norm_sqr() <= 0.0 {
        return None;
    }
    // 三个级数各算一次。
    let psi_x = riccati_bessel_psi(x, nmax)?;
    let xi_x = riccati_bessel_xi(x, nmax)?;
    let d = logarithmic_derivative_series(mx, nmax)?;

    let n_use = nmax as usize;
    let mut a = vec![Complex::ZERO; n_use + 1];
    let mut b = vec![Complex::ZERO; n_use + 1];
    for n in 1..=n_use {
        let nx = Complex::from_real(n as f64 / x);
        // a_n: D_n/m + n/x
        let ta = d[n] / m + nx;
        let num_a = ta * Complex::from_real(psi_x[n]) - Complex::from_real(psi_x[n - 1]);
        let den_a = ta * xi_x[n] - xi_x[n - 1];
        if den_a.norm_sqr() <= 0.0 {
            return None;
        }
        a[n] = num_a / den_a;

        // b_n: m·D_n + n/x
        let tb = m * d[n] + nx;
        let num_b = tb * Complex::from_real(psi_x[n]) - Complex::from_real(psi_x[n - 1]);
        let den_b = tb * xi_x[n] - xi_x[n - 1];
        if den_b.norm_sqr() <= 0.0 {
            return None;
        }
        b[n] = num_b / den_b;
    }
    Some((a, b))
}

/// 批量版的 `(Q_ext, Q_sca, Q_abs)`。
///
/// 与逐个调用 [`mie_coefficient_a`] / [`mie_coefficient_b`] 等价，但复杂度是
/// `O(nmax)` 而非 `O(nmax²)`。
pub fn mie_efficiencies_series(m: Complex, x: f64) -> Option<(f64, f64, f64)> {
    let nmax = mie_max_order(x)?;
    let (a, b) = mie_coefficients_series(m, x, nmax)?;
    let (mut sum_ext, mut sum_sca) = (0.0, 0.0);
    for n in 1..=nmax as usize {
        let w = (2 * n + 1) as f64;
        sum_ext += w * (a[n].re + b[n].re);
        sum_sca += w * (a[n].norm_sqr() + b[n].norm_sqr());
    }
    let factor = 2.0 / (x * x);
    let q_ext = factor * sum_ext;
    let q_sca = factor * sum_sca;
    if !q_ext.is_finite() || !q_sca.is_finite() {
        return None;
    }
    Some((q_ext.max(0.0), q_sca.max(0.0), (q_ext - q_sca).max(0.0)))
}
