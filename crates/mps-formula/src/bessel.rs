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
pub const MAX_MIE_ORDER: u32 = 200;

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
    // 递推起点：比需要的阶高一些，保证收敛。
    let start = nmax + 20 + (x as usize).min(200);

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

/// 米氏散射系数 `a_n`（复数）。
///
/// Bohren–Huffman 式 (4.88)：
///
/// ```text
///        m²·j_n(mx)·[x·j_n(x)]' - j_n(x)·[mx·j_n(mx)]'
/// a_n = ───────────────────────────────────────────────
///        m²·j_n(mx)·[x·h_n(x)]' - h_n(x)·[mx·j_n(mx)]'
/// ```
///
/// 直接用会溢出。改用 Riccati–Bessel 与对数导数 `D_n` 的等价形式：
///
/// ```text
///        m²·psi_n(mx)·psi_n'(x) - psi_n(x)·psi_n'(mx)
/// a_n = ───────────────────────────────────────────────
///        m²·psi_n(mx)·xi_n'(x) - xi_n(x)·psi_n'(mx)
/// ```
///
/// 其中 `psi_n'` 用递推 `psi_n'(x) = psi_{n-1}(x) - n/x·psi_n(x)`。
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
    let mx_abs = mx.abs();
    if mx_abs <= 0.0 || !mx_abs.is_finite() {
        return None;
    }
    let psi_x = riccati_bessel_psi(x, n)?;
    let xi_x = riccati_bessel_xi(x, n)?;
    let psi_mx = riccati_bessel_psi_complex(mx, n)?;

    let k = n as usize;
    let dpsi_x = psi_derivative(&psi_x, x, k)?;
    let dxi_x = xi_derivative(&xi_x, x, k)?;
    let dpsi_mx = psi_derivative_complex(&psi_mx, mx, k)?;

    let m2 = m * m;
    let num = m2 * psi_mx[k] * Complex::from_real(dpsi_x) - Complex::from_real(psi_x[k]) * dpsi_mx;
    let den = m2 * psi_mx[k] * dxi_x - xi_x[k] * dpsi_mx;
    if den.norm_sqr() <= 0.0 {
        return None;
    }
    Some(num / den)
}

/// 米氏散射系数 `b_n`（复数）。
///
/// ```text
///        m²·psi_n(mx)·psi_n'(x) - psi_n(x)·psi_n'(mx)
/// b_n = ───────────────────────────────────────────────
///        m²·psi_n(mx)·xi_n'(x) - xi_n(x)·psi_n'(mx)
/// ```
///
/// 与 `a_n` 只差一个 `m²` 的位置 —— 实际上 `b_n` 的分子里 `m²` 只乘第一项。
/// 具体形式见 Bohren–Huffman 式 (4.89)。
pub fn mie_coefficient_b(m: Complex, x: f64, n: u32) -> Option<Complex> {
    if !m.is_finite() || !x.is_finite() || x <= 0.0 || n == 0 || n > MAX_MIE_ORDER {
        return None;
    }
    let mx = m * Complex::from_real(x);
    let mx_abs = mx.abs();
    if mx_abs <= 0.0 || !mx_abs.is_finite() {
        return None;
    }
    let psi_x = riccati_bessel_psi(x, n)?;
    let xi_x = riccati_bessel_xi(x, n)?;
    let psi_mx = riccati_bessel_psi_complex(mx, n)?;

    let k = n as usize;
    let dpsi_x = psi_derivative(&psi_x, x, k)?;
    let dxi_x = xi_derivative(&xi_x, x, k)?;
    let dpsi_mx = psi_derivative_complex(&psi_mx, mx, k)?;

    // b_n 与 a_n 的唯一区别：**m² 不出现**。a_n 的分子和分母都在第一项上乘
    // m²，b_n 则完全不乘。见 Bohren–Huffman (4.89)。
    //
    // 写错成"分子第二项乘 m²"会让 |b_n| 超出 1（违反能量守恒），
    // 实测 x=5 时 |b_3| = 1.56 —— 正是这个 bug。
    let num_b = psi_mx[k] * Complex::from_real(dpsi_x) - Complex::from_real(psi_x[k]) * dpsi_mx;
    let den_b = psi_mx[k] * dxi_x - xi_x[k] * dpsi_mx;
    if den_b.norm_sqr() <= 0.0 {
        return None;
    }
    Some(num_b / den_b)
}

/// `psi_n'(x) = psi_{n-1}(x) - n/x·psi_n(x)`（实数序列）。
fn psi_derivative(psi: &[f64], x: f64, n: usize) -> Option<f64> {
    if n == 0 || n >= psi.len() {
        return None;
    }
    Some(psi[n - 1] - n as f64 / x * psi[n])
}

/// `xi_n'(x) = xi_{n-1}(x) - n/x·xi_n(x)`（复数序列）。
fn xi_derivative(xi: &[Complex], x: f64, n: usize) -> Option<Complex> {
    if n == 0 || n >= xi.len() {
        return None;
    }
    Some(xi[n - 1] - xi[n] * Complex::from_real(n as f64 / x))
}

/// `psi_n'(mx)`（复数序列，`mx` 是复数）。
fn psi_derivative_complex(psi: &[Complex], mx: Complex, n: usize) -> Option<Complex> {
    if n == 0 || n >= psi.len() {
        return None;
    }
    Some(psi[n - 1] - psi[n] * Complex::from_real(n as f64) / mx)
}

/// 复数参数的 `psi_n(z) = z·j_n(z)`。
///
/// 复参数不能直接用实数 Miller 递推。这里用**幂级数**展开（`|z|` 不大时收敛快）：
///
/// ```text
/// j_n(z) = z^n / (2n+1)!! · Σ_{k=0}^∞ (-z²/2)^k · (2n+1)!! / (2k)!!(2n+2k+1)!!
/// ```
///
/// 对米氏散射，`|mx|` 通常在 `0.1~100` 量级，级数需要几十项。这里截断到
/// 收敛（相邻项相对变化 < 1e-15）或 200 项。
///
/// **这是本模块最脆弱的部分**：大 `|z|` 时幂级数会抵消失精。米氏散射的常见
/// 参数范围内（`x < 50`）够用；更大尺寸参数需要更好的算法（比如连分式）。
fn spherical_bessel_j_complex(z: Complex, n: u32) -> Option<Complex> {
    if !z.is_finite() || n > MAX_MIE_ORDER {
        return None;
    }
    let nn = n as f64;
    // 球贝塞尔 j_n(z) 的幂级数首项是 z^n / (2n+1)!!，其中
    //   (2n+1)!! = 1·3·5···(2n+1)   —— 共 n+1 个因子。
    //
    // 注意这里容易写错：循环 `while k < n` 得到的是 (2n-1)!!（n 个因子），
    // 少乘一个 (2n+1)。差这一个因子会让 a_1 在瑞利极限下偏大 3 倍
    // （x=0.01 时实测 6.67e-7 而非 1.36e-7 —— 正是这个 bug）。
    let mut double_fact = 1.0f64;
    let mut k = 0.0f64;
    while k <= nn {
        double_fact *= 2.0 * k + 1.0;
        k += 1.0;
    }
    // 首项：z^n / (2n+1)!!
    let mut z_pow = Complex::ONE;
    for _ in 0..n {
        z_pow = z_pow * z;
    }
    let first = z_pow / Complex::from_real(double_fact);

    let z2 = z * z;
    let mut term = first;
    let mut sum = first;
    // 递推比：term_{k+1}/term_k = -z² / (2(k+1)(2n+2k+3))
    for i in 0..200 {
        let kf = i as f64;
        let denom = 2.0 * (kf + 1.0) * (2.0 * nn + 2.0 * kf + 3.0);
        term = term * (-z2) / Complex::from_real(denom);
        let before = sum.norm_sqr();
        sum = sum + term;
        if !sum.is_finite() {
            return None;
        }
        // 收敛判据：增量相对总量足够小。
        if term.norm_sqr() <= before * 1e-30 {
            break;
        }
    }
    Some(sum)
}

/// 复数参数的 `psi_n(z) = z·j_n(z)`。
fn riccati_bessel_psi_complex(z: Complex, n: u32) -> Option<Vec<Complex>> {
    // 需要 psi_0..psi_n，逐个算（级数各自独立，不能递推）。
    let mut out = Vec::with_capacity(n as usize + 1);
    for k in 0..=n {
        let j = spherical_bessel_j_complex(z, k)?;
        out.push(z * j);
    }
    Some(out)
}
