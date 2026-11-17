//! 丁达尔效应的**光谱**版本：把散射算成 RGB 三个通道。
//!
//! ## 为什么需要分通道
//!
//! [`crate::tyndall`] 的标量版本假设所有波长一视同仁。但现实中：
//!
//! * 米氏散射的波长依赖**弱但非零** —— 蓝光消光略强于红光
//! * 所以**穿过厚烟雾后，光柱会偏红**（蓝光先被散射掉）
//! * 这正是日落时太阳偏红的同一个机制
//!
//! 标量版本只能让光柱"变暗"，分通道版本能让它"变暖"——视觉上差别很大。
//!
//! ## 波长到 RGB 的转换
//!
//! 用 Dan Bruton 的经典近似（`www.midnightkite.com/color.html`），它把
//! 可见光波段（380~780 nm）分成六段线性插值。这不是 CIE 标准观察者的精确
//! 匹配，但对渲染足够——误差主要体现在深紫和深红两端，那两个区域在光柱里
//! 本来就不显著。
//!
//! **不追求色度学精度**：真正的 CIE 1931 匹配需要三张 471 项的查找表，而收益
//! 只是让紫色更准。这里的选择是工程判断，写在文档里以免误解。
//!
//! ## 能量守恒的约束
//!
//! RGB 三通道的散射系数必须满足 `σ_sca_channel <= σ_ext_channel`（每个通道
//! 独立）。这个约束在 [`SpectralOptics::is_valid`] 里检查。

/// 可见光波长范围（米）。
pub const VISIBLE_MIN_M: f64 = 380e-9;
/// 可见光波长上限（米）。
pub const VISIBLE_MAX_M: f64 = 780e-9;

/// 三通道（RGB）光学参数。
///
/// 每个通道独立持有散射/消光截面，因为**消光依赖波长**。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpectralOptics {
    /// 红通道散射截面（m²）。
    pub sigma_sca_r: f64,
    /// 绿通道散射截面（m²）。
    pub sigma_sca_g: f64,
    /// 蓝通道散射截面（m²）。
    pub sigma_sca_b: f64,
    /// 红通道消光截面（m²）。
    pub sigma_ext_r: f64,
    /// 绿通道消光截面（m²）。
    pub sigma_ext_g: f64,
    /// 蓝通道消光截面（m²）。
    pub sigma_ext_b: f64,
    /// 单颗粒质量（kg）。
    pub mass: f64,
}

impl SpectralOptics {
    /// 校验：每通道 `0 <= sigma_sca <= sigma_ext`，`mass > 0`，全为有限值。
    pub fn is_valid(&self) -> bool {
        let ch_ok = |s: f64, e: f64| s.is_finite() && e.is_finite() && s >= 0.0 && e >= s;
        ch_ok(self.sigma_sca_r, self.sigma_ext_r)
            && ch_ok(self.sigma_sca_g, self.sigma_ext_g)
            && ch_ok(self.sigma_sca_b, self.sigma_ext_b)
            && self.mass.is_finite()
            && self.mass > 0.0
    }

    /// 由单波长参数 + 中心波长构造三通道版本。
    ///
    /// 用米氏散射的波长标度关系 `σ ∝ λ^(-n)` 把同一个颗粒的截面推到三个
    /// 通道波长上：
    ///
    /// ```text
    /// σ(λ) = σ_ref · (λ / λ_ref)^(-n)
    /// ```
    ///
    /// `n` 的选择：
    ///
    /// | `n` | 对应 |
    /// | --- | --- |
    /// | `4` | 瑞利极限（强烈偏蓝） |
    /// | `1~2` | 米氏区（弱偏蓝，**丁达尔的实际情形**） |
    /// | `0` | 无波长依赖 |
    ///
    /// 所以做丁达尔光柱时 `n` 应该取 `1.0~1.5`，而不是 4。
    ///
    /// ## 散射与消光用**不同的**指数
    ///
    /// 这是本函数容易写错的地方。第一版对 `sigma_sca` 和 `sigma_ext` 用了同一个
    /// `exponent`，结果是三通道的反照率**完全相同** —— 而反照率的通道差异正是
    /// "光柱偏色"的一半来源（另一半是消光差异）。
    ///
    /// 物理上：`σ_ext = σ_sca + σ_abs`。**散射**有强的波长依赖（∝λ⁻ⁿ，瑞利极限
    /// n=4），**吸收**的波长依赖通常弱得多（灰体/宽带吸收近似 n≈0）。所以消光
    /// 的有效指数位于两者之间，具体由 `absorption_exponent` 单独给出：
    ///
    /// ```text
    /// σ_sca(λ) = σ_sca_ref · (λ/λ_ref)^(-n_sca)
    /// σ_abs_ref = σ_ext_ref - σ_sca_ref
    /// σ_abs(λ) = σ_abs_ref · (λ/λ_ref)^(-n_abs)
    /// σ_ext(λ) = σ_sca(λ) + σ_abs(λ)
    /// ```
    ///
    /// `absorption_exponent` 取 `0`（灰体吸收）时，纯散射介质（
    /// `σ_abs = 0`）会退化为 `σ_ext = σ_sca`，与第一版行为一致。
    pub fn from_monochrome(
        sigma_sca: f64,
        sigma_ext: f64,
        mass: f64,
        wavelength_ref: f64,
        exponent: f64,
    ) -> Option<Self> {
        // 灰体吸收：吸收部分无波长依赖。这是透明烟雾/水滴的合理近似。
        Self::from_monochrome_split(sigma_sca, sigma_ext, mass, wavelength_ref, exponent, 0.0)
    }

    /// 同 [`Self::from_monochrome`]，但允许散射与吸收有不同的波长指数。
    ///
    /// `absorption_exponent = 0` 是无波长偏好的灰体吸收；取值越大表示吸收越偏
    /// 蓝端。要求 `absorption_exponent >= 0`（吸收随波长向蓝端增强，物理上少见
    /// 但数学上允许；负值会让红光吸收更强，已拒绝以免误用）。
    pub fn from_monochrome_split(
        sigma_sca: f64,
        sigma_ext: f64,
        mass: f64,
        wavelength_ref: f64,
        scattering_exponent: f64,
        absorption_exponent: f64,
    ) -> Option<Self> {
        if !sigma_sca.is_finite()
            || !sigma_ext.is_finite()
            || !mass.is_finite()
            || !wavelength_ref.is_finite()
            || !scattering_exponent.is_finite()
            || !absorption_exponent.is_finite()
        {
            return None;
        }
        if sigma_sca < 0.0
            || sigma_ext < sigma_sca
            || mass <= 0.0
            || wavelength_ref <= 0.0
            || absorption_exponent < 0.0
        {
            return None;
        }
        let sigma_abs = sigma_ext - sigma_sca;
        let sca_scale = |lambda: f64| (lambda / wavelength_ref).powf(-scattering_exponent);
        let abs_scale = |lambda: f64| (lambda / wavelength_ref).powf(-absorption_exponent);
        let channel = |lambda_nm: f64| {
            let lambda = lambda_nm * 1e-9;
            let s = sigma_sca * sca_scale(lambda);
            let a = sigma_abs * abs_scale(lambda);
            (s, s + a)
        };
        let (sr, er) = channel(RED_NM);
        let (sg, eg) = channel(GREEN_NM);
        let (sb, eb) = channel(BLUE_NM);
        Some(Self {
            sigma_sca_r: sr,
            sigma_sca_g: sg,
            sigma_sca_b: sb,
            sigma_ext_r: er,
            sigma_ext_g: eg,
            sigma_ext_b: eb,
            mass,
        })
    }
}

/// 红通道中心波长（nm）。
pub const RED_NM: f64 = 650.0;
/// 绿通道中心波长（nm）。
pub const GREEN_NM: f64 = 550.0;
/// 蓝通道中心波长（nm）。
pub const BLUE_NM: f64 = 450.0;

impl SpectralOptics {
    /// 由**精确米氏散射**+ 材料参数直接算出三通道截面（ADR-0030）。
    ///
    /// 这是 [`SpectralOptics::from_monochrome`] 的物理版本。区别：
    ///
    /// | | `from_monochrome` | `from_mie`（本函数） |
    /// | --- | --- | --- |
    /// | 波长依赖 | 幂律拟合 `σ ∝ λ^(-n)` | **逐通道精确米氏解** |
    /// | 需要输入 | 手调的 `exponent` | 颗粒直径 + 复折射率 + 密度 |
    /// | 物理来源 | 无（拟合） | Clausius–Mossotti / 米氏级数 |
    ///
    /// ## 计算链
    ///
    /// ```text
    /// 直径 d、折射率 (n, κ)、密度 ρ
    ///   ↓ x = π·d/λ            每个通道各自的尺寸参数
    ///   ↓ 精确米氏              Q_sca(λ), Q_ext(λ)
    ///   ↓ × 几何截面 πd²/4     σ_sca(λ), σ_ext(λ)
    ///   ↓ 质量 = ρ·πd³/6
    /// SpectralOptics
    /// ```
    ///
    /// ## 为什么这比幂律拟合好
    ///
    /// 幂律 `σ ∝ λ^(-n)` 只在**瑞利区**（`x ≪ 1`）严格成立，那里 `n = 4`。
    /// 米氏区（`x ~ 1`）的波长依赖不是幂律 —— 有振荡（干涉结构），
    /// 用单一 `n` 拟合只能近似。
    ///
    /// 本函数逐通道解米氏方程，**自动包含振荡结构**。
    ///
    /// # Parameters
    /// * `diameter` 颗粒直径（m）
    /// * `refractive_index_real` 实部 `n`（水滴 1.33、矿物 1.5~1.6、烟灰 1.5~2.0）
    /// * `refractive_index_imag` 虚部 `κ`（0 = 透明，烟灰 0.1~0.8）
    /// * `density` 颗粒密度（kg/m³，水 1000、石英 2650、烟灰 ~1800）
    ///
    /// # Returns
    /// `Some(SpectralOptics)`；参数非法或任一通道的米氏级数失败时 `None`。
    pub fn from_mie(
        diameter: f64,
        refractive_index_real: f64,
        refractive_index_imag: f64,
        density: f64,
    ) -> Option<Self> {
        if !diameter.is_finite()
            || !refractive_index_real.is_finite()
            || !refractive_index_imag.is_finite()
            || !density.is_finite()
        {
            return None;
        }
        if diameter <= 0.0
            || refractive_index_real <= 0.0
            || refractive_index_imag < 0.0
            || density <= 0.0
        {
            return None;
        }
        let radius = diameter * 0.5;
        // 几何截面 πr²。
        let geom = core::f64::consts::PI * radius * radius;

        let channel = |lambda_nm: f64| {
            let lambda = lambda_nm * 1e-9;
            let x = core::f64::consts::PI * diameter / lambda;
            let (q_ext, q_sca, _) = crate::tyndall::mie_efficiencies_complex(
                x,
                refractive_index_real,
                refractive_index_imag,
            )?;
            // 钳住 `sigma_ext >= sigma_sca`：透明颗粒时两者在数学上相等，
            // 但浮点减法可能让 `q_sca` 略大于 `q_ext`（1e-17 量级）。
            // 不钳的话 `is_valid()` 会随机拒绝合法输入。
            let sca = (q_sca * geom).max(0.0);
            let ext = (q_ext * geom).max(sca);
            Some((sca, ext))
        };
        let (sr, er) = channel(RED_NM)?;
        let (sg, eg) = channel(GREEN_NM)?;
        let (sb, eb) = channel(BLUE_NM)?;

        // 质量 = ρ·(4/3)πr³ = ρ·πd³/6
        let mass = density * core::f64::consts::PI * diameter.powi(3) / 6.0;
        if mass <= 0.0 || !mass.is_finite() {
            return None;
        }
        let optics = Self {
            sigma_sca_r: sr,
            sigma_sca_g: sg,
            sigma_sca_b: sb,
            sigma_ext_r: er,
            sigma_ext_g: eg,
            sigma_ext_b: eb,
            mass,
        };
        if optics.is_valid() {
            Some(optics)
        } else {
            None
        }
    }
}
/// RGB 三通道亮度。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpectralLuminance {
    /// 红通道。
    pub r: f64,
    /// 绿通道。
    pub g: f64,
    /// 蓝通道。
    pub b: f64,
}

impl SpectralLuminance {
    /// 全零。
    pub const ZERO: Self = Self {
        r: 0.0,
        g: 0.0,
        b: 0.0,
    };

    /// 三通道都有限且非负。
    pub fn is_finite_non_negative(&self) -> bool {
        self.r.is_finite()
            && self.g.is_finite()
            && self.b.is_finite()
            && self.r >= 0.0
            && self.g >= 0.0
            && self.b >= 0.0
    }

    /// 感知亮度（Rec. 709 权重），用于与标量版本对比。
    pub fn luminance(&self) -> f64 {
        0.2126 * self.r + 0.7152 * self.g + 0.0722 * self.b
    }

    /// 归一化到最大通道为 1，返回 `(r, g, b)`。
    ///
    /// 用于**色调分析**：光柱偏红还是偏蓝。全零时返回全零而不是 NaN。
    pub fn normalized(&self) -> (f64, f64, f64) {
        let max = self.r.max(self.g).max(self.b);
        if max <= 0.0 {
            return (0.0, 0.0, 0.0);
        }
        (self.r / max, self.g / max, self.b / max)
    }
}

/// 波长（米）转 RGB，各通道落在 `0..=1`。
///
/// 可见光范围外返回全零（不是错误 —— 紫外线不产生视觉）。
///
/// 算法：Dan Bruton 的分段线性近似。
pub fn wavelength_to_rgb(wavelength_m: f64) -> Option<(f64, f64, f64)> {
    if !wavelength_m.is_finite() {
        return None;
    }
    let nm = wavelength_m * 1e9;
    if !(VISIBLE_MIN_M * 1e9..=VISIBLE_MAX_M * 1e9).contains(&nm) {
        return Some((0.0, 0.0, 0.0));
    }

    let (r, g, b) = if nm < 440.0 {
        (-(nm - 440.0) / 60.0, 0.0, 1.0)
    } else if nm < 490.0 {
        (0.0, (nm - 440.0) / 50.0, 1.0)
    } else if nm < 510.0 {
        (0.0, 1.0, -(nm - 510.0) / 20.0)
    } else if nm < 580.0 {
        ((nm - 510.0) / 70.0, 1.0, 0.0)
    } else if nm < 645.0 {
        (1.0, -(nm - 645.0) / 65.0, 0.0)
    } else {
        (1.0, 0.0, 0.0)
    };

    // 两端（<420nm 和 >700nm）的视觉灵敏度低，按 Bruton 的做法做强度衰减。
    let factor = if nm < 420.0 {
        0.3 + 0.7 * (nm - 380.0) / 40.0
    } else if nm > 700.0 {
        0.3 + 0.7 * (780.0 - nm) / 80.0
    } else {
        1.0
    };

    Some((
        (r * factor).clamp(0.0, 1.0),
        (g * factor).clamp(0.0, 1.0),
        (b * factor).clamp(0.0, 1.0),
    ))
}

/// 沿视线穿过**均匀**介质的分通道散射亮度。
///
/// 与 [`crate::tyndall::inscatter_along_ray`] 同一个模型，只是对三个通道各算
/// 一次，每通道用自己的 `σ_sca` / `σ_ext`。
///
/// 这样就能表现"厚烟雾里光柱偏红"：蓝光的 `σ_ext` 更大，衰减更快。
///
/// # Parameters
/// * `optics` 三通道光学参数
/// * `number_density` 数密度（个/m³）
/// * `phase` 相函数值（三通道共用 —— 相函数的波长依赖远弱于截面）
/// * `light` 三通道入射光强度
/// * `ray_length` 视线长度
/// * `steps` 采样段数
#[allow(clippy::too_many_arguments)]
pub fn inscatter_spectral(
    optics: SpectralOptics,
    number_density: f64,
    phase: f64,
    light: SpectralLuminance,
    ray_length: f64,
    steps: u32,
) -> Option<SpectralLuminance> {
    if !optics.is_valid() || !light.is_finite_non_negative() {
        return None;
    }
    if !number_density.is_finite()
        || !phase.is_finite()
        || !ray_length.is_finite()
        || number_density < 0.0
        || phase < 0.0
        || ray_length < 0.0
        || steps == 0
    {
        return None;
    }
    if ray_length == 0.0 || number_density == 0.0 {
        return Some(SpectralLuminance::ZERO);
    }

    let n_steps = steps as f64;
    let ds = ray_length / n_steps;

    // 每通道：scattered = σ_sca·n·phase·light，tau_per_step = σ_ext·n·ds。
    // 用中点法，与标量版本一致，保证单通道时两者逐位可比。
    let mut out = SpectralLuminance::ZERO;
    for (sca, ext, l_in, acc) in [
        (optics.sigma_sca_r, optics.sigma_ext_r, light.r, &mut out.r),
        (optics.sigma_sca_g, optics.sigma_ext_g, light.g, &mut out.g),
        (optics.sigma_sca_b, optics.sigma_ext_b, light.b, &mut out.b),
    ] {
        let scattered = sca * number_density * phase * l_in;
        let trans_per_step = (-ext * number_density * ds).exp();
        let mut trans = trans_per_step.sqrt();
        let mut sum = 0.0;
        for _ in 0..steps {
            sum += trans * scattered * ds;
            trans *= trans_per_step;
        }
        *acc = sum;
    }
    Some(out)
}

/// 分通道的单散射反照率（逐通道 `σ_sca/σ_ext`）。
///
/// 归一化后的比值能直接看出"散射偏好哪个颜色"。
pub fn spectral_albedo(optics: SpectralOptics) -> Option<SpectralLuminance> {
    if !optics.is_valid() {
        return None;
    }
    Some(SpectralLuminance {
        r: optics.sigma_sca_r / optics.sigma_ext_r,
        g: optics.sigma_sca_g / optics.sigma_ext_g,
        b: optics.sigma_sca_b / optics.sigma_ext_b,
    })
}

/// 分通道的 Beer–Lambert 透射率。
pub fn spectral_transmittance(
    optics: SpectralOptics,
    number_density: f64,
    path_length: f64,
) -> Option<SpectralLuminance> {
    if !optics.is_valid() {
        return None;
    }
    if !number_density.is_finite() || !path_length.is_finite() {
        return None;
    }
    if number_density < 0.0 || path_length < 0.0 {
        return None;
    }
    let tau = |e: f64| (-e * number_density * path_length).exp();
    Some(SpectralLuminance {
        r: tau(optics.sigma_ext_r),
        g: tau(optics.sigma_ext_g),
        b: tau(optics.sigma_ext_b),
    })
}

// ── 粒子驱动的光谱散射（ADR-0021）───────────────────────────────────────────
//
// 把 `tyndall_particles` 的粒子云采样与 `tyndall_spectral` 的分通道累积合起来：
// 沿视线逐段从粒子分布采数密度，再对**三个通道各自**累积 Beer–Lambert。
//
// 这是"流动的烟雾里的彩色光柱"的完整实现 —— 之前的两个模块各占一半：
//
// * `tyndall_particles` 有粒子驱动，但输出标量
// * `tyndall_spectral` 有分通道，但只支持均匀介质

/// 沿视线穿过**粒子云**的分通道散射亮度。
///
/// 物理模型与 [`crate::tyndall_particles::inscatter_through_particles`] 相同，
/// 只是对 R/G/B 三通道各算一次，每通道用自己的 `σ_sca` / `σ_ext`。
///
/// ## 关键实现细节：三通道的透射率必须**独立**累积
///
/// 容易写错的地方：用同一个透射率标量累加三个通道。那样蓝光不会比红光衰减得
/// 快，"偏红"就消失了。每通道必须有独立的 `trans_r` / `trans_g` / `trans_b`。
///
/// ## 采样点复用
///
/// 三个通道在**同一条视线上**采样，所以数密度只算一次、三通道共用 ——
/// 相比调三次单通道函数，核求值的开销降到三分之一。
#[allow(clippy::too_many_arguments)]
pub fn inscatter_spectral_through_particles(
    positions: &[[f64; 3]],
    optics: SpectralOptics,
    origin: [f64; 3],
    direction: [f64; 3],
    length: f64,
    smoothing_radius: f64,
    phase: f64,
    light: SpectralLuminance,
    steps: u32,
) -> Option<SpectralLuminance> {
    if !optics.is_valid() || !light.is_finite_non_negative() {
        return None;
    }
    if !origin.iter().all(|v| v.is_finite())
        || !direction.iter().all(|v| v.is_finite())
        || !length.is_finite()
        || !phase.is_finite()
    {
        return None;
    }
    if length < 0.0 || phase < 0.0 || steps == 0 {
        return None;
    }
    // 方向必须已归一化 —— 与标量版同一约束，理由见 tyndall_particles。
    let norm =
        (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2])
            .sqrt();
    if (norm - 1.0).abs() > 1e-6 {
        return None;
    }
    if length == 0.0 || positions.is_empty() {
        return Some(SpectralLuminance::ZERO);
    }

    let n_steps = steps as f64;
    let ds = length / n_steps;

    // 每通道的散射源强（与数密度无关的部分）。
    let src_r = optics.sigma_sca_r * phase * light.r;
    let src_g = optics.sigma_sca_g * phase * light.g;
    let src_b = optics.sigma_sca_b * phase * light.b;

    // 三通道独立的透射率。
    let mut trans_r = 1.0f64;
    let mut trans_g = 1.0f64;
    let mut trans_b = 1.0f64;
    let mut sum_r = 0.0f64;
    let mut sum_g = 0.0f64;
    let mut sum_b = 0.0f64;

    // 建一次均匀网格，把每步的密度查询从 O(N) 降到 O(k)。三通道共享采样，
    // 所以网格的收益与标量版相同（见 ADR-0023）。
    // 粒子少时建网格不划算，走朴素路径。
    const GRID_THRESHOLD: usize = 64;
    let grid = if positions.len() >= GRID_THRESHOLD {
        Some(crate::tyndall_particles::UniformGrid::build(
            positions,
            smoothing_radius,
        )?)
    } else {
        None
    };

    for k in 0..steps {
        let t = (k as f64 + 0.5) * ds;
        let q = [
            origin[0] + direction[0] * t,
            origin[1] + direction[1] * t,
            origin[2] + direction[2] * t,
        ];
        // 数密度只采样一次，三通道共用。
        let n = match &grid {
            Some(g) => g.number_density(q, smoothing_radius)?,
            None => {
                crate::tyndall_particles::sample_number_density(positions, q, smoothing_radius)?
            }
        };

        sum_r += trans_r * n * src_r * ds;
        sum_g += trans_g * n * src_g * ds;
        sum_b += trans_b * n * src_b * ds;

        trans_r *= (-optics.sigma_ext_r * n * ds).exp();
        trans_g *= (-optics.sigma_ext_g * n * ds).exp();
        trans_b *= (-optics.sigma_ext_b * n * ds).exp();

        // 三通道都衰减到可忽略时提前退出。
        if trans_r < 1e-12 && trans_g < 1e-12 && trans_b < 1e-12 {
            break;
        }
    }

    Some(SpectralLuminance {
        r: sum_r,
        g: sum_g,
        b: sum_b,
    })
}
/// 粒子驱动 + 分通道 + **多次散射** + **空间加速** 的完整版本。
///
/// 把前四条 ADR 的能力合成一个函数：
///
/// | 能力 | 来源 |
/// | --- | --- |
/// | 粒子云驱动 | [ADR-0019](../../adr/0019-particle-driven-volumetric-scattering.md) |
/// | RGB 分通道 | [ADR-0020](../../adr/0020-spectral-tyndall.md) |
/// | 多次散射 | [ADR-0022](../../adr/0022-multiple-scattering.md) |
/// | 均匀网格加速 | [ADR-0023](../../adr/0023-uniform-grid-acceleration.md) |
///
/// ## 多次散射在分通道下的处理
///
/// 每通道有**自己的反照率** `ω_ch = σ_sca_ch / σ_ext_ch`，所以级数比 `r_ch`
/// 也不同：
///
/// ```text
/// L_ch = L1_ch · (1 - r_ch^N) / (1 - r_ch)      r_ch = ω_ch · g
/// ```
///
/// 不能用一个共享的 `r` —— 那会让三通道的修正倍数相同，"厚介质偏红"的高阶部分
/// 就丢失了。
///
/// ## 加速
///
/// 三通道共享同一条视线的数密度采样，所以网格只建一次、每个采样点只查一次。
#[allow(clippy::too_many_arguments)]
pub fn inscatter_spectral_full(
    positions: &[[f64; 3]],
    optics: SpectralOptics,
    origin: [f64; 3],
    direction: [f64; 3],
    length: f64,
    smoothing_radius: f64,
    phase: f64,
    light: SpectralLuminance,
    steps: u32,
    orders: u32,
    geometric_factor: f64,
) -> Option<SpectralLuminance> {
    if orders == 0 || orders > crate::tyndall::MAX_SCATTER_ORDERS {
        return None;
    }
    if !geometric_factor.is_finite() || geometric_factor <= 0.0 || geometric_factor > 1.0 {
        return None;
    }

    // 单次散射（含加速）。
    let first = inscatter_spectral_through_particles(
        positions,
        optics,
        origin,
        direction,
        length,
        smoothing_radius,
        phase,
        light,
        steps,
    )?;
    if orders == 1 {
        return Some(first);
    }

    // 每通道自己的反照率。
    let albedo = crate::tyndall_spectral::spectral_albedo(optics)?;
    let series = |l1: f64, alb: f64| {
        let r = alb * geometric_factor;
        if r >= 1.0 {
            // 退化：纯散射 + g=1，所有光都留在系统里。
            return l1 * orders as f64;
        }
        l1 * (1.0 - r.powi(orders as i32)) / (1.0 - r)
    };
    Some(SpectralLuminance {
        r: series(first.r, albedo.r),
        g: series(first.g, albedo.g),
        b: series(first.b, albedo.b),
    })
}
// ── CIE 1931 色匹配函数与光谱积分（ADR-0032）────────────────────────────────
//
// 三离散波长（650/550/450 nm）只能粗略表示颜色。要做"物理正确的颜色"，
// 需要按 CIE 1931 标准观察者的匹配函数对**连续光谱**加权积分。
//
// 这里用 Wyman, Sloan, Shirley (JCGT 2013) 的**多瓣高斯解析拟合**，
// 不需要 471 项查找表：
//
// ```text
// x̄(λ) = 1.056 g(λ;599.8,37.9,31.0) + 0.362 g(λ;442.0,16.0,26.7) - 0.065 g(λ;501.1,20.4,26.2)
// ȳ(λ) = 0.821 g(λ;568.8,46.9,40.5) + 0.286 g(λ;530.9,16.3,31.1)
// z̄(λ) = 1.217 g(λ;437.0,11.8,36.0) + 0.681 g(λ;459.0,26.0,13.8)
// ```
//
// `g` 是分段高斯：`σ` 在 `λ < μ` 时取 `σ₁`，否则取 `σ₂`。
//
// **精度**：拟合误差约峰值的 1%，对渲染足够。需要色度学级精度时请用官方表。

/// 分段高斯瓣 `g(λ; μ, σ₁, σ₂)`。
fn cie_lobe(lambda_nm: f64, mu: f64, s1: f64, s2: f64) -> f64 {
    let s = if lambda_nm < mu { s1 } else { s2 };
    let t = (lambda_nm - mu) / s;
    (-0.5 * t * t).exp()
}

/// CIE 1931 标准观察者的色匹配函数 `(x̄, ȳ, z̄)`。
///
/// 用 [Wyman, Sloan, Shirley (JCGT 2013)] 的多瓣高斯拟合。`ȳ` 同时也是
/// 光度函数 `V(λ)`。
///
/// 可见光范围外返回接近零的值（高斯衰减），不做特殊处理。
///
/// [Wyman, Sloan, Shirley (JCGT 2013)]: http://jcgt.org/published/0002/02/01/
pub fn cie_color_matching(lambda_nm: f64) -> Option<(f64, f64, f64)> {
    if !lambda_nm.is_finite() {
        return None;
    }
    let xb = 1.056 * cie_lobe(lambda_nm, 599.8, 37.9, 31.0)
        + 0.362 * cie_lobe(lambda_nm, 442.0, 16.0, 26.7)
        - 0.065 * cie_lobe(lambda_nm, 501.1, 20.4, 26.2);
    let yb = 0.821 * cie_lobe(lambda_nm, 568.8, 46.9, 40.5)
        + 0.286 * cie_lobe(lambda_nm, 530.9, 16.3, 31.1);
    let zb = 1.217 * cie_lobe(lambda_nm, 437.0, 11.8, 36.0)
        + 0.681 * cie_lobe(lambda_nm, 459.0, 26.0, 13.8);
    Some((xb, yb, zb))
}

/// CIE XYZ（D65 白点）→ **线性** sRGB。
///
/// 用 IEC 61966-2-1 的矩阵。负值钳到 0（超出 sRGB 色域的颜色无法表示）。
///
/// 结果是**线性**值，要显示还需 gamma 编码（`v^(1/2.2)` 近似）。
pub fn xyz_to_linear_srgb(x: f64, y: f64, z: f64) -> Option<(f64, f64, f64)> {
    if !x.is_finite() || !y.is_finite() || !z.is_finite() {
        return None;
    }
    let r = 3.2406 * x - 1.5372 * y - 0.4986 * z;
    let g = -0.9689 * x + 1.8758 * y + 0.0415 * z;
    let b = 0.0557 * x - 0.2040 * y + 1.0570 * z;
    Some((r.max(0.0), g.max(0.0), b.max(0.0)))
}

/// 光谱积分：由**采样点**（波长 nm + 光谱辐亮度）算出 RGB。
///
/// 做梯形积分：
///
/// ```text
/// X = ∫ L(λ)·x̄(λ) dλ        Y = ∫ L(λ)·ȳ(λ) dλ        Z = ∫ L(λ)·z̄(λ) dλ
/// ```
///
/// 然后经 [`xyz_to_linear_srgb`] 转成 RGB。
///
/// # Parameters
/// * `samples` 至少 2 个 `(lambda_nm, radiance)`，按波长**升序**
///
/// # Returns
/// `Some(SpectralLuminance)`（**线性** sRGB）；采样不足或非有限时 `None`。
///
/// # 归一化
///
/// 结果按 `ȳ` 的积分归一化，使得"平坦光谱 = 单位亮度"。这样调用方不用关心
/// 绝对标度。
pub fn spectral_samples_to_rgb(samples: &[(f64, f64)]) -> Option<SpectralLuminance> {
    if samples.len() < 2 {
        return None;
    }
    let mut xyz = [0.0f64; 3];
    let mut y_norm = 0.0f64;
    for i in 1..samples.len() {
        let (l0, r0) = samples[i - 1];
        let (l1, r1) = samples[i];
        if !l0.is_finite() || !l1.is_finite() || !r0.is_finite() || !r1.is_finite() {
            return None;
        }
        if l1 <= l0 {
            // 必须升序，否则积分无意义。
            return None;
        }
        if r0 < 0.0 || r1 < 0.0 {
            return None;
        }
        let d = l1 - l0;
        let (x0, y0, z0) = cie_color_matching(l0)?;
        let (x1, y1, z1) = cie_color_matching(l1)?;
        // 梯形：0.5·(f0+f1)·dλ
        xyz[0] += 0.5 * (r0 * x0 + r1 * x1) * d;
        xyz[1] += 0.5 * (r0 * y0 + r1 * y1) * d;
        xyz[2] += 0.5 * (r0 * z0 + r1 * z1) * d;
        // 归一化分母：白光的 ȳ 积分（辐射度 = 1）。
        y_norm += 0.5 * (y0 + y1) * d;
    }
    if y_norm <= 0.0 {
        return None;
    }
    let (r, g, b) = xyz_to_linear_srgb(xyz[0] / y_norm, xyz[1] / y_norm, xyz[2] / y_norm)?;
    Some(SpectralLuminance { r, g, b })
}

/// 逐波长的米氏光谱采样：返回 `(λ_nm, σ_sca, σ_ext)`。
///
/// 这是 [`spectral_samples_to_rgb`] 的数据来源 —— 把颗粒的连续光谱离散成
/// `samples` 个采样点，供梯形积分使用。
///
/// 波长范围固定为 `380~780 nm`（可见光），等间距。
///
/// # Returns
/// `Some(Vec)`，长度 `samples`，按波长升序；`samples < 2` 或参数非法时 `None`。
#[allow(clippy::too_many_arguments)]
pub fn mie_spectral_samples(
    diameter: f64,
    refractive_index_real: f64,
    refractive_index_imag: f64,
    samples: u32,
) -> Option<Vec<(f64, f64, f64)>> {
    if !diameter.is_finite()
        || !refractive_index_real.is_finite()
        || !refractive_index_imag.is_finite()
    {
        return None;
    }
    if diameter <= 0.0 || refractive_index_real <= 0.0 || refractive_index_imag < 0.0 {
        return None;
    }
    if !(2..=512).contains(&samples) {
        return None;
    }
    let radius = diameter * 0.5;
    let geom = core::f64::consts::PI * radius * radius;
    let n = samples as f64;
    let mut out = Vec::with_capacity(samples as usize);
    for i in 0..samples {
        let lambda_nm =
            VISIBLE_MIN_M * 1e9 + (VISIBLE_MAX_M - VISIBLE_MIN_M) * 1e9 * (i as f64) / (n - 1.0);
        let lambda = lambda_nm * 1e-9;
        let x = core::f64::consts::PI * diameter / lambda;
        let (q_ext, q_sca, _) = crate::tyndall::mie_efficiencies_complex(
            x,
            refractive_index_real,
            refractive_index_imag,
        )?;
        let sca = (q_sca * geom).max(0.0);
        let ext = (q_ext * geom).max(sca);
        out.push((lambda_nm, sca, ext));
    }
    Some(out)
}

/// **均匀介质的连续光谱散射 → 线性 sRGB**（ADR-0032）。
///
/// 这是三离散波长版本的物理升级版：
///
/// | | 三离散波长（ADR-0020） | 本函数 |
/// | --- | --- | --- |
/// | 光谱采样 | 固定 650/550/450 nm | `samples` 个等间距点（380~780 nm） |
/// | 加权 | 直接取三点 | **CIE 1931 匹配函数梯形积分** |
/// | 米氏区的振荡结构 | 被三点采样漏掉 | 逐点解出，积分保留 |
/// | 代价 | 3 次米氏 | `samples` 次米氏 + `samples` 次散射积分 |
///
/// ## 为什么不能先积分 σ 再算一次散射
///
/// 散射亮度对 `σ` 是**非线性**的（`1 − exp(−τ)` 与 Beer–Lambert 累积），
/// 所以必须先逐波长算出 `L(λ)`，再对 `L(λ)` 做线性积分。这是辐射度学的基本要求。
///
/// # Parameters
/// * `diameter` / `refractive_index_real` / `refractive_index_imag` 材料参数
/// * `number_density` 数密度（个/m³）
/// * `phase` 相函数值（丁达尔取前向，`0.7~0.9` 的 Henyey–Greenstein）
/// * `light_intensity` 入射光强度（白光谱，平坦）
/// * `ray_length` 视线长度
/// * `steps` 单次散射积分的采样段数
/// * `samples` 光谱采样点数（推荐 `15~31`）
///
/// # Returns
/// `Some(SpectralLuminance)`（线性 sRGB，`ȳ` 归一化）；参数非法时 `None`。
#[allow(clippy::too_many_arguments)]
pub fn inscatter_rgb_uniform(
    diameter: f64,
    refractive_index_real: f64,
    refractive_index_imag: f64,
    number_density: f64,
    phase: f64,
    light_intensity: f64,
    ray_length: f64,
    steps: u32,
    samples: u32,
) -> Option<SpectralLuminance> {
    if !number_density.is_finite()
        || !phase.is_finite()
        || !light_intensity.is_finite()
        || !ray_length.is_finite()
    {
        return None;
    }
    if number_density < 0.0 || phase < 0.0 || light_intensity < 0.0 || ray_length <= 0.0 {
        return None;
    }
    let spectrum = mie_spectral_samples(
        diameter,
        refractive_index_real,
        refractive_index_imag,
        samples,
    )?;

    let mut radiance: Vec<(f64, f64)> = Vec::with_capacity(spectrum.len());
    for (lambda_nm, sca, ext) in spectrum {
        let l = crate::tyndall::inscatter_along_ray(
            sca,
            ext,
            number_density,
            phase,
            light_intensity,
            ray_length,
            steps,
        )?;
        radiance.push((lambda_nm, l));
    }
    spectral_samples_to_rgb(&radiance)
}
