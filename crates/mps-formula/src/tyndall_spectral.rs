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
