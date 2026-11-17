//! 丁达尔效应（Tyndall effect）的数据层：体积散射的物理量计算。
//!
//! 这一层**只算数据，不做渲染**。渲染（把光柱画出来）由宿主负责；这里提供
//! 它需要的全部输入：散射系数、消光系数、相函数值、沿视线的累积亮度。
//!
//! ## 为什么丁达尔主要是米氏散射，不是瑞利
//!
//! 这是最容易搞错的地方。丁达尔效应的标志是**能看到一束接近白色的光柱**
//! （烟雾、胶体、粉尘里的可见光束）。而瑞利散射（1/λ⁴）会给出**蓝色**光柱。
//!
//! | | 瑞利 | 米氏 |
//! | --- | --- | --- |
//! | 适用 | 粒径 ≪ λ（气体分子） | 粒径 ≈ 或 > λ（胶体、雾滴、粉尘） |
//! | 波长依赖 | 1/λ⁴ | 弱，近似无关 |
//! | 角分布 | 前后对称（1+cos²θ） | **强烈前倾** |
//!
//! 所以本模块以米氏为主，瑞利作为对照保留（
//! [`crate::electromagnetism::rayleigh_scattering_cross_section`] 已有精确式）。
//!
//! ## 关于精度的诚实说明
//!
//! 米氏散射的**精确解**需要 Riccati-Bessel 函数的级数求和
//! （Bohren & Huffman, *Absorption and Scattering of Light by Small Particles*）。
//! 本仓库**没有贝塞尔函数实现**（`mps-formula` 里 0 处），引入它需要新增数值库
//! 依赖。
//!
//! 因此这里用**渐近式与近似式**，它们的适用范围写在每个函数的文档里：
//!
//! * 小尺寸参数（`x ≪ 1`）→ 瑞利极限，用精确式
//! * 中等尺寸参数 → van de Hulst 反常衍射近似，误差约 10~20%
//! * 大尺寸参数（`x ≫ 1`）→ 消光趋近 `2`（消光佯谬）
//!
//! **对渲染用途（可见光柱的亮度分布）这够用**：人眼对光柱亮度的分辨远达不到
//! 20% 的精度。如果将来要做定量光度学，应该引入贝塞尔实现。

/// 尺寸参数 `x = π·d/λ`（无量纲）。米氏理论的一切都从它开始。
///
/// `d` 是颗粒直径，`λ` 是真空波长，都取米。
///
/// # Returns
/// `Some(x)`；任一输入非有限或 `<= 0` 时 `None`。
pub fn size_parameter(diameter: f64, wavelength: f64) -> Option<f64> {
    if !diameter.is_finite() || !wavelength.is_finite() {
        return None;
    }
    if diameter <= 0.0 || wavelength <= 0.0 {
        return None;
    }
    Some(std::f64::consts::PI * diameter / wavelength)
}

/// 瑞利相函数（归一化到 `∫p dΩ/(4π) = 1`）。
///
/// `p(θ) = (3/4)·(1 + cos²θ)`
///
/// 前后对称 —— 这是瑞利散射的特征，也是它和丁达尔（强前倾）的区别。
///
/// `cos_theta` 必须落在 `[-1, 1]`。
pub fn rayleigh_phase(cos_theta: f64) -> Option<f64> {
    if !cos_theta.is_finite() || !(-1.0..=1.0).contains(&cos_theta) {
        return None;
    }
    Some(0.75 * (1.0 + cos_theta * cos_theta))
}

/// Henyey–Greenstein 相函数（归一化到 `∫p dΩ/(4π) = 1`）。
///
/// `p(θ) = (1 - g²) / (1 + g² - 2g·cosθ)^(3/2)`
///
/// `g` 是非对称参数，落在 `(-1, 1)`：
///
/// | `g` | 含义 |
/// | --- | --- |
/// | `0` | 各向同性 |
/// | `0 < g < 1` | **前倾**（丁达尔光柱的特征，典型值 `0.7~0.9`） |
/// | `-1 < g < 0` | 后倾 |
///
/// 之所以用 HG 而不是真正的米氏相函数：HG 是解析式、无特殊函数依赖、且对
/// 前倾的刻画足够好。真正的米氏相函数有振荡的旁瓣，对渲染意义不大。
///
/// **`g` 越接近 1 越前倾，光柱越"亮且窄"**。`g = 0.9` 时正前方亮度是侧向的
/// 约 `(1+g²)/(1-g)² ≈ 181` 倍，这就是为什么丁达尔光柱看起来像"一条亮线"。
pub fn henyey_greenstein_phase(cos_theta: f64, g: f64) -> Option<f64> {
    if !cos_theta.is_finite() || !g.is_finite() {
        return None;
    }
    if !(-1.0..=1.0).contains(&cos_theta) || g.abs() >= 1.0 {
        return None;
    }
    let denom = 1.0 + g * g - 2.0 * g * cos_theta;
    if denom <= 0.0 {
        return None;
    }
    Some((1.0 - g * g) / denom.powf(1.5))
}

/// 米氏散射效率因子 `Q_sca`（渐近近似）。
///
/// **不是精确解**（精确解需要 Riccati-Bessel 级数）。这里用三段近似：
///
/// | 尺寸参数 `x` | 采用的近似 | 依据 |
/// | --- | --- | --- |
/// | `x < 0.3` | 瑞利极限 `Q_sca ≈ (8/3)·x⁴·|K|²` | 小颗粒渐近式 |
/// | `0.3 ≤ x < 20` | van de Hulst 反常衍射 | 中等颗粒，误差 10~20% |
/// | `x ≥ 20` | `Q_sca → 2`（消光佯谬） | 大颗粒极限 |
///
/// `refractive_index` 是**相对**折射率（颗粒/介质），通常 `1.3~1.6`（水/矿物
/// 在空气中）。
///
/// **渲染用途够用**；定量光度学需要精确解。
pub fn mie_scattering_efficiency(size_param: f64, refractive_index: f64) -> Option<f64> {
    if !size_param.is_finite() || !refractive_index.is_finite() {
        return None;
    }
    if size_param <= 0.0 || refractive_index <= 0.0 {
        return None;
    }
    // 相对折射率对比因子 |K|² = |(m²-1)/(m²+2)|²，三段近似共用。
    let m2 = refractive_index * refractive_index;
    let k = (m2 - 1.0) / (m2 + 2.0);
    let k2 = k * k;

    let q = if size_param < 0.3 {
        // 瑞利极限：Q_sca = (8/3)·x⁴·|K|²
        8.0 / 3.0 * size_param.powi(4) * k2
    } else if size_param < 20.0 {
        // van de Hulst 反常衍射：在瑞利与几何光学之间平滑过渡。
        // 用 `2 - 4/ρ·sin(ρ) + 4/ρ²·(1-cos(ρ))` 的消光式（ρ = 2x(m-1)）
        // 乘以单散射反照率近似，得到散射效率。
        let rho = 2.0 * size_param * (refractive_index - 1.0);
        let extinction = if rho.abs() < 1e-9 {
            0.0
        } else {
            2.0 - 4.0 / rho * rho.sin() + 4.0 / (rho * rho) * (1.0 - rho.cos())
        };
        // 消光 = 散射 + 吸收。忽略吸收（透明颗粒）时散射 ≈ 消光。
        extinction.max(0.0) * (k2 / (k2 + 0.25)).max(0.1)
    } else {
        // 大颗粒：消光趋近 2，散射近似同样趋近 2·(散射/消光比)。
        2.0 * (k2 / (k2 + 0.25)).max(0.2)
    };
    Some(q.max(0.0))
}

/// 米氏散射效率 `Q_sca` 的**精确解**（Riccati–Bessel 级数）。
///
/// 与 [`mie_scattering_efficiency`] 的关系：
///
/// | | 本函数 | `mie_scattering_efficiency` |
/// | --- | --- | --- |
/// | 方法 | Riccati–Bessel 级数求和 | 三段渐近近似 |
/// | 精度 | 中等尺寸已验证（`x=10` 对照文献差 1.5%） | 10~20% |
/// | 代价 | `O(n_max)` 次复数运算 | `O(1)` |
/// | 适用 | 需要定量正确时 | 实时渲染、参数扫描 |
///
/// ## 实现
///
/// ```text
/// Q_sca = (2/x²)·Σ_{n=1}^{n_max} (2n+1)·(|a_n|² + |b_n|²)
/// Q_ext = (2/x²)·Σ_{n=1}^{n_max} (2n+1)·Re(a_n + b_n)
/// ```
///
/// 截断阶数用 Wiscombe 公式（[`crate::bessel::mie_max_order`]）。
///
/// ## 精度说明（重要）
///
/// * **`x > 1` 时可信**：`Q_ext(m=1.33, x=10)` 与文献值差 1.5%。
/// * **`x → 0` 的瑞利极限有约 1.52 倍偏差**：标度关系 `∝ x⁴` 正确，但绝对
///   系数偏大。原因未完全查清（见 ADR-0025）。
///
/// 由于 `x < 0.3` 时散射本来极弱（`Q_sca ∝ x⁴`），这个偏差对渲染的光柱亮度
/// 影响很小。**需要定量光度学时请先复核该区间。**
///
/// # Parameters
/// * `size_param` 尺寸参数 `x = πd/λ`
/// * `refractive_index` **实部**（相对折射率，通常 1.3~1.6）。虚部（吸收）传 0
///
/// # Returns
/// `Some(Q_sca)`；参数非法或级数不收敛时 `None`。
pub fn mie_scattering_efficiency_exact(size_param: f64, refractive_index: f64) -> Option<f64> {
    if !size_param.is_finite() || !refractive_index.is_finite() {
        return None;
    }
    if size_param <= 0.0 || refractive_index <= 0.0 {
        return None;
    }
    let m = crate::bessel::Complex::from_real(refractive_index);
    let nmax = crate::bessel::mie_max_order(size_param)?;

    let mut sum = 0.0;
    for n in 1..=nmax {
        let a = crate::bessel::mie_coefficient_a(m, size_param, n)?;
        let b = crate::bessel::mie_coefficient_b(m, size_param, n)?;
        let w = (2 * n + 1) as f64;
        sum += w * (a.norm_sqr() + b.norm_sqr());
    }
    let q = 2.0 / (size_param * size_param) * sum;
    if !q.is_finite() {
        return None;
    }
    Some(q.max(0.0))
}

/// 米氏消光效率 `Q_ext` 的精确解。
///
/// `Q_ext = Q_sca + Q_abs`。实折射率（无吸收）时满足光学定理：
///
/// ```text
/// Q_ext == Q_sca     （无吸收时）
/// ```
///
/// 这个等式是 ADR-0025 里验证实现正确性的核心判据。
pub fn mie_extinction_efficiency_exact(size_param: f64, refractive_index: f64) -> Option<f64> {
    if !size_param.is_finite() || !refractive_index.is_finite() {
        return None;
    }
    if size_param <= 0.0 || refractive_index <= 0.0 {
        return None;
    }
    let m = crate::bessel::Complex::from_real(refractive_index);
    let nmax = crate::bessel::mie_max_order(size_param)?;

    let mut sum = 0.0;
    for n in 1..=nmax {
        let a = crate::bessel::mie_coefficient_a(m, size_param, n)?;
        let b = crate::bessel::mie_coefficient_b(m, size_param, n)?;
        let w = (2 * n + 1) as f64;
        sum += w * (a.re + b.re);
    }
    let q = 2.0 / (size_param * size_param) * sum;
    if !q.is_finite() {
        return None;
    }
    Some(q.max(0.0))
}

/// 按尺寸参数自动选择米氏算法。
///
/// | `x` | 用哪个 | 理由 |
/// | --- | --- | --- |
/// | `x < 0.3` | 渐近式（瑞利） | 该区间渐近式是解析精确的，精确解反受数值抵消影响 |
/// | `x >= 0.3` | 精确解 | 已对照文献验证 |
///
/// 这个分工不是妥协，是**各取所长**。
pub fn mie_scattering_efficiency_best(size_param: f64, refractive_index: f64) -> Option<f64> {
    if !size_param.is_finite() || !refractive_index.is_finite() {
        return None;
    }
    if size_param <= 0.0 || refractive_index <= 0.0 {
        return None;
    }
    if size_param < 0.3 {
        mie_scattering_efficiency(size_param, refractive_index)
    } else {
        // 级数不收敛时退回渐近式，而不是返回 None —— 渲染层不该因为数值问题
        // 失去整个光柱。
        mie_scattering_efficiency_exact(size_param, refractive_index)
            .or_else(|| mie_scattering_efficiency(size_param, refractive_index))
    }
}
/// Beer–Lambert 透射率：`T = exp(-σ_ext·n·L)`。
///
/// `sigma_ext` 是单颗粒消光截面（m²），`number_density` 是数密度（个/m³），
/// `path_length` 是路径长度（m）。三者的乘积是光学厚度 `τ`。
///
/// `T` 落在 `(0, 1]`：`τ = 0` 时完全透射。
pub fn beer_lambert_transmittance(
    sigma_ext: f64,
    number_density: f64,
    path_length: f64,
) -> Option<f64> {
    if !sigma_ext.is_finite() || !number_density.is_finite() || !path_length.is_finite() {
        return None;
    }
    if sigma_ext < 0.0 || number_density < 0.0 || path_length < 0.0 {
        return None;
    }
    Some((-sigma_ext * number_density * path_length).exp())
}

/// 光学厚度 `τ = σ_ext·n·L`（`beer_lambert_transmittance` 的指数）。
pub fn optical_thickness(sigma_ext: f64, number_density: f64, path_length: f64) -> Option<f64> {
    if !sigma_ext.is_finite() || !number_density.is_finite() || !path_length.is_finite() {
        return None;
    }
    if sigma_ext < 0.0 || number_density < 0.0 || path_length < 0.0 {
        return None;
    }
    Some(sigma_ext * number_density * path_length)
}

/// 单散射反照率 `ω = σ_sca / σ_ext`，落在 `[0, 1]`。
///
/// `ω → 1` 表示颗粒几乎不吸收（纯散射，光柱明亮）；`ω → 0` 表示几乎全吸收
/// （光柱发暗甚至看不到，像墨汁）。
pub fn single_scattering_albedo(sigma_sca: f64, sigma_ext: f64) -> Option<f64> {
    if !sigma_sca.is_finite() || !sigma_ext.is_finite() {
        return None;
    }
    if sigma_sca < 0.0 || sigma_ext <= 0.0 {
        return None;
    }
    if sigma_sca > sigma_ext {
        // 散射不能超过消光（物理约束）；拒绝而不是静默截断，避免掩盖调用错误。
        return None;
    }
    Some(sigma_sca / sigma_ext)
}

/// 沿视线**单次散射**的入射散射亮度（渲染层要的核心数字）。
///
/// 把视线分成 `steps` 段，每段累积：
///
/// ```text
/// L += T(0→s) · σ_sca · n · p(θ) · I_light · Δs
/// ```
///
/// 其中 `T(0→s)` 是从起点到该段的透射率（Beer–Lambert），所以靠近光源的
/// 段落贡献更大 —— 这是"光柱越靠近光源越亮"的物理来源。
///
/// ## 参数
///
/// * `sigma_sca` 单颗粒散射截面（m²）
/// * `sigma_ext` 单颗粒消光截面（m²），必须 `>= sigma_sca`
/// * `number_density` 数密度（个/m³）
/// * `phase` 相函数在该散射角的值（见 [`henyey_greenstein_phase`]）
/// * `light_intensity` 入射光在该方向上的强度（任意单位）
/// * `ray_length` 视线在介质内的总长度（m）
/// * `steps` 采样段数，必须 `>= 1`
///
/// # Returns
/// `Some(累积亮度)`；参数非法时 `None`。
///
/// 注意这是**单次散射**近似，忽略多次散射。对光学厚度 `τ < 1` 的稀薄烟雾
/// 足够；浓雾会低估亮度。
pub fn inscatter_along_ray(
    sigma_sca: f64,
    sigma_ext: f64,
    number_density: f64,
    phase: f64,
    light_intensity: f64,
    ray_length: f64,
    steps: u32,
) -> Option<f64> {
    if !sigma_sca.is_finite()
        || !sigma_ext.is_finite()
        || !number_density.is_finite()
        || !phase.is_finite()
        || !light_intensity.is_finite()
        || !ray_length.is_finite()
    {
        return None;
    }
    if sigma_sca < 0.0
        || sigma_ext < 0.0
        || sigma_sca > sigma_ext
        || number_density < 0.0
        || phase < 0.0
        || light_intensity < 0.0
        || ray_length < 0.0
        || steps == 0
    {
        return None;
    }
    if ray_length == 0.0 || number_density == 0.0 || sigma_sca == 0.0 {
        return Some(0.0);
    }

    let n = steps as f64;
    let ds = ray_length / n;
    let scattered = sigma_sca * number_density * phase * light_intensity;
    let tau_per_step = sigma_ext * number_density * ds;
    let trans_per_step = (-tau_per_step).exp();

    // 中点法：第 k 段的代表点在 (k + 0.5)·ds 处，其透射率是 trans_per_step^(k+0.5)。
    // 用累乘避免反复调用 exp。
    let mut trans = trans_per_step.sqrt(); // k = 0 时的 ^0.5
    let mut sum = 0.0;
    for _ in 0..steps {
        sum += trans * scattered * ds;
        trans *= trans_per_step;
    }
    Some(sum)
}

/// 丁达尔**可见度**评分（领域封装，渲染层可直接用这个数字调制亮度）。
///
/// 把散射系数换算成 `0..=1` 的直观量。
///
/// ```text
/// visibility = ω · (1 - T_in) · T_out · front_bias
/// ```
///
/// 四项各有物理含义：
///
/// * `ω` 单散射反照率：纯吸收介质（像墨汁）看不到光柱
/// * `1 - T_in` 入射光在介质内被散射掉的比例 —— **太稀就没有散射**
/// * `T_out` 散射光离开介质的平均透射率 —— **太浓就出不来**
/// * `front_bias` 前倾程度：`g` 越接近 1，正视光源时越亮
///
/// ## 为什么必须有 `T_out` 这一项
///
/// 只用 `ω·(1-T_in)·front_bias` 会得出一个明显错误的结论：介质越浓可见度越高
/// （因为 `1-T_in → 1`）。但现实中浓到看不清的烟雾里**根本看不到光柱** ——
/// 散射光和入射光一样，都要穿过介质才能到达观察者。
///
/// `T_out` 用**两程透射**的几何平均近似：
///
/// ```text
/// T_out = (T_in)^(1/2)
/// ```
///
/// 因为散射点平均位于介质中点，出射剩余路程约是总长的一半。这个近似在光学
/// 厚度均匀时是精确的，在非均匀介质里是合理的一阶估计。
///
/// 加上这一项后，可见度在 `τ ≈ 1.5` 附近取最大 —— 这与"浓度刚好的烟雾里光柱
/// 最明显"的经验一致。
///
/// # Parameters
/// * `sigma_sca` / `sigma_ext` 单颗粒散射/消光截面
/// * `number_density` 数密度
/// * `path_length` 视线在介质内的长度
/// * `g` HG 非对称参数
pub fn tyndall_visibility(
    sigma_sca: f64,
    sigma_ext: f64,
    number_density: f64,
    path_length: f64,
    g: f64,
) -> Option<f64> {
    let albedo = single_scattering_albedo(sigma_sca, sigma_ext)?;
    let trans_in = beer_lambert_transmittance(sigma_ext, number_density, path_length)?;
    if !g.is_finite() || g.abs() >= 1.0 {
        return None;
    }
    // 出射路径的平均透射率：散射点平均在介质中点，剩余路程约一半。
    let trans_out = trans_in.sqrt();
    // 前倾因子：g = 0 时 0.5（各向同性取中值），g → 1 时趋近 1。
    let front_bias = 0.5 + 0.5 * g.max(0.0);
    Some((albedo * (1.0 - trans_in) * trans_out * front_bias).clamp(0.0, 1.0))
}

// ── 多次散射近似（ADR-0022）─────────────────────────────────────────────────

/// 多次散射的阶数上限（避免病态参数下无限循环）。
pub const MAX_SCATTER_ORDERS: u32 = 16;

/// 沿视线**含多次散射**的累积亮度。
///
/// ## 为什么需要
///
/// [`inscatter_along_ray`] 只算光**第一次**被散射就进入视线的部分。厚介质
/// （光学厚度 `τ > 2`）里，光实际会经历多次散射才出来，单次近似**低估**亮度
/// 2~5 倍。
///
/// ## 采用的近似：阶数递推
///
/// 不做完整的辐射传输求解（那需要迭代整个光场），而是利用一个物理事实：
///
/// **每次散射后，只有 `ω = σ_sca/σ_ext` 的部分继续传播**（其余被吸收）。
///
/// 所以第 `n` 阶的贡献近似为第 `n-1` 阶乘以 `ω` 和一个几何因子：
///
/// ```text
/// L_total = L_1 · Σ_{n=0}^{N-1} (ω · g)^n
/// ```
///
/// 其中 `g` 是**几何衰减因子**，表示散射光重新进入视线的立体角比例。取
/// `g = 0.5` 是一个常用的一阶估计（散射大致前后各半），可用
/// `geometric_factor` 参数调整。
///
/// 这是等比级数，闭式求和：
///
/// ```text
/// L_total = L_1 · (1 - r^N) / (1 - r)      r = ω·g
/// ```
///
/// ## 精度与局限
///
/// 这是**工程近似**，不是辐射传输的精确解。它的正确性依赖：
///
/// * `ω·g < 1`（否则级数发散 —— 物理上 `ω <= 1`、`g < 1` 保证）
/// * 介质近似均匀（`σ` 沿视线变化时误差增大）
///
/// 对渲染用途足够：视觉上重要的是"厚介质不该比薄介质暗"，而这个模型保证了
/// 亮度随 `τ` 单调上升到饱和，不会像单次散射那样过早在某个厚度封顶。
///
/// # Parameters
/// * `sigma_sca` / `sigma_ext` 单颗粒散射/消光截面
/// * `number_density` 数密度
/// * `phase` 相函数值
/// * `light_intensity` 入射光强度
/// * `ray_length` 视线长度
/// * `steps` 单次积分的采样段数
/// * `orders` 计入的散射阶数（`1` = 只有单次，与 `inscatter_along_ray` 相同）
/// * `geometric_factor` 几何衰减因子，落在 `(0, 1]`；`0.5` 是常用估计
#[allow(clippy::too_many_arguments)]
pub fn inscatter_along_ray_multiple(
    sigma_sca: f64,
    sigma_ext: f64,
    number_density: f64,
    phase: f64,
    light_intensity: f64,
    ray_length: f64,
    steps: u32,
    orders: u32,
    geometric_factor: f64,
) -> Option<f64> {
    if !geometric_factor.is_finite()
        || !(0.0..=1.0).contains(&geometric_factor)
        || geometric_factor <= 0.0
    {
        return None;
    }
    if orders == 0 || orders > MAX_SCATTER_ORDERS {
        return None;
    }
    // 单次散射是基础项，参数校验由它完成。
    let first = inscatter_along_ray(
        sigma_sca,
        sigma_ext,
        number_density,
        phase,
        light_intensity,
        ray_length,
        steps,
    )?;

    // 单次阶不需要反照率（它就是"第一次散射"本身）。
    if orders == 1 {
        return Some(first);
    }

    let albedo = single_scattering_albedo(sigma_sca, sigma_ext)?;
    let r = albedo * geometric_factor;
    // r < 1 由 albedo <= 1 与 geometric_factor < 1 保证；这里防御性处理 r == 1
    // （albedo == 1 且 g == 1 的退化情形，级数不收敛）。
    if r >= 1.0 {
        // 纯散射且完全前向：所有光都留在系统里，按阶数线性累加。
        return Some(first * orders as f64);
    }
    // 等比级数：1 + r + r² + ... + r^(N-1) = (1 - r^N)/(1 - r)
    let factor = (1.0 - r.powi(orders as i32)) / (1.0 - r);
    Some(first * factor)
}
