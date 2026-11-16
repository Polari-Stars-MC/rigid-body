// ── 粒子 → 体积散射的耦合（ADR-0019）────────────────────────────────────────
//
// 把 SPH 流体 / 颗粒介质的**粒子分布**转成光学量，让光柱随烟雾/尘埃流动而
// 变化，而不是静态贴图。
//
// ## 数据流
//
// ```text
// rapier FluidWorld.particles   (pos, mass, density)     ← SPH 求解产出
// rapier GranularWorld.particles(pos, mass, radius)      ← DEM 求解产出
//          ↓ 本模块
// 沿线采样的局部数密度 n(s)
//          ↓ tyndall::inscatter_along_ray
// 累积散射亮度 L
// ```
//
// ## 为什么数密度要从粒子分布重新算
//
// `FluidParticle.density` 是 SPH 的**质量密度**（kg/m³，Poly6 核求和），而散射
// 需要的是**数密度**（个/m³）。两者差一个单颗粒质量：
//
// ```text
// n = ρ / m_particle
// ```
//
// 但 SPH 的 `density` 是**在粒子位置上**的核加权密度，直接用它在任意点上采样
// 会有偏（粒子之间是空的）。所以这里提供一个**按位置查询的核加权数密度**：
// 对查询点附近的粒子做距离加权求和，权重用 Poly6 核。
//
// ## 一个必须说清的取舍
//
// 真实的 SPH 粒子是**离散点**，它们之间的空间是空的。如果严格按粒子算，
// 光柱会呈现"颗粒状斑点"而不是连续光柱。
//
// 现实里的烟雾之所以是连续介质，是因为粒子数极多（10⁶ 以上）。渲染时通常
// 用**核平滑**把离散粒子抹成连续密度场 —— 本模块提供 `sample_number_density`
// 就是这个用途。

/// 单个粒子的光学贡献参数。
///
/// 把"一个粒子有多大、多能散射"集中到一处，供 [`sample_number_density`] 与
/// [`inscatter_through_particles`] 使用。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParticleOptics {
    /// 单颗粒散射截面 `σ_sca`（m²）。
    pub sigma_sca: f64,
    /// 单颗粒消光截面 `σ_ext`（m²），必须 `>= sigma_sca`。
    pub sigma_ext: f64,
    /// 单颗粒质量（kg），用于把 SPH 质量密度换算成数密度。
    pub mass: f64,
}

impl ParticleOptics {
    /// 校验：`sigma_ext >= sigma_sca >= 0`、`mass > 0`，全为有限值。
    pub fn is_valid(&self) -> bool {
        self.sigma_sca.is_finite()
            && self.sigma_ext.is_finite()
            && self.mass.is_finite()
            && self.sigma_sca >= 0.0
            && self.sigma_ext >= self.sigma_sca
            && self.mass > 0.0
    }
}

/// Poly6 核函数（SPH 标准核，用于把离散粒子抹成连续密度）。
///
/// `W(r, h) = (315 / (64π h⁹)) · (h² - r²)³`，`r < h` 时；否则 `0`。
///
/// 选 Poly6 是因为**项目已有同款实现**（`fluid_sph_poly6_kernel`），保持物理
/// 口径一致比换一个更"适合渲染"的核更重要 —— 否则同一个烟雾在求解器和渲染器
/// 里的密度分布会对不上。
pub fn poly6_kernel(distance: f64, smoothing_radius: f64) -> Option<f64> {
    if !distance.is_finite() || !smoothing_radius.is_finite() {
        return None;
    }
    if distance < 0.0 || smoothing_radius <= 0.0 {
        return None;
    }
    let h = smoothing_radius;
    if distance >= h {
        return Some(0.0);
    }
    let h2 = h * h;
    let r2 = distance * distance;
    let diff = h2 - r2;
    Some(315.0 / (64.0 * std::f64::consts::PI * h.powi(9)) * diff.powi(3))
}

/// 在查询点处按核加权估计**数密度**（个/m³）。
///
/// 对 `positions` 里每个粒子求 `W(|p - query|, h)`，求和后乘以总粒子数：
///
/// ```text
/// n(query) = Σ_i W(|p_i - query|, h)
/// ```
///
/// 注意这里**不需要除以质量** —— Poly6 核已归一化，`Σ W` 直接给出"单位体积内
/// 有多少个粒子"的估计。若用 SPH 质量密度 `ρ`，则 `n = ρ / m_particle`。
///
/// # Parameters
/// * `positions` 粒子位置，`[(x, y, z); N]` 形式
/// * `query` 查询点
/// * `smoothing_radius` 核半径 `h`
///
/// # Returns
/// `Some(n)`；`h <= 0` 或含非有限值时 `None`。
pub fn sample_number_density(
    positions: &[[f64; 3]],
    query: [f64; 3],
    smoothing_radius: f64,
) -> Option<f64> {
    if !smoothing_radius.is_finite() || smoothing_radius <= 0.0 {
        return None;
    }
    if !query.iter().all(|v| v.is_finite()) {
        return None;
    }
    let mut sum = 0.0;
    for p in positions {
        if !p.iter().all(|v| v.is_finite()) {
            continue;
        }
        let dx = p[0] - query[0];
        let dy = p[1] - query[1];
        let dz = p[2] - query[2];
        let r = (dx * dx + dy * dy + dz * dz).sqrt();
        sum += poly6_kernel(r, smoothing_radius)?;
    }
    Some(sum)
}

/// 把 SPH 的**质量密度**换算成**数密度**。
///
/// ```text
/// n = ρ / m_particle
/// ```
///
/// `FluidParticle.density` 就是 `ρ`（Poly6 核加权质量密度）。这个换算是
/// [`sample_number_density`] 的替代路径：如果宿主直接读求解器算好的
/// `density`，用这个函数比重新扫粒子快得多（O(1) vs O(N)）。
///
/// # Returns
/// `Some(n)`；`mass <= 0` 或非有限时 `None`。
pub fn number_density_from_mass_density(mass_density: f64, particle_mass: f64) -> Option<f64> {
    if !mass_density.is_finite() || !particle_mass.is_finite() {
        return None;
    }
    if mass_density < 0.0 || particle_mass <= 0.0 {
        return None;
    }
    Some(mass_density / particle_mass)
}

/// 沿视线穿过粒子云的单次散射累积亮度。
///
/// 在 `[origin, origin + direction·length]` 上均匀取 `steps` 个采样点，每个点用
/// [`sample_number_density`] 求局部数密度，再按 Beer–Lambert 累积：
///
/// ```text
/// L = Σ_k  T(s_k) · σ_sca · n(s_k) · p(θ) · I_light · Δs
/// T(s_k) = exp(-σ_ext · Σ_{j<k} n(s_j) · Δs)
/// ```
///
/// 这与 [`crate::tyndall::inscatter_along_ray`] 的区别：后者的数密度是**常数**，
/// 这里每步都从粒子分布**重新采样**，所以能表现烟雾的浓淡变化。
///
/// # Parameters
/// * `positions` / `optics` 粒子云
/// * `origin` 视线起点
/// * `direction` 视线方向（**必须已归一化**）
/// * `length` 视线长度
/// * `smoothing_radius` 核半径 `h`
/// * `phase` 相函数在该散射角的值
/// * `light_intensity` 入射光强度
/// * `steps` 采样段数，`>= 1`
///
/// # Returns
/// `Some(亮度)`；参数非法时 `None`。
#[allow(clippy::too_many_arguments)]
pub fn inscatter_through_particles(
    positions: &[[f64; 3]],
    optics: ParticleOptics,
    origin: [f64; 3],
    direction: [f64; 3],
    length: f64,
    smoothing_radius: f64,
    phase: f64,
    light_intensity: f64,
    steps: u32,
) -> Option<f64> {
    if !optics.is_valid() {
        return None;
    }
    if !origin.iter().all(|v| v.is_finite())
        || !direction.iter().all(|v| v.is_finite())
        || !length.is_finite()
        || !phase.is_finite()
        || !light_intensity.is_finite()
    {
        return None;
    }
    if length < 0.0 || phase < 0.0 || light_intensity < 0.0 || steps == 0 {
        return None;
    }
    // 方向必须归一化：否则 Δs 的物理长度错了，整条积分都失真。
    let norm =
        (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2])
            .sqrt();
    if (norm - 1.0).abs() > 1e-6 {
        return None;
    }
    if length == 0.0 || positions.is_empty() {
        return Some(0.0);
    }

    let n_steps = steps as f64;
    let ds = length / n_steps;
    let scattered = optics.sigma_sca * phase * light_intensity;
    let mut trans = 1.0;
    let mut sum = 0.0;

    for k in 0..steps {
        // 段中点采样，与 `inscatter_along_ray` 的中点法一致。
        let t = (k as f64 + 0.5) * ds;
        let q = [
            origin[0] + direction[0] * t,
            origin[1] + direction[1] * t,
            origin[2] + direction[2] * t,
        ];
        let n = sample_number_density(positions, q, smoothing_radius)?;

        // Beer–Lambert：先乘透射率再累加本段贡献。
        sum += trans * n * scattered * ds;
        // 本段造成的衰减。
        trans *= (-optics.sigma_ext * n * ds).exp();
        // 完全衰减后后面的段落不再有贡献，提前退出（省 O(N·steps) 的采样）。
        if trans < 1e-12 {
            break;
        }
    }
    Some(sum)
}

/// 估算粒子云的**包围盒**（渲染层做视锥剔除用）。
///
/// # Returns
/// `Some((min, max))`；粒子为空或含非有限值不足以构成包围盒时 `None`。
pub fn particle_cloud_bounds(positions: &[[f64; 3]]) -> Option<([f64; 3], [f64; 3])> {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    let mut any = false;
    for p in positions {
        if !p.iter().all(|v| v.is_finite()) {
            continue;
        }
        any = true;
        for i in 0..3 {
            if p[i] < min[i] {
                min[i] = p[i];
            }
            if p[i] > max[i] {
                max[i] = p[i];
            }
        }
    }
    if any { Some((min, max)) } else { None }
}
// ── 空间加速：均匀网格（ADR-0023）───────────────────────────────────────────
//
// `sample_number_density` 是 O(N) 的：每个查询点扫全部粒子。在
// `inscatter_through_particles` 里被调用 `steps` 次，总复杂度 `O(steps·N)`。
//
// 对 `steps = 128`、`N = 10⁶` 是 1.28 亿次核求值/帧，不可接受。
//
// ## 为什么用均匀网格而不是 k-d tree / BVH
//
// SPH 的核半径 `h` 是**常数**，所有查询都是"固定半径邻域"。这个前提下均匀
// 网格是最优结构：
//
// * 建网格 `O(N)`
// * 查询只扫所在格 + 26 个邻居
// * k-d tree 需要 `O(N log N)` 建树且每帧重建，对固定半径反而更慢
//
// ## 格子尺寸的选择
//
// `cell_size = h`：查询点最多跨 2 个格，所以只需扫 3×3×3 = 27 个邻居。
// 如果 `cell_size < h`，邻居数会增加；如果 `> h`，单格粒子数会增加。`= h`
// 是平衡点。

/// 均匀网格索引。
///
/// 由 [`UniformGrid::build`] 从粒子位置构建，随后可用 [`UniformGrid::number_density`]
/// 做 `O(k)` 查询。
pub struct UniformGrid {
    /// 格子边长（取核半径 `h`）。
    cell_size: f64,
    /// 每个格子的粒子索引。
    cells: std::collections::HashMap<(i64, i64, i64), Vec<u32>>,
    /// 粒子位置（与索引对齐）。
    positions: Vec<[f64; 3]>,
    /// 参与建网格的有效粒子数。
    count: usize,
}

impl UniformGrid {
    /// 从粒子位置构建网格。
    ///
    /// `smoothing_radius` 作为格子边长。非有限位置的粒子被**跳过**（与
    /// `sample_number_density` 的容错行为一致），但仍保留在 `positions` 里以
    /// 保持索引对齐 —— 它们只是永远不会被查询命中。
    ///
    /// # Returns
    /// `Some(grid)`；`h <= 0` 或非有限时 `None`。
    pub fn build(positions: &[[f64; 3]], smoothing_radius: f64) -> Option<Self> {
        if !smoothing_radius.is_finite() || smoothing_radius <= 0.0 {
            return None;
        }
        let mut cells: std::collections::HashMap<(i64, i64, i64), Vec<u32>> =
            std::collections::HashMap::new();
        let mut count = 0usize;
        for (i, p) in positions.iter().enumerate() {
            if !p.iter().all(|v| v.is_finite()) {
                continue;
            }
            let key = Self::cell_of(*p, smoothing_radius);
            cells.entry(key).or_default().push(i as u32);
            count += 1;
        }
        Some(Self {
            cell_size: smoothing_radius,
            cells,
            positions: positions.to_vec(),
            count,
        })
    }

    /// 有效粒子数（跳过非有限的）。
    pub fn len(&self) -> usize {
        self.count
    }

    /// 是否为空。
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// 格子数（用于观察粒子分布的离散程度）。
    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }

    fn cell_of(p: [f64; 3], h: f64) -> (i64, i64, i64) {
        (
            (p[0] / h).floor() as i64,
            (p[1] / h).floor() as i64,
            (p[2] / h).floor() as i64,
        )
    }

    /// 在查询点处按核加权估计数密度（`O(k)`，`k` 为邻域粒子数）。
    ///
    /// 与 [`sample_number_density`] **逐位相同**（同样的 Poly6 核、同样的跳过
    /// 非有限粒子规则），只是查询范围从全部粒子缩到 27 个邻居格。
    pub fn number_density(&self, query: [f64; 3], smoothing_radius: f64) -> Option<f64> {
        if !smoothing_radius.is_finite() || smoothing_radius <= 0.0 {
            return None;
        }
        if !query.iter().all(|v| v.is_finite()) {
            return None;
        }
        // 格子尺寸必须与建网格时一致，否则邻居偏移会算错。
        if (smoothing_radius - self.cell_size).abs() > 1e-12 {
            return None;
        }
        let (cx, cy, cz) = Self::cell_of(query, self.cell_size);
        let mut sum = 0.0;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let key = (cx + dx, cy + dy, cz + dz);
                    let Some(bucket) = self.cells.get(&key) else {
                        continue;
                    };
                    for &idx in bucket {
                        let p = self.positions[idx as usize];
                        let ex = p[0] - query[0];
                        let ey = p[1] - query[1];
                        let ez = p[2] - query[2];
                        let r = (ex * ex + ey * ey + ez * ez).sqrt();
                        sum += poly6_kernel(r, smoothing_radius)?;
                    }
                }
            }
        }
        Some(sum)
    }
}

/// 沿视线穿过粒子云的分通道/标量无关的**加速版**单次散射积分。
///
/// 与 [`inscatter_through_particles`] 同一个物理模型，但用 [`UniformGrid`] 把
/// 每步的密度查询从 `O(N)` 降到 `O(k)`。
///
/// 总复杂度 `O(N + steps·k)` 而不是 `O(steps·N)`。
#[allow(clippy::too_many_arguments)]
pub fn inscatter_through_particles_accelerated(
    positions: &[[f64; 3]],
    optics: ParticleOptics,
    origin: [f64; 3],
    direction: [f64; 3],
    length: f64,
    smoothing_radius: f64,
    phase: f64,
    light_intensity: f64,
    steps: u32,
) -> Option<f64> {
    // 粒子少的时候建网格不划算（HashMap 开销 > 直接扫）。
    // 经验阈值：单步要扫的粒子数少于 64 时直接走朴素路径。
    const GRID_THRESHOLD: usize = 64;
    if positions.len() < GRID_THRESHOLD {
        return inscatter_through_particles(
            positions,
            optics,
            origin,
            direction,
            length,
            smoothing_radius,
            phase,
            light_intensity,
            steps,
        );
    }

    let grid = UniformGrid::build(positions, smoothing_radius)?;
    inscatter_with_sampler(
        &grid,
        optics,
        origin,
        direction,
        length,
        smoothing_radius,
        phase,
        light_intensity,
        steps,
    )
}

/// 用给定的网格做积分。抽出来让加速版与朴素版共享累积逻辑（测试里也用它验证
/// 两条路径一致）。
#[allow(clippy::too_many_arguments)]
fn inscatter_with_sampler(
    grid: &UniformGrid,
    optics: ParticleOptics,
    origin: [f64; 3],
    direction: [f64; 3],
    length: f64,
    smoothing_radius: f64,
    phase: f64,
    light_intensity: f64,
    steps: u32,
) -> Option<f64> {
    if !optics.is_valid() {
        return None;
    }
    if !origin.iter().all(|v| v.is_finite())
        || !direction.iter().all(|v| v.is_finite())
        || !length.is_finite()
        || !phase.is_finite()
        || !light_intensity.is_finite()
    {
        return None;
    }
    if length < 0.0 || phase < 0.0 || light_intensity < 0.0 || steps == 0 {
        return None;
    }
    let norm =
        (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2])
            .sqrt();
    if (norm - 1.0).abs() > 1e-6 {
        return None;
    }
    if length == 0.0 || grid.is_empty() {
        return Some(0.0);
    }

    let n_steps = steps as f64;
    let ds = length / n_steps;
    let scattered = optics.sigma_sca * phase * light_intensity;
    let mut trans = 1.0;
    let mut sum = 0.0;
    for k in 0..steps {
        let t = (k as f64 + 0.5) * ds;
        let q = [
            origin[0] + direction[0] * t,
            origin[1] + direction[1] * t,
            origin[2] + direction[2] * t,
        ];
        let n = grid.number_density(q, smoothing_radius)?;
        sum += trans * n * scattered * ds;
        trans *= (-optics.sigma_ext * n * ds).exp();
        if trans < 1e-12 {
            break;
        }
    }
    Some(sum)
}
