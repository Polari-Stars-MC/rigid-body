//! 天体重力与 n-body 互引力计算。
//!
//! `celestial_acceleration` 复刻旧 `mps-core` 里的自适应天体重力模型分支
//! （<2R 椭球、<10R 球谐、<100R J2–J6 带谐、>100R 点质量），但作为独立实现
//! 直接调用 `mps_formula::gravitational_models`，不依赖 `mps-core`。
//! 原始 `CelestialGravityForceLaw` 已随 `world_register_celestial_gravity`
//! 一并从 `mps-core` 移除——天体重力现由本 crate 的 `CosmosWorld::add_celestial`
//! 承担。
//!
//! `n_body_acceleration` 实现经典 n-body 互引力，可加 softening 防奇点。

use mps_formula::celestial_data::{CelestialBody, G};
use mps_formula::ffi::Vec3;
use mps_formula::gravitational_models::{
    ellipsoid_gravity, spherical_harmonics_acceleration, zonal_harmonics_acceleration,
};
use rapier3d::prelude::{RigidBodyHandle, Rotation, Vector};

/// 一个注册到世界中的天体引力源。
///
/// 硬上限：带谐展开最多 J2..J6 五项，存栈上即可，避免每子步每体每源 `vec!`。
/// P1.11 起在 `CelestialSource::new` 用作预过滤缓冲长度。
const MAX_ZONAL: usize = 5;

/// `max_sh_degree` 限制球谐展开的最高阶；实际还会被 `body.max_degree` 约束。
#[derive(Clone, Copy)]
pub struct CelestialSource {
    pub body: &'static CelestialBody,
    /// 天体质心的世界坐标（米）。历史行为：锚定世界原点（`ZERO`）。浮动原点
    /// 重定心（`CosmosWorld::recenter`）会随刚体一起平移此位置——引力只依赖
    /// 相对位矢 `position - self.position`，整体平移下物理不变。
    pub position: Vector,
    /// 球谐模型最高阶（受 `body.max_degree` 上限约束）。
    pub max_sh_degree: u32,
    /// 是否启用本天体的引力（关闭则贡献为 0）。
    pub enabled: bool,
    /// P1.11: 预过滤的 J2..J6 带谐系数（仅非零项），供 `celestial_acceleration`
    /// 的 `10R..100R` 分支直接 `&zonal[..n]` 取用，避免每次调用都构造
    /// `[f64; 5]` + filter。保留 j2..j6 的原始出现序；`j_zonal_n` 是写入项数。
    /// 多数天体只有 j2 ≠ 0 → `n=1, zonals=[j2, 0, 0, 0, 0]`。
    pub j_zonal_zonals: [f64; MAX_ZONAL],
    /// 预过滤 J 系数的有效长度（≤ MAX_ZONAL）。
    pub j_zonal_n: u8,
}

impl CelestialSource {
    pub fn new(body: &'static CelestialBody, max_sh_degree: u32) -> Self {
        // P1.11: 一次性把 body 的 j2..j6 过滤掉零项，填入 `j_zonal_zonals[..n]`。
        // 旧 `celestial_acceleration` 每次调用都要做这件事，现在下沉到构造期。
        let raw = [body.j2, body.j3, body.j4, body.j5, body.j6];
        let mut zonal = [0.0_f64; MAX_ZONAL];
        let mut n = 0u8;
        for j in raw.iter().copied() {
            if j != 0.0 {
                zonal[n as usize] = j;
                n += 1;
            }
        }
        Self {
            body,
            position: Vector::ZERO, // 历史行为：天体锚定世界原点
            max_sh_degree,
            enabled: true,
            j_zonal_zonals: zonal,
            j_zonal_n: n,
        }
    }
}

/// 一个参与 n-body 互引力的质点源。
///
/// 对应场景中一个已插入的刚体（动态/固定均可）：`handle` 指向其在
/// `RigidBodySet` 中的句柄，`gm = G·mass` 是**总引力参数**——远场（|d| ≫ 源尺寸）下
/// 把该星体看成位于其质心的单个质点即可（牛测定理：外部引力等价于总质量在质心
/// 的点质量），因此 `gm` 同时是快路径的 monopole 强度。
///
/// **不规则质量分布**：星球并非球体（土豆星、双瓣小行星、扁椭球）时，远场 monopole
/// 无法捕获近场非对称拉扯。`points: Vec<MassPoint>` 以离散质点表达延展质量分布，
/// 每个点带本体局部坐标偏移 `local_offset` 与自身引力参数 `gm`；源刚体的姿态
/// `body.rotation()` 把这些局部点变到世界坐标，临近质点（|d| ≤ `near_field_factor ·
/// bounding_radius`）时按 `a = G·Σ mᵢ·dᵢ/|dᵢ|³` 累加，而不是单一 monopole。
///
/// `points` 为空时退化为纯 monopole（与历史行为完全一致，向后兼容）；`bounding_radius`
/// 给 0 时禁用近场分支（恒走 monopole 快路径）。
///
/// `n_body_acceleration` 与 `CosmosWorld::apply_forces`/`total_acceleration` 的近/远场
/// 选择见 `near_field_factor` 的文档；受力刚体通过 `exclude_handle` 跳过自身，避免
/// 自吸引奇点。
#[derive(Clone)]
pub struct NBodySource {
    pub handle: RigidBodyHandle,
    /// 源总引力参数 `gm = G·mass`。远场（|d| > `near_field_factor·bounding_radius`）
    /// 与 `points` 为空时都用它当单质点算。
    pub gm: f64,
    /// 不规则质量分布的离散质点（本体局部坐标）。空 → 纯 monopole（向后兼容）。
    /// 各点 `gm` 之和不必等于 `gm`——近场走 Σ 自洽，远场走 `gm` 自洽，二者只在
    /// `bounding_radius = 0` 或 `points` 为空时才会"塌缩到一点"一起被用到。
    pub points: Vec<MassPoint>,
    /// 质点分布的边界球半径（世界米）。|d| > `near_field_factor · bounding_radius`
    /// 切回 monopole 快路径；≤ 该阈值时走质点求和。`0` 表示不启用近场分支。
    pub bounding_radius: f64,
    /// 近场阈值倍率（无量纲），`0` 则用模块默认 `NEAR_FIELD_FACTOR`（=8）。
    /// 调大→把质点求和推得更远、精度高但每体计算量更大；调小→更多走快路径、
    /// 大规模场景更省。每个源可单独设（一个不规则小行星需精细、其它星体走默认）。
    pub near_field_factor: f64,
}

/// 一个离散质量点的本体局部坐标描述。
///
/// `local_offset` 在**源刚体的本体局部坐标系**（自转刚体的姿态会带着它一起转）；
/// `gm = G · point_mass` 是该点自身的引力参数。近场累加时由源姿态 `rotation()` 变到
/// 世界坐标后做 `1/r²` 求和。
#[derive(Clone, Copy, Debug)]
pub struct MassPoint {
    /// 本体局部坐标偏移（米），相对源刚体的质心。
    pub local_offset: Vector,
    /// 该点的引力参数 `G·mᵢ`。
    pub gm: f64,
}

/// 默认近场阈值倍率：|d| ≤ 8·bounding_radius 时走质点求和。8 给到 r² 误差 ~1.5%
/// 的 monopole，足够典型的薄壳/扁平分布过渡到 monopole。
pub const NEAR_FIELD_FACTOR: f64 = 8.0;

impl NBodySource {
    /// 构造一个**纯 monopole** 源（无离散质点），与历史 `NBodySource { handle, gm }`
    /// 等价。`points` 空、`bounding_radius=0` → 永远走 monopole 快路径。
    pub fn monopole(handle: RigidBodyHandle, gm: f64) -> Self {
        Self {
            handle,
            gm,
            points: Vec::new(),
            bounding_radius: 0.0,
            near_field_factor: 0.0,
        }
    }

    /// 构造一个**不规则质量分布**源：`total_mass` 是总质量（用于远场 gm），`points`
    /// 是本体局部坐标下的离散质量点（各自带 `gm=G·mᵢ`），`bounding_radius` 是这些
    /// 点分布的边界球半径。
    pub fn irregular(
        handle: RigidBodyHandle,
        total_gm: f64,
        points: Vec<MassPoint>,
        bounding_radius: f64,
    ) -> Self {
        Self {
            handle,
            gm: total_gm,
            points,
            bounding_radius,
            near_field_factor: 0.0, // 0 → 用 NEAR_FIELD_FACTOR 默认
        }
    }

    /// 当前生效的近场阈值（米）：`near_field_factor` 给 0 时回退到
    /// [`NEAR_FIELD_FACTOR`]，再乘 `bounding_radius`。`bounding_radius=0` → 阈值 0，
    /// 永远走 monopole。
    #[inline]
    pub fn near_field_threshold(&self) -> f64 {
        let factor = if self.near_field_factor > 0.0 {
            self.near_field_factor
        } else {
            NEAR_FIELD_FACTOR
        };
        factor * self.bounding_radius
    }
}

/// 自适应选择最匹配的引力模型并返回在该 `position` 处的引力加速度。
///
/// 选择规则（沿用旧 `mps-core` 的自适应天体重力分支）：
///
/// | 归一化高度 r/R_eq | 椭率>0 且有 SH | 选用的模型 |
/// |---|---|---|
/// | <2   | 椭率>0       | 椭球 `ellipsoid_gravity` |
/// | 2–10 | max_sh≥2 且有 SH 系数 | 球谐 `spherical_harmonics_acceleration` |
/// | 10–100 | 任意       | J2–J6 带谐 `zonal_harmonics_acceleration` |
/// | >100 | 任意         | 点质量 + J2 |
#[inline]
pub fn celestial_acceleration(position: Vector, source: &CelestialSource) -> Vector {
    if !source.enabled {
        return Vector::ZERO;
    }

    // 相对位矢：天体质心不再硬编码世界原点（浮动原点重定心会平移源位置）。
    let position = position - source.position;
    let gm = source.body.gm;
    let r_eq = source.body.equatorial_radius;
    if r_eq <= 0.0 {
        // 退化天体：点质量
        return point_mass_acceleration(position, gm);
    }

    let r = position.length();
    if r < 1.0 {
        return Vector::ZERO; // 位于天体内部母点：无意义
    }

    let normalized_altitude = r / r_eq;
    let pos = Vec3 {
        x: position.x,
        y: position.y,
        z: position.z,
    };

    let accel = if normalized_altitude < 2.0 && source.body.flattening > 0.0 {
        ellipsoid_gravity(pos, source.body)
    } else if normalized_altitude < 10.0
        && source.max_sh_degree >= 2
        && !source.body.c_coeffs.is_empty()
    {
        spherical_harmonics_acceleration(pos, source.body, source.max_sh_degree)
    } else if normalized_altitude < 100.0 {
        // P1.11: J2..J6 过滤改读 `source.j_zonal_zonals[..j_zonal_n]`（`new()` 时
        // 预计算）而非每次调用都重建 `[f64; 5]` + filter。FR8 每子步 15 次 accel
        // 评估 × N 个天体 × N 个体 → 累计每步上千次调用，省掉若干栈写。
        zonal_harmonics_acceleration(
            pos,
            gm,
            r_eq,
            &source.j_zonal_zonals[..source.j_zonal_n as usize],
        )
    } else {
        // >100R：点质量 + J2。本分支只用 j2（忽略 j3..j6），无堆分配。
        // 注：不能用 `source.j_zonal_zonals[..]`，因为预过滤数组在 j2=0 时会把
        // 后续非零 jn 顶到首项（new() 实现只过滤了"非零"序，保留了 j2..j6 的出现序）；
        // 此分支严格只想要"j2 或 空集"。
        let j2 = source.body.j2;
        let buf = [j2];
        let slice = if j2 != 0.0 { &buf[..] } else { &[][..] };
        zonal_harmonics_acceleration(pos, gm, r_eq, slice)
    };

    Vector::new(accel.x, accel.y, accel.z)
}

/// 点质量引力加速度 `a = -GM · r̂ / r²`。
#[inline]
pub fn point_mass_acceleration(position: Vector, gm: f64) -> Vector {
    let r2 = position.length_squared();
    if r2 < 1.0 {
        return Vector::ZERO;
    }
    let r = r2.sqrt();
    -position * (gm / (r2 * r))
}

/// 对位于 `position`（世界坐标）的质点，求所有 n-body 源贡献的互引力加速度。
///
/// 跳过 `exclude_handle`（通常是质点自身对应的刚体），以避免自吸引奇点。
/// `softening` 平方项加在分母上防止两体无限接近时发散（典型取几公里量级
/// 的平方，0 表示不加 soften）。
///
/// `source_positions` / `source_rotations` 是两张以 arena index 为下标的快照表
/// （由 `integrator::snapshot_source_positions` + `refresh_n_body_sources` 共同
/// 构造），分别给出源刚体质心位置与姿态；按 `RigidBodyHandle` 取对应源快照。
/// 保留 `Fn` 闭包形态以便内部 `O(1)` 索引并避免重复传参。
///
/// **就地支持不规则质量分布**：对携带 `points: Vec<MassPoint>` 的源，近距离（距质心
/// ≤ `src.near_field_threshold()`）时按 `a = Σ G·mᵢ·dᵢ/|dᵢ|³` 求和（每个点的世界位置
/// = 源姿态旋转其 `local_offset` 后加源质心）；远距离或无 `points` 的源退化为单点
/// monopole（与历史行为一致）。
#[inline]
pub fn n_body_acceleration(
    position: Vector,
    sources: &[NBodySource],
    exclude_handle: RigidBodyHandle,
    source_positions: impl Fn(RigidBodyHandle) -> Vector,
    source_rotations: impl Fn(RigidBodyHandle) -> Rotation,
    softening_sq: f64,
) -> Vector {
    let mut acc = Vector::ZERO;
    for src in sources {
        if src.handle == exclude_handle || src.gm <= 0.0 {
            continue;
        }
        let r_j = source_positions(src.handle);
        let d = r_j - position;
        let dist_sq = d.length_squared() + softening_sq;
        if dist_sq < 1.0 {
            continue;
        }
        let dist = dist_sq.sqrt();
        // 近场不规则分支：距源质心 ≤ near_field_threshold 且源有离散质点时，
        // 按 Σ G·mᵢ·dᵢ/|dᵢ|³ 求和（捕获非球分布的方向性拉扯）。
        let near_threshold = src.near_field_threshold();
        if !src.points.is_empty() && near_threshold > 0.0 && dist <= near_threshold {
            let rot = source_rotations(src.handle);
            let mut acc_local = Vector::ZERO;
            for mp in &src.points {
                if mp.gm <= 0.0 {
                    continue;
                }
                // 本体质点的世界坐标 = 源质心 + 姿态旋转 × 局部偏移
                let world = r_j + rot * mp.local_offset;
                let d_i = world - position;
                let dist_sq_i = d_i.length_squared() + softening_sq;
                if dist_sq_i < 1.0 {
                    continue;
                }
                let dist_i = dist_sq_i.sqrt();
                acc_local += d_i * (mp.gm / (dist_sq_i * dist_i));
            }
            acc += acc_local;
        } else {
            // 远场或无 points：单 monopole `a = gm · d / dist³`（牛测定理远场等价）
            acc += d * (src.gm / (dist_sq * dist));
        }
    }
    acc
}

/// 由质量恢复引力参数 `gm = G · mass`（与 `mps_formula::celestial_data::G` 一致）。
#[inline]
pub fn gm_from_mass(mass: f64) -> f64 {
    G * mass
}
