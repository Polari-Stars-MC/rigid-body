//! `CosmosWorld` — 基于 `rapier3d-f64` 的太空物理场景。
//!
//! 仿 [`mps_core::rapier::world::PhysicsWorld`] 的字段布局，但去掉 C ABI /
//! 共享 arena / 力律登记表 / 事件钩子，仅保留太空演练所需的 rapier 后端
//! 加上：
//! - 一组注册的天体引力源（[`CelestialSource`]）
//! - 一组参与 n-body 互引力的动态质点源（[`NBodySource`]）
//! - 可选环境扰动力（大气阻力、太阳光压）的 per-body 配置
//!
//! 推进循环 [`CosmosWorld::step`]：在每个物理子步之前，对所有动态刚体
//! 累加「天体引力 + n-body 互引力 + 环境扰动力」的合**力**（加速度 × 质量），
//! 然后交给 `PhysicsPipeline::step` 完成 Rapier 的常规积分/约束求解。

use crate::gravity::{CelestialSource, NBodySource, celestial_acceleration, gm_from_mass};
use crate::orbit::BodyState;
use crate::perturbation::{
    atmospheric_drag_force, dynamical_friction_force, solar_pressure_force,
    solar_wind_pressure_force,
};
use mps_formula::astrophysics::{hill_sphere_radius, roche_limit_report};
use mps_formula::celestial_data::{AU, CelestialBody, G, MOONS, Moon};
use mps_formula::ffi::RocheLimitReport;
use rapier3d::math::Pose;
use rapier3d::prelude::{
    BroadPhaseBvh, CCDSolver, ColliderHandle, ColliderSet, ImpulseJointSet, IntegrationParameters,
    IslandManager, MultibodyJointSet, NarrowPhase, PhysicsPipeline, RigidBodyBuilder,
    RigidBodyHandle, RigidBodySet, Rotation, Vector,
};
use rayon::prelude::*;

/// §6：64B cache-line 对齐的 `(Vector, f64)` SOA 表（`scratch_source_pos_gm`）。
/// 热路径（全 monopole 远场）只读这张连续表，首元素 64B 对齐可减少大 M 场景的
/// unaligned load 罚分。零 bit-identical 风险（只改分配对齐，不动数据）。
///
/// stable 下 `allocator_api`（自定义 `Allocator`）未稳定，故手写对齐分配：委托
/// `std::alloc`，把任意请求的 align 抬到 64 的倍数；`reserve` 增长时同样走对齐分配。
/// 所有元素都是 `Copy`（`Vector`=`DVec3`、`f64`），`Drop` 无需逐元素析构。
pub(crate) struct AlignedPosGm {
    ptr: std::ptr::NonNull<(Vector, f64)>,
    len: usize,
    cap: usize,
}

impl AlignedPosGm {
    pub(crate) fn new() -> Self {
        // 空缓冲：ptr 用 dangling 占位（len=cap=0，永不解引用），首次 `reserve` 才分配。
        Self {
            ptr: std::ptr::NonNull::dangling(),
            len: 0,
            cap: 0,
        }
    }

    /// 保证容量 ≥ `len + additional`，不足时按 64B 对齐几何增长。
    pub(crate) fn reserve(&mut self, additional: usize) {
        let need = self.len + additional;
        if need <= self.cap {
            return;
        }
        let new_cap = (self.cap * 2).max(need).max(8);
        let new_layout = Self::layout(new_cap);
        let new_ptr = if self.cap == 0 {
            unsafe { std::alloc::alloc(new_layout) }
        } else {
            let old_layout = Self::layout(self.cap);
            unsafe {
                std::alloc::realloc(self.ptr.as_ptr() as *mut u8, old_layout, new_layout.size())
            }
        };
        if new_ptr.is_null() {
            std::alloc::handle_alloc_error(new_layout);
        }
        self.ptr = std::ptr::NonNull::new(new_ptr as *mut (Vector, f64)).unwrap();
        self.cap = new_layout.size() / std::mem::size_of::<(Vector, f64)>();
    }

    pub(crate) fn clear(&mut self) {
        self.len = 0;
    }

    pub(crate) fn push(&mut self, v: (Vector, f64)) {
        if self.len == self.cap {
            self.reserve(1);
        }
        unsafe { self.ptr.as_ptr().add(self.len).write(v) };
        self.len += 1;
    }

    pub(crate) fn as_slice(&self) -> &[(Vector, f64)] {
        unsafe { std::slice::from_raw_parts(self.ptr.as_ptr(), self.len) }
    }

    #[allow(dead_code)]
    pub(crate) fn len(&self) -> usize {
        self.len
    }

    /// 把 `len` 个元素的布局抬到 64B 对齐。
    fn layout(cap: usize) -> std::alloc::Layout {
        std::alloc::Layout::array::<(Vector, f64)>(cap)
            .expect("AlignedPosGm capacity overflow")
            .align_to(64)
            .expect("align_to(64) must succeed")
            .pad_to_align()
    }
}

impl std::fmt::Debug for AlignedPosGm {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AlignedPosGm")
            .field("len", &self.len)
            .field("cap", &self.cap)
            .finish()
    }
}

impl Drop for AlignedPosGm {
    fn drop(&mut self) {
        if self.cap == 0 {
            return;
        }
        let layout = Self::layout(self.cap);
        unsafe { std::alloc::dealloc(self.ptr.as_ptr() as *mut u8, layout) };
    }
}

/// 单刚体的环境扰动配置。
#[derive(Clone, Copy, Debug)]
pub struct PerturbationConfig {
    /// 大气阻力系数 Cd。
    pub drag_coefficient: f64,
    /// 迎风截面积（m²）。
    pub area: f64,
    /// 是否施加该天体的大气阻力。需配合 `central_body` 设置才能取密度。
    pub enable_drag: bool,
    /// 光压系数 Cr。
    pub reflectivity: f64,
    /// 受光截面积（m²）。
    pub optical_area: f64,
    /// 是否施加太阳光压。
    pub enable_solar: bool,
    /// 太阳风质子数密度（n / m³），典型 5e6（1 AU 处静态太阳风）。
    pub solar_wind_proton_density: f64,
    /// 太阳风整体速度（m/s），典型 400–800。
    pub solar_wind_speed: f64,
    /// 迎风（太阳风方向）有效面积（m²）。
    pub solar_wind_area: f64,
    /// 是否施加太阳风动压。
    pub enable_solar_wind: bool,
    /// 背景介质密度（kg/m³），用于 Chandrasekhar 动力学摩擦。
    pub friction_background_density: f64,
    /// 库仑对数 ln Λ，典型 2–10。
    pub friction_coulomb_log: f64,
    /// 是否施加动力学摩擦。
    pub enable_dynamical_friction: bool,
    /// 是否对光压/太阳风施加日食（阴影锥）衰减。遮挡体 = `central_body`（位于世界
    /// 原点，半径 `equatorial_radius`）；光源 = `sun_position`。默认关闭 → 现有光压/
    /// 太阳风输出逐位不变（满足「原方法不变」铁律）。开启后航天器进入中心体本影时
    /// 力衰减为 0、半影内按几何线性过渡。纯几何，不改非日食场景行为。
    pub enable_eclipse: bool,
    /// 是否施加 Hut (1981) 平衡潮自旋同步力矩（潮汐演化：自旋同步化）。默认关闭
    /// → 现有输出逐位不变。开启需 `central_body` 作潮汐伴星（无伴星则静默跳过）。
    pub enable_tidal: bool,
    /// 潮汐 Love 数 k2（无量纲），典型地球 0.299、月球 0.024、木卫 0.5+。
    pub love_number_k2: f64,
    /// 潮汐品质因子 Q（耗散），典型地球海潮 12、固体地球 ~100+、月球 ~30。
    pub tidal_q: f64,
    /// 被潮体半径（m），用于力矩强度 `R⁵` 标度。
    pub tidal_radius: f64,
}

impl Default for PerturbationConfig {
    fn default() -> Self {
        Self {
            drag_coefficient: 2.2,
            area: 0.0,
            enable_drag: false,
            reflectivity: 1.3,
            optical_area: 0.0,
            enable_solar: false,
            solar_wind_proton_density: 0.0,
            solar_wind_speed: 0.0,
            solar_wind_area: 0.0,
            enable_solar_wind: false,
            friction_background_density: 0.0,
            friction_coulomb_log: 0.0,
            enable_dynamical_friction: false,
            enable_eclipse: false,
            enable_tidal: false,
            love_number_k2: 0.299,
            tidal_q: 12.0,
            tidal_radius: 0.0,
        }
    }
}

/// 太空世界的配置。
#[derive(Clone, Debug)]
pub struct CosmosWorldConfig {
    /// 全局加速度锚（一般为 ZERO：太空场景无统一重力，引力由天体源贡献）。
    pub gravity: Vector,
    /// 积分步长（秒）。
    pub dt: f64,
    /// 约束求解迭代次数。
    pub solver_iterations: u32,
    /// CCD 子步数。
    pub ccd_substeps: u32,
    /// n-body 互引力的软化平方项（m²），避免两体无限接近时 1/r² 发散。
    ///
    /// 物理上引力的"硬截断"（`integrator.rs` 内 `dist_sq < 1.0` 跳过）只在距离
    /// <1m 时生效，对真实航天器间距永远不会触发；而两体近距离交会（如编队
    /// 飞行、对接接近）若 `softening_sq = 0`，1/r² 在数值上会瞬态冲高。
    /// 默认 `1e3` m²（约 31.6m 软化长度）——对千米级以上轨道间距无感，仅在
    /// 极近距离起到数值限幅。设为 `0.0` 则完全无软化（仅保留 1m 硬截断）。
    pub n_body_softening_sq: f64,
    /// n-body 中心天体（用于环境扰动力：大气密度/太阳方向参考）。
    /// 若为 `None` 则不施加基于中心天体的环境扰动。
    pub central_body: Option<&'static mps_formula::celestial_data::CelestialBody>,
    /// 轨道积分模式（默认走高阶辛积分器，长弧相位误差被压到 O(dt⁴)）。
    ///
    /// - [`OrbitIntegration::RapierForce`]：把合力用 `add_force` 喂给 rapier，
    ///   走 semi-implicit Euler。简单但长弧相位误差大（1s 步长一圈 LEO 漂~700km）。
    /// - [`OrbitIntegration::Verlet`]：天体引力 + n-body 互引力用 velocity-Verlet
    ///   显式积分直接写回 `translation`/`linvel`，rapier 只负责碰撞/约束/姿态。
    ///   2 阶辛，长弧相位误差随 dt² 收敛，每步 ~10⁻¹⁰ 能量误差。阻力/光压并入
    ///   加速度函数。
    /// - [`OrbitIntegration::Yoshida4`]：Yoshida 4 阶辛积子，3 级复合 leapfrog。
    ///   每步 ~10⁻¹⁴ 能量误差，相位误差随 dt⁴ 收敛。比 Verlet 每步多 2 次加速度
    ///   评估，但每步精度升两个量级，是默认模式。
    /// - [`OrbitIntegration::ForestRuth8`]：Forest–Ruth 8 阶辛积子，15 级 McLachlan
    ///   系数复合。每步 ~10⁻¹⁶ 能量误差（逼近 f64 极限）。算力需求约为 Verlet 的
    ///   15 倍，长弧高精导航适用。
    /// - [`OrbitIntegration::Yoshida4Kahan`] / [`ForestRuth8Kahan`]：在对应高阶
    ///   积子上叠加 Kahan 补偿累加位置/速度增量，把长弧（数千–数万步）里逐步
    ///   `r += v·dt` 的舍入积累从 ~√N·ε 降到 ~ε，长弧闭合精度再升 1–3 量级。
    pub orbit_integration: OrbitIntegration,
    /// 整步内部子步数：一次 `step(dt)` 内做 `substeps` 次小步积分，
    /// 每次 `dt/substeps` 秒。子步越多相位误差越小；1 内部子步即在 `dt` 内
    /// 走一整步（积子内部的级数由积子自身阶数定，不再切分）。
    /// 对所有非 `RapierForce` 模式生效。
    pub verlet_substeps: u32,
    /// 是否开启近心点自适应子步。开启后，当刚体进入中心天体近心点附近
    /// （`r < 2·r_eq`）时，按 `mps_formula::integrators::adaptive_step_size`
    /// 用一步误差估计 × `adaptive_tolerance` 动态加密子步；远心点仍走
    /// `verlet_substeps` 为主。默认关——对近圆轨道无感，椭圆/转移轨道可省算力。
    pub adaptive_substeps: bool,
    /// 自适应子步的目标单步相对误差。典型 `1e-9`。仅 `adaptive_substeps=true`
    /// 时生效。
    pub adaptive_tolerance: f64,
    /// 中心天体引力的相对论后牛顿（1PN/2PN）修正。
    ///
    /// 近地轨道 1PN 量级 ~10⁻⁹，多数场景无感；高轨/近日接近过心点场景下
    /// 相位修正显著可观测。默认 `None`。
    pub relativistic_correction: RelativisticCorrection,
}

/// 轨道积分模式。
///
/// 阶数与每步能量误差为典型量级（LEO 一圈），供选型参考；实际精度仍取决于
/// 子步数、轨道偏心率、扰动模型。所有非 `RapierForce` 模式都绕过 rapier 的力
/// 律，由 [`crate::integrator`] 显式积分写回 `translation`/`linvel`，rapier
/// 只跑碰撞/约束/姿态。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OrbitIntegration {
    /// 用 rapier 的 `add_force` 路径走 semi-implicit Euler。1 阶，每步 ~10⁻⁵。
    /// 仅作为兼容/对照路径。
    RapierForce,
    /// 显式 velocity-Verlet（2 阶辛 leapfrog），每步 ~10⁻¹⁰。
    Verlet,
    /// Yoshida 4 阶辛积子（3 级复合 leapfrog），每步 ~10⁻¹⁴。默认。
    #[default]
    Yoshida4,
    /// Forest–Ruth 8 阶辛积子（15 级 McLachlan 系数），每步 ~10⁻¹⁶。
    ForestRuth8,
    /// Yoshida 4 + Kahan 补偿位置/速度长弧累加。
    Yoshida4Kahan,
    /// Forest–Ruth 8 + Kahan 补偿位置/速度长弧累加。
    ForestRuth8Kahan,
}

/// 中心天体引力相对论后牛顿修正模式。
///
/// 叠加在 `total_acceleration` 中心引力项之上；n-body 与扰动项不修正
/// （多体相对论模型算法复杂、物理意义弱，不在 cosmos 范围）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RelativisticCorrection {
    /// 不做相对论修正（默认）。
    #[default]
    None,
    /// 1PN 一阶后牛顿修正（近日点进动主导项）。
    OnePN,
    /// 2PN 二阶后牛顿修正（用于太阳系内高精度历表）。
    TwoPN,
    /// 1PN + 2PN 全修正。
    Full,
}

impl Default for CosmosWorldConfig {
    fn default() -> Self {
        Self {
            gravity: Vector::ZERO,
            dt: 1.0 / 60.0,
            solver_iterations: 4,
            ccd_substeps: 4,
            n_body_softening_sq: 1e3,
            central_body: None,
            orbit_integration: OrbitIntegration::default(),
            verlet_substeps: 1,
            adaptive_substeps: false,
            adaptive_tolerance: 1e-9,
            relativistic_correction: RelativisticCorrection::default(),
        }
    }
}

/// 太空物理世界。所有公开 API 自行管理内部 `RigidBodySet` 等。
///
/// 手写 `Clone`（而非 derive）因为 `PhysicsPipeline` 不实现 `Clone`——它是
/// 无状态的工作对象（每次 `step` 内部重建临时结构），克隆时用 `::new()`
/// 恢复即可。用途：场景快照/回滚（演练器 undo、Monte Carlo 多世界并行）。
/// 成本是深拷贝整个 body/collider set；超大规模场景应考虑 `Arc` 共享只读配置。
pub struct CosmosWorld {
    pipeline: PhysicsPipeline,
    gravity: Vector,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,

    celestials: Vec<CelestialSource>,
    n_body_sources: Vec<NBodySource>,
    n_body_softening_sq: f64,
    central_body: Option<&'static mps_formula::celestial_data::CelestialBody>,

    /// per-body 环境扰动配置，按 handle 的 arena index 存储。
    perturbations: Vec<Option<PerturbationConfig>>,
    /// 太阳在世界中的位置（用于光压方向），默认放在原点。
    sun_position: Vector,
    /// 轨道积分模式（见 [`CosmosWorldConfig::orbit_integration`]）。
    orbit_integration: OrbitIntegration,
    /// Verlet 子步数。
    verlet_substeps: u32,
    /// 近心点自适应子步开关。
    adaptive_substeps: bool,
    /// 自适应子步目标误差。
    adaptive_tolerance: f64,
    /// 相对论修正模式。
    relativistic_correction: RelativisticCorrection,
    /// per-body Kahan 补偿累加态，按 arena index 存储。仅 `*Kahan` 积分模式下
    /// 使用，存 `(position_accum, velocity_accum)`；其它模式惰性保持空。
    kahan_state: Vec<Option<(mps_formula::math::KahanVec3, mps_formula::math::KahanVec3)>>,

    /// 显式积子路径的工作向量复用缓冲：存本子步要处理的动态体元组
    /// `(handle, pos, vel, mass, perturbation)`。每子步 `clear()` + `extend()`，
    /// 跨子步/跨帧复用同一分配，消除每帧每子步的 `Vec::with_capacity` 抖动。
    /// 在静态/固定刚体比例高、动态刚体数量稳定时收益明显。
    scratch_tasks: Vec<(
        RigidBodyHandle,
        Vector,
        Vector,
        f64,
        Option<PerturbationConfig>,
    )>,
    /// `collect_dynamic_tasks` 的「动态体 handle 列表」复用缓冲。每子步 `clear()`
    /// 后 `par_extend` 复用同块内存。原实现每子步 `self.scratch_tasks = collected`
    /// 会重新赋值成全新 `Vec`，导致每帧每子步多次堆分配，破坏 A1/A2「稳态零堆分配」
    /// 目标；现用本缓冲承接 handle 列表、再 `par_extend` 入 `scratch_tasks`，不再
    /// 新建或重新赋值。与 `scratch_tasks` 同生命周期、容量随工作集收敛。
    scratch_handles: Vec<RigidBodyHandle>,
    /// n-body 源位置快照复用缓冲：每子步 `clear()` + 按需写入，跨子步复用。
    scratch_source_positions: Vec<Vector>,
    /// SOA 紧凑表（`n_body_sources` 同序）：`(源世界位姿, gm)`。仅当
    /// `!has_irregular_sources`（全 monopole）时热循环读它代替逐源查
    /// `source_positions[src_idx]` + `src.gm`，提升缓存局部性。顺序与
    /// `n_body_sources` 完全一致 → 累加顺序不变 → **数值惰性（bit-identical）**。
    /// 含不规则源时该表不被使用，热循环走 `n_body_sources` 原路径。
    scratch_source_pos_gm: AlignedPosGm,
    /// n-body 源姿态快照复用缓冲（与 `scratch_source_positions` 同步写）：每子步
    /// `clear()` + 按 arena index 写入 `body.rotation()`，供不规则质量分布近场
    /// 分支把质点 `local_offset` 变到世界坐标。
    scratch_source_rotations: Vec<Rotation>,
    /// 显式积分路径下，需把「挂在刚体上的 collider」位姿同步写回。P1.7 之前每步
    /// `collect()` 全部有 parent 的 collider 到 `Vec::new()` 抛弃——这里改为跨步
    /// 复用：`clear()` + push，消除每步堆分配。元素是 `(collider_handle, world_pose)`。
    scratch_collider_updates: Vec<(ColliderHandle, Pose)>,
    /// 跨子步复用的 Kahan 累加态快照缓冲：存 `explicit_substep` 3.5 段从
    /// `kahan_state` 拷出的 per-body `(kp, kv)`（Copy）。每子步 `clear()` +
    /// 重写，消除每子步 `Vec::with_capacity` 的堆分配抖动（稳态零分配）。
    kahan_src_buf: Vec<Option<(mps_formula::math::KahanVec3, mps_formula::math::KahanVec3)>>,
    /// 跨子步复用的并行预计算缓冫：存每体 `(a0, 高阶推进, Kahan 推进)` 三元组。
    /// 由 `scratch_tasks.par_iter()` 经 `Vec::par_extend` 原地复用同块内存，
    /// 消除每子步的 `Vec::with_capacity` + `collect()` 分配。
    advance_buf: Vec<(
        Vector,
        Option<(Vector, Vector)>,
        Option<(
            Vector,
            Vector,
            mps_formula::math::KahanVec3,
            mps_formula::math::KahanVec3,
        )>,
    )>,
    /// 本世界是否含有「不规则质量分布」n-body 源（带 `points` 且
    /// `near_field_threshold > 0`）。仅当存在这类源时，热循环 n-body 互引力的
    /// 近场 O(P) 分支才会被触发。由 `refresh_n_body_sources` 每次重算（纯只读
    /// 聚合，无额外分配）。`false` 时 `total_acceleration` 会跳过每个源的
    /// `!src.points.is_empty() && near_threshold > 0.0` 测试（全 monopole 的
    /// 常见路径），等价短路——该分支在 `false` 下本就不会触发，故数值惰性。
    has_irregular_sources: bool,

    /// 可选的共享内存 arena（Java→Rust 零拷贝命令通道 + Rust→Java 零拷贝
    /// 状态回读）。由 `create_shared_arena` 创建、随世界生命周期存在；`step`
    /// 在头部 drain Java 命令、尾部 flush body 槽。`None` 时不走 arena 路径。
    shared_arena: Option<Box<crate::arena::SharedArena>>,
    /// arena index（`bodies` 插入序号）→ 本世界 `RigidBodyHandle` 的映射。
    /// `bodies.len()` 变化时才增量重建（插入序稳定，故映射稳定）。
    arena_idx_map: Vec<RigidBodyHandle>,
    /// 上一次 `arena_idx_map` 重建时的 `bodies.len()`，用于判断是否需要重建。
    arena_idx_map_body_count: usize,
    /// drain 出的 per-arena-index 命令合力（AddForce），按 arena index 存储。
    /// 显式积子路径在 `explicit_substep` 里按 `命令力/质量` 折成附加加速度注入；
    /// RapierForce 路径直接 `add_force` 消费，不读此缓冲。每条 `step` 尾部清空。
    arena_cmd_forces: Vec<Option<Vector>>,

    /// 可选的星际无线电传播子世界。step 尾部自动推进（直接读本世界刚体位置）。
    radio: Option<crate::radio::RadioWorld>,
}

/// 一次 `step` 的诊断结果。调用方原本只能靠 `step` 的静默 return 猜
/// "为什么没推进"，现在能直接判。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StepResult {
    /// 正常推进了 `dt` 秒（RapierForce 路径下就是入参 dt；Verlet 路径下
    /// 是整步 dt，内部子步已自行处理）。
    Stepped(f64),
    /// 由于子步切分，被拆成 `n` 个 `dt/n` 秒小步完成（RapierForce 路径
    /// 下 `dt > MAX_STEP_DT` 时启用）。
    Substepped { substeps: u32, sub_dt: f64 },
    /// `dt` 非法（NaN / ≤0 / 超过单步上限），整步被跳过。
    Skipped(StepSkipReason),
}

/// `step` 跳过的原因。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepSkipReason {
    /// `dt` 为 NaN 或无穷。
    NonFinite,
    /// `dt <= 0`。
    NonPositive,
    /// `dt` 超过单步安全上限（当前 10s，防止误把"一帧"当"一小时"喂进来
    /// 后让积分发散）。需要更长推进请用 `step_n` 或循环 `step`。
    TooLarge,
}

/// 单步允许的最大 dt（秒）。超过则 RapierForce 路径会做子步切分以保精度；
/// Verlet 路径由 `verlet_substeps` 控制子步，不受此上限约束。
const MAX_STEP_DT: f64 = 10.0;

impl CosmosWorld {
    pub fn new(config: CosmosWorldConfig) -> Self {
        let integration_parameters = IntegrationParameters {
            dt: config.dt,
            num_solver_iterations: config.solver_iterations as usize,
            max_ccd_substeps: config.ccd_substeps as usize,
            ..IntegrationParameters::default()
        };
        Self {
            pipeline: PhysicsPipeline::new(),
            gravity: config.gravity,
            integration_parameters,
            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            celestials: Vec::new(),
            n_body_sources: Vec::new(),
            n_body_softening_sq: config.n_body_softening_sq,
            central_body: config.central_body,
            perturbations: Vec::new(),
            sun_position: Vector::ZERO,
            orbit_integration: config.orbit_integration,
            verlet_substeps: config.verlet_substeps.max(1),
            adaptive_substeps: config.adaptive_substeps,
            adaptive_tolerance: config.adaptive_tolerance,
            relativistic_correction: config.relativistic_correction,
            kahan_state: Vec::new(),
            scratch_tasks: Vec::new(),
            scratch_handles: Vec::new(),
            scratch_source_positions: Vec::new(),
            scratch_source_pos_gm: AlignedPosGm::new(),
            scratch_source_rotations: Vec::new(),
            scratch_collider_updates: Vec::new(),
            kahan_src_buf: Vec::new(),
            advance_buf: Vec::new(),
            has_irregular_sources: false,
            shared_arena: None,
            arena_idx_map: Vec::new(),
            arena_idx_map_body_count: 0,
            arena_cmd_forces: Vec::new(),
            radio: None,
        }
    }

    /// P2.18 / M5: shallow clone — copies the *configuration and parameters*
    /// (gravity, integration params, softening, central body ref, sun
    /// position, orbit-integration mode, substep settings, relativistic
    /// correction, kahan flag) but drops all runtime mutable body state
    /// (bodies, colliders, joints, celestials, n_body_sources,
    /// perturbations, kahan_state, scratch buffers).
    ///
    /// Produces an "all-parameters-copied, fresh-empty-physics-scene"
    /// world, suitable for:
    ///   - Rolling a parallel branch for Monte Carlo study
    ///   - Spawning an "advance-the-future-N-steps" overlay while keeping
    ///     the current world untouched (the predicted branch holds its
    ///     own bodies you can later `insert_body` into).
    ///   - Avoiding the ~5–10 ms deep-copy of a 1000-body `RigidBodySet`
    ///     that the full `Clone` impl pays when the caller never needs
    ///     to retain existing body geometry.
    ///
    /// Pipeline / islands / broad-phase / narrow-phase / ccd-solver are
    /// reset via `::new()` (mirroring the `Clone` impl's stance — these
    /// are stateless work objects rebuilt each `step`).
    ///
    /// **Warning**: `celestials` / `n_body_sources` are *not* preserved —
    /// callers must re-`add_celestial` / `add_n_body` on the shallow copy
    /// if those are still wanted (they reference body handles that would
    /// dangle once the body set is reset to empty anyway).
    pub fn clone_shallow(&self) -> Self {
        Self {
            pipeline: PhysicsPipeline::new(),
            gravity: self.gravity,
            integration_parameters: self.integration_parameters,
            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            // celestials / n_body_sources intentionally dropped: see doc above.
            celestials: Vec::new(),
            n_body_sources: Vec::new(),
            n_body_softening_sq: self.n_body_softening_sq,
            central_body: self.central_body,
            perturbations: Vec::new(),
            sun_position: self.sun_position,
            orbit_integration: self.orbit_integration,
            verlet_substeps: self.verlet_substeps,
            adaptive_substeps: self.adaptive_substeps,
            adaptive_tolerance: self.adaptive_tolerance,
            relativistic_correction: self.relativistic_correction,
            kahan_state: Vec::new(),
            // scratch buffers always begin empty on a fresh world (P0.2 / P1.7).
            scratch_tasks: Vec::new(),
            scratch_handles: Vec::new(),
            scratch_source_positions: Vec::new(),
            scratch_source_pos_gm: AlignedPosGm::new(),
            scratch_source_rotations: Vec::new(),
            scratch_collider_updates: Vec::new(),
            kahan_src_buf: Vec::new(),
            advance_buf: Vec::new(),
            has_irregular_sources: false,
            shared_arena: None,
            arena_idx_map: Vec::new(),
            arena_idx_map_body_count: 0,
            arena_cmd_forces: Vec::new(),
            radio: None,
        }
    }

    /// 设太阳位置（光压方向参考）。
    pub fn set_sun_position(&mut self, pos: Vector) {
        self.sun_position = pos;
    }

    /// 创建共享内存 arena（Java 零拷贝命令通道 + 状态回读）。
    ///
    /// 一个世界最多一个 arena；已存在则原样保留（返回 `false`）。arena 容量
    /// 必须 >0 且不超过 [`crate::arena::MAX_ARENA_BODIES`] /
    /// [`crate::arena::MAX_ARENA_COMMANDS`]，且总分配 ≤ 256 MiB。`step` 会
    /// 自动 drain Java 写入的命令、并在尾部 flush body 槽供 Java 零拷贝读取。
    ///
    /// 返回 `true` 表示创建成功（`self.shared_arena` 现为 `Some`）。
    pub fn create_shared_arena(&mut self, max_bodies: u32, max_commands: u32) -> bool {
        if self.shared_arena.is_some() {
            return false;
        }
        match crate::arena::SharedArena::new(max_bodies, max_commands) {
            Some(arena) => {
                self.shared_arena = Some(Box::new(arena));
                self.arena_idx_map.clear();
                self.arena_idx_map_body_count = 0;
                true
            }
            None => false,
        }
    }

    /// 销毁共享 arena（若有的话）。`None` 时是 no-op。
    ///
    /// **Java 侧注意**：销毁前必须先释放/解映射映射该 arena 的 `MemorySegment`，
    /// 否则内存已释放而 Java 段仍指向它会造成 use-after-free。
    pub fn destroy_shared_arena(&mut self) {
        self.shared_arena = None;
    }

    /// 取 arena 基地址（无 arena 时返回 0）。供 FFI `cosmos_world_get_shared_arena_address`。
    pub fn shared_arena_address(&self) -> u64 {
        self.shared_arena.as_ref().map_or(0, |a| a.address())
    }

    /// 取 arena 总字节大小（无 arena 时返回 0）。供 FFI `cosmos_world_get_shared_arena_size`。
    pub fn shared_arena_size(&self) -> u64 {
        self.shared_arena.as_ref().map_or(0, |a| a.size() as u64)
    }

    /// Drain the arena command ring (Java→Rust) and apply each command to its
    /// target body.  Called at the top of every `step` when an arena exists.
    ///
    /// Rebuilds `arena_idx_map` (arena index → `RigidBodyHandle`) incrementally:
    /// rapier preserves insertion order, so the map only changes when the body
    /// count changes.  The command ring is SPSC (Java producer, Rust consumer);
    /// `drain_commands` reads `[0, cmd_write)`, dispatches each slot, and resets
    /// `cmd_write` to 0.
    fn drain_arena_commands(&mut self) {
        let arena = match &self.shared_arena {
            Some(a) => a,
            None => return,
        };
        let commands = arena.drain_commands();
        if commands.is_empty() {
            return;
        }

        // 增量重建 arena index → handle 映射（仅 body 数变化时）。
        let n_bodies = self.bodies.len();
        if n_bodies != self.arena_idx_map_body_count {
            self.arena_idx_map.clear();
            for (h, _) in self.bodies.iter() {
                self.arena_idx_map.push(h);
            }
            self.arena_idx_map_body_count = n_bodies;
        }
        let idx = &self.arena_idx_map;

        for (cmd_type, body_index, a0, a1, a2) in commands {
            let h = match idx.get(body_index as usize) {
                Some(&h) => h,
                None => continue,
            };
            let body = match self.bodies.get_mut(h) {
                Some(b) => b,
                None => continue,
            };
            match cmd_type {
                // AddForce — RapierForce 路径直接 add_force；显式积子路径先攒到
                // `arena_cmd_forces`（按 arena index），由 `explicit_substep` 折成附加
                // 加速度注入（rapier 的 user_force 在显式路径下不参与积分）。
                0 => {
                    if matches!(self.orbit_integration, OrbitIntegration::RapierForce) {
                        body.add_force(Vector::new(a0, a1, a2), true);
                    } else {
                        let bi = body_index as usize;
                        if bi >= self.arena_cmd_forces.len() {
                            self.arena_cmd_forces.resize_with(bi + 1, || None);
                        }
                        let existing = self.arena_cmd_forces[bi].unwrap_or(Vector::ZERO);
                        self.arena_cmd_forces[bi] = Some(existing + Vector::new(a0, a1, a2));
                    }
                }
                // AddTorque — 仅 RapierForce 路径消费（显式路径不积分角运动）。
                1 => {
                    body.add_torque(Vector::new(a0, a1, a2), true);
                }
                // SetVelocity
                2 => {
                    body.set_linvel(Vector::new(a0, a1, a2), true);
                }
                // ApplyImpulse
                3 => {
                    body.apply_impulse(Vector::new(a0, a1, a2), true);
                }
                // ApplyTorqueImpulse
                4 => {
                    body.apply_torque_impulse(Vector::new(a0, a1, a2), true);
                }
                // WakeUp
                5 => {
                    body.wake_up(true);
                }
                // Sleep
                6 => {
                    body.sleep();
                }
                // SetRotation — a0..a2 = axis·angle vector (magnitude = angle rad)
                7 => {
                    let axis_angle = Vector::new(a0, a1, a2);
                    let angle = axis_angle.length();
                    if angle > 1e-12 {
                        let unit_axis = axis_angle / angle;
                        body.set_rotation(Rotation::from_axis_angle(unit_axis, angle), true);
                    }
                }
                // SetPose — a0..a2 = position, rotation kept
                8 => {
                    let pos = rapier3d::prelude::Pose::from_parts(
                        Vector::new(a0, a1, a2),
                        *body.rotation(),
                    );
                    body.set_position(pos, true);
                }
                // SetGravityScale — a0 = scale
                9 => {
                    body.set_gravity_scale(a0, true);
                }
                // SetLinearDamping — a0 = damping
                10 => {
                    body.set_linear_damping(a0);
                }
                // SetAngularDamping — a0 = damping
                11 => {
                    body.set_angular_damping(a0);
                }
                // Unknown command type — ignore (defensive; Java only writes known types).
                _ => {}
            }
        }
    }

    /// 设/改 n-body 中心天体（用于环境扰动力：大气密度/太阳方向参考）。
    /// 传 `None` 清除，则后续不施加基于中心天体的大气阻力。
    pub fn set_central_body(
        &mut self,
        body: Option<&'static mps_formula::celestial_data::CelestialBody>,
    ) {
        self.central_body = body;
    }

    /// 注册一个天体引力源。返回其索引便于后续移除/启用切换。
    pub fn add_celestial(&mut self, source: CelestialSource) -> usize {
        self.celestials.push(source);
        self.celestials.len() - 1
    }

    /// 注册一个自然卫星（月球）引力源，复用 [`Self::add_celestial`] 的球谐
    /// 基础设施。卫星以 `Moon` 点质量载入：`max_degree=0`、球谐系数为空、
    /// `j2..j6=0`、无大气/太阳光压（月球级小天体无需高阶场）。返回索引。
    pub fn add_moon(&mut self, moon: &'static Moon) -> usize {
        let body = CelestialBody {
            name: moon.name,
            gm: moon.gm,
            equatorial_radius: moon.radius,
            flattening: 0.0,
            rotation_rate: 0.0,
            j2: 0.0,
            j3: 0.0,
            j4: 0.0,
            j5: 0.0,
            j6: 0.0,
            max_degree: 0,
            c_coeffs: &[],
            s_coeffs: &[],
            ref_radius: moon.radius,
            surface_density: 0.0,
            scale_height: 0.0,
            solar_pressure_constant: 0.0,
        };
        self.add_celestial(CelestialSource::new(Box::leak(Box::new(body)), 0))
    }

    /// 按 `MOONS` 数组下标注册卫星。越界返回 `None`（供 FFI 转 -1 守卫）。
    pub fn add_moon_by_index(&mut self, index: i32) -> Option<usize> {
        let i = usize::try_from(index).ok()?;
        let moon = MOONS.get(i)?;
        Some(self.add_moon(moon))
    }

    /// 注册一个 n-body 互引力质点源（给定质量 kg），作为**纯点质量**（monopole）。
    /// 远场/近场均按 `a = G·M·r̂ / r²` 算，与历史行为完全一致。若刚体已插入，
    /// 也可直接调 [`Self::insert_body_as_gravity_source`]。
    ///
    /// 不规则分布（非球星体、土豆星、双瓣小行星）请用 [`Self::add_n_body_irregular`]
    /// ——它带一组离散质量点来表达延展/非对流球对称的质量分布。
    pub fn add_n_body(&mut self, handle: RigidBodyHandle, mass: f64) {
        let gm = gm_from_mass(mass);
        self.n_body_sources.push(NBodySource::monopole(handle, gm));
    }

    /// 注册一个**不规则质量分布**的 n-body 互引力源：
    /// - `total_mass`：源总质量（kg），用于远场 monopole 快路径与刚体质量变化时的
    ///   `gm` 比例刷新（见 `refresh_n_body_sources`）。
    /// - `points`：本体局部坐标下的离散质量点（每个点带 `gm=G·mᵢ` 与 `local_offset`）。
    ///   各 `gm` 之和理想上等于 `G·total_mass`（守恒），但代码不强求——近场走 Σ 自洽、
    ///   远场走 `gm` 自洽，两者仅在 `bounding_radius=0` 或空 `points` 时才会被一起用到。
    /// - `bounding_radius`：这些 `points` 的边界球半径（世界米）。距源质心
    ///   ≤ `NEAR_FIELD_FACTOR · bounding_radius` 时走质点求和，更远则切回 monopole。
    ///
    /// 当刚体质量随后变化（燃料燃烧等变质量场景），`refresh_n_body_sources` 会按
    /// `新body.mass / 注册 total_mass` 比例同步缩放每点的 `gm`（和总 `gm`），让源
    /// 互引力大小随当前质量走、方向结构仍由 `points / local_offset` 决定。
    pub fn add_n_body_irregular(
        &mut self,
        handle: RigidBodyHandle,
        total_mass: f64,
        points: Vec<crate::gravity::MassPoint>,
        bounding_radius: f64,
    ) {
        let total_gm = gm_from_mass(total_mass);
        self.n_body_sources.push(NBodySource::irregular(
            handle,
            total_gm,
            points,
            bounding_radius,
        ));
    }

    /// 设置某刚体的环境扰动配置。
    pub fn set_perturbation(&mut self, handle: RigidBodyHandle, cfg: PerturbationConfig) {
        let idx = handle.into_raw_parts().0 as usize;
        if idx >= self.perturbations.len() {
            self.perturbations.resize(idx + 1, None);
        }
        self.perturbations[idx] = Some(cfg);
    }

    /// 插入一个已配置好的刚体 builder，返回其句柄。
    pub fn insert_body(&mut self, builder: RigidBodyBuilder) -> RigidBodyHandle {
        let mut rb = builder.build();
        // Rapier builder 只把 additional_mass_properties 暂存到
        // `additional_local_mprops`，要等 pipeline.step 里的
        // `handle_user_changes_to_rigid_bodies` 才会并入 `local_mprops` 并据
        // 此计算 effective_inv_mass。在 step 之前调用方若立即需要 mass/受力
        // 大小正确，就显式重算一次。
        rb.recompute_mass_properties_from_colliders(&self.colliders);
        self.bodies.insert(rb)
    }

    /// 插入刚体并将其质量登记为 n-body 源（一步到位）。
    pub fn insert_body_as_gravity_source(
        &mut self,
        builder: RigidBodyBuilder,
        mass: f64,
    ) -> RigidBodyHandle {
        let handle = self.insert_body(builder);
        self.add_n_body(handle, mass);
        handle
    }

    /// 取刚体当前位置。
    pub fn body_translation(&self, handle: RigidBodyHandle) -> Option<Vector> {
        self.bodies.get(handle).map(|b| b.translation())
    }

    /// 取刚体线速度。
    pub fn body_linvel(&self, handle: RigidBodyHandle) -> Option<Vector> {
        self.bodies.get(handle).map(|b| b.linvel())
    }

    /// 取刚体质量。
    pub fn body_mass(&self, handle: RigidBodyHandle) -> Option<f64> {
        self.bodies.get(handle).map(|b| b.mass())
    }

    /// 设刚体总质量（`total_to_add` 是要设成的目标质量 kg）。返回设之前的旧
    /// 质量（`body.mass()`），便于测试/上层做比例。仅对存在的刚体有效；不存在
    /// 刚体返回 `None`。
    ///
    /// 实现走 rapier 的 `set_additional_mass`，让质量分布按均匀块近似更新；n-body
    /// 源 `gm` 的跟随刷新在 [`step`] / Verlet 子步开头的 `refresh_n_body_sources` 处
    /// 按 `body.mass()` 重算，调用本方法后下一帧自动反映新质量。
    pub fn set_body_mass(&mut self, handle: RigidBodyHandle, total_to_add: f64) -> Option<f64> {
        let b = self.bodies.get_mut(handle)?;
        let old = b.mass();
        // rapier 0.35：额外质量通过 set_additional_mass 重设；但 `body.mass()` 读
        // 的是 `local_mprops`（由碰撞体合成），`additional_local_mprops` 默认只在
        // 下一次 step 时才合成进去。这里随即调一次 recompute 让 `body.mass()`
        // 立即反映（无 collider 的刚体也走这条路径：recompute 把 additional
        // 并进 local，使后续读取与 step 路径一致）。nobody 异常路径无 collider
        // 时 recompute 只会把 additional 平移到 local，不引入额外误差。
        b.set_additional_mass(total_to_add, true);
        b.recompute_mass_properties_from_colliders(&self.colliders);
        Some(old)
    }

    /// 对本世界中的刚体计算"中心天体对其的洛希极限"报告。洛希极限 = 行星
    /// 表面对卫星潮汐力等于卫星自引力的距离；卫星在此距离内会被撕碎。
    ///
    /// 主星数据完全由 [`set_central_body`] 注册过的中心天体推导：`equatorial_radius`
    /// 取作主星半径；`GM`/`G`/`((4/3)·π·R³)` 反算出主星平均密度。卫星密度因
    /// `RigidBody` 自身不含此信息，**必须由调用方提供**（`secondary_density`，
    /// kg/m³，由您对刚体几何/物质模型的认知决定）。
    ///
    /// 轨道距离取刚体当前位置到世界原点的模长 —— 与 `celestial_acceleration`
    /// 把中心天体放在世界原点的约定一致。
    ///
    /// 返回 `None` 当：
    /// - 未设 `central_body`，或其 `equatorial_radius`/`gm` 非正/退化；
    /// - 刚体不存在；
    /// - `secondary_density` 非有限正数。
    ///
    /// **不改推进** —— 仅做查询。期盼在越过极限时施加潮汐拉扯的物理效应，
    /// 应另行加力律；该函数为上层提供"距离是否已越界"的决策依据。
    pub fn roche_limit_for(
        &self,
        handle: RigidBodyHandle,
        secondary_density: f64,
    ) -> Option<RocheLimitReport> {
        let central = self.central_body?;
        let body = self.bodies.get(handle)?;
        let primary_radius = central.equatorial_radius;
        if primary_radius <= 0.0 || central.gm <= 0.0 {
            return None;
        }
        if !secondary_density.is_finite() || secondary_density <= 0.0 {
            return None;
        }
        // 主星平均密度 ρ = M / V = (GM/G) / ((4/3)·π·R³)
        let primary_mass = central.gm / G;
        let primary_density =
            primary_mass / ((4.0 / 3.0) * std::f64::consts::PI * primary_radius.powi(3));
        let orbital_distance = body.translation().length();
        roche_limit_report(
            primary_radius,
            primary_density,
            secondary_density,
            orbital_distance,
        )
    }

    /// 计算围绕该刚体的 Hill 球半径——其引力主导区域，是与 `roche_limit_for`
    /// 互补的另一判据：Roche 极限告知"卫星被主星潮汐撕碎"的距离，
    /// Hill 球告知"主星之外的引力主导范围"（适合判断小卫星/航天器能否稳定
    /// 围绕该刚体运行而不被主星剥离）。
    ///
    /// `r_H ≈ a · (1 - e) · (m_sec / (3·m_pri))^(1/3)`。
    ///
    /// 主星质量 m_pri 由 `set_central_body` 注册的天体 `GM/G` 反算；卫星质量
    /// m_sec 取自刚体本身；半长轴 a 与离心率 e 由调用方提供——CosmosWorld
    /// 直推轨道，不维护开普勒根数，故需用户态（如基于 `body_state` 通过
    /// `mps_formula::spaceflight::state_to_elements` 反算后传入）。
    ///
    /// 返回 `None` 当未设 `central_body` / 刚体不存在 / `a` 或 `e` 非法 /
    /// `gm`≤0 / 刚体质量≤0。
    pub fn hill_radius_for(
        &self,
        handle: RigidBodyHandle,
        semi_major_axis: f64,
        eccentricity: f64,
    ) -> Option<f64> {
        let central = self.central_body?;
        let body = self.bodies.get(handle)?;
        let body_mass = body.mass();
        if central.gm <= 0.0 || body_mass <= 0.0 {
            return None;
        }
        let primary_mass = central.gm / G;
        hill_sphere_radius(primary_mass, body_mass, semi_major_axis, eccentricity)
    }

    /// 取刚体完整状态切片（用于轨道诊断）。
    pub fn body_state(&self, handle: RigidBodyHandle) -> Option<BodyState> {
        self.bodies
            .get(handle)
            .map(|b| BodyState::new(b.translation(), b.linvel()))
    }

    /// 当前动态刚体数量。
    pub fn dynamic_body_count(&self) -> usize {
        self.bodies.iter().filter(|(_, b)| b.is_dynamic()).count()
    }

    /// 推进一个步长。
    ///
    /// 按 `orbit_integration` 配置选路径：
    /// - `RapierForce`：把合力用 `add_force` 注入，rapier 内部 semi-implicit Euler。
    ///   `dt > MAX_STEP_DT` 时内部自动拆成若干 ≤`MAX_STEP_DT` 的子步，每子步
    ///   重注入力，返回 [`StepResult::Substepped`]。
    /// - 其它模式（`Verlet` / `Yoshida4` / `ForestRuth8` / `*Kahan`）：天体引力 +
    ///   n-body 由 [`crate::integrator`] 显式辛积分写回 translation/linvel，rapier
    ///   只跑碰撞/约束/姿态。子步数由 `verlet_substeps` 控制（自适应模式下另由
    ///   近心点动态加密）。
    ///
    /// 返回 [`StepResult`]，调用方可据此判断"为什么没推进"。
    pub fn step(&mut self, dt: f64) -> StepResult {
        if !dt.is_finite() {
            return StepResult::Skipped(StepSkipReason::NonFinite);
        }
        if dt <= 0.0 {
            return StepResult::Skipped(StepSkipReason::NonPositive);
        }
        if dt > 30.0 {
            return StepResult::Skipped(StepSkipReason::TooLarge);
        }

        // --- Arena: drain Java commands (Java→Rust zero-copy channel) ---
        // 在物理推进之前把 Java 写入命令环的命令应用到刚体；推进结束后 flush
        // body 槽供 Java 零拷贝回读（见本方法尾部）。
        self.drain_arena_commands();

        let result = match self.orbit_integration {
            OrbitIntegration::RapierForce => {
                if dt > MAX_STEP_DT {
                    let substeps = (dt / MAX_STEP_DT).ceil() as u32;
                    let sub_dt = dt / substeps as f64;
                    for _ in 0..substeps {
                        self.step_via_rapier_force(sub_dt);
                    }
                    StepResult::Substepped { substeps, sub_dt }
                } else {
                    self.step_via_rapier_force(dt);
                    StepResult::Stepped(dt)
                }
            }
            OrbitIntegration::Verlet
            | OrbitIntegration::Yoshida4
            | OrbitIntegration::ForestRuth8
            | OrbitIntegration::Yoshida4Kahan
            | OrbitIntegration::ForestRuth8Kahan => {
                self.step_via_explicit(dt);
                StepResult::Stepped(dt)
            }
        };

        // --- Arena: flush body slots (Rust→Java zero-copy readback) ---
        if let Some(ref arena) = self.shared_arena {
            arena.flush_all_bodies(&self.bodies);
        }

        // 每条 step 尾部清空本步攒下的命令合力（显式路径用），避免泄漏到下步。
        self.arena_cmd_forces.clear();

        result
    }

    /// 无线电传播：收集反射天体最新位置 → 推进一轮。
    /// 由 Java 在天体 step 之后显式调用（节点/信号已提交）。
    pub fn radio_step(&mut self) {
        let Some(radio) = self.radio.as_mut() else {
            return;
        };

        // 先取出反射体句柄/半径，避免与下方 radio.step(&mut self) 借用冲突
        let reflector_desc: Vec<(RigidBodyHandle, f64)> =
            radio.reflectors().iter().map(|r| (r.handle, r.radius)).collect();

        let now_ms = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0);

        // 反射天体位置：直接读本世界刚体（f64，无 Java 拷贝）
        let mut reflector_pos: Vec<crate::radio::ReflectorState> =
            Vec::with_capacity(reflector_desc.len());
        for (handle, radius) in reflector_desc {
            if let Some(pos) = self.bodies.get(handle).map(|b| b.translation()) {
                let (idx, generation) = handle.into_raw_parts();
                reflector_pos.push(crate::radio::ReflectorState {
                    handle_raw: ((idx as u64) << 32) | (generation as u64),
                    pos,
                    radius,
                });
            }
        }
        radio.step(&reflector_pos, now_ms);
    }

    // ==================== Radio 子世界操作 ====================

    /// 启用无线电传播子世界（幂等；已启用则 no-op）。
    pub fn enable_radio(&mut self) {
        if self.radio.is_none() {
            self.radio = Some(crate::radio::RadioWorld::new());
        }
    }

    /// 是否已启用无线电。
    pub fn radio_enabled(&self) -> bool {
        self.radio.is_some()
    }

    /// 注册一个反射天体（行星/恒星等，按刚体句柄 + 半径）。
    /// 返回 false 表示尚未启用 radio 或句柄无效。
    pub fn radio_add_reflector(&mut self, handle: RigidBodyHandle, radius: f64) -> bool {
        let Some(radio) = self.radio.as_mut() else {
            return false;
        };
        if self.bodies.get(handle).is_none() {
            return false;
        }
        radio.add_reflector(handle, radius);
        true
    }

    /// 移除反射天体。
    pub fn radio_remove_reflector(&mut self, handle: RigidBodyHandle) {
        if let Some(radio) = self.radio.as_mut() {
            radio.remove_reflector(handle);
        }
    }

    /// 提交一个无线电收发器（注册或按 id 覆盖）。
    pub fn radio_register_node(&mut self, node: crate::radio::RadioNode) {
        if let Some(radio) = self.radio.as_mut() {
            radio.register_node(node);
        }
    }

    /// 整表覆盖提交收发器（Java 每 tick 传全量在线节点）。
    pub fn radio_set_nodes(&mut self, nodes: Vec<crate::radio::RadioNode>) {
        if let Some(radio) = self.radio.as_mut() {
            radio.set_nodes(nodes);
        }
    }

    /// 注销收发器。
    pub fn radio_unregister_node(&mut self, id: u64) {
        if let Some(radio) = self.radio.as_mut() {
            radio.unregister_node(id);
        }
    }

    /// 提交一个信号（发射）。
    pub fn radio_submit_signal(&mut self, signal: crate::radio::ActiveSignal) {
        if let Some(radio) = self.radio.as_mut() {
            radio.submit_signal(signal);
        }
    }

    /// 批量提交信号（发射列表）。
    pub fn radio_submit_signals(&mut self, signals: Vec<crate::radio::ActiveSignal>) {
        if let Some(radio) = self.radio.as_mut() {
            for signal in signals {
                radio.submit_signal(signal);
            }
        }
    }

    /// 取走本轮传播结果（信号 id、接收节点 id、功率、频移）。
    pub fn radio_take_results(&mut self) -> Vec<crate::radio::RadioResult> {
        self.radio
            .as_mut()
            .map(|r| r.take_results())
            .unwrap_or_default()
    }

    pub fn radio_node_count(&self) -> usize {
        self.radio.as_ref().map_or(0, |r| r.node_count())
    }

    pub fn radio_reflector_count(&self) -> usize {
        self.radio.as_ref().map_or(0, |r| r.reflector_count())
    }

    /// 批量推进 `n` 个步长，每步 `dt` 秒。等价于循环 `step(dt)`，但把 `dt`
    /// 合法性校验前置一次。返回累计诊断：
    /// - `Ok(())`：所有步都正常推进。
    /// - `Err(reason)`：`dt` 非法，整批未推进。
    pub fn step_n(&mut self, dt: f64, n: u32) -> Result<(), StepSkipReason> {
        if !dt.is_finite() {
            return Err(StepSkipReason::NonFinite);
        }
        if dt <= 0.0 {
            return Err(StepSkipReason::NonPositive);
        }
        if dt > 30.0 {
            return Err(StepSkipReason::TooLarge);
        }
        for _ in 0..n {
            // step 内部对合法 dt 不会再返回 Skipped，这里丢弃每步的 StepResult。
            let _ = self.step(dt);
        }
        Ok(())
    }

    /// 取所有 n-body 源（只读）。
    pub fn n_body_sources(&self) -> &[NBodySource] {
        &self.n_body_sources
    }

    /// 测试/诊断用：本世界是否含有「不规则质量分布」n-body 源。由
    /// `refresh_n_body_sources` 每次重算，用于短路 `total_acceleration` 的近场
    /// 分支整体判定。生产路径不依赖；仅供集成测试验证该标志的正确性。
    pub fn has_irregular_sources(&self) -> bool {
        self.has_irregular_sources
    }

    /// 取所有天体引力源（只读）。
    pub fn celestials(&self) -> &[CelestialSource] {
        &self.celestials
    }

    /// 取内部 `RigidBodySet`（只读）——供外部诊断/快照用（如
    /// [`crate::integrator::snapshot_source_positions`] 需要遍历体位置）。
    pub fn bodies(&self) -> &RigidBodySet {
        &self.bodies
    }

    /// 测试/诊断用：按 arena index 取 per-body Kahan 累加态（`None` 表示该槽未
    /// 初始化，返回默认零态）。生产路径不依赖；仅供集成测试比对累加态。
    pub fn kahan_state_debug(
        &self,
        idx: usize,
    ) -> (mps_formula::math::KahanVec3, mps_formula::math::KahanVec3) {
        self.kahan_state.get(idx).and_then(|s| *s).unwrap_or((
            mps_formula::math::KahanVec3::default(),
            mps_formula::math::KahanVec3::default(),
        ))
    }

    /// 取 n-body 互引力的软化平方项（m²）。
    pub fn n_body_softening_sq(&self) -> f64 {
        self.n_body_softening_sq
    }

    /// 取太阳位置（光压方向参考）。
    pub fn sun_position(&self) -> Vector {
        self.sun_position
    }

    /// 取轨道积分模式（P2.18：clone_shallow 测试需要 introspect）。
    pub fn orbit_integration(&self) -> OrbitIntegration {
        self.orbit_integration
    }

    /// 取当前中心天体引用（P2.18：clone_shallow 测试需要 introspect）。
    pub fn central_body(&self) -> Option<&'static mps_formula::celestial_data::CelestialBody> {
        self.central_body
    }

    /// 旧路径：力注入 → rapier 推进。
    fn step_via_rapier_force(&mut self, dt: f64) {
        // 把 integration_parameters.dt 对齐到本次子步实际 dt；rapier.pipeline.step
        // 内部所有积分都读这个值。
        self.integration_parameters.dt = dt;
        // 0. 每步先清掉上一轮累积的 user_force。Rapier 不会自动重置
        //    add_force 累加进去的力（见 rapier `reset_forces` 文档）。
        //
        //    P1.12 / L3：复用 `scratch_tasks` 的动态体 handle 列表做 reset，避免
        //    每子步 `for b in self.bodies.iter_mut() { if b.is_dynamic() {...} }`
        //    把全体 body（含 fixed/kinematic）扫一遍做 `is_dynamic()` 过滤。当
        //    `cosmosWorldStep(world, 5.0)` 拆 N 子步（MC 轨道压缩推进常用）时，
        //    每子步从 O(total_bodies) 降到 O(dynamic_bodies) × `RigidBodySet::get_mut`
        //    （arena-idx O(1)）。仅重置"被 `apply_forces` 写过 user_force 的动态体"，
        //    fixed / kinematic 不会被 `add_force`，也不需要 reset。
        //
        //    调用顺序：`collect_dynamic_tasks` 把 scratch_tasks 落成动态体的
        //    `(handle, pos, vel, mass, p)` 后，`inject_forces_from_collected_tasks`
        //    复用同一份 scratch_tasks 做 force 注入 + reset。
        self.collect_dynamic_tasks();
        self.refresh_n_body_sources();
        for &(handle, _, _, _, _) in &self.scratch_tasks {
            if let Some(b) = self.bodies.get_mut(handle) {
                b.reset_forces(false);
                b.reset_torques(false);
            }
        }

        // 1. 计算并注入每体的合力（天体引力 + n-body + 环境扰动）—— 复用上方已
        //    collected 的 scratch_tasks，不再重调 collect/refresh（L3：子步切分
        //    下原 apply_forces 每 substep 跑两遍 collect，N=1000 卫星×4 substep ≈
        //    8000 重复浅扫/tick；现已减半）。
        self.inject_forces_from_collected_tasks(dt);

        // 2. Rapier 推进
        self.pipeline.step(
            self.gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            &(),
            &(),
        );
    }

    /// 显式积子路径：把轨道积分从 rapier 力律里抽出来，由 [`crate::integrator`]
    /// 按 `orbit_integration` 选定的辛积子推进 (translation, linvel)，rapier 仍
    /// 跑碰撞/约束/姿态。
    ///
    /// 与 `step_via_rapier_force` 同理：rapier 的 `pipeline.step` 末尾
    /// `advance_to_final_positions` 会用 solver 内部积分得到的 `next_position`
    /// **覆盖** `pos.position`，把显式积子写回的 translation 抹掉。为避免这种
    /// 窜改，本路径不调 `pipeline.step`，而是手写一个最小推进：
    ///   1. 显式积子把 translation/linvel 推进 dt（同步 `pos.next_position`）。
    ///   2. collider 跟随刚体位移更新（无 collider 时空跑）。
    ///   3. 姿态/角速度按 damping 单独积分（无外力矩时与 rapier writeback 等价）。
    ///
    /// 暂不处理碰撞/关节约束求解 —— 太空场景默认不插入 collider，约束为空；
    /// 若未来需要在此路径下处理对接约束，应在此处插入一次 velocity-only 的
    /// 约束求解，避免 advance_to_final_positions 把显式位置覆盖。
    fn step_via_explicit(&mut self, dt: f64) {
        let substeps = self.verlet_substeps.max(1) as usize;
        let sub_dt = dt / substeps as f64;

        for _ in 0..substeps {
            self.explicit_substep(sub_dt);
        }

        self.sync_colliders_after_verlet();
    }

    /// Verlet 路径结束后的 collider 同步：
    /// rapier 的 `ColliderSet` 不会在没跑 pipeline 时自动跟随刚体。
    /// 对"挂在刚体上"的 collider（`parent` 非空），其 world 位姿 =
    /// `parent_body.position * pos_wrt_parent`；这里按这条链路重算写回。
    fn sync_colliders_after_verlet(&mut self) {
        // P1.7: 跨步复用 `scratch_collider_updates` 缓冲，避免 `Vec::new()` + `collect()`
        // 每步堆分配。element 是 `(collider_handle, world_pose)`，后续一次性写回。
        self.scratch_collider_updates.clear();
        // 先快照 (collider_handle, parent_handle, offset)，再写回，避开同时借用。
        for (h, co) in self.colliders.iter() {
            if let Some(ph) = co.parent()
                && let Some(b) = self.bodies.get(ph)
            {
                let offset = co.position_wrt_parent().copied();
                let world = b.position() * offset.unwrap_or_default();
                self.scratch_collider_updates.push((h, world));
            }
        }
        for (handle, world) in self.scratch_collider_updates.drain(..) {
            if let Some(co) = self.colliders.get_mut(handle) {
                co.set_position(world);
            }
        }
    }

    /// 一次显式积子子步：按 `orbit_integration` 选定积子对所有动态刚体推进 dt。
    fn explicit_substep(&mut self, dt: f64) {
        // H. 阶段耗时剖分（env-gated，默认零开销）。
        // 仅当环境变量 `COSMOS_PROFILE=1` 时启用；每子步把 4 段耗时累加进
        // thread_local 累加器，并在 `explicit_substep` 首次调用打印表头。
        // 不设该变量时整段被 `if` 短路，编译器完全剥离，不影响任何数值/语义
        // （守「原方法不变」）。
        macro_rules! profile_phase {
            ($name:expr, $body:block) => {{
                if std::env::var("COSMOS_PROFILE").as_deref() == Ok("1") {
                    let _t0 = std::time::Instant::now();
                    let _r = $body;
                    crate::__cosmos_profile_record($name, _t0.elapsed());
                    _r
                } else {
                    $body
                }
            }};
        }

        // 1. 收集动态体快照 (handle, pos, vel, mass) 到复用缓冲 +
        //    填充每体 perturbation。P0.2: 与 `apply_forces` 共用一段热路径，
        //    避免同步漂移到两份独立实现（曾因此 mps-cosmos 计算出的引力
        //    和显式积子读到的引力不一致）。
        profile_phase!("collect", {
            self.collect_dynamic_tasks();
        });

        // 2. n-body 源质心位置 + 姿态快照写入复用缓冲（按 arena index O(1) 查）。
        profile_phase!("refresh", {
            self.refresh_n_body_sources();
        });

        // 3. 构造本子步共享的 AccelContext（含相对论修正分支开关）。
        let ctx = crate::integrator::AccelContext {
            celestials: &self.celestials,
            n_body_sources: &self.n_body_sources,
            source_positions: &self.scratch_source_positions,
            source_rotations: &self.scratch_source_rotations,
            source_pos_gm: self.scratch_source_pos_gm.as_slice(),
            softening_sq: self.n_body_softening_sq,
            central_body: self.central_body,
            sun_position: self.sun_position,
            relativistic: self.relativistic_correction,
            has_irregular_sources: self.has_irregular_sources,
            nb_parallel: crate::integrator::nb_parallel_for_workload(
                self.scratch_tasks.len(),
                self.n_body_sources.len(),
            ),
            ff_simd: crate::integrator::ff_simd_enabled(),
        };

        // 3.5 并行预计算每体的初始加速度 `a0 = total_acceleration(pos, vel, ...)`。
        // 每个体的 a0 只读冻结快照 `ctx` + 自身 (pos,vel,mass,handle,pert) 标量，
        // **不读任何其它体的可变状态**——故按体并行求值与串行逐位一致（不加、不
        // 减、不重排任何浮点运算顺序）。这是 n-body 互引力 O(N·M) 最贵的一次
        // 评估（Verlet 每体恰好一次；高阶积子内部还会在偏移位置上重估，但首评
        // 即 a0，后续在并行预计算之外按原顺序串行，数值不变）。单线程下
        // `par_iter` 退化为顺序，无额外开销；多体场景由 rayon 自动分到多核。
        // `mode` 须在并行块前定下来（闭包里要用它决定走哪条积子 + 是否跳过 ho）。
        let mode = self.orbit_integration;
        let n_tasks = self.scratch_tasks.len();
        // 3.5 并行预计算每体的「初始加速度」与「高阶推进」(+ Kahan 推进)。
        // 每个体的求值都只依赖冻结快照 `ctx` + 自身 (pos,vel,mass,handle,pert)
        // 标量（+ Kahan 累加态的**拷贝**），**不读任何其它体的可变状态**——
        // 故按体并行求值与串行逐位一致（不加、不减、不重排任何浮点运算顺序）。
        // n-body 互引力 O(N·M) 最贵的评估（Verlet 的 a0、高阶积子的首评 + 偏移
        // 位置重估）都在这里并行完成；单线程下 `par_iter` 退化为顺序、无额外
        // 开销，多体场景由 rayon 自动分到多核。串行循环只做「预计算结果写回
        // body / Kahan 态」。Kahan 模式的 per-body 累加态：首次出现按 body 当前
        // pos/vel 初始化，之后每步由并行 advance 产出新态写回（见 3.5 + 写回段）。
        // 先确保 `kahan_state` 足够长且本 task 槽已初始化，再把累加态拷出到
        // `self.kahan_src_buf`（跨子步复用缓冲，纯 Copy），供并行 advance 使用。
        // 复用而非每子步新建：稳态零堆分配。
        self.kahan_src_buf.clear();
        self.kahan_src_buf.reserve(n_tasks);
        for &(h, p, v, _m, _pt) in &self.scratch_tasks {
            let idx = h.into_raw_parts().0 as usize;
            if idx >= self.kahan_state.len() {
                self.kahan_state.resize(idx + 1, None);
            }
            let need_kahan = matches!(
                mode,
                OrbitIntegration::Yoshida4Kahan | OrbitIntegration::ForestRuth8Kahan
            );
            if need_kahan && self.kahan_state[idx].is_none() {
                self.kahan_state[idx] = Some((
                    mps_formula::math::KahanVec3::new(crate::integrator::ffi_vec3_pub(p)),
                    mps_formula::math::KahanVec3::new(crate::integrator::ffi_vec3_pub(v)),
                ));
            }
            self.kahan_src_buf
                .push(self.kahan_state.get(idx).and_then(|s| *s));
        }
        // 复用 `self.advance_buf`：先 `clear()`（保留容量），再用 `par_extend`
        // 原地追加预计算结果，避免每子步 `Vec::with_capacity` + `collect()` 的
        // 堆分配。rayon `unzip` 仅支持 2 元组，这里手动收成 3 元组 vec。
        self.advance_buf.clear();
        profile_phase!("advance", {
            if n_tasks > 0 {
                self.scratch_tasks
                    .par_iter()
                    .with_min_len(16)
                    .map(|&(h, pos, vel, mass, perturb)| {
                        let a0 = crate::integrator::total_acceleration(
                            pos,
                            vel,
                            mass,
                            h,
                            &ctx,
                            perturb.as_ref(),
                        );
                        // Verlet 模式只用到 a0；高阶/Kahan 推进按需计算，避免冗余的
                        // 额外 `total_acceleration` 评估（与 a0 同输入，纯浪费）。
                        let (ho, kaho) = if matches!(mode, OrbitIntegration::Verlet) {
                            (None, None)
                        } else {
                            // 高阶推进纯函数：冻结 v0、只评估快照上加速度，与串行
                            // 内联调用逐位一致。
                            let ho = crate::integrator::advance_highorder(
                                mode, pos, vel, mass, h, perturb, &ctx, dt,
                            );
                            // Kahan 推进纯函数：用拷贝出来的累加态，跨步补偿不重置，
                            // 与串行 `explicit_highorder_kahan_step` 逐位一致。
                            let kaho = self.kahan_src_buf[h.into_raw_parts().0 as usize].map(
                                |(kp, kv)| {
                                    crate::integrator::advance_highorder_kahan(
                                        mode, pos, vel, kp, kv, mass, h, perturb, &ctx, dt,
                                    )
                                },
                            );
                            (Some(ho), kaho)
                        };
                        (a0, ho, kaho)
                    })
                    .collect_into_vec(&mut self.advance_buf);
            }
        });
        // B2: 写回循环并行化。拆开 `self` 的多个可变字段，对 `scratch_tasks` 做
        // `par_iter` 并发写回 body / Kahan 态。
        //
        // 安全性（why unsafe 是 sound 的）：每个 task 对应一个唯一动态刚体
        // `handle`，`bodies.get_mut(handle)` 只写该体；`kahan_idx` 由 handle 派生，
        // `kahan_state[kahan_idx]` 是该体独占的累加态槽。两个可变字段的写入槽在
        // 不同 task 之间**完全不重叠**，故并发写回无数据竞争、无别名——与串行
        // **逐位一致**（每体终态只取决于自身预计算结果，跨体写回顺序无关）。
        // rayon 的 `for_each` 要求 `Fn`，无法在闭包内对捕获的 `&mut` 字段做每调用
        // 重借用；这里用裸指针把「互不重叠的可变借用」显式化，等价于「每体一把锁」
        // 但零开销。kahan_state 已在 3.5 段按最大 idx 预扩容，此处直接赋值（并发
        // resize 才是数据竞争，已规避）。`&ctx` / `advance_buf` / `scratch_tasks` /
        // `arena_cmd_forces` 皆为共享只读。
        profile_phase!("writeback", {
            {
                let CosmosWorld {
                    bodies,
                    kahan_state,
                    arena_cmd_forces,
                    scratch_tasks,
                    advance_buf,
                    ..
                } = self;
                // 把两个可变字段降级为裸指针地址（usize，可 `Send`/`Sync`），在闭包内
                // 按需重借为 `&mut`。槽不重叠，故并发解引用写不同内存是安全的。
                let bodies_addr = bodies as *mut rapier3d::dynamics::RigidBodySet as usize;
                let kahan_addr = kahan_state
                    as *mut Vec<
                        Option<(mps_formula::math::KahanVec3, mps_formula::math::KahanVec3)>,
                    > as usize;
                scratch_tasks
                    .par_iter()
                    .enumerate()
                    .with_min_len(16)
                    .for_each(|(i, &(handle, _pos, _vel, mass, perturbation))| {
                        // SAFETY: 见上。`bodies_addr`/`kahan_addr` 是 `self` 字段的地址，
                        // 本闭包运行期间 `self` 不被其它地方借用；每个 task 写唯一槽。
                        let bodies =
                            unsafe { &mut *(bodies_addr as *mut rapier3d::dynamics::RigidBodySet) };
                        let kahan_state = unsafe {
                            &mut *(kahan_addr
                                as *mut Vec<
                                    Option<(
                                        mps_formula::math::KahanVec3,
                                        mps_formula::math::KahanVec3,
                                    )>,
                                >)
                        };
                        let kahan_idx = handle.into_raw_parts().0 as usize;
                        // 高阶/Kahan 推进已并行预计算（见 3.5），直接取用，避免串行
                        // 重复求值。
                        let (a0, ho, kahan) = &advance_buf[i];

                        let body = match bodies.get_mut(handle) {
                            Some(b) => b,
                            None => return,
                        };

                        match mode {
                            OrbitIntegration::Verlet => {
                                let a0 = *a0;
                                crate::integrator::verlet_step(
                                    body,
                                    a0,
                                    &ctx,
                                    mass,
                                    handle,
                                    perturbation,
                                    dt,
                                );
                            }
                            OrbitIntegration::Yoshida4 | OrbitIntegration::ForestRuth8 => {
                                #[allow(clippy::clone_on_copy)]
                                let (r1, v1) = ho.expect("ho 预计算应已就绪").clone();
                                body.set_translation(r1, false);
                                body.set_linvel(v1, false);
                            }
                            OrbitIntegration::Yoshida4Kahan
                            | OrbitIntegration::ForestRuth8Kahan => {
                                #[allow(clippy::clone_on_copy)]
                                let (r1, v1, kp, kv) = kahan.expect("kahan 预计算应已就绪").clone();
                                // 把并行 advance 产出的新 Kahan 态写回 world 缓存。
                                kahan_state[kahan_idx] = Some((kp, kv));
                                body.set_translation(r1, false);
                                body.set_linvel(v1, false);
                            }
                            OrbitIntegration::RapierForce => {
                                unreachable!("RapierForce 不走显式路径")
                            }
                        }

                        // 命令环注入的合力（AddForce，显式路径）：在积子推进之上叠加一个
                        // 常加速度半隐式欧拉修正（F 在本子步视为常量，与积子同阶精度）。
                        // 不改动 `integrator` 签名即可让 arena 命令力在所有显式积子模式下生效。
                        if mass > 0.0 {
                            let f = arena_cmd_forces.get(kahan_idx).copied().flatten();
                            if let Some(f) = f {
                                let a_cmd = f / mass;
                                let dv = a_cmd * dt;
                                let v_now = body.linvel();
                                body.set_linvel(v_now + dv, false);
                                let r_now = body.translation();
                                body.set_translation(r_now + a_cmd * (0.5 * dt * dt), false);
                            }
                        }
                    });
            }
        });
    }

    /// 收集动态刚体快照 `(handle, pos, vel, mass, perturbation)` 到
    /// `scratch_tasks`（跨帧复用，避免 `vec!` 分配）。同时用 arena index 把
    /// 对应的 `perturbations` 配置就地写回 `task.4`，规避同时 mut 借用
    /// `self.scratch_tasks` 和 immut 借用 `self.perturbations` 的冲突。
    ///
    /// 被 [`Self::explicit_substep`] (Verlet 积子) 和 [`Self::apply_forces`]
    /// (RapierForce 路径) 共用。
    ///
    /// **§1 优化（消除每子步 Vec 重新分配）**：原实现每子步 `self.scratch_tasks =
    /// collected` 把复用缓冲重新赋值成一个全新 `Vec`，导致每帧每子步 2 次堆分配 +
    /// 1 次 drop，直接破坏 A1/A2「稳态零堆分配」目标。现改为：
    /// - `scratch_handles` 复用缓冲 `clear()` + `par_extend` 收集动态体 handle；
    /// - `scratch_tasks` 复用缓冲 `clear()`（保留容量）+ `par_extend` 原地追加快照，
    ///   不再新建临时 `Vec`、不再重新赋值。稳态下两缓冲容量随工作集收敛后零分配。
    fn collect_dynamic_tasks(&mut self) {
        self.scratch_handles.clear();
        // 动态体数量在首帧工作集化时收敛，后续帧不再增长；先 reserve 避免零散
        // push 触发的 incremental grow。
        let n_dynamic_hint = self.bodies.len();
        self.scratch_handles.reserve(n_dynamic_hint);
        self.scratch_tasks.clear();
        self.scratch_tasks.reserve(n_dynamic_hint);
        // B1: 每体只读快照写入各自槽。先串行收集动态体 handle（廉价，
        // `RigidBodySet::iter` 可用），再 `par_extend` 过 handle 并行取 translation/
        // linvel/mass 快照（读 `bodies.get` 为 `&self` 共享读，`RigidBodySet` 是
        // `Sync`，跨线程安全）—— 真正耗时的快照并行化。顺序由 handle 收集保序，
        // 每个体只写自己的槽、无别名，与串行逐位一致（内容完全相同，仅求值并发）。
        // 复用 `scratch_handles`：每子步 `clear()` + `extend`（std 迭代器复用容量，
        // 仅在容量不足时增长，不做每子步新建 Vec）。
        self.scratch_handles.extend(
            self.bodies
                .iter()
                .filter(|(_, b)| b.is_dynamic())
                .map(|(h, _)| h),
        );
        // 复用 `scratch_tasks`：原地 `par_extend`（rayon 并行迭代器），不新建临时
        // Vec、不重新赋值。
        self.scratch_tasks
            .par_extend(self.scratch_handles.par_iter().map(|&h| {
                let b = self.bodies.get(h).expect("handle 来自 bodies.iter，必存在");
                (h, b.translation(), b.linvel(), b.mass(), None)
            }));
        // 填充每体 perturbation（Copy）。先单独线性扫一遍 perturbations（与
        // scratch_tasks 按 arena index 对齐），再就地写回 task.4，规避
        // "在 iter_mut 借用内同时不可变借 self.perturbations" 的借用冲突。
        for task in self.scratch_tasks.iter_mut() {
            let idx = task.0.into_raw_parts().0 as usize;
            let p = self
                .perturbations
                .get(idx)
                .and_then(|c| c.as_ref())
                .copied();
            task.4 = p;
        }
    }

    /// n-body 源质心位置 + 姿态快照写入复用缓冲（按 arena index O(1) 查）。
    /// 同时按 `body.mass()` 重算每源的 `gm`，并对不规则源按比例缩放每个
    /// `MassPoint.gm`（保持几何结构不变，仅质量跟随刚体走）。
    ///
    /// 被 [`Self::explicit_substep`] (Verlet 积子) 和 [`Self::apply_forces`]
    /// (RapierForce 路径) 共用 —— P0.2: 原先两处分别内联，曾出现单边更新
    /// 漂移（explore_substep 已内联版本会跳过空源 fast-path 但 apply_forces
    /// 旧版本仍用空源 `gm=0` 分支，导致 RapierForce 路径下新加入的源 gm 不
    /// 跟随质量变化，引力数值与显式积子错位）。
    fn refresh_n_body_sources(&mut self) {
        // P2.15: avoid the previous `clear() + resize(n, ZERO/IDENTITY)` which
        // zeroed the entire buffer every frame. We instead:
        //   1. `truncate(bodies.len())` — drops the tail when bodies shrank
        //      (drop happens in-place, no alloc; remaining slots retain last
        //      frame's values which are about to be overwritten below).
        //   2. `resize_with(bodies.len(), default)` — grows only when bodies
        //      grew (zeroes just the *new* tail, not the whole buffer).
        //   3. The `idx < scratch.len()` guard at the write site already covers
        //      removed-source slots — those slots keep their old value but the
        //      source's `gm` was just set to 0 in this same iteration, so the
        //      stale position data is never read (downstream `celestial_*` /
        //      `n_body_*` multiply by `gm`).
        // On a stable or shrinking `bodies.len()` (steady state — the common
        // case), neither truncate nor resize_with does any element writes: the
        // previous full-buffer `clear()+resize(n)` zero-fill is eliminated.
        let bodies_len = self.bodies.len();
        self.scratch_source_positions.truncate(bodies_len);
        if self.scratch_source_positions.len() < bodies_len {
            self.scratch_source_positions
                .resize_with(bodies_len, || Vector::ZERO);
        }
        self.scratch_source_rotations.truncate(bodies_len);
        if self.scratch_source_rotations.len() < bodies_len {
            self.scratch_source_rotations
                .resize_with(bodies_len, || Rotation::IDENTITY);
        }
        // B1: 源之间完全独立（各写自己 idx 槽、只读 bodies）。拆成两段并行：
        //   Pass 1 —— `par_iter_mut` 刷新每个源的 `gm` 与 `points` 缩放（拆开
        //     `self` 可变借用：`bodies` 只读 + `n_body_sources` 可写，互不 alias）。
        //   Pass 2 —— 并行读每个源刚体的 translation/rotation，收集 `(idx,pos,rot)`，
        //     再串行写回 `scratch_source_*`（idx 唯一、写回廉价，与串行逐位一致）。
        {
            let CosmosWorld {
                bodies,
                n_body_sources,
                ..
            } = self;
            n_body_sources.par_iter_mut().for_each(|s| {
                let Some(b) = bodies.get(s.handle) else {
                    // 源刚体已移除：清零引力参数（含质点 gm），位置/姿态保持占位
                    s.gm = 0.0;
                    for mp in &mut s.points {
                        mp.gm = 0.0;
                    }
                    return;
                };
                let new_gm = gm_from_mass(b.mass());
                // 不规则源：按新质量/原 gm 比例缩放每个 MassPoint.gm（保持几何结构
                // 不变、只是质量跟着刚体走）。
                if !s.points.is_empty() && s.gm > 0.0 {
                    let scale = new_gm / s.gm;
                    if (scale - 1.0).abs() > 1e-12 {
                        for mp in &mut s.points {
                            mp.gm *= scale;
                        }
                    }
                }
                s.gm = new_gm;
            });
        }
        // Pass 2: 并行取每个源刚体的 world 位姿，串行写回各自 idx 槽；同时攒
        // SOA 表 `scratch_source_pos_gm`（与 `n_body_sources` 同序），供全 monopole
        // 热路径只读一张连续表（缓存局部性更好）。idx 唯一、写回廉价。
        self.scratch_source_pos_gm.clear();
        self.scratch_source_pos_gm
            .reserve(self.n_body_sources.len());
        let snaps: Vec<(usize, Vector, Rotation, f64)> = self
            .n_body_sources
            .par_iter()
            .map(|s| {
                let idx = s.handle.into_raw_parts().0 as usize;
                match self.bodies.get(s.handle) {
                    // 已移除源（gm 已置 0）：占位零位姿；下游乘 gm=0 不读，等价。
                    None => (idx, Vector::ZERO, Rotation::IDENTITY, 0.0),
                    Some(b) => (idx, b.translation(), *b.rotation(), s.gm),
                }
            })
            .collect();
        for (idx, p, r, gm) in snaps {
            if idx < self.scratch_source_positions.len() {
                self.scratch_source_positions[idx] = p;
                self.scratch_source_rotations[idx] = r;
            }
            self.scratch_source_pos_gm.push((p, gm));
        }
        // 聚合「是否存在不规则质量分布源」：带 `points` 且 `near_field_threshold > 0`
        // 的源才计入。用于 `total_acceleration` 短路近场 O(P) 分支的整体判定
        // （数值惰性：false 时该分支本就不触发）。纯只读聚合，无额外分配。
        let has_irregular = self
            .n_body_sources
            .iter()
            .any(|s| !s.points.is_empty() && s.near_field_threshold() > 0.0 && s.gm > 0.0);
        self.has_irregular_sources = has_irregular;
    }

    /// 基于 `scratch_tasks` 已构建好的动态体列表 + `scratch_source_positions` /
    /// `scratch_source_rotations` 快照做一次 force 注入。调用方必须**先**调过
    /// [`Self::collect_dynamic_tasks`] 与 [`Self::refresh_n_body_sources`]，否则
    /// `scratch_tasks` 是空（首次）/ stale（与 body 数不同步）。
    ///
    /// 不重置 user_force —— 调用方负责先 reset_forces（rapier 不会自动清掉
    /// 上轮 `add_force` 累积量，见 rapier doc）。
    ///
    /// P1.12 / L3：原 `apply_forces` 的 force 注入主体；保留为独立方法以让
    /// `step_via_rapier_force` 在 reset_forces 与力注入之间插入"按 scratch_tasks
    /// 精准 reset dynamic bodies"步骤而无需重复 collect_dynamic_tasks。
    ///
    /// 多线程：逐体合力求值只读冻结快照（`scratch_tasks`、源位姿快照、
    /// `celestials` / `n_body_sources` / `central_body` / `sun_position`），写回
    /// 只落在本体的 force / torque 累加槽——按体并行（`with_min_len(16)`，与
    /// 显式路径的 advance / 写回段同款）与串行**逐位一致**：每体的计算序列一个
    /// 浮点运算都不变，跨体的 `add_force` 写入槽互不重叠、顺序无关。唯一需要
    /// 读 `bodies` 的潮汐自旋采样被提前到串行预备段（潮汐默认关闭，常见配置
    /// 下整段退化为一次 `any()` 判定）。
    fn inject_forces_from_collected_tasks(&mut self, dt: f64) {
        let n_tasks = self.scratch_tasks.len();
        if n_tasks == 0 {
            return;
        }

        // 潮汐（Hut 1981）需要每个体的角速度。并行阶段对 `bodies` 只做
        // `get_mut` 写回（裸指针模式，见下），不能与共享读并发——把「需要潮汐」
        // 配置下的 spin 采样提前到串行预备段。仅当存在 enable_tidal 扰动时才
        // 真正遍历 bodies；默认关闭 → 这里是一次 O(N) `any()` 判定。
        let need_tidal = self.central_body.is_some()
            && self.scratch_tasks.iter().any(|(_, _, _, _, pt)| {
                pt.as_ref()
                    .is_some_and(|c| c.enable_tidal && c.tidal_radius > 0.0)
            });
        let tidal_spins: Vec<Vector> = if need_tidal {
            self.scratch_tasks
                .iter()
                .map(|&(handle, _, _, _, _)| {
                    self.bodies
                        .get(handle)
                        .map(|b| {
                            let av = b.angvel();
                            Vector::new(av.x, av.y, av.z)
                        })
                        .unwrap_or(Vector::ZERO)
                })
                .collect()
        } else {
            Vec::new()
        };

        {
            let CosmosWorld {
                celestials,
                n_body_sources,
                scratch_source_positions,
                scratch_source_rotations,
                n_body_softening_sq,
                central_body,
                sun_position,
                bodies,
                scratch_tasks,
                ..
            } = self;
            // 只读字段降为共享引用 / Copy 标量（跨线程共享安全：显式路径的
            // advance 段已按同样形状把 `AccelContext` 共享给 rayon worker）。
            let celestials: &[crate::gravity::CelestialSource] = celestials;
            let n_body_sources: &[crate::gravity::NBodySource] = n_body_sources;
            let scratch_source_positions: &[Vector] = scratch_source_positions;
            let scratch_source_rotations: &[Rotation] = scratch_source_rotations;
            let softening_sq = *n_body_softening_sq;
            let sun_position = *sun_position;
            let central_body = *central_body;
            // 把唯一可变字段降级为裸指针地址（usize，可 `Send`/`Sync`），在闭包内
            // 按需重借为 `&mut`。槽不重叠，故并发解引用写不同内存是安全的。
            let bodies_addr = std::ptr::from_mut(&mut *bodies) as usize;
            scratch_tasks
                .par_iter()
                .enumerate()
                .with_min_len(16)
                .for_each(|(i, &(handle, pos, vel, mass, perturbation))| {
                    let mut total_force = Vector::ZERO;
                    // 潮汐自旋力矩累加（Hut 1981），在扰动块内计算、于下方与力一并施加。
                    let mut tidal_torque_vec = Vector::ZERO;

                    // 天体引力：加速度 × 质量
                    for src in celestials {
                        let accel = celestial_acceleration(pos, src);
                        total_force += accel * mass;
                    }

                    // n-body 互引力：直接 slice 索引取源位置/姿态，跳过空源快路径与闭包虚调用。
                    // 不规则质量分布分支：近场（dist ≤ src.near_field_threshold()）且源带 points
                    // 时按 Σ G·mᵢ·dᵢ/|dᵢ|³ 求和，由源姿态把 local_offset 变到世界坐标——这是
                    // 非球星体方向性拉扯的物理路径。远场/无 points → 单 monopole。
                    if !n_body_sources.is_empty() {
                        let exclude = handle.into_raw_parts().0 as usize;
                        let mut acc_nb = Vector::ZERO;
                        // 与 `integrator::total_acceleration` 同款优化：源快照按 `bodies.len()`
                        // 建好、索引必在界内，用 `get_unchecked` 省掉每源每体的 `Option` +
                        // `unwrap_or` 分支；近场不规则分支（带 `points` 的源）从主 monopole
                        // 循环里摘出，主路径不再判 `!src.points.is_empty()`。
                        debug_assert!(scratch_source_positions.len() >= n_body_sources.len());
                        for src in n_body_sources {
                            let src_idx = src.handle.into_raw_parts().0 as usize;
                            if src_idx == exclude || src.gm <= 0.0 {
                                continue;
                            }
                            let r_j = unsafe { *scratch_source_positions.get_unchecked(src_idx) };
                            let d = r_j - pos;
                            let dist_sq = d.length_squared() + softening_sq;
                            if dist_sq < 1.0 {
                                continue;
                            }
                            let dist = dist_sq.sqrt();
                            let near_threshold = src.near_field_threshold();
                            if !src.points.is_empty()
                                && near_threshold > 0.0
                                && dist <= near_threshold
                            {
                                let rot =
                                    unsafe { *scratch_source_rotations.get_unchecked(src_idx) };
                                for mp in &src.points {
                                    if mp.gm <= 0.0 {
                                        continue;
                                    }
                                    let world = r_j + rot * mp.local_offset;
                                    let d_i = world - pos;
                                    let dist_sq_i = d_i.length_squared() + softening_sq;
                                    if dist_sq_i < 1.0 {
                                        continue;
                                    }
                                    let dist_i = dist_sq_i.sqrt();
                                    acc_nb += d_i * (mp.gm / (dist_sq_i * dist_i));
                                }
                            } else {
                                acc_nb += d * (src.gm / (dist_sq * dist));
                            }
                        }
                        total_force += acc_nb * mass;
                    }

                    // 环境扰动
                    if let Some(cfg) = perturbation {
                        if cfg.enable_drag
                            && let Some(central) = central_body
                        {
                            let altitude = pos.length() - central.equatorial_radius;
                            let density =
                                crate::perturbation::atmosphere_density_at(central, altitude);
                            if density > 0.0 {
                                let atmosphere_vel = angular_velocity_of(central).cross3(pos);
                                if let Some(f) = atmospheric_drag_force(
                                    vel,
                                    atmosphere_vel,
                                    density,
                                    cfg.drag_coefficient,
                                    cfg.area,
                                    mass,
                                ) {
                                    total_force += f;
                                }
                            }
                        }
                        if cfg.enable_solar && cfg.optical_area > 0.0 {
                            let sun_to_body = pos - sun_position;
                            let r = sun_to_body.length();
                            let sun_dir = if r > 1e-9 {
                                -sun_to_body / r
                            } else {
                                Vector::ZERO
                            };
                            let mut f = solar_pressure_force(
                                sun_to_body,
                                sun_dir,
                                cfg.optical_area,
                                cfg.reflectivity,
                                AU,
                            );
                            // 日食（阴影锥）衰减：仅当显式开启且设有 center_body 时生效，
                            // 默认关闭 → 不影响现有光压输出（原方法不变）。
                            if cfg.enable_eclipse
                                && let Some(central) = central_body
                                && central.equatorial_radius > 0.0
                            {
                                let att = crate::perturbation::eclipse_attenuation(
                                    pos,
                                    sun_position,
                                    central.equatorial_radius,
                                    mps_formula::celestial_data::SUN_EQ_RADIUS,
                                );
                                f *= att;
                            }
                            total_force += f;
                        }
                        // 太阳风动压：沿用与光压相同的「太阳→物体」几何方向。
                        if cfg.enable_solar_wind && cfg.solar_wind_area > 0.0 {
                            let sun_to_body = pos - sun_position;
                            if let Some(f) = solar_wind_pressure_force(
                                sun_to_body,
                                vel,
                                cfg.solar_wind_proton_density,
                                cfg.solar_wind_speed,
                                cfg.solar_wind_area,
                            ) {
                                let mut f = f;
                                // 日食（阴影锥）衰减：与光压共用同一几何（中心体在原点、太阳
                                // 在 `sun_position`）。默认关闭 → 不影响现有太阳风输出。
                                if cfg.enable_eclipse
                                    && let Some(central) = central_body
                                    && central.equatorial_radius > 0.0
                                {
                                    let att = crate::perturbation::eclipse_attenuation(
                                        pos,
                                        sun_position,
                                        central.equatorial_radius,
                                        mps_formula::celestial_data::SUN_EQ_RADIUS,
                                    );
                                    f *= att;
                                }
                                total_force += f;
                            }
                        }
                        // Chandrasekhar 动力学摩擦：背景介质的引力拖尾，反速度方向。
                        if cfg.enable_dynamical_friction
                            && cfg.friction_background_density > 0.0
                            && let Some(f) = dynamical_friction_force(
                                mass,
                                cfg.friction_background_density,
                                vel,
                                cfg.friction_coulomb_log,
                            )
                        {
                            total_force += f;
                        }
                        // Hut (1981) 平衡潮自旋同步力矩：默认关闭 → 现有输出逐位不变。
                        // 仅当开启且设有 central_body 作潮汐伴星时计算；力矩施加于下方与力一并注入。
                        // 自旋在并行阶段前已串行采样（见 `tidal_spins`）。
                        if cfg.enable_tidal
                            && cfg.tidal_radius > 0.0
                            && let Some(central) = central_body
                        {
                            let spin = tidal_spins.get(i).copied().unwrap_or(Vector::ZERO);
                            tidal_torque_vec = crate::perturbation::tidal_torque(
                                pos,
                                vel,
                                spin,
                                central.gm,
                                central.equatorial_radius,
                                cfg.tidal_radius,
                                cfg.love_number_k2,
                                cfg.tidal_q,
                            );
                        }
                    }

                    // SAFETY: 见上。`bodies_addr` 是 `self` 字段的地址，本闭包运行期间
                    // `self` 的其它字段仅共享读；每个 task 只 `get_mut` 自己的 handle
                    // → 写入槽完全不重叠。
                    let bodies =
                        unsafe { &mut *(bodies_addr as *mut rapier3d::dynamics::RigidBodySet) };
                    if let Some(body) = bodies.get_mut(handle) {
                        if total_force != Vector::ZERO {
                            body.add_force(total_force, true);
                        }
                        // 潮汐力矩（Hut 1981）：apply_torque_impulse 直接施加角冲量 = 力矩 × dt。
                        if tidal_torque_vec != Vector::ZERO {
                            body.apply_torque_impulse(tidal_torque_vec * dt, true);
                        }
                    }
                });
        }
    }

    #[allow(dead_code)]
    fn perturbation_for(&self, handle: RigidBodyHandle) -> Option<&PerturbationConfig> {
        let idx = handle.into_raw_parts().0 as usize;
        self.perturbations.get(idx).and_then(|c| c.as_ref())
    }
}

impl Clone for CosmosWorld {
    /// 深拷贝整个物理状态。`PhysicsPipeline` 不实现 `Clone`——它是无状态
    /// 工作对象（每次 `step` 重建临时结构），克隆用 `::new()` 恢复。
    fn clone(&self) -> Self {
        // radio 子世界（Java 独占句柄）不随 clone 复制：克隆体无 radio。
        Self {
            pipeline: PhysicsPipeline::new(),
            gravity: self.gravity,
            integration_parameters: self.integration_parameters,
            islands: self.islands.clone(),
            broad_phase: self.broad_phase.clone(),
            narrow_phase: self.narrow_phase.clone(),
            bodies: self.bodies.clone(),
            colliders: self.colliders.clone(),
            impulse_joints: self.impulse_joints.clone(),
            multibody_joints: self.multibody_joints.clone(),
            ccd_solver: self.ccd_solver.clone(),
            celestials: self.celestials.clone(),
            n_body_sources: self.n_body_sources.clone(),
            n_body_softening_sq: self.n_body_softening_sq,
            central_body: self.central_body,
            perturbations: self.perturbations.clone(),
            sun_position: self.sun_position,
            orbit_integration: self.orbit_integration,
            verlet_substeps: self.verlet_substeps,
            adaptive_substeps: self.adaptive_substeps,
            adaptive_tolerance: self.adaptive_tolerance,
            relativistic_correction: self.relativistic_correction,
            kahan_state: self.kahan_state.clone(),
            // scratch buffer 属于每帧工作内存，克隆副本从空开始（复用从首帧起复用）。
            scratch_tasks: Vec::new(),
            scratch_handles: Vec::new(),
            scratch_source_positions: Vec::new(),
            scratch_source_pos_gm: AlignedPosGm::new(),
            scratch_source_rotations: Vec::new(),
            scratch_collider_updates: Vec::new(),
            kahan_src_buf: Vec::new(),
            advance_buf: Vec::new(),
            has_irregular_sources: false,
            shared_arena: None,
            arena_idx_map: Vec::new(),
            arena_idx_map_body_count: 0,
            arena_cmd_forces: Vec::new(),
            radio: None,
        }
    }
}

/// 由天体自转速率与位置近似出大气的惯性系速度 `ω × r`。
/// 这里用叉乘；`^` 在 nalgebra 上也是叉乘别名，但为清晰用显式实现。
fn angular_velocity_of(body: &mps_formula::celestial_data::CelestialBody) -> Vector {
    // 简化：假设自转轴沿 +z，速率 = rotation_rate。
    // 真实模型可后续细化；当前足以给出赤道大气速度的方向。
    Vector::new(0.0, 0.0, body.rotation_rate)
}

// 显式实现 ω × r，避免依赖 nalgebra 的 `^` 运算符可读性。
trait CrossR {
    fn cross3(self, other: Vector) -> Vector;
}
impl CrossR for Vector {
    fn cross3(self, o: Vector) -> Vector {
        Vector::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
}
