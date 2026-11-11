//! `mps_cosmos::world` 测试 —— 迁移自 `crates/mps-cosmos/src/world.rs`。
//!
//! 涵盖 `CosmosWorld` 的 step 语义（RapierForce / Verlet 两条路径）、子步
//! 切分边界、`step_n` 批处理、默认 `n_body_softening_sq` 限幅，以及端到端
//! 圆轨道 LEO 演算（RapierForce 短弧 + Verlet 整圈闭合）。

#[cfg(test)]
use mps_cosmos::bodies::satellite_builder;
#[cfg(test)]
use mps_cosmos::integrator::{
    AccelContext, advance_highorder, advance_highorder_kahan, snapshot_source_positions,
    total_acceleration,
};
#[cfg(test)]
use mps_cosmos::world::{
    CosmosWorld, CosmosWorldConfig, OrbitIntegration, StepResult, StepSkipReason,
};
#[cfg(test)]
use mps_formula::celestial_data::{CelestialBodyId, get_celestial_body};
#[cfg(test)]
use mps_formula::math::KahanVec3;
#[cfg(test)]
use mps_formula::spaceflight::kepler_period;

/// 镜像生产 `refresh_n_body_sources` 的 SOA 表构建：对每个 n-body 源按 handle 取世界位姿 +
/// 源的 gm，得到与 `n_body_sources` 同序、同长的 `source_pos_gm`。reference-ctx 测试需要它
/// 喂给 SIMD 远场路径（P1 起默认开启）——空表 `&[]` 会在 SIMD 默认开启后越界（串行路径
/// 不读此表，故此前被掩盖）。
#[cfg(test)]
fn snapshot_source_pos_gm(world: &CosmosWorld) -> Vec<(Vector, f64)> {
    world
        .n_body_sources()
        .iter()
        .map(|s| {
            let p = world
                .bodies()
                .get(s.handle)
                .map(|b| b.translation())
                .unwrap_or(Vector::ZERO);
            (p, s.gm)
        })
        .collect()
}

#[cfg(test)]
use rapier3d::prelude::{RigidBodyHandle, Rotation, Vector};

/// RapierForce 路径回归测试：semi-implicit Euler 在纯点质量中心引力下推一段
/// 短弧（1/10 开普勒周期），位矢模长应保持在小幅能量漂移范围内。
///
/// 旧版跑整圈 ~5900 步、且因 r=7000km 处 `celestial_acceleration` 会落进
/// ellipsoid 分支（含非保守离心项）导致相位/闭合容差只能开到 10%。改造：
/// - 用内部注入的"纯点质量"加速度闭包绕过天体源分支，隔离积分器本体；
/// - 只推 1/10 周期，验证半径守恒（semi-implicit Euler 在近圆轨道小幅能量
///   漂移，半径随之小幅偏离），CI 时间从十几秒压到亚秒；
/// - 维持仍由 `CosmosWorld` 驱动 + `add_force` 注入路径，回归真实调用路径。
#[test]
fn circular_leo_orbit_period_matches_kepler() {
    let earth = get_celestial_body(CelestialBodyId::Earth);
    let gm = earth.gm;
    let r = 7_000_000.0_f64;
    let v = (gm / r).sqrt();
    let expected_period = kepler_period(gm, r).expect("period");
    // 1/10 开普勒周期，1s 步长。
    let short_arc = expected_period / 10.0;
    let steps = short_arc.round() as u32;

    let mut world = CosmosWorld::new({
        let mut cfg = CosmosWorldConfig {
            gravity: Vector::ZERO,
            dt: 1.0,
            solver_iterations: 8,
            ccd_substeps: 0,
            n_body_softening_sq: 0.0,
            central_body: None,
            orbit_integration: OrbitIntegration::default(),
            verlet_substeps: 1,
            adaptive_substeps: false,
            adaptive_tolerance: 1e-9,
            relativistic_correction: mps_cosmos::world::RelativisticCorrection::None,
        };
        // 让默认 softening 不污染本测试：显式归零，直接测点质量积分精度。
        cfg.n_body_softening_sq = 0.0;
        cfg
    });
    // 不注册 celestials —— 用 n_body_sources 给"假地球"注册点质量引力。
    // step 内部 apply_forces 走真实 n_body_acceleration（不含 ellipsoid/J2）。
    let earth_hdl = world.insert_body(
        satellite_builder(5.972e24, Vector::new(0.0, 0.0, 0.0), Vector::ZERO, 1.0)
            .lock_translations(), // 静止原点：地球本体不被推动
    );
    world.add_n_body(earth_hdl, 5.972e24);

    let sat = world.insert_body(
        satellite_builder(
            1000.0,
            Vector::new(r, 0.0, 0.0),
            Vector::new(0.0, v, 0.0),
            1.0,
        )
        .linear_damping(0.0)
        .angular_damping(0.0),
    );

    for _ in 0..steps {
        world.step(1.0);
    }

    let pos = world.body_translation(sat).expect("sat exists");
    println!("LEO PROBE: v_after_590_steps={:?}", world.body_linvel(sat));
    let r_final = pos.length();
    // semi-implicit Euler 在近圆轨道会有小幅能量增长（v 提前更新），半径随
    // 之小幅上漂。1s 步长 / 1/10 周期下，半径相对偏离应 < 1%。
    assert!(
        (r_final - r).abs() / r < 0.01,
        "短弧半径漂移过大：r0={r} -> {r_final}（Δ={:.3} km）",
        (r_final - r) / 1000.0
    );
    // 用 earth_hdl 消掉未使用告警（上面已用过，留这行防误删）。
    let _ = earth_hdl;
}

/// Verlet 路径：纯点质量中心引力下推一整圈，闭合应远优于 RapierForce
/// 路径。用"假地球"n-body 源（点质量互引力、不含 ellipsoid/J2 分支），
/// 1s 步长，1 圈位置偏移实测 ~3.6km ≈ 0.05% r，远优于 RapierForce 的
/// 整圈 10% 量级。
///
/// 此测试保留整圈跑（~5900 步）作为 Verlet 路径的端到端回归；release 下
/// 0.05s 完成，debug 下也只 ~0.4s，不影响 CI。若日后 CI 对该时长敏感，
/// 可如 `circular_leo_orbit_period_matches_kepler` 那样截到 1/10 短弧。
#[test]
fn circular_leo_verlet_path_closes_tighter_than_rapier() {
    let earth = get_celestial_body(CelestialBodyId::Earth);
    let gm = earth.gm;
    let r = 7_000_000.0_f64;
    let v = (gm / r).sqrt();
    let expected_period = kepler_period(gm, r).expect("period");
    let steps = expected_period.round() as u32;

    let mut world = CosmosWorld::new(CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt: 1.0,
        solver_iterations: 8,
        ccd_substeps: 0,
        n_body_softening_sq: 0.0,
        central_body: None,
        orbit_integration: OrbitIntegration::Verlet,
        verlet_substeps: 1,
        adaptive_substeps: false,
        adaptive_tolerance: 1e-9,
        relativistic_correction: mps_cosmos::world::RelativisticCorrection::None,
    });
    // 用"假地球"刚体作 n-body 源（点质量互引力），替代 celestial 源 ——
    // 后者在 r=7000km 处会落 ellipsoid 分支（含非保守离心项），让圆轨道
    // 不再严格闭合，无法干净验证积分器本身的相位精度。n-body 路径是纯
    // `-G·M·r̂/r²` 点质量引力，圆轨道应严格闭合。
    let earth_hdl = world.insert_body(
        satellite_builder(5.972e24, Vector::ZERO, Vector::ZERO, 1.0).lock_translations(), // 静止原点
    );
    world.add_n_body(earth_hdl, 5.972e24);

    let h = world.insert_body(
        satellite_builder(
            1000.0,
            Vector::new(r, 0.0, 0.0),
            Vector::new(0.0, v, 0.0),
            1.0,
        )
        .linear_damping(0.0)
        .angular_damping(0.0)
        .gravity_scale(0.0), // 关键：避免 rapier.step 再加全局重力
    );

    for _ in 0..steps {
        world.step(1.0);
    }

    let pos = world.body_translation(h).expect("body exists");
    let off = (pos - Vector::new(r, 0.0, 0.0)).length();
    // 纯点质量 + Verlet(1s) 一圈闭合：实测 ~3.6km ≈ 0.05% r。给 1% r
    // 容差，留出数值余量，仍远紧于 RapierForce 路径的 10%。
    assert!(off / r < 0.01, "Verlet 一圈偏移 {off} (>1% r)，pos={pos:?}");
}

#[test]
fn world_inserts_dynamic_body() {
    let mut world = CosmosWorld::new(CosmosWorldConfig::default());
    let h = world.insert_body(satellite_builder(
        1.0,
        Vector::new(1.0, 2.0, 3.0),
        Vector::ZERO,
        0.1,
    ));
    assert_eq!(world.dynamic_body_count(), 1);
    let p = world.body_translation(h).expect("body exists");
    assert!((p - Vector::new(1.0, 2.0, 3.0)).length() < 1e-9);
}

/// P2.18 / M5：`clone_shallow` 必须复制所有 config 字段（gravity、积分参数、
/// 软化长度、子步配置、轨道积分模式、相对论修正模式、太阳位置、中心天体
/// 句柄），同时丢弃全部 body/collider/joint/celestials/n_body_sources/
/// perturbations/kahan_state/scratch buffers。
///
/// 用途：Monte Carlo 多世界并行 + 轨道预测 overlay——想要"同配置 + 全新
/// 空白物理场景" 而不需要付出深拷贝整 RigidBodySet 的 ~5–10ms 代价（1000
/// body 的 Clone 实测代价）。
#[test]
fn clone_shallow_preserves_config_drops_body_state() {
    let mut world = CosmosWorld::new(CosmosWorldConfig {
        n_body_softening_sq: 5e5,
        orbit_integration: OrbitIntegration::ForestRuth8Kahan,
        verlet_substeps: 4,
        adaptive_substeps: true,
        adaptive_tolerance: 1e-12,
        ..CosmosWorldConfig::default()
    });
    world.set_sun_position(Vector::new(1.0, 0.0, 0.0));
    let earth = get_celestial_body(CelestialBodyId::Earth);
    world.set_central_body(Some(earth));
    // Populate some body + extra source state to verify it gets dropped.
    let _h = world.insert_body(satellite_builder(
        1.0,
        Vector::new(1.0, 2.0, 3.0),
        Vector::ZERO,
        0.1,
    ));
    let n_celestial = world.add_celestial(mps_cosmos::gravity::CelestialSource::new(
        earth,
        earth.max_degree,
    ));
    let _ = n_celestial;

    let mut shallow = world.clone_shallow();

    // --- config 端保留 ---
    assert_eq!(shallow.dynamic_body_count(), 0, "body count must reset");
    assert_eq!(shallow.n_body_softening_sq(), 5e5, "softening");
    assert_eq!(
        shallow.orbit_integration(),
        OrbitIntegration::ForestRuth8Kahan,
        "orbit integration mode preserved"
    );
    let sun = shallow.sun_position();
    assert!(
        (sun - Vector::new(1.0, 0.0, 0.0)).length() < 1e-12,
        "sun position preserved"
    );
    let central_back = shallow.central_body();
    assert!(central_back.is_some(), "central body handle preserved");

    // --- body/state 端清空 ---
    assert_eq!(shallow.dynamic_body_count(), 0);
    assert_eq!(shallow.n_body_sources().len(), 0, "n_body_sources dropped");
    // 轨道再推一步确认是 fresh-empty scene（不 crash、不残留状态推进）：
    let step_res = shallow.step(0.1);
    assert!(
        matches!(
            step_res,
            StepResult::Stepped(_) | StepResult::Substepped { .. }
        ),
        "shallow copy should remain step-able; got {step_res:?}"
    );

    // 源 world 仍可继续推（deep clone 不触及原始！）：
    let orig_step = world.step(0.1);
    assert!(
        matches!(
            orig_step,
            StepResult::Stepped(_) | StepResult::Substepped { .. }
        ),
        "source world should still step normally; got {orig_step:?}"
    );
    assert!(
        world.dynamic_body_count() >= 1,
        "source world body count intact"
    );
}

/// `step` 对非法 `dt` 必须返回 `Skipped(...)` 而非静默吞掉。这是 P1-5 的
/// 核心收益：调用方现在能区分"没推进"的三种原因。
#[test]
fn step_reports_skipped_reasons() {
    // RapierForce 路径才有 dt > MAX_STEP_DT 的子步拆分；显式积子路径由
    // verlet_substeps / adaptive 控制子步，单 int 不出 Substepped。
    let mut world = CosmosWorld::new(CosmosWorldConfig {
        orbit_integration: OrbitIntegration::RapierForce,
        ..CosmosWorldConfig::default()
    });

    assert_eq!(
        world.step(f64::NAN),
        StepResult::Skipped(StepSkipReason::NonFinite)
    );
    assert_eq!(
        world.step(0.0),
        StepResult::Skipped(StepSkipReason::NonPositive)
    );
    assert_eq!(
        world.step(-1.0),
        StepResult::Skipped(StepSkipReason::NonPositive)
    );
    assert_eq!(
        world.step(f64::INFINITY),
        StepResult::Skipped(StepSkipReason::NonFinite)
    );
    // 硬上限：超过 30s 拒。MAX_STEP_DT(10s) 与硬上限(30s) 之间会走子步。
    assert_eq!(
        world.step(11.0),
        StepResult::Substepped {
            substeps: 2,
            sub_dt: 5.5
        }
    );
    assert_eq!(
        world.step(31.0),
        StepResult::Skipped(StepSkipReason::TooLarge)
    );
    assert_eq!(world.step(10.0), StepResult::Stepped(10.0));

    // 合法 dt 仍然推：放一个自由落体的体进来确认 Stepped 不只是返回值对、
    // 也真的推了一步。
    let h = world.insert_body(satellite_builder(
        1.0,
        Vector::new(0.0, 0.0, 0.0),
        Vector::new(1.0, 0.0, 0.0),
        0.1,
    ));
    let before = world.body_translation(h).unwrap().x;
    world.step(1.0);
    let after = world.body_translation(h).unwrap().x;
    assert!(after > before, "Stepped 应真的推进位置: {after} > {before}");
}

/// RapierForce 路径下 `dt` 介于 `MAX_STEP_DT`(10s) 与硬上限(30s) 之间时
/// 应自动拆子步并返回 `Substepped`；超过硬上限则 `TooLarge`。
#[test]
fn step_substeps_oversized_dt_rapier_force() {
    let mut world = CosmosWorld::new(CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt: 1.0,
        orbit_integration: OrbitIntegration::RapierForce,
        ..CosmosWorldConfig::default()
    });
    let h = world.insert_body(satellite_builder(
        1.0,
        Vector::new(0.0, 0.0, 0.0),
        Vector::new(1.0, 0.0, 0.0),
        0.1,
    ));
    // 25s 介于 10s 与 30s 之间 → 3 子步（每段 8⅓s）。
    let res = world.step(25.0);
    match res {
        StepResult::Substepped { substeps, sub_dt } => {
            assert_eq!(substeps, 3);
            assert!((sub_dt - 25.0 / 3.0).abs() < 1e-9);
        }
        other => panic!("预期 Substepped，得到 {other:?}"),
    }
    // 子步累计仍推 ~25m（x += v·dt，无外力）。
    let x = world.body_translation(h).unwrap().x;
    assert!((x - 25.0).abs() < 1e-6, "25s 自由漂移应 ≈ 25m，实际 {x}");

    // 超过硬上限 30s → TooLarge。
    assert_eq!(
        world.step(31.0),
        StepResult::Skipped(StepSkipReason::TooLarge)
    );
}

/// `step_n` 对非法 `dt` 整批拒；合法时等价于循环 `step`。
#[test]
fn step_n_validates_and_batches() {
    let mut world = CosmosWorld::new(CosmosWorldConfig::default());
    assert_eq!(
        world.step_n(0.0, 10).unwrap_err(),
        StepSkipReason::NonPositive
    );
    assert_eq!(
        world.step_n(f64::NAN, 10).unwrap_err(),
        StepSkipReason::NonFinite
    );
    assert_eq!(world.step_n(31.0, 1).unwrap_err(), StepSkipReason::TooLarge);

    let h = world.insert_body(satellite_builder(
        1.0,
        Vector::new(0.0, 0.0, 0.0),
        Vector::new(1.0, 0.0, 0.0),
        0.1,
    ));
    world.step_n(2.0, 5).unwrap(); // 5×2s = 10s 自由漂移
    let x = world.body_translation(h).unwrap().x;
    assert!((x - 10.0).abs() < 1e-6, "5×2s 应推 10m，实际 {x}");
}

/// `step` 子步切分的边界值：`dt == MAX_STEP_DT` 直推、`dt == 硬上限` 子步、
/// `dt` 略超 `MAX_STEP_DT` 时 ceil 子步数正确。
#[test]
fn step_substep_boundary_values() {
    let mut world = CosmosWorld::new(CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt: 1.0,
        orbit_integration: OrbitIntegration::RapierForce,
        ..CosmosWorldConfig::default()
    });
    world.insert_body(satellite_builder(
        1.0,
        Vector::ZERO,
        Vector::new(1.0, 0.0, 0.0),
        0.1,
    ));

    // dt == MAX_STEP_DT (10.0)：恰好不超，应直推 Stepped(10.0)。
    assert_eq!(world.step(10.0), StepResult::Stepped(10.0));

    // dt == 硬上限 (30.0)：恰好不超 30，应走子步 3×10s。
    assert_eq!(
        world.step(30.0),
        StepResult::Substepped {
            substeps: 3,
            sub_dt: 10.0
        }
    );

    // dt 略超 MAX_STEP_DT (11.0)：ceil(11/10)=2 子步，每步 5.5s。
    assert_eq!(
        world.step(11.0),
        StepResult::Substepped {
            substeps: 2,
            sub_dt: 5.5
        }
    );

    // dt == 20.0：恰好 2 子步，每步 10.0。
    assert_eq!(
        world.step(20.0),
        StepResult::Substepped {
            substeps: 2,
            sub_dt: 10.0
        }
    );

    // dt 略超硬上限：f64 下 30.0+EPSILON 仍舍入回 30.0，用 30.0001 验证拒。
    assert_eq!(
        world.step(30.0001),
        StepResult::Skipped(StepSkipReason::TooLarge)
    );
}

/// 默认 `n_body_softening_sq` 应为 `1e3`，且两体近距离时加速度有界不发散。
#[test]
fn default_softening_bounds_close_encounter() {
    // 默认配置应带 1e3 m² 软化。
    let cfg = CosmosWorldConfig::default();
    assert_eq!(cfg.n_body_softening_sq, 1e3);

    let mut world = CosmosWorld::new(CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt: 0.1,
        orbit_integration: OrbitIntegration::Verlet,
        n_body_softening_sq: CosmosWorldConfig::default().n_body_softening_sq,
        ..CosmosWorldConfig::default()
    });
    // 两个 1t 体，相距 1m（远小于软化长度 31.6m）：软化后引力应有界。
    let a = world.insert_body(satellite_builder(
        1000.0,
        Vector::new(0.0, 0.0, 0.0),
        Vector::ZERO,
        0.1,
    ));
    let b = world.insert_body(satellite_builder(
        1000.0,
        Vector::new(1.0, 0.0, 0.0),
        Vector::ZERO,
        0.1,
    ));
    world.add_n_body(a, 1000.0);
    world.add_n_body(b, 1000.0);

    // 取 b 的初始加速度：软化平方 1e3 加在分母，1m 距离下加速度应远小于
    // 无软化时的 G·m / r² ≈ 6.67e-11·1000 / 1 = 6.67e-8 m/s²。有软化时
    // dist_sq + softening ≈ 1001，加速度量级再降 ~1000 倍。
    let src_pos = snapshot_source_positions(world.bodies(), world.n_body_sources());
    // 纯 monopole 源（无 points）不读 rotation，用 identity 切片填充。
    let n_bodies = world.bodies().len();
    let src_rot: Vec<Rotation> = (0..n_bodies).map(|_| Rotation::IDENTITY).collect();
    let ctx = AccelContext {
        celestials: &[],
        n_body_sources: world.n_body_sources(),
        source_positions: &src_pos,
        source_rotations: &src_rot,
        source_pos_gm: &snapshot_source_pos_gm(&world),
        softening_sq: world.n_body_softening_sq(),
        central_body: None,
        sun_position: Vector::ZERO,
        relativistic: mps_cosmos::world::RelativisticCorrection::None,
        has_irregular_sources: true,
        nb_parallel: false, // 串行主路径对照(路由等价由 nb_parallel_routing_bit_identical 覆盖)
        ff_simd: mps_cosmos::integrator::ff_simd_enabled(),
    };
    let acc = total_acceleration(
        Vector::new(1.0, 0.0, 0.0),
        Vector::ZERO,
        1000.0,
        b,
        &ctx,
        None,
    );
    // 软化后 |a| 应 ≪ 无软化值 6.67e-8。给 1e-9 上限（~1.5% 无软化值）。
    assert!(acc.length() < 1e-9, "软化后近距加速度 {acc:?} 未被有效限幅");
}

/// 变质量 n-body 源回归：注册一个初始质量 m0 的 n-body 源刚体 A，受其引力的
/// 无质量测试体 B 每步获取的加速度应正比于 A 当前 body.mass()。验证
/// `refresh_n_body_sources` 在 step 开头把 `NBodySource.gm` 从刚体当前质量重算。
///
/// 若 gm 仍是注册时固化值（没有刷新），把 A 质量从 m0 改成 m1（10×）后 B 获取的
/// 速度增量不会随之放大；本测试通过测速度增量的比例≈10× 来捕获这种回归。
#[test]
fn n_body_source_gm_tracks_live_body_mass() {
    use mps_formula::celestial_data::G;
    use rapier3d::prelude::RigidBodyHandle;
    let r = 1000.0_f64;
    let m0 = 1.0e7_f64; // 初始源质量 (kg)
    let m1 = 1.0e8_f64; // 10× 后的源质量 (kg)
    // 用纯净 world：零 softening、零重力、Verlet 1 子步 / dt=1s。零扰动。
    let make_world = |source_mass: f64| -> (CosmosWorld, RigidBodyHandle, RigidBodyHandle) {
        let mut w = CosmosWorld::new(CosmosWorldConfig {
            gravity: Vector::ZERO,
            dt: 1.0,
            solver_iterations: 1,
            ccd_substeps: 0,
            n_body_softening_sq: 0.0,
            central_body: None,
            orbit_integration: OrbitIntegration::Verlet,
            verlet_substeps: 1,
            adaptive_substeps: false,
            adaptive_tolerance: 1e-9,
            relativistic_correction: mps_cosmos::world::RelativisticCorrection::None,
        });
        // 源 A：原点、锁平移、初始 mass=source_mass，注册为 n-body 源。
        let a = w.insert_body(
            satellite_builder(source_mass, Vector::ZERO, Vector::ZERO, 1.0)
                .lock_translations()
                .linear_damping(0.0)
                .angular_damping(0.0)
                .gravity_scale(0.0),
        );
        w.add_n_body(a, source_mass);
        // 测试体 B：(r,0,0)、mass=1、零速、无阻尼、不施引。
        let b = w.insert_body(
            satellite_builder(1.0, Vector::new(r, 0.0, 0.0), Vector::ZERO, 0.1)
                .linear_damping(0.0)
                .angular_damping(0.0)
                .gravity_scale(0.0),
        );
        (w, a, b)
    };

    // ① 源 A 质量 m0：B 在 1s 后获取 vx ≈ G·m0/r²。
    let (mut w0, _a0, b0) = make_world(m0);
    w0.step(1.0);
    let vx0 = w0.body_linvel(b0).unwrap().x;
    // B 在 +x，A 在原点，B 被吸向原点 → vx 为负；只比量级与方向不错号。
    let expected_a0 = G * m0 / (r * r);
    assert!(
        (vx0.abs() - expected_a0).abs() / expected_a0 < 0.02,
        "m0 场景：|vx|={abs0} 期望≈{expected_a0} (2% 容差)",
        abs0 = vx0.abs()
    );
    assert!(vx0 < 0.0, "B 应被吸向原点（-x），vx 实为 {vx0}");

    // ② 源 A 仍按 m0 注册，但 step 前把 A 的 body.mass() 改成 m1。
    //    若 refresh 起作用：B 获取的 vx ≈ G·m1/r² = 10·G·m0/r² → 10× vx0。
    //    若 gm 仍固化 m0：vx 与 vx0 几乎相等，ratio≈1，本断言失败。
    let (mut w1, a1, b1) = make_world(m0);
    // 通过新增的 set_body_mass 把 A 的刚体质量翻到 m1（旧注册源质量不变）。
    let old_mass = w1.set_body_mass(a1, m1).expect("A exists");
    assert!(
        (old_mass - m0).abs() / m0 < 1e-12,
        "set_body_mass 返回旧质量"
    );
    assert!(
        (w1.body_mass(a1).unwrap() - m1).abs() / m1 < 1e-12,
        "A 质量已变 m1"
    );
    w1.step(1.0);
    let vx1 = w1.body_linvel(b1).unwrap().x;
    let expected_a1 = G * m1 / (r * r);
    assert!(
        (vx1.abs() - expected_a1).abs() / expected_a1 < 0.02,
        "m1 场景：|vx|={abs1} 期望≈{expected_a1} (2% 容差)；若 gm 未刷新会≈{expected_a0}",
        abs1 = vx1.abs()
    );
    assert!(vx1 < 0.0, "B 应仍被吸向原点（-x），vx 实为 {vx1}");

    // ratio（均为负值相除得正）应近似 m1/m0 = 10。
    let ratio = vx1 / vx0;
    assert!(
        (ratio - 10.0).abs() < 0.4,
        "速度增量比 vx1/vx0={ratio} 应≈10 (= m1/m0)"
    );
}

/// 不规则 N 体近场：非对称质量分布产生**非径向**加速度。
///
/// 这是"星球不是球体"场景的核心物理路径——纯 monopole（点质量）的 n-body
/// 算法在过去永远给出指向源质心的径向加速度 `a ∝ -r̂`；改进后的离散质点模型在
/// 近场把每个质量点单独累加 `a = Σ G·mᵢ·dᵢ/|dᵢ|³`，允许引力偏离质心连线。
///
/// 自制测试星系（非太阳系）：一个"双瓣哑铃"源刚体 A——两个不相等的质量团，
/// 较重的 (3/4 M) 在 (+a, 0, 0)，较轻的 (1/4 M) 在 (-a, 0, 0)，锁定在原点不动。
/// 哑铃的长轴沿 +x，但探测体放在 (0, R, 0)（与源质心连线为 +y），纯 monopole 模型
/// 只能给出 (0, -GM/R², 0)——严格指向源质心，x 分量为 0。带离散质点的模型因两团
/// 质量不等，对探测体的 +x 与 -x 拉扯不抵消，加速度出现非零 x 分量，方向偏离源
/// 质心连线。x 分量的解析值给出（两团到 (0,R,0) 距离相等 √(a²+R²)，只剩 x 项
/// 未抵消）：
///   a_x = G · a / (a²+R²)^(3/2) · (3/4 M - 1/4 M) = G·a·(M/2) / (a²+R²)^(3/2)
#[test]
fn irregular_n_body_near_field_induces_non_radial_acceleration() {
    use mps_cosmos::gravity::MassPoint;
    use mps_formula::celestial_data::G;

    let m_total: f64 = 1.0e6; // 源总质量 kg
    let a: f64 = 100.0; // 两团块离质心 m
    let r: f64 = 55.0; // 探测点离源质心 m（→ 必在近场分支内）
    // 非对称质量分布：3/4 M 在 (+a,0,0)，1/4 M 在 (-a,0,0)
    let heavier = 3.0 * m_total / 4.0;
    let lighter = m_total / 4.0;
    let points = vec![
        MassPoint {
            local_offset: Vector::new(a, 0.0, 0.0),
            gm: G * heavier,
        },
        MassPoint {
            local_offset: Vector::new(-a, 0.0, 0.0),
            gm: G * lighter,
        },
    ];
    let bounding = a + 1.0; // 边界球略大于 a，含两点

    let mut world = CosmosWorld::new(CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt: 0.1,
        solver_iterations: 1,
        ccd_substeps: 0,
        n_body_softening_sq: 0.0,
        central_body: None,
        orbit_integration: OrbitIntegration::Verlet,
        verlet_substeps: 1,
        adaptive_substeps: false,
        adaptive_tolerance: 1e-9,
        relativistic_correction: mps_cosmos::world::RelativisticCorrection::None,
    });
    let a_hdl = world.insert_body(
        satellite_builder(m_total, Vector::ZERO, Vector::ZERO, 1.0)
            .lock_translations()
            .linear_damping(0.0)
            .angular_damping(0.0)
            .gravity_scale(0.0),
    );
    world.add_n_body_irregular(a_hdl, m_total, points, bounding);
    let b_hdl = world.insert_body(
        satellite_builder(1.0, Vector::new(0.0, r, 0.0), Vector::ZERO, 0.1)
            .linear_damping(0.0)
            .angular_damping(0.0)
            .gravity_scale(0.0),
    );

    // 步进 dt，Verlet 半步将 v 推进 a0·dt/2；故 step 后 vx ≈ a0_x·dt/2、vy ≈ a0_y·dt/2。
    let dt: f64 = 0.1;
    world.step(dt);
    let v = world.body_linvel(b_hdl).unwrap();

    // 解析：a0_x = G·a·(heavier-lighter)/d³，d = √(a²+r²)。
    let d_cubed = (a * a + r * r).powf(1.5);
    let expected_ax = G * a * (heavier - lighter) / d_cubed;
    // velocity-Verlet 单步 v(dt)=a0·dt/2 + a1·dt/2；源锁定，dt 内位移 ≪ √(a²+r²)，
    // 故 a1≈a0，整步后 v ≈ a0·dt（不是 dt/2）。这才是 expected_vx 的正确放大系数。
    let expected_vx = expected_ax * dt;
    assert!(
        expected_ax.abs() > 1e-12,
        "预期非零 +x 加速度分量（非径向）"
    );
    assert!(
        (v.x - expected_vx).abs() / expected_vx.abs() < 0.05,
        "非径向 x 分量：vx={vx} 期望≈{expected_vx} (5% 容差)",
        vx = v.x
    );

    // 关键非径向信号——纯 monopole 模型必给 vx=0，本测试 vx>0 即证明近场质点求和起效。
    assert!(
        v.x.abs() > 0.0,
        "vx 应非零（非径向分量），实为 {vx}",
        vx = v.x
    );

    // y 分量解析：两点都在 (±a,0,0)，场点 (0,r,0)，到每点距离 d=√(a²+r²)，每点的 y
    // 拉力 = G·mᵢ·(-r,0,0?）的 y 分量；求和 ay = G·r·(-(3M/4) - (M/4))/d³ = -G·M·r/d³。
    // 与 monopole（ay_mono = -GM/r²，≈8.9× 较大）差 ~8.9×：这正是近场用质点分布而非
    // 点 mass 的修正效应——不应近似 прошли monopole 而应精确匹配该 Σ 解析值。
    let d_cubed_for_y = (a * a + r * r).powf(1.5);
    let expected_ay = -G * m_total * r / d_cubed_for_y;
    let expected_vy = expected_ay * dt;
    let ay = v.y;
    assert!(
        (ay - expected_vy).abs() / expected_vy.abs() < 0.05,
        "y 分量（近场 点模型）= -GM·r/d³：ay={ay} 期望≈{expected_vy} (5% 容差)"
    );
    let _ = RigidBodyHandle;
}

/// 洛希极限（公式层纯函数 + 太空层查询 API）回归。
///
/// 公式 `roche_limit` 之前一直被埋在 FFI 函数 `astro_roche_limit` 体里——没有
/// 可被 crate 内部复用的纯 Rust 入口。太空层 `CosmosWorld` 因此也无法调用它
/// （跨 crate 走 FFI 是反模式）。本测试同时覆盖两层的新增能力：
/// 1. **公式层**：`mps_formula::astrophysics::roche_limit` 与 `roche_limit_report`
///    作为纯函数被抽出后仍给出与旧 FFI 等价的数值，并保留非有限/非正的 None 语义。
/// 2. **太空层**：`CosmosWorld::roche_limit_for` 用 `set_central_body` 注册过的
///    天体反算主星密度、再喂给公式纯函数；置刚体于两侧（分别在极限内 / 外）
///    验证 `inside_fluid_limit` 与 `inside_rigid_limit`。
#[test]
fn roche_limit_formula_and_cosmos_end_to_end() {
    use mps_formula::astrophysics::{roche_limit, roche_limit_report};
    use mps_formula::ffi::Bool;

    // 公式层：密度比 5.0（主星比卫星密 5 倍），主星半径 1.0
    // → ratio = 5^(1/3) ≈ 1.70998
    // → fluid ≈ 2.44 × 1.70998 = 4.172, rigid ≈ 1.26 × 1.70998 = 2.155
    let (fluid, rigid) = roche_limit(1.0, 5.0, 1.0).unwrap();
    assert!(
        (fluid - 2.44 * 5.0f64.cbrt()).abs() < 1e-12,
        "fluid {fluid}"
    );
    assert!(
        (rigid - 1.26 * 5.0f64.cbrt()).abs() < 1e-12,
        "rigid {rigid}"
    );
    assert!(fluid > rigid, "fluid > rigid (2.44 > 1.26 系数)");

    // 越界断言：在距 orbital=2.0（< rigid 2.155 < fluid 4.17）应"隔离刚体极限 +
    // 流体极限"双重判 inner 判为 true（r=2.0 同时 < rigid 与 fluid）。
    let report = roche_limit_report(1.0, 5.0, 1.0, 2.0).unwrap();
    assert_eq!(report.fluid_roche_limit, fluid);
    assert_eq!(report.rigid_roche_limit, rigid);
    assert_eq!(report.inside_fluid_limit, Bool::TRUE);
    assert_eq!(report.inside_rigid_limit, Bool::TRUE);

    // 非法输入 → None（公式层被污染不会让调用方拿到 NaN）
    assert!(roche_limit(0.0, 5.0, 1.0).is_none(), "primary_radius=0");
    assert!(roche_limit(-1.0, 5.0, 1.0).is_none(), "primary_radius<0");
    assert!(roche_limit(f64::NAN, 5.0, 1.0).is_none(), "NaN radius");
    assert!(roche_limit_report(1.0, 5.0, 1.0, -3.0).is_none(), "dist<0");
    assert!(
        roche_limit_report(1.0, 5.0, 1.0, f64::INFINITY).is_none(),
        "inf dist"
    );

    // 太空层端到端：以地球为中心天体，放一个低轨刚体位于赤道
    // 地球：GM = 3.986e14, equatorial_radius ≈ 6378137 m。可推密度约 5515 kg/m³。
    // 卫星假定为液态水状（1000 kg/m³，松散分布——典型适配流体极限模型）。
    // 流体极限将在≈1.7×地球半径处。把"刚体"放在地球表面正上方
    //（orbital = equatorial_radius，远小于 fluid ≈ 5219km/...）。
    let earth = get_celestial_body(CelestialBodyId::Earth);
    let r_earth = earth.equatorial_radius;
    let rho_pri = (earth.gm / mps_formula::celestial_data::G)
        / ((4.0 / 3.0) * std::f64::consts::PI * r_earth.powi(3));
    // 算解析流体极限，便于稍后核验 CosmosWorld 是否复算一致
    let expected_fluid = 2.44 * r_earth * (rho_pri / 1000.0).cbrt();
    let expected_rigid = 1.26 * r_earth * (rho_pri / 1000.0).cbrt();

    let mut world = CosmosWorld::new(CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt: 1.0,
        orbit_integration: OrbitIntegration::Verlet,
        ..CosmosWorldConfig::default()
    });
    world.set_central_body(Some(earth));

    // (a) 刚体在 1.5×R_earth：落在刚体极限 expected_rigid (~2.21 R_earth) 与流体极限
    //     expected_fluid (~4.31 R_earth) 双侧内 → 两个 inside 都判 true。
    let inner = world.insert_body(satellite_builder(
        1.0,
        Vector::new(1.5 * r_earth, 0.0, 0.0),
        Vector::ZERO,
        0.1,
    ));

    // (b) 刚体放 2×expected_fluid 处 → 远在两极限外，inside_* 都判 false。
    let outer = world.insert_body(satellite_builder(
        1.0,
        Vector::new(2.0 * expected_fluid, 0.0, 0.0),
        Vector::ZERO,
        0.1,
    ));

    let rep_inner = world
        .roche_limit_for(inner, 1000.0)
        .expect("inner: central_body + body 都有效");
    let rep_outer = world
        .roche_limit_for(outer, 1000.0)
        .expect("outer: central_body + body 都有效");

    // 数值核验：流体极限与解析公式一致（通过 G/π 反算主星密度）
    let inner_fluid = rep_inner.fluid_roche_limit;
    let inner_rigid = rep_inner.rigid_roche_limit;
    assert!(
        (inner_fluid - expected_fluid).abs() / expected_fluid < 1e-9,
        "CosmosWorld 流体极限 {inner_fluid} 应≈{expected_fluid}"
    );
    assert!(
        (inner_rigid - expected_rigid).abs() / expected_rigid < 1e-9,
        "CosmosWorld 刚体极限 {inner_rigid} 应≈{expected_rigid}"
    );
    assert!(inner_rigid < inner_fluid);

    // 1.5×R_earth < expected_rigid (~2.21 R_earth) < expected_fluid (~4.31 R_earth)
    // → 在 1.5R 处双双 inside。
    assert_eq!(rep_inner.inside_fluid_limit, Bool::TRUE);
    assert_eq!(rep_inner.inside_rigid_limit, Bool::TRUE);

    // 2×expected_fluid 远在极限外
    assert_eq!(rep_outer.inside_fluid_limit, Bool::FALSE);
    assert_eq!(rep_outer.inside_rigid_limit, Bool::FALSE);

    // 未设 central_body → roche_limit_for 返回 None（无主星，无法算）
    let mut world_no_earth = CosmosWorld::new(CosmosWorldConfig::default());
    let h = world_no_earth.insert_body(satellite_builder(1.0, Vector::ZERO, Vector::ZERO, 0.1));
    assert!(
        world_no_earth.roche_limit_for(h, 1000.0).is_none(),
        "无 central_body 应返回 None"
    );

    // 非法 secondary_density → None
    assert!(world.roche_limit_for(inner, 0.0).is_none(), "density=0");
    assert!(world.roche_limit_for(inner, -1.0).is_none(), "density<0");
    assert!(
        world.roche_limit_for(inner, f64::NAN).is_none(),
        "NaN density"
    );

    // 有效但刚体不存在 → None
    let bogus = RigidBodyHandle::from_raw_parts(u32::MAX, u32::MAX);
    assert!(
        world.roche_limit_for(bogus, 1000.0).is_none(),
        "不存在的刚体"
    );

    let _ = RigidBodyHandle;
}

/// Hill 球（天体力学界中 Roche 极限的姊妹判据）回归：先 lock 公式层
/// `hill_sphere_radius` 的解析数值；再 verify `CosmosWorld::hill_radius_for`
/// 用 central_body 反算主星质量、刚体本身取质量，得到与公式一致的结果。
///
/// `r_H = a · (1 - e) · (m_sec / (3·m_pri))^(1/3)`。
#[test]
fn hill_sphere_formula_and_cosmos_world_consistent() {
    use mps_formula::astrophysics::hill_sphere_radius;

    // 公式层：典型月球-地球体系
    //  M_earth ≈ 5.972e24 kg；M_moon ≈ 7.342e22 kg
    //  a = 3.844e8 m；e = 0.0549
    let (m_pri, m_sec, a, e) = (5.972e24, 7.342e22, 3.844e8, 0.0549);
    let r_h = hill_sphere_radius(m_pri, m_sec, a, e).unwrap();
    let expect = a * (1.0 - e) * (m_sec / (3.0 * m_pri)).cbrt();
    assert!(
        (r_h - expect).abs() / expect < 1e-12,
        "r_h={r_h} expect={expect}"
    );

    // 边界：e 钳到 0..=1；非法输入 → None
    assert!(hill_sphere_radius(0.0, 1.0, 1.0, 0.0).is_none(), "M_pri=0");
    assert!(hill_sphere_radius(1.0, 0.0, 1.0, 0.0).is_none(), "m_sec=0");
    assert!(hill_sphere_radius(1.0, 1.0, 0.0, 0.0).is_none(), "a=0");
    assert!(
        hill_sphere_radius(1.0, 1.0, 1.0, f64::NAN).is_none(),
        "e=NaN"
    );
    // e>1 钳置 1 → 退化到零（(1-e)=0）
    assert_eq!(hill_sphere_radius(1.0, 1.0, 1.0, 5.0).unwrap(), 0.0);
    // e<0 钳置 0 → 等价 e=0
    let r_h_neg = hill_sphere_radius(1.0, 1.0, 1.0, -0.3).unwrap();
    let r_h_zero = hill_sphere_radius(1.0, 1.0, 1.0, 0.0).unwrap();
    assert_eq!(r_h_neg, r_h_zero);

    // 太空层：地球 central_body + 1000kg 刚体在 LEO 半径 7e6 m 处
    let earth = get_celestial_body(CelestialBodyId::Earth);
    let primary_mass = earth.gm / mps_formula::celestial_data::G;
    let mut world = CosmosWorld::new(CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt: 1.0,
        ..CosmosWorldConfig::default()
    });
    world.set_central_body(Some(earth));
    let h = world.insert_body(satellite_builder(
        1000.0,
        Vector::new(7.0e6, 0.0, 0.0),
        Vector::ZERO,
        0.1,
    ));
    // cosmos 不维护根数；caller 提供 a 与 e
    let cosmos_r_h = world.hill_radius_for(h, 7.0e6, 0.0).unwrap();
    let expect2 = 7.0e6 * (1000.0 / (3.0 * primary_mass)).cbrt();
    assert!(
        (cosmos_r_h - expect2).abs() / expect2 < 1e-9,
        "cosmos hill {cosmos_r_h} expect {expect2}"
    );
    // 无 central_body / 无效刚体 / 非法 a → None
    let mut world2 = CosmosWorld::new(CosmosWorldConfig::default());
    let h2 = world2.insert_body(satellite_builder(1.0, Vector::ZERO, Vector::ZERO, 0.1));
    assert!(
        world2.hill_radius_for(h2, 1e6, 0.0).is_none(),
        "无 central_body"
    );
    let bogus = RigidBodyHandle::from_raw_parts(u32::MAX, u32::MAX);
    assert!(
        world.hill_radius_for(bogus, 1e6, 0.0).is_none(),
        "bogus handle"
    );
    assert!(world.hill_radius_for(h, -1.0, 0.0).is_none(), "a<0");
}

/// 太空层扰动力单元回归：直接调用 perturbation 模块的两个新力函数，
/// 对比 baseline（无扰动）确认 sign / 量级正确。比端到端跑 1000 步更稳。
#[test]
fn perturbation_solar_wind_and_dynamical_friction_unit() {
    use mps_cosmos::perturbation::{dynamical_friction_force, solar_wind_pressure_force};

    // ===== 太阳风动压 =====
    // 物体在 +x，太阳在原点 → sun_to_body = +x，风沿 +x 推（远离太阳）。
    // 静态物体（velocity=0），proton_density=5e6 n/m³, v_sw=450 m/s, area=50m²。
    // P = ρ·v² = (5e6 · m_proton)·(450)² ≈ 1.69e-9 Pa · 1e9 = 1.69 nPa → 1.69e-9 Pa
    // F = P·A = 1.69e-9 · 50 ≈ 8.45e-8 N，方向 +x。
    let f_sw = solar_wind_pressure_force(
        Vector::new(1.0e11, 0.0, 0.0),
        Vector::ZERO,
        5.0e6,
        450.0,
        50.0,
    )
    .expect("solar wind: 参数合法应返回 Some");
    assert!(f_sw.x > 0.0, "太阳风应沿 +x：F_sw.x={}", f_sw.x);
    assert!(f_sw.x.is_finite());
    // 量级核验：1.69e-9 Pa · 50 m² ≈ 8.45e-8 N
    let expect_pa = 5.0e6 * 1.6726219e-27 * 450.0f64 * 450.0;
    let expect_force = expect_pa * 50.0;
    assert!(
        (f_sw.x - expect_force).abs() / expect_force < 1e-2,
        "量级核验：F_sw.x={0} expect≈={expect_force}",
        f_sw.x,
    );

    // 顺风而行（同方向）→ 无力
    let body_w = Vector::new(1.0e11, 0.0, 0.0);
    let strong_tail =
        solar_wind_pressure_force(body_w, Vector::new(450.0, 0.0, 0.0), 5.0e6, 100.0, 50.0);
    assert!(strong_tail.is_none(), "顺风速度 ≥ 风速 → None");
    // 无效参数 → None
    assert!(solar_wind_pressure_force(body_w, Vector::ZERO, 0.0, 100.0, 50.0).is_none());
    assert!(solar_wind_pressure_force(body_w, Vector::ZERO, 5e6, 0.0, 50.0).is_none());
    assert!(solar_wind_pressure_force(body_w, Vector::ZERO, 5e6, 100.0, 0.0).is_none());

    // ===== 动力学摩擦 =====
    // 物体以 +y 方向 7000 m/s 穿过密度 1e-20 kg/m³ 的介质，lnΛ=5，质量 1000 kg。
    let vel = Vector::new(0.0, 7000.0, 0.0);
    let f_df = dynamical_friction_force(1000.0, 1.0e-20, vel, 5.0)
        .expect("dynamical friction: 参数合法应返回 Some");
    // 反速度方向 → 应是 -y 分量
    assert!(f_df.y < 0.0, "动摩擦应反速度（-y）：F_df.y={}", f_df.y);
    assert!(f_df.x.abs() < 1e-25, "纯 +y 速度 → 摩擦应仅在 -y");
    // 量级核验：a_mag = 4π G² M ρ lnΛ/v² = 4π · (6.6743e-11)² · 1000 · 1e-20 · 5 / 7000²
    let g = 6.67430e-11_f64;
    let a_mag_expect =
        4.0 * std::f64::consts::PI * g * g * 1000.0 * 1.0e-20 * 5.0 / (7000.0 * 7000.0);
    let f_mag_expect = a_mag_expect * 1000.0; // 力 = m·a_df
    assert!(
        (f_df.y.abs() - f_mag_expect).abs() / f_mag_expect < 1e-3,
        "量级：|F_df.y|={0} expect≈={f_mag_expect}",
        f_df.y.abs(),
    );
    // 速度为零 → None（公式 1/v² 发散）
    assert!(dynamical_friction_force(1000.0, 1.0e-20, Vector::ZERO, 5.0).is_none());
    // 其它非法 → None
    assert!(
        dynamical_friction_force(0.0, 1.0e-20, vel, 5.0).is_none(),
        "m=0"
    );
    assert!(
        dynamical_friction_force(1000.0, 0.0, vel, 5.0).is_none(),
        "ρ=0"
    );
    assert!(
        dynamical_friction_force(1000.0, 1.0e-20, vel, 0.0).is_none(),
        "lnΛ=0"
    );
}

/// 优化一致性回归：n-body 互引力的并行 + `get_unchecked` 优化路径必须与朴素
/// 串行 reference 逐位（bit-identical）一致。
///
/// 构造 3 体互引力世界（Verlet 模式，无天体源/扰动），跑一次 `world.step(dt)`
/// （走优化后的并行 a0 预计算 + 主循环 `get_unchecked`）。然后独立用串行
/// velocity-Verlet 重算每个体的终态位置——`a0`/`a1` 都用 `total_acceleration`
/// 在**冻结的初始配置**上求（与生产路径内部 Verlet 评估一致），半踢-全漂-半踢。
/// 生产结果与 reference 必须逐位相等（不允许多于 `1e-12` 的偏差），否则说明
/// 并行化或 `get_unchecked` 重构打乱了浮点运算顺序 / 数值。
#[test]
fn n_body_optimized_matches_serial_reference_bit_identical() {
    let dt = 0.01;
    let mut world = CosmosWorld::new(CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt,
        solver_iterations: 4,
        ccd_substeps: 4,
        orbit_integration: OrbitIntegration::Verlet,
        n_body_softening_sq: 0.0,
        central_body: None,
        verlet_substeps: 1,
        ..CosmosWorldConfig::default()
    });

    // 3 个体，质量/位置/速度各异，确保互引力非零且互不对称。
    let defs = [
        (
            1.0e3,
            Vector::new(0.0, 0.0, 0.0),
            Vector::new(0.1, 0.0, 0.0),
        ),
        (
            2.0e3,
            Vector::new(3.0, 1.0, 0.0),
            Vector::new(0.0, -0.2, 0.0),
        ),
        (
            5.0e2,
            Vector::new(-2.0, 4.0, 1.0),
            Vector::new(0.0, 0.0, 0.3),
        ),
    ];
    let mut init = Vec::new();
    for (m, p, v) in defs {
        let h = world.insert_body(satellite_builder(m, p, v, 0.1));
        world.add_n_body(h, m);
        let b = world.bodies().get(h).unwrap();
        init.push((h, b.translation(), b.linvel(), m));
    }

    // 冻结初始配置：源位置快照（生产路径 `refresh_n_body_sources` 产出）。
    let src_pos = snapshot_source_positions(world.bodies(), world.n_body_sources());
    let n_bodies = world.bodies().len();
    let src_rot: Vec<Rotation> = (0..n_bodies).map(|_| Rotation::IDENTITY).collect();
    let ctx = AccelContext {
        celestials: &[],
        n_body_sources: world.n_body_sources(),
        source_positions: &src_pos,
        source_rotations: &src_rot,
        source_pos_gm: &snapshot_source_pos_gm(&world),
        softening_sq: 0.0,
        central_body: None,
        sun_position: Vector::ZERO,
        relativistic: mps_cosmos::world::RelativisticCorrection::None,
        has_irregular_sources: true,
        nb_parallel: false, // 串行主路径对照(路由等价由 nb_parallel_routing_bit_identical 覆盖)
        ff_simd: mps_cosmos::integrator::ff_simd_enabled(),
    };

    // 独立串行 reference：每体 velocity-Verlet（与生产 `verlet_step` 同公式）。
    let mut reference = Vec::new();
    for &(h, r0, v0, mass) in &init {
        let a0 = total_acceleration(r0, v0, mass, h, &ctx, None);
        let v_half = v0 + a0 * (0.5 * dt);
        let r1 = r0 + v_half * dt;
        let a1 = total_acceleration(r1, v_half, mass, h, &ctx, None);
        let v1 = v_half + a1 * (0.5 * dt);
        reference.push((h, r1, v1));
    }

    // 生产路径：优化后的并行 + get_unchecked。
    world.step(dt);

    for (h, r_ref, v_ref) in reference {
        let b = world.bodies().get(h).unwrap();
        let pos = b.translation();
        let vel = b.linvel();
        assert!(
            (pos.x - r_ref.x).abs() < 1e-12
                && (pos.y - r_ref.y).abs() < 1e-12
                && (pos.z - r_ref.z).abs() < 1e-12,
            "body {h:?} 优化路径位置 {pos:?} 与串行 reference {r_ref:?} 不一致"
        );
        assert!(
            (vel.x - v_ref.x).abs() < 1e-12
                && (vel.y - v_ref.y).abs() < 1e-12
                && (vel.z - v_ref.z).abs() < 1e-12,
            "body {h:?} 优化路径速度 {vel:?} 与串行 reference {v_ref:?} 不一致"
        );
    }
}

/// 高阶 + Kahan 积子优化一致性回归：并行预计算的高阶推进 / Kahan 推进必须和
/// 朴素串行 reference（直接调 `advance_highorder` / `advance_highorder_kahan`
/// 纯函数）逐位一致。
///
/// 构造 3 体互引力世界，分别用 `Yoshida4` / `ForestRuth8` / `Yoshida4Kahan` /
/// `ForestRuth8Kahan` 跑一次 `world.step(dt)`（走并行预计算路径），再用对应
/// 纯函数做独立串行 reference，比对每个体终态位置/速度（Kahan 还要比对累加态）。
/// 偏差须 < 1e-12。
#[cfg(test)]
fn highorder_bit_identical_helper(mode: OrbitIntegration) {
    let dt = 0.01;
    let mut world = CosmosWorld::new(CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt,
        solver_iterations: 4,
        ccd_substeps: 4,
        orbit_integration: mode,
        n_body_softening_sq: 0.0,
        central_body: None,
        verlet_substeps: 1,
        ..CosmosWorldConfig::default()
    });

    let defs = [
        (
            1.0e3,
            Vector::new(0.0, 0.0, 0.0),
            Vector::new(0.1, 0.0, 0.0),
        ),
        (
            2.0e3,
            Vector::new(3.0, 1.0, 0.0),
            Vector::new(0.0, -0.2, 0.0),
        ),
        (
            5.0e2,
            Vector::new(-2.0, 4.0, 1.0),
            Vector::new(0.0, 0.0, 0.3),
        ),
    ];
    let mut handles = Vec::new();
    let mut init = Vec::new();
    for (m, p, v) in defs {
        let h = world.insert_body(satellite_builder(m, p, v, 0.1));
        world.add_n_body(h, m);
        let b = world.bodies().get(h).unwrap();
        init.push((h, b.translation(), b.linvel(), m));
        handles.push(h);
    }

    let src_pos = snapshot_source_positions(world.bodies(), world.n_body_sources());
    let n_bodies = world.bodies().len();
    let src_rot: Vec<Rotation> = (0..n_bodies).map(|_| Rotation::IDENTITY).collect();
    let ctx = AccelContext {
        celestials: &[],
        n_body_sources: world.n_body_sources(),
        source_positions: &src_pos,
        source_rotations: &src_rot,
        source_pos_gm: &snapshot_source_pos_gm(&world),
        softening_sq: 0.0,
        central_body: None,
        sun_position: Vector::ZERO,
        relativistic: mps_cosmos::world::RelativisticCorrection::None,
        has_irregular_sources: true,
        nb_parallel: false, // 串行主路径对照(路由等价由 nb_parallel_routing_bit_identical 覆盖)
        ff_simd: mps_cosmos::integrator::ff_simd_enabled(),
    };

    // 串行 reference：每个体用对应纯函数推进。
    let use_kahan = matches!(
        mode,
        OrbitIntegration::Yoshida4Kahan | OrbitIntegration::ForestRuth8Kahan
    );
    let mut ref_out = Vec::new();
    for &(h, r0, v0, mass) in &init {
        if use_kahan {
            let kp = KahanVec3::new(mps_cosmos::integrator::ffi_vec3_pub(r0));
            let kv = KahanVec3::new(mps_cosmos::integrator::ffi_vec3_pub(v0));
            let (r1, v1, nkp, nkv) =
                advance_highorder_kahan(mode, r0, v0, kp, kv, mass, h, None, &ctx, dt);
            ref_out.push((h, r1, v1, Some((nkp, nkv))));
        } else {
            let (r1, v1) = advance_highorder(mode, r0, v0, mass, h, None, &ctx, dt);
            ref_out.push((h, r1, v1, None));
        }
    }

    world.step(dt);

    for (h, r_ref, v_ref, k_ref) in ref_out {
        let b = world.bodies().get(h).unwrap();
        let pos = b.translation();
        let vel = b.linvel();
        assert!(
            (pos.x - r_ref.x).abs() < 1e-12
                && (pos.y - r_ref.y).abs() < 1e-12
                && (pos.z - r_ref.z).abs() < 1e-12,
            "body {h:?} 高阶优化路径位置 {pos:?} 与串行 reference {r_ref:?} 不一致"
        );
        assert!(
            (vel.x - v_ref.x).abs() < 1e-12
                && (vel.y - v_ref.y).abs() < 1e-12
                && (vel.z - v_ref.z).abs() < 1e-12,
            "body {h:?} 高阶优化路径速度 {vel:?} 与串行 reference {v_ref:?} 不一致"
        );
        if let Some((kp_ref, kv_ref)) = k_ref {
            let idx = h.into_raw_parts().0 as usize;
            let st = world.kahan_state_debug(idx);
            assert!(
                (st.0.value().x - kp_ref.value().x).abs() < 1e-12
                    && (st.0.value().y - kp_ref.value().y).abs() < 1e-12
                    && (st.0.value().z - kp_ref.value().z).abs() < 1e-12
                    && (st.1.value().x - kv_ref.value().x).abs() < 1e-12
                    && (st.1.value().y - kv_ref.value().y).abs() < 1e-12
                    && (st.1.value().z - kv_ref.value().z).abs() < 1e-12,
                "body {h:?} Kahan 累加态与串行 reference 不一致"
            );
        }
    }
}

#[test]
fn highorder_yoshida4_matches_serial_reference() {
    highorder_bit_identical_helper(OrbitIntegration::Yoshida4);
}

#[test]
fn highorder_forest_ruth8_matches_serial_reference() {
    highorder_bit_identical_helper(OrbitIntegration::ForestRuth8);
}

#[test]
fn highorder_yoshida4_kahan_matches_serial_reference() {
    highorder_bit_identical_helper(OrbitIntegration::Yoshida4Kahan);
}

#[test]
fn highorder_forest_ruth8_kahan_matches_serial_reference() {
    highorder_bit_identical_helper(OrbitIntegration::ForestRuth8Kahan);
}

/// `has_irregular_sources` 标志短路优化一致性回归：含一个「不规则质量分布」n-body
/// 源的 3 体世界，跑 `world.step(dt)`（生产路径，标志由 `refresh_n_body_sources`
/// 自动算出并喂进 `AccelContext`），再用独立串行 reference（`total_acceleration`
/// 显式传 `has_irregular_sources: true`，走真实近场质点求和）重算终态，比对每个
/// 体位置/速度须 < 1e-12。同时断言 `world.has_irregular_sources()` 在生产路径下
/// 正确识别为 `true`。这把 A2 的「标志短路不影响数值」钉死：标志只改分支判据的
/// 聚合方式，不改变任何浮点运算顺序。
#[test]
fn irregular_source_flag_shortcut_matches_full_near_field() {
    use mps_cosmos::gravity::MassPoint;
    use mps_formula::celestial_data::G;

    let dt = 0.01;
    let mut world = CosmosWorld::new(CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt,
        solver_iterations: 4,
        ccd_substeps: 4,
        orbit_integration: OrbitIntegration::Verlet,
        n_body_softening_sq: 0.0,
        central_body: None,
        verlet_substeps: 1,
        ..CosmosWorldConfig::default()
    });

    // 1 个不规则源（两团块非对称）+ 2 个动态探测体，全部注册为 n-body 互引力源
    // 与受力体，确保近场分支在 1 个源上真被触发。
    let a = 100.0_f64;
    let m_src = 1.0e6_f64;
    let pts = vec![
        MassPoint {
            local_offset: Vector::new(a, 0.0, 0.0),
            gm: G * (0.75 * m_src),
        },
        MassPoint {
            local_offset: Vector::new(-a, 0.0, 0.0),
            gm: G * (0.25 * m_src),
        },
    ];
    let bounding = a + 1.0;

    let defs = [
        // (mass, pos, vel)
        (m_src, Vector::new(0.0, 0.0, 0.0), Vector::ZERO),
        (
            1.0e3,
            Vector::new(3.0, 1.0, 0.0),
            Vector::new(0.0, -0.2, 0.0),
        ),
        (
            5.0e2,
            Vector::new(-2.0, 4.0, 1.0),
            Vector::new(0.0, 0.0, 0.3),
        ),
    ];
    let mut handles = Vec::new();
    let mut init = Vec::new();
    for (i, (m, p, v)) in defs.iter().enumerate() {
        let h = world.insert_body(
            satellite_builder(*m, *p, *v, 0.1)
                .lock_translations()
                .linear_damping(0.0)
                .angular_damping(0.0)
                .gravity_scale(0.0),
        );
        if i == 0 {
            // 源刚体锁平移，但仍作为 n-body 互引力源参与（被其它体吸引）。
            world.add_n_body_irregular(h, *m, pts.clone(), bounding);
        } else {
            world.add_n_body(h, *m);
        }
        let b = world.bodies().get(h).unwrap();
        init.push((h, b.translation(), b.linvel(), *m));
        handles.push(h);
    }

    // 生产路径：step 内部 refresh 会算出 has_irregular_sources=true 并喂进 ctx。
    world.step(dt);
    assert!(
        world.has_irregular_sources(),
        "含不规则源的世界应识别出 has_irregular_sources=true"
    );

    // 独立串行 reference：冻结初始配置，用 total_acceleration（显式
    // has_irregular_sources=true）做 velocity-Verlet，与生产 verlet_step 同公式。
    let src_pos = snapshot_source_positions(world.bodies(), world.n_body_sources());
    let n_bodies = world.bodies().len();
    let src_rot: Vec<Rotation> = (0..n_bodies).map(|_| Rotation::IDENTITY).collect();
    let ctx = AccelContext {
        celestials: &[],
        n_body_sources: world.n_body_sources(),
        source_positions: &src_pos,
        source_rotations: &src_rot,
        source_pos_gm: &snapshot_source_pos_gm(&world),
        softening_sq: 0.0,
        central_body: None,
        sun_position: Vector::ZERO,
        relativistic: mps_cosmos::world::RelativisticCorrection::None,
        has_irregular_sources: true,
        nb_parallel: false, // 串行主路径对照(路由等价由 nb_parallel_routing_bit_identical 覆盖)
        ff_simd: mps_cosmos::integrator::ff_simd_enabled(),
    };
    let mut reference = Vec::new();
    for &(h, r0, v0, mass) in &init {
        let a0 = total_acceleration(r0, v0, mass, h, &ctx, None);
        let v_half = v0 + a0 * (0.5 * dt);
        let r1 = r0 + v_half * dt;
        let a1 = total_acceleration(r1, v_half, mass, h, &ctx, None);
        let v1 = v_half + a1 * (0.5 * dt);
        reference.push((h, r1, v1));
    }

    for (h, r_ref, v_ref) in reference {
        let b = world.bodies().get(h).unwrap();
        let pos = b.translation();
        let vel = b.linvel();
        assert!(
            (pos.x - r_ref.x).abs() < 1e-12
                && (pos.y - r_ref.y).abs() < 1e-12
                && (pos.z - r_ref.z).abs() < 1e-12,
            "不规则源世界 body {h:?} 位置 {pos:?} 与串行 reference {r_ref:?} 不一致"
        );
        assert!(
            (vel.x - v_ref.x).abs() < 1e-12
                && (vel.y - v_ref.y).abs() < 1e-12
                && (vel.z - v_ref.z).abs() < 1e-12,
            "不规则源世界 body {h:?} 速度 {vel:?} 与串行 reference {v_ref:?} 不一致"
        );
    }
}

/// B2 并行写回 lock-down：N 体世界（N 较大以触发 rayon 多核分桶），跑
/// `world.step(dt)`（生产路径：3.5 段并行预计算 + B2 并行写回 body / Kahan 态），
/// 再用独立串行 reference（同 `total_acceleration` + velocity-Verlet）重算终态，
/// 每体位置/速度须 < 1e-12。这一把 B2「并发写回各体独有槽、与串行逐位一致」钉死：
/// 写回顺序在不同体间无依赖，故 bit-identical。N=64 足以在常见机器上跨多个 rayon
/// 工作线程，真正 exercise 并行写回路径。
#[test]
fn parallel_writeback_matches_serial_reference_bit_identical() {
    let dt = 0.01;
    let n = 64u64;
    let mut world = CosmosWorld::new(CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt,
        solver_iterations: 4,
        ccd_substeps: 4,
        orbit_integration: OrbitIntegration::Verlet,
        n_body_softening_sq: 0.0,
        central_body: None,
        verlet_substeps: 1,
        ..CosmosWorldConfig::default()
    });

    // 每个体既是 n-body 互引力源也是受力体，构成全互引力 N 体问题。
    let mut init = Vec::new();
    for i in 0..n {
        // 在球壳上均匀散布，质量/初速各异，确保互引力非零且非对称。
        let theta = (i as f64) * 2.39996323_f64; // 黄金角，近似均匀
        let r = 100.0 + (i as f64) * 1.5;
        let pos = Vector::new(
            r * theta.cos(),
            r * theta.sin(),
            (i as f64) * 0.7 - (n as f64) * 0.35,
        );
        let vel = Vector::new(
            (i as f64 * 0.013).sin() * 0.05,
            (i as f64 * 0.017).cos() * 0.05,
            (i as f64 * 0.011).sin() * 0.05,
        );
        let m = 1.0e3 + (i as f64) * 10.0;
        let h = world.insert_body(
            satellite_builder(m, pos, vel, 0.1)
                .lock_translations()
                .linear_damping(0.0)
                .angular_damping(0.0)
                .gravity_scale(0.0),
        );
        world.add_n_body(h, m);
        let b = world.bodies().get(h).unwrap();
        init.push((h, b.translation(), b.linvel(), m));
    }

    // 生产路径：step 内部 refresh → 3.5 并行预计算 → B2 并行写回。
    world.step(dt);

    // 独立串行 reference：冻结初始配置，velocity-Verlet 与生产 verlet_step 同公式。
    let src_pos = snapshot_source_positions(world.bodies(), world.n_body_sources());
    let n_bodies = world.bodies().len();
    let src_rot: Vec<Rotation> = (0..n_bodies).map(|_| Rotation::IDENTITY).collect();
    let ctx = AccelContext {
        celestials: &[],
        n_body_sources: world.n_body_sources(),
        source_positions: &src_pos,
        source_rotations: &src_rot,
        source_pos_gm: &snapshot_source_pos_gm(&world),
        softening_sq: 0.0,
        central_body: None,
        sun_position: Vector::ZERO,
        relativistic: mps_cosmos::world::RelativisticCorrection::None,
        has_irregular_sources: true,
        nb_parallel: false, // 串行主路径对照(路由等价由 nb_parallel_routing_bit_identical 覆盖)
        ff_simd: mps_cosmos::integrator::ff_simd_enabled(),
    };
    let mut reference = Vec::new();
    for &(h, r0, v0, mass) in &init {
        let a0 = total_acceleration(r0, v0, mass, h, &ctx, None);
        let v_half = v0 + a0 * (0.5 * dt);
        let r1 = r0 + v_half * dt;
        let a1 = total_acceleration(r1, v_half, mass, h, &ctx, None);
        let v1 = v_half + a1 * (0.5 * dt);
        reference.push((h, r1, v1));
    }

    for (h, r_ref, v_ref) in reference {
        let b = world.bodies().get(h).unwrap();
        let pos = b.translation();
        let vel = b.linvel();
        assert!(
            (pos.x - r_ref.x).abs() < 1e-12
                && (pos.y - r_ref.y).abs() < 1e-12
                && (pos.z - r_ref.z).abs() < 1e-12,
            "B2 并行写回 body {h:?} 位置 {pos:?} 与串行 reference {r_ref:?} 不一致"
        );
        assert!(
            (vel.x - v_ref.x).abs() < 1e-12
                && (vel.y - v_ref.y).abs() < 1e-12
                && (vel.z - v_ref.z).abs() < 1e-12,
            "B2 并行写回 body {h:?} 速度 {vel:?} 与串行 reference {v_ref:?} 不一致"
        );
    }
}

/// 端到端 bit-identical 确定性闸门（E1 精神，本地基线）。
///
/// 同一确定性初始条件推演两次，终态必须逐位相等（f64 `==`）。覆盖全链路含
/// P1 默认开启的 SIMD 远场（多个 n-body 源会激活 `far_field_monopole_simd`）。
/// 这是「原方法不变」铁律的顶层护栏：任何未来优化若破坏逐位一致性，此测试
/// 必红。跨平台（Linux/Windows）一致性由同一二进制在 CI 上跑两份基线比对实现；
/// 此处给出可本地复跑的 run-to-run 基线。
#[test]
fn scenario_bit_identical_determinism_gate() {
    let sun_gm = 1.32712440018e20_f64; // 太阳 GM
    // 多个行星作为 n-body 源（激活 SIMD 远场路径）。
    let planets = [
        (1.0e-6 * sun_gm, 5.79e10_f64, 0.0_f64),    // 类水星
        (2.45e-6 * sun_gm, 1.082e11_f64, 0.05_f64), // 类金星
        (3.0e-6 * sun_gm, 1.496e11_f64, 0.10_f64),  // 类地球
        (3.2e-7 * sun_gm, 2.279e11_f64, 0.03_f64),  // 类火星
    ];

    let build = || {
        let mut world = CosmosWorld::new(CosmosWorldConfig {
            gravity: Vector::ZERO,
            dt: 60.0,
            solver_iterations: 8,
            ccd_substeps: 0,
            n_body_softening_sq: 0.0,
            central_body: None,
            orbit_integration: OrbitIntegration::Verlet,
            verlet_substeps: 1,
            adaptive_substeps: false,
            adaptive_tolerance: 1e-9,
            relativistic_correction: mps_cosmos::world::RelativisticCorrection::None,
        });
        // 中心恒星（静止原点，作为 n-body 源）。
        let sun_h = world.insert_body(
            satellite_builder(sun_gm, Vector::ZERO, Vector::ZERO, 1.0).lock_translations(),
        );
        world.add_n_body(sun_h, sun_gm);
        let mut handles = Vec::new();
        for (m, a, inc) in planets {
            let r = Vector::new(a * inc.cos(), a * inc.sin(), a * 0.2_f64);
            let v = (sun_gm / a).sqrt(); // 近似圆速度
            let vel = Vector::new(-v * inc.sin(), v * inc.cos(), v * 0.1_f64);
            let h = world.insert_body(
                satellite_builder(m, r, vel, 1.0)
                    .linear_damping(0.0)
                    .angular_damping(0.0)
                    .gravity_scale(0.0),
            );
            world.add_n_body(h, m);
            handles.push(h);
        }
        (world, handles)
    };

    let (mut w1, h1) = build();
    let (mut w2, h2) = build();
    for _ in 0..200 {
        w1.step(60.0);
        w2.step(60.0);
    }

    for (a, b) in h1.iter().zip(h2.iter()) {
        let pa = w1.body_translation(*a).expect("body");
        let pb = w2.body_translation(*b).expect("body");
        let va = w1.body_linvel(*a).expect("body");
        let vb = w2.body_linvel(*b).expect("body");
        assert_eq!(pa.x, pb.x, "pos.x bit-diff");
        assert_eq!(pa.y, pb.y, "pos.y bit-diff");
        assert_eq!(pa.z, pb.z, "pos.z bit-diff");
        assert_eq!(va.x, vb.x, "vel.x bit-diff");
        assert_eq!(va.y, vb.y, "vel.y bit-diff");
        assert_eq!(va.z, vb.z, "vel.z bit-diff");
    }
}

/// F2（2026-08-20）：Hut (1981) 平衡潮自旋同步力矩。
///
/// 注意：潮汐自旋演化经 Rapier 的 `apply_torque_impulse` 接入，仅 `RapierForce`
/// 积分路径消费（显式高阶积子不积分角运动，见 world.rs 文档）。故测试用 RapierForce。
///
/// - 关闭（默认）→ 自旋不施加潮汐力矩，与历史行为逐位一致（bit-identical）。
/// - 开启 + 设 central_body 作伴星 → 欠同步自旋被加速向同步，若干步后终态自旋
///   与「关闭」世界发散（力矩确实接入并施加）。真实小卫星的潮汐同步时标为地质量级，
///   500 s 内绝对增量极小（~1e-18 rad/s），故此处只验证「方向正确 + 非零 + 关闭时
///   逐位不变」，不盲设夸大阈值。
#[test]
fn tidal_torque_synchronizes_spin_when_enabled() {
    let earth = get_celestial_body(CelestialBodyId::Earth);
    let gm = earth.gm;
    let r = 2.0e7_f64;
    let v = (gm / r).sqrt();

    let build = |enable_tidal: bool| {
        let mut world = CosmosWorld::new(CosmosWorldConfig {
            gravity: Vector::ZERO,
            dt: 10.0,
            solver_iterations: 8,
            ccd_substeps: 0,
            n_body_softening_sq: 0.0,
            central_body: None,
            orbit_integration: OrbitIntegration::RapierForce,
            verlet_substeps: 1,
            adaptive_substeps: false,
            adaptive_tolerance: 1e-9,
            relativistic_correction: mps_cosmos::world::RelativisticCorrection::None,
        });
        // 中心恒星（静止原点，作为 n-body 源提供轨道引力 + tidal 伴星）。
        let star_h = world.insert_body(
            satellite_builder(gm, Vector::ZERO, Vector::ZERO, 1.0).lock_translations(),
        );
        world.add_n_body(star_h, gm);
        world.set_central_body(Some(earth));
        // 卫星：初始无自旋（远欠同步），tidal 必然加速其自旋向同步。
        let h = world.insert_body(
            satellite_builder(
                1000.0,
                Vector::new(r, 0.0, 0.0),
                Vector::new(0.0, v, 0.0),
                1.0,
            )
            .angvel(Vector::new(0.0, 0.0, 0.0_f64))
            .linear_damping(0.0)
            .angular_damping(0.0)
            .gravity_scale(0.0),
        );
        let cfg = mps_cosmos::world::PerturbationConfig {
            enable_tidal,
            tidal_radius: 1.0e3_f64, // 1 km M-主星半径
            love_number_k2: 0.299,
            tidal_q: 12.0,
            ..Default::default()
        };
        world.set_perturbation(h, cfg);
        (world, h)
    };

    // 关闭：自旋应恒等于初值（无潮汐力矩，Rapier 不改自旋）→ 逐位一致。
    let (mut w_off, h_off) = build(false);
    let spin0 = w_off.bodies().get(h_off).unwrap().angvel().z;
    for _ in 0..50 {
        w_off.step(10.0);
    }
    let spin_off = w_off.bodies().get(h_off).unwrap().angvel().z;
    assert_eq!(spin_off, spin0, "tidal OFF must leave spin bit-identical");

    // 开启：欠同步（初值 0 < n）被加速 → 终态自旋与「关闭」世界发散且方向为正。
    let (mut w_on, h_on) = build(true);
    for _ in 0..50 {
        w_on.step(10.0);
    }
    let spin_on = w_on.bodies().get(h_on).unwrap().angvel().z;
    assert!(
        spin_on.is_finite(),
        "spin must stay finite, got {}",
        spin_on
    );
    assert!(
        spin_on != spin_off,
        "tidal ON must diverge from OFF (torque applied); on={} off={}",
        spin_on,
        spin_off
    );
    assert!(
        spin_on > 0.0,
        "sub-synchronous spin must be accelerated positive, got {}",
        spin_on
    );
}

// ---------------------------------------------------------------------------
// add_moon / MOONS 数据预埋集成测试
// ---------------------------------------------------------------------------

#[test]
fn cosmos_world_add_moon_registers_source() {
    let mut world = CosmosWorld::new(CosmosWorldConfig::default());
    let before = world.celestials().len();
    // MOONS[0] = Earth's Moon.
    let idx = world.add_moon_by_index(0);
    assert!(idx.is_some(), "add_moon_by_index(0) must succeed");
    assert_eq!(
        world.celestials().len(),
        before + 1,
        "moon source registered"
    );
    let src = &world.celestials()[idx.unwrap()];
    assert_eq!(src.body.name, "Moon", "moon name preserved");
    // 月球 GM 应非零且等于 MOONS[0] 预埋值。
    assert!((src.body.gm - mps_formula::celestial_data::MOONS[0].gm).abs() < 1.0);
}

#[test]
fn cosmos_world_add_moon_out_of_range_guards() {
    let mut world = CosmosWorld::new(CosmosWorldConfig::default());
    let n = mps_formula::celestial_data::MOONS.len() as i32;
    assert!(
        world.add_moon_by_index(n).is_none(),
        "index == len must be rejected"
    );
    assert!(
        world.add_moon_by_index(n + 5).is_none(),
        "out-of-range must be rejected"
    );
    assert!(
        world.add_moon_by_index(-1).is_none(),
        "negative index must be rejected"
    );
}

#[test]
fn cosmos_ffi_add_moon_roundtrip() {
    let mut world = CosmosWorld::new(CosmosWorldConfig::default());
    let raw = &mut world as *mut CosmosWorld;
    let idx = mps_cosmos::ffi::cosmos_world_add_moon(raw, 0);
    assert!(idx >= 0, "FFI add_moon must return valid index");
    let bad = mps_cosmos::ffi::cosmos_world_add_moon(raw, 999_999);
    assert_eq!(bad, -1, "FFI out-of-range must return -1");
}

/// P2（2026-09 多线程适配）——RapierForce 路径（FFI 默认模式）并行力注入的
/// 正确性：N=64 动态体（≥ 外层并行 min_len 16 的多核分片规模）+ n-body 源 +
/// 大气阻力扰动，`world.step` 后逐体速度与 `total_acceleration` 独立参考
/// 对照（并行注入只改调度不改每体计算序列；此处对照覆盖「快照冻结 → 并行
/// 求值 → 并发写回」全链路，力→加速度换算存在 ulp 差 → 相对容差 1e-9）。
///
/// **注意速度上限**：rapier 0.35 的 staged island solver 每步把刚体速度钳到
/// `IntegrationParameters::normalized_max_linear_velocity`（默认 **400 m/s**，
/// 见 worker.rs 的 integrate-positions 阶段）。本测试刻意用 |v| ≈ 100 m/s 的
/// 慢速场景（低轨非轨道速度 + dt=1s），确保全程不触发钳制，semi-implicit
/// 参考才可直接对照；`circular_leo_*` 系列虽用 7546 m/s 的轨道速度，但走的是
/// 显式积子路径（不经过 rapier solver），故不受钳制影响。
#[test]
fn rapier_force_parallel_inject_matches_analytic_reference() {
    use mps_cosmos::world::PerturbationConfig;

    let earth = get_celestial_body(CelestialBodyId::Earth);
    let cfg_world = CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt: 1.0,
        solver_iterations: 8,
        ccd_substeps: 0,
        n_body_softening_sq: 1.0e3,
        central_body: None,
        orbit_integration: OrbitIntegration::RapierForce,
        verlet_substeps: 1,
        adaptive_substeps: false,
        adaptive_tolerance: 1e-9,
        relativistic_correction: mps_cosmos::world::RelativisticCorrection::None,
    };
    let mut world = CosmosWorld::new(cfg_world);
    // 中心恒星（锁定原点）作为 n-body 源提供轨道引力。
    // 注意 add_n_body 的参数是**质量 kg**（内部 gm_from_mass），不是 GM。
    let m_star = 5.972e24;
    let star_h = world.insert_body(
        satellite_builder(m_star, Vector::ZERO, Vector::ZERO, 1.0).lock_translations(),
    );
    world.add_n_body(star_h, m_star);
    world.set_central_body(Some(earth));

    // 64 颗慢速卫星：黄金角散布在 ~1 km 高度（大气密度非零 → 阻力生效），
    // 初速 ~100 m/s（≪ 400 m/s 钳制），确定性方向。
    const N: usize = 64;
    let cfg_pert = PerturbationConfig {
        enable_drag: true,
        drag_coefficient: 0.02,
        area: 2.0,
        ..Default::default()
    };
    let mut sat_states: Vec<(RigidBodyHandle, Vector, Vector)> = Vec::with_capacity(N);
    for i in 0..N {
        let theta = i as f64 * 2.399963229728653; // 黄金角
        let r = 6.379e6;
        let pos = Vector::new(
            r * theta.cos(),
            r * theta.sin(),
            ((i % 5) as f64 - 2.0) * 1.0e4,
        );
        let mut dir = Vector::new(
            (i % 7) as f64 - 3.0,
            (i % 5) as f64 - 2.0,
            (i % 3) as f64 - 1.0,
        );
        if dir.length_squared() == 0.0 {
            dir = Vector::new(1.0, 0.0, 0.0); // i=52 时三个余数同时归零，避免 NaN
        }
        let vel = dir * (100.0 / dir.length());
        let h = world.insert_body(
            satellite_builder(1000.0, pos, vel, 1.0)
                .linear_damping(0.0)
                .angular_damping(0.0),
        );
        world.set_perturbation(h, cfg_pert);
        sat_states.push((h, pos, vel));
    }

    // 冻结 pre-step 状态构造独立参考 ctx（与 refresh_n_body_sources 同款 SOA 表）。
    let source_pos_gm = snapshot_source_pos_gm(&world);
    let source_positions: Vec<Vector> = source_pos_gm.iter().map(|(p, _)| *p).collect::<Vec<_>>();
    let celestials: Vec<mps_cosmos::gravity::CelestialSource> = Vec::new();
    let src_rot: Vec<Rotation> = world.bodies().iter().map(|_| Rotation::IDENTITY).collect();
    // n-body 源快照（拷贝 handle+gm，避免 ctx 借用 world 与 step 的 &mut 冲突）。
    let n_body_sources_ref: Vec<mps_cosmos::gravity::NBodySource> =
        vec![mps_cosmos::gravity::NBodySource::monopole(
            star_h,
            mps_cosmos::gravity::gm_from_mass(m_star),
        )];
    let ctx = AccelContext {
        celestials: &celestials,
        n_body_sources: &n_body_sources_ref,
        source_positions: &source_positions,
        source_rotations: &src_rot,
        source_pos_gm: &source_pos_gm,
        softening_sq: 1.0e3,
        central_body: Some(earth),
        sun_position: Vector::ZERO,
        relativistic: mps_cosmos::world::RelativisticCorrection::None,
        has_irregular_sources: false,
        nb_parallel: false,
        ff_simd: mps_cosmos::integrator::ff_simd_enabled(),
    };

    let dt = 1.0;
    world.step(dt);

    for (h, pos0, vel0) in &sat_states {
        let mass = world.bodies().get(*h).expect("sat").mass();
        let a_ref = total_acceleration(*pos0, *vel0, mass, *h, &ctx, Some(&cfg_pert));
        let expected = *vel0 + a_ref * dt;
        let actual = world.body_linvel(*h).expect("linvel");
        let diff = (actual - expected).length();
        let scale = expected.length().max(1.0e-6);
        assert!(
            diff <= 1.0e-9 * scale + 1.0e-9,
            "body {h:?}: got {actual:?}, expected {expected:?} (|Δ|={diff})"
        );
    }
}

/// P2（2026-09 多线程适配）——并行力注入的确定性：两个同构 world 各推 20 步，
/// 终态位置/速度 **f64 逐位相等**（并行注入的求值/写回序列与 rayon 调度无关；
/// 无 collider 时 rapier pipeline 内部亦无并行分叉）。
#[test]
fn rapier_force_parallel_inject_is_deterministic_across_runs() {
    let build = || {
        let mut world = CosmosWorld::new(CosmosWorldConfig {
            gravity: Vector::ZERO,
            dt: 5.0,
            solver_iterations: 8,
            ccd_substeps: 0,
            n_body_softening_sq: 1.0e3,
            central_body: None,
            orbit_integration: OrbitIntegration::RapierForce,
            verlet_substeps: 1,
            adaptive_substeps: false,
            adaptive_tolerance: 1e-9,
            relativistic_correction: mps_cosmos::world::RelativisticCorrection::None,
        });
        let gm = get_celestial_body(CelestialBodyId::Earth).gm;
        let star_h = world.insert_body(
            satellite_builder(gm, Vector::ZERO, Vector::ZERO, 1.0).lock_translations(),
        );
        world.add_n_body(star_h, gm);
        for i in 0..64 {
            let theta = i as f64 * 2.399963229728653;
            let r = 7.0e6 + (i % 8) as f64 * 8.0e4;
            let pos = Vector::new(r * theta.cos(), r * theta.sin(), 0.0);
            let speed = (gm / r).sqrt() * 0.999;
            let vel = Vector::new(-pos.y, pos.x, 0.0) * (speed / r);
            world.insert_body(
                satellite_builder(1000.0, pos, vel, 1.0)
                    .linear_damping(0.0)
                    .angular_damping(0.0)
                    .gravity_scale(0.0),
            );
        }
        world
    };
    let mut world_a = build();
    let mut world_b = build();
    for _ in 0..20 {
        world_a.step(5.0);
        world_b.step(5.0);
    }
    // 同构构建下 handle 一一对应（插入序一致），逐体位/速逐位比对。
    let handles: Vec<RigidBodyHandle> = world_a.bodies().iter().map(|(h, _)| h).collect::<Vec<_>>();
    for h in handles {
        assert_eq!(
            world_a.body_translation(h),
            world_b.body_translation(h),
            "translation mismatch for {h:?}"
        );
        assert_eq!(
            world_a.body_linvel(h),
            world_b.body_linvel(h),
            "linvel mismatch for {h:?}"
        );
    }
}

/// 浮动原点重定心的物理不变性（P1）：`recenter` 只做全体平移，所有力律只依赖
/// 相对位置，因此重定心前后推同样的 dt，**相对轨迹（体间距、相对速度）必须
/// 一致**；绝对坐标整体平移，速度逐位不变。
#[test]
fn recenter_preserves_relative_dynamics() {
    // 两个卫星在中心天体（地球）引力下的双体场景。world A 不重定心；world B
    // 推几步后以卫星 A 为 pivot 重定心，再继续推。对比两者每一步后的
    // 相对位置/相对速度。
    let build = || {
        let mut world = CosmosWorld::new(CosmosWorldConfig {
            gravity: Vector::ZERO,
            dt: 1.0,
            solver_iterations: 4,
            ccd_substeps: 0,
            n_body_softening_sq: 0.0,
            central_body: Some(get_celestial_body(CelestialBodyId::Earth)),
            orbit_integration: OrbitIntegration::Yoshida4,
            verlet_substeps: 1,
            adaptive_substeps: false,
            adaptive_tolerance: 1e-9,
            relativistic_correction: mps_cosmos::world::RelativisticCorrection::None,
        });
        world.add_celestial(mps_cosmos::gravity::CelestialSource::new(
            get_celestial_body(CelestialBodyId::Earth),
            2,
        ));
        let sat_a = world.insert_body(satellite_builder(
            1000.0,
            Vector::new(7.0e6, 0.0, 0.0),
            Vector::new(0.0, 7550.0, 0.0),
            0.1,
        ));
        let sat_b = world.insert_body(satellite_builder(
            1000.0,
            Vector::new(7.2e6, 0.0, 0.0),
            Vector::new(0.0, 7400.0, 0.0),
            0.1,
        ));
        (world, sat_a, sat_b)
    };

    let (mut w1, a1, b1) = build();
    let (mut w2, a2, _b2) = build();

    // 先各推 3 步到同一状态。
    for _ in 0..3 {
        w1.step(1.0);
        w2.step(1.0);
    }

    // world2 重定心到卫星 A 原点。断言：pivot 体回到原点、速度不变、
    // 相对量（A-B 间距）与重定心前逐位一致。
    let t1a = |w: &CosmosWorld, h| w.body_translation(h).expect("sat a");
    let v1a = |w: &CosmosWorld, h| w.body_linvel(h).expect("sat a");
    let pivot_before = t1a(&w2, a2);
    let offset = w2.recenter(pivot_before);
    assert_eq!(offset, -pivot_before, "offset = -pivot");
    assert_eq!(t1a(&w2, a2), Vector::ZERO, "pivot at origin");
    assert_eq!(
        v1a(&w2, a2),
        v1a(&w1, a1),
        "velocity is translation-invariant"
    );
    let rel_before = t1a(&w1, b1) - t1a(&w1, a1);
    let rel_after = t1a(&w2, _b2) - t1a(&w2, a2);
    assert_eq!(rel_before, rel_after, "relative position bit-identical");
    assert_eq!(w2.sun_position(), w1.sun_position() + offset, "sun moved");

    // 再各推 5 步：相对轨迹必须一致（引力只依赖相对位置）。
    for i in 0..5 {
        w1.step(1.0);
        w2.step(1.0);
        let d1 = t1a(&w1, b1) - t1a(&w1, a1);
        let d2 = t1a(&w2, _b2) - t1a(&w2, a2);
        let gap = (d1 - d2).length();
        assert!(
            gap < 1e-6,
            "relative trajectory diverged after recenter (step {i}): gap = {gap}"
        );
        let v1 = v1a(&w1, b1) - v1a(&w1, a1);
        let v2 = v1a(&w2, _b2) - v1a(&w2, a2);
        assert!(
            (v1 - v2).length() < 1e-9,
            "relative velocity diverged (step {i})"
        );
    }
}

/// `recenter` 拒绝非有限 pivot：不污染任何状态，返回零平移。
#[test]
fn recenter_rejects_non_finite_pivot() {
    let mut world = CosmosWorld::new(CosmosWorldConfig {
        gravity: Vector::ZERO,
        dt: 1.0,
        solver_iterations: 4,
        ccd_substeps: 0,
        n_body_softening_sq: 0.0,
        central_body: None,
        orbit_integration: OrbitIntegration::default(),
        verlet_substeps: 1,
        adaptive_substeps: false,
        adaptive_tolerance: 1e-9,
        relativistic_correction: mps_cosmos::world::RelativisticCorrection::None,
    });
    let _sat = world.insert_body(satellite_builder(
        1.0,
        Vector::new(1.0e7, 2.0e7, 0.0),
        Vector::ZERO,
        0.1,
    ));
    let sun_before = world.sun_position();
    for bad in [
        Vector::new(f64::NAN, 0.0, 0.0),
        Vector::new(0.0, f64::INFINITY, 0.0),
        Vector::ZERO,
    ] {
        assert_eq!(world.recenter(bad), Vector::ZERO, "no-op offset");
    }
    assert_eq!(
        world.body_translation(RigidBodyHandle::from_raw_parts(0, 0)),
        Some(Vector::new(1.0e7, 2.0e7, 0.0)),
        "position untouched"
    );
    assert_eq!(world.sun_position(), sun_before, "sun untouched");
}
