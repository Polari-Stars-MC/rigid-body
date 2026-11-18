//! `mps-cosmos` C ABI 的**参数落地**契约（ADR-0040）。
//!
//! ## 为什么需要这份文件
//!
//! `mps-cosmos` 的物理深度已经有很好的覆盖 —— `hohmann_transfer_validation`
//! 用教科书闭式解验 Δv，`convergence` 跑 10 万步验比能量守恒，`drift_matrix`
//! 比较 Verlet 与 Yoshida4Kahan。**缺的不是物理，是 ABI 面**。
//!
//! 逐符号扫 `cosmos.h` 的 42 个入口，有 22 个在 `mps-test` 里一次都没被提到。
//!
//! ## 这份文件测什么
//!
//! 1. **机械等价性**：`step_n(dt, n)` 与 `n` 次 `step(dt)` 必须一致
//! 2. **值落地**：`cosmos_body_mass` 必须返回插入时的质量
//! 3. **三个参数被绕过**（见下，钉住已知设计）
//! 4. **生命周期**：`cosmos_builder_destroy` 能安全销毁未插入的 builder
//!
//! ## 一个必须钉住的 API 陷阱
//!
//! `integrator.rs` 头部注释写明：
//!
//! > 轨道积分（本模块）：位置/线速度由显式 Verlet 推进，**绕过 rapier 的力律**。
//! > rapier `step`：只跑碰撞检测、约束求解、姿态/角速度积分。为此 bodies 设
//! > `gravity_scale=0` 且不注入 `user_force`，由本模块在 step 前后**直接写回**
//! > `translation`/`linvel`。
//!
//! 后果是三个 builder 参数对**被轨道积分器接管的动态体**不起作用：
//!
//! | 入口 | 为什么无效 |
//! | --- | --- |
//! | `cosmos_builder_set_gravity_scale` | 引力来自 n-body Verlet 路径，不看 rapier 的 scale |
//! | `cosmos_builder_set_linear_damping` | rapier `step` 施加阻尼，随即被积分器写回的 `linvel` 覆盖 |
//! | `cosmos_builder_lock_translations` | rapier 求解器约束，被积分器直接改 `translation` 绕过 |
//!
//! 三条签名合法、返回成功、值也进了 builder —— **但对轨道体毫无效果**。
//! 下面的测试**钉住这个现状**，不是认同它。若将来这层分工被修改，测试会响，
//! 提醒同步更新 ADR-0040。
//!
//! ## 不测什么
//!
//! radio 子系统的 8 个 `cosmos_world_radio_*` 入口**不在本文件**。它是个完整
//! 子系统（`radio.rs` 747 行 + `mps-test/cosmos/radio.rs` 657 行），测试走内部
//! API 而非 C ABI。补 ABI 层需要单独一轮，**诚实记录为未覆盖**。

use mps_cosmos::CosmosWorld;
use mps_cosmos::ffi::{
    cosmos_body_mass, cosmos_body_translation_out, cosmos_builder_destroy,
    cosmos_builder_lock_translations, cosmos_builder_set_gravity_scale,
    cosmos_builder_set_linear_damping, cosmos_fixed_body_builder, cosmos_satellite_builder,
    cosmos_world_create, cosmos_world_destroy, cosmos_world_insert_body,
    cosmos_world_insert_body_as_gravity_source, cosmos_world_step, cosmos_world_step_n,
};
use mps_formula::ffi::Vec3;

/// 太阳质量（kg）。作为引力源。
const CENTRAL_MASS: f64 = 1.989e30;

fn make_world() -> *mut CosmosWorld {
    let w = cosmos_world_create(0.01, 4, 1, 1, 1, 1.0e6);
    assert!(!w.is_null(), "cosmos_world_create 返回 null");
    w
}

/// 在原点放一个固定质点作为引力源。
fn add_central(world: *mut CosmosWorld) -> u64 {
    let b = cosmos_fixed_body_builder(0.0, 0.0, 0.0);
    assert!(!b.is_null());
    let id = cosmos_world_insert_body_as_gravity_source(world, b, CENTRAL_MASS);
    assert_ne!(id, 0, "引力源插入失败");
    id
}

/// 用默认参数插入一个卫星体，返回句柄。
fn add_satellite(
    world: *mut CosmosWorld,
    mass: f64,
    px: f64,
    py: f64,
    pz: f64,
    vx: f64,
    vy: f64,
    vz: f64,
) -> u64 {
    let b = cosmos_satellite_builder(mass, px, py, pz, vx, vy, vz, 1.0);
    assert!(!b.is_null(), "satellite_builder 返回 null");
    let id = cosmos_world_insert_body(world, b);
    assert_ne!(id, 0, "卫星插入失败");
    id
}

fn translation(world: *mut CosmosWorld, body: u64) -> (f64, f64, f64) {
    let mut out = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    assert_eq!(cosmos_body_translation_out(world, body, &mut out), 1);
    (out.x, out.y, out.z)
}

fn dist(a: (f64, f64, f64), b: (f64, f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    let dz = a.2 - b.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

// ── 1. step_n 必须等于重复 step ─────────────────────────────────────────

/// `cosmos_world_step_n(w, dt, n)` 的结果必须与 `n` 次 `cosmos_world_step(w, dt)` 相同。
///
/// 机械等价性。能抓到：少跑/多跑一步、`dt` 被误当总时长、批次路径走了不同积分器。
#[test]
fn step_n_matches_repeated_step() {
    let dt = 0.01;
    const N: u32 = 20;

    let wa = make_world();
    add_central(wa);
    let ba = add_satellite(wa, 1000.0, 1.0e7, 0.0, 0.0, 0.0, 7000.0, 0.0);

    let wb = make_world();
    add_central(wb);
    let bb = add_satellite(wb, 1000.0, 1.0e7, 0.0, 0.0, 0.0, 7000.0, 0.0);

    let rc = cosmos_world_step_n(wa, dt, N);
    assert!(rc >= 0, "step_n 返回负值 {rc}");
    for _ in 0..N {
        let rc = cosmos_world_step(wb, dt);
        assert!(rc >= 0, "step 返回负值 {rc}");
    }

    let pa = translation(wa, ba);
    let pb = translation(wb, bb);

    // 必须真的动了，否则等价性是平凡的。
    assert!(
        dist(pa, (1.0e7, 0.0, 0.0)) > 1.0,
        "20 步后卫星几乎没动，等价性断言无意义：{pa:?}"
    );
    assert!(
        dist(pa, pb) < 1e-6,
        "step_n 与重复 step 不一致：{pa:?} vs {pb:?}（差 {}）",
        dist(pa, pb)
    );

    cosmos_world_destroy(wa);
    cosmos_world_destroy(wb);
}

// ── 2. 质量必须落地 ────────────────────────────────────────────────────

/// `cosmos_body_mass` 必须返回插入时给的质量。
#[test]
fn body_mass_reflects_the_inserted_mass() {
    let world = make_world();
    add_central(world);
    let sat = add_satellite(world, 1234.5, 1.0e7, 0.0, 0.0, 0.0, 7000.0, 0.0);

    let m = cosmos_body_mass(world, sat);
    assert!((m - 1234.5).abs() < 1e-6, "插入 1234.5 kg，读回 {m}");

    cosmos_world_destroy(world);
}

// ── 3. 三个 setter 对轨道体无效（钉住文档化设计）──────────────────────

/// `gravity_scale = 0` **不能**关掉轨道体受到的 n-body 引力。
///
/// 钉住 `integrator.rs` 的设计：引力走独立 Verlet 路径，不看 rapier 的 scale。
///
/// 中心质量 `1.989e30`、距离 `1e7 m` → `a ≈ 1.33e6 m/s²`，`0.5 s` 后应下落
/// 约 `1.7e5 m`。
#[test]
fn gravity_scale_does_not_stop_orbital_gravity() {
    let dt = 0.01;
    let world = make_world();
    add_central(world);

    let b = cosmos_satellite_builder(1000.0, 1.0e7, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    assert!(!b.is_null());
    cosmos_builder_set_gravity_scale(b, 0.0);
    let sat = cosmos_world_insert_body(world, b);
    assert_ne!(sat, 0);

    let p0 = translation(world, sat);
    for _ in 0..50 {
        cosmos_world_step(world, dt);
    }
    let moved = dist(p0, translation(world, sat));

    assert!(
        moved > 1.0e4,
        "轨道体的引力不应被 gravity_scale 关掉（integrator.rs 的设计），实测只移动 {moved:.1} m；若这层分工被改，请同步更新本条与 ADR-0040"
    );

    cosmos_world_destroy(world);
}

/// `lock_translations` **不能**冻结被轨道积分器接管的动态体。
#[test]
fn lock_translations_does_not_freeze_an_orbital_body() {
    let dt = 0.01;
    let world = make_world();
    add_central(world);

    let b = cosmos_satellite_builder(1000.0, 1.0e7, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    assert!(!b.is_null());
    cosmos_builder_lock_translations(b);
    let sat = cosmos_world_insert_body(world, b);
    assert_ne!(sat, 0);

    let p0 = translation(world, sat);
    for _ in 0..50 {
        cosmos_world_step(world, dt);
    }
    let moved = dist(p0, translation(world, sat));

    assert!(
        moved > 1.0e4,
        "轨道体不应被 lock_translations 冻结（积分器直接写 translation），实测只移动 {moved:.1} m；若分工改了，请同步更新本条与 ADR-0040"
    );

    cosmos_world_destroy(world);
}

/// `linear_damping` **不能**让被轨道积分器接管的动态体减速。
///
/// 阻尼由 rapier `step` 施加，但积分器随后用 Verlet 结果覆盖 `linvel`。
#[test]
fn linear_damping_does_not_slow_an_orbital_body() {
    let dt = 0.01;
    let world = make_world();
    // 不放引力源：唯一可能的速度变化来源只能是阻尼。
    let b = cosmos_satellite_builder(1000.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 1.0);
    assert!(!b.is_null());
    cosmos_builder_set_linear_damping(b, 50.0); // 极大阻尼，时间常数 0.02 s
    let sat = cosmos_world_insert_body(world, b);
    assert_ne!(sat, 0);

    let p0 = translation(world, sat);
    for _ in 0..50 {
        cosmos_world_step(world, dt);
    }
    let travelled = dist(p0, translation(world, sat));
    let undamped = 100.0 * 50.0 * dt; // 50 m

    assert!(
        travelled > undamped * 0.9,
        "轨道体的 linear_damping 被绕过了（预期走满 {undamped} m），实测 {travelled:.3} m；若分工改了，请同步更新本条与 ADR-0040"
    );

    cosmos_world_destroy(world);
}

// ── 4. builder_destroy 必须能安全销毁未使用的 builder ───────────────────

/// 创建 builder 但**不插入**世界，然后销毁 —— 不得崩溃。
///
/// 正常流程里 `world_insert_*` 会 `Box::from_raw` 消费 builder；`builder_destroy`
/// 只用于「建了又不想要」的路径。两条 builder 构造路径各测一次。
#[test]
fn builder_destroy_accepts_an_unused_builder() {
    let b = cosmos_satellite_builder(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    assert!(!b.is_null());
    cosmos_builder_destroy(b);

    let f = cosmos_fixed_body_builder(1.0, 2.0, 3.0);
    assert!(!f.is_null());
    cosmos_builder_destroy(f);
}
