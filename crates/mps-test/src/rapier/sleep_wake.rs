//! A3：挖洞唤醒稳定性（docs/evidence/sleep-wake.md 的数据源）。
//!
//! MC 最高频操作「挖掉堆叠体下方的方块」对应物理引擎的两个压力点：
//! 睡眠链断裂的唤醒冲击、以及挖洞后结构再稳过程的速度尖峰/穿透。
//! 断言上限取「8 格塔势能释放的自由落体合理上限」：
//!   v = sqrt(2·g·h) = sqrt(2·9.81·8) ≈ 12.5 m/s → 上限取 15 m/s。
//!
//! 稳定高度包络（evidence 已实测）：4×4 底、solver 4 迭代下 **8 层稳定
//! （完全休眠）、16 层失稳**——16 层需要 12 迭代才接近稳定。堆叠测试
//! 一律用 8 层塔（MC 高频结构高度），稳定性包络单独断言（测试 4）。
//!
//! 四个测试：底挖、中挖、挖填循环、稳定高度包络。

use mps_core::rapier::ffi::{Bool, Vec3, WorldHandle, convert::pack_rigid_body_handle};
use mps_core::rapier::rigid_body::world_remove_rigid_body;
use mps_core::rapier::world::{
    world_apply_runtime_settings, world_create, world_destroy, world_step,
};
use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder, Vector};

/// RAII world 包装（照 performance_matrix.rs 的先例，不用手工 drop）。
struct World(*mut WorldHandle);
impl Drop for World {
    fn drop(&mut self) {
        world_destroy(self.0);
    }
}

const SPEED_LIMIT: f64 = 15.0;

/// 一座 side×side×height 的立方体塔（间隔 1.001 防初始穿插），
/// 返回 (world, 句柄表)。句柄表按 [layer][col] 存 **FFI 打包句柄**。
fn build_tower(side: usize, height: usize, iters: u32) -> (World, Vec<Vec<u64>>) {
    let world = World(world_create(Vec3 {
        x: 0.0,
        y: -9.81,
        z: 0.0,
    }));
    let w = unsafe { &mut (*world.0).inner };
    // 静态地面。
    w.colliders
        .insert(ColliderBuilder::cuboid(1e3, 0.5, 1e3).build());
    let mut layers = Vec::with_capacity(height);
    for layer in 0..height {
        let mut row = Vec::with_capacity(side);
        for col in 0..side {
            let h = w.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(
                        col as f64 * 1.001,
                        0.5 + layer as f64 * 1.001,
                        0.0,
                    ))
                    .build(),
            );
            let packed = pack_rigid_body_handle(h);
            w.colliders.insert_with_parent(
                ColliderBuilder::cuboid(0.5, 0.5, 0.5).build(),
                h,
                &mut w.bodies,
            );
            row.push(packed);
        }
        layers.push(row);
    }
    assert_eq!(
        world_apply_runtime_settings(world.0, iters, 1, 0, 0, 0, 1),
        Bool::TRUE
    );
    (world, layers)
}

/// 直接访问内层 Rapier 集合的便捷入口（测试镜像允许走内部状态，
/// 与 performance_matrix.rs 的 `w = &mut (*world.0).inner` 同款）。
/// # Safety
/// world 必须是 `build_tower` 返回的有效指针。
unsafe fn inner(world: &World) -> &mps_core::rapier::world::PhysicsWorld {
    &(*world.0).inner
}

/// # Safety
/// world 指针必须有效。
unsafe fn inner_mut(world: &World) -> &mut mps_core::rapier::world::PhysicsWorld {
    &mut (*world.0).inner
}

/// 塔状态快照：所有体的 |v| 最大值与位姿有限性。
/// # Safety
/// world 指针必须有效。
unsafe fn tower_stats(world: &World) -> (f64, bool) {
    let mut max_speed = 0.0f64;
    let mut finite = true;
    for (_, b) in inner(world).bodies.iter() {
        let v = b.linvel().length();
        max_speed = max_speed.max(v);
        let p = b.translation();
        finite &= p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
    }
    (max_speed, finite)
}

/// 挖掉一格：走 FFI（islands/joints 是 pub(crate)，测试侧只能经入口移除）。
/// # Safety
/// world 指针必须有效；handle 必须是本 world 的有效句柄。
unsafe fn remove_body(world: &World, handle_raw: u64) {
    assert_eq!(
        world_remove_rigid_body(world.0, handle_raw, Bool::TRUE),
        Bool::TRUE,
        "dig target must exist"
    );
}

/// 测试 1：挖掉塔底一块 → 上层平滑下落，无速度尖峰、无 NaN。
#[test]
fn dig_under_stack_wake_smoothness() {
    let (world, layers) = build_tower(4, 8, 4);
    // 落定 + 休眠（evidence：8 层 4 迭代 200 步内完全休眠）。
    for _ in 0..200 {
        world_step(world.0, 1.0 / 60.0);
    }
    let (v0, finite0) = unsafe { tower_stats(&world) };
    assert!(finite0);
    assert!(v0 < 1e-3, "tower should settle to sleep, |v|max={v0:.4}");

    // 挖掉底层正中一块。
    let victim = layers[0][2];
    unsafe { remove_body(&world, victim) };

    let mut max_seen = 0.0f64;
    for _ in 0..600 {
        world_step(world.0, 1.0 / 60.0);
        let (v, finite) = unsafe { tower_stats(&world) };
        assert!(finite, "NaN/inf pose after dig");
        max_seen = max_seen.max(v);
    }
    // 全程速度尖峰上限。
    assert!(
        max_seen <= SPEED_LIMIT,
        "speed spike {max_seen:.2} m/s exceeds {SPEED_LIMIT} m/s limit"
    );
    // 结束时塔应重新休眠或速度在衰减（不再有持续弹跳）。
    let (vend, _) = unsafe { tower_stats(&world) };
    assert!(
        vend < 0.5,
        "tower still bouncing after 10s, |v|max={vend:.3}"
    );
}

/// 测试 2：挖中间层一格 → 上半截整体下落，不被单块弹飞。
#[test]
fn dig_middle_column_split_behavior() {
    let (world, layers) = build_tower(4, 8, 4);
    for _ in 0..200 {
        world_step(world.0, 1.0 / 60.0);
    }
    // 挖第 4 层正中一块。
    let victim = layers[4][2];
    unsafe { remove_body(&world, victim) };

    let mut max_seen = 0.0f64;
    for _ in 0..600 {
        world_step(world.0, 1.0 / 60.0);
        let (v, finite) = unsafe { tower_stats(&world) };
        assert!(finite);
        max_seen = max_seen.max(v);
    }
    assert!(
        max_seen <= SPEED_LIMIT,
        "split-fall speed spike {max_seen:.2} m/s exceeds limit"
    );
}

/// 测试 3：挖填循环压力——100 次「挖一格→50 步→填回→50 步」。
/// 无 panic / 无 NaN；塔顶高度不塌超过 15%（挖填的重物会压实下方间隙，
/// 每轮允许 ~0.1 格压缩，100 轮 15% 上限覆盖物理压实 + 求解器微沉降）。
#[test]
fn repeated_dig_fill_churn() {
    let (world, layers) = build_tower(4, 8, 4);
    for _ in 0..200 {
        world_step(world.0, 1.0 / 60.0);
    }
    unsafe {
        // 初始塔顶（最高体的 y）。
        let top0 = inner(&world)
            .bodies
            .iter()
            .map(|(_, b)| b.translation().y)
            .fold(f64::NEG_INFINITY, f64::max);
        let col = 2usize;
        // 每轮记录当前该 (layer, col) 槽位的实际句柄——填回后句柄会变。
        let mut live = layers;
        for cycle in 0..100 {
            let layer = cycle % 8;
            let handle = live[layer][col];
            // 挖。
            remove_body(&world, handle);
            for _ in 0..50 {
                world_step(world.0, 1.0 / 60.0);
            }
            let (v, finite) = tower_stats(&world);
            assert!(finite, "NaN in cycle {cycle}");
            assert!(v <= SPEED_LIMIT, "churn spike {v:.2} in cycle {cycle}");
            // 填回（重建同位置的体），更新句柄表。
            let w = inner_mut(&world);
            let h = w.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(
                        col as f64 * 1.001,
                        0.5 + layer as f64 * 1.001,
                        0.0,
                    ))
                    .build(),
            );
            w.colliders.insert_with_parent(
                ColliderBuilder::cuboid(0.5, 0.5, 0.5).build(),
                h,
                &mut w.bodies,
            );
            live[layer][col] = pack_rigid_body_handle(h);
            for _ in 0..50 {
                world_step(world.0, 1.0 / 60.0);
            }
            let (v, finite) = tower_stats(&world);
            assert!(finite, "NaN after refill cycle {cycle}");
            assert!(v <= SPEED_LIMIT, "refill spike {v:.2} in cycle {cycle}");
        }
        // 最终塔顶不低于初始的 85%。
        let top_end = inner(&world)
            .bodies
            .iter()
            .map(|(_, b)| b.translation().y)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            top_end >= top0 * 0.85,
            "tower height collapsed: {top0:.2} -> {top_end:.2}"
        );
    }
}

/// 测试 4：稳定高度包络——把 evidence 的「4 迭代 8 层稳 / 16 层不稳 /
/// 12 迭代 16 层近似稳」钉住。这是 solver 迭代与堆叠深度的选型依据：
/// MC 大型结构若超过包络，宿主应调高 `worldApplyRuntimeSettings` 迭代数。
#[test]
fn stack_stability_envelope() {
    let probe = |height: usize, iters: u32| -> f64 {
        let (world, _layers) = build_tower(4, height, iters);
        let mut worst = 0.0f64;
        for s in 1..=400 {
            world_step(world.0, 1.0 / 60.0);
            if s > 200 {
                let (v, _) = unsafe { tower_stats(&world) };
                worst = worst.max(v);
            }
        }
        worst
    };

    // 8 层 @ 4 迭代：完全休眠（这是 MC 高频结构的安全高度）。
    let h8_i4 = probe(8, 4);
    assert!(
        h8_i4 < 1e-3,
        "8-layer @ 4 iters should fully sleep, |v|max={h8_i4:.4}"
    );
    // 8 层 @ 12 迭代：同样稳定（冗余迭代无害）。
    let h8_i12 = probe(8, 12);
    assert!(h8_i12 < 1e-3, "8-layer @ 12 iters drifted: {h8_i12:.4}");
    // 16 层 @ 4 迭代：**允许失稳**（包络之外）——但失稳必须是渐进的
    // （前 200 步内 |v| < 2 m/s），不能第一步就爆炸（爆炸=求解器 bug）。
    let (world16, _) = build_tower(4, 16, 4);
    let mut early_max = 0.0f64;
    for s in 1..=200 {
        world_step(world16.0, 1.0 / 60.0);
        let (v, finite) = unsafe { tower_stats(&world16) };
        assert!(finite, "16-layer exploded to NaN at step {s}");
        early_max = early_max.max(v);
    }
    assert!(
        early_max < 2.0,
        "16-layer @ 4 iters must degrade gradually (early |v|max={early_max:.2}), not explode"
    );
    // 16 层 @ 12 迭代：接近稳定（残余抖动 < 0.2 m/s）。
    let h16_i12 = probe(16, 12);
    assert!(
        h16_i12 < 0.2,
        "16-layer @ 12 iters should be near-stable, |v|max={h16_i12:.4}"
    );
}
