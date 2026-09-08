//! `mps_cosmos::ffi` 入口测试 —— FFI 批量快照接口（性能分析.MD §11.1/§12.1，
//! M1 + L1 落地的回归守护）。
//!
//! 此前 cosmos FFI 完全没有 batch snapshot 等价：MC mod 端要读 N 个卫星位置
//! 必须 N 次 `cosmos_body_translation_out` 往返 JNI（N=1000 时 ~600µs/tick 的
//! dispatch 开销 + minor GC 抖动）。新加的 `cosmos_world_dynamic_body_snapshot`
//! 把整批合并成一次 native 调用 + 一份连续 f64[]，延迟降到 ~50µs/tick。
//!
//! 本文件直接调 FFI 函数（同 crate 等价于 JNI 路径调 FFI），不经过 JNI 宏，
//! 以便离 JVM 一层运行——这正是 mps-cosmos `ffi.rs` 注释里允诺的 ABI 形态。
//! 验证要点（任一被破坏即视为 ABI 回归）：
//! 1. count 与节点数一致；
//! 2. handle 编码 = (idx << 32) | generation —— 与既有 cosmos per-body handle
//!    一致，可通过 `unpack_handle` 还原；
//! 3. pos 拍平到 `values[i*7..i*7+3]`，与 per-body `cosmos_body_translation_out`
//!    返回值逐项相等；
//! 4. 容量小于 N 时只填前 `capacity` 个体，其它不写超出；
//! 5. null world / null 缓冲 / 0 容量 → 安全返回 0（不 panic / 不 UB）。

#![cfg(test)]

use mps_cosmos::bodies::satellite_builder;
use mps_cosmos::ffi::{
    cosmos_world_dynamic_body_snapshot, cosmos_world_dynamic_body_snapshot_count,
};
use mps_cosmos::world::{CosmosWorld, CosmosWorldConfig, OrbitIntegration, RelativisticCorrection};
use rapier3d::prelude::Vector;

/// 默认 cosmos 配置（不注册任何天体 / sun；用于快照计数 + 拍平布局的纯结构性测试）。
fn empty_world() -> CosmosWorld {
    CosmosWorld::new({
        CosmosWorldConfig {
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
            relativistic_correction: RelativisticCorrection::None,
        }
    })
}

/// 还原 `pack_handle = (idx << 32) | generation`（与 mps-cosmos `ffi.rs::unpack_handle`
/// 完全相同的位运算）—— 测试侧独立写一份避免直接依赖 mps_cosmos::ffi 内部 fn。
fn unpack_handle(packed: u64) -> (u32, u32) {
    let idx = ((packed >> 32) & 0xFFFF_FFFF) as u32;
    let generation = (packed & 0xFFFF_FFFF) as u32;
    (idx, generation)
}

#[test]
fn radio_buffers_reject_null_and_misaligned_pointers() {
    use mps_cosmos::ffi::{
        cosmos_world_radio_register_node, cosmos_world_radio_submit_signal,
        cosmos_world_radio_take_results,
    };
    use mps_formula::error::{ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, error_code};

    let mut world = empty_world();
    world.enable_radio();
    let mut storage = [0.0_f64; 18];
    // The offset stays within a live allocation but violates f64 alignment.
    let misaligned = storage
        .as_mut_ptr()
        .cast::<u8>()
        .wrapping_add(1)
        .cast::<f64>();
    for ptr in [std::ptr::null_mut(), misaligned] {
        let expected = if ptr.is_null() {
            ERR_NULL_POINTER
        } else {
            ERR_INVALID_ARGUMENT
        };
        clear_error();
        assert_eq!(cosmos_world_radio_register_node(&mut world, ptr), 0);
        assert_eq!(error_code(), expected);
        clear_error();
        assert_eq!(cosmos_world_radio_submit_signal(&mut world, ptr), 0);
        assert_eq!(error_code(), expected);
        clear_error();
        assert_eq!(cosmos_world_radio_take_results(&mut world, ptr, 1), 0);
        assert_eq!(error_code(), expected);
    }
    assert_eq!(storage, [0.0; 18]);
}

#[test]
fn radio_buffers_accept_documented_seventeen_field_layout() {
    use mps_cosmos::ffi::{cosmos_world_radio_register_node, cosmos_world_radio_submit_signal};
    let mut world = empty_world();
    world.enable_radio();
    let node = [
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1e9, 1.0, 1e-12, 1.0, 1.0, 1.0, 0.0,
    ];
    let signal = [
        1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1e9, 1.0, 1.0, 1.0, 0.0,
    ];
    assert_eq!(
        cosmos_world_radio_register_node(&mut world, node.as_ptr()),
        1
    );
    assert_eq!(
        cosmos_world_radio_submit_signal(&mut world, signal.as_ptr()),
        1
    );
}

#[test]
fn snapshot_rejects_misaligned_outputs_without_writing() {
    use mps_formula::error::{ERR_INVALID_ARGUMENT, clear_error, error_code};
    let mut storage = [0_u64; 8];
    let mut values = [0.0_f64; 7];
    let world = empty_world();
    let ptr = storage
        .as_mut_ptr()
        .cast::<u8>()
        .wrapping_add(1)
        .cast::<u64>();
    clear_error();
    assert_eq!(
        cosmos_world_dynamic_body_snapshot(&world, ptr, values.as_mut_ptr(), 1),
        0
    );
    assert_eq!(error_code(), ERR_INVALID_ARGUMENT);
    assert_eq!(storage, [0; 8]);
    assert_eq!(values, [0.0; 7]);
}

#[test]
fn snapshot_count_matches_world_dynamic_body_count() {
    let mut world = empty_world();
    // 起先无 body。
    assert_eq!(cosmos_world_dynamic_body_snapshot_count(&world), 0);

    // 插入 3 个体（前两个动态卫星 + 第三个固定体）。
    let _sat1 = world.insert_body(satellite_builder(
        1.0,
        Vector::new(1.0, 0.0, 0.0),
        Vector::ZERO,
        0.1,
    ));
    let _sat2 = world.insert_body(satellite_builder(
        2.0,
        Vector::new(2.0, 0.0, 0.0),
        Vector::ZERO,
        0.1,
    ));
    let _ = world.insert_body(mps_cosmos::bodies::fixed_body_builder(Vector::new(
        0.0, 0.0, 0.0,
    )));

    // 只数动态体（fixed 不算），与 `world.dynamic_body_count()` 一致。
    assert_eq!(cosmos_world_dynamic_body_snapshot_count(&world), 2);
}

#[test]
fn snapshot_writes_pos_and_handles_in_expected_layout() {
    let mut world = empty_world();
    let sat1 = world.insert_body(satellite_builder(
        1.0,
        Vector::new(10.0, 20.0, 30.0),
        Vector::ZERO,
        0.1,
    ));
    let sat2 = world.insert_body(satellite_builder(
        2.0,
        Vector::new(40.0, 50.0, 60.0),
        Vector::ZERO,
        0.1,
    ));

    // 预分配：2 体 × 7 f64 + 2 个 u64 handle。
    let mut handles = vec![0u64; 2];
    let mut values = vec![0f64; 2 * 7];
    let n =
        cosmos_world_dynamic_body_snapshot(&world, handles.as_mut_ptr(), values.as_mut_ptr(), 2);
    assert_eq!(n, 2, "snapshot should write both bodies");

    // handle 还原后应能映射回 RigidBodyHandle（与 per-body 路径一致）。
    let (id1, gen1) = unpack_handle(handles[0]);
    let (id2, _gen2) = unpack_handle(handles[1]);
    let (expect1, expect2) = (sat1.into_raw_parts(), sat2.into_raw_parts());
    assert_eq!((id1, gen1), expect1, "handle[0] encoding");
    assert_eq!((id2, _gen2), expect2, "handle[1] encoding");

    // pos 拍平验证（values[i*7..i*7+3]），与 per-body `body_translation` 等价。
    // bodies.iter() 按插入序输出 → sat1 在前，sat2 在后。
    assert_eq!(values[0..3], [10.0, 20.0, 30.0], "sat1 pos");
    assert_eq!(values[7..10], [40.0, 50.0, 60.0], "sat2 pos");

    // 旋转槽 [3..7] 应为 identity (0,0,0,1)——satellite_builder 不自定义 rotation。
    assert_eq!(values[3..7], [0.0, 0.0, 0.0, 1.0], "sat1 identity quat");
    assert_eq!(values[10..14], [0.0, 0.0, 0.0, 1.0], "sat2 identity quat");
}

#[test]
fn snapshot_truncates_to_capacity_without_overflow() {
    let mut world = empty_world();
    for i in 0..5 {
        let _ = world.insert_body(satellite_builder(
            1.0,
            Vector::new(i as f64, 0.0, 0.0),
            Vector::ZERO,
            0.1,
        ));
    }
    assert_eq!(cosmos_world_dynamic_body_snapshot_count(&world), 5);

    // 容量 3：只能写入 3 个体，不超出缓冲。
    let mut handles = vec![u64::MAX; 5];
    let mut values = vec![f64::NAN; 5 * 7];
    let n =
        cosmos_world_dynamic_body_snapshot(&world, handles.as_mut_ptr(), values.as_mut_ptr(), 3);
    assert_eq!(n, 3);
    // 未写入部分保持 sentinel（"未污染 capacity 之外"——FFI 不应越界写入）。
    assert_eq!(handles[3], u64::MAX);
    assert_eq!(handles[4], u64::MAX);
    assert!(values[3 * 7].is_nan());
    assert!(values[4 * 7].is_nan());
}

#[test]
fn snapshot_handles_null_world_and_buffers_safely() {
    use mps_cosmos::ffi::cosmos_world_dynamic_body_count;
    use std::ptr;

    // null world → 0，且不 panic。
    assert_eq!(cosmos_world_dynamic_body_snapshot_count(ptr::null()), 0);
    assert_eq!(cosmos_world_dynamic_body_count(ptr::null()), 0);
    let mut h = vec![0u64; 1];
    let mut v = vec![0f64; 7];
    assert_eq!(
        cosmos_world_dynamic_body_snapshot(ptr::null(), h.as_mut_ptr(), v.as_mut_ptr(), 1),
        0
    );

    // 有效 world + null 缓冲 → 0。
    let world = empty_world();
    assert_eq!(
        cosmos_world_dynamic_body_snapshot(&world, ptr::null_mut(), v.as_mut_ptr(), 1),
        0
    );
    assert_eq!(
        cosmos_world_dynamic_body_snapshot(&world, h.as_mut_ptr(), ptr::null_mut(), 1),
        0
    );

    // 0 容量 → 0。
    assert_eq!(
        cosmos_world_dynamic_body_snapshot(&world, h.as_mut_ptr(), v.as_mut_ptr(), 0),
        0
    );
}

#[test]
fn snapshot_round_trip_against_per_body_translation_out() {
    // 验证 batch snapshot 与逐体 `cosmos_body_translation_out` 返回 pos 完全一致
    // —— 这是 M1+L1 改动的核心契约：Java 端可以从 N 次 jni 往返迁到 1 次 batch
    // 调用，得到完全相同的 pos 数值（这是替代 per-body 路径的前提）。
    use mps_cosmos::ffi::cosmos_body_translation_out;
    use mps_formula::ffi::Vec3;

    let mut world = empty_world();
    let h1 = world.insert_body(satellite_builder(
        1.0,
        Vector::new(1.5, 2.5, 3.5),
        Vector::ZERO,
        0.1,
    ));
    let h2 = world.insert_body(satellite_builder(
        2.0,
        Vector::new(-4.0, 0.5, 7.2),
        Vector::ZERO,
        0.1,
    ));

    // per-body 路径：循环 N 次取 pos。
    let mut per_body_pos = [Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }; 2];
    let pack = |h: rapier3d::prelude::RigidBodyHandle| -> u64 {
        let (idx, generation) = h.into_raw_parts();
        ((idx as u64) << 32) | (generation as u64)
    };
    // FFI 签名是 `out: *mut Vec3`——这里直接传裸指针，C-ABI 入口在 panic=abort
    // 防护下不会 UB（`world` 也是 `*const CosmosWorld`，Rust 引用强转裸指针对齐）。
    let _ =
        cosmos_body_translation_out(&world as *const _, pack(h1), &mut per_body_pos[0] as *mut _);
    let _ =
        cosmos_body_translation_out(&world as *const _, pack(h2), &mut per_body_pos[1] as *mut _);

    // batch 路径：一次拉。
    let mut handles = vec![0u64; 2];
    let mut values = vec![0f64; 2 * 7];
    let n =
        cosmos_world_dynamic_body_snapshot(&world, handles.as_mut_ptr(), values.as_mut_ptr(), 2);
    assert_eq!(n, 2);

    // 两个 body 的 pos 必须无误差相等。
    assert_eq!(
        values[0..3],
        [per_body_pos[0].x, per_body_pos[0].y, per_body_pos[0].z]
    );
    assert_eq!(
        values[7..10],
        [per_body_pos[1].x, per_body_pos[1].y, per_body_pos[1].z]
    );
}

// ===========================================================================
// Shared arena zero-copy round-trip (Java 命令环 → Rust drain → 状态回读)
// ===========================================================================
//
// 这些测试直接对 arena 共享内存做 native-order 读写（与 Java 侧 `ByteBuffer`
// 映射等价），验证零拷贝路径：
//   - Java 把命令写进命令环（命令类型 + body 索引 + 3 个 f64 参数）；
//   - Rust 在 `step` 头部 `drain_commands` 应用命令、尾部 `flush_all_bodies`
//     把刚体状态拍平进 body 槽；
// - 全程无一次 per-body JNI 往返。
//
// 布局常量来自 `mps_cosmos::arena`，与 `docs/cosmos-arena.md` 一致。任一偏移
// 被破坏即视为 ABI 回归。

use mps_cosmos::arena::{
    ARENA_MAGIC, ARENA_VERSION, BODY_SLOT_STRIDE, CMD_SLOT_STRIDE, HEADER_SIZE, OFF_BODY_COUNT,
    OFF_CMD_RING, OFF_CMD_WRITE,
};

/// 把 arena 基地址映射成可写裸指针，便于测试侧模拟 Java 的命令写入。
fn arena_mut(addr: u64) -> *mut u8 {
    addr as *mut u8
}

/// 读 body 槽的线速度（零拷贝回读），索引 `i`。
unsafe fn read_body_linvel(base: *mut u8, i: usize) -> [f64; 3] {
    unsafe {
        let slot = base.add(HEADER_SIZE + i * BODY_SLOT_STRIDE as usize);
        [
            (slot.add(32) as *const f64).read_unaligned(),
            (slot.add(40) as *const f64).read_unaligned(),
            (slot.add(48) as *const f64).read_unaligned(),
        ]
    }
}

/// 读 body 槽的位置（零拷贝回读）。
unsafe fn read_body_pos(base: *mut u8, i: usize) -> [f64; 3] {
    unsafe {
        let slot = base.add(HEADER_SIZE + i * BODY_SLOT_STRIDE as usize);
        [
            (slot.add(8) as *const f64).read_unaligned(),
            (slot.add(16) as *const f64).read_unaligned(),
            (slot.add(24) as *const f64).read_unaligned(),
        ]
    }
}

/// 在命令环写入一条命令（Java 侧等价操作），并 bump `cmd_write`。
unsafe fn write_command(
    base: *mut u8,
    cmd_ring_base: usize,
    slot: usize,
    cmd_type: u64,
    body_index: u64,
    a: [f64; 3],
) {
    unsafe {
        let p = base.add(cmd_ring_base + slot * CMD_SLOT_STRIDE as usize);
        (p as *mut u64).write_unaligned(cmd_type);
        (p.add(8) as *mut u64).write_unaligned(body_index);
        (p.add(16) as *mut u64).write_unaligned(a[0].to_bits());
        (p.add(24) as *mut u64).write_unaligned(a[1].to_bits());
        (p.add(32) as *mut u64).write_unaligned(a[2].to_bits());
    }
}

#[test]
fn arena_header_is_valid_after_create() {
    let mut world = empty_world();
    let _sat = world.insert_body(satellite_builder(1.0, Vector::ZERO, Vector::ZERO, 0.1));
    assert!(world.create_shared_arena(64, 64), "arena create");

    let addr = world.shared_arena_address();
    let size = world.shared_arena_size();
    assert_ne!(addr, 0);
    assert!(size > HEADER_SIZE as u64);

    // 零拷贝读 header：magic / version / body_count。
    unsafe {
        let base = arena_mut(addr);
        let magic = (base as *const u64).read_unaligned();
        assert_eq!(magic, ARENA_MAGIC, "arena magic");
        let version = (base.add(8) as *const u32).read_unaligned();
        assert_eq!(version, ARENA_VERSION, "arena version");
        let body_count = (base.add(OFF_BODY_COUNT) as *const u32).read_unaligned();
        // 还未 step，flush 未跑，body_count 仍为 0；但 layout 合法。
        assert_eq!(body_count, 0);
    }

    world.destroy_shared_arena();
    assert_eq!(world.shared_arena_address(), 0, "arena freed");
}

#[test]
fn arena_set_velocity_command_roundtrip_zero_copy() {
    let mut world = empty_world();
    // 默认 orbit_integration = Yoshida4（显式积子路径）。
    let _sat = world.insert_body(satellite_builder(
        1.0,
        Vector::new(0.0, 0.0, 0.0),
        Vector::ZERO,
        0.1,
    ));
    assert!(world.create_shared_arena(64, 64));

    let addr = world.shared_arena_address();
    let base = arena_mut(addr);
    unsafe {
        // 命令环基地址由 header 在 `new` 时写入（OFF_CMD_RING）。
        let cmd_ring_base = (base.add(OFF_CMD_RING) as *const u64).read_unaligned() as usize;
        // Java 写入 SetVelocity (type=2) 给 body 0 → (1, 2, 3)。
        write_command(base, cmd_ring_base, 0, 2, 0, [1.0, 2.0, 3.0]);
        // bump cmd_write = 1（Java 写满 1 条命令）。
        (base.add(OFF_CMD_WRITE) as *mut u32).write_unaligned(1);
    }

    // Rust 侧 step：头部 drain 命令（SetVelocity）→ 尾部 flush body 槽。
    world.step(0.5);

    // 零拷贝回读：body 0 的 linvel 应等于命令值；位置被显式积子推进（引力为 0，
    // 仅匀速直线运动 → 位移 = v·dt = (0.5, 1.0, 1.5)）。
    unsafe {
        let vel = read_body_linvel(base, 0);
        assert!(
            (vel[0] - 1.0).abs() < 1e-9
                && (vel[1] - 2.0).abs() < 1e-9
                && (vel[2] - 3.0).abs() < 1e-9,
            "linvel after SetVelocity command: {vel:?}"
        );
        let pos = read_body_pos(base, 0);
        assert!(
            (pos[0] - 0.5).abs() < 1e-9
                && (pos[1] - 1.0).abs() < 1e-9
                && (pos[2] - 1.5).abs() < 1e-9,
            "pos after SetVelocity command (v·dt): {pos:?}"
        );
        // body_count 回读应为 1（flush 已写 1 体）。
        let body_count = (base.add(OFF_BODY_COUNT) as *const u32).read_unaligned();
        assert_eq!(body_count, 1, "body_count after step");
        // 命令环已被 drain 清空（cmd_write 复位）。
        let cmd_write = (base.add(OFF_CMD_WRITE) as *const u32).read_unaligned();
        assert_eq!(cmd_write, 0, "cmd ring drained");
    }

    world.destroy_shared_arena();
}

#[test]
fn arena_add_force_command_roundtrip_explicit_path() {
    let mut world = empty_world();
    // 显式积子路径（Yoshida4）。命令力 F 应折成 a=F/m 注入半隐式欧拉修正。
    let _sat = world.insert_body(satellite_builder(
        2.0, // mass = 2 kg
        Vector::new(0.0, 0.0, 0.0),
        Vector::ZERO,
        0.1,
    ));
    assert!(world.create_shared_arena(64, 64));

    let addr = world.shared_arena_address();
    let base = arena_mut(addr);
    unsafe {
        let cmd_ring_base = (base.add(OFF_CMD_RING) as *const u64).read_unaligned() as usize;
        // AddForce (type=0) 给 body 0 → (4, 0, 0) N。a = F/m = (2, 0, 0)。
        write_command(base, cmd_ring_base, 0, 0, 0, [4.0, 0.0, 0.0]);
        (base.add(OFF_CMD_WRITE) as *mut u32).write_unaligned(1);
    }

    world.step(1.0); // dt = 1.0

    // 零拷贝回读：Δv = a·dt = (2,0,0)；位移 ≈ ½·a·dt² = (1,0,0)（半隐式修正 + 积子）。
    unsafe {
        let vel = read_body_linvel(base, 0);
        assert!(
            (vel[0] - 2.0).abs() < 1e-6,
            "linvel after AddForce (a·dt): {vel:?}"
        );
        let pos = read_body_pos(base, 0);
        assert!(
            pos[0] > 0.9 && pos[0] < 1.1,
            "pos after AddForce (½·a·dt²): {pos:?}"
        );
    }

    world.destroy_shared_arena();
}

#[test]
fn arena_ffi_create_get_address_size_roundtrip() {
    // 经 FFI 入口验证（与 JNI 路径等价）：create 返回地址/大小，get_* 读回一致。
    use mps_cosmos::ffi::{
        cosmos_world_create_shared_arena, cosmos_world_destroy_shared_arena,
        cosmos_world_get_shared_arena_address, cosmos_world_get_shared_arena_size,
    };

    let mut world = empty_world();
    let _sat = world.insert_body(satellite_builder(1.0, Vector::ZERO, Vector::ZERO, 0.1));

    let mut out_addr: u64 = 0;
    let mut out_size: u64 = 0;
    let ok = cosmos_world_create_shared_arena(
        &mut world as *mut _,
        128,
        128,
        &mut out_addr as *mut u64,
        &mut out_size as *mut u64,
    );
    assert_eq!(ok, 1, "ffi create ok");
    assert_eq!(
        out_addr,
        cosmos_world_get_shared_arena_address(&world as *const _)
    );
    assert_eq!(
        out_size,
        cosmos_world_get_shared_arena_size(&world as *const _)
    );
    assert!(out_addr != 0 && out_size > HEADER_SIZE as u64);

    cosmos_world_destroy_shared_arena(&mut world as *mut _);
    assert_eq!(cosmos_world_get_shared_arena_address(&world as *const _), 0);
}

/// P2（2026-09 多线程适配）——并行快照分支（≥ 128 体走 rayon 保序并行计算）
/// 与世界状态逐值一致：handle 可回解、位置与 `body.translation()` 逐位相同，
/// 动态体计数一致（fixed 体被排除）。
#[test]
fn snapshot_parallel_branch_matches_world_state() {
    const N: usize = 300; // ≥ SNAPSHOT_PARALLEL_MIN(128) → rayon 并行分支
    let mut world = empty_world();
    for i in 0..N {
        let pos = Vector::new(i as f64 * 1.5, -(i as f64) * 0.75, i as f64 * 3.25);
        let vel = Vector::new(i as f64 * 0.1, i as f64, -(i as f64) * 0.2);
        world.insert_body(satellite_builder(1000.0, pos, vel, 0.1));
    }
    // 一个 fixed 体：动态快照必须排除它。
    let _ = world.insert_body(mps_cosmos::bodies::fixed_body_builder(Vector::new(
        9999.0, 0.0, 0.0,
    )));

    assert_eq!(cosmos_world_dynamic_body_snapshot_count(&world), N as u32);

    let mut handles = vec![0u64; N];
    let mut values = vec![0f64; N * 7];
    let written = cosmos_world_dynamic_body_snapshot(
        &world,
        handles.as_mut_ptr(),
        values.as_mut_ptr(),
        N as u32,
    );
    assert_eq!(written as usize, N);

    for (i, handle_raw) in handles.iter().enumerate() {
        let (idx, generation) = unpack_handle(*handle_raw);
        let h = rapier3d::prelude::RigidBodyHandle::from_raw_parts(idx, generation);
        let body = world.bodies().get(h).expect("handle must resolve");
        assert!(body.is_dynamic(), "fixed body leaked into dynamic snapshot");
        let t = body.translation();
        assert_eq!(values[i * 7], t.x, "tx mismatch at {i}");
        assert_eq!(values[i * 7 + 1], t.y, "ty mismatch at {i}");
        assert_eq!(values[i * 7 + 2], t.z, "tz mismatch at {i}");
        // 四元数 w 分量逐位一致（快照只读位姿，不引入任何换算）。
        assert_eq!(
            values[i * 7 + 6],
            body.rotation().w,
            "quat w mismatch at {i}"
        );
    }
}
