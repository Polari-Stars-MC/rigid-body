//! mps-cosmos — 太空刚体演算。
//!
//! 基于 `rapier3d-f64` 维护一套太空场景物理世界，使用 `mps-formula`
//! 提供的天体数据、引力模型与积分器施加天体重力、n-body 互引力及
//! 环境扰动力。
//!
//! 与 `mps-core` 不同，本 crate 是一个独立的太空演练器，自行持有
//! `RigidBodySet`/`PhysicsPipeline` 等后端，仅复用 `mps-formula` 的纯
//! 计算函数，不介入 `mps-core` 的 C ABI / 共享 arena / 力律登记表。
//!
//! mps-cosmos 的 C ABI 由 `ffi` 模块导出（`cosmos_*` 符号），由 cbindgen
//! 生成 `include/cosmos.h`，被 `mps-jni`（JNI）与 `mps-ffm`（FFM）
//! 共同消费。

// C ABI entry points validate raw pointers at the boundary (null checks plus
// `ffi_guard`), so the safe-fn-raw-pointer lint is noise here.
#![allow(clippy::not_unsafe_ptr_arg_deref)]

pub extern crate rapier3d;

/// H. `explicit_substep` 阶段耗时剖分（env-gated，默认零开销）。
///
/// 仅在环境变量 `COSMOS_PROFILE=1` 时由 `world::explicit_substep` 调用。
/// 用 `thread_local` 累加器记录 4 段（collect / refresh / advance /
/// writeback）累计耗时与调用次数，首次调用打印表头，之后每 1000 子步
/// 打印一次占比快照（stdout，固定前缀 `[COSMOS-PROFILE]` 便于 grep）。
///
/// 不设 `COSMOS_PROFILE` 时 `world.rs` 的 `profile_phase!` 宏直接短路、
/// 不进入本函数，故零开销、不污染任何数值/语义（守「原方法不变」）。
#[doc(hidden)]
pub fn __cosmos_profile_record(phase: &'static str, elapsed: std::time::Duration) {
    use std::cell::RefCell;
    thread_local! {
        static ACC: RefCell<Option<ProfileAcc>> = const { RefCell::new(None) };
    }
    struct ProfileAcc {
        collect_ns: u64,
        refresh_ns: u64,
        advance_ns: u64,
        writeback_ns: u64,
        steps: u64,
    }
    ACC.with(|acc| {
        let mut g = acc.borrow_mut();
        let a = g.get_or_insert_with(|| {
            println!(
                "[COSMOS-PROFILE] step  phase%  collect   refresh   advance  writeback  (all µs, cumulative)"
            );
            ProfileAcc { collect_ns: 0, refresh_ns: 0, advance_ns: 0, writeback_ns: 0, steps: 0 }
        });
        match phase {
            "collect" => a.collect_ns += elapsed.as_nanos() as u64,
            "refresh" => a.refresh_ns += elapsed.as_nanos() as u64,
            "advance" => a.advance_ns += elapsed.as_nanos() as u64,
            "writeback" => a.writeback_ns += elapsed.as_nanos() as u64,
            _ => {}
        }
        a.steps += 1;
        if a.steps % 1000 == 0 {
            let total = a.collect_ns + a.refresh_ns + a.advance_ns + a.writeback_ns;
            let total = total.max(1);
            let pct = |ns: u64| format!("{:5.1}%", ns as f64 * 100.0 / total as f64);
            println!(
                "[COSMOS-PROFILE] {:>5}  {:>7}  {:>7}  {:>7}  {:>7}  (total {:.1} ms)",
                a.steps,
                pct(a.collect_ns),
                pct(a.refresh_ns),
                pct(a.advance_ns),
                pct(a.writeback_ns),
                total as f64 / 1e6,
            );
        }
    });
}

pub mod arena;
pub mod bodies;
pub mod ffi;
pub mod flight;
pub mod gravity;
pub mod integrator;
pub mod orbit;
pub mod orbit_diagnostics;
pub mod perturbation;
pub mod radio;
pub mod world;

pub use world::{CosmosWorld, CosmosWorldConfig};
