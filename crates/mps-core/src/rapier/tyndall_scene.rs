// ── 粒子云 → 体积散射的 C ABI（ADR-0019）─────────────────────────────────────
//
// 公式在 `mps_formula::tyndall_particles`，这里负责**从 WorldHandle 读粒子**再
// 调公式。这是与 `emag.rs` 的 tyndall 包装最大的不同：那些是纯计算器，这些要碰
// 世界状态。
//
// ## 为什么要单独一个模块
//
// `emag.rs` 的文档明确写着「No WorldHandle / Rapier state is touched」。把读
// 粒子云的入口塞进 `emag.rs` 会破坏这个约定，所以独立成 `tyndall_scene.rs`。
//
// ## 两种光源
//
// 流体（SPH）和颗粒（DEM）的粒子结构不同，但散射积分只看**位置**，所以两条读
// 粒子的路径汇到同一个公式。

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{Bool, Vec3, WorldHandle, vec3_finite, vec3_from_rapier};
/// 粒子云来源。
pub const TYNDALL_SOURCE_FLUID: u32 = 0;
/// 颗粒（DEM）粒子云。
pub const TYNDALL_SOURCE_GRANULAR: u32 = 1;

/// 已分配的粒子云来源个数。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_source_abi_version() -> u32 {
    ffi_guard(0, || 2)
}

/// 收集指定来源的粒子位置到内部缓冲区并返回粒子数。
///
/// 返回值是**总粒子数**（不是拷贝数）—— 与 `granular_read_particles` /
/// `soft_body_read_*` 的约定一致：调用方先用 `out == null` 探总数，再按需分配。
///
/// # Returns
/// 粒子总数；world 为 null / id 未知 → 0 并设置错误码。
///
/// # Safety
/// `world` 必须有效；`out` 可为 null（只探数量）或指向至少 `capacity` 个 `Vec3`。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_collect_particles(
    world: *const WorldHandle,
    source: u32,
    id: u32,
    out: *mut Vec3,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "tyndall_collect_particles: world is null");
            return 0;
        };
        let idx = id as usize;
        let cap = capacity as usize;

        match source {
            TYNDALL_SOURCE_FLUID => {
                if idx >= world.inner.fluids.len() {
                    set_error(ERR_NOT_FOUND, "tyndall_collect_particles: unknown fluid id");
                    return 0;
                }
                let fluid = &world.inner.fluids[idx];
                let n = fluid.len();
                if !out.is_null() {
                    let dst = unsafe { std::slice::from_raw_parts_mut(out, cap.min(n)) };
                    for (dst, src) in dst.iter_mut().zip(fluid.particles.iter()) {
                        *dst = vec3_from_rapier(src.pos);
                    }
                }
                clear_error();
                n as u32
            }
            TYNDALL_SOURCE_GRANULAR => {
                if idx >= world.inner.granular_bodies.len() {
                    set_error(
                        ERR_NOT_FOUND,
                        "tyndall_collect_particles: unknown granular id",
                    );
                    return 0;
                }
                let g = &world.inner.granular_bodies[idx];
                let n = g.len();
                if !out.is_null() {
                    let dst = unsafe { std::slice::from_raw_parts_mut(out, cap.min(n)) };
                    for (dst, src) in dst.iter_mut().zip(g.particles.iter()) {
                        *dst = vec3_from_rapier(src.pos);
                    }
                }
                clear_error();
                n as u32
            }
            _ => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_collect_particles: unknown source (0=fluid, 1=granular)",
                );
                0
            }
        }
    })
}

/// 沿视线穿过粒子云的单次散射累积亮度（直接读世界里的粒子）。
///
/// 与 [`mps_formula::tyndall_particles::inscatter_through_particles`] 同一个
/// 物理模型，区别是粒子由 `(source, id)` 从世界里取，而不是调用方传入。
///
/// # Returns
/// `Bool::TRUE` 成功并把亮度写进 `out`；失败不动 `out`。
///
/// # Safety
/// `world` 必须有效；`out` 必须可写。
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn tyndall_inscatter_from_world(
    world: *const WorldHandle,
    source: u32,
    id: u32,
    origin: Vec3,
    direction: Vec3,
    length: f64,
    smoothing_radius: f64,
    sigma_sca: f64,
    sigma_ext: f64,
    phase: f64,
    light_intensity: f64,
    steps: u32,
    out: *mut f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(
                ERR_NULL_POINTER,
                "tyndall_inscatter_from_world: world is null",
            );
            return Bool::FALSE;
        };
        if out.is_null() {
            set_error(
                ERR_NULL_POINTER,
                "tyndall_inscatter_from_world: out is null",
            );
            return Bool::FALSE;
        }
        if !vec3_finite(origin) || !vec3_finite(direction) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "tyndall_inscatter_from_world: non-finite origin/direction",
            );
            return Bool::FALSE;
        }

        // 取粒子位置到本地缓冲。渲染每帧调用一次，粒子数一般 10³~10⁵，
        // 这个拷贝是必要的——公式层不依赖 rapier 类型。
        let idx = id as usize;
        let positions: Vec<[f64; 3]> = match source {
            TYNDALL_SOURCE_FLUID => {
                if idx >= world.inner.fluids.len() {
                    set_error(
                        ERR_NOT_FOUND,
                        "tyndall_inscatter_from_world: unknown fluid id",
                    );
                    return Bool::FALSE;
                }
                world.inner.fluids[idx]
                    .particles
                    .iter()
                    .map(|p| [p.pos.x, p.pos.y, p.pos.z])
                    .collect()
            }
            TYNDALL_SOURCE_GRANULAR => {
                if idx >= world.inner.granular_bodies.len() {
                    set_error(
                        ERR_NOT_FOUND,
                        "tyndall_inscatter_from_world: unknown granular id",
                    );
                    return Bool::FALSE;
                }
                world.inner.granular_bodies[idx]
                    .particles
                    .iter()
                    .map(|p| [p.pos.x, p.pos.y, p.pos.z])
                    .collect()
            }
            _ => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_inscatter_from_world: unknown source (0=fluid, 1=granular)",
                );
                return Bool::FALSE;
            }
        };

        // 单颗粒质量：流体取首粒子质量（SPH 里通常全部相同），颗粒取首粒子。
        // 若粒子质量不一致，通道的语义是"用第一个粒子的代表值"——这在求解器里
        // 是常见前提，注释在此以免误解。
        let particle_mass = match source {
            TYNDALL_SOURCE_FLUID => world.inner.fluids[idx]
                .particles
                .first()
                .map(|p| p.mass)
                .unwrap_or(1.0),
            _ => world.inner.granular_bodies[idx]
                .particles
                .first()
                .map(|p| p.mass)
                .unwrap_or(1.0),
        };

        let optics = mps_formula::tyndall_particles::ParticleOptics {
            sigma_sca,
            sigma_ext,
            mass: particle_mass,
        };
        let value = mps_formula::tyndall_particles::inscatter_through_particles(
            &positions,
            optics,
            [origin.x, origin.y, origin.z],
            [direction.x, direction.y, direction.z],
            length,
            smoothing_radius,
            phase,
            light_intensity,
            steps,
        );
        match value {
            Some(v) => {
                unsafe {
                    *out = v;
                }
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_inscatter_from_world: invalid arguments (check direction is \
                     normalised, sigma_ext >= sigma_sca, steps >= 1)",
                );
                Bool::FALSE
            }
        }
    })
}

/// 在查询点估计粒子云的**数密度**（个/m³），直接读世界。
///
/// 渲染层可以用它做自适应采样密度：在数密度高的区域多取几个采样点。
///
/// # Safety
/// `world` 必须有效；`out` 必须可写。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_density_from_world(
    world: *const WorldHandle,
    source: u32,
    id: u32,
    query: Vec3,
    smoothing_radius: f64,
    out: *mut f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(
                ERR_NULL_POINTER,
                "tyndall_density_from_world: world is null",
            );
            return Bool::FALSE;
        };
        if out.is_null() {
            set_error(ERR_NULL_POINTER, "tyndall_density_from_world: out is null");
            return Bool::FALSE;
        }
        if !vec3_finite(query) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "tyndall_density_from_world: non-finite query",
            );
            return Bool::FALSE;
        }

        let idx = id as usize;
        let positions: Vec<[f64; 3]> = match source {
            TYNDALL_SOURCE_FLUID => {
                if idx >= world.inner.fluids.len() {
                    set_error(
                        ERR_NOT_FOUND,
                        "tyndall_density_from_world: unknown fluid id",
                    );
                    return Bool::FALSE;
                }
                world.inner.fluids[idx]
                    .particles
                    .iter()
                    .map(|p| [p.pos.x, p.pos.y, p.pos.z])
                    .collect()
            }
            TYNDALL_SOURCE_GRANULAR => {
                if idx >= world.inner.granular_bodies.len() {
                    set_error(
                        ERR_NOT_FOUND,
                        "tyndall_density_from_world: unknown granular id",
                    );
                    return Bool::FALSE;
                }
                world.inner.granular_bodies[idx]
                    .particles
                    .iter()
                    .map(|p| [p.pos.x, p.pos.y, p.pos.z])
                    .collect()
            }
            _ => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_density_from_world: unknown source (0=fluid, 1=granular)",
                );
                return Bool::FALSE;
            }
        };

        match mps_formula::tyndall_particles::sample_number_density(
            &positions,
            [query.x, query.y, query.z],
            smoothing_radius,
        ) {
            Some(v) => {
                unsafe {
                    *out = v;
                }
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_density_from_world: smoothing radius must be > 0",
                );
                Bool::FALSE
            }
        }
    })
}

/// 粒子云的包围盒（渲染层做视锥剔除）。
///
/// # Safety
/// `world` 必须有效；`out_min` / `out_max` 必须可写。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_bounds_from_world(
    world: *const WorldHandle,
    source: u32,
    id: u32,
    out_min: *mut Vec3,
    out_max: *mut Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "tyndall_bounds_from_world: world is null");
            return Bool::FALSE;
        };
        if out_min.is_null() || out_max.is_null() {
            set_error(ERR_NULL_POINTER, "tyndall_bounds_from_world: out is null");
            return Bool::FALSE;
        }
        let idx = id as usize;
        let positions: Vec<[f64; 3]> = match source {
            TYNDALL_SOURCE_FLUID => {
                if idx >= world.inner.fluids.len() {
                    set_error(ERR_NOT_FOUND, "tyndall_bounds_from_world: unknown fluid id");
                    return Bool::FALSE;
                }
                world.inner.fluids[idx]
                    .particles
                    .iter()
                    .map(|p| [p.pos.x, p.pos.y, p.pos.z])
                    .collect()
            }
            TYNDALL_SOURCE_GRANULAR => {
                if idx >= world.inner.granular_bodies.len() {
                    set_error(
                        ERR_NOT_FOUND,
                        "tyndall_bounds_from_world: unknown granular id",
                    );
                    return Bool::FALSE;
                }
                world.inner.granular_bodies[idx]
                    .particles
                    .iter()
                    .map(|p| [p.pos.x, p.pos.y, p.pos.z])
                    .collect()
            }
            _ => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_bounds_from_world: unknown source (0=fluid, 1=granular)",
                );
                return Bool::FALSE;
            }
        };

        match mps_formula::tyndall_particles::particle_cloud_bounds(&positions) {
            Some((min, max)) => {
                unsafe {
                    *out_min = Vec3 {
                        x: min[0],
                        y: min[1],
                        z: min[2],
                    };
                    *out_max = Vec3 {
                        x: max[0],
                        y: max[1],
                        z: max[2],
                    };
                }
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_bounds_from_world: particle cloud is empty",
                );
                Bool::FALSE
            }
        }
    })
}
