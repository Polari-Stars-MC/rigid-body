//! ABI 探针与兼容层 cdylib（`mps_ffm_probe` 或被 Java 宿主以任意名加载）。
//!
//! 职责：
//! 1. **ABI 版本探针**：`abi_version` / `abi_supports_ffm` / `abi_supports_jni`
//!    —— JVM 侧（FFM `Linker` 或 JNI `System.loadLibrary`）加载后先调这组
//!    函数确认符号面一致，防止"每次都失效"式的静默 ABI 漂移。
//! 2. **公式层错误通道**：`last_error_*` / `error_code_name` 从本 cdylib
//!    也导出——mps-formula 的 ~200 个纯公式 C 入口（`chaos_*`、`em_*`、
//!    `wo_*`、`rel_*`…）是 JVM 可以**不经 mps-core 的世界 API** 直接 FFM
//!    链接的；此前这些入口的错误槽只能通过 mps-core cdylib 的
//!    `last_error_*` 读，宿主只加载探针库 + 公式函数时拿不到错误详情。
//!    本库静态链接了同一份 mps-formula（槽实现同源），导出同一组符号后
//!    宿主从本库链接公式函数时错误读写在同一槽上闭环。
//!
//! **槽配对语义**：thread-local 槽按 **cdylib 实例**隔离——一个 JVM 里同时
//! 加载 `mps_rigid_body`（mps-jni）和本库时，两边各有一份 mps-formula 静态
//! 代码，槽不共享。从哪个库链接公式函数，就从哪个库读 `last_error_*`。
//! `abi_error_slot_note` 导出这条规则的机器可读文本。

use mps_core::rapier::error as er;
use mps_core::rapier::ffi::Bool;

pub const ABI_VERSION: u32 = 2;

#[unsafe(no_mangle)]
pub extern "C" fn abi_version() -> u32 {
    ABI_VERSION
}

/// FFM 通道可用性：本库是标准 `extern "C"` 面，FFM `Linker::downcall` 直接可调。
#[unsafe(no_mangle)]
pub extern "C" fn abi_supports_ffm() -> Bool {
    Bool::TRUE
}

/// JNI 通道可用性：JNI 符号面由 `mps_rigid_body`（mps-jni）承担；本库不带
/// `Java_*` 符号，JNI 宿主应加载 mps-jni。显式返回 FALSE 让探针不再靠
/// "加载失败" 才发现。
#[unsafe(no_mangle)]
pub extern "C" fn abi_supports_jni() -> Bool {
    Bool::FALSE
}

/// 公式层 C ABI 是否已进入本库的探针覆盖：宿主可以据此判断"公式错误槽在
/// 本库可读"。ABI v2 起恒为 TRUE。
#[unsafe(no_mangle)]
pub extern "C" fn abi_supports_formula_ffi() -> Bool {
    Bool::TRUE
}

/// 错误槽配对语义的静态说明（供探针/日志打印）：同 JVM 内 mps-jni 与 mps-ffm
/// 两个 cdylib 的 thread-local 错误槽互不共享，读错误必须从写入它的同一个
/// 库读。返回指针 'static，不可写、不可释放。
///
/// # Safety
/// 无参数；返回的指针指向 'static NUL 结尾字符串，只读、不释放。
#[unsafe(no_mangle)]
pub extern "C" fn abi_error_slot_note() -> *const std::os::raw::c_char {
    c"errors are thread-local per cdylib; read last_error_* from the same library that reported the error".as_ptr()
}

// ---------------------------------------------------------------------------
// 公式层错误通道：`last_error_code` / `last_error_message` / `last_error_clear`
// / `error_code_name` 不在此重复定义——mps-core rlib 里它们已带 `#[no_mangle]`，
// 链接进本 cdylib 后自动出现在导出表。本库静态链接的 mps-formula 与 mps-core
// 是同一份编译单元，公式入口写的槽就是这四个访问器读的槽：闭环成立。
// （若未来 mps-core 不再传递导出，可在此以 #[no_mangle] 转发补齐——注意不要
// 与 rlib 符号重复定义，否则 MSVC LNK2005。）
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 公式层冒烟探针：宿主加载后调一次，确认公式 C ABI 在本库真的可调、错误槽
// 真的可读（非法输入 → ERR_INVALID_ARGUMENT → last_error_code 返回 2）。
// 返回 ERR_OK 表示整链路健康；否则返回非零码，Java 侧可判定"公式层失效"。
// ---------------------------------------------------------------------------

/// # Safety
/// 无参数；无副作用（探针内部 clear 错误槽）。
#[unsafe(no_mangle)]
pub extern "C" fn abi_formula_ffi_healthcheck() -> u32 {
    use mps_formula::error::{ERR_INVALID_ARGUMENT, ERR_OK};
    er::last_error_clear();
    // 走一个真实的公式 C 入口（wo_wavenumber）：非法输入 → NaN 哨兵 + 槽写
    // ERR_INVALID_ARGUMENT；合法输入 → 确定值 + 槽清回 OK。两段都过才算健康。
    let bad = mps_formula::wave_optics::wo_wavenumber(-1.0);
    let bad_code = er::last_error_code();
    if !(bad.is_nan() && bad_code == ERR_INVALID_ARGUMENT) {
        return if bad_code == ERR_OK {
            ERR_INTERNAL_CODE
        } else {
            bad_code
        };
    }
    let good = mps_formula::wave_optics::wo_wavenumber(1.0);
    let good_code = er::last_error_code();
    // 2π/1m = 6.283185307…（f64 精确到两位尾差以内）。
    if (good - std::f64::consts::TAU).abs() > 1e-12 || good_code != ERR_OK {
        return if good_code == ERR_OK {
            ERR_INTERNAL_CODE
        } else {
            good_code
        };
    }
    er::last_error_clear();
    ERR_OK
}

/// `ERR_INTERNAL` 的本地副本（探针自身对域的"非预期"分类）。
const ERR_INTERNAL_CODE: u32 = 6;

// ---------------------------------------------------------------------------
// 端到端冒烟探针（v3 加的，ABI_VERSION 不动——只加符号不改旧符号语义）：
// 走真实 mps-core 世界 API 的最小往返，验证"探针库加载后世界面也能用"。
// 世界面主通道仍是 mps-jni；本探针是加载诊断的兜底（JVM 侧如果怀疑
// mps_rigid_body 坏了，先调这两个，能区分"库没链接好"和"JNI 层坏了"）。
// ---------------------------------------------------------------------------

/// 录制往返探针：建世界 → 开录 → 捕获 2 帧 → 停录 → 两段式导出 → 校验
/// blob 头部 [2,0] 与首个帧 magic 是 MPSDR01。返回 ERR_OK=0 表示健康，
/// 否则返回错误码（ERR_INTERNAL=6 表示"跑了但结果不符合契约"）。
///
/// 探针自建自毁世界，不留全局状态；错误槽最后清回 OK。
///
/// # Safety
/// 无参数；内部调用 world_create/world_destroy 等安全包装，无裸指针越界。
#[unsafe(no_mangle)]
pub extern "C" fn abi_recording_roundtrip_probe() -> u32 {
    use mps_core::rapier::debug_recording::{
        debug_recording_capture, debug_recording_export, debug_recording_start,
        debug_recording_stop,
    };
    use mps_core::rapier::ffi::Vec3;
    use mps_formula::error::ERR_OK;
    er::last_error_clear();
    // 直接调 rlib 里的 FFI 函数（同一编译单元，符号不经过动态导出表）。
    let world = mps_core::rapier::world::world_create(Vec3 {
        x: 0.0,
        y: -9.81,
        z: 0.0,
    });
    if world.is_null() {
        return ERR_INTERNAL_CODE;
    }
    let fail = ERR_INTERNAL_CODE;
    // 开始录制 → 两次捕获 → 停止 → 导出（两段式）。
    let started = debug_recording_start(world, 16);
    debug_recording_capture(world);
    debug_recording_capture(world);
    let stopped = debug_recording_stop(world);
    let size = debug_recording_export(world, std::ptr::null_mut(), 0);
    if started == 0 || stopped != 2 || size < 2 {
        mps_core::rapier::world::world_destroy(world);
        return fail;
    }
    let mut blob = vec![0.0f64; size as usize];
    let written = debug_recording_export(world, blob.as_mut_ptr(), size);
    mps_core::rapier::world::world_destroy(world);
    if written != size {
        return fail;
    }
    // 头部：frame_count == 2, dropped == 0；首帧 magic == MPSDR01。
    let frames = blob[0] as u32;
    let dropped = blob[1] as u32;
    let magic = f64::to_bits(blob[2]);
    // MPSDR01 = "MPSDR01" 的 f64 位型（与 debug_recording.rs 的常量逐位一致）。
    const MAGIC_V1: u64 = 0x4d50_5344_5230_3131;
    if frames != 2 || dropped != 0 || magic != MAGIC_V1 {
        return fail;
    }
    er::last_error_clear();
    ERR_OK
}

/// voxel-bake 确定性探针：同一份 4×2×4 体素网格 bake 两次，逐槽比较
/// 输出 box 列表完全一致（MC 端缓存的前提）。返回 ERR_OK 表示健康。
///
/// # Safety
/// 无参数；体素网格在探针栈上，全程无并发。
#[unsafe(no_mangle)]
pub extern "C" fn abi_voxel_bake_determinism_probe() -> u32 {
    use mps_core::rapier::ffi::Vec3;
    use mps_core::rapier::voxel::voxel_bake_boxes;
    use mps_formula::error::ERR_OK;
    er::last_error_clear();
    // 4×2×4 网格：左半全实心（应合并成一个大 box），右半交错（多 box）。
    let (sx, sy, sz) = (4u32, 2u32, 4u32);
    let mut voxels = vec![0u8; (sx * sy * sz) as usize];
    for x in 0..2usize {
        for y in 0..sy as usize {
            for z in 0..sz as usize {
                voxels[(y * sz as usize + z) * sx as usize + x] = 1;
            }
        }
    }
    voxels[3] = 1; // 右上角单个体素 → 第二个 box。
    let bake = |out: &mut Vec<f64>| -> u32 {
        voxel_bake_boxes(
            voxels.as_ptr(),
            sx,
            sy,
            sz,
            1.0,
            1.0,
            1.0,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            out.as_mut_ptr(),
            (out.len() / 6).max(1) as u32,
        )
    };
    // 两段式：先查 count，再分配后 bake。
    let count = bake(&mut Vec::new());
    if count == 0 {
        return ERR_INTERNAL_CODE;
    }
    let mut a = vec![0.0f64; count as usize * 6];
    let n_a = bake(&mut a);
    let mut b = vec![0.0f64; count as usize * 6];
    let n_b = bake(&mut b);
    if n_a != count || n_b != count || a != b {
        return ERR_INTERNAL_CODE;
    }
    er::last_error_clear();
    ERR_OK
}
