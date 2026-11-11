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
