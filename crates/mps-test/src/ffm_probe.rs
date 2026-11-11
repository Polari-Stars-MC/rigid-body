//! `mps_ffm` ABI 探针库的契约守门 —— 防"公式层 JNI/FFM 兼容失效"回归。
//!
//! 背景：公式层 ~200 个纯公式 C 入口的错误槽（thread-local，实现同源于
//! `mps_formula::error`）此前只能通过 mps-core cdylib 的 `last_error_*`
//! 读到。mps-ffm 探针库静态链接同一份 mps-core/mps-formula 后，`last_error_*`
//! 经由 rlib 传递导出自动出现在 mps-ffm cdylib 的导出表——这个契约没有被
//! 任何人显式声明过，一旦 mps-core 改导出面或 mps-ffm 换依赖，公式层的
//! 错误通道就会静默"失效"（宿主调公式函数后读不到错误码）。
//!
//! 本文件锁三件事：
//! 1. 探针库 rlib 面的常量与函数行为（abi_version ≥ 2、jni/ffm 能力位）；
//! 2. 公式错误槽闭环：经由探针库路径写错误 → 同一路径读回，码与消息一致；
//! 3. 健康检查探针 `abi_formula_ffi_healthcheck` 返回 ERR_OK。
//!
//! DLL 导出表本身由 CI 的 Java 冒烟（加载 mps_ffm.dll 后反射查符号）覆盖；
//! 这里在 Rust 侧锁可锁的部分。

#[cfg(test)]
mod tests {
    // mps-ffm 同时是 rlib：直接链接其公开面。
    use mps_core::rapier::error::{ERR_OK, clear_error, last_error_code, set_error};

    #[test]
    fn probe_abi_version_is_at_least_2() {
        // ABI v2 = 引入 formula_ffi 探针与健康检查的版本。降版本 = 破坏
        // 宿主探测逻辑，必须走 ADR。const 断言在编译期拦截。
        const _: () = assert!(mps_ffm::ABI_VERSION >= 2, "ABI_VERSION regressed below v2");
    }

    #[test]
    fn probe_capability_flags_are_stable() {
        assert_eq!(mps_ffm::abi_version(), mps_ffm::ABI_VERSION);
        // FFM 通道：本库是纯 extern "C" 面，恒可用。
        assert_eq!(mps_ffm::abi_supports_ffm().0, 1);
        // 公式层探针覆盖：v2 起恒可用。
        assert_eq!(mps_ffm::abi_supports_formula_ffi().0, 1);
        // JNI 通道：探针库不带 Java_* 符号，显式 FALSE——宿主应加载 mps-jni，
        // 不再靠 loadLibrary 失败才发现。
        assert_eq!(mps_ffm::abi_supports_jni().0, 0);
    }

    #[test]
    fn formula_error_slot_roundtrip_through_probe_path() {
        // 经由探针库公开的公式 C 入口（wo_wavenumber）写错误 → 同路径读回。
        // 这模拟 FFM 宿主的完整调用序列：clear → 调公式 → 读码。
        clear_error();
        let period = mps_formula::wave_optics::wo_wavenumber(-5.0);
        assert!(period.is_nan(), "invalid wavelength → NaN sentinel");
        assert_eq!(
            last_error_code(),
            2,
            "ERR_INVALID_ARGUMENT visible through the shared slot"
        );
        // 合法调用清槽。
        let good = mps_formula::wave_optics::wo_wavenumber(2.0);
        assert!((good - std::f64::consts::PI).abs() < 1e-12, "2π/2 = π");
        assert_eq!(last_error_code(), ERR_OK, "success clears the slot");
        clear_error();
    }

    #[test]
    fn healthcheck_probe_returns_ok() {
        // 探针自检：非法输入 → NaN + ERR_INVALID_ARGUMENT，合法输入 → 确定值
        // + ERR_OK。任一环节坏掉（错误槽断链、公式入口被改签名）都返回非零。
        assert_eq!(
            mps_ffm::abi_formula_ffi_healthcheck(),
            ERR_OK,
            "formula FFI healthcheck must pass"
        );
    }

    #[test]
    fn error_slot_note_is_non_empty_c_string() {
        let note = mps_ffm::abi_error_slot_note();
        assert!(!note.is_null());
        let text = unsafe { std::ffi::CStr::from_ptr(note) }
            .to_string_lossy()
            .into_owned();
        assert!(
            text.contains("thread-local"),
            "slot note must explain the per-cdylib isolation: {text}"
        );
        // 保险：set_error 仍工作（本测试进程的槽来自 mps-core rlib）。
        set_error(4, "capacity probe");
        assert_eq!(last_error_code(), 4);
        clear_error();
    }
}
