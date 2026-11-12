//! ffi_guard 覆盖守门（AGENTS.md：每个 `extern "C"` 入口用 `ffi_guard`）。
//!
//! 静态扫描 `mps-core/src/rapier/**/*.rs` 与 `mps-cosmos/src/ffi.rs`：
//! 找出所有 `pub ... extern "C" fn` 定义，并要求函数体里出现 panic 屏障
//! ——`ffi_guard`（完整入口）或 `ffi_scalar`（标量出口变体，`75671f7`
//! 引入的 catch_unwind 包装，语义等价）。

#[cfg(test)]
mod tests {
    use std::fs;
    use std::path::{Path, PathBuf};

    fn collect_rs_files(dir: &Path) -> Vec<PathBuf> {
        let mut out = Vec::new();
        let Ok(entries) = fs::read_dir(dir) else {
            return out;
        };
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_dir() {
                out.extend(collect_rs_files(&p));
            } else if p.extension().and_then(|s| s.to_str()) == Some("rs") {
                out.push(p);
            }
        }
        out
    }

    fn sibling_crate_dir(subdir: &str) -> PathBuf {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        manifest.parent().expect("crates/ parent").join(subdir)
    }

    /// 扫描一个文件的 `pub extern "C"` 定义，返回 (函数名, 函数体是否含
    /// panic 屏障（ffi_guard/ffi_scalar）, 行号)。函数体从签名行扫到下一
    /// 个顶层 `}`（近似——对本守门足够，因为 FFI 函数体通常很短）。
    fn scan_extern_c(path: &Path) -> Vec<(String, bool, usize)> {
        let Ok(content) = fs::read_to_string(path) else {
            return Vec::new();
        };
        let lines: Vec<&str> = content.lines().collect();
        let mut results = Vec::new();
        let mut i = 0usize;
        while i < lines.len() {
            let line = lines[i];
            let is_def = line.contains("extern \"C\"")
                && line.contains("fn ")
                && !line.trim_start().starts_with("//");
            if is_def {
                let name = line
                    .split("fn ")
                    .nth(1)
                    .and_then(|rest| {
                        rest.split(|c: char| !(c.is_alphanumeric() || c == '_'))
                            .next()
                    })
                    .unwrap_or("<unknown>")
                    .to_string();
                // 函数体扫描：签名行（含行尾 `{` 或后续行）把 depth 从 0
                // 抬到 ≥1，扫到 depth 回 0 为止（闭 `}` 之后的下一行是
                // 顶层——精确处理签名跨行 + `#[unsafe(no_mangle)]` 前缀）。
                let mut body = String::new();
                let mut j = i;
                let mut depth = 0usize;
                let mut seen_open = false;
                while j < lines.len() {
                    let l = lines[j];
                    depth += l.matches('{').count();
                    depth -= l.matches('}').count();
                    if depth > 0 {
                        seen_open = true;
                    }
                    body.push_str(l);
                    body.push('\n');
                    if seen_open && depth == 0 {
                        break;
                    }
                    j += 1;
                }
                let guarded = body.contains("ffi_guard") || body.contains("ffi_scalar");
                results.push((name, guarded, i + 1));
                i = j.max(i + 1);
            } else {
                i += 1;
            }
        }
        results
    }

    /// 刻意不带 ffi_guard/ffi_scalar 的白名单。每条必须有理由注释——审查
    /// 者看到新白名单条目时应当质疑。
    const ALLOWLIST: &[&str] = &[
        // 纯常量构造：返回按值 repr(C) 结构体，无指针、无分配、无 panic 路径。
        // 套 ffi_guard 反而引入无意义的返回值语义问题（guard 需要哨兵返回值）。
        "world_cross_validate_default_config",
        // 纯转发垫片：直接 tail-call 到已套 ffi_guard 的完整入口，二次包裹
        // 只会让错误槽语义复杂化（内层已写好错误码）。
        "fracture_mesh_body_set_trigger_stress",
        // 以下三个同样 tail-call 到内含 ffi_guard 的 set_wheel_field 垫片。
        "vehicle_controller_set_engine_force",
        "vehicle_controller_set_brake",
        "vehicle_controller_set_steering",
        // tail-call world_set_region_active（内含 ffi_guard）。
        "world_wake_region",
        // mps-ffm 探针 crate 的导出在 mps-ffm/src/lib.rs，不在本扫描范围。
    ];

    #[test]
    fn every_core_extern_c_uses_ffi_guard() {
        let dir = sibling_crate_dir("mps-core/src/rapier");
        let mut offenders = Vec::new();
        let mut checked = 0usize;
        for path in collect_rs_files(&dir) {
            for (name, guarded, line) in scan_extern_c(&path) {
                checked += 1;
                if !guarded && !ALLOWLIST.contains(&name.as_str()) {
                    let rel = path.file_name().and_then(|s| s.to_str()).unwrap_or("?");
                    offenders.push(format!("{rel}:{line} {name}"));
                }
            }
        }
        assert!(
            checked > 800,
            "scan sanity: expected 800+ extern \"C\" defs in mps-core, got {checked} \
             (scan broke? fix collect path)"
        );
        assert!(
            offenders.is_empty(),
            "extern \"C\" fns without ffi_guard (AGENTS.md hard rule):\n  {}",
            offenders.join("\n  ")
        );
    }

    #[test]
    fn every_cosmos_extern_c_uses_ffi_guard() {
        let dir = sibling_crate_dir("mps-cosmos/src");
        let mut offenders = Vec::new();
        let mut checked = 0usize;
        for path in collect_rs_files(&dir) {
            for (name, guarded, line) in scan_extern_c(&path) {
                checked += 1;
                if !guarded && !ALLOWLIST.contains(&name.as_str()) {
                    let rel = path.file_name().and_then(|s| s.to_str()).unwrap_or("?");
                    offenders.push(format!("{rel}:{line} {name}"));
                }
            }
        }
        assert!(
            checked > 30,
            "scan sanity: expected 30+ extern \"C\" defs in mps-cosmos, got {checked}"
        );
        assert!(
            offenders.is_empty(),
            "extern \"C\" fns without ffi_guard (AGENTS.md hard rule):\n  {}",
            offenders.join("\n  ")
        );
    }
}
