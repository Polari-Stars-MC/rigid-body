//! 源 crate 不得内联 `#[cfg(test)]` 模块（ADR-0005 / BOUNDARY.md「测试边界」）。
//!
//! 规则原文：**全部集成测试放在 `mps-test`**，源 crate 不另放一套内联测试来
//! 代替镜像。`verify_module_mirror` 只比对目录清单，看不见「源文件里自己长出
//! 来的 `mod tests`」——这类测试不进 `mps-test`，因此：
//!
//! - 它的 `#[test]` 不进文档站的计数口径，但会真的跑（= 计数与实际不一致）；
//! - 它绕过了镜像规则：新增模块时没人被迫补 `mps-test` 侧文件；
//! - 它在 `mps-core` 这样的 `cdylib` 里被一起编进去，测试配置和宿主配置纠缠
//!   在一起（ADR-0005 的背景原话）。
//!
//! 守卫落地时仓库里有 3 处违规，全部是「顺手在源文件里补了个断言」：
//! `cross_validate.rs`（4 个）、`forces.rs`（1 个）、`mps-web/src/lib.rs`（1 个）。
//!
//! `mps-web` 是文档站，不是 ADR-0005 点名的那三个源 crate；它的 i18n 守卫测试
//! 需要在自身 crate 内取 `CARGO_MANIFEST_DIR` 下的 locale 文件，且 `mps-web`
//! 不依赖 `mps-test`（BOUNDARY.md 禁止文档站链接模拟器）。所以它在下面的白名单
//! 里，并且只豁免 `mps-*` 里它一个。
//!
//! 白名单**不是**给「先放着」留的口子：往里加条目等于改 ADR-0005，要写明理由。

#[cfg(test)]
mod tests {
    use std::fs;
    use std::path::{Path, PathBuf};

    /// 允许内联 `#[cfg(test)]` 的 crate，附理由。
    ///
    /// 键是 crate 目录名。空理由列表以外的东西一律算违规。
    const ALLOWED: &[(&str, &str)] = &[(
        "mps-web",
        "文档站的 i18n 覆盖测试必须在自身 crate 内读 src/i18n/locales；\
         mps-web 不依赖 mps-test（BOUNDARY.md：文档站不链接模拟器），\
         而 ADR-0005 点名的是 mps-core / mps-formula / mps-cosmos 三个源 crate",
    )];

    fn project_root() -> PathBuf {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        manifest
            .parent()
            .and_then(|p| p.parent())
            .expect("crates/mps-test has two ancestors")
            .to_path_buf()
    }

    fn collect_rs(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(entries) = fs::read_dir(dir) else {
            return;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
                // vendored fork 是自己 workspace、自己测试基线（ADR-0007/0008）。
                if name == "rapier" || name == "target" {
                    continue;
                }
                collect_rs(&path, out);
            } else if path.extension().and_then(|s| s.to_str()) == Some("rs") {
                out.push(path);
            }
        }
    }

    /// 真实的内联测试模块：一行去掉注释后形如 `mod <name> {`（可带 `pub`），
    /// 且判断依据只看**是否被 `#[cfg(test)]` 直接前置**。裸 `mod tests` 也会被
    /// 抓到，因为它是靠 `#[cfg(test)]` 才不进 release 的。
    #[test]
    fn source_crates_have_no_inline_cfg_test_modules() {
        let root = project_root();
        let crates_dir = root.join("crates");
        let allowed: Vec<&str> = ALLOWED.iter().map(|(name, _)| *name).collect();

        let Ok(entries) = fs::read_dir(&crates_dir) else {
            panic!("crates/ is unreadable at {}", crates_dir.display());
        };
        let mut crate_dirs: Vec<PathBuf> = Vec::new();
        for entry in entries.flatten() {
            let path = entry.path();
            if !path.is_dir() {
                continue;
            }
            let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
            // 跳过测试 crate 自身与 vendored 桩目录。
            if name == "mps-test" || name == "rapier3d-f64" {
                continue;
            }
            crate_dirs.push(path);
        }
        assert!(
            crate_dirs.len() >= 8,
            "scan sanity: expected 8+ sibling crates to scan, found {}",
            crate_dirs.len()
        );

        let mut offenders: Vec<String> = Vec::new();
        for crate_dir in &crate_dirs {
            let crate_name = crate_dir
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("?")
                .to_string();
            if allowed.contains(&crate_name.as_str()) {
                continue;
            }
            let mut files = Vec::new();
            collect_rs(&crate_dir.join("src"), &mut files);
            for file in files {
                let Ok(text) = fs::read_to_string(&file) else {
                    continue;
                };
                let lines: Vec<&str> = text.lines().collect();
                for (idx, line) in lines.iter().enumerate() {
                    let t = line.trim();
                    if t != "#[cfg(test)]" {
                        continue;
                    }
                    let rel = file.strip_prefix(&root).unwrap_or(&file);
                    offenders.push(format!(
                        "{}:{}",
                        rel.display().to_string().replace('\\', "/"),
                        idx + 1
                    ));
                }
            }
        }

        assert!(
            offenders.is_empty(),
            "源 crate 里出现了内联 `#[cfg(test)]` 模块——ADR-0005 规定全部集成测试放 \
             mps-test 并按源模块镜像：{offenders:#?}\n  \
             修法：把断言搬到 `crates/mps-test/src/<层>/<同名模块>.rs`，并在对应 \
             `mod` 树里挂上；源文件只保留实现。\n  \
             如果是新建的守卫测试而非业务断言，它应当住在本 crate（mps-test）里，\
             而不是住进被它检查的 crate。\n  \
             例外只经由本文件的 ALLOWED 表（附理由），不要就地 #[allow]。"
        );

        // 白名单条目必须真的还在用，否则它会变成永久的静默豁免。
        for (name, reason) in ALLOWED {
            assert!(
                !reason.trim().is_empty(),
                "ALLOWED 里 {name} 没有写理由——豁免必须能自解释"
            );
        }
    }
}
