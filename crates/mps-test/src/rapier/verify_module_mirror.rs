//! Module-mirror CI guard for mps-test ↔ mps-core/mps-cosmos/mps-formula.
//!
//! DESIGN.md mandates that whenever a `mpd-core::rapier::*` submodule is added
//! or renamed, the corresponding `mpd-test/src/rapier/<name>.rs` file must
//! be kept in lockstep; likewise for cosmos. Until now this rule has been a
//! manual convention only — a developer who deleted `core/rapier/foo.rs` but
//! left `mod foo;` dangling got an inscrutable *compile* failure, and a
//! developer who added a new core submodule without a test file got *silent
//! missed coverage*. This module turns that DSL convention into a CI-visible
//! assertion, per OPTIMIZATION.md §8.
//!
//! The rule it enforces, computed by directory listing only (no parsing):
//!
//! ```text
//! let test_rapier  = files(crate "mpd-test/src/rapier/")
//! let core_rapier  = files(crate "mpd-core/src/rapier/")
//! let formula_src  = files(crate "mpd-formula/src/")
//! let expected     = (core_rapier |union| formula_src)
//!                     - {mod.rs, lib.rs}     (structural, no test file)
//! let extra_ok     = {ffi.rs, arena_compat.rs, ...}  // test-only adapters
//! assert!(test_rapier == expected |union| extra_ok)
//! ```
//!
//! For cosmos the rule is simpler (1:1 mirror of `crates/mpd-cosmos/src/`
//! minus `lib.rs`/`ffi.rs`):
//!
//! ```text
//! let test_cosmos = files("mpd-test/src/cosmos/")
//! let cosmos_src  = files("mpd-cosmos/src/")
//! let extra_ok    = {}  // no test-only adapters expected
//! assert!(test_cosmos == cosmos_src - {lib.rs, ffi.rs})
//! ```
//!
//! # §N1 增强：形态度校验
//!
//! 上一轮 §3 把 `core/rapier/spaceflight.rs` 拆成 `spaceflight/` 含 8 个子
//! 文件；formula 那侧恰好保留了同名 `spaceflight.rs` 纯公式文件，导致
//! 上面的 union 集合仍包含 `spaceflight.rs`、test 那侧 1746 行单文件 mirror
//! 被"撞名"通过 — 漏报。`list_entries` 现既列文件名也列**目录形态标记**
//! (`(name, is_dir)`),`mirror_shape_matches` 额外校验：当 core 把某个名字
//! 拆成 `<name>/` 目录时,mirror 那侧允许相应文件以以下两种形式出现:
//!   (a) `<name>/` 目录（对应 §N1 方案 A 的同步拆分，推荐）;
//!   (b) `<name>.rs` 文件（mirror 不拆，作为过渡，但 CI 提醒）。
//! 然而**反过来禁止**：mirror 是目录而 core 仍是单文件 —— 说明 mirror
//! 漂在 core 前面，是 mirror 的孤立拆分。

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::fs;
    use std::path::PathBuf;

    /// A directory entry: `(name, is_dir)`. The name is the basename of a
    /// `.rs` file (with `.rs` stripped for module-name comparison) or a
    /// subdirectory name. `mod.rs`/`lib.rs` are NOT included by this helper;
    /// callers handle them separately.
    fn list_entries(dir: PathBuf) -> BTreeMap<String, bool> {
        let mut out = BTreeMap::new();
        let Ok(entries) = fs::read_dir(&dir) else {
            return out;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
                continue;
            };
            // Skip module/crate roots — they are structural and tested elsewhere.
            if name == "mod.rs" || name == "lib.rs" {
                continue;
            }
            if path.is_dir() {
                // Subdirectory → a Rust module declared as `pub mod <name>;`
                // in the parent `mod.rs`. Its content lives in
                // `<name>/mod.rs` (or `<name>/<anything>.rs`).
                out.insert(name.to_string(), true);
            } else if let Some(stripped) = name.strip_suffix(".rs") {
                // `.rs` file → a Rust module declared as `pub mod <name>;`
                // in the parent `mod.rs`. The file is `<name>.rs`.
                out.insert(stripped.to_string(), false);
            }
            // Non-`.rs` files (e.g. README) are ignored.
        }
        out
    }

    /// Resolve a sibling crate directory relative to this crate.
    ///
    /// Uses `CARGO_MANIFEST_DIR` (which points at `crates/mpd-test/` for this
    /// crate) and steps up two levels to reach the workspace root, then down
    /// into the requested sibling crate.
    fn sibling_crate_src(subdir: &str) -> PathBuf {
        let manifest = env!("CARGO_MANIFEST_DIR");
        // crates/mpd-test -> crates/<sibling>
        let manifest = PathBuf::from(manifest);
        let crates_root = manifest
            .parent()
            .expect("crate root has a parent (crates/)");
        crates_root.join(subdir)
    }

    /// rapier submodules: `mpd-test::rapier` mirrors the union of
    /// `mpd-core::rapier::*` and `mpd-formula::*` (the latter providing
    /// pure-formula physics domains whose tests live under rapier/ for
    /// historical reasons). The `STRUCTURAL_OR_EXTRA_ALLOW` files below
    /// are the only legal deviations from the union.
    ///
    /// Additionally (§N1) it enforces morphological alignment: if core has
    /// `<name>/` directory, test must not have `<name>.rs` (one-way drift),
    /// and vice-versa.
    #[test]
    fn rapier_submodules_mirror_core_and_formula() {
        let test_rapier = list_entries(sibling_crate_src("mps-test/src/rapier"));
        let core_rapier = list_entries(sibling_crate_src("mps-core/src/rapier"));
        let formula_src = list_entries(sibling_crate_src("mps-formula/src"));

        // Union of all module names that live on the production side (either
        // core::rapier or formula). list_entries already strips `.rs` so all
        // keys are bare module names; the bool flags the directory shape.
        //
        // The test side is deliberately NOT folded in here: `expected` is the
        // production-side set, so `orphans` (below) can actually fire when a
        // test file has no production counterpart. Seeding it with
        // `test_rapier.keys()` made `orphans` unconditionally empty — the
        // assertion was dead and three test-only files drifted past it.
        let mut expected: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
        expected.extend(core_rapier.keys().cloned());
        expected.extend(formula_src.keys().cloned());

        // Test-side-only files that are NOT mirrored back into core/formula:
        //   ffi                    — mps-test has its own minimal FFI helper
        //   arena_compat           — ABI lock-in test for shared_arena (OPT §10)
        //   error_consistency      — cross-crate ERR_* consistency test (OPT §1)
        //   verify_module_mirror   — THIS FILE (meta CI guard, OPT §8)
        //   version_consistency    — cross-crate ARENA_VERSION↔ABI_VERSION (§N6)
        //   verify_metrics_sync    — metrics drift guard (OPT §N3)
        //   force_queue            — mirrors the nested `rapier::ffi::force_queue`
        //                            module, which is not a top-level name on the
        //                            production side
        //   force_queue_integration— end-to-end arena/force-queue wiring test
        //   performance_matrix     — `#[ignore]`d step benchmark (docs/world-step-benchmarks.md)
        //   sleep_wake             — A3 挖洞唤醒 + 稳定包络（evidence/numeric-drift.md）
        //   record_replay          — D2 位级录制重演守门（evidence/record-replay.md）
        let test_only_extras: std::collections::BTreeSet<String> = [
            "ffi",
            "arena_compat",
            "error_consistency",
            "verify_module_mirror",
            "version_consistency",
            "verify_metrics_sync",
            "force_queue",
            "force_queue_integration",
            "performance_matrix",
            "sleep_wake",
            "record_replay",
        ]
        .into_iter()
        .map(String::from)
        .collect();
        for extra in &test_only_extras {
            expected.remove(extra);
        }

        // Names that are only on test side but not in allow-list → orphan
        let orphans: std::collections::BTreeSet<String> = test_rapier
            .keys()
            .filter(|n| !expected.contains(*n) && !test_only_extras.contains(*n))
            .cloned()
            .collect();
        // Names that are in core/formula but not mirrored to test side
        // (excluding allow-listed extras)
        let missing: std::collections::BTreeSet<String> = expected
            .iter()
            .filter(|n| !test_rapier.contains_key(*n))
            .cloned()
            .collect();

        // §N1 morphology alignment: when core side resolves a module via
        // a subdirectory `<name>/`, test side must at least REGISTER the
        // module (file or dir). We additionally flag **shape mismatch** so
        // drift becomes CI-visible:
        //   (a) core dir + test file  → soft warn (transition state, allowed)
        //   (b) core file + test dir  → hard fail (test drift alone)
        let mut shape_mismatch_hard: Vec<String> = Vec::new();
        let mut shape_mismatch_soft: Vec<String> = Vec::new();
        for (name, test_is_dir) in &test_rapier {
            // Check against the union of core & formula shapes:
            let core_is_dir = core_rapier.get(name).copied();
            let formula_is_dir = formula_src.get(name).copied();
            // Most-authoritative production-side shape:
            // - if EITHER production side is a directory → treat as "should be
            //   directory" (we want mirror to mirror the most granular form)
            // - if neither production side has the entry at all AND it's a
            //   test-only extra we already allow-listed → skip.
            let prod_dir = match (core_is_dir, formula_is_dir) {
                (Some(true), _) | (_, Some(true)) => true,
                (Some(false), _) | (_, Some(false)) => false,
                (None, None) => {
                    // test-only by allow-list; no production side declares it
                    continue;
                }
            };
            if prod_dir && !*test_is_dir {
                // core has spaceflight/ but test has spaceflight.rs
                // — acceptable transition state, but emit a soft hint so
                // future maintainers know to push the split through.
                shape_mismatch_soft.push(name.clone());
            } else if !prod_dir && *test_is_dir {
                // test has spaceflight/ but both core/formula are files →
                // mirror drift ahead of production. Hard fail.
                shape_mismatch_hard.push(name.clone());
            }
        }

        if !orphans.is_empty() || !missing.is_empty() || !shape_mismatch_hard.is_empty() {
            panic!(
                "rapier mirror drift detected:\n  \
                 孤儿 test-modules (test has but neither core nor formula declares \
                 such a submodule): {orphans:?}\n  \
                 缺失 test-modules (core/formula has submodule but test has no mirror): \
                 {missing:?}\n  \
                 形态错位 (test has <name>/ but core only has <name>.rs — test drifted \
                 AHEAD of production, fix by either aligning core or collapsing test): \
                 {shape_mismatch_hard:?}\n\n  \
                 EDITION.md / DESIGN.md rule: when adding/removing/renaming a \
                 `mpd-core::rapier::*` or `mpd-formula::*` submodule, the \
                 corresponding `mpd-test/src/rapier/<name>.rs` (or `<name>/` directory \
                 + `pub mod <name>;` in lib.rs) MUST be kept in sync. \
                 Do not silence this test — fix the drift."
            );
        }
        // Soft hint: print to stderr so a CI log surfaces the transition debt
        // without failing the build, until §N1 方案 A is finished.
        if !shape_mismatch_soft.is_empty() {
            eprintln!(
                "rapier mirror: test side still uses single-file <name>.rs for module(s) \
                 {shape_mismatch_soft:?} which the production side has split into \
                 <name>/ subdirectories. Consider syncing the split on test side \
                 (OPTIMIZATION.md §N1 方案 A) for mirror shape parity."
            );
        }
    }

    /// cosmos submodules: `mpd-test::cosmos::*` mirrors `mpd-cosmos/src/*.rs`
    /// modulo the structural `lib.rs` / `ffi.rs`. No test-only extras are
    /// expected today.
    #[test]
    fn cosmos_submodules_mirror_cosmos_src() {
        let test_cosmos = list_entries(sibling_crate_src("mps-test/src/cosmos"));
        let cosmos_src = list_entries(sibling_crate_src("mps-cosmos/src"));

        // cosmos/src/lib.rs and cosmos/src/ffi.rs are structural/FFI-only and
        // NOT mirrored into mps-test/src/cosmos/. lib.rs is already filtered
        // out by list_entries; ffi must be explicitly excluded below.
        let missing_in_test: std::collections::BTreeSet<String> = cosmos_src
            .keys()
            .filter(|n| *n != "ffi" && !test_cosmos.contains_key(*n))
            .cloned()
            .collect();
        let orphans_in_test: std::collections::BTreeSet<String> = test_cosmos
            .keys()
            // test-only adapters (no cosmos submodule): evidence/benchmark-only
            // test files — same policy as the rapier-side `test_only_extras`.
            .filter(|n| {
                !cosmos_src.contains_key(*n)
                    // C1 长期轨道基准（evidence/numeric-drift.md 附表）
                    && *n != "drift_matrix"
                    && *n != "long_orbit"
                    // A2 收敛性扫描 + adaptive 修复证据
                    && *n != "convergence"
                    // C3 天体自转表面平衡（evidence/convergence.md）
                    && *n != "celestial_rotation"
                    // C2 central_body 切换 SOI 契约（evidence/convergence.md）
                    && *n != "central_body_switch"
                    // E2 双世界一致性抽查
                    && *n != "cross_world"
            })
            .cloned()
            .collect();

        // §N1 morphology alignment (cosmos side, same rules as rapier above)
        let shape_mismatch_hard: Vec<String> = test_cosmos
            .iter()
            .filter(|(name, is_dir)| cosmos_src.get(*name).copied() == Some(false) && **is_dir)
            .map(|(n, _)| n.clone())
            .collect();

        if !missing_in_test.is_empty()
            || !orphans_in_test.is_empty()
            || !shape_mismatch_hard.is_empty()
        {
            panic!(
                "cosmos mirror drift detected:\n  \
                 missing test modules (cosmos src has submodule but test has no mirror): \
                 {missing_in_test:?}\n  \
                 孤儿 test modules (test has but cosmos src doesn't define): \
                 {orphans_in_test:?}\n  \
                 形态错位 (test has <name>/ but cosmos src only has <name>.rs): \
                 {shape_mismatch_hard:?}\n"
            );
        }
    }

    /// Every `*.rs` under `crates/mps-test/src` must be reachable from the
    /// module tree.
    ///
    /// The mirror check above only compares *file names*. A file that exists
    /// but is never declared (`pub mod x;`) therefore passes it while never
    /// being compiled and never running its tests — which is exactly how
    /// `rapier/compat.rs` (16 tests) and `cosmos/flight/` (13 tests) went dead
    /// while still inflating the published `TEST_COUNT`.
    ///
    /// Files whose name starts with `_` are treated as scratch and skipped.
    #[test]
    fn every_test_file_is_wired_into_the_module_tree() {
        let src_root = sibling_crate_src("mps-test/src");
        let mut files: Vec<PathBuf> = Vec::new();
        collect_rs_files(&src_root, &mut files);

        let sources: Vec<(String, String)> = files
            .iter()
            .filter_map(|p| {
                fs::read_to_string(p)
                    .ok()
                    .map(|c| (p.display().to_string(), c))
            })
            .collect();

        let mut unwired: Vec<String> = Vec::new();
        for path in &files {
            let Ok(rel) = path.strip_prefix(&src_root) else {
                continue;
            };
            let file_name = rel.file_name().and_then(|n| n.to_str()).unwrap_or("");
            let stem = if file_name == "mod.rs" {
                rel.parent()
                    .and_then(|p| p.file_name())
                    .and_then(|n| n.to_str())
                    .unwrap_or("")
                    .to_string()
            } else {
                rel.file_stem()
                    .and_then(|n| n.to_str())
                    .unwrap_or("")
                    .to_string()
            };
            if stem.is_empty() || stem == "lib" || stem.starts_with('_') {
                continue;
            }
            // Wire-up detection must match a *real* module declaration, not a
            // substring: `body.contains("mod x;")` also matches a comment line
            // ("// mod x; removed") or a suffix collision ("mod x_old;" does
            // not, but "mod x;" inside "/// mod x;" does). Parse per line and
            // require the declaration token at line start (after optional
            // `pub`), and the statement terminated by `;`.
            let needle_line = |body: &str| {
                body.lines().any(|line| {
                    let t = line.trim_start();
                    let t = t.strip_prefix("pub ").unwrap_or(t);
                    let t = t.strip_prefix("mod ").unwrap_or(t);
                    let t = t.trim_end();
                    t == format!("{stem};")
                })
            };
            if !sources.iter().any(|(_, body)| needle_line(body)) {
                unwired.push(rel.display().to_string());
            }
        }
        unwired.sort();

        assert!(
            unwired.is_empty(),
            "test files exist but are never declared with `mod <name>;` — they are \
             not compiled and their tests never run: {unwired:?}\n  \
             Fix by adding the `pub mod` line to the enclosing module, or delete the \
             file. Do not silence this test."
        );
    }

    /// Recursively collect every `*.rs` file under `dir`.
    fn collect_rs_files(dir: &std::path::Path, out: &mut Vec<PathBuf>) {
        let Ok(entries) = fs::read_dir(dir) else {
            return;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                collect_rs_files(&path, out);
            } else if path.extension().and_then(|s| s.to_str()) == Some("rs") {
                out.push(path);
            }
        }
    }
}
