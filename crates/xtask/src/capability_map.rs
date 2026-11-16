//! `xtask capability-map` — generate `docs/CAPABILITY_MAP.md`.
//!
//! Changing one C ABI entry point in this workspace means touching up to six
//! places that have no mechanical link to each other:
//!
//! ```text
//! crates/mps-core/src/rapier/<module>.rs   the extern "C" fn itself
//! crates/mps-core/include/rigid_body.h     generated; must be regenerated
//! crates/mps-jni/src/lib.rs                the Java-facing wrapper, if any
//! crates/mps-test/java/…                   the JNI contract fixture, if any
//! crates/mps-test/src/rapier/<module>.rs   the mirrored integration test
//! docs/mps-core/<module>.md                the per-file tour
//! ```
//!
//! Before this command the mapping lived in people's heads, so an agent had to
//! re-derive it with five greps and still could not tell whether the JNI layer
//! or the tests referenced a symbol at all. The map below is generated from the
//! actual sources, so it cannot disagree with them.
//!
//! `--check` regenerates in memory and compares against the file on disk: the
//! `verify` gate runs that form, which makes staleness impossible and — unlike
//! hand-written guards — needs no duplicate copy of the scan logic.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use walkdir::WalkDir;

/// Relative path of the generated map.
pub(crate) const OUTPUT: &str = "docs/CAPABILITY_MAP.md";

/// One C ABI entry point discovered in `mps-core`.
struct Entry {
    name: String,
    feature: Option<String>,
}

/// Which mergeability class a symbol belongs to.
///
/// This is the input to the ABI-convergence plan
/// ([docs/abi-convergence.md](../abi-convergence.md)): it answers "if we replace
/// the 949 dedicated entry points with a generic property channel, which ones
/// disappear and which have to stay".
///
/// The classification is deliberately mechanical — it reads only the symbol
/// name — so it can be regenerated and cannot silently drift from the sources.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum MergeGroup {
    /// `*_set_<prop>` — becomes one `*_set_param(entity, PROP, value)`.
    SetParam,
    /// `*_clear_<prop>` — becomes `*_set_param(entity, PROP, none)`.
    ClearParam,
    /// `*_get_<prop>` / `*_read_<prop>` — becomes `*_get_param(entity, PROP, out)`.
    ReadParam,
    /// `*_add_<element>` / `*_insert_<element>` / `*_remove_<element>` — topology
    /// edits. Mergeable in principle (a `*_edit(entity, OP, ...)` channel) but
    /// the payloads differ a lot, so treat as a separate, harder batch.
    TopologyEdit,
    /// Everything else: world/lifecycle/solve/query entry points that carry real
    /// logic and must stay dedicated.
    Dedicated,
}

impl MergeGroup {
    /// Column label used in the generated table.
    fn label(self) -> &'static str {
        match self {
            MergeGroup::SetParam => "set_param",
            MergeGroup::ClearParam => "clear_param",
            MergeGroup::ReadParam => "read_param",
            MergeGroup::TopologyEdit => "topology",
            MergeGroup::Dedicated => "dedicated",
        }
    }

    /// Whether the group is expected to collapse into a generic channel.
    fn is_mergeable(self) -> bool {
        matches!(
            self,
            MergeGroup::SetParam | MergeGroup::ClearParam | MergeGroup::ReadParam
        )
    }
}

/// Bucket a symbol by its verb segment.
///
/// The C ABI naming convention is `模块_动词[_对象]`, but compound module names
/// put the verb later — `world_set_gravity` has the verb in segment 2,
/// `soft_body_set_coriolis` in segment 3, `rigid_body_set_ccd_enabled` in
/// segment 3. So the verb is **the first segment that is a known verb**, not a
/// fixed position.
///
/// Verb must come before the object: `set_gravity` is a setter; `world_get_count`
/// is a reader. If no segment is a known verb the symbol is behaviour, not data.
fn merge_group(name: &str) -> MergeGroup {
    for part in name.split('_') {
        match part {
            // Property accessors: readable/writable scalar state.
            "set" => return MergeGroup::SetParam,
            "clear" => return MergeGroup::ClearParam,
            "get" | "read" | "count" => return MergeGroup::ReadParam,
            // Topology edits: change the element set, not a scalar. Mergeable in
            // principle but the payloads differ a lot, so a separate batch.
            "add" | "insert" | "remove" | "detach" | "attach" => {
                return MergeGroup::TopologyEdit;
            }
            _ => {}
        }
    }
    MergeGroup::Dedicated
}

/// Everything the map reports for one source module.
struct ModuleRow {
    /// Source-relative module path, e.g. `rapier/rigid_body`.
    module: String,
    /// Shared name prefix of the module's symbols, when it has one.
    prefix: Option<String>,
    entries: usize,
    feature_gated: usize,
    doc_page: Option<String>,
    test_mirror: Option<String>,
    jni_covered: usize,
    test_hits: usize,
    /// Symbols with neither a JNI wrapper nor a test reference.
    unlinked: Vec<String>,
    /// Per-group symbol counts, in `MergeGroup` declaration order.
    ///
    /// The `set_param` / `clear_param` / `read_param` counts in aggregate are the
    /// "how many entry points a property channel would absorb" number from
    /// [docs/abi-convergence.md](../abi-convergence.md).
    merge_groups: [usize; 5],
}

pub(crate) fn run(root: &Path, check: bool) -> Result<String, String> {
    let rendered = render(root)?;
    let out_path = root.join(OUTPUT);

    if check {
        let existing = std::fs::read_to_string(&out_path).unwrap_or_default();
        if existing == rendered {
            return Ok(format!("{OUTPUT} is up to date"));
        }
        return Err(format!(
            "{OUTPUT} is stale — it no longer matches the sources.\n\
             Regenerate it, review the diff (the changes tell you what moved), and commit both:\n  \
             cargo run -p xtask -- capability-map\n\
             Do not hand-edit the file; it is derived from source."
        ));
    }

    std::fs::write(&out_path, &rendered)
        .map_err(|e| format!("write {}: {e}", out_path.display()))?;
    Ok(format!("wrote {OUTPUT} ({} bytes)", rendered.len()))
}

// ---------------------------------------------------------------------------
// collection
// ---------------------------------------------------------------------------

fn collect_core_entries(root: &Path) -> BTreeMap<String, Vec<Entry>> {
    let mut by_module: BTreeMap<String, Vec<Entry>> = BTreeMap::new();
    let dir = root.join("crates/mps-core/src/rapier");
    for entry in WalkDir::new(&dir).into_iter().filter_map(Result::ok) {
        let path = entry.path();
        if path.extension().and_then(|s| s.to_str()) != Some("rs") {
            continue;
        }
        let Ok(rel) = path.strip_prefix(&dir) else {
            continue;
        };
        let module = rel
            .with_extension("")
            .components()
            .map(|c| c.as_os_str().to_string_lossy().to_string())
            .collect::<Vec<_>>()
            .join("/");
        // `mod.rs` belongs to its directory, not to a module of its own.
        let module = module
            .trim_end_matches("/mod")
            .trim_end_matches("mod")
            .to_string();
        if module.is_empty() {
            continue;
        }

        let Ok(text) = std::fs::read_to_string(path) else {
            continue;
        };
        let mut pending_feature: Option<String> = None;
        let mut entries: Vec<Entry> = Vec::new();
        for line in text.lines() {
            let t = line.trim();
            if let Some(rest) = t.strip_prefix("#[cfg(feature = \"")
                && let Some(feature) = rest.split('"').next()
            {
                // A gate applies to the next item; only keep it if that item is
                // an FFI entry (attributes like `#[unsafe(no_mangle)]` sit in
                // between and must not clear it).
                pending_feature = Some(feature.to_string());
                continue;
            }
            let is_fn = t.starts_with("pub extern \"C\" fn ")
                || t.starts_with("pub unsafe extern \"C\" fn ")
                || t.starts_with("pub(crate) extern \"C\" fn ");
            if is_fn
                && let Some(name) = t.split("fn ").nth(1).and_then(|rest| {
                    let ident: String = rest
                        .chars()
                        .take_while(|c| c.is_alphanumeric() || *c == '_')
                        .collect();
                    (!ident.is_empty()).then_some(ident)
                })
            {
                entries.push(Entry {
                    name,
                    feature: pending_feature.clone(),
                });
                pending_feature = None;
                continue;
            }
            // Any other attribute keeps the gate pending; any other item clears
            // it (the gate was for something that is not an FFI entry).
            if !t.starts_with("#[") && !t.starts_with("//") && !t.is_empty() {
                pending_feature = None;
            }
        }
        if !entries.is_empty() {
            by_module.entry(module).or_default().extend(entries);
        }
    }
    by_module
}

/// Names declared in the generated C header.
///
/// cbindgen emits one declaration per symbol, starting at column 0, wrapped at
/// the argument list; the name is the identifier immediately before `(`.
/// Preprocessor lines are skipped so `#if defined(__cplusplus)` does not
/// register.
///
/// The `*` matters: a pointer return type renders as
/// `struct WorldHandle *world_create(struct Vec3 gravity);`, so the token before
/// `(` is `*world_create`. Missing that dropped 44 symbols — every
/// `*_builder_create`, `world_create`, `last_error_message` — from the map, which
/// is exactly the class of silent under-reporting this command exists to
/// prevent.
fn header_symbols(root: &Path) -> BTreeSet<String> {
    let path = root.join("crates/mps-core/include/rigid_body.h");
    let Ok(text) = std::fs::read_to_string(&path) else {
        return BTreeSet::new();
    };
    let mut out = BTreeSet::new();
    for line in text.lines() {
        if line.starts_with('#') || line.starts_with(char::is_whitespace) {
            continue;
        }
        let Some(open) = line.find('(') else {
            continue;
        };
        let head = &line[..open];
        let Some(token) = head.split_whitespace().next_back() else {
            continue;
        };
        let name = token.trim_start_matches('*');
        if !name.is_empty() && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
            out.insert(name.to_string());
        }
    }
    out
}

/// Core symbols referenced from inside `jni!` / `jni_e_c!` / `jni_space!` bodies.
///
/// Scoped to the macro invocations rather than the whole file: a doc comment
/// mentioning a symbol should not count as a Java binding for it.
fn jni_bound_symbols(root: &Path, known: &BTreeSet<String>) -> BTreeSet<String> {
    let path = root.join("crates/mps-jni/src/lib.rs");
    let Ok(text) = std::fs::read_to_string(&path) else {
        return BTreeSet::new();
    };
    let mut out = BTreeSet::new();
    for marker in ["jni!(", "jni_e_c!(", "jni_space!("] {
        for chunk in text.split(marker).skip(1) {
            // An invocation ends at the closing `});` of its body.
            let body = match chunk.find("});") {
                Some(end) => &chunk[..end],
                None => &chunk[..chunk.len().min(4000)],
            };
            for ident in identifiers(body) {
                if known.contains(&ident) {
                    out.insert(ident);
                }
            }
        }
    }
    out
}

/// Core symbols referenced from the test crate.
fn test_referenced_symbols(root: &Path, known: &BTreeSet<String>) -> BTreeSet<String> {
    let dir = root.join("crates/mps-test/src");
    let mut out = BTreeSet::new();
    for entry in WalkDir::new(&dir).into_iter().filter_map(Result::ok) {
        let path = entry.path();
        if path.extension().and_then(|s| s.to_str()) != Some("rs") {
            continue;
        }
        let Ok(text) = std::fs::read_to_string(path) else {
            continue;
        };
        for ident in identifiers(&text) {
            if known.contains(&ident) {
                out.insert(ident);
            }
        }
    }
    out
}

/// Every Rust identifier in `text` (runs of `[A-Za-z0-9_]`).
fn identifiers(text: &str) -> impl Iterator<Item = String> + '_ {
    text.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
        .filter(|s| !s.is_empty())
        .map(str::to_string)
}

/// Longest shared prefix of `names`, cut back to a `_` boundary and inclusive of
/// it, so `rigid_body_add_force` / `rigid_body_create` share `rigid_body_`.
///
/// Returns `None` unless the prefix covers **most** of the module: a prefix
/// shared by two outliers in a module of twenty is a coincidence, not a naming
/// convention, and reporting it as one would mislead the next person naming a
/// symbol here. Modules that mix families (e.g. `bounds`, which exposes both
/// `collider_builder_create_*` variants and k-DOP entries) land in that case and
/// honestly report "—".
fn shared_prefix(names: &[String]) -> Option<String> {
    if names.len() < 2 {
        return None;
    }
    let mut sorted: Vec<&str> = names.iter().map(String::as_str).collect();
    sorted.sort_unstable();
    let first = sorted[0];
    let last = sorted[sorted.len() - 1];
    let common: String = first
        .chars()
        .zip(last.chars())
        .take_while(|(a, b)| a == b)
        .map(|(a, _)| a)
        .collect();
    let boundary = common.rfind('_')?;
    let prefix = &common[..=boundary];
    // Guard against reporting a prefix that only a minority of the module obeys.
    let covered = names.iter().filter(|n| n.starts_with(prefix)).count();
    if prefix.is_empty() || covered * 2 < names.len() {
        return None;
    }
    Some(prefix.to_string())
}

/// Locate the per-file tour for a module, tolerating both naming styles used in
/// `docs/mps-core/` (`soft_body.md` keeps underscores, `shared-arena-mod.md`
/// hyphenates the directory split).
fn find_doc(root: &Path, module: &str) -> Option<String> {
    let base = module.strip_prefix("rapier/").unwrap_or(module);
    let flattened = base.replace('/', "-");
    let candidates = [
        format!("docs/mps-core/{flattened}.md"),
        format!("docs/mps-core/{}.md", flattened.replace('_', "-")),
        format!("docs/mps-core/{base}.md"),
    ];
    candidates
        .iter()
        .find(|c| root.join(c).is_file())
        .map(|c| c.trim_start_matches("docs/").to_string())
}

/// Locate the mirrored integration-test file for a module.
///
/// Mirrors `verify_module_mirror`'s rule: a production-side directory may still
/// be a single file on the test side (the documented transition state), so
/// `spaceflight/kepler` resolves to `rapier/spaceflight.rs`.
///
/// The returned path is project-root-relative (links in the generated page are
/// relative to `docs/`, so the renderer prefixes `../`).
fn find_test_mirror(root: &Path, module: &str) -> Option<String> {
    let base = module.strip_prefix("rapier/").unwrap_or(module);
    let dir = root.join("crates/mps-test/src/rapier");
    let mut candidates: Vec<PathBuf> = vec![dir.join(format!("{base}.rs"))];
    if let Some((head, _)) = base.split_once('/') {
        candidates.push(dir.join(format!("{head}.rs")));
    }
    candidates.iter().find(|p| p.is_file()).map(|p| {
        p.strip_prefix(root)
            .unwrap_or(p)
            .display()
            .to_string()
            .replace('\\', "/")
    })
}

// ---------------------------------------------------------------------------
// rendering
// ---------------------------------------------------------------------------

fn render(root: &Path) -> Result<String, String> {
    let by_module = collect_core_entries(root);
    let header = header_symbols(root);
    let all_names: BTreeSet<String> = by_module
        .values()
        .flatten()
        .map(|e| e.name.clone())
        .collect();
    let jni = jni_bound_symbols(root, &all_names);
    let tested = test_referenced_symbols(root, &all_names);

    let total_entries: usize = by_module.values().map(Vec::len).sum();
    let mut rows: Vec<ModuleRow> = Vec::new();
    for (module, entries) in &by_module {
        let names: Vec<String> = entries.iter().map(|e| e.name.clone()).collect();
        let unlinked: Vec<String> = names
            .iter()
            .filter(|n| !jni.contains(*n) && !tested.contains(*n))
            .cloned()
            .collect();
        rows.push(ModuleRow {
            module: module.clone(),
            prefix: shared_prefix(&names),
            entries: names.len(),
            feature_gated: entries.iter().filter(|e| e.feature.is_some()).count(),
            doc_page: find_doc(root, module),
            test_mirror: find_test_mirror(root, module),
            jni_covered: names.iter().filter(|n| jni.contains(*n)).count(),
            test_hits: names.iter().filter(|n| tested.contains(*n)).count(),
            unlinked,
            merge_groups: {
                let mut counts = [0usize; 5];
                for name in &names {
                    counts[merge_group(name) as usize] += 1;
                }
                counts
            },
        });
    }
    // Fail loudly rather than emit a map that silently under-reports.
    if rows.is_empty() {
        return Err(format!(
            "no `pub extern \"C\" fn` found under {} — the scan is broken, refusing to write a \
             map that claims the workspace has no C ABI",
            root.join("crates/mps-core/src/rapier").display()
        ));
    }

    let mut out = String::new();
    writeln!(
        out,
        "<!-- Generated by `cargo run -p xtask -- capability-map`. Do NOT edit. -->"
    )
    .ok();
    writeln!(out).ok();
    writeln!(out, "# 能力地图（C ABI → 模块 → 导览 → 测试 → JNI）").ok();
    writeln!(out).ok();
    writeln!(
        out,
        "本页由 `xtask capability-map` 从源码生成，`cargo run -p xtask -- verify` 的 \
         `capability-map` 步会在它过期时失败。**不要手改**。"
    )
    .ok();
    writeln!(out).ok();
    writeln!(
        out,
        "它回答一个问题：改一个 C ABI 入口时，那六处彼此没有机械联系的地方分别是哪几处。\
         改之前先查这里，不要 grep 五遍。"
    )
    .ok();
    writeln!(out).ok();
    writeln!(out, "## 汇总").ok();
    writeln!(out).ok();
    writeln!(out, "| 项 | 数量 |").ok();
    writeln!(out, "| --- | --- |").ok();
    writeln!(
        out,
        "| `mps-core` 里有 `extern \"C\"` 入口的源模块 | {} |",
        rows.len()
    )
    .ok();
    writeln!(out, "| `mps-core` C ABI 入口总数 | {total_entries} |").ok();
    writeln!(
        out,
        "| 其中出现在生成头文件 `rigid_body.h` 里 | {} |",
        all_names.iter().filter(|n| header.contains(*n)).count()
    )
    .ok();
    writeln!(out, "| 其中被 `mps-jni` 的 `jni!` 体引用 | {} |", jni.len()).ok();
    writeln!(out, "| 其中被 `mps-test` 引用 | {} |", tested.len()).ok();
    let unlinked_total: usize = rows.iter().map(|r| r.unlinked.len()).sum();
    writeln!(out, "| 既无 JNI 绑定也无测试引用 | {unlinked_total} |").ok();
    writeln!(out).ok();

    // --- main table -------------------------------------------------------
    writeln!(out, "## 模块表").ok();
    writeln!(out).ok();
    writeln!(
        out,
        "`入口` 是该模块的 C ABI 数量；`门控` 是其中带 `#[cfg(feature = …)]`、默认构建里\
         **不存在**的数量；`JNI` / `测试` 是该模块入口被 `mps-jni` 宏体 / `mps-test` 引用到的数量。"
    )
    .ok();
    writeln!(out).ok();
    writeln!(
        out,
        "`可合并组` 只按符号名的动词段机械判定，供 [abi-convergence.md](abi-convergence.md) \
         的收敛设计使用：`set_param` / `clear_param` / `read_param` 三类合计就是通用属性通道\
         能吸收的入口数；`topology` 是增删元素（可合并但载荷差异大，属第二批）；\
         `dedicated` 带真实逻辑，必须保留专用入口。"
    )
    .ok();
    writeln!(out).ok();
    writeln!(
        out,
        "| 模块 | 前缀 | 入口 | 门控 | 可合并组 | 导览页 | 镜像测试 | JNI | 测试 |"
    )
    .ok();
    writeln!(
        out,
        "| --- | --- | ---: | ---: | --- | --- | --- | ---: | ---: |"
    )
    .ok();
    for row in &rows {
        // `docs/CAPABILITY_MAP.md` lives in `docs/`, so links to `docs/mps-core/`
        // are `mps-core/…` and links to anything else climb one level.
        let doc = match &row.doc_page {
            Some(p) => format!("[{}]({})", p.trim_start_matches("mps-core/"), p),
            None => "—".to_string(),
        };
        let mirror = match &row.test_mirror {
            Some(p) => format!("[{}](../{})", p.rsplit('/').next().unwrap_or(p), p),
            None => "—".to_string(),
        };
        let prefix = match &row.prefix {
            Some(p) => format!("`{}*`", p),
            None => "—".to_string(),
        };
        let gated = if row.feature_gated == 0 {
            "—".to_string()
        } else {
            row.feature_gated.to_string()
        };
        // Only show the groups that actually occur, keeping the row narrow: a
        // module with 60 property accessors and 3 real functions should say
        // `set 40 / clear 12 / read 8 / dedicated 3`, not five zeros.
        let group_cell = {
            let mut parts: Vec<String> = Vec::new();
            for group in [
                MergeGroup::SetParam,
                MergeGroup::ClearParam,
                MergeGroup::ReadParam,
                MergeGroup::TopologyEdit,
                MergeGroup::Dedicated,
            ] {
                let n = row.merge_groups[group as usize];
                if n > 0 {
                    parts.push(format!("{} {}", group.label(), n));
                }
            }
            parts.join(" / ")
        };
        writeln!(
            out,
            "| `{}` | {} | {} | {} | {} | {} | {} | {} | {} |",
            row.module,
            prefix,
            row.entries,
            gated,
            group_cell,
            doc,
            mirror,
            row.jni_covered,
            row.test_hits
        )
        .ok();
    }
    writeln!(out).ok();

    // --- merge groups in aggregate ---------------------------------------
    let mut totals = [0usize; 5];
    for row in &rows {
        for (i, n) in row.merge_groups.iter().enumerate() {
            totals[i] += n;
        }
    }
    let mergeable = totals[MergeGroup::SetParam as usize]
        + totals[MergeGroup::ClearParam as usize]
        + totals[MergeGroup::ReadParam as usize];
    // Sanity: the aggregate must equal the sum over the groups `is_mergeable`
    // reports, and every symbol must land in exactly one group. Cheap, and it
    // catches a classifier change that quietly starts double-counting.
    let mergeable_by_flag: usize = [
        MergeGroup::SetParam,
        MergeGroup::ClearParam,
        MergeGroup::ReadParam,
    ]
    .into_iter()
    .filter(|g| g.is_mergeable())
    .map(|g| totals[g as usize])
    .sum();
    let grouped: usize = totals.iter().sum();
    if mergeable_by_flag != mergeable || grouped != total_entries {
        return Err(format!(
            "merge-group accounting is inconsistent: is_mergeable sum {mergeable_by_flag} != \
             set+clear+read {mergeable}, grouped {grouped} != total {total_entries}"
        ));
    }
    writeln!(out, "### 可合并组汇总").ok();
    writeln!(out).ok();
    writeln!(out, "| 组 | 含义 | 入口数 |").ok();
    writeln!(out, "| --- | --- | ---: |").ok();
    writeln!(
        out,
        "| `set_param` | `*_set_<prop>` → `*_set_param(e, PROP, v)` | {} |",
        totals[MergeGroup::SetParam as usize]
    )
    .ok();
    writeln!(
        out,
        "| `clear_param` | `*_clear_<prop>` → `*_set_param(e, PROP, none)` | {} |",
        totals[MergeGroup::ClearParam as usize]
    )
    .ok();
    writeln!(
        out,
        "| `read_param` | `*_get_<prop>` / `*_read_<prop>` → `*_get_param(e, PROP, out)` | {} |",
        totals[MergeGroup::ReadParam as usize]
    )
    .ok();
    writeln!(
        out,
        "| `topology` | `*_add/insert/remove/detach/attach_*`（第二批） | {} |",
        totals[MergeGroup::TopologyEdit as usize]
    )
    .ok();
    writeln!(
        out,
        "| `dedicated` | 带真实逻辑，必须保留专用入口 | {} |",
        totals[MergeGroup::Dedicated as usize]
    )
    .ok();
    writeln!(out).ok();
    writeln!(
        out,
        "**属性通道可直接吸收 {mergeable} 个入口（占 {total_entries} 的 {}%）。**",
        mergeable * 100 / total_entries.max(1)
    )
    .ok();
    writeln!(out).ok();

    // --- feature-gated ----------------------------------------------------
    let gated: Vec<(&str, &Entry)> = by_module
        .iter()
        .flat_map(|(m, es)| {
            es.iter()
                .filter(|e| e.feature.is_some())
                .map(move |e| (m.as_str(), e))
        })
        .collect();
    writeln!(out, "## 头文件声明了、默认构建里没有").ok();
    writeln!(out).ok();
    if gated.is_empty() {
        writeln!(out, "当前没有 feature 门控的入口。").ok();
    } else {
        writeln!(
            out,
            "cbindgen 不按 feature 过滤，所以这些符号出现在 `rigid_body.h` 里，但默认 features \
             编出的 cdylib 里**没有**它们。C/FFM 调用方链接时会失败。这不是 bug，是 \
             `default = []` 的代价——改这里时要记得宿主得自己开对应 feature。"
        )
        .ok();
        writeln!(out).ok();
        writeln!(out, "| 符号 | feature | 模块 |").ok();
        writeln!(out, "| --- | --- | --- |").ok();
        for (module, entry) in &gated {
            writeln!(
                out,
                "| `{}` | `{}` | `{}` |",
                entry.name,
                entry.feature.as_deref().unwrap_or("—"),
                module
            )
            .ok();
        }
    }
    writeln!(out).ok();

    // --- unlinked ---------------------------------------------------------
    writeln!(out, "## 既无 JNI 绑定也无测试引用").ok();
    writeln!(out).ok();
    writeln!(
        out,
        "这些入口是纯 C/FFM 面（或纯粹没人用）。改动它们时没有 Java 夹具会替你报警——\
         唯一的回归防线是 `mps-test` 里手写的用例。按模块列出，具体符号在该模块的导览页里。"
    )
    .ok();
    writeln!(out).ok();
    writeln!(out, "| 模块 | 数量 |").ok();
    writeln!(out, "| --- | ---: |").ok();
    for row in rows.iter().filter(|r| !r.unlinked.is_empty()) {
        writeln!(out, "| `{}` | {} |", row.module, row.unlinked.len()).ok();
    }
    writeln!(out).ok();

    // --- recipe -----------------------------------------------------------
    out.push_str(RECIPE);
    Ok(out)
}

/// Hand-written half of the map: the actual procedure. Generated tables say
/// *where things are*; this says *what to do* in each case, which is the part
/// that cannot be derived from source.
const RECIPE: &str = concat!(
    "## 改动配方\n",
    "\n",
    "### 加一个 C ABI 入口\n",
    "\n",
    "1. 写在对应模块的 `.rs` 里，用 `ffi_guard` 包住（`verify_ffi_guard` 会扫）。\n",
    "2. 名字跟本模块的前缀走（见上表「前缀」列）。\n",
    "3. `cargo build --release` → `rigid_body.h` 自动重生。\n",
    "4. 在 `mps-test` 的镜像文件里加用例；新子模块要同时在 `crates/mps-test/src/lib.rs` 挂 `pub mod`。\n",
    "5. 需要 Java 用就加 `jni!` 绑定，并在 `crates/mps-test/java/` 对应 fixture 里加断言。\n",
    "6. 改了这个模块的职责就更新 `docs/mps-core/<module>.md`。\n",
    "7. `cargo run -p xtask -- verify`。\n",
    "\n",
    "### 加一个体家族 / 子模块\n",
    "\n",
    "1. 新文件 + 在 `crates/mps-core/src/rapier/mod.rs` 挂 `pub mod`。\n",
    "2. **同一变更里**加 `crates/mps-test/src/rapier/<name>.rs`，并在 `lib.rs` 挂 `pub mod` \
       ——`verify_module_mirror` 两条断言都会红。\n",
    "3. 加 `docs/mps-core/<name>.md`，并在 `docs/mps-core.md` 加一行——`docs_index_consistency` 会红。\n",
    "4. 跑 `capability-map` 重生本页。\n",
    "\n",
    "### 改 arena 布局 / ABI 常量\n",
    "\n",
    "先读 [BOUNDARY.md](BOUNDARY.md) 的「ABI 兼容」与 [cosmos-arena.md](cosmos-arena.md)。两侧常量、\
     `mps-ffm::ABI_VERSION`、workspace 版本必须一起动；`arena_compat` 与 `version_consistency` 会红。\n",
    "\n",
    "### 改 JNI 方法签名\n",
    "\n",
    "1. 改 `crates/mps-jni/src/lib.rs` 的 `jni!` / `jni_e_c!` 条目。\n",
    "2. 改 `crates/mps-test/java/` 里对应的 fixture（`RapierNative` / `CosmosNative` / \
       `RecordingNative` / `BakerNative` / `EventsNative`）。\n",
    "3. **`xtask verify` 不跑 JVM**——这一层只有 CI 的 `jni-smoke` job 会验。本地绿 ≠ CI 绿。\n",
    "\n",
    "### 改公式（`mps-formula`）\n",
    "\n",
    "公式层没有世界、没有句柄，也不进本页的 C ABI 统计（本页只覆盖 `mps-core`）。\
     数值域规则见 [formula-numeric-domains.md](formula-numeric-domains.md)。\n",
);
