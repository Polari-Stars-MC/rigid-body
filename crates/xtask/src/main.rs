//! Workspace automation helper (OPTIMIZATION.md §N3).
//!
//! Subcommands:
//!   `dump-metrics`  →  regenerates `crates/mps-web/src/metrics.rs`
//!   `gen-java`       →  scans `#[java_struct]`/`#[java_enum]` annotations and
//!                        generates Java value classes under the configured output dir.

mod gen_java;

use std::path::Path;
use std::process::ExitCode;

use walkdir::WalkDir;

pub(crate) const JAVA_PACKAGE_DEFAULT: &str = "org.polaris2023.mps.ffi";

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let Some(sub) = args.next() else {
        eprintln!(
            "xtask: missing subcommand. Expected one of: dump-metrics, gen-java\n\
             Usage: cargo run -p xtask -- dump-metrics\n\
             Usage: cargo run -p xtask -- gen-java [output_dir]"
        );
        return ExitCode::from(2);
    };

    let workspace_root = find_workspace_root().expect("workspace root");

    match sub.as_str() {
        "dump-metrics" => match dump_metrics(&workspace_root) {
            Ok(report) => {
                println!("{report}");
                ExitCode::from(0)
            }
            Err(err) => {
                eprintln!("xtask dump-metrics failed: {err}");
                ExitCode::from(1)
            }
        },
        "gen-java" => {
            let output_dir = args.next();
            match gen_java::run(&workspace_root, output_dir.as_deref()) {
                Ok(report) => {
                    println!("{report}");
                    ExitCode::from(0)
                }
                Err(err) => {
                    eprintln!("xtask gen-java failed: {err}");
                    ExitCode::from(1)
                }
            }
        }
        other => {
            eprintln!(
                "xtask: unknown subcommand {other:?}. Expected one of: dump-metrics, gen-java."
            );
            ExitCode::from(2)
        }
    }
}

/// Locate the workspace root by walking up from CARGO_MANIFEST_DIR (which
/// points at `crates/xtask/` when running via `cargo run -p xtask`) until a
/// `Cargo.toml` containing `[workspace]` is found.
fn find_workspace_root() -> Option<std::path::PathBuf> {
    use std::path::PathBuf;
    let manifest = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR")?);
    let mut dir = Some(manifest.as_path());
    while let Some(p) = dir {
        let toml = p.join("Cargo.toml");
        if toml.is_file() {
            let content = std::fs::read_to_string(&toml).ok()?;
            if content.contains("[workspace]") {
                return Some(p.to_path_buf());
            }
        }
        dir = p.parent();
    }
    None
}

/// Count `#[test]` lines across the test crate's `src/` tree.
fn count_tests(workspace_root: &Path) -> usize {
    let test_dir = workspace_root.join("crates/mps-test/src");
    let mut count = 0;
    for entry in WalkDir::new(&test_dir).into_iter().filter_map(|e| e.ok()) {
        let path = entry.path();
        if path.extension().and_then(|s| s.to_str()) != Some("rs") {
            continue;
        }
        let Ok(content) = std::fs::read_to_string(path) else {
            continue;
        };
        // Matches `#[test]` (possibly with arbitrary whitespace inside the
        // brackets). Trims to entire line for cheap check.
        count += content
            .lines()
            .filter(|line| line.trim() == "#[test]")
            .count();
    }
    count
}

/// Count `jni!(` and `jni_e_c!(` token-starts in `crates/mps-jni/src/lib.rs`.
fn count_jni_methods(workspace_root: &Path) -> usize {
    let lib = workspace_root.join("crates/mps-jni/src/lib.rs");
    let Ok(content) = std::fs::read_to_string(&lib) else {
        return 0;
    };
    let mut count = 0;
    for line in content.lines() {
        // simple contains-check. We accept false positives only if a
        // comment mentions `jni!(:` literally — unlikely.
        let trimmed = line.trim();
        if trimmed.starts_with("//") {
            continue;
        }
        if trimmed.contains("jni!(") || trimmed.contains("jni_e_c!(") {
            count += 1;
        }
    }
    count
}

/// Count `pub extern "C"` declarations across `crates/mps-core/src/rapier/**/*.rs`.
fn count_core_ffi(workspace_root: &Path) -> usize {
    let dir = workspace_root.join("crates/mps-core/src/rapier");
    let mut count = 0;
    for entry in WalkDir::new(&dir).into_iter().filter_map(|e| e.ok()) {
        let path = entry.path();
        if path.extension().and_then(|s| s.to_str()) != Some("rs") {
            continue;
        }
        let Ok(content) = std::fs::read_to_string(path) else {
            continue;
        };
        // Match `pub extern "C" fn ...` (allowing for `unsafe` between pub
        // and extern, and either "C" or C without quotes is invalid so ignore).
        // Simple line-start anchor.
        for line in content.lines() {
            let t = line.trim_start();
            // Accept: pub extern "C" fn, pub(crate) extern "C" fn,
            // pub unsafe extern "C" fn, pub(crate) unsafe extern "C" fn.
            if (t.starts_with("pub extern \"C\"")
                || t.starts_with("pub(crate) extern \"C\"")
                || t.starts_with("pub unsafe extern \"C\"")
                || t.starts_with("pub(crate) unsafe extern \"C\""))
                && t.contains("fn ")
            {
                count += 1;
            }
        }
    }
    count
}

/// Count distinct gravity models exposed through `world_set_*_gravity` C-ABI setters
/// in `crates/mps-core/src/rapier` (e.g. `world_set_gravity`, `world_set_newton_gravity`,
/// `world_set_mond_gravity`, `world_set_cross_validate_gravity`). Returns 0 on failure.
fn count_gravity_models(workspace_root: &Path) -> usize {
    use std::collections::HashSet;
    let dir = workspace_root.join("crates/mps-core/src/rapier");
    let mut seen = HashSet::new();
    for entry in WalkDir::new(&dir).into_iter().filter_map(|e| e.ok()) {
        let path = entry.path();
        if path.extension().and_then(|s| s.to_str()) != Some("rs") {
            continue;
        }
        let Ok(content) = std::fs::read_to_string(path) else {
            continue;
        };
        for line in content.lines() {
            let t = line.trim_start();
            if (t.starts_with("pub extern \"C\"")
                || t.starts_with("pub(crate) extern \"C\"")
                || t.starts_with("pub unsafe extern \"C\"")
                || t.starts_with("pub(crate) unsafe extern \"C\""))
                && t.contains("fn ")
            {
                let Some(after) = t.split("fn ").nth(1) else {
                    continue;
                };
                let name = after
                    .split(|c: char| !(c.is_alphanumeric() || c == '_'))
                    .next()
                    .unwrap_or("");
                if name.starts_with("world_set_")
                    && name.contains("gravity")
                    && !name.ends_with("_flag")
                {
                    seen.insert(name.to_string());
                }
            }
        }
    }
    seen.len()
}

/// Read the workspace `version` from the root `Cargo.toml`.
fn read_workspace_version(workspace_root: &Path) -> String {
    let toml = workspace_root.join("Cargo.toml");
    if let Ok(content) = std::fs::read_to_string(&toml) {
        for line in content.lines() {
            let t = line.trim();
            if let Some(rest) = t.strip_prefix("version")
                && let Some(v) = rest.trim_start().strip_prefix('=')
            {
                let v = v.trim().trim_matches('"').trim();
                if !v.is_empty() {
                    return v.to_string();
                }
            }
        }
    }
    "0.0.0".to_string()
}

/// Count `pub mod <name>` declarations under a module directory (the number of
/// formula submodules). Returns 0 if the directory is absent.
fn count_pub_mods(workspace_root: &Path, rel_dirs: &[&str]) -> usize {
    let mut count = 0;
    for rel in rel_dirs {
        let dir = workspace_root.join(rel);
        if !dir.is_dir() {
            continue;
        }
        for entry in WalkDir::new(&dir).into_iter().filter_map(|e| e.ok()) {
            let path = entry.path();
            if path.extension().and_then(|s| s.to_str()) != Some("rs") {
                continue;
            }
            if let Ok(content) = std::fs::read_to_string(path) {
                for line in content.lines() {
                    let t = line.trim_start();
                    if t.starts_with("pub mod ") {
                        count += 1;
                    }
                }
            }
        }
    }
    count
}

/// Count enum variants of `pub enum <name>` in a file (CelestialBodyId,
/// OrbitIntegration, ...). Returns 0 on any failure.
fn count_enum_variants(workspace_root: &Path, rel_file: &str, enum_name: &str) -> usize {
    let file = workspace_root.join(rel_file);
    let Ok(content) = std::fs::read_to_string(&file) else {
        return 0;
    };
    let mut count = 0;
    let mut in_enum = false;
    for line in content.lines() {
        if !in_enum {
            if line
                .trim_start()
                .starts_with(&format!("pub enum {enum_name}"))
            {
                in_enum = true;
            }
            continue;
        }
        let t = line.trim_start();
        if t.starts_with('}') {
            break;
        }
        // A variant line: `Name` / `Name = N` / `Name(...)`.
        if let Some(name) = t.split_whitespace().next()
            && !name.is_empty()
            && name
                .chars()
                .next()
                .map(|c| c.is_uppercase())
                .unwrap_or(false)
            && !name.contains("//")
        {
            count += 1;
        }
    }
    count
}

/// Count `pub extern "C" fn <prefix>_*` declarations in `crates/mps-core/src/rapier`
/// whose name starts with `prefix_` (unique function name). Returns 0 on failure.
fn count_core_ffi_prefix(workspace_root: &Path, prefix: &str) -> usize {
    use std::collections::HashSet;
    let dir = workspace_root.join("crates/mps-core/src/rapier");
    let mut seen = HashSet::new();
    for entry in WalkDir::new(&dir).into_iter().filter_map(|e| e.ok()) {
        let path = entry.path();
        if path.extension().and_then(|s| s.to_str()) != Some("rs") {
            continue;
        }
        let Ok(content) = std::fs::read_to_string(path) else {
            continue;
        };
        for line in content.lines() {
            let t = line.trim_start();
            if (t.starts_with("pub extern \"C\"")
                || t.starts_with("pub(crate) extern \"C\"")
                || t.starts_with("pub unsafe extern \"C\"")
                || t.starts_with("pub(crate) unsafe extern \"C\""))
                && t.contains("fn ")
            {
                // Extract the function name: the token after `fn`.
                if let Some(after) = t.split("fn ").nth(1) {
                    let name = after
                        .split(|c: char| !(c.is_alphanumeric() || c == '_'))
                        .next()
                        .unwrap_or("");
                    if name.starts_with(prefix) {
                        seen.insert(name.to_string());
                    }
                }
            }
        }
    }
    seen.len()
}

/// Generate `crates/mps-web/src/metrics.rs` from the latest counts.
/// Count `jni!`/`jni_e_c!` entries whose method name starts with `softBody` in
/// `crates/mps-jni/src/lib.rs`. Mirrors `count_jni_methods` but scoped to soft body.
fn count_soft_body_jni(workspace_root: &Path) -> usize {
    let lib = workspace_root.join("crates/mps-jni/src/lib.rs");
    let Ok(content) = std::fs::read_to_string(&lib) else {
        return 0;
    };
    content
        .lines()
        .filter(|line| {
            let t = line.trim();
            !t.starts_with("//")
                && (t.contains("jni!(") || t.contains("jni_e_c!("))
                && t.contains("softBody")
        })
        .count()
}

/// Count `#[test]` functions whose test name contains `soft_body` in `mps-test`.
fn count_soft_body_tests(workspace_root: &Path) -> usize {
    let test_dir = workspace_root.join("crates/mps-test/src");
    let mut count = 0;
    for entry in WalkDir::new(&test_dir).into_iter().filter_map(|e| e.ok()) {
        let path = entry.path();
        if path.extension().and_then(|s| s.to_str()) != Some("rs") {
            continue;
        }
        let Ok(content) = std::fs::read_to_string(path) else {
            continue;
        };
        let lines: Vec<&str> = content.lines().collect();
        for (i, line) in lines.iter().enumerate() {
            if line.trim() == "#[test]" {
                // The test fn name is on the next non-empty line.
                if let Some(next) = lines[i + 1..].iter().find(|l| !l.trim().is_empty())
                    && next.contains("soft_body")
                {
                    count += 1;
                }
            }
        }
    }
    count
}

/// Count natural satellites in `mps-formula::celestial_data::MOONS`.
/// Each entry carries a `parent_planet:` field, so we count those lines.
fn count_moons(workspace_root: &Path) -> usize {
    let file = workspace_root.join("crates/mps-formula/src/celestial_data.rs");
    let Ok(content) = std::fs::read_to_string(&file) else {
        return 0;
    };
    content
        .lines()
        .filter(|line| line.trim().starts_with("parent_planet:"))
        .count()
}

fn dump_metrics(workspace_root: &Path) -> Result<String, String> {
    let tests = count_tests(workspace_root);
    let jni_methods = count_jni_methods(workspace_root);
    let core_ffi = count_core_ffi(workspace_root);

    // Extended counts consumed by the documentation site pages.
    let version = read_workspace_version(workspace_root);
    let formula_modules = count_pub_mods(
        workspace_root,
        &[
            "crates/mps-formula/src/scientists",
            "crates/mps-formula/src/disciplines",
        ],
    );
    let celestial = count_enum_variants(
        workspace_root,
        "crates/mps-formula/src/celestial_data.rs",
        "CelestialBodyId",
    );
    let gravity_models = count_gravity_models(workspace_root);
    let integrators = count_enum_variants(
        workspace_root,
        "crates/mps-cosmos/src/world.rs",
        "OrbitIntegration",
    );
    let ffi_world = count_core_ffi_prefix(workspace_root, "world");
    let ffi_rigid_body = count_core_ffi_prefix(workspace_root, "rigid_body");
    let ffi_collider = count_core_ffi_prefix(workspace_root, "collider");
    let ffi_query = count_core_ffi_prefix(workspace_root, "query");
    let ffi_soft_body = count_core_ffi_prefix(workspace_root, "soft_body");
    let ffi_disaster = count_core_ffi_prefix(workspace_root, "disaster");
    let ffi_volcano = count_core_ffi_prefix(workspace_root, "volcano");
    let jni_soft_body = count_soft_body_jni(workspace_root);
    let soft_body_tests = count_soft_body_tests(workspace_root);
    let moon_count = count_moons(workspace_root);

    let mut body = format!(
        "// Auto-generated by `cargo run -p xtask -- dump-metrics`.\n\
         // Do NOT edit by hand — re-run the command above after large changes\n\
         // (OPTIMIZATION.md §N3).\n\
         //\n\
         // Source of truth:\n\
         //   TEST_COUNT       = `grep -rh '#[test]' crates/mps-test/src/ | wc -l`\n\
         //   JNI_METHOD_COUNT= `grep -cE 'jni!\\\\(|jni_e_c!\\\\(' crates/mps-jni/src/lib.rs`\n\
         //   CORE_FFI_COUNT  = `grep -rhE '^pub extern \"C\"' crates/mps-core/src/rapier/ | wc -l`\n\
         //   VERSION         = workspace `version` in root Cargo.toml\n\
         //   FORMULA_MODULE_COUNT / CELESTIAL_COUNT / GRAVITY_MODEL_COUNT / INTEGRATOR_COUNT\n\
         //                 = module / enum counts derived from source\n\
         //   FFI_WORLD / FFI_RIGID_BODY / FFI_COLLIDER / FFI_QUERY\n\
         //                 = `pub extern \"C\" fn <prefix>_*` in crates/mps-core/src/rapier\n\
         \n\
         /// Workspace version (from root Cargo.toml), for the footer / brand.\n\
         pub const VERSION: &str = \"{version}\";\n\
         /// Total number of `#[test]` items in `mps-test`.\n\
         pub const TEST_COUNT: &str = \"{tests}\";\n\
         /// Total number of `jni!(`/`jni_e_c!(` method entries in `mps-jni`.\n\
         pub const JNI_METHOD_COUNT: &str = \"{jni_methods}\";\n\
         /// Total number of `pub extern \"C\" fn` declarations in `mps-core/rapier`.\n\
         pub const CORE_FFI_COUNT: &str = \"{core_ffi}\";\n\
         /// Number of `pub mod` formula submodules under mps-formula scientists+disciplines.\n\
         pub const FORMULA_MODULE_COUNT: &str = \"{formula_modules}\";\n\
         /// Number of `CelestialBodyId` variants (built-in celestial bodies).\n\
         pub const CELESTIAL_COUNT: &str = \"{celestial}\";\n\
         /// Number of `world_set_*gravity*` C-ABI setters in mps-core (one per
         /// selectable gravity model). Counted from `crates/mps-core/src/rapier`;
         /// there is no `mps-core/src/gravity` directory.\n\
         pub const GRAVITY_MODEL_COUNT: &str = \"{gravity_models}\";\n\
         /// Number of `OrbitIntegration` variants (integrator selection).\n\
         pub const INTEGRATOR_COUNT: &str = \"{integrators}\";\n\
         /// `pub extern \"C\" fn world_*` declarations in mps-core/rapier.\n\
         pub const FFI_WORLD: &str = \"{ffi_world}\";\n\
         /// `pub extern \"C\" fn rigid_body_*` declarations in mps-core/rapier.\n\
         pub const FFI_RIGID_BODY: &str = \"{ffi_rigid_body}\";\n\
         /// `pub extern \"C\" fn collider_*` declarations in mps-core/rapier.\n\
         pub const FFI_COLLIDER: &str = \"{ffi_collider}\";\n\
         /// `pub extern \"C\" fn query_*` declarations in mps-core/rapier.\n\
         pub const FFI_QUERY: &str = \"{ffi_query}\";\n\
         /// `pub extern \"C\" fn soft_body_*` declarations in mps-core/rapier.\n\
         pub const FFI_SOFT_BODY: &str = \"{ffi_soft_body}\";\n\
         /// `pub extern \"C\" fn disaster_*` declarations in mps-core/rapier.\n\
         pub const FFI_DISASTER: &str = \"{ffi_disaster}\";\n\
         /// `pub extern \"C\" fn volcano_*` declarations in mps-core/rapier.\n\
         pub const FFI_VOLCANO: &str = \"{ffi_volcano}\";\n\
         /// `jni!` entries with a `softBody*` method name in mps-jni.\n\
         pub const JNI_SOFT_BODY: &str = \"{jni_soft_body}\";\n\
         /// `#[test]` functions whose name contains `soft_body` in mps-test.\n\
        pub const TEST_SOFT_BODY: &str = \"{soft_body_tests}\";\n"
    );
    // MOON_COUNT appended separately to avoid multi-line format! line-continuation escapes.
    let trimmed = body.trim_end_matches('\n').to_string();
    body = format!(
        "{}\n/// Number of natural satellites (moons) in `mps-formula::celestial_data::MOONS`.\n
pub const MOON_COUNT: &str = \"{}\";\n",
        trimmed, moon_count,
    );
    // Collapse any accidental double newlines introduced by appended blocks.
    body = body.replace("\n\n", "\n");
    let out_path = workspace_root.join("crates/mps-web/src/metrics.rs");
    std::fs::write(&out_path, &body).map_err(|e| format!("write {}: {e}", out_path.display()))?;

    Ok(format!(
        "xtask: wrote {out}\n  VERSION           = {version}\n  TEST_COUNT       = {tests}\n  JNI_METHOD_COUNT = {jni_methods}\n  CORE_FFI_COUNT   = {core_ffi}\n  FORMULA_MODULE_COUNT = {formula_modules}\n  CELESTIAL_COUNT  = {celestial}\n  GRAVITY_MODEL_COUNT = {gravity_models}\n  INTEGRATOR_COUNT = {integrators}\n  FFI_WORLD        = {ffi_world}\n  FFI_RIGID_BODY   = {ffi_rigid_body}\n  FFI_COLLIDER     = {ffi_collider}\n  FFI_QUERY        = {ffi_query}\n  FFI_SOFT_BODY    = {ffi_soft_body}\n  FFI_DISASTER     = {ffi_disaster}\n  FFI_VOLCANO      = {ffi_volcano}\n  JNI_SOFT_BODY    = {jni_soft_body}\n  TEST_SOFT_BODY   = {soft_body_tests}",
        out = out_path.display()
    ))
}
