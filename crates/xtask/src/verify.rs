//! `xtask verify` — the single acceptance gate for this workspace.
//!
//! Before this subcommand the definition of "done" was spread across
//! `AGENTS.md`, `docs/CHECKPOINT_RECOVERY.md`, the four `.github/workflows/ci.yml`
//! jobs and five guard tests — and the copies had already drifted (AGENTS.md
//! told you to run `cargo fmt --all --check`, which CI explicitly forbids
//! because `--all` reflows the vendored `rapier/` against the fork's own
//! baseline). A human could carry the real procedure in their head; an agent
//! could not.
//!
//! This module runs the CI steps in CI's order and reports one exit code.
//! It is not a second CI: every step shells out to exactly the command
//! `.github/workflows/ci.yml` runs, so the two cannot disagree about *how* to
//! check — only about *where*.
//!
//! Steps, in order:
//!
//! ```text
//! fmt            cargo fmt --check -p <every workspace member>   (never --all)
//! clippy         cargo clippy --all-targets -- -D warnings
//! test           cargo test
//! build-release  cargo build --release        (also regenerates both headers)
//! headers        git diff --exit-code -- crates/{mps-core,mps-cosmos}/include/*
//! metrics        in-process `dump-metrics`, then git diff on metrics.rs
//! web-audit      <python> crates/mps-web/scripts/web_audit.py
//! ```
//!
//! The generated-artifact steps run *after* the builds because cbindgen writes
//! the headers from a `build.rs`: a stale header is only visible once something
//! has had the chance to regenerate it.
//!
//! Windows note: `cargo test` rebuilds `target/debug/xtask.exe`, which is the
//! image currently executing this gate. Cargo skips the link because we just
//! built it, so this is normally a non-issue; if a future cargo ever relinks it,
//! `--skip test` keeps the gate usable.
//!
//! Host limits worth knowing (both are *this machine*, not this repo):
//!
//! - `clippy`/`test` build `mps-web`, which pulls `reqwest` → `rustls` → `ring`,
//!   and `ring` compiles C. If the host has no working C toolchain for the
//!   active Rust target, those steps fail on `ring` and the gate reports it.
//!   That is a real "CI would fail here" signal for a dev box, not a false
//!   positive — the fix is the toolchain, not skipping the step.
//! - `web-audit` needs a Python 3 interpreter (`python3` or `python` on PATH).

use std::path::Path;
use std::process::Command;
use std::time::{Duration, Instant};

/// Steps in the order CI runs them.
pub(crate) const ALL_STEPS: &[&str] = &[
    "fmt",
    "clippy",
    "test",
    "build-release",
    "headers",
    "metrics",
    "web-audit",
];

/// One-line description of each step, for `--list` and the summary table.
fn describe(step: &str) -> &'static str {
    match step {
        "fmt" => {
            "rustfmt over the 10 workspace members only (never `--all`: the vendored rapier fork has its own baseline)"
        }
        "clippy" => "clippy over all targets with -D warnings",
        "test" => "the full default-feature test suite (includes every in-repo guard test)",
        "build-release" => "release build; also regenerates rigid_body.h / cosmos.h via cbindgen",
        "headers" => "generated C headers are byte-identical to a fresh build",
        "metrics" => "crates/mps-web/src/metrics.rs matches the generator",
        "web-audit" => {
            "i18n parity, in-page anchors, stat-num hygiene (crates/mps-web/scripts/web_audit.py)"
        }
        _ => "unknown step",
    }
}

#[derive(Default)]
pub(crate) struct Options {
    pub keep_going: bool,
    pub skip: Vec<String>,
    pub verbose: bool,
}

struct StepOutcome {
    name: &'static str,
    ok: bool,
    detail: String,
    elapsed: Duration,
}

/// Human-readable plan, printed by `xtask verify --list`.
pub(crate) fn plan() -> String {
    let mut out =
        String::from("xtask verify — acceptance gate (mirrors .github/workflows/ci.yml)\n");
    for (i, step) in ALL_STEPS.iter().enumerate() {
        out.push_str(&format!("  {}. {:<14} {}\n", i + 1, step, describe(step)));
        out.push_str(&format!("     {}\n", command_preview(step)));
    }
    out.push_str(
        "\nflags: --list | --keep-going | --verbose | --skip <step>[,<step>...]\n\
         exit:  0 = every step passed, 1 = at least one step failed, 2 = bad usage\n",
    );
    out
}

/// The exact command a step runs, for `--list` and failure messages.
fn command_preview(step: &str) -> &'static str {
    match step {
        "fmt" => "cargo fmt --check -p <workspace member>...",
        "clippy" => "cargo clippy --all-targets -- -D warnings",
        "test" => "cargo test",
        "build-release" => "cargo build --release",
        "headers" => {
            "git diff --exit-code -- crates/mps-core/include/rigid_body.h crates/mps-cosmos/include/cosmos.h"
        }
        "metrics" => {
            "dump-metrics (in process) + git diff --exit-code -- crates/mps-web/src/metrics.rs"
        }
        "web-audit" => "<python> crates/mps-web/scripts/web_audit.py",
        _ => "?",
    }
}

pub(crate) fn run(root: &Path, opts: &Options) -> Result<String, String> {
    for name in &opts.skip {
        if !ALL_STEPS.contains(&name.as_str()) {
            return Err(format!(
                "unknown step {name:?} in --skip. Known steps: {}",
                ALL_STEPS.join(", ")
            ));
        }
    }

    let started = Instant::now();
    let mut outcomes: Vec<StepOutcome> = Vec::new();
    let mut failed_detail = String::new();

    // `headers` and `metrics` compare against what a build just produced. If
    // every build step was skipped, say so instead of implying the comparison
    // proved something it could not.
    let any_build = ["clippy", "test", "build-release"]
        .iter()
        .any(|s| !opts.skip.iter().any(|k| k == s));

    for &step in ALL_STEPS {
        if opts.skip.iter().any(|k| k == step) {
            outcomes.push(StepOutcome {
                name: step,
                ok: true,
                detail: "skipped (--skip)".to_string(),
                elapsed: Duration::ZERO,
            });
            println!("[skip] {step}");
            continue;
        }

        println!("[ .. ] {step} — {}", describe(step));
        let begun = Instant::now();
        let result = match step {
            "fmt" => step_fmt(root, opts),
            "clippy" => step_cargo(
                root,
                opts,
                "clippy",
                &["clippy", "--all-targets", "--", "-D", "warnings"],
            ),
            "test" => step_cargo(root, opts, "test", &["test"]),
            "build-release" => step_cargo(root, opts, "build-release", &["build", "--release"]),
            "headers" => step_headers(root),
            "metrics" => step_metrics(root),
            "web-audit" => step_web_audit(root, opts),
            other => Err(format!("internal: no implementation for step {other:?}")),
        };
        let elapsed = begun.elapsed();

        let outcome = match result {
            Ok(detail) => StepOutcome {
                name: step,
                ok: true,
                detail,
                elapsed,
            },
            Err(detail) => StepOutcome {
                name: step,
                ok: false,
                detail: detail.clone(),
                elapsed,
            },
        };

        if outcome.ok {
            println!("[ ok ] {step} ({})", fmt_dur(outcome.elapsed));
            if !outcome.detail.is_empty() {
                println!("       {}", outcome.detail.replace('\n', "\n       "));
            }
        } else {
            println!("[FAIL] {step} ({})", fmt_dur(outcome.elapsed));
            println!("       {}", outcome.detail.replace('\n', "\n       "));
            println!("       next: {}", command_preview(step));
            failed_detail.push_str(&format!("- {step}: {}\n", first_line(&outcome.detail)));
            outcomes.push(outcome);
            if !opts.keep_going {
                break;
            }
            continue;
        }
        outcomes.push(outcome);
    }

    let total = started.elapsed();
    let ran = outcomes
        .iter()
        .filter(|o| o.detail != "skipped (--skip)")
        .count();
    let passed = outcomes
        .iter()
        .filter(|o| o.ok && o.detail != "skipped (--skip)")
        .count();
    let failed: Vec<&str> = outcomes.iter().filter(|o| !o.ok).map(|o| o.name).collect();

    let mut report = String::new();
    report.push_str("\nverify summary\n");
    for o in &outcomes {
        let mark = if o.detail == "skipped (--skip)" {
            "--"
        } else if o.ok {
            "ok"
        } else {
            "FAIL"
        };
        report.push_str(&format!(
            "  {mark:<4} {:<14} {:>8}\n",
            o.name,
            fmt_dur(o.elapsed)
        ));
    }
    if !any_build {
        report.push_str(
            "  note: every build step was skipped, so the headers/metrics diffs \
             compared against whatever was already on disk\n",
        );
    }

    if failed.is_empty() {
        report.push_str(&format!(
            "\nverify: {passed}/{ran} steps passed in {}\n",
            fmt_dur(total)
        ));
        Ok(report)
    } else {
        report.push_str(&format!(
            "\nverify: FAILED ({passed}/{ran} steps passed, {} failed: {}) after {}\n{}",
            failed.len(),
            failed.join(", "),
            fmt_dur(total),
            failed_detail,
        ));
        Err(report)
    }
}

// ---------------------------------------------------------------------------
// steps
// ---------------------------------------------------------------------------

fn step_fmt(root: &Path, opts: &Options) -> Result<String, String> {
    let packages = member_packages(root)?;
    let mut args: Vec<String> = vec!["fmt".into(), "--check".into()];
    for p in &packages {
        args.push("-p".into());
        args.push(p.clone());
    }
    let args_ref: Vec<&str> = args.iter().map(String::as_str).collect();
    run_tool(root, opts, cargo_program(), &args_ref, "cargo fmt --check")?;
    Ok(format!(
        "{} workspace members, vendored rapier/ untouched",
        packages.len()
    ))
}

fn step_cargo(root: &Path, opts: &Options, name: &str, args: &[&str]) -> Result<String, String> {
    run_tool(
        root,
        opts,
        cargo_program(),
        args,
        &format!("cargo {}", args.join(" ")),
    )?;
    Ok(format!("{name} clean"))
}

fn step_headers(root: &Path) -> Result<String, String> {
    let paths = [
        "crates/mps-core/include/rigid_body.h",
        "crates/mps-cosmos/include/cosmos.h",
    ];
    let diff = git_diff(root, &paths)?;
    if !diff.trim().is_empty() {
        return Err(format!(
            "generated header(s) differ from a fresh build — do NOT hand-edit them.\n\
             Rebuild (cargo build --release) and commit the regenerated files together with the \
             Rust change that produced them.\n\n{diff}"
        ));
    }
    Ok("rigid_body.h / cosmos.h up to date".to_string())
}

fn step_metrics(root: &Path) -> Result<String, String> {
    // Regenerate in-process rather than shelling out to
    // `cargo run -p xtask -- dump-metrics`: this binary is already xtask, and a
    // nested cargo would contend for the target-dir lock.
    crate::dump_metrics(root).map_err(|e| format!("dump-metrics failed: {e}"))?;
    let paths = ["crates/mps-web/src/metrics.rs"];
    let diff = git_diff(root, &paths)?;
    if !diff.trim().is_empty() {
        return Err(format!(
            "metrics.rs did not match the generator and has now been regenerated.\n\
             This compares the working tree against git HEAD, so it fails both when the file\n\
             was forgotten (`cargo test` counts changed but nobody re-ran dump-metrics) and when\n\
             it was regenerated but not yet committed. Commit the regeneration — never hand-edit\n\
             the file. All 18 constants are covered here; the in-repo guard only recomputes 3.\n\n{diff}"
        ));
    }
    Ok("metrics.rs up to date (18 constants)".to_string())
}

fn step_web_audit(root: &Path, opts: &Options) -> Result<String, String> {
    let python = find_python().ok_or_else(|| {
        "no python interpreter found (tried python3, python). The web audit is the only \
         non-Rust gate; install python 3 or pass --skip web-audit."
            .to_string()
    })?;
    let script = root.join("crates/mps-web/scripts/web_audit.py");
    if !script.is_file() {
        return Err(format!("missing {}", script.display()));
    }
    run_tool(
        root,
        opts,
        &python,
        &["crates/mps-web/scripts/web_audit.py"],
        &format!("{python} crates/mps-web/scripts/web_audit.py"),
    )?;
    Ok("i18n / anchors / stat-num clean".to_string())
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

fn cargo_program() -> &'static str {
    // `cargo run` exports the cargo that invoked us; prefer it over PATH so a
    // rustup shim mismatch cannot silently pick a different toolchain.
    static CARGO: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    CARGO.get_or_init(|| std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string()))
}

fn run_tool(
    root: &Path,
    opts: &Options,
    program: &str,
    args: &[&str],
    label: &str,
) -> Result<(), String> {
    let output = Command::new(program)
        .args(args)
        .current_dir(root)
        .output()
        .map_err(|e| format!("could not start `{label}` ({program}): {e}"))?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    if opts.verbose && (!stdout.is_empty() || !stderr.is_empty()) {
        println!("--- {label} stdout ---\n{stdout}--- {label} stderr ---\n{stderr}--- end ---");
    }
    if output.status.success() {
        return Ok(());
    }
    let tail = tail_lines(&format!("{stderr}{stdout}"), 40);
    let code = output
        .status
        .code()
        .map(|c| c.to_string())
        .unwrap_or_else(|| "signal".to_string());
    Err(format!("`{label}` failed (exit {code}):\n{tail}"))
}

/// `git diff HEAD -- <paths>`: the generated file as it exists in the working
/// tree **and** the index, against the last commit.
///
/// Comparing against the index (`git diff -- <paths>`) would treat a freshly
/// staged regeneration as clean, so a gate run right after `git add` would go
/// green while the file it just rewrote is still uncommitted. Comparing against
/// `HEAD` makes "the tree contains exactly what the generator produces, and it
/// is committed" the single condition — which is what CI's `git diff
/// --exit-code` step asserts on a fresh checkout.
fn git_diff(root: &Path, paths: &[&str]) -> Result<String, String> {
    let output = Command::new("git")
        .arg("diff")
        .arg("HEAD")
        .arg("--")
        .args(paths)
        .current_dir(root)
        .output()
        .map_err(|e| format!("could not run git (is it on PATH?): {e}"))?;
    if !output.status.success() {
        return Err(format!(
            "git diff failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        ));
    }
    Ok(String::from_utf8_lossy(&output.stdout).to_string())
}

fn find_python() -> Option<String> {
    for candidate in ["python3", "python"] {
        let ok = Command::new(candidate)
            .arg("--version")
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false);
        if ok {
            return Some(candidate.to_string());
        }
    }
    None
}

/// Package names of every root-`Cargo.toml` workspace member.
///
/// Read from the manifests instead of hardcoding a list: a new member crate
/// must be picked up by the fmt step automatically, otherwise the gate would
/// quietly stop checking it. Only the `members = [ ... ]` literal form used by
/// this repo is parsed; anything else is an error, never a silent fallback to
/// `--all` (which is the bug this step exists to prevent).
fn member_packages(root: &Path) -> Result<Vec<String>, String> {
    let manifest = root.join("Cargo.toml");
    let text = std::fs::read_to_string(&manifest)
        .map_err(|e| format!("read {}: {e}", manifest.display()))?;

    let mut members: Vec<String> = Vec::new();
    let mut in_members = false;
    for raw in text.lines() {
        let line = raw.trim();
        if line.starts_with('#') {
            continue;
        }
        if !in_members {
            if line.replace(' ', "").starts_with("members=[") {
                in_members = true;
            }
            continue;
        }
        if line.starts_with(']') {
            break;
        }
        if let Some(start) = line.find('"')
            && let Some(end) = line[start + 1..].find('"')
        {
            members.push(line[start + 1..start + 1 + end].to_string());
        }
    }
    if members.is_empty() {
        return Err(
            "could not parse `members = [...]` from the root Cargo.toml; refusing to fall back \
             to `cargo fmt --all`, which would reformat the vendored rapier fork"
                .to_string(),
        );
    }

    let mut packages = Vec::new();
    for member in &members {
        let path = root.join(member).join("Cargo.toml");
        let text =
            std::fs::read_to_string(&path).map_err(|e| format!("read {}: {e}", path.display()))?;
        let name = text
            .lines()
            .map(str::trim)
            .find_map(|l| l.strip_prefix("name"))
            .and_then(|rest| rest.trim_start().strip_prefix('='))
            .map(|v| v.trim().trim_matches('"').to_string())
            .filter(|v| !v.is_empty())
            .ok_or_else(|| format!("no `name = ...` in {}", path.display()))?;
        packages.push(name);
    }
    Ok(packages)
}

fn tail_lines(text: &str, n: usize) -> String {
    let lines: Vec<&str> = text.lines().collect();
    let start = lines.len().saturating_sub(n);
    lines[start..].join("\n")
}

fn first_line(text: &str) -> String {
    text.lines().next().unwrap_or("").trim().to_string()
}

fn fmt_dur(d: Duration) -> String {
    let secs = d.as_secs();
    if secs >= 60 {
        format!("{}m{:02}s", secs / 60, secs % 60)
    } else if secs > 0 {
        format!("{secs}.{:01}s", d.subsec_millis() / 100)
    } else {
        format!("{}ms", d.as_millis())
    }
}
