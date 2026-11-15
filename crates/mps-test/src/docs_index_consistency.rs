//! Documentation index consistency guard.
//!
//! The docs in `docs/` are the operating manual for agents working in this
//! repo: `AGENTS.md` says "read `docs/evidence/README.md` before claiming a
//! behaviour is pinned", `docs/README.md` says the per-file tours under
//! `docs/mps-core/` are reachable *only* through `docs/mps-core.md`. Those
//! promises are only worth as much as the indexes keeping up, and nothing was
//! checking them — by the time this test was written the indexes had already
//! drifted in three independent ways:
//!
//! - `docs/adr/0009-per-body-simulation-domains.md` existed and three C ABI
//!   entry points were frozen by it, but `docs/adr/README.md` stopped at 0008;
//! - five of eleven `docs/evidence/` files were missing from that index, so an
//!   agent following AGENTS.md would conclude those behaviours were unproven
//!   and re-derive them;
//! - `docs/mps-core/articulation.md` was orphaned, and `docs/integration/`
//!   (the only written host-side wiring contract) had no entry anywhere.
//!
//! Code has `verify_module_mirror` / `error_consistency` / `arena_compat` for
//! this class of drift. These four checks are the documentation equivalent.
//!
//! 1. **Index reachability** — every `.md` under `docs/` is reachable from
//!    `docs/README.md` by following relative links. A file nobody links to is
//!    a file nobody reads.
//! 2. **Relative links resolve** — every relative markdown link in the docs
//!    and in the four root documents points at a path that exists.
//! 3. **ADR numbering** — `docs/adr/NNNN-*.md` numbers are contiguous from
//!    0001 and every one of them is listed in `docs/adr/README.md`.
//! 4. **Code → docs references resolve** — a Rust comment that names a docs
//!    markdown path must name one that exists. Four such references were
//!    dangling when this landed: the cosmos arena layout map (cited from two
//!    places) and two evidence pages named from test-file headers.
//!
//! Deliberately *not* checked: prose accuracy, stale numbers inside a page,
//! and whether a document is up to date. Those need a human. This module only
//! proves the graph is connected and the pointers are not broken.

#[cfg(test)]
mod tests {
    use std::collections::{BTreeMap, BTreeSet, VecDeque};
    use std::fs;
    use std::path::{Path, PathBuf};

    /// Project root, resolved from `crates/mps-test`.
    fn project_root() -> PathBuf {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        manifest
            .parent()
            .and_then(|p| p.parent())
            .expect("crates/mps-test has two ancestors")
            .to_path_buf()
    }

    /// Collect every `*.md` under `dir`, recursively.
    fn collect_md(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(entries) = fs::read_dir(dir) else {
            return;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                collect_md(&path, out);
            } else if path.extension().and_then(|s| s.to_str()) == Some("md") {
                out.push(path);
            }
        }
    }

    /// Collect every `*.rs` under `dir`, recursively.
    fn collect_rs(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(entries) = fs::read_dir(dir) else {
            return;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                // The vendored fork is a separate workspace with its own docs
                // discipline (ADR-0008) and its own build artifacts.
                if path.file_name().and_then(|n| n.to_str()) == Some("rapier") {
                    continue;
                }
                collect_rs(&path, out);
            } else if path.extension().and_then(|s| s.to_str()) == Some("rs") {
                out.push(path);
            }
        }
    }

    /// One markdown link target: `(text)`, `(</path>)` and title suffixes are
    /// reduced to the bare relative path. Returns `None` for absolute URLs and
    /// pure in-page anchors.
    fn link_target(raw: &str) -> Option<String> {
        let t = raw.trim();
        let t = t
            .strip_prefix('<')
            .and_then(|r| r.strip_suffix('>'))
            .unwrap_or(t);
        // Drop a title: `(path "title")`.
        let t = t.split_whitespace().next().unwrap_or(t);
        // Drop the fragment.
        let t = t.split('#').next().unwrap_or(t);
        if t.is_empty() {
            return None;
        }
        if t.starts_with("http://")
            || t.starts_with("https://")
            || t.starts_with("mailto:")
            || t.starts_with('/')
        {
            return None;
        }
        Some(t.to_string())
    }

    /// Extract every link target **inside a code span excluded** from the
    /// supplied markdown.
    ///
    /// Rust-signature lines like `` `foo[_flag](world, ...)` `` are not links,
    /// so backtick spans and fenced blocks are stripped first. Without that,
    /// the checker invents two false positives (one per `[_flag](` occurrence
    /// in the spaceflight tours) and starts failing for the wrong reason.
    fn markdown_links(text: &str) -> Vec<String> {
        let stripped = strip_code_spans(text);
        let bytes: Vec<char> = stripped.chars().collect();
        let mut out = Vec::new();
        let mut i = 0;
        while i + 1 < bytes.len() {
            if bytes[i] == ']' && bytes[i + 1] == '(' {
                // Walk forward to the matching ')' (nested parens allowed).
                let mut j = i + 2;
                let mut depth = 1;
                let mut target = String::new();
                while j < bytes.len() {
                    match bytes[j] {
                        '(' => depth += 1,
                        ')' => {
                            depth -= 1;
                            if depth == 0 {
                                break;
                            }
                        }
                        c => target.push(c),
                    }
                    j += 1;
                }
                if let Some(t) = link_target(&target) {
                    out.push(t);
                }
                i = j;
            }
            i += 1;
        }
        out
    }

    /// Blank out fenced blocks and inline code spans, preserving line breaks so
    /// line numbers still line up for diagnostics.
    fn strip_code_spans(text: &str) -> String {
        let mut out = String::with_capacity(text.len());
        let mut fence: Option<String> = None;
        for line in text.split_inclusive('\n') {
            let trimmed = line.trim_start();
            match &fence {
                Some(marker) => {
                    // Inside a fence: emit a newline only.
                    if trimmed.starts_with(marker.as_str()) {
                        fence = None;
                    }
                    out.push('\n');
                    continue;
                }
                None => {
                    for m in ["```", "~~~"] {
                        if trimmed.starts_with(m) {
                            fence = Some(m.to_string());
                            break;
                        }
                    }
                    if fence.is_some() {
                        out.push('\n');
                        continue;
                    }
                }
            }
            // Inline code spans: drop everything between paired backticks.
            let mut in_code = false;
            for ch in line.chars() {
                if ch == '`' {
                    in_code = !in_code;
                    continue;
                }
                out.push(if in_code { ' ' } else { ch });
            }
        }
        out
    }

    /// Resolve `target` relative to the directory holding `from`, normalising
    /// `..`/`.` without touching the filesystem.
    fn resolve(from_file: &Path, target: &str) -> PathBuf {
        let mut base: Vec<String> = from_file
            .parent()
            .map(|p| {
                p.components()
                    .map(|c| c.as_os_str().to_string_lossy().to_string())
                    .collect()
            })
            .unwrap_or_default();
        for part in target.split('/') {
            match part {
                "" | "." => {}
                ".." => {
                    base.pop();
                }
                other => base.push(other.to_string()),
            }
        }
        let mut path = PathBuf::new();
        for part in base {
            path.push(part);
        }
        path
    }

    /// Check 1: every `.md` under `docs/` is reachable from `docs/README.md`.
    ///
    /// Unreachable pages are the failure this test exists for: the reader who
    /// follows the documented entry point never learns they exist.
    #[test]
    fn every_doc_page_is_reachable_from_the_docs_index() {
        let root = project_root();
        let docs = root.join("docs");
        let entry = docs.join("README.md");

        let mut files: Vec<PathBuf> = Vec::new();
        collect_md(&docs, &mut files);

        let mut reachable: BTreeSet<PathBuf> = BTreeSet::new();
        let mut queue: VecDeque<PathBuf> = VecDeque::new();
        let Ok(entry_text) = fs::read_to_string(&entry) else {
            panic!("docs/README.md is the documented entry point but is unreadable");
        };
        reachable.insert(entry.clone());
        queue.push_back(entry);

        let mut pending_text: BTreeMap<PathBuf, String> = BTreeMap::new();
        pending_text.insert(queue[0].clone(), entry_text);

        while let Some(current) = queue.pop_front() {
            let text = match pending_text.remove(&current) {
                Some(t) => t,
                None => match fs::read_to_string(&current) {
                    Ok(t) => t,
                    Err(_) => continue,
                },
            };
            for target in markdown_links(&text) {
                let resolved = resolve(&current, &target);
                // Only follow links that stay inside `docs/`.
                if !resolved.starts_with(&docs) || !resolved.is_file() {
                    continue;
                }
                if resolved.extension().and_then(|s| s.to_str()) != Some("md") {
                    continue;
                }
                if reachable.insert(resolved.clone()) {
                    queue.push_back(resolved);
                }
            }
        }

        let mut unreachable: Vec<String> = files
            .iter()
            .filter(|p| !reachable.contains(*p))
            .map(|p| {
                p.strip_prefix(&root)
                    .unwrap_or(p)
                    .display()
                    .to_string()
                    .replace('\\', "/")
            })
            .collect();
        unreachable.sort();

        assert!(
            unreachable.is_empty(),
            "docs pages that no index links to — a reader following \
             docs/README.md never finds them: {unreachable:#?}\n  \
             Fix by adding a row/link to the index that owns them \
             (docs/README.md for a new section, docs/adr/README.md for an ADR, \
             docs/evidence/README.md for an evidence record, docs/mps-core.md \
             for a per-source-file tour). Do not delete the page to make this \
             pass unless it is genuinely obsolete."
        );
    }

    /// Check 2: every relative markdown link in the docs and the root
    /// documents points at an existing path.
    #[test]
    fn every_relative_doc_link_resolves() {
        let root = project_root();
        let mut files: Vec<PathBuf> = Vec::new();
        collect_md(&root.join("docs"), &mut files);
        for name in ["AGENTS.md", "README.md", "DESIGN.md", "OPTIMIZATION.md"] {
            let p = root.join(name);
            if p.is_file() {
                files.push(p);
            }
        }

        let mut broken: Vec<String> = Vec::new();
        for file in &files {
            let Ok(text) = fs::read_to_string(file) else {
                continue;
            };
            for (idx, line) in text.lines().enumerate() {
                // Cheap filter: only lines that can contain a link.
                if !line.contains("](") {
                    continue;
                }
                for target in markdown_links(line) {
                    let resolved = resolve(file, &target);
                    if resolved.exists() {
                        continue;
                    }
                    let rel = file
                        .strip_prefix(&root)
                        .unwrap_or(file)
                        .display()
                        .to_string()
                        .replace('\\', "/");
                    broken.push(format!("{rel}:{} -> {target}", idx + 1));
                }
            }
        }

        assert!(
            broken.is_empty(),
            "relative links that do not resolve (path is missing): {broken:#?}\n  \
             Fix the link, or create the page if the link documented something \
             that should exist."
        );
    }

    /// Check 3: `docs/adr/NNNN-*.md` numbering is contiguous from 0001 and
    /// every ADR appears in `docs/adr/README.md`.
    #[test]
    fn adr_numbers_are_contiguous_and_indexed() {
        let root = project_root();
        let adr_dir = root.join("docs/adr");
        let index = adr_dir.join("README.md");
        let Ok(index_text) = fs::read_to_string(&index) else {
            panic!("docs/adr/README.md is missing — the ADR index is the only way in");
        };

        let mut numbers: Vec<u32> = Vec::new();
        let mut not_indexed: Vec<String> = Vec::new();
        let Ok(entries) = fs::read_dir(&adr_dir) else {
            panic!("docs/adr is unreadable");
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().and_then(|s| s.to_str()) != Some("md") {
                continue;
            }
            let name = path
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or_default()
                .to_string();
            if name == "README.md" {
                continue;
            }
            let Some(prefix) = name.get(0..4) else {
                panic!("ADR file {name:?} is not named NNNN-title.md");
            };
            let Ok(number) = prefix.parse::<u32>() else {
                panic!("ADR file {name:?} does not start with a 4-digit number");
            };
            numbers.push(number);
            if !index_text.contains(&name) {
                not_indexed.push(name);
            }
        }
        numbers.sort_unstable();

        assert!(
            !numbers.is_empty(),
            "no ADR files found under docs/adr — did the directory move?"
        );
        for (i, n) in numbers.iter().enumerate() {
            let expected = i as u32 + 1;
            assert_eq!(
                *n, expected,
                "ADR numbering has a gap or duplicate at {n:04} (expected {expected:04}). \
                 docs/adr/README.md says numbers are contiguous and never reused."
            );
        }
        assert!(
            not_indexed.is_empty(),
            "ADR files absent from docs/adr/README.md's table: {not_indexed:#?}\n  \
             An unindexed ADR is invisible to anyone asking \"was this already \
             decided?\", which is exactly how a frozen ABI gets re-litigated."
        );
    }

    /// Check 4: a Rust comment that names a docs path must name a real one.
    ///
    /// The dangling pointers this caught when it landed were the cosmos arena
    /// map note (in `mps-cosmos/src/arena.rs` and the cosmos FFI mirror test),
    /// plus two evidence pages referenced by name from test-file headers. A
    /// comment pointing at a missing page is worse than no comment: it reads as
    /// "the contract is written down somewhere" when it is not.
    #[test]
    fn every_doc_path_named_in_a_rust_comment_exists() {
        let root = project_root();
        let mut files: Vec<PathBuf> = Vec::new();
        collect_rs(&root.join("crates"), &mut files);

        // Prefixes that appear bare in comments (without the `docs/` part).
        const BARE_PREFIXES: &[&str] = &["evidence/", "adr/", "integration/", "mps-core/"];

        let mut broken: Vec<String> = Vec::new();
        for file in &files {
            let Ok(text) = fs::read_to_string(file) else {
                continue;
            };
            for (idx, line) in text.lines().enumerate() {
                if !line.contains(".md") {
                    continue;
                }
                let trimmed = line.trim_start();
                // Only comment lines carry these pointers.
                if !(trimmed.starts_with("//")
                    || trimmed.contains("///")
                    || trimmed.contains("//!"))
                {
                    continue;
                }
                for token in md_path_tokens(line) {
                    let candidate = if token.starts_with("docs/") {
                        root.join(&token)
                    } else if BARE_PREFIXES.iter().any(|p| token.starts_with(p)) {
                        root.join("docs").join(&token)
                    } else {
                        continue;
                    };
                    if candidate.is_file() {
                        continue;
                    }
                    let rel = file
                        .strip_prefix(&root)
                        .unwrap_or(file)
                        .display()
                        .to_string()
                        .replace('\\', "/");
                    broken.push(format!("{rel}:{} -> {token}", idx + 1));
                }
            }
        }
        broken.sort();
        broken.dedup();

        assert!(
            broken.is_empty(),
            "Rust comments pointing at docs that do not exist: {broken:#?}\n  \
             Fix the pointer to the page that actually holds the contract, or \
             write the page if the comment documented something real."
        );
    }

    /// Every path-shaped markdown token on a line, in source order.
    ///
    /// Hand-rolled instead of pulling in `regex`: this crate deliberately has a
    /// short dependency list, and the token shape is simple.
    fn md_path_tokens(line: &str) -> Vec<String> {
        fn is_path_char(c: char) -> bool {
            c.is_ascii_alphanumeric() || c == '_' || c == '.' || c == '/' || c == '-'
        }
        let chars: Vec<char> = line.chars().collect();
        let mut out = Vec::new();
        let mut i = 0;
        while i < chars.len() {
            if !is_path_char(chars[i]) {
                i += 1;
                continue;
            }
            let start = i;
            while i < chars.len() && is_path_char(chars[i]) {
                i += 1;
            }
            let mut run: String = chars[start..i].iter().collect();
            // Trailing sentence punctuation is not part of the path.
            while run.ends_with('.') {
                run.pop();
            }
            if run.ends_with(".md") {
                out.push(run);
            }
        }
        out
    }
}
