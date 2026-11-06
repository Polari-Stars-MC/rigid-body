//! Shared cbindgen invocation for crates that publish a C header
//! (`mps-core`'s `rigid_body.h` and `mps-cosmos`' `cosmos.h`).
//!
//! Before §7 each crate carried its own `build.rs` that differed only by a
//! single `header_path` literal and by the `cbindgen.toml` filename, which
//! forced two copies of the same cbindgen boilerplate. Both crate roots
//! declared an identical `cbindgen = "0.29.4"` build-time dependency. We
//! pull that single 22-line invocation here so:
//!
//! - The cbindgen invocation step (+ version pin + error message) lives in
//!   one place.
//! - Each crate's `build.rs` shrinks to a 3-line call that only pins down
//!   its own `cargo:rerun-if-changed` paths and its `cbindgen.toml` /
//!   `include/<header>.h` names.
//! - Crates that don't need cbindgen don't carry the build dep at all.
//!
//! Usage from a crate's `build.rs`:
//!
//! ```no_run
//! // crates/mps-core/build.rs
//! use std::path::PathBuf;
//!
//! fn main() {
//!     println!("cargo:rerun-if-changed=src");
//!     println!("cargo:rerun-if-changed=cbindgen.toml");
//!     let crate_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
//!     mps_build_common::run_cbindgen(&crate_dir, "rigid_body.h");
//! }
//! ```
//!
//! See OPTIMIZATION.md §7 for background.

use std::path::Path;

/// Invokes cbindgen for the crate located at `crate_dir`, reading its
/// `cbindgen.toml` (falling back to cbindgen defaults if the file is
/// absent), generating a header, and writing it to
/// `crate_dir/include/<header_name>`.
///
/// # Panics
///
/// Panics with a contextual message if `cbindgen.toml` cannot be read or
/// cbindgen fails to parse the crate. The panic is intentional — build
/// failures are part of the build script contract, and silence would hide
/// header-generation regressions from the CI gate.
pub fn run_cbindgen(crate_dir: &Path, header_name: &str) {
    let include_dir = crate_dir.join("include");
    std::fs::create_dir_all(&include_dir).unwrap_or_else(|err| {
        panic!(
            "mpd-build-common: failed to create {} for cbindgen output: {err}",
            include_dir.display()
        )
    });

    let header_path = include_dir.join(header_name);

    let config =
        cbindgen::Config::from_file(crate_dir.join("cbindgen.toml")).unwrap_or_else(|err| {
            // Fall back to defaults if the file is missing — but panic if
            // it exists but cannot be parsed (likely a TOML typo).
            if crate_dir.join("cbindgen.toml").is_file() {
                panic!(
                    "mpd-build-common: cbindgen.toml exists at {} but failed to parse: {err}",
                    crate_dir.join("cbindgen.toml").display()
                );
            }
            cbindgen::Config::default()
        });

    // `cbindgen::Builder::with_crate` shells out to `cargo metadata` from
    // inside this build script. That nested cargo inherits the build-script
    // environment, which cargo does *not* populate with `CARGO_NET_OFFLINE`,
    // so the nested call happily tries to download every package in the
    // resolve graph — including platform-specific ones the current target
    // never builds (e.g. `alsa` on Windows). On a machine whose cargo
    // registry cache is read-only, or with no network, header generation then
    // fails with a message that points at cbindgen instead of at the real
    // cause. Force the nested call offline: cargo has already resolved and
    // fetched everything this crate needs before running this script, so
    // offline metadata is sufficient and cannot reach the network.
    //
    // SAFETY: build scripts are single-threaded at this point and this runs
    // before any thread is spawned.
    unsafe { std::env::set_var("CARGO_NET_OFFLINE", "true") };

    cbindgen::Builder::new()
        .with_config(config)
        .with_crate(crate_dir)
        .generate()
        .unwrap_or_else(|err| {
            panic!(
                "mpd-build-common: cbindgen failed to generate {header_name} for {}: {err}",
                crate_dir.display()
            )
        })
        .write_to_file(header_path);
}
