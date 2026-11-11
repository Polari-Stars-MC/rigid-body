//! Cross-crate **version-constant consistency** lock (OPTIMIZATION.md §N6).
//!
//! This is the generalisation of [`error_consistency`] (§1) and
//! [`arena_compat`] (§10): the project has more than one "abstraction-level
//! cross-crate identically-named const" pair that exists *only* because
//! cbindgen does not follow `pub use` — namely:
//!
//! | pair                                  | formula/core side                | used by (Java/foreign side) |
//! |---------------------------------------|----------------------------------|------------------------------|
//! | `ERR_*` (7 items)                     | mps-formula::error / rapier::error | indirect, header-only       |
//! | `ARENA_VERSION`                       | mps_core::rapier::shared_arena   | mps-ffm                     |
//! | `ABI_VERSION`                         | mps-ffm                          | Java-side `IllegalStateException` 防呆 |
//!
//! The two version constants (`ARENA_VERSION` and `ABI_VERSION`) are the
//! most ABI-sensitive of all: if they drift out of the contract documented
//! below without a coordinated bump, downstream Java consumers may either:
//!
//! - read shifted bytes (ARENA_VERSION bumps alone → silent data corruption)
//! - reject all increment/deserializable calls (ABI_VERSION bumps alone →
//!   Java-side `IllegalStateException` 即使 arena layout 完全没变).
//!
//! This module does **not** assert `ARENA_VERSION == ABI_VERSION` — those
//! are deliberately **distinct numbers** tracking **different concerns**:
//!
//! - `ARENA_VERSION = 2` pins the binary layout of the shared-memory ring
//!   (slot strides, header size, slot capacities).
//! - `ABI_VERSION = 2` pins the **top-level** foreign-function entry-point
//!   surface as seen by Java via `mps-ffm::abi_version()`. v2 = v1 + 纯增量
//!   探针符号（`abi_supports_formula_ffi` / `abi_formula_ffi_healthcheck` /
//!   `abi_error_slot_note`，公式层 FFM 兼容性），旧符号全部保留。
//!
//! What this module *does* enforce:
//!
//! 1. **Pin current values centrally** so a drift on either side surfaces as
//!    a single, CI-noisy failure with a precise hint.
//! 2. **Mirror the values into arena_compat.rs's existing lock** — if the
//!    two test modules disagree on the canonical "current values", CI fails.
//! 3. **Bump-path reminder**: every assertion message names the upgrade
//!    procedure so the next maintainer hits a one-line actionable hint.
//!
//! See also: `arena_compat.rs` (locks the 11 stride/cap constants + ABI_VERSION
//! ⇒ arena-layout contract).

/// `pub` under `cfg(test)` so `version_consistency` can cross-check the same
/// canonical "current" value (OPTIMIZATION.md §N6). Mirror of the same-named
/// constant in `arena_compat.rs`; the cross-test below asserts they stay equal.
#[cfg(test)]
pub const CURRENT_ARENA_VERSION: u32 = 2;
#[cfg(test)]
pub const CURRENT_ABI_VERSION: u32 = 2;

#[cfg(test)]
mod tests {
    use mps_core::rapier::shared_arena::ARENA_VERSION;
    use mps_ffm::ABI_VERSION;

    use super::{CURRENT_ABI_VERSION, CURRENT_ARENA_VERSION};

    /// Pins `mpd_core::rapier::shared_arena::ARENA_VERSION` to the value
    /// documented above. If this fails, the shared-memory arena layout was
    /// changed — the expected upgrade procedure is:
    ///
    ///   1. Update `CURRENT_ARENA_VERSION` in this file to the new value.
    ///   2. Bump `CURRENT_ABI_VERSION` (and `mpd-ffm::ABI_VERSION`) **iff**
    ///      the change is binary-incompatible with old arena bytes (e.g., a
    ///      slot stride / header size / capacity changed). A pure addition
    ///      at the tail of the header that the Java reader can ignore does
    ///      not require an ABI_VERSION bump.
    ///   3. Re-run `cargo test -p mps-test --lib arena_compat` and update
    ///      the 11 stride/capacity constants there if any changed.
    ///   4. Bump the matching Java-side `EXPECTED_ABI_VERSION` constant so
    ///      old binaries fail loudly (`IllegalStateException`) instead of
    ///      reading shifted fields.
    #[test]
    fn arena_version_is_pinned() {
        assert_eq!(
            ARENA_VERSION, CURRENT_ARENA_VERSION,
            "ARENA_VERSION drifted: the shared-memory arena layout version \
             changed. Follow the documented upgrade procedure in this \
             test's source before re-running — DO NOT silence this test by \
             bumping CURRENT_ARENA_VERSION without thinking it through."
        );
    }

    /// Pins `mpd-ffm::ABI_VERSION` to the value documented above. If this
    /// fails, the foreign-function entry-point surface changed — the
    /// expected upgrade procedure is:
    ///
    ///   1. Update `CURRENT_ABI_VERSION` in this file to the new value.
    ///   2. Bump the Java-side `EXPECTED_ABI_VERSION` constant so old
    ///      binaries raise `IllegalStateException` instead of mis-exposing
    ///      entry points that no longer exist or moved.
    ///   3. If `ARENA_VERSION` did NOT also change in this release, run
    ///      `cargo test -p mps-test --lib arena_compat` to confirm the 11
    ///      stride/capacity constants still match.
    #[test]
    fn abi_version_is_pinned() {
        assert_eq!(
            ABI_VERSION, CURRENT_ABI_VERSION,
            "ABI_VERSION drifted: the foreign-function surface version \
             changed. Follow the documented upgrade procedure in this \
             test's source before re-running — DO NOT silence this test \
             by bumping CURRENT_ABI_VERSION without thinking it through."
        );
    }

    /// Confirms `arena_compat::arena_constants_are_locked` (§10) and this
    /// module agree on the value of `ABI_VERSION`. If this fails, the two
    /// test files disagree on what "**the current ABI version**" is — pick
    /// the canonical value and edit both `CURRENT_ABI_VERSION` here and
    /// the literal in `arena_compat.rs::arena_constants_are_locked`'s
    /// final `assert_eq!` so CI gives the same hint from either direction.
    ///
    /// Why interesting: `arena_compat` lives far away from this file
    /// conceptually (it pins binary-layout constants) and its ABI_VERSION
    /// pin was written before §N6 generalised the pattern. It is easy to
    /// bump one and miss the other. This test forces coordination.
    #[test]
    fn abi_version_matches_arena_compat_pin() {
        assert_eq!(
            ABI_VERSION,
            crate::rapier::arena_compat::ABI_VERSION_PIN,
            "ABI_VERSION pin drift between version_consistency and \
             arena_compat — both must agree on CURRENT_ABI_VERSION. Pick the \
             canonical value and update either CURRENT_ABI_VERSION here and \
             the literal in arena_compat.rs::arena_constants_are_locked, or \
             the ABI_VERSION_PIN const in arena_compat.rs."
        );
    }

    /// Same as above, but for `ARENA_VERSION`. The arena_compat module
    /// already pins the 11 stride/capacity constants; this only checks the
    /// arena *version* integer is in sync between the two modules.
    #[test]
    fn arena_version_matches_arena_compat_pin() {
        assert_eq!(
            ARENA_VERSION,
            crate::rapier::arena_compat::ARENA_VERSION_PIN,
            "ARENA_VERSION pin drift between version_consistency and \
             arena_compat — both must agree on CURRENT_ARENA_VERSION. Pick \
             the canonical value and update either CURRENT_ARENA_VERSION \
             here or the ARENA_VERSION_PIN const in arena_compat.rs."
        );
    }
}
