//! ABI lock-in test for `shared_arena` constants (OPTIMIZATION.md §10).
//!
//! The 11 constants below pin the shared-memory arena's binary layout, which
//! Java reads via `java.lang.foreign.MemorySegment` through `mps-ffm`. They
//! are **ABI-sensitive**: changing any of them silently breaks cross-language
//! access without a compile-time or runtime signal. `mps-ffm::ABI_VERSION` is
//! the companion guard — when any of these values intentionally changes,
//! bump `ABI_VERSION` so the Java side raises `IllegalStateException` on a
//! version mismatch.
//!
//! This module locks the current values with a unit test so accidental drift
//! fails CI loudly instead of producing silent memory corruption.

/// `pub` under `cfg(test)` so [`version_consistency`] (§N6) can cross-check
/// the same canonical "current" value. The mirror constants in
/// `version_consistency.rs::CURRENT_*` are asserted equal to these in a
/// third cross-test, so bumping one side without the other fails CI visibly.
#[cfg(test)]
pub const ARENA_VERSION_PIN: u32 = 2;
#[cfg(test)]
pub const ABI_VERSION_PIN: u32 = 2;

#[cfg(test)]
mod tests {
    use super::{ABI_VERSION_PIN, ARENA_VERSION_PIN};
    use mps_core::rapier::shared_arena::*;

    /// Asserts the 11 ABI-sensitive `shared_arena` constants keep their
    /// locked values. If this test fails, the arena ABI was changed
    /// breakingly — bump [`mps_ffm::ABI_VERSION`] (and the matching Java-side
    /// version check) so foreign callers fail loudly instead of reading
    /// shifted fields.
    ///
    /// The `*_PIN` constants immediately above (file-level, `cfg(test)`) are
    /// re-exported to §N6's `version_consistency` test so that this file and
    /// that file agree on the canonical "current" versions — if either side
    /// drifts, both tests fail in their own terms AND a third cross-check
    /// fails.
    ///
    /// See OPTIMIZATION.md §10 for rationale and §13 for the dead-code
    /// cleanup context.
    #[test]
    fn arena_constants_are_locked() {
        assert_eq!(
            ARENA_VERSION, ARENA_VERSION_PIN,
            "ARENA_VERSION drifted: shared_arena ABI changed — bump mps-ffm::ABI_VERSION \
             to trigger Java-side IllegalStateException 防呆"
        );
        assert_eq!(
            BODY_SLOT_STRIDE, 96,
            "BODY_SLOT_STRIDE drifted: shared_arena ABI changed — bump mps-ffm::ABI_VERSION"
        );
        assert_eq!(
            CMD_SLOT_STRIDE, 32,
            "CMD_SLOT_STRIDE drifted: shared_arena ABI changed — bump mps-ffm::ABI_VERSION"
        );
        assert_eq!(
            COLLIDER_SLOT_STRIDE, 80,
            "COLLIDER_SLOT_STRIDE drifted: shared_arena ABI changed — bump mps-ffm::ABI_VERSION"
        );
        assert_eq!(
            EVENT_SLOT_STRIDE, 64,
            "EVENT_SLOT_STRIDE drifted: shared_arena ABI changed — bump mps-ffm::ABI_VERSION"
        );
        assert_eq!(
            HEADER_SIZE, 128,
            "HEADER_SIZE drifted: shared_arena ABI changed — bump mps-ffm::ABI_VERSION"
        );
        assert_eq!(
            MAX_ARENA_BODIES, 1_000_000,
            "MAX_ARENA_BODIES drifted: shared_arena ABI changed — bump mps-ffm::ABI_VERSION"
        );
        assert_eq!(
            MAX_ARENA_COLLIDERS, 1_000_000,
            "MAX_ARENA_COLLIDERS drifted: shared_arena ABI changed — bump mps-ffm::ABI_VERSION"
        );
        assert_eq!(
            MAX_ARENA_COMMANDS, 1_000_000,
            "MAX_ARENA_COMMANDS drifted: shared_arena ABI changed — bump mps-ffm::ABI_VERSION"
        );
        assert_eq!(
            MAX_ARENA_EVENTS, 1_000_000,
            "MAX_ARENA_EVENTS drifted: shared_arena ABI changed — bump mps-ffm::ABI_VERSION"
        );
        assert_eq!(
            MAX_ARENA_TOTAL_BYTES,
            256 * 1024 * 1024,
            "MAX_ARENA_TOTAL_BYTES drifted: shared_arena ABI changed — bump mps-ffm::ABI_VERSION"
        );

        // Companion version lock (same direction as ARENA_VERSION):
        // if shared_arena::ARENA_VERSION changes, ABI_VERSION must change too.
        assert_eq!(
            mps_ffm::ABI_VERSION,
            ABI_VERSION_PIN,
            "mps-ffm::ABI_VERSION drifted — when ARENA_VERSION bumps this must bump in lockstep \
             so Java raises IllegalStateException 防呆 (OPTIMIZATION.md §10)"
        );
    }
}
