//! Thread-local last-error slot shared with `mps-formula` / `mps-core`.
//!
//! The slot itself lives in `mps_formula::error` (the lowest-level crate), so
//! errors written by formula code and by the cosmos FFI layer land in the same
//! place and are both visible to Java through the `last_error_*` ABI. This
//! module re-exports the shared implementation and adds the `ERR_*` codes as
//! **local literals**.
//!
//! The error codes are re-declared as local `pub const`s (instead of `pub use`
//! re-exports) so cbindgen — which only parses this crate, not dependencies —
//! emits them as `#define`s in the generated `include/cosmos.h`, keeping the
//! header self-contained (matching `rigid_body.h`). The values must be
//! literals (cbindgen cannot evaluate cross-crate paths); the compile-time
//! assertions below pin them to the canonical values in `mps_formula::error`.

pub use mps_formula::error::{clear_error, set_error};

pub const ERR_OK: u32 = 0;
pub const ERR_NULL_POINTER: u32 = 1;
pub const ERR_INVALID_ARGUMENT: u32 = 2;
pub const ERR_NOT_FOUND: u32 = 3;
pub const ERR_CAPACITY: u32 = 4;
pub const ERR_UNSUPPORTED: u32 = 5;
pub const ERR_INTERNAL: u32 = 6;

const _: () = {
    assert!(ERR_OK == mps_formula::error::ERR_OK);
    assert!(ERR_NULL_POINTER == mps_formula::error::ERR_NULL_POINTER);
    assert!(ERR_INVALID_ARGUMENT == mps_formula::error::ERR_INVALID_ARGUMENT);
    assert!(ERR_NOT_FOUND == mps_formula::error::ERR_NOT_FOUND);
    assert!(ERR_CAPACITY == mps_formula::error::ERR_CAPACITY);
    assert!(ERR_UNSUPPORTED == mps_formula::error::ERR_UNSUPPORTED);
    assert!(ERR_INTERNAL == mps_formula::error::ERR_INTERNAL);
};
