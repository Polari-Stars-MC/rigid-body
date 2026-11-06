pub mod convert;
pub mod force_queue;
pub mod types;

pub use convert::*;
pub use force_queue::*;
pub use types::*;

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ERR_OK, clear_error, set_error,
};

/// Common body for the *pure calculator* FFI wrappers.
///
/// Given an `out` pointer (which may be null) and a thunk that computes the
/// `Option<f64>` result of a `mps_formula` helper, this writes the value into
/// `out` on `Some` and returns `Bool::TRUE`, or returns `Bool::FALSE` when the
/// pointer is null or the computation yields `None` (invalid input).
///
/// Behaviour is identical to the hand-written expansion previously duplicated
/// across every scalar FFI function (see `matmech.rs` / `thermo.rs`):
/// null `out` → `FALSE` without writing; `None` → `FALSE` without writing.
///
/// Every rejection also reports through the thread-local error slot. Without
/// this the ~180 scalar wrappers returned `FALSE` while leaving
/// `last_error_code()` at `ERR_OK` (or at a stale error from an unrelated
/// earlier call), so a host could not tell a rejected input from a bug.
/// A specific reason already written by the formula layer is preserved; the
/// generic message is only used when the slot is still clean.
pub(crate) fn ffi_scalar<F>(out: *mut f64, f: F) -> Bool
where
    F: FnOnce() -> Option<f64>,
{
    if out.is_null() {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return Bool::FALSE;
    }
    match f() {
        Some(v) => {
            unsafe { *out = v };
            clear_error();
            Bool::TRUE
        }
        None => {
            if mps_formula::error::error_code() == ERR_OK {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "input rejected by the formula domain checks",
                );
            }
            Bool::FALSE
        }
    }
}
