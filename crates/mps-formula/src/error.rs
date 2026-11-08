use std::cell::{Cell, RefCell};
use std::ffi::CString;
use std::os::raw::c_char;
use std::panic::{AssertUnwindSafe, catch_unwind};

thread_local! {
    static LAST_ERROR_CODE: Cell<u32> = const { Cell::new(0) };
    static LAST_ERROR_MESSAGE: RefCell<CString> = RefCell::new(CString::new("ok").expect("static string has no nul"));
}

pub const ERR_OK: u32 = 0;
pub const ERR_NULL_POINTER: u32 = 1;
pub const ERR_INVALID_ARGUMENT: u32 = 2;
pub const ERR_NOT_FOUND: u32 = 3;
pub const ERR_CAPACITY: u32 = 4;
pub const ERR_UNSUPPORTED: u32 = 5;
pub const ERR_INTERNAL: u32 = 6;

pub fn clear_error() {
    set_error(ERR_OK, "ok");
}

pub fn set_error(code: u32, message: &str) {
    LAST_ERROR_CODE.with(|cell| cell.set(code));
    LAST_ERROR_MESSAGE.with(|cell| {
        let sanitized = message.replace('\0', " ");
        if let Ok(value) = CString::new(sanitized) {
            *cell.borrow_mut() = value;
        }
    });
}

/// Run `f`, converting any panic into `ERR_INTERNAL` and `default`.
///
/// Every `extern "C"` entry point in this crate wraps its body in this guard
/// so a Rust panic can never unwind across the C ABI boundary (which would
/// abort the host JVM). Same shape as `mps-core`'s `ffi_guard`
/// (`crates/mps-core/src/rapier/error.rs`) and `mps-cosmos`'s local copy
/// (`crates/mps-cosmos/src/ffi.rs`); this crate cannot depend on either, so
/// the barrier lives here next to the shared error slot. `default` is the
/// return value a caller already treats as the failure sentinel for that
/// entry point (`0` / `f64::NAN` / `Bool::FALSE` / `()`).
pub fn ffi_guard<R>(default: R, f: impl FnOnce() -> R) -> R {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(value) => value,
        Err(_) => {
            set_error(ERR_INTERNAL, "internal panic");
            default
        }
    }
}

/// Current thread's last error code (`ERR_OK` when no error).
pub fn error_code() -> u32 {
    LAST_ERROR_CODE.with(Cell::get)
}

/// Current thread's last error message as a NUL-terminated C string.
///
/// The returned pointer is owned by the thread-local slot and is invalidated
/// by the next `set_error`/`clear_error` call on the same thread.
pub fn error_message() -> *const c_char {
    LAST_ERROR_MESSAGE.with(|cell| cell.borrow().as_ptr())
}
