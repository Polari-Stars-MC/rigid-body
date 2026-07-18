use std::cell::{Cell, RefCell};
use std::ffi::CString;
use std::os::raw::c_char;

thread_local! {
    static LAST_ERROR_CODE: Cell<u32> = const { Cell::new(0) };
    static LAST_ERROR_MESSAGE: RefCell<CString> = RefCell::new(CString::new("ok").expect("static string has no nul"));
}

pub(crate) const ERR_OK: u32 = 0;
pub(crate) const ERR_NULL_POINTER: u32 = 1;
pub(crate) const ERR_INVALID_ARGUMENT: u32 = 2;
pub(crate) const ERR_NOT_FOUND: u32 = 3;
pub(crate) const ERR_CAPACITY: u32 = 4;
pub(crate) const ERR_UNSUPPORTED: u32 = 5;

pub(crate) fn clear_error() {
    set_error(ERR_OK, "ok");
}

pub(crate) fn set_error(code: u32, message: &str) {
    LAST_ERROR_CODE.with(|cell| cell.set(code));
    LAST_ERROR_MESSAGE.with(|cell| {
        let sanitized = message.replace('\0', " ");
        if let Ok(value) = CString::new(sanitized) {
            *cell.borrow_mut() = value;
        }
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn last_error_code() -> u32 {
    LAST_ERROR_CODE.with(Cell::get)
}

#[unsafe(no_mangle)]
pub extern "C" fn last_error_message() -> *const c_char {
    LAST_ERROR_MESSAGE.with(|cell| cell.borrow().as_ptr())
}

#[unsafe(no_mangle)]
pub extern "C" fn last_error_clear() {
    clear_error();
}
