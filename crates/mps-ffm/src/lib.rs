use mps_core::rapier::ffi::Bool;

pub const ABI_VERSION: u32 = 1;

#[unsafe(no_mangle)]
pub extern "C" fn abi_version() -> u32 {
    ABI_VERSION
}

#[unsafe(no_mangle)]
pub extern "C" fn abi_supports_ffm() -> Bool {
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn abi_supports_jni() -> Bool {
    Bool::TRUE
}
