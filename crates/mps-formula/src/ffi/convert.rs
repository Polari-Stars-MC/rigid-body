use crate::domain::FormulaError;
use crate::math::Vector3f64;

use super::types::Vec3;

/// Convert a checked formula result at the FFI boundary. Pointer errors must
/// be handled separately. Pure Rust callers should use the original Result.
pub fn formula_result<T>(result: Result<T, crate::FormulaError>) -> Option<T> {
    match result {
        Ok(value) => {
            crate::error::clear_error();
            Some(value)
        }
        Err(error) => {
            let code = match error {
                FormulaError::NullPointer { .. } => crate::error::ERR_NULL_POINTER,
                FormulaError::InvalidLength { .. } | FormulaError::CapacityOverflow { .. } => {
                    crate::error::ERR_CAPACITY
                }
                FormulaError::MisalignedPointer { .. } => crate::error::ERR_INVALID_ARGUMENT,
                _ => crate::error::ERR_INVALID_ARGUMENT,
            };
            crate::error::set_error(code, &error.to_string());
            None
        }
    }
}

/// # Safety
/// `ptr` must point to `len` readable initialized values and remain valid for
/// the returned lifetime; the allocation must not be concurrently mutated.
pub unsafe fn checked_input_slice<'a, T>(
    ptr: *const T,
    len: usize,
    parameter: &'static str,
) -> Result<&'a [T], FormulaError> {
    if ptr.is_null() {
        return Err(FormulaError::NullPointer { parameter });
    }
    if !ptr.is_aligned() {
        return Err(FormulaError::MisalignedPointer { parameter });
    }
    let bytes = len
        .checked_mul(std::mem::size_of::<T>())
        .ok_or(FormulaError::CapacityOverflow { parameter })?;
    if bytes > isize::MAX as usize || (ptr as usize).checked_add(bytes).is_none() {
        return Err(FormulaError::CapacityOverflow { parameter });
    }
    Ok(unsafe { std::slice::from_raw_parts(ptr, len) })
}

/// # Safety
/// `ptr` must point to `len` writable values and remain valid for the returned
/// lifetime; no aliased access may occur concurrently.
pub unsafe fn checked_output_slice<'a, T>(
    ptr: *mut T,
    len: usize,
    parameter: &'static str,
) -> Result<&'a mut [T], FormulaError> {
    if ptr.is_null() {
        return Err(FormulaError::NullPointer { parameter });
    }
    if !ptr.is_aligned() {
        return Err(FormulaError::MisalignedPointer { parameter });
    }
    let bytes = len
        .checked_mul(std::mem::size_of::<T>())
        .ok_or(FormulaError::CapacityOverflow { parameter })?;
    if bytes > isize::MAX as usize || (ptr as usize).checked_add(bytes).is_none() {
        return Err(FormulaError::CapacityOverflow { parameter });
    }
    Ok(unsafe { std::slice::from_raw_parts_mut(ptr, len) })
}

pub fn checked_product(a: usize, b: usize, parameter: &'static str) -> Result<usize, FormulaError> {
    a.checked_mul(b)
        .ok_or(FormulaError::CapacityOverflow { parameter })
}

pub fn checked_product3(
    a: usize,
    b: usize,
    c: usize,
    parameter: &'static str,
) -> Result<usize, FormulaError> {
    checked_product(checked_product(a, b, parameter)?, c, parameter)
}

pub fn vec3_to_rapier(value: Vec3) -> Vector3f64 {
    Vector3f64::new(value.x, value.y, value.z)
}

pub fn vec3_finite(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

pub fn vec3_from_rapier(value: Vector3f64) -> Vec3 {
    Vec3 {
        x: value.x,
        y: value.y,
        z: value.z,
    }
}
