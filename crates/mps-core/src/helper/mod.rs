use std::mem::ManuallyDrop;

use ljni::JNIEnv;
use ljni::objects::{JByteArray, JDoubleArray};
use ljni::sys::{jbyteArray, jdoubleArray};

pub fn jdoublearray_to_array(env: &JNIEnv, data: jdoubleArray) -> Option<Vec<f64>> {
    if data.is_null() {
        return None;
    }

    let data = ManuallyDrop::new(unsafe { JDoubleArray::from_raw(data) });
    let len = env.get_array_length(&*data).ok()? as usize;
    let mut buf = vec![0f64; len];
    env.get_double_array_region(&*data, 0, &mut buf).ok()?;
    Some(buf)
}

pub fn jbytearray_to_array(env: &JNIEnv, data: jbyteArray) -> Option<Vec<u8>> {
    if data.is_null() {
        return None;
    }

    let data = ManuallyDrop::new(unsafe { JByteArray::from_raw(data) });
    let len = env.get_array_length(&*data).ok()? as usize;
    let mut buf = vec![0i8; len];
    env.get_byte_array_region(&*data, 0, &mut buf).ok()?;
    Some(buf.into_iter().map(|value| value as u8).collect())
}
