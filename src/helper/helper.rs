use std::mem::ManuallyDrop;

use ljni::JNIEnv;
use ljni::objects::JDoubleArray;
use ljni::sys::jdoubleArray;

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
