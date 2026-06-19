use ljni::JNIEnv;
use ljni::objects::{JBooleanArray, JDoubleArray, JLongArray};
use ljni::sys::{jdoubleArray, jint, jlong, jlongArray, jsize};
use rapier3d::geometry::Array2;
use rapier3d::math::Vector;
use rapier3d::prelude::{ColliderBuilder};
use crate::ColliderBuilderHandle;

pub fn array_to_array2(heightmap: *const f64, x: u32, y: u32) -> Array2<f64> {
    let mut data_array2 = Array2::<f64>::zeros(x as usize, y as usize);
    for xp in 0..x {
        for yp in 0..y {
            data_array2[(xp as usize, yp as usize)] = unsafe { *heightmap.add((yp * x + xp) as usize) };
        }
    }
    data_array2
}