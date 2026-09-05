//! JNI byte-array → Rust `Vec<u8>` 转换 helper。仅 voxel builder 调用，
//! 不在每 tick 热路径——但单次 voxel 构造（128³ ≈ 2 MB）下原实装有 3 遍
//! 拷贝 / 分配，chunk gen 阶段 GC 暴冲概率高，故做一次合并拷贝优化
//! （见 `性能分析.MD` §12.5 / L5）。
//!
//! 优化点：
//! 1. 取消 `vec![0i8; len]` 的零填充——`get_byte_array_region` 写满整个
//!    缓冲，零写是无用功；
//! 2. 取消 `.into_iter().map(|v| v as u8).collect()` 第三遍 alloc ——直接
//!    在 `Vec<u8>` 上 transmute 切片喂给 `get_byte_array_region`。`i8` 与
//!    `u8` 单字节布局相同，`get_byte_array_region` 写入的字节即 Java 端 `byte`，
//!    按 `u8` 读取只是重新解释，零成本。
//!
//! 使用 `MaybeUninit<u8>` 容器，避免 `vec![0i8; len]` 的 memset + clippy
//! `uninit_vec` lint 兼容。`get_byte_array_region` 成功后 `assume_init`。

use std::mem::{ManuallyDrop, MaybeUninit};

use ljni::JNIEnv;
use ljni::objects::{JByteArray, JDoubleArray};
use ljni::sys::{jbyteArray, jdoubleArray};

/// 把一个 Java `byte[]` 拷贝到 `Vec<u8>`，零填充 + 单次分配。
///
/// 实装：`Vec<MaybeUninit<u8>>::with_capacity(len)` 不 touch 已分配字节，
/// `get_byte_array_region` 直接将 Java 端字节写入 uninit 缓冲（按 i8 视图），
/// 成功后 `assume_init()` 升级为 `Vec<u8>`。失败 return None，不泄露 uninit。
pub fn jbytearray_to_array(env: &JNIEnv, data: jbyteArray) -> Option<Vec<u8>> {
    if data.is_null() {
        return None;
    }

    // SAFETY: JByteArray 只是为 `data` 提供本地引用包装；用 ManuallyDrop 避免
    //   本地 ref 被 drop（JNIEnv frame 仍持有）。
    let data = unsafe { JByteArray::from_raw(data) };
    // 用 ManuallyDrop 以免 from_raw 构造的 JByteArray 被 drop——JNI 局部 ref
    // 由上层 Java frame 管理，我们不应主动释放。
    let data = ManuallyDrop::new(data);

    let len = env.get_array_length(&*data).ok()? as usize;

    // 单次分配 uninit 缓冲（覆 `vec![0i8; len]` 的零填充）。
    let mut uninit: Vec<MaybeUninit<u8>> = Vec::with_capacity(len);
    // SAFETY: 也许 uninit 是 'unfilled capacity'，但 MaybeUninit<u8> 本身就是
    //   'uninit' 容器，set_len 后 Vec 内全是 MaybeUninit 元素，不允许 caller 读
    //   'assume_init' 之前的 byte 内容。仅在 get_byte_array_region 成功写满
    //   之后执行 assume_init 升级到 Vec<u8>。Vec<MaybeUninit<u8>> 的 Drop 不会
    //   读 uninit 字节（MaybeUninit<u8>::drop 是 nop），所以即使失败 return None
    //   也没有 UB。
    unsafe { uninit.set_len(len) };

    // SAFETY: `&mut [MaybeUninit<u8>]` 与 `&mut [u8]` 内存布局相同；进一步
    //   `&mut [u8]` 与 `&mut [i8]` 相同字节数，每个元素单字节布局。在此做
    //   `*mut MaybeUninit<u8> -> *mut i8` 指针 transmute，喂给
    //   `get_byte_array_region`（JNI 会写入 len 个 i8 字节到 buf 中）。
    //   写入的字节完全覆盖原先 uninit 部分；成功后我们立即 `assume_init`，所以
    //   'uninit bytes no longer observable'。
    let i8_slice: &mut [i8] =
        unsafe { std::slice::from_raw_parts_mut(uninit.as_mut_ptr() as *mut i8, len) };
    match env.get_byte_array_region(&*data, 0, i8_slice) {
        Ok(()) => {
            // SAFETY: `get_byte_array_region` 已为每个 slot 写入 i8 字节，因此
            //   衡量上每个 MaybeUninit<u8> 都被 init 成有效 u8 值。我们把
            //   Vec<MaybeUninit<u8>> 的 raw parts 直接重新解释成 Vec<u8>——
            //   `MaybeUninit<u8>` 与 `u8` 单字节、相同 align，`Vec` 的 layout
            //   仅取决于元素 size/align，故三联组 (ptr, len, cap) safe to
            //   reinterpret。`std::mem::forget(uninit)` 防止 MaybeUninit 版本的
            //   Drop 释放同一 allocation（double-free）。
            let ptr = uninit.as_mut_ptr() as *mut u8;
            let cap = uninit.capacity();
            let len_uninit = uninit.len();
            std::mem::forget(uninit);
            Some(unsafe { Vec::from_raw_parts(ptr, len_uninit, cap) })
        }
        Err(_) => None,
    }
}

/// 把一个 Java `double[]` 拷贝到 `Vec<f64>`（CompoundBoxesArray 等用）。
pub fn jdoublearray_to_array(env: &JNIEnv, data: jdoubleArray) -> Option<Vec<f64>> {
    if data.is_null() {
        return None;
    }

    let data = ManuallyDrop::new(unsafe { JDoubleArray::from_raw(data) });
    let len = env.get_array_length(&*data).ok()? as usize;
    let mut buf = vec![0.0f64; len];
    env.get_double_array_region(&*data, 0, &mut buf).ok()?;
    Some(buf)
}
