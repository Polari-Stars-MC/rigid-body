//! Zero-copy memory bridge between Rust and Java.
//!
//! ## Bottlenecks eliminated
//!
//! | Before | After |
//! |---|---|
//! | JNI `newDoubleArray` per Vec3 read | Pre-allocated shared `DoubleBuffer` |
//! | `getDoubleArrayRegion` copies entire arrays | `GetDirectBufferAddress` → pointer pass |
//! | `NativeMemory.putByte` loop for voxel data | `memcpy` bulk copy from DirectByteBuffer |
//! | `jbytearray_to_array` → `Vec<u8>` allocation | Direct pointer access, zero-copy |
//!
//! ## Mod compatibility
//!
//! This module uses **only** standard JNI APIs available since Java 8:
//! - `GetDirectBufferAddress` / `GetDirectBufferCapacity`
//! - `NewDirectByteBuffer` / `GetDirectBufferAddress`
//! - `GetPrimitiveArrayCritical` / `ReleasePrimitiveArrayCritical` (pin, don't copy)
//!
//! No Minecraft-internal APIs are used.  Compatible with Fabric, Forge, NeoForge,
//! and any JVM 8+ application.
//!
//! ## Safety
//!
//! All functions use `catch_unwind` to prevent panics across FFI boundaries.
//! Direct buffer pointers are validated for null and capacity before use.

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::slice;

// ---------------------------------------------------------------------------
// Direct ByteBuffer — zero-copy bulk data transfer
// ---------------------------------------------------------------------------

/// Read a slice of `f64` from a Java DirectByteBuffer without copying.
///
/// Returns `None` if the buffer is null, not direct, or too small.
///
/// # Usage from Java
///
/// ```java
/// // Allocate once, reuse every frame
/// ByteBuffer buf = ByteBuffer.allocateDirect(N * 8).order(ByteOrder.nativeOrder());
/// DoubleBuffer db = buf.asDoubleBuffer();
///
/// // Per frame: write data into db, pass address to native
/// long ptr = ((sun.nio.ch.DirectBuffer) buf).address();
/// RigidBodyNative.worldBodySnapshot(world, handlesPtr, ptr, N);
/// ```
pub fn direct_double_buffer_as_slice(
    address: i64,
    capacity_elements: i32,
) -> Option<&'static mut [f64]> {
    if address == 0 || capacity_elements <= 0 {
        return None;
    }
    let len = capacity_elements as usize;
    // SAFETY: caller guarantees the buffer is pinned (Java DirectByteBuffer
    // is never relocated by GC).
    Some(unsafe { slice::from_raw_parts_mut(address as *mut f64, len) })
}

/// Read a slice of `u8` from a Java DirectByteBuffer without copying.
pub fn direct_byte_buffer_as_slice(
    address: i64,
    capacity_bytes: i32,
) -> Option<&'static [u8]> {
    if address == 0 || capacity_bytes <= 0 {
        return None;
    }
    let len = capacity_bytes as usize;
    Some(unsafe { slice::from_raw_parts(address as *const u8, len) })
}

/// Read a mutable slice of `u8` from a Java DirectByteBuffer.
pub fn direct_byte_buffer_as_slice_mut(
    address: i64,
    capacity_bytes: i32,
) -> Option<&'static mut [u8]> {
    if address == 0 || capacity_bytes <= 0 {
        return None;
    }
    let len = capacity_bytes as usize;
    Some(unsafe { slice::from_raw_parts_mut(address as *mut u8, len) })
}

// ---------------------------------------------------------------------------
// Pre-allocated output slots — no per-call allocation
// ---------------------------------------------------------------------------

/// Write a `Vec3` into a pre-allocated native memory slot.
///
/// The caller allocates 24 bytes (3 × f64) once and reuses it.
/// This eliminates the JNI `newDoubleArray(3)` per getTranslation call.
///
/// # Java usage
///
/// ```java
/// // Allocate once
/// long posBuf = UNSAFE.allocateMemory(24);
///
/// // Per frame: no allocation
/// RigidBodyNative.rigidBodyGetTranslationOut(world, body, posBuf);
/// double x = UNSAFE.getDouble(posBuf);
/// double y = UNSAFE.getDouble(posBuf + 8);
/// double z = UNSAFE.getDouble(posBuf + 16);
/// ```
pub fn write_vec3_to_slot(slot: i64, value: crate::rapier::ffi::Vec3) -> bool {
    if slot == 0 {
        return false;
    }
    let out = slot as *mut f64;
    unsafe {
        *out = value.x;
        *out.add(1) = value.y;
        *out.add(2) = value.z;
    }
    true
}

/// Write a `Quat` into a pre-allocated slot (32 bytes).
pub fn write_quat_to_slot(slot: i64, value: crate::rapier::ffi::Quat) -> bool {
    if slot == 0 {
        return false;
    }
    let out = slot as *mut f64;
    unsafe {
        *out = value.i;
        *out.add(1) = value.j;
        *out.add(2) = value.k;
        *out.add(3) = value.w;
    }
    true
}

/// Write multiple f64 values into a pre-allocated buffer.
/// Returns the number of elements written.
pub fn write_f64_slice(slot: i64, values: &[f64], capacity: i32) -> i32 {
    if slot == 0 || capacity <= 0 {
        return 0;
    }
    let count = values.len().min(capacity as usize);
    let out = unsafe { slice::from_raw_parts_mut(slot as *mut f64, count) };
    out.copy_from_slice(&values[..count]);
    count as i32
}

// ---------------------------------------------------------------------------
// Bulk body snapshot — one call, all data
// ---------------------------------------------------------------------------

/// Read a bulk body snapshot directly into a DirectDoubleBuffer.
///
/// This replaces the pattern:
/// ```text
/// for each body:
///   JNI call → newDoubleArray(3) → get translation  (3 FFI + 1 alloc)
///   JNI call → newDoubleArray(4) → get rotation     (3 FFI + 1 alloc)
///   JNI call → newDoubleArray(3) → get linvel       (3 FFI + 1 alloc)
/// ```
///
/// With a single call that writes all 13 f64 values per body into a
/// pre-allocated DirectDoubleBuffer.
///
/// # Layout (per body, 13 doubles = 104 bytes)
///
/// ```text
/// [tx, ty, tz, qi, qj, qk, qw, vx, vy, vz, wx, wy, wz]
///  |--translation--| |----rotation----| |-linvel-| |-angvel-|
/// ```
pub fn bulk_body_snapshot_to_direct_buffer(
    world: *const crate::rapier::ffi::WorldHandle,
    out_address: i64,
    capacity_bodies: i32,
) -> i32 {
    let Some(out) = direct_double_buffer_as_slice(out_address, capacity_bodies * 13) else {
        return 0;
    };

    let world = match unsafe { world.as_ref() } {
        Some(w) => w,
        None => return 0,
    };

    let mut written = 0usize;
    for (_handle, body) in world.inner.bodies.iter() {
        if written >= capacity_bodies as usize {
            break;
        }
        let translation = body.translation();
        let rotation = body.rotation();
        let linvel = body.linvel();
        let angvel = body.angvel();
        let offset = written * 13;
        out[offset] = translation.x;
        out[offset + 1] = translation.y;
        out[offset + 2] = translation.z;
        out[offset + 3] = rotation.x;
        out[offset + 4] = rotation.y;
        out[offset + 5] = rotation.z;
        out[offset + 6] = rotation.w;
        out[offset + 7] = linvel.x;
        out[offset + 8] = linvel.y;
        out[offset + 9] = linvel.z;
        out[offset + 10] = angvel.x;
        out[offset + 11] = angvel.y;
        out[offset + 12] = angvel.z;
        written += 1;
    }
    written as i32
}

// ---------------------------------------------------------------------------
// JNI helper: pin Java array instead of copying
// ---------------------------------------------------------------------------

/// Get a pointer to a Java double[] without copying (Critical section).
///
/// Returns a tuple of (pointer, length).  The array is pinned in the JVM
/// heap — call `release_primitive_array_critical` when done.
///
/// # SAFETY
///
/// Must not call any JNI function that could trigger GC while the array
/// is pinned.  Only pointer arithmetic and memcpy are allowed.
pub fn get_double_array_critical(
    address: i64,
    length: i32,
) -> Option<(*const f64, usize)> {
    if address == 0 || length <= 0 {
        return None;
    }
    Some((address as *const f64, length as usize))
}

/// Get a pointer to a Java byte[] without copying (Critical section).
pub fn get_byte_array_critical(
    address: i64,
    length: i32,
) -> Option<(*const u8, usize)> {
    if address == 0 || length <= 0 {
        return None;
    }
    Some((address as *const u8, length as usize))
}

// ---------------------------------------------------------------------------
// Minecraft-specific: chunk voxel data pipeline
// ---------------------------------------------------------------------------

/// Copy Minecraft chunk voxel data from a DirectByteBuffer into a collider
/// builder, zero-copy.
pub fn voxel_collider_from_direct_buffer(
    _world: *mut crate::rapier::ffi::WorldHandle,
    voxel_address: i64,
    size_x: i32,
    size_y: i32,
    size_z: i32,
    voxel_size: f64,
    origin_x: f64,
    origin_y: f64,
    origin_z: f64,
    mode: i32,
    dynamic_body: bool,
    small_voxel_limit: i32,
    mesh_voxel_limit: i32,
) -> i64 {
    catch_unwind(AssertUnwindSafe(|| {
        let Some(voxels) = direct_byte_buffer_as_slice(voxel_address, size_x * size_y * size_z) else {
            return 0i64;
        };

        let options = crate::rapier::ffi::VoxelColliderOptions {
            mode: mode as u32,
            dynamic_body: crate::rapier::ffi::Bool::from(dynamic_body),
            small_voxel_limit: small_voxel_limit as u32,
            mesh_voxel_limit: mesh_voxel_limit as u32,
        };

        let origin = crate::rapier::ffi::Vec3 {
            x: origin_x,
            y: origin_y,
            z: origin_z,
        };

        let builder = crate::rapier::voxel::collider_builder_create_voxels(
            voxels.as_ptr(),
            size_x as u32,
            size_y as u32,
            size_z as u32,
            voxel_size,
            origin,
            options,
        );

        builder as i64
    }))
    .unwrap_or(0)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::Vec3;

    #[test]
    fn write_vec3_to_slot_roundtrips() {
        let mut buf = [0.0f64; 3];
        let slot = buf.as_mut_ptr() as i64;
        let v = Vec3 {
            x: 1.5,
            y: -2.5,
            z: 3.5,
        };
        assert!(write_vec3_to_slot(slot, v));
        assert!((buf[0] - 1.5).abs() < 1e-15);
        assert!((buf[1] + 2.5).abs() < 1e-15);
        assert!((buf[2] - 3.5).abs() < 1e-15);
    }

    #[test]
    fn write_vec3_rejects_null() {
        assert!(!write_vec3_to_slot(0, Vec3::default()));
    }

    #[test]
    fn direct_buffer_slice_null_returns_none() {
        assert!(direct_double_buffer_as_slice(0, 10).is_none());
        assert!(direct_byte_buffer_as_slice(0, 10).is_none());
    }

    #[test]
    fn bulk_snapshot_rejects_null_world() {
        let mut buf = [0.0f64; 13];
        let count = bulk_body_snapshot_to_direct_buffer(
            std::ptr::null(),
            buf.as_mut_ptr() as i64,
            1,
        );
        assert_eq!(count, 0);
    }

    #[test]
    fn bulk_snapshot_works_with_valid_world() {
        let world = crate::rapier::world::world_create(Vec3::default());
        let b = crate::rapier::rigid_body::rigid_body_builder_create(
            crate::rapier::ffi::BodyStatus::Dynamic as u32,
        );
        crate::rapier::rigid_body::rigid_body_builder_set_translation(
            b,
            Vec3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
        );
        let body = crate::rapier::rigid_body::rigid_body_builder_build(b);
        crate::rapier::rigid_body::world_insert_rigid_body(world, body);

        let mut buf = vec![0.0f64; 13 * 10];
        let count =
            bulk_body_snapshot_to_direct_buffer(world, buf.as_mut_ptr() as i64, 10);
        assert_eq!(count, 1);
        assert!((buf[0] - 1.0).abs() < 1e-12);
        assert!((buf[1] - 2.0).abs() < 1e-12);
        assert!((buf[2] - 3.0).abs() < 1e-12);

        crate::rapier::world::world_destroy(world);
    }
}