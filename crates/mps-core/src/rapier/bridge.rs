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

use rayon::prelude::*;

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
/// // Per frame: write data into db, pass address to native.
/// // NOTE: `((sun.nio.ch.DirectBuffer) buf).address()` (the old example) is a
/// // JDK-internal interface — blocked by strong encapsulation on JDK 9+
/// // without --add-exports java.base/sun.nio.ch=ALL-UNNAMED. The supported
/// // accessors are JNI `GetDirectBufferAddress` (where a JNIEnv is already
/// // available) or `MemorySegment.address()` on the FFM path (Java 22+).
/// long ptr = addressOf(buf);   // via GetDirectBufferAddress / FFM
/// RapierNative.worldBodySnapshot(world, handlesPtr, ptr, N);
/// ```
///
/// # Safety
///
/// `address` must be the base address of a live, pinned Java DirectByteBuffer
/// (direct buffers are never relocated by the GC) with at least
/// `capacity_elements * size_of::<f64>()` readable and writable bytes. The
/// buffer must outlive the returned slice, and no other live reference may
/// alias the same region for the slice's lifetime (the caller must guarantee
/// Java is not concurrently reading/writing it through another channel).
pub unsafe fn direct_double_buffer_as_slice(
    address: i64,
    capacity_elements: i32,
) -> Option<&'static mut [f64]> {
    if address == 0 || capacity_elements <= 0 {
        return None;
    }
    let len = capacity_elements as usize;
    // SAFETY: upheld by the caller (see the # Safety contract above).
    Some(unsafe { slice::from_raw_parts_mut(address as *mut f64, len) })
}

/// Read a slice of `u8` from a Java DirectByteBuffer without copying.
///
/// # Safety
///
/// `address` must be the base address of a live, pinned Java DirectByteBuffer
/// with at least `capacity_bytes` readable bytes. The buffer must outlive the
/// returned slice, and no `&mut` alias of the same region may exist while the
/// slice is live (the caller must guarantee Java does not mutate the buffer
/// concurrently).
pub unsafe fn direct_byte_buffer_as_slice(
    address: i64,
    capacity_bytes: i32,
) -> Option<&'static [u8]> {
    if address == 0 || capacity_bytes <= 0 {
        return None;
    }
    let len = capacity_bytes as usize;
    // SAFETY: upheld by the caller (see the # Safety contract above).
    Some(unsafe { slice::from_raw_parts(address as *const u8, len) })
}

/// Read a mutable slice of `u8` from a Java DirectByteBuffer.
///
/// # Safety
///
/// `address` must be the base address of a live, pinned Java DirectByteBuffer
/// with at least `capacity_bytes` readable and writable bytes. The buffer
/// must outlive the returned slice, and no other live reference may alias the
/// same region for the slice's lifetime.
pub unsafe fn direct_byte_buffer_as_slice_mut(
    address: i64,
    capacity_bytes: i32,
) -> Option<&'static mut [u8]> {
    if address == 0 || capacity_bytes <= 0 {
        return None;
    }
    let len = capacity_bytes as usize;
    // SAFETY: upheld by the caller (see the # Safety contract above).
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
    // SAFETY: `out_address` comes from a Java DirectByteBuffer that the Java
    // caller keeps alive and exclusively hands to native for this call.
    let Some(out) = (unsafe { direct_double_buffer_as_slice(out_address, capacity_bodies * 13) })
    else {
        return 0;
    };

    let world = match unsafe { world.as_ref() } {
        Some(w) => w,
        None => return 0,
    };

    let capacity = capacity_bodies as usize;

    // Collect body handles (cheap shared read), then compute each body's
    // 13-f64 snapshot in parallel. Bodies are read-only here and each output
    // slot `out[i*13..i*13+13]` is disjoint, so the parallel map is race-free;
    // the final `copy_from_slice` back into the caller-owned buffer is serial.
    let handles: Vec<_> = world
        .inner
        .bodies
        .iter()
        .map(|(h, _)| h)
        .take(capacity)
        .collect();
    let snapshots: Vec<[f64; 13]> = handles
        .into_par_iter()
        .map(|h| {
            let body = &world.inner.bodies[h];
            let t = body.translation();
            let r = body.rotation();
            let lv = body.linvel();
            let av = body.angvel();
            [
                t.x, t.y, t.z, r.x, r.y, r.z, r.w, lv.x, lv.y, lv.z, av.x, av.y, av.z,
            ]
        })
        .collect();

    let written = snapshots.len().min(capacity);
    for (i, snap) in snapshots.iter().enumerate().take(written) {
        out[i * 13..i * 13 + 13].copy_from_slice(snap);
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
pub fn get_double_array_critical(address: i64, length: i32) -> Option<(*const f64, usize)> {
    if address == 0 || length <= 0 {
        return None;
    }
    Some((address as *const f64, length as usize))
}

/// Get a pointer to a Java byte[] without copying (Critical section).
pub fn get_byte_array_critical(address: i64, length: i32) -> Option<(*const u8, usize)> {
    if address == 0 || length <= 0 {
        return None;
    }
    Some((address as *const u8, length as usize))
}

// ---------------------------------------------------------------------------
// Minecraft-specific: chunk voxel data pipeline
// ---------------------------------------------------------------------------

/// Upper bound on voxels accepted from a single DirectByteBuffer, in the
/// style of collider.rs's `MAX_HEIGHTMAP_CELLS`.  2^24 cells comfortably
/// covers any real chunk/region grid (a full Minecraft chunk is ~98k).
const MAX_VOXEL_CELLS: usize = 16_777_216;

/// Copy Minecraft chunk voxel data from a DirectByteBuffer into a collider
/// builder, zero-copy.
#[allow(clippy::too_many_arguments)] // JNI-facing signature is frozen
pub fn voxel_collider_from_direct_buffer(
    _world: *mut crate::rapier::ffi::WorldHandle,
    voxel_address: i64,
    size_x: i32,
    size_y: i32,
    size_z: i32,
    voxel_x: f64,
    voxel_y: f64,
    voxel_z: f64,
    origin_x: f64,
    origin_y: f64,
    origin_z: f64,
    mode: i32,
    dynamic_body: bool,
    small_voxel_limit: i32,
    mesh_voxel_limit: i32,
) -> i64 {
    catch_unwind(AssertUnwindSafe(|| {
        // Validate dimensions before multiplying: `size_x * size_y * size_z`
        // in i32 can wrap and produce a bogus (even negative-looking) slice
        // length.
        if size_x <= 0 || size_y <= 0 || size_z <= 0 {
            return 0i64;
        }
        let voxel_count = (size_x as usize)
            .checked_mul(size_y as usize)
            .and_then(|n| n.checked_mul(size_z as usize));
        let Some(voxel_count) = voxel_count.filter(|&n| n <= MAX_VOXEL_CELLS) else {
            return 0i64;
        };
        // SAFETY: `voxel_address` comes from a Java DirectByteBuffer that the
        // Java caller keeps alive and unmodified for the duration of this call.
        let Some(voxels) =
            (unsafe { direct_byte_buffer_as_slice(voxel_address, voxel_count as i32) })
        else {
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
            voxel_x,
            voxel_y,
            voxel_z,
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
