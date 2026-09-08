/// Shared-memory force queue header for zero-copy Java<->Rust force application.
///
/// Memory layout (cache-line aligned, 64-byte header + bitmap + payload):
///
/// ForceQueueHeader (64 bytes, aligned to 64)
/// +----------+----------+----------+----------+--------+-----------+
/// |capacity  | head     | tail     |generation| stride |  flags    |
/// |  u64     |  u64     |  u64     |   u64    |  u32   |   u32     |
/// +----------+----------+----------+----------+--------+-----------+
/// Bitmap: (capacity + 63) / 64  x  u64  (1 bit per slot)
/// Payload: capacity x stride x 8 bytes (f64 per component)
/// stride = 6 (body_id + force[3])  or  7 (body_id + force[3] + torque[3])
///
/// Synchronization (single-producer / single-consumer, lock-free):
/// - Java is the **sole writer** to each slot's payload and its bitmap bit.
/// - Rust is the **sole reader** of slots where bitmap bit = 1.
/// - `head` (Java writes, Rust reads) uses **release** store / **acquire** load.
/// - `tail` (Rust writes, Java reads) uses **release** store / **acquire** load.
/// - Bitmap bits: Java sets bit with `atomic_or` (release), Rust clears with
///   `atomic_andnot` (release) after processing. No CAS loops needed --
///   single-writer per word eliminates contention.
/// - `flags` bit 0 = paused (Java writes, Rust reads with acquire). Optional
///   gate; does **not** protect enqueue/cancel.
/// - `generation` increments on `head` wrap to resolve ABA.
use crate::rapier::error::{ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ERR_OK};
use crate::rapier::ffi::convert::unpack_rigid_body_handle;
use core::sync::atomic::{AtomicU64, Ordering};
use mps_bindgen_macro::java_struct;
use rapier3d::prelude::Vector;

#[repr(C, align(64))]
#[java_struct(package = "org.polaris2023.mps.ffi")]
pub struct ForceQueueHeader {
    /// Total slot capacity (must be power of 2 for fast modulo via mask).
    pub capacity: u64,
    /// Write index (Java advances via CAS with release; Rust reads with acquire).
    pub head: u64,
    /// Read index (Rust advances with release; Java reads with acquire).
    pub tail: u64,
    /// Generation counter (incremented when `head` wraps; resolves ABA).
    pub generation: u64,
    /// f64 count per slot: 6 (body_id + force) or 7 (body_id + force + torque).
    pub stride: u32,
    /// Bitmap follows immediately after this struct in memory:
    /// `bitmap_words = (capacity + 63) / 64` u64 words.
    /// Payload follows bitmap: `capacity * stride * 8` bytes.
    /// Bit 0 = paused (1 = Rust should skip consumption this frame).
    pub flags: u32,
}

impl ForceQueueHeader {
    /// Returns `true` if the paused flag is set.
    #[inline]
    pub fn is_paused(&self) -> bool {
        (self.flags & 1) != 0
    }

    /// Number of u64 words in the bitmap array.
    #[inline]
    pub fn bitmap_words(&self) -> usize {
        self.capacity
            .checked_add(63)
            .map(|v| (v / 64) as usize)
            .unwrap_or(usize::MAX)
    }

    /// Byte offset from start of header to the bitmap array.
    #[inline]
    pub fn bitmap_offset(&self) -> usize {
        core::mem::size_of::<Self>()
    }

    /// Byte offset from start of header to the payload array.
    #[inline]
    pub fn payload_offset(&self) -> usize {
        self.bitmap_offset()
            .saturating_add(self.bitmap_words().saturating_mul(8))
    }

    /// Total size in bytes of the entire queue (header + bitmap + payload).
    #[inline]
    pub fn total_size(&self) -> usize {
        self.payload_offset().saturating_add(
            (self.capacity as usize)
                .saturating_mul(self.stride as usize)
                .saturating_mul(8),
        )
    }

    /// Returns a pointer to the bitmap array (as `AtomicU64` slice).
    ///
    /// # Safety
    /// Caller must guarantee `self` points to a valid, properly allocated header
    /// with sufficient trailing capacity for bitmap + payload.
    #[inline]
    pub unsafe fn bitmap(&self) -> &[AtomicU64] {
        if self.bitmap_words() == usize::MAX {
            return &[];
        }
        unsafe {
            let ptr = (self as *const Self).byte_add(self.bitmap_offset()) as *const AtomicU64;
            core::slice::from_raw_parts(ptr, self.bitmap_words())
        }
    }

    /// Returns a pointer to the payload array (f64 slice).
    ///
    /// # Safety
    /// Same as `bitmap()`.
    #[inline]
    pub unsafe fn payload(&self) -> &[f64] {
        let Some(len) = (self.capacity as usize).checked_mul(self.stride as usize) else {
            return &[];
        };
        unsafe {
            let ptr = (self as *const Self).byte_add(self.payload_offset()) as *const f64;
            core::slice::from_raw_parts(ptr, len)
        }
    }

    /// Returns a mutable pointer to the payload array (f64 slice).
    ///
    /// # Safety
    /// Same as `bitmap()`. Caller must ensure exclusive access (producer side).
    #[inline]
    pub unsafe fn payload_mut(&mut self) -> &mut [f64] {
        let Some(len) = (self.capacity as usize).checked_mul(self.stride as usize) else {
            return &mut [];
        };
        unsafe {
            let ptr = (self as *mut Self).byte_add(self.payload_offset()) as *mut f64;
            core::slice::from_raw_parts_mut(ptr, len)
        }
    }

    /// Checks if the bitmap bit at `index` is set.
    #[inline]
    pub fn is_bit_set(&self, index: u64) -> bool {
        let word = index / 64;
        let bit = index % 64;
        unsafe {
            (*self.bitmap().get_unchecked(word as usize)).load(Ordering::Acquire) & (1u64 << bit)
                != 0
        }
    }

    /// Clears the bitmap bit at `index` (Rust consumer side).
    #[inline]
    pub fn clear_bit(&self, index: u64) {
        let word = index / 64;
        let bit = index % 64;
        unsafe {
            (*self.bitmap().get_unchecked(word as usize))
                .fetch_and(!(1u64 << bit), Ordering::Release);
        }
    }

    /// Validates the header configuration.
    /// Returns `ERR_OK` if valid, `ERR_INVALID_ARGUMENT` otherwise.
    #[inline]
    pub fn validate(&self) -> u32 {
        if self.capacity == 0 || !self.capacity.is_power_of_two() {
            return ERR_INVALID_ARGUMENT;
        }
        if self.stride != 6 && self.stride != 7 {
            return ERR_INVALID_ARGUMENT;
        }
        ERR_OK
    }
}

/// Consumes all active slots in the force queue and applies forces to Rapier bodies.
///
/// # Safety
/// - `world` must be a valid `WorldHandle` from `rigid_body_world_create`.
/// - `queue` must point to a valid `ForceQueueHeader` allocated by Java with
///   matching `capacity`, `stride`, and sufficient trailing memory for bitmap + payload.
/// - Java must be the sole producer; Rust (this call) is the sole consumer.
/// - The queue memory must remain valid for the duration of this call.
#[unsafe(no_mangle)]
pub extern "C" fn rigid_body_consume_force_queue(
    world: *mut crate::WorldHandle,
    queue: *mut ForceQueueHeader,
) -> u32 {
    if world.is_null() || queue.is_null() {
        return ERR_NULL_POINTER;
    }

    let hdr = unsafe { &*queue };

    // Validate configuration
    if hdr.validate() != ERR_OK {
        return ERR_INVALID_ARGUMENT;
    }

    // Early exit if paused (Java sets flags bit 0)
    if hdr.is_paused() {
        return ERR_OK;
    }

    let world = unsafe { &mut *world };
    let bitmap = unsafe { hdr.bitmap() };
    let payload = unsafe { hdr.payload() };

    // Load head with Acquire ordering (pairs with Java's Release store)
    let head =
        unsafe { (*(core::ptr::addr_of!(hdr.head) as *const AtomicU64)).load(Ordering::Acquire) };
    let mut tail =
        unsafe { (*(core::ptr::addr_of!(hdr.tail) as *const AtomicU64)).load(Ordering::Relaxed) };

    let capacity = hdr.capacity;
    let mask = capacity - 1; // capacity is power of 2
    let stride = hdr.stride as usize;

    // Process slots [tail, head)
    while tail != head {
        let idx = tail & mask;
        let word_idx = idx / 64;
        let bit = idx % 64;

        // Check if this slot is active (bitmap bit set)
        let word = unsafe { bitmap.get_unchecked(word_idx as usize) };
        if word.load(Ordering::Acquire) & (1u64 << bit) != 0 {
            // Slot is active — read payload
            let base = (idx as usize) * stride;
            let body_id = payload[base] as u64;
            let fx = payload[base + 1];
            let fy = payload[base + 2];
            let fz = payload[base + 3];
            let force = Vector::new(fx, fy, fz);

            // Apply force at center of mass
            let handle = unpack_rigid_body_handle(body_id);
            if let Some(body) = world.inner.bodies.get_mut(handle) {
                body.add_force(force, true); // true = wake up
            }

            // If stride == 7, also apply torque
            if stride == 7 {
                let tx = payload[base + 4];
                let ty = payload[base + 5];
                let tz = payload[base + 6];
                let torque = Vector::new(tx, ty, tz);
                if let Some(body) = world.inner.bodies.get_mut(handle) {
                    body.add_torque(torque, true);
                }
            }

            // Clear bitmap bit with Release ordering (pairs with Java's Acquire load if any)
            word.fetch_and(!(1u64 << bit), Ordering::Release);
        }

        // Advance tail with Release ordering
        tail = tail.wrapping_add(1);
        unsafe {
            (*(core::ptr::addr_of!(hdr.tail) as *mut AtomicU64)).store(tail, Ordering::Release);
        }
    }

    ERR_OK
}
