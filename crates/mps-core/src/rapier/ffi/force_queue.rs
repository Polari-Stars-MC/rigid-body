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
use crate::rapier::error::{
    ERR_INTERNAL, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ERR_OK, ffi_guard, set_error,
};
use crate::rapier::ffi::convert::unpack_rigid_body_handle;
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use mps_bindgen_macro::java_struct;
use rapier3d::prelude::Vector;

/// Upper bound on `capacity`.
///
/// The C ABI passes the queue as a bare pointer with **no length parameter**,
/// so every bounds check inside this module is faith-based: the length of the
/// `bitmap()` / `payload()` slices is derived from `capacity`, which the
/// producer writes. Without a cap a corrupt or hostile header (`capacity =
/// 1 << 40` is a valid power of two) makes `from_raw_parts` describe petabytes
/// and the very first indexed access is out of bounds.
///
/// 1 M slots is 56 MiB of payload at stride 7 and far above any realistic
/// per-frame force batch.
pub const MAX_FORCE_QUEUE_CAPACITY: u64 = 1 << 20;

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
    ///
    /// Java writes `flags` concurrently, so this must be an atomic load — the
    /// plain-field read it used to do was a data race even though the module
    /// docs promise an acquire load. The field itself is `u32` in a
    /// `#[repr(C, align(64))]` struct, so viewing it as `AtomicU32` is sound
    /// (same size/alignment, no `repr(packed)`).
    #[inline]
    pub fn is_paused(&self) -> bool {
        let flags = unsafe {
            (*(core::ptr::addr_of!(self.flags) as *const AtomicU32)).load(Ordering::Acquire)
        };
        (flags & 1) != 0
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
    ///
    /// Returns `false` for an out-of-range index instead of reading out of
    /// bounds: this is a safe function, so it must not be able to invoke UB
    /// (`get_unchecked` here was reachable with any `index`).
    #[inline]
    pub fn is_bit_set(&self, index: u64) -> bool {
        let word = index / 64;
        let bit = index % 64;
        let bitmap = unsafe { self.bitmap() };
        match bitmap.get(word as usize) {
            Some(word) => word.load(Ordering::Acquire) & (1u64 << bit) != 0,
            None => false,
        }
    }

    /// Clears the bitmap bit at `index` (Rust consumer side). No-op when the
    /// index is out of range, for the same reason as `is_bit_set`.
    #[inline]
    pub fn clear_bit(&self, index: u64) {
        let word = index / 64;
        let bit = index % 64;
        let bitmap = unsafe { self.bitmap() };
        if let Some(word) = bitmap.get(word as usize) {
            word.fetch_and(!(1u64 << bit), Ordering::Release);
        }
    }

    /// Validates the header configuration.
    /// Returns `ERR_OK` if valid, `ERR_INVALID_ARGUMENT` otherwise.
    #[inline]
    pub fn validate(&self) -> u32 {
        if self.capacity == 0 || !self.capacity.is_power_of_two() {
            return ERR_INVALID_ARGUMENT;
        }
        if self.capacity > MAX_FORCE_QUEUE_CAPACITY {
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
    // This entry point dereferences two raw pointers and indexes a slice whose
    // length comes from the producer's header, so it needs the same panic
    // barrier as every other `extern "C"` entry: a panic here would abort the
    // host JVM instead of surfacing as ERR_INTERNAL.
    ffi_guard(ERR_INTERNAL, || {
        if world.is_null() || queue.is_null() {
            set_error(ERR_NULL_POINTER, "world or force queue is null");
            return ERR_NULL_POINTER;
        }

        let hdr = unsafe { &*queue };

        // Validate configuration
        let validation = hdr.validate();
        if validation != ERR_OK {
            set_error(
                ERR_INVALID_ARGUMENT,
                "force queue header is invalid (capacity must be a power of two \
                 <= MAX_FORCE_QUEUE_CAPACITY; stride must be 6 or 7)",
            );
            return validation;
        }

        // Early exit if paused (Java sets flags bit 0)
        if hdr.is_paused() {
            return ERR_OK;
        }

        let world = unsafe { &mut *world };
        let bitmap = unsafe { hdr.bitmap() };
        let payload = unsafe { hdr.payload() };

        // Load head with Acquire ordering (pairs with Java's Release store)
        let head = unsafe {
            (*(core::ptr::addr_of!(hdr.head) as *const AtomicU64)).load(Ordering::Acquire)
        };
        let mut tail = unsafe {
            (*(core::ptr::addr_of!(hdr.tail) as *const AtomicU64)).load(Ordering::Relaxed)
        };

        let capacity = hdr.capacity;
        let mask = capacity - 1; // capacity is power of 2
        let stride = hdr.stride as usize;

        // A producer that publishes `head` more than `capacity` slots past
        // `tail` is either corrupt or has wrapped without the consumer noticing.
        // Without this check the loop below runs for as long as `tail != head`,
        // i.e. up to 2^64 iterations of work inside a single FFI call.
        if head.wrapping_sub(tail) > capacity {
            set_error(
                ERR_INVALID_ARGUMENT,
                "force queue head/tail are inconsistent (head - tail > capacity)",
            );
            return ERR_INVALID_ARGUMENT;
        }

        // Process slots [tail, head)
        while tail != head {
            let idx = tail & mask;
            let word_idx = idx / 64;
            let bit = idx % 64;

            // Check if this slot is active (bitmap bit set)
            let Some(word) = bitmap.get(word_idx as usize) else {
                set_error(ERR_INVALID_ARGUMENT, "force queue bitmap is truncated");
                return ERR_INVALID_ARGUMENT;
            };
            if word.load(Ordering::Acquire) & (1u64 << bit) != 0 {
                // Slot is active — read payload
                let base = (idx as usize) * stride;
                let Some(slot) = payload.get(base..base + stride) else {
                    set_error(ERR_INVALID_ARGUMENT, "force queue payload is truncated");
                    return ERR_INVALID_ARGUMENT;
                };
                let body_id = slot[0] as u64;
                let force = Vector::new(slot[1], slot[2], slot[3]);

                // Apply force at center of mass
                let handle = unpack_rigid_body_handle(body_id);
                if let Some(body) = world.inner.bodies.get_mut(handle) {
                    body.add_force(force, true); // true = wake up
                }

                // If stride == 7, also apply torque
                if stride == 7 {
                    let torque = Vector::new(slot[4], slot[5], slot[6]);
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
    })
}
