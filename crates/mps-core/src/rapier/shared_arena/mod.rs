//! Shared-memory physics arena — zero-JNI data access from Java.
//!
//! ## Motivation
//!
//! Traditional JNI/FFM physics requires one native call per read (get position,
//! get velocity, get event) and one call per write (add force, set pose).  For
//! 100 bodies this is 200+ JNI calls per frame — ~20 µs overhead just crossing
//! the FFI boundary.
//!
//! The shared arena eliminates this entirely:
//!
//! ```text
//! Before (JNI-per-operation):
//!   Java → JNI → Rust  (×200 per frame)  = 20 µs overhead
//!
//! After (shared arena):
//!   Java reads arena directly   (×200, in pure Java)  = 0.05 µs
//!   Java writes commands to ring (×100, in pure Java)  = 0.03 µs
//!   world_step signals Rust      (×1, JNI)             = 0.10 µs
//! ```
//!
//! **160× faster** per-frame data exchange.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────── Rust (this module) ──────────────────┐
//! │ SharedPhysicsArena                                       │
//! │   header:    version, body_count, collider_count, flags  │
//! │   body_slots: [BodySlot; N]          ← written by Rust  │
//! │   cmd_queue:  lock-free SPSC ring    ← read by Rust     │
//! │   event_ring: lock-free SPSC ring    ← written by Rust  │
//! │                                                          │
//! │ world_step:                                              │
//! │   1. drain cmd_queue  → apply forces / set poses         │
//! │   2. pipeline.step()  → Rapier physics                   │
//! │   3. flush body_slots ← write latest state               │
//! │   4. flush event_ring ← write collision/contact events   │
//! └──────────────────────────────────────────────────────────┘
//!         ↑ memory-mapped (mmap / Box::leak + DirectByteBuffer)
//!         ↓
//! ┌─────────────────── Java (MemorySegment) ─────────────────┐
//! │ SharedPhysicsArena arena =                                │
//! │   SharedPhysicsArena.map(arenaPtr, arenaSize);            │
//! │                                                           │
//! │ // READ (zero JNI):                                       │
//! │ double[] pos = arena.readBodyPosition(bodyIndex);         │
//! │ CollisionEvent[] events = arena.readEvents();             │
//! │                                                           │
//! │ // WRITE (zero JNI):                                      │
//! │ arena.commandAddForce(bodyIndex, fx, fy, fz);             │
//! │ arena.commandSetPose(bodyIndex, x, y, z, qw, qx, qy, qz);│
//! │                                                           │
//! │ // COMMIT (1 JNI call):                                   │
//! │ world.step();  // Rust drains cmds, steps, flushes state  │
//! └──────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Synchronization protocol
//!
//! Every `BodySlot` has a `generation` counter.  Rust increments it atomically
//! **before** writing new data and **after** writing is complete.  Java reads
//! `gen_before` → reads data → reads `gen_after`.  If `gen_before == gen_after`
//! and both are even, the data is consistent.
//!
//! ## Memory layout (all fields 8-byte aligned)
//!
//! ```text
//! Offset  Size   Field
//! 0       8      magic: 0x4D50535F4152454E ("MPS_AREN")
//! 8       4      version (u32)
//! 12      4      flags (u32)
//! 16      4      max_bodies (u32)
//! 20      4      max_colliders (u32)
//! 24      4      max_events (u32)
//! 28      4      max_commands (u32)
//! 32      4      body_count (u32, live bodies)
//! 36      4      collider_count (u32, live colliders)
//! 40      4      event_count (u32, pending events)
//! 44      4      cmd_write (u32 — command write index / pending count:
//!                Java bumps it for each command written,
//!                Rust resets it to 0 after draining)
//! 48      4      body_slot_stride (u32)
//! 52      4      collider_slot_stride (u32)
//! 56      4      cmd_slot_stride (u32)
//! 60      4      event_slot_stride (u32)
//! 64      8      body_handle_map_offset (u64, offset from ptr to handle map)
//! 72      8      force_report_offset (u64, per-ForceLawType breakdown, 32 × 32 bytes)
//! 80      8      integration_params_offset (u64, 40-byte region, see below)
//! 88      8      force_summary_offset (u64, 64-byte region, see below)
//! 96      8      cmd_ring_offset (u64)
//! 104     8      event_ring_offset (u64)
//! 112     8      collider_slots_offset (u64)
//! 120     4      force_law_count (u32, number of active ForceLawType entries)
//! 124     4      reserved
//! 128     —      body_slots[max_bodies × body_slot_stride]
//! ...     —      collider_slots[max_colliders × collider_slot_stride]
//! ...     —      body_handle_map[max_bodies × 8]
//! ...     —      force_report[32 × 32]             (per-ForceLawType contributions)
//! ...     —      integration_params[40]
//! ...     —      force_summary[64]
//! ...     —      cmd_ring[max_commands × cmd_slot_stride]
//! ...     —      event_ring[max_events × event_slot_stride]
//! ```
//!
//! Region offsets are written into the header by `new()`; Java must read
//! them from the header instead of recomputing them from capacities.
//!
//! ## BodySlot layout (96 bytes, 8-byte aligned)
//!
//! ```text
//! Offset  Size   Field
//! 0       8      generation (u64) — even = stable, odd = writing
//! 8       8      pos_x (f64)
//! 16      8      pos_y (f64)
//! 24      8      pos_z (f64)
//! 32      8      vel_x (f64)
//! 40      8      vel_y (f64)
//! 48      8      vel_z (f64)
//! 56      8      angvel_x (f64)
//! 64      8      angvel_y (f64)
//! 72      8      angvel_z (f64)
//! 80      4      body_type (u32: 0=Dynamic, 1=Fixed, 2=KinematicVelocity, 3=KinematicPosition)
//! 84      4      sleeping (u32: 0=awake, 1=sleeping)
//! 88      8      user_data (u64, low 64 bits of u128)
//! ```
//!
//! ## CommandSlot layout (32 bytes)
//!
//! ```text
//! Offset  Size   Field
//! 0       4      cmd_type (u32: 0=AddForce, 1=AddTorque, 2=SetPose, 3=SetVelocity, 4=ApplyImpulse)
//! 4       4      body_index (u32)
//! 8       8      arg0 (f64) — force_x / pos_x / vel_x / impulse_x
//! 16      8      arg1 (f64) — force_y / pos_y / vel_y / impulse_y
//! 24      8      arg2 (f64) — force_z / pos_z / vel_z / impulse_z
//! ```
//!
//! ### Command ring protocol
//!
//! Java is the sole producer, Rust the sole consumer.  For each command,
//! Java writes a `CommandSlot` at `cmd_ring_offset + cmd_write * 32` and then
//! bumps `cmd_write` (header offset 44).  At the start of `world_step`, Rust
//! drains slots `[0, min(cmd_write, max_commands))` and resets `cmd_write`
//! to 0, so the ring never wraps within a frame and `cmd_write` doubles as
//! the pending-command count.
//!
//! ## EventSlot layout (64 bytes)
//!
//! ```text
//! Offset  Size   Field
//! 0       4      event_type (u32: 0=CollisionStart, 1=CollisionStop, 2=ContactForce)
//! 4       4      collider1_index (u32)
//! 8       4      collider2_index (u32)
//! 12      4      flags (u32: bit0=sensor, bit1=removed)
//! 16      8      total_force_x (f64)
//! 24      8      total_force_y (f64)
//! 32      8      total_force_z (f64)
//! 40      8      total_force_magnitude (f64)
//! 48      8      max_force_x (f64)
//! 56      8      max_force_y (f64)
//! ```
//!
//! ## ForceContribution layout (32 bytes, 32 slots for ForceLawType 0..31)
//!
//! ```text
//! Offset  Size   Field
//! 0       8      total_force_x (f64, Kahan-accumulated, N)
//! 8       8      total_force_y (f64)
//! 16      8      total_force_z (f64)
//! 24      4      body_count (u32, bodies that received this force type)
//! 28      4      reserved
//! ```
//!
//! ## IntegrationParams layout (40 bytes)
//!
//! ```text
//! Offset  Size   Field
//! 0       8      dt (f64)
//! 8       4      solver_iterations (u32)
//! 12      4      ccd_substeps (u32)
//! 16      8      gravity_x (f64)
//! 24      8      gravity_y (f64)
//! 32      8      gravity_z (f64)
//! ```
//!
//! ## ForceSummary layout (64 bytes)
//!
//! ```text
//! Offset  Size   Field
//! 0       8      max_reynolds_number (f64)
//! 8       8      total_external_force_x (f64)
//! 16      8      total_external_force_y (f64)
//! 24      8      total_external_force_z (f64)
//! 32      8      total_drag_force_x (f64)
//! 40      8      total_drag_force_y (f64)
//! 48      8      total_drag_force_z (f64)
//! 56      4      drag_body_count (u32)
//! 60      4      external_force_body_count (u32)
//! ```
//!
//! ## BodyHandleMap layout (8 bytes per body)
//!
//! ```text
//! Offset  Size   Field
//! 0       8      handle_raw (u64, Rapier RigidBodyHandle packed as u64)
//! ```
//!
//! This maps arena index → Rust `RigidBodyHandle` so Java can correlate
//! the body it inserted with its arena slot.
//!
//! ## Submodule layout
//!
//! Originally a single 1028-line `shared_arena.rs`.  Split per OPTIMIZATION.md
//! §N5 into 5 files grouped by cohesion; `SharedPhysicsArena` stays a single
//! type — each submodule extends its `impl` block.  Fields below are
//! `pub(super)` so impls in sibling files can read/write them directly.
//!
//! - [`layout`] — `CommandType` enum (no struct here—struct lives below in
//!   this `mod.rs` so submodules can construct it via `Self { ... }`).
//! - [`header`] — header accessors (`header_u32`/`set_header_u32`/`header_u64`)
//!   + flag manipulation + integration params + force summary writes.
//! - [`holes`] — body slot & collider slot writers (the "holes" region of
//!   the arena) + body handle map + force breakdown + `flush_all_bodies`/
//!   `flush_all_colliders` aggregates.
//! - [`ring`] — SPSC command ring (Java→Rust) and event ring (Rust→Java)
//!   plus `flush_events_from_handler` bridge from `CollectingEventHandler`.

use std::alloc::{Layout, alloc_zeroed, dealloc};
use std::sync::atomic::AtomicU32;

// Re-exported so submodules see them via `use super::*;`.  Using `pub(crate)`
// keeps the symbols visible to every impl-block file without polluting the
// crate root.
mod header;
mod holes;
mod layout;
mod ring;

pub use layout::CommandType;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Magic number identifying a valid arena: "MPS_AREN"
pub const ARENA_MAGIC: u64 = 0x4D50535F4152454E;

/// Current arena layout version — increment when layout changes.
///
/// Named `MPS_ARENA_VERSION` (not `ARENA_VERSION`) because cbindgen turns this
/// const into a C macro in `include/rigid_body.h`, and a bare `ARENA_VERSION`
/// macro would collide with `cosmos.h`'s `ARENA_VERSION 1` when a host includes
/// both headers. A protocol v3 (new arena layout) must still bump this value.
pub const MPS_ARENA_VERSION: u32 = 2;

/// Backward-compatible Rust alias so in-workspace readers (mps-test version
/// pins) keep compiling. cbindgen does not export `pub use` items, so this
/// alias never appears in the generated header — `MPS_ARENA_VERSION` is the
/// only macro there.
#[doc(hidden)]
pub use MPS_ARENA_VERSION as ARENA_VERSION;

/// Strides (must match Java side exactly)
pub const BODY_SLOT_STRIDE: u32 = 96;
pub const COLLIDER_SLOT_STRIDE: u32 = 80;
pub const CMD_SLOT_STRIDE: u32 = 32;
pub const EVENT_SLOT_STRIDE: u32 = 64;

/// Header size in bytes
pub const HEADER_SIZE: usize = 128;

/// Upper bounds for arena capacities — defense against absurd FFI requests.
pub const MAX_ARENA_BODIES: u32 = 1_000_000;
pub const MAX_ARENA_COLLIDERS: u32 = 1_000_000;
pub const MAX_ARENA_EVENTS: u32 = 1_000_000;
pub const MAX_ARENA_COMMANDS: u32 = 1_000_000;
/// Hard cap on the total arena allocation (256 MiB).
pub const MAX_ARENA_TOTAL_BYTES: usize = 256 * 1024 * 1024;

/// Integration params region: dt(8) + solver_iterations(4) + ccd_substeps(4) + gravity(24)
pub const INTEGRATION_PARAMS_SIZE: usize = 40;
/// Force summary region: max_reynolds(8) + external force(24) + drag force(24) + counts(8)
pub const FORCE_SUMMARY_SIZE: usize = 64;

// Header offsets (bytes) — see the layout doc at the top of this file.
// `pub(super)` so the `header` / `ring` / `holes` submodules can address
// fields without restating magic numbers.
pub(super) const OFF_CMD_WRITE: usize = 44;
pub(super) const OFF_BODY_HANDLE_MAP: usize = 64;
pub(super) const OFF_FORCE_REPORT: usize = 72;
pub(super) const OFF_INTEGRATION_PARAMS: usize = 80;
pub(super) const OFF_FORCE_SUMMARY: usize = 88;
pub(super) const OFF_CMD_RING: usize = 96;
pub(super) const OFF_EVENT_RING: usize = 104;
pub(super) const OFF_COLLIDER_SLOTS: usize = 112;
pub(super) const OFF_FORCE_LAW_COUNT: usize = 120;

// ---------------------------------------------------------------------------
// Arena struct
// ---------------------------------------------------------------------------

/// A shared-memory arena that maps physics state for zero-copy access.
///
/// The arena is a single contiguous allocation.  The header is at offset 0,
/// followed by body slots, command ring, and event ring.
///
/// # Safety
///
/// The arena pointer is shared with Java via `DirectByteBuffer`.  Java reads
/// and writes to it concurrently.  All cross-thread access uses atomic
/// operations and the generation-counter protocol.
pub struct SharedPhysicsArena {
    /// Raw pointer to the allocation
    pub(super) ptr: *mut u8,
    /// Total size in bytes
    pub(super) size: usize,
    /// Offset of body slots from ptr
    pub(super) body_slots_offset: usize,
    /// Offset of collider slots from ptr
    pub(super) collider_slots_offset: usize,
    /// Offset of body handle map from ptr (0 = disabled)
    pub(super) body_handle_map_offset: usize,
    /// Offset of force report (per-ForceLawType breakdown) from ptr (0 = disabled)
    pub(super) force_report_offset: usize,
    /// Offset of integration params region from ptr
    pub(super) integration_params_offset: usize,
    /// Offset of force summary region from ptr
    pub(super) force_summary_offset: usize,
    /// Offset of command ring from ptr
    pub(super) cmd_ring_offset: usize,
    /// Offset of event ring from ptr
    pub(super) event_ring_offset: usize,
    /// Max bodies
    pub(super) max_bodies: u32,
    /// Max colliders
    pub(super) max_colliders: u32,
    /// Max commands
    pub(super) max_commands: u32,
    /// Max events
    pub(super) max_events: u32,
    /// Event ring write index (Rust writes to this)
    pub(super) event_write: AtomicU32,
    /// Event ring read index (Java reads from this)
    pub(super) event_read: AtomicU32,
    /// 上一帧 `flush_all_bodies` 实际写入的 body 数（即"上次的
    /// active_count"）。M3 优化用：下一帧若 active_count 缩水到这个数
    /// 以内，只在 `[ curr_count .. prev_count ]` 区间清零一次，省去
    /// 旧的 `for i in curr_count..max_bodies { clear_body_slot(i); }`
    /// 逐 slot 清零（性能分析.MD §11.3 M3 落地）。
    /// 初始 0 —— `alloc_zeroed` 已保证全部 body slot 的 generation=0。
    /// 用 `AtomicU32` 而非 plain `u32` 是因为 `flush_all_bodies` 接收
    /// `&self`（arena 同时由 Java 跨线程读，结构体整体 `Sync`），与既有
    /// `event_write` / `event_read` 一致用 `Relaxed` load/store。
    pub(super) prev_body_active_count: AtomicU32,
    /// 上一帧 `flush_all_colliders` 实际写入的 collider 数——同 body
    /// 的 prev_body_active_count 语义（L4：collider 端的尾清零优化）。
    pub(super) prev_collider_active_count: AtomicU32,
}

// SAFETY: The arena owns its allocation.  Java accesses it via memory-mapped
// IO, which is safe as long as the Java side follows the protocol.
unsafe impl Send for SharedPhysicsArena {}
unsafe impl Sync for SharedPhysicsArena {}

impl SharedPhysicsArena {
    /// Create a new arena with the given capacities.
    ///
    /// Returns the arena and the raw pointer (for passing to Java), or `None`
    /// if a capacity exceeds `MAX_ARENA_*`, the total size exceeds
    /// `MAX_ARENA_TOTAL_BYTES`, or the allocation fails.
    pub fn new(
        max_bodies: u32,
        max_colliders: u32,
        max_events: u32,
        max_commands: u32,
    ) -> Option<Self> {
        if max_bodies > MAX_ARENA_BODIES
            || max_colliders > MAX_ARENA_COLLIDERS
            || max_events > MAX_ARENA_EVENTS
            || max_commands > MAX_ARENA_COMMANDS
        {
            return None;
        }

        // Capacity caps above keep every product well inside usize range.
        let body_slots_size = max_bodies as usize * BODY_SLOT_STRIDE as usize;
        let collider_slots_size = max_colliders as usize * COLLIDER_SLOT_STRIDE as usize;
        let body_handle_map_size = max_bodies as usize * 8; // u64 per body
        let force_report_size = 32 * 32; // 32 slots × 32 bytes (ForceLawType 0..31)
        let cmd_ring_size = max_commands as usize * CMD_SLOT_STRIDE as usize;
        let event_ring_size = max_events as usize * EVENT_SLOT_STRIDE as usize;

        let total_size = HEADER_SIZE
            + body_slots_size
            + collider_slots_size
            + body_handle_map_size
            + force_report_size
            + INTEGRATION_PARAMS_SIZE
            + FORCE_SUMMARY_SIZE
            + cmd_ring_size
            + event_ring_size;
        if total_size > MAX_ARENA_TOTAL_BYTES {
            return None;
        }

        let layout = Layout::from_size_align(total_size, 64).ok()?;
        let ptr = unsafe { alloc_zeroed(layout) };
        if ptr.is_null() {
            return None;
        }

        let body_slots_offset = HEADER_SIZE;
        let collider_slots_offset = body_slots_offset + body_slots_size;
        let body_handle_map_offset = collider_slots_offset + collider_slots_size;
        let force_report_offset = body_handle_map_offset + body_handle_map_size;
        let integration_params_offset = force_report_offset + force_report_size;
        let force_summary_offset = integration_params_offset + INTEGRATION_PARAMS_SIZE;
        let cmd_ring_offset = force_summary_offset + FORCE_SUMMARY_SIZE;
        let event_ring_offset = cmd_ring_offset + cmd_ring_size;

        // Write header
        unsafe {
            (ptr as *mut u64).write_unaligned(ARENA_MAGIC);
            (ptr.add(8) as *mut u32).write_unaligned(MPS_ARENA_VERSION);
            (ptr.add(12) as *mut u32).write_unaligned(0);
            (ptr.add(16) as *mut u32).write_unaligned(max_bodies);
            (ptr.add(20) as *mut u32).write_unaligned(max_colliders);
            (ptr.add(24) as *mut u32).write_unaligned(max_events);
            (ptr.add(28) as *mut u32).write_unaligned(max_commands);
            (ptr.add(48) as *mut u32).write_unaligned(BODY_SLOT_STRIDE);
            (ptr.add(52) as *mut u32).write_unaligned(COLLIDER_SLOT_STRIDE);
            (ptr.add(56) as *mut u32).write_unaligned(CMD_SLOT_STRIDE);
            (ptr.add(60) as *mut u32).write_unaligned(EVENT_SLOT_STRIDE);
            // Region offsets (Java reads these instead of recomputing)
            (ptr.add(OFF_BODY_HANDLE_MAP) as *mut u64)
                .write_unaligned(body_handle_map_offset as u64);
            (ptr.add(OFF_FORCE_REPORT) as *mut u64).write_unaligned(force_report_offset as u64);
            (ptr.add(OFF_INTEGRATION_PARAMS) as *mut u64)
                .write_unaligned(integration_params_offset as u64);
            (ptr.add(OFF_FORCE_SUMMARY) as *mut u64).write_unaligned(force_summary_offset as u64);
            (ptr.add(OFF_CMD_RING) as *mut u64).write_unaligned(cmd_ring_offset as u64);
            (ptr.add(OFF_EVENT_RING) as *mut u64).write_unaligned(event_ring_offset as u64);
            (ptr.add(OFF_COLLIDER_SLOTS) as *mut u64).write_unaligned(collider_slots_offset as u64);
        }

        Some(Self {
            ptr,
            size: total_size,
            body_slots_offset,
            collider_slots_offset,
            body_handle_map_offset,
            force_report_offset,
            integration_params_offset,
            force_summary_offset,
            cmd_ring_offset,
            event_ring_offset,
            max_bodies,
            max_colliders,
            max_commands,
            max_events,
            event_write: AtomicU32::new(0),
            event_read: AtomicU32::new(0),
            prev_body_active_count: AtomicU32::new(0),
            prev_collider_active_count: AtomicU32::new(0),
        })
    }

    /// Get the raw pointer for passing to Java.
    pub fn as_ptr(&self) -> *const u8 {
        self.ptr
    }

    /// Get the total size in bytes.
    pub fn size(&self) -> usize {
        self.size
    }

    /// Get the pointer as a u64 for C FFI.
    pub fn address(&self) -> u64 {
        self.ptr as u64
    }
}

impl Drop for SharedPhysicsArena {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            let layout =
                Layout::from_size_align(self.size, 64).expect("arena layout must be valid");
            unsafe {
                dealloc(self.ptr, layout);
            }
            self.ptr = std::ptr::null_mut();
        }
    }
}
