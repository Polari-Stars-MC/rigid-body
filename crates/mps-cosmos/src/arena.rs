//! Shared-memory physics arena for `mps-cosmos` — zero-JNI data access from Java.
//!
//! Forked from `mps-core`'s `shared_arena` (the proven design), trimmed to the
//! needs of the space simulator: **body slots** (Rust→Java readback) and a
//! **command ring** (Java→Rust one-way write).  The collider slots, force
//! report, force summary, event ring and integration-params regions are all
//! `mps-core`-specific (MC body/collider/event/force-law machinery) and are
//! intentionally dropped here — cosmos has no colliders-by-default, no
//! `ForceLawType` registry, no collision events.
//!
//! The layout is *not* identical to `mps-core`'s: cosmos keeps a 128-byte
//! header (so Java-side readers can reuse the same offset table shape) but
//! only populates `body_slots` + `cmd_ring` + `body_handle_map`.  The header
//! magic/version are cosmos-specific (`0x434F534D_4152454E` = "COSMAREN",
//! version 1) so a Java `ByteBuffer` can distinguish a cosmos arena from a
//! core arena at map time and reject a mismatch.
//!
//! ## Synchronization protocol
//!
//! Every `BodySlot` carries a generation counter (seqlock-style): Rust bumps it
//! to odd *before* writing, back to even *after* writing; Java reads
//! `gen_before` → data → `gen_after`, treating `gen_before == gen_after &&
//! even` as a consistent read.  The command ring is SPSC (Java producer, Rust
//! consumer): Java writes a 32-byte slot then bumps `cmd_write` (header offset
//! 44); Rust drains `[0, cmd_write)` at the top of `step` and resets the index
//! to 0.
//!
//! Payload words are stored as atomic integer bit patterns. Java readers must
//! use Acquire/volatile reads for payload words between the two generation
//! reads, preserving the byte layout while preventing torn `f64` values.

use std::alloc::{Layout, alloc_zeroed, dealloc};
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Magic number identifying a valid cosmos arena: "COSMAREN".
pub const ARENA_MAGIC: u64 = 0x434F534D_4152454E;

/// Current cosmos arena layout version — increment on any layout change.
pub const ARENA_VERSION: u32 = 1;

/// Body slot stride (must match Java side exactly).
pub const BODY_SLOT_STRIDE: u32 = 96;
/// Command slot stride — 5 × u64 (cmd_type, body_index, a0, a1, a2).
pub const CMD_SLOT_STRIDE: u32 = 40;

/// Header size in bytes.
pub const HEADER_SIZE: usize = 128;

/// Upper bounds for arena capacities — defense against absurd FFI requests.
pub const MAX_ARENA_BODIES: u32 = 1_000_000;
pub const MAX_ARENA_COMMANDS: u32 = 1_000_000;
/// Hard cap on the total arena allocation (256 MiB) — also the Java
/// `ByteBuffer.capacity()` 2 GiB ceiling.  Keep ≤ `i32::MAX`.
pub const MAX_ARENA_TOTAL_BYTES: usize = 256 * 1024 * 1024;

// Header offset table (bytes) — stable ABI, mirrored by the Java side and the
// round-trip tests in `mps-test`.  See `docs/cosmos-arena.md` for the full map.
pub const OFF_BODY_COUNT: usize = 32; // u32: bodies flushed last step
pub const OFF_CMD_WRITE: usize = 44; // u32: SPSC ring write index (Java → Rust)
pub const OFF_BODY_SLOT_BASE: usize = HEADER_SIZE; // body slots start here
/// Header offset (u64) storing the command-ring base offset (dynamic: depends on
/// max_bodies).  Read this at map time instead of recomputing the layout.
pub const OFF_CMD_RING: usize = 96;

// Internal offsets (private to the module).
pub(super) const OFF_BODY_HANDLE_MAP: usize = 64;

// ---------------------------------------------------------------------------
// Arena struct
// ---------------------------------------------------------------------------

/// A shared-memory arena that maps cosmos physics state for zero-copy access.
pub struct SharedArena {
    /// Raw pointer to the allocation.
    pub(super) ptr: *mut u8,
    /// Total size in bytes.
    pub(super) size: usize,
    /// Offset of body slots from ptr.
    pub(super) body_slots_offset: usize,
    /// Offset of body handle map from ptr (0 = disabled).
    pub(super) body_handle_map_offset: usize,
    /// Offset of command ring from ptr.
    pub(super) cmd_ring_offset: usize,
    /// Max bodies.
    pub(super) max_bodies: u32,
    /// Max commands.
    pub(super) max_commands: u32,
    /// 上一帧 `flush_all_bodies` 实际写入的 body 数（尾清零回收用，同 mps-core
    /// `prev_body_active_count` 语义）。
    pub(super) prev_body_active_count: AtomicU32,
}

// SAFETY: the arena owns its allocation.  Java accesses it via memory-mapped IO,
// which is safe as long as the Java side follows the protocol (native-order
// `ByteBuffer`, seqlock on body-slot generation).
unsafe impl Send for SharedArena {}
unsafe impl Sync for SharedArena {}

impl SharedArena {
    /// Create a new arena with the given body/command capacities.
    ///
    /// Returns the arena, or `None` if a capacity exceeds `MAX_ARENA_*`, the
    /// total size exceeds `MAX_ARENA_TOTAL_BYTES`, or the allocation fails.
    pub fn new(max_bodies: u32, max_commands: u32) -> Option<Self> {
        if max_bodies == 0
            || max_commands == 0
            || max_bodies > MAX_ARENA_BODIES
            || max_commands > MAX_ARENA_COMMANDS
        {
            return None;
        }

        let body_slots_size = max_bodies as usize * BODY_SLOT_STRIDE as usize;
        let body_handle_map_size = max_bodies as usize * 8; // u64 per body
        let cmd_ring_size = max_commands as usize * CMD_SLOT_STRIDE as usize;

        let total_size = HEADER_SIZE + body_slots_size + body_handle_map_size + cmd_ring_size;
        // Arena is exposed to Java as a `ByteBuffer` whose `capacity()` is `int`,
        // so the total size MUST stay ≤ `Integer.MAX_VALUE` (2 GiB).  The 256 MiB
        // cap already guarantees this, but we assert it explicitly so a future cap
        // bump cannot silently break the Java consumer.
        if total_size > MAX_ARENA_TOTAL_BYTES || total_size > (i32::MAX as usize) {
            return None;
        }

        let layout = Layout::from_size_align(total_size, 64).ok()?;
        let ptr = unsafe { alloc_zeroed(layout) };
        if ptr.is_null() {
            return None;
        }

        let body_slots_offset = HEADER_SIZE;
        let body_handle_map_offset = body_slots_offset + body_slots_size;
        let cmd_ring_offset = body_handle_map_offset + body_handle_map_size;

        unsafe {
            (ptr as *mut u64).write_unaligned(ARENA_MAGIC);
            (ptr.add(8) as *mut u32).write_unaligned(ARENA_VERSION);
            (ptr.add(12) as *mut u32).write_unaligned(0); // flags
            (ptr.add(16) as *mut u32).write_unaligned(max_bodies);
            (ptr.add(20) as *mut u32).write_unaligned(0); // max_colliders (unused)
            (ptr.add(24) as *mut u32).write_unaligned(0); // max_events (unused)
            (ptr.add(28) as *mut u32).write_unaligned(max_commands);
            (ptr.add(32) as *mut u32).write_unaligned(0); // body_count (live)
            (ptr.add(36) as *mut u32).write_unaligned(0); // collider_count (unused)
            (ptr.add(40) as *mut u32).write_unaligned(0); // event_count (unused)
            (ptr.add(44) as *mut u32).write_unaligned(0); // cmd_write (pending count)
            (ptr.add(48) as *mut u32).write_unaligned(BODY_SLOT_STRIDE);
            (ptr.add(52) as *mut u32).write_unaligned(0); // collider_slot_stride (unused)
            (ptr.add(56) as *mut u32).write_unaligned(CMD_SLOT_STRIDE);
            (ptr.add(60) as *mut u32).write_unaligned(0); // event_slot_stride (unused)
            // Region offsets (Java reads these instead of recomputing).
            (ptr.add(OFF_BODY_HANDLE_MAP) as *mut u64)
                .write_unaligned(body_handle_map_offset as u64);
            (ptr.add(OFF_CMD_RING) as *mut u64).write_unaligned(cmd_ring_offset as u64);
            // `body_slots_offset` is always HEADER_SIZE (128); no header slot for it.
        }

        Some(Self {
            ptr,
            size: total_size,
            body_slots_offset,
            body_handle_map_offset,
            cmd_ring_offset,
            max_bodies,
            max_commands,
            prev_body_active_count: AtomicU32::new(0),
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

    /// Get the base address as a u64 (for `MemorySegment.ofAddress` in Java).
    pub fn address(&self) -> u64 {
        self.ptr as u64
    }

    /// Header u32 accessor. Header fields are shared with Java, so even
    /// metadata reads use an atomic load to avoid a Rust data race with the
    /// producer's `cmd_write` publication.
    pub(super) fn header_u32(&self, offset: usize) -> u32 {
        debug_assert_eq!(offset % std::mem::align_of::<AtomicU32>(), 0);
        unsafe { (&*(self.ptr.add(offset) as *const AtomicU32)).load(Ordering::Acquire) }
    }

    pub(super) fn set_header_u32(&self, offset: usize, value: u32) {
        debug_assert_eq!(offset % std::mem::align_of::<AtomicU32>(), 0);
        unsafe { (&*(self.ptr.add(offset) as *const AtomicU32)).store(value, Ordering::Release) }
    }

    /// Pointer to a body slot.
    pub(super) fn body_slot_ptr(&self, index: u32) -> *mut u8 {
        unsafe {
            self.ptr
                .add(self.body_slots_offset + index as usize * BODY_SLOT_STRIDE as usize)
        }
    }

    /// Flush a single body's state into its arena slot (seqlock generation).
    #[allow(clippy::too_many_arguments)]
    pub(super) fn flush_body(
        &self,
        index: u32,
        pos_x: f64,
        pos_y: f64,
        pos_z: f64,
        vel_x: f64,
        vel_y: f64,
        vel_z: f64,
        angvel_x: f64,
        angvel_y: f64,
        angvel_z: f64,
        body_type: u32,
        sleeping: u32,
        user_data: u64,
    ) {
        if index >= self.max_bodies {
            return;
        }
        let slot = self.body_slot_ptr(index);
        unsafe {
            let gen_ptr = &*(slot as *const AtomicU64);
            let current_gen = gen_ptr.load(Ordering::Relaxed);
            gen_ptr.store(current_gen.wrapping_add(1) | 1, Ordering::Release);
            // Payload is written with relaxed atomics. The final even
            // generation Release publishes all preceding stores; readers use
            // Acquire around their seqlock validation.
            (&*(slot.add(8) as *const AtomicU64)).store(pos_x.to_bits(), Ordering::Relaxed);
            (&*(slot.add(16) as *const AtomicU64)).store(pos_y.to_bits(), Ordering::Relaxed);
            (&*(slot.add(24) as *const AtomicU64)).store(pos_z.to_bits(), Ordering::Relaxed);
            (&*(slot.add(32) as *const AtomicU64)).store(vel_x.to_bits(), Ordering::Relaxed);
            (&*(slot.add(40) as *const AtomicU64)).store(vel_y.to_bits(), Ordering::Relaxed);
            (&*(slot.add(48) as *const AtomicU64)).store(vel_z.to_bits(), Ordering::Relaxed);
            (&*(slot.add(56) as *const AtomicU64)).store(angvel_x.to_bits(), Ordering::Relaxed);
            (&*(slot.add(64) as *const AtomicU64)).store(angvel_y.to_bits(), Ordering::Relaxed);
            (&*(slot.add(72) as *const AtomicU64)).store(angvel_z.to_bits(), Ordering::Relaxed);
            (&*(slot.add(80) as *const AtomicU32)).store(body_type, Ordering::Relaxed);
            (&*(slot.add(84) as *const AtomicU32)).store(sleeping, Ordering::Relaxed);
            (&*(slot.add(88) as *const AtomicU64)).store(user_data, Ordering::Relaxed);
            gen_ptr.store(current_gen.wrapping_add(2), Ordering::Release);
        }
    }

    /// Mark a body slot as empty (gen = 0).
    pub(super) fn clear_body_slot(&self, index: u32) {
        if index >= self.max_bodies {
            return;
        }
        let slot = self.body_slot_ptr(index);
        unsafe {
            (&*(slot as *const AtomicU64)).store(0, Ordering::Release);
        }
    }

    fn write_body_handle(&self, index: u32, handle_raw: u64) {
        if self.body_handle_map_offset == 0 || index >= self.max_bodies {
            return;
        }
        unsafe {
            let slot = self
                .ptr
                .add(self.body_handle_map_offset + index as usize * 8);
            (&*(slot as *const AtomicU64)).store(handle_raw, Ordering::Relaxed);
        }
    }

    /// Flush all body state to the arena (after `step`).  Maps arena index →
    /// body (in insertion order, matching `arena_idx_map`), writes the body
    /// slot, and tail-clears only the `[curr .. prev]` region (M3-style).
    ///
    /// 多线程：体数 ≥ [`FLUSH_PARALLEL_MIN`] 时槽写入阶段按 rayon 并行——每个
    /// body 槽（96B seqlock 域 + handle map 项）互不重叠，逐槽写序列与串行完全
    /// 相同（seqlock 协议按槽独立，Java 读侧语义不变）；header 活跃数与尾部
    /// 清零仍在所有槽写入完成后**串行**执行（顺序依赖：header 是「已写槽数」
    /// 的发布点）。低于阈值走原串行循环（零分配、行为逐位不变）。
    pub fn flush_all_bodies(&self, bodies: &rapier3d::prelude::RigidBodySet) {
        use rayon::prelude::*;

        // 并行 flush 的体数下限：低于它 seqlock 槽写本身足够便宜，rayon 分片
        // 调度开销不划算。
        const FLUSH_PARALLEL_MIN: usize = 512;

        let total = bodies.len().min(self.max_bodies as usize);
        if total >= FLUSH_PARALLEL_MIN {
            // 先按插入序收集 (arena index, handle)，再并行写各自槽。
            let pairs: Vec<(u32, rapier3d::prelude::RigidBodyHandle)> = bodies
                .iter()
                .take(total)
                .enumerate()
                .map(|(i, hb)| (i as u32, hb.0))
                .collect();
            pairs.par_iter().for_each(|&(index, handle)| {
                self.flush_one_body(bodies, index, handle);
            });
        } else {
            for (index, (handle, _)) in bodies.iter().take(total).enumerate() {
                self.flush_one_body(bodies, index as u32, handle);
            }
        }

        // 槽全部写完后才发布 header 活跃数 + 尾部清零（与串行版的收尾一致；
        // header 是「已写槽数」的发布点，必须在所有槽写入之后）。
        let total_u32 = total as u32;
        self.set_header_u32(32, total_u32);
        let prev = self.prev_body_active_count.load(Ordering::Relaxed);
        if total_u32 < prev {
            for i in total_u32..prev {
                self.clear_body_slot(i);
            }
        }
        self.prev_body_active_count
            .store(total_u32, Ordering::Relaxed);
    }

    /// 把单个 body 的状态写进它的 arena 槽（`flush_all_bodies` 的每体主体，
    /// 串行/并行共用同一段写序列）。
    fn flush_one_body(
        &self,
        bodies: &rapier3d::prelude::RigidBodySet,
        index: u32,
        handle: rapier3d::prelude::RigidBodyHandle,
    ) {
        self.write_body_handle(index, handle.into_raw_parts().0 as u64);
        let Some(body) = bodies.get(handle) else {
            return;
        };
        let pos = body.translation();
        let vel = body.linvel();
        let angvel = body.angvel();
        let body_type = match body.body_type() {
            rapier3d::prelude::RigidBodyType::Dynamic => 0u32,
            rapier3d::prelude::RigidBodyType::Fixed => 1u32,
            rapier3d::prelude::RigidBodyType::KinematicVelocityBased => 2u32,
            rapier3d::prelude::RigidBodyType::KinematicPositionBased => 3u32,
        };
        let sleeping = if body.is_sleeping() { 1u32 } else { 0u32 };
        let user_data = (body.user_data & 0xFFFF_FFFF_FFFF_FFFF) as u64;
        self.flush_body(
            index, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, angvel.x, angvel.y, angvel.z,
            body_type, sleeping, user_data,
        );
    }

    // -----------------------------------------------------------------------
    // Command ring (Java→Rust)
    // -----------------------------------------------------------------------

    fn cmd_slot_ptr(&self, index: u32) -> *mut u8 {
        let wrapped = index % self.max_commands;
        unsafe {
            self.ptr
                .add(self.cmd_ring_offset + wrapped as usize * CMD_SLOT_STRIDE as usize)
        }
    }

    /// Drain all pending commands from the command ring.
    ///
    /// Returns `Vec<(cmd_type, body_index, arg0, arg1, arg2)>`.  Called at the
    /// top of `step`.  Clamps to `max_commands` so a broken producer cannot
    /// read past the ring; resets `cmd_write` to 0 after draining.
    pub fn drain_commands(&self) -> Vec<(u32, u32, f64, f64, f64)> {
        let mut commands = Vec::new();
        let write = self.header_u32(44).min(self.max_commands);
        for read in 0..write {
            let slot = self.cmd_slot_ptr(read);
            // 命令槽布局（与 `write_command` / Java 侧一致）：5 × u64 —
            // [cmd_type@0, body_index@8, a0@16, a1@24, a2@32]。
            let cmd_type = unsafe { (slot as *const u32).read_unaligned() };
            let body_index = unsafe { (slot.add(8) as *const u32).read_unaligned() };
            let arg0 = unsafe { (slot.add(16) as *const f64).read_unaligned() };
            let arg1 = unsafe { (slot.add(24) as *const f64).read_unaligned() };
            let arg2 = unsafe { (slot.add(32) as *const f64).read_unaligned() };
            commands.push((cmd_type, body_index, arg0, arg1, arg2));
        }
        self.set_header_u32(44, 0);
        commands
    }
}

impl Drop for SharedArena {
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

/// Command types understood by the cosmos arena command ring.  Matches the
/// mps-core `CommandType` subset that makes sense for the space sim (forces,
/// torques, velocities, poses, impulses — no collider/event commands).
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandType {
    /// `a0..a2` = force (N) at COM.
    AddForce = 0,
    /// `a0..a2` = torque (N·m).
    AddTorque = 1,
    /// `a0..a2` = linear velocity (m/s).
    SetVelocity = 2,
    /// `a0..a2` = impulse (N·s) at COM.
    ApplyImpulse = 3,
    /// `a0..a2` = angular impulse (N·m·s).
    ApplyTorqueImpulse = 4,
    /// wake the body.
    WakeUp = 5,
    /// put the body to sleep.
    Sleep = 6,
    /// `a0..a2` = axis·angle vector (magnitude = angle rad).
    SetRotation = 7,
    /// `a0..a2` = position; rotation kept.
    SetPose = 8,
    /// `a0` = gravity scale.
    SetGravityScale = 9,
    /// `a0` = linear damping.
    SetLinearDamping = 10,
    /// `a0` = angular damping.
    SetAngularDamping = 11,
}
