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
//! 44      4      cmd_count (u32, pending commands)
//! 48      4      body_slot_stride (u32)
//! 52      4      cmd_slot_stride (u32)
//! 56      4      event_slot_stride (u32)
//! 60      4      force_law_count (u32, number of active ForceLawType entries)
//! 64      8      body_handle_map_offset (u64, offset from ptr to handle map, 0=none)
//! 72      8      force_report_offset (u64, offset from ptr to force contributions, 0=none)
//! 80      8      reserved_0
//! 88      8      reserved_1
//! 96      32     [reserved; 32 bytes]
//! 128     —      body_slots[max_bodies × body_slot_stride]
//! ...     —      body_handle_map[max_bodies × 8]   (optional, after body_slots)
//! ...     —      cmd_ring[max_commands × cmd_slot_stride]
//! ...     —      force_report[32 × 32]             (optional: per-ForceLawType contributions)
//! ...     —      event_ring[max_events × event_slot_stride]
//! ```
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
//! ## BodyHandleMap layout (8 bytes per body)
//!
//! ```text
//! Offset  Size   Field
//! 0       8      handle_raw (u64, Rapier RigidBodyHandle packed as u64)
//! ```
//!
//! This maps arena index → Rust `RigidBodyHandle` so Java can correlate
//! the body it inserted with its arena slot.

use std::alloc::{Layout, alloc_zeroed, dealloc};
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use rapier3d::prelude::RigidBodyType;

use crate::rapier::ffi::Vec3;
use crate::rapier::forces::ForceLawType;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Magic number identifying a valid arena: "MPS_AREN"
const ARENA_MAGIC: u64 = 0x4D50535F4152454E;

/// Current arena layout version — increment when layout changes
const ARENA_VERSION: u32 = 1;

/// Arena flags
const FLAG_DIRTY_BODIES: u32 = 1 << 0;
const FLAG_DIRTY_EVENTS: u32 = 1 << 1;
const FLAG_STEP_IN_PROGRESS: u32 = 1 << 2;

/// Strides (must match Java side exactly)
const BODY_SLOT_STRIDE: u32 = 96;
const COLLIDER_SLOT_STRIDE: u32 = 80;
const CMD_SLOT_STRIDE: u32 = 32;
const EVENT_SLOT_STRIDE: u32 = 64;

/// Header size in bytes
const HEADER_SIZE: usize = 128;

/// Default arena sizes
const DEFAULT_MAX_BODIES: u32 = 1024;
const DEFAULT_MAX_COLLIDERS: u32 = 2048;
const DEFAULT_MAX_EVENTS: u32 = 4096;
const DEFAULT_MAX_COMMANDS: u32 = 4096;

// ---------------------------------------------------------------------------
// Command types
// ---------------------------------------------------------------------------

#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandType {
    AddForce = 0,
    AddTorque = 1,
    SetPose = 2,
    SetVelocity = 3,
    ApplyImpulse = 4,
    ApplyTorqueImpulse = 5,
    WakeUp = 6,
    Sleep = 7,
    SetRotation = 8,
    SetGravityScale = 9,
    SetLinearDamping = 10,
    SetAngularDamping = 11,
    AddForceAtPoint = 12,
}

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
    ptr: *mut u8,
    /// Total size in bytes
    size: usize,
    /// Offset of body slots from ptr
    body_slots_offset: usize,
    /// Offset of collider slots from ptr
    collider_slots_offset: usize,
    /// Offset of body handle map from ptr (0 = disabled)
    body_handle_map_offset: usize,
    /// Offset of command ring from ptr
    cmd_ring_offset: usize,
    /// Offset of force report from ptr (0 = disabled)
    force_report_offset: usize,
    /// Offset of event ring from ptr
    event_ring_offset: usize,
    /// Max bodies
    max_bodies: u32,
    /// Max colliders
    max_colliders: u32,
    /// Max commands
    max_commands: u32,
    /// Max events
    max_events: u32,
    /// Command ring write index (Rust reads from this)
    cmd_write: AtomicU32,
    /// Command ring read index (Rust reads from this)
    cmd_read: AtomicU32,
    /// Event ring write index (Rust writes to this)
    event_write: AtomicU32,
    /// Event ring read index (Java reads from this)
    event_read: AtomicU32,
}

// SAFETY: The arena owns its allocation.  Java accesses it via memory-mapped
// IO, which is safe as long as the Java side follows the protocol.
unsafe impl Send for SharedPhysicsArena {}
unsafe impl Sync for SharedPhysicsArena {}

impl SharedPhysicsArena {
    /// Create a new arena with the given capacities.
    ///
    /// Returns the arena and the raw pointer (for passing to Java).
    pub fn new(max_bodies: u32, max_colliders: u32, max_events: u32, max_commands: u32) -> Self {
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
            + cmd_ring_size
            + event_ring_size;

        let layout = Layout::from_size_align(total_size, 64)
            .expect("arena layout must be valid");
        let ptr = unsafe { alloc_zeroed(layout) };
        assert!(!ptr.is_null(), "arena allocation failed");

        let body_slots_offset = HEADER_SIZE;
        let collider_slots_offset = body_slots_offset + body_slots_size;
        let body_handle_map_offset = collider_slots_offset + collider_slots_size;
        let force_report_offset = body_handle_map_offset + body_handle_map_size;
        let cmd_ring_offset = force_report_offset + force_report_size;
        let event_ring_offset = cmd_ring_offset + cmd_ring_size;

        // Write header
        unsafe {
            (ptr as *mut u64).write_unaligned(ARENA_MAGIC);
            (ptr.add(8) as *mut u32).write_unaligned(ARENA_VERSION);
            (ptr.add(12) as *mut u32).write_unaligned(0);
            (ptr.add(16) as *mut u32).write_unaligned(max_bodies);
            (ptr.add(20) as *mut u32).write_unaligned(max_colliders);
            (ptr.add(24) as *mut u32).write_unaligned(max_events);
            (ptr.add(28) as *mut u32).write_unaligned(max_commands);
            (ptr.add(48) as *mut u32).write_unaligned(BODY_SLOT_STRIDE);
            (ptr.add(52) as *mut u32).write_unaligned(COLLIDER_SLOT_STRIDE);
            (ptr.add(56) as *mut u32).write_unaligned(CMD_SLOT_STRIDE);
            (ptr.add(60) as *mut u32).write_unaligned(EVENT_SLOT_STRIDE);
            // body_handle_map_offset and force_report_offset
            (ptr.add(64) as *mut u64).write_unaligned(body_handle_map_offset as u64);
            (ptr.add(72) as *mut u64).write_unaligned(force_report_offset as u64);
        }

        Self {
            ptr,
            size: total_size,
            body_slots_offset,
            collider_slots_offset,
            body_handle_map_offset,
            cmd_ring_offset,
            force_report_offset,
            event_ring_offset,
            max_bodies,
            max_colliders,
            max_commands,
            max_events,
            cmd_write: AtomicU32::new(0),
            cmd_read: AtomicU32::new(0),
            event_write: AtomicU32::new(0),
            event_read: AtomicU32::new(0),
        }
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

    // -----------------------------------------------------------------------
    // Header accessors
    // -----------------------------------------------------------------------

    fn header_u32(&self, offset: usize) -> u32 {
        unsafe { (self.ptr.add(offset) as *const u32).read_unaligned() }
    }

    fn set_header_u32(&self, offset: usize, value: u32) {
        unsafe { (self.ptr.add(offset) as *mut u32).write_unaligned(value); }
    }

    fn header_u64(&self, offset: usize) -> u64 {
        unsafe { (self.ptr.add(offset) as *const u64).read_unaligned() }
    }

    // -----------------------------------------------------------------------
    // Body slot access
    // -----------------------------------------------------------------------

    /// Get pointer to a body slot.
    fn body_slot_ptr(&self, index: u32) -> *mut u8 {
        unsafe {
            self.ptr.add(
                self.body_slots_offset + index as usize * BODY_SLOT_STRIDE as usize,
            )
        }
    }

    /// Flush a single body's state to its arena slot.
    ///
    /// Called after `world_step` for each active body.
    pub fn flush_body(
        &self,
        index: u32,
        pos_x: f64, pos_y: f64, pos_z: f64,
        vel_x: f64, vel_y: f64, vel_z: f64,
        angvel_x: f64, angvel_y: f64, angvel_z: f64,
        body_type: u32,
        sleeping: u32,
        user_data: u64,
    ) {
        if index >= self.max_bodies {
            return;
        }

        let slot = self.body_slot_ptr(index);

        unsafe {
            // Increment generation to odd (writing)
            let gen_ptr = &*(slot as *const AtomicU64);
            let current_gen = gen_ptr.load(Ordering::Relaxed);
            gen_ptr.store(current_gen.wrapping_add(1) | 1, Ordering::Release);

            // Write data
            (slot.add(8) as *mut f64).write_unaligned(pos_x);
            (slot.add(16) as *mut f64).write_unaligned(pos_y);
            (slot.add(24) as *mut f64).write_unaligned(pos_z);
            (slot.add(32) as *mut f64).write_unaligned(vel_x);
            (slot.add(40) as *mut f64).write_unaligned(vel_y);
            (slot.add(48) as *mut f64).write_unaligned(vel_z);
            (slot.add(56) as *mut f64).write_unaligned(angvel_x);
            (slot.add(64) as *mut f64).write_unaligned(angvel_y);
            (slot.add(72) as *mut f64).write_unaligned(angvel_z);
            (slot.add(80) as *mut u32).write_unaligned(body_type);
            (slot.add(84) as *mut u32).write_unaligned(sleeping);
            (slot.add(88) as *mut u64).write_unaligned(user_data);

            // Increment generation to even (done writing)
            gen_ptr.store(current_gen.wrapping_add(2), Ordering::Release);
        }
    }

    /// Mark a body slot as empty (no longer in use).
    pub fn clear_body_slot(&self, index: u32) {
        if index >= self.max_bodies {
            return;
        }
        let slot = self.body_slot_ptr(index);
        unsafe {
            // Set generation to 0 (Java side treats gen=0 as "empty slot")
            (&*(slot as *const AtomicU64)).store(0, Ordering::Release);
        }
    }

    // -----------------------------------------------------------------------
    // Body handle map — arena index → Rapier RigidBodyHandle
    // -----------------------------------------------------------------------

    /// Write a body handle into the handle map.
    fn write_body_handle(&self, index: u32, handle_raw: u64) {
        if self.body_handle_map_offset == 0 || index >= self.max_bodies {
            return;
        }
        unsafe {
            let slot = self.ptr.add(self.body_handle_map_offset + index as usize * 8);
            (slot as *mut u64).write_unaligned(handle_raw);
        }
    }

    // -----------------------------------------------------------------------
    // Force report — per-ForceLawType contributions (32 slots)
    // -----------------------------------------------------------------------

    /// Map ForceLawType to a compact u32 tag for the arena force report section.
    pub fn force_law_type_tag(ft: ForceLawType) -> u32 {
        match ft {
            ForceLawType::WorldGravity => 0,
            ForceLawType::UserForce => 1,
            ForceLawType::NewtonianGravity => 2,
            ForceLawType::CoulombFriction => 3,
            ForceLawType::AirDrag => 4,
            ForceLawType::Buoyancy => 5,
            ForceLawType::Electromagnetic => 6,
            ForceLawType::ElasticSpring => 7,
            ForceLawType::PointGravity => 8,
            ForceLawType::AerodynamicSurface => 9,
            ForceLawType::AerodynamicVoxel => 10,
            ForceLawType::FluidAABB => 11,
            ForceLawType::MolecularLennardJones => 12,
            ForceLawType::MolecularCoulomb => 13,
            ForceLawType::SpaceJ2 => 14,
            ForceLawType::SpaceCMG => 15,
            ForceLawType::SpaceAtmosphericDrag => 16,
            ForceLawType::SpaceSolarRadiation => 17,
            ForceLawType::SpaceGravityGradient => 18,
            ForceLawType::SpaceMagneticTorquer => 19,
            ForceLawType::TrajectoryCoriolis => 20,
            ForceLawType::TrajectoryCentrifugal => 21,
            ForceLawType::TrajectoryGravity => 22,
            ForceLawType::ControlPID => 23,
            ForceLawType::Custom(_) => 0xFF,
            _ => 0xFE,
        }
    }

    /// Flush the per-frame ForceReport to the arena's force_report region.
    ///
    /// Writes up to 32 ForceLawType contributions so Java can read which
    /// force types are active and how much force each contributed.
    pub fn flush_force_breakdown(
        &self,
        report: &crate::rapier::forces::ForceReport,
    ) {
        if self.force_report_offset == 0 {
            return;
        }

        let mut count = 0u32;
        for (law_type, contrib) in &report.contributions {
            if count >= 32 {
                break;
            }
            let type_tag = Self::force_law_type_tag(*law_type);
            let offset = self.force_report_offset + count as usize * 32;
            unsafe {
                (self.ptr.add(offset) as *mut f64).write_unaligned(contrib.total_force.x);
                (self.ptr.add(offset + 8) as *mut f64).write_unaligned(contrib.total_force.y);
                (self.ptr.add(offset + 16) as *mut f64).write_unaligned(contrib.total_force.z);
                (self.ptr.add(offset + 24) as *mut u32).write_unaligned(contrib.body_count);
                (self.ptr.add(offset + 28) as *mut u32).write_unaligned(type_tag);
            }
            count += 1;
        }

        // Update header: force_law_count
        self.set_header_u32(60, count);

        // Clear remaining slots
        for i in count..32 {
            let offset = self.force_report_offset + i as usize * 32;
            unsafe {
                (self.ptr.add(offset) as *mut f64).write_unaligned(0.0);
                (self.ptr.add(offset + 8) as *mut f64).write_unaligned(0.0);
                (self.ptr.add(offset + 16) as *mut f64).write_unaligned(0.0);
                (self.ptr.add(offset + 24) as *mut u32).write_unaligned(0);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Command ring (Java writes, Rust reads)
    // -----------------------------------------------------------------------

    fn cmd_slot_ptr(&self, index: u32) -> *mut u8 {
        let wrapped = index % self.max_commands;
        unsafe {
            self.ptr.add(
                self.cmd_ring_offset + wrapped as usize * CMD_SLOT_STRIDE as usize,
            )
        }
    }

    /// Drain all pending commands from the command ring.
    ///
    /// Returns a Vec of (cmd_type, body_index, arg0, arg1, arg2) tuples.
    /// Called at the beginning of `world_step`.
    pub fn drain_commands(&self) -> Vec<(u32, u32, f64, f64, f64)> {
        let mut commands = Vec::new();
        let write = self.cmd_write.load(Ordering::Acquire);
        let mut read = self.cmd_read.load(Ordering::Relaxed);

        while read != write {
            let slot = self.cmd_slot_ptr(read);
            let cmd_type = unsafe { (slot as *const u32).read_unaligned() };
            let body_index = unsafe { (slot.add(4) as *const u32).read_unaligned() };
            let arg0 = unsafe { (slot.add(8) as *const f64).read_unaligned() };
            let arg1 = unsafe { (slot.add(16) as *const f64).read_unaligned() };
            let arg2 = unsafe { (slot.add(24) as *const f64).read_unaligned() };

            commands.push((cmd_type, body_index, arg0, arg1, arg2));

            read = read.wrapping_add(1);
        }

        self.cmd_read.store(read, Ordering::Release);
        // Update header
        self.set_header_u32(44, 0);

        commands
    }

    // -----------------------------------------------------------------------
    // Event ring (Rust writes, Java reads)
    // -----------------------------------------------------------------------

    fn event_slot_ptr(&self, index: u32) -> *mut u8 {
        let wrapped = index % self.max_events;
        unsafe {
            self.ptr.add(
                self.event_ring_offset + wrapped as usize * EVENT_SLOT_STRIDE as usize,
            )
        }
    }

    /// Push a collision event to the event ring.
    pub fn push_collision_event(
        &self,
        started: bool,
        collider1: u32,
        collider2: u32,
        sensor: bool,
        removed: bool,
    ) {
        let write = self.event_write.load(Ordering::Relaxed);
        let read = self.event_read.load(Ordering::Acquire);

        // Ring full check
        if write.wrapping_sub(read) >= self.max_events {
            return; // drop event (ring full)
        }

        let slot = self.event_slot_ptr(write);

        let flags: u32 = if sensor { 1 } else { 0 } | if removed { 2 } else { 0 };

        unsafe {
            (slot as *mut u32).write_unaligned(if started { 0 } else { 1 });
            (slot.add(4) as *mut u32).write_unaligned(collider1);
            (slot.add(8) as *mut u32).write_unaligned(collider2);
            (slot.add(12) as *mut u32).write_unaligned(flags);
            // Zero out force fields
            (slot.add(16) as *mut f64).write_unaligned(0.0);
            (slot.add(24) as *mut f64).write_unaligned(0.0);
            (slot.add(32) as *mut f64).write_unaligned(0.0);
            (slot.add(40) as *mut f64).write_unaligned(0.0);
            (slot.add(48) as *mut f64).write_unaligned(0.0);
            (slot.add(56) as *mut f64).write_unaligned(0.0);
        }

        self.event_write.store(write.wrapping_add(1), Ordering::Release);
        // Update header event count
        let count = write.wrapping_add(1).wrapping_sub(read);
        self.set_header_u32(40, count.min(self.max_events));
    }

    /// Push a contact force event to the event ring.
    pub fn push_contact_force_event(
        &self,
        collider1: u32,
        collider2: u32,
        total_force_x: f64, total_force_y: f64, total_force_z: f64,
        total_force_mag: f64,
        max_force_x: f64, max_force_y: f64, max_force_z: f64,
    ) {
        let write = self.event_write.load(Ordering::Relaxed);
        let read = self.event_read.load(Ordering::Acquire);

        if write.wrapping_sub(read) >= self.max_events {
            return;
        }

        let slot = self.event_slot_ptr(write);

        unsafe {
            (slot as *mut u32).write_unaligned(2); // ContactForce
            (slot.add(4) as *mut u32).write_unaligned(collider1);
            (slot.add(8) as *mut u32).write_unaligned(collider2);
            (slot.add(12) as *mut u32).write_unaligned(0);
            (slot.add(16) as *mut f64).write_unaligned(total_force_x);
            (slot.add(24) as *mut f64).write_unaligned(total_force_y);
            (slot.add(32) as *mut f64).write_unaligned(total_force_z);
            (slot.add(40) as *mut f64).write_unaligned(total_force_mag);
            (slot.add(48) as *mut f64).write_unaligned(max_force_x);
            (slot.add(56) as *mut f64).write_unaligned(max_force_y);
        }

        self.event_write.store(write.wrapping_add(1), Ordering::Release);
        let count = write.wrapping_add(1).wrapping_sub(read);
        self.set_header_u32(40, count.min(self.max_events));
    }

    /// Reset event ring (called after Java drains events).
    pub fn reset_event_ring(&self) {
        let write = self.event_write.load(Ordering::Relaxed);
        self.event_read.store(write, Ordering::Release);
        self.set_header_u32(40, 0);
    }

    // -----------------------------------------------------------------------
    // Full flush after world_step
    // -----------------------------------------------------------------------

    /// Flush all active bodies to their arena slots.
    ///
    /// Called after `world_step` completes.
    pub fn flush_all_bodies(&self, bodies: &rapier3d::prelude::RigidBodySet) {
        let mut index = 0u32;
        for (handle, body) in bodies.iter() {
            if index >= self.max_bodies {
                break;
            }

            // Write body handle map (arena index → Rapier handle)
            self.write_body_handle(index, handle.into_raw_parts().0 as u64);

            let pos = body.translation();
            let vel = body.linvel();
            let angvel = body.angvel();

            let body_type = match body.body_type() {
                RigidBodyType::Dynamic => 0u32,
                RigidBodyType::Fixed => 1u32,
                RigidBodyType::KinematicVelocityBased => 2u32,
                RigidBodyType::KinematicPositionBased => 3u32,
            };

            let sleeping = if body.is_sleeping() { 1u32 } else { 0u32 };
            let user_data = (body.user_data & 0xFFFF_FFFF_FFFF_FFFF) as u64;

            self.flush_body(
                index,
                pos.x, pos.y, pos.z,
                vel.x, vel.y, vel.z,
                angvel.x, angvel.y, angvel.z,
                body_type,
                sleeping,
                user_data,
            );

            index += 1;
        }

        // Update body count in header
        self.set_header_u32(32, index);

        // Clear remaining slots
        for i in index..self.max_bodies {
            self.clear_body_slot(i);
        }
    }

    // -----------------------------------------------------------------------
    // Collider slot access
    // -----------------------------------------------------------------------

    fn collider_slot_ptr(&self, index: u32) -> *mut u8 {
        unsafe {
            self.ptr.add(
                self.collider_slots_offset + index as usize * COLLIDER_SLOT_STRIDE as usize,
            )
        }
    }

    /// Flush a single collider's state.  Layout (80 bytes):
    ///   offset 0 : generation (u64)
    ///   offset 8 : parent_body_index (u32) + padding (4)
    ///   offset 16: pos_x, pos_y, pos_z (3 × f64)
    ///   offset 40: friction (f64)
    ///   offset 48: restitution (f64)
    ///   offset 56: density (f64)
    ///   offset 64: sensor (u32), active_events (u32)
    ///   offset 72: collision_groups_memberships (u32), filter (u32)
    pub fn flush_collider(
        &self,
        index: u32,
        parent_body_index: u32,
        pos_x: f64, pos_y: f64, pos_z: f64,
        friction: f64,
        restitution: f64,
        density: f64,
        sensor: u32,
        active_events: u32,
        collision_groups_memberships: u32,
        collision_groups_filter: u32,
    ) {
        if index >= self.max_colliders {
            return;
        }
        let slot = self.collider_slot_ptr(index);
        unsafe {
            let gen_ptr = &*(slot as *const AtomicU64);
            let current_gen = gen_ptr.load(Ordering::Relaxed);
            gen_ptr.store(current_gen.wrapping_add(1) | 1, Ordering::Release);

            (slot.add(8) as *mut u32).write_unaligned(parent_body_index);
            (slot.add(16) as *mut f64).write_unaligned(pos_x);
            (slot.add(24) as *mut f64).write_unaligned(pos_y);
            (slot.add(32) as *mut f64).write_unaligned(pos_z);
            (slot.add(40) as *mut f64).write_unaligned(friction);
            (slot.add(48) as *mut f64).write_unaligned(restitution);
            (slot.add(56) as *mut f64).write_unaligned(density);
            (slot.add(64) as *mut u32).write_unaligned(sensor);
            (slot.add(68) as *mut u32).write_unaligned(active_events);
            (slot.add(72) as *mut u32).write_unaligned(collision_groups_memberships);
            (slot.add(76) as *mut u32).write_unaligned(collision_groups_filter);

            gen_ptr.store(current_gen.wrapping_add(2), Ordering::Release);
        }
    }

    fn clear_collider_slot(&self, index: u32) {
        if index >= self.max_colliders { return; }
        let slot = self.collider_slot_ptr(index);
        unsafe { (&*(slot as *const AtomicU64)).store(0, Ordering::Release); }
    }

    /// Flush all colliders after world_step.
    pub fn flush_all_colliders(&self, colliders: &rapier3d::prelude::ColliderSet) {
        let mut index = 0u32;
        for (_handle, collider) in colliders.iter() {
            if index >= self.max_colliders { break; }
            let pos = collider.translation();
            let parent = collider.parent().map_or(u32::MAX, |h| {
                (h.into_raw_parts().0 & 0xFFFF_FFFF) as u32
            });
            self.flush_collider(
                index, parent,
                pos.x, pos.y, pos.z,
                collider.friction(),
                collider.restitution(),
                collider.density(),
                if collider.is_sensor() { 1 } else { 0 },
                collider.active_events().bits(),
                collider.collision_groups().memberships.bits(),
                collider.collision_groups().filter.bits(),
            );
            index += 1;
        }
        self.set_header_u32(36, index);
        for i in index..self.max_colliders { self.clear_collider_slot(i); }
    }

    // -----------------------------------------------------------------------
    // Integration parameters (zero-JNI read/write by Java)
    // -----------------------------------------------------------------------

    /// Flush integration parameters and force report into the header's reserved space.
    pub fn flush_integration_params(
        &self,
        dt: f64,
        solver_iterations: u32,
        ccd_substeps: u32,
        gravity: &rapier3d::prelude::Vector,
    ) {
        unsafe {
            (self.ptr.add(64) as *mut f64).write_unaligned(dt);
            (self.ptr.add(72) as *mut u32).write_unaligned(solver_iterations);
            (self.ptr.add(76) as *mut u32).write_unaligned(ccd_substeps);
            (self.ptr.add(80) as *mut f64).write_unaligned(gravity.x);
            (self.ptr.add(88) as *mut f64).write_unaligned(gravity.y);
            (self.ptr.add(96) as *mut f64).write_unaligned(gravity.z);
        }
    }

    /// Flush force report data for zero-JNI reading.
    pub fn flush_force_report(
        &self,
        max_reynolds: f64,
        total_external_force: &crate::rapier::ffi::Vec3,
        total_drag_force: &crate::rapier::ffi::Vec3,
        drag_body_count: u32,
        ext_force_body_count: u32,
    ) {
        unsafe {
            (self.ptr.add(104) as *mut f64).write_unaligned(max_reynolds);
            (self.ptr.add(112) as *mut f64).write_unaligned(total_external_force.x);
            (self.ptr.add(120) as *mut f64).write_unaligned(total_external_force.y);
            (self.ptr.add(128) as *mut f64).write_unaligned(total_external_force.z);
            (self.ptr.add(136) as *mut f64).write_unaligned(total_drag_force.x);
            (self.ptr.add(144) as *mut f64).write_unaligned(total_drag_force.y);
            (self.ptr.add(152) as *mut f64).write_unaligned(total_drag_force.z);
            (self.ptr.add(160) as *mut u32).write_unaligned(drag_body_count);
            (self.ptr.add(164) as *mut u32).write_unaligned(ext_force_body_count);
        }
    }

    /// Flush collision and contact-force events from the event handler to the arena event ring.
    ///
    /// Called after `world_step` so Java can read events zero-JNI.
    pub fn flush_events_from_handler(
        &self,
        events: &std::sync::Arc<crate::rapier::events::CollectingEventHandler>,
    ) {
        let mut evt_count = 0u32;

        // Drain collision events
        let col_count = events.collision_event_count();
        for i in 0..col_count {
            if let Some(evt) = events.collision_event(i) {
                let collider1 = (evt.collider1 & 0xFFFF_FFFF) as u32;
                let collider2 = (evt.collider2 & 0xFFFF_FFFF) as u32;
                let started = evt.started.0 != 0;
                let sensor = evt.sensor.0 != 0;
                let removed = evt.removed.0 != 0;
                self.push_collision_event(started, collider1, collider2, sensor, removed);
                evt_count += 1;
            }
        }

        // Drain contact force events
        let cf_count = events.contact_force_event_count();
        for i in 0..cf_count {
            if let Some(evt) = events.contact_force_event(i) {
                let collider1 = (evt.collider1 & 0xFFFF_FFFF) as u32;
                let collider2 = (evt.collider2 & 0xFFFF_FFFF) as u32;
                self.push_contact_force_event(
                    collider1, collider2,
                    evt.total_force.x, evt.total_force.y, evt.total_force.z,
                    evt.total_force_magnitude,
                    evt.max_force_direction.x, evt.max_force_direction.y, evt.max_force_direction.z,
                );
                evt_count += 1;
            }
        }

        // Update event count in header
        self.set_header_u32(40, evt_count);
    }

    /// Set flags in the header atomically.
    pub fn set_flags(&self, flags: u32) {
        let ptr = unsafe { self.ptr.add(12) as *mut AtomicU32 };
        unsafe { (*ptr).fetch_or(flags, Ordering::Release); }
    }

    /// Clear flags in the header atomically.
    pub fn clear_flags(&self, flags: u32) {
        let ptr = unsafe { self.ptr.add(12) as *mut AtomicU32 };
        unsafe { (*ptr).fetch_and(!flags, Ordering::Release); }
    }
}

impl Drop for SharedPhysicsArena {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            let layout = Layout::from_size_align(self.size, 64)
                .expect("arena layout must be valid");
            unsafe { dealloc(self.ptr, layout); }
            self.ptr = std::ptr::null_mut();
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arena_create_and_drop() {
        let arena = SharedPhysicsArena::new(16, 32, 64, 128);
        assert!(!arena.as_ptr().is_null());
        let expected_size = HEADER_SIZE
            + 16 * BODY_SLOT_STRIDE as usize
            + 32 * COLLIDER_SLOT_STRIDE as usize
            + 16 * 8 // body_handle_map
            + 32 * 32 // force_report
            + 128 * CMD_SLOT_STRIDE as usize
            + 64 * EVENT_SLOT_STRIDE as usize;
        assert_eq!(arena.size(), expected_size,
            "expected {} got {}", expected_size, arena.size());

        // Check header magic
        let magic = arena.header_u64(0);
        assert_eq!(magic, ARENA_MAGIC);

        let version = arena.header_u32(8);
        assert_eq!(version, ARENA_VERSION);
    }

    #[test]
    fn body_flush_and_readback() {
        let arena = SharedPhysicsArena::new(8, 0, 0, 0);

        arena.flush_body(0, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.01, 0.02, 0.03, 0, 0, 42);

        // Read back from the slot
        let slot = arena.body_slot_ptr(0);
        unsafe {
            let generation_val = (slot as *const u64).read_unaligned();
            assert!(generation_val > 0, "generation should be non-zero");
            assert_eq!(generation_val & 1, 0, "generation should be even (stable)");

            let pos_x = (slot.add(8) as *const f64).read_unaligned();
            assert!((pos_x - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn command_drain_empty() {
        let arena = SharedPhysicsArena::new(8, 0, 0, 16);
        let cmds = arena.drain_commands();
        assert!(cmds.is_empty());
    }

    #[test]
    fn event_push_and_reset() {
        let arena = SharedPhysicsArena::new(0, 0, 8, 0);

        arena.push_collision_event(true, 1, 2, false, false);
        arena.push_collision_event(false, 3, 4, true, true);

        // Event count in header should be 2
        let count = arena.header_u32(40);
        assert_eq!(count, 2);

        arena.reset_event_ring();
        let count = arena.header_u32(40);
        assert_eq!(count, 0);
    }
}