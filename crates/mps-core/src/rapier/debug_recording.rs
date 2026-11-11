//! Debug recording (Jolt-Viewer-style capture & replay) — FFI ring buffer of
//! per-frame body snapshots.
//!
//! Frame layout (little-endian, matches the export blob):
//! `MPSDR01` (8B) | body_count u32 | dt f64 | then per body:
//! packed handle u64 + 13 f64 (pos3 + quat4 + linvel3 + angvel3) — the same
//! 13-lane layout as `world_body_snapshot`.
//!
//! **v2 event section（可选，向后兼容）**：`debug_recording_capture_events`
//! 在 capture 之前收集本步的碰撞/接触力事件，帧尾追加一段
//! `event_count u32 | per event: kind u32 + handleA u64 + handleB u64`
//! （每事件 5 个 f64 槽，kind: 1=collision-start, 2=collision-stop,
//! 3=contact-force）。v1 解码器按 `magic` 槽低 32 位识别版本：v1 magic
//! `...3131`、v2 magic `...3231`（"MPSDR02"）——读到 v1 magic 的解码器
//! 停在 body 表尾，不会误读事件段；v2 解码器读到 v1 magic 时知道帧尾
//! 无事件段。两类旧/新查看器互不误读。

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, ERR_OK, ERR_UNSUPPORTED,
    ffi_guard, set_error,
};
use crate::rapier::ffi::{Bool, MAX_OUTPUT_CAPACITY, WorldHandle, pack_rigid_body_handle};

/// v1 frame magic: `MPSDR01`（版本号在低 16 位：`..01`）。
const RECORDING_MAGIC_V1: u64 = 0x4d50534452303131;
/// v2 frame magic: `MPSDR02`（帧尾带 event section）。
const RECORDING_MAGIC_V2: u64 = 0x4d50534452303231;

/// Per-frame per-body payload: handle + 13 f64 lanes.
const LANES_PER_BODY: usize = 13;

/// Per-event payload in f64 slots: kind + handleA + handleB.
const SLOTS_PER_EVENT: usize = 5;

/// 事件种类（写入 kind 槽）。
pub(crate) const EVENT_COLLISION_STARTED: u32 = 1;
pub(crate) const EVENT_COLLISION_STOPPED: u32 = 2;
pub(crate) const EVENT_CONTACT_FORCE: u32 = 3;

/// 环形录制缓冲。容量按帧数计，溢出覆盖最旧帧（Jolt Viewer 语义：永远保留
/// 最近 N 帧）。`frames` 里每帧是已序列化的 blob 尾部（不含 magic），避免
/// 导出时二次拷贝。
#[derive(Default)]
pub(crate) struct RecordingBuffer {
    /// 每帧：(dt, [(packed_handle, [f64; 13])], events)。
    /// `events` 只在 v2 捕获（`capture_with_events`）时非空；v1 帧恒空。
    frames: std::collections::VecDeque<(
        f64,
        Vec<(u64, [f64; LANES_PER_BODY])>,
        Vec<[f64; SLOTS_PER_EVENT]>,
    )>,
    max_frames: usize,
    /// 累计丢弃的旧帧数（导出时写进头部，供回放器对齐）。
    dropped: u64,
    /// v2 模式：调用过 `enable_events` 后置位，后续帧带 event section。
    events_enabled: bool,
    active: bool,
}

impl RecordingBuffer {
    fn start(&mut self, max_frames: u32) {
        self.frames.clear();
        self.dropped = 0;
        self.max_frames = (max_frames as usize).max(1);
        self.active = true;
    }

    fn capture(
        &mut self,
        bodies_set: &rapier3d::dynamics::RigidBodySet,
        dt: f64,
        events: Vec<[f64; SLOTS_PER_EVENT]>,
    ) {
        if !self.active {
            return;
        }
        // Reuse the shared 13-lane snapshot helper (pos3 + quat4 + linvel3 +
        // angvel3), which also handles the parallel path for large worlds.
        let handles: Vec<_> = bodies_set
            .iter()
            .filter(|(_, b)| b.is_enabled())
            .map(|(h, _)| h)
            .collect();
        let snaps = crate::rapier::parallel::body_pose_snapshots(&handles, bodies_set, true);
        let bodies: Vec<(u64, [f64; LANES_PER_BODY])> = handles
            .iter()
            .zip(snaps)
            .map(|(h, snap)| (pack_rigid_body_handle(*h), snap))
            .collect();
        if self.frames.len() == self.max_frames {
            self.frames.pop_front();
            self.dropped += 1;
        }
        self.frames.push_back((dt, bodies, events));
    }

    fn stop(&mut self) -> usize {
        self.active = false;
        self.frames.len()
    }
}

/// Start capturing. Replaces any previous recording. `max_frames` is the ring
/// capacity; once full, the oldest frame is dropped.
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn debug_recording_start(world: *mut WorldHandle, max_frames: u32) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if max_frames == 0 || max_frames > MAX_OUTPUT_CAPACITY {
            set_error(ERR_INVALID_ARGUMENT, "max_frames out of range");
            return 0;
        }
        let buf = world
            .inner
            .debug_recording
            .get_or_insert_with(Default::default);
        buf.start(max_frames);
        ERR_OK
    })
}

/// Capture one frame of the current world state. No-op (returns ERR_OK) when
/// recording is not active — a hot caller can capture unconditionally after
/// every step without branching on the Java side.
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn debug_recording_capture(world: *mut WorldHandle) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let dt = world.inner.integration_parameters.dt;
        let events_enabled = world
            .inner
            .debug_recording
            .as_ref()
            .is_some_and(|b| b.active && b.events_enabled);
        // Pre-collect events before taking the &mut on the buffer (the event
        // queue lives on the same world struct — split borrows won't work
        // through Option::as_mut on the same field chain).
        let events = if events_enabled {
            collect_frame_events(&world.inner)
        } else {
            Vec::new()
        };
        match world.inner.debug_recording.as_mut() {
            Some(buf) if buf.active => {
                buf.capture(&world.inner.bodies, dt, events);
                ERR_OK
            }
            _ => ERR_OK, // inactive: silent no-op by contract
        }
    })
}

/// Collect this step's collision / contact-force events as v2 event slots.
/// Handles are the raw packed collider handles from the event records
/// (`CollisionEventRecord.collider1/2` are already packed u64s).
/// Errors are silent: a recording must never turn event collection into a
/// capture failure.
fn collect_frame_events(world: &crate::rapier::world::PhysicsWorld) -> Vec<[f64; SLOTS_PER_EVENT]> {
    const CAP: usize = 256;
    let mut events = Vec::new();
    let mut collisions = [crate::rapier::ffi::CollisionEventRecord::default(); CAP];
    let n = world.events.collision_events(&mut collisions) as usize;
    for c in &collisions[..n] {
        let kind = if c.started.0 != 0 {
            EVENT_COLLISION_STARTED
        } else {
            EVENT_COLLISION_STOPPED
        };
        events.push([
            kind as f64,
            f64::from_bits(c.collider1),
            f64::from_bits(c.collider2),
            0.0,
            0.0,
        ]);
    }
    let mut forces = [crate::rapier::ffi::ContactForceEventRecord::default(); CAP];
    let n = world.events.contact_force_events(&mut forces) as usize;
    for f in &forces[..n] {
        events.push([
            EVENT_CONTACT_FORCE as f64,
            f64::from_bits(f.collider1),
            f64::from_bits(f.collider2),
            f.total_force_magnitude,
            f.max_force_magnitude,
        ]);
    }
    world.events.clear();
    events
}

/// Stop capturing. Returns the retained frame count (oldest-dropped frames
/// are NOT counted).
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn debug_recording_stop(world: *mut WorldHandle) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        match world.inner.debug_recording.as_mut() {
            Some(buf) if buf.active => buf.stop() as u32,
            _ => {
                set_error(ERR_UNSUPPORTED, "recording not active");
                0
            }
        }
    })
}

/// Drop the retained recording (frees memory). Returns ERR_OK.
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn debug_recording_drop(world: *mut WorldHandle) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        world.inner.debug_recording = None;
        ERR_OK
    })
}

/// Size of one exported frame in f64 slots: magic(1) + count(1) + dt(1) +
/// per-body (handle 1 + 13 lanes).
fn frame_f64_len(body_count: usize) -> usize {
    3 + body_count * (1 + LANES_PER_BODY)
}

/// Export the retained recording into `out` (capacity `out_len` f64 slots).
/// Returns the number of f64 slots written. Blob layout: per frame,
/// `MPSDR01`-magic slot + body_count + dt + per body (packed handle + 13
/// lanes). A leading slot carries the frame count so a replay viewer can
/// allocate: `[frame_count, f64_per_frame_max, ...frames]`.
///
/// # Safety
/// `world` must be a valid world pointer (or null); `out` must point to
/// `out_len` writable f64s.
#[unsafe(no_mangle)]
pub extern "C" fn debug_recording_export(
    world: *mut WorldHandle,
    out: *mut f64,
    out_len: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let Some(buf) = world.inner.debug_recording.as_ref() else {
            set_error(ERR_UNSUPPORTED, "no recording");
            return 0;
        };
        if buf.active {
            set_error(ERR_UNSUPPORTED, "recording still active; call stop first");
            return 0;
        }
        let total: usize = buf
            .frames
            .iter()
            .map(|(_, b, e)| frame_f64_len(b.len()) + event_section_len(e.len()))
            .sum();
        let header = 2usize; // [frame_count, dropped]
        if out.is_null() {
            // Size query.
            return (total + header).min(u32::MAX as usize) as u32;
        }
        if (out_len as usize) < total + header {
            set_error(ERR_CAPACITY, "export buffer too small");
            return 0;
        }
        let out = unsafe { std::slice::from_raw_parts_mut(out, out_len as usize) };
        out[0] = buf.frames.len() as f64;
        out[1] = buf.dropped as f64;
        let mut i = header;
        for &(dt, ref bodies, ref events) in &buf.frames {
            let magic = if events.is_empty() {
                RECORDING_MAGIC_V1
            } else {
                RECORDING_MAGIC_V2
            };
            out[i] = f64::from_bits(magic);
            out[i + 1] = bodies.len() as f64;
            out[i + 2] = dt;
            i += 3;
            for &(h, lanes) in bodies {
                out[i] = f64::from_bits(h);
                out[i + 1..i + 1 + LANES_PER_BODY].copy_from_slice(&lanes);
                i += 1 + LANES_PER_BODY;
            }
            if !events.is_empty() {
                out[i] = events.len() as f64;
                i += 1;
                for e in events {
                    out[i..i + SLOTS_PER_EVENT].copy_from_slice(e);
                    i += SLOTS_PER_EVENT;
                }
            }
        }
        (i) as u32
    })
}

/// Read one retained frame (by ring index, 0 = oldest retained) into `out`
/// (capacity `out_len` f64 slots). Layout: `[body_count, dt, then per body
/// handle + 13 lanes]`. Returns lanes written or 0 on error / index OOB.
/// Used by a live viewer to poll frames without exporting the whole blob.
///
/// # Safety
/// `world` must be a valid world pointer (or null); `out` must point to
/// `out_len` writable f64s.
#[unsafe(no_mangle)]
pub extern "C" fn debug_recording_frame(
    world: *mut WorldHandle,
    frame_index: u32,
    out: *mut f64,
    out_len: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let Some(buf) = world.inner.debug_recording.as_ref() else {
            set_error(ERR_UNSUPPORTED, "no recording");
            return 0;
        };
        let Some(&(dt, ref bodies, ref events)) = buf.frames.get(frame_index as usize) else {
            set_error(ERR_NOT_FOUND, "frame index out of range");
            return 0;
        };
        let need = frame_f64_len(bodies.len()) - 1 + event_section_len(events.len()); // without the magic slot
        if out.is_null() || (out_len as usize) < need {
            set_error(ERR_CAPACITY, "frame buffer too small");
            return 0;
        }
        let out = unsafe { std::slice::from_raw_parts_mut(out, out_len as usize) };
        out[0] = bodies.len() as f64;
        out[1] = dt;
        let mut i = 2;
        for &(h, lanes) in bodies {
            out[i] = f64::from_bits(h);
            out[i + 1..i + 1 + LANES_PER_BODY].copy_from_slice(&lanes);
            i += 1 + LANES_PER_BODY;
        }
        if !events.is_empty() {
            out[i] = events.len() as f64;
            i += 1;
            for e in events {
                out[i..i + SLOTS_PER_EVENT].copy_from_slice(e);
                i += SLOTS_PER_EVENT;
            }
        }
        i as u32
    })
}

/// Event section length in f64 slots (0 when empty; else 1 count slot +
/// 5 slots per event).
fn event_section_len(event_count: usize) -> usize {
    if event_count == 0 {
        0
    } else {
        1 + event_count * SLOTS_PER_EVENT
    }
}

/// Enable (or disable) the v2 event section: subsequent captures record this
/// step's collision start/stop and contact-force events into the frame tail
/// and stamp frames with the `MPSDR02` magic. Frames captured while disabled
/// carry the `MPSDR01` magic and no event section — both old and new viewers
/// decode either layout via the magic check.
///
/// # Safety
/// `world` must be a valid world pointer (or null).
#[unsafe(no_mangle)]
pub extern "C" fn debug_recording_enable_events(world: *mut WorldHandle, enable: Bool) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        match world.inner.debug_recording.as_mut() {
            Some(buf) => {
                buf.events_enabled = enable == Bool::TRUE;
                ERR_OK
            }
            None => {
                set_error(ERR_UNSUPPORTED, "no recording; call start first");
                0
            }
        }
    })
}
