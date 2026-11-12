# debug_recording.rs — Jolt-Viewer 风格录制回放

## 职责

环形录制缓冲 + 逐帧刚体快照，给 Java 宿主（MC mod）提供「录一段物理
状态 → 离线回放/实时预览」的能力。与 `events.rs` 互补：events 是即时
消费的队列，本模块是**回放型**留存。

## 数据流

```
debug_recording_start(world, max_frames)   开录（环容量帧数，溢出覆盖最旧）
    ↓
world_step → debug_recording_capture       每帧捕获（未录制时静默 no-op）
    ↓
debug_recording_stop                       返回保留帧数
    ↓
debug_recording_frame(world, idx, …)       逐帧读（实时预览）
debug_recording_export(world, out, len)    整段导出（两段式，回放器用）
debug_recording_drop(world)                释放
```

## blob 布局（little-endian f64 槽）

- 头部：`[frame_count, dropped]`
- 逐帧：`magic | body_count | dt` + 逐体 `(packed_handle, 13 lanes)`
  - magic：`MPSDR01`（v1，无事件段）/ `MPSDR02`（v2，帧尾带事件段）
  - 13 lanes：pos3 + quat4 + linvel3 + angvel3（与 `world_body_snapshot`
    同款，快照复用 `parallel::body_pose_snapshots`）
  - packed_handle = `(generation<<32 | index) + 1`
- v2 事件段（`debug_recording_enable_events` 开启后）：
  `event_count` + 逐事件 5 槽 `(kind, colliderA, colliderB, force, maxForce)`
  kind：1=碰撞开始、2=碰撞结束、3=接触力

## v1/v2 兼容

magic 槽标识版本：v1 解码器读到 `MPSDR01` 停在 body 表尾；v2 解码器对
`MPSDR01` 帧跳过事件段。新旧查看器互不误读——不需要版本协商。

## FFI 入口（全部走 `ffi_guard`）

`debug_recording_start` / `_capture` / `_stop` / `_drop` / `_export` /
`_frame` / `_enable_events`。

## 测试

- Rust：`mps-test/src/rapier/debug_recording.rs`（环形溢出、blob 布局、
  v2 事件段解码、inactive 守门）
- Java：`crates/mps-test/java/debug-recording/`（JNI 全链路）与
  `crates/mps-test/java/recording-events/`（事件消费）
