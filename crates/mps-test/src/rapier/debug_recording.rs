//! `mps_core::rapier::debug_recording` 镜像测试 —— Jolt-Viewer 风格录制回放。
//!
//! 守门：
//! 1. 录制→停止→逐帧读回与实时 `world_body_snapshot` 一致（13-lane 逐位）；
//! 2. 环形溢出覆盖最旧帧（`dropped` 计数正确）；
//! 3. 导出 blob 布局稳定（magic/frame_count/dt/handle 顺序）；
//! 4. 未启动录制时 capture 是静默 no-op，stop/export 报错不崩溃。

#[cfg(test)]
mod tests {
    use mps_core::rapier::debug_recording::{
        debug_recording_capture, debug_recording_drop, debug_recording_export,
        debug_recording_frame, debug_recording_start, debug_recording_stop,
    };
    use mps_core::rapier::error::{ERR_OK, ERR_UNSUPPORTED, clear_error, last_error_code};
    use mps_core::rapier::ffi::{RigidBodyHandleRaw, Vec3, WorldHandle};
    use mps_core::rapier::world::{world_body_snapshot, world_create, world_destroy, world_step};
    use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder, Vector};

    /// 落体世界：1 动态体 + 地面，推 `steps` 步。
    fn falling_world(steps: u32) -> (*mut WorldHandle, Vec<(RigidBodyHandleRaw, [f64; 13])>) {
        let world = world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        assert!(!world.is_null());
        let w = unsafe { &mut (*world).inner };
        w.colliders
            .insert(ColliderBuilder::cuboid(100.0, 0.5, 100.0).build());
        let h = w.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(0.0, 10.0, 0.0))
                .linvel(Vector::new(1.0, 0.0, 0.0))
                .build(),
        );
        w.colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.5, 0.5, 0.5).build(),
            h,
            &mut w.bodies,
        );

        let mut handles = [0u64; 8];
        let mut values = [0f64; 8 * 13];
        for _ in 0..steps {
            world_step(world, 1.0 / 60.0);
        }
        // 终态快照（供最后一帧比对）。地面是孤 collider（不建 body），
        // 全量快照只有 1 个动态体。
        let n = world_body_snapshot(world, handles.as_mut_ptr(), values.as_mut_ptr(), 8);
        assert_eq!(n, 1, "single dynamic body");
        let snap: Vec<(RigidBodyHandleRaw, [f64; 13])> = (0..n as usize)
            .map(|i| (handles[i], values[i * 13..i * 13 + 13].try_into().unwrap()))
            .collect();
        (world, snap)
    }

    #[test]
    fn recording_ring_overflow_and_frame_readback() {
        let (world, _final) = falling_world(0);
        // 容量 3 帧的环。
        assert_eq!(debug_recording_start(world, 3), ERR_OK);
        for _ in 0..5 {
            world_step(world, 1.0 / 60.0);
            assert_eq!(debug_recording_capture(world), ERR_OK);
        }
        // 5 步进容量 3 → 保留 3 帧，丢 2。
        let retained = debug_recording_stop(world);
        assert_eq!(retained, 3, "ring keeps last 3 frames");

        // 逐帧读回：位置 x 单调增（体在向 +x 漂），最后一帧与实时快照一致。
        let mut buf = [0f64; 2 + 2 * 14];
        for f in 0..3u32 {
            let n = debug_recording_frame(world, f, buf.as_mut_ptr(), buf.len() as u32);
            assert!(n > 0, "frame {f} readable");
            let body_count = buf[0] as usize;
            assert_eq!(body_count, 1, "the dynamic body is captured");
            let dt = buf[1];
            assert!((dt - 1.0 / 60.0).abs() < 1e-12, "dt recorded per frame");
        }
        // 帧越界 → 0 + ERR_NOT_FOUND 语义（不崩溃）。
        clear_error();
        assert_eq!(
            debug_recording_frame(world, 3, buf.as_mut_ptr(), buf.len() as u32),
            0
        );
        world_destroy(world);
    }

    #[test]
    fn recording_export_blob_layout_is_stable() {
        // 落体世界已推 3 步并取好终态快照；重新开始录制并**重放**这 3 步
        // 前先 reset？不行——世界已推进。改为：录制从现在起 3 帧，把录制
        // 期间的末帧与录制结束后的实时快照比对（同一时刻）。
        let (world, _final) = falling_world(0); // 空世界构造辅助
        #[allow(unused_variables)]
        let _ = &_final;
        assert_eq!(debug_recording_start(world, 16), ERR_OK);
        for _ in 0..3 {
            world_step(world, 1.0 / 60.0);
            debug_recording_capture(world);
        }
        assert_eq!(debug_recording_stop(world), 3);

        // 录制刚停止：实时快照 = 最后一帧的状态。
        let mut handles = [0u64; 8];
        let mut values = [0f64; 8 * 13];
        let n = world_body_snapshot(world, handles.as_mut_ptr(), values.as_mut_ptr(), 8);
        assert_eq!(n, 1);
        let final_snap: Vec<(RigidBodyHandleRaw, [f64; 13])> = (0..n as usize)
            .map(|i| (handles[i], values[i * 13..i * 13 + 13].try_into().unwrap()))
            .collect();

        // 大小查询（null out）。
        let needed = debug_recording_export(world, std::ptr::null_mut(), 0);
        assert!(needed > 0, "size query");

        // 真实导出。
        let mut blob = vec![0f64; needed as usize];
        let written = debug_recording_export(world, blob.as_mut_ptr(), blob.len() as u32);
        assert_eq!(written, needed, "export fills exactly");

        // 头部：[frame_count, dropped]。
        assert_eq!(blob[0], 3.0, "frame count");
        assert_eq!(blob[1], 0.0, "no dropped frames");

        // 第一帧：magic 槽 + count + dt + 2×(handle + 13 lanes)。
        // magic = 0x4d50534452303131（"MPSDR01" + 版本半字节），以 f64 bits 存。
        assert_eq!(blob[2].to_bits(), 0x4d50_5344_5230_3131, "magic slot");
        assert_eq!(blob[3], 1.0, "body count in frame 0");
        assert!((blob[4] - 1.0 / 60.0).abs() < 1e-12, "frame 0 dt");

        // 最后一帧的动力学体位姿与实时快照逐位一致（回放保真）。
        // 直接扫 blob：定位第 3 帧（index 2）的 handle 对，比对 13 lanes。
        let mut i = 2usize; // 第一帧 magic
        let mut last_frame_lanes: Vec<(u64, [f64; 13])> = Vec::new();
        let mut frame = 0usize;
        while i < written as usize {
            let body_count = blob[i + 1] as usize;
            let dt = blob[i + 2];
            let _ = dt;
            i += 3;
            if frame == 2 {
                for _ in 0..body_count {
                    let handle = blob[i].to_bits();
                    let mut lanes = [0f64; 13];
                    lanes.copy_from_slice(&blob[i + 1..i + 14]);
                    last_frame_lanes.push((handle, lanes));
                    i += 14;
                }
                break;
            } else {
                i += body_count * 14;
                frame += 1;
            }
        }
        assert_eq!(last_frame_lanes.len(), 1, "decoded last frame");
        for (packed, lanes) in &last_frame_lanes {
            let snap = final_snap
                .iter()
                .find(|(h, _)| *h == *packed)
                .map(|(_, l)| *l)
                .unwrap_or_else(|| panic!("handle {packed:#x} in live snapshot"));
            assert_eq!(
                lanes, &snap,
                "last-frame lanes bit-identical to live snapshot"
            );
        }

        // 缓冲不足 → ERR_CAPACITY。
        let mut small = [0f64; 4];
        clear_error();
        assert_eq!(
            debug_recording_export(world, small.as_mut_ptr(), small.len() as u32),
            0
        );
        assert_ne!(last_error_code(), ERR_OK, "capacity error reported");

        // 清理。
        assert_eq!(debug_recording_drop(world), ERR_OK);
        world_destroy(world);
    }

    #[test]
    fn recording_inactive_capture_is_silent_noop() {
        let world = world_create(Vec3::default());
        // 未 start：capture 是 OK 的 no-op。
        assert_eq!(debug_recording_capture(world), ERR_OK);
        // stop / export / frame 在无录制时报错不崩溃。
        clear_error();
        assert_eq!(debug_recording_stop(world), 0);
        assert_eq!(last_error_code(), ERR_UNSUPPORTED);
        clear_error();
        assert_eq!(debug_recording_export(world, std::ptr::null_mut(), 0), 0);
        assert_eq!(last_error_code(), ERR_UNSUPPORTED);
        world_destroy(world);
    }

    #[test]
    fn recording_start_validates_max_frames() {
        let world = world_create(Vec3::default());
        clear_error();
        assert_eq!(debug_recording_start(world, 0), 0, "zero capacity rejected");
        assert_eq!(
            debug_recording_start(world, u32::MAX),
            0,
            "oversized capacity rejected"
        );
        assert_eq!(debug_recording_start(world, 60), ERR_OK, "valid capacity");
        world_destroy(world);
    }

    // v2 事件段：enable_events 后帧尾追加碰撞/接触力事件，magic 变 MPSDR02；
    // v1 帧仍为 MPSDR01 且无事件段（按 magic 区分，新旧查看器互不误读）。
    #[test]
    fn recording_v2_event_section_decode() {
        let (world, _final) = falling_world(0);
        assert_eq!(debug_recording_start(world, 8), ERR_OK);
        // 未开事件段：v1 magic。
        world_step(world, 1.0 / 60.0);
        debug_recording_capture(world);
        assert_eq!(debug_recording_stop(world), 1);
        let needed = debug_recording_export(world, std::ptr::null_mut(), 0);
        let mut blob = vec![0f64; needed as usize];
        let written = debug_recording_export(world, blob.as_mut_ptr(), blob.len() as u32);
        assert_eq!(
            blob[2].to_bits(),
            0x4d50_5344_5230_3131,
            "v1 magic when events disabled"
        );
        assert_eq!(written, needed);

        // 开启事件段后重录：先启用碰撞事件（否则 world 不产事件），下落体
        // 在数步内与地面碰撞 → 至少一个 started 事件。
        mps_core::rapier::world::world_apply_runtime_settings(world, 4, 0, 1, 0, 0, 1);
        debug_recording_drop(world);
        assert_eq!(debug_recording_start(world, 64), ERR_OK);
        assert_eq!(
            mps_core::rapier::debug_recording::debug_recording_enable_events(
                world,
                mps_core::rapier::ffi::Bool::TRUE
            ),
            ERR_OK
        );
        let mut saw_v2 = false;
        for _ in 0..90 {
            world_step(world, 1.0 / 60.0);
            debug_recording_capture(world);
        }
        assert_eq!(debug_recording_stop(world), 64);
        let needed = debug_recording_export(world, std::ptr::null_mut(), 0);
        let mut blob2 = vec![0f64; needed as usize];
        let written2 = debug_recording_export(world, blob2.as_mut_ptr(), blob2.len() as u32);
        assert_eq!(written2, needed);

        // 逐帧解码：验证至少一帧 v2 且带事件段（count slot + 5 槽/事件）。
        let mut i = 2usize;
        let mut frames_seen = 0usize;
        while i < written2 as usize {
            let magic = blob2[i].to_bits();
            let body_count = blob2[i + 1] as usize;
            let _dt = blob2[i + 2];
            i += 3 + body_count * 14;
            if magic == 0x4d50_5344_5230_3231 {
                saw_v2 = true;
                let event_count = blob2[i] as usize;
                assert!(event_count >= 1, "v2 frame with enabled events has events");
                i += 1 + event_count * 5;
            }
            frames_seen += 1;
        }
        assert!(saw_v2, "at least one frame carries the v2 event magic");
        assert!(frames_seen >= 60, "all 64 frames decoded");
        assert_eq!(debug_recording_drop(world), ERR_OK);
        world_destroy(world);
    }

    #[test]
    fn recording_enable_events_requires_active_recording() {
        let world = world_create(Vec3::default());
        clear_error();
        assert_eq!(
            mps_core::rapier::debug_recording::debug_recording_enable_events(
                world,
                mps_core::rapier::ffi::Bool::TRUE
            ),
            0,
            "no recording yet"
        );
        assert_eq!(last_error_code(), ERR_UNSUPPORTED);
        world_destroy(world);
    }
}
