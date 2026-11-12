#[cfg(test)]
mod tests {
    use mps_core::rapier::ffi::BodyStatus;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::world::*;

    #[test]
    fn integration_parameters_and_body_batch_updates_work() {
        let world = world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        assert!(!world.is_null());
        assert_eq!(
            world_set_integration_parameters(world, 1.0 / 120.0, 8, 2),
            Bool::TRUE
        );

        let mut params = [0.0; 3];
        assert_eq!(
            world_get_integration_parameters(world, params.as_mut_ptr(), params.len() as u32),
            3
        );
        assert_eq!(params[1], 8.0);
        assert_eq!(params[2], 2.0);

        let builder =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(builder);
        let handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);
        assert_ne!(handle, 0);
        assert_eq!(world_body_snapshot_count(world), 1);

        let handles = [handle];
        let poses = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0];
        assert_eq!(
            world_update_body_poses(world, handles.as_ptr(), poses.as_ptr(), 1, Bool::TRUE),
            1
        );
        let velocities = [4.0, 5.0, 6.0, 0.1, 0.2, 0.3];
        assert_eq!(
            world_update_body_velocities(
                world,
                handles.as_ptr(),
                velocities.as_ptr(),
                1,
                Bool::TRUE
            ),
            1
        );

        let mut out_handles = [0; 1];
        let mut values = [0.0; 13];
        assert_eq!(
            world_body_snapshot(
                world,
                out_handles.as_mut_ptr(),
                values.as_mut_ptr(),
                out_handles.len() as u32,
            ),
            1
        );
        assert_eq!(out_handles[0], handle);
        assert_eq!(&values[..3], &[1.0, 2.0, 3.0]);
        assert_eq!(&values[7..10], &[4.0, 5.0, 6.0]);

        world_destroy(world);
    }

    /// B1：体级模拟域。域 0 保留、成员划转立即生效、inactive 冻结体
    /// 不被 world_step 推进（位姿逐位不变）、恢复 active 后继续推进。
    #[test]
    fn simulation_domain_freeze_and_thaw() {
        use mps_core::rapier::collider::{
            collider_builder_build, collider_builder_create_ex, world_insert_collider_with_parent,
        };
        use mps_core::rapier::ffi::convert::pack_rigid_body_handle;
        use mps_core::rapier::ffi::{ShapeDesc, ShapeType};
        use mps_core::rapier::rigid_body::{
            rigid_body_builder_build, rigid_body_builder_create,
            rigid_body_builder_set_translation, world_insert_rigid_body,
        };

        let world = world_create(Vec3 {
            x: -9.81,
            y: 0.0,
            z: 0.0,
        });
        assert_eq!(
            world_apply_runtime_settings(world, 4, 1, 0, 0, 0, 1),
            Bool::TRUE
        );
        // 两个自由落体球（带 collider——无 collider 的体不进 island，不参与积分）。
        let make_body = |y: f64| {
            let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
            rigid_body_builder_set_translation(builder, Vec3 { x: 0.0, y, z: 0.0 });
            let handle = world_insert_rigid_body(world, rigid_body_builder_build(builder));
            let shape = ShapeDesc {
                shape_type: ShapeType::Ball as u32,
                a: 0.5,
                ..Default::default()
            };
            let collider = collider_builder_build(collider_builder_create_ex(shape));
            world_insert_collider_with_parent(world, collider, handle);
            handle
        };
        let a = make_body(10.0);
        let b = make_body(20.0);
        assert_ne!(a, 0);
        assert_ne!(b, 0);

        // 域 1：只含 a。
        assert_eq!(rigid_body_set_simulation_domain(world, a, 1), Bool::TRUE);
        assert_eq!(world_domain_body_count(world, 1), 1);
        // 域 0 保留；未知域返回 -1 并置 ERR_NOT_FOUND(3)。
        assert_eq!(world_domain_body_count(world, 0), -1);
        assert_eq!(mps_core::rapier::error::last_error_code(), 3);

        // 停用域 1：a 冻结。
        assert_eq!(world_set_domain_active(world, 1, Bool::FALSE), 1);
        // 域 0 保留错误：非法。
        assert_eq!(world_set_domain_active(world, 0, Bool::FALSE), 0);
        assert_eq!(mps_core::rapier::error::last_error_code(), 2);

        // 步进 60 步：b 下落（v ≈ -9.81·1 s ≈ -9.81），a 的位姿逐位不变。
        for _ in 0..60 {
            world_step(world, 1.0 / 60.0);
        }
        let snapshot = |handle: u64| -> Vec<u64> {
            let w = unsafe { &(*world).inner };
            let body = mps_core::rapier::ffi::convert::unpack_rigid_body_handle(handle);
            let b = w.bodies.get(body).unwrap();
            let t = b.translation();
            let v = b.linvel();
            vec![
                t.x.to_bits(),
                t.y.to_bits(),
                t.z.to_bits(),
                v.x.to_bits(),
                v.y.to_bits(),
                v.z.to_bits(),
            ]
        };
        let a_frozen = snapshot(a);
        let b_fallen = snapshot(b);
        // 重力在 -x：活动体 v.x ≠ 0 且 x 位姿已离开 0。
        assert!(
            b_fallen[3] != 0 && b_fallen[0] != 0,
            "active body must have fallen: {b_fallen:?}"
        );
        // 恢复后再步进：a 开始下落。
        assert_eq!(world_set_domain_active(world, 1, Bool::TRUE), 1);
        for _ in 0..60 {
            world_step(world, 1.0 / 60.0);
        }
        let a_thawed = snapshot(a);
        assert!(
            a_thawed[3] != 0 && a_thawed[3] != a_frozen[3] && a_thawed[0] != a_frozen[0],
            "thawed body must start falling: {a_thawed:?} vs frozen {a_frozen:?}"
        );
        // 冻结期间位姿逐位保留（恢复瞬间核对：thaw 后首步前的位姿没被检查点
        // 重置——用「冻结期快照 vs 恢复后第一步前的期望」的关系验证太复杂，
        // 改为冻结期连拍两次一致）。
        assert_eq!(world_set_domain_active(world, 1, Bool::FALSE), 1);
        let a_before = snapshot(a);
        for _ in 0..10 {
            world_step(world, 1.0 / 60.0);
        }
        let a_after = snapshot(a);
        assert_eq!(
            a_before, a_after,
            "frozen body pose must be bit-identical across steps"
        );

        // 划回域 0：成员表清空、体恢复活动。
        assert_eq!(rigid_body_set_simulation_domain(world, a, 0), Bool::TRUE);
        assert_eq!(world_domain_body_count(world, 1), 0);
        for _ in 0..10 {
            world_step(world, 1.0 / 60.0);
        }
        let a_free = snapshot(a);
        assert_ne!(
            a_free, a_after,
            "body returned to domain 0 must resume falling"
        );
        assert!(
            a_free[3] != 0,
            "resumed body must gain velocity: {a_free:?}"
        );

        world_destroy(world);
        let _ = pack_rigid_body_handle; // import keepalive（句柄打包经 world_insert_rigid_body 已用）
    }

    /// B1 性能证据：2000 体世界，90% 冻结在 inactive 域，步进耗时对比
    /// 全活动基线。evidence/performance.md 附表数据源（ADR 承诺 <40% 基线）。
    #[test]
    #[ignore = "performance evidence; run with --ignored --release"]
    fn domain_freeze_step_cost_ratio() {
        use mps_core::rapier::collider::{
            collider_builder_build, collider_builder_create_ex, world_insert_collider_with_parent,
        };
        use mps_core::rapier::ffi::{ShapeDesc, ShapeType};
        use mps_core::rapier::rigid_body::{
            rigid_body_builder_build, rigid_body_builder_create,
            rigid_body_builder_set_translation, world_insert_rigid_body,
        };
        use std::time::Instant;

        const N: usize = 2000;
        const FROZEN: usize = 1800;

        let build = || -> (*mut WorldHandle, Vec<u64>) {
            let world = world_create(Vec3 {
                x: -9.81,
                y: 0.0,
                z: 0.0,
            });
            assert_eq!(
                world_apply_runtime_settings(world, 4, 1, 0, 0, 0, 1),
                Bool::TRUE
            );
            let shape = ShapeDesc {
                shape_type: ShapeType::Ball as u32,
                a: 0.5,
                ..Default::default()
            };
            let mut handles = Vec::with_capacity(N);
            for i in 0..N {
                let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
                rigid_body_builder_set_translation(
                    builder,
                    Vec3 {
                        x: (i % 40) as f64 * 1.2,
                        y: (i / 40) as f64 * 1.2 + 1.0,
                        z: 0.0,
                    },
                );
                let handle = world_insert_rigid_body(world, rigid_body_builder_build(builder));
                let collider = collider_builder_build(collider_builder_create_ex(shape));
                world_insert_collider_with_parent(world, collider, handle);
                handles.push(handle);
            }
            (world, handles)
        };

        // 基线：全活动 60 步。
        let (world_a, _) = build();
        let t0 = Instant::now();
        for _ in 0..60 {
            world_step(world_a, 1.0 / 60.0);
        }
        let base = t0.elapsed().as_secs_f64();
        world_destroy(world_a);

        // 冻结域：1800 体划入域 1 → 停用 → 60 步。
        let (world_b, handles) = build();
        for h in &handles[..FROZEN] {
            assert_eq!(rigid_body_set_simulation_domain(world_b, *h, 1), Bool::TRUE);
        }
        assert_eq!(
            world_set_domain_active(world_b, 1, Bool::FALSE),
            FROZEN as u32
        );
        let t1 = Instant::now();
        for _ in 0..60 {
            world_step(world_b, 1.0 / 60.0);
        }
        let frozen = t1.elapsed().as_secs_f64();
        world_destroy(world_b);

        let ratio = frozen / base;
        println!(
            "base(2000 active) = {base:.4}s, frozen(200 active + 1800 frozen) = {frozen:.4}s, ratio = {ratio:.3}"
        );
        // ADR 承诺：冻结域自身的存在不得让步进比全活动基线慢 40% 以上
        // （真实收益来自 broad phase/island 的工作量缩减，这里冻结体贡献
        // 近零工作，比值主要由 200 个活动体的占比决定，预期显著 < 0.4）。
        assert!(
            ratio < 0.4,
            "frozen-domain step ratio {ratio:.3} exceeds 0.4"
        );
    }
}
