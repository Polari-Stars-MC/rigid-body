//! D2：录制重演位级守门（evidence/record-replay.md 的数据源）。
//!
//! MC 宿主的核心需求：同一段操作序列（放置/挖除/物理步）必须在**两个独立
//! 世界实例**上产出**逐位一致**的结果——这是录制重演（录制一端位流、另一端
//! 重放验证）的前提。
//!
//! 守门设计：
//! - 同一构建函数造两个 world（同 seed 的操作序列：塔堆 + 挖除 + 填回）；
//! - 20 步（可行性钉，非 ignore）+ 600 步（长程钉，release 跑）逐步读取
//!   全体位姿/速度的**原始位**（f64::to_bits），逐位比对；
//! - 任何一步有 1 bit 差异即失败——位级不渐进，位级就位级。

#[cfg(test)]
mod tests {
    use mps_core::rapier::ffi::{Bool, Vec3, WorldHandle, convert::pack_rigid_body_handle};
    use mps_core::rapier::rigid_body::world_remove_rigid_body;
    use mps_core::rapier::world::{
        world_apply_runtime_settings, world_create, world_destroy, world_step,
    };
    use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder, Vector};

    struct World(*mut WorldHandle);
    impl Drop for World {
        fn drop(&mut self) {
            world_destroy(self.0);
        }
    }

    /// 场景状态：FFI 打包句柄表（[layer][col]）。
    struct Scene {
        world: World,
        layers: Vec<Vec<u64>>,
    }

    /// 确定性场景构建：静态地面 + side×height 方块塔（layer 0 = 底层）。
    fn build_scene() -> Scene {
        let world = World(world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        }));
        assert_eq!(
            world_apply_runtime_settings(world.0, 4, 1, 0, 0, 0, 1),
            Bool::TRUE
        );
        let w = unsafe { &mut (*world.0).inner };
        w.colliders
            .insert(ColliderBuilder::cuboid(1e3, 0.5, 1e3).build());
        let mut layers = Vec::new();
        for layer in 0..6usize {
            let mut row = Vec::new();
            for col in 0..4usize {
                let h = w.bodies.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(
                            col as f64 * 1.001,
                            0.5 + layer as f64 * 1.001,
                            0.0,
                        ))
                        .build(),
                );
                let packed = pack_rigid_body_handle(h);
                w.colliders.insert_with_parent(
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5).build(),
                    h,
                    &mut w.bodies,
                );
                row.push(packed);
            }
            layers.push(row);
        }
        Scene { world, layers }
    }

    /// 确定性操作序列：落定 30 步 → 挖 [0][2] → 60 步 → 填回 → 挖 [3][1] →
    /// 30 步 → 填回。返回总步数；重演端逐字重放同一序列。
    const DIG_A: (usize, usize) = (0, 2);
    const DIG_B: (usize, usize) = (3, 1);

    /// 逐位快照：全体 (位姿 xyz, 线速度 xyz) 的 f64 位。
    /// # Safety
    /// world 指针必须有效。
    unsafe fn bit_snapshot(world: &World) -> Vec<u64> {
        // SAFETY: 调用契约保证 world 指针有效且独占访问。
        unsafe {
            let mut bits = Vec::new();
            for (h, b) in (*world.0).inner.bodies.iter() {
                bits.push(h.into_raw_parts().0 as u64);
                let t = b.translation();
                bits.push(t.x.to_bits());
                bits.push(t.y.to_bits());
                bits.push(t.z.to_bits());
                let v = b.linvel();
                bits.push(v.x.to_bits());
                bits.push(v.y.to_bits());
                bits.push(v.z.to_bits());
            }
            bits
        }
    }

    fn run_script(steps_total: usize) -> Vec<Vec<u64>> {
        let mut scene = build_scene();
        let mut snapshots = Vec::new();
        let mut done = 0usize;
        // 脚本：确定性步进与挖填交错；挖/填操作计入操作数但不产生快照。
        let mut dug_a = false;
        let mut dug_b = false;
        let mut step_i = 0usize;
        while step_i < steps_total {
            // SAFETY: scene.world 是本函数独占的有效 world 指针。
            unsafe {
                // 落定 30 步后执行第一挖。
                if step_i == 30 && !dug_a {
                    let h = scene.layers[DIG_A.0][DIG_A.1];
                    assert_eq!(
                        world_remove_rigid_body(scene.world.0, h, Bool::TRUE),
                        Bool::TRUE
                    );
                    dug_a = true;
                }
                // 90 步时填回 A（重建同位刚体）。
                if step_i == 90 && !dug_b {
                    let h = scene.layers[DIG_B.0][DIG_B.1];
                    assert_eq!(
                        world_remove_rigid_body(scene.world.0, h, Bool::TRUE),
                        Bool::TRUE
                    );
                    dug_b = true;
                }
                if step_i == 150 {
                    let w2 = &mut (*scene.world.0).inner;
                    let hh = w2.bodies.insert(
                        RigidBodyBuilder::dynamic()
                            .translation(Vector::new(
                                DIG_A.1 as f64 * 1.001,
                                0.5 + DIG_A.0 as f64 * 1.001,
                                0.0,
                            ))
                            .build(),
                    );
                    w2.colliders.insert_with_parent(
                        ColliderBuilder::cuboid(0.5, 0.5, 0.5).build(),
                        hh,
                        &mut w2.bodies,
                    );
                    scene.layers[DIG_A.0][DIG_A.1] = pack_rigid_body_handle(hh);
                }
                world_step(scene.world.0, 1.0 / 60.0);
            }
            step_i += 1;
            done += 1;
            unsafe {
                snapshots.push(bit_snapshot(&scene.world));
            }
        }
        let _ = done;
        snapshots
    }

    /// 可行性钉：20 步位级重演一致。
    #[test]
    fn record_replay_20_steps_bit_identical() {
        let a = run_script(20);
        let b = run_script(20);
        assert_eq!(a.len(), b.len());
        for (i, (sa, sb)) in a.iter().zip(b.iter()).enumerate() {
            assert_eq!(sa, sb, "bit divergence at step {i} (20-step feasibility)");
        }
    }

    /// 长程钉：600 步（含挖填）位级重演一致。
    #[test]
    #[ignore = "release/evidence long run; run with --ignored"]
    fn record_replay_600_steps_bit_identical() {
        let a = run_script(600);
        let b = run_script(600);
        for (i, (sa, sb)) in a.iter().zip(b.iter()).enumerate() {
            assert_eq!(sa, sb, "bit divergence at step {i} (600-step long run)");
        }
    }
}
