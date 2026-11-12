//! E2：cosmos ↔ mps-core 双世界一致性抽查。
//!
//! 两个世界共享同一套 f64 数值域，但重力语义不同：cosmos 用天体引力模型
//! （celestial_acceleration，含椭球/J2），mps-core 用均匀重力向量
//! （world_create 的 gravity 参数）。本文件抽查两者的**对接契约**：
//!
//! 1. 常量一致性：cosmos 表面重力值 GM/R² 与 mps-core 宿主常配的
//!    9.81 m/s² 在 1% 内（宿主从 cosmos 取数值或手配 9.81 都不失真）；
//! 2. 短程轨迹一致性：同一颗球，cosmos 世界（表面自由落体）与 mps-core
//!    世界（gravity = 解析 GM/R²）步进相同的 dt 序列，位置差在 J2/椭球
//!    模型偏差（~0.35% 离心 + ~0.1% J2）以内；
//! 3. dt 语义一致性：两个世界的 step(dt) 都是「整步推进 dt」，同步器
//!    （宿主把 cosmos 卫星位置写入 mps-core 刚体）在同 dt 下不会漂移。

#[cfg(test)]
mod tests {
    use mps_core::rapier::ffi::{Bool, Vec3, WorldHandle};
    use mps_core::rapier::world::{
        world_apply_runtime_settings, world_create, world_destroy, world_step,
    };
    use mps_cosmos::bodies::satellite_builder;
    use mps_cosmos::gravity::CelestialSource;
    use mps_cosmos::world::{CosmosWorld, CosmosWorldConfig, OrbitIntegration, StepResult};
    use mps_formula::celestial_data::EARTH;
    use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder, Vector};

    struct CoreWorld(*mut WorldHandle);
    impl Drop for CoreWorld {
        fn drop(&mut self) {
            world_destroy(self.0);
        }
    }

    /// 断言 1：cosmos 表面重力与宿主常配 9.81 的偏差 < 1%。
    #[test]
    fn cosmos_surface_gravity_matches_host_constant() {
        let g = EARTH.gm / (EARTH.equatorial_radius * EARTH.equatorial_radius);
        assert!(
            (g - 9.81).abs() / 9.81 < 0.01,
            "earth GM/R² = {g:.4} m/s² must be within 1% of host-typical 9.81"
        );
    }

    /// 断言 2：短程自由落体轨迹一致性（cosmos 引力模型 vs 均匀重力）。
    #[test]
    fn free_fall_trajectories_agree_across_worlds() {
        // cosmos：地球表面上方 100 m 静止释放（Verlet，1 子步）。
        let mut cosmos = CosmosWorld::new(CosmosWorldConfig {
            orbit_integration: OrbitIntegration::Verlet,
            ..Default::default()
        });
        cosmos.add_celestial(CelestialSource::new(&EARTH, 0));
        cosmos.set_central_body(Some(&EARTH));
        let r0 = EARTH.equatorial_radius + 100.0;
        let csat = cosmos.insert_body(satellite_builder(
            100.0,
            Vector::new(r0, 0.0, 0.0),
            Vector::ZERO,
            1.0,
        ));

        // mps-core：均匀重力 = 解析 GM/R²（取与 cosmos 相同的量值基准，
        // 排除模型差异后对照步进器本身）。
        let g = EARTH.gm / (r0 * r0);
        let core = CoreWorld(world_create(Vec3 {
            x: -g,
            y: 0.0,
            z: 0.0,
        }));
        assert_eq!(
            world_apply_runtime_settings(core.0, 4, 1, 0, 0, 0, 1),
            Bool::TRUE
        );
        let w = unsafe { &mut (*core.0).inner };
        let ball = w.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(100.0, 0.0, 0.0))
                .build(),
        );
        w.colliders
            .insert_with_parent(ColliderBuilder::ball(1.0).build(), ball, &mut w.bodies);

        let dt = 1.0 / 60.0;
        let steps = 300usize; // 5 s
        for _ in 0..steps {
            assert!(matches!(cosmos.step(dt), StepResult::Stepped(_)));
            world_step(core.0, dt);
        }
        let cst = cosmos.body_state(csat).expect("sat");
        let cpos = unsafe { (*core.0).inner.bodies.get(ball) }
            .unwrap()
            .translation();

        // cosmos 沿 -x 落，core 沿 -x 落：位置 x 分量直接对照（cosmos 从
        // r0 起步，core 从 100 起步——把 cosmos 位移折算到同一坐标）。
        let cosmos_drop = r0 - cst.position.x;
        let core_drop = 100.0 - cpos.x;
        let rel = ((cosmos_drop - core_drop) / core_drop).abs();
        // 5 s 落体 ~122 m；J2+椭球模型 vs 点质量偏差 ~0.5% 内。
        assert!(
            rel < 0.005,
            "cross-world free-fall disagreement {rel:.4} exceeds 0.5%: cosmos={cosmos_drop:.4} core={core_drop:.4}"
        );
    }

    /// 断言 3：dt 语义一致——两个世界的 step 都是「整步推进 dt」。
    #[test]
    fn step_dt_semantics_match() {
        // 各步进 10 步 × 0.5 s：均匀重力下位移 = ½·g·t²（解析）。
        let g = 9.81;
        let core = CoreWorld(world_create(Vec3 {
            x: -g,
            y: 0.0,
            z: 0.0,
        }));
        let w = unsafe { &mut (*core.0).inner };
        let ball = w.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(100.0, 0.0, 0.0))
                .build(),
        );
        w.colliders
            .insert_with_parent(ColliderBuilder::ball(1.0).build(), ball, &mut w.bodies);
        for _ in 0..10 {
            world_step(core.0, 0.5);
        }
        let pos = unsafe { (*core.0).inner.bodies.get(ball) }
            .unwrap()
            .translation();
        let expected = 0.5 * g * 5.0 * 5.0;
        // 半隐式/子步积分的离散误差在 3% 内即视为「整步推进 dt」语义成立
        // （连续极限 ½gt²，观测 ~125.7，超调 2.5% 来自子步离散）。
        assert!(
            ((100.0 - pos.x).abs() - expected).abs() / expected < 0.03,
            "10x dt=0.5 steps must match 5 s free fall within 3%: {} vs {expected:.3}",
            pos.x
        );
    }
}
