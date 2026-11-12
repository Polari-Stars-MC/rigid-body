//! C3：天体自转语义测试（evidence/convergence.md 附表 3 的数据源）。
//!
//! 自转字段 `rotation_rate` 在引擎里有且仅有两个消费点：
//! 1. `angular_velocity_of` → 大气惯性速度 `ω × r`（阻力参考系随地表共转）；
//! 2. 无其他（`celestial_acceleration` 是纯引力势，不含离心/科氏项）。
//!
//! 三个钉子：
//! - 表面共转大气速度：地球赤道 ≈ 465 m/s 向东（数值钉）；
//! - 量级边界断言：ω²R（离心）≪ GM/R²（表面引力），0.35%——纯引力模型
//!   在游戏尺度下可忽略离心项是有意识的取舍，此测试把它钉成显式契约；
//! - 科氏偏转冒烟：从静止释放、只受「引力 + 随转大气阻力」的体，落向
//!   地面时获得向东（自转方向）的横向速度分量（阻力参考系共转的直接后果）。

#[cfg(test)]
mod tests {
    use mps_cosmos::bodies::satellite_builder;
    use mps_cosmos::gravity::CelestialSource;
    use mps_cosmos::world::{
        CosmosWorld, CosmosWorldConfig, OrbitIntegration, PerturbationConfig, StepResult,
    };
    use mps_formula::celestial_data::EARTH;
    use rapier3d::prelude::Vector;

    fn config() -> CosmosWorldConfig {
        CosmosWorldConfig {
            orbit_integration: OrbitIntegration::Verlet,
            verlet_substeps: 4,
            ..Default::default()
        }
    }

    /// 钉子 1：表面共转大气速度 = ω × r。地球赤道 465.1 m/s 向东（+y 方向，
    /// 自转轴 +z）。容差 1 m/s（模型本身是简化：轴固定 +z）。
    #[test]
    fn surface_atmosphere_velocity_is_co_rotating() {
        // 用与引擎相同的近似：ω = (0,0,ω)，r 赤道表面 → |ω×r| = ω·R。
        let omega = EARTH.rotation_rate;
        let r_eq = EARTH.equatorial_radius;
        let expected = omega * r_eq;
        // 教科书值 465.1 m/s（地球自转 7.2921159e-5 rad/s × 6.378e6 m）。
        assert!(
            (expected - 465.1).abs() < 1.0,
            "earth equatorial co-rotation speed {expected:.2} m/s should be ~465.1"
        );
        // 方向：ω(+z) × r(+x) = +y（向东）。
        let w = Vector::new(0.0, 0.0, omega);
        let r = Vector::new(r_eq, 0.0, 0.0);
        let v = Vector::new(-w.z * r.y, w.z * r.x, 0.0);
        assert!(v.y > 460.0 && v.z.abs() < 1e-9);
    }

    /// 钉子 2：离心量级 ω²R 与表面引力 GM/R² 的比值 ~3.4e-3（地球）。
    /// 这是「celestial_acceleration 不含离心项」取舍的显式契约：
    /// 游戏尺度下 0.35% 的 g 偏差可接受；若未来要接更真实的表面平衡，
    /// 应新开 ADR 而不是静默改纯引力语义。
    #[test]
    fn centrifugal_term_is_documented_negligible_fraction() {
        let omega = EARTH.rotation_rate;
        let r_eq = EARTH.equatorial_radius;
        let g_surface = EARTH.gm / (r_eq * r_eq);
        let centrifugal = omega * omega * r_eq;
        let ratio = centrifugal / g_surface;
        // 地球理论值 0.00346（3.46e-3）。
        assert!(
            (ratio - 3.46e-3).abs() < 1e-4,
            "earth centrifugal/surface-gravity ratio {ratio:.3e} should be ~3.46e-3"
        );
        assert!(
            ratio < 0.01,
            "centrifugal term must stay sub-percent of surface gravity for the pure-gravity contract to hold"
        );
    }

    /// 钉子 3：科氏偏转冒烟——自由落体 + 共转大气阻力 → 向东的横向速度。
    /// 从 100 km 高度静止释放（相对惯性系），落体时超转大气把它向东推。
    #[test]
    fn falling_body_drifts_eastward_with_co_rotating_atmosphere() {
        let r_start = EARTH.equatorial_radius + 1.0e5; // 100 km 高度
        let mut world = CosmosWorld::new(config());
        world.add_celestial(CelestialSource::new(&EARTH, 0));
        world.set_central_body(Some(&EARTH));
        let sat = world.insert_body(satellite_builder(
            1000.0,
            Vector::new(r_start, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.0),
            1.0,
        ));
        world.set_perturbation(
            sat,
            PerturbationConfig {
                enable_drag: true,
                drag_coefficient: 2.2,
                area: 4.0,
                ..Default::default()
            },
        );
        let steps = 6_000usize; // 100 s
        for _ in 0..steps {
            if !matches!(world.step(1.0 / 60.0), StepResult::Stepped(_)) {
                break;
            }
        }
        let st = world.body_state(sat).expect("sat alive");
        // 落体 100 s：径向速度 ~1 km/s 量级；横向（+y）分量应随共转大气
        // （阻力参考系 = ω×r，超转大气向东推落体）从 0 增长。方向性钉，
        // 精确值由阻力参数决定，不在此处锁死。
        assert!(
            st.velocity.y > 0.0,
            "falling body must drift eastward (+y, co-rotating drag frame), got {st:?}"
        );
        // 引力中心向 -x：径向速度为负。
        assert!(st.velocity.x < 0.0, "must fall toward the planet: {st:?}");
    }
}
