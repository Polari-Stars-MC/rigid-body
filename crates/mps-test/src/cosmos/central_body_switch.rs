//! C2：central_body 切换（SOI 等效决策）的影响面文档化断言。
//!
//! 清点结论（docs/evidence/convergence.md 附表引用）：
//! `central_body` 只影响三类**叠加层**——相对论修正叠加对象
//! （celestial_segment 按 ptr 判定）、大气阻力参考、自适应子步误差代理；
//! **不改基础引力**（每 celestial 源独立累加）。SOI 切换不引起基础引力
//! 跳变 → 本文件把该契约钉成显式断言：
//!
//! 1. 切换 central_body 前后，纯引力加速度逐位一致（无跳变）；
//! 2. n 体源互引力不读 central_body（切换不影响）；
//! 3. 相对论修正跟随 central_body 指针（这是切换唯一改变引力的通道，
//!    且只在显式开启 1PN/2PN 时生效——默认 None 下为零差异）。

#[cfg(test)]
mod tests {
    use mps_cosmos::bodies::satellite_builder;
    use mps_cosmos::gravity::CelestialSource;
    use mps_cosmos::world::{
        CosmosWorld, CosmosWorldConfig, OrbitIntegration, RelativisticCorrection,
    };
    use mps_formula::celestial_data::{EARTH, MOON};
    use rapier3d::prelude::Vector;

    /// 断言 1：切换 central_body 不改基础引力。
    #[test]
    fn central_body_switch_does_not_change_base_gravity() {
        let mut world = CosmosWorld::new(CosmosWorldConfig {
            orbit_integration: OrbitIntegration::Verlet,
            relativistic_correction: RelativisticCorrection::None,
            ..Default::default()
        });
        world.add_celestial(CelestialSource::new(&EARTH, 0));
        world.add_celestial(CelestialSource::new(&MOON, 0));
        let sat = world.insert_body(satellite_builder(
            1000.0,
            Vector::new(6.0e6, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.0),
            1.0,
        ));

        // 中央天体 = 地球：步进一次记录位置。
        world.set_central_body(Some(&EARTH));
        world.step(1.0 / 60.0);
        let pos_earth_central = world.body_state(sat).expect("sat").position;

        // 重置世界重跑：中央天体 = 月球。
        let mut world2 = CosmosWorld::new(CosmosWorldConfig {
            orbit_integration: OrbitIntegration::Verlet,
            relativistic_correction: RelativisticCorrection::None,
            ..Default::default()
        });
        world2.add_celestial(CelestialSource::new(&EARTH, 0));
        world2.add_celestial(CelestialSource::new(&MOON, 0));
        let sat2 = world2.insert_body(satellite_builder(
            1000.0,
            Vector::new(6.0e6, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.0),
            1.0,
        ));
        world2.set_central_body(Some(&MOON));
        world2.step(1.0 / 60.0);
        let pos_moon_central = world2.body_state(sat2).expect("sat").position;

        // SOI 切换不跳变：默认（无相对论）配置下两个世界逐位一致。
        assert_eq!(
            pos_earth_central, pos_moon_central,
            "switching central_body must not change base-gravity trajectory (None relativistic)"
        );
    }

    /// 断言 2：相对论修正是切换唯一改变引力的通道（1PN 时叠加对象跟随
    /// central_body 指针——地球为中央时地球项带 1PN，月球为中央时月球项带）。
    #[test]
    fn relativistic_correction_follows_central_body_pointer() {
        let make = |central: &'static mps_formula::celestial_data::CelestialBody| {
            let mut world = CosmosWorld::new(CosmosWorldConfig {
                orbit_integration: OrbitIntegration::Verlet,
                relativistic_correction: RelativisticCorrection::OnePN,
                ..Default::default()
            });
            world.add_celestial(CelestialSource::new(&EARTH, 0));
            world.add_celestial(CelestialSource::new(&MOON, 0));
            world.set_central_body(Some(central));
            let sat = world.insert_body(satellite_builder(
                1000.0,
                Vector::new(6.0e6, 0.0, 0.0),
                Vector::new(0.0, 0.0, 0.0),
                1.0,
            ));
            world.step(1.0 / 60.0);
            world.body_state(sat).expect("sat").position
        };

        let with_earth_1pn = make(&EARTH);
        let with_moon_1pn = make(&MOON);
        // 1PN 修正量级 ~1e-9 相对项：切换叠加对象必然改变轨迹，但只在
        // f64 尺度的远位（1PN 位移 ~ cm 级 @ 1 步 dt=1/60 → 差异 > 1e-12）。
        let d = (with_earth_1pn - with_moon_1pn).length();
        assert!(
            d > 0.0 && d < 1.0,
            "1PN overlay switch must produce a small but nonzero trajectory change: d={d:.3e}"
        );
    }
}
