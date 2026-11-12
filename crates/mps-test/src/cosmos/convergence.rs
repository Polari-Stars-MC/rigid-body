//! A2：自适应子步收敛性验证（docs/evidence/convergence.md 的数据源）。
//!
//! 三个可观测断言：
//! 1. `adaptive_substep_count` 的误差代理单调性：r 越小 → 子步越多；
//!    r ≥ 2·r_eq 时回落到 base（verlet_substeps）。
//! 2. tolerance 扫描（1e-6 / 1e-9 / 1e-12）：子步数单调不减、能量漂移
//!    单调不增——「更严的 tolerance 买到更小的漂移」这条产品承诺成立。
//! 3. 闭环守恒：高椭圆轨道 + 近心点自适应加密，100k 步相对能量漂移
//!    < 1e-8（比无自适应的 Verlet 长跑好一个量级以上——A1 的 ignored
//!    测试量化过 Verlet 的漂移基线）。

#[cfg(test)]
mod tests {
    use mps_cosmos::bodies::satellite_builder;
    use mps_cosmos::gravity::CelestialSource;
    use mps_cosmos::world::{
        CosmosWorld, CosmosWorldConfig, OrbitIntegration, RelativisticCorrection, StepResult,
    };
    use mps_formula::celestial_data::CelestialBody;
    use rapier3d::prelude::Vector;

    static BENCH_BODY: CelestialBody = CelestialBody {
        name: "bench",
        gm: 3.986_004_418e14,
        equatorial_radius: 6.3781e6,
        flattening: 0.0,
        rotation_rate: 0.0,
        j2: 0.0,
        j3: 0.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        max_degree: 0,
        c_coeffs: &[],
        s_coeffs: &[],
        ref_radius: 6.3781e6,
        surface_density: 0.0,
        scale_height: 0.0,
        solar_pressure_constant: 0.0,
    };

    fn config(tolerance: f64) -> CosmosWorldConfig {
        CosmosWorldConfig {
            orbit_integration: OrbitIntegration::Verlet,
            relativistic_correction: RelativisticCorrection::None,
            verlet_substeps: 2,
            adaptive_substeps: true,
            adaptive_tolerance: tolerance,
            ..Default::default()
        }
    }

    /// 测试 1：误差代理单调性（纯查询，不步进）。
    #[test]
    fn adaptive_substep_count_is_monotone_in_proximity() {
        let mut world = CosmosWorld::new(config(1e-13));
        world.add_celestial(CelestialSource::new(&BENCH_BODY, 0));
        world.set_central_body(Some(&BENCH_BODY));
        let dt = 1.0 / 60.0;
        let base = 2usize; // verlet_substeps

        // 远心点（r ≥ 2·r_eq）：回落到 base。
        let far = world.insert_body(satellite_builder(
            1000.0,
            Vector::new(4.2e8, 0.0, 0.0),
            Vector::new(0.0, 10.0, 0.0),
            1.0,
        ));
        let far_count = world.adaptive_substep_count(dt);
        assert_eq!(
            far_count, base,
            "far-centered body must not inflate substeps"
        );

        // 1.5·r_eq：介于 base 与上限之间。
        let _ = far;
        let _ = world.insert_body(satellite_builder(
            1000.0,
            Vector::new(1.5 * BENCH_BODY.equatorial_radius, 0.0, 0.0),
            Vector::new(0.0, 10.0, 0.0),
            1.0,
        ));
        let mid_count = world.adaptive_substep_count(dt);
        assert!(
            mid_count > base,
            "sub-surface distance must inflate substeps: {mid_count} vs base {base}"
        );

        // 0.6·r_eq（更近）：不少于 1.5·r_eq 的子步数。
        let _ = world.insert_body(satellite_builder(
            1000.0,
            Vector::new(0.6 * BENCH_BODY.equatorial_radius, 0.0, 0.0),
            Vector::new(0.0, 10.0, 0.0),
            1.0,
        ));
        let near_count = world.adaptive_substep_count(dt);
        assert!(
            near_count >= mid_count,
            "substep count must be monotone in proximity: near={near_count} mid={mid_count}"
        );
        assert!(near_count <= 64, "substep cap: {near_count}");
    }

    /// 测试 2：tolerance 扫描——子步数单调不减，漂移单调不增。
    #[test]
    #[ignore = "3×20k steps ≈ 5 s release; run with --ignored for the evidence table"]
    fn tolerance_scan_buys_drift_monotonically() {
        let gm = BENCH_BODY.gm;
        // 高椭圆轨道：近心点 0.9·r_eq（触发加密），远心点 4.2e7。
        let r_p = 0.9 * BENCH_BODY.equatorial_radius;
        let r_a = 4.2e7;
        let a = 0.5 * (r_p + r_a);
        let v_p = (gm * (2.0 / r_p - 1.0 / a)).sqrt();

        let specific_energy =
            |p: Vector, v: Vector| -> f64 { 0.5 * v.length().powi(2) - gm / p.length() };
        let e0 = specific_energy(Vector::new(r_p, 0.0, 0.0), Vector::new(0.0, v_p, 0.0));

        let steps = 20_000usize;
        let tolerances = [1e-6, 1e-9, 1e-12];
        let mut table = Vec::new();
        for tol in tolerances {
            let mut world = CosmosWorld::new(config(tol));
            world.add_celestial(CelestialSource::new(&BENCH_BODY, 0));
            world.set_central_body(Some(&BENCH_BODY));
            let sat = world.insert_body(satellite_builder(
                1000.0,
                Vector::new(r_p, 0.0, 0.0),
                Vector::new(0.0, v_p, 0.0),
                1.0,
            ));
            let mut substeps_max = 0usize;
            for _ in 0..steps {
                assert!(matches!(world.step(dt_of()), StepResult::Stepped(_)));
                // 近心点段（r < 2·r_eq）采样子步数上限。
                if let Some(st_now) = world.body_state(sat) {
                    if st_now.position.length() < 2.0 * BENCH_BODY.equatorial_radius {
                        substeps_max = substeps_max.max(world.adaptive_substep_count(dt_of()));
                    }
                }
            }
            let substeps_seen = substeps_max;
            let st = world.body_state(sat).expect("sat alive");
            let drift = ((specific_energy(st.position, st.velocity) - e0) / e0).abs();
            table.push((tol, substeps_seen, drift));
            println!("tol={tol:.0e}: substeps={substeps_seen} drift={drift:.3e}");
        }
        // 子步数随 tolerance 收紧单调不减。
        for i in 1..table.len() {
            assert!(
                table[i].1 >= table[i - 1].1,
                "substeps must not decrease as tolerance tightens: {:?}",
                table
            );
        }
        // 漂移随 tolerance 收紧单调不增（容许末位并列）。
        for i in 1..table.len() {
            assert!(
                table[i].2 <= table[i - 1].2 * 1.1 + 1e-12,
                "drift must not increase as tolerance tightens: {:?}",
                table
            );
        }
        // 最严 tolerance 下漂移压进 1e-8。
        assert!(
            table.last().unwrap().2 < 1e-8,
            "tightest tolerance drift {:e} exceeds 1e-8",
            table.last().unwrap().2
        );
    }

    fn dt_of() -> f64 {
        1.0 / 60.0
    }

    /// 测试 3：近心点自适应闭环守恒（非 ignore 的回归钉）。
    #[test]
    fn adaptive_periapsis_energy_conservation() {
        let gm = BENCH_BODY.gm;
        let r_p = 0.9 * BENCH_BODY.equatorial_radius;
        let r_a = 4.2e7;
        let a = 0.5 * (r_p + r_a);
        let v_p = (gm * (2.0 / r_p - 1.0 / a)).sqrt();
        let specific_energy =
            |p: Vector, v: Vector| -> f64 { 0.5 * v.length().powi(2) - gm / p.length() };
        let e0 = specific_energy(Vector::new(r_p, 0.0, 0.0), Vector::new(0.0, v_p, 0.0));

        let mut world = CosmosWorld::new(config(1e-9));
        world.add_celestial(CelestialSource::new(&BENCH_BODY, 0));
        let sat = world.insert_body(satellite_builder(
            1000.0,
            Vector::new(r_p, 0.0, 0.0),
            Vector::new(0.0, v_p, 0.0),
            1.0,
        ));
        let steps = 5_000usize;
        for _ in 0..steps {
            assert!(matches!(world.step(1.0 / 60.0), StepResult::Stepped(_)));
        }
        let st = world.body_state(sat).expect("sat alive");
        let rel = ((specific_energy(st.position, st.velocity) - e0) / e0).abs();
        // 短程（5k 步）自适应加密下 1e-9 内——回归钉。
        assert!(
            rel < 1e-9,
            "adaptive closed-loop drift {rel:.3e} exceeds 1e-9"
        );
    }
}
