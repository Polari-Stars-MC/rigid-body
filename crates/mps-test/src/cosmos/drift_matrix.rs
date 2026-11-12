//! A1：f64 长时积分漂移回归矩阵（docs/evidence/numeric-drift.md 的数据源）。
//!
//! 「f64 + Kahan 值得存在」的量化证据：
//! - 单摆能量守恒（Verlet vs Yoshida4Kahan）
//! - 圆轨道半长轴/偏心率漂移（kepler elements 提取）
//! - 每个测试打印 drift-vs-time 数据点供 evidence 表格抄录。
//!
//! 长跑项标 `#[ignore]`，evidence 复现命令：
//! `cargo test -p mps-test --lib -- drift_matrix --ignored --nocapture`

#[cfg(test)]
mod tests {
    use mps_cosmos::bodies::satellite_builder;
    use mps_cosmos::gravity::{CelestialSource, point_mass_acceleration};
    use mps_cosmos::orbit::elements_of;
    use mps_cosmos::world::{
        CosmosWorld, CosmosWorldConfig, OrbitIntegration, RelativisticCorrection, StepResult,
    };
    use mps_formula::celestial_data::CelestialBody;
    use rapier3d::prelude::Vector;

    /// 一个绝热线性的中心天体（球对称、无 J2、无大气）——漂移矩阵的基准源。
    static BENCH_BODY: CelestialBody = CelestialBody {
        name: "bench",
        gm: 3.986_004_418e14, // 地球 GM
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

    fn config(mode: OrbitIntegration) -> CosmosWorldConfig {
        CosmosWorldConfig {
            gravity: Vector::ZERO,
            dt: 1.0 / 60.0,
            solver_iterations: 4,
            ccd_substeps: 0,
            n_body_softening_sq: 0.0,
            central_body: None,
            orbit_integration: mode,
            verlet_substeps: 1,
            adaptive_substeps: false,
            adaptive_tolerance: 1e-9,
            relativistic_correction: RelativisticCorrection::None,
        }
    }

    /// 圆轨道速度（解析）：v = sqrt(GM / r)。
    fn circular_speed(gm: f64, r: f64) -> f64 {
        (gm / r).sqrt()
    }

    /// 摆体能量：E = ½v² − GM/r（单位质量）。
    fn specific_energy(pos: Vector, vel: Vector, gm: f64) -> f64 {
        0.5 * vel.length_squared() - gm / pos.length()
    }

    /// 测试 1：单摆（高椭圆轨道 e≈0.97）能量漂移——Verlet vs Yoshida4Kahan。
    ///
    /// 初始近心点 r_p = 6.6e6 m（地表上方 ~200 km），远心点 r_a = 4.2e8 m
    /// （约月球距离）。椭圆轨道用 vis-viva 定近心点速度。
    #[test]
    #[ignore = "long-running (2×100k steps); run with --ignored for evidence table"]
    fn drift_pendulum_energy_kahan_vs_verlet() {
        let gm = BENCH_BODY.gm;
        let r_p = 6.6e6;
        let r_a = 4.2e8;
        let a = 0.5 * (r_p + r_a);
        // vis-viva: v² = GM(2/r − 1/a)，在近心点切向出发。
        let v_p = (gm * (2.0 / r_p - 1.0 / a)).sqrt();
        let e0 = specific_energy(Vector::new(r_p, 0.0, 0.0), Vector::new(0.0, v_p, 0.0), gm);

        let steps = 100_000usize;
        let sample_every = 10_000usize;

        let run = |mode: OrbitIntegration| -> Vec<(usize, f64)> {
            let mut world = CosmosWorld::new(config(mode));
            world.add_celestial(CelestialSource::new(&BENCH_BODY, 0));
            let sat = world.insert_body(satellite_builder(
                1000.0,
                Vector::new(r_p, 0.0, 0.0),
                Vector::new(0.0, v_p, 0.0),
                1.0,
            ));
            let mut samples = Vec::new();
            for s in 1..=steps {
                assert!(
                    matches!(world.step(1.0 / 60.0), StepResult::Stepped(_)),
                    "step {s} skipped"
                );
                if s % sample_every == 0 {
                    let st = world.body_state(sat).expect("sat alive");
                    samples.push((s, specific_energy(st.position, st.velocity, gm)));
                }
            }
            samples
        };

        let verlet = run(OrbitIntegration::Verlet);
        let kahan = run(OrbitIntegration::Yoshida4Kahan);

        // 相对漂移 |E(t) − E(0)| / |E(0)|。
        let rel = |samples: &[(usize, f64)]| -> Vec<(usize, f64)> {
            samples
                .iter()
                .map(|&(s, e)| (s, ((e - e0) / e0).abs()))
                .collect()
        };
        let rv = rel(&verlet);
        let rk = rel(&kahan);

        println!("pendulum relative energy drift (|E(t)-E0|/|E0|):");
        println!("steps      verlet        yoshida4+kahan");
        for i in 0..rv.len() {
            println!("{:>8}  {:.3e}  {:.3e}", rv[i].0, rv[i].1, rk[i].1);
        }

        // Kahan 版必须显著优于 Verlet：最终漂移至少低一个量级。
        let last_v = rv.last().expect("samples").1;
        let last_k = rk.last().expect("samples").1;
        assert!(
            last_k < last_v * 0.1,
            "yoshida4+kahan drift ({last_k:.3e}) should beat verlet ({last_v:.3e}) by 10x+"
        );
        // 辛积子守恒：Kahan 版 100k 步相对漂移 < 1e-9。
        assert!(last_k < 1e-9, "kahan drift {last_k:.3e} exceeds 1e-9");
    }

    /// 测试 2：圆轨道半长轴/偏心率漂移（Yoshida4Kahan，kepler elements 提取）。
    #[test]
    #[ignore = "long-running (200k steps); run with --ignored for evidence table"]
    fn drift_circular_orbit_elements_yoshida4_kahan() {
        let gm = BENCH_BODY.gm;
        let r0 = 8.0e6; // 地表上方 ~1.6e6 m，软化区（ε=1 m）之外
        let v0 = circular_speed(gm, r0);
        let mu = gm;

        let mut world = CosmosWorld::new(config(OrbitIntegration::Yoshida4Kahan));
        world.add_celestial(CelestialSource::new(&BENCH_BODY, 0));
        let sat = world.insert_body(satellite_builder(
            1000.0,
            Vector::new(r0, 0.0, 0.0),
            Vector::new(0.0, v0, 0.0),
            1.0,
        ));

        let steps = 200_000usize;
        let sample_every = 20_000usize;
        println!("circular orbit r0={r0:.0} v0={v0:.3}:");
        println!("steps      a (m)            da/a         e");
        let mut max_da = 0.0f64;
        let mut max_e = 0.0f64;
        for s in 1..=steps {
            assert!(
                matches!(world.step(1.0 / 60.0), StepResult::Stepped(_)),
                "step {s} skipped"
            );
            if s % sample_every == 0 {
                let st = world.body_state(sat).expect("sat alive");
                let Some(el) = elements_of(st, mu) else {
                    panic!("elements extraction failed at step {s}");
                };
                let da = ((el.semi_major_axis - r0) / r0).abs();
                let e = el.eccentricity;
                max_da = max_da.max(da);
                max_e = max_e.max(e);
                println!(
                    "{:>8}  {:>14.3}  {:.3e}  {:.3e}",
                    s, el.semi_major_axis, da, e
                );
            }
        }
        assert!(
            max_da < 1e-9,
            "semi-major-axis drift {max_da:.3e} exceeds 1e-9"
        );
        assert!(
            max_e < 1e-9,
            "eccentricity excitation {max_e:.3e} exceeds 1e-9"
        );
    }

    /// 测试 3（轻量，非 ignore）：短程守恒冒烟——确保上面两个 ignored 测试的
    /// 基础设施（energy 提取、elements 提取）没坏。1000 步 Verlet 圆轨道。
    #[test]
    fn drift_matrix_infrastructure_smoke() {
        let gm = BENCH_BODY.gm;
        let r0 = 8.0e6;
        let v0 = circular_speed(gm, r0);
        let mut world = CosmosWorld::new(config(OrbitIntegration::Verlet));
        world.add_celestial(CelestialSource::new(&BENCH_BODY, 0));
        let sat = world.insert_body(satellite_builder(
            1000.0,
            Vector::new(r0, 0.0, 0.0),
            Vector::new(0.0, v0, 0.0),
            1.0,
        ));
        let e0 = specific_energy(Vector::new(r0, 0.0, 0.0), Vector::new(0.0, v0, 0.0), gm);
        for s in 1..=1000 {
            assert!(
                matches!(world.step(1.0 / 60.0), StepResult::Stepped(_)),
                "step {s}"
            );
        }
        let st = world.body_state(sat).expect("sat alive");
        let e1 = specific_energy(st.position, st.velocity, gm);
        let rel = ((e1 - e0) / e0).abs();
        // 1000 步 Verlet 短程：1e-8 内（100k 步的长程断言在 ignored 测试里）。
        assert!(rel < 1e-8, "short-run drift {rel:.3e} exceeds 1e-8");
        // 点质量回归：解析对照一次性验证。
        let a = point_mass_acceleration(Vector::new(2.0, 0.0, 0.0), gm);
        let expect = gm / 4.0; // GM/r², r=2
        assert!((a.x + expect).abs() < 1e-6 * expect);
    }
}
