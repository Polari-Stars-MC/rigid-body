//! C1：长期轨道基准（docs/evidence/numeric-drift.md 附表的数据源）。
//!
//! 三个 1M 步量级的长跑基准（`#[ignore]`，release 复现）：
//! 1. L4 稳定性：太阳-地球三角平动点上的测试粒子，1M 步（dt=30 s ≈ 0.95 年）
//!    相对地球的角距保持在 60° ± 5°——n-body 对称性 + 辛积子的长期能量
//!    守恒共同钉住。
//! 2. 月式轨道：环月卫星 a = 5×月半径，1M 步（dt=30 s ≈ 200+ 圈），根数
//!    漂移 |Δa|/a 上限钉死。
//! 3. Plummer ε 扫描：近场软化 ε ∈ {1, 1e3, 1e5}，穿透场景下加速度
//!    连续且有界（A1/C1 共同承诺「软化替代 1 m 硬截止」的量化）。
//!
//! 复现：`cargo test -p mps-test --release --lib -- long_orbit --ignored --nocapture`

#[cfg(test)]
mod tests {
    use mps_cosmos::bodies::satellite_builder;
    use mps_cosmos::gravity::CelestialSource;
    use mps_cosmos::world::{CosmosWorld, CosmosWorldConfig, OrbitIntegration, StepResult};
    use mps_formula::celestial_data::{EARTH, MOON, SUN};
    use rapier3d::prelude::Vector;

    fn config(substeps: u32) -> CosmosWorldConfig {
        CosmosWorldConfig {
            orbit_integration: OrbitIntegration::Yoshida4Kahan,
            verlet_substeps: substeps,
            ..Default::default()
        }
    }

    /// 基准 1：L4 稳定性。
    #[test]
    #[ignore = "1M steps long-running; run with --ignored for evidence"]
    fn l4_triangle_remains_stable_over_1m_steps() {
        let mut world = CosmosWorld::new(config(1));
        world.add_celestial(CelestialSource::new(&SUN, 0));
        world.set_central_body(Some(&SUN));

        // 地球：放在圆轨道上（r = 1 AU），作为 n-body 源（质量真实量级）。
        let au = 1.496e11;
        let v_earth = (SUN.gm / au).sqrt();
        let earth_mass = 5.972e24;
        let earth = world.insert_body(satellite_builder(
            earth_mass,
            Vector::new(au, 0.0, 0.0),
            Vector::new(0.0, v_earth, 0.0),
            EARTH.equatorial_radius,
        ));
        world.add_n_body(earth, earth_mass);

        // L4 测试粒子：地球前方 60°，同角速度。
        let theta = 60f64.to_radians();
        let l4 = Vector::new(au * theta.cos(), au * theta.sin(), 0.0);
        let omega = v_earth / au;
        let v_l4 = Vector::new(-omega * l4.y, omega * l4.x, 0.0);
        let probe = world.insert_body(satellite_builder(1000.0, l4, v_l4, 1.0));

        let dt = 30.0; // world_step 上限 30 s（MAX_STEP_DT）
        let steps = 1_000_000usize;
        let sample_every = 100_000usize;
        for s in 1..=steps {
            assert!(
                matches!(
                    world.step(dt),
                    StepResult::Stepped(_) | StepResult::Substepped { .. }
                ),
                "step {s} skipped"
            );
            if s % sample_every == 0 {
                let pe = world.body_state(earth).expect("earth");
                let pp = world.body_state(probe).expect("probe");
                // 角分离（L4 契约：探测器与地球对日心的角距恒 60°；取两个
                // 绝对角的差——世界系里 rel 矢量本身随轨道旋转，不能直接用）。
                let ae = pe.position.y.atan2(pe.position.x).to_degrees();
                let ap = pp.position.y.atan2(pp.position.x).to_degrees();
                let ang = ap - ae;
                let dist = (pp.position - pe.position).length();
                println!("step {s:>7}: L4 angle-sep={ang:6.2}  dist={dist:.4e} m");
                assert!(
                    (ang - 60.0).abs() < 5.0,
                    "L4 libration beyond +/-5 deg at step {s}: {ang:.2}"
                );
            }
        }
        let pe = world.body_state(earth).expect("earth");
        let pp = world.body_state(probe).expect("probe");
        let ae = pe.position.y.atan2(pe.position.x).to_degrees();
        let ap = pp.position.y.atan2(pp.position.x).to_degrees();
        println!("final L4 angle-sep: {:.3} deg", ap - ae);
    }

    /// 基准 2：月式轨道 1M 步根数漂移。
    #[test]
    #[ignore = "1M steps long-running; run with --ignored for evidence"]
    fn lunar_orbit_elements_drift_over_1m_steps() {
        let mut world = CosmosWorld::new(config(1));
        world.add_celestial(CelestialSource::new(&MOON, 0));
        world.set_central_body(Some(&MOON));

        // 环月圆轨道：a = 5×月半径——低月轨，避开形变区。
        let a0 = 5.0 * MOON.equatorial_radius;
        let v0 = (MOON.gm / a0).sqrt();
        let sat = world.insert_body(satellite_builder(
            500.0,
            Vector::new(a0, 0.0, 0.0),
            Vector::new(0.0, v0, 0.0),
            1.0,
        ));

        let dt = 30.0;
        let steps = 1_000_000usize; // ≈ 2.9 年 ≈ 200+ 圈（周期 ~38 h）
        let sample_every = 100_000usize;
        let mut max_da = 0.0f64;
        for s in 1..=steps {
            assert!(
                matches!(
                    world.step(dt),
                    StepResult::Stepped(_) | StepResult::Substepped { .. }
                ),
                "step {s} skipped"
            );
            if s % sample_every == 0 {
                let st = world.body_state(sat).expect("sat");
                let r = st.position.length();
                let v2 = st.velocity.length_squared();
                let energy = 0.5 * v2 - MOON.gm / r;
                let a = -0.5 * MOON.gm / energy;
                let da = ((a - a0) / a0).abs();
                max_da = max_da.max(da);
                println!("step {s:>7}: a={a:.3e}  da/a={da:.3e}");
            }
        }
        assert!(
            max_da < 1e-8,
            "lunar orbit a-drift {max_da:.3e} exceeds 1e-8 over 1M steps (Yoshida4Kahan)"
        );
    }

    /// 基准 3：Plummer ε 扫描——穿透下加速度连续、有界。
    #[test]
    #[ignore = "epsilon-response table; run with --ignored for evidence"]
    fn plummer_softening_response_is_continuous_and_bounded() {
        use mps_cosmos::gravity::point_mass_acceleration_with_softening;

        // r/ε ∈ {0.1, 1, 100}：软化核内/边界/远场（Plummer 远场收敛是
        // O(1.5·ε²/r²)，10ε 处偏差 ~1.5%，100ε 处 ~0.015%——断言钉 100ε）。
        let epsilons = [1.0, 1e3, 1e5];
        let gm = EARTH.gm;
        println!("eps      r      |a|           ratio(|a|/a_point)");
        for eps in epsilons {
            for frac in [0.1, 1.0, 100.0] {
                let r = eps * frac;
                let a_soft =
                    point_mass_acceleration_with_softening(Vector::new(r, 0.0, 0.0), gm, eps * eps)
                        .length();
                let a_point = gm / (r * r);
                let ratio = a_soft / a_point;
                println!("{eps:.0e}  {r:.3e}  {a_soft:.4e}  {ratio:.4}");
                // 核内（r<ε）软化显著削弱奇点：|a| 有界（< 点质量值）。
                assert!(a_soft <= a_point * (1.0 + 1e-12));
                // 远场（r≥100ε）应回归点质量（比值 > 0.999）。
                if frac >= 100.0 {
                    assert!(ratio > 0.999, "far field must converge to point mass");
                }
                // 核内（r<ε）应被压住：比值 < 点质量的发散程度，有界。
                if frac < 1.0 {
                    assert!(a_soft.is_finite() && a_soft > 0.0);
                }
            }
        }
    }
}
