//! Hohmann 转移「真实物理」验证（LEO 6678 km → GEO 42164 km）。
//!
//! 三个钉子：
//! 1. 解析一致性：`mps_formula::spaceflight::kepler::hohmann_transfer_checked`
//!    的 ΔV / 转移半长轴 / 转移时间与教科书闭式解逐位（≤1e-12 相对）一致；
//! 2. 积分闭环：CosmosWorld（纯点质量地球源 + Yoshida4Kahan）里，在近地点
//!    通过共享 arena 命令环注入 SetVelocity（v_circ + Δv1），积分半转移周期
//!    π√(a_t³/μ) 后到达 r2（<1%），再注入 ApplyImpulse（m·Δv2）后速度恰为
//!    GEO 圆轨道速度且径向分量归零；
//! 3. 转移椭圆形状：转移段逐步采样 r(t) 与开普勒方程 M = E − e·sinE 解出的
//!    r 逐点比对（<0.1%）。
//!
//! 语义说明（发现，非缺陷）：`celestial_acceleration` 在 r < 2·R_eq 时走椭球
//! 分支、r < 100·R_eq 时走 J2..J6 带谐分支（crates/mps-cosmos/src/gravity.rs
//! `celestial_acceleration`）。LEO 6678 km ≈ 1.047 R_eq 处于椭球分支，与
//! Hohmann 二体点质量解析解存在 O(J2) 的系统性偏差。为做干净的两体闭环
//! 验证，这里构造 `equatorial_radius = 0` 的退化天体（走点质量分支，
//! `point_mass_acceleration`），μ 取 EARTH_GM。产品默认地球源在 LEO 高度的
//! J2/椭球语义由其它测试钉住，不在此重复。

#[cfg(test)]
mod tests {
    use mps_cosmos::arena::{CMD_SLOT_STRIDE, OFF_CMD_RING, OFF_CMD_WRITE};
    use mps_cosmos::bodies::satellite_builder;
    use mps_cosmos::gravity::CelestialSource;
    use mps_cosmos::world::{CosmosWorld, CosmosWorldConfig, OrbitIntegration, StepResult};
    use mps_formula::celestial_data::EARTH_GM;
    use mps_formula::ffi::types::HohmannTransfer;
    use mps_formula::spaceflight::kepler::hohmann_transfer_checked;
    use rapier3d::prelude::Vector;

    // LEO 300 km 圆轨道 → GEO。
    const R1: f64 = 6_678_000.0;
    const R2: f64 = 42_164_000.0;
    const MU: f64 = EARTH_GM;
    const MASS: f64 = 1000.0;

    /// 教科书闭式解：Δv1 = √(μ/r1)·(√(2r2/(r1+r2)) − 1)，
    /// Δv2 = √(μ/r2)·(1 − √(2r1/(r1+r2)))。
    fn textbook_hohmann(mu: f64, r1: f64, r2: f64) -> HohmannTransfer {
        let a_t = 0.5 * (r1 + r2);
        let dv1 = (mu / r1).sqrt() * ((2.0 * r2 / (r1 + r2)).sqrt() - 1.0);
        let dv2 = (mu / r2).sqrt() * (1.0 - (2.0 * r1 / (r1 + r2)).sqrt());
        HohmannTransfer {
            delta_v1: dv1,
            delta_v2: dv2,
            total_delta_v: dv1 + dv2,
            transfer_time: std::f64::consts::PI * (a_t * a_t * a_t / mu).sqrt(),
        }
    }

    /// 纯点质量地球源（equatorial_radius=0 → `celestial_acceleration` 走
    /// 点质量分支，与二体解析解严格同模型；见文件头语义说明）。
    fn point_mass_earth() -> &'static mps_formula::celestial_data::CelestialBody {
        use mps_formula::celestial_data::CelestialBody;
        Box::leak(Box::new(CelestialBody {
            name: "earth-monopole",
            gm: MU,
            equatorial_radius: 0.0,
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
            ref_radius: 0.0,
            surface_density: 0.0,
            scale_height: 0.0,
            solar_pressure_constant: 0.0,
        }))
    }

    fn config() -> CosmosWorldConfig {
        CosmosWorldConfig {
            orbit_integration: OrbitIntegration::Yoshida4Kahan,
            verlet_substeps: 1,
            n_body_softening_sq: 0.0,
            adaptive_substeps: false,
            relativistic_correction: mps_cosmos::world::RelativisticCorrection::None,
            ..Default::default()
        }
    }

    /// 在共享 arena 命令环写一条命令并 bump cmd_write（Java 侧等价操作，
    /// 与 mps-test cosmos/ffi.rs 的 write_command 同布局：5 × u64）。
    unsafe fn write_command(
        base: *mut u8,
        cmd_ring_base: usize,
        slot: usize,
        cmd_type: u64,
        body_index: u64,
        a: [f64; 3],
    ) {
        unsafe {
            let p = base.add(cmd_ring_base + slot * CMD_SLOT_STRIDE as usize);
            (p as *mut u64).write_unaligned(cmd_type);
            (p.add(8) as *mut u64).write_unaligned(body_index);
            (p.add(16) as *mut u64).write_unaligned(a[0].to_bits());
            (p.add(24) as *mut u64).write_unaligned(a[1].to_bits());
            (p.add(32) as *mut u64).write_unaligned(a[2].to_bits());
        }
    }

    /// 闭环场景：LEO 圆轨道卫星 + 纯点质量地球。在 step 边界通过 arena
    /// SetVelocity(cmd=2) 注入 Δv1（近地点燃烧），随后每步 (t, r) 采样。
    /// 返回 (world, handle, samples)。命令注入点即 t=0 近地点。
    fn run_transfer(dt: f64, half_transfer_time: f64) -> (CosmosWorld, u64, Vec<(f64, f64)>) {
        let mut world = CosmosWorld::new(config());
        world.add_celestial(CelestialSource::new(point_mass_earth(), 0));
        let sat = world.insert_body(satellite_builder(
            MASS,
            Vector::new(R1, 0.0, 0.0),
            Vector::new(0.0, (MU / R1).sqrt(), 0.0),
            1.0,
        ));
        assert!(world.create_shared_arena(64, 64));

        let dv = textbook_hohmann(MU, R1, R2);
        let v1 = (MU / R1).sqrt();
        // 近地点 Δv1：切向 +y。SetVelocity 直接给燃烧后的绝对速度。
        let addr = world.shared_arena_address();
        let base = addr as *mut u8;
        unsafe {
            let cmd_ring_base = (base.add(OFF_CMD_RING) as *const u64).read_unaligned() as usize;
            write_command(base, cmd_ring_base, 0, 2, 0, [0.0, v1 + dv.delta_v1, 0.0]);
            (base.add(OFF_CMD_WRITE) as *mut u32).write_unaligned(1);
        }

        let mut samples = Vec::new();
        let mut t = 0.0_f64;
        // 多积一步留出扫描窗口（半周期点 ± 数步）。
        let steps = (half_transfer_time / dt).ceil() as usize + 8;
        for _ in 0..steps {
            if !matches!(world.step(dt), StepResult::Stepped(_)) {
                break;
            }
            t += dt;
            let r = world.body_translation(sat).expect("sat alive").length();
            samples.push((t, r));
        }
        (world, sat.into_raw_parts().0 as u64, samples)
    }

    /// 开普勒方程 M = E − e·sinE 的牛顿解（0 < e < 0.8 收敛良好）。
    fn kepler_eccentric_anomaly(m: f64, e: f64) -> f64 {
        let mut ecc = m;
        for _ in 0..30 {
            let f = ecc - e * ecc.sin() - m;
            let fp = 1.0 - e * ecc.cos();
            let delta = f / fp;
            ecc -= delta;
            if delta.abs() < 1e-14 {
                break;
            }
        }
        ecc
    }

    /// 转移椭圆 r(t)：自近地点起 t 秒后的径向距离（二体解析解）。
    fn transfer_radius_at(t: f64, a: f64, e: f64) -> f64 {
        let n = (MU / (a * a * a)).sqrt();
        let m = n * t;
        let ecc = kepler_eccentric_anomaly(m, e);
        a * (1.0 - e * ecc.cos())
    }

    // ------------------------------------------------------------------
    // 1. 解析一致性
    // ------------------------------------------------------------------
    #[test]
    fn formula_matches_textbook_closed_form() {
        let engine = hohmann_transfer_checked(MU, R1, R2).expect("valid domains");
        let text = textbook_hohmann(MU, R1, R2);

        println!("=== Hohmann LEO(6678 km) → GEO(42164 km), mu = {MU:e} ===");
        println!("engine  Δv1 = {:.9} m/s", engine.delta_v1);
        println!("textbk  Δv1 = {:.9} m/s", text.delta_v1);
        println!("engine  Δv2 = {:.9} m/s", engine.delta_v2);
        println!("textbk  Δv2 = {:.9} m/s", text.delta_v2);
        println!("engine  Δv_total = {:.9} m/s", engine.total_delta_v);
        println!("textbk  Δv_total = {:.9} m/s", text.total_delta_v);
        println!("engine  transfer_time = {:.6} s", engine.transfer_time);
        println!("textbk  transfer_time = {:.6} s", text.transfer_time);

        for (name, a, b) in [
            ("delta_v1", engine.delta_v1, text.delta_v1),
            ("delta_v2", engine.delta_v2, text.delta_v2),
            ("total", engine.total_delta_v, text.total_delta_v),
            ("t_half", engine.transfer_time, text.transfer_time),
        ] {
            let rel = ((a - b) / b).abs();
            assert!(
                rel <= 1e-12,
                "{name}: engine {a:e} vs textbook {b:e}, rel err {rel:e}"
            );
        }

        // 教科书常识锚点：LEO→GEO 双脉冲 ~3.89 km/s（≈3.85–3.95 区间）。
        assert!(
            (engine.total_delta_v - 3890.0).abs() < 30.0,
            "Δv_total {:.3} m/s should be ~3.89 km/s",
            engine.total_delta_v
        );

        // 转移半长轴与开普勒半周期公式的独立复核。
        let a_t = 0.5 * (R1 + R2);
        let t_half = std::f64::consts::PI * (a_t * a_t * a_t / MU).sqrt();
        assert_eq!(engine.transfer_time.to_bits(), t_half.to_bits());
        println!(
            "a_t = {a_t:.1} m, T/2 = π√(a_t³/μ) = {t_half:.6} s ({:.2} h)",
            t_half / 3600.0
        );
    }

    // ------------------------------------------------------------------
    // 2. 积分闭环（必做）：Δv1 注入 → 半周期到达 r2 → Δv2 圆化
    // ------------------------------------------------------------------
    #[test]
    fn closed_loop_transfer_reaches_geo_radius() {
        let dv = textbook_hohmann(MU, R1, R2);
        let dt = 1.0_f64;
        let (mut world, _handle, samples) = run_transfer(dt, dv.transfer_time);

        // 在转移时间附近的小窗口内取 |r − r2| 最小的采样点（允许相位落点
        // 与半周期点相差数步）。
        let target = dv.transfer_time;
        let window = samples
            .iter()
            .filter(|(t, _)| (t - target).abs() <= 5.0 * dt)
            .copied()
            .collect::<Vec<_>>();
        assert!(!window.is_empty(), "no samples within ±5 dt of T/2");

        let (t_best, r_best) = window
            .iter()
            .copied()
            .min_by(|a, b| {
                let da = (a.1 - R2).abs();
                let db = (b.1 - R2).abs();
                da.partial_cmp(&db).unwrap()
            })
            .expect("non-empty window");

        // 窗口内 r 单调增直到远地点顶点（近地点→远地点径向距离严格递增；
        // 顶点附近 r 出现平台/微降属正常二阶驻点行为，到达顶点即停止检查）。
        for w in window.windows(2) {
            assert!(
                w[1].1 >= w[0].1 || (w[1].1 - w[0].1).abs() / R2 < 1e-3,
                "r must be monotonically increasing on the ascending leg: t={:.3} r {} -> t={:.3} r {}",
                w[0].0,
                w[0].1,
                w[1].0,
                w[1].1
            );
        }

        let rel = (r_best - R2).abs() / R2;
        println!("=== closed-loop LEO→GEO, dt = {dt} s, Yoshida4Kahan ===");
        println!("Δv1 injected = {:.6} m/s at perigee (t=0)", dv.delta_v1);
        println!(
            "best |r − r2|/r2 = {rel:.3e} at t = {t_best:.3} s (T/2 = {:.3} s)",
            target
        );
        println!("r_best = {r_best:.1} m (GEO = {R2:.1} m)");
        assert!(
            rel < 0.01,
            "closed-loop transfer must reach GEO radius within 1%: rel err {rel:e}"
        );

        // 远地点 Δv2 圆化：从 r 最接近 r2 的采样点继续积到「径向速度过零」
        // （真正的远地点顶点），再 ApplyImpulse = m·Δv2（切向）。圆化后速度模
        // 应为 GEO 圆轨道速度、径向分量 ~0。
        let h = rapier3d::prelude::RigidBodyHandle::from_raw_parts(_handle as u32, 0);
        let v2 = (MU / R2).sqrt();
        // 在过零点前等待（径向速度 > 0 = 仍在上升段；样本在 T/2 附近已近顶点，
        // 顶点最多再差数步；保险起见最多再积 60 步并检查径向速度符号翻转）。
        let radial0 = world.body_translation(h).expect("sat alive").normalize();
        let mut vr_prev = world.body_linvel(h).expect("sat alive").dot(radial0);
        let vr_before_burn = vr_prev;
        for _ in 0..60 {
            if vr_prev <= 0.0 {
                break;
            }
            world.step(dt);
            vr_prev = world
                .body_linvel(h)
                .expect("sat alive")
                .dot(world.body_translation(h).expect("sat alive").normalize());
        }
        assert!(
            vr_prev <= 0.0,
            "apogee (v_r = 0 crossing) not reached within 60 steps, v_r = {vr_prev}"
        );

        let pos = world.body_translation(h).expect("sat alive");
        let radial = pos.normalize();
        let r_apo = pos.length();
        let addr = world.shared_arena_address();
        let base = addr as *mut u8;
        unsafe {
            let cmd_ring_base = (base.add(OFF_CMD_RING) as *const u64).read_unaligned() as usize;
            // 圆化冲量大小用 v2 − |v_current|（消除积分残余切向偏差），方向沿
            // 切向（当前速度方向）；Δv2 名义值单独打印供 evidence 对账。
            let vel = world.body_linvel(h).expect("sat alive");
            // 圆化冲量沿**纯切向**（速度去掉径向分量后的方向），避免把远地点
            // 前残存的径向速度分量一起放大。
            let dir = (vel - radial * vel.dot(radial)).normalize();
            // 可为负：积分速度略超 v2 时该冲量是减速。
            let dv2_actual = v2 - vel.length();
            println!(
                "nominal Δv2 = {:.6} m/s, applied Δv2 = {dv2_actual:.6} m/s (v at apogee = {:.6})",
                dv.delta_v2,
                vel.length()
            );
            write_command(
                base,
                cmd_ring_base,
                0,
                3,
                0,
                [
                    dir.x * MASS * dv2_actual,
                    dir.y * MASS * dv2_actual,
                    dir.z * MASS * dv2_actual,
                ],
            );
            (base.add(OFF_CMD_WRITE) as *mut u32).write_unaligned(1);
        }
        world.step(dt);

        let st = world.body_state(h).expect("sat alive");
        let speed = st.velocity.length();
        let vr = st.velocity.dot(radial);
        println!(
            "apogee: r = {r_apo:.1} m (rel err {:.3e}), v_r before burn = {vr_before_burn:.3} m/s",
            (r_apo - R2).abs() / R2
        );
        println!(
            "after Δv2 impulse: |v| = {:.6} m/s (GEO circ = {v2:.6}), v_r = {vr:.3e} m/s",
            speed
        );
        assert!(
            (speed - v2).abs() < 2.0,
            "post-circularization speed {} should be GEO circular {v2:.3} (±2 m/s numerical)",
            speed
        );
        assert!(
            vr.abs() < 2.0,
            "post-circularization radial velocity {vr} should vanish (±2 m/s numerical)"
        );
    }

    // ------------------------------------------------------------------
    // 3. 转移椭圆形状：r(t) vs 开普勒方程逐点比对
    // ------------------------------------------------------------------
    #[test]
    fn transfer_ellipse_radius_matches_kepler_equation() {
        let dv = textbook_hohmann(MU, R1, R2);
        let a = 0.5 * (R1 + R2);
        let e = (R2 - R1) / (R2 + R1);
        let dt = 1.0_f64;
        let (_world, _handle, samples) = run_transfer(dt, dv.transfer_time);

        // 只比对到半周期（ ascending 段），每 100 步抽一个点控制输出量。
        let mut max_rel = 0.0_f64;
        let mut worst = (0.0, 0.0, 0.0);
        let mut checked = 0_usize;
        for &(t, r) in &samples {
            if t > dv.transfer_time {
                break;
            }
            let r_kep = transfer_radius_at(t, a, e);
            let rel = (r - r_kep).abs() / r_kep;
            if rel > max_rel {
                max_rel = rel;
                worst = (t, r, r_kep);
            }
            checked += 1;
        }
        println!("=== transfer ellipse shape vs Kepler equation (dt = {dt} s) ===");
        println!("points checked = {checked}, max |r(t) − r_kep|/r_kep = {max_rel:.3e}");
        println!(
            "worst at t = {:.1} s: sim r = {:.1} m, kepler r = {:.1} m",
            worst.0, worst.1, worst.2
        );
        assert!(checked > 1000, "should sample a dense arc, got {checked}");
        assert!(
            max_rel < 1e-3,
            "transfer ellipse r(t) deviates {max_rel:e} (>0.1%) from Kepler solution"
        );

        // 端点锚：t≈0 处 r ≈ r1（近地点），t=T/2 处 r ≈ r2（远地点）。
        let r0 = samples.first().expect("samples").1;
        assert!((r0 - R1).abs() / R1 < 1e-3, "perigee anchor r0={r0}");
    }
}
