//! 解析解对照（ROADMAP「还开着」第 6 条 / ADR-0036）。
//!
//! ## 为什么这一层是必要的
//!
//! [ADR-0034](0034-ffi-finiteness-sweep.md) 的有限性扫描只能证明「输出是有限数」。
//! [ADR-0035](0035-conservation-laws.md) 的守恒律只能证明「总量不漂移」。
//!
//! 两者都抓不到**公式写错**：一个把 `1 − Tc/Th` 写成 `Tc/Th` 的函数输出完全有限，
//! 也不违反任何守恒律，但它就是错的。
//!
//! 唯一能抓这类缺陷的是**解析解对照** —— 从物理第一原理写出期望值，与实现比对。
//! [ADR-0025](0025-riccati-bessel-mie.md)~[0033](0033-per-step-exact-integration.md)
//! 那一批正是靠这个方法找出 5 个真 bug。
//!
//! ## 两条原则
//!
//! 1. **期望值来自物理，不来自实现。** 先写 `1 − Tc/Th`，再跑测试。绝不先读实现
//!    抄一遍 —— 那只会把 bug 一起抄进断言。
//! 2. **优先用恒等式。** 恒等式（如 `E² − (pc)² = (mc²)²`、`COP_HP = COP_R + 1`）
//!    不依赖符号约定与参数顺序，写错的概率最低，且能同时验证多个函数。

#[cfg(test)]
mod tests {
    use mps_formula::plasma::{
        gyrofrequency, larmor_radius, mhd_wave_speeds, mirror_loss_cone_angle, mirror_ratio,
        plasma_beta,
    };
    use mps_formula::relativity::{
        chirp_mass, cosmological_redshift, gravitational_redshift, hubble_recession_velocity,
        photon_sphere_radius, redshift_from_wavelengths, relativistic_doppler_longitudinal,
        relativistic_energy_from_momentum, relativistic_momentum, relativistic_total_energy,
        schwarzschild_isco,
    };
    use mps_formula::thermodynamics::{
        carnot_efficiency, carnot_refrigeration_cop, entropy_change_constant_volume, heat_pump_cop,
        ideal_gas_pressure, ideal_gas_temperature, ideal_gas_volume, otto_efficiency,
        prandtl_number, reynolds_number, van_der_waals_critical_point,
    };

    /// 通用气体常数（J/(mol·K)），2019 SI 定义。
    const R: f64 = 8.314_462_618;
    /// 真空光速（m/s），SI 定义。
    const C: f64 = 299_792_458.0;
    /// 万有引力常数（m³/(kg·s²)），CODATA 2018。
    const G: f64 = 6.674_30e-11;

    fn rel(a: f64, b: f64) -> f64 {
        let d = (a - b).abs();
        let s = a.abs().max(b.abs());
        if s == 0.0 { d } else { d / s }
    }

    // ── 热力学：解析值 ──────────────────────────────────────────────────────

    /// 卡诺效率：`η = 1 − Tc/Th`。
    ///
    /// 这是热力学第二定律给的上界，任何热机都超不过。数值上也是最好验的一条：
    /// 两个温度直接算。
    #[test]
    fn carnot_efficiency_matches_the_textbook_formula() {
        for (th, tc) in [(500.0, 300.0), (800.0, 300.0), (373.15, 273.15)] {
            let eta = carnot_efficiency(th, tc).unwrap();
            let expect = 1.0 - tc / th;
            assert!(
                rel(eta, expect) < 1e-12,
                "η(Th={th}, Tc={tc}) = {eta}，应为 {expect}"
            );
            // 物理边界：0 <= η < 1。
            assert!((0.0..1.0).contains(&eta));
        }
    }

    /// **恒等式**：`COP_制冷 = (1 − η) / η`。
    ///
    /// 这条把两个**独立实现**的函数绑在一起 —— 卡诺效率与制冷 COP 都从
    /// `Th`、`Tc` 算，但走不同的公式路径。任一个写错，恒等式就破。
    ///
    /// 它不依赖符号约定，也不会因为参数顺序写反而失效（两边同时交换也一样）。
    #[test]
    fn refrigeration_cop_is_consistent_with_carnot_efficiency() {
        for (th, tc) in [(500.0, 300.0), (300.0, 280.0), (1000.0, 250.0)] {
            let eta = carnot_efficiency(th, tc).unwrap();
            let cop = carnot_refrigeration_cop(tc, th).unwrap();
            let expect = (1.0 - eta) / eta;
            assert!(
                rel(cop, expect) < 1e-12,
                "COP_R(Th={th}, Tc={tc}) = {cop}，(1−η)/η = {expect}"
            );
        }
    }

    /// **恒等式**：`COP_热泵 = COP_制冷 + 1`。
    ///
    /// 物理来源：热泵的收益是"送进高温端的热"，制冷机的收益是"从低温端抽走的热"，
    /// 两者差恰好是输入的功（归一化为 1）。
    #[test]
    fn heat_pump_cop_is_refrigeration_cop_plus_one() {
        for (th, tc) in [(500.0, 300.0), (320.0, 290.0), (900.0, 300.0)] {
            let cop_r = carnot_refrigeration_cop(tc, th).unwrap();
            let cop_hp = heat_pump_cop(tc, th).unwrap();
            assert!(
                (cop_hp - (cop_r + 1.0)).abs() < 1e-12,
                "COP_HP = {cop_hp}，COP_R + 1 = {}",
                cop_r + 1.0
            );
        }
    }

    /// 奥托循环效率：`η = 1 − r^(1−γ)`。
    ///
    /// 等价形式 `1 − 1/r^(γ−1)`。用 `r^(1−γ)` 写，与实现形式不同但数学等价 ——
    /// 这能顺带验证指数符号没写反。
    #[test]
    fn otto_efficiency_matches_the_textbook_formula() {
        for (r, gamma) in [(8.0, 1.4), (10.0, 1.4), (6.0, 1.3)] {
            let eta = otto_efficiency(r, gamma).unwrap();
            let expect = 1.0 - r.powf(1.0 - gamma);
            assert!(
                rel(eta, expect) < 1e-12,
                "η_otto(r={r}, γ={gamma}) = {eta}，应为 {expect}"
            );
            // 压缩比越大效率越高（同 γ）。
            let higher = otto_efficiency(r * 1.5, gamma).unwrap();
            assert!(higher > eta, "压缩比增大效率应升高：{eta} → {higher}");
        }
    }

    /// 理想气体状态方程：`P = nRT/V`。
    #[test]
    fn ideal_gas_law_matches_pv_equals_nrt() {
        let (n, t, vol) = (2.0, 300.0, 0.05);
        let p = ideal_gas_pressure(vol, n, t).unwrap();
        let expect = n * R * t / vol;
        assert!(rel(p, expect) < 1e-12, "P = {p}，应为 {expect}");
    }

    /// **恒等式**：`P → V → T` 三步必须回到原值。
    ///
    /// 三个函数是同一个式子的三种解，互相代入应当闭合。这条抓「某个反解写错」
    /// —— 单看每个函数的绝对值都"合理"，只有闭环才暴露。
    #[test]
    fn ideal_gas_solvers_form_a_closed_loop() {
        let (n, t, vol) = (1.5, 350.0, 0.02);
        let p = ideal_gas_pressure(vol, n, t).unwrap();
        let v2 = ideal_gas_volume(p, n, t).unwrap();
        assert!(rel(v2, vol) < 1e-12, "V 闭环：{v2} vs {vol}");

        let t2 = ideal_gas_temperature(p, vol, n).unwrap();
        assert!(rel(t2, t) < 1e-12, "T 闭环：{t2} vs {t}");
    }

    /// 定容熵变：`ΔS = n·Cv·ln(T2/T1)`。
    #[test]
    fn entropy_change_at_constant_volume_matches_the_formula() {
        let (n, cv, t1, t2) = (1.0, 20.8, 300.0, 600.0);
        let ds = entropy_change_constant_volume(n, cv, t1, t2).unwrap();
        let expect = n * cv * (t2 / t1).ln();
        assert!(rel(ds, expect) < 1e-12, "ΔS = {ds}，应为 {expect}");
        // 逆过程必须精确抵消。
        let back = entropy_change_constant_volume(n, cv, t2, t1).unwrap();
        assert!((ds + back).abs() < 1e-12, "熵变必须可逆：{ds} + {back}");
    }

    /// 普朗特数：`Pr = cp·μ/k`。
    #[test]
    fn prandtl_number_matches_the_formula() {
        let (cp, mu, k) = (1005.0, 1.81e-5, 0.026);
        let pr = prandtl_number(cp, mu, k).unwrap();
        let expect = cp * mu / k;
        assert!(rel(pr, expect) < 1e-12, "Pr = {pr}，应为 {expect}");
        // 空气的 Pr 应在 0.7 附近 —— 量级兜底。
        assert!((0.6..0.8).contains(&pr), "空气 Pr 应约 0.7，实际 {pr}");
    }

    /// 雷诺数：`Re = ρvL/μ`。
    #[test]
    fn reynolds_number_matches_the_formula() {
        let (rho, vel, len, mu) = (1.225, 10.0, 1.0, 1.81e-5);
        let re = reynolds_number(rho, vel, len, mu).unwrap();
        let expect = rho * vel * len / mu;
        assert!(rel(re, expect) < 1e-12, "Re = {re}，应为 {expect}");
        // 线性性：速度翻倍 → Re 翻倍。
        let re2 = reynolds_number(rho, vel * 2.0, len, mu).unwrap();
        assert!(rel(re2, 2.0 * re) < 1e-12, "Re 应对速度线性");
    }

    /// 范德瓦尔斯临界点：`Tc = 8a/(27Rb)`、`Pc = a/(27b²)`、`Vc = 3b`。
    ///
    /// 返回值顺序是 `(tc, pc, vc)`（据函数文档）。
    #[test]
    fn van_der_waals_critical_point_matches_the_formulas() {
        let (a, b) = (1.0, 1.0e-3);
        let (tc, pc, vc) = van_der_waals_critical_point(a, b).unwrap();
        assert!(rel(tc, 8.0 * a / (27.0 * R * b)) < 1e-12, "Tc = {tc}");
        assert!(rel(pc, a / (27.0 * b * b)) < 1e-12, "Pc = {pc}");
        assert!(rel(vc, 3.0 * b) < 1e-12, "Vc = {vc}");
        // 压缩因子 Zc = Pc·Vc/(R·Tc) = 3/8 —— 范德瓦尔斯气体的普适常数。
        let zc = pc * vc / (R * tc);
        assert!(rel(zc, 0.375) < 1e-12, "Zc 应为 3/8，实际 {zc}");
    }

    // ── 相对论：解析值 + 恒等式 ─────────────────────────────────────────────

    /// Beta = 0.6 时 gamma = 1.25，是**精确**有理数，适合当基准点。
    const BETA: f64 = 0.6;
    const GAMMA: f64 = 1.25;

    /// `γ = 1/√(1−β²)` 在 β = 0.6 时精确等于 1.25。
    ///
    /// 先验证这个前提，后面所有相对论断言都依赖它。
    #[test]
    fn gamma_benchmark_is_exact() {
        let g = 1.0 / (1.0 - BETA * BETA).sqrt();
        assert!((g - GAMMA).abs() < 1e-15, "γ(0.6c) = {g}，应为 1.25");
    }

    /// 总能量 `E = γmc²` 与动量 `p = γmv`。
    #[test]
    fn relativistic_energy_and_momentum_match_the_formulas() {
        let m = 1.0;
        let e = relativistic_total_energy(m, GAMMA).unwrap();
        assert!(rel(e, GAMMA * m * C * C) < 1e-12, "E = {e}");

        let p = relativistic_momentum(m, BETA * C).unwrap();
        assert!(rel(p, GAMMA * m * BETA * C) < 1e-12, "p = {p}");
    }

    /// **恒等式**：`E² − (pc)² = (mc²)²`（相对论色散关系）。
    ///
    /// 这是最强的一条：它把**三个独立函数**绑在一起 —— 总能量、动量、以及
    /// 「由动量反解能量」。任一个写错，等式就破。
    ///
    /// 而且它与符号约定、单位制无关：只要三者自洽，等式恒成立。
    #[test]
    fn relativistic_dispersion_relation_holds() {
        let m = 1.0;
        for beta in [0.1f64, 0.3, 0.5, 0.6, 0.8, 0.95] {
            let gamma = 1.0 / (1.0 - beta * beta).sqrt();
            let e = relativistic_total_energy(m, gamma).unwrap();
            let p = relativistic_momentum(m, beta * C).unwrap();
            let lhs = e * e - (p * C) * (p * C);
            let rhs = (m * C * C) * (m * C * C);
            assert!(
                rel(lhs, rhs) < 1e-10,
                "β={beta}：E²−(pc)² = {lhs:.6e}，应为 (mc²)² = {rhs:.6e}"
            );
        }
    }

    /// **恒等式**：`relativistic_energy_from_momentum(m, p)` 必须等于
    /// `relativistic_total_energy(m, γ)`（当 `p = γmv` 时）。
    ///
    /// 两条路径算同一个物理量，必须一致。
    #[test]
    fn energy_from_momentum_agrees_with_total_energy() {
        let m = 2.0;
        for beta in [0.2f64, 0.5, 0.7, 0.9] {
            let gamma = 1.0 / (1.0 - beta * beta).sqrt();
            let p = relativistic_momentum(m, beta * C).unwrap();
            let e1 = relativistic_energy_from_momentum(m, p).unwrap();
            let e2 = relativistic_total_energy(m, gamma).unwrap();
            assert!(
                rel(e1, e2) < 1e-10,
                "β={beta}：由动量 {e1:.6e} vs 由 γ {e2:.6e}"
            );
        }
    }

    /// 静止时 `E = mc²`、`p = 0`。
    #[test]
    fn at_rest_energy_is_rest_mass_and_momentum_is_zero() {
        let m = 3.0;
        let e = relativistic_total_energy(m, 1.0).unwrap();
        assert!(rel(e, m * C * C) < 1e-12);
        let p = relativistic_momentum(m, 0.0).unwrap();
        assert!(p.abs() < 1e-30, "静止动量应为 0，实际 {p}");
    }

    /// 多普勒纵效应：**往返互易** `f_接近 · f_远离 = f²`。
    ///
    /// 这条不依赖「approaching 为 true 是蓝移还是红移」的约定 —— 无论哪种，
    /// 两个方向的乘积都是 `f²`。
    #[test]
    fn longitudinal_doppler_is_reciprocal() {
        let f = 1000.0;
        for beta in [0.1f64, 0.3, 0.6] {
            let v = beta * C;
            let up = relativistic_doppler_longitudinal(f, v, true).unwrap();
            let down = relativistic_doppler_longitudinal(f, v, false).unwrap();
            assert!(
                rel(up * down, f * f) < 1e-10,
                "β={beta}：f⁺·f⁻ = {:.6}，应为 f² = {:.6}",
                up * down,
                f * f
            );
            // 两个方向的偏移必须相反。
            assert!(
                (up > f) != (down > f),
                "β={beta}：两个方向必须一个蓝移一个红移"
            );
        }
    }

    /// 宇宙学红移：`z = 1/a − 1`。
    #[test]
    fn cosmological_redshift_matches_the_scale_factor_formula() {
        for a in [1.0, 0.5, 0.25, 0.1] {
            let z = cosmological_redshift(a).unwrap();
            let expect = 1.0 / a - 1.0;
            assert!(rel(z, expect) < 1e-12, "z(a={a}) = {z}，应为 {expect}");
        }
        // 今天（a = 1）红移必须为零。
        assert!(cosmological_redshift(1.0).unwrap().abs() < 1e-15);
    }

    /// 波长红移：`z = (λ_obs − λ_emit)/λ_emit`。
    #[test]
    fn wavelength_redshift_matches_the_definition() {
        let (obs, emit) = (656.3, 486.1);
        let z = redshift_from_wavelengths(obs, emit).unwrap();
        let expect = (obs - emit) / emit;
        assert!(rel(z, expect) < 1e-12, "z = {z}，应为 {expect}");
        // 无位移时 z = 0。
        assert!(redshift_from_wavelengths(500.0, 500.0).unwrap().abs() < 1e-15);
    }

    /// 史瓦西 ISCO：`r = 6GM/c²`；光子球：`r = 3GM/c²`。
    ///
    /// 两者比值**精确为 2** —— 这是与 `M`、`G`、`c` 都无关的普适常数比，
    /// 所以是最干净的交叉检验。
    #[test]
    fn schwarzschild_radii_have_the_textbook_ratio() {
        let m = 1.989e30; // 太阳质量
        let isco = schwarzschild_isco(m, G).unwrap();
        let photon = photon_sphere_radius(m, G).unwrap();
        let rs = 2.0 * G * m / (C * C);

        assert!(rel(isco, 3.0 * rs) < 1e-12, "ISCO 应为 3·rs，实际 {isco}");
        assert!(
            rel(photon, 1.5 * rs) < 1e-12,
            "光子球应为 1.5·rs，实际 {photon}"
        );
        assert!(rel(isco / photon, 2.0) < 1e-12, "ISCO / 光子球 应为 2");
    }

    /// 引力红移的**弱场极限**：`z ≈ GM/(rc²)`。
    ///
    /// 只断言弱场（`r ≫ rs`）—— 那里所有合理实现都收敛到同一阶，不受
    /// 「用精确式还是近似式」的影响。
    #[test]
    fn gravitational_redshift_has_the_weak_field_limit() {
        let m = 1.989e30;
        let rs = 2.0 * G * m / (C * C);
        let r = 1000.0 * rs; // 深弱场
        let z = gravitational_redshift(m, r, G).unwrap();
        let expect = G * m / (r * C * C);
        assert!(
            rel(z, expect) < 1e-3,
            "弱场红移 z = {z:.6e}，应为 GM/(rc²) = {expect:.6e}"
        );
    }

    /// 啁啾质量：`M_c = (m1·m2)^(3/5) / (m1+m2)^(1/5)`。
    ///
    /// 等质量时退化为 `M_c = m/2^(1/5) · ...`，用对称性与极限各验一次。
    #[test]
    fn chirp_mass_matches_the_formula_and_is_symmetric() {
        let (m1, m2) = (30.0, 10.0);
        let mc = chirp_mass(m1, m2).unwrap();
        let expect = (m1 * m2).powf(0.6) / (m1 + m2).powf(0.2);
        assert!(rel(mc, expect) < 1e-12, "M_c = {mc}，应为 {expect}");

        // 对称：交换质量不变。
        let mc_swap = chirp_mass(m2, m1).unwrap();
        assert!(rel(mc, mc_swap) < 1e-12, "啁啾质量应对称");

        // 等质量 m：M_c = m / 2^(1/5)。
        let eq = chirp_mass(20.0, 20.0).unwrap();
        assert!(
            rel(eq, 20.0 / 2.0f64.powf(0.2)) < 1e-12,
            "等质量 M_c = {eq}"
        );
    }

    /// 哈勃退行速度：`v = H0·d`（低红移线性区）。
    #[test]
    fn hubble_recession_is_linear_in_distance() {
        let h0 = 70.0; // km/s/Mpc
        let d = 100.0; // Mpc
        let v = hubble_recession_velocity(d, h0).unwrap();
        // 实现是纯乘积 `H0 · d`，单位由调用方保证（H0 用 km/s/Mpc、d 用 Mpc
        // 则 v 得 km/s）。这里断言的是**乘积本身**，不做单位换算。
        assert!(rel(v, h0 * d) < 1e-12, "v = {v}，应为 H0·d = {}", h0 * d);
        // 线性性（与单位无关）。
        let v2 = hubble_recession_velocity(d * 2.0, h0).unwrap();
        assert!(rel(v2, 2.0 * v) < 1e-12, "退行速度应对距离线性");
    }

    // ── 等离子体：恒等式优先 ────────────────────────────────────────────────

    /// 真空磁导率（H/m）。2019 SI 后是测量值，与 `4π×10⁻⁷` 差 6e-10 相对。
    const MU0: f64 = 1.256_637_062_12e-6;
    /// 玻尔兹曼常数（J/K），SI 定义。
    const KB: f64 = 1.380_649e-23;

    /// **恒等式**：`r_L · ω_c = v_⟂`。
    ///
    /// 回旋半径 `r_L = m·v_⟂/(qB)` 与回旋频率 `ω_c = qB/m` 互为倒数关系 ——
    /// 两者相乘必须精确得到垂直速度。这条同时验证两个函数，且**与 q、B、m 的
    /// 单位约定无关**（只要两处一致）。
    #[test]
    fn larmor_radius_times_gyrofrequency_is_perpendicular_velocity() {
        for (q, b, m, v) in [
            (1.602_176_634e-19, 0.01, 9.109e-31, 1.0e6), // 电子
            (1.602_176_634e-19, 0.5, 1.673e-27, 1.0e5),  // 质子
            (3.204_353_268e-19, 0.2, 6.644e-27, 5.0e4),  // α 粒子
        ] {
            let r = larmor_radius(m, v, q, b).unwrap();
            let w = gyrofrequency(q, b, m).unwrap();
            assert!(
                rel(r * w, v) < 1e-12,
                "r_L·ω_c = {:.6e}，应为 v_⟂ = {v:.6e}",
                r * w
            );
            // 半径对速度线性、对磁场反比 —— 抓「除错量」这类缺陷。
            let r2 = larmor_radius(m, v * 2.0, q, b).unwrap();
            assert!(rel(r2, 2.0 * r) < 1e-12, "r_L 应对 v_⟂ 线性");
            let r3 = larmor_radius(m, v, q, b * 2.0).unwrap();
            assert!(rel(r3, r / 2.0) < 1e-12, "r_L 应对 B 反比");
        }
    }

    /// **恒等式**：`sin²(θ_lc) · R_m = 1`。
    ///
    /// 损失锥的定义就是 `sin²(θ_lc) = B_min/B_max = 1/R_m`，所以两者互逆。
    /// 这条把「磁镜比」与「损失锥角」两个独立函数绑在一起。
    #[test]
    fn loss_cone_angle_is_the_inverse_of_the_mirror_ratio() {
        for (bmax, bmin) in [(2.0, 1.0), (10.0, 1.0), (1.5, 0.5)] {
            let rm = mirror_ratio(bmax, bmin).unwrap();
            assert!(rel(rm, bmax / bmin) < 1e-12, "R_m = {rm}");
            let theta = mirror_loss_cone_angle(bmax, bmin).unwrap();
            let sin2 = theta.sin().powi(2);
            assert!(
                rel(sin2 * rm, 1.0) < 1e-12,
                "sin²θ_lc · R_m = {:.10}，应为 1（θ = {theta}）",
                sin2 * rm
            );
            // 损失锥角应在 (0, π/2)。
            assert!(theta > 0.0 && theta < std::f64::consts::FRAC_PI_2);
        }
    }

    /// **恒等式**：磁声波速满足 `v_fast² + v_slow² = c_s² + v_A²`。
    ///
    /// 这是理想 MHD 的色散关系：快慢磁声波是同一个二次方程的两个根，所以
    /// 两根之和（平方）等于系数和。**与传播角无关** —— 这是它最漂亮的地方，
    /// 任何角度都必须成立。
    #[test]
    fn magnetosonic_speeds_satisfy_the_dispersion_identity() {
        let (cs, va) = (300.0, 500.0);
        for deg in [0.0, 15.0, 30.0, 45.0, 60.0, 90.0] {
            let angle = deg * std::f64::consts::PI / 180.0;
            let (slow, _alfven, fast) = mhd_wave_speeds(cs, va, angle).unwrap();
            assert!(
                rel(fast * fast + slow * slow, cs * cs + va * va) < 1e-12,
                "θ={deg}°：v_fast²+v_slow² = {:.6}，应为 {:.6}",
                fast * fast + slow * slow,
                cs * cs + va * va
            );
            assert!(fast >= slow, "θ={deg}°：快波必须不小于慢波");
        }
    }

    /// 磁声波的两个角度极限：
    ///
    /// * **平行**（`θ = 0`）：`v_fast = max(cs, vA)`、`v_slow = min(cs, vA)`
    /// * **垂直**（`θ = 90°`）：`v_fast = √(cs² + vA²)`、`v_slow = 0`
    #[test]
    fn magnetosonic_speeds_have_the_right_limits() {
        let (cs, va) = (300.0, 500.0);

        let (slow0, _a0, fast0) = mhd_wave_speeds(cs, va, 0.0).unwrap();
        assert!(rel(fast0, va.max(cs)) < 1e-12, "平行快波 = {fast0}");
        assert!(rel(slow0, va.min(cs)) < 1e-12, "平行慢波 = {slow0}");

        // 中间的返回值是剪切 Alfvén 波速：平行时 = v_A，垂直时不能传播（0）。
        let (_, alfa0, _) = mhd_wave_speeds(cs, va, 0.0).unwrap();
        assert!(
            rel(alfa0, va) < 1e-12,
            "平行 Alfvén 波速 = {alfa0}，应为 v_A"
        );

        let half_pi = std::f64::consts::FRAC_PI_2;
        let (slow90, alfa90, fast90) = mhd_wave_speeds(cs, va, half_pi).unwrap();
        assert!(
            alfa90.abs() < 1e-9,
            "垂直 Alfvén 波速应为 0（不能垂直传播），实际 {alfa90}"
        );
        assert!(
            rel(fast90, (cs * cs + va * va).sqrt()) < 1e-12,
            "垂直快波 = {fast90}"
        );
        assert!(slow90.abs() < 1e-9, "垂直慢波应为 0，实际 {slow90}");
    }

    /// 等离子体 beta：`β = 2μ₀·n·k·T / B²`。
    #[test]
    fn plasma_beta_matches_the_formula() {
        let (n, t, b) = (1.0e19, 1.0e6, 0.1);
        let beta = plasma_beta(n, t, b).unwrap();
        let expect = 2.0 * MU0 * n * KB * t / (b * b);
        assert!(rel(beta, expect) < 1e-6, "β = {beta}，应为 {expect}");
        // beta 对温度线性、对磁场平方反比。
        let beta_t = plasma_beta(n, t * 2.0, b).unwrap();
        assert!(rel(beta_t, 2.0 * beta) < 1e-12, "β 应对 T 线性");
        let beta_b = plasma_beta(n, t, b * 2.0).unwrap();
        assert!(rel(beta_b, beta / 4.0) < 1e-12, "β 应对 B² 反比");
    }

    // ── 退化情形：光子动量（ROADMAP 历史缺陷 #3）─────────────────────────────

    /// 构造一个全零的粒子报告，用于接收输出。
    fn zero_particle() -> mps_formula::ffi::RelativisticParticle {
        mps_formula::ffi::RelativisticParticle {
            lorentz_factor: 0.0,
            total_energy: 0.0,
            kinetic_energy: 0.0,
            momentum_magnitude: 0.0,
            momentum: mps_formula::ffi::Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rapidity: 0.0,
        }
    }

    /// **轴对齐光子**（历史缺陷的原始触发条件）。
    ///
    /// `relativity.rs` 里的注释写得很清楚：
    ///
    /// > Earlier this multiplied the direction by `INFINITY`, so any axis-aligned
    /// > photon got `momentum = (inf, NaN, NaN)` (`0 * inf == NaN`) while the
    /// > function returned success.
    ///
    /// 所以这条测试直接钉住那个输入：`m = 0`、`v = (c, 0, 0)`。修好后
    /// `momentum` 必须是**单位方向** `(1, 0, 0)`，三个分量都不得是 `NaN`/`inf`。
    ///
    /// 三个坐标轴各测一次 —— `0 * inf == NaN` 只在**零分量**上发作，所以只测
    /// 对角线会漏掉（对角线三个分量都非零）。
    #[test]
    fn axis_aligned_photon_momentum_is_a_unit_vector_not_nan() {
        let cases = [
            (
                mps_formula::ffi::Vec3 {
                    x: C,
                    y: 0.0,
                    z: 0.0,
                },
                [1.0, 0.0, 0.0],
            ),
            (
                mps_formula::ffi::Vec3 {
                    x: 0.0,
                    y: C,
                    z: 0.0,
                },
                [0.0, 1.0, 0.0],
            ),
            (
                mps_formula::ffi::Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: C,
                },
                [0.0, 0.0, 1.0],
            ),
        ];
        for (velocity, expect) in cases {
            let mut out = zero_particle();
            let ok = mps_formula::relativity::rel_particle_properties(0.0, velocity, &mut out);
            assert_eq!(ok, mps_formula::ffi::Bool::TRUE);

            let m = out.momentum;
            assert!(
                m.x.is_finite() && m.y.is_finite() && m.z.is_finite(),
                "光子动量出现非有限分量：({}, {}, {})",
                m.x,
                m.y,
                m.z
            );
            assert!(
                (m.x - expect[0]).abs() < 1e-12
                    && (m.y - expect[1]).abs() < 1e-12
                    && (m.z - expect[2]).abs() < 1e-12,
                "光子动量方向应为 {expect:?}，实际 ({}, {}, {})",
                m.x,
                m.y,
                m.z
            );
            // 单位向量：长度必须为 1（不是 INFINITY）。
            let len = (m.x * m.x + m.y * m.y + m.z * m.z).sqrt();
            assert!((len - 1.0).abs() < 1e-12, "动量方向向量长度 = {len}");
        }
    }

    /// 斜向光子：动量必须是**归一化方向**，长度 1。
    #[test]
    fn diagonal_photon_momentum_is_normalised() {
        let s = 1.0 / 3.0f64.sqrt();
        let velocity = mps_formula::ffi::Vec3 {
            x: C * s,
            y: C * s,
            z: C * s,
        };
        let mut out = zero_particle();
        assert_eq!(
            mps_formula::relativity::rel_particle_properties(0.0, velocity, &mut out),
            mps_formula::ffi::Bool::TRUE
        );
        let m = out.momentum;
        for (name, got, want) in [("x", m.x, s), ("y", m.y, s), ("z", m.z, s)] {
            assert!(
                (got - want).abs() < 1e-12,
                "动量 {name} = {got}，应为 {want}"
            );
        }
    }

    /// `momentum_magnitude` 对光子是 `INFINITY`（**文档化的约定**，不是 bug）。
    ///
    /// 这条断言把「文档说的」与「代码做的」钉在一起 —— 如果将来有人把它改成
    /// 有限值，测试会提醒他同时改文档。
    ///
    /// 顺带记录一个**设计张力**：物理上光子的动量是有限的（`p = E/c`），
    /// 这里的 `INFINITY` 是 `γmv` 在 `γ→∞, m→0` 下的不定型产物。函数选择用
    /// `INFINITY` 表示「没有静止系」，方向另用单位向量给出。这是可辩护的取舍，
    /// 但与 ADR-0034「返回 TRUE 就要给有限值」的约定不一致 —— 记在这里。
    #[test]
    fn photon_momentum_magnitude_is_documented_as_infinite() {
        let velocity = mps_formula::ffi::Vec3 {
            x: C,
            y: 0.0,
            z: 0.0,
        };
        let mut out = zero_particle();
        assert_eq!(
            mps_formula::relativity::rel_particle_properties(0.0, velocity, &mut out),
            mps_formula::ffi::Bool::TRUE
        );
        assert!(out.momentum_magnitude.is_infinite());
        assert!(out.lorentz_factor.is_infinite());
    }

    /// 有质量粒子：`p = γmv`，且方向与速度同向。
    #[test]
    fn massive_particle_momentum_follows_gamma_mv() {
        let velocity = mps_formula::ffi::Vec3 {
            x: BETA * C,
            y: 0.0,
            z: 0.0,
        };
        let mut out = zero_particle();
        assert_eq!(
            mps_formula::relativity::rel_particle_properties(1.0, velocity, &mut out),
            mps_formula::ffi::Bool::TRUE
        );
        assert!(
            rel(out.lorentz_factor, GAMMA) < 1e-12,
            "γ = {}",
            out.lorentz_factor
        );
        assert!(
            rel(out.momentum_magnitude, GAMMA * BETA * C) < 1e-12,
            "|p| = {}",
            out.momentum_magnitude
        );
        assert!(rel(out.momentum.x, GAMMA * BETA * C) < 1e-12);
        assert!(out.momentum.y.abs() < 1e-12 && out.momentum.z.abs() < 1e-12);
        // 总能量与动能：E = γmc², K = (γ−1)mc²。
        assert!(rel(out.total_energy, GAMMA * C * C) < 1e-12);
        assert!(rel(out.kinetic_energy, (GAMMA - 1.0) * C * C) < 1e-12);
    }

    /// 无质量但速度低于 `c` 是**非法输入**，必须拒绝而不是返回无意义值。
    #[test]
    fn massless_below_c_is_rejected() {
        let velocity = mps_formula::ffi::Vec3 {
            x: 0.5 * C,
            y: 0.0,
            z: 0.0,
        };
        let mut out = zero_particle();
        assert_eq!(
            mps_formula::relativity::rel_particle_properties(0.0, velocity, &mut out),
            mps_formula::ffi::Bool::FALSE
        );
        // 负质量同样非法。
        let mut out2 = zero_particle();
        assert_eq!(
            mps_formula::relativity::rel_particle_properties(
                -1.0,
                mps_formula::ffi::Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0
                },
                &mut out2
            ),
            mps_formula::ffi::Bool::FALSE
        );
    }

    // ── SPH 力账（ROADMAP 历史缺陷 #1）───────────────────────────────────────

    fn sph_p(pos: [f64; 3], density: f64, pressure: f64) -> mps_core::rapier::ffi::SphParticle {
        mps_core::rapier::ffi::SphParticle {
            position: mps_core::rapier::ffi::Vec3 {
                x: pos[0],
                y: pos[1],
                z: pos[2],
            },
            velocity: mps_core::rapier::ffi::Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            mass: 1.0,
            density,
            pressure,
        }
    }

    fn zero_report() -> mps_core::rapier::ffi::SphForceReport {
        let z = || mps_core::rapier::ffi::Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        mps_core::rapier::ffi::SphForceReport {
            density: 0.0,
            pressure: 0.0,
            pressure_force: z(),
            viscosity_force: z(),
            surface_tension_force: z(),
            total_force: z(),
        }
    }

    /// 对称双粒子：**牛顿第三定律** —— 两个粒子受到的压力必须等大反向。
    ///
    /// 这条抓 ROADMAP 点名的「SPH 自贡献重复计算」以及一切非对称的邻居遍历：
    /// 如果某个粒子把自己算进邻居、或者邻居循环少走了一个方向，等大反向就破。
    ///
    /// 关闭粘性与表面张力，只留压力项 —— 压力才是对心的。
    #[test]
    fn sph_pair_forces_are_equal_and_opposite() {
        let h = 1.2;
        let d = 1.0;
        let particles = [
            sph_p([-d * 0.5, 0.0, 0.0], 1000.0, 500.0),
            sph_p([d * 0.5, 0.0, 0.0], 1000.0, 500.0),
        ];
        let eval = |target: mps_core::rapier::ffi::SphParticle| {
            let mut out = zero_report();
            let ok = mps_core::rapier::fluid::fluid_sph_estimate_forces(
                target,
                particles.as_ptr(),
                particles.len() as u32,
                h,
                2000.0, // gas_constant
                1000.0, // rest_density
                0.0,    // viscosity
                0.0,    // surface_tension
                &mut out,
            );
            assert_eq!(ok, mps_core::rapier::ffi::Bool::TRUE);
            out.total_force
        };

        let fa = eval(particles[0]);
        let fb = eval(particles[1]);

        // 力必须显著（否则等大反向是平凡的真）。
        let mag = (fa.x * fa.x + fa.y * fa.y + fa.z * fa.z).sqrt();
        assert!(mag > 1e-6, "压力必须非零，否则第三定律断言是平凡的：{mag}");

        // 等大反向。
        let scale = mag.max(1e-30);
        assert!(
            ((fa.x + fb.x) / scale).abs() < 1e-9,
            "x 方向不满足第三定律：{:.6} + {:.6}",
            fa.x,
            fb.x
        );
        assert!(((fa.y + fb.y) / scale).abs() < 1e-9, "y 方向不满足第三定律");
        assert!(((fa.z + fb.z) / scale).abs() < 1e-9, "z 方向不满足第三定律");

        // 排斥：A 在 -x，B 在 +x，所以 A 应被推向 -x，B 被推向 +x。
        assert!(fa.x < 0.0, "A 应被推向 -x，实际 {:.6}", fa.x);
        assert!(fb.x > 0.0, "B 应被推向 +x，实际 {:.6}", fb.x);
    }

    /// 孤立粒子（没有任何邻居）受到的总力必须**精确为零**。
    ///
    /// 这是「自贡献」最直接的检验：如果粒子把自己算成邻居，且核梯度在 `r = 0`
    /// 处没有正确取零，就会得到一个虚假的自力 —— 那会让整团流体自发加速。
    #[test]
    fn isolated_particle_feels_no_force() {
        let particles = [sph_p([0.0, 0.0, 0.0], 1000.0, 500.0)];
        let mut out = zero_report();
        assert_eq!(
            mps_core::rapier::fluid::fluid_sph_estimate_forces(
                particles[0],
                particles.as_ptr(),
                1,
                1.2,
                2000.0,
                1000.0,
                0.0,
                0.0,
                &mut out,
            ),
            mps_core::rapier::ffi::Bool::TRUE
        );
        let f = out.total_force;
        assert_eq!(f.x, 0.0, "孤立粒子不得受 x 方向自力");
        assert_eq!(f.y, 0.0, "孤立粒子不得受 y 方向自力");
        assert_eq!(f.z, 0.0, "孤立粒子不得受 z 方向自力");
    }

    /// 对称晶格中心粒子受到的总力必须为零（净力抵消）。
    ///
    /// 6 个面心邻居关于中心对称，压力项逐对抵消。这条不依赖任何系数取值 ——
    /// 只要邻居遍历是**对称且完整**的，净力必然为零。
    #[test]
    fn symmetric_lattice_centre_feels_no_net_force() {
        let d = 1.0;
        let h = 1.2;
        let mut particles = vec![sph_p([0.0, 0.0, 0.0], 1000.0, 500.0)];
        for axis in 0..3 {
            for sign in [-1.0, 1.0] {
                let mut p = [0.0, 0.0, 0.0];
                p[axis] = sign * d;
                particles.push(sph_p(p, 1000.0, 500.0));
            }
        }
        let mut out = zero_report();
        assert_eq!(
            mps_core::rapier::fluid::fluid_sph_estimate_forces(
                particles[0],
                particles.as_ptr(),
                particles.len() as u32,
                h,
                2000.0,
                1000.0,
                0.0,
                0.0,
                &mut out,
            ),
            mps_core::rapier::ffi::Bool::TRUE
        );
        let f = out.total_force;
        let scale = out.pressure.abs().max(1.0);
        assert!(
            f.x.abs() < 1e-9 * scale && f.y.abs() < 1e-9 * scale && f.z.abs() < 1e-9 * scale,
            "对称晶格中心净力应为零，实际 ({:.3e}, {:.3e}, {:.3e})",
            f.x,
            f.y,
            f.z
        );
    }
}
