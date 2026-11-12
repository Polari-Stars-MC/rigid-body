//! 大气阻力 / 终端速度物理语义测试。
//!
//! 引擎的大气模型（`mps-cosmos::perturbation`）：
//! - 密度：`ρ(h) = surface_density · exp(-h / scale_height)`
//!   （`spaceflight::atmospheric_density_scale_height`，参考高度取表面）；
//! - 阻力：**平方律** `a = -½·ρ·|v_rel|·Cd·A/m · v_rel`，`v_rel = v - ω×r`。
//!
//! 三个钉子（全部用 `Box::leak` 的自定义中心天体控制大气参数）：
//! 1. 终端速度量级：低空（密度近似随高度缓变的层内）自由落体的稳态速度
//!    `v_t = sqrt(2·m·g / (ρ·Cd·A))` 与世界模拟实测一致（容差 10%）；
//! 2. 阻力加速度方向：纯阻力（无引力）下速度只衰减不反向——速率单调下降、
//!    方向与初速度保持一致（静止大气下阻力严格反平行于速度）；
//! 3. 衰减律：等密度层（scale_height→∞）内平方律给出
//!    `d|v|/dt = -k·v²`，即 **1/|v| 随时间线性增长**，斜率恰为
//!    `k = ½·ρ·Cd·A/m`（线性回归断言）。注：`ln|v|` 线性（指数律）是
//!    *线性*阻力 `dv/dt = -k·v` 的特征；本引擎是平方律，故钉 1/v 斜率。
//!
//! 复现命令：
//! ```text
//! cargo test -p mps-test --lib -- drag_terminal_velocity
//! ```
//!
//! 注意：本文件尚未挂入 `crates/mps-test/src/lib.rs`（由主会话统一接入）；
//! 接入后上述命令即可运行。

#[cfg(test)]
mod tests {
    use mps_cosmos::bodies::satellite_builder;
    use mps_cosmos::perturbation::atmosphere_density_at;
    use mps_cosmos::world::{
        CosmosWorld, CosmosWorldConfig, OrbitIntegration, PerturbationConfig, StepResult,
    };
    use mps_formula::celestial_data::{CelestialBody, EARTH};
    use rapier3d::prelude::Vector;

    const DT: f64 = 1.0 / 60.0;

    fn config() -> CosmosWorldConfig {
        CosmosWorldConfig {
            orbit_integration: OrbitIntegration::Verlet,
            verlet_substeps: 4,
            ..Default::default()
        }
    }

    /// 构造一个泄漏到 'static 的自定义中心天体（引擎 API 要求 `&'static`）。
    /// `rotation_rate = 0` → 静止大气，`v_rel = v`，排除共转项干扰。
    fn leaked_body(surface_density: f64, scale_height: f64) -> &'static CelestialBody {
        Box::leak(Box::new(CelestialBody {
            name: "test-atmosphere-body",
            gm: EARTH.gm,
            equatorial_radius: EARTH.equatorial_radius,
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
            ref_radius: EARTH.equatorial_radius,
            surface_density,
            scale_height,
            solar_pressure_constant: 0.0,
        }))
    }

    fn drag_cfg(cdv: f64, area: f64) -> PerturbationConfig {
        PerturbationConfig {
            enable_drag: true,
            drag_coefficient: cdv,
            area,
            ..Default::default()
        }
    }

    /// 钉子 1：终端速度量级。100 kg 球体、Cd=2.2、A=1 m²，从 1000 m 高度
    /// 静止下落 15 s。平方律终端速度 `v_t = sqrt(2mg/(ρ(h)·Cd·A))`，ρ 用
    /// 引擎自己的 `atmosphere_density_at` 在当前高度采样。落体过程中高度
    /// 下降 → 密度上升 → v_t 缓慢下降，准稳态驰豫时间 τ = v_t/g ≈ 2.9 s
    /// 远小于 v_t 漂移时标，实测稳态速度应贴住当前高度的 v_t（10% 容差）。
    #[test]
    fn terminal_velocity_matches_quadratic_drag_formula() {
        let body = leaked_body(1.225, 8_500.0); // 地表密度与 scale height
        let r_start = EARTH.equatorial_radius + 1_000.0;
        let mut world = CosmosWorld::new(config());
        world.add_celestial(mps_cosmos::gravity::CelestialSource::new(body, 0));
        world.set_central_body(Some(body));
        let mass = 100.0;
        let cd = 2.2;
        let area = 1.0;
        let sat = world.insert_body(satellite_builder(
            mass,
            Vector::new(r_start, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.0),
            1.0,
        ));
        world.set_perturbation(sat, drag_cfg(cd, area));

        let steps = 900usize; // 15 s
        for _ in 0..steps {
            if !matches!(world.step(DT), StepResult::Stepped(_)) {
                break;
            }
        }
        let st = world.body_state(sat).expect("sat alive");
        let speed = st.velocity.length();
        let altitude = st.position.length() - body.equatorial_radius;
        let g = body.gm / st.position.length().powi(2);
        let rho = atmosphere_density_at(body, altitude);
        let v_t = (2.0 * mass * g / (rho * cd * area)).sqrt();

        println!(
            "[terminal_velocity] measured steady speed = {speed:.4} m/s @ altitude {altitude:.1} m, \
             rho = {rho:.6} kg/m^3, local v_t = {v_t:.4} m/s, ratio = {:.4}",
            speed / v_t
        );
        assert!(
            (speed / v_t - 1.0).abs() < 0.10,
            "steady fall speed {speed:.3} m/s must match local terminal velocity \
             {v_t:.3} m/s within 10%"
        );
        // 方向性：应朝向行星中心下落（径向分量向内）。
        let inward = st.position.normalize().dot(st.velocity);
        assert!(inward < 0.0, "must fall inward, radial speed {inward:.3}");
    }

    /// 钉子 2：纯阻力（无引力源、静止大气）下速度只衰减不反向。
    /// 阻力严格反平行于速度 → 速率每步严格单调下降，方向恒定（横向分量
    /// 始终为 0），任何一步速率上升都是方向语义缺陷。
    #[test]
    fn drag_only_never_accelerates_along_velocity() {
        let body = leaked_body(1.225, 1.0e9); // scale_height→∞：全程等密度
        let mut world = CosmosWorld::new(config());
        world.set_central_body(Some(body)); // 只设中心体取密度，不加引力源
        let sat = world.insert_body(satellite_builder(
            100.0,
            Vector::new(body.equatorial_radius, 0.0, 0.0),
            Vector::new(300.0, 0.0, 0.0),
            1.0,
        ));
        world.set_perturbation(sat, drag_cfg(2.2, 1.0));

        let mut prev_speed = 300.0;
        for _ in 0..360 {
            // 6 s：k·v0·t = 0.0135·300·6 ≈ 24 → 速度已衰减 ~90%
            assert!(
                matches!(world.step(DT), StepResult::Stepped(_)),
                "world must step"
            );
            let st = world.body_state(sat).expect("sat alive");
            let speed = st.velocity.length();
            assert!(
                speed < prev_speed,
                "pure drag must monotonically decrease speed: {prev_speed:.6} -> {speed:.6}"
            );
            // 方向：静止大气下阻力反平行速度 → 横向分量恒为零。
            assert!(
                st.velocity.y.abs() < 1e-9 && st.velocity.z.abs() < 1e-9,
                "velocity must stay anti-parallel to drag (no lateral growth): {st:?}"
            );
            prev_speed = speed;
        }
        println!("[drag_direction] speed after 6 s = {prev_speed:.4} m/s (from 300.0)");
        assert!(
            prev_speed < 30.0,
            "drag should have removed most of the speed by now: {prev_speed:.3}"
        );
    }

    /// 钉子 3：等密度层内的衰减律。平方律 `d|v|/dt = -k·|v|²`，
    /// 解析解 `1/|v| = 1/v0 + k·t`，对 1/|v| 做最小二乘线性回归，
    /// 斜率应等于 `k = ½·ρ·Cd·A/m`（1% 容差；Verlet + 力每子步重算，
    /// dt=1/240 时离散误差 ≪ 1%）。
    #[test]
    fn speed_decay_follows_quadratic_law_inverse_v_linear_in_t() {
        let body = leaked_body(1.225, 1.0e9);
        let mut world = CosmosWorld::new(config());
        world.set_central_body(Some(body));
        let mass = 100.0;
        let cd = 2.2;
        let area = 1.0;
        let sat = world.insert_body(satellite_builder(
            mass,
            Vector::new(body.equatorial_radius, 0.0, 0.0),
            Vector::new(100.0, 0.0, 0.0),
            1.0,
        ));
        world.set_perturbation(sat, drag_cfg(cd, area));

        let k = 0.5 * 1.225 * cd * area / mass; // 0.013475 1/m
        let v0 = 100.0;
        let steps = 480usize; // 8 s：v(8s) = v0/(1+k·v0·t) ≈ 8.6 m/s
        let mut samples: Vec<(f64, f64)> = Vec::new(); // (t, 1/v)
        for i in 0..steps {
            assert!(matches!(world.step(DT), StepResult::Stepped(_)));
            if i % 8 == 7 {
                let t = (i + 1) as f64 * DT;
                let st = world.body_state(sat).expect("sat alive");
                let speed = st.velocity.length();
                assert!(speed > 0.0, "speed must stay positive: {st:?}");
                samples.push((t, 1.0 / speed));
            }
        }
        // 最小二乘斜率。
        let n = samples.len() as f64;
        let (sx, sy, sxx, sxy) = samples
            .iter()
            .fold((0.0, 0.0, 0.0, 0.0), |(sx, sy, sxx, sxy), &(t, y)| {
                (sx + t, sy + y, sxx + t * t, sxy + t * y)
            });
        let slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        let t_end = steps as f64 * DT;
        let v_end_expected = v0 / (1.0 + k * v0 * t_end);
        let st = world.body_state(sat).expect("sat alive");
        let v_end_measured = st.velocity.length();

        println!(
            "[decay_law] k = {k:.6} 1/m, fitted slope(1/v vs t) = {slope:.6} 1/m, \
             v(8 s) measured = {v_end_measured:.4} m/s, analytic = {v_end_expected:.4} m/s"
        );
        assert!(
            (slope / k - 1.0).abs() < 0.01,
            "fitted 1/v slope {slope:.6} must equal quadratic-drag k = {k:.6} within 1%"
        );
        assert!(
            (v_end_measured / v_end_expected - 1.0).abs() < 0.02,
            "end speed {v_end_measured:.4} must match analytic v0/(1+k·v0·t) = \
             {v_end_expected:.4} within 2%"
        );
    }
}
