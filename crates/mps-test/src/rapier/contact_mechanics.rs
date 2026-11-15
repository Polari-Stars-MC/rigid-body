#[cfg(test)]
mod tests {
    use mps_formula::contact_mechanics::*;
    use std::f64::consts::PI;

    // -----------------------------------------------------------------------
    // 有效材料/几何属性
    // -----------------------------------------------------------------------

    #[test]
    fn effective_modulus_of_identical_bodies_is_half_the_youngs_modulus() {
        // 两个相同材料：1/E* = 2(1-ν²)/E  ⇒  E* = E / (2(1-ν²))
        let e = 200.0e9;
        let nu = 0.3;
        let e_star = effective_youngs_modulus(e, nu, e, nu).unwrap();
        let expected = e / (2.0 * (1.0 - nu * nu));
        assert!((e_star - expected).abs() / expected < 1e-12);
    }

    #[test]
    fn effective_modulus_against_rigid_plane() {
        let e = 70.0e9;
        let nu = 0.33;
        let e_star = effective_modulus_rigid_plane(e, nu).unwrap();
        let expected = e / (1.0 - nu * nu);
        assert!((e_star - expected).abs() / expected < 1e-12);
        // 与双体形式在 youngs2 = ∞ 时一致
        let both = effective_youngs_modulus(e, nu, f64::INFINITY, 0.0).unwrap();
        assert!((both - e_star).abs() / e_star < 1e-12);
    }

    #[test]
    fn effective_radius_rules() {
        // 球-平面：R* = R
        assert_eq!(effective_radius(2.0, f64::INFINITY).unwrap(), 2.0);
        // 两个等半径球：R* = R/2
        assert_eq!(effective_radius(2.0, 2.0).unwrap(), 1.0);
        // 两个平面：无定义
        assert!(effective_radius(f64::INFINITY, f64::INFINITY).is_none());
        // 负半径/零半径非法
        assert!(effective_radius(-1.0, 1.0).is_none());
        assert!(effective_radius(0.0, 1.0).is_none());
    }

    #[test]
    fn poisson_ratio_outside_physical_range_is_rejected() {
        // -1 与 0.5 是开区间端点；0.5 表示不可压缩
        assert!(effective_youngs_modulus(1e9, 0.5, 1e9, 0.3).is_none());
        assert!(effective_youngs_modulus(1e9, -1.0, 1e9, 0.3).is_none());
        assert!(effective_youngs_modulus(1e9, 0.49, 1e9, 0.3).is_some());
    }

    // -----------------------------------------------------------------------
    // Hertz 法向
    // -----------------------------------------------------------------------

    #[test]
    fn hertz_force_matches_the_closed_form() {
        // F = (4/3) E* √R* δ^{3/2}
        let (e_star, r_star, delta) = (1.0e9, 0.01, 1.0e-4);
        let f = hertz_force(delta, e_star, r_star).unwrap();
        let expected = (4.0 / 3.0) * e_star * r_star.sqrt() * delta.powf(1.5);
        assert!((f - expected).abs() / expected < 1e-12);
    }

    #[test]
    fn hertz_force_scales_as_delta_to_the_three_halves() {
        // 经典检验：δ 增 4 倍 ⇒ F 增 8 倍
        let base = hertz_force(1.0e-4, 1e9, 0.01).unwrap();
        let scaled = hertz_force(4.0e-4, 1e9, 0.01).unwrap();
        assert!((scaled / base - 8.0).abs() < 1e-9);
    }

    #[test]
    fn hertz_is_non_adhesive_below_zero_penetration() {
        // 分离的物体不产生拉力
        assert_eq!(hertz_force(-1.0e-3, 1e9, 0.01).unwrap(), 0.0);
        assert_eq!(hertz_force(0.0, 1e9, 0.01).unwrap(), 0.0);
    }

    #[test]
    fn hertz_contact_pressure_relation() {
        // 峰值压力是平均压力的 1.5 倍（Hertz 分布的解析结论）
        let c = hertz_contact(1.0e-4, 1e9, 0.01).unwrap();
        assert!(c.force > 0.0 && c.contact_radius > 0.0);
        assert!(
            (c.peak_pressure / c.mean_pressure - 1.5).abs() < 1e-12,
            "p0/pm = {}",
            c.peak_pressure / c.mean_pressure
        );
        // 平均压力 = F / (πa²)
        let area = PI * c.contact_radius * c.contact_radius;
        assert!((c.mean_pressure - c.force / area).abs() / c.mean_pressure < 1e-12);
    }

    #[test]
    fn hertz_contact_radius_matches_sqrt_r_delta() {
        let (r_star, delta) = (0.01, 1.0e-4);
        let c = hertz_contact(delta, 1e9, r_star).unwrap();
        assert!((c.contact_radius - (r_star * delta).sqrt()).abs() < 1e-18);
    }

    #[test]
    fn hertz_secant_stiffness_is_consistent_with_force_derivative() {
        // dF/dδ = 2 E* √(R* δ)：与数值差分对照
        let (e_star, r_star, delta) = (1.0e9, 0.01, 1.0e-4);
        let c = hertz_contact(delta, e_star, r_star).unwrap();
        let h = delta * 1e-6;
        let f_plus = hertz_force(delta + h, e_star, r_star).unwrap();
        let f_minus = hertz_force(delta - h, e_star, r_star).unwrap();
        let numeric = (f_plus - f_minus) / (2.0 * h);
        assert!(
            (c.secant_stiffness - numeric).abs() / numeric < 1e-6,
            "analytic {} vs numeric {}",
            c.secant_stiffness,
            numeric
        );
    }

    #[test]
    fn hertz_linearized_stiffness_reproduces_force_at_design_point() {
        // 线性化刚度 × 设计穿透量 == 该点的 Hertz 力
        let (e_star, r_star, delta) = (1.0e9, 0.01, 2.0e-4);
        let k = hertz_linearized_stiffness(delta, e_star, r_star).unwrap();
        let f = hertz_force(delta, e_star, r_star).unwrap();
        assert!((k * delta - f).abs() / f < 1e-12);
    }

    #[test]
    fn hertz_rejects_bad_inputs() {
        assert!(hertz_force(f64::NAN, 1e9, 0.01).is_none());
        assert!(hertz_force(1e-4, 0.0, 0.01).is_none());
        assert!(hertz_force(1e-4, 1e9, 0.0).is_none());
        assert!(hertz_force(1e-4, f64::INFINITY, 0.01).is_none());
        assert!(hertz_linearized_stiffness(0.0, 1e9, 0.01).is_none());
    }

    // -----------------------------------------------------------------------
    // Mindlin 切向
    // -----------------------------------------------------------------------

    #[test]
    fn mindlin_sticks_below_the_coulomb_cap() {
        // k_t = 8 G* a
        let (g_star, a) = (1.0e9, 1.0e-3);
        let k = mindlin_tangential_stiffness_stick(g_star, a).unwrap();
        assert!((k - 8.0 * g_star * a).abs() / k < 1e-12);

        let c = mindlin_contact(1.0e-6, 100.0, 0.5, g_star, a).unwrap();
        assert!(!c.sliding);
        assert!((c.force - k * 1.0e-6).abs() / c.force < 1e-12);
        assert!((c.tangential_stiffness - k).abs() / k < 1e-12);
    }

    #[test]
    fn mindlin_saturates_at_the_coulomb_cap() {
        let (g_star, a, normal, mu) = (1.0e9, 1.0e-3, 100.0, 0.5);
        // 位移大到必然超过 μN
        let c = mindlin_contact(1.0, normal, mu, g_star, a).unwrap();
        assert!(c.sliding);
        assert!((c.force - mu * normal).abs() < 1e-12);
        // 饱和后追加位移不再增加力
        assert_eq!(c.tangential_stiffness, 0.0);
    }

    #[test]
    fn mindlin_force_is_never_above_the_cap() {
        let (g_star, a, normal, mu) = (1.0e9, 1.0e-3, 50.0, 0.3);
        for i in 0..200 {
            let d = i as f64 * 1.0e-5;
            let c = mindlin_contact(d, normal, mu, g_star, a).unwrap();
            assert!(
                c.force <= mu * normal + 1e-12,
                "d={d} force={} cap={}",
                c.force,
                mu * normal
            );
        }
    }

    #[test]
    fn mindlin_with_zero_normal_force_exerts_nothing() {
        let c = mindlin_contact(1.0, 0.0, 0.5, 1e9, 1e-3).unwrap();
        assert_eq!(c.force, 0.0);
        assert!(!c.sliding);
    }

    #[test]
    fn effective_shear_modulus_matches_hand_computation() {
        // 同材料两体：1/G* = 2(2-ν)/G
        let (e, nu) = (200.0e9, 0.3);
        let g = e / (2.0 * (1.0 + nu));
        let g_star = effective_shear_modulus(e, nu, e, nu).unwrap();
        let expected = g / (2.0 * (2.0 - nu));
        assert!((g_star - expected).abs() / expected < 1e-12);
    }

    #[test]
    fn mindlin_rejects_negative_friction_and_bad_radius() {
        assert!(mindlin_contact(0.1, 100.0, -0.1, 1e9, 1e-3).is_none());
        assert!(mindlin_contact(0.1, 100.0, 0.5, 1e9, 0.0).is_none());
        assert!(mindlin_tangential_stiffness_stick(0.0, 1e-3).is_none());
    }

    // -----------------------------------------------------------------------
    // 耗散扩展（Hertz + Hunt-Crossley）
    // -----------------------------------------------------------------------

    #[test]
    fn hunt_crossley_reduces_to_hertz_at_zero_rate() {
        let (e_star, r_star, delta) = (1.0e9, 0.01, 1.0e-4);
        let pure = hertz_force(delta, e_star, r_star).unwrap();
        let damped = hertz_hunt_crossley_force(delta, 0.0, e_star, r_star, 5.0).unwrap();
        assert!((damped - pure).abs() / pure < 1e-12);
    }

    #[test]
    fn hunt_crossley_damps_approach_only() {
        let (e_star, r_star, delta) = (1.0e9, 0.01, 1.0e-4);
        let pure = hertz_force(delta, e_star, r_star).unwrap();
        // 接近：力变大
        let approaching = hertz_hunt_crossley_force(delta, 1.0, e_star, r_star, 5.0).unwrap();
        assert!(approaching > pure);
        // 分离：阻尼项不生效（不得凭空产生能量）
        let separating = hertz_hunt_crossley_force(delta, -1.0, e_star, r_star, 5.0).unwrap();
        assert!((separating - pure).abs() / pure < 1e-12);
    }

    #[test]
    fn hunt_crossley_never_pulls() {
        // 极大的 alpha + 高速分离：力必须被夹在 0，而不是变成吸附
        let f = hertz_hunt_crossley_force(1.0e-4, -1.0e6, 1e9, 0.01, 1.0).unwrap();
        assert!(f >= 0.0, "force = {f}");
    }

    #[test]
    fn hunt_crossley_rejects_negative_alpha() {
        assert!(hertz_hunt_crossley_force(1e-4, 0.0, 1e9, 0.01, -1.0).is_none());
    }

    // -----------------------------------------------------------------------
    // 物理量级抽查：钢球压钢板的接触半径应在 0.1 mm 量级
    // -----------------------------------------------------------------------

    #[test]
    fn steel_ball_on_steel_plate_contact_is_physically_plausible() {
        // E = 210 GPa, ν = 0.3（钢），球 R = 10 mm，载荷 100 N
        let e = 210.0e9;
        let nu = 0.3;
        let r = 0.01;
        let load = 100.0;
        let e_star = effective_modulus_rigid_plane(e, nu).unwrap();
        let r_star = effective_radius_sphere_plane(r).unwrap();

        // 由 F 反解 δ：δ = (3F/(4 E* √R*))^{2/3}
        let delta = (3.0 * load / (4.0 * e_star * r_star.sqrt())).powf(2.0 / 3.0);
        let c = hertz_contact(delta, e_star, r_star).unwrap();

        // 力自洽
        assert!((c.force - load).abs() / load < 1e-9);
        // 接触半径：a = √(Rδ)，钢球 100 N 下应在几十微米量级
        assert!(
            (1.0e-5..1.0e-3).contains(&c.contact_radius),
            "contact radius {} m outside the expected 10 µm .. 1 mm band",
            c.contact_radius
        );
        // 峰值压力：钢的 Hertz 接触典型在 1 GPa 量级
        assert!(
            (1.0e8..1.0e10).contains(&c.peak_pressure),
            "peak pressure {} Pa outside the expected 0.1..10 GPa band",
            c.peak_pressure
        );
    }
}
