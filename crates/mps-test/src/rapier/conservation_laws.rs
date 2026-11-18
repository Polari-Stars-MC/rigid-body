//! 守恒律断言（ROADMAP「还开着」第 6 条 / ADR-0035）。
//!
//! ## 为什么需要这一组
//!
//! `docs/ROADMAP.md` 第 6 条举的三个历史缺陷：
//!
//! * **SPH 自贡献重复计算**（每个粒子把自己算进邻居）
//! * **轮胎力量纲错误**
//! * **光子动量必得 NaN**
//!
//! 三个都在「测试全绿」的状态下存在过。它们有共同特征：**不是崩溃，不是 NaN，
//! 就是数值错了**。[ADR-0034](0034-ffi-finiteness-sweep.md) 的有限性扫描抓不到
//! 这类 —— 输出是有限的好数字，只是错的。
//!
//! 守恒律是抓这类缺陷的机械手段：**总动量、总角动量、总能量在无外力时必须守恒**。
//! 任何"多算一份力""少乘一个质量""积分器写反"都会立刻破坏它。
//!
//! ## 这组测试的位置
//!
//! 放在 `mps-test`，只用 C ABI（与 `friction_restitution_energy.rs` 同风格）。
//! 不改生产代码就能跑，所以它测的是**发布出去的那个面**，不是内部实现。

#[cfg(test)]
mod tests {
    use mps_core::rapier::collider::{
        collider_builder_build, collider_builder_create_ex, collider_builder_set_friction,
        collider_builder_set_restitution, collider_set_restitution_combine_rule,
        world_insert_collider_with_parent,
    };
    use mps_core::rapier::ffi::{BodyStatus, Bool, RigidBodyHandleRaw, ShapeDesc, ShapeType, Vec3};
    use mps_core::rapier::rigid_body::{
        rigid_body_builder_build, rigid_body_builder_create, rigid_body_builder_set_angvel,
        rigid_body_builder_set_can_sleep, rigid_body_builder_set_linear_damping,
        rigid_body_builder_set_linvel, rigid_body_builder_set_translation, rigid_body_get_angvel,
        rigid_body_get_linvel, rigid_body_get_mass, rigid_body_get_translation,
        world_insert_rigid_body,
    };
    use mps_core::rapier::world::{world_create, world_destroy, world_step};

    const DT: f64 = 1.0 / 60.0;
    /// Max combine rule（`collider_set_restitution_combine_rule` 的 3）。
    const COMBINE_MAX: u32 = 3;

    fn v(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    fn ball(radius: f64) -> ShapeDesc {
        ShapeDesc {
            shape_type: ShapeType::Ball as u32,
            a: radius,
            ..Default::default()
        }
    }

    /// 零重力世界：动量/角动量测试的基础（没有外力）。
    fn make_zero_g_world() -> *mut mps_core::rapier::ffi::WorldHandle {
        world_create(v(0.0, 0.0, 0.0))
    }

    fn make_world(gy: f64) -> *mut mps_core::rapier::ffi::WorldHandle {
        world_create(v(0.0, gy, 0.0))
    }

    /// 生成一个动态球体。默认关闭休眠、清零阻尼 —— 否则能量会被静默抽走，
    /// 守恒断言会误报。
    fn spawn(
        world: *mut mps_core::rapier::ffi::WorldHandle,
        pos: Vec3,
        linvel: Vec3,
        angvel: Vec3,
        radius: f64,
        restitution: f64,
        friction: f64,
    ) -> RigidBodyHandleRaw {
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_translation(b, pos);
        rigid_body_builder_set_linvel(b, linvel);
        rigid_body_builder_set_angvel(b, angvel);
        rigid_body_builder_set_can_sleep(b, Bool::FALSE);
        rigid_body_builder_set_linear_damping(b, 0.0);
        let body = world_insert_rigid_body(world, rigid_body_builder_build(b));

        let cb = collider_builder_create_ex(ball(radius));
        collider_builder_set_restitution(cb, restitution);
        collider_builder_set_friction(cb, friction);
        let col = world_insert_collider_with_parent(world, collider_builder_build(cb), body);
        collider_set_restitution_combine_rule(world, col, COMBINE_MAX);
        body
    }

    /// 总线性动量 `Σ m·v`。
    fn momentum(
        world: *mut mps_core::rapier::ffi::WorldHandle,
        bodies: &[RigidBodyHandleRaw],
    ) -> Vec3 {
        let mut p = v(0.0, 0.0, 0.0);
        for &b in bodies {
            let m = rigid_body_get_mass(world, b);
            let vel = rigid_body_get_linvel(world, b);
            p.x += m * vel.x;
            p.y += m * vel.y;
            p.z += m * vel.z;
        }
        p
    }

    /// 总角动量 `Σ m·(r × v)`（相对世界原点）。
    ///
    /// 注意：这**不含**自转角动量 `I·ω`。测试里给所有球体零 `angvel`，所以自转项
    /// 为零；这样断言的是纯粹的轨道角动量守恒。
    fn angular_momentum(
        world: *mut mps_core::rapier::ffi::WorldHandle,
        bodies: &[RigidBodyHandleRaw],
    ) -> Vec3 {
        let mut l = v(0.0, 0.0, 0.0);
        for &b in bodies {
            let m = rigid_body_get_mass(world, b);
            let p = rigid_body_get_translation(world, b);
            let vel = rigid_body_get_linvel(world, b);
            // r × (m·v)
            l.x += p.y * m * vel.z - p.z * m * vel.y;
            l.y += p.z * m * vel.x - p.x * m * vel.z;
            l.z += p.x * m * vel.y - p.y * m * vel.x;
        }
        l
    }

    /// 总动能 `Σ ½m|v|²`（不含转动动能 —— 所有测试用零 `angvel`）。
    fn kinetic(
        world: *mut mps_core::rapier::ffi::WorldHandle,
        bodies: &[RigidBodyHandleRaw],
    ) -> f64 {
        let mut ke = 0.0;
        for &b in bodies {
            let m = rigid_body_get_mass(world, b);
            let vel = rigid_body_get_linvel(world, b);
            ke += 0.5 * m * (vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
        }
        ke
    }

    /// 总机械能 `Σ (½m|v|² + m·g·y)`。
    fn mechanical(
        world: *mut mps_core::rapier::ffi::WorldHandle,
        bodies: &[RigidBodyHandleRaw],
        g: f64,
    ) -> f64 {
        let mut e = 0.0;
        for &b in bodies {
            let m = rigid_body_get_mass(world, b);
            let vel = rigid_body_get_linvel(world, b);
            let p = rigid_body_get_translation(world, b);
            e += 0.5 * m * (vel.x * vel.x + vel.y * vel.y + vel.z * vel.z) + m * g * p.y;
        }
        e
    }

    fn rel_err(a: f64, b: f64) -> f64 {
        let d = (a - b).abs();
        let s = a.abs().max(b.abs());
        if s == 0.0 { d } else { d / s }
    }

    // ── 1. 线性动量 ─────────────────────────────────────────────────────────

    /// 无外力时总线性动量必须守恒。
    ///
    /// 这是**最强的一条**：任何"凭空多算一份力""质量算错""速度更新写反"都会破坏它。
    /// 容差取 `1e-10`（无接触、无重力，只剩积分舍入）。
    #[test]
    fn linear_momentum_is_conserved_without_forces() {
        let world = make_zero_g_world();
        let bodies = vec![
            spawn(
                world,
                v(-50.0, 0.0, 0.0),
                v(3.0, 1.0, -2.0),
                v(0.0, 0.0, 0.0),
                0.5,
                0.0,
                0.0,
            ),
            spawn(
                world,
                v(0.0, 50.0, 0.0),
                v(-1.0, 4.0, 0.5),
                v(0.0, 0.0, 0.0),
                0.8,
                0.0,
                0.0,
            ),
            spawn(
                world,
                v(50.0, 0.0, 50.0),
                v(0.0, -2.0, 7.0),
                v(0.0, 0.0, 0.0),
                0.3,
                0.0,
                0.0,
            ),
        ];
        // 三体相距 50+ 单位，半径 ≤0.8，永不接触。

        let p0 = momentum(world, &bodies);
        for _ in 0..240 {
            world_step(world, DT);
        }
        let p1 = momentum(world, &bodies);

        assert!(
            (p1.x - p0.x).abs() < 1e-10,
            "p.x 漂移 {:.3e}（{:.6} → {:.6}）",
            (p1.x - p0.x).abs(),
            p0.x,
            p1.x
        );
        assert!(
            (p1.y - p0.y).abs() < 1e-10,
            "p.y 漂移 {:.3e}",
            (p1.y - p0.y).abs()
        );
        assert!(
            (p1.z - p0.z).abs() < 1e-10,
            "p.z 漂移 {:.3e}",
            (p1.z - p0.z).abs()
        );

        world_destroy(world);
    }

    // ── 2. 角动量 ───────────────────────────────────────────────────────────

    /// 无外力矩时总角动量必须守恒。
    ///
    /// 与线性动量互补：它能抓到"力臂算错""力矩符号反"这类缺陷 —— 那类错误可以
    /// 让线性动量守恒但角动量不守恒。
    #[test]
    fn angular_momentum_is_conserved_without_torques() {
        let world = make_zero_g_world();
        // 每个物体都给一个切向速度，让角动量非零（否则守恒断言是平凡的）。
        let bodies = vec![
            spawn(
                world,
                v(20.0, 0.0, 0.0),
                v(0.0, 0.0, 5.0),
                v(0.0, 0.0, 0.0),
                0.5,
                0.0,
                0.0,
            ),
            spawn(
                world,
                v(0.0, -30.0, 0.0),
                v(2.0, 0.0, 0.0),
                v(0.0, 0.0, 0.0),
                0.6,
                0.0,
                0.0,
            ),
            spawn(
                world,
                v(0.0, 0.0, -40.0),
                v(0.0, 3.0, 0.0),
                v(0.0, 0.0, 0.0),
                0.4,
                0.0,
                0.0,
            ),
        ];

        let l0 = angular_momentum(world, &bodies);
        assert!(
            l0.x.abs() + l0.y.abs() + l0.z.abs() > 1.0,
            "角动量必须是显著的，否则守恒断言是平凡的：{l0:?}"
        );

        for _ in 0..240 {
            world_step(world, DT);
        }
        let l1 = angular_momentum(world, &bodies);

        for (name, a, b) in [("Lx", l0.x, l1.x), ("Ly", l0.y, l1.y), ("Lz", l0.z, l1.z)] {
            assert!(
                (b - a).abs() < 1e-9,
                "{name} 漂移 {:.3e}（{a:.6} → {b:.6}）",
                (b - a).abs()
            );
        }

        world_destroy(world);
    }

    // ── 3. 自由落体的机械能 ─────────────────────────────────────────────────

    /// 无接触的自由落体：`KE + PE` 必须近似守恒。
    ///
    /// 半隐式欧拉（symplectic Euler）会让能量在**有界范围内振荡**而不是单调漂移，
    /// 所以这里断言"相对漂移小于 2%"，不是精确相等。
    ///
    /// 这条能抓到"重力加速度被应用了两次""重力缩放被忽略"这类缺陷。
    #[test]
    fn free_fall_conserves_mechanical_energy() {
        let g = 9.81;
        let world = make_world(-g);
        let bodies = vec![
            spawn(
                world,
                v(0.0, 100.0, 0.0),
                v(0.0, 0.0, 0.0),
                v(0.0, 0.0, 0.0),
                0.5,
                0.0,
                0.0,
            ),
            spawn(
                world,
                v(30.0, 60.0, 0.0),
                v(1.0, 0.0, 0.0),
                v(0.0, 0.0, 0.0),
                0.5,
                0.0,
                0.0,
            ),
        ];

        let e0 = mechanical(world, &bodies, g);
        assert!(e0.abs() > 1.0, "机械能必须显著：{e0}");

        for _ in 0..120 {
            world_step(world, DT);
        }
        let e1 = mechanical(world, &bodies, g);

        assert!(
            rel_err(e0, e1) < 0.02,
            "机械能漂移 {:.4}%（{e0:.6} → {e1:.6}）",
            rel_err(e0, e1) * 100.0
        );

        world_destroy(world);
    }

    // ── 4. 弹性碰撞 ─────────────────────────────────────────────────────────

    /// 恢复系数 = 1 的正碰：总动能必须守恒。
    ///
    /// 这条钉住碰撞冲量的**能量学**。如果冲量公式少乘/多乘一个系数，动量可能仍然
    /// 守恒（等大反向）但动能会变 —— 所以它和动量测试互补。
    #[test]
    fn elastic_collision_conserves_kinetic_energy() {
        let world = make_zero_g_world();
        // 沿 x 轴正碰：A 在左向右，B 在右向左。
        let bodies = vec![
            spawn(
                world,
                v(-3.0, 0.0, 0.0),
                v(4.0, 0.0, 0.0),
                v(0.0, 0.0, 0.0),
                0.5,
                1.0,
                0.0,
            ),
            spawn(
                world,
                v(3.0, 0.0, 0.0),
                v(-4.0, 0.0, 0.0),
                v(0.0, 0.0, 0.0),
                0.5,
                1.0,
                0.0,
            ),
        ];

        let ke0 = kinetic(world, &bodies);
        let p0 = momentum(world, &bodies);
        assert!(ke0 > 0.0);

        // 走到碰撞后再多走一段，确保冲量已经施加。
        for _ in 0..180 {
            world_step(world, DT);
        }
        let ke1 = kinetic(world, &bodies);
        let p1 = momentum(world, &bodies);

        // 动量在碰撞中必须严格守恒。
        assert!(
            (p1.x - p0.x).abs() < 1e-9,
            "碰撞前后动量不守恒：{:.6} → {:.6}",
            p0.x,
            p1.x
        );
        // 动能：完全弹性时守恒（容许求解器的少量数值耗散）。
        assert!(
            rel_err(ke0, ke1) < 0.05,
            "弹性碰撞动能漂移 {:.2}%（{ke0:.6} → {ke1:.6}）",
            rel_err(ke0, ke1) * 100.0
        );

        world_destroy(world);
    }

    // ── 5. 非弹性碰撞 ───────────────────────────────────────────────────────

    /// 恢复系数 = 0 的正碰：动能必须**减少**，且不能增加。
    ///
    /// 与上一条形成对照：同一条碰撞路径，只改恢复系数，能量行为必须反向。
    /// 如果两条都"守恒"，说明恢复系数根本没生效。
    #[test]
    fn inelastic_collision_dissipates_energy() {
        let world = make_zero_g_world();
        let bodies = vec![
            spawn(
                world,
                v(-3.0, 0.0, 0.0),
                v(4.0, 0.0, 0.0),
                v(0.0, 0.0, 0.0),
                0.5,
                0.0,
                0.0,
            ),
            spawn(
                world,
                v(3.0, 0.0, 0.0),
                v(-4.0, 0.0, 0.0),
                v(0.0, 0.0, 0.0),
                0.5,
                0.0,
                0.0,
            ),
        ];

        let ke0 = kinetic(world, &bodies);
        let p0 = momentum(world, &bodies);

        let mut prev = ke0;
        for _ in 0..180 {
            world_step(world, DT);
            let ke = kinetic(world, &bodies);
            assert!(
                ke <= prev * 1.001 + 1e-9,
                "完全非弹性碰撞中动能不得增加：{prev:.6} → {ke:.6}"
            );
            prev = ke;
        }
        let ke1 = kinetic(world, &bodies);

        // 动量仍然必须守恒（非弹性损失的是能量，不是动量）。
        assert!((p0.x - momentum(world, &bodies).x).abs() < 1e-9);
        // 动能必须显著下降（完全非弹性正碰，等质量时掉到一半以下）。
        assert!(
            ke1 < ke0 * 0.9,
            "完全非弹性碰撞应显著损失动能：{ke0:.6} → {ke1:.6}"
        );

        world_destroy(world);
    }

    // ── 6. 阻尼 ─────────────────────────────────────────────────────────────

    /// 线性阻尼必须让动能**单调下降**。
    ///
    /// 阻尼是"故意不守恒"的机制。如果它被忽略，能量会守恒 —— 这条断言反过来
    /// 证明阻尼参数确实接进了求解器。
    #[test]
    fn linear_damping_dissipates_energy_monotonically() {
        let world = make_zero_g_world();
        let b = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_translation(b, v(0.0, 0.0, 0.0));
        rigid_body_builder_set_linvel(b, v(10.0, 0.0, 0.0));
        rigid_body_builder_set_can_sleep(b, Bool::FALSE);
        rigid_body_builder_set_linear_damping(b, 2.0);
        let body = world_insert_rigid_body(world, rigid_body_builder_build(b));
        let cb = collider_builder_create_ex(ball(0.5));
        world_insert_collider_with_parent(world, collider_builder_build(cb), body);
        let bodies = vec![body];

        let ke0 = kinetic(world, &bodies);
        let mut prev = ke0;
        for _ in 0..120 {
            world_step(world, DT);
            let ke = kinetic(world, &bodies);
            assert!(
                ke <= prev * 1.000_001 + 1e-12,
                "有阻尼时动能不得增加：{prev:.6} → {ke:.6}"
            );
            prev = ke;
        }
        assert!(
            prev < ke0 * 0.5,
            "阻尼 2.0 下 2 秒后动能应大幅下降：{ke0:.6} → {prev:.6}"
        );

        world_destroy(world);
    }

    // ── 7. 零重力静止 ───────────────────────────────────────────────────────

    /// 零重力、零初速、无接触的物体必须**完全不动**。
    ///
    /// 这条看着平凡，但它能抓到"求解器引入了虚假速度"（比如接触缓存残留、
    /// 休眠唤醒误触发）—— 那是 `sleep_wake` 那批测试之外的另一类漂移。
    #[test]
    fn bodies_at_rest_stay_at_rest_in_zero_gravity() {
        let world = make_zero_g_world();
        let bodies = vec![
            spawn(
                world,
                v(0.0, 0.0, 0.0),
                v(0.0, 0.0, 0.0),
                v(0.0, 0.0, 0.0),
                0.5,
                0.0,
                0.0,
            ),
            spawn(
                world,
                v(20.0, 0.0, 0.0),
                v(0.0, 0.0, 0.0),
                v(0.0, 0.0, 0.0),
                0.5,
                0.0,
                0.0,
            ),
        ];
        let p0: Vec<Vec3> = bodies
            .iter()
            .map(|&b| rigid_body_get_translation(world, b))
            .collect();

        for _ in 0..120 {
            world_step(world, DT);
        }

        for (i, &b) in bodies.iter().enumerate() {
            let p = rigid_body_get_translation(world, b);
            let d = ((p.x - p0[i].x).powi(2) + (p.y - p0[i].y).powi(2) + (p.z - p0[i].z).powi(2))
                .sqrt();
            assert!(d < 1e-9, "静止物体 #{i} 漂移了 {d:.3e} m");
            let vel = rigid_body_get_linvel(world, b);
            let speed = (vel.x * vel.x + vel.y * vel.y + vel.z * vel.z).sqrt();
            assert!(speed < 1e-9, "静止物体 #{i} 获得了速度 {speed:.3e} m/s");
            // 角速度也必须是零（没有外力矩）。
            let w = rigid_body_get_angvel(world, b);
            assert!(w.x.abs() + w.y.abs() + w.z.abs() < 1e-9);
        }

        world_destroy(world);
    }
    // ── 8. SPH 密度的解析对照 ───────────────────────────────────────────────

    /// Poly6 核的**解析式**（与实现独立，用于对照）：
    ///
    /// ```text
    /// W(r, h) = 315/(64πh⁹) · (h² − r²)³     r < h
    ///           0                             r >= h
    /// ```
    ///
    /// 归一化 `∫W dV = 1`。
    fn poly6_analytic(r: f64, h: f64) -> f64 {
        if r >= h {
            return 0.0;
        }
        let h2 = h * h;
        let diff = h2 - r * r;
        315.0 / (64.0 * std::f64::consts::PI * h.powi(9)) * diff.powi(3)
    }

    fn sph(position: Vec3, mass: f64) -> mps_core::rapier::ffi::SphParticle {
        mps_core::rapier::ffi::SphParticle {
            position,
            velocity: v(0.0, 0.0, 0.0),
            mass,
            density: 0.0,
            pressure: 0.0,
        }
    }

    /// 简单立方晶格上的 SPH 密度必须等于**解析值** `m·[W(0) + 6·W(d)]`。
    ///
    /// ## 为什么这条能抓到「自贡献重复计算」
    ///
    /// ROADMAP 举的第一个历史缺陷就是 SPH 自贡献重复计算。它不崩溃、不出 NaN，
    /// 只是密度**偏大**一个 `W(0)` 的量。
    ///
    /// 这里取 `h = 1.2·d`，于是只有 `r = 0`（自身）与 `r = d`（6 个面心邻居）
    /// 落在核半径内，第二近邻在 `d√2 ≈ 1.414d > h` 之外。**每一项都可解析算出**：
    ///
    /// ```text
    /// ρ = m·[ W(0,h) + 6·W(d,h) ]
    /// ```
    ///
    /// 自身多算一次 → 相对偏差 `W(0)/(W(0)+6W(d)) ≈ 85%`，绝对抓得到。
    #[test]
    fn sph_lattice_density_matches_the_analytic_sum() {
        let d = 1.0;
        let h = 1.2; // d < h < d·√2，只含自身与 6 个面心邻居
        let m = 1.0;

        let mut particles = vec![sph(v(0.0, 0.0, 0.0), m)];
        for axis in 0..3 {
            for sign in [-1.0, 1.0] {
                let mut p = v(0.0, 0.0, 0.0);
                match axis {
                    0 => p.x = sign * d,
                    1 => p.y = sign * d,
                    _ => p.z = sign * d,
                }
                particles.push(sph(p, m));
            }
        }
        // 一个远在核半径外的粒子：必须对密度零贡献（紧支撑）。
        particles.push(sph(v(10.0, 0.0, 0.0), m));

        let mut density = 0.0;
        let ok = mps_core::rapier::fluid::fluid_sph_estimate_density(
            v(0.0, 0.0, 0.0),
            particles.as_ptr(),
            particles.len() as u32,
            h,
            &mut density,
        );
        assert_eq!(ok, Bool::TRUE);

        let expected = m * (poly6_analytic(0.0, h) + 6.0 * poly6_analytic(d, h));
        assert!(
            rel_err(expected, density) < 1e-12,
            "SPH 密度应为解析值 {expected:.10}，实际 {density:.10}（相对偏差 {:.3e}）",
            rel_err(expected, density)
        );
    }

    /// 密度对质量**线性**：所有质量翻倍，密度必须翻倍。
    ///
    /// 这条抓「质量被平方」「漏乘质量」这类缺陷 —— 它们能通过上面的绝对值对照
    /// （如果碰巧只在一个质量下测过），但过不了线性性。
    #[test]
    fn sph_density_is_linear_in_mass() {
        let d = 1.0;
        let h = 1.2;
        let make = |m: f64| {
            let mut ps = vec![sph(v(0.0, 0.0, 0.0), m)];
            for axis in 0..3 {
                for sign in [-1.0, 1.0] {
                    let mut p = v(0.0, 0.0, 0.0);
                    match axis {
                        0 => p.x = sign * d,
                        1 => p.y = sign * d,
                        _ => p.z = sign * d,
                    }
                    ps.push(sph(p, m));
                }
            }
            ps
        };
        let eval = |ps: &[mps_core::rapier::ffi::SphParticle]| {
            let mut out = 0.0;
            assert_eq!(
                mps_core::rapier::fluid::fluid_sph_estimate_density(
                    v(0.0, 0.0, 0.0),
                    ps.as_ptr(),
                    ps.len() as u32,
                    h,
                    &mut out,
                ),
                Bool::TRUE
            );
            out
        };

        let one = eval(&make(1.0));
        let two = eval(&make(2.0));
        let half = eval(&make(0.5));
        assert!(rel_err(two, 2.0 * one) < 1e-12, "质量翻倍密度未翻倍");
        assert!(rel_err(half, 0.5 * one) < 1e-12, "质量减半密度未减半");
    }

    /// 核半径外的粒子必须**零贡献**（紧支撑）。
    ///
    /// 这条抓「忘了判断 `r >= h` 就直接算」—— 那会让 `(h²−r²)³` 变成负数或
    /// 错误的大值。
    #[test]
    fn sph_density_has_compact_support() {
        let h = 1.2;
        let near = vec![sph(v(0.0, 0.0, 0.0), 1.0)];
        let mut far = near.clone();
        far.push(sph(v(2.0 * h, 0.0, 0.0), 1.0)); // 2h 之外

        let eval = |ps: &[mps_core::rapier::ffi::SphParticle]| {
            let mut out = 0.0;
            assert_eq!(
                mps_core::rapier::fluid::fluid_sph_estimate_density(
                    v(0.0, 0.0, 0.0),
                    ps.as_ptr(),
                    ps.len() as u32,
                    h,
                    &mut out,
                ),
                Bool::TRUE
            );
            out
        };
        assert_eq!(eval(&near), eval(&far), "核半径外的粒子不得贡献密度");
    }
}
