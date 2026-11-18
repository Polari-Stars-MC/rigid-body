//! End-to-end tests for the Pacejka-style tire model layered on the ray-cast
//! vehicle controller.

#[cfg(test)]
mod tests {
    use mps_core::rapier::error::{
        ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, ERR_OK,
        last_error_code,
    };
    use mps_core::rapier::ffi::{Bool, ShapeDesc, ShapeType, Vec3, WorldHandle};
    use mps_core::rapier::tire_model::{
        tire_model_create, tire_model_get_forces, tire_model_remove, tire_model_set_params,
        tire_model_update,
    };
    use mps_core::rapier::vehicle::{
        vehicle_controller_add_wheel, vehicle_controller_create, vehicle_controller_set_brake,
        vehicle_controller_set_engine_force, vehicle_controller_update,
    };
    use mps_core::rapier::world::{world_create, world_destroy};

    fn v3(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    fn make_world() -> *mut WorldHandle {
        world_create(v3(0.0, -9.81, 0.0))
    }

    /// A chassis + 4 wheels (corners, suspension down -Y, axle along -Z),
    /// floating in free space — no floor, so wheels are airborne and the
    /// static-load fallback drives the tire model.
    fn make_car(world: *mut WorldHandle) -> u32 {
        let chassis = ShapeDesc {
            shape_type: ShapeType::Cuboid as u32,
            a: 1.0,
            b: 0.3,
            c: 2.0,
            ..Default::default()
        };
        let id = vehicle_controller_create(world, chassis, v3(0.0, 20.0, 0.0));
        assert_ne!(id, u32::MAX);
        for x in [-0.9f64, 0.9] {
            for z in [-1.5f64, 1.5] {
                let idx = vehicle_controller_add_wheel(
                    world,
                    id,
                    v3(x, 0.0, z),
                    v3(0.0, -1.0, 0.0),
                    v3(0.0, 0.0, -1.0),
                    0.3,    // suspension rest length
                    0.4,    // wheel radius
                    24.0,   // suspension stiffness
                    0.8,    // damping compression
                    0.8,    // damping relaxation
                    1.5,    // friction slip
                    0.3,    // max suspension travel
                    6000.0, // max suspension force
                    0.5,    // side friction stiffness
                );
                assert_ne!(idx, u32::MAX);
            }
        }
        id
    }

    #[test]
    fn tire_model_create_requires_vehicle() {
        let world = make_world();
        assert_eq!(tire_model_create(world, 999, 4), u32::MAX);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn tire_model_create_validates_wheel_count() {
        let world = make_world();
        let vehicle = make_car(world);
        assert_eq!(tire_model_create(world, vehicle, 0), u32::MAX);
        assert_eq!(last_error_code(), ERR_CAPACITY);
        assert_eq!(tire_model_create(world, vehicle, 33), u32::MAX);
        assert_eq!(last_error_code(), ERR_CAPACITY);

        let id = tire_model_create(world, vehicle, 4);
        assert_ne!(id, u32::MAX);
        assert_eq!(last_error_code(), ERR_OK);
        assert_eq!(tire_model_remove(world, id), Bool::TRUE);
        assert_eq!(tire_model_remove(world, id), Bool::TRUE); // idempotent
        world_destroy(world);
    }

    #[test]
    fn tire_model_set_params_validates_input() {
        let world = make_world();
        let vehicle = make_car(world);
        let id = tire_model_create(world, vehicle, 4);
        assert_ne!(id, u32::MAX);

        assert_eq!(
            tire_model_set_params(world, id, 0, 1.4, 1.3, 0.12, 0.15, 0.85, 1.2),
            Bool::TRUE
        );
        // Wheel index out of range.
        assert_eq!(
            tire_model_set_params(world, id, 4, 1.4, 1.3, 0.12, 0.15, 0.85, 1.2),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        // Non-physical parameter (zero peak friction).
        assert_eq!(
            tire_model_set_params(world, id, 0, 0.0, 1.3, 0.12, 0.15, 0.85, 1.2),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        // Unknown tire model.
        assert_eq!(
            tire_model_set_params(world, 987654, 0, 1.4, 1.3, 0.12, 0.15, 0.85, 1.2),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn tire_model_update_reports_zero_forces_at_rest() {
        let world = make_world();
        let vehicle = make_car(world);
        let id = tire_model_create(world, vehicle, 4);
        assert_ne!(id, u32::MAX);

        assert_eq!(
            vehicle_controller_update(world, vehicle, 1.0 / 60.0),
            Bool::TRUE
        );
        assert_eq!(tire_model_update(world, id, 1.0 / 60.0), Bool::TRUE);

        let mut fx = 0.0;
        let mut fy = 0.0;
        for wheel in 0..4 {
            assert_eq!(
                tire_model_get_forces(world, id, wheel, &mut fx, &mut fy),
                Bool::TRUE
            );
            assert_eq!(fx, 0.0, "wheel {wheel} at rest must have no drive force");
            assert_eq!(fy, 0.0, "wheel {wheel} at rest must have no side force");
        }
        assert_eq!(last_error_code(), ERR_OK);
        world_destroy(world);
    }

    #[test]
    fn tire_model_engine_force_produces_longitudinal_force() {
        let world = make_world();
        let vehicle = make_car(world);
        let id = tire_model_create(world, vehicle, 4);
        assert_ne!(id, u32::MAX);

        assert_eq!(
            vehicle_controller_set_engine_force(world, vehicle, 0, 1000.0),
            Bool::TRUE
        );
        assert_eq!(
            vehicle_controller_update(world, vehicle, 1.0 / 60.0),
            Bool::TRUE
        );
        assert_eq!(tire_model_update(world, id, 1.0 / 60.0), Bool::TRUE);

        let mut fx = 0.0;
        let mut fy = 0.0;
        assert_eq!(
            tire_model_get_forces(world, id, 0, &mut fx, &mut fy),
            Bool::TRUE
        );
        assert!(
            fx > 0.0,
            "a driven wheel must produce positive longitudinal force, got {fx}"
        );
        // The force must saturate at the friction-ellipse cap, i.e. stay
        // bounded by peak μ · load^α · ellipse factor (well below 1e4 here).
        assert!(fx < 1.0e4, "force must stay bounded, got {fx}");
        world_destroy(world);
    }

    #[test]
    fn tire_model_brake_locks_wheels() {
        let world = make_world();
        let vehicle = make_car(world);
        let id = tire_model_create(world, vehicle, 4);
        assert_ne!(id, u32::MAX);

        // Spin the wheels up first.
        assert_eq!(
            vehicle_controller_set_engine_force(world, vehicle, 0, 2000.0),
            Bool::TRUE
        );
        assert_eq!(
            vehicle_controller_update(world, vehicle, 1.0 / 60.0),
            Bool::TRUE
        );
        assert_eq!(tire_model_update(world, id, 1.0 / 60.0), Bool::TRUE);

        // Full brake locks them: slip ratio becomes negative (wheel slower
        // than the road) and the longitudinal force opposes the motion.
        assert_eq!(
            vehicle_controller_set_brake(world, vehicle, 0, 1.0e9),
            Bool::TRUE
        );
        assert_eq!(
            vehicle_controller_update(world, vehicle, 1.0 / 60.0),
            Bool::TRUE
        );
        assert_eq!(tire_model_update(world, id, 1.0 / 60.0), Bool::TRUE);

        let mut fx = 0.0;
        let mut fy = 0.0;
        assert_eq!(
            tire_model_get_forces(world, id, 0, &mut fx, &mut fy),
            Bool::TRUE
        );
        assert!(fx <= 0.0, "a locked wheel must not drive, got {fx}");
        world_destroy(world);
    }

    #[test]
    fn tire_model_get_forces_validates_input() {
        let world = make_world();
        let vehicle = make_car(world);
        let id = tire_model_create(world, vehicle, 4);
        assert_ne!(id, u32::MAX);
        assert_eq!(tire_model_update(world, id, 1.0 / 60.0), Bool::TRUE);

        let mut fx = 0.0;
        let mut fy = 0.0;
        // Wheel index out of range.
        assert_eq!(
            tire_model_get_forces(world, id, 4, &mut fx, &mut fy),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        // Null out pointers.
        assert_eq!(
            tire_model_get_forces(world, id, 0, std::ptr::null_mut(), std::ptr::null_mut()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        // Invalid dt.
        assert_eq!(tire_model_update(world, id, 0.0), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        // Unknown tire model.
        assert_eq!(tire_model_update(world, 987654, 1.0 / 60.0), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn tire_model_null_world_is_rejected() {
        assert_eq!(tire_model_create(std::ptr::null_mut(), 0, 4), u32::MAX);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            tire_model_update(std::ptr::null_mut(), 0, 1.0 / 60.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }
    // ── 量纲/物理不变量（ADR-0038，ROADMAP 历史缺陷 #2）─────────────────────

    /// 跑 `steps` 步稳定驱动，返回指定轮最后的 `(fx, fy)`。
    ///
    /// **必须多步**：滑移比是**状态量**（由 `engine_force` 逐步积分），单步下
    /// 轮速还没养起来，力与引擎力几乎无关。这一点是写这组测试时才发现的
    /// —— 第一版用单步，断言全部落空。
    fn drive_steps(mu: f64, engine_force: f64, steps: u32) -> (f64, f64) {
        let dt = 1.0 / 60.0;
        let world = make_world();
        let vehicle = make_car(world);
        let id = tire_model_create(world, vehicle, 4);
        assert_ne!(id, u32::MAX);
        assert_eq!(
            tire_model_set_params(world, id, 0, mu, mu, 0.12, 0.15, 1.0, 1.0),
            Bool::TRUE
        );
        assert_eq!(
            vehicle_controller_set_engine_force(world, vehicle, 0, engine_force),
            Bool::TRUE
        );
        for _ in 0..steps {
            assert_eq!(vehicle_controller_update(world, vehicle, dt), Bool::TRUE);
            assert_eq!(tire_model_update(world, id, dt), Bool::TRUE);
        }
        let mut fx = 0.0;
        let mut fy = 0.0;
        assert_eq!(
            tire_model_get_forces(world, id, 0, &mut fx, &mut fy),
            Bool::TRUE
        );
        world_destroy(world);
        (fx, fy)
    }

    /// **量纲检查**：纵向力必须与 `peak_mu_long` **线性**成正比。
    ///
    /// ROADMAP 记录的历史缺陷是「轮胎力量纲错误」。量纲错通常表现为某个因子的
    /// 幂次不对 —— `μ²`、漏乘 `μ`、或把 `μ` 套进指数。
    ///
    /// 判据：`F(μ=1.4) / F(μ=0.7)` 必须精确等于 **2**。
    /// 用 `μ²` 会得 4；漏乘 `μ` 会得 1。
    ///
    /// 载荷因子与滑移在两档之间完全相同（同一场景、同一引擎力），所以唯一的
    /// 变量就是 `μ`。
    #[test]
    fn tire_longitudinal_force_scales_linearly_with_peak_mu() {
        let (f_low, _) = drive_steps(0.7, 3.0e4, 30);
        let (f_high, _) = drive_steps(1.4, 3.0e4, 30);
        assert!(f_low > 1e-6, "低 μ 下应有驱动力，实际 {f_low}");
        let ratio = f_high / f_low;
        assert!(
            (ratio - 2.0).abs() < 1e-9,
            "纵向力应对 peak_mu 线性：F(1.4)/F(0.7) = {ratio}，应为 2"
        );
    }

    /// **库仑上限**：`|Fx| ≤ μ · Fz_max`，`Fz_max = max_suspension_force = 6000 N`。
    ///
    /// 轮胎力不能超过「摩擦系数 × 最大法向载荷」—— 那是库仑摩擦的定义。
    /// 这条抓「忘记封顶」：不封顶的实现会让驱动力随引擎力无限增长。
    #[test]
    fn tire_force_stays_below_the_coulomb_cap() {
        const MAX_SUSPENSION_FORCE: f64 = 6000.0;
        for mu in [0.7, 1.0, 1.4] {
            let (fx, fy) = drive_steps(mu, 1.0e6, 30);
            let cap = mu * MAX_SUSPENSION_FORCE;
            assert!(
                fx.abs() <= cap * 1.000_001,
                "μ={mu}：|Fx| = {fx:.3} N 超过库仑上限 {cap:.3} N"
            );
            assert!(
                fy.abs() <= cap * 1.000_001,
                "μ={mu}：|Fy| = {fy:.3} N 超过库仑上限 {cap:.3} N"
            );
        }
    }

    /// **饱和**：引擎力 ×100，纵向力几乎不变。
    ///
    /// 这是 Magic Formula 与一切真实轮胎的标志性行为：进入摩擦饱和区后，
    /// 驱动力由 `μ` 和法向载荷决定，与发动机输出**无关**。
    ///
    /// 若实现没有封顶，力会与引擎力成正比 —— 比值接近 100。
    #[test]
    fn tire_force_saturates_and_becomes_independent_of_engine_force() {
        let (f_small, _) = drive_steps(1.0, 600.0, 60);
        let (f_big, _) = drive_steps(1.0, 60_000.0, 60);
        assert!(f_small > 1e-6, "小引擎力下应有驱动力，实际 {f_small}");

        // 饱和：两者接近（容许 20% 差异 —— 滑移仍在缓慢演化）。
        let ratio = f_big / f_small;
        assert!(
            (ratio - 1.0).abs() < 0.2,
            "引擎力 ×100 后力应几乎不变（饱和），实际比值 {ratio}（{f_small} → {f_big}）"
        );
    }

    /// **制动使力回落到零**（静止锁死的正确行为），且过程中**从不产生正力**。
    ///
    /// ## 这里验证的是一个特定（且容易误解）的物理场景
    ///
    /// 本文件的 `make_car` 把整车悬在**自由空间**（无地板、无水平力施加到车身），
    /// 所以车速恒为 0。此时：
    ///
    /// * 驱动 → 轮子在原地空转，`slip > 0`，产生正力
    /// * 满刹车 → 轮速归零，`slip = 0`，力回落到 **0**
    ///
    /// **这不是 bug**：真正的「滑移制动」（车比轮快，`slip < 0`）需要车身真的在
    /// 动，而本文件的测试脚手架不把轮胎力施加到车身（那是调用方的事，见模块文档）。
    ///
    /// 所以这条测的是：制动的**符号方向**正确（力不会变正）、且最终归零。
    /// **滑移制动区（`slip < 0`）未覆盖** —— 诚实记录。
    #[test]
    fn tire_braking_returns_force_to_zero_without_going_positive() {
        const MAX_SUSPENSION_FORCE: f64 = 6000.0;
        let mu = 1.0;
        let dt = 1.0 / 60.0;

        let world = make_world();
        let vehicle = make_car(world);
        let id = tire_model_create(world, vehicle, 4);
        assert_ne!(id, u32::MAX);
        assert_eq!(
            tire_model_set_params(world, id, 0, mu, mu, 0.12, 0.15, 1.0, 1.0),
            Bool::TRUE
        );

        let read_fx = || {
            let mut fx = 0.0;
            let mut fy = 0.0;
            assert_eq!(
                tire_model_get_forces(world, id, 0, &mut fx, &mut fy),
                Bool::TRUE
            );
            fx
        };

        // 驱动 30 步建立滑移。
        assert_eq!(
            vehicle_controller_set_engine_force(world, vehicle, 0, 3.0e4),
            Bool::TRUE
        );
        for _ in 0..30 {
            assert_eq!(vehicle_controller_update(world, vehicle, dt), Bool::TRUE);
            assert_eq!(tire_model_update(world, id, dt), Bool::TRUE);
        }
        let fx_drive = read_fx();
        assert!(fx_drive > 1e-6, "驱动段应产生正力，实际 {fx_drive}");

        // 满刹车，再跑 60 步。
        assert_eq!(
            vehicle_controller_set_engine_force(world, vehicle, 0, 0.0),
            Bool::TRUE
        );
        assert_eq!(
            vehicle_controller_set_brake(world, vehicle, 0, 1.0e9),
            Bool::TRUE
        );
        let cap = mu * MAX_SUSPENSION_FORCE;
        let mut last = f64::NAN;
        for step in 0..60 {
            assert_eq!(vehicle_controller_update(world, vehicle, dt), Bool::TRUE);
            assert_eq!(tire_model_update(world, id, dt), Bool::TRUE);
            last = read_fx();
            assert!(
                last <= 1e-9,
                "制动第 {step} 步出现了正力 {last} —— 制动绝不应驱动车轮"
            );
            assert!(last.abs() <= cap * 1.000_001, "制动力超过库仑上限");
        }

        // 静止锁死后力必须归零。
        assert!(
            last.abs() < 1e-6,
            "轮子锁死且车速为零时纵向力应归零，实际 {last}"
        );

        world_destroy(world);
    }
}
