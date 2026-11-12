# 真实物理现象验证（本轮四路并行）

> 采集日期：2026-11-12。四个方向各由一个专项测试文件钉死，
> 表格数字由测试 `--nocapture` 输出转抄。

## 1. 大气阻力终端速度（drag_terminal_velocity）

> 复现：`cargo test -p mps-test --lib -- drag_terminal_velocity --nocapture`（3 passed, 0.11s）

物理：球体等密度层内终端速度 v_t = √(2mg/(ρ·C_d·A))；引擎为**平方律**
阻力 a = −½ρ|v|·C_d·A/m·v，衰减律钉的是 1/|v| 线性（斜率 k =
½ρC_d·A/m），而非线性阻力的 ln|v| 线性。

| 测试 | 期望 | 实测 | 偏差 |
| --- | --- | --- | --- |
| `terminal_velocity_matches_quadratic_drag_formula`（100 kg、C_d=2.2、A=1 m²，1000 m 落 15 s） | 当地 v_t = 27.9844 m/s（ρ=1.137209） | 稳态 28.0487 m/s @ 632.1 m | **0.23%**（容差 10%） |
| `drag_only_never_accelerates_along_velocity`（v₀=300，等密度） | 速率每步严格单调降、横向 < 1e-9 | 6 s 后 11.8660 m/s，全程单调 | ✓ |
| `speed_decay_follows_quadratic_law_inverse_v_linear_in_t`（v₀=100 跑 8 s） | 拟合斜率 = k = 0.013475 1/m；终速 8.4890 | 斜率 0.013482；终速 8.4840 | 0.05% / 0.06% |

无产品缺陷——密度模型、方向语义、平方律系数与
`mps_formula::spaceflight::atmospheric_drag_acceleration` 文档一致。

## 2. 摩擦与恢复系数能量学（friction_restitution_energy）

> 复现：`cargo test -p mps-test --lib -- friction_restitution_energy --nocapture`（4 passed）

| 测试 | 物理期望 | 实测 |
| --- | --- | --- |
| `friction_cone_incline_static_below_critical_angle`（μ=0.4，θ=10.90° < arctan μ=21.80°） | 滑块零位移 | drift = 0.000000 m ✓ |
| `friction_cone_incline_slides_at_predicted_acceleration`（θ=32.70°） | a = g(sinθ−μcosθ) = 1.9981 m/s² | measured 1.9981 m/s²，**rel_err = 0.000** |
| `restitution_bounce_speed_ratio_matches_e`（e=0.6，Max combine 双侧） | 反弹/撞击速度比 ≈ e | 3.9649 → 2.3380，ratio = **0.5897**（e=0.6，-1.7%） |
| `static_friction_holds_below_threshold_then_slides`（阈值 10.423 N） | F<阈值零漂移，F>阈值滑动 | 3.127 N → drift 0.000639 m（<1mm）；31.269 N → 58.65 m，v=39.08 m/s |

库仑锥、稳态加速度、恢复系数能量比、静摩擦阈值全部与解析解吻合。

## 3. Hohmann 转移闭环（hohmann_transfer_validation）

> 复现：`cargo test -p mps-test --lib -- hohmann_transfer_validation --nocapture`（3 passed）

LEO(6678 km) → GEO(42164 km)，μ = 3.986004415e14，CosmosWorld
Yoshida4Kahan + arena 命令环注入（SetVelocity/ApplyImpulse）：

| 测试 | 期望 | 实测 |
| --- | --- | --- |
| `formula_matches_textbook_closed_form` | `hohmann_transfer_checked` 与闭式解一致 | Δv₁=2425.769027、Δv₂=1466.838715、Δv_total=3892.607742 m/s；T/2=18990.051846 s 与 π√(a_t³/μ) **to_bits 逐位相等** |
| `closed_loop_transfer_reaches_geo_radius`（注入 Δv₁ 积 T/2 再圆化） | 到达 r₂ < 1% | best \|r−r₂\|/r₂ = **6.346e-12**；真远地点 r=42163993.5 m（rel 1.55e-7）；圆化后 \|v\|=3074.666 m/s（GEO 圆速度）、v_r=−1.68 m/s |
| `transfer_ellipse_radius_matches_kepler_equation` | r(t) vs 开普勒方程 < 0.1% | 18990 采样点 max dev = **1.154e-12** |

语义取舍（有意，写在文件头）：`celestial_acceleration` 在 r<2R 走椭球、
r<100R 走 J2..J6；LEO≈1.047R 落椭球分支会引入 O(J2) 系统偏差，故测试用
`equatorial_radius=0` 退化天体走点质量分支做干净二体验证。**若产品要在
LEO 做 Hohmann 需考虑 J2 补偿**。API 缺口：CosmosWorld 无公开中途改速度
API（`set_linvel` 不暴露），闭环只能走共享 arena 命令环——宿主唯一路径。

## 4. 颗粒休止角（granular_repose）

> 复现：`cargo test -p mps-test --release --lib -- granular_repose --ignored --nocapture --test-threads=1`
> （DEM + 100 刚体代理 debug 构建过慢，两测试 `#[ignore]`）

N=100 粒子（r=0.05 m，mass=1.0 → ρ≈1909.9 kg/m³），落定 600 步 × dt=1/120 s：

| 测试 | 断言 | 实测 |
| --- | --- | --- |
| `pile_is_symmetric_and_conservative` | 粒子数守恒、质心水平漂移 < 0.1·r、无穿地 | drift = **0.004584 m**（限值 0.005，通过）；最低粒子中心 y = **0.832·r**（裕度 0.8·r，通过） |
| `angle_of_repose_increases_with_friction` | μ=0.05 与 μ=0.8 休止角单调递增 | **49.23° == 49.23°（逐位一致，Δ=0.000°）→ 断言失败 |

### 4.1 产品缺陷：granular friction 未传导到刚体代理 collider

- DEM 求解器本身 μ **生效**：`rapier/src/dynamics/granular.rs` 切向项为
  `min(tangential_damping·v_t, μ·|F_n|)`，Coulomb cap 正确接入。
- 缺陷在 `granular_enable_collision`（granular.rs:403）：代理 collider 用
  `ColliderBuilder::ball(radius).density(0.0)` 构建，**friction 硬编码
  Rapier 默认 0.5**，无 FFI 入口设置。开代理后 `world_step` 先用刚体接触
  解算粒子姿态再读回，堆形由 friction=0.5 的刚体接触主导——DEM μ 的
  差异被淹没，两组休止角逐位一致。49.23° 是 friction≈0.5 刚体接触的
  自然休止角（非几何锁定值：arctan(2R/spacing)≈44.4° 不符）。
- 修复建议：`granular_enable_collision` 加 friction/restitution 参数，
  或代理 collider 直接继承 `granular_create` 的 friction。
- 次要缺口：无 per-particle friction/density 入口（密度经 mass/radius
  间接表达）；无动态改参 API（每组工况须重建世界）。
- monotonic 测试以 `#[ignore = "product defect: …"]` 钉住缺陷现场，
  修复后去掉 ignore 即可复跑。
