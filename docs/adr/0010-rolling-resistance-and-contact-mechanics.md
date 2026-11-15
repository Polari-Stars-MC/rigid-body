# 0010 — 刚体接触的滚动阻力进 fork 求解器，接触力学与凸分解进公式层

## 背景

2026-11-13 的公式/物理/碰撞清点（对着仓库实证逐项核对）发现三处真实缺口，其中两处属于「文档与实现不符」或「能力完全缺失」：

1. **滚动阻力不存在。** `rolling_friction` 全仓只出现在
   `rapier/src/control/ray_cast_vehicle_controller.rs`，是车辆轮子专用的特例；
   `parry`/`rapier` 的刚体接触求解器里没有任何对应自由度。后果是球体、圆柱体
   在平面上**永远滚下去**——库仑摩擦约束的是接触点滑移，而纯滚动时接触点滑移
   为零，所以摩擦力对滚动的减速度贡献恒为零。
   需要先确认 `FrictionModel::Simplified`（默认）里的 twist 约束是否已覆盖：
   读 `contact_with_twist_friction.rs` 后确认它约束的是**绕法向**的旋转（自旋），
   滚动是绕**切平面内轴**的旋转，两者是不同的自由度，所以确实缺失。
2. **`granular.rs` 的注释与实现不符。** 文件头写 "Hertzian-style"，实现是线性
   弹簧 `F = k_n·δ`。线性弹簧是合法的 DEM 选择，但它不是 Hertz：没有
   `δ^{3/2}` 标度，也没有接触半径与压力分布的概念。任何按材料杨氏模量调 `k`
   的人都得不到物理上可对的值。
3. **凹形网格无法直接当动态碰撞体。** 形状族完整（凸原语、凸包、三角网格、
   高度场、圆角、复合），但 `convex_decomposition|vhacd|decompose` 全仓 0 处。
   宿主拿到一个凹模型只能手工拆成复合体，而分解质量直接决定碰撞质量。

顺带更正清点中的一个误判：**静摩擦/动摩擦已经实现**，通过
`events.rs` 的 `modify_solver_contacts` 钩子按切向速度阈值在
`CoulombFrictionLaw::static_coefficient` 与 `dynamic_coefficient` 之间切换。
清点初稿曾把它列为缺失，读代码后撤回。

## 决定

1. **滚动阻力进 fork 求解器**（而不是在 `mps-core` 里 step 后补阻尼力）。
   理由是后者拿不到逐接触对的法向冲量，只能按全局系数衰减角速度，与"阻力正比
   于法向载荷"的物理定义不符，且会在无接触时也衰减。
   实现沿用 twist 约束的形状：
   - `ColliderMaterial` 加 `rolling_resistance` + 合并规则（默认 `0.0`）；
   - `ContactManifoldData` 加 `rolling_resistance` 与 `rolling_arm`；
   - `ContactConstraintRollingPartSlim` 两个标量约束（每个切向轴一个），
     预算 `μ_r · Σ N · r`，`rhs = 0`（纯耗散，不注入能量）；
   - `rolling_arm` 由 narrow phase 从两个碰撞体包围半径的较小者算出，
     **不能**用接触点到摩擦中心的距离：球在平面上是单点流形，该距离恒为 0。
2. **Hertz + Mindlin 进 `mps-formula`**（`contact_mechanics.rs`），纯函数、
   不进求解器。理由：公式层零 ABI 风险，且它同时给出「把现有线性弹簧调成
   物理值」的换算函数 `hertz_linearized_stiffness`，让 fork 的 DEM 路径不必
   改动就能被正确标定。
3. **凸分解进 `mps-formula`**（`convex_decomposition.rs`），递归平面切分，
   输出可直接喂 `collider_builder_create_convex_hull` 的点集。
   刻意**不做**体素化 V-HACD：体素法需要水密且尺度良好的网格与稠密三维栅格，
   而本项目的世界是方块/台阶/阶梯形状，平面切分对它们是精确的（盒子切成两个
   盒子），对有机网格则优雅退化。

## 后果

- **`μ_r = 0` 逐位不变。** 预算是 `μ_r · Σ N · r`，系数为 0 时 clamp 退化为
  `[-0, 0]`，约束写入为零；测试 `rolling_resistance_stops_a_rolling_ball`
  断言 `μ_r = 0` 时 `|ωz|` 与初值相差 < 1e-9（实测逐位相同）。
- 滚动阻力是**耗散**项，`rhs` 恒为 0，不可能给系统注入能量。
- 单点流形也生效（不 gate 在 `num_contacts > 1`），这是与 twist 约束的关键区别：
  球在平面上只有一个接触点，而那正是最需要滚动阻力的情形。
- `mps-formula` 的 `contact_mechanics` 覆盖 Hertz 法向、Mindlin 切向
  （含部分滑移的库仑封顶）、有效模量/半径、以及 Hertz + Hunt-Crossley 耗散。
  钢球压钢板的量级抽查写进了测试（接触半径 10 µm–1 mm、峰值压力 0.1–10 GPa）。
- 凸分解不保证是精确分割：相邻部件只保证共享切分平面上的顶点（浮点意义下），
  不做网格修复、不做自交消解。
- 三处新代码都遵守既有边界：公式层无 `WorldHandle`、无 Rapier；fork 改动
  按 ADR-0008 的补丁纪律，是「修 rapier 自身缺失能力」而不是服务上层的 hack。

## 相关代码

- `rapier/src/geometry/collider_components.rs`（`ColliderMaterial::rolling_resistance`）
- `rapier/src/geometry/collider.rs`（`Collider`/`ColliderBuilder` 访问器）
- `rapier/src/geometry/contact_pair.rs`（`ContactManifoldData::rolling_resistance` / `rolling_arm`）
- `rapier/src/geometry/narrow_phase/pair_update.rs`（系数合并 + 杠杆臂计算）
- `rapier/src/dynamics/solver/contact_constraint/contact_constraint_element.rs`
  （`ContactConstraintRollingPartSlim`）
- `rapier/src/dynamics/solver/contact_constraint/contact_with_twist_friction.rs`
  （generate / warmstart / solve / update 接线）
- `crates/mps-core/src/rapier/collider.rs`
  （`collider_set_rolling_resistance` / `_flag` / `collider_get_rolling_resistance`）
- `crates/mps-formula/src/contact_mechanics.rs`（Hertz / Mindlin / Hunt-Crossley）
- `crates/mps-formula/src/convex_decomposition.rs`（平面切分凸分解）
- `crates/mps-test/src/rapier/{collider,contact_mechanics,convex_decomposition}.rs`
