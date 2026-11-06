# rapier/rope_knot.rs

## 作用
绳结/编织系统。在 `rope.rs` 之上的组合层：每股（strand）一个链式软体，粒子间用 fork 的 XPBD 距离约束（`add_distance_constraint`，柔度 = 1/刚度）连接，`SoftSolver::Xpbd` 求解。碰撞走 Phase 5f 内建机制——`build` 完成后逐股调用 `soft_body_enable_collision` 为每个自由粒子生成代理刚体 + 球碰撞体：**同股粒子互不碰撞**（连接约束本就相连），**不同股之间以及与地形会碰撞**（摩擦系数用调用方给的 rope-on-rope 摩擦），因此多股辫可以真实地互相绞结。编织模式：0=Overhand（单股打结）、1=FigureEight、2=SquareBraid、3=RoundBraid（每股一条绕轴线的螺旋线，相位错开）、4=Custom（控制点直接给几何）。粒子质量由密度×半径²×段长推出，代理质量与之相等。

## 关键导出
- `pub extern "C" fn rope_knot_create(world, pattern, strand_count, control_points*, control_point_count, radius, stiffness, self_friction, density)` — 登记配置，返回稳定 id（custom 必须给控制点，≤ `MAX_KNOT_POINTS = 256`）。
- `pub extern "C" fn rope_knot_build(world, id, start, end)` — 生成几何并建软体 + 碰撞代理；重复 build 报 `ERR_UNSUPPORTED`；start==end 退化跨度报 `ERR_INVALID_ARGUMENT`。
- `pub extern "C" fn rope_knot_set_wind(world, id, wind)` — 风场推送到已建各股（未建时缓存，build 时应用）。
- `pub extern "C" fn rope_knot_strand_soft_body(world, id, strand_index) -> u32` — 查询股的 `SoftBodyId.0`。
- `pub extern "C" fn rope_knot_remove(world, id)` — 先拆各股碰撞代理（镜像 enable_collision 的 teardown 路径）再删软体。
- 内部：`RopeKnotSystem`（世界内 `rope_knots` 哈希表）、`MAX_KNOT_STRANDS = 16`、`KNOT_XPBD_ITERATIONS = 8`、`FALLBACK_COMPLIANCE = 1e-4`、`BRAID_TURNS = 4`、`BRAID_STRAND_POINTS = 20`。

## 依赖
- `crate::rapier::soft_body::soft_body_enable_collision` — 复用其代理构建与碰撞组逻辑。
- fork `rapier3d::prelude::soft_body_fork::{SoftBody, SoftBodyId, SoftSolver}` — `Xpbd { iterations, compliance }` 变体、`add_particle`/`add_distance_constraint`/`apply_wind`。
- `crate::rapier::ffi` — `Bool`、`Vec3`、`WorldHandle`、`vec3_finite`/`vec3_to_rapier`；`soft_body_proxies` teardown 所需的世界字段。
- `crate::rapier::error` — 错误码与 `ffi_guard`。

## 测试
`mps-test/src/rapier/rope_knot.rs`：四种预置模式 + custom（含控制点坐标逐一校验）建/拆、辫子每股一个软体（strand 数校验）、非法输入（未知模式/股数越界/四组参数校验/无控制点/退化与非有限跨度）、重复 build、风场 + 多步后粒子保持有限、null world。
