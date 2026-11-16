# 0014 — fork 的力容器模型：持久力不必每帧重加

## 背景

rapier 原生的力模型是「扁平」的：用户通过 `add_force` / `add_torque` 把力累加进
`RigidBodyForces::user_force`，然后每步结束由 `reset_forces` 清空。这意味着
**任何持续作用的力都必须每帧重新 `add_force`**——推力、磁力锚点、风场，全部如此。

而重力不是这样：重力每步都被重新施加，从不清空。

于是同一份代码里存在两种力生命周期，却只有一个 API。宿主的典型写法是一段
"先清后加"的仪式：

```rust
// 每帧
body.reset_forces(true);
body.add_force(thrust, true);
```

漏掉一次 `reset_forces` 力就翻倍；漏掉一次 `add_force` 力就消失。这不是物理问题，
是 API 形状问题。

## 决定

引入 `force_containers` 模块，把力按 **kind** 分类，每个 kind 一个
`KindContainer`，容器自己记录 `Persistence`：

| 概念 | 含义 |
| --- | --- |
| `ForceKind` | 力的来源分类（Gravity / Thrust / Magnetic / Wind / Friction / ContactReaction / OneShot / User …） |
| `Persistence::Persistent` | 跨步存活（重力、稳定推力） |
| `Persistence::Transient` | 每步排空（事件、摩擦、接触反力） |
| `KindContainer` | 持有该 kind 的条目 + 自己的 persistence 标志 |

`compute_body_effective_forces` 每步把「原生 user slot + 重力容器 + 所有 body 容器」
求和写进 `forces.force` / `forces.torque`；`drain_transient_forces` 在步末排空
transient 容器。

## 三个必须记住的实现约束

**1. 原生 `add_force` 语义完全没变。** 核实过：`add_force` 仍是
`self.forces.user_force += force`，`reset_forces` 仍只清 `user_force`。
这个改动是**纯增量**，不是替换。老代码不受影响。

**2. `forces.force` 的写入点现在是唯一的。** 全仓只有
`force_containers.rs:318` 写 `rb.forces.force`（另一处是 `quarantine.rs` 清零）。
所以**任何想直接改 `forces.force` 的代码都会在下一步被覆盖**。改力必须走容器
或原生 `add_force`。这是最容易踩的坑。

**3. 求解器涌现的 kind 是只读观测，绝不参与求和。**
`Friction` 与 `ContactReaction` 由 `bridge_solver_contact_forces` 写入，但
`compute_body_effective_forces` 里被显式 `continue` 跳过——接触求解器已经施加了
这些冲量，再求和一次就是**双重施力**，会直接让求解器失稳。注释里写明了这一点。

**4. transient 在步级排空，不是子步级。** `solve.rs` 的遍历每个 CCD 子步都跑一次
`compute_body_effective_forces`，但 drain 只在 `substep.rs` 的步末做一次。
若在子步里 drain，一次性事件力会在第一个子步后消失。

## 后果

- 宿主不再需要"每帧重加"仪式：`add_thrust(..., Persistence::Persistent)` 一次，
  `remove_force_by_kind` 移除。
- 重力被搬进容器，与其它持久力走同一条路径——这是行为上的统一，不是行为改变
  （重力本来就是每步重加）。
- `force_containers` 外部引用 55 处，是 fork 里影响面最大的单个改动。
- 有确定性测试 `crates/rapier3d-f64/tests/force_container_determinism.rs`。

## 与官方的关系

官方 v0.36.1 **没有**等价能力（`reset_forces` 语义未变）。这是真实扩展，
符合 ADR-0008 的补丁纪律。

**建议**：这是通用能力，不依赖 `mps` 任何假设，值得提 PR 给上游。
如果上游接受，就能从 fork 里删掉。

## 相关代码

- `rapier/src/dynamics/force_containers.rs`（303 行起）
- `rapier/src/dynamics/rigid_body.rs`（`force_container*` / `add_thrust` / `remove_force_by_kind`）
- `rapier/src/pipeline/physics_pipeline/solve.rs`（两处 `compute_body_effective_forces`）
- `rapier/src/pipeline/physics_pipeline/substep.rs`（`drain_transient_forces`）
- `rapier/crates/rapier3d-f64/tests/force_container_determinism.rs`

## 记录来源

本 ADR 是对历史提交 `894d34e3`（2026-08-22，600 行）的**事后补记**。
该提交消息只有日期，没有说明意图；代码本身有完整文档头，但从未形成 ADR。
按 ADR-0008 的补丁纪律，改核心行为应有 ADR —— 这是一笔欠账，现在补上。
