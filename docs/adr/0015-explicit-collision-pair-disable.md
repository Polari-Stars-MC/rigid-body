# 0015 — fork 的显式禁用碰撞对

## 背景

rapier 原生有三种方式让两个碰撞体不碰撞：

1. **collision groups**（位掩码）
2. **solver groups**（只影响求解器，不影响检测）
3. **关节的 `contacts_enabled`**（关节两端）

三者都是**规则式**的：你得让规则推导出"这两个不碰"。但宿主常常需要**逐对**
控制——"这一对，就这一对，永远不碰"。用 collision groups 表达这个需求，要么
把两个碰撞体挪进专属的组（污染全局分组空间），要么根本做不到。

典型场景：一个实体由多个碰撞体组成但内部不互碰；两个特定物体需要"幽灵化"
互相穿透但都与世界碰撞。

## 决定

在 narrow phase 加一张 `disabled_collider_pairs: HashMap<ColliderPair, ()>`，
并提供三个入口（`PhysicsWorld` 上是 `set_collision_enabled` /
`is_collision_enabled`，narrow phase 上是 `disable_collision` /
`enable_collision` / `is_collision_enabled`）。

语义：

- **无视** collision groups、solver hooks、是否被关节连接
- **对称**：`set(a, b, false)` 等价于 `set(b, a, false)`
- **跨 step 持久**：设置一次一直有效，不需要每帧重设
- 禁用时**清掉已存在的 manifold 与 intersecting 标志**

## 三个实现上不显然的点

**1. 必须每步清扫，不能只在 pair 更新时检查。**
pair-update 只重访「碰到被修改的碰撞体或活动刚体」的对。如果两个已静止的碰撞体
之间禁用了一对，它们的陈旧 manifold 会**永远留着**。所以 `pair_management.rs`
每步扫两张图（contact / intersection）清掉被禁用的对。代价很小——只遍历已存在的边。

**2. 拆除语义要对齐 `remove_pair`，但边要保留。**
清 solver color、唤醒 partner、发 `Stopped` 事件、断持久岛链接——这些都和
"这对不再接触"一致。但**边本身保留**，所以重新启用不需要走一遍 broad-phase。

**3. 移除碰撞体时必须清理引用它的禁用项。**
`pair_management.rs:43` 的 `retain` 做这件事。不清理的话，碰撞体句柄的 generation
永不重复，那些条目会变成永久死重，而且会让每步的清扫循环一直跑下去。

## 后果

- 宿主获得逐对控制能力，不必污染 collision groups。
- 有测试 `disable_collision_between_specific_colliders`（`narrow_phase/test.rs:296`）。
- 每步多一次"遍历已存在的边"的开销；禁用表为空时整段被
  `if !self.disabled_collider_pairs.is_empty()` 跳过，所以未使用的场景零代价。

## 与官方的关系

官方 v0.36.1 **没有**等价能力。这是真实扩展，符合 ADR-0008 的补丁纪律。

**提 PR 的障碍**：这个改动和另外 7 个模块的横向清理混在同一个提交
`33292dc3`（979 行 / 36 文件）里，没法干净地拆出单一功能的 patch。
要提 PR 得先做一次拆分。

## 相关代码

- `rapier/src/geometry/narrow_phase/mod.rs`（`disabled_collider_pairs` + 三个方法）
- `rapier/src/geometry/narrow_phase/pair_management.rs`（每步清扫 + 移除时 retain）
- `rapier/src/geometry/narrow_phase/contacts.rs` / `intersections.rs`（消费方）
- `rapier/src/pipeline/physics_world.rs`（`set_collision_enabled` / `is_collision_enabled`）
- `rapier/src/geometry/narrow_phase/test.rs`（回归测试）

## 记录来源

本 ADR 是对历史提交 `33292dc3`（2026-11-07，979 行）与 `02f6beaf`（34 行，
`PhysicsWorld` 入口）的**事后补记**。这两个提交的消息分别是纯日期和
"Update physics_world.rs"，从未形成 ADR。按 ADR-0008 的补丁纪律，
改核心行为应有 ADR —— 这是一笔欠账，现在补上。

同时记录一笔技术债：`33292dc3` 把「禁用碰撞对」与 7 个模块的顺手改混在一起，
这既解释了它消息无信息量的原因，也是将来提 PR 时需要先拆分的原因。
