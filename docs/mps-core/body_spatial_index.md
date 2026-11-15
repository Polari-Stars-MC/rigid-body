# body_spatial_index.rs — 刚体中心 BVH

`crates/mps-core/src/rapier/body_spatial_index.rs`。

`PhysicsWorld` 私有的 `BodySpatialIndex`：只用**刚体中心点**建 BVH，不看碰撞体形状。目的是让「这个区域里有哪些刚体」变成一次 BVH 遍历，而不是一次 `bodies.iter()`。

## 它不是什么

- **不是 Rapier 的碰撞体 BVH。** 碰撞检测仍然走 Rapier 自己的 broad phase，两套并存、互不替代。
- **不是外部可换的索引。** 字段是 `pub(crate)`，跟着 `PhysicsWorld` 生灭。
- **不保证和 `bodies` 集合自动同步。** 使用者不能换掉 `bodies` 集还指望索引仍然对得上——插入/删除必须走带索引维护的那几条路径。

## 维护点

- **步进后增量刷新**：`world_step` 末尾的力复位循环顺手把「本步位置变过」的叶子标脏并更新，稳态成本按变动体数算，不是全量重建。
- **删除/重插**：先解析出当前 arena 的 generation 再改叶子，避免把旧代句柄写进索引（见 [BOUNDARY.md](../BOUNDARY.md)「句柄与内存」）。

## 用法与边界

只服务区域策略查询（`world_*region*` 与按区域启用/冻结那一路）。单点/射线/形状查询走 `query.rs` 的 Rapier 路径。专题说明在 [body-spatial-index.md](../body-spatial-index.md)。

## 测试

`crates/mps-test/src/rapier/body_spatial_index.rs`，配合 `mps-test/src/rapier/world.rs` 的区域用例。
