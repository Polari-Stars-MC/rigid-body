# soft_upstream.rs — 把软体授权数据镜像到 fork 的 `SoftBody` 上求解

`crates/mps-core/src/rapier/soft_upstream.rs`。

mps 侧的软体**授权**（`add_particle` / `add_spring` / `add_triangle` / `add_tetrahedron`，以及粒子/能量/体积/应力查询）和 **求解** 被分开：求解只有一份实现——官方 XPBD 元件 + 可变形碰撞体。这个文件负责两边对接，89 个 `soft_body_*` C ABI 签名一个都不变。

## 每步的两次同步

```text
prepare():  拓扑变了就重建官方体 → 位姿/速度推过去 → 扩展项写进 fork_extras → 每体重力
pipeline.step(...)                → 官方求解（弹性/体积/接触/CCD）
collect():  官方体的解写回授权数据   → C ABI 的读接口拿到的就是本步结果
```

关键点：`collect()` 之后，`soft_body_read_*` 那批入口读的是**官方求解器的输出**，不是 mps 自己再算一遍。拓扑没变时 `prepare()` 不重建，只同步状态。

## 后端选择

`SoftBodyBackend` 有 `Fork`（自研积分器，历史路径）与 `Upstream`（官方求解，默认）两档，C ABI 上的模式值是 `0` / `1`。默认就是 `Upstream`；`Fork` 留着做对照与回退，不是推荐路径。

## 为什么这层存在

直接让上层调 fork 的 `SoftBodyBuilder` 会把 89 个已发布签名的语义绑死在 fork 的内部结构上——fork 一同步（ADR-0008）就会漏改。镜像层让签名留在 mps 侧，fork 只负责求解。

## 测试

`crates/mps-test/src/rapier/soft_upstream.rs`、`soft_body.rs`（主套件）、`softbody.rs`（早期骨骼链对照）。
