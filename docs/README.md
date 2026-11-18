# docs

设计约束和证据。给人读的项目简要在仓库根 [README.md](../README.md)；给代理的操作清单在 [AGENTS.md](../AGENTS.md)。

代码注释里的 `DESIGN.md §N` / `OPTIMIZATION.md §N` 仍指向仓库根的两份章节索引。那两个文件的章节号是守门测试的锚，不搬进这里。

## 读哪一份

| 文档 | 回答的问题 |
| --- | --- |
| [ARCHITECTURE.md](ARCHITECTURE.md) | 系统现在长什么样，数据怎么过边界 |
| [BOUNDARY.md](BOUNDARY.md) | 什么不许跨、什么必须成对修改 |
| [ROADMAP.md](ROADMAP.md) | 现在做什么、明确不做哪些 |
| [CHECKPOINT_RECOVERY.md](CHECKPOINT_RECOVERY.md) | 长任务断了之后从哪接着查 |
| [adr/](adr/README.md) | 已经做出的决定，以及为什么 |
| [evidence/](evidence/README.md) | 哪些行为被测试或基准钉住 |
| [mps-core.md](mps-core.md) | `mps-core` 每个源文件做什么 |
| [host-integration.md](host-integration.md) | 宿主从哪开始：1045 个 ABI 里 90% 场景只需哪 20 个 |
| [CAPABILITY_MAP.md](CAPABILITY_MAP.md) | 改一个 C ABI 要动哪几处（生成物，别手改） |
| [integration/mc-host.md](integration/mc-host.md) | 宿主（Java 侧）怎么接线、JNI 怎么声明 |

## 专题（实现契约，不是路线）

| 文档 | 内容 |
| --- | --- |
| [formula-numeric-domains.md](formula-numeric-domains.md) | 纯公式的数值域：`FormulaError`，不写 FFI 错误槽 |
| [world-collision-mode.md](world-collision-mode.md) | 默认碰撞体策略（None / Simple / Compound / Adaptive） |
| [body-spatial-index.md](body-spatial-index.md) | 刚体中心 BVH：区域查询，不替代碰撞体 BVH |
| [world-step-benchmarks.md](world-step-benchmarks.md) | `world_step` 性能矩阵怎么跑、CSV 是什么 |
| [cosmos-arena.md](cosmos-arena.md) | cosmos 共享竞技场的偏移表、槽位和上限（跨语言 ABI） |
| [tyndall-mc-integration.md](tyndall-mc-integration.md) | 丁达尔效应：MC 侧集成指南（23 个 C ABI 怎么用） |
| [abi-merge-scan.md](abi-merge-scan.md) | dedicated 入口的可合并性扫描（确切数字与修正后的判据） |
| [abi-convergence.md](abi-convergence.md) | 949 个 C ABI 怎么收敛成通用属性通道（设计稿） |
| [rapier-fork-delta.md](rapier-fork-delta.md) | fork 相对官方 v0.36.1 改了什么、为什么、哪些该删 |
| [mps-formula-motors.md](mps-formula-motors.md) | 电机家族（直流/串励/无刷/步进/异步/伺服/直线/压电/SMA）的转矩–转速与热模型 |

`mps-core/` 下是按源文件拆开的导览，只从 [mps-core.md](mps-core.md) 进入。不要在新文档里再抄一份文件清单。

## 这些索引本身有守门

`crates/mps-test/src/docs_index_consistency.rs` 断言四件事，任何一条破了 `cargo test` 就红：

1. `docs/` 下每个 `.md` 都能从本页顺着相对链接走到（没人链接的页面 = 没人读的页面）；
2. 文档和四份根文档里的每个相对链接都能解析到真实文件；
3. `docs/adr/NNNN-*.md` 编号从 0001 连续、且都进了 ADR 索引表；
4. Rust 注释里点名的 docs 路径真实存在。

新增一篇文档时，**在拥有它的那份索引里加一行**，不要只把文件放进目录。

## 生成物

| 文件 | 生成命令 | 守门 |
| --- | --- | --- |
| [host-integration.md](host-integration.md) | 宿主从哪开始：1045 个 ABI 里 90% 场景只需哪 20 个 |
| [CAPABILITY_MAP.md](CAPABILITY_MAP.md) | `cargo run -p xtask -- capability-map` | `xtask verify` 的 `capability-map` 步（`--check` 比对） |
