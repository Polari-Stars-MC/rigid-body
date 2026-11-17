# ADR

记录已经在代码里生效、而且反过来改会破坏兼容或分层的决定。不记录想法。

编号连着写，不复用、不重排。正文用中文，四段就够：背景、决定、后果、相关代码。

| 编号 | 标题 | 状态 |
| --- | --- | --- |
| [0001](0001-f64-rapier.md) | 模拟用 f64 Rapier，不提供 f32 世界 | 接受 |
| [0002](0002-two-worlds.md) | 通用世界与太空世界分开，公式无世界 | 接受 |
| [0003](0003-c-abi-error-codes.md) | C ABI 错误码双侧字面声明，panic 不跨边界 | 接受 |
| [0004](0004-opaque-handles.md) | 句柄是不透明指针或 packed u64 | 接受 |
| [0005](0005-tests-live-in-mps-test.md) | 集成测试集中在 mps-test 并镜像源码 | 接受 |
| [0006](0006-generated-artifacts.md) | 头文件与 metrics 只由工具生成 | 接受 |
| [0007](0007-vendored-rapier-workspace.md) | vendored rapier 保持独立 workspace | 接受 |
| [0008](0008-rapier-fork-sync-policy.md) | rapier fork 固定 python-v tag 基线，同步走独立变基分支 | 接受 |
| [0009](0009-per-body-simulation-domains.md) | 体级模拟域：边界外用 `set_enabled` 冻结，region 语义是 sleep | 接受 |
| [0010](0010-rolling-resistance-and-contact-mechanics.md) | 滚动阻力进 fork 求解器；Hertz/Mindlin 与凸分解进公式层 | 接受 |
| [0011](0011-multibody-joint-damping-and-brake.md) | multibody 关节的逐 DoF 阻尼与 rake()：让电机真的能停 | 接受 |
| [0012](0012-motor-families-in-formula-layer.md) | 电机家族进公式层：从「一个伺服」到「一族机器」 | 接受 |
| [0013](0013-multibody-joint-force-cap.md) | multibody 关节的力矩上限：让九种电机模型在两条路径上通用 | 接受 |
| [0014](0014-fork-force-containers.md) | fork 的力容器模型：持久力不必每帧重加（事后补记） | 接受 |
| [0015](0015-explicit-collision-pair-disable.md) | fork 的显式禁用碰撞对（事后补记） | 接受 |
| [0016](0016-generic-abi-property-channel.md) | C ABI 通用属性通道（soft_body 首批切片） | 接受 |
| [0017](0017-vec3-with-flag-channel.md) | 向量 + 可选开关通道（14 个 `Vec3+Bool` 入口） | 接受 |
| [0018](0018-tyndall-effect-data-layer.md) | 丁达尔效应的数据层（体积散射，渲染交给 MC） | 接受 |
| [0019](0019-particle-driven-volumetric-scattering.md) | SPH/颗粒粒子云驱动的体积散射（光柱随烟雾流动） | 接受 |
| [0020](0020-spectral-tyndall.md) | 丁达尔效应的光谱版本（RGB 分通道，厚介质偏红） | 接受 |
| [0021](0021-particle-driven-spectral.md) | 粒子驱动的光谱散射（流动烟雾里的彩色光柱） | 接受 |
| [0022](0022-multiple-scattering.md) | 多次散射近似（阶数递推 + 等比级数） | 接受 |
| [0023](0023-uniform-grid-acceleration.md) | 均匀网格空间加速（O(steps·N) → O(N + steps·k)） | 接受 |
| [0024](0024-full-tyndall-composition.md) | 丁达尔效应的完整合成（粒子+分通道+多次散射+加速） | 接受 |
| [0025](0025-riccati-bessel-mie.md) | Riccati–Bessel 与米氏精确解（含未查清的 1.52 倍差异） | 接受 |
| [0026](0026-exact-mie-integration.md) | 米氏精确解接入（分区策略：x<0.3 用渐近式，否则精确解） | 接受 |

仓库根的 [DESIGN.md](../../DESIGN.md) 和 [OPTIMIZATION.md](../../OPTIMIZATION.md) 是被源码注释引用的章节索引，不是 ADR 的前身。它们的 `§` 编号继续留在原地。
