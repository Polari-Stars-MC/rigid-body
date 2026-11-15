# 证据

每条记录一件**已经被仓库钉住**的事：哪段代码、哪条命令、失败时会怎样。不记录「应该再测什么」。

数字会过期。计数以 `crates/mps-web/src/metrics.rs` 为准，那份文件由 xtask 生成；本目录不复制那些数字。

| 记录 | 钉住了什么 |
| --- | --- |
| [error-codes.md](error-codes.md) | 七个 `ERR_*` 双侧相等 |
| [module-mirror.md](module-mirror.md) | 测试目录与源模块一致 |
| [metrics.md](metrics.md) | 文档站计数与源码一致 |
| [abi-versions.md](abi-versions.md) | arena ABI 与 FFM 版本一起动 |
| [world-step.md](world-step.md) | 步进性能矩阵的命令和 CSV |
| [numeric-domains.md](numeric-domains.md) | 已迁移的 checked 公式 |
| [numeric-drift.md](numeric-drift.md) | f64 长时积分漂移、挖洞唤醒、长期轨道、Plummer 软化、双世界一致性、录制重演位级守门 |
| [convergence.md](convergence.md) | 自适应子步收敛性、天体自转、central_body 切换 SOI 契约 |
| [performance-domains-fracture.md](performance-domains-fracture.md) | 冻结域步进开销（B1）与接触力→自动碎裂闭环（B2） |
| [real-physics-validation.md](real-physics-validation.md) | 与解析解对照的真实物理系列：终端速度、摩擦/恢复系数能量学、Hohmann 闭环、颗粒休止角 |
| [engine-compare.md](engine-compare.md) | 与 Jolt 的同负载性能对比协议（含对齐要求） |

CI 本身（fmt、clippy、`cargo test`、release 构建、Linux 上的 `rigid_body.h` 与 `cosmos.h` diff、`web_audit.py`）记在 `.github/workflows/ci.yml`，不在这里重复步骤。
