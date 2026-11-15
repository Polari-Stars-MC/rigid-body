# mps — Motion Physics System

**mps** 既是米每秒（m/s），也是 Motion Physics System。仓库名是 `rigid-body`；文档和代码里的项目名是 **mps**。

它是一套 Rust workspace（0.1.4，edition 2024），用双精度 [`rapier3d-f64`](https://rapier.rs) 做刚体后端，把世界、体、碰撞、查询和事件收在稳定的 C ABI 后面。Java 宿主加载的是 `mps-jni` 产出的 `mps_rigid_body`；Rapier 类型不过边界。公式层没有物理世界，可以单独复用。

```text
mps-formula   纯公式（无 Rapier）
mps-core      通用物理世界 + C ABI（rigid_body.h）
mps-cosmos    独立太空世界 + C ABI（cosmos.h），只依赖公式
mps-jni       Java 加载的 cdylib
mps-ffm       ABI 版本探针
mps-web       Dioxus 文档站
```

`rapier/` 是 vendored fork，自己的 workspace，通过 path 依赖进来。

## 为什么是 f64

长时轨道、航天和多体里，f32 的漂移不可接受。代价是内存和一部分算子更贵。这是默认，不是 feature。

## 两层

| 层 | 做什么 | 不做什么 |
| --- | --- | --- |
| `mps-formula` | 数值进、数值出 | 不碰 `WorldHandle`、刚体、Rapier |
| `mps-core` / `mps-cosmos` | 读状态、调用公式、施力、步进 | 不把求解器类型送过 ABI |

已发布的 C 函数名、参数、错误码和 `_flag` 变体保持兼容。错误码是 `0..6` 的 `ERR_*`；panic 在 FFI 边界收成 `ERR_INTERNAL`。

## 布局

```text
crates/     10 个 crate（公式、核心、太空、JNI、FFM、测试、文档站、构建辅助、宏、xtask）
docs/       架构、边界、路线、ADR、证据、mps-core 源码导览
rapier/     vendored rapier3d-f64（独立 workspace）
```

源码导览和设计约束从 [docs/README.md](docs/README.md) 进。给代理的操作约定在 [AGENTS.md](AGENTS.md)。

## 构建

```powershell
cargo run -p xtask -- verify     # 唯一的验收入口：fmt / clippy / test / release / 生成物 / web audit
```

`xtask verify` 跑的八步与 CI 一一对应，退出码就是结论；`cargo run -p xtask -- verify --list` 会打印每一步及其命令。单跑某一条时用 `cargo test -p mps-test <名称>` 或 `cargo build --release -p mps-jni`。

改 C ABI 之前先查 [docs/CAPABILITY_MAP.md](docs/CAPABILITY_MAP.md)：它由 `cargo run -p xtask -- capability-map` 生成，逐模块列出 C ABI 前缀、导览页、镜像测试、JNI 与测试覆盖。

CI 在 Ubuntu、Windows、macOS 上跑同一套默认 features，另加一个 `jni-smoke` job：用真实 JVM 加载 cdylib，跑 `crates/mps-test/java/` 下的 Java fixture。可选的 `anvilkit-bridge`、`relative-force`、`profiler` 默认关闭。

在线文档由 `crates/mps-web` 生成，发布到 <https://Polari-Stars-MC.github.io/rigid-body/>。站点基数来自 GitHub Pages 的 `base_path`，fork 不需要改硬编码路径。
