# 路线

只写从当前代码能证明的状态。没有日期，没有承诺的版本号。完成的判据是「仓库里已经这样」，不是「计划过」。

## 现在就是这样

这些已经落地，继续沿用，不单独立项重做：

- f64 Rapier 后端，C ABI，JNI 库名 `mps_rigid_body`，FFM 只做版本探针。
- 公式 / 通用世界 / 太空世界三层，太空世界不进入 `mps-core`。
- 体家族（刚体、软体、布料、绳、气囊、颗粒、铰接、角色、传感器、车辆、伺服）各有模块和测试镜像。
- 错误码双侧字面声明、模块镜像、metrics、arena ABI、版本常量，都有守门测试。
- `rigid_body.h` 与 `cosmos.h` 都由 cbindgen 生成；Linux CI 在 release 构建后 diff 这两份。
- 文档站是 Dioxus 0.7 单页 SSR，计数由 xtask 生成。
- 默认碰撞体策略、刚体中心 BVH、`world_step` 性能矩阵已有专题文档和测试入口。
- 录制回放（`debug_recording_*`）与浮动原点（`cosmosWorldRecenter` / 自动 recenter）有 Rust 守门 + Java JNI 冒烟 fixture，CI 里有 `jni-smoke` job 用真实 JVM 验证导出符号与链接。
- 引力近场用 Plummer 软化（`NEAR_FIELD_SOFTENING_SQ`）取代旧 1 m 硬截止；`ffi_guard`/`ffi_scalar` 覆盖有守门测试扫描。

## 还开着

按「不做就会继续伤到边界」排序，不是排期。

1. **数值域迁移没有铺满。** `FormulaError` 只覆盖 [formula-numeric-domains.md](formula-numeric-domains.md) 里列出的 `_checked` API。其余公式仍是旧的直接返回。新公式走 checked 路径；旧函数只有在不改 C 签名和失败哨兵的前提下才加 checked 变体。
2. **`anvilkit-bridge` 的测试编译是坏的。** `crates/mps-test/src/rapier/anvilkit.rs` 在 `--all-features` 下有既存错误。默认 CI 不编它。要么修到 feature 自己能 `cargo test -p mps-test --features anvilkit-bridge`，要么在 ADR 里写明这个桥冻结；不要把它放进 `default`。
3. **Java 声明在仓库外。** `world-collision-mode.md` 仍要求宿主手工补 JNI 声明。`xtask gen-java` 只覆盖带 `#[java_struct]` / `#[java_enum]` 的值类型。JNI 方法清单没有生成进本仓库。在边界稳定之前，不把宿主工程并进来。（2026-11-12 起有所缓解：`crates/mps-test/java/` 的冒烟 fixture + CI `jni-smoke` job 把高价值 JNI 面锁进了契约；宿主仓库仍需自行声明。）

## 明确不做

- 不提供 f32 世界，也不把 f32 作为 feature。
- 不把 `mps-cosmos` 并进 `PhysicsWorld`，不共享 arena。
- 不把 Rapier 类型、Rust `Vec`/`String` 放进 C ABI。
- 不把测试搬回源 crate 的 `#[cfg(test)]`。
- 不把 `anvilkit` 或 `bevy_ecs` 变成默认依赖。
- 不改 `DESIGN.md` / `OPTIMIZATION.md` 已有章节号来「整理文档」。新决定进 `docs/adr/`。
- 不把 `rapier/` fork 的上游同步做成无记录的大合并。要同步就单独做，并写清行为差异。

## 改路线时

删掉一行「还开着」之前，对应的守门或专题文档必须已经存在。新增一行「明确不做」时，在 [adr/](adr/README.md) 留一条，避免下一轮再提案。
