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
- 引力近场用 Plummer 软化（`NEAR_FIELD_SOFTENING_SQ`）取代旧 1 m 硬截止；`ffi_guard`/`ffi_scalar` 覆盖有守门测试扫描（`verify_ffi_guard`，扫 `mps-core` 与 `mps-cosmos`；`mps-formula` 的 162 个入口当前 **0 违规但不在扫描范围**）。
- 验收入口收敛成一条命令：`cargo run -p xtask -- verify`（fmt / clippy / test / release / 两份头文件 diff / metrics diff / web audit），与 CI 步骤一一对应。
- 文档索引自身有守门：`docs_index_consistency` 钉住索引可达性、相对链接、ADR 编号连续性和代码注释里的 docs 指针。

## 还开着

按「不做就会继续伤到边界」排序，不是排期。

**每条都必须附一条能证明它仍然开着的命令。** 第 2 条曾在这里挂了整整一天，其实早已不成立——没有可执行判据的条目会腐烂成传说，下一轮的人会照着它做错事。删掉一条之前，对应的守门或复现命令必须存在。

1. **数值域迁移没有铺满。** `FormulaError` 只覆盖 [formula-numeric-domains.md](formula-numeric-domains.md) 里列出的 `_checked` API。其余公式仍是旧的直接返回。新公式走 checked 路径；旧函数只有在不改 C 签名和失败哨兵的前提下才加 checked 变体。
2. ~~**`anvilkit-bridge` 的测试编译是坏的。**~~ **已不成立（2026-11-13 实测）**：`cargo test -p mps-test --features anvilkit-bridge --lib anvilkit` → 2 passed；`--all-features --no-run` 也通过。这条从 2026-11-12 起就过期了，没人清。教训写进下一条。
3. **Java 声明在仓库外。** `world-collision-mode.md` 仍要求宿主手工补 JNI 声明。`xtask gen-java` 只覆盖带 `#[java_struct]` / `#[java_enum]` 的值类型。JNI 方法清单没有生成进本仓库。在边界稳定之前，不把宿主工程并进来。（2026-11-12 起有所缓解：`crates/mps-test/java/` 的冒烟 fixture + CI `jni-smoke` job 把高价值 JNI 面锁进了契约；宿主仓库仍需自行声明。）
4. **`xtask verify` 不覆盖两层 CI。** 它跑的是 Rust 侧的七项加 web audit；`jni-smoke`（真实 JVM）和跨 OS 矩阵（Ubuntu / Windows / macOS）只能由 CI 跑。改 JNI 签名或平台相关代码时，本地绿 ≠ CI 绿。
5. **文档守门只查图结构，不查内容。** `docs_index_consistency` 证明索引连通、链接可解析、ADR 编号连续、代码注释指针不悬空；它**不**证明页面里写的数字或结论仍然成立（`soft_body.md` 曾经写「87 个入口」而实际是 88）。数字以生成物 `metrics.rs` 为准，不要手抄进文档。
6. **正确性没有下界。** 1233 个测试全绿只证明「没崩、没变」，不证明「算对了」——上一轮审查发现的 SPH 自贡献重复计算、轮胎力量纲错误、光子动量必得 NaN，全部在测试全绿的状态下存在过。缺的是解析解对照、守恒律断言，以及 114 个既无 JNI 绑定也无测试引用的入口（见 [CAPABILITY_MAP.md](CAPABILITY_MAP.md)）的基本有限性检查。
7. **没有内存安全机械化。** `crates/mps-core` 有 799 个 `unsafe` 块，CI 里没有 Miri / ASAN / fuzz，也没有 `cargo audit`。`ffi_guard` 这条硬规则现在由 `verify_ffi_guard` 静态扫描（三个 crate 全覆盖），但扫描是文本级的，只能证明标识符出现在函数体里，不能证明它接得对。

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
