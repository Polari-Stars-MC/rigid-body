# 边界

越过这些线的改动要先有 ADR，或者先改对应的守门测试并在同一变更里说明。章节号的权威索引仍是仓库根 [DESIGN.md](../DESIGN.md) 与 [OPTIMIZATION.md](../OPTIMIZATION.md)。

## 依赖方向

允许：

```text
mps-bindgen-macro ← mps-formula ← mps-core    ← mps-jni
                                ← mps-cosmos  ← mps-jni
                                ← mps-web
mps-core ← mps-ffm ← mps-jni
mps-build-common ← mps-core/build.rs、mps-cosmos/build.rs
rapier3d-f64 ← 只有 mps-core 与 mps-cosmos
```

禁止：

- `mps-formula` 依赖 `rapier3d`、`mps-core`、`mps-cosmos`，或在签名里出现 `WorldHandle`。
- `mps-cosmos` 依赖 `mps-core`，或调用它的 C ABI / 共享 arena / 力律表。
- `mps-web` 依赖 `mps-core` 或 `mps-cosmos`。文档站不链接模拟器。
- 把 `rapier/` 加进根 workspace members。
- 为了通过编译而给公式 crate 加物理世界类型。

`mps-test` 可以同时依赖公式、core、cosmos、ffm 和 rapier，因为它是检测层，不是库。

## 错误有两条通道，不许混用

| 通道 | 谁用 | 失败长什么样 |
| --- | --- | --- |
| `FormulaError` | 纯公式的 `_checked` API | `Result`。不读不写线程局部槽 |
| `ERR_*` + `last_error_*` | C ABI / JNI | 返回哨兵，细节在调用线程的槽里 |

七个码两边都是字面常量，数值必须相同：

| 码 | 值 |
| --- | --- |
| `ERR_OK` | 0 |
| `ERR_NULL_POINTER` | 1 |
| `ERR_INVALID_ARGUMENT` | 2 |
| `ERR_NOT_FOUND` | 3 |
| `ERR_CAPACITY` | 4 |
| `ERR_UNSUPPORTED` | 5 |
| `ERR_INTERNAL` | 6 |

定义位置：`crates/mps-formula/src/error.rs` 与 `crates/mps-core/src/rapier/error.rs`。core 侧用 `const _: () = { assert!(...); }` 钉住。守门：`error_consistency`。

数值域错误在 FFI 上变成 `ERR_INVALID_ARGUMENT`，不是 `ERR_INTERNAL`。`ERR_INTERNAL` 留给 `ffi_guard` 接住的 panic。指针问题保持 `ERR_NULL_POINTER`。

`last_error_message` 返回的指针属于 Rust 的线程局部槽，下一次同线程报错就失效，调用者不得 free。槽不跨线程。

## ABI 兼容

已发布的 `extern "C"` 名字、参数顺序、`#[repr(C)]` 布局、失败哨兵、`_flag` 变体，默认冻结。

可以加新符号。不可以：

- 改已有符号的签名或改名后只留新名字。
- 把 Rapier / Rust 枚举、引用、`Vec`、`String` 放进 ABI。
- 让 `extern "C"` 函数在 panic 时展开进宿主。每个入口走 `ffi_guard`；根 `profile.release` 保持 `panic = "unwind"`。
- 手改 `rigid_body.h` / `cosmos.h`。改 Rust 侧可导出项，然后重新构建，提交生成结果。
- 改 `shared_arena` 已发布常量而不升版本。`ARENA_VERSION` 与 `mps-ffm::ABI_VERSION` 一起动。守门：`arena_compat`、`version_consistency`。

`mps-ffm` 当前只承诺三件事：版本号，以及「这份库带 FFM / JNI 探针」。它不是第二套物理 API。

## 句柄与内存

- 世界和 builder 的指针由创建它的那一侧 destroy。不在 Java 里 `free`。
- packed `u64` 在删除并复用代后失效。不要把句柄当数组下标缓存到对象死亡之后。
- 共享 arena 与 force queue 是给 DirectByteBuffer 的零拷贝布局。改字段偏移等于改 ABI。
- 默认碰撞体策略只影响之后的 `worldInsertDefaultCollider`。它不关闭全局碰撞，也不替换已经插入的形状。见 [world-collision-mode.md](world-collision-mode.md)。
- 刚体中心 BVH 只服务区域策略查询。它不替代 Rapier 的碰撞体 BVH，也不允许外部换掉 `bodies` 集再指望索引仍对。见 [body-spatial-index.md](body-spatial-index.md)。

## Feature

`default = []`。CI 只编默认集。

| Feature | 开了什么 | 约束 |
| --- | --- | --- |
| `anvilkit-bridge` | `anvilkit` + `bevy_ecs`，模块整段 `cfg` | 不得进入 `default`。`--all-features` 会编到有既存错误的测试，不要顺手开 |
| `relative-force` | 相对力路径 | 同上，默认关 |
| `profiler` | `rapier3d/profiler` | 只用于步进矩阵的分阶段列。验收绝对时间时不要开 |

大型可选集成用 `cfg(feature)` 把门关死，而不是运行时探测后半初始化。

## 测试边界

- 集成测试只放 `mps-test`，目录与源模块镜像。守门：`verify_module_mirror`。
- 源 crate 不放 `#[cfg(test)]` 模块来代替镜像。
- 守门测试失败时修产生漂移的那一侧，不要 `#[ignore]`、不要放宽断言来让 CI 绿。
- 性能矩阵是 `#[ignore]`，不进默认 `cargo test`。跑法见 [world-step-benchmarks.md](world-step-benchmarks.md)。

## 生成物

| 文件 | 生成命令 | 可否手改 |
| --- | --- | --- |
| `crates/mps-core/include/rigid_body.h` | 构建 `mps-core`（cbindgen） | 否。Linux CI 会 diff |
| `crates/mps-cosmos/include/cosmos.h` | 构建 `mps-cosmos`（cbindgen） | 否。Linux CI 会 diff |
| `crates/mps-web/src/metrics.rs` | `cargo run -p xtask -- dump-metrics` | 否。仓内守门 `verify_metrics_sync`（3 个核心计数）+ Linux CI 重新生成后 `git diff`（全部 18 个常量） |
| xtask `gen-java` 输出 | `cargo run -p xtask -- gen-java [dir]` | 否 |

Java 业务源码在仓库外。本仓库的 JNI 符号是契约，Java 声明要跟 C 头文件走，而不是反过来改 Rust 去凑一份未提交的 Java。

## 数值

- 模拟与公式的浮点是 f64。不要在 ABI 上另开一套 f32 世界。
- 纯公式：NaN 与无穷大是非法输入，除非该函数的文档点名允许某一个无穷结果。没有全局 `allow_infinity` 开关。
- 次正规数和向 0 下溢按 IEEE 754。下溢成 0 的分母仍然是除零。
- 保守的有限性检查可以拒绝「数学上有限、但中间值溢出」的极端输入。这是策略，不是 bug。细节在 [formula-numeric-domains.md](formula-numeric-domains.md)。

## 文档边界

- 根 `README.md` 只保留人能在一屏内看完的介绍。
- `AGENTS.md` 只保留代理操作约定，不复制源文件导览。
- `DESIGN.md` / `OPTIMIZATION.md` 只保留被代码引用的章节。不要给它们编新的叙事章节号。
- 新的不可逆决定写 [adr/](adr/README.md)。新的「已被钉住」的行为写 [evidence/](evidence/README.md)，并指向真实测试或命令。
