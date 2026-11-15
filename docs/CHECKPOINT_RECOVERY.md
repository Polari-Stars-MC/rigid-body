# 检查点与恢复

长任务（跨模块重构、公式迁移、头文件或 metrics 再生、性能矩阵）中断后，用这份清单把仓库恢复到可提交状态。不要靠聊天记录回忆生成物有没有更新。

## 开工前记一笔

在任务笔记里写清这四项，不要写进生成文件：

1. 要动的 crate（公式 / core / cosmos / jni / 测试）。
2. 会不会改 `extern "C"`、`#[repr(C)]`、`ERR_*`、arena 常量。
3. 预期要再生的文件（见下表）。
4. 用来证明做完的命令——现在是**一条**：`cargo run -p xtask -- verify`。需要中间态时再补一条具体滤镜（`cargo test -p mps-test <名称>`），不要写「跑一下测试」。

## 生成物对照

| 若你改了 | 必须再产生 | 怎么确认没漂 |
| --- | --- | --- |
| `mps-core` 里 cbindgen 能看见的项 | `crates/mps-core/include/rigid_body.h` | `verify` 的 `headers` 步（重新构建后 `git diff` 该文件） |
| `mps-cosmos` 里 cbindgen 能看见的项 | `crates/mps-cosmos/include/cosmos.h` | 同上，同一步骤 |
| 测试数量、`jni!` / `jni_e_c!`、`pub extern "C" fn` | `crates/mps-web/src/metrics.rs` | `verify` 的 `metrics` 步；仓内另有 `cargo test -p mps-test verify_metrics`（只比 3 个常量） |
| `#[java_struct]` / `#[java_enum]` | `gen-java` 的输出目录 | 输出头有 “Do NOT edit by hand” |
| `mps-core` / `cosmos` / `formula` 的子模块增删改名 | `mps-test` 里同名镜像 | `cargo test -p mps-test verify_module_mirror` |
| 任一侧 `ERR_*` 数值 | 另一侧的字面常量 | `cargo test -p mps-test error_consistency` |
| `MPS_ARENA_VERSION` / `COSMOS_ARENA_VERSION` / `ABI_VERSION` | 两侧常量和 workspace 版本锁 | `arena_compat` 与 `version_consistency` |
| 任何 JNI 方法签名 | `crates/mps-test/java/` 里对应 fixture | 本机没有 JVM 时靠 CI 的 `jni-smoke`；**`verify` 不跑它** |
| 新增 / 移动 / 删除 `.md` | 拥有它的那份索引里的一行 | `cargo test -p mps-test docs_index_consistency` |

注意两个头文件里的版本宏现在是 `MPS_ARENA_VERSION`（core）与 `COSMOS_ARENA_VERSION`（cosmos），Rust 侧常量仍叫 `ARENA_VERSION`。这是为了让宿主能同时 include 两份却不触发宏重定义，不要「统一」回去。

只改了纯公式的内部实现、且 C 签名不变时，头文件和 metrics 通常不动。不确定就跑一次 `verify`，不要猜。

## 中断时先看工作区

```powershell
git status --short
git diff --stat
```

按这个顺序处理，不要反过来：

1. **生成的头文件和 `metrics.rs` 有手改痕迹。** 丢掉手改，从源码重新生成。不要在冲突里「留着看起来对的一行」。
2. **源码改了，头文件 / metrics 没变。** 按上表补生成，把生成结果和源码放进同一次提交。
3. **只多了测试文件或少了源模块。** 先跑 `verify_module_mirror`。失败信息会指出哪一侧缺文件。
4. **`cargo test` 中途杀掉。** 这不是失败结论。用上次的滤镜重跑那一条。全量 `cargo test` 很重，恢复时不要靠它定位。
5. **开过 `--all-features` 或 `anvilkit-bridge`。** 该 feature 的测试有既存编译错误。默认 CI 不编它。除非这次任务就是修桥，否则把命令退回默认 features 再判断对错。

## 可以停下来的状态

下面每条都为真，任务才算停在安全点：

- `cargo run -p xtask -- verify` 退出码 0。它一次盖住 fmt（逐包圈定，不是 `--all`）、clippy、`cargo test`、release 构建、两份生成头文件的 `git diff`、`metrics.rs` 的 `git diff` 和 web audit——也就是 CI 的全部步骤。
- 恢复期间只想确认某一项时用 `--skip`：`cargo run -p xtask -- verify --skip test,build-release` 只跑检查不跑长跑。但**收尾判断必须跑不带 `--skip` 的那一次**。
- 没有把 `panic = "abort"`、`anvilkit-bridge` 进 `default`、或公式 crate 对 Rapier 的依赖留在工作区里。

做不到时，在任务笔记里写明卡在哪一步（`verify` 的步骤名）和哪一个测试名。不要用「差不多好了」当作检查点。

## 性能矩阵中断

矩阵默认 `#[ignore]`，产物在 `target/world-step-matrix.csv` 和 `target/world-step-matrix-raw.csv`。下一次运行会覆盖这两个文件。中断或重跑之前，若还要旧数字，先把 CSV 拷到仓库外。不要把 `target/` 提交进 git。

绝对时间对比要在同一台机器、同一电源策略、不开 `profiler` feature 的 release 下进行。分阶段列只有 `--features profiler` 才有意义，那一组数字不能拿来和绝对时间验收比。

## 文档任务中断

- 根 `README.md` 保持短。细节补进 `docs/`，不要补回 README。
- 不要改 `DESIGN.md` / `OPTIMIZATION.md` 的既有 `§` 编号。
- 新决定写 `docs/adr/NNNN-标题.md` 并在 `docs/adr/README.md` 加一行。
- `docs/mps-core/*.md` 是源文件导览。只在对应 `.rs` 的职责变了才改那一篇，不要顺手重写整个目录。
- 新增/移动/删除任何 `.md` 之后跑 `cargo test -p mps-test docs_index_consistency`：它检查索引可达性、相对链接、ADR 编号和代码注释里的 docs 指针。四项里前三项正是「文件放进目录但没进索引」这类漂移的唯一防线。
