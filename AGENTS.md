# AGENTS.md

给在本仓库里改代码的代理。人读的简要在 [README.md](README.md)；约束的正文在 [docs/](docs/README.md)。

`mps`（Motion Physics System）是 Rust workspace，版本 **0.1.4**，edition **2024**。它把 path 依赖的 `rapier3d-f64` 包进稳定 C ABI，再由 JNI / FFM 给仓库外的 Java 宿主用。workspace 有 10 个 crate。`rapier/` 是独立 workspace，根 `Cargo.toml` 的 `exclude` 已经把它和 `docs/` 排除在外。

## 先读这些

| 文件 | 什么时候读 |
| --- | --- |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 要动 crate 边界、世界步进、句柄或 FFI 面 |
| [docs/BOUNDARY.md](docs/BOUNDARY.md) | 要加依赖、错误码、生成物、feature 或测试 |
| [docs/adr/README.md](docs/adr/README.md) | 要改一条已经记下来的决定 |
| [docs/evidence/README.md](docs/evidence/README.md) | 要宣称某条行为已被测试或基准钉住 |
| [docs/ROADMAP.md](docs/ROADMAP.md) | 要判断一件事是现在做还是先别做 |
| [docs/CHECKPOINT_RECOVERY.md](docs/CHECKPOINT_RECOVERY.md) | 长任务中断后要接着做 |
| [DESIGN.md](DESIGN.md) / [OPTIMIZATION.md](OPTIMIZATION.md) | 代码注释里的 `§N` 指向这里；章节号是守门测试的锚，不能改号 |

`docs/mps-core/` 是 `mps-core` 每个源文件的导览，索引在 [docs/mps-core.md](docs/mps-core.md)。不要把那一套导览再复制进本文件。

## Crates

| Crate | 产物 | 职责 |
| --- | --- | --- |
| `mps-formula` | `rlib` | 纯公式。禁止 Rapier、`WorldHandle`、共享 arena |
| `mps-core` | `cdylib` + `rlib` | 物理世界、体家族、查询、C ABI。头文件 `include/rigid_body.h` |
| `mps-cosmos` | `rlib` | 独立太空世界。只用 `mps-formula`，不进 `mps-core` 的世界。头文件 `include/cosmos.h` |
| `mps-jni` | `cdylib` + `rlib`，库名 `mps_rigid_body` | Java 实际加载的库 |
| `mps-ffm` | `cdylib` + `rlib` | ABI 版本探针（`abi_version` / `abi_supports_*`） |
| `mps-test` | lib（测试） | 全部集成测试。源 crate 里不放 `#[cfg(test)]` |
| `mps-web` | bin | Dioxus 0.7 SSR 文档站。`src/metrics.rs` 由 xtask 生成 |
| `mps-build-common` | `rlib` | `run_cbindgen()`，只给两个 `build.rs` 用 |
| `mps-bindgen-macro` | proc-macro | `#[java_struct]` / `#[java_enum]` |
| `xtask` | bin | `verify`（验收入口）、`dump-metrics`、`gen-java` |

依赖只能沿 `mps-formula → mps-core / mps-cosmos → mps-jni` 走。`mps-cosmos` 不得依赖 `mps-core`。

## 改代码时必须守的

- 公式函数只做输入到输出。读刚体、施力、步进放在 `mps-core` 或 `mps-cosmos`。
- C ABI 名用 `模块_动词`，snake_case。已发布的名字、参数、失败哨兵和 `_flag` 变体保持不变；要破坏就新开 ADR。
- 错误码是 `u32`：`ERR_OK=0` … `ERR_INTERNAL=6`。`mps-formula` 与 `mps-core` **各自字面声明**（cbindgen 不认 `pub use`）。改一边必须改另一边。纯公式的数值域错误走 `FormulaError`，不写线程局部错误槽；FFI 边界再映射成 `ERR_*`。
- 每个 `extern "C"` 入口用 `ffi_guard`（`catch_unwind`）。release profile 的 `panic` 必须保持 `"unwind"`。
- 句柄：世界和 builder 是不透明指针；刚体、碰撞体、关节是 packed `u64`。不要把 Rapier 类型送过 ABI。
- 新增 `mps-core` / `mps-cosmos` / `mps-formula` 子模块时，同步加 `mps-test` 镜像文件，并在 `crates/mps-test/src/lib.rs` 里用 `pub mod` 挂上。守门在 `verify_module_mirror`（两条：文件名镜像 + 每个测试文件都必须被 `mod` 声明，未声明的文件不会被编译）。
- 改了会进入头文件的 FFI 后要重新构建，并提交生成的 `rigid_body.h` 和 `cosmos.h`。CI 只在 Linux 上对这两份做 `git diff --exit-code`，不要手改。
- 改了测试、JNI 或 core FFI 数量后跑 `cargo run -p xtask -- dump-metrics`，提交 `crates/mps-web/src/metrics.rs`。CI 会在 Linux 上重新生成再 `git diff`（覆盖全部 18 个常量）。
- `default = []`。`anvilkit-bridge`、`relative-force`、`profiler` 不进默认集，也不要为了“顺便编译过”去开 `--all-features`。
- 格式：`rustfmt.toml`（edition 2024，宽 100，4 空格）。文档注释里的缩写跟 `clippy.toml` 的 `doc-valid-idents` 一致，不要展开。

## 命令

**开工前先跑一次 `cargo run -p xtask -- verify --list`**，它打印的七步就是 CI 跑的全部步骤。做完再跑：

```text
cargo run -p xtask -- verify              # 唯一的验收入口，一个退出码
cargo run -p xtask -- verify --keep-going # 一次看到所有失败步骤
cargo run -p xtask -- verify --skip test  # 长跑时跳过测试重跑
```

七步分别是 `fmt` / `clippy` / `test` / `build-release` / `headers` / `metrics` / `web-audit`，顺序与 `.github/workflows/ci.yml` 一致。`fmt` 步骤**从根 `Cargo.toml` 的 members 现场解析包名**再逐包 `-p` 圈定——不要改回 `cargo fmt --all --check`：新版 rustfmt 会顺着 path 依赖把 vendored `rapier/`、`parry/` 卷进 `--all`，按本仓库的 `rustfmt.toml` 重排 fork 代码，破坏 ADR-0008 的补丁纪律（提交 `52a4b59` 就是在修这个）。`headers` / `metrics` 两步要求重新生成后 `git diff` 为空。

定位问题时才单跑某一条：

```text
cargo test -p mps-test <名称>           # 不要无目的地跑完全部
cargo build --release -p mps-jni        # Java 加载的库
cargo run -p xtask -- dump-metrics      # 只重生 metrics.rs（verify 里已含）
```

CI（`.github/workflows/ci.yml`）在 Ubuntu / Windows / macOS 上跑同一套默认 features，外加 Linux 上的两份头文件 diff、`dump-metrics` diff、`python3 crates/mps-web/scripts/web_audit.py`，以及 `jni-smoke` job——那个 job 用真实 JVM 加载 cdylib 并跑 [crates/mps-test/java/](crates/mps-test/java) 下的 6 个 Java fixture。**改了 JNI 签名就要改 fixture**，Rust 侧测试全绿不代表 CI 会绿。

本机 Windows 用 GNU 工具链（`stable-x86_64-pc-windows-gnu`）。MSVC 目标在 `.cargo/config.toml` 里有 rustflags，但本机没有可用的 `link.exe` 时不要切过去。

## 不要做

- 不要把 `rapier/` 并进本 workspace，也不要改它来迁就上层，除非任务明确是修 vendored fork。
- 不要手改 `include/*.h`、`metrics.rs`、xtask 生成的 Java。
- 不要静默 `error_consistency`、`verify_module_mirror`、`verify_metrics_sync`、`arena_compat`、`version_consistency`、`verify_ffi_guard`、`docs_index_consistency`。
- 新增文档时，在拥有它的那份索引里加一行（`docs/README.md` / `docs/adr/README.md` / `docs/evidence/README.md` / `docs/mps-core.md`），不要把文件放进目录就算完——`docs_index_consistency` 会红。
- 不要把提交信息写成 Conventional Commits。本仓库用日期戳（如 `2026.8.11.20.8`）或一句短描述。
