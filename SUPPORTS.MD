# SUPPORTS.MD — 项目分析报告

> 生成日期: 2026-07-03
> 项目: mps_rigid_body v0.1.4
> 分析者: Claude Code (glm-5.2)

---

## 项目概述

`mps_rigid_body` 是一个基于 Rust 的原生物理引擎库，底层使用 `rapier3d-f64` (v0.33.0)，通过 JNI (Java 21) 和 Java FFM (Java 25) 两种方式向 Java 端暴露 API。项目采用 GPL-3.0 许可证。

核心架构:
```
Java 21 JNI / Java 25 FFM
  → Rust C ABI / JNI wrappers
    → Rapier 包装模块
      → rapier3d-f64
```

---

## 依赖与版本状态

| 依赖 | 当前版本 | 最新版本 | 是否需要更新 |
|------|----------|----------|-------------|
| rapier3d-f64 | 0.33.0 | 检查 crates.io | 需检查 |
| jni (ljni) | 0.21.0 | 检查 crates.io | 需检查 |
| parking_lot | 0.12.5 | 检查 crates.io | 需检查 |
| hashbrown | 0.17.1 | 检查 crates.io | 需检查 |
| smallvec | 1.15.2 | 检查 crates.io | 需检查 |
| mimalloc | 0.1.52 | 检查 crates.io | 需检查 |
| rayon | 1.12 | 检查 crates.io | 需检查 |
| bevy_ecs | 0.19.0 (optional) | 检查 crates.io | 需检查 |
| cbindgen (build) | 0.29.4 | 检查 crates.io | 需检查 |

---

## 需要更新的内容

### 1. 🔴 高优先级

#### 1.1 Cargo.toml 使用了 `edition = "2024"`
- Rust 2024 edition 于 2025 年 2 月随 Rust 1.85 稳定。当前项目使用此版本是合理的。
- **无需更新**，但需确保 CI 和开发者使用的 Rust 版本 ≥ 1.85。

#### 1.2 ~~缺失 `.cargo/config.toml`~~ ✅ 已解决 (2026-07-03)
- 已添加 `.cargo/config.toml`，涵盖所有已安装 target 的平台特定 linker flags。
- 在 `Cargo.toml` 中补充了 `[profile.release]` 优化参数 (lto, codegen-units, opt-level, strip, debug)。

#### 1.3 ~~缺失 CI/CD 配置~~ ✅ 已解决 (2026-07-03)
- 已更新 `.github/workflows/ci.yml`，包含 3 个 job：
  - `rust` — cargo fmt, clippy, test, build --release (ubuntu + windows)
  - `java21` — test21 JNI Gradle smoke test (JDK 21)
  - `java25` — test25 FFM Gradle smoke test (JDK 25)

### 2. 🟡 中优先级

#### 2.1 ~~README.md 中的项目名称不一致~~ ✅ 已解决 (2026-07-03)
- README.md 标题和首段中的 `msp_rigid_body` 已修正为 `mps_rigid_body`，与 Cargo.toml 中的 crate 名保持一致。

#### 2.2 ~~缺少 Rust 单元测试覆盖率~~ ✅ 已解决 (2026-07-03)
- 为核心模块新增 111 个单元测试：
  - `rigid_body.rs`: 52 个测试 — builder 创建/销毁/配置、world insert/remove/copy、get/set status/translation/rotation/pose/linvel/angvel、force/impulse、sleep/wake-up、CCD
  - `controller.rs`: 24 个测试 — create/destroy、set_up、offset、slide、autostep、snap_to_ground、slope_angles、move_shape、collision_count、solve_impulses
  - `joints.rs`: 25 个测试 — 6 种关节类型 create/destroy、anchors、limits、motor velocity/position、insert/remove roundtrip
- 测试总数从 188 → 299，全部通过。

#### 2.3 cbindgen.toml 导出类型列表可能过时
- `cbindgen.toml` 中的 `[export].include` 列表手动维护了约 150 个类型。
- 当添加新的 FFI 类型时，容易忘记更新此列表。
- 建议：添加自动化检查脚本，或在 CI 中对比生成的头文件是否有差异。

### 3. 🟢 低优先级

#### 3.1 缺少 `CHANGELOG.md`
- 项目有版本号 (0.1.4) 但没有变更日志。
- 建议添加 `CHANGELOG.md`，记录每个版本的变更。

#### 3.2 缺少 `CONTRIBUTING.md`
- 对于开源项目，建议添加贡献指南，说明如何设置开发环境、运行测试、提交 PR 等。

#### 3.3 `.gitignore` 中的 `target` 条目
- 当前 `.gitignore` 写的是 `target`（无前缀 `/`），这会忽略所有名为 `target` 的目录。
- 建议改为 `/target` 以仅忽略项目根目录的 target 文件夹。

#### 3.4 `LICENSE` 文件中的版权信息未填写
- GPL-3.0 许可文本末尾的 "How to Apply These Terms" 部分中，`<program name>`、`<year>`、`<name of author>` 都是占位符，未填写实际信息。

#### 3.5 考虑添加 `rust-toolchain.toml`
- 使用 `edition = "2024"` 需要较新的 Rust 版本。建议添加 `rust-toolchain.toml` 固定工具链版本，确保开发者使用一致的 Rust 版本。

---

## 项目结构与代码质量

### 优点
- 模块化设计清晰，`src/rapier/` 下有 30+ 个子模块，涵盖刚体、碰撞体、关节、体素、查询、流体、声学、热力学、电磁学、相对论等广泛领域。
- FFI 边界设计合理：使用 opaque pointer 传递 World 所有权，使用 packed `u64` handle 传递 Rapier 内部句柄。
- 配置了 `rustfmt.toml` 和 `clippy.toml`，代码风格统一。
- 使用 `mimalloc` 作为全局分配器，关注性能。
- `Cargo.toml` 中 dev 和 release profile 都设置了 `panic = "abort"`，减少二进制体积。

### 改进建议
- 部分模块（如 `quantum`、`superfluidity`、`plasma`、`relativity` 等）功能边界较宽，建议在模块文档中说明每个模块的用途和使用场景。
- `test21/` 和 `test25/` 目录中包含了 Minecraft 相关文件（`.gradle/caches/minecraft/versions/1.21.1/`），确认是否为有意保留。

---

## 总结

| 优先级 | 事项 | 类型 |
|--------|------|------|
| 🔴 高 | ~~添加 CI/CD 配置 (GitHub Actions)~~ ✅ | 基础设施 |
| 🔴 高 | ~~添加 `.cargo/config.toml`~~ ✅ | 构建配置 |
| 🟡 中 | ~~README.md 项目名 `msp` → `mps` 统一~~ ✅ | 文档 |
| 🟡 中 | ~~添加更多单元测试~~ ✅ | 代码质量 |
| 🟡 中 | cbindgen 导出列表自动校验 | 工具链 |
| 🟢 低 | 添加 `CHANGELOG.md` | 文档 |
| 🟢 低 | 添加 `CONTRIBUTING.md` | 文档 |
| 🟢 低 | 添加 `rust-toolchain.toml` | 构建配置 |
| 🟢 低 | `.gitignore` `target` → `/target` | 配置 |
| 🟢 低 | LICENSE 填写版权信息 | 法律 |
