# 0008 — rapier fork 固定在 python-v0.36.1 基线，同步走独立变基分支

## 背景

`rapier/` 从上游 `python-v0.36.1` tag 分叉，当前领先 72 个提交（force_container 确定性修复、soft_body fork_extras、granular、SPH 修复 d2137c84 等）。上游 Rapier 迭代快、且 python 绑定 tag（`python-v*`）是**f64 默认构建**的稳定线，与 ADR-0001 的 f64 承诺同源。有两次外部审查建议"跟进上游最新版"；但 fork 提交从未按上游 main 重放过，直接 rebase 会把 72 个补丁暴露给 0.3x 之间未审计的语义漂移（solver 内部接口、collider 事件时序都已变过）。

## 决定

1. **基线固定**：`rapier/` 的合并基线永远是某个 `python-vX.Y.Z` tag。不允许从上游 `main`/`master` 取提交。
2. **同步节奏**：跟随 python 绑定的稳定 tag 线（即上游发 `python-v0.37.0` 且 mps 侧完成下述清单后才升），**不**按上游每个 minor 版本跟进。
3. **同步流程**（一次同步 = 一个独立变更，不夹带 mps 功能提交，呼应 ADR-0007）：
   - 在 `rapier/` 里建 `sync/v0.37` 分支，`git rebase --onto python-v0.37.0 python-v0.36.1 fork-main`；
   - 逐个重放 fork 补丁，凡与上游新实现冲突的补丁，先在上游 issue 区确认语义再改写；
   - 跑 `cargo test`（rapier 自己的 workspace）+ 回根仓库跑 `cargo test -p mps-test`（确定性与 arena 兼容守门：`scenario_bit_identical_determinism_gate`、`arena_compat`、`verify_metrics_sync`）；
   - 升级在 ADR 追加一节记录（from/to tag、重放的补丁数、放弃/改写的补丁及原因）。
4. **补丁纪律**：fork 新增提交必须能被一句话描述并进 ADR 记录；禁止在 fork 里做只服务 mps 上层的 hack——那类改动放 `mps-core`/`mps-cosmos` 包装层（fork 只改 rapier 自身的缺陷或缺失能力）。
5. **依赖冻结**：fork 内 Cargo 依赖升级（如 indexmap、simba）随同步一起做，不单独升级，避免锁文件漂移不可归因。

## 后果

- "要不要升上游"变成一个有清单、有记录的动作，而不是看到上游 release 就想 rebase。
- fork 补丁数是同步成本的一次性度量；补丁纪律保证它不无限膨胀。
- 短期内放弃上游新特性（如新 solver 模式）；`mps` 需要时优先在包装层实现。
- 依赖版本落后上游期间，安全公告（cargo audit 报的传递依赖）按"是否影响 fork 使用面"评估，不自动升级。

## 相关代码

- `rapier/`（独立 workspace，合并基线 `python-v0.36.1`）
- 根 `Cargo.toml` 的 `rapier3d` path 依赖与 `enhanced-determinism` feature
- ADR-0001（f64）、ADR-0007（独立 workspace）
