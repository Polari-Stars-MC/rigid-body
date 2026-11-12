# 跨引擎性能对比（mps ↔ Jolt）

## 协议

- **mps 侧**：`cargo test -p mps-test --release -- --ignored engine_benchmark_vs_jolt_protocol --test-threads=1 --nocapture`
  （可调 `MPS_COMPARE_SIDE`（默认 40 → 1600 立方体）、`MPS_COMPARE_STEPS`（默认 600））。
  输出 `target/engine-compare-mps.csv`：mean/p50/p95 每步毫秒，10 轮取分位数，预热 60 步不计入。
- **Jolt 侧**：JoltPhysics 官方 Testbed `PerformanceTest`（`Testbed/Tests/PerformanceTest.cpp`），
  或自建同负载场景（N×N 立方体堆 + 静态地面，solver 12 迭代，sleeping 开启，60 Hz）。
  **必须同负载、同迭代数、同休眠策略**——Jolt 默认迭代（12）与 mps 的 `world_apply_runtime_settings(12,…)` 对齐。
- **可比性条件**：同一台机器、同一次提交、同 CPU 频率策略（关 boost 差异需记录）。
- 场景选立方体堆叠而非无限反弹球对：堆叠有休眠收益，是 MC 服务器真实负载形态；
  球对矩阵（`world_step_performance_matrix`）保留作 mps 自身的回归面。

## 对照表

> 在本机跑两侧命令后填入。不要把别人的机器数字抄进来。

| 日期 | mps commit | CPU | 场景 | mps p50 ms/step | Jolt p50 ms/step | 比值 |
| --- | --- | --- | --- | --- | --- | --- |
| _待填_ | | | 1600 cubes, 12 iters, sleeping | | | |
| 2026-11-12 | `edcc8c8` | i7-8750H @ 2.20 GHz（12 线程，boost 默认） | 1600 cubes (40×40), 12 iters, sleeping, 600 steps | 13.608 | _待测_ | — |

## 采集记录

### 2026-11-12 mps 侧首采

- commit：`edcc8c8`（main）
- 命令：`cargo test -p mps-test --release --lib -- --ignored engine_benchmark_vs_jolt_protocol --test-threads=1 --nocapture`
- 环境：Windows 11，i7-8750H（笔记本，无频率锁定——p50 跨 10 轮稳定在 13.6 ms，p95 24 ms 反映 boost 波动）。
- 原始 CSV：`target/engine-compare-mps.csv`
  `mps,stacked_cubes,1600,600,14.8910,13.6082,24.2184,12`
- 说明：sleeping 收敛后 p50 ≈ 13.6 ms/step（1600 体）。Jolt 侧待同机采集后填入对照表；同机对比前不得跨机器引用他人数字。
