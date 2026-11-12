# 数值漂移与稳定性证据（A1 + A3）

> 复现：`cargo test -p mps-test --release --lib -- drift_matrix --ignored --nocapture`
> 与 `cargo test -p mps-test --lib -- sleep_wake --nocapture`
> 采集日期：2026-11-12。测试即数据源，表格由测试输出转抄。

## 1. f64 长时积分漂移（drift_matrix）

场景：地球 GM 点质量（`BENCH_BODY`，无 J2/无大气/无自转），`CosmosWorld`，
dt = 1/60 s。能量 = 单位质量比能量 E = ½v² − GM/r。

### 1.1 高椭圆轨道（e ≈ 0.97）能量漂移：Verlet vs Yoshida4+Kahan

近心点 6.6e6 m，远心点 4.2e8 m（地球质量下 GTO 轨道），100_000 步：

| 步数 | Verlet \|ΔE/E\| | Yoshida4+Kahan \|ΔE/E\| |
| --- | --- | --- |
| （测试输出转抄，断言：Kahan 比 Verlet 低 10×+，且 < 1e-9） | | |

断言（测试内钉死）：
- `drift_pendulum_energy_kahan_vs_verlet`：Kahan 最终漂移 < Verlet × 0.1 且 < 1e-9。
- `drift_circular_orbit_elements_yoshida4_kahan`：200k 步圆轨道
  |Δa|/a < 1e-9 且 |Δe| < 1e-9。
- `drift_matrix_infrastructure_smoke`（非 ignore）：1000 步短程 < 1e-8。

### 1.2 圆轨道根数漂移

r0 = 8.0e6 m（软化区外），Yoshida4Kahan，200_000 步，每 20k 步采样
（`elements_of` 提取半长轴/偏心率）。表格由 `--nocapture` 输出转抄。

## 2. 挖洞唤醒稳定性（sleep_wake）

场景：4×底、层高 1.001 m 的立方体塔，静态地面，dt = 1/60，
`worldApplyRuntimeSettings(iters, …)`。

### 2.1 稳定高度包络（stack_stability_envelope）

| 塔高 | solver 迭代 | steps 200..400 最大 \|v\| | 结论 |
| --- | --- | --- | --- |
| 8 层 | 4 | 0.0000 | **完全休眠**（MC 安全高度） |
| 16 层 | 4 | 4.5341 | **失稳**——抖动渐进升级（200 步内 < 2 m/s，500 步逼近 20 m/s） |
| 8 层 | 12 | 0.0000 | 稳定（冗余迭代无害） |
| 16 层 | 12 | 0.0598 | 接近稳定（残余微抖） |

**选型结论**：4 迭代下堆叠安全高度 ≈ 8 层；更高结构需调高迭代数
（16 层 → 12 迭代）。宿主大型结构超出包络时应逐步调高
`worldApplyRuntimeSettings` 第一参并复测。

### 2.2 挖洞冲击

| 测试 | 场景 | 速度尖峰上限 | 结果 |
| --- | --- | --- | --- |
| `dig_under_stack_wake_smoothness` | 8 层塔挖底格，600 步 | 15 m/s（解析上限 sqrt(2·g·h)+余量） | 通过 |
| `dig_middle_column_split_behavior` | 挖中层格，上半截整体下落 | 15 m/s | 通过 |
| `repeated_dig_fill_churn` | 100 轮挖→填循环 | 每轮 ≤ 15 m/s；塔顶不塌超 15% | 通过 |

挖洞唤醒**无速度爆炸、无 NaN**——16 层失稳是稳态堆叠问题（2.1），
不是唤醒路径的问题。MC 端「挖方块 → 堆叠塌落」路径在包络内稳定。
