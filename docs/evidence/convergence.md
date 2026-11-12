# 自适应子步收敛性证据（A2）

> 复现：`cargo test -p mps-test --release --lib -- tolerance_scan --ignored --nocapture`
> 采集日期：2026-11-12（release 模式）。

## 1. tolerance 扫描（tolerance_scan_buys_drift_monotonically）

高椭圆轨道，近心点 0.9·R⊕（5.74e6 m，触发近心点加密区），远心点 4.2e7 m，
Verlet + `verlet_substeps=2`，dt = 1/60 s，20 000 步：

| adaptive_tolerance | 近心点最大子步数 | 20k 步相对能量漂移 |
| --- | --- | --- |
| 1e-6  | 2  | 6.240e-11 |
| 1e-9  | 2  | 6.240e-11 |
| 1e-12 | **10** | **3.771e-12** |

结论：**tolerance 收紧 6 个量级 → 子步 ×5 → 漂移改善 16.5×**。
「更严的 tolerance 买到更小的漂移」这条承诺成立，且单调性由断言钉死
（`substeps not decrease` / `drift not increase`）。

## 2. 误差代理单调性（adaptive_substep_count_is_monotone_in_proximity）

| 体位置 | 子步数 |
| --- | --- |
| 4.2e8（远心点，r ≥ 2·R⊕） | 2（= base，不加密） |
| 1.5·R⊕ | < 近心点值 |
| 0.6·R⊕（天体表面下） | ≥ 1.5·R⊕ 值，≤ 64 上限 |

## 3. 闭环守恒回归钉（adaptive_periapsis_energy_conservation）

近心点 0.9·R⊕，tol=1e-9，5 000 步：相对能量漂移 < 1e-9（非 ignore，
每轮全量测试跑）。

## 4. 修复记录（本轮 A2）

- `adaptive_substep_count` 由 private 改 pub（观测接口，宿主诊断需要）。
- **修复加密失效缺陷**：旧实现 `substeps = ceil(base · scale)`——控制器给出
  `scale = dt_next/dt`，误差**超**容忍时 scale < 1，ceil(base·scale) ≤ base，
  子步被压回基准值，近心点加密从未生效（tol 扫描三种值行为完全一致暴露了
  这一点）。新实现：`scale < 1` 时 `substeps = ceil(base / scale)`（用切分
  实现控制器想要的更小步长），`scale ≥ 1` 不放大。修复后 tol=1e-12 子步
  2→10，漂移 6.24e-11 → 3.77e-12。
- 语义注意：`central_body` 必须用 `set_central_body(Some(&body))` 设置；
  `add_celestial` 不设置中央天体（自适应与大气阻力都依赖它）。
