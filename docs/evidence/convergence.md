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

## 5. 天体自转表面平衡（C3，celestial_rotation）

> 复现：`cargo test -p mps-test --lib -- celestial_rotation --nocapture`

| 断言 | 期望 | 实测 |
| --- | --- | --- |
| 地表共转线速度（ω×R） | 465.1 m/s ± 1 | ✓ |
| 离心/表面引力比 | 3.46e-3 ± 1e-4 | ✓ |
| 低轨落体向东偏转（科氏项，含大气阻力） | v.y > 0 且 v.x < 0 | ✓ |

## 6. SOI（central_body 切换）影响面（C2）

清点结论（`integrator.rs` 逐条核对）已钉成契约测试
（`central_body_switch.rs`，非 ignore，每轮全量跑）：

| 通道 | 是否依赖 central_body | 测试 |
| --- | --- | --- |
| 基础引力（每源独立累加，integrator.rs:290） | 否 | 切换前后轨迹逐位一致 |
| 相对论修正叠加对象（integrator.rs:293，按 ptr 判定） | 是 | 1PN 开启时切换产生 cm 级差异 |
| 大气阻力参考（integrator.rs:329） | 是 | （C3 落体测试覆盖路径） |

## 7. 双世界一致性（E2）与录制重演（D2）

见 `numeric-drift.md` §5/§6（E2 跨世界抽查 3 断言、D2 位级重演
20/600 步逐位一致）。
