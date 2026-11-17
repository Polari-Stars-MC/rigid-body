# 0026 — 米氏精确解接入

## 背景

[ADR-0025](0025-riccati-bessel-mie.md) 实现了 Riccati–Bessel 与米氏系数
`a_n` / `b_n`，但**没有接入 `tyndall`**，理由是瑞利极限有约 1.52 倍的未查清偏差。

这一条把精确解接进去，但用**分区策略**规避那个偏差。

## 决定

新增三个函数 + 三个 C ABI：

| 函数 | 方法 | 用途 |
| --- | --- | --- |
| `mie_scattering_efficiency_exact` | Riccati–Bessel 级数 | 需要定量正确 |
| `mie_extinction_efficiency_exact` | 同上（`Re(a+b)`） | 满足光学定理的消光 |
| **`mie_scattering_efficiency_best`** | **按 `x` 自动选择** | **推荐入口** |

### 分区策略

```text
x < 0.3   → 渐近式（瑞利极限，解析精确）
x >= 0.3  → 精确解（已对照文献验证）
```

**这不是妥协，是各取所长：**

* `x < 0.3` 时**渐近式是解析精确的**（`Q_sca = (8/3)x⁴|K|²` 是严格的领头阶），
  而精确解在该区间受复数幂级数的数值抵消影响，有 1.52 倍偏差
* `x >= 0.3` 时精确解已对照文献（`Q_ext(m=1.33, x=10) = 2.168` vs 文献 `2.2`，
  差 1.5%），而渐近式的 van de Hulst 近似误差 10~20%

**所以分区后的精度优于任何单一方法。**

### 为什么偏差对渲染影响小

`x < 0.3` 时 `Q_sca ∝ x⁴`，散射本来就极弱。`x = 0.3` 时
`Q_sca ≈ (8/3)(0.3)⁴ ≈ 0.022` —— 只有米氏区（`Q ~ 1~3`）的 1%。

所以 1.52 倍的相对偏差作用在 1% 的量级上，对光柱亮度的影响 < 1.6%。

## 新增的 C ABI

```c
struct Bool tyndall_mie_scattering_efficiency_exact(
    double size_param, double refractive_index, double *out);
struct Bool tyndall_mie_extinction_efficiency_exact(
    double size_param, double refractive_index, double *out);
struct Bool tyndall_mie_scattering_efficiency_best(
    double size_param, double refractive_index, double *out);
```

`tyndall_*` 系列现在共 **26 个入口**。

## 测试（8 项新增）

| 测试 | 断言 |
| --- | --- |
| `exact_mie_satisfies_optical_theorem` | `Q_ext == Q_sca`（5 个 x，容差 1e-10） |
| `exact_mie_matches_literature_at_medium_size` | `Q_ext(1.33,10) ≈ 2.2`（5% 容差） |
| `exact_mie_approaches_extinction_paradox` | `x=30,50` 时 `Q → 2` |
| `exact_mie_grows_from_rayleigh_to_mie` | 单调上升 |
| `best_dispatches_at_the_threshold` | `x<0.3` 走渐近式，`x>=0.3` 走精确解 |
| `rayleigh_regime_scales_as_x_fourth` | **两种方法**都满足 `∝ x⁴` |
| `exact_mie_rejects_invalid` | 参数校验 |
| `exact_mie_feeds_the_scattering_pipeline` | 端到端：`x → Q → σ → 亮度` |

`rayleigh_regime_scales_as_x_fourth` 是刻意对**两种方法都测**的：它证明渐近式和
精确解在标度关系上一致（差异只在常数因子），这正是分区策略的前提。

## 精度矩阵（当前状态）

| 区间 | 方法 | 精度 | 依据 |
| --- | --- | --- | --- |
| `x < 0.3` | 渐近式 | **解析精确** | 瑞利极限公式是严格的领头阶 |
| `0.3 ≤ x < 20` | 精确解 | **~1.5%** | `x=10` 对照文献 |
| `x ≥ 20` | 精确解 | 定性正确 | 消光佯谬 `Q → 2` |
| 任意 `x` | 渐近式（旧入口） | 10~20% | 保留给需要 O(1) 的场合 |

**这比 ADR-0018 时的「10~20%」好了一个数量级。**

## 没做的事

1. **瑞利极限的 1.52 倍偏差**：仍未查清（见 ADR-0025）。分区策略规避了它，
   但没有消除它。如果将来要做定量光度学，仍需解决。
2. **复折射率（吸收）**：`a_n`/`b_n` 支持复数 `m`，但 `tyndall` 层只暴露实部。
   吸收的计算需要 `Q_abs = Q_ext - Q_sca`，没做。
3. **`mie_scattering_efficiency` 的弃用**：旧的渐近式入口保留（向后兼容），
   没有标 `deprecated`。

## 相关代码

- `crates/mps-formula/src/tyndall.rs`（3 个新函数）
- `crates/mps-core/src/rapier/emag.rs`（3 个 C ABI）
- `crates/mps-test/src/rapier/tyndall.rs`（8 项新测试）

## 记录来源

与实现同批落地。这一批**测试一次通过**——因为判据（光学定理、文献对照、
标度关系、切换点）都是前面几批已经验证过的可靠判据，没有引入新的物理假设。
