# 0030 — 由精确米氏算光谱光学参数

## 背景

[ADR-0020](0020-spectral-tyndall.md) 的 `SpectralOptics::from_monochrome` 用**幂律**
生成三通道截面：

```text
σ(λ) = σ_ref · (λ/λ_ref)^(-n)
```

`n` 要宿主手调（ADr-0020 建议丁达尔用 `1.0~1.5`）。这个模型的局限：

1. **幂律只在瑞利区严格成立**（那里 `n = 4`）
2. 米氏区（`x ~ 1`）的波长依赖**不是幂律** —— 有振荡（干涉结构）
3. `n` 没有物理含义，纯粹是拟合参数

[ADR-0026](0026-exact-mie-integration.md) / [ADR-0027](0027-complex-refractive-index.md) /
[ADR-0029](0029-logarithmic-derivative.md) 之后，精确米氏已经可用且全区间验证。
这一条把光谱链条的最后一次近似去掉。

## 决定

新增 `SpectralOptics::from_mie` + C ABI `tyndall_spectral_optics_from_mie`。

### 计算链

```text
直径 d、折射率 (n, κ)、密度 ρ
  ↓ x = π·d/λ                每个通道各自的尺寸参数
  ↓ 精确米氏                  Q_sca(λ), Q_ext(λ)
  ↓ × 几何截面 πd²/4         σ_sca(λ), σ_ext(λ)
  ↓ 质量 = ρ·πd³/6
SpectralOptics
```

**输入是材料参数，不是拟合系数。** 宿主不需要知道 `n` 该取多少。

### 与 `from_monochrome` 的分工

| | `from_monochrome` | `from_mie` |
| --- | --- | --- |
| 输入 | `σ` 参考值 + 波长指数 | 直径 + 复折射率 + 密度 |
| 波长依赖 | 幂律拟合 | **逐通道精确米氏** |
| 手动调参 | 需要 | **不需要** |
| 代价 | `O(1)` | 3 × 米氏级数 |
| 适用 | 简单场景、只要一个可调的白色光柱 | 要物理正确的颜色 |

两个都保留 —— 前者在"我只想要个能调的光柱"时更省事。

## 物理行为验证（9 项测试）

| 测试 | 断言 | 结果 |
| --- | --- | --- |
| `tiny_particles_scatter_blue_far_more_than_red` | 10 nm 水滴 `σ_b/σ_r ≈ (650/450)⁴ = 4.35` | ✅ 15% 容差 |
| `large_particles_are_nearly_colour_neutral` | 5 µm 水滴 `σ_b/σ_r ≈ 1` | ✅ 35% 容差 |
| `transparent_particles_have_no_absorption` | `κ=0` 时 `σ_ext == σ_sca` | ✅ 1e-10 |
| `absorbing_particles_lose_blue_light` | 烟灰消光更大、反照率更低 | ✅ |
| `mass_follows_the_sphere_formula` | `mass = ρ·πd³/6` | ✅ 1e-12 |
| `from_mie_respects_the_structural_constraints` | `σ >= 0`、`σ_ext >= σ_sca` | ✅ |
| **`rayleigh_regime_recovers_the_power_law_exponent`** | 反解幂律指数应 ≈ 4 | ✅ 0.5 内 |
| `from_mie_rejects_invalid` | 参数校验 | ✅ |
| `water_and_soot_beams_differ_as_expected` | 水滴亮、烟灰暗 | ✅ |

第 7 条是**交叉验证**：从 `from_mie` 的结果反解 `σ ∝ λ^(-n)` 的指数，
在瑞利区应回到 4。它证明两种表示在瑞利极限一致，差异只在米氏区。

### 尺寸决定颜色（实测）

| 直径 | `x` | 光柱颜色 |
| ---: | ---: | --- |
| 10 nm | ~0.06 | 强烈偏蓝（指数 ≈4） |
| 100 nm | ~0.6 | 偏蓝 |
| 1 µm | ~6 | 近中性 |
| 5 µm+ | ~28+ | 中性 |

这是丁达尔光柱"白"还是"蓝"的物理根源 —— 不是美术参数。

## 新增的 C ABI

```c
struct Bool tyndall_spectral_optics_from_mie(
    double diameter, double refractive_index_real, double refractive_index_imag,
    double density,
    double *out_sca_r, double *out_sca_g, double *out_sca_b,
    double *out_ext_r, double *out_ext_g, double *out_ext_b);
```

六个输出指针**全部成功才写**。

`tyndall_*` 系列现在共 **29 个入口**。

## 一个浮点陷阱

`SpectralOptics::is_valid()` 要求每个通道 `σ_ext >= σ_sca`。透明颗粒时两者在
数学上相等，但 `Q_ext − Q_sca` 的浮点残差可能让 `σ_sca` 略大于 `σ_ext`（`1e-17`）。

结果：**5 个测试随机失败**，`from_mie` 返回 `None`。

修法是在构造时钳住：

```rust
let sca = (q_sca * geom).max(0.0);
let ext = (q_ext * geom).max(sca);      // 保证 ext >= sca
```

这与 [ADR-0027](0027-complex-refractive-index.md) 的 `q_abs.max(0.0)`、
[ADR-0029](0029-logarithmic-derivative.md) 的断言放宽是同一类问题：
**结构的数学不变式在浮点下需要显式维护。**

## 相关文档更新

`docs/tyndall-mc-integration.md` 之前的状态是过时的：

* 说"米氏散射用渐近式，10~20% 误差" → 已有精确解
* 没有任何"用材料参数算 `sigma`"的指引 → 新增一节

现在指南里有材料参数表和尺寸-颜色对照表，宿主可以直接查表。

## 后果

- 光谱链条上**最后一次近似去掉**：波长依赖现在来自物理，不是拟合
- 宿主可以从"水滴 1 µm"这种描述直接得到正确的光柱颜色
- 米氏区的振荡结构（幂律拟合不出来的部分）自动包含

## 没做的事

1. **多波长批量**：三个通道分三次调米氏，可以做一次批量求解（截断阶数共享）。
   当前每次调用 3 次级数求和，性能可接受但非最优
2. **`x > 200`**：`MAX_MIE_ORDER = 200` 限制。对应水滴直径 > 17 µm（@550nm）
3. **反照率加权的最优波长**：用 `RED_NM=650` 等固定中心波长，
   更好的做法是按 CIE 匹配函数做加权积分

## 相关代码

- `crates/mps-formula/src/tyndall_spectral.rs`（`SpectralOptics::from_mie`）
- `crates/mps-core/src/rapier/emag.rs`（1 个 C ABI）
- `crates/mps-test/src/rapier/tyndall_spectral.rs`（9 项新测试）
- `docs/tyndall-mc-integration.md`（材料参数表、尺寸-颜色表）

## 记录来源

与实现同批落地。测试两轮通过：第一轮 5 项因浮点钳位失败，修后在 35 项
`tyndall_spectral` 上全绿。
