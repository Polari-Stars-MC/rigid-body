# 0033 — 每步解析积分替代中点法

## 背景

[ADR-0018](0018-tyndall-effect-data-layer.md) 的 `inscatter_along_ray` 把视线分成
`steps` 段，每段用**中点法**累积：

```rust
let scattered = sigma_sca * number_density * phase * light_intensity;
let tau_per_step = sigma_ext * number_density * ds;
let trans_per_step = (-tau_per_step).exp();
let mut trans = trans_per_step.sqrt();   // k = 0 时的 ^0.5
for _ in 0..steps {
    sum += trans * scattered * ds;
    trans *= trans_per_step;
}
```

这个实现有两个问题，都是写 [ADR-0032](0032-cie-weighted-integration.md) 的测试时
才暴露的。

## 问题 1：厚介质下溢，亮度静默归零

`τ_step > ~700` 时 `exp(−τ_step)` 下溢到 `0.0`，于是 `trans_per_step.sqrt() = 0`，
整个循环 `sum` 恒为 0。

**实测**：5 µm 水滴、`n = 1e13`、`L = 10 m`（`τ ≈ 4000`）
→ 旧实现返回 `1e-12`，正确答案应该是 `≈1`（饱和在反照率）。

这是**静默的**错误：没有 `None`，没有 `NaN`，就是一个错得离谱的小数。
渲染层看到的是"雾越浓光柱越黑"，视觉上完全反了。

## 问题 2：中点法只是近似

单步内密度均匀时，该段贡献有**闭式解**：

```text
∫₀^ds T(s)·σ_sca·n·p·I ds = T_step_start · (σ_sca/σ_ext)·p·I · (1 − e^(−τ_step))
```

中点法 `trans^(k+0.5)·σ_sca·n·p·I·ds` 只是它的一阶近似。

## 决定

改用**每步解析积分**：

```rust
let albedo = sigma_sca / sigma_ext;
let source = albedo * phase * light_intensity;
let absorbed_per_step = 1.0 - trans_per_step;
let mut trans = 1.0;
let mut sum = 0.0;
for _ in 0..steps {
    sum += trans * source * absorbed_per_step;
    trans *= trans_per_step;
}
```

## 为什么这样就修好了

关键在 `(1 − e^(−τ_step))` 的**行为**：

| `τ_step` | `e^(−τ_step)` | `1 − e^(−τ_step)` |
| ---: | ---: | ---: |
| `0.001` | `0.999` | `0.001` |
| `1` | `0.368` | `0.632` |
| `700` | `1e-304` | `1.0` |
| `4000` | `0.0`（下溢） | **`1.0`** |

**下溢时 `1 − 0 = 1`，是正确答案。** 旧实现下溢时 `sqrt(0) = 0`，是错误答案。
同一处下溢，一个无害一个致命。

## 顺带的收益

新写法对**分段均匀密度是精确的**，所以结果不再依赖 `steps`：

```
steps = 8    →  与 steps = 512 一致到 1e-9
```

旧的中点法在 `steps` 少时有可见误差。现在 `steps` 只影响"密度空间变化的
解析度"，不影响积分精度 —— 语义更清晰。

## 新增的 5 项测试

| 测试 | 断言 |
| --- | --- |
| `thin_medium_matches_the_analytic_formula` | 光学薄时 `L = σ_sca·n·p·I·L`（2% 容差） |
| **`extremely_thick_medium_saturates_at_albedo`** | `τ = 2e11` 时 `L = σ_sca/σ_ext`（1% 容差），且 `> 0` |
| `brightness_grows_monotonically_with_thickness` | `n` 从 1 到 1e12 单调不减，最终饱和于反照率 |
| `pure_absorber_has_no_scattered_light` | `σ_sca = 0` 时 `L = 0` |
| `result_is_insensitive_to_step_count_for_uniform_media` | `steps = 8` 与 `512` 一致到 1e-9 |

第二条是核心回归测试 —— 它正是旧实现失败的场景。

## 影响面

`inscatter_along_ray` 是底层函数，改动会传导到：

- `inscatter_along_ray_multiple`（[ADR-0022](0022-multiple-scattering.md) 的多次散射）
- `tyndall_visibility`（[ADR-0018](0018-tyndall-effect-data-layer.md) 的可见度评分）
- `tyndall_inscatter_from_world` / `..._accelerated`（[ADR-0019](0019-particle-driven-volumetric-scattering.md) / [0023](0023-uniform-grid-acceleration.md)）
- `inscatter_rgb_uniform`（[ADR-0032](0032-cie-weighted-integration.md)）

**全部 1574 项测试在改动后仍通过**，说明没有破坏既有行为（差异在容差内）。

## 后果

- 厚介质的光柱亮度不再被静默算成 0
- 结果对 `steps` 不敏感（均匀介质下精确）
- 数值上更稳定（无下溢敏感点）

## 没做的事

1. **非均匀密度的精确积分**：仍然假设每步内密度均匀（这是 SPH 采样的模型，
   不是缺陷）
2. **自适应步长**：密度剧变时 `steps` 仍要足够大，没有自动细分

## 相关代码

- `crates/mps-formula/src/tyndall.rs`（`inscatter_along_ray`）
- `crates/mps-test/src/rapier/tyndall.rs`（5 项新测试）

## 记录来源

这个 bug 存在了 15 条 ADR（0018 → 0032）才被发现。原因是：**所有既有测试都在
光学薄到中等厚度的区间**，`τ_step` 从未超过 700。写 CIE 端到端测试时用了
`n = 1e13`（因为 5 µm 颗粒的截面小，需要高数密度才可见），才踩到。

**教训：物理函数的测试要覆盖参数的完整量级范围，不只是"典型值"。**
