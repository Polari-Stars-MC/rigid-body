# 0032 — CIE 加权积分替代三离散波长

## 背景

[ADR-0020](0020-spectral-tyndall.md) 用**三个离散波长**（650/550/450 nm）代表
RGB 通道：

```text
σ_sca_r = σ(650 nm)    σ_sca_g = σ(550 nm)    σ_sca_b = σ(450 nm)
```

这是渲染里的常见简化，但它有两个物理问题：

1. **三点采样漏掉米氏区的振荡结构**。`Q_sca(x)` 在 `x ~ 1` 附近有干涉振荡，
   周期约 `Δx ≈ 1`。650/550/450 nm 之间的间距太大，采样会跳过整段振荡。
2. **没有色度学加权**。真实颜色是 `∫ L(λ)·x̄(λ) dλ`，三个点无法做这个积分。

[ADR-0030](0030-spectral-from-mie.md) 之后，逐波长的米氏解已经便宜到可以算
几十个采样点，所以这个简化不再必要。

## 决定

新增 CIE 1931 色匹配函数与光谱积分，替代三离散波长。

### 1. 色匹配函数用解析拟合，不用查找表

```rust
pub fn cie_color_matching(lambda_nm: f64) -> Option<(f64, f64, f64)>
```

用 **Wyman, Sloan, Shirley (JCGT 2013)** 的多瓣高斯拟合：

```text
x̄(λ) = 1.056 g(λ;599.8,37.9,31.0) + 0.362 g(λ;442.0,16.0,26.7) − 0.065 g(λ;501.1,20.4,26.2)
ȳ(λ) = 0.821 g(λ;568.8,46.9,40.5) + 0.286 g(λ;530.9,16.3,31.1)
z̄(λ) = 1.217 g(λ;437.0,11.8,36.0) + 0.681 g(λ;459.0,26.0,13.8)
```

`g` 是分段高斯：`λ < μ` 用 `σ₁`，否则用 `σ₂`。

**为什么不嵌 CIE 官方表（471 项）**：我没有可靠的逐项记忆，编造一张表比用有
文献出处的解析拟合更糟。拟合误差约峰值的 1%，对渲染足够；需要色度学级精度时
应该换成官方表。

### 2. XYZ → 线性 sRGB

```rust
pub fn xyz_to_linear_srgb(x, y, z) -> Option<(f64, f64, f64)>
```

IEC 61966-2-1 矩阵。**钳位是逐通道的**：负的 `X` 只把 `r` 钳到 0，
`g = −0.9689·X` 仍为正 —— 这点我第一版测试写错了。

### 3. 光谱积分

```rust
pub fn spectral_samples_to_rgb(samples: &[(f64, f64)]) -> Option<SpectralLuminance>
```

梯形积分 `X = ∫L·x̄ dλ` 等，然后按 `∫ȳ dλ` 归一化，使"平坦光谱 = 单位亮度"。
调用方不用关心绝对标度。

### 4. 端到端入口

```rust
pub fn mie_spectral_samples(diameter, n, kappa, samples)
    -> Option<Vec<(f64, f64, f64)>>        // (λ_nm, σ_sca, σ_ext)

pub fn inscatter_rgb_uniform(diameter, n, kappa, number_density,
                             phase, light_intensity, ray_length, steps, samples)
    -> Option<SpectralLuminance>
```

## 一个关键的实现顺序

**必须先逐波长算 `L(λ)`，再对 `L` 做线性积分。**

不能先对 `σ` 积分再算一次散射 —— 散射亮度对 `σ` 是**非线性**的
（`1 − exp(−τ)` 与 Beer–Lambert 累积），先积分会得到错误的量级。

这是辐射度学的基本要求，写进了函数文档。

## 实测

`mie_spectral_samples(1e-6, 1.33, 0, 15)` 覆盖 380~780 nm 等间距 15 点。

CIE 拟合的峰值对照（CIE 1931 2° 表）：

| λ (nm) | 表值 | 拟合 | 误差 |
| ---: | ---: | ---: | ---: |
| 600 (`x̄`) | 1.0622 | ≈1.054 | 0.8% |
| 555 (`ȳ`) | 1.0000 | ≈1.0 | <0.1% |
| 445 (`z̄`) | 1.7826 | 1.7764 | 0.35% |

D65 白点 `(0.95047, 1.0, 1.08883)` → 线性 sRGB `(1,1,1)`，误差 <1%。

## 新增的 11 项测试

| 测试 | 断言 |
| --- | --- |
| `cie_color_matching_matches_known_peaks` | 三个峰值对上 CIE 表（3~5% 容差） |
| `cie_color_matching_decays_outside_the_visible_range` | 320/820 nm 接近零 |
| `xyz_to_linear_srgb_maps_d65_white_to_unity` | D65 → (1,1,1)，逐通道钳位 |
| `flat_spectrum_is_normalised_to_unit_luminance` | 等能白亮度 = 1 |
| `narrow_bands_produce_the_expected_dominant_channel` | 蓝/绿/红窄带的主导通道 |
| `spectral_samples_to_rgb_rejects_invalid` | 采样不足、降序、负值、NaN |
| `mie_spectral_samples_covers_the_visible_range` | 380~780 升序 |
| `mie_spectral_samples_rejects_invalid` | 参数校验 |
| `optically_thin_fine_smoke_is_blue` | 10 nm 颗粒 → 蓝光柱 |
| `large_particles_give_a_neutral_beam` | 5 µm 颗粒 → 近中性 |
| `inscatter_rgb_uniform_rejects_invalid` | 参数校验 |

## 一个发现：`inscatter_along_ray` 的厚介质下溢

写测试时发现 `n = 1e13` 的光柱亮度是 `1e-12` 而不是 `≈1`。

根因不在 CIE，而在 [ADR-0018](0018-tyndall-effect-data-layer.md) 的
`inscatter_along_ray`：它用中点法，`τ_step > ~700` 时 `exp(−τ_step)` 下溢到 0，
`sqrt(0) = 0`，整条视线亮度被**静默算成 0**。

已在 [ADR-0033](0033-per-step-exact-integration.md) 修复（每步解析积分）。

## 后果

- 颜色来自**连续光谱的 CIE 积分**，不是三个点的近似
- 米氏区的振荡结构被正确积分进去
- 与三离散波长版本并存：`SpectralOptics`（三点）仍保留给需要 `O(1)` 的场合

## 没做的事

1. **CIE 官方表**：用解析拟合替代，误差 ~1%
2. **gamma 编码**：输出是**线性** sRGB，显示端还要 `v^(1/2.2)`
3. **多观察者**：只有 CIE 1931 2°（没有 10° 观察者或 CIE 1964）
4. **色适应**：没有 von Kries 变换，假设 D65 白点

## 相关代码

- `crates/mps-formula/src/tyndall_spectral.rs`（`cie_lobe`、`cie_color_matching`、
  `xyz_to_linear_srgb`、`spectral_samples_to_rgb`、`mie_spectral_samples`、
  `inscatter_rgb_uniform`）
- `crates/mps-test/src/rapier/tyndall_spectral.rs`（11 项新测试）
