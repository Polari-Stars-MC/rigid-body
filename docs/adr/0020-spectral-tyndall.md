# 0020 — 丁达尔效应的光谱版本（RGB 分通道）

## 背景

[ADR-0018](0018-tyndall-effect-data-layer.md) / [ADR-0019](0019-particle-driven-volumetric-scattering.md)
把丁达尔效应算成了**标量亮度**。

标量版本只能让光柱"变暗"——穿过厚烟雾后整体变灰。但真实光柱会**变暖**：
蓝光被散射掉得更多，剩下的是偏红的光。这与日落偏红是同一个机制。

## 决定

新增 `mps_formula::tyndall_spectral` + `mps_core` 里的 4 个 C ABI。

### 数据结构

```rust
pub struct SpectralOptics {
    pub sigma_sca_r: f64, pub sigma_sca_g: f64, pub sigma_sca_b: f64,
    pub sigma_ext_r: f64, pub sigma_ext_g: f64, pub sigma_ext_b: f64,
    pub mass: f64,
}

pub struct SpectralLuminance { pub r: f64, pub g: f64, pub b: f64 }
```

每个通道独立持有散射/消光截面，因为**消光依赖波长**。

### 波长 → RGB

用 Dan Bruton 的经典分段线性近似，把 380~780 nm 分成六段。

**不追求色度学精度**：真正的 CIE 1931 匹配需要三张 471 项查找表，收益只是让
深紫和深红更准——那两个区域在光柱里本来就不显著。这是工程判断。

可见光范围外返回 `(0,0,0)` 而不是错误：紫外线不产生视觉，是合法输入。

## 一个被测试抓出来的设计错误（重要）

第一版 `from_monochrome` 对 `sigma_sca` 和 `sigma_ext` 用了**同一个**波长指数：

```text
σ_sca(λ) = σ_sca_ref · (λ/λ_ref)^(-n)
σ_ext(λ) = σ_ext_ref · (λ/λ_ref)^(-n)     ← 同样的 n
```

结果：三通道的**反照率完全相同**（`ω = σ_sca/σ_ext` 与 λ 无关）。

测试 `spectral_albedo_is_per_channel` 立刻失败：

```text
blue must have lower albedo when its extinction is higher: b=0.5 r=0.5
```

**物理上这是错的。** `σ_ext = σ_sca + σ_abs`，而：

* **散射**有强的波长依赖（∝λ⁻ⁿ，瑞利极限 n=4）
* **吸收**的波长依赖通常弱得多（灰体/宽带近似 n≈0）

所以消光的有效指数位于两者之间，不能与散射相同。

修正后的模型：

```text
σ_sca(λ) = σ_sca_ref · (λ/λ_ref)^(-n_sca)
σ_abs_ref = σ_ext_ref - σ_sca_ref
σ_abs(λ) = σ_abs_ref · (λ/λ_ref)^(-n_abs)
σ_ext(λ) = σ_sca(λ) + σ_abs(λ)
```

`from_monochrome` 保留原签名，内部调用
`from_monochrome_split(..., absorption_exponent = 0.0)`（灰体吸收）。这也是透明
烟雾/水滴的合理近似。

## 第二个需要想清楚的物理点

修正后测试仍然失败了一次，但那次是**我的断言写错了**：

```text
blue must have lower albedo when its extinction is higher: b=0.550 r=0.458
```

我假设"蓝光消光更强 → 反照率更低"。实际相反：`σ_sca`、`σ_ext` 固定时，
蓝光的**散射**更强而**吸收部分不变**，所以蓝光的反照率**更高**。

**"光柱偏红"来自消光更快，不是反照率更低。** 两个概念不同：

| 量 | 含义 |
| --- | --- |
| 反照率 `ω` | 被消光的光里有多少被**散射出来** |
| 消光 `σ_ext` | 光衰减得多快 |

蓝光的 `σ_ext` 更大 → 衰减更快 → 远处剩下红光。这才是偏红的原因。

只有吸收也强烈偏蓝（`absorption_exponent > 0`）时，蓝的反照率才会降低。

## 核心断言：偏红可测

测试 `blue_is_attenuated_faster_than_red` 与 `spectral_inscatter_shows_the_reddening`：

```rust
let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 1.0).unwrap();
let v = inscatter_spectral(o, 1e13, 1.0, white, 4.0, 256).unwrap();
let (r, g, b) = v.normalized();
assert!(r > b, "the beam must be warm after travelling");
```

## 新增的 C ABI

| 入口 | 用途 |
| --- | --- |
| `tyndall_wavelength_to_rgb(λ, out_r, out_g, out_b)` | 波长 → RGB |
| `tyndall_spectral_optics_from_monochrome(...)` | 单波长参数 → 三通道 |
| `tyndall_inscatter_spectral(...)` | **核心**：分通道散射亮度 |
| `tyndall_spectral_transmittance(...)` | 分通道透射率 |

三个 `out_*` 指针**全部成功才写**，避免半更新状态。

## 参数选择建议

做丁达尔光柱时的 `exponent`：

| `exponent` | 效果 | 适用 |
| --- | --- | --- |
| `0` | 三通道相同（无偏色） | 纯白烟雾 |
| `1.0~1.5` | 弱偏蓝，厚介质里偏红 | **丁达尔的实际情形** |
| `4.0` | 强烈偏蓝 | 瑞利极限（蓝天） |

**用 4.0 会做出蓝天，不是光柱。** 这与 ADR-0018 里强调的"丁达尔是米氏不是瑞利"
是同一个提醒。

## 后果

- 光柱穿过厚烟雾后**自然偏红**，而不是单调变灰
- 可与 ADR-0019 的粒子云版本组合（分通道 + 粒子驱动）
  —— **但那一层还没做**，见下

## 没做的事

1. **粒子驱动的光谱版**：现在是均匀介质。把 ADR-0019 的粒子云采样与分通道结合
   需要新函数（`inscatter_spectral_through_particles`），没做。
2. **CIE 精确色彩**：见上文，工程判断。
3. **多次散射**：与标量版相同的限制。

## 相关代码

- `crates/mps-formula/src/tyndall_spectral.rs`（公式）
- `crates/mps-core/src/rapier/emag.rs`（4 个 C ABI）
- `crates/mps-test/src/rapier/tyndall_spectral.rs`（16 项测试）

## 记录来源

与实现同批落地。两个物理错误都是测试抓出来的：一个是真的设计错（指数共用），
一个是我的断言错（反照率方向）。**两者都记账在此**，因为后者更容易被忽略——
测试失败不总是代码错。
