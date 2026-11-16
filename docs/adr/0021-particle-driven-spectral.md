# 0021 — 粒子驱动的光谱散射

## 背景

丁达尔效应此前分两块完成，各占一半：

| ADR | 能力 | 缺口 |
| --- | --- | --- |
| [0019](0019-particle-driven-volumetric-scattering.md) | 粒子驱动（SPH/颗粒） | 只输出标量 |
| [0020](0020-spectral-tyndall.md) | RGB 分通道 | 只支持均匀介质 |

这一条把两者合起来：**流动的烟雾里的彩色光柱**。

## 决定

新增 `inscatter_spectral_through_particles` + C ABI `tyndall_spectral_from_world`。

### 关键实现：三通道的透射率必须独立

容易写错的地方是用同一个透射率标量累加三个通道——那样蓝光不会比红光衰减得
快，"偏红"就消失了。每通道必须有独立的 `trans_r` / `trans_g` / `trans_b`。

### 采样点复用

三个通道在**同一条视线上**采样，所以数密度只算一次、三通道共用。相比调三次
单通道函数，核求值的开销降到三分之一。

## 三个物理结论（都是测试逼出来的）

这一批的测试失败了三次，每次都不是实现 bug，而是**我的物理直觉错**。三条结论
值得记下来，因为它们都反直觉。

### 1. 光学薄时，蓝光一直更亮

```text
L(λ) ∝ σ_sca(λ) · (1 - exp(-τ(λ))) ≈ σ_sca(λ) · τ(λ) ∝ σ_sca(λ)
```

蓝光的 `σ_sca` 更大 → 散射更多 → **更亮**。这与"光柱偏红"的直觉相反，但正确。

测试 `optically_thin_medium_stays_blue` 钉住这一点。

### 2. 偏红必须有**吸收**

纯散射介质（`σ_ext == σ_sca`）里，无论多厚，颜色**恒等于** `σ_sca` 的通道比值：

```text
L(λ) ∝ (1 - exp(-τ(λ)))     τ 大时各通道都饱和到 1 → 比值退化为 σ_sca 比
```

所以"厚烟雾偏红"必须同时满足：

* `σ_ext > σ_sca`（有吸收）
* 吸收对蓝端更强（`absorption_exponent > 0`）
* 光学厚度 `τ ≫ 1`

这也解释了为什么 [ADR-0020](0020-spectral-tyndall.md) 要把散射与吸收的波长指数
**分开**——用同一个指数时三通道反照率相同，永远做不出偏色。

### 3. 量级差 9 个数量级

`sample_number_density` 返回 Poly6 核归一化后的**真实数密度**。对间距 0.1 m、
核半径 0.8 m 的粒子云，`n ≈ 12.7`（个/m³）。

要让 `τ = σ·n·L ≈ 1`：

```text
σ ≈ 1 / (n · L) = 1 / (12.7 × 60) ≈ 1.3e-3
```

**`σ` 必须是 `1e-3` 量级，而不是 `1e-12`。** 后者是**单分子**截面，与宏观粒子云
的数密度搭配会得到 `τ ≈ 1e-10`——透射率恒为 1，三通道只是线性累加。

我第一版测试全用了 `1e-12`，所以 `r/b` 恒等于 `σ_sca` 的比值（0.6923），
与厚度、吸收全部无关。**这个数字看起来像"实现没生效"，其实是参数差了 9 个数量级。**

实测对照（60 粒子、`L = 60`、`na = 4`）：

| `σ_sca` | `r/b` | 区域 |
| ---: | ---: | --- |
| `1e-4` | 0.717 | 光学薄，蓝光强 |
| `1e-3` | 0.949 | 近中性 |
| `1e-2` | **1.988** | 光学厚，红光胜出 |

## 新增的 C ABI

```c
struct Bool tyndall_spectral_from_world(
    const struct WorldHandle *world, uint32_t source, uint32_t id,
    struct Vec3 origin, struct Vec3 direction,
    double length, double smoothing_radius,
    double sigma_sca_r, double sigma_sca_g, double sigma_sca_b,
    double sigma_ext_r, double sigma_ext_g, double sigma_ext_b,
    double phase,
    double light_r, double light_g, double light_b,
    uint32_t steps,
    double *out_r, double *out_g, double *out_b);
```

三个 `out_*` 指针**全部成功才写**，避免半更新状态。

`tyndall_*` 系列现在共 **19 个入口**。

## 重构：抽出 `collect_positions`

三个场景层入口（标量/光谱/密度）原本各写一段读粒子的 `match`。这次抽成
`collect_positions(world, source, id)` 和 `particle_mass(...)` 两个辅助函数。

**没有立即重构前两个入口**——它们已经通过测试，改动它们会让这一批的 diff 变大。
留给下次动那两个函数时顺手做。

## 后果

- 光柱可以**随 SPH 粒子流动而变形**，且**穿过厚烟雾后偏红**
- 参数选择：`σ_sca ≈ 1e-3`、`σ_ext/σ_sca ≈ 3~5`、`absorption_exponent ≈ 4`、
  `steps = 128~256`

## 没做的事

1. **多次散射**：单次近似，`τ ≫ 1` 时低估亮度。这是下一个明显缺口。
2. **空间加速**：`steps × N` 纯扫，10⁶ 粒子会慢。
3. **前两个场景入口的辅助函数重构**：见上。

## 相关代码

- `crates/mps-formula/src/tyndall_spectral.rs`（`inscatter_spectral_through_particles`）
- `crates/mps-core/src/rapier/tyndall_scene.rs`（`tyndall_spectral_from_world`）
- `crates/mps-test/src/rapier/tyndall_spectral.rs`（22 项）
- `crates/mps-test/src/rapier/tyndall_scene.rs`（7 项）

## 记录来源

与实现同批落地。三次测试失败全部是我的物理直觉错，已逐条记账——**测试失败不总是
代码错**，这一批是最好的例子。
