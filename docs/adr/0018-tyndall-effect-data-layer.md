# 0018 — 丁达尔效应的数据层

## 背景

需求是在 Minecraft 里渲染**光柱**（丁达尔效应），Rust 侧只负责**埋好数据层计算**，
渲染交给 MC。

先盘点了项目已有能力：

| 组件 | 状态 |
| --- | --- |
| 瑞利散射截面（1/λ⁴） | ✅ `mps_formula::electromagnetism::rayleigh_scattering_cross_section` |
| 全波光学（菲涅尔/夫琅禾费衍射、薄膜干涉、杨氏双缝） | ✅ `mps_formula::wave_optics`（971 行） |
| **米氏散射** | ❌ 0 处 |
| **散射相函数** | ❌ 0 处 |
| **Beer–Lambert 消光** | ❌ 只有零散一处 |
| **沿视线散射积分** | ❌ 0 处 |

也就是说：**底子在，但缺的恰好是丁达尔的核心。**

## 一个必须先澄清的物理点

**丁达尔效应主要是米氏散射，不是瑞利散射。**

标志是能看到一束**接近白色**的光柱（烟雾、胶体、粉尘里的可见光束）。

| | 瑞利 | 米氏 |
| --- | --- | --- |
| 适用 | 粒径 ≪ λ（气体分子） | 粒径 ≈ 或 > λ（胶体、雾滴、粉尘） |
| 波长依赖 | **1/λ⁴** | 弱，近似无关 |
| 角分布 | 前后对称（1+cos²θ） | **强烈前倾** |
| 光柱颜色 | 蓝 | **白** |

只用现有的 `rayleigh_scattering_cross_section` 做出来的是**蓝天效应**，不是丁达尔。

## 决定

新增 `mps_formula::tyndall`（纯公式）+ `mps_core::rapier::emag` 里的 C ABI 包装。

### 公式层（`crates/mps-formula/src/tyndall.rs`）

| 函数 | 物理意义 |
| --- | --- |
| `size_parameter(d, λ)` | `x = πd/λ`，米氏理论的一切从它开始 |
| `rayleigh_phase(cosθ)` | `(3/4)(1+cos²θ)`，前后对称 |
| `henyey_greenstein_phase(cosθ, g)` | 通用前倾相函数，丁达尔"亮线"的来源 |
| `mie_scattering_efficiency(x, n)` | 三段渐近近似（见下） |
| `beer_lambert_transmittance(σ, n, L)` | `exp(-τ)` |
| `optical_thickness(σ, n, L)` | `τ = σ·n·L` |
| `single_scattering_albedo(σ_sca, σ_ext)` | `ω = σ_sca/σ_ext` |
| `inscatter_along_ray(...)` | 沿视线单次散射累积亮度 |
| `tyndall_visibility(...)` | **领域封装**：`0..=1` 的可见度评分 |

### 关于米氏精度的诚实说明

米氏散射的**精确解**需要 Riccati-Bessel 函数的级数求和
（Bohren & Huffman）。本项目**没有贝塞尔函数实现**（0 处），引入它需要新增数值库。

所以 `mie_scattering_efficiency` 用三段近似：

| 尺寸参数 `x` | 近似 | 精度 |
| --- | --- | --- |
| `x < 0.3` | 瑞利极限 `(8/3)x⁴|K|²` | 精确 |
| `0.3 ≤ x < 20` | van de Hulst 反常衍射 | 误差 10~20% |
| `x ≥ 20` | `Q → 2`（消光佯谬） | 定性正确 |

**对渲染用途够用**：人眼对光柱亮度的分辨远达不到 20%。将来若要定量光度学，
应该引入贝塞尔实现。这个限制写在函数文档里，也写在这里。

## 一个被测试抓出来的物理错误

`tyndall_visibility` 的第一版是：

```text
visibility = ω · (1 - T) · front_bias
```

测试 `visibility_peaks_at_intermediate_optical_thickness` 立刻失败：

```text
too thick is less visible: optimal 0.569 vs thick 0.900
```

**τ = 20（极浓）时可见度反而接近 1.0**，因为 `1 - T → 1`。但现实中浓到看不清的
烟雾里**根本看不到光柱**。

根本原因：光柱的散射光**也要穿过介质才能到达观察者**，而第一版只算了入射光被
散射掉多少，没算出射衰减。

修正后加了一项 `T_out`：

```text
visibility = ω · (1 - T_in) · T_out · front_bias
T_out = sqrt(T_in)      # 散射点平均在介质中点，出射路程约一半
```

数值验证（`ω=1, g=0.8`）：

| τ | visibility |
| ---: | ---: |
| 0.1 | 0.082 |
| 0.5 | 0.276 |
| **1.0** | **0.345** ← 峰值 |
| 2.0 | 0.286 |
| 10 | 0.006 |
| 20 | 0.000 |

现在与"浓度刚好的烟雾里光柱最明显"的经验一致。

## 架构：为什么 ABI 不在 formula 层

第一版把 `extern "C"` 写在了 `mps-formula/src/tyndall.rs` 里，构建后
`rigid_body.h` 里**一个 `tyndall_` 都没有**。

查明原因：**`mps-formula` 的 `extern "C"` 不进 `rigid_body.h`** —— cbindgen 只处理
`mps-core` 和 `mps-cosmos`。这也解释了为什么 `emag.rs` 要把
`rayleigh_scattering_cross_section` 重新包一遍。

所以最终结构是：

```text
mps-formula/src/tyndall.rs     纯公式，无 extern "C"（247 行）
        ↓
mps-core/src/rapier/emag.rs    9 个 tyndall_* C ABI，用 ffi_scalar 包装
        ↓
rigid_body.h                   MC 侧通过 JNI 调用
```

`ffi_scalar` 自带 panic barrier（`catch_unwind`），符合 BOUNDARY.md 的要求。

## 给 MC 侧的接口

9 个 C 入口，全部是 `(参数..., double *out) -> Bool` 形态：

```c
struct Bool tyndall_size_parameter(double diameter, double wavelength, double *out);
struct Bool tyndall_rayleigh_phase(double cos_theta, double *out);
struct Bool tyndall_henyey_greenstein_phase(double cos_theta, double g, double *out);
struct Bool tyndall_mie_scattering_efficiency(double size_param, double n, double *out);
struct Bool tyndall_beer_lambert_transmittance(double sigma_ext, double n, double L, double *out);
struct Bool tyndall_optical_thickness(double sigma_ext, double n, double L, double *out);
struct Bool tyndall_single_scattering_albedo(double sigma_sca, double sigma_ext, double *out);
struct Bool tyndall_inscatter_along_ray(..., uint32_t steps, double *out);
struct Bool tyndall_visibility_score(double sigma_sca, double sigma_ext,
                                     double n, double L, double g, double *out);
```

**渲染层最可能只用两个**：

* `tyndall_visibility_score(...)` → 直接得到 `0..=1` 的亮度调制系数
* `tyndall_inscatter_along_ray(...)` → 得到沿视线的累积亮度，用于逐像素/逐段着色

## 后果

- 丁达尔效应的**物理数据**齐了：散射系数、相函数、消光、沿视线积分、可见度
- **不含渲染**：怎么把光柱画出来（billboard、体积光、后处理）是 MC 侧的事
- 与现有能力互补：`rayleigh_scattering_cross_section` 仍在（做蓝天），
  `wave_optics` 的衍射仍在（做波纹/彩虹），本模块专做**体积散射**

## 没做的事

1. **多次散射**：`inscatter_along_ray` 是单次散射近似，光学厚度 `τ < 1` 时够用，
   浓雾会低估亮度。
2. **精确米氏解**：缺贝塞尔函数，见上文。
3. **与 SPH/粒子的耦合**：项目有 `fluid.rs`（SPH）和 `granular.rs`，理论上可以让
   粒子分布直接驱动局部数密度 `n`，做出"烟随流体飘、光柱随之变化"的效果。
   这是最有意思的下一步，但需要新接口（读粒子密度场）。

## 相关代码

- `crates/mps-formula/src/tyndall.rs`（公式，247 行）
- `crates/mps-core/src/rapier/emag.rs`（9 个 C ABI）
- `crates/mps-test/src/rapier/tyndall.rs`（27 项测试：22 公式 + 5 ABI）

## 记录来源

与实现同批落地。设计先行、测试验证物理、再修公式——`visibility` 的修正就是
这条纪律的产物。
