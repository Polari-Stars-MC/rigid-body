# 0024 — 丁达尔效应的完整合成

## 背景

丁达尔效应分四条 ADR 完成，能力分散在多个函数里：

| ADR | 能力 | 入口 |
| --- | --- | --- |
| [0018](0018-tyndall-effect-data-layer.md) | 米氏散射 / 相函数 / Beer–Lambert / 可见度 | 9 个纯公式 |
| [0019](0019-particle-driven-volumetric-scattering.md) | 粒子云驱动（SPH / 颗粒） | 5 个场景入口 |
| [0020](0020-spectral-tyndall.md) | RGB 分通道 | 4 个纯公式 |
| [0021](0021-particle-driven-spectral.md) | 粒子 + 分通道 | 1 个场景入口 |
| [0022](0022-multiple-scattering.md) | 多次散射 | 2 个纯公式 |
| [0023](0023-uniform-grid-acceleration.md) | 空间加速 | 1 个场景入口 |

宿主想同时用"粒子驱动 + 分通道 + 多次散射 + 加速"时，没有单个入口能表达。

## 决定

新增 `inscatter_spectral_full` + C ABI `tyndall_spectral_full_from_world`。

### 关键：每通道用自己的反照率驱动级数

这是本批唯一有实质风险的地方。多次散射的级数比是 `r = ω · g`，而分通道时
**每个通道有自己的 `ω_ch = σ_sca_ch / σ_ext_ch`**：

```text
L_ch = L1_ch · (1 - r_ch^N) / (1 - r_ch)      r_ch = ω_ch · g
```

**不能用一个共享的 `r`** —— 那会让三通道的修正倍数相同，"高阶散射也偏红"这一半
物理就丢失了。

测试 `full_uses_per_channel_albedo_for_the_series` 构造三通道反照率差异明显的
介质（`ω_r ≈ 0.99`、`ω_g = 0.25`、`ω_b = 0.02`），断言：

```text
ratio_r > ratio_g > ratio_b >= 1
```

即反照率越高的通道，高阶增益越大。

### `orders = 1` 的兼容性

`orders = 1` 时与 `inscatter_spectral_through_particles` **逐位相同**。
测试 `full_with_one_order_matches_single_scatter` 用 80 个粒子、64 步比对
三通道，容差 `1e-15` 相对。

这保证了宿主可以无条件替换旧入口，不需要评估行为差异。

## 新增的 C ABI

```c
struct Bool tyndall_spectral_full_from_world(
    const struct WorldHandle *world, uint32_t source, uint32_t id,
    struct Vec3 origin, struct Vec3 direction,
    double length, double smoothing_radius,
    double sigma_sca_r, double sigma_sca_g, double sigma_sca_b,
    double sigma_ext_r, double sigma_ext_g, double sigma_ext_b,
    double phase,
    double light_r, double light_g, double light_b,
    uint32_t steps, uint32_t orders, double geometric_factor,
    double *out_r, double *out_g, double *out_b);
```

`tyndall_*` 系列现在共 **23 个入口**。

## 参数速查

做"流动烟雾里的彩色光柱"时的推荐值：

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `sigma_sca` | `1e-3` 量级 | 见 [ADR-0021](0021-particle-driven-spectral.md) 的量级说明 |
| `sigma_ext / sigma_sca` | `3~5` | 需要吸收才能偏红 |
| 吸收的波长指数 | `4` | 蓝端强吸收 |
| `steps` | `128~256` | 与粒子数无关，只影响积分精度 |
| `orders` | `4~8` | 更多阶的边际收益递减（`r ≈ 0.5` 时 8 阶已收敛） |
| `geometric_factor` | `0.5` | 前后各半的一阶估计 |

## 一个尚未消除的重复

`inscatter_spectral_through_particles`（ADR-0021）与
`inscatter_spectral_full`（本 ADR）的单次散射部分是同一段代码——前者被后者调用。

**没有合并成一个函数带 `orders` 参数**，理由：ADR-0021 的入口已经发布且被测试
覆盖，改它的签名会让那一批的 diff 变大。留给下次动它时顺手做。

同理，`tyndall_scene.rs` 里五个场景入口的参数解析有重复（每个都校验
`origin` / `direction` / `out`）。抽成辅助函数是可行的，但会让这一批的改动面变大。

## 后果

- 宿主可以用**一个**入口拿到全部能力
- `orders = 1` 向后兼容
- 三通道的多次散射物理正确（每通道独立反照率）

## 没做的事

1. **光谱版的网格加速**：`inscatter_spectral_full` 内部调
   `inscatter_spectral_through_particles`，后者仍是朴素采样。加速收益与标量版
   相同（三通道共享一次密度查询），但没接。
2. **场景入口的参数解析去重**：见上。
3. **`tyndall_scene.rs` 的 `collect_positions` 重构**：ADR-0021 已经抽出了这个
   辅助函数，但前两个入口（标量版）还没改用它。

## 相关代码

- `crates/mps-formula/src/tyndall_spectral.rs`（`inscatter_spectral_full`）
- `crates/mps-core/src/rapier/tyndall_scene.rs`（1 个 C ABI）
- `crates/mps-test/src/rapier/tyndall_spectral.rs`（4 项新测试）

## 记录来源

与实现同批落地。测试一次通过——「每通道反照率」那条断言的构造（三通道反照率
差异明显）是关键，它让"共享 r"的实现会立刻失败。
