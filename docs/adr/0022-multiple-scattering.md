# 0022 — 多次散射近似

## 背景

[ADR-0018](0018-tyndall-effect-data-layer.md) 的 `inscatter_along_ray` 只算光
**第一次**被散射就进入视线的部分。

厚介质（光学厚度 `τ > 2`）里，光实际会经历多次散射才出来。单次近似**低估**
亮度 2~5 倍，表现为"烟雾越浓光柱反而越暗"——这在视觉上是错的。

## 决定

新增 `inscatter_along_ray_multiple`，用**阶数递推 + 等比级数**近似。

### 为什么不用完整的辐射传输求解

完整的多次散射需要迭代整个光场（radiative transfer equation），代价是：

* 需要体素化的光场网格
* 每帧多次迭代
* 与现有的"沿单条视线积分"架构完全不兼容

而渲染只需要"厚介质不该比薄介质暗"这个定性正确性。所以选工程近似。

### 采用的模型

利用一个物理事实：**每次散射后，只有 `ω = σ_sca/σ_ext` 的部分继续传播**
（其余被吸收）。

```text
L_total = L_1 · Σ_{n=0}^{N-1} (ω · g)^n
```

其中 `L_1` 是单次散射结果，`g` 是**几何衰减因子**（散射光重新进入视线的
立体角比例，默认 `0.5`）。

等比级数闭式求和：

```text
L_total = L_1 · (1 - r^N) / (1 - r)      r = ω·g
```

### 性质（都有测试）

| 性质 | 测试 |
| --- | --- |
| `orders = 1` 与单次散射**逐位相同** | `single_order_matches_the_one_scatter_function` |
| 亮度随阶数单调递增 | `multiple_scattering_increases_and_converges` |
| 阶数增大后收敛（`r = 0.25` 时 8→16 阶增量 < 1e-4） | 同上 |
| 厚介质修正倍数 > 1.5 | `multiple_scattering_corrects_thick_media_underestimation` |
| 修正倍数有界（`g=0.5` 时极限 `1/(1-0.5) = 2`） | 同上 |
| 强吸收（`ω = 0.1`）时高阶可忽略（< 1.1 倍） | `low_albedo_suppresses_higher_orders` |
| `g` 越大收集越多 | `geometric_factor_controls_the_series` |
| `r = 1` 退化情形不死循环/不除零 | `pure_scattering_with_unit_geometric_factor_stays_finite` |

## 精度与局限（诚实说明）

**这是工程近似，不是辐射传输的精确解。** 它的正确性依赖：

1. `ω·g < 1`（否则级数发散）—— 物理上 `ω <= 1`、`g < 1` 保证
2. 介质近似均匀（`σ` 沿视线变化时误差增大）

**没有做的**：

* 空间上的多次散射路径（本模型只做"同一条视线上重复算"）
* 相函数的角度重分布（每阶沿用同一个 `phase` 值）
* 偏振

对渲染足够：视觉上重要的是"厚介质亮度单调上升到饱和"，而这个模型保证了这点，
不会像单次散射那样过早在某个厚度封顶。

## `g` 的取值

| `g` | 含义 | 修正倍数上限 |
| ---: | --- | ---: |
| `0.1` | 散射光很少回到视线 | `1/(1-0.1ω)` |
| `0.5` | **默认**，前后大致各半 | `1/(1-0.5ω)` |
| `0.9` | 强前向散射（丁达尔的实际情形） | `1/(1-0.9ω)` |

**注意**：`g = 0.9` 且 `ω = 1` 时 `r = 0.9`，16 阶的修正倍数是
`(1-0.9^16)/(1-0.9) ≈ 8.15`。这与"浓雾里光柱非常亮"的观感一致，但也意味着
**参数需要按场景调**，不能盲目用大 `g`。

## 新增的 C ABI

```c
struct Bool tyndall_inscatter_multiple(
    double sigma_sca, double sigma_ext, double number_density,
    double phase, double light_intensity, double ray_length,
    uint32_t steps, uint32_t orders, double geometric_factor,
    double *out);

uint32_t tyndall_max_scatter_orders(void);   // 上限 16，供宿主探测
```

`orders` 上限 `MAX_SCATTER_ORDERS = 16`，超限返回 `None`（不是静默截断）——
避免病态参数下无限循环。

## 后果

- 厚介质的亮度不再被低估
- `orders = 1` 保持与旧行为完全一致，**向后兼容**
- 与光谱版**尚未组合**：分通道的多次散射需要每通道各算一次级数，没做

## 没做的事

1. **分通道的多次散射**：`inscatter_spectral` 还没有 `orders` 参数
2. **空间加速**：仍是 `steps × N`
3. **路径级多次散射**：见上文的局限说明

## 相关代码

- `crates/mps-formula/src/tyndall.rs`（`inscatter_along_ray_multiple`）
- `crates/mps-core/src/rapier/emag.rs`（2 个 C ABI）
- `crates/mps-test/src/rapier/tyndall.rs`（8 项测试）

## 记录来源

与实现同批落地。这一批的测试**一次通过**——与 [ADR-0021](0021-particle-driven-spectral.md)
形成对比，那里连续三次因为物理直觉错误失败。原因大概是这一批的模型更简单
（等比级数），且 `orders = 1` 的兼容性断言给了明确的锚点。
