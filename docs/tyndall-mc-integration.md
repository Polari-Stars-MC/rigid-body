# 丁达尔效应：MC 侧集成指南

**给 Minecraft 渲染层的使用说明。** 物理计算全部在 Rust 侧完成，本文只讲怎么调用。

设计依据：[ADR-0018](adr/0018-tyndall-effect-data-layer.md) … [ADR-0024](adr/0024-full-tyndall-composition.md)。

## 一句话上手

**大多数情况只需要一个函数**：

```c
struct Bool tyndall_spectral_full_from_world(
    const struct WorldHandle *world,
    uint32_t source,            // TYNDALL_SOURCE_FLUID 或 TYNDALL_SOURCE_GRANULAR
    uint32_t id,                // 流体/颗粒世界的 id
    struct Vec3 origin,         // 视线起点（相机位置）
    struct Vec3 direction,      // 视线方向，**必须已归一化**
    double length,              // 视线在介质内的长度
    double smoothing_radius,    // 核半径，通常取 SPH 的 h
    double sigma_sca_r, sigma_sca_g, sigma_sca_b,
    double sigma_ext_r, sigma_ext_g, sigma_ext_b,
    double phase,               // 相函数值，见下文
    double light_r, light_g, light_b,   // 入射光颜色
    uint32_t steps,             // 采样段数，128~256
    uint32_t orders,            // 散射阶数，1（单次）~ 8
    double geometric_factor,    // 0.5
    double *out_r, *out_g, *out_b);     // 输出 RGB 亮度
```

返回值是 `Bool`：`TRUE` 表示三个 `out_*` 已写入；`FALSE` 表示失败且**没有动过 `out_*`**。

## 三条必须遵守的约定

### 1. `direction` 必须已归一化

未归一化会返回 `FALSE`（不是静默算错）。理由：`Δs = length / steps`，方向长度
不是 1 会让实际步长缩放，整条积分失真且不报错。

```java
Vec3 dir = camera.getLookAngle().normalize();   // 别忘了 normalize()
```

### 2. 失败时 `out_*` 不变

三个指针**全部成功才写**。所以：

* 可以先填哨兵值，失败时检查是否被改动
* 也可以直接检查返回值，忽略 `out_*`

```java
double r = -1, g = -1, b = -1;
boolean ok = native.tyndallSpectralFullFromWorld(..., r, g, b);
if (!ok) { /* 用上一帧的值或跳过光柱 */ }
```

### 3. 参数的量级很重要

`sigma_*` **不是**单分子截面。经验值：

| 参数 | 推荐值 | 说明 |
| --- | --- | --- |
| `sigma_sca_*` | `1e-3` 量级 | 见下方「为什么不是 1e-12」 |
| `sigma_ext_* / sigma_sca_*` | `3~5` | 需要吸收才能偏红 |
| `smoothing_radius` | SPH 的 `h`（通常 0.5~1.0 m） | 与求解器一致，否则光柱偏离粒子 |
| `steps` | `128~256` | 只影响积分精度，与粒子数无关 |
| `orders` | `4~8` | `geometric_factor = 0.5` 时 8 阶已收敛 |
| `geometric_factor` | `0.5` | 前后各半的一阶估计 |

**为什么不是 `1e-12`**：`sigma` 的单位是 m²，`1e-12` 是**单分子**的截面。而
`tyndall_density_from_world` 返回的是 Poly6 核归一化后的**宏观数密度**（间距
0.1 m 的粒子云约 12.7 个/m³）。两者搭配得到 `τ ≈ 1e-10`，透射率恒为 1，
光柱不会有任何厚度效果。

正确的关系：`σ ≈ 1 / (n · L)`。`n ≈ 12.7`、`L ≈ 60` 时 `σ ≈ 1.3e-3`。

## 怎么选入口

| 场景 | 用哪个 |
| --- | --- |
| **流动的烟雾 + 彩色光柱**（推荐） | `tyndall_spectral_full_from_world` |
| 只要亮度，不要颜色 | `tyndall_inscatter_from_world_accelerated` |
| 静态均匀雾（无粒子系统） | `tyndall_inscatter_spectral` 或 `tyndall_inscatter_along_ray` |
| 调参数、不想跑模拟 | `tyndall_visibility_score`（`0..=1` 的直观评分） |
| 只要单个物理量 | `tyndall_mie_scattering_efficiency` 等纯公式 |

## 完整示例（Java 侧伪代码）

```java
// 1. 每帧为每个可见的烟雾世界算一次
long fluidId = smokeWorld.getId();
Vec3 cam = camera.getPosition();
Vec3 dir = camera.getLookAngle().normalize();

// 2. 视线长度：从相机到烟雾包围盒出口
double len = 60.0;   // 或用 tyndall_bounds_from_world 精确算

// 3. 调用
double r = 0, g = 0, b = 0;
boolean ok = MPSNative.tyndallSpectralFullFromWorld(
    worldHandle, TYNDALL_SOURCE_FLUID, fluidId,
    cam.x, cam.y, cam.z,
    dir.x, dir.y, dir.z,
    len, 0.8,                          // smoothing_radius
    8.46e-3, 1.0e-2, 1.22e-2,          // sigma_sca RGB
    2.36e-2, 3.3e-2, 7.91e-2,          // sigma_ext RGB（蓝端强吸收）
    0.8,                               // phase（前向散射）
    1.0, 1.0, 1.0,                     // 白光
    192, 8, 0.5,                       // steps, orders, geometric_factor
    r, g, b);

if (ok) {
    // 4. 把 (r,g,b) 作为体积光的颜色系数传给 shader
    renderBeam(r, g, b);
}
```

## 相函数怎么取

`phase` 是散射角方向的相函数值。丁达尔效应是**前向散射**，所以取正前方：

```java
// 相机与光源方向的夹角余弦
double cosTheta = lightDir.dot(viewDir);
// g 越大越前倾（0.7~0.9 是丁达尔典型值）
double phase = henyeyGreensteinPhase(cosTheta, 0.8);
```

`henyey_greenstein_phase` 的公式：

```text
p(θ) = (1 - g²) / (1 + g² - 2g·cosθ)^(3/2)
```

**`g = 0.8` 时正前方亮度是背向的约 181 倍** —— 这就是为什么丁达尔光柱看起来
像"一条亮线"而不是均匀发光。

## 性能

| 粒子数 | 每帧成本 | 说明 |
| --- | --- | --- |
| < 64 | 朴素路径 | 建网格不划算，自动退化 |
| 10³ | 建网格 + `steps × k` | `k` 是单格粒子数 |
| 10⁶ | 同上 | 网格把 `O(steps·N)` 降到 `O(N + steps·k)` |

**优化建议**：

1. **先用 `tyndall_bounds_from_world` 做视锥剔除** —— 不在视野里就别算
2. **`steps` 按距离调**：远处烟雾用 64，近处用 256
3. **降频**：光柱亮度不需要每帧更新，可以 2~3 帧算一次

## 常见问题

**Q: 光柱是蓝的，不是白的。**
`orders` 太小或 `sigma_ext` 等于 `sigma_sca`（无吸收）。丁达尔需要：
光学厚（`τ > 1`）+ 有吸收 + 吸收偏蓝端。

**Q: 光柱太暗/看不见。**
检查 `sigma` 的量级（见上文）。`1e-12` 会得到几乎为零的结果。

**Q: 光柱位置和粒子对不上。**
`smoothing_radius` 必须与 SPH 求解器的 `h` 一致。不一致时核的形状不同，
密度场会偏移。

**Q: `FALSE` 返回，怎么查原因？**
调 `last_error_code()`：`ERR_NULL_POINTER`(1) = 指针为空；
`ERR_INVALID_ARGUMENT`(2) = 方向未归一化或参数非法；`ERR_NOT_FOUND`(3) = id 不存在。

## 还没做的（诚实说明）

1. **多次散射是近似**：阶数递推 + 等比级数，不是辐射传输精确解。
   介质高度非均匀时误差增大。
2. **米氏散射用渐近式**：精确解需要贝塞尔函数，仓库里没有。
   渲染够用，定量光度学不够。
3. **不含渲染**：本文所有函数都只给数据，"怎么画出光柱"（billboard / 体积光 /
   后处理）是渲染层的事。

## 相关文档

- [ADR-0018](adr/0018-tyndall-effect-data-layer.md) — 物理基础（米氏散射 / 相函数 / Beer-Lambert）
- [ADR-0019](adr/0019-particle-driven-volumetric-scattering.md) — 粒子云驱动
- [ADR-0020](adr/0020-spectral-tyndall.md) — RGB 分通道
- [ADR-0021](adr/0021-particle-driven-spectral.md) — 粒子 + 分通道
- [ADR-0022](adr/0022-multiple-scattering.md) — 多次散射
- [ADR-0023](adr/0023-uniform-grid-acceleration.md) — 空间加速
- [ADR-0024](adr/0024-full-tyndall-composition.md) — 完整合成
