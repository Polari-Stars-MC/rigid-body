# 0019 — SPH / 颗粒粒子云驱动的体积散射

## 背景

[ADR-0018](0018-tyndall-effect-data-layer.md) 完成了丁达尔效应的**常数介质**数据层：
给定固定的 `σ_sca` / `σ_ext` / `n`，算出散射亮度。

但真实的烟雾是**流动的**——光柱应该随 SPH 粒子的运动而变形、明暗变化，而不是
一张静态贴图。

项目已有两个粒子系统：

| 系统 | 位置 | 粒子字段 |
| --- | --- | --- |
| SPH 流体 | `rapier/src/dynamics/fluid.rs` | `pos` / `vel` / `mass` / **`density`** |
| 颗粒（DEM） | `rapier/src/dynamics/granular.rs` | `pos` / `vel` / `mass` / `radius` |

这层耦合是把**粒子分布**变成**光学量**。

## 决定

新增两块：

1. `mps_formula::tyndall_particles` —— 纯公式（Poly6 核、数密度采样、粒子云散射积分）
2. `mps_core::rapier::tyndall_scene` —— 从 `WorldHandle` 读粒子并调公式

### 为什么单独一个 `tyndall_scene` 模块

`emag.rs` 的模块文档明确写着「**No `WorldHandle` / Rapier state is touched**」。
把读粒子云的入口塞进 `emag.rs` 会破坏这个约定，所以独立成模块。

### 为什么用 Poly6 核

渲染时需要把**离散粒子**抹成**连续密度场**，否则光柱会呈现颗粒状斑点。

选 Poly6 而不是"更适合渲染"的核，理由是：**项目已有同款实现**
（`fluid_sph_poly6_kernel`）。保持物理口径一致比换一个核更重要——否则同一个
烟雾在求解器里和渲染器里的密度分布会对不上，光柱会偏离粒子实际位置。

```text
W(r, h) = (315 / (64π h⁹)) · (h² - r²)³     r < h
        = 0                                  r >= h
```

核已归一化，所以 `Σ W` 直接给出"单位体积内有多少粒子"（数密度），不需要再除质量。

### 两条通往数密度的路径

| 路径 | 复杂度 | 何时用 |
| --- | --- | --- |
| `sample_number_density(positions, query, h)` | O(N) | 需要任意点的密度（光柱着色） |
| `number_density_from_mass_density(ρ, m)` | O(1) | 宿主直接读 `FluidParticle.density` |

第二条是 `n = ρ / m`，利用求解器已经算好的 Poly6 质量密度。**但它只在粒子位置
上准确**——粒子之间的空间里没有粒子，直接外推会有偏。所以渲染用第一条。

## 数据流

```text
rapier FluidWorld.particles      ← SPH 求解每步更新
        ↓ tyndall_collect_particles / 内部直接读
tyndall_particles::sample_number_density   (Poly6 核加权)
        ↓
tyndall_particles::inscatter_through_particles
        ↓  逐段 Beer–Lambert 累积
亮度 L  → MC 渲染层
```

## 新增的 C ABI

| 入口 | 用途 |
| --- | --- |
| `tyndall_source_abi_version()` | 来源枚举的版本探测 |
| `tyndall_collect_particles(world, source, id, out, capacity)` | 导出粒子位置（探数量 / 按容量读） |
| `tyndall_inscatter_from_world(...)` | **核心**：直接读世界粒子算沿视线亮度 |
| `tyndall_density_from_world(world, source, id, query, h, out)` | 任意点的数密度（自适应采样用） |
| `tyndall_bounds_from_world(world, source, id, out_min, out_max)` | 包围盒（视锥剔除） |

来源枚举：

```c
#define TYNDALL_SOURCE_FLUID 0
#define TYNDALL_SOURCE_GRANULAR 1
```

`tyndall_collect_particles` 沿用仓库既有的「先探总数、再按容量读」约定
（与 `granular_read_particles`、`soft_body_read_*` 一致）：`out == null` 时只
返回总数，`capacity` 不足时只写前 `capacity` 个但仍返回总数。

## 一个必须拒绝的输入

`inscatter_through_particles` 要求 **`direction` 必须已归一化**，否则返回 `None`。

理由：`Δs` 的物理长度是 `length / steps`，如果方向长度不是 1，实际步长会缩放，
整条积分失真且**不会报错**——算出的是错的亮度，而不是失败。这类静默错误最难查，
所以宁可显式拒绝。

测试 `inscatter_rejects_unnormalised_direction` 钉住这一点。

## 性能考量

`tyndall_inscatter_from_world` 每帧调用一次，内部做 `steps × particles` 次核求值。

对 `steps = 128`、`10⁴` 粒子是 128 万次乘加，单帧可接受但不算便宜。三个已实现的
优化：

1. **透射率提前退出**：`trans < 1e-12` 时 `break`，浓烟里后面的段落不再采样
2. **粒子位置一次性拷贝**：公式层不依赖 rapier 类型，代价是一次 `Vec` 分配
3. **`tyndall_bounds_from_world`**：让渲染层先做视锥剔除，不画就不算

**没做的优化**：空间哈希 / BVH 加速核查询。`steps` 个查询点各自扫全部粒子是
O(steps·N)。如果将来粒子数到 10⁶，应该加一个均匀网格。

## 与 ADR-0018 的关系

| | ADR-0018 | ADR-0019（本条） |
| --- | --- | --- |
| 介质 | **均匀**（常数 `n`） | **非均匀**（从粒子采样） |
| 入口 | 纯计算器，不碰世界 | 读 `WorldHandle` |
| 用途 | 快速估算、参数调试 | 真实烟雾的逐帧光柱 |

两者互补：调参时用 0018 的 `tyndall_visibility_score`，运行时用 0019 的
`tyndall_inscatter_from_world`。

## 后果

- 光柱可以**随 SPH 粒子流动而变形**，不再是静态贴图
- 颗粒（DEM）也支持，可用于沙尘/灰烬的散射
- 物理层仍不含渲染：怎么把亮度变成像素是 MC 侧的事

## 没做的事

1. **多次散射**：单次散射近似，`τ < 1` 时够用
2. **空间加速结构**：见上文
3. **粒子质量不一致的处理**：通道取首粒子质量作为代表值。SPH 里通常全部相同，
   但如果有例外，这个近似会偏。

## 相关代码

- `crates/mps-formula/src/tyndall_particles.rs`（Poly6、数密度、粒子云积分）
- `crates/mps-core/src/rapier/tyndall_scene.rs`（5 个 C ABI）
- `crates/mps-test/src/rapier/tyndall_particles.rs`（24 项：17 公式 + 7 端到端）
- `crates/mps-test/src/rapier/tyndall_scene.rs`（5 项：入口契约）

## 记录来源

与实现同批落地。`verify_module_mirror` 在实施过程中正确拦住了
`tyndall_scene` 缺镜像测试文件的问题——已补上。
