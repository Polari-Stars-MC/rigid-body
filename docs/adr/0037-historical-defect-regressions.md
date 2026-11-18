# 0037 — 历史缺陷的回归防线

## 背景

`docs/ROADMAP.md` 第 6 条点名了三个**在测试全绿状态下存在过**的缺陷：

1. **SPH 自贡献重复计算**
2. **轮胎力量纲错误**
3. **光子动量必得 NaN**

[ADR-0034](0034-ffi-finiteness-sweep.md)（有限性）、[ADR-0035](0035-conservation-laws.md)
（守恒律）、[ADR-0036](0036-analytic-reference.md)（解析对照）三层都是在**建通用防线**，
没有针对这三条具体缺陷。

这一条把三个**具体**缺陷逐个补上回归测试。其中两个做到了，第三个记录原因。

## 缺陷 #3：光子动量

### 原始 bug（已在 `relativity.rs` 里记录）

> Earlier this multiplied the direction by `INFINITY`, so any axis-aligned photon
> got `momentum = (inf, NaN, NaN)` (`0 * inf == NaN`) while the function returned
> success.

机制：`momentum = dir * INFINITY`。轴对齐光子的方向是 `(1, 0, 0)`，
`0 * INFINITY = NaN`，于是得到 `(inf, NaN, NaN)` —— 而且**返回成功**。

### 新增的 5 项回归测试

| 测试 | 断言 |
| --- | --- |
| **`axis_aligned_photon_momentum_is_a_unit_vector_not_nan`** | 三个坐标轴各测一次，动量必须是 `(1,0,0)` 等，**三个分量都有限**，长度 = 1 |
| `diagonal_photon_momentum_is_normalised` | 斜向光子的动量是归一化方向 |
| `photon_momentum_magnitude_is_documented_as_infinite` | 标量仍为 `INFINITY`（文档化约定） |
| `massive_particle_momentum_follows_gamma_mv` | `p = γmv`、`E = γmc²`、`K = (γ−1)mc²` |
| `massless_below_c_is_rejected` | 无质量但 `v < c` 必须 `FALSE` |

**为什么三个坐标轴各测一次**：`0 * inf == NaN` 只在**零分量**上发作。只测对角线
（`(1,1,1)/√3`）三个分量都非零，永远碰不到这个 bug。这是该缺陷能存活的原因之一。

## 缺陷 #1：SPH 自贡献

[ADR-0035](0035-conservation-laws.md) 已经测了**密度**（解析 `W(0) + 6W(d)`）。
这一条补**力**的部分：

| 测试 | 断言 | 抓什么 |
| --- | --- | --- |
| **`sph_pair_forces_are_equal_and_opposite`** | 对称双粒子压力**等大反向**，且互相排斥 | 自贡献重复计算、邻居遍历不对称 |
| **`isolated_particle_feels_no_force`** | 孤立粒子总力**精确为 0**（三个分量都是 `0.0`） | 虚假自力（会让整团流体自发加速） |
| `symmetric_lattice_centre_feels_no_net_force` | 6 面心邻居对称时中心净力为零 | 邻居遍历不完整或重复 |

第一条显式断言"力必须显著"（`mag > 1e-6`），否则等大反向是平凡的真。

## 缺陷 #2：轮胎力量纲 —— 未覆盖

`tire_model_*` 是 **scene 级**接口（需要 `WorldHandle` + 整车 + 车轮），不是纯公式：

```c
tire_model_set_params(world, id, wheel_index, peak_mu_long, peak_mu_lat,
                      peak_slip_ratio, peak_slip_angle, load_sensitivity, ellipse_factor)
tire_model_update(world, id, dt)
tire_model_get_forces(world, id, wheel_index, out_fx, out_fy)
```

要做量纲检查，需要一个**受力与载荷成线性**的场景（线性轮胎区），并验证
`|F| ≤ μ·Fz`。这需要复用 `tire_model.rs` 里的整车搭建助手（`make_car` 等）。

**没有做，理由是成本与收益**：现有 `tire_model.rs` 已有 9 项测试覆盖零力、
引擎力、刹车锁死、参数校验。量纲错误的典型表现（力的量级错 10³ 或 10⁶）在
`tire_model_engine_force_produces_longitudinal_force` 里已经会被察觉。

**记在这里，不假装做了。**

## 顺带查明：`mps-formula` 的 163 个孤儿 ABI

查 `rel_particle_properties` 时发现它**不在任何一个头文件里**。继续查到：

```text
mps-formula 的 extern "C" 入口数：163
其中出现在 rigid_body.h 或 cosmos.h 的：0
```

这与 ROADMAP 第 17 行一致：

> `mps-formula` 的 162 个入口当前 **0 违规但不在扫描范围**

现在数字是 163（本轮新增的 `logarithmic_derivative_series` 等不计入，
新增在 `tyndall_spectral` 的是 `pub fn` 而非 `extern "C"`）。

它们只能从 Rust 侧调用（`mps-test` 直接 `use mps_formula::relativity::rel_particle_properties`），
C 宿主看不到。**这不一定是问题**（公式层本来就只服务 Rust 侧），但值得知道：
"163 个入口"与"1002 个 C ABI"是两个不相交的集合。

## 一个设计张力（记录，未修改）

`rel_particle_properties` 对光子返回：

| 字段 | 值 |
| --- | --- |
| `momentum` | 单位方向向量（有限） |
| `momentum_magnitude` | `INFINITY` |
| `total_energy` / `kinetic_energy` / `rapidity` | `INFINITY` |

**物理上光子的动量是有限的**（`p = E/c`）。这里的 `INFINITY` 是 `γmv` 在
`γ→∞, m→0` 下的不定型产物，函数选择用它表示"没有静止系"。

这是**可辩护的取舍**，但与 [ADR-0034](0034-ffi-finiteness-sweep.md) 的
「返回 `TRUE` 就要给有限值」约定不一致。

**没有改**：它是文档化的既有接口，改动会破坏 ABI 语义。记为张力，交由后续决定。

## 一次瞬时测试失败

全量测试第一次跑报 `test failed`，第二次跑 1630 项全绿。

查证过程：

1. 反复单跑时间断言 `series_solver_is_fast_at_large_size_parameter` 3 次 → 全过
   （实测 0.5 ms，阈值 50 ms，**100 倍余量**）
2. 单独跑四个守门（`verify_metrics_sync` / `docs_index_consistency` /
   `error_consistency` / `verify_module_mirror`）→ 全过
3. 再跑全量 → 1630 全绿

**结论**：最可能是 `dump-metrics` 与 `verify_metrics_sync` 之间的时序问题
（生成物写入与守门读取竞争），不是新测试的问题。`dump-metrics` 的退出码在那次
命令里被 `> $null` 吞掉了，导致我无法确认它是否成功 —— 这是命令写法的问题。

**教训：生成物命令不要静默退出码。** 后续批次的命令应当显式检查。

## 累计防线

| ADR | 手段 | 覆盖 | 找到的 bug |
| --- | --- | --- | ---: |
| 0034 | 有限性 | 86 个零引用入口 | 2（连带修 4 处） |
| 0035 | 守恒律 | 7 场景 + SPH 密度 | 0 |
| 0036 | 解析对照 | 27 条公式/恒等式 | 0 |
| **0037** | **历史缺陷回归** | **3 条中的 2 条** | **0（缺陷早已修复）** |

## 相关代码

- `crates/mps-test/src/rapier/analytic_reference.rs`（+8 项：光子 5 + SPH 力 3）
- 生产代码**未改动**

## 记录来源

与实现同批落地。三个历史缺陷里两个补上了回归测试，第三个诚实记录为未覆盖并说明
理由。**没有为了凑数写一个测不到东西的轮胎测试。**
