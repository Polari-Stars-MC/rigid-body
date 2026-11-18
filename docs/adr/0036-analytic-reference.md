# 0036 — 解析解对照（热力学 / 相对论 / 等离子体）

## 背景

前三层各抓一类缺陷，互为补充：

| 层 | 手段 | 抓什么 | 抓不到 |
| --- | --- | --- | --- |
| [0034](0034-ffi-finiteness-sweep.md) | 有限性 | `TRUE` 却给 `NaN`/`inf` | 数值有限但算错 |
| [0035](0035-conservation-laws.md) | 守恒律 | 总量漂移 | **单个公式写错** |
| 本条 | 解析解对照 | **公式本身错** | 极端参数的数值稳定性 |

一个把 `1 − Tc/Th` 写成 `Tc/Th` 的函数：输出完全有限，不违反任何守恒律，
但它就是错的。**只有从物理第一原理写出期望值才能抓到。**

## 两条方法原则

### 1. 期望值来自物理，不来自实现

先写 `η = 1 − Tc/Th`，再跑测试。**绝不先读实现抄一遍** —— 那只会把 bug 一起
抄进断言。

这条原则在这一轮救过一次：写 `from_mie` 测试时如果先读了实现，就发现不了
[ADR-0028](0028-mie-normalisation-fix.md) 的 `m²` vs `m`。

### 2. 优先用恒等式，其次用绝对值

恒等式不依赖符号约定、参数顺序、单位制，写错的概率最低，且**能同时验证多个
函数**。这一批 27 项里有 11 项是恒等式。

## 本次覆盖

### 热力学 10 项

| 测试 | 判据 |
| --- | --- |
| `carnot_efficiency_matches_the_textbook_formula` | `η = 1 − Tc/Th`，且 `0 ≤ η < 1` |
| **`refrigeration_cop_is_consistent_with_carnot_efficiency`** | **`COP_R = (1−η)/η`** |
| **`heat_pump_cop_is_refrigeration_cop_plus_one`** | **`COP_HP = COP_R + 1`** |
| `otto_efficiency_matches_the_textbook_formula` | `η = 1 − r^(1−γ)`，且随 `r` 单调 |
| `ideal_gas_law_matches_pv_equals_nrt` | `P = nRT/V` |
| **`ideal_gas_solvers_form_a_closed_loop`** | **`P → V → T` 三步闭合** |
| `entropy_change_at_constant_volume_matches_the_formula` | `ΔS = n·Cv·ln(T2/T1)`，且可逆 |
| `prandtl_number_matches_the_formula` | `Pr = cp·μ/k`，空气 ≈ 0.7 |
| `reynolds_number_matches_the_formula` | `Re = ρvL/μ`，且对 `v` 线性 |
| `van_der_waals_critical_point_matches_the_formulas` | `(Tc,Pc,Vc)` + **`Zc = 3/8`** |

`Zc = Pc·Vc/(R·Tc) = 3/8` 是范德瓦尔斯气体的普适常数 —— 与 `a`、`b` 都无关，
所以它比单看三个临界值更难蒙对。

### 相对论 12 项

| 测试 | 判据 |
| --- | --- |
| `gamma_benchmark_is_exact` | `β=0.6 ⟹ γ=1.25`（精确有理数，作为后续基准） |
| `relativistic_energy_and_momentum_match_the_formulas` | `E=γmc²`、`p=γmv` |
| **`relativistic_dispersion_relation_holds`** | **`E² − (pc)² = (mc²)²`（6 个 β）** |
| **`energy_from_momentum_agrees_with_total_energy`** | **两条路径算同一个 `E`** |
| `at_rest_energy_is_rest_mass_and_momentum_is_zero` | `E=mc²`、`p=0` |
| **`longitudinal_doppler_is_reciprocal`** | **`f⁺·f⁻ = f²`** |
| `cosmological_redshift_matches_the_scale_factor_formula` | `z = 1/a − 1` |
| `wavelength_redshift_matches_the_definition` | `z = (λobs−λemit)/λemit` |
| **`schwarzschild_radii_have_the_textbook_ratio`** | **`ISCO/光子球 = 2`**（与 `M,G,c` 无关） |
| `gravitational_redshift_has_the_weak_field_limit` | 弱场 `z ≈ GM/(rc²)` |
| **`chirp_mass_matches_the_formula_and_is_symmetric`** | **对称性 + 等质量极限 `m/2^(1/5)`** |
| `hubble_recession_is_linear_in_distance` | `v = H0·d`（纯乘积，单位由调用方保证） |

色散关系那条是最强的：它把 `relativistic_total_energy`、`relativistic_momentum`、
`relativistic_energy_from_momentum` **三个独立函数**绑在一个等式里。任一个写错
就破，而且它与符号约定、单位制完全无关。

多普勒的互易性 `f⁺·f⁻ = f²` 同样：不依赖「`approaching = true` 是蓝移还是红移」
这个约定。

### 等离子体 5 项

| 测试 | 判据 |
| --- | --- |
| **`larmor_radius_times_gyrofrequency_is_perpendicular_velocity`** | **`r_L·ω_c = v_⟂`** |
| **`loss_cone_angle_is_the_inverse_of_the_mirror_ratio`** | **`sin²θ_lc · R_m = 1`** |
| **`magnetosonic_speeds_satisfy_the_dispersion_identity`** | **`v_fast² + v_slow² = c_s² + v_A²`** |
| `magnetosonic_speeds_have_the_right_limits` | 平行/垂直两个极限 + Alfvén 波速 `v_A·cosθ` |
| `plasma_beta_matches_the_formula` | `β = 2μ₀nkT/B²`，对 `T` 线性、对 `B²` 反比 |

`v_fast² + v_slow² = c_s² + v_A²` **与传播角无关** —— 快慢磁声波是同一个二次方程的
两个根，所以平方和恒等于系数和。这条抓不到单点错误，但能抓「快慢波公式互换」。

### 顺带补上一个被忽略的返回值

写磁声波测试时注意到 `mhd_wave_speeds` 返回**三个**值，而我只用了两个。
中间的剪切 Alfvén 波速补测：

* 平行（`θ=0`）：`= v_A`
* 垂直（`θ=90°`）：`= 0`（不能垂直传播）

**返回值被忽略就等于没有测试** —— 这是这一批唯一的「新增覆盖」而非「验证」。

## 诚实结论：零 bug

这一批 **27 项全部一次通过，没有找到缺陷**。

热力学、相对论、等离子体三个模块在这些公式上是正确的。

### 这与前三批的对比

| 批次 | 测试 | 找到的 bug |
| --- | ---: | ---: |
| [0034](0034-ffi-finiteness-sweep.md) 有限性 | +7 | **2**（连带修 4 处） |
| [0035](0035-conservation-laws.md) 守恒律 | +10 | 0 |
| 本条 解析解对照 | +27 | 0 |

[ADR-0025](0025-riccati-bessel-mie.md)~[0033](0033-per-step-exact-integration.md)
那一批找到 5 个 bug，但那批是**在写新代码** —— 新代码本来就有 bug。
这三批是**验证已有代码**，而 `mps-formula` 的纯公式层经得起验证。

这个结果本身是有价值的信息：**`mps-formula` 的解析公式不是薄弱环节**。
薄弱环节在别处（ROADMAP 举的三个历史缺陷分别在 SPH 求解器、轮胎模型、光子
动量 —— 都是**多步耦合**，不是单条公式）。

## 这一批的真正收益

不是「找到 bug」，是**把这 27 条从"没人验证过"变成"有回归防线"**。

具体地：

* 任何对 `carnot_efficiency`、`otto_efficiency`、`relativistic_*` 的重构都会被
  立刻检查
* 恒等式那一组（色散关系、COP 关系、磁声波平方和）能抓到**跨函数的**不一致 ——
  这类缺陷靠单点测试永远抓不到

## 没做的事

1. **多步耦合模块**：ROADMAP 点名的三个历史缺陷都属于这一类。需要的是
   「端到端场景 + 解析解」，比单条公式难得多
2. **`acoustics` / `wave_optics`**：两个模块的纯公式在 `scientists/` 子模块里，
   本批未覆盖
3. **数值稳定性**：解析对照用"好"参数，不测极端值 —— 那是
   [ADR-0034](0034-ffi-finiteness-sweep.md) 的职责
4. **更多模块**：`stellar`、`nuclear`、`materials` 等仍未做解析对照

## 相关代码

- `crates/mps-test/src/rapier/analytic_reference.rs`（27 项测试）
- `crates/mps-test/src/rapier/verify_module_mirror.rs`（允许表加一行）
- 生产代码**未改动**（这一批无 bug 可修）

## 记录来源

与实现同批落地。零 bug 是诚实结论：断言都是 `1e-12` 容差下的解析值或
跨函数恒等式，不存在「写松了所以通过」的空间。
