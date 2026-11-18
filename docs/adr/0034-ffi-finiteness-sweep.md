# 0034 — 零引用 C ABI 的有限性扫描

## 背景

`docs/ROADMAP.md`「还开着」第 6 条：

> **正确性没有下界。** 测试全绿只证明「没崩、没变」，不证明「算对了」……
> 缺的是解析解对照、守恒律断言，以及**既无 JNI 绑定也无测试引用**的入口的
> 基本有限性检查。

[ADR-0025](0025-riccati-bessel-mie.md)~[0033](0033-per-step-exact-integration.md)
那一批证明了「解析解对照」这条路的威力（找出 5 个真 bug），但那只覆盖了
Tyndall 一个模块。这一条处理**面**：143 个从未被任何测试碰过的入口。

## 现状量化

能力地图实测：

| 项 | 数量 |
| --- | ---: |
| `mps-core` C ABI 入口总数 | 1002 |
| 被 `mps-jni` 引用 | 613 |
| 被 `mps-test` 引用 | 753 |
| **既无 JNI 绑定也无测试引用** | **143** |

143 个入口分布在 15 个模块：

| 模块 | 数量 | 模块 | 数量 |
| --- | ---: | --- | ---: |
| `emag` | 26 | `acoustics_ffi` | 7 |
| `rel` | 22 | `world` | 6 |
| `qphys` | 21 | `terrain_gravity` | 6 |
| `astrocalc` | 18 | `anvilkit` | 4 |
| `joints` | 13 | `aerodynamics` | 3 |
| `nucphys` | 10 | `voxel` / `rigid_body` / `tyndall_scene` | 2 各 |
| | | `trajectory` | 1 |

## 决定

新增 `crates/mps-test/src/rapier/ffi_finiteness.rs`：对签名形状**严格为**
`struct Bool f(double…, double *out)` 的 **86 个**入口做表驱动扫描。

### 为什么不手写 86 个用例

签名统一就能用函数指针表：

```rust
type F2 = extern "C" fn(f64, f64, *mut f64) -> Bool;
const A2: &[(&str, F2)] = &[("astrophysics_transit_depth", …), …];
```

六个 arity（0~5 个 `double`）各一张表，六个测试各扫一张。

### 输入向量池

| # | 向量 | 目的 |
| --- | --- | --- |
| 1 | 全 `1.0` | 典型 |
| 2 | `[0.5, 2.0, …]` | 混合 |
| 3 | `[1e-3, 1e3, 1.0, 1e6, …]` | 跨 6 个数量级 |
| 4 | 天文/微观量级 | 触发溢出 |
| 5 | 全 `0.0` | 非法域 |
| 6 | 全 `-1.0` | 非法域 |
| 7 | 含 `NaN` | 非法 |
| 8 | 含 `INFINITY` | 非法 |

### 唯一断言

```text
返回 TRUE  ⟹  输出必须有限
```

返回 `FALSE` 是**合法**结果（输入被拒绝），不断言 —— 各入口的合法域不同，
统一输入不可能全部命中。

### 一次报全部，不是报第一个

第一版用 `assert!` 直接炸，一次只暴露一个问题。改成收集 `Vec<String>` 再
统一断言，一次运行列出所有违规。

**这个改动很关键**：它把「修一个、跑一次、再修一个」变成「跑一次看全貌」。

## 找到的 2 个真 bug

### 1. `astrophysics_blackbody_spectral_radiance`：`exp(x) − 1` 相消

输入 `λ = 1.989e30 m`（天文量级向量）时：

```text
exponent = hc/(λ·kb·T) ≈ 1e-41
exp(1e-41) 精确等于 1.0      ← 相消
exp(x) - 1 = 0              ← 分母为 0
radiance = 2hc²/λ⁵ / 0 = inf
```

函数返回 `TRUE` 且输出 `inf`。

**这是经典的 `exp(x) − 1` 精度问题**，标准修法是 `exp_m1`（无相消实现）。

全仓库 grep 出 **3 处**同类模式，全部修复：

| 文件 | 行 |
| --- | --- |
| `astrophysics.rs` | `blackbody_spectral_radiance` |
| `max_planck.rs` | Planck 分布 |
| `satyendra_nath_bose.rs` | Bose–Einstein 分布 |

修复前 `exp_m1` 在仓库里出现 **0 次**。

### 2. `astrophysics_mass_luminosity_relation`：`powf` 溢出

```rust
Some(mass_solar.powf(exponent))   // 1.989e30^6.96e8 = inf
```

数学上「L → ∞」没错，但 ABI 契约是「返回 `TRUE` 就意味着拿到了可用数字」。
加 `is_finite()` 兜底。

## 这两个 bug 的共同点

**都是「算完直接 `Some(x)`，不检查 `x` 是否可表示」。**

输入校验（`finite_many`、`> 0`）都有，但**输出**没有校验。这是一个系统性盲区：

| 校验了 | 没校验 |
| --- | --- |
| 参数有限 | 结果有限 |
| 参数在合法域 | 中间量溢出 |
| 参数非零 | 相消导致除零 |

## 这不是「算对了」的证明

有限性只是**下界中的下界**。它抓不到：

- 量纲错误（轮胎力量纲那类）
- 公式写反、系数漏乘（`a_n` 的 `m²` vs `m` 那类）
- 物理约束违反（`σ_ext < σ_sca` 那类）

那些需要解析解对照与守恒律 —— 即 [ADR-0025](0025-riccati-bessel-mie.md)~
[0033](0033-per-step-exact-integration.md) 用的方法。

**两个方法互补，不能互相替代。**

## 剩下的 57 个入口

签名不统一，无法表驱动：

| 原因 | 例子 |
| --- | --- |
| 需要 `WorldHandle` | `world_*`、`joints`、`terrain_gravity` |
| `int32_t` / `struct Bool` 参数 | `quantum_spherical_harmonic_real`、`relativity_kerr_isco` |
| 多个输出指针 | `tyndall_wavelength_to_rgb`（3 个）、`tyndall_mie_efficiencies_complex`（3 个） |
| 返回 `u32`/`u64` 而非 `Bool` | `tyndall_max_scatter_orders` |

其中 `tyndall_*` 已有场景级测试；其余仍无覆盖。

## 一个守卫改动

新测试文件没有对应的生产侧模块，所以加进了
`verify_module_mirror.rs` 的 `test_only_extras` 允许表（与 `error_consistency`、
`arena_compat` 同类），并在表里写了理由。

**这不是「静默关守卫」** —— 该表本身就是为了登记这类跨模块 meta 测试而存在的，
条目带注释说明。

## 后果

- 86 个从未被测试碰过的入口现在有了**回归防线**（`NaN`/`inf` 输入下不得静默
  产出非有限值）
- 修复了 3 处 `exp(x) − 1` 相消（不只 Tyndall，还有两个量子/热统模块）
- 修复了 1 处 `powf` 溢出
- 143 → 57 个入口仍无覆盖

## 没做的事

1. **57 个签名不统一的入口**：需要逐个写 harness，或先收敛签名
2. **输出量纲/物理约束检查**：有限性不覆盖这些
3. **更多输入向量**：8 组是成本与覆盖的折中，可以扩到随机采样（fuzz）

## 相关代码

- `crates/mps-test/src/rapier/ffi_finiteness.rs`（217 行，86 个入口 × 8 组向量）
- `crates/mps-formula/src/astrophysics.rs`（2 处修复）
- `crates/mps-formula/src/max_planck.rs` / `satyendra_nath_bose.rs`（`exp_m1`）
- `crates/mps-test/src/rapier/verify_module_mirror.rs`（允许表加一行）

## 记录来源

与实现同批落地。扫描一次运行就报出全部违规（改进了「报第一个就停」的初版），
两个 bug 都是 `Some(结果)` 缺输出校验，属于同一类盲区。
