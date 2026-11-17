# 0027 — 复折射率与吸收

## 背景

[ADR-0026](0026-exact-mie-integration.md) 接入了米氏精确解，但只暴露**实折射率**。

真实颗粒几乎都有吸收：

| 颗粒 | `n`（实部） | `κ`（虚部） |
| --- | ---: | ---: |
| 水滴 | 1.33 | 0 |
| 玻璃 | 1.5 | 0 |
| 矿物粉尘 | 1.5~1.6 | 0.001~0.01 |
| 烟灰 | 1.5~2.0 | 0.1~0.8 |
| 金属氧化物 | 2.0+ | 0.01~1.0 |

`bessel.rs` 的 `a_n` / `b_n` **本来就接受复数 `m`**（[ADR-0025](0025-riccati-bessel-mie.md)），
只是 `tyndall` 层没暴露。所以这一批改动很小。

## 决定

新增两个函数 + 两个 C ABI：

| 函数 | 返回 |
| --- | --- |
| `mie_efficiencies_complex` | `(Q_ext, Q_sca, Q_abs)` 三元组 |
| `single_scattering_albedo_complex` | `ω = Q_sca / Q_ext` |

### 为什么返回三元组而不是单值

`Q_ext = Q_sca + Q_abs`。宿主需要区分：

* **`Q_sca`** —— 有多少光被**散射出来**（这是光柱亮度的来源）
* **`Q_abs`** —— 有多少光被**转化成热**（这部分不产生光柱）
* **`Q_ext`** —— 总消光（决定穿透深度）

只给一个数会让宿主无法计算光柱亮度。

## 物理行为验证（8 项测试）

| 测试 | 断言 | 结果 |
| --- | --- | --- |
| `zero_imaginary_part_means_no_absorption` | `κ=0` 时 `Q_abs = 0` 且满足光学定理 | ✅ |
| `absorption_grows_with_imaginary_part` | `Q_abs` 随 `κ` 单调增长 | ✅ |
| **`scattering_decreases_with_absorption`** | **`Q_sca` 随 `κ` 增大而下降** | ✅ |
| `efficiency_inequalities_hold` | `Q_ext >= Q_sca`、`Q_abs = Q_ext - Q_sca` | ✅ |
| `albedo_reflects_absorption_strength` | `κ=0` 时 `ω=1`，强吸收时 `ω→0` | ✅ |
| `complex_matches_real_when_kappa_is_zero` | `κ=0` 时与实折射率版逐位一致 | ✅ |
| `complex_rejects_invalid` | `κ < 0` 被拒绝 | ✅ |
| `soot_beam_is_dimmer_than_water_beam` | 烟灰光柱比水滴暗 | ✅ |

### 最反直觉的一条：吸收让散射**下降**

`scattering_decreases_with_absorption` 断言 `Q_sca(κ=0.1) < Q_sca(κ=0)`。

直觉上"吸收是额外的事"，实际是**竞争**：入射能量要么被散射、要么被吸收，
吸收多了散射就少了。实测（`m=1.33`, `x=5`）：

| `κ` | `Q_ext` | `Q_sca` | `Q_abs` |
| ---: | ---: | ---: | ---: |
| 0 | 3.452 | 3.452 | 0 |
| 0.001 | 3.443 | 3.423 | 0.020 |
| 0.01 | 3.363 | 3.179 | 0.184 |
| 0.1 | 2.839 | 1.821 | 1.018 |

`Q_sca` 从 3.45 掉到 1.82，而 `Q_ext` 只从 3.45 降到 2.84 —— 吸收确实在
"抢"能量。

## 与 ADR-0020 的关系

[ADR-0020](0020-spectral-tyndall.md) 里 `single_scattering_albedo` 是
**参数化的**（宿主直接给 `σ_sca` / `σ_ext`）。这一批提供**物理来源**：

```text
颗粒的 n、κ、粒径、波长
    ↓ mie_efficiencies_complex
Q_sca / Q_ext
    ↓ × 几何截面 πr²
σ_sca / σ_ext
    ↓ 喂给 ADR-0020 的 spectral 函数
彩色光柱
```

也就是说：以前宿主得猜 `σ` 的比值，现在可以**从材料参数算出来**。

## 新增的 C ABI

```c
struct Bool tyndall_mie_efficiencies_complex(
    double size_param, double refractive_index_real, double refractive_index_imag,
    double *out_ext, double *out_sca, double *out_abs);

struct Bool tyndall_single_scattering_albedo_complex(
    double size_param, double refractive_index_real, double refractive_index_imag,
    double *out);
```

三个输出指针**全部成功才写**。

`tyndall_*` 系列现在共 **28 个入口**。

## 一个数值细节

`Q_abs = Q_ext - Q_sca` 在浮点下可能出现 `-1e-16` 量级的值（两个几乎相等的数
相减）。钳到 0 而不是报错：

```rust
let q_abs = (q_ext - q_sca).max(0.0);
```

这是浮点误差，不是物理问题。测试 `efficiency_inequalities_hold` 用
`1e-12` 容差验证三者自洽。

## 后果

- 宿主可以从**材料参数**（`n`、`κ`、粒径、波长）算出完整的米氏效率
- 烟灰/粉尘/水滴的光柱亮度差异有了物理依据
- 与光谱版组合后，可以做"不同材料在不同厚度下呈现不同颜色"

## 没做的事

1. **瑞利极限的 1.52 倍偏差**：仍未查清（ADR-0025）。复折射率版本同样受影响。
2. **多波长批量**：每次调用只算一个 `x`。做光谱时宿主需要循环三次（R/G/B）。
   可以加一个批量入口，但当前调用开销不大。
3. **`tyndall` 的旧 `single_scattering_albedo` 没标弃用**：保留向后兼容。

## 相关代码

- `crates/mps-formula/src/tyndall.rs`（2 个新函数）
- `crates/mps-core/src/rapier/emag.rs`（2 个 C ABI）
- `crates/mps-test/src/rapier/tyndall.rs`（8 项新测试）

## 记录来源

与实现同批落地。测试一次通过——物理约束（`Q_abs >= 0`、`Q_ext >= Q_sca`、
`κ=0` 退化为透明）都是**结构性**的，不依赖文献值，所以没有猜测空间。
