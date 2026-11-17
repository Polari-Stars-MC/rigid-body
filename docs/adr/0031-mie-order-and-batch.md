# 0031 — 放开尺寸参数上限与批量米氏求解

## 背景

[ADR-0029](0029-logarithmic-derivative.md) 用对数导数递推替换了复数幂级数，让米氏
系数在大 `|z|` 时不再相消失精。但当时留了两个限制：

1. **`MAX_MIE_ORDER = 200`** —— 阶数上限，对应 `x ≲ 190`
2. **`mie_coefficient_a` / `mie_coefficient_b` 每次调用都重算整个递推** ——
   循环 `n = 1..=nmax` 就变成 `O(nmax²)`

第二条在 `x = 200` 时是 200² = 4 万次递推步，`x = 1000` 时是 100 万次。
这是放开上限的真正障碍。

这一条把两个都解决。

## 决定

### 1. `MAX_MIE_ORDER` 200 → 2000

对应 `x ≲ 1900`，即 550 nm 下直径约 330 µm 的颗粒（毛毛雨量级）。

### 2. Miller 起始阶必须越过转折点

`j_n(x)` 在 `n < x` 时振荡，`n > x` 时指数衰减。Miller 向下递推**只在衰减区
起点有效** —— 起点落在振荡区会得到错误的归一化。

旧代码：

```rust
let start = nmax + 20 + (x as usize).min(200);   // 上限 200，x 大时不够
```

新代码：

```rust
let start = (nmax + 20).max((x.ceil() as usize) + 30);
```

`⌈x⌉ + 30` 保证起点确实在衰减区；`nmax + 20` 保证覆盖所需阶数。

**这个 `min(200)` 是旧实现隐藏的第二个上限** —— 即使把 `MAX_MIE_ORDER` 调大，
`x > 200` 仍会算错。两个上限必须同时放开。

### 3. 批量求解：`O(nmax²)` → `O(nmax)`

新增：

```rust
pub fn mie_coefficients_series(m: Complex, x: f64, nmax: u32)
    -> Option<(Vec<Complex>, Vec<Complex>)>      // 全部 a_n, b_n

pub fn mie_efficiencies_series(m: Complex, x: f64)
    -> Option<(f64, f64, f64)>                    // (Q_ext, Q_sca, Q_abs)
```

三个级数（`psi`、`xi`、`D_n`）各算**一次**，然后只做 `O(nmax)` 的组装。

`tyndall::mie_efficiencies_complex`、`mie_scattering_efficiency_exact`、
`mie_extinction_efficiency_exact` 全部改用它。

## 实测

| `x` | `Q_ext` | 耗时 |
| ---: | ---: | ---: |
| 200 | 2.0557 | 413 µs |
| 500 | 2.0323 | 260 µs |
| 1000 | 2.0196 | 509 µs |

`Q_ext` 单调下降趋于 2 —— 消光佯谬在 `x = 1000` 仍然成立。

**耗时几乎不随 `x` 增长**（500 µs 量级），因为 `O(nmax)` 里 `nmax ≈ x`，
线性增长，而常数很小。

## 新增的 5 项测试

| 测试 | 断言 |
| --- | --- |
| `extinction_paradox_holds_up_to_x_1000` | `x = 200/500/1000` 时 `\|Q_ext − 2\| < 0.1`，且单调下降 |
| `series_solver_matches_single_order_calls` | 批量结果与逐阶调用**逐位相同**（1e-12） |
| `series_solver_is_fast_at_large_size_parameter` | `x = 1000` 在 50 ms 内 |
| `thin_medium_matches_the_analytic_formula` | 见 ADR-0033 |
| `brightness_grows_monotonically_with_thickness` | 见 ADR-0033 |

第二条是关键：它保证批量版与单阶版**不是两套实现**，只是同一算法的缓存。

## 一个踩过的坑（记录）

改这段时用 PowerShell 双引号字符串写 Rust 注释，里面的反引号被 PowerShell
当成转义符，把 `` `n < x` `` 里的 `` `n `` 变成了换行，`bessel.rs` 第 147–150 行
被写成无法解析的乱码。

**教训：给 Rust 文件插入含反引号的内容时，用单引号 here-string，或者用
`IndexOf` 定位后按行重建，不要用双引号字符串拼接。**

## 后果

- 米氏精确解的 `x` 上界从 ~190 提到 ~1900
- 大 `x` 的求解开销从秒级降到亚毫秒
- 无 ABI 变化

## 没做的事

1. **`x > 1900`**：`MAX_MIE_ORDER = 2000` 仍是硬上限。再往上需要动态阶数
   （`nmax` 随 `x` 增长，`Vec` 分配会成为瓶颈）
2. **并行化**：`n` 之间的组装互相独立，可以并行；但 Rust 标准库没有轻量线程池，
   而 `mps-formula` 不能加依赖

## 相关代码

- `crates/mps-formula/src/bessel.rs`（`MAX_MIE_ORDER`、Miller 起始阶、两个批量函数）
- `crates/mps-formula/src/tyndall.rs`（三个函数改用它）
- `crates/mps-test/src/rapier/tyndall.rs`（3 项新测试）
