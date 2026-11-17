# 0029 — 对数导数递推替代复数幂级数

## 背景

[ADR-0025](0025-riccati-bessel-mie.md) 计算复参数 `psi_n(mx)` 用的是**幂级数**：

```text
j_n(z) = z^n/(2n+1)!! · Σ (-z²/2)^k / [(2k)!!(2n+2k+1)!!]
```

当时在 ADR-0025 里就标了这是**最脆弱的部分**：

> 大 `|z|` 时幂级数会抵消失精。米氏散射的常见参数范围内（`x < 50`）够用；
> 更大尺寸参数需要更好的算法（比如连分式）。

这一条把这个缺口补上。

## 决定

改用 Bohren–Huffman 的**对数导数**形式，删掉复数幂级数。

### 关键观察：米氏系数不需要 `psi_n(mx)`

`a_n` / `b_n` 里出现 `psi_n(mx)` 的地方，与分母共有一个因子，可以约掉。定义

```text
D_n(z) = psi_n'(z) / psi_n(z)
```

则（`psi_n(mx)` 被约掉后）：

```text
a_n = [(D_n(mx)/m + n/x)·psi_n(x) − psi_{n−1}(x)]
      ─────────────────────────────────────────────
      [(D_n(mx)/m + n/x)·xi_n(x)  − xi_{n−1}(x)]

b_n = [(m·D_n(mx) + n/x)·psi_n(x) − psi_{n−1}(x)]
      ─────────────────────────────────────────────
      [(m·D_n(mx) + n/x)·xi_n(x)  − xi_{n−1}(x)]
```

只剩**实参数**的 `psi_n(x)` / `xi_n(x)`（Miller 递推，稳定）和复参数的 `D_n(mx)`。

### `D_n` 的一阶递推

```text
D_{n−1}(z) = n/z − 1/(D_n(z) + n/z)
```

**向下**求解（`n` 从大到小）对任何 `|z|` 都稳定。起始条件用 `D_N ≈ 0` 当 `N ≫ |z|`。

起始阶取 `max(nmax, ⌈|z|⌉) + 25`：`⌈|z|⌉` 保证 `N ≫ |z|`，`+25` 保证递推收敛。

## 收益

| 方面 | 幂级数（旧） | 对数导数（新） |
| --- | --- | --- |
| 大 `\|z\|` | 相消失精，`x > 50` 不可信 | 稳定，测到 `x = 200` |
| 小 `\|z\|` | 需 200 项截断 | 递推自然收敛 |
| 代码量 | ~45 行 + 复贝塞尔辅助 | ~25 行 |
| 脆弱性 | ADR-0025 标为「最脆弱」 | 标准算法，无截断参数 |

**删掉了 3 个私有函数**：`spherical_bessel_j_complex`、`riccati_bessel_psi_complex`、
`psi_derivative_complex`。

## 等价性验证

所有既有判据在新实现下**全部仍然通过**：

| 判据 | 状态 |
| --- | --- |
| Clausius–Mossotti `Q_sca = (8/3)x⁴\|K\|²` | ✅ 1e-6 |
| `\|a_1\| = (2/3)k·x³` | ✅ 1e-6 |
| `\|b_1\|/\|a_1\| = O(x²)` | ✅ |
| 光学定理 `Q_ext == Q_sca` | ✅ 1e-12 |
| `m = 1` 零散射 | ✅ 1e-12 |
| `Q_ext(m=1.33, x=10) ≈ 2.2` | ✅ 1.5% |

这证明两种算法**在共同可用的区间内一致**，而新算法覆盖更宽。

## 新增的 4 项测试

| 测试 | 内容 |
| --- | --- |
| `logarithmic_derivative_matches_definition` | `D_k = psi_{k-1}/psi_k − k/x`（定义自洽） |
| `logarithmic_derivative_has_the_small_x_limit` | `D_1 ≈ 2/x − x/5` |
| **`mie_coefficients_stay_sane_at_large_size_parameter`** | `x = 50/100/200` 全部有限、`\|a\|,\|b\| <= 1.5`、`Q_ext → 2` |
| `logarithmic_derivative_rejects_invalid` | 参数校验 |

第三条是关键：它测的正是旧实现做不到的区间。

## 一个测试断言的修正

`zero_imaginary_part_means_no_absorption` 原本断言 `q_abs == 0.0`（精确相等）。
换算法后变成 `1.39e-17` —— 因为 `Q_abs = max(Q_ext − Q_sca, 0)`，两个几乎相等的
数相减留下浮点残差。

旧算法恰好给出精确 0 是**巧合**，不是不变式。断言改为 `q_abs <= 1e-12`。

这与 [ADR-0027](0027-complex-refractive-index.md) 的记录一致（那里已经说明
`Q_abs` 可能出现 `1e-16` 量级负值，钳到 0 是处理浮点误差而非物理问题）。

## 后果

- 米氏精确解在 `x` 的上界从 ~50 扩到 200+
- 代码更短，删掉了 ADR-0025 标记为最脆弱的部分
- 无 ABI 变化（三个 C ABI 签名不变）

## 没做的事

1. **`x > 200`**：未测。`MAX_MIE_ORDER = 200` 是阶数上限，`x` 更大时阶数不够
2. **`Q_abs` 的文献对照**：仍只有结构性判据（`>= 0`、`= Q_ext − Q_sca`）
3. **递推的误差估计**：没有严格的前向误差分析，只有实测稳定性

## 相关代码

- `crates/mps-formula/src/bessel.rs`（`logarithmic_derivative_series` + 重写的两个系数函数）
- `crates/mps-test/src/rapier/bessel.rs`（4 项新测试）
- `crates/mps-test/src/rapier/tyndall.rs`（1 处断言放宽）

## 记录来源

与实现同批落地。测试两轮通过：第一轮 `bessel` 17 项全过但 `tyndall` 有一项
因为精确零断言失败，改断言后全绿。

**这次没有发现新的物理错误** —— 因为判据（Clausius–Mossotti、光学定理、
`m=1` 归零、文献对照）都是前面几批已经验证过的可靠判据，算法替换由它们兜底。
