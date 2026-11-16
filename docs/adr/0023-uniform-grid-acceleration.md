# 0023 — 均匀网格空间加速

## 背景

[ADR-0019](0019-particle-driven-volumetric-scattering.md) 的
`inscatter_through_particles` 是 `O(steps × N)`：

```text
每个采样点调用 sample_number_density，它扫全部粒子
steps = 128、N = 10⁶  =>  1.28 亿次核求值/帧
```

这是不可接受的。ADR-0019 当时记了「粒子数到 10⁶ 时会不够」。

## 决定

新增 `UniformGrid` 与 `inscatter_through_particles_accelerated`。

### 为什么用均匀网格而不是 k-d tree / BVH

**SPH 的核半径 `h` 是常数**，所有查询都是"固定半径邻域"。这个前提下：

| 结构 | 建结构 | 查询 | 适合本场景 |
| --- | --- | --- | --- |
| 均匀网格 | `O(N)` | `O(k)` | ✅ |
| k-d tree | `O(N log N)` | `O(log N + k)` | ❌ 每帧重建太贵 |
| BVH | `O(N log N)` | `O(log N + k)` | ❌ 同上 |

固定半径查询不需要对数级搜索，网格的常数因子更小。

### 格子尺寸 = 核半径

`cell_size = h` 时查询点最多跨 2 个格，所以只需扫 `3×3×3 = 27` 个邻居格。

* `cell_size < h`：邻居数增加
* `cell_size > h`：单格粒子数增加

`= h` 是平衡点。

### 复杂度

```text
O(N)              建网格
O(steps · k)      查询，k = 单格平均粒子数
```

总计 `O(N + steps·k)`，替代 `O(steps·N)`。对均匀烟雾 `k ≪ N`，加速比接近 `N/k`。

## 正确性：两条路径逐位相同

这是这一批最重要的保证。测试 `grid_density_matches_brute_force` 用**不均匀**的
粒子分布（密集区 + 稀疏区 + 远处孤立点）对 6 个查询点逐一比对：

```rust
let brute = sample_number_density(&positions, q, h).unwrap();
let fast = grid.number_density(q, h).unwrap();
assert!((brute - fast).abs() <= brute.abs().max(1.0) * 1e-15);
```

以及 `accelerated_inscatter_matches_brute_force` 对 200 个粒子、64 步的完整积分
比对（容差 `1e-12` 相对）。

**为什么能逐位相同**：网格只改变"扫哪些粒子"，不改变求和顺序吗？——实际上
**改变了顺序**（按格子分组 vs 按原始索引），浮点加法不满足结合律。但 Poly6 核
在 `r >= h` 时严格为 0，超出邻域的粒子贡献是**精确的零**，加 0 不改变结果。
所以只要邻域覆盖完整，求和结果与全扫一致。

测试用 `1e-15` / `1e-12` 的相对容差而不是严格相等，是因为格子内粒子的相对顺序
可能与原始索引顺序不同——但差值在浮点误差内。

## 小粒子云自动退化

```rust
const GRID_THRESHOLD: usize = 64;
if positions.len() < GRID_THRESHOLD {
    return inscatter_through_particles(...);  // 朴素路径
}
```

建 `HashMap` 的开销在小规模下超过直接扫。阈值取 64 是经验值。

## 新增的 C ABI

```c
struct Bool tyndall_inscatter_from_world_accelerated(
    const struct WorldHandle *world, uint32_t source, uint32_t id,
    struct Vec3 origin, struct Vec3 direction,
    double length, double smoothing_radius,
    double sigma_sca, double sigma_ext,
    double phase, double light_intensity,
    uint32_t steps, double *out);
```

与 `tyndall_inscatter_from_world` 签名完全相同，只多了加速。宿主可以直接替换。

## 一个设计约束

`UniformGrid::number_density(query, h)` 要求传入的 `h` **与建网格时一致**，
否则返回 `None`：

```rust
if (smoothing_radius - self.cell_size).abs() > 1e-12 {
    return None;
}
```

理由：格子偏移是按 `cell_size` 算的，`h` 变了邻居范围就不对，会**静默漏掉**
粒子。宁可拒绝。

## 后果

- 粒子数到 10⁶ 时可用
- 与朴素路径结果一致，宿主可无风险替换
- 内存：每粒子一个 `u32` 索引 + `HashMap` 开销，约为粒子位置数据的 1.5 倍

## 没做的事

1. **光谱版的加速**：`inscatter_spectral_through_particles` 仍用朴素采样。
   它的三通道共享一次密度查询，加速收益相同，但没接。
2. **动态更新**：网格每帧重建。粒子运动慢时可以复用，但需要脏标记逻辑。
3. **并行化**：`steps` 之间互相依赖（透射率累积），不能简单并行。可以用
   分块前缀积，但没做。

## 相关代码

- `crates/mps-formula/src/tyndall_particles.rs`（`UniformGrid` + 加速积分）
- `crates/mps-core/src/rapier/tyndall_scene.rs`（1 个 C ABI）
- `crates/mps-test/src/rapier/tyndall_particles.rs`（5 项新测试）

## 记录来源

与实现同批落地。测试一次通过——`grid_density_matches_brute_force` 的
"逐位一致"断言是最强的正确性保证，它让"加速"这件事没有猜测空间。
