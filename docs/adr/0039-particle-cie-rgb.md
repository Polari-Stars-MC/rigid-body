# 0039 — 粒子云 + CIE 积分

## 背景

丁达尔链条的能力一直是**二选一**：

| 入口 | 粒子驱动 | 连续光谱 |
| --- | :---: | :---: |
| `inscatter_spectral_full`（[ADR-0024](0024-full-tyndall-composition.md)） | ✅ | ❌ 三离散波长 |
| `inscatter_rgb_uniform`（[ADR-0032](0032-cie-weighted-integration.md)） | ❌ 均匀介质 | ✅ |

[ADR-0032](0032-cie-weighted-integration.md) 明确记了这个缺口：

> 要「粒子云 + CIE 积分」的组合，当前需要宿主自己循环波长 —— 没有单入口。

这一条补上。同时也是「按推荐转去加功能」的第一个动作。

## 决定

新增 `inscatter_rgb_from_particles` + C ABI `tyndall_rgb_from_world`。

### 计算顺序不可交换

```text
for each λ:  x = πd/λ  →  米氏  →  σ_sca(λ), σ_ext(λ)
             粒子云沿视线积分  →  L(λ)          ← 非线性，必须先算
然后:        CIE 梯形积分 L(λ)  →  XYZ  →  线性 sRGB   ← 线性，可以后算
```

**不能先对 `σ` 积分再算一次散射。** 散射亮度对 `σ` 是非线性的
（`1 − exp(−τ)` 与 Beer–Lambert 累积），顺序反了量级就错。

这条在三个地方都写进了文档（本 ADR、函数文档、MC 指南），因为它是这类
函数最容易犯的错。

### 颗粒质量从世界里读

`tyndall_rgb_from_world` 用已有的 `particle_mass(world, source, id)` 取质量，
宿主不用提供。签名因此比 `inscatter_rgb_from_particles` 少一个参数。

## 新增的 7 项测试

### 公式层 5 项

| 测试 | 断言 |
| --- | --- |
| `particle_cie_rgb_fine_particles_are_blue` | 10 nm 颗粒的粒子云 → 偏蓝（瑞利区） |
| `particle_cie_rgb_large_particles_are_neutral` | 5 µm 颗粒 → 三通道比值 < 1.8（几何光学区） |
| `particle_cie_rgb_is_black_without_particles` | 空粒子云 → 全零 |
| `particle_cie_rgb_soot_is_dimmer_than_water` | `κ=0.5` 的烟灰比 `κ=0` 的透明颗粒暗 |
| `particle_cie_rgb_rejects_invalid` | 质量非正 / 方向未归一化 / 直径非正 / 采样不足 |

细颗粒偏蓝、大颗粒中性这两条形成**尺寸对照** —— 它们验证的是
「粒径决定颜色」这条物理规律在**粒子云**路径上也成立，而不只是在均匀介质路径上。

### ABI 层 2 项

按 `tyndall_scene.rs` 的既定约定（只覆盖入口契约，物理归公式层）：

| 测试 | 断言 |
| --- | --- |
| `rgb_from_world_writes_three_outputs` | 成功时三输出全写、全非负、全有限 |
| `rgb_from_world_rejects_invalid_without_writing` | null 输出 / 未归一化方向 / 未知来源 → `FALSE` 且**不写** |

## 性能：每个波长重建一次网格

`inscatter_through_particles_accelerated` 内部按需建 `UniformGrid`，而粒子位置
**与波长无关**。当前实现每个波长各建一次 —— `samples` 次 `O(N)` 建格。

对 `samples = 15`、`N = 10⁶`，这是 15 × 10⁶ 次插入，相比每波长
`O(steps·k)` 的积分开销可以接受，但**不是最优**。

优化路径明确：把 `UniformGrid` 提到循环外，让所有波长共享。需要给
`inscatter_through_particles_accelerated` 加一个「接受已有网格」的变体。
**没做**，记在这里。

## 后果

- 丁达尔链条上**没有能力缺口了**：粒子驱动、分通道、多次散射、空间加速、
  CIE 积分可以任意组合
- `tyndall_*` C ABI 从 31 增至 **32** 个
- MC 指南更新了「没有单入口」那段，改成给出新入口的签名

## 没做的事

1. **网格复用**（见上）
2. **多次散射 + CIE**：`inscatter_rgb_from_particles` 是单次散射
   （`inscatter_through_particles_accelerated` 不带 `orders`）。
   [ADR-0022](0022-multiple-scattering.md) 的级数在分通道版本里可用，
   但没接进 CIE 路径
3. **粒子云 + CIE 的性能测试**：没有 `samples`/`steps` 的耗时矩阵

## 相关代码

- `crates/mps-formula/src/tyndall_spectral.rs`（`inscatter_rgb_from_particles`）
- `crates/mps-core/src/rapier/tyndall_scene.rs`（`tyndall_rgb_from_world`）
- `crates/mps-test/src/rapier/tyndall_spectral.rs`（5 项）
- `crates/mps-test/src/rapier/tyndall_scene.rs`（2 项）
- `docs/tyndall-mc-integration.md`（入口签名）

## 记录来源

与实现同批落地。测试两轮通过：第一轮只有 import 顺序问题（`cargo fmt` 之前
手工插入的符号没被找到），修好后 51 项 `tyndall_spectral` 全绿。