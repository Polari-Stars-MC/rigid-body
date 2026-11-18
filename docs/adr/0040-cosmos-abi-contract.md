# 0040 — `mps-cosmos` ABI 契约与三个无效 setter

## 背景：我的推荐前提是错的

上一轮我的建议是：

> `mps-cosmos` 的覆盖率是最大的结构性不均衡——28 个入口几乎没有测试纵深。

**两个数字都错，而且错得离谱。**

| 我的说法 | 实际 |
| --- | --- |
| 28 个入口 | **42** 个 |
| 「几乎没有测试纵深」 | cosmos 已有约 **6000 行**测试，含 `hohmann_transfer_validation`（336 行，教科书 Δv + 开普勒方程）、`convergence`（10 万步比能量守恒）、`drift_matrix`（vis-viva + Verlet/Yoshida4Kahan 对照）、`long_orbit`、`world`（1984 行） |

错误来源：第一次用了一个宽松的头文件正则，数出 28；第二次用 `\b` 边界匹配
`mps-test` 源码，数出 24 个"未覆盖"，但逐个直接 grep 之后是 **22**。
两次统计都不可靠。

**教训：报数字前先用手工抽查校验一次。** 我用 `cosmos_world_insert_body`
抽查时就发现它明明在 `bodies.rs` 里被引用，与统计结果矛盾 —— 那时就该停下重算。

## 真实缺口

42 个入口里 22 个在 `mps-test` 中零引用，集中在三类：

| 类别 | 数量 | 例子 |
| --- | ---: | --- |
| radio 子系统 | 8 | `cosmos_world_radio_step`、`radio_add_reflector` |
| builder setter | 5 | `set_gravity_scale`、`set_linear_damping`、`lock_translations` |
| world 配置 | 9 | `set_perturbation`、`set_central_body`、`set_sun_position`、`step_n` |

**不是物理缺口，是 ABI 面缺口。**

## 决定

新增 `crates/mps-test/src/cosmos/abi_contract.rs`，6 项测试。

### 通过的三项

| 测试 | 判据 |
| --- | --- |
| **`step_n_matches_repeated_step`** | `step_n(dt, 20)` 与 20 次 `step(dt)` 位置差 `< 1e-6`（且确实移动了 > 1 m，否则等价性平凡） |
| `body_mass_reflects_the_inserted_mass` | 插入 1234.5 kg → 读回 1234.5 |
| `builder_destroy_accepts_an_unused_builder` | 建了不插入，销毁不崩 |

`step_n` 那条能抓到：少跑/多跑一步、`dt` 被误当总时长、批次路径走了不同积分器。

## 关键发现：三个 setter 对轨道体无效

这是本条最有价值的部分 —— **而且不是 bug，是文档化的设计。**

`integrator.rs` 头部注释写明：

> 轨道积分（本模块）：位置/线速度由显式 Verlet 推进，**绕过 rapier 的力律**。
> rapier `step`：只跑碰撞检测、约束求解、姿态/角速度积分。为此 bodies 设
> `gravity_scale=0` 且不注入 `user_force`，由本模块在 step 前后**直接写回**
> `translation`/`linvel`。

后果：

| 入口 | 为什么对轨道体无效 |
| --- | --- |
| `cosmos_builder_set_gravity_scale` | 引力来自 n-body Verlet 路径，不看 rapier 的 scale |
| `cosmos_builder_set_linear_damping` | rapier `step` 施加阻尼，随即被积分器写回的 `linvel` 覆盖 |
| `cosmos_builder_lock_translations` | rapier 求解器约束，被积分器直接改 `translation` 绕过 |

三条签名合法、返回成功、值也真的写进了 builder —— **但对轨道体毫无效果**。

### 实测证据

| 场景 | 期望（若 setter 生效） | 实测 |
| --- | --- | --- |
| `gravity_scale = 0` | 不动 | 下落 **166 868 m**（0.5 s，与默认一致） |
| `lock_translations` | 不动 | 下落 **166 818 m** |
| `linear_damping = 50`（τ=0.02 s） | 行程接近 0 | 行程 **50.0 m** = 无阻尼满行程 |

### 处理方式：钉住现状，不改实现

这三条测试**不是**在断言"参数应该生效"，而是在钉住已知分工。写法上：

```rust
assert!(
    moved > 1.0e4,
    "轨道体的引力不应被 gravity_scale 关掉（integrator.rs 的设计），\
     实测只移动 {moved:.1} m；若这层分工被改，请同步更新本条与 ADR-0040"
);
```

**不改生产代码**的理由：绕过 rapier 力律是刻意选择（`integrator.rs` 解释了
为什么 —— rapier 的 semi-implicit Euler 对轨道长弧会累积相位误差，一圈 LEO
漂数百公里）。改它会破坏轨道精度。

**但这是一个真实的 API 陷阱**：三个入口看起来能控制刚体，实际不能。
本条把陷阱写进文档并钉住，至少下次有人踩到时能查到。

### 一个我走过的弯路

我一度以为找到了 bug：`world.rs:803/807` 出现 `set_gravity_scale(a0, true)`
和 `set_linear_damping(a0)`，看起来像是在 `insert_body` 里覆盖 builder 的值。

读了上下文才发现那是**共享 arena 的命令分发**（cmd 9/10/11 = SetGravityScale /
SetLinearDamping / SetAngularDamping）—— 运行时命令，不是插入时的覆盖。

**又一次「没读全就下结论」。** 记在这里。

## 仍未覆盖：radio 子系统 8 个入口

`cosmos_world_radio_*` 是一个完整子系统（`radio.rs` 747 行 +
`mps-test/cosmos/radio.rs` 657 行），但测试走**内部 Rust API**，不走 C ABI。

补 ABI 层需要：建世界 → `cosmos_world_enable_radio` → 注册节点/反射体 →
`submit_signal` → `radio_step` → `take_results`，一轮的投入。

**诚实记录为未覆盖。**

## 后果

- `mps-cosmos` 的 ABI 契约从"22 个零引用"降到 16 个（radio 8 + world 配置 8）
- 三个无效 setter 有了行为锁定与文档
- 修正了一个我自己的统计错误并记在案

## 没做的事

1. **radio 的 8 个 ABI 入口**（见上）
2. **world 配置类 9 个入口**：`set_perturbation`（13 个参数）、`set_central_body`、
   `set_sun_position`、`add_celestial`、`add_n_body`、`auto_recenter`
3. **`cosmos_hill_radius_for`**：纯公式，可用 `r_H = a·(m/(3M))^(1/3)` 解析对照，
   需要中心天体质量上下文，未做

## 相关代码

- `crates/mps-test/src/cosmos/abi_contract.rs`（6 项测试）
- `crates/mps-test/src/lib.rs`（`pub mod abi_contract;`）
- `crates/mps-test/src/rapier/verify_module_mirror.rs`（cosmos 允许表加一行）
- 生产代码**未改动**

## 记录来源

与实现同批落地。**这一条的主要价值是纠错**：纠正了我上一轮基于错误统计给出的
推荐，并顺带发现了一个真实的 API 陷阱（三个无效 setter）。
