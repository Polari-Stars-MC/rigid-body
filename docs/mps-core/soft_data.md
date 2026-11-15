# soft_data.rs — 软体数据结构与自由粒子积分器

`crates/mps-core/src/rapier/soft_data.rs`。

软体的**数据层**：点质量云（`SoftParticle`）+ Hookean 弹簧（`Spring`），加上一个独立的自由粒子积分器。刻意不依赖 SoA/SIMD 求解器边界（`helpers.rs` / `worker.rs` / `generic_contact_constraint.rs` 读的是 `velocities` / `accelerations` 那套 nalgebra 缓冲），所以能单独编译、单独测。

## 数值约定

- 弹簧力 `F = −k·(|x_b − x_a| − rest)·dir − c·(v_rel·dir)·dir`。
- 积分是**半隐式（辛）Euler**：先更新速度 `v += dt·M⁻¹·f`，再更新位置 `x += dt·v`。与刚体积分器的运算顺序一致，在 `enhanced-determinism` 下跨次运行的浮点序列逐位相同。
- 全部走 `crate::math::Vector`（glam 后端）与 `Real`，跟 fork 的向量约定保持一致。

## 它是哪一层

`soft_body.rs` 是 FFI 主模块（88 个入口），`soft_upstream.rs` 把授权数据镜像到 fork 的官方 `SoftBody` 上求解，`soft_data.rs` 只提供数据结构与可独立验证的积分。**求解真值只有一份**（官方 XPBD），这两个文件不是两套物理。

## 测试

`crates/mps-test/src/rapier/soft_data.rs`。
