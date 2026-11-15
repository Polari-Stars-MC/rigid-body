# collision_mode.rs — 默认碰撞体策略

`crates/mps-core/src/rapier/collision_mode.rs`。

`world_insert_default_collider` 用哪种碰撞体、还是干脆不插，由 `WorldCollisionMode` 决定。这是给 Java/JNI 和 C/FFM 调用方的**便利入口**，不是全局开关。

## 四种策略

| 模式 | 行为 |
| --- | --- |
| `None` | 不插碰撞体。体质量必须由调用方显式给（`rigid_body_builder_set_mass` / `additional_mass`），否则是零质量体。 |
| `Simple` | 插一个与体外形最接近的单一凸形状。 |
| `Compound` | 按复合体描述插多个形状。 |
| `Adaptive` | 按尺寸/形状复杂度在 Simple 与 Compound 之间挑。 |

枚举带 `cbindgen:prefix-with-name`，所以在 `rigid_body.h` 里是 `WorldCollisionMode_*` 形式的宏，不是裸名——两个头文件同时 include 时不会撞名。

## 边界（容易搞错的部分）

- 改策略**只影响之后的** `world_insert_default_collider` 调用。
- 不关闭全局碰撞，也不替换**已经插进去的**碰撞体。想换形状就显式重建碰撞体。
- 走显式 `collider_*` / `world_insert_collider` 的路径完全不受它影响。

完整契约与宿主侧接线在 [world-collision-mode.md](../world-collision-mode.md)。

## 并发

选器操作走 `init_guard()`/`step_active` 与 `world_step` 互斥，步进进行中切换返回 `ERR_UNSUPPORTED`，不是 UB。

## 测试

`crates/mps-test/src/rapier/collision_mode.rs`；JNI 面由 `crates/mps-test/java/collision-mode/` 的冒烟 fixture 在 CI `jni-smoke` job 里跑。
