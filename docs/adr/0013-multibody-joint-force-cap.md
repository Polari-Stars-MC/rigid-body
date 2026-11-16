# 0013 — multibody 关节的力矩上限：让九种电机模型在两条关节路径上通用

## 背景

ADR-0012 把九个电机家族放进了公式层。要让它们真的驱动一个关节，宿主需要
每步做：

```text
ω = 读当前转速
τ = torque_at_speed(ω)      // 公式层算包络
写回：把 τ 作为这个关节能出的最大力矩
```

前两步在 ADR-0012 之后已经成立。第三步卡在两条关节路径的能力不对等：

| 能力 | impulse joint | multibody joint（改动前） |
| --- | --- | --- |
| target + stiffness | ✅ | ✅ |
| damping（运行时） | ✅ `world_joint_set_motor` | ✅ ADR-0011 的 `spring_damping` |
| **力矩上限** | ✅ `max_force` | ❌ **缺失** |
| 批量设置 | ✅ SoA | ❌ |
| 读当前角度 | ✅ | ✅ ADR-0011 |
| 读当前转速 | ✅ 读刚体 | ❌ **缺失**（且刚体速度不可信） |
| 串联机构稳定性 | 一般 | 好（解运动方程） |

两条缺口都只影响 multibody，而串联机构（机械臂、`articulation_body`）恰恰
必须用 multibody。结果是：**最需要真实电机模型的那条路径，反而表达不了它。**

## 决定

1. **`MultibodyJoint` 加逐 DoF 的 `spring_max_force`**，默认 `Real::MAX`
   （不设限），把弹簧总出力 `−k(q−rest) − c·v` 在进入求解器前 clamp 到
   `±max_force`。
2. **限幅作用在力上，不在加速度上。** 电机额定值是机器属性，与它驱动的负载
   无关；限加速度会让同一个电机在轻载时"更有力"，那是错的。
3. **`Multibody::joint_axis_velocity(link, axis)`** 读权威转速。
   不加这个的话，调用方唯一能读的是刚体 `angvel`，而求解器每步用广义速度
   覆盖它（`update_velocities` 里 `bodies.vels = link.joint_velocity`），
   读到的是二手数据。
4. **新 C ABI 三个**：`world_multibody_joint_set_max_force` / `_flag`、
   `_get_max_force`、`_get_velocity`。

## 两个实现细节

**限幅要乘在总力上，不能只截位置项。**
`total = −k(q−rest) − (k+c)·dt·v`，然后 `total.clamp(-cap, cap)`。只截位置项
会让阻尼项在接近目标时把力"顶"回去，等于限幅失效。

**`Real::MAX` 默认值保证逐位兼容。**
`cap < Real::MAX` 才进入 clamp 分支，所以不设限时执行路径与改动前完全相同。
测试 `multibody_max_force_roundtrips_and_defaults_to_unlimited` 断言新关节
读出来就是 `MAX`。

## 后果

- 九种电机模型现在两条关节路径都能用：宿主读 `get_velocity` → 算包络 →
  写 `set_max_force`。
- `max_force = 0` 让该轴完全不输出力但保留弹簧参数，比拆掉重建便宜。
- 三个 `f64` 返回型入口在失败时返回 `NaN` 而不是 `0`——返回 0 会被误读成
  "上限是零"，那是个完全不同且很危险的结论。

## 一个被我自己的测试纠正的假设

写 `multibody_velocity_readout_tracks_the_drive` 时我先断言速度为正，结果失败：
实测 `v ≈ −2.01 rad/s`，而坐标单调递增。原因是关节坐标的正方向由铰链轴与
局部帧共同决定，不是"我设的正轴就等于坐标正向"。

改成断言**幅值**并与坐标的数值微分比对（实测 `|v| = 2.01` vs `Δc/Δt ≈ 2.0`），
这才是"读数可信"的正确表述。断言符号会把这个测试变成对一个与主题无关的
约定的检查。

## 相关代码

- `rapier/src/dynamics/joint/multibody_joint/multibody_joint.rs`
  （`spring_max_force`、`set_spring_max_force`、`spring_max_force`）
- `rapier/src/dynamics/joint/multibody_joint/multibody.rs`
  （力项 clamp、`joint_axis_velocity`、`set_joint_max_force`、`joint_max_force`）
- `crates/mps-core/src/rapier/joints.rs`（三个 C ABI）
- `crates/mps-test/src/rapier/joints.rs`（6 项新测试）
- `crates/mps-jni/src/lib.rs`（三个 JNI 绑定）
- [docs/mps-formula-motors.md](../mps-formula-motors.md)（接入方式）
