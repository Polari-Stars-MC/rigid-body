# 0011 — multibody 关节的逐 DoF 阻尼与 `brake()`：让电机真的能停

## 背景

`space` 项目的 `ModuleBoardBlockEntity.StepperMotorModule`（游戏里那个
「电机」）松手后刹不住车。顺着链路查下来，问题不在 Java 侧，而在 multibody
关节只有「位置伺服」这一个自由度模型：

1. **`MultibodyJoint::set_spring(axis, stiffness, rest)` 没有阻尼参数。**
   求解器里 `update_acceleration` 的弹簧项是
   `−k·(q − rest) − k·dt·v`——那个 `−k·dt·v` 是 backward-Euler 隐式化
   `k·(q⁺ − rest)` 的副产物，不是独立的阻尼：它的系数被锁死在 `k` 上。
2. **`default_damping` 给每个自由角 DoF 硬编码 `0.1`。** 相对
   `MOTOR_STIFFNESS = 10_000_000`，阻尼比刚度小 8 个数量级，等于没有。
3. **`ModuleBoardBlockEntity` 在 `speed <= 0` 时直接 `return`，不再调用
   `setMultibodyJointMotor`。** 弹簧于是把关节一直拉回最后一次写入的 `angle`，
   表现为「刹不住 + 反复振荡」。

也就是说：缺的不是「刹车代码」，而是**「只耗散、不追目标」这个自由度**。

## 决定

1. **给 `MultibodyJoint` 加独立的 `spring_damping`**，力
   `F = −k·(q − rest) − c·v`，backward-Euler 隐式化后：力项
   `−k(q−rest) − (k+c)·dt·v`，质量矩阵对角加 `k·dt² + c·dt`。`c = 0` 时
   与改动前**逐位相同**。
2. **让 `brake()` 同时写两条路径**：`spring_damping`（弹簧自己的阻尼）与
   multibody 的逐 DoF `damping` 向量（求解器里以 `dt·d` 进质量矩阵）。
   后者才是 `default_damping` 那个 `0.1` 的归宿；不覆盖它，刹车系数再大也会
   被 0.1 的默认值稀释掉一半意义。
3. **新增 `Multibody::damping_index(link, axis)`** 做空间轴 → 装配索引的换算。
   锁定轴不占槽位，所以两者只在「下面所有轴都自由」时才重合；直接拿轴枚举当
   索引会静默写错 DoF（`articulation.rs` 里 `set_spring(3, ...)` 能成立只是
   因为 revolute 恰好锁了 0/1/2/4/5）。
4. **`brake()` 把 `rest` 锚到当前角**，这样即使 `k` 残留也不产生回复力。

## 后果

- `c = 0`（默认）行为不变；`default_damping = 0.1` 的既有基线由
  `zero_damping_preserves_previous_behaviour` 钉住。
- `brake(damping)` 在 2 秒内把角速度压到初始的 5% 以下
  （`brake_stops_without_pulling_back_to_a_target`）。
- **观察刹车必须读关节坐标，不能读刚体角速度。**
  `multibody::update_velocities` 每步用 `bodies.vels = link.joint_velocity`
  覆盖刚体速度，外部读写 `rigid_body_get/set_angvel` 对挂在 multibody 上的
  刚体完全无效。这条踩了很久，所以加了 `world_multibody_joint_get_coordinate`
  作为可观测入口，并把原因写进测试注释。
- 给刚体质量/惯量必须挂碰撞体：`rigid_body_builder_set_additional_mass`
  只加质量**不加转动惯量**，零惯量的刚体在铰链里拿不到角速度。

## 相关代码

- `rapier/src/dynamics/joint/multibody_joint/multibody_joint.rs`
  （`spring_damping` / `set_spring_damping` / `coordinate` /
  `num_free_lin_dofs` 改 pub）
- `rapier/src/dynamics/joint/multibody_joint/multibody.rs`
  （力项 `(k+c)·dt·v`、质量矩阵 `k·dt² + c·dt`、`damping_index` /
  `set_joint_damping` / `joint_damping`）
- `crates/mps-core/src/rapier/joints.rs`
  （`world_multibody_joint_set_spring_damping` / `_flag`、`world_multibody_joint_brake` /
  `_flag`、`world_multibody_joint_get_coordinate`）
- `crates/mps-test/src/rapier/joints.rs`（6 个测试）
- `crates/mps-jni/src/lib.rs`（两个 JNI 绑定）

## 给 `space` 的接入方式

`StepperMotorModule.tickActive()` 的 `default` 分支改成：

```java
default -> {
    if (hinge == null) return powered;
    boolean running = speed > 0.0;
    if (running) {
        angle += speed * MOTOR_MAX_ANGULAR_SPEED / 20.0;
        world.setMultibodyJointMotor(hinge, ANG_X, angle, MOTOR_STIFFNESS);
    } else {
        // 松手：关伺服、上刹车。不写目标角，关节就停在原地。
        world.multibodyJointBrake(hinge, ANG_X, BRAKE_DAMPING);
    }
}
```
