# 0017 — 向量 + 可选开关通道

## 背景

[可合并性扫描](../abi-merge-scan.md) 显示，未覆盖的 156 个实体 setter 里
`Vec3 + Bool` 有 **14 个**，是混合签名中最大的一组：

```c
struct Bool rigid_body_add_force(world, handle, struct Vec3 force, struct Bool wake_up);
struct Bool rigid_body_apply_impulse(world, handle, struct Vec3 impulse, struct Bool wake_up);
struct Bool rigid_body_set_linvel(world, handle, struct Vec3 linvel, struct Bool wake_up);
...
```

[ADR-0016](0016-generic-abi-property-channel.md) 的 `collider_set_vec3_param` 装不下
末尾的 `Bool`，所以这 14 个一直留在专用入口。

## 决定

新增 `rigid_body_set_vec3_flag_param(world, handle, param, x, y, z, flag)`。

### 为什么独立成一条通道，而不是给 vec3 通道加参数

`collider_set_vec3_param(world, handle, param, x, y, z)` 的语义是"把这个向量属性
设成 `(x,y,z)`"。如果再加一个 `wake_up`，通道语义就变成"设一个值 + 可能顺带唤醒"
——而 `collider` 根本没有唤醒概念。

**给不相关的模块加无用参数会污染语义。** 所以独立命名，只服务 `rigid_body`。

### 最关键的设计点：累积与设定必须分开编号

这 14 个入口分两类，**语义完全不同**：

| 类型 | `param` | 入口 | 语义 |
| --- | --- | --- | --- |
| **累积** | 0~3 | `add_force` / `add_torque` / `apply_impulse` / `apply_torque_impulse` | **加上**一份，多次调用叠加 |
| **设定** | 4~6 | `set_linvel` / `set_angvel` / `set_translation` | **设成**这个值，覆盖 |

两类共用同一个签名形状，所以能进同一通道，但 `param` **必须分段编号**。把
`ADD_FORCE` 和 `SET_LINVEL` 混成一个编号会让"累积"和"覆盖"变成同一件事——
那是灾难性的语义错误，且**在单次调用测试里发现不了**。

所以测试 `accumulate_and_set_remain_distinct` 专门钉这一点：

```text
两次 add_force(1.0)        == 一次 add_force(2.0)      # 累积
两次 set_linvel 不同值      == 最后一次生效              # 覆盖
```

### `flag` 的确切含义

`flag` 直接透传给 `wake_up`，非零即真（与专用入口的 `wake_up.0 != 0` 一致）。

**但要注意 rapier 的既有语义**：

```rust
pub fn add_force(&mut self, force: Vector, wake_up: bool) {
    if force != Vector::ZERO && self.body_type.is_dynamic() {
        self.forces.user_force += force;
        if wake_up { self.wake_up(true); }
    }
}
```

`wake_up` 只在**力非零**时才被考虑。也就是说 `flag` 的含义是"**若这个力真的施加了，
顺带唤醒吗**"，不是"无论力大小都唤醒"。

零力 + `flag=1` **不会**唤醒刚体。这是既有行为，通道如实沿用。

## 实现时查清的两个细节

**1. 扭矩/角速度有 2D 与 3D 两个重载。**
`add_torque` / `apply_torque_impulse` / `set_angvel` 在 `cfg(dim2)` 下收 `Real`
（标量），在 `cfg(dim3)` 下收 `Vector`。通道用 `Vector` 是对的，但看到签名是
`Real` 时容易以为要特殊处理。

**2. `flag` 的类型。**
C 侧是 `Bool`（`u8` 包装），通道用裸 `u8` 接收，与仓库既有的 `_flag` 约定一致。

## 后果

| | 数值 |
| --- | ---: |
| 新增通道入口 | **2**（`set_vec3_flag_param` + `abi_version`） |
| 可吸收专用入口 | **14**（含各自的 `_flag` 变体） |
| 净减 | **−12** |

这是自 `soft_body`（−18）、`collider`（−11）之后**第三个正收益批次**，也是唯一
一个"单条通道吸收整个混合签名组"的案例。

## 与 ADR-0016 的关系

ADR-0016 已宣告收敛到适用边界（小模块净收益为负）。这一批**不违反那个结论**：

- 它不是"给小模块加通道"——`Vec3 + Bool` 的 14 个集中在 `rigid_body` 一个模块
- 它的净收益是 **−12**，不是负的
- 它吸收的是**一整种载荷形状**，摊平固定成本后仍有盈余

ADR-0016 的边界条件是"模块至少 6~8 个可合并属性"。`rigid_body` 加上这批后
有 3（标量）+ 7（向量+开关）= **10 个**，跨过了那条线。

## 相关代码

- `crates/mps-core/src/rapier/rigid_body.rs`（通道 + 7 个 `RIGID_BODY_VEC3_*`）
- `crates/mps-test/src/rapier/rigid_body.rs`（5 项测试）
- [docs/abi-merge-scan.md](../abi-merge-scan.md)（这 14 个的来源）

## 记录来源

与实现同批落地（提交 `b6f904d`）。设计先行、测试验证语义分歧、再实施——这条
纪律来自 ADR-0008 的补丁原则。
