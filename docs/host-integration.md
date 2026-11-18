# 宿主集成指南

**给 Java 侧的使用说明。** 仓库对外暴露 **1045 个 C ABI 入口**（`mps-core` 1003 +
`mps-cosmos` 42）。这份文档回答一个问题：**从哪开始，以及 90% 的场景只需要哪些。**

具体物理子系统的深入用法见各自的专题页（[丁达尔](tyndall-mc-integration.md)、
[太空世界](cosmos-arena.md)、[碰撞模式](world-collision-mode.md)、
[电机](mps-formula-motors.md)）。全量入口清单见生成物
[CAPABILITY_MAP.md](CAPABILITY_MAP.md)。

---

## 1. 加载哪个库

| 产物 | 用途 |
| --- | --- |
| `mps.dll` / `libmps.so` / `libmps.dylib` | **Java 实际加载的库**（`mps-jni`） |
| `mps_ffm` | ABI 版本探针（FFM 路径用） |

**先探版本再调任何东西**：

```c
uint32_t abi_version(void);
struct Bool abi_supports_ffm(void);
struct Bool abi_supports_jni(void);
struct Bool abi_supports_formula_ffi(void);
uint32_t abi_formula_ffi_healthcheck(void);   // 0 = 正常
```

不匹配就别往下走 —— 后面每个入口的语义都按当前版本约定。

---

## 2. 五分钟路径

最小可用循环：建世界 → 插入刚体 → 插入碰撞体 → 步进 → 读回。

```c
// 1) 建世界。Vec3 是重力加速度。
struct WorldHandle *world = world_create(vec3(0.0, -9.81, 0.0));

// 2) 刚体 builder。status 用 rigid_body.h 里的 BodyStatus 枚举。
struct RigidBodyBuilderHandle *rb = rigid_body_builder_create(BODY_STATUS_DYNAMIC);
rigid_body_builder_set_translation(rb, vec3(0.0, 10.0, 0.0));

// 3) build() 产出「内存句柄」，insert() 把它装进世界并**消费** builder。
struct RigidBodyMemHandle *rb_mem = rigid_body_builder_build(rb);
RigidBodyHandleRaw body = world_insert_rigid_body(world, rb_mem);

// 4) 碰撞体。形状用 ShapeDesc 按值传入。
struct ColliderBuilderHandle *cb = collider_builder_create_ex(ball(0.5));
struct ColliderMemHandle *cb_mem = collider_builder_build(cb);
ColliderHandleRaw collider = world_insert_collider_with_parent(world, cb_mem, body);

// 5) 步进（固定步长，通常 1/60）。
world_step(world, 1.0 / 60.0);

// 6) 读回。
struct Vec3 p = rigid_body_get_translation(world, body);

// 7) 收尾。
world_destroy(world);
```

**关键点：builder 在 `build` + `insert` 后被消费。** 之后不要再 `builder_destroy`
（那会 double-free —— `cosmos_builder_destroy` 的注释明确写了这一点）。
`builder_destroy` 只用于「建了 builder 又不想要」的路径。

---

## 3. 句柄的两种形态

| 类型 | 形态 | 语义 |
| --- | --- | --- |
| `struct WorldHandle *` | 不透明指针 | 世界的所有权。`world_destroy` 释放 |
| `struct *_BuilderHandle *` | 不透明指针 | 一次性。插入后失效 |
| `struct *_MemHandle *` | 不透明指针 | 中间产物。`insert` 后所有权转移 |
| `RigidBodyHandleRaw` / `ColliderHandleRaw` | **packed `u64`** | 值语义。可存进 Java `long` |

packed `u64` 是故意设计的：Java 侧可以直接用 `long` 持有，不用管生命周期，
也不会持有悬垂指针。世界销毁后这些 `u64` 才失效。

**不要把 Rapier 类型送过 ABI。** `Vec3`/`Quat`/`ShapeDesc` 是仓库自己的 POD 结构。

---

## 4. 错误模型

两条通道，不要混用：

### 世界类入口：返回哨兵 + 线程局部错误槽

```c
uint32_t last_error_code(void);   // ERR_OK=0 … ERR_INTERNAL=6
```

失败哨兵按返回类型：

| 返回类型 | 失败值 |
| --- | --- |
| `struct Bool` | `FALSE`（`.value == 0`） |
| `uint32_t`（索引/id） | `u32::MAX` = `4294967295` |
| `uint64_t`（句柄） | `0` 或 `u64::MAX`（见该入口文档） |
| 指针 | `null` |

**每个失败都会设置错误槽。** 所以「返回 `FALSE` 但没设错误码」本身就是缺陷——
守门测试 `error_consistency` 在盯这个。

### 纯公式入口：数值域错误直接返回失败

公式层的非法输入（`NaN`、负值、除零）不写线程局部槽，就地映射成 `ERR_*`。
所以对公式入口，**不要读 `last_error_code` 判断具体原因**，只看返回值。

---

## 5. 三个必须知道的约定

### 5.1 `_flag` 变体：可选参数走显式开关

```c
struct Bool rigid_body_add_force(      struct WorldHandle *, RigidBodyHandleRaw, struct Vec3, struct Bool wake_up);
struct Bool rigid_body_add_force_flag( struct WorldHandle *, RigidBodyHandleRaw, struct Vec3, uint8_t flag);
```

`_flag` 版本把「可选参数」编码成 `uint8_t` 位掩码，避免 ABI 上出现可选语义。
**已发布的名字、参数、失败哨兵和 `_flag` 变体保持不变** —— 要破坏就新开 ADR。

### 5.2 缓冲区输出：**两个不同的返回约定**

这是本仓库 ABI 里最容易踩的一处不一致。所有带 `(out, capacity)` 的入口长得很像，
但返回值语义**分两类**：

| 家族 | 返回 | 依据 |
| --- | --- | --- |
| `*_read_*`（11 个：`soft_body_read_*`、`granular_read_particles`） | **总数**（可能 > `capacity`） | `granular_read_particles` 返回 `g.len()` |
| `query_intersect_*` / `world_batch_*` / `world_merge_*`（39 个） | **已写入数**（≤ `capacity`） | `intersect_bound` 返回 `written`，满了就 `break` |

所以：

* 问 `_read_*` → 返回值告诉你**总共有多少**，据此分配再调一次
* 问 `query_*` → 返回值告诉你**拿回来几条**；想知道是否有更多，只能加大 `capacity` 重试

**把两者混用会静默出错**：`_read_*` 的返回值当写入数会误判缓冲区不足；
`query_*` 的返回值当总数会漏掉被截断的结果。

判断方法：去头文件看那个入口有没有 `_read_` 中缀。没有就按"已写入数"处理更安全。
### 5.3 方向向量必须已归一化

凡是有 `direction` 参数的入口（相机射线、力方向、推力轴），**调用方必须
`normalize()`**。未归一化会返回 `FALSE`（不是静默算错），因为 `Δs = length/steps`
会让整条积分失真。

---

## 6. 入口地图：1003 个里你只需要一小撮

按前缀分布（`rigid_body.h`）：

| 前缀 | 数量 | 前缀 | 数量 |
| --- | ---: | --- | ---: |
| `world_` | 153 | `fracture_` | 18 |
| `soft_body_` | 101 | `fluid_` | 17 |
| `collider_` | 91 | `sensor_` | 13 |
| `space_` | 74 | `vehicle_` | 12 |
| `rigid_body_` | 70 | `acoustics_` | 12 |
| `query_` | 58 | `rtree_` / `servo_` / `crb_` | 10 各 |
| `character_` | 34 | `disaster_` / `volcano_` | 9 各 |
| `tyndall_` | 32 | `generic_` / `joint_` / `molecular_` / `granular_` | 8 各 |
| `relativity_` / `quantum_` / `material_` | 29 / 27 / 27 | `terrain_` | 7 |
| `astrophysics_` / `electromagnetism_` | 26 / 21 | 其余 | … |

### 90% 场景只需这 20 个

| 用途 | 入口 |
| --- | --- |
| 建/销世界 | `world_create`、`world_destroy` |
| 步进 | `world_step` |
| 重力 | `world_set_gravity` |
| 建刚体 | `rigid_body_builder_create`、`rigid_body_builder_set_translation`、`rigid_body_builder_build`、`world_insert_rigid_body` |
| 建碰撞体 | `collider_builder_create_ex`、`collider_builder_set_friction`、`collider_builder_set_restitution`、`collider_builder_build`、`world_insert_collider_with_parent` |
| 读状态 | `rigid_body_get_translation`、`rigid_body_get_linvel`、`rigid_body_get_angvel`、`rigid_body_get_mass` |
| 施力 | `rigid_body_add_force`、`rigid_body_apply_impulse` |
| 查错 | `last_error_code` |

**其余 980 多个是可选能力**：软体、布料、绳、颗粒、车辆、角色控制器、传感器、
关节、事件、录制回放、体积查询……按需查 [CAPABILITY_MAP.md](CAPABILITY_MAP.md)。

### 太空世界是独立世界

`mps-cosmos` 的 42 个入口（`cosmos_*`）**不进 `mps-core` 的世界**，不共享 arena。
它有自己的 `cosmos_world_create` / `cosmos_world_step`。两者不要混用句柄。

---

## 7. 常见陷阱（都是实测踩过的）

| 陷阱 | 症状 | 正确做法 |
| --- | --- | --- |
| **builder 插入后再 destroy** | double-free / 段错误 | 插入后不要再 destroy |
| **buffers 返回值当写入数** | 结果静默截断 | 返回值是**总数**，可能 > capacity |
| **方向未归一化** | 返回 `FALSE`（至少不静默） | 调 `normalize()` |
| **`cosmos` 的三个 builder setter 对轨道体无效** | 参数被接受但无效果 | 见下一条 |
| **`cosmos_builder_set_gravity_scale` / `set_linear_damping` / `lock_translations`** | 轨道体照样被引力驱动、照样满行程 | **这三个入口对轨道积分体是 no-op**（ADR-0040）：引力走独立的 Verlet n-body 路径、阻尼被积分器写回的 `linvel` 覆盖、锁定被直接改 `translation` 绕过 |
| **光子 `momentum_magnitude = INFINITY`** | 返回 `TRUE` 却拿到 `inf` | 无质量粒子按文档约定用 `INFINITY` 表示「没有静止系」；方向另由单位向量给出（ADR-0037） |
| **SPH/丁达尔的 `σ` 量级** | 光柱几乎不出现 | `σ ≈ 1/(n·L)`，宏观数密度下是 `1e-3` 量级，不是 `1e-12`（ADR-0021） |

---

## 8. 想验证「我这次调用算对了」

仓库自带 1647 项测试，分六层：

| 层 | 查什么 |
| --- | --- |
| 有限性（ADR-0034） | 86 个零引用入口在 `NaN`/`inf` 输入下不得静默产出非有限值 |
| 守恒律（ADR-0035） | 动量/角动量/机械能；SPH 密度解析对照 |
| 解析对照（ADR-0036） | 热力学/相对论/等离子体的教科书闭式解与恒等式 |
| 历史缺陷（ADR-0037） | 光子动量、SPH 自贡献 |
| 量纲不变量（ADR-0038） | 轮胎 `μ` 线性、库仑上限 |
| cosmos ABI（ADR-0040） | `step_n` 等价、三个无效 setter |

宿主侧若发现与这些不一致，**优先怀疑自己的调用**（单位、归一化、句柄生命周期），
再看对应专题页的「没做的事」一节 —— 未覆盖的边界都记在那里。

---

## 9. 相关文档

| 要做什么 | 看哪个 |
| --- | --- |
| 查某个入口是否存在、有没有测试 | [CAPABILITY_MAP.md](CAPABILITY_MAP.md)（生成物，别手改） |
| 丁达尔光柱 | [tyndall-mc-integration.md](tyndall-mc-integration.md) |
| 浮动原点 / 录制回放 / 体素烘焙 / JNI 声明 | [integration/mc-host.md](integration/mc-host.md) |
| 太空世界的 arena 协议 | [cosmos-arena.md](cosmos-arena.md) |
| 碰撞模式策略 | [world-collision-mode.md](world-collision-mode.md) |
| 电机家族 | [mps-formula-motors.md](mps-formula-motors.md) |
| 步进性能 | [world-step-benchmarks.md](world-step-benchmarks.md) |
| 数值域错误的约定 | [formula-numeric-domains.md](formula-numeric-domains.md) |
| 什么还没做 | [ROADMAP.md](ROADMAP.md)「还开着」 |
