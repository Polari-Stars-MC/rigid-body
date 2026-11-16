# 0016 — C ABI 通用属性通道

## 背景

`mps-core` 导出 949 个 C ABI 入口，`mps-jni` 619 个 JNI 方法。每加一个能力要动
6 处：core FFI → 头文件 → JNI → Java 结构 → mirror test → metrics。这个成本结构
导致三个后果：

1. 每处都可能漏（8 个守卫测试里有 3 个专门防它漂移）
2. fork rebase 后要复验 949 个包装
3. 新增能力的边际成本恒定在 6 处，不随经验下降

而 `mps-formula`（35188 行、35 个模块）零 ABI、零 fork 依赖、几乎零维护。这个对比
说明问题不在物理实现，在 ABI 形状。

[CAPABILITY_MAP.md](../CAPABILITY_MAP.md) 的「可合并组」列量化了可收敛的规模：
按符号名的动词段机械分类，**339 个入口（35%）是 `set`/`clear`/`get`/`read` 型
属性访问**。

## 决定

引入通用属性通道，让"设置一个标量属性"不再各占一个符号：

```c
uint32_t soft_body_param_abi_version(void);
struct Bool soft_body_set_param(struct WorldHandle *world, uint32_t id,
                                uint32_t param, double value);
uint8_t soft_body_set_param_flag(...);   /* u8 返回变体，沿用既有约定 */
struct Bool soft_body_get_param(struct WorldHandle *world, uint32_t id,
                                uint32_t param, double *out);
```

`param` 是 `u32` 枚举，按值域分组，每组预留槽位：

| 段 | 用途 |
| --- | --- |
| `0x0000_0000..0x0000_03E7` | 标量属性 |
| `0x0000_03E8..0x0000_07CF` | 向量属性（保留） |
| `0x0000_07D0..0x0000_0BB7` | 拓扑编辑（保留） |

**已分配的枚举值永不复用。** 版本不匹配时旧库会收到新枚举值，必须明确拒绝。

## 三条语义规则

**1. 失败是"不改"，不是"改成 0"。**
任何一步校验失败，实体状态完全不变。`get` 失败时**绝不动 `out`**。测试用哨兵值
`-12345.0` 逐条验证四种失败路径。

**2. 未知 `param` 返回 `ERR_INVALID_ARGUMENT`，不静默当作 0。**
静默会把"拼错参数名"变成"设置成功了但值是 0"，这类 bug 极难查。配合
`*_param_abi_version()` 让宿主能探测哪些属性可用。

**3. 每个 `param` 必须有文档化的属性名与值域。**
否则调用方无法判断合法性，也就无法做前向兼容。

## 为什么分批做而不是一次全换

949 个入口一次重构会让失败定位变得极难——现有 1388 项测试是逐步积累的，一次性
大改后，一个数值偏差可能来自任意一处。所以分批，每批验证后再决定是否继续。

### 第一批（`d711790`）：体级纯标量 setter

6 个属性：`COROTATED` / `NEO_HOOKEAN` / `PRESSURE` / `DAMPING` / `SUBSTEPS` /
`ACTIVATION`。

**没选 `set_tear_strain` / `set_tear_stress` / `set_tear_energy`**，因为它们带
额外的 `enabled: u8` 参数，是双参数 setter，不属于"纯标量"这一类。

### 第二批（`24580b7`）：clear 与 indexed

`soft_body` 的属性访问有三类形状，第一批只覆盖了第一类：

| 形状 | 数量 | 通道 |
| --- | ---: | --- |
| `(world, id, value)` 体级标量 | 9 | `set_param` / `get_param` |
| `(world, id)` 清除 | 8 | `clear_param` |
| `(world, id, index, value)` 逐元素 | 5 | `set_indexed_param` |

**为什么 `clear` 要独立通道**：不能用 `set_param(e, PROP, 0.0)` 代替。`0.0` 是
合法的属性值——体积守恒柔度 `0` 表示"硬/不可压缩"。**"关闭"和"设为 0"是两件
不同的事。**

`clear` 的属性编号与 `set` 独立计数（`soft_body_clear_param_abi_version`），
因为两个集合不重合：`damping` 可设但没有 `clear`，`cohesion` 可清但没有 `set`。

**为什么 `indexed` 要独立通道**：硬塞进 `set_param` 会让签名变成
`(world, id, param, value)` 而丢掉索引。

### 第三批（`39d4a4c`）：collider —— 跨模块验证

前两批都在 `soft_body` 内部，只证明了这套模式在**一个模块内**可行。这一批换到
`collider`，确认它能否跨模块复用。

`collider` 与 `soft_body` 有三处实质差异：

| 差异 | soft_body | collider |
| --- | --- | --- |
| 实体句柄 | `u32` 索引 | packed `ColliderHandleRaw`（`u64`，含 generation） |
| `_flag` 变体 | 无 | 9 个 setter 里 8 个另有 `_flag` |
| 必要副作用 | 无 | 每个 setter 都要 `refresh_region_body` |

**结论：设计撑住了。** 差异体现在签名（`handle` 而非 `id`）与副作用（必须逐个
保留 `refresh_region_body`），不影响"一个通道吸收 N 个入口"的核心主张。

`_flag` 的处理方式值得记一笔：不为 8 个专用入口各写一个 `_flag`，而是提供**一个**
`collider_set_param_flag` 统一包装。这正是收敛要省掉的东西。

9 个属性：`FRICTION` / `ROLLING_RESISTANCE` / `RESTITUTION` /
`CONTACT_FORCE_EVENT_THRESHOLD` / `FRICTION_COMBINE_RULE` /
`RESTITUTION_COMBINE_RULE` / `ACTIVE_EVENTS` / `ACTIVE_HOOKS` / `SENSOR`。

### 顺带发现的既有语义陷阱（记录，未改）

`collider_set_friction_combine_rule` 的内联 `match` 是：

```rust
_ => CoefficientCombineRule::Average,
```

**任何非法值都被静默映射成 `Average`**，而不是拒绝。通道沿用同样的映射，因为
通道的职责是等价迁移，不是顺手修 bug。真要改应该两个入口一起改并单独记 ADR。

但通道的**入参校验**比专用入口严：分数、负数、`> 4` 全部返回
`ERR_INVALID_ARGUMENT`，不会走到那个兜底分支。

### 第四批（`d468fc6`）：向量与复合载荷 —— 测出设计上限

前三批都是纯标量。这一批处理两类不同形状的载荷，目的是测出这套设计的**真实
上限**，而不是继续在舒适区里加批次。

**结论：两类都能收敛，但代价不同。**

| 载荷形状 | 通道 | 收敛性质 |
| --- | --- | --- |
| 标量 `f64` | `set_param` | **形状级**：任何标量属性都能进来 |
| 向量 `Vec3` / `Quat` | `set_vec3_param` / `set_quat_param` | **形状级**：3 或 4 个 `f64` 的属性都能进来 |
| 复合 `2 × u32` | `set_u32x2_param` | **宽度级**：只能吸收恰好 2 个 `u32` 的 |

这就是边界：**标量与向量的收敛是"形状级"的，复合载荷的收敛是"宽度级"的。**
载荷变成 3 个字段、或含混合类型（比如 `f64` + `u32`）时，必须再开一个通道。
复合载荷不能无限扩展。

顺带一个观察：`InteractionGroupsDesc` 只有 2 个 `u32`，其实可以塞进一个 `u64`
（高 32 位 `memberships`，低 32 位 `filter`）。但那样会让 C 侧调用者手工做位移，
容易错。展开成两个参数更清楚，代价是通道签名固定。

本批吸收 5 个入口：`set_pose` / `set_translation` / `set_rotation` /
`set_collision_groups` / `set_solver_groups`。

### 剩余的形状：`(index + 向量)`

`set_fibre_direction` / `set_spring_fibre_direction` / `set_particle_velocity`
是 `(world, id, index, x, y, z)` —— 索引 + 向量。理论上可以再加一个
`set_indexed_vec3_param`，但那会让通道族变成二维（索引 × 形状）。

`set_anisotropy` / `set_gravity` 则是 `(world, id, x, y, z)`，不带索引，能被
`set_vec3_param` 直接吸收。

`set_thermal`（4 个标量）、`set_plasticity`（2 个）、`set_cohesion` /
`set_self_collision` / `set_cross_collision`（2~3 个）属于复合载荷，受宽度级
限制，需要按宽度分别开通道或改用 `payload + len` 的通用形式。

**这里的取舍值得记下**：与其为 2/3/4 个字段各开一个通道，不如一次性采用
`(opaque payload pointer, len)` 的形式。但那会失去类型安全，且和 ADR 的三条语义
规则（尤其是"失败不改"）更难保证。所以这一批**故意停在宽度级**，把
`payload + len` 留作将来真需要时的选项。

这两类是下一批的事。

## 迁移路径（三步，每步可独立发布）

```c
/* 1. 新增通道，旧入口不动 */
bool soft_body_set_corotated(world, id, stiffness);
bool soft_body_set_param(world, id, SOFT_COROTATED, v);

/* 2. 旧入口改为转发（行为不变） */
bool soft_body_set_corotated(world, id, s) {
    return soft_body_set_param(world, id, SOFT_COROTATED, s);
}

/* 3. 宿主全部迁移后删旧入口（需 major 版本） */
```

**第 2 步是关键**：它让"旧入口 = 新通道"成为可验证的等式。测试
`dedicated_setters_agree_with_the_generic_channel` 已经在第一步就钉住了这个等式
（10 组 `(param, value)`，含 4 组非法值，两条路径结论必须一致）。所以将来做第 2 步
时，测试已经准备好了。

## 后果

**收益**：

- 新增标量属性只需加枚举值，**ABI 签名不变**
- 预计 949 → 约 600，JNI 619 → 约 400
- 新增能力的改动点从 6 处降到 4 处
- fork rebase 的复验面同步缩小

**代价**：

- `set_param(e, PROP, f64)` 拿不到编译期类型检查。补救是靠
  `mps-bindgen-macro` + `xtask gen-java` 生成类型化包装，把弱类型限制在
  Rust↔C 边界上，Java 侧仍是 `body.setCorotated(0.5)`
- `param` 枚举本身成为新的兼容负担，需要一份值域台账并在 CI 里检查不重复

## 不做什么

**❌ 不为减少数字而合并语义不同的东西。** `soft_body_set_gravity`（软体每体重力）
和 `world_set_gravity`（世界重力）不是一个属性，即使名字像。

**❌ 不动 `dedicated` 里的 553 个来凑数。** 它们带真实逻辑，有保留理由。

**❌ 不在一个提交里跨多个模块。** 每批一个模块，验证后再下一批。

## 相关代码

- `crates/mps-core/src/rapier/soft_body.rs`（通道实现 + 6 个 `SOFT_PARAM_*`）
- `crates/mps-test/src/rapier/soft_body.rs`（7 项测试）
- `crates/xtask/src/capability_map.rs`（`MergeGroup` 分类器）
- [docs/abi-convergence.md](../abi-convergence.md)（完整设计，含分批计划）
- [docs/CAPABILITY_MAP.md](../CAPABILITY_MAP.md)（可合并组列与汇总）

## 记录来源

本条 ADR 与实现同批落地（提交 `d711790`），不是事后补记。设计先行、切片验证、
再决定是否继续 —— 这条纪律来自 ADR-0008 的补丁原则。
