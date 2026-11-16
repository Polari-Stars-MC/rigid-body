# ABI 收敛设计

**状态**：第一批切片已落地（[ADR-0016](adr/0016-generic-abi-property-channel.md)，提交 `d711790`）。后续批次待定。

## 问题

`mps-core` 导出 **949 个 C ABI 入口**，`mps-jni` 有 **619 个 JNI 方法**。这不是"库"，
是一套把物理引擎逐功能暴露成平面的 API 面。

每个新能力要动 **6 处**：core FFI → 头文件 → JNI → Java 结构 → mirror test →
metrics。[CAPABILITY_MAP.md](CAPABILITY_MAP.md) 就是为这个写的；`xtask verify` 的
8 个步骤里有 3 个在防它漂移。

代价是复合的：

| 成本 | 现在 |
| --- | --- |
| 加一个能力的改动点 | 6 处，每处都可能漏 |
| 守卫测试 | 8 个（`verify_ffi_guard` / `verify_module_mirror` / `verify_metrics_sync` / …） |
| fork rebase 后要复验的包装 | 949 个 |
| 头文件生成物 | 每次 rebuild 都可能 diff |

而 `mps-formula`（35188 行、35 个模块）**零 ABI、零 fork 依赖、几乎零维护**。这个
对比就是答案。

## 数据：可合并组

[CAPABILITY_MAP.md](CAPABILITY_MAP.md) 的「可合并组」列只按符号名的动词段机械判定
（动词是**第一个已知动词段**，不是固定位置——`world_set_gravity` 在段 2，
`soft_body_set_coriolis` 在段 3）。

| 组 | 入口数 | 含义 |
| --- | ---: | --- |
| `set_param` | 192 | `*_set_<prop>` |
| `clear_param` | 30 | `*_clear_<prop>` |
| `read_param` | 117 | `*_get_<prop>` / `*_read_<prop>` / `*_count` |
| **小计（第一/二批）** | **339** | **35%** |
| `topology` | 57 | `*_add/insert/remove/detach/attach_*` |
| `dedicated` | 553 | 带真实逻辑 |

`dedicated` 里仍有大量可归并的（如 `world_set_*` 这类其实是属性），但因为动词不在
名字里，机械分类抓不到。所以 **339 是下限，不是上限**。

## 设计

### 核心：三条通用通道

```c
/* 1. 写：设置一个属性 */
bool <module>_set_param(
    <Entity> e,          /* 实体：world / body / collider / soft body / … */
    uint32_t param,      /* 属性枚举，见各模块头文件 */
    double value         /* 标量载荷；向量/结构体另开 *_set_param_vec3 等 */
);

/* 2. 读：取一个属性 */
bool <module>_get_param(
    const <Entity> e,
    uint32_t param,
    double *out          /* 失败时不动 out，返回 false + 错误码 */
);

/* 3. 拓扑：改元素集合（第二批） */
bool <module>_edit(
    <Entity> e,
    uint32_t op,         /* OP_ADD_PARTICLE / OP_REMOVE_SPRING / … */
    const void *payload, /* 按 op 解释 */
    uint32_t len
);
```

`param` 枚举**按值域分组**，每组预留 1000 个槽位：

```
0x0000_0000..0x0000_03E7  soft_body 标量属性
0x0000_03E8..0x0000_07CF  soft_body 向量属性
...
```

这样新增能力只需加枚举值，**ABI 签名永不变**。

### 语义规则（三条，必须写进头文件）

**1. 失败是"不改"，不是"改成 0"。**
`set_param` 校验失败时实体状态完全不变；`get_param` 失败时不写 `out`。这和现有
`ffi_scalar` 的约定一致（`error_consistency` 守门已覆盖 0..6 的语义）。

**2. 未知 `param` 返回 `ERR_INVALID_ARGUMENT`，不 panic、不静默。**
因为 `param` 是 `u32`，版本不匹配时旧库会收到新枚举值——必须**明确拒绝**而不是
当作 0 处理。这一条要配一个 `abi_param_count` 探测入口。

**3. 每个 `param` 枚举必须有 `[min, max]` 文档化的值域。**
否则调用方无法判断某个值是否合法，也就无法做前向兼容。

### 保留专用入口的判据

满足任一条就**不合并**：

- 参数多于 2 个标量（如 `collider_builder_set_mass_properties(center, mass, inertia)`）
- 需要返回结构体（如 `*_get_pipeline_timings(out_values, capacity)`）
- 是生命周期操作（`create` / `destroy` / `insert` / `step`）
- 有非平凡的前置条件（如 `world_step` 的 delta 校验）

按这套判据，`dedicated` 的 553 里预计能再挖出 150~200 个可归并的。

### 兼容策略：旧入口留作 shim

**绝不直接删旧入口。** 迁移分三步，每步都可独立发布：

```c
/* 第 1 步：新增通道，旧入口保持原样 */
bool soft_body_set_corotated(world, id, stiffness);        /* 保留 */
bool soft_body_set_param(world, id, SOFT_COROTATED, v);    /* 新增 */

/* 第 2 步：旧入口改为转发（行为不变，测一遍） */
bool soft_body_set_corotated(world, id, s) {
    return soft_body_set_param(world, id, SOFT_COROTATED, s);
}

/* 第 3 步：确认宿主全部迁移后，删旧入口（需要 major 版本） */
```

**关键**：第 2 步让"旧入口 = 新通道"成为可验证的等式。测试可以断言
`set_corotated(x) == set_param(SOFT_COROTATED, x)`，这样迁移过程有硬证据。

## 分批

### 第一批：`soft_body`（84 个入口里约 50 个是属性）

`soft_body_*` 是最大的单一模块，也是 60 个 `soft_body_set_*` 里绝大部分纯标量。

```c
/* 现在：60 个函数 */
soft_body_set_corotated / set_neo_hookean / set_pressure / set_damping /
set_substeps / set_anisotropy / set_activation / set_gravity / set_thermal /
set_viscoelastic / set_plasticity / set_tear_strain / set_tear_stress /
set_tear_energy / set_cohesion / set_self_collision / set_cross_collision /
set_volume_conservation / set_spring_stiffness / set_fibre_direction / ...

/* 之后：1 个通道 + 枚举 */
soft_body_set_param(world, id, SOFT_COROTATED, k);
```

**预期收益**：`soft_body` 从 84 → 约 30，JNI 同比减少。

### 第二批：`topology`（57 个）

`add_particle` / `add_spring` / `add_tetrahedron` 载荷不同（`usize` / 两个索引 /
四个索引），需要 `*_edit(entity, op, payload, len)` 加 op 专属的 payload 布局。
比第一批难，但结构同样的。

### 第三批：`world` / `collider`

`world` 有 103 个入口，但很多是 region / domain / settings 这类**真正的多参数**
操作，可合并比例低。`collider` 71 个里 `collider_set_*` 有 46 个，可合并。

## 预期结果

| 阶段 | 入口数 | JNI | 加能力的改动点 |
| --- | ---: | ---: | ---: |
| 现在 | 949 | 619 | 6 |
| 第一批后 | ~880 | ~570 | 6 |
| 第二批后 | ~820 | ~530 | 6 |
| 三批全做完 | **~600** | **~400** | **4**（枚举 + 一处 FFI + 一处 JNI + 一处 Java） |

`~600` 是保守估计。`dedicated` 里那 553 个按判据还能再挖一批。

**真正的收益不是数字，是新增能力的边际成本**：从"6 处、每处都可能漏"降到
"加一个枚举值 + 一处 JNI 绑定"。

## 风险

**1. 类型安全下降。** `set_param(e, PROP, f64)` 拿不到编译期检查。补救：用
`mps-bindgen-macro` + `xtask gen-java` **生成类型化包装**（Java 侧仍是
`body.setCorotated(0.5)`），把弱类型限制在 Rust↔C 边界上。

**2. 一次改太多容易全错。** 所以分批 + shim + 等式断言。每迁一个模块，跑一次
`xtask verify` 和 1374 项测试。

**3. `param` 枚举成为新的兼容负担。** 一旦发布，枚举值**永不复用**。需要一份
`abi-params.md` 记录已分配的段，并在 CI 里检查不重复——这是新增的第 9 个守卫。

## 不建议做的

**❌ 不要一次重构 949 个。** 上面那条"一次改太多容易全错"是真风险：现有 1374 项
测试是逐步积累的，一次性大改会让失败定位变得极难。

**❌ 不要为了减少数字而合并语义不同的东西。** `set_gravity`（软体每体重力）和
`world_set_gravity`（世界重力）不是一个属性，即使名字像。

**❌ 不要动已经在 `dedicated` 里的东西来凑数。** 553 这个数字大，但它们是有理由的。

## 与 fork 策略的关系

ABI 收敛的收益有一半在于**降低 fork rebase 的复验面**：949 个包装里很多只是转发到
fork API，收敛后复验点减半。

但这条路线**不依赖 fork 版本**——它是纯 `mps-core` 侧的重构，可以现在就做，也可以
等 fork 有变动时再做。这也是为什么它优先级高于"追上游"。

## 下一步

1. **确认 `param` 枚举的值域分配方案**（决定是否要新增第 9 个守卫）
2. **在 `soft_body` 上做一个完整切片**：`set_param` / `get_param` + 10 个属性的
   枚举 + shim + 等式断言测试
3. 切片通过 `xtask verify` 后，再问是否继续
