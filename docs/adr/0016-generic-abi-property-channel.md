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

## 为什么先做一个切片而不是一次全换

949 个入口一次重构会让失败定位变得极难——现有 1381 项测试是逐步积累的，一次性
大改后，一个数值偏差可能来自任意一处。所以先把 6 个属性做完、验证、再决定是否
继续。

第一批切片选了 `soft_body` 的 6 个纯标量 setter：
`COROTATED` / `NEO_HOOKEAN` / `PRESSURE` / `DAMPING` / `SUBSTEPS` / `ACTIVATION`。

**没选 `set_tear_strain` / `set_tear_stress` / `set_tear_energy`**，因为它们带
额外的 `enabled: u8` 参数，是双参数 setter，不属于"纯标量"这一类。把它们塞进
`set_param` 会让 `param` 的语义变得含混。

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
