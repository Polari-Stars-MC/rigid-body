# 0009 — 体级模拟域：炮弹停在模拟边界外时用 set_enabled 冻结

## 背景

MC 宿主把模拟边界外的对象（飞出边界的炮弹、远处未加载区块的结构）留在世界里
会造成两类浪费：这些体持续参与 broad phase/island 求解（2000 体世界里 90% 的
体可以完全不参与模拟），而 region 的 sleep 语义不足以保证"完全不动"——sleeping
体仍占 broad phase 槽位、仍可能被接触对唤醒，且它的休眠内能从未被冻结承诺。
宿主需要的是**显式成员资格**：宿主自己决定哪些体划出模拟，而不是靠空间谓词每步
重算。

## 决定

1. **三个 C ABI 入口**（`world.rs`，沿用 `模块_动词` 命名）：
   - `rigid_body_set_simulation_domain(world, handle, domain)`：划转成员资格，
     domain 0 = 无域（调用即从当前域移除并恢复活动）。切换立即生效：目标域
     inactive 时刚体当场 `set_enabled(false)`。
   - `world_set_domain_active(world, domain, active) -> u32`：返回状态实际
     翻转的刚体数。inactive → 成员全体 `set_enabled(false)`。
   - `world_domain_body_count(world, domain) -> i32`：成员数（含被冻结的），
     未知域 -1 + `ERR_NOT_FOUND`。
2. **冻结 = Rapier `RigidBody::set_enabled(false)`**（fork 内建 API，62 处
   消费点已审计）。禁用体从 broad phase、island、求解管线全部撤出，位姿与
   速度逐位保留；`wake_up` 在禁用体上是 no-op——这给出比 region-sleep 更强
   的"完全不动"保证（守门：`simulation_domain_freeze_and_thaw` 断言冻结期
   10 步位级零变化）。
3. **与 region 的分工**：region 是空间谓词、每步重算、语义是 sleep；domain
   是显式成员表、宿主控制、语义是 disable。两者可叠加（一个体可以同时在
   inactive region 和 inactive domain 里）。
4. **成员资格生命周期**：`world_remove_rigid_body` 会把被移除的体从所有域的
   成员表清掉（悬空句柄防护）。域本身没有生命周期管理——空域允许存在，
   `world_set_domain_active` 对从未创建的域报 `ERR_NOT_FOUND`。
5. **性能承诺**：2000 体世界把 1800 体冻结后，步进耗时约为全活动基线的
   9.8%（证据：`cargo test -p mps-test --release --lib -- domain_freeze_step_cost --ignored --nocapture`，
   2026.11.12 实测 base=0.1507s frozen=0.0148s ratio=0.098）。承诺上限
   "冻结机制自身开销 < 全活动基线的 40%"有充分余量；真实收益来自
   broad phase/island 工作量随冻结体近线性缩减。
6. **JNI 面**：`rigidBodySetSimulationDomain` / `worldSetDomainActive` /
   `worldDomainBodyCount` 三方法进 `RapierNative` 冒烟 fixture（collision-mode
   场景，含划转/冻结/恢复/重复停用 no-op/成员清零五断言）。

## 后果

- 宿主获得 O(1) 成员划转与零每步查询开销的模拟分级手段；region 保留给
  "按位置自动暂停"的场景。
- 禁用体不可被 `world_step` 推进，也不能被任何事件唤醒——需要活动的宿主
  必须显式 `world_set_domain_active(.., true)`（或把体划回域 0）。
- 成员表是 `HashMap<u32, HashSet<RigidBodyHandle>>`，域数量大时的内存
  开销由宿主控制（每域一个 HashSet）。
- 与 ADR-0001 的 f64 承诺一致：冻结/恢复不引入任何数值重算路径，
  位级保留由测试钉住。

## 相关代码

- `crates/mps-core/src/rapier/world.rs`（三入口 + `simulation_domains`/
  `inactive_domains` 字段）
- `crates/mps-core/src/rapier/rigid_body.rs`（`world_remove_rigid_body`
  成员表清理）
- `crates/mps-test/src/rapier/world.rs`（`simulation_domain_freeze_and_thaw`、
  `domain_freeze_step_cost_ratio`）
- `crates/mps-test/java/collision-mode/org/polaris2023/mps/rapier/RapierNative.java`
- `crates/mps-jni/src/lib.rs`（三 JNI 方法）
