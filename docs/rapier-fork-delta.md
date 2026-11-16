# fork 差异台账（相对官方 `v0.36.1`）

生成于 2026-11-16。基线：`rapier` 官方 `v0.36.1`（当前最新，见文末「为什么不是 0.37」）。

## 一句话结论

**75 个自有提交、13535 行新增、88 个文件。其中约 45% 是与官方重复的软体实现，
约 19% 是零引用的死代码。真正需要保留的扩展不足 15%。**

| 类别 | 提交数 | 行数（约） | 判断 |
| --- | --- | --- | --- |
| A. 软体 fork | 41 | ~6300 | ⚠️ **大部分与官方 0.36 重复** |
| B. `linalg` 自研线代 | 4 | 2631 | ✅ **已删除**（`1957c0da`，净减 2629 行） |
| C. 官方 issue 修复 | 5 | ~1500 | ✅ 真实价值（上游未修） |
| D. 新增模块（granular/fluid/…） | 3 | ~800 | ⚠️ 官方无对应，但引用面很窄 |
| E. 我们新增能力 | 3 | ~700 | ✅ **必留**（电机相关，ADR-0010~0013） |
| F. Merge 提交 | 4 | ~236k | — 合并上游，非自有改动 |
| G. 杂项/可见性/格式 | 15 | ~2000 | 逐条判断 |

## A. 软体 fork（41 个提交，最大的一块）

### ⚠️ 重大更正：实际存在三套软体，fork 那套是孤儿

逐个查清后发现，台账原先说「45% 是与官方重复的软体实现」**低估了混乱程度**。
真实情况是**三套并存**：

| # | 是什么 | 位置 | 行数 | 谁在用 |
| --- | --- | --- | --- | --- |
| 1 | **fork 的独立软体系统** | `rapier/src/dynamics/soft_body_fork.rs` | 3640 | **无人用（见下）** |
| 2 | **官方软体求解器** | `rapier/src/dynamics/soft_body/` | — | `mps-core` 经 `soft_upstream` 调用 |
| 3 | **`mps-core` 的数据层** | `crates/mps-core/src/rapier/soft_data.rs` | 1880 | `mps-core` 自己（69 处引用） |

`mps-core` 的 `PhysicsWorld` 里同时挂着两个集合：

```rust
pub soft_bodies: SoftBodySet,                                     // 第 3 套：mps-core 自己的数据类型
pub(crate) upstream_soft_bodies: rapier3d::dynamics::SoftBodySet, // 第 2 套：官方求解器
```

而 `soft_upstream::prepare` / `collect` 负责在两者之间同步。**第 1 套完全没被卷入。**

### 第 1 套（`soft_body_fork.rs`）为什么是孤儿

证据链：

1. **`mps-core` / `mps-cosmos` / `mps-jni` / `mps-formula` 对它零代码引用。**
   grep 到的 2 处（`events.rs:612`、`world.rs:327`）都是**注释**，且在描述历史。
2. **`mps-test/src/rapier/soft_data.rs` 的注释直接说明了搬迁**：
   > 这些类型从 rapier 的 `soft_body_fork` 搬到了 mps-core：求解交给官方软体，
   > 这里只保留授权/查询所需的数据结构与纯数据操作
3. **没有任何代码或测试插入过它**：`insert_soft_body_fork` / `ForkSoftBody`
   在 `src/` 与 `crates/` 下**零调用点**。
4. 但 `physics_world.rs` 每步仍在推进它：
   ```rust
   self.soft_bodies_fork.write_spring_forces(&mut self.bodies);
   self.soft_bodies_fork.step(dt);
   self.soft_bodies_fork.follow_rigid_bodies(&self.bodies);
   ```
   由于集合永远是空的，这是**空转**。

### 判断

✅ **`soft_body_fork.rs` 可以删除，与 `linalg.rs` 同类。**

但要拆的东西比 `linalg.rs` 多：

- `src/dynamics/mod.rs:49` 的 `pub mod soft_body_fork;`
- `physics_world.rs` 的 8 处：2 个 `use`、字段声明与初始化、3 处 step 接线、
  `insert_soft_body_fork` / `soft_body_fork` / `soft_body_fork_mut` /
  `sleep_soft_body_fork` / `wake_soft_body_fork` / `is_soft_body_fork_sleeping`
  六个公开方法

**风险高于 `linalg.rs`**：`physics_world.rs` 是热路径，step 接线拆错会影响所有
物理行为。必须先跑完整测试基线，逐处拆，每拆一处验证一次。

### 官方 0.36 的能力覆盖（供后续核对）

在 `src/dynamics/soft_body/`（官方目录）里逐项 grep，我们那 14 项 Phase 能力
全部有对应实现：塑性 102 处、撕裂 297、簇 405、FEM 39、弯曲 29、自碰撞 9、
主动应变 8……**但命中次数不等于行为等价**，删除第 1 套不需要这些等价性
（它本来就没人用）；将来若要删第 3 套才需要逐项核对。

### `fork_extras.rs` 的定位（不是孤儿）

`src/dynamics/soft_body/fork_extras.rs`（410 行）是**活的**：它把 fork 独有的
材质行为（撕裂三准则、肌纤维、主动应变、粘弹、热）作为**数据层**挂在官方
`SoftBody` 上，由官方求解器消费。文档头写明了「official body is the only soft
body, and the extra capabilities are stored here as data」。

这一块**必须保留**——官方没有等价物，且已正确接进官方求解器。


## B. `linalg` 自研线性代数（4 个提交，2631 行）—— ✅ 已删除

**状态：2026-11-16 已删除，fork commit `1957c0da`。净减 2629 行。**

验证为真正的零行为变化：

```text
rapier3d-f64 --lib   126 -> 110    （少的 16 项全是 linalg::tests::* 自带测试）
mps-test --lib      1374 -> 1374   （上游消费者一项不差）
```

以下是删除前的分析记录，留档。

### 事实：零引用死代码

`src/linalg.rs` 有 2631 行，`src/lib.rs:154` 声明 `pub mod linalg;`。

全仓 grep `linalg` 的 5 处外部命中，逐一核实：

| 位置 | 性质 |
| --- | --- |
| `lib.rs:153-154` | `pub mod` 声明本身 |
| `soft_body_fork.rs:2697-2698` | **注释**，且明说「no linalg matrix solve is needed」 |
| `soft_element_constraint/mod.rs:5,12` | `soft_element_linalg` —— **官方的另一个模块，名字撞车** |

**没有任何代码调用它。**

### 判断

❌ **可以直接删除 2631 行**，零功能影响。

它当初的目的（提交 `3552202f` 等）是「bit-identical to nalgebra」地替换 nalgebra。
但官方 0.36 已经切换到 glam 标量核心 + `libm` 固定超越函数来做确定性
（见 CHANGELOG v0.35.0-beta.0），这条自研路线**已经被上游方案取代**。

### 为什么这条最该先动

- 收益/风险比最高：删 2631 行，零引用意味着零行为变化
- 但**不能只删文件**：`lib.rs` 的 `pub mod linalg;` 也要删，否则编译失败

## C. 官方 issue 修复（5 个提交，~1500 行）

这些是修**上游自己的 bug**，官方 `v0.36.1` 里还没修：

| 提交 | 修的 issue |
| --- | --- |
| `965fb06c` | #778 圆柱轴向、#488 KCC 移动平台、#223 convex_decomposition panic、#984 CCD bullet-bullet 穿透、#305 EPA |
| `bf65a5c6` | #181 `set_max_linear/angular_velocity` |
| `6e98d068` | #457 joint 查询/力 |
| `c6d6cc82` | #548 CCD 撞击点 |
| `dc818096` | #488 #984 #993 #223 #778 #548 #457 #181 汇总 |

### 判断

✅ **必须保留**。这是 fork 存在的正当理由之一 —— 上游没修而我们依赖。

**但应该做的事**：把这些逐个提 PR 给 `dimforge/rapier`。合了就能从 fork 里删掉，
不合也留下了公开记录。这是降低 fork 维护成本的唯一可持续路径。

## D. 新增模块（granular / fluid / direction_hull / user_index / recorder）

| 模块 | 行数 | 外部引用 | 是什么 | 判断 |
| --- | --- | --- | --- | --- |
| `granular.rs` | 411 | 5 | 颗粒 DEM（官方无） | 保留 |
| `fluid.rs` | 400 | 16 | SPH 流体（官方无） | 保留 |
| `direction_hull.rs` | 214 | 2 | k-DOP / 固定方向凸包 | 保留（见 G'） |
| `user_index.rs` | 285 | 1 | 用户空间 AABB R-tree | 保留（见 G'） |
| `recorder.rs` | 227 | 1 | 整世界录制/回放 | 保留 |
| `compound_baker.rs` | 133 | — | 烘焙式复合碰撞（serde 门控） | 保留（见 G'） |

### 判断

官方没有对应能力，所以**不能简单删**。原先标为「用途不明」的 `direction_hull.rs`
和 `user_index.rs`，在查清 `87f7521c` 后确认都有明确设计意图（见 G' 节），
不是实验残留。

**剩下的真问题**：这些模块引用面很窄（1~16 处），说明是边缘功能。需要确认
它们是否真的被 `mps-core` / `mps-cosmos` 用到 —— 如果只是 fork 里的孤儿，
那和 `linalg.rs` 是同一类问题，只是小一些。

## E. 我们新增的能力（3 个提交，~700 行）

| 提交 | 能力 | 文档 |
| --- | --- | --- |
| `0620c893` | 滚动阻力（`ColliderMaterial` + twist 求解器路径） | ADR-0010 |
| `8b494879` | multibody 逐 DoF 阻尼 + `damping_index` | ADR-0011 |
| `fcdc78da` | multibody 力矩上限 + `joint_axis_velocity` | ADR-0013 |

### 判断

✅ **必留**。这是本项目真实需要、官方没有、且已有 ADR 记录和测试覆盖的能力。

**唯一该改进的**：按 ADR-0008 的补丁纪律，这三个也该向上游提 PR
（它们都是通用能力，不是 `mps` 特有的 hack）。

## F / G. Merge 与杂项

**F. Merge（4 个）**：`528258d5` 合并官方 v0.36.1、`f8de028e` 合并 dimforge/master 等。
这些提交的 `--numstat` 显示 23 万行，是因为把上游全部历史算进来了，**不是自有改动**。
统计 fork 规模时必须排除，否则数字毫无意义。

**G. 杂项（15 个）**，按性质分：

- **可见性调整**（可能影响上层）：`b9180cb7` `get_mut_internal` 改 pub
- **持久接触缓存**：`7f9a9e27` 增加持久接触缓存能力（值得单独评估）
- **force containers**：`372d1252` 接触求解器桥（标注 observation-only）
- **并行优化**：`9383b01a` wake/sleep 两趟 rayon fold
- **子模块**：`731d9e0a` / `085ad954` / `079afe53`（nalgebra、parry 子模块增删）
- **格式**：`f3391e57` rustfmt、`7c5c76ed` 测试修 12 参数

## G'. 5 个「无信息量」提交——已逐个查清

这 5 个提交的消息是纯日期或「Update xxx.rs」，约 2900 行，原先无法从历史判断意图。
逐个读 diff 后的结果：

### `87f7521c`（514 行）→ D 类：`direction_hull` + `user_index` 的诞生

两个模块的**初始提交**，之前 D 类表格里「用途不明」的疑问在这里有答案：

- `direction_hull.rs`：**k-DOP / 固定方向凸包**。把点云投影到一组方向得到 slab 边界，
  再求半空间交得到凸包顶点，喂给 `ColliderBuilder::convex_hull`。文档明确说是
  「通用几何工具，不绑定碰撞管线，也可被 compound baking 或用户空间查询复用」。
- `user_index.rs`：**用户空间 AABB R-tree**。独立于引擎内部 `BroadPhaseBvh`，
  给宿主用任意 `u64` id 做区域查询（游戏实体分桶之类）。

**判断**：不是实验残留，是有明确设计意图的通用工具。保留，但引用面窄（各 1~2 处）。

### `b989b8be`（863 行）→ 三个独立能力

- `compound_baker.rs`（133 行）：**Phase C 烘焙式复合碰撞**。把 parry `Compound`
  的 BVH 连同质量属性序列化，加载时直接复用不重建。`cfg(feature = "serde-serialize")` 门控。
- `recorder.rs`（228 行）：**Phase D 录制/回放**。整世界快照序列化，回放时还原。
- `wheel_joint.rs`（207 行）：车轮关节。
- 附带 `narrow_phase/test.rs` 的 121 行测试。

**判断**：都是真实能力，且 `compound_baker` 有 `crates/rapier3d-f64/tests/` 的测试覆盖。

### `894d34e3`（600 行）→ force containers 的诞生

`src/dynamics/force_containers.rs`（303 行）+ `rigid_body.rs`（253 行）+ 管线接线。

文档头写得很清楚：替代 rapier 的「扁平」力模型。原生模型里用户力每步被
`reset_forces` 抹掉，持久力（推力、磁力锚点）必须每帧重新 `add_force`；而这个
模块用 **{persistent, transient} 双生命周期 + 按 kind 分类的容器**解决它。

**判断**：真实需求（本项目确实需要持久推力）。但这是**改动了核心力模型**，
影响面大 —— `force_containers` 外部引用 55 处，是 G 类里影响最大的一个。

### `33292dc3`（979 行，36 个文件）→ 「显式禁用碰撞对」+ 一轮横向清理

主体是 `disabled_collider_pairs`：允许宿主指定两个碰撞体永不碰撞，
**无视 collision groups、solver hooks 和是否被关节连接**。

实现要点（从 diff 注释读出）：

- 每步清扫 `contact_graph` 与 `intersection_graph`，清掉已禁用对的陈旧 manifold
  和 intersecting 标志
- 移除碰撞体时 `retain` 掉引用它的禁用项（否则陈旧项会永远留在扫描里）
- 拆除语义对齐 `remove_pair`（清 solver color、唤醒 partner、发 `Stopped` 事件、
  断持久岛链接），但**保留边**，所以重新启用不需要走一遍 broad-phase

同时夹带了 `ccd_solver` / `fluid` / `granular` / `soft_body_fork` / `collider` /
`force_containers` 的一轮小改（各几十行），以及 `crates/rapier3d-f64/Cargo.toml`
加 dev-dependency 和 `tests/force_container_determinism.rs`（118 行）。

**判断**：核心能力（禁用碰撞对）是真实的，且有确定性测试。但**提交混杂**——
一个「禁用碰撞对」的改动里夹了 7 个模块的顺手改，这是它消息无信息量的原因，
也是将来如果要拆分的难点。

### `02f6beaf`（34 行）→ `33292dc3` 的 `PhysicsWorld` 入口

只加两个方法：`set_collision_enabled` / `is_collision_enabled`，转发到
narrow-phase 的 per-pair filter。文档写明了对称性和「设置跨 `step` 持久」的语义。

**判断**：`33292dc3` 的配套，两者应视为一件事。

### 小结

**5 个提交全部是有意为之的功能，没有一个是垃圾。** 原先的「无信息量」只是
提交消息的问题，不是代码的问题。这条信息缺口的实际影响是：

| 提交 | 真实性质 | 建议 |
| --- | --- | --- |
| `87f7521c` | 通用几何/空间索引工具 | 保留；评估是否提 PR |
| `b989b8be` | 烘焙复合 + 录制回放 + 车轮关节 | 保留；`compound_baker` 可提 PR |
| `894d34e3` | 重构核心力模型（影响面最大） | 保留；**该写 ADR** |
| `33292dc3` | 禁用碰撞对 + 横向清理 | 保留；**该写 ADR**，并考虑拆分提交 |
| `02f6beaf` | 上一个的 API 入口 | 与 `33292dc3` 合并看待 |

**两个缺口**：`894d34e3`（重构力模型）和 `33292dc3`（禁用碰撞对）都是
**改动核心行为却没有 ADR** 的，按本仓库 ADR-0008 的补丁纪律这是欠账。
应该补两份 ADR，说明为什么必须改核心、官方方案为什么不够。

## 建议的处理顺序

按「收益/风险比」排序：

1. ~~**删 `linalg.rs`**（2631 行，零引用，零风险）~~ ✅ **已完成** `1957c0da`
2. ~~**补齐 5 个无信息量提交的说明**~~ ✅ **已完成**（见 G' 节；结论是全部有真实意图，没有垃圾）
3. ~~**给 `894d34e3`（力模型重构）和 `33292dc3`（禁用碰撞对）补 ADR**~~ ✅ **已完成**
   → [ADR-0014](adr/0014-fork-force-containers.md) / [ADR-0015](adr/0015-explicit-collision-pair-disable.md)
4. **把 C 类（官方 issue 修复）和 E 类（我们的能力）提 PR 给上游**
5. **逐 Phase 核对软体 fork 与官方的行为差异**，能搬的搬、能删的删
6. **决定 D 类模块去留**（`87f7521c` 查清后倾向保留）

## 补 ADR 时查出的两处新事实

**1. `894d34e3` 是纯增量，不是替换。** 原先台账写它「重构了核心力模型」，措辞不准。
核实：原生 `add_force` 仍是 `self.forces.user_force += force`，`reset_forces` 仍只清
`user_force`，**老代码完全不受影响**。它新增的是并行的容器路径。

但有一条**必须记住的坑**：`rb.forces.force` 的写入点现在是唯一的
（`force_containers.rs:318`，另一处是 `quarantine.rs` 清零）。任何直接改
`forces.force` 的代码会在下一步被静默覆盖。改力必须走容器或原生 `add_force`。

**2. 求解器涌现的力 kind 是只读观测。** `Friction` / `ContactReaction` 由
`bridge_solver_contact_forces` 写入，但 `compute_body_effective_forces` 显式跳过它们——
接触求解器已经施加了那些冲量，再求和就是双重施力，会直接让求解器失稳。


## 为什么不是「同步 0.37.0」

你提供的链接 [475caa3](https://github.com/dimforge/rapier/commit/475caa3ff4646fd9a796b67d14116256f4962bc7)
标题是 **"Release bevy_rapier v0.37.0 (#1036)"**。

`bevy_rapier` 和 `rapier` 核心是两条版本线：

| 包 | 版本 | 位置 |
| --- | --- | --- |
| `bevy_rapier3d`（Bevy 集成层） | 0.37.0 | `bindings/bevy_rapier/` |
| `rapier3d`（物理核心，我们的基线） | **0.36.1** | `src/`、`crates/` |

核实结果：

- 该 commit 只改 5 个文件，全在 `bindings/bevy_rapier/`，`src/` 改动 **0 个文件**
- 我们的基线 `v0.36.1` 是它的**祖先**，只落后 2 个提交
- 那 2 个提交与我们的唯一文件重叠是 `bindings/bevy_rapier/publish.sh`，**仅权限位 755→644**

**结论：没有需要同步的核心改动。** 合并那 2 个提交是零风险但也接近零收益。

官方 tag 最新为 `v0.36.1`，crates.io `newest_version` 也是 `0.36.1`（2026-10-10 发布）。
