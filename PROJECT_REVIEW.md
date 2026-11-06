# mps（rigid-body）项目审计报告

> 只读静态审计。沙箱内 `cargo` 无法执行（spawn 被拒），因此**没有构建、没有跑测试**；所有结论来自源码、生成物与配置。行号为编辑器行号。

## 0. 结论一句话

工程纪律（文档、ADR、守门测试、生成物流程）搭得比大多数同类项目完整，**但纪律的落地率不高**：约 340 个 `extern "C"` 入口违反了自己写下的 `ffi_guard` 硬规则，5 个守门测试里 2 个是死断言，共享内存通道有 3 处会静默算错，同一个物理量在仓库里普遍有 2–3 份独立实现。真正的风险不在「代码写得糙」，而在**「声称已钉住」和「实际钉住」之间的落差**。

---

## 0.1 本轮已修（含验证方式）

全部改动都在本仓库内，并已用 `cargo fmt --all --check`、`cargo clippy --all-targets -D warnings`（除 `mps-web`）、`cargo test -p mps-test --lib`（1154 passed / 0 failed）验证。

| # | 缺陷 | 修法 | 落点 |
| --- | --- | --- | --- |
| 1 | build script 里的 cbindgen 会跑嵌套 `cargo metadata`，依赖网络与可写 registry（本机 registry 只读时直接构建失败） | 构建前在进程内强制 `CARGO_NET_OFFLINE=true`，嵌套调用只用本地 lock | `mps-build-common/src/lib.rs` |
| 2 | `world_step` 对 null world / 非法 dt 静默返回 | 补 `ERR_NULL_POINTER` / `ERR_INVALID_ARGUMENT` | `world.rs:576-600` |
| 3 | ~180 个标量包装失败时 `last_error_code()` 仍是 `ERR_OK` | `ffi_scalar` 在 null / `None` 分支写错误槽，并在成功时清槽；保留 formula 已写的原因 | `rapier/ffi/mod.rs` |
| 4 | `verify_module_mirror` 的 orphan 分支恒空（`expected` 并入了 test 侧自身） | `expected` 只取生产侧；白名单补 3 个真实 test-only 文件 | `verify_module_mirror.rs` |
| 5 | 6 处测试文件从不编译（29+ 个测试从未运行）却计入 `TEST_COUNT` | 新增「每个 `.rs` 必须被 `mod` 声明」断言；接活 `rapier/compat`、`cosmos/flight`、`disciplines`、`scientists`、`rotor`；补导出 `mps_cosmos::flight::StepConfig` | `verify_module_mirror.rs`、`mps-test/src/lib.rs`、`flight/mod.rs` |
| 6 | metrics 18 个常量只有 3 个受守门 | CI 增加 `dump-metrics` + `git diff --exit-code`（覆盖全部 18）；修正 `GRAVITY_MODEL_COUNT` 的错误注释；同步 docs | `ci.yml`、`xtask/src/main.rs`、`docs/evidence/metrics.md` |
| 7 | `arena_idx_map` 只在 body 数变化时重建 → 同长度换体后命令静默丢弃 | 每次 drain 时重建（只在有命令时，O(n)） | `world.rs:643-670` |
| 8 | cosmos 同上；且 arena AddForce 写用 arena 序号、读用 rapier 槽号；`drain` 在 `reset_forces` 之前导致**力 100% 被清零** | 三处一起修：重建映射、读写统一按槽号、`drain` 移到 `reset_forces` 之后 | `mps-cosmos/src/world.rs` |
| 9 | `builderLockTranslations` 换掉 Box 却不回写句柄 → 悬垂指针 | 原地 `ptr::read` + `ptr::write`，句柄地址不变 | `mps-jni/src/lib.rs`、`mps-cosmos/src/ffi.rs` |
| 10 | `rigid_body_consume_force_queue` 无 `ffi_guard`、无 capacity 上界、`get_unchecked`、`flags` 非原子读、head/tail 不校验 | 全部补上；越界索引改为 `get()` 返回 `ERR_INVALID_ARGUMENT` | `ffi/force_queue.rs` |
| 11 | `soft_body_destroy` 只删集合槽位（代理刚体/蒙皮/体素元数据全部遗留）；重复 `enable_collision(true)` 泄漏整套代理 | 新增 `teardown_soft_body` / `remove_soft_body_proxies`，destroy 与三个体家族的 rebuild 路径共用 | `soft_body.rs`、`granular.rs`、`fluid_sph.rs` |
| 12 | JNI 数组长度不校验（compound boxes / voxel bytes / colliderSetVoxels 越界读） | 新增 `*_exact` + `voxel_len` 校验，不符返回 `ERR_INVALID_ARGUMENT` | `mps-jni/src/helper.rs`、`lib.rs` |
| 13 | SPH 密度把自贡献算两遍（液体永久自膨胀） | 密度循环跳过 `i == j` | `rapier/src/dynamics/fluid.rs`（vendored fork） |
| 14 | 轮胎：刹车项漏乘 `dt`、载荷敏感性量纲错（`N^0.8`）、滑移曲线 `1/|s|` 归零 | 补 `dt`；改 `μ·N·(N/Fz0)^(n-1)`；滑移曲线饱和到 `0.8·peak` | `tire_model.rs` |
| 15 | relativity 三处「成功返回但写 NaN/INF」 | 光子动量改为方向单位向量；ISCO 内与 β²≥1 改为失败而非 NaN/INF | `relativity.rs` |
| 16 | plasma 德拜长度阈值与域检查不一致，`1e-13` 密度产出 `INFINITY` | 阈值统一为 `> 0` | `plasma.rs` |

未做（明确留给后续，含原因）：删 163 个 mps-formula `extern "C"`（无头文件、无调用者，但会牵动镜像测试，宜单独一次变更）；给 force queue 的 C ABI 加 buffer 长度参数（破坏 ABI，需新 ADR）；把 `query_lock` 覆盖到全部变更入口并文档化线程模型（结构改动）；arena handle map 改写 packed 编码（需 Java 侧同步）；给 `cosmos.h` 的宏加 `COSMOS_` 前缀（破坏 ABI）；重填 9 颗卫星的 `gm`（需要权威数据源逐条核对，不宜凭记忆改）。

---


## 1. 项目画像（实测）

| Crate | 行数 | 产物 | C 导出 | 备注 |
| --- | --- | --- | --- | --- |
| `mps-core` | 42,872 | cdylib + rlib | **904** | `rigid_body.h` 9,791 行 / 372 KB / 860 条声明 |
| `mps-formula` | 32,389 | rlib | **163** | 无头文件、无下游调用者、无 guard |
| `mps-cosmos` | 6,341 | rlib | 39 | 39/39 有 `ffi_guard`（唯一守住规则的 crate） |
| `mps-jni` | 2,520 | cdylib | ~626 JNI | 3 个 `export_name` 前缀 + 4 个手写 |
| `mps-test` | 36,770 | lib | — | 1,165 个 `#[test]`，其中 **29 个不参与编译** |
| `rapier/` | 282,727 | submodule | — | 独立 workspace，指向个人 fork |

- 源码里 **1,125 个 `unsafe {}` 块**，CI 里 **0 个** Miri / ASAN / fuzz / loom。
- 最大单文件：`soft_body.rs` 5,016 行 / 89 个 FFI 入口；`events.rs` 2,750 行 / 66 个；`world.rs` 2,193 行，其中 `world_step` 一个函数 **536 行**。
- `mps-core/src/rapier/mod.rs` 89 行装 87 个模块。

---

## 2. P0 — 会让宿主崩溃、或静默算错物理

### 2.1 共享内存命令通道：三处断裂（最值钱的一类缺陷）

**a) `mps-core` 的 arena 索引映射只在 body 数量变化时重建**
`world.rs:636-643` 用 `n_bodies != arena_idx_map_body_count` 作为唯一重建条件。删一个刚体再加一个，长度不变 → 映射不重建 → 旧句柄失效 → `world.rs:646-648` 的 `bodies.get_mut(*h)` 返回 `None` → **命令被静默丢弃，不设错误码**。`arena_idx_map_body_count` 全仓只在初始化和 `world_step` 里写，插入/删除路径都不失效它。`mps-cosmos/world.rs:648-654` 有同一个 bug（只在创建 arena 时清）。

**b) `mps-cosmos` 的 arena 力用了两套索引**
写：`world.rs:674-679` 用 arena 槽序号；读：`world.rs:1532`/`1579` 用 `handle.into_raw_parts().0`（rapier 槽号）。删过任意刚体后两者不等 → 力叠到错的刚体，或被 `get()` 静默丢弃。

**c) `RapierForce` 路径把 arena 的力在步进前清掉**
`world.rs:1009` 先 `drain_arena_commands()`（内部 `add_force`/`add_torque`）→ `world.rs:1267-1268` `reset_forces(false)` 把 `user_force` 清零 → `world.rs:1279` `pipeline.step`。默认集成模式下 **Java 经命令环写的所有力/力矩 100% 丢失**，而 `SetVelocity`/`ApplyImpulse` 仍然生效 —— 表现为「有的命令有效有的无效」。

### 2.2 `cosmosBuilderLockTranslations` 制造悬垂指针（UAF）
`crates/mps-jni/src/lib.rs:1857-1865`：`Box::from_raw(builder)` → `lock_translations()`（消费 self）→ `Box::new` → `let _ = Box::into_raw(...)` **丢弃新地址**。调用后 Java 手里的 `long builder` 指向已释放内存，后续 set/insert 是 UAF；新分配的 Box 同时泄漏。注释声称「等价于链尾 `.lock_translations()`」，实际不等价。

### 2.3 `rigid_body_consume_force_queue`：无 guard + 无上界 + 无长度参数
`ffi/force_queue.rs:181`。全仓唯一「既没 `catch_unwind`、又真有越界/panic 路径」的入口：
- `validate()`（`:161-169`）只查 2 的幂与 stride，**没有 capacity 上界**；
- `bitmap()`/`payload()`（`:96-119`）据此 `from_raw_parts` 造任意长度切片；
- `payload[base]`（`:222-230`）是带边界检查的索引 → 越界即 panic → `extern "C"` 边界 abort 整个 JVM（内层 abort，`mps-jni` 的 `catch_unwind` 救不了）；
- C 签名里**没有 buffer 长度参数**，所以所有边界检查都是信仰式的；
- `is_paused()` 用普通读读 `flags`（`:54-56`），而文档 `:22` 声称 acquire —— 与 Java 并发写构成 data race；
- `while tail != head`（`:216`）不校验 `head - tail ≤ capacity`，损坏的 head 会让 Rust 侧空转；
- `generation` 字段（`:41`）文档说解决 ABA，全仓无任何读写 —— 死字段。

### 2.4 `soft_body_destroy` 只删集合槽位，代理刚体全部遗留
`soft_body.rs:4168-4183` 只有 `soft_bodies.remove(sid)`。而同一文件 `:4995-5010`（`enable_collision(FALSE)`）有一条完整拆除路径：遍历 `soft_body_proxies[id]` 逐个 `bodies.remove`。`destroy` 没复用它，于是 `soft_body_proxies`、`skin_bindings`、`voxel_soft_meta` 三张旁路表 + 每个粒子的代理 `RigidBody`/`Collider` 全部留在世界里。`world_step` 的同步循环靠 `soft_bodies.get(sid)` 判空后 `continue`，所以这些代理**不再被驱动但仍然是实心碰撞体**，会持续推挤其它刚体。销毁一块 512×512 布料 = 26 万个不可见实心球。

同类：`*_enable_collision(TRUE)` 重复调用（`soft_body.rs:5055`、`granular.rs:400`、`fluid_sph.rs:303`）直接 `insert` 覆盖，不先拆除旧代理 —— 而 `granular.rs:327` 的文档明写「Enabling again re-syncs the proxy set」，等于文档引导用户踩坑。
另外 `fluid_*` / `granular_*` **根本没有 destroy/remove 入口**（各自只有 6 / 8 个入口），存储是 `Vec<FluidWorld>` / `Vec<GranularWorld>`，id 就是下标 → 只能增不能减。

### 2.5 SPH 密度把自贡献算了两次
`rapier/src/dynamics/fluid.rs:184-192`：`:184` 先加 `poly6(0,h)*m` 作为自贡献，`:185` 的邻居循环**缺 `if i == j { continue }`**（同一函数 `:211-214` 的力循环有），`j == i` 时 `d2 = 0 < h2` 成立，再加一遍。密度偏大 → `pressure = max(k(ρ−ρ0),0)` 在恰好静止密度时仍为正 → **液体永久自膨胀**。仓库自己的测试注释（`fluid.rs:299`）写「~3.1 local density」，实际算出 ≈4.65，测试只用 `rest_density = 2.0` 判「是否排斥」，所以从没被发现。

### 2.6 事件回调里调 FFI：一边死锁，一边 UB
回调在 `pipeline.step` 内部被调用（`events.rs:426-453`，Java 以裸 `usize` 注册 `:1997-2020`），此时 `world_step` 持有 `query_lock.write()`（`world.rs:588`）。回调里：
- 调 `query_*`（`query.rs:88` 取 read）→ parking_lot RwLock 同线程重入 → **永久死锁**；
- 调 `collider_*` / `rigid_body_set_*`（不取锁）→ 与栈上活着的 `&mut *world` 构成别名 → **UB**。

而 `events.rs` 只写了「注册是 init-time only」，**没有任何一句说明回调内允许做什么**。

### 2.7 `query_lock` 的设计对、用法坏
全仓只有 10 个入口取这把锁（`query_*` 读，`rigid_body`/`joints`/`controller`/`character_body`/`world` 写）。其余约 890 个入口——包括 `collider_create`、`world_insert_collider`、所有体家族——**不取锁**。于是：
- 锁只能序列化「已加锁的那 10 个」；
- `world.as_mut()` 在取锁**之前**发生（`rigid_body.rs:677-682` vs `world.rs:582-588`），两个并存的 `&mut WorldHandle` 已经构成 UB，锁补不回来；
- 全仓没有任何一处文档写明这套 ABI 的线程模型。

---

## 3. P1 — 错误通道与 ABI 契约

| 缺陷 | 位置 | 后果 |
| --- | --- | --- |
| `world_step` 对非法 dt 静默返回 | `world.rs:582-591` | 同一份 dt 校验在 `world_set_integration_parameters:1399-1408` 会 `set_error(ERR_INVALID_ARGUMENT)`；`world_step` 直接 `return`，宿主无法发现步进被吞 |
| **176 个 core 入口 + 163 个 formula 入口没有 `ffi_guard`** | `rel.rs`(30)、`qphys.rs`(28)、`astrocalc.rs`(27)、`matmech.rs`(25)、`emag.rs`(22)、`nucphys.rs`(19)、`acoustics_ffi.rs`(13)、`thermo.rs`(6)、`plasma_ffi.rs`(6)、`ffi/force_queue.rs`(1) | 直接违反 AGENTS.md「每个 `extern "C"` 入口用 `ffi_guard`」。今天这些函数体大多无 panic 路径（已逐个 grep），但 `matmech.rs:318`、`control_theory.rs:315-320` 这类裸 `from_raw_parts` 一旦遇畸形输入就是 UB，不是错误码 |
| `ffi_scalar` 失败不写错误槽 | `ffi/mod.rs`（`ffi_scalar` 全文无 `set_error`） | 上述 ~176 个标量包装失败时返回 `FALSE`，`last_error_code()` 仍是 `ERR_OK` 或上一次的陈旧错误 → 宿主拿到「成功 + 失败返回值」 |
| `last_error_message` 指针比文档说的更早失效 | `mps-formula/src/error.rs:18-30`；文档 `mps-core/src/rapier/error.rs:66-80` | 每个成功入口末尾 `clear_error()` → `set_error` → 替换并 drop 旧 `CString`。文档只说「下一次报错调用失效」，实际**任何一次成功调用**都失效 → Java 缓存指针后读 = UAF |
| `rigid_body.h` 声明了 9 个默认构建不存在的符号 | `world.rs:2151-2153` 等 5 处 + `rigid_body.rs:1116/1156/1198/1215` 共 9 处 `#[cfg(feature="relative-force")]`，但 `rigid_body.h:9891-9929` 照样导出 | cbindgen 不按 feature 过滤；CI 只跑默认 features 且只 diff 文本 → 头文件承诺的符号在 Java 实际 `dlopen` 的 cdylib 里不存在 |
| arena handle map 写裸 id，文档说是 packed | 代码 `shared_arena/holes.rs:116`（`handle.into_raw_parts().0`）；文档 `shared_arena/mod.rs:206`（「packed as u64」） | 与 `force_queue` 期望的 packed 编码不兼容（`convert.rs:60-63` 会 `checked_sub(1)` 再拆高低位）→ 经 handle map 回传的句柄喂进力队列会解成错的/无效的刚体 |
| arena 槽位 gen 非单调 | `holes.rs:87`（`clear_body_slot` 写 0）、`:58/:75` | 槽位 gen 序列 0→1→2→0→1→2，Java 缓存的 (slot, gen=2) 在换 body 后仍然相等 → 真 ABA |
| 两个头文件同名宏冲突 | `rigid_body.h:84/89/98` vs `cosmos.h:16/21/31`：`ARENA_MAGIC`、`ARENA_VERSION`(2 vs 1)、`CMD_SLOT_STRIDE`(32 vs 40)；`BODY_SLOT_STRIDE` 值同为 96 但槽布局不同 | 同时 include 两头 = 宏重定义，`ARENA_VERSION` 取决于包含顺序——恰好是宿主判断布局的那个常数 |
| `ARENA_VERSION` 无运行时查询、与 `ABI_VERSION` 无守门 | `mps-ffm/src/lib.rs:3`（`ABI_VERSION=1`）；pin 在 `version_consistency.rs:46-48`、`arena_compat.rs:19-21`，而 `version_consistency.rs:22-28` 明写「**不**断言两者相等」，现状就是 2 vs 1 | 布局 2→3 而 ABI_VERSION 不变时，宿主 `abiVersion()` 仍得 1，防呆不触发。BOUNDARY.md:63 要求「一起动」，但没有任何可执行约束 |
| 两套句柄编码 | core：`((generation<<32)|id)+1`（`convert.rs:56-62`）；cosmos/JNI：`(idx<<32)|generation`（`lib.rs:1801-1811`、`ffi.rs:75-85`） | 高低 32 位顺序相反、还多一个 +1。core 句柄误传进 cosmos 接口**不报错**，命中另一个刚体 |
| JNI 数组长度与声明尺寸不校验 | `lib.rs:391-396`（compound boxes）、`:416-427`（voxel bytes）、`:515-533`；`helper.rs:40/60` 拿到了 `len` 却只用于拷贝 | Java 传 `new byte[1]` + `128³` → 2 MB 越界读 |

---

## 4. P1 — 架构性重复：同一个物理有 2–3 份实现

1. **`spaceflight/` 在两个 crate 各一份**：`mps-formula/src/spaceflight/`（9 文件 / 73 KB）与 `mps-core/src/rapier/spaceflight/`（9 文件 / 81 KB）同名同域。core 侧不是薄包装而是**重写**：`space_elements_to_state`（`kepler.rs:15-77`）、`space_hohmann_transfer`（`:82-110`）自己算了一遍，只有 `space_kepler_period`（`:119-127`）转调 formula 的 `_checked` 版本。
2. **两套软体求解器并存**：`rapier/mod.rs:65-66` 并排放着 `pub mod soft_body;`（fork 的 `SoftBodySet`，5,016 行 / 89 入口）与 `pub use mps_formula::softbody;`（758 行 / 7 入口）。5 组函数一一对应（`softbody_solve_xpbd_distance_constraints` ↔ `soft_body.rs:2641 solve_distance_constraint` 等），是两套独立实现，不是「公式 + 状态」。**「公式层」已经不是物理的唯一真相。**
3. **`scientists/` 是各域模块的第二份活实现**：`scientists/mod.rs:4` 与 `albert_einstein.rs:3-5` 都声称「实现放在 scientists，域模块仅 `pub use` 重导出」，但 `relativity.rs` 里 `pub use` 数量为 **0**，`relativistic_total_energy` 在 `relativity.rs:1138` 与 `albert_einstein.rs:428` 各有一份。同名 `pub fn` 跨文件重复约 **120 组**；`isaac_newton.rs`（657 行）是 `gravitational_models.rs`（590 行）的整份复制，`joseph_louis_lagrange.rs:61` 还有第三份 `carlson_rf/rd`。
4. **同名函数语义相反**：`kepler_period(mu, a)`（`spaceflight/kepler.rs:177`）vs `kepler_period(a, mass)`（`scientists/johannes_kepler.rs:112`）——同 crate、同名、同为 `(f64,f64)->Option<f64>`，编译器抓不到。
5. **SPH 核函数三份**：fork `fluid.rs`、`mps-formula/src/scientists/leonhard_euler.rs:161`、`mps-core/src/rapier/fluid.rs:11-14`（走 formula 那份）。只有 fork 那份有 2.5 的自贡献 bug。
6. **`checked_input_slice` 两份**：`mps-formula/ffi/convert.rs:32`（返 `Result`）与 `mps-core/ffi/convert.rs:22`（返 `Option`），语义不同。
7. **`write_out` 五份逐字相同**：`chaos.rs:29`、`plasma.rs:34`、`relativity.rs:20`、`superfluidity.rs:33`、`wave_optics.rs:30` vs `math.rs:535`；`math.rs:15` 因此开了模块级 `#![allow(dead_code)]`，把约 40 个工具的真实使用率永久隐藏。
8. **单体化**：`soft_body.rs` 5,016 行 / 89 入口 / 文件内实现与 FFI 混排；`world_step` 536 行；`rigid_body.h` 9,791 行。改任一体家族都要重编整个 cdylib，且头文件已经大到任何 C 端 `#include` 都要解析 372 KB。

---

## 5. P1 — 质量门禁给出虚假保证（这部分比代码 bug 更值得先修）

1. **`verify_module_mirror` 的「孤儿」分支恒空**：`verify_module_mirror.rs:119-122` 先把 `test_rapier.keys()` 并入 `expected`，`:146-150` 的 `orphans` 因此永远为空。实证：`force_queue.rs`、`force_queue_integration.rs`、`performance_matrix.rs` 三个测试专用文件都不在 `:130-137` 的白名单里，本该报 orphan 却通过。
2. **6 个测试文件从未编译、29 个测试永不运行，却计入公开的 `TEST_COUNT`**：`crates/mps-test/src/rapier/compat.rs`（479 行 / 16 测试）不在 `lib.rs` 的模块树；`crates/mps-test/src/cosmos/flight/`（4 文件 / 13 测试）全仓没有 `mod flight`。`metrics.rs` 的 `TEST_COUNT=1165` 里含这 29 个，实际可跑 1136。
3. **metrics 守门只覆盖 18 个常量中的 3 个**：`verify_metrics_sync.rs:162` 只比 `TEST_COUNT`/`JNI_METHOD_COUNT`/`CORE_FFI_COUNT`，而 `xtask/src/main.rs:418-474` 写出 18 个。且守门与生成器是**同一份扫描逻辑的复制**（都只认 `line.trim() == "#[test]"`），共同盲区无法发现。
4. **`version_consistency` 两条「交叉校验」恒真**：`version_consistency.rs:117-142` 与 `arena_compat.rs:93-98` 完全同构。
5. **`controller.rs` 25 个测试只有 11 个断言**，15 个 `rejects_*` 测试没有任何「被拒绝」的观测量（返回值被丢弃）。
6. **关键模块的镜像只覆盖极少数入口**：`mps-test/src/rapier/world.rs` 1 个测试对 130 个 `world_*` 入口；`ffi.rs` 1 个对 4 个源文件。
7. **`web_audit.py` 的 stat-num 检查 0/3 命中**：`web_audit.py:93-101` 的正则要求 `class: "stat-num", "<字面量>"`，实际写法是 `class: "stat-num", { FFI_SOFT_BODY }`。把常量换成硬编码数字也不会失败。锚点检查只 `PAGES.glob("*.rs")`（非递归），漏掉 `layouts/mod.rs` 里 46 处引用中的 28 处。
8. **CI 的盲区**：`anvilkit-bridge` / `relative-force` / `profiler` 三个 feature 全流程无人编译（ROADMAP:22 自认 `anvilkit-bridge` 测试是坏的）；`cargo build --release` 只 build 不 test（release 的 `panic="unwind"`/LTO/strip 无覆盖）；没有 `dump-metrics` + `git diff` 步骤。
9. **源 crate 里有 17 个 `#[test]`**（`cross_validate.rs:796` 4 个、`forces.rs:985` 1 个、`celestial_data.rs:1050` 3 个、`cosmos/radio.rs:508` 8 个、`mps-web/src/lib.rs:31` 1 个），直接违反 AGENTS.md:30 / ROADMAP.md:30 / ADR-0005。
10. **9 个 `#[ignore]` 里 7 个没有任何文档或 CI 钉住**（`parallel.rs` 6 个 + `cosmos/integrator.rs:529`，后者连 ignore 理由都没写）。

正面：`cloth.rs`、`granular.rs` 的镜像**是合格的**（断言 spring 总数恰为 39、逐点坐标、sag 单调性、撕裂后数量减少）。问题不是「镜像都是空壳」，而是**镜像质量没有下限约束** —— 而 2.4/2.5 这类缺陷恰好全落在弱镜像家族（tire / fluid / hair）。

---

## 6. P1 — 数值与物理正确性

| 缺陷 | 位置 | 后果 |
| --- | --- | --- |
| 轮胎力的量纲错了 | `tire_model.rs:294`（`normal_load.powf(0.8)`）+ `:325`（`μ · load_factor`） | `μ · N^0.8` 不是力（`TireParams` 里没有参考载荷 Fz0）。1200 kg 车单轮静载 2943 N 时峰值力 ≈715 N，正确值 ≈3531 N —— **抓地力低约 5 倍且不随载荷线性增长**。同一错误量也进了摩擦椭圆上限 `:337` |
| 制动力增量漏乘 `dt` | `tire_model.rs:305` | `:304` 的驱动项有 `dt`，`:305` 的刹车项没有 → 30 Hz 与 120 Hz 调用同一 `brake` 值得到 4 倍不同的减速度 |
| 滑移曲线按 `1/|s|` 衰减 | `tire_model.rs:320-332` | 超出峰值后 `fx = peak·μ/|s|` → **完全抱死时制动力趋 0**、大侧滑时侧向力趋 0（把 ABS 失效的语义反过来了）。测试 `tire_model.rs:219` 只断言 `fx <= 0.0`，把 bug 固化成期望 |
| DEM 完全重合粒子受零排斥力 | `rapier/src/dynamics/granular.rs:188-197` | `d2 == 0` 时 `n_hat = 0` → 法向/切向力都是零向量，尽管 `overlap = rsum > 0`。批量同坐标播种、体素挖格产出的颗粒会永久叠在一起 |
| DEM 显式积分稳定性只在注释里 | `granular.rs:64-65`（注释）vs `:135`/`:225-226` | 「`k/m·dt² < 1`」无人校验、无子步细分；`k=1e6, m=0.01, dt=1/60` → 判据值 27,800，一个步长内颗粒穿出世界 |
| 软体碰撞组每 31 个 id 复用一位 | `soft_body.rs:5041-5044`（`1u32 << ((id % 31) + 1)`） | 第 32 个开碰撞耦合的软体开始，`id₁ ≡ id₂ (mod 31)` 的两体互不碰撞，完全静默 |
| 光子动量必得 NaN | `relativity.rs:640`（`dir * f64::INFINITY`） | 轴向光子 `dir = (1,0,0)` → `0*inf = NaN`，返回 `Bool::TRUE`。而 `:597-600` 的文档描述的正是这个不可能的结果 |
| 成功返回却写 NaN / INF | `relativity.rs:493-507`（`orbital_factor: NaN`）、`relativity.rs:109-113`（矩阵里 `INFINITY`）、`plasma.rs:129-133`（`debye_len = INFINITY`） | 直接违反 `docs/formula-numeric-domains.md:9/13-14/24-25` |
| 失败返回不写错误槽 | `relativity.rs:331/381/432/481`、`wave_optics.rs:656/987`、`superfluidity.rs:965-978`、`transmission.rs:315-323` | 与 `formula-numeric-domains.md:22-26` 的承诺矛盾 |
| MARS 太阳光压漏了 `1/a²` | `celestial_data.rs:488` vs `:549`(水星)/`:569`(金星)/`:508`(木星)/`:528`(土星)/`:589`/`:609` | 其余行星都除了 `a²`，只有火星用了 1 AU 值 → 高估 **2.32 倍** |
| 9 颗卫星的 `gm` 是占位值 | `celestial_data.rs:866/882/930/978/1003/1011/1027/1035/1043` | 按 `ρ = 3GM/(4πGR³)` 反算：Pan 105,982 kg/m³（锇 22,600）、Prometheus 45,784、Janus 26,005、Enceladus 849（冰卫星应 1,610）… 全是 `1.0e7/1.0e8/2.0e9` 这种圆整数。守护测试 `moons_satisfy_kepler_third_law` 用 `planet_gm + m.gm`，卫星 GM 小 5–6 个数量级，**结构上不可能发现** |

---

## 7. P2 — 工程与可复现性

- **没有 `rust-toolchain.toml`，也没有 `rust-version`**（10 个 crate 全无）。CI 用 `dtolnay/rust-toolchain@stable` 浮动。而代码里有 15 处 let-chains（如 `world.rs:646`），edition 2024 + let-chains 实际要求 **≥1.88**，这个 MSRV 从未声明。
- **干净 clone（不带 submodule、无网络）不能构建**：`Cargo.toml:26` 的 `rapier3d` 是 path 依赖指向 submodule，`.cargo/config.toml` 没有 `[source]` 替换、仓库里没有 vendor 目录。
- **每次 `cargo build` 都可能改写受版本控制的两份头文件**：`mps-build-common/src/lib.rs:78-80` 把 cbindgen 输出写回 `crate/include/`，触发条件是 `rerun-if-changed=src`。生成到 `OUT_DIR` + 显式 `xtask gen-headers` 更干净。
- **`dashmap 7.0.0-rc2`** 作为生产依赖（`Cargo.toml:34`），lock 里同时存在 6.2.1。
- **文档可证伪的声明**：`docs/evidence/module-mirror.md:5`（白名单 6 个 vs 实际 8 个 test-only 文件）、`docs/evidence/metrics.md:5`（「若干分项一致」实际只守 3 项）、`docs/mps-core.md:3`（`body_spatial_index`/`collision_mode`/`registry` 三个源模块无页面）。
- **文档宣称的 JNI 类名在产物里不存在**：`mps-web/src/pages/jni.rs:67-68` 写 `org.polaris2023.mps_rigid_body.RigidBodyNative`，`bridge.rs:49/129` 也以 `RigidBodyNative.*` 为例；实际只有 `RapierNative` / `SpaceNative` / `RustMemoryFree` 三个前缀。照文档写 Java 会 `UnsatisfiedLinkError`。
- **`COSMOS_PROFILE` 不是零开销**：`cosmos/world.rs:1355` 每个 phase 都 `std::env::var(...)`（每子步 4 次环境查找 + String 分配），与 `lib.rs:21-29` 的「默认零开销」相反。`OnceLock<bool>` 就够。
- **n-body 恒为 O(N·M)**，而 `mps-formula/src/astrophysics.rs:372` 已有 Barnes-Hut 且**全仓无调用者**。

---

## 8. 建议的修复顺序（按性价比，不按严重度）

**第一批：三行级、零风险、立刻降低误导**
1. `world_step` 的 dt 校验补 `set_error`（对齐 `world_set_integration_parameters`）。
2. `ffi_scalar` 的 `None` 分支补 `set_error(ERR_INVALID_ARGUMENT, ...)`。
3. `verify_module_mirror` 的 `expected` 去掉 `test_rapier.keys()`；补 `pub mod` 可达性校验（这样 `compat.rs`/`flight/` 会立刻红灯）。
4. 把 `compat.rs`、`cosmos/flight/` 接进模块树，或删掉并从 `TEST_COUNT` 口径里排除。
5. `metrics.rs` 的 18 个常量全部纳入守门；CI 加 `dump-metrics && git diff --exit-code`。

**第二批：共享内存通道（这三条是「静默算错」的根源）**
6. `arena_idx_map` 在插入/删除路径失效（不要只看长度）；cosmos 同步修。
7. `drain_arena_commands()` 移到 `reset_forces` 之后，或 reset 只清非 arena 来源。
8. 统一 handle 编码：arena handle map 写 packed，或在 `force_queue` 侧接受裸 id；cosmos 与 core 的位序对齐。

**第三批：内存安全**
9. `cosmosBuilderLockTranslations` 改原地 `mem::replace`（JNI + `cosmos/ffi.rs:174-185` 同修）。
10. `rigid_body_consume_force_queue` 加 `ffi_guard` + capacity 上界 + `head-tail ≤ capacity` 校验；`is_paused` 改原子读。
11. `soft_body_destroy` / `*_enable_collision(TRUE)` 抽一个 `teardown_soft_body()` 共用；`skin_bindings`/`voxel_soft_meta` 一并清。
12. JNI 数组长度校验（compound boxes / voxel bytes）。

**第四批：物理**
13. 轮胎：加 `reference_load`，力改成 `μ·N·(N/Fz0)^(n-1)`；brake 乘 `dt`；滑移曲线改饱和而非 1/x。
14. SPH 密度循环补 `if i == j { continue }`，并给 `fluid_get_particle` 的密度通道补一条断言。
15. `rel_particle_properties` 光子分支、`rel_gravitational_time_dilation` 的 NaN 分支、`pl_plasma_params` 的 INF 分支改成失败。
16. MARS 光压 / 9 颗卫星 GM 按 JPL fact sheet 重填，加 `implied_density ∈ (300,4000)` 断言。

**第五批：结构（要 ADR，不能顺手做）**
17. 删掉 `mps-formula` 的 163 个 `extern "C"`（无头文件、无调用者、无 guard）—— 净减 163 个不安全入口，零成本。
18. `spaceflight` 收敛到 formula，core 只做包装；`scientists/` 只留元数据。
19. `mps-formula::softbody` 与 fork `SoftBodySet` 选一个权威（按 ADR 应是 formula），另一个降级为状态容器。
20. 线程模型写成文档 + 让 `query_lock` 覆盖所有变更入口（或删掉它，明确声明单线程）。

---

## 9. 我没能验证的

- **构建与测试没跑起来**（沙箱拒绝 spawn `cargo`）。所以「`anvilkit-bridge` 编译是坏的」「干净 clone 不能构建」是**从配置和 path 依赖推断**，不是实测。
- **Java 宿主不在仓库**，所以 JNI 契约只能从 Rust 侧看；radio 的 18 vs 17 stride 这类问题可能有一侧是故意的，需要 Java 侧确认。
- **`rapier/` fork 与上游的差异无法评估**（git 不可用，submodule 状态未知）。282k 行的 fork 本身是已接受的 ADR-0007 决定，但「同步上游」没有任何流程记录。
- 报告中 2.1b、2.1c、2.3、3（句柄编码/宏冲突）、4.1–4.5、6 的大部分条目，我逐条回到源码确认过；少数（JNI 局部引用、`granular_dig_spawn` 单槽、`hair_system_build` 中途失败锁死）只做了代码路径阅读，未做运行时验证。
