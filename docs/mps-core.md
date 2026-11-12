# mps-core 源码导览

`mps-core` 是 `mps_rigid_body` 的物理世界 + Rapier 封装 + C ABI crate（`crates/mps-core`）。本文档按子模块分区，链接到 `docs/mps-core/` 下每个源文件的作用分析。所有分析基于真实源码。

分层、句柄和不许跨越的边界不在这里重复，见 [ARCHITECTURE.md](ARCHITECTURE.md) 与 [BOUNDARY.md](BOUNDARY.md)。文档索引在 [README.md](README.md)。

源码根: `crates/mps-core/src/`。crate 入口 `lib.rs` 把 `rapier` 模块整体导出。

---

## 入口与模块根

- [lib.md](mps-core/lib.md) — `src/lib.rs`：crate 入口，re-export `rapier` 模块与 `jni_api`，anvilkit-bridge feature 门控。
- [rapier-mod.md](mps-core/rapier-mod.md) — `src/rapier/mod.rs`：`rapier` 顶层模块，声明并 re-export 全部子模块。

## 物理世界核心

- [world.md](mps-core/world.md) — `world.rs`：`PhysicsWorld` 主结构与 `world_*` C ABI 入口（创建/步进/重力/积分参数/共享竞技场/相对力）。
- [rigid_body.md](mps-core/rigid_body.md) — `rigid_body.rs`：刚体 builder/创建/位姿/速度/力/脉冲/CCD/sleep 等 `rigid_body_*` 入口。
- [collider.md](mps-core/collider.md) — `collider.rs`：碰撞体 builder/创建/材质/分组/事件/钩子等 `collider_*` 入口。
- [events.md](mps-core/events.md) — `events.rs`：碰撞/接触力事件队列 + hooks 回调。
- [debug_recording.md](mps-core/debug_recording.md) — `debug_recording.rs`：Jolt-Viewer 风格录制回放（环形缓冲、13-lane 帧快照、v2 事件段、`debug_recording_*` 入口）。
- [forces.md](mps-core/forces.md) — `forces.rs`：力法则注册表、BodyForceLog、ForceFacade。
- [parallel.md](mps-core/parallel.md) — `parallel.rs`：每帧工作的多线程执行（两段式逐体力填充、chunk-pair 成对引力、并行快照、线程池 FFI）。

## 查询与交互

- [query.md](mps-core/query.md) — `query.rs`：射线/点/AABB/OBB/球/形状投掷等 `query_*` 空间查询入口。
- [interaction.md](mps-core/interaction.md) — `interaction.rs`：交互组、碰撞分组、接触交互配置。
- [joints.md](mps-core/joints.md) — `joints.rs`：关节 builder 与 `joint_*` 入口。
- [controller.md](mps-core/controller.md) — `controller.rs`：角色控制器与碰撞响应。
- [batch.md](mps-core/batch.md) — `batch.rs`：批量操作（批量创建/查询/事件读取）。

## 空间索引与体素

- [crbtree.md](mps-core/crbtree.md) — `crbtree.rs`：紧凑 R 树（CRbTree）空间索引。
- [rtree.md](mps-core/rtree.md) — `rtree.rs`：R 树空间索引。
- [voxel.md](mps-core/voxel.md) — `voxel.rs`：体素碰撞体构建（AABB/OBB/网格）与体素查询。

## 边界与范围

- [bounds.md](mps-core/bounds.md) — `bounds.rs`：扩展边界 builder（胶囊/SSV/椭球/棱柱/圆柱/壳/kDOP/FDH/神经边界）。
- [dop.md](mps-core/dop.md) — `dop.rs`：k-DOP 有向离散定向多面体边界。
- [neural.md](mps-core/neural.md) — `neural.rs`：神经学习边界（NeuralBounds）。

## 物理域扩展

- [aerodynamics.md](mps-core/aerodynamics.md) — `aerodynamics.rs`：气动面/体素网格施力 `aero_*` 入口。
- [disasters.md](mps-core/disasters.md) — `disasters.rs`：自然灾害（台风/龙卷风/冰雹）场模拟 `disaster_*` 入口。
- [volcano.md](mps-core/volcano.md) — `volcano.rs`：火山喷发（烟柱流场/加热/火山弹）与刚体渐进熔化 `volcano_*` 入口。
- [soft_body.md](mps-core/soft_body.md) — `soft_body.rs`：软体 FFI 主模块（骨骼链 + 点质量体 87 个入口、Phase 5f 碰撞代理、蒙皮、撕裂/细分/存档）。
- [character_body.md](mps-core/character_body.md) — `character_body.rs`：角色体（第三种体，运动学 KCC 驱动，`character_body_*` 入口）。
- [cloth.md](mps-core/cloth.md) — `cloth.rs`：布料体（矩形网格 + 结构/剪切/弯曲三族弹簧，`soft_cloth_*` 入口）。
- [rope.md](mps-core/rope.md) — `rope.rs`：绳索/缆绳体（单向 cable 约束 + 绞盘，`soft_rope_create` 入口）。
- [rope_knot.md](mps-core/rope_knot.md) — `rope_knot.rs`：绳结/编织系统（每股一软体 + 股间碰撞代理，`rope_knot_*` 入口）。
- [hair.md](mps-core/hair.md) — `hair.rs`：毛发/皮毛系统（链式软体发丝 + 粒子锚定刚体，`hair_system_*` 入口）。
- [tire_model.md](mps-core/tire_model.md) — `tire_model.rs`：Pacejka 简化轮胎模型（叠加车辆控制器，`tire_model_*` 入口）。
- [vehicle.md](mps-core/vehicle.md) — `vehicle.rs`：射线悬架车辆（第五种体，`vehicle_controller_*` 入口）。
- [sensor.md](mps-core/sensor.md) — `sensor.rs`：传感器触发区（第四种体，重叠集跟踪，`sensor_zone_*` 入口）。
- [servo_body.md](mps-core/servo_body.md) — `servo_body.rs`：PD/PID 伺服体（第六种体，速度级驱动到目标位姿，`servo_body_*` 入口）。
- [balloon.md](mps-core/balloon.md) — `balloon.rs`：气囊/充气体（闭合受压球壳 + Phase 11 压力模型，`soft_balloon_create` 入口）。
- [granular.md](mps-core/granular.md) — `granular.rs`：颗粒体（DEM 接触模型 + Coulomb 摩擦，`granular_*` 入口）。
- [fluid.md](mps-core/fluid.md) — `fluid.rs`：流体力学力（浮力/阻力/SPH 等）。
- [fluid_sph.md](mps-core/fluid_sph.md) — `fluid_sph.rs`：SPH 流体体（fork `FluidWorld` 薄封装 + 碰撞代理，`fluid_*` 入口）。
- [molecular.md](mps-core/molecular.md) — `molecular.rs`：分子力（Lennard-Jones/Coulomb）。
- [fracture.md](mps-core/fracture.md) — `fracture.rs`：断裂力学与刚体碎裂 `fracture_*` 入口。
- [fracture_mesh.md](mps-core/fracture_mesh.md) — `fracture_mesh.rs`：可碎裂复合刚体（触发器/疲劳/应力阈值/Voronoi 预切割，`fracture_mesh_body_*` 入口）。
- [trajectory.md](mps-core/trajectory.md) — `trajectory.rs`：轨迹力估计/积分/施加 `trajectory_*` 入口。
- [terrain_gravity.md](mps-core/terrain_gravity.md) — `terrain_gravity.rs`：不规则天体/地形重力（多面体/DEM/Mascon）。
- [cross_validate.md](mps-core/cross_validate.md) — `cross_validate.rs`：多公式交叉验证引力（Newton 锚定/Mean/Median 聚合的 ForceLaw）。

## 公式 FFI（纯计算器，无世界状态）

以下模块都是 `mps-formula` 纯标量公式的 C ABI 薄封装：复用 `ffi_scalar`（null `out` 或 `None` → `Bool::FALSE`），不触碰 `WorldHandle`/Rapier 状态；逐函数显式写出（cbindgen 不展开声明宏）。

- [acoustics_ffi.md](mps-core/acoustics_ffi.md) — `acoustics_ffi.rs`：声学标量公式（扩散/吸收/RT60/阻抗/Doppler/声呐，`acoustics_*` 前缀）。
- [astrocalc.md](mps-core/astrocalc.md) — `astrocalc.rs`：天体物理（Hill/Roche/NFW/黑体/Jeans/双星/系外行星，`astrophysics_*` 前缀）。
- [emag.md](mps-core/emag.md) — `emag.rs`：电磁学（平面波/天线/Friis/VSWR/Rayleigh/Faraday，`electromagnetism_*` 前缀）。
- [matmech.md](mps-core/matmech.md) — `matmech.rs`：材料力学（屈服/断裂/疲劳/蠕变/梁柱，`material_mechanics_*` 前缀）。
- [nucphys.md](mps-core/nucphys.md) — `nucphys.rs`：核物理（衰变/结合能/聚变裂变 Q 值/四因子，`nuclear_*` 前缀）。
- [plasma_ffi.md](mps-core/plasma_ffi.md) — `plasma_ffi.rs`：等离子体磁流体（beta/回旋频率/拉莫尔半径/磁镜，`plasma_*` 前缀）。
- [qphys.md](mps-core/qphys.md) — `qphys.rs`：量子力学（势阱/氢原子/康普顿/Landau/Rabi，`quantum_*` 前缀）。
- [rel.md](mps-core/rel.md) — `rel.rs`：相对论（Kerr 黑洞/引力波/相对论运动学/透镜/宇宙学，`relativity_*` 前缀）。
- [thermo.md](mps-core/thermo.md) — `thermo.rs`：热力学（理想气体/多方过程，`thermodynamics_*` 前缀）。

## 空间飞行子模块（src/rapier/spaceflight/）

原 2610 行 `spaceflight.rs` 按 OPTIMIZATION.md §3 拆分为 8 个 per-domain 子模块，共用 [spaceflight-mod.md](mps-core/spaceflight-mod.md) 中的辅助与常量。

- [spaceflight-mod.md](mps-core/spaceflight-mod.md) — `mod.rs`：子模块枢纽，共用数值辅助与 FFI 类型 re-export。
- [spaceflight-debris.md](mps-core/spaceflight-debris.md) — `debris.rs`：SGP4 J2 进动 + 碎片碰撞概率。
- [spaceflight-dynamics.md](mps-core/spaceflight-dynamics.md) — `dynamics.rs`：相对运动制导/机械臂/柔性/对接/变分/bang-off-bang。
- [spaceflight-gnss.md](mps-core/spaceflight-gnss.md) — `gnss.rs`：GNSS 伪距/双差、Friis 链路、雷达测速。
- [spaceflight-kepler.md](mps-core/spaceflight-kepler.md) — `kepler.rs`：开普勒轨道力学基础（根数↔状态/Lambert/Hohmann/Tsiolkovsky）。
- [spaceflight-perturbation.md](mps-core/spaceflight-perturbation.md) — `perturbation.rs`：J2/大气阻力/太阳辐射压/重力梯度等摄动力施加到刚体。
- [spaceflight-propulsion.md](mps-core/spaceflight-propulsion.md) — `propulsion.rs`：推进与电源（Hall/Sabatier/电池/太阳板/接触力）。
- [spaceflight-rotation.md](mps-core/spaceflight-rotation.md) — `rotation.rs`：姿态确定与控制（CMG/TRIAD/EKF/磁场力矩器）。
- [spaceflight-thermal.md](mps-core/spaceflight-thermal.md) — `thermal.rs`：热控（热管/辐射器/气闸/SPE 氧气/Whipple）。

## C ABI 层（src/rapier/ffi/）

- [ffi-mod.md](mps-core/ffi-mod.md) — `ffi/mod.rs`：FFI 模块根。
- [ffi-convert.md](mps-core/ffi-convert.md) — `ffi/convert.rs`：`Vec3`/`Quat` 与 Rapier `Vector`/`Rotation` 双向转换、pack/unpack 句柄。
- [ffi-types.md](mps-core/ffi-types.md) — `ffi/types.rs`：所有 `#[repr(C)]` ABI 结构体与句柄类型别名（WorldHandle、ColliderHandleRaw 等）。
- [ffi-force-queue.md](mps-core/ffi-force-queue.md) — `ffi/force_queue.rs`：共享内存力队列（SPSC 无锁、DirectByteBuffer 零拷贝施力，`rigid_body_consume_force_queue`）。

## 共享竞技场（src/rapier/shared_arena/）

零拷贝 Rust↔外部 内存竞技场，供 `mps-jni` DirectByteBuffer 桥接。

- [shared-arena-mod.md](mps-core/shared-arena-mod.md) — `mod.rs`：`SharedPhysicsArena` 主结构与 C ABI 入口。
- [shared-arena-layout.md](mps-core/shared-arena-layout.md) — `layout.rs`：竞技场内存布局。
- [shared-arena-header.md](mps-core/shared-arena-header.md) — `header.rs`：竞技场头部元数据。
- [shared-arena-holes.md](mps-core/shared-arena-holes.md) — `holes.rs`：空闲槽位管理（holes 链表）。
- [shared-arena-ring.md](mps-core/shared-arena-ring.md) — `ring.rs`：事件环形缓冲区。

## 兼容、桥接与错误

- [compat.md](mps-core/compat.md) — `compat.rs`：旧 API 兼容的刚体插入与 AABB 查询。
- [bridge.md](mps-core/bridge.md) — `bridge.rs`：Rust↔Java 零拷贝内存桥（JNI DirectByteBuffer，`pub unsafe fn`，非 extern "C"）。
- [error.md](mps-core/error.md) — `error.rs`：线程局部错误槽、`ffi_guard`、`last_error_*` ABI、7 个 `ERR_*` 常量。

## 特性桥接

- [anvilkit.md](mps-core/anvilkit.md) — `anvilkit.rs`：`anvilkit-bridge` feature 下的 AnvilKit 集成（app create/update/destroy），`#[cfg(feature = "anvilkit-bridge")]` 门控。
