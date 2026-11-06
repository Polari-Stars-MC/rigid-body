# rapier/soft_body.rs

## 作用
软体 FFI 主模块（仓库中最大的源文件，5000+ 行，87 个入口）。覆盖两层：
1. **骨骼式软体**（Phase 1 route A，`soft_chain_*`）：刚体"节点"链/树 + 弹簧关节——最便宜的软体表达，完全复用刚体求解器与冲量关节。
2. **点质量软体**（fork `SoftBody` 的 FFI 全集）：`soft_body_*` 系列管理 fork 内 `SoftBodySet` 中的点质量体，覆盖构建（`soft_body_build_*`、体素 `soft_body_voxel_build`、布料三角形 `add_triangle`/`add_bending`、四面体 `add_tetrahedron`）、参数（29 个 `soft_body_set_*`：刚度/阻尼/重力/睡眠/风/求解器切换/XPBD 逐约束柔度/主动应变等）、增量编辑（`add_particle`/`add_spring`/`add_distance_constraint`）、撕裂（`soft_body_tear_*`）、细分（`subdivide`）、状态读写（`soft_body_save_state`/`restore`）、查询（11 个 `soft_body_read_*`：粒子/边/三角形/四面体/AABB/表面网格/应力/法线/接触力，配 `soft_body_get_particle`、`soft_body_particle_count`）。

碰撞耦合走 Phase 5f：`soft_body_enable_collision` 为每个自由粒子生成代理刚体 + 球碰撞体（按体分配碰撞组避免同体自爆），`world.rs` 在刚体步前把粒子力/位姿推进代理、步后把接触位姿回读。皮肤绑定（LBS）`soft_body_skin_*` 把骨骼刚体系到粒子上做线性混合蒙皮。粒子也可 `attach`/`detach` 到刚体（Phase 8），弹簧力经 `write_spring_forces` 回传。

## 关键导出（分组，共 87 个）
- 骨骼链：`soft_chain_create`、`soft_chain_node_handles`。
- 构建/编辑：`soft_body_build_*`（cloth/chain 等）、`soft_body_voxel_build`（体素网格→粒子+弹簧，Phase 5d 记录体素↔粒子映射供挖掘联动）、`add_particle`/`add_spring`/`add_distance_constraint`/`add_tetrahedron`/`add_triangle`/`add_bending`。
- 参数：`soft_body_set_*` ×29（见源码；含求解器 MassSpring↔Xpbd 切换、风场、重力、睡眠）。
- 碰撞/耦合：`soft_body_enable_collision`、`soft_body_apply_*`（冲量/风）、attach/detach 粒子到刚体。
- 查询：`soft_body_read_particles`/`_edges`/`_tetrahedra`/`_triangles`/`_aabb`/`_surface_mesh`/`_stress`/`_normals`/`_contact_force`/`_spring_forces`、`soft_body_get_particle`、`soft_body_particle_count`、`soft_body_kinetic_energy`/`_total_*`、`soft_body_state_*`。
- 生命周期：`soft_body_save_state`/`restore_state`、`soft_body_tear_*`（应力阈值撕裂）、`soft_body_subdivide_*`、`soft_body_wake_up`/`sleep`、`soft_body_is_*`、`soft_body_remove_*`、`soft_body_scale_*`、`soft_body_clear_*` ×8、`soft_body_step_*`。

## 依赖
- fork `rapier3d::prelude::soft_body_fork::{SoftBody, SoftBodySet, SoftBodyId, SoftSolver, SoftParticle, Spring, DistanceConstraint}`。
- `crate::rapier::world`（`soft_body_proxies`、`voxel_soft_meta`、step 内的耦合流水线）。
- `crate::rapier::ffi`（`Bool`、`Vec3`、`WorldHandle`、handle 打包、`vec3_*`）、`crate::rapier::error`。

## 测试
`mps-test/src/rapier/soft_body.rs`（主套件）、`softbody.rs`（早期/骨骼链）、`fluid_sph.rs` 的代理耦合对照。
