# rapier/hair.rs

## 作用
毛发/皮毛系统。一个 hair system 是附着在刚体上的一组发丝（strand），每根发丝一个链式软体：粒子沿局部方向排布、相邻粒子以 MassSpring 弹簧相连（刚度/阻尼可调），根部粒子通过 fork 的 `SoftBody::attach_particle` 绑定到宿主刚体（发丝跟随刚体运动，弹簧力经 `write_spring_forces` 回传）。支持整体风场（fork 内建 `apply_wind` 恒定风加速 + 线性阻力）与逐系统重力缩放（0 = 无重力，如水下头发）。`create` 只登记描述，`build` 才实际创建软体（可延迟、可复用同一描述重建）。

## 关键导出
- `pub extern "C" fn hair_system_create(world, attached_body, strands*, strand_count)` — 登记发丝描述，返回稳定 id。`HairStrandDesc`（repr(C)，96 字节/项）含 root_local/direction/segment_count/length/segment_radius/stiffness/damping/density。
- `pub extern "C" fn hair_system_build(world, id)` — 按描述创建各发丝软体（重复 build 报 `ERR_UNSUPPORTED`）。
- `pub extern "C" fn hair_system_set_wind(world, id, wind)` — 设置风场并推送到已建软体（未建时缓存，build 时应用）。
- `pub extern "C" fn hair_system_set_gravity_scale(world, id, scale)` — 重力缩放，实时改写已建软体的 gravity。
- `pub extern "C" fn hair_system_strand_soft_body(world, id, strand_index) -> u32` — 查询发丝的 `SoftBodyId.0`（配合 `soft_body_get_particle` 读取粒子做渲染）。
- `pub extern "C" fn hair_system_remove(world, id)` — 移除系统并删除各发丝软体。
- 内部：`HairSystem`（世界内 `hair_systems` 哈希表）、`MAX_HAIR_STRANDS = 512`、`MAX_HAIR_SEGMENTS = 64`、`FALLBACK_STIFFNESS`（stiffness 为 0 时的弹簧常数兜底）、`HAIR_WIND_DRAG`。

## 依赖
- fork `rapier3d::prelude::soft_body_fork::{SoftBody, SoftBodyId}` — `new`/`add_particle`/`add_spring`/`attach_particle`/`apply_wind`。
- `crate::rapier::ffi` — `Bool`、`Vec3`、`WorldHandle`、`RigidBodyHandleRaw`（`unpack_rigid_body_handle`）、`vec3_finite`/`vec3_to_rapier`。
- `crate::rapier::error` — 错误码与 `ffi_guard`。

## 测试
`mps-test/src/rapier/hair.rs`：生命周期（create→build→wind→gravity→step→remove）、根部粒子跟随自由落体刚体（`soft_body_get_particle` 验证 y 下降）、非法输入拒绝（空指针/容量/未知宿主/坏描述/负缩放/NaN 风）、strand 索引越界、null world。
