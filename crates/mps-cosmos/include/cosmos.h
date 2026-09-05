#ifndef COSMOS_H
#define COSMOS_H

#pragma once

/* Generated with cbindgen:0.29.4 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Magic number identifying a valid cosmos arena: "COSMAREN".
 */
#define ARENA_MAGIC 4850186914974811470

/**
 * Current cosmos arena layout version — increment on any layout change.
 */
#define ARENA_VERSION 1

/**
 * Body slot stride (must match Java side exactly).
 */
#define BODY_SLOT_STRIDE 96

/**
 * Command slot stride — 5 × u64 (cmd_type, body_index, a0, a1, a2).
 */
#define CMD_SLOT_STRIDE 40

/**
 * Header size in bytes.
 */
#define HEADER_SIZE 128

/**
 * Upper bounds for arena capacities — defense against absurd FFI requests.
 */
#define MAX_ARENA_BODIES 1000000

#define MAX_ARENA_COMMANDS 1000000

/**
 * Hard cap on the total arena allocation (256 MiB) — also the Java
 * `ByteBuffer.capacity()` 2 GiB ceiling.  Keep ≤ `i32::MAX`.
 */
#define MAX_ARENA_TOTAL_BYTES ((256 * 1024) * 1024)

#define OFF_BODY_COUNT 32

#define OFF_CMD_WRITE 44

#define OFF_BODY_SLOT_BASE HEADER_SIZE

/**
 * Header offset (u64) storing the command-ring base offset (dynamic: depends on
 * max_bodies).  Read this at map time instead of recomputing the layout.
 */
#define OFF_CMD_RING 96

/**
 * 默认近场阈值倍率：|d| ≤ 8·bounding_radius 时走质点求和。8 给到 r² 误差 ~1.5%
 * 的 monopole，足够典型的薄壳/扁平分布过渡到 monopole。
 */
#define NEAR_FIELD_FACTOR 8.0

/**
 * 最大反射次数
 */
#define MAX_BOUNCES 3

/**
 * 每次反射能量保留比
 */
#define REFLECT_ATTENUATION 0.8

/**
 * 命中判定最小距离（m）
 */
#define HIT_EPS 1.0e-6

/**
 * 信号最长存活时间（ms）
 */
#define SIGNAL_TTL_MS 5000

/**
 * 灵敏度兜底下限（W）
 */
#define MIN_SENSITIVITY 1.0e-15

/**
 * 太空物理世界。所有公开 API 自行管理内部 `RigidBodySet` 等。
 *
 * 手写 `Clone`（而非 derive）因为 `PhysicsPipeline` 不实现 `Clone`——它是
 * 无状态的工作对象（每次 `step` 内部重建临时结构），克隆时用 `::new()`
 * 恢复即可。用途：场景快照/回滚（演练器 undo、Monte Carlo 多世界并行）。
 * 成本是深拷贝整个 body/collider set；超大规模场景应考虑 `Arc` 共享只读配置。
 */
typedef struct CosmosWorld CosmosWorld;



#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * 构造一个动态刚体 builder（质量 kg、初始位置/速度）。返回 `*mut` 给调用
 * 方；后续交给 `cosmos_world_insert_body` 插入。失败（panic）返回 null。
 */
RigidBodyBuilder *cosmos_satellite_builder(double mass,
                                           double px,
                                           double py,
                                           double pz,
                                           double vx,
                                           double vy,
                                           double vz,
                                           double radius);

/**
 * 构造固定（静态）刚体 builder —— 适合做 n-body 引力源中心本体。
 */
RigidBodyBuilder *cosmos_fixed_body_builder(double px, double py, double pz);

void cosmos_builder_set_linear_damping(RigidBodyBuilder *builder, double value);

void cosmos_builder_set_angular_damping(RigidBodyBuilder *builder, double value);

void cosmos_builder_set_gravity_scale(RigidBodyBuilder *builder, double value);

/**
 * **激活**平移锁定（动态刚体不再平动，仅可转动）。`RigidBodyBuilder::lock_translations`
 * 是消费 self 的链式 API，这里把裸指针的 builder 取出、调用后再放回原地。
 */
void cosmos_builder_lock_translations(RigidBodyBuilder *builder);

/**
 * 显式释放一个**未插入**的 builder。插入 `cosmos_world_insert_body` 后所有权
 * 已转移，**不要**再调本函数（会 double-free）。null 是 no-op。
 */
void cosmos_builder_destroy(RigidBodyBuilder *builder);

/**
 * 创建一个 `CosmosWorld`。
 *
 * 参数：
 * - `dt`：积分步长（秒），合法范围 `0 < dt ≤ 30`；
 * - `solver_iterations`、`ccd_substeps`：rapier 求解器参数；
 * - `orbit_integration`：0 = `RapierForce`（默认），1 = `Verlet`，
 *   2 = `Yoshida4`，3 = `Yoshida4Kahan`，4 = `ForestRuth8`，5 = `ForestRuth8Kahan`；
 * - `verlet_substeps`：Verlet 路径的内部子步数（≥1，仅 `Verlet` 模式生效）；
 * - `n_body_softening_sq`：n-body 互引力软化平方项（m²），0 表示无软化。
 *
 * 失败（panic）返回 null。
 */
struct CosmosWorld *cosmos_world_create(double dt,
                                        uint32_t solver_iterations,
                                        uint32_t ccd_substeps,
                                        uint32_t orbit_integration,
                                        uint32_t verlet_substeps,
                                        double n_body_softening_sq);

/**
 * 销毁 `cosmos_world_create` 产出的世界。null 是 no-op。
 */
void cosmos_world_destroy(struct CosmosWorld *world);

/**
 * 设太阳位置（光压方向参考）。
 */
void cosmos_world_set_sun_position(struct CosmosWorld *world, Vec3 pos);

/**
 * 设/改 n-body 中心天体（按整数 id：0=Sun..9=Neptune）。`id < 0` 清除。
 * 返回 1 成功 / 0 失败（world 为 null）。
 */
uint8_t cosmos_world_set_central_body(struct CosmosWorld *world, int32_t id);

/**
 * 注册一个天体引力源。`celestial_id` 见 `cosmos_world_set_central_body`；
 * `max_sh_degree` 限制球谐展开最高阶（受 `body.max_degree` 上限约束）。
 * 返回注册到世界中的源索引（≥0 成功；-1 参数错 / world 为 null）。
 */
int32_t cosmos_world_add_celestial(struct CosmosWorld *world,
                                   int32_t celestial_id,
                                   uint32_t max_sh_degree);

/**
 * 注册一个自然卫星（月球）引力源，按 `mps_formula::celestial_data::MOONS`
 * 数组下标查找。越界或空世界返回 -1，成功返回内部索引（与 `add_celestial` 同语义）。
 */
int32_t cosmos_world_add_moon(struct CosmosWorld *world,
                              int32_t moon_index);

/**
 * 把已插入的刚体登记为 n-body 互引力质点源（给定质量 kg）。
 * `body` 是 `cosmos_world_insert_body` 返回的 packed handle。返回 1 / 0。
 */
uint8_t cosmos_world_add_n_body(struct CosmosWorld *world, uint64_t body, double mass);

/**
 * 插入 builder，返回打包的 body 句柄（0 = 失败）。builder 所有权转移。
 * 连带把质量登记为 n-body 源。
 */
uint64_t cosmos_world_insert_body_as_gravity_source(struct CosmosWorld *world,
                                                    RigidBodyBuilder *builder,
                                                    double mass);

/**
 * 插入 builder，返回打包的 body 句柄（0 = 失败）。builder 所有权转移。
 */
uint64_t cosmos_world_insert_body(struct CosmosWorld *world, RigidBodyBuilder *builder);

/**
 * 设置某刚体的环境扰动配置（大气阻力 + 太阳光压 + 太阳风动压 +
 * Chandrasekhar 动力学摩擦）。返回 1 / 0。
 *
 * `sun_position` 通过 `cosmos_world_set_sun_position` 单独设置；太阳风方向
 * 复用 `sun_position → 刚体位置` 的世界方向。
 */
uint8_t cosmos_world_set_perturbation(struct CosmosWorld *world,
                                      uint64_t body,
                                      double drag_coefficient,
                                      double area,
                                      int32_t enable_drag,
                                      double reflectivity,
                                      double optical_area,
                                      int32_t enable_solar,
                                      double solar_wind_proton_density,
                                      double solar_wind_speed,
                                      double solar_wind_area,
                                      int32_t enable_solar_wind,
                                      double friction_background_density,
                                      double friction_coulomb_log,
                                      int32_t enable_dynamical_friction);

/**
 * 推进一步，返回一个 `int` 编码的 `StepResult`：
 * - `>0`：`Stepped(n)` —— 实际推进的秒数 ×1000；
 * - `-1`：`Substepped`（拆子步完成）；
 * - `-2`：`Skipped(NonFinite)`（dt 为 NaN/Inf）；
 * - `-3`：`Skipped(NonPositive)`（dt ≤ 0）；
 * - `-4`：`Skipped(TooLarge)`（dt 超过 30s 硬上限）。
 */
int32_t cosmos_world_step(struct CosmosWorld *world, double dt);

/**
 * 循环 `n` 次推进 `dt`，任一步非法整批拒。
 * 返回 0 = 成功；1 = NonFinite；2 = NonPositive；3 = TooLarge。
 */
int32_t cosmos_world_step_n(struct CosmosWorld *world, double dt, uint32_t n);

/**
 * 创建共享内存 arena（Java 零拷贝命令通道 + 状态回读）。
 *
 * 写入 `out_address` / `out_size`（传 `null` 可跳过对应输出）；返回的 `*mut CosmosWorld`
 * 不变。一个世界最多一个 arena，已存在则原样保留并返回 `false`。容量必须 >0 且
 * 不超过上限，总分配 ≤ 256 MiB。Java 侧用 `out_address`/`out_size` 把这块内存
 * 映射成 native-order 的 `ByteBuffer`，命令环写入 + body 槽零拷贝读取都走它。
 *
 * # Safety
 * `world` 须为 `cosmos_world_create` 产出的有效指针或 null；`out_address` /
 * `out_size` 若为非负则指向 8 字节可写内存。
 */
int32_t cosmos_world_create_shared_arena(struct CosmosWorld *world,
                                         uint32_t max_bodies,
                                         uint32_t max_commands,
                                         uint64_t *out_address,
                                         uint64_t *out_size);

/**
 * 销毁共享 arena（若有的话）。`null` world 是 no-op。销毁前 Java 必须已释放映射
 * 该 arena 的 `ByteBuffer`，否则会 use-after-free。
 *
 * # Safety
 * `world` 须为 `cosmos_world_create` 产出的有效指针或 null。
 */
void cosmos_world_destroy_shared_arena(struct CosmosWorld *world);

/**
 * 取 arena 基地址（无 arena 时返回 0）。供 Java 映射 `ByteBuffer` 的地址来源。
 *
 * # Safety
 * `world` 须为 `cosmos_world_create` 产出的有效指针或 null。
 */
uint64_t cosmos_world_get_shared_arena_address(const struct CosmosWorld *world);

/**
 * 取 arena 总字节大小（无 arena 时返回 0）。供 Java 映射 `ByteBuffer` 的容量来源。
 *
 * # Safety
 * `world` 须为 `cosmos_world_create` 产出的有效指针或 null。
 */
uint64_t cosmos_world_get_shared_arena_size(const struct CosmosWorld *world);

/**
 * 取刚体当前位置（3×f64）。`out` 指向 24 字节 native 缓冲（`Vec3`）。
 * 返回 1 成功 / 0 句柄无效或 world 为 null。
 */
int32_t cosmos_body_translation_out(const struct CosmosWorld *world, uint64_t body, Vec3 *out);

/**
 * 取刚体当前线速度（3×f64）。
 */
int32_t cosmos_body_linvel_out(const struct CosmosWorld *world, uint64_t body, Vec3 *out);

/**
 * 取刚体质量（kg）。`NaN` 表示句柄无效 / world 为 null。
 */
double cosmos_body_mass(const struct CosmosWorld *world, uint64_t body);

/**
 * Hill 球半径（m）：刚体作为卫星时其自引力主导范围，与 Roche 极限互补。
 *
 * 主星质量由 `cosmos_world_set_central_body` 注册的天体 GM/G 反算；卫星质量
 * 取自刚体本身；`semi_major_axis`（m）与 `eccentricity`（0..=1）由调用方提
 * 供。`NaN` 表示无 `central_body` / 句柄无效 / 参数非法。
 */
double cosmos_hill_radius_for(const struct CosmosWorld *world,
                              uint64_t body,
                              double semi_major_axis,
                              double eccentricity);

/**
 * 当前动态刚体数量。
 */
uint32_t cosmos_world_dynamic_body_count(const struct CosmosWorld *world);

/**
 * 动态刚体数量（用于 sizing `cosmos_world_dynamic_body_snapshot` 调用）。
 *
 * 与 [`cosmos_world_dynamic_body_count`] 在当前实现里返回相同数；
 * 单独导出独立计法以与 mps-core `world_dynamic_body_snapshot_count`
 * 的 ABI 形态对齐——Java 端可以以完全相同的 Java 代码模式先用
 * `cosmosWorldDynamicBodySnapshotCount` 拿到 N，分配 `long[N]` 与
 * `double[N * 7]`，再调 `cosmosWorldDynamicBodySnapshot` 拉一次全数据。
 *
 * # Safety
 * `world` 可为 null（返回 0），其余情形须是 `cosmos_world_create` 产出的有效指针。
 */
uint32_t cosmos_world_dynamic_body_snapshot_count(const struct CosmosWorld *world);

/**
 * 批量快照动态刚体的 handle + pose（7 f64/body：pos3 + quat4）。
 *
 * 与 mps-core `world_dynamic_body_snapshot` 完全平行，目的同样：把每 tick
 * Java 端原本要按 N 次 `cosmos_body_translation_out` 往返取所有 pos 的
 * 路径合并成**一次 JNI 调用 + 一份连续 f64[]**——N=1000 卫星的取位延迟
 * 从 ~600 µs/tick 砍到 ~50 µs/tick（见 `性能分析.MD` §11.1 / §12.1，
 * M1 + L1 同根改动）。
 *
 * # 布局
 * - `out_handles[i]`: `pack_handle = (idx << 32) | generation` —— 与
 *   `cosmos_insert_body` / `cosmos_body_translation_out` 等所有 cosmos
 *   body handle ABI 一致（**注意**：与 mps-core 的 `+1/-1` 编码不同）。
 * - `out_values[i * 7 .. i * 7 + 3]`：位置 `pos.x, pos.y, pos.z`
 * - `out_values[i * 7 + 3 .. i * 7 + 7]`：旋转 `quat.i, quat.j, quat.k, quat.w`
 *   （与 rapier3d `Rotation::xyzw` 顺序一致，Java 端可以直接映射到
 *   `Quatd(i, j, k, w)` 或 JOML `Quaterniond`）。
 *
 * 只写动态刚体（`is_dynamic() == true`），跳过 fixed / kinematic —— 与
 * `dynamic_body_count` / `cosmosWorldDynamicBodySnapshotCount` 一致。容量
 * 不够时只填到 `capacity` 为止并返回实际数量；调用方应按 count 先分配。
 *
 * # 返回值
 * 实际写入的 body 数；任一前置参数非法返回 0（并 `set_error`）：
 * - `world` null → `ERR_NULL_POINTER`，返回 0
 * - `out_handles` / `out_values` null，或 `capacity == 0`，
 *   或 `capacity > MAX_OUTPUT_CAPACITY` → `ERR_CAPACITY`，返回 0
 *
 * # Safety
 * `out_handles` 指向至少 `capacity` 个 `u64` 可写内存；`out_values` 指向
 * 至少 `capacity * 7` 个 `f64` 可写内存。`world` 须为 `cosmos_world_create`
 * 产出的指针或 null。调用方应在写入完成前不让另一线程同时操作这两个缓冲。
 */
uint32_t cosmos_world_dynamic_body_snapshot(const struct CosmosWorld *world,
                                            uint64_t *out_handles,
                                            double *out_values,
                                            uint32_t capacity);

/**
 * 启用无线电子世界。之后天体可注册为反射体、收发器可注册/发射。
 */
uint8_t cosmos_world_enable_radio(struct CosmosWorld *world);

/**
 * 查询无线电子世界是否已启用。
 */
uint8_t cosmos_world_radio_enabled(const struct CosmosWorld *world);

/**
 * 注册一个反射天体：`body` 是打包的刚体句柄（`pack_handle`），`radius` 米。
 */
uint8_t cosmos_world_radio_add_reflector(struct CosmosWorld *world, uint64_t body, double radius);

/**
 * 移除反射天体。
 */
void cosmos_world_radio_remove_reflector(struct CosmosWorld *world, uint64_t body);

/**
 * 注册/覆盖一个收发器节点。
 *
 * 参数布局（全部 f64，按顺序）：
 * `id, px, py, pz, vx, vy, vz, dx, dy, dz, frequency, power, sensitivity,
 *  rx_gain, tx_gain, beam_angle, owner_body`。
 * `owner_body`：节点所属刚体句柄（飞船），0 表示无（用于跳过源自身反射）。
 */
uint8_t cosmos_world_radio_register_node(struct CosmosWorld *world,
                                         const double *values);

/**
 * 注销收发器节点（`id` 按 u64 位型传入 f64 槽，即 `f64::from_bits(id)`）。
 */
void cosmos_world_radio_unregister_node(struct CosmosWorld *world, double id);

/**
 * 提交一个活跃信号（发射）。
 *
 * 参数布局（f64，按顺序）：
 * `id, tx_node_id, birth_ms, ox, oy, oz, ovx, ovy, ovz, odx, ody, odz,
 *  frequency, energy, tx_gain, beam_angle, owner_body`
 */
uint8_t cosmos_world_radio_submit_signal(struct CosmosWorld *world, const double *values);

/**
 * 取走本轮传播结果：`out` 指向 `capacity * 4` 个 f64 缓冲，
 * 每条 = `signal_id, rx_node_id, received_power, received_frequency`（f64 位型存 id）。
 * 返回实际条数。
 */
uint32_t cosmos_world_radio_take_results(struct CosmosWorld *world, double *out, uint32_t capacity);

/**
 * 显式推进一轮无线电传播（节点/信号提交后调用；读取最新天体位置）。
 * 返回 1 成功 / 0 world 无效或未启用 radio。
 */
uint8_t cosmos_world_radio_step(struct CosmosWorld *world);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* COSMOS_H */
