# 体级模拟域与碎裂闭环证据（B1 + B2）

> 采集日期：2026-11-12。复现命令见各节。

## 1. B1：冻结域步进开销（domain_freeze_step_cost_ratio）

> 复现：`cargo test -p mps-test --release --lib -- domain_freeze_step_cost --ignored --nocapture`

2000 体世界（40×50 球阵），1800 体划入 inactive 域，60 步步进耗时：

| 场景 | 60 步耗时 | 相对基线 |
| --- | --- | --- |
| 全活动基线（2000 体） | 0.1507 s | 1.000 |
| 冻结域（200 活动 + 1800 冻结） | 0.0148 s | **0.098** |

**ADR-0009 承诺（< 40% 基线）实测 9.8%**——收益随冻结体数量近线性
（200/2000 ≈ 10% 工作量 → 9.8% 耗时）。

## 2. B1：冻结语义守门（simulation_domain_freeze_and_thaw，非 ignore）

- 域 0 保留：`world_domain_body_count(world, 0)` → -1 + `ERR_NOT_FOUND(3)`；
  `world_set_domain_active(world, 0, false)` → 0 + `ERR_INVALID_ARGUMENT(2)`。
- inactive 域成员 10 步内位姿+线速度**逐位不变**（f64::to_bits 全量快照）。
- 恢复 active 后立即恢复积分；划回域 0 后同样恢复。
- 重复停用（已 active → inactive → inactive）返回 0（no-op）。
- `world_remove_rigid_body` 清理成员表：移除后 `world_domain_body_count` → 0。

## 3. B1：JNI 冒烟（RapierNative.java collision-mode 场景）

划入域 1 → 成员数 1 → 停用（翻转数 1）→ 恢复（翻转数 1）→
重复停用（0）→ 划回域 0 → 成员数 0。全部通过。

## 4. B2：接触力→自动碎裂闭环（contact_force_event_auto_fracture_loop，非 ignore）

> 复现：`cargo test -p mps-test --lib -- contact_force_event_auto --nocapture`

重球（r=0.5，v=-20 m/s）从 2.5 m 高处砸碎裂板（应力阈 1000，触发模式
StressIntensity）：

| 步骤 | 结果 |
| --- | --- |
| 冲击在 120 步内产生接触力事件 | ✓（实测 < 20 步） |
| `world_auto_fracture_from_contact_forces(scale=1.0)` | 峰值力过阈 → 板碎裂（走手动触发同一条替换路径） |
| 碎裂体 id 回写 out_ids | ids[0] == 板 id |
| 第二次调用 | 返回 0——事件队列已被第一次调用清空（单消费者语义） |

参数校验（`auto_fracture_rejects_bad_input`）：null world → `ERR_NULL_POINTER`；
负 scale → `ERR_INVALID_ARGUMENT`。

## 5. B2 与既有 impact_damage 路径的分工

| 路径 | 输入 | 语义 | 调用时机 |
| --- | --- | --- | --- |
| `accumulate_impact_damage`（既有） | solver 接触对**累计冲量** | 损伤累积 `impulse × scale`，过阈碎裂 | `world_step` 内每步自动 |
| `world_auto_fracture_from_contact_forces`（B2 新增） | 事件**峰值力** `max_force_magnitude × scale` | 应力强度代理，走 StressIntensity/Griffith 触发 | 宿主在 step 后显式调用一次 |

两者互补：累计冲量反映"打了多久"，峰值力反映"打得多狠"（更接近断裂
力学的应力强度语义）。
