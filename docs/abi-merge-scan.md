# `dedicated` 入口的可合并性扫描

**目的**：ADR-0016 的判据说 `dedicated` 里还有 150~200 个可合并。这份文档把这个
估计变成确切数字，并说明为什么实际数字比预期低得多。

**方法**：从生成的 `crates/mps-core/include/rigid_body.h` 解析全部 917 个声明
（头文件是权威且格式规整，源码多行签名难以稳定解析），然后按
「实体句柄 + 载荷形状」两级分类。

## 第一层：区分 setter 与公式函数

这一步推翻了我的初始假设。按「最后一个参数是 `double`」筛出的 89 个里，绝大多数
**根本不是 setter**：

```c
astrophysics_transit_depth(double planet_radius, double star_radius, double *out);
material_mechanics_hookes_law_uniaxial(double stress, double youngs_modulus, double *out);
electromagnetism_intrinsic_impedance(double permeability, double permittivity, double *out);
```

没有 `world`、没有实体句柄、没有 `param` —— 这些是**纯公式函数**，属于
`mps-formula` 层的数学实现。它们不该走属性通道，也不该算进 ABI 收敛的账。

正确的判据是：**第 1 个参数是 `WorldHandle`/`AppHandle`，第 2 个是实体句柄或索引。**

| 类别 | 数量 |
| --- | ---: |
| 实体属性 setter（未覆盖） | **200** |
| 纯公式函数（带 `*out`，无实体） | 257 |
| 已进通道 | 36 |

## 第二层：200 个 setter 按载荷形状

| 形状 | 数量 | 能否进现有通道 |
| --- | ---: | --- |
| A. 无载荷（`clear` 型） | 27 | ✅ `clear_param` 同族 |
| B. 单个 `f64` | 23 | ✅ `set_param` |
| C. 单个 `u32` | 9 | ✅ `set_param` |
| D. 单个 `Bool` | 10 | ✅ `set_param` |
| E. `Vec3` | 16 | ✅ `set_vec3_param` |
| F. `Quat` | 2 | ✅ `set_quat_param` |
| G. `index + f64` | 10 | ✅ `set_indexed_param` |
| H. `2 × f64` | 7 | ❌ 宽度级限制 |
| I. `3 × f64`（非 index） | 1 | ❌ 宽度级限制 |
| J. `index + 3 × f64` | 5 | ❌ 需 `indexed_vec3` 通道 |
| K. `InteractionGroupsDesc` | 4 | ✅ `set_u32x2_param` |
| **L. 含指针** | **33** | ❌ 生命周期/缓冲区，必须专用 |
| M. 其他混合签名 | 53 | ❌ 见下 |

### 小结

**直接能进现有六个通道的：约 101 个**（A 27 + B 23 + C 9 + D 10 + E 16 + F 2 +
G 10 + K 4）。

比 ADR 里估的 150~200 低，原因是两条：

1. **公式函数被误计入**。如果只看「末位是 `double`」会得到虚高的数字。
2. **含指针的 33 个必须专用**。它们是缓冲区操作（`set_vertex_weights`、
   `apply_surfaces`、`*_out` 类），载荷是调用方内存，天然不适合值语义通道。

## M 类（53 个混合签名）明细

这些是真正"形状奇特"的一组，按签名聚合：

| 签名 | 数量 | 说明 |
| --- | ---: | --- |
| `Vec3 + Bool` | 14 | 向量 + 开关，可考虑 `vec3` 通道加 flag 变体 |
| `Vec3 + Vec3 + Bool` | 5 | 两向量，需新通道 |
| `double + uint8_t` | 4 | 值 + enabled 标志（`set_tear_*` 一类） |
| `double + Bool` | 3 | 同上 |
| `ShapeDesc` | 3 | 形状描述，结构体载荷 |
| `Vec3 + Quat` | 3 | pose，已有 `set_pose` 专用 |
| `Vec3 + Quat + Bool` | 2 | pose + 开关 |
| `Quat + Bool` | 2 | 旋转 + 开关 |
| `u32 + u32` | 1 | 可进 `set_u32x2_param` 的候选 |
| `uint32_t + uint32_t + double` | 2 | 索引对 + 值 |
| 其余 | 12 | 各 1 个 |

`Vec3 + Bool` 有 14 个，是 M 类里最大的一组。给 `set_vec3_param` 加一个 `flag`
变体（或在载荷后加 `enabled`）能吸收它们，但那会让向量通道的语义变成"值 + 可选
开关"，值得单独设计。

## 判据的修正

ADR-0016 原来的判据是：

> 参数多于 2 个标量 → 不合并

**这条判据不够**。扫描后发现两个缺失条件：

1. **必须排除纯公式函数**（首参非 `WorldHandle`）。否则会把 257 个数学函数算进来，
   严重虚高。
2. **含指针的必须排除**（33 个）。指针载荷是调用方缓冲区，不能用值语义通道。

修正后的判据：

> 要合并一个入口，必须同时满足：
> 1. 第 1 参数是 `WorldHandle`/`AppHandle`，第 2 参数是实体句柄或索引；
> 2. 载荷不含指针；
> 3. 载荷形状落在已实现的通道里（标量 / 向量 / 四元数 / `2×u32`）；
> 4. 不是生命周期操作（`create`/`destroy`/`build`/`insert`/`remove`/`step`）。

## 结论

| 阶段 | 入口数 | 说明 |
| --- | ---: | --- |
| 现在（含新通道，旧入口未删） | 917 + 36 | 新增了 36 个通道入口 |
| 若把 101 个改走通道并删旧入口 | **816** | 净减 101 |
| 再加上 `indexed_vec3`（J 类 5 个） | 811 | 边际收益很小 |
| 再解决 `Vec3 + Bool`（14 个） | 797 | 需要一个"值 + 开关"设计 |

**从 917 减到约 800 是现实目标**，不是最初设想的 600。

900 这个量级里有 257 个是公式函数（本来就该留在 `mps-formula` 的 C ABI 上），
33 个是缓冲区操作，53 个是混合签名——这三类合计 343 个，占 37%，**天然不适合
属性通道**。

### 真正的收益在哪

不在总数下降，而在**新增能力的边际成本**。现在加一个标量属性要动 6 处；收敛后
只需在对应 `*_PARAM_*` 枚举上加一行 + 一个 match 分支 + 一个测试。这是
`O(能力数) → O(1)` 的差别，与入口总数是两回事。

## 复现方法

```text
# 1. 解析头文件（权威签名）
#    见本次扫描：把 \r\n 和连续空白折成单空格，再按
#    '(\w[\w\s\*]*?)\s+(\w+)\(([^;]*?)\)\s*;' 提取
# 2. 分出 setter：首参匹配 WorldHandle|AppHandle，次参匹配句柄/索引
# 3. 按载荷类型序列分组
```

扫描是一次性的，没有进 `xtask`。如果将来要常态化，可以加一个
`xtask abi-shapes` 子命令——但这需要先确认它不会和 `capability-map` 的
`MergeGroup` 分类器产生两套口径。
