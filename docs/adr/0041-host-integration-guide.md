# 0041 — 宿主集成指南与三处事实修正

## 背景

前几轮全在做验证（六层防线、40 条 ADR、1647 项测试）。这一轮换个方向：**可用性**。

量化现状：

| 项 | 数量 |
| --- | ---: |
| 对外 C ABI 入口 | **1045**（`mps-core` 1003 + `mps-cosmos` 42） |
| 宿主视角的「从哪开始」指南 | **0** |

已有的两份相关文档各有专属范围：

* `docs/integration/mc-host.md`（75 行）—— 只讲 2026.11.12 之后新增的四个功能
  （浮动原点、录制回放、体素烘焙、CI 契约），是**功能清单**不是入口指南
* `docs/tyndall-mc-integration.md`（192 行，本轮写的）—— 只讲丁达尔一个子系统

一个 Java 开发者面对 1045 个入口，没有"我该先调哪几个"的答案。

## 决定

新增 `docs/host-integration.md`。九节：

1. 加载哪个库
2. 五分钟路径（完整可用代码）
3. 句柄的两种形态（不透明指针 vs packed `u64`）
4. 错误模型（两条通道不混用）
5. 三个必须知道的约定
6. **入口地图**：前缀分布 + 「90% 场景只需这 20 个」
7. 常见陷阱（都是实测踩过的）
8. 怎么验证「我这次调用算对了」
9. 相关文档

## 写的过程中发现三处事实错误

### 1. 库名写错了（`mc-host.md` 的真 bug）

`docs/integration/mc-host.md` 原文：

> 库名不变：`System.load(.../mps_rigid_body.dll)`（Windows）/
> `libmps_rigid_body.so`（Linux）。

**实际产物是 `mps.dll`。** 依据：

```toml
# crates/mps-jni/Cargo.toml
[lib]
name = "mps"
crate-type = ["cdylib", "rlib"]
```

构建产物实测也只有 `mps.dll` / `mps_core.dll` / `mps_ffm.dll` —— 仓库里
**不存在** `mps_rigid_body.dll`。

`mps_rigid_body` 只出现在两处：

* **Java 包名** `org.polaris2023.mps_rigid_body.ffi`
* **JNI 符号名** `Java_org_polaris2023_mps_rigid_body_*`

照抄旧文档会直接 `UnsatisfiedLinkError`。已在 `mc-host.md` 就地修正，并注明
修正日期与原因。

### 2. 缓冲区返回值**有两个约定**（不是我原先以为的一个）

我第一版指南写的是「缓冲区输出返回**总数**，不是拷贝数」。查证之后发现**这是错的**，
至少对一半入口不成立：

| 家族 | 数量 | 返回 | 证据 |
| --- | ---: | --- | --- |
| `*_read_*`（`soft_body_read_*`、`granular_read_particles`） | 11 | **总数**（可能 > capacity） | `granular_read_particles` 返回 `g.len()` |
| `query_intersect_*` / `world_batch_*` / `world_merge_*` | 39 | **已写入数**（≤ capacity） | `intersect_bound` 满了 `break`，返回 `written` |

**把两者混用会静默出错**：`_read_*` 的返回值当写入数会误判缓冲区不足；
`query_*` 的返回值当总数会漏掉被截断的结果。

指南改成了明确的两行表格 + 判断方法（看有没有 `_read_` 中缀）。

这本身就是一条值得记的 ABI 不一致 —— 50 个长得很像的入口，两种语义。

### 3. `world_step_n` 不存在

我原稿在「90% 只需这 20 个」表里写了 `world_step`（批量：`world_step_n`）。
查证：`world_step_n` 在 `rigid_body.h` 里**不存在**，只有 `cosmos_world_step_n`。
已删除。

## 两个我自己的弯路

### 弯路 1：漏了子目录，差点写重复文档

第一次扫描 `docs/` 用的是 `Get-ChildItem docs -Filter *.md` —— **非递归**，
于是漏掉了 `docs/integration/mc-host.md`。我据此得出「只有 1 份宿主指南」的结论，
写完新文档才发现索引里已经链着 `integration/mc-host.md`。

补扫之后确认：两份文档范围不重叠（一份是功能清单，一份是入口指南），所以保留，
但加了交叉引用。

**教训：统计「有没有 X」之前先 `-Recurse`。** 这个错误和上一轮 ADR-0040 里
「28 vs 42」是同一类 —— 都是**扫描方式决定结论**。

### 弯路 2：先把猜测当事实写进文档

缓冲区约定那条，我是按"已知惯例"写的，没有先验证。所幸核查惯例让我去查
`intersect_bound` 的实现，才发现是相反的。

**在文档里写"约定"比在代码里写更危险** —— 代码有测试兜底，文档没有。所以这一轮
指南里每一条约定都附了**依据**（函数名 + 返回的表达式），可复核。

## 后果

- 宿主有了「从哪开始」的答案
- `mc-host.md` 的加载语句从错的改成对的（避免 `UnsatisfiedLinkError`）
- 缓冲区双约定被写进文档（此前只在源码里，没人总结过）

## 这是文档改动，没有新增测试

`mps-test` 保持 **1647**。`docs_index_consistency` 守门通过（新文件已注册）。

**价值不在测试数，在可用性。** 1045 个入口的能力此前只能靠 grep 头文件发现。

## 没做的事

1. **`mps-cosmos` 的入口指南**：42 个入口只在本指南第 6 节末尾提了一句
   「是独立世界、不要混用句柄」，没有细分
2. **Java 侧 API 生成的自动校验**：`xtask gen-java` 只覆盖带
   `#[java_struct]` / `#[java_enum]` 的值类型，JNI 方法清单仍在仓库外
   （ROADMAP 第 3 条）
3. **指南的自动化校验**：没有测试检查「20 个核心入口」是否真的存在且签名未变

## 相关代码

- `docs/host-integration.md`（新增）
- `docs/integration/mc-host.md`（库名修正）
- `docs/README.md`（索引注册）

## 记录来源

与实现同批落地。这一轮的两处弯路（漏子目录、把猜测当事实）与上一轮的统计错误
是同一模式：**先下结论，再用扫描去确认，而扫描方式本身就有偏**。
