# cosmos 共享竞技场布局

`crates/mps-cosmos/src/arena.rs` 里的常量是**跨语言 ABI**：Java 宿主拿 `DirectByteBuffer` 直接按这些偏移读写，Rust 侧只负责映射与校验。改任何一个偏移都等于改 ABI，`COSMOS_ARENA_VERSION` 必须一起动，`mps-test` 的 cosmos arena 往返测试和 Java fixture 都要重跑。

本页只钉两件事：**内存是怎么排的**，以及**谁在守它**。不是第二份设计说明。

## 为什么和 `mps-core` 不共用一套

`rigid_body.h` 和 `cosmos.h` 都导出 `ARENA_MAGIC` / `BODY_SLOT_STRIDE` / `CMD_SLOT_STRIDE`，但值不同（版本分别是 2 / 1，命令槽 32 / 40 字节）。这是刻意的：两个世界各自持有后端，arena 只描述各自世界的槽位。两个头文件同时 `#include` 时，版本宏靠改名避开重定义——`MPS_ARENA_VERSION` 与 `COSMOS_ARENA_VERSION`，见 [adr/0002-two-worlds.md](adr/0002-two-worlds.md) 与 [BOUNDARY.md](BOUNDARY.md) 的「句柄与内存」一节。

## 头（`HEADER_SIZE = 128` 字节）

| 偏移 | 字段 | 类型 | 谁写 |
| --- | --- | --- | --- |
| 0 | magic（`"COSMAREN"` = `0x434F534D_4152454E`） | `u64` | Rust（创建时） |
| 8 | arena version（= 1） | `u32` | Rust |
| 12 | flags | `u32` | Rust |
| 16 | `max_bodies` | `u32` | Rust（创建时定死） |
| 20 / 24 | `max_colliders` / `max_events` | `u32` | 占位，恒 0（cosmos 无碰撞体、无事件环） |
| 28 | `max_commands` | `u32` | Rust |
| 32 | `body_count` | `u32` | Rust（本步刷新的刚体数） |
| 36 / 40 | `collider_count` / `event_count` | `u32` | 占位，恒 0 |
| 44 | `cmd_write`（待消费命令数） | `u32` | Java（SPSC 写者） |
| 48 / 56 | `body_slot_stride` / `cmd_slot_stride` | `u32` | Rust（自描述，便于 Java 校验） |
| 64 | body handle map 偏移 | `u64` | Rust（内部，私有常量） |
| 96 | cmd ring 基址偏移 | `u64` | Rust（动态，取决于 `max_bodies`） |

`OFF_CMD_RING` 是**运行期才知道**的：命令环跟在刚体槽区后面，所以调用方必须先读它再算地址，不要自己拿 `HEADER_SIZE + max_bodies * BODY_SLOT_STRIDE` 推。

`magic` / `version` 是 cosmos 专属值，Java 侧在 map 时据此拒绝把 core arena 当 cosmos arena 用。

## 槽位

| 常量 | 值 | 含义 |
| --- | --- | --- |
| `BODY_SLOT_STRIDE` | 96 | 一个刚体槽的字节数；与 Java 侧逐字段镜像 |
| `CMD_SLOT_STRIDE` | 40 | 一条命令槽 = 5 × `u64`（`cmd_type, body_index, a0, a1, a2`） |
| `OFF_BODY_SLOT_BASE` | `HEADER_SIZE` | 刚体槽区起点 |

`BODY_SLOT_STRIDE` 两个头文件都叫这个名字、都等于 96，但**槽内字段布局不同**。不要写一份解析代码同时喂两个世界。

刚体槽用 seqlock 风格的一致性协议：Rust 写前把 generation 加一成奇数，写回偶数；Java 读 `gen_before → data → gen_after`，两者相等且为偶数才算读到一致快照。载荷字以原子位模式存放，Java 侧必须用 Acquire/volatile 读，否则 `f64` 会被撕成两半。

## 上限

`MAX_ARENA_BODIES` / `MAX_ARENA_COMMANDS` 各 1,000,000，`MAX_ARENA_TOTAL_BYTES` = 256 MiB（也是 Java `ByteBuffer.capacity()` 2 GiB 天花板下的安全值）。这些是防御值，不是建议值：FFI 请求超过就直接拒绝，不要尝试分配。

## 守门在哪

- `crates/mps-test/src/cosmos/arena.rs` —— 往返测试：Rust 写、按 Java 的读法读回。
- `crates/mps-test/src/cosmos/ffi.rs` —— 布局常量与 `mps_cosmos::arena` 逐条对齐。
- `crates/mps-test/src/rapier/version_consistency.rs` 与 `arena_compat.rs` —— 版本常量与 `mps-ffm::ABI_VERSION` 的联动。**注意**：`arena_compat` 钉的是 `mps-core` 那一套常量，cosmos 侧没有对应的 pin 测试；改 `BODY_SLOT_STRIDE` / `CMD_SLOT_STRIDE` / `HEADER_SIZE` 时靠 cosmos 往返测试兜。
