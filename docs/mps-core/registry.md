# registry.rs — per-body 管理器的稳定 id 注册表

`crates/mps-core/src/rapier/registry.rs`。

铰接体、角色体、传感器区、车辆控制器、轮胎模型、伺服体、可碎复合刚体、毛发系统、绳结，这些模块原本各自写着同一段 `HashMap<u32, T>` + `_next_id: u32` 样板：创建时分配 id、取出时查表、删除时清表。`IdRegistry<T>` 把这段收成一份。

## 两条保证

- **id 单调递增，用 `wrapping_add` 分配。** 合并前各模块一半写 `+= 1`、一半写 `wrapping_add`——前者在 debug 构建下创建约 40 亿个体会 panic。现在一律回绕。
- **id 不复用。** 删掉一个体再建一个，新 id 比旧的大，不会撞上宿主还缓存的旧 id；`get` 返回 `Option`，删除后按旧 id 查一定拿到 `None`，宿主不需要第二套 generation 校验。

## 为什么单独成文件

九个模块共用同一个 id 分配语义，才能让「一个 id 只属于一个体家族」在 ABI 上说得清——C 侧的 id 是裸 `u32`，不带家族标签，靠的就是各家族自己的注册表永不互相借号。

## 边界

纯容器：`HashMap` + 计数器，不碰 Rapier、不碰 `WorldHandle`、不发 FFI、不进生成的 C 头文件。`pub` 而不是 `pub(crate)` 只是为了满足模块镜像规则让 `mps-test` 能测它（OPTIMIZATION.md §8）。

## 测试

`crates/mps-test/src/rapier/registry.rs`。
