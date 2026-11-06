# 模块镜像

## 主张

两条，都由 `crates/mps-test/src/rapier/verify_module_mirror.rs` 钉住：

1. **文件名镜像。** `mps-test/src/rapier` 的文件集合等于 `mps-core/src/rapier` 与 `mps-formula/src` 的并集（去掉 `mod.rs` / `lib.rs`，加上测试专用适配文件）。`mps-test/src/cosmos` 镜像 `mps-cosmos/src`（去掉 `lib.rs` 与 `ffi.rs`）。源是目录而测试仍是单文件时，守门允许过渡但会提示；测试是目录而源还是单文件则失败。
2. **可达性。** `mps-test/src` 下每个 `.rs` 都必须被某处的 `mod <name>;` 声明。只存在文件、没有被 `mod` 挂上的模块不会被编译，其中的 `#[test]` 永远不会跑，但仍然会被计数进文档站的 `TEST_COUNT`。

第 2 条是补上的：在此之前的守门只比文件名，于是 `rapier/compat.rs`（16 个测试）、`cosmos/flight/`（13 个）、`rapier/disciplines/`、`rapier/scientists/`、`rapier/rotor/` 都整块没被编译，却一直通过镜像检查。文件名以 `_` 开头视为临时脚本，两条都不管。

## 在哪

`crates/mps-test/src/rapier/verify_module_mirror.rs`

比较的是目录清单与 `mod` 声明文本，不解析 `#[test]` 正文。

## 怎么复现

```powershell
cargo test -p mps-test verify_module_mirror
```

新增子模块却没有对应测试文件时失败；写了测试文件却没在 `lib.rs`（或所属 `mod.rs`）里 `mod` 声明时也失败。补声明或补文件，而不是把名字加进忽略列表，除非那个文件本来就是测试专用适配（`test_only_extras` 在源码里）。
