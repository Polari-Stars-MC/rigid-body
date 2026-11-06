# 文档站计数

## 主张

`crates/mps-web/src/metrics.rs` 里的 **18 个常量**都等于在源码上重新数出来的值。

## 在哪

- 生成器：`crates/xtask/src/main.rs` 的 `dump-metrics`
- 生成结果：`crates/mps-web/src/metrics.rs`（文件头写明每个常量的计数口径）
- CI 守门：`.github/workflows/ci.yml` 的 `Check generated metrics are up to date`
- 仓内测试：`crates/mps-test/src/rapier/verify_metrics_sync.rs`

两层不互相替代：

| 层 | 覆盖范围 | 怎么判 |
| --- | --- | --- |
| CI | 全部 18 个常量 | 重新跑 `dump-metrics`，再 `git diff --exit-code`。比对对象就是生成器本身，所以不可能与生成器漂移 |
| `verify_metrics_sync` | 只重算 `TEST_COUNT` / `JNI_METHOD_COUNT` / `CORE_FFI_COUNT` | 用独立的扫描实现重算后与 `metrics.rs` 里的字面量比 |

`verify_metrics_sync` 的扫描逻辑与 xtask 是同一套口径的复制（都只认 `line.trim() == "#[test]"`、都只数行首的 `pub extern "C"`），所以它能发现「忘了重新生成」，但发现不了「生成器本身口径错了」。新增常量时必须同步更新 `metrics.rs` 的文件头注释。

## 怎么复现

```powershell
cargo run -p xtask -- dump-metrics
git diff --exit-code -- crates/mps-web/src/metrics.rs
cargo test -p mps-test verify_metrics
```

先重新生成再比对。手改 `metrics.rs` 会在下一次生成时被覆盖，CI 也会失败。
