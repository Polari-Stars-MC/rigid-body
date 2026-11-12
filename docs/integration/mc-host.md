# MC 宿主接入清单（mps_rigid_body + mps_cosmos）

本仓库 2026.11.12 之后新增的 Java 面 API 的完整接线步骤。四个 Java 冒烟
fixture（`crates/mps-test/java/`）是逐条可运行的参考实现——**先读 fixture
再抄声明**，它们在 CI `jni-smoke` job 里持续验证符号与语义。

## 前置：ABI 版本

- `EXPECTED_ABI_VERSION` 必须 **2**（`bdc54d2` 起）。加载时
  `abiVersion()` ≠ 2 拒绝启动并给出升级提示。
- 库名不变：`System.load(.../mps_rigid_body.dll)`（Windows）/
  `libmps_rigid_body.so`（Linux）。类名 FQN 的 `_1` 转义只影响
  `Java_org_polaris2023_mps_*` 符号生成，Java 侧无需感知。

## 1. Floating origin（cosmos 世界）

### 声明

```java
// 手动 recenter：以 (px,py,pz) 为新原点；offset 收 3×f64 平移量。
public static native int cosmosWorldRecenter(long world, double px, double py, double pz, long outOffset);
// 自动 recenter：pivot 距原点 > threshold 米才触发；返回 1 = 已平移。
public static native int cosmosWorldAutoRecenter(long world, long pivot, double threshold, long outOffset);
```

### 接线（服务器 tick 内，一行）

```java
// 每帧 cosmosWorldStep 之后调用；threshold 建议 1e7（f64 精度边界）。
int fired = SpaceNative.cosmosWorldAutoRecenter(world, shipHandle, 1.0e7, offsetPtr);
if (fired == 1) {
    // offset = 本次平移量：同步离线玩家坐标 / 重绘星图 / 记日志。
    // 所有 Java 侧缓存的绝对坐标也要 += offset（或改存相对坐标）。
}
```

### 语义要点

- 未触发（返回 0）不是错误，不写错误槽；`threshold <= 0 / NaN` 才写
  `ERR_INVALID_ARGUMENT`。
- pivot 无效（句柄不存在）是静默 no-op。
- 必须在两次 `cosmosWorldStep` 之间调用，不能在 step 回调里。

## 2. Debug recording（回放录制）

### 声明

```java
public static native int debugRecordingStart(long world, int maxFrames);   // 环容量帧数
public static native int debugRecordingCapture(long world);                // 每帧 step 后；未录制时静默 no-op
public static native int debugRecordingStop(long world);                   // 返回保留帧数
public static native int debugRecordingDrop(long world);                   // 释放
public static native int debugRecordingExport(long world, long out, int outLen); // 两段式：out=0 查槽数
public static native int debugRecordingFrame(long world, int frameIndex, long out, int outLen); // 逐帧读
public static native int debugRecordingEnableEvents(long world, int enable); // v2 事件段开关
```

### 接线（F3 录制开关示例）

```java
// 开录：热路径可无条件 capture（未录制时是 no-op）。
debugRecordingStart(world, 600);          // 10 秒 @60Hz
debugRecordingEnableEvents(world, 1);     // 碰撞/接触力进帧尾（v2）

// 服务器 tick：
worldStep(world, dt);
debugRecordingCapture(world);

// 停止 + 导出存盘：
int retained = debugRecordingStop(world);
int slots = debugRecordingExport(world, 0, 0);
long blob = Unsafe.allocateMemory(8L * slots);
debugRecordingExport(world, blob, slots);
writeToFile(blob, slots);                 // 离线轨迹查看器直接吃这个文件
debugRecordingDrop(world);
```

### blob 格式（解码见 `RecordingNative.java` + web 页 Rust 参考实现）

- 头部 `[frame_count, dropped]`，之后逐帧：
  `magic(MPSDR01/02) | body_count | dt | 逐体 (packed_handle + 13 lanes)`
- v2 帧尾：`event_count | 逐事件 (kind, colliderA, colliderB, force, maxForce)`
  kind：1=碰撞开始、2=碰撞结束、3=接触力。旧查看器按 magic 停在 body 表尾，互不误读。
- handle 打包 = `(generation<<32 | index) + 1`。

## 3. Compound baker 缓存（voxel chunk）

### 声明

```java
// 两段式：outBoxes=0 查框数，再分配缓冲取框（每框 6 f64，min/max 世界坐标）。
public static native int voxelBakeBoxes(long voxels, int sizeX, int sizeY, int sizeZ,
        double voxelSizeX, double voxelSizeY, double voxelSizeZ,
        double originX, double originY, double originZ,
        long outBoxes, int outCount);
public static native long colliderBuilderCreateCompoundBoxes(long boxData, int boxCount);
public static native void colliderBuilderDestroy(long builder);
```

### 接线（chunk 加载/重载）

```java
double[] cached = bakeCache.get(chunkKey);
if (cached == null) {                     // 首次加载：烘一次
    int n = voxelBakeBoxes(voxelPtr, sx, sy, sz, 1,1,1, ox,oy,oz, 0, 0);
    long buf = Unsafe.allocateMemory(8L * n * 6);
    voxelBakeBoxes(voxelPtr, sx, sy, sz, 1,1,1, ox,oy,oz, buf, n);
    cached = readDoubles(buf, n * 6);
    bakeCache.put(chunkKey, cached);      // 确定性输出，缓存即有效
}
long builder = colliderBuilderCreateCompoundBoxes(pointerTo(cached), cached.length / 6);
// insert collider…；chunk 卸载时 colliderBuilderDestroy(builder)
// block 更新 → bakeCache.remove(chunkKey) → 下次加载重烘。
```

### 生产建议

- 缓存上限 + LRU（参考 `BakerNative.VoxelBakeCache`，加容量上限即可）。
- 失效键：chunk 坐标 + 修改计数（防同 key 旧缓存）。
- 烘焙是纯函数且确定性（fixture 有逐位断言），缓存命中零风险。

## 4. CI 验证

宿主侧声明一旦与 fixture 漂移（签名/返回类型不符），`jni-smoke` job 会先红
——**fixture 是契约**，不要在本仓库外另起一份没有验证的声明。

## 5. 离线回放工具

`crates/mps-test/java/replay-viewer/ReplayViewer.java` 是独立 Swing 小工具
（零仓库内依赖）：`java ReplayViewer blob.mpsdr` 加载导出 blob，渲染 XZ
俯视轨迹 + 事件统计。与 web 文档页的 Rust 解码器同一份布局契约；录制
导出后直接可视化验证录制是否完整。
