package org.polaris2023.mps.rapier;

import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * JNI smoke fixture — compound baker + Java-side chunk cache integration.
 *
 * 烘一次 → 缓存 → 从缓存重建 collider（chunk 重载路径）：这是
 * voxelBakeBoxes + colliderBuilderCreateCompoundBoxes 的预期接法。
 * MC 集成时把 {@code VoxelBakeCache} 挂在 chunk 存储上，键 = chunk 坐标。
 */
public final class BakerNative {
    private BakerNative() {}

    public static native int voxelBakeBoxes(long voxels, int sizeX, int sizeY, int sizeZ,
            double voxelSizeX, double voxelSizeY, double voxelSizeZ,
            double originX, double originY, double originZ,
            long outBoxes, int outCount);
    public static native long colliderBuilderCreateCompoundBoxes(long boxData, int boxCount);
    public static native void colliderBuilderDestroy(long builder);
    public static native int abiLastErrorCode();

    /**
     * Java 侧 chunk 缓存：键 = chunk 坐标（此处用打包 int 演示），值 =
     * 烘好的框列表（min/max ×6/框）。烘焙是纯函数 + 确定性输出，缓存键
     * 命中即零成本重建；失效策略由宿主（dirty 标记 / 修改计数）决定。
     */
    static final class VoxelBakeCache {
        final Map<Long, double[]> boxes = new ConcurrentHashMap<>();

        /** 查缓存；miss 返回 null（调用方烘一次后 put）。 */
        double[] get(long chunkKey) { return boxes.get(chunkKey); }

        void put(long chunkKey, double[] baked) { boxes.put(chunkKey, baked); }

        /** 从缓存框列表重建 compound collider builder（0 = 失败，查 abiLastErrorCode）。 */
        long rebuildBuilder(double[] baked, Arena arena) {
            MemorySegment seg = arena.allocate(8L * baked.length, 8);
            for (int i = 0; i < baked.length; i++) seg.setDouble(8L * i, baked[i]);
            return colliderBuilderCreateCompoundBoxes(seg.address(), baked.length / 6);
        }
    }

    /**
     * 生产级 chunk 烘焙缓存：LRU 上限 + 修改计数失效。
     *
     * - 失效：put 时带 `revision`（宿主的 chunk 修改计数）；get 校验
     *   revision 不匹配即视为 miss（block 更新 → revision++ → 自动重烘）。
     * - LRU：访问顺由 {@code LinkedHashMap.accessOrder=true} 维护；
     *   超上限逐出最久未用 chunk，防长距离探索内存膨胀。
     * - 线程：{@code synchronized}——烘焙/重建本身是重活，锁开销可忽略。
     */
    static final class VoxelBakeCache2 {
        private final int maxEntries;
        private final LinkedHashMap<Long, Entry> lru;

        record Entry(double[] boxes, long revision) {}

        VoxelBakeCache2(int maxEntries) {
            this.maxEntries = maxEntries;
            this.lru = new LinkedHashMap<>(16, 0.75f, true) {
                @Override protected boolean removeEldestEntry(Map.Entry<Long, Entry> e) {
                    return size() > maxEntries;
                }
            };
        }

        /** 命中返回框列表；miss 或 revision 过期返回 null。 */
        synchronized double[] get(long chunkKey, long revision) {
            Entry e = lru.get(chunkKey);
            return (e != null && e.revision() == revision) ? e.boxes() : null;
        }

        synchronized void put(long chunkKey, long revision, double[] baked) {
            lru.put(chunkKey, new Entry(baked, revision));
        }

        synchronized int size() { return lru.size(); }
    }

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }

    public static void main(String[] args) {
        System.load(args[0]);
        try (Arena arena = Arena.ofConfined()) {
            // 4×2×4 体素：中间一层 8 个实心（两个 2×1×4 条会被贪心合并）。
            int sx = 4, sy = 2, sz = 4;
            byte[] grid = new byte[sx * sy * sz];
            for (int x = 0; x < sx; x++)
                for (int z = 0; z < sz; z++)
                    grid[(y(1) + x + z * sx)] = 1;
            long chunkKey = 0x0001_0002L; // (chunkX=1, chunkZ=2) 打包演示
            VoxelBakeCache cache = new VoxelBakeCache();

            double[] cached = cache.get(chunkKey);
            check(cached == null, "cache miss on first load");

            // 第一次烘焙（chunk 加载路径）。
            MemorySegment voxels = arena.allocate(grid.length, 1);
            for (int i = 0; i < grid.length; i++) voxels.set(i, grid[i]);
            int count = voxelBakeBoxes(voxels.address(), sx, sy, sz, 1, 1, 1, 0, 0, 0, 0, 0);
            check(count > 0 && count < sx * sy * sz, "baked into fewer boxes than cells: " + count);

            MemorySegment baked = arena.allocate(8L * count * 6, 8);
            int written = voxelBakeBoxes(voxels.address(), sx, sy, sz, 1, 1, 1, 0, 0, 0, baked.address(), count);
            check(written == count, "bake fill");

            double[] boxes = new double[count * 6];
            for (int i = 0; i < boxes.length; i++) boxes[i] = baked.getDouble(8L * i);

            // 确定性：重烘逐位一致（缓存正确性前提）。
            MemorySegment baked2 = arena.allocate(8L * count * 6, 8);
            check(voxelBakeBoxes(voxels.address(), sx, sy, sz, 1, 1, 1, 0, 0, 0, baked2.address(), count) == count, "rebake");
            for (int i = 0; i < boxes.length; i++)
                check(Double.doubleToRawLongBits(boxes[i]) == Double.doubleToRawLongBits(baked2.getDouble(8L * i)),
                        "bake is deterministic");

            cache.put(chunkKey, boxes);

            // chunk 重载路径：零体素上传，直接从缓存重建 builder。
            long builder = cache.rebuildBuilder(cache.get(chunkKey), arena);
            check(builder != 0, "compound rebuild from cache");
            colliderBuilderDestroy(builder);

            // 缓存命中后不再调 voxelBakeBoxes —— 零重烘、零体素重上传。
            check(cache.get(chunkKey) == boxes, "cache hit returns the same array");
            check(abiLastErrorCode() == 0, "no error leaked");

            // ── VoxelBakeCache2：revision 失效 + LRU 逐出 ──
            VoxelBakeCache2 cache2 = new VoxelBakeCache2(2);
            double[] fakeA = {0, 0, 0, 1, 1, 1};
            double[] fakeB = {1, 0, 0, 2, 1, 1};
            double[] fakeC = {2, 0, 0, 3, 1, 1};
            cache2.put(100, 1, fakeA);
            cache2.put(200, 1, fakeB);
            check(cache2.get(100, 1) == fakeA, "hit at revision 1");

            // block 更新 → revision++ → 旧缓存视为 miss（触发重烘）。
            check(cache2.get(100, 2) == null, "revision bump invalidates");
            cache2.put(100, 2, fakeC);
            check(cache2.get(100, 2) == fakeC, "re-baked entry cached");

            // LRU：容量 2，访问 100 → 200 变最旧；放入 300 逐出 200。
            check(cache2.get(200, 1) == fakeB, "200 present before eviction");
            cache2.get(100, 2);          // touch 100
            cache2.put(300, 1, fakeA);   // 逐出 200
            check(cache2.size() == 2, "LRU cap");
            check(cache2.get(200, 1) == null, "200 evicted (least recently used)");
            check(cache2.get(100, 2) == fakeC, "100 retained (recently touched)");
            check(cache2.get(300, 1) == fakeA, "300 present");

            System.out.println("JNI compound-baker cache smoke test passed");
        }
    }

    /** 体素数组内 y 层偏移（与 Rust 的 x + z*sx + y*sx*sz 布局一致）。 */
    private static int y(int layer) { return layer * 4 * 4; }
}
