package org.polaris2023.mps.cosmos;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.Arena;

/**
 * JNI smoke fixture — cosmos world: auto-recenter + floating origin.
 *
 * 与 {@code org.polaris2023.mps.rapier.RapierNative} 同款独立冒烟：
 * {@code System.load} 加载 mps_rigid_body 后依次断言 cosmos 面 API。
 * 编译：javac（独立于应用类路径）；运行：java CosmosNative /path/to/lib。
 */
public final class CosmosNative {
    private CosmosNative() {}

    // ── Cosmos world lifecycle ──────────────────────────────────────────
    public static native long cosmosWorldCreate(double dt, int solverIterations, int ccdSubsteps,
            int orbitIntegration, int verletSubsteps, double nBodySofteningSq);
    public static native void cosmosWorldDestroy(long world);
    public static native int cosmosWorldStep(long world, double dt);
    public static native long cosmosSatelliteBuilder(double mass, double px, double py, double pz,
            double vx, double vy, double vz, double radius);
    public static native void cosmosBuilderDestroy(long builder);
    public static native long cosmosWorldInsertBody(long world, long builder);
    public static native int cosmosBodyTranslationOut(long world, long body, long outTranslation);

    // ── Floating origin（本轮新增）─────────────────────────────────────
    /** 手动 recenter：以 (px,py,pz) 为新原点，offset 收 3×f64 平移量，返回 1/0。 */
    public static native int cosmosWorldRecenter(long world, double px, double py, double pz, long outOffset);
    /** 自动 recenter：pivot 距原点超 threshold 米才触发。返回 1 = 已平移。 */
    public static native int cosmosWorldAutoRecenter(long world, long pivot, double threshold, long outOffset);

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }

    public static void main(String[] args) {
        System.load(args[0]);
        long world = cosmosWorldCreate(1.0, 4, 0, 0, 1, 0.0);
        check(world != 0, "cosmos world creation failed");
        try (Arena arena = Arena.ofConfined()) {
            MemorySegment offset = arena.allocate(8 * 3, 8);
            MemorySegment translation = arena.allocate(8 * 3, 8);

            long builder = cosmosSatelliteBuilder(1000.0, 5.0e6, 0, 0, 0, 0, 0, 0.1);
            check(builder != 0, "builder failed");
            long ship = cosmosWorldInsertBody(world, builder);
            check(ship != 0, "body insert failed");

            // 位置读回（sanity）
            check(cosmosBodyTranslationOut(world, ship, translation.address()) == 1, "translation read");
            check(translation.getDouble(0) == 5.0e6, "initial x");

            // auto-recenter：threshold 1e7 > 距离 5e6 → 不触发。
            check(cosmosWorldAutoRecenter(world, ship, 1.0e7, offset.address()) == 0, "below threshold must not fire");
            // threshold 4e6 ≤ 距离 5e6 → 触发，offset = (-5e6, 0, 0)。
            check(cosmosWorldAutoRecenter(world, ship, 4.0e6, offset.address()) == 1, "above threshold must fire");
            check(offset.getDouble(0) == -5.0e6 && offset.getDouble(1) == 0.0 && offset.getDouble(2) == 0.0,
                    "offset = -pivot");
            check(cosmosBodyTranslationOut(world, ship, translation.address()) == 1, "post-recenter read");
            check(translation.getDouble(0) == 0.0, "pivot at origin");

            // 非法 threshold：不触发（0 返回），世界不动。
            check(cosmosWorldAutoRecenter(world, ship, 0.0, offset.address()) == 0, "zero threshold rejected");

            // 手动 recenter 仍可用：以任意点重定位。
            check(cosmosWorldRecenter(world, 10.0, 0, 0, offset.address()) == 1, "manual recenter");
            check(offset.getDouble(0) == -10.0, "manual offset");

            // 步进正常（recenter 后物理不被破坏）。
            check(cosmosWorldStep(world, 1.0) == 1, "step after recenter");

            System.out.println("JNI cosmos floating-origin smoke test passed");
        } finally {
            cosmosWorldDestroy(world);
        }
    }
}
