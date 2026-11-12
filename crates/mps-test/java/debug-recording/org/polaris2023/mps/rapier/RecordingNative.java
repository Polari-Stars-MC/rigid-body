package org.polaris2023.mps.rapier;

import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;

/**
 * JNI smoke fixture — debug recording: start/capture/stop/export round trip.
 *
 * 录制环形缓冲全链路：start → 逐帧 capture → stop → export → Java 侧按
 * blob 布局解码（v1 magic + body 表）。这是录制格式契约的 Java 侧校验。
 */
public final class RecordingNative {
    private RecordingNative() {}

    public static native long worldCreate(double x, double y, double z);
    public static native void worldDestroy(long world);
    public static native void worldStep(long world, double dt);
    public static native long rigidBodyBuilderCreate(int status);
    public static native long rigidBodyBuilderBuild(long builder);
    public static native void rigidBodyBuilderSetTranslation(long builder, double x, double y, double z);
    public static native long worldInsertRigidBody(long world, long body);
    public static native int abiLastErrorCode();

    public static native int debugRecordingStart(long world, int maxFrames);
    public static native int debugRecordingCapture(long world);
    public static native int debugRecordingStop(long world);
    public static native int debugRecordingDrop(long world);
    public static native int debugRecordingExport(long world, long out, int outLen);
    public static native int debugRecordingFrame(long world, int frameIndex, long out, int outLen);
    public static native int debugRecordingEnableEvents(long world, int enable);

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }

    public static void main(String[] args) {
        System.load(args[0]);
        long world = worldCreate(0, -9.81, 0);
        check(world != 0, "world creation failed");
        try (Arena arena = Arena.ofConfined()) {
            long builder = rigidBodyBuilderCreate(0);
            rigidBodyBuilderSetTranslation(builder, 0, 10, 0);
            long body = worldInsertRigidBody(world, rigidBodyBuilderBuild(builder));
            check(body != 0, "body insert failed");

            // 录制 5 帧，环容量 3 → 保留 3、丢 2。
            check(debugRecordingStart(world, 3) == 0, "start ok (ERR_OK=0)");
            for (int i = 0; i < 5; i++) {
                worldStep(world, 1.0 / 60.0);
                debugRecordingCapture(world);
            }
            int retained = debugRecordingStop(world);
            check(retained == 3, "ring keeps last 3 frames, got " + retained);

            // 两段式导出。
            int slots = debugRecordingExport(world, 0, 0);
            check(slots > 0, "size query");
            MemorySegment blob = arena.allocate(8L * slots, 8);
            int written = debugRecordingExport(world, blob.address(), slots);
            check(written == slots, "export fills exactly");

            // 头部解码：frame_count=3、dropped=2、首帧 magic = MPSDR01。
            int frameCount = (int) blob.getDouble(0);
            int dropped = (int) blob.getDouble(1);
            check(frameCount == 3, "frame count");
            check(dropped == 2, "dropped count");
            long magic = Double.doubleToRawLongBits(blob.getDouble(2));
            check(magic == 0x4D50534452303131L, "v1 magic MPSDR01, got " + Long.toHexString(magic));
            int bodyCount = (int) blob.getDouble(3);
            check(bodyCount == 1, "one dynamic body");
            double dt = blob.getDouble(4);
            check(Math.abs(dt - 1.0 / 60.0) < 1e-12, "frame dt");

            // 逐帧读接口也能取到。
            MemorySegment frame = arena.allocate(8 * 32, 8);
            int frameSlots = debugRecordingFrame(world, 0, frame.address(), 32);
            check(frameSlots > 0, "frame read");
            check((int) frame.getDouble(0) == 1, "frame body count");

            check(debugRecordingDrop(world) == 0, "drop ok");
            check(abiLastErrorCode() == 0, "no error leaked");
            System.out.println("JNI debug-recording smoke test passed");
        } finally {
            worldDestroy(world);
        }
    }
}
