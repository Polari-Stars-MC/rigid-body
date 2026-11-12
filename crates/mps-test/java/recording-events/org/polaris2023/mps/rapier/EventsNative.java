package org.polaris2023.mps.rapier;

import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.util.ArrayList;
import java.util.List;

/**
 * JNI smoke fixture — recording v2 event sections: Java-side consumption.
 *
 * 录制开启事件段后逐帧解码碰撞/接触力事件——MC 集成时这是音效/粒子触发
 * 和碰撞伤害计算的输入源。与 {@link RecordingNative} 互补：那里验证 blob
 * 头部与 v1 兼容，这里验证事件段语义。
 */
public final class EventsNative {
    private EventsNative() {}

    public static native long worldCreate(double x, double y, double z);
    public static native void worldDestroy(long world);
    public static native void worldStep(long world, double dt);
    public static native long rigidBodyBuilderCreate(int status);
    public static native long rigidBodyBuilderBuild(long builder);
    public static native void rigidBodyBuilderSetTranslation(long builder, double x, double y, double z);
    public static native long worldInsertRigidBody(long world, long body);
    public static native int worldApplyRuntimeSettings(long world, int iterations, int substeps,
            int collision, int contact, int ccd, int sleeping);
    public static native int abiLastErrorCode();

    public static native int debugRecordingStart(long world, int maxFrames);
    public static native int debugRecordingCapture(long world);
    public static native int debugRecordingStop(long world);
    public static native int debugRecordingDrop(long world);
    public static native int debugRecordingExport(long world, long out, int outLen);
    public static native int debugRecordingEnableEvents(long world, int enable);

    /** 事件种类（与 Rust EVENT_* 常量一一对应）。 */
    public static final int EVENT_COLLISION_STARTED = 1;
    public static final int EVENT_COLLISION_STOPPED = 2;
    public static final int EVENT_CONTACT_FORCE = 3;

    /** 解码后的单条事件。 */
    record PhysicsEvent(int kind, long colliderA, long colliderB, double force, double maxForce) {}

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }

    /** 整段 blob 解码：逐帧收集事件（v1 帧无事件段则跳过）。 */
    static List<PhysicsEvent> decodeEvents(MemorySegment blob, long slots) {
        List<PhysicsEvent> events = new ArrayList<>();
        int frameCount = (int) blob.getDouble(0);
        int cursor = 2;
        final long MAGIC_V1 = 0x4D50534452303131L;
        final long MAGIC_V2 = 0x4D50534452303231L;
        for (int f = 0; f < frameCount; f++) {
            long magic = Double.doubleToRawLongBits(blob.getDouble(cursor));
            int bodies = (int) blob.getDouble(cursor + 1);
            cursor += 3;
            for (int b = 0; b < bodies; b++) cursor += 1 + 13; // handle + 13 lanes
            if (magic == MAGIC_V2) {
                int eventCount = (int) blob.getDouble(cursor++);
                for (int e = 0; e < eventCount; e++) {
                    int kind = (int) blob.getDouble(cursor);
                    long a = Double.doubleToRawLongBits(blob.getDouble(cursor + 1));
                    long bH = Double.doubleToRawLongBits(blob.getDouble(cursor + 2));
                    double force = blob.getDouble(cursor + 3);
                    double maxForce = blob.getDouble(cursor + 4);
                    events.add(new PhysicsEvent(kind, a, bH, force, maxForce));
                    cursor += 5;
                }
            } else {
                check(magic == MAGIC_V1, "unknown magic " + Long.toHexString(magic));
            }
        }
        return events;
    }

    public static void main(String[] args) {
        System.load(args[0]);
        long world = worldCreate(0, -9.81, 0);
        check(world != 0, "world creation failed");
        try (Arena arena = Arena.ofConfined()) {
            // 开碰撞 + 接触力事件（否则 world 不产事件）。
            check(worldApplyRuntimeSettings(world, 4, 1, 1, 1, 0, 1), "runtime settings");
            // 落体 + 地面：数步内必碰撞。
            long builder = rigidBodyBuilderCreate(0);
            rigidBodyBuilderSetTranslation(builder, 0, 1.5, 0);
            long body = worldInsertRigidBody(world, rigidBodyBuilderBuild(builder));
            check(body != 0, "body insert");

            check(debugRecordingStart(world, 128) == 0, "start");
            check(debugRecordingEnableEvents(world, 1) == 0, "enable events");
            for (int i = 0; i < 30; i++) {
                worldStep(world, 1.0 / 60.0);
                debugRecordingCapture(world);
            }
            debugRecordingStop(world);

            int slots = debugRecordingExport(world, 0, 0);
            MemorySegment blob = arena.allocate(8L * slots, 8);
            check(debugRecordingExport(world, blob.address(), slots) == slots, "export");

            List<PhysicsEvent> events = decodeEvents(blob, slots);
            check(!events.isEmpty(), "v2 event section produced events");

            // 必有 started；started 与 stopped 数量关系：每对碰撞先 started。
            long started = events.stream().filter(e -> e.kind() == EVENT_COLLISION_STARTED).count();
            long stopped = events.stream().filter(e -> e.kind() == EVENT_COLLISION_STOPPED).count();
            check(started >= 1, "at least one collision started");
            check(events.stream().allMatch(e -> e.kind() == EVENT_COLLISION_STARTED
                    || e.kind() == EVENT_COLLISION_STOPPED
                    || e.kind() == EVENT_CONTACT_FORCE), "only known kinds");

            // 接触力事件（如有）力值有限且非负。
            for (PhysicsEvent e : events) {
                if (e.kind() == EVENT_CONTACT_FORCE) {
                    check(Double.isFinite(e.force()) && e.force() >= 0.0, "finite force");
                }
            }

            // MC 集成示例：started → 音效/粒子；contact-force → 伤害输入。
            for (PhysicsEvent e : events) {
                switch (e.kind()) {
                    case EVENT_COLLISION_STARTED -> onImpact(e.colliderA(), e.colliderB());
                    case EVENT_CONTACT_FORCE -> onContactForce(e.colliderA(), e.colliderB(), e.force());
                    default -> {}
                }
            }

            check(debugRecordingDrop(world) == 0, "drop");
            check(abiLastErrorCode() == 0, "no error leaked");
            System.out.println("JNI recording-events consumption smoke test passed (" + events.size() + " events, "
                    + started + " started, " + stopped + " stopped)");
        } finally {
            worldDestroy(world);
        }
    }

    // MC 集成点（此处只打印；真实宿主接音效/粒子/伤害）。
    static void onImpact(long colliderA, long colliderB) { /* sound + particles */ }
    static void onContactForce(long colliderA, long colliderB, double force) { /* damage = f(force) */ }
}
