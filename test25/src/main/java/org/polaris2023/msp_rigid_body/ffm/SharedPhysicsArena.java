package org.polaris2023.msp_rigid_body.ffm;

import java.lang.foreign.*;
import java.lang.foreign.MemoryLayout.PathElement;

/**
 * Zero-JNI/FFM physics state access via shared memory arena.  Uses Java 25
 * {@code MemorySegment} for direct native memory access.</p>
 */
public final class SharedPhysicsArena {

    static final int HEADER_SIZE = 128;
    static final int BODY_SLOT_STRIDE = 96;
    static final int CMD_SLOT_STRIDE = 32;

    static final long OFF_BODY_COUNT = 32;
    static final long OFF_EVENT_COUNT = 40;

    static final int CMD_ADD_FORCE = 0;

    private final MemorySegment seg;
    private final int maxBodies;
    private final long bodySlotsStart;
    private final long cmdRingStart;
    private final long eventRingStart;

    public SharedPhysicsArena(MemorySegment segment) {
        this.seg = segment;
        long magic = seg.get(ValueLayout.JAVA_LONG_UNALIGNED, 0);
        if (magic != 0x4D50535F4152454EL) {
            throw new IllegalArgumentException("invalid arena magic: 0x" + Long.toHexString(magic));
        }
        this.maxBodies = seg.get(ValueLayout.JAVA_INT_UNALIGNED, 16);
        int maxCommands = seg.get(ValueLayout.JAVA_INT_UNALIGNED, 28);
        this.bodySlotsStart = HEADER_SIZE;
        this.cmdRingStart = bodySlotsStart + (long) maxBodies * BODY_SLOT_STRIDE;
        this.eventRingStart = cmdRingStart + (long) maxCommands * CMD_SLOT_STRIDE;
    }

    public int getMaxBodies()   { return maxBodies; }
    public int getBodyCount()   { return seg.get(ValueLayout.JAVA_INT_UNALIGNED, OFF_BODY_COUNT); }
    public int getEventCount()  { return seg.get(ValueLayout.JAVA_INT_UNALIGNED, OFF_EVENT_COUNT); }

    private long bodyAddr(int i) { return bodySlotsStart + (long) i * BODY_SLOT_STRIDE; }
    public double getBodyPX(int i)  { return seg.get(ValueLayout.JAVA_DOUBLE_UNALIGNED, bodyAddr(i) + 8); }
    public double getBodyPY(int i)  { return seg.get(ValueLayout.JAVA_DOUBLE_UNALIGNED, bodyAddr(i) + 16); }
    public double getBodyPZ(int i)  { return seg.get(ValueLayout.JAVA_DOUBLE_UNALIGNED, bodyAddr(i) + 24); }
    public double getBodyVX(int i)  { return seg.get(ValueLayout.JAVA_DOUBLE_UNALIGNED, bodyAddr(i) + 32); }
    public double getBodyVY(int i)  { return seg.get(ValueLayout.JAVA_DOUBLE_UNALIGNED, bodyAddr(i) + 40); }
    public double getBodyVZ(int i)  { return seg.get(ValueLayout.JAVA_DOUBLE_UNALIGNED, bodyAddr(i) + 48); }

    private long cmdAddr(int i) { return cmdRingStart + (long)i * CMD_SLOT_STRIDE; }
    public void cmdAddForce(int bodyIdx, double fx, double fy, double fz) {
        long a = cmdAddr(0);
        seg.set(ValueLayout.JAVA_INT_UNALIGNED, a, CMD_ADD_FORCE);
        seg.set(ValueLayout.JAVA_INT_UNALIGNED, a + 4, bodyIdx);
        seg.set(ValueLayout.JAVA_DOUBLE_UNALIGNED, a + 8, fx);
        seg.set(ValueLayout.JAVA_DOUBLE_UNALIGNED, a + 16, fy);
        seg.set(ValueLayout.JAVA_DOUBLE_UNALIGNED, a + 24, fz);
    }
}