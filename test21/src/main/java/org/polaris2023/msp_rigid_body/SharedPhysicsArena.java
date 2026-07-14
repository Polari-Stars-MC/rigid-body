package org.polaris2023.msp_rigid_body;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;

/**
 * Zero-JNI physics state access via shared memory arena.
 *
 * <p>No {@code sun.misc.*} or preview APIs needed — uses only
 * {@code java.nio.ByteBuffer} (Java 1.4+).  Compatible with all
 * JVM versions and mod loaders (Fabric, Forge, NeoForge).</p>
 *
 * <h3>Usage</h3>
 * <pre>{@code
 * // 1. Init
 * long world = RigidBodyNative.worldCreate(0, -9.81, 0);
 * long[] addrSize = new long[2];
 * RigidBodyNative.worldCreateSharedArena(world, 1024, 0, 4096, 4096, addrSize);
 * SharedPhysicsArena arena = SharedPhysicsArena.fromDirectBuffer(
 *     RigidBodyNative.worldGetArenaDirectByteBuffer(world));
 *
 * // 2. Main loop
 * while (running) {
 *     // Read body state — zero JNI
 *     for (int i = 0; i < arena.getBodyCount(); i++) {
 *         double posX = arena.getBodyPosX(i);
 *         double posY = arena.getBodyPosY(i);
 *         double posZ = arena.getBodyPosZ(i);
 *     }
 *
 *     // Write commands — zero JNI
 *     for (MyForce f : forces) {
 *         arena.commandAddForce(f.bodyIndex, f.fx, f.fy, f.fz);
 *     }
 *
 *     // Step — 1 JNI call
 *     RigidBodyNative.worldStep(world, 1.0 / 60.0);
 * }
 *
 * // 3. Cleanup
 *     RigidBodyNative.worldDestroySharedArena(world);
 *     RigidBodyNative.worldDestroy(world);
 * }
 * }</pre>
 */
public final class SharedPhysicsArena {

    // ---- Layout constants (must match shared_arena.rs exactly) ----

    static final int HEADER_SIZE = 128;
    static final int BODY_SLOT_STRIDE = 96;
    static final int CMD_SLOT_STRIDE = 32;
    static final int EVENT_SLOT_STRIDE = 64;

    // Header offsets (bytes)
    static final int OFF_MAGIC = 0;
    static final int OFF_VERSION = 8;
    static final int OFF_FLAGS = 12;
    static final int OFF_MAX_BODIES = 16;
    static final int OFF_MAX_COLLIDERS = 20;
    static final int OFF_MAX_EVENTS = 24;
    static final int OFF_MAX_COMMANDS = 28;
    static final int OFF_BODY_COUNT = 32;
    static final int OFF_COLLIDER_COUNT = 36;
    static final int OFF_EVENT_COUNT = 40;
    static final int OFF_CMD_COUNT = 44;
    static final int OFF_BODY_STRIDE = 48;
    static final int OFF_CMD_STRIDE = 52;
    static final int OFF_EVENT_STRIDE = 56;

    // Command types (must match CommandType in shared_arena.rs)
    static final int CMD_ADD_FORCE = 0;
    static final int CMD_ADD_TORQUE = 1;
    static final int CMD_SET_POSE = 2;
    static final int CMD_SET_VELOCITY = 3;
    static final int CMD_APPLY_IMPULSE = 4;
    static final int CMD_APPLY_TORQUE_IMPULSE = 5;
    static final int CMD_WAKE_UP = 6;
    static final int CMD_SLEEP = 7;

    // Event types
    static final int EVENT_COLLISION_START = 0;
    static final int EVENT_COLLISION_STOP = 1;
    static final int EVENT_CONTACT_FORCE = 2;

    // ---- Arena state ----

    private final ByteBuffer buf;
    private final DoubleBuffer doubleView;
    private final int maxBodies;
    private final int maxCommands;
    private final int maxEvents;

    // Pre-computed offsets (in doubles, since DoubleBuffer uses double indices)
    private final int bodySlotsStart; // in doubles
    private final int bodySlotsStartBytes;
    private final int cmdRingStart;
    private final int cmdRingStartBytes;
    private final int eventRingStart;
    private final int eventRingStartBytes;

    private SharedPhysicsArena(ByteBuffer buffer) {
        this.buf = buffer.order(ByteOrder.nativeOrder());
        this.doubleView = buf.asDoubleBuffer();

        this.maxBodies = readInt(OFF_MAX_BODIES);
        this.maxCommands = readInt(OFF_MAX_COMMANDS);
        this.maxEvents = readInt(OFF_MAX_EVENTS);

        this.bodySlotsStartBytes = HEADER_SIZE;
        this.bodySlotsStart = bodySlotsStartBytes / 8;
        this.cmdRingStartBytes = bodySlotsStartBytes + maxBodies * BODY_SLOT_STRIDE;
        this.cmdRingStart = cmdRingStartBytes / 8;
        this.eventRingStartBytes = cmdRingStartBytes + maxCommands * CMD_SLOT_STRIDE;
        this.eventRingStart = eventRingStartBytes / 8;
    }

    /**
     * Create a SharedPhysicsArena from a DirectByteBuffer obtained via
     * {@code RigidBodyNative.worldGetArenaDirectByteBuffer(world)}.
     *
     * <p>The returned buffer wraps the native arena memory directly.
     * No copies are made.</p>
     *
     * @param arenaBuffer a native-order DirectByteBuffer wrapping the arena
     * @return the arena helper
     * @throws IllegalArgumentException if the buffer is not a valid arena
     */
    public static SharedPhysicsArena fromDirectBuffer(ByteBuffer arenaBuffer) {
        if (!arenaBuffer.isDirect()) {
            throw new IllegalArgumentException("arena buffer must be direct");
        }
        SharedPhysicsArena arena = new SharedPhysicsArena(arenaBuffer);
        long magic = ((long) arena.readInt(OFF_MAGIC + 4) << 32) | (arena.readInt(OFF_MAGIC) & 0xFFFF_FFFFL);
        if (magic != 0x4D50535F4152454EL) { // "MPS_AREN"
            throw new IllegalArgumentException("invalid arena magic: 0x" + Long.toHexString(magic));
        }
        return arena;
    }

    // ---- Header readers ----

    private int readInt(int byteOffset) {
        // ByteBuffer.getInt uses native order if set earlier
        return buf.getInt(byteOffset);
    }

    public int getMaxBodies() { return maxBodies; }
    public int getMaxCommands() { return maxCommands; }
    public int getMaxEvents() { return maxEvents; }

    /** Number of active bodies in the arena (set by Rust after world_step). */
    public int getBodyCount() { return readInt(OFF_BODY_COUNT); }

    /** Number of pending events (set by Rust after world_step). */
    public int getEventCount() { return readInt(OFF_EVENT_COUNT); }

    // ---- Body state readers (zero JNI) ----

    private long bodySlotAddr(int index) {
        return (long) bodySlotsStartBytes + (long) index * BODY_SLOT_STRIDE;
    }

    /** Read a f64 field from a body slot at the given byte offset from the slot start. */
    private double readBodyDouble(int index, int fieldOffset) {
        return doubleView.get((bodySlotsStartBytes + index * BODY_SLOT_STRIDE + fieldOffset) / 8);
    }

    public long   getBodyGeneration(int index) { return buf.getLong((int)bodySlotAddr(index)); }
    public double getBodyPosX(int index)       { return readBodyDouble(index, 8); }
    public double getBodyPosY(int index)       { return readBodyDouble(index, 16); }
    public double getBodyPosZ(int index)       { return readBodyDouble(index, 24); }
    public double getBodyVelX(int index)       { return readBodyDouble(index, 32); }
    public double getBodyVelY(int index)       { return readBodyDouble(index, 40); }
    public double getBodyVelZ(int index)       { return readBodyDouble(index, 48); }
    public double getBodyAngVelX(int index)    { return readBodyDouble(index, 56); }
    public double getBodyAngVelY(int index)    { return readBodyDouble(index, 64); }
    public double getBodyAngVelZ(int index)    { return readBodyDouble(index, 72); }
    public int    getBodyType(int index)       { return buf.getInt((int)bodySlotAddr(index) + 80); }
    public boolean isBodySleeping(int index)   { return buf.getInt((int)bodySlotAddr(index) + 84) != 0; }

    /**
     * Safely read a body position, checking the generation counter to ensure
     * the data is not being written concurrently by Rust.
     *
     * @param index body slot index
     * @param out   array of length ≥ 3 to receive [x, y, z]
     * @return true if the read was consistent (generation matched)
     */
    public boolean safeReadBodyPosition(int index, double[] out) {
        int retries = 3;
        while (retries-- > 0) {
            long gen1 = getBodyGeneration(index);
            if ((gen1 & 1) != 0) continue; // odd = writing in progress

            out[0] = getBodyPosX(index);
            out[1] = getBodyPosY(index);
            out[2] = getBodyPosZ(index);

            long gen2 = getBodyGeneration(index);
            if (gen1 == gen2) return true; // consistent
        }
        return false;
    }

    // ---- Command writers (zero JNI) ----

    private long cmdSlotAddr(int index) {
        return (long) cmdRingStartBytes + (long) index * CMD_SLOT_STRIDE;
    }

    /**
     * Write an AddForce command to the arena command ring.
     *
     * <p>The command is processed by Rust at the start of the next
     * {@code worldStep} call.</p>
     *
     * @param bodyIndex body slot index (0-based)
     * @param fx force X component (N)
     * @param fy force Y component (N)
     * @param fz force Z component (N)
     */
    public void commandAddForce(int bodyIndex, double fx, double fy, double fz) {
        int cmdIdx = allocateCommandSlot();
        long addr = cmdSlotAddr(cmdIdx);
        buf.putInt((int)addr, CMD_ADD_FORCE);
        buf.putInt((int)addr + 4, bodyIndex);
        buf.putDouble((int)addr + 8, fx);
        buf.putDouble((int)addr + 16, fy);
        buf.putDouble((int)addr + 24, fz);
    }

    /**
     * Write an AddTorque command.
     */
    public void commandAddTorque(int bodyIndex, double tx, double ty, double tz) {
        int cmdIdx = allocateCommandSlot();
        long addr = cmdSlotAddr(cmdIdx);
        buf.putInt((int)addr, CMD_ADD_TORQUE);
        buf.putInt((int)addr + 4, bodyIndex);
        buf.putDouble((int)addr + 8, tx);
        buf.putDouble((int)addr + 16, ty);
        buf.putDouble((int)addr + 24, tz);
    }

    /**
     * Write an ApplyImpulse command.
     */
    public void commandApplyImpulse(int bodyIndex, double ix, double iy, double iz) {
        int cmdIdx = allocateCommandSlot();
        long addr = cmdSlotAddr(cmdIdx);
        buf.putInt((int)addr, CMD_APPLY_IMPULSE);
        buf.putInt((int)addr + 4, bodyIndex);
        buf.putDouble((int)addr + 8, ix);
        buf.putDouble((int)addr + 16, iy);
        buf.putDouble((int)addr + 24, iz);
    }

    /**
     * Write a WakeUp command.
     */
    public void commandWakeUp(int bodyIndex) {
        int cmdIdx = allocateCommandSlot();
        long addr = cmdSlotAddr(cmdIdx);
        buf.putInt((int)addr, CMD_WAKE_UP);
        buf.putInt((int)addr + 4, bodyIndex);
    }

    /**
     * Write a Sleep command.
     */
    public void commandSleep(int bodyIndex) {
        int cmdIdx = allocateCommandSlot();
        long addr = cmdSlotAddr(cmdIdx);
        buf.putInt((int)addr, CMD_SLEEP);
        buf.putInt((int)addr + 4, bodyIndex);
    }

    /**
     * Allocate a command slot in the lock-free SPSC ring.
     *
     * <p>This is safe without atomics because Java is the sole producer
     * and Rust is the sole consumer.</p>
     */
    private int allocateCommandSlot() {
        // Read write pointer from header (u32 at offset 44)
        int write = readInt(OFF_CMD_COUNT);
        // Simple bump allocator — ring wraps in drain_commands
        buf.putInt(OFF_CMD_COUNT, write + 1);
        return write;
    }

    // ---- Event ring readers (zero JNI) ----

    /**
     * Read an event from the event ring.
     *
     * @param index 0-based index into the event ring (0..eventCount-1)
     * @param out   array of length ≥ 8 to receive [type, collider1, collider2, flags, fx, fy, fz, fmag]
     * @return true if the read was successful
     */
    public boolean readEvent(int index, int[] out, double[] forceOut) {
        if (index < 0 || index >= getEventCount()) return false;

        long addr = (long) eventRingStartBytes + (long) index * EVENT_SLOT_STRIDE;

        out[0] = buf.getInt((int)addr);           // event_type
        out[1] = buf.getInt((int)addr + 4);       // collider1
        out[2] = buf.getInt((int)addr + 8);       // collider2
        out[3] = buf.getInt((int)addr + 12);      // flags

        if (forceOut != null && forceOut.length >= 4) {
            forceOut[0] = buf.getDouble((int)addr + 16);  // total_force_x
            forceOut[1] = buf.getDouble((int)addr + 24);  // total_force_y
            forceOut[2] = buf.getDouble((int)addr + 32);  // total_force_z
            forceOut[3] = buf.getDouble((int)addr + 40);  // total_force_magnitude
        }
        return true;
    }

    /**
     * Read all pending events into arrays.
     *
     * @return the number of events read
     */
    public int readAllEvents(int[] types, int[] c1, int[] c2, int[] flags, double[] fx, double[] fy, double[] fz) {
        int count = getEventCount();
        int n = Math.min(count, types.length);
        for (int i = 0; i < n; i++) {
            long addr = (long) eventRingStartBytes + (long) i * EVENT_SLOT_STRIDE;
            types[i] = buf.getInt((int)addr);
            c1[i] = buf.getInt((int)addr + 4);
            c2[i] = buf.getInt((int)addr + 8);
            flags[i] = buf.getInt((int)addr + 12);
            if (fx != null) fx[i] = buf.getDouble((int)addr + 16);
            if (fy != null) fy[i] = buf.getDouble((int)addr + 24);
            if (fz != null) fz[i] = buf.getDouble((int)addr + 32);
        }
        return n;
    }
}