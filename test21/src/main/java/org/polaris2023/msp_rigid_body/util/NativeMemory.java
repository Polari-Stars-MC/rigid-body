package org.polaris2023.msp_rigid_body.util;

import sun.misc.Unsafe;

import java.lang.reflect.Field;
import java.util.Arrays;

final class NativeMemory implements AutoCloseable {
    static final Unsafe UNSAFE = unsafe();
    static final int ADDRESS_BYTES = Long.BYTES;

    private long address;
    private final long bytes;

    // P7: preallocate reusable arrays to avoid per-frame allocations
    private final ThreadLocal<double[]> vec3Buffer = ThreadLocal.withInitial(() -> new double[3]);

    NativeMemory(long bytes) {
        if (bytes <= 0L) {
            throw new IllegalArgumentException("native memory size must be positive");
        }
        this.bytes = bytes;
        this.address = UNSAFE.allocateMemory(bytes);
        UNSAFE.setMemory(address, bytes, (byte) 0);
    }

    static NativeMemory longs(int count) {
        if (count <= 0) {
            throw new IllegalArgumentException("count must be positive");
        }
        return new NativeMemory((long) count * Long.BYTES);
    }

    static NativeMemory bytes(byte[] data) {
        NativeMemory memory = new NativeMemory(data.length);
        for (int i = 0; i < data.length; i++) {
            UNSAFE.putByte(memory.address + i, data[i]);
        }
        return memory;
    }

    long address() {
        if (address == 0L) {
            throw new IllegalStateException("native memory is closed");
        }
        return address;
    }

    long getLong(long offset) {
        return UNSAFE.getLong(address() + offset);
    }

    int getInt(long offset) {
        return UNSAFE.getInt(address() + offset);
    }

    double getDouble(long offset) {
        return UNSAFE.getDouble(address() + offset);
    }

    boolean getBool(long offset) {
        return UNSAFE.getByte(address() + offset) != 0;
    }

    void putByte(long offset, int value) {
        UNSAFE.putByte(address() + offset, (byte) value);
    }

    void putLong(long offset, long value) {
        UNSAFE.putLong(address() + offset, value);
    }

    void putDouble(long offset, double value) {
        UNSAFE.putDouble(address() + offset, value);
    }

    /// P7: reuse a preallocated array via ThreadLocal to avoid per-read allocations.
    /// Callers MUST consume/process the returned array before calling getVec3 again.
    double[] getVec3(long offset) {
        double[] buf = vec3Buffer.get();
        buf[0] = getDouble(offset);
        buf[1] = getDouble(offset + 8);
        buf[2] = getDouble(offset + 16);
        return buf;
    }

    /// Safe copy variant: writes into the caller-provided array without allocation.
    void getVec3(long offset, double[] out, int outOffset) {
        out[outOffset] = getDouble(offset);
        out[outOffset + 1] = getDouble(offset + 8);
        out[outOffset + 2] = getDouble(offset + 16);
    }

    long[] getLongs(int count) {
        if ((long) count * Long.BYTES > bytes) {
            throw new IllegalArgumentException("count exceeds buffer size");
        }
        long[] values = new long[count];
        for (int i = 0; i < count; i++) {
            values[i] = getLong((long) i * Long.BYTES);
        }
        return values;
    }

    long[] getLongs(int count, int limit) {
        return Arrays.copyOf(getLongs(count), limit);
    }

    @Override
    public void close() {
        if (address != 0L) {
            UNSAFE.freeMemory(address);
            address = 0L;
        }
    }

    private static Unsafe unsafe() {
        try {
            Field field = Unsafe.class.getDeclaredField("theUnsafe");
            field.setAccessible(true);
            return (Unsafe) field.get(null);
        } catch (ReflectiveOperationException exception) {
            throw new ExceptionInInitializerError(exception);
        }
    }
}
