package org.polaris2023.msp_rigid_body.util;

import org.polaris2023.msp_rigid_body.RigidBodyNative;

public final class Joint {
    public static final int FIXED = 0;
    public static final int REVOLUTE = 1;
    public static final int PRISMATIC = 2;
    public static final int ROPE = 3;
    public static final int SPRING = 4;
    public static final int SPHERICAL = 5;

    public static final int AXIS_X = 0;
    public static final int AXIS_Y = 1;
    public static final int AXIS_Z = 2;
    public static final int AXIS_ANG_X = 3;
    public static final int AXIS_ANG_Y = 4;
    public static final int AXIS_ANG_Z = 5;

    private final PhysicsWorld world;
    private long handle;

    Joint(PhysicsWorld world, long handle) {
        this.world = world;
        this.handle = handle;
    }

    public boolean isEmpty() {
        return handle == 0L;
    }

    public long handle() {
        return handle;
    }

    public boolean remove(boolean wakeUp) {
        if (handle == 0L) {
            return false;
        }
        boolean removed = RigidBodyNative.worldRemoveImpulseJoint(world.handle(), handle, wakeUp ? 1 : 0);
        if (removed) {
            handle = 0L;
        }
        return removed;
    }

    public static final class Builder implements AutoCloseable {
        private final PhysicsWorld world;
        private long handle;

        private Builder(PhysicsWorld world, long handle) {
            this.world = world;
            this.handle = handle;
        }

        public static Builder fixed(PhysicsWorld world) {
            return create(world, FIXED, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        public static Builder revolute(PhysicsWorld world, double ax, double ay, double az) {
            return create(world, REVOLUTE, ax, ay, az, 0.0, 0.0);
        }

        public static Builder prismatic(PhysicsWorld world, double ax, double ay, double az) {
            return create(world, PRISMATIC, ax, ay, az, 0.0, 0.0);
        }

        public static Builder rope(PhysicsWorld world, double maxDistance) {
            return create(world, ROPE, 0.0, 0.0, 0.0, maxDistance, 0.0);
        }

        public static Builder spring(PhysicsWorld world, double restLength, double stiffness, double damping) {
            return create(world, SPRING, restLength, 0.0, 0.0, stiffness, damping);
        }

        public static Builder spherical(PhysicsWorld world) {
            return create(world, SPHERICAL, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        public static Builder create(PhysicsWorld world, int type, double ax, double ay, double az, double b, double c) {
            long handle = RigidBodyNative.jointBuilderCreate(type, ax, ay, az, b, c);
            return new Builder(world, handle);
        }

        public Builder contactsEnabled(boolean enabled) {
            requireOpen();
            RigidBodyNative.jointBuilderSetContactsEnabled(handle, enabled ? 1 : 0);
            return this;
        }

        public Builder localAnchor1(double x, double y, double z) {
            requireOpen();
            RigidBodyNative.jointBuilderSetLocalAnchor1(handle, x, y, z);
            return this;
        }

        public Builder localAnchor2(double x, double y, double z) {
            requireOpen();
            RigidBodyNative.jointBuilderSetLocalAnchor2(handle, x, y, z);
            return this;
        }

        public Builder limits(int axis, double min, double max) {
            requireOpen();
            RigidBodyNative.jointBuilderSetLimits(handle, axis, min, max);
            return this;
        }

        public Builder motorVelocity(int axis, double targetVelocity, double factor) {
            requireOpen();
            RigidBodyNative.jointBuilderSetMotorVelocity(handle, axis, targetVelocity, factor);
            return this;
        }

        public Builder motorPosition(int axis, double targetPosition, double stiffness, double damping) {
            requireOpen();
            RigidBodyNative.jointBuilderSetMotorPosition(handle, axis, targetPosition, stiffness, damping);
            return this;
        }

        public Joint insert(RigidBody body1, RigidBody body2, boolean wakeUp) {
            requireOpen();
            long joint = RigidBodyNative.worldInsertImpulseJoint(
                    world.handle(), body1.handle(), body2.handle(), handle, wakeUp ? 1 : 0);
            handle = 0L;
            return new Joint(world, joint);
        }

        @Override
        public void close() {
            if (handle != 0L) {
                RigidBodyNative.jointBuilderDestroy(handle);
                handle = 0L;
            }
        }

        private void requireOpen() {
            if (handle == 0L) {
                throw new IllegalStateException("joint builder is closed");
            }
        }
    }
}
