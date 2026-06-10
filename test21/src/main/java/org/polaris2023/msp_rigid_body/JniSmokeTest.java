package org.polaris2023.msp_rigid_body;

import org.polaris2023.msp_rigid_body.util.PhysicsWorld;

public final class JniSmokeTest {
    private static final double EPSILON = 1.0e-9;

    private JniSmokeTest() {
    }

    public static void main(String[] args) {
        int javaVersion = Runtime.version().feature();
        if (javaVersion != 21) {
            throw new AssertionError("test21 must run on Java 21, got Java " + javaVersion);
        }

        int abiVersion = RigidBody.abiVersion();
        if (abiVersion < 1) {
            throw new AssertionError("invalid ABI version: " + abiVersion);
        }
        PhysicsWorld world = new PhysicsWorld(0.0, -9.81, 0.0);
        if (world.isEmpty()) {
            throw new AssertionError("worldCreate returned null");
        }

        assertClose(-9.81, world.gravityY(), "initial gravity y");
        world.set(1.0, 2.0, 3.0);
        assertClose(1.0, world.gravityX(), "gravity x");
        assertClose(2.0, world.gravityY(), "gravity y");
        assertClose(3.0, world.gravityZ(), "gravity z");

        world.body(0);
        if (world.bodyEmpty()) {
            throw new AssertionError("rigidBodyBuilderCreate returned null");
        }

        world.translation(4.0, 5.0, 6.0);
        PhysicsWorld insert = world.insert();
        if (world.physicsEmpty()) {
            throw new AssertionError("worldInsertRigidBody returned zero handle");
        }
        assertClose(4.0, world.translationX(), "body translation x");
        assertClose(5.0, world.translationY(), "body translation y");
        assertClose(6.0, world.translationZ(), "body translation z");
        world.step();

        long tree = RigidBody.crbTreeCreate();
        if (tree == 0L) {
            throw new AssertionError("crbTreeCreate returned null");
        }
        try {
            if (!RigidBody.crbTreeInsert(tree, 10L, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0)) {
                throw new AssertionError("crbTreeInsert 10 failed");
            }
            if (!RigidBody.crbTreeInsert(tree, 20L, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0)) {
                throw new AssertionError("crbTreeInsert 20 failed");
            }
            int hitCount = RigidBody.crbTreeQueryAabbCount(tree, 0.5, 0.5, 0.5, 2.5, 2.5, 2.5);
            if (hitCount != 2) {
                throw new AssertionError("crbTreeQueryAabbCount expected 2, got " + hitCount);
            }
        } finally {
            RigidBody.crbTreeDestroy(tree);
        }

        try {
            world.close();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        System.out.println("JNI smoke test passed on Java " + javaVersion);
    }

    private static void assertClose(double expected, double actual, String label) {
        if (Math.abs(expected - actual) > EPSILON) {
            throw new AssertionError(label + ": expected " + expected + ", got " + actual);
        }
    }
}

