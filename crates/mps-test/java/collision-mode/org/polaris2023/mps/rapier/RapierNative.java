package org.polaris2023.mps.rapier;

/** Standalone JNI smoke fixture; compile separately from the application class. */
public final class RapierNative {
    private RapierNative() {}
    public static native long worldCreateWithCollisionMode(double x, double y, double z, int mode);
    public static native boolean worldSetDefaultCollisionMode(long world, int mode);
    public static native int worldGetDefaultCollisionMode(long world);
    public static native long worldInsertDefaultCollider(long world, long body, long simple, long compound);
    public static native void worldDestroy(long world);
    public static native void worldStep(long world, double dt);
    public static native int worldGetColliderSetSize(long world);
    public static native long rigidBodyBuilderCreate(int status);
    public static native long rigidBodyBuilderBuild(long builder);
    public static native long worldInsertRigidBody(long world, long body);
    public static native long colliderBuilderCreate(int type, double x, double y, double z);
    public static native long colliderBuilderCreateCompoundBoxesArray(double[] boxes, int count);
    public static native void colliderBuilderDestroy(long builder);
    public static native int abiLastErrorCode();

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }

    public static void main(String[] args) {
        System.load(args[0]);
        long world = worldCreateWithCollisionMode(0, 0, 0, 1);
        check(world != 0, "world creation failed");
        long simple = 0;
        long compound = 0;
        try {
            long body = worldInsertRigidBody(world, rigidBodyBuilderBuild(rigidBodyBuilderCreate(0)));
            check(body != 0, "body insertion failed");
            simple = colliderBuilderCreate(0, 0.5, 0, 0);
            compound = colliderBuilderCreateCompoundBoxesArray(new double[] {
                -0.5, -0.5, -0.5, 0, 0.5, 0.5,
                0, -0.5, -0.5, 0.5, 0.5, 0.5
            }, 2);
            check(simple != 0 && compound != 0, "builder creation failed");
            check(worldInsertDefaultCollider(world, body, simple, 0) != 0, "simple insertion failed");
            check(worldSetDefaultCollisionMode(world, 2), "compound policy failed");
            check(worldInsertDefaultCollider(world, body, 0, compound) != 0, "compound insertion failed");
            check(worldSetDefaultCollisionMode(world, 0), "none policy failed");
            check(worldInsertDefaultCollider(world, body, 0, 0) == 0 && abiLastErrorCode() == 0, "none insertion failed");
            check(worldGetColliderSetSize(world) == 2, "existing colliders changed");
            check(!worldSetDefaultCollisionMode(world, -1), "negative mode accepted");
            check(abiLastErrorCode() == 2, "wrong invalid mode error");
            check(worldGetDefaultCollisionMode(world) == 0, "invalid mode mutated policy");
            worldStep(world, 0.01);
            System.out.println("JNI collision mode smoke test passed");
        } finally {
            colliderBuilderDestroy(simple);
            colliderBuilderDestroy(compound);
            worldDestroy(world);
        }
    }
}
