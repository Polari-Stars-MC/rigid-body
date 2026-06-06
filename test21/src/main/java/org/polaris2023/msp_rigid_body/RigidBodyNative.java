package org.polaris2023.msp_rigid_body;

import java.nio.file.Files;
import java.nio.file.Path;

public final class RigidBodyNative {
    static {
        loadNativeLibrary();
    }

    private RigidBodyNative() {
    }

    private static void loadNativeLibrary() {
        String explicitPath = System.getProperty("rigidbody.native.path");
        if (explicitPath != null && !explicitPath.isBlank()) {
            System.load(Path.of(explicitPath).toAbsolutePath().normalize().toString());
            return;
        }

        try {
            System.loadLibrary("msp_rigid_body");
            return;
        } catch (UnsatisfiedLinkError loadLibraryError) {
            String mappedName = System.mapLibraryName("msp_rigid_body");
            Path[] candidates = {
                    Path.of("target", "release", mappedName),
                    Path.of("..", "target", "release", mappedName)
            };

            for (Path candidate : candidates) {
                Path absolute = candidate.toAbsolutePath().normalize();
                if (Files.isRegularFile(absolute)) {
                    System.load(absolute.toString());
                    return;
                }
            }

            UnsatisfiedLinkError error = new UnsatisfiedLinkError(
                    loadLibraryError.getMessage()
                            + "; also tried -Drigidbody.native.path and "
                            + "target/release/" + mappedName);
            error.initCause(loadLibraryError);
            throw error;
        }
    }

    public static native int abiVersion();

    public static native long worldCreate(double gravityX, double gravityY, double gravityZ);

    public static native long colliderBuilderCreatePointCloudBounds(long pointsXyz, int pointCount);

    public static native long colliderBuilderCreateDoubleBv(
            double aMinX,
            double aMinY,
            double aMinZ,
            double aMaxX,
            double aMaxY,
            double aMaxZ,
            double bMinX,
            double bMinY,
            double bMinZ,
            double bMaxX,
            double bMaxY,
            double bMaxZ);

    public static native long colliderBuilderCreateSkewedObb(
            double centerX,
            double centerY,
            double centerZ,
            double axisXX,
            double axisXY,
            double axisXZ,
            double axisYX,
            double axisYY,
            double axisYZ,
            double axisZX,
            double axisZY,
            double axisZZ);

    public static native long colliderBuilderCreateDiscreteObb(long pointsXyz, int pointCount, int axis);

    public static native long colliderBuilderCreateDiscreteObbEx(
            long pointsXyz, int pointCount, long rotationsXyzw, int rotationCount);

    public static native long colliderBuilderCreateFusedCollapsingBounds(long pointsXyz, int pointCount, double padding);

    public static native long colliderBuilderCreateEdgeBvh(
            long verticesXyz,
            int vertexCount,
            long edges,
            int edgeCount,
            double radius);

    public static native long colliderBuilderCreateMedialSpheres(long spheresXyzw, int sphereCount);

    public static native long boundShapeCreateCapsule(
            double ax, double ay, double az, double bx, double by, double bz, double radius);

    public static native long boundShapeCreateSsv(
            double ax, double ay, double az, double bx, double by, double bz, double radius);

    public static native long boundShapeCreateEllipsoid(
            double cx,
            double cy,
            double cz,
            double rx,
            double ry,
            double rz,
            double qi,
            double qj,
            double qk,
            double qw,
            int segments);

    public static native long boundShapeCreatePrism(
            double cx,
            double cy,
            double cz,
            double radius,
            double halfHeight,
            int sides,
            double qi,
            double qj,
            double qk,
            double qw);

    public static native long boundShapeCreateCylinder(
            double cx,
            double cy,
            double cz,
            double radius,
            double halfHeight,
            double qi,
            double qj,
            double qk,
            double qw);

    public static native long boundShapeCreateSphericalShell(
            double cx, double cy, double cz, double innerRadius, double outerRadius);

    public static native void boundShapeDestroy(long bound);

    public static native int queryIntersectBoundShapeCount(
            long world,
            long bound,
            int flags,
            int memberships,
            int filter,
            int useGroups,
            long excludeCollider,
            int useExcludeCollider,
            long excludeRigidBody,
            int useExcludeRigidBody);

    public static native int queryIntersectBoundShape(
            long world,
            long bound,
            int flags,
            int memberships,
            int filter,
            int useGroups,
            long excludeCollider,
            int useExcludeCollider,
            long excludeRigidBody,
            int useExcludeRigidBody,
            long outHandles,
            int capacity);

    public static native long neuralBoundsCreate(
            double cx,
            double cy,
            double cz,
            double hx,
            double hy,
            double hz,
            double qi,
            double qj,
            double qk,
            double qw,
            int sampleResolution,
            int hiddenWidth,
            int hiddenLayers,
            int activation,
            double outputScale,
            double padding,
            long weights,
            int weightCount);

    public static native void neuralBoundsDestroy(long neural);

    public static native int queryIntersectNeuralBoundsHandleCount(
            long world,
            long neural,
            int flags,
            int memberships,
            int filter,
            int useGroups,
            long excludeCollider,
            int useExcludeCollider,
            long excludeRigidBody,
            int useExcludeRigidBody);

    public static native int queryIntersectNeuralBoundsHandle(
            long world,
            long neural,
            int flags,
            int memberships,
            int filter,
            int useGroups,
            long excludeCollider,
            int useExcludeCollider,
            long excludeRigidBody,
            int useExcludeRigidBody,
            long outHandles,
            int capacity);

    public static native void worldDestroy(long world);

    public static native void worldStep(long world, double deltaSeconds);

    public static native void worldSetGravity(long world, double gravityX, double gravityY, double gravityZ);

    public static native double worldGetGravityX(long world);

    public static native double worldGetGravityY(long world);

    public static native double worldGetGravityZ(long world);

    public static native int worldDynamicBodySnapshotCount(long world);

    public static native long rigidBodyBuilderCreate(int status);

    public static native void rigidBodyBuilderDestroy(long builder);

    public static native void rigidBodyBuilderSetTranslation(long builder, double x, double y, double z);

    public static native long worldInsertRigidBody(long world, long builder);

    public static native double rigidBodyGetTranslationX(long world, long body);

    public static native double rigidBodyGetTranslationY(long world, long body);

    public static native double rigidBodyGetTranslationZ(long world, long body);

    public static native long crbTreeCreate();

    public static native void crbTreeDestroy(long tree);

    public static native void crbTreeClear(long tree);

    public static native int crbTreeLen(long tree);

    public static native int crbTreeStats(long tree, long outStats);

    public static native boolean crbTreeContains(long tree, long id);

    public static native int crbTreeContainsBatch(long tree, long ids, int count, long outValues);

    public static native boolean crbTreeInsert(
            long tree,
            long id,
            double minX,
            double minY,
            double minZ,
            double maxX,
            double maxY,
            double maxZ);

    public static native boolean crbTreeUpdate(
            long tree,
            long id,
            double minX,
            double minY,
            double minZ,
            double maxX,
            double maxY,
            double maxZ);

    public static native boolean crbTreeRemove(long tree, long id);
    public static native int crbTreeUpdateBatch(long tree, long ids, long aabbs, int count);
    public static native int crbTreeRemoveBatch(long tree, long ids, int count);

    public static native int rtreeInsertBatch(long tree, long ids, long aabbs, int count);
    public static native int rtreeUpdateBatch(long tree, long ids, long aabbs, int count);
    public static native int rtreeRemoveBatch(long tree, long ids, int count);

    public static native int crbTreeQueryAabbCount(
            long tree,
            double minX,
            double minY,
            double minZ,
            double maxX,
            double maxY,
            double maxZ);

    public static native int crbTreeQueryAabb(
            long tree,
            double minX,
            double minY,
            double minZ,
            double maxX,
            double maxY,
            double maxZ,
            long outIds,
            int capacity);

    public static native int crbTreeQueryAabbCounts(long tree, long aabbs, int count, long outCounts);
    public static native int crbTreeQueryAabbs(
            long tree, long aabbs, int count, long outOffsets, long outIds, int idCapacity);

    public static native int crbTreeQueryAabbUnsorted(
            long tree,
            double minX,
            double minY,
            double minZ,
            double maxX,
            double maxY,
            double maxZ,
            long outIds,
            int capacity);

    public static native boolean abiSupportsFfm();
    public static native boolean abiSupportsJni();
    public static native int worldDynamicBodySnapshot(long world, long outHandles, long outValues, int capacity);
    public static native long colliderBuilderCreate(int shapeType, double a, double b, double c);
    public static native long colliderBuilderCreateEx(int shapeType, double a, double b, double c, double d);
    public static native long colliderBuilderCreateSphere(double x, double y, double z, double radius);
    public static native long colliderBuilderCreateObb(double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw);
    public static native long colliderBuilderCreateConvexHull(long pointsXyz, int pointCount);
    public static native void colliderBuilderDestroy(long builder);
    public static native void colliderBuilderSetTranslation(long builder, double x, double y, double z);
    public static native void colliderBuilderSetRotation(long builder, double x, double y, double z);
    public static native void colliderBuilderSetPose(long builder, double x, double y, double z, double qi, double qj, double qk, double qw);
    public static native void colliderBuilderSetSensor(long builder, int sensor);
    public static native void colliderBuilderSetFriction(long builder, double friction);
    public static native void colliderBuilderSetRestitution(long builder, double restitution);
    public static native void colliderBuilderSetDensity(long builder, double density);
    public static native void colliderBuilderSetCollisionGroups(long builder, int memberships, int filter);
    public static native void colliderBuilderSetSolverGroups(long builder, int memberships, int filter);
    public static native void colliderBuilderSetActiveEvents(long builder, int bits);
    public static native void colliderBuilderSetActiveHooks(long builder, int bits);
    public static native void colliderBuilderSetContactForceEventThreshold(long builder, double threshold);
    public static native long worldInsertCollider(long world, long builder);
    public static native long worldInsertColliderWithParent(long world, long builder, long parent);
    public static native boolean worldRemoveCollider(long world, long handle, int wakeUp);
    public static native boolean colliderSetPose(long world, long handle, double x, double y, double z, double qi, double qj, double qk, double qw);
    public static native boolean colliderSetSensor(long world, long handle, int sensor);
    public static native boolean colliderSetFriction(long world, long handle, double friction);
    public static native boolean colliderSetRestitution(long world, long handle, double restitution);
    public static native boolean colliderSetCollisionGroups(long world, long handle, int memberships, int filter);
    public static native boolean colliderSetSolverGroups(long world, long handle, int memberships, int filter);
    public static native boolean colliderSetActiveEvents(long world, long handle, int bits);
    public static native boolean colliderSetActiveHooks(long world, long handle, int bits);
    public static native boolean colliderSetContactForceEventThreshold(long world, long handle, double threshold);
    public static native double colliderGetDensity(long world, long handle);
    public static native void rigidBodyBuilderSetRotation(long builder, double x, double y, double z);
    public static native void rigidBodyBuilderSetPose(long builder, double x, double y, double z, double qi, double qj, double qk, double qw);
    public static native void rigidBodyBuilderSetLinvel(long builder, double x, double y, double z);
    public static native void rigidBodyBuilderSetAngvel(long builder, double x, double y, double z);
    public static native void rigidBodyBuilderSetGravityScale(long builder, double value);
    public static native void rigidBodyBuilderSetLinearDamping(long builder, double value);
    public static native void rigidBodyBuilderSetAngularDamping(long builder, double value);
    public static native void rigidBodyBuilderSetCanSleep(long builder, int value);
    public static native void rigidBodyBuilderSetEnabledRotations(long builder, int x, int y, int z);
    public static native void rigidBodyBuilderSetUserData(long builder, long low, long high);
    public static native void rigidBodyBuilderSetAdditionalMass(long builder, double mass);
    public static native boolean worldRemoveRigidBody(long world, long handle, int removeAttachedColliders);
    public static native int rigidBodyGetStatus(long world, long handle);
    public static native boolean rigidBodySetStatus(long world, long handle, int status, int wakeUp);
    public static native boolean rigidBodySetPose(long world, long body, double x, double y, double z, double qi, double qj, double qk, double qw, int wakeUp);
    public static native boolean rigidBodySetLinvel(long world, long body, double x, double y, double z, int wakeUp);
    public static native boolean rigidBodySetAngvel(long world, long body, double x, double y, double z, int wakeUp);
    public static native boolean rigidBodyAddForce(long world, long body, double x, double y, double z, int wakeUp);
    public static native boolean rigidBodyAddTorque(long world, long body, double x, double y, double z, int wakeUp);
    public static native boolean rigidBodyApplyImpulse(long world, long body, double x, double y, double z, int wakeUp);
    public static native boolean rigidBodyApplyTorqueImpulse(long world, long body, double x, double y, double z, int wakeUp);
    public static native boolean rigidBodyEnableCcd(long world, long body, int enabled);
    public static native boolean rigidBodySleep(long world, long body);
    public static native boolean rigidBodyWakeUp(long world, long body, int strong);
    public static native boolean rigidBodyIsSleeping(long world, long body);
    public static native long colliderBuilderCreateCapsule(double ax, double ay, double az, double bx, double by, double bz, double radius);
    public static native long colliderBuilderCreateSsv(double ax, double ay, double az, double bx, double by, double bz, double radius);
    public static native long colliderBuilderCreateEllipsoid(double cx, double cy, double cz, double rx, double ry, double rz, double qi, double qj, double qk, double qw, int segments);
    public static native long colliderBuilderCreatePrism(double cx, double cy, double cz, double radius, double halfHeight, int sides, double qi, double qj, double qk, double qw);
    public static native long colliderBuilderCreateCylinder(double cx, double cy, double cz, double radius, double halfHeight, double qi, double qj, double qk, double qw);
    public static native long colliderBuilderCreateSphericalShell(double cx, double cy, double cz, double innerRadius, double outerRadius);
    public static native long queryCastRay(long world, double ox, double oy, double oz, double dx, double dy, double dz, double maxToi, int solid, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody, long outHit);
    public static native int queryIntersectAabbCount(long world, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody);
    public static native int queryIntersectObb(long world, double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody, long outHandles, int capacity);
    public static native int queryIntersectSphere(long world, double cx, double cy, double cz, double radius, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody, long outHandles, int capacity);
    public static native long queryCastShape(long world, int shapeType, double a, double b, double c, double d, double tx, double ty, double tz, double qi, double qj, double qk, double qw, double vx, double vy, double vz, double maxToi, double targetDistance, int stopAtPenetration, int computeImpactGeometryOnPenetration, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody, long outHit);
    public static native long colliderBuilderCreateKdop(long pointsXyz, int pointCount, int preset);
    public static native long colliderBuilderCreateFdh(long pointsXyz, int pointCount, long directionsXyz, int directionCount);
    public static native int neuralBoundsRequiredWeightCount(int hiddenWidth, int hiddenLayers);
    public static native long colliderBuilderCreateNeuralBounds(double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, int sampleResolution, int hiddenWidth, int hiddenLayers, int activation, double outputScale, double padding, long weights, int weightCount);
    public static native long colliderBuilderCreateVoxels(long voxels, int sizeX, int sizeY, int sizeZ, double voxelSize, double originX, double originY, double originZ, int mode, int dynamicBody, int smallVoxelLimit, int meshVoxelLimit);
    public static native int worldInsertUrdfFromBytesDefault(long world, long urdfBytes, int urdfLen, long outBodyHandles, int bodyCapacity, long outResult);
    public static native int worldInsertUrdfFromBytesDefaultEx(long world, long urdfBytes, int urdfLen, long outBodyHandles, int bodyCapacity, long outColliderHandles, int colliderCapacity, long outJointHandles, int jointCapacity, long outResult);
    public static native int worldInsertUrdfFromBytes(long world, long urdfBytes, int urdfLen, int createCollisionColliders, int createVisualColliders, int makeRootsFixed, double scale, double density, double friction, double restitution, long outBodyHandles, int bodyCapacity, long outResult);
    public static native int worldInsertUrdfFromBytesEx(long world, long urdfBytes, int urdfLen, int createCollisionColliders, int createVisualColliders, int makeRootsFixed, double scale, double density, double friction, double restitution, long outBodyHandles, int bodyCapacity, long outColliderHandles, int colliderCapacity, long outJointHandles, int jointCapacity, long outResult);
    public static native int worldInsertUrdfFromBytesCount(long world, long urdfBytes, int urdfLen);
    public static native int worldInsertMjcfFromBytesDefault(long world, long mjcfBytes, int mjcfLen, long outBodyHandles, int bodyCapacity, long outResult);
    public static native int worldInsertMjcfFromBytesDefaultEx(long world, long mjcfBytes, int mjcfLen, long outBodyHandles, int bodyCapacity, long outColliderHandles, int colliderCapacity, long outJointHandles, int jointCapacity, long outResult);
    public static native int worldInsertMjcfFromBytes(long world, long mjcfBytes, int mjcfLen, int makeRootsFixed, double scale, double density, double friction, double restitution, long outBodyHandles, int bodyCapacity, long outResult);
    public static native int worldInsertMjcfFromBytesEx(long world, long mjcfBytes, int mjcfLen, int makeRootsFixed, double scale, double density, double friction, double restitution, long outBodyHandles, int bodyCapacity, long outColliderHandles, int colliderCapacity, long outJointHandles, int jointCapacity, long outResult);
    public static native int worldInsertMjcfFromBytesCount(long world, long mjcfBytes, int mjcfLen);
    public static native long worldInsertDynamicCuboids(long world, double x, double y, double z, double qi, double qj, double qk, double qw, double lvx, double lvy, double lvz, long cuboids, int cuboidCount, double density, double friction, double restitution, int collisionMemberships, int collisionFilter, int solverMemberships, int solverFilter);
    public static native long worldInsertStaticTrimesh(long world, long verticesXyz, int vertexXyzLen, long indices, int indexLen, double friction, double restitution);
    public static native long jointBuilderCreate(int jointType, double ax, double ay, double az, double b, double c);
    public static native void jointBuilderDestroy(long builder);
    public static native void jointBuilderSetContactsEnabled(long builder, int enabled);
    public static native void jointBuilderSetLocalAnchor1(long builder, double x, double y, double z);
    public static native void jointBuilderSetLocalAnchor2(long builder, double x, double y, double z);
    public static native void jointBuilderSetLimits(long builder, int axis, double min, double max);
    public static native void jointBuilderSetMotorVelocity(long builder, int axis, double targetVel, double factor);
    public static native void jointBuilderSetMotorPosition(long builder, int axis, double targetPos, double stiffness, double damping);
    public static native long worldInsertImpulseJoint(long world, long body1, long body2, long builder, int wakeUp);
    public static native boolean worldRemoveImpulseJoint(long world, long handle, int wakeUp);
    public static native long characterControllerCreate();
    public static native void characterControllerDestroy(long controller);
    public static native void characterControllerSetUp(long controller, double x, double y, double z);
    public static native void characterControllerSetOffsetAbsolute(long controller, double offset);
    public static native void characterControllerSetOffsetRelative(long controller, double offset);
    public static native void characterControllerSetSlide(long controller, int slide);
    public static native void characterControllerSetAutostep(long controller, int enabled, double maxHeight, double minWidth, int includeDynamicBodies);
    public static native void characterControllerSetSnapToGround(long controller, int enabled, double distance);
    public static native void characterControllerSetSlopeAngles(long controller, double maxClimbAngle, double minSlideAngle);
    public static native boolean characterControllerMoveShape(long world, long controller, double dt, int shapeType, double a, double b, double c, double d, double tx, double ty, double tz, double qi, double qj, double qk, double qw, double dx, double dy, double dz, long outMovement);
    public static native int characterControllerCollisionCount(long controller);
    public static native long characterControllerGetCollision(long controller, int index, long outCollision);
    public static native boolean characterControllerSolveImpulses(long world, long controller, double dt, int shapeType, double a, double b, double c, double d, double characterMass);
    public static native void worldClearEvents(long world);
    public static native int worldCollisionEventCount(long world);
    public static native long worldGetCollisionEvent(long world, int index, long outEvent);
    public static native int worldDrainCollisionEvents(long world, long outEvents, int capacity);
    public static native int worldContactForceEventCount(long world);
    public static native long worldGetContactForceEvent(long world, int index, long outEvent);
    public static native int worldDrainContactForceEvents(long world, long outEvents, int capacity);
    public static native void worldSetContactPairFilterCallback(long world, long callback, long userData);
    public static native void worldSetIntersectionPairFilterCallback(long world, long callback, long userData);
    public static native void worldClearContactPairFilterCallback(long world);
    public static native void worldClearIntersectionPairFilterCallback(long world);
    public static native long rtreeCreate();
    public static native void rtreeDestroy(long tree);
    public static native void rtreeClear(long tree);
    public static native int rtreeLen(long tree);
    public static native int rtreeNodeCount(long tree);
    public static native int rtreeHeight(long tree);
    public static native boolean rtreeIsDirty(long tree);
    public static native int rtreeStats(long tree, long outStats);
    public static native boolean rtreeContains(long tree, long id);
    public static native int rtreeContainsBatch(long tree, long ids, int count, long outValues);
    public static native boolean rtreeInsert(long tree, long id, double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
    public static native boolean rtreeUpdate(long tree, long id, double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
    public static native boolean rtreeRemove(long tree, long id);
    public static native void rtreeRebuild(long tree);
    public static native int rtreeQueryAabbCount(long tree, double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
    public static native int rtreeQueryAabbCounts(long tree, long aabbs, int count, long outCounts);
    public static native int rtreeQueryAabbs(
            long tree, long aabbs, int count, long outOffsets, long outIds, int idCapacity);
    public static native int rtreeQueryAabb(long tree, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, long outIds, int capacity);
}
