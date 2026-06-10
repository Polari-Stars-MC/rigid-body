package org.polaris2023.msp_rigid_body;

public final class RigidBody {
    private RigidBody() {
    }

    public static int abiVersion() {
        return RigidBodyNative.abiVersion();
    }

    public static long worldCreate(double gravityX, double gravityY, double gravityZ) {
        return RigidBodyNative.worldCreate(gravityX, gravityY, gravityZ);
    }

    public static long colliderBuilderCreatePointCloudBounds(long pointsXyz, int pointCount) {
        return RigidBodyNative.colliderBuilderCreatePointCloudBounds(pointsXyz, pointCount);
    }

    public static long colliderBuilderCreateDoubleBv(double aMinX, double aMinY, double aMinZ, double aMaxX, double aMaxY, double aMaxZ, double bMinX, double bMinY, double bMinZ, double bMaxX, double bMaxY, double bMaxZ) {
        return RigidBodyNative.colliderBuilderCreateDoubleBv(aMinX, aMinY, aMinZ, aMaxX, aMaxY, aMaxZ, bMinX, bMinY, bMinZ, bMaxX, bMaxY, bMaxZ);
    }

    public static long colliderBuilderCreateSkewedObb(double centerX, double centerY, double centerZ, double axisXX, double axisXY, double axisXZ, double axisYX, double axisYY, double axisYZ, double axisZX, double axisZY, double axisZZ) {
        return RigidBodyNative.colliderBuilderCreateSkewedObb(centerX, centerY, centerZ, axisXX, axisXY, axisXZ, axisYX, axisYY, axisYZ, axisZX, axisZY, axisZZ);
    }

    public static long colliderBuilderCreateDiscreteObb(long pointsXyz, int pointCount, int axis) {
        return RigidBodyNative.colliderBuilderCreateDiscreteObb(pointsXyz, pointCount, axis);
    }

    public static long colliderBuilderCreateDiscreteObbEx(long pointsXyz, int pointCount, long rotationsXyzw, int rotationCount) {
        return RigidBodyNative.colliderBuilderCreateDiscreteObbEx(pointsXyz, pointCount, rotationsXyzw, rotationCount);
    }

    public static long colliderBuilderCreateFusedCollapsingBounds(long pointsXyz, int pointCount, double padding) {
        return RigidBodyNative.colliderBuilderCreateFusedCollapsingBounds(pointsXyz, pointCount, padding);
    }

    public static long colliderBuilderCreateEdgeBvh(long verticesXyz, int vertexCount, long edges, int edgeCount, double radius) {
        return RigidBodyNative.colliderBuilderCreateEdgeBvh(verticesXyz, vertexCount, edges, edgeCount, radius);
    }

    public static long colliderBuilderCreateMedialSpheres(long spheresXyzw, int sphereCount) {
        return RigidBodyNative.colliderBuilderCreateMedialSpheres(spheresXyzw, sphereCount);
    }

    public static long boundShapeCreateCapsule(double ax, double ay, double az, double bx, double by, double bz, double radius) {
        return RigidBodyNative.boundShapeCreateCapsule(ax, ay, az, bx, by, bz, radius);
    }

    public static long boundShapeCreateSsv(double ax, double ay, double az, double bx, double by, double bz, double radius) {
        return RigidBodyNative.boundShapeCreateSsv(ax, ay, az, bx, by, bz, radius);
    }

    public static long boundShapeCreateEllipsoid(double cx, double cy, double cz, double rx, double ry, double rz, double qi, double qj, double qk, double qw, int segments) {
        return RigidBodyNative.boundShapeCreateEllipsoid(cx, cy, cz, rx, ry, rz, qi, qj, qk, qw, segments);
    }

    public static long boundShapeCreatePrism(double cx, double cy, double cz, double radius, double halfHeight, int sides, double qi, double qj, double qk, double qw) {
        return RigidBodyNative.boundShapeCreatePrism(cx, cy, cz, radius, halfHeight, sides, qi, qj, qk, qw);
    }

    public static long boundShapeCreateCylinder(double cx, double cy, double cz, double radius, double halfHeight, double qi, double qj, double qk, double qw) {
        return RigidBodyNative.boundShapeCreateCylinder(cx, cy, cz, radius, halfHeight, qi, qj, qk, qw);
    }

    public static long boundShapeCreateSphericalShell(double cx, double cy, double cz, double innerRadius, double outerRadius) {
        return RigidBodyNative.boundShapeCreateSphericalShell(cx, cy, cz, innerRadius, outerRadius);
    }

    public static void boundShapeDestroy(long bound) {
        RigidBodyNative.boundShapeDestroy(bound);
    }

    public static int queryIntersectBoundShapeCount(long world, long bound, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody) {
        return RigidBodyNative.queryIntersectBoundShapeCount(world, bound, flags, memberships, filter, useGroups, excludeCollider, useExcludeCollider, excludeRigidBody, useExcludeRigidBody);
    }

    public static int queryIntersectBoundShape(long world, long bound, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody, long outHandles, int capacity) {
        return RigidBodyNative.queryIntersectBoundShape(world, bound, flags, memberships, filter, useGroups, excludeCollider, useExcludeCollider, excludeRigidBody, useExcludeRigidBody, outHandles, capacity);
    }

    public static long neuralBoundsCreate(double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, int sampleResolution, int hiddenWidth, int hiddenLayers, int activation, double outputScale, double padding, long weights, int weightCount) {
        return RigidBodyNative.neuralBoundsCreate(cx, cy, cz, hx, hy, hz, qi, qj, qk, qw, sampleResolution, hiddenWidth, hiddenLayers, activation, outputScale, padding, weights, weightCount);
    }

    public static void neuralBoundsDestroy(long neural) {
        RigidBodyNative.neuralBoundsDestroy(neural);
    }

    public static int queryIntersectNeuralBoundsHandleCount(long world, long neural, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody) {
        return RigidBodyNative.queryIntersectNeuralBoundsHandleCount(world, neural, flags, memberships, filter, useGroups, excludeCollider, useExcludeCollider, excludeRigidBody, useExcludeRigidBody);
    }

    public static int queryIntersectNeuralBoundsHandle(long world, long neural, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody, long outHandles, int capacity) {
        return RigidBodyNative.queryIntersectNeuralBoundsHandle(world, neural, flags, memberships, filter, useGroups, excludeCollider, useExcludeCollider, excludeRigidBody, useExcludeRigidBody, outHandles, capacity);
    }

    public static void worldDestroy(long world) {
        RigidBodyNative.worldDestroy(world);
    }

    public static void worldStep(long world, double deltaSeconds) {
        RigidBodyNative.worldStep(world, deltaSeconds);
    }

    public static void worldSetGravity(long world, double gravityX, double gravityY, double gravityZ) {
        RigidBodyNative.worldSetGravity(world, gravityX, gravityY, gravityZ);
    }

    public static double[] worldGetGravity(long world) {
        return RigidBodyNative.worldGetGravity(world);
    }

    public static double worldGetGravityX(long world) {
        return RigidBodyNative.worldGetGravityX(world);
    }

    public static double worldGetGravityY(long world) {
        return RigidBodyNative.worldGetGravityY(world);
    }

    public static double worldGetGravityZ(long world) {
        return RigidBodyNative.worldGetGravityZ(world);
    }

    public static int worldDynamicBodySnapshotCount(long world) {
        return RigidBodyNative.worldDynamicBodySnapshotCount(world);
    }

    public static long rigidBodyBuilderCreate(int status) {
        return RigidBodyNative.rigidBodyBuilderCreate(status);
    }

    public static void rigidBodyBuilderDestroy(long builder) {
        RigidBodyNative.rigidBodyBuilderDestroy(builder);
    }

    public static void rigidBodyBuilderSetTranslation(long builder, double x, double y, double z) {
        RigidBodyNative.rigidBodyBuilderSetTranslation(builder, x, y, z);
    }

    public static long worldInsertRigidBody(long world, long builder) {
        return RigidBodyNative.worldInsertRigidBody(world, builder);
    }

    public static double[] rigidBodyGetTranslation(long world, long body) {
        return RigidBodyNative.rigidBodyGetTranslation(world, body);
    }

    public static double rigidBodyGetTranslationX(long world, long body) {
        return RigidBodyNative.rigidBodyGetTranslationX(world, body);
    }

    public static double rigidBodyGetTranslationY(long world, long body) {
        return RigidBodyNative.rigidBodyGetTranslationY(world, body);
    }

    public static double rigidBodyGetTranslationZ(long world, long body) {
        return RigidBodyNative.rigidBodyGetTranslationZ(world, body);
    }

    public static long crbTreeCreate() {
        return RigidBodyNative.crbTreeCreate();
    }

    public static void crbTreeDestroy(long tree) {
        RigidBodyNative.crbTreeDestroy(tree);
    }

    public static void crbTreeClear(long tree) {
        RigidBodyNative.crbTreeClear(tree);
    }

    public static int crbTreeLen(long tree) {
        return RigidBodyNative.crbTreeLen(tree);
    }

    public static int crbTreeStats(long tree, long outStats) {
        return RigidBodyNative.crbTreeStats(tree, outStats);
    }

    public static boolean crbTreeContains(long tree, long id) {
        return RigidBodyNative.crbTreeContains(tree, id);
    }

    public static int crbTreeContainsBatch(long tree, long ids, int count, long outValues) {
        return RigidBodyNative.crbTreeContainsBatch(tree, ids, count, outValues);
    }

    public static boolean crbTreeInsert(long tree, long id, double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        return RigidBodyNative.crbTreeInsert(tree, id, minX, minY, minZ, maxX, maxY, maxZ);
    }

    public static boolean crbTreeUpdate(long tree, long id, double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        return RigidBodyNative.crbTreeUpdate(tree, id, minX, minY, minZ, maxX, maxY, maxZ);
    }

    public static boolean crbTreeRemove(long tree, long id) {
        return RigidBodyNative.crbTreeRemove(tree, id);
    }

    public static int crbTreeUpdateBatch(long tree, long ids, long aabbs, int count) {
        return RigidBodyNative.crbTreeUpdateBatch(tree, ids, aabbs, count);
    }

    public static int crbTreeRemoveBatch(long tree, long ids, int count) {
        return RigidBodyNative.crbTreeRemoveBatch(tree, ids, count);
    }

    public static int rtreeInsertBatch(long tree, long ids, long aabbs, int count) {
        return RigidBodyNative.rtreeInsertBatch(tree, ids, aabbs, count);
    }

    public static int rtreeUpdateBatch(long tree, long ids, long aabbs, int count) {
        return RigidBodyNative.rtreeUpdateBatch(tree, ids, aabbs, count);
    }

    public static int rtreeRemoveBatch(long tree, long ids, int count) {
        return RigidBodyNative.rtreeRemoveBatch(tree, ids, count);
    }

    public static int crbTreeQueryAabbCount(long tree, double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        return RigidBodyNative.crbTreeQueryAabbCount(tree, minX, minY, minZ, maxX, maxY, maxZ);
    }

    public static int crbTreeQueryAabb(long tree, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, long outIds, int capacity) {
        return RigidBodyNative.crbTreeQueryAabb(tree, minX, minY, minZ, maxX, maxY, maxZ, outIds, capacity);
    }

    public static int crbTreeQueryAabbCounts(long tree, long aabbs, int count, long outCounts) {
        return RigidBodyNative.crbTreeQueryAabbCounts(tree, aabbs, count, outCounts);
    }

    public static int crbTreeQueryAabbs(long tree, long aabbs, int count, long outOffsets, long outIds, int idCapacity) {
        return RigidBodyNative.crbTreeQueryAabbs(tree, aabbs, count, outOffsets, outIds, idCapacity);
    }

    public static int crbTreeQueryAabbUnsorted(long tree, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, long outIds, int capacity) {
        return RigidBodyNative.crbTreeQueryAabbUnsorted(tree, minX, minY, minZ, maxX, maxY, maxZ, outIds, capacity);
    }

    public static boolean abiSupportsFfm() {
        return RigidBodyNative.abiSupportsFfm();
    }

    public static boolean abiSupportsJni() {
        return RigidBodyNative.abiSupportsJni();
    }

    public static int worldDynamicBodySnapshot(long world, long outHandles, long outValues, int capacity) {
        return RigidBodyNative.worldDynamicBodySnapshot(world, outHandles, outValues, capacity);
    }

    public static long colliderBuilderCreate(int shapeType, double a, double b, double c) {
        return RigidBodyNative.colliderBuilderCreate(shapeType, a, b, c);
    }

    public static long colliderBuilderCreateEx(int shapeType, double a, double b, double c, double d) {
        return RigidBodyNative.colliderBuilderCreateEx(shapeType, a, b, c, d);
    }

    public static long colliderBuilderCreateSphere(double x, double y, double z, double radius) {
        return RigidBodyNative.colliderBuilderCreateSphere(x, y, z, radius);
    }

    public static long colliderBuilderCreateObb(double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw) {
        return RigidBodyNative.colliderBuilderCreateObb(cx, cy, cz, hx, hy, hz, qi, qj, qk, qw);
    }

    public static long colliderBuilderCreateConvexHull(long pointsXyz, int pointCount) {
        return RigidBodyNative.colliderBuilderCreateConvexHull(pointsXyz, pointCount);
    }

    public static void colliderBuilderDestroy(long builder) {
        RigidBodyNative.colliderBuilderDestroy(builder);
    }

    public static void colliderBuilderSetTranslation(long builder, double x, double y, double z) {
        RigidBodyNative.colliderBuilderSetTranslation(builder, x, y, z);
    }

    public static void colliderBuilderSetRotation(long builder, double x, double y, double z) {
        RigidBodyNative.colliderBuilderSetRotation(builder, x, y, z);
    }

    public static void colliderBuilderSetPose(long builder, double x, double y, double z, double qi, double qj, double qk, double qw) {
        RigidBodyNative.colliderBuilderSetPose(builder, x, y, z, qi, qj, qk, qw);
    }

    public static void colliderBuilderSetSensor(long builder, int sensor) {
        RigidBodyNative.colliderBuilderSetSensor(builder, sensor);
    }

    public static void colliderBuilderSetFriction(long builder, double friction) {
        RigidBodyNative.colliderBuilderSetFriction(builder, friction);
    }

    public static void colliderBuilderSetRestitution(long builder, double restitution) {
        RigidBodyNative.colliderBuilderSetRestitution(builder, restitution);
    }

    public static void colliderBuilderSetDensity(long builder, double density) {
        RigidBodyNative.colliderBuilderSetDensity(builder, density);
    }

    public static void colliderBuilderSetCollisionGroups(long builder, int memberships, int filter) {
        RigidBodyNative.colliderBuilderSetCollisionGroups(builder, memberships, filter);
    }

    public static void colliderBuilderSetSolverGroups(long builder, int memberships, int filter) {
        RigidBodyNative.colliderBuilderSetSolverGroups(builder, memberships, filter);
    }

    public static void colliderBuilderSetActiveEvents(long builder, int bits) {
        RigidBodyNative.colliderBuilderSetActiveEvents(builder, bits);
    }

    public static void colliderBuilderSetActiveHooks(long builder, int bits) {
        RigidBodyNative.colliderBuilderSetActiveHooks(builder, bits);
    }

    public static void colliderBuilderSetContactForceEventThreshold(long builder, double threshold) {
        RigidBodyNative.colliderBuilderSetContactForceEventThreshold(builder, threshold);
    }

    public static long worldInsertCollider(long world, long builder) {
        return RigidBodyNative.worldInsertCollider(world, builder);
    }

    public static long worldInsertColliderWithParent(long world, long builder, long parent) {
        return RigidBodyNative.worldInsertColliderWithParent(world, builder, parent);
    }

    public static boolean worldRemoveCollider(long world, long handle, int wakeUp) {
        return RigidBodyNative.worldRemoveCollider(world, handle, wakeUp);
    }

    public static boolean colliderSetPose(long world, long handle, double x, double y, double z, double qi, double qj, double qk, double qw) {
        return RigidBodyNative.colliderSetPose(world, handle, x, y, z, qi, qj, qk, qw);
    }

    public static boolean colliderSetSensor(long world, long handle, int sensor) {
        return RigidBodyNative.colliderSetSensor(world, handle, sensor);
    }

    public static boolean colliderSetFriction(long world, long handle, double friction) {
        return RigidBodyNative.colliderSetFriction(world, handle, friction);
    }

    public static boolean colliderSetRestitution(long world, long handle, double restitution) {
        return RigidBodyNative.colliderSetRestitution(world, handle, restitution);
    }

    public static boolean colliderSetCollisionGroups(long world, long handle, int memberships, int filter) {
        return RigidBodyNative.colliderSetCollisionGroups(world, handle, memberships, filter);
    }

    public static boolean colliderSetSolverGroups(long world, long handle, int memberships, int filter) {
        return RigidBodyNative.colliderSetSolverGroups(world, handle, memberships, filter);
    }

    public static boolean colliderSetActiveEvents(long world, long handle, int bits) {
        return RigidBodyNative.colliderSetActiveEvents(world, handle, bits);
    }

    public static boolean colliderSetActiveHooks(long world, long handle, int bits) {
        return RigidBodyNative.colliderSetActiveHooks(world, handle, bits);
    }

    public static boolean colliderSetContactForceEventThreshold(long world, long handle, double threshold) {
        return RigidBodyNative.colliderSetContactForceEventThreshold(world, handle, threshold);
    }

    public static double colliderGetDensity(long world, long handle) {
        return RigidBodyNative.colliderGetDensity(world, handle);
    }

    public static void rigidBodyBuilderSetRotation(long builder, double x, double y, double z) {
        RigidBodyNative.rigidBodyBuilderSetRotation(builder, x, y, z);
    }

    public static void rigidBodyBuilderSetPose(long builder, double x, double y, double z, double qi, double qj, double qk, double qw) {
        RigidBodyNative.rigidBodyBuilderSetPose(builder, x, y, z, qi, qj, qk, qw);
    }

    public static void rigidBodyBuilderSetLinvel(long builder, double x, double y, double z) {
        RigidBodyNative.rigidBodyBuilderSetLinvel(builder, x, y, z);
    }

    public static void rigidBodyBuilderSetAngvel(long builder, double x, double y, double z) {
        RigidBodyNative.rigidBodyBuilderSetAngvel(builder, x, y, z);
    }

    public static void rigidBodyBuilderSetGravityScale(long builder, double value) {
        RigidBodyNative.rigidBodyBuilderSetGravityScale(builder, value);
    }

    public static void rigidBodyBuilderSetLinearDamping(long builder, double value) {
        RigidBodyNative.rigidBodyBuilderSetLinearDamping(builder, value);
    }

    public static void rigidBodyBuilderSetAngularDamping(long builder, double value) {
        RigidBodyNative.rigidBodyBuilderSetAngularDamping(builder, value);
    }

    public static void rigidBodyBuilderSetCanSleep(long builder, int value) {
        RigidBodyNative.rigidBodyBuilderSetCanSleep(builder, value);
    }

    public static void rigidBodyBuilderSetEnabledRotations(long builder, int x, int y, int z) {
        RigidBodyNative.rigidBodyBuilderSetEnabledRotations(builder, x, y, z);
    }

    public static void rigidBodyBuilderSetUserData(long builder, long low, long high) {
        RigidBodyNative.rigidBodyBuilderSetUserData(builder, low, high);
    }

    public static void rigidBodyBuilderSetAdditionalMass(long builder, double mass) {
        RigidBodyNative.rigidBodyBuilderSetAdditionalMass(builder, mass);
    }

    public static boolean worldRemoveRigidBody(long world, long handle, int removeAttachedColliders) {
        return RigidBodyNative.worldRemoveRigidBody(world, handle, removeAttachedColliders);
    }

    public static int rigidBodyGetStatus(long world, long handle) {
        return RigidBodyNative.rigidBodyGetStatus(world, handle);
    }

    public static boolean rigidBodySetStatus(long world, long handle, int status, int wakeUp) {
        return RigidBodyNative.rigidBodySetStatus(world, handle, status, wakeUp);
    }

    public static boolean rigidBodySetPose(long world, long body, double x, double y, double z, double qi, double qj, double qk, double qw, int wakeUp) {
        return RigidBodyNative.rigidBodySetPose(world, body, x, y, z, qi, qj, qk, qw, wakeUp);
    }

    public static boolean rigidBodySetLinvel(long world, long body, double x, double y, double z, int wakeUp) {
        return RigidBodyNative.rigidBodySetLinvel(world, body, x, y, z, wakeUp);
    }

    public static boolean rigidBodySetAngvel(long world, long body, double x, double y, double z, int wakeUp) {
        return RigidBodyNative.rigidBodySetAngvel(world, body, x, y, z, wakeUp);
    }

    public static boolean rigidBodyAddForce(long world, long body, double x, double y, double z, int wakeUp) {
        return RigidBodyNative.rigidBodyAddForce(world, body, x, y, z, wakeUp);
    }

    public static boolean rigidBodyAddTorque(long world, long body, double x, double y, double z, int wakeUp) {
        return RigidBodyNative.rigidBodyAddTorque(world, body, x, y, z, wakeUp);
    }

    public static boolean rigidBodyApplyImpulse(long world, long body, double x, double y, double z, int wakeUp) {
        return RigidBodyNative.rigidBodyApplyImpulse(world, body, x, y, z, wakeUp);
    }

    public static boolean rigidBodyApplyTorqueImpulse(long world, long body, double x, double y, double z, int wakeUp) {
        return RigidBodyNative.rigidBodyApplyTorqueImpulse(world, body, x, y, z, wakeUp);
    }

    public static boolean rigidBodyEnableCcd(long world, long body, int enabled) {
        return RigidBodyNative.rigidBodyEnableCcd(world, body, enabled);
    }

    public static boolean rigidBodySleep(long world, long body) {
        return RigidBodyNative.rigidBodySleep(world, body);
    }

    public static boolean rigidBodyWakeUp(long world, long body, int strong) {
        return RigidBodyNative.rigidBodyWakeUp(world, body, strong);
    }

    public static boolean rigidBodyIsSleeping(long world, long body) {
        return RigidBodyNative.rigidBodyIsSleeping(world, body);
    }

    public static long colliderBuilderCreateCapsule(double ax, double ay, double az, double bx, double by, double bz, double radius) {
        return RigidBodyNative.colliderBuilderCreateCapsule(ax, ay, az, bx, by, bz, radius);
    }

    public static long colliderBuilderCreateSsv(double ax, double ay, double az, double bx, double by, double bz, double radius) {
        return RigidBodyNative.colliderBuilderCreateSsv(ax, ay, az, bx, by, bz, radius);
    }

    public static long colliderBuilderCreateEllipsoid(double cx, double cy, double cz, double rx, double ry, double rz, double qi, double qj, double qk, double qw, int segments) {
        return RigidBodyNative.colliderBuilderCreateEllipsoid(cx, cy, cz, rx, ry, rz, qi, qj, qk, qw, segments);
    }

    public static long colliderBuilderCreatePrism(double cx, double cy, double cz, double radius, double halfHeight, int sides, double qi, double qj, double qk, double qw) {
        return RigidBodyNative.colliderBuilderCreatePrism(cx, cy, cz, radius, halfHeight, sides, qi, qj, qk, qw);
    }

    public static long colliderBuilderCreateCylinder(double cx, double cy, double cz, double radius, double halfHeight, double qi, double qj, double qk, double qw) {
        return RigidBodyNative.colliderBuilderCreateCylinder(cx, cy, cz, radius, halfHeight, qi, qj, qk, qw);
    }

    public static long colliderBuilderCreateSphericalShell(double cx, double cy, double cz, double innerRadius, double outerRadius) {
        return RigidBodyNative.colliderBuilderCreateSphericalShell(cx, cy, cz, innerRadius, outerRadius);
    }

    public static long queryCastRay(long world, double ox, double oy, double oz, double dx, double dy, double dz, double maxToi, int solid, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody, long outHit) {
        return RigidBodyNative.queryCastRay(world, ox, oy, oz, dx, dy, dz, maxToi, solid, flags, memberships, filter, useGroups, excludeCollider, useExcludeCollider, excludeRigidBody, useExcludeRigidBody, outHit);
    }

    public static int queryIntersectAabbCount(long world, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody) {
        return RigidBodyNative.queryIntersectAabbCount(world, minX, minY, minZ, maxX, maxY, maxZ, flags, memberships, filter, useGroups, excludeCollider, useExcludeCollider, excludeRigidBody, useExcludeRigidBody);
    }

    public static int queryIntersectObb(long world, double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody, long outHandles, int capacity) {
        return RigidBodyNative.queryIntersectObb(world, cx, cy, cz, hx, hy, hz, qi, qj, qk, qw, flags, memberships, filter, useGroups, excludeCollider, useExcludeCollider, excludeRigidBody, useExcludeRigidBody, outHandles, capacity);
    }

    public static int queryIntersectSphere(long world, double cx, double cy, double cz, double radius, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody, long outHandles, int capacity) {
        return RigidBodyNative.queryIntersectSphere(world, cx, cy, cz, radius, flags, memberships, filter, useGroups, excludeCollider, useExcludeCollider, excludeRigidBody, useExcludeRigidBody, outHandles, capacity);
    }

    public static long queryCastShape(long world, int shapeType, double a, double b, double c, double d, double tx, double ty, double tz, double qi, double qj, double qk, double qw, double vx, double vy, double vz, double maxToi, double targetDistance, int stopAtPenetration, int computeImpactGeometryOnPenetration, int flags, int memberships, int filter, int useGroups, long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody, long outHit) {
        return RigidBodyNative.queryCastShape(world, shapeType, a, b, c, d, tx, ty, tz, qi, qj, qk, qw, vx, vy, vz, maxToi, targetDistance, stopAtPenetration, computeImpactGeometryOnPenetration, flags, memberships, filter, useGroups, excludeCollider, useExcludeCollider, excludeRigidBody, useExcludeRigidBody, outHit);
    }

    public static long colliderBuilderCreateKdop(long pointsXyz, int pointCount, int preset) {
        return RigidBodyNative.colliderBuilderCreateKdop(pointsXyz, pointCount, preset);
    }

    public static long colliderBuilderCreateFdh(long pointsXyz, int pointCount, long directionsXyz, int directionCount) {
        return RigidBodyNative.colliderBuilderCreateFdh(pointsXyz, pointCount, directionsXyz, directionCount);
    }

    public static int neuralBoundsRequiredWeightCount(int hiddenWidth, int hiddenLayers) {
        return RigidBodyNative.neuralBoundsRequiredWeightCount(hiddenWidth, hiddenLayers);
    }

    public static long colliderBuilderCreateNeuralBounds(double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, int sampleResolution, int hiddenWidth, int hiddenLayers, int activation, double outputScale, double padding, long weights, int weightCount) {
        return RigidBodyNative.colliderBuilderCreateNeuralBounds(cx, cy, cz, hx, hy, hz, qi, qj, qk, qw, sampleResolution, hiddenWidth, hiddenLayers, activation, outputScale, padding, weights, weightCount);
    }

    public static long colliderBuilderCreateVoxels(long voxels, int sizeX, int sizeY, int sizeZ, double voxelSize, double originX, double originY, double originZ, int mode, int dynamicBody, int smallVoxelLimit, int meshVoxelLimit) {
        return RigidBodyNative.colliderBuilderCreateVoxels(voxels, sizeX, sizeY, sizeZ, voxelSize, originX, originY, originZ, mode, dynamicBody, smallVoxelLimit, meshVoxelLimit);
    }

    public static int worldInsertUrdfFromBytesDefault(long world, long urdfBytes, int urdfLen, long outBodyHandles, int bodyCapacity, long outResult) {
        return RigidBodyNative.worldInsertUrdfFromBytesDefault(world, urdfBytes, urdfLen, outBodyHandles, bodyCapacity, outResult);
    }

    public static int worldInsertUrdfFromBytesDefaultEx(long world, long urdfBytes, int urdfLen, long outBodyHandles, int bodyCapacity, long outColliderHandles, int colliderCapacity, long outJointHandles, int jointCapacity, long outResult) {
        return RigidBodyNative.worldInsertUrdfFromBytesDefaultEx(world, urdfBytes, urdfLen, outBodyHandles, bodyCapacity, outColliderHandles, colliderCapacity, outJointHandles, jointCapacity, outResult);
    }

    public static int worldInsertUrdfFromBytes(long world, long urdfBytes, int urdfLen, int createCollisionColliders, int createVisualColliders, int makeRootsFixed, double scale, double density, double friction, double restitution, long outBodyHandles, int bodyCapacity, long outResult) {
        return RigidBodyNative.worldInsertUrdfFromBytes(world, urdfBytes, urdfLen, createCollisionColliders, createVisualColliders, makeRootsFixed, scale, density, friction, restitution, outBodyHandles, bodyCapacity, outResult);
    }

    public static int worldInsertUrdfFromBytesEx(long world, long urdfBytes, int urdfLen, int createCollisionColliders, int createVisualColliders, int makeRootsFixed, double scale, double density, double friction, double restitution, long outBodyHandles, int bodyCapacity, long outColliderHandles, int colliderCapacity, long outJointHandles, int jointCapacity, long outResult) {
        return RigidBodyNative.worldInsertUrdfFromBytesEx(world, urdfBytes, urdfLen, createCollisionColliders, createVisualColliders, makeRootsFixed, scale, density, friction, restitution, outBodyHandles, bodyCapacity, outColliderHandles, colliderCapacity, outJointHandles, jointCapacity, outResult);
    }

    public static int worldInsertUrdfFromBytesCount(long world, long urdfBytes, int urdfLen) {
        return RigidBodyNative.worldInsertUrdfFromBytesCount(world, urdfBytes, urdfLen);
    }

    public static int worldInsertMjcfFromBytesDefault(long world, long mjcfBytes, int mjcfLen, long outBodyHandles, int bodyCapacity, long outResult) {
        return RigidBodyNative.worldInsertMjcfFromBytesDefault(world, mjcfBytes, mjcfLen, outBodyHandles, bodyCapacity, outResult);
    }

    public static int worldInsertMjcfFromBytesDefaultEx(long world, long mjcfBytes, int mjcfLen, long outBodyHandles, int bodyCapacity, long outColliderHandles, int colliderCapacity, long outJointHandles, int jointCapacity, long outResult) {
        return RigidBodyNative.worldInsertMjcfFromBytesDefaultEx(world, mjcfBytes, mjcfLen, outBodyHandles, bodyCapacity, outColliderHandles, colliderCapacity, outJointHandles, jointCapacity, outResult);
    }

    public static int worldInsertMjcfFromBytes(long world, long mjcfBytes, int mjcfLen, int makeRootsFixed, double scale, double density, double friction, double restitution, long outBodyHandles, int bodyCapacity, long outResult) {
        return RigidBodyNative.worldInsertMjcfFromBytes(world, mjcfBytes, mjcfLen, makeRootsFixed, scale, density, friction, restitution, outBodyHandles, bodyCapacity, outResult);
    }

    public static int worldInsertMjcfFromBytesEx(long world, long mjcfBytes, int mjcfLen, int makeRootsFixed, double scale, double density, double friction, double restitution, long outBodyHandles, int bodyCapacity, long outColliderHandles, int colliderCapacity, long outJointHandles, int jointCapacity, long outResult) {
        return RigidBodyNative.worldInsertMjcfFromBytesEx(world, mjcfBytes, mjcfLen, makeRootsFixed, scale, density, friction, restitution, outBodyHandles, bodyCapacity, outColliderHandles, colliderCapacity, outJointHandles, jointCapacity, outResult);
    }

    public static int worldInsertMjcfFromBytesCount(long world, long mjcfBytes, int mjcfLen) {
        return RigidBodyNative.worldInsertMjcfFromBytesCount(world, mjcfBytes, mjcfLen);
    }

    public static long worldInsertDynamicCuboids(long world, double x, double y, double z, double qi, double qj, double qk, double qw, double lvx, double lvy, double lvz, long cuboids, int cuboidCount, double density, double friction, double restitution, int collisionMemberships, int collisionFilter, int solverMemberships, int solverFilter) {
        return RigidBodyNative.worldInsertDynamicCuboids(world, x, y, z, qi, qj, qk, qw, lvx, lvy, lvz, cuboids, cuboidCount, density, friction, restitution, collisionMemberships, collisionFilter, solverMemberships, solverFilter);
    }

    public static long worldInsertStaticTrimesh(long world, long verticesXyz, int vertexXyzLen, long indices, int indexLen, double friction, double restitution) {
        return RigidBodyNative.worldInsertStaticTrimesh(world, verticesXyz, vertexXyzLen, indices, indexLen, friction, restitution);
    }

    public static long jointBuilderCreate(int jointType, double ax, double ay, double az, double b, double c) {
        return RigidBodyNative.jointBuilderCreate(jointType, ax, ay, az, b, c);
    }

    public static void jointBuilderDestroy(long builder) {
        RigidBodyNative.jointBuilderDestroy(builder);
    }

    public static void jointBuilderSetContactsEnabled(long builder, int enabled) {
        RigidBodyNative.jointBuilderSetContactsEnabled(builder, enabled);
    }

    public static void jointBuilderSetLocalAnchor1(long builder, double x, double y, double z) {
        RigidBodyNative.jointBuilderSetLocalAnchor1(builder, x, y, z);
    }

    public static void jointBuilderSetLocalAnchor2(long builder, double x, double y, double z) {
        RigidBodyNative.jointBuilderSetLocalAnchor2(builder, x, y, z);
    }

    public static void jointBuilderSetLimits(long builder, int axis, double min, double max) {
        RigidBodyNative.jointBuilderSetLimits(builder, axis, min, max);
    }

    public static void jointBuilderSetMotorVelocity(long builder, int axis, double targetVel, double factor) {
        RigidBodyNative.jointBuilderSetMotorVelocity(builder, axis, targetVel, factor);
    }

    public static void jointBuilderSetMotorPosition(long builder, int axis, double targetPos, double stiffness, double damping) {
        RigidBodyNative.jointBuilderSetMotorPosition(builder, axis, targetPos, stiffness, damping);
    }

    public static long worldInsertImpulseJoint(long world, long body1, long body2, long builder, int wakeUp) {
        return RigidBodyNative.worldInsertImpulseJoint(world, body1, body2, builder, wakeUp);
    }

    public static boolean worldRemoveImpulseJoint(long world, long handle, int wakeUp) {
        return RigidBodyNative.worldRemoveImpulseJoint(world, handle, wakeUp);
    }

    public static long characterControllerCreate() {
        return RigidBodyNative.characterControllerCreate();
    }

    public static void characterControllerDestroy(long controller) {
        RigidBodyNative.characterControllerDestroy(controller);
    }

    public static void characterControllerSetUp(long controller, double x, double y, double z) {
        RigidBodyNative.characterControllerSetUp(controller, x, y, z);
    }

    public static void characterControllerSetOffsetAbsolute(long controller, double offset) {
        RigidBodyNative.characterControllerSetOffsetAbsolute(controller, offset);
    }

    public static void characterControllerSetOffsetRelative(long controller, double offset) {
        RigidBodyNative.characterControllerSetOffsetRelative(controller, offset);
    }

    public static void characterControllerSetSlide(long controller, int slide) {
        RigidBodyNative.characterControllerSetSlide(controller, slide);
    }

    public static void characterControllerSetAutostep(long controller, int enabled, double maxHeight, double minWidth, int includeDynamicBodies) {
        RigidBodyNative.characterControllerSetAutostep(controller, enabled, maxHeight, minWidth, includeDynamicBodies);
    }

    public static void characterControllerSetSnapToGround(long controller, int enabled, double distance) {
        RigidBodyNative.characterControllerSetSnapToGround(controller, enabled, distance);
    }

    public static void characterControllerSetSlopeAngles(long controller, double maxClimbAngle, double minSlideAngle) {
        RigidBodyNative.characterControllerSetSlopeAngles(controller, maxClimbAngle, minSlideAngle);
    }

    public static boolean characterControllerMoveShape(long world, long controller, double dt, int shapeType, double a, double b, double c, double d, double tx, double ty, double tz, double qi, double qj, double qk, double qw, double dx, double dy, double dz, long outMovement) {
        return RigidBodyNative.characterControllerMoveShape(world, controller, dt, shapeType, a, b, c, d, tx, ty, tz, qi, qj, qk, qw, dx, dy, dz, outMovement);
    }

    public static int characterControllerCollisionCount(long controller) {
        return RigidBodyNative.characterControllerCollisionCount(controller);
    }

    public static long characterControllerGetCollision(long controller, int index, long outCollision) {
        return RigidBodyNative.characterControllerGetCollision(controller, index, outCollision);
    }

    public static boolean characterControllerSolveImpulses(long world, long controller, double dt, int shapeType, double a, double b, double c, double d, double characterMass) {
        return RigidBodyNative.characterControllerSolveImpulses(world, controller, dt, shapeType, a, b, c, d, characterMass);
    }

    public static void worldClearEvents(long world) {
        RigidBodyNative.worldClearEvents(world);
    }

    public static int worldCollisionEventCount(long world) {
        return RigidBodyNative.worldCollisionEventCount(world);
    }

    public static long worldGetCollisionEvent(long world, int index, long outEvent) {
        return RigidBodyNative.worldGetCollisionEvent(world, index, outEvent);
    }

    public static int worldDrainCollisionEvents(long world, long outEvents, int capacity) {
        return RigidBodyNative.worldDrainCollisionEvents(world, outEvents, capacity);
    }

    public static int worldContactForceEventCount(long world) {
        return RigidBodyNative.worldContactForceEventCount(world);
    }

    public static long worldGetContactForceEvent(long world, int index, long outEvent) {
        return RigidBodyNative.worldGetContactForceEvent(world, index, outEvent);
    }

    public static int worldDrainContactForceEvents(long world, long outEvents, int capacity) {
        return RigidBodyNative.worldDrainContactForceEvents(world, outEvents, capacity);
    }

    public static void worldSetContactPairFilterCallback(long world, long callback, long userData) {
        RigidBodyNative.worldSetContactPairFilterCallback(world, callback, userData);
    }

    public static void worldSetIntersectionPairFilterCallback(long world, long callback, long userData) {
        RigidBodyNative.worldSetIntersectionPairFilterCallback(world, callback, userData);
    }

    public static void worldClearContactPairFilterCallback(long world) {
        RigidBodyNative.worldClearContactPairFilterCallback(world);
    }

    public static void worldClearIntersectionPairFilterCallback(long world) {
        RigidBodyNative.worldClearIntersectionPairFilterCallback(world);
    }

    public static long rtreeCreate() {
        return RigidBodyNative.rtreeCreate();
    }

    public static void rtreeDestroy(long tree) {
        RigidBodyNative.rtreeDestroy(tree);
    }

    public static void rtreeClear(long tree) {
        RigidBodyNative.rtreeClear(tree);
    }

    public static int rtreeLen(long tree) {
        return RigidBodyNative.rtreeLen(tree);
    }

    public static int rtreeNodeCount(long tree) {
        return RigidBodyNative.rtreeNodeCount(tree);
    }

    public static int rtreeHeight(long tree) {
        return RigidBodyNative.rtreeHeight(tree);
    }

    public static boolean rtreeIsDirty(long tree) {
        return RigidBodyNative.rtreeIsDirty(tree);
    }

    public static int rtreeStats(long tree, long outStats) {
        return RigidBodyNative.rtreeStats(tree, outStats);
    }

    public static boolean rtreeContains(long tree, long id) {
        return RigidBodyNative.rtreeContains(tree, id);
    }

    public static int rtreeContainsBatch(long tree, long ids, int count, long outValues) {
        return RigidBodyNative.rtreeContainsBatch(tree, ids, count, outValues);
    }

    public static boolean rtreeInsert(long tree, long id, double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        return RigidBodyNative.rtreeInsert(tree, id, minX, minY, minZ, maxX, maxY, maxZ);
    }

    public static boolean rtreeUpdate(long tree, long id, double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        return RigidBodyNative.rtreeUpdate(tree, id, minX, minY, minZ, maxX, maxY, maxZ);
    }

    public static boolean rtreeRemove(long tree, long id) {
        return RigidBodyNative.rtreeRemove(tree, id);
    }

    public static void rtreeRebuild(long tree) {
        RigidBodyNative.rtreeRebuild(tree);
    }

    public static int rtreeQueryAabbCount(long tree, double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        return RigidBodyNative.rtreeQueryAabbCount(tree, minX, minY, minZ, maxX, maxY, maxZ);
    }

    public static int rtreeQueryAabbCounts(long tree, long aabbs, int count, long outCounts) {
        return RigidBodyNative.rtreeQueryAabbCounts(tree, aabbs, count, outCounts);
    }

    public static int rtreeQueryAabbs(long tree, long aabbs, int count, long outOffsets, long outIds, int idCapacity) {
        return RigidBodyNative.rtreeQueryAabbs(tree, aabbs, count, outOffsets, outIds, idCapacity);
    }

    public static int rtreeQueryAabb(long tree, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, long outIds, int capacity) {
        return RigidBodyNative.rtreeQueryAabb(tree, minX, minY, minZ, maxX, maxY, maxZ, outIds, capacity);
    }
}
