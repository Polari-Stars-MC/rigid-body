package org.polaris2023.msp_rigid_body;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;

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
            System.loadLibrary("mps_rigid_body");
            return;
        } catch (UnsatisfiedLinkError loadLibraryError) {
            String mappedName = System.mapLibraryName("mps_rigid_body");
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

            if (loadBundledLibrary(mappedName)) {
                return;
            }

            UnsatisfiedLinkError error = new UnsatisfiedLinkError(
                    loadLibraryError.getMessage()
                            + "; also tried -Drigidbody.native.path, target/release/, and bundled native/"
                            + mappedName);
            error.initCause(loadLibraryError);
            throw error;
        }
    }

    private static boolean loadBundledLibrary(String mappedName) {
        String[] resourcePaths = {
                "/native/" + nativeOs() + "/" + nativeArch() + "/" + mappedName,
                "/native/" + mappedName
        };
        for (String resourcePath : resourcePaths) {
            if (loadBundledLibrary(resourcePath, mappedName)) {
                return true;
            }
        }
        return false;
    }

    private static boolean loadBundledLibrary(String resourcePath, String mappedName) {
        try (var input = RigidBodyNative.class.getResourceAsStream(resourcePath)) {
            if (input == null) {
                return false;
            }
            Path directory = Files.createTempDirectory("mps_rigid_body_native_");
            Path library = directory.resolve(mappedName);
            Files.copy(input, library, StandardCopyOption.REPLACE_EXISTING);
            library.toFile().deleteOnExit();
            directory.toFile().deleteOnExit();
            System.load(library.toAbsolutePath().normalize().toString());
            return true;
        } catch (Exception exception) {
            UnsatisfiedLinkError error = new UnsatisfiedLinkError("failed to load bundled native library " + resourcePath);
            error.initCause(exception);
            throw error;
        }
    }

    private static String nativeOs() {
        String os = System.getProperty("os.name", "").toLowerCase(java.util.Locale.ROOT);
        if (os.contains("win")) {
            return "windows";
        }
        if (os.contains("mac") || os.contains("darwin")) {
            return "macos";
        }
        return "linux";
    }

    private static String nativeArch() {
        String arch = System.getProperty("os.arch", "").toLowerCase(java.util.Locale.ROOT);
        if (arch.equals("amd64") || arch.equals("x86_64")) {
            return "x86_64";
        }
        if (arch.equals("aarch64") || arch.equals("arm64")) {
            return "aarch64";
        }
        return arch;
    }

    public static native int abiVersion();
    public static native boolean abiSupportsFfm();
    public static native boolean abiSupportsJni();
    public static native int abiLastErrorCode();
    public static native String abiLastErrorMessage();
    public static native void abiClearLastError();

    public static native long worldCreate(double gravityX, double gravityY, double gravityZ);
    public static native void worldDestroy(long world);
    public static native void worldStep(long world, double deltaSeconds);
    public static native void worldSetGravity(long world, double x, double y, double z);
    public static native double[] worldGetGravity(long world);
    public static native void worldGetGravityOut(long world, long outGravity);
    public static native int worldGetRigidBodySetSize(long world);
    public static native int worldGetColliderSetSize(long world);
    public static native int worldDynamicBodySnapshotCount(long world);
    public static native int worldDynamicBodySnapshot(long world, long outHandles, long outValues, int capacity);
    public static native boolean worldSetIntegrationParameters(long world, double dt, int solverIterations, int ccdSubsteps);
    public static native int worldGetIntegrationParameters(long world, long outValues, int capacity);
    public static native int worldBodySnapshotCount(long world);
    public static native int worldBodySnapshot(long world, long outHandles, long outValues, int capacity);
    public static native int worldUpdateBodyPoses(long world, long handles, long values, int count, int wakeUp);
    public static native int worldUpdateBodyVelocities(long world, long handles, long values, int count, int wakeUp);

    /*
     * Ownership rules for raw pointers:
     * - *BuilderBuild consumes the builder pointer; do not call *BuilderDestroy after a successful build.
     * - worldInsertRigidBody/worldInsertCollider consume the raw built object pointer.
     * - worldCopyRigidBody/worldCopyCollider return raw heap objects; release unused copies with *DestroyRaw.
     * - worldInsertImpulseJoint consumes the joint builder pointer.
     */
    public static native long worldInsertRigidBody(long world, long memoryHandle);
    public static native boolean worldRemoveRigidBody(long world, long handle, int removeAttachedColliders);
    public static native long worldCopyRigidBody(long world, long handle);
    public static native void rigidBodyDestroyRaw(long rigidBody);
    public static native long worldInsertCollider(long world, long memoryHandle);
    public static native long worldInsertColliderWithParent(long world, long memoryHandle, long parent);
    public static native boolean worldRemoveCollider(long world, long handle, int wakeUp);
    public static native long worldCopyCollider(long world, long handle);
    public static native void colliderDestroyRaw(long collider);

    public static native long colliderBuilderCreate(int shapeType, double a, double b, double c);
    public static native long colliderBuilderCreateHeightmap(
            double[] data, int dataX, int dataY, double scaleX, double scaleY, double scaleZ);
    public static native long colliderBuilderCreateEx(int shapeType, double a, double b, double c, double d);
    public static native long colliderBuilderCreateSphere(double x, double y, double z, double radius);
    public static native long colliderBuilderCreateObb(
            double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw);
    public static native long colliderBuilderCreateConvexHull(long pointsXyz, int pointCount);
    public static native long colliderBuilderCreatePointCloudBounds(long pointsXyz, int pointCount);
    public static native long colliderBuilderCreateDoubleBv(
            double aMinX, double aMinY, double aMinZ, double aMaxX, double aMaxY, double aMaxZ,
            double bMinX, double bMinY, double bMinZ, double bMaxX, double bMaxY, double bMaxZ);
    public static native long colliderBuilderCreateSkewedObb(
            double cx, double cy, double cz,
            double axX, double axY, double axZ,
            double ayX, double ayY, double ayZ,
            double azX, double azY, double azZ);
    public static native long colliderBuilderCreateDiscreteObb(long pointsXyz, int pointCount, int axis);
    public static native long colliderBuilderCreateFusedCollapsingBounds(long pointsXyz, int pointCount, double padding);
    public static native long colliderBuilderCreateEdgeBvh(
            long verticesXyz, int vertexCount, long edges, int edgeCount, double radius);
    public static native long colliderBuilderCreateMedialSpheres(long spheresXyzw, int sphereCount);
    public static native long colliderBuilderCreateCapsule(
            double ax, double ay, double az, double bx, double by, double bz, double radius);
    public static native long colliderBuilderCreateSsv(
            double ax, double ay, double az, double bx, double by, double bz, double radius);
    public static native long colliderBuilderCreateEllipsoid(
            double cx, double cy, double cz, double rx, double ry, double rz,
            double qi, double qj, double qk, double qw, int segments);
    public static native long colliderBuilderCreatePrism(
            double cx, double cy, double cz, double radius, double halfHeight, int sides,
            double qi, double qj, double qk, double qw);
    public static native long colliderBuilderCreateCylinder(
            double cx, double cy, double cz, double radius, double halfHeight,
            double qi, double qj, double qk, double qw);
    public static native long colliderBuilderCreateSphericalShell(
            double cx, double cy, double cz, double innerRadius, double outerRadius);
    public static native long colliderBuilderCreateKdop(long pointsXyz, int pointCount, int preset);
    public static native long colliderBuilderCreateFdh(long pointsXyz, int pointCount, long directionsXyz, int directionCount);
    public static native long colliderBuilderCreateNeuralBounds(
            double cx, double cy, double cz, double hx, double hy, double hz,
            double qi, double qj, double qk, double qw,
            int sampleResolution, int hiddenWidth, int hiddenLayers, int activation,
            double outputScale, double padding, long weights, int weightCount);
    public static native long colliderBuilderCreateVoxels(
            long voxels, int sizeX, int sizeY, int sizeZ, double voxelSize,
            double originX, double originY, double originZ,
            int mode, int dynamicBody, int smallVoxelLimit, int meshVoxelLimit);
    public static native long colliderBuilderCreateVoxelsAuto(
            long voxels, int sizeX, int sizeY, int sizeZ, double voxelSize,
            double originX, double originY, double originZ, int dynamicBody);
    public static native long colliderBuilderCreateVoxelBytes(
            byte[] voxels, int sizeX, int sizeY, int sizeZ, double voxelSize,
            double originX, double originY, double originZ,
            int mode, int dynamicBody, int smallVoxelLimit, int meshVoxelLimit);
    public static native long colliderBuilderCreateVoxelBytesAuto(
            byte[] voxels, int sizeX, int sizeY, int sizeZ, double voxelSize,
            double originX, double originY, double originZ, int dynamicBody);
    public static native long colliderBuilderCreateVoxelAabb(
            double minX, double minY, double minZ, double maxX, double maxY, double maxZ,
            double voxelSize, int mode, int dynamicBody, int smallVoxelLimit, int meshVoxelLimit);
    public static native long colliderBuilderCreateVoxelAabbAuto(
            double minX, double minY, double minZ, double maxX, double maxY, double maxZ,
            double voxelSize, int dynamicBody);
    public static native long colliderBuilderCreateVoxelObb(
            double cx, double cy, double cz, double hx, double hy, double hz,
            double qi, double qj, double qk, double qw,
            double voxelSize, int mode, int dynamicBody, int smallVoxelLimit, int meshVoxelLimit);
    public static native long colliderBuilderCreateVoxelObbAuto(
            double cx, double cy, double cz, double hx, double hy, double hz,
            double qi, double qj, double qk, double qw,
            double voxelSize, int dynamicBody);
    public static native void voxelBuildStats(
            long voxels, int sizeX, int sizeY, int sizeZ, double voxelSize,
            double originX, double originY, double originZ,
            int mode, int dynamicBody, int smallVoxelLimit, int meshVoxelLimit, long outStats);
    public static native void voxelAabbBuildStats(
            double minX, double minY, double minZ, double maxX, double maxY, double maxZ,
            double voxelSize, int mode, int dynamicBody, int smallVoxelLimit, int meshVoxelLimit, long outStats);
    public static native void voxelObbBuildStats(
            double cx, double cy, double cz, double hx, double hy, double hz,
            double qi, double qj, double qk, double qw,
            double voxelSize, int mode, int dynamicBody, int smallVoxelLimit, int meshVoxelLimit, long outStats);
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
    public static native long colliderBuilderBuild(long builder);
    public static native void colliderBuilderDestroy(long builder);
    public static native double[] colliderGetTranslation(long world, long handle);
    public static native double[] colliderGetRotation(long world, long handle);
    public static native void colliderGetTranslationOut(long world, long handle, long outTranslation);
    public static native void colliderGetRotationOut(long world, long handle, long outRotation);
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

    public static native long rigidBodyBuilderCreate(int status);
    public static native void rigidBodyBuilderSetTranslation(long builder, double x, double y, double z);
    public static native void rigidBodyBuilderSetRotation(long builder, double x, double y, double z);
    public static native void rigidBodyBuilderSetPose(long builder, double x, double y, double z, double qi, double qj, double qk, double qw);
    public static native void rigidBodyBuilderSetAdditionalMassProperties(long builder, double cx, double cy, double cz, double mass, double lx, double ly, double lz);
    public static native void rigidBodyBuilderSetLinvel(long builder, double x, double y, double z);
    public static native void rigidBodyBuilderSetAngvel(long builder, double x, double y, double z);
    public static native void rigidBodyBuilderSetGravityScale(long builder, double value);
    public static native void rigidBodyBuilderSetLinearDamping(long builder, double value);
    public static native void rigidBodyBuilderSetAngularDamping(long builder, double value);
    public static native void rigidBodyBuilderSetCanSleep(long builder, int value);
    public static native void rigidBodyBuilderSetEnabledRotations(long builder, int x, int y, int z);
    public static native void rigidBodyBuilderSetUserData(long builder, long low, long high);
    public static native void rigidBodyBuilderSetAdditionalMass(long builder, double mass);
    public static native long rigidBodyBuilderBuild(long builder);
    public static native void rigidBodyBuilderDestroy(long builder);
    public static native int rigidBodyGetStatus(long world, long handle);
    public static native boolean rigidBodySetStatus(long world, long handle, int status, int wakeUp);
    public static native double[] rigidBodyGetTranslation(long world, long body);
    public static native double[] rigidBodyGetRotation(long world, long body);
    public static native void rigidBodyGetTranslationOut(long world, long body, long outTranslation);
    public static native void rigidBodyGetRotationOut(long world, long body, long outRotation);
    public static native boolean rigidBodySetPose(long world, long body, double x, double y, double z, double qi, double qj, double qk, double qw, int wakeUp);
    public static native boolean rigidBodySetTranslation(long world, long body, double x, double y, double z, int wakeUp);
    public static native boolean rigidBodySetRotation(long world, long body, double qi, double qj, double qk, double qw, int wakeUp);
    public static native double[] rigidBodyGetLinvel(long world, long body);
    public static native void rigidBodyGetLinvelOut(long world, long body, long outLinvel);
    public static native boolean rigidBodySetLinvel(long world, long body, double x, double y, double z, int wakeUp);
    public static native double[] rigidBodyGetAngvel(long world, long body);
    public static native void rigidBodyGetAngvelOut(long world, long body, long outAngvel);
    public static native boolean rigidBodySetAngvel(long world, long body, double x, double y, double z, int wakeUp);
    public static native boolean rigidBodyAddForce(long world, long body, double x, double y, double z, int wakeUp);
    public static native boolean rigidBodyAddTorque(long world, long body, double x, double y, double z, int wakeUp);
    public static native boolean rigidBodyApplyImpulse(long world, long body, double x, double y, double z, int wakeUp);
    public static native boolean rigidBodyApplyTorqueImpulse(long world, long body, double x, double y, double z, int wakeUp);
    public static native boolean rigidBodyEnableCcd(long world, long body, int enabled);
    public static native boolean rigidBodySleep(long world, long body);
    public static native boolean rigidBodyWakeUp(long world, long body, int strong);
    public static native boolean rigidBodyIsSleeping(long world, long body);

    public static native long queryCastRay(
            long world, double ox, double oy, double oz, double dx, double dy, double dz,
            double maxToi, int solid, int flags, int memberships, int filter, int useGroups,
            long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody,
            long outHit);
    public static native int queryCastRays(
            long world, long rays, int rayCount, double maxToi, int solid,
            int flags, int memberships, int filter, int useGroups,
            long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody,
            long outHits, int capacity);
    public static native long queryProjectPoint(
            long world, double x, double y, double z, double maxDist, int solid,
            int flags, int memberships, int filter, int useGroups,
            long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody,
            long outProjection);
    public static native int queryIntersectPointCount(
            long world, double x, double y, double z,
            int flags, int memberships, int filter, int useGroups,
            long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody);
    public static native int queryIntersectAabbCount(
            long world, double minX, double minY, double minZ, double maxX, double maxY, double maxZ,
            int flags, int memberships, int filter, int useGroups,
            long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody);
    public static native int queryIntersectObb(
            long world, double cx, double cy, double cz, double hx, double hy, double hz,
            double qi, double qj, double qk, double qw,
            int flags, int memberships, int filter, int useGroups,
            long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody,
            long outHandles, int capacity);
    public static native int queryIntersectSphere(
            long world, double cx, double cy, double cz, double radius,
            int flags, int memberships, int filter, int useGroups,
            long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody,
            long outHandles, int capacity);
    public static native int queryIntersectVoxelAabb(
            long world, double minX, double minY, double minZ, double maxX, double maxY, double maxZ,
            int flags, int memberships, int filter, int useGroups,
            long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody,
            long outHandles, int capacity);
    public static native int queryIntersectVoxelAabbCount(
            long world, double minX, double minY, double minZ, double maxX, double maxY, double maxZ,
            int flags, int memberships, int filter, int useGroups,
            long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody);
    public static native int queryIntersectVoxelObb(
            long world, double cx, double cy, double cz, double hx, double hy, double hz,
            double qi, double qj, double qk, double qw,
            int flags, int memberships, int filter, int useGroups,
            long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody,
            long outHandles, int capacity);
    public static native int queryIntersectVoxelObbCount(
            long world, double cx, double cy, double cz, double hx, double hy, double hz,
            double qi, double qj, double qk, double qw,
            int flags, int memberships, int filter, int useGroups,
            long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody);
    public static native long queryCastShape(
            long world, int shapeType, double a, double b, double c, double d,
            double tx, double ty, double tz, double qi, double qj, double qk, double qw,
            double vx, double vy, double vz, double maxToi, double targetDistance,
            int stopAtPenetration, int computeImpactGeometryOnPenetration,
            int flags, int memberships, int filter, int useGroups,
            long excludeCollider, int useExcludeCollider, long excludeRigidBody, int useExcludeRigidBody,
            long outHit);
    public static native int neuralBoundsRequiredWeightCount(int hiddenWidth, int hiddenLayers);
    public static native long worldInsertDynamicCuboids(
            long world, double x, double y, double z, double qi, double qj, double qk, double qw,
            double lvx, double lvy, double lvz, long cuboids, int cuboidCount,
            double density, double friction, double restitution,
            int collisionMemberships, int collisionFilter, int solverMemberships, int solverFilter);
    public static native long worldInsertStaticTrimesh(
            long world, long verticesXyz, int vertexXyzLen, long indices, int indexLen, double friction, double restitution);
    public static native long worldInsertStaticVoxelAabb(
            long world, double minX, double minY, double minZ, double maxX, double maxY, double maxZ,
            double voxelSize, int mode, int smallVoxelLimit, int meshVoxelLimit, double friction, double restitution);
    public static native long worldInsertDynamicVoxelObb(
            long world, double cx, double cy, double cz, double hx, double hy, double hz,
            double qi, double qj, double qk, double qw,
            double voxelSize, int mode, int smallVoxelLimit, int meshVoxelLimit,
            double density, double friction, double restitution);

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
    public static native boolean characterControllerMoveShape(
            long world, long controller, double dt, int shapeType, double a, double b, double c, double d,
            double tx, double ty, double tz, double qi, double qj, double qk, double qw,
            double dx, double dy, double dz, long outMovement);
    public static native int characterControllerCollisionCount(long controller);
    public static native long characterControllerGetCollision(long controller, int index, long outCollision);
    public static native boolean characterControllerSolveImpulses(
            long world, long controller, double dt, int shapeType, double a, double b, double c, double d, double characterMass);

    public static native void worldClearEvents(long world);
    public static native int worldCollisionEventCount(long world);
    public static native long worldGetCollisionEvent(long world, int index, long outEvent);
    public static native int worldGetCollisionEvents(long world, long outEvents, int capacity);
    public static native int worldContactForceEventCount(long world);
    public static native long worldGetContactForceEvent(long world, int index, long outEvent);
    public static native int worldGetContactForceEvents(long world, long outEvents, int capacity);
    public static native void worldClearContactPairFilterCallback(long world);
    public static native void worldClearIntersectionPairFilterCallback(long world);

    public static native double spaceKeplerPeriod(double mu, double semiMajorAxis);
    public static native double spaceKeplerSemiMajorAxis(double mu, double period);
    public static native boolean spaceHohmannTransfer(double mu, double radius1, double radius2, long outTransfer);
    public static native boolean spaceAtmosphericDragAcceleration(
            double vx, double vy, double vz,
            double atmosphereVx, double atmosphereVy, double atmosphereVz,
            double density, double dragCoefficient, double area, double mass,
            long outAcceleration);
    public static native boolean spaceApplyAtmosphericDragToBody(
            long world, long body,
            double atmosphereVx, double atmosphereVy, double atmosphereVz,
            double density, double dragCoefficient, double area, double mass,
            int wakeUp, long outAcceleration);
    public static native boolean spaceTriadAttitude(
            double bodyPrimaryX, double bodyPrimaryY, double bodyPrimaryZ,
            double bodySecondaryX, double bodySecondaryY, double bodySecondaryZ,
            double referencePrimaryX, double referencePrimaryY, double referencePrimaryZ,
            double referenceSecondaryX, double referenceSecondaryY, double referenceSecondaryZ,
            long outAttitude);
    public static native boolean spaceQuaternionDerivative(
            double qi, double qj, double qk, double qw,
            double wx, double wy, double wz,
            long outDerivative);
    public static native boolean spaceEkfPredictScalar(
            double state, double covariance, double nonlinearDelta, double jacobian, double processNoise,
            long outPrediction);
    public static native double spaceEkfGainScalar(double covariance, double measurementJacobian, double measurementNoise);
    public static native boolean spaceEkfUpdateScalar(
            double predictedState, double predictedCovariance,
            double measurement, double predictedMeasurement,
            double kalmanGain, double measurementJacobian,
            long outUpdate);

    public static native long rtreeCreate();
    public static native void rtreeDestroy(long tree);
    public static native void rtreeClear(long tree);
    public static native int rtreeLen(long tree);
    public static native boolean rtreeInsert(long tree, long id, double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
    public static native boolean rtreeUpdate(long tree, long id, double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
    public static native boolean rtreeRemove(long tree, long id);
    public static native void rtreeRebuild(long tree);
    public static native int rtreeQueryAabbCount(long tree, double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
    public static native int rtreeQueryAabb(long tree, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, long outIds, int capacity);

    public static native long crbTreeCreate();
    public static native void crbTreeDestroy(long tree);
    public static native void crbTreeClear(long tree);
    public static native int crbTreeLen(long tree);
    public static native boolean crbTreeInsert(long tree, long id, double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
    public static native boolean crbTreeUpdate(long tree, long id, double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
    public static native boolean crbTreeRemove(long tree, long id);
    public static native int crbTreeQueryAabbCount(long tree, double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
    public static native int crbTreeQueryAabb(long tree, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, long outIds, int capacity);
}
