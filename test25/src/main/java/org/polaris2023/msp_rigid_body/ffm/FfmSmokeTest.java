package org.polaris2023.msp_rigid_body.ffm;

import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.nio.file.Path;

public final class FfmSmokeTest {
    private static final double EPSILON = 1.0e-9;

    private FfmSmokeTest() {
    }

    public static void main(String[] args) {
        int javaVersion = Runtime.version().feature();
        if (javaVersion != 25) {
            throw new AssertionError("test25 must run on Java 25, got Java " + javaVersion);
        }

        String nativePath = System.getProperty("rigidbody.native.path");
        if (nativePath == null || nativePath.isBlank()) {
            throw new AssertionError("missing rigidbody.native.path");
        }

        try (Arena arena = Arena.ofShared()) {
            RigidBodyFfm api = new RigidBodyFfm(Path.of(nativePath), arena);

            if (api.abiVersion() < 1) {
                throw new AssertionError("invalid ABI version");
            }

            assertSpaceFormulaWrappers(api);

            MemorySegment world = api.worldCreate(0.0, -9.81, 0.0);
            try {
                MemorySegment gravity = api.worldGetGravity(world);
                assertClose(-9.81, RigidBodyFfm.y(gravity), "initial gravity y");

                api.worldSetGravity(world, 1.0, 2.0, 3.0);
                gravity = api.worldGetGravity(world);
                assertClose(1.0, RigidBodyFfm.x(gravity), "gravity x");
                assertClose(2.0, RigidBodyFfm.y(gravity), "gravity y");
                assertClose(3.0, RigidBodyFfm.z(gravity), "gravity z");

                MemorySegment builder = api.rigidBodyBuilderCreate(0);
                api.rigidBodyBuilderSetTranslation(builder, 4.0, 5.0, 6.0);
                MemorySegment builtBody = api.rigidBodyBuilderBuild(builder);
                if (builtBody.address() == 0L) {
                    throw new AssertionError("rigid_body_builder_build returned null");
                }
                long body = api.worldInsertRigidBody(world, builtBody);
                if (body == 0L) {
                    throw new AssertionError("world_insert_rigid_body returned zero handle");
                }
                MemorySegment massBuilder = api.colliderBuilderCreate(1, 0.5, 0.5, 0.5);
                api.colliderBuilderSetDensity(massBuilder, 1.0);
                MemorySegment massCollider = api.colliderBuilderBuild(massBuilder);
                if (api.worldInsertColliderWithParent(world, massCollider, body) == 0L) {
                    throw new AssertionError("mass collider insert failed");
                }

                MemorySegment translation = api.rigidBodyGetTranslation(world, body);
                assertClose(4.0, RigidBodyFfm.x(translation), "body translation x");
                assertClose(5.0, RigidBodyFfm.y(translation), "body translation y");
                assertClose(6.0, RigidBodyFfm.z(translation), "body translation z");
                if (!api.rigidBodySetTranslation(world, body, 7.0, 8.0, 9.0, true)) {
                    throw new AssertionError("rigid_body_set_translation failed");
                }
                translation = api.rigidBodyGetTranslation(world, body);
                assertClose(7.0, RigidBodyFfm.x(translation), "body updated translation x");
                assertClose(8.0, RigidBodyFfm.y(translation), "body updated translation y");
                assertClose(9.0, RigidBodyFfm.z(translation), "body updated translation z");
                if (!api.rigidBodySetRotation(world, body, 0.0, 0.0, 0.0, 1.0, true)
                        || !api.rigidBodySetPose(world, body, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, true)) {
                    throw new AssertionError("rigid body pose/rotation setters failed");
                }
                translation = api.rigidBodyGetTranslation(world, body);
                assertClose(1.0, RigidBodyFfm.x(translation), "body pose translation x");
                assertClose(2.0, RigidBodyFfm.y(translation), "body pose translation y");
                assertClose(3.0, RigidBodyFfm.z(translation), "body pose translation z");
                if (!api.rigidBodySetLinvel(world, body, 0.0, -2.0, 0.0, true)
                        || !api.rigidBodySetAngvel(world, body, 0.0, 1.0, 0.0, true)) {
                    throw new AssertionError("rigid body velocity setters failed");
                }
                MemorySegment linvel = api.rigidBodyGetLinvel(world, body);
                assertClose(-2.0, RigidBodyFfm.y(linvel), "body linvel y");
                MemorySegment angvel = api.rigidBodyGetAngvel(world, body);
                assertClose(1.0, RigidBodyFfm.y(angvel), "body angvel y");
                if (!api.rigidBodyAddForce(world, body, 0.0, 1.0, 0.0, true)
                        || !api.rigidBodyAddTorque(world, body, 0.0, 1.0, 0.0, true)
                        || !api.rigidBodyApplyImpulse(world, body, 0.0, 0.5, 0.0, true)
                        || !api.rigidBodyApplyTorqueImpulse(world, body, 0.0, 0.5, 0.0, true)
                        || !api.rigidBodyEnableCcd(world, body, true)) {
                    throw new AssertionError("rigid body force/impulse/ccd calls failed");
                }
                if (!api.rigidBodySetLinvel(world, body, 10.0, 0.0, 0.0, true)) {
                    throw new AssertionError("rigid_body_set_linvel for drag test failed");
                }
                MemorySegment dragAcceleration = api.spaceApplyAtmosphericDragToBody(
                        world,
                        body,
                        0.0, 0.0, 0.0,
                        1.225,
                        2.2,
                        1.0,
                        10.0,
                        true);
                if (RigidBodyFfm.x(dragAcceleration) >= 0.0) {
                    throw new AssertionError("space atmospheric drag should oppose positive x velocity");
                }
                if (!api.rigidBodySleep(world, body)) {
                    throw new AssertionError("rigid_body_sleep failed");
                }
                if (!api.rigidBodyWakeUp(world, body, true)) {
                    throw new AssertionError("rigid_body_wake_up failed");
                }
                api.rigidBodyIsSleeping(world, body);
                MemorySegment aeroReport = api.aeroApplyVoxelGrid(
                        world,
                        body,
                        10.0, 0.0, 0.0,
                        1.0,
                        new byte[] {1},
                        1, 1, 1,
                        1.0,
                        -0.5, -0.5, -0.5,
                        1.0,
                        0.0,
                        true);
                if (RigidBodyFfm.aeroReportSurfaceCount(aeroReport) != 6
                        || RigidBodyFfm.aeroReportActiveSurfaceCount(aeroReport) <= 0
                        || RigidBodyFfm.x(RigidBodyFfm.aeroReportTotalForce(aeroReport)) <= 0.0) {
                    throw new AssertionError("aero voxel grid force report was invalid");
                }
                api.worldStep(world, 1.0 / 60.0);
                linvel = api.rigidBodyGetLinvel(world, body);
                if (RigidBodyFfm.x(linvel) <= 0.0) {
                    throw new AssertionError("aero voxel grid did not accelerate body in wind direction");
                }
            } finally {
                api.worldDestroy(world);
            }

            MemorySegment tree = api.crbTreeCreate();
            try {
                if (!api.crbTreeInsert(tree, 10L, api.aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0))) {
                    throw new AssertionError("crb_tree_insert 10 failed");
                }
                if (!api.crbTreeInsert(tree, 20L, api.aabb(2.0, 2.0, 2.0, 3.0, 3.0, 3.0))) {
                    throw new AssertionError("crb_tree_insert 20 failed");
                }
                int hitCount = api.crbTreeQueryAabbCount(tree, api.aabb(0.5, 0.5, 0.5, 2.5, 2.5, 2.5));
                if (hitCount != 2) {
                    throw new AssertionError("crb_tree_query_aabb_count expected 2, got " + hitCount);
                }
            } finally {
                api.crbTreeDestroy(tree);
            }

            MemorySegment rtree = api.rtreeCreate();
            try {
                if (rtree.address() == 0L) {
                    throw new AssertionError("rtree_create returned null");
                }
                if (!api.rtreeInsert(rtree, 10L, api.aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0))) {
                    throw new AssertionError("rtree_insert 10 failed");
                }
                if (!api.rtreeInsert(rtree, 20L, api.aabb(2.0, 2.0, 2.0, 3.0, 3.0, 3.0))) {
                    throw new AssertionError("rtree_insert 20 failed");
                }
                if (api.rtreeLen(rtree) != 2) {
                    throw new AssertionError("rtree_len expected 2");
                }
                int hitCount = api.rtreeQueryAabbCount(rtree, api.aabb(0.5, 0.5, 0.5, 2.5, 2.5, 2.5));
                if (hitCount != 2) {
                    throw new AssertionError("rtree_query_aabb_count expected 2, got " + hitCount);
                }
                long[] hits = api.rtreeQueryAabb(rtree, api.aabb(0.5, 0.5, 0.5, 2.5, 2.5, 2.5), 4);
                if (hits.length != 2 || hits[0] != 10L || hits[1] != 20L) {
                    throw new AssertionError("rtree_query_aabb returned unexpected handles");
                }
                if (!api.rtreeUpdate(rtree, 20L, api.aabb(10.0, 10.0, 10.0, 11.0, 11.0, 11.0))) {
                    throw new AssertionError("rtree_update 20 failed");
                }
                api.rtreeRebuild(rtree);
                if (api.rtreeQueryAabbCount(rtree, api.aabb(0.5, 0.5, 0.5, 2.5, 2.5, 2.5)) != 1) {
                    throw new AssertionError("rtree update did not move id 20");
                }
                if (!api.rtreeRemove(rtree, 10L) || api.rtreeLen(rtree) != 1) {
                    throw new AssertionError("rtree_remove 10 failed");
                }
                api.rtreeClear(rtree);
                if (api.rtreeLen(rtree) != 0) {
                    throw new AssertionError("rtree_clear failed");
                }
            } finally {
                api.rtreeDestroy(rtree);
            }

            world = api.worldCreate(0.0, -9.81, 0.0);
            try {
                api.worldClearEvents(world);
                MemorySegment voxelAabb = api.aabb(0.0, 0.0, 0.0, 2.0, 1.0, 1.0);
                MemorySegment voxelObb = api.obb(0.0, 0.0, 0.0, 1.0, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0);
                MemorySegment options = api.voxelOptions(0, false, 128, 20_000);
                MemorySegment stats = api.voxelAabbBuildStats(voxelAabb, 0.5, options);
                if (RigidBodyFfm.voxelStatsSolidCount(stats) == 0 || RigidBodyFfm.voxelStatsSelectedMode(stats) == 0) {
                    throw new AssertionError("invalid voxel AABB stats");
                }
                MemorySegment obbStats = api.voxelObbBuildStats(voxelObb, 0.5, options);
                if (RigidBodyFfm.voxelStatsSolidCount(obbStats) == 0 || RigidBodyFfm.voxelStatsSelectedMode(obbStats) == 0) {
                    throw new AssertionError("invalid voxel OBB stats");
                }

                MemorySegment builder = api.colliderBuilderCreateVoxelAabb(voxelAabb, 0.5, options);
                if (builder.address() == 0L) {
                    throw new AssertionError("collider_builder_create_voxel_aabb returned null");
                }
                api.colliderBuilderSetFriction(builder, 0.8);
                api.colliderBuilderSetRestitution(builder, 0.1);
                api.colliderBuilderSetSensor(builder, false);
                MemorySegment collider = api.colliderBuilderBuild(builder);
                if (collider.address() == 0L) {
                    throw new AssertionError("collider_builder_build for voxel AABB returned null");
                }
                long colliderHandle = api.worldInsertCollider(world, collider);
                if (colliderHandle == 0L) {
                    throw new AssertionError("world_insert_collider for voxel AABB returned zero");
                }
                if (!api.colliderSetPose(world, colliderHandle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
                        || !api.colliderSetSensor(world, colliderHandle, false)
                        || !api.colliderSetFriction(world, colliderHandle, 0.7)
                        || !api.colliderSetRestitution(world, colliderHandle, 0.2)
                        || !api.colliderSetCollisionGroups(world, colliderHandle, 0xffff, 0xffff)
                        || !api.colliderSetSolverGroups(world, colliderHandle, 0xffff, 0xffff)
                        || !api.colliderSetActiveEvents(world, colliderHandle, 3)
                        || !api.colliderSetActiveHooks(world, colliderHandle, 3)
                        || !api.colliderSetContactForceEventThreshold(world, colliderHandle, 0.0)) {
                    throw new AssertionError("collider runtime setters failed");
                }
                MemorySegment colliderTranslation = api.colliderGetTranslation(world, colliderHandle);
                assertClose(0.0, RigidBodyFfm.x(colliderTranslation), "collider translation x");
                api.colliderGetRotation(world, colliderHandle);

                MemorySegment obbBuilder = api.colliderBuilderCreateVoxelObb(voxelObb, 0.5, options);
                if (obbBuilder.address() == 0L) {
                    throw new AssertionError("collider_builder_create_voxel_obb returned null");
                }
                api.colliderBuilderSetDensity(obbBuilder, 1.0);
                api.colliderBuilderSetFriction(obbBuilder, 0.5);
                MemorySegment obbCollider = api.colliderBuilderBuild(obbBuilder);
                if (obbCollider.address() == 0L) {
                    throw new AssertionError("collider_builder_build for voxel OBB returned null");
                }
                long obbColliderHandle = api.worldInsertCollider(world, obbCollider);
                if (obbColliderHandle == 0L) {
                    throw new AssertionError("world_insert_collider for voxel OBB returned zero");
                }
                if (api.colliderGetDensity(world, obbColliderHandle) <= 0.0) {
                    throw new AssertionError("collider_get_density should return positive density");
                }
                api.worldStep(world, 1.0 / 60.0);
                if (api.worldGetColliderSetSize(world) != 2) {
                    throw new AssertionError("expected two voxel colliders");
                }
                if (api.queryIntersectVoxelAabbCount(world, voxelAabb) != 2) {
                    throw new AssertionError("voxel AABB query should hit both colliders");
                }
                long[] aabbHits = api.queryIntersectVoxelAabb(world, voxelAabb, 4);
                if (aabbHits.length != 2) {
                    throw new AssertionError("voxel AABB query expected 2 handles, got " + aabbHits.length);
                }
                if (api.queryIntersectVoxelObbCount(world, voxelObb) != 2) {
                    throw new AssertionError("voxel OBB query should hit both colliders");
                }
                long[] obbHits = api.queryIntersectVoxelObb(world, voxelObb, 4);
                if (obbHits.length != 2) {
                    throw new AssertionError("voxel OBB query expected 2 handles, got " + obbHits.length);
                }

                if (api.queryIntersectAabbCount(world, voxelAabb) != 2) {
                    throw new AssertionError("AABB query should hit both colliders");
                }
                if (api.queryIntersectAabb(world, voxelAabb, 4).length != 2) {
                    throw new AssertionError("AABB query handle output failed");
                }
                if (api.queryIntersectObbCount(world, voxelObb) != 2) {
                    throw new AssertionError("OBB query should hit both colliders");
                }
                if (api.queryIntersectObb(world, voxelObb, 4).length != 2) {
                    throw new AssertionError("OBB query handle output failed");
                }
                MemorySegment sphere = api.sphere(1.0, 0.5, 0.5, 2.0);
                if (api.queryIntersectSphereCount(world, sphere) != 2) {
                    throw new AssertionError("sphere query should hit both colliders");
                }
                if (api.queryIntersectSphere(world, sphere, 4).length != 2) {
                    throw new AssertionError("sphere query handle output failed");
                }

                MemorySegment rayHit = api.queryCastRay(
                        world,
                        1.0, 3.0, 0.5,
                        0.0, -1.0, 0.0,
                        10.0,
                        true);
                if (RigidBodyFfm.rayHitCollider(rayHit) == 0L || RigidBodyFfm.rayHitTimeOfImpact(rayHit) < 0.0) {
                    throw new AssertionError("ray cast should hit voxel collider");
                }

                MemorySegment projection = api.queryProjectPoint(world, 1.0, 0.5, 0.5, 5.0, true);
                if (!RigidBodyFfm.pointProjectionInside(projection)) {
                    throw new AssertionError("point projection should be inside voxel collider");
                }

                MemorySegment shapeCastHit = api.queryCastShape(
                        world,
                        api.shapeDesc(0, 0.25, 0.0, 0.0, 0.0),
                        api.vec3(1.0, 3.0, 0.5),
                        api.quat(0.0, 0.0, 0.0, 1.0),
                        api.vec3(0.0, -1.0, 0.0),
                        10.0);
                if (RigidBodyFfm.shapeCastHitCollider(shapeCastHit) == 0L
                        || RigidBodyFfm.shapeCastHitTimeOfImpact(shapeCastHit) < 0.0) {
                    throw new AssertionError("shape cast should hit voxel collider");
                }

                api.worldClearEvents(world);
                if (api.worldCollisionEventCount(world) != 0 || api.worldContactForceEventCount(world) != 0) {
                    throw new AssertionError("world_clear_events did not clear event queues");
                }
            } finally {
                api.worldDestroy(world);
            }

            assertFfmEvents(api);
        }

        System.out.println("FFM smoke test passed on Java " + javaVersion);
    }

    private static void assertClose(double expected, double actual, String label) {
        if (Math.abs(expected - actual) > EPSILON) {
            throw new AssertionError(label + ": expected " + expected + ", got " + actual);
        }
    }

    private static void assertSpaceFormulaWrappers(RigidBodyFfm api) {
        double mu = 3.986004418e14;
        double semiMajorAxis = 7_000_000.0;
        double period = api.spaceKeplerPeriod(mu, semiMajorAxis);
        if (!Double.isFinite(period) || period <= 0.0) {
            throw new AssertionError("space_kepler_period returned invalid value");
        }
        assertClose(semiMajorAxis, api.spaceKeplerSemiMajorAxis(mu, period), "space Kepler round trip");

        MemorySegment hohmann = api.spaceHohmannTransfer(mu, 7_000_000.0, 42_164_000.0);
        if (RigidBodyFfm.hohmannTotalDeltaV(hohmann) <= 0.0 || RigidBodyFfm.hohmannTransferTime(hohmann) <= 0.0) {
            throw new AssertionError("space_hohmann_transfer returned invalid transfer");
        }

        MemorySegment drag = api.spaceAtmosphericDragAcceleration(
                10.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                1.225,
                2.2,
                1.0,
                100.0);
        if (RigidBodyFfm.x(drag) >= 0.0) {
            throw new AssertionError("space_atmospheric_drag_acceleration should oppose velocity");
        }

        MemorySegment attitude = api.spaceTriadAttitude(
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0);
        assertClose(1.0, attitude.get(java.lang.foreign.ValueLayout.JAVA_DOUBLE, RigidBodyFfm.QUAT.byteOffset(java.lang.foreign.MemoryLayout.PathElement.groupElement("w"))), "TRIAD identity quaternion w");

        MemorySegment qdot = api.spaceQuaternionDerivative(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0);
        if (qdot.get(java.lang.foreign.ValueLayout.JAVA_DOUBLE, RigidBodyFfm.QUATERNION_DERIVATIVE.byteOffset(java.lang.foreign.MemoryLayout.PathElement.groupElement("k_dot"))) <= 0.0) {
            throw new AssertionError("space_quaternion_derivative returned invalid derivative");
        }

        MemorySegment prediction = api.spaceEkfPredictScalar(1.0, 2.0, 0.5, 1.0, 0.1);
        double gain = api.spaceEkfGainScalar(RigidBodyFfm.scalarKalmanCovariance(prediction), 1.0, 0.5);
        MemorySegment update = api.spaceEkfUpdateScalar(RigidBodyFfm.scalarKalmanValue(prediction), RigidBodyFfm.scalarKalmanCovariance(prediction), 2.0, 1.5, gain, 1.0);
        if (!Double.isFinite(RigidBodyFfm.scalarKalmanValue(update)) || RigidBodyFfm.scalarKalmanCovariance(update) < 0.0) {
            throw new AssertionError("space EKF scalar wrappers returned invalid state");
        }
    }

    private static void assertFfmEvents(RigidBodyFfm api) {
        MemorySegment world = api.worldCreate(0.0, -9.81, 0.0);
        try {
            api.worldClearEvents(world);

            MemorySegment groundBuilder = api.colliderBuilderCreate(1, 4.0, 0.25, 4.0);
            api.colliderBuilderSetFriction(groundBuilder, 0.8);
            api.colliderBuilderSetActiveEvents(groundBuilder, 3);
            api.colliderBuilderSetContactForceEventThreshold(groundBuilder, 0.0);
            MemorySegment groundCollider = api.colliderBuilderBuild(groundBuilder);
            long ground = api.worldInsertCollider(world, groundCollider);
            if (ground == 0L) {
                throw new AssertionError("ground collider insert failed");
            }

            MemorySegment bodyBuilder = api.rigidBodyBuilderCreate(0);
            api.rigidBodyBuilderSetTranslation(bodyBuilder, 0.0, 1.0, 0.0);
            MemorySegment bodyMemory = api.rigidBodyBuilderBuild(bodyBuilder);
            long body = api.worldInsertRigidBody(world, bodyMemory);
            if (body == 0L) {
                throw new AssertionError("dynamic body insert failed");
            }

            MemorySegment dynamicBuilder = api.colliderBuilderCreate(1, 0.5, 0.5, 0.5);
            api.colliderBuilderSetDensity(dynamicBuilder, 1.0);
            api.colliderBuilderSetActiveEvents(dynamicBuilder, 3);
            api.colliderBuilderSetContactForceEventThreshold(dynamicBuilder, 0.0);
            MemorySegment dynamicCollider = api.colliderBuilderBuild(dynamicBuilder);
            long dynamic = api.worldInsertColliderWithParent(world, dynamicCollider, body);
            if (dynamic == 0L) {
                throw new AssertionError("dynamic collider insert failed");
            }

            for (int i = 0; i < 120 && api.worldCollisionEventCount(world) == 0; i++) {
                api.worldStep(world, 1.0 / 60.0);
            }

            int collisionCount = api.worldCollisionEventCount(world);
            if (collisionCount <= 0) {
                throw new AssertionError("expected at least one collision event");
            }
            MemorySegment collisionEvents = api.worldGetCollisionEvents(world, collisionCount);
            if (RigidBodyFfm.eventCount(collisionEvents, RigidBodyFfm.COLLISION_EVENT) != collisionCount) {
                throw new AssertionError("collision event bulk read count mismatch");
            }
            boolean foundStartedPair = false;
            for (int i = 0; i < collisionCount; i++) {
                long c1 = RigidBodyFfm.collisionEventCollider1(collisionEvents, i);
                long c2 = RigidBodyFfm.collisionEventCollider2(collisionEvents, i);
                if (RigidBodyFfm.collisionEventStarted(collisionEvents, i)
                        && ((c1 == ground && c2 == dynamic) || (c1 == dynamic && c2 == ground))) {
                    foundStartedPair = true;
                }
            }
            if (!foundStartedPair) {
                throw new AssertionError("collision events did not include the dynamic-ground pair");
            }

            int contactForceCount = api.worldContactForceEventCount(world);
            MemorySegment contactForceEvents = api.worldGetContactForceEvents(world, contactForceCount);
            if (RigidBodyFfm.eventCount(contactForceEvents, RigidBodyFfm.CONTACT_FORCE_EVENT) != contactForceCount) {
                throw new AssertionError("contact force event bulk read count mismatch");
            }
            for (int i = 0; i < contactForceCount; i++) {
                if (RigidBodyFfm.contactForceEventCollider1(contactForceEvents, i) == 0L
                        || RigidBodyFfm.contactForceEventCollider2(contactForceEvents, i) == 0L
                        || RigidBodyFfm.contactForceEventTotalForceMagnitude(contactForceEvents, i) < 0.0) {
                    throw new AssertionError("invalid contact force event record");
                }
            }
            api.worldClearEvents(world);
            if (api.worldCollisionEventCount(world) != 0 || api.worldContactForceEventCount(world) != 0) {
                throw new AssertionError("world_clear_events did not clear event queues");
            }
        } finally {
            api.worldDestroy(world);
        }
    }
}
