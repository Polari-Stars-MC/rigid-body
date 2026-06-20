package org.polaris2023.msp_rigid_body.ffm;

import java.lang.foreign.Arena;
import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.Linker;
import java.lang.foreign.MemoryLayout;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.SegmentAllocator;
import java.lang.foreign.SymbolLookup;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.nio.file.Path;

public final class RigidBodyFfm {
    public static final MemoryLayout BOOL = MemoryLayout.structLayout(ValueLayout.JAVA_BYTE.withName("_0"));
    public static final MemoryLayout VEC3 = MemoryLayout.structLayout(
            ValueLayout.JAVA_DOUBLE.withName("x"),
            ValueLayout.JAVA_DOUBLE.withName("y"),
            ValueLayout.JAVA_DOUBLE.withName("z"));
    public static final MemoryLayout QUAT = MemoryLayout.structLayout(
            ValueLayout.JAVA_DOUBLE.withName("i"),
            ValueLayout.JAVA_DOUBLE.withName("j"),
            ValueLayout.JAVA_DOUBLE.withName("k"),
            ValueLayout.JAVA_DOUBLE.withName("w"));
    public static final MemoryLayout AABB = MemoryLayout.structLayout(
            VEC3.withName("mins"),
            VEC3.withName("maxs"));
    public static final MemoryLayout SPHERE = MemoryLayout.structLayout(
            VEC3.withName("center"),
            ValueLayout.JAVA_DOUBLE.withName("radius"));
    public static final MemoryLayout OBB = MemoryLayout.structLayout(
            VEC3.withName("center"),
            VEC3.withName("half_extents"),
            QUAT.withName("rotation"));
    public static final MemoryLayout SHAPE_DESC = MemoryLayout.structLayout(
            ValueLayout.JAVA_INT.withName("shape_type"),
            MemoryLayout.paddingLayout(4),
            ValueLayout.JAVA_DOUBLE.withName("a"),
            ValueLayout.JAVA_DOUBLE.withName("b"),
            ValueLayout.JAVA_DOUBLE.withName("c"),
            ValueLayout.JAVA_DOUBLE.withName("d"));
    public static final MemoryLayout INTERACTION_GROUPS = MemoryLayout.structLayout(
            ValueLayout.JAVA_INT.withName("memberships"),
            ValueLayout.JAVA_INT.withName("filter"));
    public static final MemoryLayout QUERY_FILTER = MemoryLayout.structLayout(
            ValueLayout.JAVA_INT.withName("flags"),
            INTERACTION_GROUPS.withName("groups"),
            BOOL.withName("use_groups"),
            MemoryLayout.paddingLayout(3),
            ValueLayout.JAVA_LONG.withName("exclude_collider"),
            BOOL.withName("use_exclude_collider"),
            MemoryLayout.paddingLayout(7),
            ValueLayout.JAVA_LONG.withName("exclude_rigid_body"),
            BOOL.withName("use_exclude_rigid_body"),
            MemoryLayout.paddingLayout(7));
    public static final MemoryLayout VOXEL_OPTIONS = MemoryLayout.structLayout(
            ValueLayout.JAVA_INT.withName("mode"),
            ValueLayout.JAVA_BYTE.withName("dynamic_body"),
            MemoryLayout.paddingLayout(3),
            ValueLayout.JAVA_INT.withName("small_voxel_limit"),
            ValueLayout.JAVA_INT.withName("mesh_voxel_limit"));
    public static final MemoryLayout VOXEL_STATS = MemoryLayout.structLayout(
            ValueLayout.JAVA_INT.withName("cell_count"),
            ValueLayout.JAVA_INT.withName("solid_count"),
            ValueLayout.JAVA_INT.withName("selected_mode"),
            ValueLayout.JAVA_INT.withName("estimated_parts"),
            ValueLayout.JAVA_INT.withName("estimated_vertices"),
            ValueLayout.JAVA_INT.withName("estimated_triangles"),
            ValueLayout.JAVA_INT.withName("size_x"),
            ValueLayout.JAVA_INT.withName("size_y"),
            ValueLayout.JAVA_INT.withName("size_z"));
    public static final MemoryLayout RAY_HIT = MemoryLayout.structLayout(
            ValueLayout.JAVA_LONG.withName("collider"),
            ValueLayout.JAVA_DOUBLE.withName("time_of_impact"),
            VEC3.withName("normal"),
            ValueLayout.JAVA_INT.withName("feature"),
            MemoryLayout.paddingLayout(4));
    public static final MemoryLayout POINT_PROJECTION = MemoryLayout.structLayout(
            VEC3.withName("point"),
            BOOL.withName("is_inside"),
            MemoryLayout.paddingLayout(7));
    public static final MemoryLayout SHAPE_CAST_OPTIONS = MemoryLayout.structLayout(
            ValueLayout.JAVA_DOUBLE.withName("max_time_of_impact"),
            ValueLayout.JAVA_DOUBLE.withName("target_distance"),
            BOOL.withName("stop_at_penetration"),
            BOOL.withName("compute_impact_geometry_on_penetration"),
            MemoryLayout.paddingLayout(6));
    public static final MemoryLayout SHAPE_CAST_HIT = MemoryLayout.structLayout(
            ValueLayout.JAVA_LONG.withName("collider"),
            ValueLayout.JAVA_DOUBLE.withName("time_of_impact"),
            VEC3.withName("witness1"),
            VEC3.withName("witness2"),
            VEC3.withName("normal1"),
            VEC3.withName("normal2"),
            ValueLayout.JAVA_INT.withName("status"),
            MemoryLayout.paddingLayout(4));
    public static final MemoryLayout COLLISION_EVENT = MemoryLayout.structLayout(
            BOOL.withName("started"),
            MemoryLayout.paddingLayout(7),
            ValueLayout.JAVA_LONG.withName("collider1"),
            ValueLayout.JAVA_LONG.withName("collider2"),
            BOOL.withName("sensor"),
            BOOL.withName("removed"),
            MemoryLayout.paddingLayout(6));
    public static final MemoryLayout CONTACT_FORCE_EVENT = MemoryLayout.structLayout(
            ValueLayout.JAVA_LONG.withName("collider1"),
            ValueLayout.JAVA_LONG.withName("collider2"),
            VEC3.withName("total_force"),
            ValueLayout.JAVA_DOUBLE.withName("total_force_magnitude"),
            VEC3.withName("max_force_direction"),
            ValueLayout.JAVA_DOUBLE.withName("max_force_magnitude"));
    public static final MemoryLayout AERO_REPORT = MemoryLayout.structLayout(
            VEC3.withName("total_force"),
            VEC3.withName("total_torque"),
            ValueLayout.JAVA_INT.withName("surface_count"),
            ValueLayout.JAVA_INT.withName("active_surface_count"));
    public static final MemoryLayout HOHMANN_TRANSFER = MemoryLayout.structLayout(
            ValueLayout.JAVA_DOUBLE.withName("delta_v1"),
            ValueLayout.JAVA_DOUBLE.withName("delta_v2"),
            ValueLayout.JAVA_DOUBLE.withName("total_delta_v"),
            ValueLayout.JAVA_DOUBLE.withName("transfer_time"));
    public static final MemoryLayout QUATERNION_DERIVATIVE = MemoryLayout.structLayout(
            ValueLayout.JAVA_DOUBLE.withName("i_dot"),
            ValueLayout.JAVA_DOUBLE.withName("j_dot"),
            ValueLayout.JAVA_DOUBLE.withName("k_dot"),
            ValueLayout.JAVA_DOUBLE.withName("w_dot"));
    public static final MemoryLayout SCALAR_KALMAN = MemoryLayout.structLayout(
            ValueLayout.JAVA_DOUBLE.withName("value"),
            ValueLayout.JAVA_DOUBLE.withName("covariance"));

    public static final long VEC3_X = VEC3.byteOffset(MemoryLayout.PathElement.groupElement("x"));
    public static final long VEC3_Y = VEC3.byteOffset(MemoryLayout.PathElement.groupElement("y"));
    public static final long VEC3_Z = VEC3.byteOffset(MemoryLayout.PathElement.groupElement("z"));
    public static final long RAY_HIT_COLLIDER = RAY_HIT.byteOffset(MemoryLayout.PathElement.groupElement("collider"));
    public static final long RAY_HIT_TOI = RAY_HIT.byteOffset(MemoryLayout.PathElement.groupElement("time_of_impact"));
    public static final long POINT_PROJECTION_INSIDE = POINT_PROJECTION.byteOffset(
            MemoryLayout.PathElement.groupElement("is_inside"),
            MemoryLayout.PathElement.groupElement("_0"));
    public static final long SHAPE_CAST_HIT_COLLIDER = SHAPE_CAST_HIT.byteOffset(MemoryLayout.PathElement.groupElement("collider"));
    public static final long SHAPE_CAST_HIT_TOI = SHAPE_CAST_HIT.byteOffset(MemoryLayout.PathElement.groupElement("time_of_impact"));
    public static final long COLLISION_EVENT_STARTED = COLLISION_EVENT.byteOffset(
            MemoryLayout.PathElement.groupElement("started"),
            MemoryLayout.PathElement.groupElement("_0"));
    public static final long COLLISION_EVENT_COLLIDER1 = COLLISION_EVENT.byteOffset(MemoryLayout.PathElement.groupElement("collider1"));
    public static final long COLLISION_EVENT_COLLIDER2 = COLLISION_EVENT.byteOffset(MemoryLayout.PathElement.groupElement("collider2"));
    public static final long CONTACT_FORCE_EVENT_COLLIDER1 = CONTACT_FORCE_EVENT.byteOffset(MemoryLayout.PathElement.groupElement("collider1"));
    public static final long CONTACT_FORCE_EVENT_COLLIDER2 = CONTACT_FORCE_EVENT.byteOffset(MemoryLayout.PathElement.groupElement("collider2"));
    public static final long CONTACT_FORCE_EVENT_TOTAL_FORCE_MAGNITUDE = CONTACT_FORCE_EVENT.byteOffset(MemoryLayout.PathElement.groupElement("total_force_magnitude"));
    public static final long VOXEL_STATS_SOLID_COUNT = VOXEL_STATS.byteOffset(MemoryLayout.PathElement.groupElement("solid_count"));
    public static final long VOXEL_STATS_SELECTED_MODE = VOXEL_STATS.byteOffset(MemoryLayout.PathElement.groupElement("selected_mode"));
    public static final long AERO_REPORT_TOTAL_FORCE = AERO_REPORT.byteOffset(MemoryLayout.PathElement.groupElement("total_force"));
    public static final long AERO_REPORT_SURFACE_COUNT = AERO_REPORT.byteOffset(MemoryLayout.PathElement.groupElement("surface_count"));
    public static final long AERO_REPORT_ACTIVE_SURFACE_COUNT = AERO_REPORT.byteOffset(MemoryLayout.PathElement.groupElement("active_surface_count"));
    public static final long HOHMANN_TOTAL_DELTA_V = HOHMANN_TRANSFER.byteOffset(MemoryLayout.PathElement.groupElement("total_delta_v"));
    public static final long HOHMANN_TRANSFER_TIME = HOHMANN_TRANSFER.byteOffset(MemoryLayout.PathElement.groupElement("transfer_time"));
    public static final long SCALAR_KALMAN_VALUE = SCALAR_KALMAN.byteOffset(MemoryLayout.PathElement.groupElement("value"));
    public static final long SCALAR_KALMAN_COVARIANCE = SCALAR_KALMAN.byteOffset(MemoryLayout.PathElement.groupElement("covariance"));

    private static final Linker LINKER = Linker.nativeLinker();

    private final SymbolLookup lookup;
    private final Arena arena;
    private final MethodHandle abiVersion;
    private final MethodHandle worldCreate;
    private final MethodHandle worldDestroy;
    private final MethodHandle worldStep;
    private final MethodHandle worldSetGravity;
    private final MethodHandle worldGetGravityOut;
    private final MethodHandle worldGetColliderSetSize;
    private final MethodHandle worldClearEvents;
    private final MethodHandle worldCollisionEventCount;
    private final MethodHandle worldGetCollisionEvents;
    private final MethodHandle worldContactForceEventCount;
    private final MethodHandle worldGetContactForceEvents;
    private final MethodHandle rigidBodyBuilderCreate;
    private final MethodHandle rigidBodyBuilderDestroy;
    private final MethodHandle rigidBodyBuilderSetTranslation;
    private final MethodHandle rigidBodyBuilderBuild;
    private final MethodHandle worldInsertRigidBody;
    private final MethodHandle rigidBodyGetTranslationOut;
    private final MethodHandle rigidBodyGetRotationOut;
    private final MethodHandle rigidBodySetPose;
    private final MethodHandle rigidBodySetTranslation;
    private final MethodHandle rigidBodySetRotation;
    private final MethodHandle rigidBodyGetLinvelOut;
    private final MethodHandle rigidBodySetLinvel;
    private final MethodHandle rigidBodyGetAngvelOut;
    private final MethodHandle rigidBodySetAngvel;
    private final MethodHandle rigidBodyAddForce;
    private final MethodHandle rigidBodyAddTorque;
    private final MethodHandle rigidBodyApplyImpulse;
    private final MethodHandle rigidBodyApplyTorqueImpulse;
    private final MethodHandle rigidBodyEnableCcd;
    private final MethodHandle rigidBodySleepFlag;
    private final MethodHandle rigidBodyWakeUpFlag;
    private final MethodHandle rigidBodyIsSleepingFlag;
    private final MethodHandle aeroApplyVoxelGrid;
    private final MethodHandle spaceKeplerPeriod;
    private final MethodHandle spaceKeplerSemiMajorAxis;
    private final MethodHandle spaceHohmannTransfer;
    private final MethodHandle spaceAtmosphericDragAcceleration;
    private final MethodHandle spaceApplyAtmosphericDragToBodyFlag;
    private final MethodHandle spaceTriadAttitude;
    private final MethodHandle spaceQuaternionDerivative;
    private final MethodHandle spaceEkfPredictScalar;
    private final MethodHandle spaceEkfGainScalar;
    private final MethodHandle spaceEkfUpdateScalar;
    private final MethodHandle crbTreeCreate;
    private final MethodHandle crbTreeDestroy;
    private final MethodHandle crbTreeInsertFlag;
    private final MethodHandle crbTreeQueryAabbCount;
    private final MethodHandle rtreeCreate;
    private final MethodHandle rtreeDestroy;
    private final MethodHandle rtreeClear;
    private final MethodHandle rtreeLen;
    private final MethodHandle rtreeInsert;
    private final MethodHandle rtreeUpdate;
    private final MethodHandle rtreeRemove;
    private final MethodHandle rtreeRebuild;
    private final MethodHandle rtreeQueryAabbCount;
    private final MethodHandle rtreeQueryAabb;
    private final MethodHandle voxelAabbBuildStats;
    private final MethodHandle voxelObbBuildStats;
    private final MethodHandle colliderBuilderCreate;
    private final MethodHandle colliderBuilderCreateVoxelAabb;
    private final MethodHandle colliderBuilderCreateVoxelObb;
    private final MethodHandle colliderBuilderSetFriction;
    private final MethodHandle colliderBuilderSetRestitution;
    private final MethodHandle colliderBuilderSetDensity;
    private final MethodHandle colliderBuilderSetSensor;
    private final MethodHandle colliderBuilderSetActiveEvents;
    private final MethodHandle colliderBuilderSetContactForceEventThreshold;
    private final MethodHandle colliderBuilderBuild;
    private final MethodHandle colliderBuilderDestroy;
    private final MethodHandle worldInsertCollider;
    private final MethodHandle worldInsertColliderWithParent;
    private final MethodHandle colliderGetTranslationOut;
    private final MethodHandle colliderGetRotationOut;
    private final MethodHandle colliderSetPose;
    private final MethodHandle colliderSetSensor;
    private final MethodHandle colliderSetFriction;
    private final MethodHandle colliderSetRestitution;
    private final MethodHandle colliderSetCollisionGroups;
    private final MethodHandle colliderSetSolverGroups;
    private final MethodHandle colliderSetActiveEvents;
    private final MethodHandle colliderSetActiveHooks;
    private final MethodHandle colliderSetContactForceEventThreshold;
    private final MethodHandle colliderGetDensity;
    private final MethodHandle queryCastRayOut;
    private final MethodHandle queryProjectPointOut;
    private final MethodHandle queryIntersectAabb;
    private final MethodHandle queryIntersectAabbCount;
    private final MethodHandle queryIntersectObb;
    private final MethodHandle queryIntersectObbCount;
    private final MethodHandle queryIntersectSphere;
    private final MethodHandle queryIntersectSphereCount;
    private final MethodHandle queryCastShapeOut;
    private final MethodHandle queryIntersectVoxelAabb;
    private final MethodHandle queryIntersectVoxelAabbCount;
    private final MethodHandle queryIntersectVoxelObb;
    private final MethodHandle queryIntersectVoxelObbCount;

    public RigidBodyFfm(Path library, Arena arena) {
        this.lookup = SymbolLookup.libraryLookup(library, arena);
        this.arena = arena;
        abiVersion = downcall("abi_version", FunctionDescriptor.of(ValueLayout.JAVA_INT));
        worldCreate = downcall("world_create", FunctionDescriptor.of(ValueLayout.ADDRESS, VEC3));
        worldDestroy = downcall("world_destroy", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS));
        worldStep = downcall("world_step", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_DOUBLE));
        worldSetGravity = downcall("world_set_gravity", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, VEC3));
        worldGetGravityOut = downcall("world_get_gravity_out", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.ADDRESS));
        worldGetColliderSetSize = downcall("world_get_collider_set_size", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS));
        worldClearEvents = downcall("world_clear_events", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS));
        worldCollisionEventCount = downcall("world_collision_event_count", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS));
        worldGetCollisionEvents = downcall("world_get_collision_events", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, ValueLayout.ADDRESS, ValueLayout.JAVA_INT));
        worldContactForceEventCount = downcall("world_contact_force_event_count", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS));
        worldGetContactForceEvents = downcall("world_get_contact_force_events", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, ValueLayout.ADDRESS, ValueLayout.JAVA_INT));
        rigidBodyBuilderCreate = downcall("rigid_body_builder_create", FunctionDescriptor.of(ValueLayout.ADDRESS, ValueLayout.JAVA_INT));
        rigidBodyBuilderDestroy = downcall("rigid_body_builder_destroy", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS));
        rigidBodyBuilderSetTranslation = downcall("rigid_body_builder_set_translation", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, VEC3));
        rigidBodyBuilderBuild = downcall("rigid_body_builder_build", FunctionDescriptor.of(ValueLayout.ADDRESS, ValueLayout.ADDRESS));
        worldInsertRigidBody = downcall("world_insert_rigid_body", FunctionDescriptor.of(ValueLayout.JAVA_LONG, ValueLayout.ADDRESS, ValueLayout.ADDRESS));
        rigidBodyGetTranslationOut = downcall("rigid_body_get_translation_out", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, ValueLayout.ADDRESS));
        rigidBodyGetRotationOut = downcall("rigid_body_get_rotation_out", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, ValueLayout.ADDRESS));
        rigidBodySetPose = downcall("rigid_body_set_pose_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, VEC3, QUAT, BOOL));
        rigidBodySetTranslation = downcall("rigid_body_set_translation_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, VEC3, BOOL));
        rigidBodySetRotation = downcall("rigid_body_set_rotation_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, QUAT, BOOL));
        rigidBodyGetLinvelOut = downcall("rigid_body_get_linvel_out", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, ValueLayout.ADDRESS));
        rigidBodySetLinvel = downcall("rigid_body_set_linvel_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, VEC3, BOOL));
        rigidBodyGetAngvelOut = downcall("rigid_body_get_angvel_out", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, ValueLayout.ADDRESS));
        rigidBodySetAngvel = downcall("rigid_body_set_angvel_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, VEC3, BOOL));
        rigidBodyAddForce = downcall("rigid_body_add_force_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, VEC3, BOOL));
        rigidBodyAddTorque = downcall("rigid_body_add_torque_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, VEC3, BOOL));
        rigidBodyApplyImpulse = downcall("rigid_body_apply_impulse_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, VEC3, BOOL));
        rigidBodyApplyTorqueImpulse = downcall("rigid_body_apply_torque_impulse_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, VEC3, BOOL));
        rigidBodyEnableCcd = downcall("rigid_body_enable_ccd_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, BOOL));
        rigidBodySleepFlag = downcall("rigid_body_sleep_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG));
        rigidBodyWakeUpFlag = downcall("rigid_body_wake_up_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, BOOL));
        rigidBodyIsSleepingFlag = downcall("rigid_body_is_sleeping_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG));
        aeroApplyVoxelGrid = downcall("aero_apply_voxel_grid_flag", FunctionDescriptor.of(
                ValueLayout.JAVA_BYTE,
                ValueLayout.ADDRESS,
                ValueLayout.JAVA_LONG,
                VEC3,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.ADDRESS,
                ValueLayout.JAVA_INT,
                ValueLayout.JAVA_INT,
                ValueLayout.JAVA_INT,
                ValueLayout.JAVA_DOUBLE,
                VEC3,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                BOOL,
                ValueLayout.ADDRESS));
        spaceKeplerPeriod = downcall("space_kepler_period", FunctionDescriptor.of(ValueLayout.JAVA_DOUBLE, ValueLayout.JAVA_DOUBLE, ValueLayout.JAVA_DOUBLE));
        spaceKeplerSemiMajorAxis = downcall("space_kepler_semi_major_axis", FunctionDescriptor.of(ValueLayout.JAVA_DOUBLE, ValueLayout.JAVA_DOUBLE, ValueLayout.JAVA_DOUBLE));
        spaceHohmannTransfer = downcall("space_hohmann_transfer", FunctionDescriptor.of(BOOL, ValueLayout.JAVA_DOUBLE, ValueLayout.JAVA_DOUBLE, ValueLayout.JAVA_DOUBLE, ValueLayout.ADDRESS));
        spaceAtmosphericDragAcceleration = downcall("space_atmospheric_drag_acceleration", FunctionDescriptor.of(
                BOOL,
                VEC3,
                VEC3,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.ADDRESS));
        spaceApplyAtmosphericDragToBodyFlag = downcall("space_apply_atmospheric_drag_to_body_flag", FunctionDescriptor.of(
                ValueLayout.JAVA_BYTE,
                ValueLayout.ADDRESS,
                ValueLayout.JAVA_LONG,
                VEC3,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                BOOL,
                ValueLayout.ADDRESS));
        spaceTriadAttitude = downcall("space_triad_attitude", FunctionDescriptor.of(BOOL, VEC3, VEC3, VEC3, VEC3, ValueLayout.ADDRESS));
        spaceQuaternionDerivative = downcall("space_quaternion_derivative", FunctionDescriptor.of(BOOL, QUAT, VEC3, ValueLayout.ADDRESS));
        spaceEkfPredictScalar = downcall("space_ekf_predict_scalar", FunctionDescriptor.of(
                BOOL,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.ADDRESS));
        spaceEkfGainScalar = downcall("space_ekf_gain_scalar", FunctionDescriptor.of(ValueLayout.JAVA_DOUBLE, ValueLayout.JAVA_DOUBLE, ValueLayout.JAVA_DOUBLE, ValueLayout.JAVA_DOUBLE));
        spaceEkfUpdateScalar = downcall("space_ekf_update_scalar", FunctionDescriptor.of(
                BOOL,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_DOUBLE,
                ValueLayout.ADDRESS));
        crbTreeCreate = downcall("crb_tree_create", FunctionDescriptor.of(ValueLayout.ADDRESS));
        crbTreeDestroy = downcall("crb_tree_destroy", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS));
        crbTreeInsertFlag = downcall("crb_tree_insert_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, AABB));
        crbTreeQueryAabbCount = downcall("crb_tree_query_aabb_count", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, AABB));
        rtreeCreate = downcall("rtree_create", FunctionDescriptor.of(ValueLayout.ADDRESS));
        rtreeDestroy = downcall("rtree_destroy", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS));
        rtreeClear = downcall("rtree_clear", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS));
        rtreeLen = downcall("rtree_len", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS));
        rtreeInsert = downcall("rtree_insert", FunctionDescriptor.of(BOOL, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, AABB));
        rtreeUpdate = downcall("rtree_update", FunctionDescriptor.of(BOOL, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, AABB));
        rtreeRemove = downcall("rtree_remove", FunctionDescriptor.of(BOOL, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG));
        rtreeRebuild = downcall("rtree_rebuild", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS));
        rtreeQueryAabbCount = downcall("rtree_query_aabb_count", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, AABB));
        rtreeQueryAabb = downcall("rtree_query_aabb", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, AABB, ValueLayout.ADDRESS, ValueLayout.JAVA_INT));
        voxelAabbBuildStats = downcall("voxel_aabb_build_stats_out", FunctionDescriptor.ofVoid(AABB, ValueLayout.JAVA_DOUBLE, VOXEL_OPTIONS, ValueLayout.ADDRESS));
        voxelObbBuildStats = downcall("voxel_obb_build_stats_out", FunctionDescriptor.ofVoid(OBB, ValueLayout.JAVA_DOUBLE, VOXEL_OPTIONS, ValueLayout.ADDRESS));
        colliderBuilderCreate = downcall("collider_builder_create", FunctionDescriptor.of(ValueLayout.ADDRESS, ValueLayout.JAVA_INT, VEC3));
        colliderBuilderCreateVoxelAabb = downcall("collider_builder_create_voxel_aabb", FunctionDescriptor.of(ValueLayout.ADDRESS, AABB, ValueLayout.JAVA_DOUBLE, VOXEL_OPTIONS));
        colliderBuilderCreateVoxelObb = downcall("collider_builder_create_voxel_obb", FunctionDescriptor.of(ValueLayout.ADDRESS, OBB, ValueLayout.JAVA_DOUBLE, VOXEL_OPTIONS));
        colliderBuilderSetFriction = downcall("collider_builder_set_friction", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_DOUBLE));
        colliderBuilderSetRestitution = downcall("collider_builder_set_restitution", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_DOUBLE));
        colliderBuilderSetDensity = downcall("collider_builder_set_density", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_DOUBLE));
        colliderBuilderSetSensor = downcall("collider_builder_set_sensor", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, BOOL));
        colliderBuilderSetActiveEvents = downcall("collider_builder_set_active_events", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_INT));
        colliderBuilderSetContactForceEventThreshold = downcall("collider_builder_set_contact_force_event_threshold", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_DOUBLE));
        colliderBuilderBuild = downcall("collider_builder_build", FunctionDescriptor.of(ValueLayout.ADDRESS, ValueLayout.ADDRESS));
        colliderBuilderDestroy = downcall("collider_builder_destroy", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS));
        worldInsertCollider = downcall("world_insert_collider", FunctionDescriptor.of(ValueLayout.JAVA_LONG, ValueLayout.ADDRESS, ValueLayout.ADDRESS));
        worldInsertColliderWithParent = downcall("world_insert_collider_with_parent", FunctionDescriptor.of(ValueLayout.JAVA_LONG, ValueLayout.ADDRESS, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG));
        colliderGetTranslationOut = downcall("collider_get_translation_out", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, ValueLayout.ADDRESS));
        colliderGetRotationOut = downcall("collider_get_rotation_out", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, ValueLayout.ADDRESS));
        colliderSetPose = downcall("collider_set_pose_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, VEC3, QUAT));
        colliderSetSensor = downcall("collider_set_sensor_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, BOOL));
        colliderSetFriction = downcall("collider_set_friction_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, ValueLayout.JAVA_DOUBLE));
        colliderSetRestitution = downcall("collider_set_restitution_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, ValueLayout.JAVA_DOUBLE));
        colliderSetCollisionGroups = downcall("collider_set_collision_groups_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, INTERACTION_GROUPS));
        colliderSetSolverGroups = downcall("collider_set_solver_groups_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, INTERACTION_GROUPS));
        colliderSetActiveEvents = downcall("collider_set_active_events_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, ValueLayout.JAVA_INT));
        colliderSetActiveHooks = downcall("collider_set_active_hooks_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, ValueLayout.JAVA_INT));
        colliderSetContactForceEventThreshold = downcall("collider_set_contact_force_event_threshold_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, ValueLayout.JAVA_DOUBLE));
        colliderGetDensity = downcall("collider_get_density", FunctionDescriptor.of(ValueLayout.JAVA_DOUBLE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG));
        queryCastRayOut = downcall("query_cast_ray_out", FunctionDescriptor.of(ValueLayout.JAVA_LONG, ValueLayout.ADDRESS, VEC3, VEC3, ValueLayout.JAVA_DOUBLE, BOOL, QUERY_FILTER, ValueLayout.ADDRESS));
        queryProjectPointOut = downcall("query_project_point_out", FunctionDescriptor.of(ValueLayout.JAVA_LONG, ValueLayout.ADDRESS, VEC3, ValueLayout.JAVA_DOUBLE, BOOL, QUERY_FILTER, ValueLayout.ADDRESS, ValueLayout.ADDRESS));
        queryIntersectAabb = downcall("query_intersect_aabb", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, AABB, QUERY_FILTER, ValueLayout.ADDRESS, ValueLayout.JAVA_INT));
        queryIntersectAabbCount = downcall("query_intersect_aabb_count", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, AABB, QUERY_FILTER));
        queryIntersectObb = downcall("query_intersect_obb", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, OBB, QUERY_FILTER, ValueLayout.ADDRESS, ValueLayout.JAVA_INT));
        queryIntersectObbCount = downcall("query_intersect_obb_count", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, OBB, QUERY_FILTER));
        queryIntersectSphere = downcall("query_intersect_sphere", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, SPHERE, QUERY_FILTER, ValueLayout.ADDRESS, ValueLayout.JAVA_INT));
        queryIntersectSphereCount = downcall("query_intersect_sphere_count", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, SPHERE, QUERY_FILTER));
        queryCastShapeOut = downcall("query_cast_shape_out", FunctionDescriptor.of(ValueLayout.JAVA_LONG, ValueLayout.ADDRESS, SHAPE_DESC, VEC3, QUAT, VEC3, SHAPE_CAST_OPTIONS, QUERY_FILTER, ValueLayout.ADDRESS));
        queryIntersectVoxelAabb = downcall("query_intersect_voxel_aabb", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, AABB, QUERY_FILTER, ValueLayout.ADDRESS, ValueLayout.JAVA_INT));
        queryIntersectVoxelAabbCount = downcall("query_intersect_voxel_aabb_count", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, AABB, QUERY_FILTER));
        queryIntersectVoxelObb = downcall("query_intersect_voxel_obb", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, OBB, QUERY_FILTER, ValueLayout.ADDRESS, ValueLayout.JAVA_INT));
        queryIntersectVoxelObbCount = downcall("query_intersect_voxel_obb_count", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, OBB, QUERY_FILTER));
    }

    public int abiVersion() {
        try {
            return (int) abiVersion.invokeExact();
        } catch (Throwable throwable) {
            throw callFailed("abi_version", throwable);
        }
    }

    public MemorySegment worldCreate(double gravityX, double gravityY, double gravityZ) {
        try {
            return (MemorySegment) worldCreate.invokeExact(vec3(gravityX, gravityY, gravityZ));
        } catch (Throwable throwable) {
            throw callFailed("world_create", throwable);
        }
    }

    public void worldDestroy(MemorySegment world) {
        try {
            worldDestroy.invokeExact(world);
        } catch (Throwable throwable) {
            throw callFailed("world_destroy", throwable);
        }
    }

    public void worldStep(MemorySegment world, double deltaSeconds) {
        try {
            worldStep.invokeExact(world, deltaSeconds);
        } catch (Throwable throwable) {
            throw callFailed("world_step", throwable);
        }
    }

    public void worldSetGravity(MemorySegment world, double x, double y, double z) {
        try {
            worldSetGravity.invokeExact(world, vec3(x, y, z));
        } catch (Throwable throwable) {
            throw callFailed("world_set_gravity", throwable);
        }
    }

    public MemorySegment worldGetGravity(MemorySegment world) {
        MemorySegment out = arena.allocate(VEC3);
        try {
            worldGetGravityOut.invokeExact(world, out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("world_get_gravity_out", throwable);
        }
    }

    public int worldGetColliderSetSize(MemorySegment world) {
        try {
            return (int) worldGetColliderSetSize.invokeExact(world);
        } catch (Throwable throwable) {
            throw callFailed("world_get_collider_set_size", throwable);
        }
    }

    public void worldClearEvents(MemorySegment world) {
        try {
            worldClearEvents.invokeExact(world);
        } catch (Throwable throwable) {
            throw callFailed("world_clear_events", throwable);
        }
    }

    public int worldCollisionEventCount(MemorySegment world) {
        try {
            return (int) worldCollisionEventCount.invokeExact(world);
        } catch (Throwable throwable) {
            throw callFailed("world_collision_event_count", throwable);
        }
    }

    public MemorySegment worldGetCollisionEvents(MemorySegment world, int capacity) {
        MemorySegment out = arena.allocate(COLLISION_EVENT, Math.max(1, capacity));
        if (capacity <= 0) {
            return out.asSlice(0, 0);
        }
        try {
            int written = (int) worldGetCollisionEvents.invokeExact(world, out, capacity);
            return out.asSlice(0, (long) Math.max(0, Math.min(written, capacity)) * COLLISION_EVENT.byteSize());
        } catch (Throwable throwable) {
            throw callFailed("world_get_collision_events", throwable);
        }
    }

    public int worldContactForceEventCount(MemorySegment world) {
        try {
            return (int) worldContactForceEventCount.invokeExact(world);
        } catch (Throwable throwable) {
            throw callFailed("world_contact_force_event_count", throwable);
        }
    }

    public MemorySegment worldGetContactForceEvents(MemorySegment world, int capacity) {
        MemorySegment out = arena.allocate(CONTACT_FORCE_EVENT, Math.max(1, capacity));
        if (capacity <= 0) {
            return out.asSlice(0, 0);
        }
        try {
            int written = (int) worldGetContactForceEvents.invokeExact(world, out, capacity);
            return out.asSlice(0, (long) Math.max(0, Math.min(written, capacity)) * CONTACT_FORCE_EVENT.byteSize());
        } catch (Throwable throwable) {
            throw callFailed("world_get_contact_force_events", throwable);
        }
    }

    public MemorySegment rigidBodyBuilderCreate(int status) {
        try {
            return (MemorySegment) rigidBodyBuilderCreate.invokeExact(status);
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_builder_create", throwable);
        }
    }

    public void rigidBodyBuilderDestroy(MemorySegment builder) {
        try {
            rigidBodyBuilderDestroy.invokeExact(builder);
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_builder_destroy", throwable);
        }
    }

    public void rigidBodyBuilderSetTranslation(MemorySegment builder, double x, double y, double z) {
        try {
            rigidBodyBuilderSetTranslation.invokeExact(builder, vec3(x, y, z));
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_builder_set_translation", throwable);
        }
    }

    public MemorySegment rigidBodyBuilderBuild(MemorySegment builder) {
        try {
            return (MemorySegment) rigidBodyBuilderBuild.invokeExact(builder);
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_builder_build", throwable);
        }
    }

    public long worldInsertRigidBody(MemorySegment world, MemorySegment body) {
        try {
            return (long) worldInsertRigidBody.invokeExact(world, body);
        } catch (Throwable throwable) {
            throw callFailed("world_insert_rigid_body", throwable);
        }
    }

    public MemorySegment rigidBodyGetTranslation(MemorySegment world, long body) {
        MemorySegment out = arena.allocate(VEC3);
        try {
            rigidBodyGetTranslationOut.invokeExact(world, body, out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_get_translation_out", throwable);
        }
    }

    public MemorySegment rigidBodyGetRotation(MemorySegment world, long body) {
        MemorySegment out = arena.allocate(QUAT);
        try {
            rigidBodyGetRotationOut.invokeExact(world, body, out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_get_rotation_out", throwable);
        }
    }

    public boolean rigidBodySetPose(
            MemorySegment world,
            long body,
            double x, double y, double z,
            double qi, double qj, double qk, double qw,
            boolean wakeUp) {
        try {
            return ((byte) rigidBodySetPose.invokeExact(
                    world, body, vec3(x, y, z), quat(qi, qj, qk, qw), bool(wakeUp))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_set_pose", throwable);
        }
    }

    public boolean rigidBodySetTranslation(MemorySegment world, long body, double x, double y, double z, boolean wakeUp) {
        try {
            return ((byte) rigidBodySetTranslation.invokeExact(world, body, vec3(x, y, z), bool(wakeUp))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_set_translation", throwable);
        }
    }

    public boolean rigidBodySetRotation(MemorySegment world, long body, double qi, double qj, double qk, double qw, boolean wakeUp) {
        try {
            return ((byte) rigidBodySetRotation.invokeExact(world, body, quat(qi, qj, qk, qw), bool(wakeUp))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_set_rotation", throwable);
        }
    }

    public MemorySegment rigidBodyGetLinvel(MemorySegment world, long body) {
        MemorySegment out = arena.allocate(VEC3);
        try {
            rigidBodyGetLinvelOut.invokeExact(world, body, out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_get_linvel_out", throwable);
        }
    }

    public boolean rigidBodySetLinvel(MemorySegment world, long body, double x, double y, double z, boolean wakeUp) {
        try {
            return ((byte) rigidBodySetLinvel.invokeExact(world, body, vec3(x, y, z), bool(wakeUp))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_set_linvel", throwable);
        }
    }

    public MemorySegment rigidBodyGetAngvel(MemorySegment world, long body) {
        MemorySegment out = arena.allocate(VEC3);
        try {
            rigidBodyGetAngvelOut.invokeExact(world, body, out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_get_angvel_out", throwable);
        }
    }

    public boolean rigidBodySetAngvel(MemorySegment world, long body, double x, double y, double z, boolean wakeUp) {
        try {
            return ((byte) rigidBodySetAngvel.invokeExact(world, body, vec3(x, y, z), bool(wakeUp))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_set_angvel", throwable);
        }
    }

    public boolean rigidBodyAddForce(MemorySegment world, long body, double x, double y, double z, boolean wakeUp) {
        try {
            return ((byte) rigidBodyAddForce.invokeExact(world, body, vec3(x, y, z), bool(wakeUp))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_add_force", throwable);
        }
    }

    public boolean rigidBodyAddTorque(MemorySegment world, long body, double x, double y, double z, boolean wakeUp) {
        try {
            return ((byte) rigidBodyAddTorque.invokeExact(world, body, vec3(x, y, z), bool(wakeUp))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_add_torque", throwable);
        }
    }

    public boolean rigidBodyApplyImpulse(MemorySegment world, long body, double x, double y, double z, boolean wakeUp) {
        try {
            return ((byte) rigidBodyApplyImpulse.invokeExact(world, body, vec3(x, y, z), bool(wakeUp))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_apply_impulse", throwable);
        }
    }

    public boolean rigidBodyApplyTorqueImpulse(MemorySegment world, long body, double x, double y, double z, boolean wakeUp) {
        try {
            return ((byte) rigidBodyApplyTorqueImpulse.invokeExact(world, body, vec3(x, y, z), bool(wakeUp))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_apply_torque_impulse", throwable);
        }
    }

    public boolean rigidBodyEnableCcd(MemorySegment world, long body, boolean enabled) {
        try {
            return ((byte) rigidBodyEnableCcd.invokeExact(world, body, bool(enabled))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_enable_ccd", throwable);
        }
    }

    public boolean rigidBodySleep(MemorySegment world, long body) {
        try {
            return ((byte) rigidBodySleepFlag.invokeExact(world, body)) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_sleep_flag", throwable);
        }
    }

    public boolean rigidBodyWakeUp(MemorySegment world, long body, boolean strong) {
        try {
            return ((byte) rigidBodyWakeUpFlag.invokeExact(world, body, bool(strong))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_wake_up_flag", throwable);
        }
    }

    public boolean rigidBodyIsSleeping(MemorySegment world, long body) {
        try {
            return ((byte) rigidBodyIsSleepingFlag.invokeExact(world, body)) != 0;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_is_sleeping_flag", throwable);
        }
    }

    public MemorySegment aeroApplyVoxelGrid(
            MemorySegment world,
            long body,
            double windX, double windY, double windZ,
            double airDensity,
            byte[] voxels,
            int sizeX, int sizeY, int sizeZ,
            double voxelSize,
            double originX, double originY, double originZ,
            double dragCoefficient,
            double liftCoefficient,
            boolean wakeUp) {
        if (voxels.length != Math.multiplyExact(Math.multiplyExact(sizeX, sizeY), sizeZ)) {
            throw new IllegalArgumentException("voxel array length does not match dimensions");
        }
        MemorySegment voxelMemory = arena.allocate(ValueLayout.JAVA_BYTE, voxels.length);
        for (int i = 0; i < voxels.length; i++) {
            voxelMemory.setAtIndex(ValueLayout.JAVA_BYTE, i, voxels[i]);
        }
        MemorySegment report = arena.allocate(AERO_REPORT);
        try {
            byte ok = (byte) aeroApplyVoxelGrid.invokeExact(
                    world,
                    body,
                    vec3(windX, windY, windZ),
                    airDensity,
                    voxelMemory,
                    sizeX,
                    sizeY,
                    sizeZ,
                    voxelSize,
                    vec3(originX, originY, originZ),
                    dragCoefficient,
                    liftCoefficient,
                    bool(wakeUp),
                    report);
            if (ok == 0) {
                throw new IllegalStateException("aero_apply_voxel_grid returned false");
            }
            return report;
        } catch (Throwable throwable) {
            throw callFailed("aero_apply_voxel_grid", throwable);
        }
    }

    public double spaceKeplerPeriod(double mu, double semiMajorAxis) {
        try {
            return (double) spaceKeplerPeriod.invokeExact(mu, semiMajorAxis);
        } catch (Throwable throwable) {
            throw callFailed("space_kepler_period", throwable);
        }
    }

    public double spaceKeplerSemiMajorAxis(double mu, double period) {
        try {
            return (double) spaceKeplerSemiMajorAxis.invokeExact(mu, period);
        } catch (Throwable throwable) {
            throw callFailed("space_kepler_semi_major_axis", throwable);
        }
    }

    public MemorySegment spaceHohmannTransfer(double mu, double radius1, double radius2) {
        MemorySegment out = arena.allocate(HOHMANN_TRANSFER);
        try {
            SegmentAllocator allocator = arena;
            boolean ok = boolValue((MemorySegment) spaceHohmannTransfer.invokeExact(allocator, mu, radius1, radius2, out));
            if (!ok) {
                throw new IllegalStateException("space_hohmann_transfer returned false");
            }
            return out;
        } catch (Throwable throwable) {
            throw callFailed("space_hohmann_transfer", throwable);
        }
    }

    public MemorySegment spaceAtmosphericDragAcceleration(
            double vx, double vy, double vz,
            double atmosphereVx, double atmosphereVy, double atmosphereVz,
            double density, double dragCoefficient, double area, double mass) {
        MemorySegment out = arena.allocate(VEC3);
        try {
            SegmentAllocator allocator = arena;
            boolean ok = boolValue((MemorySegment) spaceAtmosphericDragAcceleration.invokeExact(
                    allocator,
                    vec3(vx, vy, vz),
                    vec3(atmosphereVx, atmosphereVy, atmosphereVz),
                    density,
                    dragCoefficient,
                    area,
                    mass,
                    out));
            if (!ok) {
                throw new IllegalStateException("space_atmospheric_drag_acceleration returned false");
            }
            return out;
        } catch (Throwable throwable) {
            throw callFailed("space_atmospheric_drag_acceleration", throwable);
        }
    }

    public MemorySegment spaceApplyAtmosphericDragToBody(
            MemorySegment world,
            long body,
            double atmosphereVx, double atmosphereVy, double atmosphereVz,
            double density, double dragCoefficient, double area, double mass,
            boolean wakeUp) {
        MemorySegment out = arena.allocate(VEC3);
        try {
            byte ok = (byte) spaceApplyAtmosphericDragToBodyFlag.invokeExact(
                    world,
                    body,
                    vec3(atmosphereVx, atmosphereVy, atmosphereVz),
                    density,
                    dragCoefficient,
                    area,
                    mass,
                    bool(wakeUp),
                    out);
            if (ok == 0) {
                throw new IllegalStateException("space_apply_atmospheric_drag_to_body returned false");
            }
            return out;
        } catch (Throwable throwable) {
            throw callFailed("space_apply_atmospheric_drag_to_body", throwable);
        }
    }

    public MemorySegment spaceTriadAttitude(
            double bodyPrimaryX, double bodyPrimaryY, double bodyPrimaryZ,
            double bodySecondaryX, double bodySecondaryY, double bodySecondaryZ,
            double referencePrimaryX, double referencePrimaryY, double referencePrimaryZ,
            double referenceSecondaryX, double referenceSecondaryY, double referenceSecondaryZ) {
        MemorySegment out = arena.allocate(QUAT);
        try {
            SegmentAllocator allocator = arena;
            boolean ok = boolValue((MemorySegment) spaceTriadAttitude.invokeExact(
                    allocator,
                    vec3(bodyPrimaryX, bodyPrimaryY, bodyPrimaryZ),
                    vec3(bodySecondaryX, bodySecondaryY, bodySecondaryZ),
                    vec3(referencePrimaryX, referencePrimaryY, referencePrimaryZ),
                    vec3(referenceSecondaryX, referenceSecondaryY, referenceSecondaryZ),
                    out));
            if (!ok) {
                throw new IllegalStateException("space_triad_attitude returned false");
            }
            return out;
        } catch (Throwable throwable) {
            throw callFailed("space_triad_attitude", throwable);
        }
    }

    public MemorySegment spaceQuaternionDerivative(
            double qi, double qj, double qk, double qw,
            double wx, double wy, double wz) {
        MemorySegment out = arena.allocate(QUATERNION_DERIVATIVE);
        try {
            SegmentAllocator allocator = arena;
            boolean ok = boolValue((MemorySegment) spaceQuaternionDerivative.invokeExact(
                    allocator,
                    quat(qi, qj, qk, qw),
                    vec3(wx, wy, wz),
                    out));
            if (!ok) {
                throw new IllegalStateException("space_quaternion_derivative returned false");
            }
            return out;
        } catch (Throwable throwable) {
            throw callFailed("space_quaternion_derivative", throwable);
        }
    }

    public MemorySegment spaceEkfPredictScalar(double state, double covariance, double nonlinearDelta, double jacobian, double processNoise) {
        MemorySegment out = arena.allocate(SCALAR_KALMAN);
        try {
            SegmentAllocator allocator = arena;
            boolean ok = boolValue((MemorySegment) spaceEkfPredictScalar.invokeExact(
                    allocator,
                    state,
                    covariance,
                    nonlinearDelta,
                    jacobian,
                    processNoise,
                    out));
            if (!ok) {
                throw new IllegalStateException("space_ekf_predict_scalar returned false");
            }
            return out;
        } catch (Throwable throwable) {
            throw callFailed("space_ekf_predict_scalar", throwable);
        }
    }

    public double spaceEkfGainScalar(double covariance, double measurementJacobian, double measurementNoise) {
        try {
            return (double) spaceEkfGainScalar.invokeExact(covariance, measurementJacobian, measurementNoise);
        } catch (Throwable throwable) {
            throw callFailed("space_ekf_gain_scalar", throwable);
        }
    }

    public MemorySegment spaceEkfUpdateScalar(
            double predictedState, double predictedCovariance,
            double measurement, double predictedMeasurement,
            double kalmanGain, double measurementJacobian) {
        MemorySegment out = arena.allocate(SCALAR_KALMAN);
        try {
            SegmentAllocator allocator = arena;
            boolean ok = boolValue((MemorySegment) spaceEkfUpdateScalar.invokeExact(
                    allocator,
                    predictedState,
                    predictedCovariance,
                    measurement,
                    predictedMeasurement,
                    kalmanGain,
                    measurementJacobian,
                    out));
            if (!ok) {
                throw new IllegalStateException("space_ekf_update_scalar returned false");
            }
            return out;
        } catch (Throwable throwable) {
            throw callFailed("space_ekf_update_scalar", throwable);
        }
    }

    public MemorySegment crbTreeCreate() {
        try {
            return (MemorySegment) crbTreeCreate.invokeExact();
        } catch (Throwable throwable) {
            throw callFailed("crb_tree_create", throwable);
        }
    }

    public void crbTreeDestroy(MemorySegment tree) {
        try {
            crbTreeDestroy.invokeExact(tree);
        } catch (Throwable throwable) {
            throw callFailed("crb_tree_destroy", throwable);
        }
    }

    public boolean crbTreeInsert(MemorySegment tree, long id, MemorySegment aabb) {
        try {
            return ((byte) crbTreeInsertFlag.invokeExact(tree, id, aabb)) != 0;
        } catch (Throwable throwable) {
            throw callFailed("crb_tree_insert", throwable);
        }
    }

    public int crbTreeQueryAabbCount(MemorySegment tree, MemorySegment aabb) {
        try {
            return (int) crbTreeQueryAabbCount.invokeExact(tree, aabb);
        } catch (Throwable throwable) {
            throw callFailed("crb_tree_query_aabb_count", throwable);
        }
    }

    public MemorySegment rtreeCreate() {
        try {
            return (MemorySegment) rtreeCreate.invokeExact();
        } catch (Throwable throwable) {
            throw callFailed("rtree_create", throwable);
        }
    }

    public void rtreeDestroy(MemorySegment tree) {
        try {
            rtreeDestroy.invokeExact(tree);
        } catch (Throwable throwable) {
            throw callFailed("rtree_destroy", throwable);
        }
    }

    public void rtreeClear(MemorySegment tree) {
        try {
            rtreeClear.invokeExact(tree);
        } catch (Throwable throwable) {
            throw callFailed("rtree_clear", throwable);
        }
    }

    public int rtreeLen(MemorySegment tree) {
        try {
            return (int) rtreeLen.invokeExact(tree);
        } catch (Throwable throwable) {
            throw callFailed("rtree_len", throwable);
        }
    }

    public boolean rtreeInsert(MemorySegment tree, long id, MemorySegment aabb) {
        try {
            SegmentAllocator allocator = arena;
            return boolValue((MemorySegment) rtreeInsert.invokeExact(allocator, tree, id, aabb));
        } catch (Throwable throwable) {
            throw callFailed("rtree_insert", throwable);
        }
    }

    public boolean rtreeUpdate(MemorySegment tree, long id, MemorySegment aabb) {
        try {
            SegmentAllocator allocator = arena;
            return boolValue((MemorySegment) rtreeUpdate.invokeExact(allocator, tree, id, aabb));
        } catch (Throwable throwable) {
            throw callFailed("rtree_update", throwable);
        }
    }

    public boolean rtreeRemove(MemorySegment tree, long id) {
        try {
            SegmentAllocator allocator = arena;
            return boolValue((MemorySegment) rtreeRemove.invokeExact(allocator, tree, id));
        } catch (Throwable throwable) {
            throw callFailed("rtree_remove", throwable);
        }
    }

    public void rtreeRebuild(MemorySegment tree) {
        try {
            rtreeRebuild.invokeExact(tree);
        } catch (Throwable throwable) {
            throw callFailed("rtree_rebuild", throwable);
        }
    }

    public int rtreeQueryAabbCount(MemorySegment tree, MemorySegment aabb) {
        try {
            return (int) rtreeQueryAabbCount.invokeExact(tree, aabb);
        } catch (Throwable throwable) {
            throw callFailed("rtree_query_aabb_count", throwable);
        }
    }

    public long[] rtreeQueryAabb(MemorySegment tree, MemorySegment aabb, int capacity) {
        if (capacity <= 0) {
            return new long[0];
        }
        MemorySegment out = arena.allocate(ValueLayout.JAVA_LONG, capacity);
        try {
            int written = (int) rtreeQueryAabb.invokeExact(tree, aabb, out, capacity);
            return longs(out, Math.max(0, Math.min(written, capacity)));
        } catch (Throwable throwable) {
            throw callFailed("rtree_query_aabb", throwable);
        }
    }

    public MemorySegment voxelAabbBuildStats(MemorySegment aabb, double voxelSize, MemorySegment options) {
        MemorySegment out = arena.allocate(VOXEL_STATS);
        try {
            voxelAabbBuildStats.invokeExact(aabb, voxelSize, options, out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("voxel_aabb_build_stats_out", throwable);
        }
    }

    public MemorySegment voxelObbBuildStats(MemorySegment obb, double voxelSize, MemorySegment options) {
        MemorySegment out = arena.allocate(VOXEL_STATS);
        try {
            voxelObbBuildStats.invokeExact(obb, voxelSize, options, out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("voxel_obb_build_stats_out", throwable);
        }
    }

    public MemorySegment colliderBuilderCreateVoxelAabb(MemorySegment aabb, double voxelSize, MemorySegment options) {
        try {
            return (MemorySegment) colliderBuilderCreateVoxelAabb.invokeExact(aabb, voxelSize, options);
        } catch (Throwable throwable) {
            throw callFailed("collider_builder_create_voxel_aabb", throwable);
        }
    }

    public MemorySegment colliderBuilderCreate(int shapeType, double a, double b, double c) {
        try {
            return (MemorySegment) colliderBuilderCreate.invokeExact(shapeType, vec3(a, b, c));
        } catch (Throwable throwable) {
            throw callFailed("collider_builder_create", throwable);
        }
    }

    public MemorySegment colliderBuilderCreateVoxelObb(MemorySegment obb, double voxelSize, MemorySegment options) {
        try {
            return (MemorySegment) colliderBuilderCreateVoxelObb.invokeExact(obb, voxelSize, options);
        } catch (Throwable throwable) {
            throw callFailed("collider_builder_create_voxel_obb", throwable);
        }
    }

    public void colliderBuilderSetFriction(MemorySegment builder, double friction) {
        try {
            colliderBuilderSetFriction.invokeExact(builder, friction);
        } catch (Throwable throwable) {
            throw callFailed("collider_builder_set_friction", throwable);
        }
    }

    public void colliderBuilderSetRestitution(MemorySegment builder, double restitution) {
        try {
            colliderBuilderSetRestitution.invokeExact(builder, restitution);
        } catch (Throwable throwable) {
            throw callFailed("collider_builder_set_restitution", throwable);
        }
    }

    public void colliderBuilderSetDensity(MemorySegment builder, double density) {
        try {
            colliderBuilderSetDensity.invokeExact(builder, density);
        } catch (Throwable throwable) {
            throw callFailed("collider_builder_set_density", throwable);
        }
    }

    public void colliderBuilderSetSensor(MemorySegment builder, boolean sensor) {
        try {
            colliderBuilderSetSensor.invokeExact(builder, bool(sensor));
        } catch (Throwable throwable) {
            throw callFailed("collider_builder_set_sensor", throwable);
        }
    }

    public void colliderBuilderSetActiveEvents(MemorySegment builder, int bits) {
        try {
            colliderBuilderSetActiveEvents.invokeExact(builder, bits);
        } catch (Throwable throwable) {
            throw callFailed("collider_builder_set_active_events", throwable);
        }
    }

    public void colliderBuilderSetContactForceEventThreshold(MemorySegment builder, double threshold) {
        try {
            colliderBuilderSetContactForceEventThreshold.invokeExact(builder, threshold);
        } catch (Throwable throwable) {
            throw callFailed("collider_builder_set_contact_force_event_threshold", throwable);
        }
    }

    public MemorySegment colliderBuilderBuild(MemorySegment builder) {
        try {
            return (MemorySegment) colliderBuilderBuild.invokeExact(builder);
        } catch (Throwable throwable) {
            throw callFailed("collider_builder_build", throwable);
        }
    }

    public void colliderBuilderDestroy(MemorySegment builder) {
        try {
            colliderBuilderDestroy.invokeExact(builder);
        } catch (Throwable throwable) {
            throw callFailed("collider_builder_destroy", throwable);
        }
    }

    public long worldInsertCollider(MemorySegment world, MemorySegment collider) {
        try {
            return (long) worldInsertCollider.invokeExact(world, collider);
        } catch (Throwable throwable) {
            throw callFailed("world_insert_collider", throwable);
        }
    }

    public long worldInsertColliderWithParent(MemorySegment world, MemorySegment collider, long parent) {
        try {
            return (long) worldInsertColliderWithParent.invokeExact(world, collider, parent);
        } catch (Throwable throwable) {
            throw callFailed("world_insert_collider_with_parent", throwable);
        }
    }

    public MemorySegment colliderGetTranslation(MemorySegment world, long collider) {
        MemorySegment out = arena.allocate(VEC3);
        try {
            colliderGetTranslationOut.invokeExact(world, collider, out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("collider_get_translation_out", throwable);
        }
    }

    public MemorySegment colliderGetRotation(MemorySegment world, long collider) {
        MemorySegment out = arena.allocate(QUAT);
        try {
            colliderGetRotationOut.invokeExact(world, collider, out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("collider_get_rotation_out", throwable);
        }
    }

    public boolean colliderSetPose(
            MemorySegment world,
            long collider,
            double x, double y, double z,
            double qi, double qj, double qk, double qw) {
        try {
            return ((byte) colliderSetPose.invokeExact(
                    world, collider, vec3(x, y, z), quat(qi, qj, qk, qw))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("collider_set_pose", throwable);
        }
    }

    public boolean colliderSetSensor(MemorySegment world, long collider, boolean sensor) {
        try {
            return ((byte) colliderSetSensor.invokeExact(world, collider, bool(sensor))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("collider_set_sensor", throwable);
        }
    }

    public boolean colliderSetFriction(MemorySegment world, long collider, double friction) {
        try {
            return ((byte) colliderSetFriction.invokeExact(world, collider, friction)) != 0;
        } catch (Throwable throwable) {
            throw callFailed("collider_set_friction", throwable);
        }
    }

    public boolean colliderSetRestitution(MemorySegment world, long collider, double restitution) {
        try {
            return ((byte) colliderSetRestitution.invokeExact(world, collider, restitution)) != 0;
        } catch (Throwable throwable) {
            throw callFailed("collider_set_restitution", throwable);
        }
    }

    public boolean colliderSetCollisionGroups(MemorySegment world, long collider, int memberships, int filter) {
        try {
            return ((byte) colliderSetCollisionGroups.invokeExact(world, collider, interactionGroups(memberships, filter))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("collider_set_collision_groups", throwable);
        }
    }

    public boolean colliderSetSolverGroups(MemorySegment world, long collider, int memberships, int filter) {
        try {
            return ((byte) colliderSetSolverGroups.invokeExact(world, collider, interactionGroups(memberships, filter))) != 0;
        } catch (Throwable throwable) {
            throw callFailed("collider_set_solver_groups", throwable);
        }
    }

    public boolean colliderSetActiveEvents(MemorySegment world, long collider, int bits) {
        try {
            return ((byte) colliderSetActiveEvents.invokeExact(world, collider, bits)) != 0;
        } catch (Throwable throwable) {
            throw callFailed("collider_set_active_events", throwable);
        }
    }

    public boolean colliderSetActiveHooks(MemorySegment world, long collider, int bits) {
        try {
            return ((byte) colliderSetActiveHooks.invokeExact(world, collider, bits)) != 0;
        } catch (Throwable throwable) {
            throw callFailed("collider_set_active_hooks", throwable);
        }
    }

    public boolean colliderSetContactForceEventThreshold(MemorySegment world, long collider, double threshold) {
        try {
            return ((byte) colliderSetContactForceEventThreshold.invokeExact(world, collider, threshold)) != 0;
        } catch (Throwable throwable) {
            throw callFailed("collider_set_contact_force_event_threshold", throwable);
        }
    }

    public double colliderGetDensity(MemorySegment world, long collider) {
        try {
            return (double) colliderGetDensity.invokeExact(world, collider);
        } catch (Throwable throwable) {
            throw callFailed("collider_get_density", throwable);
        }
    }

    public MemorySegment queryCastRay(
            MemorySegment world,
            double ox, double oy, double oz,
            double dx, double dy, double dz,
            double maxToi,
            boolean solid) {
        MemorySegment out = arena.allocate(RAY_HIT);
        try {
            long ignored = (long) queryCastRayOut.invokeExact(
                    world,
                    vec3(ox, oy, oz),
                    vec3(dx, dy, dz),
                    maxToi,
                    bool(solid),
                    queryFilterAll(),
                    out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("query_cast_ray_out", throwable);
        }
    }

    public MemorySegment queryProjectPoint(
            MemorySegment world,
            double x, double y, double z,
            double maxDist,
            boolean solid) {
        MemorySegment outCollider = arena.allocate(ValueLayout.JAVA_LONG);
        MemorySegment outProjection = arena.allocate(POINT_PROJECTION);
        try {
            long ignored = (long) queryProjectPointOut.invokeExact(
                    world,
                    vec3(x, y, z),
                    maxDist,
                    bool(solid),
                    queryFilterAll(),
                    outCollider,
                    outProjection);
            return outProjection;
        } catch (Throwable throwable) {
            throw callFailed("query_project_point_out", throwable);
        }
    }

    public int queryIntersectAabbCount(MemorySegment world, MemorySegment aabb) {
        try {
            return (int) queryIntersectAabbCount.invokeExact(world, aabb, queryFilterAll());
        } catch (Throwable throwable) {
            throw callFailed("query_intersect_aabb_count", throwable);
        }
    }

    public long[] queryIntersectAabb(MemorySegment world, MemorySegment aabb, int capacity) {
        if (capacity <= 0) {
            return new long[0];
        }
        MemorySegment out = arena.allocate(ValueLayout.JAVA_LONG, capacity);
        try {
            int written = (int) queryIntersectAabb.invokeExact(world, aabb, queryFilterAll(), out, capacity);
            return longs(out, Math.max(0, Math.min(written, capacity)));
        } catch (Throwable throwable) {
            throw callFailed("query_intersect_aabb", throwable);
        }
    }

    public int queryIntersectObbCount(MemorySegment world, MemorySegment obb) {
        try {
            return (int) queryIntersectObbCount.invokeExact(world, obb, queryFilterAll());
        } catch (Throwable throwable) {
            throw callFailed("query_intersect_obb_count", throwable);
        }
    }

    public long[] queryIntersectObb(MemorySegment world, MemorySegment obb, int capacity) {
        if (capacity <= 0) {
            return new long[0];
        }
        MemorySegment out = arena.allocate(ValueLayout.JAVA_LONG, capacity);
        try {
            int written = (int) queryIntersectObb.invokeExact(world, obb, queryFilterAll(), out, capacity);
            return longs(out, Math.max(0, Math.min(written, capacity)));
        } catch (Throwable throwable) {
            throw callFailed("query_intersect_obb", throwable);
        }
    }

    public int queryIntersectSphereCount(MemorySegment world, MemorySegment sphere) {
        try {
            return (int) queryIntersectSphereCount.invokeExact(world, sphere, queryFilterAll());
        } catch (Throwable throwable) {
            throw callFailed("query_intersect_sphere_count", throwable);
        }
    }

    public long[] queryIntersectSphere(MemorySegment world, MemorySegment sphere, int capacity) {
        if (capacity <= 0) {
            return new long[0];
        }
        MemorySegment out = arena.allocate(ValueLayout.JAVA_LONG, capacity);
        try {
            int written = (int) queryIntersectSphere.invokeExact(world, sphere, queryFilterAll(), out, capacity);
            return longs(out, Math.max(0, Math.min(written, capacity)));
        } catch (Throwable throwable) {
            throw callFailed("query_intersect_sphere", throwable);
        }
    }

    public MemorySegment queryCastShape(
            MemorySegment world,
            MemorySegment shape,
            MemorySegment translation,
            MemorySegment rotation,
            MemorySegment velocity,
            double maxToi) {
        MemorySegment out = arena.allocate(SHAPE_CAST_HIT);
        try {
            long ignored = (long) queryCastShapeOut.invokeExact(
                    world,
                    shape,
                    translation,
                    rotation,
                    velocity,
                    shapeCastOptions(maxToi, 0.0, true, true),
                    queryFilterAll(),
                    out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("query_cast_shape_out", throwable);
        }
    }

    public int queryIntersectVoxelAabbCount(MemorySegment world, MemorySegment aabb) {
        try {
            return (int) queryIntersectVoxelAabbCount.invokeExact(world, aabb, queryFilterAll());
        } catch (Throwable throwable) {
            throw callFailed("query_intersect_voxel_aabb_count", throwable);
        }
    }

    public long[] queryIntersectVoxelAabb(MemorySegment world, MemorySegment aabb, int capacity) {
        if (capacity <= 0) {
            return new long[0];
        }
        MemorySegment out = arena.allocate(ValueLayout.JAVA_LONG, capacity);
        try {
            int written = (int) queryIntersectVoxelAabb.invokeExact(world, aabb, queryFilterAll(), out, capacity);
            return longs(out, Math.max(0, Math.min(written, capacity)));
        } catch (Throwable throwable) {
            throw callFailed("query_intersect_voxel_aabb", throwable);
        }
    }

    public int queryIntersectVoxelObbCount(MemorySegment world, MemorySegment obb) {
        try {
            return (int) queryIntersectVoxelObbCount.invokeExact(world, obb, queryFilterAll());
        } catch (Throwable throwable) {
            throw callFailed("query_intersect_voxel_obb_count", throwable);
        }
    }

    public long[] queryIntersectVoxelObb(MemorySegment world, MemorySegment obb, int capacity) {
        if (capacity <= 0) {
            return new long[0];
        }
        MemorySegment out = arena.allocate(ValueLayout.JAVA_LONG, capacity);
        try {
            int written = (int) queryIntersectVoxelObb.invokeExact(world, obb, queryFilterAll(), out, capacity);
            return longs(out, Math.max(0, Math.min(written, capacity)));
        } catch (Throwable throwable) {
            throw callFailed("query_intersect_voxel_obb", throwable);
        }
    }

    public MemorySegment vec3(double x, double y, double z) {
        MemorySegment value = arena.allocate(VEC3);
        value.set(ValueLayout.JAVA_DOUBLE, VEC3_X, x);
        value.set(ValueLayout.JAVA_DOUBLE, VEC3_Y, y);
        value.set(ValueLayout.JAVA_DOUBLE, VEC3_Z, z);
        return value;
    }

    public MemorySegment quat(double i, double j, double k, double w) {
        MemorySegment value = arena.allocate(QUAT);
        value.set(ValueLayout.JAVA_DOUBLE, QUAT.byteOffset(MemoryLayout.PathElement.groupElement("i")), i);
        value.set(ValueLayout.JAVA_DOUBLE, QUAT.byteOffset(MemoryLayout.PathElement.groupElement("j")), j);
        value.set(ValueLayout.JAVA_DOUBLE, QUAT.byteOffset(MemoryLayout.PathElement.groupElement("k")), k);
        value.set(ValueLayout.JAVA_DOUBLE, QUAT.byteOffset(MemoryLayout.PathElement.groupElement("w")), w);
        return value;
    }

    public MemorySegment aabb(double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        MemorySegment value = arena.allocate(AABB);
        value.asSlice(0, VEC3.byteSize()).copyFrom(vec3(minX, minY, minZ));
        value.asSlice(VEC3.byteSize(), VEC3.byteSize()).copyFrom(vec3(maxX, maxY, maxZ));
        return value;
    }

    public MemorySegment sphere(double x, double y, double z, double radius) {
        MemorySegment value = arena.allocate(SPHERE);
        value.asSlice(0, VEC3.byteSize()).copyFrom(vec3(x, y, z));
        value.set(ValueLayout.JAVA_DOUBLE, SPHERE.byteOffset(MemoryLayout.PathElement.groupElement("radius")), radius);
        return value;
    }

    public MemorySegment obb(
            double cx, double cy, double cz,
            double hx, double hy, double hz,
            double qi, double qj, double qk, double qw) {
        MemorySegment value = arena.allocate(OBB);
        value.asSlice(0, VEC3.byteSize()).copyFrom(vec3(cx, cy, cz));
        value.asSlice(VEC3.byteSize(), VEC3.byteSize()).copyFrom(vec3(hx, hy, hz));
        value.asSlice(VEC3.byteSize() * 2, QUAT.byteSize()).copyFrom(quat(qi, qj, qk, qw));
        return value;
    }

    public MemorySegment voxelOptions(int mode, boolean dynamicBody, int smallVoxelLimit, int meshVoxelLimit) {
        MemorySegment value = arena.allocate(VOXEL_OPTIONS);
        value.set(ValueLayout.JAVA_INT, VOXEL_OPTIONS.byteOffset(MemoryLayout.PathElement.groupElement("mode")), mode);
        value.set(ValueLayout.JAVA_BYTE, VOXEL_OPTIONS.byteOffset(MemoryLayout.PathElement.groupElement("dynamic_body")), (byte) (dynamicBody ? 1 : 0));
        value.set(ValueLayout.JAVA_INT, VOXEL_OPTIONS.byteOffset(MemoryLayout.PathElement.groupElement("small_voxel_limit")), smallVoxelLimit);
        value.set(ValueLayout.JAVA_INT, VOXEL_OPTIONS.byteOffset(MemoryLayout.PathElement.groupElement("mesh_voxel_limit")), meshVoxelLimit);
        return value;
    }

    public MemorySegment shapeDesc(int shapeType, double a, double b, double c, double d) {
        MemorySegment value = arena.allocate(SHAPE_DESC);
        value.set(ValueLayout.JAVA_INT, SHAPE_DESC.byteOffset(MemoryLayout.PathElement.groupElement("shape_type")), shapeType);
        value.set(ValueLayout.JAVA_DOUBLE, SHAPE_DESC.byteOffset(MemoryLayout.PathElement.groupElement("a")), a);
        value.set(ValueLayout.JAVA_DOUBLE, SHAPE_DESC.byteOffset(MemoryLayout.PathElement.groupElement("b")), b);
        value.set(ValueLayout.JAVA_DOUBLE, SHAPE_DESC.byteOffset(MemoryLayout.PathElement.groupElement("c")), c);
        value.set(ValueLayout.JAVA_DOUBLE, SHAPE_DESC.byteOffset(MemoryLayout.PathElement.groupElement("d")), d);
        return value;
    }

    public MemorySegment shapeCastOptions(
            double maxTimeOfImpact,
            double targetDistance,
            boolean stopAtPenetration,
            boolean computeImpactGeometryOnPenetration) {
        MemorySegment value = arena.allocate(SHAPE_CAST_OPTIONS);
        value.set(ValueLayout.JAVA_DOUBLE, SHAPE_CAST_OPTIONS.byteOffset(MemoryLayout.PathElement.groupElement("max_time_of_impact")), maxTimeOfImpact);
        value.set(ValueLayout.JAVA_DOUBLE, SHAPE_CAST_OPTIONS.byteOffset(MemoryLayout.PathElement.groupElement("target_distance")), targetDistance);
        value.set(ValueLayout.JAVA_BYTE, SHAPE_CAST_OPTIONS.byteOffset(
                MemoryLayout.PathElement.groupElement("stop_at_penetration"),
                MemoryLayout.PathElement.groupElement("_0")), (byte) (stopAtPenetration ? 1 : 0));
        value.set(ValueLayout.JAVA_BYTE, SHAPE_CAST_OPTIONS.byteOffset(
                MemoryLayout.PathElement.groupElement("compute_impact_geometry_on_penetration"),
                MemoryLayout.PathElement.groupElement("_0")), (byte) (computeImpactGeometryOnPenetration ? 1 : 0));
        return value;
    }

    public MemorySegment queryFilterAll() {
        MemorySegment value = arena.allocate(QUERY_FILTER);
        value.set(ValueLayout.JAVA_INT, QUERY_FILTER.byteOffset(MemoryLayout.PathElement.groupElement("flags")), 0);
        value.set(ValueLayout.JAVA_INT, QUERY_FILTER.byteOffset(
                MemoryLayout.PathElement.groupElement("groups"),
                MemoryLayout.PathElement.groupElement("memberships")), 0xffff);
        value.set(ValueLayout.JAVA_INT, QUERY_FILTER.byteOffset(
                MemoryLayout.PathElement.groupElement("groups"),
                MemoryLayout.PathElement.groupElement("filter")), 0xffff);
        return value;
    }

    public MemorySegment interactionGroups(int memberships, int filter) {
        MemorySegment value = arena.allocate(INTERACTION_GROUPS);
        value.set(ValueLayout.JAVA_INT, INTERACTION_GROUPS.byteOffset(MemoryLayout.PathElement.groupElement("memberships")), memberships);
        value.set(ValueLayout.JAVA_INT, INTERACTION_GROUPS.byteOffset(MemoryLayout.PathElement.groupElement("filter")), filter);
        return value;
    }

    public MemorySegment bool(boolean value) {
        MemorySegment segment = arena.allocate(BOOL);
        segment.set(ValueLayout.JAVA_BYTE, 0, (byte) (value ? 1 : 0));
        return segment;
    }

    public static int voxelStatsSolidCount(MemorySegment stats) {
        return stats.get(ValueLayout.JAVA_INT, VOXEL_STATS_SOLID_COUNT);
    }

    public static int voxelStatsSelectedMode(MemorySegment stats) {
        return stats.get(ValueLayout.JAVA_INT, VOXEL_STATS_SELECTED_MODE);
    }

    public static MemorySegment aeroReportTotalForce(MemorySegment report) {
        return report.asSlice(AERO_REPORT_TOTAL_FORCE, VEC3.byteSize());
    }

    public static int aeroReportSurfaceCount(MemorySegment report) {
        return report.get(ValueLayout.JAVA_INT, AERO_REPORT_SURFACE_COUNT);
    }

    public static int aeroReportActiveSurfaceCount(MemorySegment report) {
        return report.get(ValueLayout.JAVA_INT, AERO_REPORT_ACTIVE_SURFACE_COUNT);
    }

    public static double hohmannTotalDeltaV(MemorySegment transfer) {
        return transfer.get(ValueLayout.JAVA_DOUBLE, HOHMANN_TOTAL_DELTA_V);
    }

    public static double hohmannTransferTime(MemorySegment transfer) {
        return transfer.get(ValueLayout.JAVA_DOUBLE, HOHMANN_TRANSFER_TIME);
    }

    public static double scalarKalmanValue(MemorySegment state) {
        return state.get(ValueLayout.JAVA_DOUBLE, SCALAR_KALMAN_VALUE);
    }

    public static double scalarKalmanCovariance(MemorySegment state) {
        return state.get(ValueLayout.JAVA_DOUBLE, SCALAR_KALMAN_COVARIANCE);
    }

    public static double x(MemorySegment vec3) {
        return vec3.get(ValueLayout.JAVA_DOUBLE, VEC3_X);
    }

    public static double y(MemorySegment vec3) {
        return vec3.get(ValueLayout.JAVA_DOUBLE, VEC3_Y);
    }

    public static double z(MemorySegment vec3) {
        return vec3.get(ValueLayout.JAVA_DOUBLE, VEC3_Z);
    }

    public static boolean boolValue(MemorySegment bool) {
        return bool.get(ValueLayout.JAVA_BYTE, 0) != 0;
    }

    public static long rayHitCollider(MemorySegment hit) {
        return hit.get(ValueLayout.JAVA_LONG, RAY_HIT_COLLIDER);
    }

    public static double rayHitTimeOfImpact(MemorySegment hit) {
        return hit.get(ValueLayout.JAVA_DOUBLE, RAY_HIT_TOI);
    }

    public static boolean pointProjectionInside(MemorySegment projection) {
        return projection.get(ValueLayout.JAVA_BYTE, POINT_PROJECTION_INSIDE) != 0;
    }

    public static long shapeCastHitCollider(MemorySegment hit) {
        return hit.get(ValueLayout.JAVA_LONG, SHAPE_CAST_HIT_COLLIDER);
    }

    public static double shapeCastHitTimeOfImpact(MemorySegment hit) {
        return hit.get(ValueLayout.JAVA_DOUBLE, SHAPE_CAST_HIT_TOI);
    }

    public static int eventCount(MemorySegment events, MemoryLayout eventLayout) {
        return (int) (events.byteSize() / eventLayout.byteSize());
    }

    public static boolean collisionEventStarted(MemorySegment events, int index) {
        return events.get(ValueLayout.JAVA_BYTE, eventOffset(COLLISION_EVENT, index, COLLISION_EVENT_STARTED)) != 0;
    }

    public static long collisionEventCollider1(MemorySegment events, int index) {
        return events.get(ValueLayout.JAVA_LONG, eventOffset(COLLISION_EVENT, index, COLLISION_EVENT_COLLIDER1));
    }

    public static long collisionEventCollider2(MemorySegment events, int index) {
        return events.get(ValueLayout.JAVA_LONG, eventOffset(COLLISION_EVENT, index, COLLISION_EVENT_COLLIDER2));
    }

    public static long contactForceEventCollider1(MemorySegment events, int index) {
        return events.get(ValueLayout.JAVA_LONG, eventOffset(CONTACT_FORCE_EVENT, index, CONTACT_FORCE_EVENT_COLLIDER1));
    }

    public static long contactForceEventCollider2(MemorySegment events, int index) {
        return events.get(ValueLayout.JAVA_LONG, eventOffset(CONTACT_FORCE_EVENT, index, CONTACT_FORCE_EVENT_COLLIDER2));
    }

    public static double contactForceEventTotalForceMagnitude(MemorySegment events, int index) {
        return events.get(ValueLayout.JAVA_DOUBLE, eventOffset(CONTACT_FORCE_EVENT, index, CONTACT_FORCE_EVENT_TOTAL_FORCE_MAGNITUDE));
    }

    private static long eventOffset(MemoryLayout layout, int index, long fieldOffset) {
        return (long) index * layout.byteSize() + fieldOffset;
    }

    private static long[] longs(MemorySegment values, int count) {
        long[] longs = new long[count];
        for (int i = 0; i < count; i++) {
            longs[i] = values.getAtIndex(ValueLayout.JAVA_LONG, i);
        }
        return longs;
    }

    private MethodHandle downcall(String symbol, FunctionDescriptor descriptor) {
        MemorySegment address = lookup.find(symbol)
                .orElseThrow(() -> new UnsatisfiedLinkError("missing native symbol: " + symbol));
        return LINKER.downcallHandle(address, descriptor);
    }

    private static IllegalStateException callFailed(String symbol, Throwable throwable) {
        return new IllegalStateException("native call failed: " + symbol, throwable);
    }
}
