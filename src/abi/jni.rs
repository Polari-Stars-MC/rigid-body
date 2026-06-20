use crate::abi::ffm as abi;
use crate::helper::helper::{jbytearray_to_array, jdoublearray_to_array};
use crate::rapier::ffi::{
    AabbDesc, Bool, CRbTreeHandle as CRTH,
    Capsule, CharacterCollision, CharacterControllerHandle as CCH, ColliderBuilderHandle as CBH,
    ColliderHandleRaw as CRaw, CollisionEventRecord as CER, ContactForceEventRecord, Cylinder,
    EffectiveCharacterMovement, Ellipsoid, HohmannTransfer,
    ImpulseJointHandleRaw as JRaw, InteractionGroupsDesc, JointBuilderHandle as JBH,
    NeuralBoundsDesc, Obb, PointProjection, Prism, Quat, QueryFilterDesc,
    QuaternionDerivative, RTreeHandle as RTH, RayHit, RigidBodyBuilderHandle as RBH,
    RigidBodyHandleRaw as RRaw, ScalarKalman, ShapeCastHit, ShapeCastOptionsDesc, ShapeDesc,
    Sphere, SphericalShell, Ssv, Vec3, VoxelColliderOptions, WorldHandle as WH,
};
#[cfg(feature = "anvilkit-bridge")]
use crate::rapier::ffi::{
    AeroForceReport, AeroSurface, AnvilKitAppHandle as AKH, FluidForceReport, FluidVolume,
    TrajectoryEnvironment, TrajectoryForceReport,
};
use crate::rapier::{
    bounds as bo, collider as col, compat as com, controller as cc, crbtree as crt, dop,
    error as er, events as ev, joints as jo, neural as neu, query as qu, rigid_body as rb,
    rtree as rt, spaceflight as sf, voxel as vx, world as wo,
};
#[cfg(feature = "anvilkit-bridge")]
use crate::rapier::anvilkit as ak;
use ljni::JNIEnv;
use ljni::sys::{jbyte, jbyteArray, jclass, jdouble, jdoubleArray, jint, jlong, jstring};
use rapier3d::prelude::{Collider as CB, RigidBody as RB};
use std::panic::{AssertUnwindSafe, catch_unwind};

fn to_jlong<T>(value: *mut T) -> jlong {
    value as isize as jlong
}

fn m<T>(value: jlong) -> *mut T {
    value as isize as *mut T
}

fn cp<T>(value: jlong) -> *const T {
    value as isize as *const T
}

fn p<T>(value: jlong) -> *const T {
    value as isize as *const T
}

fn pm<T>(value: jlong) -> *mut T {
    value as isize as *mut T
}

fn jb(value: jint) -> Bool {
    Bool((value != 0) as u8)
}

fn u32_from_jint(value: jint) -> u32 {
    u32::try_from(value).unwrap_or(0)
}

fn v3(x: jdouble, y: jdouble, z: jdouble) -> Vec3 {
    Vec3 { x, y, z }
}

fn qt(i: jdouble, j: jdouble, k: jdouble, w: jdouble) -> Quat {
    Quat { i, j, k, w }
}

fn grp(memberships: jint, filter: jint) -> InteractionGroupsDesc {
    InteractionGroupsDesc {
        memberships: memberships as u32,
        filter: filter as u32,
    }
}

fn aa(
    min_x: jdouble,
    min_y: jdouble,
    min_z: jdouble,
    max_x: jdouble,
    max_y: jdouble,
    max_z: jdouble,
) -> AabbDesc {
    AabbDesc {
        mins: v3(min_x, min_y, min_z),
        maxs: v3(max_x, max_y, max_z),
    }
}

fn qfilter(
    flags: jint,
    memberships: jint,
    filter: jint,
    use_groups: jint,
    exclude_collider: jlong,
    use_exclude_collider: jint,
    exclude_rigid_body: jlong,
    use_exclude_rigid_body: jint,
) -> QueryFilterDesc {
    QueryFilterDesc {
        flags: flags as u32,
        groups: grp(memberships, filter),
        use_groups: jb(use_groups),
        exclude_collider: exclude_collider as CRaw,
        use_exclude_collider: jb(use_exclude_collider),
        exclude_rigid_body: exclude_rigid_body as RRaw,
        use_exclude_rigid_body: jb(use_exclude_rigid_body),
    }
}

fn shape_type(value: jint) -> u32 {
    u32_from_jint(value)
}

fn body_status(value: jint) -> u32 {
    u32_from_jint(value)
}

fn joint_type(value: jint) -> u32 {
    u32_from_jint(value)
}

fn joint_axis(value: jint) -> u32 {
    u32_from_jint(value)
}

fn kdop_preset(value: jint) -> u32 {
    u32_from_jint(value)
}

fn neural_activation(value: jint) -> u32 {
    u32_from_jint(value)
}

fn voxel_mode(value: jint) -> u32 {
    u32_from_jint(value)
}

fn vec3_to_j_double_array(_env: JNIEnv, vec3: Vec3) -> jdoubleArray {
    let Ok(arr) = _env.new_double_array(3) else {
        return std::ptr::null_mut();
    };
    if _env
        .set_double_array_region(&arr, 0, &[vec3.x, vec3.y, vec3.z])
        .is_err()
    {
        return std::ptr::null_mut();
    }
    arr.as_raw()
}

fn quat_to_j_double_array(_env: JNIEnv, quat: Quat) -> jdoubleArray {
    let Ok(arr) = _env.new_double_array(4) else {
        return std::ptr::null_mut();
    };
    if _env
        .set_double_array_region(&arr, 0, &[quat.i, quat.j, quat.k, quat.w])
        .is_err()
    {
        return std::ptr::null_mut();
    }
    arr.as_raw()
}

fn sd(shape_type: jint, a: jdouble, b: jdouble, c: jdouble, d: jdouble) -> ShapeDesc {
    ShapeDesc {
        shape_type: self::shape_type(shape_type),
        a,
        b,
        c,
        d,
    }
}

macro_rules! jni {
    (@ty long) => { jlong };
    (@ty boolean) => { jbyte };
    (@ty byte_array) => { jbyteArray };
    (@ty double) => { jdouble };
    (@ty int) => { jint };
    (@ty void) => { () };
    (@ty double_array) => { jdoubleArray };
    (@ty long_array) => { jlongArray };
    (@ty bool_array) => { jbooleanArray };
    (@default long) => { 0 };
    (@default boolean) => { 0 };
    (@default byte_array) => { std::ptr::null_mut() };
    (@default double) => { 0.0 };
    (@default int) => { 0 };
    (@default void) => { () };
    (@default double_array) => { std::ptr::null_mut() };
    (@default long_array) => { std::ptr::null_mut() };
    (@default bool_array) => { std::ptr::null_mut() };
    ($ret:ident $method:ident ( $($kind:ident $arg:ident),* ) $body:block) => {
        #[unsafe(export_name = concat!(
            "Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_",
            stringify!($method)
        ))]
        #[allow(non_snake_case)]
        pub extern "system" fn $method(_env: JNIEnv, _class: jclass, $($arg: jni!(@ty $kind)),*) -> jni!(@ty $ret) {
            catch_unwind(AssertUnwindSafe(|| $body)).unwrap_or(jni!(@default $ret))
        }
    };
}

macro_rules! jni_e_c {
    (@ty long) => { jlong };
    (@ty boolean) => { jbyte };
    (@ty byte_array) => { jbyteArray };
    (@ty double) => { jdouble };
    (@ty int) => { jint };
    (@ty void) => { () };
    (@ty double_array) => { jdoubleArray };
    (@ty long_array) => { jlongArray };
    (@ty bool_array) => { jbooleanArray };
    (@ty env) => { JNIEnv };
    (@ty class) => { jclass };
    (@default long) => { 0 };
    (@default boolean) => { 0 };
    (@default byte_array) => { std::ptr::null_mut() };
    (@default double) => { 0.0 };
    (@default int) => { 0 };
    (@default void) => { () };
    (@default double_array) => { std::ptr::null_mut() };
    (@default long_array) => { std::ptr::null_mut() };
    (@default bool_array) => { std::ptr::null_mut() };
    ($ret:ident $method:ident ( $($kind:ident $arg:ident),* ) $body:block) => {
        #[unsafe(export_name = concat!(
            "Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_",
            stringify!($method)
        ))]
        #[allow(non_snake_case)]
        pub extern "system" fn $method( $($arg: jni_e_c!(@ty $kind)),*) -> jni_e_c!(@ty $ret) {
            catch_unwind(AssertUnwindSafe(|| $body)).unwrap_or(jni_e_c!(@default $ret))
        }
    };
}

jni!(int abiVersion() { abi::abi_version() as jint });
jni!(boolean abiSupportsFfm() { abi::abi_supports_ffm().0 as jbyte });
jni!(boolean abiSupportsJni() { abi::abi_supports_jni().0 as jbyte });
jni!(int abiLastErrorCode() { er::last_error_code() as jint });
jni!(void abiClearLastError() { er::last_error_clear(); });

#[unsafe(export_name = "Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_abiLastErrorMessage")]
#[allow(non_snake_case)]
pub extern "system" fn abiLastErrorMessage(env: JNIEnv, _class: jclass) -> jstring {
    catch_unwind(AssertUnwindSafe(|| {
        let ptr = er::last_error_message();
        if ptr.is_null() {
            return std::ptr::null_mut();
        }
        let message = unsafe { std::ffi::CStr::from_ptr(ptr) }
            .to_string_lossy()
            .into_owned();
        env.new_string(message)
            .map(|value| value.as_raw())
            .unwrap_or(std::ptr::null_mut())
    }))
    .unwrap_or(std::ptr::null_mut())
}

//世界管理
jni!(long worldCreate(double gravity_x, double gravity_y, double gravity_z) { to_jlong(wo::world_create(v3(gravity_x, gravity_y, gravity_z))) });
jni!(void worldDestroy(long world) { wo::world_destroy(m::<WH>(world)); });
jni!(void worldStep(long world, double delta_seconds) { wo::world_step(m::<WH>(world), delta_seconds); });

jni!(void worldSetGravity(long world, double x, double y, double z) { wo::world_set_gravity(m::<WH>(world), v3(x, y, z)); });

jni_e_c!(double_array worldGetGravity(env _env, class _class, long world) { vec3_to_j_double_array(_env, wo::world_get_gravity(cp::<WH>(world))) });
jni!(void worldGetGravityOut(long world, long out_gravity) { wo::world_get_gravity_out(cp::<WH>(world), pm::<Vec3>(out_gravity)); });
jni!(int worldGetRigidBodySetSize(long world) { wo::world_get_rigid_body_set_size(cp::<WH>(world)) });
jni!(int worldGetColliderSetSize(long world) { wo::world_get_collider_set_size(cp::<WH>(world)) });

jni!(int worldDynamicBodySnapshotCount(long world) { wo::world_dynamic_body_snapshot_count(cp::<WH>(world)) as jint });
jni!(int worldDynamicBodySnapshot(long world, long out_handles, long out_values, int capacity) { wo::world_dynamic_body_snapshot(cp::<WH>(world),pm::<RRaw>(out_handles),pm::<f64>(out_values),u32_from_jint(capacity)) as jint });
jni!(boolean worldSetIntegrationParameters(long world, double dt, int solver_iterations, int ccd_substeps) { wo::world_set_integration_parameters(m::<WH>(world), dt, u32_from_jint(solver_iterations), u32_from_jint(ccd_substeps)).0 as jbyte });
jni!(int worldGetIntegrationParameters(long world, long out_values, int capacity) { wo::world_get_integration_parameters(cp::<WH>(world), pm::<f64>(out_values), u32_from_jint(capacity)) as jint });
jni!(int worldBodySnapshotCount(long world) { wo::world_body_snapshot_count(cp::<WH>(world)) as jint });
jni!(int worldBodySnapshot(long world, long out_handles, long out_values, int capacity) { wo::world_body_snapshot(cp::<WH>(world), pm::<RRaw>(out_handles), pm::<f64>(out_values), u32_from_jint(capacity)) as jint });
jni!(int worldUpdateBodyPoses(long world, long handles, long values, int count, int wake_up) { wo::world_update_body_poses(m::<WH>(world), p::<RRaw>(handles), p::<f64>(values), u32_from_jint(count), jb(wake_up)) as jint });
jni!(int worldUpdateBodyVelocities(long world, long handles, long values, int count, int wake_up) { wo::world_update_body_velocities(m::<WH>(world), p::<RRaw>(handles), p::<f64>(values), u32_from_jint(count), jb(wake_up)) as jint });

//世界插入
jni!(long worldInsertRigidBody(long world, long memory_handle) { rb::world_insert_rigid_body(m::<WH>(world), m::<RB>(memory_handle)) as jlong });
jni!(boolean worldRemoveRigidBody(long world, long handle, int remove_attached_colliders) { rb::world_remove_rigid_body(m::<WH>(world), handle as RRaw, jb(remove_attached_colliders)).0 as jbyte });
jni!(long worldCopyRigidBody(long world, long handle) { rb::world_copy_rigid_body(m::<WH>(world), handle as RRaw) as jlong });
jni!(void rigidBodyDestroyRaw(long rigid_body) { rb::rigid_body_destroy_raw(m::<RB>(rigid_body)); });
jni!(long worldInsertCollider(long world, long memory_handle) { col::world_insert_collider(m::<WH>(world), m::<CB>(memory_handle)) as jlong });
jni!(long worldInsertColliderWithParent(long world, long memory_handle, long parent) { col::world_insert_collider_with_parent(m::<WH>(world), m::<CB>(memory_handle), parent as RRaw) as jlong });
jni!(boolean worldRemoveCollider(long world, long handle, int wake_up) { col::world_remove_collider(m::<WH>(world), handle as CRaw, jb(wake_up)).0 as jbyte });
jni!(long worldCopyCollider(long world, long handle)  { col::world_copy_collider(m::<WH>(world), handle as CRaw) as jlong });
jni!(void colliderDestroyRaw(long collider) { col::collider_destroy_raw(m::<CB>(collider)); });

jni!(long colliderBuilderCreate(int shape_type, double a, double b, double c) { to_jlong(col::collider_builder_create(self::shape_type(shape_type), v3(a, b, c))) });
jni_e_c!(long colliderBuilderCreateHeightmap(env _env, class _class, double_array data, int data_x, int data_y, double scale_x, double scale_y, double scale_z) {
    let Some(values) = jdoublearray_to_array(&_env, data) else {
        return 0;
    };
    to_jlong(col::collider_builder_create_heightmap(values.as_ptr(), u32_from_jint(data_x), u32_from_jint(data_y), Vec3 { x: scale_x, y: scale_y, z: scale_z }))
});
jni!(long colliderBuilderCreateEx(int shape_type, double a, double b, double c, double d) { to_jlong(col::collider_builder_create_ex(sd(shape_type, a, b, c, d))) });
jni!(long colliderBuilderCreateSphere(double x, double y, double z, double radius) { to_jlong(col::collider_builder_create_sphere(Sphere { center: v3(x, y, z), radius })) });
jni!(long colliderBuilderCreateObb(double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw) { to_jlong(col::collider_builder_create_obb(Obb {center: v3(cx, cy, cz),half_extents: v3(hx, hy, hz),rotation: qt(qi, qj, qk, qw),})) });
jni!(long colliderBuilderCreateConvexHull(long points_xyz, int point_count) { to_jlong(col::collider_builder_create_convex_hull(p::<f64>(points_xyz), u32_from_jint(point_count))) });
jni!(long colliderBuilderCreatePointCloudBounds(long points_xyz, int point_count) { to_jlong(col::collider_builder_create_point_cloud_bounds(p::<f64>(points_xyz), u32_from_jint(point_count))) });
jni!(long colliderBuilderCreateDoubleBv(double a_min_x, double a_min_y, double a_min_z, double a_max_x, double a_max_y, double a_max_z, double b_min_x, double b_min_y, double b_min_z, double b_max_x, double b_max_y, double b_max_z) { to_jlong(col::collider_builder_create_double_bv(aa(a_min_x,a_min_y,a_min_z,a_max_x,a_max_y,a_max_z), aa(b_min_x,b_min_y,b_min_z,b_max_x,b_max_y,b_max_z))) });
jni!(long colliderBuilderCreateSkewedObb(double cx, double cy, double cz, double ax_x, double ax_y, double ax_z, double ay_x, double ay_y, double ay_z, double az_x, double az_y, double az_z) { to_jlong(col::collider_builder_create_skewed_obb(v3(cx,cy,cz), v3(ax_x,ax_y,ax_z), v3(ay_x,ay_y,ay_z), v3(az_x,az_y,az_z))) });
jni!(long colliderBuilderCreateDiscreteObb(long points_xyz, int point_count, int axis) { to_jlong(col::collider_builder_create_discrete_obb(p::<f64>(points_xyz), u32_from_jint(point_count), u32_from_jint(axis))) });
jni!(long colliderBuilderCreateFusedCollapsingBounds(long points_xyz, int point_count, double padding) { to_jlong(col::collider_builder_create_fused_collapsing_bounds(p::<f64>(points_xyz), u32_from_jint(point_count), padding)) });
jni!(long colliderBuilderCreateEdgeBvh(long vertices_xyz, int vertex_count, long edges, int edge_count, double radius) { to_jlong(col::collider_builder_create_edge_bvh(p::<f64>(vertices_xyz), u32_from_jint(vertex_count), p::<u32>(edges), u32_from_jint(edge_count), radius)) });
jni!(long colliderBuilderCreateMedialSpheres(long spheres_xyzw, int sphere_count) { to_jlong(col::collider_builder_create_medial_spheres(p::<f64>(spheres_xyzw), u32_from_jint(sphere_count))) });
jni!(long colliderBuilderCreateCapsule(double ax, double ay, double az, double bx, double by, double bz, double radius) { to_jlong(bo::collider_builder_create_capsule(Capsule { a: v3(ax, ay, az), b: v3(bx, by, bz), radius })) });
jni!(long colliderBuilderCreateSsv(double ax, double ay, double az, double bx, double by, double bz, double radius) { to_jlong(bo::collider_builder_create_ssv(Ssv { a: v3(ax, ay, az), b: v3(bx, by, bz), radius })) });
jni!(long colliderBuilderCreateEllipsoid(double cx, double cy, double cz, double rx, double ry, double rz, double qi, double qj, double qk, double qw, int segments) { to_jlong(bo::collider_builder_create_ellipsoid(Ellipsoid { center: v3(cx, cy, cz), radii: v3(rx, ry, rz), rotation: qt(qi, qj, qk, qw), segments: u32_from_jint(segments) })) });
jni!(long colliderBuilderCreatePrism(double cx, double cy, double cz, double radius, double half_height, int sides, double qi, double qj, double qk, double qw) { to_jlong(bo::collider_builder_create_prism(Prism { center: v3(cx, cy, cz), radius, half_height, sides: u32_from_jint(sides), rotation: qt(qi, qj, qk, qw) })) });
jni!(long colliderBuilderCreateCylinder(double cx, double cy, double cz, double radius, double half_height, double qi, double qj, double qk, double qw) { to_jlong(bo::collider_builder_create_cylinder(Cylinder { center: v3(cx, cy, cz), radius, half_height, rotation: qt(qi, qj, qk, qw) })) });
jni!(long colliderBuilderCreateSphericalShell(double cx, double cy, double cz, double inner_radius, double outer_radius) { to_jlong(bo::collider_builder_create_spherical_shell(SphericalShell { center: v3(cx, cy, cz), inner_radius, outer_radius })) });
jni!(long colliderBuilderCreateKdop(long points_xyz, int point_count, int preset) { to_jlong(dop::collider_builder_create_kdop(p::<f64>(points_xyz), u32_from_jint(point_count), kdop_preset(preset))) });
jni!(long colliderBuilderCreateFdh(long points_xyz, int point_count, long directions_xyz, int direction_count) { to_jlong(dop::collider_builder_create_fdh(p::<f64>(points_xyz), u32_from_jint(point_count), p::<f64>(directions_xyz), u32_from_jint(direction_count))) });
jni!(long colliderBuilderCreateNeuralBounds(double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, int sample_resolution, int hidden_width, int hidden_layers, int activation, double output_scale, double padding, long weights, int weight_count) { to_jlong(neu::collider_builder_create_neural_bounds(NeuralBoundsDesc { center: v3(cx,cy,cz), half_extents: v3(hx,hy,hz), rotation: qt(qi,qj,qk,qw), sample_resolution: u32_from_jint(sample_resolution), hidden_width: u32_from_jint(hidden_width), hidden_layers: u32_from_jint(hidden_layers), activation: neural_activation(activation), output_scale, padding,}, p::<f64>(weights), u32_from_jint(weight_count))) });
jni_e_c!(long colliderBuilderCreateVoxels(env _env, class _class, long voxels, int size_x, int size_y, int size_z, double voxel_size, double origin_x, double origin_y, double origin_z, int mode, int dynamic_body, int small_voxel_limit, int mesh_voxel_limit) { to_jlong(vx::collider_builder_create_voxels(p::<u8>(voxels), u32_from_jint(size_x), u32_from_jint(size_y), u32_from_jint(size_z), voxel_size, v3(origin_x, origin_y, origin_z), VoxelColliderOptions { mode: voxel_mode(mode), dynamic_body: jb(dynamic_body), small_voxel_limit: u32_from_jint(small_voxel_limit), mesh_voxel_limit: u32_from_jint(mesh_voxel_limit) })) });
jni_e_c!(long colliderBuilderCreateVoxelsAuto(env _env, class _class, long voxels, int size_x, int size_y, int size_z, double voxel_size, double origin_x, double origin_y, double origin_z, int dynamic_body) { to_jlong(vx::collider_builder_create_voxels_auto(p::<u8>(voxels), u32_from_jint(size_x), u32_from_jint(size_y), u32_from_jint(size_z), voxel_size, v3(origin_x, origin_y, origin_z), jb(dynamic_body))) });
jni_e_c!(long colliderBuilderCreateVoxelBytes(env _env, class _class, byte_array voxels, int size_x, int size_y, int size_z, double voxel_size, double origin_x, double origin_y, double origin_z, int mode, int dynamic_body, int small_voxel_limit, int mesh_voxel_limit) {
    let Some(values) = jbytearray_to_array(&_env, voxels) else {
        return 0;
    };
    to_jlong(vx::collider_builder_create_voxels(values.as_ptr(), u32_from_jint(size_x), u32_from_jint(size_y), u32_from_jint(size_z), voxel_size, v3(origin_x, origin_y, origin_z), VoxelColliderOptions { mode: voxel_mode(mode), dynamic_body: jb(dynamic_body), small_voxel_limit: u32_from_jint(small_voxel_limit), mesh_voxel_limit: u32_from_jint(mesh_voxel_limit) }))
});
jni_e_c!(long colliderBuilderCreateVoxelBytesAuto(env _env, class _class, byte_array voxels, int size_x, int size_y, int size_z, double voxel_size, double origin_x, double origin_y, double origin_z, int dynamic_body) {
    let Some(values) = jbytearray_to_array(&_env, voxels) else {
        return 0;
    };
    to_jlong(vx::collider_builder_create_voxels_auto(values.as_ptr(), u32_from_jint(size_x), u32_from_jint(size_y), u32_from_jint(size_z), voxel_size, v3(origin_x, origin_y, origin_z), jb(dynamic_body)))
});
jni!(long colliderBuilderCreateVoxelAabb(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double voxel_size, int mode, int dynamic_body, int small_voxel_limit, int mesh_voxel_limit) {
    to_jlong(vx::collider_builder_create_voxel_aabb(
        aa(min_x, min_y, min_z, max_x, max_y, max_z),
        voxel_size,
        VoxelColliderOptions { mode: voxel_mode(mode), dynamic_body: jb(dynamic_body), small_voxel_limit: u32_from_jint(small_voxel_limit), mesh_voxel_limit: u32_from_jint(mesh_voxel_limit) }
    ))
});
jni!(long colliderBuilderCreateVoxelAabbAuto(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double voxel_size, int dynamic_body) {
    to_jlong(vx::collider_builder_create_voxel_aabb_auto(
        aa(min_x, min_y, min_z, max_x, max_y, max_z),
        voxel_size,
        jb(dynamic_body)
    ))
});
jni!(long colliderBuilderCreateVoxelObb(double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, double voxel_size, int mode, int dynamic_body, int small_voxel_limit, int mesh_voxel_limit) {
    to_jlong(vx::collider_builder_create_voxel_obb(
        Obb { center: v3(cx, cy, cz), half_extents: v3(hx, hy, hz), rotation: qt(qi, qj, qk, qw) },
        voxel_size,
        VoxelColliderOptions { mode: voxel_mode(mode), dynamic_body: jb(dynamic_body), small_voxel_limit: u32_from_jint(small_voxel_limit), mesh_voxel_limit: u32_from_jint(mesh_voxel_limit) }
    ))
});
jni!(long colliderBuilderCreateVoxelObbAuto(double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, double voxel_size, int dynamic_body) {
    to_jlong(vx::collider_builder_create_voxel_obb_auto(
        Obb { center: v3(cx, cy, cz), half_extents: v3(hx, hy, hz), rotation: qt(qi, qj, qk, qw) },
        voxel_size,
        jb(dynamic_body)
    ))
});
jni!(void voxelBuildStats(long voxels, int size_x, int size_y, int size_z, double voxel_size, double origin_x, double origin_y, double origin_z, int mode, int dynamic_body, int small_voxel_limit, int mesh_voxel_limit, long out_stats) {
    let stats = vx::voxel_build_stats(
        p::<u8>(voxels),
        u32_from_jint(size_x),
        u32_from_jint(size_y),
        u32_from_jint(size_z),
        voxel_size,
        v3(origin_x, origin_y, origin_z),
        VoxelColliderOptions { mode: voxel_mode(mode), dynamic_body: jb(dynamic_body), small_voxel_limit: u32_from_jint(small_voxel_limit), mesh_voxel_limit: u32_from_jint(mesh_voxel_limit) },
    );
    if let Some(out) = unsafe { pm::<crate::rapier::ffi::VoxelBuildStats>(out_stats).as_mut() } { *out = stats; }
});
jni!(void voxelAabbBuildStats(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double voxel_size, int mode, int dynamic_body, int small_voxel_limit, int mesh_voxel_limit, long out_stats) {
    let stats = vx::voxel_aabb_build_stats(
        aa(min_x, min_y, min_z, max_x, max_y, max_z),
        voxel_size,
        VoxelColliderOptions { mode: voxel_mode(mode), dynamic_body: jb(dynamic_body), small_voxel_limit: u32_from_jint(small_voxel_limit), mesh_voxel_limit: u32_from_jint(mesh_voxel_limit) },
    );
    if let Some(out) = unsafe { pm::<crate::rapier::ffi::VoxelBuildStats>(out_stats).as_mut() } { *out = stats; }
});
jni!(void voxelObbBuildStats(double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, double voxel_size, int mode, int dynamic_body, int small_voxel_limit, int mesh_voxel_limit, long out_stats) {
    let stats = vx::voxel_obb_build_stats(
        Obb { center: v3(cx, cy, cz), half_extents: v3(hx, hy, hz), rotation: qt(qi, qj, qk, qw) },
        voxel_size,
        VoxelColliderOptions { mode: voxel_mode(mode), dynamic_body: jb(dynamic_body), small_voxel_limit: u32_from_jint(small_voxel_limit), mesh_voxel_limit: u32_from_jint(mesh_voxel_limit) },
    );
    if let Some(out) = unsafe { pm::<crate::rapier::ffi::VoxelBuildStats>(out_stats).as_mut() } { *out = stats; }
});

jni!(void colliderBuilderSetTranslation(long builder, double x, double y, double z) { col::collider_builder_set_translation(m::<CBH>(builder), v3(x, y, z)); });
jni!(void colliderBuilderSetRotation(long builder, double x, double y, double z) { col::collider_builder_set_rotation(m::<CBH>(builder), v3(x, y, z)); });
jni!(void colliderBuilderSetPose(long builder, double x, double y, double z, double qi, double qj, double qk, double qw) { col::collider_builder_set_pose(m::<CBH>(builder), v3(x, y, z), qt(qi, qj, qk, qw)); });
jni!(void colliderBuilderSetSensor(long builder, int sensor) { col::collider_builder_set_sensor(m::<CBH>(builder), jb(sensor)); });
jni!(void colliderBuilderSetFriction(long builder, double friction) { col::collider_builder_set_friction(m::<CBH>(builder), friction); });
jni!(void colliderBuilderSetRestitution(long builder, double restitution) { col::collider_builder_set_restitution(m::<CBH>(builder), restitution); });
jni!(void colliderBuilderSetDensity(long builder, double density) { col::collider_builder_set_density(m::<CBH>(builder), density); });
jni!(void colliderBuilderSetCollisionGroups(long builder, int memberships, int filter) { col::collider_builder_set_collision_groups(m::<CBH>(builder), grp(memberships, filter)); });
jni!(void colliderBuilderSetSolverGroups(long builder, int memberships, int filter) { col::collider_builder_set_solver_groups(m::<CBH>(builder), grp(memberships, filter)); });
jni!(void colliderBuilderSetActiveEvents(long builder, int bits) { col::collider_builder_set_active_events(m::<CBH>(builder), bits as u32); });
jni!(void colliderBuilderSetActiveHooks(long builder, int bits) { col::collider_builder_set_active_hooks(m::<CBH>(builder), bits as u32); });
jni!(void colliderBuilderSetContactForceEventThreshold(long builder, double threshold) { col::collider_builder_set_contact_force_event_threshold(m::<CBH>(builder), threshold); });

jni!(long colliderBuilderBuild(long builder) { to_jlong(col::collider_builder_build(m::<CBH>(builder))) });

jni!(void colliderBuilderDestroy(long builder) { col::collider_builder_destroy(m::<CBH>(builder)); });

jni_e_c!(double_array colliderGetTranslation(env _env, class _class, long world, long handle) { vec3_to_j_double_array(_env, col::collider_get_translation(cp::<WH>(world), handle as CRaw)) });
jni_e_c!(double_array colliderGetRotation(env _env, class _class, long world, long handle) { quat_to_j_double_array(_env, col::collider_get_rotation(cp::<WH>(world), handle as CRaw)) });
jni!(void colliderGetTranslationOut(long world, long handle, long out_translation) { col::collider_get_translation_out(cp::<WH>(world), handle as CRaw, pm::<Vec3>(out_translation)); });
jni!(void colliderGetRotationOut(long world, long handle, long out_rotation) { col::collider_get_rotation_out(cp::<WH>(world), handle as CRaw, pm::<Quat>(out_rotation)); });
jni!(boolean colliderSetPose(long world, long handle, double x, double y, double z, double qi, double qj, double qk, double qw) { col::collider_set_pose(m::<WH>(world), handle as CRaw, v3(x, y, z), qt(qi, qj, qk, qw)).0 as jbyte });
jni!(boolean colliderSetSensor(long world, long handle, int sensor) { col::collider_set_sensor(m::<WH>(world), handle as CRaw, jb(sensor)).0 as jbyte });
jni!(boolean colliderSetFriction(long world, long handle, double friction) { col::collider_set_friction(m::<WH>(world), handle as CRaw, friction).0 as jbyte });
jni!(boolean colliderSetRestitution(long world, long handle, double restitution) { col::collider_set_restitution(m::<WH>(world), handle as CRaw, restitution).0 as jbyte });
jni!(boolean colliderSetCollisionGroups(long world, long handle, int memberships, int filter) { col::collider_set_collision_groups(m::<WH>(world), handle as CRaw, grp(memberships, filter)).0 as jbyte });
jni!(boolean colliderSetSolverGroups(long world, long handle, int memberships, int filter) { col::collider_set_solver_groups(m::<WH>(world), handle as CRaw, grp(memberships, filter)).0 as jbyte });
jni!(boolean colliderSetActiveEvents(long world, long handle, int bits) { col::collider_set_active_events(m::<WH>(world), handle as CRaw, bits as u32).0 as jbyte });
jni!(boolean colliderSetActiveHooks(long world, long handle, int bits) { col::collider_set_active_hooks(m::<WH>(world), handle as CRaw, bits as u32).0 as jbyte });
jni!(boolean colliderSetContactForceEventThreshold(long world, long handle, double threshold) { col::collider_set_contact_force_event_threshold(m::<WH>(world), handle as CRaw, threshold).0 as jbyte });
jni!(double colliderGetDensity(long world, long handle) { col::collider_get_density(cp::<WH>(world), handle as CRaw) });

jni!(long rigidBodyBuilderCreate(int status) { to_jlong(rb::rigid_body_builder_create(body_status(status))) });

jni!(void rigidBodyBuilderSetTranslation(long builder, double x, double y, double z) { rb::rigid_body_builder_set_translation(m::<RBH>(builder), v3(x, y, z)); });
jni!(void rigidBodyBuilderSetRotation(long builder, double x, double y, double z) { rb::rigid_body_builder_set_rotation(m::<RBH>(builder), v3(x, y, z)); });
jni!(void rigidBodyBuilderSetPose(long builder, double x, double y, double z, double qi, double qj, double qk, double qw) { rb::rigid_body_builder_set_pose(m::<RBH>(builder), v3(x, y, z), qt(qi, qj, qk, qw)); });
jni!(void rigidBodyBuilderSetAdditionalMassProperties(long builder, double cx, double cy, double cz, double mass, double lx, double ly, double lz) { rb::rigid_body_builder_set_additional_mass_properties(m::<RBH>(builder), v3(cx, cy, cz), mass, v3(lx, ly, lz)); });
jni!(void rigidBodyBuilderSetLinvel(long builder, double x, double y, double z) { rb::rigid_body_builder_set_linvel(m::<RBH>(builder), v3(x, y, z)); });
jni!(void rigidBodyBuilderSetAngvel(long builder, double x, double y, double z) { rb::rigid_body_builder_set_angvel(m::<RBH>(builder), v3(x, y, z)); });
jni!(void rigidBodyBuilderSetGravityScale(long builder, double value) { rb::rigid_body_builder_set_gravity_scale(m::<RBH>(builder), value); });
jni!(void rigidBodyBuilderSetLinearDamping(long builder, double value) { rb::rigid_body_builder_set_linear_damping(m::<RBH>(builder), value); });
jni!(void rigidBodyBuilderSetAngularDamping(long builder, double value) { rb::rigid_body_builder_set_angular_damping(m::<RBH>(builder), value); });
jni!(void rigidBodyBuilderSetCanSleep(long builder, int value) { rb::rigid_body_builder_set_can_sleep(m::<RBH>(builder), jb(value)); });
jni!(void rigidBodyBuilderSetEnabledRotations(long builder, int x, int y, int z) { rb::rigid_body_builder_set_enabled_rotations(m::<RBH>(builder), jb(x), jb(y), jb(z)); });
jni!(void rigidBodyBuilderSetUserData(long builder, long low, long high) { rb::rigid_body_builder_set_user_data(m::<RBH>(builder), low as u64, high as u64); });
jni!(void rigidBodyBuilderSetAdditionalMass(long builder, double mass) { rb::rigid_body_builder_set_additional_mass(m::<RBH>(builder), mass); });

jni!(long rigidBodyBuilderBuild(long builder) { to_jlong(rb::rigid_body_builder_build(m::<RBH>(builder))) });

jni!(void rigidBodyBuilderDestroy(long builder) { rb::rigid_body_builder_destroy(m::<RBH>(builder)); });

jni!(int rigidBodyGetStatus(long world, long handle) { rb::rigid_body_get_status(cp::<WH>(world), handle as RRaw) as jint });
jni!(boolean rigidBodySetStatus(long world, long handle, int status, int wake_up) { rb::rigid_body_set_status(m::<WH>(world), handle as RRaw, body_status(status), jb(wake_up)).0 as jbyte });

jni_e_c!(double_array rigidBodyGetTranslation(env _env, class _class, long world, long body) { vec3_to_j_double_array(_env, rb::rigid_body_get_translation(cp::<WH>(world), body as RRaw)) });
jni_e_c!(double_array rigidBodyGetRotation(env _env, class _class, long world, long body) { quat_to_j_double_array(_env, rb::rigid_body_get_rotation(cp::<WH>(world), body as RRaw)) });
jni!(void rigidBodyGetTranslationOut(long world, long body, long out_translation) { rb::rigid_body_get_translation_out(cp::<WH>(world), body as RRaw, pm::<Vec3>(out_translation)); });
jni!(void rigidBodyGetRotationOut(long world, long body, long out_rotation) { rb::rigid_body_get_rotation_out(cp::<WH>(world), body as RRaw, pm::<Quat>(out_rotation)); });
jni!(boolean rigidBodySetPose(long world, long body, double x, double y, double z, double qi, double qj, double qk, double qw, int wake_up) { rb::rigid_body_set_pose(m::<WH>(world), body as RRaw, v3(x, y, z), qt(qi, qj, qk, qw), jb(wake_up)).0 as jbyte });
jni!(boolean rigidBodySetTranslation(long world, long body, double x, double y, double z, int wake_up) { rb::rigid_body_set_translation(m::<WH>(world), body as RRaw, v3(x, y, z), jb(wake_up)).0 as jbyte });
jni!(boolean rigidBodySetRotation(long world, long body, double qi, double qj, double qk, double qw, int wake_up) { rb::rigid_body_set_rotation(m::<WH>(world), body as RRaw, qt(qi, qj, qk, qw), jb(wake_up)).0 as jbyte });
jni_e_c!(double_array rigidBodyGetLinvel(env _env, class _class, long world, long body) { vec3_to_j_double_array(_env, rb::rigid_body_get_linvel(cp::<WH>(world), body as RRaw)) });
jni!(void rigidBodyGetLinvelOut(long world, long body, long out_linvel) { rb::rigid_body_get_linvel_out(cp::<WH>(world), body as RRaw, pm::<Vec3>(out_linvel)); });
jni!(boolean rigidBodySetLinvel(long world, long body, double x, double y, double z, int wake_up) { rb::rigid_body_set_linvel(m::<WH>(world), body as RRaw, v3(x, y, z), jb(wake_up)).0 as jbyte });
jni_e_c!(double_array rigidBodyGetAngvel(env _env, class _class, long world, long body) { vec3_to_j_double_array(_env, rb::rigid_body_get_angvel(cp::<WH>(world), body as RRaw)) });
jni!(void rigidBodyGetAngvelOut(long world, long body, long out_angvel) { rb::rigid_body_get_angvel_out(cp::<WH>(world), body as RRaw, pm::<Vec3>(out_angvel)); });
jni!(boolean rigidBodySetAngvel(long world, long body, double x, double y, double z, int wake_up) { rb::rigid_body_set_angvel(m::<WH>(world), body as RRaw, v3(x, y, z), jb(wake_up)).0 as jbyte });
jni!(boolean rigidBodyAddForce(long world, long body, double x, double y, double z, int wake_up) { rb::rigid_body_add_force(m::<WH>(world), body as RRaw, v3(x, y, z), jb(wake_up)).0 as jbyte });
jni!(boolean rigidBodyAddTorque(long world, long body, double x, double y, double z, int wake_up) { rb::rigid_body_add_torque(m::<WH>(world), body as RRaw, v3(x, y, z), jb(wake_up)).0 as jbyte });
jni!(boolean rigidBodyApplyImpulse(long world, long body, double x, double y, double z, int wake_up) { rb::rigid_body_apply_impulse(m::<WH>(world), body as RRaw, v3(x, y, z), jb(wake_up)).0 as jbyte });
jni!(boolean rigidBodyApplyTorqueImpulse(long world, long body, double x, double y, double z, int wake_up) { rb::rigid_body_apply_torque_impulse(m::<WH>(world), body as RRaw, v3(x, y, z), jb(wake_up)).0 as jbyte });
jni!(boolean rigidBodyEnableCcd(long world, long body, int enabled) { rb::rigid_body_enable_ccd(m::<WH>(world), body as RRaw, jb(enabled)).0 as jbyte });
jni!(boolean rigidBodySleep(long world, long body) { rb::rigid_body_sleep(m::<WH>(world), body as RRaw).0 as jbyte });
jni!(boolean rigidBodyWakeUp(long world, long body, int strong) { rb::rigid_body_wake_up(m::<WH>(world), body as RRaw, jb(strong)).0 as jbyte });
jni!(boolean rigidBodyIsSleeping(long world, long body) { rb::rigid_body_is_sleeping(cp::<WH>(world), body as RRaw).0 as jbyte });

macro_rules! query_filter_args {
    ($flags:ident,$memberships:ident,$filter:ident,$use_groups:ident,$exclude_collider:ident,$use_exclude_collider:ident,$exclude_rigid_body:ident,$use_exclude_rigid_body:ident) => {
        qfilter(
            $flags,
            $memberships,
            $filter,
            $use_groups,
            $exclude_collider,
            $use_exclude_collider,
            $exclude_rigid_body,
            $use_exclude_rigid_body,
        )
    };
}

jni!(long queryCastRay(long world, double ox, double oy, double oz, double dx, double dy, double dz, double max_toi, int solid, int flags, int memberships, int filter, int use_groups, long exclude_collider, int use_exclude_collider, long exclude_rigid_body, int use_exclude_rigid_body, long out_hit) {
    let hit = qu::query_cast_ray(cp::<WH>(world), v3(ox, oy, oz), v3(dx, dy, dz), max_toi, jb(solid), query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body));
    if let Some(out) = unsafe { pm::<RayHit>(out_hit).as_mut() } { *out = hit; }
    hit.collider as jlong
});

jni!(int queryCastRays(long world, long rays, int ray_count, double max_toi, int solid, int flags, int memberships, int filter, int use_groups, long exclude_collider, int use_exclude_collider, long exclude_rigid_body, int use_exclude_rigid_body, long out_hits, int capacity) {
    qu::query_cast_rays(cp::<WH>(world), p::<f64>(rays), u32_from_jint(ray_count), max_toi, jb(solid), query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body), pm::<RayHit>(out_hits), u32_from_jint(capacity)) as jint
});

jni!(long queryProjectPoint(long world, double x, double y, double z, double max_dist, int solid, int flags, int memberships, int filter, int use_groups, long exclude_collider, int use_exclude_collider, long exclude_rigid_body, int use_exclude_rigid_body, long out_projection) {
    let mut collider: CRaw = 0;
    let projection = qu::query_project_point(cp::<WH>(world), v3(x, y, z), max_dist, jb(solid), query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body), &mut collider as *mut CRaw);
    if let Some(out) = unsafe { pm::<PointProjection>(out_projection).as_mut() } { *out = projection; }
    collider as jlong
});

jni!(int queryIntersectPointCount(long world, double x, double y, double z, int flags, int memberships, int filter, int use_groups, long exclude_collider, int use_exclude_collider, long exclude_rigid_body, int use_exclude_rigid_body) {
    qu::query_intersect_point_count(cp::<WH>(world), v3(x, y, z), query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body)) as jint
});

jni!(int queryIntersectAabbCount(long world, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, int flags, int memberships, int filter, int use_groups, long exclude_collider, int use_exclude_collider, long exclude_rigid_body, int use_exclude_rigid_body) {
    qu::query_intersect_aabb_count(cp::<WH>(world), aa(min_x,min_y,min_z,max_x,max_y,max_z), query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body)) as jint
});

jni!(int queryIntersectObb(long world, double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, int flags, int memberships, int filter, int use_groups, long exclude_collider, int use_exclude_collider, long exclude_rigid_body, int use_exclude_rigid_body, long out_handles, int capacity) {
    qu::query_intersect_obb(cp::<WH>(world), Obb { center: v3(cx,cy,cz), half_extents: v3(hx,hy,hz), rotation: qt(qi,qj,qk,qw) }, query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body), pm::<CRaw>(out_handles), u32_from_jint(capacity)) as jint
});

jni!(int queryIntersectSphere(long world, double cx, double cy, double cz, double radius, int flags, int memberships, int filter, int use_groups, long exclude_collider, int use_exclude_collider, long exclude_rigid_body, int use_exclude_rigid_body, long out_handles, int capacity) {
    qu::query_intersect_sphere(cp::<WH>(world), Sphere { center: v3(cx,cy,cz), radius }, query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body), pm::<CRaw>(out_handles), u32_from_jint(capacity)) as jint
});

jni!(int queryIntersectVoxelAabb(long world, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, int flags, int memberships, int filter, int use_groups, long exclude_collider, int use_exclude_collider, long exclude_rigid_body, int use_exclude_rigid_body, long out_handles, int capacity) {
    vx::query_intersect_voxel_aabb(cp::<WH>(world), aa(min_x,min_y,min_z,max_x,max_y,max_z), query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body), pm::<CRaw>(out_handles), u32_from_jint(capacity)) as jint
});

jni!(int queryIntersectVoxelAabbCount(long world, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, int flags, int memberships, int filter, int use_groups, long exclude_collider, int use_exclude_collider, long exclude_rigid_body, int use_exclude_rigid_body) {
    vx::query_intersect_voxel_aabb_count(cp::<WH>(world), aa(min_x,min_y,min_z,max_x,max_y,max_z), query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body)) as jint
});

jni!(int queryIntersectVoxelObb(long world, double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, int flags, int memberships, int filter, int use_groups, long exclude_collider, int use_exclude_collider, long exclude_rigid_body, int use_exclude_rigid_body, long out_handles, int capacity) {
    vx::query_intersect_voxel_obb(cp::<WH>(world), Obb { center: v3(cx,cy,cz), half_extents: v3(hx,hy,hz), rotation: qt(qi,qj,qk,qw) }, query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body), pm::<CRaw>(out_handles), u32_from_jint(capacity)) as jint
});

jni!(int queryIntersectVoxelObbCount(long world, double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, int flags, int memberships, int filter, int use_groups, long exclude_collider, int use_exclude_collider, long exclude_rigid_body, int use_exclude_rigid_body) {
    vx::query_intersect_voxel_obb_count(cp::<WH>(world), Obb { center: v3(cx,cy,cz), half_extents: v3(hx,hy,hz), rotation: qt(qi,qj,qk,qw) }, query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body)) as jint
});

jni!(long queryCastShape(long world, int shape_type, double a, double b, double c, double d, double tx, double ty, double tz, double qi, double qj, double qk, double qw, double vx, double vy, double vz, double max_toi, double target_distance, int stop_at_penetration, int compute_impact_geometry_on_penetration, int flags, int memberships, int filter, int use_groups, long exclude_collider, int use_exclude_collider, long exclude_rigid_body, int use_exclude_rigid_body, long out_hit) {
    let hit = qu::query_cast_shape(
        cp::<WH>(world),
        sd(shape_type, a, b, c, d),
        v3(tx,ty,tz),
        qt(qi,qj,qk,qw),
        v3(vx,vy,vz),
        ShapeCastOptionsDesc { max_time_of_impact: max_toi, target_distance, stop_at_penetration: jb(stop_at_penetration), compute_impact_geometry_on_penetration: jb(compute_impact_geometry_on_penetration) },
        query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body),
    );
    if let Some(out) = unsafe { pm::<ShapeCastHit>(out_hit).as_mut() } { *out = hit; }
    hit.collider as jlong
});

jni!(int neuralBoundsRequiredWeightCount(int hidden_width, int hidden_layers) {
    neu::neural_bounds_required_weight_count(u32_from_jint(hidden_width), u32_from_jint(hidden_layers)) as jint
});

jni!(long worldInsertDynamicCuboids(long world, double x, double y, double z, double qi, double qj, double qk, double qw, double lvx, double lvy, double lvz, long cuboids, int cuboid_count, double density, double friction, double restitution, int collision_memberships, int collision_filter, int solver_memberships, int solver_filter) {
    com::world_insert_dynamic_cuboids(m::<WH>(world), v3(x,y,z), qt(qi,qj,qk,qw), v3(lvx,lvy,lvz), p::<f64>(cuboids), u32_from_jint(cuboid_count), density, friction, restitution, grp(collision_memberships, collision_filter), grp(solver_memberships, solver_filter)) as jlong
});
jni!(long worldInsertStaticTrimesh(long world, long vertices_xyz, int vertex_xyz_len, long indices, int index_len, double friction, double restitution) {
    com::world_insert_static_trimesh(m::<WH>(world), p::<f64>(vertices_xyz), u32_from_jint(vertex_xyz_len), p::<u32>(indices), u32_from_jint(index_len), friction, restitution) as jlong
});
jni!(long worldInsertStaticVoxelAabb(long world, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, double voxel_size, int mode, int small_voxel_limit, int mesh_voxel_limit, double friction, double restitution) {
    vx::world_insert_static_voxel_aabb(m::<WH>(world), aa(min_x,min_y,min_z,max_x,max_y,max_z), voxel_size, VoxelColliderOptions { mode: voxel_mode(mode), dynamic_body: Bool::FALSE, small_voxel_limit: u32_from_jint(small_voxel_limit), mesh_voxel_limit: u32_from_jint(mesh_voxel_limit) }, friction, restitution) as jlong
});
jni!(long worldInsertDynamicVoxelObb(long world, double cx, double cy, double cz, double hx, double hy, double hz, double qi, double qj, double qk, double qw, double voxel_size, int mode, int small_voxel_limit, int mesh_voxel_limit, double density, double friction, double restitution) {
    vx::world_insert_dynamic_voxel_obb(m::<WH>(world), Obb { center: v3(cx,cy,cz), half_extents: v3(hx,hy,hz), rotation: qt(qi,qj,qk,qw) }, voxel_size, VoxelColliderOptions { mode: voxel_mode(mode), dynamic_body: Bool::TRUE, small_voxel_limit: u32_from_jint(small_voxel_limit), mesh_voxel_limit: u32_from_jint(mesh_voxel_limit) }, density, friction, restitution) as jlong
});

jni!(long jointBuilderCreate(int joint_type, double ax, double ay, double az, double b, double c) {
    to_jlong(jo::joint_builder_create(self::joint_type(joint_type), v3(ax, ay, az), b, c))
});
jni!(void jointBuilderDestroy(long builder) { jo::joint_builder_destroy(m::<JBH>(builder)); });
jni!(void jointBuilderSetContactsEnabled(long builder, int enabled) { jo::joint_builder_set_contacts_enabled(m::<JBH>(builder), jb(enabled)); });
jni!(void jointBuilderSetLocalAnchor1(long builder, double x, double y, double z) { jo::joint_builder_set_local_anchor1(m::<JBH>(builder), v3(x,y,z)); });
jni!(void jointBuilderSetLocalAnchor2(long builder, double x, double y, double z) { jo::joint_builder_set_local_anchor2(m::<JBH>(builder), v3(x,y,z)); });
jni!(void jointBuilderSetLimits(long builder, int axis, double min, double max) { jo::joint_builder_set_limits(m::<JBH>(builder), joint_axis(axis), min, max); });
jni!(void jointBuilderSetMotorVelocity(long builder, int axis, double target_vel, double factor) { jo::joint_builder_set_motor_velocity(m::<JBH>(builder), joint_axis(axis), target_vel, factor); });
jni!(void jointBuilderSetMotorPosition(long builder, int axis, double target_pos, double stiffness, double damping) { jo::joint_builder_set_motor_position(m::<JBH>(builder), joint_axis(axis), target_pos, stiffness, damping); });
jni!(long worldInsertImpulseJoint(long world, long body1, long body2, long builder, int wake_up) { jo::world_insert_impulse_joint(m::<WH>(world), body1 as RRaw, body2 as RRaw, m::<JBH>(builder), jb(wake_up)) as jlong });
jni!(boolean worldRemoveImpulseJoint(long world, long handle, int wake_up) { jo::world_remove_impulse_joint(m::<WH>(world), handle as JRaw, jb(wake_up)).0 as jbyte });

jni!(long characterControllerCreate() { to_jlong(cc::character_controller_create()) });
jni!(void characterControllerDestroy(long controller) { cc::character_controller_destroy(m::<CCH>(controller)); });
jni!(void characterControllerSetUp(long controller, double x, double y, double z) { cc::character_controller_set_up(m::<CCH>(controller), v3(x,y,z)); });
jni!(void characterControllerSetOffsetAbsolute(long controller, double offset) { cc::character_controller_set_offset_absolute(m::<CCH>(controller), offset); });
jni!(void characterControllerSetOffsetRelative(long controller, double offset) { cc::character_controller_set_offset_relative(m::<CCH>(controller), offset); });
jni!(void characterControllerSetSlide(long controller, int slide) { cc::character_controller_set_slide(m::<CCH>(controller), jb(slide)); });
jni!(void characterControllerSetAutostep(long controller, int enabled, double max_height, double min_width, int include_dynamic_bodies) { cc::character_controller_set_autostep(m::<CCH>(controller), jb(enabled), max_height, min_width, jb(include_dynamic_bodies)); });
jni!(void characterControllerSetSnapToGround(long controller, int enabled, double distance) { cc::character_controller_set_snap_to_ground(m::<CCH>(controller), jb(enabled), distance); });
jni!(void characterControllerSetSlopeAngles(long controller, double max_climb_angle, double min_slide_angle) { cc::character_controller_set_slope_angles(m::<CCH>(controller), max_climb_angle, min_slide_angle); });
jni!(boolean characterControllerMoveShape(long world, long controller, double dt, int shape_type, double a, double b, double c, double d, double tx, double ty, double tz, double qi, double qj, double qk, double qw, double dx, double dy, double dz, long out_movement) {
    let movement = cc::character_controller_move_shape(cp::<WH>(world), m::<CCH>(controller), dt, sd(shape_type,a,b,c,d), v3(tx,ty,tz), qt(qi,qj,qk,qw), v3(dx,dy,dz));
    if let Some(out) = unsafe { pm::<EffectiveCharacterMovement>(out_movement).as_mut() } { *out = movement; }
    movement.grounded.0 as jbyte
});
jni!(int characterControllerCollisionCount(long controller) { cc::character_controller_collision_count(cp::<CCH>(controller)) as jint });
jni!(long characterControllerGetCollision(long controller, int index, long out_collision) {
    let collision = cc::character_controller_get_collision(cp::<CCH>(controller), u32_from_jint(index));
    if let Some(out) = unsafe { pm::<CharacterCollision>(out_collision).as_mut() } { *out = collision; }
    collision.collider as jlong
});
jni!(boolean characterControllerSolveImpulses(long world, long controller, double dt, int shape_type, double a, double b, double c, double d, double character_mass) {
    cc::character_controller_solve_impulses(m::<WH>(world), m::<CCH>(controller), dt, sd(shape_type,a,b,c,d), character_mass).0 as jbyte
});

jni!(void worldClearEvents(long world) { ev::world_clear_events(m::<WH>(world)); });
jni!(int worldCollisionEventCount(long world) { ev::world_collision_event_count(cp::<WH>(world)) as jint });
jni!(long worldGetCollisionEvent(long world, int index, long out_event) {
    let event = ev::world_get_collision_event(cp::<WH>(world), u32_from_jint(index));
    if let Some(out) = unsafe { pm::<CER>(out_event).as_mut() } { *out = event; }
    event.collider1 as jlong
});
jni!(int worldGetCollisionEvents(long world, long out_events, int capacity) {
    ev::world_get_collision_events(cp::<WH>(world), pm::<CER>(out_events), u32_from_jint(capacity)) as jint
});
jni!(int worldContactForceEventCount(long world) { ev::world_contact_force_event_count(cp::<WH>(world)) as jint });
jni!(long worldGetContactForceEvent(long world, int index, long out_event) {
    let event = ev::world_get_contact_force_event(cp::<WH>(world), u32_from_jint(index));
    if let Some(out) = unsafe { pm::<ContactForceEventRecord>(out_event).as_mut() } { *out = event; }
    event.collider1 as jlong
});
jni!(int worldGetContactForceEvents(long world, long out_events, int capacity) {
    ev::world_get_contact_force_events(cp::<WH>(world), pm::<ContactForceEventRecord>(out_events), u32_from_jint(capacity)) as jint
});
jni!(void worldClearContactPairFilterCallback(long world) { ev::world_clear_contact_pair_filter_callback(m::<WH>(world)); });
jni!(void worldClearIntersectionPairFilterCallback(long world) { ev::world_clear_intersection_pair_filter_callback(m::<WH>(world)); });

#[cfg(feature = "anvilkit-bridge")]
jni!(long anvilKitAppCreate() { to_jlong(ak::anvilkit_app_create()) });
#[cfg(feature = "anvilkit-bridge")]
jni!(void anvilKitAppDestroy(long app) { ak::anvilkit_app_destroy(m::<AKH>(app)); });
#[cfg(feature = "anvilkit-bridge")]
jni!(void anvilKitAppUpdate(long app) { ak::anvilkit_app_update(m::<AKH>(app)); });
#[cfg(feature = "anvilkit-bridge")]
jni!(long anvilKitAppSpawnBody(long app, double tx, double ty, double tz, double qi, double qj, double qk, double qw, int status) {
    ak::anvilkit_app_spawn_body(m::<AKH>(app), v3(tx, ty, tz), qt(qi, qj, qk, qw), u32_from_jint(status)) as jlong
});
#[cfg(feature = "anvilkit-bridge")]
jni!(long anvilKitAppSpawnBodyWithCollider(long app, double tx, double ty, double tz, double qi, double qj, double qk, double qw, int status, int shape_type, double a, double b, double c, double d) {
    ak::anvilkit_app_spawn_body_with_collider(m::<AKH>(app), v3(tx, ty, tz), qt(qi, qj, qk, qw), u32_from_jint(status), sd(shape_type, a, b, c, d)) as jlong
});
#[cfg(feature = "anvilkit-bridge")]
jni!(boolean anvilKitAppSetTransform(long app, long entity_bits, double tx, double ty, double tz, double qi, double qj, double qk, double qw) {
    ak::anvilkit_app_set_transform(m::<AKH>(app), entity_bits as u64, v3(tx, ty, tz), qt(qi, qj, qk, qw)).0 as jbyte
});
#[cfg(feature = "anvilkit-bridge")]
jni!(int anvilKitAppSyncToWorld(long app, long world) {
    ak::anvilkit_app_sync_to_world(m::<AKH>(app), m::<WH>(world)) as jint
});
#[cfg(feature = "anvilkit-bridge")]
jni!(long anvilKitAppEntityToBody(long app, long entity_bits) {
    ak::anvilkit_app_entity_to_body(cp::<AKH>(app), entity_bits as u64) as jlong
});
#[cfg(feature = "anvilkit-bridge")]
jni!(long anvilKitAppEntityToCollider(long app, long entity_bits) {
    ak::anvilkit_app_entity_to_collider(cp::<AKH>(app), entity_bits as u64) as jlong
});
#[cfg(feature = "anvilkit-bridge")]
jni!(boolean anvilKitAppApplyAeroSurfaces(long app, long world, long entity_bits, double wind_x, double wind_y, double wind_z, double air_density, long surfaces, int surface_count, int wake_up, long out_report) {
    ak::anvilkit_app_apply_aero_surfaces(m::<AKH>(app), m::<WH>(world), entity_bits as u64, v3(wind_x, wind_y, wind_z), air_density, p::<AeroSurface>(surfaces), u32_from_jint(surface_count), jb(wake_up), pm::<AeroForceReport>(out_report)).0 as jbyte
});
#[cfg(feature = "anvilkit-bridge")]
jni!(boolean anvilKitAppApplyAeroVoxelGrid(long app, long world, long entity_bits, double wind_x, double wind_y, double wind_z, double air_density, long voxels, int size_x, int size_y, int size_z, double voxel_size, double origin_x, double origin_y, double origin_z, double drag_coefficient, double lift_coefficient, int wake_up, long out_report) {
    ak::anvilkit_app_apply_aero_voxel_grid(m::<AKH>(app), m::<WH>(world), entity_bits as u64, v3(wind_x, wind_y, wind_z), air_density, p::<u8>(voxels), u32_from_jint(size_x), u32_from_jint(size_y), u32_from_jint(size_z), voxel_size, v3(origin_x, origin_y, origin_z), drag_coefficient, lift_coefficient, jb(wake_up), pm::<AeroForceReport>(out_report)).0 as jbyte
});
#[cfg(feature = "anvilkit-bridge")]
jni!(boolean anvilKitAppApplyFluidAabbForces(long app, long world, long entity_bits, double center_x, double center_y, double center_z, double half_x, double half_y, double half_z, double density, double linear_drag, double quadratic_drag, double angular_drag, double flow_x, double flow_y, double flow_z, double gravity_x, double gravity_y, double gravity_z, double body_half_x, double body_half_y, double body_half_z, double body_volume, int wake_up, long out_report) {
    ak::anvilkit_app_apply_fluid_aabb_forces(
        m::<AKH>(app),
        m::<WH>(world),
        entity_bits as u64,
        FluidVolume {
            center: v3(center_x, center_y, center_z),
            half_extents: v3(half_x, half_y, half_z),
            density,
            linear_drag,
            quadratic_drag,
            angular_drag,
            flow_velocity: v3(flow_x, flow_y, flow_z),
            gravity: v3(gravity_x, gravity_y, gravity_z),
        },
        v3(body_half_x, body_half_y, body_half_z),
        body_volume,
        jb(wake_up),
        pm::<FluidForceReport>(out_report)
    ).0 as jbyte
});
#[cfg(feature = "anvilkit-bridge")]
jni!(boolean anvilKitAppApplyTrajectoryForces(long app, long world, long entity_bits, double gravity_x, double gravity_y, double gravity_z, double flow_x, double flow_y, double flow_z, double mass, double reference_area, double density, double drag_coefficient, double lift_coefficient, double lift_x, double lift_y, double lift_z, int wake_up, long out_report) {
    ak::anvilkit_app_apply_trajectory_forces(
        m::<AKH>(app),
        m::<WH>(world),
        entity_bits as u64,
        TrajectoryEnvironment {
            gravity: v3(gravity_x, gravity_y, gravity_z),
            flow_velocity: v3(flow_x, flow_y, flow_z),
            mass,
            reference_area,
            density,
            drag_coefficient,
            lift_coefficient,
            lift_direction: v3(lift_x, lift_y, lift_z),
        },
        jb(wake_up),
        pm::<TrajectoryForceReport>(out_report)
    ).0 as jbyte
});

jni!(double spaceKeplerPeriod(double mu, double semi_major_axis) { sf::space_kepler_period(mu, semi_major_axis) });
jni!(double spaceKeplerSemiMajorAxis(double mu, double period) { sf::space_kepler_semi_major_axis(mu, period) });
jni!(boolean spaceHohmannTransfer(double mu, double radius1, double radius2, long out_transfer) {
    sf::space_hohmann_transfer(mu, radius1, radius2, pm::<HohmannTransfer>(out_transfer)).0 as jbyte
});
jni!(boolean spaceAtmosphericDragAcceleration(double vx, double vy, double vz, double avx, double avy, double avz, double density, double drag_coefficient, double area, double mass, long out_acceleration) {
    sf::space_atmospheric_drag_acceleration(v3(vx, vy, vz), v3(avx, avy, avz), density, drag_coefficient, area, mass, pm::<Vec3>(out_acceleration)).0 as jbyte
});
jni!(boolean spaceApplyAtmosphericDragToBody(long world, long body, double avx, double avy, double avz, double density, double drag_coefficient, double area, double mass, int wake_up, long out_acceleration) {
    sf::space_apply_atmospheric_drag_to_body(m::<WH>(world), body as RRaw, v3(avx, avy, avz), density, drag_coefficient, area, mass, jb(wake_up), pm::<Vec3>(out_acceleration)).0 as jbyte
});
jni!(boolean spaceTriadAttitude(double b1x, double b1y, double b1z, double b2x, double b2y, double b2z, double r1x, double r1y, double r1z, double r2x, double r2y, double r2z, long out_attitude) {
    sf::space_triad_attitude(v3(b1x, b1y, b1z), v3(b2x, b2y, b2z), v3(r1x, r1y, r1z), v3(r2x, r2y, r2z), pm::<Quat>(out_attitude)).0 as jbyte
});
jni!(boolean spaceQuaternionDerivative(double qi, double qj, double qk, double qw, double wx, double wy, double wz, long out_derivative) {
    sf::space_quaternion_derivative(qt(qi, qj, qk, qw), v3(wx, wy, wz), pm::<QuaternionDerivative>(out_derivative)).0 as jbyte
});
jni!(boolean spaceEkfPredictScalar(double state, double covariance, double nonlinear_delta, double jacobian, double process_noise, long out_prediction) {
    sf::space_ekf_predict_scalar(state, covariance, nonlinear_delta, jacobian, process_noise, pm::<ScalarKalman>(out_prediction)).0 as jbyte
});
jni!(double spaceEkfGainScalar(double covariance, double measurement_jacobian, double measurement_noise) {
    sf::space_ekf_gain_scalar(covariance, measurement_jacobian, measurement_noise)
});
jni!(boolean spaceEkfUpdateScalar(double predicted_state, double predicted_covariance, double measurement, double predicted_measurement, double kalman_gain, double measurement_jacobian, long out_update) {
    sf::space_ekf_update_scalar(predicted_state, predicted_covariance, measurement, predicted_measurement, kalman_gain, measurement_jacobian, pm::<ScalarKalman>(out_update)).0 as jbyte
});

jni!(long rtreeCreate() { to_jlong(rt::rtree_create()) });
jni!(void rtreeDestroy(long tree) { rt::rtree_destroy(m::<RTH>(tree)); });
jni!(void rtreeClear(long tree) { rt::rtree_clear(m::<RTH>(tree)); });
jni!(int rtreeLen(long tree) { rt::rtree_len(cp::<RTH>(tree)) as jint });
jni!(boolean rtreeInsert(long tree, long id, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z) { rt::rtree_insert(m::<RTH>(tree), id as u64, aa(min_x,min_y,min_z,max_x,max_y,max_z)).0 as jbyte });
jni!(boolean rtreeUpdate(long tree, long id, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z) { rt::rtree_update(m::<RTH>(tree), id as u64, aa(min_x,min_y,min_z,max_x,max_y,max_z)).0 as jbyte });
jni!(boolean rtreeRemove(long tree, long id) { rt::rtree_remove(m::<RTH>(tree), id as u64).0 as jbyte });
jni!(void rtreeRebuild(long tree) { rt::rtree_rebuild(m::<RTH>(tree)); });
jni!(int rtreeQueryAabbCount(long tree, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z) { rt::rtree_query_aabb_count(m::<RTH>(tree), aa(min_x,min_y,min_z,max_x,max_y,max_z)) as jint });
jni!(int rtreeQueryAabb(long tree, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, long out_ids, int capacity) { rt::rtree_query_aabb(m::<RTH>(tree), aa(min_x,min_y,min_z,max_x,max_y,max_z), pm::<u64>(out_ids), u32_from_jint(capacity)) as jint });

jni!(long crbTreeCreate() { to_jlong(crt::crb_tree_create()) });
jni!(void crbTreeDestroy(long tree) { crt::crb_tree_destroy(m::<CRTH>(tree)); });
jni!(void crbTreeClear(long tree) { crt::crb_tree_clear(m::<CRTH>(tree)); });
jni!(int crbTreeLen(long tree) { crt::crb_tree_len(cp::<CRTH>(tree)) as jint });
jni!(boolean crbTreeInsert(long tree, long id, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z) { crt::crb_tree_insert(m::<CRTH>(tree), id as u64, aa(min_x,min_y,min_z,max_x,max_y,max_z)).0 as jbyte });
jni!(boolean crbTreeUpdate(long tree, long id, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z) { crt::crb_tree_update(m::<CRTH>(tree), id as u64, aa(min_x,min_y,min_z,max_x,max_y,max_z)).0 as jbyte });
jni!(boolean crbTreeRemove(long tree, long id) { crt::crb_tree_remove(m::<CRTH>(tree), id as u64).0 as jbyte });
jni!(int crbTreeQueryAabbCount(long tree, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z) { crt::crb_tree_query_aabb_count(cp::<CRTH>(tree), aa(min_x,min_y,min_z,max_x,max_y,max_z)) as jint });
jni!(int crbTreeQueryAabb(long tree, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, long out_ids, int capacity) { crt::crb_tree_query_aabb(cp::<CRTH>(tree), aa(min_x,min_y,min_z,max_x,max_y,max_z), pm::<u64>(out_ids), u32_from_jint(capacity)) as jint });
