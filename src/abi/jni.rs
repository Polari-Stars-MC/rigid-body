use std::ffi::c_void;

use crate::events::{ContactPairFilterCallback, IntersectionPairFilterCallback};
use crate::ffi::{
    AabbDesc, BodyStatus, Bool, Capsule, CharacterCollision, CharacterControllerHandle,
    ColliderBuilderHandle, ColliderHandleRaw, ContactForceEventRecord, Cylinder,
    EffectiveCharacterMovement, Ellipsoid, ImpulseJointHandleRaw, InteractionGroupsDesc,
    JointAxisDesc, JointBuilderHandle, JointTypeDesc, KdopPreset, NeuralActivation,
    NeuralBoundsDesc, Obb, Prism, Quat, QueryFilterDesc, RTreeHandle, RayHit,
    RigidBodyBuilderHandle, RigidBodyHandleRaw, ShapeCastHit, ShapeCastOptionsDesc, ShapeDesc,
    ShapeType, Sphere, SphericalShell, Ssv, Vec3, VoxelColliderMode, VoxelColliderOptions,
    WorldHandle,
};

type JNIEnv = *mut c_void;
type JClass = *mut c_void;
type JByte = i8;
type JDouble = f64;
type JInt = i32;
type JLong = i64;

fn ptr_to_jlong<T>(value: *mut T) -> JLong {
    value as isize as JLong
}

fn jlong_to_mut<T>(value: JLong) -> *mut T {
    value as isize as *mut T
}

fn jlong_to_const<T>(value: JLong) -> *const T {
    value as isize as *const T
}

fn jlong_to_slice<T>(value: JLong) -> *const T {
    value as isize as *const T
}

fn jlong_to_slice_mut<T>(value: JLong) -> *mut T {
    value as isize as *mut T
}

fn bool(value: JInt) -> Bool {
    Bool((value != 0) as u8)
}

fn vec3(x: JDouble, y: JDouble, z: JDouble) -> Vec3 {
    Vec3 { x, y, z }
}

fn quat(i: JDouble, j: JDouble, k: JDouble, w: JDouble) -> Quat {
    Quat { i, j, k, w }
}

fn groups(memberships: JInt, filter: JInt) -> InteractionGroupsDesc {
    InteractionGroupsDesc {
        memberships: memberships as u32,
        filter: filter as u32,
    }
}

fn aabb(
    min_x: JDouble,
    min_y: JDouble,
    min_z: JDouble,
    max_x: JDouble,
    max_y: JDouble,
    max_z: JDouble,
) -> AabbDesc {
    AabbDesc {
        mins: vec3(min_x, min_y, min_z),
        maxs: vec3(max_x, max_y, max_z),
    }
}

fn qfilter(
    flags: JInt,
    memberships: JInt,
    filter: JInt,
    use_groups: JInt,
    exclude_collider: JLong,
    use_exclude_collider: JInt,
    exclude_rigid_body: JLong,
    use_exclude_rigid_body: JInt,
) -> QueryFilterDesc {
    QueryFilterDesc {
        flags: flags as u32,
        groups: groups(memberships, filter),
        use_groups: bool(use_groups),
        exclude_collider: exclude_collider as ColliderHandleRaw,
        use_exclude_collider: bool(use_exclude_collider),
        exclude_rigid_body: exclude_rigid_body as RigidBodyHandleRaw,
        use_exclude_rigid_body: bool(use_exclude_rigid_body),
    }
}

fn shape_type(value: JInt) -> ShapeType {
    match value {
        1 => ShapeType::Cuboid,
        2 => ShapeType::CapsuleY,
        3 => ShapeType::CapsuleX,
        4 => ShapeType::CapsuleZ,
        5 => ShapeType::Cylinder,
        6 => ShapeType::RoundCylinder,
        7 => ShapeType::Cone,
        8 => ShapeType::RoundCone,
        9 => ShapeType::RoundCuboid,
        _ => ShapeType::Ball,
    }
}

fn body_status(value: JInt) -> BodyStatus {
    match value {
        0 => BodyStatus::Dynamic,
        1 => BodyStatus::Fixed,
        2 => BodyStatus::KinematicPositionBased,
        3 => BodyStatus::KinematicVelocityBased,
        _ => BodyStatus::Fixed,
    }
}

fn joint_type(value: JInt) -> JointTypeDesc {
    match value {
        1 => JointTypeDesc::Revolute,
        2 => JointTypeDesc::Prismatic,
        3 => JointTypeDesc::Rope,
        4 => JointTypeDesc::Spring,
        5 => JointTypeDesc::Spherical,
        _ => JointTypeDesc::Fixed,
    }
}

fn joint_axis(value: JInt) -> JointAxisDesc {
    match value {
        1 => JointAxisDesc::LinY,
        2 => JointAxisDesc::LinZ,
        3 => JointAxisDesc::AngX,
        4 => JointAxisDesc::AngY,
        5 => JointAxisDesc::AngZ,
        _ => JointAxisDesc::LinX,
    }
}

fn kdop_preset(value: JInt) -> KdopPreset {
    match value {
        14 => KdopPreset::K14,
        18 => KdopPreset::K18,
        26 => KdopPreset::K26,
        _ => KdopPreset::K6,
    }
}

fn neural_activation(value: JInt) -> NeuralActivation {
    match value {
        1 => NeuralActivation::Tanh,
        2 => NeuralActivation::Sin,
        3 => NeuralActivation::Linear,
        _ => NeuralActivation::Relu,
    }
}

fn voxel_mode(value: JInt) -> VoxelColliderMode {
    match value {
        1 => VoxelColliderMode::Cuboids,
        2 => VoxelColliderMode::GreedyCuboids,
        3 => VoxelColliderMode::SurfaceMesh,
        _ => VoxelColliderMode::Auto,
    }
}

fn shape_desc(shape_type: JInt, a: JDouble, b: JDouble, c: JDouble, d: JDouble) -> ShapeDesc {
    ShapeDesc {
        shape_type: self::shape_type(shape_type),
        a,
        b,
        c,
        d,
    }
}

macro_rules! jni {
    ($name:ident ( $($arg:ident : $ty:ty),* ) -> $ret:ty $body:block) => {
        #[unsafe(no_mangle)]
        pub extern "system" fn $name(_env: JNIEnv, _class: JClass, $($arg: $ty),*) -> $ret $body
    };
    ($name:ident ( $($arg:ident : $ty:ty),* ) $body:block) => {
        #[unsafe(no_mangle)]
        pub extern "system" fn $name(_env: JNIEnv, _class: JClass, $($arg: $ty),*) $body
    };
}

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_abiVersion() -> JInt {
    crate::abi::ffm::abi_version() as JInt
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_abiSupportsFfm() -> JByte {
    crate::abi::ffm::abi_supports_ffm().0 as JByte
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_abiSupportsJni() -> JByte {
    crate::abi::ffm::abi_supports_jni().0 as JByte
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldCreate(gravity_x: JDouble, gravity_y: JDouble, gravity_z: JDouble) -> JLong {
    ptr_to_jlong(crate::world::world_create(vec3(gravity_x, gravity_y, gravity_z)))
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldDestroy(world: JLong) {
    crate::world::world_destroy(jlong_to_mut::<WorldHandle>(world));
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldStep(world: JLong, delta_seconds: JDouble) {
    crate::world::world_step(jlong_to_mut::<WorldHandle>(world), delta_seconds);
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldSetGravity(world: JLong, x: JDouble, y: JDouble, z: JDouble) {
    crate::world::world_set_gravity(jlong_to_mut::<WorldHandle>(world), vec3(x, y, z));
});

fn world_gravity(world: JLong) -> Vec3 {
    crate::world::world_get_gravity(jlong_to_const::<WorldHandle>(world))
}

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldGetGravityX(world: JLong) -> JDouble { world_gravity(world).x });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldGetGravityY(world: JLong) -> JDouble { world_gravity(world).y });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldGetGravityZ(world: JLong) -> JDouble { world_gravity(world).z });

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldDynamicBodySnapshotCount(world: JLong) -> JInt {
    crate::world::world_dynamic_body_snapshot_count(jlong_to_const::<WorldHandle>(world)) as JInt
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldDynamicBodySnapshot(world: JLong, out_handles: JLong, out_values: JLong, capacity: JInt) -> JInt {
    crate::world::world_dynamic_body_snapshot(
        jlong_to_const::<WorldHandle>(world),
        jlong_to_slice_mut::<RigidBodyHandleRaw>(out_handles),
        jlong_to_slice_mut::<f64>(out_values),
        capacity as u32,
    ) as JInt
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreate(shape_type: JInt, a: JDouble, b: JDouble, c: JDouble) -> JLong {
    ptr_to_jlong(crate::collider::collider_builder_create(self::shape_type(shape_type), vec3(a, b, c)))
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateEx(shape_type: JInt, a: JDouble, b: JDouble, c: JDouble, d: JDouble) -> JLong {
    ptr_to_jlong(crate::collider::collider_builder_create_ex(shape_desc(shape_type, a, b, c, d)))
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateSphere(x: JDouble, y: JDouble, z: JDouble, radius: JDouble) -> JLong {
    ptr_to_jlong(crate::collider::collider_builder_create_sphere(Sphere { center: vec3(x, y, z), radius }))
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateObb(cx: JDouble, cy: JDouble, cz: JDouble, hx: JDouble, hy: JDouble, hz: JDouble, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble) -> JLong {
    ptr_to_jlong(crate::collider::collider_builder_create_obb(Obb {
        center: vec3(cx, cy, cz),
        half_extents: vec3(hx, hy, hz),
        rotation: quat(qi, qj, qk, qw),
    }))
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateConvexHull(points_xyz: JLong, point_count: JInt) -> JLong {
    ptr_to_jlong(crate::collider::collider_builder_create_convex_hull(jlong_to_slice::<f64>(points_xyz), point_count as u32))
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderDestroy(builder: JLong) {
    crate::collider::collider_builder_destroy(jlong_to_mut::<ColliderBuilderHandle>(builder));
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderSetTranslation(builder: JLong, x: JDouble, y: JDouble, z: JDouble) {
    crate::collider::collider_builder_set_translation(jlong_to_mut::<ColliderBuilderHandle>(builder), vec3(x, y, z));
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderSetRotation(builder: JLong, x: JDouble, y: JDouble, z: JDouble) {
    crate::collider::collider_builder_set_rotation(jlong_to_mut::<ColliderBuilderHandle>(builder), vec3(x, y, z));
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderSetPose(builder: JLong, x: JDouble, y: JDouble, z: JDouble, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble) {
    crate::collider::collider_builder_set_pose(jlong_to_mut::<ColliderBuilderHandle>(builder), vec3(x, y, z), quat(qi, qj, qk, qw));
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderSetSensor(builder: JLong, sensor: JInt) {
    crate::collider::collider_builder_set_sensor(jlong_to_mut::<ColliderBuilderHandle>(builder), bool(sensor));
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderSetFriction(builder: JLong, friction: JDouble) { crate::collider::collider_builder_set_friction(jlong_to_mut::<ColliderBuilderHandle>(builder), friction); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderSetRestitution(builder: JLong, restitution: JDouble) { crate::collider::collider_builder_set_restitution(jlong_to_mut::<ColliderBuilderHandle>(builder), restitution); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderSetDensity(builder: JLong, density: JDouble) { crate::collider::collider_builder_set_density(jlong_to_mut::<ColliderBuilderHandle>(builder), density); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderSetCollisionGroups(builder: JLong, memberships: JInt, filter: JInt) { crate::collider::collider_builder_set_collision_groups(jlong_to_mut::<ColliderBuilderHandle>(builder), groups(memberships, filter)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderSetSolverGroups(builder: JLong, memberships: JInt, filter: JInt) { crate::collider::collider_builder_set_solver_groups(jlong_to_mut::<ColliderBuilderHandle>(builder), groups(memberships, filter)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderSetActiveEvents(builder: JLong, bits: JInt) { crate::collider::collider_builder_set_active_events(jlong_to_mut::<ColliderBuilderHandle>(builder), bits as u32); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderSetActiveHooks(builder: JLong, bits: JInt) { crate::collider::collider_builder_set_active_hooks(jlong_to_mut::<ColliderBuilderHandle>(builder), bits as u32); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderSetContactForceEventThreshold(builder: JLong, threshold: JDouble) { crate::collider::collider_builder_set_contact_force_event_threshold(jlong_to_mut::<ColliderBuilderHandle>(builder), threshold); });

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldInsertCollider(world: JLong, builder: JLong) -> JLong {
    crate::collider::world_insert_collider(jlong_to_mut::<WorldHandle>(world), jlong_to_mut::<ColliderBuilderHandle>(builder)) as JLong
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldInsertColliderWithParent(world: JLong, builder: JLong, parent: JLong) -> JLong {
    crate::collider::world_insert_collider_with_parent(jlong_to_mut::<WorldHandle>(world), jlong_to_mut::<ColliderBuilderHandle>(builder), parent as RigidBodyHandleRaw) as JLong
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldRemoveCollider(world: JLong, handle: JLong, wake_up: JInt) -> JByte {
    crate::collider::world_remove_collider(jlong_to_mut::<WorldHandle>(world), handle as ColliderHandleRaw, bool(wake_up)).0 as JByte
});

fn collider_translation(world: JLong, handle: JLong) -> Vec3 {
    crate::collider::collider_get_translation(
        jlong_to_const::<WorldHandle>(world),
        handle as ColliderHandleRaw,
    )
}
fn collider_rotation(world: JLong, handle: JLong) -> Quat {
    crate::collider::collider_get_rotation(
        jlong_to_const::<WorldHandle>(world),
        handle as ColliderHandleRaw,
    )
}
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderGetTranslationX(world: JLong, handle: JLong) -> JDouble { collider_translation(world, handle).x });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderGetTranslationY(world: JLong, handle: JLong) -> JDouble { collider_translation(world, handle).y });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderGetTranslationZ(world: JLong, handle: JLong) -> JDouble { collider_translation(world, handle).z });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderGetRotationI(world: JLong, handle: JLong) -> JDouble { collider_rotation(world, handle).i });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderGetRotationJ(world: JLong, handle: JLong) -> JDouble { collider_rotation(world, handle).j });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderGetRotationK(world: JLong, handle: JLong) -> JDouble { collider_rotation(world, handle).k });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderGetRotationW(world: JLong, handle: JLong) -> JDouble { collider_rotation(world, handle).w });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderSetPose(world: JLong, handle: JLong, x: JDouble, y: JDouble, z: JDouble, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble) -> JByte { crate::collider::collider_set_pose(jlong_to_mut::<WorldHandle>(world), handle as ColliderHandleRaw, vec3(x, y, z), quat(qi, qj, qk, qw)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderSetSensor(world: JLong, handle: JLong, sensor: JInt) -> JByte { crate::collider::collider_set_sensor(jlong_to_mut::<WorldHandle>(world), handle as ColliderHandleRaw, bool(sensor)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderSetFriction(world: JLong, handle: JLong, friction: JDouble) -> JByte { crate::collider::collider_set_friction(jlong_to_mut::<WorldHandle>(world), handle as ColliderHandleRaw, friction).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderSetRestitution(world: JLong, handle: JLong, restitution: JDouble) -> JByte { crate::collider::collider_set_restitution(jlong_to_mut::<WorldHandle>(world), handle as ColliderHandleRaw, restitution).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderSetCollisionGroups(world: JLong, handle: JLong, memberships: JInt, filter: JInt) -> JByte { crate::collider::collider_set_collision_groups(jlong_to_mut::<WorldHandle>(world), handle as ColliderHandleRaw, groups(memberships, filter)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderSetSolverGroups(world: JLong, handle: JLong, memberships: JInt, filter: JInt) -> JByte { crate::collider::collider_set_solver_groups(jlong_to_mut::<WorldHandle>(world), handle as ColliderHandleRaw, groups(memberships, filter)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderSetActiveEvents(world: JLong, handle: JLong, bits: JInt) -> JByte { crate::collider::collider_set_active_events(jlong_to_mut::<WorldHandle>(world), handle as ColliderHandleRaw, bits as u32).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderSetActiveHooks(world: JLong, handle: JLong, bits: JInt) -> JByte { crate::collider::collider_set_active_hooks(jlong_to_mut::<WorldHandle>(world), handle as ColliderHandleRaw, bits as u32).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderSetContactForceEventThreshold(world: JLong, handle: JLong, threshold: JDouble) -> JByte { crate::collider::collider_set_contact_force_event_threshold(jlong_to_mut::<WorldHandle>(world), handle as ColliderHandleRaw, threshold).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderGetDensity(world: JLong, handle: JLong) -> JDouble { crate::collider::collider_get_density(jlong_to_const::<WorldHandle>(world), handle as ColliderHandleRaw) });

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderCreate(status: JInt) -> JLong {
    ptr_to_jlong(crate::rigid_body::rigid_body_builder_create(body_status(status)))
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderDestroy(builder: JLong) { crate::rigid_body::rigid_body_builder_destroy(jlong_to_mut::<RigidBodyBuilderHandle>(builder)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetTranslation(builder: JLong, x: JDouble, y: JDouble, z: JDouble) { crate::rigid_body::rigid_body_builder_set_translation(jlong_to_mut::<RigidBodyBuilderHandle>(builder), vec3(x, y, z)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetRotation(builder: JLong, x: JDouble, y: JDouble, z: JDouble) { crate::rigid_body::rigid_body_builder_set_rotation(jlong_to_mut::<RigidBodyBuilderHandle>(builder), vec3(x, y, z)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetPose(builder: JLong, x: JDouble, y: JDouble, z: JDouble, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble) { crate::rigid_body::rigid_body_builder_set_pose(jlong_to_mut::<RigidBodyBuilderHandle>(builder), vec3(x, y, z), quat(qi, qj, qk, qw)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetLinvel(builder: JLong, x: JDouble, y: JDouble, z: JDouble) { crate::rigid_body::rigid_body_builder_set_linvel(jlong_to_mut::<RigidBodyBuilderHandle>(builder), vec3(x, y, z)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetAngvel(builder: JLong, x: JDouble, y: JDouble, z: JDouble) { crate::rigid_body::rigid_body_builder_set_angvel(jlong_to_mut::<RigidBodyBuilderHandle>(builder), vec3(x, y, z)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetGravityScale(builder: JLong, value: JDouble) { crate::rigid_body::rigid_body_builder_set_gravity_scale(jlong_to_mut::<RigidBodyBuilderHandle>(builder), value); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetLinearDamping(builder: JLong, value: JDouble) { crate::rigid_body::rigid_body_builder_set_linear_damping(jlong_to_mut::<RigidBodyBuilderHandle>(builder), value); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetAngularDamping(builder: JLong, value: JDouble) { crate::rigid_body::rigid_body_builder_set_angular_damping(jlong_to_mut::<RigidBodyBuilderHandle>(builder), value); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetCanSleep(builder: JLong, value: JInt) { crate::rigid_body::rigid_body_builder_set_can_sleep(jlong_to_mut::<RigidBodyBuilderHandle>(builder), bool(value)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetEnabledRotations(builder: JLong, x: JInt, y: JInt, z: JInt) { crate::rigid_body::rigid_body_builder_set_enabled_rotations(jlong_to_mut::<RigidBodyBuilderHandle>(builder), bool(x), bool(y), bool(z)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetUserData(builder: JLong, low: JLong, high: JLong) { crate::rigid_body::rigid_body_builder_set_user_data(jlong_to_mut::<RigidBodyBuilderHandle>(builder), low as u64, high as u64); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyBuilderSetAdditionalMass(builder: JLong, mass: JDouble) { crate::rigid_body::rigid_body_builder_set_additional_mass(jlong_to_mut::<RigidBodyBuilderHandle>(builder), mass); });

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldInsertRigidBody(world: JLong, builder: JLong) -> JLong {
    crate::rigid_body::world_insert_rigid_body(jlong_to_mut::<WorldHandle>(world), jlong_to_mut::<RigidBodyBuilderHandle>(builder)) as JLong
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldRemoveRigidBody(world: JLong, handle: JLong, remove_attached_colliders: JInt) -> JByte { crate::rigid_body::world_remove_rigid_body(jlong_to_mut::<WorldHandle>(world), handle as RigidBodyHandleRaw, bool(remove_attached_colliders)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetStatus(world: JLong, handle: JLong) -> JInt { crate::rigid_body::rigid_body_get_status(jlong_to_const::<WorldHandle>(world), handle as RigidBodyHandleRaw) as JInt });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodySetStatus(world: JLong, handle: JLong, status: JInt, wake_up: JInt) -> JByte { crate::rigid_body::rigid_body_set_status(jlong_to_mut::<WorldHandle>(world), handle as RigidBodyHandleRaw, body_status(status), bool(wake_up)).0 as JByte });

fn rb_translation(world: JLong, body: JLong) -> Vec3 {
    crate::rigid_body::rigid_body_get_translation(
        jlong_to_const::<WorldHandle>(world),
        body as RigidBodyHandleRaw,
    )
}
fn rb_rotation(world: JLong, body: JLong) -> Quat {
    crate::rigid_body::rigid_body_get_rotation(
        jlong_to_const::<WorldHandle>(world),
        body as RigidBodyHandleRaw,
    )
}
fn rb_linvel(world: JLong, body: JLong) -> Vec3 {
    crate::rigid_body::rigid_body_get_linvel(
        jlong_to_const::<WorldHandle>(world),
        body as RigidBodyHandleRaw,
    )
}
fn rb_angvel(world: JLong, body: JLong) -> Vec3 {
    crate::rigid_body::rigid_body_get_angvel(
        jlong_to_const::<WorldHandle>(world),
        body as RigidBodyHandleRaw,
    )
}
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetTranslationX(world: JLong, body: JLong) -> JDouble { rb_translation(world, body).x });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetTranslationY(world: JLong, body: JLong) -> JDouble { rb_translation(world, body).y });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetTranslationZ(world: JLong, body: JLong) -> JDouble { rb_translation(world, body).z });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetRotationI(world: JLong, body: JLong) -> JDouble { rb_rotation(world, body).i });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetRotationJ(world: JLong, body: JLong) -> JDouble { rb_rotation(world, body).j });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetRotationK(world: JLong, body: JLong) -> JDouble { rb_rotation(world, body).k });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetRotationW(world: JLong, body: JLong) -> JDouble { rb_rotation(world, body).w });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodySetPose(world: JLong, body: JLong, x: JDouble, y: JDouble, z: JDouble, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble, wake_up: JInt) -> JByte { crate::rigid_body::rigid_body_set_pose(jlong_to_mut::<WorldHandle>(world), body as RigidBodyHandleRaw, vec3(x, y, z), quat(qi, qj, qk, qw), bool(wake_up)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetLinvelX(world: JLong, body: JLong) -> JDouble { rb_linvel(world, body).x });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetLinvelY(world: JLong, body: JLong) -> JDouble { rb_linvel(world, body).y });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetLinvelZ(world: JLong, body: JLong) -> JDouble { rb_linvel(world, body).z });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodySetLinvel(world: JLong, body: JLong, x: JDouble, y: JDouble, z: JDouble, wake_up: JInt) -> JByte { crate::rigid_body::rigid_body_set_linvel(jlong_to_mut::<WorldHandle>(world), body as RigidBodyHandleRaw, vec3(x, y, z), bool(wake_up)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetAngvelX(world: JLong, body: JLong) -> JDouble { rb_angvel(world, body).x });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetAngvelY(world: JLong, body: JLong) -> JDouble { rb_angvel(world, body).y });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyGetAngvelZ(world: JLong, body: JLong) -> JDouble { rb_angvel(world, body).z });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodySetAngvel(world: JLong, body: JLong, x: JDouble, y: JDouble, z: JDouble, wake_up: JInt) -> JByte { crate::rigid_body::rigid_body_set_angvel(jlong_to_mut::<WorldHandle>(world), body as RigidBodyHandleRaw, vec3(x, y, z), bool(wake_up)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyAddForce(world: JLong, body: JLong, x: JDouble, y: JDouble, z: JDouble, wake_up: JInt) -> JByte { crate::rigid_body::rigid_body_add_force(jlong_to_mut::<WorldHandle>(world), body as RigidBodyHandleRaw, vec3(x, y, z), bool(wake_up)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyAddTorque(world: JLong, body: JLong, x: JDouble, y: JDouble, z: JDouble, wake_up: JInt) -> JByte { crate::rigid_body::rigid_body_add_torque(jlong_to_mut::<WorldHandle>(world), body as RigidBodyHandleRaw, vec3(x, y, z), bool(wake_up)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyApplyImpulse(world: JLong, body: JLong, x: JDouble, y: JDouble, z: JDouble, wake_up: JInt) -> JByte { crate::rigid_body::rigid_body_apply_impulse(jlong_to_mut::<WorldHandle>(world), body as RigidBodyHandleRaw, vec3(x, y, z), bool(wake_up)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyApplyTorqueImpulse(world: JLong, body: JLong, x: JDouble, y: JDouble, z: JDouble, wake_up: JInt) -> JByte { crate::rigid_body::rigid_body_apply_torque_impulse(jlong_to_mut::<WorldHandle>(world), body as RigidBodyHandleRaw, vec3(x, y, z), bool(wake_up)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyEnableCcd(world: JLong, body: JLong, enabled: JInt) -> JByte { crate::rigid_body::rigid_body_enable_ccd(jlong_to_mut::<WorldHandle>(world), body as RigidBodyHandleRaw, bool(enabled)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodySleep(world: JLong, body: JLong) -> JByte { crate::rigid_body::rigid_body_sleep(jlong_to_mut::<WorldHandle>(world), body as RigidBodyHandleRaw).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyWakeUp(world: JLong, body: JLong, strong: JInt) -> JByte { crate::rigid_body::rigid_body_wake_up(jlong_to_mut::<WorldHandle>(world), body as RigidBodyHandleRaw, bool(strong)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rigidBodyIsSleeping(world: JLong, body: JLong) -> JByte { crate::rigid_body::rigid_body_is_sleeping(jlong_to_const::<WorldHandle>(world), body as RigidBodyHandleRaw).0 as JByte });

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateCapsule(ax: JDouble, ay: JDouble, az: JDouble, bx: JDouble, by: JDouble, bz: JDouble, radius: JDouble) -> JLong {
    ptr_to_jlong(crate::bounds::collider_builder_create_capsule(Capsule { a: vec3(ax, ay, az), b: vec3(bx, by, bz), radius }))
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateSsv(ax: JDouble, ay: JDouble, az: JDouble, bx: JDouble, by: JDouble, bz: JDouble, radius: JDouble) -> JLong {
    ptr_to_jlong(crate::bounds::collider_builder_create_ssv(Ssv { a: vec3(ax, ay, az), b: vec3(bx, by, bz), radius }))
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateEllipsoid(cx: JDouble, cy: JDouble, cz: JDouble, rx: JDouble, ry: JDouble, rz: JDouble, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble, segments: JInt) -> JLong {
    ptr_to_jlong(crate::bounds::collider_builder_create_ellipsoid(Ellipsoid { center: vec3(cx, cy, cz), radii: vec3(rx, ry, rz), rotation: quat(qi, qj, qk, qw), segments: segments as u32 }))
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreatePrism(cx: JDouble, cy: JDouble, cz: JDouble, radius: JDouble, half_height: JDouble, sides: JInt, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble) -> JLong {
    ptr_to_jlong(crate::bounds::collider_builder_create_prism(Prism { center: vec3(cx, cy, cz), radius, half_height, sides: sides as u32, rotation: quat(qi, qj, qk, qw) }))
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateCylinder(cx: JDouble, cy: JDouble, cz: JDouble, radius: JDouble, half_height: JDouble, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble) -> JLong {
    ptr_to_jlong(crate::bounds::collider_builder_create_cylinder(Cylinder { center: vec3(cx, cy, cz), radius, half_height, rotation: quat(qi, qj, qk, qw) }))
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateSphericalShell(cx: JDouble, cy: JDouble, cz: JDouble, inner_radius: JDouble, outer_radius: JDouble) -> JLong {
    ptr_to_jlong(crate::bounds::collider_builder_create_spherical_shell(SphericalShell { center: vec3(cx, cy, cz), inner_radius, outer_radius }))
});

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

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_queryCastRay(world: JLong, ox: JDouble, oy: JDouble, oz: JDouble, dx: JDouble, dy: JDouble, dz: JDouble, max_toi: JDouble, solid: JInt, flags: JInt, memberships: JInt, filter: JInt, use_groups: JInt, exclude_collider: JLong, use_exclude_collider: JInt, exclude_rigid_body: JLong, use_exclude_rigid_body: JInt, out_hit: JLong) -> JLong {
    let hit = crate::query::query_cast_ray(jlong_to_const::<WorldHandle>(world), vec3(ox, oy, oz), vec3(dx, dy, dz), max_toi, bool(solid), query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body));
    if let Some(out) = unsafe { jlong_to_slice_mut::<RayHit>(out_hit).as_mut() } { *out = hit; }
    hit.collider as JLong
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_queryIntersectAabbCount(world: JLong, min_x: JDouble, min_y: JDouble, min_z: JDouble, max_x: JDouble, max_y: JDouble, max_z: JDouble, flags: JInt, memberships: JInt, filter: JInt, use_groups: JInt, exclude_collider: JLong, use_exclude_collider: JInt, exclude_rigid_body: JLong, use_exclude_rigid_body: JInt) -> JInt {
    crate::query::query_intersect_aabb_count(jlong_to_const::<WorldHandle>(world), aabb(min_x,min_y,min_z,max_x,max_y,max_z), query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body)) as JInt
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_queryIntersectObb(world: JLong, cx: JDouble, cy: JDouble, cz: JDouble, hx: JDouble, hy: JDouble, hz: JDouble, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble, flags: JInt, memberships: JInt, filter: JInt, use_groups: JInt, exclude_collider: JLong, use_exclude_collider: JInt, exclude_rigid_body: JLong, use_exclude_rigid_body: JInt, out_handles: JLong, capacity: JInt) -> JInt {
    crate::query::query_intersect_obb(jlong_to_const::<WorldHandle>(world), Obb { center: vec3(cx,cy,cz), half_extents: vec3(hx,hy,hz), rotation: quat(qi,qj,qk,qw) }, query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body), jlong_to_slice_mut::<ColliderHandleRaw>(out_handles), capacity as u32) as JInt
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_queryIntersectSphere(world: JLong, cx: JDouble, cy: JDouble, cz: JDouble, radius: JDouble, flags: JInt, memberships: JInt, filter: JInt, use_groups: JInt, exclude_collider: JLong, use_exclude_collider: JInt, exclude_rigid_body: JLong, use_exclude_rigid_body: JInt, out_handles: JLong, capacity: JInt) -> JInt {
    crate::query::query_intersect_sphere(jlong_to_const::<WorldHandle>(world), Sphere { center: vec3(cx,cy,cz), radius }, query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body), jlong_to_slice_mut::<ColliderHandleRaw>(out_handles), capacity as u32) as JInt
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_queryCastShape(world: JLong, shape_type: JInt, a: JDouble, b: JDouble, c: JDouble, d: JDouble, tx: JDouble, ty: JDouble, tz: JDouble, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble, vx: JDouble, vy: JDouble, vz: JDouble, max_toi: JDouble, target_distance: JDouble, stop_at_penetration: JInt, compute_impact_geometry_on_penetration: JInt, flags: JInt, memberships: JInt, filter: JInt, use_groups: JInt, exclude_collider: JLong, use_exclude_collider: JInt, exclude_rigid_body: JLong, use_exclude_rigid_body: JInt, out_hit: JLong) -> JLong {
    let hit = crate::query::query_cast_shape(
        jlong_to_const::<WorldHandle>(world),
        shape_desc(shape_type, a, b, c, d),
        vec3(tx,ty,tz),
        quat(qi,qj,qk,qw),
        vec3(vx,vy,vz),
        ShapeCastOptionsDesc { max_time_of_impact: max_toi, target_distance, stop_at_penetration: bool(stop_at_penetration), compute_impact_geometry_on_penetration: bool(compute_impact_geometry_on_penetration) },
        query_filter_args!(flags,memberships,filter,use_groups,exclude_collider,use_exclude_collider,exclude_rigid_body,use_exclude_rigid_body),
    );
    if let Some(out) = unsafe { jlong_to_slice_mut::<ShapeCastHit>(out_hit).as_mut() } { *out = hit; }
    hit.collider as JLong
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateKdop(points_xyz: JLong, point_count: JInt, preset: JInt) -> JLong {
    ptr_to_jlong(crate::dop::collider_builder_create_kdop(jlong_to_slice::<f64>(points_xyz), point_count as u32, kdop_preset(preset)))
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateFdh(points_xyz: JLong, point_count: JInt, directions_xyz: JLong, direction_count: JInt) -> JLong {
    ptr_to_jlong(crate::dop::collider_builder_create_fdh(jlong_to_slice::<f64>(points_xyz), point_count as u32, jlong_to_slice::<f64>(directions_xyz), direction_count as u32))
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_neuralBoundsRequiredWeightCount(hidden_width: JInt, hidden_layers: JInt) -> JInt {
    crate::neural::neural_bounds_required_weight_count(hidden_width as u32, hidden_layers as u32) as JInt
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateNeuralBounds(cx: JDouble, cy: JDouble, cz: JDouble, hx: JDouble, hy: JDouble, hz: JDouble, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble, sample_resolution: JInt, hidden_width: JInt, hidden_layers: JInt, activation: JInt, output_scale: JDouble, padding: JDouble, weights: JLong, weight_count: JInt) -> JLong {
    ptr_to_jlong(crate::neural::collider_builder_create_neural_bounds(NeuralBoundsDesc {
        center: vec3(cx,cy,cz), half_extents: vec3(hx,hy,hz), rotation: quat(qi,qj,qk,qw),
        sample_resolution: sample_resolution as u32, hidden_width: hidden_width as u32, hidden_layers: hidden_layers as u32,
        activation: neural_activation(activation), output_scale, padding,
    }, jlong_to_slice::<f64>(weights), weight_count as u32))
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_colliderBuilderCreateVoxels(voxels: JLong, size_x: JInt, size_y: JInt, size_z: JInt, voxel_size: JDouble, origin_x: JDouble, origin_y: JDouble, origin_z: JDouble, mode: JInt, dynamic_body: JInt, small_voxel_limit: JInt, mesh_voxel_limit: JInt) -> JLong {
    ptr_to_jlong(crate::voxel::collider_builder_create_voxels(jlong_to_slice::<u8>(voxels), size_x as u32, size_y as u32, size_z as u32, voxel_size, vec3(origin_x, origin_y, origin_z), VoxelColliderOptions {
        mode: voxel_mode(mode), dynamic_body: bool(dynamic_body), small_voxel_limit: small_voxel_limit as u32, mesh_voxel_limit: mesh_voxel_limit as u32,
    }))
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldInsertDynamicCuboids(world: JLong, x: JDouble, y: JDouble, z: JDouble, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble, lvx: JDouble, lvy: JDouble, lvz: JDouble, cuboids: JLong, cuboid_count: JInt, density: JDouble, friction: JDouble, restitution: JDouble, collision_memberships: JInt, collision_filter: JInt, solver_memberships: JInt, solver_filter: JInt) -> JLong {
    crate::compat::world_insert_dynamic_cuboids(jlong_to_mut::<WorldHandle>(world), vec3(x,y,z), quat(qi,qj,qk,qw), vec3(lvx,lvy,lvz), jlong_to_slice::<f64>(cuboids), cuboid_count as u32, density, friction, restitution, groups(collision_memberships, collision_filter), groups(solver_memberships, solver_filter)) as JLong
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldInsertStaticTrimesh(world: JLong, vertices_xyz: JLong, vertex_xyz_len: JInt, indices: JLong, index_len: JInt, friction: JDouble, restitution: JDouble) -> JLong {
    crate::compat::world_insert_static_trimesh(jlong_to_mut::<WorldHandle>(world), jlong_to_slice::<f64>(vertices_xyz), vertex_xyz_len as u32, jlong_to_slice::<u32>(indices), index_len as u32, friction, restitution) as JLong
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_jointBuilderCreate(joint_type: JInt, ax: JDouble, ay: JDouble, az: JDouble, b: JDouble, c: JDouble) -> JLong {
    ptr_to_jlong(crate::joints::joint_builder_create(self::joint_type(joint_type), vec3(ax, ay, az), b, c))
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_jointBuilderDestroy(builder: JLong) { crate::joints::joint_builder_destroy(jlong_to_mut::<JointBuilderHandle>(builder)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_jointBuilderSetContactsEnabled(builder: JLong, enabled: JInt) { crate::joints::joint_builder_set_contacts_enabled(jlong_to_mut::<JointBuilderHandle>(builder), bool(enabled)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_jointBuilderSetLocalAnchor1(builder: JLong, x: JDouble, y: JDouble, z: JDouble) { crate::joints::joint_builder_set_local_anchor1(jlong_to_mut::<JointBuilderHandle>(builder), vec3(x,y,z)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_jointBuilderSetLocalAnchor2(builder: JLong, x: JDouble, y: JDouble, z: JDouble) { crate::joints::joint_builder_set_local_anchor2(jlong_to_mut::<JointBuilderHandle>(builder), vec3(x,y,z)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_jointBuilderSetLimits(builder: JLong, axis: JInt, min: JDouble, max: JDouble) { crate::joints::joint_builder_set_limits(jlong_to_mut::<JointBuilderHandle>(builder), joint_axis(axis), min, max); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_jointBuilderSetMotorVelocity(builder: JLong, axis: JInt, target_vel: JDouble, factor: JDouble) { crate::joints::joint_builder_set_motor_velocity(jlong_to_mut::<JointBuilderHandle>(builder), joint_axis(axis), target_vel, factor); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_jointBuilderSetMotorPosition(builder: JLong, axis: JInt, target_pos: JDouble, stiffness: JDouble, damping: JDouble) { crate::joints::joint_builder_set_motor_position(jlong_to_mut::<JointBuilderHandle>(builder), joint_axis(axis), target_pos, stiffness, damping); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldInsertImpulseJoint(world: JLong, body1: JLong, body2: JLong, builder: JLong, wake_up: JInt) -> JLong { crate::joints::world_insert_impulse_joint(jlong_to_mut::<WorldHandle>(world), body1 as RigidBodyHandleRaw, body2 as RigidBodyHandleRaw, jlong_to_mut::<JointBuilderHandle>(builder), bool(wake_up)) as JLong });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldRemoveImpulseJoint(world: JLong, handle: JLong, wake_up: JInt) -> JByte { crate::joints::world_remove_impulse_joint(jlong_to_mut::<WorldHandle>(world), handle as ImpulseJointHandleRaw, bool(wake_up)).0 as JByte });

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerCreate() -> JLong { ptr_to_jlong(crate::controller::character_controller_create()) });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerDestroy(controller: JLong) { crate::controller::character_controller_destroy(jlong_to_mut::<CharacterControllerHandle>(controller)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerSetUp(controller: JLong, x: JDouble, y: JDouble, z: JDouble) { crate::controller::character_controller_set_up(jlong_to_mut::<CharacterControllerHandle>(controller), vec3(x,y,z)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerSetOffsetAbsolute(controller: JLong, offset: JDouble) { crate::controller::character_controller_set_offset_absolute(jlong_to_mut::<CharacterControllerHandle>(controller), offset); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerSetOffsetRelative(controller: JLong, offset: JDouble) { crate::controller::character_controller_set_offset_relative(jlong_to_mut::<CharacterControllerHandle>(controller), offset); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerSetSlide(controller: JLong, slide: JInt) { crate::controller::character_controller_set_slide(jlong_to_mut::<CharacterControllerHandle>(controller), bool(slide)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerSetAutostep(controller: JLong, enabled: JInt, max_height: JDouble, min_width: JDouble, include_dynamic_bodies: JInt) { crate::controller::character_controller_set_autostep(jlong_to_mut::<CharacterControllerHandle>(controller), bool(enabled), max_height, min_width, bool(include_dynamic_bodies)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerSetSnapToGround(controller: JLong, enabled: JInt, distance: JDouble) { crate::controller::character_controller_set_snap_to_ground(jlong_to_mut::<CharacterControllerHandle>(controller), bool(enabled), distance); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerSetSlopeAngles(controller: JLong, max_climb_angle: JDouble, min_slide_angle: JDouble) { crate::controller::character_controller_set_slope_angles(jlong_to_mut::<CharacterControllerHandle>(controller), max_climb_angle, min_slide_angle); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerMoveShape(world: JLong, controller: JLong, dt: JDouble, shape_type: JInt, a: JDouble, b: JDouble, c: JDouble, d: JDouble, tx: JDouble, ty: JDouble, tz: JDouble, qi: JDouble, qj: JDouble, qk: JDouble, qw: JDouble, dx: JDouble, dy: JDouble, dz: JDouble, out_movement: JLong) -> JByte {
    let movement = crate::controller::character_controller_move_shape(jlong_to_const::<WorldHandle>(world), jlong_to_mut::<CharacterControllerHandle>(controller), dt, shape_desc(shape_type,a,b,c,d), vec3(tx,ty,tz), quat(qi,qj,qk,qw), vec3(dx,dy,dz));
    if let Some(out) = unsafe { jlong_to_slice_mut::<EffectiveCharacterMovement>(out_movement).as_mut() } { *out = movement; }
    movement.grounded.0 as JByte
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerCollisionCount(controller: JLong) -> JInt { crate::controller::character_controller_collision_count(jlong_to_const::<CharacterControllerHandle>(controller)) as JInt });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerGetCollision(controller: JLong, index: JInt, out_collision: JLong) -> JLong {
    let collision = crate::controller::character_controller_get_collision(jlong_to_const::<CharacterControllerHandle>(controller), index as u32);
    if let Some(out) = unsafe { jlong_to_slice_mut::<CharacterCollision>(out_collision).as_mut() } { *out = collision; }
    collision.collider as JLong
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_characterControllerSolveImpulses(world: JLong, controller: JLong, dt: JDouble, shape_type: JInt, a: JDouble, b: JDouble, c: JDouble, d: JDouble, character_mass: JDouble) -> JByte {
    crate::controller::character_controller_solve_impulses(jlong_to_mut::<WorldHandle>(world), jlong_to_mut::<CharacterControllerHandle>(controller), dt, shape_desc(shape_type,a,b,c,d), character_mass).0 as JByte
});

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldClearEvents(world: JLong) { crate::events::world_clear_events(jlong_to_mut::<WorldHandle>(world)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldCollisionEventCount(world: JLong) -> JInt { crate::events::world_collision_event_count(jlong_to_const::<WorldHandle>(world)) as JInt });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldGetCollisionEvent(world: JLong, index: JInt, out_event: JLong) -> JLong {
    let event = crate::events::world_get_collision_event(jlong_to_const::<WorldHandle>(world), index as u32);
    if let Some(out) = unsafe { jlong_to_slice_mut::<crate::ffi::CollisionEventRecord>(out_event).as_mut() } { *out = event; }
    event.collider1 as JLong
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldContactForceEventCount(world: JLong) -> JInt { crate::events::world_contact_force_event_count(jlong_to_const::<WorldHandle>(world)) as JInt });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldGetContactForceEvent(world: JLong, index: JInt, out_event: JLong) -> JLong {
    let event = crate::events::world_get_contact_force_event(jlong_to_const::<WorldHandle>(world), index as u32);
    if let Some(out) = unsafe { jlong_to_slice_mut::<ContactForceEventRecord>(out_event).as_mut() } { *out = event; }
    event.collider1 as JLong
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldSetContactPairFilterCallback(world: JLong, callback: JLong, user_data: JLong) {
    if callback != 0 {
        let callback: ContactPairFilterCallback = unsafe { std::mem::transmute(callback as usize) };
        crate::events::world_set_contact_pair_filter_callback(jlong_to_mut::<WorldHandle>(world), callback, user_data as usize);
    }
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldSetIntersectionPairFilterCallback(world: JLong, callback: JLong, user_data: JLong) {
    if callback != 0 {
        let callback: IntersectionPairFilterCallback = unsafe { std::mem::transmute(callback as usize) };
        crate::events::world_set_intersection_pair_filter_callback(jlong_to_mut::<WorldHandle>(world), callback, user_data as usize);
    }
});
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldClearContactPairFilterCallback(world: JLong) { crate::events::world_clear_contact_pair_filter_callback(jlong_to_mut::<WorldHandle>(world)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_worldClearIntersectionPairFilterCallback(world: JLong) { crate::events::world_clear_intersection_pair_filter_callback(jlong_to_mut::<WorldHandle>(world)); });

jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rtreeCreate() -> JLong { ptr_to_jlong(crate::rtree::rtree_create()) });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rtreeDestroy(tree: JLong) { crate::rtree::rtree_destroy(jlong_to_mut::<RTreeHandle>(tree)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rtreeClear(tree: JLong) { crate::rtree::rtree_clear(jlong_to_mut::<RTreeHandle>(tree)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rtreeLen(tree: JLong) -> JInt { crate::rtree::rtree_len(jlong_to_const::<RTreeHandle>(tree)) as JInt });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rtreeInsert(tree: JLong, id: JLong, min_x: JDouble, min_y: JDouble, min_z: JDouble, max_x: JDouble, max_y: JDouble, max_z: JDouble) -> JByte { crate::rtree::rtree_insert(jlong_to_mut::<RTreeHandle>(tree), id as u64, aabb(min_x,min_y,min_z,max_x,max_y,max_z)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rtreeUpdate(tree: JLong, id: JLong, min_x: JDouble, min_y: JDouble, min_z: JDouble, max_x: JDouble, max_y: JDouble, max_z: JDouble) -> JByte { crate::rtree::rtree_update(jlong_to_mut::<RTreeHandle>(tree), id as u64, aabb(min_x,min_y,min_z,max_x,max_y,max_z)).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rtreeRemove(tree: JLong, id: JLong) -> JByte { crate::rtree::rtree_remove(jlong_to_mut::<RTreeHandle>(tree), id as u64).0 as JByte });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rtreeRebuild(tree: JLong) { crate::rtree::rtree_rebuild(jlong_to_mut::<RTreeHandle>(tree)); });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rtreeQueryAabbCount(tree: JLong, min_x: JDouble, min_y: JDouble, min_z: JDouble, max_x: JDouble, max_y: JDouble, max_z: JDouble) -> JInt { crate::rtree::rtree_query_aabb_count(jlong_to_mut::<RTreeHandle>(tree), aabb(min_x,min_y,min_z,max_x,max_y,max_z)) as JInt });
jni!(Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_rtreeQueryAabb(tree: JLong, min_x: JDouble, min_y: JDouble, min_z: JDouble, max_x: JDouble, max_y: JDouble, max_z: JDouble, out_ids: JLong, capacity: JInt) -> JInt { crate::rtree::rtree_query_aabb(jlong_to_mut::<RTreeHandle>(tree), aabb(min_x,min_y,min_z,max_x,max_y,max_z), jlong_to_slice_mut::<u64>(out_ids), capacity as u32) as JInt });
