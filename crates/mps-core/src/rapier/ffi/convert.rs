use rapier3d::math::{Pose, Rotation, Vector};
use rapier3d::parry::query::ShapeCastOptions;
use rapier3d::parry::shape::SharedShape;
use rapier3d::prelude::{
    ActiveEvents, ActiveHooks, ColliderHandle, Group,
    ImpulseJointHandle as RapierImpulseJointHandle, InteractionGroups, InteractionTestMode,
    JointAxis, QueryFilter, QueryFilterFlags, RigidBodyHandle,
};

use super::types::{
    BodyStatus, ColliderHandleRaw, ImpulseJointHandleRaw, InteractionGroupsDesc, JointAxisDesc,
    JointTypeDesc, KdopPreset, NeuralActivation, Quat, QueryFilterDesc, RigidBodyHandleRaw,
    ShapeCastOptionsDesc, ShapeDesc, ShapeType, Vec3, VoxelColliderMode,
};

pub(crate) const MAX_OUTPUT_CAPACITY: u32 = 1_000_000;
pub(crate) const MAX_TREE_ENTRIES: usize = 1_000_000;

const INVALID_HANDLE_RAW: u64 = u64::MAX;

fn pack_handle_parts(id: u32, generation: u32) -> u64 {
    (((generation as u64) << 32) | (id as u64)).wrapping_add(1)
}

fn unpack_handle_parts(handle: u64) -> (u32, u32) {
    let raw = handle.checked_sub(1).unwrap_or(INVALID_HANDLE_RAW);
    ((raw & 0xffff_ffff) as u32, (raw >> 32) as u32)
}

pub(crate) fn vec3_to_rapier(value: Vec3) -> Vector {
    Vector::new(value.x, value.y, value.z)
}

pub(crate) fn vec3_finite(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

pub(crate) fn vec3_from_rapier(value: Vector) -> Vec3 {
    Vec3 {
        x: value.x,
        y: value.y,
        z: value.z,
    }
}

/// Returns true when `value` is finite and >= 0.
#[inline]
pub(crate) fn finite_non_negative(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

/// Returns true when `value` is finite and > 0.
#[inline]
pub(crate) fn finite_positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

/// Clamp `value` to the closed interval [0, 1].
#[inline]
pub(crate) fn clamp01(value: f64) -> f64 {
    value.clamp(0.0, 1.0)
}

pub(crate) fn quat_to_rapier(value: Quat) -> Rotation {
    Rotation::from_xyzw(value.i, value.j, value.k, value.w)
}

pub(crate) fn quat_finite(value: Quat) -> bool {
    value.i.is_finite() && value.j.is_finite() && value.k.is_finite() && value.w.is_finite()
}

pub(crate) fn quat_from_rapier(value: Rotation) -> Quat {
    Quat {
        i: value.x,
        j: value.y,
        k: value.z,
        w: value.w,
    }
}

pub(crate) fn isometry_from_parts(translation: Vec3, rotation: Quat) -> Pose {
    Pose::from_parts(vec3_to_rapier(translation), quat_to_rapier(rotation))
}

pub(crate) fn pack_rigid_body_handle(handle: RigidBodyHandle) -> RigidBodyHandleRaw {
    let (id, generation) = handle.into_raw_parts();
    pack_handle_parts(id, generation)
}

pub(crate) fn unpack_rigid_body_handle(handle: RigidBodyHandleRaw) -> RigidBodyHandle {
    let (id, generation) = unpack_handle_parts(handle);
    RigidBodyHandle::from_raw_parts(id, generation)
}

pub(crate) fn pack_collider_handle(handle: ColliderHandle) -> ColliderHandleRaw {
    let (id, generation) = handle.into_raw_parts();
    pack_handle_parts(id, generation)
}

pub(crate) fn unpack_collider_handle(handle: ColliderHandleRaw) -> ColliderHandle {
    let (id, generation) = unpack_handle_parts(handle);
    ColliderHandle::from_raw_parts(id, generation)
}

pub(crate) fn pack_impulse_joint_handle(handle: RapierImpulseJointHandle) -> ImpulseJointHandleRaw {
    let (id, generation) = handle.into_raw_parts();
    pack_handle_parts(id, generation)
}

pub(crate) fn unpack_impulse_joint_handle(
    handle: ImpulseJointHandleRaw,
) -> RapierImpulseJointHandle {
    let (id, generation) = unpack_handle_parts(handle);
    RapierImpulseJointHandle::from_raw_parts(id, generation)
}

pub(crate) fn body_status_to_rapier(status: BodyStatus) -> rapier3d::prelude::RigidBodyType {
    match status {
        BodyStatus::Dynamic => rapier3d::prelude::RigidBodyType::Dynamic,
        BodyStatus::Fixed => rapier3d::prelude::RigidBodyType::Fixed,
        BodyStatus::KinematicPositionBased => {
            rapier3d::prelude::RigidBodyType::KinematicPositionBased
        }
        BodyStatus::KinematicVelocityBased => {
            rapier3d::prelude::RigidBodyType::KinematicVelocityBased
        }
    }
}

pub(crate) fn body_status_from_raw(value: u32) -> BodyStatus {
    match value {
        0 => BodyStatus::Dynamic,
        1 => BodyStatus::Fixed,
        2 => BodyStatus::KinematicPositionBased,
        3 => BodyStatus::KinematicVelocityBased,
        _ => BodyStatus::Fixed,
    }
}

pub(crate) fn body_status_from_rapier(status: rapier3d::prelude::RigidBodyType) -> BodyStatus {
    match status {
        rapier3d::prelude::RigidBodyType::Dynamic => BodyStatus::Dynamic,
        rapier3d::prelude::RigidBodyType::Fixed => BodyStatus::Fixed,
        rapier3d::prelude::RigidBodyType::KinematicPositionBased => {
            BodyStatus::KinematicPositionBased
        }
        rapier3d::prelude::RigidBodyType::KinematicVelocityBased => {
            BodyStatus::KinematicVelocityBased
        }
    }
}

pub(crate) fn body_status_to_raw(status: BodyStatus) -> u32 {
    status as u32
}

pub(crate) fn shape_type_from_raw(value: u32) -> ShapeType {
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

pub(crate) fn shape_from_desc(desc: ShapeDesc) -> SharedShape {
    match shape_type_from_raw(desc.shape_type) {
        ShapeType::Ball => SharedShape::ball(desc.a),
        ShapeType::Cuboid => SharedShape::cuboid(desc.a, desc.b, desc.c),
        ShapeType::CapsuleY => SharedShape::capsule_y(desc.a, desc.b),
        ShapeType::CapsuleX => SharedShape::capsule_x(desc.a, desc.b),
        ShapeType::CapsuleZ => SharedShape::capsule_z(desc.a, desc.b),
        ShapeType::Cylinder => SharedShape::cylinder(desc.a, desc.b),
        ShapeType::RoundCylinder => SharedShape::round_cylinder(desc.a, desc.b, desc.c),
        ShapeType::Cone => SharedShape::cone(desc.a, desc.b),
        ShapeType::RoundCone => SharedShape::round_cone(desc.a, desc.b, desc.c),
        ShapeType::RoundCuboid => SharedShape::round_cuboid(desc.a, desc.b, desc.c, desc.d),
    }
}

pub(crate) fn shape_desc_valid(desc: ShapeDesc) -> bool {
    if !desc.a.is_finite() || !desc.b.is_finite() || !desc.c.is_finite() || !desc.d.is_finite() {
        return false;
    }

    match shape_type_from_raw(desc.shape_type) {
        ShapeType::Ball => desc.a > 0.0,
        ShapeType::Cuboid => desc.a > 0.0 && desc.b > 0.0 && desc.c > 0.0,
        ShapeType::CapsuleY | ShapeType::CapsuleX | ShapeType::CapsuleZ => {
            desc.a > 0.0 && desc.b > 0.0
        }
        ShapeType::Cylinder | ShapeType::Cone => desc.a > 0.0 && desc.b > 0.0,
        ShapeType::RoundCylinder | ShapeType::RoundCone => {
            desc.a > 0.0 && desc.b > 0.0 && desc.c >= 0.0
        }
        ShapeType::RoundCuboid => desc.a > 0.0 && desc.b > 0.0 && desc.c > 0.0 && desc.d >= 0.0,
    }
}

pub(crate) fn voxel_collider_mode_from_raw(value: u32) -> VoxelColliderMode {
    match value {
        1 => VoxelColliderMode::Cuboids,
        2 => VoxelColliderMode::GreedyCuboids,
        3 => VoxelColliderMode::SurfaceMesh,
        _ => VoxelColliderMode::Auto,
    }
}

pub(crate) fn neural_activation_from_raw(value: u32) -> NeuralActivation {
    match value {
        1 => NeuralActivation::Tanh,
        2 => NeuralActivation::Sin,
        3 => NeuralActivation::Linear,
        _ => NeuralActivation::Relu,
    }
}

pub(crate) fn kdop_preset_from_raw(value: u32) -> KdopPreset {
    match value {
        14 => KdopPreset::K14,
        18 => KdopPreset::K18,
        26 => KdopPreset::K26,
        _ => KdopPreset::K6,
    }
}

pub(crate) fn joint_type_from_raw(value: u32) -> JointTypeDesc {
    match value {
        1 => JointTypeDesc::Revolute,
        2 => JointTypeDesc::Prismatic,
        3 => JointTypeDesc::Rope,
        4 => JointTypeDesc::Spring,
        5 => JointTypeDesc::Spherical,
        _ => JointTypeDesc::Fixed,
    }
}

pub(crate) fn interaction_groups_to_rapier(groups: InteractionGroupsDesc) -> InteractionGroups {
    InteractionGroups::new(
        Group::from_bits_truncate(groups.memberships),
        Group::from_bits_truncate(groups.filter),
        InteractionTestMode::And,
    )
}

pub(crate) fn active_events_from_bits(bits: u32) -> ActiveEvents {
    ActiveEvents::from_bits_truncate(bits)
}

pub(crate) fn active_hooks_from_bits(bits: u32) -> ActiveHooks {
    ActiveHooks::from_bits_truncate(bits)
}

pub(crate) fn query_filter_from_desc(desc: QueryFilterDesc) -> QueryFilter<'static> {
    let mut filter = QueryFilter::from(QueryFilterFlags::from_bits_truncate(desc.flags));

    if desc.use_groups.0 != 0 {
        filter = filter.groups(interaction_groups_to_rapier(desc.groups));
    }
    if desc.use_exclude_collider.0 != 0 {
        filter = filter.exclude_collider(unpack_collider_handle(desc.exclude_collider));
    }
    if desc.use_exclude_rigid_body.0 != 0 {
        filter = filter.exclude_rigid_body(unpack_rigid_body_handle(desc.exclude_rigid_body));
    }

    filter
}

pub(crate) fn shape_cast_options_to_rapier(options: ShapeCastOptionsDesc) -> ShapeCastOptions {
    ShapeCastOptions {
        max_time_of_impact: options.max_time_of_impact,
        target_distance: options.target_distance,
        stop_at_penetration: options.stop_at_penetration.0 != 0,
        compute_impact_geometry_on_penetration: options.compute_impact_geometry_on_penetration.0
            != 0,
    }
}

pub(crate) fn joint_axis_to_rapier(axis: JointAxisDesc) -> JointAxis {
    match axis {
        JointAxisDesc::LinX => JointAxis::LinX,
        JointAxisDesc::LinY => JointAxis::LinY,
        JointAxisDesc::LinZ => JointAxis::LinZ,
        JointAxisDesc::AngX => JointAxis::AngX,
        JointAxisDesc::AngY => JointAxis::AngY,
        JointAxisDesc::AngZ => JointAxis::AngZ,
    }
}

pub(crate) fn joint_axis_from_raw(value: u32) -> JointAxisDesc {
    match value {
        1 => JointAxisDesc::LinY,
        2 => JointAxisDesc::LinZ,
        3 => JointAxisDesc::AngX,
        4 => JointAxisDesc::AngY,
        5 => JointAxisDesc::AngZ,
        _ => JointAxisDesc::LinX,
    }
}
