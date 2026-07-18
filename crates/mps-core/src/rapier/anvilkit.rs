use anvilkit::core::math::Transform;
use anvilkit::ecs::physics as ak_physics;
use anvilkit::ecs::prelude::*;
use hashbrown::HashMap;
use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder, RigidBodyType};

use crate::rapier::aerodynamics;
use crate::rapier::ffi::{
    AeroForceReport, AeroSurface, Bool, ColliderHandleRaw, FluidForceReport, FluidVolume,
    HertzContactReport, ImpulseJointHandleRaw, MaterialProperties, Quat, RigidBodyHandleRaw,
    ShapeDesc, StressStrainReport, TrajectoryEnvironment, TrajectoryForceReport, Vec3, WorldHandle,
    pack_collider_handle, pack_rigid_body_handle, quat_finite, unpack_rigid_body_handle,
    vec3_finite,
};
use crate::rapier::fluid;
use crate::rapier::trajectory;

const EPSILON: f64 = 1.0e-12;

pub(crate) struct AnvilKitAppState {
    app: anvilkit::ecs::app::App,
    entity_to_body: HashMap<Entity, RigidBodyHandleRaw>,
    entity_to_collider: HashMap<Entity, ColliderHandleRaw>,
    constraint_to_joint: HashMap<u64, ImpulseJointHandleRaw>,
    next_constraint_id: u64,
}

#[derive(Component, Clone, Copy)]
struct BodyLink {
    handle: RigidBodyHandleRaw,
}

#[derive(Component, Clone, Copy)]
struct ColliderLink {
    handle: ColliderHandleRaw,
}

#[derive(Component, Clone, Copy)]
struct PendingCollider {
    shape: ShapeDesc,
}

#[derive(Component, Clone, Copy)]
struct PendingMaterial {
    material: MaterialProperties,
}

fn transform_from_parts(translation: Vec3, rotation: Quat) -> Transform {
    Transform::from_xyz(
        translation.x as f32,
        translation.y as f32,
        translation.z as f32,
    )
    .with_rotation(anvilkit::core::Quat::from_xyzw(
        rotation.i as f32,
        rotation.j as f32,
        rotation.k as f32,
        rotation.w as f32,
    ))
}

fn vec3_from_glam(value: anvilkit::core::Vec3) -> Vec3 {
    Vec3 {
        x: value.x as f64,
        y: value.y as f64,
        z: value.z as f64,
    }
}

fn quat_from_glam(value: anvilkit::core::Quat) -> Quat {
    Quat {
        i: value.x as f64,
        j: value.y as f64,
        k: value.z as f64,
        w: value.w as f64,
    }
}

fn body_type_from_raw(status: u32) -> RigidBodyType {
    match status {
        0 => RigidBodyType::Dynamic,
        2 => RigidBodyType::KinematicPositionBased,
        3 => RigidBodyType::KinematicVelocityBased,
        _ => RigidBodyType::Fixed,
    }
}

fn ak_body_type_from_raw(status: u32) -> ak_physics::RigidBodyType {
    match status {
        0 => ak_physics::RigidBodyType::Dynamic,
        2 | 3 => ak_physics::RigidBodyType::Kinematic,
        _ => ak_physics::RigidBodyType::Fixed,
    }
}

fn shape_builder(shape: ShapeDesc) -> Option<ColliderBuilder> {
    if !crate::rapier::ffi::shape_desc_valid(shape) {
        return None;
    }
    Some(ColliderBuilder::new(crate::rapier::ffi::shape_from_desc(
        shape,
    )))
}

fn material_valid(material: MaterialProperties) -> bool {
    material.density.is_finite()
        && material.friction.is_finite()
        && material.restitution.is_finite()
        && material.youngs_modulus.is_finite()
        && material.poisson_ratio.is_finite()
        && material.thermal_expansion.is_finite()
        && material.density >= 0.0
        && material.friction >= 0.0
        && material.restitution >= 0.0
        && material.youngs_modulus >= 0.0
        && material.poisson_ratio > -1.0
        && material.poisson_ratio < 0.5
}

fn apply_material_to_builder(
    builder: ColliderBuilder,
    material: MaterialProperties,
) -> ColliderBuilder {
    builder
        .density(material.density)
        .friction(material.friction)
        .restitution(material.restitution)
}

impl AnvilKitAppState {
    fn new() -> Self {
        let mut app = anvilkit::ecs::app::App::new();
        app.add_plugins(anvilkit::ecs::plugin::AnvilKitEcsPlugin);
        Self {
            app,
            entity_to_body: HashMap::new(),
            entity_to_collider: HashMap::new(),
            constraint_to_joint: HashMap::new(),
            next_constraint_id: 1,
        }
    }

    fn entity_from_bits(&self, entity_bits: u64) -> Option<Entity> {
        Entity::try_from_bits(entity_bits)
            .ok()
            .filter(|entity| self.app.world.entities().contains(*entity))
    }

    fn spawn_body(&mut self, translation: Vec3, rotation: Quat, status: u32) -> u64 {
        if !vec3_finite(translation) || !quat_finite(rotation) {
            return 0;
        }
        let entity = self
            .app
            .world
            .spawn((
                transform_from_parts(translation, rotation),
                ak_physics::RigidBody::new(ak_body_type_from_raw(status)),
            ))
            .id();
        entity.to_bits()
    }

    fn spawn_body_with_collider(
        &mut self,
        translation: Vec3,
        rotation: Quat,
        status: u32,
        shape: ShapeDesc,
    ) -> u64 {
        if shape_builder(shape).is_none() {
            return 0;
        }
        let entity_bits = self.spawn_body(translation, rotation, status);
        if entity_bits == 0 {
            return 0;
        }
        let Some(entity) = self.entity_from_bits(entity_bits) else {
            return 0;
        };
        self.app
            .world
            .entity_mut(entity)
            .insert(PendingCollider { shape });
        entity_bits
    }

    fn set_transform(&mut self, entity_bits: u64, translation: Vec3, rotation: Quat) -> Bool {
        if !vec3_finite(translation) || !quat_finite(rotation) {
            return Bool::FALSE;
        }
        let Some(entity) = self.entity_from_bits(entity_bits) else {
            return Bool::FALSE;
        };
        let Some(mut transform) = self.app.world.get_mut::<Transform>(entity) else {
            return Bool::FALSE;
        };
        *transform = transform_from_parts(translation, rotation);
        Bool::TRUE
    }

    fn set_material(&mut self, entity_bits: u64, material: MaterialProperties) -> Bool {
        if !material_valid(material) {
            return Bool::FALSE;
        }
        let Some(entity) = self.entity_from_bits(entity_bits) else {
            return Bool::FALSE;
        };
        self.app
            .world
            .entity_mut(entity)
            .insert(PendingMaterial { material });
        Bool::TRUE
    }

    fn sync_to_world(&mut self, world: &mut WorldHandle) -> u32 {
        let entities: Vec<_> = self
            .app
            .world
            .query::<(
                Entity,
                &Transform,
                &ak_physics::RigidBody,
                Option<&PendingCollider>,
                Option<&PendingMaterial>,
            )>()
            .iter(&self.app.world)
            .map(|(entity, transform, body, collider, material)| {
                (
                    entity,
                    *transform,
                    body.body_type,
                    collider.copied(),
                    material.copied(),
                )
            })
            .collect();

        let mut synced = 0u32;
        for (entity, transform, body_type, pending_collider, pending_material) in entities {
            let translation = vec3_from_glam(transform.translation);
            let rotation = quat_from_glam(transform.rotation);
            let body_handle = if let Some(handle) = self.entity_to_body.get(&entity).copied() {
                handle
            } else {
                let body = RigidBodyBuilder::new(body_type_from_raw(match body_type {
                    ak_physics::RigidBodyType::Dynamic => 0,
                    ak_physics::RigidBodyType::Fixed => 1,
                    ak_physics::RigidBodyType::Kinematic => 2,
                }))
                .pose(crate::rapier::ffi::isometry_from_parts(
                    translation,
                    rotation,
                ))
                .build();
                let packed = pack_rigid_body_handle(world.inner.bodies.insert(body));
                self.entity_to_body.insert(entity, packed);
                self.app
                    .world
                    .entity_mut(entity)
                    .insert(BodyLink { handle: packed });
                packed
            };

            if let Some(body) = world
                .inner
                .bodies
                .get_mut(unpack_rigid_body_handle(body_handle))
            {
                body.set_position(
                    crate::rapier::ffi::isometry_from_parts(translation, rotation),
                    false,
                );
                synced = synced.saturating_add(1);
            }

            if self.entity_to_collider.contains_key(&entity) {
                continue;
            }
            let Some(pending_collider) = pending_collider else {
                continue;
            };
            let Some(builder) = shape_builder(pending_collider.shape) else {
                continue;
            };
            let mut builder = builder;
            if let Some(pending_material) = pending_material {
                builder = apply_material_to_builder(builder, pending_material.material);
            }
            let collider = builder.build();
            let handle = world.inner.colliders.insert_with_parent(
                collider,
                unpack_rigid_body_handle(body_handle),
                &mut world.inner.bodies,
            );
            let packed = pack_collider_handle(handle);
            self.entity_to_collider.insert(entity, packed);
            self.app
                .world
                .entity_mut(entity)
                .insert(ColliderLink { handle: packed });
        }

        synced
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_create() -> *mut crate::rapier::ffi::AnvilKitAppHandle {
    Box::into_raw(Box::new(crate::rapier::ffi::AnvilKitAppHandle {
        inner: AnvilKitAppState::new(),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_destroy(app: *mut crate::rapier::ffi::AnvilKitAppHandle) {
    if app.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(app));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_update(app: *mut crate::rapier::ffi::AnvilKitAppHandle) {
    let Some(app) = (unsafe { app.as_mut() }) else {
        return;
    };
    app.inner.app.update();
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_spawn_body(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    translation: Vec3,
    rotation: Quat,
    status: u32,
) -> u64 {
    let Some(app) = (unsafe { app.as_mut() }) else {
        return 0;
    };
    app.inner.spawn_body(translation, rotation, status)
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_spawn_body_with_collider(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    translation: Vec3,
    rotation: Quat,
    status: u32,
    shape: ShapeDesc,
) -> u64 {
    let Some(app) = (unsafe { app.as_mut() }) else {
        return 0;
    };
    app.inner
        .spawn_body_with_collider(translation, rotation, status, shape)
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_set_transform(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    entity_bits: u64,
    translation: Vec3,
    rotation: Quat,
) -> Bool {
    let Some(app) = (unsafe { app.as_mut() }) else {
        return Bool::FALSE;
    };
    app.inner.set_transform(entity_bits, translation, rotation)
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_set_material(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    entity_bits: u64,
    material: MaterialProperties,
) -> Bool {
    let Some(app) = (unsafe { app.as_mut() }) else {
        return Bool::FALSE;
    };
    app.inner.set_material(entity_bits, material)
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_sync_to_world(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    world: *mut WorldHandle,
) -> u32 {
    let Some(app) = (unsafe { app.as_mut() }) else {
        return 0;
    };
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0;
    };
    app.inner.sync_to_world(world)
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_entity_to_body(
    app: *const crate::rapier::ffi::AnvilKitAppHandle,
    entity_bits: u64,
) -> RigidBodyHandleRaw {
    let Some(app) = (unsafe { app.as_ref() }) else {
        return 0;
    };
    let Ok(entity) = Entity::try_from_bits(entity_bits) else {
        return 0;
    };
    app.inner.entity_to_body.get(&entity).copied().unwrap_or(0)
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_entity_to_collider(
    app: *const crate::rapier::ffi::AnvilKitAppHandle,
    entity_bits: u64,
) -> ColliderHandleRaw {
    let Some(app) = (unsafe { app.as_ref() }) else {
        return 0;
    };
    let Ok(entity) = Entity::try_from_bits(entity_bits) else {
        return 0;
    };
    app.inner
        .entity_to_collider
        .get(&entity)
        .copied()
        .unwrap_or(0)
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_create_constraint(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    world: *mut WorldHandle,
    entity1_bits: u64,
    entity2_bits: u64,
    joint_type: u32,
    axis_or_primary: Vec3,
    b: f64,
    c: f64,
    wake_up: Bool,
) -> u64 {
    let Some(app) = (unsafe { app.as_mut() }) else {
        return 0;
    };
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0;
    };
    let Ok(entity1) = Entity::try_from_bits(entity1_bits) else {
        return 0;
    };
    let Ok(entity2) = Entity::try_from_bits(entity2_bits) else {
        return 0;
    };
    let Some(body1) = app.inner.entity_to_body.get(&entity1).copied() else {
        return 0;
    };
    let Some(body2) = app.inner.entity_to_body.get(&entity2).copied() else {
        return 0;
    };

    let builder = crate::rapier::joints::joint_builder_create(joint_type, axis_or_primary, b, c);
    if builder.is_null() {
        return 0;
    }
    let handle =
        crate::rapier::joints::world_insert_impulse_joint(world, body1, body2, builder, wake_up);
    crate::rapier::joints::joint_builder_destroy(builder);
    if handle == 0 {
        return 0;
    }

    let id = app.inner.next_constraint_id;
    app.inner.next_constraint_id = app.inner.next_constraint_id.saturating_add(1);
    app.inner.constraint_to_joint.insert(id, handle);
    id
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_constraint_to_joint(
    app: *const crate::rapier::ffi::AnvilKitAppHandle,
    constraint_id: u64,
) -> ImpulseJointHandleRaw {
    let Some(app) = (unsafe { app.as_ref() }) else {
        return 0;
    };
    app.inner
        .constraint_to_joint
        .get(&constraint_id)
        .copied()
        .unwrap_or(0)
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_remove_constraint(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    world: *mut WorldHandle,
    constraint_id: u64,
    wake_up: Bool,
) -> Bool {
    let Some(app) = (unsafe { app.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(handle) = app.inner.constraint_to_joint.remove(&constraint_id) else {
        return Bool::FALSE;
    };
    crate::rapier::joints::world_remove_impulse_joint(world, handle, wake_up)
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_apply_aero_surfaces(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    world: *mut WorldHandle,
    entity_bits: u64,
    wind_velocity: Vec3,
    air_density: f64,
    surfaces: *const AeroSurface,
    surface_count: u32,
    wake_up: Bool,
    out_report: *mut AeroForceReport,
) -> Bool {
    let Some(app) = (unsafe { app.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Ok(entity) = Entity::try_from_bits(entity_bits) else {
        return Bool::FALSE;
    };
    let Some(handle) = app.inner.entity_to_body.get(&entity).copied() else {
        return Bool::FALSE;
    };

    aerodynamics::aero_apply_surfaces(
        world,
        handle,
        wind_velocity,
        air_density,
        surfaces,
        surface_count,
        wake_up,
        out_report,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_apply_aero_voxel_grid(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    world: *mut WorldHandle,
    entity_bits: u64,
    wind_velocity: Vec3,
    air_density: f64,
    voxels: *const u8,
    size_x: u32,
    size_y: u32,
    size_z: u32,
    voxel_size: f64,
    local_origin: Vec3,
    drag_coefficient: f64,
    lift_coefficient: f64,
    wake_up: Bool,
    out_report: *mut AeroForceReport,
) -> Bool {
    let Some(app) = (unsafe { app.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Ok(entity) = Entity::try_from_bits(entity_bits) else {
        return Bool::FALSE;
    };
    let Some(handle) = app.inner.entity_to_body.get(&entity).copied() else {
        return Bool::FALSE;
    };

    aerodynamics::aero_apply_voxel_grid(
        world,
        handle,
        wind_velocity,
        air_density,
        voxels,
        size_x,
        size_y,
        size_z,
        voxel_size,
        local_origin,
        drag_coefficient,
        lift_coefficient,
        wake_up,
        out_report,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_apply_fluid_aabb_forces(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    world: *mut WorldHandle,
    entity_bits: u64,
    fluid_volume: FluidVolume,
    body_half_extents: Vec3,
    body_volume: f64,
    wake_up: Bool,
    out_report: *mut FluidForceReport,
) -> Bool {
    let Some(app) = (unsafe { app.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Ok(entity) = Entity::try_from_bits(entity_bits) else {
        return Bool::FALSE;
    };
    let Some(handle) = app.inner.entity_to_body.get(&entity).copied() else {
        return Bool::FALSE;
    };

    fluid::fluid_apply_aabb_forces(
        world,
        handle,
        fluid_volume,
        body_half_extents,
        body_volume,
        wake_up,
        out_report,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_apply_trajectory_forces(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    world: *mut WorldHandle,
    entity_bits: u64,
    environment: TrajectoryEnvironment,
    wake_up: Bool,
    out_report: *mut TrajectoryForceReport,
) -> Bool {
    let Some(app) = (unsafe { app.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(world) = (unsafe { world.as_mut() }) else {
        return Bool::FALSE;
    };
    let Ok(entity) = Entity::try_from_bits(entity_bits) else {
        return Bool::FALSE;
    };
    let Some(handle) = app.inner.entity_to_body.get(&entity).copied() else {
        return Bool::FALSE;
    };

    trajectory::trajectory_apply_forces_to_body(world, handle, environment, wake_up, out_report)
}

#[unsafe(no_mangle)]
pub extern "C" fn material_stress_strain_linear(
    material: MaterialProperties,
    strain: f64,
    delta_temperature: f64,
    out_report: *mut StressStrainReport,
) -> Bool {
    if !material_valid(material) || !strain.is_finite() || !delta_temperature.is_finite() {
        return Bool::FALSE;
    }
    let thermal_strain = material.thermal_expansion * delta_temperature;
    let mechanical_strain = strain - thermal_strain;
    let stress = material.youngs_modulus * mechanical_strain;
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        return Bool::FALSE;
    };
    *out_report = StressStrainReport {
        strain,
        stress,
        elastic_energy_density: 0.5 * stress * mechanical_strain,
        thermal_strain,
    };
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn material_elastic_collision_relative_speed(
    relative_normal_speed: f64,
    restitution: f64,
) -> f64 {
    if !relative_normal_speed.is_finite() || !restitution.is_finite() || restitution < 0.0 {
        return f64::NAN;
    }
    -restitution * relative_normal_speed
}

#[unsafe(no_mangle)]
pub extern "C" fn material_hertz_contact_force(
    material1: MaterialProperties,
    material2: MaterialProperties,
    radius1: f64,
    radius2: f64,
    penetration: f64,
    penetration_rate: f64,
    damping: f64,
    out_report: *mut HertzContactReport,
) -> Bool {
    if !material_valid(material1)
        || !material_valid(material2)
        || !radius1.is_finite()
        || !radius2.is_finite()
        || !penetration.is_finite()
        || !penetration_rate.is_finite()
        || !damping.is_finite()
        || radius1 <= 0.0
        || radius2 <= 0.0
        || penetration < 0.0
        || damping < 0.0
        || material1.youngs_modulus <= 0.0
        || material2.youngs_modulus <= 0.0
    {
        return Bool::FALSE;
    }

    let compliance1 =
        (1.0 - material1.poisson_ratio * material1.poisson_ratio) / material1.youngs_modulus;
    let compliance2 =
        (1.0 - material2.poisson_ratio * material2.poisson_ratio) / material2.youngs_modulus;
    let effective_modulus = 1.0 / (compliance1 + compliance2);
    let effective_radius = 1.0 / (1.0 / radius1 + 1.0 / radius2);
    let contact_radius = (effective_radius * penetration).sqrt();
    let normal_force =
        (4.0 / 3.0) * effective_modulus * effective_radius.sqrt() * penetration.powf(1.5);
    let stiffness = if penetration > EPSILON {
        2.0 * effective_modulus * (effective_radius * penetration).sqrt()
    } else {
        0.0
    };
    let damping_force = damping * penetration_rate.max(0.0);
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        return Bool::FALSE;
    };
    *out_report = HertzContactReport {
        effective_modulus,
        effective_radius,
        contact_radius,
        contact_area: std::f64::consts::PI * contact_radius * contact_radius,
        normal_force,
        stiffness,
        damping_force,
        total_force: normal_force + damping_force,
    };
    Bool::TRUE
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_material() -> MaterialProperties {
        MaterialProperties {
            density: 2.0,
            friction: 0.6,
            restitution: 0.3,
            youngs_modulus: 2.0e11,
            poisson_ratio: 0.3,
            thermal_expansion: 1.2e-5,
        }
    }

    #[test]
    fn material_formulas_work() {
        let material = test_material();
        let mut stress = StressStrainReport::default();
        assert_eq!(
            material_stress_strain_linear(material, 0.001, 10.0, &mut stress),
            Bool::TRUE
        );
        assert!(stress.stress > 0.0);
        assert!(stress.thermal_strain > 0.0);

        let rebound = material_elastic_collision_relative_speed(-5.0, material.restitution);
        assert!(rebound > 0.0);

        let mut hertz = HertzContactReport::default();
        assert_eq!(
            material_hertz_contact_force(
                material, material, 0.5, 0.5, 0.001, 0.2, 10.0, &mut hertz,
            ),
            Bool::TRUE
        );
        assert!(hertz.normal_force > 0.0);
        assert!(hertz.contact_area > 0.0);
        assert!(hertz.total_force > hertz.normal_force);
    }
}
