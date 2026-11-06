use anvilkit::core::math::Transform;
use anvilkit::ecs::physics as ak_physics;
use anvilkit::ecs::prelude::*;
use dashmap::DashMap;
use rapier3d::math::Vector;
use crate::rapier::soft_data::SoftBodyId;
use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder, RigidBodyType};

use crate::rapier::aerodynamics;
use crate::rapier::error::ffi_guard;
use crate::rapier::ffi::{
    AeroForceReport, AeroSurface, Bool, ColliderHandleRaw, FluidForceReport, FluidVolume,
    HertzContactReport, ImpulseJointHandleRaw, MaterialProperties, Quat, RigidBodyHandleRaw,
    ShapeDesc, StressStrainReport, TrajectoryEnvironment, TrajectoryForceReport, Vec3, WorldHandle,
    pack_collider_handle, pack_rigid_body_handle, quat_finite, unpack_rigid_body_handle,
    vec3_finite,
};
use crate::rapier::fluid;
use crate::rapier::soft_body::soft_body_build_tetra_mesh;
use crate::rapier::trajectory;

const EPSILON: f64 = 1.0e-12;

pub(crate) struct AnvilKitAppState {
    app: anvilkit::ecs::app::App,
    entity_to_body: DashMap<Entity, RigidBodyHandleRaw>,
    entity_to_collider: DashMap<Entity, ColliderHandleRaw>,
    entity_to_soft_body: DashMap<Entity, SoftBodyId>,
    constraint_to_joint: DashMap<u64, ImpulseJointHandleRaw>,
    next_constraint_id: u64,
}

#[derive(Component, Clone, Copy)]
#[allow(dead_code)] // `handle` is written when the component is inserted; read path lands with the reverse-lookup API.
struct BodyLink {
    handle: RigidBodyHandleRaw,
}

#[derive(Component, Clone, Copy)]
#[allow(dead_code)]
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
            entity_to_body: DashMap::new(),
            entity_to_collider: DashMap::new(),
            entity_to_soft_body: DashMap::new(),
            constraint_to_joint: DashMap::new(),
            next_constraint_id: 1,
        }
    }

    fn entity_from_bits(&self, entity_bits: u64) -> Option<Entity> {
        Entity::try_from_bits(entity_bits)
            .ok()
            .filter(|entity| self.app.world.entities().contains(*entity))
    }

    /// Build a soft body bound to an anvilkit `Entity` and register the
    /// `entity → SoftBodyId` link. The soft body is created as a **bounding-box
    /// tetrahedral mesh**: eight corner particles at the entity's world AABB
    /// corners plus six tetrahedra spanning the box. After switching to the XPBD
    /// solver (via [`soft_body_build_tetra_mesh`]), the box behaves like a
    /// compressible "blob" creature — reuse of the Phase 3 volume constraint.
    ///
    /// Returns the `SoftBodyId` (as `u32`) on success, or `0` on error.
    fn spawn_soft_body(
        &mut self,
        world: &mut WorldHandle,
        entity_bits: u64,
        particle_mass: f64,
        stiffness: f64,
        damping: f64,
        pin: bool,
    ) -> u32 {
        let Some(entity) = self.entity_from_bits(entity_bits) else {
            return 0;
        };
        let Some(transform) = self.app.world.get::<Transform>(entity) else {
            return 0;
        };
        let pos = transform.translation;
        // Entity center in our f64 world units (anvilkit uses f32 translations).
        let center = rapier3d::math::Vector::new(pos.x as f64, pos.y as f64, pos.z as f64);

        // Use a unit-ish half-extent so the blob has a real volume; the caller's
        // `particle_mass` sets the per-corner mass and `stiffness`/`damping` are
        // forwarded as XPBD compliance/iterations via the shared builder.
        let h = 0.5f64;
        let corners: [rapier3d::math::Vector; 8] = [
            center + Vector::new(-h, -h, -h),
            center + Vector::new(h, -h, -h),
            center + Vector::new(-h, h, -h),
            center + Vector::new(h, h, -h),
            center + Vector::new(-h, -h, h),
            center + Vector::new(h, -h, h),
            center + Vector::new(-h, h, h),
            center + Vector::new(h, h, h),
        ];
        // 6 tetrahedra decomposing the cube (classic 5/6-tet split, here 6).
        let tets: [u32; 24] = [
            0, 2, 3, 7, // bottom-back
            0, 3, 1, 7, // bottom-front
            0, 1, 5, 7, // front
            0, 5, 4, 7, // top-front
            0, 4, 6, 7, // top-back
            0, 6, 2, 7, // back
        ];

        // `stiffness`/`damping` are repurposed as XPBD compliance / iterations so
        // the caller keeps the same 7-arg ABI while still tuning the blob.
        let compliance = if stiffness.is_finite() && stiffness >= 0.0 {
            stiffness
        } else {
            0.0
        };
        let iterations = if damping.is_finite() && damping >= 1.0 {
            damping as u32
        } else {
            20
        };
        let _ = pin; // bounding-box blob has no pinned corner by default; reserved.

        // Delegate to the shared tetra-mesh builder (same helper mps-core FFI uses).
        let particles: Vec<Vec3> = corners
            .iter()
            .map(|c| Vec3 {
                x: c.x,
                y: c.y,
                z: c.z,
            })
            .collect();
        let g = world.inner.gravity;
        let id = soft_body_build_tetra_mesh(
            world,
            Vec3 {
                x: g.x,
                y: g.y,
                z: g.z,
            },
            particles.as_ptr(),
            particles.len() as u32,
            tets.as_ptr(),
            (tets.len() / 4) as u32,
            particle_mass,
            compliance,
            iterations,
        );
        if id == u32::MAX {
            return 0;
        }
        let sid = SoftBodyId(id);
        // If the entity should be pinned, pin the center-most corner (corner 0).
        #[allow(clippy::collapsible_if)]
        if pin {
            if let Some(body) = world.inner.soft_bodies.get_mut(sid) {
                // Re-pin by zeroing the first particle's inverse mass.
                if let Some(p) = body.particles.first_mut() {
                    p.inv_mass = 0.0;
                }
            }
        }
        self.entity_to_soft_body.insert(entity, sid);
        sid.0
    }

    fn entity_to_soft_body_id(&self, entity_bits: u64) -> u32 {
        let Some(entity) = self.entity_from_bits(entity_bits) else {
            return 0;
        };
        self.entity_to_soft_body
            .get(&entity)
            .map(|v| v.0)
            .unwrap_or(0)
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
            let body_handle = if let Some(handle) = self.entity_to_body.get(&entity).map(|r| *r) {
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

/// Creates a new AnvilKit app state and returns an opaque handle to it.
///
/// # Safety
///
/// Takes no pointers and cannot fail on input; the returned handle is owned by
/// the caller and must eventually be passed to `anvilkit_app_destroy` (or
/// leaked).
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_create() -> *mut crate::rapier::ffi::AnvilKitAppHandle {
    ffi_guard(std::ptr::null_mut(), || {
        Box::into_raw(Box::new(crate::rapier::ffi::AnvilKitAppHandle {
            inner: AnvilKitAppState::new(),
        }))
    })
}

/// # Safety
///
/// `app` must be null or a handle returned by `anvilkit_app_create` that has
/// not been destroyed yet; ownership transfers back to Rust and the handle is
/// invalid after this call.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_destroy(app: *mut crate::rapier::ffi::AnvilKitAppHandle) {
    ffi_guard((), || {
        if app.is_null() {
            return;
        }
        unsafe {
            drop(Box::from_raw(app));
        }
    })
}

/// # Safety
///
/// `app` must be null or a valid handle returned by `anvilkit_app_create`.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_update(app: *mut crate::rapier::ffi::AnvilKitAppHandle) {
    ffi_guard((), || {
        let Some(app) = (unsafe { app.as_mut() }) else {
            return;
        };
        app.inner.app.update();
    })
}

/// # Safety
///
/// `app` must be null or a valid handle returned by `anvilkit_app_create`.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_spawn_body(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    translation: Vec3,
    rotation: Quat,
    status: u32,
) -> u64 {
    ffi_guard(0, || {
        let Some(app) = (unsafe { app.as_mut() }) else {
            return 0;
        };
        app.inner.spawn_body(translation, rotation, status)
    })
}

/// # Safety
///
/// `app` must be null or a valid handle returned by `anvilkit_app_create`.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_spawn_body_with_collider(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    translation: Vec3,
    rotation: Quat,
    status: u32,
    shape: ShapeDesc,
) -> u64 {
    ffi_guard(0, || {
        let Some(app) = (unsafe { app.as_mut() }) else {
            return 0;
        };
        app.inner
            .spawn_body_with_collider(translation, rotation, status, shape)
    })
}

/// # Safety
///
/// `app` must be null or a valid handle returned by `anvilkit_app_create`.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_set_transform(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    entity_bits: u64,
    translation: Vec3,
    rotation: Quat,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(app) = (unsafe { app.as_mut() }) else {
            return Bool::FALSE;
        };
        app.inner.set_transform(entity_bits, translation, rotation)
    })
}

/// # Safety
///
/// `app` must be null or a valid handle returned by `anvilkit_app_create`.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_set_material(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    entity_bits: u64,
    material: MaterialProperties,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(app) = (unsafe { app.as_mut() }) else {
            return Bool::FALSE;
        };
        app.inner.set_material(entity_bits, material)
    })
}

/// # Safety
///
/// `app` and `world` must be null or valid handles returned by
/// `anvilkit_app_create` / the world-creation ABI.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_sync_to_world(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    world: *mut WorldHandle,
) -> u32 {
    ffi_guard(0, || {
        let Some(app) = (unsafe { app.as_mut() }) else {
            return 0;
        };
        let Some(world) = (unsafe { world.as_mut() }) else {
            return 0;
        };
        app.inner.sync_to_world(world)
    })
}

/// # Safety
///
/// `app` must be null or a valid handle returned by `anvilkit_app_create`.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_entity_to_body(
    app: *const crate::rapier::ffi::AnvilKitAppHandle,
    entity_bits: u64,
) -> RigidBodyHandleRaw {
    ffi_guard(0, || {
        let Some(app) = (unsafe { app.as_ref() }) else {
            return 0;
        };
        let Ok(entity) = Entity::try_from_bits(entity_bits) else {
            return 0;
        };
        app.inner
            .entity_to_body
            .get(&entity)
            .map(|v| *v)
            .unwrap_or(0)
    })
}

/// # Safety
///
/// `app` must be null or a valid handle returned by `anvilkit_app_create`.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_entity_to_collider(
    app: *const crate::rapier::ffi::AnvilKitAppHandle,
    entity_bits: u64,
) -> ColliderHandleRaw {
    ffi_guard(0, || {
        let Some(app) = (unsafe { app.as_ref() }) else {
            return 0;
        };
        let Ok(entity) = Entity::try_from_bits(entity_bits) else {
            return 0;
        };
        app.inner
            .entity_to_collider
            .get(&entity)
            .map(|v| *v)
            .unwrap_or(0)
    })
}

/// # Safety
///
/// `app` and `world` must be null or valid handles returned by
/// `anvilkit_app_create` / the world-creation ABI.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_spawn_soft_body(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    world: *mut WorldHandle,
    entity_bits: u64,
    particle_mass: f64,
    stiffness: f64,
    damping: f64,
    pin: Bool,
) -> u32 {
    ffi_guard(0, || {
        let Some(app) = (unsafe { app.as_mut() }) else {
            return 0;
        };
        let Some(world) = (unsafe { world.as_mut() }) else {
            return 0;
        };
        let pin = pin == Bool::TRUE;
        app.inner
            .spawn_soft_body(world, entity_bits, particle_mass, stiffness, damping, pin)
    })
}

/// # Safety
///
/// `app` must be null or a valid handle returned by `anvilkit_app_create`.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_entity_to_soft_body(
    app: *const crate::rapier::ffi::AnvilKitAppHandle,
    entity_bits: u64,
) -> u32 {
    ffi_guard(0, || {
        let Some(app) = (unsafe { app.as_ref() }) else {
            return 0;
        };
        app.inner.entity_to_soft_body_id(entity_bits)
    })
}

/// # Safety
///
/// `app` and `world` must be null or valid handles returned by
/// `anvilkit_app_create` / the world-creation ABI.
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
    ffi_guard(0, || {
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
        let Some(body1) = app.inner.entity_to_body.get(&entity1).map(|v| *v) else {
            return 0;
        };
        let Some(body2) = app.inner.entity_to_body.get(&entity2).map(|v| *v) else {
            return 0;
        };

        let builder =
            crate::rapier::joints::joint_builder_create(joint_type, axis_or_primary, b, c);
        if builder.is_null() {
            return 0;
        }
        let handle = crate::rapier::joints::world_insert_impulse_joint(
            world, body1, body2, builder, wake_up,
        );
        crate::rapier::joints::joint_builder_destroy(builder);
        if handle == 0 {
            return 0;
        }

        let id = app.inner.next_constraint_id;
        app.inner.next_constraint_id = app.inner.next_constraint_id.saturating_add(1);
        app.inner.constraint_to_joint.insert(id, handle);
        id
    })
}

/// # Safety
///
/// `app` must be null or a valid handle returned by `anvilkit_app_create`.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_constraint_to_joint(
    app: *const crate::rapier::ffi::AnvilKitAppHandle,
    constraint_id: u64,
) -> ImpulseJointHandleRaw {
    ffi_guard(0, || {
        let Some(app) = (unsafe { app.as_ref() }) else {
            return 0;
        };
        app.inner
            .constraint_to_joint
            .get(&constraint_id)
            .map(|v| *v)
            .unwrap_or(0)
    })
}

/// # Safety
///
/// `app` and `world` must be null or valid handles returned by
/// `anvilkit_app_create` / the world-creation ABI.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_remove_constraint(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    world: *mut WorldHandle,
    constraint_id: u64,
    wake_up: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(app) = (unsafe { app.as_mut() }) else {
            return Bool::FALSE;
        };
        let Some(world) = (unsafe { world.as_mut() }) else {
            return Bool::FALSE;
        };
        let Some((_, handle)) = app.inner.constraint_to_joint.remove(&constraint_id) else {
            return Bool::FALSE;
        };
        crate::rapier::joints::world_remove_impulse_joint(world, handle, wake_up)
    })
}

/// # Safety
///
/// `app` and `world` must be null or valid handles. `surfaces` must point to
/// `surface_count` readable `AeroSurface` entries, and `out_report` must be
/// null or point to a valid, writable `AeroForceReport`.
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
    ffi_guard(Bool::FALSE, || {
        let Some(app) = (unsafe { app.as_mut() }) else {
            return Bool::FALSE;
        };
        let Some(world) = (unsafe { world.as_mut() }) else {
            return Bool::FALSE;
        };
        let Ok(entity) = Entity::try_from_bits(entity_bits) else {
            return Bool::FALSE;
        };
        let Some(handle) = app.inner.entity_to_body.get(&entity).map(|v| *v) else {
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
    })
}

/// # Safety
///
/// `app` and `world` must be null or valid handles. `voxels` must point to at
/// least `size_x * size_y * size_z` readable bytes, and `out_report` must be
/// null or point to a valid, writable `AeroForceReport`.
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
    ffi_guard(Bool::FALSE, || {
        let Some(app) = (unsafe { app.as_mut() }) else {
            return Bool::FALSE;
        };
        let Some(world) = (unsafe { world.as_mut() }) else {
            return Bool::FALSE;
        };
        let Ok(entity) = Entity::try_from_bits(entity_bits) else {
            return Bool::FALSE;
        };
        let Some(handle) = app.inner.entity_to_body.get(&entity).map(|v| *v) else {
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
    })
}

/// # Safety
///
/// `app` and `world` must be null or valid handles. `out_report` must be null
/// or point to a valid, writable `FluidForceReport`.
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
    ffi_guard(Bool::FALSE, || {
        let Some(app) = (unsafe { app.as_mut() }) else {
            return Bool::FALSE;
        };
        let Some(world) = (unsafe { world.as_mut() }) else {
            return Bool::FALSE;
        };
        let Ok(entity) = Entity::try_from_bits(entity_bits) else {
            return Bool::FALSE;
        };
        let Some(handle) = app.inner.entity_to_body.get(&entity).map(|v| *v) else {
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
    })
}

/// # Safety
///
/// `app` and `world` must be null or valid handles. `out_report` must be null
/// or point to a valid, writable `TrajectoryForceReport`.
#[unsafe(no_mangle)]
pub extern "C" fn anvilkit_app_apply_trajectory_forces(
    app: *mut crate::rapier::ffi::AnvilKitAppHandle,
    world: *mut WorldHandle,
    entity_bits: u64,
    environment: TrajectoryEnvironment,
    wake_up: Bool,
    out_report: *mut TrajectoryForceReport,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(app) = (unsafe { app.as_mut() }) else {
            return Bool::FALSE;
        };
        let Some(world) = (unsafe { world.as_mut() }) else {
            return Bool::FALSE;
        };
        let Ok(entity) = Entity::try_from_bits(entity_bits) else {
            return Bool::FALSE;
        };
        let Some(handle) = app.inner.entity_to_body.get(&entity).map(|v| *v) else {
            return Bool::FALSE;
        };

        trajectory::trajectory_apply_forces_to_body(world, handle, environment, wake_up, out_report)
    })
}

/// # Safety
///
/// `out_report` must be null or point to a valid, writable `StressStrainReport`.
#[unsafe(no_mangle)]
pub extern "C" fn material_stress_strain_linear(
    material: MaterialProperties,
    strain: f64,
    delta_temperature: f64,
    out_report: *mut StressStrainReport,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
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
    })
}

/// Computes the post-collision relative normal speed from restitution.
///
/// # Safety
///
/// All parameters are passed by value; this function performs no memory
/// access and is always memory-safe. Non-finite inputs or a negative
/// `restitution` yield `NaN`.
#[unsafe(no_mangle)]
pub extern "C" fn material_elastic_collision_relative_speed(
    relative_normal_speed: f64,
    restitution: f64,
) -> f64 {
    ffi_guard(0.0, || {
        if !relative_normal_speed.is_finite() || !restitution.is_finite() || restitution < 0.0 {
            return f64::NAN;
        }
        -restitution * relative_normal_speed
    })
}

/// # Safety
///
/// `out_report` must be null or point to a valid, writable `HertzContactReport`.
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
    ffi_guard(Bool::FALSE, || {
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
    })
}
