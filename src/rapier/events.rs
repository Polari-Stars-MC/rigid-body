use parking_lot::{Mutex, RwLock};
use rapier3d::geometry::{CollisionEvent, CollisionEventFlags, ContactPair, SolverFlags};
use rapier3d::prelude::{
    ColliderSet, ContactForceEvent, EventHandler, PhysicsHooks, Real, RigidBodySet, Vector,
};

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ERR_UNSUPPORTED, clear_error, set_error,
};
use crate::rapier::ffi::{
    AirDragLaw, Bool, CollisionEventRecord, ContactForceEventRecord, CoulombFrictionLaw,
    CustomPhysicsReport, ExternalForceLaw, MAX_OUTPUT_CAPACITY, WorldHandle, pack_collider_handle,
    vec3_finite, vec3_from_rapier, vec3_to_rapier,
};
use crate::rapier::math::KahanVec3;

const MAX_EVENT_RECORDS: usize = 16_384;

#[derive(Clone, Debug, Default)]
pub(crate) struct CustomPhysicsState {
    pub(crate) coulomb_friction: Option<CoulombFrictionLaw>,
    pub(crate) air_drag: Option<AirDragLaw>,
    pub(crate) external_force: Option<ExternalForceLaw>,
    pub(crate) last_report: CustomPhysicsReport,
}

#[derive(Default)]
pub(crate) struct CollectingEventHandler {
    collision_events: Mutex<Vec<CollisionEventRecord>>,
    contact_force_events: Mutex<Vec<ContactForceEventRecord>>,
    custom_physics: RwLock<CustomPhysicsState>,
}

impl CollectingEventHandler {
    pub(crate) fn clear(&self) {
        self.collision_events.lock().clear();
        self.contact_force_events.lock().clear();
    }

    pub(crate) fn collision_event_count(&self) -> usize {
        self.collision_events.lock().len()
    }

    pub(crate) fn collision_event(&self, index: usize) -> Option<CollisionEventRecord> {
        self.collision_events.lock().get(index).copied()
    }

    pub(crate) fn collision_events(&self, out: &mut [CollisionEventRecord]) -> u32 {
        let events = self.collision_events.lock();
        let count = out.len().min(events.len());
        out[..count].copy_from_slice(&events[..count]);
        count as u32
    }

    pub(crate) fn contact_force_event_count(&self) -> usize {
        self.contact_force_events.lock().len()
    }

    pub(crate) fn contact_force_event(&self, index: usize) -> Option<ContactForceEventRecord> {
        self.contact_force_events.lock().get(index).copied()
    }

    pub(crate) fn contact_force_events(&self, out: &mut [ContactForceEventRecord]) -> u32 {
        let events = self.contact_force_events.lock();
        let count = out.len().min(events.len());
        out[..count].copy_from_slice(&events[..count]);
        count as u32
    }

    pub(crate) fn custom_physics(&self) -> CustomPhysicsState {
        self.custom_physics.read().clone()
    }

    pub(crate) fn set_last_custom_physics_report(&self, report: CustomPhysicsReport) {
        self.custom_physics.write().last_report = report;
    }
}

fn push_event<T>(events: &mut Vec<T>, event: T) {
    if events.len() < MAX_EVENT_RECORDS {
        events.push(event);
    }
}

impl EventHandler for CollectingEventHandler {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
        let record = match event {
            CollisionEvent::Started(h1, h2, flags) => CollisionEventRecord {
                started: Bool::TRUE,
                collider1: pack_collider_handle(h1),
                collider2: pack_collider_handle(h2),
                sensor: flags.contains(CollisionEventFlags::SENSOR).into(),
                removed: flags.contains(CollisionEventFlags::REMOVED).into(),
            },
            CollisionEvent::Stopped(h1, h2, flags) => CollisionEventRecord {
                started: Bool::FALSE,
                collider1: pack_collider_handle(h1),
                collider2: pack_collider_handle(h2),
                sensor: flags.contains(CollisionEventFlags::SENSOR).into(),
                removed: flags.contains(CollisionEventFlags::REMOVED).into(),
            },
        };

        push_event(&mut self.collision_events.lock(), record);
    }

    fn handle_contact_force_event(
        &self,
        dt: Real,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        let event = ContactForceEvent::from_contact_pair(dt, contact_pair, total_force_magnitude);
        push_event(
            &mut self.contact_force_events.lock(),
            ContactForceEventRecord {
                collider1: pack_collider_handle(event.collider1),
                collider2: pack_collider_handle(event.collider2),
                total_force: vec3_from_rapier(event.total_force),
                total_force_magnitude: event.total_force_magnitude,
                max_force_direction: vec3_from_rapier(event.max_force_direction),
                max_force_magnitude: event.max_force_magnitude,
            },
        );
    }
}

#[derive(Default)]
pub(crate) struct CallbackPhysicsHooks {
    custom_physics: std::sync::Arc<CollectingEventHandler>,
}

impl CallbackPhysicsHooks {
    pub(crate) fn new(custom_physics: std::sync::Arc<CollectingEventHandler>) -> Self {
        Self { custom_physics }
    }
}

impl PhysicsHooks for CallbackPhysicsHooks {
    fn filter_contact_pair(
        &self,
        _context: &rapier3d::prelude::PairFilterContext,
    ) -> Option<SolverFlags> {
        Some(SolverFlags::COMPUTE_IMPULSES)
    }

    fn filter_intersection_pair(&self, _context: &rapier3d::prelude::PairFilterContext) -> bool {
        true
    }

    fn modify_solver_contacts(&self, context: &mut rapier3d::prelude::ContactModificationContext) {
        let Some(law) = self.custom_physics.custom_physics().coulomb_friction else {
            return;
        };
        if law.enabled.0 == 0 {
            return;
        }

        let static_mu = law.static_coefficient.max(0.0);
        let dynamic_mu = law.dynamic_coefficient.max(0.0);
        let threshold = law.velocity_threshold.max(0.0);
        let relative_velocity = match (context.rigid_body1, context.rigid_body2) {
            (Some(rb1), Some(rb2)) => {
                let v1 = context
                    .bodies
                    .get(rb1)
                    .map(|body| body.linvel())
                    .unwrap_or(Vector::ZERO);
                let v2 = context
                    .bodies
                    .get(rb2)
                    .map(|body| body.linvel())
                    .unwrap_or(Vector::ZERO);
                v1 - v2
            }
            (Some(rb1), None) => context
                .bodies
                .get(rb1)
                .map(|body| body.linvel())
                .unwrap_or(Vector::ZERO),
            (None, Some(rb2)) => -context
                .bodies
                .get(rb2)
                .map(|body| body.linvel())
                .unwrap_or(Vector::ZERO),
            (None, None) => Vector::ZERO,
        };
        let normal_speed = relative_velocity.dot(*context.normal);
        let tangential_speed = (relative_velocity - *context.normal * normal_speed).length();
        let friction = if tangential_speed <= threshold {
            static_mu
        } else {
            dynamic_mu
        };

        for contact in context.solver_contacts.iter_mut() {
            contact.friction = friction;
        }
    }
}

fn coulomb_law_valid(law: CoulombFrictionLaw) -> bool {
    law.static_coefficient.is_finite()
        && law.dynamic_coefficient.is_finite()
        && law.velocity_threshold.is_finite()
        && law.static_coefficient >= 0.0
        && law.dynamic_coefficient >= 0.0
        && law.velocity_threshold >= 0.0
}

fn air_drag_law_valid(law: AirDragLaw) -> bool {
    vec3_finite(law.fluid_velocity)
        && law.density.is_finite()
        && law.dynamic_viscosity.is_finite()
        && law.characteristic_length.is_finite()
        && law.reference_area.is_finite()
        && law.drag_coefficient.is_finite()
        && law.reynolds_stokes_limit.is_finite()
        && law.density >= 0.0
        && law.dynamic_viscosity > 0.0
        && law.characteristic_length > 0.0
        && law.reference_area >= 0.0
        && law.drag_coefficient >= 0.0
        && law.reynolds_stokes_limit >= 0.0
}

fn external_force_law_valid(law: ExternalForceLaw) -> bool {
    vec3_finite(law.buoyancy_gravity)
        && vec3_finite(law.electric_field)
        && vec3_finite(law.magnetic_field)
        && vec3_finite(law.spring_anchor)
        && vec3_finite(law.gravity_source)
        && law.fluid_density.is_finite()
        && law.displaced_volume.is_finite()
        && law.charge.is_finite()
        && law.spring_stiffness.is_finite()
        && law.spring_damping.is_finite()
        && law.gravitational_parameter.is_finite()
        && law.fluid_density >= 0.0
        && law.displaced_volume >= 0.0
        && law.spring_stiffness >= 0.0
        && law.spring_damping >= 0.0
        && law.gravitational_parameter >= 0.0
}

pub(crate) fn apply_custom_external_forces(world: &mut crate::rapier::world::PhysicsWorld) {
    let custom = world.events.custom_physics();
    let air_drag = custom
        .air_drag
        .filter(|law| law.enabled.0 != 0 && air_drag_law_valid(*law));
    let external_force = custom
        .external_force
        .filter(|law| law.enabled.0 != 0 && external_force_law_valid(*law));

    if air_drag.is_none() && external_force.is_none() {
        world
            .events
            .set_last_custom_physics_report(CustomPhysicsReport::default());
        return;
    }

    let mut report = CustomPhysicsReport::default();
    let mut total_drag = KahanVec3::default();
    let mut total_external = KahanVec3::default();
    for (_, body) in world.bodies.iter_mut() {
        report.body_count += 1;
        if !body.is_dynamic() {
            continue;
        }

        if let Some(law) = air_drag {
            let fluid_velocity = vec3_to_rapier(law.fluid_velocity);
            let relative_velocity = body.linvel() - fluid_velocity;
            let speed = relative_velocity.length();
            if speed > 1.0e-12 {
                let reynolds =
                    law.density * speed * law.characteristic_length / law.dynamic_viscosity;
                report.max_reynolds_number = report.max_reynolds_number.max(reynolds);
                let drag_magnitude = if reynolds <= law.reynolds_stokes_limit {
                    3.0 * std::f64::consts::PI
                        * law.dynamic_viscosity
                        * law.characteristic_length
                        * speed
                } else {
                    0.5 * law.density * speed * speed * law.drag_coefficient * law.reference_area
                };
                let force = -relative_velocity / speed * drag_magnitude;
                body.add_force(force, true);
                total_drag.add(vec3_from_rapier(force));
                report.drag_body_count += 1;
            }
        }

        if let Some(law) = external_force {
            let mut force = Vector::ZERO;
            if law.buoyancy_enabled.0 != 0 {
                force += -vec3_to_rapier(law.buoyancy_gravity)
                    * (law.fluid_density * law.displaced_volume);
            }
            if law.electromagnetic_enabled.0 != 0 {
                let velocity = body.linvel();
                let magnetic = velocity.cross(vec3_to_rapier(law.magnetic_field));
                force += (vec3_to_rapier(law.electric_field) + magnetic) * law.charge;
            }
            if law.elastic_enabled.0 != 0 {
                let displacement = body.translation() - vec3_to_rapier(law.spring_anchor);
                let damping = body.linvel() * law.spring_damping;
                force += -displacement * law.spring_stiffness - damping;
            }
            if law.gravity_enabled.0 != 0 {
                let offset = vec3_to_rapier(law.gravity_source) - body.translation();
                let distance_squared = offset.length_squared();
                if distance_squared > 1.0e-12 {
                    let mass = body.mass();
                    if mass > 0.0 {
                        force += offset / distance_squared.sqrt()
                            * (law.gravitational_parameter * mass / distance_squared);
                    }
                }
            }
            if force != Vector::ZERO {
                body.add_force(force, true);
                total_external.add(vec3_from_rapier(force));
                report.external_force_body_count += 1;
            }
        }
    }

    report.total_drag_force = total_drag.value();
    report.total_external_force = total_external.value();
    world.events.set_last_custom_physics_report(report);
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_coulomb_friction_law(
    world: *mut WorldHandle,
    law: CoulombFrictionLaw,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    if !coulomb_law_valid(law) {
        set_error(ERR_INVALID_ARGUMENT, "invalid Coulomb friction law");
        return Bool::FALSE;
    }

    world.inner.events.custom_physics.write().coulomb_friction =
        if law.enabled.0 != 0 { Some(law) } else { None };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_coulomb_friction_law_flag(
    world: *mut WorldHandle,
    law: CoulombFrictionLaw,
) -> u8 {
    world_set_coulomb_friction_law(world, law).0
}

#[unsafe(no_mangle)]
pub extern "C" fn world_clear_coulomb_friction_law(world: *mut WorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.events.custom_physics.write().coulomb_friction = None;
    clear_error();
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_coulomb_friction_law(
    world: *const WorldHandle,
    out_law: *mut CoulombFrictionLaw,
) -> Bool {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(out_law) = (unsafe { out_law.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Coulomb friction output is null");
        return Bool::FALSE;
    };

    *out_law = world
        .inner
        .events
        .custom_physics()
        .coulomb_friction
        .unwrap_or_default();
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_air_drag_law(world: *mut WorldHandle, law: AirDragLaw) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    if !air_drag_law_valid(law) {
        set_error(ERR_INVALID_ARGUMENT, "invalid air drag law");
        return Bool::FALSE;
    }

    world.inner.events.custom_physics.write().air_drag =
        if law.enabled.0 != 0 { Some(law) } else { None };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_air_drag_law_flag(world: *mut WorldHandle, law: AirDragLaw) -> u8 {
    world_set_air_drag_law(world, law).0
}

#[unsafe(no_mangle)]
pub extern "C" fn world_clear_air_drag_law(world: *mut WorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.events.custom_physics.write().air_drag = None;
    world
        .inner
        .events
        .set_last_custom_physics_report(CustomPhysicsReport::default());
    clear_error();
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_air_drag_law(
    world: *const WorldHandle,
    out_law: *mut AirDragLaw,
) -> Bool {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(out_law) = (unsafe { out_law.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "air drag output is null");
        return Bool::FALSE;
    };

    *out_law = world
        .inner
        .events
        .custom_physics()
        .air_drag
        .unwrap_or_default();
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_external_force_law(
    world: *mut WorldHandle,
    law: ExternalForceLaw,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    if !external_force_law_valid(law) {
        set_error(ERR_INVALID_ARGUMENT, "invalid external force law");
        return Bool::FALSE;
    }

    world.inner.events.custom_physics.write().external_force =
        if law.enabled.0 != 0 { Some(law) } else { None };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_external_force_law_flag(
    world: *mut WorldHandle,
    law: ExternalForceLaw,
) -> u8 {
    world_set_external_force_law(world, law).0
}

#[unsafe(no_mangle)]
pub extern "C" fn world_clear_external_force_law(world: *mut WorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.events.custom_physics.write().external_force = None;
    clear_error();
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_external_force_law(
    world: *const WorldHandle,
    out_law: *mut ExternalForceLaw,
) -> Bool {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(out_law) = (unsafe { out_law.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "external force output is null");
        return Bool::FALSE;
    };

    *out_law = world
        .inner
        .events
        .custom_physics()
        .external_force
        .unwrap_or_default();
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_custom_physics_report(
    world: *const WorldHandle,
    out_report: *mut CustomPhysicsReport,
) -> Bool {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "custom physics report output is null");
        return Bool::FALSE;
    };

    *out_report = world.inner.events.custom_physics().last_report;
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn world_clear_events(world: *mut WorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.events.clear();
}

#[unsafe(no_mangle)]
pub extern "C" fn world_collision_event_count(world: *const WorldHandle) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    world.inner.events.collision_event_count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_collision_event(
    world: *const WorldHandle,
    index: u32,
) -> CollisionEventRecord {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return CollisionEventRecord::default();
    };
    world
        .inner
        .events
        .collision_event(index as usize)
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_collision_events(
    world: *const WorldHandle,
    out_events: *mut CollisionEventRecord,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    };
    if out_events.is_null() {
        set_error(ERR_NULL_POINTER, "collision event output is null");
        return 0;
    }
    if capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
        set_error(ERR_CAPACITY, "invalid collision event output capacity");
        return 0;
    }

    clear_error();
    let out = unsafe { std::slice::from_raw_parts_mut(out_events, capacity as usize) };
    world.inner.events.collision_events(out)
}

#[unsafe(no_mangle)]
pub extern "C" fn world_contact_force_event_count(world: *const WorldHandle) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    world.inner.events.contact_force_event_count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_contact_force_event(
    world: *const WorldHandle,
    index: u32,
) -> ContactForceEventRecord {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return ContactForceEventRecord::default();
    };
    world
        .inner
        .events
        .contact_force_event(index as usize)
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_contact_force_events(
    world: *const WorldHandle,
    out_events: *mut ContactForceEventRecord,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    };
    if out_events.is_null() {
        set_error(ERR_NULL_POINTER, "contact force event output is null");
        return 0;
    }
    if capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
        set_error(ERR_CAPACITY, "invalid contact force event output capacity");
        return 0;
    }

    clear_error();
    let out = unsafe { std::slice::from_raw_parts_mut(out_events, capacity as usize) };
    world.inner.events.contact_force_events(out)
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_contact_pair_filter_callback(
    world: *mut WorldHandle,
    _callback: usize,
    _user_data: usize,
) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return;
    };
    set_error(
        ERR_UNSUPPORTED,
        "external contact pair callbacks are disabled for ABI safety",
    );
    world.inner.hooks = CallbackPhysicsHooks::new(world.inner.events.clone());
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_intersection_pair_filter_callback(
    world: *mut WorldHandle,
    _callback: usize,
    _user_data: usize,
) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return;
    };
    set_error(
        ERR_UNSUPPORTED,
        "external intersection callbacks are disabled for ABI safety",
    );
    world.inner.hooks = CallbackPhysicsHooks::new(world.inner.events.clone());
}

#[unsafe(no_mangle)]
pub extern "C" fn world_clear_contact_pair_filter_callback(world: *mut WorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.hooks = CallbackPhysicsHooks::new(world.inner.events.clone());
}

#[unsafe(no_mangle)]
pub extern "C" fn world_clear_intersection_pair_filter_callback(world: *mut WorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.hooks = CallbackPhysicsHooks::new(world.inner.events.clone());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::{BodyStatus, ShapeDesc, Vec3};

    #[test]
    fn custom_air_drag_law_applies_before_world_step() {
        let world = crate::rapier::world::world_create(Vec3::default());
        let builder =
            crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        crate::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder, 1.0);
        crate::rapier::rigid_body::rigid_body_builder_set_linvel(
            builder,
            Vec3 {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let body = crate::rapier::rigid_body::rigid_body_builder_build(builder);
        let handle = crate::rapier::rigid_body::world_insert_rigid_body(world, body);

        assert_eq!(
            world_set_air_drag_law(
                world,
                AirDragLaw {
                    fluid_velocity: Vec3::default(),
                    density: 1.225,
                    dynamic_viscosity: 1.8e-5,
                    characteristic_length: 0.1,
                    reference_area: 0.01,
                    drag_coefficient: 0.47,
                    reynolds_stokes_limit: 1.0,
                    enabled: Bool::TRUE,
                },
            ),
            Bool::TRUE
        );
        crate::rapier::world::world_step(world, 1.0 / 60.0);
        let velocity = crate::rapier::rigid_body::rigid_body_get_linvel(world, handle);
        assert!(velocity.x < 10.0);

        let mut report = CustomPhysicsReport::default();
        assert_eq!(
            world_get_custom_physics_report(world, &mut report),
            Bool::TRUE
        );
        assert_eq!(report.drag_body_count, 1);
        assert!(report.max_reynolds_number > 1.0);
        assert!(report.total_drag_force.x < 0.0);
        crate::rapier::world::world_destroy(world);
    }

    #[test]
    fn coulomb_friction_law_enables_contact_modification_hook() {
        let world = crate::rapier::world::world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        assert_eq!(
            world_set_coulomb_friction_law(
                world,
                CoulombFrictionLaw {
                    static_coefficient: 0.9,
                    dynamic_coefficient: 0.4,
                    velocity_threshold: 0.01,
                    enabled: Bool::TRUE,
                },
            ),
            Bool::TRUE
        );

        let ground_builder =
            crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Fixed as u32);
        crate::rapier::rigid_body::rigid_body_builder_set_translation(
            ground_builder,
            Vec3 {
                x: 0.0,
                y: -0.5,
                z: 0.0,
            },
        );
        let ground = crate::rapier::rigid_body::rigid_body_builder_build(ground_builder);
        let ground_handle = crate::rapier::rigid_body::world_insert_rigid_body(world, ground);
        let ground_collider = crate::rapier::collider::collider_builder_build(
            crate::rapier::collider::collider_builder_create_ex(ShapeDesc {
                shape_type: 1,
                a: 2.0,
                b: 0.25,
                c: 2.0,
                d: 0.0,
            }),
        );
        crate::rapier::collider::world_insert_collider_with_parent(
            world,
            ground_collider,
            ground_handle,
        );

        let body_builder =
            crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        crate::rapier::rigid_body::rigid_body_builder_set_translation(
            body_builder,
            Vec3 {
                x: 0.0,
                y: 0.1,
                z: 0.0,
            },
        );
        crate::rapier::rigid_body::rigid_body_builder_set_additional_mass(body_builder, 1.0);
        let body = crate::rapier::rigid_body::rigid_body_builder_build(body_builder);
        let body_handle = crate::rapier::rigid_body::world_insert_rigid_body(world, body);
        let body_collider = crate::rapier::collider::collider_builder_build(
            crate::rapier::collider::collider_builder_create_ex(ShapeDesc {
                shape_type: 1,
                a: 0.25,
                b: 0.25,
                c: 0.25,
                d: 0.0,
            }),
        );
        crate::rapier::collider::world_insert_collider_with_parent(
            world,
            body_collider,
            body_handle,
        );

        crate::rapier::world::world_step(world, 1.0 / 60.0);
        let mut out = CoulombFrictionLaw::default();
        assert_eq!(world_get_coulomb_friction_law(world, &mut out), Bool::TRUE);
        assert_eq!(out.enabled, Bool::TRUE);
        assert_eq!(out.dynamic_coefficient, 0.4);
        crate::rapier::world::world_destroy(world);
    }

    #[test]
    fn external_force_law_applies_buoyancy_em_elastic_and_gravity() {
        let world = crate::rapier::world::world_create(Vec3::default());
        let builder =
            crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        crate::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder, 2.0);
        crate::rapier::rigid_body::rigid_body_builder_set_translation(
            builder,
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
        );
        crate::rapier::rigid_body::rigid_body_builder_set_linvel(
            builder,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
        );
        let body = crate::rapier::rigid_body::rigid_body_builder_build(builder);
        let handle = crate::rapier::rigid_body::world_insert_rigid_body(world, body);

        assert_eq!(
            world_set_external_force_law(
                world,
                ExternalForceLaw {
                    buoyancy_enabled: Bool::TRUE,
                    fluid_density: 1.0,
                    displaced_volume: 1.0,
                    buoyancy_gravity: Vec3 {
                        x: 0.0,
                        y: -9.81,
                        z: 0.0,
                    },
                    electromagnetic_enabled: Bool::TRUE,
                    charge: 2.0,
                    electric_field: Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 1.0,
                    },
                    magnetic_field: Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 1.0,
                    },
                    elastic_enabled: Bool::TRUE,
                    spring_anchor: Vec3::default(),
                    spring_stiffness: 4.0,
                    spring_damping: 0.1,
                    gravity_enabled: Bool::TRUE,
                    gravity_source: Vec3::default(),
                    gravitational_parameter: 3.0,
                    enabled: Bool::TRUE,
                },
            ),
            Bool::TRUE
        );

        crate::rapier::world::world_step(world, 1.0 / 60.0);
        let velocity = crate::rapier::rigid_body::rigid_body_get_linvel(world, handle);
        assert!(velocity.x < 0.0);
        assert!(velocity.y > 1.0);
        assert!(velocity.z > 0.0);

        let mut report = CustomPhysicsReport::default();
        assert_eq!(
            world_get_custom_physics_report(world, &mut report),
            Bool::TRUE
        );
        assert_eq!(report.external_force_body_count, 1);
        assert!(report.total_external_force.x < 0.0);
        assert!(report.total_external_force.y > 0.0);
        assert!(report.total_external_force.z > 0.0);
        crate::rapier::world::world_destroy(world);
    }
}
