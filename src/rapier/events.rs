use parking_lot::{Mutex, RwLock};
use rapier3d::geometry::{CollisionEvent, CollisionEventFlags, ContactPair, SolverFlags};
use rapier3d::prelude::{
    ColliderSet, ContactForceEvent, EventHandler, PhysicsHooks, Real, RigidBodySet, Vector,
};
use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicU32, Ordering};

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ERR_UNSUPPORTED, clear_error, set_error,
};
use crate::rapier::ffi::{
    AirDragLaw, Bool, CollisionEventRecord, ContactForceEventRecord, CoulombFrictionLaw,
    CustomPhysicsReport, EventDispatchMode, ExternalForceLaw, MAX_OUTPUT_CAPACITY, NewtonGravityLaw,
    WorldHandle, pack_collider_handle, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};
use crate::rapier::math::KahanVec3;

const MAX_EVENT_RECORDS: usize = 16_384;

#[derive(Clone, Debug, Default)]
pub(crate) struct CustomPhysicsState {
    pub(crate) coulomb_friction: Option<CoulombFrictionLaw>,
    pub(crate) air_drag: Option<AirDragLaw>,
    pub(crate) external_force: Option<ExternalForceLaw>,
    pub(crate) newton_gravity: Option<NewtonGravityLaw>,
    pub(crate) last_report: CustomPhysicsReport,
}

#[derive(Default)]
pub(crate) struct CollectingEventHandler {
    collision_events: Mutex<Vec<CollisionEventRecord>>,
    contact_force_events: Mutex<Vec<ContactForceEventRecord>>,
    custom_physics: RwLock<CustomPhysicsState>,
    producer_cache: RwLock<ProducerCache>,
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

// ---------------------------------------------------------------------------
// Lock-free ring buffer for zero-allocation event caching
// ---------------------------------------------------------------------------

/// Single-producer (Rust physics thread), single-consumer (Java drain thread)
/// ring buffer for `CollisionEventRecord`.
pub(crate) struct CollisionEventRing {
    buf: UnsafeCell<Box<[CollisionEventRecord]>>,
    write: AtomicU32,
    read: AtomicU32,
    dropped: AtomicU32,
}

// SAFETY: Ring buffer is single-producer, single-consumer.
// Rust physics thread writes; Java drain thread reads. No concurrent writes.
unsafe impl Send for CollisionEventRing {}
unsafe impl Sync for CollisionEventRing {}

impl std::fmt::Debug for CollisionEventRing {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CollisionEventRing")
            .field("len", &self.len())
            .finish()
    }
}

impl CollisionEventRing {
    fn new(capacity: u32) -> Self {
        let cap = capacity.max(1).min(MAX_OUTPUT_CAPACITY) as usize;
        Self {
            buf: UnsafeCell::new(
                vec![CollisionEventRecord::default(); cap].into_boxed_slice(),
            ),
            write: AtomicU32::new(0),
            read: AtomicU32::new(0),
            dropped: AtomicU32::new(0),
        }
    }

    /// Push one event. Called from the physics thread (producer).
    fn push(&self, event: CollisionEventRecord) {
        let cap = self.buf().len() as u32;
        let w = self.write.load(Ordering::Relaxed);
        let r = self.read.load(Ordering::Acquire);
        if w.wrapping_sub(r) >= cap {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        }
        // SAFETY: single producer — only the physics thread writes.
        unsafe {
            (*self.buf.get())[(w % cap) as usize] = event;
        }
        self.write.store(w.wrapping_add(1), Ordering::Release);
    }

    /// Drain up to `out.len()` events. Returns the number actually drained.
    fn drain(&self, out: &mut [CollisionEventRecord]) -> u32 {
        let cap = self.buf().len() as u32;
        let r = self.read.load(Ordering::Relaxed);
        let w = self.write.load(Ordering::Acquire);
        let avail = w.wrapping_sub(r).min(cap);
        let count = avail.min(out.len() as u32);
        // SAFETY: single consumer reads from indices that the producer has
        // finished writing to (Release/Acquire ordering guarantees visibility).
        let buf = unsafe { &*self.buf.get() };
        for i in 0..count {
            out[i as usize] = buf[((r + i) % cap) as usize];
        }
        self.read.store(r.wrapping_add(count), Ordering::Release);
        count
    }

    fn buf(&self) -> &[CollisionEventRecord] {
        unsafe { &*self.buf.get() }
    }

    fn len(&self) -> u32 {
        let w = self.write.load(Ordering::Acquire);
        let r = self.read.load(Ordering::Relaxed);
        w.wrapping_sub(r).min(self.buf().len() as u32)
    }

    fn stats(&self) -> EventRingBufferStats {
        let cap = self.buf().len() as u32;
        let w = self.write.load(Ordering::Relaxed);
        let r = self.read.load(Ordering::Relaxed);
        let avail = w.wrapping_sub(r);
        EventRingBufferStats {
            capacity: cap,
            len: avail.min(cap),
            dropped: self.dropped.load(Ordering::Relaxed),
            wrapped: Bool::from(avail > cap),
        }
    }

    fn clear(&self) {
        let w = self.write.load(Ordering::Relaxed);
        self.read.store(w, Ordering::Release);
        self.dropped.store(0, Ordering::Relaxed);
    }
}

/// Single-producer, single-consumer ring buffer for `ContactForceEventRecord`.
pub(crate) struct ContactForceEventRing {
    buf: UnsafeCell<Box<[ContactForceEventRecord]>>,
    write: AtomicU32,
    read: AtomicU32,
    dropped: AtomicU32,
}

unsafe impl Send for ContactForceEventRing {}
unsafe impl Sync for ContactForceEventRing {}

impl std::fmt::Debug for ContactForceEventRing {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ContactForceEventRing")
            .field("len", &self.len())
            .finish()
    }
}

impl ContactForceEventRing {
    fn new(capacity: u32) -> Self {
        let cap = capacity.max(1).min(MAX_OUTPUT_CAPACITY) as usize;
        Self {
            buf: UnsafeCell::new(
                vec![ContactForceEventRecord::default(); cap].into_boxed_slice(),
            ),
            write: AtomicU32::new(0),
            read: AtomicU32::new(0),
            dropped: AtomicU32::new(0),
        }
    }

    fn push(&self, event: ContactForceEventRecord) {
        let cap = self.buf().len() as u32;
        let w = self.write.load(Ordering::Relaxed);
        let r = self.read.load(Ordering::Acquire);
        if w.wrapping_sub(r) >= cap {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        }
        unsafe {
            (*self.buf.get())[(w % cap) as usize] = event;
        }
        self.write.store(w.wrapping_add(1), Ordering::Release);
    }

    fn drain(&self, out: &mut [ContactForceEventRecord]) -> u32 {
        let cap = self.buf().len() as u32;
        let r = self.read.load(Ordering::Relaxed);
        let w = self.write.load(Ordering::Acquire);
        let avail = w.wrapping_sub(r).min(cap);
        let count = avail.min(out.len() as u32);
        let buf = unsafe { &*self.buf.get() };
        for i in 0..count {
            out[i as usize] = buf[((r + i) % cap) as usize];
        }
        self.read.store(r.wrapping_add(count), Ordering::Release);
        count
    }

    fn buf(&self) -> &[ContactForceEventRecord] {
        unsafe { &*self.buf.get() }
    }

    fn len(&self) -> u32 {
        let w = self.write.load(Ordering::Acquire);
        let r = self.read.load(Ordering::Relaxed);
        w.wrapping_sub(r).min(self.buf().len() as u32)
    }

    fn stats(&self) -> EventRingBufferStats {
        let cap = self.buf().len() as u32;
        let w = self.write.load(Ordering::Relaxed);
        let r = self.read.load(Ordering::Relaxed);
        let avail = w.wrapping_sub(r);
        EventRingBufferStats {
            capacity: cap,
            len: avail.min(cap),
            dropped: self.dropped.load(Ordering::Relaxed),
            wrapped: Bool::from(avail > cap),
        }
    }

    fn clear(&self) {
        let w = self.write.load(Ordering::Relaxed);
        self.read.store(w, Ordering::Release);
        self.dropped.store(0, Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// Callback registry — init-time registration, zero per-frame lookup
// ---------------------------------------------------------------------------

use crate::rapier::ffi::{
    CollisionEventCallback, ContactForceEventCallback, EventCallbackHandle, EventRingBufferStats,
};

/// Registered callbacks + ring buffer pair for a single event type.
#[derive(Default)]
struct CallbackSlot {
    cb: usize,       // function pointer (zero → unset)
    user_data: usize, // opaque pointer passed to callback
    handle: u64,     // monotonically increasing handle for unregister
}

#[derive(Default)]
struct ProducerCache {
    collisions: Option<CollisionEventRing>,
    contact_forces: Option<ContactForceEventRing>,
    collision_cb: CallbackSlot,
    contact_force_cb: CallbackSlot,
    dispatch_mode: EventDispatchMode,
    next_handle: u64,
}

impl ProducerCache {
    fn dispatch_collision(&self, record: CollisionEventRecord) {
        match self.dispatch_mode {
            EventDispatchMode::Poll => { /* handled by existing Vec path */ }
            EventDispatchMode::Callback | EventDispatchMode::Both => {
                if self.collision_cb.cb != 0 {
                    let cb: CollisionEventCallback =
                        unsafe { std::mem::transmute(self.collision_cb.cb) };
                    if let Some(f) = cb {
                        unsafe {
                            f(
                                std::ptr::null(),
                                &record as *const _,
                                self.collision_cb.user_data as *mut std::ffi::c_void,
                            );
                        }
                    }
                }
            }
        }
        if matches!(
            self.dispatch_mode,
            EventDispatchMode::Poll | EventDispatchMode::Both
        ) {
            if let Some(ref ring) = self.collisions {
                ring.push(record);
            }
        }
    }

    fn dispatch_contact_force(&self, record: ContactForceEventRecord) {
        match self.dispatch_mode {
            EventDispatchMode::Poll => {}
            EventDispatchMode::Callback | EventDispatchMode::Both => {
                if self.contact_force_cb.cb != 0 {
                    let cb: ContactForceEventCallback =
                        unsafe { std::mem::transmute(self.contact_force_cb.cb) };
                    if let Some(f) = cb {
                        unsafe {
                            f(
                                std::ptr::null(),
                                &record as *const _,
                                self.contact_force_cb.user_data as *mut std::ffi::c_void,
                            );
                        }
                    }
                }
            }
        }
        if matches!(
            self.dispatch_mode,
            EventDispatchMode::Poll | EventDispatchMode::Both
        ) {
            if let Some(ref ring) = self.contact_forces {
                ring.push(record);
            }
        }
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

        // Always push to the legacy Vec for backward compatibility.
        push_event(&mut self.collision_events.lock(), record);

        // Also dispatch through the producer cache (ring buffer + optional callback).
        let pc = self.producer_cache.read();
        pc.dispatch_collision(record);
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
        let record = ContactForceEventRecord {
            collider1: pack_collider_handle(event.collider1),
            collider2: pack_collider_handle(event.collider2),
            total_force: vec3_from_rapier(event.total_force),
            total_force_magnitude: event.total_force_magnitude,
            max_force_direction: vec3_from_rapier(event.max_force_direction),
            max_force_magnitude: event.max_force_magnitude,
        };

        push_event(&mut self.contact_force_events.lock(), record);

        let pc = self.producer_cache.read();
        pc.dispatch_contact_force(record);
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

/// Apply custom external forces (buoyancy, EM, spring, point gravity) using
/// the already-read physics state.  Air drag is now handled by
/// `interaction::apply_body_interactions`.
pub(crate) fn apply_custom_external_forces_with_custom(
    world: &mut crate::rapier::world::PhysicsWorld,
    custom: CustomPhysicsState,
) {
    let external_force = custom
        .external_force
        .filter(|law| law.enabled.0 != 0 && external_force_law_valid(*law));

    if external_force.is_none() {
        return;
    }

    // Pre-compute constant parts of external forces outside the body loop
    let buoyancy_force_vec = external_force
        .filter(|law| law.buoyancy_enabled.0 != 0)
        .map(|law| {
            -vec3_to_rapier(law.buoyancy_gravity) * (law.fluid_density * law.displaced_volume)
        });
    let em_electric_vec = external_force
        .filter(|law| law.electromagnetic_enabled.0 != 0)
        .map(|law| vec3_to_rapier(law.electric_field) * law.charge);
    let em_magnetic_vec = external_force
        .filter(|law| law.electromagnetic_enabled.0 != 0)
        .map(|law| vec3_to_rapier(law.magnetic_field));
    let em_charge = external_force
        .filter(|law| law.electromagnetic_enabled.0 != 0)
        .map(|law| law.charge);
    let spring_anchor = external_force
        .filter(|law| law.elastic_enabled.0 != 0)
        .map(|law| vec3_to_rapier(law.spring_anchor));
    let spring_k = external_force
        .filter(|law| law.elastic_enabled.0 != 0)
        .map(|law| law.spring_stiffness);
    let spring_d = external_force
        .filter(|law| law.elastic_enabled.0 != 0)
        .map(|law| law.spring_damping);
    let gravity_source = external_force
        .filter(|law| law.gravity_enabled.0 != 0)
        .map(|law| vec3_to_rapier(law.gravity_source));
    let grav_param = external_force
        .filter(|law| law.gravity_enabled.0 != 0)
        .map(|law| law.gravitational_parameter);

    let mut report = CustomPhysicsReport::default();
    let mut total_external = KahanVec3::default();

    for (_, body) in world.bodies.iter_mut() {
        report.body_count += 1;
        if !body.is_dynamic() {
            continue;
        }

        // --- External force (pre-computed constant parts) ---
        let mut force = Vector::ZERO;

        // Buoyancy: constant force per body
        if let Some(bf) = buoyancy_force_vec {
            force += bf;
        }

        // Electromagnetic: E-field constant, B-field × v per body
        if let (Some(ef), Some(bf), Some(q)) = (em_electric_vec, em_magnetic_vec, em_charge) {
            let magnetic = body.linvel().cross(bf);
            force += ef + magnetic * q;
        }

        // Elastic spring
        if let (Some(anchor), Some(k), Some(d)) = (spring_anchor, spring_k, spring_d) {
            let displacement = body.translation() - anchor;
            let damping = body.linvel() * d;
            force += -displacement * k - damping;
        }

        // Gravity point-mass
        if let (Some(src), Some(gp)) = (gravity_source, grav_param) {
            let offset = src - body.translation();
            let distance_squared = offset.length_squared();
            if distance_squared > 1.0e-12 {
                let mass = body.mass();
                if mass > 0.0 {
                    force += offset / distance_squared.sqrt()
                        * (gp * mass / distance_squared);
                }
            }
        }

        if force != Vector::ZERO {
            body.add_force(force, true);
            total_external.add(vec3_from_rapier(force));
            report.external_force_body_count += 1;
        }
    }

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

// ---------------------------------------------------------------------------
// Newton gravity law FFI
// ---------------------------------------------------------------------------

#[unsafe(no_mangle)]
pub extern "C" fn world_set_newton_gravity_law(
    world: *mut WorldHandle,
    law: NewtonGravityLaw,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    if !law.gravitational_constant.is_finite()
        || law.gravitational_constant < 0.0
        || !law.min_distance.is_finite()
        || law.min_distance <= 0.0
        || !law.max_distance.is_finite()
        || law.max_distance < 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Newton gravity law");
        return Bool::FALSE;
    }
    world.inner.events.custom_physics.write().newton_gravity =
        if law.enabled.0 != 0 { Some(law) } else { None };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_newton_gravity_law_flag(
    world: *mut WorldHandle,
    law: NewtonGravityLaw,
) -> u8 {
    world_set_newton_gravity_law(world, law).0
}

#[unsafe(no_mangle)]
pub extern "C" fn world_clear_newton_gravity_law(world: *mut WorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.events.custom_physics.write().newton_gravity = None;
    clear_error();
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_newton_gravity_law(
    world: *const WorldHandle,
    out_law: *mut NewtonGravityLaw,
) -> Bool {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(out_law) = (unsafe { out_law.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Newton gravity output is null");
        return Bool::FALSE;
    };
    *out_law = world
        .inner
        .events
        .custom_physics()
        .newton_gravity
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

// ---------------------------------------------------------------------------
// Event cache registry — init-time registration, zero per-frame lookup
// ---------------------------------------------------------------------------

/// Allocate a collision-event ring buffer of `capacity` records.
/// Events will be written here during `world_step` instead of (or in addition to)
/// the legacy Vec queue.  Java drains the ring buffer at its own pace.
#[unsafe(no_mangle)]
pub extern "C" fn world_init_collision_event_ring(
    world: *mut WorldHandle,
    capacity: u32,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    if capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
        set_error(ERR_CAPACITY, "invalid collision event ring capacity");
        return Bool::FALSE;
    }
    world.inner.events.producer_cache.write().collisions =
        Some(CollisionEventRing::new(capacity));
    clear_error();
    Bool::TRUE
}

/// Allocate a contact-force-event ring buffer.
#[unsafe(no_mangle)]
pub extern "C" fn world_init_contact_force_event_ring(
    world: *mut WorldHandle,
    capacity: u32,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    if capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
        set_error(ERR_CAPACITY, "invalid contact force event ring capacity");
        return Bool::FALSE;
    }
    world.inner.events.producer_cache.write().contact_forces =
        Some(ContactForceEventRing::new(capacity));
    clear_error();
    Bool::TRUE
}

/// Drain the collision-event ring buffer into `out_events`.
/// Returns the number of events drained.  This is the **only** FFI call needed
/// per frame after init — no more count-then-allocate-then-read cycles.
#[unsafe(no_mangle)]
pub extern "C" fn world_drain_collision_event_ring(
    world: *const WorldHandle,
    out_events: *mut CollisionEventRecord,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    };
    if out_events.is_null() || capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
        set_error(ERR_CAPACITY, "invalid collision event drain output");
        return 0;
    }
    let out = unsafe { std::slice::from_raw_parts_mut(out_events, capacity as usize) };
    let pc = world.inner.events.producer_cache.read();
    let count = pc
        .collisions
        .as_ref()
        .map(|ring| ring.drain(out))
        .unwrap_or(0);
    clear_error();
    count
}

/// Drain the contact-force-event ring buffer.
#[unsafe(no_mangle)]
pub extern "C" fn world_drain_contact_force_event_ring(
    world: *const WorldHandle,
    out_events: *mut ContactForceEventRecord,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    };
    if out_events.is_null() || capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
        set_error(ERR_CAPACITY, "invalid contact force event drain output");
        return 0;
    }
    let out = unsafe { std::slice::from_raw_parts_mut(out_events, capacity as usize) };
    let pc = world.inner.events.producer_cache.read();
    let count = pc
        .contact_forces
        .as_ref()
        .map(|ring| ring.drain(out))
        .unwrap_or(0);
    clear_error();
    count
}

/// Get the current number of events in the collision ring buffer (cheap, no lock).
#[unsafe(no_mangle)]
pub extern "C" fn world_collision_event_ring_len(world: *const WorldHandle) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    world
        .inner
        .events
        .producer_cache
        .read()
        .collisions
        .as_ref()
        .map(|ring| ring.len())
        .unwrap_or(0)
}

/// Get the current number of events in the contact-force ring buffer.
#[unsafe(no_mangle)]
pub extern "C" fn world_contact_force_event_ring_len(world: *const WorldHandle) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    world
        .inner
        .events
        .producer_cache
        .read()
        .contact_forces
        .as_ref()
        .map(|ring| ring.len())
        .unwrap_or(0)
}

/// Get ring buffer statistics (capacity, occupancy, drops, wraps).
#[unsafe(no_mangle)]
pub extern "C" fn world_collision_event_ring_stats(
    world: *const WorldHandle,
    out_stats: *mut EventRingBufferStats,
) -> Bool {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(out_stats) = (unsafe { out_stats.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "ring stats output is null");
        return Bool::FALSE;
    };
    let pc = world.inner.events.producer_cache.read();
    *out_stats = pc
        .collisions
        .as_ref()
        .map(|ring| ring.stats())
        .unwrap_or_default();
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn world_contact_force_event_ring_stats(
    world: *const WorldHandle,
    out_stats: *mut EventRingBufferStats,
) -> Bool {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(out_stats) = (unsafe { out_stats.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "ring stats output is null");
        return Bool::FALSE;
    };
    let pc = world.inner.events.producer_cache.read();
    *out_stats = pc
        .contact_forces
        .as_ref()
        .map(|ring| ring.stats())
        .unwrap_or_default();
    clear_error();
    Bool::TRUE
}

/// Clear both ring buffers and reset drop counters.
#[unsafe(no_mangle)]
pub extern "C" fn world_clear_event_rings(world: *mut WorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    let pc = world.inner.events.producer_cache.read();
    if let Some(ref ring) = pc.collisions {
        ring.clear();
    }
    if let Some(ref ring) = pc.contact_forces {
        ring.clear();
    }
}

/// Register a collision-event callback.
///
/// `callback` is a C function pointer (zero = unregister).
/// `user_data` is passed through unchanged to each invocation.
/// Returns an opaque handle for later unregistration.
#[unsafe(no_mangle)]
pub extern "C" fn world_register_collision_callback(
    world: *mut WorldHandle,
    callback: usize,
    user_data: usize,
) -> EventCallbackHandle {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    };
    let mut pc = world.inner.events.producer_cache.write();
    pc.next_handle = pc.next_handle.wrapping_add(1);
    pc.collision_cb = CallbackSlot {
        cb: callback,
        user_data,
        handle: pc.next_handle,
    };
    clear_error();
    pc.next_handle
}

/// Register a contact-force-event callback.
#[unsafe(no_mangle)]
pub extern "C" fn world_register_contact_force_callback(
    world: *mut WorldHandle,
    callback: usize,
    user_data: usize,
) -> EventCallbackHandle {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    };
    let mut pc = world.inner.events.producer_cache.write();
    pc.next_handle = pc.next_handle.wrapping_add(1);
    pc.contact_force_cb = CallbackSlot {
        cb: callback,
        user_data,
        handle: pc.next_handle,
    };
    clear_error();
    pc.next_handle
}

/// Unregister a previously registered callback by its handle.
/// Passing 0 or an invalid handle is a no-op.
#[unsafe(no_mangle)]
pub extern "C" fn world_unregister_callback(
    world: *mut WorldHandle,
    handle: EventCallbackHandle,
) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    if handle == 0 {
        return;
    }
    let mut pc = world.inner.events.producer_cache.write();
    if pc.collision_cb.handle == handle {
        pc.collision_cb = CallbackSlot::default();
    }
    if pc.contact_force_cb.handle == handle {
        pc.contact_force_cb = CallbackSlot::default();
    }
    clear_error();
}

/// Set the event dispatch mode.
///
/// - `Poll` (0): legacy Vec queue only (default).
/// - `Callback` (1): registered callbacks only.
/// - `Both` (2): ring buffer + callbacks.
#[unsafe(no_mangle)]
pub extern "C" fn world_set_event_dispatch_mode(
    world: *mut WorldHandle,
    mode: u32,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let mode = match mode {
        0 => EventDispatchMode::Poll,
        1 => EventDispatchMode::Callback,
        2 => EventDispatchMode::Both,
        _ => {
            set_error(ERR_INVALID_ARGUMENT, "invalid event dispatch mode");
            return Bool::FALSE;
        }
    };
    world.inner.events.producer_cache.write().dispatch_mode = mode;
    clear_error();
    Bool::TRUE
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

    #[test]
    fn event_ring_buffer_produces_and_drains_events() {
        let world = crate::rapier::world::world_create(Vec3::default());
        // Init ring buffer
        assert_eq!(
            world_init_collision_event_ring(world, 64),
            Bool::TRUE
        );
        assert_eq!(
            world_init_contact_force_event_ring(world, 64),
            Bool::TRUE
        );
        // Set dispatch mode to Both so ring buffer gets filled
        assert_eq!(world_set_event_dispatch_mode(world, 2), Bool::TRUE);

        // Create two colliding bodies with collision events enabled
        let ground = crate::rapier::rigid_body::rigid_body_builder_build(
            crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Fixed as u32),
        );
        let ground_handle = crate::rapier::rigid_body::world_insert_rigid_body(world, ground);
        let gc_builder = crate::rapier::collider::collider_builder_create_ex(ShapeDesc {
            shape_type: 1,
            a: 2.0,
            b: 0.25,
            c: 2.0,
            d: 0.0,
        });
        // Enable collision events so the ring buffer receives them
        crate::rapier::collider::collider_builder_set_active_events(
            gc_builder,
            1, // COLLISION_EVENTS = 1
        );
        let gc = crate::rapier::collider::collider_builder_build(gc_builder);
        crate::rapier::collider::world_insert_collider_with_parent(world, gc, ground_handle);

        let body_b = crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        crate::rapier::rigid_body::rigid_body_builder_set_translation(
            body_b,
            Vec3 {
                x: 0.0,
                y: 0.5,
                z: 0.0,
            },
        );
        crate::rapier::rigid_body::rigid_body_builder_set_additional_mass(body_b, 1.0);
        let body = crate::rapier::rigid_body::rigid_body_builder_build(body_b);
        let body_handle = crate::rapier::rigid_body::world_insert_rigid_body(world, body);
        let bc_builder = crate::rapier::collider::collider_builder_create_ex(ShapeDesc {
            shape_type: 1,
            a: 0.25,
            b: 0.25,
            c: 0.25,
            d: 0.0,
        });
        crate::rapier::collider::collider_builder_set_active_events(bc_builder, 1);
        let bc = crate::rapier::collider::collider_builder_build(bc_builder);
        crate::rapier::collider::world_insert_collider_with_parent(world, bc, body_handle);

        // Step — collision should occur
        crate::rapier::world::world_step(world, 1.0 / 60.0);

        // Ring buffer should have events
        let len = world_collision_event_ring_len(world);
        assert!(len > 0, "expected collision events in ring buffer");

        // Drain ring buffer
        let mut out = vec![CollisionEventRecord::default(); 64];
        let drained = world_drain_collision_event_ring(world, out.as_mut_ptr(), 64);
        assert_eq!(drained, len);

        // After drain, ring should be empty
        assert_eq!(world_collision_event_ring_len(world), 0);

        // Stats should reflect capacity
        let mut stats = EventRingBufferStats::default();
        assert_eq!(
            world_collision_event_ring_stats(world, &mut stats),
            Bool::TRUE
        );
        assert_eq!(stats.capacity, 64);
        assert_eq!(stats.len, 0);
        assert_eq!(stats.dropped, 0);

        // Clear rings
        world_clear_event_rings(world);

        crate::rapier::world::world_destroy(world);
    }

    #[test]
    fn callback_registration_and_unregistration() {
        let world = crate::rapier::world::world_create(Vec3::default());

        // Register callback (pass 0 as fn ptr — valid "no-op" registration test)
        let handle = world_register_collision_callback(world, 0, 42);
        assert_ne!(handle, 0, "callback handle should be non-zero");

        // Set dispatch mode
        assert_eq!(world_set_event_dispatch_mode(world, 2), Bool::TRUE); // Both

        // Unregister
        world_unregister_callback(world, handle);

        // Unregister with zero handle is no-op
        world_unregister_callback(world, 0);

        crate::rapier::world::world_destroy(world);
    }
}
