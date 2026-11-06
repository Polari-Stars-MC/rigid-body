//! Ray-cast vehicle controller — a **fifth body type** built on top of rapier's
//! [`DynamicRayCastVehicleController`]. It drives a dynamic chassis rigid body and
//! N wheels via suspension ray-casts against the world. Pure `mps-core` layer (no
//! fork changes); rapier already provides the controller.
//!
//! Driving model (per step):
//! 1. `world_step(world, dt)` — advances the chassis + terrains and refreshes the
//!    broad-phase BVH.
//! 2. `vehicle_controller_update(world, id, dt)` — builds a `QueryPipelineMut`
//!    (requires `&mut WorldHandle`) and calls rapier's `update_vehicle`, which
//!    applies suspension/engine/brake impulses to the chassis.

use rapier3d::control::DynamicRayCastVehicleController;
use rapier3d::prelude::{
    ColliderBuilder, QueryFilter, RigidBodyBuilder, RigidBodyHandle, RigidBodyType,
};

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    Bool, ShapeDesc, Vec3, WorldHandle, shape_desc_valid, shape_from_desc, vec3_finite,
    vec3_from_rapier, vec3_to_rapier,
};

/// A ray-cast vehicle: a dynamic chassis body + rapier's vehicle controller.
pub(crate) struct VehicleController {
    /// rapier vehicle controller (owns suspension tuning + wheel states).
    pub controller: DynamicRayCastVehicleController,
    /// Chassis rigid body handle.
    pub body: RigidBodyHandle,
}

/// Create a vehicle controller around a dynamic chassis built from `shape` at
/// `translation`. Returns a stable id, or `u32::MAX` on error.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `shape` must be a
/// valid [`ShapeDesc`] (finite params).
#[unsafe(no_mangle)]
pub extern "C" fn vehicle_controller_create(
    world: *mut WorldHandle,
    shape: ShapeDesc,
    translation: Vec3,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return u32::MAX;
        };
        if !shape_desc_valid(shape) || !vec3_finite(translation) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "vehicle_controller_create: invalid shape/translation",
            );
            return u32::MAX;
        }
        let mut builder = RigidBodyBuilder::new(RigidBodyType::Dynamic);
        builder = builder.translation(vec3_to_rapier(translation));
        let body = world.inner.bodies.insert(builder.build());
        let collider = ColliderBuilder::new(shape_from_desc(shape)).build();
        world
            .inner
            .colliders
            .insert_with_parent(collider, body, &mut world.inner.bodies);
        let controller = DynamicRayCastVehicleController::new(body);
        world
            .inner
            .vehicle_controllers
            .insert(VehicleController { controller, body })
    })
}

/// Change a vehicle's chassis collision shape after creation. The existing
/// chassis collider is removed and a new one built from `shape` is parented to
/// the same dynamic chassis body (wheels/suspension are untouched). Useful for
/// swapping the chassis hitbox (e.g. a Minecraft minecart vs. a boat).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `shape` must be a
/// valid [`ShapeDesc`] (finite params).
#[unsafe(no_mangle)]
pub extern "C" fn vehicle_controller_set_shape(
    world: *mut WorldHandle,
    id: u32,
    shape: ShapeDesc,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !shape_desc_valid(shape) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "vehicle_controller_set_shape: invalid shape",
            );
            return Bool::FALSE;
        }
        let body = match world.inner.vehicle_controllers.get(id) {
            Some(v) => v.body,
            None => {
                set_error(ERR_NOT_FOUND, "vehicle_controller_set_shape: unknown id");
                return Bool::FALSE;
            }
        };
        // Mutate the chassis collider's shape in place (same handle, same body).
        // This avoids removing/re-inserting the collider, which would leave the body
        // massless mid-step and destabilize the ray-cast vehicle suspension.
        let old_handle = match world.inner.bodies.get(body) {
            Some(b) => match b.colliders().first() {
                Some(h) => *h,
                None => {
                    set_error(
                        ERR_NOT_FOUND,
                        "vehicle_controller_set_shape: no chassis collider",
                    );
                    return Bool::FALSE;
                }
            },
            None => {
                set_error(ERR_NOT_FOUND, "vehicle_controller_set_shape: body missing");
                return Bool::FALSE;
            }
        };
        match world.inner.colliders.get_mut(old_handle) {
            Some(collider) => collider.set_shape(shape_from_desc(shape)),
            None => {
                set_error(
                    ERR_NOT_FOUND,
                    "vehicle_controller_set_shape: collider missing",
                );
                return Bool::FALSE;
            }
        }
        clear_error();
        Bool::TRUE
    })
}

/// Add a wheel to the vehicle. All vectors are in the chassis' local space.
///
/// - `chassis_connection_cs`: point on the chassis where the suspension attaches.
/// - `direction_cs`: suspension direction (e.g. `-Y` to point down).
/// - `axle_cs`: wheel axle direction (e.g. `-Z` or `+X`).
/// - `suspension_rest_length`: natural suspension length.
/// - `radius`: wheel radius.
/// - `suspension_stiffness`, `suspension_compression`, `suspension_damping`,
///   `friction_slip`, `max_suspension_travel`, `max_suspension_force`,
///   `side_friction_stiffness`: tuning (see rapier `WheelTuning`).
///
/// Returns the wheel index, or `u32::MAX` on error.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; vectors must be finite.
#[unsafe(no_mangle)]
pub extern "C" fn vehicle_controller_add_wheel(
    world: *mut WorldHandle,
    id: u32,
    chassis_connection_cs: Vec3,
    direction_cs: Vec3,
    axle_cs: Vec3,
    suspension_rest_length: f64,
    radius: f64,
    suspension_stiffness: f64,
    suspension_compression: f64,
    suspension_damping: f64,
    friction_slip: f64,
    max_suspension_travel: f64,
    max_suspension_force: f64,
    side_friction_stiffness: f64,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return u32::MAX;
        };
        let finite = vec3_finite(chassis_connection_cs)
            && vec3_finite(direction_cs)
            && vec3_finite(axle_cs)
            && suspension_rest_length.is_finite()
            && radius.is_finite()
            && suspension_stiffness.is_finite()
            && suspension_compression.is_finite()
            && suspension_damping.is_finite()
            && friction_slip.is_finite()
            && max_suspension_travel.is_finite()
            && max_suspension_force.is_finite()
            && side_friction_stiffness.is_finite();
        if !finite {
            set_error(
                ERR_INVALID_ARGUMENT,
                "vehicle_controller_add_wheel: non-finite param",
            );
            return u32::MAX;
        }
        let Some(zone) = world.inner.vehicle_controllers.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "vehicle_controller_add_wheel: unknown id");
            return u32::MAX;
        };
        let tuning = rapier3d::control::WheelTuning {
            suspension_stiffness,
            suspension_compression,
            suspension_damping,
            max_suspension_travel,
            max_suspension_force,
            friction_slip,
            side_friction_stiffness,
        };
        let idx = zone.controller.wheels().len() as u32;
        zone.controller.add_wheel(
            vec3_to_rapier(chassis_connection_cs),
            vec3_to_rapier(direction_cs),
            vec3_to_rapier(axle_cs),
            suspension_rest_length,
            radius,
            &tuning,
        );
        idx
    })
}

/// Set the engine force (drive torque) on a wheel.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn vehicle_controller_set_engine_force(
    world: *mut WorldHandle,
    id: u32,
    wheel_index: u32,
    force: f64,
) -> Bool {
    set_wheel_field(world, id, wheel_index, |w| w.engine_force = force)
}

/// Set the brake force on a wheel.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn vehicle_controller_set_brake(
    world: *mut WorldHandle,
    id: u32,
    wheel_index: u32,
    brake: f64,
) -> Bool {
    set_wheel_field(world, id, wheel_index, |w| w.brake = brake)
}

/// Set the steering angle (radians) on a wheel.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn vehicle_controller_set_steering(
    world: *mut WorldHandle,
    id: u32,
    wheel_index: u32,
    steering: f64,
) -> Bool {
    set_wheel_field(world, id, wheel_index, |w| w.steering = steering)
}

/// Helper: mutate a single wheel field behind the FFI error contract.
fn set_wheel_field(
    world: *mut WorldHandle,
    id: u32,
    wheel_index: u32,
    f: impl FnOnce(&mut rapier3d::control::Wheel),
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(vehicle) = world.inner.vehicle_controllers.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "vehicle controller: unknown id");
            return Bool::FALSE;
        };
        let wheels = vehicle.controller.wheels_mut();
        if (wheel_index as usize) >= wheels.len() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "vehicle_controller_set_*: wheel_index out of range",
            );
            return Bool::FALSE;
        }
        f(&mut wheels[wheel_index as usize]);
        Bool::TRUE
    })
}

/// Advance the vehicle physics by `dt`: build a `QueryPipelineMut` and let rapier
/// apply suspension/engine/brake impulses to the chassis. Call **after**
/// `world_step`.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn vehicle_controller_update(world: *mut WorldHandle, id: u32, dt: f64) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(vehicle) = world.inner.vehicle_controllers.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "vehicle_controller_update: unknown id");
            return Bool::FALSE;
        };
        let query = world.inner.broad_phase.as_query_pipeline_mut(
            world.inner.narrow_phase.query_dispatcher(),
            &mut world.inner.bodies,
            &mut world.inner.colliders,
            QueryFilter::default(),
        );
        vehicle.controller.update_vehicle(dt, query);
        Bool::TRUE
    })
}

/// Read the chassis world-space translation.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `out` may be null.
#[unsafe(no_mangle)]
pub extern "C" fn vehicle_controller_get_translation(
    world: *const WorldHandle,
    id: u32,
    out: *mut Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(vehicle) = world.inner.vehicle_controllers.get(id) else {
            set_error(
                ERR_NOT_FOUND,
                "vehicle_controller_get_translation: unknown id",
            );
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get(vehicle.body) else {
            set_error(
                ERR_NOT_FOUND,
                "vehicle_controller_get_translation: body missing",
            );
            return Bool::FALSE;
        };
        if !out.is_null() {
            unsafe { *out = vec3_from_rapier(body.translation()) };
        }
        Bool::TRUE
    })
}

/// Read the chassis world-space linear velocity.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `out` may be null.
#[unsafe(no_mangle)]
pub extern "C" fn vehicle_controller_get_velocity(
    world: *const WorldHandle,
    id: u32,
    out: *mut Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(vehicle) = world.inner.vehicle_controllers.get(id) else {
            set_error(ERR_NOT_FOUND, "vehicle_controller_get_velocity: unknown id");
            return Bool::FALSE;
        };
        let Some(body) = world.inner.bodies.get(vehicle.body) else {
            set_error(
                ERR_NOT_FOUND,
                "vehicle_controller_get_velocity: body missing",
            );
            return Bool::FALSE;
        };
        if !out.is_null() {
            unsafe { *out = vec3_from_rapier(body.linvel()) };
        }
        Bool::TRUE
    })
}

/// Read a wheel's suspension contact state (is the wheel touching the ground?).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn vehicle_controller_wheel_on_ground(
    world: *const WorldHandle,
    id: u32,
    wheel_index: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(vehicle) = world.inner.vehicle_controllers.get(id) else {
            set_error(
                ERR_NOT_FOUND,
                "vehicle_controller_wheel_on_ground: unknown id",
            );
            return Bool::FALSE;
        };
        let wheels = vehicle.controller.wheels();
        if (wheel_index as usize) >= wheels.len() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "vehicle_controller_wheel_on_ground: wheel_index out of range",
            );
            return Bool::FALSE;
        }
        let on_ground = wheels[wheel_index as usize].raycast_info().is_in_contact;
        Bool::from(on_ground)
    })
}

/// Read a wheel's contact normal (world space).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `out` may be null.
#[unsafe(no_mangle)]
pub extern "C" fn vehicle_controller_wheel_contact_normal(
    world: *const WorldHandle,
    id: u32,
    wheel_index: u32,
    out: *mut Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(vehicle) = world.inner.vehicle_controllers.get(id) else {
            set_error(
                ERR_NOT_FOUND,
                "vehicle_controller_wheel_contact_normal: unknown id",
            );
            return Bool::FALSE;
        };
        let wheels = vehicle.controller.wheels();
        if (wheel_index as usize) >= wheels.len() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "vehicle_controller_wheel_contact_normal: wheel_index out of range",
            );
            return Bool::FALSE;
        }
        if !out.is_null() {
            unsafe {
                *out = vec3_from_rapier(
                    wheels[wheel_index as usize]
                        .raycast_info()
                        .contact_normal_ws,
                );
            }
        }
        Bool::TRUE
    })
}

/// Destroy a vehicle controller and its chassis body + collider.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn vehicle_controller_destroy(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        match world.inner.vehicle_controllers.remove(id) {
            Some(vehicle) => {
                world.inner.bodies.remove(
                    vehicle.body,
                    &mut world.inner.islands,
                    &mut world.inner.colliders,
                    &mut world.inner.impulse_joints,
                    &mut world.inner.multibody_joints,
                    &mut world.inner.upstream_soft_bodies,
                    true,
                );
                Bool::TRUE
            }
            None => {
                set_error(ERR_NOT_FOUND, "vehicle_controller_destroy: unknown id");
                Bool::FALSE
            }
        }
    })
}
