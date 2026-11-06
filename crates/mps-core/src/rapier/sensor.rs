//! Sensor trigger zone — a **fourth body type** alongside rigid bodies, soft
//! bodies and character bodies. A sensor zone is a non-solid (sensor) collider
//! whose current set of overlapping colliders is tracked each step. It is the
//! canonical rapier-native "trigger volume" and is purely an `mps-core` layer on
//! top of the existing sensor + query APIs (no fork changes).
//!
//! Overlap detection uses `QueryPipeline::intersect_shape` over the broad-phase
//! BVH, which — like the character controller — is only populated after a
//! `world_step`. So the typical usage is: `world_step` → `sensor_zone_poll`.

use rapier3d::prelude::QueryFilter;

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    Bool, ColliderHandleRaw, ShapeDesc, Vec3, WorldHandle, shape_desc_valid, shape_from_desc,
    vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

/// A sensor trigger zone: a sensor collider plus a live set of currently-overlapping
/// colliders (keyed by their packed [`ColliderHandleRaw`]).
#[derive(Default)]
pub(crate) struct SensorZone {
    /// Sensor collider handle (in `world.inner.colliders`).
    pub collider: ColliderHandleRaw,
    /// Live set of overlapping colliders, recomputed every `poll`.
    pub current: std::collections::HashSet<ColliderHandleRaw>,
    /// `true` once at least one overlap has been observed (sticky until reset).
    pub ever_triggered: bool,
    /// When `false`, `poll` is a no-op (zone disabled).
    pub enabled: bool,
    /// When `true`, `is_triggered` reports a rising-edge latch (TRUE only on the
    /// step an overlap first appears) instead of the sticky level flag.
    pub edge_mode: bool,
    /// Rising-edge latch: TRUE on the poll where an overlap first appeared.
    pub edge_triggered: bool,
    /// Sticky one-shot latch: set TRUE on any rising edge, kept TRUE until the
    /// caller consumes it via [`sensor_zone_consume`] (read-and-clear) or resets it
    /// via [`sensor_zone_clear`]. Lets a one-frame edge trigger be reliably
    /// observed even if `poll` runs multiple times before the app reads it.
    pub triggered_latch: bool,
}

/// Create a sensor trigger zone from a shape descriptor. The sensor collider is
/// built with `sensor(true)` and `ActiveEvents::COLLISION_EVENTS` so rapier tracks
/// its intersections, then inserted into the world at `translation`.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `shape` must be a
/// valid [`ShapeDesc`] (finite params).
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_create(
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
                "sensor_zone_create: invalid shape/translation",
            );
            return u32::MAX;
        }
        let mut builder = ColliderBuilderShim::new(shape_from_desc(shape));
        builder = builder
            .sensor(true)
            .active_events(rapier3d::prelude::ActiveEvents::COLLISION_EVENTS)
            .translation(vec3_to_rapier(translation));
        let collider = builder.build();
        let handle = world.inner.colliders.insert(collider);
        world.inner.sensor_zones.insert(SensorZone {
            collider: crate::rapier::ffi::pack_collider_handle(handle),
            current: std::collections::HashSet::new(),
            ever_triggered: false,
            enabled: true,
            edge_mode: false,
            edge_triggered: false,
            triggered_latch: false,
        })
    })
}

/// Change a sensor zone's shape after creation. The old sensor collider is
/// removed from the world and a new one built from `shape` is inserted at the
/// zone's current position. Useful for Minecraft-style trigger volumes that grow
/// or shrink as the game state changes.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `shape` must be a
/// valid [`ShapeDesc`] (finite params).
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_set_shape(
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
            set_error(ERR_INVALID_ARGUMENT, "sensor_zone_set_shape: invalid shape");
            return Bool::FALSE;
        }
        // Mutate the existing sensor collider's shape in place (same handle, same
        // pose). This keeps `zone.collider` valid and avoids rebuilding the body.
        let zone = match world.inner.sensor_zones.get_mut(id) {
            Some(z) => z,
            None => {
                set_error(ERR_NOT_FOUND, "sensor_zone_set_shape: unknown id");
                return Bool::FALSE;
            }
        };
        let handle = crate::rapier::ffi::unpack_collider_handle(zone.collider);
        match world.inner.colliders.get_mut(handle) {
            Some(collider) => collider.set_shape(shape_from_desc(shape)),
            None => {
                set_error(ERR_NOT_FOUND, "sensor_zone_set_shape: collider missing");
                return Bool::FALSE;
            }
        }
        // Reset overlap bookkeeping; the next poll recomputes it.
        zone.current.clear();
        zone.ever_triggered = false;
        clear_error();
        Bool::TRUE
    })
}

/// Disable or (re-)enable a sensor zone. A disabled zone is skipped by `poll`.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_set_enabled(world: *mut WorldHandle, id: u32, enabled: Bool) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        match world.inner.sensor_zones.get_mut(id) {
            Some(zone) => {
                zone.enabled = enabled == Bool::TRUE;
                Bool::TRUE
            }
            None => {
                set_error(ERR_NOT_FOUND, "sensor_zone_set_enabled: unknown id");
                Bool::FALSE
            }
        }
    })
}

/// Switch a sensor zone between level triggering (sticky: `is_triggered` stays
/// TRUE while anything overlaps) and rising-edge triggering (`is_triggered` is
/// TRUE only on the poll where an overlap first appears, then FALSE until the
/// zone is empty and re-entered). Edge mode is what you want for one-shot
/// "player entered the room" events.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_set_edge(world: *mut WorldHandle, id: u32, edge: Bool) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        match world.inner.sensor_zones.get_mut(id) {
            Some(zone) => {
                zone.edge_mode = edge == Bool::TRUE;
                Bool::TRUE
            }
            None => {
                set_error(ERR_NOT_FOUND, "sensor_zone_set_edge: unknown id");
                Bool::FALSE
            }
        }
    })
}

/// Recompute the set of colliders currently overlapping this sensor zone.
///
/// Returns `Bool::TRUE` on success. After a successful poll, use
/// [`sensor_zone_contact_count`] / [`sensor_zone_get_contacts`] to read the
/// overlaps, or [`sensor_zone_is_triggered`] for the sticky "ever triggered" flag.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_poll(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(zone) = world.inner.sensor_zones.get(id) else {
            set_error(ERR_NOT_FOUND, "sensor_zone_poll: unknown id");
            return Bool::FALSE;
        };
        if !zone.enabled {
            return Bool::TRUE;
        }
        let handle = crate::rapier::ffi::unpack_collider_handle(zone.collider);
        if world.inner.colliders.get(handle).is_none() {
            set_error(ERR_NOT_FOUND, "sensor_zone_poll: collider missing");
            return Bool::FALSE;
        }
        // The query borrows `world.inner` immutably; the `collider` lookup is scoped
        // to this block so the borrow ends before we mutate the zone map below.
        let overlaps: std::collections::HashSet<ColliderHandleRaw> = {
            let query = world.inner.broad_phase.as_query_pipeline(
                world.inner.narrow_phase.query_dispatcher(),
                &world.inner.bodies,
                &world.inner.colliders,
                QueryFilter::default(),
            );
            let collider = world.inner.colliders.get(handle).unwrap();
            let pose = *collider.position();
            let shape = collider.shape();
            let mut set = std::collections::HashSet::new();
            // Exclude the zone's own collider so it doesn't report itself.
            for (other, _) in query.intersect_shape(pose, shape) {
                if other == handle {
                    continue;
                }
                set.insert(crate::rapier::ffi::pack_collider_handle(other));
            }
            set
        };
        // `query` is dropped above, so we can now mutably borrow the zone map.
        let zone = world.inner.sensor_zones.get_mut(id).unwrap();
        let was_empty = zone.current.is_empty();
        if !overlaps.is_empty() {
            zone.ever_triggered = true;
        }
        // Rising edge: an overlap appeared this poll that was not present before.
        zone.edge_triggered = !overlaps.is_empty() && was_empty;
        // Latch the edge so a one-frame trigger can be reliably consumed later.
        zone.triggered_latch |= zone.edge_triggered;
        zone.current = overlaps;
        Bool::TRUE
    })
}

/// Number of colliders currently overlapping the zone (last [`sensor_zone_poll`]).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_contact_count(world: *const WorldHandle, id: u32) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        match world.inner.sensor_zones.get(id) {
            Some(zone) => zone.current.len() as u32,
            None => {
                set_error(ERR_NOT_FOUND, "sensor_zone_contact_count: unknown id");
                0
            }
        }
    })
}

/// Write up to `max_count` overlapping collider handles into `out` (packed
/// [`ColliderHandleRaw`]). Returns the number actually written.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `out` may be null
/// (then only the count is returned).
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_get_contacts(
    world: *const WorldHandle,
    id: u32,
    out: *mut ColliderHandleRaw,
    max_count: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let Some(zone) = world.inner.sensor_zones.get(id) else {
            set_error(ERR_NOT_FOUND, "sensor_zone_get_contacts: unknown id");
            return 0;
        };
        let mut written = 0u32;
        if !out.is_null() {
            for (i, h) in zone.current.iter().take(max_count as usize).enumerate() {
                unsafe { *out.add(i) = *h };
                written += 1;
            }
        } else {
            written = zone.current.len().min(max_count as usize) as u32;
        }
        written
    })
}

/// `Bool::TRUE` if the zone has ever overlapped anything since creation (sticky).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_is_triggered(world: *const WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        match world.inner.sensor_zones.get(id) {
            Some(zone) => {
                if zone.edge_mode {
                    Bool::from(zone.edge_triggered)
                } else {
                    Bool::from(zone.ever_triggered)
                }
            }
            None => {
                set_error(ERR_NOT_FOUND, "sensor_zone_is_triggered: unknown id");
                Bool::FALSE
            }
        }
    })
}

/// Read-and-clear the zone's sticky edge latch. Returns `Bool::TRUE` if a rising
/// edge had been observed since the last consume/clear, then resets the latch to
/// `FALSE`. In edge mode this is the reliable way to handle a one-shot trigger:
/// call `poll` (or `world_step` + `poll`) then `consume` exactly once per event,
/// so a single entry is never handled twice. No fork changes.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_consume(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(zone) = world.inner.sensor_zones.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "sensor_zone_consume: unknown id");
            return Bool::FALSE;
        };
        let was = zone.triggered_latch;
        zone.triggered_latch = false;
        clear_error();
        Bool::from(was)
    })
}

/// Reset the zone's trigger state: clears the sticky edge latch and the
/// `ever_triggered` sticky flag. The current overlaps are left as-is (until the
/// next `poll`). Use this to re-arm a zone after handling an event, or to forget a
/// previous entry. No fork changes.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_clear(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(zone) = world.inner.sensor_zones.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "sensor_zone_clear: unknown id");
            return Bool::FALSE;
        };
        zone.triggered_latch = false;
        zone.ever_triggered = false;
        clear_error();
        Bool::TRUE
    })
}

/// Read the zone's world-space translation (its sensor collider pose).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`; `out` may be null.
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_get_translation(
    world: *const WorldHandle,
    id: u32,
    out: *mut Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(zone) = world.inner.sensor_zones.get(id) else {
            set_error(ERR_NOT_FOUND, "sensor_zone_get_translation: unknown id");
            return Bool::FALSE;
        };
        let handle = crate::rapier::ffi::unpack_collider_handle(zone.collider);
        let Some(collider) = world.inner.colliders.get(handle) else {
            set_error(
                ERR_NOT_FOUND,
                "sensor_zone_get_translation: collider missing",
            );
            return Bool::FALSE;
        };
        if !out.is_null() {
            unsafe { *out = vec3_from_rapier(collider.position().translation) };
        }
        Bool::TRUE
    })
}

/// Move the sensor collider (call before [`sensor_zone_poll`] to re-evaluate at a
/// new position without recreating the zone).
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_set_translation(
    world: *mut WorldHandle,
    id: u32,
    translation: Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !vec3_finite(translation) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "sensor_zone_set_translation: non-finite",
            );
            return Bool::FALSE;
        }
        let Some(zone) = world.inner.sensor_zones.get(id) else {
            set_error(ERR_NOT_FOUND, "sensor_zone_set_translation: unknown id");
            return Bool::FALSE;
        };
        let handle = crate::rapier::ffi::unpack_collider_handle(zone.collider);
        match world.inner.colliders.get_mut(handle) {
            Some(collider) => {
                collider.set_translation(vec3_to_rapier(translation));
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_NOT_FOUND,
                    "sensor_zone_set_translation: collider missing",
                );
                Bool::FALSE
            }
        }
    })
}

/// Destroy a sensor zone and remove its collider from the world.
///
/// # Safety
/// `world` must be a valid pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn sensor_zone_destroy(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        match world.inner.sensor_zones.remove(id) {
            Some(zone) => {
                let handle = crate::rapier::ffi::unpack_collider_handle(zone.collider);
                world.inner.colliders.remove(
                    handle,
                    &mut world.inner.islands,
                    &mut world.inner.bodies,
                    &mut world.inner.upstream_soft_bodies,
                    true,
                );
                Bool::TRUE
            }
            None => {
                set_error(ERR_NOT_FOUND, "sensor_zone_destroy: unknown id");
                Bool::FALSE
            }
        }
    })
}

// Local shim so the collider builder helper reads the same way as the rest of the
// crate (avoids a top-level `use` collision with `ColliderBuilder` if refactored).
use rapier3d::prelude::ColliderBuilder as ColliderBuilderShim;
