//! Hair / fur system — chain-based hair simulation attached to rigid bodies.
//!
//! A **hair system** is a collection of hair strands, each strand being a chain
//! of point masses connected by springs (similar to ropes but with much finer
//! segments and different physical properties). Hair strands can be attached to
//! rigid bodies (e.g., character head) and respond to wind, gravity, and motion.
//!
//! This is a pure composition layer on top of `soft_body.rs` — each hair strand
//! is stored as a `SoftBody` with specific parameters optimized for hair simulation
//! (low stiffness, high damping, mass-spring solver).

use rapier3d::math::Vector;
use rapier3d::prelude::RigidBodyHandle;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, ERR_UNSUPPORTED,
    clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    Bool, RigidBodyHandleRaw, Vec3, WorldHandle, unpack_rigid_body_handle, vec3_finite,
    vec3_to_rapier,
};

const MAX_HAIR_STRANDS: u32 = 512;
const MAX_HAIR_SEGMENTS: u32 = 64;
/// Fallback spring constant when a strand declares `stiffness == 0` (the
/// original compliance formulation `1e-3` inverted: k = 1/1e-3).
const FALLBACK_STIFFNESS: f64 = 1.0e3;
/// Linear air-resistance coefficient passed to `SoftBody::apply_wind` alongside
/// the wind acceleration (`F = m·wind − m·drag·v`); keeps wind from runaway.
const HAIR_WIND_DRAG: f64 = 0.1;

/// Hair strand configuration.
///
/// Passed as an array to `hair_system_create`; one strand becomes one soft
/// body whose root particle is bound to the attached rigid body.
#[derive(Debug, Clone)]
#[repr(C)]
pub struct HairStrandDesc {
    /// Root position (in local space of the attached body).
    pub root_local: Vec3,
    /// Strand direction (in local space, normalized internally).
    pub direction: Vec3,
    /// Number of segments in this strand.
    pub segment_count: u32,
    /// Total length of the strand.
    pub length: f64,
    /// Radius of each hair segment (for collision).
    pub segment_radius: f64,
    /// Linear stiffness (spring constant k; lower = softer hair).
    pub stiffness: f64,
    /// Damping coefficient for the chain springs (0-1).
    pub damping: f64,
    /// Density of hair material.
    pub density: f64,
}

/// Hair system state.
pub(crate) struct HairSystem {
    /// The rigid body this hair is attached to (hair moves with body).
    pub attached_body: RigidBodyHandle,
    /// Hair strand descriptors (used for recreation).
    pub strands: Vec<HairStrandDesc>,
    /// Soft body IDs for each strand (one SoftBody per strand).
    pub strand_soft_bodies: Vec<crate::rapier::soft_data::SoftBodyId>,
    /// Wind force applied to all strands.
    pub wind: Vec3,
    /// Gravity scale (1.0 = normal gravity, 0.0 = no gravity).
    pub gravity_scale: f64,
}

/// Create a hair system attached to a rigid body.
///
/// Returns a stable id, or `u32::MAX` on error.
///
/// # Safety
///
/// `world` must be a valid world pointer. `strands` must point to
/// `strand_count` valid descriptors.
#[unsafe(no_mangle)]
pub extern "C" fn hair_system_create(
    world: *mut WorldHandle,
    attached_body: RigidBodyHandleRaw,
    strands: *const HairStrandDesc,
    strand_count: u32,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return u32::MAX;
        };
        if strand_count == 0 || strand_count > MAX_HAIR_STRANDS {
            set_error(ERR_CAPACITY, "invalid strand count");
            return u32::MAX;
        }
        if strands.is_null() {
            set_error(ERR_NULL_POINTER, "strands is null");
            return u32::MAX;
        }

        let attached_handle = unpack_rigid_body_handle(attached_body);
        if world.inner.bodies.get(attached_handle).is_none() {
            set_error(ERR_NOT_FOUND, "attached body not found");
            return u32::MAX;
        }

        let strands_slice = unsafe { std::slice::from_raw_parts(strands, strand_count as usize) };
        for strand in strands_slice {
            if !vec3_finite(strand.root_local)
                || !vec3_finite(strand.direction)
                || strand.segment_count == 0
                || strand.segment_count > MAX_HAIR_SEGMENTS
                || strand.length <= 0.0
                || strand.segment_radius <= 0.0
                || strand.stiffness < 0.0
                || strand.damping < 0.0
                || strand.density <= 0.0
            {
                set_error(ERR_INVALID_ARGUMENT, "invalid hair strand descriptor");
                return u32::MAX;
            }
        }

        // Initialize hair system (strand soft bodies are created lazily)
        let id = world.inner.hair_systems.insert(HairSystem {
            attached_body: attached_handle,
            strands: strands_slice.to_vec(),
            strand_soft_bodies: Vec::new(),
            wind: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            gravity_scale: 1.0,
        });

        clear_error();
        id
    })
}

/// Build the hair strands (creates the actual soft bodies).
///
/// This is called after `hair_system_create` to instantiate the hair geometry.
/// Returns `true` on success.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn hair_system_build(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        if !world.inner.hair_systems.contains_key(id) {
            set_error(ERR_NOT_FOUND, "hair system not found");
            return Bool::FALSE;
        }
        if world
            .inner
            .hair_systems
            .get(id)
            .is_some_and(|h| !h.strand_soft_bodies.is_empty())
        {
            set_error(ERR_UNSUPPORTED, "hair system already built");
            return Bool::FALSE;
        }

        // Snapshot the attachment pose, world gravity and stored wind so the
        // per-strand loop borrows only what it needs.
        let (attached_body, attached_pose, gravity, wind, gravity_scale, strands) = {
            let Some(hair_system) = world.inner.hair_systems.get(id) else {
                set_error(ERR_NOT_FOUND, "hair system not found");
                return Bool::FALSE;
            };
            let Some(attached_body) = world.inner.bodies.get(hair_system.attached_body) else {
                set_error(ERR_NOT_FOUND, "attached body not found");
                return Bool::FALSE;
            };
            (
                hair_system.attached_body,
                *attached_body.position(),
                world.inner.gravity,
                hair_system.wind,
                hair_system.gravity_scale,
                hair_system.strands.clone(),
            )
        };

        for strand_desc in &strands {
            let segment_length = strand_desc.length / strand_desc.segment_count as f64;
            // Cylinder volume per segment → point mass at each node.
            let segment_mass = strand_desc.density
                * std::f64::consts::PI
                * strand_desc.segment_radius.powi(2)
                * segment_length;

            // Build particle positions along the strand direction (world space).
            let dir = vec3_to_rapier(strand_desc.direction).normalize_or_zero();
            let root_world = attached_pose * vec3_to_rapier(strand_desc.root_local);

            let mut soft_body = crate::rapier::soft_data::SoftBody::new(Vector::ZERO);
            for i in 0..=strand_desc.segment_count {
                let offset = dir * (i as f64 * segment_length);
                let idx = soft_body.add_particle(root_world + offset);
                soft_body.particles[idx].inv_mass = 1.0 / segment_mass.max(1e-12);
            }
            // Chain springs between consecutive particles. The old compliance
            // formulation (c = 1/k) maps directly onto the fork's spring
            // constant k = stiffness.
            let stiffness = if strand_desc.stiffness > 0.0 {
                strand_desc.stiffness
            } else {
                FALLBACK_STIFFNESS
            };
            for i in 0..strand_desc.segment_count as usize {
                soft_body.add_spring(i, i + 1, stiffness, strand_desc.damping);
            }

            // Environment: wind field + scaled gravity.
            soft_body.apply_wind(vec3_to_rapier(wind), HAIR_WIND_DRAG);
            soft_body.gravity = gravity * gravity_scale;

            // Anchor the root particle to the attached body so the strand
            // tracks the body's motion; spring forces route back via
            // `SoftBodySet::write_spring_forces` during the world step.
            if !soft_body.attach_particle(0, attached_body, root_world, &world.inner.bodies) {
                set_error(ERR_NOT_FOUND, "attached body not found");
                return Bool::FALSE;
            }

            let soft_id = world.inner.soft_bodies.insert(soft_body);
            world
                .inner
                .hair_systems
                .get_mut(id)
                .expect("checked above")
                .strand_soft_bodies
                .push(soft_id);
        }

        clear_error();
        Bool::TRUE
    })
}

/// Set wind force for a hair system.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn hair_system_set_wind(world: *mut WorldHandle, id: u32, wind: Vec3) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        let Some(hair_system) = world.inner.hair_systems.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "hair system not found");
            return Bool::FALSE;
        };

        if !vec3_finite(wind) {
            set_error(ERR_INVALID_ARGUMENT, "invalid wind vector");
            return Bool::FALSE;
        }

        hair_system.wind = wind;

        // Push the new wind field into every strand soft body (built or not —
        // `build` re-applies the stored wind when strands are created later).
        let strand_ids = hair_system.strand_soft_bodies.clone();
        for soft_id in strand_ids {
            if let Some(soft_body) = world.inner.soft_bodies.get_mut(soft_id) {
                soft_body.apply_wind(vec3_to_rapier(wind), HAIR_WIND_DRAG);
            }
        }

        clear_error();
        Bool::TRUE
    })
}

/// Set gravity scale for a hair system.
///
/// `scale = 0.0` disables gravity for hair (e.g., underwater hair).
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn hair_system_set_gravity_scale(
    world: *mut WorldHandle,
    id: u32,
    scale: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        let Some(hair_system) = world.inner.hair_systems.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "hair system not found");
            return Bool::FALSE;
        };

        if !scale.is_finite() || scale < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid gravity scale");
            return Bool::FALSE;
        }

        hair_system.gravity_scale = scale;
        // Re-derive each strand's gravity from the world gravity so a scale of
        // 0.0 truly disables it (e.g. underwater hair).
        let strand_ids = hair_system.strand_soft_bodies.clone();
        let world_gravity = world.inner.gravity;
        for soft_id in strand_ids {
            if let Some(soft_body) = world.inner.soft_bodies.get_mut(soft_id) {
                soft_body.gravity = world_gravity * scale;
            }
        }
        clear_error();
        Bool::TRUE
    })
}

/// Remove a hair system from the world.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn hair_system_remove(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        let Some(hair_system) = world.inner.hair_systems.remove(id) else {
            set_error(ERR_NOT_FOUND, "hair system not found");
            return Bool::FALSE;
        };

        // Remove all strand soft bodies (hair strands run without collision
        // proxies, so no proxy teardown is needed).
        for soft_id in hair_system.strand_soft_bodies {
            world.inner.soft_bodies.remove(soft_id);
        }

        clear_error();
        Bool::TRUE
    })
}

/// Query the soft-body id backing a hair strand (for particle read-out, e.g.
/// rendering). Only valid after `hair_system_build`.
///
/// Returns the `SoftBodyId.0`, or `u32::MAX` on error.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn hair_system_strand_soft_body(
    world: *mut WorldHandle,
    id: u32,
    strand_index: u32,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return u32::MAX;
        };
        let Some(hair_system) = world.inner.hair_systems.get(id) else {
            set_error(ERR_NOT_FOUND, "hair system not found");
            return u32::MAX;
        };
        match hair_system.strand_soft_bodies.get(strand_index as usize) {
            Some(sid) => {
                clear_error();
                sid.0
            }
            None => {
                set_error(ERR_INVALID_ARGUMENT, "strand index out of range");
                u32::MAX
            }
        }
    })
}
