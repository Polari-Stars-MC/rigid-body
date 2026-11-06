//! Rope / tether bodies — one-sided cable chains (composition layer, route A).
//!
//! A **rope body** is a chain of point masses strung along the straight line
//! `start → end`, joined by XPBD distance constraints stored in an ordinary
//! [`SoftBody`]. Like `soft_chain_create` / `soft_cloth_create` this module
//! *composes* existing fork primitives — no new physics:
//!
//! * **Stretch compliance** `α_s` — how much the rope yields under tension
//!   (`0` = inextensible cable, small `1e-6..1e-4` = elastic hawser).
//! * **Unilateral (cable) mode** — Phase 19's anisotropic `DistanceConstraint`
//!   carries separate tension/compression compliance. With `unilateral` set,
//!   the compression side gets [`ROPE_CABLE_COMPRESSION_COMPLIANCE`]
//!   (effectively free), so the rope only resists *stretching*: it goes slack
//!   when its ends approach, exactly like a real cable — a mooring tether, a
//!   tow line, a winch cable. With `unilateral` clear, both sides use `α_s`
//!   and the rope behaves like an elastic band.
//! * **Slack** — rest lengths are laid out at `spacing · (1 + slack)`, so
//!   `slack > 0` creates a rope whose rest length exceeds the span; it hangs
//!   in a catenary instead of pulling taut.
//!
//! Everything else is inherited from the `soft_body_*` surface:
//!
//! * anchor an end to a rigid body → `soft_body_attach_particle`
//!   (particle `0` = `start`, particle `segments` = `end`);
//! * winch in / pay out → `soft_body_scale_rest_length` (scales both springs
//!   and distance constraints);
//! * read back → `soft_body_read_particles` / `soft_body_read_edges`.
//!
//! The body is switched to the `Xpbd` solver automatically — distance
//! constraints are inert under the default `MassSpring` path.
//!
//! ## Difference from `soft_body_build_rope`
//!
//! `soft_body_build_rope` strings bilateral XPBD constraints — an *elastic*
//! cord that also resists shortening. A rope body adds the one-sided cable
//! behaviour (unilateral compression compliance), rest-length slack, and
//! straight-span generation aimed at tether/winch scenarios.

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{Bool, Vec3, WorldHandle, vec3_finite, vec3_to_rapier};
use rapier3d::prelude::soft_body_fork::{SoftBody, SoftSolver};

/// Upper bound on rope particles accepted in one creation call.
///
/// Ropes are one-dimensional; 64k particles is far past any interactive
/// tether use and caps allocation on hostile inputs.
pub const ROPE_MAX_PARTICLES: u32 = 65_536;

/// Compression compliance used in unilateral (cable) mode.
///
/// With `dt = 1/60 s` the XPBD projection weight `α/dt²` reaches ~3.6e12,
/// dwarfing typical inverse masses (~10), so the positional correction on the
/// compression side is ~1e-12 of a normal constraint — i.e. shortening is
/// free, which is exactly what a cable is.
pub const ROPE_CABLE_COMPRESSION_COMPLIANCE: f64 = 1e9;

/// Which endpoints of the rope are pinned at creation time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum RopePinMode {
    /// Nothing is pinned — the rope falls freely (anchor later via
    /// `soft_body_attach_particle`).
    Free = 0,
    /// Pin particle `0` (the `start` endpoint).
    Start = 1,
    /// Pin the last particle (the `end` endpoint).
    End = 2,
    /// Pin both endpoints (spanned tether).
    Both = 3,
}

impl RopePinMode {
    fn from_u32(v: u32) -> Option<Self> {
        match v {
            0 => Some(Self::Free),
            1 => Some(Self::Start),
            2 => Some(Self::End),
            3 => Some(Self::Both),
            _ => None,
        }
    }
}

/// Descriptor for [`soft_rope_create`].
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct RopeDesc {
    /// Number of rope *segments*; the rope has `segments + 1` particles.
    /// Must be ≥ 1 and ≤ [`ROPE_MAX_PARTICLES`] − 1.
    pub segments: u32,
    /// World position of the first particle.
    pub start: Vec3,
    /// World position of the last particle. Must be finite and farther than
    /// 1e-9 from `start` (the rope is laid out along the straight span).
    pub end: Vec3,
    /// Mass of each *free* particle. Must be > 0 and finite. Pinned endpoints
    /// carry infinite mass regardless.
    pub particle_mass: f64,
    /// XPBD stretch compliance `α_s` (tension side). `0` = inextensible;
    /// larger = more elastic. Must be ≥ 0 and finite.
    pub stretch_compliance: f64,
    /// Rest-length slack factor: each segment's rest length is
    /// `span/segments · (1 + slack)`. `0` = laid out exactly taut. Must be
    /// ≥ 0 and finite.
    pub slack: f64,
    /// Gauss-Seidel projection iterations per XPBD substep. Must be ≥ 1.
    pub iterations: u32,
    /// When [`Bool::TRUE`], the rope only resists stretching (cable); when
    /// [`Bool::FALSE`], it is a bilateral elastic cord.
    pub unilateral: Bool,
    /// Endpoint pinning, see [`RopePinMode`].
    pub pin_mode: u32,
}

/// Create a rope body along the straight span `start → end`.
///
/// Returns the new `SoftBodyId` (as `u32`), or `u32::MAX` with the
/// thread-local error slot set to an `ERR_*` code on failure. The rope
/// integrates automatically in `world_step` — no separate stepping call is
/// needed.
///
/// # Safety
///
/// `world` must be a valid world pointer or null (null reports
/// `ERR_NULL_POINTER` and returns `u32::MAX`). No other pointers are
/// dereferenced; `desc` is passed by value.
#[unsafe(no_mangle)]
pub extern "C" fn soft_rope_create(world: *mut WorldHandle, desc: RopeDesc) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_rope_create: world is null");
            return u32::MAX;
        };

        let Some(pin_mode) = RopePinMode::from_u32(desc.pin_mode) else {
            set_error(ERR_INVALID_ARGUMENT, "soft_rope_create: unknown pin_mode");
            return u32::MAX;
        };
        if desc.segments == 0 || desc.iterations == 0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_rope_create: segments/iterations must be ≥ 1",
            );
            return u32::MAX;
        }
        let particles = desc.segments as u64 + 1;
        if particles > ROPE_MAX_PARTICLES as u64 {
            set_error(
                ERR_CAPACITY,
                "soft_rope_create: particle count exceeds ROPE_MAX_PARTICLES",
            );
            return u32::MAX;
        }
        if !vec3_finite(desc.start) || !vec3_finite(desc.end) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_rope_create: non-finite endpoints",
            );
            return u32::MAX;
        }
        let (a, b) = (vec3_to_rapier(desc.start), vec3_to_rapier(desc.end));
        let span = (b - a).length();
        if span <= 1e-9 {
            set_error(ERR_INVALID_ARGUMENT, "soft_rope_create: degenerate span");
            return u32::MAX;
        }
        if !desc.particle_mass.is_finite()
            || desc.particle_mass <= 0.0
            || !desc.stretch_compliance.is_finite()
            || desc.stretch_compliance < 0.0
            || !desc.slack.is_finite()
            || desc.slack < 0.0
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_rope_create: invalid scalar params",
            );
            return u32::MAX;
        }

        let mut body = SoftBody::new(world.inner.gravity);
        let inv_mass = 1.0 / desc.particle_mass;
        let pin_start = matches!(pin_mode, RopePinMode::Start | RopePinMode::Both);
        let pin_end = matches!(pin_mode, RopePinMode::End | RopePinMode::Both);
        let segments = desc.segments as usize;
        for i in 0..=segments {
            let t = i as f64 / segments as f64;
            let pos = a + (b - a) * t;
            let idx = match (pin_start && i == 0, pin_end && i == segments) {
                (true, _) | (_, true) => body.add_pinned(pos),
                _ => {
                    let i = body.add_particle(pos);
                    body.particles[i].inv_mass = inv_mass;
                    i
                }
            };
            debug_assert_eq!(idx, i);
        }

        // One distance constraint per segment. `add_distance_constraint` takes
        // the rest length from the laid-out (taut) positions and initialises
        // `compression = compliance` (isotropic); slack and the cable's
        // one-sided behaviour are applied right after.
        for i in 0..segments {
            body.add_distance_constraint(i, i + 1, desc.stretch_compliance);
        }
        let seg_rest = span / segments as f64 * (1.0 + desc.slack);
        for c in body.distance_constraints.iter_mut() {
            c.rest = seg_rest;
            if desc.unilateral == Bool::TRUE {
                c.compression = ROPE_CABLE_COMPRESSION_COMPLIANCE;
            }
        }

        // Distance constraints are only projected by the XPBD solver.
        body.solver = SoftSolver::Xpbd {
            iterations: desc.iterations,
            compliance: desc.stretch_compliance,
        };

        clear_error();
        world.inner.soft_bodies.insert(body).0
    })
}
