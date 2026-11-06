//! Balloon / inflatable bodies — pressurized closed shells (composition layer).
//!
//! A **balloon body** is a closed UV-sphere particle shell (latitude rings +
//! two poles) whose triangles feed the fork's Phase 11 pressure model. Like
//! `soft_cloth_create` / `soft_rope_create` this module *composes* existing
//! primitives — no new physics:
//!
//! * **Shell mesh** — particles are laid out on the sphere `center ± radius`
//!   (latitude rings `rings × segments` plus one particle per pole); each
//!   quad is split into two triangles, each pole capped by a fan. Adding a
//!   triangle auto-registers its three edges as XPBD distance constraints
//!   (deduplicated), so the shell wireframe — ring edges, meridians, one
//!   diagonal per quad, pole spokes — comes for free.
//! * **Pressure** — `F_i = Σ_t P · area(t) · n̂(t)` over the incident
//!   triangles, with centroid-oriented outward normals (Phase 11). Applied in
//!   the XPBD predict step alongside gravity/wind, so a closed shell inflates
//!   symmetrically. `pressure = 0` leaves the field unset (no per-step cost).
//! * **Softness** — every shell edge shares `edge_compliance` on the tension
//!   side; `0` = inextensible skin, larger = a stretchy balloon that inflates
//!   until edge tension balances the internal pressure.
//!
//! Everything else is inherited from the `soft_body_*` surface:
//! `soft_body_set_pressure` (pump up / vent at runtime), `soft_body_apply_wind`,
//! `soft_body_attach_particle` (tether the balloon), `soft_body_read_particles`
//! / `soft_body_read_surface_mesh` (render read-back).
//!
//! Volume conservation (`soft_body_set_volume_conservation`) is *not* wired
//! here: it constrains tetrahedra, and the balloon shell carries none — the
//! pressure model alone maintains inflation.
//!
//! ## Error convention
//!
//! As with `soft_cloth_create` / `soft_rope_create`, `soft_balloon_create`
//! returns `u32::MAX` on error (error slot carries the `ERR_*` code) and the
//! new `SoftBodyId` otherwise, clearing the thread-local error slot.

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{Vec3, WorldHandle, vec3_finite};
use rapier3d::math::Vector;
use rapier3d::prelude::soft_body_fork::{SoftBody, SoftSolver};

/// Upper bound on balloon particles accepted in one creation call.
///
/// The shell has `rings · segments + 2` particles and `add_triangle` dedups
/// its edges with an O(existing-edges) scan, giving O(n²) build cost — 4k
/// particles (~8k edges) keeps that well under a second while capping
/// allocation on hostile inputs.
pub const BALLOON_MAX_PARTICLES: u32 = 4_096;

/// Index of the shell particle at latitude ring `r`, longitude `s`.
fn shell_index(r: u32, s: u32, segments: u32) -> usize {
    (r * segments + s) as usize
}

/// Descriptor for [`soft_balloon_create`].
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct BalloonDesc {
    /// Latitude rings between the poles. Must be ≥ 2. Shell particle count is
    /// `rings · segments + 2` (see [`BALLOON_MAX_PARTICLES`]).
    pub rings: u32,
    /// Longitude segments per ring. Must be ≥ 3.
    pub segments: u32,
    /// World position of the shell centre.
    pub center: Vec3,
    /// Shell radius. Must be > 0 and finite.
    pub radius: f64,
    /// Mass of each shell particle. Must be > 0 and finite.
    pub particle_mass: f64,
    /// XPBD compliance shared by every shell edge (tension side). `0` =
    /// inextensible skin; larger = stretchier balloon. Must be ≥ 0.
    pub edge_compliance: f64,
    /// Initial internal pressure `P` (see the module docs). `0` starts the
    /// balloon uninflated — pump it up later via `soft_body_set_pressure`.
    /// Must be ≥ 0 and finite.
    pub pressure: f64,
    /// Gauss-Seidel projection iterations per XPBD substep. Must be ≥ 1.
    pub iterations: u32,
}

/// Create an inflated balloon: a closed, pressurized sphere shell.
///
/// Returns the new `SoftBodyId` (as `u32`), or `u32::MAX` with the
/// thread-local error slot set to an `ERR_*` code on failure. The balloon
/// integrates automatically in `world_step` — no separate stepping call is
/// needed.
///
/// # Safety
///
/// `world` must be a valid world pointer or null (null reports
/// `ERR_NULL_POINTER` and returns `u32::MAX`). No other pointers are
/// dereferenced; `desc` is passed by value.
#[unsafe(no_mangle)]
pub extern "C" fn soft_balloon_create(world: *mut WorldHandle, desc: BalloonDesc) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_balloon_create: world is null");
            return u32::MAX;
        };

        if desc.rings < 2 || desc.segments < 3 || desc.iterations == 0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_balloon_create: need rings ≥ 2, segments ≥ 3, iterations ≥ 1",
            );
            return u32::MAX;
        }
        let particles = (desc.rings as u64) * (desc.segments as u64) + 2;
        if particles > BALLOON_MAX_PARTICLES as u64 {
            set_error(
                ERR_CAPACITY,
                "soft_balloon_create: shell exceeds BALLOON_MAX_PARTICLES",
            );
            return u32::MAX;
        }
        if !vec3_finite(desc.center)
            || !desc.radius.is_finite()
            || desc.radius <= 0.0
            || !desc.particle_mass.is_finite()
            || desc.particle_mass <= 0.0
            || !desc.edge_compliance.is_finite()
            || desc.edge_compliance < 0.0
            || !desc.pressure.is_finite()
            || desc.pressure < 0.0
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_balloon_create: invalid scalar params",
            );
            return u32::MAX;
        }

        let mut body = SoftBody::new(world.inner.gravity);
        let inv_mass = 1.0 / desc.particle_mass;
        let (rings, segments) = (desc.rings, desc.segments);

        // Latitude rings between the poles (theta from the +Y pole), then the
        // two pole particles.
        for r in 0..rings {
            let theta = std::f64::consts::PI * (r + 1) as f64 / (rings + 1) as f64;
            let (sin, cos) = theta.sin_cos();
            let ring_radius = desc.radius * sin;
            for s in 0..segments {
                let phi = std::f64::consts::TAU * s as f64 / segments as f64;
                let p = Vector::new(
                    desc.center.x + ring_radius * phi.cos(),
                    desc.center.y + desc.radius * cos,
                    desc.center.z + ring_radius * phi.sin(),
                );
                let i = body.add_particle(p);
                body.particles[i].inv_mass = inv_mass;
            }
        }
        let top = body.add_particle(Vector::new(
            desc.center.x,
            desc.center.y + desc.radius,
            desc.center.z,
        ));
        body.particles[top].inv_mass = inv_mass;
        let bottom = body.add_particle(Vector::new(
            desc.center.x,
            desc.center.y - desc.radius,
            desc.center.z,
        ));
        body.particles[bottom].inv_mass = inv_mass;

        // Quads between adjacent rings, split into two triangles each (the
        // shared diagonal becomes a constraint automatically), then pole caps.
        for r in 0..rings - 1 {
            for s in 0..segments {
                let s_next = (s + 1) % segments;
                let a = shell_index(r, s, segments);
                let b = shell_index(r, s_next, segments);
                let c = shell_index(r + 1, s_next, segments);
                let d = shell_index(r + 1, s, segments);
                body.add_triangle([a as u32, b as u32, c as u32]);
                body.add_triangle([a as u32, c as u32, d as u32]);
            }
        }
        for s in 0..segments {
            let s_next = (s + 1) % segments;
            body.add_triangle([
                top as u32,
                shell_index(0, s_next, segments) as u32,
                shell_index(0, s, segments) as u32,
            ]);
            body.add_triangle([
                bottom as u32,
                shell_index(rings - 1, s, segments) as u32,
                shell_index(rings - 1, s_next, segments) as u32,
            ]);
        }

        // Uniform shell softness on the tension side of every edge.
        for c in body.distance_constraints.iter_mut() {
            c.compliance = desc.edge_compliance;
        }

        // Pressure drives the XPBD predict step (Phase 11); distance
        // constraints are only projected by the XPBD solver.
        body.solver = SoftSolver::Xpbd {
            iterations: desc.iterations,
            compliance: desc.edge_compliance,
        };
        if desc.pressure > 0.0 {
            body.pressure = Some(desc.pressure);
        }

        clear_error();
        world.inner.soft_bodies.insert(body).0
    })
}
