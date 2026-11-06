//! Soft-body (deformable body) support — Phase 0 foundation.
//!
//! This module is the **data-structure + free-particle integrator** skeleton for
//! soft-body simulation. It is intentionally independent of the SoA SIMD solver
//! boundary (`helpers.rs` / `worker.rs` / `generic_contact_constraint.rs` read
//! the `velocities` / `accelerations` nalgebra buffers) so it can be compiled and
//! unit-tested in isolation.
//!
//! ## Design notes (see `.hermes/plans/2026-08-24_soft-body-roadmap.md`)
//!
//! * A soft body is a cloud of point masses (`SoftParticle`) connected by
//!   Hookean springs (`Spring`). Springs carry `stiffness` (`k`) and `damping`
//!   (`c`); the spring force is `F = -k·(|x_b - x_a| - rest)·dir - c·(v_rel · dir)·dir`.
//! * Integration is **semi-implicit (symplectic) Euler**: velocities are updated
//!   first (`v += dt · M⁻¹ · f`), then positions (`x += dt · v`). This matches the
//!   operation order used by the rigid-body integrator, and keeps the floating-point
//!   sequence bit-identical across runs under `enhanced-determinism`.
//! * The internal spring/damping forces are the natural payload for the existing
//!   `force_containers` `ForceKind::Custom(Persistent)` model (Phase 2 will route
//!   them there); Phase 0 keeps them local so the numerics can be tested directly.
//! * All math uses `crate::math::Vector` (glam-backed) and `Real` so the module
//!   stays aligned with the rest of the fork's vector conventions.
//!
//! Phase 0a (this file) = data structures + integrator + tests. Phase 0b (later)
//! wires `SoftBodySet` into `World` / `PersistentIslands`. Phase 1+ add joint-based
//! and mass-spring/FEM coupling.

use rapier3d::math::{AngVector, Real, Vector};
use rapier3d::prelude::{RigidBodyHandle, RigidBodySet};
use std::collections::{HashMap, HashSet};
use std::vec::Vec;

/// `ForceKind::Custom` discriminator for soft-body internal spring/damping forces.
/// Routed through `force_containers` so they share the same `Persistent` lifecycle
/// and `compute_body_effective_forces` summation as gravity/thrust (Phase 2).
pub const SOFT_SPRING_CUSTOM_ID: u32 = 0x5_042; // "SB" encoded; arbitrary custom tag

/// Opaque id of a [`SoftBody`] inside a [`SoftBodySet`].
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct SoftBodyId(pub u32);

/// A single point mass in a soft body.
#[derive(Clone, Debug)]
pub struct SoftParticle {
    /// Current world-space position.
    pub pos: Vector,
    /// Current world-space linear velocity.
    pub vel: Vector,
    /// Accumulated force for the current substep (cleared by `compute_forces`).
    pub force: Vector,
    /// Inverse mass (`0` = pinned / immovable). `mass = 1 / inv_mass`.
    pub inv_mass: Real,
    /// Optional rigid body this particle is bound to. When set, the particle's
    /// internal spring/damping force is routed into that body's `force_containers`
    /// (Phase 2), so the soft body drives the rigid body through the standard
    /// effective-force path rather than integrating the particle directly.
    pub bound_body: Option<RigidBodyHandle>,
    /// Phase 8: when `bound_body` is `Some`, this is the attachment point
    /// expressed in the bound body's *local* frame (so the particle rigidly
    /// follows the body as it translates/rotates). Computed at attach time from
    /// the world-space attach point; ignored when `bound_body` is `None`.
    pub bound_local: Vector,
}

impl SoftParticle {
    /// Creates a free (movable) particle with unit mass.
    pub fn new(pos: Vector) -> Self {
        Self {
            pos,
            vel: Vector::ZERO,
            force: Vector::ZERO,
            inv_mass: 1.0,
            bound_body: None,
            bound_local: Vector::ZERO,
        }
    }

    /// Creates a pinned (immovable) particle — its `inv_mass` is `0`.
    pub fn pinned(pos: Vector) -> Self {
        Self {
            pos,
            vel: Vector::ZERO,
            force: Vector::ZERO,
            inv_mass: 0.0,
            bound_body: None,
            bound_local: Vector::ZERO,
        }
    }

    /// Mass of the particle; `0` for pinned particles.
    #[inline]
    pub fn mass(&self) -> Real {
        if self.inv_mass == 0.0 {
            0.0
        } else {
            1.0 / self.inv_mass
        }
    }
}

/// A Hookean spring connecting two particles, with linear damping along its axis.
#[derive(Clone, Copy, Debug)]
pub struct Spring {
    /// Index of the first endpoint particle within the parent [`SoftBody`].
    pub a: usize,
    /// Index of the second endpoint particle within the parent [`SoftBody`].
    pub b: usize,
    /// Rest length (natural length) of the spring.
    pub rest_length: Real,
    /// Spring constant `k` (stiffness).
    pub stiffness: Real,
    /// Damping coefficient `c` (axial velocity damping).
    pub damping: Real,
    /// Phase 31 — active-strain activation `γ ∈ [0, 1]`. Scales the effective rest
    /// length toward zero (`rest_eff = rest_length * (1 - activation)`), pulling the
    /// two endpoints together like a contracting muscle. `0` (default) = passive.
    pub activation: Real,
    /// Phase 32 — muscle-fibre direction (unit vector). When `Some(dir)`, the active
    /// contraction is oriented along `dir` instead of purely along the edge — i.e. the
    /// spring acts as a muscle fibre with a defined orientation (anisotropic drive).
    /// `None` (default) = contract along the edge (degenerates to Phase 31 behaviour).
    pub fibre: Option<Vector>,
}

/// A distance constraint (XPBD) between two particles — the edge of a deformable
/// mesh. Unlike [`Spring`] (a force), this is a *position* constraint solved by
/// XPBD's small-iteration projection, which is unconditionally stable even for
/// stiff/rigid edges (no explicit-spring explosion).
#[derive(Clone, Copy, Debug)]
pub struct DistanceConstraint {
    /// First endpoint particle index.
    pub a: usize,
    /// Second endpoint particle index.
    pub b: usize,
    /// Rest length.
    pub rest: Real,
    /// XPBD stretch compliance `α_s` (0 = rigid, > 0 = soft), applied when the edge
    /// is **longer** than `rest` (tension). Stored per-constraint so different edges
    /// can have different stiffness. Phase 19: this is the *stretch* compliance; the
    /// *compression* compliance lives in [`Self::compression`], enabling anisotropic
    /// behaviour (e.g. cloth resists stretch but folds easily under compression).
    pub compliance: Real,
    /// Phase 19 — XPBD compression compliance `α_c` (0 = rigid, > 0 = soft), applied
    /// when the edge is **shorter** than `rest` (compression). When equal to
    /// [`Self::compliance`] the edge is isotropic. Initialized to the stretch
    /// compliance by every constructor (`add_distance_constraint`, `add_triangle`,
    /// `add_bending_constraint`) so existing bodies stay isotropic unless the caller
    /// opts into anisotropy via `set_distance_constraint_compression`.
    pub compression: Real,
    /// Phase 31 — active-strain activation `γ ∈ [0, 1]`. Scales the effective rest
    /// length toward zero (`rest_eff = rest * (1 - activation)`), contracting the
    /// edge like a muscle fibre. `0` (default) = passive.
    pub activation: Real,
    /// Phase 32 — muscle-fibre direction (unit vector). See [`Spring::fibre`]; when
    /// `Some(dir)` the active contraction is oriented along `dir` rather than purely
    /// along the edge. `None` (default) = contract along the edge.
    pub fibre: Option<Vector>,
}

/// Phase 27 — fracture-mechanics tearing criterion.
///
/// Replaces the old single strain-threshold with three physically-motivated
/// fracture modes. Each edge (XPBD [`DistanceConstraint`] or MassSpring
/// [`Spring`]) is evaluated against the chosen scalar; when the scalar exceeds
/// the threshold the edge snaps (and any triangle losing a structural edge is
/// dropped too).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TearCriterion {
    /// Classic strain limit: snaps when `(|len| − rest)/rest > threshold`.
    /// Threshold is dimensionless (e.g. `0.5` = break past 50% stretch).
    Strain(Real),
    /// Griffith-style stress limit: snaps when the axial force magnitude
    /// `|k·(len − rest)|` exceeds `threshold` (force units). `k` is the spring
    /// stiffness, or `1/(compliance + ε)` for an XPBD distance constraint.
    Stress(Real),
    /// Energy-release-rate limit: snaps when the elastic strain energy
    /// `½·k·(len − rest)²` exceeds `threshold` (energy units). A proxy for the
    /// fracture toughness / critical energy release rate of the material.
    Energy(Real),
}

/// Which integrator a [`SoftBody`] uses.
///
/// * `MassSpring` — the Phase 0/2 Hookean-spring + semi-implicit Euler path.
/// * `Xpbd { iterations, compliance }` — Phase 3 position-based dynamics: edges
///   become [`DistanceConstraint`]s and tetrahedra carry a volume constraint,
///   both projected with a fixed number of Gauss-Seidel iterations. `compliance`
///   is the default used when a constraint is added without an explicit one.
#[derive(Clone, Copy, Debug, Default)]
pub enum SoftSolver {
    /// Hookean springs (default, Phase 0/2).
    #[default]
    MassSpring,
    /// XPBD position-based solver (Phase 3).
    Xpbd {
        /// Gauss-Seidel projection iterations per substep.
        iterations: u32,
        /// Default XPBD compliance for constraints created without an explicit one.
        compliance: Real,
    },
}

/// A uniform wind / air-resistance field applied to every free particle of a
/// soft body (Phase 7). It is a *pure external force* — no new mechanics, it
/// reuses the same force path as gravity:
///
/// * `accel` — constant wind acceleration (a directional push, like a sideways
///   gravity). Makes a pinned-edge cloth fly out like a flag.
/// * `drag` — linear air-resistance coefficient. Each free particle feels
///   `F = m·accel − m·drag·v`, i.e. a velocity damping toward the wind state.
///   Keep `drag·dt < 1` for stability (the integration clamp handles it too).
#[derive(Clone, Copy, Debug)]
pub struct Wind {
    /// Constant wind acceleration (`m/s²`), applied to every free particle.
    pub accel: Vector,
    /// Linear air-resistance coefficient (`1/s`). `F_drag = -m·drag·v`.
    pub drag: Real,
}

/// A soft body: a collection of point masses connected by springs.
///
/// In Phase 0/2 it is a mass-spring cloud. Phase 3 adds an optional XPBD
/// position-based solver: edges become [`DistanceConstraint`]s and tetrahedra
/// carry a volume constraint, projected each substep. The `solver` field selects
/// which path [`SoftBody::step`] takes.
#[derive(Clone, Debug)]
pub struct SoftBody {
    /// Point masses. Spring endpoints / constraint indices index into this `Vec`.
    pub particles: Vec<SoftParticle>,
    /// Springs (edges) between particles — used by the `MassSpring` solver.
    pub springs: Vec<Spring>,
    /// Distance constraints (edges) — used by the `Xpbd` solver.
    pub distance_constraints: Vec<DistanceConstraint>,
    /// Tetrahedral volume elements. Each entry is `[a, b, c, d]` particle indices.
    /// Used by the `Xpbd` solver's volume-preservation constraint.
    pub tetrahedra: Vec<[u32; 4]>,
    /// Rest (reference) signed volume of each tetrahedron, precomputed at
    /// `add_tetrahedron` time. Indexed parallel to `tetrahedra`.
    pub tetra_rest_volumes: Vec<Real>,
    /// Triangular faces (cloth / shell topology). Each entry is `[a, b, c]`
    /// particle indices, CCW for outward normal. Phase 6: cloth soft bodies are
    /// built from triangles; the structural edges are added automatically as
    /// distance constraints (see `add_triangle`), so the XPBD solver needs no new
    /// mechanics — bending is just extra distance constraints between opposite
    /// vertices of adjacent quads (composed by the caller).
    pub triangles: Vec<[u32; 3]>,
    /// Active integrator.
    pub solver: SoftSolver,
    /// Constant acceleration applied to every free particle (typically gravity).
    pub gravity: Vector,
    /// Coarse sleeping flag (island-style). When `true`, [`SoftBody::step`] is a
    /// no-op: the whole body is treated as an inactive unit, mirroring how
    /// `PersistentIslands` keeps a sleeping rigid-body island from being
    /// re-simulated. Per-particle island membership is Phase 3; this flag gives
    /// the same "skip inactive work" behavior at body granularity for now.
    pub sleeping: bool,
    /// Phase 5f: collision coupling flag. When `true` the soft body's particles
    /// are driven by external proxy rigid bodies (one `Ball` collider per free
    /// particle, maintained by the mps-core integration layer), so [`SoftBody::step`]
    /// must NOT integrate the particles itself — their positions/velocities are
    /// written back from the proxy bodies after the rigid-body narrow-phase/contact
    /// step. Forces (springs + gravity) are still computed and exported by the
    /// integration layer. Defaults to `false`.
    pub collide: bool,
    /// Phase 5f: proxy collider radius used when `collide` is enabled. Each free
    /// particle gets a `Ball` collider of this radius. Defaults to `0.1`.
    pub particle_radius: Real,
    /// Phase 7: uniform wind / air-resistance field. `None` = no wind. When set,
    /// every free particle feels `F = m·wind.accel − m·wind.drag·v` in addition to
    /// gravity — a pure external force, no new solver mechanics. Applied in both
    /// the `MassSpring` (`compute_forces`) and `Xpbd` (`step_xpbd` predict) paths.
    pub wind: Option<Wind>,
    /// Phase 11: uniform internal pressure (`P`, force/area). When `Some`, every
    /// free particle of a closed triangular mesh gets an outward push along the
    /// surface normal, `F = P · area` per incident triangle (balloon / gas-law
    /// model). A pure external force mirroring [`Self::wind`]; applied in both the
    /// MassSpring (`compute_forces`) and XPBD (`step_xpbd` predict) paths. `None`
    /// (default) = no pressure. Closed manifold (`self.triangles`) needed for a
    /// real balloon; an open sheet just bulges along single-sided normals.
    pub pressure: Option<Real>,
    /// Phase 27: optional viscoelastic (rate-dependent) constitutive model. `None` = purely elastic.
    pub viscoelastic: Option<ViscoelasticParams>,
    /// Phase 27: optional uniform thermal field. `None` = isothermal.
    pub temperature: Option<ThermalParams>,
    /// Phase 27: optional body-level orthotropic stiffness axes. When `Some(v)`
    /// (each component ≥ 0), an edge's effective XPBD compliance is divided by the
    /// projection `nᵀ·diag(v)·n` where `n` is the edge unit direction — so an edge
    /// aligned with the x-axis uses compliance `α / v.x` (stiffer if `v.x > 1`).
    /// This adds directional (orthotropic) material response on top of the
    /// per-edge stretch/compression anisotropies ([`DistanceConstraint::compliance`
    /// and `::compression`]); `None` (default) keeps every edge isotropic.
    pub anisotropy: Option<Vector>,
    /// Phase 18: global internal (structural) damping. Each step, every free
    /// particle's velocity is scaled by `1 − damping` (after the solver recovers
    /// velocities), giving a body-wide "jelly / slime" energy loss that is
    /// orthogonal to the per-spring axial `damping` (Phase 0) and to the
    /// distance-solver `compliance` (Phase 13). `0` (default) = no damping (energy
    /// conserving for the internal modes); larger values settle oscillation / jitter
    /// faster. Clamped to `[0, 1)` — `1` would fully freeze motion, so it is rejected.
    /// Applied in both the MassSpring (`integrate`) and XPBD (`step_xpbd` velocity
    /// recovery) paths. No new solver mechanics, no SoA interaction.
    pub damping: Real,
    /// Phase 24: XPBD/MassSpring substeps per [`SoftBody::step`] call. Splits the
    /// frame `dt` into `substeps` equal slices and runs the active solver once per
    /// slice (so constraint projection happens at a finer time resolution). `1`
    /// reproduces the historic single-substep behaviour. Larger values make stiff
    /// materials / high compliance converge faster and stay stable; they cost
    /// `substeps`× the per-step work. Mirrors `World::integration_parameters`
    /// substepping for the rigid side — same idea, local to each soft body.
    pub substeps: u32,
    /// Phase 9: tearing threshold. When `Some(ε)`, any structural edge (XPBD
    /// distance constraint or MassSpring spring) whose strain `(|len| − rest)/rest`
    /// exceeds `ε` is removed at the start of each [`SoftBody::step`]. Triangular
    /// faces that lose any structural edge are dropped too, so a torn cloth stops
    /// rendering the broken face. `None` (default) = no tearing. Pure topology
    /// edit — no new solver mechanics, no SoA interaction.
    pub tear: Option<TearCriterion>,
    /// Phase 10: plasticity (permanent deformation, like putty / memory foam).
    /// When `Some(params)`, any structural edge whose elastic strain magnitude
    /// exceeds `params.yield_strain` has its rest length permanently shifted
    /// toward the current length by `params.creep` (clamped to `[0,1]`) each step,
    /// so the deformation "freezes in" instead of springing back. `None` (default)
    /// = perfectly elastic (Hookean). Pure rest-length edit — no new solver
    /// mechanics, no SoA interaction.
    pub plasticity: Option<PlasticityParams>,
    /// Phase 12: self-collision. When `Some(params)`, the body's free particles
    /// repel each other when their centres come within `2·params.radius` (each
    /// particle behaves as a sphere of that radius). Broad-phase uses a uniform
    /// spatial hash; detected pairs are solved as stiff XPBD distance constraints
    /// (rest = `2·radius`, compliance = `params.stiffness`) every solver iteration,
    /// in both the MassSpring and XPBD paths. Direct structural neighbours (linked
    /// by a [`SoftBody::distance_constraints`] edge) are excluded so existing
    /// springs/cloth edges are not treated as collisions. `None` (default) = off.
    /// Pure positional projection — no new solver mechanics, no SoA interaction.
    pub self_collision: Option<SelfCollisionParams>,
    /// Phase 14: soft-soft (cross-body) collision. When `Some(params)`, this body
    /// collides with *other* soft bodies that also have `cross_collision` set: their
    /// free particles repel when centres come within `2·min(radius_a, radius_b)`.
    /// Reuses the same spatial-hash broad-phase + XPBD push-apart as self-collision,
    /// but the world-level pass runs over pairs of bodies. `None` (default) = off.
    /// Pure positional projection — no new solver mechanics, no SoA interaction.
    pub cross_collision: Option<SelfCollisionParams>,
    /// Phase 16: dedicated volume-conservation compliance for the tetrahedral
    /// volume constraints. When `Some(c)`, every tetra volume constraint in
    /// `step_xpbd` is solved with `α̃ = c / dt²` — *independent* of the distance
    /// solver's compliance. This makes it possible to have soft edges but a hard
    /// (incompressible) blob, or to keep volume conserved even when the distance
    /// solver is soft. It is orthogonal to Phase 11 pressure (an outward force):
    /// pressure inflates, this constraint holds the total volume. `None` (default)
    /// = fall back to the global solver compliance for tetra volume (existing
    /// behaviour). Pure positional constraint — no new solver mechanics.
    pub volume_conservation: Option<Real>,
    /// Phase 29: corotated linear elasticity (per-tetrahedron shape matching).
    /// When `Some(stiffness)`, each XPBD iteration additionally projects every
    /// tetrahedron toward its best-fit-rotated rest shape (polar-decomposition
    /// shape matching), giving rotation-invariant linear elasticity on top of
    /// the existing volume constraint. `stiffness` in `(0, 1]` is the per-iteration
    /// relaxation factor. `None` (default) = off.
    pub corotated: Option<Real>,
    /// Phase 30: Neo-Hookean logarithmic volume energy. When `Some(stiffness)`
    /// and XPBD is active, each tetrahedron's volume constraint uses the
    /// nonlinear residual `C = ln(J)` (`J = V/V₀`, clamped to a small positive
    /// floor) and a `stiffness/dt²` compliance, replacing the linear
    /// `V − V₀` residual + `volume_conservation` compliance for that tet.
    /// The logarithmic form makes the volumetric resistance grow unboundedly as
    /// the tet collapses (physically correct incompressibility) instead of the
    /// linear response's finite push-back. `None` (default) = linear volume.
    pub neo_hookean: Option<Real>,
    /// Phase 29: inverse rest shape matrix per tetrahedron (columns are the rest
    /// edge vectors of the reference configuration). Built by
    /// [`SoftBody::set_corotated`] from the particle positions at enable time.
    /// Indexed parallel to `tetrahedra`; shorter than `tetrahedra` after a later
    /// `subdivide_tetrahedra` (extra tets are skipped, not corrupted).
    pub tetra_rest_shapes: Vec<[[Real; 3]; 3]>,
    /// Phase 17: cohesion (adhesion / breakable glue) between this body and *other*
    /// bodies. When `Some(p)`, free particles of this body within `p.radius` of a
    /// free particle of another cohesion-enabled body attract toward contact, bonding
    /// the bodies (the dual of Phase 9 tearing). Bonds break when pulled apart beyond
    /// `p.break_distance`. Solved at world level by `solve_cohesion`. `None` (default)
    /// = off.
    pub cohesion: Option<CohesionParams>,
}
/// Phase 10: plasticity parameters (see [`SoftBody::plasticity`]).
#[derive(Clone, Copy, Debug)]
pub struct PlasticityParams {
    /// Yield strain: elastic deformation below this magnitude is fully recovered;
    /// above it, the excess becomes permanent. Must be ≥ 0.
    pub yield_strain: Real,
    /// Creep rate in `[0,1]`: fraction of the over-yield strain that is transferred
    /// from elastic to plastic (rest-length) each step. `1` = instantly frozen at
    /// the yield surface; `0` = no plasticity (elastic).
    pub creep: Real,
}

/// Phase 27: viscoelastic (rate-dependent) constitutive parameters.
///
/// Implements a Kelvin-Voigt-style **strain-rate hardening**: the effective spring
/// stiffness grows with the magnitude of the local stretch rate, so a rapidly
/// stretched edge resists more than a slowly stretched one (polymer / viscoelastic
/// behaviour). `rate_coefficient >= 0`; `0` (or `None`) is pure elasticity.
#[derive(Clone, Copy, Debug)]
pub struct ViscoelasticParams {
    /// Strain-rate stiffening gain. `k_eff = k·(1 + rate_coefficient·|d(strain)/dt|)`.
    pub rate_coefficient: Real,
}

/// Phase 27: thermal field parameters.
///
/// A uniform body temperature `temp` (relative to `ambient`) modulates the material:
/// rest lengths expand by `expansion·ΔT` (thermal expansion) and stiffness softens by
/// `stiffness_temp_coeff·ΔT` (temperature-dependent modulus). Both effects are applied
/// to every spring/constraint, giving a temperature-dependent soft body.
#[derive(Clone, Copy, Debug)]
pub struct ThermalParams {
    /// Current uniform body temperature.
    pub temp: Real,
    /// Reference (ambient) temperature the material was characterised at.
    pub ambient: Real,
    /// Linear thermal-expansion coefficient (rest-length change per unit ΔT).
    pub expansion: Real,
    /// Stiffness temperature coefficient (modulus drop per unit ΔT; clamped so the
    /// effective stiffness stays > 0).
    pub stiffness_temp_coeff: Real,
}

/// Phase 12: self-collision parameters (see [`SoftBody::self_collision`]).
#[derive(Clone, Copy, Debug)]
pub struct SelfCollisionParams {
    /// Particle collision radius. Two *free* particles whose centres come within
    /// `2·radius` are pushed apart (each treated as a sphere of this radius). Should
    /// match `SoftBody::particle_radius` used by the proxy-collider path, but is
    /// independent here so self-collision works without rigin-body coupling. Must be > 0.
    pub radius: Real,
    /// XPBD compliance of the repulsion constraint. `0` = perfectly hard (rigid
    /// non-penetration); larger values allow softer, springier contact. Must be ≥ 0.
    pub stiffness: Real,
    /// Phase 20: contact friction coefficient for the tangential relative slip at a
    /// soft-soft contact (self-collision and, via the shared struct, cross-collision).
    /// `None` = frictionless (default). When set, the tangential relative velocity of a
    /// contacting pair is damped by `μ` each step (Coulomb-style, bounded to `[0,1]`:
    /// `μ = 0` no friction, `μ = 1` fully kills tangential slip). Must be `0 ≤ μ ≤ 1`.
    pub friction: Option<Real>,
}

/// Phase 17: cohesion (adhesion / breakable glue) parameters for inter-body
/// (soft-soft) contact — the dual of Phase 9 tearing. Two *free* particles from
/// *different* bodies whose centres come within `radius` attract toward contact
/// (rest distance `radius`), bonding the bodies together like glue. The bond is
/// *breakable*: if the pair separation ever exceeds `break_distance` (which is
/// usually `> radius`, giving a hysteresis so bonded pairs need to be pulled apart
/// to break), the attraction is released for that pair for the rest of the step —
/// i.e. the glue tears. Each step is stateless: bonds are re-evaluated from the
/// current geometry, so it composes naturally with Phase 14 cross-collision.
#[derive(Clone, Copy, Debug)]
pub struct CohesionParams {
    /// Capture radius: a free particle from another body within this distance is
    /// attracted and bonded. Should match `SoftBody::particle_radius` conceptually.
    /// Must be > 0.
    pub radius: Real,
    /// XPBD compliance of the attraction constraint. `0` = hard glue (bonded pairs
    /// snap to exactly `radius` apart); larger values give springier, stretchier
    /// glue. Must be ≥ 0.
    pub stiffness: Real,
    /// Break distance: once a bonded pair is pulled apart beyond this separation the
    /// attraction releases (the glue tears). Must be `> radius`. `inf` disables
    /// breaking (permanent glue).
    pub break_distance: Real,
}

impl SoftBody {
    /// Creates an empty soft body in a gravity field `gravity`.
    pub fn new(gravity: Vector) -> Self {
        Self {
            particles: Vec::new(),
            springs: Vec::new(),
            distance_constraints: Vec::new(),
            tetrahedra: Vec::new(),
            tetra_rest_volumes: Vec::new(),
            triangles: Vec::new(),
            solver: SoftSolver::MassSpring,
            gravity,
            sleeping: false,
            collide: false,
            particle_radius: 0.1,
            wind: None,
            tear: None,
            plasticity: None,
            pressure: None,
            anisotropy: None,
            viscoelastic: None,
            temperature: None,
            damping: 0.0,
            self_collision: None,
            cross_collision: None,
            volume_conservation: None,
            corotated: None,
            neo_hookean: None,
            tetra_rest_shapes: Vec::new(),
            cohesion: None,
            substeps: 1,
        }
    }

    /// Adds a free particle and returns its index.
    pub fn add_particle(&mut self, pos: Vector) -> usize {
        let idx = self.particles.len();
        self.particles.push(SoftParticle::new(pos));
        idx
    }

    /// Adds a pinned (immovable) particle and returns its index.
    pub fn add_pinned(&mut self, pos: Vector) -> usize {
        let idx = self.particles.len();
        self.particles.push(SoftParticle::pinned(pos));
        idx
    }

    /// Phase 8: anchors `particle` to a rigid body `body`, so it rigidly follows
    /// that body's motion. `world_attach_point` is the world-space point where the
    /// particle binds (usually the particle's current position). The point is
    /// stored in the body's *local* frame so the particle tracks translation and
    /// rotation. The particle stops integrating locally; its spring/damping force
    /// is instead routed to the body via [`SoftBodySet::write_spring_forces`].
    ///
    /// Returns `false` if `particle` is out of range or `body` is not in `bodies`.
    pub fn attach_particle(
        &mut self,
        particle: usize,
        body: RigidBodyHandle,
        world_attach_point: Vector,
        bodies: &RigidBodySet,
    ) -> bool {
        let Some(rb) = bodies.get(body) else {
            return false;
        };
        let local = rb.position().inverse_transform_point(world_attach_point);
        let Some(p) = self.particles.get_mut(particle) else {
            return false;
        };
        p.bound_body = Some(body);
        p.bound_local = local;
        true
    }

    /// Phase 8: detaches `particle` from any bound rigid body (returns it to a
    /// free, locally-integrated particle). No-op if already free.
    pub fn detach_particle(&mut self, particle: usize) -> bool {
        let Some(p) = self.particles.get_mut(particle) else {
            return false;
        };
        p.bound_body = None;
        p.bound_local = Vector::ZERO;
        true
    }

    /// Phase 7: enables a uniform wind / air-resistance field for this body.
    /// `accel` is a constant wind acceleration applied to every free particle
    /// (like a sideways gravity); `drag` is a linear air-resistance coefficient
    /// (`F_drag = −m·drag·v`). See [`Wind`]. Pass `accel = ZERO, drag = 0` to
    /// get the same effect as [`Self::clear_wind`].
    pub fn apply_wind(&mut self, accel: Vector, drag: Real) {
        self.wind = Some(Wind { accel, drag });
    }

    /// Phase 7: disables the wind field (`None`).
    pub fn clear_wind(&mut self) {
        self.wind = None;
    }

    /// Phase 11: enables a uniform internal pressure `P` (force/area). When `P > 0`,
    /// every free particle of a closed triangular mesh is pushed outward along the
    /// surface normal with force `F = P · area` per incident triangle (the balloon /
    /// gas-law model). Pass `P <= 0` (or [`Self::clear_pressure`]) to disable.
    pub fn set_pressure(&mut self, pressure: Real) {
        if pressure > 0.0 {
            self.pressure = Some(pressure);
        } else {
            self.pressure = None;
        }
    }

    /// Phase 11: disables internal pressure (`None`).
    pub fn clear_pressure(&mut self) {
        self.pressure = None;
    }

    pub fn set_self_collision(&mut self, radius: Real, stiffness: Real) -> bool {
        if !(radius > 0.0) || stiffness < 0.0 {
            return false;
        }
        self.self_collision = Some(SelfCollisionParams {
            radius,
            stiffness,
            friction: None,
        });
        true
    }

    /// Phase 20: sets the contact friction coefficient `μ` for self-collision.
    /// Requires `self_collision` to be enabled first. Rejects non-finite or out-of-range
    /// `μ` (`0 ≤ μ ≤ 1`). `clear_self_collision` resets it to `None` (frictionless).
    pub fn set_self_collision_friction(&mut self, mu: Real) -> bool {
        if !mu.is_finite() || mu < 0.0 || mu > 1.0 {
            return false;
        }
        let Some(p) = self.self_collision.as_mut() else {
            return false;
        };
        p.friction = Some(mu);
        true
    }

    /// Phase 12: disables self-collision (`None`).
    pub fn clear_self_collision(&mut self) {
        self.self_collision = None;
    }

    /// Phase 14: enables soft-soft (cross-body) collision with the given `radius`
    /// (particle sphere radius) and `stiffness` (XPBD compliance of the repulsion
    /// constraint, `0` = hard). Two bodies only collide if *both* have
    /// `cross_collision` set; the effective repulsion distance is
    /// `2·min(radius_a, radius_b)`. Rejects non-positive `radius` or negative
    /// `stiffness`.
    pub fn set_cross_collision(&mut self, radius: Real, stiffness: Real) -> bool {
        if !(radius > 0.0) || stiffness < 0.0 {
            return false;
        }
        self.cross_collision = Some(SelfCollisionParams {
            radius,
            stiffness,
            friction: None,
        });
        true
    }

    /// Phase 20: sets the contact friction coefficient `μ` for cross-collision.
    /// Requires `cross_collision` to be enabled first. Rejects non-finite or out-of-range
    /// `μ` (`0 ≤ μ ≤ 1`). `clear_cross_collision` resets it to `None` (frictionless).
    pub fn set_cross_collision_friction(&mut self, mu: Real) -> bool {
        if !mu.is_finite() || mu < 0.0 || mu > 1.0 {
            return false;
        }
        let Some(p) = self.cross_collision.as_mut() else {
            return false;
        };
        p.friction = Some(mu);
        true
    }

    /// Phase 14: disables soft-soft (cross-body) collision (`None`).
    pub fn clear_cross_collision(&mut self) {
        self.cross_collision = None;
    }

    /// Phase 16: enables the dedicated volume-conservation constraint with the given
    /// `compliance` (`c`). Each tetra volume constraint in `step_xpbd` is then solved
    /// with `α̃ = c / dt²`, independent of the distance solver's compliance. `c == 0`
    /// gives a hard (incompressible) blob. Returns `false` (and does nothing) on a
    /// non-finite or negative `compliance`.
    pub fn set_volume_conservation(&mut self, compliance: Real) -> bool {
        if !compliance.is_finite() || compliance < 0.0 {
            return false;
        }
        self.volume_conservation = Some(compliance);
        true
    }

    /// Phase 16: disables the dedicated volume-conservation constraint (`None`),
    /// reverting tetra volume to the global solver compliance.
    pub fn clear_volume_conservation(&mut self) {
        self.volume_conservation = None;
    }
    /// Phase 29: enables corotated linear elasticity with the given per-iteration
    /// relaxation `stiffness` (`(0, 1]`). The rest shape of every *current*
    /// tetrahedron is snapshotted into `tetra_rest_shapes` from the particle
    /// positions at call time (enable on the undeformed mesh). Degenerate
    /// tetrahedra are skipped (their rest shape stays the zero matrix). Returns
    /// `false` for non-finite / out-of-range `stiffness` or an empty body.
    pub fn set_corotated(&mut self, stiffness: Real) -> bool {
        if !stiffness.is_finite() || stiffness <= 0.0 || stiffness > 1.0 {
            return false;
        }
        let mut shapes = Vec::with_capacity(self.tetrahedra.len());
        for tet in &self.tetrahedra {
            let [a, b, c, d] = *tet;
            let m = rest_shape_matrix(
                self.particles[a as usize].pos,
                self.particles[b as usize].pos,
                self.particles[c as usize].pos,
                self.particles[d as usize].pos,
            );
            shapes.push(m);
        }
        self.tetra_rest_shapes = shapes;
        self.corotated = Some(stiffness);
        true
    }

    /// Phase 29: disables corotated elasticity (`None`).
    pub fn clear_corotated(&mut self) {
        self.corotated = None;
    }
    /// Phase 30: enables the Neo-Hookean logarithmic volume energy with the
    /// given volumetric `stiffness` (compliance = `stiffness/dt²`; larger =
    /// closer to incompressible). Returns `false` for non-finite values or
    /// `stiffness < 0`.
    pub fn set_neo_hookean(&mut self, stiffness: Real) -> bool {
        if !stiffness.is_finite() || stiffness < 0.0 {
            return false;
        }
        self.neo_hookean = Some(stiffness);
        true
    }

    /// Phase 30: disables the Neo-Hookean volume energy (back to linear).
    pub fn clear_neo_hookean(&mut self) {
        self.neo_hookean = None;
    }

    /// Phase 17: enables cohesion (adhesion / breakable glue) toward other bodies with
    /// the given `radius`, `stiffness` (compliance of the attraction constraint) and
    /// `break_distance` (separation at which a bond tears). Returns `false` (and does
    /// nothing) if `radius <= 0`, `stiffness < 0`, `break_distance <= radius`, or
    /// `break_distance`/`radius`/`stiffness` is `NaN`. `break_distance == +inf` is
    /// explicitly allowed (permanent, unbreakable glue).
    pub fn set_cohesion(&mut self, radius: Real, stiffness: Real, break_distance: Real) -> bool {
        if !radius.is_finite()
            || !stiffness.is_finite()
            || break_distance.is_nan()
            || !(radius > 0.0)
            || stiffness < 0.0
            || break_distance <= radius
        {
            return false;
        }
        self.cohesion = Some(CohesionParams {
            radius,
            stiffness,
            break_distance,
        });
        true
    }

    /// Phase 17: disables cohesion (`None`).
    pub fn clear_cohesion(&mut self) {
        self.cohesion = None;
    }

    /// Phase 18: sets the global internal (structural) damping coefficient `d`.
    /// Each step every free particle's velocity is scaled by `1 − d`. `0` = no
    /// damping; values in `[0, 1)` settle oscillation faster; `d >= 1` would fully
    /// freeze motion and is rejected (returns `false`). Non-finite `d` is rejected.
    pub fn set_damping(&mut self, d: Real) -> bool {
        if !d.is_finite() || d < 0.0 || d >= 1.0 {
            return false;
        }
        self.damping = d;
        true
    }

    /// Phase 24: set the number of solver substeps per [`SoftBody::step`] call.
    /// `n >= 1` splits the frame `dt` into `n` equal slices, projecting constraints
    /// at a finer time resolution. A value of `0` is rejected (kept at the previous
    /// setting) so a body never silently degrades to a no-op step. See the
    /// `substeps` field for the stability/convergence rationale.
    pub fn set_substeps(&mut self, n: u32) -> bool {
        if n == 0 {
            return false;
        }
        self.substeps = n;
        true
    }

    /// Phase 27: enables/disables the viscoelastic (strain-rate hardening) model.
    /// `None` = purely elastic. `rate_coefficient >= 0`; negative values are
    /// rejected (returns `false`).
    pub fn set_viscoelastic(&mut self, params: Option<ViscoelasticParams>) -> bool {
        match params {
            Some(p) if p.rate_coefficient >= 0.0 => {
                self.viscoelastic = Some(p);
                true
            }
            Some(_) => false,
            None => {
                self.viscoelastic = None;
                true
            }
        }
    }

    /// Phase 27: enables/disables the uniform thermal field. `None` = isothermal.
    /// All four components must be finite; `temp`/`ambient`/`expansion` unconstrained
    /// but `stiffness_temp_coeff·(temp − ambient)` must keep stiffness positive
    /// (i.e. `stiffness_temp_coeff·|ΔT| < 1`). Invalid input returns `false`.
    pub fn set_thermal(&mut self, params: Option<ThermalParams>) -> bool {
        match params {
            Some(p) => {
                let delta_t = p.temp - p.ambient;
                if !p.temp.is_finite()
                    || !p.ambient.is_finite()
                    || !p.expansion.is_finite()
                    || !p.stiffness_temp_coeff.is_finite()
                    || p.stiffness_temp_coeff * delta_t.abs() >= 1.0
                {
                    return false;
                }
                self.temperature = Some(p);
                true
            }
            None => {
                self.temperature = None;
                true
            }
        }
    }

    pub fn add_spring(
        &mut self,
        a: usize,
        b: usize,
        stiffness: Real,
        damping: Real,
    ) -> Option<usize> {
        let (pa, pb) = (self.particles.get(a)?, self.particles.get(b)?);
        let rest = (pb.pos - pa.pos).length();
        if rest == 0.0 {
            return None;
        }
        let idx = self.springs.len();
        self.springs.push(Spring {
            a,
            b,
            rest_length: rest,
            stiffness,
            damping,
            activation: 0.0,
            fibre: None,
        });
        Some(idx)
    }

    /// Computes the per-particle internal force from all springs (Hookean +
    /// axial damping), *without* gravity. Returns one `Vector` per particle.
    /// Pinned particles (`inv_mass == 0`) receive no force (they act as anchors).
    ///
    /// This is the force that Phase 2 routes into `force_containers` for bound
    /// particles; `compute_forces` reuses it and adds gravity for free particles.
    pub fn spring_damping_forces(&self) -> Vec<Vector> {
        let mut out = Vec::with_capacity(self.particles.len());
        for _ in 0..self.particles.len() {
            out.push(Vector::ZERO);
        }
        for s in &self.springs {
            let (pa_pos, pb_pos, pa_vel, pb_vel, pa_im, pb_im) =
                match (self.particles.get(s.a), self.particles.get(s.b)) {
                    (Some(a), Some(b)) => (a.pos, b.pos, a.vel, b.vel, a.inv_mass, b.inv_mass),
                    _ => continue,
                };
            let delta = pb_pos - pa_pos;
            let len = delta.length();
            if len == 0.0 {
                continue;
            }
            let dir = delta / len;
            // Phase 27 B5: thermal modulation of rest length + stiffness.
            let (rest_eff, mut k_eff) = if let Some(th) = self.temperature {
                let delta_t = th.temp - th.ambient;
                (
                    s.rest_length * (1.0 + th.expansion * delta_t),
                    s.stiffness * (1.0 - th.stiffness_temp_coeff * delta_t),
                )
            } else {
                (s.rest_length, s.stiffness)
            };
            // Phase 27 B4: viscoelastic strain-rate hardening.
            if let Some(ve) = self.viscoelastic {
                let strain_rate = (pb_vel - pa_vel).dot(dir) / len.max(1e-9);
                k_eff *= 1.0 + ve.rate_coefficient * strain_rate.abs();
            }
            // Phase 31: active-strain contraction — shrink the effective rest length
            // toward zero as activation rises, pulling the endpoints together.
            let rest_eff = rest_eff * (1.0 - s.activation.clamp(0.0, 1.0));
            let f_spring = k_eff * (len - rest_eff);
            let rel_vel = pb_vel - pa_vel;
            let f_damp = s.damping * rel_vel.dot(dir);
            let f_axial = f_spring + f_damp;
            // Phase 32: the active-strain drive follows the muscle-fibre direction
            // when one is set, otherwise the spring edge (Phase 31 default).
            let dir_active = match s.fibre {
                Some(fd) => fd,
                None => dir,
            };
            let f = dir_active * f_axial;
            if pa_im != 0.0 {
                out[s.a] += f;
            }
            if pb_im != 0.0 {
                out[s.b] -= f;
            }
        }
        out
    }

    pub fn kinetic_energy(&self) -> Real {
        self.particles
            .iter()
            .filter(|p| p.inv_mass != 0.0)
            .map(|p| 0.5 * p.mass() * p.vel.dot(p.vel))
            .fold(0.0, |acc, e| acc + e)
    }

    pub fn configure_xpbd(&mut self, iterations: u32, compliance: Real) {
        self.solver = SoftSolver::Xpbd {
            iterations,
            compliance,
        };
    }

    /// Adds a distance constraint between particles `a` and `b`. The rest length
    /// is taken from the current distance (0 is rejected to avoid a degenerate
    /// axis). Returns `None` if indices are out of bounds or coincide.
    pub fn add_distance_constraint(
        &mut self,
        a: usize,
        b: usize,
        compliance: Real,
    ) -> Option<usize> {
        let (pa, pb) = (self.particles.get(a)?, self.particles.get(b)?);
        let rest = (pb.pos - pa.pos).length();
        if rest == 0.0 {
            return None;
        }
        let idx = self.distance_constraints.len();
        self.distance_constraints.push(DistanceConstraint {
            a,
            b,
            rest,
            compliance,
            compression: compliance,
            activation: 0.0,
            fibre: None,
        });
        Some(idx)
    }

    /// Phase 13 — sets the `stiffness` (Hookean `k`) of an existing spring (by the
    /// index returned from `add_spring`) at runtime. Lets callers tune a body's
    /// material heterogeneity after construction (e.g. stiffen a "bone", loosen a
    /// "tendon") without rebuilding the topology. Returns `false` for an out-of-range
    /// index or a negative/non-finite stiffness.
    pub fn set_spring_stiffness(&mut self, index: usize, stiffness: Real) -> bool {
        if stiffness < 0.0 || !stiffness.is_finite() {
            return false;
        }
        match self.springs.get_mut(index) {
            Some(s) => {
                s.stiffness = stiffness;
                true
            }
            None => false,
        }
    }

    /// Phase 13 — sets the XPBD `compliance` (α) of an existing distance constraint
    /// (by the index returned from `add_distance_constraint`) at runtime. Per-constraint
    /// compliance is honored by the XPBD solver (see `step_xpbd`). Phase 19: this sets
    /// **both** the stretch and compression compliance to the same value, i.e. it keeps
    /// the edge isotropic. Use `set_distance_constraint_compression` to make it
    /// anisotropic (different stretch vs compression softness). Returns `false` for
    /// an out-of-range index or a negative/non-finite compliance.
    pub fn set_distance_constraint_compliance(&mut self, index: usize, compliance: Real) -> bool {
        if compliance < 0.0 || !compliance.is_finite() {
            return false;
        }
        match self.distance_constraints.get_mut(index) {
            Some(c) => {
                c.compliance = compliance;
                c.compression = compliance;
                true
            }
            None => false,
        }
    }

    /// Phase 19 — sets the XPBD **compression** compliance `α_c` of an existing
    /// distance constraint (by the index returned from `add_distance_constraint`)
    /// at runtime, independently of its stretch compliance. This is the anisotropic
    /// knob: a cloth edge can resist stretch (`compliance`, small) but fold/compress
    /// easily (`compression`, large). The solver selects the compliance by the
    /// current strain sign each iteration (see `step_xpbd`). Returns `false` for an
    /// out-of-range index or a negative/non-finite compliance.
    pub fn set_distance_constraint_compression(&mut self, index: usize, compression: Real) -> bool {
        if compression < 0.0 || !compression.is_finite() {
            return false;
        }
        match self.distance_constraints.get_mut(index) {
            Some(c) => {
                c.compression = compression;
                true
            }
            None => false,
        }
    }

    /// Adds a tetrahedral volume element `[a, b, c, d]`. The rest (reference)
    /// signed volume is computed from the current particle positions and cached.
    /// Returns `None` if any index is out of bounds or duplicated.
    pub fn add_tetrahedron(&mut self, tet: [u32; 4]) -> Option<usize> {
        let [a, b, c, d] = tet;
        let (pa, pb, pc, pd) = (
            self.particles.get(a as usize)?,
            self.particles.get(b as usize)?,
            self.particles.get(c as usize)?,
            self.particles.get(d as usize)?,
        );
        // Reject degenerate (duplicate) indices.
        if a == b || a == c || a == d || b == c || b == d || c == d {
            return None;
        }
        let vol = signed_tetra_volume(pa.pos, pb.pos, pc.pos, pd.pos);
        let idx = self.tetrahedra.len();
        self.tetrahedra.push(tet);
        self.tetra_rest_volumes.push(vol);
        Some(idx)
    }

    /// Phase 21 — adaptive tetrahedral subdivision (1 → 4 barycentric split).
    ///
    /// Each source tetrahedron `[a,b,c,d]` gains one new particle at its centroid
    /// (position = vertex mean; `inv_mass` = mean of the four endpoints, so a centroid
    /// bounded by pinned vertices stays effectively pinned) and is replaced by four
    /// sub-tetrahedra sharing that centroid: `(m,a,b,c)`, `(m,a,b,d)`, `(m,a,c,d)`,
    /// `(m,b,c,d)`. The four sub-volumes sum exactly to the parent volume, so the
    /// XPBD volume-conservation constraint (Phase 16) stays consistent — the centroid
    /// particle is a vertex of every sub-tet, so it is driven by those volume
    /// constraints directly (no extra distance edges are added, which would
    /// over-constrain and destabilise the solve). The shell topology (`triangles`)
    /// is left untouched — this is a volumetric refinement.
    ///
    /// *Adaptive*: a source tet is only split when its longest edge exceeds
    /// `max_edge_len`. Pass `max_edge_len = +∞` (the default when `!max_edge_len.is_finite()`)
    /// to subdivide every tet unconditionally. Returns the number of source tetrahedra
    /// actually split (0 if none qualified, e.g. all edges already short enough).
    ///
    /// Pure topology edit — no solver state, no SoA interaction. Determinism: source
    /// tets are processed in index order, so subdivision is reproducible.
    pub fn subdivide_tetrahedra(&mut self, max_edge_len: Real) -> usize {
        let src_tets: Vec<[u32; 4]> = self.tetrahedra.clone();
        let src_rests: Vec<Real> = self.tetra_rest_volumes.clone();
        if src_tets.is_empty() {
            return 0;
        }
        // Adaptive filter: only split tets whose longest edge exceeds the threshold.
        let adaptive = max_edge_len.is_finite();
        let mut new_tets: Vec<[u32; 4]> = Vec::with_capacity(src_tets.len() * 4);
        let mut new_rests: Vec<Real> = Vec::with_capacity(src_tets.len() * 4);
        let mut split_count = 0usize;
        for (ti, &tet) in src_tets.iter().enumerate() {
            let [a, b, c, d] = tet;
            let (pa, pb, pc, pd) = (
                self.particles[a as usize].pos,
                self.particles[b as usize].pos,
                self.particles[c as usize].pos,
                self.particles[d as usize].pos,
            );
            let longest = (pa - pb)
                .length()
                .max((pa - pc).length())
                .max((pa - pd).length())
                .max((pb - pc).length())
                .max((pb - pd).length())
                .max((pc - pd).length());
            if adaptive && longest <= max_edge_len {
                // Keep the parent tet unchanged.
                new_tets.push(tet);
                new_rests.push(src_rests[ti]);
                continue;
            }
            // Centroid particle (mean position + mean inverse mass).
            let mpos = (pa + pb + pc + pd) * 0.25;
            let im = (self.particles[a as usize].inv_mass
                + self.particles[b as usize].inv_mass
                + self.particles[c as usize].inv_mass
                + self.particles[d as usize].inv_mass)
                * 0.25;
            let m = self.particles.len() as u32;
            self.particles.push(SoftParticle {
                pos: mpos,
                vel: Vector::ZERO,
                force: Vector::ZERO,
                inv_mass: im,
                bound_body: None,
                bound_local: Vector::ZERO,
            });
            // Four sub-tetrahedra; each sub-rest-volume = 1/4 of the parent.
            let sub_rest = src_rests[ti] * 0.25;
            for sub in [[m, a, b, c], [m, a, b, d], [m, a, c, d], [m, b, c, d]] {
                new_tets.push(sub);
                new_rests.push(sub_rest);
            }
            split_count += 1;
        }
        self.tetrahedra = new_tets;
        self.tetra_rest_volumes = new_rests;
        split_count
    }

    /// Phase 6 — cloth: adds a triangular face `[a, b, c]` (CCW for outward
    /// normal) to the body's shell topology **and** automatically registers its
    /// three structural edges as distance constraints (rest length from the
    /// current particle spacing) so the existing XPBD solver keeps the face
    /// shape. Duplicate edges (shared by neighbouring triangles) are silently
    /// de-duplicated against existing distance constraints to avoid double
    /// stiffness. Returns `None` (and does nothing) if any index is out of
    /// bounds or duplicated, or if the face is degenerate (a zero-length edge).
    ///
    /// Bending stiffness is *not* added here: it is composed by the caller via
    /// [`Self::add_distance_constraint`] between opposite vertices of adjacent
    /// quad pairs (cross-diagonal edges), which needs no new mechanics.
    pub fn add_triangle(&mut self, tri: [u32; 3]) -> Option<usize> {
        let [a, b, c] = tri;
        let (pa, pb, pc) = (
            self.particles.get(a as usize)?,
            self.particles.get(b as usize)?,
            self.particles.get(c as usize)?,
        );
        if a == b || a == c || b == c {
            return None;
        }
        // Reject degenerate faces (any edge has zero rest length).
        let ab = (pb.pos - pa.pos).length();
        let bc = (pc.pos - pb.pos).length();
        let ca = (pa.pos - pc.pos).length();
        if ab == 0.0 || bc == 0.0 || ca == 0.0 {
            return None;
        }
        // Register structural edges (a-b, b-c, c-a) as distance constraints,
        // skipping any edge already present (shared by a neighbour triangle).
        for (u, v, rest) in [(a, b, ab), (b, c, bc), (c, a, ca)] {
            let exists = self.distance_constraints.iter().any(|d| {
                (d.a == u as usize && d.b == v as usize) || (d.a == v as usize && d.b == u as usize)
            });
            if !exists {
                self.distance_constraints.push(DistanceConstraint {
                    a: u as usize,
                    b: v as usize,
                    rest,
                    // Default compliance; tune via configure_xpbd / explicit later.
                    compliance: 0.0,
                    compression: 0.0,
                    activation: 0.0,
                    fibre: None,
                });
            }
        }
        let idx = self.triangles.len();
        self.triangles.push(tri);
        Some(idx)
    }

    /// Phase 6 — cloth bending: adds a single bending edge between two particles
    /// `p` and `q` as a distance constraint (rest length from current spacing).
    /// Compose bending across a quad by calling this for its two diagonals, or
    /// across a fold line by linking the un-shared vertices of two adjacent
    /// triangles. Reuses the existing XPBD distance solver (no new mechanics).
    /// Returns `None` (and does nothing) if indices are out of bounds or the
    /// endpoints coincide.
    pub fn add_bending_constraint(&mut self, p: usize, q: usize) -> Option<usize> {
        let (pp, pq) = (self.particles.get(p)?, self.particles.get(q)?);
        let rest = (pq.pos - pp.pos).length();
        if rest == 0.0 {
            return None;
        }
        let idx = self.distance_constraints.len();
        self.distance_constraints.push(DistanceConstraint {
            a: p,
            b: q,
            rest,
            compliance: 0.0,
            compression: 0.0,
            activation: 0.0,
            fibre: None,
        });
        Some(idx)
    }

    /// Phase 9: enables/disables tearing. `strain` is the max allowed strain
    /// `(|len| − rest)/rest` before a structural edge snaps. Pass `None` to
    /// disable (default). A value ≤ 0 tears immediately on any stretch.
    /// Phase 9/27: enables/disables tearing by **strain** threshold. `strain` is
    /// the max allowed strain `(|len| − rest)/rest` before a structural edge
    /// snaps. Pass `None` to disable (default). A value ≤ 0 tears immediately on
    /// any stretch.
    pub fn set_tear_strain(&mut self, strain: Option<Real>) {
        self.tear = strain.map(TearCriterion::Strain);
    }

    /// Phase 27: enables/disables tearing by **axial stress** threshold. `stress`
    /// is the max allowed `|k·(len − rest)|` before an edge snaps. Pass `None` to
    /// disable. A value ≤ 0 tears immediately.
    pub fn set_tear_stress(&mut self, stress: Option<Real>) {
        self.tear = stress.map(TearCriterion::Stress);
    }

    /// Phase 27: enables/disables tearing by **strain-energy** (fracture-toughness)
    /// threshold. `energy` is the max allowed `½·k·(len − rest)²` before an edge
    /// snaps. Pass `None` to disable. A value ≤ 0 tears immediately.
    pub fn set_tear_energy(&mut self, energy: Option<Real>) {
        self.tear = energy.map(TearCriterion::Energy);
    }

    /// Phase 27: sets the body-level orthotropic stiffness axes (see
    /// [`Self::anisotropy`]). `None` disables directional response.
    pub fn set_anisotropy(&mut self, axes: Option<Vector>) {
        self.anisotropy = axes;
    }

    /// Phase 31 — sets the body-wide active-strain activation `γ ∈ [0, 1]` on every
    /// spring and distance constraint at once (the "muscle contraction" level). The
    /// effective rest length of each edge becomes `rest * (1 - γ)`, so a positive
    /// activation actively pulls endpoints together. Values are clamped to `[0, 1]`;
    /// `activate(0)` is the passive baseline. Non-finite input is ignored (no-op).
    pub fn set_activation(&mut self, gamma: Real) -> bool {
        if !gamma.is_finite() || gamma < 0.0 || gamma > 1.0 {
            return false;
        }
        for s in &mut self.springs {
            s.activation = gamma;
        }
        for c in &mut self.distance_constraints {
            c.activation = gamma;
        }
        true
    }

    /// Phase 31 — sets the active-strain activation of a single spring (by the index
    /// returned from `add_spring`). Out-of-range or non-finite `activation` (or one
    /// outside `[0, 1]`) is rejected. Returns `false` on any invalid input.
    pub fn set_spring_activation(&mut self, index: usize, activation: Real) -> bool {
        if !activation.is_finite() || activation < 0.0 || activation > 1.0 {
            return false;
        }
        match self.springs.get_mut(index) {
            Some(s) => {
                s.activation = activation;
                true
            }
            None => false,
        }
    }

    /// Phase 31 — sets the active-strain activation of a single distance constraint
    /// (by the index returned from `add_distance_constraint`). Out-of-range or
    /// non-finite `activation` (or one outside `[0, 1]`) is rejected. Returns
    /// `false` on any invalid input.
    pub fn set_distance_constraint_activation(&mut self, index: usize, activation: Real) -> bool {
        if !activation.is_finite() || activation < 0.0 || activation > 1.0 {
            return false;
        }
        match self.distance_constraints.get_mut(index) {
            Some(c) => {
                c.activation = activation;
                true
            }
            None => false,
        }
    }

    /// Phase 32 — sets the muscle-fibre direction of a single distance constraint
    /// (by the index returned from `add_distance_constraint`). When `dir` is a
    /// finite non-zero vector it is normalized and stored as the fibre orientation
    /// for anisotropic active contraction; a zero vector clears the fibre (back to
    /// edge-aligned contraction). Returns `false` for an unknown index.
    pub fn set_fibre_direction(&mut self, index: usize, dir: Vector) -> bool {
        if !dir.is_finite() {
            return false;
        }
        match self.distance_constraints.get_mut(index) {
            Some(c) => {
                if dir.x == 0.0 && dir.y == 0.0 && dir.z == 0.0 {
                    c.fibre = None;
                } else {
                    c.fibre = Some(dir.normalize());
                }
                true
            }
            None => false,
        }
    }

    /// Phase 32 — sets the muscle-fibre direction of a single spring (by the index
    /// returned from `add_spring`). See [`Self::set_fibre_direction`]; a zero vector
    /// clears the fibre. Returns `false` for an unknown index.
    pub fn set_spring_fibre_direction(&mut self, index: usize, dir: Vector) -> bool {
        if !dir.is_finite() {
            return false;
        }
        match self.springs.get_mut(index) {
            Some(s) => {
                if dir.x == 0.0 && dir.y == 0.0 && dir.z == 0.0 {
                    s.fibre = None;
                } else {
                    s.fibre = Some(dir.normalize());
                }
                true
            }
            None => false,
        }
    }

    /// Phase 9: removes every over-stretched structural edge. Called once at the
    /// top of [`SoftBody::step`]; a no-op when `tear_strain` is `None`.
    ///
    /// Edge strain is `s = (|len| − rest) / rest`. An XPBD distance constraint or
    /// MassSpring spring with `s > threshold` is dropped. Triangular faces that
    /// lose any of their structural edges (a-b, b-c, c-a) are dropped as well, so
    /// a torn cloth stops rendering the broken face. Pure topology edit — neither
    /// the SoA solver nor the integration order is touched.
    pub fn tear(&mut self) {
        let threshold = match self.tear {
            Some(TearCriterion::Strain(t))
            | Some(TearCriterion::Stress(t))
            | Some(TearCriterion::Energy(t)) => t,
            None => return,
        };
        if threshold <= 0.0 {
            return; // 0 / negative would tear everything on the first step.
        }
        // Per-edge fracture scalar for the active criterion.
        let metric = |rest: Real, len: Real, k: Real| -> Real {
            match self.tear {
                Some(TearCriterion::Strain(_)) => (len - rest) / rest.max(1e-12),
                Some(TearCriterion::Stress(_)) => (k * (len - rest)).abs(),
                Some(TearCriterion::Energy(_)) => 0.5 * k * (len - rest) * (len - rest),
                None => 0.0,
            }
        };
        let spring_k = |s: &Spring| s.stiffness;
        let dc_k = |c: &DistanceConstraint| 1.0 / (c.compliance + 1e-9);

        // Collect the surviving distance-constraint edges (and their indices).
        let mut keep_dc: Vec<(usize, DistanceConstraint)> = Vec::new();
        for (i, c) in self.distance_constraints.iter().enumerate() {
            let (pa, pb) = match (self.particles.get(c.a), self.particles.get(c.b)) {
                (Some(a), Some(b)) => (a, b),
                _ => continue, // dangling edge → drop.
            };
            let len = (pb.pos - pa.pos).length();
            let k = dc_k(c);
            if metric(c.rest, len, k) <= threshold {
                keep_dc.push((i, *c));
            }
        }
        let broken_dc: HashSet<usize> = {
            let mut s = HashSet::new();
            for i in 0..self.distance_constraints.len() {
                if !keep_dc.iter().any(|(ki, _)| *ki == i) {
                    s.insert(i);
                }
            }
            s
        };
        self.distance_constraints = keep_dc.into_iter().map(|(_, c)| c).collect();

        // Same for MassSpring springs.
        let mut keep_sp: Vec<Spring> = Vec::new();
        for s in self.springs.drain(..) {
            let (pa, pb) = match (self.particles.get(s.a), self.particles.get(s.b)) {
                (Some(a), Some(b)) => (a, b),
                _ => continue,
            };
            let len = (pb.pos - pa.pos).length();
            let k = spring_k(&s);
            if metric(s.rest_length, len, k) <= threshold {
                keep_sp.push(s);
            }
        }
        self.springs = keep_sp;

        // Drop triangles that lost any structural edge. A triangle's structural
        // edges are its three sides; an edge is "structural" if it survives as a
        // distance constraint (the XPBD path used by cloth) before this tear.
        // (MassSpring cloth is built from springs, which we also dropped above, so
        // we check both: an edge that is neither a surviving distance-constraint
        // nor a surviving spring has snapped.)
        let has_edge = |a: u32, b: u32| -> bool {
            let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
            self.distance_constraints.iter().any(|c| {
                let (ca, cb) = if c.a as u32 <= c.b as u32 {
                    (c.a as u32, c.b as u32)
                } else {
                    (c.b as u32, c.a as u32)
                };
                (ca, cb) == (lo, hi)
            }) || self.springs.iter().any(|s| {
                let (sa, sb) = if s.a as u32 <= s.b as u32 {
                    (s.a as u32, s.b as u32)
                } else {
                    (s.b as u32, s.a as u32)
                };
                (sa, sb) == (lo, hi)
            })
        };
        self.triangles
            .retain(|t| has_edge(t[0], t[1]) && has_edge(t[1], t[2]) && has_edge(t[2], t[0]));
        let _ = broken_dc; // retained for clarity; broken edges already excluded.
    }

    /// Phase 10: enables/disables plasticity. Pass `None` to disable (perfectly
    /// elastic, the default). With `Some(PlasticityParams { yield_strain, creep })`,
    /// edges whose elastic strain magnitude exceeds `yield_strain` permanently shift
    /// their rest length toward the current length by `creep` (clamped to `[0,1]`)
    /// each step — the deformation freezes in instead of springing back.
    ///
    /// `yield_strain` is clamped to `≥ 0`; `creep` is clamped to `[0,1]`.
    pub fn set_plasticity(&mut self, params: Option<PlasticityParams>) {
        self.plasticity = params.map(|p| PlasticityParams {
            yield_strain: p.yield_strain.max(0.0),
            creep: p.creep.clamp(0.0, 1.0),
        });
    }

    /// Phase 10: permanently deforms over-yielded edges. Called once at the top of
    /// [`SoftBody::step`] after [`SoftBody::tear`]; a no-op when `plasticity` is
    /// `None`.
    ///
    /// For every structural edge (XPBD distance constraint / MassSpring spring) the
    /// elastic strain `s = (|len| − rest) / rest` is measured. If `|s| > yield_strain`,
    /// the over-yield portion is frozen: `rest += creep · (len − rest)` (the rest
    /// length moves toward the current length, so the edge no longer pulls back toward
    /// its original size). Pure rest-length edit — neither the SoA solver nor the
    /// integration order is touched.
    pub fn apply_plasticity(&mut self) {
        let (yield_strain, creep) = match self.plasticity {
            Some(p) => (p.yield_strain, p.creep),
            None => return,
        };
        if yield_strain <= 0.0 || creep <= 0.0 {
            return; // degenerate: no plasticity.
        }

        // Distance constraints (XPBD path). Filtered first into a `Vec`, then the
        // inner `self.particles` borrow for length measurement is released before we
        // mutate `self.distance_constraints` — keeps the two borrows disjoint.
        let new_rests: Vec<Real> = self
            .distance_constraints
            .iter()
            .map(|c| {
                let (pa, pb) = match (self.particles.get(c.a), self.particles.get(c.b)) {
                    (Some(a), Some(b)) => (a, b),
                    _ => return c.rest, // dangling edge → leave unchanged.
                };
                let len = (pb.pos - pa.pos).length();
                if c.rest <= 0.0 {
                    return c.rest;
                }
                let strain = (len - c.rest) / c.rest;
                if strain.abs() <= yield_strain {
                    c.rest
                } else {
                    // Freeze part of the elastic stretch into the rest length.
                    c.rest + creep * (len - c.rest)
                }
            })
            .collect();
        for (c, nr) in self.distance_constraints.iter_mut().zip(new_rests) {
            c.rest = nr;
        }

        // MassSpring springs.
        let new_springs: Vec<Spring> = self
            .springs
            .iter()
            .map(|s| {
                let (pa, pb) = match (self.particles.get(s.a), self.particles.get(s.b)) {
                    (Some(a), Some(b)) => (a, b),
                    _ => return *s,
                };
                let len = (pb.pos - pa.pos).length();
                if s.rest_length <= 0.0 {
                    return *s;
                }
                let strain = (len - s.rest_length) / s.rest_length;
                if strain.abs() <= yield_strain {
                    *s
                } else {
                    let mut ns = *s;
                    ns.rest_length += creep * (len - s.rest_length);
                    ns
                }
            })
            .collect();
        self.springs = new_springs;
    }

    /// Removes the particle at `index`, keeping the topology consistent.
    ///
    /// Springs, distance constraints, and tetrahedra that *reference* the removed
    /// particle are dropped; every remaining index `> index` is decremented by one
    /// so it still points at the same particle. `tetra_rest_volumes` is filtered in
    /// lockstep with `tetrahedra`. Returns `false` (and does nothing) if `index` is
    /// out of bounds.
    ///
    /// This is the per-particle counterpart of [`SoftBodySet::remove`]: deleting a
    /// block/voxel in a Minecraft chunk maps to removing the corresponding particle
    /// plus its incident springs/edges, after which the body keeps simulating under
    /// the new (smaller) topology.
    pub fn remove_particle(&mut self, index: usize) -> bool {
        if index >= self.particles.len() {
            return false;
        }
        self.particles.remove(index);

        // Springs: drop any touching `index`, shift the rest.
        self.springs.retain_mut(|s| {
            if s.a == index || s.b == index {
                return false;
            }
            if s.a > index {
                s.a -= 1;
            }
            if s.b > index {
                s.b -= 1;
            }
            true
        });

        // Distance constraints: same treatment.
        self.distance_constraints.retain_mut(|c| {
            if c.a == index || c.b == index {
                return false;
            }
            if c.a > index {
                c.a -= 1;
            }
            if c.b > index {
                c.b -= 1;
            }
            true
        });

        // Tetrahedra: drop any containing `index`; otherwise shift each index down.
        let keep: Vec<bool> = self
            .tetrahedra
            .iter()
            .map(|t| !t.contains(&(index as u32)))
            .collect();
        let remapped: Vec<[u32; 4]> = self
            .tetrahedra
            .iter()
            .enumerate()
            .filter(|(i, _)| keep[*i])
            .map(|(_, t)| {
                let mut t = *t;
                for x in t.iter_mut() {
                    if *x as usize > index {
                        *x -= 1;
                    }
                }
                t
            })
            .collect();
        let kept_volumes: Vec<Real> = self
            .tetra_rest_volumes
            .iter()
            .enumerate()
            .filter(|(i, _)| keep[*i])
            .map(|(_, v)| *v)
            .collect();
        self.tetrahedra = remapped;
        self.tetra_rest_volumes = kept_volumes;

        // Triangles: drop any containing `index`; otherwise shift each index down.
        let keep_tri: Vec<bool> = self
            .triangles
            .iter()
            .map(|t| !t.contains(&(index as u32)))
            .collect();
        let remapped_tri: Vec<[u32; 3]> = self
            .triangles
            .iter()
            .enumerate()
            .filter(|(i, _)| keep_tri[*i])
            .map(|(_, t)| {
                let mut t = *t;
                for x in t.iter_mut() {
                    if *x as usize > index {
                        *x -= 1;
                    }
                }
                t
            })
            .collect();
        self.triangles = remapped_tri;

        true
    }

    /// Total (sum of absolute) signed volume of all tetrahedra — a finite,
    /// deformation-sensitive scalar useful for regression tests.
    pub fn total_volume(&self) -> Real {
        self.tetra_rest_volumes
            .iter()
            .zip(self.tetrahedra.iter())
            .map(|(rest_vol, tet)| {
                let [a, b, c, d] = *tet;
                let (pa, pb, pc, pd) = (
                    self.particles[a as usize].pos,
                    self.particles[b as usize].pos,
                    self.particles[c as usize].pos,
                    self.particles[d as usize].pos,
                );
                signed_tetra_volume(pa, pb, pc, pd).abs() / rest_vol.abs().max(1e-12)
            })
            .fold(0.0, |acc, r| acc + r)
    }
}


/// Signed volume of the tetrahedron `(p0, p1, p2, p3)`.
fn signed_tetra_volume(p0: Vector, p1: Vector, p2: Vector, p3: Vector) -> Real {
    let e1 = p1 - p0;
    let e2 = p2 - p0;
    let e3 = p3 - p0;
    e1.cross(e2).dot(e3) / 6.0
}

/// Rest shape matrix of the tetrahedron `(p0, p1, p2, p3)` (columns are the edges).
fn rest_shape_matrix(p0: Vector, p1: Vector, p2: Vector, p3: Vector) -> [[Real; 3]; 3] {
    let e1 = p1 - p0;
    let e2 = p2 - p0;
    let e3 = p3 - p0;
    [[e1.x, e2.x, e3.x], [e1.y, e2.y, e3.y], [e1.z, e2.z, e3.z]]
}

/// A set of soft bodies keyed by slot index: the authoring side of the migrated
/// soft-body layer. Simulation lives in the official solver (see
/// [`crate::rapier::soft_upstream`]).
#[derive(Default)]
pub struct SoftBodySet {
    /// Slot storage. A slot becomes `None` once its body is removed via
    /// [`SoftBodySet::remove`], which keeps `SoftBodyId`s stable: ids are slot
    /// indices, so removal is id-preserving and never reshuffles live bodies
    /// (important for the FFI layer that hands `SoftBodyId`s to callers).
    bodies: Vec<Option<SoftBody>>,
}

impl SoftBodySet {
    /// Creates an empty set.
    pub fn new() -> Self {
        Self::default()
    }

    /// Inserts a soft body and returns its id.
    pub fn insert(&mut self, body: SoftBody) -> SoftBodyId {
        let id = SoftBodyId(self.bodies.len() as u32);
        self.bodies.push(Some(body));
        id
    }

    /// Number of slots (including tombstoned/removed ones).
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    /// Number of live (not removed) soft bodies.
    pub fn count(&self) -> usize {
        self.bodies.iter().filter(|b| b.is_some()).count()
    }

    /// Whether the set holds no live bodies.
    pub fn is_empty(&self) -> bool {
        self.count() == 0
    }

    /// Removes the soft body with the given id, freeing its storage. The id
    /// stays reserved (the slot becomes a tombstone), so every other live
    /// `SoftBodyId` remains valid — callers may keep holding ids across removals.
    /// Returns `true` if a live body was removed.
    pub fn remove(&mut self, id: SoftBodyId) -> bool {
        let slot = match self.bodies.get_mut(id.0 as usize) {
            Some(slot) => slot,
            None => return false,
        };
        match slot.take() {
            Some(_) => true,
            None => false,
        }
    }

    /// Immutable access by id. Returns `None` for an unknown or removed id.
    #[allow(dead_code)] // consumed by later integration phases (World/FFI).
    pub fn get(&self, id: SoftBodyId) -> Option<&SoftBody> {
        self.bodies.get(id.0 as usize).and_then(|b| b.as_ref())
    }

    /// Mutable access by id. Returns `None` for an unknown or removed id.
    pub fn get_mut(&mut self, id: SoftBodyId) -> Option<&mut SoftBody> {
        self.bodies.get_mut(id.0 as usize).and_then(|b| b.as_mut())
    }

    /// Iterator over all live `(SoftBodyId, &SoftBody)` pairs, in ascending id order.
    pub fn iter(&self) -> impl Iterator<Item = (SoftBodyId, &SoftBody)> {
        self.bodies
            .iter()
            .enumerate()
            .filter_map(|(i, b)| b.as_ref().map(|sb| (SoftBodyId(i as u32), sb)))
    }

    /// Simultaneous mutable access to two distinct live bodies. Returns
    /// `(None, None)` if either id is unknown/removed or the two ids are equal.
    /// Used by the Phase 14 cross-body collision pass to project two bodies apart.
    pub fn get2_mut(
        &mut self,
        a: SoftBodyId,
        b: SoftBodyId,
    ) -> (Option<&mut SoftBody>, Option<&mut SoftBody>) {
        if a == b {
            return (None, None);
        }
        let (lo, hi) = if a.0 <= b.0 { (a, b) } else { (b, a) };
        let (lo_i, hi_i) = (lo.0 as usize, hi.0 as usize);
        let len = self.bodies.len();
        if lo_i >= len || hi_i >= len {
            return (None, None);
        }
        // Split the slice so the two borrows are disjoint.
        let (left, right) = self.bodies.split_at_mut(hi_i);
        let first = left.get_mut(lo_i).and_then(|b| b.as_mut());
        let second = right.get_mut(0).and_then(|b| b.as_mut());
        if a.0 <= b.0 {
            (first, second)
        } else {
            (second, first)
        }
    }

    pub fn sleep(&mut self, id: SoftBodyId) -> bool {
        match self.bodies.get_mut(id.0 as usize).and_then(|b| b.as_mut()) {
            Some(b) => {
                b.sleeping = true;
                true
            }
            None => false,
        }
    }

    /// Wakes a sleeping soft body.
    pub fn wake(&mut self, id: SoftBodyId) -> bool {
        match self.bodies.get_mut(id.0 as usize).and_then(|b| b.as_mut()) {
            Some(b) => {
                b.sleeping = false;
                true
            }
            None => false,
        }
    }

    /// Whether the soft body is currently sleeping.
    pub fn is_sleeping(&self, id: SoftBodyId) -> bool {
        self.bodies
            .get(id.0 as usize)
            .and_then(|b| b.as_ref())
            .map(|b| b.sleeping)
            .unwrap_or(false)
    }

}
