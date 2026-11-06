//! Rope knot/weaving system — complex rope topology with inter-strand collision.
//!
//! A **rope knot system** extends the basic rope (`rope.rs`) with:
//!
//! * **Inter-strand collision** — each braid strand is its own soft body, so
//!   the Phase 5f collision-proxy mechanism (`soft_body_enable_collision`)
//!   gives strand-on-strand contact and friction while a strand never
//!   collides with itself (its own segments stay connected by constraints)
//! * **High compliance control** — XPBD distance constraints with the
//!   caller-supplied stiffness keep the topology flexible enough for knots
//! * **Weaving patterns** — procedural overhand / figure-eight knots and
//!   helical braid topologies
//!
//! This is a pure composition layer on top of the fork's `SoftBody`: each
//! strand is a chain of point masses joined by XPBD distance constraints, and
//! collision proxies are built through the existing per-particle mechanism.

use rapier3d::math::Vector;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, ERR_UNSUPPORTED,
    clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{Bool, Vec3, WorldHandle, vec3_finite, vec3_to_rapier};

const MAX_KNOT_POINTS: u32 = 256;
const MAX_KNOT_STRANDS: u32 = 16;
/// Gauss-Seidel projection iterations per substep for the XPBD solver.
const KNOT_XPBD_ITERATIONS: u32 = 8;
/// Fallback compliance when a knot declares `stiffness == 0`.
const FALLBACK_COMPLIANCE: f64 = 1.0e-4;
/// Full helix turns of each braid strand along the rope length.
const BRAID_TURNS: f64 = 4.0;
/// Particles per braid strand.
const BRAID_STRAND_POINTS: usize = 20;

/// Knot/weaving pattern types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum KnotPattern {
    /// Simple overhand knot.
    Overhand,
    /// Figure-eight knot.
    FigureEight,
    /// Square braid (4-strand).
    SquareBraid,
    /// Round braid (3-strand).
    RoundBraid,
    /// Custom weaving pattern (defined by control points).
    Custom,
}

/// Rope knot configuration.
#[derive(Debug, Clone)]
pub(crate) struct RopeKnotDesc {
    /// Pattern type.
    pub pattern: KnotPattern,
    /// Number of rope strands (for braids).
    pub strand_count: u32,
    /// Control points for custom patterns (world space).
    pub control_points: Vec<Vec3>,
    /// Rope radius (for collision proxies).
    pub radius: f64,
    /// Rope stiffness (lower = more flexible for knots).
    pub stiffness: f64,
    /// Rope-on-rope friction coefficient.
    pub self_friction: f64,
    /// Rope density.
    pub density: f64,
}

/// Rope knot system state.
pub(crate) struct RopeKnotSystem {
    /// Rope knot descriptor.
    pub desc: RopeKnotDesc,
    /// Soft body IDs, one per strand (braids) or a single body (knots/custom).
    pub soft_bodies: Vec<rapier3d::prelude::soft_body_fork::SoftBodyId>,
    /// Wind force applied to the rope.
    pub wind: Vec3,
}

/// Create a rope knot system.
///
/// Returns a stable id, or `u32::MAX` on error.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn rope_knot_create(
    world: *mut WorldHandle,
    pattern: u32,
    strand_count: u32,
    control_points: *const Vec3,
    control_point_count: u32,
    radius: f64,
    stiffness: f64,
    self_friction: f64,
    density: f64,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return u32::MAX;
        };

        let knot_pattern = match pattern {
            0 => KnotPattern::Overhand,
            1 => KnotPattern::FigureEight,
            2 => KnotPattern::SquareBraid,
            3 => KnotPattern::RoundBraid,
            4 => KnotPattern::Custom,
            _ => {
                set_error(ERR_INVALID_ARGUMENT, "invalid knot pattern");
                return u32::MAX;
            }
        };

        if strand_count == 0 || strand_count > MAX_KNOT_STRANDS {
            set_error(ERR_CAPACITY, "invalid strand count");
            return u32::MAX;
        }

        if !radius.is_finite() || radius <= 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid radius");
            return u32::MAX;
        }

        if !stiffness.is_finite() || stiffness < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid stiffness");
            return u32::MAX;
        }

        if !self_friction.is_finite() || self_friction < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid self friction");
            return u32::MAX;
        }

        if !density.is_finite() || density <= 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "invalid density");
            return u32::MAX;
        }

        let mut control_points_vec = Vec::new();
        if knot_pattern == KnotPattern::Custom {
            if control_points.is_null() || control_point_count == 0 {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "custom pattern requires control points",
                );
                return u32::MAX;
            }
            if control_point_count > MAX_KNOT_POINTS {
                set_error(ERR_CAPACITY, "too many control points");
                return u32::MAX;
            }
            control_points_vec =
                unsafe { std::slice::from_raw_parts(control_points, control_point_count as usize) }
                    .to_vec();
            for cp in &control_points_vec {
                if !vec3_finite(*cp) {
                    set_error(ERR_INVALID_ARGUMENT, "invalid control point");
                    return u32::MAX;
                }
            }
        }

        let id = world.inner.rope_knots.insert(RopeKnotSystem {
            desc: RopeKnotDesc {
                pattern: knot_pattern,
                strand_count,
                control_points: control_points_vec,
                radius,
                stiffness,
                self_friction,
                density,
            },
            soft_bodies: Vec::new(),
            wind: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        });

        clear_error();
        id
    })
}

/// Build the rope knot geometry (creates the per-strand soft bodies and their
/// collision proxies).
///
/// Returns `true` on success.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn rope_knot_build(
    world: *mut WorldHandle,
    id: u32,
    start: Vec3,
    end: Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        if !vec3_finite(start) || !vec3_finite(end) {
            set_error(ERR_INVALID_ARGUMENT, "invalid start/end positions");
            return Bool::FALSE;
        }

        // Snapshot the descriptor + wind so the mutation pass can run without
        // holding the `rope_knots` borrow.
        let (desc, wind, already_built) = match world.inner.rope_knots.get(id) {
            Some(knot) => (knot.desc.clone(), knot.wind, !knot.soft_bodies.is_empty()),
            None => {
                set_error(ERR_NOT_FOUND, "rope knot system not found");
                return Bool::FALSE;
            }
        };
        if already_built {
            set_error(ERR_UNSUPPORTED, "rope knot already built");
            return Bool::FALSE;
        }

        let start_vec = vec3_to_rapier(start);
        let end_vec = vec3_to_rapier(end);
        let span = end_vec - start_vec;
        let length = span.length();
        if length <= 1e-9 {
            set_error(ERR_INVALID_ARGUMENT, "degenerate start/end span");
            return Bool::FALSE;
        }
        let dir = span / length;
        // Perpendicular basis for twist / helix offsets.
        let reference = if dir.x.abs() < 0.9 {
            Vector::X
        } else {
            Vector::Y
        };
        let perp1 = dir.cross(reference).normalize_or_zero();
        let perp2 = dir.cross(perp1).normalize_or_zero();

        let compliance = if desc.stiffness > 0.0 {
            1.0 / desc.stiffness
        } else {
            FALLBACK_COMPLIANCE
        };
        let solver = rapier3d::prelude::soft_body_fork::SoftSolver::Xpbd {
            iterations: KNOT_XPBD_ITERATIONS,
            compliance,
        };

        // One polyline (world-space particle positions) per strand.
        let strand_polylines: Vec<Vec<Vector>> = match desc.pattern {
            KnotPattern::Overhand | KnotPattern::FigureEight => {
                let count = if desc.pattern == KnotPattern::Overhand {
                    24
                } else {
                    32
                };
                let twist_turns = if desc.pattern == KnotPattern::Overhand {
                    1.0
                } else {
                    2.0
                };
                let mut points = Vec::with_capacity(count);
                for i in 0..count {
                    let t = i as f64 / (count - 1) as f64;
                    let twist = (t * std::f64::consts::TAU * twist_turns).sin() * desc.radius;
                    points.push(start_vec + dir * (length * t) + perp1 * twist);
                }
                vec![points]
            }
            KnotPattern::SquareBraid | KnotPattern::RoundBraid => {
                // Each strand is a helix around the centerline, phase-shifted
                // so the strands weave around each other.
                let mut strands = Vec::with_capacity(desc.strand_count as usize);
                for strand in 0..desc.strand_count as usize {
                    let phase = strand as f64 * std::f64::consts::TAU / desc.strand_count as f64;
                    let mut points = Vec::with_capacity(BRAID_STRAND_POINTS);
                    for i in 0..BRAID_STRAND_POINTS {
                        let t = i as f64 / (BRAID_STRAND_POINTS - 1) as f64;
                        let angle = t * std::f64::consts::TAU * BRAID_TURNS + phase;
                        points.push(
                            start_vec
                                + dir * (length * t)
                                + perp1 * (desc.radius * angle.cos())
                                + perp2 * (desc.radius * angle.sin()),
                        );
                    }
                    strands.push(points);
                }
                strands
            }
            KnotPattern::Custom => {
                vec![
                    desc.control_points
                        .iter()
                        .map(|cp| vec3_to_rapier(*cp))
                        .collect(),
                ]
            }
        };

        // Build one XPBD soft body per strand.
        let mut strand_ids: Vec<u32> = Vec::with_capacity(strand_polylines.len());
        for points in &strand_polylines {
            if points.len() < 2 {
                set_error(ERR_INVALID_ARGUMENT, "knot pattern produced too few points");
                return Bool::FALSE;
            }
            // Segment length along the strand → point mass from the rope
            // density (cylinder volume per segment). The proxy mechanism reads
            // the mass back from `inv_mass`, so the collision proxies carry
            // the same mass as the rope.
            let spacing = length / (points.len() - 1).max(1) as f64;
            let mass =
                (desc.density * std::f64::consts::PI * desc.radius.powi(2) * spacing).max(1e-9);

            let mut soft_body = rapier3d::prelude::soft_body_fork::SoftBody::new(Vector::ZERO);
            soft_body.solver = solver;
            for p in points {
                let idx = soft_body.add_particle(*p);
                soft_body.particles[idx].inv_mass = 1.0 / mass;
            }
            for i in 0..points.len() - 1 {
                soft_body.add_distance_constraint(i, i + 1, compliance);
            }
            soft_body.apply_wind(vec3_to_rapier(wind), 0.0);

            let soft_id = world.inner.soft_bodies.insert(soft_body);
            strand_ids.push(soft_id.0);
        }

        // Record the strand ids, then enable collision coupling so every free
        // particle gets a proxy ball (same-body particles don't collide with
        // each other; different strands and terrain do — with the
        // caller-supplied rope-on-rope friction).
        if let Some(knot) = world.inner.rope_knots.get_mut(id) {
            knot.soft_bodies = strand_ids
                .iter()
                .map(|&sid| rapier3d::prelude::soft_body_fork::SoftBodyId(sid))
                .collect();
        }
        for sid in strand_ids {
            crate::rapier::soft_body::soft_body_enable_collision(
                world,
                sid,
                desc.radius,
                Bool::TRUE,
            );
        }

        // `self_friction` is applied on the proxy colliders; patch them in
        // place (enable_collision builds them with the collider default).
        set_proxy_friction(world, id, desc.self_friction);

        clear_error();
        Bool::TRUE
    })
}

/// Patches the friction coefficient on every proxy collider of a knot's
/// strands (the enable-collision path builds them with the default friction).
fn set_proxy_friction(world: &mut WorldHandle, id: u32, friction: f64) {
    let strand_ids: Vec<u32> = world
        .inner
        .rope_knots
        .get(id)
        .map(|k| k.soft_bodies.iter().map(|s| s.0).collect())
        .unwrap_or_default();
    for sid in strand_ids {
        if let Some(proxies) = world.inner.soft_body_proxies.get(&sid) {
            for ph in proxies.iter().flatten() {
                if let Some(rb) = world.inner.bodies.get(*ph) {
                    for col_handle in rb.colliders() {
                        if let Some(col) = world.inner.colliders.get_mut(*col_handle) {
                            col.set_friction(friction);
                        }
                    }
                }
            }
        }
    }
}

/// Set wind force for a rope knot system.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn rope_knot_set_wind(world: *mut WorldHandle, id: u32, wind: Vec3) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        let Some(knot_system) = world.inner.rope_knots.get_mut(id) else {
            set_error(ERR_NOT_FOUND, "rope knot system not found");
            return Bool::FALSE;
        };

        if !vec3_finite(wind) {
            set_error(ERR_INVALID_ARGUMENT, "invalid wind vector");
            return Bool::FALSE;
        }

        knot_system.wind = wind;

        // Push the new wind field into every strand soft body (built or not —
        // `build` re-applies the stored wind when strands are created later).
        let strand_ids = knot_system.soft_bodies.clone();
        for soft_id in strand_ids {
            if let Some(soft_body) = world.inner.soft_bodies.get_mut(soft_id) {
                soft_body.apply_wind(vec3_to_rapier(wind), 0.0);
            }
        }

        clear_error();
        Bool::TRUE
    })
}

/// Remove a rope knot system from the world.
///
/// Tears down the per-strand collision proxies before removing the soft
/// bodies themselves.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn rope_knot_remove(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };

        let Some(knot_system) = world.inner.rope_knots.remove(id) else {
            set_error(ERR_NOT_FOUND, "rope knot system not found");
            return Bool::FALSE;
        };

        for soft_id in knot_system.soft_bodies {
            // Remove the collision proxies first (mirrors the teardown path of
            // `soft_body_enable_collision`).
            if let Some(proxies) = world.inner.soft_body_proxies.remove(&soft_id.0) {
                for proxy_handle in proxies.into_iter().flatten() {
                    world.inner.bodies.remove(
                        proxy_handle,
                        &mut world.inner.islands,
                        &mut world.inner.colliders,
                        &mut world.inner.impulse_joints,
                        &mut world.inner.multibody_joints,
                        &mut world.inner.upstream_soft_bodies,
                        false,
                    );
                }
            }
            world.inner.soft_bodies.remove(soft_id);
        }

        clear_error();
        Bool::TRUE
    })
}

/// Query the soft-body id backing a knot strand (for particle read-out, e.g.
/// rendering). Braids own one soft body per strand; knots and custom patterns
/// own a single one. Only valid after `rope_knot_build`.
///
/// Returns the `SoftBodyId.0`, or `u32::MAX` on error.
///
/// # Safety
///
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn rope_knot_strand_soft_body(
    world: *mut WorldHandle,
    id: u32,
    strand_index: u32,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return u32::MAX;
        };
        let Some(knot) = world.inner.rope_knots.get(id) else {
            set_error(ERR_NOT_FOUND, "rope knot system not found");
            return u32::MAX;
        };
        match knot.soft_bodies.get(strand_index as usize) {
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
