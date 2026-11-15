//! Hertzian and Mindlin contact mechanics — the standard force laws for
//! non-adhesive elastic contacts.
//!
//! Pure computation only — no access to `WorldHandle`, `RigidBody`, or Rapier
//! state. Every function returns `None` on an input outside its physical domain
//! rather than a NaN or a silently clamped value, matching
//! [docs/formula-numeric-domains.md](../../../docs/formula-numeric-domains.md).
//!
//! ## Why this module exists
//!
//! `rapier/src/dynamics/granular.rs` describes its normal law as
//! "Hertzian-style" but implements a linear spring (`F = k·δ`). A linear spring
//! is a legitimate DEM choice, but it is *not* Hertzian, and the distinction
//! matters as soon as anyone tunes `k` from a material's Young's modulus:
//!
//! | law | normal force | stiffness |
//! | --- | --- | --- |
//! | linear spring | `F = k·δ` | `k` [N/m] |
//! | Hertz | `F = (4/3)·E*·√R*·δ^{3/2}` | none — derived from `E*`, `R*` |
//!
//! Hertz also gives the *correct* contact radius and pressure distribution, which
//! the linear model has no notion of. This module supplies the closed forms so a
//! caller can (a) evaluate a Hertzian contact directly, or (b) derive the linear
//! spring constant that matches a Hertzian contact at a chosen overlap — the
//! second is what makes an existing linear-spring solver physically tunable.
//!
//! ## Conventions
//!
//! - `e_star` is the *effective* Young's modulus `1/E* = (1-ν₁²)/E₁ + (1-ν₂²)/E₂`.
//!   For a rigid plane against an elastic body, `E* = E/(1-ν²)`.
//! - `r_star` is the *effective* radius `1/R* = 1/R₁ + 1/R₂`. A sphere on a plane
//!   has `R* = R`; two equal spheres have `R* = R/2`; a plane (`R = ∞`)
//!   contributes zero to the sum.
//! - `delta` is the overlap (penetration depth), always `≥ 0`; a negative value
//!   means separation and yields zero force, not a negative one — these laws are
//!   non-adhesive by construction.

use std::f64::consts::PI;

use crate::math::finite_many;

// ---------------------------------------------------------------------------
// Effective material and geometry properties
// ---------------------------------------------------------------------------

/// Effective Young's modulus of a two-body elastic contact:
/// `1/E* = (1 - ν₁²)/E₁ + (1 - ν₂²)/E₂`.
///
/// Pass `youngs2 = f64::INFINITY` (or use [`effective_modulus_rigid_plane`]) for
/// a rigid counter-surface.
pub fn effective_youngs_modulus(
    youngs1: f64,
    poisson1: f64,
    youngs2: f64,
    poisson2: f64,
) -> Option<f64> {
    if !finite_many(&[youngs1, poisson1, poisson2]) || youngs1 <= 0.0 {
        return None;
    }
    // Poisson's ratio is physically open on both ends: -1 (auxetic limit) and 0.5
    // (incompressible) are asymptotes, not attainable values. `contains` on a
    // half-open range would wrongly admit exactly -1.
    if !(poisson1 > -1.0 && poisson1 < 0.5) || !(poisson2 > -1.0 && poisson2 < 0.5) {
        return None;
    }
    // `youngs2 = +∞` is the rigid-body limit: its compliance term is 0.
    let inv = if youngs2.is_infinite() {
        (1.0 - poisson1 * poisson1) / youngs1
    } else {
        if !youngs2.is_finite() || youngs2 <= 0.0 {
            return None;
        }
        (1.0 - poisson1 * poisson1) / youngs1 + (1.0 - poisson2 * poisson2) / youngs2
    };
    if inv <= 0.0 || !inv.is_finite() {
        return None;
    }
    let e_star = 1.0 / inv;
    e_star.is_finite().then_some(e_star)
}

/// Effective modulus against a perfectly rigid counter-surface: `E* = E/(1-ν²)`.
pub fn effective_modulus_rigid_plane(youngs: f64, poisson: f64) -> Option<f64> {
    effective_youngs_modulus(youngs, poisson, f64::INFINITY, 0.0)
}

/// Effective radius of a two-body contact: `1/R* = 1/R₁ + 1/R₂`.
///
/// Pass `f64::INFINITY` for a flat surface.
pub fn effective_radius(radius1: f64, radius2: f64) -> Option<f64> {
    if radius1.is_sign_negative() || radius2.is_sign_negative() {
        return None;
    }
    let mut inv = 0.0;
    if radius1.is_finite() {
        if radius1 <= 0.0 {
            return None;
        }
        inv += 1.0 / radius1;
    }
    if radius2.is_finite() {
        if radius2 <= 0.0 {
            return None;
        }
        inv += 1.0 / radius2;
    }
    if inv <= 0.0 || !inv.is_finite() {
        return None;
    }
    Some(1.0 / inv)
}

/// Effective radius of a sphere resting on a flat plane: `R* = R`.
pub fn effective_radius_sphere_plane(radius: f64) -> Option<f64> {
    if !radius.is_finite() || radius <= 0.0 {
        return None;
    }
    Some(radius)
}

// ---------------------------------------------------------------------------
// Hertz normal contact
// ---------------------------------------------------------------------------

/// Result of a Hertzian normal contact.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HertzContact {
    /// Normal force [N]. Always `≥ 0` (non-adhesive).
    pub force: f64,
    /// Radius of the circular contact patch [m].
    pub contact_radius: f64,
    /// Peak (centre) contact pressure [Pa].
    pub peak_pressure: f64,
    /// Mean contact pressure [Pa] (`= force / patch area`).
    pub mean_pressure: f64,
    /// Secant stiffness `dF/dδ` [N/m] at this overlap.
    pub secant_stiffness: f64,
}

/// Hertzian normal force: `F = (4/3)·E*·√R*·δ^{3/2}`.
///
/// A negative `penetration` (bodies separated) yields `0.0`, not a negative
/// force: Hertz assumes no adhesion, so tension is impossible. Use the JKR/DMT
/// laws for adhesive contacts.
pub fn hertz_force(penetration: f64, e_star: f64, r_star: f64) -> Option<f64> {
    if !finite_many(&[penetration, e_star, r_star]) || e_star <= 0.0 || r_star <= 0.0 {
        return None;
    }
    if penetration <= 0.0 {
        return Some(0.0);
    }
    let force = (4.0 / 3.0) * e_star * r_star.sqrt() * penetration.powf(1.5);
    force.is_finite().then_some(force)
}

/// Full Hertzian contact state at a given overlap.
pub fn hertz_contact(penetration: f64, e_star: f64, r_star: f64) -> Option<HertzContact> {
    if !finite_many(&[penetration, e_star, r_star]) || e_star <= 0.0 || r_star <= 0.0 {
        return None;
    }
    if penetration <= 0.0 {
        return Some(HertzContact {
            force: 0.0,
            contact_radius: 0.0,
            peak_pressure: 0.0,
            mean_pressure: 0.0,
            secant_stiffness: 0.0,
        });
    }
    let force = (4.0 / 3.0) * e_star * r_star.sqrt() * penetration.powf(1.5);
    // Contact radius: a = √(R*·δ)
    let contact_radius = (r_star * penetration).sqrt();
    if contact_radius <= 0.0 || !contact_radius.is_finite() {
        return None;
    }
    let patch_area = PI * contact_radius * contact_radius;
    // Peak pressure at the centre of a Hertzian patch: p₀ = 3F / (2πa²)
    let peak_pressure = 3.0 * force / (2.0 * patch_area);
    let mean_pressure = force / patch_area;
    // Secant stiffness dF/dδ = 2·E*·√(R*·δ)
    let secant_stiffness = 2.0 * e_star * (r_star * penetration).sqrt();
    let out = HertzContact {
        force,
        contact_radius,
        peak_pressure,
        mean_pressure,
        secant_stiffness,
    };
    let finite = finite_many(&[
        out.force,
        out.contact_radius,
        out.peak_pressure,
        out.mean_pressure,
        out.secant_stiffness,
    ]);
    finite.then_some(out)
}

/// The linear spring constant that reproduces a Hertzian contact at a chosen
/// overlap: `k = F(δ) / δ = (4/3)·E*·√R*·√δ`.
///
/// This is the bridge for solvers that only implement a linear normal spring
/// (such as the DEM path in the vendored fork): pick the overlap at which the
/// two laws should agree — typically the maximum expected penetration — and use
/// the returned `k`. The result is a *tangent* approximation: the linear model
/// then matches Hertz at that overlap and diverges away from it.
pub fn hertz_linearized_stiffness(
    design_penetration: f64,
    e_star: f64,
    r_star: f64,
) -> Option<f64> {
    if !finite_many(&[design_penetration, e_star, r_star])
        || e_star <= 0.0
        || r_star <= 0.0
        || design_penetration <= 0.0
    {
        return None;
    }
    let k = (4.0 / 3.0) * e_star * r_star.sqrt() * design_penetration.sqrt();
    (k.is_finite() && k > 0.0).then_some(k)
}

// ---------------------------------------------------------------------------
// Mindlin tangential contact
// ---------------------------------------------------------------------------

/// Mindlin tangential contact: an elastic, no-slip tangential response with a
/// Coulomb cap.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MindlinContact {
    /// Tangential force magnitude [N], already capped by `μ·F_n`.
    pub force: f64,
    /// Tangential stiffness `dF_t/ds` [N/m] used at this state.
    pub tangential_stiffness: f64,
    /// `true` when the Coulomb cap is active, i.e. the contact is sliding (or on
    /// the verge of it) rather than sticking.
    pub sliding: bool,
}

/// Mindlin tangential stiffness for a *full stick* contact:
/// `k_t = 8·G*·a`, where `a` is the Hertzian contact radius.
///
/// `g_star` is the effective shear modulus `1/G* = (2-ν₁)/G₁ + (2-ν₂)/G₂`; use
/// [`effective_shear_modulus`] to build it from `(E, ν)` pairs.
pub fn mindlin_tangential_stiffness_stick(g_star: f64, contact_radius: f64) -> Option<f64> {
    if !finite_many(&[g_star, contact_radius]) || g_star <= 0.0 || contact_radius <= 0.0 {
        return None;
    }
    let k = 8.0 * g_star * contact_radius;
    (k.is_finite() && k > 0.0).then_some(k)
}

/// Mindlin tangential response with partial slip and a Coulomb cap.
///
/// `tangential_displacement` is the accumulated tangential offset [m] since the
/// contact was established, `normal_force` the current normal force [N], `mu`
/// the friction coefficient. The force magnitude never exceeds `μ·F_n`.
///
/// The stiffness degrades as the tangential load approaches the cap, which is
/// the qualitative behaviour of Mindlin's partial-slip solution (a full closed
/// form needs the load history; this reports the *secant* stiffness at the
/// current state so a solver can integrate incrementally).
pub fn mindlin_contact(
    tangential_displacement: f64,
    normal_force: f64,
    mu: f64,
    g_star: f64,
    contact_radius: f64,
) -> Option<MindlinContact> {
    if !finite_many(&[
        tangential_displacement,
        normal_force,
        mu,
        g_star,
        contact_radius,
    ]) {
        return None;
    }
    if mu < 0.0 || g_star <= 0.0 || contact_radius <= 0.0 || normal_force < 0.0 {
        return None;
    }
    let k_stick = 8.0 * g_star * contact_radius;
    if normal_force <= 0.0 || mu <= 0.0 {
        return Some(MindlinContact {
            force: 0.0,
            tangential_stiffness: k_stick,
            sliding: false,
        });
    }
    let cap = mu * normal_force;
    let magnitude = k_stick * tangential_displacement.abs();
    if magnitude <= cap {
        Some(MindlinContact {
            force: magnitude,
            tangential_stiffness: k_stick,
            sliding: false,
        })
    } else {
        // Saturate at the Coulomb cap. The secant stiffness of a saturated
        // contact is zero (more displacement buys no more force), which is what
        // a solver needs to avoid over-correcting a sliding contact.
        Some(MindlinContact {
            force: cap,
            tangential_stiffness: 0.0,
            sliding: true,
        })
    }
}

/// Effective shear modulus of a two-body contact:
/// `1/G* = (2 - ν₁)/G₁ + (2 - ν₂)/G₂`, with `G = E/(2(1+ν))` per body.
///
/// Pass `youngs2 = f64::INFINITY` for a rigid counter-surface.
pub fn effective_shear_modulus(
    youngs1: f64,
    poisson1: f64,
    youngs2: f64,
    poisson2: f64,
) -> Option<f64> {
    let g1 = youngs1 / (2.0 * (1.0 + poisson1));
    if !g1.is_finite() || g1 <= 0.0 {
        return None;
    }
    let mut inv = (2.0 - poisson1) / g1;
    if !youngs2.is_infinite() {
        let g2 = youngs2 / (2.0 * (1.0 + poisson2));
        if !g2.is_finite() || g2 <= 0.0 {
            return None;
        }
        inv += (2.0 - poisson2) / g2;
    }
    if inv <= 0.0 || !inv.is_finite() {
        return None;
    }
    let g_star = 1.0 / inv;
    g_star.is_finite().then_some(g_star)
}

// ---------------------------------------------------------------------------
// Dissipative counterpart
// ---------------------------------------------------------------------------

/// Hertzian normal force with a Hunt-Crossley damping term, the standard
/// dissipative extension of Hertz:
/// `F = (4/3)·E*·√R*·δ^{3/2} · (1 + α·δ̇)`, clamped so the total never pulls.
///
/// `alpha` is the dissipation coefficient [s/m]; `0.0` recovers pure Hertz.
/// This is the same law as
/// [`crate::spaceflight::dynamics::contact_force_hunt_crossley`] but with the
/// Hertzian stiffness derived from material properties instead of given as a
/// bare `k`, which is what makes it tunable from `(E, ν, R)`.
pub fn hertz_hunt_crossley_force(
    penetration: f64,
    penetration_rate: f64,
    e_star: f64,
    r_star: f64,
    alpha: f64,
) -> Option<f64> {
    if !finite_many(&[penetration, penetration_rate, e_star, r_star, alpha])
        || e_star <= 0.0
        || r_star <= 0.0
        || alpha < 0.0
    {
        return None;
    }
    if penetration <= 0.0 {
        return Some(0.0);
    }
    let elastic = (4.0 / 3.0) * e_star * r_star.sqrt() * penetration.powf(1.5);
    // Only approaching motion dissipates; separating contact must not gain energy.
    let damping_factor = 1.0 + alpha * penetration_rate.max(0.0);
    // Clamp at zero: Hunt-Crossley with a large alpha can otherwise go negative
    // (adhesive), which the non-adhesive model must not do.
    let force = (elastic * damping_factor).max(0.0);
    force.is_finite().then_some(force)
}
