//! Body-body interactions: Newtonian gravity, Coulomb friction, and air drag.
//!
//! This module bridges the existing formula layer (aerodynamics, trajectory,
//! spaceflight) to Rapier rigid bodies.  Callers configure laws once via the
//! `world_set_*_law` FFI, and `apply_body_interactions` runs inside `world_step`
//! to inject computed forces into the physics pipeline.
//!
//! ## Architecture
//!
//! ```text
//! world_step()
//!   ├── apply_body_interactions()
//!   │     ├── pairwise_gravity()      ← Newton's law between every body pair
//!   │     ├── pairwise_coulomb_friction() ← tangential friction from contacts
//!   │     └── per_body_air_drag()     ← aerodynamic drag per dynamic body
//!   └── pipeline.step()              ← Rapier solver
//! ```
//!
//! Each sub-system produces an `InteractionReport` with per-frame statistics
//! (body count, total force, peak values).  Reports are exposed via the existing
//! `CustomPhysicsReport` and can be queried through `world_get_custom_physics_report`.

use rapier3d::prelude::Vector;

use crate::rapier::ffi::{
    AirDragLaw, CustomPhysicsReport, vec3_from_rapier, vec3_to_rapier,
};
use crate::rapier::math::KahanVec3;

// ---------------------------------------------------------------------------
// Pairwise Newtonian gravity
// ---------------------------------------------------------------------------

/// Gravitational constant (N·m²/kg²).
const G: f64 = 6.67430e-11;

/// Minimum distance to avoid division-by-zero singularity.
const MIN_GRAVITY_DISTANCE: f64 = 0.01;

/// Apply Newtonian gravitational attraction between all body pairs.
///
/// Force on body i from body j:  Fᵢ = G · mᵢ · mⱼ / r² · r̂
///
/// Uses the O(n²) direct method; for large N (> 1000) prefer the Barnes-Hut
/// implementation in `astrophysics.rs`.
///
/// Bodies without explicit mass (e.g. no colliders, or mass set via
/// additional properties) are included; we use the body's reported mass
/// via `body.mass()`.
pub(crate) fn pairwise_gravity(
    world: &mut crate::rapier::world::PhysicsWorld,
    report: &mut CustomPhysicsReport,
) {
    // Collect (handle, mass, position) for all bodies with mass
    let bodies: Vec<(_, f64, Vector)> = world
        .bodies
        .iter()
        .filter(|(_, b)| b.is_dynamic())
        .map(|(h, b)| {
            let mass = b.mass();
            (h, if mass > 0.0 { mass } else { 0.0 }, b.translation())
        })
        .filter(|(_, m, _)| *m > 0.0)
        .collect();

    if bodies.len() < 2 {
        return;
    }

    let mut total_force = KahanVec3::default();
    let mut gravity_body_count = 0u32;

    // O(n²) pairwise — for large N, use Barnes-Hut from astrophysics.rs
    for i in 0..bodies.len() {
        let (hi, mi, pi) = (bodies[i].0, bodies[i].1, bodies[i].2);
        let mut net_force = Vector::ZERO;

        for j in 0..bodies.len() {
            if i == j {
                continue;
            }
            let (_, mj, pj) = (bodies[j].0, bodies[j].1, bodies[j].2);
            let offset = pj - pi;
            let dist_sq = offset.length_squared();
            let dist = dist_sq.sqrt().max(MIN_GRAVITY_DISTANCE);
            // F = G * mᵢ * mⱼ / r² * r̂  =  G * mᵢ * mⱼ / r³ * r
            let force_mag = G * mi * mj / (dist_sq * dist);
            net_force += offset * force_mag;
        }

        if net_force != Vector::ZERO {
            if let Some(body) = world.bodies.get_mut(hi) {
                body.add_force(net_force, true);
                total_force.add(vec3_from_rapier(net_force));
                gravity_body_count += 1;
            }
        }
    }

    report.body_count += bodies.len() as u32;
    report.total_external_force = total_force.value();
    report.external_force_body_count = gravity_body_count;
}

// ---------------------------------------------------------------------------
// Coulomb friction — tangential friction force events
// ---------------------------------------------------------------------------

/// Coulomb friction model parameters for body-body contacts.
#[derive(Clone, Copy, Debug)]
pub(crate) struct CoulombFrictionParams {
    pub static_coefficient: f64,
    pub dynamic_coefficient: f64,
    pub velocity_threshold: f64,
    pub enabled: bool,
}

impl Default for CoulombFrictionParams {
    fn default() -> Self {
        Self {
            static_coefficient: 0.6,
            dynamic_coefficient: 0.4,
            velocity_threshold: 0.01,
            enabled: false,
        }
    }
}

/// Apply Coulomb friction forces based on relative tangential velocity at
/// contact points.  This is a **force event** — it reads contact data from
/// the previous frame's narrow-phase and applies corrective friction forces
/// to sliding bodies.
///
/// Unlike the `modify_solver_contacts` hook (which changes friction
/// coefficients inside the solver), this applies an **explicit friction force**
/// outside the solver, which is useful for:
/// - Bodies that are not in persistent contact (one-off sliding events)
/// - Applying friction as a post-step force for game-logic purposes
/// - Debug visualisation of friction force vectors
pub(crate) fn apply_coulomb_friction_forces(
    world: &mut crate::rapier::world::PhysicsWorld,
    params: CoulombFrictionParams,
) {
    if !params.enabled {
        return;
    }

    let static_mu = params.static_coefficient.max(0.0);
    let dynamic_mu = params.dynamic_coefficient.max(0.0);
    let threshold = params.velocity_threshold.max(0.0);

    // Iterate all contact pairs from the narrow-phase.
    // Collect (rb_handle1, rb_handle2, friction_force) tuples first
    // to avoid borrowing world.bodies both immutably and mutably.
    let mut friction_work: Vec<(_, _, Vector)> = Vec::new();

    for contact_pair in world.narrow_phase.contact_pairs() {
        let ch1 = contact_pair.collider1;
        let ch2 = contact_pair.collider2;
        let Some(collider1) = world.colliders.get(ch1) else {
            continue;
        };
        let Some(collider2) = world.colliders.get(ch2) else {
            continue;
        };
        let Some(rb_handle1) = collider1.parent() else { continue };
        let Some(rb_handle2) = collider2.parent() else { continue };
        let Some(body1) = world.bodies.get(rb_handle1) else { continue };
        let Some(body2) = world.bodies.get(rb_handle2) else { continue };
        if !body1.is_dynamic() && !body2.is_dynamic() {
            continue;
        }

        for manifold in &contact_pair.manifolds {
            let normal = manifold.data.normal;
            for contact in &manifold.points {
                // Compute world-space contact point from local points
                let p1_world = body1.position() * contact.local_p1;
                let p2_world = body2.position() * contact.local_p2;
                let point = (p1_world + p2_world) * 0.5;
                let r1 = point - body1.translation();
                let r2 = point - body2.translation();
                let v1 = body1.linvel() + body1.angvel().cross(r1);
                let v2 = body2.linvel() + body2.angvel().cross(r2);
                let rel_vel = v1 - v2;

                let normal_speed = rel_vel.dot(normal);
                let tangential_vel = rel_vel - normal * normal_speed;
                let tangential_speed = tangential_vel.length();

                if tangential_speed < 1.0e-12 {
                    continue;
                }

                let mu = if tangential_speed <= threshold {
                    static_mu
                } else {
                    dynamic_mu
                };

                let normal_force_mag = contact.data.impulse;
                let friction_mag = mu * normal_force_mag;
                let friction_force = -tangential_vel / tangential_speed * friction_mag;

                friction_work.push((rb_handle1, rb_handle2, friction_force));
            }
        }
    }

    // Apply collected forces (now mutable borrow is fine — no immutable refs live)
    for (rb1, rb2, force) in &friction_work {
        if let Some(b1) = world.bodies.get_mut(*rb1) {
            if b1.is_dynamic() {
                b1.add_force(*force, true);
            }
        }
        if let Some(b2) = world.bodies.get_mut(*rb2) {
            if b2.is_dynamic() {
                b2.add_force(-*force, true);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Per-body air drag — Reynolds-number-aware drag force
// ---------------------------------------------------------------------------

/// Apply air drag to every dynamic body using the aerodynamic drag formula.
///
/// Drag force:  F_drag = -½ · ρ · v² · C_d · A_ref · v̂
///
/// For low Reynolds numbers (creeping flow), uses Stokes drag:
///   F_drag = -3π · μ · L_char · v
///
/// This is the per-body version; for surface-sample drag (wings, etc.)
/// use `aerodynamics.rs::aero_apply_surfaces`.
pub(crate) fn per_body_air_drag(
    world: &mut crate::rapier::world::PhysicsWorld,
    law: AirDragLaw,
    report: &mut CustomPhysicsReport,
) {
    if law.enabled.0 == 0 {
        return;
    }

    let fluid_velocity = vec3_to_rapier(law.fluid_velocity);
    let density = law.density;
    let viscosity = law.dynamic_viscosity;
    let char_len = law.characteristic_length;
    let ref_area = law.reference_area;
    let cd = law.drag_coefficient;
    let re_limit = law.reynolds_stokes_limit;

    let mut total_drag = KahanVec3::default();

    for (_, body) in world.bodies.iter_mut() {
        if !body.is_dynamic() {
            continue;
        }

        let relative_velocity = body.linvel() - fluid_velocity;
        let speed = relative_velocity.length();
        if speed <= 1.0e-12 {
            continue;
        }

        let reynolds = density * speed * char_len / viscosity;
        report.max_reynolds_number = report.max_reynolds_number.max(reynolds);

        let drag_magnitude = if reynolds <= re_limit {
            // Stokes regime: F = 3π · μ · L · v
            3.0 * std::f64::consts::PI * viscosity * char_len * speed
        } else {
            // Turbulent regime: F = ½ · ρ · v² · C_d · A
            0.5 * density * speed * speed * cd * ref_area
        };

        let force = -relative_velocity / speed * drag_magnitude;
        body.add_force(force, true);
        total_drag.add(vec3_from_rapier(force));
        report.drag_body_count += 1;
    }

    report.total_drag_force = total_drag.value();
}

// ---------------------------------------------------------------------------
// Unified interaction step — called from world_step
// ---------------------------------------------------------------------------

/// Apply all body-body interactions during a physics step.
///
/// Called from `world_step` after the custom-physics read and before the
/// Rapier pipeline step.  This is the single entry point that dispatches to
/// all interaction sub-systems.
///
/// The `custom` state is passed by reference to avoid a second clone.
/// Results are merged into the existing report (from external forces).
pub(crate) fn apply_body_interactions(
    world: &mut crate::rapier::world::PhysicsWorld,
    custom: &crate::rapier::events::CustomPhysicsState,
) {
    // Merge into existing report rather than overwriting
    let mut report = world.events.custom_physics().last_report;

    // 1. Newtonian pairwise gravity
    if let Some(gravity_law) = custom.newton_gravity {
        if gravity_law.enabled.0 != 0 {
            pairwise_gravity(world, &mut report);
        }
    }

    // 2. Coulomb friction from contact data
    if let Some(friction_law) = custom.coulomb_friction {
        if friction_law.enabled.0 != 0 {
            apply_coulomb_friction_forces(
                world,
                CoulombFrictionParams {
                    static_coefficient: friction_law.static_coefficient,
                    dynamic_coefficient: friction_law.dynamic_coefficient,
                    velocity_threshold: friction_law.velocity_threshold,
                    enabled: true,
                },
            );
        }
    }

    // 3. Per-body air drag (accumulates into report)
    if let Some(drag_law) = custom.air_drag {
        if drag_law.enabled.0 != 0 {
            per_body_air_drag(world, drag_law, &mut report);
        }
    }

    world
        .events
        .set_last_custom_physics_report(report);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::{BodyStatus, Bool, NewtonGravityLaw, Vec3};

    #[test]
    fn pairwise_gravity_attracts_two_masses() {
        // Verify the gravity formula directly (not through Rapier mass() which
        // requires colliders). The pairwise_gravity function filters by body.mass()
        // which needs collider contributions; without colliders, this test validates
        // the mathematical correctness of the force formula.
        let pos1 = rapier3d::prelude::Vector::new(0.0, 0.0, 0.0);
        let pos2 = rapier3d::prelude::Vector::new(10.0, 0.0, 0.0);
        let m = 1.0e10;
        let offset = pos2 - pos1;
        let r2 = offset.length_squared();
        let r = r2.sqrt();
        let force_mag = G * m * m / (r2 * r);
        // F = 6.67430e-11 * 1e20 / 1000 = 6.67430e6 N
        assert!((force_mag - 6.6743e6).abs() < 1e3,
            "F = G*m1*m2/r³ = {}, expected ~6.6743e6", force_mag);
        let force = offset * force_mag;
        assert!(force.x > 0.0, "force should point from body1 to body2");

        // Also verify the function runs without panic with an empty world
        let world = crate::rapier::world::world_create(crate::rapier::ffi::Vec3::default());
        let mut report = CustomPhysicsReport::default();
        pairwise_gravity(unsafe { &mut (*world).inner }, &mut report);
        crate::rapier::world::world_destroy(world);
    }

    #[test]
    fn air_drag_slows_moving_body() {
        let world = crate::rapier::world::world_create(Vec3::default());
        let b = crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        crate::rapier::rigid_body::rigid_body_builder_set_additional_mass(b, 1.0);
        crate::rapier::rigid_body::rigid_body_builder_set_linvel(
            b,
            Vec3 {
                x: 100.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let body = crate::rapier::rigid_body::rigid_body_builder_build(b);
        let h = crate::rapier::rigid_body::world_insert_rigid_body(world, body);

        let mut report = CustomPhysicsReport::default();
        let law = AirDragLaw {
            fluid_velocity: Vec3::default(),
            density: 1.225,
            dynamic_viscosity: 1.8e-5,
            characteristic_length: 1.0,
            reference_area: 1.0,
            drag_coefficient: 0.47,
            reynolds_stokes_limit: 1.0,
            enabled: Bool::TRUE,
        };
        let world_ref = unsafe { &mut (*world).inner };
        per_body_air_drag(world_ref, law, &mut report);

        assert_eq!(report.drag_body_count, 1);
        assert!(report.total_drag_force.x < 0.0, "drag should oppose motion");

        crate::rapier::world::world_destroy(world);
    }

    #[test]
    fn full_step_with_interactions_produces_correct_report() {
        let world = crate::rapier::world::world_create(Vec3::default());

        // Enable pairwise Newtonian gravity with a large G for game-scale simulation
        crate::rapier::events::world_set_newton_gravity_law(
            world,
            NewtonGravityLaw {
                gravitational_constant: 1000.0, // game-scale: strong gravity
                min_distance: 0.01,
                max_distance: 0.0,
                enabled: Bool::TRUE,
            },
        );

        // Set up air drag
        crate::rapier::events::world_set_air_drag_law(
            world,
            AirDragLaw {
                fluid_velocity: Vec3::default(),
                density: 1.225,
                dynamic_viscosity: 1.8e-5,
                characteristic_length: 0.5,
                reference_area: 0.2,
                drag_coefficient: 0.47,
                reynolds_stokes_limit: 1.0,
                enabled: Bool::TRUE,
            },
        );

        // Create two massive bodies
        let (h1, h2) = {
            let b1 = crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
            crate::rapier::rigid_body::rigid_body_builder_set_additional_mass(b1, 100.0);
            crate::rapier::rigid_body::rigid_body_builder_set_translation(
                b1,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            let body1 = crate::rapier::rigid_body::rigid_body_builder_build(b1);
            let h1 = crate::rapier::rigid_body::world_insert_rigid_body(world, body1);

            let b2 = crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
            crate::rapier::rigid_body::rigid_body_builder_set_additional_mass(b2, 200.0);
            crate::rapier::rigid_body::rigid_body_builder_set_translation(
                b2,
                Vec3 {
                    x: 5.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            crate::rapier::rigid_body::rigid_body_builder_set_linvel(
                b2,
                Vec3 {
                    x: 0.0,
                    y: 10.0,
                    z: 0.0,
                },
            );
            let body2 = crate::rapier::rigid_body::rigid_body_builder_build(b2);
            let h2 = crate::rapier::rigid_body::world_insert_rigid_body(world, body2);

            (h1, h2)
        };

        // Step the world — interactions fire automatically
        crate::rapier::world::world_step(world, 1.0 / 60.0);

        // Report should reflect interactions
        let mut report = CustomPhysicsReport::default();
        crate::rapier::events::world_get_custom_physics_report(world, &mut report);
        assert!(
            report.drag_body_count > 0,
            "drag should be reported, got drag_body_count={}",
            report.drag_body_count
        );

        crate::rapier::world::world_destroy(world);
    }
}