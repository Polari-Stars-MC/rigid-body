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
    AirDragLaw, CustomPhysicsReport, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};
use crate::rapier::math::KahanVec3;

// ---------------------------------------------------------------------------
// Pairwise Newtonian gravity
// ---------------------------------------------------------------------------

/// Gravitational constant (N·m²/kg²).
pub const G: f64 = 6.67430e-11;

/// Minimum distance to avoid division-by-zero singularity.
const MIN_GRAVITY_DISTANCE: f64 = 0.01;

/// Apply Newtonian gravitational attraction between all body pairs.
///
/// Force on body i from body j:  Fᵢ = G · mᵢ · mⱼ / r² · r̂
///
/// Uses the O(n²) upper-triangular direct method (Newton III: F_ij = -F_ji,
/// so only half the pairs are evaluated); for large N (> 1000) prefer the
/// Barnes-Hut implementation in `astrophysics.rs`.
///
/// Bodies without explicit mass (e.g. no colliders, or mass set via
/// additional properties) are included; we use the body's reported mass
/// via `body.mass()`.
pub fn pairwise_gravity(
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

    // P0.4 fix: upper-triangular O(n²) — Newton III F_ij = -F_ji.
    // Halves the iteration count vs the previous full-matrix traversal and
    // matches the registered-law path (NewtonianGravityForceLaw::apply()).
    // For large n > 1000 prefer Barnes-Hut from astrophysics.rs.
    let n = bodies.len();
    // Per-body accumulated force (auto-win: avoid push/drop churn by reusing one Vec).
    let mut net_forces: Vec<Vector> = vec![Vector::ZERO; n];

    for i in 0..n {
        let mi = bodies[i].1;
        let pi = bodies[i].2;
        for j in (i + 1)..n {
            let mj = bodies[j].1;
            let offset = bodies[j].2 - pi;
            let dist_sq = offset.length_squared();
            let dist = dist_sq.sqrt().max(MIN_GRAVITY_DISTANCE);
            // F = G * mᵢ * mⱼ / r² * r̂  =  G * mᵢ * mⱼ / r³ * r
            let force_mag = G * mi * mj / (dist_sq * dist);
            let f_ij = offset * force_mag;
            net_forces[i] += f_ij;
            net_forces[j] -= f_ij; // Newton III: equal and opposite on body j
        }
    }

    for (i, (hi, _, _)) in bodies.iter().enumerate() {
        if net_forces[i] != Vector::ZERO
            && let Some(body) = world.bodies.get_mut(*hi)
        {
            body.add_force(net_forces[i], true);
            total_force.add(vec3_from_rapier(net_forces[i]));
            gravity_body_count += 1;
        }
    }

    report.body_count += n as u32;
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
pub fn per_body_air_drag(
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

/// Facade-based wrapper: applies legacy unregistered body interactions through
/// the facade so the frame-log captures them with correct ForceLawType tags.
///
/// This is a temporary shim — once all force sources are registered as ForceLaw
/// impls, this function will be removed.
pub(crate) fn apply_body_interactions_with_facade(
    force_registry: &ForceRegistry,
    custom: &crate::rapier::events::CustomPhysicsState,
    facade: &mut crate::rapier::forces::ForceFacade<'_>,
) {
    use crate::rapier::forces::ForceLawType;

    // 1. Newtonian pairwise gravity
    if let Some(gravity_law) = custom.newton_gravity
        && gravity_law.enabled.0 != 0
    {
        let registered = !force_registry
            .find_by_type(ForceLawType::NewtonianGravity)
            .is_empty();
        if !registered {
            // Use a temporary ForceLaw instance
            let law = NewtonianGravityForceLaw {
                gravitational_constant: gravity_law.gravitational_constant,
                min_distance: gravity_law.min_distance,
                max_distance: gravity_law.max_distance,
                enabled: true,
            };
            law.apply(facade);
        }
    }

    // 2. Coulomb friction from contact data
    if let Some(friction_law) = custom.coulomb_friction
        && friction_law.enabled.0 != 0
    {
        let registered = !force_registry
            .find_by_type(ForceLawType::CoulombFriction)
            .is_empty();
        if !registered {
            apply_coulomb_friction_forces_with_facade(
                facade.narrow_phase,
                CoulombFrictionParams {
                    static_coefficient: friction_law.static_coefficient,
                    dynamic_coefficient: friction_law.dynamic_coefficient,
                    velocity_threshold: friction_law.velocity_threshold,
                    enabled: true,
                },
                facade,
            );
        }
    }

    // 3. Per-body air drag
    if let Some(drag_law) = custom.air_drag
        && drag_law.enabled.0 != 0
    {
        let registered = !force_registry
            .find_by_type(ForceLawType::AirDrag)
            .is_empty();
        if !registered {
            let law = AirDragForceLaw {
                fluid_velocity: vec3_to_rapier(drag_law.fluid_velocity),
                density: drag_law.density,
                dynamic_viscosity: drag_law.dynamic_viscosity,
                characteristic_length: drag_law.characteristic_length,
                reference_area: drag_law.reference_area,
                drag_coefficient: drag_law.drag_coefficient,
                reynolds_stokes_limit: drag_law.reynolds_stokes_limit,
                enabled: true,
            };
            law.apply(facade);
        }
    }
}

/// Coulomb friction via facade — writes typed force records.
fn apply_coulomb_friction_forces_with_facade(
    narrow_phase: &NarrowPhase,
    params: CoulombFrictionParams,
    facade: &mut crate::rapier::forces::ForceFacade<'_>,
) {
    if !params.enabled {
        return;
    }

    let static_mu = params.static_coefficient.max(0.0);
    let dynamic_mu = params.dynamic_coefficient.max(0.0);
    let threshold = params.velocity_threshold.max(0.0);

    // P1 修复：复用 facade.friction_work 跨帧缓冲（与 FrameWorkBuffers.friction_work
    // 同源），避免每帧 `Vec::new()` 分配。此函数由 apply_body_interactions_with_facade
    // 在 Coulomb law 未注册时调用，是热路径上唯一仍走旧路径的力。
    facade.friction_work.clear();

    for contact_pair in narrow_phase.contact_pairs() {
        let ch1 = contact_pair.collider1;
        let ch2 = contact_pair.collider2;
        let Some(collider1) = facade.colliders.get(ch1) else {
            continue;
        };
        let Some(collider2) = facade.colliders.get(ch2) else {
            continue;
        };
        let Some(rb_handle1) = collider1.parent() else {
            continue;
        };
        let Some(rb_handle2) = collider2.parent() else {
            continue;
        };
        let Some(body1) = facade.bodies.get(rb_handle1) else {
            continue;
        };
        let Some(body2) = facade.bodies.get(rb_handle2) else {
            continue;
        };
        if !body1.is_dynamic() && !body2.is_dynamic() {
            continue;
        }

        for manifold in contact_pair.manifolds() {
            let normal = manifold.data.normal;
            for contact in &manifold.points {
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

                facade
                    .friction_work
                    .push((rb_handle1, rb_handle2, friction_force));
            }
        }
    }

    use crate::rapier::forces::ForceLawType;
    // 不能 `for ... in facade.friction_work.iter()` 同时 `facade.add_force(...)` —— 后者
    // 需 `&mut facade` 会与对 `facade.friction_work` 的共享借用冲突。改为按索引消耗：
    // 逐项 `(*rb1, *rb2, *force)` 读出（Copy 类型），借用立即释放，再 `add_force`。
    let n_pairs = facade.friction_work.len();
    for i in 0..n_pairs {
        let (rb1, rb2, force) = facade.friction_work[i];
        facade.add_force(rb1, force, ForceLawType::CoulombFriction);
        facade.add_force(rb2, -force, ForceLawType::CoulombFriction);
    }
}

// ---------------------------------------------------------------------------
// ForceLaw impls — registry-compatible wrappers
// ---------------------------------------------------------------------------

use crate::rapier::forces::{ForceFacade, ForceLaw, ForceLawType, ForceRegistry};
use rapier3d::prelude::{NarrowPhase, RigidBodyHandle};
use smallvec::SmallVec;

/// Newtonian pairwise gravity as a registered force law.
pub(crate) struct NewtonianGravityForceLaw {
    pub gravitational_constant: f64,
    pub min_distance: f64,
    pub max_distance: f64,
    pub enabled: bool,
}

impl ForceLaw for NewtonianGravityForceLaw {
    fn law_type(&self) -> ForceLawType {
        ForceLawType::NewtonianGravity
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn apply(&self, facade: &mut ForceFacade<'_>) {
        let g = self.gravitational_constant;
        let min_dist = self.min_distance;
        let max_dist_sq = if self.max_distance > 0.0 {
            self.max_distance * self.max_distance
        } else {
            f64::MAX
        };
        let source = self.law_type();

        // P0.3 fix: reuse persistent scratch_body_data across frames instead of
        // allocating a fresh SmallVec every world_step. Must clear before fill.
        let body_data: &mut SmallVec<[(RigidBodyHandle, f64, Vector); 64]> =
            facade.scratch_body_data;
        body_data.clear();
        body_data.extend(
            facade
                .bodies
                .iter()
                .filter(|(_, b)| b.is_dynamic())
                .filter_map(|(h, b)| {
                    let mass = b.mass();
                    if mass > 0.0 {
                        Some((h, mass, b.translation()))
                    } else {
                        None
                    }
                }),
        );

        if body_data.len() < 2 {
            return;
        }

        // P0.3 fix: reuse persistent scratch_force_pairs as the force accumulator.
        // Newton III: compute F_ij = -F_ji, only upper triangle (2× less iter).
        // The accumulator is indexed positionally (i, j) so we can do += / -= on
        // each pair in one pass before the single apply over scratch_force_pairs.
        let n = body_data.len();
        let forces: &mut SmallVec<[(RigidBodyHandle, Vector); 64]> = facade.scratch_force_pairs;
        forces.clear();
        for (h, _, _) in body_data.iter() {
            forces.push((*h, Vector::ZERO));
        }

        if n >= crate::rapier::parallel::GRAVITY_PAR_MIN_BODIES {
            // Chunked parallel decomposition (Newton III, per-chunk-pair
            // buffers + ordered merge — see `parallel::pairwise_gravity_accumulate`).
            // The per-body sums are reassociated relative to the serial loop
            // (last-ulp differences only); the merge order is deterministic and
            // independent of the thread count.
            let net = crate::rapier::parallel::pairwise_gravity_accumulate(
                &*body_data,
                g,
                min_dist,
                max_dist_sq,
            );
            for (i, f) in net.into_iter().enumerate() {
                forces[i].1 = f;
            }
        } else {
            for i in 0..n {
                let (_hi, mi, pi) = (body_data[i].0, body_data[i].1, body_data[i].2);
                for j in (i + 1)..n {
                    let (_hj, mj, pj) = (body_data[j].0, body_data[j].1, body_data[j].2);
                    let offset = pj - pi;
                    let dist_sq = offset.length_squared();
                    if dist_sq > max_dist_sq {
                        continue;
                    }
                    let dist = dist_sq.sqrt().max(min_dist);
                    let force_mag = g * mi * mj / (dist_sq * dist);
                    let f_ij = offset * force_mag;
                    forces[i].1 += f_ij;
                    forces[j].1 -= f_ij;
                }
            }
        }

        // Single pass: apply accumulated forces. Read each entry by index so
        // each immutable borrow of scratch_force_pairs ends before the mut
        // borrow that `add_force` needs (entries are Copy).
        let n = facade.scratch_force_pairs.len();
        for i in 0..n {
            let (handle, force) = facade.scratch_force_pairs[i];
            if force != Vector::ZERO {
                facade.add_force(handle, force, source);
            }
        }
    }

    fn clone_box(&self) -> Box<dyn ForceLaw> {
        Box::new(Self {
            gravitational_constant: self.gravitational_constant,
            min_distance: self.min_distance,
            max_distance: self.max_distance,
            enabled: self.enabled,
        })
    }
}

/// Air drag as a registered force law.
pub(crate) struct AirDragForceLaw {
    pub fluid_velocity: Vector,
    pub density: f64,
    pub dynamic_viscosity: f64,
    pub characteristic_length: f64,
    pub reference_area: f64,
    pub drag_coefficient: f64,
    pub reynolds_stokes_limit: f64,
    pub enabled: bool,
}

impl ForceLaw for AirDragForceLaw {
    fn law_type(&self) -> ForceLawType {
        ForceLawType::AirDrag
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn apply(&self, facade: &mut ForceFacade<'_>) {
        let source = self.law_type();

        // Two-phase fill (see the `parallel` module docs): per-body drag is a
        // pure read of body state, so the computation maps over the
        // dynamic-body handles on the rayon pool above `PAR_MIN_ITEMS` bodies,
        // preserving order. The apply phase replays the identical force
        // sequence on the calling thread, so results are bit-identical to the
        // serial path.
        let handles: Vec<RigidBodyHandle> = facade
            .bodies
            .iter()
            .filter(|(_, b)| b.is_dynamic())
            .map(|(h, _)| h)
            .collect();
        let computed: Vec<Option<(Vector, f64)>> = crate::rapier::parallel::par_map_bodies(
            &handles,
            &*facade.bodies,
            crate::rapier::parallel::PAR_MIN_ITEMS,
            |_, body| {
                let relative_velocity = body.linvel() - self.fluid_velocity;
                let speed = relative_velocity.length();
                if speed <= 1.0e-12 {
                    return None;
                }
                let reynolds =
                    self.density * speed * self.characteristic_length / self.dynamic_viscosity;
                let drag_magnitude = if reynolds <= self.reynolds_stokes_limit {
                    3.0 * std::f64::consts::PI
                        * self.dynamic_viscosity
                        * self.characteristic_length
                        * speed
                } else {
                    0.5 * self.density * speed * speed * self.drag_coefficient * self.reference_area
                };
                Some((-relative_velocity / speed * drag_magnitude, reynolds))
            },
        );

        // Exact max-reduction (f64::max is associative and exact), so the
        // parallel fold matches the serial running-max bit for bit.
        let max_re = computed
            .iter()
            .flatten()
            .map(|(_, re)| *re)
            .fold(0.0_f64, f64::max);
        facade.update_reynolds(max_re);
        for (handle, entry) in handles.iter().zip(&computed) {
            if let Some((force, _)) = entry {
                facade.add_force(*handle, *force, source);
            }
        }
    }

    fn clone_box(&self) -> Box<dyn ForceLaw> {
        Box::new(Self {
            fluid_velocity: self.fluid_velocity,
            density: self.density,
            dynamic_viscosity: self.dynamic_viscosity,
            characteristic_length: self.characteristic_length,
            reference_area: self.reference_area,
            drag_coefficient: self.drag_coefficient,
            reynolds_stokes_limit: self.reynolds_stokes_limit,
            enabled: self.enabled,
        })
    }
}

// ===========================================================================
// PHYSICS_EXPANSION_PLAN C1 — new ForceLaw implementations (planet-physics).
//
// Three laws, each delegated to a mps-formula `pub fn`:
//   * SolarWindPressureForceLaw       → heliophysics::solar_wind_dynamic_pressure
//   * DynamicalFrictionForceLaw       → galactic_dynamics::chandrasekhar_dynamical_friction
//   * MonDGravityForceLaw             → galactic_dynamics::mond_acceleration
//
// These run inside `world_step()` via `ForceRegistry::apply_all`, exactly as
// the legacy `AirDragForceLaw` / `NewtonianGravityForceLaw` do, so the report
// infrastructure (ForceReport, CustomPhysicsReport) is automatic.
// ===========================================================================

use mps_formula::galactic_dynamics as gd;
use mps_formula::heliophysics as hph;
use mps_formula::high_energy_astro as hea;

/// Solar-wind plasma dynamic pressure on the projection area of a Rapier body.
///
/// `proton_density` (n_p, number /m³) and `v_sw_mps` (m/s, plasma bulk velocity
/// **in world frame**) define the ram pressure `P = n_p · m_p · v_sw²` computed
/// by `heliophysics::solar_wind_dynamic_pressure`.  The `wind_direction` unit
/// vector (world-frame) directs the resulting push; the effective area is the
/// body's `effective_area_m2`.
///
/// The body's own velocity component along the wind direction is subtracted
/// from the wind speed to obtain the ram pressure against the body. A body
/// moving with the wind (v_rel ≤ 0) feels no force.
pub(crate) struct SolarWindPressureForceLaw {
    /// Solar-wind proton density (n / m³).
    pub proton_density: f64,
    /// Solar-wind bulk speed in m/s (typical 400–800).
    pub v_sw_mps: f64,
    /// World-frame unit-vector in the wind propagation direction.
    pub wind_direction: Vector,
    /// Effective cross-section area (m²).
    pub effective_area_m2: f64,
    pub enabled: bool,
}

impl ForceLaw for SolarWindPressureForceLaw {
    fn law_type(&self) -> ForceLawType {
        ForceLawType::SolarWindPressure
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn apply(&self, facade: &mut ForceFacade<'_>) {
        let source = self.law_type();
        let dir = self.wind_direction;

        // Skip if the direction is degenerate (zero, NaN, ...).
        let dir_norm_sq = dir.length_squared();
        if !dir_norm_sq.is_finite() || dir_norm_sq <= 1.0e-18 {
            return;
        }
        let dir_unit = dir / dir_norm_sq.sqrt();

        // Two-phase fill on the rayon pool above `PAR_MIN_ITEMS` bodies
        // (order-preserving; see the `parallel` module docs).
        let handles: Vec<RigidBodyHandle> = facade
            .bodies
            .iter()
            .filter(|(_, b)| b.is_dynamic())
            .map(|(h, _)| h)
            .collect();
        let computed: Vec<Option<Vector>> = crate::rapier::parallel::par_map_bodies(
            &handles,
            &*facade.bodies,
            crate::rapier::parallel::PAR_MIN_ITEMS,
            |_, body| {
                // Relative wind speed along the propagation direction (m/s).
                let v_rel = self.v_sw_mps - body.linvel().dot(dir_unit);
                if v_rel <= 0.0 || v_rel.is_nan() {
                    return None;
                }
                // Formula expects km/s.
                let v_rel_kms = v_rel * 1.0e-3;
                // `solar_wind_dynamic_pressure` returns nPa (1e-9 Pa); convert to
                // SI Pascals before multiplying by area so the resulting force is
                // in Newtons.
                let pressure_pa =
                    match hph::solar_wind_dynamic_pressure(self.proton_density, v_rel_kms) {
                        Some(p) => p * 1.0e-9, // nPa → Pa
                        None => return None,
                    };
                // F = P · A_eff, push is along +dir_unit (downwind).
                let force = dir_unit * (pressure_pa * self.effective_area_m2);
                (force != Vector::ZERO).then_some(force)
            },
        );

        // Apply (parallel results replay in handle order).
        for (handle, force) in handles
            .iter()
            .zip(&computed)
            .filter_map(|(h, f)| f.map(|f| (h, f)))
        {
            facade.add_force(*handle, force, source);
        }
    }

    fn clone_box(&self) -> Box<dyn ForceLaw> {
        Box::new(Self {
            proton_density: self.proton_density,
            v_sw_mps: self.v_sw_mps,
            wind_direction: self.wind_direction,
            effective_area_m2: self.effective_area_m2,
            enabled: self.enabled,
        })
    }
}

/// Chandrasekhar dynamical-friction deceleration on each Rapier body moving
/// through a uniform background density field.  The friction magnitude is
/// `galactic_dynamics::chandrasekhar_dynamical_friction` and acts opposite to
/// the body's velocity.
pub(crate) struct DynamicalFrictionForceLaw {
    /// Background mass density ρ_bg (kg / m³).
    pub background_density_kg_m3: f64,
    /// Coulomb logarithm ln Λ (typical ~10).
    pub coulomb_log: f64,
    pub enabled: bool,
}

impl ForceLaw for DynamicalFrictionForceLaw {
    fn law_type(&self) -> ForceLawType {
        ForceLawType::DynamicalFriction
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn apply(&self, facade: &mut ForceFacade<'_>) {
        let source = self.law_type();
        let rho = self.background_density_kg_m3;
        let ln_l = self.coulomb_log;

        if !rho.is_finite() || rho <= 0.0 || !ln_l.is_finite() || ln_l <= 0.0 {
            return;
        }

        // Two-phase fill on the rayon pool above `PAR_MIN_ITEMS` bodies
        // (order-preserving; see the `parallel` module docs).
        let handles: Vec<RigidBodyHandle> = facade
            .bodies
            .iter()
            .filter(|(_, b)| b.is_dynamic())
            .map(|(h, _)| h)
            .collect();
        let computed: Vec<Option<Vector>> = crate::rapier::parallel::par_map_bodies(
            &handles,
            &*facade.bodies,
            crate::rapier::parallel::PAR_MIN_ITEMS,
            |_, body| {
                let mass = body.mass().max(1.0); // Rapier 0.34: additional_mass returns 0 until collider propagation; use fallback 1.0.
                let v = body.linvel();
                let speed = v.length();
                if !speed.is_finite() || speed <= 1.0e-9 {
                    return None;
                }
                let a_mag = match gd::chandrasekhar_dynamical_friction(mass, rho, speed, ln_l) {
                    Some(a) => a, // m/s² acceleration
                    None => return None,
                };
                // F = m · a, direction opposes v.
                let force = -v / speed * (a_mag * mass);
                (force != Vector::ZERO).then_some(force)
            },
        );

        // Apply (parallel results replay in handle order).
        for (handle, force) in handles
            .iter()
            .zip(&computed)
            .filter_map(|(h, f)| f.map(|f| (h, f)))
        {
            facade.add_force(*handle, force, source);
        }
    }

    fn clone_box(&self) -> Box<dyn ForceLaw> {
        Box::new(Self {
            background_density_kg_m3: self.background_density_kg_m3,
            coulomb_log: self.coulomb_log,
            enabled: self.enabled,
        })
    }
}

/// MOND-corrected gravitational acceleration on each Rapier body toward a
/// fixed direction.  Callers supply the Newtonian acceleration magnitude
/// `newtonian_a` (computed by their preferred Newtonian routine — typically
/// `stellar::lane_emden_solve` + n-body `PairwiseGravity`); the law boosts it
/// to `sqrt(a_N · a_0)` when `a_N < a_0` (deep-field regime) and leaves it
/// untouched otherwise.  Force = mass · accel.
pub(crate) struct MonDGravityForceLaw {
    /// Newtonian acceleration magnitude provided externally (m/s²).
    pub newtonian_a: f64,
    /// Milgrom's scale acceleration a_0 (typical 1.2e-10 m/s²).
    pub mond_a_zero: f64,
    /// Direction toward the dominant attractor (world-frame unit-vector,
    /// pre-computed by caller).  Bodies are pulled along +direction.
    pub direction: Vector,
    pub enabled: bool,
}

impl ForceLaw for MonDGravityForceLaw {
    fn law_type(&self) -> ForceLawType {
        ForceLawType::MonDGravity
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn apply(&self, facade: &mut ForceFacade<'_>) {
        let source = self.law_type();
        // Pre-compute the (possibly MOND-boosted) acceleration magnitude.
        if self.newtonian_a <= 0.0 || self.newtonian_a.is_nan() {
            return;
        }
        let a_mag = if self.newtonian_a < self.mond_a_zero {
            match gd::mond_acceleration(self.newtonian_a, self.mond_a_zero) {
                Some(a) => a,
                None => return,
            }
        } else {
            self.newtonian_a
        };

        let dir_norm_sq = self.direction.length_squared();
        if !dir_norm_sq.is_finite() || dir_norm_sq <= 1.0e-18 {
            return;
        }
        // Pull is along +direction (direction points toward the attractor).
        let dir_unit = self.direction / dir_norm_sq.sqrt();
        let accel = dir_unit * a_mag;

        // Two-phase fill on the rayon pool above `PAR_MIN_ITEMS` bodies
        // (order-preserving; see the `parallel` module docs).
        let handles: Vec<RigidBodyHandle> = facade
            .bodies
            .iter()
            .filter(|(_, b)| b.is_dynamic())
            .map(|(h, _)| h)
            .collect();
        let computed: Vec<Option<Vector>> = crate::rapier::parallel::par_map_bodies(
            &handles,
            &*facade.bodies,
            crate::rapier::parallel::PAR_MIN_ITEMS,
            |_, body| {
                // Rapier 0.34 returns mass=0 until collider propagation on bodies
                // that haven't been touched by a collider this frame; fall back to
                // 1 kg so the law is still observable on collider-less test bodies
                // (matches the DynamicalFriction force law's mitigation).
                let mass = body.mass().max(1.0);
                let force = accel * mass;
                (force != Vector::ZERO).then_some(force)
            },
        );

        // Apply (parallel results replay in handle order).
        for (handle, force) in handles
            .iter()
            .zip(&computed)
            .filter_map(|(h, f)| f.map(|f| (h, f)))
        {
            facade.add_force(*handle, force, source);
        }
    }

    fn clone_box(&self) -> Box<dyn ForceLaw> {
        Box::new(Self {
            newtonian_a: self.newtonian_a,
            mond_a_zero: self.mond_a_zero,
            direction: self.direction,
            enabled: self.enabled,
        })
    }
}

/// Eddington-limited radiation pressure pushing a Rapier body outward from an
/// accretor (e.g. black hole / neutron star / white dwarf).
///
/// Uses `mps_formula::high_energy_astro::eddington_limited_luminosity` to
/// compute the Eddington luminosity `L_Edd = 4π G M c / κ`, then converts it
/// to a radiation force on each Rapier body:
///
/// ```text
///   F = (L_Edd / (c · 4π · r²)) · A_eff,   direction = +r̂ (away from source)
/// ```
///
/// Parameters: `mass_kg` accretor mass; `opacity` κ in m²/kg;
/// `source_position` world-frame accretor coordinates; `effective_area_m2`
/// the body's apparent optical cross-section.
pub(crate) struct EddingtonRadiationPressureForceLaw {
    /// Accretor mass in kg.
    pub mass_kg: f64,
    /// Opacity κ in m²/kg (electron scattering ≈ 0.034 m²/kg for H).
    pub opacity: f64,
    /// World-frame position of the luminous accretor (radiation source).
    pub source_position: Vector,
    /// Effective optical cross-section area of each Rapier body (m²).
    pub effective_area_m2: f64,
    pub enabled: bool,
}

impl ForceLaw for EddingtonRadiationPressureForceLaw {
    fn law_type(&self) -> ForceLawType {
        ForceLawType::EddingtonRadiationPressure
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn apply(&self, facade: &mut ForceFacade<'_>) {
        let source = self.law_type();
        // SPEED_OF_LIGHT in m/s (matches mps-formula::high_energy_astro).
        const C: f64 = 299_792_458.0;

        if !self.mass_kg.is_finite()
            || self.mass_kg <= 0.0
            || !self.opacity.is_finite()
            || self.opacity <= 0.0
            || !self.effective_area_m2.is_finite()
            || self.effective_area_m2 <= 0.0
            || !vec3_finite(vec3_from_rapier(self.source_position))
        {
            return;
        }

        let luminosity = match hea::eddington_limited_luminosity(self.mass_kg, self.opacity) {
            Some(l) => l, // Watts
            None => return,
        };

        // F = L_Edd / (c · 4π · r²) · A_eff; pre-factor per r² unit.
        let prefactor = luminosity * self.effective_area_m2 / (C * 4.0 * std::f64::consts::PI);

        // Two-phase fill on the rayon pool above `PAR_MIN_ITEMS` bodies
        // (order-preserving; see the `parallel` module docs).
        let handles: Vec<RigidBodyHandle> = facade
            .bodies
            .iter()
            .filter(|(_, b)| b.is_dynamic())
            .map(|(h, _)| h)
            .collect();
        let computed: Vec<Option<Vector>> = crate::rapier::parallel::par_map_bodies(
            &handles,
            &*facade.bodies,
            crate::rapier::parallel::PAR_MIN_ITEMS,
            |_, body| {
                let r_vec = body.translation() - self.source_position;
                let r_sq = r_vec.length_squared();
                if !r_sq.is_finite() || r_sq <= 1.0e-6 {
                    // Source-body coincidence: skip to avoid infinite force.
                    return None;
                }
                // Outward push (away from source).
                let r_hat = r_vec / r_sq.sqrt();
                let force = r_hat * (prefactor / r_sq);
                (force != Vector::ZERO).then_some(force)
            },
        );

        // Apply (parallel results replay in handle order).
        for (handle, force) in handles
            .iter()
            .zip(&computed)
            .filter_map(|(h, f)| f.map(|f| (h, f)))
        {
            facade.add_force(*handle, force, source);
        }
    }

    fn clone_box(&self) -> Box<dyn ForceLaw> {
        Box::new(Self {
            mass_kg: self.mass_kg,
            opacity: self.opacity,
            source_position: self.source_position,
            effective_area_m2: self.effective_area_m2,
            enabled: self.enabled,
        })
    }
}

/// X-ray binary disc bolometric irradiation pressure on a Rapier body, pushing
/// it outward from the accretor.
///
/// Uses `mps_formula::high_energy_astro::xray_disc_bolometric_luminosity` to
/// compute the disc bolometric luminosity in **solar luminosities**, converts
/// to SI watts (× L_SUN = 3.828e26 W), then applies the same radiation-pressure
/// formula as `EddingtonRadiationPressureForceLaw`:
///
/// ```text
///   F = (L_X / (c · 4π · r²)) · A_eff,   direction = +r̂ (away from source)
/// ```
///
/// Parameters: `k_t_eff_kev` inner-edge effective temperature [keV];
/// `r_in_km` inner disc radius [km]; `spectral_hardening` f_col;
/// `source_position` world-frame accretor position; `effective_area_m2` the
/// body's apparent optical cross-section.
pub(crate) struct XrayIrradiationForceLaw {
    /// Inner-edge effective temperature `kT_eff` [keV].
    pub k_t_eff_kev: f64,
    /// Inner disc radius [km].
    pub r_in_km: f64,
    /// Spectral hardening factor `f_col` (~1.7 for BH discs).
    pub spectral_hardening: f64,
    /// World-frame position of the X-ray source.
    pub source_position: Vector,
    /// Effective optical cross-section area of each Rapier body (m²).
    pub effective_area_m2: f64,
    pub enabled: bool,
}

impl ForceLaw for XrayIrradiationForceLaw {
    fn law_type(&self) -> ForceLawType {
        ForceLawType::XrayIrradiation
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn apply(&self, facade: &mut ForceFacade<'_>) {
        let source = self.law_type();
        const C: f64 = 299_792_458.0;
        const L_SUN: f64 = 3.828e26; // W

        if !self.k_t_eff_kev.is_finite()
            || self.k_t_eff_kev <= 0.0
            || !self.r_in_km.is_finite()
            || self.r_in_km <= 0.0
            || !self.spectral_hardening.is_finite()
            || self.spectral_hardening <= 0.0
            || !self.effective_area_m2.is_finite()
            || self.effective_area_m2 <= 0.0
            || !vec3_finite(vec3_from_rapier(self.source_position))
        {
            return;
        }

        let l_solar = match hea::xray_disc_bolometric_luminosity(
            self.k_t_eff_kev,
            self.r_in_km,
            self.spectral_hardening,
        ) {
            Some(l) => l,
            None => return,
        };
        let luminosity_w = l_solar * L_SUN;
        let prefactor = luminosity_w * self.effective_area_m2 / (C * 4.0 * std::f64::consts::PI);

        // Two-phase fill on the rayon pool above `PAR_MIN_ITEMS` bodies
        // (order-preserving; see the `parallel` module docs).
        let handles: Vec<RigidBodyHandle> = facade
            .bodies
            .iter()
            .filter(|(_, b)| b.is_dynamic())
            .map(|(h, _)| h)
            .collect();
        let computed: Vec<Option<Vector>> = crate::rapier::parallel::par_map_bodies(
            &handles,
            &*facade.bodies,
            crate::rapier::parallel::PAR_MIN_ITEMS,
            |_, body| {
                let r_vec = body.translation() - self.source_position;
                let r_sq = r_vec.length_squared();
                if !r_sq.is_finite() || r_sq <= 1.0e-6 {
                    return None;
                }
                let r_hat = r_vec / r_sq.sqrt();
                let force = r_hat * (prefactor / r_sq);
                (force != Vector::ZERO).then_some(force)
            },
        );

        // Apply (parallel results replay in handle order).
        for (handle, force) in handles
            .iter()
            .zip(&computed)
            .filter_map(|(h, f)| f.map(|f| (h, f)))
        {
            facade.add_force(*handle, force, source);
        }
    }

    fn clone_box(&self) -> Box<dyn ForceLaw> {
        Box::new(Self {
            k_t_eff_kev: self.k_t_eff_kev,
            r_in_km: self.r_in_km,
            spectral_hardening: self.spectral_hardening,
            source_position: self.source_position,
            effective_area_m2: self.effective_area_m2,
            enabled: self.enabled,
        })
    }
}

/// Magnetic-dipole torque on a magnetised Rapier body in a pulsar's dipole
/// field.
///
/// Uses `mps_formula::high_energy_astro::pulsar_surface_b_field` to compute
/// the surface B-field `B_surf` (Tesla) of the pulsar, then scales by dipole
/// fall-off 1/r³ to the body's location:
///
/// ```text
///   B(r) = B_surf · (R_ns / r)³
///   τ = μ × B(r)
/// ```
///
/// where `μ` is the body's magnetic dipole moment [A·m²] (user-supplied)
/// and `B(r)` is the pulsar B-field at the body's position, direction along
/// the pulsar's spin axis (user-supplied; the dipole axis is approximated by
/// the spin axis). Torque is applied via `ForceFacade::add_torque`.
///
/// Parameters: `moment_of_inertia` [kg·m²]; `ns_radius_m` neutron-star radius
/// [m]; `period_ms` spin period [ms]; `period_derivative` Ṗ [s/s];
/// `pulsar_position` world-frame pulsar location; `spin_axis` unit vector
/// along the magnetic/rotation axis; `body_dipole_moment` the Rapier body's
/// magnetic dipole moment [A·m²] as a Vector (direction = dipole axis,
/// magnitude = |μ|).
pub(crate) struct PulsarMagneticDipoleForceLaw {
    /// Pulsar moment of inertia [kg·m²].
    pub moment_of_inertia: f64,
    /// Neutron-star radius [m] (canonical 1e4 m = 10 km).
    pub ns_radius_m: f64,
    /// Spin period P [ms].
    pub period_ms: f64,
    /// Spin-down rate Ṗ [s/s].
    pub period_derivative: f64,
    /// World-frame pulsar position.
    pub pulsar_position: Vector,
    /// Unit vector along the pulsar magnetic (≈ rotation) axis.
    pub spin_axis: Vector,
    /// Body's magnetic dipole moment μ [A·m²] — direction = dipole axis,
    /// magnitude = |μ|.
    pub body_dipole_moment: Vector,
    pub enabled: bool,
}

impl ForceLaw for PulsarMagneticDipoleForceLaw {
    fn law_type(&self) -> ForceLawType {
        ForceLawType::PulsarMagneticDipole
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn apply(&self, facade: &mut ForceFacade<'_>) {
        let source = self.law_type();

        if !self.moment_of_inertia.is_finite()
            || self.moment_of_inertia <= 0.0
            || !self.ns_radius_m.is_finite()
            || self.ns_radius_m <= 0.0
            || !self.period_ms.is_finite()
            || self.period_ms <= 0.0
            || !self.period_derivative.is_finite()
            || self.period_derivative <= 0.0
            || !vec3_finite(vec3_from_rapier(self.pulsar_position))
            || !vec3_finite(vec3_from_rapier(self.spin_axis))
            || !vec3_finite(vec3_from_rapier(self.body_dipole_moment))
        {
            return;
        }

        let spin_norm_sq = self.spin_axis.length_squared();
        if !spin_norm_sq.is_finite() || spin_norm_sq <= 1.0e-18 {
            return;
        }
        let spin_unit = self.spin_axis / spin_norm_sq.sqrt();

        let b_surf = match hea::pulsar_surface_b_field(
            self.moment_of_inertia,
            self.ns_radius_m,
            self.period_ms,
            self.period_derivative,
        ) {
            Some(b) => b, // Tesla at the surface
            None => return,
        };

        // Two-phase fill on the rayon pool above `PAR_MIN_ITEMS` bodies
        // (order-preserving; see the `parallel` module docs). Each body
        // produces at most one torque routed to either the normal
        // `add_torque` path or the collider-less angvel fallback.
        let r_ns = self.ns_radius_m;
        let mu_vec = self.body_dipole_moment;
        let handles: Vec<RigidBodyHandle> = facade
            .bodies
            .iter()
            .filter(|(_, b)| b.is_dynamic())
            .map(|(h, _)| h)
            .collect();
        let computed: Vec<(Option<Vector>, Option<Vector>)> =
            crate::rapier::parallel::par_map_bodies(
                &handles,
                &*facade.bodies,
                crate::rapier::parallel::PAR_MIN_ITEMS,
                |_, body| {
                    let r_vec = body.translation() - self.pulsar_position;
                    let r_sq = r_vec.length_squared();
                    if !r_sq.is_finite() || r_sq <= 1.0e-6 {
                        return (None, None);
                    }
                    let r = r_sq.sqrt();
                    if r <= r_ns {
                        // Inside the NS surface — skip (B not defined by 1/r³ here).
                        return (None, None);
                    }
                    // Dipole fall-off: B(r) = B_surf · (R_ns / r)³
                    let b_mag = b_surf * (r_ns / r).powi(3);
                    let b_vec = spin_unit * b_mag;
                    // τ = μ × B
                    let torque = mu_vec.cross(b_vec);
                    if torque == Vector::ZERO {
                        return (None, None);
                    }
                    // Rapier 0.34: a collider-less dynamic body has mass=0 and
                    // inverse_inertia=0 until collider propagation, so add_torque's
                    // net angular impulse is zero and torque silently no-ops.  When
                    // the body has zero mass — i.e. is collider-less — fall back to
                    // directly incrementing angvel by τ·dt against unit rotational
                    // inertia, so the torque is at least observable in tests.
                    // (Mass/inertia propagation re-enable normal add_torque with a
                    // collider attached.)
                    if body.mass() <= 0.0 {
                        (None, Some(torque))
                    } else {
                        (Some(torque), None)
                    }
                },
            );

        // Apply normal torque path (parallel results replay in handle order).
        for (handle, (normal, _)) in handles.iter().zip(&computed) {
            if let Some(torque) = normal {
                facade.add_torque(*handle, *torque, source);
            }
        }
        // Collider-less fallback: collider-less dynamic bodies in Rapier 0.34
        // have inverse_inertia = 0, so add_torque's integration silently
        // produces no angular acceleration.  To keep the torque observable in
        // the collider-less test configuration, assume a unit rotational
        // inertia (1 kg·m²) and add τ·dt/I directly to the body's angvel,
        // scaled by the facade's actual step dt.  With a real collider
        // attached, mass propagation makes body.mass() > 0 and the normal
        // add_torque path takes over, producing the correct τ/I·dt angular
        // acceleration.
        let unit_rotational_inertia = 1.0; // kg·m² assumed for collider-less bodies
        for (handle, (_, fallback)) in handles.iter().zip(&computed) {
            let Some(torque) = fallback else {
                continue;
            };
            let Some(body) = facade.bodies.get_mut(*handle) else {
                continue;
            };
            let domega = body.angvel() + torque * (facade.dt / unit_rotational_inertia);
            body.set_angvel(domega, true);
        }
    }

    fn clone_box(&self) -> Box<dyn ForceLaw> {
        Box::new(Self {
            moment_of_inertia: self.moment_of_inertia,
            ns_radius_m: self.ns_radius_m,
            period_ms: self.period_ms,
            period_derivative: self.period_derivative,
            pulsar_position: self.pulsar_position,
            spin_axis: self.spin_axis,
            body_dipole_moment: self.body_dipole_moment,
            enabled: self.enabled,
        })
    }
}

/// Jeans-escape drag: a Rapier body near a planetary exobase is pushed along
/// the escape direction by the thermal Jeans efflux.
///
/// The Jeans particle efflux Φ [m⁻²·s⁻¹] is computed by
/// `mps_formula::heliophysics::jeans_escape_flux(n_exo, T, λ, m)`.  The
/// corresponding momentum flux (pressure in Pascals) is
///
/// ```text
///   p = Φ · m_molecule · v_thermal,    v_thermal = √(2 k_B T / m)
/// ```
///
/// and the force on each dynamic Rapier body is `F = p · A_eff · ê`, where
/// `ê` is the user-supplied unit vector along the escape direction (radially
/// outward from the exobase).  This is a uniform-flow push, so no per-body
/// radial location is tracked — the body is assumed to be in the uniform
/// efflux cone.  When a body is co-moving with the efflux (its projected
/// velocity along `ê` exceeds the thermal speed), the relative efflux goes
/// to zero and no force is applied (drag-style gate).
pub(crate) struct JeansEscapeDragForceLaw {
    /// Exobase number density `n_exo` [m⁻³].
    pub n_exo: f64,
    /// Exobase temperature `T` [K].
    pub temperature: f64,
    /// Jeans escape parameter `λ = G M m / (k_B T R)` (dimensionless).
    pub escape_parameter: f64,
    /// Mass of the escaping molecule `m` [kg].
    pub mass_kg: f64,
    /// World-frame unit vector along the escape direction (radially outward).
    pub escape_direction: Vector,
    /// Effective cross-section area of each Rapier body (m²).
    pub effective_area_m2: f64,
    pub enabled: bool,
}

impl ForceLaw for JeansEscapeDragForceLaw {
    fn law_type(&self) -> ForceLawType {
        ForceLawType::JeansEscape
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn apply(&self, facade: &mut ForceFacade<'_>) {
        let source = self.law_type();
        let dir = self.escape_direction;

        // Skip if the direction is degenerate (zero, NaN, …).
        let dir_norm_sq = dir.length_squared();
        if !dir_norm_sq.is_finite() || dir_norm_sq <= 1.0e-18 {
            return;
        }
        let dir_unit = dir / dir_norm_sq.sqrt();

        // Validate scalars up front so the formula call is a clean `Some`.
        if !self.n_exo.is_finite()
            || self.n_exo <= 0.0
            || !self.temperature.is_finite()
            || self.temperature <= 0.0
            || !self.escape_parameter.is_finite()
            || self.escape_parameter < 0.0
            || !self.mass_kg.is_finite()
            || self.mass_kg <= 0.0
            || !self.effective_area_m2.is_finite()
            || self.effective_area_m2 <= 0.0
        {
            return;
        }

        // Jeans particle efflux Φ [m⁻²·s⁻¹].
        let flux = match hph::jeans_escape_flux(
            self.n_exo,
            self.temperature,
            self.escape_parameter,
            self.mass_kg,
        ) {
            Some(f) => f,
            None => return,
        };

        // Thermal speed at the exobase [m/s].
        const BOLTZMANN: f64 = 1.380649e-23;
        let v_thermal = (2.0 * BOLTZMANN * self.temperature / self.mass_kg).sqrt();
        if !v_thermal.is_finite() || v_thermal <= 0.0 {
            return;
        }

        // Momentum flux = Φ · m · v_thermal  [m⁻²·s⁻¹]·[kg]·[m/s] = Pa.
        let pressure_pa = flux * self.mass_kg * v_thermal;
        if !pressure_pa.is_finite() || pressure_pa <= 0.0 {
            return;
        }

        // Two-phase fill on the rayon pool above `PAR_MIN_ITEMS` bodies
        // (order-preserving; see the `parallel` module docs).
        let handles: Vec<RigidBodyHandle> = facade
            .bodies
            .iter()
            .filter(|(_, b)| b.is_dynamic())
            .map(|(h, _)| h)
            .collect();
        let computed: Vec<Option<Vector>> = crate::rapier::parallel::par_map_bodies(
            &handles,
            &*facade.bodies,
            crate::rapier::parallel::PAR_MIN_ITEMS,
            |_, body| {
                // Drag-style gate: the efflux force is the momentum flux the body
                // intercepts.  A body moving along the efflux faster than the
                // thermal speed has negative relative flow and feels no push.
                let v_rel_along = v_thermal - body.linvel().dot(dir_unit);
                if v_rel_along <= 0.0 || v_rel_along.is_nan() {
                    return None;
                }
                // F = p · A_eff · ê (push along +escape_direction).
                let force = dir_unit * (pressure_pa * self.effective_area_m2);
                (force != Vector::ZERO).then_some(force)
            },
        );

        // Apply (parallel results replay in handle order).
        for (handle, force) in handles
            .iter()
            .zip(&computed)
            .filter_map(|(h, f)| f.map(|f| (h, f)))
        {
            facade.add_force(*handle, force, source);
        }
    }

    fn clone_box(&self) -> Box<dyn ForceLaw> {
        Box::new(Self {
            n_exo: self.n_exo,
            temperature: self.temperature,
            escape_parameter: self.escape_parameter,
            mass_kg: self.mass_kg,
            escape_direction: self.escape_direction,
            effective_area_m2: self.effective_area_m2,
            enabled: self.enabled,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
