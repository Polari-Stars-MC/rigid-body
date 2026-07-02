//! Superfluidity and quantum vortex dynamics:
//! - Vortex filament model (Biot–Savart law for velocity induction)
//! - Quantised circulation (h/m quanta)
//! - Gross–Pitaevskii equation (simplified: order parameter evolution, energy densities)
//! - Vortex ring dynamics (self-induced motion)
//! - Vortex reconnection events (segment topology changes)
//! - Vortex tangle statistics (line density, kinetic energy)
//!
//! All functions are FFI-exported with C-compatible types.

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    BiotSavartVelocity, Bool, GpEnergyDensity, GpGridPoint, GpOrderParameter,
    GpTimeEvolutionParams, QuantisedCirculation, Vec3, VortexReconnectionReport, VortexRing,
    VortexSegment, VortexTangleStats, finite_non_negative, finite_positive, vec3_finite,
};
use crate::rapier::math::KahanSum;

const EPSILON: f64 = 1.0e-14;
const FOUR_PI: f64 = 12.566_370_614_359_172;
/// Planck constant (kg·m²/s)
const PLANCK: f64 = 6.626_070_15e-34;
/// Reduced Planck constant ħ = h / 2π
const HBAR: f64 = 1.054_571_817e-34;
/// ⁴He mass (kg)
const HELIUM_MASS: f64 = 6.646_476_4e-27;
/// ⁴He scattering length a (m)
const HELIUM_SCATTERING_LENGTH: f64 = 2.2e-10;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn write_out<T: Copy>(out: *mut T, value: T) -> Bool {
    let Some(out) = (unsafe { out.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return Bool::FALSE;
    };
    *out = value;
    clear_error();
    Bool::TRUE
}

fn vec3_length_sq(v: Vec3) -> f64 {
    v.x * v.x + v.y * v.y + v.z * v.z
}

fn vec3_length(v: Vec3) -> f64 {
    vec3_length_sq(v).sqrt()
}

fn vec3_sub(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.x - b.x,
        y: a.y - b.y,
        z: a.z - b.z,
    }
}

fn vec3_cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    }
}

fn vec3_dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn vec3_normalize(v: Vec3) -> Vec3 {
    let len = vec3_length(v);
    if len > EPSILON {
        Vec3 {
            x: v.x / len,
            y: v.y / len,
            z: v.z / len,
        }
    } else {
        Vec3::default()
    }
}

fn vec3_scale(v: Vec3, s: f64) -> Vec3 {
    Vec3 {
        x: v.x * s,
        y: v.y * s,
        z: v.z * s,
    }
}

fn vec3_add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.x + b.x,
        y: a.y + b.y,
        z: a.z + b.z,
    }
}

fn segment_valid(seg: &VortexSegment) -> bool {
    vec3_finite(seg.start) && vec3_finite(seg.end) && finite_positive(seg.core_radius)
}

fn ring_valid(ring: &VortexRing) -> bool {
    vec3_finite(ring.center) && finite_positive(ring.radius) && vec3_finite(ring.axis)
}

// ===========================================================================
// A. Biot–Savart law for vortex segments
// ===========================================================================

/// Compute the velocity induced by a straight vortex segment at a field point
/// using the Biot–Savart law.
///
/// v = (κ / 4π) * ∫ (dℓ × r̂) / |r|²
///
/// For a straight segment from s₁ to s₂, the induced velocity at point p is
/// evaluated using the analytical formula involving the solid angle.
#[unsafe(no_mangle)]
pub extern "C" fn sf_biot_savart_velocity(
    segment: VortexSegment,
    field_point: Vec3,
    out_velocity: *mut BiotSavartVelocity,
) -> Bool {
    if !segment_valid(&segment) || !vec3_finite(field_point) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "segment and field_point must be finite",
        );
        return Bool::FALSE;
    }

    let s1 = segment.start;
    let s2 = segment.end;
    let kappa = (segment.circulation_quantum as f64) * circulation_quantum_const();

    // Vectors from field point to endpoints
    let r1 = vec3_sub(s1, field_point);
    let r2 = vec3_sub(s2, field_point);
    let seg_vec = vec3_sub(s2, s1);

    let r1_len = vec3_length(r1);
    let r2_len = vec3_length(r2);
    let seg_len = vec3_length(seg_vec);

    if r1_len < EPSILON || r2_len < EPSILON {
        // Field point coincides with an endpoint → near-singular
        set_error(ERR_INVALID_ARGUMENT, "field point coincides with segment endpoint");
        return Bool::FALSE;
    }
    if seg_len < EPSILON {
        return write_out(
            out_velocity,
            BiotSavartVelocity {
                velocity: Vec3::default(),
                magnitude: 0.0,
                distance: (r1_len + r2_len) * 0.5,
            },
        );
    }

    let cross_prod = vec3_cross(r1, r2);
    let cross_len_sq = vec3_length_sq(cross_prod);

    if cross_len_sq < EPSILON {
        // Field point is collinear with the segment → zero induced velocity
        let dist = vec3_length(vec3_cross(seg_vec, r1)) / seg_len;
        // Also check if point lies between endpoints (on the segment body)
        let dot1 = vec3_dot(r1, seg_vec);
        let dot2 = vec3_dot(r2, seg_vec);
        let on_segment = (dot1 >= 0.0) != (dot2 >= 0.0);
        if on_segment {
            set_error(ERR_INVALID_ARGUMENT, "field point lies on the vortex segment");
            return Bool::FALSE;
        }
        return write_out(
            out_velocity,
            BiotSavartVelocity {
                velocity: Vec3::default(),
                magnitude: 0.0,
                distance: dist,
            },
        );
    }

    let cross_len = cross_len_sq.sqrt();

    // Prefactor: (s₁-p)/|s₁-p| + (s₂-p)/|s₂-p|
    let r1_hat = vec3_scale(r1, 1.0 / r1_len);
    let r2_hat = vec3_scale(r2, 1.0 / r2_len);
    let sum_hats = vec3_add(r1_hat, r2_hat);

    // Dot with segment direction
    let seg_dir = vec3_scale(seg_vec, 1.0 / seg_len);
    let dot = vec3_dot(sum_hats, seg_dir);

    // Induced velocity magnitude and direction
    let prefactor = kappa / FOUR_PI;
    let vel_mag = prefactor * dot / cross_len;
    let vel_dir = vec3_normalize(cross_prod);

    // For a straight segment, the velocity is in the direction of r1×r2
    let distance = vec3_length(vec3_cross(seg_vec, r1)) / seg_len;

    write_out(
        out_velocity,
        BiotSavartVelocity {
            velocity: vec3_scale(vel_dir, vel_mag),
            magnitude: vel_mag.abs(),
            distance,
        },
    )
}

/// Compute the self-induced velocity of a vortex ring.
///
/// For a circular vortex ring of radius R, the self-induced velocity is:
///
///   v_ring = (κ / 4πR) * [ln(8R/ξ) - 1/2]
///
/// where κ = h/m is the circulation quantum and ξ is the healing length.
#[unsafe(no_mangle)]
pub extern "C" fn sf_vortex_ring_velocity(
    ring: VortexRing,
    out_velocity: *mut Vec3,
) -> Bool {
    if !ring_valid(&ring) {
        set_error(ERR_INVALID_ARGUMENT, "ring parameters must be finite and positive");
        return Bool::FALSE;
    }

    let kappa = (ring.circulation_quantum as f64) * circulation_quantum_const();
    let r = ring.radius;
    let xi = 1.0e-10; // default healing length ~ 0.1 nm for ⁴He

    if r <= xi {
        set_error(ERR_INVALID_ARGUMENT, "ring radius must exceed healing length");
        return Bool::FALSE;
    }

    let speed = (kappa / (FOUR_PI * r)) * ((8.0 * r / xi).ln() - 0.5);
    let axis_dir = vec3_normalize(ring.axis);

    write_out(out_velocity, vec3_scale(axis_dir, speed))
}

/// Return the circulation quantum constant κ₀ = h/m for ⁴He.
#[unsafe(no_mangle)]
pub extern "C" fn sf_circulation_quantum() -> f64 {
    PLANCK / HELIUM_MASS
}

fn circulation_quantum_const() -> f64 {
    PLANCK / HELIUM_MASS
}

// ===========================================================================
// B. Quantised circulation
// ===========================================================================

/// Compute the circulation around a closed loop by summing Biot–Savart
/// contributions along a set of segments forming the loop.
///
/// `segments` — pointer to an array of `VortexSegment`.
/// `segment_count` — number of segments.
/// `sample_point` — a point on the loop where the velocity is integrated.
#[unsafe(no_mangle)]
pub extern "C" fn sf_circulation_around_loop(
    segments: *const VortexSegment,
    segment_count: u32,
    sample_point: Vec3,
    out_circulation: *mut QuantisedCirculation,
) -> Bool {
    if segments.is_null() {
        set_error(ERR_NULL_POINTER, "segments pointer is null");
        return Bool::FALSE;
    }
    if segment_count == 0 {
        set_error(ERR_INVALID_ARGUMENT, "segment_count must be > 0");
        return Bool::FALSE;
    }
    if !vec3_finite(sample_point) {
        set_error(ERR_INVALID_ARGUMENT, "sample_point must be finite");
        return Bool::FALSE;
    }

    let segs = unsafe { std::slice::from_raw_parts(segments, segment_count as usize) };
    let kappa_0 = circulation_quantum_const();

    // Compute circulation by summing the tangential component of velocity
    // around the loop, approximated as sum over segments of v_segment · dℓ
    let mut circulation_acc = KahanSum::default();

    for seg in segs {
        if !segment_valid(seg) {
            continue;
        }
        let mut biot = BiotSavartVelocity::default();
        let result = sf_biot_savart_velocity(*seg, sample_point, &mut biot);
        if result == Bool::FALSE {
            continue;
        }
        // dL along segment direction
        let seg_len = vec3_sub(seg.end, seg.start);
        circulation_acc.add(vec3_dot(biot.velocity, seg_len));
    }

    let circulation = circulation_acc.value();

    let kappa_0 = if kappa_0.abs() > EPSILON {
        kappa_0
    } else {
        circulation_quantum_const()
    };
    let n = (circulation / kappa_0).round() as i32;
    let quantised = (circulation / kappa_0 - n as f64).abs() < 0.5;

    write_out(
        out_circulation,
        QuantisedCirculation {
            circulation,
            quantum_number: n,
            circulation_quantum: kappa_0,
            quantised: Bool::from(quantised),
        },
    )
}

/// Estimate the quantum number n = ∮v·dℓ / (h/m) given a velocity field
/// around a loop approximated by N tangent velocity samples.
///
/// `tangent_velocities` — pointer to an array of tangential velocity
/// components (m/s) equally spaced around the loop.
/// `loop_radius` — radius of the circular loop (m).
/// `sample_count` — number of samples.
#[unsafe(no_mangle)]
pub extern "C" fn sf_quantum_number_estimate(
    tangent_velocities: *const f64,
    loop_radius: f64,
    sample_count: u32,
    out_quantum: *mut i32,
) -> Bool {
    if tangent_velocities.is_null() {
        set_error(ERR_NULL_POINTER, "tangent_velocities pointer is null");
        return Bool::FALSE;
    }
    if !finite_positive(loop_radius) {
        set_error(ERR_INVALID_ARGUMENT, "loop_radius must be positive and finite");
        return Bool::FALSE;
    }
    if sample_count < 3 {
        set_error(ERR_INVALID_ARGUMENT, "sample_count must be >= 3");
        return Bool::FALSE;
    }

    let kappa_0 = circulation_quantum_const();
    let samples = unsafe { std::slice::from_raw_parts(tangent_velocities, sample_count as usize) };

    // Circulation: sum v_t · ds around loop, using midpoint rule
    let ds = 2.0 * std::f64::consts::PI * loop_radius / (sample_count as f64);
    let circulation: f64 = samples.iter().map(|&v| v * ds).sum();

    let n = (circulation / kappa_0).round() as i32;
    write_out(out_quantum, n)
}

// ===========================================================================
// C. Gross–Pitaevskii equation (simplified)
// ===========================================================================

/// Evaluate the Gross–Pitaevskii order parameter ψ = √n · exp(iφ) at a point,
/// returning amplitude, phase, and density.
///
/// For a generic vortex line passing through `vortex_center` with direction
/// `vortex_axis`, the phase wraps by 2π around the line.
#[unsafe(no_mangle)]
pub extern "C" fn sf_gp_order_parameter(
    x: f64,
    y: f64,
    z: f64,
    vortex_center: Vec3,
    vortex_axis: Vec3,
    circulation_quantum: i32,
    healing_length: f64,
    background_density: f64,
    out_param: *mut GpOrderParameter,
) -> Bool {
    if !vec3_finite(vortex_center) || !vec3_finite(vortex_axis) {
        set_error(ERR_INVALID_ARGUMENT, "vortex_center and vortex_axis must be finite");
        return Bool::FALSE;
    }
    if !finite_positive(healing_length) {
        set_error(ERR_INVALID_ARGUMENT, "healing_length must be positive and finite");
        return Bool::FALSE;
    }
    if !finite_positive(background_density) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "background_density must be positive and finite",
        );
        return Bool::FALSE;
    }

    let point = Vec3 { x, y, z };
    let rel = vec3_sub(point, vortex_center);
    let axis = vec3_normalize(vortex_axis);

    // Distance from the point to the vortex axis
    let cross = vec3_cross(axis, rel);
    let dist = vec3_length(cross);

    // Amplitude: density depletion near core
    // n(r) = n₀ * r² / (r² + 2ξ²)  (Pade approximation to GP vortex profile)
    let xi = healing_length;
    let amplitude = if dist < EPSILON {
        0.0
    } else {
        let r2 = dist * dist;
        (background_density * r2 / (r2 + 2.0 * xi * xi)).sqrt()
    };

    // Phase: φ = n * arctan2(y', x') where (x', y') are in the plane orthogonal to axis
    // Build a local 2D coordinate system in the plane perpendicular to the axis
    let ref_dir = if axis.x.abs() < 0.9 {
        vec3_normalize(vec3_cross(axis, Vec3 { x: 1.0, y: 0.0, z: 0.0 }))
    } else {
        vec3_normalize(vec3_cross(axis, Vec3 { x: 0.0, y: 1.0, z: 0.0 }))
    };
    let ref_dir2 = vec3_cross(axis, ref_dir);

    let xp = vec3_dot(rel, ref_dir);
    let yp = vec3_dot(rel, ref_dir2);
    let phase = (circulation_quantum as f64) * yp.atan2(xp);

    write_out(
        out_param,
        GpOrderParameter {
            amplitude,
            phase,
            density: amplitude * amplitude,
        },
    )
}

/// Compute the Gross–Pitaevskii energy density (per unit volume) terms:
///
///   ε_kin  = (ħ² / 2m) |∇ψ|²
///   ε_int  = (g / 2) |ψ|⁴
///   ε_trap = V_trap |ψ|²
///
/// Simplified: uses a Thomas–Fermi approximation with a harmonic trapping
/// potential V_trap = ½ m ω² r².
#[unsafe(no_mangle)]
pub extern "C" fn sf_gp_energy_density(
    density: f64,
    trapping_frequency: f64,
    mass: f64,
    coupling_constant: f64,
    radius_from_center: f64,
    out_energy: *mut GpEnergyDensity,
) -> Bool {
    if !finite_non_negative(density) {
        set_error(ERR_INVALID_ARGUMENT, "density must be finite and non-negative");
        return Bool::FALSE;
    }
    if !finite_non_negative(trapping_frequency) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "trapping_frequency must be finite and non-negative",
        );
        return Bool::FALSE;
    }
    if !finite_positive(mass) {
        set_error(ERR_INVALID_ARGUMENT, "mass must be positive and finite");
        return Bool::FALSE;
    }
    if !finite_positive(coupling_constant) {
        set_error(ERR_INVALID_ARGUMENT, "coupling_constant must be positive and finite");
        return Bool::FALSE;
    }
    if !finite_non_negative(radius_from_center) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "radius_from_center must be finite and non-negative",
        );
        return Bool::FALSE;
    }

    let hbar = HBAR;

    // Kinetic energy density (gradient term): approximate as ħ²n/(2mξ²)
    // using the healing length relation ξ² = ħ²/(2mgn)
    let kinetic_density = if density > EPSILON && coupling_constant > EPSILON {
        let g_n = coupling_constant * density;
        // ∇ψ ~ √n / ξ where ξ = ħ/√(2mgn)
        let xi = hbar / (2.0 * mass * g_n).sqrt();
        (hbar * hbar / (2.0 * mass)) * density / (xi * xi)
    } else {
        0.0
    };

    // Interaction energy density: (g/2) n²
    let interaction_density = 0.5 * coupling_constant * density * density;

    // Trapping potential energy density: V(r) n, V(r) = ½ m ω² r²
    let trapping_density = if trapping_frequency > EPSILON {
        0.5 * mass * trapping_frequency * trapping_frequency * radius_from_center * radius_from_center
            * density
    } else {
        0.0
    };

    let total_density = kinetic_density + interaction_density + trapping_density;

    // Chemical potential: μ = gn + V_trap (local density approximation)
    let chemical_potential = coupling_constant * density
        + 0.5 * mass * trapping_frequency * trapping_frequency * radius_from_center
            * radius_from_center;

    write_out(
        out_energy,
        GpEnergyDensity {
            kinetic_density,
            interaction_density,
            trapping_density,
            total_density,
            chemical_potential,
        },
    )
}

/// Time-evolve the Gross–Pitaevskii order parameter at a single spatial point
/// using imaginary-time propagation (simple relaxation to ground state).
///
/// The homogeneous GP equation in imaginary time τ = i·t gives:
///   ∂ψ/∂τ = -(1/ħ) · (g|ψ|² - μ) · ψ
///
/// For the real amplitude a = |ψ|:
///   ∂a/∂τ = -(1/ħ) · (g a² - μ) · a
///
/// This converges to the equilibrium a = √(μ/g).
#[unsafe(no_mangle)]
pub extern "C" fn sf_gp_amplitude_evolution(
    amplitude: f64,
    density: f64,
    params: GpTimeEvolutionParams,
    out_next_amplitude: *mut f64,
) -> Bool {
    if !finite_non_negative(amplitude) {
        set_error(ERR_INVALID_ARGUMENT, "amplitude must be finite and non-negative");
        return Bool::FALSE;
    }
    if !finite_non_negative(density) {
        set_error(ERR_INVALID_ARGUMENT, "density must be finite and non-negative");
        return Bool::FALSE;
    }
    if !finite_positive(params.healing_length)
        || !finite_positive(params.sound_speed)
        || !finite_positive(params.chemical_potential)
        || !finite_positive(params.coupling_constant)
        || !finite_positive(params.dt)
    {
        set_error(ERR_INVALID_ARGUMENT, "all GP params must be positive");
        return Bool::FALSE;
    }

    let a = amplitude;
    let g = params.coupling_constant;
    let mu = params.chemical_potential;
    let hbar = HBAR;
    // n0 = μ / g
    let n0 = mu / g;

    // Imaginary time evolution: ∂a/∂τ = -(1/ħ) · (g a² - μ) · a
    let da_dt = -(1.0 / hbar) * (g * a * a - mu) * a;

    // Use a small enough step so amplitude doesn't overshoot
    let dt_eff = params.dt.min(hbar / mu);
    let next_amplitude = a + dt_eff * da_dt;

    // Clamp to [0, √n₀] — the ground state is the fixed point
    let next_amplitude = next_amplitude.max(0.0).min(n0.sqrt() * 2.0);

    write_out(out_next_amplitude, next_amplitude)
}

// ===========================================================================
// D. Vortex reconnection
// ===========================================================================

/// Detect and perform a vortex reconnection between two line segments if
/// they are closer than `reconnection_distance`.
///
/// Reconnection model:
///   1. Find the closest points between the two segments.
///   2. If the minimum distance < reconnection_distance, reconnect by
///      swapping endpoints: s1_start ↔ s2_start and s1_end ↔ s2_end.
///   3. Return the new segments and energy dissipation estimate.
#[unsafe(no_mangle)]
pub extern "C" fn sf_vortex_reconnection(
    seg1: VortexSegment,
    seg2: VortexSegment,
    reconnection_distance: f64,
    healing_length: f64,
    out_report: *mut VortexReconnectionReport,
) -> Bool {
    if !segment_valid(&seg1) || !segment_valid(&seg2) {
        set_error(ERR_INVALID_ARGUMENT, "segments must be valid");
        return Bool::FALSE;
    }
    if !finite_positive(reconnection_distance) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "reconnection_distance must be positive and finite",
        );
        return Bool::FALSE;
    }
    if !finite_positive(healing_length) {
        set_error(ERR_INVALID_ARGUMENT, "healing_length must be positive and finite");
        return Bool::FALSE;
    }

    // Find closest points between two line segments
    let d1 = vec3_sub(seg1.end, seg1.start);
    let d2 = vec3_sub(seg2.end, seg2.start);
    let r = vec3_sub(seg1.start, seg2.start);

    let d1_len_sq = vec3_length_sq(d1);
    let d2_len_sq = vec3_length_sq(d2);

    let closest_approach = if d1_len_sq < EPSILON || d2_len_sq < EPSILON {
        // Degenerate segments
        let c1 = seg1.start;
        let c2 = seg2.start;
        vec3_length(vec3_sub(c1, c2))
    } else {
        let a = vec3_dot(d1, d1);
        let b = vec3_dot(d1, d2);
        let c = vec3_dot(d2, d2);
        let d = vec3_dot(d1, r);
        let e = vec3_dot(d2, r);

        let det = a * c - b * b;
        let (s, t) = if det.abs() > EPSILON {
            let s = (b * e - c * d) / det;
            let t = (a * e - b * d) / det;
            let s = s.clamp(0.0, 1.0);
            let t = t.clamp(0.0, 1.0);
            (s, t)
        } else {
            (0.0, 0.0)
        };

        let p1 = vec3_add(seg1.start, vec3_scale(d1, s));
        let p2 = vec3_add(seg2.start, vec3_scale(d2, t));
        vec3_length(vec3_sub(p1, p2))
    };

    let reconnected = closest_approach < reconnection_distance;

    // Energy dissipation: estimate as E_diss = κ² / (4πξ) * (reconnection_distance - d_min)
    let kappa = circulation_quantum_const();
    let dissipated = if reconnected {
        let xi = healing_length;
        let delta = reconnection_distance - closest_approach;
        let energy = (kappa * kappa / (FOUR_PI * xi)) * delta * 0.5;
        energy.max(0.0)
    } else {
        0.0
    };

    // Output segments: after reconnection, the topology changes
    // Simple X-reconnection model: swap endpoints
    if reconnected {
        write_out(
            out_report,
            VortexReconnectionReport {
                closest_approach,
                reconnected: Bool::TRUE,
                // New seg1: start of seg1 → start of seg2
                seg1_start: seg1.start,
                seg1_end: seg2.start,
                // New seg2: end of seg1 → end of seg2
                seg2_start: seg1.end,
                seg2_end: seg2.end,
                energy_dissipated: dissipated,
            },
        )
    } else {
        // No reconnection — return original segments
        write_out(
            out_report,
            VortexReconnectionReport {
                closest_approach,
                reconnected: Bool::FALSE,
                seg1_start: seg1.start,
                seg1_end: seg1.end,
                seg2_start: seg2.start,
                seg2_end: seg2.end,
                energy_dissipated: 0.0,
            },
        )
    }
}

// ===========================================================================
// E. Vortex tangle statistics
// ===========================================================================

/// Compute statistics for a vortex filament tangle (array of segments).
///
/// `segments` — pointer to array of `VortexSegment`.
/// `segment_count` — number of segments.
/// `box_volume` — volume of the bounding box containing the tangle (for line density).
#[unsafe(no_mangle)]
pub extern "C" fn sf_vortex_tangle_stats(
    segments: *const VortexSegment,
    segment_count: u32,
    box_volume: f64,
    out_stats: *mut VortexTangleStats,
) -> Bool {
    if segments.is_null() || segment_count == 0 {
        return write_out(
            out_stats,
            VortexTangleStats::default(),
        );
    }
    if !finite_non_negative(box_volume) {
        set_error(ERR_INVALID_ARGUMENT, "box_volume must be finite and non-negative");
        return Bool::FALSE;
    }

    let segs = unsafe { std::slice::from_raw_parts(segments, segment_count as usize) };

    let mut total_length_acc = KahanSum::default();
    let mut total_curvature_acc = KahanSum::default();
    let mut curvature_count = 0u32;

    // To compute curvature, we need triplets of consecutive segments.
    // We assume the segments are connected in order (polynomial chain).
    for (i, seg) in segs.iter().enumerate() {
        if !segment_valid(seg) {
            continue;
        }
        let seg_vec = vec3_sub(seg.end, seg.start);
        total_length_acc.add(vec3_length(seg_vec));

        // Approximate curvature from angle between consecutive segments
        if i > 0 {
            let prev = &segs[i - 1];
            if segment_valid(prev) {
                let prev_vec = vec3_sub(prev.end, prev.start);
                let pv_len = vec3_length(prev_vec);
                let sv_len = vec3_length(seg_vec);
                if pv_len > EPSILON && sv_len > EPSILON {
                    let cos_theta = vec3_dot(
                        vec3_scale(prev_vec, 1.0 / pv_len),
                        vec3_scale(seg_vec, 1.0 / sv_len),
                    )
                    .clamp(-1.0, 1.0);
                    let angle = cos_theta.acos();
                    total_curvature_acc.add(angle / ((pv_len + sv_len) * 0.5));
                    curvature_count += 1;
                }
            }
        }
    }

    let kappa = circulation_quantum_const();
    // Kinetic energy per unit length: E/L ≈ (κ²/4π) * ln(R/ξ)
    // Use a simplified form per segment
    let mut total_ke_acc = KahanSum::default();
    for seg in segs {
        if segment_valid(seg) {
            let seg_len = vec3_length(vec3_sub(seg.end, seg.start));
            let xi = seg.core_radius;
            let r_cutoff = seg_len.max(xi * 10.0);
            // Energy per unit length: (κ²/4π) * ln(R/ξ)
            let energy_per_length = if r_cutoff > xi {
                (kappa * kappa / FOUR_PI) * (r_cutoff / xi).ln()
            } else {
                0.0
            };
            total_ke_acc.add(energy_per_length * seg_len);
        }
    }

    let total_length = total_length_acc.value();
    let total_curvature = total_curvature_acc.value();
    let total_ke = total_ke_acc.value();

    let avg_curvature = if curvature_count > 0 {
        total_curvature / curvature_count as f64
    } else {
        0.0
    };

    let line_density = if box_volume > EPSILON {
        total_length / box_volume
    } else {
        0.0
    };

    write_out(
        out_stats,
        VortexTangleStats {
            segment_count,
            total_length,
            average_curvature: avg_curvature,
            total_kinetic_energy: total_ke,
            vortex_line_density: line_density,
        },
    )
}

// ===========================================================================
// F. Grid sampling of GP wavefunction (2D cross-section)
// ===========================================================================

/// Sample the GP order parameter on a 2D grid cross-section (for visualisation).
///
/// The grid lies in the plane perpendicular to `plane_axis`, centered at
/// `plane_center`, with `nx` × `ny` points covering extents `extent_x` × `extent_y`.
///
/// `out_grid` — pre-allocated buffer of `GpGridPoint` of length `nx * ny`.
#[unsafe(no_mangle)]
pub extern "C" fn sf_gp_grid_sample(
    plane_center: Vec3,
    plane_axis: Vec3,
    nx: u32,
    ny: u32,
    extent_x: f64,
    extent_y: f64,
    vortex_center: Vec3,
    vortex_axis: Vec3,
    circulation_quantum: i32,
    healing_length: f64,
    background_density: f64,
    out_grid: *mut GpGridPoint,
    out_len: u32,
) -> u32 {
    if !vec3_finite(plane_center) || !vec3_finite(plane_axis) {
        return 0;
    }
    if !vec3_finite(vortex_center) || !vec3_finite(vortex_axis) {
        return 0;
    }
    if nx == 0 || ny == 0 {
        clear_error();
        return 0;
    }
    if !finite_positive(extent_x) || !finite_positive(extent_y) {
        return 0;
    }
    if !finite_positive(healing_length) || !finite_positive(background_density) {
        return 0;
    }
    if out_grid.is_null() {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return 0;
    }

    let total = (nx as usize).saturating_mul(ny as usize);
    let cap = out_len as usize;
    let count = total.min(cap);
    let buf = unsafe { std::slice::from_raw_parts_mut(out_grid, count) };

    // Build local frame in the plane
    let axis = vec3_normalize(plane_axis);
    let ref_dir = if axis.x.abs() < 0.9 {
        vec3_normalize(vec3_cross(axis, Vec3 { x: 1.0, y: 0.0, z: 0.0 }))
    } else {
        vec3_normalize(vec3_cross(axis, Vec3 { x: 0.0, y: 1.0, z: 0.0 }))
    };
    let ref_dir2 = vec3_cross(axis, ref_dir);

    let mut idx = 0usize;
    for iy in 0..ny {
        let fy = (iy as f64) / ((ny - 1).max(1) as f64) - 0.5;
        for ix in 0..nx {
            if idx >= count {
                break;
            }
            let fx = (ix as f64) / ((nx - 1).max(1) as f64) - 0.5;
            let px = plane_center.x + fx * extent_x * ref_dir.x + fy * extent_y * ref_dir2.x;
            let py = plane_center.y + fx * extent_x * ref_dir.y + fy * extent_y * ref_dir2.y;
            let pz = plane_center.z + fx * extent_x * ref_dir.z + fy * extent_y * ref_dir2.z;

            let mut param = GpOrderParameter::default();
            let _ = sf_gp_order_parameter(
                px, py, pz,
                vortex_center, vortex_axis,
                circulation_quantum,
                healing_length, background_density,
                &mut param,
            );

            buf[idx] = GpGridPoint {
                x: fx * extent_x,
                y: fy * extent_y,
                amplitude: param.amplitude,
                phase: param.phase,
                density: param.density,
            };
            idx += 1;
        }
    }

    clear_error();
    idx as u32
}

// ===========================================================================
// G. Utility: healing length and sound speed for ⁴He
// ===========================================================================

/// Compute the healing length ξ = ħ / √(2mgn) given the coupling constant
/// and background density.
#[unsafe(no_mangle)]
pub extern "C" fn sf_healing_length(
    coupling_constant: f64,
    mass: f64,
    background_density: f64,
) -> f64 {
    if !finite_positive(coupling_constant)
        || !finite_positive(mass)
        || !finite_positive(background_density)
    {
        set_error(ERR_INVALID_ARGUMENT, "all parameters must be positive and finite");
        return f64::NAN;
    }
    clear_error();
    let hbar = HBAR;
    hbar / (2.0 * mass * coupling_constant * background_density).sqrt()
}

/// Compute the speed of sound c = √(gn/m) for a superfluid.
#[unsafe(no_mangle)]
pub extern "C" fn sf_sound_speed(
    coupling_constant: f64,
    mass: f64,
    background_density: f64,
) -> f64 {
    if !finite_positive(coupling_constant)
        || !finite_positive(mass)
        || !finite_positive(background_density)
    {
        set_error(ERR_INVALID_ARGUMENT, "all parameters must be positive and finite");
        return f64::NAN;
    }
    clear_error();
    (coupling_constant * background_density / mass).sqrt()
}

/// Return the helium mass constant.
#[unsafe(no_mangle)]
pub extern "C" fn sf_helium_mass() -> f64 {
    HELIUM_MASS
}

/// Return the scattering length for ⁴He.
#[unsafe(no_mangle)]
pub extern "C" fn sf_helium_scattering_length() -> f64 {
    HELIUM_SCATTERING_LENGTH
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::Vec3;

    const VORTEX_CORE: f64 = 1.0e-10;

    fn make_seg(x1: f64, y1: f64, z1: f64, x2: f64, y2: f64, z2: f64) -> VortexSegment {
        VortexSegment {
            start: Vec3 { x: x1, y: y1, z: z1 },
            end: Vec3 { x: x2, y: y2, z: z2 },
            circulation_quantum: 1,
            core_radius: VORTEX_CORE,
        }
    }

    #[test]
    fn biot_savart_induces_velocity() {
        // Segment along x-axis from (0,0,0) to (1,0,0)
        let seg = make_seg(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let field = Vec3 {
            x: 0.5,
            y: 0.1,
            z: 0.0,
        };
        let mut vel = BiotSavartVelocity::default();
        assert_eq!(
            sf_biot_savart_velocity(seg, field, &mut vel),
            Bool::TRUE
        );
        // Velocity should be in the z-direction (for a segment on x-axis and point in xy-plane)
        // Direction of r1×r2 near the segment midpoint should be ±z
        assert!(vel.velocity.z != 0.0 || vel.magnitude.abs() < 1e-20);
        assert!(vel.magnitude >= 0.0);
    }

    #[test]
    fn biot_savart_collinear_returns_zero() {
        // Field point on the line of the segment but outside
        let seg = make_seg(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let field = Vec3 {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        };
        let mut vel = BiotSavartVelocity::default();
        assert_eq!(
            sf_biot_savart_velocity(seg, field, &mut vel),
            Bool::TRUE
        );
        assert_eq!(vel.velocity.x, 0.0);
        assert_eq!(vel.velocity.y, 0.0);
        assert_eq!(vel.velocity.z, 0.0);
    }

    #[test]
    fn biot_savart_on_segment_rejected() {
        let seg = make_seg(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let field = Vec3 {
            x: 0.5,
            y: 0.0,
            z: 0.0,
        }; // on the segment
        let mut vel = BiotSavartVelocity::default();
        assert_eq!(
            sf_biot_savart_velocity(seg, field, &mut vel),
            Bool::FALSE
        );
    }

    #[test]
    fn vortex_ring_velocity_finite() {
        let ring = VortexRing {
            center: Vec3::default(),
            radius: 1.0e-6,
            circulation_quantum: 1,
            axis: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            velocity: Vec3::default(),
        };
        let mut vel = Vec3::default();
        assert_eq!(
            sf_vortex_ring_velocity(ring, &mut vel),
            Bool::TRUE
        );
        // Ring should move along its axis
        assert!(vel.z != 0.0);
        assert!(vel.x == 0.0 && vel.y == 0.0);
        // Speed should be positive (for positive circulation)
        assert!(vel.z != 0.0); // direction depends on quantum sign
    }

    #[test]
    fn circulation_quantum_constant_is_positive() {
        let kappa = sf_circulation_quantum();
        assert!(kappa > 0.0, "circulation quantum must be positive");
        assert!(kappa.is_finite());
    }

    #[test]
    fn gp_order_parameter_vortex_profile() {
        let center = Vec3::default();
        let axis = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        };
        let xi = 1.0;
        let n0 = 1.0;

        // At core center: amplitude should be zero
        let mut param = GpOrderParameter::default();
        assert_eq!(
            sf_gp_order_parameter(0.0, 0.0, 0.0, center, axis, 1, xi, n0, &mut param),
            Bool::TRUE
        );
        assert_eq!(param.amplitude, 0.0, "amplitude at core should be zero");
        assert_eq!(param.density, 0.0, "density at core should be zero");

        // Far from core: amplitude should approach √n₀
        let mut param2 = GpOrderParameter::default();
        assert_eq!(
            sf_gp_order_parameter(10.0, 0.0, 0.0, center, axis, 1, xi, n0, &mut param2),
            Bool::TRUE
        );
        assert!(
            (param2.amplitude - 1.0).abs() < 0.05,
            "far-field amplitude should approach √n₀, got {}",
            param2.amplitude
        );
    }

    #[test]
    fn gp_energy_density_is_nonnegative() {
        let mut energy = GpEnergyDensity::default();
        assert_eq!(
            sf_gp_energy_density(
                1.0, 100.0, // trap frequency
                HELIUM_MASS, 1.0e-10, // coupling
                1.0e-6, // radius from center
                &mut energy,
            ),
            Bool::TRUE
        );
        assert!(energy.kinetic_density >= 0.0);
        assert!(energy.interaction_density >= 0.0);
        assert!(energy.trapping_density >= 0.0);
        assert!(energy.total_density >= 0.0);
        assert!(energy.chemical_potential.is_finite());
    }

    #[test]
    fn gp_simple_evolution_converges() {
        // Imaginary-time evolution should converge to n₀
        let params = GpTimeEvolutionParams {
            healing_length: 1.0,
            sound_speed: 1.0,
            chemical_potential: 1.0,
            coupling_constant: 1.0,
            dt: 0.01,
        };
        let mut amp = 0.1_f64;
        for _ in 0..500 {
            let mut next = 0.0_f64;
            let density = amp * amp;
            let result = sf_gp_amplitude_evolution(amp, density, params, &mut next);
            assert_eq!(result, Bool::TRUE);
            amp = next;
        }
        // Should approach √n₀ = 1.0
        assert!((amp - 1.0).abs() < 0.05, "amplitude should approach 1, got {amp}");
    }

    #[test]
    fn vortex_reconnection_detection() {
        // Two perpendicular segments far apart
        let seg1 = make_seg(-2.0, 0.0, 0.0, 2.0, 0.0, 0.0);
        let seg2 = make_seg(0.0, -2.0, 0.0, 0.0, 2.0, 0.0);
        let xi = VORTEX_CORE;

        let mut report = VortexReconnectionReport::default();
        // With a large reconnection distance covering the crossing, they reconnect
        assert_eq!(
            sf_vortex_reconnection(seg1, seg2, 5.0, xi, &mut report),
            Bool::TRUE
        );
        assert_eq!(report.reconnected, Bool::TRUE);
        assert!(report.closest_approach < 5.0);

        // With a smaller reconnection distance than the closest approach, no reconnect
        // Use segments that don't intersect and are separated
        let seg3 = make_seg(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let seg4 = make_seg(0.0, 0.0, 10.0, 0.0, 1.0, 10.0); // 10 units away
        let mut report2 = VortexReconnectionReport::default();
        assert_eq!(
            sf_vortex_reconnection(seg3, seg4, 1.0, xi, &mut report2),
            Bool::TRUE
        );
        assert_eq!(report2.reconnected, Bool::FALSE);
    }

    #[test]
    fn tangle_stats_with_single_segment() {
        let seg = make_seg(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let mut stats = VortexTangleStats::default();
        assert_eq!(
            sf_vortex_tangle_stats(&seg, 1, 1.0, &mut stats),
            Bool::TRUE
        );
        assert_eq!(stats.segment_count, 1);
        assert!((stats.total_length - 1.0).abs() < 1e-12);
        assert!(stats.total_kinetic_energy > 0.0);
        assert!((stats.vortex_line_density - 1.0).abs() < 1e-12);
    }

    #[test]
    fn tangle_stats_empty() {
        let mut stats = VortexTangleStats::default();
        assert_eq!(
            sf_vortex_tangle_stats(std::ptr::null(), 0, 1.0, &mut stats),
            Bool::TRUE
        );
        // Should return zeros
        assert_eq!(stats.segment_count, 0);
    }

    #[test]
    fn healing_length_and_sound_speed_consistent() {
        let g = 1.0e-10;
        let m = HELIUM_MASS;
        let n = 1.0e20; // ~10²⁰ m⁻³ for ⁴He

        let xi = sf_healing_length(g, m, n);
        assert!(xi.is_finite() && xi > 0.0);

        let c = sf_sound_speed(g, m, n);
        assert!(c.is_finite() && c > 0.0);

        // Consistency: c = ħ / (√2 m ξ)
        let c_from_xi = HBAR / (2.0f64.sqrt() * m * xi);
        assert!((c - c_from_xi).abs() / c < 0.01,
            "sound speed inconsistent with healing length: c={c}, c_from_xi={c_from_xi}");
    }

    #[test]
    fn grid_sample_produces_output() {
        let mut grid = [GpGridPoint::default(); 100];
        let count = sf_gp_grid_sample(
            Vec3::default(), // plane center
            Vec3 { x: 0.0, y: 0.0, z: 1.0 }, // plane axis
            10, 10,
            5.0, 5.0,
            Vec3::default(), // vortex center
            Vec3 { x: 0.0, y: 0.0, z: 1.0 }, // vortex axis
            1,
            1.0, 1.0,
            grid.as_mut_ptr(), 100,
        );
        assert_eq!(count, 100);
        // Center point (core): density ≈ 0
        assert!(grid[45].density < 1.0 || grid[55].density < 1.0);
        // Corner points: density ≈ 1
        assert!(
            (grid[0].density - 1.0).abs() < 0.5 || (grid[99].density - 1.0).abs() < 0.5
        );
    }

    #[test]
    fn null_pointer_rejected() {
        let seg = make_seg(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        assert_eq!(
            sf_biot_savart_velocity(seg, Vec3::default(), std::ptr::null_mut()),
            Bool::FALSE
        );
    }

    #[test]
    fn circulation_quantum_number_estimate() {
        // For a circulation of exactly n quanta, the estimate should match
        let kappa = circulation_quantum_const();
        let radius = 1e-6;
        let n = 3_i32;
        let tangential_v = (n as f64) * kappa / (2.0 * std::f64::consts::PI * radius);
        let samples = [tangential_v; 36];
        let mut quantum = 0_i32;
        assert_eq!(
            sf_quantum_number_estimate(
                samples.as_ptr(),
                radius,
                36,
                &mut quantum,
            ),
            Bool::TRUE
        );
        assert_eq!(quantum, n);
    }
}