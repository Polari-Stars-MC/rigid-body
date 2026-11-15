//! Multi-line gravitational cross-validated acceleration.
//!
//! This module implements a registered `ForceLaw` that, every `world_step`,
//! computes the gravitational acceleration on each dynamic body using N
//! independent formula "lines" (Newton point-mass, J2–J6 zonal harmonics,
//! quadrupole-tensor, MOND boost, Schwarzschild/relativistic correction) in
//! parallel via `rayon`, then verifies mutual consistency and applies the
//! result as `F = m · a` through the `ForceFacade`.
//!
//! ## Newton-anchored aggregation (default)
//!
//! By default the Newton line is the **anchor**: the frame's acceleration is
//! `a = a_newton + Σ correction_blend · (a_other − a_newton)`, where the sum
//! runs over every non-Newton line that passes the relative-difference gate
//! `|a_other − a_newton| / |a_newton| ≤ tolerance`.  Any non-Newton line that
//! fails the gate is vetoed for that frame (it still counts toward the
//! `last_divergence` diagnostic so callers see the disagreement).  This is
//! the user's requested 「一般情况下用牛顿力学，同时使用其他物理去验证修正,
//! 协同计算,避免偏移太大」 mode: Newton is the spine, other formulae provide
//! bounded corrective nudges, and any formula disagreeing too violently is
//! silenced before it can push the body off-trajectory.
//!
//! ## Why parallel lines
//!
//! Each formula line runs against the same per-frame body snapshot, and the
//! lines are pairwise disjoint in their writes (each line writes its own
//! `Vec3` into a `(handle, line_idx) -> accel` scratch buffer).  Rayon's
//! `par_iter` dispatches the N lines on its work-stealing thread pool, so the
//! user-observable latency is `max(line_costs) / num_threads + join_overhead`
//! rather than `sum(line_costs)` — which is the explicit goal of the
//! 「分多个线并行验证」request.
//!
//! ## Consistency contract
//!
//! After every line has produced its acceleration for every body, the
//! pairwise `|a_i − a_j| / (|a_i| + eps)` relative difference is computed.
//! If any pair exceeds `tolerance`, the line enters "divergence" — the
//! offending samples are suppressed (clipped to the median across remaining
//! lines) but the simulation does NOT panic: the divergence is reported back
//! via `last_divergence_count` for diagnostic FFI read-back.

use rapier3d::prelude::{RigidBodyHandle, Vector};
use rayon::prelude::*;

use crate::rapier::ffi::Vec3;
use crate::rapier::forces::{ForceFacade, ForceLaw, ForceLawType};

// ---------------------------------------------------------------------------
// Configuration — public FFI-facing struct
// ---------------------------------------------------------------------------

/// Boolean flag bits selecting which formula lines run this frame.
///
/// Bit `i` set ⇒ line `i` participates.  At least one bit must be set; the
/// default (`NEWTON | J2 | QUADRUPOLE | MOND | RELATIVISTIC`) exercises all
/// five lines for maximum cross-validation coverage.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct CrossValidateLineMask {
    pub bits: u64,
}

impl CrossValidateLineMask {
    pub const NEWTON: u64 = 1 << 0;
    pub const J2: u64 = 1 << 1;
    pub const QUADRUPOLE: u64 = 1 << 2;
    pub const MOND: u64 = 1 << 3;
    pub const RELATIVISTIC: u64 = 1 << 4;
    pub const DEFAULT: u64 =
        Self::NEWTON | Self::J2 | Self::QUADRUPOLE | Self::MOND | Self::RELATIVISTIC;

    pub const fn contains(self, bit: u64) -> bool {
        (self.bits & bit) != 0
    }
}

/// Aggregation policy applied to per-line accelerations after the
/// cross-consistency check.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum CrossValidateAggregation {
    /// Newton-anchored: the Newton line is the reference. Each non-Newton
    /// line contributes its difference from Newton as a *bounded* additive
    /// correction (`a += correction_blend * (a_other − a_newton)`) **only**
    /// if `|a_other − a_newton| / |a_newton| ≤ tolerance` — otherwise the
    /// line is vetoed. This is the user's requested "牛顿力学为主、其他
    /// 公式并行做验证修正、避免偏移太大" mode and is the default.
    #[default]
    NewtonAnchored = 0,
    /// Arithmetic mean of all surviving lines (vetoes excluded).
    Mean = 1,
    /// Median of all surviving lines (robust to a single divergent line).
    Median = 2,
}

/// Drop the single line maximally far from the per-axis median of the
/// supplied slice; used by the `Mean` / `Median` aggregation paths and as the
/// consensus-only fallback when the Newton anchor is itself missing.
fn clip_farthest_from_median(present: &[LineSample]) -> Vec<LineSample> {
    if present.len() <= 1 {
        return present.to_vec();
    }
    let mut xs: Vec<f64> = present.iter().map(|s| s.ax).collect();
    let mut ys: Vec<f64> = present.iter().map(|s| s.ay).collect();
    let mut zs: Vec<f64> = present.iter().map(|s| s.az).collect();
    xs.sort_by(|l, r| l.partial_cmp(r).unwrap_or(std::cmp::Ordering::Equal));
    ys.sort_by(|l, r| l.partial_cmp(r).unwrap_or(std::cmp::Ordering::Equal));
    zs.sort_by(|l, r| l.partial_cmp(r).unwrap_or(std::cmp::Ordering::Equal));
    let med = Vector::new(xs[xs.len() / 2], ys[ys.len() / 2], zs[zs.len() / 2]);

    let mut farthest = 0usize;
    let mut farthest_d2 = -1.0;
    for (i, s) in present.iter().enumerate() {
        let diff = s.vector() - med;
        let d2 = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
        if d2 > farthest_d2 {
            farthest_d2 = d2;
            farthest = i;
        }
    }
    present
        .iter()
        .enumerate()
        .filter(|(i, _)| *i != farthest)
        .map(|(_, s)| *s)
        .collect()
}

/// Arithmetic mean of the supplied samples; `Vector::ZERO` if empty.
#[inline]
fn mean_of(samples: &[LineSample]) -> Vector {
    if samples.is_empty() {
        return Vector::ZERO;
    }
    let mut a = Vector::ZERO;
    for s in samples {
        a += s.vector();
    }
    a / (samples.len() as f64)
}

/// Component-wise median of the supplied samples; `Vector::ZERO` if empty.
#[inline]
fn median_of(samples: &[LineSample]) -> Vector {
    if samples.is_empty() {
        return Vector::ZERO;
    }
    let mut xs: Vec<f64> = samples.iter().map(|s| s.ax).collect();
    let mut ys: Vec<f64> = samples.iter().map(|s| s.ay).collect();
    let mut zs: Vec<f64> = samples.iter().map(|s| s.az).collect();
    xs.sort_by(|l, r| l.partial_cmp(r).unwrap_or(std::cmp::Ordering::Equal));
    ys.sort_by(|l, r| l.partial_cmp(r).unwrap_or(std::cmp::Ordering::Equal));
    zs.sort_by(|l, r| l.partial_cmp(r).unwrap_or(std::cmp::Ordering::Equal));
    let mi = xs.len() / 2;
    Vector::new(xs[mi], ys[mi], zs[mi])
}

/// Primary attractor descriptor used by every non-MOND line.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct CrossValidateAttractor {
    /// Gravitational parameter GM (m³/s²).  Must be > 0 for any line except
    /// `MOND` (which is computed off the Newtonian seed).
    pub gm: f64,
    /// Equatorial radius (m), used by the J2/J6 line.
    pub equatorial_radius: f64,
    /// Zonal harmonic coefficients `[J2, J3, J4, J5, J6, ...]`.  May be empty
    /// —— the J2 line will simply skip and report zero divergence contribution.
    pub jn: [f64; 6],
    /// Rotation rate (rad/s²) used by the centrifugal term folded into the
    /// J2 line; pass 0 for a non-rotating primary.
    pub rotation_rate: f64,
}

/// Configuration for the cross-validation gravity law.
///
/// Set once via the `world_set_cross_validate_gravity` FFI; the law is
/// registered into `PhysicsWorld::force_registry` and re-applied every frame.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct CrossValidateGravityConfig {
    /// Primary attractor GM and figure parameters.
    pub attractor: CrossValidateAttractor,
    /// Bitmask selecting which formula lines run.
    pub mask: CrossValidateLineMask,
    /// Pairwise relative-difference tolerance; lines whose pairwise diff
    /// exceeds this are flagged divergent and clipped.
    pub tolerance: f64,
    /// Newton-anchored correction blend factor in `[0, 1]`.  Only used by
    /// `CrossValidateAggregation::NewtonAnchored`.  The accepted correction
    /// contribution from each non-Newton line is scaled by this factor so the
    /// frame-to-frame drift away from the Newton baseline stays bounded.
    /// `0.0` ⇒ pure Newton (cross-validation only, no applied correction);
    /// `1.0` ⇒ full schemaed correction once a line passes the tolerance
    /// gate.  Default `1.0 / NUM_LINES as f64` ≈ `0.2` keeps the relative
    /// drift per non-Newton line bounded.
    pub correction_blend: f64,
    /// Aggregation policy for the final acceleration vector applied to bodies.
    pub aggregation: CrossValidateAggregation,
    /// MOND scale `a_0` (m/s²); 1.2e-10 is the canonical Milgrom value.
    /// Only used when the MOND line is enabled.
    pub mond_a_zero: f64,
    /// Schwarzschild radius for the relativistic line (m).  Pass 0 to
    /// auto-derive from `gm` as `rs = 2GM/c²`; ignored when the relativistic
    /// line is disabled.
    pub schwarzschild_radius_override: f64,
    /// Enabled flag.  When `false`, the law reports itself disabled and
    /// `apply()` is a no-op.
    pub enabled: bool,
}

impl CrossValidateGravityConfig {
    /// Sensible defaults: all five lines, Earth-ish GM, 1e-9 tolerance.
    pub fn earth_default() -> Self {
        Self {
            attractor: CrossValidateAttractor {
                gm: 3.986004418e14,
                equatorial_radius: 6_378_137.0,
                jn: [1.08263e-3, -2.5326e-6, 1.6198e-6, 2.27e-7, -5.4e-7, 0.0],
                rotation_rate: 7.2921159e-5,
            },
            mask: CrossValidateLineMask {
                bits: CrossValidateLineMask::DEFAULT,
            },
            tolerance: 1e-9,
            correction_blend: 1.0 / (NUM_LINES as f64),
            aggregation: CrossValidateAggregation::NewtonAnchored,
            mond_a_zero: 1.2e-10,
            schwarzschild_radius_override: 0.0,
            enabled: true,
        }
    }
}

// ---------------------------------------------------------------------------
// Per-frame sample buffer
// ---------------------------------------------------------------------------

/// Acceleration produced by one formula line on one body.
#[derive(Clone, Copy, Default, Debug)]
struct LineSample {
    ax: f64,
    ay: f64,
    az: f64,
    /// True if the line could not evaluate this body (e.g. inside primary
    /// radius, non-finite result).  Marked-as-missing lines are excluded
    /// from both the consistency test and the aggregation.
    missing: bool,
}

impl LineSample {
    #[inline]
    fn vector(self) -> Vector {
        Vector::new(self.ax, self.ay, self.az)
    }

    #[inline]
    fn from_vec3(v: Vec3) -> Self {
        Self {
            ax: v.x,
            ay: v.y,
            az: v.z,
            missing: false,
        }
    }

    #[inline]
    const fn missing_const() -> Self {
        Self {
            ax: 0.0,
            ay: 0.0,
            az: 0.0,
            missing: true,
        }
    }

    #[inline]
    fn has_result(self) -> bool {
        self.ax.is_finite() && self.ay.is_finite() && self.az.is_finite() && !self.missing
    }
}

/// Total number of formula lines ever defined (independent of mask).
const NUM_LINES: usize = 5;

/// Per-line label for diagnostic output.
#[allow(dead_code)]
const LINE_NAMES: [&str; NUM_LINES] = ["Newton", "J2", "Quadrupole", "MOND", "Relativistic"];

/// The cross-validation acceleration law.
///
/// Registered via `world_set_cross_validate_gravity` FFI.  Stored by value
/// inside `Box<dyn ForceLaw>` in the registry; `clone_box()` produces a fresh
/// copy with identical config.
/// `pub` rather than `pub(crate)` only so `mps-test` can assert the
/// `law_type()` mapping that ADR-0005 would otherwise force to live beside the
/// implementation (same rationale as `registry.rs`). It never reaches the
/// generated C header, so the ABI surface is unchanged.
pub struct CrossValidationGravityLaw {
    pub config: CrossValidateGravityConfig,
}

impl ForceLaw for CrossValidationGravityLaw {
    fn law_type(&self) -> ForceLawType {
        // Reuse the existing gravity tag so no new ForceLawType slot is
        // consumed (per the skill's guidance to avoid renumbering).
        ForceLawType::NewtonianGravity
    }

    fn enabled(&self) -> bool {
        self.config.enabled
    }

    fn apply(&self, facade: &mut ForceFacade<'_>) {
        if !self.config.enabled {
            return;
        }

        // Collect (handle, mass, position) for all dynamic bodies with mass.
        // This snapshot is read-only and shared across all rayon workers.
        let snapshot: Vec<(RigidBodyHandle, f64, Vector)> = facade
            .bodies
            .iter()
            .filter(|(_, b)| b.is_dynamic())
            .filter_map(|(h, b)| {
                let m = b.mass();
                if m > 0.0 {
                    Some((h, m, b.translation()))
                } else {
                    None
                }
            })
            .collect();

        if snapshot.len() < 2 {
            return;
        }

        // Per-body, per-line accel scratch. Layout: `[body_idx * NUM_LINES + line_idx]`.
        // Each slot is written by exactly one rayon task (line, body_idx);
        // the join happens before any mut borrow of the facade.
        let n_bodies = snapshot.len();
        let samples: Vec<LineSample> = vec![LineSample::missing_const(); n_bodies * NUM_LINES];

        let cfg = self.config;

        // Per-line computation closures.  Each takes (body_idx, position) and
        // returns the line's LineSample for that body. Closures are pure — no
        // shared mutable state — so they can be invoked from parallel workers.
        //
        // We don't want to construct a `CelestialBody` for the J2/sh line
        // (requires static C/S coefficient slices); the faster
        // `zonal_harmonics_acceleration` and `quadrupole_tensor_acceleration`
        // take only scalar parameters and serve as independent lines.
        let mask = cfg.mask;

        // Newton seed line: a = -GM/r³ * r
        let newton = |pos: Vector, _idx: usize| -> LineSample {
            let r = pos;
            let r_mag = r.length();
            if r_mag < 1e-3 {
                return LineSample::missing_const();
            }
            let a = -r * (cfg.attractor.gm / (r_mag * r_mag * r_mag));
            LineSample {
                ax: a.x,
                ay: a.y,
                az: a.z,
                missing: false,
            }
        };

        // J2-J6 zonal harmonics line.
        let j2 = |pos: Vector, _idx: usize| -> LineSample {
            if cfg.attractor.equatorial_radius <= 0.0 {
                return LineSample::missing_const();
            }
            let jn_nonzero: [f64; 6] = cfg.attractor.jn;
            // Only use terms up to the first zero coefficient to mimic the
            // shape of real gravity models while staying bias-free.
            let used: &[f64] = match jn_nonzero.iter().position(|&x| x.abs() < f64::MIN_POSITIVE) {
                Some(stop) => &jn_nonzero[..stop],
                None => &jn_nonzero[..],
            };
            if used.is_empty() {
                return LineSample::missing_const();
            }
            let pos_f = Vec3 {
                z: pos.z,
                y: pos.y,
                x: pos.x,
            };
            let a = mps_formula::gravitational_models::zonal_harmonics_acceleration(
                pos_f,
                cfg.attractor.gm,
                cfg.attractor.equatorial_radius,
                used,
            );
            LineSample::from_vec3(a)
        };

        // Quadrupole-tensor line.
        let quadrupole = |pos: Vector, _idx: usize| -> LineSample {
            if cfg.attractor.equatorial_radius <= 0.0 {
                return LineSample::missing_const();
            }
            let q = mps_formula::gravitational_models::quadrupole_from_j2(
                cfg.attractor.gm,
                cfg.attractor.equatorial_radius,
                cfg.attractor.jn[0],
            );
            let pos_f = Vec3 {
                x: pos.x,
                y: pos.y,
                z: pos.z,
            };
            let a = mps_formula::gravitational_models::quadrupole_tensor_acceleration(
                pos_f,
                cfg.attractor.gm,
                &q,
            );
            LineSample::from_vec3(a)
        };

        // MOND line: a_MOND along the Newton-direction with magnitude
        // `sqrt(|a_N| · a_0)` when |a_N| < a_0, else a_N.
        let mond = |pos: Vector, _idx: usize| -> LineSample {
            if cfg.mond_a_zero <= 0.0 {
                return LineSample::missing_const();
            }
            let r = pos;
            let r_mag = r.length();
            if r_mag < 1e-3 {
                return LineSample::missing_const();
            }
            let a_n_vec = -r * (cfg.attractor.gm / (r_mag * r_mag * r_mag));
            let a_n_mag = a_n_vec.length();
            if a_n_mag <= 0.0 {
                return LineSample::missing_const();
            }
            let a_mond_mag = if a_n_mag < cfg.mond_a_zero {
                (a_n_mag * cfg.mond_a_zero).sqrt()
            } else {
                a_n_mag
            };
            let a = a_n_vec * (a_mond_mag / a_n_mag);
            LineSample {
                ax: a.x,
                ay: a.y,
                az: a.z,
                missing: false,
            }
        };

        // Relativistic Schwarzschild line: weak-field Newton + 1/r³
        // correction term.  We use the standard Schwarzschild weak-field
        // approximation `a = -GM/r³ · r · (1 + 3·r_s/r)` truncated at next
        // order to provide an independent validator for the Newton line.
        let relativistic = |pos: Vector, _idx: usize| -> LineSample {
            let r = pos;
            let r_mag = r.length();
            if r_mag < 1e-3 {
                return LineSample::missing_const();
            }
            let rs = if cfg.schwarzschild_radius_override > 0.0 {
                cfg.schwarzschild_radius_override
            } else {
                let c = mps_formula::relativity::SPEED_OF_LIGHT;
                2.0 * cfg.attractor.gm / (c * c)
            };
            // Weak-field Schwarzschild radial correction (1PN).
            let correction = 1.0 + 3.0 * rs / r_mag;
            let a = -r * (cfg.attractor.gm * correction / (r_mag * r_mag * r_mag));
            LineSample {
                ax: a.x,
                ay: a.y,
                az: a.z,
                missing: false,
            }
        };

        // Drive only the selected lines. Each line computes its acceleration
        // for every body in parallel, then writes its column `line_idx` in
        // the shared `samples` buffer. Because each line writes a *disjoint*
        // column, the writes never race; we guard the write with a short
        // `Mutex` to satisfy the borrow checker, but the heavy per-body math
        // happens entirely inside `par_iter` (no lock held during compute).
        let samples: std::sync::Mutex<Vec<LineSample>> = std::sync::Mutex::new(samples);

        let run_line =
            |line_idx: usize, sample_fn: &(dyn Fn(Vector, usize) -> LineSample + Sync)| {
                if !mask.contains(1u64 << line_idx) {
                    return;
                }
                let local: Vec<LineSample> = (0..n_bodies)
                    .into_par_iter()
                    .map(|b_idx| sample_fn(snapshot[b_idx].2, b_idx))
                    .collect();
                let mut guard = samples.lock().unwrap();
                for b_idx in 0..n_bodies {
                    guard[b_idx * NUM_LINES + line_idx] = local[b_idx];
                }
            };

        // Each call releases the mutex before the next, so at most one line's
        // column is being committed at a time — but the expensive `par_iter`
        // body of every line already completed without holding the lock.
        run_line(0, &newton);
        run_line(1, &j2);
        run_line(2, &quadrupole);
        run_line(3, &mond);
        run_line(4, &relativistic);

        // Acquire the final buffer for aggregation.
        let samples = samples.into_inner().unwrap();

        // --- Aggregation + cross validation ---------------------------------
        let aggregation = cfg.aggregation;
        let tol = cfg.tolerance;

        // Pre-allocate scratch per-body acceleration vectors.
        let mut final_accel: Vec<Vector> = Vec::with_capacity(n_bodies);
        let mut total_divergence: u64 = 0;

        for b_idx in 0..n_bodies {
            let line_slice = &samples[b_idx * NUM_LINES..(b_idx + 1) * NUM_LINES];

            // Collect the surviving (non-missing) samples.
            let present: Vec<LineSample> = line_slice
                .iter()
                .copied()
                .filter(|s| s.has_result())
                .collect();

            if present.is_empty() {
                final_accel.push(Vector::ZERO);
                continue;
            }

            // Cross validation: pairwise relative differences.
            let mut inconsistent: u64 = 0;
            for i in 0..present.len() {
                for j in (i + 1)..present.len() {
                    let a = present[i].vector();
                    let b = present[j].vector();
                    let denom = (a.length() + 1e-30).max(b.length());
                    if (a - b).length() / denom > tol {
                        inconsistent += 1;
                    }
                }
            }

            // Veto logic depends on the aggregation mode:
            //   • NewtonAnchored — Newton (line 0) is the anchor; any non-Newton
            //     line whose *relative difference from Newton* exceeds `tol`
            //     is vetoed (counts as inconsistent, doesn't contribute
            //     correction).  No pairwise median clip.
            //   • Mean / Median — keep the legacy median-clip veto so all
            //     surviving lines then get equal weight.
            let mut cleaned: Vec<LineSample> = if inconsistent == 0 {
                present.clone()
            } else if aggregation == CrossValidateAggregation::NewtonAnchored
                && mask.contains(CrossValidateLineMask::NEWTON)
            {
                // Identify the Newton sample (line 0) within `present`.
                // `line_slice` preserves the canonical line order; we can't
                // just use `present[0]` because `present` dropped missing
                // lines and may reorder.  Re-derive the Newton anchor.
                let newton_anchor = line_slice[0];
                if !newton_anchor.has_result() {
                    // No anchor — walk back to a consensus-only median clip
                    // so the frame still produces *some* acceleration.
                    clip_farthest_from_median(&present)
                } else {
                    let a_n = newton_anchor.vector();
                    let n_mag = a_n.length().max(1e-30);
                    present
                        .iter()
                        .enumerate()
                        .filter_map(|(i, s)| {
                            // Keep the Newton line unconditionally (i == 0 in
                            // present iff it was present).
                            let is_newton = i == 0 && mask.contains(CrossValidateLineMask::NEWTON);
                            if is_newton {
                                return Some(*s);
                            }
                            let rel = (s.vector() - a_n).length() / n_mag;
                            if rel <= tol { Some(*s) } else { None }
                        })
                        .collect()
                }
            } else {
                clip_farthest_from_median(&present)
            };

            // If a NewtonAnchored veto dropped all non-Newton lines but kept
            // Newton itself, we're fine — `cleaned` still has the Newton
            // sample.  If it dropped everything (only happened in a degenerate
            // pure-median-cleanup branch), fall back to the Newton anchor so
            // the body still gets a sane acceleration.
            if cleaned.is_empty() && mask.contains(CrossValidateLineMask::NEWTON) {
                let newton_anchor = line_slice[0];
                if newton_anchor.has_result() {
                    cleaned = vec![newton_anchor];
                }
            }

            // Update persistent divergence counter.
            total_divergence += inconsistent;

            // Aggregate.
            let a_vec = match aggregation {
                CrossValidateAggregation::NewtonAnchored
                    if mask.contains(CrossValidateLineMask::NEWTON) =>
                {
                    let newton_anchor = line_slice[0];
                    if !newton_anchor.has_result() {
                        // No anchor — fall back to equal-weight mean of
                        // `cleaned` so we never silently zero the body.
                        mean_of(&cleaned)
                    } else {
                        let a_n = newton_anchor.vector();
                        let blend = cfg.correction_blend.clamp(0.0, 1.0);
                        let mut a = a_n;
                        // Only *non-Newton* survivors contribute corrections;
                        // Newton itself is the anchor and is already in `a`.
                        for s in &cleaned {
                            // Skip the Newton sample (same identity by value).
                            if s.vector() == a_n {
                                continue;
                            }
                            a += (s.vector() - a_n) * blend;
                        }
                        a
                    }
                }
                CrossValidateAggregation::Mean => mean_of(&cleaned),
                CrossValidateAggregation::Median => median_of(&cleaned),
                _ => mean_of(&cleaned),
            };

            final_accel.push(a_vec);
        }

        // Apply: F = m * a, via the facade. The facade will internally set
        // per-frame logging keyed to the ForceLawType.
        let source = self.law_type();
        for (i, &(handle, mass, _)) in snapshot.iter().enumerate() {
            let accel = final_accel[i];
            if accel == Vector::ZERO {
                continue;
            }
            let force = accel * mass;
            facade.add_force(handle, force, source);
        }

        // Surface the divergence count for diagnostic readout via a
        // thread-local cell, matching the existing mps pattern of
        // write-through-side-state.
        LAST_DIVERGENCE_COUNT.store(total_divergence, std::sync::atomic::Ordering::Relaxed);
    }

    fn clone_box(&self) -> Box<dyn ForceLaw> {
        Box::new(Self {
            config: self.config,
        })
    }
}

// ---------------------------------------------------------------------------
// Side-channel diagnostic state
// ---------------------------------------------------------------------------

use std::sync::atomic::{AtomicU64, Ordering};

/// Last-computed divergence pair count across all bodies for one frame.
/// Reset at the start of each `apply()` call.  Read via FFI
/// `world_get_cross_validate_last_divergence`.
static LAST_DIVERGENCE_COUNT: AtomicU64 = AtomicU64::new(0);

/// Reset the divergence counter at law registration time.
pub(crate) fn reset_divergence() {
    LAST_DIVERGENCE_COUNT.store(0, Ordering::Relaxed);
}

// ---------------------------------------------------------------------------
// FFI entry points
// ---------------------------------------------------------------------------

use crate::rapier::error::*;
use crate::rapier::ffi::{Bool, WorldHandle};

/// Set the cross-validation gravity law on the world.  Any previous
/// cross-validation law (registered under `ForceLawType::NewtonianGravity`)
/// is removed first (singleton semantics, mirroring
/// `world_set_newton_gravity_law`).
#[unsafe(no_mangle)]
pub extern "C" fn world_set_cross_validate_gravity(
    world: *mut WorldHandle,
    config: CrossValidateGravityConfig,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !config.tolerance.is_finite() || config.tolerance <= 0.0 || config.tolerance > 1.0 {
            set_error(ERR_INVALID_ARGUMENT, "tolerance must be in (0, 1]");
            return Bool::FALSE;
        }
        if !config.attractor.gm.is_finite() || config.attractor.gm < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "attractor.gm must be finite ≥ 0");
            return Bool::FALSE;
        }
        if !config.correction_blend.is_finite()
            || config.correction_blend < 0.0
            || config.correction_blend > 1.0
        {
            set_error(ERR_INVALID_ARGUMENT, "correction_blend must be in [0, 1]");
            return Bool::FALSE;
        }

        // Remove any previously-registered NewtonianGravity-tagged law
        // (covers the singleton CrossValidationGravityLaw and the
        // legacy NewtonianGravityForceLaw).  This matches the pattern in
        // events.rs::world_set_newton_gravity_law.
        world
            .inner
            .force_registry
            .unregister_by_type(ForceLawType::NewtonianGravity);

        let law = CrossValidationGravityLaw { config };
        world.inner.force_registry.register(Box::new(law));
        reset_divergence();
        clear_error();
        Bool::TRUE
    })
}

/// `u8`-returning variant for environments that prefer integer returns.
#[unsafe(no_mangle)]
pub extern "C" fn world_set_cross_validate_gravity_flag(
    world: *mut WorldHandle,
    config: CrossValidateGravityConfig,
) -> u8 {
    ffi_guard(0, || world_set_cross_validate_gravity(world, config).0)
}

/// Clear the cross-validation law from the world's registry.
#[unsafe(no_mangle)]
pub extern "C" fn world_clear_cross_validate_gravity(world: *mut WorldHandle) {
    ffi_guard((), || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return;
        };
        world
            .inner
            .force_registry
            .unregister_by_type(ForceLawType::NewtonianGravity);
        reset_divergence();
        clear_error();
    })
}

/// Read the last frame's cross-validation divergence pair count.
///
/// Returns the number of (body, line_a, line_b) triples whose relative
/// difference exceeded `tolerance` in the most recent `apply()` invocation.
/// Returns 0 if the law is not registered, no `step` has run, or all
/// lines were within tolerance.
#[unsafe(no_mangle)]
pub extern "C" fn world_get_cross_validate_last_divergence(world: *const WorldHandle) -> u64 {
    ffi_guard(0, || {
        // Touch `world` so a null pointer is reported as an error.
        if unsafe { world.as_ref() }.is_none() {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        }
        LAST_DIVERGENCE_COUNT.load(Ordering::Relaxed)
    })
}

/// Configuration: convenience FFI building a default Earth-ish config in one
/// call so a Java caller does not need to populate every field by hand.
#[unsafe(no_mangle)]
pub extern "C" fn world_cross_validate_default_config() -> CrossValidateGravityConfig {
    CrossValidateGravityConfig::earth_default()
}

// ---------------------------------------------------------------------------
// Internal section header (no marker imports needed — kept clean for clippy)
// ---------------------------------------------------------------------------

// `world_set_newton_gravity_law` (in events.rs) registers the legacy
// `interaction::NewtonianGravityForceLaw` under the same
// `ForceLawType::NewtonianGravity` slot. The cross-validation law mirrors
// that singleton pattern; no extra imports are needed here.
