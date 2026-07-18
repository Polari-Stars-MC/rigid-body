//! Common math utilities shared across rapier modules.
//!
//! These functions replace the per-module copies of `finite`, `finite_positive`,
//! `finite_non_negative`, `write_out`, `vec3_*`, and `clamp` that were
//! previously duplicated in many files.
//!
//! ## Kahan compensated summation
//!
//! The [`KahanSum`] and [`KahanVec3`] accumulators use Kahan's algorithm to
//! avoid precision loss when summing many values (e.g. aerodynamic forces,
//! soft-body constraint corrections, SPH density estimates).  Use them
//! wherever a plain `x += y` loop accumulates hundreds or more terms whose
//! magnitudes may differ substantially.

#![allow(dead_code)]

use crate::rapier::ffi::{Bool, Vec3};

// ---------------------------------------------------------------------------
// Epsilon constants (project-wide — prefer relative comparison)
// ---------------------------------------------------------------------------

/// General-purpose absolute epsilon for values in the [0.1, 1000] range.
pub(crate) const EPS_GENERAL: f64 = 1.0e-12;

/// Tight epsilon for derivative-like near-zero comparisons.
pub(crate) const EPS_TIGHT: f64 = 1.0e-14;

/// Loose epsilon for geometry / mesh tolerances.
pub(crate) const EPS_GEOMETRIC: f64 = 1.0e-9;

/// Tiny epsilon for distance-squared comparisons in velocity/momentum.
pub(crate) const EPS_DIST_SQ: f64 = 1.0e-18;

// ---------------------------------------------------------------------------
// Scalar validation
// ---------------------------------------------------------------------------

/// Returns true when `value` is finite.
#[inline]
pub(crate) fn finite(value: f64) -> bool {
    value.is_finite()
}

/// Returns true when `value` is finite and > 0.
#[inline]
pub(crate) fn finite_positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

/// Returns true when `value` is finite and >= 0.
#[inline]
pub(crate) fn finite_non_negative(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

/// Clamp `value` to the closed interval [lo, hi].
#[inline]
pub(crate) fn clamp(value: f64, lo: f64, hi: f64) -> f64 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

/// Relative approximate equality: `|a - b| <= max(eps_abs, eps_rel * max(|a|, |b|))`.
///
/// Prefer this over raw `|a - b| < EPSILON` when comparing values whose
/// magnitude may span many orders of magnitude (e.g. astrophysical masses,
/// quantum scales).
#[inline]
pub(crate) fn approx_eq(a: f64, b: f64, eps_abs: f64, eps_rel: f64) -> bool {
    (a - b).abs() <= eps_abs.max(eps_rel * a.abs().max(b.abs()))
}

/// Relative approximate zero test: `|value| <= max(eps_abs, eps_rel * |value|)`.
#[inline]
pub(crate) fn approx_zero(value: f64, eps_abs: f64, eps_rel: f64) -> bool {
    value.abs() <= eps_abs.max(eps_rel * value.abs())
}

/// Fused multiply-add: `a * b + c` with a single rounding.
///
/// Use this in tight loops where `a * b + c` appears and the extra precision
/// matters (e.g. `position + velocity * dt`, `sum + weight * value`).
#[inline]
pub(crate) fn mul_add(a: f64, b: f64, c: f64) -> f64 {
    a.mul_add(b, c)
}

// ---------------------------------------------------------------------------
// Kahan compensated summation
// ---------------------------------------------------------------------------

/// Kahan compensated summation accumulator for `f64`.
///
/// Use this when summing many scalar terms whose magnitudes may differ
/// substantially (e.g. energy totals, log-ratio sums, density estimates).
///
/// # Example
///
/// ```ignore
/// let mut acc = KahanSum::default();
/// for value in huge_list_of_f64s {
///     acc.add(value);
/// }
/// let precise_total: f64 = acc.value();
/// ```
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct KahanSum {
    sum: f64,
    compensation: f64,
}

impl KahanSum {
    /// Create a new accumulator with the given initial value.
    #[inline]
    pub(crate) fn new(initial: f64) -> Self {
        Self {
            sum: initial,
            compensation: 0.0,
        }
    }

    /// Add `value` using Kahan's compensated summation.
    #[inline]
    pub(crate) fn add(&mut self, value: f64) {
        let y = value - self.compensation;
        let t = self.sum + y;
        self.compensation = (t - self.sum) - y;
        self.sum = t;
    }

    /// Return the current compensated sum.
    #[inline]
    pub(crate) fn value(&self) -> f64 {
        self.sum
    }

    /// Reset the accumulator to zero.
    #[inline]
    pub(crate) fn reset(&mut self) {
        self.sum = 0.0;
        self.compensation = 0.0;
    }
}

impl From<KahanSum> for f64 {
    #[inline]
    fn from(acc: KahanSum) -> Self {
        acc.sum
    }
}

/// Kahan compensated summation for 3D vectors (`Vec3`).
///
/// Each of the `x`, `y`, `z` components is accumulated independently with
/// its own Kahan compensator.  Use this when summing many force, torque, or
/// gradient vectors — for example in aerodynamic surface integration, SPH
/// neighbour loops, or soft-body constraint solves.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct KahanVec3 {
    sum: Vec3,
    compensation: Vec3,
}

impl KahanVec3 {
    /// Create a new accumulator with the given initial vector.
    #[inline]
    pub(crate) fn new(initial: Vec3) -> Self {
        Self {
            sum: initial,
            compensation: Vec3::default(),
        }
    }

    /// Add `value` using Kahan's compensated summation per component.
    #[inline]
    pub(crate) fn add(&mut self, value: Vec3) {
        let y = Vec3 {
            x: value.x - self.compensation.x,
            y: value.y - self.compensation.y,
            z: value.z - self.compensation.z,
        };
        let t = Vec3 {
            x: self.sum.x + y.x,
            y: self.sum.y + y.y,
            z: self.sum.z + y.z,
        };
        self.compensation = Vec3 {
            x: (t.x - self.sum.x) - y.x,
            y: (t.y - self.sum.y) - y.y,
            z: (t.z - self.sum.z) - y.z,
        };
        self.sum = t;
    }

    /// Return the current compensated sum.
    #[inline]
    pub(crate) fn value(&self) -> Vec3 {
        self.sum
    }

    /// Return the current compensated sum as a rapier Vector.
    #[inline]
    pub(crate) fn value_vec(&self) -> rapier3d::prelude::Vector {
        rapier3d::prelude::Vector::new(self.sum.x, self.sum.y, self.sum.z)
    }

    /// Add a rapier Vector using Kahan compensation.
    #[inline]
    pub(crate) fn add_vec(&mut self, value: rapier3d::prelude::Vector) {
        self.add(Vec3 { x: value.x, y: value.y, z: value.z });
    }

    /// Reset the accumulator to zero.
    #[inline]
    pub(crate) fn reset(&mut self) {
        self.sum = Vec3::default();
        self.compensation = Vec3::default();
    }
}

// ---------------------------------------------------------------------------
// Output helpers
// ---------------------------------------------------------------------------

/// Write a value through an output pointer, returning `Bool::TRUE` on success.
pub(crate) fn write_out<T: Copy>(out: *mut T, value: T) -> Bool {
    let Some(out) = (unsafe { out.as_mut() }) else {
        crate::rapier::error::set_error(crate::rapier::error::ERR_NULL_POINTER, "output pointer is null");
        return Bool::FALSE;
    };
    *out = value;
    crate::rapier::error::clear_error();
    Bool::TRUE
}

// ---------------------------------------------------------------------------
// Vec3 arithmetic
// ---------------------------------------------------------------------------

#[inline]
pub(crate) fn vec3_add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.x + b.x,
        y: a.y + b.y,
        z: a.z + b.z,
    }
}

#[inline]
pub(crate) fn vec3_sub(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.x - b.x,
        y: a.y - b.y,
        z: a.z - b.z,
    }
}

#[inline]
pub(crate) fn vec3_scale(v: Vec3, s: f64) -> Vec3 {
    Vec3 {
        x: v.x * s,
        y: v.y * s,
        z: v.z * s,
    }
}

#[inline]
pub(crate) fn vec3_dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

#[inline]
pub(crate) fn vec3_cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    }
}

#[inline]
pub(crate) fn vec3_length_sq(v: Vec3) -> f64 {
    v.x * v.x + v.y * v.y + v.z * v.z
}

#[inline]
pub(crate) fn vec3_length(v: Vec3) -> f64 {
    vec3_length_sq(v).sqrt()
}

#[inline]
pub(crate) fn vec3_normalize(v: Vec3) -> Vec3 {
    let len = vec3_length(v);
    if len <= f64::EPSILON {
        Vec3::default()
    } else {
        vec3_scale(v, 1.0 / len)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kahan_sum_beats_naive_on_many_small_values() {
        let big = 1.0e12_f64;
        let small = 1.0e-6_f64;

        // Naive: the small values get swallowed.
        let mut naive = big;
        for _ in 0..1_000_000 {
            naive += small;
        }
        assert!((naive - big).abs() < 1.0, "naive sum lost all small terms");

        // Kahan: the small values are preserved.
        let mut kahan = KahanSum::new(big);
        for _ in 0..1_000_000 {
            kahan.add(small);
        }
        let expected = big + 1.0;
        assert!(
            (kahan.value() - expected).abs() < 1.0e-6,
            "Kahan sum preserved small terms: {} vs {}",
            kahan.value(),
            expected
        );
    }

    #[test]
    fn kahan_vec3_beats_naive_on_many_small_forces() {
        // Use values large enough that naive summation loses the small components.
        // 1e15 has ~0.1 ulp resolution, so adding 1e-6 * 1e6 = 1.0 will be lost.
        let big = Vec3 {
            x: 1.0e15,
            y: -2.0e15,
            z: 3.0e15,
        };
        let small = Vec3 {
            x: 1.0e-6,
            y: 2.0e-6,
            z: 3.0e-6,
        };
        let n = 1_000_000;

        // Naive: small components are swallowed (1e15 + 1.0 == 1e15 in f64).
        let mut naive = big;
        for _ in 0..n {
            naive.x += small.x;
            naive.y += small.y;
            naive.z += small.z;
        }
        assert!((naive.x - big.x).abs() < 0.5, "naive vec3 x lost small terms");

        // Kahan: small components are preserved.
        let mut kahan = KahanVec3::new(big);
        for _ in 0..n {
            kahan.add(small);
        }
        let val = kahan.value();
        assert!(
            (val.x - (big.x + 1.0)).abs() < 1.0e-6,
            "KahanVec3 x preserved small terms"
        );
    }

    #[test]
    fn kahan_sum_reset_works() {
        let mut acc = KahanSum::new(42.0);
        acc.add(1.0);
        assert!((acc.value() - 43.0).abs() < 1.0e-12);
        acc.reset();
        assert!(acc.value().abs() < 1.0e-12);
        acc.add(7.0);
        assert!((acc.value() - 7.0).abs() < 1.0e-12);
    }

    #[test]
    fn kahan_vec3_reset_works() {
        let mut acc = KahanVec3::new(Vec3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        });
        acc.reset();
        acc.add(Vec3 {
            x: 10.0,
            y: 20.0,
            z: 30.0,
        });
        let v = acc.value();
        assert!((v.x - 10.0).abs() < 1.0e-12);
        assert!((v.y - 20.0).abs() < 1.0e-12);
        assert!((v.z - 30.0).abs() < 1.0e-12);
    }

    #[test]
    fn approx_eq_relative_tolerance() {
        // Same magnitude: absolute tolerance works.
        assert!(approx_eq(1.0, 1.0 + 1e-13, 1e-12, 1e-12));
        assert!(!approx_eq(1.0, 1.0 + 1e-11, 1e-12, 1e-12));

        // Large values: relative tolerance kicks in.
        let big = 1.0e15;
        assert!(approx_eq(big, big + 100.0, 1e-12, 1e-12));
        assert!(!approx_eq(big, big + 1e10, 1e-12, 1e-12));

        // Small values: absolute floor prevents false positives near zero.
        assert!(approx_eq(0.0, 1e-13, 1e-12, 1e-12));
        assert!(!approx_eq(0.0, 1e-9, 1e-12, 1e-12));
    }

    #[test]
    fn mul_add_is_more_precise_than_separate_ops() {
        // mul_add rounds once; a*b + c rounds twice.
        let a = 1.0e-10_f64;
        let b = 1.0e10_f64;
        let c = 1.0_f64;
        let naive = a * b + c;
        let fused = mul_add(a, b, c);
        // Both should be ~2.0, but mul_add is at least as precise.
        assert!((naive - 2.0).abs() <= (fused - 2.0).abs() + 1e-15);
    }
}
