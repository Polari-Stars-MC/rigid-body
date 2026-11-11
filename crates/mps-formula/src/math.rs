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

use crate::ffi::{Bool, Vec3};

// ---------------------------------------------------------------------------
// 3D vector type (Vector3f64)
// ---------------------------------------------------------------------------

/// 3D vector with f64 components - lightweight replacement for nalgebra::Vector3<f64>
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector3f64 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3f64 {
    /// Create a new vector from components
    #[inline]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Zero vector
    #[inline]
    pub fn zeros() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// Unit vector along X axis
    #[inline]
    pub fn x() -> Self {
        Self::new(1.0, 0.0, 0.0)
    }

    /// Unit vector along Y axis
    #[inline]
    pub fn y() -> Self {
        Self::new(0.0, 1.0, 0.0)
    }

    /// Unit vector along Z axis
    #[inline]
    pub fn z() -> Self {
        Self::new(0.0, 0.0, 1.0)
    }

    /// Dot product
    #[inline]
    pub fn dot(&self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Cross product
    #[inline]
    pub fn cross(&self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    /// Squared length
    #[inline]
    pub fn length_squared(&self) -> f64 {
        self.dot(*self)
    }

    /// Length
    #[inline]
    pub fn length(&self) -> f64 {
        self.length_squared().sqrt()
    }

    /// Try to normalize, returning None if the vector is zero
    #[inline]
    pub fn try_normalize(&self) -> Option<Self> {
        let len = self.length();
        if len > f64::EPSILON {
            Some(Self::new(self.x / len, self.y / len, self.z / len))
        } else {
            None
        }
    }

    /// Add two vectors
    #[inline]
    pub fn add(&self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }

    /// Subtract two vectors
    #[inline]
    pub fn sub(&self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }

    /// Scale by scalar
    #[inline]
    pub fn scale(&self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

impl std::ops::Add for Vector3f64 {
    type Output = Self;

    #[inline]
    fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl std::ops::AddAssign for Vector3f64 {
    #[inline]
    fn add_assign(&mut self, other: Self) {
        *self = Self::new(self.x + other.x, self.y + other.y, self.z + other.z);
    }
}

impl std::ops::Sub for Vector3f64 {
    type Output = Self;

    #[inline]
    fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

impl std::ops::SubAssign for Vector3f64 {
    #[inline]
    fn sub_assign(&mut self, other: Self) {
        *self = Self::new(self.x - other.x, self.y - other.y, self.z - other.z);
    }
}

impl std::ops::Neg for Vector3f64 {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl std::ops::Mul<f64> for Vector3f64 {
    type Output = Self;

    #[inline]
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

impl std::ops::MulAssign<f64> for Vector3f64 {
    #[inline]
    fn mul_assign(&mut self, s: f64) {
        *self = Self::new(self.x * s, self.y * s, self.z * s);
    }
}

impl std::ops::Mul<Vector3f64> for f64 {
    type Output = Vector3f64;

    #[inline]
    fn mul(self, v: Vector3f64) -> Vector3f64 {
        Vector3f64::new(self * v.x, self * v.y, self * v.z)
    }
}

impl std::ops::Div<f64> for Vector3f64 {
    type Output = Self;

    #[inline]
    fn div(self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s, self.z / s)
    }
}

// ---------------------------------------------------------------------------
// 3x3 matrix type (Matrix3f64)
// ---------------------------------------------------------------------------

/// 3x3 matrix with f64 components - lightweight replacement for nalgebra::Matrix3<f64>
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Matrix3f64 {
    pub cols: [Vector3f64; 3],
}

impl Matrix3f64 {
    /// Create matrix from column vectors
    #[inline]
    pub fn from_cols(c0: Vector3f64, c1: Vector3f64, c2: Vector3f64) -> Self {
        Self { cols: [c0, c1, c2] }
    }

    /// Try to compute the inverse matrix via the closed-form adjugate method.
    /// Returns None if the matrix is singular (determinant too close to zero).
    pub fn try_inverse(&self) -> Option<Self> {
        let (c0, c1, c2) = (self.cols[0], self.cols[1], self.cols[2]);
        let det = c0.dot(c1.cross(c2));
        if det.abs() < f64::EPSILON {
            return None; // Singular matrix
        }
        let inv_det = 1.0 / det;
        Some(Self::from_cols(
            c1.cross(c2) * inv_det,
            c2.cross(c0) * inv_det,
            c0.cross(c1) * inv_det,
        ))
    }

    /// Identity matrix
    #[inline]
    pub fn identity() -> Self {
        Self::from_cols(Vector3f64::x(), Vector3f64::y(), Vector3f64::z())
    }

    /// Matrix multiplication
    #[inline]
    pub fn mul(&self, other: &Self) -> Self {
        let c0 = other.cols[0];
        let c1 = other.cols[1];
        let c2 = other.cols[2];

        Self::from_cols(
            self.mul_vector(c0),
            self.mul_vector(c1),
            self.mul_vector(c2),
        )
    }

    /// Multiply matrix by vector
    #[inline]
    pub fn mul_vector(&self, v: Vector3f64) -> Vector3f64 {
        Vector3f64::new(
            self.cols[0].x * v.x + self.cols[1].x * v.y + self.cols[2].x * v.z,
            self.cols[0].y * v.x + self.cols[1].y * v.y + self.cols[2].y * v.z,
            self.cols[0].z * v.x + self.cols[1].z * v.y + self.cols[2].z * v.z,
        )
    }

    /// Convert to column-major array
    pub fn to_cols_array(&self) -> [f64; 9] {
        [
            self.cols[0].x,
            self.cols[0].y,
            self.cols[0].z,
            self.cols[1].x,
            self.cols[1].y,
            self.cols[1].z,
            self.cols[2].x,
            self.cols[2].y,
            self.cols[2].z,
        ]
    }
}

impl std::ops::Mul for Matrix3f64 {
    type Output = Self;

    #[inline]
    fn mul(self, other: Self) -> Self {
        Matrix3f64::mul(&self, &other)
    }
}

impl std::ops::Mul<Vector3f64> for Matrix3f64 {
    type Output = Vector3f64;

    #[inline]
    fn mul(self, v: Vector3f64) -> Vector3f64 {
        Matrix3f64::mul_vector(&self, v)
    }
}

// ---------------------------------------------------------------------------
// Epsilon constants (project-wide — prefer relative comparison)
// ---------------------------------------------------------------------------

/// General-purpose absolute epsilon for values in the [0.1, 1000] range.
pub const EPS_GENERAL: f64 = 1.0e-12;

/// Tight epsilon for derivative-like near-zero comparisons.
pub const EPS_TIGHT: f64 = 1.0e-14;

/// Loose epsilon for geometry / mesh tolerances.
pub const EPS_GEOMETRIC: f64 = 1.0e-9;

/// Tiny epsilon for distance-squared comparisons in velocity/momentum.
pub const EPS_DIST_SQ: f64 = 1.0e-18;

// ---------------------------------------------------------------------------
// Scalar validation
// ---------------------------------------------------------------------------

/// Returns true when `value` is finite.
#[inline]
pub fn finite(value: f64) -> bool {
    value.is_finite()
}

/// Returns true when `value` is finite and > 0.
#[inline]
pub fn finite_positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

/// Returns true when `value` is finite and >= 0.
#[inline]
pub fn finite_non_negative(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

/// Returns true when every scalar in `values` is finite.
///
/// Replaces the per-module `finite_3` / `finite_4` / `finite_5` / `finite_6`
/// copies — callers pass `&[a, b, c, d, e]` (or a `&[f64; N]` array directly,
/// which coerces to a slice).  Allocates nothing.
#[inline]
pub fn finite_many(values: &[f64]) -> bool {
    values.iter().all(|v| v.is_finite())
}

/// Returns true when all three components of `v` are finite.
#[inline]
pub fn finite_vec3(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

/// Clamp `value` to the closed interval [0.0, 1.0].
///
/// Common saturation helper for throttle / mixture / weight coefficients that
/// previously had per-module `clamp01` copies in `fluid.rs` and
/// `ffi/convert.rs`.
#[inline]
pub fn clamp01(value: f64) -> f64 {
    value.clamp(0.0, 1.0)
}

/// Clamp `value` to the closed interval [lo, hi].
#[inline]
pub fn clamp(value: f64, lo: f64, hi: f64) -> f64 {
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
pub fn approx_eq(a: f64, b: f64, eps_abs: f64, eps_rel: f64) -> bool {
    (a - b).abs() <= eps_abs.max(eps_rel * a.abs().max(b.abs()))
}

/// Relative approximate zero test: `|value| <= max(eps_abs, eps_rel * |value|)`.
#[inline]
pub fn approx_zero(value: f64, eps_abs: f64, eps_rel: f64) -> bool {
    value.abs() <= eps_abs.max(eps_rel * value.abs())
}

/// Fused multiply-add: `a * b + c` with a single rounding.
///
/// Use this in tight loops where `a * b + c` appears and the extra precision
/// matters (e.g. `position + velocity * dt`, `sum + weight * value`).
#[inline]
pub fn mul_add(a: f64, b: f64, c: f64) -> f64 {
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
pub struct KahanSum {
    sum: f64,
    compensation: f64,
}

impl KahanSum {
    /// Create a new accumulator with the given initial value.
    #[inline]
    pub fn new(initial: f64) -> Self {
        Self {
            sum: initial,
            compensation: 0.0,
        }
    }

    /// Add `value` using Kahan's compensated summation.
    #[inline]
    pub fn add(&mut self, value: f64) {
        let y = value - self.compensation;
        let t = self.sum + y;
        self.compensation = (t - self.sum) - y;
        self.sum = t;
    }

    /// Return the current compensated sum.
    #[inline]
    pub fn value(&self) -> f64 {
        self.sum
    }

    /// Reset the accumulator to zero.
    #[inline]
    pub fn reset(&mut self) {
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
pub struct KahanVec3 {
    sum: Vec3,
    compensation: Vec3,
}

impl KahanVec3 {
    /// Create a new accumulator with the given initial vector.
    #[inline]
    pub fn new(initial: Vec3) -> Self {
        Self {
            sum: initial,
            compensation: Vec3::default(),
        }
    }

    /// Add `value` using Kahan's compensated summation per component.
    #[inline]
    pub fn add(&mut self, value: Vec3) {
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
    pub fn value(&self) -> Vec3 {
        self.sum
    }

    /// Return the current compensated sum as a Vector3f64.
    #[inline]
    pub fn value_vec(&self) -> Vector3f64 {
        Vector3f64::new(self.sum.x, self.sum.y, self.sum.z)
    }

    /// Add a Vector3f64 using Kahan compensation.
    #[inline]
    pub fn add_vec(&mut self, value: Vector3f64) {
        self.add(Vec3 {
            x: value.x,
            y: value.y,
            z: value.z,
        });
    }

    /// Reset the accumulator to zero.
    #[inline]
    pub fn reset(&mut self) {
        self.sum = Vec3::default();
        self.compensation = Vec3::default();
    }

    /// Overwrite the compensated sum, clearing the compensation term.
    ///
    /// Floating-origin recentering uses this to translate the accumulator's
    /// baseline after the world moves: the accumulated *increments* stay valid
    /// (they are frame-independent), but their running base must be re-anchored
    /// to the new origin, and the old large-magnitude compensation becomes
    /// meaningless at the new origin — resetting it is exactly the point of
    /// recentering.
    #[inline]
    pub fn set_value(&mut self, value: Vec3) {
        self.sum = value;
        self.compensation = Vec3::default();
    }
}

// ---------------------------------------------------------------------------
// Output helpers
// ---------------------------------------------------------------------------

/// Write a value through an output pointer, returning `Bool::TRUE` on success.
pub fn write_out<T: Copy>(out: *mut T, value: T) -> Bool {
    let Some(out) = (unsafe { out.as_mut() }) else {
        crate::error::set_error(crate::error::ERR_NULL_POINTER, "output pointer is null");
        return Bool::FALSE;
    };
    *out = value;
    crate::error::clear_error();
    Bool::TRUE
}

// ---------------------------------------------------------------------------
// Vec3 arithmetic
// ---------------------------------------------------------------------------

#[inline]
pub fn vec3_add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.x + b.x,
        y: a.y + b.y,
        z: a.z + b.z,
    }
}

#[inline]
pub fn vec3_sub(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.x - b.x,
        y: a.y - b.y,
        z: a.z - b.z,
    }
}

#[inline]
pub fn vec3_scale(v: Vec3, s: f64) -> Vec3 {
    Vec3 {
        x: v.x * s,
        y: v.y * s,
        z: v.z * s,
    }
}

#[inline]
pub fn vec3_dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

#[inline]
pub fn vec3_cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    }
}

#[inline]
pub fn vec3_length_sq(v: Vec3) -> f64 {
    v.x * v.x + v.y * v.y + v.z * v.z
}

#[inline]
pub fn vec3_length(v: Vec3) -> f64 {
    vec3_length_sq(v).sqrt()
}

#[inline]
pub fn vec3_normalize(v: Vec3) -> Vec3 {
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
