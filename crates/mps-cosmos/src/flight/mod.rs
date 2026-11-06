//! `mps_cosmos::flight` — rotorcraft flight-dynamics integration layer.
//!
//! Unlike `mps_formula::rotor` (pure computation), this module is the
//! **integration layer** that touches Rapier `RigidBody` state directly:
//! it reads the body's translation / linear-velocity / angular-velocity,
//! evaluates the rotor + atmosphere + gravity force/moment totals through
//! `mps_formula::rotor`, and advances the body one time-step using a
//! semi-implicit Euler scheme (matching the convention `mps-core` uses for
//! non-orbital bodies; the orbital leapfrog in [`crate::integrator`] is
//! reserved for long-arc conservative gravity, where it pays for itself).
//!
//! # Submodules
//!
//! | submodule | role |
//! |---|---|
//! | [`dynamics`] | 6-DOF force/moment synthesis + `simulate_one_step` |
//! | [`trim`] | Newton–Raphson trim solver (hover + steady level flight) |
//! | [`stability`] | numerical linearization + 4×4 power-iteration eigenvalue estimate |
//!
//! All units SI; `RigidBody` state read/written through the public Rapier
//! `prelude` API (`translation`, `linvel`, `angvel`, `set_*`).

pub mod dynamics;
pub mod stability;
pub mod trim;

pub use dynamics::{
    FlightControls, FlightDynamics, RigidBodyState, StepConfig, default_airfoil, simulate_one_step,
    total_forces_and_moments,
};
pub use stability::{
    PowerIterationResult, StabilityDerivatives, linearize, longitudinal_modes,
    longitudinal_submatrix, power_iteration,
};
pub use trim::{FlightTarget, TrimControls, TrimError, Trimmer, hover_target, level_flight_target};

use rapier3d::prelude::{Rotation, Vector};

/// Atmosphere interface — the caller supplies density as a function of
/// altitude (and, eventually, wind direction).  Implementations in
/// `mps-cosmos::perturbation` (density table) or a separate ISA standard
/// atmosphere adapter; the trait here decouples the integrator from the
/// atmosphere model.
pub trait Atmosphere {
    /// Air density (kg/m³) at the given altitude above the reference
    /// ellipsoid (m).  Returns `None` when the altitude is outside the
    /// atmosphere model's range (e.g. above the exobase).
    fn density(&self, altitude: f64) -> Option<f64>;
}

/// Standard sea-level density helper — a fixed atmosphere (ISA sea level,
/// 1.225 kg/m³) for tests that want a known constant density without
/// bringing in the perturbation density table.  Public so `mps-test` can
/// use it as a fixture.
pub struct SeaLevelAtmosphere;
impl Atmosphere for SeaLevelAtmosphere {
    fn density(&self, _altitude: f64) -> Option<f64> {
        Some(1.225) // kg/m³ — ISA sea level
    }
}

/// Uniform gravity model — gravity vector is the same for every body, which
/// is the right approximation inside a single rotorcraft's flight envelope
/// (regional scale, well under Earth's curvature).  The integration layer
/// reads `gravity_vector(altitude)` and applies `F = m·g` on the body.
pub trait Gravity {
    /// Gravitational acceleration vector (m/s²) at the given altitude.
    /// Conventionally points along `-z` of the world frame for a flat-Earth
    /// rotorcraft env.
    fn gravity_vector(&self, altitude: f64) -> Vector;
}

/// Constant-`g` flat-Earth gravity used by tests.  Public so `mps-test`
/// can share it as a fixture without re-implementing the trait.
pub struct ConstantGravity {
    pub g: f64,
}
impl Gravity for ConstantGravity {
    fn gravity_vector(&self, _altitude: f64) -> Vector {
        Vector::new(0.0, 0.0, -self.g)
    }
}

/// Convert a Rapier [`Rotation`] (a glam `DQuat` unit quaternion under the
/// hood in the f64 build) into the 3×3 body-to-world rotation matrix.
///
/// Field order is glam's `(x, y, z, w)`; the matrix is the standard
/// "R(body→world)" form, rows are the world-frame components of each body
/// axis.  Exposed so trim and dynamics share one definition of body-frame
/// vector → world-frame projection.
pub(crate) fn rotation_to_matrix(r: &Rotation) -> [[f64; 3]; 3] {
    // glam DQuat exposes .x .y .z .w directly.
    let (x, y, z, w) = (r.x, r.y, r.z, r.w);
    // rogue scale avoided: assume unit quaternion (rapier enforces this).
    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let xy = x * y;
    let xz = x * z;
    let yz = y * z;
    let wx = w * x;
    let wy = w * y;
    let wz = w * z;
    [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]
}

/// Rotate a body-frame vector `v_b` into the world frame using the
/// rotation matrix from [`rotation_to_matrix`].
pub(crate) fn body_to_world(r: &Rotation, v_b: Vector) -> Vector {
    let m = rotation_to_matrix(r);
    Vector::new(
        m[0][0] * v_b.x + m[0][1] * v_b.y + m[0][2] * v_b.z,
        m[1][0] * v_b.x + m[1][1] * v_b.y + m[1][2] * v_b.z,
        m[2][0] * v_b.x + m[2][1] * v_b.y + m[2][2] * v_b.z,
    )
}

/// Rotate a world-frame vector into the body frame (transpose of
/// [`body_to_world`]).
pub(crate) fn world_to_body(r: &Rotation, v_w: Vector) -> Vector {
    let m = rotation_to_matrix(r);
    // transpose: m[col][row]
    Vector::new(
        m[0][0] * v_w.x + m[1][0] * v_w.y + m[2][0] * v_w.z,
        m[0][1] * v_w.x + m[1][1] * v_w.y + m[2][1] * v_w.z,
        m[0][2] * v_w.x + m[1][2] * v_w.y + m[2][2] * v_w.z,
    )
}
