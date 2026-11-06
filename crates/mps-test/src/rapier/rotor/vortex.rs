//! `rotor::vortex` — Biot–Savart segment induced-velocity tests.
//!
//! A straight vortex segment produces a known induced velocity at a
//! perpendicular field point; we check the closed-form value for a simple
//! symmetric configuration.

#[cfg(test)]
mod tests {
    use mps_core::rapier::rotor::*;
    use mps_formula::ffi::Vec3;

    /// A segment along the +x axis, Γ = 1, field point at (0, 1, 0).  The
    /// induced velocity should be along ±z (Biot–Savart) and non-zero.
    #[test]
    fn vortex_segment_induced_velocity_is_biot_savart() {
        let a = Vec3 {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        };
        let b = Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        let p = Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        let v = rotor_vortex_segment_induced_velocity(1.0, a, b, p).expect("induced vel");
        // Cross product r1×r2 is along +z (for r1=(-1,-1,0), r2=(1,-1,0)).
        // Induced velocity: ~ along +z if sign of Γ positive.
        assert!(v.z.abs() > 1.0e-3, "z component is zero: v={v:?}");
        // Sign: Biot–Savart convention with Γ > 0 and segment along +x,
        // field point at +y, gives induced velocity in −z (right-hand rule
        // about the +x filament).
        assert!(
            v.z < 0.0,
            "expected -z induced vel (Γ>0 +x filament @ +y), got {v:?}"
        );
        // x should be zero by symmetry.
        assert!(v.x.abs() < 1.0e-3, "x should be near zero: v={v:?}");
    }

    #[test]
    fn vortex_segment_rejects_degenerate_geometry() {
        // field point coincides with endpoint → singular.
        let a = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let b = Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        // field point exactly on segment endpoint:
        assert!(rotor_vortex_segment_induced_velocity(1.0, a, b, a).is_none());
        // segment seen end-on: a = b
        let p = Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        assert!(rotor_vortex_segment_induced_velocity(1.0, a, a, p).is_none());
        // NaN inputs:
        let nan = Vec3 {
            x: f64::NAN,
            y: 0.0,
            z: 0.0,
        };
        assert!(rotor_vortex_segment_induced_velocity(1.0, nan, b, p).is_none());
        assert!(rotor_vortex_segment_induced_velocity(f64::NAN, a, b, p).is_none());
    }

    #[test]
    fn rotor_tip_circulation_basic() {
        // Γ = T / (2 ρ R V_tip).  All inputs positive → positive Γ.
        let g = rotor_tip_circulation(5000.0, 1.225, 5.0, 200.0).unwrap();
        let expected = 5000.0 / (2.0 * 1.225 * 5.0 * 200.0);
        assert!((g - expected).abs() < 1.0e-9, "Γ={g} expected={expected}");
        // Zero thrust → zero circulation.
        assert!(rotor_tip_circulation(0.0, 1.225, 5.0, 200.0).unwrap().abs() < 1.0e-12);
        // Bad inputs:
        assert!(rotor_tip_circulation(-1.0, 1.225, 5.0, 200.0).is_none());
        assert!(rotor_tip_circulation(5000.0, 0.0, 5.0, 200.0).is_none());
        assert!(rotor_tip_circulation(5000.0, 1.225, 0.0, 200.0).is_none());
        assert!(rotor_tip_circulation(5000.0, 1.225, 5.0, 0.0).is_none());
    }

    #[test]
    fn rotor_wake_sum_adds_segments() {
        let a = Vec3 {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        };
        let b = Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        let p = Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        // Two identical segments at the same location → sum = 2× single.
        let tysegs = [(a, b), (a, b)];
        let two = rotor_wake_induced_velocity(1.0, &tysegs, p);
        let one = rotor_vortex_segment_induced_velocity(1.0, a, b, p).unwrap();
        assert!(
            (two.z - 2.0 * one.z).abs() < 1.0e-6,
            "two.z={} one.z={}",
            two.z,
            one.z
        );
        let empty = rotor_wake_induced_velocity(1.0, &[], p);
        assert!(
            empty.x.abs() < 1.0e-12 && empty.y.abs() < 1.0e-12 && empty.z.abs() < 1.0e-12,
            "expected zero induced velocity, got {empty:?}"
        );
    }
}
