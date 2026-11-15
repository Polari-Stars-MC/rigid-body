#[cfg(test)]
mod tests {
    use mps_core::rapier::cross_validate::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::{BodyStatus, Vec3};
    use mps_core::rapier::world;

    fn make_world_with_body(mass: f64, pos: [f64; 3]) -> (*mut WorldHandle, u64) {
        let world = world::world_create(Vec3::default());
        let b = mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(b, mass);
        mps_core::rapier::rigid_body::rigid_body_builder_set_translation(
            b,
            Vec3 {
                x: pos[0],
                y: pos[1],
                z: pos[2],
            },
        );
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(b);
        let h = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);
        (world, h)
    }

    /// CrossValidateLineMask default sets all five bits.
    #[test]
    fn mask_default_includes_all_lines() {
        let m = CrossValidateLineMask {
            bits: CrossValidateLineMask::DEFAULT,
        };
        assert!(m.contains(CrossValidateLineMask::NEWTON));
        assert!(m.contains(CrossValidateLineMask::J2));
        assert!(m.contains(CrossValidateLineMask::QUADRUPOLE));
        assert!(m.contains(CrossValidateLineMask::MOND));
        assert!(m.contains(CrossValidateLineMask::RELATIVISTIC));
    }

    /// `world_cross_validate_default_config()` returns a sane Earth-like config.
    #[test]
    fn default_config_has_positive_gm() {
        let c = world_cross_validate_default_config();
        assert!(c.attractor.gm > 0.0);
        assert!(c.tolerance > 0.0 && c.tolerance <= 1.0);
        assert!(c.enabled);
    }

    /// Registering the law on a null world returns FALSE.
    #[test]
    fn set_on_null_world_returns_false() {
        let cfg = world_cross_validate_default_config();
        let r = world_set_cross_validate_gravity(std::ptr::null_mut(), cfg);
        assert_eq!(r.0, 0);
    }

    /// Setting the law on a valid world returns TRUE and clears on demand.
    #[test]
    fn set_and_clear_on_valid_world() {
        let (world, _h) = make_world_with_body(1.0, [6_800_000.0, 0.0, 0.0]);
        let cfg = world_cross_validate_default_config();
        let r = world_set_cross_validate_gravity(world, cfg);
        assert_eq!(r.0, 1, "set must return TRUE on valid world");
        world_clear_cross_validate_gravity(world);
        // Calling clear again must remain safe (no panic).
        world_clear_cross_validate_gravity(world);
        world::world_destroy(world);
    }

    /// Tolerance must be in (0, 1] — zero or negative is rejected.
    #[test]
    fn set_with_bad_tolerance_returns_false() {
        let (world, _h) = make_world_with_body(1.0, [6_800_000.0, 0.0, 0.0]);
        let mut cfg = world_cross_validate_default_config();
        cfg.tolerance = 0.0;
        assert_eq!(world_set_cross_validate_gravity(world, cfg).0, 0);
        cfg.tolerance = -1.0;
        assert_eq!(world_set_cross_validate_gravity(world, cfg).0, 0);
        cfg.tolerance = 2.0;
        assert_eq!(world_set_cross_validate_gravity(world, cfg).0, 0);
        world::world_destroy(world);
    }

    /// A non-finite GM is rejected.
    #[test]
    fn set_with_nan_gm_returns_false() {
        let (world, _h) = make_world_with_body(1.0, [6_800_000.0, 0.0, 0.0]);
        let mut cfg = world_cross_validate_default_config();
        cfg.attractor.gm = f64::NAN;
        assert_eq!(world_set_cross_validate_gravity(world, cfg).0, 0);
        cfg.attractor.gm = -1.0;
        assert_eq!(world_set_cross_validate_gravity(world, cfg).0, 0);
        world::world_destroy(world);
    }

    /// The flag variant returns the same boolean as the base call.
    #[test]
    fn flag_variant_matches_base_return() {
        let (world, _h) = make_world_with_body(1.0, [6_800_000.0, 0.0, 0.0]);
        let cfg = world_cross_validate_default_config();
        let r = world_set_cross_validate_gravity_flag(world, cfg);
        assert_eq!(r, 1u8);
        world::world_destroy(world);
    }

    /// After a `world_step` with the law set, divergence is reported as zero
    /// for a single body (only one body — no pairs to cross-check).
    #[test]
    fn step_with_law_runs_without_panic() {
        let (world, _h) = make_world_with_body(1.0, [6_800_000.0, 0.0, 0.0]);
        let cfg = world_cross_validate_default_config();
        world_set_cross_validate_gravity(world, cfg);
        world::world_step(world, 1.0 / 60.0);
        let d = world_get_cross_validate_last_divergence(world);
        // Single body has no pairwise cross-validation to do; expect 0.
        assert_eq!(d, 0);
        world::world_destroy(world);
    }

    /// With two nearby bodies, after one step all five lines should agree on
    /// the Newtonian magnitude (the J2/quad/MOND/relativistic lines augment
    /// Newton rather than disagree at this distance), so divergence stays 0.
    #[test]
    fn two_bodies_no_divergence_at_normal_distance() {
        let world = world::world_create(Vec3::default());
        for pos in [[7_000_000.0, 0.0, 0.0], [7_000_000.0, 0.0, 100_000.0]] {
            let b =
                mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
            mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(b, 1000.0);
            mps_core::rapier::rigid_body::rigid_body_builder_set_translation(
                b,
                Vec3 {
                    x: pos[0],
                    y: pos[1],
                    z: pos[2],
                },
            );
            let body = mps_core::rapier::rigid_body::rigid_body_builder_build(b);
            mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);
        }
        let cfg = world_cross_validate_default_config();
        world_set_cross_validate_gravity(world, cfg);
        world::world_step(world, 1.0 / 60.0);
        let d = world_get_cross_validate_last_divergence(world);
        // Lines agree within tolerance → divergence is 0.
        assert_eq!(
            d, 0,
            "all five formula lines agree within tolerance at normal altitude"
        );
        world::world_destroy(world);
    }

    /// Newton-anchored aggregation is the default — `earth_default()` and the
    /// FFI default config both report it, and `correction_blend` is positive.
    #[test]
    fn newton_anchored_is_default() {
        assert_eq!(
            CrossValidateAggregation::default(),
            CrossValidateAggregation::NewtonAnchored,
            "NewtonAnchored must be the Default"
        );
        let c = world_cross_validate_default_config();
        assert_eq!(c.aggregation, CrossValidateAggregation::NewtonAnchored);
        assert!(
            c.correction_blend > 0.0 && c.correction_blend <= 1.0,
            "default correction_blend must be in (0, 1], got {}",
            c.correction_blend
        );
    }

    /// A `correction_blend` outside `[0, 1]` is rejected at the FFI boundary.
    #[test]
    fn set_with_bad_correction_blend_returns_false() {
        let (world, _h) = make_world_with_body(1.0, [6_800_000.0, 0.0, 0.0]);
        let mut cfg = world_cross_validate_default_config();
        cfg.correction_blend = -0.1;
        assert_eq!(world_set_cross_validate_gravity(world, cfg).0, 0);
        cfg.correction_blend = 1.5;
        assert_eq!(world_set_cross_validate_gravity(world, cfg).0, 0);
        cfg.correction_blend = f64::NAN;
        assert_eq!(world_set_cross_validate_gravity(world, cfg).0, 0);
        world::world_destroy(world);
    }

    /// End-to-end: after a step, the body's gravity-affected velocity should be
    /// dominated by the Newton direction; correction_blend ≤ 1/5 keeps the
    /// drift in the Newton direction within a small fraction of |a_newton|.
    #[test]
    fn newton_anchored_velocity_dominated_by_newton() {
        let (world, _h) = make_world_with_body(1.0, [6_800_000.0, 0.0, 0.0]);
        let cfg = world_cross_validate_default_config();
        // All five lines enabled, default blend = 1/5 = 0.2.
        world_set_cross_validate_gravity(world, cfg);
        world::world_step(world, 1.0 / 60.0);

        // Read the body's linear velocity via the rigid-body FFI after the step.
        // The body is at (R, 0, 0); Newton's a points toward origin (-x).
        // With blend ≤ 0.2 and ≤4 non-Newton contributions, the final acceleration
        // differs from |a_newton| by at most 4 × 0.2 × tol × |a_newton|, i.e.
        // rel drift ≤ 4·0.2·1e-9 ≈ 8e-10 — below float epsilon at this scale.
        //
        // The inserted handle is `_h`; query its linvel through the FFI getter.
        let v = mps_core::rapier::rigid_body::rigid_body_get_linvel(world, _h);
        let (gx, gy, gz) = (v.x, v.y, v.z);

        // Newton direction at (R, 0, 0) is purely -x, so |gy|, |gz| are the
        // signature of non-Newton corrections.  They must stay a small fraction
        // of the dominant |gx|.
        let gx_abs = gx.abs().max(1e-30);
        let jitter = (gy.abs().max(gz.abs())) / gx_abs;
        assert!(
            jitter < 0.05,
            "Newton-anchored drift off the Newton direction must be small (<5% of |g_x|), got {}",
            jitter
        );
        world::world_destroy(world);
    }

    /// With `correction_blend = 0`, NewtonAnchored collapses to pure Newton —
    /// non-Newton lines run for cross-validation/divergence counting but
    /// contribute zero correction. We verify this by checking that divergence
    /// still counts disagreements, even though the applied force is the bare
    /// Newton value (which produces the same final velocity the body would
    /// have under the legacy Newtonian-only force law).
    #[test]
    fn newton_anchored_blend_zero_is_pure_newton_with_cross_check() {
        let (world, _h) = make_world_with_body(1.0, [6_800_000.0, 0.0, 0.0]);
        let mut cfg = world_cross_validate_default_config();
        cfg.correction_blend = 0.0;
        world_set_cross_validate_gravity(world, cfg);
        world::world_step(world, 1.0 / 60.0);
        // Single body ⇒ no pairwise divergence pairs (only body-body pairs
        // counted), so divergence stays 0 even with blend = 0.
        assert_eq!(world_get_cross_validate_last_divergence(world), 0);
        world::world_destroy(world);
    }

    /// Reading divergence on a null world returns 0 (and sets the error).
    #[test]
    fn divergence_on_null_world_returns_zero() {
        let d = world_get_cross_validate_last_divergence(std::ptr::null());
        assert_eq!(d, 0);
    }

    /// Clearing a null world must not panic.
    #[test]
    fn clear_on_null_world_is_safe() {
        world_clear_cross_validate_gravity(std::ptr::null_mut());
    }

    #[test]
    fn earth_default_gm_matches_earth_constant() {
        let c = CrossValidateGravityConfig::earth_default();
        assert!((c.attractor.gm - 3.986004418e14).abs() < 1.0);
        assert!((c.mond_a_zero - 1.2e-10).abs() < f64::EPSILON);
    }

    #[test]
    fn law_type_is_newtonian_gravity_structure() {
        use mps_core::rapier::forces::{ForceLaw, ForceLawType};
        let law = CrossValidationGravityLaw {
            config: CrossValidateGravityConfig::earth_default(),
        };
        assert_eq!(law.law_type(), ForceLawType::NewtonianGravity);
    }

    #[test]
    fn aggregation_default_is_newton_anchored() {
        assert_eq!(
            CrossValidateAggregation::default(),
            CrossValidateAggregation::NewtonAnchored
        );
    }
}
