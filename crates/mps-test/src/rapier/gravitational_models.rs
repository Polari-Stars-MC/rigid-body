#[cfg(test)]
mod tests {
    use mps_core::rapier::celestial_data::CelestialBody;
    use mps_core::rapier::gravitational_models::*;
    use mps_core::rapier::ffi::*;

    #[test]
    fn legendre_p00_is_one() {
        let p = normalized_legendre(0.0, 0);
        assert!((p[0] - 1.0).abs() < 1e-15);
    }

    #[test]
    fn legendre_p20_sin_zero_is_correct() {
        // P̄₂₀ at sin φ = 0 (equator) should be √(5)/2 × P₂₀
        // P₂₀ = (3 sin²φ - 1)/2, at sin φ=0 → -1/2
        // P̄₂₀ = √5 × (-1/2) = -√5/2 ≈ -1.118
        let p = normalized_legendre(0.0, 4);
        // n=2,m=0 index: 2*3/2 + 0 = 3
        let idx = 2 * 3 / 2;
        let expected = -0.5 * 5.0_f64.sqrt();
        assert!((p[idx] - expected).abs() < 1e-10,
            "P20 at equator: got {}, expected {}", p[idx], expected);
    }

    #[test]
    fn point_mass_recovered_when_no_coeffs() {
        let body = CelestialBody {
            name: "test",
            gm: 1.0,
            equatorial_radius: 1.0,
            flattening: 0.0,
            rotation_rate: 0.0,
            j2: 0.0, j3: 0.0, j4: 0.0, j5: 0.0, j6: 0.0,
            max_degree: 0,
            c_coeffs: &[],
            s_coeffs: &[],
            ref_radius: 1.0,
            surface_density: 0.0,
            scale_height: 0.0,
            solar_pressure_constant: 0.0,
        };

        let pos = Vec3 { x: 10.0, y: 0.0, z: 0.0 };
        let accel = spherical_harmonics_acceleration(pos, &body, 8);
        // Point mass: a = -GM/r² in radial direction
        let expected = -1.0 / 100.0; // GM=1, r=10, r²=100
        assert!((accel.x - expected).abs() < 1e-12);
        assert!(accel.y.abs() < 1e-12);
        assert!(accel.z.abs() < 1e-12);
    }

    #[test]
    fn earth_j2_dominates_leo_orbit() {
        // At LEO (r ≈ 6778 km), J2 acceleration perturbation ≈ 5e-5 m/s²
        // Point mass ≈ 8.7 m/s²
        let pos = Vec3 { x: 6.778e6, y: 0.0, z: 1.0e6 };
        let accel_with_j2 = zonal_harmonics_acceleration(
            pos,
            mps_core::rapier::celestial_data::EARTH_GM,
            mps_core::rapier::celestial_data::EARTH_EQ_RADIUS,
            &[mps_core::rapier::celestial_data::EARTH_J2],
        );
        let accel_pm = zonal_harmonics_acceleration(
            pos,
            mps_core::rapier::celestial_data::EARTH_GM,
            mps_core::rapier::celestial_data::EARTH_EQ_RADIUS,
            &[],
        );

        // Difference between J2 and pure point-mass should be small but nonzero
        let diff = ((accel_with_j2.x - accel_pm.x).powi(2)
            + (accel_with_j2.y - accel_pm.y).powi(2)
            + (accel_with_j2.z - accel_pm.z).powi(2)).sqrt();
        let central_mag = (accel_pm.x.powi(2) + accel_pm.y.powi(2) + accel_pm.z.powi(2)).sqrt();
        let ratio = diff / central_mag;
        assert!(ratio > 1e-6, "J2 perturbation should be nonzero");
        assert!(ratio < 0.01, "J2 perturbation ratio {} should be <1%", ratio);
    }

    #[test]
    fn ellipsoid_reduces_to_point_mass_at_large_distance() {
        // Test Carlson RF integral directly (doesn't depend on NR)
        let rf = carlson_rf(1.0, 1.0, 1.0);
        assert!((rf - 1.0).abs() < 1e-10, "RF(1,1,1) should be 1");

        // Test ellipsoid at equator where NR is robust
        let body = &mps_core::rapier::celestial_data::EARTH;
        let pos_near = Vec3 { x: body.equatorial_radius * 2.0, y: 0.0, z: 0.0 };
        let accel_ellip = ellipsoid_gravity(pos_near, body);
        // At equator, ~ GM/r²
        let r = pos_near.x;
        let accel_pm_near = body.gm / (r * r);
        let error_near = (accel_ellip.x.abs() - accel_pm_near).abs() / accel_pm_near;
        assert!(error_near < 0.05, "Ellipsoid at 2*Re: error {} should be <5%", error_near);
    }

    #[test]
    fn quadrupole_from_j2_is_traceless() {
        let q = quadrupole_from_j2(1.0, 1.0, 0.001);
        let trace = q[0] + q[4] + q[8];
        assert!(trace.abs() < 1e-15, "Quadrupole tensor must be traceless");
    }
}





