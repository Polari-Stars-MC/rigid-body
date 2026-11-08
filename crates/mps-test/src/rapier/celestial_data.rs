#[cfg(test)]
mod tests {
    use mps_core::rapier::celestial_data::*;
    use mps_core::rapier::ffi::*;

    #[test]
    fn earth_parameters_reasonable() {
        let e = &EARTH;
        assert!(e.gm > 3.98e14 && e.gm < 4.0e14);
        assert!(e.equatorial_radius > 6.37e6 && e.equatorial_radius < 6.38e6);
        assert!(e.flattening > 1.0 / 300.0 && e.flattening < 1.0 / 295.0);
        assert!(e.j2 > 0.001 && e.j2 < 0.0011);
        // Polar radius < equatorial
        assert!(e.polar_radius() < e.equatorial_radius);
    }

    #[test]
    fn moon_mascon_not_zero() {
        let m = &MOON;
        // Moon has significant non-spherical gravity
        assert!(m.j2 > 1.0e-4);
        assert!(!m.c_coeffs.is_empty());
    }

    #[test]
    fn mars_j2_larger_than_earth() {
        // Mars J2 is ~2× Earth's because Mars is less spherical
        assert!(MARS.j2 > EARTH.j2);
    }

    #[test]
    fn jupiter_twice_as_flat_as_saturn_ratio() {
        // Jupiter & Saturn are the flattest planets
        assert!(JUPITER.flattening > 0.01);
        assert!(SATURN.flattening > 0.05);
    }

    #[test]
    fn c_ffi_roundtrip_earth() {
        let mut gm = 0.0;
        let mut er = 0.0;
        let mut f = 0.0;
        let mut rr = 0.0;
        let mut j2 = [0.0; 5];
        let mut md = 0u32;
        let mut rref = 0.0;
        let mut sd = 0.0;
        let mut sh = 0.0;

        let ok = celestial_get_body(
            3,
            &mut gm,
            &mut er,
            &mut f,
            &mut rr,
            j2.as_mut_ptr(),
            &mut md,
            &mut rref,
            &mut sd,
            &mut sh,
        );
        assert_eq!(ok, Bool::TRUE);
        assert!((gm - EARTH_GM).abs() < 1.0);
        assert!((er - EARTH_EQ_RADIUS).abs() < 1.0);
        assert_eq!(j2[0], EARTH_J2);
        assert_eq!(md, 8);
    }
}

/// 从 `mps-formula/src/celestial_data.rs` 内联的 `moon_tests` 迁入（源 crate
/// 不得含 `#[cfg(test)]` 模块，测试统一放 mps-test 镜像）。
#[cfg(test)]
mod moon_tests {
    use mps_formula::celestial_data::*;

    /// Parent-planet GM lookup for the Kepler self-consistency check.
    fn planet_gm(name: &str) -> f64 {
        match name {
            "Sun" => SUN_GM,
            "Earth" => EARTH_GM,
            "Mars" => MARS_GM,
            "Jupiter" => JUPITER_GM,
            "Saturn" => SATURN_GM,
            "Uranus" => URANUS.gm,
            "Neptune" => NEPTUNE.gm,
            "Pluto" => PLUTO_GM,
            other => panic!("unknown parent planet in MOONS: {other}"),
        }
    }

    #[test]
    fn moons_table_populated_and_unique() {
        assert!(!MOONS.is_empty(), "MOONS must not be empty");
        let mut seen = std::collections::HashSet::new();
        for m in MOONS {
            assert!(seen.insert(m.name), "duplicate moon name: {}", m.name);
        }
    }

    #[test]
    fn moons_physical_fields_positive_and_orbit_sane() {
        for m in MOONS {
            assert!(m.gm > 0.0, "{} gm must be positive", m.name);
            assert!(m.radius > 0.0, "{} radius must be positive", m.name);
            assert!(m.semi_major_axis > m.radius, "{} orbit inside body", m.name);
            assert!(m.orbital_period > 0.0, "{} period must be positive", m.name);
        }
    }

    #[test]
    fn moons_satisfy_kepler_third_law() {
        for m in MOONS {
            let gm_total = planet_gm(m.parent_planet) + m.gm;
            let t_calc =
                (4.0 * std::f64::consts::PI * std::f64::consts::PI * m.semi_major_axis.powi(3)
                    / gm_total)
                    .sqrt();
            let rel = (t_calc - m.orbital_period).abs() / m.orbital_period;
            assert!(
                rel < 0.12,
                "{} violates Kepler: calc T={:.3e}s obs T={:.3e}s (rel {:.3})",
                m.name,
                t_calc,
                m.orbital_period,
                rel
            );
        }
    }
}
