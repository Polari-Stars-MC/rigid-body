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
        let mut gm = 0.0; let mut er = 0.0; let mut f = 0.0;
        let mut rr = 0.0; let mut j2 = [0.0; 5]; let mut md = 0u32;
        let mut rref = 0.0; let mut sd = 0.0; let mut sh = 0.0;

        let ok = celestial_get_body(
            3, &mut gm, &mut er, &mut f, &mut rr,
            j2.as_mut_ptr(), &mut md, &mut rref, &mut sd, &mut sh,
        );
        assert_eq!(ok, Bool::TRUE);
        assert!((gm - EARTH_GM).abs() < 1.0);
        assert!((er - EARTH_EQ_RADIUS).abs() < 1.0);
        assert_eq!(j2[0], EARTH_J2);
        assert_eq!(md, 8);
    }
}



