#[cfg(test)]
mod tests {
    use mps_core::rapier::wave_optics::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::ComplexAmplitude;

    const VISIBLE_RED: f64 = 700e-9;
    const VISIBLE_GREEN: f64 = 550e-9;
    const VISIBLE_BLUE: f64 = 450e-9;

    #[test]
    fn wavenumber_from_wavelength() {
        let k = wo_wavenumber(VISIBLE_GREEN);
        assert!(k.is_finite() && k > 0.0);
        let lambda = wo_wavelength(k);
        assert!((lambda - VISIBLE_GREEN).abs() / VISIBLE_GREEN < 1e-12);
    }

    #[test]
    fn plane_wave_at_origin() {
        let params = PlaneWaveParams::default();
        let mut amp = ComplexAmplitude::default();
        assert_eq!(
            wo_plane_wave(params, 0.0, 0.0, 0.0, 0.0, 0.0, params.wavenumber, &mut amp),
            Bool::TRUE
        );
        // At origin, E = A₀ · exp(-i φ₀) = A₀ since φ₀=0
        assert!((amp.real - params.amplitude).abs() < 1e-12);
        assert!((amp.intensity - params.amplitude * params.amplitude).abs() < 1e-12);
    }

    #[test]
    fn spherical_wave_decays_with_distance() {
        let k = wo_wavenumber(VISIBLE_GREEN);
        let mut wave1 = SphericalWavePoint::default();
        let mut wave2 = SphericalWavePoint::default();

        assert_eq!(
            wo_spherical_wave(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, k, 1.0, &mut wave1),
            Bool::TRUE
        );
        assert_eq!(
            wo_spherical_wave(0.0, 0.0, 0.0, 2.0, 0.0, 0.0, k, 1.0, &mut wave2),
            Bool::TRUE
        );

        // Intensity should be ~4× smaller at 2× distance (1/r² decay)
        let ratio = wave1.amplitude.intensity / wave2.amplitude.intensity;
        assert!((ratio - 4.0).abs() < 0.01, "intensity ratio should be ~4, got {ratio}");
    }

    #[test]
    fn huygens_fresnel_superposition() {
        // Two in-phase point sources at symmetric positions
        let sources = [
            PointSource {
                x: -0.5,
                y: 0.0,
                z: 0.0,
                phase: 0.0,
                amplitude: 1.0,
            },
            PointSource {
                x: 0.5,
                y: 0.0,
                z: 0.0,
                phase: 0.0,
                amplitude: 1.0,
            },
        ];
        let k = wo_wavenumber(VISIBLE_GREEN);
        let mut amp = ComplexAmplitude::default();
        // Observation point on the midline (constructive interference)
        assert_eq!(
            wo_huygens_fresnel(sources.as_ptr(), 2, 0.0, 0.0, 10.0, k, &mut amp),
            Bool::TRUE
        );
        assert!(amp.intensity > 0.0);
    }

    #[test]
    fn young_slit_interference_central_maximum() {
        let mut point = YoungSlitPoint::default();
        assert_eq!(
            wo_young_slit_point(
                1e-3,  // d = 1 mm
                5e-5,  // a = 0.05 mm
                1.0,   // D = 1 m
                VISIBLE_GREEN,
                0.0,   // on-axis
                0.0,
                &mut point,
            ),
            Bool::TRUE
        );
        // On-axis: constructive interference, intensity should be 1.0
        assert!((point.intensity - 1.0).abs() < 1e-10);
        assert_eq!(point.path_difference, 0.0);
    }

    #[test]
    fn young_slit_first_minimum() {
        let d = 1e-3;
        let lambda = VISIBLE_GREEN;
        let d_screen = 1.0;
        // First minimum when d·sinθ = λ/2 → x = λD / (2d)
        let x_min = lambda * d_screen / (2.0 * d);

        let mut point = YoungSlitPoint::default();
        assert_eq!(
            wo_young_slit_point(d, 0.0, d_screen, lambda, x_min, 0.0, &mut point),
            Bool::TRUE
        );
        // Phase difference should be π
        assert!(
            (point.phase_difference - PI).abs() < 0.01,
            "phase diff should be π at first minimum, got {}",
            point.phase_difference
        );
        // Intensity should be near zero
        assert!(point.intensity < 0.01, "intensity at first minimum should be ~0, got {}", point.intensity);
    }

    #[test]
    fn young_slit_pattern_fills_buffer() {
        let mut intensities = [0.0_f64; 51];
        let count = wo_young_slit_pattern(
            1e-3, 0.0, 1.0, VISIBLE_GREEN,
            -0.01, 0.01, 51,
            intensities.as_mut_ptr(), 51,
        );
        assert_eq!(count, 51);
        // Central point (at x=0 with odd number of points) should be brightest
        // due to constructive interference at the centre
        let mid = 25; // exact centre
        assert!(
            intensities[mid] > 0.99,
            "central intensity should be near 1, got {}",
            intensities[mid]
        );
    }

    #[test]
    fn thin_film_constructive_interference() {
        // For normal incidence on a film with n_film=1.5, n_incident=1.0, n_substrate=1.0
        // Half-wave loss occurs at top surface only → net π shift
        // Constructive when: 2 n t = (m + 1/2) λ
        let n_film = 1.5;
        let t = 500e-9;
        let lambda = 2.0 * n_film * t / 0.5; // m=0: λ = 4 n t = 3000 nm → near IR

        let params = ThinFilmParams {
            thickness: t,
            n_film,
            n_substrate: 1.0,
            n_incident: 1.0,
            incidence_angle: 0.0,
        };

        let mut report = ThinFilmInterferenceReport::default();
        assert_eq!(
            wo_thin_film_interference(params, lambda, &mut report),
            Bool::TRUE
        );
        // For m=0, 2nt = λ/2 → δ = π + π = 2π → constructive
        assert!(report.intensity > 0.99, "constructive intensity should be near 1, got {}", report.intensity);
        assert_eq!(report.half_wave_loss, Bool::TRUE);
    }

    #[test]
    fn thin_film_destructive_interference() {
        // Destructive when: 2 n t = m λ (with half-wave loss)
        let n_film = 1.5;
        let t = 500e-9;
        let lambda = 2.0 * n_film * t / 1.0; // m=1: λ = 1500 nm

        let params = ThinFilmParams {
            thickness: t,
            n_film,
            n_substrate: 1.0,
            n_incident: 1.0,
            incidence_angle: 0.0,
        };

        let mut report = ThinFilmInterferenceReport::default();
        assert_eq!(
            wo_thin_film_interference(params, lambda, &mut report),
            Bool::TRUE
        );
        // 2nt = λ → δ = 2π + π = 3π → destructive (cos(3π) = -1, so I = 0)
        assert!(report.intensity < 0.01, "destructive intensity should be near 0, got {}", report.intensity);
    }

    #[test]
    fn thin_film_spectrum_writes_intensities() {
        let params = ThinFilmParams {
            thickness: 500e-9,
            n_film: 1.5,
            n_substrate: 1.0,
            n_incident: 1.0,
            incidence_angle: 0.0,
        };
        let waves = [VISIBLE_RED, VISIBLE_GREEN, VISIBLE_BLUE];
        let mut intensities = [0.0_f64; 3];
        let count = wo_thin_film_spectrum(
            params,
            waves.as_ptr(),
            intensities.as_mut_ptr(),
            3,
        );
        assert_eq!(count, 3);
        for &i in intensities.iter() {
            assert!((0.0..=1.0).contains(&i));
        }
    }

    #[test]
    fn fresnel_zone_radius_increases_with_index() {
        let mut zone1 = FresnelZoneReport::default();
        let mut zone2 = FresnelZoneReport::default();
        assert_eq!(
            wo_fresnel_zone(1, 1.0, VISIBLE_GREEN, &mut zone1),
            Bool::TRUE
        );
        assert_eq!(
            wo_fresnel_zone(2, 1.0, VISIBLE_GREEN, &mut zone2),
            Bool::TRUE
        );
        assert!(zone2.zone_radius > zone1.zone_radius);
        assert!((zone2.zone_radius / zone1.zone_radius - 2.0_f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn fresnel_diffraction_point_basic() {
        let aperture = ApertureDesc {
            half_width_x: 1e-3,
            half_width_y: 1e-3,
            center_x: 0.0,
            center_y: 0.0,
            transmission: 1.0,
        };
        let k = wo_wavenumber(VISIBLE_GREEN);
        let mut point = DiffractionPoint::default();
        assert_eq!(
            wo_fresnel_diffraction_point(
                aperture, 0.0, 0.0, 1.0, k, 8, 8, &mut point,
            ),
            Bool::TRUE
        );
        assert!(point.amplitude.intensity >= 0.0);
        assert!(point.amplitude.intensity.is_finite());
    }

    #[test]
    fn kirchhoff_diffraction_includes_obliquity() {
        let aperture = ApertureDesc {
            half_width_x: 1e-3,
            half_width_y: 1e-3,
            center_x: 0.0,
            center_y: 0.0,
            transmission: 1.0,
        };
        let k = wo_wavenumber(VISIBLE_GREEN);
        let mut point = KirchhoffDiffractionPoint::default();
        assert_eq!(
            wo_kirchhoff_diffraction_point(
                aperture, 0.0, 0.0, 1.0, k, 8, 8, &mut point,
            ),
            Bool::TRUE
        );
        // Obliquity factor should be positive and ≤ 1
        assert!(point.obliquity_factor > 0.0 && point.obliquity_factor <= 1.0);
        assert!(point.amplitude.intensity.is_finite());
    }

    #[test]
    fn fresnel_zone_sum_is_finite() {
        let mut intensity = 0.0;
        assert_eq!(
            wo_fresnel_zone_sum(10, 1.0, VISIBLE_GREEN, &mut intensity),
            Bool::TRUE
        );
        assert!(intensity >= 0.0 && intensity.is_finite());
    }

    #[test]
    fn null_pointer_rejected() {
        let params = PlaneWaveParams::default();
        assert_eq!(
            wo_plane_wave(params, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, std::ptr::null_mut()),
            Bool::FALSE
        );
    }

    #[test]
    fn thin_film_rainbow_colours_vary_with_thickness() {
        // For a thin film, changing thickness should shift which wavelengths
        // are constructive / destructive
        let params_red = ThinFilmParams {
            thickness: 200e-9,
            n_film: 1.5,
            n_substrate: 1.0,
            n_incident: 1.0,
            incidence_angle: 0.0,
        };
        let params_blue = ThinFilmParams {
            thickness: 100e-9,
            ..params_red
        };

        let mut report_r = ThinFilmInterferenceReport::default();
        let mut report_b = ThinFilmInterferenceReport::default();

        // For a fixed wavelength, different thicknesses give different intensities
        wo_thin_film_interference(params_red, VISIBLE_GREEN, &mut report_r);
        wo_thin_film_interference(params_blue, VISIBLE_GREEN, &mut report_b);

        // The intensities should differ
        assert!(
            (report_r.intensity - report_b.intensity).abs() > 0.01,
            "intensities for different thicknesses should differ"
        );
    }

    #[test]
    fn fresnel_grid_produces_output() {
        let aperture = ApertureDesc {
            half_width_x: 1e-3,
            half_width_y: 1e-3,
            center_x: 0.0,
            center_y: 0.0,
            transmission: 1.0,
        };
        let k = wo_wavenumber(VISIBLE_GREEN);
        let mut grid = [DiffractionPoint::default(); 16];
        let count = wo_fresnel_grid(
            aperture, 1.0, k,
            4, 4,
            1e-3, 1e-3,
            4, 4,
            grid.as_mut_ptr(), 16,
        );
        assert_eq!(count, 16);
        // Centre point should have non-zero intensity
        assert!(grid[7].amplitude.intensity > 0.0 || grid[8].amplitude.intensity > 0.0);
    }

    #[test]
    fn spherical_wave_at_source_rejected() {
        let k = wo_wavenumber(VISIBLE_GREEN);
        let mut wave = SphericalWavePoint::default();
        assert_eq!(
            wo_spherical_wave(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, k, 1.0, &mut wave),
            Bool::FALSE
        );
    }
}



