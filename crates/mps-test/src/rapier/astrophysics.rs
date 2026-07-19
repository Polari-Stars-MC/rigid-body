#[cfg(test)]
mod tests {
    use mps_core::rapier::astrophysics::*;
    use mps_core::rapier::ffi::*;

    fn params() -> NBodySolverParams {
        NBodySolverParams {
            gravitational_constant: 1.0,
            softening: 0.0,
            opening_angle: 0.5,
            multipole_order: 0,
        }
    }

    #[test]
    fn direct_nbody_accelerates_toward_mass() {
        let particles = [
            NBodyParticle {
                position: Vec3::default(),
                velocity: Vec3::default(),
                mass: 1.0,
            },
            NBodyParticle {
                position: Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                velocity: Vec3::default(),
                mass: 2.0,
            },
        ];
        let mut out = [Vec3::default(); 2];
        let mut report = NBodyForceReport::default();
        assert_eq!(
            astro_nbody_direct_accelerations(
                particles.as_ptr(),
                particles.len() as u32,
                params(),
                out.as_mut_ptr(),
                out.len() as u32,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(out[0].x > 0.0);
        assert!(out[1].x < 0.0);
        assert!(report.potential_energy < 0.0);
    }

    #[test]
    fn roche_and_resonance_reports_work() {
        let mut roche = RocheLimitReport::default();
        assert_eq!(
            astro_roche_limit(1.0, 5.0, 1.0, 2.0, &mut roche),
            Bool::TRUE
        );
        assert!(roche.fluid_roche_limit > roche.rigid_roche_limit);
        assert_eq!(roche.inside_fluid_limit, Bool::TRUE);

        let mut resonance = OrbitalResonanceReport::default();
        assert_eq!(
            astro_orbital_resonance_detect(1.0, 2.01, 8, 0.01, &mut resonance),
            Bool::TRUE
        );
        assert_eq!(resonance.ratio_numerator, 2);
        assert_eq!(resonance.ratio_denominator, 1);
        assert_eq!(resonance.resonant, Bool::TRUE);
    }

    #[test]
    fn relativistic_correction_is_finite() {
        let mut report = RelativisticOrbitReport::default();
        assert_eq!(
            astro_relativistic_orbit_correction(
                Vec3 {
                    x: 1.0e7,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: 1.0e4,
                    z: 0.0,
                },
                1.0e30,
                6.67430e-11,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(report.schwarzschild_radius > 0.0);
        assert!(report.correction_acceleration.x.is_finite());
    }
}



