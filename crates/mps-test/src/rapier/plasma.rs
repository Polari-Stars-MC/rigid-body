#[cfg(test)]
mod tests {
    use mps_core::rapier::plasma::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::GridField;

    #[test]
    fn debye_length_positive() {
        let d = pl_debye_length(1e20, 1e4);
        assert!(d.is_finite() && d > 0.0);
        // For a typical fusion plasma: n_e = 10²⁰, T_e = 10⁴ K
        // λ_D ≈ √(ε₀ kT / n e²) ≈ 7e-6 m
        assert!(d > 1e-7 && d < 1e-4);
    }

    #[test]
    fn plasma_frequency_positive() {
        let f = pl_plasma_frequency(1e20);
        assert!(f.is_finite() && f > 0.0);
        // ω_pe ≈ √(10²⁰ * e² / ε₀ m_e) ≈ 5.6e11 rad/s
        assert!(f > 1e10 && f < 1e13);
    }

    #[test]
    fn plasma_params_self_consistent() {
        let mut params = PlasmaParamsReport::default();
        assert_eq!(
            pl_plasma_params(1e20, 1e4, 1e20, 1.672e-27, 1.0, &mut params),
            Bool::TRUE
        );
        assert!(params.debye_length > 0.0);
        assert!(params.plasma_frequency > 0.0);
        assert!(params.ion_plasma_frequency > 0.0);
        // ω_pi should be much smaller than ω_pe
        assert!(params.ion_plasma_frequency < params.plasma_frequency);
        assert!(params.debye_sphere_count > 0.0);
        assert!(params.thermal_velocity > 0.0);
    }

    #[test]
    fn boris_push_conserves_energy_in_b_field() {
        // Pure B-field: kinetic energy should be conserved
        let particle = PicParticle {
            x: 0.0, y: 0.0, z: 0.0,
            vx: 1e6, vy: 0.0, vz: 0.0,
            charge: -ELECTRON_CHARGE,
            mass: ELECTRON_MASS,
            weight: 1.0,
        };
        let field = GridField {
            ex: 0.0, ey: 0.0, ez: 0.0,
            bx: 0.0, by: 0.0, bz: 1.0, // 1 T along z
        };
        let params = BorisPusherParams {
            dt: 1e-11,
            charge_to_mass_ratio: -ELECTRON_CHARGE / ELECTRON_MASS,
        };

        let mut next = PicParticle::default();
        let ke_initial = 0.5 * ELECTRON_MASS * 1e12;

        let mut ke_min = f64::MAX;
        let mut ke_max = 0.0_f64;

        let mut p = particle;
        for _ in 0..100 {
            assert_eq!(pl_boris_push(p, field, params, &mut next), Bool::TRUE);
            let ke = 0.5 * ELECTRON_MASS * (next.vx * next.vx + next.vy * next.vy + next.vz * next.vz);
            if ke < ke_min {
                ke_min = ke;
            }
            if ke > ke_max {
                ke_max = ke;
            }
            p = next;
        }
        // Energy should be conserved to within 1% over 100 gyro-steps
        let drift = (ke_max - ke_min) / ke_initial;
        assert!(drift < 0.01, "energy drift {drift} > 1%");
    }

    #[test]
    fn boris_push_accelerates_in_e_field() {
        const E_FIELD: f64 = 1e5; // 100 kV/m
        const DT: f64 = 1e-9;
        const STEPS: u32 = 10;

        let particle = PicParticle {
            x: 0.0, y: 0.0, z: 0.0,
            vx: 0.0, vy: 0.0, vz: 0.0,
            charge: -ELECTRON_CHARGE,
            mass: ELECTRON_MASS,
            weight: 1.0,
        };
        let field = GridField {
            ex: E_FIELD, ey: 0.0, ez: 0.0,
            bx: 0.0, by: 0.0, bz: 0.0,
        };
        let params = BorisPusherParams {
            dt: DT,
            charge_to_mass_ratio: -ELECTRON_CHARGE / ELECTRON_MASS,
        };

        let mut p = particle;
        for _ in 0..STEPS {
            let mut next = PicParticle::default();
            assert_eq!(pl_boris_push(p, field, params, &mut next), Bool::TRUE);
            p = next;
        }
        // After N steps in constant E-field: v = qE/m * N*dt
        let expected_v = -ELECTRON_CHARGE / ELECTRON_MASS * E_FIELD * DT * (STEPS as f64);
        assert!(
            (p.vx - expected_v).abs() / expected_v.abs() < 0.01,
            "velocity should be ≈{expected_v}, got {}",
            p.vx
        );
    }

    #[test]
    fn interpolate_field_trilinear() {
        let mut grid = [GridField::default(); 27]; // 3×3×3
        // Set a uniform E-field in the x-direction
        for cell in grid.iter_mut() {
            cell.ex = 1.0;
        }
        let mut out = GridField::default();
        assert_eq!(
            pl_interpolate_field(
                grid.as_ptr(), 3, 3, 3, 1.0,
                0.0, 0.0, 0.0,
                0.5, 0.5, 0.5,
                &mut out,
            ),
            Bool::TRUE
        );
        assert!((out.ex - 1.0).abs() < 1e-12);
    }

    #[test]
    fn deposit_particle_charge_conserved() {
        let particle = PicParticle {
            x: 0.0, y: 0.0, z: 0.0,
            vx: 1e5, vy: 0.0, vz: 0.0,
            charge: ELECTRON_CHARGE,
            mass: ELECTRON_MASS,
            weight: 1e10,
        };
        let mut density = ChargeDensityCell::default();
        assert_eq!(
            pl_deposit_particle(particle, 1e-3, 1e-9, &mut density),
            Bool::TRUE
        );
        // Total charge in cell: q*w = e * 1e10
        let total_charge = density.rho * 1e-9; // rho * V_cell
        assert!((total_charge - ELECTRON_CHARGE * 1e10).abs() / (ELECTRON_CHARGE * 1e10) < 1e-10);
    }

    #[test]
    fn vlasov_moments_single_particle() {
        // Single particle has zero temperature in its own bulk frame
        let particle = PicParticle {
            x: 0.0, y: 0.0, z: 0.0,
            vx: 1e5, vy: 2e5, vz: -1e5,
            charge: ELECTRON_CHARGE,
            mass: ELECTRON_MASS,
            weight: 1.0,
        };
        let mut moments = VlasovMomentReport::default();
        assert_eq!(
            pl_vlasov_moments(&particle, 1, &mut moments),
            Bool::TRUE
        );
        assert!((moments.density - 1.0).abs() < 1e-12);
        assert!((moments.ux - 1e5).abs() < 1.0);
        assert!((moments.uy - 2e5).abs() < 1.0);
        assert!((moments.uz + 1e5).abs() < 1.0);
        // For a single particle, T = 0 (no thermal spread around bulk)
        assert!(moments.temperature == 0.0 || moments.temperature.abs() < 1e-20);

        // With multiple particles having different velocities, T > 0
        let particles = [
            PicParticle {
                vx: 1e5, vy: 0.0, vz: 0.0,
                ..particle
            },
            PicParticle {
                vx: -1e5, vy: 0.0, vz: 0.0,
                ..particle
            },
        ];
        let mut moments2 = VlasovMomentReport::default();
        assert_eq!(
            pl_vlasov_moments(particles.as_ptr(), 2, &mut moments2),
            Bool::TRUE
        );
        assert!(
            moments2.temperature > 0.0,
            "temperature should be positive for thermal spread"
        );
        // Bulk velocity should be zero (symmetric)
        assert!(moments2.ux.abs() < 1.0);
    }

    #[test]
    fn poisson_solve_1d_linear_potential() {
        // Uniform charge density → parabolic potential
        let n = 10u32;
        let rho = vec![1.0; n as usize];
        let mut phi = vec![0.0_f64; n as usize];
        let mut e = vec![0.0_f64; n as usize];

        assert_eq!(
            pl_poisson_solve_1d(rho.as_ptr(), n, 0.1, phi.as_mut_ptr(), e.as_mut_ptr()),
            Bool::TRUE
        );
        // φ should be symmetric (parabolic) with maximum at the centre
        assert!(phi[0] == 0.0);
        assert!(phi[n as usize - 1] == 0.0);
        for val in phi.iter().take(n as usize - 1).skip(1) {
            assert!(*val > 0.0, "potential should be positive inside");
        }
        // E-field should be anti-symmetric
        assert!((e[0] + e[n as usize - 1]).abs() < 1e-10 || e[0] * e[n as usize - 1] < 0.0);
    }

    #[test]
    fn find_xpoint_detects_null() {
        // Create a 2D hyperbolic null: Bx = y, By = x
        let nx = 10u32;
        let ny = 10u32;
        let cell = 1.0;
        let mut bx = vec![0.0_f64; (nx * ny) as usize];
        let mut by = vec![0.0_f64; (nx * ny) as usize];

        for iy in 0..ny {
            for ix in 0..nx {
                let idx = (iy * nx + ix) as usize;
                let x = (ix as f64) - (nx as f64 / 2.0);
                let y = (iy as f64) - (ny as f64 / 2.0);
                bx[idx] = y;
                by[idx] = x;
            }
        }

        let mut xpoint = MagneticXPoint::default();
        assert_eq!(
            pl_find_xpoint(
                bx.as_ptr(), by.as_ptr(),
                nx, ny, cell,
                -(nx as f64 / 2.0), -(ny as f64 / 2.0),
                1.0,
                &mut xpoint,
            ),
            Bool::TRUE
        );
        assert_eq!(xpoint.valid, Bool::TRUE);
    }

    #[test]
    fn sweet_parker_and_petschek_rates() {
        let s = 1e8;
        let r_sp = pl_sweet_parker_rate(s);
        assert!(r_sp.is_finite() && r_sp > 0.0);
        // Sweet–Parker: R = 1/√S = 1e-4
        assert!((r_sp - 1e-4).abs() < 1e-6);

        let r_pet = pl_petschek_rate(s);
        assert!(r_pet.is_finite() && r_pet > 0.0);
        // Petschek: R ≈ π / (4 ln S) ≈ 0.043
        assert!(r_pet > r_sp);
    }

    #[test]
    fn alfven_speed_finite() {
        let v = pl_alfven_speed(1.0, 1e20, 1.672e-27);
        assert!(v.is_finite() && v > 0.0);
        // v_A ≈ 69 km/s for B=1T, n=10²⁰, m_i=proton
        assert!(v > 1e3 && v < 1e7, "v_A = {v} should be ~2.2e6");
    }

    #[test]
    fn pic_step_report_generates_stats() {
        let particles = [
            PicParticle {
                x: 0.0, y: 0.0, z: 0.0,
                vx: 1e5, vy: 0.0, vz: 0.0,
                charge: ELECTRON_CHARGE, mass: ELECTRON_MASS, weight: 1.0,
            },
        ];
        let cells = [GridField {
            ex: 1e4, ey: 0.0, ez: 0.0,
            bx: 1.0, by: 0.0, bz: 0.0,
        }];
        let mut report = PicStepReport::default();
        assert_eq!(
            pl_pic_step_report(
                particles.as_ptr(), 1,
                cells.as_ptr(), 1,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(report.particle_count, 1);
        assert!(report.total_kinetic_energy > 0.0);
        assert!(report.max_electric_field > 0.0);
        assert!(report.total_field_energy > 0.0);
    }

    #[test]
    fn null_pointer_rejected() {
        let p = PicParticle::default();
        let f = GridField::default();
        let bp = BorisPusherParams::default();
        assert_eq!(
            pl_boris_push(p, f, bp, std::ptr::null_mut()),
            Bool::FALSE
        );
    }
}



