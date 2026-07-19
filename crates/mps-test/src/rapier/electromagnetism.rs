#[cfg(test)]
mod tests {
    use mps_core::rapier::electromagnetism::*;
    use mps_core::rapier::ffi::*;

    fn v3(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    #[test]
    fn electromagnetic_formulas_work() {
        let field = ElectromagneticField {
            electric: v3(1.0, 0.0, 0.0),
            magnetic: v3(0.0, 0.0, 2.0),
        };
        let mut lorentz = LorentzForceReport::default();
        assert_eq!(
            em_lorentz_force(3.0, v3(0.0, 4.0, 0.0), field, 2.0, &mut lorentz),
            Bool::TRUE
        );
        assert_eq!(lorentz.total_force.x, 27.0);

        let mut flux = MagneticFluxReport::default();
        assert_eq!(
            em_magnetic_flux(v3(0.0, 0.0, 2.0), v3(0.0, 0.0, 1.0), 3.0, &mut flux),
            Bool::TRUE
        );
        assert_eq!(flux.flux, 6.0);

        let mut induction = FaradayInductionReport::default();
        assert_eq!(
            em_faraday_induction(2.0, 5.0, 0.5, 10.0, 5.0, &mut induction),
            Bool::TRUE
        );
        assert_eq!(induction.induced_emf, -60.0);
        assert_eq!(induction.induced_current, -12.0);
    }

    #[test]
    fn maxwell_and_yee_updates_work() {
        let field = ElectromagneticField {
            electric: v3(1.0, 0.0, 0.0),
            magnetic: v3(0.0, 1.0, 0.0),
        };
        let mut maxwell = MaxwellPointReport::default();
        assert_eq!(
            em_maxwell_point_update(
                field,
                v3(0.0, 0.0, 1.0),
                v3(0.0, 0.0, 2.0),
                v3(0.0, 0.0, 0.0),
                0.0,
                0.0,
                0.0,
                2.0,
                4.0,
                0.5,
                &mut maxwell,
            ),
            Bool::TRUE
        );
        assert_eq!(maxwell.next_field.magnetic.z, -0.5);

        let electric = [v3(1.0, 0.0, 0.0)];
        let magnetic = [v3(0.0, 1.0, 0.0)];
        let curl_e = [v3(0.0, 0.0, 1.0)];
        let curl_b = [v3(0.0, 0.0, 2.0)];
        let mut out_e = [Vec3::default()];
        let mut out_b = [Vec3::default()];
        let mut report = FdtdYeeReport::default();
        assert_eq!(
            em_fdtd_yee_update(
                electric.as_ptr(),
                magnetic.as_ptr(),
                curl_e.as_ptr(),
                curl_b.as_ptr(),
                1,
                2.0,
                4.0,
                0.0,
                0.5,
                out_e.as_mut_ptr(),
                out_b.as_mut_ptr(),
                1,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(out_b[0].z, -0.5);
        assert_eq!(report.cell_count, 1);
    }
}



