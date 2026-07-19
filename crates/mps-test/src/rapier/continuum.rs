#[cfg(test)]
mod tests {
    use mps_core::rapier::continuum::*;
    use mps_core::rapier::ffi::*;

    fn tetra() -> FemTetrahedron {
        FemTetrahedron {
            a: Vec3::default(),
            b: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            c: Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            d: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        }
    }

    fn material() -> MaterialProperties {
        MaterialProperties {
            youngs_modulus: 1000.0,
            poisson_ratio: 0.25,
            ..MaterialProperties::default()
        }
    }

    #[test]
    fn tetra_shape_functions_and_volume_work() {
        assert!((continuum_tetra_volume(tetra()) - 1.0 / 6.0).abs() < 1.0e-12);
        let mut report = FemShapeFunctionReport::default();
        assert_eq!(
            continuum_tetra_shape_functions(
                tetra(),
                Vec3 {
                    x: 0.25,
                    y: 0.25,
                    z: 0.25
                },
                &mut report
            ),
            Bool::TRUE
        );
        assert_eq!(report.inside, Bool::TRUE);
        assert!(
            report
                .weights
                .iter()
                .all(|weight| (*weight - 0.25).abs() < 1.0e-12)
        );
    }

    #[test]
    fn constitutive_and_tetra_stiffness_work() {
        let mut d = [0.0; 36];
        let mut report = FemConstitutiveReport::default();
        assert_eq!(
            continuum_linear_elastic_constitutive_matrix(
                material(),
                d.as_mut_ptr(),
                d.len() as u32,
                &mut report
            ),
            Bool::TRUE
        );
        assert!(d[0] > d[3 * 6 + 3]);
        assert!(report.shear_modulus > 0.0);

        let mut k = [0.0; 144];
        let mut volume = 0.0;
        assert_eq!(
            continuum_linear_tetra_element_stiffness(
                tetra(),
                material(),
                k.as_mut_ptr(),
                k.len() as u32,
                &mut volume
            ),
            Bool::TRUE
        );
        assert!(volume > 0.0);
        assert!(k[0] > 0.0);
    }

    #[test]
    fn newmark_beta_solve_advances_sdof() {
        let m = [1.0];
        let c = [0.0];
        let k = [4.0];
        let u = [0.0];
        let v = [0.0];
        let a = [0.0];
        let f = [1.0];
        let mut du = [0.0];
        let mut next_u = [0.0];
        let mut next_v = [0.0];
        let mut next_a = [0.0];
        let mut report = NewmarkBetaReport::default();
        assert_eq!(
            continuum_newmark_beta_solve(
                m.as_ptr(),
                c.as_ptr(),
                k.as_ptr(),
                u.as_ptr(),
                v.as_ptr(),
                a.as_ptr(),
                f.as_ptr(),
                1,
                NewmarkBetaParameters {
                    beta: 0.25,
                    gamma: 0.5,
                    dt: 0.1
                },
                du.as_mut_ptr(),
                next_u.as_mut_ptr(),
                next_v.as_mut_ptr(),
                next_a.as_mut_ptr(),
                1,
                &mut report
            ),
            Bool::TRUE
        );
        assert!(du[0] > 0.0);
        assert!(next_u[0] > 0.0);
        assert!(report.residual_norm < 1.0e-10);
    }
}



