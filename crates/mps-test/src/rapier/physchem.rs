#[cfg(test)]
mod tests {
    use mps_core::rapier::physchem::*;
    use mps_core::rapier::ffi::*;

    fn params() -> GrayScottParams {
        GrayScottParams {
            diffusion_u: 0.16,
            diffusion_v: 0.08,
            feed_rate: 0.060,
            kill_rate: 0.062,
            dx: 1.0,
        }
    }

    #[test]
    fn catalyst_and_gray_scott_terms_work() {
        let catalyst = CatalystEffect {
            concentration: 2.0,
            strength: 1.0,
            saturation: 2.0,
        };
        let mut catalyst_report = CatalystReport::default();
        assert_eq!(
            physchem_catalyst_rate_multiplier(4.0, catalyst, &mut catalyst_report),
            Bool::TRUE
        );
        assert_eq!(catalyst_report.rate_multiplier, 1.5);
        assert_eq!(catalyst_report.effective_rate, 6.0);

        let mut terms = GrayScottReactionReport::default();
        assert_eq!(
            physchem_gray_scott_reaction_terms(1.0, 0.5, 0.0, 0.0, params(), catalyst, &mut terms),
            Bool::TRUE
        );
        assert!(terms.reaction_rate > 0.0);
        assert!(terms.du_dt < 0.0);
        assert!(terms.dv_dt > 0.0);
    }

    #[test]
    fn gray_scott_grid_step_updates_concentrations() {
        let u = [1.0, 1.0, 1.0, 1.0];
        let v = [0.0, 0.2, 0.0, 0.0];
        let mut out_u = [0.0; 4];
        let mut out_v = [0.0; 4];
        let mut report = ReactionDiffusionReport::default();
        assert_eq!(
            physchem_gray_scott_step_2d(
                u.as_ptr(),
                v.as_ptr(),
                2,
                2,
                params(),
                CatalystEffect::default(),
                0.1,
                out_u.as_mut_ptr(),
                out_v.as_mut_ptr(),
                4,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(report.cell_count, 4);
        assert!(report.max_delta_v > 0.0);
        assert!(out_u[1] < 1.0);
    }

    #[test]
    fn concentration_buoyancy_maps_density_to_force() {
        let mut report = ConcentrationBuoyancyReport::default();
        assert_eq!(
            physchem_concentration_buoyancy(
                2.0,
                1.0,
                1000.0,
                0.1,
                0.5,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(report.density < 1000.0);
        assert!(report.buoyancy_force.y < 0.0);
    }
}



