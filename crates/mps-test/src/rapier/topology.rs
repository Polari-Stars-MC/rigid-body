#[cfg(test)]
mod tests {
    use mps_core::rapier::topology::*;
    use mps_core::rapier::ffi::*;

    fn params() -> TopologyOptimizationParams {
        TopologyOptimizationParams {
            volume_fraction: 0.5,
            penalization: 3.0,
            min_density: 0.001,
            move_limit: 0.2,
            filter_radius: 1.5,
            stiffness_min: 1.0e-6,
            stiffness_solid: 1.0,
        }
    }

    #[test]
    fn simp_material_and_sensitivity_work() {
        let mut report = SimpMaterialReport::default();
        assert_eq!(
            topology_simp_material(0.5, params(), &mut report),
            Bool::TRUE
        );
        assert!(report.stiffness > 0.0);
        assert!(report.stiffness_derivative > 0.0);
        let sensitivity = topology_compliance_sensitivity(0.5, 2.0, params());
        assert!(sensitivity < 0.0);
    }

    #[test]
    fn oc_update_preserves_volume_fraction() {
        let densities = [0.5, 0.5, 0.5, 0.5];
        let sensitivities = [-4.0, -1.0, -1.0, -4.0];
        let mut out = [0.0; 4];
        let mut report = TopologyOptimizationReport::default();
        assert_eq!(
            topology_oc_update(
                densities.as_ptr(),
                sensitivities.as_ptr(),
                4,
                params(),
                out.as_mut_ptr(),
                4,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!((report.average_density - 0.5).abs() < 1.0e-3);
        assert!(out[0] > out[1]);
    }

    #[test]
    fn density_filter_and_voxelization_work() {
        let densities = [1.0, 0.0, 0.0, 1.0];
        let mut filtered = [0.0; 4];
        assert_eq!(
            topology_density_filter_2d(densities.as_ptr(), 2, 2, 1.5, filtered.as_mut_ptr(), 4),
            Bool::TRUE
        );
        assert!(filtered[0] < 1.0);
        assert!(filtered[1] > 0.0);

        let mut voxels = [0u8; 4];
        let mut stats = DensityFieldStats::default();
        assert_eq!(
            topology_density_to_voxels(
                filtered.as_ptr(),
                4,
                0.5,
                voxels.as_mut_ptr(),
                4,
                &mut stats,
            ),
            Bool::TRUE
        );
        assert_eq!(stats.cell_count, 4);
    }
}



