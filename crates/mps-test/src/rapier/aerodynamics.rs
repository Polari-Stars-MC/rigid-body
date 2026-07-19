#[cfg(test)]
mod tests {
    use mps_core::rapier::aerodynamics::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::BodyStatus;

    #[test]
    fn estimates_drag_force_from_exposed_surface() {
        let surface = AeroSurface {
            point: Vec3::default(),
            normal: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            area: 2.0,
            drag_coefficient: 1.0,
            lift_coefficient: 0.0,
        };
        let mut report = AeroForceReport::default();

        assert_eq!(
            aero_estimate_surface_force(
                Vec3::default(),
                Vec3::default(),
                Vec3::default(),
                Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
                1.2,
                surface,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(report.active_surface_count, 1);
        assert!((report.total_force.x - 120.0).abs() < 1.0e-9);
        assert_eq!(report.total_force.y, 0.0);
        assert_eq!(report.total_force.z, 0.0);
    }

    #[test]
    fn applies_aerodynamic_force_to_rapier_body() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let builder =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder, 1.0);
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(builder);
        let body_handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);
        assert_ne!(body_handle, 0);

        let surfaces = [AeroSurface {
            point: Vec3::default(),
            normal: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            area: 1.0,
            drag_coefficient: 1.0,
            lift_coefficient: 0.0,
        }];
        let mut report = AeroForceReport::default();
        assert_eq!(
            aero_apply_surfaces(
                world,
                body_handle,
                Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
                1.0,
                surfaces.as_ptr(),
                surfaces.len() as u32,
                Bool::TRUE,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(report.active_surface_count, 1);
        assert!(report.total_force.x > 0.0);

        mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        let velocity = mps_core::rapier::rigid_body::rigid_body_get_linvel(world, body_handle);
        assert!(velocity.x > 0.0);

        mps_core::rapier::world::world_destroy(world);
    }
}



