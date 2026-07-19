#[cfg(test)]
mod tests {
    use mps_core::rapier::world::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::BodyStatus;

    #[test]
    fn integration_parameters_and_body_batch_updates_work() {
        let world = world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        assert!(!world.is_null());
        assert_eq!(
            world_set_integration_parameters(world, 1.0 / 120.0, 8, 2),
            Bool::TRUE
        );

        let mut params = [0.0; 3];
        assert_eq!(
            world_get_integration_parameters(world, params.as_mut_ptr(), params.len() as u32),
            3
        );
        assert_eq!(params[1], 8.0);
        assert_eq!(params[2], 2.0);

        let builder =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(builder);
        let handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);
        assert_ne!(handle, 0);
        assert_eq!(world_body_snapshot_count(world), 1);

        let handles = [handle];
        let poses = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0];
        assert_eq!(
            world_update_body_poses(world, handles.as_ptr(), poses.as_ptr(), 1, Bool::TRUE),
            1
        );
        let velocities = [4.0, 5.0, 6.0, 0.1, 0.2, 0.3];
        assert_eq!(
            world_update_body_velocities(
                world,
                handles.as_ptr(),
                velocities.as_ptr(),
                1,
                Bool::TRUE
            ),
            1
        );

        let mut out_handles = [0; 1];
        let mut values = [0.0; 13];
        assert_eq!(
            world_body_snapshot(
                world,
                out_handles.as_mut_ptr(),
                values.as_mut_ptr(),
                out_handles.len() as u32,
            ),
            1
        );
        assert_eq!(out_handles[0], handle);
        assert_eq!(&values[..3], &[1.0, 2.0, 3.0]);
        assert_eq!(&values[7..10], &[4.0, 5.0, 6.0]);

        world_destroy(world);
    }
}



