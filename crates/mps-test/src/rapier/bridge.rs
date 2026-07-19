#[cfg(test)]
mod tests {
    use mps_core::rapier::bridge::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::Vec3;

    #[test]
    fn write_vec3_to_slot_roundtrips() {
        let mut buf = [0.0f64; 3];
        let slot = buf.as_mut_ptr() as i64;
        let v = Vec3 {
            x: 1.5,
            y: -2.5,
            z: 3.5,
        };
        assert!(write_vec3_to_slot(slot, v));
        assert!((buf[0] - 1.5).abs() < 1e-15);
        assert!((buf[1] + 2.5).abs() < 1e-15);
        assert!((buf[2] - 3.5).abs() < 1e-15);
    }

    #[test]
    fn write_vec3_rejects_null() {
        assert!(!write_vec3_to_slot(0, Vec3::default()));
    }

    #[test]
    fn direct_buffer_slice_null_returns_none() {
        assert!(direct_double_buffer_as_slice(0, 10).is_none());
        assert!(direct_byte_buffer_as_slice(0, 10).is_none());
    }

    #[test]
    fn bulk_snapshot_rejects_null_world() {
        let mut buf = [0.0f64; 13];
        let count = bulk_body_snapshot_to_direct_buffer(
            std::ptr::null(),
            buf.as_mut_ptr() as i64,
            1,
        );
        assert_eq!(count, 0);
    }

    #[test]
    fn bulk_snapshot_works_with_valid_world() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let b = mps_core::rapier::rigid_body::rigid_body_builder_create(
            mps_core::rapier::ffi::BodyStatus::Dynamic as u32,
        );
        mps_core::rapier::rigid_body::rigid_body_builder_set_translation(
            b,
            Vec3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
        );
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(b);
        mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);

        let mut buf = vec![0.0f64; 13 * 10];
        let count =
            bulk_body_snapshot_to_direct_buffer(world, buf.as_mut_ptr() as i64, 10);
        assert_eq!(count, 1);
        assert!((buf[0] - 1.0).abs() < 1e-12);
        assert!((buf[1] - 2.0).abs() < 1e-12);
        assert!((buf[2] - 3.0).abs() < 1e-12);

        mps_core::rapier::world::world_destroy(world);
    }
}



