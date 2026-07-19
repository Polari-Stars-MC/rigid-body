#[cfg(test)]
mod tests {
    use mps_core::rapier::voxel::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::{Bool, Quat};

    fn options(mode: VoxelColliderMode) -> VoxelColliderOptions {
        VoxelColliderOptions {
            mode: mode as u32,
            dynamic_body: Bool::FALSE,
            small_voxel_limit: 128,
            mesh_voxel_limit: 20_000,
        }
    }

    #[test]
    fn empty_voxels_build_no_collider() {
        let grid = VoxelGrid {
            voxels: &[0; 8],
            size_x: 2,
            size_y: 2,
            size_z: 2,
            voxel_size_x: 1.0,
            voxel_size_y: 1.0,
            voxel_size_z: 1.0,
            origin: Vec3::default(),
        };

        assert!(build_voxel_collider(&grid, options(VoxelColliderMode::Auto)).is_none());
    }

    #[test]
    fn solid_voxels_build_with_each_mode() {
        let voxels = [1; 8];
        let grid = VoxelGrid {
            voxels: &voxels,
            size_x: 2,
            size_y: 2,
            size_z: 2,
            voxel_size_x: 1.0,
            voxel_size_y: 1.0,
            voxel_size_z: 1.0,
            origin: Vec3::default(),
        };

        assert!(build_voxel_collider(&grid, options(VoxelColliderMode::Cuboids)).is_some());
        assert!(build_voxel_collider(&grid, options(VoxelColliderMode::GreedyCuboids)).is_some());
        assert!(build_voxel_collider(&grid, options(VoxelColliderMode::SurfaceMesh)).is_some());
    }

    #[test]
    fn voxel_aabb_and_obb_build() {
        let aabb = AabbDesc {
            mins: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            maxs: Vec3 {
                x: 2.0,
                y: 1.0,
                z: 1.0,
            },
        };
        let aabb_builder =
            collider_builder_create_voxel_aabb(aabb, 0.5, 0.5, 0.5, options(VoxelColliderMode::Auto));
        assert!(!aabb_builder.is_null());
        mps_core::rapier::collider::collider_builder_destroy(aabb_builder);

        let obb = Obb {
            center: Vec3::default(),
            half_extents: Vec3 {
                x: 1.0,
                y: 0.5,
                z: 0.5,
            },
            rotation: Quat {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
        };
        let obb_builder =
            collider_builder_create_voxel_obb(obb, 0.5, 0.5, 0.5, options(VoxelColliderMode::Auto));
        assert!(!obb_builder.is_null());
        mps_core::rapier::collider::collider_builder_destroy(obb_builder);
    }
}



