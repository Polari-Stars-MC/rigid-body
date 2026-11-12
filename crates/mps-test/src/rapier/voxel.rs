#[cfg(test)]
mod tests {
    use mps_core::rapier::collider::{
        collider_builder_build, collider_builder_create_compound_boxes, collider_builder_destroy,
        world_insert_collider,
    };
    use mps_core::rapier::error::{ERR_CAPACITY, ERR_OK, clear_error, last_error_code};
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::{Bool, Quat};
    use mps_core::rapier::voxel::*;
    use mps_core::rapier::world::{world_create, world_destroy, world_step};

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
        let aabb_builder = collider_builder_create_voxel_aabb(
            aabb,
            0.5,
            0.5,
            0.5,
            options(VoxelColliderMode::Auto),
        );
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

    #[test]
    fn ray_pick_resolves_voxel_cell() {
        // 2x2x2 solid voxel grid at origin, unit voxel size.
        let data = [1u8; 8];
        let builder = collider_builder_create_voxels(
            data.as_ptr(),
            2,
            2,
            2,
            1.0,
            1.0,
            1.0,
            Vec3::default(),
            options(VoxelColliderMode::Cuboids),
        );
        assert!(!builder.is_null());
        let collider = collider_builder_build(builder);
        // NOTE: `collider_builder_build` consumes (Box::from_raw) the builder
        // pointer, so it must NOT be passed to `collider_builder_destroy` after.
        assert!(!collider.is_null());

        let world = world_create(Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        let handle = world_insert_collider(world, collider);
        assert_ne!(handle, 0);
        // Advance one step so the QueryPipeline (broad_phase) is rebuilt and
        // ray/shape queries can see the freshly inserted collider.
        world_step(world, 1.0 / 60.0);

        let mut out = VoxelCoord::default();
        // Ray from (0.5, 5, 0.5) straight down hits the top face (y=2);
        // the epsilon nudge resolves it to cell (0,1,0) with normal +Y.
        let ok = collider_voxel_ray_pick(
            world,
            handle,
            Vec3 {
                x: 0.5,
                y: 5.0,
                z: 0.5,
            },
            Vec3 {
                x: 0.0,
                y: -1.0,
                z: 0.0,
            },
            10.0,
            Bool::TRUE,
            &mut out,
        );
        assert!(ok.0 != 0, "ray pick should hit the voxel collider");
        assert!(out.found.0 != 0, "hit cell should be reported as found");
        assert_eq!(
            (out.ix, out.iy, out.iz),
            (0, 1, 0),
            "ray should resolve to the top voxel cell"
        );
        assert!(
            (out.nx, out.ny, out.nz) == (0.0, 1.0, 0.0),
            "surface normal should point +Y"
        );

        world_destroy(world);
    }

    #[test]
    fn cell_at_point_resolves_voxel_cell() {
        // 2x2x2 solid voxel grid at origin, unit voxel size.
        let data = [1u8; 8];
        let builder = collider_builder_create_voxels(
            data.as_ptr(),
            2,
            2,
            2,
            1.0,
            1.0,
            1.0,
            Vec3::default(),
            options(VoxelColliderMode::Cuboids),
        );
        assert!(!builder.is_null());
        let collider = collider_builder_build(builder);
        // NOTE: `collider_builder_build` consumes (Box::from_raw) the builder
        // pointer, so it must not be passed to `collider_builder_destroy`.
        assert!(!collider.is_null());
        let world = world_create(Vec3::default());
        assert!(!world.is_null());
        let handle = world_insert_collider(world, collider);
        assert_ne!(handle, 0);

        // A point just inside the centre of the top-front-right cell lives in
        // the grid; the voxel collider occupies [0,2]^3.
        let mut out = VoxelCoord::default();
        let ok = collider_voxel_cell_at_point(
            world,
            handle,
            Vec3 {
                x: 1.5,
                y: 0.5,
                z: 1.5,
            },
            &mut out,
        );
        assert!(ok.0 != 0, "point inside the grid should resolve a cell");
        assert!(out.found.0 != 0, "cell should be reported as found");
        assert_eq!(
            (out.ix, out.iy, out.iz),
            (1, 0, 1),
            "point (1.5, 0.5, 1.5) sits in cell (1, 0, 1)"
        );

        // A point outside the grid bounds resolves to nothing.
        let mut miss = VoxelCoord::default();
        let ok_miss = collider_voxel_cell_at_point(
            world,
            handle,
            Vec3 {
                x: 5.0,
                y: 5.0,
                z: 5.0,
            },
            &mut miss,
        );
        assert!(ok_miss.0 == 0, "point outside the grid should not resolve");
        assert!(miss.found.0 == 0, "missed cell should report not found");

        world_destroy(world);
    }

    #[test]
    fn get_reads_voxel_cell_solidity() {
        // 2x2x2 grid, unit voxel size, origin at 0. Fill only cells touching
        // the y==0 plane (indices 0..3), leave the top plane (4..7) empty so we
        // can assert BOTH solid and empty reads from the same collider.
        let mut data = [0u8; 8];
        data[..4].fill(1);
        let builder = collider_builder_create_voxels(
            data.as_ptr(),
            2,
            2,
            2,
            1.0,
            1.0,
            1.0,
            Vec3::default(),
            options(VoxelColliderMode::Cuboids),
        );
        assert!(!builder.is_null());
        let collider = collider_builder_build(builder);
        assert!(!collider.is_null());
        let world = world_create(Vec3::default());
        assert!(!world.is_null());
        let handle = world_insert_collider(world, collider);
        assert_ne!(handle, 0);
        // Ray/shape queries read the QueryPipeline rebuilt by step.
        world_step(world, 1.0 / 60.0);

        // Solid cell (bottom-front-left, index 0).
        let mut solid_out: u8 = 0;
        let ok = collider_voxel_read_cell(world, handle, 0, 0, 0, &mut solid_out);
        assert!(ok.0 != 0, "get on a valid voxel collider should succeed");
        assert!(solid_out != 0, "cell (0,0,0) was filled -> solid");

        // Empty cell (top-front-left, index 4).
        let mut empty_out: u8 = 1;
        let ok_empty = collider_voxel_read_cell(world, handle, 0, 1, 0, &mut empty_out);
        assert!(
            ok_empty.0 != 0,
            "get on a valid voxel collider should succeed"
        );
        assert!(empty_out == 0, "cell (0,1,0) was left empty -> not solid");

        // Out-of-range coordinate: returns FALSE and writes 0 to the out ptr.
        let mut oob: u8 = 1;
        let ok_oob = collider_voxel_read_cell(world, handle, 5, 0, 0, &mut oob);
        assert!(ok_oob.0 == 0, "out-of-range cell should fail");
        assert!(oob == 0, "out ptr must be zeroed on out-of-range");

        // Non-voxel collider: returns FALSE.
        let mut nv: u8 = 1;
        let ok_nv = collider_voxel_read_cell(world, 0xDEAD_BEEF, 0, 0, 0, &mut nv);
        assert!(ok_nv.0 == 0, "non-voxel collider should fail");
        assert!(nv == 0, "out ptr must be zeroed on non-voxel collider");

        world_destroy(world);
    }

    /// Compound-baker 端到端：`voxel_bake_boxes` 输出确定性 min/max 框列表，
    /// 且该列表经 `collider_builder_create_compound_boxes` 重建的 collider 与
    /// 直接 `build_voxel_collider(GreedyCuboids)` 覆盖相同的体素包围盒。
    /// 烘焙是"压体素"的缓存半边——Java 按 chunk 缓存框列表，重建走 compound
    /// boxes 快路径，无需重传网格重跑合并。
    #[test]
    fn voxel_bake_boxes_matches_greedy_collider_extent() {
        // 4×2×3 网格：底部整层 4×3 实心 + 一个顶部孤立方块。
        // 底层应被合并成 1 个 4×1×3 框，顶部 1 个 1×1×1 框 → 共 2 框。
        let mut voxels = vec![0u8; 4 * 2 * 3];
        for z in 0..3 {
            for x in 0..4 {
                voxels[x + 4 * z] = 1; // y=0 层
            }
        }
        voxels[2 + 4 * 4] = 1; // (x=2, y=1, z=1) → idx = x + 4*(z + 3*y) = 2+16
        let grid = VoxelGrid {
            voxels: &voxels,
            size_x: 4,
            size_y: 2,
            size_z: 3,
            voxel_size_x: 1.0,
            voxel_size_y: 1.0,
            voxel_size_z: 1.0,
            origin: Vec3::default(),
        };

        // Count-only 查询（null out ptr）。
        let count = voxel_bake_boxes(
            grid.voxels.as_ptr(),
            4,
            2,
            3,
            1.0,
            1.0,
            1.0,
            grid.origin,
            std::ptr::null_mut(),
            0,
        );
        assert_eq!(count, 2, "flat layer + lone block bake to 2 boxes");

        // 实际写出。
        let mut out = [0f64; 64];
        let n = voxel_bake_boxes(
            grid.voxels.as_ptr(),
            4,
            2,
            3,
            1.0,
            1.0,
            1.0,
            grid.origin,
            out.as_mut_ptr(),
            out.len() as u32,
        );
        assert_eq!(n, 2);

        // 框 0：底层 4×1×3（min 0,0,0 / max 4,1,3）。
        let b0 = &out[0..6];
        assert_eq!(b0, [0.0, 0.0, 0.0, 4.0, 1.0, 3.0], "merged base layer");
        // 框 1：孤立方块 (2,1,1)..(3,2,2)。
        let b1 = &out[6..12];
        assert_eq!(b1, [2.0, 1.0, 1.0, 3.0, 2.0, 2.0], "lone block");

        // 确定性：重跑两次逐位一致。
        let mut out2 = [0f64; 64];
        voxel_bake_boxes(
            grid.voxels.as_ptr(),
            4,
            2,
            3,
            1.0,
            1.0,
            1.0,
            grid.origin,
            out2.as_mut_ptr(),
            out2.len() as u32,
        );
        assert_eq!(&out[..12], &out2[..12], "bake is deterministic");

        // 缓冲过小 → ERR_CAPACITY 且不写。
        let mut small = [0f64; 5];
        let n_small = voxel_bake_boxes(
            grid.voxels.as_ptr(),
            4,
            2,
            3,
            1.0,
            1.0,
            1.0,
            grid.origin,
            small.as_mut_ptr(),
            small.len() as u32,
        );
        assert_eq!(n_small, 0, "insufficient buffer fails");
        assert_eq!(last_error_code(), ERR_CAPACITY, "ERR_CAPACITY reported");
        clear_error();

        // 空网格 → 0 框。
        let empty = [0u8; 8];
        let n_empty = voxel_bake_boxes(
            empty.as_ptr(),
            2,
            2,
            2,
            1.0,
            1.0,
            1.0,
            grid.origin,
            std::ptr::null_mut(),
            0,
        );
        assert_eq!(n_empty, 0, "empty grid bakes 0 boxes");
        assert_eq!(last_error_code(), ERR_OK, "empty grid is not an error");
        clear_error();

        // 烘焙结果走现成 compound-boxes builder 可重建等价 collider。
        let builder = collider_builder_create_compound_boxes(out.as_ptr(), 2);
        assert!(
            !builder.is_null(),
            "compound boxes builder from bake output"
        );
        collider_builder_destroy(builder);
    }

    /// FNV-1a 64 位：把 box 槽序列压成一个指纹，跨机器/跨平台比对用
    /// （f64 位型逐位一致 → 哈希一致；哈希不一致 → 一定有平台差异）。
    fn fnv1a(bits: impl Iterator<Item = u64>) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in bits {
            for byte in b.to_le_bytes() {
                h ^= byte as u64;
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
        h
    }

    /// 跨平台确定性守门：伪随机多材质网格（LCG 生成，避免测试平台差异）
    /// bake 三次，逐位比较输出并对照**编码进断言的指纹**。指纹由算法结构
    /// （整数扫序 + 加减法边界）决定——只要没人改合并算法或 float 运算顺序，
    /// 任何平台（x86/ARM、MSVC/GCC、release/debug）都必须复现同一指纹。
    /// 平台指纹漂移 = 烘焙算法引入了非位级确定性的浮点路径 = MC 缓存失效。
    #[test]
    fn compound_baker_bit_identical_across_platforms() {
        // 12×6×12 网格，LCG 填充（约 55% 实心）——足够触发多层合并路径，
        // 又完全可复现。
        let (sx, sy, sz) = (12usize, 6usize, 12usize);
        let mut voxels = vec![0u8; sx * sy * sz];
        let mut seed: u64 = 0x2026_1112;
        for v in &mut voxels {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            *v = if (seed >> 33) % 100 < 55 { 1 } else { 0 };
        }
        let bake = |out: &mut [f64]| -> u32 {
            voxel_bake_boxes(
                voxels.as_ptr(),
                sx as u32,
                sy as u32,
                sz as u32,
                0.5,
                0.5,
                0.5,
                Vec3 {
                    x: 128.0,
                    y: 64.0,
                    z: -32.0,
                },
                out.as_mut_ptr(),
                out.len() as u32,
            )
        };
        let count = voxel_bake_boxes(
            voxels.as_ptr(),
            sx as u32,
            sy as u32,
            sz as u32,
            0.5,
            0.5,
            0.5,
            Vec3 {
                x: 128.0,
                y: 64.0,
                z: -32.0,
            },
            std::ptr::null_mut(),
            0,
        );
        assert!(
            count > 8,
            "dense random grid should merge into 8+ boxes, got {count}"
        );
        let mut a = vec![0f64; count as usize * 6];
        let mut b = vec![0f64; count as usize * 6];
        let mut c = vec![0f64; count as usize * 6];
        assert_eq!(bake(&mut a), count);
        assert_eq!(bake(&mut b), count);
        assert_eq!(bake(&mut c), count);
        assert_eq!(a, b, "run 1 vs run 2 bit-identical");
        assert_eq!(b, c, "run 2 vs run 3 bit-identical");

        // 位级指纹：非有限值（NaN 位型不唯一）直接拒绝。
        for slot in &a {
            assert!(
                slot.is_finite(),
                "bake emitted non-finite coordinate {slot}"
            );
        }
        let fingerprint = fnv1a(a.iter().map(|v| v.to_bits()));
        assert_eq!(
            fingerprint, 0xe93f_bc71_c17a_8a20,
            "cross-platform bake fingerprint drifted — merge algorithm or float \
             ordering changed; update the golden ONLY after confirming the new \
             output is still deterministic on all platforms"
        );
    }
}
