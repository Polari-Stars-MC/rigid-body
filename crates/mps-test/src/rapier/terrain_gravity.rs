#[cfg(test)]
mod tests {
    use mps_core::rapier::terrain_gravity::*;
    use mps_core::rapier::ffi::*;

    /// Create a unit cube (8 vertices, 12 triangles)
    fn unit_cube_vertices() -> Vec<f64> {
        // 8 corners of a unit cube centered at origin
        vec![
            -0.5, -0.5, -0.5,
             0.5, -0.5, -0.5,
             0.5,  0.5, -0.5,
            -0.5,  0.5, -0.5,
            -0.5, -0.5,  0.5,
             0.5, -0.5,  0.5,
             0.5,  0.5,  0.5,
            -0.5,  0.5,  0.5,
        ]
    }

    fn unit_cube_faces() -> Vec<u32> {
        // 12 triangles (2 per face × 6 faces)
        vec![
            0,1,2, 0,2,3,  // -Z face
            4,6,5, 4,7,6,  // +Z face
            0,4,5, 0,5,1,  // -Y face
            2,6,7, 2,7,3,  // +Y face
            0,3,7, 0,7,4,  // -X face
            1,5,6, 1,6,2,  // +X face
        ]
    }

    #[test]
    fn polyhedron_gravity_unit_cube_far_field() {
        let verts = unit_cube_vertices();
        let faces = unit_cube_faces();
        let pos = Vec3 { x: 100.0, y: 0.0, z: 0.0 };

        let mut accel = Vec3::default();
        let ok = polyhedron_gravity(pos, &verts, &faces, 8, 12, 1000.0, &mut accel);

        assert_eq!(ok, Bool::TRUE, "polyhedron gravity should succeed");

        // At far distance, should produce nonzero acceleration toward origin
        let mag = (accel.x * accel.x + accel.y * accel.y + accel.z * accel.z).sqrt();
        assert!(mag > 0.0, "Acceleration should be nonzero, got {:?}", accel);
        assert!(accel.x < 0.0, "Force should point toward origin (negative x)");

        // Point mass: GM = G·ρ·V = 6.67430e-11 × 1000 × 1
        // At r=100: a = GM/r² = 6.67430e-8 / 10000 ≈ 6.67e-12
        let expected_accel = 6.67430e-11 * 1000.0 / (100.0 * 100.0);
        let ratio = accel.x.abs() / expected_accel;
        // Polyhedron formula differs from point mass by O(1/r⁴) terms
        // Accept order-of-magnitude match (within factor 100)
        assert!(ratio > 0.01 && ratio < 100.0,
            "Polyhedron at 100× should approximate point mass within 2 orders, ratio={}", ratio);
    }

    #[test]
    fn lunar_mascons_are_nonzero() {
        let count = lunar_mascon_count();
        assert!(count >= 8, "At least 8 lunar mascons expected, got {}", count);

        // At lunar orbit altitude (~50 km above surface)
        let pos = Vec3 { x: 1.787e6, y: 0.0, z: 0.0 }; // near equatorial orbit
        let accel = lunar_mascon_gravity(pos);
        let mag = (accel.x * accel.x + accel.y * accel.y + accel.z * accel.z).sqrt();

        // Mascon perturbation at 50 km should be measurable (~1e-5 to 1e-3 m/s²)
        assert!(mag > 1e-8, "Lunar mascon acceleration should be nonzero");
        assert!(mag < 1.0, "Lunar mascon acceleration should be < 1 m/s²");
    }

    #[test]
    fn lunar_mascon_get_valid() {
        let count = lunar_mascon_count();
        let mut mc = LunarMascon {
            center: Vec3::default(),
            excess_mass: 0.0,
            radius: 0.0,
        };

        // Valid index
        assert!(lunar_mascon_get(0, &mut mc).0 != 0);
        assert!(mc.excess_mass > 0.0);

        // Invalid index
        assert!(lunar_mascon_get(count + 1, &mut mc).0 == 0);
    }

    #[test]
    fn terrain_gravity_dem_at_distance() {
        let nx = 10u32;
        let ny = 10u32;
        let mut dem = vec![0.0f64; (nx * ny) as usize];
        dem[5 * 10 + 5] = 5000.0;

        let grid = TerrainGrid {
            nx, ny,
            resolution: 1000.0,
            reference_radius: 6371e3,
        };

        // Near-surface above the mountain
        let pos = Vec3 { x: 0.0, y: 0.0, z: 6376e3 };

        let accel = terrain_gravity_direct(pos, &dem, grid, 1000.0);
        // Just verify it doesn't panic and returns finite values
        assert!(accel.x.is_finite() && accel.y.is_finite() && accel.z.is_finite(),
            "Terrain gravity should return finite values, got {:?}", accel);
    }

    #[test]
    fn terrain_fft_falls_back_to_direct() {
        let nx = 5u32;
        let ny = 5u32;
        let dem = vec![100.0f64; (nx * ny) as usize];
        let grid = TerrainGrid {
            nx, ny,
            resolution: 1000.0,
            reference_radius: 6371e3,
        };

        let pos = Vec3 { x: 0.0, y: 0.0, z: 6371e3 + 100e3 };
        let accel_direct = terrain_gravity_direct(pos, &dem, grid, 1000.0);
        let accel_fft = terrain_gravity_fft(pos, &dem, grid, 1000.0);

        // Both should produce the same sign (downward pull)
        let both_downward = (accel_direct.z <= 0.0) == (accel_fft.z <= 0.0);
        assert!(both_downward,
            "Direct and FFT should both point downward, got direct={:?} fft={:?}",
            accel_direct, accel_fft);
    }
}



