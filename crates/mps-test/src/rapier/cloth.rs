#[cfg(test)]
mod tests {
    use mps_core::rapier::cloth::{CLOTH_MAX_PARTICLES, ClothDesc, soft_cloth_create};
    use mps_core::rapier::error::{
        ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ERR_OK, last_error_code,
    };
    use mps_core::rapier::ffi::{Bool, Vec3, WorldHandle};
    use mps_core::rapier::soft_body::{
        soft_body_apply_wind, soft_body_particle_count, soft_body_read_particles,
        soft_body_set_tear_strain, soft_body_tear_now,
    };
    use mps_core::rapier::world::{world_create, world_destroy, world_step};

    const SENTINEL: u32 = u32::MAX;

    fn make_world() -> *mut WorldHandle {
        world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        })
    }

    fn desc(cols: u32, rows: u32, pin_mode: u32) -> ClothDesc {
        ClothDesc {
            cols,
            rows,
            spacing: 0.25,
            origin: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            u_axis: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            v_axis: Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            particle_mass: 0.1,
            stiffness: 80.0,
            damping: 1.0,
            shear_ratio: 0.55,
            bend_ratio: 0.1,
            pin_mode,
        }
    }

    /// Mean of one position channel over a grid column, via the public
    /// `soft_body_read_particles` FFI.
    fn column_mean_x(world: *const WorldHandle, id: u32, cols: u32, col: u32) -> f64 {
        let n = soft_body_particle_count(world, id);
        let mut pos = vec![Vec3::default(); n as usize];
        let mut inv = vec![0.0f64; n as usize];
        let read = soft_body_read_particles(world, id, pos.as_mut_ptr(), inv.as_mut_ptr(), n);
        assert_eq!(read, n, "read_particles should fill the whole buffer");
        let rows = (n / cols) as usize;
        (0..rows)
            .map(|r| pos[r * cols as usize + col as usize].x)
            .sum::<f64>()
            / rows as f64
    }

    fn spring_count(world: *mut WorldHandle, id: u32) -> usize {
        unsafe {
            (*world)
                .inner
                .soft_bodies
                .get(rapier3d::prelude::soft_body_fork::SoftBodyId(id))
                .expect("cloth present")
                .springs
                .len()
        }
    }

    #[test]
    fn cloth_create_builds_grid_topology() {
        let world = make_world();
        let id = soft_cloth_create(world, desc(4, 3, 0));
        assert_ne!(id, SENTINEL, "create should succeed");
        assert_eq!(last_error_code(), ERR_OK);

        // Particle count + row-major grid positions.
        assert_eq!(soft_body_particle_count(world, id), 12);
        let mut pos = vec![Vec3::default(); 12];
        let mut inv = vec![0.0f64; 12];
        let read = soft_body_read_particles(world, id, pos.as_mut_ptr(), inv.as_mut_ptr(), 12);
        assert_eq!(read, 12);
        for row in 0..3usize {
            for col in 0..4usize {
                let p = pos[row * 4 + col];
                assert!(
                    (p.x - col as f64 * 0.25).abs() < 1e-9,
                    "col {col}: x {}",
                    p.x
                );
                assert!(
                    (p.y - row as f64 * 0.25).abs() < 1e-9,
                    "row {row}: y {}",
                    p.y
                );
                assert!((p.z).abs() < 1e-9);
                assert!((inv[row * 4 + col] - 1.0 / 0.1).abs() < 1e-9);
            }
        }

        // Spring families: 17 structural + 12 shear + 10 bend = 39.
        assert_eq!(spring_count(world, id), 39);
        world_destroy(world);
    }

    #[test]
    fn cloth_zero_ratios_disable_shear_and_bend() {
        let world = make_world();
        let mut d = desc(3, 3, 0);
        d.shear_ratio = 0.0;
        d.bend_ratio = 0.0;
        let id = soft_cloth_create(world, d);
        assert_ne!(id, SENTINEL);
        // Structural only: (3-1)*3 horizontal + 3*(3-1) vertical = 12.
        assert_eq!(spring_count(world, id), 12);
        world_destroy(world);
    }

    #[test]
    fn cloth_pinned_flag_hangs_under_gravity() {
        let world = make_world();
        // 5×4 flag: u = +X columns, v = +Y rows, left edge (col 0) pinned.
        let id = soft_cloth_create(world, desc(5, 4, 2));
        assert_ne!(id, SENTINEL);

        // Pinned column must be frozen: inv_mass == 0.
        let mut pos = vec![Vec3::default(); 20];
        let mut inv = vec![0.0f64; 20];
        soft_body_read_particles(world, id, pos.as_mut_ptr(), inv.as_mut_ptr(), 20);
        for row in 0..4usize {
            assert_eq!(inv[row * 5], 0.0, "col-0 particle must be pinned");
            assert!(inv[row * 5 + 1] > 0.0, "col-1 particle must be free");
        }
        let pinned_before = pos[0];

        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
        }

        let mut pos_after = vec![Vec3::default(); 20];
        let mut inv_after = vec![0.0f64; 20];
        let n = soft_body_read_particles(
            world,
            id,
            pos_after.as_mut_ptr(),
            inv_after.as_mut_ptr(),
            20,
        );
        assert_eq!(n, 20);

        // Pinned edge exactly where it was.
        let pa = pos_after[0];
        assert!((pa.x - pinned_before.x).abs() < 1e-9);
        assert!((pa.y - pinned_before.y).abs() < 1e-9);

        // Free far edge (col 4) must sag under gravity yet stay finite/bounded.
        for p in &pos_after {
            assert!(p.x.is_finite() && p.y.is_finite() && p.z.is_finite());
            assert!(p.y > -20.0 && p.y < 20.0);
        }
        let mean = |col: usize| (0..4).map(|r| pos_after[r * 5 + col].y).sum::<f64>() / 4.0;
        assert!(
            mean(4) < mean(0) - 0.03,
            "far edge should sag: col4 mean y {} vs pinned {}",
            mean(4),
            mean(0)
        );
        world_destroy(world);
    }

    #[test]
    fn cloth_wind_deflects_free_edge_monotonically() {
        let world = make_world();
        let id = soft_cloth_create(world, desc(5, 4, 2)); // pin col 0 (x = 0)
        assert_ne!(id, SENTINEL);

        let ok = soft_body_apply_wind(
            world,
            id,
            Vec3 {
                x: 15.0,
                y: 0.0,
                z: 0.0,
            },
            0.2,
        );
        assert_eq!(ok, Bool::TRUE);

        for _ in 0..150 {
            world_step(world, 1.0 / 60.0);
        }

        // Downstream columns deflect further than upstream ones; the pinned
        // edge stays put.
        let c0 = column_mean_x(world, id, 5, 0);
        let c1 = column_mean_x(world, id, 5, 1);
        let c4 = column_mean_x(world, id, 5, 4);
        assert!(c0.abs() < 1e-6, "pinned edge must not move");
        assert!(c1 > 0.0, "wind should push free cloth downstream: c1 {c1}");
        assert!(
            c4 > c1,
            "far edge should deflect more than near edge: c4 {c4} vs c1 {c1}"
        );
        world_destroy(world);
    }

    #[test]
    fn cloth_tears_overstretched_springs() {
        let world = make_world();
        let id = soft_cloth_create(world, desc(4, 4, 2));
        assert_ne!(id, SENTINEL);
        let before = spring_count(world, id);
        assert!(before > 0);

        // A 1% strain threshold plus a violent gust must sever springs.
        assert_eq!(soft_body_set_tear_strain(world, id, 0.01, 1), Bool::TRUE);
        assert_eq!(
            soft_body_apply_wind(
                world,
                id,
                Vec3 {
                    x: 400.0,
                    y: 0.0,
                    z: 0.0
                },
                0.0
            ),
            Bool::TRUE
        );
        for _ in 0..30 {
            world_step(world, 1.0 / 60.0);
        }
        assert_eq!(soft_body_tear_now(world, id), Bool::TRUE);
        let after = spring_count(world, id);
        assert!(
            after < before,
            "tearing should remove springs: before {before}, after {after}"
        );
        world_destroy(world);
    }

    #[test]
    fn cloth_create_rejects_bad_params() {
        // Null world.
        assert_eq!(
            soft_cloth_create(std::ptr::null_mut(), desc(4, 4, 0)),
            SENTINEL
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        let world = make_world();
        let cases: [(fn(&mut ClothDesc), &str); 8] = [
            (|d| d.cols = 1, "cols < 2"),
            (|d| d.rows = 1, "rows < 2"),
            (|d| d.spacing = 0.0, "zero spacing"),
            (|d| d.particle_mass = 0.0, "zero mass"),
            (|d| d.stiffness = -1.0, "negative stiffness"),
            (|d| d.shear_ratio = 1.5, "shear ratio out of range"),
            (|d| d.pin_mode = 7, "unknown pin mode"),
            (
                |d| {
                    d.v_axis = Vec3 {
                        x: 1.0,
                        y: 0.0,
                        z: 0.0,
                    }
                },
                "parallel axes",
            ),
        ];
        for (mutate, label) in cases {
            let mut d = desc(4, 4, 0);
            mutate(&mut d);
            assert_eq!(soft_cloth_create(world, d), SENTINEL, "case: {label}");
            assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT, "case: {label}");
        }

        // Oversized grid → ERR_CAPACITY.
        let d = desc(CLOTH_MAX_PARTICLES / 2 + 2, 2, 0);
        assert_eq!(soft_cloth_create(world, d), SENTINEL);
        assert_eq!(last_error_code(), ERR_CAPACITY);

        world_destroy(world);
    }

    #[test]
    fn cloth_free_falls_and_stays_finite() {
        let world = make_world();
        let id = soft_cloth_create(world, desc(3, 3, 0)); // Free
        assert_ne!(id, SENTINEL);
        for _ in 0..90 {
            world_step(world, 1.0 / 60.0);
        }
        let mut pos = vec![Vec3::default(); 9];
        let mut inv = vec![0.0f64; 9];
        soft_body_read_particles(world, id, pos.as_mut_ptr(), inv.as_mut_ptr(), 9);
        let mut min_y = f64::INFINITY;
        for p in &pos {
            assert!(p.x.is_finite() && p.y.is_finite() && p.z.is_finite());
            assert!(p.y.abs() < 50.0, "bounded fall, got y {}", p.y);
            min_y = min_y.min(p.y);
        }
        assert!(min_y < 0.0, "free cloth must fall below its start height");
        world_destroy(world);
    }
}
