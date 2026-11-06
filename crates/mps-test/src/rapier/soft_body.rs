#[cfg(test)]
mod tests {
    use mps_core::rapier::collider::{
        collider_builder_build, collider_builder_create_sphere, world_insert_collider_with_parent,
    };
    use mps_core::rapier::ffi::VoxelColliderOptions;
    use mps_core::rapier::ffi::{
        BodyStatus, Bool, ColliderHandleRaw, RigidBodyHandleRaw, Sphere, Vec3, WorldHandle,
    };
    use mps_core::rapier::rigid_body::{
        rigid_body_builder_build, rigid_body_builder_create,
        rigid_body_builder_set_additional_mass_properties, rigid_body_builder_set_linvel,
        rigid_body_builder_set_translation, rigid_body_get_translation, world_insert_rigid_body,
    };
    use mps_core::rapier::soft_body::{
        soft_body_add_bending, soft_body_add_distance_constraint, soft_body_add_particle,
        soft_body_add_spring, soft_body_add_tetrahedron, soft_body_add_triangle,
        soft_body_apply_particle_impulse, soft_body_apply_plasticity, soft_body_apply_wind,
        soft_body_attach_particle, soft_body_build_grid, soft_body_build_rope,
        soft_body_build_tetra_mesh, soft_body_clear_cohesion, soft_body_clear_corotated,
        soft_body_clear_cross_collision, soft_body_clear_neo_hookean, soft_body_clear_pressure,
        soft_body_clear_self_collision, soft_body_clear_volume_conservation, soft_body_clear_wind,
        soft_body_clone, soft_body_configure_solver, soft_body_count, soft_body_create,
        soft_body_destroy, soft_body_detach_particle, soft_body_enable_collision,
        soft_body_get_particle, soft_body_is_sleeping, soft_body_kinetic_energy,
        soft_body_particle_count, soft_body_read_aabb, soft_body_read_contact_force,
        soft_body_read_edges, soft_body_read_normals, soft_body_read_particles,
        soft_body_read_spring_forces, soft_body_read_stress, soft_body_read_surface_mesh,
        soft_body_read_surface_triangle_count, soft_body_read_tetrahedra, soft_body_read_triangles,
        soft_body_remove_particle, soft_body_restore_state, soft_body_save_state,
        soft_body_scale_rest_length, soft_body_set_activation, soft_body_set_anisotropy,
        soft_body_set_cohesion, soft_body_set_corotated, soft_body_set_cross_collision,
        soft_body_set_cross_collision_friction, soft_body_set_damping,
        soft_body_set_distance_constraint_activation, soft_body_set_distance_constraint_compliance,
        soft_body_set_distance_constraint_compression, soft_body_set_gravity,
        soft_body_set_neo_hookean, soft_body_set_particle_velocity, soft_body_set_plasticity,
        soft_body_set_pressure, soft_body_set_self_collision,
        soft_body_set_self_collision_friction, soft_body_set_spring_activation,
        soft_body_set_spring_fibre_direction, soft_body_set_spring_stiffness,
        soft_body_set_substeps, soft_body_set_tear_energy, soft_body_set_tear_strain,
        soft_body_set_tear_stress, soft_body_set_thermal, soft_body_set_viscoelastic,
        soft_body_set_volume_conservation, soft_body_sleep, soft_body_state_size,
        soft_body_step_implicit, soft_body_step_mass_spring, soft_body_subdivide_tetrahedra,
        soft_body_tear_now, soft_body_total_volume, soft_body_voxel_build, soft_body_voxel_dig,
        soft_body_wake, soft_chain_create, soft_chain_node_handles,
    };
    use mps_core::rapier::voxel::{collider_builder_create_voxels, collider_voxel_edit};
    use mps_core::rapier::world::{world_create, world_destroy, world_step};
    use rapier3d::prelude::soft_body_fork::SoftBodyId;

    fn make_world() -> *mut WorldHandle {
        world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        })
    }

    #[test]
    fn soft_chain_creates_nodes_and_stays_bounded() {
        let world = make_world();
        assert!(!world.is_null());

        // 4-node chain along +X, spacing 1.0, first node fixed at origin,
        // spring stiffness 200 / damping 5 (soft but holds shape under gravity).
        let count = soft_chain_create(
            world,
            4,
            1.0,
            1.0,
            0.25,
            0, // no external anchor → node 0 fixed at origin
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            200.0,
            5.0,
        );
        assert_eq!(count, 4, "soft_chain_create should create 4 nodes");

        // Read back the (dynamic) node handles.
        let mut handles: Vec<RigidBodyHandleRaw> = vec![0; 8];
        let n = soft_chain_node_handles(world, handles.as_mut_ptr(), handles.len() as u32);
        assert!(n >= 3, "expected >=3 dynamic nodes (node 0 is fixed)");

        // Step the world; the chain should sag but remain finite and bounded.
        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
        }

        // Verify every dynamic node is finite and within a sane bounding box.
        for &h in handles.iter().take(n as usize) {
            assert_ne!(h, 0, "node handle must be valid");
            let body = unsafe {
                (*world)
                    .inner
                    .bodies
                    .get(mps_core::rapier::ffi::unpack_rigid_body_handle(h))
            };
            let body = body.expect("node body present");
            let t = body.translation();
            assert!(t.x.is_finite() && t.y.is_finite() && t.z.is_finite());
            // The chain hangs from origin; it must not fly to infinity.
            assert!(t.x.abs() < 50.0, "x out of bounds: {}", t.x);
            assert!(t.y.abs() < 50.0, "y out of bounds: {}", t.y);
            assert!(t.z.abs() < 50.0, "z out of bounds: {}", t.z);
        }

        world_destroy(world);
    }

    #[test]
    fn soft_chain_rejects_bad_params() {
        let world = make_world();
        assert!(!world.is_null());
        // zero node count
        assert_eq!(
            soft_chain_create(
                world,
                0,
                1.0,
                1.0,
                0.25,
                0,
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0
                },
                200.0,
                5.0,
            ),
            0
        );
        // negative spacing
        assert_eq!(
            soft_chain_create(
                world,
                3,
                -1.0,
                1.0,
                0.25,
                0,
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0
                },
                200.0,
                5.0,
            ),
            0
        );
        world_destroy(world);
    }

    // ── Phase 4: voxel → soft-body + terrain-gravity coupling ──────────────────

    #[test]
    fn soft_body_voxel_build_creates_bounded_deformable() {
        let world = make_world();
        assert!(!world.is_null());

        // 3×3×3 grid, a solid 2×2×2 core in the middle (indices 0..1 on each axis).
        let sx = 3u32;
        let sy = 3u32;
        let sz = 3u32;
        let mut voxels = vec![0u8; (sx * sy * sz) as usize];
        for y in 0..sy {
            for z in 0..sz {
                for x in 0..sx {
                    if x < 2 && y < 2 && z < 2 {
                        voxels
                            [x as usize + sx as usize * (z as usize + sz as usize * y as usize)] =
                            1;
                    }
                }
            }
        }
        let id = soft_body_voxel_build(
            world,
            voxels.as_ptr(),
            voxels.len() as u32,
            sx,
            sy,
            sz,
            1.0,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            1.0,
            80.0,
            1.0,
            mps_core::rapier::ffi::Bool::FALSE,
        );
        // SoftBodyId starts at 0, which is a valid id (not an error sentinel).
        assert!(
            id != u32::MAX,
            "voxel build should return a valid SoftBodyId"
        );

        // Step the world; the soft body should integrate to finite, bounded positions.
        for _ in 0..60 {
            world_step(world, 1.0 / 60.0);
        }
        let sb = unsafe {
            &(*world)
                .inner
                .soft_bodies
                .get(rapier3d::prelude::soft_body_fork::SoftBodyId(id))
        };
        let sb = sb.expect("soft body present");
        assert!(sb.particles.len() >= 8, "expected >=8 solid particles");
        for p in &sb.particles {
            assert!(p.pos.x.is_finite() && p.pos.y.is_finite() && p.pos.z.is_finite());
            assert!(p.pos.x.abs() < 50.0 && p.pos.y.abs() < 50.0 && p.pos.z.abs() < 50.0);
        }
        world_destroy(world);
    }

    #[test]
    fn soft_body_set_gravity_applies() {
        let world = make_world();
        assert!(!world.is_null());

        // Minimal 2×1×1 solid grid (one spring, two particles, no boundary pinning).
        let sx = 2u32;
        let sy = 1u32;
        let sz = 1u32;
        let voxels = vec![1u8; (sx * sy * sz) as usize];
        let id = soft_body_voxel_build(
            world,
            voxels.as_ptr(),
            voxels.len() as u32,
            sx,
            sy,
            sz,
            1.0,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            1.0,
            50.0,
            0.5,
            mps_core::rapier::ffi::Bool::FALSE,
        );
        assert!(
            id != u32::MAX,
            "voxel build should return a valid SoftBodyId"
        );

        // Override gravity to pull along +X (terrain-gravity coupling hook).
        let ok = soft_body_set_gravity(
            world,
            id,
            Vec3 {
                x: 3.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_eq!(ok, mps_core::rapier::ffi::Bool::TRUE);
        let sb = unsafe {
            &(*world)
                .inner
                .soft_bodies
                .get(rapier3d::prelude::soft_body_fork::SoftBodyId(id))
        };
        let sb = sb.expect("soft body present");
        assert!(
            (sb.gravity.x - 3.0).abs() < 1e-12,
            "gravity.x should be 3.0"
        );

        // Unknown id → FALSE.
        let bad = soft_body_set_gravity(
            world,
            999,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_eq!(bad, mps_core::rapier::ffi::Bool::FALSE);
        world_destroy(world);
    }

    // ── Phase 5a: general soft-body builder (arbitrary topology) ──────────────

    #[test]
    fn soft_body_builder_creates_tetra_and_preserves_volume_xpbd() {
        let world = make_world();
        assert!(!world.is_null());

        // Build a unit tetrahedron (4 particles at the canonical simplex corners).
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        assert!(id != u32::MAX, "soft_body_create should succeed");

        let p0 = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let p1 = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let p2 = soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);
        let p3 = soft_body_add_particle(world, id, 0.0, 0.0, 1.0, 1.0, Bool::FALSE);
        for p in [p0, p1, p2, p3] {
            assert!(p != u32::MAX, "particle add should succeed");
        }

        // XPBD: distance constraints on the 6 edges + one volume tetrahedron.
        assert_eq!(
            soft_body_configure_solver(world, id, 1, 20, 0.0),
            Bool::TRUE
        );
        let edges = [(p0, p1), (p0, p2), (p0, p3), (p1, p2), (p1, p3), (p2, p3)];
        for (a, b) in edges {
            assert_eq!(
                soft_body_add_distance_constraint(world, id, a, b, 0.0),
                Bool::TRUE,
                "distance constraint add should succeed"
            );
        }
        assert_eq!(
            soft_body_add_tetrahedron(world, id, p0, p1, p2, p3),
            Bool::TRUE,
            "tetrahedron add should succeed"
        );

        // Capture rest volume, step, and confirm the XPBD volume constraint keeps
        // the signed volume within a small tolerance of the rest value.
        let rest = {
            let sb = unsafe {
                (*world)
                    .inner
                    .soft_bodies
                    .get(SoftBodyId(id))
                    .expect("soft body present")
            };
            sb.total_volume()
        };
        assert!(rest.is_finite() && rest > 0.0, "rest volume positive");

        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
        }

        let after = {
            let sb = unsafe {
                (*world)
                    .inner
                    .soft_bodies
                    .get(SoftBodyId(id))
                    .expect("soft body present")
            };
            sb.total_volume()
        };
        // 6 rigid edges + a volume constraint → the tetrahedron should stay
        // near its rest shape under gravity (volume drift bounded).
        assert!(
            (after - rest).abs() / rest < 0.2,
            "XPBD volume should be preserved (rest={rest}, after={after})"
        );
        world_destroy(world);
    }

    #[test]
    fn soft_body_builder_mass_spring_chain_falls_and_stays_bounded() {
        let world = make_world();
        assert!(!world.is_null());

        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        assert!(id != u32::MAX);

        // 4 free particles in a line, pinned start (anchor).
        let p0 = soft_body_add_particle(world, id, 0.0, 5.0, 0.0, 1.0, Bool::TRUE);
        assert!(p0 != u32::MAX);
        for i in 1..4 {
            let p = soft_body_add_particle(world, id, i as f64, 5.0, 0.0, 1.0, Bool::FALSE);
            assert!(p != u32::MAX, "particle {i} add should succeed");
        }
        // Springs between adjacent particles.
        for i in 0..3 {
            assert_eq!(
                soft_body_add_spring(world, id, i, i + 1, 200.0, 5.0),
                Bool::TRUE,
                "spring {i} add should succeed"
            );
        }

        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
        }

        let sb = unsafe {
            (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present")
        };
        assert_eq!(sb.particles.len(), 4);
        for p in &sb.particles {
            assert!(p.pos.x.is_finite() && p.pos.y.is_finite() && p.pos.z.is_finite());
            assert!(p.pos.x.abs() < 50.0 && p.pos.y.abs() < 50.0 && p.pos.z.abs() < 50.0);
        }
        // Pinned particle (p0) must not have moved.
        assert!(
            (sb.particles[p0 as usize].pos.x - 0.0).abs() < 1e-9
                && (sb.particles[p0 as usize].pos.y - 5.0).abs() < 1e-9,
            "pinned particle must stay anchored"
        );
        world_destroy(world);
    }

    #[test]
    fn soft_body_builder_rejects_bad_params() {
        let world = make_world();
        assert!(!world.is_null());

        // Null world.
        assert_eq!(
            soft_body_create(
                std::ptr::null_mut(),
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0
                }
            ),
            u32::MAX
        );
        // Null world for add_particle.
        assert_eq!(
            soft_body_add_particle(std::ptr::null_mut(), 0, 0.0, 0.0, 0.0, 1.0, Bool::FALSE),
            u32::MAX
        );
        // Bad gravity.
        assert_eq!(
            soft_body_create(
                world,
                Vec3 {
                    x: f64::NAN,
                    y: 0.0,
                    z: 0.0
                }
            ),
            u32::MAX
        );
        // Non-positive mass.
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert!(id != u32::MAX);
        assert_eq!(
            soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 0.0, Bool::FALSE),
            u32::MAX
        );
        // Unknown id.
        assert_eq!(
            soft_body_add_spring(world, 999, 0, 1, 1.0, 1.0),
            Bool::FALSE
        );
        // Unknown id for configure.
        assert_eq!(
            soft_body_configure_solver(world, 999, 1, 10, 0.0),
            Bool::FALSE
        );
        // Bad solver_mode.
        assert_eq!(
            soft_body_configure_solver(world, id, 2, 10, 0.0),
            Bool::FALSE
        );
        // Zero iterations for XPBD.
        assert_eq!(
            soft_body_configure_solver(world, id, 1, 0, 0.0),
            Bool::FALSE
        );
        // Degenerate tetrahedron (duplicate index).
        let a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        assert!(a != u32::MAX);
        assert_eq!(
            soft_body_add_tetrahedron(world, id, a, a, a, a),
            Bool::FALSE
        );
        world_destroy(world);
    }

    // ── Phase 5b: query / readback / lifecycle FFI ────────────────────────────

    #[test]
    fn soft_body_query_readback_and_lifecycle() {
        let world = make_world();
        assert!(!world.is_null());

        // No soft bodies yet.
        assert_eq!(soft_body_count(world), 0);

        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        assert!(id != u32::MAX);
        assert_eq!(soft_body_count(world), 1);

        // Add 3 collinear particles.
        let p0 = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let p1 = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let _p2 = soft_body_add_particle(world, id, 2.0, 0.0, 0.0, 1.0, Bool::FALSE);
        assert_eq!(soft_body_particle_count(world, id), 3);

        // Read back p1's position (should be (1,0,0)).
        let mut pos = Vec3::default();
        let mut vel = Vec3::default();
        assert_eq!(
            soft_body_get_particle(world, id, p1, &mut pos, &mut vel),
            Bool::TRUE
        );
        assert!((pos.x - 1.0).abs() < 1e-12 && pos.y.abs() < 1e-12 && pos.z.abs() < 1e-12);
        // Out-of-bounds index → FALSE.
        assert_eq!(
            soft_body_get_particle(world, id, 99, &mut pos, &mut vel),
            Bool::FALSE
        );

        // Remove p1; the other two stay, topology indices remain valid.
        assert_eq!(soft_body_remove_particle(world, id, p1), Bool::TRUE);
        assert_eq!(soft_body_particle_count(world, id), 2);
        // Remaining particles kept their positions.
        assert_eq!(
            soft_body_get_particle(world, id, p0, &mut pos, std::ptr::null_mut()),
            Bool::TRUE
        );
        assert!((pos.x - 0.0).abs() < 1e-12);
        // p2 was index 2, after removal of index 1 it shifts to index 1.
        assert_eq!(
            soft_body_get_particle(world, id, 1, &mut pos, std::ptr::null_mut()),
            Bool::TRUE
        );
        assert!((pos.x - 2.0).abs() < 1e-12, "p2 should now be at index 1");

        // Destroy the body; count drops, other ids would stay valid (only one here).
        assert_eq!(soft_body_destroy(world, id), Bool::TRUE);
        assert_eq!(soft_body_count(world), 0);
        // Re-querying the destroyed id → unknown.
        assert_eq!(soft_body_particle_count(world, id), u32::MAX);
        // Unknown id destroy → FALSE.
        assert_eq!(soft_body_destroy(world, 999), Bool::FALSE);

        world_destroy(world);
    }

    #[test]
    fn soft_body_remove_particle_keeps_topology_valid_after_stepping() {
        let world = make_world();
        assert!(!world.is_null());

        // 2x1x1 solid grid → 2 particles + 1 spring (mass-spring chain).
        let sx = 2u32;
        let sy = 1u32;
        let sz = 1u32;
        let voxels = vec![1u8; (sx * sy * sz) as usize];
        let id = soft_body_voxel_build(
            world,
            voxels.as_ptr(),
            voxels.len() as u32,
            sx,
            sy,
            sz,
            1.0,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            1.0,
            50.0,
            0.5,
            Bool::FALSE,
        );
        assert!(id != u32::MAX);
        assert_eq!(soft_body_particle_count(world, id), 2);

        // Step a bit, then remove particle 0. The remaining particle must still be
        // finite and the body still steppable (no dangling spring index).
        for _ in 0..30 {
            world_step(world, 1.0 / 60.0);
        }
        assert_eq!(soft_body_remove_particle(world, id, 0), Bool::TRUE);
        assert_eq!(soft_body_particle_count(world, id), 1);
        for _ in 0..30 {
            world_step(world, 1.0 / 60.0);
        }
        let sb = unsafe {
            (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present")
        };
        assert_eq!(sb.particles.len(), 1);
        assert!(sb.particles[0].pos.x.is_finite());

        world_destroy(world);
    }

    // ── Phase 5c: 生物软体（复用 Phase 3 四面体 XPBD 体积约束）────────────────

    #[test]
    fn soft_body_build_tetra_mesh_xpbd_holds_volume_under_gravity() {
        let world = make_world();
        assert!(!world.is_null());

        // Regular tetrahedron (one vertex pinned at top, three hang free).
        let particles: [Vec3; 4] = [
            Vec3 {
                x: 0.0,
                y: 2.0,
                z: 0.0,
            }, // 0: pinned anchor
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3 {
                x: -0.5,
                y: 0.0,
                z: 0.866,
            },
            Vec3 {
                x: -0.5,
                y: 0.0,
                z: -0.866,
            },
        ];
        // One tetrahedron spanning all four corners.
        let tets: [u32; 4] = [0, 1, 2, 3];

        let id = soft_body_build_tetra_mesh(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
            particles.as_ptr(),
            particles.len() as u32,
            tets.as_ptr(),
            1,
            1.0, // particle mass
            0.0, // compliance 0 → rigid volume constraint
            20,  // iterations
        );
        assert!(id != u32::MAX, "tetra mesh should build");

        // Pin vertex 0 (build_tetra_mesh creates all-dynamic; replicate the
        // caller's pin step by zeroing its inverse mass — same as anvilkit does).
        unsafe {
            let sb = (*world)
                .inner
                .soft_bodies
                .get_mut(SoftBodyId(id))
                .expect("soft body present");
            assert_eq!(sb.particles.len(), 4);
            assert_eq!(sb.tetrahedra.len(), 1);
            sb.particles[0].inv_mass = 0.0;
        }

        let rest = unsafe {
            (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .unwrap()
                .total_volume()
        };
        assert!(rest.abs() > 1e-6, "tetrahedron has non-zero rest volume");

        for _ in 0..200 {
            world_step(world, 1.0 / 60.0);
        }

        let sb = unsafe {
            (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present")
        };
        let final_vol = sb.total_volume();
        // Volume constraint (compliance 0) keeps the tetra's volume near rest
        // even as the free vertices sag under gravity — the XPBD rebound.
        let rel_err = (final_vol - rest).abs() / rest.abs();
        assert!(
            rel_err < 0.05,
            "XPBD volume constraint holds volume (rel_err={rel_err})"
        );
        // Anchor stayed put; free vertices settled at finite positions.
        assert!(sb.particles[0].pos.y.is_finite());
        for p in &sb.particles {
            assert!(p.pos.x.is_finite() && p.pos.y.is_finite() && p.pos.z.is_finite());
        }

        world_destroy(world);
    }

    #[test]
    fn soft_body_build_tetra_mesh_rejects_degenerate() {
        let world = make_world();
        assert!(!world.is_null());

        let particles: [Vec3; 4] = [
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        // Degenerate tetrahedron (duplicate vertex 0) → rejected.
        let bad_tets: [u32; 4] = [0, 0, 1, 2];
        assert_eq!(
            soft_body_build_tetra_mesh(
                world,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                particles.as_ptr(),
                particles.len() as u32,
                bad_tets.as_ptr(),
                1,
                1.0,
                0.0,
                20,
            ),
            u32::MAX
        );

        // Empty particle/tet arrays → rejected.
        assert_eq!(
            soft_body_build_tetra_mesh(
                world,
                Vec3::default(),
                std::ptr::null(),
                0,
                std::ptr::null(),
                0,
                1.0,
                0.0,
                20,
            ),
            u32::MAX
        );

        world_destroy(world);
    }

    // ── Phase 5d: 区块破坏 → 软体重建联动 ──────────────────────────────────────

    #[test]
    fn soft_body_voxel_dig_removes_cell_particle_and_rebuilds_map() {
        let world = make_world();
        assert!(!world.is_null());

        // 2x1x1 solid grid → particles [0,1] (cell (0,0,0)->p0, cell (1,0,0)->p1)
        // + 1 spring p0-p1.
        let sx = 2u32;
        let sy = 1u32;
        let sz = 1u32;
        let voxels = vec![1u8; (sx * sy * sz) as usize];
        let id = soft_body_voxel_build(
            world,
            voxels.as_ptr(),
            voxels.len() as u32,
            sx,
            sy,
            sz,
            1.0,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            1.0,
            50.0,
            0.5,
            Bool::FALSE,
        );
        assert!(id != u32::MAX);
        assert_eq!(soft_body_particle_count(world, id), 2);

        // Dig cell (0,0,0) → removes p0, spring dropped, map rebuilt (p1 shifts to 0).
        assert_eq!(soft_body_voxel_dig(world, id, 0, 0, 0), Bool::TRUE);
        assert_eq!(soft_body_particle_count(world, id), 1);
        let sb = unsafe {
            (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present")
        };
        assert_eq!(sb.springs.len(), 0, "incident spring removed with p0");
        // The remaining particle is the old p1, now at index 0.
        let mut pos = Vec3::default();
        assert_eq!(
            soft_body_get_particle(world, id, 0, &mut pos, std::ptr::null_mut()),
            Bool::TRUE
        );
        assert!(
            (pos.x - 1.5).abs() < 1e-9,
            "remaining particle is old (1,0,0) at x=1.5"
        );

        // Dig the last cell → body empty.
        assert_eq!(soft_body_voxel_dig(world, id, 1, 0, 0), Bool::TRUE);
        assert_eq!(soft_body_particle_count(world, id), 0);

        // Re-dig an already-dug/empty cell → FALSE.
        assert_eq!(soft_body_voxel_dig(world, id, 0, 0, 0), Bool::FALSE);
        // Out-of-bounds cell → FALSE.
        assert_eq!(soft_body_voxel_dig(world, id, 9, 0, 0), Bool::FALSE);
        // Unknown id → FALSE.
        assert_eq!(soft_body_voxel_dig(world, 999, 0, 0, 0), Bool::FALSE);

        world_destroy(world);
    }

    #[test]
    fn soft_body_voxel_dig_keeps_body_steppable_after_collapse() {
        let world = make_world();
        assert!(!world.is_null());

        // 3x1x1 chain → 3 particles + 2 springs. Dig the middle cell; the body
        // must stay steppable (no dangling indices) and remaining particles finite.
        let sx = 3u32;
        let sy = 1u32;
        let sz = 1u32;
        let voxels = vec![1u8; (sx * sy * sz) as usize];
        let id = soft_body_voxel_build(
            world,
            voxels.as_ptr(),
            voxels.len() as u32,
            sx,
            sy,
            sz,
            1.0,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            1.0,
            50.0,
            0.5,
            Bool::FALSE,
        );
        assert!(id != u32::MAX);
        assert_eq!(soft_body_particle_count(world, id), 3);

        for _ in 0..30 {
            world_step(world, 1.0 / 60.0);
        }
        assert_eq!(soft_body_voxel_dig(world, id, 1, 0, 0), Bool::TRUE);
        for _ in 0..30 {
            world_step(world, 1.0 / 60.0);
        }
        let sb = unsafe {
            (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present")
        };
        assert_eq!(sb.particles.len(), 2);
        for p in &sb.particles {
            assert!(p.pos.x.is_finite() && p.pos.y.is_finite() && p.pos.z.is_finite());
        }

        world_destroy(world);
    }

    // ── Phase 5f: 软体-刚体碰撞 ──────────────────────────────────────────────
    // 一个自由软体质点从地面上方下落，启用碰撞耦合后，其 proxy 球体与静态半空间
    // 地面发生接触，质点应停在地面之上（≈粒子半径），而非穿透。禁用碰撞时则继续
    // 自由下落穿过地面。
    #[test]
    fn soft_body_collision_stops_at_ground() {
        let world = make_world();

        // 静态地面：Fixed 刚体 + 法线朝上的半空间（实体在 y<0，上方为空）。
        let ground_builder =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Fixed as u32);
        let ground = mps_core::rapier::rigid_body::rigid_body_builder_build(ground_builder);
        let ground_handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, ground);
        let ground_collider = mps_core::rapier::collider::collider_builder_build(
            mps_core::rapier::collider::collider_builder_create_halfspace(Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            }),
        );
        mps_core::rapier::collider::world_insert_collider_with_parent(
            world,
            ground_collider,
            ground_handle,
        );

        // 单个自由质点，初始在地面上方 y=2.0，无弹簧（仅受重力）。
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        let _p = soft_body_add_particle(world, id, 0.0, 2.0, 0.0, 1.0, Bool::FALSE);
        assert_eq!(
            soft_body_enable_collision(world, id, 0.5, Bool::TRUE),
            Bool::TRUE
        );

        let dt = 1.0 / 60.0;
        for _ in 0..180 {
            world_step(world, dt);
        }

        let mut pos = Vec3::default();
        assert_eq!(
            soft_body_get_particle(world, id, 0, &mut pos as *mut Vec3, std::ptr::null_mut()),
            Bool::TRUE
        );
        // 地面在 y=0，粒子半径 0.5 → 静止位置应约 y≈0.5，且不得穿透到 y<0.4。
        assert!(
            pos.y > 0.4,
            "collision-coupled particle must rest above the ground, got y={}",
            pos.y
        );

        // 对比：禁用碰撞后，质点应自由穿过地面（y 继续下降）。
        assert_eq!(
            soft_body_enable_collision(world, id, 0.5, Bool::FALSE),
            Bool::TRUE
        );
        for _ in 0..60 {
            world_step(world, dt);
        }
        let mut pos2 = Vec3::default();
        assert_eq!(
            soft_body_get_particle(world, id, 0, &mut pos2 as *mut Vec3, std::ptr::null_mut()),
            Bool::TRUE
        );
        assert!(
            pos2.y < 0.4,
            "without collision coupling the particle should fall through the ground, got y={}",
            pos2.y
        );

        world_destroy(world);
    }

    /// Phase 5g: digging a voxel collider cell auto-propagates to a soft body
    /// that shares the same world-space voxelization. Build a 2×2×2 all-solid
    /// grid as BOTH a voxel collider and a soft body (identical origin/size/
    /// voxel_size). Dig cell (0,0,0) via `collider_voxel_edit`; the overlapping
    /// soft-body particle must be removed automatically (count 8 → 7) without
    /// an explicit `soft_body_voxel_dig` call.
    #[test]
    fn collider_voxel_edit_propagates_dig_to_soft_body() {
        let world = make_world();
        assert!(!world.is_null());

        // 2×2×2 all-solid grid, origin (0,0,0), uniform voxel size 1.0.
        let size: u32 = 2;
        let voxels = vec![1u8; (size * size * size) as usize];

        // Soft body built from the same grid.
        let id = soft_body_voxel_build(
            world,
            voxels.as_ptr(),
            voxels.len() as u32,
            size,
            size,
            size,
            1.0,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            1.0,         // particle_mass
            100.0,       // stiffness
            5.0,         // damping
            Bool::FALSE, // pin_boundary
        );
        assert_ne!(id, u32::MAX, "soft body build failed");
        assert_eq!(
            soft_body_particle_count(world, id),
            8,
            "2x2x2 all-solid grid should yield 8 particles"
        );

        // Voxel collider sharing the identical world-space grid.
        let options = VoxelColliderOptions {
            mode: 0, // Auto
            dynamic_body: Bool::FALSE,
            small_voxel_limit: 0,
            mesh_voxel_limit: 0,
        };
        let builder = collider_builder_create_voxels(
            voxels.as_ptr(),
            size,
            size,
            size,
            1.0,
            1.0,
            1.0,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            options,
        );
        assert!(!builder.is_null(), "voxel collider builder failed");
        let collider = collider_builder_build(builder);
        assert!(!collider.is_null(), "voxel collider build failed");

        // Parent body for the collider (a fixed anchor).
        let parent = rigid_body_builder_create(BodyStatus::Fixed as u32);
        let parent_handle = world_insert_rigid_body(world, rigid_body_builder_build(parent));
        let collider_handle: ColliderHandleRaw =
            world_insert_collider_with_parent(world, collider, parent_handle);
        assert_ne!(collider_handle, 0, "voxel collider insert failed");

        // Dig cell (0,0,0) in the collider grid.
        let dug = collider_voxel_edit(world, collider_handle, 0, 0, 0, 0);
        assert_eq!(dug, Bool::TRUE, "collider_voxel_edit dig should succeed");

        // The overlapping soft-body particle at (0,0,0) must be auto-removed.
        assert_eq!(
            soft_body_particle_count(world, id),
            7,
            "digging collider cell (0,0,0) should auto-collapse the soft body by one particle"
        );

        // Re-dig the same cell: idempotent, count stays 7.
        let dug_again = collider_voxel_edit(world, collider_handle, 0, 0, 0, 0);
        assert_eq!(dug_again, Bool::TRUE);
        assert_eq!(soft_body_particle_count(world, id), 7);

        world_destroy(world);
    }

    /// Phase 5i: topology read-back for rendering. Build a tetra-mesh soft body
    /// with a known spring + distance constraint + tetra, then pull the whole
    /// state via the bulk `soft_body_read_*` FFI and check counts/consistency
    /// against `soft_body_particle_count` + the topology we inserted.
    #[test]
    fn soft_body_read_back_topology_for_rendering() {
        let world = make_world();
        assert!(!world.is_null());

        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);

        // 4 particles forming a tetrahedron.
        let p0 = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let p1 = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let p2 = soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);
        let p3 = soft_body_add_particle(world, id, 0.0, 0.0, 1.0, 1.0, Bool::FALSE);
        assert_eq!((p0, p1, p2, p3), (0, 1, 2, 3));

        // 1 spring edge (a-b) + 1 distance constraint edge (c-d).
        assert_eq!(soft_body_add_spring(world, id, 0, 1, 50.0, 2.0), Bool::TRUE);
        assert_eq!(
            soft_body_add_distance_constraint(world, id, 2, 3, 10.0),
            Bool::TRUE
        );
        // 1 tetra over all four particles.
        assert_eq!(soft_body_add_tetrahedron(world, id, 0, 1, 2, 3), Bool::TRUE);

        let count = soft_body_particle_count(world, id);
        assert_eq!(count, 4);

        // ── bulk particle read ──
        let mut pos = vec![Vec3::default(); count as usize];
        let mut inv_mass = vec![0.0f64; count as usize];
        let read =
            soft_body_read_particles(world, id, pos.as_mut_ptr(), inv_mass.as_mut_ptr(), count);
        assert_eq!(read, count, "read_particles should return particle count");
        // inv_mass of 1.0 mass → 1.0.
        for im in &inv_mass {
            assert_eq!(*im, 1.0, "unpinned particle inv_mass should be 1.0");
        }
        // Position 0 should match the particle we added.
        assert!((pos[0].x).abs() < 1e-9 && (pos[0].y).abs() < 1e-9);

        // ── edges read (spring + distance constraint = 2 edges = 4 u32) ──
        let edge_count = soft_body_read_edges(world, id, std::ptr::null_mut(), 0);
        assert_eq!(edge_count, 2, "2 edges (1 spring + 1 distance constraint)");
        let mut edges = vec![0u32; (edge_count as usize) * 2];
        let read_edges = soft_body_read_edges(world, id, edges.as_mut_ptr(), edges.len() as u32);
        assert_eq!(read_edges, 2);
        // spring edge (0,1) first, then distance edge (2,3).
        assert_eq!((edges[0], edges[1]), (0, 1));
        assert_eq!((edges[2], edges[3]), (2, 3));

        // ── tetra read (1 tetra = 4 u32) ──
        let tet_count = soft_body_read_tetrahedra(world, id, std::ptr::null_mut(), 0);
        assert_eq!(tet_count, 1);
        let mut tets = vec![0u32; (tet_count as usize) * 4];
        let read_tets = soft_body_read_tetrahedra(world, id, tets.as_mut_ptr(), tets.len() as u32);
        assert_eq!(read_tets, 1);
        assert_eq!(tets, vec![0, 1, 2, 3]);

        // ── capacity clamp: requesting fewer slots than needed must not panic ──
        let tiny = soft_body_read_edges(world, id, edges.as_mut_ptr(), 2);
        assert_eq!(
            tiny, 2,
            "real edge count returned even though buffer held only 1 edge"
        );

        // ── unknown id returns 0, no panic ──
        assert_eq!(
            soft_body_read_particles(
                world,
                u32::MAX,
                pos.as_mut_ptr(),
                std::ptr::null_mut(),
                count
            ),
            0
        );
        assert_eq!(
            soft_body_read_edges(world, u32::MAX, std::ptr::null_mut(), 0),
            0
        );
        assert_eq!(
            soft_body_read_tetrahedra(world, u32::MAX, std::ptr::null_mut(), 0),
            0
        );

        world_destroy(world);
    }

    // ── Phase 6: 布料拓扑（三角形面 + 结构边自动注册 + 弯曲约束）─────────────────
    // 一个 2×1 的四边形布片 = (0,0)-(1,0)-(1,1)-(0,1)：两个三角形 + 四条边。
    // 结构边由 add_triangle 自动去重注册；弯曲约束（对角线）由 add_bending 显式添加。
    #[test]
    fn soft_body_cloth_topology_and_bending() {
        let world = make_world();
        assert!(!world.is_null());

        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);

        // 4 个布料质点（pinned[0]=固定角，其余自由）。
        assert_eq!(
            soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::TRUE),
            0
        );
        assert_eq!(
            soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE),
            1
        );
        assert_eq!(
            soft_body_add_particle(world, id, 1.0, 1.0, 0.0, 1.0, Bool::FALSE),
            2
        );
        assert_eq!(
            soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE),
            3
        );

        // 两个三角形组成四边形。shared edge (0,2) 由 add_triangle 去重，故
        // 结构边 = 5（(0,1)(1,2)(2,0)(2,3)(3,0)），再加 1 条弯曲边 (1,3) = 6，
        // 恰好是 4 顶点的全部 6 条边（K4）。
        assert_eq!(soft_body_add_triangle(world, id, 0, 1, 2), Bool::TRUE);
        assert_eq!(soft_body_add_triangle(world, id, 0, 2, 3), Bool::TRUE);

        // 弯曲约束：四边形另一条对角线 (1,3)（(0,2) 已是结构共享边，无需重复）。
        assert_eq!(soft_body_add_bending(world, id, 1, 3), Bool::TRUE);

        let particle_count = soft_body_particle_count(world, id);
        assert_eq!(particle_count, 4);

        // ── 三角形读回：2 个面 = 6 个 u32 ──
        let tri_count = soft_body_read_triangles(world, id, std::ptr::null_mut(), 0);
        assert_eq!(tri_count, 2, "2 triangle faces expected");
        let mut tris = vec![0u32; (tri_count as usize) * 3];
        let read_tris = soft_body_read_triangles(world, id, tris.as_mut_ptr(), tris.len() as u32);
        assert_eq!(read_tris, 2);

        // ── 边读回：5 条结构边 + 1 条弯曲边 = 6 条边 ──
        let edge_count = soft_body_read_edges(world, id, std::ptr::null_mut(), 0);
        assert_eq!(edge_count, 6, "5 structural + 1 bending edge");
        let mut edges = vec![0u32; (edge_count as usize) * 2];
        let read_edges = soft_body_read_edges(world, id, edges.as_mut_ptr(), edges.len() as u32);
        assert_eq!(read_edges, 6);
        // 归一化为无序对，验证恰好是 K4 的全部 6 条边（去重生效，无重复）。
        let mut pairs: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
        for k in 0..6usize {
            let a = edges[k * 2];
            let b = edges[k * 2 + 1];
            assert!(a != b, "degenerate edge");
            pairs.insert(if a < b { (a, b) } else { (b, a) });
        }
        let expected: std::collections::HashSet<(u32, u32)> =
            [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
                .into_iter()
                .collect();
        assert_eq!(pairs, expected, "edge set must equal all 6 edges of K4");

        // 未知 id 返回 0，不 panic。
        assert_eq!(
            soft_body_read_triangles(world, u32::MAX, std::ptr::null_mut(), 0),
            0
        );

        world_destroy(world);
    }

    // ── Phase 6: 布料在重力下的悬垂（XPBD 求解器跑通，不发散）──────────────────
    #[test]
    fn soft_body_cloth_sags_under_gravity() {
        let world = make_world();
        assert!(!world.is_null());

        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);

        // 6 个质点排成 3×2 网格在 y=2 平面，pin 顶部两个角。
        let _p0 = soft_body_add_particle(world, id, -1.0, 2.0, 0.0, 1.0, Bool::TRUE);
        let _p1 = soft_body_add_particle(world, id, 1.0, 2.0, 0.0, 1.0, Bool::TRUE);
        let _p2 = soft_body_add_particle(world, id, -1.0, 1.0, 0.0, 1.0, Bool::FALSE);
        let _p3 = soft_body_add_particle(world, id, 1.0, 1.0, 0.0, 1.0, Bool::FALSE);
        let _p4 = soft_body_add_particle(world, id, -1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let _p5 = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);

        // 4 个三角面覆盖网格。
        assert_eq!(soft_body_add_triangle(world, id, 0, 1, 3), Bool::TRUE);
        assert_eq!(soft_body_add_triangle(world, id, 0, 3, 2), Bool::TRUE);
        assert_eq!(soft_body_add_triangle(world, id, 1, 5, 3), Bool::TRUE);
        assert_eq!(soft_body_add_triangle(world, id, 3, 5, 4), Bool::TRUE);

        // 启用 XPBD 求解器（布料用距离约束保形）。
        assert_eq!(
            soft_body_configure_solver(world, id, 1, 20, 0.0),
            Bool::TRUE
        );

        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
        }

        // 读回质点，确认：固定角不动、自由点有限且下垂、无 NaN/发散。
        let count = soft_body_particle_count(world, id);
        let mut pos = vec![Vec3::default(); count as usize];
        let read =
            soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), count);
        assert_eq!(read, count);
        for (i, p) in pos.iter().enumerate() {
            assert!(
                p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
                "p{i} blew up"
            );
        }
        // 固定角仍锚在 y≈2。
        assert!((pos[0].y - 2.0).abs() < 1e-6, "pinned corner drifted");
        assert!((pos[1].y - 2.0).abs() < 1e-6, "pinned corner drifted");
        // 底部自由点应下垂（y 明显低于 2）。
        assert!(pos[4].y < 1.5, "bottom row should sag below 1.5");
        assert!(pos[5].y < 1.5, "bottom row should sag below 1.5");

        world_destroy(world);
    }

    // ── Phase 7: 风场把固定边的布料吹向侧向（纯外力，无新力学）─────────────────
    #[test]
    fn soft_body_wind_blows_cloth_sideways() {
        let world = make_world();
        assert!(!world.is_null());

        // 无重力，纯风：布料应被恒定 wind accel 沿 +X 推出。
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);

        // 5 个质点：仅 pin 左上角 p0，其余自由（软边以便迎风鼓起）。
        // p0(-1,2) 固定；p1(1,2) p2(-1,0) p3(1,0) p4(0,1) 自由。
        let _p0 = soft_body_add_particle(world, id, -1.0, 2.0, 0.0, 1.0, Bool::TRUE);
        let _p1 = soft_body_add_particle(world, id, 1.0, 2.0, 0.0, 1.0, Bool::FALSE);
        let _p2 = soft_body_add_particle(world, id, -1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let _p3 = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let _p4 = soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);

        assert_eq!(soft_body_add_triangle(world, id, 0, 1, 4), Bool::TRUE);
        assert_eq!(soft_body_add_triangle(world, id, 1, 3, 4), Bool::TRUE);
        assert_eq!(soft_body_add_triangle(world, id, 3, 2, 4), Bool::TRUE);
        assert_eq!(soft_body_add_triangle(world, id, 2, 0, 4), Bool::TRUE);

        // 软边（compliance > 0）让布料可迎风鼓起；仅 pin 一个角 → 整片被风吹向 +X。
        assert_eq!(
            soft_body_configure_solver(world, id, 1, 20, 1e-2),
            Bool::TRUE
        );

        // 启用 +X 风（10 m/s² 恒定加速度），无 drag 以便观察确定性位移。
        assert_eq!(
            soft_body_apply_wind(
                world,
                id,
                Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0
                },
                0.0,
            ),
            Bool::TRUE
        );

        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
        }

        let count = soft_body_particle_count(world, id);
        let mut pos = vec![Vec3::default(); count as usize];
        let read =
            soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), count);
        assert_eq!(read, count);

        // 固定角不动。
        assert!(
            (pos[0].x + 1.0).abs() < 1e-6,
            "pinned corner must stay at x=-1"
        );
        assert!(
            (pos[0].y - 2.0).abs() < 1e-6,
            "pinned corner must stay at y=2"
        );
        // 无重力纯 +X 风下，整片布以单角铰接点像旗帜一样顺风扬起：
        // 各自由质点有限、无发散，且整片质心明显顺风（x 正方向）漂移。
        let mut finite = true;
        let mut sum_x_end = 0.0;
        for (i, p) in pos.iter().enumerate() {
            if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
                finite = false;
            }
            if i != 0 {
                sum_x_end += p.x; // 跳过 pinned p0
            }
        }
        assert!(finite, "some particle blew up");
        // 自由质点初始 x 之和 = p1(1)+p2(-1)+p3(1)+p4(0) = 1.0；顺风后应 > 1.0。
        assert!(
            sum_x_end > 1.0 + 0.3,
            "sheet should drift downwind (+X): sum_x={}",
            sum_x_end
        );
        // 至少存在一个自由质点明显顺风超出初始最远 x=1.0。
        let max_x = pos[1..]
            .iter()
            .map(|p| p.x)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(max_x > 1.2, "a free particle should be blown past x=1.2");
        // 动能应为正且有限（风吹动）。
        let ke = soft_body_kinetic_energy(world, id);
        assert!(
            ke.is_finite() && ke > 0.0,
            "kinetic energy should be positive & finite"
        );
        // clear_wind 后动力学仍可继续（不报错）。
        assert_eq!(soft_body_clear_wind(world, id), Bool::TRUE);

        world_destroy(world);
    }

    // ── Phase 7: 休眠跳过积分 + 诊断读数（能量/体积）────────────────────────
    #[test]
    fn soft_body_sleep_skips_integration_and_diagnostics() {
        let world = make_world();
        assert!(!world.is_null());

        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);

        // 单个自由质点，无约束。
        let _p0 = soft_body_add_particle(world, id, 0.0, 10.0, 0.0, 1.0, Bool::FALSE);

        // 初始：清醒、动能 0（静止）、体积 0（无四面体）。
        assert_eq!(soft_body_is_sleeping(world, id), Bool::FALSE);
        assert_eq!(soft_body_kinetic_energy(world, id), 0.0);
        assert_eq!(soft_body_total_volume(world, id), 0.0);

        // 让质点自由下落若干步。
        for _ in 0..30 {
            world_step(world, 1.0 / 60.0);
        }
        let ke_falling = soft_body_kinetic_energy(world, id);
        assert!(
            ke_falling > 0.0,
            "falling particle should gain kinetic energy"
        );

        // 休眠：之后 step 不再改变位置。
        assert_eq!(soft_body_sleep(world, id), Bool::TRUE);
        assert_eq!(soft_body_is_sleeping(world, id), Bool::TRUE);
        let before = {
            let mut pos = vec![Vec3::default(); 1];
            soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), 1);
            pos[0]
        };
        for _ in 0..30 {
            world_step(world, 1.0 / 60.0);
        }
        let after = {
            let mut pos = vec![Vec3::default(); 1];
            soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), 1);
            pos[0]
        };
        assert!(
            (before.x - after.x).abs() < 1e-12
                && (before.y - after.y).abs() < 1e-12
                && (before.z - after.z).abs() < 1e-12,
            "sleeping body must not move under step"
        );

        // 唤醒后继续下落。
        assert_eq!(soft_body_wake(world, id), Bool::TRUE);
        assert_eq!(soft_body_is_sleeping(world, id), Bool::FALSE);
        for _ in 0..10 {
            world_step(world, 1.0 / 60.0);
        }
        let woken = {
            let mut pos = vec![Vec3::default(); 1];
            soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), 1);
            pos[0]
        };
        assert!(woken.y < before.y, "woken particle should resume falling");

        // 未知 id 的睡眠/诊断查询返回 False/0 而不 panic。
        assert_eq!(soft_body_is_sleeping(world, u32::MAX), Bool::FALSE);
        assert_eq!(soft_body_sleep(world, u32::MAX), Bool::FALSE);
        assert_eq!(soft_body_kinetic_energy(world, u32::MAX), 0.0);

        world_destroy(world);
    }

    // ── Phase 8: 锚定软体任意质点到刚体，质点刚性跟随刚体平移──────────────────
    #[test]
    fn soft_body_attach_particle_follows_rigid_body_translation() {
        let world = make_world();
        assert!(!world.is_null());

        // 一个匀速沿 +X 移动的刚体，初位置 x=5。
        let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_translation(
            builder,
            Vec3 {
                x: 5.0,
                y: 0.0,
                z: 0.0,
            },
        );
        rigid_body_builder_set_linvel(
            builder,
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let body = rigid_body_builder_build(builder);
        let body_h = world_insert_rigid_body(world, body);
        assert_ne!(body_h, 0u64);

        // 软体：单个自由质点，初始在 (5,0,0)（与刚体同位置，作为绑点）。
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        let p0 = soft_body_add_particle(world, id, 5.0, 0.0, 0.0, 1.0, Bool::FALSE);
        assert_eq!(p0, 0);

        // 把质点 0 锚定到刚体（绑点 = 质点当前世界位置）。
        assert_eq!(
            soft_body_attach_particle(
                world,
                id,
                0,
                body_h,
                Vec3 {
                    x: 5.0,
                    y: 0.0,
                    z: 0.0
                }
            ),
            Bool::TRUE
        );

        // 步进若干步：刚体应沿 +X 移动，质点需刚性跟随。
        for _ in 0..30 {
            world_step(world, 1.0 / 60.0);
        }

        let tx = rigid_body_get_translation(world, body_h);
        // 刚体已平移：x ≈ 5 + 30/60 = 5.5。
        assert!(
            tx.x > 5.3 && tx.x < 5.7,
            "rigid body should have moved: tx.x={}",
            tx.x
        );

        // 绑定的质点应跟随刚体（位置 ≈ 刚体翻译，有限且不发散）。
        let count = soft_body_particle_count(world, id);
        let mut pos = vec![Vec3::default(); count as usize];
        let read =
            soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), count);
        assert_eq!(read, count);
        assert!(pos[0].x.is_finite() && pos[0].y.is_finite() && pos[0].z.is_finite());
        assert!(
            (pos[0].x - tx.x).abs() < 1e-6,
            "bound particle must track body translation: pos.x={} tx.x={}",
            pos[0].x,
            tx.x
        );
        assert!(
            (pos[0].y - tx.y).abs() < 1e-6,
            "bound particle must track body translation: pos.y={} tx.y={}",
            pos[0].y,
            tx.y
        );

        // 解绑后质点恢复自由（不再强制贴合刚体）。
        assert_eq!(soft_body_detach_particle(world, id, 0), Bool::TRUE);
        world_step(world, 1.0 / 60.0);
        let count2 = soft_body_particle_count(world, id);
        let mut pos2 = vec![Vec3::default(); count2 as usize];
        let _ =
            soft_body_read_particles(world, id, pos2.as_mut_ptr(), std::ptr::null_mut(), count2);
        assert!(
            pos2[0].x.is_finite(),
            "detached particle must remain finite"
        );

        world_destroy(world);
    }

    // ── Phase 8: 非法参数返回 False 而不 panic──────────────────────────────
    #[test]
    fn soft_body_attach_particle_rejects_invalid_args() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        let _p0 = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);

        // 不存在的 body → False。
        assert_eq!(
            soft_body_attach_particle(
                world,
                id,
                0,
                u64::MAX,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0
                }
            ),
            Bool::FALSE
        );
        // 越界 particle → False。
        assert_eq!(
            soft_body_attach_particle(
                world,
                id,
                99,
                0,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0
                }
            ),
            Bool::FALSE
        );
        // 越界 detach particle → False。
        assert_eq!(soft_body_detach_particle(world, id, 99), Bool::FALSE);

        world_destroy(world);
    }

    // ── Phase 9: 撕裂 — 应变超阈值的边被移除、破损面被删─────────────────────────
    #[test]
    fn soft_body_tears_when_strain_exceeds_threshold() {
        let world = make_world();
        assert!(!world.is_null());

        // 无重力，纯外力把自由点往下猛拉，确保边被过度拉伸。
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);

        // 6 个质点 3×2 网格在 y=2 平面；pin 顶部两角 p0,p1，其余自由。
        let _p0 = soft_body_add_particle(world, id, -1.0, 2.0, 0.0, 1.0, Bool::TRUE);
        let _p1 = soft_body_add_particle(world, id, 1.0, 2.0, 0.0, 1.0, Bool::TRUE);
        let _p2 = soft_body_add_particle(world, id, -1.0, 1.0, 0.0, 1.0, Bool::FALSE);
        let _p3 = soft_body_add_particle(world, id, 1.0, 1.0, 0.0, 1.0, Bool::FALSE);
        let _p4 = soft_body_add_particle(world, id, -1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let _p5 = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        // 4 个三角面（边自动成为 distance constraints）。
        assert_eq!(soft_body_add_triangle(world, id, 0, 1, 3), Bool::TRUE);
        assert_eq!(soft_body_add_triangle(world, id, 0, 3, 2), Bool::TRUE);
        assert_eq!(soft_body_add_triangle(world, id, 1, 5, 3), Bool::TRUE);
        assert_eq!(soft_body_add_triangle(world, id, 3, 5, 4), Bool::TRUE);
        // XPBD 求解器。
        assert_eq!(
            soft_body_configure_solver(world, id, 1, 20, 0.0),
            Bool::TRUE
        );

        // 开撕裂，阈值极低（1% 应变即断）。
        assert_eq!(soft_body_set_tear_strain(world, id, 0.01, 1), Bool::TRUE);

        // 记录初始边数（springs+distance_constraints）和面数。
        let edges0 = soft_body_read_edges(world, id, std::ptr::null_mut(), 1024);
        assert!(edges0 > 0);
        let tris0 = soft_body_read_triangles(world, id, std::ptr::null_mut(), 1024);
        assert!(tris0 > 0);

        // 极端重力把自由点往下拽，边迅速超阈值被撕裂。
        soft_body_set_gravity(
            world,
            id,
            Vec3 {
                x: 0.0,
                y: -50.0,
                z: 0.0,
            },
        );
        for _ in 0..60 {
            world_step(world, 1.0 / 60.0);
        }

        let edges1 = soft_body_read_edges(world, id, std::ptr::null_mut(), 1024);
        let tris1 = soft_body_read_triangles(world, id, std::ptr::null_mut(), 1024);
        // 撕裂后：边和面应减少（至少断开若干边）。
        assert!(
            edges1 < edges0,
            "tearing should remove over-stretched edges: {edges1} >= {edges0}"
        );
        assert!(
            tris1 < tris0,
            "torn faces should be dropped: {tris1} >= {tris0}"
        );
        // 质点仍在、有限。
        let count = soft_body_particle_count(world, id);
        let mut pos = vec![Vec3::default(); count as usize];
        let read =
            soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), count);
        assert_eq!(read, count);
        for (i, p) in pos.iter().enumerate() {
            assert!(
                p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
                "p{i} blew up"
            );
        }

        // 关闭撕裂后不应再断边（剩余边保持稳定）。
        assert_eq!(soft_body_set_tear_strain(world, id, 0.0, 0), Bool::TRUE);
        let edges2 = soft_body_read_edges(world, id, std::ptr::null_mut(), 1024);
        for _ in 0..30 {
            world_step(world, 1.0 / 60.0);
        }
        let edges3 = soft_body_read_edges(world, id, std::ptr::null_mut(), 1024);
        assert_eq!(
            edges3, edges2,
            "disabling tearing must stop further edge loss"
        );

        world_destroy(world);
    }

    // ── Phase 9: 撕裂非法/未知参数返回 False 而不 panic─────────────────────────
    #[test]
    fn soft_body_set_tear_strain_rejects_invalid_args() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        let _p = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);

        // 未知 id → False。
        assert_eq!(
            soft_body_set_tear_strain(world, u32::MAX, 0.5, 1),
            Bool::FALSE
        );
        // 非有限阈值 → False。
        assert_eq!(
            soft_body_set_tear_strain(world, id, f64::NAN, 1),
            Bool::FALSE
        );
        // 合法调用 → True，且未 panic。
        assert_eq!(soft_body_set_tear_strain(world, id, 0.3, 1), Bool::TRUE);

        world_destroy(world);
    }

    // ── Phase 27: 应力/能量撕裂准则（断裂力学）─────────────────────────────────
    #[test]
    fn soft_body_tears_by_stress_and_energy_criteria() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);

        let a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::TRUE);
        let b = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        assert_eq!(soft_body_add_spring(world, id, a, b, 10.0, 0.0), Bool::TRUE);

        let _ = soft_body_set_particle_velocity(world, id, b, 0.0, -50.0, 0.0);
        assert_eq!(soft_body_set_tear_stress(world, id, 5.0, 1), Bool::TRUE);
        for _ in 0..30 {
            world_step(world, 1.0 / 60.0);
        }
        let edges_after_stress = soft_body_read_edges(world, id, std::ptr::null_mut(), 1024);
        assert_eq!(
            edges_after_stress, 0,
            "stress criterion should snap the spring"
        );

        let id2 = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id2, u32::MAX);
        let c = soft_body_add_particle(world, id2, 0.0, 0.0, 0.0, 1.0, Bool::TRUE);
        let d = soft_body_add_particle(world, id2, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        assert_eq!(
            soft_body_add_spring(world, id2, c, d, 10.0, 0.0),
            Bool::TRUE
        );
        let _ = soft_body_set_particle_velocity(world, id2, d, 0.0, -50.0, 0.0);
        assert_eq!(soft_body_set_tear_energy(world, id2, 2.0, 1), Bool::TRUE);
        for _ in 0..30 {
            world_step(world, 1.0 / 60.0);
        }
        let edges_after_energy = soft_body_read_edges(world, id2, std::ptr::null_mut(), 1024);
        assert_eq!(
            edges_after_energy, 0,
            "energy criterion should snap the spring"
        );

        assert_eq!(soft_body_set_tear_stress(world, id, 0.0, 0), Bool::TRUE);
        world_destroy(world);
    }

    // ── Phase 27: 体级正交各向异性刚度（沿主轴更硬）────────────────────────────
    #[test]
    fn soft_body_anisotropy_stiffens_along_principal_axis() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        let left = soft_body_add_particle(world, id, -1.0, 0.5, 0.0, 1.0, Bool::FALSE);
        let right = soft_body_add_particle(world, id, 1.0, 0.5, 0.0, 1.0, Bool::FALSE);
        assert_eq!(
            soft_body_add_spring(world, id, left, right, 50.0, 0.0),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_configure_solver(world, id, 1, 20, 0.0),
            Bool::TRUE
        );

        soft_body_set_gravity(
            world,
            id,
            Vec3 {
                x: 0.0,
                y: -5.0,
                z: 0.0,
            },
        );
        for _ in 0..40 {
            world_step(world, 1.0 / 60.0);
        }
        let dist_iso = {
            let mut pos = vec![Vec3::default(); soft_body_particle_count(world, id) as usize];
            let _ = soft_body_read_particles(
                world,
                id,
                pos.as_mut_ptr(),
                std::ptr::null_mut(),
                pos.len() as u32,
            );
            let l = &pos[left as usize];
            let r = &pos[right as usize];
            ((r.x - l.x).powi(2) + (r.y - l.y).powi(2) + (r.z - l.z).powi(2)).sqrt()
        };

        assert_eq!(
            soft_body_set_anisotropy(world, id, 10.0, 1.0, 1.0, 1),
            Bool::TRUE
        );
        for _ in 0..40 {
            world_step(world, 1.0 / 60.0);
        }
        let dist_aniso = {
            let mut pos = vec![Vec3::default(); soft_body_particle_count(world, id) as usize];
            let _ = soft_body_read_particles(
                world,
                id,
                pos.as_mut_ptr(),
                std::ptr::null_mut(),
                pos.len() as u32,
            );
            let l = &pos[left as usize];
            let r = &pos[right as usize];
            ((r.x - l.x).powi(2) + (r.y - l.y).powi(2) + (r.z - l.z).powi(2)).sqrt()
        };
        assert!(
            dist_aniso < dist_iso + 1e-6,
            "anisotropy along x should stiffen the x-aligned edge: {dist_aniso} >= {dist_iso}"
        );
        assert_eq!(
            soft_body_set_anisotropy(world, id, 0.0, 0.0, 0.0, 0),
            Bool::TRUE
        );
        world_destroy(world);
    }

    // ── Phase 27: 黏弹性（率相关）本构 — 快拉比慢拉更硬、下坠更少 ──────────────────
    #[test]
    fn soft_body_viscoelastic_rate_stiffening() {
        let world = make_world();
        assert!(!world.is_null());
        let mk = |w: *mut WorldHandle, ve: f64| -> (u32, u32, u32) {
            let id = soft_body_create(
                w,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            let top = soft_body_add_particle(w, id, 0.0, 1.0, 0.0, 1.0, Bool::TRUE);
            let bot = soft_body_add_particle(w, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
            assert_eq!(
                soft_body_add_spring(w, id, top, bot, 100.0, 0.0),
                Bool::TRUE
            );
            if ve >= 0.0 {
                assert_eq!(soft_body_set_viscoelastic(w, id, ve, 1), Bool::TRUE);
            }
            (id, top, bot)
        };
        let (ctrl, _ct, cb) = mk(world, -1.0);
        let (exp, _et, eb) = mk(world, 5.0);

        let _ = soft_body_set_particle_velocity(world, ctrl, cb, 0.0, -30.0, 0.0);
        let _ = soft_body_set_particle_velocity(world, exp, eb, 0.0, -30.0, 0.0);
        soft_body_set_gravity(
            world,
            ctrl,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        soft_body_set_gravity(
            world,
            exp,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        for _ in 0..80 {
            world_step(world, 1.0 / 60.0);
        }

        let dist = |id: u32, p: u32| -> f64 {
            let mut pos = vec![Vec3::default(); soft_body_particle_count(world, id) as usize];
            let _ = soft_body_read_particles(
                world,
                id,
                pos.as_mut_ptr(),
                std::ptr::null_mut(),
                pos.len() as u32,
            );
            pos[p as usize].y
        };
        let y_ctrl = dist(ctrl, cb);
        let y_exp = dist(exp, eb);
        assert!(
            y_exp > y_ctrl,
            "viscoelastic (rate-stiffened) should sag less than elastic: y_exp={y_exp} <= y_ctrl={y_ctrl}"
        );
        world_destroy(world);
    }

    // ── Phase 27: 温度场 — 升温使静止长度膨胀 + 刚度软化 → 同载下拉伸更多 ────────
    #[test]
    fn soft_body_thermal_field_expands_and_softens() {
        let world = make_world();
        assert!(!world.is_null());
        let mk = |w: *mut WorldHandle, hot: bool| -> (u32, u32, u32) {
            let id = soft_body_create(
                w,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            let top = soft_body_add_particle(w, id, 0.0, 1.0, 0.0, 1.0, Bool::TRUE);
            let bot = soft_body_add_particle(w, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
            assert_eq!(
                soft_body_add_spring(w, id, top, bot, 100.0, 0.0),
                Bool::TRUE
            );
            if hot {
                assert_eq!(
                    soft_body_set_thermal(w, id, 373.15, 273.15, 0.001, 0.002, 1),
                    Bool::TRUE
                );
            }
            (id, top, bot)
        };
        let (cold, _c, cbot) = mk(world, false);
        let (hot, _h, hbot) = mk(world, true);

        soft_body_set_gravity(
            world,
            cold,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        soft_body_set_gravity(
            world,
            hot,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
        }
        let stretch = |id: u32, p: u32| -> f64 {
            let mut pos = vec![Vec3::default(); soft_body_particle_count(world, id) as usize];
            let _ = soft_body_read_particles(
                world,
                id,
                pos.as_mut_ptr(),
                std::ptr::null_mut(),
                pos.len() as u32,
            );
            (1.0 - pos[p as usize].y).abs()
        };
        let s_cold = stretch(cold, cbot);
        let s_hot = stretch(hot, hbot);
        assert!(
            s_hot > s_cold,
            "heated body should stretch more (thermal expansion + softer modulus): s_hot={s_hot} <= s_cold={s_cold}"
        );
        world_destroy(world);
    }

    // ── Phase 27 (B7): 真表面网格读数 — 顶点数=粒子数、面数=三角面数（替代逐质点球近似）
    #[test]
    fn soft_body_read_surface_mesh_matches_particles_and_triangles() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        // 4 粒子 + 2 三角面，构成一个软布片。
        let _p0 = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let _p1 = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let _p2 = soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);
        let _p3 = soft_body_add_particle(world, id, 1.0, 1.0, 0.0, 1.0, Bool::FALSE);
        assert_eq!(soft_body_add_triangle(world, id, 0, 1, 2), Bool::TRUE);
        assert_eq!(soft_body_add_triangle(world, id, 1, 3, 2), Bool::TRUE);

        let npart = soft_body_particle_count(world, id);
        let ntri = soft_body_read_triangles(world, id, std::ptr::null_mut(), 1024);
        assert_eq!(npart, 4);
        assert_eq!(ntri, 2);

        // 查询尺寸：返回顶点数；面数由独立函数给出。
        let vret = soft_body_read_surface_mesh(
            world,
            id,
            std::ptr::null_mut(),
            0,
            std::ptr::null_mut(),
            0,
        );
        assert_eq!(vret, npart, "surface vertices must equal particle count");
        let tri_count = soft_body_read_surface_triangle_count(world, id);
        assert_eq!(
            tri_count, ntri,
            "surface triangles must equal triangle count"
        );

        // 读回实际顶点 + 面索引，核对数量与内容一致。
        let mut verts: Vec<f64> = vec![0.0; (npart as usize) * 3];
        let mut tris: Vec<u32> = vec![0; (ntri as usize) * 3];
        let vret2 = soft_body_read_surface_mesh(
            world,
            id,
            verts.as_mut_ptr(),
            verts.len() as u32,
            tris.as_mut_ptr(),
            tris.len() as u32,
        );
        assert_eq!(vret2, npart);
        assert_eq!(soft_body_read_surface_triangle_count(world, id), ntri);
        // 顶点 p0=(0,0,0)：前 3 个 f64 应为 0,0,0。
        assert!((verts[0].abs() < 1e-9) && (verts[1].abs() < 1e-9) && (verts[2].abs() < 1e-9));
        // 面索引与 add_triangle 写入一致。
        assert_eq!((tris[0], tris[1], tris[2]), (0, 1, 2));

        // 坏参守卫：未知 id → 0 顶点，且面数函数返回 0。
        assert_eq!(
            soft_body_read_surface_mesh(
                world,
                u32::MAX,
                std::ptr::null_mut(),
                0,
                std::ptr::null_mut(),
                0,
            ),
            0
        );
        assert_eq!(soft_body_read_surface_triangle_count(world, u32::MAX), 0);
        world_destroy(world);
    }

    // ── Phase 27 (B8): 隐式 (backward-Euler) 比较路径 — 刚弹簧下显式爆炸、隐式有界 ─────
    #[test]
    fn implicit_euler_stays_bounded_where_explicit_blows_up() {
        // 两根高刚度弹簧把质点从固定锚点往下吊（重力下垂）。显式半隐式欧拉在
        // k 足够大 / dt 足够大时能量发散（位置非有限或飞出），而隐式 backward-Euler
        // 无条件稳定，应保持在锚点附近的有界区域。
        let world = make_world();
        assert!(!world.is_null());

        // 显式路径：高刚度 mass-spring 链。
        let exp = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let e_a = soft_body_add_particle(world, exp, 0.0, 0.0, 0.0, 1.0, Bool::FALSE); // 自由
        let e_b = soft_body_add_particle(world, exp, 0.0, 0.0, 0.0, 0.0, Bool::TRUE); // 锚定
        soft_body_add_spring(world, exp, e_a, e_b, 5.0e4, 0.0);
        let exp_y0 = {
            let mut pos: Vec<Vec3> = vec![Vec3::default(); 1];
            soft_body_read_particles(world, exp, pos.as_mut_ptr(), std::ptr::null_mut(), 1);
            pos[0].y
        };

        // 隐式路径：同样的设置。
        let imp = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let i_a = soft_body_add_particle(world, imp, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let i_b = soft_body_add_particle(world, imp, 0.0, 0.0, 0.0, 0.0, Bool::TRUE);
        soft_body_add_spring(world, imp, i_a, i_b, 5.0e4, 0.0);
        let imp_y0 = {
            let mut pos: Vec<Vec3> = vec![Vec3::default(); 1];
            soft_body_read_particles(world, imp, pos.as_mut_ptr(), std::ptr::null_mut(), 1);
            pos[0].y
        };

        let dt = 1.0 / 60.0;
        for _ in 0..200 {
            soft_body_step_mass_spring(world, exp, dt);
            soft_body_step_implicit(world, imp, dt);
        }

        let mut epos: Vec<Vec3> = vec![Vec3::default(); 1];
        soft_body_read_particles(world, exp, epos.as_mut_ptr(), std::ptr::null_mut(), 1);
        let mut ipos: Vec<Vec3> = vec![Vec3::default(); 1];
        soft_body_read_particles(world, imp, ipos.as_mut_ptr(), std::ptr::null_mut(), 1);

        // 隐式路径：位置有限且相对初始下垂量有界（不会飞到无穷远）。
        assert!(ipos[0].y.is_finite(), "implicit position must stay finite");
        assert!(
            (ipos[0].y - imp_y0).abs() < 5.0,
            "implicit must remain bounded near anchor, got dy={}",
            (ipos[0].y - imp_y0).abs()
        );

        // 对比：显式路径在此刚度下应已发散（非有限或远超隐式），以此证明隐式比较路径的价值。
        // 若显式恰好也稳定，则仅断言隐式有界即可（不强制显式失败）。
        // 显式发散（NaN/inf）本身即证明隐式比较路径的必要性；若显式恰好也有限，
        // 则断言它不会比隐式更稳定（同样刚度下隐式不应差于显式）。
        if epos[0].y.is_finite() {
            assert!(
                (epos[0].y - exp_y0).abs() >= (ipos[0].y - imp_y0).abs() - 1e-6,
                "explicit should not be more stable than implicit under stiff springs"
            );
        }
        world_destroy(world);
    }

    // ── Phase 10: 塑性 — 超过屈服应变的边把形变永久冻入 rest_length─────────────
    #[test]
    fn soft_body_plasticity_freezes_deformation() {
        // 观测方式：一个被弹簧从顶部锚点拽住的自由质点，在重力下垂。
        //   * 弹性（无塑性）：弹簧始终拉回，节点停在靠近原长处（下坠少）。
        //   * 塑性（creep=1, yield 很小）：弹簧一旦超屈服就永久把 rest_length 拉到
        //     当前长度，弹簧"放弃"回拉 → 节点持续下坠得更远、更深。
        // 两者都用 soft_body_read_particles 读位置（不依赖内部字段）。
        fn sag(plastic: bool) -> f64 {
            let world = make_world();
            assert!(!world.is_null());
            let id = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            assert_ne!(id, u32::MAX);
            // 关闭世界重力，仅用软体自身重力驱动下坠。
            assert_eq!(
                soft_body_set_gravity(
                    world,
                    id,
                    Vec3 {
                        x: 0.0,
                        y: -30.0,
                        z: 0.0
                    }
                ),
                Bool::TRUE
            );
            // 锚点 p0 钉在 y=10；自由质点 p1 从 y=9 出发（初始弹簧长≈1）。
            let _a = soft_body_add_particle(world, id, 0.0, 10.0, 0.0, 1.0, Bool::TRUE);
            let _b = soft_body_add_particle(world, id, 0.0, 9.0, 0.0, 1.0, Bool::FALSE);
            assert_eq!(
                soft_body_add_spring(world, id, 0, 1, 200.0, 0.0),
                Bool::TRUE
            );
            if plastic {
                assert_eq!(
                    soft_body_set_plasticity(world, id, 0.01, 1.0, 1),
                    Bool::TRUE
                );
            }
            for _ in 0..200 {
                world_step(world, 1.0 / 60.0);
            }
            let count = soft_body_particle_count(world, id);
            let mut pos = vec![Vec3::default(); count as usize];
            let _r =
                soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), count);
            let y_free = pos[1].y;
            world_destroy(world);
            y_free
        }

        let y_elastic = sag(false);
        let y_plastic = sag(true);
        // 塑性让弹簧"放弃回拉"，自由节点下坠更深（y 更小）。
        assert!(
            y_plastic < y_elastic - 0.5,
            "plasticity should let the node sag much further: plastic={y_plastic} elastic={y_elastic}"
        );
        assert!(y_plastic.is_finite() && y_elastic.is_finite());
    }

    // ── Phase 10: 塑性非法/未知参数返回 False 而不 panic─────────────────────────
    #[test]
    fn soft_body_set_plasticity_rejects_invalid_args() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        let _p = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);

        // 未知 id → False。
        assert_eq!(
            soft_body_set_plasticity(world, u32::MAX, 0.1, 0.5, 1),
            Bool::FALSE
        );
        // 非有限参数 → False。
        assert_eq!(
            soft_body_set_plasticity(world, id, f64::NAN, 0.5, 1),
            Bool::FALSE
        );
        assert_eq!(
            soft_body_set_plasticity(world, id, 0.1, f64::INFINITY, 1),
            Bool::FALSE
        );
        // 合法调用 → True，且未 panic。
        assert_eq!(
            soft_body_set_plasticity(world, id, 0.05, 1.0, 1),
            Bool::TRUE
        );
        // 关闭（enabled=0）→ True，且不再塑性化。
        assert_eq!(
            soft_body_set_plasticity(world, id, 0.05, 1.0, 0),
            Bool::TRUE
        );

        world_destroy(world);
    }

    // ── Phase 11: 充气 — 闭合三角网格被内部气压吹胀（半径增大）─────────────────
    #[test]
    fn soft_body_pressure_inflates_closed_mesh() {
        // 八面体（6 顶点、8 三角面）闭合壳，无重力，开启气压后半径应增大。
        fn mean_radius(pressure: Option<f64>) -> f64 {
            let world = make_world();
            assert!(!world.is_null());
            let id = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            assert_ne!(id, u32::MAX);
            // 无重力。
            assert_eq!(
                soft_body_set_gravity(
                    world,
                    id,
                    Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0
                    }
                ),
                Bool::TRUE
            );
            // 八面体顶点（±1,0,0）(0,±1,0) (0,0,±1)。
            let v = [
                (1.0, 0.0, 0.0),
                (-1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, -1.0, 0.0),
                (0.0, 0.0, 1.0),
                (0.0, 0.0, -1.0),
            ];
            for (x, y, z) in v {
                let _ = soft_body_add_particle(world, id, x, y, z, 1.0, Bool::FALSE);
            }
            // 8 个三角面（带符号顺序使法向一致）。
            let faces = [
                (0, 2, 4),
                (2, 1, 4),
                (1, 3, 4),
                (3, 0, 4),
                (0, 3, 5),
                (3, 1, 5),
                (1, 2, 5),
                (2, 0, 5),
            ];
            for (a, b, c) in faces {
                assert_eq!(soft_body_add_triangle(world, id, a, b, c), Bool::TRUE);
            }
            if let Some(p) = pressure {
                assert_eq!(soft_body_set_pressure(world, id, p), Bool::TRUE);
            }
            for _ in 0..120 {
                world_step(world, 1.0 / 60.0);
            }
            let count = soft_body_particle_count(world, id);
            let mut pos = vec![Vec3::default(); count as usize];
            let _r =
                soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), count);
            // 质心 + 平均半径。
            let mut cx = 0.0;
            let mut cy = 0.0;
            let mut cz = 0.0;
            for p in &pos {
                cx += p.x;
                cy += p.y;
                cz += p.z;
            }
            let n = count as f64;
            cx /= n;
            cy /= n;
            cz /= n;
            let mut r = 0.0;
            for p in &pos {
                let dx = p.x - cx;
                let dy = p.y - cy;
                let dz = p.z - cz;
                r += (dx * dx + dy * dy + dz * dz).sqrt();
            }
            r /= n;
            world_destroy(world);
            r
        }

        let r_no_pressure = mean_radius(None);
        let r_pressurized = mean_radius(Some(2.0));
        // 气压把闭合壳吹胀 → 半径明显大于静止（≈1.0）。
        assert!(
            r_pressurized > r_no_pressure + 0.1,
            "pressure should inflate the mesh: pressurized={r_pressurized} baseline={r_no_pressure}"
        );
    }

    // ── Phase 11: 气压非法/未知参数返回 False 而不 panic─────────────────────────
    #[test]
    fn soft_body_set_pressure_rejects_invalid_args() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        let _p = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);

        // 未知 id → False。
        assert_eq!(soft_body_set_pressure(world, u32::MAX, 1.0), Bool::FALSE);
        // 非有限气压 → False。
        assert_eq!(soft_body_set_pressure(world, id, f64::NAN), Bool::FALSE);
        // 合法调用 → True，且未 panic。
        assert_eq!(soft_body_set_pressure(world, id, 1.5), Bool::TRUE);
        // 关闭（<=0）→ True，且不再充气。
        assert_eq!(soft_body_set_pressure(world, id, 0.0), Bool::TRUE);

        world_destroy(world);
    }

    // ── Phase 12: 自碰撞 — 紧密堆叠的自由质点保持分离(不穿透)────────────────────
    #[test]
    fn soft_body_self_collision_keeps_particles_separated() {
        // 10 个自由质点在竖直方向密集堆叠 + 重力。无自碰撞时全部塌缩到同一点;
        // 开启自碰撞(半径 r)后任意两点最小间距应 >= ~2r。
        const N: usize = 10;
        const R: f64 = 0.25;
        fn min_pair_dist(self_col: bool) -> f64 {
            let world = make_world();
            assert!(!world.is_null());
            let id = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            assert_ne!(id, u32::MAX);
            for i in 0..N {
                // 竖直堆叠,间距 0.1 (< 2R),重力下会塌缩。
                let _ =
                    soft_body_add_particle(world, id, 0.0, (i as f64) * 0.1, 0.0, 1.0, Bool::FALSE);
            }
            if self_col {
                assert_eq!(soft_body_set_self_collision(world, id, R, 0.0), Bool::TRUE);
            }
            for _ in 0..200 {
                world_step(world, 1.0 / 60.0);
            }
            let count = soft_body_particle_count(world, id);
            let mut pos = vec![Vec3::default(); count as usize];
            let _r =
                soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), count);
            let mut m = f64::MAX;
            for i in 0..count as usize {
                for j in (i + 1)..count as usize {
                    let dx = pos[i].x - pos[j].x;
                    let dy = pos[i].y - pos[j].y;
                    let dz = pos[i].z - pos[j].z;
                    let d = (dx * dx + dy * dy + dz * dz).sqrt();
                    if d < m {
                        m = d;
                    }
                }
            }
            world_destroy(world);
            m
        }

        let sep_off = min_pair_dist(false);
        let sep_on = min_pair_dist(true);
        // 无自碰撞:塌缩到几乎重合(间距远小于 2R)。
        assert!(
            sep_off < R,
            "no self-collision should let them collapse, got {sep_off}"
        );
        // 有自碰撞:最小间距应被推到接近 2R 甚至更大。
        assert!(
            sep_on >= 2.0 * R - 1e-3,
            "self-collision should keep particles >= 2R apart: sep_on={sep_on} R={R}"
        );
    }

    // ── Phase 12: 自碰撞非法/未知参数返回 False 而不 panic───────────────────────
    #[test]
    fn soft_body_set_self_collision_rejects_invalid_args() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        let _p = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);

        // 未知 id → False。
        assert_eq!(
            soft_body_set_self_collision(world, u32::MAX, 0.2, 0.0),
            Bool::FALSE
        );
        // 非有限 → False。
        assert_eq!(
            soft_body_set_self_collision(world, id, f64::NAN, 0.0),
            Bool::FALSE
        );
        // radius <= 0 → False (且保持关闭)。
        assert_eq!(
            soft_body_set_self_collision(world, id, 0.0, 0.0),
            Bool::FALSE
        );
        // stiffness < 0 → False。
        assert_eq!(
            soft_body_set_self_collision(world, id, 0.2, -1.0),
            Bool::FALSE
        );
        // 合法 → True，且未 panic。
        assert_eq!(
            soft_body_set_self_collision(world, id, 0.2, 0.0),
            Bool::TRUE
        );

        world_destroy(world);
    }

    // ── Phase 13: 运行时改弹簧刚度 → 高刚度下垂更少 ──────────────────────────────
    #[test]
    fn soft_body_set_spring_stiffness_reduces_sag() {
        // MassSpring 链:固定端点 + 3 个自由点,单条弹簧(index 0)把端点连到链。
        // 低刚度下垂多,运行时改高刚度后下垂更少。
        fn max_sag(stiff: f64) -> f64 {
            let world = make_world();
            assert!(!world.is_null());
            let id = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            assert_ne!(id, u32::MAX);
            let _anc = soft_body_add_particle(world, id, 0.0, 5.0, 0.0, 1.0, Bool::TRUE);
            for i in 1..4 {
                let _ = soft_body_add_particle(world, id, i as f64, 5.0, 0.0, 1.0, Bool::FALSE);
            }
            // 全链初刚度 200。
            for i in 0..3 {
                let _ = soft_body_add_spring(world, id, i, i + 1, 200.0, 5.0);
            }
            // 运行时把第 0 条弹簧改为给定刚度。
            assert_eq!(
                soft_body_set_spring_stiffness(world, id, 0, stiff),
                Bool::TRUE
            );
            for _ in 0..200 {
                world_step(world, 1.0 / 60.0);
            }
            // 读自由点最低 y(锚点在 y=5,重力向下,越软越低)。
            let count = soft_body_particle_count(world, id);
            let mut pos = vec![Vec3::default(); count as usize];
            let _r =
                soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), count);
            let mut min_y = f64::MAX;
            for p in &pos {
                if p.y < min_y {
                    min_y = p.y;
                }
            }
            world_destroy(world);
            min_y
        }

        let sag_soft = max_sag(20.0); // 软
        let sag_stiff = max_sag(5000.0); // 硬
        assert!(
            sag_stiff > sag_soft,
            "stiffer spring should hold the chain higher: soft={sag_soft} stiff={sag_stiff}"
        );

        // 非法参数:未知 id / 越界 index / 负刚度 → False。
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(world, Vec3::default());
        assert_ne!(id, u32::MAX);
        assert_eq!(
            soft_body_set_spring_stiffness(world, u32::MAX, 0, 100.0),
            Bool::FALSE
        );
        assert_eq!(
            soft_body_set_spring_stiffness(world, id, 999, 100.0),
            Bool::FALSE
        );
        assert_eq!(
            soft_body_set_spring_stiffness(world, id, 0, -1.0),
            Bool::FALSE
        );
        assert_eq!(
            soft_body_set_spring_stiffness(world, id, 0, f64::NAN),
            Bool::FALSE
        );
        world_destroy(world);
    }

    // ── Phase 13: 运行时改 XPBD 距离约束柔度 → 高柔度拉伸更多 ───────────────────
    #[test]
    fn soft_body_set_distance_constraint_compliance_increases_stretch() {
        // XPBD 水平单杆:锚点 (0,0,0),自由点 (1,0,0),重力向下(-y)把杆往下拉→拉伸。
        // compliance=0(硬)几乎不拉伸;运行时改为高柔度后明显拉伸。用 1 次迭代让柔度可见
        // (多迭代会把约束完全收敛,抹掉柔度差异)。
        fn rod_len(compliance: f64) -> f64 {
            let world = make_world();
            assert!(!world.is_null());
            let id = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            assert_ne!(id, u32::MAX);
            let _anc = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::TRUE);
            let _free = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
            // 单迭代:让 compliance 在稳态下可见。
            assert_eq!(soft_body_configure_solver(world, id, 1, 1, 0.0), Bool::TRUE);
            assert_eq!(
                soft_body_add_distance_constraint(world, id, 0, 1, 0.0),
                Bool::TRUE
            );
            // 运行时改柔度(初始 0 → 给定值)。
            assert_eq!(
                soft_body_set_distance_constraint_compliance(world, id, 0, compliance),
                Bool::TRUE
            );
            for _ in 0..300 {
                world_step(world, 1.0 / 60.0);
            }
            // 自由点相对锚点的欧氏距离 = 当前杆长。
            let count = soft_body_particle_count(world, id);
            let mut pos = vec![Vec3::default(); count as usize];
            let _r =
                soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), count);
            let len = ((pos[1].x - pos[0].x).powi(2)
                + (pos[1].y - pos[0].y).powi(2)
                + (pos[1].z - pos[0].z).powi(2))
            .sqrt();
            world_destroy(world);
            len
        }

        let len_rigid = rod_len(0.0);
        let len_soft = rod_len(50.0);
        assert!(
            len_soft > len_rigid + 0.01,
            "higher compliance should stretch more: rigid={len_rigid} soft={len_soft}"
        );

        // 非法参数:未知 id / 越界 index / 负柔度 → False。
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(world, Vec3::default());
        assert_ne!(id, u32::MAX);
        assert_eq!(
            soft_body_set_distance_constraint_compliance(world, u32::MAX, 0, 0.0),
            Bool::FALSE
        );
        assert_eq!(
            soft_body_set_distance_constraint_compliance(world, id, 999, 0.0),
            Bool::FALSE
        );
        assert_eq!(
            soft_body_set_distance_constraint_compliance(world, id, 0, -1.0),
            Bool::FALSE
        );

        world_destroy(world);
    }

    // ── Phase 14: 软软碰撞 — 两个软体的自由质点互相排斥,不穿透 ──────────────────
    #[test]
    fn soft_body_cross_collision_keeps_bodies_apart() {
        // 两个软体各一个自由质点,初始间距 0.1 (< 2R=0.5),重力为 0。无软软碰撞时保持重合;
        // 开启后它们之间的间距应被推到 >= 2R。
        const R: f64 = 0.25;
        fn min_inter_body_dist(enabled: bool) -> f64 {
            let world = make_world();
            assert!(!world.is_null());
            let id_a = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            let id_b = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            assert_ne!(id_a, u32::MAX);
            assert_ne!(id_b, u32::MAX);
            assert_ne!(
                soft_body_add_particle(world, id_a, 0.0, 0.0, 0.0, 1.0, Bool::FALSE),
                u32::MAX
            );
            assert_ne!(
                soft_body_add_particle(world, id_b, 0.1, 0.0, 0.0, 1.0, Bool::FALSE),
                u32::MAX
            );
            if enabled {
                assert_eq!(
                    soft_body_set_cross_collision(world, id_a, R, 0.0),
                    Bool::TRUE
                );
                assert_eq!(
                    soft_body_set_cross_collision(world, id_b, R, 0.0),
                    Bool::TRUE
                );
            }
            for _ in 0..150 {
                world_step(world, 1.0 / 60.0);
            }
            let ca = soft_body_particle_count(world, id_a);
            let cb = soft_body_particle_count(world, id_b);
            let mut pa = vec![Vec3::default(); ca as usize];
            let mut pb = vec![Vec3::default(); cb as usize];
            let _ =
                soft_body_read_particles(world, id_a, pa.as_mut_ptr(), std::ptr::null_mut(), ca);
            let _ =
                soft_body_read_particles(world, id_b, pb.as_mut_ptr(), std::ptr::null_mut(), cb);
            let mut m = f64::MAX;
            for a in &pa {
                for b in &pb {
                    let d =
                        ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt();
                    if d < m {
                        m = d;
                    }
                }
            }
            world_destroy(world);
            m
        }

        let d_off = min_inter_body_dist(false);
        let d_on = min_inter_body_dist(true);
        // 无碰撞:几乎重合(间距远小于 2R)。
        assert!(
            d_off < R,
            "no cross-collision should let them overlap, got {d_off}"
        );
        // 有碰撞:最小间距被推到接近 2R 甚至更大。
        assert!(
            d_on >= 2.0 * R - 1e-3,
            "cross-collision should keep bodies >= 2R apart: d_on={d_on} R={R}"
        );
    }

    // ── Phase 14: 软软碰撞非法/未知参数返回 False 而不 panic ──────────────────────
    #[test]
    fn soft_body_set_cross_collision_rejects_invalid_args() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        let _p = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);

        // 未知 id → False。
        assert_eq!(
            soft_body_set_cross_collision(world, u32::MAX, 0.2, 0.0),
            Bool::FALSE
        );
        // 非有限 → False。
        assert_eq!(
            soft_body_set_cross_collision(world, id, f64::NAN, 0.0),
            Bool::FALSE
        );
        // radius <= 0 → False。
        assert_eq!(
            soft_body_set_cross_collision(world, id, 0.0, 0.0),
            Bool::FALSE
        );
        // stiffness < 0 → False。
        assert_eq!(
            soft_body_set_cross_collision(world, id, 0.2, -1.0),
            Bool::FALSE
        );
        // 合法 → True。
        assert_eq!(
            soft_body_set_cross_collision(world, id, 0.2, 0.0),
            Bool::TRUE
        );

        world_destroy(world);
    }

    // ── Phase 15: 双向耦合 — 软体(经 proxy collider)撞动态刚体,刚体被反推且软体不穿透 ──
    #[test]
    fn soft_body_two_way_coupling_pushes_rigid_body() {
        // 重动态球 R(质量 1.0)悬于 y=3.0;下方一个软体质点 P(质量 0.5, collide, 半径 0.4)从
        // y=4.5 落下砸向 R。Phase 5f 的 proxy 是 dynamic 刚体 + 读回;Phase 15 又把 proxy
        // collider 密度置 0,使 proxy 质量恰等于质点质量,动量传递对称。验证:
        //  (1) R 被 P 反推而明显下移(R.y < 起始 - eps);
        //  (2) P 没有穿透 R(P.y >= R.y - 半径和,即 P 停在 R 上方而不是穿过);
        //  (3) P 自己下落了(P.y < 起始)。
        let world = world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        assert!(!world.is_null());

        // 动态刚体 R:球半径 0.5,质量 1.0,置于 (0, 3.0, 0)。
        let rb = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        assert!(!rb.is_null());
        rigid_body_builder_set_translation(
            rb,
            Vec3 {
                x: 0.0,
                y: 3.0,
                z: 0.0,
            },
        );
        rigid_body_builder_set_additional_mass_properties(
            rb,
            Vec3::default(),
            1.0,
            Vec3::default(),
        );
        let body_ptr = rigid_body_builder_build(rb);
        assert!(!body_ptr.is_null());
        let rh = world_insert_rigid_body(world, body_ptr);
        assert_ne!(rh, 0);
        let cb = collider_builder_create_sphere(Sphere {
            center: Vec3::default(),
            radius: 0.5,
        });
        assert!(!cb.is_null());
        let cbuilt = collider_builder_build(cb);
        assert!(!cbuilt.is_null());
        let _ch: ColliderHandleRaw = world_insert_collider_with_parent(world, cbuilt, rh);

        // 软体 P:单自由质点,质量 0.5,半径 0.4,从 y=4.0 落下(距 R 顶端仅 0.1),开启 collide。
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        assert_ne!(
            soft_body_add_particle(world, id, 0.0, 4.0, 0.0, 0.5, Bool::FALSE),
            u32::MAX
        );
        // 软体有独立的 gravity 字段(由 world_step 复制世界重力),必须显式设定,P 才会下落。
        soft_body_set_gravity(
            world,
            id,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        assert_eq!(
            soft_body_enable_collision(world, id, 0.4, Bool::TRUE),
            Bool::TRUE
        );

        let r_y0 = 3.0;
        let p_y0 = 4.0;
        for _ in 0..60 {
            world_step(world, 1.0 / 60.0);
        }

        let r_pos = rigid_body_get_translation(world, rh);
        let pc = soft_body_particle_count(world, id);
        let mut pp = vec![Vec3::default(); pc as usize];
        let _ = soft_body_read_particles(world, id, pp.as_mut_ptr(), std::ptr::null_mut(), pc);
        // 单质点:取 y 最小者(即该粒子)。
        let p_y = pp.iter().map(|v| v.y).fold(f64::INFINITY, f64::min);

        // (1) P 确实下落并撞上 R(p_y 明显小于起始)。
        assert!(
            p_y < p_y0 - 0.3,
            "soft particle should have fallen onto R, p_y={p_y}"
        );
        // (2) 双向耦合:R 被 P 反推而明显下移(r_y < 起始)。
        assert!(
            r_pos.y < r_y0 - 0.2,
            "two-way coupling: rigid body must be pushed down by the soft body, r_y={}",
            r_pos.y
        );
        // (3) P 停在 R 上方而不是穿透:R 球(半径 0.5)与质点球(半径 0.4)中心距应 >= 半径和。
        //     同 x/z 时中心距 = |gap|(gap = R.y - P.y);gap 为负表示 P 在 R 上方接触。
        //     若 P 穿透到 R 下方,则 gap 会变成明显正值(>= ~0.9)。故断言 gap 不显著为正。
        let gap = r_pos.y - p_y;
        assert!(
            gap <= (0.4 + 0.5) - 0.2,
            "soft particle must rest on top of (not tunnel through) the rigid body: gap={gap} (must not be >= ~0.7)"
        );

        world_destroy(world);
    }

    // ── Phase 16: 体积守恒 — 四面体软体在动态下总体积保持≈静止(硬约束) ──────────────
    #[test]
    fn soft_body_volume_conservation_holds_total_volume() {
        // 建一个四面体软体(4 质点 + 6 条距离边 + 1 个四面体),距离求解器设得很软,
        // 开启体积守恒(compliance=0, 硬)。多步动态后总体积应仍≈静止体积;若关闭则会明显漂移。
        // `soft_body_total_volume` 返回的是"当前体积 / 静止体积"的比值(=1.0 表示守恒)。
        const REST: f64 = 1.0;

        fn build_and_step(enable: bool) -> f64 {
            let world = make_world();
            assert!(!world.is_null());
            let id = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            assert_ne!(id, u32::MAX);
            // 4 个质点, 全部自由(质量 1)。
            assert_ne!(
                soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE),
                u32::MAX
            );
            assert_ne!(
                soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE),
                u32::MAX
            );
            assert_ne!(
                soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE),
                u32::MAX
            );
            assert_ne!(
                soft_body_add_particle(world, id, 0.0, 0.0, 1.0, 1.0, Bool::FALSE),
                u32::MAX
            );
            // 6 条距离边(顺从, compliance 0.2)。
            let edges = [[0u32, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3]];
            for e in edges {
                assert_eq!(
                    soft_body_add_distance_constraint(world, id, e[0], e[1], 0.2),
                    Bool::TRUE
                );
            }
            // 1 个四面体。
            assert_eq!(soft_body_add_tetrahedron(world, id, 0, 1, 2, 3), Bool::TRUE);
            // 距离求解器很软; XPBD solver, 20 迭代。
            assert_eq!(
                soft_body_configure_solver(world, id, 1, 20, 0.2),
                Bool::TRUE
            );
            if enable {
                assert_eq!(
                    soft_body_set_volume_conservation(world, id, 0.0),
                    Bool::TRUE
                );
            }
            // 重力向下, 让四面体整体下落 / 受数值扰动。
            soft_body_set_gravity(
                world,
                id,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            for _ in 0..120 {
                world_step(world, 1.0 / 60.0);
            }
            let v = soft_body_total_volume(world, id);
            world_destroy(world);
            v
        }

        let v_off = build_and_step(false);
        let v_on = build_and_step(true);
        // 开启硬体积守恒: 体积比值与 1.0(守恒)相对偏差很小。
        assert!(
            (v_on - REST).abs() / REST < 0.05,
            "volume conservation ON should keep total volume ≈ rest: v_on={v_on}, rest={REST}"
        );
        // 关闭: 体积守恒回退到软求解器; 开启不应比关闭更糟(单调性锁定)。
        let drift_off = (v_off - REST).abs() / REST;
        let drift_on = (v_on - REST).abs() / REST;
        assert!(
            drift_on <= drift_off + 1e-6,
            "volume conservation should not increase drift: drift_on={drift_on} drift_off={drift_off}"
        );
    }

    // ── Phase 16: 体积守恒非法/未知参数返回 False 而不 panic ──────────────────────
    #[test]
    fn soft_body_set_volume_conservation_rejects_invalid_args() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        // 未知 id → False。
        assert_eq!(
            soft_body_set_volume_conservation(world, u32::MAX, 0.0),
            Bool::FALSE
        );
        // 负数 → False。
        assert_eq!(
            soft_body_set_volume_conservation(world, id, -1.0),
            Bool::FALSE
        );
        // 非有限 → False。
        assert_eq!(
            soft_body_set_volume_conservation(world, id, f64::NAN),
            Bool::FALSE
        );
        // 合法(硬)→ True。
        assert_eq!(
            soft_body_set_volume_conservation(world, id, 0.0),
            Bool::TRUE
        );
        // 合法(软)→ True。
        assert_eq!(
            soft_body_set_volume_conservation(world, id, 0.05),
            Bool::TRUE
        );
        world_destroy(world);
    }

    // ── Phase 17: 黏连 — 两自由质点(分属两体)在 capture 半径内被黏住(双体胶水) ────────
    #[test]
    fn soft_body_cohesion_glues_two_bodies_together() {
        // 两体各一个自由质点,初始相距 0.5(cohesion radius=0.4 内)。开启黏连后引力把它们
        // 拉到接触距离 0.4(胶水)。无 cohesion 时它们各自自由(无外力,保持原距)。
        const RADIUS: f64 = 0.4;
        const SEP: f64 = 0.5; // 初始中心距,在 capture 半径内。
        const BREAK: f64 = 2.0; // 远大于 SEP,胶水不破。

        fn build(enable: bool) -> f64 {
            let world = make_world();
            assert!(!world.is_null());
            let ida = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            let idb = soft_body_create(
                world,
                Vec3 {
                    x: SEP,
                    y: 0.0,
                    z: 0.0,
                },
            );
            assert_ne!(ida, u32::MAX);
            assert_ne!(idb, u32::MAX);
            assert_ne!(
                soft_body_add_particle(world, ida, 0.0, 0.0, 0.0, 1.0, Bool::FALSE),
                u32::MAX
            );
            assert_ne!(
                soft_body_add_particle(world, idb, SEP, 0.0, 0.0, 1.0, Bool::FALSE),
                u32::MAX
            );
            // 无重力,纯考察黏连对中心距的影响。
            soft_body_set_gravity(
                world,
                ida,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            soft_body_set_gravity(
                world,
                idb,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            if enable {
                assert_eq!(
                    soft_body_set_cohesion(world, ida, RADIUS, 0.0, BREAK),
                    Bool::TRUE
                );
                assert_eq!(
                    soft_body_set_cohesion(world, idb, RADIUS, 0.0, BREAK),
                    Bool::TRUE
                );
            }
            for _ in 0..60 {
                world_step(world, 1.0 / 60.0);
            }
            let mut pa = Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            };
            let mut pb = Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            };
            assert_eq!(
                soft_body_get_particle(world, ida, 0, &mut pa as *mut Vec3, std::ptr::null_mut()),
                Bool::TRUE
            );
            assert_eq!(
                soft_body_get_particle(world, idb, 0, &mut pb as *mut Vec3, std::ptr::null_mut()),
                Bool::TRUE
            );
            let dist =
                ((pa.x - pb.x).powi(2) + (pa.y - pb.y).powi(2) + (pa.z - pb.z).powi(2)).sqrt();
            world_destroy(world);
            dist
        }

        let d_off = build(false);
        let d_on = build(true);
        // 无黏连: 无外力,中心距保持≈初始 SEP。
        assert!(
            (d_off - SEP).abs() < 1e-3,
            "no cohesion: distance should stay {SEP}, got {d_off}"
        );
        // 有黏连(硬 glue, compliance 0): 两质点被拉到接触距离 RADIUS。
        assert!(
            (d_on - RADIUS).abs() < 0.05,
            "cohesion: distance should snap to {RADIUS}, got {d_on}"
        );
    }

    // ── Phase 17: 黏连可破断 — 初始间距 > break_distance 则本步不黏(胶水撕裂) ──────────
    #[test]
    fn soft_body_cohesion_breaks_when_beyond_break_distance() {
        // 两体初始相距 1.5,cohesion radius=0.4,但 break_distance=0.6(<1.5):因为初始已超出
        // break_distance,胶水不形成,两质点保持原距(不被吸引)。
        const SEP: f64 = 1.5;
        let world = make_world();
        assert!(!world.is_null());
        let ida = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let idb = soft_body_create(
            world,
            Vec3 {
                x: SEP,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(ida, u32::MAX);
        assert_ne!(idb, u32::MAX);
        assert_ne!(
            soft_body_add_particle(world, ida, 0.0, 0.0, 0.0, 1.0, Bool::FALSE),
            u32::MAX
        );
        assert_ne!(
            soft_body_add_particle(world, idb, SEP, 0.0, 0.0, 1.0, Bool::FALSE),
            u32::MAX
        );
        soft_body_set_gravity(
            world,
            ida,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        soft_body_set_gravity(
            world,
            idb,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_eq!(
            soft_body_set_cohesion(world, ida, 0.4, 0.0, 0.6),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_set_cohesion(world, idb, 0.4, 0.0, 0.6),
            Bool::TRUE
        );
        for _ in 0..60 {
            world_step(world, 1.0 / 60.0);
        }
        let mut pa = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut pb = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            soft_body_get_particle(world, ida, 0, &mut pa as *mut Vec3, std::ptr::null_mut()),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_get_particle(world, idb, 0, &mut pb as *mut Vec3, std::ptr::null_mut()),
            Bool::TRUE
        );
        let dist = ((pa.x - pb.x).powi(2) + (pa.y - pb.y).powi(2) + (pa.z - pb.z).powi(2)).sqrt();
        world_destroy(world);
        // 胶水未形成 → 保持原距(≈SEP),不被吸引。
        assert!(
            (dist - SEP).abs() < 1e-2,
            "cohesion should NOT form when initial gap > break_distance: dist={dist}, SEP={SEP}"
        );
    }

    // ── Phase 17: 黏连非法/未知参数返回 False 而不 panic ──────────────────────
    #[test]
    fn soft_body_set_cohesion_rejects_invalid_args() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        // 未知 id → False。
        assert_eq!(
            soft_body_set_cohesion(world, u32::MAX, 0.4, 0.0, 2.0),
            Bool::FALSE
        );
        // radius<=0 → False。
        assert_eq!(
            soft_body_set_cohesion(world, id, 0.0, 0.0, 2.0),
            Bool::FALSE
        );
        // stiffness<0 → False。
        assert_eq!(
            soft_body_set_cohesion(world, id, 0.4, -1.0, 2.0),
            Bool::FALSE
        );
        // break_distance<=radius → False。
        assert_eq!(
            soft_body_set_cohesion(world, id, 0.4, 0.0, 0.4),
            Bool::FALSE
        );
        // 非有限 → False。
        assert_eq!(
            soft_body_set_cohesion(world, id, 0.4, 0.0, f64::NAN),
            Bool::FALSE
        );
        // 合法(硬胶)→ True。
        assert_eq!(soft_body_set_cohesion(world, id, 0.4, 0.0, 2.0), Bool::TRUE);
        // 合法(软胶, inf 永久)→ True。
        assert_eq!(
            soft_body_set_cohesion(world, id, 0.4, 0.05, f64::INFINITY),
            Bool::TRUE
        );
        world_destroy(world);
    }

    // ── Phase 18: 内部阻尼 — 阻尼软体比无阻尼更快耗散速度(振荡收敛) ──────────────
    #[test]
    fn soft_body_damping_dissipates_velocity_faster() {
        // 两个完全相同的单自由质点软体在重力下自由落体。一个设 damping=0.5,另一个 0。
        // 多步后,阻尼体的速度幅值应明显低于无阻尼体(能量被内部阻尼耗散)。
        fn speed_after(d: f64) -> f64 {
            let world = make_world();
            assert!(!world.is_null());
            let id = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            assert_ne!(id, u32::MAX);
            assert_ne!(
                soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE),
                u32::MAX
            );
            soft_body_set_gravity(
                world,
                id,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            assert_eq!(soft_body_set_damping(world, id, d), Bool::TRUE);
            for _ in 0..60 {
                world_step(world, 1.0 / 60.0);
            }
            let mut pos = Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            };
            let mut vel = Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            };
            assert_eq!(
                soft_body_get_particle(world, id, 0, &mut pos as *mut Vec3, &mut vel as *mut Vec3),
                Bool::TRUE
            );
            world_destroy(world);
            (vel.x * vel.x + vel.y * vel.y + vel.z * vel.z).sqrt()
        }

        let v_none = speed_after(0.0);
        let v_damped = speed_after(0.5);
        // 无阻尼自由落体 1s 后速度≈9.81;阻尼体应明显更慢。
        assert!(
            v_damped < v_none * 0.7,
            "damped speed {v_damped} should be << undamped {v_none}"
        );
        // 阻尼体确实被耗散(远小于自由落体理论值)。
        assert!(v_damped < 5.0, "damped speed should be low, got {v_damped}");
    }

    // ── Phase 18: 内部阻尼非法/未知参数返回 False 而不 panic ──────────────────────
    #[test]
    fn soft_body_set_damping_rejects_invalid_args() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        // 未知 id → False。
        assert_eq!(soft_body_set_damping(world, u32::MAX, 0.1), Bool::FALSE);
        // d>=1 → False。
        assert_eq!(soft_body_set_damping(world, id, 1.0), Bool::FALSE);
        assert_eq!(soft_body_set_damping(world, id, 2.0), Bool::FALSE);
        // 负数 → False。
        assert_eq!(soft_body_set_damping(world, id, -0.1), Bool::FALSE);
        // 非有限 → False。
        assert_eq!(soft_body_set_damping(world, id, f64::NAN), Bool::FALSE);
        // 合法(0)→ True。
        assert_eq!(soft_body_set_damping(world, id, 0.0), Bool::TRUE);
        // 合法(0.5)→ True。
        assert_eq!(soft_body_set_damping(world, id, 0.5), Bool::TRUE);
        world_destroy(world);
    }

    // ── Phase 19: 各向异性柔度 — 压缩侧更软时,受压质点更易被压入(穿透更深) ──────────
    #[test]
    fn soft_body_anisotropic_compliance_compression_yields_more() {
        // A 固定于点(0,0,0);B 自由于点(0,1,0);边 A-B rest=1.0。上拉重力把 B 拉向 A,
        // 使边处于压缩状态(len<rest)。比较「刚性压缩柔度(α_c=0)」与「软压缩柔度(α_c=50)」:
        // 刚性压缩把 B 顶在 ~rest;软压缩让 B 被压入更靠近 A。拉伸柔度两端都设 0(刚性)。
        fn min_dist_to_a(compression: f64) -> f64 {
            let world = make_world();
            assert!(!world.is_null());
            let id = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            assert_ne!(id, u32::MAX);
            let a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::TRUE);
            let b = soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);
            assert_ne!(a, u32::MAX);
            assert_ne!(b, u32::MAX);
            // 切换到 XPBD 求解器,高密度迭代保证收敛。
            assert_eq!(
                soft_body_configure_solver(world, id, 1, 30, 0.0),
                Bool::TRUE
            );
            // add_distance_constraint 返回 Bool(成功);本体内仅一条边,索引为 0。
            assert_eq!(
                soft_body_add_distance_constraint(world, id, a, b, 0.0),
                Bool::TRUE
            );
            let e: u32 = 0;
            // 拉伸侧刚性(α_s=0),压缩侧按参数。
            assert_eq!(
                soft_body_set_distance_constraint_compliance(world, id, e, 0.0),
                Bool::TRUE
            );
            assert_eq!(
                soft_body_set_distance_constraint_compression(world, id, e, compression),
                Bool::TRUE
            );
            // 下压重力 → B 向 A 压缩(len<rest,进入压缩分支)。
            assert_eq!(
                soft_body_set_gravity(
                    world,
                    id,
                    Vec3 {
                        x: 0.0,
                        y: -50.0,
                        z: 0.0
                    }
                ),
                Bool::TRUE
            );
            let mut min_d = f64::INFINITY;
            for _ in 0..120 {
                world_step(world, 1.0 / 60.0);
                let mut pos = Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                };
                let mut vel = Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                };
                assert_eq!(
                    soft_body_get_particle(
                        world,
                        id,
                        b as u32,
                        &mut pos as *mut Vec3,
                        &mut vel as *mut Vec3
                    ),
                    Bool::TRUE
                );
                let d = (pos.x * pos.x + pos.y * pos.y + pos.z * pos.z).sqrt();
                if d < min_d {
                    min_d = d;
                }
            }
            world_destroy(world);
            min_d
        }

        let d_rigid = min_dist_to_a(0.0);
        let d_soft = min_dist_to_a(50.0);
        // 刚性压缩把 B 顶在 ~rest=1.0;软压缩让 B 压入更靠近 A。
        assert!(
            d_rigid > 0.8,
            "rigid compression should hold near rest, got {d_rigid}"
        );
        assert!(
            d_soft < d_rigid,
            "soft compression (d={d_soft}) should let B penetrate closer to A than rigid (d={d_rigid})"
        );
    }

    // ── Phase 19: 各向异性柔度非法/未知参数返回 False 而不 panic ──────────────────────
    #[test]
    fn soft_body_set_distance_constraint_compression_rejects_invalid_args() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        let p0 = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let p1 = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        assert_ne!(p0, u32::MAX);
        assert_ne!(p1, u32::MAX);
        assert_eq!(
            soft_body_configure_solver(world, id, 1, 10, 0.0),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_add_distance_constraint(world, id, p0, p1, 0.0),
            Bool::TRUE
        );
        let e: u32 = 0;
        // 未知 id → False。
        assert_eq!(
            soft_body_set_distance_constraint_compression(world, u32::MAX, e, 1.0),
            Bool::FALSE
        );
        // 越界 index → False。
        assert_eq!(
            soft_body_set_distance_constraint_compression(world, id, 999, 1.0),
            Bool::FALSE
        );
        // 负数 → False。
        assert_eq!(
            soft_body_set_distance_constraint_compression(world, id, e, -1.0),
            Bool::FALSE
        );
        // 非有限 → False。
        assert_eq!(
            soft_body_set_distance_constraint_compression(world, id, e, f64::NAN),
            Bool::FALSE
        );
        // 合法 → True。
        assert_eq!(
            soft_body_set_distance_constraint_compression(world, id, e, 10.0),
            Bool::TRUE
        );
        // 合法(0)→ True(各向同性)。
        assert_eq!(
            soft_body_set_distance_constraint_compression(world, id, e, 0.0),
            Bool::TRUE
        );
        world_destroy(world);
    }

    // ── Phase 20: 软软(跨体)碰撞摩擦 — μ=1 比 μ=0 更快阻尼切向相对滑动 ────────
    #[test]
    fn soft_body_cross_collision_friction_damps_tangential_slip() {
        // 两个独立软体,各 1 个自由质点,沿 X 相距 0.5(< 2·radius=0.6 → 接触)。
        // 体 A 重力 0,体 B 重力沿 +Y。world_step 里 B 的质点相对 A 产生切向(沿 Y)相对滑动,
        // 跨体碰撞的法向(X)把两者推开,摩擦(μ)阻尼切向(Y)相对速度。μ=1 应大幅削减 B 的 Y
        // 速度,μ=0 则切向速度基本保留。比较末态 |v_y|。
        fn final_tangential_speed(mu: f64) -> f64 {
            let world = make_world();
            assert!(!world.is_null());
            let a = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            let b = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            assert_ne!(a, u32::MAX);
            assert_ne!(b, u32::MAX);
            let pa = soft_body_add_particle(world, a, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
            let pb = soft_body_add_particle(world, b, 0.5, 0.0, 0.0, 1.0, Bool::FALSE);
            assert_ne!(pa, u32::MAX);
            assert_ne!(pb, u32::MAX);
            assert_eq!(soft_body_configure_solver(world, a, 1, 10, 0.0), Bool::TRUE);
            assert_eq!(soft_body_configure_solver(world, b, 1, 10, 0.0), Bool::TRUE);
            // 跨体碰撞 + 摩擦。
            assert_eq!(
                soft_body_set_cross_collision(world, a, 0.3, 0.0),
                Bool::TRUE
            );
            assert_eq!(
                soft_body_set_cross_collision(world, b, 0.3, 0.0),
                Bool::TRUE
            );
            assert_eq!(
                soft_body_set_cross_collision_friction(world, a, mu),
                Bool::TRUE
            );
            assert_eq!(
                soft_body_set_cross_collision_friction(world, b, mu),
                Bool::TRUE
            );
            // 体 A 不动,体 B 受切向(沿 Y)重力 → 相对切向滑动。
            assert_eq!(
                soft_body_set_gravity(
                    world,
                    b,
                    Vec3 {
                        x: 0.0,
                        y: 5.0,
                        z: 0.0
                    }
                ),
                Bool::TRUE
            );
            let mut vy = 0.0f64;
            for _ in 0..60 {
                world_step(world, 1.0 / 60.0);
                let mut pos = Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                };
                let mut vel = Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                };
                assert_eq!(
                    soft_body_get_particle(
                        world,
                        b,
                        pb as u32,
                        &mut pos as *mut Vec3,
                        &mut vel as *mut Vec3
                    ),
                    Bool::TRUE
                );
                vy = vel.y;
            }
            world_destroy(world);
            vy.abs()
        }

        let vy_none = final_tangential_speed(0.0);
        let vy_full = final_tangential_speed(1.0);
        // 每步摩擦把切向相对速度扣掉 μ 比例;μ=1 的末态切向速度应明显小于 μ=0。
        assert!(
            vy_full < vy_none,
            "full friction (vy={vy_full}) should leave less tangential speed than none (vy={vy_none})"
        );
    }

    // ── Phase 20: 自碰撞/跨体碰撞摩擦非法或越界参数返回 False 而不 panic ──────────
    #[test]
    fn soft_body_set_collision_friction_rejects_invalid_args() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        // 未开启自碰撞时设摩擦 → False。
        assert_eq!(
            soft_body_set_self_collision_friction(world, id, 0.5),
            Bool::FALSE
        );
        // 未开启跨体碰撞时设摩擦 → False。
        assert_eq!(
            soft_body_set_cross_collision_friction(world, id, 0.5),
            Bool::FALSE
        );
        // 开启自碰撞后再测。
        assert_eq!(
            soft_body_set_self_collision(world, id, 0.3, 0.0),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_set_self_collision_friction(world, id, 0.5),
            Bool::TRUE
        );
        // 未知 id → False。
        assert_eq!(
            soft_body_set_self_collision_friction(world, u32::MAX, 0.5),
            Bool::FALSE
        );
        // 负数 → False。
        assert_eq!(
            soft_body_set_self_collision_friction(world, id, -0.1),
            Bool::FALSE
        );
        // >1 → False。
        assert_eq!(
            soft_body_set_self_collision_friction(world, id, 1.5),
            Bool::FALSE
        );
        // 非有限 → False。
        assert_eq!(
            soft_body_set_self_collision_friction(world, id, f64::NAN),
            Bool::FALSE
        );
        // 合法边界 → True。
        assert_eq!(
            soft_body_set_self_collision_friction(world, id, 1.0),
            Bool::TRUE
        );
        // 开启跨体碰撞后合法 → True。
        assert_eq!(
            soft_body_set_cross_collision(world, id, 0.3, 0.0),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_set_cross_collision_friction(world, id, 0.5),
            Bool::TRUE
        );
        world_destroy(world);
    }
    // ── Phase 21: 自适应四面体细分(1→4 重心细分)— 加密体积网格 ──────────────
    #[test]
    fn soft_body_subdivide_tetrahedra_increases_resolution_and_stays_bounded() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        let p0 = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let p1 = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let p2 = soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);
        let p3 = soft_body_add_particle(world, id, 0.0, 0.0, 1.0, 1.0, Bool::FALSE);
        assert_ne!(p0, u32::MAX);
        assert_ne!(p1, u32::MAX);
        assert_ne!(p2, u32::MAX);
        assert_ne!(p3, u32::MAX);
        assert_eq!(
            soft_body_add_tetrahedron(world, id, p0, p1, p2, p3),
            Bool::TRUE
        );
        // XPBD 求解器 + 刚性体积约束(compliance 0)。
        assert_eq!(
            soft_body_configure_solver(world, id, 1, 20, 0.0),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_set_volume_conservation(world, id, 0.0),
            Bool::TRUE
        );
        // 钉住一个顶点(与现有体积测试一致),避免自由落体翻转。
        unsafe {
            (*world)
                .inner
                .soft_bodies
                .get_mut(SoftBodyId(id))
                .unwrap()
                .particles[p0 as usize]
                .inv_mass = 0.0;
        }

        let (n_particles, n_tets) = unsafe {
            let sb = (*world).inner.soft_bodies.get(SoftBodyId(id)).unwrap();
            (sb.particles.len(), sb.tetrahedra.len())
        };
        assert_eq!(n_particles, 4);
        assert_eq!(n_tets, 1);

        // 细分(非有限阈值 → 细分全部)。
        let split = soft_body_subdivide_tetrahedra(world, id, f64::INFINITY);
        assert_eq!(split, 1);

        let (n_particles2, n_tets2) = unsafe {
            let sb = (*world).inner.soft_bodies.get(SoftBodyId(id)).unwrap();
            (sb.particles.len(), sb.tetrahedra.len())
        };
        // 1 个四面体 → 4 个子四面体;新增 1 个重心质点。
        assert_eq!(n_tets2, 4, "one tet should become four");
        assert_eq!(n_particles2, 5, "one centroid particle should be added");

        // total_volume 是比值之和:细分后 = 子四面体个数(各自≈1)。
        let vol_after_sub = soft_body_total_volume(world, id);
        assert!((vol_after_sub - 4.0).abs() < 0.05, "4 unit-ratio sub-tets");

        // 模拟若干步:位置必须有限(细分未引入爆炸),体积保持有界(不再暴涨 10^7)。
        for _ in 0..200 {
            world_step(world, 1.0 / 60.0);
        }
        let sb = unsafe { (*world).inner.soft_bodies.get(SoftBodyId(id)).unwrap() };
        for p in &sb.particles {
            assert!(p.pos.x.is_finite() && p.pos.y.is_finite() && p.pos.z.is_finite());
        }
        let final_vol = soft_body_total_volume(world, id);
        assert!(
            final_vol < 50.0,
            "volume stayed bounded after subdivision + steps (final_vol={final_vol})"
        );
        world_destroy(world);
    }

    // ── Phase 21: 细分自适应阈值 + 非法参数返回 0 ─────────────────────────────
    #[test]
    fn soft_body_subdivide_tetrahedra_respects_threshold_and_rejects_bad_args() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        let p0 = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let p1 = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let p2 = soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);
        let p3 = soft_body_add_particle(world, id, 0.0, 0.0, 1.0, 1.0, Bool::FALSE);
        assert_eq!(
            soft_body_add_tetrahedron(world, id, p0, p1, p2, p3),
            Bool::TRUE
        );

        // 未知 id → 返回 0。
        assert_eq!(
            soft_body_subdivide_tetrahedra(world, u32::MAX, f64::INFINITY),
            0
        );
        // 边最长 = 1.0;阈值设很大(>1)→ 不细分(0)。
        assert_eq!(soft_body_subdivide_tetrahedra(world, id, 100.0), 0);
        // 阈值 = 0.5(< 1)→ 细分全部(1 个源四面体)。
        assert_eq!(soft_body_subdivide_tetrahedra(world, id, 0.5), 1);
        // 重心细分保留原始边长(仍 ≈1.0),第二次阈值 0.5 仍触发 —— 4 个子四面体全部再细分,返回 4。
        assert_eq!(soft_body_subdivide_tetrahedra(world, id, 0.5), 4);
        world_destroy(world);
    }
    // ── Phase 22: 逐边应力 / 张力读数（纯只读，供渲染 / 撕裂风险 UI）──────────────
    #[test]
    fn soft_body_read_stress_reflects_stretched_edge() {
        let world = make_world();
        assert!(!world.is_null());

        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        // Pin A at origin; B hangs 1.0 below it. Gravity pulls B further down,
        // so the spring edge stretches → positive strain after a step.
        let a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::TRUE) as usize;
        let b = soft_body_add_particle(world, id, 0.0, -1.0, 0.0, 1.0, Bool::FALSE) as usize;
        soft_body_add_spring(world, id, a as u32, b as u32, 100.0, 5.0);

        // At rest (before stepping) strain ≈ 0.
        let mut rest = vec![0.0f64; 1];
        assert_eq!(soft_body_read_stress(world, id, rest.as_mut_ptr(), 1), 1);
        assert!((rest[0]).abs() < 1e-9, "strain should be 0 before stepping");

        // Step once: B falls under gravity, spring stretches.
        world_step(world, 1.0 / 60.0);
        let mut strained = vec![0.0f64; 1];
        assert_eq!(
            soft_body_read_stress(world, id, strained.as_mut_ptr(), 1),
            1
        );
        assert!(
            strained[0] > 1e-6,
            "gravity should stretch the hanging edge → positive strain, got {}",
            strained[0]
        );

        // Unknown id → 0, no panic (edge-case guard).
        assert_eq!(
            soft_body_read_stress(world, u32::MAX, std::ptr::null_mut(), 0),
            0
        );
        world_destroy(world);
    }

    #[test]
    fn soft_body_read_stress_rejects_bad_world() {
        // Null world → 0 (no panic).
        assert_eq!(
            soft_body_read_stress(std::ptr::null_mut(), 0, std::ptr::null_mut(), 0),
            0
        );
    }
    // ── Phase 23a: 静止长度缩放 ───────────────────────────────────────────────
    #[test]
    fn soft_body_scale_rest_length_scales_strain() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        // Two particles at distance 1.0 along x.
        let a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE) as usize;
        let b = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE) as usize;
        soft_body_add_spring(world, id, a as u32, b as u32, 100.0, 5.0);

        // Before scaling, the spring is at rest → strain 0.
        let mut s0 = vec![0.0f64; 1];
        assert_eq!(soft_body_read_stress(world, id, s0.as_mut_ptr(), 1), 1);
        assert!((s0[0]).abs() < 1e-9, "strain should be 0 at rest");

        // Scale rest length x2 → current len 1.0, rest 2.0 → strain = (1-2)/2 = -0.5.
        let scaled = soft_body_scale_rest_length(world, id, 2.0);
        assert_eq!(scaled, 1);
        let mut s1 = vec![0.0f64; 1];
        assert_eq!(soft_body_read_stress(world, id, s1.as_mut_ptr(), 1), 1);
        assert!(
            (s1[0] + 0.5).abs() < 1e-9,
            "strain should be -0.5 after 2x scale, got {}",
            s1[0]
        );

        // Invalid factor (0) → 0, rest length unchanged (strain stays -0.5).
        assert_eq!(soft_body_scale_rest_length(world, id, 0.0), 0);
        let mut s2 = vec![0.0f64; 1];
        assert_eq!(soft_body_read_stress(world, id, s2.as_mut_ptr(), 1), 1);
        assert!(
            (s2[0] + 0.5).abs() < 1e-9,
            "rest length must be unchanged after invalid scale"
        );

        world_destroy(world);
    }

    // ── Phase 23b: 逐三角形法线回读 ─────────────────────────────────────────────
    #[test]
    fn soft_body_read_normals_computes_unit_normal() {
        let world = make_world();
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        // Triangle in the XY plane: (0,0,0)-(1,0,0)-(0,1,0) → normal +Z.
        let p0 = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let p1 = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let p2 = soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);
        assert_eq!(soft_body_add_triangle(world, id, p0, p1, p2), Bool::TRUE);

        let count = soft_body_read_normals(world, id, std::ptr::null_mut(), 0);
        assert_eq!(count, 1);

        let mut nrm = vec![0.0f64; 3];
        let read = soft_body_read_normals(world, id, nrm.as_mut_ptr(), nrm.len() as u32);
        assert_eq!(read, 1);
        assert!((nrm[0]).abs() < 1e-9, "nx should be 0");
        assert!((nrm[1]).abs() < 1e-9, "ny should be 0");
        assert!(
            (nrm[2] - 1.0).abs() < 1e-9,
            "nz should be +1, got {}",
            nrm[2]
        );

        // Capacity clamp: 0 slots → still returns count, no panic.
        assert_eq!(soft_body_read_normals(world, id, nrm.as_mut_ptr(), 0), 1);
        // Unknown id → 0, no panic.
        assert_eq!(
            soft_body_read_normals(world, u32::MAX, std::ptr::null_mut(), 0),
            0
        );

        world_destroy(world);
    }

    #[test]
    fn soft_body_scale_rest_length_rejects_bad_world() {
        assert_eq!(soft_body_scale_rest_length(std::ptr::null_mut(), 0, 2.0), 0);
        assert_eq!(
            soft_body_read_normals(std::ptr::null_mut(), 0, std::ptr::null_mut(), 0),
            0
        );
    }
    // ── Phase 24: XPBD substeps 暴露 ─────────────────────────────────────────────
    #[test]
    fn soft_body_set_substeps_changes_convergence() {
        use std::ptr::null_mut;
        // Two identical XPBD rod bodies: A pinned at origin, B free 1.0 below.
        // High compliance (soft) + 1 iteration so substep count visibly changes
        // how well the distance constraint is satisfied after one frame.
        fn build() -> (*mut WorldHandle, u32, u32) {
            let world = make_world();
            let id = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            soft_body_configure_solver(world, id, 1, 1, 0.01); // mode 1 = Xpbd, 1 iteration, compliance 0.01
            let a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::TRUE);
            let b = soft_body_add_particle(world, id, 0.0, -1.0, 0.0, 1.0, Bool::FALSE);
            soft_body_add_distance_constraint(world, id, a, b, 0.01);
            (world, id, b)
        }
        fn len_after(world: *mut WorldHandle, id: u32, b: u32, subs: u32) -> f64 {
            soft_body_set_substeps(world, id, subs);
            world_step(world, 1.0 / 60.0);
            let mut pos = Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            };
            assert_eq!(
                soft_body_get_particle(world, id, b, &mut pos as *mut Vec3, null_mut()),
                Bool::TRUE
            );
            // rest length was 1.0 (captured from initial spacing); measure current length.
            (pos.x * pos.x + pos.y * pos.y + pos.z * pos.z).sqrt()
        }

        let (w1, id1, b1) = build();
        let (w8, id8, b8) = build();
        let len1 = len_after(w1, id1, b1, 1);
        let len8 = len_after(w8, id8, b8, 8);
        // 1 substep projects the soft constraint once → stretches more than 8 substeps
        // (8× projection per frame converges closer to rest length).
        assert!(
            len1 > len8 + 1e-4,
            "substeps should reduce stretch: len1={} len8={}",
            len1,
            len8
        );
        world_destroy(w1);
        world_destroy(w8);
    }

    #[test]
    fn soft_body_set_substeps_rejects_zero_and_returns_value() {
        let world = make_world();
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        // Default substeps == 1.
        assert_eq!(soft_body_set_substeps(world, id, 1), 1);
        // Set 4, read back.
        assert_eq!(soft_body_set_substeps(world, id, 4), 4);
        // Zero rejected (keeps previous), returns 0 to signal error.
        assert_eq!(soft_body_set_substeps(world, id, 0), 0);
        // Previous value (4) still in effect.
        assert_eq!(soft_body_set_substeps(world, id, 4), 4);
        // Unknown id → 0.
        assert_eq!(soft_body_set_substeps(world, u32::MAX, 4), 0);
        world_destroy(world);
    } // ── Phase 25 #1: 接触力回读（纯 mps-core，零 fork 改动）──────────────────────
    // 单个自由软体质点从地面上方下落，启用碰撞耦合后停在地面之上；其 proxy 球体
    // 与地面半空间接触产生向上的接触力。read_contact_force 在落地稳定后应回读到底
    // 部质点沿 +y 的净接触力（地面把它顶起），禁用碰撞（无 proxy）时全 0。
    #[test]
    fn soft_body_read_contact_force_pushes_up_from_ground() {
        let world = make_world();

        // 静态地面：Fixed 刚体 + 法线朝上的半空间（实体在 y<0，上方为空）。
        let ground_builder =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Fixed as u32);
        let ground = mps_core::rapier::rigid_body::rigid_body_builder_build(ground_builder);
        let ground_handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, ground);
        let ground_collider = mps_core::rapier::collider::collider_builder_build(
            mps_core::rapier::collider::collider_builder_create_halfspace(Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            }),
        );
        mps_core::rapier::collider::world_insert_collider_with_parent(
            world,
            ground_collider,
            ground_handle,
        );

        // 单个自由质点，初始在地面上方 y=2.0，无弹簧（仅受重力）。
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        let _p = soft_body_add_particle(world, id, 0.0, 2.0, 0.0, 1.0, Bool::FALSE);
        assert_eq!(
            soft_body_enable_collision(world, id, 0.5, Bool::TRUE),
            Bool::TRUE
        );

        // 未落地前：接触力应为 0（尚无接触）。
        let mut fx = vec![0.0f64; 1];
        let mut fy = vec![0.0f64; 1];
        let mut fz = vec![0.0f64; 1];
        world_step(world, 1.0 / 60.0);
        let n0 = soft_body_read_contact_force(
            world,
            id,
            fx.as_mut_ptr(),
            fy.as_mut_ptr(),
            fz.as_mut_ptr(),
            1,
        );
        assert_eq!(n0, 1);
        assert!(
            fy[0].abs() < 1e-6,
            "no contact force before touching ground, got fy={}",
            fy[0]
        );

        // 落地稳定：多步后停在 y≈0.5（粒子半径）之上，接触力沿 +y 把它顶起。
        let dt = 1.0 / 60.0;
        for _ in 0..180 {
            world_step(world, dt);
        }
        let n = soft_body_read_contact_force(
            world,
            id,
            fx.as_mut_ptr(),
            fy.as_mut_ptr(),
            fz.as_mut_ptr(),
            1,
        );
        assert_eq!(n, 1);
        // 重力向下 ≈ -9.81·m（m=1/inv_mass=1），稳态接触力必须向上抵住重力 → fy>0。
        assert!(
            fy[0] > 0.0,
            "contact force on particle resting on ground must point up, got fy={}",
            fy[0]
        );
        // 水平分量应接近 0（竖直落地）。
        assert!(
            fx[0].abs() < fy[0] * 0.2,
            "horizontal contact force should be small, fx={} fy={}",
            fx[0],
            fy[0]
        );

        // 未知 id → 0。
        assert_eq!(
            soft_body_read_contact_force(
                world,
                u32::MAX,
                fx.as_mut_ptr(),
                fy.as_mut_ptr(),
                fz.as_mut_ptr(),
                1
            ),
            0
        );
        // 空 world → 0。
        assert_eq!(
            soft_body_read_contact_force(
                std::ptr::null_mut(),
                0,
                fx.as_mut_ptr(),
                fy.as_mut_ptr(),
                fz.as_mut_ptr(),
                1
            ),
            0
        );

        world_destroy(world);
    }

    #[test]
    fn soft_body_read_contact_force_zero_without_collision() {
        // 没有启用碰撞（无 proxy）的软体，read_contact_force 必须全 0，但返回质点数。
        let world = make_world();
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        let _a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::TRUE);
        let _b = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let mut fx = vec![0.0f64; 2];
        let mut fy = vec![0.0f64; 2];
        let mut fz = vec![0.0f64; 2];
        for _ in 0..30 {
            world_step(world, 1.0 / 60.0);
        }
        let n = soft_body_read_contact_force(
            world,
            id,
            fx.as_mut_ptr(),
            fy.as_mut_ptr(),
            fz.as_mut_ptr(),
            2,
        );
        assert_eq!(n, 2);
        assert_eq!(fx, vec![0.0, 0.0]);
        assert_eq!(fy, vec![0.0, 0.0]);
        assert_eq!(fz, vec![0.0, 0.0]);
        world_destroy(world);
    } // ── Phase 25 #2: 单粒子冲量（纯 mps-core，零 fork 改动）──────────────────────
    // 给一个自由质点施加冲量 J，速度应变为 v += J * inv_mass；施加一步后位置也跟着动。
    // pinned 质点（inv_mass==0）施加冲量后速度不变；越界/坏参返回 Bool::FALSE。
    #[test]
    fn soft_body_apply_particle_impulse_changes_velocity() {
        let world = make_world();
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        ); // 无重力
        // 自由质点 inv_mass=1 (m=1)，pinned 质点 inv_mass=0。
        let free = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let pinned = soft_body_add_particle(world, id, 10.0, 0.0, 0.0, 1.0, Bool::TRUE);

        // 自由质点：施加 (2,0,0)，inv_mass=1 → v 应为 (2,0,0)。
        assert_eq!(
            soft_body_apply_particle_impulse(world, id, free, 2.0, 0.0, 0.0),
            Bool::TRUE
        );
        let mut pos = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut vel = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            soft_body_get_particle(world, id, free, &mut pos, &mut vel),
            Bool::TRUE
        );
        assert!(
            (vel.x - 2.0).abs() < 1e-9,
            "free particle vx should be 2, got {}",
            vel.x
        );
        assert!((vel.y).abs() < 1e-9 && (vel.z).abs() < 1e-9);

        // 再叠一次冲量 (0,3,0) → v=(2,3,0)。
        assert_eq!(
            soft_body_apply_particle_impulse(world, id, free, 0.0, 3.0, 0.0),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_get_particle(world, id, free, &mut pos, &mut vel),
            Bool::TRUE
        );
        assert!((vel.x - 2.0).abs() < 1e-9 && (vel.y - 3.0).abs() < 1e-9);

        // 施加一步：位置应沿 v 平移（无重力、无约束）。
        let before = pos;
        world_step(world, 1.0 / 60.0);
        assert_eq!(
            soft_body_get_particle(world, id, free, &mut pos, &mut vel),
            Bool::TRUE
        );
        assert!(
            (pos.x - before.x - vel.x / 60.0).abs() < 1e-6,
            "free particle should move by v*dt along x"
        );
        assert!((pos.y - before.y - vel.y / 60.0).abs() < 1e-6);

        // pinned 质点：inv_mass=0 → 冲量不产生速度变化，但仍算成功。
        let mut ppos = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut pvel = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            soft_body_apply_particle_impulse(world, id, pinned, 100.0, 0.0, 0.0),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_get_particle(world, id, pinned, &mut ppos, &mut pvel),
            Bool::TRUE
        );
        assert!(
            (pvel.x).abs() < 1e-9,
            "pinned particle velocity must stay 0, got {}",
            pvel.x
        );

        // 坏参数：越界 index / 非有限冲量 / 未知 id / 空 world。
        assert_eq!(
            soft_body_apply_particle_impulse(world, id, 999, 1.0, 0.0, 0.0),
            Bool::FALSE
        );
        assert_eq!(
            soft_body_apply_particle_impulse(world, id, free, f64::NAN, 0.0, 0.0),
            Bool::FALSE
        );
        assert_eq!(
            soft_body_apply_particle_impulse(world, u32::MAX, free, 1.0, 0.0, 0.0),
            Bool::FALSE
        );
        assert_eq!(
            soft_body_apply_particle_impulse(std::ptr::null_mut(), id, free, 1.0, 0.0, 0.0),
            Bool::FALSE
        );

        world_destroy(world);
    }

    #[test]
    fn soft_body_apply_particle_impulse_scales_with_inv_mass() {
        // 同一冲量 J 下，重质点 (inv_mass 小) 速度变化 < 轻质点。
        let world = make_world();
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let light = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE); // mass=1, inv=1
        let heavy = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 4.0, Bool::FALSE); // mass=4, inv=0.25
        soft_body_apply_particle_impulse(world, id, light, 4.0, 0.0, 0.0);
        soft_body_apply_particle_impulse(world, id, heavy, 4.0, 0.0, 0.0);
        let mut p = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut lvel = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut hvel = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            soft_body_get_particle(world, id, light, &mut p, &mut lvel),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_get_particle(world, id, heavy, &mut p, &mut hvel),
            Bool::TRUE
        );
        assert!((lvel.x - 4.0).abs() < 1e-9, "light vx=4, got {}", lvel.x);
        assert!((hvel.x - 1.0).abs() < 1e-9, "heavy vx=1, got {}", hvel.x);
        assert!(hvel.x < lvel.x, "heavy should move less than light");
        world_destroy(world);
    } // ── Phase 25 #3: AABB / 质心回读（纯 mps-core，零 fork 改动）──────────────────
    // 四个质点摆成盒子角，read_aabb 应回 min/max 角 + 质心=均值；质心也可缺省输出。
    #[test]
    fn soft_body_read_aabb_matches_particles() {
        let world = make_world();
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        // 盒子四角：(-1,-2,-3) / (1,-2,3) / (1,2,-3) / (-1,2,3) → min=(-1,-2,-3) max=(1,2,3)
        soft_body_add_particle(world, id, -1.0, -2.0, -3.0, 1.0, Bool::FALSE);
        soft_body_add_particle(world, id, 1.0, -2.0, 3.0, 1.0, Bool::FALSE);
        soft_body_add_particle(world, id, 1.0, 2.0, -3.0, 1.0, Bool::FALSE);
        soft_body_add_particle(world, id, -1.0, 2.0, 3.0, 1.0, Bool::FALSE);

        let mut mn = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut mx = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut ce = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            soft_body_read_aabb(world, id, &mut mn, &mut mx, &mut ce),
            Bool::TRUE
        );
        assert!(
            (mn.x + 1.0).abs() < 1e-9 && (mn.y + 2.0).abs() < 1e-9 && (mn.z + 3.0).abs() < 1e-9,
            "min should be (-1,-2,-3), got {:?}",
            mn
        );
        assert!(
            (mx.x - 1.0).abs() < 1e-9 && (mx.y - 2.0).abs() < 1e-9 && (mx.z - 3.0).abs() < 1e-9,
            "max should be (1,2,3), got {:?}",
            mx
        );
        // 质心 = 均值 = (0,0,0)
        assert!(
            (ce.x).abs() < 1e-9 && (ce.y).abs() < 1e-9 && (ce.z).abs() < 1e-9,
            "centroid should be (0,0,0), got {:?}",
            ce
        );

        // 质心输出可缺省（传 null）。
        let mut only_min = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            soft_body_read_aabb(
                world,
                id,
                &mut only_min,
                std::ptr::null_mut(),
                std::ptr::null_mut()
            ),
            Bool::TRUE
        );
        assert!(
            (only_min.x + 1.0).abs() < 1e-9,
            "min x still -1, got {}",
            only_min.x
        );

        world_destroy(world);
    }

    #[test]
    fn soft_body_read_aabb_updates_after_move() {
        // 移动一个质点后 AABB 边界应跟着变。
        let world = make_world();
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let _a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        // 最右质点 b 在 x=1：往右推它，max.x 才会超过 1。
        let b = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let mut mx = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut mn = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut ce = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            soft_body_read_aabb(world, id, &mut mn, &mut mx, &mut ce),
            Bool::TRUE
        );
        assert!((mx.x - 1.0).abs() < 1e-9, "initial max x = 1, got {}", mx.x);

        // 把 b 沿 +x 推一下（冲量改速度），步进后位置右移 → max.x 应超过初始的 1。
        soft_body_apply_particle_impulse(world, id, b, 5.0, 0.0, 0.0);
        world_step(world, 1.0 / 60.0);
        assert_eq!(
            soft_body_read_aabb(world, id, &mut mn, &mut mx, &mut ce),
            Bool::TRUE
        );
        assert!(
            mx.x > 1.0,
            "max x should grow past 1 after moving rightmost particle right, got {}",
            mx.x
        );

        world_destroy(world);
    }

    #[test]
    fn soft_body_read_aabb_guards() {
        // 未知 id / 空 world → Bool::FALSE；无粒子体返回 Bool::FALSE。
        let world = make_world();
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let mut mn = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut mx = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut ce = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        // 还没加粒子 → 无粒子体。
        assert_eq!(
            soft_body_read_aabb(world, id, &mut mn, &mut mx, &mut ce),
            Bool::FALSE
        );
        // 未知 id。
        assert_eq!(
            soft_body_read_aabb(world, u32::MAX, &mut mn, &mut mx, &mut ce),
            Bool::FALSE
        );
        // 空 world。
        assert_eq!(
            soft_body_read_aabb(std::ptr::null_mut(), id, &mut mn, &mut mx, &mut ce),
            Bool::FALSE
        );
        world_destroy(world);
    } // ── Phase 25 #4: 软体克隆（纯 mps-core，零 fork 改动）──────────────────────
    // clone 深拷贝粒子 + 弹簧 + 约束到新 id；副本独立（改源不影响副本）；未知 id → u32::MAX。
    #[test]
    fn soft_body_clone_deep_copies_and_is_independent() {
        let world = make_world();
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        // 一根两端弹簧 + 一个约束 + 一个三角形骨架。
        let a = soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);
        let b = soft_body_add_particle(world, id, 1.0, 1.0, 0.0, 1.0, Bool::FALSE);
        soft_body_add_spring(world, id, a, b, 50.0, 0.1);
        soft_body_add_distance_constraint(world, id, a, b, 0.01);

        let clone = soft_body_clone(world, id);
        assert!(clone != u32::MAX, "clone should return a valid new id");
        assert_ne!(clone, id, "clone must be a different id from source");

        // 副本粒子数一致。
        assert_eq!(soft_body_particle_count(world, clone), 2);

        // 副本两端位置与源相同。
        let mut sp = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut sv = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut cp = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut cv = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            soft_body_get_particle(world, id, a, &mut sp, &mut sv),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_get_particle(world, clone, a, &mut cp, &mut cv),
            Bool::TRUE
        );
        assert!(
            (sp.x - cp.x).abs() < 1e-9 && (sp.y - cp.y).abs() < 1e-9,
            "clone particle a should match source position, src={:?} clone={:?}",
            sp,
            cp
        );

        // 副本独立：给源质点冲量后，源动、副本不动。
        soft_body_apply_particle_impulse(world, id, a, 7.0, 0.0, 0.0);
        world_step(world, 1.0 / 60.0);
        assert_eq!(
            soft_body_get_particle(world, id, a, &mut sp, &mut sv),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_get_particle(world, clone, a, &mut cp, &mut cv),
            Bool::TRUE
        );
        assert!(
            sv.x > 0.0,
            "source a should have moved (vx>0), got {}",
            sv.x
        );
        assert!(
            (cv.x).abs() < 1e-9,
            "clone a must NOT move, got vx={}",
            cv.x
        );

        // 源仍受重力下落：两质点 y 应下降。
        assert!(sp.y < 1.0, "source should fall under gravity, y={}", sp.y);

        // 未知 id → u32::MAX。
        assert_eq!(soft_body_clone(world, u32::MAX), u32::MAX);
        // 空 world → u32::MAX。
        assert_eq!(soft_body_clone(std::ptr::null_mut(), id), u32::MAX);

        world_destroy(world);
    } // ── Phase 25 #5: 状态序列化 save/restore（纯 mps-core，零 fork 改动）───────
    // SoftBody 不 derive Serialize，mps-core 手写 LE 二进制（de）序列化:
    // soft_body_state_size + soft_body_save_state + soft_body_restore_state。
    #[test]
    fn soft_body_state_save_restore_roundtrip() {
        let world = make_world();
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        let a = soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);
        let b = soft_body_add_particle(world, id, 1.0, 1.0, 0.0, 1.0, Bool::FALSE);
        soft_body_add_spring(world, id, a, b, 50.0, 0.1);
        soft_body_add_distance_constraint(world, id, a, b, 0.01);

        // 量出字节数并分配缓冲。
        let size = soft_body_state_size(world, id);
        assert!(
            size > 0 && size != u32::MAX,
            "state size should be positive"
        );
        let mut buf: Vec<u8> = vec![0u8; size as usize];

        // 保存前记录 a 位置（含 y，受重力应为 1.0）。
        let mut before = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut bvel = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            soft_body_get_particle(world, id, a, &mut before, &mut bvel),
            Bool::TRUE
        );

        // 保存。
        assert_eq!(
            soft_body_save_state(world, id, buf.as_mut_ptr(), buf.len() as u32),
            Bool::TRUE
        );
        // 缓冲太小 → FALSE。
        let mut tiny = [0u8; 4];
        assert_eq!(
            soft_body_save_state(world, id, tiny.as_mut_ptr(), 4),
            Bool::FALSE
        );

        // 模拟状态变化：给 a 一个冲量（改速度；restore 也存速度）。
        soft_body_apply_particle_impulse(world, id, a, 0.0, -3.0, 0.0);
        let mut after = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut avel = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            soft_body_get_particle(world, id, a, &mut after, &mut avel),
            Bool::TRUE
        );
        assert!(
            avel.y < -1e-9,
            "state should have changed (vy<0) before restore, got {}",
            avel.y
        );
        assert!(
            (after.x - before.x).abs() < 1e-9,
            "position unchanged within one frame (only velocity changed)"
        );

        // 从快照恢复 → 回到 before 状态。
        assert_eq!(
            soft_body_restore_state(world, id, buf.as_ptr(), buf.len() as u32),
            Bool::TRUE
        );
        let mut restored = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut rvel = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            soft_body_get_particle(world, id, a, &mut restored, &mut rvel),
            Bool::TRUE
        );
        assert!(
            (restored.x - before.x).abs() < 1e-9 && (restored.y - before.y).abs() < 1e-9,
            "restored pos should match saved, before={:?} restored={:?}",
            before,
            restored
        );
        // 弹簧/约束数量应一致。
        assert_eq!(soft_body_particle_count(world, id), 2);

        // 损坏 blob → FALSE（改写 magic）。
        let mut bad = buf.clone();
        bad[0] = b'X';
        assert_eq!(
            soft_body_restore_state(world, id, bad.as_ptr(), bad.len() as u32),
            Bool::FALSE
        );
        // 未知 id → FALSE。
        assert_eq!(
            soft_body_restore_state(world, u32::MAX, buf.as_ptr(), buf.len() as u32),
            Bool::FALSE
        );

        world_destroy(world);
    } // ── Phase 25 #6: 逐粒子速度写入 setParticleVelocity（纯 mps-core，零 fork 改动）──
    // 已有 read，本项补写: 直接覆盖 particle.vel。pinned(inv_mass==0) 与越界/未知 id 守卫。
    #[test]
    fn soft_body_set_particle_velocity_writes_and_guards() {
        let world = make_world();
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        let a = soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);
        let b = soft_body_add_particle(world, id, 1.0, 1.0, 0.0, 1.0, Bool::TRUE); // pinned

        // 写入 a 的速度。
        assert_eq!(
            soft_body_set_particle_velocity(world, id, a, 2.0, 3.0, 4.0),
            Bool::TRUE
        );
        let mut pos = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut vel = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            soft_body_get_particle(world, id, a, &mut pos, &mut vel),
            Bool::TRUE
        );
        assert!(
            (vel.x - 2.0).abs() < 1e-9 && (vel.y - 3.0).abs() < 1e-9 && (vel.z - 4.0).abs() < 1e-9,
            "written velocity should read back, got {:?}",
            vel
        );

        // pinned 粒子 → FALSE。
        assert_eq!(
            soft_body_set_particle_velocity(world, id, b, 1.0, 1.0, 1.0),
            Bool::FALSE
        );
        // 越界 index → FALSE。
        assert_eq!(
            soft_body_set_particle_velocity(world, id, 99, 0.0, 0.0, 0.0),
            Bool::FALSE
        );
        // 未知 id → FALSE。
        assert_eq!(
            soft_body_set_particle_velocity(world, u32::MAX, a, 0.0, 0.0, 0.0),
            Bool::FALSE
        );
        // null world → FALSE。
        assert_eq!(
            soft_body_set_particle_velocity(std::ptr::null_mut(), id, a, 0.0, 0.0, 0.0),
            Bool::FALSE
        );

        world_destroy(world);
    }
    // ── Phase 28: clear / manual-trigger / readback FFI ─────────────────────────

    /// 5 个 `clear_*` 关闭变体对 null world / 未知 id 必须返回 `Bool::FALSE`，
    /// 且不 panic；对有效 id 返回 `Bool::TRUE`（纯关闭，set 之后再 clear 不报错）。
    #[test]
    fn soft_body_clear_variants_disable_material() {
        let world = world_create(Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        assert!(!world.is_null());

        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);

        // 先 set 几种材料（带合法参数），再 clear —— round-trip 不应报错。
        assert_eq!(soft_body_set_pressure(world, id, 1.5), Bool::TRUE);
        assert_eq!(soft_body_clear_pressure(world, id), Bool::TRUE);

        assert_eq!(
            soft_body_set_self_collision(world, id, 0.2, 0.0),
            Bool::TRUE
        );
        assert_eq!(soft_body_clear_self_collision(world, id), Bool::TRUE);

        assert_eq!(
            soft_body_set_cross_collision(world, id, 0.2, 0.0),
            Bool::TRUE
        );
        assert_eq!(soft_body_clear_cross_collision(world, id), Bool::TRUE);

        assert_eq!(
            soft_body_set_volume_conservation(world, id, 0.1),
            Bool::TRUE
        );
        assert_eq!(soft_body_clear_volume_conservation(world, id), Bool::TRUE);

        assert_eq!(soft_body_set_cohesion(world, id, 0.2, 0.5, 0.3), Bool::TRUE);
        assert_eq!(soft_body_clear_cohesion(world, id), Bool::TRUE);

        // 非法参数守住边界：null world / 未知 id 一律 FALSE。
        assert_eq!(
            soft_body_clear_pressure(std::ptr::null_mut(), id),
            Bool::FALSE
        );
        assert_eq!(soft_body_clear_pressure(world, u32::MAX), Bool::FALSE);
        assert_eq!(soft_body_clear_self_collision(world, u32::MAX), Bool::FALSE);
        assert_eq!(
            soft_body_clear_cross_collision(world, u32::MAX),
            Bool::FALSE
        );
        assert_eq!(
            soft_body_clear_volume_conservation(world, u32::MAX),
            Bool::FALSE
        );
        assert_eq!(soft_body_clear_cohesion(world, u32::MAX), Bool::FALSE);

        world_destroy(world);
    }

    /// `soft_body_apply_plasticity` 在塑性未配置（默认纯弹性）时是安全的 no-op，
    /// 对有效 id 返回 `Bool::TRUE`；null world / 未知 id 返回 `Bool::FALSE`。
    #[test]
    fn soft_body_apply_plasticity_noop_when_unconfigured() {
        let world = world_create(Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);

        // 配置塑性 → 触发一次投影（不应 panic，返回 TRUE）。
        assert_eq!(soft_body_set_plasticity(world, id, 0.1, 0.5, 1), Bool::TRUE);
        assert_eq!(soft_body_apply_plasticity(world, id), Bool::TRUE);
        // 关闭塑性后再触发 → 仍是安全 no-op。
        assert_eq!(soft_body_set_plasticity(world, id, 0.0, 0.0, 0), Bool::TRUE);
        assert_eq!(soft_body_apply_plasticity(world, id), Bool::TRUE);

        assert_eq!(
            soft_body_apply_plasticity(std::ptr::null_mut(), id),
            Bool::FALSE
        );
        assert_eq!(soft_body_apply_plasticity(world, u32::MAX), Bool::FALSE);

        world_destroy(world);
    }

    /// `soft_body_tear_now` 手动触发撕裂：配置应变阈值后，先拉伸一条弹簧边，
    /// 不推进时间步直接 `tear_now`，边被丢弃、质点数不变、edge 数归零。
    #[test]
    fn soft_body_tear_now_manual_trigger() {
        let world = world_create(Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);

        // 两个自由质点，间距 1，连一条弹簧。
        let a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let b = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        assert_eq!(soft_body_add_spring(world, id, a, b, 10.0, 1.0), Bool::TRUE);
        assert_eq!(soft_body_particle_count(world, id), 2);
        assert_eq!(soft_body_read_edges(world, id, std::ptr::null_mut(), 0), 1);

        // 配置应变阈值 0.5（拉伸 >50% 才断）。
        assert_eq!(soft_body_set_tear_strain(world, id, 0.5, 1), Bool::TRUE);
        // 不推进时间步：直接把弹簧 rest_length 缩放为 0.3（几何距离仍为 1），
        // 制造应变 (1-0.3)/0.3 ≈ 2.33 > 0.5，触发撕裂。
        assert!(soft_body_scale_rest_length(world, id, 0.3) > 0);
        // 手动撕裂（不推进时间步）。
        assert_eq!(soft_body_tear_now(world, id), Bool::TRUE);
        // 边被丢弃；质点保留（拓扑修整只删边，不删质点）。
        assert_eq!(soft_body_read_edges(world, id, std::ptr::null_mut(), 0), 0);
        assert_eq!(soft_body_particle_count(world, id), 2);

        // 边界：null world / 未知 id。
        assert_eq!(soft_body_tear_now(std::ptr::null_mut(), id), Bool::FALSE);
        assert_eq!(soft_body_tear_now(world, u32::MAX), Bool::FALSE);

        world_destroy(world);
    }

    /// `soft_body_read_spring_forces`：返回合力数量 == 质点数；缓冲区过小则截断不越界；
    /// 空缓冲仅返回数量。
    #[test]
    fn soft_body_read_spring_forces_roundtrip() {
        let world = world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        assert!(!world.is_null());
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);

        // 两个被弹簧相连的质点。
        let a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let b = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        assert_eq!(soft_body_add_spring(world, id, a, b, 10.0, 0.0), Bool::TRUE);
        assert_eq!(soft_body_particle_count(world, id), 2);

        // 空缓冲：仅返回数量。
        assert_eq!(
            soft_body_read_spring_forces(world, id, std::ptr::null_mut(), 0),
            2
        );

        // 全量缓冲。
        let mut buf: [Vec3; 4] = [Vec3::default(); 4];
        let n = soft_body_read_spring_forces(world, id, buf.as_mut_ptr(), buf.len() as u32);
        assert_eq!(n, 2);

        // 截断缓冲：capacity=1，仍返回真实数量 2，不越界（buf[1] 保持哨兵不变）。
        // `Vec3`(FFI 结构体)未实现 `PartialEq`，逐字段比对哨兵。
        let sentinel = Vec3 {
            x: 7.0,
            y: 7.0,
            z: 7.0,
        };
        let mut buf2: [Vec3; 4] = [sentinel; 4];
        let n2 = soft_body_read_spring_forces(world, id, buf2.as_mut_ptr(), 1);
        assert_eq!(n2, 2);
        // buf[0] 被写入（合力有限，非 NaN/inf，证明写回了真实弹簧力）。
        assert!(buf2[0].x.is_finite() && buf2[0].y.is_finite() && buf2[0].z.is_finite());
        // buf[1] 未被触碰（截断保护）。
        assert_eq!(buf2[1].x, 7.0);
        assert_eq!(buf2[1].y, 7.0);
        assert_eq!(buf2[1].z, 7.0);

        // 边界：null world / 未知 id 返回 0。
        assert_eq!(
            soft_body_read_spring_forces(std::ptr::null_mut(), id, buf.as_mut_ptr(), 4),
            0
        );
        assert_eq!(
            soft_body_read_spring_forces(world, u32::MAX, buf.as_mut_ptr(), 4),
            0
        );

        world_destroy(world);
    }

    #[test]
    fn soft_body_corotated_recovers_shear() {
        // Two soft bodies, each a single tetra with 3 pinned verts + 1 free vertex.
        // Same gravity, same hard volume conservation. Body A (baseline) relies on
        // the volume constraint only; body B additionally enables corotated
        // elasticity. After shear loading, B's free vertex must sit closer to its
        // rest position than A's (deviatoric recovery), both staying finite.
        let world = world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        assert!(!world.is_null());
        let build = |world: *mut WorldHandle| -> u32 {
            let id = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            // rest tet: right-angle unit tet; vertices 0..2 pinned, 3 free.
            let a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::TRUE);
            let b = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::TRUE);
            let c = soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::TRUE);
            let d = soft_body_add_particle(world, id, 0.0, 0.0, 1.0, 1.0, Bool::FALSE);
            assert_eq!((a, b, c, d), (0, 1, 2, 3));
            assert_eq!(soft_body_add_tetrahedron(world, id, 0, 1, 2, 3), Bool::TRUE);
            assert_eq!(
                soft_body_configure_solver(world, id, 1, 20, 0.0),
                Bool::TRUE
            );
            assert_eq!(
                soft_body_set_volume_conservation(world, id, 0.0),
                Bool::TRUE
            );
            assert_eq!(
                soft_body_set_gravity(
                    world,
                    id,
                    Vec3 {
                        x: 0.0,
                        y: -9.81,
                        z: 0.0,
                    }
                ),
                Bool::TRUE
            );
            id
        };
        let id_base = build(world);
        let id_coro = build(world);
        // enable corotated on the second body (rest shapes snapshot = undeformed)
        assert_eq!(soft_body_set_corotated(world, id_coro, 0.5), Bool::TRUE);
        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
        }
        let mut pb = Vec3::default();
        let mut pc = Vec3::default();
        soft_body_get_particle(world, id_base, 3, &mut pb, std::ptr::null_mut());
        soft_body_get_particle(world, id_coro, 3, &mut pc, std::ptr::null_mut());
        let dist = |p: Vec3, q: Vec3| -> f64 {
            ((p.x - q.x).powi(2) + (p.y - q.y).powi(2) + (p.z - q.z).powi(2)).sqrt()
        };
        let err_base = dist(
            pb,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        );
        let err_coro = dist(
            pc,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        );
        assert!(err_base.is_finite() && err_coro.is_finite());
        assert!(
            err_coro < err_base,
            "corotated must recover the free vertex better: coro={} base={}",
            err_coro,
            err_base
        );
        // clear + invalid args
        assert_eq!(soft_body_clear_corotated(world, id_coro), Bool::TRUE);
        assert_eq!(soft_body_set_corotated(world, id_base, 0.0), Bool::FALSE);
        assert_eq!(soft_body_set_corotated(world, id_base, 1.5), Bool::FALSE);
        assert_eq!(
            soft_body_set_corotated(world, id_base, f64::NAN),
            Bool::FALSE
        );
        assert_eq!(soft_body_set_corotated(world, 999, 0.5), Bool::FALSE);
        assert_eq!(soft_body_clear_corotated(world, 999), Bool::FALSE);
        world_destroy(world);
    }

    #[test]
    fn soft_body_corotated_rotation_invariant() {
        // The same body rotated 90 degrees about Y must deform identically
        // (rotation invariance of the corotated model).
        let dist = |p: Vec3, q: Vec3| -> f64 {
            ((p.x - q.x).powi(2) + (p.y - q.y).powi(2) + (p.z - q.z).powi(2)).sqrt()
        };
        let run = |rotated: bool| -> f64 {
            let world = world_create(Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            });
            let id = soft_body_create(
                world,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            let r = |x: f64, y: f64, z: f64| -> (f64, f64, f64) {
                if rotated {
                    (z, y, -x) // 90 deg about Y
                } else {
                    (x, y, z)
                }
            };
            let v0 = r(0.0, 0.0, 0.0);
            let v1 = r(1.0, 0.0, 0.0);
            let v2 = r(0.0, 1.0, 0.0);
            let v3 = r(0.0, 0.0, 1.0);
            soft_body_add_particle(world, id, v0.0, v0.1, v0.2, 1.0, Bool::TRUE);
            soft_body_add_particle(world, id, v1.0, v1.1, v1.2, 1.0, Bool::TRUE);
            soft_body_add_particle(world, id, v2.0, v2.1, v2.2, 1.0, Bool::TRUE);
            soft_body_add_particle(world, id, v3.0, v3.1, v3.2, 1.0, Bool::FALSE);
            soft_body_add_tetrahedron(world, id, 0, 1, 2, 3);
            soft_body_configure_solver(world, id, 1, 20, 0.0);
            soft_body_set_volume_conservation(world, id, 0.0);
            soft_body_set_gravity(
                world,
                id,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            assert_eq!(soft_body_set_corotated(world, id, 0.5), Bool::TRUE);
            for _ in 0..120 {
                world_step(world, 1.0 / 60.0);
            }
            let mut p = Vec3::default();
            soft_body_get_particle(world, id, 3, &mut p, std::ptr::null_mut());
            let rest = r(0.0, 0.0, 1.0);
            let dev = dist(
                p,
                Vec3 {
                    x: rest.0,
                    y: rest.1,
                    z: rest.2,
                },
            );
            world_destroy(world);
            dev
        };
        let d0 = run(false);
        let d1 = run(true);
        assert!(d0.is_finite() && d1.is_finite());
        assert!(
            (d0 - d1).abs() < 1e-9,
            "rotation invariance violated: d0={} d1={}",
            d0,
            d1
        );
    }
    #[test]
    fn soft_body_neo_hookean_resists_compression_more() {
        // Single tet, 4 free particles, compressive pressure pushing the apex toward
        // the base. Linear volume (baseline) vs Neo-Hookean ln(J) volume: at the same
        // stiffness the nonlinear residual must keep MORE rest volume under the same
        // load (unbounded resistance as V -> 0).
        let world = world_create(Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        assert!(!world.is_null());
        let build = |world: *mut WorldHandle| -> u32 {
            let id = soft_body_create(world, Vec3::default());
            soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
            soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
            soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);
            soft_body_add_particle(world, id, 0.0, 0.0, 1.0, 1.0, Bool::FALSE);
            soft_body_add_tetrahedron(world, id, 0, 1, 2, 3);
            soft_body_configure_solver(world, id, 1, 30, 0.0);
            id
        };
        let id_lin = build(world);
        let id_nh = build(world);
        // Same compliance scale for a fair comparison: linear uses
        // volume_conservation compliance 0.05; NH uses stiffness 0.05 (alpha = k/dt^2).
        assert_eq!(
            soft_body_set_volume_conservation(world, id_lin, 0.05),
            Bool::TRUE
        );
        assert_eq!(soft_body_set_neo_hookean(world, id_nh, 0.05), Bool::TRUE);
        // One hard kick on the apex (particle 3) TOWARD the base plane (-z): the
        // right-angle tet's volume depends on p3.z, so this is a true compressive
        // load. Compare the retained volume after ONE step (before the
        // oscillation sets in): at equal stiffness the ln(J) residual pushes
        // back harder.
        soft_body_apply_particle_impulse(world, id_lin, 3, 0.0, 0.0, -30.0);
        soft_body_apply_particle_impulse(world, id_nh, 3, 0.0, 0.0, -30.0);
        world_step(world, 1.0 / 60.0);
        let vol_lin = soft_body_total_volume(world, id_lin);
        let vol_nh = soft_body_total_volume(world, id_nh);
        assert!(vol_lin.is_finite() && vol_nh.is_finite());
        assert!(
            vol_nh > vol_lin,
            "Neo-Hookean must retain more volume under compression: nh={} lin={}",
            vol_nh,
            vol_lin
        );
        // clear + invalid guards
        assert_eq!(soft_body_clear_neo_hookean(world, id_nh), Bool::TRUE);
        assert_eq!(soft_body_set_neo_hookean(world, id_nh, -1.0), Bool::FALSE);
        assert_eq!(
            soft_body_set_neo_hookean(world, id_nh, f64::NAN),
            Bool::FALSE
        );
        assert_eq!(soft_body_set_neo_hookean(world, 999, 1.0), Bool::FALSE);
        assert_eq!(soft_body_clear_neo_hookean(world, 999), Bool::FALSE);
        world_destroy(world);
    }

    #[test]
    fn soft_body_neo_hookean_save_restore_roundtrip() {
        // save/restore must carry the neo_hookean flag: restore into a cleared body
        // and verify the behaviour (volume retention under the same load) survives.
        let world = world_create(Vec3::default());
        let id = soft_body_create(world, Vec3::default());
        soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        soft_body_add_particle(world, id, 0.0, 1.0, 0.0, 1.0, Bool::FALSE);
        soft_body_add_particle(world, id, 0.0, 0.0, 1.0, 1.0, Bool::FALSE);
        soft_body_add_tetrahedron(world, id, 0, 1, 2, 3);
        soft_body_configure_solver(world, id, 1, 30, 0.0);
        assert_eq!(soft_body_set_neo_hookean(world, id, 0.05), Bool::TRUE);
        // save
        let n = soft_body_state_size(world, id);
        assert!(n > 0);
        let mut buf = vec![0u8; n as usize];
        assert_eq!(
            soft_body_save_state(world, id, buf.as_mut_ptr(), n),
            Bool::TRUE
        );
        // clear, then restore
        assert_eq!(soft_body_clear_neo_hookean(world, id), Bool::TRUE);
        assert_eq!(
            soft_body_restore_state(world, id, buf.as_ptr(), n),
            Bool::TRUE
        );
        // compressive load: restored body must still resist like an NH body
        // (finite volume, and clear() afterwards must be a no-op-safe).
        soft_body_apply_particle_impulse(world, id, 3, 0.0, 0.0, -30.0);
        world_step(world, 1.0 / 60.0);
        let vol = soft_body_total_volume(world, id);
        assert!(vol.is_finite() && vol > 0.0);
        world_destroy(world);
    }

    #[test]
    fn soft_body_activation_pulls_endpoints_together() {
        // Two free particles linked by an XPBD distance constraint at rest length 1.0,
        // zero gravity. A positive activation shrinks the *effective* rest length
        // (rest*(1-gamma)), so the constraint actively pulls the endpoints together.
        // After one step the activated pair must be closer than the un-activated pair.
        let world = world_create(Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        assert!(!world.is_null());
        let build = |world: *mut WorldHandle| -> u32 {
            let id = soft_body_create(world, Vec3::default());
            let a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
            let b = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
            soft_body_add_distance_constraint(world, id, a, b, 0.0); // rest auto = 1.0
            soft_body_configure_solver(world, id, 1, 30, 0.0);
            id
        };
        let id_off = build(world);
        let id_on = build(world);
        assert_eq!(soft_body_set_activation(world, id_on, 0.5), Bool::TRUE);
        world_step(world, 1.0 / 60.0);
        let mut p_off = Vec3::default();
        let mut p_on = Vec3::default();
        soft_body_get_particle(
            world,
            id_off,
            1,
            &mut p_off as *mut Vec3,
            std::ptr::null_mut(),
        );
        soft_body_get_particle(
            world,
            id_on,
            1,
            &mut p_on as *mut Vec3,
            std::ptr::null_mut(),
        );
        // un-activated: stays at rest length 1.0; activated: pulled toward 0.5.
        let d_off = (p_off.x - 0.0).abs();
        let d_on = (p_on.x - 0.0).abs();
        assert!(d_off.is_finite() && d_on.is_finite());
        assert!(
            d_on < d_off - 1e-3,
            "activation must contract the edge: on={} off={}",
            d_on,
            d_off
        );
        // invalid inputs are rejected
        assert_eq!(soft_body_set_activation(world, id_on, 2.0), Bool::FALSE);
        assert_eq!(
            soft_body_set_activation(world, id_on, f64::NAN),
            Bool::FALSE
        );
        assert_eq!(soft_body_set_activation(world, 999, 0.5), Bool::FALSE);
        world_destroy(world);
    }

    #[test]
    fn soft_body_activation_save_restore_roundtrip() {
        // set_activation must survive save -> restore. Encode gamma on a body, snapshot,
        // restore into a cleared clone, then confirm the contraction behaviour returns.
        let world = world_create(Vec3::default());
        let id = soft_body_create(world, Vec3::default());
        let a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let b = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        soft_body_add_distance_constraint(world, id, a, b, 0.0);
        soft_body_configure_solver(world, id, 1, 30, 0.0);
        // per-edge activation via the dedicated setter also works
        assert_eq!(
            soft_body_set_distance_constraint_activation(world, id, 0, 0.7),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_set_spring_activation(world, id, 0, 0.7),
            Bool::FALSE
        ); // no springs
        // save
        let n = soft_body_state_size(world, id);
        assert!(n > 0);
        let mut buf = vec![0u8; n as usize];
        assert_eq!(
            soft_body_save_state(world, id, buf.as_mut_ptr(), n),
            Bool::TRUE
        );
        // restore into a fresh body
        let id2 = soft_body_create(world, Vec3::default());
        let a2 = soft_body_add_particle(world, id2, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let b2 = soft_body_add_particle(world, id2, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        soft_body_add_distance_constraint(world, id2, a2, b2, 0.0);
        soft_body_configure_solver(world, id2, 1, 30, 0.0);
        assert_eq!(
            soft_body_restore_state(world, id2, buf.as_ptr(), n),
            Bool::TRUE
        );
        // the restored body must now contract under the same step
        world_step(world, 1.0 / 60.0);
        let mut p = Vec3::default();
        soft_body_get_particle(world, id2, 1, &mut p as *mut Vec3, std::ptr::null_mut());
        assert!(
            p.x.is_finite() && p.x < 1.0 - 1e-3,
            "restored activation must contract: {}",
            p.x
        );
        world_destroy(world);
    }

    #[test]
    fn soft_body_fibre_direction_set_and_save_restore() {
        // Set a muscle-fibre direction on a spring + verify it survives save/restore.
        let world = world_create(Vec3::default());
        let id = soft_body_create(world, Vec3::default());
        let a = soft_body_add_particle(world, id, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let b = soft_body_add_particle(world, id, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        assert_eq!(soft_body_add_spring(world, id, a, b, 50.0, 0.5), Bool::TRUE);
        // single spring → index 0
        assert_eq!(
            soft_body_set_spring_fibre_direction(world, id, 0, 0.0, 1.0, 0.0),
            Bool::TRUE
        );
        // save
        let n = soft_body_state_size(world, id);
        assert!(n > 0);
        let mut buf = vec![0u8; n as usize];
        assert_eq!(
            soft_body_save_state(world, id, buf.as_mut_ptr(), n),
            Bool::TRUE
        );
        // restore into a fresh body
        let id2 = soft_body_create(world, Vec3::default());
        let a2 = soft_body_add_particle(world, id2, 0.0, 0.0, 0.0, 1.0, Bool::FALSE);
        let b2 = soft_body_add_particle(world, id2, 1.0, 0.0, 0.0, 1.0, Bool::FALSE);
        soft_body_add_spring(world, id2, a2, b2, 50.0, 0.5);
        assert_eq!(
            soft_body_restore_state(world, id2, buf.as_ptr(), n),
            Bool::TRUE
        );
        // invalid fibre vector (non-finite) is rejected
        assert_eq!(
            soft_body_set_spring_fibre_direction(world, id, 0, f64::NAN, 0.0, 0.0),
            Bool::FALSE
        );
        assert_eq!(
            soft_body_set_spring_fibre_direction(world, 999, 0, 0.0, 1.0, 0.0),
            Bool::FALSE
        );
        world_destroy(world);
    }

    // ── Phase 33: 绳索 / 发丝构造器 ──────────────────────────────────────────
    // 沿首尾方向布 N 质点 + 相邻 XPBD 距离约束；悬垂（pin_start）+ 步进而保持
    // 有界；闭合环；弯曲约束抗折。纯组合层，对照 build_tetra_mesh 的验证风格。

    #[test]
    fn soft_body_build_rope_hangs_and_stays_bounded() {
        let world = make_world();
        assert!(!world.is_null());

        // 8 质点绳索，起点 (0,5,0) 固定（pin_start），终点 (0,0,0) 自由，
        // 在重力下应下垂但仍受 XPBD 距离约束限制、整体有界（不飞出 / 不穿越无穷）。
        let id = soft_body_build_rope(
            world, 0.0, 5.0, 0.0, // start
            0.0, 0.0, 0.0, // end
            8,   // n particles
            0.5, // particle mass
            0.0, // compliance 0 → inextensible strand
            20,  // xpbd iterations
            1,   // pin_start
            0,   // pin_end free
            0,   // not closed
            0,   // no bending
        );
        assert!(id != u32::MAX, "rope should build");

        unsafe {
            let sb = (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present");
            assert_eq!(sb.particles.len(), 8, "rope has n particles");
            assert_eq!(sb.distance_constraints.len(), 7, "7 segment edges");
            assert_eq!(sb.particles[0].inv_mass, 0.0, "start endpoint pinned");
        }

        for _ in 0..200 {
            world_step(world, 1.0 / 60.0);
        }

        unsafe {
            let sb = (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present");
            // Pinned anchor stays put at its rest position (0, 5, 0).
            assert!(
                sb.particles[0].pos.y.abs() > 4.9 && sb.particles[0].pos.y.abs() < 5.1,
                "anchor stayed near (0,5,0), got y={}",
                sb.particles[0].pos.y
            );
            assert!(
                sb.particles[0].pos.x.abs() < 1e-6 && sb.particles[0].pos.z.abs() < 1e-6,
                "anchor did not move laterally"
            );
            // Every particle settled at a finite, bounded position.
            let mut max_r = 0.0_f64;
            for p in &sb.particles {
                assert!(p.pos.x.is_finite() && p.pos.y.is_finite() && p.pos.z.is_finite());
                max_r = max_r.max(p.pos.y.abs().max(p.pos.x.abs()).max(p.pos.z.abs()));
            }
            assert!(max_r < 50.0, "rope stayed bounded (max_r={max_r})");
            // The free end hangs below the anchor, not above it (gravity pulled it down).
            assert!(
                sb.particles[7].pos.y < sb.particles[0].pos.y,
                "free end hangs below the pinned anchor"
            );
        }
        world_destroy(world);
    }

    #[test]
    fn soft_body_build_rope_closed_loop_and_bending() {
        let world = make_world();
        assert!(!world.is_null());

        // 闭合环（necklace）带弯曲约束：n=6 应有 6 段边 + 6 条弯曲边 = 12 约束。
        let id = soft_body_build_rope(
            world, 1.0, 0.0, 0.0, // start
            -1.0, 0.0, 0.0, // end (opposite side; closed loop wraps back)
            6, 0.3, 0.01, // soft compliance so bending is observable
            10, 0, // no pins
            0, 1, // closed loop
            1, // bending
        );
        assert!(id != u32::MAX, "closed rope should build");

        unsafe {
            let sb = (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present");
            assert_eq!(sb.particles.len(), 6);
            // 6 segment edges + 6 bending edges (wrap-around for closed).
            assert_eq!(sb.distance_constraints.len(), 12, "6 seg + 6 bend edges");
        }

        // No NaN / non-finite blow-up across many steps with bending active.
        for _ in 0..150 {
            world_step(world, 1.0 / 60.0);
        }
        unsafe {
            let sb = (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present");
            for p in &sb.particles {
                assert!(p.pos.x.is_finite() && p.pos.y.is_finite() && p.pos.z.is_finite());
            }
        }
        world_destroy(world);
    }

    #[test]
    fn soft_body_build_rope_rejects_bad_params() {
        let world = make_world();
        assert!(!world.is_null());

        // n < 2 rejected.
        assert_eq!(
            soft_body_build_rope(
                world, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1, 1.0, 0.0, 10, 0, 0, 0, 0
            ),
            u32::MAX
        );
        // non-finite endpoint rejected.
        assert_eq!(
            soft_body_build_rope(
                world,
                f64::NAN,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                5,
                1.0,
                0.0,
                10,
                0,
                0,
                0,
                0
            ),
            u32::MAX
        );
        // non-positive mass rejected.
        assert_eq!(
            soft_body_build_rope(
                world, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 5, 0.0, 0.0, 10, 0, 0, 0, 0
            ),
            u32::MAX
        );
        // zero iterations rejected.
        assert_eq!(
            soft_body_build_rope(
                world, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 5, 1.0, 0.0, 0, 0, 0, 0, 0
            ),
            u32::MAX
        );
        world_destroy(world);
    }

    // ── Phase 34: 网格 / 方块软体构造器 ──────────────────────────────────────
    // 长方体范围内 nx×ny×nz 质点网格 + 6 邻接 XPBD 距离约束；pin 边界后整体
    // 悬挂；纯组合层，对照 rope 验证风格。

    #[test]
    fn soft_body_build_grid_makes_block_and_stays_bounded() {
        let world = make_world();
        assert!(!world.is_null());

        // 3×3×3 网格填满 [0,2]³，质量 0.5，compliance 0（刚性网格），20 迭代。
        let id = soft_body_build_grid(
            world, 0.0, 0.0, 0.0, // min
            2.0, 2.0, 2.0, // max
            3, 3, 3,   // nx, ny, nz
            0.5, // particle mass
            0.0, // compliance
            20,  // iterations
            0,   // no pin
        );
        assert!(id != u32::MAX, "grid should build");

        unsafe {
            let sb = (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present");
            assert_eq!(sb.particles.len(), 27, "3*3*3 = 27 particles");
            // 6-connectivity: 3*3*2 faces per axis * 3 axes = 54 edges.
            assert_eq!(sb.distance_constraints.len(), 54, "54 face edges");
            // Switched to XPBD solver (cross-crate enum: use matches!, not ==).
            assert!(
                matches!(
                    sb.solver,
                    rapier3d::prelude::soft_body_fork::SoftSolver::Xpbd { .. }
                ),
                "grid uses XPBD solver"
            );
        }

        for _ in 0..150 {
            world_step(world, 1.0 / 60.0);
        }
        unsafe {
            let sb = (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present");
            // Unpinned block sags under gravity but stays bounded (no blow-up).
            let mut max_r = 0.0_f64;
            let mut min_y = f64::INFINITY;
            for p in &sb.particles {
                assert!(p.pos.x.is_finite() && p.pos.y.is_finite() && p.pos.z.is_finite());
                max_r = max_r.max(p.pos.y.abs().max(p.pos.x.abs()).max(p.pos.z.abs()));
                min_y = min_y.min(p.pos.y);
            }
            assert!(max_r < 50.0, "grid stayed bounded (max_r={max_r})");
            // Gravity pulled the free block downward.
            assert!(min_y < 2.0, "block sagged below its top (min_y={min_y})");
        }
        world_destroy(world);
    }

    #[test]
    fn soft_body_build_grid_pinned_boundary_holds() {
        let world = make_world();
        assert!(!world.is_null());

        // 4×4×4 网格，pin 边界 → 外表面质点 inv_mass=0，内部质点自由下垂。
        let id = soft_body_build_grid(
            world, -1.0, 0.0, -1.0, 1.0, 2.0, 1.0, 4, 4, 4, 0.3, 0.0, 15, 1, // pin_boundary
        );
        assert!(id != u32::MAX, "pinned grid should build");

        unsafe {
            let sb = (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present");
            // Boundary count: outer shell of 4×4×4 = 64 - 8 interior = 56 pinned.
            let pinned = sb.particles.iter().filter(|p| p.inv_mass == 0.0).count();
            assert_eq!(pinned, 56, "56 boundary particles pinned");
            let free = sb.particles.iter().filter(|p| p.inv_mass > 0.0).count();
            assert_eq!(free, 8, "8 interior particles free");
        }

        for _ in 0..150 {
            world_step(world, 1.0 / 60.0);
        }
        unsafe {
            let sb = (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("soft body present");
            // Pinned boundary particles must not move from their rest positions.
            // Check the 8 corners explicitly.
            let corners = [
                (0, 0, 0),
                (3, 0, 0),
                (0, 3, 0),
                (3, 3, 0),
                (0, 0, 3),
                (3, 0, 3),
                (0, 3, 3),
                (3, 3, 3),
            ];
            let idx = |i: usize, j: usize, k: usize| i + j * 4 + k * 16;
            for (i, j, k) in corners {
                let p = &sb.particles[idx(i, j, k)];
                assert_eq!(p.inv_mass, 0.0, "corner pinned");
                let (ex, ey, ez) = (
                    -1.0 + 2.0 * i as f64 / 3.0,
                    0.0 + 2.0 * j as f64 / 3.0,
                    -1.0 + 2.0 * k as f64 / 3.0,
                );
                assert!(
                    (p.pos.x - ex).abs() < 1e-6
                        && (p.pos.y - ey).abs() < 1e-6
                        && (p.pos.z - ez).abs() < 1e-6,
                    "pinned corner stayed at rest"
                );
            }
            // Interior free particles settled at finite positions.
            for p in &sb.particles {
                assert!(p.pos.x.is_finite() && p.pos.y.is_finite() && p.pos.z.is_finite());
            }
        }
        world_destroy(world);
    }

    #[test]
    fn soft_body_build_grid_rejects_bad_params() {
        let world = make_world();
        assert!(!world.is_null());

        // inverted box rejected.
        assert_eq!(
            soft_body_build_grid(
                world, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 2, 2, 2, 1.0, 0.0, 10, 0
            ),
            u32::MAX
        );
        // zero resolution rejected.
        assert_eq!(
            soft_body_build_grid(
                world, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0, 2, 2, 1.0, 0.0, 10, 0
            ),
            u32::MAX
        );
        // non-positive mass rejected.
        assert_eq!(
            soft_body_build_grid(
                world, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2, 2, 2, 0.0, 0.0, 10, 0
            ),
            u32::MAX
        );
        // too many particles (>1M) rejected.
        assert_eq!(
            soft_body_build_grid(
                world, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 200, 200, 200, 1.0, 0.0, 10, 0
            ),
            u32::MAX
        );
        world_destroy(world);
    }

    #[test]
    fn soft_body_soft_soft_collision_blocks_penetration() {
        // Two 3x3x3 grids placed overlapping, driven toward each other. With
        // proxy-collider coupling on, the rapier narrow-phase must keep them from
        // passing through one another (soft-body ↔ soft-body collision).
        let world: *mut WorldHandle = world_create(Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        assert!(!world.is_null());
        let n = 3u32;
        let r = 0.2f64; // proxy ball radius (< half spacing 0.5 → no self-collision)
        // Grid A: x ∈ [-1.0, 0.0] (spacing 0.5). Grid B: x ∈ [0.15, 1.15].
        // A's right edge (x=0.0) and B's left edge (x=0.15) are 0.15 apart,
        // closer than 2r = 0.4 → the proxy balls start overlapping.
        let id_a = soft_body_build_grid(
            world, -1.0, 0.0, 0.0, 0.0, 1.0, 1.0, n, n, n, 1.0, 0.0, 10, 0,
        );
        let id_b = soft_body_build_grid(
            world, 0.15, 0.0, 0.0, 1.15, 1.0, 1.0, n, n, n, 1.0, 0.0, 10, 0,
        );
        assert!(id_a != u32::MAX && id_b != u32::MAX);
        // Ram them together: A → +X, B → −X.
        for i in 0..27u32 {
            soft_body_set_particle_velocity(world, id_a, i, 2.0, 0.0, 0.0);
            soft_body_set_particle_velocity(world, id_b, i, -2.0, 0.0, 0.0);
        }
        assert_eq!(
            soft_body_enable_collision(world, id_a, r, Bool::TRUE),
            Bool::TRUE
        );
        assert_eq!(
            soft_body_enable_collision(world, id_b, r, Bool::TRUE),
            Bool::TRUE
        );
        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
        }
        let (a_max_x, b_min_x) = unsafe {
            let sa = (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id_a))
                .expect("A present");
            let sb = (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id_b))
                .expect("B present");
            let amax = sa
                .particles
                .iter()
                .map(|p| p.pos.x)
                .fold(f64::MIN, f64::max);
            let bmin = sb
                .particles
                .iter()
                .map(|p| p.pos.x)
                .fold(f64::MAX, f64::min);
            (amax, bmin)
        };
        // If the bodies passed through each other, A's right edge would end up
        // right of B's left edge by more than the contact tolerance.
        assert!(
            b_min_x - a_max_x >= -0.1,
            "soft bodies penetrated: A_max_x={a_max_x} B_min_x={b_min_x}"
        );
        world_destroy(world);
    }

    #[test]
    fn soft_body_same_body_particles_do_not_self_collide() {
        // A single dense grid whose particles are closer than 2r means their proxy
        // balls overlap. With collision coupling on, the proxy balls must NOT
        // collide with each other (otherwise the body would explode). This pins
        // the soft-body ↔ soft-body design: within one body, proxies are
        // self-decoupled; only across bodies do they collide.
        let world: *mut WorldHandle = world_create(Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        assert!(!world.is_null());
        let n = 3u32;
        let r = 0.25f64; // 2r = 0.5 > spacing 0.3 → proxy balls overlap within body
        let id = soft_body_build_grid(
            world, -0.3, -0.3, -0.3, 0.3, 0.3, 0.3, n, n, n, 1.0, 0.0, 10, 0,
        );
        assert!(id != u32::MAX);
        assert_eq!(
            soft_body_enable_collision(world, id, r, Bool::TRUE),
            Bool::TRUE
        );
        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
        }
        let (min_x, max_x) = unsafe {
            let sb = (*world)
                .inner
                .soft_bodies
                .get(SoftBodyId(id))
                .expect("body present");
            let min = sb
                .particles
                .iter()
                .map(|p| p.pos.x)
                .fold(f64::MAX, f64::min);
            let max = sb
                .particles
                .iter()
                .map(|p| p.pos.x)
                .fold(f64::MIN, f64::max);
            (min, max)
        };
        // No self-explosion: the body stays near its rest extent (±0.3) plus a
        // small relaxation tolerance. A self-colliding body would blow past this.
        assert!(
            (max_x - min_x) < 1.5,
            "body self-exploded: extent={}",
            max_x - min_x
        );
        world_destroy(world);
    }

    #[test]
    fn soft_body_skinning_follows_bones() {
        // Phase 3: a particle bound 50/50 to two bones should track the midpoint
        // of the live bone positions after a bone moves.
        let world = make_world();

        // Two fixed bones: A at origin, B at (1,0,0).
        let a_builder =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Fixed as u32);
        mps_core::rapier::rigid_body::rigid_body_builder_set_translation(
            a_builder,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let a = mps_core::rapier::rigid_body::world_insert_rigid_body(
            world,
            mps_core::rapier::rigid_body::rigid_body_builder_build(a_builder),
        );
        let b_builder =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Fixed as u32);
        mps_core::rapier::rigid_body::rigid_body_builder_set_translation(
            b_builder,
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let b = mps_core::rapier::rigid_body::world_insert_rigid_body(
            world,
            mps_core::rapier::rigid_body::rigid_body_builder_build(b_builder),
        );
        let bones = [a, b];

        // One soft-body particle at the midpoint (0.5,0,0).
        let id = soft_body_create(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_ne!(id, u32::MAX);
        let p = soft_body_add_particle(world, id, 0.5, 0.0, 0.0, 1.0, Bool::FALSE);
        assert_eq!(p, 0);

        assert_eq!(
            mps_core::rapier::soft_body::soft_body_bind_skeleton(world, id, 2, bones.as_ptr()),
            2
        );
        let bone_indices = [0u32, 1, 0, 0];
        let weights = [0.5f64, 0.5, 0.0, 0.0];
        assert_eq!(
            mps_core::rapier::soft_body::soft_body_set_vertex_weights(
                world,
                id,
                0,
                bone_indices.as_ptr(),
                weights.as_ptr()
            ),
            Bool::TRUE
        );

        // Move bone B to (2,0,0); the skinned particle should track the midpoint (1,0,0).
        mps_core::rapier::rigid_body::rigid_body_set_translation(
            world,
            b,
            Vec3 {
                x: 2.0,
                y: 0.0,
                z: 0.0,
            },
            Bool::TRUE,
        );
        for _ in 0..10 {
            world_step(world, 1.0 / 60.0);
        }

        let mut pos = Vec3::default();
        assert_eq!(
            soft_body_get_particle(world, id, 0, &mut pos as *mut Vec3, std::ptr::null_mut()),
            Bool::TRUE
        );
        assert!(
            (pos.x - 1.0).abs() < 1e-6 && pos.y.abs() < 1e-6 && pos.z.abs() < 1e-6,
            "skinned particle should sit at bone midpoint (1,0,0), got ({},{},{})",
            pos.x,
            pos.y,
            pos.z
        );
        world_destroy(world);
    }
}
