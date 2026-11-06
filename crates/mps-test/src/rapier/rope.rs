#[cfg(test)]
mod tests {
    use mps_core::rapier::collider::{
        collider_builder_build, collider_builder_create_sphere, world_insert_collider_with_parent,
    };
    use mps_core::rapier::error::{
        ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ERR_OK, last_error_code,
    };
    use mps_core::rapier::ffi::{BodyStatus, Bool, Sphere, Vec3, WorldHandle};
    use mps_core::rapier::rigid_body::{
        rigid_body_builder_build, rigid_body_builder_create, rigid_body_builder_set_linvel,
        rigid_body_builder_set_translation, world_insert_rigid_body,
    };
    use mps_core::rapier::rope::{
        ROPE_CABLE_COMPRESSION_COMPLIANCE, ROPE_MAX_PARTICLES, RopeDesc, soft_rope_create,
    };
    use mps_core::rapier::soft_body::{
        soft_body_attach_particle, soft_body_particle_count, soft_body_read_particles,
        soft_body_scale_rest_length, soft_body_set_damping,
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

    fn desc(start: Vec3, end: Vec3, pin_mode: u32, unilateral: Bool) -> RopeDesc {
        RopeDesc {
            segments: 8,
            start,
            end,
            particle_mass: 0.2,
            stretch_compliance: 0.0,
            slack: 0.0,
            iterations: 8,
            unilateral,
            pin_mode,
        }
    }

    fn v(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    /// Read back all particle positions via the public FFI.
    fn read_positions(world: *const WorldHandle, id: u32) -> Vec<Vec3> {
        let n = soft_body_particle_count(world, id);
        assert_ne!(n, u32::MAX, "rope must exist");
        let mut pos = vec![Vec3::default(); n as usize];
        let read = soft_body_read_particles(world, id, pos.as_mut_ptr(), std::ptr::null_mut(), n);
        assert_eq!(read, n);
        pos
    }

    #[test]
    fn rope_create_builds_particle_chain() {
        let world = make_world();
        let id = soft_rope_create(
            world,
            desc(v(0.0, 2.0, 0.0), v(1.0, 2.0, 0.0), 3, Bool::FALSE),
        );
        assert_ne!(id, SENTINEL);
        assert_eq!(last_error_code(), ERR_OK);

        // segments + 1 particles, laid out linearly along the span.
        let pos = read_positions(world, id);
        assert_eq!(pos.len(), 9);
        for (i, p) in pos.iter().enumerate() {
            let t = i as f64 / 8.0;
            assert!((p.x - t).abs() < 1e-9, "particle {i} x {}", p.x);
            assert!((p.y - 2.0).abs() < 1e-9);
            assert!((p.z).abs() < 1e-9);
        }

        // Constraint bookkeeping: one per segment, rest = taut spacing, XPBD
        // solver selected, bilateral by default, both endpoints pinned.
        let sb = unsafe {
            (*world)
                .inner
                .soft_bodies
                .get(rapier3d::prelude::soft_body_fork::SoftBodyId(id))
                .expect("rope present")
        };
        assert_eq!(sb.distance_constraints.len(), 8);
        for c in &sb.distance_constraints {
            assert!((c.rest - 0.125).abs() < 1e-9, "rest {}", c.rest);
            assert_eq!(c.compression, c.compliance, "bilateral default");
            assert_eq!(c.compression, 0.0);
        }
        assert!(matches!(
            sb.solver,
            rapier3d::prelude::soft_body_fork::SoftSolver::Xpbd { .. }
        ));
        assert_eq!(sb.particles[0].inv_mass, 0.0, "start pinned");
        assert_eq!(sb.particles[8].inv_mass, 0.0, "end pinned");
        assert!(sb.particles[4].inv_mass > 0.0, "middle free");
        world_destroy(world);
    }

    #[test]
    fn rope_cable_mode_makes_compression_free() {
        let world = make_world();
        let id = soft_rope_create(
            world,
            desc(v(0.0, 2.0, 0.0), v(1.0, 2.0, 0.0), 3, Bool::TRUE),
        );
        assert_ne!(id, SENTINEL);
        let sb = unsafe {
            (*world)
                .inner
                .soft_bodies
                .get(rapier3d::prelude::soft_body_fork::SoftBodyId(id))
                .expect("rope present")
        };
        for c in &sb.distance_constraints {
            assert_eq!(c.compression, ROPE_CABLE_COMPRESSION_COMPLIANCE);
            assert_eq!(c.compliance, 0.0, "tension side stays inextensible");
        }
        world_destroy(world);
    }

    #[test]
    fn rope_slack_inflates_rest_lengths() {
        let world = make_world();
        let mut d = desc(v(0.0, 2.0, 0.0), v(1.0, 2.0, 0.0), 0, Bool::FALSE);
        d.slack = 0.5;
        let id = soft_rope_create(world, d);
        assert_ne!(id, SENTINEL);
        let sb = unsafe {
            (*world)
                .inner
                .soft_bodies
                .get(rapier3d::prelude::soft_body_fork::SoftBodyId(id))
                .expect("rope present")
        };
        let total: f64 = sb.distance_constraints.iter().map(|c| c.rest).sum();
        assert!((total - 1.5).abs() < 1e-9, "rest total {total} != span·1.5");
        world_destroy(world);
    }

    #[test]
    fn rope_slack_cable_hangs_as_catenary() {
        // A slack cable (rest 1.4 vs span 1.0, both ends pinned) hangs using
        // its full length like a real rope: it sags well below the straight
        // line, every link stays taut (a hanging chain is in pure tension, so
        // the one-sided compression side never engages here — that side is
        // verified structurally in `rope_cable_mode_makes_compression_free`),
        // and the whole thing stays stable and bounded.
        let world = make_world();
        let mut d = desc(v(0.0, 2.0, 0.0), v(1.0, 2.0, 0.0), 3, Bool::TRUE);
        d.slack = 0.4;
        let id = soft_rope_create(world, d);
        assert_ne!(id, SENTINEL);
        assert_eq!(soft_body_set_damping(world, id, 0.05), Bool::TRUE);
        for _ in 0..240 {
            world_step(world, 1.0 / 60.0);
        }
        let pos = read_positions(world, id);
        let mut path = 0.0;
        for w in pos.windows(2) {
            let dx = w[1].x - w[0].x;
            let dy = w[1].y - w[0].y;
            let dz = w[1].z - w[0].z;
            path += (dx * dx + dy * dy + dz * dz).sqrt();
            assert!(w[0].x.is_finite() && w[0].y.is_finite() && w[0].z.is_finite());
            assert!(w[0].y > -50.0 && w[0].y < 50.0);
        }
        // Sag: middle particle well below the straight span at y = 2.
        assert!(
            pos[4].y < 1.9,
            "slack cable must sag: middle y {}",
            pos[4].y
        );
        // The hanging catenary deploys (approximately) the full rest length.
        assert!(
            path > 1.25 && path < 1.45,
            "hanging cable should use ~its full rest length: path {path}"
        );
        world_destroy(world);
    }

    #[test]
    fn rope_winch_reels_in_via_scale_rest_length() {
        let world = make_world();
        let id = soft_rope_create(
            world,
            desc(v(0.0, 2.0, 0.0), v(1.0, 2.0, 0.0), 3, Bool::TRUE),
        );
        assert_ne!(id, SENTINEL);
        let rest_sum = |world: *mut WorldHandle, id: u32| unsafe {
            (*world)
                .inner
                .soft_bodies
                .get(rapier3d::prelude::soft_body_fork::SoftBodyId(id))
                .expect("rope present")
                .distance_constraints
                .iter()
                .map(|c| c.rest)
                .sum::<f64>()
        };
        let before = rest_sum(world, id);
        assert!((before - 1.0).abs() < 1e-9);

        // Winch in twice by 10%: rest total shrinks to 0.81 m. Cable mode
        // (compression ≈ free) means the physical rope then goes slack.
        assert_eq!(soft_body_scale_rest_length(world, id, 0.9), 8);
        assert_eq!(soft_body_scale_rest_length(world, id, 0.9), 8);
        let after = rest_sum(world, id);
        assert!(
            (after - 0.81).abs() < 1e-9,
            "rest total after winch {after}"
        );

        // Reel out back beyond the span: the cable pulls taut again.
        assert_eq!(soft_body_scale_rest_length(world, id, 1.3), 8);
        world_destroy(world);
    }

    #[test]
    fn rope_attach_end_tracks_rigid_body() {
        let world = make_world();
        // Dynamic body gliding at constant velocity along +X from x=5.
        let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_translation(builder, v(5.0, 0.0, 0.0));
        rigid_body_builder_set_linvel(builder, v(1.0, 0.0, 0.0));
        let body = rigid_body_builder_build(builder);
        let body_h = world_insert_rigid_body(world, body);
        assert_ne!(body_h, 0u64);
        // Give the body a collider so it is a well-formed dynamic body.
        let cb = collider_builder_create_sphere(Sphere {
            center: v(5.0, 0.0, 0.0),
            radius: 0.1,
        });
        let collider = collider_builder_build(cb);
        world_insert_collider_with_parent(world, collider, body_h);

        // Free rope hanging down from the body's position; anchor particle 0.
        let id = soft_rope_create(
            world,
            desc(v(5.0, 0.0, 0.0), v(5.0, -1.0, 0.0), 0, Bool::TRUE),
        );
        assert_ne!(id, SENTINEL);
        assert_eq!(
            soft_body_attach_particle(world, id, 0, body_h, v(5.0, 0.0, 0.0)),
            Bool::TRUE
        );

        for _ in 0..60 {
            world_step(world, 1.0 / 60.0);
        }
        let pos = read_positions(world, id);
        let bx = 5.0 + 1.0; // body translation after 1 s at 1 m/s
        assert!(
            (pos[0].x - bx).abs() < 1e-6,
            "anchored particle must track the body: {} vs {bx}",
            pos[0].x
        );
        world_destroy(world);
    }

    #[test]
    fn rope_create_rejects_bad_params() {
        // Null world.
        assert_eq!(
            soft_rope_create(
                std::ptr::null_mut(),
                desc(v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), 0, Bool::FALSE)
            ),
            SENTINEL
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        let world = make_world();
        let cases: [(fn(&mut RopeDesc), &str); 8] = [
            (|d| d.segments = 0, "zero segments"),
            (|d| d.iterations = 0, "zero iterations"),
            (
                |d| {
                    d.end = d.start;
                },
                "degenerate span",
            ),
            (|d| d.particle_mass = 0.0, "zero mass"),
            (|d| d.stretch_compliance = -1.0, "negative compliance"),
            (|d| d.slack = -0.1, "negative slack"),
            (|d| d.pin_mode = 4, "unknown pin mode"),
            (|d| d.segments = ROPE_MAX_PARTICLES, "oversized"),
        ];
        for (mutate, label) in cases {
            let mut d = desc(v(0.0, 2.0, 0.0), v(1.0, 2.0, 0.0), 0, Bool::FALSE);
            mutate(&mut d);
            assert_eq!(soft_rope_create(world, d), SENTINEL, "case: {label}");
            let code = last_error_code();
            let expected = if label == "oversized" {
                ERR_CAPACITY
            } else {
                ERR_INVALID_ARGUMENT
            };
            assert_eq!(code, expected, "case: {label}");
        }
        world_destroy(world);
    }
}
