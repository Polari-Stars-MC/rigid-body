//! 官方 `SoftBody` 后端的集成测试（`soft_body_set_backend`）。
//!
//! 这些测试只走 C ABI：授权接口不变，只把求解后端切到 rapier v0.36 的官方软体，
//! 验证「同一套 API、换一个求解器」这条迁移路径能跑通。

#[cfg(test)]
mod tests {
    use mps_core::rapier::ffi::{Bool, Vec3, WorldHandle};
    use mps_core::rapier::soft_body::{
        soft_body_build_rope, soft_body_kinetic_energy, soft_body_particle_count,
        soft_body_read_particles, soft_body_set_backend, soft_body_set_gravity,
    };
    use mps_core::rapier::world::{world_create, world_destroy, world_step};

    fn make_world() -> *mut WorldHandle {
        world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        })
    }

    /// A rope hanging from its first particle: the official solver must keep it
    /// finite and let it sag under gravity.
    #[test]
    fn upstream_backend_steps_a_rope() {
        let world = make_world();
        assert!(!world.is_null());

        assert_eq!(
            soft_body_set_backend(world, 1),
            Bool::TRUE,
            "mode 1 must select the official backend"
        );

        // An 8-particle rope from (0,0,0) to (0,3,0), pinned at the start. This entry
        // point creates the body itself and returns its id.
        let id = soft_body_build_rope(
            world, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 8, 1.0, 1.0e-6, 8, 1, 0, 0, 0,
        );
        assert_ne!(id, u32::MAX, "rope creation must succeed");

        // `soft_body_build_rope` creates its body with zero gravity, so give it -9.81
        // explicitly: that is the driver the sag below is about.
        assert_eq!(
            soft_body_set_gravity(
                world,
                id,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            ),
            Bool::TRUE,
            "gravity must be accepted"
        );

        let count = soft_body_particle_count(world, id);
        assert_eq!(count, 8, "the rope must keep its particles");

        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
        }

        let mut positions = vec![
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            };
            count as usize
        ];
        let mut inv_masses = vec![0.0f64; count as usize];
        let written = soft_body_read_particles(
            world,
            id,
            positions.as_mut_ptr(),
            inv_masses.as_mut_ptr(),
            count,
        );
        assert_eq!(written, count, "all particles must be readable");

        for p in &positions {
            assert!(
                p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
                "official backend produced a non-finite particle: {p:?}"
            );
        }
        // The pinned end stays put; the free end sagged away from its start.
        assert!(
            positions[0].y.abs() < 1.0e-2,
            "pinned end must not move, got {:?}",
            positions[0]
        );
        assert!(
            positions[7].y < 3.0,
            "free end should sag, got y = {}",
            positions[7].y
        );

        let energy = soft_body_kinetic_energy(world, id);
        assert!(
            energy.is_finite() && energy >= 0.0,
            "kinetic energy must be finite and non-negative, got {energy}"
        );

        world_destroy(world);
    }

    #[test]
    fn unknown_backend_mode_is_rejected() {
        let world = make_world();
        assert!(!world.is_null());
        assert_eq!(
            soft_body_set_backend(world, 7),
            Bool::FALSE,
            "only 0 (fork) and 1 (upstream) are valid"
        );
        world_destroy(world);
    }
}
