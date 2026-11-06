#[cfg(all(test, feature = "anvilkit-bridge"))]
mod tests {
    use mps_core::rapier::anvilkit::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::world::{world_create, world_destroy};

    fn test_material() -> MaterialProperties {
        MaterialProperties {
            density: 2.0,
            friction: 0.6,
            restitution: 0.3,
            youngs_modulus: 2.0e11,
            poisson_ratio: 0.3,
            thermal_expansion: 1.2e-5,
        }
    }

    #[test]
    fn material_formulas_work() {
        let material = test_material();
        let mut stress = StressStrainReport::default();
        assert_eq!(
            material_stress_strain_linear(material, 0.001, 10.0, &mut stress),
            Bool::TRUE
        );
        assert!(stress.stress > 0.0);
        assert!(stress.thermal_strain > 0.0);

        let rebound = material_elastic_collision_relative_speed(-5.0, material.restitution);
        assert!(rebound > 0.0);

        let mut hertz = HertzContactReport::default();
        assert_eq!(
            material_hertz_contact_force(
                material, material, 0.5, 0.5, 0.001, 0.2, 10.0, &mut hertz,
            ),
            Bool::TRUE
        );
        assert!(hertz.normal_force > 0.0);
        assert!(hertz.contact_area > 0.0);
        assert!(hertz.total_force > hertz.normal_force);
    }

    #[test]
    fn anvilkit_entity_links_to_soft_body() {
        let app = anvilkit_app_create();
        assert!(!app.is_null());
        let world = world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        assert!(!world.is_null());

        // Spawn an anvilkit entity (a rigid body at the origin).
        let entity_bits = anvilkit_app_spawn_body(
            app,
            Vec3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            Quat {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
            0, // dynamic
        );
        assert_ne!(entity_bits, 0, "entity should spawn");

        // Bind a soft body to that entity.
        let sb_id =
            anvilkit_app_spawn_soft_body(app, world, entity_bits, 1.0, 80.0, 1.0, Bool::FALSE);
        assert!(sb_id != u32::MAX, "soft body should be created for entity");

        // The entity→soft-body reverse lookup must resolve.
        let looked_up = anvilkit_app_entity_to_soft_body(app, entity_bits);
        assert_eq!(looked_up, sb_id, "entity maps back to its soft body");

        // The soft body must exist in the world and contain the anchored particle.
        let sb = unsafe {
            (*world)
                .inner
                .soft_bodies
                .get(rapier3d::prelude::soft_body_fork::SoftBodyId(sb_id))
        };
        let sb = sb.expect("soft body present in world");
        // Phase 5c: the bound soft body is a bounding-box tetra mesh — eight corner
        // particles plus six tetrahedra (was a single point-mass before 5c).
        assert_eq!(
            sb.particles.len(),
            8,
            "bounding-box tetra soft body (8 corners)"
        );
        assert_eq!(
            sb.tetrahedra.len(),
            6,
            "bounding-box tetra soft body (6 tetrahedra)"
        );

        // A bogus entity yields no soft body.
        assert_eq!(
            anvilkit_app_entity_to_soft_body(app, 0xFFFF_FFFF_FFFF_FFFF),
            0
        );

        anvilkit_app_destroy(app);
        world_destroy(world);
    }
}
