#[cfg(test)]
mod tests {
    use mps_core::rapier::molecular::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::BodyStatus;

    fn particle(x: f64, charge: f64) -> MolecularParticle {
        MolecularParticle {
            position: Vec3 { x, y: 0.0, z: 0.0 },
            velocity: Vec3::default(),
            mass: 1.0,
            charge,
            epsilon: 1.0,
            sigma: 1.0,
        }
    }

    fn law() -> MolecularForceLaw {
        MolecularForceLaw {
            coulomb_constant: 1.0,
            relative_permittivity: 1.0,
            cutoff_radius: 0.0,
            softening: 0.0,
            lennard_jones_enabled: Bool::TRUE,
            coulomb_enabled: Bool::TRUE,
        }
    }

    #[test]
    fn lennard_jones_and_coulomb_formulas_work() {
        let potential = molecular_lennard_jones_potential(2.0_f64.powf(1.0 / 6.0), 1.0, 1.0);
        assert!((potential + 1.0).abs() < 1.0e-12);

        let mut force = Vec3::default();
        assert_eq!(
            molecular_lennard_jones_force(
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0
                },
                1.0,
                1.0,
                0.0,
                &mut force
            ),
            Bool::TRUE
        );
        assert!(force.x > 0.0);

        assert_eq!(
            molecular_coulomb_force(
                Vec3 {
                    x: 2.0,
                    y: 0.0,
                    z: 0.0
                },
                1.0,
                1.0,
                1.0,
                1.0,
                0.0,
                &mut force
            ),
            Bool::TRUE
        );
        assert!((force.x - 0.25).abs() < 1.0e-12);
    }

    #[test]
    fn pair_interaction_reports_force_on_first_particle() {
        let mut report = MolecularPairReport::default();
        assert_eq!(
            molecular_pair_interaction(particle(0.0, 1.0), particle(2.0, -1.0), law(), &mut report),
            Bool::TRUE
        );
        assert!(report.total_potential.is_finite());
        assert!(report.coulomb_force.x > 0.0);
    }

    #[test]
    fn applies_equal_and_opposite_forces_to_bodies() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let builder_a =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder_a, 1.0);
        let body_a = mps_core::rapier::rigid_body::rigid_body_builder_build(builder_a);
        let handle_a = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body_a);
        let builder_b =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder_b, 1.0);
        let body_b = mps_core::rapier::rigid_body::rigid_body_builder_build(builder_b);
        let handle_b = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body_b);
        let mut report = MolecularPairReport::default();

        assert_eq!(
            molecular_apply_pair_forces(
                world,
                handle_a,
                handle_b,
                particle(0.0, 1.0),
                particle(2.0, -1.0),
                law(),
                Bool::TRUE,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(report.total_force.x > 0.0);

        mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        let velocity_a = mps_core::rapier::rigid_body::rigid_body_get_linvel(world, handle_a);
        let velocity_b = mps_core::rapier::rigid_body::rigid_body_get_linvel(world, handle_b);
        assert!(velocity_a.x > 0.0);
        assert!(velocity_b.x < 0.0);
        mps_core::rapier::world::world_destroy(world);
    }
}



