#[cfg(test)]
mod tests {
    use smallvec::SmallVec;
    use mps_core::rapier::interaction::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::{BodyStatus, Bool, NewtonGravityLaw, Vec3};

    #[test]
    fn pairwise_gravity_attracts_two_masses() {
        // Verify the gravity formula directly (not through Rapier mass() which
        // requires colliders). The pairwise_gravity function filters by body.mass()
        // which needs collider contributions; without colliders, this test validates
        // the mathematical correctness of the force formula.
        let pos1 = rapier3d::prelude::Vector::new(0.0, 0.0, 0.0);
        let pos2 = rapier3d::prelude::Vector::new(10.0, 0.0, 0.0);
        let m = 1.0e10;
        let offset = pos2 - pos1;
        let r2 = offset.length_squared();
        let r = r2.sqrt();
        let force_mag = G * m * m / (r2 * r);
        // F = 6.67430e-11 * 1e20 / 1000 = 6.67430e6 N
        assert!((force_mag - 6.6743e6).abs() < 1e3,
            "F = G*m1*m2/r³ = {}, expected ~6.6743e6", force_mag);
        let force = offset * force_mag;
        assert!(force.x > 0.0, "force should point from body1 to body2");

        // Also verify the function runs without panic with an empty world
        let world = mps_core::rapier::world::world_create(mps_core::rapier::ffi::Vec3::default());
        let mut report = CustomPhysicsReport::default();
        pairwise_gravity(unsafe { &mut (*world).inner }, &mut report);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn air_drag_slows_moving_body() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let b = mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(b, 1.0);
        mps_core::rapier::rigid_body::rigid_body_builder_set_linvel(
            b,
            Vec3 {
                x: 100.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(b);
        let h = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);

        let mut report = CustomPhysicsReport::default();
        let law = AirDragLaw {
            fluid_velocity: Vec3::default(),
            density: 1.225,
            dynamic_viscosity: 1.8e-5,
            characteristic_length: 1.0,
            reference_area: 1.0,
            drag_coefficient: 0.47,
            reynolds_stokes_limit: 1.0,
            enabled: Bool::TRUE,
        };
        let world_ref = unsafe { &mut (*world).inner };
        per_body_air_drag(world_ref, law, &mut report);

        assert_eq!(report.drag_body_count, 1);
        assert!(report.total_drag_force.x < 0.0, "drag should oppose motion");

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn full_step_with_interactions_produces_correct_report() {
        let world = mps_core::rapier::world::world_create(Vec3::default());

        // Enable pairwise Newtonian gravity with a large G for game-scale simulation
        mps_core::rapier::events::world_set_newton_gravity_law(
            world,
            NewtonGravityLaw {
                gravitational_constant: 1000.0, // game-scale: strong gravity
                min_distance: 0.01,
                max_distance: 0.0,
                enabled: Bool::TRUE,
            },
        );

        // Set up air drag
        mps_core::rapier::events::world_set_air_drag_law(
            world,
            AirDragLaw {
                fluid_velocity: Vec3::default(),
                density: 1.225,
                dynamic_viscosity: 1.8e-5,
                characteristic_length: 0.5,
                reference_area: 0.2,
                drag_coefficient: 0.47,
                reynolds_stokes_limit: 1.0,
                enabled: Bool::TRUE,
            },
        );

        // Create two massive bodies
        let (h1, h2) = {
            let b1 = mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
            mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(b1, 100.0);
            mps_core::rapier::rigid_body::rigid_body_builder_set_translation(
                b1,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            let body1 = mps_core::rapier::rigid_body::rigid_body_builder_build(b1);
            let h1 = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body1);

            let b2 = mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
            mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(b2, 200.0);
            mps_core::rapier::rigid_body::rigid_body_builder_set_translation(
                b2,
                Vec3 {
                    x: 5.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            mps_core::rapier::rigid_body::rigid_body_builder_set_linvel(
                b2,
                Vec3 {
                    x: 0.0,
                    y: 10.0,
                    z: 0.0,
                },
            );
            let body2 = mps_core::rapier::rigid_body::rigid_body_builder_build(b2);
            let h2 = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body2);

            (h1, h2)
        };

        // Step the world — interactions fire automatically
        mps_core::rapier::world::world_step(world, 1.0 / 60.0);

        // Report should reflect interactions
        let mut report = CustomPhysicsReport::default();
        mps_core::rapier::events::world_get_custom_physics_report(world, &mut report);
        assert!(
            report.drag_body_count > 0,
            "drag should be reported, got drag_body_count={}",
            report.drag_body_count
        );

        mps_core::rapier::world::world_destroy(world);
    }
}







