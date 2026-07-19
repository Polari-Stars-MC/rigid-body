#[cfg(test)]
mod tests {
    use smallvec::SmallVec;
    use mps_core::rapier::events::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::{BodyStatus, ShapeDesc, Vec3};

    #[test]
    fn custom_air_drag_law_applies_before_world_step() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let builder =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder, 1.0);
        mps_core::rapier::rigid_body::rigid_body_builder_set_linvel(
            builder,
            Vec3 {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(builder);
        let handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);

        assert_eq!(
            world_set_air_drag_law(
                world,
                AirDragLaw {
                    fluid_velocity: Vec3::default(),
                    density: 1.225,
                    dynamic_viscosity: 1.8e-5,
                    characteristic_length: 0.1,
                    reference_area: 0.01,
                    drag_coefficient: 0.47,
                    reynolds_stokes_limit: 1.0,
                    enabled: Bool::TRUE,
                },
            ),
            Bool::TRUE
        );
        mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        let velocity = mps_core::rapier::rigid_body::rigid_body_get_linvel(world, handle);
        assert!(velocity.x < 10.0);

        let mut report = CustomPhysicsReport::default();
        assert_eq!(
            world_get_custom_physics_report(world, &mut report),
            Bool::TRUE
        );
        assert_eq!(report.drag_body_count, 1);
        assert!(report.max_reynolds_number > 1.0);
        assert!(report.total_drag_force.x < 0.0);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn coulomb_friction_law_enables_contact_modification_hook() {
        let world = mps_core::rapier::world::world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        assert_eq!(
            world_set_coulomb_friction_law(
                world,
                CoulombFrictionLaw {
                    static_coefficient: 0.9,
                    dynamic_coefficient: 0.4,
                    velocity_threshold: 0.01,
                    enabled: Bool::TRUE,
                },
            ),
            Bool::TRUE
        );

        let ground_builder =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Fixed as u32);
        mps_core::rapier::rigid_body::rigid_body_builder_set_translation(
            ground_builder,
            Vec3 {
                x: 0.0,
                y: -0.5,
                z: 0.0,
            },
        );
        let ground = mps_core::rapier::rigid_body::rigid_body_builder_build(ground_builder);
        let ground_handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, ground);
        let ground_collider = mps_core::rapier::collider::collider_builder_build(
            mps_core::rapier::collider::collider_builder_create_ex(ShapeDesc {
                shape_type: 1,
                a: 2.0,
                b: 0.25,
                c: 2.0,
                d: 0.0,
            }),
        );
        mps_core::rapier::collider::world_insert_collider_with_parent(
            world,
            ground_collider,
            ground_handle,
        );

        let body_builder =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        mps_core::rapier::rigid_body::rigid_body_builder_set_translation(
            body_builder,
            Vec3 {
                x: 0.0,
                y: 0.1,
                z: 0.0,
            },
        );
        mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(body_builder, 1.0);
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(body_builder);
        let body_handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);
        let body_collider = mps_core::rapier::collider::collider_builder_build(
            mps_core::rapier::collider::collider_builder_create_ex(ShapeDesc {
                shape_type: 1,
                a: 0.25,
                b: 0.25,
                c: 0.25,
                d: 0.0,
            }),
        );
        mps_core::rapier::collider::world_insert_collider_with_parent(
            world,
            body_collider,
            body_handle,
        );

        mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        let mut out = CoulombFrictionLaw::default();
        assert_eq!(world_get_coulomb_friction_law(world, &mut out), Bool::TRUE);
        assert_eq!(out.enabled, Bool::TRUE);
        assert_eq!(out.dynamic_coefficient, 0.4);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn external_force_law_applies_buoyancy_em_elastic_and_gravity() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let builder =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder, 2.0);
        mps_core::rapier::rigid_body::rigid_body_builder_set_translation(
            builder,
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
        );
        mps_core::rapier::rigid_body::rigid_body_builder_set_linvel(
            builder,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
        );
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(builder);
        let handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);

        assert_eq!(
            world_set_external_force_law(
                world,
                ExternalForceLaw {
                    buoyancy_enabled: Bool::TRUE,
                    fluid_density: 1.0,
                    displaced_volume: 1.0,
                    buoyancy_gravity: Vec3 {
                        x: 0.0,
                        y: -9.81,
                        z: 0.0,
                    },
                    electromagnetic_enabled: Bool::TRUE,
                    charge: 2.0,
                    electric_field: Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 1.0,
                    },
                    magnetic_field: Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 1.0,
                    },
                    elastic_enabled: Bool::TRUE,
                    spring_anchor: Vec3::default(),
                    spring_stiffness: 4.0,
                    spring_damping: 0.1,
                    gravity_enabled: Bool::TRUE,
                    gravity_source: Vec3::default(),
                    gravitational_parameter: 3.0,
                    enabled: Bool::TRUE,
                },
            ),
            Bool::TRUE
        );

        mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        let velocity = mps_core::rapier::rigid_body::rigid_body_get_linvel(world, handle);
        assert!(velocity.x < 0.0);
        assert!(velocity.y > 1.0);
        assert!(velocity.z > 0.0);

        let mut report = CustomPhysicsReport::default();
        assert_eq!(
            world_get_custom_physics_report(world, &mut report),
            Bool::TRUE
        );
        assert_eq!(report.external_force_body_count, 1);
        assert!(report.total_external_force.x < 0.0);
        assert!(report.total_external_force.y > 0.0);
        assert!(report.total_external_force.z > 0.0);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn event_ring_buffer_produces_and_drains_events() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        // Init ring buffer
        assert_eq!(
            world_init_collision_event_ring(world, 64),
            Bool::TRUE
        );
        assert_eq!(
            world_init_contact_force_event_ring(world, 64),
            Bool::TRUE
        );
        // Set dispatch mode to Both so ring buffer gets filled
        assert_eq!(world_set_event_dispatch_mode(world, 2), Bool::TRUE);

        // Create two colliding bodies with collision events enabled
        let ground = mps_core::rapier::rigid_body::rigid_body_builder_build(
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Fixed as u32),
        );
        let ground_handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, ground);
        let gc_builder = mps_core::rapier::collider::collider_builder_create_ex(ShapeDesc {
            shape_type: 1,
            a: 2.0,
            b: 0.25,
            c: 2.0,
            d: 0.0,
        });
        // Enable collision events so the ring buffer receives them
        mps_core::rapier::collider::collider_builder_set_active_events(
            gc_builder,
            1, // COLLISION_EVENTS = 1
        );
        let gc = mps_core::rapier::collider::collider_builder_build(gc_builder);
        mps_core::rapier::collider::world_insert_collider_with_parent(world, gc, ground_handle);

        let body_b = mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        mps_core::rapier::rigid_body::rigid_body_builder_set_translation(
            body_b,
            Vec3 {
                x: 0.0,
                y: 0.5,
                z: 0.0,
            },
        );
        mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(body_b, 1.0);
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(body_b);
        let body_handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);
        let bc_builder = mps_core::rapier::collider::collider_builder_create_ex(ShapeDesc {
            shape_type: 1,
            a: 0.25,
            b: 0.25,
            c: 0.25,
            d: 0.0,
        });
        mps_core::rapier::collider::collider_builder_set_active_events(bc_builder, 1);
        let bc = mps_core::rapier::collider::collider_builder_build(bc_builder);
        mps_core::rapier::collider::world_insert_collider_with_parent(world, bc, body_handle);

        // Step — collision should occur
        mps_core::rapier::world::world_step(world, 1.0 / 60.0);

        // Ring buffer should have events
        let len = world_collision_event_ring_len(world);
        assert!(len > 0, "expected collision events in ring buffer");

        // Drain ring buffer
        let mut out = vec![CollisionEventRecord::default(); 64];
        let drained = world_drain_collision_event_ring(world, out.as_mut_ptr(), 64);
        assert_eq!(drained, len);

        // After drain, ring should be empty
        assert_eq!(world_collision_event_ring_len(world), 0);

        // Stats should reflect capacity
        let mut stats = EventRingBufferStats::default();
        assert_eq!(
            world_collision_event_ring_stats(world, &mut stats),
            Bool::TRUE
        );
        assert_eq!(stats.capacity, 64);
        assert_eq!(stats.len, 0);
        assert_eq!(stats.dropped, 0);

        // Clear rings
        world_clear_event_rings(world);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn callback_registration_and_unregistration() {
        let world = mps_core::rapier::world::world_create(Vec3::default());

        // Register callback (pass 0 as fn ptr — valid "no-op" registration test)
        let handle = world_register_collision_callback(world, 0, 42);
        assert_ne!(handle, 0, "callback handle should be non-zero");

        // Set dispatch mode
        assert_eq!(world_set_event_dispatch_mode(world, 2), Bool::TRUE); // Both

        // Unregister
        world_unregister_callback(world, handle);

        // Unregister with zero handle is no-op
        world_unregister_callback(world, 0);

        mps_core::rapier::world::world_destroy(world);
    }
}







