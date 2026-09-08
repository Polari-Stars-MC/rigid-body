#[cfg(test)]
mod tests {
    use mps_core::rapier::error::{
        ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ERR_OK, ERR_UNSUPPORTED,
        last_error_code,
    };
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
        assert_eq!(world_init_collision_event_ring(world, 64), Bool::TRUE);
        assert_eq!(world_init_contact_force_event_ring(world, 64), Bool::TRUE);
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
            gc_builder, 1, // COLLISION_EVENTS = 1
        );
        let gc = mps_core::rapier::collider::collider_builder_build(gc_builder);
        mps_core::rapier::collider::world_insert_collider_with_parent(world, gc, ground_handle);

        let body_b =
            mps_core::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
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

    fn valid_coulomb_law() -> CoulombFrictionLaw {
        CoulombFrictionLaw {
            static_coefficient: 0.9,
            dynamic_coefficient: 0.4,
            velocity_threshold: 0.01,
            enabled: Bool::TRUE,
        }
    }

    fn valid_air_drag_law() -> AirDragLaw {
        AirDragLaw {
            fluid_velocity: Vec3::default(),
            density: 1.225,
            dynamic_viscosity: 1.8e-5,
            characteristic_length: 0.1,
            reference_area: 0.01,
            drag_coefficient: 0.47,
            reynolds_stokes_limit: 1.0,
            enabled: Bool::TRUE,
        }
    }

    fn valid_external_force_law() -> ExternalForceLaw {
        ExternalForceLaw {
            buoyancy_enabled: Bool::FALSE,
            fluid_density: 1.0,
            displaced_volume: 1.0,
            buoyancy_gravity: Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
            electromagnetic_enabled: Bool::FALSE,
            charge: 1.0,
            electric_field: Vec3::default(),
            magnetic_field: Vec3::default(),
            elastic_enabled: Bool::FALSE,
            spring_anchor: Vec3::default(),
            spring_stiffness: 1.0,
            spring_damping: 0.1,
            gravity_enabled: Bool::FALSE,
            gravity_source: Vec3::default(),
            gravitational_parameter: 1.0,
            enabled: Bool::TRUE,
        }
    }

    #[test]
    fn custom_physics_law_setters_reject_null_world() {
        assert_eq!(
            world_set_coulomb_friction_law(std::ptr::null_mut(), valid_coulomb_law()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert_eq!(
            world_set_air_drag_law(std::ptr::null_mut(), valid_air_drag_law()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert_eq!(
            world_set_external_force_law(std::ptr::null_mut(), valid_external_force_law()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert_eq!(
            world_set_newton_gravity_law(std::ptr::null_mut(), NewtonGravityLaw::default()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    #[test]
    fn custom_physics_law_getters_reject_null_pointers() {
        let world = mps_core::rapier::world::world_create(Vec3::default());

        let mut law = CoulombFrictionLaw::default();
        assert_eq!(
            world_get_coulomb_friction_law(std::ptr::null(), &mut law),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_get_coulomb_friction_law(world, std::ptr::null_mut()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        let mut drag = AirDragLaw::default();
        assert_eq!(
            world_get_air_drag_law(std::ptr::null(), &mut drag),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_get_air_drag_law(world, std::ptr::null_mut()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        let mut external = ExternalForceLaw::default();
        assert_eq!(
            world_get_external_force_law(std::ptr::null(), &mut external),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_get_external_force_law(world, std::ptr::null_mut()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        let mut gravity = NewtonGravityLaw::default();
        assert_eq!(
            world_get_newton_gravity_law(std::ptr::null(), &mut gravity),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_get_newton_gravity_law(world, std::ptr::null_mut()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        let mut report = CustomPhysicsReport::default();
        assert_eq!(
            world_get_custom_physics_report(std::ptr::null(), &mut report),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_get_custom_physics_report(world, std::ptr::null_mut()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn coulomb_friction_law_rejects_invalid_coefficients() {
        let world = mps_core::rapier::world::world_create(Vec3::default());

        let nan_static = CoulombFrictionLaw {
            static_coefficient: f64::NAN,
            ..valid_coulomb_law()
        };
        assert_eq!(
            world_set_coulomb_friction_law(world, nan_static),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let negative_dynamic = CoulombFrictionLaw {
            dynamic_coefficient: -0.1,
            ..valid_coulomb_law()
        };
        assert_eq!(
            world_set_coulomb_friction_law(world, negative_dynamic),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let infinite_threshold = CoulombFrictionLaw {
            velocity_threshold: f64::INFINITY,
            ..valid_coulomb_law()
        };
        assert_eq!(
            world_set_coulomb_friction_law_flag(world, infinite_threshold),
            0
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // A valid law clears the error slot again.
        assert_eq!(
            world_set_coulomb_friction_law(world, valid_coulomb_law()),
            Bool::TRUE
        );
        assert_eq!(last_error_code(), ERR_OK);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn air_drag_law_rejects_invalid_parameters() {
        let world = mps_core::rapier::world::world_create(Vec3::default());

        let nan_density = AirDragLaw {
            density: f64::NAN,
            ..valid_air_drag_law()
        };
        assert_eq!(world_set_air_drag_law(world, nan_density), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // dynamic_viscosity must be strictly positive.
        let zero_viscosity = AirDragLaw {
            dynamic_viscosity: 0.0,
            ..valid_air_drag_law()
        };
        assert_eq!(world_set_air_drag_law(world, zero_viscosity), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // characteristic_length must be strictly positive.
        let negative_length = AirDragLaw {
            characteristic_length: -1.0,
            ..valid_air_drag_law()
        };
        assert_eq!(world_set_air_drag_law_flag(world, negative_length), 0);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn external_force_law_rejects_invalid_parameters() {
        let world = mps_core::rapier::world::world_create(Vec3::default());

        let nan_charge = ExternalForceLaw {
            charge: f64::NAN,
            ..valid_external_force_law()
        };
        assert_eq!(world_set_external_force_law(world, nan_charge), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let negative_density = ExternalForceLaw {
            fluid_density: -1.0,
            ..valid_external_force_law()
        };
        assert_eq!(
            world_set_external_force_law(world, negative_density),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let negative_stiffness = ExternalForceLaw {
            spring_stiffness: -0.5,
            ..valid_external_force_law()
        };
        assert_eq!(
            world_set_external_force_law_flag(world, negative_stiffness),
            0
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn newton_gravity_law_rejects_invalid_parameters() {
        let world = mps_core::rapier::world::world_create(Vec3::default());

        // The default law is valid and accepted.
        assert_eq!(
            world_set_newton_gravity_law(world, NewtonGravityLaw::default()),
            Bool::TRUE
        );
        assert_eq!(last_error_code(), ERR_OK);

        // min_distance must be strictly positive.
        let zero_min_distance = NewtonGravityLaw {
            min_distance: 0.0,
            ..NewtonGravityLaw::default()
        };
        assert_eq!(
            world_set_newton_gravity_law(world, zero_min_distance),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let negative_constant = NewtonGravityLaw {
            gravitational_constant: -1.0,
            ..NewtonGravityLaw::default()
        };
        assert_eq!(
            world_set_newton_gravity_law(world, negative_constant),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let nan_max_distance = NewtonGravityLaw {
            max_distance: f64::NAN,
            ..NewtonGravityLaw::default()
        };
        assert_eq!(
            world_set_newton_gravity_law_flag(world, nan_max_distance),
            0
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn legacy_event_reads_reject_null_and_invalid_capacity() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let mut collision_out = vec![CollisionEventRecord::default(); 8];
        let mut force_out = vec![ContactForceEventRecord::default(); 8];

        // Null world.
        assert_eq!(
            world_get_collision_events(std::ptr::null(), collision_out.as_mut_ptr(), 8),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_get_contact_force_events(std::ptr::null(), force_out.as_mut_ptr(), 8),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Null output buffer.
        assert_eq!(
            world_get_collision_events(world, std::ptr::null_mut(), 8),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_get_contact_force_events(world, std::ptr::null_mut(), 8),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Zero capacity and capacity above MAX_OUTPUT_CAPACITY (1_000_000).
        assert_eq!(
            world_get_collision_events(world, collision_out.as_mut_ptr(), 0),
            0
        );
        assert_eq!(last_error_code(), ERR_CAPACITY);
        assert_eq!(
            world_get_contact_force_events(world, force_out.as_mut_ptr(), 1_000_001),
            0
        );
        assert_eq!(last_error_code(), ERR_CAPACITY);

        // Null-world scalar reads return failure sentinels.
        assert_eq!(world_collision_event_count(std::ptr::null()), 0);
        assert_eq!(world_contact_force_event_count(std::ptr::null()), 0);
        let record = world_get_collision_event(std::ptr::null(), 0);
        assert_eq!(record.collider1, 0);
        let record = world_get_contact_force_event(std::ptr::null(), 0);
        assert_eq!(record.collider1, 0);

        // Out-of-range index on a valid world returns a zeroed record.
        let record = world_get_collision_event(world, 42);
        assert_eq!(record.collider1, 0);
        assert_eq!(record.started, Bool::FALSE);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn event_ring_init_rejects_null_world_and_invalid_capacity() {
        let world = mps_core::rapier::world::world_create(Vec3::default());

        assert_eq!(
            world_init_collision_event_ring(std::ptr::null_mut(), 64),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_init_contact_force_event_ring(std::ptr::null_mut(), 64),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Capacity 0 and capacity above MAX_OUTPUT_CAPACITY (1_000_000).
        assert_eq!(world_init_collision_event_ring(world, 0), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_CAPACITY);
        assert_eq!(
            world_init_contact_force_event_ring(world, 1_000_001),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_CAPACITY);

        // A valid init still succeeds after the failures.
        assert_eq!(world_init_collision_event_ring(world, 64), Bool::TRUE);
        assert_eq!(last_error_code(), ERR_OK);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn event_ring_drain_rejects_invalid_arguments() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let mut collision_out = vec![CollisionEventRecord::default(); 8];
        let mut force_out = vec![ContactForceEventRecord::default(); 8];

        // Null world.
        assert_eq!(
            world_drain_collision_event_ring(std::ptr::null(), collision_out.as_mut_ptr(), 8),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_drain_contact_force_event_ring(std::ptr::null(), force_out.as_mut_ptr(), 8),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Null output, zero capacity and over-limit capacity all report ERR_CAPACITY.
        assert_eq!(
            world_drain_collision_event_ring(world, std::ptr::null_mut(), 8),
            0
        );
        assert_eq!(last_error_code(), ERR_CAPACITY);
        assert_eq!(
            world_drain_contact_force_event_ring(world, force_out.as_mut_ptr(), 0),
            0
        );
        assert_eq!(last_error_code(), ERR_CAPACITY);
        assert_eq!(
            world_drain_collision_event_ring(world, collision_out.as_mut_ptr(), 1_000_001),
            0
        );
        assert_eq!(last_error_code(), ERR_CAPACITY);

        // Draining an uninitialized ring is valid and yields zero events.
        assert_eq!(
            world_drain_contact_force_event_ring(world, force_out.as_mut_ptr(), 8),
            0
        );
        assert_eq!(last_error_code(), ERR_OK);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn event_ring_stats_and_len_reject_null_pointers() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let mut stats = EventRingBufferStats::default();

        assert_eq!(
            world_collision_event_ring_stats(std::ptr::null(), &mut stats),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_collision_event_ring_stats(world, std::ptr::null_mut()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert_eq!(
            world_contact_force_event_ring_stats(std::ptr::null(), &mut stats),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_contact_force_event_ring_stats(world, std::ptr::null_mut()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Null-world ring lengths return 0.
        assert_eq!(world_collision_event_ring_len(std::ptr::null()), 0);
        assert_eq!(world_contact_force_event_ring_len(std::ptr::null()), 0);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn callback_registration_rejects_null_world() {
        assert_eq!(
            world_register_collision_callback(std::ptr::null_mut(), 0, 0),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_register_contact_force_callback(std::ptr::null_mut(), 0, 0),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Unregistering on a null world is a no-op and must not crash.
        world_unregister_callback(std::ptr::null_mut(), 1);
    }

    #[test]
    fn dispatch_mode_rejects_invalid_value() {
        let world = mps_core::rapier::world::world_create(Vec3::default());

        assert_eq!(
            world_set_event_dispatch_mode(std::ptr::null_mut(), 0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        assert_eq!(world_set_event_dispatch_mode(world, 3), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(world_set_event_dispatch_mode(world, u32::MAX), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // All documented modes are accepted.
        for mode in 0..=2 {
            assert_eq!(world_set_event_dispatch_mode(world, mode), Bool::TRUE);
            assert_eq!(last_error_code(), ERR_OK);
        }

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn disabled_pair_filter_callbacks_report_unsupported() {
        let world = mps_core::rapier::world::world_create(Vec3::default());

        world_set_contact_pair_filter_callback(std::ptr::null_mut(), 0, 0);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        world_set_intersection_pair_filter_callback(std::ptr::null_mut(), 0, 0);
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // External pair-filter callbacks are disabled for ABI safety.
        world_set_contact_pair_filter_callback(world, 0, 0);
        assert_eq!(last_error_code(), ERR_UNSUPPORTED);
        world_set_intersection_pair_filter_callback(world, 0, 0);
        assert_eq!(last_error_code(), ERR_UNSUPPORTED);

        mps_core::rapier::world::world_destroy(world);
    }

    // =========================================================================
    // PHYSICS_EXPANSION_PLAN C1: solar-wind pressure / dynamical-friction /
    // MOND gravity force-law FFI mirroring (world_set_* / world_clear_*).
    // Each law accepts a valid configuration, rejects NaN / non-positive
    // parameters, and round-trips a clear → set → clear sequence.
    // =========================================================================

    #[test]
    fn solar_wind_pressure_law_accepts_valid_config() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let law = SolarWindPressureLaw {
            proton_density: 5.0e6,
            v_sw_mps: 400.0,
            wind_direction: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            effective_area_m2: 10.0,
            enabled: Bool::TRUE,
        };
        assert_eq!(world_set_solar_wind_pressure_law(world, law), Bool::TRUE);
        assert_eq!(last_error_code(), ERR_OK);
        // Disable path: enabled=0 should also succeed (clears prior law).
        let disabled = SolarWindPressureLaw {
            enabled: Bool::FALSE,
            ..law
        };
        assert_eq!(
            world_set_solar_wind_pressure_law(world, disabled),
            Bool::TRUE
        );
        assert_eq!(last_error_code(), ERR_OK);
        // flag variant returns 1.
        let one = world_set_solar_wind_pressure_law_flag(world, law);
        assert_eq!(one, 1);
        world_clear_solar_wind_pressure_law(world);
        assert_eq!(last_error_code(), ERR_OK);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn solar_wind_pressure_law_rejects_invalid_parameters() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let base = SolarWindPressureLaw {
            proton_density: 5.0e6,
            v_sw_mps: 400.0,
            wind_direction: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            effective_area_m2: 10.0,
            enabled: Bool::TRUE,
        };
        // zero proton_density
        let bad = SolarWindPressureLaw {
            proton_density: 0.0,
            ..base
        };
        assert_eq!(world_set_solar_wind_pressure_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        // negative v_sw_mps
        let bad = SolarWindPressureLaw {
            v_sw_mps: -1.0,
            ..base
        };
        assert_eq!(world_set_solar_wind_pressure_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        // zero effective_area
        let bad = SolarWindPressureLaw {
            effective_area_m2: 0.0,
            ..base
        };
        assert_eq!(world_set_solar_wind_pressure_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        // NaN direction
        let bad = SolarWindPressureLaw {
            wind_direction: Vec3 {
                x: f64::NAN,
                y: 0.0,
                z: 0.0,
            },
            ..base
        };
        assert_eq!(world_set_solar_wind_pressure_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        // null world
        assert_eq!(
            world_set_solar_wind_pressure_law(std::ptr::null_mut(), base),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn dynamical_friction_law_accepts_valid_config() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let law = DynamicalFrictionLaw {
            background_density_kg_m3: 1.0e-21,
            coulomb_log: 10.0,
            enabled: Bool::TRUE,
        };
        assert_eq!(world_set_dynamical_friction_law(world, law), Bool::TRUE);
        assert_eq!(last_error_code(), ERR_OK);
        let disabled = DynamicalFrictionLaw {
            enabled: Bool::FALSE,
            ..law
        };
        assert_eq!(
            world_set_dynamical_friction_law(world, disabled),
            Bool::TRUE
        );
        let one = world_set_dynamical_friction_law_flag(world, law);
        assert_eq!(one, 1);
        world_clear_dynamical_friction_law(world);
        assert_eq!(last_error_code(), ERR_OK);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn dynamical_friction_law_rejects_invalid_parameters() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let base = DynamicalFrictionLaw {
            background_density_kg_m3: 1.0e-21,
            coulomb_log: 10.0,
            enabled: Bool::TRUE,
        };
        let bad = DynamicalFrictionLaw {
            background_density_kg_m3: 0.0,
            ..base
        };
        assert_eq!(world_set_dynamical_friction_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = DynamicalFrictionLaw {
            coulomb_log: 0.0,
            ..base
        };
        assert_eq!(world_set_dynamical_friction_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = DynamicalFrictionLaw {
            background_density_kg_m3: f64::NAN,
            ..base
        };
        assert_eq!(world_set_dynamical_friction_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            world_set_dynamical_friction_law(std::ptr::null_mut(), base),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn mond_gravity_law_accepts_valid_config() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let law = MonDGravityLaw {
            newtonian_a: 1.0e-10,
            mond_a_zero: 1.2e-10,
            direction: Vec3 {
                x: 0.0,
                y: -1.0,
                z: 0.0,
            },
            enabled: Bool::TRUE,
        };
        assert_eq!(world_set_mond_gravity_law(world, law), Bool::TRUE);
        assert_eq!(last_error_code(), ERR_OK);
        let disabled = MonDGravityLaw {
            enabled: Bool::FALSE,
            ..law
        };
        assert_eq!(world_set_mond_gravity_law(world, disabled), Bool::TRUE);
        let one = world_set_mond_gravity_law_flag(world, law);
        assert_eq!(one, 1);
        world_clear_mond_gravity_law(world);
        assert_eq!(last_error_code(), ERR_OK);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn mond_gravity_law_rejects_invalid_parameters() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let base = MonDGravityLaw {
            newtonian_a: 1.0e-10,
            mond_a_zero: 1.2e-10,
            direction: Vec3 {
                x: 0.0,
                y: -1.0,
                z: 0.0,
            },
            enabled: Bool::TRUE,
        };
        let bad = MonDGravityLaw {
            mond_a_zero: 0.0,
            ..base
        };
        assert_eq!(world_set_mond_gravity_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = MonDGravityLaw {
            newtonian_a: -1.0,
            ..base
        };
        assert_eq!(world_set_mond_gravity_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = MonDGravityLaw {
            direction: Vec3 {
                x: f64::NAN,
                y: 0.0,
                z: 0.0,
            },
            ..base
        };
        assert_eq!(world_set_mond_gravity_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            world_set_mond_gravity_law(std::ptr::null_mut(), base),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        mps_core::rapier::world::world_destroy(world);
    }

    // =========================================================================
    // PHYSICS_EXPANSION_PLAN C2: Eddington-limited radiation-pressure force-law
    // FFI mirroring (world_set_eddington_radiation_pressure_law &
    // world_clear_eddington_radiation_pressure_law).
    // =========================================================================

    #[test]
    fn eddington_radiation_pressure_law_accepts_valid_config() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let law = EddingtonRadiationPressureLaw {
            mass_kg: 1.989e31,
            opacity: 0.034,
            source_position: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            effective_area_m2: 1.0,
            enabled: Bool::TRUE,
        };
        assert_eq!(
            world_set_eddington_radiation_pressure_law(world, law),
            Bool::TRUE
        );
        assert_eq!(last_error_code(), ERR_OK);
        let disabled = EddingtonRadiationPressureLaw {
            enabled: Bool::FALSE,
            ..law
        };
        assert_eq!(
            world_set_eddington_radiation_pressure_law(world, disabled),
            Bool::TRUE
        );
        assert_eq!(last_error_code(), ERR_OK);
        let one = world_set_eddington_radiation_pressure_law_flag(world, law);
        assert_eq!(one, 1);
        world_clear_eddington_radiation_pressure_law(world);
        assert_eq!(last_error_code(), ERR_OK);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn eddington_radiation_pressure_law_rejects_invalid_parameters() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let base = EddingtonRadiationPressureLaw {
            mass_kg: 1.989e31,
            opacity: 0.034,
            source_position: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            effective_area_m2: 1.0,
            enabled: Bool::TRUE,
        };
        let bad = EddingtonRadiationPressureLaw {
            mass_kg: 0.0,
            ..base
        };
        assert_eq!(
            world_set_eddington_radiation_pressure_law(world, bad),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = EddingtonRadiationPressureLaw {
            opacity: 0.0,
            ..base
        };
        assert_eq!(
            world_set_eddington_radiation_pressure_law(world, bad),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = EddingtonRadiationPressureLaw {
            effective_area_m2: 0.0,
            ..base
        };
        assert_eq!(
            world_set_eddington_radiation_pressure_law(world, bad),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = EddingtonRadiationPressureLaw {
            source_position: Vec3 {
                x: f64::NAN,
                y: 0.0,
                z: 0.0,
            },
            ..base
        };
        assert_eq!(
            world_set_eddington_radiation_pressure_law(world, bad),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            world_set_eddington_radiation_pressure_law(std::ptr::null_mut(), base),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        mps_core::rapier::world::world_destroy(world);
    }

    // =========================================================================
    // PHYSICS_EXPANSION_PLAN C3: X-ray disc bolometric irradiation force-law
    // FFI mirroring (world_set_xray_irradiation_law & world_clear_xray_irradiation_law).
    // =========================================================================

    #[test]
    fn xray_irradiation_law_accepts_valid_config() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let law = XrayIrradiationLaw {
            k_t_eff_kev: 1.0,
            r_in_km: 10.0,
            spectral_hardening: 1.7,
            source_position: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            effective_area_m2: 1.0,
            enabled: Bool::TRUE,
        };
        assert_eq!(world_set_xray_irradiation_law(world, law), Bool::TRUE);
        assert_eq!(last_error_code(), ERR_OK);
        let disabled = XrayIrradiationLaw {
            enabled: Bool::FALSE,
            ..law
        };
        assert_eq!(world_set_xray_irradiation_law(world, disabled), Bool::TRUE);
        assert_eq!(last_error_code(), ERR_OK);
        let one = world_set_xray_irradiation_law_flag(world, law);
        assert_eq!(one, 1);
        world_clear_xray_irradiation_law(world);
        assert_eq!(last_error_code(), ERR_OK);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn xray_irradiation_law_rejects_invalid_parameters() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let base = XrayIrradiationLaw {
            k_t_eff_kev: 1.0,
            r_in_km: 10.0,
            spectral_hardening: 1.7,
            source_position: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            effective_area_m2: 1.0,
            enabled: Bool::TRUE,
        };
        let bad = XrayIrradiationLaw {
            k_t_eff_kev: 0.0,
            ..base
        };
        assert_eq!(world_set_xray_irradiation_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = XrayIrradiationLaw {
            r_in_km: 0.0,
            ..base
        };
        assert_eq!(world_set_xray_irradiation_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = XrayIrradiationLaw {
            spectral_hardening: 0.0,
            ..base
        };
        assert_eq!(world_set_xray_irradiation_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = XrayIrradiationLaw {
            source_position: Vec3 {
                x: 0.0,
                y: f64::NAN,
                z: 0.0,
            },
            ..base
        };
        assert_eq!(world_set_xray_irradiation_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            world_set_xray_irradiation_law(std::ptr::null_mut(), base),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        mps_core::rapier::world::world_destroy(world);
    }

    // =========================================================================
    // PHYSICS_EXPANSION_PLAN C3: Pulsar magnetic-dipole torque force-law
    // FFI mirroring (world_set_pulsar_magnetic_dipole_law &
    // world_clear_pulsar_magnetic_dipole_law).
    // =========================================================================

    #[test]
    fn pulsar_magnetic_dipole_law_accepts_valid_config() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let law = PulsarMagneticDipoleLaw {
            moment_of_inertia: 1.0e38, // typical NS
            ns_radius_m: 1.0e4,        // 10 km
            period_ms: 33.4,           // Crab-like
            period_derivative: 4.2e-13,
            pulsar_position: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            spin_axis: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            body_dipole_moment: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            enabled: Bool::TRUE,
        };
        assert_eq!(world_set_pulsar_magnetic_dipole_law(world, law), Bool::TRUE);
        assert_eq!(last_error_code(), ERR_OK);
        let disabled = PulsarMagneticDipoleLaw {
            enabled: Bool::FALSE,
            ..law
        };
        assert_eq!(
            world_set_pulsar_magnetic_dipole_law(world, disabled),
            Bool::TRUE
        );
        assert_eq!(last_error_code(), ERR_OK);
        let one = world_set_pulsar_magnetic_dipole_law_flag(world, law);
        assert_eq!(one, 1);
        world_clear_pulsar_magnetic_dipole_law(world);
        assert_eq!(last_error_code(), ERR_OK);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn pulsar_magnetic_dipole_law_rejects_invalid_parameters() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let base = PulsarMagneticDipoleLaw {
            moment_of_inertia: 1.0e38,
            ns_radius_m: 1.0e4,
            period_ms: 33.4,
            period_derivative: 4.2e-13,
            pulsar_position: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            spin_axis: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            body_dipole_moment: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            enabled: Bool::TRUE,
        };
        let bad = PulsarMagneticDipoleLaw {
            moment_of_inertia: 0.0,
            ..base
        };
        assert_eq!(
            world_set_pulsar_magnetic_dipole_law(world, bad),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = PulsarMagneticDipoleLaw {
            ns_radius_m: 0.0,
            ..base
        };
        assert_eq!(
            world_set_pulsar_magnetic_dipole_law(world, bad),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = PulsarMagneticDipoleLaw {
            period_ms: 0.0,
            ..base
        };
        assert_eq!(
            world_set_pulsar_magnetic_dipole_law(world, bad),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = PulsarMagneticDipoleLaw {
            period_derivative: 0.0,
            ..base
        };
        assert_eq!(
            world_set_pulsar_magnetic_dipole_law(world, bad),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = PulsarMagneticDipoleLaw {
            spin_axis: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            ..base
        };
        // spin_axis length=0: still passes FFI validation (vec3_finite passes
        // (0,0,0)); the apply() guards the zero-length path. The FFI accepts
        // the config — the law will just no-op in apply.
        assert_eq!(world_set_pulsar_magnetic_dipole_law(world, bad), Bool::TRUE);
        let bad = PulsarMagneticDipoleLaw {
            pulsar_position: Vec3 {
                x: f64::NAN,
                y: 0.0,
                z: 0.0,
            },
            ..base
        };
        assert_eq!(
            world_set_pulsar_magnetic_dipole_law(world, bad),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            world_set_pulsar_magnetic_dipole_law(std::ptr::null_mut(), base),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        mps_core::rapier::world::world_destroy(world);
    }

    // =========================================================================
    // PHYSICS_EXPANSION_PLAN C4: Jeans-escape drag force-law
    // FFI mirroring (world_set_jeans_escape_law & world_clear_jeans_escape_law).
    // =========================================================================

    #[test]
    fn jeans_escape_law_accepts_valid_config() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let law = JeansEscapeLaw {
            n_exo: 1.0e12,         // 1e12 m⁻³ — exobase density
            temperature: 1000.0,   // 1000 K exobase
            escape_parameter: 7.5, // Earth H λ ≈ 7.5
            mass_kg: 1.673e-27,    // H atom ~ 1.673e-27 kg
            escape_direction: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            effective_area_m2: 1.0,
            enabled: Bool::TRUE,
        };
        assert_eq!(world_set_jeans_escape_law(world, law), Bool::TRUE);
        assert_eq!(last_error_code(), ERR_OK);
        let disabled = JeansEscapeLaw {
            enabled: Bool::FALSE,
            ..law
        };
        assert_eq!(world_set_jeans_escape_law(world, disabled), Bool::TRUE);
        assert_eq!(last_error_code(), ERR_OK);
        let one = world_set_jeans_escape_law_flag(world, law);
        assert_eq!(one, 1);
        world_clear_jeans_escape_law(world);
        assert_eq!(last_error_code(), ERR_OK);
        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn jeans_escape_law_rejects_invalid_parameters() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let base = JeansEscapeLaw {
            n_exo: 1.0e12,
            temperature: 1000.0,
            escape_parameter: 7.5,
            mass_kg: 1.673e-27,
            escape_direction: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            effective_area_m2: 1.0,
            enabled: Bool::TRUE,
        };
        let bad = JeansEscapeLaw { n_exo: 0.0, ..base };
        assert_eq!(world_set_jeans_escape_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = JeansEscapeLaw {
            temperature: 0.0,
            ..base
        };
        assert_eq!(world_set_jeans_escape_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = JeansEscapeLaw {
            escape_parameter: -1.0,
            ..base
        };
        assert_eq!(world_set_jeans_escape_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = JeansEscapeLaw {
            mass_kg: 0.0,
            ..base
        };
        assert_eq!(world_set_jeans_escape_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = JeansEscapeLaw {
            effective_area_m2: 0.0,
            ..base
        };
        assert_eq!(world_set_jeans_escape_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        let bad = JeansEscapeLaw {
            escape_direction: Vec3 {
                x: f64::NAN,
                y: 0.0,
                z: 0.0,
            },
            ..base
        };
        assert_eq!(world_set_jeans_escape_law(world, bad), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            world_set_jeans_escape_law(std::ptr::null_mut(), base),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        mps_core::rapier::world::world_destroy(world);
    }
}

/// Ad-hoc verification that the rapier3d 0.35 hoist of `friction` off of
/// per-point `SolverContact` onto `ContactModificationContext::friction` is
/// honored end-to-end via the public FFI code path.
///
/// Strategy: drop an identical slab onto a fixed floor with horizontal velocity
/// twice -- once with the Coulomb friction hook mu = 0 (frictionless), once with
/// mu = 1 (sticky). If the hook is wired into `context.friction` correctly, the
/// sticky case must lose noticeably more horizontal speed than the
/// frictionless case after the same number of steps. If the hook were a no-op
/// both cases would coast identically at the default combined friction.
#[cfg(test)]
mod verify_friction_hoist {
    use mps_core::rapier::collider::collider_builder_build;
    use mps_core::rapier::collider::collider_builder_create_ex;
    use mps_core::rapier::collider::world_insert_collider_with_parent;
    use mps_core::rapier::events::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::rigid_body::rigid_body_builder_build;
    use mps_core::rapier::rigid_body::rigid_body_builder_create;
    use mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass;
    use mps_core::rapier::rigid_body::rigid_body_builder_set_linear_damping;
    use mps_core::rapier::rigid_body::rigid_body_builder_set_linvel;
    use mps_core::rapier::rigid_body::rigid_body_builder_set_translation;
    use mps_core::rapier::rigid_body::rigid_body_get_linvel_out;
    use mps_core::rapier::rigid_body::world_insert_rigid_body;
    use mps_core::rapier::world::{world_create, world_destroy, world_step};

    /// Build a 0.5 cuboid resting on a large fixed floor, sliding at vx = 5.0.
    fn run_slide(mu: f64) -> (f64, f64) {
        let world = world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        assert_eq!(
            world_set_coulomb_friction_law(
                world,
                CoulombFrictionLaw {
                    static_coefficient: mu,
                    dynamic_coefficient: mu,
                    velocity_threshold: 0.01,
                    enabled: Bool::TRUE,
                },
            ),
            Bool::TRUE
        );

        let ground_builder = rigid_body_builder_create(BodyStatus::Fixed as u32);
        rigid_body_builder_set_translation(
            ground_builder,
            Vec3 {
                x: 0.0,
                y: -0.5,
                z: 0.0,
            },
        );
        let ground = rigid_body_builder_build(ground_builder);
        let ground_handle = world_insert_rigid_body(world, ground);
        let ground_collider = collider_builder_build(collider_builder_create_ex(ShapeDesc {
            shape_type: 1,
            a: 5.0,
            b: 0.25,
            c: 5.0,
            d: 0.0,
        }));
        world_insert_collider_with_parent(world, ground_collider, ground_handle);

        let body_builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_translation(
            body_builder,
            Vec3 {
                x: 0.0,
                y: 0.05,
                z: 0.0,
            },
        );
        rigid_body_builder_set_additional_mass(body_builder, 1.0);
        rigid_body_builder_set_linear_damping(body_builder, 0.0);
        rigid_body_builder_set_linvel(
            body_builder,
            Vec3 {
                x: 5.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let body = rigid_body_builder_build(body_builder);
        let body_handle = world_insert_rigid_body(world, body);
        let body_collider = collider_builder_build(collider_builder_create_ex(ShapeDesc {
            shape_type: 1,
            a: 0.25,
            b: 0.25,
            c: 0.25,
            d: 0.0,
        }));
        world_insert_collider_with_parent(world, body_collider, body_handle);

        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
        }

        let mut v = Vec3::default();
        rigid_body_get_linvel_out(world, body_handle, &mut v);
        world_destroy(world);
        (v.x, v.y)
    }

    #[test]
    fn hook_sets_manifold_friction_observed_in_kinematics() {
        let (vx_free, _vy_free) = run_slide(0.0);
        let (vx_stick, _vy_stick) = run_slide(1.0);

        assert!(
            vx_free > vx_stick,
            "frictionless vx ({vx_free}) must exceed sticky vx ({vx_stick})"
        );
        assert!(
            vx_free > 4.5,
            "mu=0 slab should retain most of its 5.0 m/s but got vx={vx_free}"
        );
        assert!(
            vx_stick < 2.0,
            "mu=1 slab should decelerate sharply but got vx={vx_stick}"
        );
    }
}

/// `world_step` and the Java-facing ring drain are the two concurrent users
/// permitted by `CollectingEventHandler`'s `Send`/`Sync` contract. Repeating
/// both paths catches accidental lock scope regressions and validates that
/// the step/init guards do not alias the producer cache.
#[test]
fn event_ring_concurrent_step_and_drain_stays_safe() {
    use mps_core::rapier::events::{
        world_collision_event_ring_len, world_contact_force_event_ring_len,
        world_drain_collision_event_ring, world_drain_contact_force_event_ring,
        world_init_collision_event_ring, world_init_contact_force_event_ring,
    };
    use mps_core::rapier::ffi::{
        Bool, CollisionEventRecord, ContactForceEventRecord, Vec3, WorldHandle,
    };
    use std::thread;
    let world = mps_core::rapier::world::world_create(Vec3::default());
    assert_eq!(world_init_collision_event_ring(world, 128), Bool::TRUE);
    assert_eq!(world_init_contact_force_event_ring(world, 128), Bool::TRUE);
    let world_addr = world as usize;
    let stepper = thread::spawn(move || {
        let world = world_addr as *mut WorldHandle;
        for _ in 0..2_000 {
            mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        }
    });
    let mut collisions = vec![CollisionEventRecord::default(); 128];
    let mut forces = vec![ContactForceEventRecord::default(); 128];
    for _ in 0..2_000 {
        let _ = world_drain_collision_event_ring(world, collisions.as_mut_ptr(), 128);
        let _ = world_drain_contact_force_event_ring(world, forces.as_mut_ptr(), 128);
    }
    stepper.join().unwrap();
    assert_eq!(world_collision_event_ring_len(world), 0);
    assert_eq!(world_contact_force_event_ring_len(world), 0);
    mps_core::rapier::world::world_destroy(world);
}
