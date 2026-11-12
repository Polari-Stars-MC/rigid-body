//! End-to-end tests for fracture mesh bodies (fracturable composite rigid
//! bodies built on top of `fracture.rs`).

#[cfg(test)]
mod tests {
    use mps_core::rapier::collider::{
        collider_builder_build, collider_builder_create_ex, world_insert_collider_with_parent,
    };
    use mps_core::rapier::error::{
        ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, ERR_OK,
        ERR_UNSUPPORTED, last_error_code,
    };
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::fracture_mesh::{
        fracture_mesh_body_add_fatigue_damage, fracture_mesh_body_create,
        fracture_mesh_body_create_with_voronoi, fracture_mesh_body_is_fractured,
        fracture_mesh_body_remove, fracture_mesh_body_set_stress, fracture_mesh_body_set_trigger,
        fracture_mesh_body_set_trigger_stress, fracture_mesh_body_trigger,
    };
    use mps_core::rapier::fracture_mesh::{
        fracture_mesh_body_enable_impact_damage, fracture_mesh_body_get_impact_damage,
        fracture_mesh_body_link_granular_debris,
    };
    use mps_core::rapier::granular::{granular_create, granular_particle_count};
    use mps_core::rapier::rigid_body::{
        rigid_body_builder_build, rigid_body_builder_create, rigid_body_builder_set_translation,
        world_insert_rigid_body,
    };
    use mps_core::rapier::world::{world_create, world_destroy, world_step};
    fn v3(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    fn make_world() -> *mut WorldHandle {
        world_create(v3(0.0, -9.81, 0.0))
    }

    fn valid_material() -> FractureMaterial {
        FractureMaterial {
            youngs_modulus: 200.0e9,
            poisson_ratio: 0.3,
            fracture_toughness: 50.0e6,
            surface_energy: 10.0,
            density: 7850.0,
        }
    }

    fn valid_fragments() -> [FractureFragmentDesc; 2] {
        [
            FractureFragmentDesc {
                local_center: v3(-0.5, 0.0, 0.0),
                half_extents: v3(0.25, 0.5, 0.5),
                initial_velocity: v3(0.0, 0.0, 0.0),
                density: 1.0,
                friction: 0.5,
                restitution: 0.1,
            },
            FractureFragmentDesc {
                local_center: v3(0.5, 0.0, 0.0),
                half_extents: v3(0.25, 0.5, 0.5),
                initial_velocity: v3(0.0, 0.0, 0.0),
                density: 1.0,
                friction: 0.5,
                restitution: 0.1,
            },
        ]
    }

    fn cuboid_shape() -> ShapeDesc {
        ShapeDesc {
            shape_type: ShapeType::Cuboid as u32,
            a: 1.0,
            b: 1.0,
            c: 1.0,
            ..Default::default()
        }
    }

    fn make_mesh(world: *mut WorldHandle) -> u32 {
        let frags = valid_fragments();
        fracture_mesh_body_create(
            world,
            cuboid_shape(),
            v3(0.0, 10.0, 0.0),
            frags.as_ptr(),
            frags.len() as u32,
            valid_material(),
            Bool::TRUE,
        )
    }

    #[test]
    fn fracture_mesh_create_reports_intact_state() {
        let world = make_world();
        let id = make_mesh(world);
        assert_ne!(id, u32::MAX);
        assert_eq!(last_error_code(), ERR_OK);
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_OK);
        world_destroy(world);
    }

    #[test]
    fn fracture_mesh_create_rejects_bad_input() {
        let world = make_world();
        let frags = valid_fragments();

        // No fragments / too many fragments.
        assert_eq!(
            fracture_mesh_body_create(
                world,
                cuboid_shape(),
                v3(0.0, 10.0, 0.0),
                frags.as_ptr(),
                0,
                valid_material(),
                Bool::TRUE
            ),
            u32::MAX
        );
        assert_eq!(last_error_code(), ERR_CAPACITY);

        // Null fragment pointer.
        assert_eq!(
            fracture_mesh_body_create(
                world,
                cuboid_shape(),
                v3(0.0, 10.0, 0.0),
                std::ptr::null(),
                1,
                valid_material(),
                Bool::TRUE
            ),
            u32::MAX
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Invalid material (poisson ratio out of range).
        let mut bad_material = valid_material();
        bad_material.poisson_ratio = 0.9;
        assert_eq!(
            fracture_mesh_body_create(
                world,
                cuboid_shape(),
                v3(0.0, 10.0, 0.0),
                frags.as_ptr(),
                frags.len() as u32,
                bad_material,
                Bool::TRUE
            ),
            u32::MAX
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Invalid fragment descriptor (zero half extent).
        let mut bad_frags = valid_fragments();
        bad_frags[1].half_extents = v3(0.25, 0.0, 0.5);
        assert_eq!(
            fracture_mesh_body_create(
                world,
                cuboid_shape(),
                v3(0.0, 10.0, 0.0),
                bad_frags.as_ptr(),
                bad_frags.len() as u32,
                valid_material(),
                Bool::TRUE
            ),
            u32::MAX
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        world_destroy(world);
    }

    #[test]
    fn fracture_mesh_trigger_fractures_once() {
        let world = make_world();
        let id = make_mesh(world);
        assert_ne!(id, u32::MAX);

        assert_eq!(fracture_mesh_body_trigger(world, id), Bool::TRUE);
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::TRUE);

        // The fractured body keeps its id; a second trigger is rejected.
        assert_eq!(fracture_mesh_body_trigger(world, id), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_UNSUPPORTED);

        // Fragments are dynamic bodies — the world must step cleanly.
        for _ in 0..10 {
            world_step(world, 1.0 / 60.0);
        }
        world_destroy(world);
    }

    #[test]
    fn fracture_mesh_stress_trigger_auto_fractures() {
        let world = make_world();
        let id = make_mesh(world);
        assert_ne!(id, u32::MAX);

        assert_eq!(
            fracture_mesh_body_set_trigger(world, id, 1, 10.0),
            Bool::TRUE
        );
        // Below threshold: no fracture.
        assert_eq!(fracture_mesh_body_set_stress(world, id, 5.0), Bool::TRUE);
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::FALSE);
        // Above threshold: auto-fracture.
        assert_eq!(fracture_mesh_body_set_stress(world, id, 12.0), Bool::TRUE);
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::TRUE);
        world_destroy(world);
    }

    #[test]
    fn fracture_mesh_set_trigger_stress_wraps_mode_1() {
        let world = make_world();
        let id = make_mesh(world);
        assert_ne!(id, u32::MAX);

        assert_eq!(
            fracture_mesh_body_set_trigger_stress(world, id, 3.0),
            Bool::TRUE
        );
        assert_eq!(fracture_mesh_body_set_stress(world, id, 3.0), Bool::TRUE);
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::TRUE);

        // Invalid threshold is rejected.
        let id2 = make_mesh(world);
        assert_ne!(id2, u32::MAX);
        assert_eq!(
            fracture_mesh_body_set_trigger_stress(world, id2, 0.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }

    #[test]
    fn fracture_mesh_fatigue_trigger_auto_fractures() {
        let world = make_world();
        let id = make_mesh(world);
        assert_ne!(id, u32::MAX);

        assert_eq!(
            fracture_mesh_body_set_trigger(world, id, 3, 0.0),
            Bool::TRUE
        );
        assert_eq!(
            fracture_mesh_body_add_fatigue_damage(world, id, 0.4),
            Bool::TRUE
        );
        assert_eq!(
            fracture_mesh_body_add_fatigue_damage(world, id, 0.4),
            Bool::TRUE
        );
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::FALSE);
        // Reaches 1.0 → auto-fracture.
        assert_eq!(
            fracture_mesh_body_add_fatigue_damage(world, id, 0.2),
            Bool::TRUE
        );
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::TRUE);

        // Negative damage is rejected.
        let id2 = make_mesh(world);
        assert_ne!(id2, u32::MAX);
        assert_eq!(
            fracture_mesh_body_add_fatigue_damage(world, id2, -1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }

    #[test]
    fn fracture_mesh_set_trigger_rejects_bad_mode() {
        let world = make_world();
        let id = make_mesh(world);
        assert_ne!(id, u32::MAX);
        assert_eq!(
            fracture_mesh_body_set_trigger(world, id, 7, 1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }

    #[test]
    fn fracture_mesh_remove_drops_the_body() {
        let world = make_world();
        let id = make_mesh(world);
        assert_ne!(id, u32::MAX);

        assert_eq!(fracture_mesh_body_remove(world, id), Bool::TRUE);
        assert_eq!(fracture_mesh_body_remove(world, id), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        // Unknown ids report not-found.
        assert_eq!(fracture_mesh_body_is_fractured(world, 987654), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        world_destroy(world);
    }

    #[test]
    fn fracture_mesh_null_world_is_rejected() {
        assert_eq!(
            fracture_mesh_body_create(
                std::ptr::null_mut(),
                cuboid_shape(),
                v3(0.0, 0.0, 0.0),
                std::ptr::null(),
                1,
                valid_material(),
                Bool::TRUE
            ),
            u32::MAX
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            fracture_mesh_body_trigger(std::ptr::null_mut(), 0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
    }

    #[test]
    fn create_with_voronoi_and_trigger_fractures_into_seeds() {
        let world = make_world();
        let seeds = [v3(-0.5, 0.0, 0.0), v3(0.5, 0.0, 0.0)];
        let id = fracture_mesh_body_create_with_voronoi(
            world,
            cuboid_shape(),
            v3(0.0, 10.0, 0.0),
            v3(-1.0, -1.0, -1.0),
            v3(1.0, 1.0, 1.0),
            seeds.as_ptr(),
            seeds.len() as u32,
            valid_material(),
            Bool::FALSE,
            0.0,
        );
        assert_ne!(id, u32::MAX);
        assert_eq!(last_error_code(), ERR_OK);
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::FALSE);

        // Triggering replaces the source with one fragment per seed cell.
        assert_eq!(fracture_mesh_body_trigger(world, id), Bool::TRUE);
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::TRUE);
        // One-shot semantics still hold for generated fragment sets.
        assert_eq!(fracture_mesh_body_trigger(world, id), Bool::FALSE);
        assert_eq!(last_error_code(), ERR_UNSUPPORTED);

        assert_eq!(fracture_mesh_body_remove(world, id), Bool::TRUE);
        world_destroy(world);
    }

    #[test]
    fn create_with_voronoi_rejects_bad_input() {
        let world = make_world();
        let seeds = [v3(-0.5, 0.0, 0.0), v3(0.5, 0.0, 0.0)];

        // Null seed pointer.
        assert_eq!(
            fracture_mesh_body_create_with_voronoi(
                world,
                cuboid_shape(),
                v3(0.0, 10.0, 0.0),
                v3(-1.0, -1.0, -1.0),
                v3(1.0, 1.0, 1.0),
                std::ptr::null(),
                2,
                valid_material(),
                Bool::FALSE,
                0.0
            ),
            u32::MAX
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Zero seed count.
        assert_eq!(
            fracture_mesh_body_create_with_voronoi(
                world,
                cuboid_shape(),
                v3(0.0, 10.0, 0.0),
                v3(-1.0, -1.0, -1.0),
                v3(1.0, 1.0, 1.0),
                seeds.as_ptr(),
                0,
                valid_material(),
                Bool::FALSE,
                0.0
            ),
            u32::MAX
        );
        assert_eq!(last_error_code(), ERR_CAPACITY);

        // Inverted AABB.
        assert_eq!(
            fracture_mesh_body_create_with_voronoi(
                world,
                cuboid_shape(),
                v3(0.0, 10.0, 0.0),
                v3(1.0, 1.0, 1.0),
                v3(-1.0, -1.0, -1.0),
                seeds.as_ptr(),
                seeds.len() as u32,
                valid_material(),
                Bool::FALSE,
                0.0
            ),
            u32::MAX
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Out-of-range shrink.
        assert_eq!(
            fracture_mesh_body_create_with_voronoi(
                world,
                cuboid_shape(),
                v3(0.0, 10.0, 0.0),
                v3(-1.0, -1.0, -1.0),
                v3(1.0, 1.0, 1.0),
                seeds.as_ptr(),
                seeds.len() as u32,
                valid_material(),
                Bool::FALSE,
                0.6
            ),
            u32::MAX
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Invalid material.
        let mut bad_material = valid_material();
        bad_material.poisson_ratio = 0.9;
        assert_eq!(
            fracture_mesh_body_create_with_voronoi(
                world,
                cuboid_shape(),
                v3(0.0, 10.0, 0.0),
                v3(-1.0, -1.0, -1.0),
                v3(1.0, 1.0, 1.0),
                seeds.as_ptr(),
                seeds.len() as u32,
                bad_material,
                Bool::FALSE,
                0.0
            ),
            u32::MAX
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Null world.
        assert_eq!(
            fracture_mesh_body_create_with_voronoi(
                std::ptr::null_mut(),
                cuboid_shape(),
                v3(0.0, 0.0, 0.0),
                v3(-1.0, -1.0, -1.0),
                v3(1.0, 1.0, 1.0),
                seeds.as_ptr(),
                seeds.len() as u32,
                valid_material(),
                Bool::FALSE,
                0.0
            ),
            u32::MAX
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        world_destroy(world);
    }

    /// Fixed floor cuboid whose top surface sits at y = 0.
    fn make_floor(world: *mut WorldHandle) -> RigidBodyHandleRaw {
        let builder = rigid_body_builder_create(BodyStatus::Fixed as u32);
        rigid_body_builder_set_translation(builder, v3(0.0, -0.5, 0.0));
        let body = world_insert_rigid_body(world, rigid_body_builder_build(builder));
        let shape = ShapeDesc {
            shape_type: ShapeType::Cuboid as u32,
            a: 20.0,
            b: 0.5,
            c: 20.0,
            ..Default::default()
        };
        world_insert_collider_with_parent(
            world,
            collider_builder_build(collider_builder_create_ex(shape)),
            body,
        );
        body
    }

    #[test]
    fn impact_damage_accumulates_and_auto_fractures() {
        let world = make_world();
        make_floor(world);

        // 2-seed voronoi box dropped onto the floor.
        let seeds = [v3(-0.5, 0.0, 0.0), v3(0.5, 0.0, 0.0)];
        let id = fracture_mesh_body_create_with_voronoi(
            world,
            cuboid_shape(),
            v3(0.0, 1.5, 0.0),
            v3(-1.0, -1.0, -1.0),
            v3(1.0, 1.0, 1.0),
            seeds.as_ptr(),
            seeds.len() as u32,
            valid_material(),
            Bool::FALSE,
            0.0,
        );
        assert_ne!(id, u32::MAX);

        // Scale 1.0 + threshold 1e-9: any landing impulse must fracture.
        assert_eq!(
            fracture_mesh_body_enable_impact_damage(world, id, 1.0, 1.0e-9),
            Bool::TRUE
        );
        let mut damage = 1.0;
        assert_eq!(
            fracture_mesh_body_get_impact_damage(world, id, &mut damage),
            Bool::TRUE
        );
        assert_eq!(damage, 0.0);

        let mut fractured = false;
        for _ in 0..120 {
            world_step(world, 1.0 / 60.0);
            if fracture_mesh_body_is_fractured(world, id) == Bool::TRUE {
                fractured = true;
                break;
            }
        }
        assert!(fractured, "mesh body did not auto-fracture on landing");
        world_destroy(world);
    }

    #[test]
    fn impact_damage_disabled_by_default_and_rejects_bad_input() {
        let world = make_world();
        make_floor(world);

        let seeds = [v3(-0.5, 0.0, 0.0), v3(0.5, 0.0, 0.0)];
        let id = fracture_mesh_body_create_with_voronoi(
            world,
            cuboid_shape(),
            v3(0.0, 1.5, 0.0),
            v3(-1.0, -1.0, -1.0),
            v3(1.0, 1.0, 1.0),
            seeds.as_ptr(),
            seeds.len() as u32,
            valid_material(),
            Bool::FALSE,
            0.0,
        );
        assert_ne!(id, u32::MAX);

        // Without enabling, landing must not fracture; damage stays 0.
        for _ in 0..90 {
            world_step(world, 1.0 / 60.0);
        }
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::FALSE);
        let mut damage = 1.0;
        assert_eq!(
            fracture_mesh_body_get_impact_damage(world, id, &mut damage),
            Bool::TRUE
        );
        assert_eq!(damage, 0.0);

        // Validation: bad scale / threshold.
        assert_eq!(
            fracture_mesh_body_enable_impact_damage(world, id, 0.0, 1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            fracture_mesh_body_enable_impact_damage(world, id, 1.0, 0.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        // Unknown id.
        assert_eq!(
            fracture_mesh_body_enable_impact_damage(world, 987654, 1.0, 1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(
            fracture_mesh_body_get_impact_damage(world, 987654, &mut damage),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        // Null out pointer.
        assert_eq!(
            fracture_mesh_body_get_impact_damage(world, id, std::ptr::null_mut()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        // Null world.
        assert_eq!(
            fracture_mesh_body_enable_impact_damage(std::ptr::null_mut(), id, 1.0, 1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        world_destroy(world);
    }

    #[test]
    fn granular_debris_link_routes_small_fragments_to_grains() {
        let world = make_world();
        let granular_id = granular_create(world, v3(0.0, -9.81, 0.0), 0.05, 800.0, 0.5, 0.6, 0.4);
        assert_ne!(granular_id, u32::MAX);

        // One large fragment (half 0.5) and one small one (half 0.1).
        let fragments = [
            FractureFragmentDesc {
                local_center: v3(-0.5, 0.0, 0.0),
                half_extents: v3(0.5, 0.5, 0.5),
                initial_velocity: v3(0.0, 0.0, 0.0),
                density: 1.0,
                friction: 0.5,
                restitution: 0.1,
            },
            FractureFragmentDesc {
                local_center: v3(0.5, 0.0, 0.0),
                half_extents: v3(0.1, 0.1, 0.1),
                initial_velocity: v3(0.0, 0.0, 0.0),
                density: 1.0,
                friction: 0.5,
                restitution: 0.1,
            },
        ];
        let id = fracture_mesh_body_create(
            world,
            cuboid_shape(),
            v3(0.0, 5.0, 0.0),
            fragments.as_ptr(),
            fragments.len() as u32,
            valid_material(),
            Bool::FALSE,
        );
        assert_ne!(id, u32::MAX);

        // Threshold 0.2: the small fragment becomes a grain, the large one
        // stays a rigid fragment.
        assert_eq!(
            fracture_mesh_body_link_granular_debris(world, id, granular_id, 0.2, 0.4, 0.05),
            Bool::TRUE
        );
        assert_eq!(fracture_mesh_body_trigger(world, id), Bool::TRUE);
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::TRUE);
        assert_eq!(granular_particle_count(world, granular_id), 1);
        assert_eq!(fracture_mesh_body_remove(world, id), Bool::TRUE);
        world_destroy(world);
    }

    #[test]
    fn granular_debris_all_fragments_become_grains() {
        let world = make_world();
        let granular_id = granular_create(world, v3(0.0, -9.81, 0.0), 0.05, 800.0, 0.5, 0.6, 0.4);
        assert_ne!(granular_id, u32::MAX);

        let id = make_mesh(world);
        // Threshold above every half-extent: the whole body turns to grains,
        // exercising the "no rigid fragments" direct-removal path.
        assert_eq!(
            fracture_mesh_body_link_granular_debris(world, id, granular_id, 10.0, 0.2, 0.04),
            Bool::TRUE
        );
        assert_eq!(fracture_mesh_body_trigger(world, id), Bool::TRUE);
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::TRUE);
        assert_eq!(granular_particle_count(world, granular_id), 2);
        // Double trigger stays rejected after the all-debris path too.
        assert_eq!(fracture_mesh_body_trigger(world, id), Bool::FALSE);
        world_destroy(world);
    }

    #[test]
    fn granular_debris_link_rejects_bad_input() {
        let world = make_world();
        let id = make_mesh(world);

        // Unknown granular body.
        assert_eq!(
            fracture_mesh_body_link_granular_debris(world, id, 987654, 0.2, 0.4, 0.05),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        // Bad threshold / mass / radius.
        assert_eq!(
            fracture_mesh_body_link_granular_debris(world, id, 0, 0.0, 0.4, 0.05),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            fracture_mesh_body_link_granular_debris(world, id, 0, 0.2, 0.0, 0.05),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            fracture_mesh_body_link_granular_debris(world, id, 0, 0.2, 0.4, -0.05),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        // Unknown mesh id / null world.
        assert_eq!(
            fracture_mesh_body_link_granular_debris(world, 987654, 0, 0.2, 0.4, 0.05),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert_eq!(
            fracture_mesh_body_link_granular_debris(std::ptr::null_mut(), id, 0, 0.2, 0.4, 0.05),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Unlink (u32::MAX) succeeds even with placeholder params.
        assert_eq!(
            fracture_mesh_body_link_granular_debris(world, id, u32::MAX, 0.0, 0.0, 0.0),
            Bool::TRUE
        );

        // After fracture the link is refused.
        assert_eq!(fracture_mesh_body_trigger(world, id), Bool::TRUE);
        assert_eq!(
            fracture_mesh_body_link_granular_debris(world, id, 0, 0.2, 0.4, 0.05),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_UNSUPPORTED);

        world_destroy(world);
    }

    /// B2：接触力事件 → 自动碎裂闭环。重球砸碎裂板 →
    /// `world_auto_fracture_from_contact_forces` 消费事件峰值力 →
    /// 过阈自动碎裂（走手动触发同一条替换路径）。
    #[test]
    fn contact_force_event_auto_fracture_loop() {
        use mps_core::rapier::rigid_body::rigid_body_builder_set_linvel;
        use mps_core::rapier::world::{world_apply_runtime_settings, world_step};

        let world = make_world();
        assert_eq!(
            world_apply_runtime_settings(world, 4, 1, 0, 1, 0, 1),
            Bool::TRUE
        );
        // 碎裂板（在 y=0 处静置），应力触发阈 1000（峰值力代理）。
        let id = make_mesh(world);
        assert_ne!(id, u32::MAX);
        assert_eq!(
            fracture_mesh_body_set_trigger(world, id, 1, 1000.0),
            Bool::TRUE
        );
        assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::FALSE);

        // 重球：碎裂板（make_mesh 在 y=10）上方 2.5 m，向下 20 m/s 冲击。
        let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_translation(builder, v3(0.0, 12.5, 0.0));
        rigid_body_builder_set_linvel(builder, v3(0.0, -20.0, 0.0));
        let ball = world_insert_rigid_body(world, rigid_body_builder_build(builder));
        assert_ne!(ball, 0);
        let shape = ShapeDesc {
            shape_type: ShapeType::Ball as u32,
            a: 0.5,
            ..Default::default()
        };
        let collider = collider_builder_build(collider_builder_create_ex(shape));
        world_insert_collider_with_parent(world, collider, ball);

        // 步进直到有接触力事件产生（最多 120 步）。
        let mut steps = 0usize;
        loop {
            world_step(world, 1.0 / 60.0);
            steps += 1;
            let hit = fracture_mesh_body_is_fractured(world, id) == Bool::TRUE
                || mps_core::rapier::events::world_contact_force_event_count(world) > 0;
            if hit || steps >= 120 {
                break;
            }
        }
        assert!(
            steps < 120,
            "impact must produce a contact within 120 steps"
        );

        // 闭环调用：消费事件峰值力 → 应力 = 峰值 × scale(1.0)。
        let mut ids = [0u32; 4];
        let fractured = mps_core::rapier::fracture_mesh::world_auto_fracture_from_contact_forces(
            world,
            1.0,
            ids.as_mut_ptr(),
            ids.len() as u32,
        );
        if fractured > 0 {
            assert_eq!(fractured, 1);
            assert_eq!(ids[0], id);
            assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::TRUE);
        } else {
            // 冲击力未过阈：应力必须已写入但碎裂未发生。
            assert_eq!(fracture_mesh_body_is_fractured(world, id), Bool::FALSE);
        }

        // 消费后事件队列清空：第二次调用返回 0。
        let again = mps_core::rapier::fracture_mesh::world_auto_fracture_from_contact_forces(
            world,
            1.0,
            std::ptr::null_mut(),
            0,
        );
        assert_eq!(again, 0, "queue must be drained by the first call");

        world_destroy(world);
    }

    /// B2 参数校验：负 scale、null world。
    #[test]
    fn auto_fracture_rejects_bad_input() {
        assert_eq!(
            mps_core::rapier::fracture_mesh::world_auto_fracture_from_contact_forces(
                std::ptr::null_mut(),
                1.0,
                std::ptr::null_mut(),
                0,
            ),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        let world = make_world();
        assert_eq!(
            mps_core::rapier::fracture_mesh::world_auto_fracture_from_contact_forces(
                world,
                -1.0,
                std::ptr::null_mut(),
                0,
            ),
            0
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }
}
