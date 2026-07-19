#[cfg(test)]
mod tests {
    use mps_core::rapier::neural::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::collider::{collider_builder_destroy, world_insert_collider};
    use mps_core::rapier::ffi::{Bool, Quat, Vec3};

    fn identity_rotation() -> Quat {
        Quat {
            i: 0.0,
            j: 0.0,
            k: 0.0,
            w: 1.0,
        }
    }

    fn desc() -> NeuralBoundsDesc {
        NeuralBoundsDesc {
            center: Vec3::default(),
            half_extents: Vec3 {
                x: 1.0,
                y: 0.5,
                z: 1.5,
            },
            rotation: identity_rotation(),
            sample_resolution: 8,
            hidden_width: 4,
            hidden_layers: 1,
            activation: NeuralActivation::Relu as u32,
            output_scale: 0.1,
            padding: 0.02,
        }
    }

    #[test]
    fn required_weight_count_matches_layout() {
        assert_eq!(neural_bounds_required_weight_count(0, 0), 4);
        assert_eq!(neural_bounds_required_weight_count(4, 1), 21);
        assert_eq!(neural_bounds_required_weight_count(4, 2), 41);
    }

    #[test]
    fn neural_bounds_builds_collider() {
        let desc = desc();
        let weights = vec![0.0; neural_bounds_required_weight_count(4, 1) as usize];
        let builder =
            collider_builder_create_neural_bounds(desc, weights.as_ptr(), weights.len() as u32);
        assert!(!builder.is_null());
        collider_builder_destroy(builder);
    }

    #[test]
    fn neural_bounds_query_intersects_inserted_collider() {
        let desc = desc();
        let weights = vec![0.0; neural_bounds_required_weight_count(4, 1) as usize];
        let builder = mps_core::rapier::collider::collider_builder_build(
            collider_builder_create_neural_bounds(desc, weights.as_ptr(), weights.len() as u32),
        );
        assert!(!builder.is_null());

        let world = mps_core::rapier::world::world_create(Vec3::default());
        let collider = world_insert_collider(world, builder);
        assert_ne!(collider, 0);
        mps_core::rapier::world::world_step(world, 1.0 / 60.0);

        let filter = QueryFilterDesc {
            use_groups: Bool::FALSE,
            ..QueryFilterDesc::default()
        };
        assert_eq!(
            query_intersect_neural_bounds_count(
                world,
                desc,
                weights.as_ptr(),
                weights.len() as u32,
                filter
            ),
            1
        );

        mps_core::rapier::world::world_destroy(world);
    }
}



