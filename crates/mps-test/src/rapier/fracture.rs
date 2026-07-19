#[cfg(test)]
mod tests {
    use mps_core::rapier::fracture::*;
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::Vec3;
    use rapier3d::prelude::{RigidBodyBuilder, ColliderBuilder};
    use mps_core::rapier::world::world_create;

    fn v3(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    #[test]
    fn fracture_formulas_work() {
        let mut intensity = StressIntensityReport::default();
        assert_eq!(
            fracture_stress_intensity_factor(100.0, 0.01, 1.0, 10.0, &mut intensity),
            Bool::TRUE
        );
        assert!(intensity.stress_intensity > 0.0);
        assert_eq!(intensity.critical, Bool::TRUE);

        let material = FractureMaterial {
            youngs_modulus: 200.0e9,
            poisson_ratio: 0.3,
            fracture_toughness: 50.0e6,
            surface_energy: 10.0,
            density: 7850.0,
        };
        let mut griffith = GriffithReport::default();
        assert_eq!(
            fracture_griffith_criterion(1.0e6, 0.01, material, &mut griffith),
            Bool::TRUE
        );
        assert!(griffith.critical_stress > 0.0);
        assert_eq!(griffith.critical_energy_release_rate, 20.0);

        let cycles = [100.0, 50.0];
        let lives = [1000.0, 500.0];
        let mut damage = MinerDamageReport::default();
        assert_eq!(
            fracture_miner_damage(
                cycles.as_ptr(),
                lives.as_ptr(),
                cycles.len() as u32,
                &mut damage
            ),
            Bool::TRUE
        );
        assert!((damage.damage - 0.2).abs() < 1.0e-12);
        assert_eq!(damage.failed, Bool::FALSE);

        let mut sn = SnCurveReport::default();
        assert_eq!(
            fracture_sn_curve_life(50.0, 1.0e12, 3.0, 100.0, &mut sn),
            Bool::TRUE
        );
        assert_eq!(sn.infinite_life, Bool::TRUE);

        let mut energy = FractureEnergyReport::default();
        assert_eq!(
            fracture_energy_release(120.0, 10.0, 8.0, 0.0, &mut energy),
            Bool::TRUE
        );
        assert_eq!(energy.will_fracture, Bool::TRUE);
        assert_eq!(energy.fragment_kinetic_energy, 40.0);

        let mut mode = FractureModeReport::default();
        assert_eq!(
            fracture_mode_from_stress(1.0, 3.0, 2.0, &mut mode),
            Bool::TRUE
        );
        assert_eq!(mode.mode, 2);
    }

    #[test]
    fn fracture_replaces_body_with_connected_fragments() {
        let world = world_create(v3(0.0, -9.81, 0.0));
        assert!(!world.is_null());
        let world = unsafe { &mut *world };

        let source = world
            .inner
            .bodies
            .insert(RigidBodyBuilder::dynamic().build());
        world.inner.colliders.insert_with_parent(
            ColliderBuilder::cuboid(1.0, 1.0, 1.0).density(1.0).build(),
            source,
            &mut world.inner.bodies,
        );

        let fragments = [
            FractureFragmentDesc {
                local_center: v3(-0.5, 0.0, 0.0),
                half_extents: v3(0.25, 0.5, 0.5),
                initial_velocity: v3(-1.0, 0.0, 0.0),
                density: 1.0,
                friction: 0.5,
                restitution: 0.1,
            },
            FractureFragmentDesc {
                local_center: v3(0.5, 0.0, 0.0),
                half_extents: v3(0.25, 0.5, 0.5),
                initial_velocity: v3(1.0, 0.0, 0.0),
                density: 1.0,
                friction: 0.5,
                restitution: 0.1,
            },
        ];
        let mut bodies = [0; 2];
        let mut joints = [0; 2];
        let mut report = FractureReplaceReport::default();
        assert_eq!(
            world_replace_body_with_fracture_fragments(
                world,
                pack_rigid_body_handle(source),
                fragments.as_ptr(),
                fragments.len() as u32,
                Bool::TRUE,
                Bool::TRUE,
                bodies.as_mut_ptr(),
                joints.as_mut_ptr(),
                bodies.len() as u32,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(report.fragment_count, 2);
        assert_eq!(report.joint_count, 1);
        assert_eq!(report.removed_source, Bool::TRUE);
        assert!(bodies.iter().all(|handle| *handle != 0));
        assert_ne!(joints[0], 0);
        assert_eq!(world.inner.bodies.len(), 2);
    }
}



