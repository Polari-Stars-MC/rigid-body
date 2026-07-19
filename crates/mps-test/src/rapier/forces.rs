#[cfg(test)]
mod tests {
    use smallvec::SmallVec;
    use rapier3d::prelude::{RigidBodySet, ColliderSet, NarrowPhase, RigidBodyHandle, RigidBodyBuilder, Vector};
    use mps_core::rapier::forces::*;

    /// A trivial test law that just records that it was called.
    struct TestLaw {
        enabled: bool,
        call_count: std::sync::atomic::AtomicU32,
    }

    impl TestLaw {
        fn new(enabled: bool) -> Self {
            Self {
                enabled,
                call_count: std::sync::atomic::AtomicU32::new(0),
            }
        }

        fn call_count(&self) -> u32 {
            self.call_count.load(std::sync::atomic::Ordering::Relaxed)
        }
    }

    impl ForceLaw for TestLaw {
        fn law_type(&self) -> ForceLawType {
            ForceLawType::Custom(999)
        }

        fn enabled(&self) -> bool {
            self.enabled
        }

        fn apply(&self, facade: &mut ForceFacade<'_>) {
            self.call_count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            facade.max_reynolds = facade.max_reynolds.max(42.0);
        }

        fn clone_box(&self) -> Box<dyn ForceLaw> {
            Box::new(Self {
                enabled: self.enabled,
                call_count: std::sync::atomic::AtomicU32::new(self.call_count()),
            })
        }
    }

    fn make_facade<'a>(
        bodies: &'a mut RigidBodySet,
        colliders: &'a mut ColliderSet,
        narrow_phase: &'a NarrowPhase,
        log: &'a mut Vec<Option<BodyForceLog>>,
        pending: &'a mut SmallVec<[mps_core::rapier::events::PendingForce; 128]>,
        friction: &'a mut Vec<(RigidBodyHandle, RigidBodyHandle, Vector)>,
    ) -> ForceFacade<'a> {
        ForceFacade::new(bodies, colliders, narrow_phase, log, pending, friction)
    }

    #[test]
    fn registry_register_and_apply() {
        let mut reg = ForceRegistry::new();
        assert!(reg.is_empty());

        let law = TestLaw::new(true);
        let handle = reg.register(Box::new(law));
        assert_eq!(reg.len(), 1);
        assert!(!reg.is_empty());

        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let narrow_phase = NarrowPhase::new();
        let mut log: Vec<Option<BodyForceLog>> = Vec::new();
        let mut pending = SmallVec::new();
        let mut friction = Vec::new();
        let mut facade = make_facade(&mut bodies, &mut colliders, &narrow_phase, &mut log, &mut pending, &mut friction);
        let report_before = facade.drain_report();
        assert_eq!(report_before.max_reynolds_number, 0.0);

        // Apply via registry → facade
        reg.apply_at(0, &mut facade);
        let report = facade.drain_report();
        assert_eq!(report.max_reynolds_number, 42.0);

        let law_ref = reg.get(handle).unwrap();
        assert_eq!(law_ref.law_type(), ForceLawType::Custom(999));
        assert!(law_ref.enabled());
    }

    #[test]
    fn registry_skips_disabled_laws() {
        let mut reg = ForceRegistry::new();
        reg.register(Box::new(TestLaw::new(false)));
        assert_eq!(reg.len(), 1);

        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let narrow_phase = NarrowPhase::new();
        let mut log: Vec<Option<BodyForceLog>> = Vec::new();
        let mut pending = SmallVec::new();
        let mut friction = Vec::new();
        let mut facade = make_facade(&mut bodies, &mut colliders, &narrow_phase, &mut log, &mut pending, &mut friction);

        reg.apply_at(0, &mut facade);
        let report = facade.drain_report();
        assert_eq!(report.max_reynolds_number, 0.0);
    }

    #[test]
    fn registry_unregister() {
        let mut reg = ForceRegistry::new();
        let h1 = reg.register(Box::new(TestLaw::new(true)));
        let h2 = reg.register(Box::new(TestLaw::new(true)));
        assert_eq!(reg.len(), 2);

        assert!(reg.unregister(h1));
        assert_eq!(reg.len(), 1);
        assert!(!reg.unregister(h1));
        assert!(reg.get(h1).is_none());
        assert!(reg.get(h2).is_some());
    }

    #[test]
    fn registry_find_by_type() {
        let mut reg = ForceRegistry::new();
        let h1 = reg.register(Box::new(TestLaw::new(true)));
        let h2 = reg.register(Box::new(TestLaw::new(true)));
        let h3 = reg.register(Box::new(TestLaw::new(true)));

        let found = reg.find_by_type(ForceLawType::Custom(999));
        assert_eq!(found.len(), 3);
        assert!(found.contains(&h1));
        assert!(found.contains(&h2));
        assert!(found.contains(&h3));

        let not_found = reg.find_by_type(ForceLawType::AirDrag);
        assert!(not_found.is_empty());
    }

    #[test]
    fn registry_reuses_freed_slots() {
        let mut reg = ForceRegistry::new();
        let h1 = reg.register(Box::new(TestLaw::new(true)));
        let h2 = reg.register(Box::new(TestLaw::new(true)));
        assert!(reg.unregister(h1));
        assert!(reg.unregister(h2));
        assert!(reg.is_empty());

        let h3 = reg.register(Box::new(TestLaw::new(true)));
        assert_eq!(reg.len(), 1);
        assert!(reg.get(h3).is_some());
    }

    #[test]
    fn facade_add_force_and_report() {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let narrow_phase = NarrowPhase::new();
        let mut log: Vec<Option<BodyForceLog>> = Vec::new();

        // Insert a dynamic body
        let builder = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.0, 0.0));
        let handle = bodies.insert(builder.build());

        let mut pending = SmallVec::new();
        let mut friction = Vec::new();
        let mut facade = make_facade(&mut bodies, &mut colliders, &narrow_phase, &mut log, &mut pending, &mut friction);

        let force = Vector::new(10.0, 0.0, 0.0);
        assert!(facade.add_force(handle, force, ForceLawType::AirDrag));
        assert!(facade.add_force(handle, Vector::new(0.0, -50.0, 0.0), ForceLawType::PointGravity));

        let report = facade.drain_report();
        let drag = report.contributions.get(&ForceLawType::AirDrag).unwrap();
        assert_eq!(drag.body_count, 1);
        assert!((drag.total_force.x - 10.0).abs() < 1e-12);

        let grav = report.contributions.get(&ForceLawType::PointGravity).unwrap();
        assert_eq!(grav.body_count, 1);

        // Log should be empty after drain
        assert!(log.is_empty());
    }

    #[test]
    fn facade_skips_fixed_body() {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let narrow_phase = NarrowPhase::new();
        let mut log: Vec<Option<BodyForceLog>> = Vec::new();

        let builder = RigidBodyBuilder::fixed();
        let handle = bodies.insert(builder.build());

        let mut pending = SmallVec::new();
        let mut friction = Vec::new();
        let mut facade = make_facade(&mut bodies, &mut colliders, &narrow_phase, &mut log, &mut pending, &mut friction);
        assert!(facade.add_force(handle, Vector::new(10.0, 0.0, 0.0), ForceLawType::AirDrag));

        let report = facade.drain_report();
        assert!(report.contributions.is_empty());
    }

    #[test]
    fn force_report_to_legacy_conversion() {
        let mut report = ForceReport::default();
        report.add(
            ForceLawType::AirDrag,
            mps_core::rapier::ffi::vec3_to_rapier(mps_core::rapier::ffi::Vec3 {
                x: -10.0,
                y: 0.0,
                z: 0.0,
            }),
            3,
        );
        report.add(
            ForceLawType::PointGravity,
            mps_core::rapier::ffi::vec3_to_rapier(mps_core::rapier::ffi::Vec3 {
                x: 0.0,
                y: -50.0,
                z: 0.0,
            }),
            1,
        );

        let legacy = report.to_legacy_report();
        assert_eq!(legacy.drag_body_count, 3);
        assert_eq!(legacy.total_drag_force.x, -10.0);
        assert!(legacy.total_external_force.x == -10.0);
    }
}







