//! 软体授权数据模型（`crates/mps-core/src/rapier/soft_data.rs`）的测试。
//!
//! 这些类型从 rapier 的 `soft_body_fork` 搬到了 mps-core：求解交给官方软体，这里只保留
//! 授权/查询所需的数据结构与纯数据操作，所以测试只钉数据语义（拓扑、静止长度、能量、
//! 体积、休眠标志），不钉求解结果。

#[cfg(test)]
mod tests {
    use mps_core::rapier::soft_data::{SoftBody, SoftBodySet, SoftSolver};
    use rapier3d::prelude::Vector;

    #[test]
    fn particles_carry_mass_and_pinning() {
        let mut body = SoftBody::new(Vector::new(0.0, -9.81, 0.0));
        let free = body.add_particle(Vector::new(0.0, 1.0, 0.0));
        let pinned = body.add_pinned(Vector::new(0.0, 0.0, 0.0));

        assert_eq!(body.particles.len(), 2);
        assert!(body.particles[free].inv_mass > 0.0, "free particle moves");
        assert_eq!(
            body.particles[pinned].inv_mass, 0.0,
            "pinned particle is immovable"
        );
    }

    #[test]
    fn springs_and_constraints_keep_their_rest_measure() {
        let mut body = SoftBody::new(Vector::ZERO);
        let a = body.add_particle(Vector::new(0.0, 0.0, 0.0));
        let b = body.add_particle(Vector::new(0.0, 2.0, 0.0));

        body.add_spring(a, b, 10.0, 0.5);
        body.add_distance_constraint(a, b, 0.01);

        assert_eq!(body.springs.len(), 1);
        assert_eq!(body.distance_constraints.len(), 1);
        assert!(
            (body.springs[0].rest_length - 2.0).abs() < 1.0e-9,
            "spring rest length is the initial distance"
        );
        assert!(
            (body.distance_constraints[0].rest - 2.0).abs() < 1.0e-9,
            "constraint rest length is the initial distance"
        );
    }

    #[test]
    fn kinetic_energy_is_a_pure_readout() {
        let mut body = SoftBody::new(Vector::ZERO);
        body.add_particle(Vector::new(0.0, 0.0, 0.0));
        let b = body.add_particle(Vector::new(1.0, 0.0, 0.0));
        body.particles[b].vel = Vector::new(2.0, 0.0, 0.0);

        let energy = body.kinetic_energy();
        assert!(
            energy.is_finite() && energy > 0.0,
            "moving particle has energy"
        );
        assert_eq!(
            body.kinetic_energy(),
            energy,
            "reading the energy must not change any state"
        );
    }

    #[test]
    fn set_keeps_ids_stable_across_removals() {
        let mut set = SoftBodySet::new();
        let first = set.insert(SoftBody::new(Vector::ZERO));
        let second = set.insert(SoftBody::new(Vector::ZERO));

        assert_eq!(set.count(), 2);
        assert!(set.remove(first));
        assert_eq!(set.count(), 1);
        assert!(set.get(first).is_none(), "removed slot is a tombstone");
        assert!(set.get(second).is_some(), "other ids stay valid");
        assert_eq!(set.len(), 2, "slots are not reshuffled");
    }

    #[test]
    fn solver_defaults_to_mass_spring() {
        let body = SoftBody::new(Vector::ZERO);
        assert!(matches!(body.solver, SoftSolver::MassSpring));
    }
}
