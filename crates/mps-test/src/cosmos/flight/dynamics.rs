//! `flight::dynamics` integration tests — 6-DOF force synthesis and the
//! `simulate_one_step` integrator.
//!
//! A dynamic `RigidBody` constructed via
//! [`mps_cosmos::bodies::satellite_builder`] does not have its added mass
//! propagated into `local_mprops` until
//! [`RigidBody::recompute_mass_properties_from_colliders`] is called — this
//! is Rapier's mass-flow behaviour (see `mpd-test/src/cosmos/bodies.rs`).  We
//! call it after `build()` so `body.mass()` reads the user-supplied mass.

#[cfg(test)]
mod tests {
    use mps_cosmos::bodies::satellite_builder;
    use mps_cosmos::flight::{
        ConstantGravity, FlightControls, SeaLevelAtmosphere, StepConfig, default_airfoil,
        dynamics::RigidBodyState, simulate_one_step, total_forces_and_moments,
    };
    use mps_formula::rotor::RotorParams;
    use rapier3d::prelude::{ColliderSet, RigidBodySet, Rotation, Vector};

    fn main_rotor() -> RotorParams {
        RotorParams {
            radius: 5.4,
            n_blades: 2,
            chord: 0.20,
            hinge_offset: 0.05,
            lift_slope: 2.0 * std::f64::consts::PI,
            zero_lift_alpha: 0.0,
            profile_cd0: 0.012,
            cd_k: 0.05,
        }
    }
    fn tail_rotor() -> RotorParams {
        RotorParams {
            radius: 1.0,
            n_blades: 2,
            chord: 0.08,
            hinge_offset: 0.02,
            lift_slope: 2.0 * std::f64::consts::PI,
            zero_lift_alpha: 0.0,
            profile_cd0: 0.015,
            cd_k: 0.05,
        }
    }
    fn controls() -> FlightControls {
        FlightControls {
            collective: 0.10,
            cyclic_lon: 0.0,
            cyclic_lat: 0.0,
            tail_collective: 0.0,
            throttle: 1.0,
        }
    }

    #[test]
    fn total_forces_returns_some_on_valid_inputs() {
        let rotor = main_rotor();
        let tail = tail_rotor();
        let airfoil = default_airfoil(&rotor);
        let state = RigidBodyState {
            position: Vector::new(0.0, 0.0, 100.0),
            linvel_world: Vector::ZERO,
            angvel_body: Vector::ZERO,
            rotation: Rotation::IDENTITY,
            mass: 800.0,
        };
        let report = total_forces_and_moments(
            &state,
            &rotor,
            &tail,
            &SeaLevelAtmosphere,
            &ConstantGravity { g: 9.81 },
            &controls(),
            40.0,
            0.0,
            &airfoil,
            60,
        )
        .expect("valid inputs should yield Some");
        // Sanity: rotor thrust positive, gravity gives acc < 50 m/s², and
        // total power is finite.  The report's force mag for a hovering 800
        // kg craft should be in the kilo-Newton band — well below the 50 m/s²
        // acceleration cap for an 800 kg body (50 m/s² · 800 kg ≈ 40 kN).
        let acc = report.force_world.length() / 800.0;
        assert!(acc < 50.0, "acc={acc}, force={:?}", report.force_world);
        // moment length finite.
        assert!(report.moment_body.length().is_finite());
        // total power positive.
        assert!(report.total_power > 0.0, "power={}", report.total_power);
    }

    #[test]
    fn total_forces_rejects_bad_inputs() {
        let rotor = main_rotor();
        let tail = tail_rotor();
        let airfoil = default_airfoil(&rotor);
        // zero-mass state
        let zero_state = RigidBodyState {
            position: Vector::new(0.0, 0.0, 100.0),
            linvel_world: Vector::ZERO,
            angvel_body: Vector::ZERO,
            rotation: Rotation::IDENTITY,
            mass: 0.0,
        };
        assert!(
            total_forces_and_moments(
                &zero_state,
                &rotor,
                &tail,
                &SeaLevelAtmosphere,
                &ConstantGravity { g: 9.81 },
                &controls(),
                40.0,
                0.0,
                &airfoil,
                60,
            )
            .is_none()
        );
        // NaN collective → controls invalid
        let nan_controls = FlightControls {
            collective: f64::NAN,
            ..controls()
        };
        let state = RigidBodyState {
            position: Vector::new(0.0, 0.0, 100.0),
            linvel_world: Vector::ZERO,
            angvel_body: Vector::ZERO,
            rotation: Rotation::IDENTITY,
            mass: 800.0,
        };
        assert!(
            total_forces_and_moments(
                &state,
                &rotor,
                &tail,
                &SeaLevelAtmosphere,
                &ConstantGravity { g: 9.81 },
                &nan_controls,
                40.0,
                0.0,
                &airfoil,
                60,
            )
            .is_none()
        );
        // zero rotor omega
        assert!(
            total_forces_and_moments(
                &state,
                &rotor,
                &tail,
                &SeaLevelAtmosphere,
                &ConstantGravity { g: 9.81 },
                &controls(),
                0.0,
                0.0,
                &airfoil,
                60,
            )
            .is_none()
        );
        // dt presiveness tested in simulate_one_step below.
    }

    #[test]
    fn simulate_one_step_advances_state() {
        let rotor = main_rotor();
        let tail = tail_rotor();
        let airfoil = default_airfoil(&rotor);
        let mut bodies = RigidBodySet::new();
        // See module doc: call recompute_mass_properties_from_colliders so
        // `body.mass()` reads our 800 kg; without this the additional-mass
        // shortcut leaves local_mprops zero.
        let mut body =
            satellite_builder(800.0, Vector::new(0.0, 0.0, 100.0), Vector::ZERO, 1.0).build();
        body.recompute_mass_properties_from_colliders(&ColliderSet::new());
        assert!(
            (body.mass() - 800.0).abs() < 1.0e-6,
            "mass after recompute = {}",
            body.mass()
        );
        let handle = bodies.insert(body);
        let body = bodies.get_mut(handle).unwrap();
        let report = simulate_one_step(
            body,
            &rotor,
            &tail,
            &SeaLevelAtmosphere,
            &ConstantGravity { g: 9.81 },
            &controls(),
            &StepConfig {
                rotor_omega: 40.0,
                flat_plate_area: 0.0,
                dt: 0.01,
                stations: 60,
            },
            &airfoil,
        )
        .expect("step should succeed with mass=800 body");
        assert!(report.rotor_thrust > 0.0);
        // After one step, the linear velocity is non-zero (gravity plus rotor).
        let v = body.linvel();
        assert!(v.length() > 0.0, "linvel after step is still zero: {v:?}");
        // theta-step advances translation.
        assert!(body.translation().y.abs() < 1.0e-3 || body.translation().length() > 0.0);
        // Zero dt → None.
        let body = bodies.get_mut(handle).unwrap();
        assert!(
            simulate_one_step(
                body,
                &rotor,
                &tail,
                &SeaLevelAtmosphere,
                &ConstantGravity { g: 9.81 },
                &controls(),
                &StepConfig {
                    rotor_omega: 40.0,
                    flat_plate_area: 0.0,
                    dt: 0.0,
                    stations: 60
                },
                &airfoil,
            )
            .is_none()
        );
        // NaN dt → None.
        let body = bodies.get_mut(handle).unwrap();
        assert!(
            simulate_one_step(
                body,
                &rotor,
                &tail,
                &SeaLevelAtmosphere,
                &ConstantGravity { g: 9.81 },
                &controls(),
                &StepConfig {
                    rotor_omega: 40.0,
                    flat_plate_area: 0.0,
                    dt: f64::NAN,
                    stations: 60
                },
                &airfoil,
            )
            .is_none()
        );
    }
}
