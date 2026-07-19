#[cfg(test)]
mod tests {
    use mps_core::rapier::spaceflight::*;
    use mps_core::rapier::ffi::*;

    #[test]
    fn kepler_period_round_trips_semi_major_axis() {
        let mu = 3.986_004_418e14;
        let a = 7_000_000.0;
        let period = space_kepler_period(mu, a);
        let round_trip = space_kepler_semi_major_axis(mu, period);
        assert!((round_trip - a).abs() < 1.0e-6);
    }

    #[test]
    fn orbital_elements_convert_to_state_and_back() {
        let elements = OrbitalElements {
            semi_major_axis: 7_000_000.0,
            eccentricity: 0.01,
            inclination: 0.3,
            raan: 0.4,
            argument_of_periapsis: 0.5,
            true_anomaly: 0.6,
        };
        let mut state = StateVector::default();
        assert_eq!(
            space_elements_to_state(elements, 3.986_004_418e14, &mut state),
            Bool::TRUE
        );
        let mut out = OrbitalElements::default();
        assert_eq!(
            space_state_to_elements(state, 3.986_004_418e14, &mut out),
            Bool::TRUE
        );
        assert!((out.semi_major_axis - elements.semi_major_axis).abs() < 1.0e-6);
        assert!((out.eccentricity - elements.eccentricity).abs() < 1.0e-10);
    }

    #[test]
    fn engineering_formulas_return_expected_signs() {
        let mut j2 = Vec3::default();
        assert_eq!(
            space_j2_acceleration(
                Vec3 {
                    x: 7_000_000.0,
                    y: 0.0,
                    z: 0.0,
                },
                3.986_004_418e14,
                6_378_137.0,
                1.082_626_68e-3,
                &mut j2,
            ),
            Bool::TRUE
        );
        assert!(j2.x < 0.0);

        let mut cw = CwDerivative::default();
        assert_eq!(
            space_cw_derivative(
                CwState {
                    position: Vec3 {
                        x: 10.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    velocity: Vec3::default(),
                },
                0.001,
                &mut cw,
            ),
            Bool::TRUE
        );
        assert!(cw.acceleration.x > 0.0);
    }

    #[test]
    fn transfer_and_link_formulas_work() {
        let dv = space_tsiolkovsky_delta_v(300.0, 9.80665, 500.0, 300.0);
        assert!(dv > 0.0);

        let mut hohmann = HohmannTransfer::default();
        assert_eq!(
            space_hohmann_transfer(3.986_004_418e14, 7_000_000.0, 42_164_000.0, &mut hohmann),
            Bool::TRUE
        );
        assert!(hohmann.total_delta_v > 0.0);
        assert!(hohmann.transfer_time > 0.0);

        let mut link = FriisLink::default();
        assert_eq!(
            space_friis_link(10.0, 2.0, 2.0, 0.03, 1_000.0, 1.0, &mut link),
            Bool::TRUE
        );
        assert!(link.received_power > 0.0);
    }

    #[test]
    fn estimation_and_attitude_formulas_work() {
        let mut q = Quat::default();
        assert_eq!(
            space_triad_attitude(
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                &mut q,
            ),
            Bool::TRUE
        );
        assert!(q.w > 0.99);

        let gain = space_ekf_gain_scalar(4.0, 1.0, 1.0);
        assert!((gain - 0.8).abs() < 1.0e-12);
        let mut update = ScalarKalman::default();
        assert_eq!(
            space_ekf_update_scalar(10.0, 4.0, 12.0, 10.0, gain, 1.0, &mut update),
            Bool::TRUE
        );
        assert!(update.value > 10.0);
    }

    #[test]
    fn environment_and_vehicle_formulas_work() {
        let density = space_atmospheric_density_scale_height(1.225, 7200.0, 0.0, 7200.0);
        assert!(density > 0.0 && density < 1.225);

        let mut battery = BatteryEquivalentCircuit::default();
        assert_eq!(
            space_battery_equivalent_circuit(
                4.0,
                2.0,
                0.05,
                0.1,
                10.0,
                100.0,
                3600.0,
                &mut battery
            ),
            Bool::TRUE
        );
        assert!(battery.terminal_voltage < 4.0);

        let mut thruster = HallThrusterPerformance::default();
        assert_eq!(
            space_hall_thruster_performance(1.0e-5, 15_000.0, 1_500.0, 9.80665, &mut thruster),
            Bool::TRUE
        );
        assert!(thruster.thrust > 0.0);
    }

    #[test]
    fn guidance_environment_and_control_formulas_work() {
        let mut command = Vec3::default();
        assert_eq!(
            space_artificial_potential_guidance(
                Vec3::default(),
                Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0
                },
                Vec3 {
                    x: -10.0,
                    y: 0.0,
                    z: 0.0
                },
                1.0,
                1.0,
                5.0,
                &mut command,
            ),
            Bool::TRUE
        );
        assert!(command.x > 0.0);

        let mut radiator = RadiatorPower::default();
        assert_eq!(
            space_radiator_power(2.0, 0.8, 300.0, 3.0, 100.0, &mut radiator),
            Bool::TRUE
        );
        assert!(radiator.emitted_power > 0.0);

        let mut airlock = AirlockDepressurization::default();
        assert_eq!(
            space_airlock_depressurization(101_325.0, 0.0, 10.0, 1.0, 1.0, &mut airlock),
            Bool::TRUE
        );
        assert!(airlock.pressure < 101_325.0);
    }

    #[test]
    fn space_formulas_apply_to_rapier_body() {
        let world = mps_core::rapier::world::world_create(Vec3::default());
        let builder = mps_core::rapier::rigid_body::rigid_body_builder_create(
            mps_core::rapier::ffi::BodyStatus::Dynamic as u32,
        );
        mps_core::rapier::rigid_body::rigid_body_builder_set_translation(
            builder,
            Vec3 {
                x: 7_000_000.0,
                y: 0.0,
                z: 0.0,
            },
        );
        mps_core::rapier::rigid_body::rigid_body_builder_set_linvel(
            builder,
            Vec3 {
                x: 7_500.0,
                y: 0.0,
                z: 0.0,
            },
        );
        mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder, 1.0);
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(builder);
        let handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);

        let mut j2 = Vec3::default();
        assert_eq!(
            space_apply_j2_force_to_body(
                world,
                handle,
                3.986_004_418e14,
                6_378_137.0,
                1.082_626_68e-3,
                1.0,
                Bool::TRUE,
                &mut j2,
            ),
            Bool::TRUE
        );
        assert!(j2.x < 0.0);

        let mut drag = Vec3::default();
        assert_eq!(
            space_apply_atmospheric_drag_to_body(
                world,
                handle,
                Vec3::default(),
                1.0e-12,
                2.2,
                1.0,
                1.0,
                Bool::TRUE,
                &mut drag,
            ),
            Bool::TRUE
        );
        assert!(drag.x < 0.0);

        let mut srp = Vec3::default();
        assert_eq!(
            space_apply_solar_radiation_pressure_to_body(
                world,
                handle,
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                1361.0,
                1.2,
                2.0,
                1.0,
                Bool::TRUE,
                &mut srp,
            ),
            Bool::TRUE
        );
        assert!(srp.x > 0.0);

        let mut gravity_gradient = Vec3::default();
        assert_eq!(
            space_apply_gravity_gradient_torque_to_body(
                world,
                handle,
                Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                3.986_004_418e14,
                Bool::TRUE,
                &mut gravity_gradient,
            ),
            Bool::TRUE
        );

        let mut magnetic_dipole = Vec3::default();
        assert_eq!(
            space_apply_magnetic_torquer_to_body(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0,
                },
                Vec3 {
                    x: 1.0e-5,
                    y: 0.0,
                    z: 0.0,
                },
                10.0,
                Bool::TRUE,
                &mut magnetic_dipole,
            ),
            Bool::TRUE
        );
        assert!(magnetic_dipole.y.abs() > 0.0);

        let mut exchange = CmgExchange::default();
        assert_eq!(
            space_apply_cmg_torque_to_body(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0,
                },
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                0.5,
                Bool::TRUE,
                &mut exchange,
            ),
            Bool::TRUE
        );
        assert!(exchange.body_torque.y.abs() > 0.0);

        mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        let velocity = mps_core::rapier::rigid_body::rigid_body_get_linvel(world, handle);
        assert!(velocity.x.is_finite());
        mps_core::rapier::world::world_destroy(world);
    }
}



