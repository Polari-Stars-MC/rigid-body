#[cfg(test)]
mod tests {
    use mps_core::rapier::error::{
        ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, ERR_OK, last_error_code,
    };
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::spaceflight::*;

    #[test]
    fn kepler_period_round_trips_semi_major_axis() {
        let mu = 3.986_004_418e14;
        let a = 7_000_000.0;
        let period = space_kepler_period(mu, a);
        let round_trip = space_kepler_semi_major_axis(mu, period);
        assert!((round_trip - a).abs() < 1.0e-6);
    }

    #[test]
    fn walker_delta_layout_phases_planes_by_f() {
        use mps_formula::spaceflight::walker_delta_layout;

        // GPS Block II "24/3/2": s = 8 sats/plane, RAAN spacing 120°,
        // in-plane spacing 45°, per-plane phasing f·360°/t = 30°.
        // idx=17 → plane 2, pos 1 → RAAN=240°, M = 45° + 2·30° = 105°.
        let (raan, m) = walker_delta_layout(24, 3, 2, 17).unwrap();
        assert!((raan - 240.0).abs() < 1.0e-9, "raan={raan}");
        assert!((m - 105.0).abs() < 1.0e-9, "m={m}");

        // f must change the *relative* geometry between planes: with f=0 the
        // same idx sits at M=45°, with f=1 at M=75° (per-plane offset 15°
        // × plane index 2).
        let (_, m_f0) = walker_delta_layout(24, 3, 0, 17).unwrap();
        let (_, m_f1) = walker_delta_layout(24, 3, 1, 17).unwrap();
        assert!((m_f0 - 45.0).abs() < 1.0e-9, "m_f0={m_f0}");
        assert!((m_f1 - 75.0).abs() < 1.0e-9, "m_f1={m_f1}");

        // First satellite of each plane: pos=0, so M is pure phasing.
        let (_, m_plane1) = walker_delta_layout(24, 3, 2, 8).unwrap();
        assert!((m_plane1 - 30.0).abs() < 1.0e-9, "m_plane1={m_plane1}");

        // Constraint violations.
        assert!(walker_delta_layout(0, 3, 0, 0).is_none());
        assert!(walker_delta_layout(24, 0, 0, 0).is_none());
        assert!(walker_delta_layout(24, 3, 3, 0).is_none()); // f >= p
        assert!(walker_delta_layout(24, 3, 0, 24).is_none()); // idx >= t
        assert!(walker_delta_layout(24, 5, 0, 0).is_none()); // p does not divide t
        assert!(walker_delta_layout(24, 30, 0, 0).is_none()); // p > t
    }

    #[test]
    fn sun_synchronous_inclination_600km_is_about_97_8_deg() {
        use mps_formula::spaceflight::sun_synchronous_inclination;

        // Earth, 600 km LEO, one-rev-per-year RAAN precession ⇒ i ≈ 97.8°.
        let i = sun_synchronous_inclination(6378.0, 600.0, 3.986_004_418e5, 1.991e-7).unwrap();
        assert!((i - 97.79).abs() < 0.05, "i={i}");
        // NaN / invalid inputs must be rejected, not returned as Some(NaN).
        assert!(sun_synchronous_inclination(f64::NAN, 600.0, 3.986e5, 1.991e-7).is_none());
        assert!(sun_synchronous_inclination(6378.0, f64::NAN, 3.986e5, 1.991e-7).is_none());
        assert!(sun_synchronous_inclination(6378.0, 600.0, f64::NAN, 1.991e-7).is_none());
        assert!(sun_synchronous_inclination(6378.0, 600.0, 3.986e5, f64::NAN).is_none());
        assert!(sun_synchronous_inclination(6378.0, 600.0, 3.986e5, 0.0).is_none());
        // Rate too large for J2 at this altitude → no real inclination.
        assert!(sun_synchronous_inclination(6378.0, 600.0, 3.986e5, 1.0).is_none());
    }

    #[test]
    fn molniya_critical_elements_are_the_design_point() {
        use mps_formula::spaceflight::molniya_critical_elements;

        let (inclination_deg, arg_perigee_deg) = molniya_critical_elements();
        assert!((inclination_deg - 63.4).abs() < 1.0e-12);
        assert!((arg_perigee_deg - 270.0).abs() < 1.0e-12);
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

    #[test]
    fn attitude_kinematics_formulas_work() {
        let mut derivative = QuaternionDerivative::default();
        assert_eq!(
            space_quaternion_derivative(
                Quat {
                    i: 0.0,
                    j: 0.0,
                    k: 0.0,
                    w: 1.0,
                },
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                &mut derivative,
            ),
            Bool::TRUE
        );
        assert!((derivative.i_dot - 0.5).abs() < 1.0e-12);
        assert!(derivative.j_dot.abs() < 1.0e-12);
        assert!(derivative.w_dot.abs() < 1.0e-12);

        let mut euler = RigidBodyEulerDerivative::default();
        assert_eq!(
            space_rigid_body_euler_derivative(
                Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                Vec3::default(),
                Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                &mut euler,
            ),
            Bool::TRUE
        );
        assert!((euler.angular_acceleration.x - 1.0).abs() < 1.0e-12);
        assert!((euler.angular_acceleration.z - 1.0).abs() < 1.0e-12);

        let mut exchange = CmgExchange::default();
        assert_eq!(
            space_cmg_exchange(
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
                &mut exchange,
            ),
            Bool::TRUE
        );
        assert!((exchange.wheel_momentum_dot.y - 0.5).abs() < 1.0e-12);
        assert!((exchange.body_torque.y + 0.5).abs() < 1.0e-12);
    }

    #[test]
    fn manipulator_formulas_work() {
        let mut transform = DhTransform::default();
        assert_eq!(
            space_dh_transform(0.0, 0.5, 1.0, 0.0, &mut transform),
            Bool::TRUE
        );
        assert!((transform.m00 - 1.0).abs() < 1.0e-12);
        assert!((transform.m03 - 1.0).abs() < 1.0e-12);
        assert!((transform.m23 - 0.5).abs() < 1.0e-12);
        assert!((transform.m33 - 1.0).abs() < 1.0e-12);

        let first = space_arm_first_joint_inverse(1.0, 1.0);
        assert!((first - std::f64::consts::FRAC_PI_4).abs() < 1.0e-12);

        let elbow_up = space_arm_third_joint_angle(1.0, 0.0, 1.0, 1.0, Bool::TRUE);
        assert!(elbow_up > 0.0);
        let elbow_down = space_arm_third_joint_angle(1.0, 0.0, 1.0, 1.0, Bool::FALSE);
        assert!((elbow_up + elbow_down).abs() < 1.0e-12);

        let mut dynamics = ManipulatorDynamics::default();
        assert_eq!(
            space_manipulator_dynamics_diag(
                Vec3 {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
                Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                Vec3::default(),
                Vec3::default(),
                &mut dynamics,
            ),
            Bool::TRUE
        );
        assert!((dynamics.torque.x - 1.0).abs() < 1.0e-12);
        assert!((dynamics.torque.z - 3.0).abs() < 1.0e-12);
    }

    #[test]
    fn power_thermal_and_life_support_formulas_work() {
        let mut panel = SolarPanelPower::default();
        assert_eq!(
            space_solar_panel_power(1361.0, 2.0, 0.3, 0.0, 1.0, &mut panel),
            Bool::TRUE
        );
        assert!((panel.incident_power - 2722.0).abs() < 1.0e-9);
        assert!((panel.electrical_power - 816.6).abs() < 1.0e-9);

        let mut thermal = ThermalBalance::default();
        assert_eq!(
            space_thermal_balance(100.0, 0.0, 1.0, 1.0, &mut thermal),
            Bool::TRUE
        );
        assert!((thermal.net_power - 100.0).abs() < 1.0e-12);
        assert!(thermal.equilibrium_temperature > 200.0);
        assert!(thermal.equilibrium_temperature < 210.0);

        let mut co2 = Co2MassBalance::default();
        assert_eq!(
            space_co2_mass_balance(1.0, 0.1, 0.05, 0.0, 10.0, 10.0, &mut co2),
            Bool::TRUE
        );
        assert!((co2.mass_rate - 0.05).abs() < 1.0e-12);
        assert!((co2.next_mass - 1.5).abs() < 1.0e-12);

        let resistance = space_heat_pipe_thermal_resistance(0.1, 0.01, 0.1, 0.02);
        assert!((resistance - 0.23).abs() < 1.0e-12);

        let mut loop_heat = FluidLoopHeatTransfer::default();
        assert_eq!(
            space_single_phase_loop_heat_transfer(0.1, 4186.0, 293.0, 418.6, &mut loop_heat),
            Bool::TRUE
        );
        assert!((loop_heat.outlet_temperature - 294.0).abs() < 1.0e-9);

        let mut sabatier = ChemicalReactionRate::default();
        assert_eq!(
            space_sabatier_methane_rate(1.0, 8.0, 0.5, &mut sabatier),
            Bool::TRUE
        );
        assert!((sabatier.reactant_rate - 0.5).abs() < 1.0e-12);
        assert!((sabatier.product_rate - 0.5).abs() < 1.0e-12);

        let mut oxygen = ChemicalReactionRate::default();
        assert_eq!(
            space_spe_oxygen_rate(10.0, 2.0, 0.9, &mut oxygen),
            Bool::TRUE
        );
        assert!(oxygen.product_rate > 0.0);
        assert!(oxygen.reactant_rate > oxygen.product_rate);
    }

    #[test]
    fn navigation_and_sensor_formulas_work() {
        let mut observation = GnssObservation::default();
        assert_eq!(
            space_gnss_pseudorange(
                Vec3::default(),
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 20_200_000.0,
                },
                0.0,
                0.0,
                0.0,
                0.0,
                &mut observation,
            ),
            Bool::TRUE
        );
        assert!((observation.geometric_range - 20_200_000.0).abs() < 1.0e-6);
        assert!((observation.value - observation.geometric_range).abs() < 1.0e-6);

        let phase = space_gnss_double_difference_carrier_phase(100.0, 90.0, 95.0, 85.0, 0.19, 2.0);
        assert!((phase - 2.0).abs() < 1.0e-12);

        let mut radar = RadarMeasurement::default();
        assert_eq!(
            space_radar_range_rate(
                Vec3::default(),
                Vec3 {
                    x: 100.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3::default(),
                Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
                &mut radar,
            ),
            Bool::TRUE
        );
        assert!((radar.range - 100.0).abs() < 1.0e-12);
        assert!((radar.range_rate - 10.0).abs() < 1.0e-12);

        assert!(space_sagnac_phase_rate(1.0, 0.01, 1.55e-6) > 0.0);

        let mut prediction = ScalarKalman::default();
        assert_eq!(
            space_ekf_predict_scalar(10.0, 4.0, 1.0, 1.0, 0.1, &mut prediction),
            Bool::TRUE
        );
        assert!((prediction.value - 11.0).abs() < 1.0e-12);
        assert!((prediction.covariance - 4.1).abs() < 1.0e-12);

        let mut attitude = LeastSquaresAttitude::default();
        assert_eq!(
            space_least_squares_attitude_two_vector(
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
                &mut attitude,
            ),
            Bool::TRUE
        );
        assert!(attitude.attitude.w > 0.99);
        assert!(attitude.rms_error.abs() < 1.0e-12);
    }

    #[test]
    fn orbit_environment_and_debris_formulas_work() {
        let decay = space_semi_major_axis_decay_rate(
            7_000_000.0,
            1.0e-12,
            2.2,
            1.0,
            500.0,
            3.986_004_418e14,
        );
        assert!(decay < 0.0);

        let mut rates = Sgp4SecularRates::default();
        assert_eq!(
            space_sgp4_j2_secular_rates(
                7_000_000.0,
                0.01,
                0.5,
                0.001,
                6_378_137.0,
                1.082_626_68e-3,
                &mut rates,
            ),
            Bool::TRUE
        );
        assert!(rates.raan_dot < 0.0);
        assert!(rates.mean_motion_dot.abs() < 1.0e-12);

        let mut variational = VariationalState::default();
        assert_eq!(
            space_variational_two_body(
                Vec3 {
                    x: 7_000_000.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: 7_500.0,
                    z: 0.0,
                },
                3.986_004_418e14,
                &mut variational,
            ),
            Bool::TRUE
        );
        assert!((variational.position_dot.y - 7_500.0).abs() < 1.0e-9);
        assert!(variational.velocity_dot.x < 0.0);

        let mut collision = CollisionProbability::default();
        assert_eq!(
            space_debris_collision_probability(0.0, 10.0, 100.0, 1000.0, &mut collision),
            Bool::TRUE
        );
        assert!(collision.probability > 0.0 && collision.probability <= 1.0);
        assert!(collision.combined_sigma > 0.0);

        let mut erosion = AtomicOxygenErosion::default();
        assert_eq!(
            space_atomic_oxygen_erosion(1.0e20, 3.0e-24, 1.0, 1000.0, &mut erosion),
            Bool::TRUE
        );
        assert!((erosion.volume_loss - 3.0e-4).abs() < 1.0e-16);
        assert!((erosion.mass_loss - 0.3).abs() < 1.0e-12);

        let diameter =
            space_whipple_critical_projectile_diameter(0.001, 2700.0, 7800.0, 7_000.0, 0.1);
        assert!(diameter > 0.0);

        let dose = space_radiation_absorbed_dose(10.0, 2.0, 1.0);
        assert!((dose - 5.0).abs() < 1.0e-12);

        let balance = space_surface_charging_current_balance(1.0, 1.0, 1.0, 3.0, 1.0);
        assert!((balance - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn guidance_structures_and_docking_formulas_work() {
        let lambert = space_lambert_time_elliptic(3.986_004_418e14, 10_000_000.0, 1.0, 0.5, 0);
        assert!(lambert.is_finite() && lambert > 0.0);

        let frequency = space_structural_natural_frequency(10_000.0, 100.0, 1.0);
        assert!((frequency - 10.0 / std::f64::consts::TAU).abs() < 1.0e-12);

        let mut contact = ContactForceModel::default();
        assert_eq!(
            space_contact_force_hunt_crossley(0.01, 0.5, 1000.0, 10.0, 1.5, &mut contact),
            Bool::TRUE
        );
        assert!((contact.normal_force - 1.0).abs() < 1.0e-12);
        assert!(contact.total_force > contact.normal_force);

        let mut mode = FlexibleModeDerivative::default();
        assert_eq!(
            space_flexible_mode_derivative(1.0, 0.0, 2.0, 0.1, 0.0, 1.0, &mut mode),
            Bool::TRUE
        );
        assert!(mode.displacement_dot.abs() < 1.0e-12);
        assert!((mode.velocity_dot + 4.0).abs() < 1.0e-12);

        let mut slosh = SloshPendulumDerivative::default();
        assert_eq!(
            space_slosh_pendulum_derivative(0.1, 0.0, 1.0, 0.0, 0.0, 9.81, &mut slosh),
            Bool::TRUE
        );
        assert!(slosh.angle_dot.abs() < 1.0e-12);
        assert!(slosh.angular_rate_dot < 0.0);

        let mut properties = MassProperties::default();
        assert_eq!(
            space_mass_properties_two_body(
                1.0,
                Vec3 {
                    x: -1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3::default(),
                1.0,
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3::default(),
                &mut properties,
            ),
            Bool::TRUE
        );
        assert!(properties.center_of_mass.x.abs() < 1.0e-12);
        assert!(properties.inertia_diag.x.abs() < 1.0e-12);
        assert!((properties.inertia_diag.y - 2.0).abs() < 1.0e-12);
        assert!((properties.inertia_diag.z - 2.0).abs() < 1.0e-12);

        let energy = space_docking_buffer_energy(0.1, 100.0, 0.05, 0.5);
        assert!((energy - 1.0).abs() < 1.0e-12);

        let mut profile = BangOffBangProfile::default();
        assert_eq!(
            space_bang_off_bang_profile(0.1, 1.0, 1.0, &mut profile),
            Bool::TRUE
        );
        assert!(profile.coast_time.abs() < 1.0e-12);
        assert!((profile.total_time - 2.0 * 0.1_f64.sqrt()).abs() < 1.0e-12);

        let command = space_docking_glideslope_command(100.0, 0.01, 5.0);
        assert!((command + 1.0).abs() < 1.0e-12);

        let torque = space_solar_array_pd_torque(0.1, 0.05, 1.0, 2.0);
        assert!((torque - 0.2).abs() < 1.0e-12);

        let mut inverse = CmgRobustInverse::default();
        assert_eq!(
            space_cmg_robust_pseudoinverse_diag(
                Vec3 {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
                Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                0.1,
                &mut inverse,
            ),
            Bool::TRUE
        );
        assert!((inverse.gimbal_rates.x - 1.0 / 1.01).abs() < 1.0e-12);
        assert!((inverse.damping - 0.1).abs() < 1.0e-12);

        let wavelength = space_friis_wavelength_from_frequency(10.0e9);
        assert!((wavelength - 0.029_979_245_8).abs() < 1.0e-12);
    }

    #[test]
    fn pure_acceleration_and_torque_wrappers_work() {
        let mut drag = Vec3::default();
        assert_eq!(
            space_atmospheric_drag_acceleration(
                Vec3 {
                    x: 7_500.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3::default(),
                1.0e-12,
                2.2,
                1.0,
                1.0,
                &mut drag,
            ),
            Bool::TRUE
        );
        assert!(drag.x < 0.0);

        let mut srp = Vec3::default();
        assert_eq!(
            space_solar_radiation_pressure_acceleration(
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                1361.0,
                1.2,
                2.0,
                1.0,
                &mut srp,
            ),
            Bool::TRUE
        );
        assert!(srp.x > 0.0);

        let arm = 7_000_000.0 / 2.0_f64.sqrt();
        let mut gravity_gradient = Vec3::default();
        assert_eq!(
            space_gravity_gradient_torque(
                Vec3 {
                    x: arm,
                    y: arm,
                    z: 0.0,
                },
                Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                3.986_004_418e14,
                &mut gravity_gradient,
            ),
            Bool::TRUE
        );
        assert!(gravity_gradient.z > 0.0);

        let mut dipole = Vec3::default();
        assert_eq!(
            space_magnetic_torquer_dipole(
                Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0e-5,
                },
                10.0,
                &mut dipole,
            ),
            Bool::TRUE
        );
        // The unconstrained dipole would be huge, so it must clamp to max_dipole.
        let magnitude = (dipole.x * dipole.x + dipole.y * dipole.y + dipole.z * dipole.z).sqrt();
        assert!((magnitude - 10.0).abs() < 1.0e-9);
    }

    #[test]
    fn apply_flag_variants_match_bool_wrappers() {
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
        mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder, 1.0);
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(builder);
        let handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);

        let mut out = Vec3::default();
        assert_eq!(
            space_apply_j2_force_to_body_flag(
                world,
                handle,
                3.986_004_418e14,
                6_378_137.0,
                1.082_626_68e-3,
                1.0,
                Bool::TRUE,
                &mut out,
            ),
            1
        );

        let mut exchange = CmgExchange::default();
        assert_eq!(
            space_apply_cmg_torque_to_body_flag(
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
            1
        );

        assert_eq!(
            space_apply_atmospheric_drag_to_body_flag(
                world,
                handle,
                Vec3::default(),
                1.0e-12,
                2.2,
                1.0,
                1.0,
                Bool::TRUE,
                &mut out,
            ),
            1
        );

        assert_eq!(
            space_apply_solar_radiation_pressure_to_body_flag(
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
                &mut out,
            ),
            1
        );

        assert_eq!(
            space_apply_gravity_gradient_torque_to_body_flag(
                world,
                handle,
                Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                3.986_004_418e14,
                Bool::TRUE,
                &mut out,
            ),
            1
        );

        assert_eq!(
            space_apply_magnetic_torquer_to_body_flag(
                world,
                handle,
                Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0e-5,
                },
                10.0,
                Bool::TRUE,
                &mut out,
            ),
            1
        );

        // A null world must fail through the flag path as well.
        assert_eq!(
            space_apply_j2_force_to_body_flag(
                std::ptr::null_mut(),
                handle,
                3.986_004_418e14,
                6_378_137.0,
                1.082_626_68e-3,
                1.0,
                Bool::TRUE,
                &mut out,
            ),
            0
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        mps_core::rapier::world::world_destroy(world);
    }

    #[test]
    fn invalid_orbit_and_attitude_inputs_report_invalid_argument() {
        let mut state = StateVector::default();
        assert_eq!(
            space_elements_to_state(
                OrbitalElements {
                    semi_major_axis: 7_000_000.0,
                    eccentricity: 1.5,
                    inclination: 0.0,
                    raan: 0.0,
                    argument_of_periapsis: 0.0,
                    true_anomaly: 0.0,
                },
                3.986_004_418e14,
                &mut state,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut elements = OrbitalElements::default();
        assert_eq!(
            space_state_to_elements(StateVector::default(), 3.986_004_418e14, &mut elements),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut vec_out = Vec3::default();
        assert_eq!(
            space_j2_acceleration(
                Vec3::default(),
                3.986_004_418e14,
                6_378_137.0,
                1.082_626_68e-3,
                &mut vec_out,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut derivative = QuaternionDerivative::default();
        assert_eq!(
            space_quaternion_derivative(
                Quat {
                    i: 0.0,
                    j: 0.0,
                    k: 0.0,
                    w: 1.0,
                },
                Vec3 {
                    x: f64::NAN,
                    y: 0.0,
                    z: 0.0,
                },
                &mut derivative,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut euler = RigidBodyEulerDerivative::default();
        assert_eq!(
            space_rigid_body_euler_derivative(
                Vec3::default(),
                Vec3::default(),
                Vec3::default(),
                &mut euler,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut exchange = CmgExchange::default();
        assert_eq!(
            space_cmg_exchange(
                Vec3::default(),
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                0.5,
                &mut exchange,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut cw = CwDerivative::default();
        assert_eq!(
            space_cw_derivative(
                CwState {
                    position: Vec3::default(),
                    velocity: Vec3::default(),
                },
                f64::NAN,
                &mut cw,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Parallel TRIAD vectors cannot form a basis.
        let mut attitude = Quat::default();
        assert_eq!(
            space_triad_attitude(
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 2.0,
                    y: 0.0,
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
                &mut attitude,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut least_squares = LeastSquaresAttitude::default();
        assert_eq!(
            space_least_squares_attitude_two_vector(
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 2.0,
                    y: 0.0,
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
                &mut least_squares,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut kalman = ScalarKalman::default();
        assert_eq!(
            space_ekf_predict_scalar(10.0, -1.0, 1.0, 1.0, 0.1, &mut kalman),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert_eq!(
            space_ekf_update_scalar(10.0, 4.0, f64::NAN, 10.0, 0.8, 1.0, &mut kalman),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut variational = VariationalState::default();
        assert_eq!(
            space_variational_two_body(
                Vec3::default(),
                Vec3::default(),
                3.986_004_418e14,
                &mut variational,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut rates = Sgp4SecularRates::default();
        assert_eq!(
            space_sgp4_j2_secular_rates(
                7_000_000.0,
                1.5,
                0.5,
                0.001,
                6_378_137.0,
                1.082_626_68e-3,
                &mut rates,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut hohmann = HohmannTransfer::default();
        assert_eq!(
            space_hohmann_transfer(3.986_004_418e14, 0.0, 42_164_000.0, &mut hohmann),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut profile = BangOffBangProfile::default();
        assert_eq!(
            space_bang_off_bang_profile(0.1, 0.0, 1.0, &mut profile),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert_eq!(
            space_magnetic_torquer_dipole(
                Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                Vec3::default(),
                10.0,
                &mut vec_out,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut inverse = CmgRobustInverse::default();
        assert_eq!(
            space_cmg_robust_pseudoinverse_diag(
                Vec3 {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
                Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                -0.1,
                &mut inverse,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert_eq!(
            space_gravity_gradient_torque(
                Vec3::default(),
                Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                3.986_004_418e14,
                &mut vec_out,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn invalid_vehicle_and_environment_inputs_report_invalid_argument() {
        let mut transform = DhTransform::default();
        assert_eq!(
            space_dh_transform(f64::NAN, 0.5, 1.0, 0.0, &mut transform),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut dynamics = ManipulatorDynamics::default();
        assert_eq!(
            space_manipulator_dynamics_diag(
                Vec3 {
                    x: 1.0,
                    y: f64::NAN,
                    z: 1.0,
                },
                Vec3::default(),
                Vec3::default(),
                Vec3::default(),
                &mut dynamics,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut panel = SolarPanelPower::default();
        assert_eq!(
            space_solar_panel_power(1361.0, 2.0, -0.1, 0.0, 1.0, &mut panel),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut thermal = ThermalBalance::default();
        assert_eq!(
            space_thermal_balance(100.0, 0.0, 0.0, 1.0, &mut thermal),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut co2 = Co2MassBalance::default();
        assert_eq!(
            space_co2_mass_balance(1.0, 0.1, 0.05, 0.0, 0.0, 10.0, &mut co2),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut link = FriisLink::default();
        assert_eq!(
            space_friis_link(10.0, 2.0, 2.0, 0.03, 0.0, 1.0, &mut link),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut vec_out = Vec3::default();
        assert_eq!(
            space_atmospheric_drag_acceleration(
                Vec3 {
                    x: 7_500.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3::default(),
                1.0e-12,
                2.2,
                1.0,
                0.0,
                &mut vec_out,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut observation = GnssObservation::default();
        assert_eq!(
            space_gnss_pseudorange(
                Vec3 {
                    x: f64::NAN,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 20_200_000.0,
                },
                0.0,
                0.0,
                0.0,
                0.0,
                &mut observation,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut contact = ContactForceModel::default();
        assert_eq!(
            space_contact_force_hunt_crossley(0.01, 0.5, 1000.0, 10.0, 0.0, &mut contact),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut battery = BatteryEquivalentCircuit::default();
        assert_eq!(
            space_battery_equivalent_circuit(4.0, 2.0, 0.05, 0.1, 10.0, 0.0, 3600.0, &mut battery,),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut thruster = HallThrusterPerformance::default();
        assert_eq!(
            space_hall_thruster_performance(1.0e-5, 15_000.0, 0.0, 9.80665, &mut thruster),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert_eq!(
            space_artificial_potential_guidance(
                Vec3::default(),
                Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: -10.0,
                    y: 0.0,
                    z: 0.0,
                },
                1.0,
                1.0,
                0.0,
                &mut vec_out,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut collision = CollisionProbability::default();
        assert_eq!(
            space_debris_collision_probability(0.0, 10.0, 0.0, 1000.0, &mut collision),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut erosion = AtomicOxygenErosion::default();
        assert_eq!(
            space_atomic_oxygen_erosion(-1.0, 3.0e-24, 1.0, 1000.0, &mut erosion),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut mode = FlexibleModeDerivative::default();
        assert_eq!(
            space_flexible_mode_derivative(1.0, 0.0, 2.0, 0.1, 0.0, 0.0, &mut mode),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut slosh = SloshPendulumDerivative::default();
        assert_eq!(
            space_slosh_pendulum_derivative(0.1, 0.0, 0.0, 0.0, 0.0, 9.81, &mut slosh),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut loop_heat = FluidLoopHeatTransfer::default();
        assert_eq!(
            space_single_phase_loop_heat_transfer(0.0, 4186.0, 293.0, 418.6, &mut loop_heat),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut radar = RadarMeasurement::default();
        assert_eq!(
            space_radar_range_rate(
                Vec3::default(),
                Vec3::default(),
                Vec3::default(),
                Vec3::default(),
                &mut radar,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut properties = MassProperties::default();
        assert_eq!(
            space_mass_properties_two_body(
                0.0,
                Vec3::default(),
                Vec3::default(),
                0.0,
                Vec3::default(),
                Vec3::default(),
                &mut properties,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert_eq!(
            space_solar_radiation_pressure_acceleration(
                Vec3::default(),
                1361.0,
                1.2,
                2.0,
                1.0,
                &mut vec_out,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut rate = ChemicalReactionRate::default();
        assert_eq!(
            space_sabatier_methane_rate(1.0, 8.0, 1.5, &mut rate),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert_eq!(
            space_spe_oxygen_rate(10.0, 0.0, 0.9, &mut rate),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut radiator = RadiatorPower::default();
        assert_eq!(
            space_radiator_power(-1.0, 0.8, 300.0, 3.0, 100.0, &mut radiator),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        let mut airlock = AirlockDepressurization::default();
        assert_eq!(
            space_airlock_depressurization(101_325.0, 0.0, 0.0, 1.0, 1.0, &mut airlock),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn invalid_scalar_inputs_return_nan_and_report_invalid_argument() {
        assert!(space_kepler_period(0.0, 7_000_000.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_kepler_semi_major_axis(3.986_004_418e14, 0.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_lambert_time_elliptic(0.0, 10_000_000.0, 1.0, 0.5, 0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_arm_first_joint_inverse(0.0, 0.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // Geometrically unreachable target for the given link lengths.
        assert!(space_arm_third_joint_angle(10.0, 0.0, 1.0, 1.0, Bool::TRUE).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_friis_wavelength_from_frequency(0.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_tsiolkovsky_delta_v(300.0, 9.80665, 300.0, 500.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_atmospheric_density_scale_height(1.225, 100.0, 0.0, 0.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_ekf_gain_scalar(0.0, 1.0, 0.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(
            space_gnss_double_difference_carrier_phase(100.0, 90.0, 95.0, 85.0, 0.0, 2.0).is_nan()
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_structural_natural_frequency(10_000.0, 0.0, 1.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_radiation_absorbed_dose(10.0, 0.0, 1.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(
            space_semi_major_axis_decay_rate(
                7_000_000.0,
                1.0e-12,
                2.2,
                1.0,
                0.0,
                3.986_004_418e14,
            )
            .is_nan()
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_heat_pipe_thermal_resistance(f64::NAN, 0.1, 0.1, 0.1).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_docking_buffer_energy(0.1, 100.0, 0.0, 0.5).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_docking_glideslope_command(100.0, 0.01, -1.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_sagnac_phase_rate(1.0, 0.01, 0.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_solar_array_pd_torque(f64::NAN, 0.0, 1.0, 2.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(
            space_whipple_critical_projectile_diameter(0.001, 2700.0, 7800.0, 0.0, 0.1).is_nan()
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert!(space_surface_charging_current_balance(f64::NAN, 1.0, 1.0, 1.0, 1.0).is_nan());
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // A successful call resets the error slot.
        assert!(space_kepler_period(3.986_004_418e14, 7_000_000.0) > 0.0);
        assert_eq!(last_error_code(), ERR_OK);
    }

    #[test]
    fn null_output_pointers_report_invalid_argument() {
        assert_eq!(
            space_hohmann_transfer(
                3.986_004_418e14,
                7_000_000.0,
                42_164_000.0,
                std::ptr::null_mut(),
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert_eq!(
            space_quaternion_derivative(
                Quat {
                    i: 0.0,
                    j: 0.0,
                    k: 0.0,
                    w: 1.0,
                },
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                std::ptr::null_mut(),
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        assert_eq!(
            space_friis_link(10.0, 2.0, 2.0, 0.03, 1_000.0, 1.0, std::ptr::null_mut()),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
    }

    #[test]
    fn apply_wrappers_validate_world_body_and_mass() {
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
        mps_core::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder, 1.0);
        let body = mps_core::rapier::rigid_body::rigid_body_builder_build(builder);
        let handle = mps_core::rapier::rigid_body::world_insert_rigid_body(world, body);

        let mut out = Vec3::default();

        // Null world -> ERR_NULL_POINTER.
        assert_eq!(
            space_apply_atmospheric_drag_to_body(
                std::ptr::null_mut(),
                handle,
                Vec3::default(),
                1.0e-12,
                2.2,
                1.0,
                1.0,
                Bool::TRUE,
                &mut out,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);

        // Unknown body handle -> ERR_NOT_FOUND.
        let mut exchange = CmgExchange::default();
        assert_eq!(
            space_apply_cmg_torque_to_body(
                world,
                u64::MAX,
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
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);

        // Non-positive mass is rejected before the world lookup.
        assert_eq!(
            space_apply_j2_force_to_body(
                std::ptr::null_mut(),
                handle,
                3.986_004_418e14,
                6_378_137.0,
                1.082_626_68e-3,
                0.0,
                Bool::TRUE,
                &mut out,
            ),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

        // A null output pointer is tolerated by apply wrappers.
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
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert_eq!(last_error_code(), ERR_OK);

        mps_core::rapier::world::world_destroy(world);
    }

    /// `_checked` 公式迁移守门：checked 变体与 `Option` 包装对合法输入**逐位
    /// 一致**；域违例返回携带参数名的 `FormulaError`，不再是无差别的 `None`。
    #[test]
    fn checked_formula_variants_match_option_wrappers_and_report_domains() {
        use mps_formula::FormulaError;
        use mps_formula::spaceflight::debris::{
            radiation_absorbed_dose, radiation_absorbed_dose_checked,
        };
        use mps_formula::spaceflight::kepler::{
            bi_elliptic_transfer_delta_v, bi_elliptic_transfer_delta_v_checked,
            combined_maneuver_delta_v, combined_maneuver_delta_v_checked, hohmann_transfer,
            hohmann_transfer_checked,
        };
        use mps_formula::spaceflight::thermal::{
            heat_pipe_thermal_resistance, heat_pipe_thermal_resistance_checked, radiator_power,
            radiator_power_checked, reentry_peak_g_load, reentry_peak_g_load_checked,
            single_phase_loop_heat_transfer, single_phase_loop_heat_transfer_checked,
            sutton_graves_heat_rate, sutton_graves_heat_rate_checked, thermal_balance,
            thermal_balance_checked,
        };

        let mu = 3.986_004_418e14;

        // --- 合法输入：checked 与 Option 包装逐位一致 ---
        let (dv1, dv2) = {
            let h = hohmann_transfer(mu, 7.0e6, 4.2164e7).unwrap();
            let hc = hohmann_transfer_checked(mu, 7.0e6, 4.2164e7).unwrap();
            assert_eq!(hc.delta_v1, h.delta_v1);
            assert_eq!(hc.delta_v2, h.delta_v2);
            assert_eq!(hc.total_delta_v, h.total_delta_v);
            assert_eq!(hc.transfer_time, h.transfer_time);
            (h.delta_v1, h.delta_v2)
        };
        assert!(dv1 > 0.0 && dv2 > 0.0, "raising orbit: both burns prograde");

        let be = bi_elliptic_transfer_delta_v(mu, 7.0e6, 4.2164e7, 8.0e7).unwrap();
        let bec = bi_elliptic_transfer_delta_v_checked(mu, 7.0e6, 4.2164e7, 8.0e7).unwrap();
        assert_eq!((be.0, be.1, be.2, be.3), bec);

        let cm = combined_maneuver_delta_v(mu, 7.0e6, 4.2164e7, 0.1).unwrap();
        let cmc = combined_maneuver_delta_v_checked(mu, 7.0e6, 4.2164e7, 0.1).unwrap();
        assert_eq!(cm, cmc);

        let hp = heat_pipe_thermal_resistance(0.1, 0.2, 0.3, 0.05).unwrap();
        assert_eq!(
            hp,
            heat_pipe_thermal_resistance_checked(0.1, 0.2, 0.3, 0.05).unwrap()
        );

        let rp = radiator_power(2.0, 0.85, 300.0, 3.0, 50.0).unwrap();
        let rpc = radiator_power_checked(2.0, 0.85, 300.0, 3.0, 50.0).unwrap();
        assert_eq!(rp.emitted_power, rpc.emitted_power);
        assert_eq!(rp.net_power, rpc.net_power);

        let gl = reentry_peak_g_load(100.0, 7800.0, -0.05, 8500.0).unwrap();
        assert_eq!(
            gl,
            reentry_peak_g_load_checked(100.0, 7800.0, -0.05, 8500.0).unwrap()
        );

        let lp = single_phase_loop_heat_transfer(0.5, 4180.0, 293.0, 1000.0).unwrap();
        let lpc = single_phase_loop_heat_transfer_checked(0.5, 4180.0, 293.0, 1000.0).unwrap();
        assert_eq!(lp.heat_rate, lpc.heat_rate);
        assert_eq!(lp.outlet_temperature, lpc.outlet_temperature);

        let sg = sutton_graves_heat_rate(1e-4, 7800.0, 0.5, 1.83e-4).unwrap();
        assert_eq!(
            sg,
            sutton_graves_heat_rate_checked(1e-4, 7800.0, 0.5, 1.83e-4).unwrap()
        );

        let tb = thermal_balance(800.0, 100.0, 4.0, 0.9).unwrap();
        let tbc = thermal_balance_checked(800.0, 100.0, 4.0, 0.9).unwrap();
        assert_eq!(tb.net_power, tbc.net_power);
        assert_eq!(tb.equilibrium_temperature, tbc.equilibrium_temperature);

        let dose = radiation_absorbed_dose_checked(10.0, 2.0, 1.0).unwrap();
        assert_eq!(dose, 5.0);

        // --- 域违例：报错携带参数名（不是笼统 None）---
        // Hohmann：半径非正。
        match hohmann_transfer_checked(mu, 0.0, 4.2164e7) {
            Err(FormulaError::OutOfDomain { parameter, .. }) => {
                assert_eq!(parameter, "radius1");
            }
            other => panic!("expected OutOfDomain(radius1), got {other:?}"),
        }
        // 双椭圆：mu 非有限。
        assert!(matches!(
            bi_elliptic_transfer_delta_v_checked(f64::NAN, 1.0, 1.0, 1.0),
            Err(FormulaError::NonFiniteInput { parameter: "mu" })
        ));
        // 组合机动：倾角变化为负。
        assert!(matches!(
            combined_maneuver_delta_v_checked(mu, 7.0e6, 4.2164e7, -0.1),
            Err(FormulaError::OutOfDomain {
                parameter: "inclination_change",
                ..
            })
        ));
        // 再入 g 载：近掠入射角（sin γ < EPS）。
        assert!(matches!(
            reentry_peak_g_load_checked(100.0, 7800.0, 1e-15, 8500.0),
            Err(FormulaError::OutOfDomain {
                parameter: "entry_angle",
                ..
            })
        ));
        // 吸收剂量：质量非正。
        assert!(matches!(
            radiation_absorbed_dose_checked(10.0, 0.0, 1.0),
            Err(FormulaError::OutOfDomain {
                parameter: "mass_kg",
                ..
            })
        ));
        // Option 包装在这些输入下仍然返回 None（行为不变）。
        assert!(hohmann_transfer(mu, 0.0, 4.2164e7).is_none());
        assert!(radiation_absorbed_dose(10.0, 0.0, 1.0).is_none());
    }

    /// 球谐 checked 变体守门：天体内部（|r| < 1 m）显式报错而不是静默零矢量；
    /// 轨道高度处与原函数逐位一致。
    #[test]
    fn spherical_harmonics_checked_exposes_interior_silence() {
        use mps_formula::FormulaError;
        use mps_formula::celestial_data::{CelestialBodyId, get_celestial_body};
        use mps_formula::ffi::Vec3;
        use mps_formula::gravitational_models::{
            spherical_harmonics_acceleration, spherical_harmonics_acceleration_checked,
        };

        let earth = get_celestial_body(CelestialBodyId::Earth);

        // 轨道高度：与原函数逐位一致。
        let p = Vec3 {
            x: 7.0e6,
            y: 0.0,
            z: 0.0,
        };
        let a = spherical_harmonics_acceleration(p, earth, 2);
        let ac = spherical_harmonics_acceleration_checked(p, earth, 2).unwrap();
        assert_eq!(a.x, ac.x);
        assert_eq!(a.y, ac.y);
        assert_eq!(a.z, ac.z);

        // 天体内部：checked 报 OutOfDomain，原函数仍静默返回零（行为不变）。
        let inside = Vec3 {
            x: 0.1,
            y: 0.0,
            z: 0.0,
        };
        match spherical_harmonics_acceleration_checked(inside, earth, 2) {
            Err(FormulaError::OutOfDomain { parameter, .. }) => {
                assert!(parameter.starts_with("position"), "{parameter}");
            }
            other => panic!("expected OutOfDomain for interior point, got {other:?}"),
        }
        assert_eq!(spherical_harmonics_acceleration(inside, earth, 2).x, 0.0);
        assert_eq!(spherical_harmonics_acceleration(inside, earth, 2).y, 0.0);
        assert_eq!(spherical_harmonics_acceleration(inside, earth, 2).z, 0.0);
    }
}
