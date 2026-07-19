//! Spaceflight engineering — orbital mechanics, attitude control, thermal, propulsion, and environment formulas.
//!
//! Pure computation only — no access to `WorldHandle`, `RigidBody`, or Rapier state.

use std::f64::consts::{PI, TAU};

use crate::ffi::{
    AirlockDepressurization, AtomicOxygenErosion, BangOffBangProfile, BatteryEquivalentCircuit,
    Bool, ChemicalReactionRate, CmgExchange, CmgRobustInverse, Co2MassBalance,
    CollisionProbability, ContactForceModel, CwDerivative, CwState, DhTransform,
    FlexibleModeDerivative, FluidLoopHeatTransfer, FriisLink, GnssObservation,
    HallThrusterPerformance, HohmannTransfer, LeastSquaresAttitude, ManipulatorDynamics,
    MassProperties, OrbitalElements, Quat, QuaternionDerivative, RadarMeasurement, RadiatorPower,
    RigidBodyEulerDerivative, ScalarKalman, Sgp4SecularRates,
    SloshPendulumDerivative, SolarPanelPower, StateVector, ThermalBalance, VariationalState, Vec3,
    vec3_finite, vec3_to_rapier, vec3_from_rapier,
};
// local helper

const EPS: f64 = 1.0e-12;
const SIGMA: f64 = 5.670_374_419e-8;
const SPEED_OF_LIGHT: f64 = 299_792_458.0;


fn finite(values: &[f64]) -> bool {
    values.iter().all(|v| v.is_finite())
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    }
}

fn clamp_unit(value: f64) -> f64 {
    value.clamp(-1.0, 1.0)
}

// ---------------------------------------------------------------------------
// Orbital mechanics
// ---------------------------------------------------------------------------

pub fn kepler_period(mu: f64, semi_major_axis: f64) -> Option<f64> {
    if !finite(&[mu, semi_major_axis]) || mu <= 0.0 || semi_major_axis <= 0.0 { return None; }
    Some(TAU * (semi_major_axis.powi(3) / mu).sqrt())
}

pub fn kepler_semi_major_axis(mu: f64, period: f64) -> Option<f64> {
    if !finite(&[mu, period]) || mu <= 0.0 || period <= 0.0 { return None; }
    Some((mu * (period / TAU).powi(2)).cbrt())
}

pub fn elements_to_state(elements: OrbitalElements, mu: f64) -> Option<StateVector> {
    if !finite(&[
        elements.semi_major_axis, elements.eccentricity, elements.inclination,
        elements.raan, elements.argument_of_periapsis, elements.true_anomaly, mu,
    ]) || mu <= 0.0 || elements.semi_major_axis <= 0.0 || elements.eccentricity < 0.0 || elements.eccentricity >= 1.0
    { return None; }

    let a = elements.semi_major_axis;
    let e = elements.eccentricity;
    let i = elements.inclination;
    let raan = elements.raan;
    let argp = elements.argument_of_periapsis;
    let nu = elements.true_anomaly;
    let p = a * (1.0 - e * e);
    if p <= 0.0 { return None; }

    let r = p / (1.0 + e * nu.cos());
    let r_pf = vec3_to_rapier(Vec3 { x: r * nu.cos(), y: r * nu.sin(), z: 0.0 });
    let v_pf = vec3_to_rapier(Vec3 {
        x: -(mu / p).sqrt() * nu.sin(),
        y: (mu / p).sqrt() * (e + nu.cos()),
        z: 0.0,
    });

    let (so, co) = raan.sin_cos();
    let (si, ci) = i.sin_cos();
    let (sw, cw) = argp.sin_cos();
    let rotate = |v: rapier3d::prelude::Vector| -> rapier3d::prelude::Vector {
        rapier3d::prelude::Vector::new(
            (co * cw - so * sw * ci) * v.x + (-co * sw - so * cw * ci) * v.y,
            (so * cw + co * sw * ci) * v.x + (-so * sw + co * cw * ci) * v.y,
            (sw * si) * v.x + (cw * si) * v.y,
        )
    };

    Some(StateVector {
        position: vec3_from_rapier(rotate(r_pf)),
        velocity: vec3_from_rapier(rotate(v_pf)),
    })
}

pub fn state_to_elements(state: StateVector, mu: f64) -> Option<OrbitalElements> {
    if !vec3_finite(state.position) || !vec3_finite(state.velocity) || !mu.is_finite() || mu <= 0.0 { return None; }

    let r_vec = vec3_to_rapier(state.position);
    let v_vec = vec3_to_rapier(state.velocity);
    let r = r_vec.length();
    let v2 = v_vec.length_squared();
    if r <= EPS { return None; }

    let h_vec = r_vec.cross(v_vec);
    let h = h_vec.length();
    if h <= EPS { return None; }
    let n_vec = rapier3d::prelude::Vector::Z.cross(h_vec);
    let n = n_vec.length();
    let e_vec = v_vec.cross(h_vec) / mu - r_vec / r;
    let e = e_vec.length();
    let energy = 0.5 * v2 - mu / r;
    if energy.abs() <= EPS { return None; }

    let a = -mu / (2.0 * energy);
    let inclination = clamp_unit(h_vec.z / h).acos();
    let raan = if n > EPS { n_vec.y.atan2(n_vec.x).rem_euclid(TAU) } else { 0.0 };
    let argument_of_periapsis = if n > EPS && e > EPS {
        let mut value = clamp_unit(n_vec.dot(e_vec) / (n * e)).acos();
        if e_vec.z < 0.0 { value = TAU - value; }
        value
    } else { 0.0 };
    let true_anomaly = if e > EPS {
        let mut value = clamp_unit(e_vec.dot(r_vec) / (e * r)).acos();
        if r_vec.dot(v_vec) < 0.0 { value = TAU - value; }
        value
    } else if n > EPS {
        let mut value = clamp_unit(n_vec.dot(r_vec) / (n * r)).acos();
        if r_vec.z < 0.0 { value = TAU - value; }
        value
    } else { r_vec.y.atan2(r_vec.x).rem_euclid(TAU) };

    Some(OrbitalElements {
        semi_major_axis: a, eccentricity: e, inclination, raan,
        argument_of_periapsis, true_anomaly,
    })
}

pub fn j2_acceleration(position: Vec3, mu: f64, equatorial_radius: f64, j2: f64) -> Option<Vec3> {
    if !vec3_finite(position) || !finite(&[mu, equatorial_radius, j2]) || mu <= 0.0 || equatorial_radius <= 0.0 { return None; }
    let r = vec3_to_rapier(position);
    let radius = r.length();
    if radius <= EPS { return None; }
    let z2_r2 = (r.z * r.z) / (radius * radius);
    let factor = 1.5 * j2 * mu * equatorial_radius * equatorial_radius / radius.powi(5);
    Some(Vec3 {
        x: factor * r.x * (5.0 * z2_r2 - 1.0),
        y: factor * r.y * (5.0 * z2_r2 - 1.0),
        z: factor * r.z * (5.0 * z2_r2 - 3.0),
    })
}

// ---------------------------------------------------------------------------
// Attitude, kinematics, control
// ---------------------------------------------------------------------------

pub fn quaternion_derivative(attitude: Quat, angular_velocity: Vec3) -> Option<QuaternionDerivative> {
    if !finite(&[attitude.i, attitude.j, attitude.k, attitude.w]) || !vec3_finite(angular_velocity) { return None; }
    let wx = angular_velocity.x; let wy = angular_velocity.y; let wz = angular_velocity.z;
    Some(QuaternionDerivative {
        i_dot: 0.5 * (attitude.w * wx + attitude.j * wz - attitude.k * wy),
        j_dot: 0.5 * (attitude.w * wy + attitude.k * wx - attitude.i * wz),
        k_dot: 0.5 * (attitude.w * wz + attitude.i * wy - attitude.j * wx),
        w_dot: -0.5 * (attitude.i * wx + attitude.j * wy + attitude.k * wz),
    })
}

pub fn rigid_body_euler_derivative(inertia_diag: Vec3, angular_velocity: Vec3, torque: Vec3) -> Option<RigidBodyEulerDerivative> {
    if !vec3_finite(inertia_diag) || !vec3_finite(angular_velocity) || !vec3_finite(torque)
        || inertia_diag.x <= 0.0 || inertia_diag.y <= 0.0 || inertia_diag.z <= 0.0 { return None; }
    let omega = vec3_to_rapier(angular_velocity);
    let h = rapier3d::prelude::Vector::new(
        inertia_diag.x * omega.x, inertia_diag.y * omega.y, inertia_diag.z * omega.z,
    );
    Some(RigidBodyEulerDerivative {
        angular_acceleration: Vec3 {
            x: (torque.x - (omega.y * h.z - omega.z * h.y)) / inertia_diag.x,
            y: (torque.y - (omega.z * h.x - omega.x * h.z)) / inertia_diag.y,
            z: (torque.z - (omega.x * h.y - omega.y * h.x)) / inertia_diag.z,
        },
    })
}

pub fn cmg_exchange(gimbal_axis: Vec3, wheel_momentum: Vec3, gimbal_rate: f64) -> Option<CmgExchange> {
    if !vec3_finite(gimbal_axis) || !vec3_finite(wheel_momentum) || !gimbal_rate.is_finite() { return None; }
    let Some(axis) = vec3_to_rapier(gimbal_axis).try_normalize() else { return None; };
    let h_dot = (axis * gimbal_rate).cross(vec3_to_rapier(wheel_momentum));
    Some(CmgExchange {
        body_torque: vec3_from_rapier(-h_dot),
        wheel_momentum_dot: vec3_from_rapier(h_dot),
    })
}

pub fn cw_derivative(state: CwState, mean_motion: f64) -> Option<CwDerivative> {
    if !vec3_finite(state.position) || !vec3_finite(state.velocity) || !mean_motion.is_finite() { return None; }
    let n = mean_motion;
    let r = state.position; let v = state.velocity;
    Some(CwDerivative {
        velocity: v,
        acceleration: Vec3 {
            x: 3.0 * n * n * r.x + 2.0 * n * v.y,
            y: -2.0 * n * v.x,
            z: -n * n * r.z,
        },
    })
}

pub fn lambert_time_elliptic(mu: f64, semi_major_axis: f64, alpha: f64, beta: f64, revolutions: u32) -> Option<f64> {
    if !finite(&[mu, semi_major_axis, alpha, beta]) || mu <= 0.0 || semi_major_axis <= 0.0 { return None; }
    let m = revolutions as f64;
    Some((semi_major_axis.powi(3) / mu).sqrt() * ((alpha - alpha.sin()) - (beta - beta.sin()) + TAU * m))
}

pub fn dh_transform(theta: f64, d: f64, a: f64, alpha: f64) -> Option<DhTransform> {
    if !finite(&[theta, d, a, alpha]) { return None; }
    let (st, ct) = theta.sin_cos(); let (sa, ca) = alpha.sin_cos();
    Some(DhTransform {
        m00: ct, m01: -st * ca, m02: st * sa, m03: a * ct,
        m10: st, m11: ct * ca, m12: -ct * sa, m13: a * st,
        m20: 0.0, m21: sa, m22: ca, m23: d,
        m30: 0.0, m31: 0.0, m32: 0.0, m33: 1.0,
    })
}

pub fn arm_first_joint_inverse(wrist_x: f64, wrist_y: f64) -> Option<f64> {
    if !finite(&[wrist_x, wrist_y]) || (wrist_x.abs() <= EPS && wrist_y.abs() <= EPS) { return None; }
    Some(wrist_y.atan2(wrist_x))
}

pub fn arm_third_joint_angle(planar_radius: f64, vertical_offset: f64, link2: f64, link3: f64, elbow_up: bool) -> Option<f64> {
    if !finite(&[planar_radius, vertical_offset, link2, link3]) || link2 <= 0.0 || link3 <= 0.0 { return None; }
    let c3 = (planar_radius * planar_radius + vertical_offset * vertical_offset - link2 * link2 - link3 * link3) / (2.0 * link2 * link3);
    if !(-1.0..=1.0).contains(&c3) { return None; }
    let s3 = (1.0 - c3 * c3).sqrt() * if elbow_up { 1.0 } else { -1.0 };
    Some(s3.atan2(c3))
}

pub fn manipulator_dynamics_diag(mass_matrix_diag: Vec3, joint_acceleration: Vec3, coriolis: Vec3, gravity: Vec3) -> Option<ManipulatorDynamics> {
    if !vec3_finite(mass_matrix_diag) || !vec3_finite(joint_acceleration) || !vec3_finite(coriolis) || !vec3_finite(gravity) { return None; }
    Some(ManipulatorDynamics {
        torque: Vec3 {
            x: mass_matrix_diag.x * joint_acceleration.x + coriolis.x + gravity.x,
            y: mass_matrix_diag.y * joint_acceleration.y + coriolis.y + gravity.y,
            z: mass_matrix_diag.z * joint_acceleration.z + coriolis.z + gravity.z,
        },
    })
}

// ---------------------------------------------------------------------------
// Power, thermal, environment
// ---------------------------------------------------------------------------

pub fn solar_panel_power(solar_flux: f64, area: f64, efficiency: f64, incidence_angle: f64, degradation: f64) -> Option<SolarPanelPower> {
    if !finite(&[solar_flux, area, efficiency, incidence_angle, degradation]) || solar_flux < 0.0 || area < 0.0 || efficiency < 0.0 || degradation < 0.0 { return None; }
    let incident = solar_flux * area * incidence_angle.cos().max(0.0);
    Some(SolarPanelPower { incident_power: incident, electrical_power: incident * efficiency * degradation })
}

pub fn thermal_balance(absorbed_power: f64, internal_power: f64, emitted_area: f64, emissivity: f64) -> Option<ThermalBalance> {
    if !finite(&[absorbed_power, internal_power, emitted_area, emissivity]) || emitted_area <= 0.0 || emissivity <= 0.0 { return None; }
    let net = absorbed_power + internal_power;
    let equilibrium_temperature = if net > 0.0 { (net / (emissivity * SIGMA * emitted_area)).powf(0.25) } else { 0.0 };
    Some(ThermalBalance { net_power: net, equilibrium_temperature })
}

pub fn co2_mass_balance(current_mass: f64, generation_rate: f64, removal_rate: f64, leakage_rate: f64, volume: f64, dt: f64) -> Option<Co2MassBalance> {
    if !finite(&[current_mass, generation_rate, removal_rate, leakage_rate, volume, dt]) || volume <= 0.0 || dt < 0.0 { return None; }
    let mass_rate = generation_rate - removal_rate - leakage_rate;
    let next_mass = (current_mass + mass_rate * dt).max(0.0);
    Some(Co2MassBalance { mass_rate, next_mass, concentration_rate: mass_rate / volume })
}

pub fn friis_link(transmit_power: f64, transmit_gain: f64, receive_gain: f64, wavelength: f64, range: f64, system_loss: f64) -> Option<FriisLink> {
    if !finite(&[transmit_power, transmit_gain, receive_gain, wavelength, range, system_loss]) || transmit_power < 0.0 || transmit_gain < 0.0 || receive_gain < 0.0 || wavelength <= 0.0 || range <= 0.0 || system_loss <= 0.0 { return None; }
    let path_gain = (wavelength / (4.0 * PI * range)).powi(2);
    let path_loss = 1.0 / path_gain;
    Some(FriisLink { received_power: transmit_power * transmit_gain * receive_gain * path_gain / system_loss, path_loss })
}

pub fn friis_wavelength_from_frequency(frequency: f64) -> Option<f64> {
    if !frequency.is_finite() || frequency <= 0.0 { return None; }
    Some(SPEED_OF_LIGHT / frequency)
}

pub fn tsiolkovsky_delta_v(specific_impulse: f64, standard_gravity: f64, initial_mass: f64, final_mass: f64) -> Option<f64> {
    if !finite(&[specific_impulse, standard_gravity, initial_mass, final_mass]) || specific_impulse <= 0.0 || standard_gravity <= 0.0 || initial_mass <= 0.0 || final_mass <= 0.0 || initial_mass < final_mass { return None; }
    Some(specific_impulse * standard_gravity * (initial_mass / final_mass).ln())
}

pub fn hohmann_transfer(mu: f64, radius1: f64, radius2: f64) -> Option<HohmannTransfer> {
    if !finite(&[mu, radius1, radius2]) || mu <= 0.0 || radius1 <= 0.0 || radius2 <= 0.0 { return None; }
    let transfer_a = 0.5 * (radius1 + radius2);
    let circular1 = (mu / radius1).sqrt();
    let circular2 = (mu / radius2).sqrt();
    let transfer_periapsis = (mu * (2.0 / radius1 - 1.0 / transfer_a)).sqrt();
    let transfer_apoapsis = (mu * (2.0 / radius2 - 1.0 / transfer_a)).sqrt();
    let delta_v1 = transfer_periapsis - circular1;
    let delta_v2 = circular2 - transfer_apoapsis;
    Some(HohmannTransfer {
        delta_v1, delta_v2,
        total_delta_v: delta_v1.abs() + delta_v2.abs(),
        transfer_time: PI * (transfer_a.powi(3) / mu).sqrt(),
    })
}

pub fn atmospheric_density_scale_height(reference_density: f64, altitude: f64, reference_altitude: f64, scale_height: f64) -> Option<f64> {
    if !finite(&[reference_density, altitude, reference_altitude, scale_height]) || reference_density < 0.0 || scale_height <= 0.0 { return None; }
    Some(reference_density * (-(altitude - reference_altitude) / scale_height).exp())
}

pub fn atmospheric_drag_acceleration(velocity: Vec3, atmosphere_velocity: Vec3, density: f64, drag_coefficient: f64, area: f64, mass: f64) -> Option<Vec3> {
    if !vec3_finite(velocity) || !vec3_finite(atmosphere_velocity) || !finite(&[density, drag_coefficient, area, mass]) || density < 0.0 || drag_coefficient < 0.0 || area < 0.0 || mass <= 0.0 { return None; }
    let rel = vec3_to_rapier(velocity) - vec3_to_rapier(atmosphere_velocity);
    let speed = rel.length();
    let acc = if speed > EPS { -rel * (0.5 * density * speed * drag_coefficient * area / mass) } else { rapier3d::prelude::Vector::ZERO };
    Some(vec3_from_rapier(acc))
}

pub fn triad_attitude(body_primary: Vec3, body_secondary: Vec3, reference_primary: Vec3, reference_secondary: Vec3) -> Option<Quat> {
    let make_basis = |a: Vec3, b: Vec3| -> Option<(rapier3d::prelude::Vector, rapier3d::prelude::Vector, rapier3d::prelude::Vector)> {
        let t1 = vec3_to_rapier(a).try_normalize()?;
        let t2 = t1.cross(vec3_to_rapier(b)).try_normalize()?;
        let t3 = t1.cross(t2);
        Some((t1, t2, t3))
    };
    let (bt1, bt2, bt3) = make_basis(body_primary, body_secondary)?;
    let (rt1, rt2, rt3) = make_basis(reference_primary, reference_secondary)?;
    let m00 = bt1.x * rt1.x + bt2.x * rt2.x + bt3.x * rt3.x;
    let m01 = bt1.x * rt1.y + bt2.x * rt2.y + bt3.x * rt3.y;
    let m02 = bt1.x * rt1.z + bt2.x * rt2.z + bt3.x * rt3.z;
    let m10 = bt1.y * rt1.x + bt2.y * rt2.x + bt3.y * rt3.x;
    let m11 = bt1.y * rt1.y + bt2.y * rt2.y + bt3.y * rt3.y;
    let m12 = bt1.y * rt1.z + bt2.y * rt2.z + bt3.y * rt3.z;
    let m20 = bt1.z * rt1.x + bt2.z * rt2.x + bt3.z * rt3.x;
    let m21 = bt1.z * rt1.y + bt2.z * rt2.y + bt3.z * rt3.y;
    let m22 = bt1.z * rt1.z + bt2.z * rt2.z + bt3.z * rt3.z;
    let trace = m00 + m11 + m22;
    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        Quat { w: 0.25 * s, i: (m21 - m12) / s, j: (m02 - m20) / s, k: (m10 - m01) / s }
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        Quat { w: (m21 - m12) / s, i: 0.25 * s, j: (m01 + m10) / s, k: (m02 + m20) / s }
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        Quat { w: (m02 - m20) / s, i: (m01 + m10) / s, j: 0.25 * s, k: (m12 + m21) / s }
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        Quat { w: (m10 - m01) / s, i: (m02 + m20) / s, j: (m12 + m21) / s, k: 0.25 * s }
    };
    Some(q)
}

pub fn ekf_predict_scalar(state: f64, covariance: f64, nonlinear_delta: f64, jacobian: f64, process_noise: f64) -> Option<ScalarKalman> {
    if !finite(&[state, covariance, nonlinear_delta, jacobian, process_noise]) || covariance < 0.0 || process_noise < 0.0 { return None; }
    Some(ScalarKalman { value: state + nonlinear_delta, covariance: jacobian * covariance * jacobian + process_noise })
}

pub fn ekf_gain_scalar(covariance: f64, measurement_jacobian: f64, measurement_noise: f64) -> Option<f64> {
    if !finite(&[covariance, measurement_jacobian, measurement_noise]) || covariance < 0.0 || measurement_noise < 0.0 { return None; }
    let innovation_covariance = measurement_jacobian * covariance * measurement_jacobian + measurement_noise;
    if innovation_covariance <= EPS { return None; }
    Some(covariance * measurement_jacobian / innovation_covariance)
}

pub fn ekf_update_scalar(predicted_state: f64, predicted_covariance: f64, measurement: f64, predicted_measurement: f64, kalman_gain: f64, measurement_jacobian: f64) -> Option<ScalarKalman> {
    if !finite(&[predicted_state, predicted_covariance, measurement, predicted_measurement, kalman_gain, measurement_jacobian]) || predicted_covariance < 0.0 { return None; }
    Some(ScalarKalman { value: predicted_state + kalman_gain * (measurement - predicted_measurement), covariance: (1.0 - kalman_gain * measurement_jacobian) * predicted_covariance })
}

pub fn least_squares_attitude_two_vector(body_primary: Vec3, body_secondary: Vec3, reference_primary: Vec3, reference_secondary: Vec3) -> Option<LeastSquaresAttitude> {
    let quat = triad_attitude(body_primary, body_secondary, reference_primary, reference_secondary)?;
    Some(LeastSquaresAttitude { attitude: quat, rms_error: 0.0 })
}

pub fn gnss_pseudorange(receiver: Vec3, satellite: Vec3, receiver_clock_bias: f64, satellite_clock_bias: f64, ionosphere_delay: f64, troposphere_delay: f64) -> Option<GnssObservation> {
    if !vec3_finite(receiver) || !vec3_finite(satellite) || !finite(&[receiver_clock_bias, satellite_clock_bias, ionosphere_delay, troposphere_delay]) { return None; }
    let range = (vec3_to_rapier(satellite) - vec3_to_rapier(receiver)).length();
    Some(GnssObservation {
        value: range + SPEED_OF_LIGHT * (receiver_clock_bias - satellite_clock_bias) + ionosphere_delay + troposphere_delay,
        geometric_range: range,
    })
}

pub fn gnss_double_difference_carrier_phase(range_rover_sat_a: f64, range_rover_sat_b: f64, range_base_sat_a: f64, range_base_sat_b: f64, wavelength: f64, ambiguity: f64) -> Option<f64> {
    if !finite(&[range_rover_sat_a, range_rover_sat_b, range_base_sat_a, range_base_sat_b, wavelength, ambiguity]) || wavelength <= 0.0 { return None; }
    Some(((range_rover_sat_a - range_rover_sat_b) - (range_base_sat_a - range_base_sat_b)) / wavelength + ambiguity)
}

pub fn structural_natural_frequency(stiffness: f64, mass: f64, mode_factor: f64) -> Option<f64> {
    if !finite(&[stiffness, mass, mode_factor]) || stiffness <= 0.0 || mass <= 0.0 { return None; }
    Some(mode_factor * (stiffness / mass).sqrt() / TAU)
}

pub fn contact_force_hunt_crossley(penetration: f64, penetration_rate: f64, stiffness: f64, damping: f64, exponent: f64) -> Option<ContactForceModel> {
    if !finite(&[penetration, penetration_rate, stiffness, damping, exponent]) || stiffness < 0.0 || damping < 0.0 || exponent <= 0.0 { return None; }
    let depth = penetration.max(0.0);
    let normal = stiffness * depth.powf(exponent);
    let damping_force = damping * depth.powf(exponent) * penetration_rate.max(0.0);
    Some(ContactForceModel { normal_force: normal, damping_force, total_force: normal + damping_force })
}

pub fn radiation_absorbed_dose(energy_joules: f64, mass_kg: f64, quality_factor: f64) -> Option<f64> {
    if !finite(&[energy_joules, mass_kg, quality_factor]) || mass_kg <= 0.0 || quality_factor < 0.0 { return None; }
    Some(energy_joules / mass_kg * quality_factor)
}

pub fn semi_major_axis_decay_rate(semi_major_axis: f64, density: f64, drag_coefficient: f64, area: f64, mass: f64, mu: f64) -> Option<f64> {
    if !finite(&[semi_major_axis, density, drag_coefficient, area, mass, mu]) || semi_major_axis <= 0.0 || density < 0.0 || drag_coefficient < 0.0 || area < 0.0 || mass <= 0.0 || mu <= 0.0 { return None; }
    let v = (mu / semi_major_axis).sqrt();
    Some(-density * drag_coefficient * area / mass * semi_major_axis * v)
}

pub fn heat_pipe_thermal_resistance(evaporator_resistance: f64, vapor_resistance: f64, condenser_resistance: f64, wick_resistance: f64) -> Option<f64> {
    if !finite(&[evaporator_resistance, vapor_resistance, condenser_resistance, wick_resistance]) { return None; }
    Some(evaporator_resistance + vapor_resistance + condenser_resistance + wick_resistance)
}

pub fn battery_equivalent_circuit(open_circuit_voltage: f64, current: f64, ohmic_resistance: f64, rc_voltage: f64, rc_resistance: f64, rc_capacitance: f64, capacity_coulombs: f64) -> Option<BatteryEquivalentCircuit> {
    if !finite(&[open_circuit_voltage, current, ohmic_resistance, rc_voltage, rc_resistance, rc_capacitance, capacity_coulombs]) || ohmic_resistance < 0.0 || rc_resistance <= 0.0 || rc_capacitance <= 0.0 || capacity_coulombs <= 0.0 { return None; }
    Some(BatteryEquivalentCircuit {
        terminal_voltage: open_circuit_voltage - current * ohmic_resistance - rc_voltage,
        rc_voltage_dot: -rc_voltage / (rc_resistance * rc_capacitance) + current / rc_capacitance,
        state_of_charge_dot: -current / capacity_coulombs,
    })
}

pub fn hall_thruster_performance(mass_flow_rate: f64, exhaust_velocity: f64, input_power: f64, standard_gravity: f64) -> Option<HallThrusterPerformance> {
    if !finite(&[mass_flow_rate, exhaust_velocity, input_power, standard_gravity]) || mass_flow_rate < 0.0 || exhaust_velocity < 0.0 || input_power <= 0.0 || standard_gravity <= 0.0 { return None; }
    let thrust = mass_flow_rate * exhaust_velocity;
    Some(HallThrusterPerformance { thrust, specific_impulse: exhaust_velocity / standard_gravity, efficiency: 0.5 * mass_flow_rate * exhaust_velocity * exhaust_velocity / input_power })
}

pub fn artificial_potential_guidance(position: Vec3, target: Vec3, obstacle: Vec3, attractive_gain: f64, repulsive_gain: f64, influence_radius: f64) -> Option<Vec3> {
    if !vec3_finite(position) || !vec3_finite(target) || !vec3_finite(obstacle) || !finite(&[attractive_gain, repulsive_gain, influence_radius]) || influence_radius <= 0.0 { return None; }
    let p = vec3_to_rapier(position);
    let attractive = (vec3_to_rapier(target) - p) * attractive_gain;
    let away = p - vec3_to_rapier(obstacle);
    let d = away.length();
    let repulsive = if d > EPS && d < influence_radius { away / d * repulsive_gain * (1.0 / d - 1.0 / influence_radius) / (d * d) } else { rapier3d::prelude::Vector::ZERO };
    Some(vec3_from_rapier(attractive + repulsive))
}

pub fn debris_collision_probability(miss_distance: f64, combined_radius: f64, sigma_radial: f64, sigma_intrack: f64) -> Option<CollisionProbability> {
    if !finite(&[miss_distance, combined_radius, sigma_radial, sigma_intrack]) || combined_radius < 0.0 || sigma_radial <= 0.0 || sigma_intrack <= 0.0 { return None; }
    let sigma = (sigma_radial * sigma_intrack).sqrt();
    let probability = (combined_radius * combined_radius / (2.0 * sigma_radial * sigma_intrack)) * (-0.5 * miss_distance * miss_distance / (sigma * sigma)).exp();
    Some(CollisionProbability { probability: probability.clamp(0.0, 1.0), combined_sigma: sigma })
}

pub fn atomic_oxygen_erosion(fluence: f64, erosion_yield: f64, area: f64, density: f64) -> Option<AtomicOxygenErosion> {
    if !finite(&[fluence, erosion_yield, area, density]) || fluence < 0.0 || erosion_yield < 0.0 || area < 0.0 || density < 0.0 { return None; }
    let volume_loss = fluence * erosion_yield * area;
    Some(AtomicOxygenErosion { volume_loss, mass_loss: volume_loss * density })
}

pub fn flexible_mode_derivative(displacement: f64, velocity: f64, natural_frequency: f64, damping_ratio: f64, modal_force: f64, modal_mass: f64) -> Option<FlexibleModeDerivative> {
    if !finite(&[displacement, velocity, natural_frequency, damping_ratio, modal_force, modal_mass]) || natural_frequency < 0.0 || damping_ratio < 0.0 || modal_mass <= 0.0 { return None; }
    Some(FlexibleModeDerivative { displacement_dot: velocity, velocity_dot: modal_force / modal_mass - 2.0 * damping_ratio * natural_frequency * velocity - natural_frequency * natural_frequency * displacement })
}

pub fn slosh_pendulum_derivative(angle: f64, angular_rate: f64, length: f64, damping: f64, lateral_acceleration: f64, gravity: f64) -> Option<SloshPendulumDerivative> {
    if !finite(&[angle, angular_rate, length, damping, lateral_acceleration, gravity]) || length <= 0.0 { return None; }
    Some(SloshPendulumDerivative { angle_dot: angular_rate, angular_rate_dot: -(gravity / length) * angle.sin() - damping * angular_rate - lateral_acceleration / length })
}

pub fn variational_two_body(position: Vec3, velocity: Vec3, mu: f64) -> Option<VariationalState> {
    if !vec3_finite(position) || !vec3_finite(velocity) || !mu.is_finite() || mu <= 0.0 { return None; }
    let r = vec3_to_rapier(position);
    let rn = r.length();
    if rn <= EPS { return None; }
    Some(VariationalState { position_dot: velocity, velocity_dot: vec3_from_rapier(-r * (mu / (rn * rn.sqrt()))) })
}

pub fn single_phase_loop_heat_transfer(mass_flow_rate: f64, specific_heat: f64, inlet_temperature: f64, heat_input: f64) -> Option<FluidLoopHeatTransfer> {
    if !finite(&[mass_flow_rate, specific_heat, inlet_temperature, heat_input]) || mass_flow_rate <= 0.0 || specific_heat <= 0.0 { return None; }
    Some(FluidLoopHeatTransfer { heat_rate: heat_input, outlet_temperature: inlet_temperature + heat_input / (mass_flow_rate * specific_heat) })
}

pub fn radar_range_rate(radar_position: Vec3, target_position: Vec3, radar_velocity: Vec3, target_velocity: Vec3) -> Option<RadarMeasurement> {
    if !vec3_finite(radar_position) || !vec3_finite(target_position) || !vec3_finite(radar_velocity) || !vec3_finite(target_velocity) { return None; }
    let line = vec3_to_rapier(target_position) - vec3_to_rapier(radar_position);
    let range = line.length();
    if range <= EPS { return None; }
    let rel_v = vec3_to_rapier(target_velocity) - vec3_to_rapier(radar_velocity);
    Some(RadarMeasurement { range, range_rate: rel_v.dot(line / range) })
}

pub fn mass_properties_two_body(mass1: f64, position1: Vec3, inertia1_diag: Vec3, mass2: f64, position2: Vec3, inertia2_diag: Vec3) -> Option<MassProperties> {
    if !finite(&[mass1, mass2]) || mass1 < 0.0 || mass2 < 0.0 || mass1 + mass2 <= 0.0 || !vec3_finite(position1) || !vec3_finite(position2) || !vec3_finite(inertia1_diag) || !vec3_finite(inertia2_diag) { return None; }
    let p1 = vec3_to_rapier(position1); let p2 = vec3_to_rapier(position2);
    let total = mass1 + mass2;
    let com = (p1 * mass1 + p2 * mass2) / total;
    let parallel = |m: f64, p: rapier3d::prelude::Vector, i: Vec3| -> Vec3 { let d = p - com; Vec3 { x: i.x + m * (d.y * d.y + d.z * d.z), y: i.y + m * (d.x * d.x + d.z * d.z), z: i.z + m * (d.x * d.x + d.y * d.y) } };
    let i1 = parallel(mass1, p1, inertia1_diag); let i2 = parallel(mass2, p2, inertia2_diag);
    Some(MassProperties { center_of_mass: vec3_from_rapier(com), inertia_diag: Vec3 { x: i1.x + i2.x, y: i1.y + i2.y, z: i1.z + i2.z } })
}

pub fn docking_buffer_energy(relative_speed: f64, reduced_mass: f64, stroke: f64, efficiency: f64) -> Option<f64> {
    if !finite(&[relative_speed, reduced_mass, stroke, efficiency]) || reduced_mass < 0.0 || stroke <= 0.0 || efficiency <= 0.0 { return None; }
    Some(0.5 * reduced_mass * relative_speed * relative_speed / efficiency)
}

pub fn bang_off_bang_profile(angle: f64, max_acceleration: f64, max_rate: f64) -> Option<BangOffBangProfile> {
    if !finite(&[angle, max_acceleration, max_rate]) || max_acceleration <= 0.0 || max_rate <= 0.0 { return None; }
    let theta = angle.abs();
    let triangular_angle = max_rate * max_rate / max_acceleration;
    let (coast, total, switch_angle) = if theta <= triangular_angle {
        let t = (theta / max_acceleration).sqrt(); (0.0, 2.0 * t, 0.5 * theta)
    } else {
        let accel_time = max_rate / max_acceleration; let coast = (theta - triangular_angle) / max_rate;
        (coast, 2.0 * accel_time + coast, 0.5 * triangular_angle)
    };
    Some(BangOffBangProfile { coast_time: coast, total_time: total, switch_angle })
}

pub fn solar_radiation_pressure_acceleration(sun_direction: Vec3, solar_flux: f64, reflectivity: f64, area: f64, mass: f64) -> Option<Vec3> {
    if !vec3_finite(sun_direction) || !finite(&[solar_flux, reflectivity, area, mass]) || solar_flux < 0.0 || reflectivity < 0.0 || area < 0.0 || mass <= 0.0 { return None; }
    let dir = vec3_to_rapier(sun_direction).try_normalize()?;
    Some(vec3_from_rapier(dir * (solar_flux / SPEED_OF_LIGHT * reflectivity * area / mass)))
}

pub fn gravity_gradient_torque(position: Vec3, inertia_diag: Vec3, mu: f64) -> Option<Vec3> {
    if !vec3_finite(position) || !vec3_finite(inertia_diag) || !mu.is_finite() || mu <= 0.0 { return None; }
    let r = vec3_to_rapier(position); let rn = r.length();
    if rn <= EPS { return None; }
    let n = r / rn;
    let in_vec = rapier3d::prelude::Vector::new(inertia_diag.x * n.x, inertia_diag.y * n.y, inertia_diag.z * n.z);
    Some(vec3_from_rapier(n.cross(in_vec) * (3.0 * mu / (rn * rn.sqrt()))))
}

pub fn magnetic_torquer_dipole(commanded_torque: Vec3, magnetic_field: Vec3, max_dipole: f64) -> Option<Vec3> {
    if !vec3_finite(commanded_torque) || !vec3_finite(magnetic_field) || !max_dipole.is_finite() || max_dipole < 0.0 { return None; }
    let b = vec3_to_rapier(magnetic_field); let b2 = b.length_squared();
    if b2 <= EPS { return None; }
    let mut m = b.cross(vec3_to_rapier(commanded_torque)) / b2;
    let mn = m.length();
    if mn > max_dipole && mn > EPS { m *= max_dipole / mn; }
    Some(vec3_from_rapier(m))
}

pub fn cmg_robust_pseudoinverse_diag(jacobian_diag: Vec3, desired_torque: Vec3, damping: f64) -> Option<CmgRobustInverse> {
    if !vec3_finite(jacobian_diag) || !vec3_finite(desired_torque) || !damping.is_finite() || damping < 0.0 { return None; }
    let solve = |j: f64, t: f64| j * t / (j * j + damping * damping);
    Some(CmgRobustInverse { gimbal_rates: Vec3 { x: solve(jacobian_diag.x, desired_torque.x), y: solve(jacobian_diag.y, desired_torque.y), z: solve(jacobian_diag.z, desired_torque.z) }, damping })
}

pub fn sgp4_j2_secular_rates(semi_major_axis: f64, eccentricity: f64, inclination: f64, mean_motion: f64, equatorial_radius: f64, j2: f64) -> Option<Sgp4SecularRates> {
    if !finite(&[semi_major_axis, eccentricity, inclination, mean_motion, equatorial_radius, j2]) || semi_major_axis <= 0.0 || !(0.0..1.0).contains(&eccentricity) { return None; }
    let p = semi_major_axis * (1.0 - eccentricity * eccentricity);
    let factor = 1.5 * j2 * mean_motion * (equatorial_radius / p).powi(2);
    Some(Sgp4SecularRates { mean_motion_dot: 0.0, raan_dot: -factor * inclination.cos(), argument_of_perigee_dot: 0.5 * factor * (5.0 * inclination.cos().powi(2) - 1.0) })
}

pub fn docking_glideslope_command(range: f64, desired_slope: f64, closing_speed_limit: f64) -> Option<f64> {
    if !finite(&[range, desired_slope, closing_speed_limit]) || closing_speed_limit < 0.0 { return None; }
    Some((-desired_slope * range).clamp(-closing_speed_limit, closing_speed_limit))
}

pub fn sagnac_phase_rate(area: f64, angular_rate: f64, wavelength: f64) -> Option<f64> {
    if !finite(&[area, angular_rate, wavelength]) || wavelength <= 0.0 { return None; }
    Some(8.0 * PI * area * angular_rate / (wavelength * SPEED_OF_LIGHT))
}

pub fn solar_array_pd_torque(angle_error: f64, rate_error: f64, kp: f64, kd: f64) -> Option<f64> {
    if !finite(&[angle_error, rate_error, kp, kd]) { return None; }
    Some(kp * angle_error + kd * rate_error)
}

pub fn sabatier_methane_rate(co2_molar_rate: f64, h2_molar_rate: f64, conversion: f64) -> Option<ChemicalReactionRate> {
    if !finite(&[co2_molar_rate, h2_molar_rate, conversion]) || co2_molar_rate < 0.0 || h2_molar_rate < 0.0 || !(0.0..=1.0).contains(&conversion) { return None; }
    let methane = co2_molar_rate.min(h2_molar_rate / 4.0) * conversion;
    Some(ChemicalReactionRate { reactant_rate: methane, product_rate: methane })
}

pub fn spe_oxygen_rate(current: f64, cells: f64, faraday_efficiency: f64) -> Option<ChemicalReactionRate> {
    if !finite(&[current, cells, faraday_efficiency]) || current < 0.0 || cells <= 0.0 || !(0.0..=1.0).contains(&faraday_efficiency) { return None; }
    let faraday = 96_485.332_12;
    let oxygen = current * cells * faraday_efficiency / (4.0 * faraday);
    Some(ChemicalReactionRate { reactant_rate: current * cells / (2.0 * faraday), product_rate: oxygen })
}

pub fn radiator_power(area: f64, emissivity: f64, temperature: f64, sink_temperature: f64, absorbed_power: f64) -> Option<RadiatorPower> {
    if !finite(&[area, emissivity, temperature, sink_temperature, absorbed_power]) || area < 0.0 || emissivity < 0.0 || temperature < 0.0 || sink_temperature < 0.0 { return None; }
    let emitted = emissivity * SIGMA * area * (temperature.powi(4) - sink_temperature.powi(4)).max(0.0);
    Some(RadiatorPower { emitted_power: emitted, net_power: emitted - absorbed_power })
}

pub fn whipple_critical_projectile_diameter(bumper_thickness: f64, bumper_density: f64, projectile_density: f64, impact_velocity: f64, standoff: f64) -> Option<f64> {
    if !finite(&[bumper_thickness, bumper_density, projectile_density, impact_velocity, standoff]) || bumper_thickness <= 0.0 || bumper_density <= 0.0 || projectile_density <= 0.0 || impact_velocity <= 0.0 || standoff <= 0.0 { return None; }
    Some(bumper_thickness * (bumper_density / projectile_density).sqrt() * (standoff / bumper_thickness).powf(1.0 / 3.0) * (7_000.0 / impact_velocity).powf(2.0 / 3.0))
}

pub fn surface_charging_current_balance(photo_current: f64, secondary_current: f64, backscatter_current: f64, electron_current: f64, ion_current: f64) -> Option<f64> {
    if !finite(&[photo_current, secondary_current, backscatter_current, electron_current, ion_current]) { return None; }
    Some(photo_current + secondary_current + backscatter_current + ion_current - electron_current)
}

pub fn airlock_depressurization(pressure: f64, ambient_pressure: f64, volume: f64, conductance: f64, dt: f64) -> Option<AirlockDepressurization> {
    if !finite(&[pressure, ambient_pressure, volume, conductance, dt]) || volume <= 0.0 || conductance < 0.0 || dt < 0.0 { return None; }
    let rate = -conductance / volume * (pressure - ambient_pressure);
    Some(AirlockDepressurization { pressure: ambient_pressure + (pressure - ambient_pressure) * (-conductance * dt / volume).exp(), pressure_rate: rate })
}