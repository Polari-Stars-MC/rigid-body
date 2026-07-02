use std::f64::consts::{PI, TAU};

use rapier3d::prelude::Vector;

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    AirlockDepressurization, AtomicOxygenErosion, BangOffBangProfile, BatteryEquivalentCircuit,
    Bool, ChemicalReactionRate, CmgExchange, CmgRobustInverse, Co2MassBalance,
    CollisionProbability, ContactForceModel, CwDerivative, CwState, DhTransform,
    FlexibleModeDerivative, FluidLoopHeatTransfer, FriisLink, GnssObservation,
    HallThrusterPerformance, HohmannTransfer, LeastSquaresAttitude, ManipulatorDynamics,
    MassProperties, OrbitalElements, Quat, QuaternionDerivative, RadarMeasurement, RadiatorPower,
    RigidBodyEulerDerivative, RigidBodyHandleRaw, ScalarKalman, Sgp4SecularRates,
    SloshPendulumDerivative, SolarPanelPower, StateVector, ThermalBalance, VariationalState, Vec3,
    WorldHandle, unpack_rigid_body_handle, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

const EPS: f64 = 1.0e-12;
const SIGMA: f64 = 5.670_374_419e-8;
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

fn finite(values: &[f64]) -> bool {
    values.iter().all(|value| value.is_finite())
}

fn write_out<T: Copy>(out: *mut T, value: T) -> Bool {
    let Some(out) = (unsafe { out.as_mut() }) else {
        set_error(ERR_INVALID_ARGUMENT, "output pointer is null");
        return Bool::FALSE;
    };
    *out = value;
    clear_error();
    Bool::TRUE
}

fn write_optional_out<T: Copy>(out: *mut T, value: T) {
    if let Some(out) = unsafe { out.as_mut() } {
        *out = value;
    }
}

fn invalid_nan(message: &str) -> f64 {
    set_error(ERR_INVALID_ARGUMENT, message);
    f64::NAN
}

fn cross(a: Vector, b: Vector) -> Vector {
    Vector::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

fn clamp_unit(value: f64) -> f64 {
    value.clamp(-1.0, 1.0)
}

#[unsafe(no_mangle)]
pub extern "C" fn space_kepler_period(mu: f64, semi_major_axis: f64) -> f64 {
    if !finite(&[mu, semi_major_axis]) || mu <= 0.0 || semi_major_axis <= 0.0 {
        return invalid_nan("invalid Kepler period parameters");
    }
    clear_error();
    TAU * (semi_major_axis.powi(3) / mu).sqrt()
}

#[unsafe(no_mangle)]
pub extern "C" fn space_kepler_semi_major_axis(mu: f64, period: f64) -> f64 {
    if !finite(&[mu, period]) || mu <= 0.0 || period <= 0.0 {
        return invalid_nan("invalid Kepler semi-major-axis parameters");
    }
    clear_error();
    (mu * (period / TAU).powi(2)).cbrt()
}

#[unsafe(no_mangle)]
pub extern "C" fn space_elements_to_state(
    elements: OrbitalElements,
    mu: f64,
    out_state: *mut StateVector,
) -> Bool {
    if !finite(&[
        elements.semi_major_axis,
        elements.eccentricity,
        elements.inclination,
        elements.raan,
        elements.argument_of_periapsis,
        elements.true_anomaly,
        mu,
    ]) || mu <= 0.0
        || elements.semi_major_axis <= 0.0
        || elements.eccentricity < 0.0
        || elements.eccentricity >= 1.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid orbital elements");
        return Bool::FALSE;
    }

    let a = elements.semi_major_axis;
    let e = elements.eccentricity;
    let i = elements.inclination;
    let raan = elements.raan;
    let argp = elements.argument_of_periapsis;
    let nu = elements.true_anomaly;
    let p = a * (1.0 - e * e);
    if p <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "invalid orbital semi-latus rectum");
        return Bool::FALSE;
    }

    let r = p / (1.0 + e * nu.cos());
    let r_pf = Vector::new(r * nu.cos(), r * nu.sin(), 0.0);
    let v_pf = Vector::new(
        -(mu / p).sqrt() * nu.sin(),
        (mu / p).sqrt() * (e + nu.cos()),
        0.0,
    );

    let (so, co) = raan.sin_cos();
    let (si, ci) = i.sin_cos();
    let (sw, cw) = argp.sin_cos();
    let rotate = |v: Vector| -> Vector {
        Vector::new(
            (co * cw - so * sw * ci) * v.x + (-co * sw - so * cw * ci) * v.y,
            (so * cw + co * sw * ci) * v.x + (-so * sw + co * cw * ci) * v.y,
            (sw * si) * v.x + (cw * si) * v.y,
        )
    };

    write_out(
        out_state,
        StateVector {
            position: vec3_from_rapier(rotate(r_pf)),
            velocity: vec3_from_rapier(rotate(v_pf)),
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_state_to_elements(
    state: StateVector,
    mu: f64,
    out_elements: *mut OrbitalElements,
) -> Bool {
    if !vec3_finite(state.position) || !vec3_finite(state.velocity) || !mu.is_finite() || mu <= 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid state vector");
        return Bool::FALSE;
    }

    let r_vec = vec3_to_rapier(state.position);
    let v_vec = vec3_to_rapier(state.velocity);
    let r = r_vec.length();
    let v2 = v_vec.length_squared();
    if r <= EPS {
        set_error(ERR_INVALID_ARGUMENT, "position magnitude is zero");
        return Bool::FALSE;
    }

    let h_vec = cross(r_vec, v_vec);
    let h = h_vec.length();
    if h <= EPS {
        set_error(ERR_INVALID_ARGUMENT, "angular momentum magnitude is zero");
        return Bool::FALSE;
    }
    let n_vec = cross(Vector::Z, h_vec);
    let n = n_vec.length();
    let e_vec = cross(v_vec, h_vec) / mu - r_vec / r;
    let e = e_vec.length();
    let energy = 0.5 * v2 - mu / r;
    if energy.abs() <= EPS {
        set_error(ERR_INVALID_ARGUMENT, "parabolic orbit is unsupported");
        return Bool::FALSE;
    }

    let a = -mu / (2.0 * energy);
    let inclination = clamp_unit(h_vec.z / h).acos();
    let raan = if n > EPS {
        n_vec.y.atan2(n_vec.x).rem_euclid(TAU)
    } else {
        0.0
    };
    let argument_of_periapsis = if n > EPS && e > EPS {
        let mut value = clamp_unit(n_vec.dot(e_vec) / (n * e)).acos();
        if e_vec.z < 0.0 {
            value = TAU - value;
        }
        value
    } else {
        0.0
    };
    let true_anomaly = if e > EPS {
        let mut value = clamp_unit(e_vec.dot(r_vec) / (e * r)).acos();
        if r_vec.dot(v_vec) < 0.0 {
            value = TAU - value;
        }
        value
    } else if n > EPS {
        let mut value = clamp_unit(n_vec.dot(r_vec) / (n * r)).acos();
        if r_vec.z < 0.0 {
            value = TAU - value;
        }
        value
    } else {
        r_vec.y.atan2(r_vec.x).rem_euclid(TAU)
    };

    write_out(
        out_elements,
        OrbitalElements {
            semi_major_axis: a,
            eccentricity: e,
            inclination,
            raan,
            argument_of_periapsis,
            true_anomaly,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_j2_acceleration(
    position: Vec3,
    mu: f64,
    equatorial_radius: f64,
    j2: f64,
    out_acceleration: *mut Vec3,
) -> Bool {
    if !vec3_finite(position)
        || !finite(&[mu, equatorial_radius, j2])
        || mu <= 0.0
        || equatorial_radius <= 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid J2 parameters");
        return Bool::FALSE;
    }
    let r = vec3_to_rapier(position);
    let radius = r.length();
    if radius <= EPS {
        set_error(ERR_INVALID_ARGUMENT, "position magnitude is zero");
        return Bool::FALSE;
    }
    let z2_r2 = (r.z * r.z) / (radius * radius);
    let factor = 1.5 * j2 * mu * equatorial_radius * equatorial_radius / radius.powi(5);
    write_out(
        out_acceleration,
        vec3_from_rapier(Vector::new(
            factor * r.x * (5.0 * z2_r2 - 1.0),
            factor * r.y * (5.0 * z2_r2 - 1.0),
            factor * r.z * (5.0 * z2_r2 - 3.0),
        )),
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_apply_j2_force_to_body(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    mu: f64,
    equatorial_radius: f64,
    j2: f64,
    mass: f64,
    wake_up: Bool,
    out_acceleration: *mut Vec3,
) -> Bool {
    if !mass.is_finite() || mass <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "invalid J2 body mass");
        return Bool::FALSE;
    }
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(body) = world
        .inner
        .bodies
        .get_mut(unpack_rigid_body_handle(body_handle))
    else {
        set_error(ERR_NOT_FOUND, "body was not found");
        return Bool::FALSE;
    };
    let position = vec3_from_rapier(body.translation());
    let mut acceleration = Vec3::default();
    if space_j2_acceleration(position, mu, equatorial_radius, j2, &mut acceleration) == Bool::FALSE
    {
        return Bool::FALSE;
    }
    body.add_force(vec3_to_rapier(acceleration) * mass, wake_up.0 != 0);
    write_optional_out(out_acceleration, acceleration);
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn space_apply_j2_force_to_body_flag(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    mu: f64,
    equatorial_radius: f64,
    j2: f64,
    mass: f64,
    wake_up: Bool,
    out_acceleration: *mut Vec3,
) -> u8 {
    space_apply_j2_force_to_body(
        world,
        body_handle,
        mu,
        equatorial_radius,
        j2,
        mass,
        wake_up,
        out_acceleration,
    )
    .0
}

#[unsafe(no_mangle)]
pub extern "C" fn space_quaternion_derivative(
    attitude: Quat,
    angular_velocity: Vec3,
    out_derivative: *mut QuaternionDerivative,
) -> Bool {
    if !finite(&[attitude.i, attitude.j, attitude.k, attitude.w]) || !vec3_finite(angular_velocity)
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid quaternion kinematics parameters",
        );
        return Bool::FALSE;
    }
    let wx = angular_velocity.x;
    let wy = angular_velocity.y;
    let wz = angular_velocity.z;
    write_out(
        out_derivative,
        QuaternionDerivative {
            i_dot: 0.5 * (attitude.w * wx + attitude.j * wz - attitude.k * wy),
            j_dot: 0.5 * (attitude.w * wy + attitude.k * wx - attitude.i * wz),
            k_dot: 0.5 * (attitude.w * wz + attitude.i * wy - attitude.j * wx),
            w_dot: -0.5 * (attitude.i * wx + attitude.j * wy + attitude.k * wz),
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_rigid_body_euler_derivative(
    inertia_diag: Vec3,
    angular_velocity: Vec3,
    torque: Vec3,
    out_derivative: *mut RigidBodyEulerDerivative,
) -> Bool {
    if !vec3_finite(inertia_diag)
        || !vec3_finite(angular_velocity)
        || !vec3_finite(torque)
        || inertia_diag.x <= 0.0
        || inertia_diag.y <= 0.0
        || inertia_diag.z <= 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Euler rigid-body parameters");
        return Bool::FALSE;
    }
    let omega = vec3_to_rapier(angular_velocity);
    let h = Vector::new(
        inertia_diag.x * omega.x,
        inertia_diag.y * omega.y,
        inertia_diag.z * omega.z,
    );
    let accel = Vector::new(
        (torque.x - (omega.y * h.z - omega.z * h.y)) / inertia_diag.x,
        (torque.y - (omega.z * h.x - omega.x * h.z)) / inertia_diag.y,
        (torque.z - (omega.x * h.y - omega.y * h.x)) / inertia_diag.z,
    );
    write_out(
        out_derivative,
        RigidBodyEulerDerivative {
            angular_acceleration: vec3_from_rapier(accel),
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_cmg_exchange(
    gimbal_axis: Vec3,
    wheel_momentum: Vec3,
    gimbal_rate: f64,
    out_exchange: *mut CmgExchange,
) -> Bool {
    if !vec3_finite(gimbal_axis) || !vec3_finite(wheel_momentum) || !gimbal_rate.is_finite() {
        set_error(ERR_INVALID_ARGUMENT, "invalid CMG parameters");
        return Bool::FALSE;
    }
    let Some(axis) = vec3_to_rapier(gimbal_axis).try_normalize() else {
        set_error(ERR_INVALID_ARGUMENT, "CMG gimbal axis is zero");
        return Bool::FALSE;
    };
    let h_dot = cross(axis * gimbal_rate, vec3_to_rapier(wheel_momentum));
    write_out(
        out_exchange,
        CmgExchange {
            body_torque: vec3_from_rapier(-h_dot),
            wheel_momentum_dot: vec3_from_rapier(h_dot),
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_apply_cmg_torque_to_body(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    gimbal_axis: Vec3,
    wheel_momentum: Vec3,
    gimbal_rate: f64,
    wake_up: Bool,
    out_exchange: *mut CmgExchange,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(body) = world
        .inner
        .bodies
        .get_mut(unpack_rigid_body_handle(body_handle))
    else {
        set_error(ERR_NOT_FOUND, "body was not found");
        return Bool::FALSE;
    };
    let mut exchange = CmgExchange::default();
    if space_cmg_exchange(gimbal_axis, wheel_momentum, gimbal_rate, &mut exchange) == Bool::FALSE {
        return Bool::FALSE;
    }
    body.add_torque(vec3_to_rapier(exchange.body_torque), wake_up.0 != 0);
    write_optional_out(out_exchange, exchange);
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn space_apply_cmg_torque_to_body_flag(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    gimbal_axis: Vec3,
    wheel_momentum: Vec3,
    gimbal_rate: f64,
    wake_up: Bool,
    out_exchange: *mut CmgExchange,
) -> u8 {
    space_apply_cmg_torque_to_body(
        world,
        body_handle,
        gimbal_axis,
        wheel_momentum,
        gimbal_rate,
        wake_up,
        out_exchange,
    )
    .0
}

#[unsafe(no_mangle)]
pub extern "C" fn space_cw_derivative(
    state: CwState,
    mean_motion: f64,
    out_derivative: *mut CwDerivative,
) -> Bool {
    if !vec3_finite(state.position) || !vec3_finite(state.velocity) || !mean_motion.is_finite() {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid Clohessy-Wiltshire parameters",
        );
        return Bool::FALSE;
    }
    let n = mean_motion;
    let r = state.position;
    let v = state.velocity;
    write_out(
        out_derivative,
        CwDerivative {
            velocity: v,
            acceleration: Vec3 {
                x: 3.0 * n * n * r.x + 2.0 * n * v.y,
                y: -2.0 * n * v.x,
                z: -n * n * r.z,
            },
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_lambert_time_elliptic(
    mu: f64,
    semi_major_axis: f64,
    alpha: f64,
    beta: f64,
    revolutions: u32,
) -> f64 {
    if !finite(&[mu, semi_major_axis, alpha, beta]) || mu <= 0.0 || semi_major_axis <= 0.0 {
        return invalid_nan("invalid Lambert time parameters");
    }
    clear_error();
    let m = revolutions as f64;
    (semi_major_axis.powi(3) / mu).sqrt() * ((alpha - alpha.sin()) - (beta - beta.sin()) + TAU * m)
}

#[unsafe(no_mangle)]
pub extern "C" fn space_dh_transform(
    theta: f64,
    d: f64,
    a: f64,
    alpha: f64,
    out_transform: *mut DhTransform,
) -> Bool {
    if !finite(&[theta, d, a, alpha]) {
        set_error(ERR_INVALID_ARGUMENT, "invalid D-H parameters");
        return Bool::FALSE;
    }
    let (st, ct) = theta.sin_cos();
    let (sa, ca) = alpha.sin_cos();
    write_out(
        out_transform,
        DhTransform {
            m00: ct,
            m01: -st * ca,
            m02: st * sa,
            m03: a * ct,
            m10: st,
            m11: ct * ca,
            m12: -ct * sa,
            m13: a * st,
            m20: 0.0,
            m21: sa,
            m22: ca,
            m23: d,
            m30: 0.0,
            m31: 0.0,
            m32: 0.0,
            m33: 1.0,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_arm_first_joint_inverse(wrist_x: f64, wrist_y: f64) -> f64 {
    if !finite(&[wrist_x, wrist_y]) || (wrist_x.abs() <= EPS && wrist_y.abs() <= EPS) {
        return invalid_nan("invalid first joint inverse parameters");
    }
    clear_error();
    wrist_y.atan2(wrist_x)
}

#[unsafe(no_mangle)]
pub extern "C" fn space_arm_third_joint_angle(
    planar_radius: f64,
    vertical_offset: f64,
    link2: f64,
    link3: f64,
    elbow_up: Bool,
) -> f64 {
    if !finite(&[planar_radius, vertical_offset, link2, link3]) || link2 <= 0.0 || link3 <= 0.0 {
        return invalid_nan("invalid third joint inverse parameters");
    }
    let c3 = (planar_radius * planar_radius + vertical_offset * vertical_offset
        - link2 * link2
        - link3 * link3)
        / (2.0 * link2 * link3);
    if !(-1.0..=1.0).contains(&c3) {
        return invalid_nan("third joint target is unreachable");
    }
    clear_error();
    let s3 = (1.0 - c3 * c3).sqrt() * if elbow_up.0 != 0 { 1.0 } else { -1.0 };
    s3.atan2(c3)
}

#[unsafe(no_mangle)]
pub extern "C" fn space_manipulator_dynamics_diag(
    mass_matrix_diag: Vec3,
    joint_acceleration: Vec3,
    coriolis: Vec3,
    gravity: Vec3,
    out_dynamics: *mut ManipulatorDynamics,
) -> Bool {
    if !vec3_finite(mass_matrix_diag)
        || !vec3_finite(joint_acceleration)
        || !vec3_finite(coriolis)
        || !vec3_finite(gravity)
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid manipulator dynamics parameters",
        );
        return Bool::FALSE;
    }
    write_out(
        out_dynamics,
        ManipulatorDynamics {
            torque: Vec3 {
                x: mass_matrix_diag.x * joint_acceleration.x + coriolis.x + gravity.x,
                y: mass_matrix_diag.y * joint_acceleration.y + coriolis.y + gravity.y,
                z: mass_matrix_diag.z * joint_acceleration.z + coriolis.z + gravity.z,
            },
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_solar_panel_power(
    solar_flux: f64,
    area: f64,
    efficiency: f64,
    incidence_angle: f64,
    degradation: f64,
    out_power: *mut SolarPanelPower,
) -> Bool {
    if !finite(&[solar_flux, area, efficiency, incidence_angle, degradation])
        || solar_flux < 0.0
        || area < 0.0
        || efficiency < 0.0
        || degradation < 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid solar panel parameters");
        return Bool::FALSE;
    }
    let incident = solar_flux * area * incidence_angle.cos().max(0.0);
    write_out(
        out_power,
        SolarPanelPower {
            incident_power: incident,
            electrical_power: incident * efficiency * degradation,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_thermal_balance(
    absorbed_power: f64,
    internal_power: f64,
    emitted_area: f64,
    emissivity: f64,
    out_balance: *mut ThermalBalance,
) -> Bool {
    if !finite(&[absorbed_power, internal_power, emitted_area, emissivity])
        || emitted_area <= 0.0
        || emissivity <= 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid thermal balance parameters");
        return Bool::FALSE;
    }
    let net = absorbed_power + internal_power;
    let equilibrium_temperature = if net > 0.0 {
        (net / (emissivity * SIGMA * emitted_area)).powf(0.25)
    } else {
        0.0
    };
    write_out(
        out_balance,
        ThermalBalance {
            net_power: net,
            equilibrium_temperature,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_co2_mass_balance(
    current_mass: f64,
    generation_rate: f64,
    removal_rate: f64,
    leakage_rate: f64,
    volume: f64,
    dt: f64,
    out_balance: *mut Co2MassBalance,
) -> Bool {
    if !finite(&[
        current_mass,
        generation_rate,
        removal_rate,
        leakage_rate,
        volume,
        dt,
    ]) || volume <= 0.0
        || dt < 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid CO2 mass balance parameters");
        return Bool::FALSE;
    }
    let mass_rate = generation_rate - removal_rate - leakage_rate;
    let next_mass = (current_mass + mass_rate * dt).max(0.0);
    write_out(
        out_balance,
        Co2MassBalance {
            mass_rate,
            next_mass,
            concentration_rate: mass_rate / volume,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_friis_link(
    transmit_power: f64,
    transmit_gain: f64,
    receive_gain: f64,
    wavelength: f64,
    range: f64,
    system_loss: f64,
    out_link: *mut FriisLink,
) -> Bool {
    if !finite(&[
        transmit_power,
        transmit_gain,
        receive_gain,
        wavelength,
        range,
        system_loss,
    ]) || transmit_power < 0.0
        || transmit_gain < 0.0
        || receive_gain < 0.0
        || wavelength <= 0.0
        || range <= 0.0
        || system_loss <= 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Friis link parameters");
        return Bool::FALSE;
    }
    let path_gain = (wavelength / (4.0 * PI * range)).powi(2);
    let path_loss = 1.0 / path_gain;
    write_out(
        out_link,
        FriisLink {
            received_power: transmit_power * transmit_gain * receive_gain * path_gain / system_loss,
            path_loss,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_friis_wavelength_from_frequency(frequency: f64) -> f64 {
    if !frequency.is_finite() || frequency <= 0.0 {
        return invalid_nan("invalid Friis frequency");
    }
    clear_error();
    SPEED_OF_LIGHT / frequency
}

#[unsafe(no_mangle)]
pub extern "C" fn space_tsiolkovsky_delta_v(
    specific_impulse: f64,
    standard_gravity: f64,
    initial_mass: f64,
    final_mass: f64,
) -> f64 {
    if !finite(&[specific_impulse, standard_gravity, initial_mass, final_mass])
        || specific_impulse <= 0.0
        || standard_gravity <= 0.0
        || initial_mass <= 0.0
        || final_mass <= 0.0
        || initial_mass < final_mass
    {
        return invalid_nan("invalid Tsiolkovsky parameters");
    }
    clear_error();
    specific_impulse * standard_gravity * (initial_mass / final_mass).ln()
}

#[unsafe(no_mangle)]
pub extern "C" fn space_hohmann_transfer(
    mu: f64,
    radius1: f64,
    radius2: f64,
    out_transfer: *mut HohmannTransfer,
) -> Bool {
    if !finite(&[mu, radius1, radius2]) || mu <= 0.0 || radius1 <= 0.0 || radius2 <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "invalid Hohmann transfer parameters");
        return Bool::FALSE;
    }
    let transfer_a = 0.5 * (radius1 + radius2);
    let circular1 = (mu / radius1).sqrt();
    let circular2 = (mu / radius2).sqrt();
    let transfer_periapsis = (mu * (2.0 / radius1 - 1.0 / transfer_a)).sqrt();
    let transfer_apoapsis = (mu * (2.0 / radius2 - 1.0 / transfer_a)).sqrt();
    let delta_v1 = transfer_periapsis - circular1;
    let delta_v2 = circular2 - transfer_apoapsis;
    write_out(
        out_transfer,
        HohmannTransfer {
            delta_v1,
            delta_v2,
            total_delta_v: delta_v1.abs() + delta_v2.abs(),
            transfer_time: PI * (transfer_a.powi(3) / mu).sqrt(),
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_atmospheric_density_scale_height(
    reference_density: f64,
    altitude: f64,
    reference_altitude: f64,
    scale_height: f64,
) -> f64 {
    if !finite(&[
        reference_density,
        altitude,
        reference_altitude,
        scale_height,
    ]) || reference_density < 0.0
        || scale_height <= 0.0
    {
        return invalid_nan("invalid atmospheric density scale-height parameters");
    }
    clear_error();
    reference_density * (-(altitude - reference_altitude) / scale_height).exp()
}

#[unsafe(no_mangle)]
pub extern "C" fn space_atmospheric_drag_acceleration(
    velocity: Vec3,
    atmosphere_velocity: Vec3,
    density: f64,
    drag_coefficient: f64,
    area: f64,
    mass: f64,
    out_acceleration: *mut Vec3,
) -> Bool {
    if !vec3_finite(velocity)
        || !vec3_finite(atmosphere_velocity)
        || !finite(&[density, drag_coefficient, area, mass])
        || density < 0.0
        || drag_coefficient < 0.0
        || area < 0.0
        || mass <= 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid atmospheric drag parameters");
        return Bool::FALSE;
    }
    let rel = vec3_to_rapier(velocity) - vec3_to_rapier(atmosphere_velocity);
    let speed = rel.length();
    let acc = if speed > EPS {
        -rel * (0.5 * density * speed * drag_coefficient * area / mass)
    } else {
        Vector::ZERO
    };
    write_out(out_acceleration, vec3_from_rapier(acc))
}

#[unsafe(no_mangle)]
pub extern "C" fn space_apply_atmospheric_drag_to_body(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    atmosphere_velocity: Vec3,
    density: f64,
    drag_coefficient: f64,
    area: f64,
    mass: f64,
    wake_up: Bool,
    out_acceleration: *mut Vec3,
) -> Bool {
    if !mass.is_finite() || mass <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "invalid atmospheric drag body mass");
        return Bool::FALSE;
    }
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(body) = world
        .inner
        .bodies
        .get_mut(unpack_rigid_body_handle(body_handle))
    else {
        set_error(ERR_NOT_FOUND, "body was not found");
        return Bool::FALSE;
    };
    let velocity = vec3_from_rapier(body.linvel());
    let mut acceleration = Vec3::default();
    if space_atmospheric_drag_acceleration(
        velocity,
        atmosphere_velocity,
        density,
        drag_coefficient,
        area,
        mass,
        &mut acceleration,
    ) == Bool::FALSE
    {
        return Bool::FALSE;
    }
    body.add_force(vec3_to_rapier(acceleration) * mass, wake_up.0 != 0);
    write_optional_out(out_acceleration, acceleration);
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn space_apply_atmospheric_drag_to_body_flag(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    atmosphere_velocity: Vec3,
    density: f64,
    drag_coefficient: f64,
    area: f64,
    mass: f64,
    wake_up: Bool,
    out_acceleration: *mut Vec3,
) -> u8 {
    space_apply_atmospheric_drag_to_body(
        world,
        body_handle,
        atmosphere_velocity,
        density,
        drag_coefficient,
        area,
        mass,
        wake_up,
        out_acceleration,
    )
    .0
}

#[unsafe(no_mangle)]
pub extern "C" fn space_triad_attitude(
    body_primary: Vec3,
    body_secondary: Vec3,
    reference_primary: Vec3,
    reference_secondary: Vec3,
    out_attitude: *mut Quat,
) -> Bool {
    let make_basis = |a: Vec3, b: Vec3| -> Option<(Vector, Vector, Vector)> {
        let t1 = vec3_to_rapier(a).try_normalize()?;
        let t2 = cross(t1, vec3_to_rapier(b)).try_normalize()?;
        let t3 = cross(t1, t2);
        Some((t1, t2, t3))
    };
    let Some((bt1, bt2, bt3)) = make_basis(body_primary, body_secondary) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid TRIAD body vectors");
        return Bool::FALSE;
    };
    let Some((rt1, rt2, rt3)) = make_basis(reference_primary, reference_secondary) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid TRIAD reference vectors");
        return Bool::FALSE;
    };
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
        Quat {
            w: 0.25 * s,
            i: (m21 - m12) / s,
            j: (m02 - m20) / s,
            k: (m10 - m01) / s,
        }
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        Quat {
            w: (m21 - m12) / s,
            i: 0.25 * s,
            j: (m01 + m10) / s,
            k: (m02 + m20) / s,
        }
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        Quat {
            w: (m02 - m20) / s,
            i: (m01 + m10) / s,
            j: 0.25 * s,
            k: (m12 + m21) / s,
        }
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        Quat {
            w: (m10 - m01) / s,
            i: (m02 + m20) / s,
            j: (m12 + m21) / s,
            k: 0.25 * s,
        }
    };
    write_out(out_attitude, q)
}

#[unsafe(no_mangle)]
pub extern "C" fn space_ekf_predict_scalar(
    state: f64,
    covariance: f64,
    nonlinear_delta: f64,
    jacobian: f64,
    process_noise: f64,
    out_prediction: *mut ScalarKalman,
) -> Bool {
    if !finite(&[state, covariance, nonlinear_delta, jacobian, process_noise])
        || covariance < 0.0
        || process_noise < 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid EKF prediction parameters");
        return Bool::FALSE;
    }
    write_out(
        out_prediction,
        ScalarKalman {
            value: state + nonlinear_delta,
            covariance: jacobian * covariance * jacobian + process_noise,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_ekf_gain_scalar(
    covariance: f64,
    measurement_jacobian: f64,
    measurement_noise: f64,
) -> f64 {
    if !finite(&[covariance, measurement_jacobian, measurement_noise])
        || covariance < 0.0
        || measurement_noise < 0.0
    {
        return invalid_nan("invalid EKF gain parameters");
    }
    let innovation_covariance =
        measurement_jacobian * covariance * measurement_jacobian + measurement_noise;
    if innovation_covariance <= EPS {
        return invalid_nan("invalid EKF innovation covariance");
    }
    clear_error();
    covariance * measurement_jacobian / innovation_covariance
}

#[unsafe(no_mangle)]
pub extern "C" fn space_ekf_update_scalar(
    predicted_state: f64,
    predicted_covariance: f64,
    measurement: f64,
    predicted_measurement: f64,
    kalman_gain: f64,
    measurement_jacobian: f64,
    out_update: *mut ScalarKalman,
) -> Bool {
    if !finite(&[
        predicted_state,
        predicted_covariance,
        measurement,
        predicted_measurement,
        kalman_gain,
        measurement_jacobian,
    ]) || predicted_covariance < 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid EKF update parameters");
        return Bool::FALSE;
    }
    write_out(
        out_update,
        ScalarKalman {
            value: predicted_state + kalman_gain * (measurement - predicted_measurement),
            covariance: (1.0 - kalman_gain * measurement_jacobian) * predicted_covariance,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_least_squares_attitude_two_vector(
    body_primary: Vec3,
    body_secondary: Vec3,
    reference_primary: Vec3,
    reference_secondary: Vec3,
    out_attitude: *mut LeastSquaresAttitude,
) -> Bool {
    let mut quat = Quat::default();
    if space_triad_attitude(
        body_primary,
        body_secondary,
        reference_primary,
        reference_secondary,
        &mut quat,
    ) != Bool::TRUE
    {
        return Bool::FALSE;
    }
    write_out(
        out_attitude,
        LeastSquaresAttitude {
            attitude: quat,
            rms_error: 0.0,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_gnss_pseudorange(
    receiver: Vec3,
    satellite: Vec3,
    receiver_clock_bias: f64,
    satellite_clock_bias: f64,
    ionosphere_delay: f64,
    troposphere_delay: f64,
    out_observation: *mut GnssObservation,
) -> Bool {
    if !vec3_finite(receiver)
        || !vec3_finite(satellite)
        || !finite(&[
            receiver_clock_bias,
            satellite_clock_bias,
            ionosphere_delay,
            troposphere_delay,
        ])
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid GNSS pseudorange parameters");
        return Bool::FALSE;
    }
    let range = (vec3_to_rapier(satellite) - vec3_to_rapier(receiver)).length();
    write_out(
        out_observation,
        GnssObservation {
            value: range
                + SPEED_OF_LIGHT * (receiver_clock_bias - satellite_clock_bias)
                + ionosphere_delay
                + troposphere_delay,
            geometric_range: range,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_gnss_double_difference_carrier_phase(
    range_rover_sat_a: f64,
    range_rover_sat_b: f64,
    range_base_sat_a: f64,
    range_base_sat_b: f64,
    wavelength: f64,
    ambiguity: f64,
) -> f64 {
    if !finite(&[
        range_rover_sat_a,
        range_rover_sat_b,
        range_base_sat_a,
        range_base_sat_b,
        wavelength,
        ambiguity,
    ]) || wavelength <= 0.0
    {
        return invalid_nan("invalid double-difference carrier phase parameters");
    }
    clear_error();
    ((range_rover_sat_a - range_rover_sat_b) - (range_base_sat_a - range_base_sat_b)) / wavelength
        + ambiguity
}

#[unsafe(no_mangle)]
pub extern "C" fn space_structural_natural_frequency(
    stiffness: f64,
    mass: f64,
    mode_factor: f64,
) -> f64 {
    if !finite(&[stiffness, mass, mode_factor]) || stiffness <= 0.0 || mass <= 0.0 {
        return invalid_nan("invalid structural frequency parameters");
    }
    clear_error();
    mode_factor * (stiffness / mass).sqrt() / TAU
}

#[unsafe(no_mangle)]
pub extern "C" fn space_contact_force_hunt_crossley(
    penetration: f64,
    penetration_rate: f64,
    stiffness: f64,
    damping: f64,
    exponent: f64,
    out_force: *mut ContactForceModel,
) -> Bool {
    if !finite(&[penetration, penetration_rate, stiffness, damping, exponent])
        || stiffness < 0.0
        || damping < 0.0
        || exponent <= 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid contact force parameters");
        return Bool::FALSE;
    }
    let depth = penetration.max(0.0);
    let normal = stiffness * depth.powf(exponent);
    let damping_force = damping * depth.powf(exponent) * penetration_rate.max(0.0);
    write_out(
        out_force,
        ContactForceModel {
            normal_force: normal,
            damping_force,
            total_force: normal + damping_force,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_radiation_absorbed_dose(
    energy_joules: f64,
    mass_kg: f64,
    quality_factor: f64,
) -> f64 {
    if !finite(&[energy_joules, mass_kg, quality_factor]) || mass_kg <= 0.0 || quality_factor < 0.0
    {
        return invalid_nan("invalid radiation dose parameters");
    }
    clear_error();
    energy_joules / mass_kg * quality_factor
}

#[unsafe(no_mangle)]
pub extern "C" fn space_semi_major_axis_decay_rate(
    semi_major_axis: f64,
    density: f64,
    drag_coefficient: f64,
    area: f64,
    mass: f64,
    mu: f64,
) -> f64 {
    if !finite(&[semi_major_axis, density, drag_coefficient, area, mass, mu])
        || semi_major_axis <= 0.0
        || density < 0.0
        || drag_coefficient < 0.0
        || area < 0.0
        || mass <= 0.0
        || mu <= 0.0
    {
        return invalid_nan("invalid semi-major-axis decay parameters");
    }
    clear_error();
    let v = (mu / semi_major_axis).sqrt();
    -density * drag_coefficient * area / mass * semi_major_axis * v
}

#[unsafe(no_mangle)]
pub extern "C" fn space_heat_pipe_thermal_resistance(
    evaporator_resistance: f64,
    vapor_resistance: f64,
    condenser_resistance: f64,
    wick_resistance: f64,
) -> f64 {
    if !finite(&[
        evaporator_resistance,
        vapor_resistance,
        condenser_resistance,
        wick_resistance,
    ]) {
        return invalid_nan("invalid heat pipe resistance parameters");
    }
    clear_error();
    evaporator_resistance + vapor_resistance + condenser_resistance + wick_resistance
}

#[unsafe(no_mangle)]
pub extern "C" fn space_battery_equivalent_circuit(
    open_circuit_voltage: f64,
    current: f64,
    ohmic_resistance: f64,
    rc_voltage: f64,
    rc_resistance: f64,
    rc_capacitance: f64,
    capacity_coulombs: f64,
    out_battery: *mut BatteryEquivalentCircuit,
) -> Bool {
    if !finite(&[
        open_circuit_voltage,
        current,
        ohmic_resistance,
        rc_voltage,
        rc_resistance,
        rc_capacitance,
        capacity_coulombs,
    ]) || ohmic_resistance < 0.0
        || rc_resistance <= 0.0
        || rc_capacitance <= 0.0
        || capacity_coulombs <= 0.0
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid battery equivalent-circuit parameters",
        );
        return Bool::FALSE;
    }
    write_out(
        out_battery,
        BatteryEquivalentCircuit {
            terminal_voltage: open_circuit_voltage - current * ohmic_resistance - rc_voltage,
            rc_voltage_dot: -rc_voltage / (rc_resistance * rc_capacitance)
                + current / rc_capacitance,
            state_of_charge_dot: -current / capacity_coulombs,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_hall_thruster_performance(
    mass_flow_rate: f64,
    exhaust_velocity: f64,
    input_power: f64,
    standard_gravity: f64,
    out_performance: *mut HallThrusterPerformance,
) -> Bool {
    if !finite(&[
        mass_flow_rate,
        exhaust_velocity,
        input_power,
        standard_gravity,
    ]) || mass_flow_rate < 0.0
        || exhaust_velocity < 0.0
        || input_power <= 0.0
        || standard_gravity <= 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Hall thruster parameters");
        return Bool::FALSE;
    }
    let thrust = mass_flow_rate * exhaust_velocity;
    write_out(
        out_performance,
        HallThrusterPerformance {
            thrust,
            specific_impulse: exhaust_velocity / standard_gravity,
            efficiency: 0.5 * mass_flow_rate * exhaust_velocity * exhaust_velocity / input_power,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_artificial_potential_guidance(
    position: Vec3,
    target: Vec3,
    obstacle: Vec3,
    attractive_gain: f64,
    repulsive_gain: f64,
    influence_radius: f64,
    out_command: *mut Vec3,
) -> Bool {
    if !vec3_finite(position)
        || !vec3_finite(target)
        || !vec3_finite(obstacle)
        || !finite(&[attractive_gain, repulsive_gain, influence_radius])
        || influence_radius <= 0.0
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid artificial potential guidance parameters",
        );
        return Bool::FALSE;
    }
    let p = vec3_to_rapier(position);
    let attractive = (vec3_to_rapier(target) - p) * attractive_gain;
    let away = p - vec3_to_rapier(obstacle);
    let d = away.length();
    let repulsive = if d > EPS && d < influence_radius {
        away / d * repulsive_gain * (1.0 / d - 1.0 / influence_radius) / (d * d)
    } else {
        Vector::ZERO
    };
    write_out(out_command, vec3_from_rapier(attractive + repulsive))
}

#[unsafe(no_mangle)]
pub extern "C" fn space_debris_collision_probability(
    miss_distance: f64,
    combined_radius: f64,
    sigma_radial: f64,
    sigma_intrack: f64,
    out_probability: *mut CollisionProbability,
) -> Bool {
    if !finite(&[miss_distance, combined_radius, sigma_radial, sigma_intrack])
        || combined_radius < 0.0
        || sigma_radial <= 0.0
        || sigma_intrack <= 0.0
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid debris collision probability parameters",
        );
        return Bool::FALSE;
    }
    let sigma = (sigma_radial * sigma_intrack).sqrt();
    let probability = (combined_radius * combined_radius / (2.0 * sigma_radial * sigma_intrack))
        * (-0.5 * miss_distance * miss_distance / (sigma * sigma)).exp();
    write_out(
        out_probability,
        CollisionProbability {
            probability: probability.clamp(0.0, 1.0),
            combined_sigma: sigma,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_atomic_oxygen_erosion(
    fluence: f64,
    erosion_yield: f64,
    area: f64,
    density: f64,
    out_erosion: *mut AtomicOxygenErosion,
) -> Bool {
    if !finite(&[fluence, erosion_yield, area, density])
        || fluence < 0.0
        || erosion_yield < 0.0
        || area < 0.0
        || density < 0.0
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid atomic oxygen erosion parameters",
        );
        return Bool::FALSE;
    }
    let volume_loss = fluence * erosion_yield * area;
    write_out(
        out_erosion,
        AtomicOxygenErosion {
            volume_loss,
            mass_loss: volume_loss * density,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_flexible_mode_derivative(
    displacement: f64,
    velocity: f64,
    natural_frequency: f64,
    damping_ratio: f64,
    modal_force: f64,
    modal_mass: f64,
    out_derivative: *mut FlexibleModeDerivative,
) -> Bool {
    if !finite(&[
        displacement,
        velocity,
        natural_frequency,
        damping_ratio,
        modal_force,
        modal_mass,
    ]) || natural_frequency < 0.0
        || damping_ratio < 0.0
        || modal_mass <= 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid flexible mode parameters");
        return Bool::FALSE;
    }
    write_out(
        out_derivative,
        FlexibleModeDerivative {
            displacement_dot: velocity,
            velocity_dot: modal_force / modal_mass
                - 2.0 * damping_ratio * natural_frequency * velocity
                - natural_frequency * natural_frequency * displacement,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_slosh_pendulum_derivative(
    angle: f64,
    angular_rate: f64,
    length: f64,
    damping: f64,
    lateral_acceleration: f64,
    gravity: f64,
    out_derivative: *mut SloshPendulumDerivative,
) -> Bool {
    if !finite(&[
        angle,
        angular_rate,
        length,
        damping,
        lateral_acceleration,
        gravity,
    ]) || length <= 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid slosh pendulum parameters");
        return Bool::FALSE;
    }
    write_out(
        out_derivative,
        SloshPendulumDerivative {
            angle_dot: angular_rate,
            angular_rate_dot: -(gravity / length) * angle.sin()
                - damping * angular_rate
                - lateral_acceleration / length,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_variational_two_body(
    position: Vec3,
    velocity: Vec3,
    mu: f64,
    out_derivative: *mut VariationalState,
) -> Bool {
    if !vec3_finite(position) || !vec3_finite(velocity) || !mu.is_finite() || mu <= 0.0 {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid variational equation parameters",
        );
        return Bool::FALSE;
    }
    let r = vec3_to_rapier(position);
    let rn = r.length();
    if rn <= EPS {
        set_error(ERR_INVALID_ARGUMENT, "variational position is zero");
        return Bool::FALSE;
    }
    write_out(
        out_derivative,
        VariationalState {
            position_dot: velocity,
            // Compute mu/r³ as mu/(r² * |r|) to avoid powi(3) overflow
            velocity_dot: vec3_from_rapier(-r * (mu / (rn * rn.sqrt()))),
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_single_phase_loop_heat_transfer(
    mass_flow_rate: f64,
    specific_heat: f64,
    inlet_temperature: f64,
    heat_input: f64,
    out_heat: *mut FluidLoopHeatTransfer,
) -> Bool {
    if !finite(&[mass_flow_rate, specific_heat, inlet_temperature, heat_input])
        || mass_flow_rate <= 0.0
        || specific_heat <= 0.0
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid single-phase loop heat parameters",
        );
        return Bool::FALSE;
    }
    write_out(
        out_heat,
        FluidLoopHeatTransfer {
            heat_rate: heat_input,
            outlet_temperature: inlet_temperature + heat_input / (mass_flow_rate * specific_heat),
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_radar_range_rate(
    radar_position: Vec3,
    target_position: Vec3,
    radar_velocity: Vec3,
    target_velocity: Vec3,
    out_measurement: *mut RadarMeasurement,
) -> Bool {
    if !vec3_finite(radar_position)
        || !vec3_finite(target_position)
        || !vec3_finite(radar_velocity)
        || !vec3_finite(target_velocity)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid radar measurement parameters");
        return Bool::FALSE;
    }
    let line = vec3_to_rapier(target_position) - vec3_to_rapier(radar_position);
    let range = line.length();
    if range <= EPS {
        set_error(ERR_INVALID_ARGUMENT, "radar range is zero");
        return Bool::FALSE;
    }
    let rel_v = vec3_to_rapier(target_velocity) - vec3_to_rapier(radar_velocity);
    write_out(
        out_measurement,
        RadarMeasurement {
            range,
            range_rate: rel_v.dot(line / range),
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_mass_properties_two_body(
    mass1: f64,
    position1: Vec3,
    inertia1_diag: Vec3,
    mass2: f64,
    position2: Vec3,
    inertia2_diag: Vec3,
    out_properties: *mut MassProperties,
) -> Bool {
    if !finite(&[mass1, mass2])
        || mass1 < 0.0
        || mass2 < 0.0
        || mass1 + mass2 <= 0.0
        || !vec3_finite(position1)
        || !vec3_finite(position2)
        || !vec3_finite(inertia1_diag)
        || !vec3_finite(inertia2_diag)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid mass properties parameters");
        return Bool::FALSE;
    }
    let p1 = vec3_to_rapier(position1);
    let p2 = vec3_to_rapier(position2);
    let total = mass1 + mass2;
    let com = (p1 * mass1 + p2 * mass2) / total;
    let parallel = |m: f64, p: Vector, i: Vec3| -> Vec3 {
        let d = p - com;
        Vec3 {
            x: i.x + m * (d.y * d.y + d.z * d.z),
            y: i.y + m * (d.x * d.x + d.z * d.z),
            z: i.z + m * (d.x * d.x + d.y * d.y),
        }
    };
    let i1 = parallel(mass1, p1, inertia1_diag);
    let i2 = parallel(mass2, p2, inertia2_diag);
    write_out(
        out_properties,
        MassProperties {
            center_of_mass: vec3_from_rapier(com),
            inertia_diag: Vec3 {
                x: i1.x + i2.x,
                y: i1.y + i2.y,
                z: i1.z + i2.z,
            },
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_docking_buffer_energy(
    relative_speed: f64,
    reduced_mass: f64,
    stroke: f64,
    efficiency: f64,
) -> f64 {
    if !finite(&[relative_speed, reduced_mass, stroke, efficiency])
        || reduced_mass < 0.0
        || stroke <= 0.0
        || efficiency <= 0.0
    {
        return invalid_nan("invalid docking buffer parameters");
    }
    clear_error();
    0.5 * reduced_mass * relative_speed * relative_speed / efficiency
}

#[unsafe(no_mangle)]
pub extern "C" fn space_bang_off_bang_profile(
    angle: f64,
    max_acceleration: f64,
    max_rate: f64,
    out_profile: *mut BangOffBangProfile,
) -> Bool {
    if !finite(&[angle, max_acceleration, max_rate]) || max_acceleration <= 0.0 || max_rate <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "invalid bang-off-bang parameters");
        return Bool::FALSE;
    }
    let theta = angle.abs();
    let triangular_angle = max_rate * max_rate / max_acceleration;
    let (coast, total, switch_angle) = if theta <= triangular_angle {
        let t = (theta / max_acceleration).sqrt();
        (0.0, 2.0 * t, 0.5 * theta)
    } else {
        let accel_time = max_rate / max_acceleration;
        let coast = (theta - triangular_angle) / max_rate;
        (coast, 2.0 * accel_time + coast, 0.5 * triangular_angle)
    };
    write_out(
        out_profile,
        BangOffBangProfile {
            coast_time: coast,
            total_time: total,
            switch_angle,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_solar_radiation_pressure_acceleration(
    sun_direction: Vec3,
    solar_flux: f64,
    reflectivity: f64,
    area: f64,
    mass: f64,
    out_acceleration: *mut Vec3,
) -> Bool {
    if !vec3_finite(sun_direction)
        || !finite(&[solar_flux, reflectivity, area, mass])
        || solar_flux < 0.0
        || reflectivity < 0.0
        || area < 0.0
        || mass <= 0.0
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid solar radiation pressure parameters",
        );
        return Bool::FALSE;
    }
    let Some(dir) = vec3_to_rapier(sun_direction).try_normalize() else {
        set_error(ERR_INVALID_ARGUMENT, "sun direction is zero");
        return Bool::FALSE;
    };
    write_out(
        out_acceleration,
        vec3_from_rapier(dir * (solar_flux / SPEED_OF_LIGHT * reflectivity * area / mass)),
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_apply_solar_radiation_pressure_to_body(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    sun_direction: Vec3,
    solar_flux: f64,
    reflectivity: f64,
    area: f64,
    mass: f64,
    wake_up: Bool,
    out_acceleration: *mut Vec3,
) -> Bool {
    if !mass.is_finite() || mass <= 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "invalid solar radiation body mass");
        return Bool::FALSE;
    }
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(body) = world
        .inner
        .bodies
        .get_mut(unpack_rigid_body_handle(body_handle))
    else {
        set_error(ERR_NOT_FOUND, "body was not found");
        return Bool::FALSE;
    };
    let mut acceleration = Vec3::default();
    if space_solar_radiation_pressure_acceleration(
        sun_direction,
        solar_flux,
        reflectivity,
        area,
        mass,
        &mut acceleration,
    ) == Bool::FALSE
    {
        return Bool::FALSE;
    }
    body.add_force(vec3_to_rapier(acceleration) * mass, wake_up.0 != 0);
    write_optional_out(out_acceleration, acceleration);
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn space_apply_solar_radiation_pressure_to_body_flag(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    sun_direction: Vec3,
    solar_flux: f64,
    reflectivity: f64,
    area: f64,
    mass: f64,
    wake_up: Bool,
    out_acceleration: *mut Vec3,
) -> u8 {
    space_apply_solar_radiation_pressure_to_body(
        world,
        body_handle,
        sun_direction,
        solar_flux,
        reflectivity,
        area,
        mass,
        wake_up,
        out_acceleration,
    )
    .0
}

#[unsafe(no_mangle)]
pub extern "C" fn space_gravity_gradient_torque(
    position: Vec3,
    inertia_diag: Vec3,
    mu: f64,
    out_torque: *mut Vec3,
) -> Bool {
    if !vec3_finite(position) || !vec3_finite(inertia_diag) || !mu.is_finite() || mu <= 0.0 {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid gravity-gradient torque parameters",
        );
        return Bool::FALSE;
    }
    let r = vec3_to_rapier(position);
    let rn = r.length();
    if rn <= EPS {
        set_error(ERR_INVALID_ARGUMENT, "gravity-gradient position is zero");
        return Bool::FALSE;
    }
    let n = r / rn;
    let in_vec = Vector::new(
        inertia_diag.x * n.x,
        inertia_diag.y * n.y,
        inertia_diag.z * n.z,
    );
    write_out(
        out_torque,
        vec3_from_rapier(cross(n, in_vec) * (3.0 * mu / (rn * rn.sqrt()))),
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_apply_gravity_gradient_torque_to_body(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    inertia_diag: Vec3,
    mu: f64,
    wake_up: Bool,
    out_torque: *mut Vec3,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(body) = world
        .inner
        .bodies
        .get_mut(unpack_rigid_body_handle(body_handle))
    else {
        set_error(ERR_NOT_FOUND, "body was not found");
        return Bool::FALSE;
    };
    let position = vec3_from_rapier(body.translation());
    let mut torque = Vec3::default();
    if space_gravity_gradient_torque(position, inertia_diag, mu, &mut torque) == Bool::FALSE {
        return Bool::FALSE;
    }
    body.add_torque(vec3_to_rapier(torque), wake_up.0 != 0);
    write_optional_out(out_torque, torque);
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn space_apply_gravity_gradient_torque_to_body_flag(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    inertia_diag: Vec3,
    mu: f64,
    wake_up: Bool,
    out_torque: *mut Vec3,
) -> u8 {
    space_apply_gravity_gradient_torque_to_body(
        world,
        body_handle,
        inertia_diag,
        mu,
        wake_up,
        out_torque,
    )
    .0
}

#[unsafe(no_mangle)]
pub extern "C" fn space_magnetic_torquer_dipole(
    commanded_torque: Vec3,
    magnetic_field: Vec3,
    max_dipole: f64,
    out_dipole: *mut Vec3,
) -> Bool {
    if !vec3_finite(commanded_torque)
        || !vec3_finite(magnetic_field)
        || !max_dipole.is_finite()
        || max_dipole < 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid magnetic torquer parameters");
        return Bool::FALSE;
    }
    let b = vec3_to_rapier(magnetic_field);
    let b2 = b.length_squared();
    if b2 <= EPS {
        set_error(ERR_INVALID_ARGUMENT, "magnetic field is zero");
        return Bool::FALSE;
    }
    let mut m = cross(b, vec3_to_rapier(commanded_torque)) / b2;
    let mn = m.length();
    if mn > max_dipole && mn > EPS {
        m *= max_dipole / mn;
    }
    write_out(out_dipole, vec3_from_rapier(m))
}

#[unsafe(no_mangle)]
pub extern "C" fn space_apply_magnetic_torquer_to_body(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    commanded_torque: Vec3,
    magnetic_field: Vec3,
    max_dipole: f64,
    wake_up: Bool,
    out_dipole: *mut Vec3,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    let Some(body) = world
        .inner
        .bodies
        .get_mut(unpack_rigid_body_handle(body_handle))
    else {
        set_error(ERR_NOT_FOUND, "body was not found");
        return Bool::FALSE;
    };
    let mut dipole = Vec3::default();
    if space_magnetic_torquer_dipole(commanded_torque, magnetic_field, max_dipole, &mut dipole)
        == Bool::FALSE
    {
        return Bool::FALSE;
    }
    let torque = cross(vec3_to_rapier(dipole), vec3_to_rapier(magnetic_field));
    body.add_torque(torque, wake_up.0 != 0);
    write_optional_out(out_dipole, dipole);
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn space_apply_magnetic_torquer_to_body_flag(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    commanded_torque: Vec3,
    magnetic_field: Vec3,
    max_dipole: f64,
    wake_up: Bool,
    out_dipole: *mut Vec3,
) -> u8 {
    space_apply_magnetic_torquer_to_body(
        world,
        body_handle,
        commanded_torque,
        magnetic_field,
        max_dipole,
        wake_up,
        out_dipole,
    )
    .0
}

#[unsafe(no_mangle)]
pub extern "C" fn space_cmg_robust_pseudoinverse_diag(
    jacobian_diag: Vec3,
    desired_torque: Vec3,
    damping: f64,
    out_inverse: *mut CmgRobustInverse,
) -> Bool {
    if !vec3_finite(jacobian_diag)
        || !vec3_finite(desired_torque)
        || !damping.is_finite()
        || damping < 0.0
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid CMG robust inverse parameters",
        );
        return Bool::FALSE;
    }
    let solve = |j: f64, t: f64| j * t / (j * j + damping * damping);
    write_out(
        out_inverse,
        CmgRobustInverse {
            gimbal_rates: Vec3 {
                x: solve(jacobian_diag.x, desired_torque.x),
                y: solve(jacobian_diag.y, desired_torque.y),
                z: solve(jacobian_diag.z, desired_torque.z),
            },
            damping,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_sgp4_j2_secular_rates(
    semi_major_axis: f64,
    eccentricity: f64,
    inclination: f64,
    mean_motion: f64,
    equatorial_radius: f64,
    j2: f64,
    out_rates: *mut Sgp4SecularRates,
) -> Bool {
    if !finite(&[
        semi_major_axis,
        eccentricity,
        inclination,
        mean_motion,
        equatorial_radius,
        j2,
    ]) || semi_major_axis <= 0.0
        || !(0.0..1.0).contains(&eccentricity)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid SGP4 secular parameters");
        return Bool::FALSE;
    }
    let p = semi_major_axis * (1.0 - eccentricity * eccentricity);
    let factor = 1.5 * j2 * mean_motion * (equatorial_radius / p).powi(2);
    write_out(
        out_rates,
        Sgp4SecularRates {
            mean_motion_dot: 0.0,
            raan_dot: -factor * inclination.cos(),
            argument_of_perigee_dot: 0.5 * factor * (5.0 * inclination.cos().powi(2) - 1.0),
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_docking_glideslope_command(
    range: f64,
    desired_slope: f64,
    closing_speed_limit: f64,
) -> f64 {
    if !finite(&[range, desired_slope, closing_speed_limit]) || closing_speed_limit < 0.0 {
        return invalid_nan("invalid docking glideslope parameters");
    }
    clear_error();
    (-desired_slope * range).clamp(-closing_speed_limit, closing_speed_limit)
}

#[unsafe(no_mangle)]
pub extern "C" fn space_sagnac_phase_rate(area: f64, angular_rate: f64, wavelength: f64) -> f64 {
    if !finite(&[area, angular_rate, wavelength]) || wavelength <= 0.0 {
        return invalid_nan("invalid Sagnac parameters");
    }
    clear_error();
    8.0 * PI * area * angular_rate / (wavelength * SPEED_OF_LIGHT)
}

#[unsafe(no_mangle)]
pub extern "C" fn space_solar_array_pd_torque(
    angle_error: f64,
    rate_error: f64,
    kp: f64,
    kd: f64,
) -> f64 {
    if !finite(&[angle_error, rate_error, kp, kd]) {
        return invalid_nan("invalid solar array PD parameters");
    }
    clear_error();
    kp * angle_error + kd * rate_error
}

#[unsafe(no_mangle)]
pub extern "C" fn space_sabatier_methane_rate(
    co2_molar_rate: f64,
    h2_molar_rate: f64,
    conversion: f64,
    out_rate: *mut ChemicalReactionRate,
) -> Bool {
    if !finite(&[co2_molar_rate, h2_molar_rate, conversion])
        || co2_molar_rate < 0.0
        || h2_molar_rate < 0.0
        || !(0.0..=1.0).contains(&conversion)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Sabatier parameters");
        return Bool::FALSE;
    }
    let methane = co2_molar_rate.min(h2_molar_rate / 4.0) * conversion;
    write_out(
        out_rate,
        ChemicalReactionRate {
            reactant_rate: methane,
            product_rate: methane,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_spe_oxygen_rate(
    current: f64,
    cells: f64,
    faraday_efficiency: f64,
    out_rate: *mut ChemicalReactionRate,
) -> Bool {
    if !finite(&[current, cells, faraday_efficiency])
        || current < 0.0
        || cells <= 0.0
        || !(0.0..=1.0).contains(&faraday_efficiency)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid SPE oxygen parameters");
        return Bool::FALSE;
    }
    let faraday = 96_485.332_12;
    let oxygen = current * cells * faraday_efficiency / (4.0 * faraday);
    write_out(
        out_rate,
        ChemicalReactionRate {
            reactant_rate: current * cells / (2.0 * faraday),
            product_rate: oxygen,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_radiator_power(
    area: f64,
    emissivity: f64,
    temperature: f64,
    sink_temperature: f64,
    absorbed_power: f64,
    out_power: *mut RadiatorPower,
) -> Bool {
    if !finite(&[
        area,
        emissivity,
        temperature,
        sink_temperature,
        absorbed_power,
    ]) || area < 0.0
        || emissivity < 0.0
        || temperature < 0.0
        || sink_temperature < 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid radiator power parameters");
        return Bool::FALSE;
    }
    let emitted =
        emissivity * SIGMA * area * (temperature.powi(4) - sink_temperature.powi(4)).max(0.0);
    write_out(
        out_power,
        RadiatorPower {
            emitted_power: emitted,
            net_power: emitted - absorbed_power,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn space_whipple_critical_projectile_diameter(
    bumper_thickness: f64,
    bumper_density: f64,
    projectile_density: f64,
    impact_velocity: f64,
    standoff: f64,
) -> f64 {
    if !finite(&[
        bumper_thickness,
        bumper_density,
        projectile_density,
        impact_velocity,
        standoff,
    ]) || bumper_thickness <= 0.0
        || bumper_density <= 0.0
        || projectile_density <= 0.0
        || impact_velocity <= 0.0
        || standoff <= 0.0
    {
        return invalid_nan("invalid Whipple shield parameters");
    }
    clear_error();
    bumper_thickness
        * (bumper_density / projectile_density).sqrt()
        * (standoff / bumper_thickness).powf(1.0 / 3.0)
        * (7_000.0 / impact_velocity).powf(2.0 / 3.0)
}

#[unsafe(no_mangle)]
pub extern "C" fn space_surface_charging_current_balance(
    photo_current: f64,
    secondary_current: f64,
    backscatter_current: f64,
    electron_current: f64,
    ion_current: f64,
) -> f64 {
    if !finite(&[
        photo_current,
        secondary_current,
        backscatter_current,
        electron_current,
        ion_current,
    ]) {
        return invalid_nan("invalid surface charging current parameters");
    }
    clear_error();
    photo_current + secondary_current + backscatter_current + ion_current - electron_current
}

#[unsafe(no_mangle)]
pub extern "C" fn space_airlock_depressurization(
    pressure: f64,
    ambient_pressure: f64,
    volume: f64,
    conductance: f64,
    dt: f64,
    out_state: *mut AirlockDepressurization,
) -> Bool {
    if !finite(&[pressure, ambient_pressure, volume, conductance, dt])
        || volume <= 0.0
        || conductance < 0.0
        || dt < 0.0
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid airlock depressurization parameters",
        );
        return Bool::FALSE;
    }
    let rate = -conductance / volume * (pressure - ambient_pressure);
    write_out(
        out_state,
        AirlockDepressurization {
            pressure: ambient_pressure
                + (pressure - ambient_pressure) * (-conductance * dt / volume).exp(),
            pressure_rate: rate,
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let world = crate::rapier::world::world_create(Vec3::default());
        let builder = crate::rapier::rigid_body::rigid_body_builder_create(
            crate::rapier::ffi::BodyStatus::Dynamic as u32,
        );
        crate::rapier::rigid_body::rigid_body_builder_set_translation(
            builder,
            Vec3 {
                x: 7_000_000.0,
                y: 0.0,
                z: 0.0,
            },
        );
        crate::rapier::rigid_body::rigid_body_builder_set_linvel(
            builder,
            Vec3 {
                x: 7_500.0,
                y: 0.0,
                z: 0.0,
            },
        );
        crate::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder, 1.0);
        let body = crate::rapier::rigid_body::rigid_body_builder_build(builder);
        let handle = crate::rapier::rigid_body::world_insert_rigid_body(world, body);

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

        crate::rapier::world::world_step(world, 1.0 / 60.0);
        let velocity = crate::rapier::rigid_body::rigid_body_get_linvel(world, handle);
        assert!(velocity.x.is_finite());
        crate::rapier::world::world_destroy(world);
    }
}
