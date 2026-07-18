use rapier3d::prelude::Vector;

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, RigidBodyHandleRaw, TrajectoryEnvironment, TrajectoryForceReport,
    TrajectoryGlideEnvironment, TrajectoryGlideReport, TrajectoryGlideState, TrajectoryState,
    WorldHandle, unpack_rigid_body_handle, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};
use crate::rapier::math::mul_add;

const MAX_STEP_SECONDS: f64 = 10.0;
const MIN_GLIDE_SPEED: f64 = 1.0e-6;

fn environment_valid(env: TrajectoryEnvironment) -> bool {
    vec3_finite(env.gravity)
        && vec3_finite(env.flow_velocity)
        && vec3_finite(env.lift_direction)
        && env.mass.is_finite()
        && env.mass > 0.0
        && env.reference_area.is_finite()
        && env.reference_area >= 0.0
        && env.density.is_finite()
        && env.density >= 0.0
        && env.drag_coefficient.is_finite()
        && env.drag_coefficient >= 0.0
        && env.lift_coefficient.is_finite()
        && env.lift_coefficient >= 0.0
}

fn state_valid(state: TrajectoryState) -> bool {
    vec3_finite(state.position) && vec3_finite(state.velocity)
}

fn compute_forces(
    state: TrajectoryState,
    env: TrajectoryEnvironment,
) -> Option<TrajectoryForceReport> {
    if !state_valid(state) || !environment_valid(env) {
        return None;
    }

    let gravity_force = vec3_to_rapier(env.gravity) * env.mass;
    let relative_flow = vec3_to_rapier(env.flow_velocity) - vec3_to_rapier(state.velocity);
    let speed_squared = relative_flow.length_squared();
    let mut drag_force = Vector::ZERO;
    let mut lift_force = Vector::ZERO;

    if speed_squared > 1.0e-18 && env.reference_area > 0.0 && env.density > 0.0 {
        let speed = speed_squared.sqrt();
        let flow_dir = relative_flow / speed;
        let dynamic_pressure = 0.5 * env.density * speed_squared;
        drag_force = flow_dir * (dynamic_pressure * env.reference_area * env.drag_coefficient);

        if env.lift_coefficient > 0.0 {
            let lift_dir = vec3_to_rapier(env.lift_direction)
                .try_normalize()
                .unwrap_or(Vector::ZERO);
            if lift_dir.length_squared() > 0.0 {
                lift_force =
                    lift_dir * (dynamic_pressure * env.reference_area * env.lift_coefficient);
            }
        }
    }

    let total_force = gravity_force + drag_force + lift_force;
    let acceleration = total_force / env.mass;

    Some(TrajectoryForceReport {
        gravity_force: vec3_from_rapier(gravity_force),
        drag_force: vec3_from_rapier(drag_force),
        lift_force: vec3_from_rapier(lift_force),
        total_force: vec3_from_rapier(total_force),
        acceleration: vec3_from_rapier(acceleration),
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn trajectory_estimate_forces(
    state: TrajectoryState,
    env: TrajectoryEnvironment,
    out_report: *mut TrajectoryForceReport,
) -> Bool {
    let Some(report) = compute_forces(state, env) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid trajectory force parameters");
        return Bool::FALSE;
    };

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = report;
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn trajectory_integrate_step(
    state: TrajectoryState,
    env: TrajectoryEnvironment,
    dt: f64,
    out_state: *mut TrajectoryState,
    out_report: *mut TrajectoryForceReport,
) -> Bool {
    if !dt.is_finite() || dt <= 0.0 || dt > MAX_STEP_SECONDS {
        set_error(ERR_INVALID_ARGUMENT, "invalid trajectory timestep");
        return Bool::FALSE;
    }
    let Some(report) = compute_forces(state, env) else {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid trajectory integration parameters",
        );
        return Bool::FALSE;
    };

    let acceleration = vec3_to_rapier(report.acceleration);
    // Use mul_add for single-rounding precision when position >> velocity*dt
    let velocity = rapier3d::prelude::Vector::new(
        mul_add(acceleration.x, dt, vec3_to_rapier(state.velocity).x),
        mul_add(acceleration.y, dt, vec3_to_rapier(state.velocity).y),
        mul_add(acceleration.z, dt, vec3_to_rapier(state.velocity).z),
    );
    let position = rapier3d::prelude::Vector::new(
        mul_add(velocity.x, dt, vec3_to_rapier(state.position).x),
        mul_add(velocity.y, dt, vec3_to_rapier(state.position).y),
        mul_add(velocity.z, dt, vec3_to_rapier(state.position).z),
    );

    if let Some(out_state) = unsafe { out_state.as_mut() } {
        *out_state = TrajectoryState {
            position: vec3_from_rapier(position),
            velocity: vec3_from_rapier(velocity),
        };
    }
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = report;
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn trajectory_apply_forces_to_body(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    env: TrajectoryEnvironment,
    wake_up: Bool,
    out_report: *mut TrajectoryForceReport,
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

    let state = TrajectoryState {
        position: vec3_from_rapier(body.translation()),
        velocity: vec3_from_rapier(body.linvel()),
    };
    let Some(report) = compute_forces(state, env) else {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid trajectory body force parameters",
        );
        return Bool::FALSE;
    };

    body.add_force(vec3_to_rapier(report.total_force), wake_up.0 != 0);
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = report;
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn trajectory_apply_forces_to_body_flag(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    env: TrajectoryEnvironment,
    wake_up: Bool,
    out_report: *mut TrajectoryForceReport,
) -> u8 {
    trajectory_apply_forces_to_body(world, body_handle, env, wake_up, out_report).0
}

fn glide_state_valid(state: TrajectoryGlideState) -> bool {
    state.speed.is_finite()
        && state.speed >= 0.0
        && state.flight_path_angle.is_finite()
        && state.altitude.is_finite()
        && state.downrange.is_finite()
}

fn glide_environment_valid(env: TrajectoryGlideEnvironment) -> bool {
    env.gravity.is_finite()
        && env.gravity > 0.0
        && env.planet_radius.is_finite()
        && env.planet_radius > 0.0
        && env.ballistic_coefficient.is_finite()
        && env.ballistic_coefficient > 0.0
        && env.lift_to_drag.is_finite()
        && env.lift_to_drag >= 0.0
        && env.bank_angle.is_finite()
        && env.reference_density.is_finite()
        && env.reference_density >= 0.0
        && env.scale_height.is_finite()
        && env.scale_height > 0.0
}

fn glide_density(altitude: f64, env: TrajectoryGlideEnvironment) -> f64 {
    if env.reference_density == 0.0 {
        return 0.0;
    }
    env.reference_density * (-altitude.max(0.0) / env.scale_height).exp()
}

fn compute_glide_report(
    state: TrajectoryGlideState,
    env: TrajectoryGlideEnvironment,
) -> Option<TrajectoryGlideReport> {
    if !glide_state_valid(state) || !glide_environment_valid(env) {
        return None;
    }

    let speed = state.speed.max(MIN_GLIDE_SPEED);
    let radius = env.planet_radius + state.altitude;
    if radius <= 0.0 {
        return None;
    }

    let sin_gamma = state.flight_path_angle.sin();
    let cos_gamma = state.flight_path_angle.cos();
    let density = glide_density(state.altitude, env);
    let dynamic_pressure = 0.5 * density * speed * speed;
    let drag_acceleration = dynamic_pressure / env.ballistic_coefficient;
    let lift_acceleration = drag_acceleration * env.lift_to_drag;
    let radial_gravity = env.gravity * env.planet_radius * env.planet_radius / (radius * radius);
    let banked_lift = lift_acceleration * env.bank_angle.cos();

    let speed_dot = -drag_acceleration - radial_gravity * sin_gamma;
    let flight_path_angle_dot =
        banked_lift / speed + (speed / radius - radial_gravity / speed) * cos_gamma;
    let altitude_dot = speed * sin_gamma;
    let downrange_dot = env.planet_radius * speed * cos_gamma / radius;

    Some(TrajectoryGlideReport {
        density,
        dynamic_pressure,
        drag_acceleration,
        lift_acceleration,
        speed_dot,
        flight_path_angle_dot,
        altitude_dot,
        downrange_dot,
    })
}

fn add_glide_scaled(
    state: TrajectoryGlideState,
    report: TrajectoryGlideReport,
    scale: f64,
) -> TrajectoryGlideState {
    TrajectoryGlideState {
        speed: (state.speed + report.speed_dot * scale).max(0.0),
        flight_path_angle: state.flight_path_angle + report.flight_path_angle_dot * scale,
        altitude: state.altitude + report.altitude_dot * scale,
        downrange: state.downrange + report.downrange_dot * scale,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn trajectory_glide_estimate(
    state: TrajectoryGlideState,
    env: TrajectoryGlideEnvironment,
    out_report: *mut TrajectoryGlideReport,
) -> Bool {
    let Some(report) = compute_glide_report(state, env) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid trajectory glide parameters");
        return Bool::FALSE;
    };

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = report;
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn trajectory_glide_integrate_step(
    state: TrajectoryGlideState,
    env: TrajectoryGlideEnvironment,
    dt: f64,
    out_state: *mut TrajectoryGlideState,
    out_report: *mut TrajectoryGlideReport,
) -> Bool {
    if !dt.is_finite() || dt <= 0.0 || dt > MAX_STEP_SECONDS {
        set_error(ERR_INVALID_ARGUMENT, "invalid trajectory glide timestep");
        return Bool::FALSE;
    }

    let Some(k1) = compute_glide_report(state, env) else {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid trajectory glide integration parameters",
        );
        return Bool::FALSE;
    };
    let Some(k2) = compute_glide_report(add_glide_scaled(state, k1, dt * 0.5), env) else {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid trajectory glide midpoint parameters",
        );
        return Bool::FALSE;
    };
    let Some(k3) = compute_glide_report(add_glide_scaled(state, k2, dt * 0.5), env) else {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid trajectory glide midpoint parameters",
        );
        return Bool::FALSE;
    };
    let Some(k4) = compute_glide_report(add_glide_scaled(state, k3, dt), env) else {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid trajectory glide endpoint parameters",
        );
        return Bool::FALSE;
    };

    let out = TrajectoryGlideState {
        speed: mul_add(
            dt / 6.0,
            k1.speed_dot + 2.0 * k2.speed_dot + 2.0 * k3.speed_dot + k4.speed_dot,
            state.speed,
        )
        .max(0.0),
        flight_path_angle: mul_add(
            dt / 6.0,
            k1.flight_path_angle_dot
                + 2.0 * k2.flight_path_angle_dot
                + 2.0 * k3.flight_path_angle_dot
                + k4.flight_path_angle_dot,
            state.flight_path_angle,
        ),
        altitude: mul_add(
            dt / 6.0,
            k1.altitude_dot + 2.0 * k2.altitude_dot + 2.0 * k3.altitude_dot + k4.altitude_dot,
            state.altitude,
        ),
        downrange: mul_add(
            dt / 6.0,
            k1.downrange_dot + 2.0 * k2.downrange_dot + 2.0 * k3.downrange_dot + k4.downrange_dot,
            state.downrange,
        ),
    };

    if let Some(out_state) = unsafe { out_state.as_mut() } {
        *out_state = out;
    }
    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = k1;
    }
    clear_error();
    Bool::TRUE
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::Vec3;

    fn env() -> TrajectoryEnvironment {
        TrajectoryEnvironment {
            gravity: Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
            flow_velocity: Vec3::default(),
            mass: 2.0,
            reference_area: 0.1,
            density: 1.225,
            drag_coefficient: 0.5,
            lift_coefficient: 0.0,
            lift_direction: Vec3::default(),
        }
    }

    #[test]
    fn estimates_gravity_and_drag() {
        let mut report = TrajectoryForceReport::default();
        assert_eq!(
            trajectory_estimate_forces(
                TrajectoryState {
                    position: Vec3::default(),
                    velocity: Vec3 {
                        x: 10.0,
                        y: 0.0,
                        z: 0.0,
                    },
                },
                env(),
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(report.gravity_force.y < 0.0);
        assert!(report.drag_force.x < 0.0);
    }

    #[test]
    fn integrates_state_forward() {
        let mut out = TrajectoryState::default();
        assert_eq!(
            trajectory_integrate_step(
                TrajectoryState {
                    position: Vec3::default(),
                    velocity: Vec3 {
                        x: 10.0,
                        y: 0.0,
                        z: 0.0,
                    },
                },
                env(),
                0.1,
                &mut out,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert!(out.position.x > 0.0);
        assert!(out.velocity.y < 0.0);
    }

    fn glide_env() -> TrajectoryGlideEnvironment {
        TrajectoryGlideEnvironment {
            gravity: 9.80665,
            planet_radius: 6_371_000.0,
            ballistic_coefficient: 2_000.0,
            lift_to_drag: 1.5,
            bank_angle: 0.0,
            reference_density: 1.225,
            scale_height: 7_200.0,
        }
    }

    #[test]
    fn estimates_glide_derivatives() {
        let mut report = TrajectoryGlideReport::default();
        assert_eq!(
            trajectory_glide_estimate(
                TrajectoryGlideState {
                    speed: 3_000.0,
                    flight_path_angle: -0.05,
                    altitude: 40_000.0,
                    downrange: 0.0,
                },
                glide_env(),
                &mut report,
            ),
            Bool::TRUE
        );
        assert!(report.density > 0.0);
        assert!(report.drag_acceleration > 0.0);
        assert!(report.altitude_dot < 0.0);
        assert!(report.downrange_dot > 0.0);
    }

    #[test]
    fn integrates_glide_state_forward() {
        let mut out = TrajectoryGlideState::default();
        assert_eq!(
            trajectory_glide_integrate_step(
                TrajectoryGlideState {
                    speed: 3_000.0,
                    flight_path_angle: -0.05,
                    altitude: 40_000.0,
                    downrange: 0.0,
                },
                glide_env(),
                0.5,
                &mut out,
                std::ptr::null_mut(),
            ),
            Bool::TRUE
        );
        assert!(out.speed < 3_000.0);
        assert!(out.altitude < 40_000.0);
        assert!(out.downrange > 0.0);
    }
}
