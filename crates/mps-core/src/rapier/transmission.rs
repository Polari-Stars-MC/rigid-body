use crate::rapier::error::{ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error};
use crate::rapier::ffi::{
    Bool, CamConstraintDesc, CamConstraintReport, GearConstraintDesc, GearConstraintReport,
    ScrewConstraintDesc, ScrewConstraintReport, SpiralConstraintDesc, SpiralConstraintReport, Vec3,
};

use crate::rapier::math::{finite, finite_non_negative, finite_positive};

const EPSILON: f64 = 1.0e-12;
const TAU: f64 = std::f64::consts::TAU;

fn wrap_tau(angle: f64) -> f64 {
    angle.rem_euclid(TAU)
}

fn apply_deadband(error: f64, deadband: f64) -> f64 {
    if deadband <= 0.0 || error.abs() > deadband {
        error
    } else {
        0.0
    }
}

fn cycloidal_lift(local_angle: f64, span: f64, lift: f64) -> (f64, f64) {
    if span <= EPSILON {
        return (lift, 0.0);
    }
    let u = (local_angle / span).clamp(0.0, 1.0);
    let displacement = lift * (u - (TAU * u).sin() / TAU);
    let derivative = lift / span * (1.0 - (TAU * u).cos());
    (displacement, derivative)
}

fn gear_valid(desc: GearConstraintDesc) -> bool {
    finite_positive(desc.ratio) && finite(desc.phase) && finite_non_negative(desc.backlash)
}

fn screw_valid(desc: ScrewConstraintDesc) -> bool {
    finite(desc.lead) && finite(desc.phase)
}

fn cam_valid(desc: CamConstraintDesc) -> bool {
    finite_non_negative(desc.base_radius)
        && finite_non_negative(desc.lift)
        && finite_non_negative(desc.rise_angle)
        && finite_non_negative(desc.return_angle)
        && finite(desc.phase)
        && desc.rise_angle + desc.return_angle <= TAU + 1.0e-9
}

fn spiral_valid(desc: SpiralConstraintDesc) -> bool {
    finite_non_negative(desc.initial_radius) && finite(desc.radial_pitch) && finite(desc.phase)
}

#[unsafe(no_mangle)]
pub extern "C" fn transmission_gear_evaluate(
    driver_angle: f64,
    driven_angle: f64,
    driver_angular_velocity: f64,
    driven_angular_velocity: f64,
    desc: GearConstraintDesc,
    out_report: *mut GearConstraintReport,
) -> Bool {
    if !finite(driver_angle)
        || !finite(driven_angle)
        || !finite(driver_angular_velocity)
        || !finite(driven_angular_velocity)
        || !gear_valid(desc)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid gear constraint parameters");
        return Bool::FALSE;
    }
    let sign = if desc.opposite_direction.0 != 0 {
        -1.0
    } else {
        1.0
    };
    let effective_ratio = sign * desc.ratio;
    let target_angle = desc.phase + effective_ratio * driver_angle;
    let target_angular_velocity = effective_ratio * driver_angular_velocity;
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "gear constraint output is null");
        return Bool::FALSE;
    };
    *out_report = GearConstraintReport {
        target_angle,
        target_angular_velocity,
        angle_error: apply_deadband(target_angle - driven_angle, desc.backlash),
        velocity_error: target_angular_velocity - driven_angular_velocity,
        effective_ratio,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn transmission_gear_target_angle(
    driver_angle: f64,
    ratio: f64,
    opposite_direction: Bool,
    phase: f64,
) -> f64 {
    if !finite(driver_angle) || !finite_positive(ratio) || !finite(phase) {
        return f64::NAN;
    }
    let sign = if opposite_direction.0 != 0 { -1.0 } else { 1.0 };
    phase + sign * ratio * driver_angle
}

#[unsafe(no_mangle)]
pub extern "C" fn transmission_screw_evaluate(
    screw_angle: f64,
    nut_translation: f64,
    screw_angular_velocity: f64,
    nut_linear_velocity: f64,
    desc: ScrewConstraintDesc,
    out_report: *mut ScrewConstraintReport,
) -> Bool {
    if !finite(screw_angle)
        || !finite(nut_translation)
        || !finite(screw_angular_velocity)
        || !finite(nut_linear_velocity)
        || !screw_valid(desc)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid screw constraint parameters");
        return Bool::FALSE;
    }
    let handedness = if desc.right_handed.0 != 0 { 1.0 } else { -1.0 };
    let meters_per_radian = handedness * desc.lead / TAU;
    let target_translation = desc.phase + meters_per_radian * screw_angle;
    let target_linear_velocity = meters_per_radian * screw_angular_velocity;
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "screw constraint output is null");
        return Bool::FALSE;
    };
    *out_report = ScrewConstraintReport {
        target_translation,
        target_linear_velocity,
        translation_error: target_translation - nut_translation,
        velocity_error: target_linear_velocity - nut_linear_velocity,
        meters_per_radian,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn transmission_screw_target_translation(
    screw_angle: f64,
    lead: f64,
    right_handed: Bool,
    phase: f64,
) -> f64 {
    if !finite(screw_angle) || !finite(lead) || !finite(phase) {
        return f64::NAN;
    }
    let handedness = if right_handed.0 != 0 { 1.0 } else { -1.0 };
    phase + handedness * lead * screw_angle / TAU
}

#[unsafe(no_mangle)]
pub extern "C" fn transmission_cycloidal_cam_evaluate(
    cam_angle: f64,
    follower_displacement: f64,
    cam_angular_velocity: f64,
    desc: CamConstraintDesc,
    out_report: *mut CamConstraintReport,
) -> Bool {
    if !finite(cam_angle)
        || !finite(follower_displacement)
        || !finite(cam_angular_velocity)
        || !cam_valid(desc)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid cam constraint parameters");
        return Bool::FALSE;
    }
    let angle = wrap_tau(cam_angle - desc.phase);
    let (displacement, derivative) = if angle <= desc.rise_angle {
        cycloidal_lift(angle, desc.rise_angle, desc.lift)
    } else if angle <= desc.rise_angle + desc.return_angle {
        let local = angle - desc.rise_angle;
        let (return_drop, return_derivative) = cycloidal_lift(local, desc.return_angle, desc.lift);
        (desc.lift - return_drop, -return_derivative)
    } else {
        (0.0, 0.0)
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "cam constraint output is null");
        return Bool::FALSE;
    };
    *out_report = CamConstraintReport {
        wrapped_angle: angle,
        radius: desc.base_radius + displacement,
        follower_displacement: displacement,
        displacement_derivative: derivative,
        target_velocity: derivative * cam_angular_velocity,
        displacement_error: displacement - follower_displacement,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn transmission_archimedean_spiral_evaluate(
    angle: f64,
    radial_position: f64,
    angular_velocity: f64,
    desc: SpiralConstraintDesc,
    out_report: *mut SpiralConstraintReport,
) -> Bool {
    if !finite(angle)
        || !finite(radial_position)
        || !finite(angular_velocity)
        || !spiral_valid(desc)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid spiral constraint parameters");
        return Bool::FALSE;
    }
    let local_angle = angle - desc.phase;
    let radius = desc.initial_radius + desc.radial_pitch * local_angle;
    if radius < 0.0 {
        set_error(ERR_INVALID_ARGUMENT, "spiral radius became negative");
        return Bool::FALSE;
    }
    let cos_theta = angle.cos();
    let sin_theta = angle.sin();
    let dx_dtheta = desc.radial_pitch * cos_theta - radius * sin_theta;
    let dy_dtheta = desc.radial_pitch * sin_theta + radius * cos_theta;
    let tangent_len = (dx_dtheta * dx_dtheta + dy_dtheta * dy_dtheta).sqrt();
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "spiral constraint output is null");
        return Bool::FALSE;
    };
    *out_report = SpiralConstraintReport {
        radius,
        position: Vec3 {
            x: radius * cos_theta,
            y: radius * sin_theta,
            z: 0.0,
        },
        tangent: if tangent_len > EPSILON {
            Vec3 {
                x: dx_dtheta / tangent_len,
                y: dy_dtheta / tangent_len,
                z: 0.0,
            }
        } else {
            Vec3::default()
        },
        radial_velocity: desc.radial_pitch * angular_velocity,
        constraint_error: radius - radial_position,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn transmission_archimedean_spiral_radius(
    angle: f64,
    initial_radius: f64,
    radial_pitch: f64,
    phase: f64,
) -> f64 {
    if !finite(angle)
        || !finite_non_negative(initial_radius)
        || !finite(radial_pitch)
        || !finite(phase)
    {
        return f64::NAN;
    }
    let radius = initial_radius + radial_pitch * (angle - phase);
    if radius >= 0.0 { radius } else { f64::NAN }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gear_constraint_supports_opposite_rotation() {
        let mut report = GearConstraintReport::default();
        assert_eq!(
            transmission_gear_evaluate(
                2.0,
                -4.1,
                3.0,
                -6.0,
                GearConstraintDesc {
                    ratio: 2.0,
                    phase: 0.0,
                    backlash: 0.0,
                    opposite_direction: Bool::TRUE,
                },
                &mut report
            ),
            Bool::TRUE
        );
        assert_eq!(report.target_angle, -4.0);
        assert!((report.angle_error - 0.1).abs() < 1.0e-12);
        assert_eq!(report.target_angular_velocity, -6.0);
    }

    #[test]
    fn screw_constraint_maps_rotation_to_translation() {
        let mut report = ScrewConstraintReport::default();
        assert_eq!(
            transmission_screw_evaluate(
                TAU,
                0.08,
                TAU * 2.0,
                0.2,
                ScrewConstraintDesc {
                    lead: 0.1,
                    phase: 0.0,
                    right_handed: Bool::TRUE,
                },
                &mut report
            ),
            Bool::TRUE
        );
        assert!((report.target_translation - 0.1).abs() < 1.0e-12);
        assert!((report.target_linear_velocity - 0.2).abs() < 1.0e-12);
    }

    #[test]
    fn cycloidal_cam_and_spiral_constraints_work() {
        let mut cam = CamConstraintReport::default();
        assert_eq!(
            transmission_cycloidal_cam_evaluate(
                std::f64::consts::FRAC_PI_2,
                0.0,
                2.0,
                CamConstraintDesc {
                    base_radius: 1.0,
                    lift: 0.5,
                    rise_angle: std::f64::consts::PI,
                    return_angle: std::f64::consts::PI,
                    phase: 0.0,
                },
                &mut cam
            ),
            Bool::TRUE
        );
        assert!(cam.follower_displacement > 0.0);
        assert!(cam.radius > 1.0);

        let mut spiral = SpiralConstraintReport::default();
        assert_eq!(
            transmission_archimedean_spiral_evaluate(
                std::f64::consts::FRAC_PI_2,
                1.0,
                3.0,
                SpiralConstraintDesc {
                    initial_radius: 1.0,
                    radial_pitch: 0.2,
                    phase: 0.0,
                },
                &mut spiral
            ),
            Bool::TRUE
        );
        assert!(spiral.radius > 1.0);
        assert!(spiral.radial_velocity > 0.0);
    }
}
