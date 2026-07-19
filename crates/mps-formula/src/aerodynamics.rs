//! Aerodynamics — surface force and voxel aerodynamic formulas.
//!
//! Pure computation only — no access to `WorldHandle`, `RigidBody`, or Rapier state.

use crate::ffi::{
    AeroForceReport, AeroSurface, Vec3,
    vec3_finite, vec3_to_rapier, vec3_from_rapier,
};
use rapier3d::prelude::Vector;

fn aero_surface_valid(surface: &AeroSurface) -> bool {
    vec3_finite(surface.point)
        && vec3_finite(surface.normal)
        && surface.area.is_finite()
        && surface.drag_coefficient.is_finite()
        && surface.lift_coefficient.is_finite()
        && surface.area > 0.0
        && surface.drag_coefficient >= 0.0
        && surface.lift_coefficient >= 0.0
}

/// Compute the force and torque produced by a single aerodynamic surface.
pub fn compute_surface_force(
    surface: AeroSurface,
    body_linvel: Vec3,
    body_angvel: Vec3,
    body_center: Vec3,
    wind_velocity: Vec3,
    air_density: f64,
) -> Option<(Vec3, Vec3)> {
    if !aero_surface_valid(&surface) || !air_density.is_finite() || air_density < 0.0 {
        return None;
    }

    let point = vec3_to_rapier(surface.point);
    let normal = vec3_to_rapier(surface.normal);
    let Some(unit_normal) = normal.try_normalize() else {
        return None;
    };

    let body_center = vec3_to_rapier(body_center);
    let body_linvel = vec3_to_rapier(body_linvel);
    let body_angvel = vec3_to_rapier(body_angvel);

    let arm = point - body_center;
    let point_velocity = body_linvel + body_angvel.cross(arm);
    let relative_air = vec3_to_rapier(wind_velocity) - point_velocity;
    let speed_squared = relative_air.length_squared();
    if speed_squared <= 1.0e-18 {
        return None;
    }

    let speed = speed_squared.sqrt();
    let flow_dir = relative_air / speed;
    let exposure = flow_dir.dot(unit_normal).max(0.0);
    if exposure <= 0.0 {
        return None;
    }

    let dynamic_pressure = 0.5 * air_density * speed_squared;
    let effective_area = surface.area * exposure;
    let drag = flow_dir * (dynamic_pressure * effective_area * surface.drag_coefficient);
    let lift_axis = flow_dir.cross(unit_normal);
    let lift = lift_axis
        .try_normalize()
        .map(|axis| {
            let lift_dir = axis.cross(flow_dir);
            lift_dir * (dynamic_pressure * effective_area * surface.lift_coefficient)
        })
        .unwrap_or(rapier3d::prelude::Vector::ZERO);
    let force = drag + lift;

    Some((vec3_from_rapier(force), vec3_from_rapier(arm.cross(force))))
}

/// Estimate total surface force without modifying a body.
pub fn estimate_surface_force(
    body_linvel: Vec3,
    body_angvel: Vec3,
    body_center: Vec3,
    wind_velocity: Vec3,
    air_density: f64,
    surface: AeroSurface,
) -> Option<AeroForceReport> {
    let (force, torque) = compute_surface_force(
        surface,
        body_linvel,
        body_angvel,
        body_center,
        wind_velocity,
        air_density,
    )?;
    Some(AeroForceReport {
        total_force: force,
        total_torque: torque,
        surface_count: 1,
        active_surface_count: 1,
    })
}