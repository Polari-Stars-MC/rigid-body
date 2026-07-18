use rapier3d::prelude::Vector;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    AeroForceReport, AeroSurface, Bool, MAX_OUTPUT_CAPACITY, RigidBodyHandleRaw, Vec3, WorldHandle,
    unpack_rigid_body_handle, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};
use crate::rapier::math::KahanVec3;

fn aero_surface_valid(surface: AeroSurface) -> bool {
    vec3_finite(surface.point)
        && vec3_finite(surface.normal)
        && surface.area.is_finite()
        && surface.drag_coefficient.is_finite()
        && surface.lift_coefficient.is_finite()
        && surface.area > 0.0
        && surface.drag_coefficient >= 0.0
        && surface.lift_coefficient >= 0.0
}

fn compute_surface_force(
    surface: AeroSurface,
    body_linvel: Vector,
    body_angvel: Vector,
    body_center: Vector,
    wind_velocity: Vector,
    air_density: f64,
) -> Option<(Vector, Vector)> {
    if !aero_surface_valid(surface) || !air_density.is_finite() || air_density < 0.0 {
        return None;
    }

    let point = vec3_to_rapier(surface.point);
    let normal = vec3_to_rapier(surface.normal);
    #[allow(clippy::question_mark)]
    let Some(unit_normal) = normal.try_normalize() else {
        return None;
    };

    let arm = point - body_center;
    let point_velocity = body_linvel + body_angvel.cross(arm);
    let relative_air = wind_velocity - point_velocity;
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
        .unwrap_or(Vector::ZERO);
    let force = drag + lift;

    Some((force, arm.cross(force)))
}

fn voxel_index(size_x: usize, size_y: usize, x: usize, y: usize, z: usize) -> Option<usize> {
    z.checked_mul(size_x.checked_mul(size_y)?)?
        .checked_add(y.checked_mul(size_x)?)?
        .checked_add(x)
}

fn voxel_solid(
    voxels: &[u8],
    size_x: usize,
    size_y: usize,
    size_z: usize,
    x: isize,
    y: isize,
    z: isize,
) -> bool {
    if x < 0
        || y < 0
        || z < 0
        || x as usize >= size_x
        || y as usize >= size_y
        || z as usize >= size_z
    {
        return false;
    }

    voxel_index(size_x, size_y, x as usize, y as usize, z as usize)
        .and_then(|index| voxels.get(index))
        .is_some_and(|voxel| *voxel != 0)
}

fn make_report(
    total_force: Vector,
    total_torque: Vector,
    surface_count: u32,
    active_surface_count: u32,
) -> AeroForceReport {
    AeroForceReport {
        total_force: vec3_from_rapier(total_force),
        total_torque: vec3_from_rapier(total_torque),
        surface_count,
        active_surface_count,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn aero_apply_surfaces(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    wind_velocity: Vec3,
    air_density: f64,
    surfaces: *const AeroSurface,
    surface_count: u32,
    wake_up: Bool,
    out_report: *mut AeroForceReport,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    if surfaces.is_null() {
        set_error(ERR_NULL_POINTER, "aerodynamic surface input is null");
        return Bool::FALSE;
    }
    if surface_count == 0 || surface_count > MAX_OUTPUT_CAPACITY {
        set_error(ERR_CAPACITY, "invalid aerodynamic surface count");
        return Bool::FALSE;
    }
    if !vec3_finite(wind_velocity) || !air_density.is_finite() || air_density < 0.0 {
        return Bool::FALSE;
    }

    let surfaces = unsafe { std::slice::from_raw_parts(surfaces, surface_count as usize) };
    let Some(body) = world
        .inner
        .bodies
        .get_mut(unpack_rigid_body_handle(body_handle))
    else {
        return Bool::FALSE;
    };

    let body_linvel = body.linvel();
    let body_angvel = body.angvel();
    let body_center = body.center_of_mass();
    let wind_velocity = vec3_to_rapier(wind_velocity);
    let mut total_force = KahanVec3::default();
    let mut total_torque = KahanVec3::default();
    let mut active_surface_count = 0u32;

    for surface in surfaces {
        let Some((force, torque)) = compute_surface_force(
            *surface,
            body_linvel,
            body_angvel,
            body_center,
            wind_velocity,
            air_density,
        ) else {
            continue;
        };

        body.add_force_at_point(force, vec3_to_rapier(surface.point), wake_up.0 != 0);
        total_force.add(vec3_from_rapier(force));
        total_torque.add(vec3_from_rapier(torque));
        active_surface_count += 1;
    }

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = make_report(
            vec3_to_rapier(total_force.value()),
            vec3_to_rapier(total_torque.value()),
            surface_count,
            active_surface_count,
        );
    }

    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn aero_apply_voxel_grid(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    wind_velocity: Vec3,
    air_density: f64,
    voxels: *const u8,
    size_x: u32,
    size_y: u32,
    size_z: u32,
    voxel_size: f64,
    local_origin: Vec3,
    drag_coefficient: f64,
    lift_coefficient: f64,
    wake_up: Bool,
    out_report: *mut AeroForceReport,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    if voxels.is_null() {
        set_error(ERR_NULL_POINTER, "voxel input is null");
        return Bool::FALSE;
    }
    if size_x == 0 || size_y == 0 || size_z == 0 {
        set_error(ERR_CAPACITY, "voxel dimensions must be positive");
        return Bool::FALSE;
    }
    let Some(cell_count) = (size_x as usize)
        .checked_mul(size_y as usize)
        .and_then(|xy| xy.checked_mul(size_z as usize))
    else {
        set_error(ERR_CAPACITY, "voxel grid is too large");
        return Bool::FALSE;
    };
    if cell_count > MAX_OUTPUT_CAPACITY as usize {
        set_error(
            ERR_CAPACITY,
            "voxel grid exceeds maximum aerodynamic sample count",
        );
        return Bool::FALSE;
    }
    if !vec3_finite(wind_velocity)
        || !vec3_finite(local_origin)
        || !air_density.is_finite()
        || air_density < 0.0
        || !voxel_size.is_finite()
        || voxel_size <= 0.0
        || !drag_coefficient.is_finite()
        || drag_coefficient < 0.0
        || !lift_coefficient.is_finite()
        || lift_coefficient < 0.0
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid aerodynamic voxel parameters");
        return Bool::FALSE;
    }

    let voxels = unsafe { std::slice::from_raw_parts(voxels, cell_count) };
    let Some(body) = world
        .inner
        .bodies
        .get_mut(unpack_rigid_body_handle(body_handle))
    else {
        return Bool::FALSE;
    };

    let pose = *body.position();
    let body_linvel = body.linvel();
    let body_angvel = body.angvel();
    let body_center = body.center_of_mass();
    let wind_velocity = vec3_to_rapier(wind_velocity);
    let local_origin = vec3_to_rapier(local_origin);
    let face_area = voxel_size * voxel_size;
    let faces = [
        (
            -1isize,
            0isize,
            0isize,
            Vector::NEG_X,
            Vector::new(0.0, 0.5, 0.5),
        ),
        (1, 0, 0, Vector::X, Vector::new(1.0, 0.5, 0.5)),
        (0, -1, 0, Vector::NEG_Y, Vector::new(0.5, 0.0, 0.5)),
        (0, 1, 0, Vector::Y, Vector::new(0.5, 1.0, 0.5)),
        (0, 0, -1, Vector::NEG_Z, Vector::new(0.5, 0.5, 0.0)),
        (0, 0, 1, Vector::Z, Vector::new(0.5, 0.5, 1.0)),
    ];
    let mut total_force = KahanVec3::default();
    let mut total_torque = KahanVec3::default();
    let mut surface_count = 0u32;
    let mut active_surface_count = 0u32;

    for z in 0..size_z as usize {
        for y in 0..size_y as usize {
            for x in 0..size_x as usize {
                if !voxel_solid(
                    voxels,
                    size_x as usize,
                    size_y as usize,
                    size_z as usize,
                    x as isize,
                    y as isize,
                    z as isize,
                ) {
                    continue;
                }

                for (dx, dy, dz, local_normal, local_face_offset) in faces {
                    if voxel_solid(
                        voxels,
                        size_x as usize,
                        size_y as usize,
                        size_z as usize,
                        x as isize + dx,
                        y as isize + dy,
                        z as isize + dz,
                    ) {
                        continue;
                    }
                    surface_count += 1;
                    let local_point = local_origin
                        + (Vector::new(x as f64, y as f64, z as f64) + local_face_offset)
                            * voxel_size;
                    let world_point = pose * local_point;
                    let world_normal = pose.rotation * local_normal;
                    let surface = AeroSurface {
                        point: vec3_from_rapier(world_point),
                        normal: vec3_from_rapier(world_normal),
                        area: face_area,
                        drag_coefficient,
                        lift_coefficient,
                    };
                    let Some((force, torque)) = compute_surface_force(
                        surface,
                        body_linvel,
                        body_angvel,
                        body_center,
                        wind_velocity,
                        air_density,
                    ) else {
                        continue;
                    };

                    body.add_force_at_point(force, world_point, wake_up.0 != 0);
                    total_force.add(vec3_from_rapier(force));
                    total_torque.add(vec3_from_rapier(torque));
                    active_surface_count += 1;
                }
            }
        }
    }

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = make_report(
            vec3_to_rapier(total_force.value()),
            vec3_to_rapier(total_torque.value()),
            surface_count,
            active_surface_count,
        );
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn aero_apply_voxel_grid_flag(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    wind_velocity: Vec3,
    air_density: f64,
    voxels: *const u8,
    size_x: u32,
    size_y: u32,
    size_z: u32,
    voxel_size: f64,
    local_origin: Vec3,
    drag_coefficient: f64,
    lift_coefficient: f64,
    wake_up: Bool,
    out_report: *mut AeroForceReport,
) -> u8 {
    aero_apply_voxel_grid(
        world,
        body_handle,
        wind_velocity,
        air_density,
        voxels,
        size_x,
        size_y,
        size_z,
        voxel_size,
        local_origin,
        drag_coefficient,
        lift_coefficient,
        wake_up,
        out_report,
    )
    .0
}

#[unsafe(no_mangle)]
pub extern "C" fn aero_apply_surfaces_flag(
    world: *mut WorldHandle,
    body_handle: RigidBodyHandleRaw,
    wind_velocity: Vec3,
    air_density: f64,
    surfaces: *const AeroSurface,
    surface_count: u32,
    wake_up: Bool,
    out_report: *mut AeroForceReport,
) -> u8 {
    aero_apply_surfaces(
        world,
        body_handle,
        wind_velocity,
        air_density,
        surfaces,
        surface_count,
        wake_up,
        out_report,
    )
    .0
}

#[unsafe(no_mangle)]
pub extern "C" fn aero_estimate_surface_force(
    body_linvel: Vec3,
    body_angvel: Vec3,
    body_center: Vec3,
    wind_velocity: Vec3,
    air_density: f64,
    surface: AeroSurface,
    out_report: *mut AeroForceReport,
) -> Bool {
    if !vec3_finite(body_linvel)
        || !vec3_finite(body_angvel)
        || !vec3_finite(body_center)
        || !vec3_finite(wind_velocity)
        || !air_density.is_finite()
        || air_density < 0.0
    {
        return Bool::FALSE;
    }

    let Some((force, torque)) = compute_surface_force(
        surface,
        vec3_to_rapier(body_linvel),
        vec3_to_rapier(body_angvel),
        vec3_to_rapier(body_center),
        vec3_to_rapier(wind_velocity),
        air_density,
    ) else {
        if let Some(out_report) = unsafe { out_report.as_mut() } {
            *out_report = AeroForceReport {
                surface_count: 1,
                ..AeroForceReport::default()
            };
        }
        return Bool::TRUE;
    };

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = AeroForceReport {
            total_force: vec3_from_rapier(force),
            total_torque: vec3_from_rapier(torque),
            surface_count: 1,
            active_surface_count: 1,
        };
    }
    Bool::TRUE
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::BodyStatus;

    #[test]
    fn estimates_drag_force_from_exposed_surface() {
        let surface = AeroSurface {
            point: Vec3::default(),
            normal: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            area: 2.0,
            drag_coefficient: 1.0,
            lift_coefficient: 0.0,
        };
        let mut report = AeroForceReport::default();

        assert_eq!(
            aero_estimate_surface_force(
                Vec3::default(),
                Vec3::default(),
                Vec3::default(),
                Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
                1.2,
                surface,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(report.active_surface_count, 1);
        assert!((report.total_force.x - 120.0).abs() < 1.0e-9);
        assert_eq!(report.total_force.y, 0.0);
        assert_eq!(report.total_force.z, 0.0);
    }

    #[test]
    fn applies_aerodynamic_force_to_rapier_body() {
        let world = crate::rapier::world::world_create(Vec3::default());
        let builder =
            crate::rapier::rigid_body::rigid_body_builder_create(BodyStatus::Dynamic as u32);
        crate::rapier::rigid_body::rigid_body_builder_set_additional_mass(builder, 1.0);
        let body = crate::rapier::rigid_body::rigid_body_builder_build(builder);
        let body_handle = crate::rapier::rigid_body::world_insert_rigid_body(world, body);
        assert_ne!(body_handle, 0);

        let surfaces = [AeroSurface {
            point: Vec3::default(),
            normal: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            area: 1.0,
            drag_coefficient: 1.0,
            lift_coefficient: 0.0,
        }];
        let mut report = AeroForceReport::default();
        assert_eq!(
            aero_apply_surfaces(
                world,
                body_handle,
                Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
                1.0,
                surfaces.as_ptr(),
                surfaces.len() as u32,
                Bool::TRUE,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(report.active_surface_count, 1);
        assert!(report.total_force.x > 0.0);

        crate::rapier::world::world_step(world, 1.0 / 60.0);
        let velocity = crate::rapier::rigid_body::rigid_body_get_linvel(world, body_handle);
        assert!(velocity.x > 0.0);

        crate::rapier::world::world_destroy(world);
    }
}
