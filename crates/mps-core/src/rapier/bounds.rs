use rapier3d::math::{Pose, Rotation, Vector};
use rapier3d::prelude::{ColliderBuilder, SharedShape};
use smallvec::SmallVec;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    Capsule, ColliderBuilderHandle, ColliderHandleRaw, Cylinder, Ellipsoid, MAX_OUTPUT_CAPACITY,
    Prism, QueryFilterDesc, SphericalShell, Ssv, WorldHandle, isometry_from_parts,
    pack_collider_handle, quat_finite, query_filter_from_desc, vec3_finite, vec3_to_rapier,
};

const EPSILON: f64 = 1.0e-9;

fn identity_pose() -> Pose {
    Pose::from_parts(Vector::ZERO, Rotation::IDENTITY)
}

fn valid_segment(a: Vector, b: Vector) -> bool {
    (b - a).length_squared() > EPSILON
}

pub(crate) fn capsule_shape(capsule: Capsule) -> Option<(Pose, SharedShape)> {
    if !vec3_finite(capsule.a) || !vec3_finite(capsule.b) || !capsule.radius.is_finite() {
        return None;
    }

    let a = vec3_to_rapier(capsule.a);
    let b = vec3_to_rapier(capsule.b);
    if capsule.radius <= 0.0 || !valid_segment(a, b) {
        return None;
    }

    Some((identity_pose(), SharedShape::capsule(a, b, capsule.radius)))
}

pub(crate) fn ssv_shape(ssv: Ssv) -> Option<(Pose, SharedShape)> {
    capsule_shape(Capsule {
        a: ssv.a,
        b: ssv.b,
        radius: ssv.radius,
    })
}

pub(crate) fn cylinder_shape(cylinder: Cylinder) -> Option<(Pose, SharedShape)> {
    if !vec3_finite(cylinder.center)
        || !quat_finite(cylinder.rotation)
        || !cylinder.radius.is_finite()
        || !cylinder.half_height.is_finite()
        || cylinder.radius <= 0.0
        || cylinder.half_height <= 0.0
    {
        return None;
    }

    Some((
        isometry_from_parts(cylinder.center, cylinder.rotation),
        SharedShape::cylinder(cylinder.half_height, cylinder.radius),
    ))
}

pub(crate) fn spherical_shell_shape(shell: SphericalShell) -> Option<(Pose, SharedShape)> {
    if !vec3_finite(shell.center)
        || !shell.outer_radius.is_finite()
        || !shell.inner_radius.is_finite()
        || shell.outer_radius <= 0.0
        || shell.inner_radius < 0.0
        || shell.inner_radius > shell.outer_radius
    {
        return None;
    }

    Some((
        isometry_from_parts(
            shell.center,
            crate::rapier::ffi::Quat {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
        ),
        SharedShape::ball(shell.outer_radius),
    ))
}

fn ellipsoid_points(ellipsoid: Ellipsoid) -> Option<SmallVec<[Vector; 128]>> {
    if !vec3_finite(ellipsoid.center)
        || !vec3_finite(ellipsoid.radii)
        || !quat_finite(ellipsoid.rotation)
        || ellipsoid.radii.x <= 0.0
        || ellipsoid.radii.y <= 0.0
        || ellipsoid.radii.z <= 0.0
    {
        return None;
    }

    let segments = ellipsoid.segments.clamp(8, 64) as usize;
    let rings = (segments / 2).max(4);
    let mut points = SmallVec::<[Vector; 128]>::with_capacity((rings - 1) * segments + 2);

    points.push(Vector::new(0.0, ellipsoid.radii.y, 0.0));
    points.push(Vector::new(0.0, -ellipsoid.radii.y, 0.0));

    for ring in 1..rings {
        let phi = std::f64::consts::PI * ring as f64 / rings as f64;
        let y = ellipsoid.radii.y * phi.cos();
        let ring_scale = phi.sin();
        for segment in 0..segments {
            let theta = std::f64::consts::TAU * segment as f64 / segments as f64;
            points.push(Vector::new(
                ellipsoid.radii.x * ring_scale * theta.cos(),
                y,
                ellipsoid.radii.z * ring_scale * theta.sin(),
            ));
        }
    }

    Some(points)
}

pub(crate) fn ellipsoid_shape(ellipsoid: Ellipsoid) -> Option<(Pose, SharedShape)> {
    let points = ellipsoid_points(ellipsoid)?;
    let shape = SharedShape::convex_hull(&points)?;
    Some((
        isometry_from_parts(ellipsoid.center, ellipsoid.rotation),
        shape,
    ))
}

fn prism_points(prism: Prism) -> Option<SmallVec<[Vector; 32]>> {
    if !vec3_finite(prism.center)
        || !quat_finite(prism.rotation)
        || !prism.radius.is_finite()
        || !prism.half_height.is_finite()
        || prism.radius <= 0.0
        || prism.half_height <= 0.0
        || prism.sides < 3
    {
        return None;
    }

    let sides = prism.sides.clamp(3, 128) as usize;
    let mut points = SmallVec::<[Vector; 32]>::with_capacity(sides * 2);
    for y in [-prism.half_height, prism.half_height] {
        for side in 0..sides {
            let theta = std::f64::consts::TAU * side as f64 / sides as f64;
            points.push(Vector::new(
                prism.radius * theta.cos(),
                y,
                prism.radius * theta.sin(),
            ));
        }
    }

    Some(points)
}

pub(crate) fn prism_shape(prism: Prism) -> Option<(Pose, SharedShape)> {
    let points = prism_points(prism)?;
    let shape = SharedShape::convex_hull(&points)?;
    Some((isometry_from_parts(prism.center, prism.rotation), shape))
}

fn builder_from_shape(shape: Option<(Pose, SharedShape)>) -> *mut ColliderBuilderHandle {
    let Some((pose, shape)) = shape else {
        set_error(ERR_INVALID_ARGUMENT, "invalid collider shape parameters");
        return std::ptr::null_mut();
    };

    clear_error();
    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: ColliderBuilder::new(shape).position(pose),
        voxel_source: None,
    }))
}

fn intersect_bound_count(
    world: *const WorldHandle,
    bound: Option<(Pose, SharedShape)>,
    filter: QueryFilterDesc,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    };
    let _query_lock = world.inner.query_lock.read();
    let Some((pose, shape)) = bound else {
        set_error(ERR_INVALID_ARGUMENT, "invalid bound shape parameters");
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    clear_error();
    query.intersect_shape(pose, shape.as_ref()).count() as u32
}

fn intersect_bound(
    world: *const WorldHandle,
    bound: Option<(Pose, SharedShape)>,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return 0;
    };
    let _query_lock = world.inner.query_lock.read();
    if out_handles.is_null() {
        set_error(ERR_NULL_POINTER, "output handle buffer is null");
        return 0;
    }
    if capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
        set_error(ERR_CAPACITY, "invalid output capacity");
        return 0;
    }
    let Some((pose, shape)) = bound else {
        set_error(ERR_INVALID_ARGUMENT, "invalid bound shape parameters");
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    let out = unsafe { std::slice::from_raw_parts_mut(out_handles, capacity as usize) };
    let mut written = 0usize;
    for (handle, _) in query.intersect_shape(pose, shape.as_ref()) {
        if written >= out.len() {
            break;
        }
        out[written] = pack_collider_handle(handle);
        written += 1;
    }

    clear_error();
    written as u32
}

/// # Safety
///
/// The returned builder is owned by the caller and must be consumed by
/// `collider_builder_build` or freed with `collider_builder_destroy`.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_capsule(capsule: Capsule) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        builder_from_shape(capsule_shape(capsule))
    })
}

/// # Safety
///
/// The returned builder is owned by the caller and must be consumed by
/// `collider_builder_build` or freed with `collider_builder_destroy`.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_ssv(ssv: Ssv) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || builder_from_shape(ssv_shape(ssv)))
}

/// # Safety
///
/// The returned builder is owned by the caller and must be consumed by
/// `collider_builder_build` or freed with `collider_builder_destroy`.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_ellipsoid(
    ellipsoid: Ellipsoid,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        builder_from_shape(ellipsoid_shape(ellipsoid))
    })
}

/// # Safety
///
/// The returned builder is owned by the caller and must be consumed by
/// `collider_builder_build` or freed with `collider_builder_destroy`.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_prism(prism: Prism) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        builder_from_shape(prism_shape(prism))
    })
}

/// # Safety
///
/// The returned builder is owned by the caller and must be consumed by
/// `collider_builder_build` or freed with `collider_builder_destroy`.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_cylinder(
    cylinder: Cylinder,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        builder_from_shape(cylinder_shape(cylinder))
    })
}

/// # Safety
///
/// The returned builder is owned by the caller and must be consumed by
/// `collider_builder_build` or freed with `collider_builder_destroy`.
#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_spherical_shell(
    shell: SphericalShell,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        builder_from_shape(spherical_shell_shape(shell))
    })
}

/// # Safety
///
/// `world` must be a valid world handle.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_capsule_count(
    world: *const WorldHandle,
    capsule: Capsule,
    filter: QueryFilterDesc,
) -> u32 {
    ffi_guard(0, || {
        intersect_bound_count(world, capsule_shape(capsule), filter)
    })
}

/// # Safety
///
/// `world` must be a valid world handle.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_capsule_count_all(
    world: *const WorldHandle,
    capsule: Capsule,
) -> u32 {
    ffi_guard(0, || {
        query_intersect_capsule_count(world, capsule, QueryFilterDesc::default())
    })
}

/// # Safety
///
/// `world` must be a valid world handle and `out_handles` must point to
/// writable space for at least `capacity` collider handles.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_capsule(
    world: *const WorldHandle,
    capsule: Capsule,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        intersect_bound(world, capsule_shape(capsule), filter, out_handles, capacity)
    })
}

/// # Safety
///
/// `world` must be a valid world handle and `out_handles` must point to
/// writable space for at least `capacity` collider handles.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_capsule_all(
    world: *const WorldHandle,
    capsule: Capsule,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        query_intersect_capsule(
            world,
            capsule,
            QueryFilterDesc::default(),
            out_handles,
            capacity,
        )
    })
}

/// # Safety
///
/// `world` must be a valid world handle.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_ssv_count(
    world: *const WorldHandle,
    ssv: Ssv,
    filter: QueryFilterDesc,
) -> u32 {
    ffi_guard(0, || intersect_bound_count(world, ssv_shape(ssv), filter))
}

/// # Safety
///
/// `world` must be a valid world handle.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_ssv_count_all(world: *const WorldHandle, ssv: Ssv) -> u32 {
    ffi_guard(0, || {
        query_intersect_ssv_count(world, ssv, QueryFilterDesc::default())
    })
}

/// # Safety
///
/// `world` must be a valid world handle and `out_handles` must point to
/// writable space for at least `capacity` collider handles.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_ssv(
    world: *const WorldHandle,
    ssv: Ssv,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        intersect_bound(world, ssv_shape(ssv), filter, out_handles, capacity)
    })
}

/// # Safety
///
/// `world` must be a valid world handle and `out_handles` must point to
/// writable space for at least `capacity` collider handles.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_ssv_all(
    world: *const WorldHandle,
    ssv: Ssv,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        query_intersect_ssv(
            world,
            ssv,
            QueryFilterDesc::default(),
            out_handles,
            capacity,
        )
    })
}

/// # Safety
///
/// `world` must be a valid world handle.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_ellipsoid_count(
    world: *const WorldHandle,
    ellipsoid: Ellipsoid,
    filter: QueryFilterDesc,
) -> u32 {
    ffi_guard(0, || {
        intersect_bound_count(world, ellipsoid_shape(ellipsoid), filter)
    })
}

/// # Safety
///
/// `world` must be a valid world handle.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_ellipsoid_count_all(
    world: *const WorldHandle,
    ellipsoid: Ellipsoid,
) -> u32 {
    ffi_guard(0, || {
        query_intersect_ellipsoid_count(world, ellipsoid, QueryFilterDesc::default())
    })
}

/// # Safety
///
/// `world` must be a valid world handle and `out_handles` must point to
/// writable space for at least `capacity` collider handles.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_ellipsoid(
    world: *const WorldHandle,
    ellipsoid: Ellipsoid,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        intersect_bound(
            world,
            ellipsoid_shape(ellipsoid),
            filter,
            out_handles,
            capacity,
        )
    })
}

/// # Safety
///
/// `world` must be a valid world handle and `out_handles` must point to
/// writable space for at least `capacity` collider handles.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_ellipsoid_all(
    world: *const WorldHandle,
    ellipsoid: Ellipsoid,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        query_intersect_ellipsoid(
            world,
            ellipsoid,
            QueryFilterDesc::default(),
            out_handles,
            capacity,
        )
    })
}

/// # Safety
///
/// `world` must be a valid world handle.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_prism_count(
    world: *const WorldHandle,
    prism: Prism,
    filter: QueryFilterDesc,
) -> u32 {
    ffi_guard(0, || {
        intersect_bound_count(world, prism_shape(prism), filter)
    })
}

/// # Safety
///
/// `world` must be a valid world handle.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_prism_count_all(world: *const WorldHandle, prism: Prism) -> u32 {
    ffi_guard(0, || {
        query_intersect_prism_count(world, prism, QueryFilterDesc::default())
    })
}

/// # Safety
///
/// `world` must be a valid world handle and `out_handles` must point to
/// writable space for at least `capacity` collider handles.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_prism(
    world: *const WorldHandle,
    prism: Prism,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        intersect_bound(world, prism_shape(prism), filter, out_handles, capacity)
    })
}

/// # Safety
///
/// `world` must be a valid world handle and `out_handles` must point to
/// writable space for at least `capacity` collider handles.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_prism_all(
    world: *const WorldHandle,
    prism: Prism,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        query_intersect_prism(
            world,
            prism,
            QueryFilterDesc::default(),
            out_handles,
            capacity,
        )
    })
}

/// # Safety
///
/// `world` must be a valid world handle.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_cylinder_count(
    world: *const WorldHandle,
    cylinder: Cylinder,
    filter: QueryFilterDesc,
) -> u32 {
    ffi_guard(0, || {
        intersect_bound_count(world, cylinder_shape(cylinder), filter)
    })
}

/// # Safety
///
/// `world` must be a valid world handle.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_cylinder_count_all(
    world: *const WorldHandle,
    cylinder: Cylinder,
) -> u32 {
    ffi_guard(0, || {
        query_intersect_cylinder_count(world, cylinder, QueryFilterDesc::default())
    })
}

/// # Safety
///
/// `world` must be a valid world handle and `out_handles` must point to
/// writable space for at least `capacity` collider handles.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_cylinder(
    world: *const WorldHandle,
    cylinder: Cylinder,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        intersect_bound(
            world,
            cylinder_shape(cylinder),
            filter,
            out_handles,
            capacity,
        )
    })
}

/// # Safety
///
/// `world` must be a valid world handle and `out_handles` must point to
/// writable space for at least `capacity` collider handles.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_cylinder_all(
    world: *const WorldHandle,
    cylinder: Cylinder,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        query_intersect_cylinder(
            world,
            cylinder,
            QueryFilterDesc::default(),
            out_handles,
            capacity,
        )
    })
}

/// # Safety
///
/// `world` must be a valid world handle.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_spherical_shell_count(
    world: *const WorldHandle,
    shell: SphericalShell,
    filter: QueryFilterDesc,
) -> u32 {
    ffi_guard(0, || {
        intersect_bound_count(world, spherical_shell_shape(shell), filter)
    })
}

/// # Safety
///
/// `world` must be a valid world handle.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_spherical_shell_count_all(
    world: *const WorldHandle,
    shell: SphericalShell,
) -> u32 {
    ffi_guard(0, || {
        query_intersect_spherical_shell_count(world, shell, QueryFilterDesc::default())
    })
}

/// # Safety
///
/// `world` must be a valid world handle and `out_handles` must point to
/// writable space for at least `capacity` collider handles.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_spherical_shell(
    world: *const WorldHandle,
    shell: SphericalShell,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        intersect_bound(
            world,
            spherical_shell_shape(shell),
            filter,
            out_handles,
            capacity,
        )
    })
}

/// # Safety
///
/// `world` must be a valid world handle and `out_handles` must point to
/// writable space for at least `capacity` collider handles.
#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_spherical_shell_all(
    world: *const WorldHandle,
    shell: SphericalShell,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        query_intersect_spherical_shell(
            world,
            shell,
            QueryFilterDesc::default(),
            out_handles,
            capacity,
        )
    })
}
