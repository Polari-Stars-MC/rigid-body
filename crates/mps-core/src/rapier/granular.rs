//! Granular-body (DEM) FFI — Phase 1 of the granular roadmap.
//!
//! Thin compositional layer over the rapier fork's `GranularWorld` (see
//! `rapier/src/dynamics/granular.rs`). Each granular body is stored in
//! `PhysicsWorld.granular_bodies` keyed by its `Vec` index; the world steps
//! every cloud automatically inside `world_step` (mirroring `fluids`), so the
//! standalone `granular_step` is an optional manual-tick hook.

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{Bool, Vec3, WorldHandle, vec3_finite};
use rapier3d::prelude::granular::{GranularParams, GranularWorld};

/// Create a DEM granular body and return its id (the `Vec` index in
/// `PhysicsWorld.granular_bodies`). Returns `u32::MAX` on error.
///
/// `gravity` is the body acceleration for every particle (typically the
/// world's gravity, so a granular pile falls like everything else).
///
/// # Safety
///
/// `world` must be a valid world pointer or null.
#[unsafe(no_mangle)]
pub extern "C" fn granular_create(
    world: *mut WorldHandle,
    gravity: Vec3,
    particle_radius: f64,
    normal_stiffness: f64,
    normal_damping: f64,
    friction: f64,
    tangential_damping: f64,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "granular_create: world is null");
            return u32::MAX;
        };
        if !vec3_finite(gravity)
            || !particle_radius.is_finite()
            || particle_radius <= 0.0
            || !normal_stiffness.is_finite()
            || normal_stiffness <= 0.0
            || !normal_damping.is_finite()
            || normal_damping < 0.0
            || !friction.is_finite()
            || friction < 0.0
            || !tangential_damping.is_finite()
            || !(0.0..=1.0).contains(&tangential_damping)
        {
            set_error(ERR_INVALID_ARGUMENT, "granular_create: bad parameters");
            return u32::MAX;
        }
        let params = GranularParams {
            normal_stiffness,
            normal_damping,
            friction,
            tangential_damping,
            gravity: crate::rapier::ffi::vec3_to_rapier(gravity),
        };
        clear_error();
        let id = world.inner.granular_bodies.len() as u32;
        world.inner.granular_bodies.push(GranularWorld::new(params));
        id
    })
}

/// Append a particle to a granular body. Returns the particle index or
/// `u32::MAX` on error.
///
/// # Safety
///
/// `world` must be a valid world pointer or null.
#[unsafe(no_mangle)]
pub extern "C" fn granular_add_particle(
    world: *mut WorldHandle,
    id: u32,
    x: f64,
    y: f64,
    z: f64,
    vx: f64,
    vy: f64,
    vz: f64,
    mass: f64,
    radius: f64,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "granular_add_particle: world is null");
            return u32::MAX;
        };
        let pos = Vec3 { x, y, z };
        let vel = Vec3 {
            x: vx,
            y: vy,
            z: vz,
        };
        if !vec3_finite(pos)
            || !vec3_finite(vel)
            || !mass.is_finite()
            || mass <= 0.0
            || !radius.is_finite()
            || radius <= 0.0
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "granular_add_particle: bad parameters",
            );
            return u32::MAX;
        }
        let Some(g) = world.inner.granular_bodies.get_mut(id as usize) else {
            set_error(ERR_NOT_FOUND, "granular_add_particle: unknown id");
            return u32::MAX;
        };
        clear_error();
        g.add_particle(
            crate::rapier::ffi::vec3_to_rapier(pos),
            crate::rapier::ffi::vec3_to_rapier(vel),
            mass,
            radius,
        ) as u32
    })
}

/// Number of particles in a granular body. Returns `u32::MAX` for an unknown
/// id.
///
/// # Safety
///
/// `world` must be a valid world pointer or null.
#[unsafe(no_mangle)]
pub extern "C" fn granular_particle_count(world: *const WorldHandle, id: u32) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "granular_particle_count: world is null");
            return u32::MAX;
        };
        let Some(g) = world.inner.granular_bodies.get(id as usize) else {
            set_error(ERR_NOT_FOUND, "granular_particle_count: unknown id");
            return u32::MAX;
        };
        clear_error();
        g.len() as u32
    })
}

/// Batch-read granular particle positions + velocities into `out_pos` /
/// `out_vel` (each with `capacity` slots). Either out-pointer may be null to
/// skip that channel. Returns the real particle count (callers retry with a
/// bigger buffer when `capacity` is short). Null world / unknown id → `0`.
///
/// # Safety
///
/// `world` must be a valid world pointer or null; `out_pos` / `out_vel` must
/// be null or point to writable memory for `capacity` values each.
#[unsafe(no_mangle)]
pub extern "C" fn granular_read_particles(
    world: *const WorldHandle,
    id: u32,
    out_pos: *mut Vec3,
    out_vel: *mut Vec3,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "granular_read_particles: world is null");
            return 0;
        };
        let Some(g) = world.inner.granular_bodies.get(id as usize) else {
            set_error(ERR_NOT_FOUND, "granular_read_particles: unknown id");
            return 0;
        };
        let n = g.len();
        let cap = capacity as usize;
        if !out_pos.is_null() {
            let dst = unsafe { std::slice::from_raw_parts_mut(out_pos, cap.min(n)) };
            for (dst, src) in dst.iter_mut().zip(g.particles.iter()) {
                *dst = crate::rapier::ffi::vec3_from_rapier(src.pos);
            }
        }
        if !out_vel.is_null() {
            let dst = unsafe { std::slice::from_raw_parts_mut(out_vel, cap.min(n)) };
            for (dst, src) in dst.iter_mut().zip(g.particles.iter()) {
                *dst = crate::rapier::ffi::vec3_from_rapier(src.vel);
            }
        }
        clear_error();
        n as u32
    })
}

/// Manually advance one granular body by `dt`. `world_step` already ticks
/// every granular body — this is for callers that want a custom substep loop.
///
/// # Safety
///
/// `world` must be a valid world pointer or null.
#[unsafe(no_mangle)]
pub extern "C" fn granular_step(world: *mut WorldHandle, id: u32, dt: f64) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "granular_step: world is null");
            return Bool::FALSE;
        };
        if !dt.is_finite() || dt <= 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "granular_step: bad dt");
            return Bool::FALSE;
        }
        let Some(g) = world.inner.granular_bodies.get_mut(id as usize) else {
            set_error(ERR_NOT_FOUND, "granular_step: unknown id");
            return Bool::FALSE;
        };
        g.step(dt);
        clear_error();
        Bool::TRUE
    })
}

/// Link voxel digging to grain spawning: from now on, digging a solid cell
/// out of any voxel collider (`collider_voxel_edit` with `solid = 0`, or a
/// `soft_body_voxel_dig` that propagates to the collider grid) spawns one
/// grain of `grain_mass` / `grain_radius` at the cell's world centre into the
/// granular body `dig_grain_body`. Pass `dig_grain_body = u32::MAX` to unlink.
///
/// Returns `Bool::FALSE` (and changes nothing) when `dig_grain_body` is not
/// `u32::MAX` and does not name an existing granular body.
///
/// # Safety
///
/// `world` must be a valid world pointer or null.
#[unsafe(no_mangle)]
pub extern "C" fn granular_link_voxel_dig(
    world: *mut WorldHandle,
    dig_grain_body: u32,
    grain_mass: f64,
    grain_radius: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "granular_link_voxel_dig: world is null");
            return Bool::FALSE;
        };
        let unlink = dig_grain_body == u32::MAX;
        if !unlink
            && (!grain_mass.is_finite()
                || grain_mass <= 0.0
                || !grain_radius.is_finite()
                || grain_radius <= 0.0)
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "granular_link_voxel_dig: bad grain params",
            );
            return Bool::FALSE;
        }
        if !unlink
            && world
                .inner
                .granular_bodies
                .get(dig_grain_body as usize)
                .is_none()
        {
            set_error(
                ERR_NOT_FOUND,
                "granular_link_voxel_dig: unknown dig_grain_body",
            );
            return Bool::FALSE;
        }
        world.inner.granular_dig_spawn = if unlink {
            None
        } else {
            Some((dig_grain_body, grain_mass, grain_radius))
        };
        clear_error();
        Bool::TRUE
    })
}

/// Query the current voxel-dig → grain-spawn link. Returns `Bool::TRUE` when
/// linked (writing the body id / mass / radius through the non-null out
/// pointers), `Bool::FALSE` when unlinked or the world is null.
///
/// # Safety
///
/// All out pointers may be null; `world` must be a valid world pointer or null.
#[unsafe(no_mangle)]
pub extern "C" fn granular_get_voxel_dig_link(
    world: *const WorldHandle,
    out_body: *mut u32,
    out_mass: *mut f64,
    out_radius: *mut f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(
                ERR_NULL_POINTER,
                "granular_get_voxel_dig_link: world is null",
            );
            return Bool::FALSE;
        };
        let Some((id, mass, radius)) = world.inner.granular_dig_spawn else {
            clear_error();
            return Bool::FALSE;
        };
        if !out_body.is_null() {
            unsafe { *out_body = id };
        }
        if !out_mass.is_null() {
            unsafe { *out_mass = mass };
        }
        if !out_radius.is_null() {
            unsafe { *out_radius = radius };
        }
        clear_error();
        Bool::TRUE
    })
}

/// Enable/disable rigid-body collision coupling for a granular body (Phase 38,
/// mirroring `fluid_enable_collision`): when enabled, every particle gets a
/// dynamic proxy `RigidBody` (gravity_scale 0 — the DEM integrator applies
/// gravity itself) plus a `Ball` collider of `particle_radius`. `world_step`
/// then syncs particle poses into the proxies before the rigid step, reads the
/// contacted poses back after it, and only then runs the DEM integrator — so
/// grains pile up on voxel terrain / rigid bodies instead of falling through.
///
/// Enabling again re-syncs the proxy set to the current particle count (new
/// particles get proxies). Disabling destroys the proxies; the particles keep
/// their last synced poses.
///
/// # Safety
///
/// `world` must be a valid world pointer or null.
#[unsafe(no_mangle)]
pub extern "C" fn granular_enable_collision(
    world: *mut WorldHandle,
    id: u32,
    particle_radius: f64,
    enabled: Bool,
) -> Bool {
    use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder, RigidBodyType};
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "granular_enable_collision: world is null");
            return Bool::FALSE;
        };
        let Some(g) = world.inner.granular_bodies.get(id as usize) else {
            set_error(ERR_NOT_FOUND, "granular_enable_collision: unknown id");
            return Bool::FALSE;
        };
        let body_len = g.len();
        if particle_radius <= 0.0 || !particle_radius.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "granular_enable_collision: bad radius",
            );
            return Bool::FALSE;
        }
        // Every path that (re)installs proxies tears the previous set down
        // first. Overwriting the table entry — which is what the rebuild path
        // used to do — orphaned a full set of dynamic bodies + colliders per
        // call, and granular_enable_collision is documented as the way to
        // re-sync after adding particles, so the leak grew on every append.
        let remove_proxies = |world: &mut WorldHandle| {
            if let Some(proxies) = world.inner.granular_proxies.remove(&id) {
                for ph in proxies.into_iter().flatten() {
                    world.inner.bodies.remove(
                        ph,
                        &mut world.inner.islands,
                        &mut world.inner.colliders,
                        &mut world.inner.impulse_joints,
                        &mut world.inner.multibody_joints,
                        &mut world.inner.upstream_soft_bodies,
                        false,
                    );
                }
            }
        };
        if enabled == Bool::FALSE {
            remove_proxies(world);
            clear_error();
            return Bool::TRUE;
        }
        remove_proxies(world);
        // (Re)build proxies for every particle. `world.inner.granular_bodies`
        // borrow ends before `bodies` is mutated (snapshot the poses first).
        let snapshot: Vec<(rapier3d::math::Vector, rapier3d::math::Vector, f64)> =
            world.inner.granular_bodies[id as usize]
                .particles
                .iter()
                .map(|p| (p.pos, p.vel, p.mass))
                .collect();
        let mut proxies: Vec<Option<rapier3d::prelude::RigidBodyHandle>> =
            Vec::with_capacity(body_len);
        for (pos, vel, mass) in snapshot {
            let rb = RigidBodyBuilder::new(RigidBodyType::Dynamic)
                .gravity_scale(0.0)
                .additional_mass(mass)
                .translation(pos)
                .linvel(vel)
                .build();
            let h = world.inner.bodies.insert(rb);
            let col = ColliderBuilder::ball(particle_radius).density(0.0).build();
            world
                .inner
                .colliders
                .insert_with_parent(col, h, &mut world.inner.bodies);
            proxies.push(Some(h));
        }
        world.inner.granular_proxies.insert(id, proxies);
        clear_error();
        Bool::TRUE
    })
}
