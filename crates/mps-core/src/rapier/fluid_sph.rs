//! SPH fluid-body FFI — Phase 1 of the fluid SPH roadmap.
//!
//! Thin compositional layer over the rapier fork's `FluidWorld` (see
//! `rapier/src/dynamics/fluid.rs` and `.hermes/plans/2026-08-30_fluid-sph-roadmap.md`).
//! Each fluid is stored in `PhysicsWorld.fluids` keyed by its `Vec` index; the
//! id returned to callers is that index (as `u32`). No new physics here — this
//! module only marshals FFI arguments to/from the fork state.

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{Bool, Vec3, WorldHandle, vec3_finite, vec3_from_rapier, vec3_to_rapier};
use rapier3d::prelude::fluid::{FluidParams, FluidWorld};
use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder, RigidBodyHandle, RigidBodyType};

/// Create an SPH fluid world and return its id (the `Vec` index in
/// `PhysicsWorld.fluids`). Returns `u32::MAX` on error.
///
/// * `gravity_x/y/z` — constant body acceleration (finite).
/// * `smoothing_radius` — SPH kernel cutoff `h` (`> 0`).
/// * `gas_constant` — equation-of-state stiffness `k` (`>= 0`, finite).
/// * `rest_density` — target density `ρ₀` (`> 0`).
/// * `viscosity` — dynamic viscosity `μ` (`>= 0`).
/// * `surface_tension` — cohesion coefficient `σ` (`>= 0`).
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn fluid_create(
    world: *mut WorldHandle,
    gravity_x: f64,
    gravity_y: f64,
    gravity_z: f64,
    smoothing_radius: f64,
    gas_constant: f64,
    rest_density: f64,
    viscosity: f64,
    surface_tension: f64,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "fluid_create: world is null");
            return u32::MAX;
        };
        if !vec3_finite(Vec3 {
            x: gravity_x,
            y: gravity_y,
            z: gravity_z,
        }) || !smoothing_radius.is_finite()
            || smoothing_radius <= 0.0
            || !gas_constant.is_finite()
            || gas_constant < 0.0
            || !rest_density.is_finite()
            || rest_density <= 0.0
            || !viscosity.is_finite()
            || viscosity < 0.0
            || !surface_tension.is_finite()
            || surface_tension < 0.0
        {
            set_error(ERR_INVALID_ARGUMENT, "fluid_create: bad parameters");
            return u32::MAX;
        }
        let params = FluidParams {
            smoothing_radius,
            gas_constant,
            rest_density,
            viscosity,
            surface_tension,
            gravity: vec3_to_rapier(Vec3 {
                x: gravity_x,
                y: gravity_y,
                z: gravity_z,
            }),
        };
        let id = world.inner.fluids.len() as u32;
        world.inner.fluids.push(FluidWorld::new(params));
        clear_error();
        id
    })
}

/// Append a particle to a fluid and return its particle index (`u32::MAX` on
/// error). `mass` must be `> 0`.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn fluid_add_particle(
    world: *mut WorldHandle,
    id: u32,
    x: f64,
    y: f64,
    z: f64,
    vx: f64,
    vy: f64,
    vz: f64,
    mass: f64,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "fluid_add_particle: world is null");
            return u32::MAX;
        };
        if !x.is_finite()
            || !y.is_finite()
            || !z.is_finite()
            || !vx.is_finite()
            || !vy.is_finite()
            || !vz.is_finite()
            || !mass.is_finite()
            || mass <= 0.0
        {
            set_error(ERR_INVALID_ARGUMENT, "fluid_add_particle: bad args");
            return u32::MAX;
        }
        let idx = id as usize;
        if idx >= world.inner.fluids.len() {
            set_error(ERR_NOT_FOUND, "fluid_add_particle: unknown id");
            return u32::MAX;
        }
        let pidx = world.inner.fluids[idx].add_particle(
            vec3_to_rapier(Vec3 { x, y, z }),
            vec3_to_rapier(Vec3 {
                x: vx,
                y: vy,
                z: vz,
            }),
            mass,
        );
        clear_error();
        pidx as u32
    })
}

/// Number of particles in a fluid (`u32::MAX` for an unknown id).
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn fluid_particle_count(world: *const WorldHandle, id: u32) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "fluid_particle_count: world is null");
            return u32::MAX;
        };
        let idx = id as usize;
        if idx >= world.inner.fluids.len() {
            set_error(ERR_NOT_FOUND, "fluid_particle_count: unknown id");
            return u32::MAX;
        }
        clear_error();
        world.inner.fluids[idx].len() as u32
    })
}

/// Read a particle's position/velocity/density into the out pointers (any of
/// which may be null to skip). Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be a valid world pointer; `out_*` must be null or point to
/// writable `Vec3` / `f64` space.
#[unsafe(no_mangle)]
pub extern "C" fn fluid_get_particle(
    world: *const WorldHandle,
    id: u32,
    index: u32,
    out_pos: *mut Vec3,
    out_vel: *mut Vec3,
    out_density: *mut f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "fluid_get_particle: world is null");
            return Bool::FALSE;
        };
        let idx = id as usize;
        if idx >= world.inner.fluids.len() {
            set_error(ERR_NOT_FOUND, "fluid_get_particle: unknown id");
            return Bool::FALSE;
        }
        let fluid = &world.inner.fluids[idx];
        let pidx = index as usize;
        if pidx >= fluid.len() {
            set_error(ERR_INVALID_ARGUMENT, "fluid_get_particle: bad index");
            return Bool::FALSE;
        }
        let p = &fluid.particles[pidx];
        if let Some(out) = unsafe { out_pos.as_mut() } {
            *out = vec3_from_rapier(p.pos);
        }
        if let Some(out) = unsafe { out_vel.as_mut() } {
            *out = vec3_from_rapier(p.vel);
        }
        if let Some(out) = unsafe { out_density.as_mut() } {
            *out = p.density;
        }
        clear_error();
        Bool::TRUE
    })
}

/// Advance a fluid by `dt` seconds (`> 0`). Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn fluid_step(world: *mut WorldHandle, id: u32, dt: f64) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "fluid_step: world is null");
            return Bool::FALSE;
        };
        if !dt.is_finite() || dt <= 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "fluid_step: bad dt");
            return Bool::FALSE;
        }
        let idx = id as usize;
        if idx >= world.inner.fluids.len() {
            set_error(ERR_NOT_FOUND, "fluid_step: unknown id");
            return Bool::FALSE;
        }
        world.inner.fluids[idx].step(dt);
        clear_error();
        Bool::TRUE
    })
}

/// Enable or disable rigid-body collision coupling for an SPH fluid.
///
/// When `enabled` is `Bool::TRUE`, one dynamic `Ball` collider (radius
/// `particle_radius`) is created per particle and registered in the world's
/// collision-proxy table (`fluid_proxies`); `world_step` then syncs particle
/// poses into these proxies before the rigid step and reads the contacted poses
/// back afterwards, so the fluid is blocked/stacked by terrain and other rigid
/// bodies (and by its own particles, maintaining incompressibility). When
/// `Bool::FALSE`, any existing proxies are removed.
///
/// Unlike soft-body proxies, fluid proxies keep the default (all-groups) collision
/// filter so particles collide with each other and with rigid bodies.
///
/// Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be a valid world pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn fluid_enable_collision(
    world: *mut WorldHandle,
    id: u32,
    particle_radius: f64,
    enabled: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "fluid_enable_collision: world is null");
            return Bool::FALSE;
        };
        let idx = id as usize;
        if idx >= world.inner.fluids.len() {
            set_error(ERR_NOT_FOUND, "fluid_enable_collision: unknown id");
            return Bool::FALSE;
        }
        if particle_radius <= 0.0 || !particle_radius.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "fluid_enable_collision: bad particle_radius",
            );
            return Bool::FALSE;
        }
        // Both branches tear the previous proxy set down first. Overwriting the
        // table entry (what the rebuild path used to do) orphaned every proxy
        // body + collider from the previous call, and those orphans kept
        // colliding with the scene forever.
        let remove_proxies = |world: &mut WorldHandle| {
            if let Some(proxies) = world.inner.fluid_proxies.remove(&id) {
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
        remove_proxies(world);
        if enabled == Bool::FALSE {
            clear_error();
            return Bool::TRUE;
        }
        // Build proxies for every particle.
        let mut proxies: Vec<Option<RigidBodyHandle>> =
            Vec::with_capacity(world.inner.fluids[idx].particles.len());
        for p in &world.inner.fluids[idx].particles {
            let rb = RigidBodyBuilder::new(RigidBodyType::Dynamic)
                .gravity_scale(0.0)
                .additional_mass(p.mass)
                .translation(p.pos)
                .linvel(p.vel)
                .build();
            let h = world.inner.bodies.insert(rb);
            let col = ColliderBuilder::ball(particle_radius).density(0.0).build();
            world
                .inner
                .colliders
                .insert_with_parent(col, h, &mut world.inner.bodies);
            proxies.push(Some(h));
        }
        world.inner.fluid_proxies.insert(id, proxies);
        clear_error();
        Bool::TRUE
    })
}
