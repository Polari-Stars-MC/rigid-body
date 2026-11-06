//! Skeletal soft-body support (Phase 1, route A) — composition layer.
//!
//! A **skeletal soft body** is a chain/tree of rigid-body "nodes" connected by
//! spring joints. This is the cheapest soft-body formulation: it reuses the
//! existing rigid-body solver + impulse-joint machinery unchanged, and expresses
//! softness entirely through joint springs (`stiffness`/`damping`). No new physics
//! is invented — we only *compose* the already-exposed FFI primitives:
//!
//! * `rigid_body_builder_create` / `rigid_body_builder_build` / `world_insert_rigid_body`
//! * `collider_builder_create_sphere` / `collider_builder_build` / `world_insert_collider_with_parent`
//! * `joint_builder_create(Spring, axis, k, c)` / `world_insert_impulse_joint`
//!
//! ## Why this is safe w.r.t. the fork's invariants
//!
//! * **No SoA boundary touched.** Every node is a normal rigid body; springs are
//!   normal impulse joints. The SIMD solver buffers (`velocities`/`accelerations`)
//!   are untouched.
//! * **bit-identical friendly.** We introduce no new floating-point numerics —
//!   the spring force is computed inside rapier's own joint solver, which already
//!   respects `enhanced-determinism`.
//! * **Reuses `force_containers` indirectly.** Springs are persistent joint
//!   constraints, so they survive across steps like any other joint (no per-step
//!   re-application needed), mirroring the `Persistent` force lifecycle.
//!
//! Phase 2 (mass-spring) and Phase 3 (deformable mesh) layer on top of this with
//! point-mass + `SoftBody` structures; this module is the route-A entry point.

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INTERNAL, ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, ERR_OK,
    clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    Bool, RigidBodyHandleRaw, Sphere, Vec3, WorldHandle, pack_rigid_body_handle,
    unpack_rigid_body_handle, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};
use rapier3d::dynamics::soft_body::TearCriterion;
use rapier3d::math::{Rotation, Vector};
use rapier3d::prelude::soft_body::{
    CohesionParams, PlasticityParams, SelfCollisionParams, ThermalParams, ViscoelasticParams, Wind,
};
use rapier3d::prelude::soft_body::{SoftBody, SoftBodyId, SoftSolver};
use rapier3d::prelude::{
    ColliderBuilder, ColliderHandle, Group, InteractionGroups, RigidBodyBuilder, RigidBodyHandle,
    RigidBodyType,
};
use std::collections::HashSet;

/// Per-particle linear-blend-skinning (LBS) binding. A particle may be influenced
/// by up to 4 bones; `weight` sums to 1. `local[k]` is the particle's rest
/// position expressed in bone `bone_idx[k]`'s bind-pose frame (an offset vector),
/// precomputed at bind time, so the per-step skinning is just
/// `Σ w_k · (boneRot_k · local_k + boneTrans_k)`.
#[derive(Clone)]
pub(crate) struct VertexSkin {
    pub bone_idx: [u32; 4],
    pub weight: [f64; 4],
    pub local: [Vector; 4],
}

impl Default for VertexSkin {
    fn default() -> Self {
        Self {
            bone_idx: [0; 4],
            weight: [0.0; 4],
            local: [Vector::ZERO; 4],
        }
    }
}

/// A skeleton bound to one soft body: the list of bone rigid bodies, each bone's
/// bind-pose world translation/rotation, and the per-particle bindings.
pub(crate) struct SkeletonBinding {
    pub bones: Vec<RigidBodyHandle>,
    pub bind_translation: Vec<Vector>,
    pub bind_rotation: Vec<Rotation>,
    pub vertices: std::collections::HashMap<u32, VertexSkin>,
}

/// Phase 5d bookkeeping: which voxel cell maps to which soft-body particle.
///
/// Built by [`soft_body_voxel_build`] alongside the body, then consulted/updated
/// by [`soft_body_voxel_dig`] so a dug-out cell can be mapped back to the exact
/// particle to remove. `map[cell_linear] == particle_index` (or `-1` if the cell
/// was empty or already dug). After each dig the map is rebuilt to mirror
/// `SoftBody::remove_particle`'s index shift (every surviving index `> removed`
/// is decremented).
pub(crate) struct VoxelSoftMeta {
    pub sx: usize,
    pub sy: usize,
    pub sz: usize,
    /// World-space origin of the soft body's voxel grid (cell (0,0,0) center is
    /// `origin + 0.5 * voxel_size`). Needed by Phase 5g to map a dug collider
    /// cell (in a different grid's coordinate space) back to this soft body's
    /// cell coordinates via world-space overlap.
    pub origin: Vec3,
    /// Uniform voxel edge length of the soft body's grid (matching the
    /// `voxel_size` argument of `soft_body_voxel_build`).
    pub voxel_size: f64,
    pub map: Vec<i64>,
}

const JOINT_TYPE_SPRING: u32 = 4;

/// Construct a rigid-body node (sphere) and insert it into the world.
///
/// Returns the packed handle, or `0` (`ERR_*`) on failure. `is_anchor` nodes are
/// created `fixed` so the chain can hang from them.
///
/// # Safety
/// `world` must be a valid world pointer.
unsafe fn spawn_node(
    world: &mut WorldHandle,
    position: Vec3,
    mass: f64,
    radius: f64,
    fixed: bool,
) -> RigidBodyHandleRaw {
    let builder = rapier3d::prelude::RigidBodyBuilder::new(if fixed {
        rapier3d::prelude::RigidBodyType::Fixed
    } else {
        rapier3d::prelude::RigidBodyType::Dynamic
    })
    .translation(vec3_to_rapier(position))
    .additional_mass(mass);

    let body = Box::into_raw(Box::new(builder.build()));
    let body_handle =
        crate::rapier::rigid_body::world_insert_rigid_body(world as *mut WorldHandle, body);
    if body_handle == 0 {
        return 0;
    }

    // Attach a sphere collider so the node participates in collision.
    let sphere = Sphere {
        center: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        radius,
    };
    let collider_builder = crate::rapier::collider::collider_builder_create_sphere(sphere);
    if collider_builder.is_null() {
        return body_handle; // body is fine even without a collider
    }
    let collider = crate::rapier::collider::collider_builder_build(collider_builder);
    if !collider.is_null() {
        crate::rapier::collider::world_insert_collider_with_parent(
            world as *mut WorldHandle,
            collider,
            body_handle,
        );
    }
    body_handle
}

/// Create a spring joint between two bodies and insert it into the world.
///
/// # Safety
/// `world` must be a valid world pointer; `body1`/`body2` must be valid handles.
unsafe fn link_spring(
    world: &mut WorldHandle,
    body1: RigidBodyHandleRaw,
    body2: RigidBodyHandleRaw,
    axis: Vec3,
    stiffness: f64,
    damping: f64,
    rest_length: f64,
) -> Bool {
    let builder =
        crate::rapier::joints::joint_builder_create(JOINT_TYPE_SPRING, axis, stiffness, damping);
    if builder.is_null() {
        return Bool::FALSE;
    }
    // Spring joint rest length is configured via the joint's limits/length; we set
    // the motor target to `rest_length` so the spring centers there.
    crate::rapier::joints::joint_builder_set_motor_position(
        builder,
        0, // primary axis
        rest_length,
        stiffness,
        damping,
    );
    let _ = crate::rapier::joints::world_insert_impulse_joint(
        world as *mut WorldHandle,
        body1,
        body2,
        builder,
        Bool::TRUE,
    );
    Bool::TRUE
}

/// Create a skeletal soft body as a chain (line) of spring-linked rigid nodes.
///
/// Nodes are placed `spacing` apart along `axis` starting at the world origin (or
/// at `anchor` if `anchor != 0`). Adjacent nodes are joined by a spring joint with
/// the given `stiffness`/`damping`; the spring's rest length is `spacing`, so the
/// chain behaves like a soft rope / articulated strand.
///
/// # Parameters
/// * `node_count` — number of nodes (must be ≥ 1).
/// * `spacing` — distance between adjacent nodes / spring rest length (> 0).
/// * `node_mass` — mass of each node (> 0).
/// * `node_radius` — collision sphere radius of each node (> 0).
/// * `anchor` — `RigidBodyHandleRaw` to pin the first node to (0 = first node is a
///   free/fixed root at the origin; pass a valid handle to hang from it).
/// * `axis` — unit direction of the chain (need not be normalized; it is normalized
///   internally; must be finite and non-zero).
/// * `stiffness` / `damping` — spring coefficients (≥ 0).
///
/// # Returns
/// The number of nodes successfully created (0 on error). On partial failure the
/// already-created nodes/joints remain in the world (caller may clear the world).
///
/// # Safety
/// `world` must be a valid world pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn soft_chain_create(
    world: *mut WorldHandle,
    node_count: u32,
    spacing: f64,
    node_mass: f64,
    node_radius: f64,
    anchor: RigidBodyHandleRaw,
    axis: Vec3,
    stiffness: f64,
    damping: f64,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if node_count == 0
            || !spacing.is_finite()
            || spacing <= 0.0
            || !node_mass.is_finite()
            || node_mass <= 0.0
            || !node_radius.is_finite()
            || node_radius <= 0.0
            || !vec3_finite(axis)
            || !stiffness.is_finite()
            || stiffness < 0.0
            || !damping.is_finite()
            || damping < 0.0
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_chain_create: invalid parameters",
            );
            return 0;
        }

        let axis_v = vec3_to_rapier(axis);
        let len = axis_v.length();
        if len <= 1e-9 {
            set_error(ERR_INVALID_ARGUMENT, "soft_chain_create: zero axis");
            return 0;
        }
        let dir = axis_v / len;

        let origin = if anchor == 0 {
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }
        } else {
            match world.inner.bodies.get(unpack_rigid_body_handle(anchor)) {
                Some(b) => vec3_from_rapier(b.translation()),
                None => {
                    set_error(ERR_INVALID_ARGUMENT, "soft_chain_create: bad anchor");
                    return 0;
                }
            }
        };

        let mut handles: Vec<RigidBodyHandleRaw> = Vec::with_capacity(node_count as usize);
        let mut created = 0u32;
        // First node: a fixed root at `origin` when no anchor; otherwise the anchor
        // itself is node 0 and we link node 1 to it.
        let start_fixed = anchor == 0;
        for i in 0..node_count {
            let d = spacing * i as f64;
            let pos_v = Vec3 {
                x: origin.x + dir.x * d,
                y: origin.y + dir.y * d,
                z: origin.z + dir.z * d,
            };
            let fixed = start_fixed && i == 0;
            let h = unsafe { spawn_node(world, pos_v, node_mass, node_radius, fixed) };
            if h == 0 {
                set_error(ERR_INVALID_ARGUMENT, "soft_chain_create: node spawn failed");
                break;
            }
            handles.push(h);
            created += 1;

            // Link to previous node (or anchor for the first real node).
            let prev = if i == 0 {
                if anchor != 0 { Some(anchor) } else { None }
            } else {
                Some(handles[i as usize - 1])
            };
            if let Some(prev_h) = prev {
                let ok =
                    unsafe { link_spring(world, prev_h, h, axis, stiffness, damping, spacing) };
                if ok == Bool::FALSE {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "soft_chain_create: spring link failed",
                    );
                    break;
                }
            }
        }

        clear_error();
        created
    })
}

/// Read back the node handles of a soft chain that was just created.
///
/// Call [`soft_chain_create`] first; the chain's node handles are the last
/// `count` *dynamic* bodies, but to avoid ambiguity this helper snapshots the
/// *currently dynamic* bodies whose colliders are spheres of `node_radius`. For
/// simplicity it returns the handles of all dynamic bodies currently in the world
/// (callers typically create a fresh world per chain).
///
/// # Safety
/// `world` must be a valid world pointer; `out_handles` must point to writable
/// memory for `capacity` handles.
#[unsafe(no_mangle)]
pub extern "C" fn soft_chain_node_handles(
    world: *const WorldHandle,
    out_handles: *mut RigidBodyHandleRaw,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if out_handles.is_null() || capacity == 0 {
            return 0;
        }
        let out = unsafe { std::slice::from_raw_parts_mut(out_handles, capacity as usize) };
        let mut written = 0u32;
        for (h, body) in world.inner.bodies.iter() {
            if !body.is_dynamic() {
                continue;
            }
            if written as usize >= out.len() {
                break;
            }
            out[written as usize] = pack_rigid_body_handle(h);
            written += 1;
        }
        written
    })
}

// ── Phase 4: Minecraft voxel → soft-body + terrain-gravity coupling ──────────
//
// Phase 4 把软体接到 Minecraft 联动链路上：
// * `soft_body_voxel_build`：把一个 `VoxelGrid`（Minecraft 区块）转成质点-弹簧软体——
//   每个实心方块中心放一个质点，面相邻实心方块之间连弹簧（Phase 2 mass-spring 力，
//   走 `SoftBody::add_spring`；自由质点由 `SoftBody::step` 独立积分，绑定质点走
//   Phase 2 的 force_containers 力路由）。软体插入 `world.inner.soft_bodies`
//   （Phase 0b 接入点），由 `world_step` 统一推进。
// * `soft_body_set_gravity`：设置软体的每体常量加速度（地形重力耦合钩子）。调用方
//   每步用 `terrain_gravity_acceleration` 采样后写入此处，软体即吃到行星/球体引力。
//
// 区块破坏 → 软体更新：调用方先改 `VoxelGrid` 再重建软体（或后续接 Phase 2 的删边）；
// 本 MVP 提供从 voxel 生成软体的入口，破坏联动在 Minecraft 侧删 cell 后重建即可。
//
// 注意：`anvilkit` entity→SoftBody 联动（Phase 4 第三项）依赖 `anvilkit-bridge`
// feature，而该 feature 当前 `--all-features` 下有既存编译错误，故本 MVP 不含
// anvilkit 路径；voxel + 地形重力两条可独立验证，无需 anvilkit。

/// Build a mass-spring soft body from a `VoxelGrid` (Minecraft chunk).
///
/// One point-mass particle is placed at the center of every *solid* voxel
/// (`voxels[i] != 0`). Face-adjacent solid voxels are connected by a Hookean
/// spring with the given `stiffness`/`damping` and rest length equal to the
/// cell spacing along that axis. The resulting [`SoftBody`] is inserted into the
/// world's `soft_bodies` set and advanced by `world_step`.
///
/// # Parameters
/// * `voxels` — flat `size_x * size_y * size_z` array, indexing `x + size_x*(z +
///   size_z*y)`; non-zero = solid.
/// * `size_x/y/z` — grid dimensions (each > 0, product ≤ `voxels.len()`).
/// * `voxel_size` — world-space size of one cell edge (uniform; > 0).
/// * `origin` — world-space position of the (0,0,0) cell corner.
/// * `particle_mass` — mass of each solid-cell particle (> 0).
/// * `stiffness` / `damping` — spring coefficients (≥ 0).
/// * `pin_boundary` — when non-zero, particles whose cell touches the grid edge
///   are created pinned (`inv_mass = 0`), so the soft body is anchored to the
///   chunk boundary (useful for hanging terrain/structures from the world).
///
/// # Returns
/// The `SoftBodyId` (as `u32`) on success, or `0` on error (`ERR_*`).
///
/// # Safety
/// `world` must be a valid world pointer; `voxels` must point to `voxels_len`
/// readable bytes.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_voxel_build(
    world: *mut WorldHandle,
    voxels: *const u8,
    voxels_len: u32,
    size_x: u32,
    size_y: u32,
    size_z: u32,
    voxel_size: f64,
    origin: Vec3,
    particle_mass: f64,
    stiffness: f64,
    damping: f64,
    pin_boundary: Bool,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if voxels.is_null()
            || voxels_len == 0
            || size_x == 0
            || size_y == 0
            || size_z == 0
            || !voxel_size.is_finite()
            || voxel_size <= 0.0
            || !particle_mass.is_finite()
            || particle_mass <= 0.0
            || !stiffness.is_finite()
            || stiffness < 0.0
            || !damping.is_finite()
            || damping < 0.0
        {
            set_error(ERR_INVALID_ARGUMENT, "soft_body_voxel_build: bad params");
            return 0;
        }
        let n = size_x as usize * size_y as usize * size_z as usize;
        if n > voxels_len as usize {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_voxel_build: grid exceeds voxels_len",
            );
            return 0;
        }
        let grid = unsafe { std::slice::from_raw_parts(voxels, voxels_len as usize) };

        let mut body = rapier3d::prelude::soft_body::SoftBody::new(Vector::ZERO);
        // Map (x,y,z) -> particle index; only solid cells get a particle.
        let mut index: Vec<i64> = vec![-1; n];
        let mut count = 0u32;
        for y in 0..size_y as usize {
            for z in 0..size_z as usize {
                for x in 0..size_x as usize {
                    let gx = x as f64 * voxel_size;
                    let gy = y as f64 * voxel_size;
                    let gz = z as f64 * voxel_size;
                    let pos = Vec3 {
                        x: origin.x + gx + 0.5 * voxel_size,
                        y: origin.y + gy + 0.5 * voxel_size,
                        z: origin.z + gz + 0.5 * voxel_size,
                    };
                    let solid = grid[x + size_x as usize * (z + size_z as usize * y)] != 0;
                    if !solid {
                        continue;
                    }
                    let at_boundary = pin_boundary == Bool::TRUE
                        && (x == 0
                            || y == 0
                            || z == 0
                            || x + 1 == size_x as usize
                            || y + 1 == size_y as usize
                            || z + 1 == size_z as usize);
                    let idx = if at_boundary {
                        body.add_pinned(vec3_to_rapier(pos))
                    } else {
                        let i = body.add_particle(vec3_to_rapier(pos));
                        // set mass via inverse mass (1/m)
                        body.particles[i].inv_mass = 1.0 / particle_mass;
                        i
                    };
                    index[x + size_x as usize * (z + size_z as usize * y)] = idx as i64;
                    count += 1;
                }
            }
        }
        if count == 0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_voxel_build: no solid voxels",
            );
            return 0;
        }

        // Connect face-adjacent solid cells with springs (avoid double-linking by
        // only scanning +X, +Y, +Z neighbours).
        let sx = size_x as usize;
        let sz = size_z as usize;
        let at = |x: usize, y: usize, z: usize| x + sx * (z + sz * y);
        let neighbours = [(1, 0, 0), (0, 1, 0), (0, 0, 1)];
        for y in 0..size_y as usize {
            for z in 0..sz {
                for x in 0..sx {
                    let a = index[at(x, y, z)];
                    if a < 0 {
                        continue;
                    }
                    for (dx, dy, dz) in neighbours {
                        let nx = x + dx;
                        let ny = y + dy;
                        let nz = z + dz;
                        if nx >= sx || ny >= size_y as usize || nz >= sz {
                            continue;
                        }
                        let b = index[at(nx, ny, nz)];
                        if b < 0 {
                            continue;
                        }
                        body.add_spring(a as usize, b as usize, stiffness, damping);
                    }
                }
            }
        }

        clear_error();
        let id = world.inner.soft_bodies.insert(body);
        // Phase 5d: record the voxel→particle mapping so a later dig can resolve
        // which cell maps to which particle index (and rebuild after shifts).
        world.inner.voxel_soft_meta.insert(
            id.0,
            VoxelSoftMeta {
                sx,
                sy: size_y as usize,
                sz,
                origin,
                voxel_size,
                map: index.clone(),
            },
        );
        id.0
    })
}

/// Set the per-body constant acceleration (gravity) of a soft body.
///
/// This is the terrain-gravity coupling hook: the caller samples
/// `terrain_gravity_acceleration` per step and writes the resulting vector here,
/// so a soft body falls under planetary/spherical gravity instead of the world's
/// uniform `gravity`. Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_gravity(world: *mut WorldHandle, id: u32, gravity: Vec3) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !vec3_finite(gravity) {
            set_error(ERR_INVALID_ARGUMENT, "soft_body_set_gravity: bad gravity");
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_gravity: unknown id");
            return Bool::FALSE;
        };
        body.gravity = vec3_to_rapier(gravity);
        clear_error();
        Bool::TRUE
    })
}

// ── Phase 7: wind / air-resistance + sleeping + diagnostics ───────────────
//
// 复用 rapier 已有机制：
//   * `SoftBody::apply_wind(accel, drag)` / `clear_wind()` — 纯外力（与重力同路）。
//   * `SoftBodySet::sleep/wake/is_sleeping` — 粗粒度休眠标志（跳过 step）。
//   * `SoftBody::kinetic_energy()` / `total_volume()` — 标量诊断（能量/体积守恒）。

/// Phase 7: enable a uniform wind / air-resistance field on a soft body.
///
/// `accel` is a constant wind acceleration (`m/s²`) applied to every free
/// particle (like a sideways gravity); `drag` is a linear air-resistance
/// coefficient (`1/s`, `F_drag = -m·drag·v`). Both components must be finite.
///
/// # Returns
/// `Bool::TRUE` on success, `Bool::FALSE` on `ERR_*` (null world, bad id,
/// non-finite arguments).
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_apply_wind(
    world: *mut WorldHandle,
    id: u32,
    accel: Vec3,
    drag: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !vec3_finite(accel) || !drag.is_finite() {
            set_error(ERR_INVALID_ARGUMENT, "soft_body_apply_wind: bad accel/drag");
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_apply_wind: unknown id");
            return Bool::FALSE;
        };
        body.apply_wind(vec3_to_rapier(accel), drag);
        clear_error();
        Bool::TRUE
    })
}

/// Phase 7: disable the wind field on a soft body (`None`).
///
/// # Returns
/// `Bool::TRUE` on success, `Bool::FALSE` on `ERR_*` (null world, bad id).
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_clear_wind(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_clear_wind: unknown id");
            return Bool::FALSE;
        };
        body.clear_wind();
        clear_error();
        Bool::TRUE
    })
}

// ── Phase 28: clear/disabling variants for the *set_* material toggles ──────
//
// fork 的 `SoftBody` 给每个可开/关材料行为都配了一个 `clear_*` 方法（`clear_pressure`
// / `clear_self_collision` / `clear_cross_collision` / `clear_volume_conservation` /
// `clear_cohesion`），但此前只导出了 `clear_wind`。本组补齐其余 5 个关闭变体：
// 调用方不必把参数设成「0/负」去碰运气关掉，直接显式 disable，测试与联动逻辑更清楚。

/// Phase 28 — 关闭内部气压（等同 `pressure = None`，气球瘪掉）。
///
/// # Returns
/// `Bool::TRUE` 成功；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_clear_pressure(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_clear_pressure: unknown id");
            return Bool::FALSE;
        };
        sb.clear_pressure();
        clear_error();
        Bool::TRUE
    })
}

/// Phase 28 — 关闭自碰撞（等同 `self_collision = None`，无摩擦）。
///
/// # Returns
/// `Bool::TRUE` 成功；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_clear_self_collision(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_clear_self_collision: unknown id");
            return Bool::FALSE;
        };
        sb.clear_self_collision();
        clear_error();
        Bool::TRUE
    })
}

/// Phase 28 — 关闭跨体（软软）碰撞（等同 `cross_collision = None`，无摩擦）。
///
/// # Returns
/// `Bool::TRUE` 成功；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_clear_cross_collision(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_clear_cross_collision: unknown id");
            return Bool::FALSE;
        };
        sb.clear_cross_collision();
        clear_error();
        Bool::TRUE
    })
}

/// Phase 28 — 关闭体积守恒约束（等同 `volume_conservation = None`，blob 可随意压缩）。
///
/// # Returns
/// `Bool::TRUE` 成功；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_clear_volume_conservation(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(
                ERR_NOT_FOUND,
                "soft_body_clear_volume_conservation: unknown id",
            );
            return Bool::FALSE;
        };
        sb.clear_volume_conservation();
        clear_error();
        Bool::TRUE
    })
}

/// # Phase 29 — 开启四面体 corotated 线性弹性(旋转不变形状匹配)
///
/// 每个 XPBD 迭代里,在体积约束之后,把每个四面体向其 rest 形状的最优旋转
/// 匹配(polar 分解 shape matching)投影,提供旋转不变的线弹性偏应变回复。
/// rest 形状在调用时刻从当前质点位置快照(在未形变网格上开启)。
/// `stiffness` 为逐迭代松弛系数,取值 `(0, 1]`。
///
/// # Returns
/// `Bool::TRUE` 成功开启;`id` 未知 / world 为 null / `stiffness` 非法(非有限、<=0、>1) → `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效;`stiffness` 需为有限且 `0 < stiffness <= 1`。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_corotated(
    world: *mut WorldHandle,
    id: u32,
    stiffness: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !stiffness.is_finite() || stiffness <= 0.0 || stiffness > 1.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_corotated: stiffness must be in (0, 1]",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_corotated: unknown id");
            return Bool::FALSE;
        };
        if sb.set_corotated(stiffness) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_corotated: rejected by validator",
            );
            Bool::FALSE
        }
    })
}

/// # Phase 29 — 关闭 corotated 线性弹性
///
/// 等同 `corotated = None`;体积约束等其他特性不受影响。
///
/// # Returns
/// `Bool::TRUE` 成功关闭;`id` 未知 / world 为 null → `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_clear_corotated(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_clear_corotated: unknown id");
            return Bool::FALSE;
        };
        sb.clear_corotated();
        clear_error();
        Bool::TRUE
    })
}

/// # Phase 30 — 开启 Neo-Hookean 对数体积能量
///
/// 每个 XPBD 迭代里，四面体体积约束改用非线性残差 `C = ln(V/V₀)`
/// (J 下限 1e-6 保持有限)，compliance = `stiffness/dt²`。对数形式使体积抵抗
/// 随压缩无界增长(物理正确的不可压缩性)，取代线性 `V − V₀` 的有限推回。
/// 开启时覆盖 `volume_conservation` 的 compliance;关闭后回退线性体积约束。
///
/// # Returns
/// `Bool::TRUE` 成功开启;`id` 未知 / world 为 null / `stiffness` 非法(非有限、负数) → `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效;`stiffness` 需为有限非负值。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_neo_hookean(
    world: *mut WorldHandle,
    id: u32,
    stiffness: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_neo_hookean: unknown id");
            return Bool::FALSE;
        };
        if sb.set_neo_hookean(stiffness) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_neo_hookean: non-finite/negative stiffness",
            );
            Bool::FALSE
        }
    })
}

/// # Phase 30 — 关闭 Neo-Hookean 体积能量
///
/// 等同 `neo_hookean = None`;体积约束回退线性残差 + `volume_conservation` compliance。
///
/// # Returns
/// `Bool::TRUE` 成功关闭;`id` 未知 / world 为 null → `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_clear_neo_hookean(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_clear_neo_hookean: unknown id");
            return Bool::FALSE;
        };
        sb.clear_neo_hookean();
        clear_error();
        Bool::TRUE
    })
}

/// Phase 31 — 设置软体全局主动应变激活系数 γ∈[0,1]（「肌肉收缩」等级）。
///
/// 每条弹簧/距离约束的有效静止长度变为 `rest * (1 - γ)`，正值主动把两端拉近。
/// 非有限值被忽略（无操作）。`0` 为被动基线。
///
/// # Returns
/// `Bool::TRUE` 总是成功（除非 world 为 null 或 id 未知返回 `Bool::FALSE`）。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_activation(world: *mut WorldHandle, id: u32, gamma: f64) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_activation: unknown id");
            return Bool::FALSE;
        };
        if sb.set_activation(gamma) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_activation: activation must be in [0, 1]",
            );
            Bool::FALSE
        }
    })
}

/// Phase 31 — 设置单条弹簧（按 `add_spring` 返回的索引）的主动应变激活系数。
///
/// 越界 / 非有限 / 不在 [0,1] 的 `activation` 被拒绝，返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_spring_activation(
    world: *mut WorldHandle,
    id: u32,
    index: u32,
    activation: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_spring_activation: unknown id");
            return Bool::FALSE;
        };
        if sb.set_spring_activation(index as usize, activation) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_spring_activation: bad index/value",
            );
            Bool::FALSE
        }
    })
}

/// Phase 31 — 设置单条距离约束（按 `add_distance_constraint` 返回的索引）的主动应变激活系数。
///
/// 越界 / 非有限 / 不在 [0,1] 的 `activation` 被拒绝，返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_distance_constraint_activation(
    world: *mut WorldHandle,
    id: u32,
    index: u32,
    activation: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(
                ERR_NOT_FOUND,
                "soft_body_set_distance_constraint_activation: unknown id",
            );
            return Bool::FALSE;
        };
        if sb.set_distance_constraint_activation(index as usize, activation) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_distance_constraint_activation: bad index/value",
            );
            Bool::FALSE
        }
    })
}

/// Phase 32 — 设置单条距离约束（按 `add_distance_constraint` 返回的索引）的肌肉
/// 纤维走向 `dir = (dx, dy, dz)`。非零向量被归一化后作为主动收缩方向（各向异性
/// 驱动）；全零向量清除纤维（退回沿边收缩）。返回 `Bool::FALSE` 表示 `id` 未知 /
/// 索引越界 / 向量非有限。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_fibre_direction(
    world: *mut WorldHandle,
    id: u32,
    index: u32,
    dx: f64,
    dy: f64,
    dz: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_fibre_direction: unknown id");
            return Bool::FALSE;
        };
        let dir = Vector {
            x: dx,
            y: dy,
            z: dz,
        };
        if sb.set_fibre_direction(index as usize, dir) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_fibre_direction: bad index/value",
            );
            Bool::FALSE
        }
    })
}

/// Phase 32 — 设置单条弹簧（按 `add_spring` 返回的索引）的肌肉纤维走向，语义同
/// `soft_body_set_fibre_direction`。返回 `Bool::FALSE` 表示 `id` 未知 / 索引越界 /
/// 向量非有限。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_spring_fibre_direction(
    world: *mut WorldHandle,
    id: u32,
    index: u32,
    dx: f64,
    dy: f64,
    dz: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(
                ERR_NOT_FOUND,
                "soft_body_set_spring_fibre_direction: unknown id",
            );
            return Bool::FALSE;
        };
        let dir = Vector {
            x: dx,
            y: dy,
            z: dz,
        };
        if sb.set_spring_fibre_direction(index as usize, dir) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_spring_fibre_direction: bad index/value",
            );
            Bool::FALSE
        }
    })
}

/// Phase 28 — 关闭黏连/可撕 glue（等同 `cohesion = None`，不再互相吸附）。
///
/// # Returns
/// `Bool::TRUE` 成功；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_clear_cohesion(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_clear_cohesion: unknown id");
            return Bool::FALSE;
        };
        sb.clear_cohesion();
        clear_error();
        Bool::TRUE
    })
}

/// Phase 7: mark a soft body as sleeping (no further integration until woken).
///
/// # Returns
/// `Bool::TRUE` if the body existed and was put to sleep, `Bool::FALSE` on
/// `ERR_*` (null world, bad id).
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_sleep(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        if !world.inner.soft_bodies.sleep(sid) {
            set_error(ERR_NOT_FOUND, "soft_body_sleep: unknown id");
            return Bool::FALSE;
        }
        clear_error();
        Bool::TRUE
    })
}

/// Phase 7: wake a sleeping soft body (resume integration).
///
/// # Returns
/// `Bool::TRUE` if the body existed and was woken, `Bool::FALSE` on `ERR_*`
/// (null world, bad id).
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_wake(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        if !world.inner.soft_bodies.wake(sid) {
            set_error(ERR_NOT_FOUND, "soft_body_wake: unknown id");
            return Bool::FALSE;
        }
        clear_error();
        Bool::TRUE
    })
}

/// Phase 7: whether a soft body is currently sleeping.
///
/// # Returns
/// `Bool::TRUE` if sleeping, `Bool::FALSE` if awake or the id is unknown /
/// world is null (and `ERR_*` is set).
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_is_sleeping(world: *const WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        if world.inner.soft_bodies.get(sid).is_none() {
            set_error(ERR_NOT_FOUND, "soft_body_is_sleeping: unknown id");
            return Bool::FALSE;
        }
        // `is_sleeping` returns false for unknown ids too; we already checked.
        clear_error();
        Bool::from(world.inner.soft_bodies.is_sleeping(sid))
    })
}

/// Phase 7: total kinetic energy of a soft body's free particles (`½·m·|v|²`).
///
/// # Returns
/// The kinetic energy (finite), or `0.0` with `ERR_*` set on null world / bad id.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_kinetic_energy(world: *const WorldHandle, id: u32) -> f64 {
    ffi_guard(0.0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0.0;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_kinetic_energy: unknown id");
            return 0.0;
        };
        clear_error();
        body.kinetic_energy()
    })
}

/// Phase 7: normalized total volume of a soft body's tetrahedra
/// (sum of `|V|/|V_rest|`, so a unit-scaled, deformation-sensitive scalar).
/// For bodies with no tetrahedra this is `0.0`.
///
/// # Returns
/// The normalized volume (finite), or `0.0` with `ERR_*` set on null world /
/// bad id.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_total_volume(world: *const WorldHandle, id: u32) -> f64 {
    ffi_guard(0.0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0.0;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_total_volume: unknown id");
            return 0.0;
        };
        clear_error();
        body.total_volume()
    })
}

/// # Phase 8 — 锚定软体任意质点到刚体
///
/// 把 `id` 软体的第 `particle` 号质点绑定到刚体 `body`，使其刚性跟随该刚体的
/// 平移/旋转。`attach_point` 为绑点世界坐标（通常用该质点当前位置）；函数内部
/// 把它换算成刚体局部坐标存储，故跟随刚体运动时不会漂移。绑定后该质点停止本地
/// 积分，其弹簧/阻尼力改由 `SoftBodySet::write_spring_forces` 路由进刚体的
/// `force_containers`（软体拖动刚体）。
///
/// # Returns
/// `Bool::TRUE` 成功；`particle` 越界 / `body` 不存在 / world 为 null 返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效；`attach_point` 各分量需为有限值。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_attach_particle(
    world: *mut WorldHandle,
    id: u32,
    particle: u32,
    body: RigidBodyHandleRaw,
    attach_point: Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !vec3_finite(attach_point) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_attach_particle: non-finite point",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_attach_particle: unknown id");
            return Bool::FALSE;
        };
        let rbh = unpack_rigid_body_handle(body);
        if world.inner.bodies.get(rbh).is_none() {
            set_error(ERR_NOT_FOUND, "soft_body_attach_particle: unknown body");
            return Bool::FALSE;
        }
        let pt = vec3_to_rapier(attach_point);
        match sb.attach_particle(particle as usize, rbh, pt, &world.inner.bodies) {
            true => {
                clear_error();
                Bool::TRUE
            }
            false => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "soft_body_attach_particle: particle out of range",
                );
                Bool::FALSE
            }
        }
    })
}

/// # Phase 8 — 解除质点与刚体的锚定
///
/// 把 `id` 软体的第 `particle` 号质点从任何已绑定刚体上解绑，恢复为自由（本地积分）
/// 质点。已自由则视为成功（幂等）。
///
/// # Returns
/// `Bool::TRUE` 成功（含已自由）；`particle` 越界 / world 为 null 返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_detach_particle(
    world: *mut WorldHandle,
    id: u32,
    particle: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_detach_particle: unknown id");
            return Bool::FALSE;
        };
        match sb.detach_particle(particle as usize) {
            true => {
                clear_error();
                Bool::TRUE
            }
            false => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "soft_body_detach_particle: particle out of range",
                );
                Bool::FALSE
            }
        }
    })
}

/// # Phase 9 — 设置撕裂阈值（应变阈值）
///
/// 把 `id` 软体的撕裂阈值设为 `strain_to_break`（应变 = `(|len| − rest)/rest`，
/// 即拉伸量相对静止长度的比例）。每步 `step` 开始时，任何应变超过该阈值的**结构边**
/// （XPBD distance constraint 或 MassSpring spring）会被移除；失去任一结构边的三角形面
/// 也会被删掉，使撕裂的布料停止渲染破损面。
///
/// - `enabled != 0` 且 `strain_to_break > 0`：开启撕裂（阈值 = `strain_to_break`）。
/// - `enabled == 0`：关闭撕裂（等同于 `tear_strain = None`，默认）。
/// - `strain_to_break <= 0`：视为非法，关闭撕裂（避免首步即全撕）。
///
/// # Returns
/// `Bool::TRUE` 成功（含关闭）；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效；`strain_to_break` 需为有限值。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_tear_strain(
    world: *mut WorldHandle,
    id: u32,
    strain_to_break: f64,
    enabled: u8,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !strain_to_break.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_tear_strain: non-finite threshold",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_tear_strain: unknown id");
            return Bool::FALSE;
        };
        let strain = if enabled != 0 && strain_to_break > 0.0 {
            Some(strain_to_break)
        } else {
            None
        };
        sb.set_tear_strain(strain);
        clear_error();
        Bool::TRUE
    })
}

/// # Phase 27 — 设置断裂力学撕裂准则（轴向应力阈值）
///
/// 把 `id` 软体的撕裂准则设为 `Stress(threshold)`：任何结构边（XPBD distance
/// constraint 或 MassSpring spring）的轴向力 `|k·(len − rest)|` 超过 `threshold`
/// 时断裂。`k` = 弹簧刚度，或 `1/(compliance + ε)`（XPBD 距离约束）。
/// `enabled == 0` 或 `threshold <= 0` 关闭撕裂（等同于 `tear = None`）。
///
/// # Returns
/// `Bool::TRUE` 成功（含关闭）；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_tear_stress(
    world: *mut WorldHandle,
    id: u32,
    stress_to_break: f64,
    enabled: u8,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !stress_to_break.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_tear_stress: non-finite threshold",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_tear_stress: unknown id");
            return Bool::FALSE;
        };
        let stress = if enabled != 0 && stress_to_break > 0.0 {
            Some(stress_to_break)
        } else {
            None
        };
        sb.set_tear_stress(stress);
        clear_error();
        Bool::TRUE
    })
}

/// # Phase 27 — 设置断裂力学撕裂准则（应变能 / 断裂韧性阈值）
///
/// 把 `id` 软体的撕裂准则设为 `Energy(threshold)`：任何结构边的弹性应变能
/// `½·k·(len − rest)²` 超过 `threshold` 时断裂（断裂韧性临界释放率代理）。
/// `enabled == 0` 或 `threshold <= 0` 关闭撕裂（等同于 `tear = None`）。
///
/// # Returns
/// `Bool::TRUE` 成功（含关闭）；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_tear_energy(
    world: *mut WorldHandle,
    id: u32,
    energy_to_break: f64,
    enabled: u8,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !energy_to_break.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_tear_energy: non-finite threshold",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_tear_energy: unknown id");
            return Bool::FALSE;
        };
        let energy = if enabled != 0 && energy_to_break > 0.0 {
            Some(energy_to_break)
        } else {
            None
        };
        sb.set_tear_energy(energy);
        clear_error();
        Bool::TRUE
    })
}

/// # Phase 27 — 设置体级正交各向异性刚度轴
///
/// `anisotropy != 0` 且 `x,y,z 有限且 >= 0` 时，开启方向相关刚度：每条边有效
/// XPBD 柔度 = `base / (nᵀ·diag(x,y,z)·n)`（n 为边单位方向），使沿 x 轴对齐的
/// 边在 `x > 1` 时更硬。传 `enabled == 0` 或 `x=y=z=0` 关闭（各边保持各向同性）。
///
/// # Returns
/// `Bool::TRUE` 成功（含关闭）；`world` 为 null / 向量含非有限值 / 含负分量
/// 返回 `Bool::FALSE`。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_anisotropy(
    world: *mut WorldHandle,
    id: u32,
    x: f64,
    y: f64,
    z: f64,
    enabled: u8,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_anisotropy: non-finite component",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_anisotropy: unknown id");
            return Bool::FALSE;
        };
        let axes = if enabled != 0 && x >= 0.0 && y >= 0.0 && z >= 0.0 && (x + y + z) > 0.0 {
            Some(rapier3d::math::Vector::new(x, y, z))
        } else {
            None
        };
        sb.set_anisotropy(axes);
        clear_error();
        Bool::TRUE
    })
}

/// # Phase 27 — 设置黏弹性（率相关）本构
///
/// `enabled != 0` 且 `rate_coefficient >= 0`：开启 Kelvin-Voigt 式应变率硬化——
/// 有效刚度 `k_eff = k·(1 + rate_coefficient·|d(strain)/dt|)`，快速拉伸的边比缓慢
/// 拉伸更硬（聚合物/黏弹性行为）。`enabled == 0` 或非法参数关闭（纯弹性）。
///
/// # Returns
/// `Bool::TRUE` 成功（含关闭）；非法参数返回 `Bool::FALSE`。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_viscoelastic(
    world: *mut WorldHandle,
    id: u32,
    rate_coefficient: f64,
    enabled: u8,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !rate_coefficient.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_viscoelastic: non-finite coefficient",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_viscoelastic: unknown id");
            return Bool::FALSE;
        };
        let ok = if enabled != 0 && rate_coefficient >= 0.0 {
            sb.set_viscoelastic(Some(rapier3d::prelude::soft_body::ViscoelasticParams {
                rate_coefficient,
            }))
        } else {
            sb.set_viscoelastic(None)
        };
        if !ok {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_viscoelastic: invalid coefficient",
            );
            return Bool::FALSE;
        }
        clear_error();
        Bool::TRUE
    })
}

/// # Phase 27 — 设置均匀温度场（热膨胀 + 温度相关模量）
///
/// `enabled != 0` 且参数有限、`stiffness_temp_coeff·|temp−ambient| < 1`：开启温度场——
/// 每条边静止长度按 `rest·(1 + expansion·ΔT)` 膨胀，刚度按 `k·(1 − stiffness_temp_coeff·ΔT)`
/// 软化。关闭（`enabled == 0` 或非法）回到等温。
///
/// # Returns
/// `Bool::TRUE` 成功（含关闭）；非法参数返回 `Bool::FALSE`。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_thermal(
    world: *mut WorldHandle,
    id: u32,
    temp: f64,
    ambient: f64,
    expansion: f64,
    stiffness_temp_coeff: f64,
    enabled: u8,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !temp.is_finite()
            || !ambient.is_finite()
            || !expansion.is_finite()
            || !stiffness_temp_coeff.is_finite()
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_thermal: non-finite parameter",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_thermal: unknown id");
            return Bool::FALSE;
        };
        let ok = if enabled != 0 {
            sb.set_thermal(Some(rapier3d::prelude::soft_body::ThermalParams {
                temp,
                ambient,
                expansion,
                stiffness_temp_coeff,
            }))
        } else {
            sb.set_thermal(None)
        };
        if !ok {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_thermal: invalid thermal params",
            );
            return Bool::FALSE;
        }
        clear_error();
        Bool::TRUE
    })
}

/// # Phase 10 — 设置塑性参数（永久变形 / 像橡皮泥 / 记忆棉）
///
/// 把 `id` 软体的塑性设为 `PlasticityParams { yield_strain, creep }`：
/// - 任何结构边（XPBD distance constraint 或 MassSpring spring）的弹性应变幅度
///   `|(|len| − rest)/rest|` 超过 `yield_strain` 时，每步把 rest_length 朝当前长度
///   方向移动 `creep`（夹到 `[0,1]`），使变形永久"冻住"而不是回弹。
/// - `enabled != 0` 且 `yield_strain > 0`：开启塑性（threshold=yield_strain, rate=creep）。
/// - `enabled == 0`：关闭塑性（等同于 `plasticity = None`，即完全弹性，默认）。
/// - `yield_strain <= 0` 或 `creep <= 0`：视为非法，关闭塑性。
///
/// # Returns
/// `Bool::TRUE` 成功（含关闭）；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效；`yield_strain` / `creep` 需为有限值。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_plasticity(
    world: *mut WorldHandle,
    id: u32,
    yield_strain: f64,
    creep: f64,
    enabled: u8,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !yield_strain.is_finite() || !creep.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_plasticity: non-finite parameter",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_plasticity: unknown id");
            return Bool::FALSE;
        };
        let params = if enabled != 0 && yield_strain > 0.0 {
            Some(rapier3d::dynamics::soft_body::PlasticityParams {
                yield_strain,
                creep,
            })
        } else {
            None
        };
        sb.set_plasticity(params);
        clear_error();
        Bool::TRUE
    })
}

// ── Phase 28: manual-trigger behavior FFI (no new fork mechanics) ───────────
//
// fork 的 `SoftBody` 把塑性投影 `apply_plasticity` 与撕裂 `tear` 都放在 step 内部按
// 配置自动跑；但外部（尤其测试 / 联动闭环）有时需要「现在就跑一次」而不推进时间步。
// 本组暴露两个手动触发入口 + 一个弹簧力读回入口，全部纯包裹既有方法，零 fork 改动。

/// Phase 28 — 手动触发一次塑性投影（把超 `yield_strain` 的结构边 rest_length 朝当前长度
/// 冻结 `creep`）。通常塑性在 `step` 内自动应用；此 FFI 让调用方在不推进时间步的情况下
/// 即时「定型」（例如绑定到 Minecraft 方块编辑一次后立刻烤出永久变形）。需先经
/// `soft_body_set_plasticity` 配置过 `PlasticityParams`，否则为 no-op。
///
/// # Returns
/// `Bool::TRUE` 成功触发；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_apply_plasticity(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_apply_plasticity: unknown id");
            return Bool::FALSE;
        };
        sb.apply_plasticity();
        clear_error();
        Bool::TRUE
    })
}

/// Phase 28 — 手动触发一次撕裂：立刻丢弃所有超过 `tear` 阈值（应变 / 轴向应力 /
/// 应变能，由 `soft_body_set_tear_*` 配置）的结构边，并连带删掉失去边支撑的三角面。
/// 通常撕裂在 `step` 顶部自动发生；此 FFI 让调用方在「不推进时间步」时也能立即撕开
/// （例如一次性加载预撕裂状态、或在联动闭环里随时展示断裂）。未配置 `tear` 阈值时
/// 为 no-op（返回 `Bool::TRUE`）。
///
/// # Returns
/// `Bool::TRUE` 成功触发（含 no-op）；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_tear_now(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_tear_now: unknown id");
            return Bool::FALSE;
        };
        sb.tear();
        clear_error();
        Bool::TRUE
    })
}

/// Phase 28 — 读回每个质点累积的弹簧/阻尼合力（调试 / 可视化用）。`out_forces` 指向
/// `capacity` 个 `Vec3` 的写缓冲区；第 `i` 项为质点 `i` 的合力（按 `spring_damping_forces`
/// 计算）。缓冲区过小则截断（永不越界）；返回真实的质点数量（不受 capacity 限制）。
/// `out_forces` 为 null 或 `capacity == 0` 仅返回数量不写。
///
/// # Safety
/// `world` 必须有效；`out_forces`（若非 null）须指向至少 `capacity` 个 `Vec3`。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_read_spring_forces(
    world: *const WorldHandle,
    id: u32,
    out_forces: *mut Vec3,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let sid = SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_read_spring_forces: unknown id");
            return 0;
        };
        let forces = sb.spring_damping_forces();
        if !out_forces.is_null() && capacity > 0 {
            let out = unsafe { std::slice::from_raw_parts_mut(out_forces, capacity as usize) };
            let n = forces.len().min(out.len());
            for (i, f) in forces.iter().take(n).enumerate() {
                out[i] = vec3_from_rapier(*f);
            }
        }
        clear_error();
        forces.len() as u32
    })
}

/// # Phase 11 — 设置内部气压（充气 / 气球模型）
///
/// 把 `id` 软体的内部气压设为 `pressure`（力/面积）。每步在 `compute_forces`（MassSpring）
/// 与 `step_xpbd`（预测步）中，对每个**闭合三角网格**的自由质点沿面法向施加向外推力
/// `F = pressure · area`，把闭合壳"吹胀"。`pressure > 0` 开启；`pressure <= 0` 视为关闭
/// （等同于 `pressure = None`，默认）。
///
/// 纯外力，与风场同构；不引入新求解器力学。需 `self.triangles` 构成闭合流形才能像真气球，
/// 开口薄片会沿单面法向鼓起。
///
/// # Returns
/// `Bool::TRUE` 成功；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效；`pressure` 需为有限值。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_pressure(world: *mut WorldHandle, id: u32, pressure: f64) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !pressure.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_pressure: non-finite pressure",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_pressure: unknown id");
            return Bool::FALSE;
        };
        sb.set_pressure(pressure);
        clear_error();
        Bool::TRUE
    })
}

/// # Phase 12 — 开启/关闭软体自碰撞(self-collision)
///
/// 把 `id` 软体的自碰撞设为 `radius`(粒子球半径)+ `stiffness`(XPBD 排斥约束柔度, `0`=硬)。
/// 每步求解中,任意两个自由质点中心距 `< 2*radius` 时沿连线被推开(各自视为该半径的球),
/// 但**直接结构邻居**(已有 distance_constraint 边相连的质点对)被排除,不误判为碰撞。
/// 采用均匀空间哈希做 broad-phase,在 MassSpring 与 XPBD 两条路径内逐迭代投影,纯位置约束,
/// 不引入新求解器力学。非法参数(`radius <= 0` / `stiffness < 0` / 非有限)返回 `Bool::FALSE` 且不开。
///
/// # Returns
/// `Bool::TRUE` 成功开启;`id` 未知 / world 为 null / 参数非法返回 `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效;`radius` / `stiffness` 需为有限值。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_self_collision(
    world: *mut WorldHandle,
    id: u32,
    radius: f64,
    stiffness: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !radius.is_finite() || !stiffness.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_self_collision: non-finite radius/stiffness",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_self_collision: unknown id");
            return Bool::FALSE;
        };
        // set_self_collision rejects bad params (returns false). A valid-id rejection
        // is a parameter error -> report FALSE (mirrors soft_body_set_pressure /
        // soft_body_set_plasticity convention for invalid arguments).
        let ok = sb.set_self_collision(radius, stiffness);
        if !ok {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_self_collision: radius<=0 or stiffness<0",
            );
            return Bool::FALSE;
        }
        clear_error();
        Bool::TRUE
    })
}

/// # Phase 13 — 运行时改单条弹簧刚度
///
/// 把 `id` 软体里下标 `index`(由 `soft_body_add_spring` 返回)的弹簧刚度(Hookean `k`)改为 `stiffness`。
/// 用于构造后就地调材质异质性(例如把"骨骼"弹簧调硬、"腱"调软),无需重建拓扑。
/// `stiffness < 0` 或非有限 → 返回 `Bool::FALSE` 且不改。
///
/// # Returns
/// `Bool::TRUE` 修改成功;`id` 未知 / `index` 越界 / world 为 null / 参数非法 → `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效;`stiffness` 需为有限非负值。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_spring_stiffness(
    world: *mut WorldHandle,
    id: u32,
    index: u32,
    stiffness: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_spring_stiffness: unknown id");
            return Bool::FALSE;
        };
        if sb.set_spring_stiffness(index as usize, stiffness) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_spring_stiffness: bad index or negative/non-finite stiffness",
            );
            Bool::FALSE
        }
    })
}

/// # Phase 13 — 运行时改单条 XPBD 距离约束柔度
///
/// 把 `id` 软体里下标 `index`(由 `soft_body_add_distance_constraint` 返回)的 XPBD 距离约束柔度
/// (compliance α)改为 `compliance`。XPBD 求解器逐约束读取各自柔度(见 `step_xpbd`),因此不同边
/// 可拥有不同刚度。`compliance < 0` 或非有限 → 返回 `Bool::FALSE` 且不改。
///
/// # Returns
/// `Bool::TRUE` 修改成功;`id` 未知 / `index` 越界 / world 为 null / 参数非法 → `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效;`compliance` 需为有限非负值。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_distance_constraint_compliance(
    world: *mut WorldHandle,
    id: u32,
    index: u32,
    compliance: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(
                ERR_NOT_FOUND,
                "soft_body_set_distance_constraint_compliance: unknown id",
            );
            return Bool::FALSE;
        };
        if sb.set_distance_constraint_compliance(index as usize, compliance) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_distance_constraint_compliance: bad index or negative/non-finite compliance",
            );
            Bool::FALSE
        }
    })
}

/// # Phase 19 — 设置某条距离约束的「压缩」柔度(各向异性柔度)
///
/// 把 `id` 软体第 `index` 条距离约束的**压缩** XPBD 柔度 `α_c` 设为 `compression`。
/// 该约束原本只有单一 `compliance`(拉伸/压缩共用,各向同性)。本函数令其独立在
/// **压缩**(`len < rest`,被压短)时采用 `compression` 柔度——布料/泡沫可「抗拉伸但易折叠」,
/// 是标准的各向异性 XPBD 行为。`stretch` 柔度仍由 `soft_body_set_distance_constraint_compliance`
/// 控制;二者相等即回到各向同性。求解器每个迭代按当前应变符号选用对应柔度。
/// 非法参数(`index` 越界 / `compression` 为负或非有限)返回 `Bool::FALSE`。
///
/// # Returns
/// `Bool::TRUE` 成功;`id` 未知 / 约束 `index` 越界 / 参数非法 → `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效;`index` 须在 `[0, 约束数)`;`compression >= 0` 且有限。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_distance_constraint_compression(
    world: *mut WorldHandle,
    id: u32,
    index: u32,
    compression: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(
                ERR_NOT_FOUND,
                "soft_body_set_distance_constraint_compression: unknown id",
            );
            return Bool::FALSE;
        };
        if sb.set_distance_constraint_compression(index as usize, compression) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_distance_constraint_compression: bad index or negative/non-finite compression",
            );
            Bool::FALSE
        }
    })
}

/// # Phase 14 — 开启/关闭软体间的软软碰撞(soft-soft / cross-body)
///
/// 把 `id` 软体的软软碰撞设为 `radius`(粒子球半径)+ `stiffness`(XPBD 排斥约束柔度, `0`=硬)。
/// 世界级 step 结束后,任意两个**都**开启了软软碰撞的软体,其自由质点中心距 `< 2·min(ra,rb)`
/// 时沿连线被推开(各自视为该半径的球)。复用 Phase 12 的空间哈希 + XPBD 投影原语,但在
/// world 层遍历软体对。只排 inter-body 对(同体内自碰撞由 Phase 12 处理)。
/// 非法参数(`radius <= 0` / `stiffness < 0` / 非有限)返回 `Bool::FALSE` 且不开。
///
/// # Returns
/// `Bool::TRUE` 成功开启;`id` 未知 / world 为 null / 参数非法 → `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效;`radius` / `stiffness` 需为有限值。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_cross_collision(
    world: *mut WorldHandle,
    id: u32,
    radius: f64,
    stiffness: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !radius.is_finite() || !stiffness.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_cross_collision: non-finite radius/stiffness",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_cross_collision: unknown id");
            return Bool::FALSE;
        };
        if sb.set_cross_collision(radius, stiffness) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_cross_collision: radius<=0 or stiffness<0",
            );
            Bool::FALSE
        }
    })
}

/// # Phase 20 — 设置自碰撞接触摩擦系数 μ(0 ≤ μ ≤ 1)
///
/// 需要先 `soft_body_set_self_collision` 开启自碰撞。μ 控制接触处切向相对速度被阻尼的比例
/// (μ=0 无摩擦, μ=1 完全消除切向滑动, Coulomb 风格)。非法参数(非有限 / 越界 / 未开启自碰撞)
/// 返回 `Bool::FALSE` 且不改动状态。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_self_collision_friction(
    world: *mut WorldHandle,
    id: u32,
    mu: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(
                ERR_NOT_FOUND,
                "soft_body_set_self_collision_friction: unknown id",
            );
            return Bool::FALSE;
        };
        if sb.set_self_collision_friction(mu) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_self_collision_friction: μ 非有限/越界(0≤μ≤1)或未开启自碰撞",
            );
            Bool::FALSE
        }
    })
}

/// # Phase 20 — 设置软软(跨体)碰撞接触摩擦系数 μ(0 ≤ μ ≤ 1)
///
/// 需要先 `soft_body_set_cross_collision` 开启跨体碰撞。语义同自碰撞摩擦:阻尼接触切向相对
/// 速度。实际生效的 μ 为两体 `min(μ_a, μ_b)`(任一体无摩擦则该接触无摩擦)。非法参数返回
/// `Bool::FALSE`。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_cross_collision_friction(
    world: *mut WorldHandle,
    id: u32,
    mu: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(
                ERR_NOT_FOUND,
                "soft_body_set_cross_collision_friction: unknown id",
            );
            return Bool::FALSE;
        };
        if sb.set_cross_collision_friction(mu) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_cross_collision_friction: μ 非有限/越界(0≤μ≤1)或未开启跨体碰撞",
            );
            Bool::FALSE
        }
    })
}

/// # Phase 16 — 开启/关闭体积守恒约束(独立柔度, 与距离求解器解耦)
///
/// 把 `id` 软体的四面体体积约束柔度设为 `compliance`(`0`=硬/不可压缩)。开启后 `step_xpbd`
/// 里每条四面体体积约束用 `α̃ = compliance / dt²` 求解 —— 与距离求解器的 compliance 无关,
/// 因此可以让边很软而体积保持硬(不可压 blob)。与 Phase 11 气压正交:气压是向外吹胀的力,
/// 本约束是把总体积拉回静止值。关闭(`clear`)后体积约束回退到全局求解器 compliance。
/// 非法参数(非有限 / 负数)返回 `Bool::FALSE` 且不开。
///
/// # Returns
/// `Bool::TRUE` 成功开启;`id` 未知 / world 为 null / 参数非法 → `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效;`compliance` 需为有限非负值。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_volume_conservation(
    world: *mut WorldHandle,
    id: u32,
    compliance: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !compliance.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_volume_conservation: non-finite compliance",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(
                ERR_NOT_FOUND,
                "soft_body_set_volume_conservation: unknown id",
            );
            return Bool::FALSE;
        };
        if sb.set_volume_conservation(compliance) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_volume_conservation: negative/non-finite compliance",
            );
            Bool::FALSE
        }
    })
}

/// # Phase 18 — 设置全局内部(结构)阻尼系数
///
/// 把 `id` 软体的 `damping` 设为 `d`。每个 step 里每个自由质点的速度乘以 `1 - d`
/// (jelly / slime 式能量耗散),与 Phase 0 的弹簧轴向阻尼、Phase 13 的逐约束柔度正交。
/// `d=0` 无阻尼;`d in [0,1)` 振荡收敛更快;`d>=1` 或非法(非有限/负数)返回 `Bool::FALSE`。
///
/// # Returns
/// `Bool::TRUE` 成功设置;`id` 未知 / world 为 null / 参数非法 → `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效;`d` 需为有限且 `0 <= d < 1`。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_damping(world: *mut WorldHandle, id: u32, d: f64) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        if !d.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_damping: non-finite damping",
            );
            return Bool::FALSE;
        }
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_damping: unknown id");
            return Bool::FALSE;
        };
        if sb.set_damping(d) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_damping: need 0 <= d < 1",
            );
            Bool::FALSE
        }
    })
}

/// # Phase 17 — 开启/关闭软体间黏连(可撕黏附 glue)
///
/// 把 `id` 软体的 `cohesion` 设为 `CohesionParams{radius, stiffness, break_distance}`。
/// 开启后,本软体与*其它*也开了 cohesion 的软体之间:自由质点彼此进入 `radius` 即被互相
/// 吸引到接触距离(`radius`),把两体黏在一起(Phase 9 撕裂的对偶)。bond 可破断:若某对
/// 已被拉到 `break_distance` 之外,本步不再吸引(胶水撕裂)。`break_distance=inf` 表示永久胶。
/// 关闭(`clear`)后不再黏连。非法参数(radius<=0 / stiffness<0 / break_distance<=radius /
/// 任一为 NaN)返回 `Bool::FALSE` 且不开;注意 `break_distance=inf` 合法(永久胶)。
///
/// # Returns
/// `Bool::TRUE` 成功开启;`id` 未知 / world 为 null / 参数非法 → `Bool::FALSE`。
///
/// # Safety
/// `world` 必须有效;参数需为有限且符合约束。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_cohesion(
    world: *mut WorldHandle,
    id: u32,
    radius: f64,
    stiffness: f64,
    break_distance: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let sid = rapier3d::prelude::soft_body::SoftBodyId(id);
        let Some(sb) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_cohesion: unknown id");
            return Bool::FALSE;
        };
        if sb.set_cohesion(radius, stiffness, break_distance) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_cohesion: bad radius/stiffness/break_distance",
            );
            Bool::FALSE
        }
    })
}

// ── Phase 5a: general soft-body builder (unlock arbitrary topology) ──────────了「voxel 网格 → 软体」与「设置重力」两个高层入口，外部调用方
// （JNI/FFM/Minecraft）无法构造任意拓扑的软体（自定义质点、弹簧、XPBD 距离约束、
// 四面体体积元）也无法切求解器。本组 FFI 把 rapier `SoftBody` 的 `add_particle` /
// `add_pinned` / `add_spring` / `add_distance_constraint` / `add_tetrahedron` /
// `configure_xpbd` 暴露出来，让上层能逐点搭建软体。
//
// 惯例：成功返回真实 id / 粒子下标（可为 0，首个软体/质点下标即 0）；失败返回
// `u32::MAX`（`add_*`/`create`）或 `Bool::FALSE`（布尔类），并置 `ERR_*`。

/// Create an empty soft body in the world and return its `SoftBodyId`.
///
/// The body starts in the `MassSpring` solver; switch it to XPBD with
/// [`soft_body_configure_solver`] if you intend to use distance/tetra constraints.
///
/// # Returns
/// The `SoftBodyId` (as `u32`) on success, or `u32::MAX` on error (`ERR_*`).
///
/// # Safety
/// `world` must be a valid world pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_create(world: *mut WorldHandle, gravity: Vec3) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_create: world is null");
            return u32::MAX;
        };
        if !vec3_finite(gravity) {
            set_error(ERR_INVALID_ARGUMENT, "soft_body_create: bad gravity");
            return u32::MAX;
        }
        clear_error();
        let body = SoftBody::new(vec3_to_rapier(gravity));
        let id = world.inner.soft_bodies.insert(body);
        id.0
    })
}

/// Clone a soft body into a new standalone body, returning the new body id.
///
/// Deep-copies the source body verbatim — particles (position/velocity/inv_mass),
/// springs, distance constraints, tetrahedra (+ rest volumes), triangles, solver
/// selection, gravity, sleeping, damping, substeps, and every optional field
/// (wind, pressure, tearing, plasticity, self/cross collision, volume
/// conservation, cohesion). The original is untouched.
///
/// The clone is intentionally **collision-decoupled** (`collide = false`): proxy
/// colliders live in the world's proxy table keyed by `SoftBodyId`, not inside the
/// body, so a copied `collide == true` would have no proxies to drive it and would
/// freeze. Call `soft_body_enable_collision` on the new id to rebuild proxies if the
/// clone needs collision response.
///
/// Returns the new body id, or `u32::MAX` if `world` is null or `id` is unknown.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_clone(world: *mut WorldHandle, id: u32) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_clone: world is null");
            return u32::MAX;
        };
        let sid = SoftBodyId(id);
        let Some(src) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_clone: unknown id");
            return u32::MAX;
        };
        let mut cloned = src.clone();
        // Standalone clone: disable collision coupling so it self-integrates (its id
        // has no proxy colliders in the world's proxy table yet).
        cloned.collide = false;
        let new_id = world.inner.soft_bodies.insert(cloned);
        clear_error();
        new_id.0
    })
}

/// Add a particle to a soft body.
///
/// * `mass` — particle mass (> 0, finite). Ignored when `pinned` is non-zero
///   (a pinned particle has infinite mass / `inv_mass = 0` and acts as an anchor).
/// * `x/y/z` — initial world position (finite).
///
/// # Returns
/// The particle index (as `u32`) on success, or `u32::MAX` on error (`ERR_*`).
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_add_particle(
    world: *mut WorldHandle,
    id: u32,
    x: f64,
    y: f64,
    z: f64,
    mass: f64,
    pinned: Bool,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_add_particle: world is null");
            return u32::MAX;
        };
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            set_error(ERR_INVALID_ARGUMENT, "soft_body_add_particle: bad position");
            return u32::MAX;
        }
        if pinned == Bool::FALSE && (!mass.is_finite() || mass <= 0.0) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_add_particle: mass must be > 0",
            );
            return u32::MAX;
        }
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_add_particle: unknown id");
            return u32::MAX;
        };
        let pos = Vector::new(x, y, z);
        let idx = if pinned == Bool::TRUE {
            body.add_pinned(pos)
        } else {
            let i = body.add_particle(pos);
            body.particles[i].inv_mass = 1.0 / mass;
            i
        };
        clear_error();
        idx as u32
    })
}

/// Add a Hookean spring (edge) between two particles of a soft body.
///
/// Used by the `MassSpring` solver. Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_add_spring(
    world: *mut WorldHandle,
    id: u32,
    a: u32,
    b: u32,
    stiffness: f64,
    damping: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_add_spring: world is null");
            return Bool::FALSE;
        };
        if !stiffness.is_finite() || stiffness < 0.0 || !damping.is_finite() || damping < 0.0 {
            set_error(ERR_INVALID_ARGUMENT, "soft_body_add_spring: bad params");
            return Bool::FALSE;
        }
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_add_spring: unknown id");
            return Bool::FALSE;
        };
        match body.add_spring(a as usize, b as usize, stiffness, damping) {
            Some(_) => {
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "soft_body_add_spring: bad particle indices",
                );
                Bool::FALSE
            }
        }
    })
}

/// Add an XPBD distance constraint (edge) between two particles.
///
/// Used by the `Xpbd` solver; switch the body to XPBD first with
/// [`soft_body_configure_solver`]. Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_add_distance_constraint(
    world: *mut WorldHandle,
    id: u32,
    a: u32,
    b: u32,
    compliance: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_add_distance_constraint: world is null",
            );
            return Bool::FALSE;
        };
        if !compliance.is_finite() || compliance < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_add_distance_constraint: bad compliance",
            );
            return Bool::FALSE;
        }
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(
                ERR_NOT_FOUND,
                "soft_body_add_distance_constraint: unknown id",
            );
            return Bool::FALSE;
        };
        match body.add_distance_constraint(a as usize, b as usize, compliance) {
            Some(_) => {
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "soft_body_add_distance_constraint: bad particle indices",
                );
                Bool::FALSE
            }
        }
    })
}

/// Add a tetrahedral volume element `[a, b, c, d]` to a soft body.
///
/// Used by the `Xpbd` solver's volume-preservation constraint; the rest
/// (reference) signed volume is cached at add time. Returns `Bool::TRUE` on
/// success.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_add_tetrahedron(
    world: *mut WorldHandle,
    id: u32,
    a: u32,
    b: u32,
    c: u32,
    d: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_add_tetrahedron: world is null");
            return Bool::FALSE;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_add_tetrahedron: unknown id");
            return Bool::FALSE;
        };
        match body.add_tetrahedron([a, b, c, d]) {
            Some(_) => {
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "soft_body_add_tetrahedron: bad/degenerate indices",
                );
                Bool::FALSE
            }
        }
    })
}

/// # Phase 21 - adaptive tetrahedral subdivision (1 -> 4 barycentric split).
///
/// Inserts one new particle at the centroid of each source tetrahedron and replaces
/// it with four sub-tetrahedra sharing that centroid. The four sub-volumes sum to the
/// parent volume, so the XPBD volume-conservation constraint (Phase 16) stays
/// consistent; the centroid is a vertex of every sub-tet, so no extra distance edges
/// are added (that would over-constrain the solve). A source tet is split only when
/// its longest edge exceeds `max_edge_len`; pass a non-finite value to subdivide all.
/// The shell topology (`triangles`) is left untouched (volumetric refinement only).
/// Returns the number of source tetrahedra actually split (0 if none qualified).
/// Unknown id or a body with no tetrahedra returns 0 with no side effect.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_subdivide_tetrahedra(
    world: *mut WorldHandle,
    id: u32,
    max_edge_len: f64,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_subdivide_tetrahedra: world is null",
            );
            return 0;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_subdivide_tetrahedra: unknown id");
            return 0;
        };
        let n = body.subdivide_tetrahedra(max_edge_len);
        clear_error();
        n as u32
    })
}

/// Phase 6 — cloth: add a triangular face `[a, b, c]` to a soft body's shell
/// topology. The three structural edges are registered automatically as
/// distance constraints (rest length from current spacing); duplicate edges
/// shared with neighbouring triangles are de-duplicated inside rapier. Bending
/// is composed separately by the caller via `soft_body_add_bending` (a single
/// cross-diagonal distance constraint) — no new mechanics, fully reusing the
/// existing XPBD distance solver.
///
/// Returns `Bool::TRUE` on success. `Bool::FALSE` if the body/id is unknown, an
/// index is out of bounds or duplicated, or the face is degenerate (a zero-length
/// edge).
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_add_triangle(
    world: *mut WorldHandle,
    id: u32,
    a: u32,
    b: u32,
    c: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_add_triangle: world is null");
            return Bool::FALSE;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_add_triangle: unknown id");
            return Bool::FALSE;
        };
        match body.add_triangle([a, b, c]) {
            Some(_) => {
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "soft_body_add_triangle: bad/degenerate indices",
                );
                Bool::FALSE
            }
        }
    })
}

/// Phase 6 — cloth: add a single bending edge between particles `p` and `q` as
/// a distance constraint (rest length from current spacing). Compose bending
/// across a quad by calling this for its two diagonals, or across a fold line by
/// linking the un-shared vertices of two adjacent triangles. Reuses the existing
/// XPBD distance solver (no new mechanics).
///
/// Returns `Bool::TRUE` on success. `Bool::FALSE` if the body/id is unknown, an
/// index is out of bounds, or the endpoints coincide.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_add_bending(world: *mut WorldHandle, id: u32, p: u32, q: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_add_bending: world is null");
            return Bool::FALSE;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_add_bending: unknown id");
            return Bool::FALSE;
        };
        match body.add_bending_constraint(p as usize, q as usize) {
            Some(_) => {
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "soft_body_add_bending: bad/coincident indices",
                );
                Bool::FALSE
            }
        }
    })
}

/// Switch a soft body's solver.
///
/// * `solver_mode` — `0` = `MassSpring` (Hookean springs, semi-implicit Euler);
///   `1` = `Xpbd { iterations, compliance }` (position-based distance + volume
///   constraints).
/// * `iterations` — XPBD Gauss-Seidel iterations (> 0 when `solver_mode == 1`).
/// * `compliance` — XPBD default compliance (≥ 0, finite).
///
/// Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_configure_solver(
    world: *mut WorldHandle,
    id: u32,
    solver_mode: u32,
    iterations: u32,
    compliance: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_configure_solver: world is null",
            );
            return Bool::FALSE;
        };
        if !compliance.is_finite() || compliance < 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_configure_solver: bad compliance",
            );
            return Bool::FALSE;
        }
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_configure_solver: unknown id");
            return Bool::FALSE;
        };
        match solver_mode {
            0 => {
                body.solver = SoftSolver::MassSpring;
                clear_error();
                Bool::TRUE
            }
            1 => {
                if iterations == 0 {
                    set_error(
                        ERR_INVALID_ARGUMENT,
                        "soft_body_configure_solver: iterations must be > 0",
                    );
                    return Bool::FALSE;
                }
                body.configure_xpbd(iterations, compliance);
                clear_error();
                Bool::TRUE
            }
            _ => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "soft_body_configure_solver: unknown solver_mode",
                );
                Bool::FALSE
            }
        }
    })
}

// ── Phase 5c: 生物软体（复用 Phase 3 四面体 XPBD 体积约束）──────────────────
//
// `soft_body_build_tetra_mesh` 一次性灌入质点 + 四面体拓扑，自动为每个四面体
// 的 6 条边建 XPBD 距离约束（去重），并切到 Xpbd 求解器。体积约束在
// `SoftBody::step_xpbd` 内随四面体自动激活 —— 于是得到一个「史莱姆 / 水母」式
// 可压缩可回弹的软体生物，无需任何新物理。
// 该函数纯包裹 rapier 既有 API，不改动 fork。

/// Build a tetrahedral-mesh soft body from raw particle positions and tetrahedra,
/// then switch it to the XPBD solver so the volume constraints are active.
///
/// `particles` is a `particles_len`-long array of `Vec3`; `tets` is a flat array
/// of `tets_len * 4` `u32` vertex indices (`[a,b,c,d, a,b,c,d, ...]`). For every
/// tetrahedron, its 6 edges are added as XPBD distance constraints (deduplicated
/// across shared edges). Finally the body is configured with `iterations`/`compliance`.
///
/// Returns the new `SoftBodyId` (as `u32`) or `u32::MAX` on error.
///
/// # Safety
/// `world` must be a valid world pointer. `particles`/`tets` must point to arrays
/// of at least `particles_len` / `tets_len*4` elements respectively.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_build_tetra_mesh(
    world: *mut WorldHandle,
    gravity: Vec3,
    particles: *const Vec3,
    particles_len: u32,
    tets: *const u32,
    tets_len: u32,
    particle_mass: f64,
    compliance: f64,
    iterations: u32,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_build_tetra_mesh: world is null",
            );
            return u32::MAX;
        };
        if particles.is_null() || particles_len == 0 || tets.is_null() || tets_len == 0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_build_tetra_mesh: empty input",
            );
            return u32::MAX;
        }
        if !vec3_finite(gravity)
            || !particle_mass.is_finite()
            || particle_mass <= 0.0
            || !compliance.is_finite()
            || compliance < 0.0
            || iterations == 0
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_build_tetra_mesh: bad gravity/mass/compliance/iterations",
            );
            return u32::MAX;
        }
        let plist = unsafe { std::slice::from_raw_parts(particles, particles_len as usize) };
        let tlist = unsafe { std::slice::from_raw_parts(tets, tets_len as usize * 4) };

        let mut body = SoftBody::new(vec3_to_rapier(gravity));
        // Add particles (all dynamic; caller may pin later via soft_body_add_particle
        // is not applicable to existing particles, so pins are added up-front by the
        // caller through a separate pinned-particle path if needed).
        for p in plist {
            if !vec3_finite(*p) {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "soft_body_build_tetra_mesh: non-finite particle",
                );
                return u32::MAX;
            }
            let i = body.add_particle(vec3_to_rapier(*p));
            body.particles[i].inv_mass = 1.0 / particle_mass;
        }

        // Add tetrahedra + their 6 edges as XPBD distance constraints (dedup edges).
        let mut edges: HashSet<(u32, u32)> = HashSet::new();
        let mut tet_fail = false;
        for t in tlist.as_chunks::<4>().0 {
            let tet = [t[0], t[1], t[2], t[3]];
            if body.add_tetrahedron(tet).is_none() {
                tet_fail = true;
                break;
            }
            for (a, b) in [
                (tet[0], tet[1]),
                (tet[0], tet[2]),
                (tet[0], tet[3]),
                (tet[1], tet[2]),
                (tet[1], tet[3]),
                (tet[2], tet[3]),
            ] {
                let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
                edges.insert((lo, hi));
            }
        }
        if tet_fail {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_build_tetra_mesh: bad tetrahedron indices",
            );
            return u32::MAX;
        }
        for (lo, hi) in edges {
            // compliance propagates to edges too; rest length captured from geometry.
            body.add_distance_constraint(lo as usize, hi as usize, compliance);
        }

        body.configure_xpbd(iterations, compliance);
        let id = world.inner.soft_bodies.insert(body);
        clear_error();
        id.0
    })
}

// ── Phase 33: 绳索 / 发丝软体构造器（纯组合层，零 fork 改动）─────────────────
//
// 复用 Phase 5a 原语（`soft_body_create` + `add_particle` + `add_distance_constraint`
// + `configure_solver`），沿给定首尾方向线性布 N 个质点，相邻连 XPBD 距离约束，
// 得到一个可悬垂 / 可摆动 / 可闭合环的绳索或发丝。弯曲约束（相邻三点的跨边
// 距离约束）让发丝抗折、更像一束头发而非一串珠子。无任何新物理。

/// Build a rope / hair strand soft body from a start point to an end point.
///
/// * `start_x/y/z` / `end_x/y/z` — the two endpoints of the strand (finite).
///   The `n` particles are placed at uniform `t = i/(n-1)` interpolation
///   (`i ∈ [0, n)`), so the strand is straight at rest.
/// * `n` — particle count; must be `>= 2`.
/// * `particle_mass` — mass of each (dynamic) particle (`> 0`, finite).
/// * `compliance` / `iterations` — XPBD stretch parameters for the segment
///   edges (and the bending edges when `bending != 0`).
/// * `pin_start` / `pin_end` — when non-zero, clamp that endpoint's particle to
///   infinite mass (anchor). A hanging rope uses `pin_start = 1, pin_end = 0`;
///   a free strand uses both `0`.
/// * `closed` — when non-zero, the strand is a closed loop: an extra edge links
///   the last particle back to the first (and, with `bending`, the wrap-around
///   bending edge too). Useful for necklaces / rings.
/// * `bending` — when non-zero, every adjacent triple gets a bending distance
///   constraint across its outer particles (rest length from the straight rest
///   spacing), giving the strand resistance to sharp folding (hair-like).
///
/// The body is switched to the XPBD solver automatically. Returns the new
/// `SoftBodyId` (as `u32`) or `u32::MAX` on error (`ERR_*`).
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_build_rope(
    world: *mut WorldHandle,
    start_x: f64,
    start_y: f64,
    start_z: f64,
    end_x: f64,
    end_y: f64,
    end_z: f64,
    n: u32,
    particle_mass: f64,
    compliance: f64,
    iterations: u32,
    pin_start: u8,
    pin_end: u8,
    closed: u8,
    bending: u8,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_build_rope: world is null");
            return u32::MAX;
        };
        if !start_x.is_finite()
            || !start_y.is_finite()
            || !start_z.is_finite()
            || !end_x.is_finite()
            || !end_y.is_finite()
            || !end_z.is_finite()
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_build_rope: non-finite endpoint",
            );
            return u32::MAX;
        }
        if n < 2
            || !particle_mass.is_finite()
            || particle_mass <= 0.0
            || !compliance.is_finite()
            || compliance < 0.0
            || iterations == 0
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_build_rope: bad n/mass/compliance/iterations",
            );
            return u32::MAX;
        }
        let n = n as usize;
        let s = Vector::new(start_x, start_y, start_z);
        let e = Vector::new(end_x, end_y, end_z);
        // Uniform straight rest layout.
        let mut body = SoftBody::new(Vector::ZERO);
        for i in 0..n {
            let t = if n == 1 {
                0.0
            } else {
                i as f64 / (n - 1) as f64
            };
            let pos = s + (e - s) * t;
            // Pin endpoints only when clamping is requested for that end.
            let pin_this = (i == 0 && pin_start != 0) || (i == n - 1 && pin_end != 0);
            let idx = if pin_this {
                body.add_pinned(pos)
            } else {
                let j = body.add_particle(pos);
                body.particles[j].inv_mass = 1.0 / particle_mass;
                j
            };
            debug_assert_eq!(idx, i);
        }
        // Stretch edges (adjacent pairs; wrap for closed loop).
        let seg_pairs: Vec<(usize, usize)> = {
            let mut pairs = Vec::with_capacity(n);
            for i in 0..(n - 1) {
                pairs.push((i, i + 1));
            }
            if closed != 0 {
                pairs.push((n - 1, 0));
            }
            pairs
        };
        // Bending edges: outer particles of each adjacent triple (i, i+1, i+2);
        // wrap the last for a closed loop so hair folds resist uniformly.
        let bend_pairs: Vec<(usize, usize)> = {
            let mut pairs = Vec::with_capacity(n.max(1));
            for i in 0..(n.saturating_sub(2)) {
                pairs.push((i, i + 2));
            }
            if closed != 0 && n >= 3 {
                pairs.push((n - 2, 0));
                pairs.push((n - 1, 1));
            }
            pairs
        };
        for (a, b) in &seg_pairs {
            body.add_distance_constraint(*a, *b, compliance);
        }
        if bending != 0 {
            for (a, b) in &bend_pairs {
                body.add_distance_constraint(*a, *b, compliance);
            }
        }
        body.configure_xpbd(iterations, compliance);
        let id = world.inner.soft_bodies.insert(body);
        clear_error();
        id.0
    })
}

// ── Phase 34: 网格 / 方块软体构造器（纯组合层，零 fork 改动）─────────────────
//
// 在长方体 `[min, max]` 范围内按 `nx × ny × nz` 分辨率布质点网格，相邻质点（6
// 邻接：面相邻）连 XPBD 距离约束。可选 pin 整个外边界（pin_boundary != 0），
// 得到一个可整体形变 / 可破坏 / 可下垂的果冻、橡胶块、地形体。与 rope 同构，
// 完全复用 Phase 5a 原语，无任何新物理。

/// Build a regular grid / block soft body filling the axis-aligned box
/// `[min_*, max_*]` with `nx × ny × nz` particles spaced uniformly.
///
/// * `min_*` / `max_*` — box extents (all finite, `max_* > min_*` per axis).
/// * `nx` / `ny` / `nz` — particle counts per axis; each must be `>= 1`.
///   Total particle count = `nx * ny * nz` (capped to avoid runaway allocation:
///   rejects if `> 1_000_000`).
/// * `particle_mass` — mass of each (dynamic) particle (`> 0`, finite).
/// * `compliance` / `iterations` — XPBD stretch parameters for the grid edges.
/// * `pin_boundary` — when non-zero, every particle on the outer surface of the
///   grid (any index at `0` or `n-1` on any axis) is pinned to infinite mass,
///   so the block hangs/sits from its boundary like a fixed jelly mould.
///
/// Face-adjacent neighbours (6-connectivity) are linked by XPBD distance
/// constraints (de-duplicated). The body is switched to the XPBD solver
/// automatically. Returns the new `SoftBodyId` (as `u32`) or `u32::MAX` on error.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_build_grid(
    world: *mut WorldHandle,
    min_x: f64,
    min_y: f64,
    min_z: f64,
    max_x: f64,
    max_y: f64,
    max_z: f64,
    nx: u32,
    ny: u32,
    nz: u32,
    particle_mass: f64,
    compliance: f64,
    iterations: u32,
    pin_boundary: u8,
) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_build_grid: world is null");
            return u32::MAX;
        };
        if !min_x.is_finite()
            || !min_y.is_finite()
            || !min_z.is_finite()
            || !max_x.is_finite()
            || !max_y.is_finite()
            || !max_z.is_finite()
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_build_grid: non-finite extents",
            );
            return u32::MAX;
        }
        if max_x <= min_x || max_y <= min_y || max_z <= min_z {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_build_grid: empty/inverted box",
            );
            return u32::MAX;
        }
        if nx == 0 || ny == 0 || nz == 0 || !particle_mass.is_finite() || particle_mass <= 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_build_grid: bad resolution/mass",
            );
            return u32::MAX;
        }
        if !compliance.is_finite() || compliance < 0.0 || iterations == 0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_build_grid: bad compliance/iterations",
            );
            return u32::MAX;
        }
        let (nx, ny, nz) = (nx as usize, ny as usize, nz as usize);
        let total = nx as u64 * ny as u64 * nz as u64;
        if total > 1_000_000 {
            set_error(
                ERR_CAPACITY,
                "soft_body_build_grid: too many particles (>1M)",
            );
            return u32::MAX;
        }

        let sx = (max_x - min_x) / (nx as f64 - 1.0).max(1.0);
        let sy = (max_y - min_y) / (ny as f64 - 1.0).max(1.0);
        let sz = (max_z - min_z) / (nz as f64 - 1.0).max(1.0);

        let mut body = SoftBody::new(Vector::ZERO);
        // Linear index: i + j*nx + k*nx*ny.
        let idx = |i: usize, j: usize, k: usize| i + j * nx + k * nx * ny;
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let pos = Vector::new(
                        min_x + sx * i as f64,
                        min_y + sy * j as f64,
                        min_z + sz * k as f64,
                    );
                    let on_boundary =
                        i == 0 || i == nx - 1 || j == 0 || j == ny - 1 || k == 0 || k == nz - 1;
                    let pin_this = pin_boundary != 0 && on_boundary && (nx > 1 || ny > 1 || nz > 1);
                    let p = if pin_this {
                        body.add_pinned(pos)
                    } else {
                        let p = body.add_particle(pos);
                        body.particles[p].inv_mass = 1.0 / particle_mass;
                        p
                    };
                    debug_assert_eq!(p, idx(i, j, k));
                }
            }
        }

        // 6-connectivity face edges, de-duplicated.
        let mut edges: HashSet<(u32, u32)> = HashSet::new();
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let a = idx(i, j, k) as u32;
                    if i + 1 < nx {
                        let b = idx(i + 1, j, k) as u32;
                        edges.insert((a.min(b), a.max(b)));
                    }
                    if j + 1 < ny {
                        let b = idx(i, j + 1, k) as u32;
                        edges.insert((a.min(b), a.max(b)));
                    }
                    if k + 1 < nz {
                        let b = idx(i, j, k + 1) as u32;
                        edges.insert((a.min(b), a.max(b)));
                    }
                }
            }
        }
        for (lo, hi) in edges {
            body.add_distance_constraint(lo as usize, hi as usize, compliance);
        }

        body.configure_xpbd(iterations, compliance);
        let id = world.inner.soft_bodies.insert(body);
        clear_error();
        id.0
    })
}

// ── Phase 5b: query / readback / lifecycle FFI (close the loop) ──────────────
//
// Phase 5a 让外部能「搭」软体；本组让外部能「查 / 读 / 删 / 毁」——
// 这是 Minecraft 联动（读回质点渲染、区块破坏删质点、实体消失毁软体）必需的闭环。
// 返回 id 类沿用 `u32::MAX` 哨兵；布尔类沿用 `Bool::FALSE`；`SoftBodyId` 经 rapier
// `SoftBodySet::remove` 走 tombstone，删除后其余 id 仍有效。

/// Number of live soft bodies in the world.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_count(world: *const WorldHandle) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_count: world is null");
            return 0;
        };
        clear_error();
        world.inner.soft_bodies.count() as u32
    })
}

/// Number of particles in a soft body. Returns `u32::MAX` for an unknown id.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_particle_count(world: *const WorldHandle, id: u32) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_particle_count: world is null");
            return u32::MAX;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_particle_count: unknown id");
            return u32::MAX;
        };
        clear_error();
        body.particles.len() as u32
    })
}

/// Read back a particle's position and velocity.
///
/// `out_pos` / `out_vel` must point to writable `Vec3`; either may be null to
/// skip that output. Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be a valid world pointer; `out_pos`/`out_vel` (if non-null) must
/// point to writable `Vec3`.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_get_particle(
    world: *const WorldHandle,
    id: u32,
    index: u32,
    out_pos: *mut Vec3,
    out_vel: *mut Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_get_particle: world is null");
            return Bool::FALSE;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_get_particle: unknown id");
            return Bool::FALSE;
        };
        let i = index as usize;
        let Some(p) = body.particles.get(i) else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_get_particle: index out of bounds",
            );
            return Bool::FALSE;
        };
        if !out_pos.is_null() {
            unsafe {
                *out_pos = vec3_from_rapier(p.pos);
            }
        }
        if !out_vel.is_null() {
            unsafe {
                *out_vel = vec3_from_rapier(p.vel);
            }
        }
        clear_error();
        Bool::TRUE
    })
}

/// Set a single particle's linear velocity to `(vx, vy, vz)`, overwriting it.
///
/// Pinned particles (`inv_mass == 0`) are skipped — their velocity is meaningless
/// because the integrator reseeds it from the bound rigid body every step, so this
/// returns `Bool::FALSE` for them. `Err::FALSE` is also returned for a null world,
/// an unknown body id, or an out-of-range `index`. On success the particle's `vel`
/// field is updated in place and `Bool::TRUE` is returned.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_particle_velocity(
    world: *mut WorldHandle,
    id: u32,
    index: u32,
    vx: f64,
    vy: f64,
    vz: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_set_particle_velocity: world is null",
            );
            return Bool::FALSE;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_particle_velocity: unknown id");
            return Bool::FALSE;
        };
        let idx = index as usize;
        if idx >= body.particles.len() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_particle_velocity: index out of range",
            );
            return Bool::FALSE;
        }
        if body.particles[idx].inv_mass == 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_particle_velocity: particle is pinned",
            );
            return Bool::FALSE;
        }
        body.particles[idx].vel = Vector {
            x: vx,
            y: vy,
            z: vz,
        };
        clear_error();
        Bool::TRUE
    })
}

/// Remove a particle (and every spring / distance constraint / tetrahedron that
/// references it) from a soft body, keeping the remaining topology valid.
/// Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_remove_particle(world: *mut WorldHandle, id: u32, index: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_remove_particle: world is null");
            return Bool::FALSE;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_remove_particle: unknown id");
            return Bool::FALSE;
        };
        if body.remove_particle(index as usize) {
            clear_error();
            Bool::TRUE
        } else {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_remove_particle: index out of bounds",
            );
            Bool::FALSE
        }
    })
}

/// Apply a linear impulse to a single soft-body particle.
///
/// The impulse `J = (fx, fy, fz)` changes the particle velocity by `J * inv_mass`,
/// i.e. `p.vel += J * p.inv_mass`. For collision-coupled bodies the updated velocity
/// is pushed into the particle's proxy rigid body at the next step (see the
/// soft-body/rigid-body coupling loop), so a contact reaction naturally follows; for
/// non-coupled bodies the fork integrator consumes `p.vel` directly. Pinned particles
/// (`inv_mass == 0`, e.g. anchors) are unaffected. This is the primitive for
/// grab/poke/kick interactions on a single vertex. Pure state mutation: no solver
/// structural change.
///
/// Returns `Bool::TRUE` on success, `Bool::FALSE` if `world` is null, `id` is unknown,
/// `index` is out of bounds, or any component of the impulse is non-finite.
///
/// # Safety
/// `world` must be a valid world pointer.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_apply_particle_impulse(
    world: *mut WorldHandle,
    id: u32,
    index: u32,
    fx: f64,
    fy: f64,
    fz: f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_apply_particle_impulse: world is null",
            );
            return Bool::FALSE;
        };
        if !fx.is_finite() || !fy.is_finite() || !fz.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_apply_particle_impulse: impulse must be finite",
            );
            return Bool::FALSE;
        }
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(
                ERR_NOT_FOUND,
                "soft_body_apply_particle_impulse: unknown id",
            );
            return Bool::FALSE;
        };
        let i = index as usize;
        if i >= body.particles.len() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_apply_particle_impulse: index out of bounds",
            );
            return Bool::FALSE;
        }
        let p = &mut body.particles[i];
        let inv = p.inv_mass;
        // inv_mass == 0 (pinned) → no velocity change, but still a valid op.
        p.vel += Vector::new(fx * inv, fy * inv, fz * inv);
        clear_error();
        Bool::TRUE
    })
}

/// Read the axis-aligned bounding box (min/max corners) and centroid of a soft body.
///
/// Computes the AABB and the per-particle average position (`centroid`) from the
/// body's current particle positions. Useful for frustum culling, broad-phase
/// spatial queries, LOD, and nearest-neighbour tests against other bodies. Pure
/// read-out: does not affect the solver. Bodies with zero particles return
/// `Bool::FALSE` (the box is undefined).
///
/// Any of `out_min`/`out_max`/`out_centroid` may be null to skip that output.
///
/// Returns `Bool::TRUE` on success, `Bool::FALSE` if `world` is null, `id` is
/// unknown, or the body has no particles.
///
/// # Safety
/// `world` must be a valid world pointer; non-null output pointers must each target
/// a writable `Vec3`.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_read_aabb(
    world: *const WorldHandle,
    id: u32,
    out_min: *mut Vec3,
    out_max: *mut Vec3,
    out_centroid: *mut Vec3,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_read_aabb: world is null");
            return Bool::FALSE;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_read_aabb: unknown id");
            return Bool::FALSE;
        };
        if body.particles.is_empty() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_read_aabb: body has no particles",
            );
            return Bool::FALSE;
        }
        let mut min = body.particles[0].pos;
        let mut max = body.particles[0].pos;
        let mut sum = Vector::ZERO;
        for p in body.particles.iter() {
            min = min.min(p.pos);
            max = max.max(p.pos);
            sum += p.pos;
        }
        let n = body.particles.len() as f64;
        let centroid = sum / n;
        if !out_min.is_null() {
            unsafe { *out_min = vec3_from_rapier(min) };
        }
        if !out_max.is_null() {
            unsafe { *out_max = vec3_from_rapier(max) };
        }
        if !out_centroid.is_null() {
            unsafe { *out_centroid = vec3_from_rapier(centroid) };
        }
        clear_error();
        Bool::TRUE
    })
}

/// Destroy a soft body, freeing its storage. Other live `SoftBodyId`s remain
/// valid (the id slot becomes a tombstone). Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be a valid world pointer.
/// Phase 25 #5: hand-rolled binary (de)serialization of a [`SoftBody`].
///
/// `SoftBody` does not derive `Serialize` (no `serde` feature on the fork), so we
/// walk its public fields directly. The format is a fixed little-endian `u8` blob:
///
/// ```text
/// MAGIC[2] = b"SB" | VERSION: u8 (1) | COUNT fields...
/// ```
///
/// Every scalar is `f64` (rapier3d-f64 `Real`) written as 8 little-endian bytes;
/// `u32` as 4 LE bytes; `bool` as 1 byte; `Vector` as 3×`f64`. `Option<T>` is a 1
/// byte tag (0 = None, 1 = Some) followed by the payload. `SoftSolver` is 1 byte
/// (0 = MassSpring, 1 = Xpbd) + the Xpbd payload when present. `RigidBodyHandle`
/// (inside `SoftParticle::bound_body`) packs to `(u32 id, u32 generation)` via
/// `into_raw_parts`.
///
/// The blob is written into a caller-supplied `*mut u8` buffer of `capacity`
/// bytes; `soft_body_state_size` returns the exact size needed so the caller can
/// allocate. `soft_body_restore_state` rebuilds a body at a (possibly new) id and
/// returns `Bool::FALSE` on buffer underflow / magic mismatch so a corrupt blob
/// cannot produce a half-built body.
/// Pure mps-core: it only ever touches the `SoftBody`'s public fields via
/// `SoftBodySet`'s `get`/`get_mut`. The fork is unchanged.
const SB_MAGIC_0: u8 = b'S';
const SB_MAGIC_1: u8 = b'B';
const SB_VERSION: u8 = 2;

// ── little-endian writers ───────────────────────────────────────────────────────
fn sb_push_u8(buf: &mut Vec<u8>, v: u8) {
    buf.push(v);
}
fn sb_push_u32(buf: &mut Vec<u8>, v: u32) {
    buf.extend_from_slice(&v.to_le_bytes());
}
fn sb_push_f64(buf: &mut Vec<u8>, v: f64) {
    buf.extend_from_slice(&v.to_le_bytes());
}
fn sb_push_bool(buf: &mut Vec<u8>, v: bool) {
    buf.push(if v { 1 } else { 0 });
}
fn sb_push_vec(buf: &mut Vec<u8>, v: Vector) {
    sb_push_f64(buf, v.x);
    sb_push_f64(buf, v.y);
    sb_push_f64(buf, v.z);
}
fn sb_push_option_f64(buf: &mut Vec<u8>, v: Option<f64>) {
    match v {
        Some(x) => {
            sb_push_u8(buf, 1);
            sb_push_f64(buf, x);
        }
        None => sb_push_u8(buf, 0),
    }
}
fn sb_push_option_vec3(buf: &mut Vec<u8>, v: Option<Vector>) {
    match v {
        Some(v) => {
            sb_push_u8(buf, 1);
            sb_push_vec(buf, v);
        }
        None => sb_push_u8(buf, 0),
    }
}

// ── little-endian readers ────────────────────────────────────────────────────────
struct SbCursor<'a> {
    data: &'a [u8],
    pos: usize,
}
impl<'a> SbCursor<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }
    fn take_u8(&mut self) -> Option<u8> {
        let b = *self.data.get(self.pos)?;
        self.pos += 1;
        Some(b)
    }
    fn take_u32(&mut self) -> Option<u32> {
        let s = self.data.get(self.pos..self.pos + 4)?;
        self.pos += 4;
        Some(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn take_f64(&mut self) -> Option<f64> {
        let s = self.data.get(self.pos..self.pos + 8)?;
        self.pos += 8;
        Some(f64::from_le_bytes([
            s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7],
        ]))
    }
    fn take_bool(&mut self) -> Option<bool> {
        Some(self.take_u8()? != 0)
    }
    fn take_vec(&mut self) -> Option<Vector> {
        let x = self.take_f64()?;
        let y = self.take_f64()?;
        let z = self.take_f64()?;
        Some(Vector { x, y, z })
    }
    fn take_option_f64(&mut self) -> Option<Option<f64>> {
        match self.take_u8()? {
            0 => Some(None),
            _ => Some(Some(self.take_f64()?)),
        }
    }
    fn take_option_vec3(&mut self) -> Option<Option<Vector>> {
        match self.take_u8()? {
            0 => Some(None),
            _ => Some(self.take_vec()),
        }
    }
}

fn sb_serialize_particle(buf: &mut Vec<u8>, p: &SoftParticle) {
    sb_push_vec(buf, p.pos);
    sb_push_vec(buf, p.vel);
    sb_push_vec(buf, p.force);
    sb_push_f64(buf, p.inv_mass);
    match &p.bound_body {
        Some(h) => {
            sb_push_u8(buf, 1);
            let (id, genid) = h.into_raw_parts();
            sb_push_u32(buf, id);
            sb_push_u32(buf, genid);
        }
        None => sb_push_u8(buf, 0),
    }
    sb_push_vec(buf, p.bound_local);
}

fn sb_serialize_wind(buf: &mut Vec<u8>, w: &Wind) {
    sb_push_vec(buf, w.accel);
    sb_push_f64(buf, w.drag);
}

fn sb_serialize_plasticity(buf: &mut Vec<u8>, p: &PlasticityParams) {
    sb_push_f64(buf, p.yield_strain);
    sb_push_f64(buf, p.creep);
}

fn sb_serialize_self_collision(buf: &mut Vec<u8>, s: &SelfCollisionParams) {
    sb_push_f64(buf, s.radius);
    sb_push_f64(buf, s.stiffness);
    sb_push_option_f64(buf, s.friction);
}

fn sb_serialize_cohesion(buf: &mut Vec<u8>, c: &CohesionParams) {
    sb_push_f64(buf, c.radius);
    sb_push_f64(buf, c.stiffness);
    sb_push_f64(buf, c.break_distance);
}

fn sb_serialize_body(buf: &mut Vec<u8>, b: &SoftBody) {
    sb_push_u8(buf, SB_MAGIC_0);
    sb_push_u8(buf, SB_MAGIC_1);
    sb_push_u8(buf, SB_VERSION);
    // particles
    sb_push_u32(buf, b.particles.len() as u32);
    for p in b.particles.iter() {
        sb_serialize_particle(buf, p);
    }
    // springs
    sb_push_u32(buf, b.springs.len() as u32);
    for s in b.springs.iter() {
        sb_push_u32(buf, s.a as u32);
        sb_push_u32(buf, s.b as u32);
        sb_push_f64(buf, s.rest_length);
        sb_push_f64(buf, s.stiffness);
        sb_push_f64(buf, s.damping);
        sb_push_f64(buf, s.activation);
        sb_push_option_vec3(buf, s.fibre);
    }
    // distance constraints
    sb_push_u32(buf, b.distance_constraints.len() as u32);
    for d in b.distance_constraints.iter() {
        sb_push_u32(buf, d.a as u32);
        sb_push_u32(buf, d.b as u32);
        sb_push_f64(buf, d.rest);
        sb_push_f64(buf, d.compliance);
        sb_push_f64(buf, d.compression);
        sb_push_f64(buf, d.activation);
        sb_push_option_vec3(buf, d.fibre);
    }
    // tetrahedra (+ rest volumes)
    sb_push_u32(buf, b.tetrahedra.len() as u32);
    for t in b.tetrahedra.iter() {
        sb_push_u32(buf, t[0]);
        sb_push_u32(buf, t[1]);
        sb_push_u32(buf, t[2]);
        sb_push_u32(buf, t[3]);
    }
    sb_push_u32(buf, b.tetra_rest_volumes.len() as u32);
    for v in b.tetra_rest_volumes.iter() {
        sb_push_f64(buf, *v);
    }
    // triangles
    sb_push_u32(buf, b.triangles.len() as u32);
    for t in b.triangles.iter() {
        sb_push_u32(buf, t[0]);
        sb_push_u32(buf, t[1]);
        sb_push_u32(buf, t[2]);
    }
    // solver
    match b.solver {
        SoftSolver::MassSpring => sb_push_u8(buf, 0),
        SoftSolver::Xpbd {
            iterations,
            compliance,
        } => {
            sb_push_u8(buf, 1);
            sb_push_u32(buf, iterations);
            sb_push_f64(buf, compliance);
        }
    }
    sb_push_vec(buf, b.gravity);
    sb_push_bool(buf, b.sleeping);
    sb_push_bool(buf, b.collide);
    sb_push_f64(buf, b.particle_radius);
    // wind
    match &b.wind {
        Some(w) => {
            sb_push_u8(buf, 1);
            sb_serialize_wind(buf, w);
        }
        None => sb_push_u8(buf, 0),
    }
    // pressure
    sb_push_option_f64(buf, b.pressure);
    sb_push_f64(buf, b.damping);
    sb_push_u32(buf, b.substeps);
    // tear criterion (Phase 27: enum Strain/Stress/Energy)
    match &b.tear {
        Some(TearCriterion::Strain(t)) => {
            sb_push_u8(buf, 0);
            sb_push_f64(buf, *t);
        }
        Some(TearCriterion::Stress(t)) => {
            sb_push_u8(buf, 1);
            sb_push_f64(buf, *t);
        }
        Some(TearCriterion::Energy(t)) => {
            sb_push_u8(buf, 2);
            sb_push_f64(buf, *t);
        }
        None => sb_push_u8(buf, 0xff),
    }
    // anisotropy (Phase 27: optional orthotropic axes)
    match &b.anisotropy {
        Some(v) => {
            sb_push_u8(buf, 1);
            sb_push_vec(buf, *v);
        }
        None => sb_push_u8(buf, 0),
    }
    // viscoelastic (Phase 27: rate-dependent constitutive)
    match &b.viscoelastic {
        Some(v) => {
            sb_push_u8(buf, 1);
            sb_push_f64(buf, v.rate_coefficient);
        }
        None => sb_push_u8(buf, 0),
    }
    // temperature (Phase 27: uniform thermal field)
    match &b.temperature {
        Some(th) => {
            sb_push_u8(buf, 1);
            sb_push_f64(buf, th.temp);
            sb_push_f64(buf, th.ambient);
            sb_push_f64(buf, th.expansion);
            sb_push_f64(buf, th.stiffness_temp_coeff);
        }
        None => sb_push_u8(buf, 0),
    }
    // plasticity
    match &b.plasticity {
        Some(p) => {
            sb_push_u8(buf, 1);
            sb_serialize_plasticity(buf, p);
        }
        None => sb_push_u8(buf, 0),
    }
    // self_collision
    match &b.self_collision {
        Some(s) => {
            sb_push_u8(buf, 1);
            sb_serialize_self_collision(buf, s);
        }
        None => sb_push_u8(buf, 0),
    }
    // cross_collision
    match &b.cross_collision {
        Some(s) => {
            sb_push_u8(buf, 1);
            sb_serialize_self_collision(buf, s);
        }
        None => sb_push_u8(buf, 0),
    }
    // volume_conservation
    sb_push_option_f64(buf, b.volume_conservation);
    // cohesion
    match &b.cohesion {
        Some(c) => {
            sb_push_u8(buf, 1);
            sb_serialize_cohesion(buf, c);
        }
        None => sb_push_u8(buf, 0),
    }
    // corotated (Phase 29): Option<f64> stiffness + rest-shape matrices.
    match b.corotated {
        Some(k) => {
            sb_push_u8(buf, 1);
            sb_push_f64(buf, k);
            sb_push_u32(buf, b.tetra_rest_shapes.len() as u32);
            for m in &b.tetra_rest_shapes {
                for row in m.iter() {
                    for v in row.iter() {
                        sb_push_f64(buf, *v);
                    }
                }
            }
        }
        None => sb_push_u8(buf, 0),
    } // neo_hookean (Phase 30)
    sb_push_option_f64(buf, b.neo_hookean);
}

fn sb_deserialize_particle(c: &mut SbCursor) -> Option<SoftParticle> {
    let pos = c.take_vec()?;
    let vel = c.take_vec()?;
    let force = c.take_vec()?;
    let inv_mass = c.take_f64()?;
    let bound_body = match c.take_u8()? {
        0 => None,
        _ => {
            let id = c.take_u32()?;
            let genid = c.take_u32()?;
            Some(RigidBodyHandle::from_raw_parts(id, genid))
        }
    };
    let bound_local = c.take_vec()?;
    Some(SoftParticle {
        pos,
        vel,
        force,
        inv_mass,
        bound_body,
        bound_local,
    })
}

fn sb_deserialize_wind(c: &mut SbCursor) -> Option<Wind> {
    let accel = c.take_vec()?;
    let drag = c.take_f64()?;
    Some(Wind { accel, drag })
}

fn sb_deserialize_plasticity(c: &mut SbCursor) -> Option<PlasticityParams> {
    let yield_strain = c.take_f64()?;
    let creep = c.take_f64()?;
    Some(PlasticityParams {
        yield_strain,
        creep,
    })
}

fn sb_deserialize_self_collision(c: &mut SbCursor) -> Option<SelfCollisionParams> {
    let radius = c.take_f64()?;
    let stiffness = c.take_f64()?;
    let friction = c.take_option_f64()?;
    Some(SelfCollisionParams {
        radius,
        stiffness,
        friction,
    })
}

fn sb_deserialize_cohesion(c: &mut SbCursor) -> Option<CohesionParams> {
    let radius = c.take_f64()?;
    let stiffness = c.take_f64()?;
    let break_distance = c.take_f64()?;
    Some(CohesionParams {
        radius,
        stiffness,
        break_distance,
    })
}

fn sb_deserialize_body(c: &mut SbCursor) -> Option<SoftBody> {
    if c.take_u8()? != SB_MAGIC_0 || c.take_u8()? != SB_MAGIC_1 {
        return None;
    }
    if c.take_u8()? != SB_VERSION {
        return None;
    }
    let np = c.take_u32()? as usize;
    let mut particles = Vec::with_capacity(np);
    for _ in 0..np {
        particles.push(sb_deserialize_particle(c)?);
    }
    let ns = c.take_u32()? as usize;
    let mut springs = Vec::with_capacity(ns);
    for _ in 0..ns {
        springs.push(SoftSpring {
            a: c.take_u32()? as usize,
            b: c.take_u32()? as usize,
            rest_length: c.take_f64()?,
            stiffness: c.take_f64()?,
            damping: c.take_f64()?,
            activation: c.take_f64()?,
            fibre: c.take_option_vec3()?,
        });
    }
    let nd = c.take_u32()? as usize;
    let mut distance_constraints = Vec::with_capacity(nd);
    for _ in 0..nd {
        distance_constraints.push(SoftDistance {
            a: c.take_u32()? as usize,
            b: c.take_u32()? as usize,
            rest: c.take_f64()?,
            compliance: c.take_f64()?,
            compression: c.take_f64()?,
            activation: c.take_f64()?,
            fibre: c.take_option_vec3()?,
        });
    }
    let nt = c.take_u32()? as usize;
    let mut tetrahedra = Vec::with_capacity(nt);
    for _ in 0..nt {
        tetrahedra.push([c.take_u32()?, c.take_u32()?, c.take_u32()?, c.take_u32()?]);
    }
    let ntrv = c.take_u32()? as usize;
    let mut tetra_rest_volumes = Vec::with_capacity(ntrv);
    for _ in 0..ntrv {
        tetra_rest_volumes.push(c.take_f64()?);
    }
    let ntri = c.take_u32()? as usize;
    let mut triangles = Vec::with_capacity(ntri);
    for _ in 0..ntri {
        triangles.push([c.take_u32()?, c.take_u32()?, c.take_u32()?]);
    }
    let solver = match c.take_u8()? {
        0 => SoftSolver::MassSpring,
        _ => {
            let iterations = c.take_u32()?;
            let compliance = c.take_f64()?;
            SoftSolver::Xpbd {
                iterations,
                compliance,
            }
        }
    };
    let gravity = c.take_vec()?;
    let sleeping = c.take_bool()?;
    let collide = c.take_bool()?;
    let particle_radius = c.take_f64()?;
    let wind = match c.take_u8()? {
        0 => None,
        _ => Some(sb_deserialize_wind(c)?),
    };
    let pressure = c.take_option_f64()?;
    let damping = c.take_f64()?;
    let substeps = c.take_u32()?;
    let tear = match c.take_u8()? {
        0 => Some(TearCriterion::Strain(c.take_f64()?)),
        1 => Some(TearCriterion::Stress(c.take_f64()?)),
        2 => Some(TearCriterion::Energy(c.take_f64()?)),
        _ => None,
    };
    let anisotropy = match c.take_u8()? {
        0 => None,
        _ => Some(c.take_vec()?),
    };
    let viscoelastic = match c.take_u8()? {
        0 => None,
        _ => Some(ViscoelasticParams {
            rate_coefficient: c.take_f64()?,
        }),
    };
    let temperature = match c.take_u8()? {
        0 => None,
        _ => Some(ThermalParams {
            temp: c.take_f64()?,
            ambient: c.take_f64()?,
            expansion: c.take_f64()?,
            stiffness_temp_coeff: c.take_f64()?,
        }),
    };
    let plasticity = match c.take_u8()? {
        0 => None,
        _ => Some(sb_deserialize_plasticity(c)?),
    };
    let self_collision = match c.take_u8()? {
        0 => None,
        _ => Some(sb_deserialize_self_collision(c)?),
    };
    let cross_collision = match c.take_u8()? {
        0 => None,
        _ => Some(sb_deserialize_self_collision(c)?),
    };
    let volume_conservation = c.take_option_f64()?;
    let cohesion = match c.take_u8()? {
        0 => None,
        _ => Some(sb_deserialize_cohesion(c)?),
    };
    let (corotated, tetra_rest_shapes) = match c.take_u8()? {
        0 => (None, Vec::new()),
        _ => {
            let k = c.take_f64()?;
            let n = c.take_u32()? as usize;
            let mut shapes = Vec::with_capacity(n);
            for _ in 0..n {
                let mut m = [[0.0f64; 3]; 3];
                for row in &mut m[..] {
                    for v in &mut row[..] {
                        *v = c.take_f64()?;
                    }
                }
                shapes.push(m);
            }
            (Some(k), shapes)
        }
    };
    let neo_hookean = c.take_option_f64()?;
    Some(SoftBody {
        particles,
        springs,
        distance_constraints,
        tetrahedra,
        tetra_rest_volumes,
        triangles,
        solver,
        gravity,
        sleeping,
        collide,
        particle_radius,
        wind,
        pressure,
        damping,
        substeps,
        tear,
        anisotropy,
        viscoelastic,
        temperature,
        plasticity,
        self_collision,
        cross_collision,
        volume_conservation,
        cohesion,
        corotated,
        tetra_rest_shapes,
        neo_hookean,
    })
}

/// Local aliases so the deserializer reads the exact field types without pulling
/// the fork's private struct names into scope. `Spring`/`DistanceConstraint` are
/// `Clone + Copy` with public fields, so we construct them field-by-field.
use rapier3d::dynamics::soft_body::DistanceConstraint as SoftDistance;
use rapier3d::dynamics::soft_body::SoftParticle;
use rapier3d::dynamics::soft_body::Spring as SoftSpring;

/// Return the exact serialized size (in bytes) of a soft body's state, or
/// `u32::MAX` if `world` is null or `id` is unknown. Allocate a buffer of this
/// size before calling [`soft_body_save_state`].
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_state_size(world: *const WorldHandle, id: u32) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_state_size: world is null");
            return u32::MAX;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_state_size: unknown id");
            return u32::MAX;
        };
        let mut buf = Vec::new();
        sb_serialize_body(&mut buf, body);
        // size is exact: serialize once to measure; this is O(state) but cheap.
        buf.len() as u32
    })
}

/// Serialize a soft body's full state into `out` (capacity `out_capacity` bytes).
///
/// Returns `Bool::TRUE` on success, or `Bool::FALSE` if `world`/`id` is invalid or
/// the buffer is too small (`ERR_CAPACITY`). Call [`soft_body_state_size`] first to
/// size the buffer. The blob is portable across bodies (feed it to
/// [`soft_body_restore_state`] on the same or a new id).
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_save_state(
    world: *const WorldHandle,
    id: u32,
    out: *mut u8,
    out_capacity: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_save_state: world is null");
            return Bool::FALSE;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_save_state: unknown id");
            return Bool::FALSE;
        };
        let mut buf = Vec::new();
        sb_serialize_body(&mut buf, body);
        let needed = buf.len();
        if needed > out_capacity as usize {
            set_error(ERR_CAPACITY, "soft_body_save_state: buffer too small");
            return Bool::FALSE;
        }
        let out_slice = unsafe { std::slice::from_raw_parts_mut(out, needed) };
        out_slice.copy_from_slice(&buf);
        clear_error();
        Bool::TRUE
    })
}

/// Restore a soft body's full state from `data` (length `data_len` bytes) into the
/// body `id`. The body must already exist (created with [`soft_body_create`]); this
/// replaces its entire state. Returns `Bool::FALSE` on a null world / unknown id /
/// buffer underflow / magic-or-version mismatch (`ERR_INVALID_ARGUMENT`). A corrupt
/// blob never leaves a half-built body — the whole state is built in a temporary
/// first, then swapped in via `get_mut`.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_restore_state(
    world: *mut WorldHandle,
    id: u32,
    data: *const u8,
    data_len: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_restore_state: world is null");
            return Bool::FALSE;
        };
        let sid = SoftBodyId(id);
        if world.inner.soft_bodies.get(sid).is_none() {
            set_error(ERR_NOT_FOUND, "soft_body_restore_state: unknown id");
            return Bool::FALSE;
        }
        if data.is_null() {
            set_error(ERR_NULL_POINTER, "soft_body_restore_state: data is null");
            return Bool::FALSE;
        }
        let src = unsafe { std::slice::from_raw_parts(data, data_len as usize) };
        let mut cursor = SbCursor::new(src);
        let restored = match sb_deserialize_body(&mut cursor) {
            Some(b) => b,
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "soft_body_restore_state: corrupt blob (bad magic/version/truncated)",
                );
                return Bool::FALSE;
            }
        };
        // Swap the whole rebuilt body in. `collide` is preserved from the blob; if
        // the body was collision-coupled its proxy table is keyed by id and still
        // valid, so this is consistent.
        let target = world.inner.soft_bodies.get_mut(sid).unwrap();
        *target = restored;
        clear_error();
        Bool::TRUE
    })
}

/// Remove a soft body's collision proxies.
///
/// Each proxy is a real dynamic `RigidBody` plus a ball `Collider` inserted into
/// the world, so dropping the `SoftBodySet` slot without this leaves invisible
/// solid balls behind. Shared by `soft_body_enable_collision(false)` and by the
/// full teardown below.
fn remove_soft_body_proxies(world: &mut WorldHandle, id: u32) {
    if let Some(proxies) = world.inner.soft_body_proxies.remove(&id) {
        for ph in proxies.into_iter().flatten() {
            world.inner.bodies.remove(
                ph,
                &mut world.inner.islands,
                &mut world.inner.colliders,
                &mut world.inner.impulse_joints,
                &mut world.inner.multibody_joints,
                false,
            );
        }
    }
}

/// Remove every side table that belongs to a soft body: proxies, LBS skin
/// binding, and voxel-edit metadata.
///
/// `soft_body_destroy` used to drop only the `SoftBodySet` slot. `world_step`
/// skips ids that are no longer in `soft_bodies`, so the proxies stopped being
/// driven — but they stayed in the world as dynamic colliders, i.e. a destroyed
/// 512×512 cloth kept ~262k invisible solid balls shoving the rest of the
/// scene around. `skin_bindings` / `voxel_soft_meta` were never removed at all,
/// so they leaked for the lifetime of the world.
fn teardown_soft_body(world: &mut WorldHandle, id: u32) {
    remove_soft_body_proxies(world, id);
    world.inner.skin_bindings.remove(&id);
    world.inner.voxel_soft_meta.remove(&id);
}

#[unsafe(no_mangle)]
pub extern "C" fn soft_body_destroy(world: *mut WorldHandle, id: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_destroy: world is null");
            return Bool::FALSE;
        };
        let sid = SoftBodyId(id);
        if world.inner.soft_bodies.remove(sid) {
            teardown_soft_body(world, id);
            clear_error();
            Bool::TRUE
        } else {
            set_error(ERR_NOT_FOUND, "soft_body_destroy: unknown id");
            Bool::FALSE
        }
    })
}

// ── Phase 5i: 拓扑读回（渲染用）────────────────────────────────────────────
//
// 三个 FFI 把软体的「当前状态 + 拓扑」一次性拷给调用方提供的缓冲区，供渲染层
// （Minecraft 端）每帧拉取重建网格，无需逐个粒子调用 `soft_body_get_particle`。
// 语义对齐 `world_dynamic_body_snapshot`：调用方先给 capacity 申请足够大的缓冲，
// FFI 写入前 `min(count, capacity)` 个元素并返回 *真实* 元素数（capacity 不足时
// 调用方据此扩容重试）。所有函数对越界/空指针都 idempotent 返回 0，不 panic。

/// 批量读回粒子：位置（world-space）+ 逆质量（0 = pinned）。
/// `out_pos` 容量需 ≥ `capacity` 个 `Vec3`；`out_inv_mass` 容量需 ≥ `capacity` 个 `f64`。
/// 任一出参为 null 即跳过该通道（只写非 null 的通道），但仍返回粒子总数。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_read_particles(
    world: *const WorldHandle,
    id: u32,
    out_pos: *mut Vec3,
    out_inv_mass: *mut f64,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_read_particles: world is null");
            return 0;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_read_particles: unknown id");
            return 0;
        };
        let n = body.particles.len();
        let cap = capacity as usize;
        if !out_pos.is_null() && cap > 0 {
            let slice = unsafe { std::slice::from_raw_parts_mut(out_pos, cap) };
            for (i, p) in body.particles.iter().enumerate().take(cap) {
                slice[i] = vec3_from_rapier(p.pos);
            }
        }
        if !out_inv_mass.is_null() && cap > 0 {
            let slice = unsafe { std::slice::from_raw_parts_mut(out_inv_mass, cap) };
            for (i, p) in body.particles.iter().enumerate().take(cap) {
                slice[i] = p.inv_mass;
            }
        }
        clear_error();
        n as u32
    })
}

/// 批量读回边（弹簧 + 距离约束合并）。每条边是 2 个 `u32` 粒子索引。
/// `out_edges` 容量需 ≥ `capacity` 个 `u32`（即 `capacity/2` 条边）。
/// 边顺序：先所有 springs，再所有 distance_constraints（与 `soft_body_read_tetrahedra`
/// 配合可让渲染层区分软/硬边，若需要）。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_read_edges(
    world: *const WorldHandle,
    id: u32,
    out_edges: *mut u32,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_read_edges: world is null");
            return 0;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_read_edges: unknown id");
            return 0;
        };
        let total_edges = body.springs.len() + body.distance_constraints.len();
        let cap = capacity as usize;
        if !out_edges.is_null() && cap > 0 {
            let slice = unsafe { std::slice::from_raw_parts_mut(out_edges, cap) };
            let mut w = 0usize;
            for s in body.springs.iter() {
                if w + 2 > cap {
                    break;
                }
                slice[w] = s.a as u32;
                slice[w + 1] = s.b as u32;
                w += 2;
            }
            for d in body.distance_constraints.iter() {
                if w + 2 > cap {
                    break;
                }
                slice[w] = d.a as u32;
                slice[w + 1] = d.b as u32;
                w += 2;
            }
        }
        clear_error();
        total_edges as u32
    })
}

/// 批量读回四面体（XPBD 体积约束单元）。每个四面体是 4 个 `u32` 粒子索引。
/// `out_tets` 容量需 ≥ `capacity` 个 `u32`（即 `capacity/4` 个四面体）。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_read_tetrahedra(
    world: *const WorldHandle,
    id: u32,
    out_tets: *mut u32,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_read_tetrahedra: world is null");
            return 0;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_read_tetrahedra: unknown id");
            return 0;
        };
        let n = body.tetrahedra.len();
        let cap = capacity as usize;
        if !out_tets.is_null() && cap > 0 {
            let slice = unsafe { std::slice::from_raw_parts_mut(out_tets, cap) };
            for (i, t) in body.tetrahedra.iter().enumerate().take(cap / 4) {
                let base = i * 4;
                slice[base] = t[0];
                slice[base + 1] = t[1];
                slice[base + 2] = t[2];
                slice[base + 3] = t[3];
            }
        }
        clear_error();
        n as u32
    })
}

/// Phase 6 — cloth: 批量读回三角形面（shell 拓扑）。每个三角形是 3 个 `u32`
/// 粒子索引。 `out_tris` 容量需 ≥ `capacity` 个 `u32`（即 `capacity/3` 个三角形）。
/// 与 `soft_body_read_edges` 配合可让渲染层区分结构边与弯曲边。
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_read_triangles(
    world: *const WorldHandle,
    id: u32,
    out_tris: *mut u32,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_read_triangles: world is null");
            return 0;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_read_triangles: unknown id");
            return 0;
        };
        let n = body.triangles.len();
        let cap = capacity as usize;
        if !out_tris.is_null() && cap > 0 {
            let slice = unsafe { std::slice::from_raw_parts_mut(out_tris, cap) };
            for (i, t) in body.triangles.iter().enumerate().take(cap / 3) {
                let base = i * 3;
                slice[base] = t[0];
                slice[base + 1] = t[1];
                slice[base + 2] = t[2];
            }
        }
        clear_error();
        n as u32
    })
}

#[unsafe(no_mangle)]
/// Phase 27 (B7): exports the soft body's true triangle surface mesh (not the
/// per-particle Ball proxy approximation). Writes up to `vert_cap` vertices (3 f64
/// each) into `out_verts` and up to `tri_cap` triangle indices (3 u32 each) into
/// `out_tris`. Returns the vertex count (so the caller can size its buffers); either
/// buffer may be null to query sizes only. Triangle count comes from
/// `soft_body_read_surface_triangle_count`. This enables mesh-level collision queries
/// (ray-cast, closest-point projection vs static terrain) against the actual surface.
pub extern "C" fn soft_body_read_surface_mesh(
    world: *const WorldHandle,
    id: u32,
    out_verts: *mut f64,
    vert_cap: u32,
    out_tris: *mut u32,
    tri_cap: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_read_surface_mesh: world is null",
            );
            return 0;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_read_surface_mesh: unknown id");
            return 0;
        };
        let vcap = vert_cap as usize;
        if !out_verts.is_null() && vcap > 0 {
            let slice = unsafe { std::slice::from_raw_parts_mut(out_verts, vcap) };
            for (i, part) in body.particles.iter().enumerate().take(vcap / 3) {
                let base = i * 3;
                slice[base] = part.pos.x;
                slice[base + 1] = part.pos.y;
                slice[base + 2] = part.pos.z;
            }
        }
        let tcap = tri_cap as usize;
        if !out_tris.is_null() && tcap > 0 {
            let slice = unsafe { std::slice::from_raw_parts_mut(out_tris, tcap) };
            for (i, tri) in body.triangles.iter().enumerate().take(tcap / 3) {
                let base = i * 3;
                slice[base] = tri[0];
                slice[base + 1] = tri[1];
                slice[base + 2] = tri[2];
            }
        }
        clear_error();
        body.particles.len() as u32
    })
}

#[unsafe(no_mangle)]
/// Phase 27 (B7): returns the triangle count of a soft body's surface mesh.
pub extern "C" fn soft_body_read_surface_triangle_count(world: *const WorldHandle, id: u32) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_read_surface_triangle_count: world is null",
            );
            return 0;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(
                ERR_NOT_FOUND,
                "soft_body_read_surface_triangle_count: unknown id",
            );
            return 0;
        };
        clear_error();
        body.triangles.len() as u32
    })
}

/// Phase 27 (B8): advances one soft body with the **implicit (backward-Euler) reference
/// integrator** instead of the world's default solver. This is a comparison path:
/// for stiff springs where `step_mass_spring` (explicit) blows up, the implicit step
/// stays bounded. See `SoftBody::step_implicit_euler` (fork) for the linear-system
/// formulation. Returns 0 on success, or an error code if `world`/`id` is invalid.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_step_mass_spring(world: *mut WorldHandle, id: u32, dt: f64) -> u32 {
    ffi_guard(ERR_INTERNAL, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_step_mass_spring: world is null",
            );
            return ERR_NULL_POINTER;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_step_mass_spring: unknown id");
            return ERR_NOT_FOUND;
        };
        body.step_mass_spring(dt);
        clear_error();
        ERR_OK
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn soft_body_step_implicit(world: *mut WorldHandle, id: u32, dt: f64) -> u32 {
    ffi_guard(ERR_INTERNAL, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_step_implicit: world is null");
            return ERR_NULL_POINTER;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_step_implicit: unknown id");
            return ERR_NOT_FOUND;
        };
        body.step_implicit_euler(dt);
        clear_error();
        ERR_OK
    })
}

// ── Phase 22: 逐边应力 / 张力读数（仅供渲染层 / 上层逻辑使用，不改求解器）──────
//
// 软体现在能回读位置 / 拓扑（read_particles / read_edges / read_tetrahedra /
// read_triangles），但**没有逐边张力**——调试可视化（颜色映射应力）和"撕裂风险"
// 逻辑只能靠自己算边长。补一个纯读数 FFI：对每个结构边（弹簧 + XPBD 距离约束）
// 返回归一化应变 strain = (len - rest) / rest（rest==0 → 0.0）。长度从现有质点
// 位置现算，不改 fork、不动求解器，沿用 read_edges 的 marshalling 形态。

/// Read per-edge normalized strain (stress proxy) for a soft body.
///
/// Edges are enumerated in the same order as [`soft_body_read_edges`]: every
/// `Spring` first, then every `DistanceConstraint`. For each edge the function
/// writes `strain = (current_len - rest) / rest` (0.0 when `rest == 0`) into
/// `out_strain[..]`. Returns the total edge count (so the caller can size its
/// buffer); when `out_strain` is null or `capacity` is 0 the count is returned
/// without writing.
///
/// This is a pure read-out for debug visualisation / "tear risk" UI; it does
/// not affect the solver. Symmetry / determinism are irrelevant because no state
/// is mutated.
///
/// # Safety
/// `world` must be a valid world pointer. `out_strain` must point to an array of
/// at least `capacity` `f64` elements when non-null.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_read_stress(
    world: *const WorldHandle,
    id: u32,
    out_strain: *mut f64,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_read_stress: world is null");
            return 0;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_read_stress: unknown id");
            return 0;
        };
        let total = body.springs.len() + body.distance_constraints.len();
        let cap = capacity as usize;
        if !out_strain.is_null() && cap > 0 {
            let slice = unsafe { std::slice::from_raw_parts_mut(out_strain, cap) };
            let mut w = 0usize;
            for s in body.springs.iter() {
                if w >= cap {
                    break;
                }
                let len = (body.particles[s.a].pos - body.particles[s.b].pos).length();
                let rest = s.rest_length;
                slice[w] = if rest > 0.0 { (len - rest) / rest } else { 0.0 };
                w += 1;
            }
            for d in body.distance_constraints.iter() {
                if w >= cap {
                    break;
                }
                let len = (body.particles[d.a].pos - body.particles[d.b].pos).length();
                let rest = d.rest;
                slice[w] = if rest > 0.0 { (len - rest) / rest } else { 0.0 };
                w += 1;
            }
        }
        clear_error();
        total as u32
    })
}

// ── Phase 23a: 静止长度缩放（纯状态缩放，供呼吸 / 生长 / 挤压动画）──────────────
//
// 现有 FFI 能加弹簧 / 距离约束并设刚度，但**不能整体缩放静止长度**——做"呼吸"
// "肌肉收缩""被挤压拉长"这类动画时只能逐条改，或改不动（XPBD 距离约束 rest 是
// 加时捕获的）。补一个纯状态缩放：把所有弹簧 rest_length 与距离约束 rest 乘
// factor。不改 fork、不动求解器，和 Phase 22 同属"纯 mps-core 追加"。

/// Uniformly scale the rest length of every structural edge (springs + XPBD
/// distance constraints) in a soft body by `factor`.
///
/// This is a one-shot state mutation (not a per-step force): it multiplies each
/// `Spring::rest_length` and `DistanceConstraint::rest` by `factor`. It is the
/// cheap primitive behind "breathing" / "muscle contraction" / "squeeze-stretch"
/// effects — previously users had to retune every edge by hand. `factor` must be
/// strictly positive; a non-positive value returns `ERR_INVALID_ARGUMENT` and
/// touches nothing.
///
/// Returns the number of edges scaled (springs + distance constraints), or 0 on
/// null world / unknown id / invalid factor.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_scale_rest_length(
    world: *mut WorldHandle,
    id: u32,
    factor: f64,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_scale_rest_length: world is null",
            );
            return 0;
        };
        if !factor.is_finite() || factor <= 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_scale_rest_length: factor must be > 0",
            );
            return 0;
        }
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_scale_rest_length: unknown id");
            return 0;
        };
        let n = body.springs.len() + body.distance_constraints.len();
        for s in body.springs.iter_mut() {
            s.rest_length *= factor;
        }
        for d in body.distance_constraints.iter_mut() {
            d.rest *= factor;
        }
        clear_error();
        n as u32
    })
}

// ── Phase 23b: 逐三角形法线回读（纯只读，供渲染层打光 / 调试可视化）────────────
//
// 渲染层要正确地给软体 / 布料打光，需要逐三角形法线。现在能回读三角形拓扑
// （read_triangles），但法线得上层自己按粒子位置算。这里补一个纯只读回读：
// 对每个三角形现算法线（(p1-p0)×(p2-p0) 归一化）写出 3 个 f64。长度从现有粒子
// 位置现算，不改 fork、不动求解器，与 Phase 22 应力回读同源。

/// Read per-triangle unit normals for a soft body.
///
/// Triangles are enumerated in the order returned by [`soft_body_read_triangles`].
/// For each triangle `T = (i0, i1, i2)` the function writes the unit normal
/// `(p1 - p0) × (p2 - p0)` normalized into `out_normals[3*k .. 3*k+3]`. Returns
/// the triangle count (so the caller can size its buffer); when `out_normals` is
/// null or `capacity` is 0 the count is returned without writing. Degenerate
/// triangles yield a zero normal.
///
/// Pure read-out for rendering / debug visualisation; does not affect the solver.
///
/// # Safety
/// `world` must be a valid world pointer. `out_normals` must point to an array of
/// at least `capacity` `f64` elements when non-null.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_read_normals(
    world: *const WorldHandle,
    id: u32,
    out_normals: *mut f64,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_read_normals: world is null");
            return 0;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_read_normals: unknown id");
            return 0;
        };
        let n = body.triangles.len();
        let cap = capacity as usize;
        if !out_normals.is_null() && cap > 0 {
            let slice = unsafe { std::slice::from_raw_parts_mut(out_normals, cap) };
            for (k, t) in body.triangles.iter().enumerate().take(cap / 3) {
                let base = k * 3;
                let p0 = body.particles[t[0] as usize].pos;
                let p1 = body.particles[t[1] as usize].pos;
                let p2 = body.particles[t[2] as usize].pos;
                let nrm = (p1 - p0).cross(p2 - p0);
                let len = nrm.length();
                let (nx, ny, nz) = if len > 1e-12 {
                    let inv = 1.0 / len;
                    (nrm.x * inv, nrm.y * inv, nrm.z * inv)
                } else {
                    (0.0, 0.0, 0.0)
                };
                slice[base] = nx;
                slice[base + 1] = ny;
                slice[base + 2] = nz;
            }
        }
        clear_error();
        n as u32
    })
}

// ── Phase 25 #1: 接触力回读（纯 mps-core，零 fork 改动）────────────────────────────
//
// soft_body_collision 让每个自由质点有一枚 proxy 动态刚体 + Ball collider；接触力由
// Rapier narrow-phase 在 proxy 上算。这里在每个 proxy collider 上累加
// ContactPair::total_impulse()，写回每个质点的净接触力向量。纯只读，不改求解器，
// 与 Phase 22 应力 / Phase 23 法线回读同源。collide==false 或没有 proxy 时全 0，
// 返回质点数（与 read_normals 同约定：capacity 不足只写满 cap 个）。

/// Read the per-particle net contact force for a collision-coupled soft body.
///
/// For each free particle that has a proxy collider, the function sums the
/// `ContactPair::total_impulse` over every active contact pair touching that
/// collider, writing the net force vector into `out_fx/out_fy/out_fz[k]`. This is
/// the contact reaction the soft body exerts/feels through its proxy colliders —
/// the primitive behind "step on a soft cushion and get pushed back up" logic.
///
/// Returns the particle count (so the caller can size its buffer); when `out_fx`
/// is null or `capacity` is 0 the count is returned without writing. Bodies with
/// `collide == false` (no proxies) yield zero force for every particle. Pure
/// read-out: does not affect the solver.
///
/// # Safety
/// `world` must be a valid world pointer. `out_fx/out_fy/out_fz` must each point
/// to an array of at least `capacity` `f64` elements when non-null.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_read_contact_force(
    world: *const WorldHandle,
    id: u32,
    out_fx: *mut f64,
    out_fy: *mut f64,
    out_fz: *mut f64,
    capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_ref() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_read_contact_force: world is null",
            );
            return 0;
        };
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_read_contact_force: unknown id");
            return 0;
        };
        let n = body.particles.len();
        let cap = capacity as usize;

        // Accumulator per particle, reused whether or not we write it out.
        let mut fx = vec![0.0f64; n];
        let mut fy = vec![0.0f64; n];
        let mut fz = vec![0.0f64; n];

        // Only collision-coupled bodies have proxies; everything else is zero.
        if let Some(proxies) = world.inner.soft_body_proxies.get(&id) {
            // Map collider handle -> owning particle index (proxy colliders only).
            let mut col_to_particle: std::collections::HashMap<ColliderHandle, usize> =
                std::collections::HashMap::new();
            for (i, ph) in proxies.iter().enumerate() {
                let Some(rb_h) = ph else {
                    continue;
                };
                let Some(rb) = world.inner.bodies.get(*rb_h) else {
                    continue;
                };
                for c in rb.colliders() {
                    col_to_particle.insert(*c, i);
                }
            }
            for pair in world.inner.narrow_phase.contact_pairs() {
                let imp = pair.total_impulse();
                // Force on collider2 = +imp, on collider1 = -imp (Newton's 3rd law;
                // `total_impulse` is the force on collider2, pointing from c1 to c2).
                if let Some(&i1) = col_to_particle.get(&pair.collider1) {
                    fx[i1] -= imp.x;
                    fy[i1] -= imp.y;
                    fz[i1] -= imp.z;
                }
                if let Some(&i2) = col_to_particle.get(&pair.collider2) {
                    fx[i2] += imp.x;
                    fy[i2] += imp.y;
                    fz[i2] += imp.z;
                }
            }
        }

        if !out_fx.is_null() && !out_fy.is_null() && !out_fz.is_null() && cap > 0 {
            let m = cap.min(n);
            let sx = unsafe { std::slice::from_raw_parts_mut(out_fx, cap) };
            let sy = unsafe { std::slice::from_raw_parts_mut(out_fy, cap) };
            let sz = unsafe { std::slice::from_raw_parts_mut(out_fz, cap) };
            sx[..m].copy_from_slice(&fx[..m]);
            sy[..m].copy_from_slice(&fy[..m]);
            sz[..m].copy_from_slice(&fz[..m]);
        }
        clear_error();
        n as u32
    })
}

// ── Phase 24: XPBD/MassSpring substeps 暴露（动 fork：SoftBody.substeps 字段）──
//
// 求解器现在 step_xpbd / step_mass_spring 只有 1 substep（frame dt 一次性投影）。
// 暴露 substeps 后，每帧 dt 被切成 n 等份、每个子步独立投影，硬材质 / 高 compliance
// 收敛更快、更稳。改动在 fork 的 SoftBody::step 循环 + set_substeps，mps-core 只
// 镜像一个 FFI。n==0 拒绝（保持前值），与 fork 侧守卫一致。

/// Set the number of solver substeps per `soft_body_step` call for a soft body.
///
/// `n >= 1` splits the frame `dt` into `n` equal slices; the active solver
/// (XPBD or MassSpring) is run once per slice, projecting constraints at a finer
/// time resolution. Stiff materials and high-compliance edges converge faster
/// and stay stable with more substeps (at `n×` the per-step CPU cost). `n == 0`
/// is rejected and leaves the previous value unchanged.
///
/// Returns the new substep count, or 0 on null world / unknown id / invalid `n`.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_substeps(world: *mut WorldHandle, id: u32, n: u32) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_set_substeps: world is null");
            return 0;
        };
        if n == 0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_substeps: n must be >= 1",
            );
            return 0;
        }
        let sid = SoftBodyId(id);
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_set_substeps: unknown id");
            return 0;
        };
        body.set_substeps(n);
        clear_error();
        body.substeps
    })
}

// ── Phase 5d: 区块破坏 → 软体重建联动（Minecraft 闭环）──────────────────────

//
// 监听 voxel 地形 `set_cell` 破坏事件后，上层调用本 FFI：把被挖空的方块格
// (cx,cy,cz) 经 `voxel_soft_meta` 映射到对应质点下标，调 `SoftBody::remove_particle`
// 删该质点 + 其弹簧/约束，并就地重建映射（其余下标随 remove_particle 平移）。
// 软体在新拓扑下继续仿真 —— 这就是「破坏方块 → 软体塌缩」的闭环原子操作。

/// Dig out a single voxel cell of a soft body built via `soft_body_voxel_build`,
/// removing the particle that occupies it (plus its incident springs/constraints)
/// and rebuilding the voxel→particle map so further digs stay consistent.
///
/// Returns `Bool::TRUE` on success. `Bool::FALSE` if the body/id is unknown, the
/// cell is out of bounds, or the cell is already empty/dug.
///
/// # Safety
/// `world` must be a valid world pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_voxel_dig(
    world: *mut WorldHandle,
    id: u32,
    cell_x: u32,
    cell_y: u32,
    cell_z: u32,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_voxel_dig: world is null");
            return Bool::FALSE;
        };
        soft_body_voxel_dig_inner(world, id, cell_x, cell_y, cell_z)
    })
}

/// Core of [`soft_body_voxel_dig`] operating on an already-unwrapped world
/// handle. Exposed as `pub(crate)` so `collider_voxel_edit` can propagate a
/// terrain dig to overlapping soft bodies (Phase 5g) without re-wrapping the
/// raw pointer.
pub(crate) fn soft_body_voxel_dig_inner(
    world: &mut WorldHandle,
    id: u32,
    cell_x: u32,
    cell_y: u32,
    cell_z: u32,
) -> Bool {
    let sid = SoftBodyId(id);
    let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
        set_error(ERR_NOT_FOUND, "soft_body_voxel_dig: unknown id");
        return Bool::FALSE;
    };
    let Some(meta) = world.inner.voxel_soft_meta.get_mut(&id) else {
        set_error(
            ERR_NOT_FOUND,
            "soft_body_voxel_dig: body has no voxel mapping",
        );
        return Bool::FALSE;
    };
    // cell_linear uses the same layout as VoxelGrid::index / soft_body_voxel_build.
    if cell_x as usize >= meta.sx || cell_y as usize >= meta.sy || cell_z as usize >= meta.sz {
        set_error(
            ERR_INVALID_ARGUMENT,
            "soft_body_voxel_dig: cell out of bounds",
        );
        return Bool::FALSE;
    }
    let cell_linear = cell_x as usize + meta.sx * (cell_z as usize + meta.sz * cell_y as usize);
    let p_idx = match meta.map.get(cell_linear).copied() {
        Some(v) if v >= 0 => v as usize,
        _ => {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_voxel_dig: cell is empty or already dug",
            );
            return Bool::FALSE;
        }
    };
    if !body.remove_particle(p_idx) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "soft_body_voxel_dig: remove_particle failed",
        );
        return Bool::FALSE;
    }
    // Rebuild the map: the dug cell becomes -1; every surviving index that was
    // > p_idx shifts down by one (mirrors SoftBody::remove_particle).
    for m in meta.map.iter_mut() {
        if *m == p_idx as i64 {
            *m = -1;
        } else if *m > p_idx as i64 {
            *m -= 1;
        }
    }
    clear_error();
    Bool::TRUE
}

/// Phase 5g: when a voxel collider cell is dug (set to empty), propagate the
/// dig to every soft body whose voxel grid overlaps that world-space cell. For
/// each soft body we convert the dug cell's world-center into the soft body's
/// own cell coordinates (stored in `VoxelSoftMeta.origin` / `voxel_size`) and
/// call `soft_body_voxel_dig_inner`. Digs are best-effort and idempotent: a
/// soft cell that is already empty/dug simply returns `Bool::FALSE` without
/// disturbing the body.
///
/// Must be called when no other borrow of `world.inner` (e.g. a `VoxelCache`)
/// is live, since it mutates `soft_bodies` + `voxel_soft_meta`.
pub(crate) fn propagate_dig_to_soft_bodies(world: &mut WorldHandle, world_center: Vec3) {
    // Snapshot the ids first to keep the borrow of `voxel_soft_meta` short.
    let ids: Vec<u32> = world.inner.voxel_soft_meta.keys().copied().collect();
    for id in ids {
        let (sx, sy, sz, origin, voxel_size) = {
            let meta = match world.inner.voxel_soft_meta.get(&id) {
                Some(m) => m,
                None => continue,
            };
            (meta.sx, meta.sy, meta.sz, meta.origin, meta.voxel_size)
        };
        if !voxel_size.is_finite() || voxel_size <= 0.0 {
            continue;
        }
        // World-center → soft-body cell coordinate (uniform grid).
        let scx = ((world_center.x - origin.x) / voxel_size - 0.5).round() as i64;
        let scy = ((world_center.y - origin.y) / voxel_size - 0.5).round() as i64;
        let scz = ((world_center.z - origin.z) / voxel_size - 0.5).round() as i64;
        if scx < 0
            || scy < 0
            || scz < 0
            || scx as usize >= sx
            || scy as usize >= sy
            || scz as usize >= sz
        {
            continue;
        }
        soft_body_voxel_dig_inner(world, id, scx as u32, scy as u32, scz as u32);
    }
}

// ── Phase 5f: 软体-刚体碰撞（proxy collider 桥接）──────────────────────────
//
// 给每个自由质点（inv_mass > 0）建一个动态 `RigidBody` + `Ball` collider（proxy，
// `gravity_scale = 0`，质量 = 1/inv_mass）。`world_step` 在刚性步进前把质点力/位姿
// 写入 proxy，narrow-phase/contact 自然作用于 proxy，步进后再把 proxy 受接触后的位姿
// 读回质点 —— 软体于是被地形/实体挡住、反弹、堆叠。完全复用现有接触，不改 rapier 求解器。
// `SoftBody.collide` 置 true 后 `SoftBody::step` 跳过自积分（位置由 proxy 驱动）。
// pinned 质点（inv_mass == 0）不建 proxy，位置由外部固定。

/// Enable or disable rigid-body collision coupling for a soft body.
///
/// When `enabled` is `Bool::TRUE`, one dynamic `Ball` collider (radius `particle_radius`)
/// is created per free particle and registered in the world's collision-proxy table; the
/// body's `collide` flag is set so the integration layer drives its particles from the
/// proxies. When `Bool::FALSE`, any existing proxies are removed and `collide` is cleared.
///
/// Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be a valid world pointer returned by `world_create`.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_enable_collision(
    world: *mut WorldHandle,
    id: u32,
    particle_radius: f64,
    enabled: Bool,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_enable_collision: world is null",
            );
            return Bool::FALSE;
        };
        let sid = SoftBodyId(id);
        // Validate up front, and without holding a `&mut` into `soft_bodies`:
        // the proxy teardown below mutates `bodies`/`colliders`, and a live
        // field borrow of `soft_bodies` would make that a double borrow.
        let particle_count = match world.inner.soft_bodies.get(sid) {
            Some(body) => body.particles.len(),
            None => {
                set_error(ERR_NOT_FOUND, "soft_body_enable_collision: unknown id");
                return Bool::FALSE;
            }
        };
        if particle_radius <= 0.0 || !particle_radius.is_finite() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_enable_collision: bad particle_radius",
            );
            return Bool::FALSE;
        }
        // Both branches tear the previous proxy set down first. Re-enabling
        // must not leak it: the old code overwrote the table entry directly, so
        // every repeated `enable_collision(true)` (the documented way to
        // re-sync after adding particles) orphaned a full set of dynamic
        // bodies + colliders that stayed in the world and kept colliding.
        remove_soft_body_proxies(world, id);
        if enabled == Bool::FALSE {
            if let Some(body) = world.inner.soft_bodies.get_mut(sid) {
                body.collide = false;
            }
            clear_error();
            return Bool::TRUE;
        }
        let Some(body) = world.inner.soft_bodies.get_mut(sid) else {
            set_error(ERR_NOT_FOUND, "soft_body_enable_collision: unknown id");
            return Bool::FALSE;
        };
        body.collide = true;
        body.particle_radius = particle_radius;
        let mut proxies: Vec<Option<RigidBodyHandle>> = Vec::with_capacity(particle_count);
        for p in &body.particles {
            if p.inv_mass == 0.0 {
                proxies.push(None); // pinned: no proxy
                continue;
            }
            let mass = 1.0 / p.inv_mass;
            let rb = RigidBodyBuilder::new(RigidBodyType::Dynamic)
                .gravity_scale(0.0)
                .additional_mass(mass)
                .translation(p.pos)
                .linvel(p.vel)
                .build();
            let h = world.inner.bodies.insert(rb);
            // Phase 15: zero the collider's own density so the proxy's mass is *exactly*
            // the particle mass (`additional_mass`) and not inflated by the ball's volume.
            // A non-zero collider mass unbalances two-way momentum transfer (a light soft
            // particle would drive an over-heavy proxy that cannot cleanly push dynamic
            // rigid bodies). Density 0 keeps the reaction physically symmetric.
            // Phase 5f (extended): per-body collision group so that particles of the
            // SAME soft body do not collide with each other (their proxy balls overlap
            // when the particle spacing is below 2·radius, which would otherwise make
            // the body self-explode), while particles of DIFFERENT soft bodies — and
            // rigid-body terrain — still collide through the rapier narrow-phase.
            // We assign each body a distinct bit (bit 1..=31, cycling on id overflow);
            // membership = that bit only, filter = everything except that bit (so it
            // collides with other bodies and with group-0 rigid colliders).
            let group_bit = 1u32 << ((id % 31) + 1);
            let groups = InteractionGroups::all()
                .with_memberships(Group::from_bits_truncate(group_bit))
                .with_filter(Group::from_bits_truncate(!group_bit));
            let col = ColliderBuilder::ball(particle_radius)
                .density(0.0)
                .collision_groups(groups)
                .build();
            world
                .inner
                .colliders
                .insert_with_parent(col, h, &mut world.inner.bodies);
            proxies.push(Some(h));
        }
        world.inner.soft_body_proxies.insert(id, proxies);
        clear_error();
        Bool::TRUE
    })
}

/// Bind a skeleton (a list of rigid bodies acting as bones) to a soft body for
/// linear-blend skinning (LBS). Records each bone's current world transform as its
/// bind pose and precomputes the inverse; per-particle weights are then set with
/// [`soft_body_set_vertex_weights`]. `world_step` applies the skinning every step.
///
/// * `bone_count` — number of bones (≤ `bones.len()`).
/// * `bones` — pointer to `bone_count` `RigidBodyHandleRaw` (packed `u64`) values.
///
/// Returns the number of bones bound, or `0` on error.
///
/// # Safety
/// `world` must be valid; `bones` must point to `bone_count` valid handle words.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_bind_skeleton(
    world: *mut WorldHandle,
    id: u32,
    bone_count: u32,
    bones: *const u64,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_body_bind_skeleton: world is null");
            return 0;
        };
        if id as usize >= world.inner.soft_bodies.len() {
            set_error(ERR_NOT_FOUND, "soft_body_bind_skeleton: unknown id");
            return 0;
        }
        if bone_count == 0 || bones.is_null() {
            set_error(ERR_INVALID_ARGUMENT, "soft_body_bind_skeleton: no bones");
            return 0;
        }
        let count = bone_count as usize;
        let mut handles: Vec<RigidBodyHandle> = Vec::with_capacity(count);
        let mut bind_translation: Vec<Vector> = Vec::with_capacity(count);
        let mut bind_rotation: Vec<Rotation> = Vec::with_capacity(count);
        for i in 0..count {
            let raw = unsafe { *bones.add(i) };
            let h = unpack_rigid_body_handle(raw);
            let rb = match world.inner.bodies.get(h) {
                Some(rb) => rb,
                None => {
                    set_error(ERR_NOT_FOUND, "soft_body_bind_skeleton: bad bone handle");
                    return 0;
                }
            };
            handles.push(h);
            bind_translation.push(rb.translation());
            bind_rotation.push(*rb.rotation());
        }
        let binding = SkeletonBinding {
            bones: handles,
            bind_translation,
            bind_rotation,
            vertices: std::collections::HashMap::new(),
        };
        world.inner.skin_bindings.insert(id, binding);
        clear_error();
        bone_count
    })
}

/// Bind one soft-body particle to up to 4 bones with linear-blend-skinning
/// weights. Weights need not be normalized; they are normalized here. The
/// particle's current world position is recorded as its rest pose and converted
/// into each bone's bind-pose frame for the per-step skinning.
///
/// * `bone_indices` — pointer to `4` `u32` bone slots (unused slots ignored).
/// * `weights` — pointer to `4` `f64` weight slots.
///
/// Returns `Bool::TRUE` on success.
///
/// # Safety
/// `world` must be valid; `bone_indices`/`weights` must point to 4 elements.
#[unsafe(no_mangle)]
pub extern "C" fn soft_body_set_vertex_weights(
    world: *mut WorldHandle,
    id: u32,
    particle_index: u32,
    bone_indices: *const u32,
    weights: *const f64,
) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(
                ERR_NULL_POINTER,
                "soft_body_set_vertex_weights: world is null",
            );
            return Bool::FALSE;
        };
        let idx = id as usize;
        if idx >= world.inner.soft_bodies.len() {
            set_error(ERR_NOT_FOUND, "soft_body_set_vertex_weights: unknown id");
            return Bool::FALSE;
        }
        if bone_indices.is_null() || weights.is_null() {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_vertex_weights: null pointers",
            );
            return Bool::FALSE;
        }
        let binding = match world.inner.skin_bindings.get_mut(&id) {
            Some(b) => b,
            None => {
                set_error(
                    ERR_NOT_FOUND,
                    "soft_body_set_vertex_weights: skeleton not bound",
                );
                return Bool::FALSE;
            }
        };
        let sb = match world.inner.soft_bodies.get(SoftBodyId(id)) {
            Some(sb) => sb,
            None => {
                set_error(ERR_NOT_FOUND, "soft_body_set_vertex_weights: bad body");
                return Bool::FALSE;
            }
        };
        let pi = particle_index as usize;
        let p = match sb.particles.get(pi) {
            Some(p) => p,
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "soft_body_set_vertex_weights: bad particle index",
                );
                return Bool::FALSE;
            }
        };
        let rest = p.pos;
        let mut bone_idx = [0u32; 4];
        let mut weight = [0.0f64; 4];
        let mut local = [Vector::ZERO; 4];
        let mut wsum = 0.0f64;
        for k in 0..4 {
            let w = unsafe { *weights.add(k) };
            if w <= 0.0 {
                continue;
            }
            let bi = unsafe { *bone_indices.add(k) } as usize;
            if bi >= binding.bones.len() {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "soft_body_set_vertex_weights: bone index out of range",
                );
                return Bool::FALSE;
            }
            bone_idx[k] = bi as u32;
            weight[k] = w;
            wsum += w;
            // Rest position expressed in bone `bi`'s bind-pose frame (offset vector).
            local[k] = binding.bind_rotation[bi].inverse() * (rest - binding.bind_translation[bi]);
        }
        if wsum <= 0.0 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_body_set_vertex_weights: no positive weights",
            );
            return Bool::FALSE;
        }
        // Normalize weights so the blend is affine-invariant.
        #[allow(clippy::needless_range_loop)]
        for k in 0..4 {
            weight[k] /= wsum;
        }
        binding.vertices.insert(
            particle_index,
            VertexSkin {
                bone_idx,
                weight,
                local,
            },
        );
        clear_error();
        Bool::TRUE
    })
}
