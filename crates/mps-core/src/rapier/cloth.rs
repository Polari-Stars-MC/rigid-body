//! Cloth bodies — grid-topology soft bodies (composition layer, route A).
//!
//! A **cloth body** is a rectangular particle grid (cols × rows) bridged by
//! three spring families, stored as an ordinary [`SoftBody`] in the world's
//! `SoftBodySet`. Like `soft_chain_create` (Phase 1) and `soft_body_build_grid`
//! this module *composes* existing primitives instead of inventing physics:
//!
//! * **Structural springs** — horizontal/vertical grid neighbours at
//!   `stiffness`. Carry the cloth's tensile shape.
//! * **Shear springs** — both cell diagonals at `stiffness · shear_ratio`.
//!   Resist in-plane skewing.
//! * **Bend springs** — two-apart neighbours at `stiffness · bend_ratio`.
//!   The classic mass-spring bending proxy; with `bend_ratio = 0` the cloth
//!   creases freely.
//!
//! Because the result is a regular `SoftBody`, every existing `soft_body_*`
//! FFI works on it unchanged: `soft_body_apply_wind` (Phase 7 wind acts on
//! every free particle — no triangles needed), `soft_body_set_tear_strain` /
//! `soft_body_tear_now` (Phase 6 tearing), `soft_body_set_gravity` (terrain
//! coupling), `soft_body_read_particles` (render read-back), sleep / energy /
//! spring forces / … . The cloth therefore costs exactly one constructor —
//! everything else is inherited.
//!
//! ```text
//!   (0,rows-1) ●───●───●───● (cols-1,rows-1)
//!              │ ╲ │ ╲ │ ╲ │        ─ struct (stiffness)
//!              │ ╳ │ ╳ │ ╳ │        ╲ shear (stiffness·shear_ratio)
//!              │ ╱ │ ╱ │ ╱ │        ┄ bend   (stiffness·bend_ratio)
//!   (0,0)      ●───●───●───● (cols-1,0)
//! ```
//!
//! ## Difference from `soft_body_build_grid`
//!
//! `soft_body_build_grid` fills a 3D box (6-connectivity, XPBD, boundary
//! pinning) — a jelly block. A cloth is *2-dimensional* with the three-family
//! spring split (struct/shear/bend), arbitrary plane orientation via `u_axis`
//! / `v_axis`, and edge-selective pinning — the flag/curtain/tablecloth
//! formulation. It stays on the default `MassSpring` solver, where all three
//! families are plain springs. The body's per-body gravity starts at the
//! *world's* gravity (unlike `soft_body_voxel_build`, which pins it to zero
//! for the terrain-coupling hook to fill in).
//!
//! ## Error convention
//!
//! Following `soft_body_build_grid` / `soft_body_build_rope`,
//! `soft_cloth_create` returns `u32::MAX` on error (the error slot always
//! carries the `ERR_*` code) and the new `SoftBodyId` otherwise, clearing the
//! thread-local error slot.

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{Vec3, WorldHandle, vec3_finite, vec3_to_rapier};
use rapier3d::prelude::soft_body_fork::SoftBody;

/// Upper bound on cloth particles accepted in one creation call.
///
/// 512 × 512 ≈ 262k particles ≈ 1M springs keeps a single cloth well inside
/// interactive-step territory and caps allocation on hostile inputs.
pub const CLOTH_MAX_PARTICLES: u32 = 262_144;

/// How the cloth's border particles are pinned at creation time.
///
/// Edge/corner identifiers are **grid indices**, independent of world
/// orientation: the caller decides what "left" or "top" means by choosing
/// `u_axis` / `v_axis` (e.g. `v_axis = +Y` makes the `row = 0` edge the top
/// row).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum ClothPinMode {
    /// Nothing is pinned — the cloth falls freely.
    Free = 0,
    /// Pin the four grid corners `(0,0)`, `(cols-1,0)`, `(0,rows-1)`,
    /// `(cols-1,rows-1)`.
    Corners = 1,
    /// Pin the whole `col == 0` edge (classic flag-on-a-pole).
    UStartEdge = 2,
    /// Pin the whole `row == 0` edge (curtain rod).
    VStartEdge = 3,
}

impl ClothPinMode {
    fn from_u32(v: u32) -> Option<Self> {
        match v {
            0 => Some(Self::Free),
            1 => Some(Self::Corners),
            2 => Some(Self::UStartEdge),
            3 => Some(Self::VStartEdge),
            _ => None,
        }
    }
}

/// Descriptor for [`soft_cloth_create`].
///
/// The cloth is generated in the plane spanned by `u_axis` (columns) and
/// `v_axis` (rows); the two axes must be finite, non-zero and not parallel.
/// Both are normalised internally, so their lengths are irrelevant.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct ClothDesc {
    /// Particles along `u_axis` (columns). Must be ≥ 2.
    pub cols: u32,
    /// Particles along `v_axis` (rows). Must be ≥ 2.
    pub rows: u32,
    /// Rest length between adjacent grid particles. Must be > 0 and finite.
    pub spacing: f64,
    /// World position of grid particle `(0, 0)`.
    pub origin: Vec3,
    /// Column direction (normalised internally).
    pub u_axis: Vec3,
    /// Row direction (normalised internally, must not be parallel to `u_axis`).
    pub v_axis: Vec3,
    /// Mass of each *free* particle. Must be > 0 and finite. Pinned particles
    /// carry infinite mass regardless.
    pub particle_mass: f64,
    /// Structural spring stiffness (shear/bend are derived from it). ≥ 0.
    pub stiffness: f64,
    /// Spring damping, shared by all three spring families. ≥ 0.
    pub damping: f64,
    /// Shear stiffness = `stiffness · shear_ratio`, in `[0, 1]`. `0` disables
    /// shear springs.
    pub shear_ratio: f64,
    /// Bend stiffness = `stiffness · bend_ratio`, in `[0, 1]`. `0` disables
    /// bend springs.
    pub bend_ratio: f64,
    /// Border-pinning scheme, see [`ClothPinMode`].
    pub pin_mode: u32,
}

/// Is particle `(col, row)` pinned under `mode`?
fn is_pinned(mode: ClothPinMode, col: usize, row: usize, cols: usize, rows: usize) -> bool {
    match mode {
        ClothPinMode::Free => false,
        ClothPinMode::Corners => {
            let left = col == 0;
            let right = col + 1 == cols;
            let bottom = row == 0;
            let top = row + 1 == rows;
            (left || right) && (bottom || top)
        }
        ClothPinMode::UStartEdge => col == 0,
        ClothPinMode::VStartEdge => row == 0,
    }
}

/// Create a cloth body as a rectangular mass-spring grid.
///
/// Returns the new `SoftBodyId` (as `u32`), or `u32::MAX` with the thread-local
/// error slot set to an `ERR_*` code on failure. The cloth integrates
/// automatically in `world_step` — no separate stepping call is needed.
///
/// # Safety
///
/// `world` must be a valid world pointer or null (null reports
/// `ERR_NULL_POINTER` and returns `u32::MAX`). No other pointers are
/// dereferenced; `desc` is passed by value.
#[unsafe(no_mangle)]
pub extern "C" fn soft_cloth_create(world: *mut WorldHandle, desc: ClothDesc) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "soft_cloth_create: world is null");
            return u32::MAX;
        };

        let Some(pin_mode) = ClothPinMode::from_u32(desc.pin_mode) else {
            set_error(ERR_INVALID_ARGUMENT, "soft_cloth_create: unknown pin_mode");
            return u32::MAX;
        };
        let (cols, rows) = (desc.cols as usize, desc.rows as usize);
        if desc.cols < 2 || desc.rows < 2 {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_cloth_create: grid must be ≥ 2×2",
            );
            return u32::MAX;
        }
        if (desc.cols as u64) * (desc.rows as u64) > CLOTH_MAX_PARTICLES as u64 {
            set_error(
                ERR_CAPACITY,
                "soft_cloth_create: grid exceeds CLOTH_MAX_PARTICLES",
            );
            return u32::MAX;
        }
        if !desc.spacing.is_finite()
            || desc.spacing <= 0.0
            || !desc.particle_mass.is_finite()
            || desc.particle_mass <= 0.0
            || !desc.stiffness.is_finite()
            || desc.stiffness < 0.0
            || !desc.damping.is_finite()
            || desc.damping < 0.0
            || !desc.shear_ratio.is_finite()
            || !(0.0..=1.0).contains(&desc.shear_ratio)
            || !desc.bend_ratio.is_finite()
            || !(0.0..=1.0).contains(&desc.bend_ratio)
            || !vec3_finite(desc.origin)
        {
            set_error(
                ERR_INVALID_ARGUMENT,
                "soft_cloth_create: invalid scalar params",
            );
            return u32::MAX;
        }

        // Axes: finite, non-zero, normalised, not (near-)parallel.
        if !vec3_finite(desc.u_axis) || !vec3_finite(desc.v_axis) {
            set_error(ERR_INVALID_ARGUMENT, "soft_cloth_create: non-finite axes");
            return u32::MAX;
        }
        let uv = vec3_to_rapier(desc.u_axis);
        let vv = vec3_to_rapier(desc.v_axis);
        let (ul, vl) = (uv.length(), vv.length());
        if ul <= 1e-9 || vl <= 1e-9 {
            set_error(ERR_INVALID_ARGUMENT, "soft_cloth_create: zero-length axis");
            return u32::MAX;
        }
        let (u, v) = (uv / ul, vv / vl);
        if u.cross(v).length() < 1e-6 {
            set_error(ERR_INVALID_ARGUMENT, "soft_cloth_create: parallel axes");
            return u32::MAX;
        }

        let mut body = SoftBody::new(world.inner.gravity);
        let inv_mass = 1.0 / desc.particle_mass;
        for row in 0..rows {
            for col in 0..cols {
                let pos = vec3_to_rapier(desc.origin)
                    + u * (col as f64 * desc.spacing)
                    + v * (row as f64 * desc.spacing);
                let idx = if is_pinned(pin_mode, col, row, cols, rows) {
                    body.add_pinned(pos)
                } else {
                    let i = body.add_particle(pos);
                    body.particles[i].inv_mass = inv_mass;
                    i
                };
                debug_assert_eq!(idx, row * cols + col);
            }
        }

        // Spring families. Rest lengths come from the just-written grid
        // positions, so every spring rests exactly on the flat cloth.
        let k_struct = desc.stiffness;
        let k_shear = desc.stiffness * desc.shear_ratio;
        let k_bend = desc.stiffness * desc.bend_ratio;
        let at = |col: usize, row: usize| row * cols + col;
        let link = |body: &mut SoftBody, a: usize, b: usize, k: f64| {
            if k > 0.0 {
                body.add_spring(a, b, k, desc.damping);
            }
        };
        for row in 0..rows {
            for col in 0..cols {
                let i = at(col, row);
                // Structural: +u and +v neighbours.
                if col + 1 < cols {
                    link(&mut body, i, at(col + 1, row), k_struct);
                }
                if row + 1 < rows {
                    link(&mut body, i, at(col, row + 1), k_struct);
                }
                // Shear: both diagonals of the cell above-right of (col,row).
                if k_shear > 0.0 && col + 1 < cols && row + 1 < rows {
                    link(&mut body, i, at(col + 1, row + 1), k_shear);
                    link(&mut body, at(col + 1, row), at(col, row + 1), k_shear);
                }
                // Bend: two-apart neighbours along both axes.
                if col + 2 < cols {
                    link(&mut body, i, at(col + 2, row), k_bend);
                }
                if row + 2 < rows {
                    link(&mut body, i, at(col, row + 2), k_bend);
                }
            }
        }

        clear_error();
        world.inner.soft_bodies.insert(body).0
    })
}
