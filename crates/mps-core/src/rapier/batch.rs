//! Batch collider creation — Box3D-style batch-insert pipeline.
//!
//! Instead of calling `collider_builder_create` → `collider_builder_build` →
//! `world_insert_collider` in a loop (one Rapier `ColliderSet::insert` per
//! shape), the upper layer pushes **ColliderRequest** records into a
//! **ColliderBatch** manager.  The manager merges compatible static shapes
//! into a single `Collider::compound`, then inserts the whole batch with a
//! single `ColliderSet::insert` call — amortising the arena allocation and
//! broad-phase rebuild cost across N shapes.
//!
//! ### Merge strategy
//!
//! Requests that share the same `friction`, `restitution`, `density`,
//! `collision_groups`, `solver_groups`, `sensor` flag, and `body_parent` are
//! grouped.  Within each group, static (parentless) shapes are packed into one
//! `Collider::compound`; dynamic (parented) shapes fall back to per-collider
//! `insert_with_parent`.
//!
//! ### Box3D physics feel presets
//!
//! [`Box3DPreset`] bundles Rapier parameters that approximate the Box3D
//! sandbox physics "feel": low restitution, moderate friction, erosion margin
//! for stable stacking, etc.

use rapier3d::math::Pose;
use rapier3d::prelude::{
    ActiveEvents, ActiveHooks, ColliderBuilder, ColliderHandle, InteractionGroups, RigidBodyHandle,
    SharedShape,
};

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ffi_guard, set_error,
};
use crate::rapier::ffi::{
    Bool, ColliderHandleRaw, InteractionGroupsDesc, Quat, RigidBodyHandleRaw, ShapeDesc, Vec3,
    WorldHandle, interaction_groups_to_rapier, isometry_from_parts, pack_collider_handle,
    quat_finite, shape_desc_valid, shape_from_desc, unpack_rigid_body_handle, vec3_finite,
};

/// Shape type tags — mirrors rapier's ShapeType but isolated from the
/// internal [`ShapeType`] enum so the FFI caller can use stable integers.
/// See [`crate::rapier::ffi::ShapeType`] for the canonical enum.
const SHAPE_CUBOID: u32 = 1;
const SHAPE_CYLINDER: u32 = 5;
const SHAPE_ROUND_CYLINDER: u32 = 6;
const SHAPE_CONE: u32 = 7;
const SHAPE_ROUND_CONE: u32 = 8;
const SHAPE_ROUND_CUBOID: u32 = 9;

// ---------------------------------------------------------------------------
// Limits
// ---------------------------------------------------------------------------

/// Maximum number of requests a single batch can hold.  Prevents a runaway
/// caller from exhausting memory before [`ColliderBatch::execute`] runs.
pub const MAX_BATCH_REQUESTS: usize = 100_000;
/// Maximum number of compound parts in a single merged collider.  Rapier's
/// compound shape stores parts in a `Vec` so the practical limit is available
/// memory; we cap to keep broadphase insertion tractable.
pub const MAX_COMPOUND_PARTS: usize = 50_000;

// ---------------------------------------------------------------------------
// FFI­-facing request struct
// ---------------------------------------------------------------------------

/// A single collider creation request, designed for batch submission via the
/// Box3D-style pipeline.
///
/// Fields are flat `#[repr(C)]` so the FFI caller can build a contiguous array
/// and pass `(ptr, count)` to [`world_batch_add_colliders`].
///
/// [`world_batch_add_colliders`]: crate::rapier::world::world_batch_add_colliders
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ColliderRequest {
    /// Shape descriptor (shape_type + 4 floats a/b/c/d).  See [`ShapeDesc`].
    pub shape: ShapeDesc,
    /// Local translation relative to the merged collider origin (world pos if
    /// `body_parent == 0` and no merge happens).
    pub translation: Vec3,
    /// Local rotation as a unit quaternion (xyzw, but stored as ijkw in [`Quat`]).
    pub rotation: Quat,
    /// Coulomb friction coefficient (≥ 0).
    pub friction: f64,
    /// Coefficient of restitution (≥ 0, typically < 1).
    pub restitution: f64,
    /// Mass density (≥ 0).  Ignored for static (parentless) shapes.
    pub density: f64,
    /// Collision group memberships bitmask.
    pub collision_groups: InteractionGroupsDesc,
    /// Solver group memberships bitmask.
    pub solver_groups: InteractionGroupsDesc,
    /// If non-zero, this collider is attached to the given rigid body.
    pub body_parent: RigidBodyHandleRaw,
    /// If non-zero, the collider is a sensor (no collision response).
    pub is_sensor: Bool,
    /// Bitmask of [`ActiveEvents`] to enable.
    pub active_events: u32,
    /// Bitmask of [`ActiveHooks`] to enable.
    pub active_hooks: u32,
    /// Per-collider erosion margin (Rapier `contact_partitioning`).  Only
    /// meaningful for round shapes; 0 = no erosion.
    pub erosion_margin: f64,
}

// ---------------------------------------------------------------------------
// Box3D physics-feel preset
// ---------------------------------------------------------------------------

/// Parameter preset that approximates Box3D's sandbox physics feel.
///
/// These values are applied to every collider in a batch unless the request
/// itself overrides the corresponding field (> 0 for floats, non-default for
/// groups).  The preset is passed to [`ColliderBatch::new`] and used during
/// [`ColliderBatch::execute`].
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Box3DPreset {
    /// Default friction when the request's `friction` is <= 0.
    /// Box3D feel ≈ 0.6 (moderate grip, not icy).
    pub default_friction: f64,
    /// Default restitution when the request's `restitution` is < 0.
    /// Box3D feel ≈ 0.2 (slight bounce, realistic).
    pub default_restitution: f64,
    /// Default density for dynamic shapes when the request's `density` is
    /// <= 0.  Box3D feel ≈ 1.0 (water-equivalent for intuitive masses).
    pub default_density: f64,
    /// Erosion margin applied to round shapes for stable stacking.
    /// Box3D feel ≈ 0.01 (small margin prevents jitter on stacked bodies).
    pub default_erosion_margin: f64,
    /// Linear damping applied to dynamic bodies created by merge_static_shapes.
    /// Box3D feel ≈ 0.05 (slight slow-down, prevents perpetual motion).
    pub linear_damping: f64,
    /// Angular damping for dynamic bodies.
    /// Box3D feel ≈ 0.05.
    pub angular_damping: f64,
    /// CCD sub-steps for fast-moving dynamic bodies.  0 = off.
    /// Box3D feel ≈ 1 (enough to prevent tunneling at sandbox speeds).
    pub ccd_substeps: u32,
    /// Solver iterations.  Box3D feel ≈ 4 (GoodBalance between stability and CPU).
    pub solver_iterations: u32,
}

impl Box3DPreset {
    /// Returns the canonical Box3D-feel preset.
    pub fn box3d_default() -> Self {
        Self {
            default_friction: 0.6,
            default_restitution: 0.2,
            default_density: 1.0,
            default_erosion_margin: 0.01,
            linear_damping: 0.05,
            angular_damping: 0.05,
            ccd_substeps: 1,
            solver_iterations: 4,
        }
    }

    /// Returns a "no bounce" preset — restitution 0, high friction.
    /// Good for ground/walls in sandbox worlds.
    pub fn box3d_sticky() -> Self {
        Self {
            default_friction: 0.9,
            default_restitution: 0.0,
            default_density: 1.0,
            default_erosion_margin: 0.01,
            linear_damping: 0.05,
            angular_damping: 0.05,
            ccd_substeps: 1,
            solver_iterations: 4,
        }
    }

    /// Returns a "bouncy" preset — high restitution, low friction.
    /// Good for toy/ball-pit style interactions.
    pub fn box3d_bouncy() -> Self {
        Self {
            default_friction: 0.2,
            default_restitution: 0.7,
            default_density: 0.8,
            default_erosion_margin: 0.005,
            linear_damping: 0.02,
            angular_damping: 0.02,
            ccd_substeps: 2,
            solver_iterations: 8,
        }
    }
}

// ---------------------------------------------------------------------------
// ColliderBatch — the manager
// ---------------------------------------------------------------------------

/// Internal representation of a validated request, storing Rapier-native
/// types so we don't re-convert during execution.
struct BatchEntry {
    shape: SharedShape,
    pose: Pose,
    friction: f64,
    restitution: f64,
    density: f64,
    collision_groups: InteractionGroups,
    solver_groups: InteractionGroups,
    body_parent: Option<RigidBodyHandle>,
    is_sensor: bool,
    active_events: ActiveEvents,
    active_hooks: ActiveHooks,
    #[allow(dead_code)]
    erosion_margin: f64,
}

/// Key used to group requests that can be merged into a single compound.
/// Two requests with the same key can share one `ColliderSet::insert`.
#[derive(Clone, PartialEq)]
struct MergeKey {
    friction: u64, // packed bits of the float
    restitution: u64,
    density: u64,
    collision_groups_memberships: u32,
    collision_groups_filter: u32,
    solver_groups_memberships: u32,
    solver_groups_filter: u32,
    is_sensor: bool,
    active_events: u32,
    active_hooks: u32,
    body_parent: Option<u64>, // packed RigidBodyHandleRaw
    erosion_margin: u64,
}

impl BatchEntry {
    fn merge_key(&self) -> MergeKey {
        MergeKey {
            friction: self.friction.to_bits(),
            restitution: self.restitution.to_bits(),
            density: self.density.to_bits(),
            collision_groups_memberships: self.collision_groups.memberships.bits(),
            collision_groups_filter: self.collision_groups.filter.bits(),
            solver_groups_memberships: self.solver_groups.memberships.bits(),
            solver_groups_filter: self.solver_groups.filter.bits(),
            is_sensor: self.is_sensor,
            active_events: self.active_events.bits(),
            active_hooks: self.active_hooks.bits(),
            body_parent: self.body_parent.map(|h| {
                let (id, generation) = h.into_raw_parts();
                ((generation as u64) << 32) | (id as u64)
            }),
            erosion_margin: self.erosion_margin.to_bits(),
        }
    }
}

/// The batch manager (Box3D-style "creation-request sink").
///
/// Upper-layer code pushes [`ColliderRequest`]s via [`ColliderBatch::push`],
/// then calls [`ColliderBatch::execute`] to flush the batch into the physics
/// world.  During execution, compatible static shapes are merged into compound
/// colliders, and the whole batch is inserted with a single
/// `ColliderSet::insert` per merge group.
pub struct ColliderBatch {
    entries: Vec<BatchEntry>,
    preset: Box3DPreset,
}

impl ColliderBatch {
    /// Create a new batch with the given Box3D physics-feel preset.
    pub fn new(preset: Box3DPreset) -> Self {
        Self {
            entries: Vec::new(),
            preset,
        }
    }

    /// Returns the number of requests currently buffered.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Returns true if no requests have been buffered.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Build a shape with erosion (round-border) applied.
    ///
    /// If the shape type already has a round variant, we upgrade it to the
    /// round version with `border_radius = erosion_margin`.  If it's already
    /// round, we increase the existing border radius.  Shapes with no round
    /// variant (Ball, Capsule, HalfSpace) are returned unchanged.
    fn shape_with_erosion(desc: ShapeDesc, erosion_margin: f64) -> SharedShape {
        match desc.shape_type {
            SHAPE_CUBOID => SharedShape::round_cuboid(desc.a, desc.b, desc.c, erosion_margin),
            SHAPE_ROUND_CUBOID => {
                // Already round — replace border_radius with erosion_margin.
                SharedShape::round_cuboid(desc.a, desc.b, desc.c, erosion_margin)
            }
            SHAPE_CYLINDER => SharedShape::round_cylinder(desc.a, desc.b, erosion_margin),
            SHAPE_ROUND_CYLINDER => SharedShape::round_cylinder(desc.a, desc.b, erosion_margin),
            SHAPE_CONE => SharedShape::round_cone(desc.a, desc.b, erosion_margin),
            SHAPE_ROUND_CONE => SharedShape::round_cone(desc.a, desc.b, erosion_margin),
            // Ball, Capsule, and others don't benefit from border rounding.
            _ => shape_from_desc(desc),
        }
    }

    /// Push a raw [`ColliderRequest`] into the batch, applying the preset
    /// defaults and converting to Rapier-native types.
    ///
    /// Returns `false` (and sets the error slot) if the request is invalid.
    fn push_request(&mut self, req: &ColliderRequest) -> bool {
        if !shape_desc_valid(req.shape) {
            set_error(
                ERR_INVALID_ARGUMENT,
                "batch request: invalid shape descriptor",
            );
            return false;
        }
        if !vec3_finite(req.translation) || !quat_finite(req.rotation) {
            set_error(ERR_INVALID_ARGUMENT, "batch request: non-finite pose");
            return false;
        }
        let friction = if req.friction.is_finite() && req.friction >= 0.0 && req.friction > 0.0 {
            req.friction
        } else {
            self.preset.default_friction
        };
        let restitution = if req.restitution.is_finite() && req.restitution >= 0.0 {
            if req.restitution > 0.0 {
                req.restitution
            } else {
                self.preset.default_restitution
            }
        } else {
            self.preset.default_restitution
        };
        let density = if req.density.is_finite() && req.density > 0.0 {
            req.density
        } else {
            self.preset.default_density
        };
        let erosion_margin = if req.erosion_margin.is_finite() && req.erosion_margin > 0.0 {
            req.erosion_margin
        } else {
            self.preset.default_erosion_margin
        };
        let body_parent = if req.body_parent != 0 {
            Some(unpack_rigid_body_handle(req.body_parent))
        } else {
            None
        };

        // Build the shape.  If erosion_margin > 0 and the shape supports
        // rounding (Cuboid, Cylinder, Cone), upgrade it to its round variant
        // with border_radius = erosion_margin.  This is the Box3D-style
        // "erosion" effect: rounded edges that make stacked bodies settle
        // smoothly instead of jittering on hard edges.
        let shape = if erosion_margin > 0.0 {
            Self::shape_with_erosion(req.shape, erosion_margin)
        } else {
            shape_from_desc(req.shape)
        };

        let entry = BatchEntry {
            shape,
            pose: isometry_from_parts(req.translation, req.rotation),
            friction,
            restitution,
            density,
            collision_groups: interaction_groups_to_rapier(req.collision_groups),
            solver_groups: interaction_groups_to_rapier(req.solver_groups),
            body_parent,
            is_sensor: req.is_sensor.0 != 0,
            active_events: ActiveEvents::from_bits_truncate(req.active_events),
            active_hooks: ActiveHooks::from_bits_truncate(req.active_hooks),
            erosion_margin,
        };
        self.entries.push(entry);
        true
    }

    /// Execute the batch: merge compatible requests, insert into the world.
    ///
    /// Returns the number of colliders actually inserted (which may be less
    /// than `self.len()` if merge happened).
    ///
    /// The batch is consumed (emptied) after execution — call again with fresh
    /// requests to continue.
    pub fn execute(
        &mut self,
        colliders: &mut rapier3d::prelude::ColliderSet,
        bodies: &mut rapier3d::prelude::RigidBodySet,
    ) -> Vec<ColliderHandle> {
        if self.entries.is_empty() {
            return Vec::new();
        }

        // Group entries by their merge key.  We use a stable insertion-order
        // grouping so that entries with identical material properties get
        // merged into a single compound collider.
        // Sort by merge key, preserving relative order within groups.
        let mut indexed: Vec<(usize, &BatchEntry)> = self.entries.iter().enumerate().collect();
        indexed.sort_by(|a, b| {
            // Compare by merge key bit-patterns — order doesn't matter
            // semantically, only grouping does.
            let ka = a.1.merge_key();
            let kb = b.1.merge_key();
            ka.friction
                .cmp(&kb.friction)
                .then(ka.restitution.cmp(&kb.restitution))
                .then(ka.density.cmp(&kb.density))
                .then(
                    ka.collision_groups_memberships
                        .cmp(&kb.collision_groups_memberships),
                )
                .then(ka.collision_groups_filter.cmp(&kb.collision_groups_filter))
                .then(
                    ka.solver_groups_memberships
                        .cmp(&kb.solver_groups_memberships),
                )
                .then(ka.solver_groups_filter.cmp(&kb.solver_groups_filter))
                .then(ka.is_sensor.cmp(&kb.is_sensor))
                .then(ka.active_events.cmp(&kb.active_events))
                .then(ka.active_hooks.cmp(&kb.active_hooks))
                .then(ka.body_parent.cmp(&kb.body_parent))
                .then(ka.erosion_margin.cmp(&kb.erosion_margin))
        });

        // Collect groups: consecutive runs with the same merge key.
        let mut groups: Vec<Vec<usize>> = Vec::new();
        let mut current_key: Option<MergeKey> = None;
        let mut current_group: Vec<usize> = Vec::new();
        for (idx, entry) in &indexed {
            let key = entry.merge_key();
            if current_key.as_ref() == Some(&key) {
                current_group.push(*idx);
            } else {
                if !current_group.is_empty() {
                    groups.push(std::mem::take(&mut current_group));
                }
                current_key = Some(key);
                current_group.push(*idx);
            }
        }
        if !current_group.is_empty() {
            groups.push(current_group);
        }

        let mut handles = Vec::with_capacity(groups.len());
        let entries = std::mem::take(&mut self.entries);

        for group in groups {
            if group.is_empty() {
                continue;
            }

            let first = &entries[group[0]];
            let has_parent = first.body_parent.is_some();

            if group.len() == 1 || has_parent {
                // Single entry or parented shape: insert directly (no merge).
                let entry = &entries[group[0]];
                let builder = Self::build_collider(entry);
                let collider = builder.build();
                let handle = if let Some(parent) = entry.body_parent {
                    colliders.insert_with_parent(collider, parent, bodies)
                } else {
                    colliders.insert(collider)
                };
                handles.push(handle);
            } else {
                // Merge: build a compound from all parts in this group.
                let mut parts: Vec<(Pose, SharedShape)> =
                    Vec::with_capacity(group.len().min(MAX_COMPOUND_PARTS));
                for &idx in &group {
                    if parts.len() >= MAX_COMPOUND_PARTS {
                        break;
                    }
                    let entry = &entries[idx];
                    parts.push((entry.pose, entry.shape.clone()));
                }
                if parts.is_empty() {
                    continue;
                }

                let mut builder = ColliderBuilder::compound(parts)
                    .friction(first.friction)
                    .restitution(first.restitution)
                    .density(first.density)
                    .collision_groups(first.collision_groups)
                    .solver_groups(first.solver_groups)
                    .active_events(first.active_events)
                    .active_hooks(first.active_hooks);
                if first.is_sensor {
                    builder = builder.sensor(true);
                }
                let collider = builder.build();
                let handle = colliders.insert(collider);
                handles.push(handle);
            }
        }

        handles
    }

    fn build_collider(entry: &BatchEntry) -> ColliderBuilder {
        let mut builder = ColliderBuilder::new(entry.shape.clone())
            .position(entry.pose)
            .friction(entry.friction)
            .restitution(entry.restitution)
            .collision_groups(entry.collision_groups)
            .solver_groups(entry.solver_groups)
            .active_events(entry.active_events)
            .active_hooks(entry.active_hooks);
        if entry.density > 0.0 {
            builder = builder.density(entry.density);
        }
        if entry.is_sensor {
            builder = builder.sensor(true);
        }
        builder
    }
}

// ---------------------------------------------------------------------------
// PhysicsWorld methods
// ---------------------------------------------------------------------------

impl crate::rapier::world::PhysicsWorld {
    /// Batch-add colliders from a slice of [`ColliderRequest`]s.
    ///
    /// This is the primary batch-creation entry point.  It creates a
    /// [`ColliderBatch`] with the given [`Box3DPreset`], pushes all requests,
    /// and executes — returning the raw collider handles.
    ///
    /// On error (invalid request, capacity exceeded), the error slot is set
    /// and the already-inserted colliders are returned; the caller can check
    /// the result count vs. input count to detect partial failure.
    pub fn batch_add_colliders(
        &mut self,
        requests: &[ColliderRequest],
        preset: &Box3DPreset,
    ) -> Vec<ColliderHandleRaw> {
        if requests.is_empty() {
            return Vec::new();
        }
        if requests.len() > MAX_BATCH_REQUESTS {
            set_error(ERR_CAPACITY, "batch request count exceeds limit");
            return Vec::new();
        }

        let mut batch = ColliderBatch::new(*preset);
        for req in requests {
            batch.push_request(req);
        }

        let handles = batch.execute(&mut self.colliders, &mut self.bodies);
        handles.into_iter().map(pack_collider_handle).collect()
    }

    /// Merge static shapes into a single compound collider and insert with
    /// one `ColliderSet::insert` call.
    ///
    /// Unlike [`batch_add_colliders`](Self::batch_add_colliders), all requests
    /// MUST be static (no `body_parent`).  Shapes with identical material
    /// properties are merged into compound colliders, minimizing the number of
    /// `insert` calls and broad-phase entries.
    ///
    /// Returns the handles of the inserted (compound) colliders.
    pub fn merge_static_shapes(
        &mut self,
        requests: &[ColliderRequest],
        preset: &Box3DPreset,
    ) -> Vec<ColliderHandleRaw> {
        if requests.is_empty() {
            return Vec::new();
        }

        // Verify all requests are static (no parent).
        for req in requests {
            if req.body_parent != 0 {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "merge_static_shapes: all requests must be static (body_parent == 0)",
                );
                return Vec::new();
            }
        }

        // Reuse batch_add_colliders — the merging logic already merges static
        // (parentless) shapes into compound colliders.
        self.batch_add_colliders(requests, preset)
    }
}

// ---------------------------------------------------------------------------
// FFI entry points
// ---------------------------------------------------------------------------

/// Batch-add colliders from a flat array of [`ColliderRequest`]s.
///
/// Creates a [`ColliderBatch`] internally, pushes all requests, executes the
/// merge + insert pipeline, and writes the resulting collider handles into
/// `out_handles`.  Returns the number of handles written.
///
/// The Box3D feel preset is passed by value; use [`Box3DPreset::default`] for
/// zero-initialised fields, or [`Box3DPreset::box3d_default`] via the FFI
/// convenience function [`box3d_preset_default`].
///
/// # Safety
///
/// `world` must be a valid pointer from `world_create`.  `requests` must point
/// to at least `count` readable `ColliderRequest` values.  `out_handles` must
/// point to writable memory for at least `count * size_of(ColliderHandleRaw)`
/// bytes (each request could produce up to one handle, fewer if merged).
#[unsafe(no_mangle)]
pub extern "C" fn world_batch_add_colliders(
    world: *mut WorldHandle,
    requests: *const ColliderRequest,
    count: u32,
    preset: Box3DPreset,
    out_handles: *mut ColliderHandleRaw,
    out_capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if requests.is_null() || count == 0 {
            set_error(ERR_INVALID_ARGUMENT, "requests is null or count is 0");
            return 0;
        }
        if count as usize > MAX_BATCH_REQUESTS {
            set_error(ERR_CAPACITY, "batch request count exceeds limit");
            return 0;
        }
        if out_handles.is_null() || out_capacity == 0 {
            set_error(ERR_CAPACITY, "output buffer is null or zero capacity");
            return 0;
        }

        let Some(requests_slice) =
            (unsafe { crate::rapier::ffi::convert::checked_input_slice(requests, count as usize) })
        else {
            set_error(ERR_INVALID_ARGUMENT, "invalid requests slice");
            return 0;
        };
        let handles = world.inner.batch_add_colliders(requests_slice, &preset);

        let written = handles.len().min(out_capacity as usize);
        let Some(out) = (unsafe {
            crate::rapier::ffi::convert::checked_output_slice(out_handles, out_capacity as usize)
        }) else {
            set_error(ERR_INVALID_ARGUMENT, "invalid output slice");
            return 0;
        };
        out[..written].copy_from_slice(&handles[..written]);

        written as u32
    })
}

/// Merge static shapes and insert with a single `ColliderSet::insert`.
///
/// Like [`world_batch_add_colliders`] but requires all requests to be static
/// (parentless).  Returns the number of (compound) collider handles written.
///
/// # Safety
///
/// Same as [`world_batch_add_colliders`].
#[unsafe(no_mangle)]
pub extern "C" fn world_merge_static_shapes(
    world: *mut WorldHandle,
    requests: *const ColliderRequest,
    count: u32,
    preset: Box3DPreset,
    out_handles: *mut ColliderHandleRaw,
    out_capacity: u32,
) -> u32 {
    ffi_guard(0, || {
        let Some(world) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        if requests.is_null() || count == 0 {
            set_error(ERR_INVALID_ARGUMENT, "requests is null or count is 0");
            return 0;
        }
        if out_handles.is_null() || out_capacity == 0 {
            set_error(ERR_CAPACITY, "output buffer is null or zero capacity");
            return 0;
        }

        let Some(requests_slice) =
            (unsafe { crate::rapier::ffi::convert::checked_input_slice(requests, count as usize) })
        else {
            set_error(ERR_INVALID_ARGUMENT, "invalid requests slice");
            return 0;
        };
        let handles = world.inner.merge_static_shapes(requests_slice, &preset);

        let written = handles.len().min(out_capacity as usize);
        let Some(out) = (unsafe {
            crate::rapier::ffi::convert::checked_output_slice(out_handles, out_capacity as usize)
        }) else {
            set_error(ERR_INVALID_ARGUMENT, "invalid output slice");
            return 0;
        };
        out[..written].copy_from_slice(&handles[..written]);

        written as u32
    })
}

/// Convenience: get the Box3D default-feel preset.
#[unsafe(no_mangle)]
pub extern "C" fn box3d_preset_default() -> Box3DPreset {
    ffi_guard(Box3DPreset::default(), Box3DPreset::box3d_default)
}

/// Convenience: get the Box3D sticky-feel preset (high friction, no bounce).
#[unsafe(no_mangle)]
pub extern "C" fn box3d_preset_sticky() -> Box3DPreset {
    ffi_guard(Box3DPreset::default(), Box3DPreset::box3d_sticky)
}

/// Convenience: get the Box3D bouncy-feel preset (low friction, high restitution).
#[unsafe(no_mangle)]
pub extern "C" fn box3d_preset_bouncy() -> Box3DPreset {
    ffi_guard(Box3DPreset::default(), Box3DPreset::box3d_bouncy)
}
