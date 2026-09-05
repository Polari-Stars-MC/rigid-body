#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#pragma once

/* Generated with cbindgen:0.29.4 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Upper bound on balloon particles accepted in one creation call.
 *
 * The shell has `rings · segments + 2` particles and `add_triangle` dedups
 * its edges with an O(existing-edges) scan, giving O(n²) build cost — 4k
 * particles (~8k edges) keeps that well under a second while capping
 * allocation on hostile inputs.
 */
#define BALLOON_MAX_PARTICLES 4096

/**
 * Maximum number of requests a single batch can hold.  Prevents a runaway
 * caller from exhausting memory before [`ColliderBatch::execute`] runs.
 */
#define MAX_BATCH_REQUESTS 100000

/**
 * Maximum number of compound parts in a single merged collider.  Rapier's
 * compound shape stores parts in a `Vec` so the practical limit is available
 * memory; we cap to keep broadphase insertion tractable.
 */
#define MAX_COMPOUND_PARTS 50000

/**
 * Upper bound on cloth particles accepted in one creation call.
 *
 * 512 × 512 ≈ 262k particles ≈ 1M springs keeps a single cloth well inside
 * interactive-step territory and caps allocation on hostile inputs.
 */
#define CLOTH_MAX_PARTICLES 262144

#define ERR_OK 0

#define ERR_NULL_POINTER 1

#define ERR_INVALID_ARGUMENT 2

#define ERR_NOT_FOUND 3

#define ERR_CAPACITY 4

#define ERR_UNSUPPORTED 5

#define ERR_INTERNAL 6

/**
 * Upper bound on rope particles accepted in one creation call.
 *
 * Ropes are one-dimensional; 64k particles is far past any interactive
 * tether use and caps allocation on hostile inputs.
 */
#define ROPE_MAX_PARTICLES 65536

/**
 * Compression compliance used in unilateral (cable) mode.
 *
 * With `dt = 1/60 s` the XPBD projection weight `α/dt²` reaches ~3.6e12,
 * dwarfing typical inverse masses (~10), so the positional correction on the
 * compression side is ~1e-12 of a normal constraint — i.e. shortening is
 * free, which is exactly what a cable is.
 */
#define ROPE_CABLE_COMPRESSION_COMPLIANCE 1e9

/**
 * Gravitational constant (N·m²/kg²).
 */
#define G 6.67430e-11

/**
 * Magic number identifying a valid arena: "MPS_AREN"
 */
#define ARENA_MAGIC 5571044407640212814

/**
 * Current arena layout version — increment when layout changes
 */
#define ARENA_VERSION 2

/**
 * Strides (must match Java side exactly)
 */
#define BODY_SLOT_STRIDE 96

#define COLLIDER_SLOT_STRIDE 80

#define CMD_SLOT_STRIDE 32

#define EVENT_SLOT_STRIDE 64

/**
 * Header size in bytes
 */
#define HEADER_SIZE 128

/**
 * Upper bounds for arena capacities — defense against absurd FFI requests.
 */
#define MAX_ARENA_BODIES 1000000

#define MAX_ARENA_COLLIDERS 1000000

#define MAX_ARENA_EVENTS 1000000

#define MAX_ARENA_COMMANDS 1000000

/**
 * Hard cap on the total arena allocation (256 MiB).
 */
#define MAX_ARENA_TOTAL_BYTES ((256 * 1024) * 1024)

/**
 * Integration params region: dt(8) + solver_iterations(4) + ccd_substeps(4) + gravity(24)
 */
#define INTEGRATION_PARAMS_SIZE 40

/**
 * Force summary region: max_reynolds(8) + external force(24) + drag force(24) + counts(8)
 */
#define FORCE_SUMMARY_SIZE 64

/**
 * Aggregation policy applied to per-line accelerations after the
 * cross-consistency check.
 */
enum CrossValidateAggregation
#if defined(__cplusplus) || __STDC_VERSION__ >= 202311L
  : uint8_t
#endif // defined(__cplusplus) || __STDC_VERSION__ >= 202311L
 {
  /**
   * Newton-anchored: the Newton line is the reference. Each non-Newton
   * line contributes its difference from Newton as a *bounded* additive
   * correction (`a += correction_blend * (a_other − a_newton)`) **only**
   * if `|a_other − a_newton| / |a_newton| ≤ tolerance` — otherwise the
   * line is vetoed. This is the user's requested "牛顿力学为主、其他
   * 公式并行做验证修正、避免偏移太大" mode and is the default.
   */
  NewtonAnchored = 0,
  /**
   * Arithmetic mean of all surviving lines (vetoes excluded).
   */
  Mean = 1,
  /**
   * Median of all surviving lines (robust to a single divergent line).
   */
  Median = 2,
};
#ifndef __cplusplus
#if __STDC_VERSION__ >= 202311L
typedef enum CrossValidateAggregation CrossValidateAggregation;
#else
typedef uint8_t CrossValidateAggregation;
#endif // __STDC_VERSION__ >= 202311L
#endif // __cplusplus

typedef struct AnvilKitAppHandle AnvilKitAppHandle;

typedef struct CRbTreeHandle CRbTreeHandle;

typedef struct CharacterControllerHandle CharacterControllerHandle;

typedef struct ColliderBuilderHandle ColliderBuilderHandle;

typedef struct ForceQueueHeader ForceQueueHeader;

typedef struct JointBuilderHandle JointBuilderHandle;

typedef struct RTreeHandle RTreeHandle;

typedef struct RigidBodyBuilderHandle RigidBodyBuilderHandle;

typedef struct VoxelGrid VoxelGrid;

typedef struct WorldHandle WorldHandle;

/**
 * Descriptor for [`soft_balloon_create`].
 */
typedef struct BalloonDesc {
  /**
   * Latitude rings between the poles. Must be ≥ 2. Shell particle count is
   * `rings · segments + 2` (see [`BALLOON_MAX_PARTICLES`]).
   */
  uint32_t rings;
  /**
   * Longitude segments per ring. Must be ≥ 3.
   */
  uint32_t segments;
  /**
   * World position of the shell centre.
   */
  Vec3 center;
  /**
   * Shell radius. Must be > 0 and finite.
   */
  double radius;
  /**
   * Mass of each shell particle. Must be > 0 and finite.
   */
  double particle_mass;
  /**
   * XPBD compliance shared by every shell edge (tension side). `0` =
   * inextensible skin; larger = stretchier balloon. Must be ≥ 0.
   */
  double edge_compliance;
  /**
   * Initial internal pressure `P` (see the module docs). `0` starts the
   * balloon uninflated — pump it up later via `soft_body_set_pressure`.
   * Must be ≥ 0 and finite.
   */
  double pressure;
  /**
   * Gauss-Seidel projection iterations per XPBD substep. Must be ≥ 1.
   */
  uint32_t iterations;
} BalloonDesc;

/**
 * A single collider creation request, designed for batch submission via the
 * Box3D-style pipeline.
 *
 * Fields are flat `#[repr(C)]` so the FFI caller can build a contiguous array
 * and pass `(ptr, count)` to [`world_batch_add_colliders`].
 *
 * [`world_batch_add_colliders`]: crate::rapier::world::world_batch_add_colliders
 */
typedef struct ColliderRequest {
  /**
   * Shape descriptor (shape_type + 4 floats a/b/c/d).  See [`ShapeDesc`].
   */
  ShapeDesc shape;
  /**
   * Local translation relative to the merged collider origin (world pos if
   * `body_parent == 0` and no merge happens).
   */
  Vec3 translation;
  /**
   * Local rotation as a unit quaternion (xyzw, but stored as ijkw in [`Quat`]).
   */
  Quat rotation;
  /**
   * Coulomb friction coefficient (≥ 0).
   */
  double friction;
  /**
   * Coefficient of restitution (≥ 0, typically < 1).
   */
  double restitution;
  /**
   * Mass density (≥ 0).  Ignored for static (parentless) shapes.
   */
  double density;
  /**
   * Collision group memberships bitmask.
   */
  InteractionGroupsDesc collision_groups;
  /**
   * Solver group memberships bitmask.
   */
  InteractionGroupsDesc solver_groups;
  /**
   * If non-zero, this collider is attached to the given rigid body.
   */
  RigidBodyHandleRaw body_parent;
  /**
   * If non-zero, the collider is a sensor (no collision response).
   */
  Bool is_sensor;
  /**
   * Bitmask of [`ActiveEvents`] to enable.
   */
  uint32_t active_events;
  /**
   * Bitmask of [`ActiveHooks`] to enable.
   */
  uint32_t active_hooks;
  /**
   * Per-collider erosion margin (Rapier `contact_partitioning`).  Only
   * meaningful for round shapes; 0 = no erosion.
   */
  double erosion_margin;
} ColliderRequest;

/**
 * Parameter preset that approximates Box3D's sandbox physics feel.
 *
 * These values are applied to every collider in a batch unless the request
 * itself overrides the corresponding field (> 0 for floats, non-default for
 * groups).  The preset is passed to [`ColliderBatch::new`] and used during
 * [`ColliderBatch::execute`].
 */
typedef struct Box3DPreset {
  /**
   * Default friction when the request's `friction` is <= 0.
   * Box3D feel ≈ 0.6 (moderate grip, not icy).
   */
  double default_friction;
  /**
   * Default restitution when the request's `restitution` is < 0.
   * Box3D feel ≈ 0.2 (slight bounce, realistic).
   */
  double default_restitution;
  /**
   * Default density for dynamic shapes when the request's `density` is
   * <= 0.  Box3D feel ≈ 1.0 (water-equivalent for intuitive masses).
   */
  double default_density;
  /**
   * Erosion margin applied to round shapes for stable stacking.
   * Box3D feel ≈ 0.01 (small margin prevents jitter on stacked bodies).
   */
  double default_erosion_margin;
  /**
   * Linear damping applied to dynamic bodies created by merge_static_shapes.
   * Box3D feel ≈ 0.05 (slight slow-down, prevents perpetual motion).
   */
  double linear_damping;
  /**
   * Angular damping for dynamic bodies.
   * Box3D feel ≈ 0.05.
   */
  double angular_damping;
  /**
   * CCD sub-steps for fast-moving dynamic bodies.  0 = off.
   * Box3D feel ≈ 1 (enough to prevent tunneling at sandbox speeds).
   */
  uint32_t ccd_substeps;
  /**
   * Solver iterations.  Box3D feel ≈ 4 (GoodBalance between stability and CPU).
   */
  uint32_t solver_iterations;
} Box3DPreset;

/**
 * Descriptor for [`soft_cloth_create`].
 *
 * The cloth is generated in the plane spanned by `u_axis` (columns) and
 * `v_axis` (rows); the two axes must be finite, non-zero and not parallel.
 * Both are normalised internally, so their lengths are irrelevant.
 */
typedef struct ClothDesc {
  /**
   * Particles along `u_axis` (columns). Must be ≥ 2.
   */
  uint32_t cols;
  /**
   * Particles along `v_axis` (rows). Must be ≥ 2.
   */
  uint32_t rows;
  /**
   * Rest length between adjacent grid particles. Must be > 0 and finite.
   */
  double spacing;
  /**
   * World position of grid particle `(0, 0)`.
   */
  Vec3 origin;
  /**
   * Column direction (normalised internally).
   */
  Vec3 u_axis;
  /**
   * Row direction (normalised internally, must not be parallel to `u_axis`).
   */
  Vec3 v_axis;
  /**
   * Mass of each *free* particle. Must be > 0 and finite. Pinned particles
   * carry infinite mass regardless.
   */
  double particle_mass;
  /**
   * Structural spring stiffness (shear/bend are derived from it). ≥ 0.
   */
  double stiffness;
  /**
   * Spring damping, shared by all three spring families. ≥ 0.
   */
  double damping;
  /**
   * Shear stiffness = `stiffness · shear_ratio`, in `[0, 1]`. `0` disables
   * shear springs.
   */
  double shear_ratio;
  /**
   * Bend stiffness = `stiffness · bend_ratio`, in `[0, 1]`. `0` disables
   * bend springs.
   */
  double bend_ratio;
  /**
   * Border-pinning scheme, see [`ClothPinMode`].
   */
  uint32_t pin_mode;
} ClothDesc;

/**
 * Primary attractor descriptor used by every non-MOND line.
 */
typedef struct CrossValidateAttractor {
  /**
   * Gravitational parameter GM (m³/s²).  Must be > 0 for any line except
   * `MOND` (which is computed off the Newtonian seed).
   */
  double gm;
  /**
   * Equatorial radius (m), used by the J2/J6 line.
   */
  double equatorial_radius;
  /**
   * Zonal harmonic coefficients `[J2, J3, J4, J5, J6, ...]`.  May be empty
   * —— the J2 line will simply skip and report zero divergence contribution.
   */
  double jn[6];
  /**
   * Rotation rate (rad/s²) used by the centrifugal term folded into the
   * J2 line; pass 0 for a non-rotating primary.
   */
  double rotation_rate;
} CrossValidateAttractor;

/**
 * Boolean flag bits selecting which formula lines run this frame.
 *
 * Bit `i` set ⇒ line `i` participates.  At least one bit must be set; the
 * default (`NEWTON | J2 | QUADRUPOLE | MOND | RELATIVISTIC`) exercises all
 * five lines for maximum cross-validation coverage.
 */
typedef struct CrossValidateLineMask {
  uint64_t bits;
} CrossValidateLineMask;
#define CrossValidateLineMask_NEWTON (1 << 0)
#define CrossValidateLineMask_J2 (1 << 1)
#define CrossValidateLineMask_QUADRUPOLE (1 << 2)
#define CrossValidateLineMask_MOND (1 << 3)
#define CrossValidateLineMask_RELATIVISTIC (1 << 4)
#define CrossValidateLineMask_DEFAULT ((((CrossValidateLineMask_NEWTON | CrossValidateLineMask_J2) | CrossValidateLineMask_QUADRUPOLE) | CrossValidateLineMask_MOND) | CrossValidateLineMask_RELATIVISTIC)

/**
 * Configuration for the cross-validation gravity law.
 *
 * Set once via the `world_set_cross_validate_gravity` FFI; the law is
 * registered into `PhysicsWorld::force_registry` and re-applied every frame.
 */
typedef struct CrossValidateGravityConfig {
  /**
   * Primary attractor GM and figure parameters.
   */
  struct CrossValidateAttractor attractor;
  /**
   * Bitmask selecting which formula lines run.
   */
  struct CrossValidateLineMask mask;
  /**
   * Pairwise relative-difference tolerance; lines whose pairwise diff
   * exceeds this are flagged divergent and clipped.
   */
  double tolerance;
  /**
   * Newton-anchored correction blend factor in `[0, 1]`.  Only used by
   * `CrossValidateAggregation::NewtonAnchored`.  The accepted correction
   * contribution from each non-Newton line is scaled by this factor so the
   * frame-to-frame drift away from the Newton baseline stays bounded.
   * `0.0` ⇒ pure Newton (cross-validation only, no applied correction);
   * `1.0` ⇒ full schemaed correction once a line passes the tolerance
   * gate.  Default `1.0 / NUM_LINES as f64` ≈ `0.2` keeps the relative
   * drift per non-Newton line bounded.
   */
  double correction_blend;
  /**
   * Aggregation policy for the final acceleration vector applied to bodies.
   */
  CrossValidateAggregation aggregation;
  /**
   * MOND scale `a_0` (m/s²); 1.2e-10 is the canonical Milgrom value.
   * Only used when the MOND line is enabled.
   */
  double mond_a_zero;
  /**
   * Schwarzschild radius for the relativistic line (m).  Pass 0 to
   * auto-derive from `gm` as `rs = 2GM/c²`; ignored when the relativistic
   * line is disabled.
   */
  double schwarzschild_radius_override;
  /**
   * Enabled flag.  When `false`, the law reports itself disabled and
   * `apply()` is a no-op.
   */
  bool enabled;
} CrossValidateGravityConfig;

/**
 * Hair strand configuration.
 *
 * Passed as an array to `hair_system_create`; one strand becomes one soft
 * body whose root particle is bound to the attached rigid body.
 */
typedef struct HairStrandDesc {
  /**
   * Root position (in local space of the attached body).
   */
  Vec3 root_local;
  /**
   * Strand direction (in local space, normalized internally).
   */
  Vec3 direction;
  /**
   * Number of segments in this strand.
   */
  uint32_t segment_count;
  /**
   * Total length of the strand.
   */
  double length;
  /**
   * Radius of each hair segment (for collision).
   */
  double segment_radius;
  /**
   * Linear stiffness (spring constant k; lower = softer hair).
   */
  double stiffness;
  /**
   * Damping coefficient for the chain springs (0-1).
   */
  double damping;
  /**
   * Density of hair material.
   */
  double density;
} HairStrandDesc;

/**
 * Descriptor for [`soft_rope_create`].
 */
typedef struct RopeDesc {
  /**
   * Number of rope *segments*; the rope has `segments + 1` particles.
   * Must be ≥ 1 and ≤ [`ROPE_MAX_PARTICLES`] − 1.
   */
  uint32_t segments;
  /**
   * World position of the first particle.
   */
  Vec3 start;
  /**
   * World position of the last particle. Must be finite and farther than
   * 1e-9 from `start` (the rope is laid out along the straight span).
   */
  Vec3 end;
  /**
   * Mass of each *free* particle. Must be > 0 and finite. Pinned endpoints
   * carry infinite mass regardless.
   */
  double particle_mass;
  /**
   * XPBD stretch compliance `α_s` (tension side). `0` = inextensible;
   * larger = more elastic. Must be ≥ 0 and finite.
   */
  double stretch_compliance;
  /**
   * Rest-length slack factor: each segment's rest length is
   * `span/segments · (1 + slack)`. `0` = laid out exactly taut. Must be
   * ≥ 0 and finite.
   */
  double slack;
  /**
   * Gauss-Seidel projection iterations per XPBD substep. Must be ≥ 1.
   */
  uint32_t iterations;
  /**
   * When [`Bool::TRUE`], the rope only resists stretching (cable); when
   * [`Bool::FALSE`], it is a bilateral elastic cord.
   */
  Bool unilateral;
  /**
   * Endpoint pinning, see [`RopePinMode`].
   */
  uint32_t pin_mode;
} RopeDesc;

/**
 * A mass concentration (mascon) on the Moon's surface.
 */
typedef struct LunarMascon {
  /**
   * Center position (Moon-fixed, meters)
   */
  Vec3 center;
  /**
   * Excess mass (kg) — positive = mass excess
   */
  double excess_mass;
  /**
   * Radius of the mascon (m) — used for softening
   */
  double radius;
} LunarMascon;

/**
 * Output of `collider_voxel_ray_pick`: the voxel cell coordinate that a ray
 * hit on a voxel collider, plus the surface normal at the hit (so the caller
 * can derive the adjacent cell for "place on face").
 *
 * `found` is `FALSE` when the ray missed, hit a different collider, or the
 * resolved cell is out of the grid bounds.
 *
 * Layout (C ABI, read by Java via `Unsafe`):
 * `found` @0 (u8), `ix` @8, `iy` @16, `iz` @24 (i64),
 * `nx` @32, `ny` @40, `nz` @48 (f64). `SIZEOF` = 56.
 */
typedef struct VoxelCoord {
  Bool found;
  int64_t ix;
  int64_t iy;
  int64_t iz;
  double nx;
  double ny;
  double nz;
} VoxelCoord;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * Apply aerodynamic forces from a set of surfaces to a rigid body.
 *
 * # Safety
 *
 * `world` must be a valid, live world pointer; `surfaces` must point to at
 * least `surface_count` readable `AeroSurface`s; `out_report`, when
 * non-null, must be valid for a single `AeroForceReport` write.
 */
Bool aero_apply_surfaces(struct WorldHandle *world,
                         RigidBodyHandleRaw body_handle,
                         Vec3 wind_velocity,
                         double air_density,
                         const AeroSurface *surfaces,
                         uint32_t surface_count,
                         Bool wake_up,
                         AeroForceReport *out_report);

/**
 * Apply aerodynamic forces derived from a voxel grid to a rigid body.
 *
 * # Safety
 *
 * `world` must be a valid, live world pointer; `voxels` must point to at
 * least size_x×size_y×size_z readable bytes; `out_report`, when non-null,
 * must be valid for a single `AeroForceReport` write.
 */
Bool aero_apply_voxel_grid(struct WorldHandle *world,
                           RigidBodyHandleRaw body_handle,
                           Vec3 wind_velocity,
                           double air_density,
                           const uint8_t *voxels,
                           uint32_t size_x,
                           uint32_t size_y,
                           uint32_t size_z,
                           double voxel_size,
                           Vec3 local_origin,
                           double drag_coefficient,
                           double lift_coefficient,
                           Bool wake_up,
                           AeroForceReport *out_report);

/**
 * Flag-returning variant of `aero_apply_voxel_grid`.
 *
 * # Safety
 *
 * Same pointer contract as `aero_apply_voxel_grid`.
 */
uint8_t aero_apply_voxel_grid_flag(struct WorldHandle *world,
                                   RigidBodyHandleRaw body_handle,
                                   Vec3 wind_velocity,
                                   double air_density,
                                   const uint8_t *voxels,
                                   uint32_t size_x,
                                   uint32_t size_y,
                                   uint32_t size_z,
                                   double voxel_size,
                                   Vec3 local_origin,
                                   double drag_coefficient,
                                   double lift_coefficient,
                                   Bool wake_up,
                                   AeroForceReport *out_report);

/**
 * Flag-returning variant of `aero_apply_surfaces`.
 *
 * # Safety
 *
 * Same pointer contract as `aero_apply_surfaces`.
 */
uint8_t aero_apply_surfaces_flag(struct WorldHandle *world,
                                 RigidBodyHandleRaw body_handle,
                                 Vec3 wind_velocity,
                                 double air_density,
                                 const AeroSurface *surfaces,
                                 uint32_t surface_count,
                                 Bool wake_up,
                                 AeroForceReport *out_report);

/**
 * Estimate the aerodynamic force of a single surface without a world.
 *
 * # Safety
 *
 * `out_report`, when non-null, must be valid for a single `AeroForceReport`
 * write.
 */
Bool aero_estimate_surface_force(Vec3 body_linvel,
                                 Vec3 body_angvel,
                                 Vec3 body_center,
                                 Vec3 wind_velocity,
                                 double air_density,
                                 AeroSurface surface,
                                 AeroForceReport *out_report);

/**
 * Creates a new AnvilKit app state and returns an opaque handle to it.
 *
 * # Safety
 *
 * Takes no pointers and cannot fail on input; the returned handle is owned by
 * the caller and must eventually be passed to `anvilkit_app_destroy` (or
 * leaked).
 */
struct AnvilKitAppHandle *anvilkit_app_create(void);

/**
 * # Safety
 *
 * `app` must be null or a handle returned by `anvilkit_app_create` that has
 * not been destroyed yet; ownership transfers back to Rust and the handle is
 * invalid after this call.
 */
void anvilkit_app_destroy(struct AnvilKitAppHandle *app);

/**
 * # Safety
 *
 * `app` must be null or a valid handle returned by `anvilkit_app_create`.
 */
void anvilkit_app_update(struct AnvilKitAppHandle *app);

/**
 * # Safety
 *
 * `app` must be null or a valid handle returned by `anvilkit_app_create`.
 */
uint64_t anvilkit_app_spawn_body(struct AnvilKitAppHandle *app,
                                 Vec3 translation,
                                 Quat rotation,
                                 uint32_t status);

/**
 * # Safety
 *
 * `app` must be null or a valid handle returned by `anvilkit_app_create`.
 */
uint64_t anvilkit_app_spawn_body_with_collider(struct AnvilKitAppHandle *app,
                                               Vec3 translation,
                                               Quat rotation,
                                               uint32_t status,
                                               ShapeDesc shape);

/**
 * # Safety
 *
 * `app` must be null or a valid handle returned by `anvilkit_app_create`.
 */
Bool anvilkit_app_set_transform(struct AnvilKitAppHandle *app,
                                uint64_t entity_bits,
                                Vec3 translation,
                                Quat rotation);

/**
 * # Safety
 *
 * `app` must be null or a valid handle returned by `anvilkit_app_create`.
 */
Bool anvilkit_app_set_material(struct AnvilKitAppHandle *app,
                               uint64_t entity_bits,
                               MaterialProperties material);

/**
 * # Safety
 *
 * `app` and `world` must be null or valid handles returned by
 * `anvilkit_app_create` / the world-creation ABI.
 */
uint32_t anvilkit_app_sync_to_world(struct AnvilKitAppHandle *app, struct WorldHandle *world);

/**
 * # Safety
 *
 * `app` must be null or a valid handle returned by `anvilkit_app_create`.
 */
RigidBodyHandleRaw anvilkit_app_entity_to_body(const struct AnvilKitAppHandle *app,
                                               uint64_t entity_bits);

/**
 * # Safety
 *
 * `app` must be null or a valid handle returned by `anvilkit_app_create`.
 */
ColliderHandleRaw anvilkit_app_entity_to_collider(const struct AnvilKitAppHandle *app,
                                                  uint64_t entity_bits);

/**
 * # Safety
 *
 * `app` and `world` must be null or valid handles returned by
 * `anvilkit_app_create` / the world-creation ABI.
 */
uint32_t anvilkit_app_spawn_soft_body(struct AnvilKitAppHandle *app,
                                      struct WorldHandle *world,
                                      uint64_t entity_bits,
                                      double particle_mass,
                                      double stiffness,
                                      double damping,
                                      Bool pin);

/**
 * # Safety
 *
 * `app` must be null or a valid handle returned by `anvilkit_app_create`.
 */
uint32_t anvilkit_app_entity_to_soft_body(const struct AnvilKitAppHandle *app,
                                          uint64_t entity_bits);

/**
 * # Safety
 *
 * `app` and `world` must be null or valid handles returned by
 * `anvilkit_app_create` / the world-creation ABI.
 */
uint64_t anvilkit_app_create_constraint(struct AnvilKitAppHandle *app,
                                        struct WorldHandle *world,
                                        uint64_t entity1_bits,
                                        uint64_t entity2_bits,
                                        uint32_t joint_type,
                                        Vec3 axis_or_primary,
                                        double b,
                                        double c,
                                        Bool wake_up);

/**
 * # Safety
 *
 * `app` must be null or a valid handle returned by `anvilkit_app_create`.
 */
ImpulseJointHandleRaw anvilkit_app_constraint_to_joint(const struct AnvilKitAppHandle *app,
                                                       uint64_t constraint_id);

/**
 * # Safety
 *
 * `app` and `world` must be null or valid handles returned by
 * `anvilkit_app_create` / the world-creation ABI.
 */
Bool anvilkit_app_remove_constraint(struct AnvilKitAppHandle *app,
                                    struct WorldHandle *world,
                                    uint64_t constraint_id,
                                    Bool wake_up);

/**
 * # Safety
 *
 * `app` and `world` must be null or valid handles. `surfaces` must point to
 * `surface_count` readable `AeroSurface` entries, and `out_report` must be
 * null or point to a valid, writable `AeroForceReport`.
 */
Bool anvilkit_app_apply_aero_surfaces(struct AnvilKitAppHandle *app,
                                      struct WorldHandle *world,
                                      uint64_t entity_bits,
                                      Vec3 wind_velocity,
                                      double air_density,
                                      const AeroSurface *surfaces,
                                      uint32_t surface_count,
                                      Bool wake_up,
                                      AeroForceReport *out_report);

/**
 * # Safety
 *
 * `app` and `world` must be null or valid handles. `voxels` must point to at
 * least `size_x * size_y * size_z` readable bytes, and `out_report` must be
 * null or point to a valid, writable `AeroForceReport`.
 */
Bool anvilkit_app_apply_aero_voxel_grid(struct AnvilKitAppHandle *app,
                                        struct WorldHandle *world,
                                        uint64_t entity_bits,
                                        Vec3 wind_velocity,
                                        double air_density,
                                        const uint8_t *voxels,
                                        uint32_t size_x,
                                        uint32_t size_y,
                                        uint32_t size_z,
                                        double voxel_size,
                                        Vec3 local_origin,
                                        double drag_coefficient,
                                        double lift_coefficient,
                                        Bool wake_up,
                                        AeroForceReport *out_report);

/**
 * # Safety
 *
 * `app` and `world` must be null or valid handles. `out_report` must be null
 * or point to a valid, writable `FluidForceReport`.
 */
Bool anvilkit_app_apply_fluid_aabb_forces(struct AnvilKitAppHandle *app,
                                          struct WorldHandle *world,
                                          uint64_t entity_bits,
                                          FluidVolume fluid_volume,
                                          Vec3 body_half_extents,
                                          double body_volume,
                                          Bool wake_up,
                                          FluidForceReport *out_report);

/**
 * # Safety
 *
 * `app` and `world` must be null or valid handles. `out_report` must be null
 * or point to a valid, writable `TrajectoryForceReport`.
 */
Bool anvilkit_app_apply_trajectory_forces(struct AnvilKitAppHandle *app,
                                          struct WorldHandle *world,
                                          uint64_t entity_bits,
                                          TrajectoryEnvironment environment,
                                          Bool wake_up,
                                          TrajectoryForceReport *out_report);

/**
 * # Safety
 *
 * `out_report` must be null or point to a valid, writable `StressStrainReport`.
 */
Bool material_stress_strain_linear(MaterialProperties material,
                                   double strain,
                                   double delta_temperature,
                                   StressStrainReport *out_report);

/**
 * Computes the post-collision relative normal speed from restitution.
 *
 * # Safety
 *
 * All parameters are passed by value; this function performs no memory
 * access and is always memory-safe. Non-finite inputs or a negative
 * `restitution` yield `NaN`.
 */
double material_elastic_collision_relative_speed(double relative_normal_speed, double restitution);

/**
 * # Safety
 *
 * `out_report` must be null or point to a valid, writable `HertzContactReport`.
 */
Bool material_hertz_contact_force(MaterialProperties material1,
                                  MaterialProperties material2,
                                  double radius1,
                                  double radius2,
                                  double penetration,
                                  double penetration_rate,
                                  double damping,
                                  HertzContactReport *out_report);

/**
 * Create an articulated chain and return its id, or `u32::MAX` on error.
 *
 * `dir` is the chain direction (normalised internally), `joint_axis` the
 * local-space rotation axis of every revolute joint (must not be parallel to
 * `dir` — a perpendicular axis gives a planar arm). `target_angles` may be
 * null or shorter than `link_count − 1`; missing targets default to `0`.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null; `target_angles` must be null
 * or point to readable memory for `targets_len` doubles.
 */
uint32_t articulation_body_create(struct WorldHandle *world,
                                  Vec3 base,
                                  Vec3 dir,
                                  Vec3 joint_axis,
                                  uint32_t link_count,
                                  double link_radius,
                                  double link_mass,
                                  const double *target_angles,
                                  uint32_t targets_len,
                                  double stiffness,
                                  double damping);

/**
 * Rapier handle of chain link `index` (0 = base), for use with the existing
 * `rigid_body_*` / force FFI. Returns `0` on error.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null.
 */
uint64_t articulation_body_link_handle(const struct WorldHandle *world,
                                       uint32_t id,
                                       uint32_t link_index);

/**
 * Number of links in an articulation. Returns `u32::MAX` for an unknown id.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null.
 */
uint32_t articulation_body_link_count(const struct WorldHandle *world, uint32_t id);

/**
 * Retarget joint `joint_index`'s position motor at runtime (0-based, joint `i`
 * drives link `i` relative to link `i-1`). Reuses the gains stored at
 * creation. The whole chain is woken up so the new target takes effect
 * immediately. Returns `Bool::TRUE` on success.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null.
 */
Bool articulation_body_set_joint_target(struct WorldHandle *world,
                                        uint32_t id,
                                        uint32_t joint_index,
                                        double target_angle);

/**
 * Create an inflated balloon: a closed, pressurized sphere shell.
 *
 * Returns the new `SoftBodyId` (as `u32`), or `u32::MAX` with the
 * thread-local error slot set to an `ERR_*` code on failure. The balloon
 * integrates automatically in `world_step` — no separate stepping call is
 * needed.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null (null reports
 * `ERR_NULL_POINTER` and returns `u32::MAX`). No other pointers are
 * dereferenced; `desc` is passed by value.
 */
uint32_t soft_balloon_create(struct WorldHandle *world, struct BalloonDesc desc);

/**
 * Batch-add colliders from a flat array of [`ColliderRequest`]s.
 *
 * Creates a [`ColliderBatch`] internally, pushes all requests, executes the
 * merge + insert pipeline, and writes the resulting collider handles into
 * `out_handles`.  Returns the number of handles written.
 *
 * The Box3D feel preset is passed by value; use [`Box3DPreset::default`] for
 * zero-initialised fields, or [`Box3DPreset::box3d_default`] via the FFI
 * convenience function [`box3d_preset_default`].
 *
 * # Safety
 *
 * `world` must be a valid pointer from `world_create`.  `requests` must point
 * to at least `count` readable `ColliderRequest` values.  `out_handles` must
 * point to writable memory for at least `count * size_of(ColliderHandleRaw)`
 * bytes (each request could produce up to one handle, fewer if merged).
 */
uint32_t world_batch_add_colliders(struct WorldHandle *world,
                                   const struct ColliderRequest *requests,
                                   uint32_t count,
                                   struct Box3DPreset preset,
                                   ColliderHandleRaw *out_handles,
                                   uint32_t out_capacity);

/**
 * Merge static shapes and insert with a single `ColliderSet::insert`.
 *
 * Like [`world_batch_add_colliders`] but requires all requests to be static
 * (parentless).  Returns the number of (compound) collider handles written.
 *
 * # Safety
 *
 * Same as [`world_batch_add_colliders`].
 */
uint32_t world_merge_static_shapes(struct WorldHandle *world,
                                   const struct ColliderRequest *requests,
                                   uint32_t count,
                                   struct Box3DPreset preset,
                                   ColliderHandleRaw *out_handles,
                                   uint32_t out_capacity);

/**
 * Convenience: get the Box3D default-feel preset.
 */
struct Box3DPreset box3d_preset_default(void);

/**
 * Convenience: get the Box3D sticky-feel preset (high friction, no bounce).
 */
struct Box3DPreset box3d_preset_sticky(void);

/**
 * Convenience: get the Box3D bouncy-feel preset (low friction, high restitution).
 */
struct Box3DPreset box3d_preset_bouncy(void);

/**
 * # Safety
 *
 * The returned builder is owned by the caller and must be consumed by
 * `collider_builder_build` or freed with `collider_builder_destroy`.
 */
struct ColliderBuilderHandle *collider_builder_create_capsule(Capsule capsule);

/**
 * # Safety
 *
 * The returned builder is owned by the caller and must be consumed by
 * `collider_builder_build` or freed with `collider_builder_destroy`.
 */
struct ColliderBuilderHandle *collider_builder_create_ssv(Ssv ssv);

/**
 * # Safety
 *
 * The returned builder is owned by the caller and must be consumed by
 * `collider_builder_build` or freed with `collider_builder_destroy`.
 */
struct ColliderBuilderHandle *collider_builder_create_ellipsoid(Ellipsoid ellipsoid);

/**
 * # Safety
 *
 * The returned builder is owned by the caller and must be consumed by
 * `collider_builder_build` or freed with `collider_builder_destroy`.
 */
struct ColliderBuilderHandle *collider_builder_create_prism(Prism prism);

/**
 * # Safety
 *
 * The returned builder is owned by the caller and must be consumed by
 * `collider_builder_build` or freed with `collider_builder_destroy`.
 */
struct ColliderBuilderHandle *collider_builder_create_cylinder(Cylinder cylinder);

/**
 * # Safety
 *
 * The returned builder is owned by the caller and must be consumed by
 * `collider_builder_build` or freed with `collider_builder_destroy`.
 */
struct ColliderBuilderHandle *collider_builder_create_spherical_shell(SphericalShell shell);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_capsule_count(const struct WorldHandle *world,
                                       Capsule capsule,
                                       QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_capsule_count_all(const struct WorldHandle *world, Capsule capsule);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_capsule(const struct WorldHandle *world,
                                 Capsule capsule,
                                 QueryFilterDesc filter,
                                 ColliderHandleRaw *out_handles,
                                 uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_capsule_all(const struct WorldHandle *world,
                                     Capsule capsule,
                                     ColliderHandleRaw *out_handles,
                                     uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_ssv_count(const struct WorldHandle *world,
                                   Ssv ssv,
                                   QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_ssv_count_all(const struct WorldHandle *world, Ssv ssv);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_ssv(const struct WorldHandle *world,
                             Ssv ssv,
                             QueryFilterDesc filter,
                             ColliderHandleRaw *out_handles,
                             uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_ssv_all(const struct WorldHandle *world,
                                 Ssv ssv,
                                 ColliderHandleRaw *out_handles,
                                 uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_ellipsoid_count(const struct WorldHandle *world,
                                         Ellipsoid ellipsoid,
                                         QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_ellipsoid_count_all(const struct WorldHandle *world, Ellipsoid ellipsoid);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_ellipsoid(const struct WorldHandle *world,
                                   Ellipsoid ellipsoid,
                                   QueryFilterDesc filter,
                                   ColliderHandleRaw *out_handles,
                                   uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_ellipsoid_all(const struct WorldHandle *world,
                                       Ellipsoid ellipsoid,
                                       ColliderHandleRaw *out_handles,
                                       uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_prism_count(const struct WorldHandle *world,
                                     Prism prism,
                                     QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_prism_count_all(const struct WorldHandle *world, Prism prism);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_prism(const struct WorldHandle *world,
                               Prism prism,
                               QueryFilterDesc filter,
                               ColliderHandleRaw *out_handles,
                               uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_prism_all(const struct WorldHandle *world,
                                   Prism prism,
                                   ColliderHandleRaw *out_handles,
                                   uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_cylinder_count(const struct WorldHandle *world,
                                        Cylinder cylinder,
                                        QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_cylinder_count_all(const struct WorldHandle *world, Cylinder cylinder);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_cylinder(const struct WorldHandle *world,
                                  Cylinder cylinder,
                                  QueryFilterDesc filter,
                                  ColliderHandleRaw *out_handles,
                                  uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_cylinder_all(const struct WorldHandle *world,
                                      Cylinder cylinder,
                                      ColliderHandleRaw *out_handles,
                                      uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_spherical_shell_count(const struct WorldHandle *world,
                                               SphericalShell shell,
                                               QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_spherical_shell_count_all(const struct WorldHandle *world,
                                                   SphericalShell shell);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_spherical_shell(const struct WorldHandle *world,
                                         SphericalShell shell,
                                         QueryFilterDesc filter,
                                         ColliderHandleRaw *out_handles,
                                         uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_spherical_shell_all(const struct WorldHandle *world,
                                             SphericalShell shell,
                                             ColliderHandleRaw *out_handles,
                                             uint32_t capacity);

/**
 * Create a cloth body as a rectangular mass-spring grid.
 *
 * Returns the new `SoftBodyId` (as `u32`), or `u32::MAX` with the thread-local
 * error slot set to an `ERR_*` code on failure. The cloth integrates
 * automatically in `world_step` — no separate stepping call is needed.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null (null reports
 * `ERR_NULL_POINTER` and returns `u32::MAX`). No other pointers are
 * dereferenced; `desc` is passed by value.
 */
uint32_t soft_cloth_create(struct WorldHandle *world, struct ClothDesc desc);

/**
 * Creates a compound collider builder from a packed array of axis-aligned boxes.
 *
 * # Safety
 *
 * `box_data` must point to at least `box_count * 6` readable `f64` values,
 * each box described as min_x, min_y, min_z, max_x, max_y, max_z.
 */
struct ColliderBuilderHandle *collider_builder_create_compound_boxes(const double *box_data,
                                                                     uint32_t box_count);

/**
 * Creates a collider builder from a generic shape type and packed shape data.
 *
 * # Safety
 *
 * All parameters are passed by value; no raw pointers are dereferenced.
 * An invalid shape descriptor fails with `ERR_INVALID_ARGUMENT` and returns null.
 */
struct ColliderBuilderHandle *collider_builder_create(uint32_t shape_type, Vec3 shape_data);

/**
 * Creates a halfspace collider builder with the given plane normal.
 *
 * # Safety
 *
 * `normal` is passed by value; no raw pointers are dereferenced.
 * A non-finite normal fails with `ERR_INVALID_ARGUMENT` and returns null.
 */
struct ColliderBuilderHandle *collider_builder_create_halfspace(Vec3 normal);

/**
 * Creates a collider builder from an extended shape descriptor.
 *
 * # Safety
 *
 * `shape_desc` is passed by value; no raw pointers are dereferenced.
 * An invalid shape descriptor fails with `ERR_INVALID_ARGUMENT` and returns null.
 */
struct ColliderBuilderHandle *collider_builder_create_ex(ShapeDesc shape_desc);

/**
 * Creates an oriented box (cuboid) collider builder from an OBB descriptor.
 *
 * # Safety
 *
 * `obb` is passed by value; no raw pointers are dereferenced.
 * A non-finite center/rotation or non-positive half extents fail with
 * `ERR_INVALID_ARGUMENT` and return null.
 */
struct ColliderBuilderHandle *collider_builder_create_obb(Obb obb);

/**
 * Creates a ball collider builder from a sphere descriptor.
 *
 * # Safety
 *
 * `sphere` is passed by value; no raw pointers are dereferenced.
 * A non-finite center or a non-finite/non-positive radius fails with
 * `ERR_INVALID_ARGUMENT` and returns null.
 */
struct ColliderBuilderHandle *collider_builder_create_sphere(Sphere sphere);

/**
 * # Safety
 *
 * `data` must point to at least `data_x * data_y` readable `f64` height values.
 */
struct ColliderBuilderHandle *collider_builder_create_heightmap(const double *data,
                                                                uint32_t data_x,
                                                                uint32_t data_y,
                                                                Vec3 scale);

/**
 * # Safety
 *
 * `points_xyz` must point to at least `point_count * 3` readable `f64` values.
 */
struct ColliderBuilderHandle *collider_builder_create_convex_hull(const double *points_xyz,
                                                                  uint32_t point_count);

/**
 * # Safety
 *
 * `points_xyz` must point to at least `point_count * 3` readable `f64` values.
 */
struct ColliderBuilderHandle *collider_builder_create_point_cloud_bounds(const double *points_xyz,
                                                                         uint32_t point_count);

/**
 * Creates a collider builder covering the union of two AABBs.
 *
 * # Safety
 *
 * `first` and `second` are passed by value; no raw pointers are dereferenced.
 * An invalid AABB (non-finite or `mins > maxs`) fails with
 * `ERR_INVALID_ARGUMENT` and returns null.
 */
struct ColliderBuilderHandle *collider_builder_create_double_bv(AabbDesc first, AabbDesc second);

/**
 * Creates a convex-hull collider builder from a skewed box (center + 3 axis vectors).
 *
 * # Safety
 *
 * All parameters are passed by value; no raw pointers are dereferenced.
 * Non-finite vectors or near-zero-length axes fail with `ERR_INVALID_ARGUMENT`
 * and return null.
 */
struct ColliderBuilderHandle *collider_builder_create_skewed_obb(Vec3 center,
                                                                 Vec3 axis_x,
                                                                 Vec3 axis_y,
                                                                 Vec3 axis_z);

/**
 * # Safety
 *
 * `points_xyz` must point to at least `point_count * 3` readable `f64` values.
 */
struct ColliderBuilderHandle *collider_builder_create_discrete_obb(const double *points_xyz,
                                                                   uint32_t point_count,
                                                                   uint32_t axis);

/**
 * # Safety
 *
 * `points_xyz` must point to at least `point_count * 3` readable `f64` values.
 */
struct ColliderBuilderHandle *collider_builder_create_fused_collapsing_bounds(const double *points_xyz,
                                                                              uint32_t point_count,
                                                                              double padding);

/**
 * # Safety
 *
 * `vertices_xyz` must point to at least `vertex_count * 3` readable `f64`
 * values and `edges` to at least `edge_count * 2` readable `u32` indices.
 */
struct ColliderBuilderHandle *collider_builder_create_edge_bvh(const double *vertices_xyz,
                                                               uint32_t vertex_count,
                                                               const uint32_t *edges,
                                                               uint32_t edge_count,
                                                               double radius);

/**
 * # Safety
 *
 * `spheres_xyzw` must point to at least `sphere_count * 4` readable `f64`
 * values (center xyz + radius per sphere).
 */
struct ColliderBuilderHandle *collider_builder_create_medial_spheres(const double *spheres_xyzw,
                                                                     uint32_t sphere_count);

/**
 * # Safety
 *
 * `builder` must be a pointer returned by a `collider_builder_create_*`
 * function. It is consumed by this call and must not be used afterwards.
 */
Collider *collider_builder_build(struct ColliderBuilderHandle *builder);

/**
 * # Safety
 *
 * `builder` must be a pointer returned by a `collider_builder_create_*`
 * function that has not been consumed by `collider_builder_build`.
 */
void collider_builder_destroy(struct ColliderBuilderHandle *builder);

/**
 * # Safety
 *
 * `collider` must be a pointer returned by `collider_builder_build` or
 * `world_copy_collider` that has not already been destroyed.
 */
void collider_destroy_raw(Collider *collider);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_translation(struct ColliderBuilderHandle *builder, Vec3 translation);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_rotation(struct ColliderBuilderHandle *builder, Vec3 rotation_axis_angle);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_pose(struct ColliderBuilderHandle *builder,
                               Vec3 translation,
                               Quat rotation);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_sensor(struct ColliderBuilderHandle *builder, Bool sensor);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_friction(struct ColliderBuilderHandle *builder, double friction);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_restitution(struct ColliderBuilderHandle *builder, double restitution);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_contact_skin(struct ColliderBuilderHandle *builder, double skin);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_density(struct ColliderBuilderHandle *builder, double density);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_collision_groups(struct ColliderBuilderHandle *builder,
                                           InteractionGroupsDesc groups);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_solver_groups(struct ColliderBuilderHandle *builder,
                                        InteractionGroupsDesc groups);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_active_events(struct ColliderBuilderHandle *builder,
                                        uint32_t active_events_bits);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_active_hooks(struct ColliderBuilderHandle *builder,
                                       uint32_t active_hooks_bits);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by a `collider_builder_create_*`
 * function and not yet consumed or destroyed.
 */
void collider_builder_set_contact_force_event_threshold(struct ColliderBuilderHandle *builder,
                                                        double threshold);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create`. `memory_handle`
 * must be a pointer returned by `collider_builder_build` or
 * `world_copy_collider`; it is consumed by this call.
 */
ColliderHandleRaw world_insert_collider(struct WorldHandle *world, Collider *memory_handle);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create`. `memory_handle`
 * must be a pointer returned by `collider_builder_build` or
 * `world_copy_collider`; it is consumed by this call.
 */
ColliderHandleRaw world_insert_collider_with_parent(struct WorldHandle *world,
                                                    Collider *memory_handle,
                                                    RigidBodyHandleRaw parent);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool world_remove_collider(struct WorldHandle *world, ColliderHandleRaw handle, Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Collider *world_copy_collider(struct WorldHandle *world, ColliderHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
uint8_t world_remove_collider_flag(struct WorldHandle *world,
                                   ColliderHandleRaw handle,
                                   Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Vec3 collider_get_translation(const struct WorldHandle *world, ColliderHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
uintptr_t collider_get_shape_count(const struct WorldHandle *world, ColliderHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create`; `out_translation`
 * must point to a writable `Vec3`.
 */
void collider_get_translation_out(const struct WorldHandle *world,
                                  ColliderHandleRaw handle,
                                  Vec3 *out_translation);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Quat collider_get_rotation(const struct WorldHandle *world, ColliderHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create`; `out_rotation`
 * must point to a writable `Quat`.
 */
void collider_get_rotation_out(const struct WorldHandle *world,
                               ColliderHandleRaw handle,
                               Quat *out_rotation);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_pose(struct WorldHandle *world,
                       ColliderHandleRaw handle,
                       Vec3 translation,
                       Quat rotation);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_translation(struct WorldHandle *world,
                              ColliderHandleRaw handle,
                              Vec3 translation);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_rotation(struct WorldHandle *world, ColliderHandleRaw handle, Quat rotation);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
uint8_t collider_set_pose_flag(struct WorldHandle *world,
                               ColliderHandleRaw handle,
                               Vec3 translation,
                               Quat rotation);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_sensor(struct WorldHandle *world, ColliderHandleRaw handle, Bool sensor);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
uint8_t collider_set_sensor_flag(struct WorldHandle *world, ColliderHandleRaw handle, Bool sensor);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_friction(struct WorldHandle *world, ColliderHandleRaw handle, double friction);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
uint8_t collider_set_friction_flag(struct WorldHandle *world,
                                   ColliderHandleRaw handle,
                                   double friction);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_restitution(struct WorldHandle *world,
                              ColliderHandleRaw handle,
                              double restitution);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_friction_combine_rule(struct WorldHandle *world,
                                        ColliderHandleRaw handle,
                                        uint32_t rule);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_restitution_combine_rule(struct WorldHandle *world,
                                           ColliderHandleRaw handle,
                                           uint32_t rule);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
uint8_t collider_set_restitution_flag(struct WorldHandle *world,
                                      ColliderHandleRaw handle,
                                      double restitution);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_collision_groups(struct WorldHandle *world,
                                   ColliderHandleRaw handle,
                                   InteractionGroupsDesc groups);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
uint8_t collider_set_collision_groups_flag(struct WorldHandle *world,
                                           ColliderHandleRaw handle,
                                           InteractionGroupsDesc groups);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_solver_groups(struct WorldHandle *world,
                                ColliderHandleRaw handle,
                                InteractionGroupsDesc groups);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
uint8_t collider_set_solver_groups_flag(struct WorldHandle *world,
                                        ColliderHandleRaw handle,
                                        InteractionGroupsDesc groups);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_active_events(struct WorldHandle *world,
                                ColliderHandleRaw handle,
                                uint32_t active_events_bits);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
uint8_t collider_set_active_events_flag(struct WorldHandle *world,
                                        ColliderHandleRaw handle,
                                        uint32_t active_events_bits);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_active_hooks(struct WorldHandle *world,
                               ColliderHandleRaw handle,
                               uint32_t active_hooks_bits);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
uint8_t collider_set_active_hooks_flag(struct WorldHandle *world,
                                       ColliderHandleRaw handle,
                                       uint32_t active_hooks_bits);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool collider_set_contact_force_event_threshold(struct WorldHandle *world,
                                                ColliderHandleRaw handle,
                                                double threshold);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
uint8_t collider_set_contact_force_event_threshold_flag(struct WorldHandle *world,
                                                        ColliderHandleRaw handle,
                                                        double threshold);

/**
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
double collider_get_density(const struct WorldHandle *world, ColliderHandleRaw handle);

/**
 * Insert a dynamic rigid body built from a list of cuboids.
 *
 * # Safety
 *
 * `world` must be a valid, live world pointer; `cuboids` must point to at
 * least 6×cuboid_count readable f64s (center xyz + half-extents xyz per
 * cuboid).
 */
RigidBodyHandleRaw world_insert_dynamic_cuboids(struct WorldHandle *world,
                                                Vec3 translation,
                                                Quat rotation,
                                                Vec3 linvel,
                                                const double *cuboids,
                                                uint32_t cuboid_count,
                                                double density,
                                                double friction,
                                                double restitution,
                                                InteractionGroupsDesc collision_groups,
                                                InteractionGroupsDesc solver_groups);

/**
 * Insert a fixed rigid body with a trimesh collider.
 *
 * # Safety
 *
 * `world` must be a valid, live world pointer; `vertices_xyz` must point to
 * at least `vertex_xyz_len` readable f64s and `indices` to at least
 * `index_len` readable u32s.
 */
RigidBodyHandleRaw world_insert_static_trimesh(struct WorldHandle *world,
                                               const double *vertices_xyz,
                                               uint32_t vertex_xyz_len,
                                               const uint32_t *indices,
                                               uint32_t index_len,
                                               double friction,
                                               double restitution);

/**
 * Count the rigid bodies intersecting an AABB.
 *
 * # Safety
 *
 * `world` must be a valid, live world pointer.
 */
uint32_t query_intersect_aabb_rigid_body_count(const struct WorldHandle *world,
                                               AabbDesc aabb,
                                               QueryFilterDesc filter);

/**
 * Collect the rigid body handles intersecting an AABB.
 *
 * # Safety
 *
 * `world` must be a valid, live world pointer; `out_handles` must be valid
 * for `capacity` `RigidBodyHandleRaw` writes.
 */
uint32_t query_intersect_aabb_rigid_bodies(const struct WorldHandle *world,
                                           AabbDesc aabb,
                                           QueryFilterDesc filter,
                                           RigidBodyHandleRaw *out_handles,
                                           uint32_t capacity);

/**
 * Create a character body in `world` from a collider shape and an initial
 * translation. Returns a stable id, or `u32::MAX` on bad arguments. The character
 * is a `KinematicPositionBased` rigid body so its position is driven externally
 * by [`character_body_move`]. A world collider is inserted and parented to the
 * body so the character participates in the dynamic world (it can push other
 * bodies during `world_step`); that collider is excluded from the controller's
 * own shape-cast via a `QueryFilter`, so the character never catches itself.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
uint32_t character_body_create(struct WorldHandle *world, ShapeDesc shape, Vec3 translation);

/**
 * Change a character body's collision shape after creation. The new shape is
 * used by subsequent `character_body_move` calls (the controller shape-casts the
 * shape directly) and is applied to the world collider in place (so the character
 * keeps pushing other bodies with the new hitbox). Useful for Minecraft style
 * avatars that change hitbox (e.g. sneaking shrinks the box).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `shape` must be a
 * valid [`ShapeDesc`] (finite params).
 */
Bool character_body_set_shape(struct WorldHandle *world, uint32_t id, ShapeDesc shape);

/**
 * Advance the character by `desired` (a desired translation for this step). The
 * controller resolves collisions/slopes/steps and the result is written back to
 * the kinematic body. Returns the effective movement (resolved translation,
 * `grounded`, `is_sliding_down_slope`).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
EffectiveCharacterMovement character_body_move(struct WorldHandle *world,
                                               uint32_t id,
                                               Vec3 desired,
                                               double dt);

/**
 * Set the character's up vector (used for slope/ground semantics). Defaults to
 * world +Y. Mirrors `KinematicCharacterController::setUp`.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool character_body_set_up(struct WorldHandle *world, uint32_t id, Vec3 up);

/**
 * Set the character controller's skin/offset (absolute, in metres).
 */
Bool character_body_set_offset_absolute(struct WorldHandle *world, uint32_t id, double offset);

/**
 * Set the character controller's skin/offset (relative, as a fraction of the
 * shape's dimensions).
 */
Bool character_body_set_offset_relative(struct WorldHandle *world, uint32_t id, double offset);

/**
 * Enable / disable auto-stepping so the character can climb block-sized ledges
 * (e.g. a 1-metre Minecraft step). `max_height` and `min_width` are absolute
 * metres; `include_dynamic_bodies` lets the step ride on moving platforms.
 */
Bool character_body_set_autostep(struct WorldHandle *world,
                                 uint32_t id,
                                 Bool enabled,
                                 double max_height,
                                 double min_width,
                                 Bool include_dynamic_bodies);

/**
 * Enable / disable snap-to-ground so the character sticks to block surfaces
 * instead of floating a hair above them after a step.
 */
Bool character_body_set_snap_to_ground(struct WorldHandle *world,
                                       uint32_t id,
                                       Bool enabled,
                                       double distance);

/**
 * Set the slope-climb / slope-slide angles (radians). Tune these so the
 * character climbs gentle block ramps but slides down steep ones.
 */
Bool character_body_set_slope_angles(struct WorldHandle *world,
                                     uint32_t id,
                                     double max_climb_angle,
                                     double min_slide_angle);

/**
 * Enable / disable sliding along walls/floors when the character is blocked.
 * `slide = true` gives the smooth Minecraft-style "glide along a wall" feel;
 * `slide = false` makes the character stop dead on contact.
 */
Bool character_body_set_slide(struct WorldHandle *world, uint32_t id, Bool slide);

/**
 * Whether the character was on the ground during the last `character_body_move`.
 * Essential for Minecraft-style jump logic (only jump when grounded).
 */
Bool character_body_is_grounded(const struct WorldHandle *world, uint32_t id);

/**
 * Whether the character was sliding down a slope during the last
 * `character_body_move`. Useful for Minecraft-style ice/slide behaviour.
 */
Bool character_body_is_sliding_down_slope(const struct WorldHandle *world, uint32_t id);

/**
 * Reliable "is the character standing on something" check for Minecraft-style
 * jump logic. This fork's `is_grounded` classifies a capsule resting on a flat
 * floor as `sliding_down_slope` (see `is_grounded_at_contact_manifold`'s normal
 * convention), so it alone is NOT a good jump gate. This helper ORs `grounded`
 * with `is_sliding_down_slope` and additionally excludes the case where the
 * character is moving strongly upward (i.e. already jumping), giving a stable
 * on-ground signal the caller can gate jumps on.
 */
Bool character_body_is_on_ground(const struct WorldHandle *world, uint32_t id);

/**
 * Number of collisions captured by the most recent `character_body_move`. Use
 * this with [`character_body_get_collision`] to inspect what the character hit
 * (e.g. to apply custom push forces or build a contact-reporting system).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
uint32_t character_body_collision_count(const struct WorldHandle *world, uint32_t id);

/**
 * Read the `index`-th collision captured by the most recent `character_body_move`.
 * Returns a default (all-zero) collision if `index` is out of range.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
CharacterCollision character_body_get_collision(const struct WorldHandle *world,
                                                uint32_t id,
                                                uint32_t index);

/**
 * Apply the impulses accumulated from the latest `character_body_move` to the
 * dynamic bodies the character is touching. This is how a kinematic character
 * "pushes" crates/other rigid bodies. For each captured collision that was
 * actually blocked (non-zero `translation_remaining`) against a dynamic body, we
 * apply an impulse of `character_mass * remaining / dt` along the blocked
 * direction — i.e. the momentum the character wanted to carry into the body.
 * Rapier's own `solve_character_collision_impulses` only separates bodies, so the
 * forward push is implemented here, with no fork changes. Call this after each
 * move that reported contacts.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool character_body_solve_impulses(struct WorldHandle *world,
                                   uint32_t id,
                                   double dt,
                                   double character_mass);

/**
 * Enable or disable transferring the character's intended momentum to the dynamic
 * bodies it is blocked against (default: enabled). When disabled, the character
 * still resolves against static geometry but does not shove dynamic bodies — it
 * "ghosts" through them. No fork changes.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool character_body_set_apply_impulses_to_dynamic_bodies(struct WorldHandle *world,
                                                         uint32_t id,
                                                         Bool enabled);

/**
 * Like [`character_body_move`] but additionally samples the world's registered
 * terrain gravity (polyhedron / DEM / lunar-mascon) at the character's current
 * position and folds the resulting free-fall displacement (`½·a·dt²`) into the
 * desired translation, so the character falls toward and stands on an irregular
 * small-body surface instead of floating. When no terrain-gravity law is
 * registered this is identical to `character_body_move`.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
EffectiveCharacterMovement character_body_move_with_terrain(struct WorldHandle *world,
                                                            uint32_t id,
                                                            Vec3 desired,
                                                            double dt);

/**
 * Destroy a character body, removing its rigid body, collider and controller
 * state. Returns `Bool::TRUE` on success.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool character_body_destroy(struct WorldHandle *world, uint32_t id);

/**
 * Read the character body's current world-space translation (the kinematic body
 * pose driven by [`character_body_move`]). Writes into `out` when non-null.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `out` may be null.
 */
Bool character_body_get_translation(const struct WorldHandle *world, uint32_t id, Vec3 *out);

/**
 * Creates a new character controller and returns an opaque handle to it.
 *
 * # Safety
 *
 * The returned pointer is owned by Rust and must be passed to
 * `character_controller_destroy` exactly once. Returns null on internal
 * failure (see `last_error_code`).
 */
struct CharacterControllerHandle *character_controller_create(void);

/**
 * # Safety
 *
 * `controller` must be a pointer returned by `character_controller_create` (or null,
 * which is a no-op). Ownership is transferred to Rust and the pointer must not be
 * used after this call.
 */
void character_controller_destroy(struct CharacterControllerHandle *controller);

/**
 * # Safety
 *
 * `controller` must be a valid pointer returned by `character_controller_create`
 * and must remain alive for the duration of the call.
 */
void character_controller_set_up(struct CharacterControllerHandle *controller, Vec3 up);

/**
 * # Safety
 *
 * `controller` must be a valid pointer returned by `character_controller_create`
 * and must remain alive for the duration of the call.
 */
void character_controller_set_offset_absolute(struct CharacterControllerHandle *controller,
                                              double offset);

/**
 * # Safety
 *
 * `controller` must be a valid pointer returned by `character_controller_create`
 * and must remain alive for the duration of the call.
 */
void character_controller_set_offset_relative(struct CharacterControllerHandle *controller,
                                              double offset);

/**
 * # Safety
 *
 * `controller` must be a valid pointer returned by `character_controller_create`
 * and must remain alive for the duration of the call.
 */
void character_controller_set_slide(struct CharacterControllerHandle *controller, Bool slide);

/**
 * # Safety
 *
 * `controller` must be a valid pointer returned by `character_controller_create`
 * and must remain alive for the duration of the call.
 */
void character_controller_set_autostep(struct CharacterControllerHandle *controller,
                                       Bool enabled,
                                       double max_height,
                                       double min_width,
                                       Bool include_dynamic_bodies);

/**
 * # Safety
 *
 * `controller` must be a valid pointer returned by `character_controller_create`
 * and must remain alive for the duration of the call.
 */
void character_controller_set_snap_to_ground(struct CharacterControllerHandle *controller,
                                             Bool enabled,
                                             double distance);

/**
 * # Safety
 *
 * `controller` must be a valid pointer returned by `character_controller_create`
 * and must remain alive for the duration of the call.
 */
void character_controller_set_slope_angles(struct CharacterControllerHandle *controller,
                                           double max_climb_angle,
                                           double min_slide_angle);

/**
 * # Safety
 *
 * `world` must be a valid world pointer and `controller` a valid pointer returned by
 * `character_controller_create`; both must remain alive for the duration of the call.
 */
EffectiveCharacterMovement character_controller_move_shape(const struct WorldHandle *world,
                                                           struct CharacterControllerHandle *controller,
                                                           double dt,
                                                           ShapeDesc shape_desc,
                                                           Vec3 translation,
                                                           Quat rotation,
                                                           Vec3 desired_translation);

/**
 * # Safety
 *
 * `controller` must be a valid pointer returned by `character_controller_create`
 * and must remain alive for the duration of the call.
 */
uint32_t character_controller_collision_count(const struct CharacterControllerHandle *controller);

/**
 * # Safety
 *
 * `controller` must be a valid pointer returned by `character_controller_create`
 * and must remain alive for the duration of the call.
 */
FfiCharacterCollision character_controller_get_collision(const struct CharacterControllerHandle *controller,
                                                         uint32_t index);

/**
 * # Safety
 *
 * `world` must be a valid world pointer and `controller` a valid pointer returned by
 * `character_controller_create`; both must remain alive for the duration of the call.
 */
Bool character_controller_solve_impulses(struct WorldHandle *world,
                                         struct CharacterControllerHandle *controller,
                                         double dt,
                                         ShapeDesc shape_desc,
                                         double character_mass);

/**
 * # Safety
 *
 * `world` must be a valid world pointer and `controller` a valid pointer returned by
 * `character_controller_create`; both must remain alive for the duration of the call.
 *
 * Like [`character_controller_move_shape`] but additionally samples the world's
 * registered terrain gravity (polyhedron / DEM / lunar-mascon) at the character's
 * current `translation` and folds the resulting free-fall displacement
 * (`½·a·dt²`, directed along the local terrain-gravity acceleration `a`) into the
 * desired translation.  This lets a kinematic character fall toward and stand on an
 * irregular small-body surface instead of floating.  When no terrain-gravity law is
 * registered the call is identical to `character_controller_move_shape`.
 */
EffectiveCharacterMovement character_controller_move_shape_with_terrain(const struct WorldHandle *world,
                                                                        struct CharacterControllerHandle *controller,
                                                                        double dt,
                                                                        ShapeDesc shape_desc,
                                                                        Vec3 translation,
                                                                        Quat rotation,
                                                                        Vec3 desired_translation);

/**
 * Create an empty red-black-tree AABB index.
 *
 * # Safety
 *
 * The returned pointer is owned by the caller and must be freed exactly once
 * with `crb_tree_destroy`.
 */
struct CRbTreeHandle *crb_tree_create(void);

/**
 * Destroy an index created by `crb_tree_create`.
 *
 * # Safety
 *
 * `tree` must be null or a pointer returned by `crb_tree_create`; it must not
 * be used again after this call.
 */
void crb_tree_destroy(struct CRbTreeHandle *tree);

/**
 * Remove every entry from the tree.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `crb_tree_create`.
 */
void crb_tree_clear(struct CRbTreeHandle *tree);

/**
 * Return the number of entries stored in the tree.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `crb_tree_create`.
 */
uint32_t crb_tree_len(const struct CRbTreeHandle *tree);

/**
 * Insert or overwrite the bounds of `id` in the tree.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `crb_tree_create`.
 */
Bool crb_tree_insert(struct CRbTreeHandle *tree, uint64_t id, AabbDesc aabb);

/**
 * Flag-returning variant of `crb_tree_insert`.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `crb_tree_create`.
 */
uint8_t crb_tree_insert_flag(struct CRbTreeHandle *tree, uint64_t id, AabbDesc aabb);

/**
 * Update the bounds of an existing `id` in the tree.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `crb_tree_create`.
 */
Bool crb_tree_update(struct CRbTreeHandle *tree, uint64_t id, AabbDesc aabb);

/**
 * Remove `id` from the tree.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `crb_tree_create`.
 */
Bool crb_tree_remove(struct CRbTreeHandle *tree, uint64_t id);

/**
 * Count the entries whose bounds intersect `aabb`.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `crb_tree_create`.
 */
uint32_t crb_tree_query_aabb_count(const struct CRbTreeHandle *tree, AabbDesc aabb);

/**
 * Write the ids of entries whose bounds intersect `aabb` into `out_ids`.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `crb_tree_create`, and `out_ids`
 * must point to a writable buffer of at least `capacity` `u64` elements.
 */
uint32_t crb_tree_query_aabb(const struct CRbTreeHandle *tree,
                             AabbDesc aabb,
                             uint64_t *out_ids,
                             uint32_t capacity);

/**
 * Set the cross-validation gravity law on the world.  Any previous
 * cross-validation law (registered under `ForceLawType::NewtonianGravity`)
 * is removed first (singleton semantics, mirroring
 * `world_set_newton_gravity_law`).
 */
Bool world_set_cross_validate_gravity(struct WorldHandle *world,
                                      struct CrossValidateGravityConfig config);

/**
 * `u8`-returning variant for environments that prefer integer returns.
 */
uint8_t world_set_cross_validate_gravity_flag(struct WorldHandle *world,
                                              struct CrossValidateGravityConfig config);

/**
 * Clear the cross-validation law from the world's registry.
 */
void world_clear_cross_validate_gravity(struct WorldHandle *world);

/**
 * Read the last frame's cross-validation divergence pair count.
 *
 * Returns the number of (body, line_a, line_b) triples whose relative
 * difference exceeded `tolerance` in the most recent `apply()` invocation.
 * Returns 0 if the law is not registered, no `step` has run, or all
 * lines were within tolerance.
 */
uint64_t world_get_cross_validate_last_divergence(const struct WorldHandle *world);

/**
 * Configuration: convenience FFI building a default Earth-ish config in one
 * call so a Java caller does not need to populate every field by hand.
 */
struct CrossValidateGravityConfig world_cross_validate_default_config(void);

/**
 * Create a k-DOP collider builder from a point cloud.
 *
 * # Safety
 *
 * `points_xyz` must point to at least 3×point_count readable f64s. The
 * returned builder handle is owned by the caller and must be released
 * through the collider-builder destroy function.
 */
struct ColliderBuilderHandle *collider_builder_create_kdop(const double *points_xyz,
                                                           uint32_t point_count,
                                                           uint32_t preset);

/**
 * Create a fixed-directions-hull (FDH) collider builder from a point cloud.
 *
 * # Safety
 *
 * `points_xyz` must point to at least 3×point_count readable f64s and
 * `directions_xyz` to at least 3×direction_count readable f64s. The returned
 * builder handle is owned by the caller and must be released through the
 * collider-builder destroy function.
 */
struct ColliderBuilderHandle *collider_builder_create_fdh(const double *points_xyz,
                                                          uint32_t point_count,
                                                          const double *directions_xyz,
                                                          uint32_t direction_count);

/**
 * Create a sensor trigger zone from a shape descriptor. The sensor collider is
 * built with `sensor(true)` and `ActiveEvents::COLLISION_EVENTS` so rapier tracks
 * its intersections, then inserted into the world at `translation`.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `shape` must be a
 * valid [`ShapeDesc`] (finite params).
 */
uint32_t sensor_zone_create(struct WorldHandle *world, ShapeDesc shape, Vec3 translation);

/**
 * Change a sensor zone's shape after creation. The old sensor collider is
 * removed from the world and a new one built from `shape` is inserted at the
 * zone's current position. Useful for Minecraft-style trigger volumes that grow
 * or shrink as the game state changes.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `shape` must be a
 * valid [`ShapeDesc`] (finite params).
 */
Bool sensor_zone_set_shape(struct WorldHandle *world, uint32_t id, ShapeDesc shape);

/**
 * Disable or (re-)enable a sensor zone. A disabled zone is skipped by `poll`.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool sensor_zone_set_enabled(struct WorldHandle *world, uint32_t id, Bool enabled);

/**
 * Switch a sensor zone between level triggering (sticky: `is_triggered` stays
 * TRUE while anything overlaps) and rising-edge triggering (`is_triggered` is
 * TRUE only on the poll where an overlap first appears, then FALSE until the
 * zone is empty and re-entered). Edge mode is what you want for one-shot
 * "player entered the room" events.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool sensor_zone_set_edge(struct WorldHandle *world, uint32_t id, Bool edge);

/**
 * Recompute the set of colliders currently overlapping this sensor zone.
 *
 * Returns `Bool::TRUE` on success. After a successful poll, use
 * [`sensor_zone_contact_count`] / [`sensor_zone_get_contacts`] to read the
 * overlaps, or [`sensor_zone_is_triggered`] for the sticky "ever triggered" flag.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool sensor_zone_poll(struct WorldHandle *world, uint32_t id);

/**
 * Number of colliders currently overlapping the zone (last [`sensor_zone_poll`]).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
uint32_t sensor_zone_contact_count(const struct WorldHandle *world, uint32_t id);

/**
 * Write up to `max_count` overlapping collider handles into `out` (packed
 * [`ColliderHandleRaw`]). Returns the number actually written.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `out` may be null
 * (then only the count is returned).
 */
uint32_t sensor_zone_get_contacts(const struct WorldHandle *world,
                                  uint32_t id,
                                  ColliderHandleRaw *out,
                                  uint32_t max_count);

/**
 * `Bool::TRUE` if the zone has ever overlapped anything since creation (sticky).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool sensor_zone_is_triggered(const struct WorldHandle *world, uint32_t id);

/**
 * Read-and-clear the zone's sticky edge latch. Returns `Bool::TRUE` if a rising
 * edge had been observed since the last consume/clear, then resets the latch to
 * `FALSE`. In edge mode this is the reliable way to handle a one-shot trigger:
 * call `poll` (or `world_step` + `poll`) then `consume` exactly once per event,
 * so a single entry is never handled twice. No fork changes.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool sensor_zone_consume(struct WorldHandle *world, uint32_t id);

/**
 * Reset the zone's trigger state: clears the sticky edge latch and the
 * `ever_triggered` sticky flag. The current overlaps are left as-is (until the
 * next `poll`). Use this to re-arm a zone after handling an event, or to forget a
 * previous entry. No fork changes.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool sensor_zone_clear(struct WorldHandle *world, uint32_t id);

/**
 * Read the zone's world-space translation (its sensor collider pose).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `out` may be null.
 */
Bool sensor_zone_get_translation(const struct WorldHandle *world, uint32_t id, Vec3 *out);

/**
 * Move the sensor collider (call before [`sensor_zone_poll`] to re-evaluate at a
 * new position without recreating the zone).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool sensor_zone_set_translation(struct WorldHandle *world, uint32_t id, Vec3 translation);

/**
 * Destroy a sensor zone and remove its collider from the world.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool sensor_zone_destroy(struct WorldHandle *world, uint32_t id);

/**
 * Create a vehicle controller around a dynamic chassis built from `shape` at
 * `translation`. Returns a stable id, or `u32::MAX` on error.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `shape` must be a
 * valid [`ShapeDesc`] (finite params).
 */
uint32_t vehicle_controller_create(struct WorldHandle *world, ShapeDesc shape, Vec3 translation);

/**
 * Change a vehicle's chassis collision shape after creation. The existing
 * chassis collider is removed and a new one built from `shape` is parented to
 * the same dynamic chassis body (wheels/suspension are untouched). Useful for
 * swapping the chassis hitbox (e.g. a Minecraft minecart vs. a boat).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `shape` must be a
 * valid [`ShapeDesc`] (finite params).
 */
Bool vehicle_controller_set_shape(struct WorldHandle *world, uint32_t id, ShapeDesc shape);

/**
 * Add a wheel to the vehicle. All vectors are in the chassis' local space.
 *
 * - `chassis_connection_cs`: point on the chassis where the suspension attaches.
 * - `direction_cs`: suspension direction (e.g. `-Y` to point down).
 * - `axle_cs`: wheel axle direction (e.g. `-Z` or `+X`).
 * - `suspension_rest_length`: natural suspension length.
 * - `radius`: wheel radius.
 * - `suspension_stiffness`, `suspension_compression`, `suspension_damping`,
 *   `friction_slip`, `max_suspension_travel`, `max_suspension_force`,
 *   `side_friction_stiffness`: tuning (see rapier `WheelTuning`).
 *
 * Returns the wheel index, or `u32::MAX` on error.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; vectors must be finite.
 */
uint32_t vehicle_controller_add_wheel(struct WorldHandle *world,
                                      uint32_t id,
                                      Vec3 chassis_connection_cs,
                                      Vec3 direction_cs,
                                      Vec3 axle_cs,
                                      double suspension_rest_length,
                                      double radius,
                                      double suspension_stiffness,
                                      double suspension_compression,
                                      double suspension_damping,
                                      double friction_slip,
                                      double max_suspension_travel,
                                      double max_suspension_force,
                                      double side_friction_stiffness);

/**
 * Set the engine force (drive torque) on a wheel.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool vehicle_controller_set_engine_force(struct WorldHandle *world,
                                         uint32_t id,
                                         uint32_t wheel_index,
                                         double force);

/**
 * Set the brake force on a wheel.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool vehicle_controller_set_brake(struct WorldHandle *world,
                                  uint32_t id,
                                  uint32_t wheel_index,
                                  double brake);

/**
 * Set the steering angle (radians) on a wheel.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool vehicle_controller_set_steering(struct WorldHandle *world,
                                     uint32_t id,
                                     uint32_t wheel_index,
                                     double steering);

/**
 * Advance the vehicle physics by `dt`: build a `QueryPipelineMut` and let rapier
 * apply suspension/engine/brake impulses to the chassis. Call **after**
 * `world_step`.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool vehicle_controller_update(struct WorldHandle *world, uint32_t id, double dt);

/**
 * Read the chassis world-space translation.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `out` may be null.
 */
Bool vehicle_controller_get_translation(const struct WorldHandle *world, uint32_t id, Vec3 *out);

/**
 * Read the chassis world-space linear velocity.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `out` may be null.
 */
Bool vehicle_controller_get_velocity(const struct WorldHandle *world, uint32_t id, Vec3 *out);

/**
 * Read a wheel's suspension contact state (is the wheel touching the ground?).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool vehicle_controller_wheel_on_ground(const struct WorldHandle *world,
                                        uint32_t id,
                                        uint32_t wheel_index);

/**
 * Read a wheel's contact normal (world space).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `out` may be null.
 */
Bool vehicle_controller_wheel_contact_normal(const struct WorldHandle *world,
                                             uint32_t id,
                                             uint32_t wheel_index,
                                             Vec3 *out);

/**
 * Destroy a vehicle controller and its chassis body + collider.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool vehicle_controller_destroy(struct WorldHandle *world, uint32_t id);

/**
 * Current thread's last error code (`ERR_OK` when no error).
 *
 * # Safety
 *
 * No pointer parameters; safe to call from any thread. The error slot is
 * thread-local, so the result reflects only errors reported on the calling
 * thread.
 */
uint32_t last_error_code(void);

/**
 * Current thread's last error message ("ok" when no error).
 *
 * The returned pointer is borrowed from a thread-local slot owned by Rust;
 * it is invalidated by the next error-reporting call on the same thread and
 * must not be freed or stored.
 *
 * # Safety
 *
 * No pointer parameters; safe to call from any thread. The returned pointer
 * is borrowed from a thread-local slot owned by Rust (no ownership transfer):
 * it remains valid only until the next error-reporting call on the same
 * thread and must not be freed by the caller.
 */
const char *last_error_message(void);

/**
 * Reset the current thread's error slot to `ERR_OK` / "ok".
 *
 * # Safety
 *
 * No pointer parameters; safe to call from any thread. Only the calling
 * thread's error slot is affected.
 */
void last_error_clear(void);

/**
 * Static name of an error code ("ERR_OK", "ERR_NULL_POINTER", ...).
 *
 * Unknown codes yield "ERR_UNKNOWN". The returned pointer refers to a
 * string with `'static` lifetime owned by Rust; it must not be freed.
 *
 * # Safety
 *
 * No pointer parameters; safe to call from any thread with any `code` value
 * (unknown codes return "ERR_UNKNOWN"). The returned pointer refers to a
 * `'static` string owned by Rust (no ownership transfer) and must not be
 * freed by the caller.
 */
const char *error_code_name(uint32_t code);

/**
 * Set (or disable) the Coulomb friction law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
Bool world_set_coulomb_friction_law(struct WorldHandle *world, CoulombFrictionLaw law);

/**
 * `u8`-returning variant of `world_set_coulomb_friction_law`.
 *
 * # Safety
 *
 * Same contract as `world_set_coulomb_friction_law`.
 */
uint8_t world_set_coulomb_friction_law_flag(struct WorldHandle *world, CoulombFrictionLaw law);

/**
 * Clear the Coulomb friction law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_coulomb_friction_law(struct WorldHandle *world);

/**
 * Read the current Coulomb friction law into `out_law`.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`;
 * `out_law` must point to writable memory for one `CoulombFrictionLaw`.
 * Null pointers fail with `ERR_NULL_POINTER`.
 */
Bool world_get_coulomb_friction_law(const struct WorldHandle *world, CoulombFrictionLaw *out_law);

/**
 * Set (or disable) the air drag law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
Bool world_set_air_drag_law(struct WorldHandle *world, AirDragLaw law);

/**
 * `u8`-returning variant of `world_set_air_drag_law`.
 *
 * # Safety
 *
 * Same contract as `world_set_air_drag_law`.
 */
uint8_t world_set_air_drag_law_flag(struct WorldHandle *world, AirDragLaw law);

/**
 * Clear the air drag law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_air_drag_law(struct WorldHandle *world);

/**
 * Read the current air drag law into `out_law`.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`;
 * `out_law` must point to writable memory for one `AirDragLaw`. Null
 * pointers fail with `ERR_NULL_POINTER`.
 */
Bool world_get_air_drag_law(const struct WorldHandle *world, AirDragLaw *out_law);

/**
 * Set (or disable) the external force law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
Bool world_set_external_force_law(struct WorldHandle *world, ExternalForceLaw law);

/**
 * `u8`-returning variant of `world_set_external_force_law`.
 *
 * # Safety
 *
 * Same contract as `world_set_external_force_law`.
 */
uint8_t world_set_external_force_law_flag(struct WorldHandle *world, ExternalForceLaw law);

/**
 * Clear the external force law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_external_force_law(struct WorldHandle *world);

/**
 * Read the current external force law into `out_law`.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`;
 * `out_law` must point to writable memory for one `ExternalForceLaw`. Null
 * pointers fail with `ERR_NULL_POINTER`.
 */
Bool world_get_external_force_law(const struct WorldHandle *world, ExternalForceLaw *out_law);

/**
 * Set (or disable) the Newton gravity law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
Bool world_set_newton_gravity_law(struct WorldHandle *world, NewtonGravityLaw law);

/**
 * `u8`-returning variant of `world_set_newton_gravity_law`.
 *
 * # Safety
 *
 * Same contract as `world_set_newton_gravity_law`.
 */
uint8_t world_set_newton_gravity_law_flag(struct WorldHandle *world, NewtonGravityLaw law);

/**
 * Register a polyhedron terrain-gravity law (Werner & Scheeres 1997) on the
 * world.  `vertices_xyz` is a flat `[x,y,z]` array (3·n_vertices f64),
 * `face_indices` a flat `[a,b,c]` array (3·n_faces u32), `density` the
 * constant density (kg/m³).  Replaces any prior terrain-gravity law.
 *
 * # Safety
 * `world` must be a valid world pointer; `vertices_xyz`/`face_indices` must
 * point to readable arrays of the declared sizes.
 */
Bool world_register_terrain_gravity_polyhedron(struct WorldHandle *world,
                                               const double *vertices_xyz,
                                               uint32_t n_vertices,
                                               const uint32_t *face_indices,
                                               uint32_t n_faces,
                                               double density);

/**
 * Register a DEM surface-mass-distribution terrain-gravity law (direct
 * summation) on the world.  `dem` is a flat `[nx·ny]` height map (m above the
 * reference ellipsoid); `resolution`/`reference_radius` define the grid (m);
 * `surface_density` is kg/m².  Replaces any prior terrain-gravity law.
 *
 * # Safety
 * `world` must be a valid world pointer; `dem` must point to `nx·ny` readable
 * f64s.
 */
Bool world_register_terrain_gravity_dem(struct WorldHandle *world,
                                        const double *dem,
                                        uint32_t nx,
                                        uint32_t ny,
                                        double resolution,
                                        double reference_radius,
                                        double surface_density);

/**
 * Register the built-in lunar-mascon terrain-gravity law (GRAIL-derived,
 * Plummer-softened point masses).  Replaces any prior terrain-gravity law.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool world_register_terrain_gravity_mascon(struct WorldHandle *world);

/**
 * Unregister the terrain-gravity law from the world (disables terrain
 * gravity; uniform `world.gravity` still applies if it is non-zero).
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool world_unregister_terrain_gravity(struct WorldHandle *world);

/**
 * Clear the Newton gravity law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_newton_gravity_law(struct WorldHandle *world);

/**
 * Read the current Newton gravity law into `out_law`.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`;
 * `out_law` must point to writable memory for one `NewtonGravityLaw`. Null
 * pointers fail with `ERR_NULL_POINTER`.
 */
Bool world_get_newton_gravity_law(const struct WorldHandle *world, NewtonGravityLaw *out_law);

/**
 * Read the last custom-physics report into `out_report`.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`;
 * `out_report` must point to writable memory for one `CustomPhysicsReport`.
 * Null pointers fail with `ERR_NULL_POINTER`.
 */
Bool world_get_custom_physics_report(const struct WorldHandle *world,
                                     CustomPhysicsReport *out_report);

/**
 * Clear the legacy event queues of a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_events(struct WorldHandle *world);

/**
 * Number of queued collision events (legacy Vec queue).
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer returns 0.
 */
uint32_t world_collision_event_count(const struct WorldHandle *world);

/**
 * Read one queued collision event by index (legacy Vec queue).
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer or out-of-range index returns a zeroed record.
 */
CollisionEventRecord world_get_collision_event(const struct WorldHandle *world, uint32_t index);

/**
 * Copy up to `capacity` queued collision events into `out_events`.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`;
 * `out_events` must point to writable memory for `capacity`
 * `CollisionEventRecord` elements (`0 < capacity <= MAX_OUTPUT_CAPACITY`).
 */
uint32_t world_get_collision_events(const struct WorldHandle *world,
                                    CollisionEventRecord *out_events,
                                    uint32_t capacity);

/**
 * Number of queued contact-force events (legacy Vec queue).
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer returns 0.
 */
uint32_t world_contact_force_event_count(const struct WorldHandle *world);

/**
 * Read one queued contact-force event by index (legacy Vec queue).
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer or out-of-range index returns a zeroed record.
 */
ContactForceEventRecord world_get_contact_force_event(const struct WorldHandle *world,
                                                      uint32_t index);

/**
 * Copy up to `capacity` queued contact-force events into `out_events`.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`;
 * `out_events` must point to writable memory for `capacity`
 * `ContactForceEventRecord` elements (`0 < capacity <= MAX_OUTPUT_CAPACITY`).
 */
uint32_t world_get_contact_force_events(const struct WorldHandle *world,
                                        ContactForceEventRecord *out_events,
                                        uint32_t capacity);

/**
 * Disabled external contact-pair filter callback (always reports
 * `ERR_UNSUPPORTED` and reinstalls the default hooks).
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
void world_set_contact_pair_filter_callback(struct WorldHandle *world,
                                            uintptr_t _callback,
                                            uintptr_t _user_data);

/**
 * Disabled external intersection-pair filter callback (always reports
 * `ERR_UNSUPPORTED` and reinstalls the default hooks).
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
void world_set_intersection_pair_filter_callback(struct WorldHandle *world,
                                                 uintptr_t _callback,
                                                 uintptr_t _user_data);

/**
 * Reinstall the default contact-pair filter hooks.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_contact_pair_filter_callback(struct WorldHandle *world);

/**
 * Reinstall the default intersection-pair filter hooks.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_intersection_pair_filter_callback(struct WorldHandle *world);

/**
 * Allocate a collision-event ring buffer of `capacity` records.
 * Events will be written here during `world_step` instead of (or in addition to)
 * the legacy Vec queue.  Java drains the ring buffer at its own pace.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`.
 * Init-time only: must be called before `world_step` runs on any thread and
 * with no concurrent event-ring FFI calls on the same world.  The producer
 * cache is an `UnsafeCell`; violations of this contract are caught at runtime
 * and fail with `ERR_UNSUPPORTED` (see the `events` module docs).
 */
Bool world_init_collision_event_ring(struct WorldHandle *world, uint32_t capacity);

/**
 * Allocate a contact-force-event ring buffer.
 *
 * # Safety
 *
 * Same init-time-only contract as `world_init_collision_event_ring`.
 */
Bool world_init_contact_force_event_ring(struct WorldHandle *world, uint32_t capacity);

/**
 * Drain the collision-event ring buffer into `out_events`.
 * Returns the number of events drained.  This is the **only** FFI call needed
 * per frame after init — no more count-then-allocate-then-read cycles.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`;
 * `out_events` must point to writable memory for `capacity`
 * `CollisionEventRecord` elements (`0 < capacity <= MAX_OUTPUT_CAPACITY`).
 * May run concurrently with `world_step` (SPSC drain), but only from a
 * single consumer thread.
 */
uint32_t world_drain_collision_event_ring(const struct WorldHandle *world,
                                          CollisionEventRecord *out_events,
                                          uint32_t capacity);

/**
 * Drain the contact-force-event ring buffer.
 *
 * # Safety
 *
 * Same contract as `world_drain_collision_event_ring`, with
 * `ContactForceEventRecord` output elements.
 */
uint32_t world_drain_contact_force_event_ring(const struct WorldHandle *world,
                                              ContactForceEventRecord *out_events,
                                              uint32_t capacity);

/**
 * Get the current number of events in the collision ring buffer (cheap, no lock).
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer returns 0.
 */
uint32_t world_collision_event_ring_len(const struct WorldHandle *world);

/**
 * Get the current number of events in the contact-force ring buffer.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer returns 0.
 */
uint32_t world_contact_force_event_ring_len(const struct WorldHandle *world);

/**
 * Get ring buffer statistics (capacity, occupancy, drops, wraps).
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`;
 * `out_stats` must point to writable memory for one `EventRingBufferStats`.
 * Null pointers fail with `ERR_NULL_POINTER`.
 */
Bool world_collision_event_ring_stats(const struct WorldHandle *world,
                                      EventRingBufferStats *out_stats);

/**
 * Get contact-force ring buffer statistics.
 *
 * # Safety
 *
 * Same contract as `world_collision_event_ring_stats`.
 */
Bool world_contact_force_event_ring_stats(const struct WorldHandle *world,
                                          EventRingBufferStats *out_stats);

/**
 * Clear both ring buffers and reset drop counters.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_event_rings(struct WorldHandle *world);

/**
 * Register a collision-event callback.
 *
 * `callback` is a C function pointer (zero = unregister).
 * `user_data` is passed through unchanged to each invocation.
 * Returns an opaque handle for later unregistration.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`.
 * `callback` must be `0` ("unset") or the address of a function with the
 * exact `CollisionEventFn` signature that stays valid while registered.
 * Init-time only: must be called before `world_step` runs on any thread and
 * with no concurrent event-ring/callback FFI calls on the same world.  The
 * producer cache is an `UnsafeCell`; violations of this contract are caught
 * at runtime and fail with `ERR_UNSUPPORTED` (see the `events` module docs).
 */
EventCallbackHandle world_register_collision_callback(struct WorldHandle *world,
                                                      uintptr_t callback,
                                                      uintptr_t user_data);

/**
 * Register a contact-force-event callback.
 *
 * # Safety
 *
 * Same init-time-only contract as `world_register_collision_callback`;
 * `callback` must be `0` ("unset") or the address of a function with the
 * exact `ContactForceEventFn` signature that stays valid while registered.
 */
EventCallbackHandle world_register_contact_force_callback(struct WorldHandle *world,
                                                          uintptr_t callback,
                                                          uintptr_t user_data);

/**
 * Unregister a previously registered callback by its handle.
 * Passing 0 or an invalid handle is a no-op.
 *
 * # Safety
 *
 * Same init-time-only contract as `world_register_collision_callback`.
 */
void world_unregister_callback(struct WorldHandle *world, EventCallbackHandle handle);

/**
 * Set the event dispatch mode.
 *
 * - `Poll` (0): legacy Vec queue only (default).
 * - `Callback` (1): registered callbacks only.
 * - `Both` (2): ring buffer + callbacks.
 *
 * # Safety
 *
 * Same init-time-only contract as `world_init_collision_event_ring`.
 */
Bool world_set_event_dispatch_mode(struct WorldHandle *world, uint32_t mode);

/**
 * Set (or disable) the solar-wind dynamic-pressure force law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
Bool world_set_solar_wind_pressure_law(struct WorldHandle *world, SolarWindPressureLaw law);

/**
 * `u8`-returning variant of `world_set_solar_wind_pressure_law`.
 *
 * # Safety
 *
 * Same contract as `world_set_solar_wind_pressure_law`.
 */
uint8_t world_set_solar_wind_pressure_law_flag(struct WorldHandle *world, SolarWindPressureLaw law);

/**
 * Clear the solar-wind pressure law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_solar_wind_pressure_law(struct WorldHandle *world);

/**
 * Set (or disable) the Chandrasekhar dynamical-friction force law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
Bool world_set_dynamical_friction_law(struct WorldHandle *world, DynamicalFrictionLaw law);

/**
 * `u8`-returning variant of `world_set_dynamical_friction_law`.
 *
 * # Safety
 *
 * Same contract as `world_set_dynamical_friction_law`.
 */
uint8_t world_set_dynamical_friction_law_flag(struct WorldHandle *world, DynamicalFrictionLaw law);

/**
 * Clear the dynamical-friction law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_dynamical_friction_law(struct WorldHandle *world);

/**
 * Set (or disable) the MOND-corrected gravity force law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
Bool world_set_mond_gravity_law(struct WorldHandle *world, MonDGravityLaw law);

/**
 * `u8`-returning variant of `world_set_mond_gravity_law`.
 *
 * # Safety
 *
 * Same contract as `world_set_mond_gravity_law`.
 */
uint8_t world_set_mond_gravity_law_flag(struct WorldHandle *world, MonDGravityLaw law);

/**
 * Clear the MOND gravity law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_mond_gravity_law(struct WorldHandle *world);

/**
 * Set (or disable) the Eddington-limited radiation-pressure force law on
 * a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
Bool world_set_eddington_radiation_pressure_law(struct WorldHandle *world,
                                                EddingtonRadiationPressureLaw law);

/**
 * `u8`-returning variant of `world_set_eddington_radiation_pressure_law`.
 *
 * # Safety
 *
 * Same contract as `world_set_eddington_radiation_pressure_law`.
 */
uint8_t world_set_eddington_radiation_pressure_law_flag(struct WorldHandle *world,
                                                        EddingtonRadiationPressureLaw law);

/**
 * Clear the Eddington radiation-pressure law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_eddington_radiation_pressure_law(struct WorldHandle *world);

/**
 * Set (or disable) the X-ray disc bolometric irradiation force law on a
 * world.  See `XrayIrradiationLaw` doc for parameter semantics.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
Bool world_set_xray_irradiation_law(struct WorldHandle *world, XrayIrradiationLaw law);

/**
 * `u8`-returning variant of `world_set_xray_irradiation_law`.
 *
 * # Safety
 *
 * Same contract as `world_set_xray_irradiation_law`.
 */
uint8_t world_set_xray_irradiation_law_flag(struct WorldHandle *world, XrayIrradiationLaw law);

/**
 * Clear the X-ray irradiation law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_xray_irradiation_law(struct WorldHandle *world);

/**
 * Set (or disable) the pulsar magnetic-dipole torque law on a world.
 * See `PulsarMagneticDipoleLaw` doc for parameter semantics.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
Bool world_set_pulsar_magnetic_dipole_law(struct WorldHandle *world, PulsarMagneticDipoleLaw law);

/**
 * `u8`-returning variant of `world_set_pulsar_magnetic_dipole_law`.
 *
 * # Safety
 *
 * Same contract as `world_set_pulsar_magnetic_dipole_law`.
 */
uint8_t world_set_pulsar_magnetic_dipole_law_flag(struct WorldHandle *world,
                                                  PulsarMagneticDipoleLaw law);

/**
 * Clear the pulsar magnetic-dipole torque law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_pulsar_magnetic_dipole_law(struct WorldHandle *world);

/**
 * Set (or disable) the Jeans-escape drag force law on a world.
 * See `JeansEscapeLaw` doc for parameter semantics.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer fails with `ERR_NULL_POINTER`.
 */
Bool world_set_jeans_escape_law(struct WorldHandle *world, JeansEscapeLaw law);

/**
 * `u8`-returning variant of `world_set_jeans_escape_law`.
 *
 * # Safety
 *
 * Same contract as `world_set_jeans_escape_law`.
 */
uint8_t world_set_jeans_escape_law_flag(struct WorldHandle *world, JeansEscapeLaw law);

/**
 * Clear the Jeans-escape drag law on a world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer returned by `world_create`; a null
 * pointer is a no-op.
 */
void world_clear_jeans_escape_law(struct WorldHandle *world);

/**
 * Consumes all active slots in the force queue and applies forces to Rapier bodies.
 *
 * # Safety
 * - `world` must be a valid `WorldHandle` from `rigid_body_world_create`.
 * - `queue` must point to a valid `ForceQueueHeader` allocated by Java with
 *   matching `capacity`, `stride`, and sufficient trailing memory for bitmap + payload.
 * - Java must be the sole producer; Rust (this call) is the sole consumer.
 * - The queue memory must remain valid for the duration of this call.
 */
uint32_t rigid_body_consume_force_queue(struct WorldHandle *world, struct ForceQueueHeader *queue);

/**
 * # Safety
 *
 * `out_report` may be null or must point to writable space for one
 * `FluidForceReport`.
 */
Bool fluid_estimate_aabb_forces(FluidVolume fluid,
                                Vec3 body_center,
                                Vec3 body_half_extents,
                                double body_volume,
                                Vec3 body_linvel,
                                Vec3 body_angvel,
                                FluidForceReport *out_report);

/**
 * # Safety
 *
 * `world` must be a valid world handle; `out_report` may be null or must
 * point to writable space for one `FluidForceReport`.
 */
Bool fluid_apply_aabb_forces(struct WorldHandle *world,
                             RigidBodyHandleRaw body_handle,
                             FluidVolume fluid,
                             Vec3 body_half_extents,
                             double body_volume,
                             Bool wake_up,
                             FluidForceReport *out_report);

/**
 * # Safety
 *
 * `world` must be a valid world handle; `out_report` may be null or must
 * point to writable space for one `FluidForceReport`.
 */
uint8_t fluid_apply_aabb_forces_flag(struct WorldHandle *world,
                                     RigidBodyHandleRaw body_handle,
                                     FluidVolume fluid,
                                     Vec3 body_half_extents,
                                     double body_volume,
                                     Bool wake_up,
                                     FluidForceReport *out_report);

/**
 * # Safety
 *
 * `out_report` must point to writable space for one `NavierStokesReport`.
 */
Bool fluid_navier_stokes_simplified_step(Vec3 velocity,
                                         Vec3 advection,
                                         Vec3 pressure_gradient,
                                         Vec3 laplacian_velocity,
                                         Vec3 external_acceleration,
                                         double density,
                                         double kinematic_viscosity,
                                         double dt,
                                         NavierStokesReport *out_report);

/**
 * Evaluates the SPH poly6 kernel for a distance and smoothing radius.
 *
 * # Safety
 *
 * This function takes no pointers; all inputs are passed by value and there
 * are no safety requirements on the caller.
 */
double fluid_sph_poly6_kernel(double distance, double smoothing_radius);

/**
 * # Safety
 *
 * `out_gradient` must point to writable space for one `Vec3`.
 */
Bool fluid_sph_spiky_gradient(Vec3 offset, double smoothing_radius, Vec3 *out_gradient);

/**
 * Evaluates the Laplacian of the SPH viscosity kernel for a distance and
 * smoothing radius.
 *
 * # Safety
 *
 * This function takes no pointers; all inputs are passed by value and there
 * are no safety requirements on the caller.
 */
double fluid_sph_viscosity_laplacian(double distance, double smoothing_radius);

/**
 * # Safety
 *
 * `particles` must point to `particle_count` `SphParticle` values (or be
 * null when `particle_count` is 0); `out_density` must point to writable
 * space for one `f64`.
 */
Bool fluid_sph_estimate_density(Vec3 position,
                                const SphParticle *particles,
                                uint32_t particle_count,
                                double smoothing_radius,
                                double *out_density);

/**
 * # Safety
 *
 * `particles` must point to `particle_count` `SphParticle` values (or be
 * null when `particle_count` is 0); `out_report` must point to writable
 * space for one `SphForceReport`.
 */
Bool fluid_sph_estimate_forces(SphParticle particle,
                               const SphParticle *particles,
                               uint32_t particle_count,
                               double smoothing_radius,
                               double gas_constant,
                               double rest_density,
                               double viscosity,
                               double surface_tension,
                               SphForceReport *out_report);

/**
 * Computes the static pressure from a Bernoulli-equation total pressure.
 *
 * # Safety
 *
 * This function takes no pointers; all inputs are passed by value and there
 * are no safety requirements on the caller.
 */
double fluid_bernoulli_pressure(double total_pressure,
                                double density,
                                double velocity,
                                double gravity,
                                double elevation);

/**
 * # Safety
 *
 * `out_report` must point to writable space for one `BernoulliReport`.
 */
Bool fluid_bernoulli_report(double pressure,
                            double density,
                            double velocity,
                            double gravity,
                            double elevation,
                            BernoulliReport *out_report);

/**
 * Create an SPH fluid world and return its id (the `Vec` index in
 * `PhysicsWorld.fluids`). Returns `u32::MAX` on error.
 *
 * * `gravity_x/y/z` — constant body acceleration (finite).
 * * `smoothing_radius` — SPH kernel cutoff `h` (`> 0`).
 * * `gas_constant` — equation-of-state stiffness `k` (`>= 0`, finite).
 * * `rest_density` — target density `ρ₀` (`> 0`).
 * * `viscosity` — dynamic viscosity `μ` (`>= 0`).
 * * `surface_tension` — cohesion coefficient `σ` (`>= 0`).
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
uint32_t fluid_create(struct WorldHandle *world,
                      double gravity_x,
                      double gravity_y,
                      double gravity_z,
                      double smoothing_radius,
                      double gas_constant,
                      double rest_density,
                      double viscosity,
                      double surface_tension);

/**
 * Append a particle to a fluid and return its particle index (`u32::MAX` on
 * error). `mass` must be `> 0`.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
uint32_t fluid_add_particle(struct WorldHandle *world,
                            uint32_t id,
                            double x,
                            double y,
                            double z,
                            double vx,
                            double vy,
                            double vz,
                            double mass);

/**
 * Number of particles in a fluid (`u32::MAX` for an unknown id).
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
uint32_t fluid_particle_count(const struct WorldHandle *world, uint32_t id);

/**
 * Read a particle's position/velocity/density into the out pointers (any of
 * which may be null to skip). Returns `Bool::TRUE` on success.
 *
 * # Safety
 * `world` must be a valid world pointer; `out_*` must be null or point to
 * writable `Vec3` / `f64` space.
 */
Bool fluid_get_particle(const struct WorldHandle *world,
                        uint32_t id,
                        uint32_t index,
                        Vec3 *out_pos,
                        Vec3 *out_vel,
                        double *out_density);

/**
 * Advance a fluid by `dt` seconds (`> 0`). Returns `Bool::TRUE` on success.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool fluid_step(struct WorldHandle *world, uint32_t id, double dt);

/**
 * Enable or disable rigid-body collision coupling for an SPH fluid.
 *
 * When `enabled` is `Bool::TRUE`, one dynamic `Ball` collider (radius
 * `particle_radius`) is created per particle and registered in the world's
 * collision-proxy table (`fluid_proxies`); `world_step` then syncs particle
 * poses into these proxies before the rigid step and reads the contacted poses
 * back afterwards, so the fluid is blocked/stacked by terrain and other rigid
 * bodies (and by its own particles, maintaining incompressibility). When
 * `Bool::FALSE`, any existing proxies are removed.
 *
 * Unlike soft-body proxies, fluid proxies keep the default (all-groups) collision
 * filter so particles collide with each other and with rigid bodies.
 *
 * Returns `Bool::TRUE` on success.
 *
 * # Safety
 * `world` must be a valid world pointer returned by `world_create`.
 */
Bool fluid_enable_collision(struct WorldHandle *world,
                            uint32_t id,
                            double particle_radius,
                            Bool enabled);

/**
 * # Safety
 *
 * `out_report` must point to writable space for one `StressIntensityReport`.
 */
Bool fracture_stress_intensity_factor(double stress,
                                      double crack_length,
                                      double geometry_factor,
                                      double fracture_toughness,
                                      StressIntensityReport *out_report);

/**
 * # Safety
 *
 * `out_report` must point to writable space for one `GriffithReport`.
 */
Bool fracture_griffith_criterion(double stress,
                                 double crack_length,
                                 FractureMaterial material,
                                 GriffithReport *out_report);

/**
 * # Safety
 *
 * `cycle_counts` and `cycles_to_failure` must each point to `count` `f64`
 * values; `out_report` must point to writable space for one
 * `MinerDamageReport`.
 */
Bool fracture_miner_damage(const double *cycle_counts,
                           const double *cycles_to_failure,
                           uint32_t count,
                           MinerDamageReport *out_report);

/**
 * # Safety
 *
 * `out_report` must point to writable space for one `SnCurveReport`.
 */
Bool fracture_sn_curve_life(double stress_amplitude,
                            double coefficient,
                            double exponent,
                            double endurance_limit,
                            SnCurveReport *out_report);

/**
 * # Safety
 *
 * `out_report` must point to writable space for one `FractureEnergyReport`.
 */
Bool fracture_energy_release(double strain_energy,
                             double new_surface_area,
                             double surface_energy,
                             double kinetic_energy,
                             FractureEnergyReport *out_report);

/**
 * # Safety
 *
 * `out_report` must point to writable space for one `FractureModeReport`.
 */
Bool fracture_mode_from_stress(double tensile_stress,
                               double shear_stress,
                               double compressive_stress,
                               FractureModeReport *out_report);

/**
 * # Safety
 *
 * `world` must be a valid world handle; `fragments` must point to
 * `fragment_count` `FractureFragmentDesc` values; `out_body_handles` must
 * point to writable space for `capacity` body handles; `out_joint_handles`
 * must point to writable space for `capacity` joint handles when
 * `connect_fragments` is non-zero; `out_report` may be null or must point
 * to writable space for one `FractureReplaceReport`.
 */
Bool world_replace_body_with_fracture_fragments(struct WorldHandle *world,
                                                RigidBodyHandleRaw source_body,
                                                const FractureFragmentDesc *fragments,
                                                uint32_t fragment_count,
                                                Bool connect_fragments,
                                                Bool remove_source,
                                                RigidBodyHandleRaw *out_body_handles,
                                                ImpulseJointHandleRaw *out_joint_handles,
                                                uint32_t capacity,
                                                FractureReplaceReport *out_report);

/**
 * Create a fracture mesh body from a rigid body and pre-defined fragments.
 *
 * The body is inserted into the world as a normal rigid body; the fragments
 * are stored for later use when fracture is triggered. Returns a stable id,
 * or `u32::MAX` on error.
 *
 * # Safety
 *
 * `world` must be a valid world pointer. `fragments` must point to
 * `fragment_count` valid descriptors.
 */
uint32_t fracture_mesh_body_create(struct WorldHandle *world,
                                   ShapeDesc shape,
                                   Vec3 translation,
                                   const FractureFragmentDesc *fragments,
                                   uint32_t fragment_count,
                                   FractureMaterial material,
                                   Bool connect_fragments);

/**
 * Create a fracture mesh body whose fragments are generated by Voronoi
 * pre-splitting.
 *
 * Instead of requiring hand-authored fragment descriptors, the caller
 * supplies an AABB (in the body's local space) and a set of seed points; the
 * Voronoi cell of each seed (clipped to the AABB, bisected against every
 * other seed) is box-fitted into a `FractureFragmentDesc`. `edge_shrink` is
 * a fraction in `[0.0, 0.5)` removed from each side of every fragment's
 * half-extents so adjacent fragments start with a gap instead of
 * interpenetrating (0.0 keeps the exact cell AABB). Fragment
 * `initial_velocity` starts at zero (inherited from the source body at
 * trigger time) and `density` at 0 (inherited from the material). Duplicate
 * seeds are merged and degenerate cells skipped; at least one valid cell is
 * required. Returns a stable id, or `u32::MAX` on error.
 *
 * # Safety
 *
 * `world` must be a valid world pointer. `seeds` must point to
 * `seed_count` valid `Vec3`s.
 */
uint32_t fracture_mesh_body_create_with_voronoi(struct WorldHandle *world,
                                                ShapeDesc shape,
                                                Vec3 translation,
                                                Vec3 aabb_min,
                                                Vec3 aabb_max,
                                                const Vec3 *seeds,
                                                uint32_t seed_count,
                                                FractureMaterial material,
                                                Bool connect_fragments,
                                                double edge_shrink);

/**
 * Manually trigger fracture for a fracture mesh body.
 *
 * The original body is replaced by its pre-defined fragments. Returns `true`
 * on success, `false` on error.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool fracture_mesh_body_trigger(struct WorldHandle *world, uint32_t id);

/**
 * Set the fracture trigger mode for a fracture mesh body.
 *
 * Trigger modes: `0` = manual (`fracture_mesh_body_trigger` only), `1` =
 * stress intensity (auto-fractures when `fracture_mesh_body_set_stress`
 * reports stress ≥ `threshold`), `2` = Griffith (energy criterion, same
 * threshold form), `3` = fatigue (auto-fractures once accumulated fatigue
 * damage reaches 1.0).
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool fracture_mesh_body_set_trigger(struct WorldHandle *world,
                                    uint32_t id,
                                    uint32_t mode,
                                    double threshold);

/**
 * Set the fracture trigger mode to stress intensity (convenience wrapper
 * around `fracture_mesh_body_set_trigger` with mode `1`).
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool fracture_mesh_body_set_trigger_stress(struct WorldHandle *world,
                                           uint32_t id,
                                           double threshold);

/**
 * Report the current stress intensity for a fracture mesh body.
 *
 * Stores the value (readable for diagnostics via the trigger state) and
 * auto-fractures the body when the trigger mode is `StressIntensity` or
 * `Griffith` and the reported stress reaches the configured threshold.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool fracture_mesh_body_set_stress(struct WorldHandle *world, uint32_t id, double stress);

/**
 * Update fatigue damage for a fracture mesh body.
 *
 * Accumulates fatigue damage; when damage reaches 1.0, the body fractures
 * automatically if the trigger mode is `Fatigue`.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool fracture_mesh_body_add_fatigue_damage(struct WorldHandle *world, uint32_t id, double damage);

/**
 * Check if a fracture mesh body has fractured.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool fracture_mesh_body_is_fractured(struct WorldHandle *world, uint32_t id);

/**
 * Remove a fracture mesh body from the world.
 *
 * If the body has not yet fractured, removes the original rigid body.
 * If already fractured, this is a no-op (fragments are independent bodies).
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool fracture_mesh_body_remove(struct WorldHandle *world, uint32_t id);

/**
 * Enable automatic impact damage for a fracture mesh body.
 *
 * From then on, every `world_step` accumulates the solver contact impulse
 * (N·s) this body exchanges through any of its colliders, scaled by
 * `scale`, into the body's impact damage; once the accumulated damage
 * reaches `threshold` the body auto-fractures (same path as the manual
 * trigger: source body replaced by its fragment set). Disabling after the
 * fact is not supported — pass a huge `threshold` to effectively neutralize.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool fracture_mesh_body_enable_impact_damage(struct WorldHandle *world,
                                             uint32_t id,
                                             double scale,
                                             double threshold);

/**
 * Read the accumulated impact damage of a fracture mesh body.
 *
 * Writes the current value to `out_damage` (always allowed, even after the
 * body has fractured — the value then stays at its trigger-time level).
 *
 * # Safety
 *
 * `world` must be a valid world pointer; `out_damage` must point to
 * writable memory for one `f64`.
 */
Bool fracture_mesh_body_get_impact_damage(struct WorldHandle *world,
                                          uint32_t id,
                                          double *out_damage);

/**
 * Link (or unlink) a fracture mesh body's debris routing to a granular body.
 *
 * Once linked, triggering the fracture (manually or via any auto trigger)
 * turns every fragment whose largest half-extent is below
 * `size_threshold` into one DEM grain spawned at the fragment's world
 * centre with the source body's linear velocity; fragments at or above the
 * threshold keep becoming rigid fragment bodies. Pass `granular_id ==
 * u32::MAX` to unlink (the remaining parameters are ignored).
 *
 * Grain mass/radius are caller-chosen (the link does not derive them from
 * the fragment volume); grain spawn is best-effort — a link naming a
 * granular body destroyed before the trigger silently falls back to rigid
 * fragments for those pieces.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool fracture_mesh_body_link_granular_debris(struct WorldHandle *world,
                                             uint32_t id,
                                             uint32_t granular_id,
                                             double size_threshold,
                                             double grain_mass,
                                             double grain_radius);

/**
 * Create a DEM granular body and return its id (the `Vec` index in
 * `PhysicsWorld.granular_bodies`). Returns `u32::MAX` on error.
 *
 * `gravity` is the body acceleration for every particle (typically the
 * world's gravity, so a granular pile falls like everything else).
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null.
 */
uint32_t granular_create(struct WorldHandle *world,
                         Vec3 gravity,
                         double particle_radius,
                         double normal_stiffness,
                         double normal_damping,
                         double friction,
                         double tangential_damping);

/**
 * Append a particle to a granular body. Returns the particle index or
 * `u32::MAX` on error.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null.
 */
uint32_t granular_add_particle(struct WorldHandle *world,
                               uint32_t id,
                               double x,
                               double y,
                               double z,
                               double vx,
                               double vy,
                               double vz,
                               double mass,
                               double radius);

/**
 * Number of particles in a granular body. Returns `u32::MAX` for an unknown
 * id.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null.
 */
uint32_t granular_particle_count(const struct WorldHandle *world, uint32_t id);

/**
 * Batch-read granular particle positions + velocities into `out_pos` /
 * `out_vel` (each with `capacity` slots). Either out-pointer may be null to
 * skip that channel. Returns the real particle count (callers retry with a
 * bigger buffer when `capacity` is short). Null world / unknown id → `0`.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null; `out_pos` / `out_vel` must
 * be null or point to writable memory for `capacity` values each.
 */
uint32_t granular_read_particles(const struct WorldHandle *world,
                                 uint32_t id,
                                 Vec3 *out_pos,
                                 Vec3 *out_vel,
                                 uint32_t capacity);

/**
 * Manually advance one granular body by `dt`. `world_step` already ticks
 * every granular body — this is for callers that want a custom substep loop.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null.
 */
Bool granular_step(struct WorldHandle *world, uint32_t id, double dt);

/**
 * Link voxel digging to grain spawning: from now on, digging a solid cell
 * out of any voxel collider (`collider_voxel_edit` with `solid = 0`, or a
 * `soft_body_voxel_dig` that propagates to the collider grid) spawns one
 * grain of `grain_mass` / `grain_radius` at the cell's world centre into the
 * granular body `dig_grain_body`. Pass `dig_grain_body = u32::MAX` to unlink.
 *
 * Returns `Bool::FALSE` (and changes nothing) when `dig_grain_body` is not
 * `u32::MAX` and does not name an existing granular body.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null.
 */
Bool granular_link_voxel_dig(struct WorldHandle *world,
                             uint32_t dig_grain_body,
                             double grain_mass,
                             double grain_radius);

/**
 * Query the current voxel-dig → grain-spawn link. Returns `Bool::TRUE` when
 * linked (writing the body id / mass / radius through the non-null out
 * pointers), `Bool::FALSE` when unlinked or the world is null.
 *
 * # Safety
 *
 * All out pointers may be null; `world` must be a valid world pointer or null.
 */
Bool granular_get_voxel_dig_link(const struct WorldHandle *world,
                                 uint32_t *out_body,
                                 double *out_mass,
                                 double *out_radius);

/**
 * Enable/disable rigid-body collision coupling for a granular body (Phase 38,
 * mirroring `fluid_enable_collision`): when enabled, every particle gets a
 * dynamic proxy `RigidBody` (gravity_scale 0 — the DEM integrator applies
 * gravity itself) plus a `Ball` collider of `particle_radius`. `world_step`
 * then syncs particle poses into the proxies before the rigid step, reads the
 * contacted poses back after it, and only then runs the DEM integrator — so
 * grains pile up on voxel terrain / rigid bodies instead of falling through.
 *
 * Enabling again re-syncs the proxy set to the current particle count (new
 * particles get proxies). Disabling destroys the proxies; the particles keep
 * their last synced poses.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null.
 */
Bool granular_enable_collision(struct WorldHandle *world,
                               uint32_t id,
                               double particle_radius,
                               Bool enabled);

/**
 * Create a hair system attached to a rigid body.
 *
 * Returns a stable id, or `u32::MAX` on error.
 *
 * # Safety
 *
 * `world` must be a valid world pointer. `strands` must point to
 * `strand_count` valid descriptors.
 */
uint32_t hair_system_create(struct WorldHandle *world,
                            RigidBodyHandleRaw attached_body,
                            const struct HairStrandDesc *strands,
                            uint32_t strand_count);

/**
 * Build the hair strands (creates the actual soft bodies).
 *
 * This is called after `hair_system_create` to instantiate the hair geometry.
 * Returns `true` on success.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool hair_system_build(struct WorldHandle *world, uint32_t id);

/**
 * Set wind force for a hair system.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool hair_system_set_wind(struct WorldHandle *world, uint32_t id, Vec3 wind);

/**
 * Set gravity scale for a hair system.
 *
 * `scale = 0.0` disables gravity for hair (e.g., underwater hair).
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool hair_system_set_gravity_scale(struct WorldHandle *world, uint32_t id, double scale);

/**
 * Remove a hair system from the world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool hair_system_remove(struct WorldHandle *world, uint32_t id);

/**
 * Query the soft-body id backing a hair strand (for particle read-out, e.g.
 * rendering). Only valid after `hair_system_build`.
 *
 * Returns the `SoftBodyId.0`, or `u32::MAX` on error.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
uint32_t hair_system_strand_soft_body(struct WorldHandle *world,
                                      uint32_t id,
                                      uint32_t strand_index);

/**
 * Create a rope body along the straight span `start → end`.
 *
 * Returns the new `SoftBodyId` (as `u32`), or `u32::MAX` with the
 * thread-local error slot set to an `ERR_*` code on failure. The rope
 * integrates automatically in `world_step` — no separate stepping call is
 * needed.
 *
 * # Safety
 *
 * `world` must be a valid world pointer or null (null reports
 * `ERR_NULL_POINTER` and returns `u32::MAX`). No other pointers are
 * dereferenced; `desc` is passed by value.
 */
uint32_t soft_rope_create(struct WorldHandle *world, struct RopeDesc desc);

/**
 * Create a rope knot system.
 *
 * Returns a stable id, or `u32::MAX` on error.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
uint32_t rope_knot_create(struct WorldHandle *world,
                          uint32_t pattern,
                          uint32_t strand_count,
                          const Vec3 *control_points,
                          uint32_t control_point_count,
                          double radius,
                          double stiffness,
                          double self_friction,
                          double density);

/**
 * Build the rope knot geometry (creates the per-strand soft bodies and their
 * collision proxies).
 *
 * Returns `true` on success.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool rope_knot_build(struct WorldHandle *world, uint32_t id, Vec3 start, Vec3 end);

/**
 * Set wind force for a rope knot system.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool rope_knot_set_wind(struct WorldHandle *world, uint32_t id, Vec3 wind);

/**
 * Remove a rope knot system from the world.
 *
 * Tears down the per-strand collision proxies before removing the soft
 * bodies themselves.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool rope_knot_remove(struct WorldHandle *world, uint32_t id);

/**
 * Query the soft-body id backing a knot strand (for particle read-out, e.g.
 * rendering). Braids own one soft body per strand; knots and custom patterns
 * own a single one. Only valid after `rope_knot_build`.
 *
 * Returns the `SoftBodyId.0`, or `u32::MAX` on error.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
uint32_t rope_knot_strand_soft_body(struct WorldHandle *world, uint32_t id, uint32_t strand_index);

/**
 * Creates a joint builder of the given type and returns an owned pointer to it.
 *
 * # Safety
 *
 * No pointers are dereferenced. The returned pointer is owned by the caller and
 * must be released with `joint_builder_destroy` (or consumed by
 * `world_insert_impulse_joint`). Invalid parameters fail with
 * `ERR_INVALID_ARGUMENT` and return null.
 */
struct JointBuilderHandle *joint_builder_create(uint32_t joint_type,
                                                Vec3 axis_or_primary,
                                                double b,
                                                double c);

/**
 * # Safety
 *
 * `builder` must be a pointer returned by `joint_builder_create` (or null, which is a
 * no-op). Ownership is transferred to Rust and the pointer must not be used after
 * this call.
 */
void joint_builder_destroy(struct JointBuilderHandle *builder);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by `joint_builder_create` and must
 * remain alive for the duration of the call.
 */
void joint_builder_set_contacts_enabled(struct JointBuilderHandle *builder, Bool enabled);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by `joint_builder_create` and must
 * remain alive for the duration of the call.
 */
void joint_builder_set_local_anchor1(struct JointBuilderHandle *builder, Vec3 anchor);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by `joint_builder_create` and must
 * remain alive for the duration of the call.
 */
void joint_builder_set_local_anchor2(struct JointBuilderHandle *builder, Vec3 anchor);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by `joint_builder_create` and must
 * remain alive for the duration of the call.
 */
void joint_builder_set_limits(struct JointBuilderHandle *builder,
                              uint32_t axis,
                              double min,
                              double max);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by `joint_builder_create` and must
 * remain alive for the duration of the call.
 */
void joint_builder_set_motor_velocity(struct JointBuilderHandle *builder,
                                      uint32_t axis,
                                      double target_vel,
                                      double factor);

/**
 * # Safety
 *
 * `builder` must be a valid pointer returned by `joint_builder_create` and must
 * remain alive for the duration of the call.
 */
void joint_builder_set_motor_position(struct JointBuilderHandle *builder,
                                      uint32_t axis,
                                      double target_pos,
                                      double stiffness,
                                      double damping);

/**
 * # Safety
 *
 * `world` must be a valid world pointer. `builder` must be a pointer returned by
 * `joint_builder_create`; on success its ownership is consumed by this call and it
 * must not be used afterwards.
 */
ImpulseJointHandleRaw world_insert_impulse_joint(struct WorldHandle *world,
                                                 RigidBodyHandleRaw body1,
                                                 RigidBodyHandleRaw body2,
                                                 struct JointBuilderHandle *builder,
                                                 Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a valid world pointer and must remain alive for the duration of
 * the call.
 */
Bool world_remove_impulse_joint(struct WorldHandle *world,
                                ImpulseJointHandleRaw handle,
                                Bool wake_up);

/**
 * Computes the Lennard-Jones potential at `distance` for well depth `epsilon`
 * and size parameter `sigma`; returns `NaN` with `ERR_INVALID_ARGUMENT` on
 * invalid parameters.
 *
 * # Safety
 *
 * This function takes no pointers and transfers no ownership; it is always
 * safe to call.
 */
double molecular_lennard_jones_potential(double distance, double epsilon, double sigma);

/**
 * # Safety
 *
 * `out_force` must be null or point to a valid, writable `Vec3`.
 */
Bool molecular_lennard_jones_force(Vec3 displacement,
                                   double epsilon,
                                   double sigma,
                                   double softening,
                                   Vec3 *out_force);

/**
 * Computes the Coulomb potential between `charge_a` and `charge_b` at
 * `distance`; returns `NaN` with `ERR_INVALID_ARGUMENT` on invalid parameters.
 *
 * # Safety
 *
 * This function takes no pointers and transfers no ownership; it is always
 * safe to call.
 */
double molecular_coulomb_potential(double distance,
                                   double charge_a,
                                   double charge_b,
                                   double coulomb_constant,
                                   double relative_permittivity);

/**
 * # Safety
 *
 * `out_force` must be null or point to a valid, writable `Vec3`.
 */
Bool molecular_coulomb_force(Vec3 displacement,
                             double charge_a,
                             double charge_b,
                             double coulomb_constant,
                             double relative_permittivity,
                             double softening,
                             Vec3 *out_force);

/**
 * # Safety
 *
 * `out_report` must be null or point to a valid, writable `MolecularPairReport`.
 */
Bool molecular_pair_interaction(MolecularParticle particle_a,
                                MolecularParticle particle_b,
                                MolecularForceLaw law,
                                MolecularPairReport *out_report);

/**
 * # Safety
 *
 * `world` must be null or a valid world handle. `out_report` must be null or
 * point to a valid, writable `MolecularPairReport`.
 */
Bool molecular_apply_pair_forces(struct WorldHandle *world,
                                 RigidBodyHandleRaw body_a,
                                 RigidBodyHandleRaw body_b,
                                 MolecularParticle particle_a,
                                 MolecularParticle particle_b,
                                 MolecularForceLaw law,
                                 Bool wake_up,
                                 MolecularPairReport *out_report);

/**
 * # Safety
 *
 * Same pointer contract as `molecular_apply_pair_forces`.
 */
uint8_t molecular_apply_pair_forces_flag(struct WorldHandle *world,
                                         RigidBodyHandleRaw body_a,
                                         RigidBodyHandleRaw body_b,
                                         MolecularParticle particle_a,
                                         MolecularParticle particle_b,
                                         MolecularForceLaw law,
                                         Bool wake_up,
                                         MolecularPairReport *out_report);

/**
 * Returns the vacuum Coulomb constant (Coulomb's constant in vacuum).
 *
 * # Safety
 *
 * This function takes no pointers and transfers no ownership; it is always
 * safe to call.
 */
double molecular_vacuum_coulomb_constant(void);

/**
 * Return the number of weights the network layout requires.
 *
 * # Safety
 *
 * This function takes no pointers; any `u32` inputs are safe to pass.
 */
uint32_t neural_bounds_required_weight_count(uint32_t hidden_width, uint32_t hidden_layers);

/**
 * Create a collider builder whose shape is a neural-network-expanded bounds hull.
 *
 * # Safety
 *
 * `weights` must point to a readable buffer of `weight_count` `f64` values.
 * The returned pointer is owned by the caller and must be consumed or freed
 * through the collider-builder ABI.
 */
struct ColliderBuilderHandle *collider_builder_create_neural_bounds(NeuralBoundsDesc desc,
                                                                    const double *weights,
                                                                    uint32_t weight_count);

/**
 * Count the colliders intersecting a neural-bounds shape.
 *
 * # Safety
 *
 * `world` must be a valid world pointer, and `weights` must point to a
 * readable buffer of `weight_count` `f64` values.
 */
uint32_t query_intersect_neural_bounds_count(const struct WorldHandle *world,
                                             NeuralBoundsDesc desc,
                                             const double *weights,
                                             uint32_t weight_count,
                                             QueryFilterDesc filter);

/**
 * Count the colliders intersecting a neural-bounds shape with a default filter.
 *
 * # Safety
 *
 * `world` must be a valid world pointer, and `weights` must point to a
 * readable buffer of `weight_count` `f64` values.
 */
uint32_t query_intersect_neural_bounds_count_all(const struct WorldHandle *world,
                                                 NeuralBoundsDesc desc,
                                                 const double *weights,
                                                 uint32_t weight_count);

/**
 * Write the handles of colliders intersecting a neural-bounds shape.
 *
 * # Safety
 *
 * `world` must be a valid world pointer, `weights` must point to a readable
 * buffer of `weight_count` `f64` values, and `out_handles` must point to a
 * writable buffer of at least `capacity` handle elements.
 */
uint32_t query_intersect_neural_bounds(const struct WorldHandle *world,
                                       NeuralBoundsDesc desc,
                                       const double *weights,
                                       uint32_t weight_count,
                                       QueryFilterDesc filter,
                                       ColliderHandleRaw *out_handles,
                                       uint32_t capacity);

/**
 * Write the handles of colliders intersecting a neural-bounds shape with a
 * default filter.
 *
 * # Safety
 *
 * `world` must be a valid world pointer, `weights` must point to a readable
 * buffer of `weight_count` `f64` values, and `out_handles` must point to a
 * writable buffer of at least `capacity` handle elements.
 */
uint32_t query_intersect_neural_bounds_all(const struct WorldHandle *world,
                                           NeuralBoundsDesc desc,
                                           const double *weights,
                                           uint32_t weight_count,
                                           ColliderHandleRaw *out_handles,
                                           uint32_t capacity);

/**
 * Number of worker threads in the shared rayon pool used by mps-core's
 * parallel force fills, pairwise gravity, snapshot export, and rapier's own
 * parallel solver stages.
 *
 * Defaults to the machine's logical core count; see the `parallel` module
 * docs for the configuration knobs.
 */
uint32_t parallel_thread_count(void);

/**
 * Resize the shared rayon pool. Returns `true` on success; `false` when
 * `threads == 0` (`ERR_INVALID_ARGUMENT`) or the pool is already running
 * (`ERR_UNSUPPORTED` — set the count before the first parallel operation, or
 * via `RAYON_NUM_THREADS` at process start).
 *
 * # Safety
 *
 * No pointer parameters; safe to call from any thread.
 */
Bool parallel_set_thread_count(uint32_t threads);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
RayHit query_cast_ray(const struct WorldHandle *world,
                      Vec3 origin,
                      Vec3 direction,
                      double max_toi,
                      Bool solid,
                      QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be a valid world handle; `out_hit` may be null or must point
 * to writable space for one `RayHit`.
 */
ColliderHandleRaw query_cast_ray_out(const struct WorldHandle *world,
                                     Vec3 origin,
                                     Vec3 direction,
                                     double max_toi,
                                     Bool solid,
                                     QueryFilterDesc filter,
                                     RayHit *out_hit);

/**
 * # Safety
 *
 * `world` must be a valid world handle; `rays` must point to `ray_count * 6`
 * `f64` values and `out_hits` to writable space for `capacity` `RayHit`s.
 */
uint32_t query_cast_rays(const struct WorldHandle *world,
                         const double *rays,
                         uint32_t ray_count,
                         double max_toi,
                         Bool solid,
                         QueryFilterDesc filter,
                         RayHit *out_hits,
                         uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle; `out_collider` may be null or must
 * point to writable space for one collider handle.
 */
PointProjection query_project_point(const struct WorldHandle *world,
                                    Vec3 point,
                                    double max_dist,
                                    Bool solid,
                                    QueryFilterDesc filter,
                                    ColliderHandleRaw *out_collider);

/**
 * # Safety
 *
 * `world` must be a valid world handle; `out_collider` and `out_projection`
 * may be null or must point to writable space for one value each.
 */
ColliderHandleRaw query_project_point_out(const struct WorldHandle *world,
                                          Vec3 point,
                                          double max_dist,
                                          Bool solid,
                                          QueryFilterDesc filter,
                                          ColliderHandleRaw *out_collider,
                                          PointProjection *out_projection);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_point_count(const struct WorldHandle *world,
                                     Vec3 point,
                                     QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_aabb_count(const struct WorldHandle *world,
                                    AabbDesc aabb,
                                    QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_aabb(const struct WorldHandle *world,
                              AabbDesc aabb,
                              QueryFilterDesc filter,
                              ColliderHandleRaw *out_handles,
                              uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_aabb_count_all(const struct WorldHandle *world, AabbDesc aabb);

/**
 * # Safety
 *
 * `world` must be a valid world handle; `aabbs` must point to `query_count`
 * `AabbDesc` values and `out_counts` to writable space for `capacity` `u32`s.
 */
uint32_t query_intersect_aabb_counts(const struct WorldHandle *world,
                                     const AabbDesc *aabbs,
                                     uint32_t query_count,
                                     QueryFilterDesc filter,
                                     uint32_t *out_counts,
                                     uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_obb_count(const struct WorldHandle *world,
                                   Obb obb,
                                   QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_obb_count_all(const struct WorldHandle *world, Obb obb);

/**
 * # Safety
 *
 * `world` must be a valid world handle; `obbs` must point to `query_count`
 * `Obb` values and `out_counts` to writable space for `capacity` `u32`s.
 */
uint32_t query_intersect_obb_counts(const struct WorldHandle *world,
                                    const Obb *obbs,
                                    uint32_t query_count,
                                    QueryFilterDesc filter,
                                    uint32_t *out_counts,
                                    uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_obb(const struct WorldHandle *world,
                             Obb obb,
                             QueryFilterDesc filter,
                             ColliderHandleRaw *out_handles,
                             uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_obb_all(const struct WorldHandle *world,
                                 Obb obb,
                                 ColliderHandleRaw *out_handles,
                                 uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_sphere_count(const struct WorldHandle *world,
                                      Sphere sphere,
                                      QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_sphere_count_all(const struct WorldHandle *world, Sphere sphere);

/**
 * # Safety
 *
 * `world` must be a valid world handle; `spheres` must point to `query_count`
 * `Sphere` values and `out_counts` to writable space for `capacity` `u32`s.
 */
uint32_t query_intersect_sphere_counts(const struct WorldHandle *world,
                                       const Sphere *spheres,
                                       uint32_t query_count,
                                       QueryFilterDesc filter,
                                       uint32_t *out_counts,
                                       uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_sphere(const struct WorldHandle *world,
                                Sphere sphere,
                                QueryFilterDesc filter,
                                ColliderHandleRaw *out_handles,
                                uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` collider handles.
 */
uint32_t query_intersect_sphere_all(const struct WorldHandle *world,
                                    Sphere sphere,
                                    ColliderHandleRaw *out_handles,
                                    uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
uint32_t query_intersect_aabb_rigid_body_count_all(const struct WorldHandle *world, AabbDesc aabb);

/**
 * # Safety
 *
 * `world` must be a valid world handle and `out_handles` must point to
 * writable space for at least `capacity` rigid body handles.
 */
uint32_t query_intersect_aabb_rigid_bodies_all(const struct WorldHandle *world,
                                               AabbDesc aabb,
                                               RigidBodyHandleRaw *out_handles,
                                               uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be a valid world handle.
 */
ShapeCastHit query_cast_shape(const struct WorldHandle *world,
                              ShapeDesc shape_desc,
                              Vec3 translation,
                              Quat rotation,
                              Vec3 velocity,
                              ShapeCastOptionsDesc options,
                              QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be a valid world handle; `out_hit` may be null or must point
 * to writable space for one `ShapeCastHit`.
 */
ColliderHandleRaw query_cast_shape_out(const struct WorldHandle *world,
                                       ShapeDesc shape_desc,
                                       Vec3 translation,
                                       Quat rotation,
                                       Vec3 velocity,
                                       ShapeCastOptionsDesc options,
                                       QueryFilterDesc filter,
                                       ShapeCastHit *out_hit);

/**
 * Creates a rigid body builder for the given body status.
 *
 * # Safety
 *
 * Takes no pointers. The returned pointer is owned by the caller and must be released with
 * `rigid_body_builder_build` or `rigid_body_builder_destroy`.
 */
struct RigidBodyBuilderHandle *rigid_body_builder_create(uint32_t status);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create` (or null); ownership
 * is taken and the pointer must not be used afterwards.
 */
RigidBody *rigid_body_builder_build(struct RigidBodyBuilderHandle *builder);

/**
 * # Safety
 *
 * `builder` must be a pointer returned by `rigid_body_builder_create` (or null, which is a
 * no-op); ownership is taken and the pointer must not be used afterwards.
 */
void rigid_body_builder_destroy(struct RigidBodyBuilderHandle *builder);

/**
 * # Safety
 *
 * `rigid_body` must be a pointer returned by `rigid_body_builder_build` or
 * `world_copy_rigid_body` (or null, which is a no-op); ownership is taken and the pointer must
 * not be used afterwards.
 */
void rigid_body_destroy_raw(RigidBody *rigid_body);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_translation(struct RigidBodyBuilderHandle *builder, Vec3 translation);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_rotation(struct RigidBodyBuilderHandle *builder,
                                     Vec3 rotation_axis_angle);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_pose(struct RigidBodyBuilderHandle *builder,
                                 Vec3 translation,
                                 Quat rotation);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_additional_mass_properties(struct RigidBodyBuilderHandle *builder,
                                                       Vec3 center,
                                                       double mass,
                                                       Vec3 inertia);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_linvel(struct RigidBodyBuilderHandle *builder, Vec3 linvel);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_angvel(struct RigidBodyBuilderHandle *builder, Vec3 angvel);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_gravity_scale(struct RigidBodyBuilderHandle *builder,
                                          double gravity_scale);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_linear_damping(struct RigidBodyBuilderHandle *builder,
                                           double linear_damping);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_angular_damping(struct RigidBodyBuilderHandle *builder,
                                            double angular_damping);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_can_sleep(struct RigidBodyBuilderHandle *builder, Bool can_sleep);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_enabled_rotations(struct RigidBodyBuilderHandle *builder,
                                              Bool allow_x,
                                              Bool allow_y,
                                              Bool allow_z);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_user_data(struct RigidBodyBuilderHandle *builder,
                                      uint64_t user_data_low,
                                      uint64_t user_data_high);

/**
 * # Safety
 *
 * `builder` must be a live pointer returned by `rigid_body_builder_create`, or null.
 */
void rigid_body_builder_set_additional_mass(struct RigidBodyBuilderHandle *builder, double mass);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null. `memory_handle` must be a
 * pointer returned by `rigid_body_builder_build`; ownership is taken and the pointer must not be
 * used afterwards.
 */
RigidBodyHandleRaw world_insert_rigid_body(struct WorldHandle *world, RigidBody *memory_handle);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool world_remove_rigid_body(struct WorldHandle *world,
                             RigidBodyHandleRaw handle,
                             Bool remove_attached_colliders);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null. The returned pointer is
 * owned by the caller and must be released with `rigid_body_destroy_raw`.
 */
RigidBody *world_copy_rigid_body(struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t world_remove_rigid_body_flag(struct WorldHandle *world,
                                     RigidBodyHandleRaw handle,
                                     Bool remove_attached_colliders);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint32_t rigid_body_get_status(const struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_set_status(struct WorldHandle *world,
                           RigidBodyHandleRaw handle,
                           uint32_t status,
                           Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Vec3 rigid_body_get_translation(const struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null. `out_translation` must be
 * a valid writable pointer to a `Vec3`, or null.
 */
void rigid_body_get_translation_out(const struct WorldHandle *world,
                                    RigidBodyHandleRaw handle,
                                    Vec3 *out_translation);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Quat rigid_body_get_rotation(const struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null. `out_rotation` must be a
 * valid writable pointer to a `Quat`, or null.
 */
void rigid_body_get_rotation_out(const struct WorldHandle *world,
                                 RigidBodyHandleRaw handle,
                                 Quat *out_rotation);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_set_pose(struct WorldHandle *world,
                         RigidBodyHandleRaw handle,
                         Vec3 translation,
                         Quat rotation,
                         Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_set_translation(struct WorldHandle *world,
                                RigidBodyHandleRaw handle,
                                Vec3 translation,
                                Bool wake_up);

Bool rigid_body_set_next_kinematic_position(struct WorldHandle *world,
                                            RigidBodyHandleRaw handle,
                                            Vec3 translation);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_set_translation_flag(struct WorldHandle *world,
                                        RigidBodyHandleRaw handle,
                                        Vec3 translation,
                                        Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_set_rotation(struct WorldHandle *world,
                             RigidBodyHandleRaw handle,
                             Quat rotation,
                             Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_set_rotation_flag(struct WorldHandle *world,
                                     RigidBodyHandleRaw handle,
                                     Quat rotation,
                                     Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_set_pose_flag(struct WorldHandle *world,
                                 RigidBodyHandleRaw handle,
                                 Vec3 translation,
                                 Quat rotation,
                                 Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
double rigid_body_get_mass(struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Vec3 rigid_body_get_force(const struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Vec3 rigid_body_get_linvel(const struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null. `out_linvel` must be a
 * valid writable pointer to a `Vec3`, or null.
 */
void rigid_body_get_linvel_out(const struct WorldHandle *world,
                               RigidBodyHandleRaw handle,
                               Vec3 *out_linvel);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_set_linvel(struct WorldHandle *world,
                           RigidBodyHandleRaw handle,
                           Vec3 linvel,
                           Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_set_linvel_flag(struct WorldHandle *world,
                                   RigidBodyHandleRaw handle,
                                   Vec3 linvel,
                                   Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Vec3 rigid_body_get_angvel(const struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null. `out_angvel` must be a
 * valid writable pointer to a `Vec3`, or null.
 */
void rigid_body_get_angvel_out(const struct WorldHandle *world,
                               RigidBodyHandleRaw handle,
                               Vec3 *out_angvel);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_set_angvel(struct WorldHandle *world,
                           RigidBodyHandleRaw handle,
                           Vec3 angvel,
                           Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_set_angvel_flag(struct WorldHandle *world,
                                   RigidBodyHandleRaw handle,
                                   Vec3 angvel,
                                   Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_add_force(struct WorldHandle *world,
                          RigidBodyHandleRaw handle,
                          Vec3 force,
                          Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_add_force_at_point(struct WorldHandle *world,
                                   RigidBodyHandleRaw handle,
                                   Vec3 force,
                                   Vec3 point,
                                   Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_add_force_at_local_point(struct WorldHandle *world,
                                         RigidBodyHandleRaw handle,
                                         Vec3 force,
                                         Vec3 local_point,
                                         Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_add_torque_at_local_point(struct WorldHandle *world,
                                          RigidBodyHandleRaw handle,
                                          Vec3 torque,
                                          Vec3 _local_point,
                                          Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_add_force_at_local_point_flag(struct WorldHandle *world,
                                                 RigidBodyHandleRaw handle,
                                                 Vec3 force,
                                                 Vec3 local_point,
                                                 Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_add_torque_at_local_point_flag(struct WorldHandle *world,
                                                  RigidBodyHandleRaw handle,
                                                  Vec3 torque,
                                                  Vec3 local_point,
                                                  Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_reset_force(struct WorldHandle *world, RigidBodyHandleRaw handle, Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_add_force_flag(struct WorldHandle *world,
                                  RigidBodyHandleRaw handle,
                                  Vec3 force,
                                  Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_add_torque(struct WorldHandle *world,
                           RigidBodyHandleRaw handle,
                           Vec3 torque,
                           Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_reset_torque(struct WorldHandle *world, RigidBodyHandleRaw handle, Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_add_torque_flag(struct WorldHandle *world,
                                   RigidBodyHandleRaw handle,
                                   Vec3 torque,
                                   Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_apply_impulse(struct WorldHandle *world,
                              RigidBodyHandleRaw handle,
                              Vec3 impulse,
                              Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_apply_impulse_flag(struct WorldHandle *world,
                                      RigidBodyHandleRaw handle,
                                      Vec3 impulse,
                                      Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_apply_torque_impulse(struct WorldHandle *world,
                                     RigidBodyHandleRaw handle,
                                     Vec3 torque_impulse,
                                     Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_apply_torque_impulse_flag(struct WorldHandle *world,
                                             RigidBodyHandleRaw handle,
                                             Vec3 torque_impulse,
                                             Bool wake_up);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_enable_ccd(struct WorldHandle *world, RigidBodyHandleRaw handle, Bool enabled);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_enable_ccd_flag(struct WorldHandle *world,
                                   RigidBodyHandleRaw handle,
                                   Bool enabled);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_sleep(struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_sleep_flag(struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_wake_up(struct WorldHandle *world, RigidBodyHandleRaw handle, Bool strong);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_wake_up_flag(struct WorldHandle *world, RigidBodyHandleRaw handle, Bool strong);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
Bool rigid_body_is_sleeping(const struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * # Safety
 *
 * `world` must be a live pointer returned by `world_create`, or null.
 */
uint8_t rigid_body_is_sleeping_flag(const struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * Create an empty R-tree index.
 *
 * # Safety
 *
 * The returned pointer is owned by the caller and must be freed exactly once
 * with `rtree_destroy`.
 */
struct RTreeHandle *rtree_create(void);

/**
 * Destroy an R-tree index created by `rtree_create`.
 *
 * # Safety
 *
 * `tree` must be null or a pointer returned by `rtree_create`; it must not be
 * used again after this call.
 */
void rtree_destroy(struct RTreeHandle *tree);

/**
 * Remove every entry from the tree.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `rtree_create`.
 */
void rtree_clear(struct RTreeHandle *tree);

/**
 * Return the number of entries stored in the tree.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `rtree_create`.
 */
uint32_t rtree_len(const struct RTreeHandle *tree);

/**
 * Insert or overwrite the bounds of `id` in the tree.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `rtree_create`.
 */
Bool rtree_insert(struct RTreeHandle *tree, uint64_t id, AabbDesc aabb);

/**
 * Update the bounds of an existing `id` in the tree.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `rtree_create`.
 */
Bool rtree_update(struct RTreeHandle *tree, uint64_t id, AabbDesc aabb);

/**
 * Remove `id` from the tree.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `rtree_create`.
 */
Bool rtree_remove(struct RTreeHandle *tree, uint64_t id);

/**
 * Force an immediate rebuild of the tree structure.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `rtree_create`.
 */
void rtree_rebuild(struct RTreeHandle *tree);

/**
 * Count the entries whose bounds intersect `aabb`.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `rtree_create`.
 */
uint32_t rtree_query_aabb_count(struct RTreeHandle *tree, AabbDesc aabb);

/**
 * Write the ids of entries whose bounds intersect `aabb` into `out_ids`.
 *
 * # Safety
 *
 * `tree` must be a valid pointer returned by `rtree_create`, and `out_ids`
 * must point to a writable buffer of at least `capacity` `u64` elements.
 */
uint32_t rtree_query_aabb(struct RTreeHandle *tree,
                          AabbDesc aabb,
                          uint64_t *out_ids,
                          uint32_t capacity);

/**
 * Create a servo body in `world` from a collider shape and an initial
 * translation. The body is dynamic with a collider parented to it. Returns a
 * stable id, or `u32::MAX` on bad arguments.
 *
 * - `kp`: proportional gain (applied uniformly to all axes).
 * - `kd`: derivative gain (applied uniformly to all axes).
 * - `ki`: integral gain. When `> 0`, a full `PidController` is used; when
 *   `== 0`, a pure `PdController` (no integral term) is used instead.
 * - `axes`: bitfield selecting which axes the controller affects (see
 *   `axes_from_u8`). `0` means all axes.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `shape` must be
 * a valid [`ShapeDesc`] (finite params).
 */
uint32_t servo_body_create(struct WorldHandle *world,
                           ShapeDesc shape,
                           Vec3 translation,
                           double kp,
                           double kd,
                           double ki,
                           uint8_t axes);

/**
 * Set the target world-space position the servo drives toward.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool servo_body_set_target_position(struct WorldHandle *world, uint32_t id, Vec3 position);

/**
 * Set the target world-space rotation (as a quaternion `i, j, k, w`) the servo
 * drives toward.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool servo_body_set_target_rotation(struct WorldHandle *world, uint32_t id, Quat rotation);

/**
 * Set the target linear velocity (world space) the servo drives toward.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool servo_body_set_target_velocity(struct WorldHandle *world, uint32_t id, Vec3 velocity);

/**
 * Set the target angular velocity (world space) the servo drives toward.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool servo_body_set_target_angular_velocity(struct WorldHandle *world, uint32_t id, Vec3 velocity);

/**
 * Advance the servo controller by `dt`: compute the PD/PID velocity-level
 * correction from the body's current pose/velocity vs. the target and write
 * it back via `set_linvel`/`set_angvel`. Call **after** `world_step`.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool servo_body_update(struct WorldHandle *world, uint32_t id, double dt);

/**
 * Read the body's world-space translation.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `out` may be null.
 */
Bool servo_body_get_translation(const struct WorldHandle *world, uint32_t id, Vec3 *out);

/**
 * Read the body's world-space linear velocity.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `out` may be null.
 */
Bool servo_body_get_velocity(const struct WorldHandle *world, uint32_t id, Vec3 *out);

/**
 * Read the packed rigid-body handle so the caller can use the general
 * `rigid_body_*` FFI (forces, impulses, mass properties, etc.) on the
 * servo's underlying body.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
RigidBodyHandleRaw servo_body_get_rigid_body_handle(const struct WorldHandle *world, uint32_t id);

/**
 * Destroy a servo body by id. Removes the controller, the rigid body, and its
 * parented collider from the world. Returns `FALSE` if the id is unknown.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool servo_body_destroy(struct WorldHandle *world, uint32_t id);

/**
 * Create a skeletal soft body as a chain (line) of spring-linked rigid nodes.
 *
 * Nodes are placed `spacing` apart along `axis` starting at the world origin (or
 * at `anchor` if `anchor != 0`). Adjacent nodes are joined by a spring joint with
 * the given `stiffness`/`damping`; the spring's rest length is `spacing`, so the
 * chain behaves like a soft rope / articulated strand.
 *
 * # Parameters
 * * `node_count` — number of nodes (must be ≥ 1).
 * * `spacing` — distance between adjacent nodes / spring rest length (> 0).
 * * `node_mass` — mass of each node (> 0).
 * * `node_radius` — collision sphere radius of each node (> 0).
 * * `anchor` — `RigidBodyHandleRaw` to pin the first node to (0 = first node is a
 *   free/fixed root at the origin; pass a valid handle to hang from it).
 * * `axis` — unit direction of the chain (need not be normalized; it is normalized
 *   internally; must be finite and non-zero).
 * * `stiffness` / `damping` — spring coefficients (≥ 0).
 *
 * # Returns
 * The number of nodes successfully created (0 on error). On partial failure the
 * already-created nodes/joints remain in the world (caller may clear the world).
 *
 * # Safety
 * `world` must be a valid world pointer returned by `world_create`.
 */
uint32_t soft_chain_create(struct WorldHandle *world,
                           uint32_t node_count,
                           double spacing,
                           double node_mass,
                           double node_radius,
                           RigidBodyHandleRaw anchor,
                           Vec3 axis,
                           double stiffness,
                           double damping);

/**
 * Read back the node handles of a soft chain that was just created.
 *
 * Call [`soft_chain_create`] first; the chain's node handles are the last
 * `count` *dynamic* bodies, but to avoid ambiguity this helper snapshots the
 * *currently dynamic* bodies whose colliders are spheres of `node_radius`. For
 * simplicity it returns the handles of all dynamic bodies currently in the world
 * (callers typically create a fresh world per chain).
 *
 * # Safety
 * `world` must be a valid world pointer; `out_handles` must point to writable
 * memory for `capacity` handles.
 */
uint32_t soft_chain_node_handles(const struct WorldHandle *world,
                                 RigidBodyHandleRaw *out_handles,
                                 uint32_t capacity);

/**
 * Build a mass-spring soft body from a `VoxelGrid` (Minecraft chunk).
 *
 * One point-mass particle is placed at the center of every *solid* voxel
 * (`voxels[i] != 0`). Face-adjacent solid voxels are connected by a Hookean
 * spring with the given `stiffness`/`damping` and rest length equal to the
 * cell spacing along that axis. The resulting [`SoftBody`] is inserted into the
 * world's `soft_bodies` set and advanced by `world_step`.
 *
 * # Parameters
 * * `voxels` — flat `size_x * size_y * size_z` array, indexing `x + size_x*(z +
 *   size_z*y)`; non-zero = solid.
 * * `size_x/y/z` — grid dimensions (each > 0, product ≤ `voxels.len()`).
 * * `voxel_size` — world-space size of one cell edge (uniform; > 0).
 * * `origin` — world-space position of the (0,0,0) cell corner.
 * * `particle_mass` — mass of each solid-cell particle (> 0).
 * * `stiffness` / `damping` — spring coefficients (≥ 0).
 * * `pin_boundary` — when non-zero, particles whose cell touches the grid edge
 *   are created pinned (`inv_mass = 0`), so the soft body is anchored to the
 *   chunk boundary (useful for hanging terrain/structures from the world).
 *
 * # Returns
 * The `SoftBodyId` (as `u32`) on success, or `0` on error (`ERR_*`).
 *
 * # Safety
 * `world` must be a valid world pointer; `voxels` must point to `voxels_len`
 * readable bytes.
 */
uint32_t soft_body_voxel_build(struct WorldHandle *world,
                               const uint8_t *voxels,
                               uint32_t voxels_len,
                               uint32_t size_x,
                               uint32_t size_y,
                               uint32_t size_z,
                               double voxel_size,
                               Vec3 origin,
                               double particle_mass,
                               double stiffness,
                               double damping,
                               Bool pin_boundary);

/**
 * Set the per-body constant acceleration (gravity) of a soft body.
 *
 * This is the terrain-gravity coupling hook: the caller samples
 * `terrain_gravity_acceleration` per step and writes the resulting vector here,
 * so a soft body falls under planetary/spherical gravity instead of the world's
 * uniform `gravity`. Returns `Bool::TRUE` on success.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_set_gravity(struct WorldHandle *world, uint32_t id, Vec3 gravity);

/**
 * Phase 7: enable a uniform wind / air-resistance field on a soft body.
 *
 * `accel` is a constant wind acceleration (`m/s²`) applied to every free
 * particle (like a sideways gravity); `drag` is a linear air-resistance
 * coefficient (`1/s`, `F_drag = -m·drag·v`). Both components must be finite.
 *
 * # Returns
 * `Bool::TRUE` on success, `Bool::FALSE` on `ERR_*` (null world, bad id,
 * non-finite arguments).
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_apply_wind(struct WorldHandle *world, uint32_t id, Vec3 accel, double drag);

/**
 * Phase 7: disable the wind field on a soft body (`None`).
 *
 * # Returns
 * `Bool::TRUE` on success, `Bool::FALSE` on `ERR_*` (null world, bad id).
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_clear_wind(struct WorldHandle *world, uint32_t id);

/**
 * Phase 28 — 关闭内部气压（等同 `pressure = None`，气球瘪掉）。
 *
 * # Returns
 * `Bool::TRUE` 成功；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_clear_pressure(struct WorldHandle *world, uint32_t id);

/**
 * Phase 28 — 关闭自碰撞（等同 `self_collision = None`，无摩擦）。
 *
 * # Returns
 * `Bool::TRUE` 成功；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_clear_self_collision(struct WorldHandle *world, uint32_t id);

/**
 * Phase 28 — 关闭跨体（软软）碰撞（等同 `cross_collision = None`，无摩擦）。
 *
 * # Returns
 * `Bool::TRUE` 成功；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_clear_cross_collision(struct WorldHandle *world, uint32_t id);

/**
 * Phase 28 — 关闭体积守恒约束（等同 `volume_conservation = None`，blob 可随意压缩）。
 *
 * # Returns
 * `Bool::TRUE` 成功；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_clear_volume_conservation(struct WorldHandle *world,
                                         uint32_t id);

/**
 * # Phase 29 — 开启四面体 corotated 线性弹性(旋转不变形状匹配)
 *
 * 每个 XPBD 迭代里,在体积约束之后,把每个四面体向其 rest 形状的最优旋转
 * 匹配(polar 分解 shape matching)投影,提供旋转不变的线弹性偏应变回复。
 * rest 形状在调用时刻从当前质点位置快照(在未形变网格上开启)。
 * `stiffness` 为逐迭代松弛系数,取值 `(0, 1]`。
 *
 * # Returns
 * `Bool::TRUE` 成功开启;`id` 未知 / world 为 null / `stiffness` 非法(非有限、<=0、>1) → `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效;`stiffness` 需为有限且 `0 < stiffness <= 1`。
 */
Bool soft_body_set_corotated(struct WorldHandle *world,
                             uint32_t id,
                             double stiffness);

/**
 * # Phase 29 — 关闭 corotated 线性弹性
 *
 * 等同 `corotated = None`;体积约束等其他特性不受影响。
 *
 * # Returns
 * `Bool::TRUE` 成功关闭;`id` 未知 / world 为 null → `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_clear_corotated(struct WorldHandle *world, uint32_t id);

/**
 * # Phase 30 — 开启 Neo-Hookean 对数体积能量
 *
 * 每个 XPBD 迭代里，四面体体积约束改用非线性残差 `C = ln(V/V₀)`
 * (J 下限 1e-6 保持有限)，compliance = `stiffness/dt²`。对数形式使体积抵抗
 * 随压缩无界增长(物理正确的不可压缩性)，取代线性 `V − V₀` 的有限推回。
 * 开启时覆盖 `volume_conservation` 的 compliance;关闭后回退线性体积约束。
 *
 * # Returns
 * `Bool::TRUE` 成功开启;`id` 未知 / world 为 null / `stiffness` 非法(非有限、负数) → `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效;`stiffness` 需为有限非负值。
 */
Bool soft_body_set_neo_hookean(struct WorldHandle *world,
                               uint32_t id,
                               double stiffness);

/**
 * # Phase 30 — 关闭 Neo-Hookean 体积能量
 *
 * 等同 `neo_hookean = None`;体积约束回退线性残差 + `volume_conservation` compliance。
 *
 * # Returns
 * `Bool::TRUE` 成功关闭;`id` 未知 / world 为 null → `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_clear_neo_hookean(struct WorldHandle *world, uint32_t id);

/**
 * Phase 31 — 设置软体全局主动应变激活系数 γ∈[0,1]（「肌肉收缩」等级）。
 *
 * 每条弹簧/距离约束的有效静止长度变为 `rest * (1 - γ)`，正值主动把两端拉近。
 * 非有限值被忽略（无操作）。`0` 为被动基线。
 *
 * # Returns
 * `Bool::TRUE` 总是成功（除非 world 为 null 或 id 未知返回 `Bool::FALSE`）。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_set_activation(struct WorldHandle *world,
                              uint32_t id,
                              double gamma);

/**
 * Phase 31 — 设置单条弹簧（按 `add_spring` 返回的索引）的主动应变激活系数。
 *
 * 越界 / 非有限 / 不在 [0,1] 的 `activation` 被拒绝，返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_set_spring_activation(struct WorldHandle *world,
                                     uint32_t id,
                                     uint32_t index,
                                     double activation);

/**
 * Phase 31 — 设置单条距离约束（按 `add_distance_constraint` 返回的索引）的主动应变激活系数。
 *
 * 越界 / 非有限 / 不在 [0,1] 的 `activation` 被拒绝，返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_set_distance_constraint_activation(struct WorldHandle *world,
                                                  uint32_t id,
                                                  uint32_t index,
                                                  double activation);

/**
 * Phase 32 — 设置单条距离约束（按 `add_distance_constraint` 返回的索引）的肌肉
 * 纤维走向 `dir = (dx, dy, dz)`。非零向量被归一化后作为主动收缩方向（各向异性
 * 驱动）；全零向量清除纤维（退回沿边收缩）。返回 `Bool::FALSE` 表示 `id` 未知 /
 * 索引越界 / 向量非有限。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_set_fibre_direction(struct WorldHandle *world,
                                   uint32_t id,
                                   uint32_t index,
                                   double dx,
                                   double dy,
                                   double dz);

/**
 * Phase 32 — 设置单条弹簧（按 `add_spring` 返回的索引）的肌肉纤维走向，语义同
 * `soft_body_set_fibre_direction`。返回 `Bool::FALSE` 表示 `id` 未知 / 索引越界 /
 * 向量非有限。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_set_spring_fibre_direction(struct WorldHandle *world,
                                          uint32_t id,
                                          uint32_t index,
                                          double dx,
                                          double dy,
                                          double dz);

/**
 * Phase 28 — 关闭黏连/可撕 glue（等同 `cohesion = None`，不再互相吸附）。
 *
 * # Returns
 * `Bool::TRUE` 成功；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_clear_cohesion(struct WorldHandle *world, uint32_t id);

/**
 * Phase 7: mark a soft body as sleeping (no further integration until woken).
 *
 * # Returns
 * `Bool::TRUE` if the body existed and was put to sleep, `Bool::FALSE` on
 * `ERR_*` (null world, bad id).
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_sleep(struct WorldHandle *world, uint32_t id);

/**
 * Phase 7: wake a sleeping soft body (resume integration).
 *
 * # Returns
 * `Bool::TRUE` if the body existed and was woken, `Bool::FALSE` on `ERR_*`
 * (null world, bad id).
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_wake(struct WorldHandle *world, uint32_t id);

/**
 * Phase 7: whether a soft body is currently sleeping.
 *
 * # Returns
 * `Bool::TRUE` if sleeping, `Bool::FALSE` if awake or the id is unknown /
 * world is null (and `ERR_*` is set).
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_is_sleeping(const struct WorldHandle *world, uint32_t id);

/**
 * Phase 7: total kinetic energy of a soft body's free particles (`½·m·|v|²`).
 *
 * # Returns
 * The kinetic energy (finite), or `0.0` with `ERR_*` set on null world / bad id.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
double soft_body_kinetic_energy(const struct WorldHandle *world, uint32_t id);

/**
 * Phase 7: normalized total volume of a soft body's tetrahedra
 * (sum of `|V|/|V_rest|`, so a unit-scaled, deformation-sensitive scalar).
 * For bodies with no tetrahedra this is `0.0`.
 *
 * # Returns
 * The normalized volume (finite), or `0.0` with `ERR_*` set on null world /
 * bad id.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
double soft_body_total_volume(const struct WorldHandle *world, uint32_t id);

/**
 * # Phase 8 — 锚定软体任意质点到刚体
 *
 * 把 `id` 软体的第 `particle` 号质点绑定到刚体 `body`，使其刚性跟随该刚体的
 * 平移/旋转。`attach_point` 为绑点世界坐标（通常用该质点当前位置）；函数内部
 * 把它换算成刚体局部坐标存储，故跟随刚体运动时不会漂移。绑定后该质点停止本地
 * 积分，其弹簧/阻尼力改由 `SoftBodySet::write_spring_forces` 路由进刚体的
 * `force_containers`（软体拖动刚体）。
 *
 * # Returns
 * `Bool::TRUE` 成功；`particle` 越界 / `body` 不存在 / world 为 null 返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效；`attach_point` 各分量需为有限值。
 */
Bool soft_body_attach_particle(struct WorldHandle *world,
                               uint32_t id,
                               uint32_t particle,
                               RigidBodyHandleRaw body,
                               Vec3 attach_point);

/**
 * # Phase 8 — 解除质点与刚体的锚定
 *
 * 把 `id` 软体的第 `particle` 号质点从任何已绑定刚体上解绑，恢复为自由（本地积分）
 * 质点。已自由则视为成功（幂等）。
 *
 * # Returns
 * `Bool::TRUE` 成功（含已自由）；`particle` 越界 / world 为 null 返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_detach_particle(struct WorldHandle *world,
                               uint32_t id,
                               uint32_t particle);

/**
 * # Phase 9 — 设置撕裂阈值（应变阈值）
 *
 * 把 `id` 软体的撕裂阈值设为 `strain_to_break`（应变 = `(|len| − rest)/rest`，
 * 即拉伸量相对静止长度的比例）。每步 `step` 开始时，任何应变超过该阈值的**结构边**
 * （XPBD distance constraint 或 MassSpring spring）会被移除；失去任一结构边的三角形面
 * 也会被删掉，使撕裂的布料停止渲染破损面。
 *
 * - `enabled != 0` 且 `strain_to_break > 0`：开启撕裂（阈值 = `strain_to_break`）。
 * - `enabled == 0`：关闭撕裂（等同于 `tear_strain = None`，默认）。
 * - `strain_to_break <= 0`：视为非法，关闭撕裂（避免首步即全撕）。
 *
 * # Returns
 * `Bool::TRUE` 成功（含关闭）；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效；`strain_to_break` 需为有限值。
 */
Bool soft_body_set_tear_strain(struct WorldHandle *world,
                               uint32_t id,
                               double strain_to_break,
                               uint8_t enabled);

/**
 * # Phase 27 — 设置断裂力学撕裂准则（轴向应力阈值）
 *
 * 把 `id` 软体的撕裂准则设为 `Stress(threshold)`：任何结构边（XPBD distance
 * constraint 或 MassSpring spring）的轴向力 `|k·(len − rest)|` 超过 `threshold`
 * 时断裂。`k` = 弹簧刚度，或 `1/(compliance + ε)`（XPBD 距离约束）。
 * `enabled == 0` 或 `threshold <= 0` 关闭撕裂（等同于 `tear = None`）。
 *
 * # Returns
 * `Bool::TRUE` 成功（含关闭）；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
 */
Bool soft_body_set_tear_stress(struct WorldHandle *world,
                               uint32_t id,
                               double stress_to_break,
                               uint8_t enabled);

/**
 * # Phase 27 — 设置断裂力学撕裂准则（应变能 / 断裂韧性阈值）
 *
 * 把 `id` 软体的撕裂准则设为 `Energy(threshold)`：任何结构边的弹性应变能
 * `½·k·(len − rest)²` 超过 `threshold` 时断裂（断裂韧性临界释放率代理）。
 * `enabled == 0` 或 `threshold <= 0` 关闭撕裂（等同于 `tear = None`）。
 *
 * # Returns
 * `Bool::TRUE` 成功（含关闭）；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
 */
Bool soft_body_set_tear_energy(struct WorldHandle *world,
                               uint32_t id,
                               double energy_to_break,
                               uint8_t enabled);

/**
 * # Phase 27 — 设置体级正交各向异性刚度轴
 *
 * `anisotropy != 0` 且 `x,y,z 有限且 >= 0` 时，开启方向相关刚度：每条边有效
 * XPBD 柔度 = `base / (nᵀ·diag(x,y,z)·n)`（n 为边单位方向），使沿 x 轴对齐的
 * 边在 `x > 1` 时更硬。传 `enabled == 0` 或 `x=y=z=0` 关闭（各边保持各向同性）。
 *
 * # Returns
 * `Bool::TRUE` 成功（含关闭）；`world` 为 null / 向量含非有限值 / 含负分量
 * 返回 `Bool::FALSE`。
 */
Bool soft_body_set_anisotropy(struct WorldHandle *world,
                              uint32_t id,
                              double x,
                              double y,
                              double z,
                              uint8_t enabled);

/**
 * # Phase 27 — 设置黏弹性（率相关）本构
 *
 * `enabled != 0` 且 `rate_coefficient >= 0`：开启 Kelvin-Voigt 式应变率硬化——
 * 有效刚度 `k_eff = k·(1 + rate_coefficient·|d(strain)/dt|)`，快速拉伸的边比缓慢
 * 拉伸更硬（聚合物/黏弹性行为）。`enabled == 0` 或非法参数关闭（纯弹性）。
 *
 * # Returns
 * `Bool::TRUE` 成功（含关闭）；非法参数返回 `Bool::FALSE`。
 */
Bool soft_body_set_viscoelastic(struct WorldHandle *world,
                                uint32_t id,
                                double rate_coefficient,
                                uint8_t enabled);

/**
 * # Phase 27 — 设置均匀温度场（热膨胀 + 温度相关模量）
 *
 * `enabled != 0` 且参数有限、`stiffness_temp_coeff·|temp−ambient| < 1`：开启温度场——
 * 每条边静止长度按 `rest·(1 + expansion·ΔT)` 膨胀，刚度按 `k·(1 − stiffness_temp_coeff·ΔT)`
 * 软化。关闭（`enabled == 0` 或非法）回到等温。
 *
 * # Returns
 * `Bool::TRUE` 成功（含关闭）；非法参数返回 `Bool::FALSE`。
 */
Bool soft_body_set_thermal(struct WorldHandle *world,
                           uint32_t id,
                           double temp,
                           double ambient,
                           double expansion,
                           double stiffness_temp_coeff,
                           uint8_t enabled);

/**
 * # Phase 10 — 设置塑性参数（永久变形 / 像橡皮泥 / 记忆棉）
 *
 * 把 `id` 软体的塑性设为 `PlasticityParams { yield_strain, creep }`：
 * - 任何结构边（XPBD distance constraint 或 MassSpring spring）的弹性应变幅度
 *   `|(|len| − rest)/rest|` 超过 `yield_strain` 时，每步把 rest_length 朝当前长度
 *   方向移动 `creep`（夹到 `[0,1]`），使变形永久"冻住"而不是回弹。
 * - `enabled != 0` 且 `yield_strain > 0`：开启塑性（threshold=yield_strain, rate=creep）。
 * - `enabled == 0`：关闭塑性（等同于 `plasticity = None`，即完全弹性，默认）。
 * - `yield_strain <= 0` 或 `creep <= 0`：视为非法，关闭塑性。
 *
 * # Returns
 * `Bool::TRUE` 成功（含关闭）；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效；`yield_strain` / `creep` 需为有限值。
 */
Bool soft_body_set_plasticity(struct WorldHandle *world,
                              uint32_t id,
                              double yield_strain,
                              double creep,
                              uint8_t enabled);

/**
 * Phase 28 — 手动触发一次塑性投影（把超 `yield_strain` 的结构边 rest_length 朝当前长度
 * 冻结 `creep`）。通常塑性在 `step` 内自动应用；此 FFI 让调用方在不推进时间步的情况下
 * 即时「定型」（例如绑定到 Minecraft 方块编辑一次后立刻烤出永久变形）。需先经
 * `soft_body_set_plasticity` 配置过 `PlasticityParams`，否则为 no-op。
 *
 * # Returns
 * `Bool::TRUE` 成功触发；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_apply_plasticity(struct WorldHandle *world,
                                uint32_t id);

/**
 * Phase 28 — 手动触发一次撕裂：立刻丢弃所有超过 `tear` 阈值（应变 / 轴向应力 /
 * 应变能，由 `soft_body_set_tear_*` 配置）的结构边，并连带删掉失去边支撑的三角面。
 * 通常撕裂在 `step` 顶部自动发生；此 FFI 让调用方在「不推进时间步」时也能立即撕开
 * （例如一次性加载预撕裂状态、或在联动闭环里随时展示断裂）。未配置 `tear` 阈值时
 * 为 no-op（返回 `Bool::TRUE`）。
 *
 * # Returns
 * `Bool::TRUE` 成功触发（含 no-op）；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效。
 */
Bool soft_body_tear_now(struct WorldHandle *world,
                        uint32_t id);

/**
 * Phase 28 — 读回每个质点累积的弹簧/阻尼合力（调试 / 可视化用）。`out_forces` 指向
 * `capacity` 个 `Vec3` 的写缓冲区；第 `i` 项为质点 `i` 的合力（按 `spring_damping_forces`
 * 计算）。缓冲区过小则截断（永不越界）；返回真实的质点数量（不受 capacity 限制）。
 * `out_forces` 为 null 或 `capacity == 0` 仅返回数量不写。
 *
 * # Safety
 * `world` 必须有效；`out_forces`（若非 null）须指向至少 `capacity` 个 `Vec3`。
 */
uint32_t soft_body_read_spring_forces(const struct WorldHandle *world,
                                      uint32_t id,
                                      Vec3 *out_forces,
                                      uint32_t capacity);

/**
 * # Phase 11 — 设置内部气压（充气 / 气球模型）
 *
 * 把 `id` 软体的内部气压设为 `pressure`（力/面积）。每步在 `compute_forces`（MassSpring）
 * 与 `step_xpbd`（预测步）中，对每个**闭合三角网格**的自由质点沿面法向施加向外推力
 * `F = pressure · area`，把闭合壳"吹胀"。`pressure > 0` 开启；`pressure <= 0` 视为关闭
 * （等同于 `pressure = None`，默认）。
 *
 * 纯外力，与风场同构；不引入新求解器力学。需 `self.triangles` 构成闭合流形才能像真气球，
 * 开口薄片会沿单面法向鼓起。
 *
 * # Returns
 * `Bool::TRUE` 成功；`id` 未知 / world 为 null 返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效；`pressure` 需为有限值。
 */
Bool soft_body_set_pressure(struct WorldHandle *world,
                            uint32_t id,
                            double pressure);

/**
 * # Phase 12 — 开启/关闭软体自碰撞(self-collision)
 *
 * 把 `id` 软体的自碰撞设为 `radius`(粒子球半径)+ `stiffness`(XPBD 排斥约束柔度, `0`=硬)。
 * 每步求解中,任意两个自由质点中心距 `< 2*radius` 时沿连线被推开(各自视为该半径的球),
 * 但**直接结构邻居**(已有 distance_constraint 边相连的质点对)被排除,不误判为碰撞。
 * 采用均匀空间哈希做 broad-phase,在 MassSpring 与 XPBD 两条路径内逐迭代投影,纯位置约束,
 * 不引入新求解器力学。非法参数(`radius <= 0` / `stiffness < 0` / 非有限)返回 `Bool::FALSE` 且不开。
 *
 * # Returns
 * `Bool::TRUE` 成功开启;`id` 未知 / world 为 null / 参数非法返回 `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效;`radius` / `stiffness` 需为有限值。
 */
Bool soft_body_set_self_collision(struct WorldHandle *world,
                                  uint32_t id,
                                  double radius,
                                  double stiffness);

/**
 * # Phase 13 — 运行时改单条弹簧刚度
 *
 * 把 `id` 软体里下标 `index`(由 `soft_body_add_spring` 返回)的弹簧刚度(Hookean `k`)改为 `stiffness`。
 * 用于构造后就地调材质异质性(例如把"骨骼"弹簧调硬、"腱"调软),无需重建拓扑。
 * `stiffness < 0` 或非有限 → 返回 `Bool::FALSE` 且不改。
 *
 * # Returns
 * `Bool::TRUE` 修改成功;`id` 未知 / `index` 越界 / world 为 null / 参数非法 → `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效;`stiffness` 需为有限非负值。
 */
Bool soft_body_set_spring_stiffness(struct WorldHandle *world,
                                    uint32_t id,
                                    uint32_t index,
                                    double stiffness);

/**
 * # Phase 13 — 运行时改单条 XPBD 距离约束柔度
 *
 * 把 `id` 软体里下标 `index`(由 `soft_body_add_distance_constraint` 返回)的 XPBD 距离约束柔度
 * (compliance α)改为 `compliance`。XPBD 求解器逐约束读取各自柔度(见 `step_xpbd`),因此不同边
 * 可拥有不同刚度。`compliance < 0` 或非有限 → 返回 `Bool::FALSE` 且不改。
 *
 * # Returns
 * `Bool::TRUE` 修改成功;`id` 未知 / `index` 越界 / world 为 null / 参数非法 → `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效;`compliance` 需为有限非负值。
 */
Bool soft_body_set_distance_constraint_compliance(struct WorldHandle *world,
                                                  uint32_t id,
                                                  uint32_t index,
                                                  double compliance);

/**
 * # Phase 19 — 设置某条距离约束的「压缩」柔度(各向异性柔度)
 *
 * 把 `id` 软体第 `index` 条距离约束的**压缩** XPBD 柔度 `α_c` 设为 `compression`。
 * 该约束原本只有单一 `compliance`(拉伸/压缩共用,各向同性)。本函数令其独立在
 * **压缩**(`len < rest`,被压短)时采用 `compression` 柔度——布料/泡沫可「抗拉伸但易折叠」,
 * 是标准的各向异性 XPBD 行为。`stretch` 柔度仍由 `soft_body_set_distance_constraint_compliance`
 * 控制;二者相等即回到各向同性。求解器每个迭代按当前应变符号选用对应柔度。
 * 非法参数(`index` 越界 / `compression` 为负或非有限)返回 `Bool::FALSE`。
 *
 * # Returns
 * `Bool::TRUE` 成功;`id` 未知 / 约束 `index` 越界 / 参数非法 → `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效;`index` 须在 `[0, 约束数)`;`compression >= 0` 且有限。
 */
Bool soft_body_set_distance_constraint_compression(struct WorldHandle *world,
                                                   uint32_t id,
                                                   uint32_t index,
                                                   double compression);

/**
 * # Phase 14 — 开启/关闭软体间的软软碰撞(soft-soft / cross-body)
 *
 * 把 `id` 软体的软软碰撞设为 `radius`(粒子球半径)+ `stiffness`(XPBD 排斥约束柔度, `0`=硬)。
 * 世界级 step 结束后,任意两个**都**开启了软软碰撞的软体,其自由质点中心距 `< 2·min(ra,rb)`
 * 时沿连线被推开(各自视为该半径的球)。复用 Phase 12 的空间哈希 + XPBD 投影原语,但在
 * world 层遍历软体对。只排 inter-body 对(同体内自碰撞由 Phase 12 处理)。
 * 非法参数(`radius <= 0` / `stiffness < 0` / 非有限)返回 `Bool::FALSE` 且不开。
 *
 * # Returns
 * `Bool::TRUE` 成功开启;`id` 未知 / world 为 null / 参数非法 → `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效;`radius` / `stiffness` 需为有限值。
 */
Bool soft_body_set_cross_collision(struct WorldHandle *world,
                                   uint32_t id,
                                   double radius,
                                   double stiffness);

/**
 * # Phase 20 — 设置自碰撞接触摩擦系数 μ(0 ≤ μ ≤ 1)
 *
 * 需要先 `soft_body_set_self_collision` 开启自碰撞。μ 控制接触处切向相对速度被阻尼的比例
 * (μ=0 无摩擦, μ=1 完全消除切向滑动, Coulomb 风格)。非法参数(非有限 / 越界 / 未开启自碰撞)
 * 返回 `Bool::FALSE` 且不改动状态。
 */
Bool soft_body_set_self_collision_friction(struct WorldHandle *world,
                                           uint32_t id,
                                           double mu);

/**
 * # Phase 20 — 设置软软(跨体)碰撞接触摩擦系数 μ(0 ≤ μ ≤ 1)
 *
 * 需要先 `soft_body_set_cross_collision` 开启跨体碰撞。语义同自碰撞摩擦:阻尼接触切向相对
 * 速度。实际生效的 μ 为两体 `min(μ_a, μ_b)`(任一体无摩擦则该接触无摩擦)。非法参数返回
 * `Bool::FALSE`。
 */
Bool soft_body_set_cross_collision_friction(struct WorldHandle *world,
                                            uint32_t id,
                                            double mu);

/**
 * # Phase 16 — 开启/关闭体积守恒约束(独立柔度, 与距离求解器解耦)
 *
 * 把 `id` 软体的四面体体积约束柔度设为 `compliance`(`0`=硬/不可压缩)。开启后 `step_xpbd`
 * 里每条四面体体积约束用 `α̃ = compliance / dt²` 求解 —— 与距离求解器的 compliance 无关,
 * 因此可以让边很软而体积保持硬(不可压 blob)。与 Phase 11 气压正交:气压是向外吹胀的力,
 * 本约束是把总体积拉回静止值。关闭(`clear`)后体积约束回退到全局求解器 compliance。
 * 非法参数(非有限 / 负数)返回 `Bool::FALSE` 且不开。
 *
 * # Returns
 * `Bool::TRUE` 成功开启;`id` 未知 / world 为 null / 参数非法 → `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效;`compliance` 需为有限非负值。
 */
Bool soft_body_set_volume_conservation(struct WorldHandle *world,
                                       uint32_t id,
                                       double compliance);

/**
 * # Phase 18 — 设置全局内部(结构)阻尼系数
 *
 * 把 `id` 软体的 `damping` 设为 `d`。每个 step 里每个自由质点的速度乘以 `1 - d`
 * (jelly / slime 式能量耗散),与 Phase 0 的弹簧轴向阻尼、Phase 13 的逐约束柔度正交。
 * `d=0` 无阻尼;`d in [0,1)` 振荡收敛更快;`d>=1` 或非法(非有限/负数)返回 `Bool::FALSE`。
 *
 * # Returns
 * `Bool::TRUE` 成功设置;`id` 未知 / world 为 null / 参数非法 → `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效;`d` 需为有限且 `0 <= d < 1`。
 */
Bool soft_body_set_damping(struct WorldHandle *world,
                           uint32_t id,
                           double d);

/**
 * # Phase 17 — 开启/关闭软体间黏连(可撕黏附 glue)
 *
 * 把 `id` 软体的 `cohesion` 设为 `CohesionParams{radius, stiffness, break_distance}`。
 * 开启后,本软体与*其它*也开了 cohesion 的软体之间:自由质点彼此进入 `radius` 即被互相
 * 吸引到接触距离(`radius`),把两体黏在一起(Phase 9 撕裂的对偶)。bond 可破断:若某对
 * 已被拉到 `break_distance` 之外,本步不再吸引(胶水撕裂)。`break_distance=inf` 表示永久胶。
 * 关闭(`clear`)后不再黏连。非法参数(radius<=0 / stiffness<0 / break_distance<=radius /
 * 任一为 NaN)返回 `Bool::FALSE` 且不开;注意 `break_distance=inf` 合法(永久胶)。
 *
 * # Returns
 * `Bool::TRUE` 成功开启;`id` 未知 / world 为 null / 参数非法 → `Bool::FALSE`。
 *
 * # Safety
 * `world` 必须有效;参数需为有限且符合约束。
 */
Bool soft_body_set_cohesion(struct WorldHandle *world,
                            uint32_t id,
                            double radius,
                            double stiffness,
                            double break_distance);

/**
 * Create an empty soft body in the world and return its `SoftBodyId`.
 *
 * The body starts in the `MassSpring` solver; switch it to XPBD with
 * [`soft_body_configure_solver`] if you intend to use distance/tetra constraints.
 *
 * # Returns
 * The `SoftBodyId` (as `u32`) on success, or `u32::MAX` on error (`ERR_*`).
 *
 * # Safety
 * `world` must be a valid world pointer returned by `world_create`.
 */
uint32_t soft_body_create(struct WorldHandle *world, Vec3 gravity);

/**
 * Clone a soft body into a new standalone body, returning the new body id.
 *
 * Deep-copies the source body verbatim — particles (position/velocity/inv_mass),
 * springs, distance constraints, tetrahedra (+ rest volumes), triangles, solver
 * selection, gravity, sleeping, damping, substeps, and every optional field
 * (wind, pressure, tearing, plasticity, self/cross collision, volume
 * conservation, cohesion). The original is untouched.
 *
 * The clone is intentionally **collision-decoupled** (`collide = false`): proxy
 * colliders live in the world's proxy table keyed by `SoftBodyId`, not inside the
 * body, so a copied `collide == true` would have no proxies to drive it and would
 * freeze. Call `soft_body_enable_collision` on the new id to rebuild proxies if the
 * clone needs collision response.
 *
 * Returns the new body id, or `u32::MAX` if `world` is null or `id` is unknown.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
uint32_t soft_body_clone(struct WorldHandle *world, uint32_t id);

/**
 * Add a particle to a soft body.
 *
 * * `mass` — particle mass (> 0, finite). Ignored when `pinned` is non-zero
 *   (a pinned particle has infinite mass / `inv_mass = 0` and acts as an anchor).
 * * `x/y/z` — initial world position (finite).
 *
 * # Returns
 * The particle index (as `u32`) on success, or `u32::MAX` on error (`ERR_*`).
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
uint32_t soft_body_add_particle(struct WorldHandle *world,
                                uint32_t id,
                                double x,
                                double y,
                                double z,
                                double mass,
                                Bool pinned);

/**
 * Add a Hookean spring (edge) between two particles of a soft body.
 *
 * Used by the `MassSpring` solver. Returns `Bool::TRUE` on success.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_add_spring(struct WorldHandle *world,
                          uint32_t id,
                          uint32_t a,
                          uint32_t b,
                          double stiffness,
                          double damping);

/**
 * Add an XPBD distance constraint (edge) between two particles.
 *
 * Used by the `Xpbd` solver; switch the body to XPBD first with
 * [`soft_body_configure_solver`]. Returns `Bool::TRUE` on success.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_add_distance_constraint(struct WorldHandle *world,
                                       uint32_t id,
                                       uint32_t a,
                                       uint32_t b,
                                       double compliance);

/**
 * Add a tetrahedral volume element `[a, b, c, d]` to a soft body.
 *
 * Used by the `Xpbd` solver's volume-preservation constraint; the rest
 * (reference) signed volume is cached at add time. Returns `Bool::TRUE` on
 * success.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_add_tetrahedron(struct WorldHandle *world,
                               uint32_t id,
                               uint32_t a,
                               uint32_t b,
                               uint32_t c,
                               uint32_t d);

/**
 * # Phase 21 - adaptive tetrahedral subdivision (1 -> 4 barycentric split).
 *
 * Inserts one new particle at the centroid of each source tetrahedron and replaces
 * it with four sub-tetrahedra sharing that centroid. The four sub-volumes sum to the
 * parent volume, so the XPBD volume-conservation constraint (Phase 16) stays
 * consistent; the centroid is a vertex of every sub-tet, so no extra distance edges
 * are added (that would over-constrain the solve). A source tet is split only when
 * its longest edge exceeds `max_edge_len`; pass a non-finite value to subdivide all.
 * The shell topology (`triangles`) is left untouched (volumetric refinement only).
 * Returns the number of source tetrahedra actually split (0 if none qualified).
 * Unknown id or a body with no tetrahedra returns 0 with no side effect.
 */
uint32_t soft_body_subdivide_tetrahedra(struct WorldHandle *world,
                                        uint32_t id,
                                        double max_edge_len);

/**
 * Phase 6 — cloth: add a triangular face `[a, b, c]` to a soft body's shell
 * topology. The three structural edges are registered automatically as
 * distance constraints (rest length from current spacing); duplicate edges
 * shared with neighbouring triangles are de-duplicated inside rapier. Bending
 * is composed separately by the caller via `soft_body_add_bending` (a single
 * cross-diagonal distance constraint) — no new mechanics, fully reusing the
 * existing XPBD distance solver.
 *
 * Returns `Bool::TRUE` on success. `Bool::FALSE` if the body/id is unknown, an
 * index is out of bounds or duplicated, or the face is degenerate (a zero-length
 * edge).
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_add_triangle(struct WorldHandle *world,
                            uint32_t id,
                            uint32_t a,
                            uint32_t b,
                            uint32_t c);

/**
 * Phase 6 — cloth: add a single bending edge between particles `p` and `q` as
 * a distance constraint (rest length from current spacing). Compose bending
 * across a quad by calling this for its two diagonals, or across a fold line by
 * linking the un-shared vertices of two adjacent triangles. Reuses the existing
 * XPBD distance solver (no new mechanics).
 *
 * Returns `Bool::TRUE` on success. `Bool::FALSE` if the body/id is unknown, an
 * index is out of bounds, or the endpoints coincide.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_add_bending(struct WorldHandle *world, uint32_t id, uint32_t p, uint32_t q);

/**
 * Switch a soft body's solver.
 *
 * * `solver_mode` — `0` = `MassSpring` (Hookean springs, semi-implicit Euler);
 *   `1` = `Xpbd { iterations, compliance }` (position-based distance + volume
 *   constraints).
 * * `iterations` — XPBD Gauss-Seidel iterations (> 0 when `solver_mode == 1`).
 * * `compliance` — XPBD default compliance (≥ 0, finite).
 *
 * Returns `Bool::TRUE` on success.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_configure_solver(struct WorldHandle *world,
                                uint32_t id,
                                uint32_t solver_mode,
                                uint32_t iterations,
                                double compliance);

/**
 * Build a tetrahedral-mesh soft body from raw particle positions and tetrahedra,
 * then switch it to the XPBD solver so the volume constraints are active.
 *
 * `particles` is a `particles_len`-long array of `Vec3`; `tets` is a flat array
 * of `tets_len * 4` `u32` vertex indices (`[a,b,c,d, a,b,c,d, ...]`). For every
 * tetrahedron, its 6 edges are added as XPBD distance constraints (deduplicated
 * across shared edges). Finally the body is configured with `iterations`/`compliance`.
 *
 * Returns the new `SoftBodyId` (as `u32`) or `u32::MAX` on error.
 *
 * # Safety
 * `world` must be a valid world pointer. `particles`/`tets` must point to arrays
 * of at least `particles_len` / `tets_len*4` elements respectively.
 */
uint32_t soft_body_build_tetra_mesh(struct WorldHandle *world,
                                    Vec3 gravity,
                                    const Vec3 *particles,
                                    uint32_t particles_len,
                                    const uint32_t *tets,
                                    uint32_t tets_len,
                                    double particle_mass,
                                    double compliance,
                                    uint32_t iterations);

/**
 * Build a rope / hair strand soft body from a start point to an end point.
 *
 * * `start_x/y/z` / `end_x/y/z` — the two endpoints of the strand (finite).
 *   The `n` particles are placed at uniform `t = i/(n-1)` interpolation
 *   (`i ∈ [0, n)`), so the strand is straight at rest.
 * * `n` — particle count; must be `>= 2`.
 * * `particle_mass` — mass of each (dynamic) particle (`> 0`, finite).
 * * `compliance` / `iterations` — XPBD stretch parameters for the segment
 *   edges (and the bending edges when `bending != 0`).
 * * `pin_start` / `pin_end` — when non-zero, clamp that endpoint's particle to
 *   infinite mass (anchor). A hanging rope uses `pin_start = 1, pin_end = 0`;
 *   a free strand uses both `0`.
 * * `closed` — when non-zero, the strand is a closed loop: an extra edge links
 *   the last particle back to the first (and, with `bending`, the wrap-around
 *   bending edge too). Useful for necklaces / rings.
 * * `bending` — when non-zero, every adjacent triple gets a bending distance
 *   constraint across its outer particles (rest length from the straight rest
 *   spacing), giving the strand resistance to sharp folding (hair-like).
 *
 * The body is switched to the XPBD solver automatically. Returns the new
 * `SoftBodyId` (as `u32`) or `u32::MAX` on error (`ERR_*`).
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
uint32_t soft_body_build_rope(struct WorldHandle *world,
                              double start_x,
                              double start_y,
                              double start_z,
                              double end_x,
                              double end_y,
                              double end_z,
                              uint32_t n,
                              double particle_mass,
                              double compliance,
                              uint32_t iterations,
                              uint8_t pin_start,
                              uint8_t pin_end,
                              uint8_t closed,
                              uint8_t bending);

/**
 * Build a regular grid / block soft body filling the axis-aligned box
 * `[min_*, max_*]` with `nx × ny × nz` particles spaced uniformly.
 *
 * * `min_*` / `max_*` — box extents (all finite, `max_* > min_*` per axis).
 * * `nx` / `ny` / `nz` — particle counts per axis; each must be `>= 1`.
 *   Total particle count = `nx * ny * nz` (capped to avoid runaway allocation:
 *   rejects if `> 1_000_000`).
 * * `particle_mass` — mass of each (dynamic) particle (`> 0`, finite).
 * * `compliance` / `iterations` — XPBD stretch parameters for the grid edges.
 * * `pin_boundary` — when non-zero, every particle on the outer surface of the
 *   grid (any index at `0` or `n-1` on any axis) is pinned to infinite mass,
 *   so the block hangs/sits from its boundary like a fixed jelly mould.
 *
 * Face-adjacent neighbours (6-connectivity) are linked by XPBD distance
 * constraints (de-duplicated). The body is switched to the XPBD solver
 * automatically. Returns the new `SoftBodyId` (as `u32`) or `u32::MAX` on error.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
uint32_t soft_body_build_grid(struct WorldHandle *world,
                              double min_x,
                              double min_y,
                              double min_z,
                              double max_x,
                              double max_y,
                              double max_z,
                              uint32_t nx,
                              uint32_t ny,
                              uint32_t nz,
                              double particle_mass,
                              double compliance,
                              uint32_t iterations,
                              uint8_t pin_boundary);

/**
 * Number of live soft bodies in the world.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
uint32_t soft_body_count(const struct WorldHandle *world);

/**
 * Number of particles in a soft body. Returns `u32::MAX` for an unknown id.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
uint32_t soft_body_particle_count(const struct WorldHandle *world, uint32_t id);

/**
 * Read back a particle's position and velocity.
 *
 * `out_pos` / `out_vel` must point to writable `Vec3`; either may be null to
 * skip that output. Returns `Bool::TRUE` on success.
 *
 * # Safety
 * `world` must be a valid world pointer; `out_pos`/`out_vel` (if non-null) must
 * point to writable `Vec3`.
 */
Bool soft_body_get_particle(const struct WorldHandle *world,
                            uint32_t id,
                            uint32_t index,
                            Vec3 *out_pos,
                            Vec3 *out_vel);

/**
 * Set a single particle's linear velocity to `(vx, vy, vz)`, overwriting it.
 *
 * Pinned particles (`inv_mass == 0`) are skipped — their velocity is meaningless
 * because the integrator reseeds it from the bound rigid body every step, so this
 * returns `Bool::FALSE` for them. `Err::FALSE` is also returned for a null world,
 * an unknown body id, or an out-of-range `index`. On success the particle's `vel`
 * field is updated in place and `Bool::TRUE` is returned.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_set_particle_velocity(struct WorldHandle *world,
                                     uint32_t id,
                                     uint32_t index,
                                     double vx,
                                     double vy,
                                     double vz);

/**
 * Remove a particle (and every spring / distance constraint / tetrahedron that
 * references it) from a soft body, keeping the remaining topology valid.
 * Returns `Bool::TRUE` on success.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_remove_particle(struct WorldHandle *world, uint32_t id, uint32_t index);

/**
 * Apply a linear impulse to a single soft-body particle.
 *
 * The impulse `J = (fx, fy, fz)` changes the particle velocity by `J * inv_mass`,
 * i.e. `p.vel += J * p.inv_mass`. For collision-coupled bodies the updated velocity
 * is pushed into the particle's proxy rigid body at the next step (see the
 * soft-body/rigid-body coupling loop), so a contact reaction naturally follows; for
 * non-coupled bodies the fork integrator consumes `p.vel` directly. Pinned particles
 * (`inv_mass == 0`, e.g. anchors) are unaffected. This is the primitive for
 * grab/poke/kick interactions on a single vertex. Pure state mutation: no solver
 * structural change.
 *
 * Returns `Bool::TRUE` on success, `Bool::FALSE` if `world` is null, `id` is unknown,
 * `index` is out of bounds, or any component of the impulse is non-finite.
 *
 * # Safety
 * `world` must be a valid world pointer.
 */
Bool soft_body_apply_particle_impulse(struct WorldHandle *world,
                                      uint32_t id,
                                      uint32_t index,
                                      double fx,
                                      double fy,
                                      double fz);

/**
 * Read the axis-aligned bounding box (min/max corners) and centroid of a soft body.
 *
 * Computes the AABB and the per-particle average position (`centroid`) from the
 * body's current particle positions. Useful for frustum culling, broad-phase
 * spatial queries, LOD, and nearest-neighbour tests against other bodies. Pure
 * read-out: does not affect the solver. Bodies with zero particles return
 * `Bool::FALSE` (the box is undefined).
 *
 * Any of `out_min`/`out_max`/`out_centroid` may be null to skip that output.
 *
 * Returns `Bool::TRUE` on success, `Bool::FALSE` if `world` is null, `id` is
 * unknown, or the body has no particles.
 *
 * # Safety
 * `world` must be a valid world pointer; non-null output pointers must each target
 * a writable `Vec3`.
 */
Bool soft_body_read_aabb(const struct WorldHandle *world,
                         uint32_t id,
                         Vec3 *out_min,
                         Vec3 *out_max,
                         Vec3 *out_centroid);

/**
 * Return the exact serialized size (in bytes) of a soft body's state, or
 * `u32::MAX` if `world` is null or `id` is unknown. Allocate a buffer of this
 * size before calling [`soft_body_save_state`].
 */
uint32_t soft_body_state_size(const struct WorldHandle *world, uint32_t id);

/**
 * Serialize a soft body's full state into `out` (capacity `out_capacity` bytes).
 *
 * Returns `Bool::TRUE` on success, or `Bool::FALSE` if `world`/`id` is invalid or
 * the buffer is too small (`ERR_CAPACITY`). Call [`soft_body_state_size`] first to
 * size the buffer. The blob is portable across bodies (feed it to
 * [`soft_body_restore_state`] on the same or a new id).
 */
Bool soft_body_save_state(const struct WorldHandle *world,
                          uint32_t id,
                          uint8_t *out,
                          uint32_t out_capacity);

/**
 * Restore a soft body's full state from `data` (length `data_len` bytes) into the
 * body `id`. The body must already exist (created with [`soft_body_create`]); this
 * replaces its entire state. Returns `Bool::FALSE` on a null world / unknown id /
 * buffer underflow / magic-or-version mismatch (`ERR_INVALID_ARGUMENT`). A corrupt
 * blob never leaves a half-built body — the whole state is built in a temporary
 * first, then swapped in via `get_mut`.
 */
Bool soft_body_restore_state(struct WorldHandle *world,
                             uint32_t id,
                             const uint8_t *data,
                             uint32_t data_len);

Bool soft_body_destroy(struct WorldHandle *world, uint32_t id);

/**
 * 批量读回粒子：位置（world-space）+ 逆质量（0 = pinned）。
 * `out_pos` 容量需 ≥ `capacity` 个 `Vec3`；`out_inv_mass` 容量需 ≥ `capacity` 个 `f64`。
 * 任一出参为 null 即跳过该通道（只写非 null 的通道），但仍返回粒子总数。
 */
uint32_t soft_body_read_particles(const struct WorldHandle *world,
                                  uint32_t id,
                                  Vec3 *out_pos,
                                  double *out_inv_mass,
                                  uint32_t capacity);

/**
 * 批量读回边（弹簧 + 距离约束合并）。每条边是 2 个 `u32` 粒子索引。
 * `out_edges` 容量需 ≥ `capacity` 个 `u32`（即 `capacity/2` 条边）。
 * 边顺序：先所有 springs，再所有 distance_constraints（与 `soft_body_read_tetrahedra`
 * 配合可让渲染层区分软/硬边，若需要）。
 */
uint32_t soft_body_read_edges(const struct WorldHandle *world,
                              uint32_t id,
                              uint32_t *out_edges,
                              uint32_t capacity);

/**
 * 批量读回四面体（XPBD 体积约束单元）。每个四面体是 4 个 `u32` 粒子索引。
 * `out_tets` 容量需 ≥ `capacity` 个 `u32`（即 `capacity/4` 个四面体）。
 */
uint32_t soft_body_read_tetrahedra(const struct WorldHandle *world,
                                   uint32_t id,
                                   uint32_t *out_tets,
                                   uint32_t capacity);

/**
 * Phase 6 — cloth: 批量读回三角形面（shell 拓扑）。每个三角形是 3 个 `u32`
 * 粒子索引。 `out_tris` 容量需 ≥ `capacity` 个 `u32`（即 `capacity/3` 个三角形）。
 * 与 `soft_body_read_edges` 配合可让渲染层区分结构边与弯曲边。
 */
uint32_t soft_body_read_triangles(const struct WorldHandle *world,
                                  uint32_t id,
                                  uint32_t *out_tris,
                                  uint32_t capacity);

/**
 * Phase 27 (B7): exports the soft body's true triangle surface mesh (not the
 * per-particle Ball proxy approximation). Writes up to `vert_cap` vertices (3 f64
 * each) into `out_verts` and up to `tri_cap` triangle indices (3 u32 each) into
 * `out_tris`. Returns the vertex count (so the caller can size its buffers); either
 * buffer may be null to query sizes only. Triangle count comes from
 * `soft_body_read_surface_triangle_count`. This enables mesh-level collision queries
 * (ray-cast, closest-point projection vs static terrain) against the actual surface.
 */
uint32_t soft_body_read_surface_mesh(const struct WorldHandle *world,
                                     uint32_t id,
                                     double *out_verts,
                                     uint32_t vert_cap,
                                     uint32_t *out_tris,
                                     uint32_t tri_cap);

/**
 * Phase 27 (B7): returns the triangle count of a soft body's surface mesh.
 */
uint32_t soft_body_read_surface_triangle_count(const struct WorldHandle *world, uint32_t id);

/**
 * Phase 27 (B8): advances one soft body with the **implicit (backward-Euler) reference
 * integrator** instead of the world's default solver. This is a comparison path:
 * for stiff springs where `step_mass_spring` (explicit) blows up, the implicit step
 * stays bounded. See `SoftBody::step_implicit_euler` (fork) for the linear-system
 * formulation. Returns 0 on success, or an error code if `world`/`id` is invalid.
 */
uint32_t soft_body_step_mass_spring(struct WorldHandle *world, uint32_t id, double dt);

uint32_t soft_body_step_implicit(struct WorldHandle *world, uint32_t id, double dt);

/**
 * Read per-edge normalized strain (stress proxy) for a soft body.
 *
 * Edges are enumerated in the same order as [`soft_body_read_edges`]: every
 * `Spring` first, then every `DistanceConstraint`. For each edge the function
 * writes `strain = (current_len - rest) / rest` (0.0 when `rest == 0`) into
 * `out_strain[..]`. Returns the total edge count (so the caller can size its
 * buffer); when `out_strain` is null or `capacity` is 0 the count is returned
 * without writing.
 *
 * This is a pure read-out for debug visualisation / "tear risk" UI; it does
 * not affect the solver. Symmetry / determinism are irrelevant because no state
 * is mutated.
 *
 * # Safety
 * `world` must be a valid world pointer. `out_strain` must point to an array of
 * at least `capacity` `f64` elements when non-null.
 */
uint32_t soft_body_read_stress(const struct WorldHandle *world,
                               uint32_t id,
                               double *out_strain,
                               uint32_t capacity);

/**
 * Uniformly scale the rest length of every structural edge (springs + XPBD
 * distance constraints) in a soft body by `factor`.
 *
 * This is a one-shot state mutation (not a per-step force): it multiplies each
 * `Spring::rest_length` and `DistanceConstraint::rest` by `factor`. It is the
 * cheap primitive behind "breathing" / "muscle contraction" / "squeeze-stretch"
 * effects — previously users had to retune every edge by hand. `factor` must be
 * strictly positive; a non-positive value returns `ERR_INVALID_ARGUMENT` and
 * touches nothing.
 *
 * Returns the number of edges scaled (springs + distance constraints), or 0 on
 * null world / unknown id / invalid factor.
 */
uint32_t soft_body_scale_rest_length(struct WorldHandle *world, uint32_t id, double factor);

/**
 * Read per-triangle unit normals for a soft body.
 *
 * Triangles are enumerated in the order returned by [`soft_body_read_triangles`].
 * For each triangle `T = (i0, i1, i2)` the function writes the unit normal
 * `(p1 - p0) × (p2 - p0)` normalized into `out_normals[3*k .. 3*k+3]`. Returns
 * the triangle count (so the caller can size its buffer); when `out_normals` is
 * null or `capacity` is 0 the count is returned without writing. Degenerate
 * triangles yield a zero normal.
 *
 * Pure read-out for rendering / debug visualisation; does not affect the solver.
 *
 * # Safety
 * `world` must be a valid world pointer. `out_normals` must point to an array of
 * at least `capacity` `f64` elements when non-null.
 */
uint32_t soft_body_read_normals(const struct WorldHandle *world,
                                uint32_t id,
                                double *out_normals,
                                uint32_t capacity);

/**
 * Read the per-particle net contact force for a collision-coupled soft body.
 *
 * For each free particle that has a proxy collider, the function sums the
 * `ContactPair::total_impulse` over every active contact pair touching that
 * collider, writing the net force vector into `out_fx/out_fy/out_fz[k]`. This is
 * the contact reaction the soft body exerts/feels through its proxy colliders —
 * the primitive behind "step on a soft cushion and get pushed back up" logic.
 *
 * Returns the particle count (so the caller can size its buffer); when `out_fx`
 * is null or `capacity` is 0 the count is returned without writing. Bodies with
 * `collide == false` (no proxies) yield zero force for every particle. Pure
 * read-out: does not affect the solver.
 *
 * # Safety
 * `world` must be a valid world pointer. `out_fx/out_fy/out_fz` must each point
 * to an array of at least `capacity` `f64` elements when non-null.
 */
uint32_t soft_body_read_contact_force(const struct WorldHandle *world,
                                      uint32_t id,
                                      double *out_fx,
                                      double *out_fy,
                                      double *out_fz,
                                      uint32_t capacity);

/**
 * Set the number of solver substeps per `soft_body_step` call for a soft body.
 *
 * `n >= 1` splits the frame `dt` into `n` equal slices; the active solver
 * (XPBD or MassSpring) is run once per slice, projecting constraints at a finer
 * time resolution. Stiff materials and high-compliance edges converge faster
 * and stay stable with more substeps (at `n×` the per-step CPU cost). `n == 0`
 * is rejected and leaves the previous value unchanged.
 *
 * Returns the new substep count, or 0 on null world / unknown id / invalid `n`.
 */
uint32_t soft_body_set_substeps(struct WorldHandle *world, uint32_t id, uint32_t n);

/**
 * Dig out a single voxel cell of a soft body built via `soft_body_voxel_build`,
 * removing the particle that occupies it (plus its incident springs/constraints)
 * and rebuilding the voxel→particle map so further digs stay consistent.
 *
 * Returns `Bool::TRUE` on success. `Bool::FALSE` if the body/id is unknown, the
 * cell is out of bounds, or the cell is already empty/dug.
 *
 * # Safety
 * `world` must be a valid world pointer returned by `world_create`.
 */
Bool soft_body_voxel_dig(struct WorldHandle *world,
                         uint32_t id,
                         uint32_t cell_x,
                         uint32_t cell_y,
                         uint32_t cell_z);

/**
 * Enable or disable rigid-body collision coupling for a soft body.
 *
 * When `enabled` is `Bool::TRUE`, one dynamic `Ball` collider (radius `particle_radius`)
 * is created per free particle and registered in the world's collision-proxy table; the
 * body's `collide` flag is set so the integration layer drives its particles from the
 * proxies. When `Bool::FALSE`, any existing proxies are removed and `collide` is cleared.
 *
 * Returns `Bool::TRUE` on success.
 *
 * # Safety
 * `world` must be a valid world pointer returned by `world_create`.
 */
Bool soft_body_enable_collision(struct WorldHandle *world,
                                uint32_t id,
                                double particle_radius,
                                Bool enabled);

/**
 * Bind a skeleton (a list of rigid bodies acting as bones) to a soft body for
 * linear-blend skinning (LBS). Records each bone's current world transform as its
 * bind pose and precomputes the inverse; per-particle weights are then set with
 * [`soft_body_set_vertex_weights`]. `world_step` applies the skinning every step.
 *
 * * `bone_count` — number of bones (≤ `bones.len()`).
 * * `bones` — pointer to `bone_count` `RigidBodyHandleRaw` (packed `u64`) values.
 *
 * Returns the number of bones bound, or `0` on error.
 *
 * # Safety
 * `world` must be valid; `bones` must point to `bone_count` valid handle words.
 */
uint32_t soft_body_bind_skeleton(struct WorldHandle *world,
                                 uint32_t id,
                                 uint32_t bone_count,
                                 const uint64_t *bones);

/**
 * Bind one soft-body particle to up to 4 bones with linear-blend-skinning
 * weights. Weights need not be normalized; they are normalized here. The
 * particle's current world position is recorded as its rest pose and converted
 * into each bone's bind-pose frame for the per-step skinning.
 *
 * * `bone_indices` — pointer to `4` `u32` bone slots (unused slots ignored).
 * * `weights` — pointer to `4` `f64` weight slots.
 *
 * Returns `Bool::TRUE` on success.
 *
 * # Safety
 * `world` must be valid; `bone_indices`/`weights` must point to 4 elements.
 */
Bool soft_body_set_vertex_weights(struct WorldHandle *world,
                                  uint32_t id,
                                  uint32_t particle_index,
                                  const uint32_t *bone_indices,
                                  const double *weights);

/**
 * # Safety
 * `out_probability` must be null or point to a valid, writable `CollisionProbability`.
 */
Bool space_debris_collision_probability(double miss_distance,
                                        double combined_radius,
                                        double sigma_radial,
                                        double sigma_intrack,
                                        CollisionProbability *out_probability);

/**
 * # Safety
 * `out_rates` must be null or point to a valid, writable `Sgp4SecularRates`.
 */
Bool space_sgp4_j2_secular_rates(double semi_major_axis,
                                 double eccentricity,
                                 double inclination,
                                 double mean_motion,
                                 double equatorial_radius,
                                 double j2,
                                 Sgp4SecularRates *out_rates);

/**
 * Computes the first (base) joint angle of a planar arm from the wrist position.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_arm_first_joint_inverse(double wrist_x, double wrist_y);

/**
 * Computes the third joint angle of a planar arm via the law of cosines.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_arm_third_joint_angle(double planar_radius,
                                   double vertical_offset,
                                   double link2,
                                   double link3,
                                   Bool elbow_up);

/**
 * # Safety
 * `out_command` must be null or point to a valid, writable `Vec3`.
 */
Bool space_artificial_potential_guidance(Vec3 position,
                                         Vec3 target,
                                         Vec3 obstacle,
                                         double attractive_gain,
                                         double repulsive_gain,
                                         double influence_radius,
                                         Vec3 *out_command);

/**
 * # Safety
 * `out_profile` must be null or point to a valid, writable `BangOffBangProfile`.
 */
Bool space_bang_off_bang_profile(double angle,
                                 double max_acceleration,
                                 double max_rate,
                                 BangOffBangProfile *out_profile);

/**
 * # Safety
 * `out_derivative` must be null or point to a valid, writable `CwDerivative`.
 */
Bool space_cw_derivative(CwState state, double mean_motion, CwDerivative *out_derivative);

/**
 * # Safety
 * `out_transform` must be null or point to a valid, writable `DhTransform`.
 */
Bool space_dh_transform(double theta, double d, double a, double alpha, DhTransform *out_transform);

/**
 * Computes the kinetic energy a docking buffer must absorb, scaled by its efficiency.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_docking_buffer_energy(double relative_speed,
                                   double reduced_mass,
                                   double stroke,
                                   double efficiency);

/**
 * Computes a clamped closing-speed command for a docking glideslope.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_docking_glideslope_command(double range,
                                        double desired_slope,
                                        double closing_speed_limit);

/**
 * # Safety
 * `out_derivative` must be null or point to a valid, writable `FlexibleModeDerivative`.
 */
Bool space_flexible_mode_derivative(double displacement,
                                    double velocity,
                                    double natural_frequency,
                                    double damping_ratio,
                                    double modal_force,
                                    double modal_mass,
                                    FlexibleModeDerivative *out_derivative);

/**
 * # Safety
 * `out_dynamics` must be null or point to a valid, writable `ManipulatorDynamics`.
 */
Bool space_manipulator_dynamics_diag(Vec3 mass_matrix_diag,
                                     Vec3 joint_acceleration,
                                     Vec3 coriolis,
                                     Vec3 gravity,
                                     ManipulatorDynamics *out_dynamics);

/**
 * # Safety
 * `out_properties` must be null or point to a valid, writable `MassProperties`.
 */
Bool space_mass_properties_two_body(double mass1,
                                    Vec3 position1,
                                    Vec3 inertia1_diag,
                                    double mass2,
                                    Vec3 position2,
                                    Vec3 inertia2_diag,
                                    MassProperties *out_properties);

/**
 * Computes the absorbed radiation dose including a quality factor.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_radiation_absorbed_dose(double energy_joules, double mass_kg, double quality_factor);

/**
 * # Safety
 * `out_derivative` must be null or point to a valid, writable `SloshPendulumDerivative`.
 */
Bool space_slosh_pendulum_derivative(double angle,
                                     double angular_rate,
                                     double length,
                                     double damping,
                                     double lateral_acceleration,
                                     double gravity,
                                     SloshPendulumDerivative *out_derivative);

/**
 * # Safety
 * `out_derivative` must be null or point to a valid, writable `VariationalState`.
 */
Bool space_variational_two_body(Vec3 position,
                                Vec3 velocity,
                                double mu,
                                VariationalState *out_derivative);

/**
 * # Safety
 * `out_link` must be null or point to a valid, writable `FriisLink`.
 */
Bool space_friis_link(double transmit_power,
                      double transmit_gain,
                      double receive_gain,
                      double wavelength,
                      double range,
                      double system_loss,
                      FriisLink *out_link);

/**
 * Converts a frequency to the corresponding free-space wavelength.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_friis_wavelength_from_frequency(double frequency);

/**
 * Computes the GNSS double-difference carrier phase observable in cycles.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_gnss_double_difference_carrier_phase(double range_rover_sat_a,
                                                  double range_rover_sat_b,
                                                  double range_base_sat_a,
                                                  double range_base_sat_b,
                                                  double wavelength,
                                                  double ambiguity);

/**
 * # Safety
 * `out_observation` must be null or point to a valid, writable `GnssObservation`.
 */
Bool space_gnss_pseudorange(Vec3 receiver,
                            Vec3 satellite,
                            double receiver_clock_bias,
                            double satellite_clock_bias,
                            double ionosphere_delay,
                            double troposphere_delay,
                            GnssObservation *out_observation);

/**
 * # Safety
 * `out_measurement` must be null or point to a valid, writable `RadarMeasurement`.
 */
Bool space_radar_range_rate(Vec3 radar_position,
                            Vec3 target_position,
                            Vec3 radar_velocity,
                            Vec3 target_velocity,
                            RadarMeasurement *out_measurement);

/**
 * # Safety
 * `out_state` must be null or point to a valid, writable `StateVector`.
 */
Bool space_elements_to_state(OrbitalElements elements, double mu, StateVector *out_state);

/**
 * # Safety
 * `out_transfer` must be null or point to a valid, writable `HohmannTransfer`.
 */
Bool space_hohmann_transfer(double mu,
                            double radius1,
                            double radius2,
                            HohmannTransfer *out_transfer);

/**
 * Computes the orbital period from the gravitational parameter and semi-major axis
 * (Kepler's third law).
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_kepler_period(double mu, double semi_major_axis);

/**
 * Computes the semi-major axis from the gravitational parameter and orbital period.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_kepler_semi_major_axis(double mu, double period);

/**
 * Computes the time of flight for an elliptic Lambert arc.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_lambert_time_elliptic(double mu,
                                   double semi_major_axis,
                                   double alpha,
                                   double beta,
                                   uint32_t revolutions);

/**
 * Computes the semi-major axis decay rate due to atmospheric drag.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_semi_major_axis_decay_rate(double semi_major_axis,
                                        double density,
                                        double drag_coefficient,
                                        double area,
                                        double mass,
                                        double mu);

/**
 * # Safety
 * `out_elements` must be null or point to a valid, writable `OrbitalElements`.
 */
Bool space_state_to_elements(StateVector state, double mu, OrbitalElements *out_elements);

/**
 * Computes the Tsiolkovsky rocket equation delta-v.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_tsiolkovsky_delta_v(double specific_impulse,
                                 double standard_gravity,
                                 double initial_mass,
                                 double final_mass);

/**
 * # Safety
 * `world` must be a valid pointer to a `WorldHandle` created by this library.
 * `out_acceleration` must be null or point to a valid, writable `Vec3`.
 */
Bool space_apply_atmospheric_drag_to_body(struct WorldHandle *world,
                                          RigidBodyHandleRaw body_handle,
                                          Vec3 atmosphere_velocity,
                                          double density,
                                          double drag_coefficient,
                                          double area,
                                          double mass,
                                          Bool wake_up,
                                          Vec3 *out_acceleration);

/**
 * # Safety
 * `world` must be a valid pointer to a `WorldHandle` created by this library.
 * `out_acceleration` must be null or point to a valid, writable `Vec3`.
 */
uint8_t space_apply_atmospheric_drag_to_body_flag(struct WorldHandle *world,
                                                  RigidBodyHandleRaw body_handle,
                                                  Vec3 atmosphere_velocity,
                                                  double density,
                                                  double drag_coefficient,
                                                  double area,
                                                  double mass,
                                                  Bool wake_up,
                                                  Vec3 *out_acceleration);

/**
 * # Safety
 * `world` must be a valid pointer to a `WorldHandle` created by this library.
 * `out_torque` must be null or point to a valid, writable `Vec3`.
 */
Bool space_apply_gravity_gradient_torque_to_body(struct WorldHandle *world,
                                                 RigidBodyHandleRaw body_handle,
                                                 Vec3 inertia_diag,
                                                 double mu,
                                                 Bool wake_up,
                                                 Vec3 *out_torque);

/**
 * # Safety
 * `world` must be a valid pointer to a `WorldHandle` created by this library.
 * `out_torque` must be null or point to a valid, writable `Vec3`.
 */
uint8_t space_apply_gravity_gradient_torque_to_body_flag(struct WorldHandle *world,
                                                         RigidBodyHandleRaw body_handle,
                                                         Vec3 inertia_diag,
                                                         double mu,
                                                         Bool wake_up,
                                                         Vec3 *out_torque);

/**
 * # Safety
 * `world` must be a valid pointer to a `WorldHandle` created by this library.
 * `out_acceleration` must be null or point to a valid, writable `Vec3`.
 */
Bool space_apply_j2_force_to_body(struct WorldHandle *world,
                                  RigidBodyHandleRaw body_handle,
                                  double mu,
                                  double equatorial_radius,
                                  double j2,
                                  double mass,
                                  Bool wake_up,
                                  Vec3 *out_acceleration);

/**
 * # Safety
 * `world` must be a valid pointer to a `WorldHandle` created by this library.
 * `out_acceleration` must be null or point to a valid, writable `Vec3`.
 */
uint8_t space_apply_j2_force_to_body_flag(struct WorldHandle *world,
                                          RigidBodyHandleRaw body_handle,
                                          double mu,
                                          double equatorial_radius,
                                          double j2,
                                          double mass,
                                          Bool wake_up,
                                          Vec3 *out_acceleration);

/**
 * # Safety
 * `world` must be a valid pointer to a `WorldHandle` created by this library.
 * `out_acceleration` must be null or point to a valid, writable `Vec3`.
 */
Bool space_apply_solar_radiation_pressure_to_body(struct WorldHandle *world,
                                                  RigidBodyHandleRaw body_handle,
                                                  Vec3 sun_direction,
                                                  double solar_flux,
                                                  double reflectivity,
                                                  double area,
                                                  double mass,
                                                  Bool wake_up,
                                                  Vec3 *out_acceleration);

/**
 * # Safety
 * `world` must be a valid pointer to a `WorldHandle` created by this library.
 * `out_acceleration` must be null or point to a valid, writable `Vec3`.
 */
uint8_t space_apply_solar_radiation_pressure_to_body_flag(struct WorldHandle *world,
                                                          RigidBodyHandleRaw body_handle,
                                                          Vec3 sun_direction,
                                                          double solar_flux,
                                                          double reflectivity,
                                                          double area,
                                                          double mass,
                                                          Bool wake_up,
                                                          Vec3 *out_acceleration);

/**
 * Computes atmospheric density using the exponential scale-height model.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_atmospheric_density_scale_height(double reference_density,
                                              double altitude,
                                              double reference_altitude,
                                              double scale_height);

/**
 * # Safety
 * `out_acceleration` must be null or point to a valid, writable `Vec3`.
 */
Bool space_atmospheric_drag_acceleration(Vec3 velocity,
                                         Vec3 atmosphere_velocity,
                                         double density,
                                         double drag_coefficient,
                                         double area,
                                         double mass,
                                         Vec3 *out_acceleration);

/**
 * # Safety
 * `out_erosion` must be null or point to a valid, writable `AtomicOxygenErosion`.
 */
Bool space_atomic_oxygen_erosion(double fluence,
                                 double erosion_yield,
                                 double area,
                                 double density,
                                 AtomicOxygenErosion *out_erosion);

/**
 * # Safety
 * `out_torque` must be null or point to a valid, writable `Vec3`.
 */
Bool space_gravity_gradient_torque(Vec3 position, Vec3 inertia_diag, double mu, Vec3 *out_torque);

/**
 * # Safety
 * `out_acceleration` must be null or point to a valid, writable `Vec3`.
 */
Bool space_j2_acceleration(Vec3 position,
                           double mu,
                           double equatorial_radius,
                           double j2,
                           Vec3 *out_acceleration);

/**
 * Computes the Sagnac phase rate of a ring interferometer.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_sagnac_phase_rate(double area, double angular_rate, double wavelength);

/**
 * # Safety
 * `out_acceleration` must be null or point to a valid, writable `Vec3`.
 */
Bool space_solar_radiation_pressure_acceleration(Vec3 sun_direction,
                                                 double solar_flux,
                                                 double reflectivity,
                                                 double area,
                                                 double mass,
                                                 Vec3 *out_acceleration);

/**
 * # Safety
 * `out_battery` must be null or point to a valid, writable `BatteryEquivalentCircuit`.
 */
Bool space_battery_equivalent_circuit(double open_circuit_voltage,
                                      double current,
                                      double ohmic_resistance,
                                      double rc_voltage,
                                      double rc_resistance,
                                      double rc_capacitance,
                                      double capacity_coulombs,
                                      BatteryEquivalentCircuit *out_battery);

/**
 * # Safety
 * `out_balance` must be null or point to a valid, writable `Co2MassBalance`.
 */
Bool space_co2_mass_balance(double current_mass,
                            double generation_rate,
                            double removal_rate,
                            double leakage_rate,
                            double volume,
                            double dt,
                            Co2MassBalance *out_balance);

/**
 * # Safety
 * `out_force` must be null or point to a valid, writable `ContactForceModel`.
 */
Bool space_contact_force_hunt_crossley(double penetration,
                                       double penetration_rate,
                                       double stiffness,
                                       double damping,
                                       double exponent,
                                       ContactForceModel *out_force);

/**
 * # Safety
 * `out_performance` must be null or point to a valid, writable `HallThrusterPerformance`.
 */
Bool space_hall_thruster_performance(double mass_flow_rate,
                                     double exhaust_velocity,
                                     double input_power,
                                     double standard_gravity,
                                     HallThrusterPerformance *out_performance);

/**
 * # Safety
 * `out_rate` must be null or point to a valid, writable `ChemicalReactionRate`.
 */
Bool space_sabatier_methane_rate(double co2_molar_rate,
                                 double h2_molar_rate,
                                 double conversion,
                                 ChemicalReactionRate *out_rate);

/**
 * # Safety
 * `out_power` must be null or point to a valid, writable `SolarPanelPower`.
 */
Bool space_solar_panel_power(double solar_flux,
                             double area,
                             double efficiency,
                             double incidence_angle,
                             double degradation,
                             SolarPanelPower *out_power);

/**
 * Computes a structural natural frequency from stiffness, mass, and a mode factor.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_structural_natural_frequency(double stiffness, double mass, double mode_factor);

/**
 * # Safety
 * `world` must be a valid pointer to a `WorldHandle` created by this library.
 * `out_exchange` must be null or point to a valid, writable `CmgExchange`.
 */
Bool space_apply_cmg_torque_to_body(struct WorldHandle *world,
                                    RigidBodyHandleRaw body_handle,
                                    Vec3 gimbal_axis,
                                    Vec3 wheel_momentum,
                                    double gimbal_rate,
                                    Bool wake_up,
                                    CmgExchange *out_exchange);

/**
 * # Safety
 * `world` must be a valid pointer to a `WorldHandle` created by this library.
 * `out_exchange` must be null or point to a valid, writable `CmgExchange`.
 */
uint8_t space_apply_cmg_torque_to_body_flag(struct WorldHandle *world,
                                            RigidBodyHandleRaw body_handle,
                                            Vec3 gimbal_axis,
                                            Vec3 wheel_momentum,
                                            double gimbal_rate,
                                            Bool wake_up,
                                            CmgExchange *out_exchange);

/**
 * # Safety
 * `world` must be a valid pointer to a `WorldHandle` created by this library.
 * `out_dipole` must be null or point to a valid, writable `Vec3`.
 */
Bool space_apply_magnetic_torquer_to_body(struct WorldHandle *world,
                                          RigidBodyHandleRaw body_handle,
                                          Vec3 commanded_torque,
                                          Vec3 magnetic_field,
                                          double max_dipole,
                                          Bool wake_up,
                                          Vec3 *out_dipole);

/**
 * # Safety
 * `world` must be a valid pointer to a `WorldHandle` created by this library.
 * `out_dipole` must be null or point to a valid, writable `Vec3`.
 */
uint8_t space_apply_magnetic_torquer_to_body_flag(struct WorldHandle *world,
                                                  RigidBodyHandleRaw body_handle,
                                                  Vec3 commanded_torque,
                                                  Vec3 magnetic_field,
                                                  double max_dipole,
                                                  Bool wake_up,
                                                  Vec3 *out_dipole);

/**
 * # Safety
 * `out_exchange` must be null or point to a valid, writable `CmgExchange`.
 */
Bool space_cmg_exchange(Vec3 gimbal_axis,
                        Vec3 wheel_momentum,
                        double gimbal_rate,
                        CmgExchange *out_exchange);

/**
 * # Safety
 * `out_inverse` must be null or point to a valid, writable `CmgRobustInverse`.
 */
Bool space_cmg_robust_pseudoinverse_diag(Vec3 jacobian_diag,
                                         Vec3 desired_torque,
                                         double damping,
                                         CmgRobustInverse *out_inverse);

/**
 * Computes the scalar Kalman gain.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_ekf_gain_scalar(double covariance,
                             double measurement_jacobian,
                             double measurement_noise);

/**
 * # Safety
 * `out_prediction` must be null or point to a valid, writable `ScalarKalman`.
 */
Bool space_ekf_predict_scalar(double state,
                              double covariance,
                              double nonlinear_delta,
                              double jacobian,
                              double process_noise,
                              ScalarKalman *out_prediction);

/**
 * # Safety
 * `out_update` must be null or point to a valid, writable `ScalarKalman`.
 */
Bool space_ekf_update_scalar(double predicted_state,
                             double predicted_covariance,
                             double measurement,
                             double predicted_measurement,
                             double kalman_gain,
                             double measurement_jacobian,
                             ScalarKalman *out_update);

/**
 * # Safety
 * `out_attitude` must be null or point to a valid, writable `LeastSquaresAttitude`.
 */
Bool space_least_squares_attitude_two_vector(Vec3 body_primary,
                                             Vec3 body_secondary,
                                             Vec3 reference_primary,
                                             Vec3 reference_secondary,
                                             LeastSquaresAttitude *out_attitude);

/**
 * # Safety
 * `out_dipole` must be null or point to a valid, writable `Vec3`.
 */
Bool space_magnetic_torquer_dipole(Vec3 commanded_torque,
                                   Vec3 magnetic_field,
                                   double max_dipole,
                                   Vec3 *out_dipole);

/**
 * # Safety
 * `out_derivative` must be null or point to a valid, writable `QuaternionDerivative`.
 */
Bool space_quaternion_derivative(Quat attitude,
                                 Vec3 angular_velocity,
                                 QuaternionDerivative *out_derivative);

/**
 * # Safety
 * `out_derivative` must be null or point to a valid, writable `RigidBodyEulerDerivative`.
 */
Bool space_rigid_body_euler_derivative(Vec3 inertia_diag,
                                       Vec3 angular_velocity,
                                       Vec3 torque,
                                       RigidBodyEulerDerivative *out_derivative);

/**
 * Computes the PD control torque for a solar array drive.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_solar_array_pd_torque(double angle_error, double rate_error, double kp, double kd);

/**
 * Computes the net spacecraft surface charging current balance.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_surface_charging_current_balance(double photo_current,
                                              double secondary_current,
                                              double backscatter_current,
                                              double electron_current,
                                              double ion_current);

/**
 * # Safety
 * `out_attitude` must be null or point to a valid, writable `Quat`.
 */
Bool space_triad_attitude(Vec3 body_primary,
                          Vec3 body_secondary,
                          Vec3 reference_primary,
                          Vec3 reference_secondary,
                          Quat *out_attitude);

/**
 * # Safety
 * `out_state` must be null or point to a valid, writable `AirlockDepressurization`.
 */
Bool space_airlock_depressurization(double pressure,
                                    double ambient_pressure,
                                    double volume,
                                    double conductance,
                                    double dt,
                                    AirlockDepressurization *out_state);

/**
 * Sums the evaporator, vapor, condenser, and wick thermal resistances of a heat pipe.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_heat_pipe_thermal_resistance(double evaporator_resistance,
                                          double vapor_resistance,
                                          double condenser_resistance,
                                          double wick_resistance);

/**
 * # Safety
 * `out_power` must be null or point to a valid, writable `RadiatorPower`.
 */
Bool space_radiator_power(double area,
                          double emissivity,
                          double temperature,
                          double sink_temperature,
                          double absorbed_power,
                          RadiatorPower *out_power);

/**
 * # Safety
 * `out_heat` must be null or point to a valid, writable `FluidLoopHeatTransfer`.
 */
Bool space_single_phase_loop_heat_transfer(double mass_flow_rate,
                                           double specific_heat,
                                           double inlet_temperature,
                                           double heat_input,
                                           FluidLoopHeatTransfer *out_heat);

/**
 * # Safety
 * `out_rate` must be null or point to a valid, writable `ChemicalReactionRate`.
 */
Bool space_spe_oxygen_rate(double current,
                           double cells,
                           double faraday_efficiency,
                           ChemicalReactionRate *out_rate);

/**
 * # Safety
 * `out_balance` must be null or point to a valid, writable `ThermalBalance`.
 */
Bool space_thermal_balance(double absorbed_power,
                           double internal_power,
                           double emitted_area,
                           double emissivity,
                           ThermalBalance *out_balance);

/**
 * Computes the critical projectile diameter a Whipple shield can defeat.
 *
 * # Safety
 * This function takes no pointers and transfers no ownership; it is safe to call with
 * any argument values. Invalid inputs return `f64::NAN` and set `ERR_INVALID_ARGUMENT`.
 */
double space_whipple_critical_projectile_diameter(double bumper_thickness,
                                                  double bumper_density,
                                                  double projectile_density,
                                                  double impact_velocity,
                                                  double standoff);

/**
 * Compute polyhedron gravity.
 *
 * `vertices_xyz` — flat array of vertex positions (3×n_verts f64s)
 * `face_indices` — flat array of triangle indices (3×n_faces u32s)
 * `density` — constant density (kg/m³)
 *
 * # Safety
 *
 * `vertices_xyz` must point to at least 3×n_vertices readable f64s and
 * `face_indices` to at least 3×n_faces readable u32s; `out_acceleration`
 * must be valid for a single `Vec3` write.
 */
Bool terrain_polyhedron_gravity(Vec3 position,
                                const double *vertices_xyz,
                                uint32_t n_vertices,
                                const uint32_t *face_indices,
                                uint32_t n_faces,
                                double density,
                                Vec3 *out_acceleration);

/**
 * Compute terrain gravity from DEM (direct summation method).
 *
 * # Safety
 *
 * `dem` must point to at least nx×ny readable f64s; `out_acceleration` must
 * be valid for a single `Vec3` write.
 */
Bool terrain_gravity_dem(Vec3 position,
                         const double *dem,
                         uint32_t nx,
                         uint32_t ny,
                         double resolution,
                         double reference_radius,
                         double surface_density,
                         Vec3 *out_acceleration);

/**
 * Compute terrain gravity from DEM (FFT/quadrupole approximation).
 *
 * # Safety
 *
 * `dem` must point to at least nx×ny readable f64s; `out_acceleration` must
 * be valid for a single `Vec3` write.
 */
Bool terrain_gravity_dem_fft(Vec3 position,
                             const double *dem,
                             uint32_t nx,
                             uint32_t ny,
                             double resolution,
                             double reference_radius,
                             double surface_density,
                             Vec3 *out_acceleration);

/**
 * Compute lunar mascon gravitational acceleration.
 *
 * # Safety
 *
 * `out_acceleration` must be valid for a single `Vec3` write.
 */
Bool terrain_lunar_mascon_gravity(Vec3 position, Vec3 *out_acceleration);

/**
 * Get the number of built-in lunar mascons.
 *
 * # Safety
 *
 * This function takes no pointers and performs no memory access; it is safe
 * to call from any context.
 */
uint32_t terrain_lunar_mascon_count(void);

/**
 * Get a specific lunar mascon by index.
 *
 * # Safety
 *
 * `out_mascon` must be valid for a single `LunarMascon` write.
 */
Bool terrain_lunar_mascon_get(uint32_t index, struct LunarMascon *out_mascon);

/**
 * Create a tire model for a vehicle controller.
 *
 * Returns a stable id, or `u32::MAX` on error.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
uint32_t tire_model_create(struct WorldHandle *world, uint32_t vehicle_id, uint32_t wheel_count);

/**
 * Set tire parameters for a specific wheel.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool tire_model_set_params(struct WorldHandle *world,
                           uint32_t id,
                           uint32_t wheel_index,
                           double peak_mu_long,
                           double peak_mu_lat,
                           double peak_slip_ratio,
                           double peak_slip_angle,
                           double load_sensitivity,
                           double ellipse_factor);

/**
 * Compute tire forces based on current wheel state.
 *
 * This should be called each frame **after** `vehicle_controller_update` so
 * the wheel transforms (steering, world-space axle, suspension force) are
 * fresh. The computed forces are stored per wheel; read them with
 * `tire_model_get_forces` and apply them to the chassis via the rigid-body
 * impulse FFI.
 *
 * Returns `true` on success.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool tire_model_update(struct WorldHandle *world, uint32_t id, double dt);

/**
 * Get the computed tire forces for a specific wheel.
 *
 * # Safety
 *
 * `world` must be a valid world pointer. `out_fx` and `out_fy` must be valid pointers.
 */
Bool tire_model_get_forces(struct WorldHandle *world,
                           uint32_t id,
                           uint32_t wheel_index,
                           double *out_fx,
                           double *out_fy);

/**
 * Remove a tire model from the world.
 *
 * # Safety
 *
 * `world` must be a valid world pointer.
 */
Bool tire_model_remove(struct WorldHandle *world, uint32_t id);

Bool acoustics_spherical_spreading_loss(double range, double *out);

Bool acoustics_cylindrical_spreading_loss(double range, double *out);

Bool acoustics_thorp_absorption(double frequency_khz, double *out);

Bool acoustics_sabine_rt60(double volume, double surface_area, double mean_absorption, double *out);

Bool acoustics_eyring_rt60(double volume, double surface_area, double mean_absorption, double *out);

Bool acoustics_acoustic_impedance(double density, double sound_speed, double *out);

Bool acoustics_transmission_coefficient(double z1, double z2, double *out);

Bool acoustics_mass_law_tl(double frequency, double surface_density, double *out);

Bool acoustics_helmholtz_resonance_frequency(double sound_speed,
                                             double neck_area,
                                             double cavity_volume,
                                             double neck_length,
                                             double *out);

Bool acoustics_doppler_shift(double source_frequency,
                             double sound_speed,
                             double receiver_velocity,
                             double source_velocity,
                             Bool approach,
                             double *out);

Bool acoustics_maekawa_barrier_attenuation(double fresnel_number, double *out);

Bool acoustics_active_sonar_echo_level(double source_level,
                                       double transmission_loss,
                                       double target_strength,
                                       double noise_level,
                                       double directivity_index,
                                       double detection_threshold,
                                       double *out);

Bool astrophysics_hill_sphere_radius(double primary_mass,
                                     double secondary_mass,
                                     double semi_major_axis,
                                     double eccentricity,
                                     double *out);

Bool astrophysics_lane_emden_first_zero(double polytropic_index, double *out);

Bool astrophysics_mass_luminosity_relation(double mass_solar, double exponent, double *out);

Bool astrophysics_eddington_luminosity(double mass, double opacity, double *out);

Bool astrophysics_eddington_luminosity_solar(double mass_solar, double opacity, double *out);

Bool astrophysics_hubble_velocity(double hubble_constant, double distance, double *out);

Bool astrophysics_hubble_distance(double velocity, double hubble_constant, double *out);

Bool astrophysics_nfw_density(double radius,
                              double scale_radius,
                              double characteristic_density,
                              double *out);

Bool astrophysics_nfw_enclosed_mass(double radius,
                                    double scale_radius,
                                    double characteristic_density,
                                    double *out);

Bool astrophysics_blackbody_spectral_radiance(double wavelength, double temperature, double *out);

Bool astrophysics_wien_displacement(double temperature, double *out);

Bool astrophysics_jeans_mass(double temperature,
                             double density,
                             double mean_molecular_weight,
                             double *out);

Bool astrophysics_jeans_length(double temperature,
                               double density,
                               double mean_molecular_weight,
                               double *out);

Bool astrophysics_main_sequence_lifetime(double mass_solar, double *out);

Bool astrophysics_mass_radius_relation(double mass_solar, double *out);

Bool astrophysics_chandrasekhar_mass_limit(double *out);

Bool astrophysics_chandrasekhar_mass_kg(double *out);

Bool astrophysics_mass_function(double period_seconds, double semi_amplitude, double *out);

Bool astrophysics_binary_semi_major_axis(double total_mass, double period, double *out);

Bool astrophysics_ss73_disk_temperature(double mass_kg,
                                        double accretion_rate,
                                        double radius,
                                        double inner_radius,
                                        double *out);

Bool astrophysics_nickel56_decay_luminosity(double nickel_mass_kg, double time_days, double *out);

Bool astrophysics_transit_depth(double planet_radius, double star_radius, double *out);

Bool astrophysics_radial_velocity_semi_amplitude(double planet_mass_kg,
                                                 double star_mass_kg,
                                                 double period,
                                                 double inclination,
                                                 double *out);

Bool astrophysics_nfw_circular_velocity(double r, double v_max, double r_scale, double *out);

/**
 * Roche fluid/rigid limits. Writes (fluid, rigid) into `out_fluid` /
 * `out_rigid`. Returns `Bool::FALSE` on invalid input or a null output.
 */
Bool astrophysics_roche_limit(double primary_radius,
                              double primary_density,
                              double secondary_density,
                              double *out_fluid,
                              double *out_rigid);

/**
 * Habitable-zone inner/outer radii. Writes (inner, outer) into `out_inner` /
 * `out_outer`. Returns `Bool::FALSE` on invalid input or a null output.
 */
Bool astrophysics_habitable_zone_boundaries(double star_luminosity_solar,
                                            double *out_inner,
                                            double *out_outer);

Bool electromagnetism_poynting_magnitude_plane_wave(double e_field_magnitude, double *out);

Bool electromagnetism_phase_velocity(double refractive_index, double *out);

Bool electromagnetism_wavelength_in_medium(double frequency, double refractive_index, double *out);

Bool electromagnetism_intrinsic_impedance(double permeability, double permittivity, double *out);

Bool electromagnetism_skin_depth(double frequency,
                                 double permeability,
                                 double conductivity,
                                 double *out);

Bool electromagnetism_vacuum_wavelength(double frequency, double *out);

Bool electromagnetism_wave_frequency(double wavelength, double *out);

Bool electromagnetism_dipole_radiation_resistance(double dipole_length,
                                                  double wavelength,
                                                  double *out);

Bool electromagnetism_half_wave_dipole_directivity(double *out);

Bool electromagnetism_effective_aperture(double gain_linear, double wavelength, double *out);

Bool electromagnetism_far_field_distance(double antenna_size, double wavelength, double *out);

Bool electromagnetism_friis_power_received(double transmit_power,
                                           double tx_gain,
                                           double rx_gain,
                                           double wavelength,
                                           double range,
                                           double *out);

Bool electromagnetism_reflection_coefficient(double load_impedance,
                                             double characteristic_impedance,
                                             double *out);

Bool electromagnetism_vswr(double reflection_coeff, double *out);

Bool electromagnetism_return_loss(double reflection_coeff, double *out);

Bool electromagnetism_quarter_wave_transformer(double z0, double z_load, double *out);

Bool electromagnetism_coaxial_impedance(double inner_diameter,
                                        double outer_diameter,
                                        double relative_permittivity,
                                        double *out);

Bool electromagnetism_coaxial_cutoff_frequency(double inner_diameter,
                                               double outer_diameter,
                                               double relative_permittivity,
                                               double *out);

Bool electromagnetism_rayleigh_scattering_cross_section(double refractive_index,
                                                        double diameter,
                                                        double wavelength,
                                                        double *out);

Bool electromagnetism_faraday_rotation(double verdet_constant,
                                       double magnetic_field,
                                       double path_length,
                                       double *out);

/**
 * Transmission-line input impedance (lossless). Writes (real, imag) into
 * `out_real` / `out_imag`. Returns `Bool::FALSE` on invalid input or a null output.
 */
Bool electromagnetism_transmission_line_input_impedance(double z0,
                                                        double z_load_real,
                                                        double z_load_imag,
                                                        double phase_constant,
                                                        double length,
                                                        double *out_real,
                                                        double *out_imag);

Bool material_mechanics_hookes_law_uniaxial(double stress, double youngs_modulus, double *out);

Bool material_mechanics_stress_from_strain(double youngs_modulus, double strain, double *out);

Bool material_mechanics_shear_modulus(double youngs_modulus, double poisson_ratio, double *out);

Bool material_mechanics_bulk_modulus(double youngs_modulus, double poisson_ratio, double *out);

Bool material_mechanics_lame_lambda(double youngs_modulus, double poisson_ratio, double *out);

Bool material_mechanics_von_mises_stress(double sx,
                                         double sy,
                                         double sz,
                                         double txy,
                                         double tyz,
                                         double tzx,
                                         double *out);

Bool material_mechanics_von_mises_yield_check(double von_mises_stress,
                                              double yield_stress,
                                              double *out);

Bool material_mechanics_tresca_shear_stress(double sigma_1, double sigma_3, double *out);

Bool material_mechanics_tresca_yield_check(double sigma_1,
                                           double sigma_3,
                                           double yield_stress,
                                           double *out);

Bool material_mechanics_ki_center_crack(double stress, double crack_half_length, double *out);

Bool material_mechanics_ki_edge_crack(double stress, double crack_length, double *out);

Bool material_mechanics_fracture_check(double stress_intensity,
                                       double fracture_toughness,
                                       double *out);

Bool material_mechanics_critical_crack_length(double stress,
                                              double fracture_toughness,
                                              double *out);

Bool material_mechanics_basquin_stress_amplitude(double cycles_to_failure,
                                                 double fatigue_strength_coefficient,
                                                 double fatigue_exponent,
                                                 double *out);

Bool material_mechanics_basquin_cycles_to_failure(double stress_amplitude,
                                                  double fatigue_strength_coefficient,
                                                  double fatigue_exponent,
                                                  double *out);

Bool material_mechanics_coffin_manson_strain_amplitude(double cycles_to_failure,
                                                       double ductility_coefficient,
                                                       double ductility_exponent,
                                                       double *out);

Bool material_mechanics_goodman_correction(double stress_amplitude,
                                           double mean_stress,
                                           double ultimate_tensile,
                                           double *out);

Bool material_mechanics_norton_creep_rate(double stress,
                                          double temperature,
                                          double a,
                                          double n,
                                          double activation_energy,
                                          double gas_constant,
                                          double *out);

Bool material_mechanics_beam_bending_stress(double bending_moment,
                                            double distance_from_neutral_axis,
                                            double area_moment_of_inertia,
                                            double *out);

Bool material_mechanics_beam_deflection_center_point_load(double load,
                                                          double span,
                                                          double youngs_modulus,
                                                          double moment_of_inertia,
                                                          double *out);

Bool material_mechanics_euler_buckling_load(double youngs_modulus,
                                            double moment_of_inertia,
                                            double effective_length_factor,
                                            double column_length,
                                            double *out);

Bool material_mechanics_slenderness_ratio(double effective_length_factor,
                                          double column_length,
                                          double radius_of_gyration,
                                          double *out);

/**
 * Principal stresses from a 3D stress tensor. Writes (σ₁, σ₂, σ₃) sorted
 * descending into `out` (capacity must be ≥ 3). Returns `Bool::FALSE` on
 * invalid input or null/short `out`.
 */
Bool material_mechanics_principal_stresses(double sx,
                                           double sy,
                                           double sz,
                                           double txy,
                                           double tyz,
                                           double tzx,
                                           double *out);

/**
 * Miner's linear damage rule: D = Σ (nᵢ / N_fᵢ). `ratios` points to
 * `count` `f64` elements (each nᵢ/N_fᵢ). Writes the summed damage into `out`.
 * Returns `Bool::FALSE` on null pointers, empty/short input, or invalid data.
 */
Bool material_mechanics_miners_damage(const double *ratios, uint32_t count, double *out);

Bool nuclear_decay_constant(double half_life, double *out);

Bool nuclear_remaining_nuclei(double initial, double decay_constant, double time, double *out);

Bool nuclear_activity(double decay_constant, double nuclei, double *out);

Bool nuclear_half_life(double decay_constant, double *out);

Bool nuclear_mean_lifetime(double decay_constant, double *out);

Bool nuclear_bethe_weizsaecker_binding_energy(double mass_number,
                                              double atomic_number,
                                              double *out);

Bool nuclear_binding_energy_per_nucleon(double mass_number, double atomic_number, double *out);

Bool nuclear_reaction_q_value(double initial_mass_u, double final_mass_u, double *out);

Bool nuclear_dt_fusion_energy(double *out);

Bool nuclear_dd_fusion_branch1_energy(double *out);

Bool nuclear_dd_fusion_branch2_energy(double *out);

Bool nuclear_u235_fission_energy(double *out);

Bool nuclear_four_factor_formula(double eta, double epsilon, double p, double f, double *out);

Bool nuclear_reaction_rate(double macroscopic_cross_section, double neutron_flux, double *out);

Bool nuclear_atomic_mass_approx(double mass_number, double binding_energy_mev, double *out);

Bool nuclear_specific_activity(double decay_constant, double mass_number, double *out);

Bool nuclear_half_value_layer(double linear_attenuation, double *out);

Bool nuclear_dt_fusion_q_value(double *out);

Bool plasma_beta(double density, double temperature, double magnetic_field, double *out);

Bool plasma_gyrofrequency(double charge, double magnetic_field, double mass, double *out);

Bool plasma_larmor_radius(double mass,
                          double perpendicular_velocity,
                          double charge,
                          double magnetic_field,
                          double *out);

Bool plasma_mirror_ratio(double max_field, double min_field, double *out);

Bool plasma_mirror_loss_cone_angle(double max_field, double min_field, double *out);

Bool quantum_free_particle_energy(double wave_number, double mass, double *out);

Bool quantum_de_broglie_wavelength(double mass, double velocity, double *out);

Bool quantum_infinite_well_energy(uint32_t quantum_number,
                                  double mass,
                                  double well_width,
                                  double *out);

Bool quantum_infinite_well_wave_function(uint32_t quantum_number,
                                         double well_width,
                                         double x,
                                         double *out);

Bool quantum_bohr_radius(double *out);

Bool quantum_hydrogen_energy_level(uint32_t quantum_number, double *out);

Bool quantum_hydrogen_orbital_radius(uint32_t quantum_number, double *out);

Bool quantum_hydrogen_transition_wavelength(uint32_t n1, uint32_t n2, double *out);

Bool quantum_minimum_uncertainty_product(double *out);

Bool quantum_fermi_golden_rule_linear(double matrix_element2,
                                      double density_of_states,
                                      double *out);

Bool quantum_spin_orbit_energy(double n, double l, double j, double atomic_number, double *out);

Bool quantum_fine_structure_constant(double *out);

Bool quantum_variational_hydrogen_energy(double alpha, double *out);

Bool quantum_variational_hydrogen_optimal_alpha(double *out);

Bool quantum_coherent_state_photon_probability(double alpha_squared, uint32_t n, double *out);

Bool quantum_spherical_harmonic_real(int32_t l, int32_t m, double theta, double phi, double *out);

Bool quantum_angular_momentum_squared(double j, double *out);

Bool quantum_photoelectric_threshold(double work_function, double *out);

Bool quantum_photoelectric_max_kinetic(double frequency, double work_function, double *out);

Bool quantum_compton_wavelength_shift(double scattering_angle, double *out);

Bool quantum_compton_scattered_wavelength(double lambda, double scattering_angle, double *out);

Bool quantum_rabi_oscillation_probability(double rabi_frequency,
                                          double detuning,
                                          double time,
                                          double *out);

Bool quantum_landau_level(int32_t quantum_number,
                          double magnetic_field,
                          double charge,
                          double mass,
                          double *out);

Bool quantum_einstein_a_coefficient(double transition_frequency, double dipole_moment, double *out);

Bool quantum_clebsch_gordan_allowed(double j1,
                                    double j2,
                                    double j3,
                                    double m1,
                                    double m2,
                                    double m3,
                                    double *out);

/**
 * Degenerate 2×2 perturbation eigenvalues. Writes (λ₁, λ₂) into `out_e1` /
 * `out_e2`. Returns `Bool::FALSE` on invalid input or a null output.
 */
Bool quantum_degenerate_perturbation_2x2(double h11,
                                         double h12,
                                         double h22,
                                         double *out_e1,
                                         double *out_e2);

/**
 * Time-evolution phase factor e^{-iEt/ℏ}. Writes (real, imag) into
 * `out_real` / `out_imag`. Returns `Bool::FALSE` on a null output.
 */
Bool quantum_time_evolution_phase(double energy, double time, double *out_real, double *out_imag);

Bool relativity_kerr_horizon_radii(double mass,
                                   double spin_parameter,
                                   double g,
                                   double *out_event,
                                   double *out_cauchy);

Bool relativity_kerr_ergosphere_radius(double mass,
                                       double spin_parameter,
                                       double polar_angle,
                                       double g,
                                       double *out);

Bool relativity_kerr_frame_dragging_frequency(double mass,
                                              double spin_parameter,
                                              double r,
                                              double theta,
                                              double g,
                                              double *out);

Bool relativity_schwarzschild_isco(double mass, double g, double *out);

Bool relativity_kerr_isco(double mass, double spin_parameter, double g, Bool prograde, double *out);

Bool relativity_gravitational_redshift(double mass, double radius, double g, double *out);

Bool relativity_reissner_nordstrom_horizons(double mass,
                                            double charge,
                                            double g,
                                            double *out_outer,
                                            double *out_inner);

Bool relativity_gw_strain_amplitude(double distance,
                                    double chirp_mass_kg,
                                    double orbital_frequency,
                                    double *out);

Bool relativity_chirp_mass(double mass1, double mass2, double *out);

Bool relativity_gw_frequency_derivative(double frequency, double chirp_mass_kg, double *out);

Bool relativity_relativistic_doppler_longitudinal(double source_frequency,
                                                  double relative_velocity,
                                                  Bool approaching,
                                                  double *out);

Bool relativity_relativistic_doppler_transverse(double source_frequency,
                                                double relative_velocity,
                                                double *out);

Bool relativity_einstein_radius(double mass_kg,
                                double dist_lens,
                                double dist_source,
                                double dist_ls,
                                double *out);

Bool relativity_cosmological_redshift(double scale_factor, double *out);

Bool relativity_redshift_from_wavelengths(double observed, double emitted, double *out);

Bool relativity_lense_thirring_angular_frequency(double mass_kg,
                                                 double spin_parameter,
                                                 double orbital_radius,
                                                 double *out);

Bool relativity_schwarzschild_effective_potential(double r,
                                                  double rs,
                                                  double angular_momentum,
                                                  double *out);

Bool relativity_gw_inspiral_snr(double strain_rss,
                                double f_min,
                                double f_max,
                                double noise_psd,
                                double *out);

Bool relativity_gw_inspiral_time_to_coalescence(double chirp_mass_kg, double f_gw_hz, double *out);

Bool relativity_relativistic_total_energy(double rest_mass, double lorentz_factor, double *out);

Bool relativity_relativistic_momentum(double rest_mass, double speed, double *out);

Bool relativity_relativistic_energy_from_momentum(double rest_mass, double momentum, double *out);

Bool relativity_relativistic_aberration(double cos_theta, double beta, double *out);

Bool relativity_relativistic_doppler_beaming_factor(double beta, double cos_theta, double *out);

Bool relativity_photon_sphere_radius(double mass, double g, double *out);

Bool relativity_hawking_temperature(double mass, double g, double *out);

Bool relativity_hubble_recession_velocity(double distance, double hubble_constant, double *out);

Bool relativity_hubble_distance(double redshift, double hubble_constant, double *out);

Bool relativity_flat_universe_lookback_time(double redshift, double hubble_time, double *out);

Bool thermodynamics_ideal_gas_pressure(double volume,
                                       double moles,
                                       double temperature,
                                       double *out);

Bool thermodynamics_ideal_gas_volume(double pressure,
                                     double moles,
                                     double temperature,
                                     double *out);

Bool thermodynamics_ideal_gas_temperature(double pressure,
                                          double volume,
                                          double moles,
                                          double *out);

Bool thermodynamics_polytropic_pressure(double p1, double v1, double v2, double gamma, double *out);

Bool thermodynamics_polytropic_work(double p1,
                                    double v1,
                                    double p2,
                                    double v2,
                                    double gamma,
                                    double *out);

/**
 * Estimate the aerodynamic/gravity forces acting on a trajectory state.
 *
 * # Safety
 *
 * `out_report`, when non-null, must be valid for a single
 * `TrajectoryForceReport` write.
 */
Bool trajectory_estimate_forces(TrajectoryState state,
                                TrajectoryEnvironment env,
                                TrajectoryForceReport *out_report);

/**
 * Advance a trajectory state by one integration step.
 *
 * # Safety
 *
 * `out_state` and `out_report`, when non-null, must each be valid for a
 * single write of `TrajectoryState` / `TrajectoryForceReport`.
 */
Bool trajectory_integrate_step(TrajectoryState state,
                               TrajectoryEnvironment env,
                               double dt,
                               TrajectoryState *out_state,
                               TrajectoryForceReport *out_report);

/**
 * Apply trajectory forces to a rigid body in the world.
 *
 * # Safety
 *
 * `world` must be a valid, live world pointer; `out_report`, when non-null,
 * must be valid for a single `TrajectoryForceReport` write.
 */
Bool trajectory_apply_forces_to_body(struct WorldHandle *world,
                                     RigidBodyHandleRaw body_handle,
                                     TrajectoryEnvironment env,
                                     Bool wake_up,
                                     TrajectoryForceReport *out_report);

/**
 * Flag-returning variant of `trajectory_apply_forces_to_body`.
 *
 * # Safety
 *
 * Same pointer contract as `trajectory_apply_forces_to_body`.
 */
uint8_t trajectory_apply_forces_to_body_flag(struct WorldHandle *world,
                                             RigidBodyHandleRaw body_handle,
                                             TrajectoryEnvironment env,
                                             Bool wake_up,
                                             TrajectoryForceReport *out_report);

/**
 * Estimate the glide forces acting on a gliding trajectory state.
 *
 * # Safety
 *
 * `out_report`, when non-null, must be valid for a single
 * `TrajectoryGlideReport` write.
 */
Bool trajectory_glide_estimate(TrajectoryGlideState state,
                               TrajectoryGlideEnvironment env,
                               TrajectoryGlideReport *out_report);

/**
 * Advance a gliding trajectory state by one integration step.
 *
 * # Safety
 *
 * `out_state` and `out_report`, when non-null, must each be valid for a
 * single write of `TrajectoryGlideState` / `TrajectoryGlideReport`.
 */
Bool trajectory_glide_integrate_step(TrajectoryGlideState state,
                                     TrajectoryGlideEnvironment env,
                                     double dt,
                                     TrajectoryGlideState *out_state,
                                     TrajectoryGlideReport *out_report);

/**
 * # Safety
 *
 * `voxels` must point to at least `size_x * size_y * size_z` readable bytes
 * for the duration of the call. The returned builder handle is owned by the
 * caller and must be released through the collider-builder ABI.
 */
struct ColliderBuilderHandle *collider_builder_create_voxels(const uint8_t *voxels,
                                                             uint32_t size_x,
                                                             uint32_t size_y,
                                                             uint32_t size_z,
                                                             double voxel_size_x,
                                                             double voxel_size_y,
                                                             double voxel_size_z,
                                                             Vec3 origin,
                                                             VoxelColliderOptions options);

/**
 * # Safety
 *
 * Same pointer contract as `collider_builder_create_voxels`.
 */
struct ColliderBuilderHandle *collider_builder_create_voxels_auto(const uint8_t *voxels,
                                                                  uint32_t size_x,
                                                                  uint32_t size_y,
                                                                  uint32_t size_z,
                                                                  double voxel_size_x,
                                                                  double voxel_size_y,
                                                                  double voxel_size_z,
                                                                  Vec3 origin,
                                                                  Bool dynamic_body);

/**
 * # Safety
 *
 * `voxels` must point to at least `size_x * size_y * size_z` readable bytes
 * for the duration of the call.
 */
VoxelBuildStats voxel_build_stats(const uint8_t *voxels,
                                  uint32_t size_x,
                                  uint32_t size_y,
                                  uint32_t size_z,
                                  double voxel_size_x,
                                  double voxel_size_y,
                                  double voxel_size_z,
                                  Vec3 origin,
                                  VoxelColliderOptions options);

/**
 * Computes build statistics for a voxelized AABB without building a collider.
 *
 * # Safety
 *
 * All arguments are passed by value; no pointers are dereferenced. `aabb`
 * must have finite mins/maxs with `mins < maxs` on every axis, and each
 * voxel size must be finite and positive; violations fail with
 * `ERR_INVALID_ARGUMENT` (or `ERR_CAPACITY` when the grid exceeds the cell
 * limit) and return a zeroed `VoxelBuildStats`.
 */
VoxelBuildStats voxel_aabb_build_stats(AabbDesc aabb,
                                       double voxel_size_x,
                                       double voxel_size_y,
                                       double voxel_size_z,
                                       VoxelColliderOptions options);

/**
 * Computes build statistics for a voxelized OBB without building a collider.
 *
 * # Safety
 *
 * All arguments are passed by value; no pointers are dereferenced. `obb`
 * must have a finite center and rotation and finite, positive half extents,
 * and each voxel size must be finite and positive; violations fail with
 * `ERR_INVALID_ARGUMENT` (or `ERR_CAPACITY` when the grid exceeds the cell
 * limit) and return a zeroed `VoxelBuildStats`.
 */
VoxelBuildStats voxel_obb_build_stats(Obb obb,
                                      double voxel_size_x,
                                      double voxel_size_y,
                                      double voxel_size_z,
                                      VoxelColliderOptions options);

/**
 * # Safety
 *
 * `out_stats` must be null or point to a valid, writable `VoxelBuildStats`.
 */
void voxel_aabb_build_stats_out(AabbDesc aabb,
                                double voxel_size_x,
                                double voxel_size_y,
                                double voxel_size_z,
                                VoxelColliderOptions options,
                                VoxelBuildStats *out_stats);

/**
 * # Safety
 *
 * `out_stats` must be null or point to a valid, writable `VoxelBuildStats`.
 */
void voxel_obb_build_stats_out(Obb obb,
                               double voxel_size_x,
                               double voxel_size_y,
                               double voxel_size_z,
                               VoxelColliderOptions options,
                               VoxelBuildStats *out_stats);

/**
 * Builds a collider builder from an AABB voxelized at the given voxel size.
 *
 * # Safety
 *
 * All arguments are passed by value; no pointers are dereferenced. `aabb`
 * must have finite mins/maxs with `mins < maxs` on every axis, and each
 * voxel size must be finite and positive; violations fail with
 * `ERR_INVALID_ARGUMENT` (or `ERR_CAPACITY` when the grid exceeds the cell
 * limit) and return null. The returned builder handle is owned by the
 * caller and must be released through the collider-builder ABI.
 */
struct ColliderBuilderHandle *collider_builder_create_voxel_aabb(AabbDesc aabb,
                                                                 double voxel_size_x,
                                                                 double voxel_size_y,
                                                                 double voxel_size_z,
                                                                 VoxelColliderOptions options);

/**
 * Builds a collider builder from a voxelized AABB with default options.
 *
 * # Safety
 *
 * Same argument contract as `collider_builder_create_voxel_aabb`.
 */
struct ColliderBuilderHandle *collider_builder_create_voxel_aabb_auto(AabbDesc aabb,
                                                                      double voxel_size_x,
                                                                      double voxel_size_y,
                                                                      double voxel_size_z,
                                                                      Bool dynamic_body);

/**
 * Builds a collider builder from an OBB voxelized at the given voxel size.
 *
 * # Safety
 *
 * All arguments are passed by value; no pointers are dereferenced. `obb`
 * must have a finite center and rotation and finite, positive half extents,
 * and each voxel size must be finite and positive; violations fail with
 * `ERR_INVALID_ARGUMENT` (or `ERR_CAPACITY` when the grid exceeds the cell
 * limit) and return null. The returned builder handle is owned by the
 * caller and must be released through the collider-builder ABI.
 */
struct ColliderBuilderHandle *collider_builder_create_voxel_obb(Obb obb,
                                                                double voxel_size_x,
                                                                double voxel_size_y,
                                                                double voxel_size_z,
                                                                VoxelColliderOptions options);

/**
 * Builds a collider builder from a voxelized OBB with default options.
 *
 * # Safety
 *
 * Same argument contract as `collider_builder_create_voxel_obb`.
 */
struct ColliderBuilderHandle *collider_builder_create_voxel_obb_auto(Obb obb,
                                                                     double voxel_size_x,
                                                                     double voxel_size_y,
                                                                     double voxel_size_z,
                                                                     Bool dynamic_body);

/**
 * # Safety
 *
 * `world` must be null or a valid world handle. `out_handles` must be null
 * or point to `capacity` writable `ColliderHandleRaw` entries.
 */
uint32_t query_intersect_voxel_aabb(const struct WorldHandle *world,
                                    AabbDesc aabb,
                                    QueryFilterDesc filter,
                                    ColliderHandleRaw *out_handles,
                                    uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be null or a valid world handle.
 */
uint32_t query_intersect_voxel_aabb_count(const struct WorldHandle *world,
                                          AabbDesc aabb,
                                          QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be null or a valid world handle. `out_handles` must be null
 * or point to `capacity` writable `ColliderHandleRaw` entries.
 */
uint32_t query_intersect_voxel_obb(const struct WorldHandle *world,
                                   Obb obb,
                                   QueryFilterDesc filter,
                                   ColliderHandleRaw *out_handles,
                                   uint32_t capacity);

/**
 * # Safety
 *
 * `world` must be null or a valid world handle.
 */
uint32_t query_intersect_voxel_obb_count(const struct WorldHandle *world,
                                         Obb obb,
                                         QueryFilterDesc filter);

/**
 * # Safety
 *
 * `world` must be null or a valid world handle. On failure any partially
 * inserted body is removed again before returning 0.
 */
RigidBodyHandleRaw world_insert_static_voxel_aabb(struct WorldHandle *world,
                                                  AabbDesc aabb,
                                                  double voxel_size_x,
                                                  double voxel_size_y,
                                                  double voxel_size_z,
                                                  VoxelColliderOptions options,
                                                  double friction,
                                                  double restitution);

/**
 * # Safety
 *
 * `world` must be null or a valid world handle. On failure any partially
 * inserted body is removed again before returning 0.
 */
RigidBodyHandleRaw world_insert_dynamic_voxel_obb(struct WorldHandle *world,
                                                  Obb obb,
                                                  double voxel_size_x,
                                                  double voxel_size_y,
                                                  double voxel_size_z,
                                                  VoxelColliderOptions options,
                                                  double density,
                                                  double friction,
                                                  double restitution);

/**
 * Flip a single voxel cell of an already-inserted voxel collider **in place**,
 * rebuilding its shape and keeping the same `ColliderHandleRaw`.
 *
 * `solid` is treated as boolean (non-zero = solid). The world must hold the
 * voxel source grid for `handle` (i.e. the collider was built from
 * `collider_builder_create_voxel*`). Out-of-range coordinates are a no-op that
 * still returns `Bool::TRUE` (nothing to update). If the cell did not change,
 * the collider is left untouched (no rebuild). When the last solid cell is
 * removed and the grid becomes empty, the collider is removed from the world
 * and its handle becomes invalid — callers should drop their reference.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`.
 */
Bool collider_voxel_cell_at_point(const struct WorldHandle *world,
                                  ColliderHandleRaw collider,
                                  Vec3 point,
                                  struct VoxelCoord *out_block);

/**
 * Read whether a single voxel cell of a voxel collider is solid (non-zero)
 * or empty (zero) without modifying the grid.
 *
 * The read counterpart of `collider_voxel_edit`: `edit` writes a cell, this
 * one reads it back. It completes the in-place voxel editing toolkit so the
 * mod no longer has to keep its own mirror copy of the grid just to answer
 * "is this block solid?" — needed for block-break drops / place checks /
 * standing-on-block queries (pair it with `collider_voxel_cell_at_point` to
 * turn a world point into a (ix,iy,iz) and then ask this fn for its state).
 *
 * # Output
 * On success `out_solid` is written with the cell's solidity (non-zero if the
 * byte at `(x,y,z)` is non-zero) and the function returns `TRUE`. On a null
 * `world`, a non-voxel collider, or out-of-range coordinates it returns
 * `FALSE` and writes `0` to `out_solid`.
 *
 * # Errors
 * Returns `Bool::FALSE` and sets an error code for a null `world`, or a
 * `collider` that is not backed by a voxel grid (out-of-range coordinates use
 * `ERR_INVALID_ARGUMENT`).
 */
Bool collider_voxel_read_cell(const struct WorldHandle *world,
                              ColliderHandleRaw collider,
                              int64_t x,
                              int64_t y,
                              int64_t z,
                              uint8_t *out_solid);

Bool collider_voxel_edit(struct WorldHandle *world,
                         ColliderHandleRaw handle,
                         int64_t x,
                         int64_t y,
                         int64_t z,
                         int32_t solid);

/**
 * Overwrite the entire voxel grid of an already-inserted voxel collider **in
 * place**, rebuilding its shape and keeping the same `ColliderHandleRaw`.
 *
 * This is the bulk counterpart of `collider_voxel_edit` for chunk reloads /
 * regeneration: pass the full grid plus the same voxel sizing, origin, and
 * build options used at creation time. When the new grid is empty the
 * collider is removed (its handle becomes invalid).
 *
 * # Safety
 * `voxels` must point to at least `size_x * size_y * size_z` readable bytes
 * for the duration of the call. `world` must be a valid `world_create` handle.
 */
Bool collider_set_voxels(struct WorldHandle *world,
                         ColliderHandleRaw handle,
                         const uint8_t *voxels,
                         uint32_t size_x,
                         uint32_t size_y,
                         uint32_t size_z,
                         double voxel_size_x,
                         double voxel_size_y,
                         double voxel_size_z,
                         Vec3 origin,
                         uint32_t mode,
                         int32_t dynamic_body,
                         uint32_t small_voxel_limit,
                         uint32_t mesh_voxel_limit);

/**
 * Cast a ray restricted to a single voxel collider and resolve the hit back
 * to the voxel cell coordinate in that collider's local grid.
 *
 * Pairs with `collider_voxel_edit`: pick the cell a player's ray points at,
 * then flip it. `origin` / `direction` / `max_toi` / `solid` mirror
 * `query_cast_ray`. Returns `TRUE` and fills `out_block` only when the ray
 * actually hit `collider` (a voxel collider with a retained source grid).
 *
 * # Safety
 * `world` must be a valid `world_create` handle; `out_block` may be null or
 * must point to writable space for one `VoxelCoord`.
 */
Bool collider_voxel_ray_pick(const struct WorldHandle *world,
                             ColliderHandleRaw collider,
                             Vec3 origin,
                             Vec3 direction,
                             double max_toi,
                             Bool solid,
                             struct VoxelCoord *out_block);

/**
 * Create a new physics world.  Non-finite gravity components fall back to zero.
 *
 * The returned pointer is owned by Rust; release it with `world_destroy`.
 *
 * # Safety
 * No pointer arguments are dereferenced.  The returned pointer is owned by
 * Rust and must be released exactly once with `world_destroy`.
 */
struct WorldHandle *world_create(Vec3 gravity);

/**
 * Destroy a physics world created by `world_create`.  Null is a no-op.
 *
 * # Safety
 * `world` must be a pointer returned by `world_create` (or null) and must not
 * be used again after this call.
 */
void world_destroy(struct WorldHandle *world);

/**
 * Advance the simulation by `delta_seconds` (clamped to (0, 1]).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
void world_step(struct WorldHandle *world, double delta_seconds);

/**
 * Set integration parameters (dt, solver iterations, CCD substeps).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
Bool world_set_integration_parameters(struct WorldHandle *world,
                                      double dt,
                                      uint32_t solver_iterations,
                                      uint32_t ccd_substeps);

/**
 * Read integration parameters into `out_values` (dt, iterations, CCD substeps).
 *
 * # Safety
 * `world` must be a valid world pointer (or null); `out_values` must point to
 * writable memory for at least `capacity` f64 values.
 */
uint32_t world_get_integration_parameters(const struct WorldHandle *world,
                                          double *out_values,
                                          uint32_t capacity);

/**
 * Set the world gravity vector.  Non-finite input is ignored.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create` and not yet destroyed.
 */
void world_set_gravity(struct WorldHandle *world, Vec3 gravity);

/**
 * Get the world gravity vector.
 *
 * # Safety
 * `world` must be a valid world pointer (or null).
 */
Vec3 world_get_gravity(const struct WorldHandle *world);

/**
 * Number of rigid bodies in the world (-1 on null world).
 *
 * # Safety
 * `world` must be a valid world pointer (or null).
 */
int32_t world_get_rigid_body_set_size(const struct WorldHandle *world);

/**
 * Number of colliders in the world (-1 on null world).
 *
 * # Safety
 * `world` must be a valid world pointer (or null).
 */
int32_t world_get_collider_set_size(const struct WorldHandle *world);

/**
 * Write the world gravity into `out_gravity`.
 *
 * # Safety
 * `out_gravity` must point to a writable `Vec3` (or be null); `world` must be
 * a valid world pointer (or null).
 */
void world_get_gravity_out(const struct WorldHandle *world, Vec3 *out_gravity);

/**
 * Count of dynamic bodies (for sizing a `world_dynamic_body_snapshot` call).
 *
 * # Safety
 * `world` must be a valid world pointer (or null).
 */
uint32_t world_dynamic_body_snapshot_count(const struct WorldHandle *world);

/**
 * Snapshot dynamic body handles + poses (7 f64 per body: pos3 + quat4).
 *
 * # Safety
 * `world` must be a valid world pointer (or null); `out_handles` must point to
 * writable memory for `capacity` handles and `out_values` for `capacity * 7`
 * f64 values.
 */
uint32_t world_dynamic_body_snapshot(const struct WorldHandle *world,
                                     RigidBodyHandleRaw *out_handles,
                                     double *out_values,
                                     uint32_t capacity);

/**
 * Count of all bodies (for sizing a `world_body_snapshot` call).
 *
 * # Safety
 * `world` must be a valid world pointer (or null).
 */
uint32_t world_body_snapshot_count(const struct WorldHandle *world);

/**
 * Snapshot all body handles + poses + velocities (13 f64 per body:
 * pos3 + quat4 + linvel3 + angvel3).
 *
 * # Safety
 * `world` must be a valid world pointer (or null); `out_handles` must point to
 * writable memory for `capacity` handles and `out_values` for `capacity * 13`
 * f64 values.
 */
uint32_t world_body_snapshot(const struct WorldHandle *world,
                             RigidBodyHandleRaw *out_handles,
                             double *out_values,
                             uint32_t capacity);

/**
 * Batch-update body poses (7 f64 per body: pos3 + quat4).  Returns the number
 * of bodies actually updated.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `handles` and
 * `values` must point to readable arrays of `count` handles and `count * 7`
 * f64 values respectively.
 */
uint32_t world_update_body_poses(struct WorldHandle *world,
                                 const RigidBodyHandleRaw *handles,
                                 const double *values,
                                 uint32_t count,
                                 Bool wake_up);

/**
 * Batch-update body velocities (6 f64 per body: linvel3 + angvel3).  Returns
 * the number of bodies actually updated.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `handles` and
 * `values` must point to readable arrays of `count` handles and `count * 6`
 * f64 values respectively.
 */
uint32_t world_update_body_velocities(struct WorldHandle *world,
                                      const RigidBodyHandleRaw *handles,
                                      const double *values,
                                      uint32_t count,
                                      Bool wake_up);

/**
 * Number of force laws registered in the world's ForceRegistry.
 *
 * # Safety
 * `world` must be a valid world pointer (or null).
 */
uint32_t world_get_force_registry_count(const struct WorldHandle *world);

/**
 * Get count of registered force laws of a specific type.
 * `law_type` is the numeric discriminant of `ForceLawType`.
 *
 * # Safety
 * `world` must be a valid world pointer (or null).
 */
uint32_t world_get_force_registry_typed_count(const struct WorldHandle *world, uint32_t law_type);

/**
 * Create a shared-memory physics arena.
 *
 * Returns the arena pointer as a u64 (suitable for `MemorySegment.ofAddress` in Java).
 * The arena persists for the lifetime of the world.
 *
 * At most one arena may exist per world. Calling this again while an arena
 * is still live fails with `ERR_INVALID_ARGUMENT` and leaves the existing
 * arena untouched — call `world_destroy_shared_arena` first to recreate one.
 *
 * WARNING (Java side): before calling `world_destroy_shared_arena`, the
 * `MemorySegment` mapping the arena must be released/unmapped; destroying
 * the arena frees the underlying memory, and any still-mapped Java segment
 * would become a use-after-free.
 *
 * `max_bodies` — max concurrent bodies to mirror
 * `max_events` — max pending collision/contact events
 * `max_commands` — max pending commands (force/set pose etc.)
 * `out_address` — receives the arena base address
 * `out_size` — receives the total arena size in bytes (for Java MemorySegment mapping)
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create`; `out_address`
 * and `out_size` may be null, otherwise each must point to a writable u64.
 */
Bool world_create_shared_arena(struct WorldHandle *world,
                               uint32_t max_bodies,
                               uint32_t max_colliders,
                               uint32_t max_events,
                               uint32_t max_commands,
                               uint64_t *out_address,
                               uint64_t *out_size);

/**
 * Destroy the shared arena (if any).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create` (or null).  Any
 * Java `MemorySegment` mapping the arena must be released before this call.
 */
void world_destroy_shared_arena(struct WorldHandle *world);

/**
 * Get the arena address (returns 0 if no arena).
 *
 * # Safety
 * `world` must be a valid world pointer (or null).
 */
uint64_t world_get_shared_arena_address(const struct WorldHandle *world);

/**
 * Get the arena size (returns 0 if no arena).
 *
 * # Safety
 * `world` must be a valid world pointer (or null).
 */
uint64_t world_get_shared_arena_size(const struct WorldHandle *world);

/**
 * Reset the event ring (Java calls this after draining events).
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create` (or null) and not
 * yet destroyed.
 */
void world_reset_shared_arena_events(struct WorldHandle *world);

/**
 * Enable or disable relative force for a rigid body.
 * When enabled, forces applied via `rigid_body_add_force_at_local_point`
 * will be applied at the local attachment point instead of world coordinates.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create` (or null).
 */
Bool world_set_relative_force_enabled(struct WorldHandle *world,
                                      RigidBodyHandleRaw handle,
                                      Bool enabled,
                                      Vec3 local_point);

/**
 * Check if relative force is enabled for a rigid body.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create` (or null).
 */
Bool world_get_relative_force_enabled(const struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * Get the local attachment point for relative force.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create` (or null).
 */
Vec3 world_get_relative_force_local_point(const struct WorldHandle *world,
                                          RigidBodyHandleRaw handle);

/**
 * Set the local attachment point for relative force.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create` (or null).
 */
Bool world_set_relative_force_local_point(struct WorldHandle *world,
                                          RigidBodyHandleRaw handle,
                                          Vec3 local_point);

/**
 * Remove relative force configuration for a rigid body.
 *
 * # Safety
 * `world` must be a valid pointer returned by `world_create` (or null).
 */
Bool world_remove_relative_force(struct WorldHandle *world, RigidBodyHandleRaw handle);

/**
 * Enable or disable collision detection between two specific colliders, regardless
 * of their collision groups, solver hooks, or whether they are connected by a joint.
 *
 * This surfaces the per-pair collision filtering exposed by Rapier's `World`
 * (`set_collision_enabled`). Unlike collision groups, the two colliders need not
 * belong to the same body or be jointed; any pair can be disabled. Disabling a
 * pair that was previously disabled (or enabling a pair that was never disabled)
 * is a no-op. The setting persists across `world_step` calls: a disabled pair's
 * existing contact manifolds are cleared on the next step.
 *
 * # Safety
 *
 * `world` must be a valid pointer returned by `world_create` (or null). `collider1`
 * and `collider2` must be valid `ColliderHandleRaw` values returned at insert time.
 */
void world_set_collision_enabled(struct WorldHandle *world,
                                 ColliderHandleRaw collider1,
                                 ColliderHandleRaw collider2,
                                 Bool enabled);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* RIGID_BODY_H */
