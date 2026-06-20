# mps_rigid_body Project Analysis

## 1. Project Positioning

`mps_rigid_body` is a Rust native physics wrapper around `rapier3d-f64`.
The crate builds as both `rlib` and `cdylib`, with the `cdylib` used by Java.

The project is not a standalone Rust app. It is a cross-language physics runtime:

```text
Java 21 JNI / Java 25 FFM
  -> Rust C ABI / JNI wrappers
    -> src/rapier wrapper modules
      -> rapier3d-f64
```

Rust owns the physics world, rigid bodies, colliders, joints, events, query
pipeline, and spatial indexes. Java receives opaque pointers or packed handles
and calls back into Rust for all physics operations.

## 2. Source Layout

```text
src/
  lib.rs
  abi/
    ffm.rs
    jni.rs
    mod.rs
  helper/
    helper.rs
    mod.rs
  rapier/
    acoustics.rs
    bounds.rs
    collider.rs
    compat.rs
    control_theory.rs
    controller.rs
    crbtree.rs
    dop.rs
    electromagnetism.rs
    error.rs
    events.rs
    ffi.rs
    joints.rs
    mod.rs
    neural.rs
    spaceflight.rs
    trajectory.rs
    query.rs
    rigid_body.rs
    rtree.rs
    thermodynamics.rs
    voxel.rs
    world.rs

test21/
  Java 21 JNI smoke test and Java helper API

test25/
  Java 25 FFM smoke test and FFM binding sample
```

`src/rapier` is the core implementation. `src/abi` adapts it to external Java
entry points. `src/helper` contains JNI support utilities.

## 3. ABI Design

`src/rapier/ffi.rs` is the ABI type center. It defines `#[repr(C)]` structs used
by C ABI, JNI, and Java FFM.

Important ABI structs:

- `Vec3`: 3D vector.
- `Quat`: quaternion.
- `Bool`: one-byte boolean wrapper.
- `ShapeDesc`: generic primitive shape descriptor.
- `InteractionGroupsDesc`: collision/solver group descriptor.
- `QueryFilterDesc`: query filtering descriptor.
- `ShapeCastOptionsDesc`: shape cast options.
- `RayHit`, `PointProjection`, `ShapeCastHit`: query result records.
- `CollisionEventRecord`, `ContactForceEventRecord`: event records.
- `VoxelColliderOptions`, `VoxelBuildStats`: voxel build configuration and statistics.
- `NeuralBoundsDesc`: neural bounds configuration.
- `TrajectoryState`, `TrajectoryEnvironment`, `TrajectoryForceReport`: point-mass trajectory force model.
- `TrajectoryGlideState`, `TrajectoryGlideEnvironment`, `TrajectoryGlideReport`: boost-glide / skip-glide style trajectory model.
- `OrbitalElements`, `StateVector`: orbital element and Cartesian state conversion.
- `ScalarKalman`, `LeastSquaresAttitude`, `GnssObservation`: estimation and navigation formula results.
- `HohmannTransfer`, `FriisLink`, `SolarPanelPower`, `ThermalBalance`, `RadiatorPower`: mission engineering formula results.
- `CmgExchange`, `CmgRobustInverse`, `RigidBodyEulerDerivative`, `QuaternionDerivative`: attitude dynamics/control result records.
- `MassProperties`, `ContactForceModel`, `BatteryEquivalentCircuit`, `HallThrusterPerformance`: subsystem engineering result records.

The project intentionally does not expose Rapier internal Rust types directly.
External callers use stable C-compatible types.

### 3.1 Opaque Handles

Long-lived Rust-owned objects are exposed as opaque pointers:

- `WorldHandle`
- `RigidBodyBuilderHandle`
- `ColliderBuilderHandle`
- `JointBuilderHandle`
- `CharacterControllerHandle`
- `RTreeHandle`
- `CRbTreeHandle`

Java stores these as `long` or `MemorySegment`, depending on JNI vs FFM.

### 3.2 Packed Rapier Handles

Rapier handles are packed into nonzero `u64` values:

```text
packed = ((generation << 32) | id) + 1
```

`0` is reserved for null/invalid handles. This is used for rigid bodies,
colliders, and impulse joints.

### 3.3 FFM-Friendly Out Pointers

Some native structs are returned through out-pointer variants for Java FFM.
This avoids platform-specific complexity around returning larger structs by value.

Examples:

- `world_get_gravity_out`
- `rigid_body_get_translation_out`
- `rigid_body_get_rotation_out`
- `collider_get_translation_out`
- `collider_get_rotation_out`
- `query_cast_ray_out`
- `query_project_point_out`
- `query_cast_shape_out`
- `voxel_aabb_build_stats_out`
- `voxel_obb_build_stats_out`

For one-byte success/failure paths, Java FFM uses `_flag` functions returning
`u8` instead of `Bool` struct returns.

## 4. Java Access Layers

### 4.1 JNI

`src/abi/jni.rs` exposes JNI-compatible `Java_*` functions and delegates to the
Rust C ABI implementation.

Responsibilities:

- Convert Java primitives to Rust ABI structs.
- Convert Java arrays/native memory addresses to Rust slices.
- Convert Rust result structs back to Java arrays or native output buffers.
- Catch Rust panics at the JNI boundary.

Java 21 wrappers live in `test21/src/main/java/.../util`.

### 4.2 FFM

`test25/.../RigidBodyFfm.java` is a Java 25 FFM binding sample. It declares Java
`MemoryLayout` definitions matching the Rust `repr(C)` ABI and uses downcall
handles for native functions.

The FFM surface currently covers:

- World lifecycle and stepping.
- Gravity and collider count.
- Rigid body builder, insertion, pose/velocity/force/impulse mutation, CCD, sleep/wakeup.
- Collider builder, insertion, parent insertion, runtime material/group/event mutation.
- CRbTree basics.
- Voxel AABB/OBB build stats and collider creation.
- Voxel AABB/OBB queries.
- Regular queries: ray, point projection, AABB, OBB, sphere, shape cast.
- Collision/contact-force event bulk reads and event clearing.

## 5. World Module

`src/rapier/world.rs` owns the Rapier simulation state:

```rust
PhysicsPipeline
Vector gravity
IntegrationParameters
IslandManager
BroadPhaseBvh
NarrowPhase
RigidBodySet
ColliderSet
ImpulseJointSet
MultibodyJointSet
CCDSolver
CallbackPhysicsHooks
CollectingEventHandler
```

Main functions:

- `world_create`
- `world_destroy`
- `world_step`
- `world_set_gravity`
- `world_get_gravity`
- `world_get_gravity_out`
- `world_set_integration_parameters`
- `world_get_integration_parameters`
- `world_body_snapshot`
- `world_dynamic_body_snapshot`
- `world_update_body_poses`
- `world_update_body_velocities`
- `world_get_rigid_body_set_size`
- `world_get_collider_set_size`

`world_step` validates the timestep and advances the Rapier pipeline using the
world's hooks and event handler.

## 6. Rigid Bodies

Implemented in `src/rapier/rigid_body.rs`.

Supported body statuses:

- `Dynamic`
- `Fixed`
- `KinematicPositionBased`
- `KinematicVelocityBased`

Builder capabilities:

- translation
- rotation
- pose
- linear/angular velocity
- additional mass
- additional mass properties
- gravity scale
- damping
- sleep behavior
- enabled rotations
- user data

Runtime capabilities:

- get/set status
- get/set translation
- get/set rotation
- set pose
- get/set linear velocity
- get/set angular velocity
- add force
- add torque
- apply impulse
- apply torque impulse
- enable CCD
- sleep
- wake up
- sleeping state query
- remove/copy body

FFM uses byte-returning `_flag` variants for success/failure mutation calls.

## 7. Colliders

Implemented in `src/rapier/collider.rs`.

Primitive shapes:

- ball
- cuboid
- capsule Y/X/Z
- cylinder
- round cylinder
- cone
- round cone
- round cuboid

Additional builders:

- OBB
- sphere
- heightmap
- convex hull
- point-cloud bounds
- double BV
- skewed OBB
- discrete OBB
- fused collapsing bounds
- edge BVH
- medial spheres

Builder settings:

- translation
- rotation
- pose
- sensor
- friction
- restitution
- density
- collision groups
- solver groups
- active events
- active hooks
- contact-force event threshold

Runtime settings:

- pose
- sensor
- friction
- restitution
- collision groups
- solver groups
- active events
- active hooks
- contact-force event threshold
- density readback

Insertion paths:

- `world_insert_collider`
- `world_insert_collider_with_parent`

## 8. Queries

Implemented in `src/rapier/query.rs`.

Supported query operations:

- ray cast
- batch ray cast
- point projection
- point intersection count
- AABB intersection count and handle output
- OBB intersection count and handle output
- sphere intersection count and handle output
- shape cast

The query pipeline is built from the world's broad phase, narrow phase, bodies,
colliders, and `QueryFilterDesc`.

`QueryFilterDesc` supports:

- query flags
- interaction groups
- excluded collider
- excluded rigid body

FFM uses out-pointer variants for ray, point projection, and shape cast results.

## 9. Events

Implemented in `src/rapier/events.rs`.

The event handler stores:

- collision events
- contact-force events

Events are collected into bounded vectors protected by `parking_lot::Mutex`.
The maximum stored event count is `16_384`.

Public event API:

- `world_clear_events`
- `world_collision_event_count`
- `world_get_collision_event`
- `world_get_collision_events`
- `world_contact_force_event_count`
- `world_get_contact_force_event`
- `world_get_contact_force_events`

Java 25 FFM currently uses the bulk read functions and slices the returned
`MemorySegment` to the number of records written.

## 10. Voxel System

Implemented in `src/rapier/voxel.rs`.

Voxel colliders can be built from:

- raw occupancy grid memory
- Java `byte[]` through JNI
- AABB
- OBB

Build modes:

- `Auto`
- `Cuboids`
- `GreedyCuboids`
- `SurfaceMesh`

Default thresholds:

```text
small_voxel_limit = 128
mesh_voxel_limit = 20_000
```

Important native functions:

- `collider_builder_create_voxels`
- `collider_builder_create_voxels_auto`
- `collider_builder_create_voxel_aabb`
- `collider_builder_create_voxel_aabb_auto`
- `collider_builder_create_voxel_obb`
- `collider_builder_create_voxel_obb_auto`
- `voxel_build_stats`
- `voxel_aabb_build_stats`
- `voxel_obb_build_stats`
- `voxel_aabb_build_stats_out`
- `voxel_obb_build_stats_out`

Voxel query functions:

- `query_intersect_voxel_aabb`
- `query_intersect_voxel_aabb_count`
- `query_intersect_voxel_obb`
- `query_intersect_voxel_obb_count`

Direct insertion helpers:

- `world_insert_static_voxel_aabb`
- `world_insert_dynamic_voxel_obb`

### 10.1 Bounding Boxes as Voxel Sources

Using bounding boxes to define voxels is feasible and implemented.

For AABB, every generated cell is solid because the grid is exactly the box
volume. For OBB, the system first builds a world-space AABB around the oriented
box, then samples voxel cell centers against the OBB's local half-extents.

This keeps the voxel builder pipeline unified: AABB/OBB paths generate an owned
occupancy grid and then reuse the same voxel collider build modes.

### 10.2 Java VoxelGrid

Java 21 includes `VoxelGrid`, which supports:

- `get`
- `set`
- `clear`
- `solidCount`
- `fillBox`
- `fillAabb`
- `fillSphere`
- `copyFrom`
- `union`
- `subtract`
- `intersect`
- `toByteArray`
- `address`

This helper makes simple voxel content possible without manually writing every
occupancy byte.

## 11. Spatial Indexes

The project includes two standalone spatial index wrappers:

- `CRbTree`
- `RTree`

CRbTree has Java 25 FFM coverage for create, insert, and AABB count query.
RTree now has Java 25 FFM coverage for create, destroy, clear, len, insert,
update, remove, rebuild, AABB query count, and AABB query handle output.

Java 21 JNI has broader helper coverage through `SpatialIndex`.

## 12. Extended Geometry

Additional geometry modules:

- `bounds.rs`: capsule, SSV, ellipsoid, prism, cylinder, spherical shell.
- `dop.rs`: kDOP and FDH builders.
- `neural.rs`: neural bounds builder and neural bounds queries.
- `compat.rs`: compatibility helpers such as dynamic cuboids and static trimesh insertion.

Most of these are exposed through Rust C ABI and Java 21 JNI. Java 25 FFM does
not yet cover all of them.

## 13. Joints and Character Controller

`joints.rs` supports impulse joint creation and removal for several joint kinds.
`controller.rs` wraps Rapier's character controller movement and collision data.

These are available through JNI wrappers, but Java 25 FFM parity is still open.

## 14. Verification Status

Verified commands:

```powershell
cargo test

cd test21
.\gradlew.bat check

cd ..\test25
.\gradlew.bat check
```

Current expected result:

```text
Rust tests: passed
Java 21 JNI smoke test: passed
Java 25 FFM smoke test: passed
```

Recent local verification for the formula and spaceflight additions:

```text
cargo check: passed
cargo test: passed, 39 tests
```

## 15. Current Java 25 FFM Coverage

Java 25 FFM currently covers:

- ABI version.
- World create/destroy/step.
- Gravity get/set.
- Collider count.
- Rigid body create/insert/get translation/get rotation.
- Rigid body pose, translation, rotation mutation.
- Rigid body linear/angular velocity mutation.
- Rigid body force, torque, impulse, torque impulse.
- Rigid body CCD, sleep, wakeup, sleeping state.
- Collider builder create for primitive cuboid-like shape.
- Voxel AABB/OBB builder stats and collider creation.
- Collider builder material/event settings.
- Collider insertion and insertion with parent.
- Collider runtime pose, sensor, material, group, event, hook, threshold mutation.
- Collider translation, rotation, and density reads.
- CRbTree create/insert/AABB count query.
- RTree create/destroy/clear/len/insert/update/remove/rebuild/AABB query.
- Regular queries: ray, point projection, AABB, OBB, sphere, shape cast.
- Voxel-shaped AABB/OBB queries.
- Collision event and contact-force event bulk reads.
- Event clearing.

The generated C header also exposes the `space_*`, `trajectory_*`, and other
formula symbols, but the Java 25 sample helper does not yet wrap the full
formula surface. Java 25 callers can still bind these functions manually with
FFM downcall handles if they mirror the `repr(C)` layouts from
`include/rigid_body.h`.

## 16. Recommended Next Steps

Recommended implementation order:

1. Java 25 FFM character controller:
   controller lifecycle, configuration setters, move shape, collision count/read, solve impulses.

2. Java 25 FFM joints:
   joint builder creation, anchors, limits, motors, insert/remove.

3. Java 25 FFM advanced collider builders:
   heightmap, convex hull, point-cloud bounds, kDOP, FDH, neural bounds.

4. Higher-level Java 25 helper layer:
   `RigidBodyFfm` is currently low-level. A safer Java 25 utility layer similar
   to `test21/util` would reduce direct `MemorySegment` handling in user code.

## 17. src Feature Map

This section summarizes the concrete feature responsibility of each `src`
module. It is intended as a quick map for future development.

### 17.1 Crate and ABI Entrypoints

- `src/lib.rs`
  - Crate root.
  - Installs the global allocator.
  - Re-exports ABI-facing structs and handles.
  - Wires `abi`, `helper`, and `rapier` modules.

- `src/abi/ffm.rs`
  - Minimal C ABI capability probes.
  - Exposes `abi_version`, `abi_supports_ffm`, and `abi_supports_jni`.
  - Java 25 FFM calls the native library symbols directly; this file is not the
    whole FFM binding, only the native capability surface.

- `src/abi/jni.rs`
  - JNI bridge for Java 21.
  - Converts Java primitive arguments into Rust ABI structs.
  - Wraps Rust C ABI calls as `Java_org_polaris2023_msp_1rigid_1body_RigidBodyNative_*`.
  - Handles Java arrays, native output buffers, error boundaries, and panic safety.

- `src/helper/helper.rs`
  - JNI helper utilities for array/native-memory conversion.

### 17.2 Core Physics Runtime

- `src/rapier/world.rs`
  - Owns the Rapier physics world state.
  - Creates/destroys worlds, steps simulation, manages gravity and integration parameters.
  - Exposes body/collider counts.
  - Provides body snapshots and batch body pose/velocity updates.

- `src/rapier/rigid_body.rs`
  - Builds and inserts rigid bodies.
  - Supports dynamic, fixed, and kinematic body types.
  - Provides runtime pose, velocity, force, torque, impulse, CCD, sleep, and wakeup control.
  - Includes byte-returning `_flag` variants for Java FFM-friendly mutation calls.

- `src/rapier/collider.rs`
  - Builds and inserts colliders.
  - Supports primitive and advanced collider builders.
  - Provides builder-level material, group, sensor, active event/hook, and threshold settings.
  - Provides runtime collider pose/material/group/event/hook mutation.
  - Supports standalone collider insertion and parented collider insertion.

- `src/rapier/events.rs`
  - Collects collision events and contact-force events during `world_step`.
  - Provides count, single read, bulk read, and clear functions.
  - Stores event queues in bounded mutex-protected vectors.

### 17.3 Query and Movement Systems

- `src/rapier/query.rs`
  - Ray cast and batch ray cast.
  - Point projection.
  - Point, AABB, OBB, and sphere intersections.
  - Shape cast.
  - FFM-friendly out-pointer variants for ray, point projection, and shape cast.

- `src/rapier/aerodynamics.rs`
  - Surface-sample air force accumulation for Rapier rigid bodies.
  - Computes drag and lift from wind velocity, air density, exposed area, and surface normal.
  - Applies forces at world-space surface points with `add_force_at_point`.
  - Includes pure estimation helpers and batch application helpers.

- `src/rapier/controller.rs`
  - Character controller lifecycle and configuration.
  - Supports movement with collision reporting and impulse solving.
  - JNI coverage exists; Java 25 FFM parity is still pending.

### 17.4 Geometry Builders

- `src/rapier/bounds.rs`
  - Capsule, swept sphere volume, ellipsoid, prism, cylinder, and spherical shell builders.
  - Matching intersection queries for these bound types.

- `src/rapier/dop.rs`
  - kDOP and FDH collider builders.
  - Useful for approximate convex bounds from point clouds or directional hulls.

- `src/rapier/neural.rs`
  - Neural bounds descriptor and required weight count calculation.
  - Neural bounds collider builder.
  - Neural bounds intersection queries.

- `src/rapier/voxel.rs`
  - Raw occupancy-grid voxel colliders.
  - AABB/OBB-to-voxel generation.
  - Build modes: cuboids, greedy cuboids, surface mesh, auto.
  - Voxel build statistics.
  - Voxel-shaped AABB/OBB queries.
  - Direct static AABB and dynamic OBB voxel body insertion helpers.

- `src/rapier/compat.rs`
  - Compatibility helpers for common workflows.
  - Includes dynamic cuboid batch insertion and static trimesh insertion.
  - Includes AABB rigid body query helpers.

### 17.5 Aerospace, Controls, and Engineering Formula Layer

The project now includes a broad formula layer in addition to Rapier collision
and dynamics wrappers. This layer has two categories:

1. Pure ABI formulas that return derived values through scalar returns or
   out-pointer result structs.
2. Rapier-integrated formulas that compute forces/torques and apply them
   directly to a `RigidBodyHandleRaw` inside a `WorldHandle`.

- `src/rapier/trajectory.rs`
  - Point-mass trajectory force model with gravity, drag, lift, and optional body force application.
  - Boost-glide / skip-glide style trajectory state equations using velocity,
    flight-path angle, altitude, downrange, exponential atmosphere,
    ballistic coefficient, lift-to-drag ratio, and bank angle.
  - `trajectory_apply_forces_to_body` applies the point-mass force report to a
    Rapier rigid body.

- `src/rapier/spaceflight.rs`
  - Orbital mechanics:
    Kepler third law, orbital elements to/from Cartesian state, J2 perturbation,
    Clohessy-Wiltshire relative motion, Lambert elliptic transfer time,
    Hohmann transfer, semi-major-axis drag decay, SGP4-style J2 secular rates.
  - Attitude dynamics and control:
    quaternion kinematics, Euler rigid-body angular acceleration, CMG angular
    momentum exchange, CMG robust damped pseudo-inverse, gravity-gradient
    torque, magnetic torquer dipole, Bang-Off-Bang attitude profile, solar
    array PD torque.
  - Navigation and estimation:
    TRIAD attitude determination, two-vector least-squares attitude wrapper,
    scalar EKF predict/gain/update, GNSS pseudorange, double-difference carrier phase,
    Sagnac phase/rate helper, radar range/range-rate.
  - Space environment and spacecraft subsystem formulas:
    scale-height atmosphere, atmospheric drag acceleration, solar radiation
    pressure, radiation absorbed dose, atomic oxygen erosion, surface charging
    current balance, Whipple shield critical projectile estimate.
  - Thermal, ECLSS, power, and propulsion:
    heat-pipe thermal resistance, radiator power, thermal equilibrium,
    single-phase fluid loop heat transfer, CO2 mass balance, Sabatier methane
    rate, SPE oxygen rate, solar-panel power, battery equivalent circuit,
    Hall thruster performance, Tsiolkovsky rocket equation.
  - Robotics and docking:
    D-H transform, six-axis first joint inverse, third joint angle, simplified
    diagonal manipulator dynamics, artificial potential guidance, docking
    glideslope command, docking buffer energy absorption, airlock depressurization.
  - Structure and flexible dynamics:
    structural natural frequency, flexible mode derivative, slosh pendulum derivative,
    Hunt-Crossley contact force, two-body mass properties.

Rapier-integrated `spaceflight` entry points:

- `space_apply_j2_force_to_body`
- `space_apply_atmospheric_drag_to_body`
- `space_apply_solar_radiation_pressure_to_body`
- `space_apply_gravity_gradient_torque_to_body`
- `space_apply_magnetic_torquer_to_body`
- `space_apply_cmg_torque_to_body`

These functions locate a rigid body inside `WorldHandle`, compute the requested
spaceflight acceleration/torque, and call Rapier `add_force` or `add_torque`.
The matching `_flag` variants return `u8` for FFM-friendly success/failure.

- `src/rapier/control_theory.rs`
  - Control-oriented formulas and stepping utilities, including state-space
    stepping.
  - Useful as the natural home for future generic controller primitives before
    they are bound to a Rapier body or AnvilKit entity.

Other domain formula modules are present for specialized pure ABI formulas:

- `acoustics.rs`
- `astrophysics.rs`
- `biomechanics.rs`
- `continuum.rs`
- `electromagnetism.rs`
- `fracture.rs`
- `molecular.rs`
- `physchem.rs`
- `quantum.rs`
- `softbody.rs`
- `thermodynamics.rs`
- `topology.rs`
- `transmission.rs`

These modules should remain pure and ABI-oriented unless a formula clearly
maps to a Rapier force, torque, collider, query, or world-step operation.

### 17.6 AnvilKit Integration Layer

- `src/rapier/anvilkit.rs`
  - Owns an AnvilKit ECS app state and maps entities to Rapier rigid bodies,
    colliders, and constraints.
  - Supports spawning bodies/colliders, setting transforms/materials, syncing
    entities to a Rapier world, and resolving entity-to-body/collider handles.
  - Already includes domain-force application helpers:
    `anvilkit_app_apply_aero_surfaces`,
    `anvilkit_app_apply_aero_voxel_grid`,
    `anvilkit_app_apply_fluid_aabb_forces`,
    `anvilkit_app_apply_trajectory_forces`.

Current AnvilKit integration pattern:

```text
entity_bits
  -> anvilkit_app_entity_to_body
  -> RigidBodyHandleRaw
  -> trajectory/spaceflight/aerodynamics/fluid function applies force or torque
```

The existing AnvilKit functions use this pattern for aerodynamics, fluid
forces, and trajectory forces. Spaceflight formulas can be integrated in the
same style by adding `anvilkit_app_apply_space_*` wrappers that resolve the
entity body handle and delegate to `space_apply_*_to_body`.

Recommended AnvilKit wrappers to add next:

- `anvilkit_app_apply_space_j2_force`
- `anvilkit_app_apply_space_atmospheric_drag`
- `anvilkit_app_apply_space_solar_radiation_pressure`
- `anvilkit_app_apply_space_gravity_gradient_torque`
- `anvilkit_app_apply_space_magnetic_torquer`
- `anvilkit_app_apply_space_cmg_torque`

This keeps formulas reusable at three levels:

```text
Pure formula ABI
  -> Rapier WorldHandle/body apply function
    -> AnvilKit entity wrapper
```

### 17.7 Spatial Indexes and Support

- `src/rapier/crbtree.rs`
  - Compact RbTree spatial index.
  - Create, destroy, clear, len, insert, update, remove, AABB query count, AABB query handles.
  - Partially covered by Java 25 FFM.

- `src/rapier/rtree.rs`
  - RTree spatial index.
  - Create, destroy, clear, len, insert, update, remove, rebuild, AABB query count, AABB query handles.
  - Covered in Rust tests, Java 21 JNI helpers, and Java 25 FFM smoke tests.

- `src/rapier/joints.rs`
  - Impulse joint builders and insertion/removal.
  - Supports fixed, revolute, prismatic, rope, spring, and spherical joint styles.
  - Supports anchors, limits, motor velocity, and motor position.
  - Java 25 FFM parity is still pending.

- `src/rapier/error.rs`
  - Thread-local last-error code/message support.
  - Used for invalid pointer/capacity/unsupported operation reporting.

- `src/rapier/ffi.rs`
  - Shared ABI structs, enums, handle packing/unpacking, and conversion helpers.
  - The source of truth for C/JNI/FFM memory layout.

- `src/rapier/mod.rs`
  - Module wiring for the Rapier wrapper layer.

## 18. New Feature Recommendations

The project already has a broad Rust/JNI surface. The most valuable next work is
to improve Java 25 FFM parity and then add higher-level ergonomics.

### 18.1 High Priority

1. Spaceflight Rapier and AnvilKit wrappers

   The pure formula ABI is broad, and several formulas already apply directly
   to Rapier bodies. The next integration step is to expose matching AnvilKit
   entity helpers.

   Add AnvilKit wrappers for:

   - J2 force
   - atmospheric drag
   - solar radiation pressure
   - gravity-gradient torque
   - magnetic torquer torque
   - CMG torque

   Reason: this completes the path from mission/space formulas to actual
   simulation behavior:

   ```text
   formula -> Rapier body force/torque -> AnvilKit entity-level API
   ```

2. Java 25 FFM character controller

   Add FFM bindings for:

   - controller create/destroy
   - up vector
   - offset absolute/relative
   - slide
   - autostep
   - snap to ground
   - slope angles
   - move shape
   - collision count/read
   - solve impulses

   Reason: character movement is a complete gameplay-facing feature, and JNI
   already proves the API shape.

3. Java 25 FFM joints

   Add FFM bindings for:

   - joint builder creation/destruction
   - contacts enabled
   - local anchors
   - limits
   - motor velocity
   - motor position
   - insert/remove impulse joint

   Reason: constraints are essential for physics gameplay and mechanical setups.

### 18.2 Medium Priority

4. Java 25 FFM advanced collider builders

   Add FFM coverage for:

   - heightmap
   - convex hull
   - point cloud bounds
   - double BV
   - skewed/discrete OBB
   - fused collapsing bounds
   - edge BVH
   - medial spheres
   - kDOP
   - FDH
   - neural bounds

   Reason: the Rust and JNI APIs already expose these builders, but FFM cannot
   use most of them yet.

5. Java 25 high-level helper layer

   Build a safer utility package for Java 25 similar to `test21/util`.

   Suggested wrappers:

   - `FfmPhysicsWorld`
   - `FfmRigidBody`
   - `FfmCollider`
   - `FfmQuery`
   - `FfmSpatialIndex`
   - `FfmVoxelGrid`

   Reason: `RigidBodyFfm` is intentionally low-level. A helper layer would hide
   repetitive `MemorySegment` allocation and native ownership details.

6. Bulk query APIs

   Add batch query support beyond ray batches:

   - batch AABB intersection counts
   - batch sphere intersection counts
   - batch OBB intersection counts
   - batch shape casts

   Reason: games and simulations often need many queries per tick. Batch calls
   reduce Java/native call overhead.

7. Air-drag / lift helper layer

   Add higher-level wrappers that can turn geometry samples into
   `AeroSurface` batches and feed them into `aero_apply_surfaces`.

   Reason: this is the cleanest path for an anvilkit-style integration. The
   Rust side stays physics-centric while the caller decides how to sample
   block faces, voxel shells, or mesh triangles.

### 18.3 Longer-Term Features

6. Voxel asset pipeline

   Possible additions:

   - voxelize heightmaps
   - voxelize triangle meshes
   - bitset or RLE compressed voxel input
   - chunked voxel collider streaming
   - material IDs instead of boolean occupancy

   Reason: current voxel support is functional for generated boxes/grids, but a
   content pipeline would make it practical for terrain and destructible scenes.

7. Contact material callbacks or material table

   Add a stable ABI-safe material table for friction/restitution/contact behavior.

   Reason: direct callbacks across FFI are hard to keep safe. A material table
   would provide configurable behavior without exposing arbitrary callback code.

8. Debug geometry export

   Add APIs to export collider/query debug geometry:

   - collider AABBs
   - voxel generated cuboids/mesh stats
   - contact points/normals
   - broad phase query debug output

   Reason: debugging native physics from Java is hard without visibility.

9. Serialization snapshots

   Add optional snapshot save/load:

   - body state
   - collider descriptors
   - joints
   - integration parameters

   Reason: useful for replay, testing, rollback, and editor workflows.
