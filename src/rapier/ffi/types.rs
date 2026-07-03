pub type RigidBodyHandleRaw = u64;
pub type ColliderHandleRaw = u64;
pub type ImpulseJointHandleRaw = u64;

pub struct WorldHandle {
    pub(crate) inner: crate::rapier::world::PhysicsWorld,
}

#[cfg(feature = "anvilkit-bridge")]
pub struct AnvilKitAppHandle {
    pub(crate) inner: crate::rapier::anvilkit::AnvilKitAppState,
}

pub struct RigidBodyBuilderHandle {
    pub(crate) inner: rapier3d::prelude::RigidBodyBuilder,
}

pub struct ColliderBuilderHandle {
    pub(crate) inner: rapier3d::prelude::ColliderBuilder,
}

pub struct JointBuilderHandle {
    pub(crate) inner: crate::rapier::joints::JointBuilderKind,
}

pub struct CharacterControllerHandle {
    pub(crate) inner: crate::rapier::controller::CharacterControllerState,
}

pub struct RTreeHandle {
    pub(crate) inner: crate::rapier::rtree::RTreeIndex,
}

pub struct CRbTreeHandle {
    pub(crate) inner: crate::rapier::crbtree::CRbTreeIndex,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Quat {
    pub i: f64,
    pub j: f64,
    pub k: f64,
    pub w: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Bool(pub u8);

impl Bool {
    pub const FALSE: Self = Self(0);
    pub const TRUE: Self = Self(1);
}

impl From<bool> for Bool {
    fn from(value: bool) -> Self {
        if value { Self::TRUE } else { Self::FALSE }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BodyStatus {
    Dynamic = 0,
    Fixed = 1,
    KinematicPositionBased = 2,
    KinematicVelocityBased = 3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub enum ShapeType {
    #[default]
    Ball = 0,
    Cuboid = 1,
    CapsuleY = 2,
    CapsuleX = 3,
    CapsuleZ = 4,
    Cylinder = 5,
    RoundCylinder = 6,
    Cone = 7,
    RoundCone = 8,
    RoundCuboid = 9,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VoxelColliderMode {
    Auto = 0,
    Cuboids = 1,
    GreedyCuboids = 2,
    SurfaceMesh = 3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VoxelColliderOptions {
    pub mode: u32,
    pub dynamic_body: Bool,
    pub small_voxel_limit: u32,
    pub mesh_voxel_limit: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct VoxelBuildStats {
    pub cell_count: u32,
    pub solid_count: u32,
    pub selected_mode: u32,
    pub estimated_parts: u32,
    pub estimated_vertices: u32,
    pub estimated_triangles: u32,
    pub size_x: u32,
    pub size_y: u32,
    pub size_z: u32,
}

impl Default for VoxelColliderOptions {
    fn default() -> Self {
        Self {
            mode: VoxelColliderMode::Auto as u32,
            dynamic_body: Bool::FALSE,
            small_voxel_limit: 128,
            mesh_voxel_limit: 20_000,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ShapeDesc {
    pub shape_type: u32,
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct InteractionGroupsDesc {
    pub memberships: u32,
    pub filter: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct QueryFilterDesc {
    pub flags: u32,
    pub groups: InteractionGroupsDesc,
    pub use_groups: Bool,
    pub exclude_collider: ColliderHandleRaw,
    pub use_exclude_collider: Bool,
    pub exclude_rigid_body: RigidBodyHandleRaw,
    pub use_exclude_rigid_body: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ShapeCastOptionsDesc {
    pub max_time_of_impact: f64,
    pub target_distance: f64,
    pub stop_at_penetration: Bool,
    pub compute_impact_geometry_on_penetration: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PointProjection {
    pub point: Vec3,
    pub is_inside: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct RayHit {
    pub collider: ColliderHandleRaw,
    pub time_of_impact: f64,
    pub normal: Vec3,
    pub feature: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ShapeCastHit {
    pub collider: ColliderHandleRaw,
    pub time_of_impact: f64,
    pub witness1: Vec3,
    pub witness2: Vec3,
    pub normal1: Vec3,
    pub normal2: Vec3,
    pub status: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AabbDesc {
    pub mins: Vec3,
    pub maxs: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Obb {
    pub center: Vec3,
    pub half_extents: Vec3,
    pub rotation: Quat,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Sphere {
    pub center: Vec3,
    pub radius: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Capsule {
    pub a: Vec3,
    pub b: Vec3,
    pub radius: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Ssv {
    pub a: Vec3,
    pub b: Vec3,
    pub radius: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Ellipsoid {
    pub center: Vec3,
    pub radii: Vec3,
    pub rotation: Quat,
    pub segments: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Prism {
    pub center: Vec3,
    pub radius: f64,
    pub half_height: f64,
    pub sides: u32,
    pub rotation: Quat,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Cylinder {
    pub center: Vec3,
    pub radius: f64,
    pub half_height: f64,
    pub rotation: Quat,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SphericalShell {
    pub center: Vec3,
    pub inner_radius: f64,
    pub outer_radius: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub enum NeuralActivation {
    #[default]
    Relu = 0,
    Tanh = 1,
    Sin = 2,
    Linear = 3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct NeuralBoundsDesc {
    pub center: Vec3,
    pub half_extents: Vec3,
    pub rotation: Quat,
    pub sample_resolution: u32,
    pub hidden_width: u32,
    pub hidden_layers: u32,
    pub activation: u32,
    pub output_scale: f64,
    pub padding: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum KdopPreset {
    K6 = 6,
    K14 = 14,
    K18 = 18,
    K26 = 26,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct EffectiveCharacterMovement {
    pub translation: Vec3,
    pub grounded: Bool,
    pub is_sliding_down_slope: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CharacterCollision {
    pub collider: ColliderHandleRaw,
    pub character_translation: Vec3,
    pub translation_applied: Vec3,
    pub translation_remaining: Vec3,
    pub world_witness1: Vec3,
    pub world_witness2: Vec3,
    pub normal1: Vec3,
    pub normal2: Vec3,
    pub time_of_impact: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CollisionEventRecord {
    pub started: Bool,
    pub collider1: ColliderHandleRaw,
    pub collider2: ColliderHandleRaw,
    pub sensor: Bool,
    pub removed: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ContactForceEventRecord {
    pub collider1: ColliderHandleRaw,
    pub collider2: ColliderHandleRaw,
    pub total_force: Vec3,
    pub total_force_magnitude: f64,
    pub max_force_direction: Vec3,
    pub max_force_magnitude: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CoulombFrictionLaw {
    pub static_coefficient: f64,
    pub dynamic_coefficient: f64,
    pub velocity_threshold: f64,
    pub enabled: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AirDragLaw {
    pub fluid_velocity: Vec3,
    pub density: f64,
    pub dynamic_viscosity: f64,
    pub characteristic_length: f64,
    pub reference_area: f64,
    pub drag_coefficient: f64,
    pub reynolds_stokes_limit: f64,
    pub enabled: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ExternalForceLaw {
    pub buoyancy_enabled: Bool,
    pub fluid_density: f64,
    pub displaced_volume: f64,
    pub buoyancy_gravity: Vec3,
    pub electromagnetic_enabled: Bool,
    pub charge: f64,
    pub electric_field: Vec3,
    pub magnetic_field: Vec3,
    pub elastic_enabled: Bool,
    pub spring_anchor: Vec3,
    pub spring_stiffness: f64,
    pub spring_damping: f64,
    pub gravity_enabled: Bool,
    pub gravity_source: Vec3,
    pub gravitational_parameter: f64,
    pub enabled: Bool,
}

/// Newtonian pairwise gravity configuration for body-body attraction.
///
/// When enabled, every dynamic body attracts every other dynamic body via
/// Newton's law:  F = G · m₁ · m₂ / r².
///
/// Set `gravitational_constant` to 6.67430e-11 for real-world gravity,
/// or a larger value for game-scale simulations.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct NewtonGravityLaw {
    /// Gravitational constant (default: 6.67430e-11 N·m²/kg²).
    /// Use larger values for game-scale simulations.
    pub gravitational_constant: f64,
    /// Minimum distance to prevent division by zero (default: 0.01 m).
    pub min_distance: f64,
    /// Maximum distance for gravity to apply (0 = no limit).
    pub max_distance: f64,
    pub enabled: Bool,
}

impl Default for NewtonGravityLaw {
    fn default() -> Self {
        Self {
            gravitational_constant: 6.67430e-11,
            min_distance: 0.01,
            max_distance: 0.0,
            enabled: Bool::FALSE,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CustomPhysicsReport {
    pub body_count: u32,
    pub drag_body_count: u32,
    pub external_force_body_count: u32,
    pub total_drag_force: Vec3,
    pub total_external_force: Vec3,
    pub max_reynolds_number: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PidGains {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub output_min: f64,
    pub output_max: f64,
    pub integral_min: f64,
    pub integral_max: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PidState {
    pub integral: f64,
    pub previous_error: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PidReport {
    pub error: f64,
    pub integral: f64,
    pub derivative: f64,
    pub unclamped_output: f64,
    pub output: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StateSpaceReport {
    pub state_count: u32,
    pub input_count: u32,
    pub output_count: u32,
    pub max_state_delta: f64,
    pub output_norm: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MpcConfig {
    pub state_count: u32,
    pub input_count: u32,
    pub horizon: u32,
    pub dt: f64,
    pub control_min: f64,
    pub control_max: f64,
    pub gradient_iterations: u32,
    pub step_size: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MpcReport {
    pub horizon: u32,
    pub iterations: u32,
    pub initial_cost: f64,
    pub final_cost: f64,
    pub first_control_norm: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct TopologyOptimizationParams {
    pub volume_fraction: f64,
    pub penalization: f64,
    pub min_density: f64,
    pub move_limit: f64,
    pub filter_radius: f64,
    pub stiffness_min: f64,
    pub stiffness_solid: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct TopologyOptimizationReport {
    pub cell_count: u32,
    pub average_density: f64,
    pub min_density: f64,
    pub max_density: f64,
    pub total_compliance: f64,
    pub max_density_change: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SimpMaterialReport {
    pub density: f64,
    pub stiffness: f64,
    pub stiffness_derivative: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DensityFieldStats {
    pub cell_count: u32,
    pub solid_count: u32,
    pub average_density: f64,
    pub min_density: f64,
    pub max_density: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MaterialProperties {
    pub density: f64,
    pub friction: f64,
    pub restitution: f64,
    pub youngs_modulus: f64,
    pub poisson_ratio: f64,
    pub thermal_expansion: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressStrainReport {
    pub strain: f64,
    pub stress: f64,
    pub elastic_energy_density: f64,
    pub thermal_strain: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct HertzContactReport {
    pub effective_modulus: f64,
    pub effective_radius: f64,
    pub contact_radius: f64,
    pub contact_area: f64,
    pub normal_force: f64,
    pub stiffness: f64,
    pub damping_force: f64,
    pub total_force: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct HillMuscleDesc {
    pub max_isometric_force: f64,
    pub optimal_fiber_length: f64,
    pub tendon_slack_length: f64,
    pub max_contraction_velocity: f64,
    pub parallel_stiffness: f64,
    pub series_stiffness: f64,
    pub damping: f64,
    pub pennation_angle: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct HillMuscleState {
    pub activation: f64,
    pub fiber_length: f64,
    pub fiber_velocity: f64,
    pub tendon_length: f64,
    pub moment_arm: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct HillMuscleReport {
    pub active_force: f64,
    pub parallel_elastic_force: f64,
    pub series_elastic_force: f64,
    pub damping_force: f64,
    pub total_fiber_force: f64,
    pub tendon_force: f64,
    pub joint_torque: f64,
    pub force_length_factor: f64,
    pub force_velocity_factor: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SkeletalJointLimit {
    pub min_angle: f64,
    pub max_angle: f64,
    pub stiffness: f64,
    pub damping: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SkeletalConstraintReport {
    pub clamped_angle: f64,
    pub angle_error: f64,
    pub corrective_torque: f64,
    pub limited: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GearConstraintDesc {
    pub ratio: f64,
    pub phase: f64,
    pub backlash: f64,
    pub opposite_direction: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GearConstraintReport {
    pub target_angle: f64,
    pub target_angular_velocity: f64,
    pub angle_error: f64,
    pub velocity_error: f64,
    pub effective_ratio: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ScrewConstraintDesc {
    pub lead: f64,
    pub phase: f64,
    pub right_handed: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ScrewConstraintReport {
    pub target_translation: f64,
    pub target_linear_velocity: f64,
    pub translation_error: f64,
    pub velocity_error: f64,
    pub meters_per_radian: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CamConstraintDesc {
    pub base_radius: f64,
    pub lift: f64,
    pub rise_angle: f64,
    pub return_angle: f64,
    pub phase: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CamConstraintReport {
    pub wrapped_angle: f64,
    pub radius: f64,
    pub follower_displacement: f64,
    pub displacement_derivative: f64,
    pub target_velocity: f64,
    pub displacement_error: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SpiralConstraintDesc {
    pub initial_radius: f64,
    pub radial_pitch: f64,
    pub phase: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SpiralConstraintReport {
    pub radius: f64,
    pub position: Vec3,
    pub tangent: Vec3,
    pub radial_velocity: f64,
    pub constraint_error: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AeroSurface {
    pub point: Vec3,
    pub normal: Vec3,
    pub area: f64,
    pub drag_coefficient: f64,
    pub lift_coefficient: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AeroForceReport {
    pub total_force: Vec3,
    pub total_torque: Vec3,
    pub surface_count: u32,
    pub active_surface_count: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FluidVolume {
    pub center: Vec3,
    pub half_extents: Vec3,
    pub density: f64,
    pub linear_drag: f64,
    pub quadratic_drag: f64,
    pub angular_drag: f64,
    pub flow_velocity: Vec3,
    pub gravity: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FluidForceReport {
    pub buoyancy_force: Vec3,
    pub drag_force: Vec3,
    pub angular_damping_torque: Vec3,
    pub total_force: Vec3,
    pub total_torque: Vec3,
    pub submerged_fraction: f64,
    pub displaced_volume: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct NavierStokesReport {
    pub advection: Vec3,
    pub pressure_acceleration: Vec3,
    pub viscosity_acceleration: Vec3,
    pub external_acceleration: Vec3,
    pub total_acceleration: Vec3,
    pub next_velocity: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SphParticle {
    pub position: Vec3,
    pub velocity: Vec3,
    pub mass: f64,
    pub density: f64,
    pub pressure: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SphForceReport {
    pub density: f64,
    pub pressure: f64,
    pub pressure_force: Vec3,
    pub viscosity_force: Vec3,
    pub surface_tension_force: Vec3,
    pub total_force: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MolecularParticle {
    pub position: Vec3,
    pub velocity: Vec3,
    pub mass: f64,
    pub charge: f64,
    pub epsilon: f64,
    pub sigma: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MolecularForceLaw {
    pub coulomb_constant: f64,
    pub relative_permittivity: f64,
    pub cutoff_radius: f64,
    pub softening: f64,
    pub lennard_jones_enabled: Bool,
    pub coulomb_enabled: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MolecularPairReport {
    pub displacement: Vec3,
    pub distance: f64,
    pub lennard_jones_potential: f64,
    pub coulomb_potential: f64,
    pub total_potential: f64,
    pub lennard_jones_force: Vec3,
    pub coulomb_force: Vec3,
    pub total_force: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct QuantumWaveFunction {
    pub amplitude_real: f64,
    pub amplitude_imag: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct QuantumBarrier {
    pub particle_energy: f64,
    pub barrier_potential: f64,
    pub barrier_width: f64,
    pub particle_mass: f64,
    pub reduced_planck: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct QuantumTunnelingReport {
    pub wave_number: f64,
    pub decay_constant: f64,
    pub exponent: f64,
    pub transmission_coefficient: f64,
    pub reflection_coefficient: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct QuantumOscillatorReport {
    pub angular_frequency: f64,
    pub zero_point_energy: f64,
    pub first_excited_energy: f64,
    pub level_spacing: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FemTetrahedron {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
    pub d: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FemShapeFunctionReport {
    pub weights: [f64; 4],
    pub gradients: [Vec3; 4],
    pub volume: f64,
    pub inside: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FemConstitutiveReport {
    pub lambda: f64,
    pub shear_modulus: f64,
    pub bulk_modulus: f64,
    pub matrix_size: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct NewmarkBetaParameters {
    pub beta: f64,
    pub gamma: f64,
    pub dt: f64,
}

impl Default for NewmarkBetaParameters {
    fn default() -> Self {
        Self {
            beta: 0.25,
            gamma: 0.5,
            dt: 0.0,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct NewmarkBetaReport {
    pub dof: u32,
    pub beta: f64,
    pub gamma: f64,
    pub dt: f64,
    pub effective_stiffness_scale: f64,
    pub effective_damping_scale: f64,
    pub max_delta_displacement: f64,
    pub residual_norm: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct BernoulliReport {
    pub pressure: f64,
    pub velocity: f64,
    pub elevation: f64,
    pub total_head: f64,
    pub dynamic_pressure: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FractureMaterial {
    pub youngs_modulus: f64,
    pub poisson_ratio: f64,
    pub fracture_toughness: f64,
    pub surface_energy: f64,
    pub density: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressIntensityReport {
    pub stress_intensity: f64,
    pub critical: Bool,
    pub safety_factor: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GriffithReport {
    pub critical_stress: f64,
    pub energy_release_rate: f64,
    pub critical_energy_release_rate: f64,
    pub will_fracture: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MinerDamageReport {
    pub damage: f64,
    pub remaining_life_fraction: f64,
    pub failed: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SnCurveReport {
    pub cycles_to_failure: f64,
    pub infinite_life: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FractureEnergyReport {
    pub available_energy: f64,
    pub surface_energy_required: f64,
    pub fragment_kinetic_energy: f64,
    pub will_fracture: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FractureModeReport {
    pub mode: u32,
    pub driving_stress: f64,
    pub mixed_mode_ratio: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FractureFragmentDesc {
    pub local_center: Vec3,
    pub half_extents: Vec3,
    pub initial_velocity: Vec3,
    pub density: f64,
    pub friction: f64,
    pub restitution: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FractureReplaceReport {
    pub fragment_count: u32,
    pub joint_count: u32,
    pub removed_source: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct HeatConductionReport {
    pub temperature_delta: f64,
    pub temperature_gradient: f64,
    pub heat_flux: f64,
    pub heat_rate: f64,
    pub thermal_resistance: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PhaseChangeReport {
    pub final_temperature: f64,
    pub sensible_heat: f64,
    pub latent_heat_used: f64,
    pub phase_fraction_delta: f64,
    pub phase_changed: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ThermalRadiationReport {
    pub emitted_power: f64,
    pub absorbed_power: f64,
    pub net_power: f64,
    pub radiative_coefficient: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FemHeatNode {
    pub temperature: f64,
    pub heat_capacity: f64,
    pub heat_source: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FemHeatEdge {
    pub node_a: u32,
    pub node_b: u32,
    pub conductance: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FemHeatDiffusionReport {
    pub node_count: u32,
    pub edge_count: u32,
    pub total_heat_rate: f64,
    pub max_temperature_delta: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ThermalStressReport {
    pub free_thermal_strain: f64,
    pub mechanical_strain: f64,
    pub stress: f64,
    pub deformation: f64,
    pub elastic_energy_density: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ThermoelasticReport {
    pub thermal_strain: f64,
    pub mechanical_strain_x: f64,
    pub mechanical_strain_y: f64,
    pub mechanical_strain_z: f64,
    pub stress_x: f64,
    pub stress_y: f64,
    pub stress_z: f64,
    pub bulk_modulus: f64,
    pub shear_modulus: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ElectromagneticField {
    pub electric: Vec3,
    pub magnetic: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LorentzForceReport {
    pub electric_force: Vec3,
    pub magnetic_force: Vec3,
    pub total_force: Vec3,
    pub acceleration: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MagneticFluxReport {
    pub flux: f64,
    pub normal_component: f64,
    pub area: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FaradayInductionReport {
    pub flux_rate: f64,
    pub induced_emf: f64,
    pub induced_current: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MaxwellPointReport {
    pub next_field: ElectromagneticField,
    pub electric_derivative: Vec3,
    pub magnetic_derivative: Vec3,
    pub gauss_electric_residual: f64,
    pub gauss_magnetic_residual: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FdtdYeeReport {
    pub cell_count: u32,
    pub max_electric_delta: f64,
    pub max_magnetic_delta: f64,
    pub total_energy_density: f64,
    pub courant_number: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ModalAnalysisReport {
    pub dof: u32,
    pub mode_count: u32,
    pub stable_mode_count: u32,
    pub max_frequency_hz: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StructuralModeReport {
    pub angular_frequency: f64,
    pub frequency_hz: f64,
    pub damping_ratio: f64,
    pub damped_frequency_hz: f64,
    pub critical_damping: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AcousticWaveReport {
    pub cell_count: u32,
    pub max_pressure: f64,
    pub acoustic_energy: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AcousticResonanceReport {
    pub resonant: Bool,
    pub nearest_mode_index: u32,
    pub nearest_frequency_hz: f64,
    pub frequency_delta_hz: f64,
    pub amplification_estimate: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AcousticMaterial {
    pub density: f64,
    pub hardness: f64,
    pub damping: f64,
    pub roughness: f64,
    pub restitution: f64,
    pub sound_speed: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AcousticContactDesc {
    pub normal_force: f64,
    pub normal_velocity: f64,
    pub tangential_velocity: f64,
    pub contact_area: f64,
    pub dt: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AcousticExcitationReport {
    pub impulse: f64,
    pub normal_component: f64,
    pub scrape_component: f64,
    pub brightness: f64,
    pub damping: f64,
    pub amplitude: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ModalSynthesisReport {
    pub mode_count: u32,
    pub sample: f64,
    pub peak_modal_displacement: f64,
    pub modal_energy: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SpatializedSample {
    pub left: f64,
    pub right: f64,
    pub distance: f64,
    pub attenuation: f64,
    pub pan: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SoftSpring {
    pub particle_a: u32,
    pub particle_b: u32,
    pub rest_length: f64,
    pub stiffness: f64,
    pub damping: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SoftDistanceConstraint {
    pub particle_a: u32,
    pub particle_b: u32,
    pub rest_length: f64,
    pub stiffness: f64,
    pub compliance: f64,
    pub lambda: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SoftBendingConstraint {
    pub particle_a: u32,
    pub particle_b: u32,
    pub rest_distance: f64,
    pub stiffness: f64,
    pub compliance: f64,
    pub lambda: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SoftSphereCollision {
    pub center: Vec3,
    pub radius: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SoftVolumeConstraint {
    pub particle_a: u32,
    pub particle_b: u32,
    pub particle_c: u32,
    pub particle_d: u32,
    pub rest_volume: f64,
    pub compliance: f64,
    pub lambda: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SoftBodyStepReport {
    pub particle_count: u32,
    pub constraint_count: u32,
    pub active_particle_count: u32,
    pub max_correction: f64,
    pub total_error: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct TrajectoryState {
    pub position: Vec3,
    pub velocity: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct TrajectoryEnvironment {
    pub gravity: Vec3,
    pub flow_velocity: Vec3,
    pub mass: f64,
    pub reference_area: f64,
    pub density: f64,
    pub drag_coefficient: f64,
    pub lift_coefficient: f64,
    pub lift_direction: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct TrajectoryForceReport {
    pub gravity_force: Vec3,
    pub drag_force: Vec3,
    pub lift_force: Vec3,
    pub total_force: Vec3,
    pub acceleration: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct TrajectoryGlideState {
    pub speed: f64,
    pub flight_path_angle: f64,
    pub altitude: f64,
    pub downrange: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct TrajectoryGlideEnvironment {
    pub gravity: f64,
    pub planet_radius: f64,
    pub ballistic_coefficient: f64,
    pub lift_to_drag: f64,
    pub bank_angle: f64,
    pub reference_density: f64,
    pub scale_height: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct TrajectoryGlideReport {
    pub density: f64,
    pub dynamic_pressure: f64,
    pub drag_acceleration: f64,
    pub lift_acceleration: f64,
    pub speed_dot: f64,
    pub flight_path_angle_dot: f64,
    pub altitude_dot: f64,
    pub downrange_dot: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct OrbitalElements {
    pub semi_major_axis: f64,
    pub eccentricity: f64,
    pub inclination: f64,
    pub raan: f64,
    pub argument_of_periapsis: f64,
    pub true_anomaly: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StateVector {
    pub position: Vec3,
    pub velocity: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct NBodyParticle {
    pub position: Vec3,
    pub velocity: Vec3,
    pub mass: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct NBodySolverParams {
    pub gravitational_constant: f64,
    pub softening: f64,
    pub opening_angle: f64,
    pub multipole_order: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct NBodyForceReport {
    pub body_count: u32,
    pub approximate_node_count: u32,
    pub direct_pair_count: u32,
    pub max_acceleration: f64,
    pub potential_energy: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct RelativisticOrbitReport {
    pub schwarzschild_radius: f64,
    pub periapsis_precession_per_orbit: f64,
    pub correction_acceleration: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GravitationalTimeDilation {
    /// dtau/dt for a stationary observer at radius r = sqrt(1 - rs/r)
    pub stationary_factor: f64,
    /// dtau/dt for a circular orbiting observer = sqrt(1 - 3*rs/(2*r))
    pub orbital_factor: f64,
    /// Newtonian orbital speed at radius r = sqrt(GM/r)
    pub orbiting_velocity: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LengthContraction {
    /// Lorentz factor gamma = 1/sqrt(1 - v^2/c^2)
    pub lorentz_factor: f64,
    /// Contracted length L = L0 / gamma
    pub contracted_length: f64,
    /// Rest (proper) length L0
    pub proper_length: f64,
    /// Speed ratio beta = v/c
    pub speed_ratio: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LorentzBoost {
    /// 4x4 Lorentz boost matrix in row-major order acting on (ct, x, y, z)
    pub m00: f64,
    pub m01: f64,
    pub m02: f64,
    pub m03: f64,
    pub m10: f64,
    pub m11: f64,
    pub m12: f64,
    pub m13: f64,
    pub m20: f64,
    pub m21: f64,
    pub m22: f64,
    pub m23: f64,
    pub m30: f64,
    pub m31: f64,
    pub m32: f64,
    pub m33: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LorentzTransformedFrame {
    /// Time component in the boosted frame (c * t')
    pub ct_prime: f64,
    pub x_prime: f64,
    pub y_prime: f64,
    pub z_prime: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct RelativisticParticle {
    /// Lorentz factor gamma
    pub lorentz_factor: f64,
    /// Total energy E = gamma * m * c^2
    pub total_energy: f64,
    /// Kinetic energy K = (gamma - 1) * m * c^2
    pub kinetic_energy: f64,
    /// Momentum magnitude p = gamma * m * v
    pub momentum_magnitude: f64,
    /// Momentum 3-vector
    pub momentum: Vec3,
    /// Rapidity = arctanh(v/c)
    pub rapidity: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SchwarzschildMetric {
    /// Time-time metric coefficient g_tt = -(1 - rs/r)
    pub g_tt: f64,
    /// Radial-radial metric coefficient g_rr = 1/(1 - rs/r)
    pub g_rr: f64,
    /// Schwarzschild radius rs = 2GM/c^2
    pub schwarzschild_radius: f64,
    /// Ratio r/rs
    pub radius_over_rs: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct RocheLimitReport {
    pub fluid_roche_limit: f64,
    pub rigid_roche_limit: f64,
    pub inside_fluid_limit: Bool,
    pub inside_rigid_limit: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct OrbitalResonanceReport {
    pub ratio_numerator: u32,
    pub ratio_denominator: u32,
    pub actual_ratio: f64,
    pub target_ratio: f64,
    pub relative_error: f64,
    pub resonant: Bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct QuaternionDerivative {
    pub i_dot: f64,
    pub j_dot: f64,
    pub k_dot: f64,
    pub w_dot: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct RigidBodyEulerDerivative {
    pub angular_acceleration: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CmgExchange {
    pub body_torque: Vec3,
    pub wheel_momentum_dot: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CwState {
    pub position: Vec3,
    pub velocity: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CwDerivative {
    pub velocity: Vec3,
    pub acceleration: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DhTransform {
    pub m00: f64,
    pub m01: f64,
    pub m02: f64,
    pub m03: f64,
    pub m10: f64,
    pub m11: f64,
    pub m12: f64,
    pub m13: f64,
    pub m20: f64,
    pub m21: f64,
    pub m22: f64,
    pub m23: f64,
    pub m30: f64,
    pub m31: f64,
    pub m32: f64,
    pub m33: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ManipulatorDynamics {
    pub torque: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SolarPanelPower {
    pub incident_power: f64,
    pub electrical_power: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ThermalBalance {
    pub net_power: f64,
    pub equilibrium_temperature: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Co2MassBalance {
    pub mass_rate: f64,
    pub next_mass: f64,
    pub concentration_rate: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FriisLink {
    pub received_power: f64,
    pub path_loss: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct HohmannTransfer {
    pub delta_v1: f64,
    pub delta_v2: f64,
    pub total_delta_v: f64,
    pub transfer_time: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ScalarKalman {
    pub value: f64,
    pub covariance: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LeastSquaresAttitude {
    pub attitude: Quat,
    pub rms_error: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GnssObservation {
    pub value: f64,
    pub geometric_range: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ContactForceModel {
    pub normal_force: f64,
    pub damping_force: f64,
    pub total_force: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct BatteryEquivalentCircuit {
    pub terminal_voltage: f64,
    pub rc_voltage_dot: f64,
    pub state_of_charge_dot: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct HallThrusterPerformance {
    pub thrust: f64,
    pub specific_impulse: f64,
    pub efficiency: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CollisionProbability {
    pub probability: f64,
    pub combined_sigma: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AtomicOxygenErosion {
    pub volume_loss: f64,
    pub mass_loss: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FlexibleModeDerivative {
    pub displacement_dot: f64,
    pub velocity_dot: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SloshPendulumDerivative {
    pub angle_dot: f64,
    pub angular_rate_dot: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct VariationalState {
    pub position_dot: Vec3,
    pub velocity_dot: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FluidLoopHeatTransfer {
    pub heat_rate: f64,
    pub outlet_temperature: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct RadarMeasurement {
    pub range: f64,
    pub range_rate: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MassProperties {
    pub center_of_mass: Vec3,
    pub inertia_diag: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct BangOffBangProfile {
    pub coast_time: f64,
    pub total_time: f64,
    pub switch_angle: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CmgRobustInverse {
    pub gimbal_rates: Vec3,
    pub damping: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Sgp4SecularRates {
    pub mean_motion_dot: f64,
    pub raan_dot: f64,
    pub argument_of_perigee_dot: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ChemicalReactionRate {
    pub reactant_rate: f64,
    pub product_rate: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GrayScottParams {
    pub diffusion_u: f64,
    pub diffusion_v: f64,
    pub feed_rate: f64,
    pub kill_rate: f64,
    pub dx: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ReactionDiffusionReport {
    pub cell_count: u32,
    pub max_delta_u: f64,
    pub max_delta_v: f64,
    pub total_u: f64,
    pub total_v: f64,
    pub max_reaction_rate: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GrayScottReactionReport {
    pub reaction_rate: f64,
    pub diffusion_u_term: f64,
    pub diffusion_v_term: f64,
    pub du_dt: f64,
    pub dv_dt: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CatalystEffect {
    pub concentration: f64,
    pub strength: f64,
    pub saturation: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CatalystReport {
    pub rate_multiplier: f64,
    pub effective_rate: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ConcentrationBuoyancyReport {
    pub density: f64,
    pub density_delta: f64,
    pub buoyancy_acceleration: Vec3,
    pub buoyancy_force: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct RadiatorPower {
    pub emitted_power: f64,
    pub net_power: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AirlockDepressurization {
    pub pressure: f64,
    pub pressure_rate: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum JointAxisDesc {
    LinX = 0,
    LinY = 1,
    LinZ = 2,
    AngX = 3,
    AngY = 4,
    AngZ = 5,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum JointTypeDesc {
    Fixed = 0,
    Revolute = 1,
    Prismatic = 2,
    Rope = 3,
    Spring = 4,
    Spherical = 5,
}

// ---------------------------------------------------------------------------
// Chaos theory / nonlinear dynamics structures
// ---------------------------------------------------------------------------

/// Lorenz attractor state at a single time step.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LorenzState {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Parameters for the Lorenz system: dx/dt = sigma*(y-x), dy/dt = x*(rho-z)-y, dz/dt = x*y - beta*z.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LorenzParams {
    pub sigma: f64,
    pub rho: f64,
    pub beta: f64,
    pub dt: f64,
}

impl Default for LorenzParams {
    fn default() -> Self {
        Self {
            sigma: 10.0,
            rho: 28.0,
            beta: 8.0 / 3.0,
            dt: 0.01,
        }
    }
}

/// Full Lorenz integration report at a step.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LorenzStepReport {
    pub state: LorenzState,
    pub dx: f64,
    pub dy: f64,
    pub dz: f64,
}

/// Lyapunov exponent estimation report for a single trajectory.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LyapunovReport {
    /// Largest Lyapunov exponent (bits/s or nats/s depending on log base)
    pub largest_exponent: f64,
    /// Convergence indicator: number of orbit steps used
    pub convergence_steps: u32,
    /// Whether the exponent is positive (chaotic) within the tolerance
    pub positive: Bool,
}

/// A single bifurcation point: parameter value vs. sampled state.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct BifurcationPoint {
    pub parameter: f64,
    pub sample: f64,
}

/// Double pendulum state (generalised coordinates and their derivatives).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DoublePendulumState {
    /// Angle of upper pendulum (radians)
    pub theta1: f64,
    /// Angle of lower pendulum (radians)
    pub theta2: f64,
    /// Angular velocity of upper pendulum (rad/s)
    pub omega1: f64,
    /// Angular velocity of lower pendulum (rad/s)
    pub omega2: f64,
}

/// Double pendulum parameters (geometry and integration step).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DoublePendulumParams {
    /// Mass of upper bob
    pub m1: f64,
    /// Mass of lower bob
    pub m2: f64,
    /// Length of upper rod
    pub l1: f64,
    /// Length of lower rod
    pub l2: f64,
    /// Gravitational acceleration
    pub g: f64,
    /// Integration time step
    pub dt: f64,
}

impl Default for DoublePendulumParams {
    fn default() -> Self {
        Self {
            m1: 1.0,
            m2: 1.0,
            l1: 1.0,
            l2: 1.0,
            g: 9.81,
            dt: 0.01,
        }
    }
}

/// Double-pendulum acceleration report (RK4 intermediate computation).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DoublePendulumAccel {
    pub alpha1: f64,
    pub alpha2: f64,
}

/// Report from a chaos detection analysis.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ChaosDetectionReport {
    /// Largest Lyapunov exponent estimate
    pub lyapunov_exponent: f64,
    /// Correlation dimension estimate (box-counting style)
    pub correlation_dimension: f64,
    /// Whether the system is classified as chaotic
    pub is_chaotic: Bool,
    /// Confidence metric between 0 and 1
    pub confidence: f64,
}

/// Parameters controlling chaos detection heuristics.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ChaosDetectionParams {
    /// Number of orbit steps to sample
    pub sample_steps: u32,
    /// Embedding dimension for delay-coordinate reconstruction
    pub embedding_dim: u32,
    /// Delay (in steps) for reconstruction
    pub embedding_delay: u32,
    /// Neighbourhood radius for correlation dimension
    pub neighbourhood_radius: f64,
    /// Threshold above which Lyapunov exponent is considered chaotic
    pub chaotic_threshold: f64,
}

impl Default for ChaosDetectionParams {
    fn default() -> Self {
        Self {
            sample_steps: 10_000,
            embedding_dim: 3,
            embedding_delay: 1,
            neighbourhood_radius: 0.1,
            chaotic_threshold: 0.001,
        }
    }
}

/// Logistic map state (classic 1D chaos example).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LogisticMapState {
    pub x: f64,
    pub r: f64,
}

// ---------------------------------------------------------------------------
// Superfluidity / quantum vortex structures
// ---------------------------------------------------------------------------

/// A single quantum vortex segment (straight line in 3D).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct VortexSegment {
    pub start: Vec3,
    pub end: Vec3,
    /// Circulation quantum number (integer)
    pub circulation_quantum: i32,
    /// Core radius (healing length)
    pub core_radius: f64,
}

/// Velocity induced by a vortex segment at a field point (Biot–Savart kernel).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct BiotSavartVelocity {
    pub velocity: Vec3,
    pub magnitude: f64,
    /// Distance from segment to field point
    pub distance: f64,
}

/// Gross–Pitaevskii order parameter (condensate wavefunction) at a point.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GpOrderParameter {
    pub amplitude: f64,
    pub phase: f64,
    /// Superfluid density n = |ψ|²
    pub density: f64,
}

/// Gross–Pitaevskii chemical potential / energy density report.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GpEnergyDensity {
    pub kinetic_density: f64,
    pub interaction_density: f64,
    pub trapping_density: f64,
    pub total_density: f64,
    pub chemical_potential: f64,
}

/// State of a single quantum vortex ring (circular vortex line).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct VortexRing {
    pub center: Vec3,
    /// Radius of the ring
    pub radius: f64,
    /// Circulation quantum number
    pub circulation_quantum: i32,
    /// Orientation axis (unit vector)
    pub axis: Vec3,
    /// Translational velocity (self-induced)
    pub velocity: Vec3,
}

/// Report from a vortex reconnection event.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct VortexReconnectionReport {
    /// Distance between two segments before reconnection
    pub closest_approach: f64,
    /// Whether a reconnection occurred
    pub reconnected: Bool,
    /// Post-reconnection segment 1 start
    pub seg1_start: Vec3,
    pub seg1_end: Vec3,
    /// Post-reconnection segment 2 start
    pub seg2_start: Vec3,
    pub seg2_end: Vec3,
    /// Energy dissipated during reconnection
    pub energy_dissipated: f64,
}

/// Quantised circulation around a closed loop.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct QuantisedCirculation {
    /// Circulation κ = n × h/m
    pub circulation: f64,
    /// Quantum number n
    pub quantum_number: i32,
    /// Circulation quantum h/m
    pub circulation_quantum: f64,
    /// Whether the circulation is consistent with quantisation
    pub quantised: Bool,
}

/// Parameters for time-dependent Gross–Pitaevskii integration.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GpTimeEvolutionParams {
    /// Healing length ξ
    pub healing_length: f64,
    /// Speed of sound c
    pub sound_speed: f64,
    /// Chemical potential μ
    pub chemical_potential: f64,
    /// Nonlinear coupling constant g
    pub coupling_constant: f64,
    /// Time step dt
    pub dt: f64,
}

impl Default for GpTimeEvolutionParams {
    fn default() -> Self {
        Self {
            healing_length: 1.0,
            sound_speed: 1.0,
            chemical_potential: 1.0,
            coupling_constant: 1.0,
            dt: 0.01,
        }
    }
}

/// Vortex filament network: a collection of vortex segments forming a tangle.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct VortexTangleStats {
    pub segment_count: u32,
    pub total_length: f64,
    pub average_curvature: f64,
    pub total_kinetic_energy: f64,
    pub vortex_line_density: f64,
}

/// A single point in a 2D cross-section of the GP wavefunction (for visualisation).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GpGridPoint {
    pub x: f64,
    pub y: f64,
    pub amplitude: f64,
    pub phase: f64,
    pub density: f64,
}

// ---------------------------------------------------------------------------
// Wave optics / diffraction structures
// ---------------------------------------------------------------------------

/// Complex wave amplitude at a point.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ComplexAmplitude {
    pub real: f64,
    pub imag: f64,
    /// Intensity I = |E|²
    pub intensity: f64,
}

/// Parameters for a monochromatic plane wave.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PlaneWaveParams {
    /// Wavenumber k = 2π/λ
    pub wavenumber: f64,
    /// Wavelength λ
    pub wavelength: f64,
    /// Initial amplitude A₀
    pub amplitude: f64,
    /// Initial phase φ₀
    pub phase_offset: f64,
}

impl Default for PlaneWaveParams {
    fn default() -> Self {
        Self {
            wavenumber: 2.0 * std::f64::consts::PI / 500e-9,
            wavelength: 500e-9,
            amplitude: 1.0,
            phase_offset: 0.0,
        }
    }
}

/// Huygens–Fresnel diffraction from an aperture (single point).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DiffractionPoint {
    /// Coordinates in the observation plane
    pub x: f64,
    pub y: f64,
    /// Complex amplitude at this point
    pub amplitude: ComplexAmplitude,
}

/// A single point source used in Huygens–Fresnel superposition.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PointSource {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Initial phase at this source point
    pub phase: f64,
    /// Amplitude scaling factor
    pub amplitude: f64,
}

/// Fresnel diffraction zone plate / Fresnel zone parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FresnelZoneReport {
    /// Radius of the n-th Fresnel zone
    pub zone_radius: f64,
    /// Zone index
    pub zone_index: u32,
    /// Phase contribution from this zone
    pub zone_phase: f64,
    /// Whether the zone is constructive (phase within ±π/2 of centre)
    pub constructive: Bool,
}

/// Thin-film interference report (single layer).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ThinFilmInterferenceReport {
    /// Optical path difference
    pub opd: f64,
    /// Phase difference from path
    pub phase_difference: f64,
    /// Reflection coefficient magnitude
    pub reflection_coefficient: f64,
    /// Interference intensity (normalised)
    pub intensity: f64,
    /// Whether half-wave loss occurs (n_film > n_substrate or similar)
    pub half_wave_loss: Bool,
    /// Wavelength for which this report was computed
    pub wavelength: f64,
}

/// Parameters for a thin film.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ThinFilmParams {
    /// Film thickness (m)
    pub thickness: f64,
    /// Film refractive index
    pub n_film: f64,
    /// Substrate refractive index
    pub n_substrate: f64,
    /// Incident medium refractive index (typically 1.0 for air)
    pub n_incident: f64,
    /// Angle of incidence (radians)
    pub incidence_angle: f64,
}

impl Default for ThinFilmParams {
    fn default() -> Self {
        Self {
            thickness: 500e-9,
            n_film: 1.5,
            n_substrate: 1.0,
            n_incident: 1.0,
            incidence_angle: 0.0,
        }
    }
}

/// Fresnel–Kirchhoff diffraction integral result for a single observation point.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct KirchhoffDiffractionPoint {
    pub x: f64,
    pub y: f64,
    pub amplitude: ComplexAmplitude,
    /// Obliquity (inclination) factor cosθ
    pub obliquity_factor: f64,
}

/// A single spherical wave emitted from a point source.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SphericalWavePoint {
    pub amplitude: ComplexAmplitude,
    /// Distance from source
    pub radius: f64,
    /// 1/r amplitude decay factor
    pub decay_factor: f64,
}

/// Describes a planar aperture for diffraction calculations.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ApertureDesc {
    /// Half-width in x (m)
    pub half_width_x: f64,
    /// Half-width in y (m)
    pub half_width_y: f64,
    /// Centre position in the aperture plane
    pub center_x: f64,
    pub center_y: f64,
    /// Transmission coefficient (0=opaque, 1=fully transparent)
    pub transmission: f64,
}

/// Two-slit (Young's) interference pattern at a point.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct YoungSlitPoint {
    pub x: f64,
    pub y: f64,
    /// Phase difference between slits
    pub phase_difference: f64,
    /// Path difference in metres
    pub path_difference: f64,
    /// Interference intensity
    pub intensity: f64,
    /// Envelope (single-slit diffraction) factor
    pub envelope_factor: f64,
}

// ---------------------------------------------------------------------------
// Plasma physics structures
// ---------------------------------------------------------------------------

/// A single macroparticle used in the PIC (particle-in-cell) method.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PicParticle {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    /// Charge (C), negative for electrons
    pub charge: f64,
    /// Mass (kg)
    pub mass: f64,
    /// Weight (number of real particles this macroparticle represents)
    pub weight: f64,
}

/// Electromagnetic fields on a 3D grid cell (staggered / Yee-like).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GridField {
    pub ex: f64,
    pub ey: f64,
    pub ez: f64,
    pub bx: f64,
    pub by: f64,
    pub bz: f64,
}

/// Charge density on a grid cell (from particle deposition).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ChargeDensityCell {
    pub rho: f64,
    pub jx: f64,
    pub jy: f64,
    pub jz: f64,
}

/// Debye length and plasma frequency report.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PlasmaParamsReport {
    /// Electron Debye length λ_D = sqrt(ε₀ k_B T_e / (n_e e²))
    pub debye_length: f64,
    /// Electron plasma frequency ω_pe = sqrt(n_e e² / (ε₀ m_e))
    pub plasma_frequency: f64,
    /// Ion plasma frequency ω_pi = sqrt(n_i Z² e² / (ε₀ m_i))
    pub ion_plasma_frequency: f64,
    /// Number of particles in a Debye sphere N_D
    pub debye_sphere_count: f64,
    /// Thermal velocity v_th = sqrt(k_B T_e / m_e)
    pub thermal_velocity: f64,
}

/// Vlasov equation reduced distribution function moment report.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct VlasovMomentReport {
    /// Number density n
    pub density: f64,
    /// Bulk velocity u (drift)
    pub ux: f64,
    pub uy: f64,
    pub uz: f64,
    /// Pressure tensor trace / temperature (energy density)
    pub temperature: f64,
    /// Heat flux vector (reduced)
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
}

/// Magnetic X-point (reconnection site) report.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MagneticXPoint {
    /// Position of the X-point
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// In-plane magnetic shear angle (radians)
    pub shear_angle: f64,
    /// Reconnection rate estimate (normalised)
    pub reconnection_rate: f64,
    /// Whether this is a valid X-point (B = 0 in the reconnection plane)
    pub valid: Bool,
}

/// PIC simulation step report (self-consistent field solve summary).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PicStepReport {
    pub particle_count: u32,
    pub max_density: f64,
    pub max_electric_field: f64,
    pub max_magnetic_field: f64,
    pub total_kinetic_energy: f64,
    pub total_field_energy: f64,
}

/// Parameters for the Boris particle pusher.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BorisPusherParams {
    pub dt: f64,
    pub charge_to_mass_ratio: f64,
}

impl Default for BorisPusherParams {
    fn default() -> Self {
        Self {
            dt: 1e-12,
            charge_to_mass_ratio: -1.758_820_010e11, // e/m_e for electrons
        }
    }
}

// ---------------------------------------------------------------------------
// Event callback registry — zero-FFI-roundtrip event dispatch
// ---------------------------------------------------------------------------

/// Opaque handle returned by `world_register_*_callback` — used to unregister.
pub type EventCallbackHandle = u64;

/// Callback signature: called from the Rapier physics step for each event.
/// Must be signal-safe (no Java upcalls, no locks from callback context).
pub type CollisionEventCallback = Option<
    unsafe extern "C" fn(
        world: *const std::ffi::c_void,
        event: *const CollisionEventRecord,
        user_data: *mut std::ffi::c_void,
    ),
>;

/// Callback signature for contact-force events.
pub type ContactForceEventCallback = Option<
    unsafe extern "C" fn(
        world: *const std::ffi::c_void,
        event: *const ContactForceEventRecord,
        user_data: *mut std::ffi::c_void,
    ),
>;

/// Dispatch mode for cached event delivery.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub enum EventDispatchMode {
    /// Events stay in the in-memory ring buffer; Java polls via existing APIs.
    #[default]
    Poll = 0,
    /// Events are dispatched through registered callbacks during `world_step`.
    Callback = 1,
    /// Events go to both the ring buffer and registered callbacks.
    Both = 2,
}

/// Pre-allocated ring buffer for zero-allocation event caching.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct EventRingBufferStats {
    /// Total capacity of the ring buffer (in event records).
    pub capacity: u32,
    /// Number of events currently in the buffer.
    pub len: u32,
    /// Number of events dropped due to buffer overflow since last reset.
    pub dropped: u32,
    /// Whether the buffer has wrapped around (overwritten old events).
    pub wrapped: Bool,
}

impl Default for EventRingBufferStats {
    fn default() -> Self {
        Self {
            capacity: 0,
            len: 0,
            dropped: 0,
            wrapped: Bool::FALSE,
        }
    }
}
