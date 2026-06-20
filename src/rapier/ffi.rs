use rapier3d::math::{Pose, Rotation, Vector};
use rapier3d::parry::query::ShapeCastOptions;
use rapier3d::parry::shape::SharedShape;
use rapier3d::prelude::{
    ActiveEvents, ActiveHooks, ColliderHandle, Group,
    ImpulseJointHandle as RapierImpulseJointHandle, InteractionGroups, InteractionTestMode,
    JointAxis, QueryFilter, QueryFilterFlags, RigidBodyHandle,
};

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
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ShapeType {
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

impl Default for ShapeType {
    fn default() -> Self {
        Self::Ball
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
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NeuralActivation {
    Relu = 0,
    Tanh = 1,
    Sin = 2,
    Linear = 3,
}

impl Default for NeuralActivation {
    fn default() -> Self {
        Self::Relu
    }
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

pub type RigidBodyHandleRaw = u64;
pub type ColliderHandleRaw = u64;
pub type ImpulseJointHandleRaw = u64;

pub(crate) const MAX_OUTPUT_CAPACITY: u32 = 1_000_000;
pub(crate) const MAX_TREE_ENTRIES: usize = 1_000_000;

const INVALID_HANDLE_RAW: u64 = u64::MAX;

fn pack_handle_parts(id: u32, generation: u32) -> u64 {
    (((generation as u64) << 32) | (id as u64)).wrapping_add(1)
}

fn unpack_handle_parts(handle: u64) -> (u32, u32) {
    let raw = handle.checked_sub(1).unwrap_or(INVALID_HANDLE_RAW);
    ((raw & 0xffff_ffff) as u32, (raw >> 32) as u32)
}

pub(crate) fn vec3_to_rapier(value: Vec3) -> Vector {
    Vector::new(value.x, value.y, value.z)
}

pub(crate) fn vec3_finite(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

pub(crate) fn vec3_from_rapier(value: Vector) -> Vec3 {
    Vec3 {
        x: value.x,
        y: value.y,
        z: value.z,
    }
}

pub(crate) fn quat_to_rapier(value: Quat) -> Rotation {
    Rotation::from_xyzw(value.i, value.j, value.k, value.w)
}

pub(crate) fn quat_finite(value: Quat) -> bool {
    value.i.is_finite() && value.j.is_finite() && value.k.is_finite() && value.w.is_finite()
}

pub(crate) fn quat_from_rapier(value: Rotation) -> Quat {
    Quat {
        i: value.x,
        j: value.y,
        k: value.z,
        w: value.w,
    }
}

pub(crate) fn isometry_from_parts(translation: Vec3, rotation: Quat) -> Pose {
    Pose::from_parts(vec3_to_rapier(translation), quat_to_rapier(rotation))
}

pub(crate) fn pack_rigid_body_handle(handle: RigidBodyHandle) -> RigidBodyHandleRaw {
    let (id, generation) = handle.into_raw_parts();
    pack_handle_parts(id, generation)
}

pub(crate) fn unpack_rigid_body_handle(handle: RigidBodyHandleRaw) -> RigidBodyHandle {
    let (id, generation) = unpack_handle_parts(handle);
    RigidBodyHandle::from_raw_parts(id, generation)
}

pub(crate) fn pack_collider_handle(handle: ColliderHandle) -> ColliderHandleRaw {
    let (id, generation) = handle.into_raw_parts();
    pack_handle_parts(id, generation)
}

pub(crate) fn unpack_collider_handle(handle: ColliderHandleRaw) -> ColliderHandle {
    let (id, generation) = unpack_handle_parts(handle);
    ColliderHandle::from_raw_parts(id, generation)
}

pub(crate) fn pack_impulse_joint_handle(handle: RapierImpulseJointHandle) -> ImpulseJointHandleRaw {
    let (id, generation) = handle.into_raw_parts();
    pack_handle_parts(id, generation)
}

pub(crate) fn unpack_impulse_joint_handle(
    handle: ImpulseJointHandleRaw,
) -> RapierImpulseJointHandle {
    let (id, generation) = unpack_handle_parts(handle);
    RapierImpulseJointHandle::from_raw_parts(id, generation)
}

pub(crate) fn body_status_to_rapier(status: BodyStatus) -> rapier3d::prelude::RigidBodyType {
    match status {
        BodyStatus::Dynamic => rapier3d::prelude::RigidBodyType::Dynamic,
        BodyStatus::Fixed => rapier3d::prelude::RigidBodyType::Fixed,
        BodyStatus::KinematicPositionBased => {
            rapier3d::prelude::RigidBodyType::KinematicPositionBased
        }
        BodyStatus::KinematicVelocityBased => {
            rapier3d::prelude::RigidBodyType::KinematicVelocityBased
        }
    }
}

pub(crate) fn body_status_from_raw(value: u32) -> BodyStatus {
    match value {
        0 => BodyStatus::Dynamic,
        1 => BodyStatus::Fixed,
        2 => BodyStatus::KinematicPositionBased,
        3 => BodyStatus::KinematicVelocityBased,
        _ => BodyStatus::Fixed,
    }
}

pub(crate) fn body_status_from_rapier(status: rapier3d::prelude::RigidBodyType) -> BodyStatus {
    match status {
        rapier3d::prelude::RigidBodyType::Dynamic => BodyStatus::Dynamic,
        rapier3d::prelude::RigidBodyType::Fixed => BodyStatus::Fixed,
        rapier3d::prelude::RigidBodyType::KinematicPositionBased => {
            BodyStatus::KinematicPositionBased
        }
        rapier3d::prelude::RigidBodyType::KinematicVelocityBased => {
            BodyStatus::KinematicVelocityBased
        }
    }
}

pub(crate) fn body_status_to_raw(status: BodyStatus) -> u32 {
    status as u32
}

pub(crate) fn shape_type_from_raw(value: u32) -> ShapeType {
    match value {
        1 => ShapeType::Cuboid,
        2 => ShapeType::CapsuleY,
        3 => ShapeType::CapsuleX,
        4 => ShapeType::CapsuleZ,
        5 => ShapeType::Cylinder,
        6 => ShapeType::RoundCylinder,
        7 => ShapeType::Cone,
        8 => ShapeType::RoundCone,
        9 => ShapeType::RoundCuboid,
        _ => ShapeType::Ball,
    }
}

pub(crate) fn shape_from_desc(desc: ShapeDesc) -> SharedShape {
    match shape_type_from_raw(desc.shape_type) {
        ShapeType::Ball => SharedShape::ball(desc.a),
        ShapeType::Cuboid => SharedShape::cuboid(desc.a, desc.b, desc.c),
        ShapeType::CapsuleY => SharedShape::capsule_y(desc.a, desc.b),
        ShapeType::CapsuleX => SharedShape::capsule_x(desc.a, desc.b),
        ShapeType::CapsuleZ => SharedShape::capsule_z(desc.a, desc.b),
        ShapeType::Cylinder => SharedShape::cylinder(desc.a, desc.b),
        ShapeType::RoundCylinder => SharedShape::round_cylinder(desc.a, desc.b, desc.c),
        ShapeType::Cone => SharedShape::cone(desc.a, desc.b),
        ShapeType::RoundCone => SharedShape::round_cone(desc.a, desc.b, desc.c),
        ShapeType::RoundCuboid => SharedShape::round_cuboid(desc.a, desc.b, desc.c, desc.d),
    }
}

pub(crate) fn shape_desc_valid(desc: ShapeDesc) -> bool {
    if !desc.a.is_finite() || !desc.b.is_finite() || !desc.c.is_finite() || !desc.d.is_finite() {
        return false;
    }

    match shape_type_from_raw(desc.shape_type) {
        ShapeType::Ball => desc.a > 0.0,
        ShapeType::Cuboid => desc.a > 0.0 && desc.b > 0.0 && desc.c > 0.0,
        ShapeType::CapsuleY | ShapeType::CapsuleX | ShapeType::CapsuleZ => {
            desc.a > 0.0 && desc.b > 0.0
        }
        ShapeType::Cylinder | ShapeType::Cone => desc.a > 0.0 && desc.b > 0.0,
        ShapeType::RoundCylinder | ShapeType::RoundCone => {
            desc.a > 0.0 && desc.b > 0.0 && desc.c >= 0.0
        }
        ShapeType::RoundCuboid => desc.a > 0.0 && desc.b > 0.0 && desc.c > 0.0 && desc.d >= 0.0,
    }
}

pub(crate) fn voxel_collider_mode_from_raw(value: u32) -> VoxelColliderMode {
    match value {
        1 => VoxelColliderMode::Cuboids,
        2 => VoxelColliderMode::GreedyCuboids,
        3 => VoxelColliderMode::SurfaceMesh,
        _ => VoxelColliderMode::Auto,
    }
}

pub(crate) fn neural_activation_from_raw(value: u32) -> NeuralActivation {
    match value {
        1 => NeuralActivation::Tanh,
        2 => NeuralActivation::Sin,
        3 => NeuralActivation::Linear,
        _ => NeuralActivation::Relu,
    }
}

pub(crate) fn kdop_preset_from_raw(value: u32) -> KdopPreset {
    match value {
        14 => KdopPreset::K14,
        18 => KdopPreset::K18,
        26 => KdopPreset::K26,
        _ => KdopPreset::K6,
    }
}

pub(crate) fn joint_type_from_raw(value: u32) -> JointTypeDesc {
    match value {
        1 => JointTypeDesc::Revolute,
        2 => JointTypeDesc::Prismatic,
        3 => JointTypeDesc::Rope,
        4 => JointTypeDesc::Spring,
        5 => JointTypeDesc::Spherical,
        _ => JointTypeDesc::Fixed,
    }
}

pub(crate) fn interaction_groups_to_rapier(groups: InteractionGroupsDesc) -> InteractionGroups {
    InteractionGroups::new(
        Group::from_bits_truncate(groups.memberships),
        Group::from_bits_truncate(groups.filter),
        InteractionTestMode::And,
    )
}

pub(crate) fn active_events_from_bits(bits: u32) -> ActiveEvents {
    ActiveEvents::from_bits_truncate(bits)
}

pub(crate) fn active_hooks_from_bits(bits: u32) -> ActiveHooks {
    ActiveHooks::from_bits_truncate(bits)
}

pub(crate) fn query_filter_from_desc(desc: QueryFilterDesc) -> QueryFilter<'static> {
    let mut filter = QueryFilter::from(QueryFilterFlags::from_bits_truncate(desc.flags));

    if desc.use_groups.0 != 0 {
        filter = filter.groups(interaction_groups_to_rapier(desc.groups));
    }
    if desc.use_exclude_collider.0 != 0 {
        filter = filter.exclude_collider(unpack_collider_handle(desc.exclude_collider));
    }
    if desc.use_exclude_rigid_body.0 != 0 {
        filter = filter.exclude_rigid_body(unpack_rigid_body_handle(desc.exclude_rigid_body));
    }

    filter
}

pub(crate) fn shape_cast_options_to_rapier(options: ShapeCastOptionsDesc) -> ShapeCastOptions {
    ShapeCastOptions {
        max_time_of_impact: options.max_time_of_impact,
        target_distance: options.target_distance,
        stop_at_penetration: options.stop_at_penetration.0 != 0,
        compute_impact_geometry_on_penetration: options.compute_impact_geometry_on_penetration.0
            != 0,
    }
}

pub(crate) fn joint_axis_to_rapier(axis: JointAxisDesc) -> JointAxis {
    match axis {
        JointAxisDesc::LinX => JointAxis::LinX,
        JointAxisDesc::LinY => JointAxis::LinY,
        JointAxisDesc::LinZ => JointAxis::LinZ,
        JointAxisDesc::AngX => JointAxis::AngX,
        JointAxisDesc::AngY => JointAxis::AngY,
        JointAxisDesc::AngZ => JointAxis::AngZ,
    }
}

pub(crate) fn joint_axis_from_raw(value: u32) -> JointAxisDesc {
    match value {
        1 => JointAxisDesc::LinY,
        2 => JointAxisDesc::LinZ,
        3 => JointAxisDesc::AngX,
        4 => JointAxisDesc::AngY,
        5 => JointAxisDesc::AngZ,
        _ => JointAxisDesc::LinX,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packed_arena_handles_reserve_zero_for_null() {
        let body = RigidBodyHandle::from_raw_parts(0, 0);
        let collider = ColliderHandle::from_raw_parts(0, 0);
        let joint = RapierImpulseJointHandle::from_raw_parts(0, 0);

        assert_ne!(pack_rigid_body_handle(body), 0);
        assert_ne!(pack_collider_handle(collider), 0);
        assert_ne!(pack_impulse_joint_handle(joint), 0);

        assert_eq!(
            unpack_rigid_body_handle(pack_rigid_body_handle(body)).into_raw_parts(),
            (0, 0)
        );
        assert_eq!(
            unpack_collider_handle(pack_collider_handle(collider)).into_raw_parts(),
            (0, 0)
        );
        assert_eq!(
            unpack_impulse_joint_handle(pack_impulse_joint_handle(joint)).into_raw_parts(),
            (0, 0)
        );
    }
}
