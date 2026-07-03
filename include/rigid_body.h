#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#pragma once

/* Generated with cbindgen:0.29.4 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define ABI_VERSION 1

typedef enum BodyStatus {
  Dynamic = 0,
  Fixed = 1,
  KinematicPositionBased = 2,
  KinematicVelocityBased = 3,
} BodyStatus;

typedef enum JointAxisDesc {
  LinX = 0,
  LinY = 1,
  LinZ = 2,
  AngX = 3,
  AngY = 4,
  AngZ = 5,
} JointAxisDesc;

typedef enum JointTypeDesc {
  Fixed = 0,
  Revolute = 1,
  Prismatic = 2,
  Rope = 3,
  Spring = 4,
  Spherical = 5,
} JointTypeDesc;

typedef enum KdopPreset {
  K6 = 6,
  K14 = 14,
  K18 = 18,
  K26 = 26,
} KdopPreset;

typedef enum NeuralActivation {
  Relu = 0,
  Tanh = 1,
  Sin = 2,
  Linear = 3,
} NeuralActivation;

typedef enum ShapeType {
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
} ShapeType;

typedef enum VoxelColliderMode {
  Auto = 0,
  Cuboids = 1,
  GreedyCuboids = 2,
  SurfaceMesh = 3,
} VoxelColliderMode;

typedef struct AnvilKitAppHandle AnvilKitAppHandle;

typedef struct CRbTreeHandle CRbTreeHandle;

typedef struct CharacterControllerHandle CharacterControllerHandle;

typedef struct ColliderBuilderHandle ColliderBuilderHandle;

typedef struct JointBuilderHandle JointBuilderHandle;

typedef struct RTreeHandle RTreeHandle;

typedef struct RigidBodyBuilderHandle RigidBodyBuilderHandle;

typedef struct WorldHandle WorldHandle;

typedef struct Bool {
  uint8_t _0;
} Bool;
#define Bool_FALSE (Bool){ ._0 = 0 }
#define Bool_TRUE (Bool){ ._0 = 1 }

typedef struct ModalAnalysisReport {
  uint32_t dof;
  uint32_t mode_count;
  uint32_t stable_mode_count;
  double max_frequency_hz;
} ModalAnalysisReport;

typedef struct StructuralModeReport {
  double angular_frequency;
  double frequency_hz;
  double damping_ratio;
  double damped_frequency_hz;
  double critical_damping;
} StructuralModeReport;

typedef struct AcousticWaveReport {
  uint32_t cell_count;
  double max_pressure;
  double acoustic_energy;
} AcousticWaveReport;

typedef struct AcousticResonanceReport {
  struct Bool resonant;
  uint32_t nearest_mode_index;
  double nearest_frequency_hz;
  double frequency_delta_hz;
  double amplification_estimate;
} AcousticResonanceReport;

typedef struct AcousticMaterial {
  double density;
  double hardness;
  double damping;
  double roughness;
  double restitution;
  double sound_speed;
} AcousticMaterial;

typedef struct AcousticContactDesc {
  double normal_force;
  double normal_velocity;
  double tangential_velocity;
  double contact_area;
  double dt;
} AcousticContactDesc;

typedef struct AcousticExcitationReport {
  double impulse;
  double normal_component;
  double scrape_component;
  double brightness;
  double damping;
  double amplitude;
} AcousticExcitationReport;

typedef struct ModalSynthesisReport {
  uint32_t mode_count;
  double sample;
  double peak_modal_displacement;
  double modal_energy;
} ModalSynthesisReport;

typedef struct Vec3 {
  double x;
  double y;
  double z;
} Vec3;

typedef struct SpatializedSample {
  double left;
  double right;
  double distance;
  double attenuation;
  double pan;
} SpatializedSample;

typedef uint64_t RigidBodyHandleRaw;

typedef struct AeroSurface {
  struct Vec3 point;
  struct Vec3 normal;
  double area;
  double drag_coefficient;
  double lift_coefficient;
} AeroSurface;

typedef struct AeroForceReport {
  struct Vec3 total_force;
  struct Vec3 total_torque;
  uint32_t surface_count;
  uint32_t active_surface_count;
} AeroForceReport;

typedef struct Quat {
  double i;
  double j;
  double k;
  double w;
} Quat;

typedef struct ShapeDesc {
  uint32_t shape_type;
  double a;
  double b;
  double c;
  double d;
} ShapeDesc;

typedef struct MaterialProperties {
  double density;
  double friction;
  double restitution;
  double youngs_modulus;
  double poisson_ratio;
  double thermal_expansion;
} MaterialProperties;

typedef uint64_t ColliderHandleRaw;

typedef uint64_t ImpulseJointHandleRaw;

typedef struct FluidVolume {
  struct Vec3 center;
  struct Vec3 half_extents;
  double density;
  double linear_drag;
  double quadratic_drag;
  double angular_drag;
  struct Vec3 flow_velocity;
  struct Vec3 gravity;
} FluidVolume;

typedef struct FluidForceReport {
  struct Vec3 buoyancy_force;
  struct Vec3 drag_force;
  struct Vec3 angular_damping_torque;
  struct Vec3 total_force;
  struct Vec3 total_torque;
  double submerged_fraction;
  double displaced_volume;
} FluidForceReport;

typedef struct TrajectoryEnvironment {
  struct Vec3 gravity;
  struct Vec3 flow_velocity;
  double mass;
  double reference_area;
  double density;
  double drag_coefficient;
  double lift_coefficient;
  struct Vec3 lift_direction;
} TrajectoryEnvironment;

typedef struct TrajectoryForceReport {
  struct Vec3 gravity_force;
  struct Vec3 drag_force;
  struct Vec3 lift_force;
  struct Vec3 total_force;
  struct Vec3 acceleration;
} TrajectoryForceReport;

typedef struct StressStrainReport {
  double strain;
  double stress;
  double elastic_energy_density;
  double thermal_strain;
} StressStrainReport;

typedef struct HertzContactReport {
  double effective_modulus;
  double effective_radius;
  double contact_radius;
  double contact_area;
  double normal_force;
  double stiffness;
  double damping_force;
  double total_force;
} HertzContactReport;

typedef struct NBodyParticle {
  struct Vec3 position;
  struct Vec3 velocity;
  double mass;
} NBodyParticle;

typedef struct NBodySolverParams {
  double gravitational_constant;
  double softening;
  double opening_angle;
  uint32_t multipole_order;
} NBodySolverParams;

typedef struct NBodyForceReport {
  uint32_t body_count;
  uint32_t approximate_node_count;
  uint32_t direct_pair_count;
  double max_acceleration;
  double potential_energy;
} NBodyForceReport;

typedef struct RelativisticOrbitReport {
  double schwarzschild_radius;
  double periapsis_precession_per_orbit;
  struct Vec3 correction_acceleration;
} RelativisticOrbitReport;

typedef struct RocheLimitReport {
  double fluid_roche_limit;
  double rigid_roche_limit;
  struct Bool inside_fluid_limit;
  struct Bool inside_rigid_limit;
} RocheLimitReport;

typedef struct OrbitalResonanceReport {
  uint32_t ratio_numerator;
  uint32_t ratio_denominator;
  double actual_ratio;
  double target_ratio;
  double relative_error;
  struct Bool resonant;
} OrbitalResonanceReport;

typedef struct HillMuscleDesc {
  double max_isometric_force;
  double optimal_fiber_length;
  double tendon_slack_length;
  double max_contraction_velocity;
  double parallel_stiffness;
  double series_stiffness;
  double damping;
  double pennation_angle;
} HillMuscleDesc;

typedef struct HillMuscleState {
  double activation;
  double fiber_length;
  double fiber_velocity;
  double tendon_length;
  double moment_arm;
} HillMuscleState;

typedef struct HillMuscleReport {
  double active_force;
  double parallel_elastic_force;
  double series_elastic_force;
  double damping_force;
  double total_fiber_force;
  double tendon_force;
  double joint_torque;
  double force_length_factor;
  double force_velocity_factor;
} HillMuscleReport;

typedef struct SkeletalJointLimit {
  double min_angle;
  double max_angle;
  double stiffness;
  double damping;
} SkeletalJointLimit;

typedef struct SkeletalConstraintReport {
  double clamped_angle;
  double angle_error;
  double corrective_torque;
  struct Bool limited;
} SkeletalConstraintReport;

typedef struct Capsule {
  struct Vec3 a;
  struct Vec3 b;
  double radius;
} Capsule;

typedef struct Ssv {
  struct Vec3 a;
  struct Vec3 b;
  double radius;
} Ssv;

typedef struct Ellipsoid {
  struct Vec3 center;
  struct Vec3 radii;
  struct Quat rotation;
  uint32_t segments;
} Ellipsoid;

typedef struct Prism {
  struct Vec3 center;
  double radius;
  double half_height;
  uint32_t sides;
  struct Quat rotation;
} Prism;

typedef struct Cylinder {
  struct Vec3 center;
  double radius;
  double half_height;
  struct Quat rotation;
} Cylinder;

typedef struct SphericalShell {
  struct Vec3 center;
  double inner_radius;
  double outer_radius;
} SphericalShell;

typedef struct InteractionGroupsDesc {
  uint32_t memberships;
  uint32_t filter;
} InteractionGroupsDesc;

typedef struct QueryFilterDesc {
  uint32_t flags;
  struct InteractionGroupsDesc groups;
  struct Bool use_groups;
  ColliderHandleRaw exclude_collider;
  struct Bool use_exclude_collider;
  RigidBodyHandleRaw exclude_rigid_body;
  struct Bool use_exclude_rigid_body;
} QueryFilterDesc;

/**
 * Lorenz attractor state at a single time step.
 */
typedef struct LorenzState {
  double x;
  double y;
  double z;
} LorenzState;

/**
 * Parameters for the Lorenz system: dx/dt = sigma*(y-x), dy/dt = x*(rho-z)-y, dz/dt = x*y - beta*z.
 */
typedef struct LorenzParams {
  double sigma;
  double rho;
  double beta;
  double dt;
} LorenzParams;

/**
 * Full Lorenz integration report at a step.
 */
typedef struct LorenzStepReport {
  struct LorenzState state;
  double dx;
  double dy;
  double dz;
} LorenzStepReport;

/**
 * Lyapunov exponent estimation report for a single trajectory.
 */
typedef struct LyapunovReport {
  /**
   * Largest Lyapunov exponent (bits/s or nats/s depending on log base)
   */
  double largest_exponent;
  /**
   * Convergence indicator: number of orbit steps used
   */
  uint32_t convergence_steps;
  /**
   * Whether the exponent is positive (chaotic) within the tolerance
   */
  struct Bool positive;
} LyapunovReport;

/**
 * A single bifurcation point: parameter value vs. sampled state.
 */
typedef struct BifurcationPoint {
  double parameter;
  double sample;
} BifurcationPoint;

/**
 * Double pendulum state (generalised coordinates and their derivatives).
 */
typedef struct DoublePendulumState {
  /**
   * Angle of upper pendulum (radians)
   */
  double theta1;
  /**
   * Angle of lower pendulum (radians)
   */
  double theta2;
  /**
   * Angular velocity of upper pendulum (rad/s)
   */
  double omega1;
  /**
   * Angular velocity of lower pendulum (rad/s)
   */
  double omega2;
} DoublePendulumState;

/**
 * Double pendulum parameters (geometry and integration step).
 */
typedef struct DoublePendulumParams {
  /**
   * Mass of upper bob
   */
  double m1;
  /**
   * Mass of lower bob
   */
  double m2;
  /**
   * Length of upper rod
   */
  double l1;
  /**
   * Length of lower rod
   */
  double l2;
  /**
   * Gravitational acceleration
   */
  double g;
  /**
   * Integration time step
   */
  double dt;
} DoublePendulumParams;

/**
 * Double-pendulum acceleration report (RK4 intermediate computation).
 */
typedef struct DoublePendulumAccel {
  double alpha1;
  double alpha2;
} DoublePendulumAccel;

/**
 * Parameters controlling chaos detection heuristics.
 */
typedef struct ChaosDetectionParams {
  /**
   * Number of orbit steps to sample
   */
  uint32_t sample_steps;
  /**
   * Embedding dimension for delay-coordinate reconstruction
   */
  uint32_t embedding_dim;
  /**
   * Delay (in steps) for reconstruction
   */
  uint32_t embedding_delay;
  /**
   * Neighbourhood radius for correlation dimension
   */
  double neighbourhood_radius;
  /**
   * Threshold above which Lyapunov exponent is considered chaotic
   */
  double chaotic_threshold;
} ChaosDetectionParams;

/**
 * Report from a chaos detection analysis.
 */
typedef struct ChaosDetectionReport {
  /**
   * Largest Lyapunov exponent estimate
   */
  double lyapunov_exponent;
  /**
   * Correlation dimension estimate (box-counting style)
   */
  double correlation_dimension;
  /**
   * Whether the system is classified as chaotic
   */
  struct Bool is_chaotic;
  /**
   * Confidence metric between 0 and 1
   */
  double confidence;
} ChaosDetectionReport;

/**
 * Logistic map state (classic 1D chaos example).
 */
typedef struct LogisticMapState {
  double x;
  double r;
} LogisticMapState;

typedef struct Obb {
  struct Vec3 center;
  struct Vec3 half_extents;
  struct Quat rotation;
} Obb;

typedef struct Sphere {
  struct Vec3 center;
  double radius;
} Sphere;

typedef struct AabbDesc {
  struct Vec3 mins;
  struct Vec3 maxs;
} AabbDesc;

typedef struct FemTetrahedron {
  struct Vec3 a;
  struct Vec3 b;
  struct Vec3 c;
  struct Vec3 d;
} FemTetrahedron;

typedef struct FemShapeFunctionReport {
  double weights[4];
  struct Vec3 gradients[4];
  double volume;
  struct Bool inside;
} FemShapeFunctionReport;

typedef struct FemConstitutiveReport {
  double lambda;
  double shear_modulus;
  double bulk_modulus;
  uint32_t matrix_size;
} FemConstitutiveReport;

typedef struct NewmarkBetaParameters {
  double beta;
  double gamma;
  double dt;
} NewmarkBetaParameters;

typedef struct NewmarkBetaReport {
  uint32_t dof;
  double beta;
  double gamma;
  double dt;
  double effective_stiffness_scale;
  double effective_damping_scale;
  double max_delta_displacement;
  double residual_norm;
} NewmarkBetaReport;

typedef struct PidGains {
  double kp;
  double ki;
  double kd;
  double output_min;
  double output_max;
  double integral_min;
  double integral_max;
} PidGains;

typedef struct PidState {
  double integral;
  double previous_error;
} PidState;

typedef struct PidReport {
  double error;
  double integral;
  double derivative;
  double unclamped_output;
  double output;
} PidReport;

typedef struct StateSpaceReport {
  uint32_t state_count;
  uint32_t input_count;
  uint32_t output_count;
  double max_state_delta;
  double output_norm;
} StateSpaceReport;

typedef struct MpcConfig {
  uint32_t state_count;
  uint32_t input_count;
  uint32_t horizon;
  double dt;
  double control_min;
  double control_max;
  uint32_t gradient_iterations;
  double step_size;
} MpcConfig;

typedef struct MpcReport {
  uint32_t horizon;
  uint32_t iterations;
  double initial_cost;
  double final_cost;
  double first_control_norm;
} MpcReport;

typedef struct EffectiveCharacterMovement {
  struct Vec3 translation;
  struct Bool grounded;
  struct Bool is_sliding_down_slope;
} EffectiveCharacterMovement;

typedef struct ElectromagneticField {
  struct Vec3 electric;
  struct Vec3 magnetic;
} ElectromagneticField;

typedef struct LorentzForceReport {
  struct Vec3 electric_force;
  struct Vec3 magnetic_force;
  struct Vec3 total_force;
  struct Vec3 acceleration;
} LorentzForceReport;

typedef struct MagneticFluxReport {
  double flux;
  double normal_component;
  double area;
} MagneticFluxReport;

typedef struct FaradayInductionReport {
  double flux_rate;
  double induced_emf;
  double induced_current;
} FaradayInductionReport;

typedef struct MaxwellPointReport {
  struct ElectromagneticField next_field;
  struct Vec3 electric_derivative;
  struct Vec3 magnetic_derivative;
  double gauss_electric_residual;
  double gauss_magnetic_residual;
} MaxwellPointReport;

typedef struct FdtdYeeReport {
  uint32_t cell_count;
  double max_electric_delta;
  double max_magnetic_delta;
  double total_energy_density;
  double courant_number;
} FdtdYeeReport;

typedef struct CoulombFrictionLaw {
  double static_coefficient;
  double dynamic_coefficient;
  double velocity_threshold;
  struct Bool enabled;
} CoulombFrictionLaw;

typedef struct AirDragLaw {
  struct Vec3 fluid_velocity;
  double density;
  double dynamic_viscosity;
  double characteristic_length;
  double reference_area;
  double drag_coefficient;
  double reynolds_stokes_limit;
  struct Bool enabled;
} AirDragLaw;

typedef struct ExternalForceLaw {
  struct Bool buoyancy_enabled;
  double fluid_density;
  double displaced_volume;
  struct Vec3 buoyancy_gravity;
  struct Bool electromagnetic_enabled;
  double charge;
  struct Vec3 electric_field;
  struct Vec3 magnetic_field;
  struct Bool elastic_enabled;
  struct Vec3 spring_anchor;
  double spring_stiffness;
  double spring_damping;
  struct Bool gravity_enabled;
  struct Vec3 gravity_source;
  double gravitational_parameter;
  struct Bool enabled;
} ExternalForceLaw;

/**
 * Newtonian pairwise gravity configuration for body-body attraction.
 *
 * When enabled, every dynamic body attracts every other dynamic body via
 * Newton's law:  F = G · m₁ · m₂ / r².
 *
 * Set `gravitational_constant` to 6.67430e-11 for real-world gravity,
 * or a larger value for game-scale simulations.
 */
typedef struct NewtonGravityLaw {
  /**
   * Gravitational constant (default: 6.67430e-11 N·m²/kg²).
   * Use larger values for game-scale simulations.
   */
  double gravitational_constant;
  /**
   * Minimum distance to prevent division by zero (default: 0.01 m).
   */
  double min_distance;
  /**
   * Maximum distance for gravity to apply (0 = no limit).
   */
  double max_distance;
  struct Bool enabled;
} NewtonGravityLaw;

typedef struct CustomPhysicsReport {
  uint32_t body_count;
  uint32_t drag_body_count;
  uint32_t external_force_body_count;
  struct Vec3 total_drag_force;
  struct Vec3 total_external_force;
  double max_reynolds_number;
} CustomPhysicsReport;

typedef struct CollisionEventRecord {
  struct Bool started;
  ColliderHandleRaw collider1;
  ColliderHandleRaw collider2;
  struct Bool sensor;
  struct Bool removed;
} CollisionEventRecord;

typedef struct ContactForceEventRecord {
  ColliderHandleRaw collider1;
  ColliderHandleRaw collider2;
  struct Vec3 total_force;
  double total_force_magnitude;
  struct Vec3 max_force_direction;
  double max_force_magnitude;
} ContactForceEventRecord;

/**
 * Pre-allocated ring buffer for zero-allocation event caching.
 */
typedef struct EventRingBufferStats {
  /**
   * Total capacity of the ring buffer (in event records).
   */
  uint32_t capacity;
  /**
   * Number of events currently in the buffer.
   */
  uint32_t len;
  /**
   * Number of events dropped due to buffer overflow since last reset.
   */
  uint32_t dropped;
  /**
   * Whether the buffer has wrapped around (overwritten old events).
   */
  struct Bool wrapped;
} EventRingBufferStats;

/**
 * Opaque handle returned by `world_register_*_callback` — used to unregister.
 */
typedef uint64_t EventCallbackHandle;

typedef struct NavierStokesReport {
  struct Vec3 advection;
  struct Vec3 pressure_acceleration;
  struct Vec3 viscosity_acceleration;
  struct Vec3 external_acceleration;
  struct Vec3 total_acceleration;
  struct Vec3 next_velocity;
} NavierStokesReport;

typedef struct SphParticle {
  struct Vec3 position;
  struct Vec3 velocity;
  double mass;
  double density;
  double pressure;
} SphParticle;

typedef struct SphForceReport {
  double density;
  double pressure;
  struct Vec3 pressure_force;
  struct Vec3 viscosity_force;
  struct Vec3 surface_tension_force;
  struct Vec3 total_force;
} SphForceReport;

typedef struct BernoulliReport {
  double pressure;
  double velocity;
  double elevation;
  double total_head;
  double dynamic_pressure;
} BernoulliReport;

typedef struct StressIntensityReport {
  double stress_intensity;
  struct Bool critical;
  double safety_factor;
} StressIntensityReport;

typedef struct FractureMaterial {
  double youngs_modulus;
  double poisson_ratio;
  double fracture_toughness;
  double surface_energy;
  double density;
} FractureMaterial;

typedef struct GriffithReport {
  double critical_stress;
  double energy_release_rate;
  double critical_energy_release_rate;
  struct Bool will_fracture;
} GriffithReport;

typedef struct MinerDamageReport {
  double damage;
  double remaining_life_fraction;
  struct Bool failed;
} MinerDamageReport;

typedef struct SnCurveReport {
  double cycles_to_failure;
  struct Bool infinite_life;
} SnCurveReport;

typedef struct FractureEnergyReport {
  double available_energy;
  double surface_energy_required;
  double fragment_kinetic_energy;
  struct Bool will_fracture;
} FractureEnergyReport;

typedef struct FractureModeReport {
  uint32_t mode;
  double driving_stress;
  double mixed_mode_ratio;
} FractureModeReport;

typedef struct FractureFragmentDesc {
  struct Vec3 local_center;
  struct Vec3 half_extents;
  struct Vec3 initial_velocity;
  double density;
  double friction;
  double restitution;
} FractureFragmentDesc;

typedef struct FractureReplaceReport {
  uint32_t fragment_count;
  uint32_t joint_count;
  struct Bool removed_source;
} FractureReplaceReport;

typedef struct MolecularParticle {
  struct Vec3 position;
  struct Vec3 velocity;
  double mass;
  double charge;
  double epsilon;
  double sigma;
} MolecularParticle;

typedef struct MolecularForceLaw {
  double coulomb_constant;
  double relative_permittivity;
  double cutoff_radius;
  double softening;
  struct Bool lennard_jones_enabled;
  struct Bool coulomb_enabled;
} MolecularForceLaw;

typedef struct MolecularPairReport {
  struct Vec3 displacement;
  double distance;
  double lennard_jones_potential;
  double coulomb_potential;
  double total_potential;
  struct Vec3 lennard_jones_force;
  struct Vec3 coulomb_force;
  struct Vec3 total_force;
} MolecularPairReport;

typedef struct NeuralBoundsDesc {
  struct Vec3 center;
  struct Vec3 half_extents;
  struct Quat rotation;
  uint32_t sample_resolution;
  uint32_t hidden_width;
  uint32_t hidden_layers;
  uint32_t activation;
  double output_scale;
  double padding;
} NeuralBoundsDesc;

typedef struct CatalystEffect {
  double concentration;
  double strength;
  double saturation;
} CatalystEffect;

typedef struct CatalystReport {
  double rate_multiplier;
  double effective_rate;
} CatalystReport;

typedef struct GrayScottParams {
  double diffusion_u;
  double diffusion_v;
  double feed_rate;
  double kill_rate;
  double dx;
} GrayScottParams;

typedef struct GrayScottReactionReport {
  double reaction_rate;
  double diffusion_u_term;
  double diffusion_v_term;
  double du_dt;
  double dv_dt;
} GrayScottReactionReport;

typedef struct ReactionDiffusionReport {
  uint32_t cell_count;
  double max_delta_u;
  double max_delta_v;
  double total_u;
  double total_v;
  double max_reaction_rate;
} ReactionDiffusionReport;

typedef struct ConcentrationBuoyancyReport {
  double density;
  double density_delta;
  struct Vec3 buoyancy_acceleration;
  struct Vec3 buoyancy_force;
} ConcentrationBuoyancyReport;

/**
 * Debye length and plasma frequency report.
 */
typedef struct PlasmaParamsReport {
  /**
   * Electron Debye length λ_D = sqrt(ε₀ k_B T_e / (n_e e²))
   */
  double debye_length;
  /**
   * Electron plasma frequency ω_pe = sqrt(n_e e² / (ε₀ m_e))
   */
  double plasma_frequency;
  /**
   * Ion plasma frequency ω_pi = sqrt(n_i Z² e² / (ε₀ m_i))
   */
  double ion_plasma_frequency;
  /**
   * Number of particles in a Debye sphere N_D
   */
  double debye_sphere_count;
  /**
   * Thermal velocity v_th = sqrt(k_B T_e / m_e)
   */
  double thermal_velocity;
} PlasmaParamsReport;

/**
 * A single macroparticle used in the PIC (particle-in-cell) method.
 */
typedef struct PicParticle {
  double x;
  double y;
  double z;
  double vx;
  double vy;
  double vz;
  /**
   * Charge (C), negative for electrons
   */
  double charge;
  /**
   * Mass (kg)
   */
  double mass;
  /**
   * Weight (number of real particles this macroparticle represents)
   */
  double weight;
} PicParticle;

/**
 * Electromagnetic fields on a 3D grid cell (staggered / Yee-like).
 */
typedef struct GridField {
  double ex;
  double ey;
  double ez;
  double bx;
  double by;
  double bz;
} GridField;

/**
 * Parameters for the Boris particle pusher.
 */
typedef struct BorisPusherParams {
  double dt;
  double charge_to_mass_ratio;
} BorisPusherParams;

/**
 * Charge density on a grid cell (from particle deposition).
 */
typedef struct ChargeDensityCell {
  double rho;
  double jx;
  double jy;
  double jz;
} ChargeDensityCell;

/**
 * Vlasov equation reduced distribution function moment report.
 */
typedef struct VlasovMomentReport {
  /**
   * Number density n
   */
  double density;
  /**
   * Bulk velocity u (drift)
   */
  double ux;
  double uy;
  double uz;
  /**
   * Pressure tensor trace / temperature (energy density)
   */
  double temperature;
  /**
   * Heat flux vector (reduced)
   */
  double qx;
  double qy;
  double qz;
} VlasovMomentReport;

/**
 * Magnetic X-point (reconnection site) report.
 */
typedef struct MagneticXPoint {
  /**
   * Position of the X-point
   */
  double x;
  double y;
  double z;
  /**
   * In-plane magnetic shear angle (radians)
   */
  double shear_angle;
  /**
   * Reconnection rate estimate (normalised)
   */
  double reconnection_rate;
  /**
   * Whether this is a valid X-point (B = 0 in the reconnection plane)
   */
  struct Bool valid;
} MagneticXPoint;

/**
 * PIC simulation step report (self-consistent field solve summary).
 */
typedef struct PicStepReport {
  uint32_t particle_count;
  double max_density;
  double max_electric_field;
  double max_magnetic_field;
  double total_kinetic_energy;
  double total_field_energy;
} PicStepReport;

typedef struct QuantumWaveFunction {
  double amplitude_real;
  double amplitude_imag;
} QuantumWaveFunction;

typedef struct QuantumBarrier {
  double particle_energy;
  double barrier_potential;
  double barrier_width;
  double particle_mass;
  double reduced_planck;
} QuantumBarrier;

typedef struct QuantumTunnelingReport {
  double wave_number;
  double decay_constant;
  double exponent;
  double transmission_coefficient;
  double reflection_coefficient;
} QuantumTunnelingReport;

typedef struct QuantumOscillatorReport {
  double angular_frequency;
  double zero_point_energy;
  double first_excited_energy;
  double level_spacing;
} QuantumOscillatorReport;

typedef struct RayHit {
  ColliderHandleRaw collider;
  double time_of_impact;
  struct Vec3 normal;
  uint32_t feature;
} RayHit;

typedef struct PointProjection {
  struct Vec3 point;
  struct Bool is_inside;
} PointProjection;

typedef struct ShapeCastHit {
  ColliderHandleRaw collider;
  double time_of_impact;
  struct Vec3 witness1;
  struct Vec3 witness2;
  struct Vec3 normal1;
  struct Vec3 normal2;
  uint32_t status;
} ShapeCastHit;

typedef struct ShapeCastOptionsDesc {
  double max_time_of_impact;
  double target_distance;
  struct Bool stop_at_penetration;
  struct Bool compute_impact_geometry_on_penetration;
} ShapeCastOptionsDesc;

typedef struct LorentzBoost {
  /**
   * 4x4 Lorentz boost matrix in row-major order acting on (ct, x, y, z)
   */
  double m00;
  double m01;
  double m02;
  double m03;
  double m10;
  double m11;
  double m12;
  double m13;
  double m20;
  double m21;
  double m22;
  double m23;
  double m30;
  double m31;
  double m32;
  double m33;
} LorentzBoost;

typedef struct LorentzTransformedFrame {
  /**
   * Time component in the boosted frame (c * t')
   */
  double ct_prime;
  double x_prime;
  double y_prime;
  double z_prime;
} LorentzTransformedFrame;

typedef struct SchwarzschildMetric {
  /**
   * Time-time metric coefficient g_tt = -(1 - rs/r)
   */
  double g_tt;
  /**
   * Radial-radial metric coefficient g_rr = 1/(1 - rs/r)
   */
  double g_rr;
  /**
   * Schwarzschild radius rs = 2GM/c^2
   */
  double schwarzschild_radius;
  /**
   * Ratio r/rs
   */
  double radius_over_rs;
} SchwarzschildMetric;

typedef struct GravitationalTimeDilation {
  /**
   * dtau/dt for a stationary observer at radius r = sqrt(1 - rs/r)
   */
  double stationary_factor;
  /**
   * dtau/dt for a circular orbiting observer = sqrt(1 - 3*rs/(2*r))
   */
  double orbital_factor;
  /**
   * Newtonian orbital speed at radius r = sqrt(GM/r)
   */
  double orbiting_velocity;
} GravitationalTimeDilation;

typedef struct LengthContraction {
  /**
   * Lorentz factor gamma = 1/sqrt(1 - v^2/c^2)
   */
  double lorentz_factor;
  /**
   * Contracted length L = L0 / gamma
   */
  double contracted_length;
  /**
   * Rest (proper) length L0
   */
  double proper_length;
  /**
   * Speed ratio beta = v/c
   */
  double speed_ratio;
} LengthContraction;

typedef struct RelativisticParticle {
  /**
   * Lorentz factor gamma
   */
  double lorentz_factor;
  /**
   * Total energy E = gamma * m * c^2
   */
  double total_energy;
  /**
   * Kinetic energy K = (gamma - 1) * m * c^2
   */
  double kinetic_energy;
  /**
   * Momentum magnitude p = gamma * m * v
   */
  double momentum_magnitude;
  /**
   * Momentum 3-vector
   */
  struct Vec3 momentum;
  /**
   * Rapidity = arctanh(v/c)
   */
  double rapidity;
} RelativisticParticle;

typedef struct SoftBodyStepReport {
  uint32_t particle_count;
  uint32_t constraint_count;
  uint32_t active_particle_count;
  double max_correction;
  double total_error;
} SoftBodyStepReport;

typedef struct SoftSpring {
  uint32_t particle_a;
  uint32_t particle_b;
  double rest_length;
  double stiffness;
  double damping;
} SoftSpring;

typedef struct SoftDistanceConstraint {
  uint32_t particle_a;
  uint32_t particle_b;
  double rest_length;
  double stiffness;
  double compliance;
  double lambda;
} SoftDistanceConstraint;

typedef struct SoftBendingConstraint {
  uint32_t particle_a;
  uint32_t particle_b;
  double rest_distance;
  double stiffness;
  double compliance;
  double lambda;
} SoftBendingConstraint;

typedef struct SoftSphereCollision {
  struct Vec3 center;
  double radius;
} SoftSphereCollision;

typedef struct SoftVolumeConstraint {
  uint32_t particle_a;
  uint32_t particle_b;
  uint32_t particle_c;
  uint32_t particle_d;
  double rest_volume;
  double compliance;
  double lambda;
} SoftVolumeConstraint;

typedef struct OrbitalElements {
  double semi_major_axis;
  double eccentricity;
  double inclination;
  double raan;
  double argument_of_periapsis;
  double true_anomaly;
} OrbitalElements;

typedef struct StateVector {
  struct Vec3 position;
  struct Vec3 velocity;
} StateVector;

typedef struct QuaternionDerivative {
  double i_dot;
  double j_dot;
  double k_dot;
  double w_dot;
} QuaternionDerivative;

typedef struct RigidBodyEulerDerivative {
  struct Vec3 angular_acceleration;
} RigidBodyEulerDerivative;

typedef struct CmgExchange {
  struct Vec3 body_torque;
  struct Vec3 wheel_momentum_dot;
} CmgExchange;

typedef struct CwState {
  struct Vec3 position;
  struct Vec3 velocity;
} CwState;

typedef struct CwDerivative {
  struct Vec3 velocity;
  struct Vec3 acceleration;
} CwDerivative;

typedef struct DhTransform {
  double m00;
  double m01;
  double m02;
  double m03;
  double m10;
  double m11;
  double m12;
  double m13;
  double m20;
  double m21;
  double m22;
  double m23;
  double m30;
  double m31;
  double m32;
  double m33;
} DhTransform;

typedef struct ManipulatorDynamics {
  struct Vec3 torque;
} ManipulatorDynamics;

typedef struct SolarPanelPower {
  double incident_power;
  double electrical_power;
} SolarPanelPower;

typedef struct ThermalBalance {
  double net_power;
  double equilibrium_temperature;
} ThermalBalance;

typedef struct Co2MassBalance {
  double mass_rate;
  double next_mass;
  double concentration_rate;
} Co2MassBalance;

typedef struct FriisLink {
  double received_power;
  double path_loss;
} FriisLink;

typedef struct HohmannTransfer {
  double delta_v1;
  double delta_v2;
  double total_delta_v;
  double transfer_time;
} HohmannTransfer;

typedef struct ScalarKalman {
  double value;
  double covariance;
} ScalarKalman;

typedef struct LeastSquaresAttitude {
  struct Quat attitude;
  double rms_error;
} LeastSquaresAttitude;

typedef struct GnssObservation {
  double value;
  double geometric_range;
} GnssObservation;

typedef struct ContactForceModel {
  double normal_force;
  double damping_force;
  double total_force;
} ContactForceModel;

typedef struct BatteryEquivalentCircuit {
  double terminal_voltage;
  double rc_voltage_dot;
  double state_of_charge_dot;
} BatteryEquivalentCircuit;

typedef struct HallThrusterPerformance {
  double thrust;
  double specific_impulse;
  double efficiency;
} HallThrusterPerformance;

typedef struct CollisionProbability {
  double probability;
  double combined_sigma;
} CollisionProbability;

typedef struct AtomicOxygenErosion {
  double volume_loss;
  double mass_loss;
} AtomicOxygenErosion;

typedef struct FlexibleModeDerivative {
  double displacement_dot;
  double velocity_dot;
} FlexibleModeDerivative;

typedef struct SloshPendulumDerivative {
  double angle_dot;
  double angular_rate_dot;
} SloshPendulumDerivative;

typedef struct VariationalState {
  struct Vec3 position_dot;
  struct Vec3 velocity_dot;
} VariationalState;

typedef struct FluidLoopHeatTransfer {
  double heat_rate;
  double outlet_temperature;
} FluidLoopHeatTransfer;

typedef struct RadarMeasurement {
  double range;
  double range_rate;
} RadarMeasurement;

typedef struct MassProperties {
  struct Vec3 center_of_mass;
  struct Vec3 inertia_diag;
} MassProperties;

typedef struct BangOffBangProfile {
  double coast_time;
  double total_time;
  double switch_angle;
} BangOffBangProfile;

typedef struct CmgRobustInverse {
  struct Vec3 gimbal_rates;
  double damping;
} CmgRobustInverse;

typedef struct Sgp4SecularRates {
  double mean_motion_dot;
  double raan_dot;
  double argument_of_perigee_dot;
} Sgp4SecularRates;

typedef struct ChemicalReactionRate {
  double reactant_rate;
  double product_rate;
} ChemicalReactionRate;

typedef struct RadiatorPower {
  double emitted_power;
  double net_power;
} RadiatorPower;

typedef struct AirlockDepressurization {
  double pressure;
  double pressure_rate;
} AirlockDepressurization;

/**
 * A single quantum vortex segment (straight line in 3D).
 */
typedef struct VortexSegment {
  struct Vec3 start;
  struct Vec3 end;
  /**
   * Circulation quantum number (integer)
   */
  int32_t circulation_quantum;
  /**
   * Core radius (healing length)
   */
  double core_radius;
} VortexSegment;

/**
 * Velocity induced by a vortex segment at a field point (Biot–Savart kernel).
 */
typedef struct BiotSavartVelocity {
  struct Vec3 velocity;
  double magnitude;
  /**
   * Distance from segment to field point
   */
  double distance;
} BiotSavartVelocity;

/**
 * State of a single quantum vortex ring (circular vortex line).
 */
typedef struct VortexRing {
  struct Vec3 center;
  /**
   * Radius of the ring
   */
  double radius;
  /**
   * Circulation quantum number
   */
  int32_t circulation_quantum;
  /**
   * Orientation axis (unit vector)
   */
  struct Vec3 axis;
  /**
   * Translational velocity (self-induced)
   */
  struct Vec3 velocity;
} VortexRing;

/**
 * Quantised circulation around a closed loop.
 */
typedef struct QuantisedCirculation {
  /**
   * Circulation κ = n × h/m
   */
  double circulation;
  /**
   * Quantum number n
   */
  int32_t quantum_number;
  /**
   * Circulation quantum h/m
   */
  double circulation_quantum;
  /**
   * Whether the circulation is consistent with quantisation
   */
  struct Bool quantised;
} QuantisedCirculation;

/**
 * Gross–Pitaevskii order parameter (condensate wavefunction) at a point.
 */
typedef struct GpOrderParameter {
  double amplitude;
  double phase;
  /**
   * Superfluid density n = |ψ|²
   */
  double density;
} GpOrderParameter;

/**
 * Gross–Pitaevskii chemical potential / energy density report.
 */
typedef struct GpEnergyDensity {
  double kinetic_density;
  double interaction_density;
  double trapping_density;
  double total_density;
  double chemical_potential;
} GpEnergyDensity;

/**
 * Parameters for time-dependent Gross–Pitaevskii integration.
 */
typedef struct GpTimeEvolutionParams {
  /**
   * Healing length ξ
   */
  double healing_length;
  /**
   * Speed of sound c
   */
  double sound_speed;
  /**
   * Chemical potential μ
   */
  double chemical_potential;
  /**
   * Nonlinear coupling constant g
   */
  double coupling_constant;
  /**
   * Time step dt
   */
  double dt;
} GpTimeEvolutionParams;

/**
 * Report from a vortex reconnection event.
 */
typedef struct VortexReconnectionReport {
  /**
   * Distance between two segments before reconnection
   */
  double closest_approach;
  /**
   * Whether a reconnection occurred
   */
  struct Bool reconnected;
  /**
   * Post-reconnection segment 1 start
   */
  struct Vec3 seg1_start;
  struct Vec3 seg1_end;
  /**
   * Post-reconnection segment 2 start
   */
  struct Vec3 seg2_start;
  struct Vec3 seg2_end;
  /**
   * Energy dissipated during reconnection
   */
  double energy_dissipated;
} VortexReconnectionReport;

/**
 * Vortex filament network: a collection of vortex segments forming a tangle.
 */
typedef struct VortexTangleStats {
  uint32_t segment_count;
  double total_length;
  double average_curvature;
  double total_kinetic_energy;
  double vortex_line_density;
} VortexTangleStats;

/**
 * A single point in a 2D cross-section of the GP wavefunction (for visualisation).
 */
typedef struct GpGridPoint {
  double x;
  double y;
  double amplitude;
  double phase;
  double density;
} GpGridPoint;

typedef struct HeatConductionReport {
  double temperature_delta;
  double temperature_gradient;
  double heat_flux;
  double heat_rate;
  double thermal_resistance;
} HeatConductionReport;

typedef struct PhaseChangeReport {
  double final_temperature;
  double sensible_heat;
  double latent_heat_used;
  double phase_fraction_delta;
  struct Bool phase_changed;
} PhaseChangeReport;

typedef struct ThermalRadiationReport {
  double emitted_power;
  double absorbed_power;
  double net_power;
  double radiative_coefficient;
} ThermalRadiationReport;

typedef struct FemHeatNode {
  double temperature;
  double heat_capacity;
  double heat_source;
} FemHeatNode;

typedef struct FemHeatEdge {
  uint32_t node_a;
  uint32_t node_b;
  double conductance;
} FemHeatEdge;

typedef struct FemHeatDiffusionReport {
  uint32_t node_count;
  uint32_t edge_count;
  double total_heat_rate;
  double max_temperature_delta;
} FemHeatDiffusionReport;

typedef struct ThermalStressReport {
  double free_thermal_strain;
  double mechanical_strain;
  double stress;
  double deformation;
  double elastic_energy_density;
} ThermalStressReport;

typedef struct ThermoelasticReport {
  double thermal_strain;
  double mechanical_strain_x;
  double mechanical_strain_y;
  double mechanical_strain_z;
  double stress_x;
  double stress_y;
  double stress_z;
  double bulk_modulus;
  double shear_modulus;
} ThermoelasticReport;

typedef struct TopologyOptimizationParams {
  double volume_fraction;
  double penalization;
  double min_density;
  double move_limit;
  double filter_radius;
  double stiffness_min;
  double stiffness_solid;
} TopologyOptimizationParams;

typedef struct SimpMaterialReport {
  double density;
  double stiffness;
  double stiffness_derivative;
} SimpMaterialReport;

typedef struct TopologyOptimizationReport {
  uint32_t cell_count;
  double average_density;
  double min_density;
  double max_density;
  double total_compliance;
  double max_density_change;
} TopologyOptimizationReport;

typedef struct DensityFieldStats {
  uint32_t cell_count;
  uint32_t solid_count;
  double average_density;
  double min_density;
  double max_density;
} DensityFieldStats;

typedef struct TrajectoryState {
  struct Vec3 position;
  struct Vec3 velocity;
} TrajectoryState;

typedef struct TrajectoryGlideState {
  double speed;
  double flight_path_angle;
  double altitude;
  double downrange;
} TrajectoryGlideState;

typedef struct TrajectoryGlideEnvironment {
  double gravity;
  double planet_radius;
  double ballistic_coefficient;
  double lift_to_drag;
  double bank_angle;
  double reference_density;
  double scale_height;
} TrajectoryGlideEnvironment;

typedef struct TrajectoryGlideReport {
  double density;
  double dynamic_pressure;
  double drag_acceleration;
  double lift_acceleration;
  double speed_dot;
  double flight_path_angle_dot;
  double altitude_dot;
  double downrange_dot;
} TrajectoryGlideReport;

typedef struct GearConstraintDesc {
  double ratio;
  double phase;
  double backlash;
  struct Bool opposite_direction;
} GearConstraintDesc;

typedef struct GearConstraintReport {
  double target_angle;
  double target_angular_velocity;
  double angle_error;
  double velocity_error;
  double effective_ratio;
} GearConstraintReport;

typedef struct ScrewConstraintDesc {
  double lead;
  double phase;
  struct Bool right_handed;
} ScrewConstraintDesc;

typedef struct ScrewConstraintReport {
  double target_translation;
  double target_linear_velocity;
  double translation_error;
  double velocity_error;
  double meters_per_radian;
} ScrewConstraintReport;

typedef struct CamConstraintDesc {
  double base_radius;
  double lift;
  double rise_angle;
  double return_angle;
  double phase;
} CamConstraintDesc;

typedef struct CamConstraintReport {
  double wrapped_angle;
  double radius;
  double follower_displacement;
  double displacement_derivative;
  double target_velocity;
  double displacement_error;
} CamConstraintReport;

typedef struct SpiralConstraintDesc {
  double initial_radius;
  double radial_pitch;
  double phase;
} SpiralConstraintDesc;

typedef struct SpiralConstraintReport {
  double radius;
  struct Vec3 position;
  struct Vec3 tangent;
  double radial_velocity;
  double constraint_error;
} SpiralConstraintReport;

typedef struct VoxelColliderOptions {
  uint32_t mode;
  struct Bool dynamic_body;
  uint32_t small_voxel_limit;
  uint32_t mesh_voxel_limit;
} VoxelColliderOptions;

typedef struct VoxelBuildStats {
  uint32_t cell_count;
  uint32_t solid_count;
  uint32_t selected_mode;
  uint32_t estimated_parts;
  uint32_t estimated_vertices;
  uint32_t estimated_triangles;
  uint32_t size_x;
  uint32_t size_y;
  uint32_t size_z;
} VoxelBuildStats;

/**
 * Parameters for a monochromatic plane wave.
 */
typedef struct PlaneWaveParams {
  /**
   * Wavenumber k = 2π/λ
   */
  double wavenumber;
  /**
   * Wavelength λ
   */
  double wavelength;
  /**
   * Initial amplitude A₀
   */
  double amplitude;
  /**
   * Initial phase φ₀
   */
  double phase_offset;
} PlaneWaveParams;

/**
 * Complex wave amplitude at a point.
 */
typedef struct ComplexAmplitude {
  double real;
  double imag;
  /**
   * Intensity I = |E|²
   */
  double intensity;
} ComplexAmplitude;

/**
 * A single spherical wave emitted from a point source.
 */
typedef struct SphericalWavePoint {
  struct ComplexAmplitude amplitude;
  /**
   * Distance from source
   */
  double radius;
  /**
   * 1/r amplitude decay factor
   */
  double decay_factor;
} SphericalWavePoint;

/**
 * A single point source used in Huygens–Fresnel superposition.
 */
typedef struct PointSource {
  double x;
  double y;
  double z;
  /**
   * Initial phase at this source point
   */
  double phase;
  /**
   * Amplitude scaling factor
   */
  double amplitude;
} PointSource;

/**
 * Describes a planar aperture for diffraction calculations.
 */
typedef struct ApertureDesc {
  /**
   * Half-width in x (m)
   */
  double half_width_x;
  /**
   * Half-width in y (m)
   */
  double half_width_y;
  /**
   * Centre position in the aperture plane
   */
  double center_x;
  double center_y;
  /**
   * Transmission coefficient (0=opaque, 1=fully transparent)
   */
  double transmission;
} ApertureDesc;

/**
 * Huygens–Fresnel diffraction from an aperture (single point).
 */
typedef struct DiffractionPoint {
  /**
   * Coordinates in the observation plane
   */
  double x;
  double y;
  /**
   * Complex amplitude at this point
   */
  struct ComplexAmplitude amplitude;
} DiffractionPoint;

/**
 * Fresnel–Kirchhoff diffraction integral result for a single observation point.
 */
typedef struct KirchhoffDiffractionPoint {
  double x;
  double y;
  struct ComplexAmplitude amplitude;
  /**
   * Obliquity (inclination) factor cosθ
   */
  double obliquity_factor;
} KirchhoffDiffractionPoint;

/**
 * Two-slit (Young's) interference pattern at a point.
 */
typedef struct YoungSlitPoint {
  double x;
  double y;
  /**
   * Phase difference between slits
   */
  double phase_difference;
  /**
   * Path difference in metres
   */
  double path_difference;
  /**
   * Interference intensity
   */
  double intensity;
  /**
   * Envelope (single-slit diffraction) factor
   */
  double envelope_factor;
} YoungSlitPoint;

/**
 * Parameters for a thin film.
 */
typedef struct ThinFilmParams {
  /**
   * Film thickness (m)
   */
  double thickness;
  /**
   * Film refractive index
   */
  double n_film;
  /**
   * Substrate refractive index
   */
  double n_substrate;
  /**
   * Incident medium refractive index (typically 1.0 for air)
   */
  double n_incident;
  /**
   * Angle of incidence (radians)
   */
  double incidence_angle;
} ThinFilmParams;

/**
 * Thin-film interference report (single layer).
 */
typedef struct ThinFilmInterferenceReport {
  /**
   * Optical path difference
   */
  double opd;
  /**
   * Phase difference from path
   */
  double phase_difference;
  /**
   * Reflection coefficient magnitude
   */
  double reflection_coefficient;
  /**
   * Interference intensity (normalised)
   */
  double intensity;
  /**
   * Whether half-wave loss occurs (n_film > n_substrate or similar)
   */
  struct Bool half_wave_loss;
  /**
   * Wavelength for which this report was computed
   */
  double wavelength;
} ThinFilmInterferenceReport;

/**
 * Fresnel diffraction zone plate / Fresnel zone parameters.
 */
typedef struct FresnelZoneReport {
  /**
   * Radius of the n-th Fresnel zone
   */
  double zone_radius;
  /**
   * Zone index
   */
  uint32_t zone_index;
  /**
   * Phase contribution from this zone
   */
  double zone_phase;
  /**
   * Whether the zone is constructive (phase within ±π/2 of centre)
   */
  struct Bool constructive;
} FresnelZoneReport;

typedef struct CharacterCollision {
  ColliderHandleRaw collider;
  struct Vec3 character_translation;
  struct Vec3 translation_applied;
  struct Vec3 translation_remaining;
  struct Vec3 world_witness1;
  struct Vec3 world_witness2;
  struct Vec3 normal1;
  struct Vec3 normal2;
  double time_of_impact;
} CharacterCollision;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

uint32_t abi_version(void);

struct Bool abi_supports_ffm(void);

struct Bool abi_supports_jni(void);

struct Bool acoustic_generalized_modal_analysis(const double *stiffness_matrix,
                                                const double *mass_matrix,
                                                uint32_t dof,
                                                uint32_t requested_modes,
                                                double *out_eigenvalues,
                                                double *out_frequencies_hz,
                                                double *out_mode_shapes,
                                                uint32_t eigen_capacity,
                                                uint32_t mode_shape_capacity,
                                                struct ModalAnalysisReport *out_report);

struct Bool acoustic_structural_mode_sdof(double stiffness,
                                          double mass,
                                          double damping,
                                          struct StructuralModeReport *out_report);

struct Bool acoustic_wave_equation_step(const double *previous_pressure,
                                        const double *current_pressure,
                                        const double *laplacian_pressure,
                                        uint32_t cell_count,
                                        double sound_speed,
                                        double damping,
                                        double dt,
                                        double *out_next_pressure,
                                        uint32_t capacity,
                                        struct AcousticWaveReport *out_report);

struct Bool acoustic_detect_resonance(double excitation_frequency_hz,
                                      const double *modal_frequencies_hz,
                                      const double *damping_ratios,
                                      uint32_t mode_count,
                                      double tolerance_hz,
                                      struct AcousticResonanceReport *out_report);

struct Bool acoustic_contact_material_excitation(struct AcousticMaterial material_a,
                                                 struct AcousticMaterial material_b,
                                                 struct AcousticContactDesc contact,
                                                 struct AcousticExcitationReport *out_report);

struct Bool acoustic_modal_synthesis_step(const double *modal_frequencies_hz,
                                          const double *damping_ratios,
                                          const double *modal_gains,
                                          double *mode_displacements,
                                          double *mode_velocities,
                                          uint32_t mode_count,
                                          double excitation,
                                          double dt,
                                          double output_gain,
                                          struct ModalSynthesisReport *out_report);

struct Bool acoustic_spatialize_mono_sample(double mono_sample,
                                            struct Vec3 source_position,
                                            struct Vec3 listener_position,
                                            struct Vec3 listener_right,
                                            double reference_distance,
                                            double rolloff,
                                            struct SpatializedSample *out_sample);

struct Bool aero_apply_surfaces(struct WorldHandle *world,
                                RigidBodyHandleRaw body_handle,
                                struct Vec3 wind_velocity,
                                double air_density,
                                const struct AeroSurface *surfaces,
                                uint32_t surface_count,
                                struct Bool wake_up,
                                struct AeroForceReport *out_report);

struct Bool aero_apply_voxel_grid(struct WorldHandle *world,
                                  RigidBodyHandleRaw body_handle,
                                  struct Vec3 wind_velocity,
                                  double air_density,
                                  const uint8_t *voxels,
                                  uint32_t size_x,
                                  uint32_t size_y,
                                  uint32_t size_z,
                                  double voxel_size,
                                  struct Vec3 local_origin,
                                  double drag_coefficient,
                                  double lift_coefficient,
                                  struct Bool wake_up,
                                  struct AeroForceReport *out_report);

uint8_t aero_apply_voxel_grid_flag(struct WorldHandle *world,
                                   RigidBodyHandleRaw body_handle,
                                   struct Vec3 wind_velocity,
                                   double air_density,
                                   const uint8_t *voxels,
                                   uint32_t size_x,
                                   uint32_t size_y,
                                   uint32_t size_z,
                                   double voxel_size,
                                   struct Vec3 local_origin,
                                   double drag_coefficient,
                                   double lift_coefficient,
                                   struct Bool wake_up,
                                   struct AeroForceReport *out_report);

uint8_t aero_apply_surfaces_flag(struct WorldHandle *world,
                                 RigidBodyHandleRaw body_handle,
                                 struct Vec3 wind_velocity,
                                 double air_density,
                                 const struct AeroSurface *surfaces,
                                 uint32_t surface_count,
                                 struct Bool wake_up,
                                 struct AeroForceReport *out_report);

struct Bool aero_estimate_surface_force(struct Vec3 body_linvel,
                                        struct Vec3 body_angvel,
                                        struct Vec3 body_center,
                                        struct Vec3 wind_velocity,
                                        double air_density,
                                        struct AeroSurface surface,
                                        struct AeroForceReport *out_report);

struct AnvilKitAppHandle *anvilkit_app_create(void);

void anvilkit_app_destroy(struct AnvilKitAppHandle *app);

void anvilkit_app_update(struct AnvilKitAppHandle *app);

uint64_t anvilkit_app_spawn_body(struct AnvilKitAppHandle *app,
                                 struct Vec3 translation,
                                 struct Quat rotation,
                                 uint32_t status);

uint64_t anvilkit_app_spawn_body_with_collider(struct AnvilKitAppHandle *app,
                                               struct Vec3 translation,
                                               struct Quat rotation,
                                               uint32_t status,
                                               struct ShapeDesc shape);

struct Bool anvilkit_app_set_transform(struct AnvilKitAppHandle *app,
                                       uint64_t entity_bits,
                                       struct Vec3 translation,
                                       struct Quat rotation);

struct Bool anvilkit_app_set_material(struct AnvilKitAppHandle *app,
                                      uint64_t entity_bits,
                                      struct MaterialProperties material);

uint32_t anvilkit_app_sync_to_world(struct AnvilKitAppHandle *app, struct WorldHandle *world);

RigidBodyHandleRaw anvilkit_app_entity_to_body(const struct AnvilKitAppHandle *app,
                                               uint64_t entity_bits);

ColliderHandleRaw anvilkit_app_entity_to_collider(const struct AnvilKitAppHandle *app,
                                                  uint64_t entity_bits);

uint64_t anvilkit_app_create_constraint(struct AnvilKitAppHandle *app,
                                        struct WorldHandle *world,
                                        uint64_t entity1_bits,
                                        uint64_t entity2_bits,
                                        uint32_t joint_type,
                                        struct Vec3 axis_or_primary,
                                        double b,
                                        double c,
                                        struct Bool wake_up);

ImpulseJointHandleRaw anvilkit_app_constraint_to_joint(const struct AnvilKitAppHandle *app,
                                                       uint64_t constraint_id);

struct Bool anvilkit_app_remove_constraint(struct AnvilKitAppHandle *app,
                                           struct WorldHandle *world,
                                           uint64_t constraint_id,
                                           struct Bool wake_up);

struct Bool anvilkit_app_apply_aero_surfaces(struct AnvilKitAppHandle *app,
                                             struct WorldHandle *world,
                                             uint64_t entity_bits,
                                             struct Vec3 wind_velocity,
                                             double air_density,
                                             const struct AeroSurface *surfaces,
                                             uint32_t surface_count,
                                             struct Bool wake_up,
                                             struct AeroForceReport *out_report);

struct Bool anvilkit_app_apply_aero_voxel_grid(struct AnvilKitAppHandle *app,
                                               struct WorldHandle *world,
                                               uint64_t entity_bits,
                                               struct Vec3 wind_velocity,
                                               double air_density,
                                               const uint8_t *voxels,
                                               uint32_t size_x,
                                               uint32_t size_y,
                                               uint32_t size_z,
                                               double voxel_size,
                                               struct Vec3 local_origin,
                                               double drag_coefficient,
                                               double lift_coefficient,
                                               struct Bool wake_up,
                                               struct AeroForceReport *out_report);

struct Bool anvilkit_app_apply_fluid_aabb_forces(struct AnvilKitAppHandle *app,
                                                 struct WorldHandle *world,
                                                 uint64_t entity_bits,
                                                 struct FluidVolume fluid_volume,
                                                 struct Vec3 body_half_extents,
                                                 double body_volume,
                                                 struct Bool wake_up,
                                                 struct FluidForceReport *out_report);

struct Bool anvilkit_app_apply_trajectory_forces(struct AnvilKitAppHandle *app,
                                                 struct WorldHandle *world,
                                                 uint64_t entity_bits,
                                                 struct TrajectoryEnvironment environment,
                                                 struct Bool wake_up,
                                                 struct TrajectoryForceReport *out_report);

struct Bool material_stress_strain_linear(struct MaterialProperties material,
                                          double strain,
                                          double delta_temperature,
                                          struct StressStrainReport *out_report);

double material_elastic_collision_relative_speed(double relative_normal_speed, double restitution);

struct Bool material_hertz_contact_force(struct MaterialProperties material1,
                                         struct MaterialProperties material2,
                                         double radius1,
                                         double radius2,
                                         double penetration,
                                         double penetration_rate,
                                         double damping,
                                         struct HertzContactReport *out_report);

struct Bool astro_nbody_direct_accelerations(const struct NBodyParticle *particles,
                                             uint32_t particle_count,
                                             struct NBodySolverParams params,
                                             struct Vec3 *out_accelerations,
                                             uint32_t capacity,
                                             struct NBodyForceReport *out_report);

struct Bool astro_nbody_barnes_hut_accelerations(const struct NBodyParticle *particles,
                                                 uint32_t particle_count,
                                                 struct NBodySolverParams params,
                                                 struct Vec3 *out_accelerations,
                                                 uint32_t capacity,
                                                 struct NBodyForceReport *out_report);

struct Bool astro_fmm_monopole_acceleration(struct Vec3 position,
                                            struct Vec3 cluster_center,
                                            double cluster_mass,
                                            struct NBodySolverParams params,
                                            struct Vec3 *out_acceleration);

struct Bool astro_relativistic_orbit_correction(struct Vec3 position,
                                                struct Vec3 velocity,
                                                double central_mass,
                                                double gravitational_constant,
                                                struct RelativisticOrbitReport *out_report);

struct Bool astro_roche_limit(double primary_radius,
                              double primary_density,
                              double secondary_density,
                              double orbital_distance,
                              struct RocheLimitReport *out_report);

struct Bool astro_orbital_resonance_detect(double inner_period,
                                           double outer_period,
                                           uint32_t max_denominator,
                                           double tolerance,
                                           struct OrbitalResonanceReport *out_report);

struct Bool astro_barnes_hut_should_open(double node_width, double distance, double opening_angle);

double biomechanics_hill_force_length_factor(double fiber_length,
                                             double optimal_fiber_length,
                                             double width);

double biomechanics_hill_force_velocity_factor(double fiber_velocity,
                                               double max_contraction_velocity);

struct Bool biomechanics_hill_muscle_evaluate(struct HillMuscleDesc desc,
                                              struct HillMuscleState state,
                                              struct HillMuscleReport *out_report);

double biomechanics_hill_three_element_force(double activation,
                                             double fiber_length,
                                             double fiber_velocity,
                                             double tendon_length,
                                             struct HillMuscleDesc desc);

struct Bool biomechanics_skeletal_joint_limit(double angle,
                                              double angular_velocity,
                                              struct SkeletalJointLimit limit,
                                              struct SkeletalConstraintReport *out_report);

double biomechanics_muscle_joint_torque(double muscle_force, double moment_arm);

struct ColliderBuilderHandle *collider_builder_create_capsule(struct Capsule capsule);

struct ColliderBuilderHandle *collider_builder_create_ssv(struct Ssv ssv);

struct ColliderBuilderHandle *collider_builder_create_ellipsoid(struct Ellipsoid ellipsoid);

struct ColliderBuilderHandle *collider_builder_create_prism(struct Prism prism);

struct ColliderBuilderHandle *collider_builder_create_cylinder(struct Cylinder cylinder);

struct ColliderBuilderHandle *collider_builder_create_spherical_shell(struct SphericalShell shell);

uint32_t query_intersect_capsule_count(const struct WorldHandle *world,
                                       struct Capsule capsule,
                                       struct QueryFilterDesc filter);

uint32_t query_intersect_capsule_count_all(const struct WorldHandle *world, struct Capsule capsule);

uint32_t query_intersect_capsule(const struct WorldHandle *world,
                                 struct Capsule capsule,
                                 struct QueryFilterDesc filter,
                                 ColliderHandleRaw *out_handles,
                                 uint32_t capacity);

uint32_t query_intersect_capsule_all(const struct WorldHandle *world,
                                     struct Capsule capsule,
                                     ColliderHandleRaw *out_handles,
                                     uint32_t capacity);

uint32_t query_intersect_ssv_count(const struct WorldHandle *world,
                                   struct Ssv ssv,
                                   struct QueryFilterDesc filter);

uint32_t query_intersect_ssv_count_all(const struct WorldHandle *world, struct Ssv ssv);

uint32_t query_intersect_ssv(const struct WorldHandle *world,
                             struct Ssv ssv,
                             struct QueryFilterDesc filter,
                             ColliderHandleRaw *out_handles,
                             uint32_t capacity);

uint32_t query_intersect_ssv_all(const struct WorldHandle *world,
                                 struct Ssv ssv,
                                 ColliderHandleRaw *out_handles,
                                 uint32_t capacity);

uint32_t query_intersect_ellipsoid_count(const struct WorldHandle *world,
                                         struct Ellipsoid ellipsoid,
                                         struct QueryFilterDesc filter);

uint32_t query_intersect_ellipsoid_count_all(const struct WorldHandle *world,
                                             struct Ellipsoid ellipsoid);

uint32_t query_intersect_ellipsoid(const struct WorldHandle *world,
                                   struct Ellipsoid ellipsoid,
                                   struct QueryFilterDesc filter,
                                   ColliderHandleRaw *out_handles,
                                   uint32_t capacity);

uint32_t query_intersect_ellipsoid_all(const struct WorldHandle *world,
                                       struct Ellipsoid ellipsoid,
                                       ColliderHandleRaw *out_handles,
                                       uint32_t capacity);

uint32_t query_intersect_prism_count(const struct WorldHandle *world,
                                     struct Prism prism,
                                     struct QueryFilterDesc filter);

uint32_t query_intersect_prism_count_all(const struct WorldHandle *world, struct Prism prism);

uint32_t query_intersect_prism(const struct WorldHandle *world,
                               struct Prism prism,
                               struct QueryFilterDesc filter,
                               ColliderHandleRaw *out_handles,
                               uint32_t capacity);

uint32_t query_intersect_prism_all(const struct WorldHandle *world,
                                   struct Prism prism,
                                   ColliderHandleRaw *out_handles,
                                   uint32_t capacity);

uint32_t query_intersect_cylinder_count(const struct WorldHandle *world,
                                        struct Cylinder cylinder,
                                        struct QueryFilterDesc filter);

uint32_t query_intersect_cylinder_count_all(const struct WorldHandle *world,
                                            struct Cylinder cylinder);

uint32_t query_intersect_cylinder(const struct WorldHandle *world,
                                  struct Cylinder cylinder,
                                  struct QueryFilterDesc filter,
                                  ColliderHandleRaw *out_handles,
                                  uint32_t capacity);

uint32_t query_intersect_cylinder_all(const struct WorldHandle *world,
                                      struct Cylinder cylinder,
                                      ColliderHandleRaw *out_handles,
                                      uint32_t capacity);

uint32_t query_intersect_spherical_shell_count(const struct WorldHandle *world,
                                               struct SphericalShell shell,
                                               struct QueryFilterDesc filter);

uint32_t query_intersect_spherical_shell_count_all(const struct WorldHandle *world,
                                                   struct SphericalShell shell);

uint32_t query_intersect_spherical_shell(const struct WorldHandle *world,
                                         struct SphericalShell shell,
                                         struct QueryFilterDesc filter,
                                         ColliderHandleRaw *out_handles,
                                         uint32_t capacity);

uint32_t query_intersect_spherical_shell_all(const struct WorldHandle *world,
                                             struct SphericalShell shell,
                                             ColliderHandleRaw *out_handles,
                                             uint32_t capacity);

/**
 * Perform one RK4 step of the Lorenz system.
 */
struct Bool chaos_lorenz_step(struct LorenzState state,
                              struct LorenzParams params,
                              struct LorenzStepReport *out_report);

/**
 * Integrate the Lorenz system for N steps, writing each state into a
 * pre-allocated output buffer of length `out_len`.
 *
 * Returns the number of steps actually written.
 */
uint32_t chaos_lorenz_integrate(struct LorenzState initial,
                                struct LorenzParams params,
                                uint32_t steps,
                                struct LorenzState *out_states,
                                uint32_t out_len);

/**
 * Return the number of states written (for use after `chaos_lorenz_integrate`).
 * Identical to the return value; provided as a convenience for FFI callers
 * who want it stored in memory.
 */
uint32_t chaos_lorenz_integrate_count(uint32_t steps, uint32_t out_len);

/**
 * Estimate the largest Lyapunov exponent by tracking the divergence of two
 * nearby trajectories in the Lorenz system.
 *
 * A reference trajectory and a perturbed copy are integrated simultaneously.
 * Every `renorm_interval` steps the separation is measured, the log ratio
 * accumulated, and the perturbed trajectory is re-normalised to keep the
 * perturbation small. This gives λ ≈ (1/t) Σ ln(δ/δ₀).
 *
 * `perturbation` is the initial separation magnitude (typical 1e-8).
 * `renorm_every` re-normalises every N integration steps.
 * `total_steps` total integration steps for the estimate.
 */
struct Bool chaos_lyapunov_lorenz(struct LorenzState initial,
                                  struct LorenzParams params,
                                  double perturbation,
                                  uint32_t renorm_every,
                                  uint32_t total_steps,
                                  struct LyapunovReport *out_report);

/**
 * Compute the largest Lyapunov exponent from a 1D time series using the
 * Rosenstein algorithm (method of delays).
 *
 * `data` — pointer to an array of length `data_len` containing the scalar
 * time-series samples.
 * `embedding_dim` — embedding dimension m (typically 3–7).
 * `delay` — time delay τ in samples (typically 1–10).
 * `out_report` — filled with the estimated exponent.
 */
struct Bool chaos_lyapunov_rosenstein(const double *data,
                                      uint32_t data_len,
                                      uint32_t embedding_dim,
                                      uint32_t delay,
                                      struct LyapunovReport *out_report);

/**
 * Sample a Lorenz bifurcation diagram by scanning one parameter across a
 * range. For each parameter value:
 *   1. Discard `transient_steps` integration steps.
 *   2. Record the next `samples_per_value` local maxima of x (or y/z)
 *      as bifurcation points.
 *
 * `vary` — which parameter to vary: 0 = sigma, 1 = rho, 2 = beta.
 * `param_min`, `param_max` — range of the parameter.
 * `param_steps` — how many distinct parameter values.
 * `transient_steps` — steps to discard before recording.
 * `samples_per_value` — number of Poincaré samples per parameter value.
 * `out_points` — pre-allocated buffer for `BifurcationPoint`.
 * `out_len` — capacity of the output buffer.
 * Returns the number of points actually written.
 */
uint32_t chaos_bifurcation_lorenz(struct LorenzState initial,
                                  struct LorenzParams base_params,
                                  uint32_t vary,
                                  double param_min,
                                  double param_max,
                                  uint32_t param_steps,
                                  uint32_t transient_steps,
                                  uint32_t samples_per_value,
                                  struct BifurcationPoint *out_points,
                                  uint32_t out_len);

/**
 * Compute the angular accelerations of a double pendulum using the
 * Lagrangian equations of motion:
 *
 *   α1 = ( -g (2 m1 + m2) sin θ1 - m2 g sin(θ1 - 2 θ2)
 *         - 2 sin(θ1 - θ2) m2 ( ω2² L2 + ω1² L1 cos(θ1 - θ2) ) )
 *        / ( L1 ( 2 m1 + m2 - m2 cos(2 θ1 - 2 θ2) ) )
 *
 *   α2 = ( 2 sin(θ1 - θ2) ( ω1² L1 (m1 + m2) + g (m1 + m2) cos θ1
 *         + ω2² L2 m2 cos(θ1 - θ2) ) )
 *        / ( L2 ( 2 m1 + m2 - m2 cos(2 θ1 - 2 θ2) ) )
 */
struct Bool chaos_double_pendulum_accel(struct DoublePendulumState state,
                                        struct DoublePendulumParams params,
                                        struct DoublePendulumAccel *out_accel);

/**
 * Perform one RK4 integration step of the double pendulum.
 *
 * The system is a 4D ODE: (θ1, ω1, θ2, ω2) with ω = dθ/dt and
 * α = dω/dt given by `chaos_double_pendulum_accel`.
 */
struct Bool chaos_double_pendulum_step(struct DoublePendulumState state,
                                       struct DoublePendulumParams params,
                                       struct DoublePendulumState *out_next);

/**
 * Integrate the double pendulum for N steps, writing states into a
 * pre-allocated output buffer.
 *
 * Returns the number of states written.
 */
uint32_t chaos_double_pendulum_integrate(struct DoublePendulumState initial,
                                         struct DoublePendulumParams params,
                                         uint32_t steps,
                                         struct DoublePendulumState *out_states,
                                         uint32_t out_len);

/**
 * Analyse a 1D time series and determine if it exhibits chaotic behaviour.
 *
 * Uses two heuristics:
 *   1. Largest Lyapunov exponent via Rosenstein algorithm (if positive → chaotic).
 *   2. Correlation dimension via Grassberger–Procaccia (low fractional → periodic/quasi,
 *      high fractional → chaotic).
 *
 * `data` — pointer to an array of scalar samples.
 * `data_len` — length of the time series.
 * `params` — detection parameters (embedding, neighbourhood, threshold).
 * `out_report` — filled with the analysis results.
 */
struct Bool chaos_detect(const double *data,
                         uint32_t data_len,
                         struct ChaosDetectionParams params,
                         struct ChaosDetectionReport *out_report);

/**
 * Perform one iteration of the logistic map: x_{n+1} = r * x_n * (1 - x_n).
 */
struct Bool chaos_logistic_step(double x, double r, struct LogisticMapState *out_next);

/**
 * Run the logistic map for N steps, returning all iterates.
 */
uint32_t chaos_logistic_iterate(double initial_x,
                                double r,
                                uint32_t steps,
                                double *out_values,
                                uint32_t out_len);

/**
 * Logistic map bifurcation diagram.
 *
 * For each of `param_steps` values of r between `r_min` and `r_max`:
 *   1. Run `transient_steps` iterations to reach the attractor.
 *   2. Record the next `samples_per_value` iterates.
 */
uint32_t chaos_logistic_bifurcation(double initial_x,
                                    double r_min,
                                    double r_max,
                                    uint32_t param_steps,
                                    uint32_t transient_steps,
                                    uint32_t samples_per_value,
                                    struct BifurcationPoint *out_points,
                                    uint32_t out_len);

struct ColliderBuilderHandle *collider_builder_create(uint32_t shape_type, struct Vec3 shape_data);

struct ColliderBuilderHandle *collider_builder_create_ex(struct ShapeDesc shape_desc);

struct ColliderBuilderHandle *collider_builder_create_obb(struct Obb obb);

struct ColliderBuilderHandle *collider_builder_create_sphere(struct Sphere sphere);

struct ColliderBuilderHandle *collider_builder_create_heightmap(const double *data,
                                                                uint32_t data_x,
                                                                uint32_t data_y,
                                                                struct Vec3 scale);

struct ColliderBuilderHandle *collider_builder_create_convex_hull(const double *points_xyz,
                                                                  uint32_t point_count);

struct ColliderBuilderHandle *collider_builder_create_point_cloud_bounds(const double *points_xyz,
                                                                         uint32_t point_count);

struct ColliderBuilderHandle *collider_builder_create_double_bv(struct AabbDesc first,
                                                                struct AabbDesc second);

struct ColliderBuilderHandle *collider_builder_create_skewed_obb(struct Vec3 center,
                                                                 struct Vec3 axis_x,
                                                                 struct Vec3 axis_y,
                                                                 struct Vec3 axis_z);

struct ColliderBuilderHandle *collider_builder_create_discrete_obb(const double *points_xyz,
                                                                   uint32_t point_count,
                                                                   uint32_t axis);

struct ColliderBuilderHandle *collider_builder_create_fused_collapsing_bounds(const double *points_xyz,
                                                                              uint32_t point_count,
                                                                              double padding);

struct ColliderBuilderHandle *collider_builder_create_edge_bvh(const double *vertices_xyz,
                                                               uint32_t vertex_count,
                                                               const uint32_t *edges,
                                                               uint32_t edge_count,
                                                               double radius);

struct ColliderBuilderHandle *collider_builder_create_medial_spheres(const double *spheres_xyzw,
                                                                     uint32_t sphere_count);

Collider *collider_builder_build(struct ColliderBuilderHandle *builder);

void collider_builder_destroy(struct ColliderBuilderHandle *builder);

void collider_destroy_raw(Collider *collider);

void collider_builder_set_translation(struct ColliderBuilderHandle *builder,
                                      struct Vec3 translation);

void collider_builder_set_rotation(struct ColliderBuilderHandle *builder,
                                   struct Vec3 rotation_axis_angle);

void collider_builder_set_pose(struct ColliderBuilderHandle *builder,
                               struct Vec3 translation,
                               struct Quat rotation);

void collider_builder_set_sensor(struct ColliderBuilderHandle *builder, struct Bool sensor);

void collider_builder_set_friction(struct ColliderBuilderHandle *builder, double friction);

void collider_builder_set_restitution(struct ColliderBuilderHandle *builder, double restitution);

void collider_builder_set_density(struct ColliderBuilderHandle *builder, double density);

void collider_builder_set_collision_groups(struct ColliderBuilderHandle *builder,
                                           struct InteractionGroupsDesc groups);

void collider_builder_set_solver_groups(struct ColliderBuilderHandle *builder,
                                        struct InteractionGroupsDesc groups);

void collider_builder_set_active_events(struct ColliderBuilderHandle *builder,
                                        uint32_t active_events_bits);

void collider_builder_set_active_hooks(struct ColliderBuilderHandle *builder,
                                       uint32_t active_hooks_bits);

void collider_builder_set_contact_force_event_threshold(struct ColliderBuilderHandle *builder,
                                                        double threshold);

ColliderHandleRaw world_insert_collider(struct WorldHandle *world, Collider *memory_handle);

ColliderHandleRaw world_insert_collider_with_parent(struct WorldHandle *world,
                                                    Collider *memory_handle,
                                                    RigidBodyHandleRaw parent);

struct Bool world_remove_collider(struct WorldHandle *world,
                                  ColliderHandleRaw handle,
                                  struct Bool wake_up);

Collider *world_copy_collider(struct WorldHandle *world, ColliderHandleRaw handle);

uint8_t world_remove_collider_flag(struct WorldHandle *world,
                                   ColliderHandleRaw handle,
                                   struct Bool wake_up);

struct Vec3 collider_get_translation(const struct WorldHandle *world, ColliderHandleRaw handle);

void collider_get_translation_out(const struct WorldHandle *world,
                                  ColliderHandleRaw handle,
                                  struct Vec3 *out_translation);

struct Quat collider_get_rotation(const struct WorldHandle *world, ColliderHandleRaw handle);

void collider_get_rotation_out(const struct WorldHandle *world,
                               ColliderHandleRaw handle,
                               struct Quat *out_rotation);

struct Bool collider_set_pose(struct WorldHandle *world,
                              ColliderHandleRaw handle,
                              struct Vec3 translation,
                              struct Quat rotation);

uint8_t collider_set_pose_flag(struct WorldHandle *world,
                               ColliderHandleRaw handle,
                               struct Vec3 translation,
                               struct Quat rotation);

struct Bool collider_set_sensor(struct WorldHandle *world,
                                ColliderHandleRaw handle,
                                struct Bool sensor);

uint8_t collider_set_sensor_flag(struct WorldHandle *world,
                                 ColliderHandleRaw handle,
                                 struct Bool sensor);

struct Bool collider_set_friction(struct WorldHandle *world,
                                  ColliderHandleRaw handle,
                                  double friction);

uint8_t collider_set_friction_flag(struct WorldHandle *world,
                                   ColliderHandleRaw handle,
                                   double friction);

struct Bool collider_set_restitution(struct WorldHandle *world,
                                     ColliderHandleRaw handle,
                                     double restitution);

uint8_t collider_set_restitution_flag(struct WorldHandle *world,
                                      ColliderHandleRaw handle,
                                      double restitution);

struct Bool collider_set_collision_groups(struct WorldHandle *world,
                                          ColliderHandleRaw handle,
                                          struct InteractionGroupsDesc groups);

uint8_t collider_set_collision_groups_flag(struct WorldHandle *world,
                                           ColliderHandleRaw handle,
                                           struct InteractionGroupsDesc groups);

struct Bool collider_set_solver_groups(struct WorldHandle *world,
                                       ColliderHandleRaw handle,
                                       struct InteractionGroupsDesc groups);

uint8_t collider_set_solver_groups_flag(struct WorldHandle *world,
                                        ColliderHandleRaw handle,
                                        struct InteractionGroupsDesc groups);

struct Bool collider_set_active_events(struct WorldHandle *world,
                                       ColliderHandleRaw handle,
                                       uint32_t active_events_bits);

uint8_t collider_set_active_events_flag(struct WorldHandle *world,
                                        ColliderHandleRaw handle,
                                        uint32_t active_events_bits);

struct Bool collider_set_active_hooks(struct WorldHandle *world,
                                      ColliderHandleRaw handle,
                                      uint32_t active_hooks_bits);

uint8_t collider_set_active_hooks_flag(struct WorldHandle *world,
                                       ColliderHandleRaw handle,
                                       uint32_t active_hooks_bits);

struct Bool collider_set_contact_force_event_threshold(struct WorldHandle *world,
                                                       ColliderHandleRaw handle,
                                                       double threshold);

uint8_t collider_set_contact_force_event_threshold_flag(struct WorldHandle *world,
                                                        ColliderHandleRaw handle,
                                                        double threshold);

double collider_get_density(const struct WorldHandle *world, ColliderHandleRaw handle);

RigidBodyHandleRaw world_insert_dynamic_cuboids(struct WorldHandle *world,
                                                struct Vec3 translation,
                                                struct Quat rotation,
                                                struct Vec3 linvel,
                                                const double *cuboids,
                                                uint32_t cuboid_count,
                                                double density,
                                                double friction,
                                                double restitution,
                                                struct InteractionGroupsDesc collision_groups,
                                                struct InteractionGroupsDesc solver_groups);

RigidBodyHandleRaw world_insert_static_trimesh(struct WorldHandle *world,
                                               const double *vertices_xyz,
                                               uint32_t vertex_xyz_len,
                                               const uint32_t *indices,
                                               uint32_t index_len,
                                               double friction,
                                               double restitution);

uint32_t query_intersect_aabb_rigid_body_count(const struct WorldHandle *world,
                                               struct AabbDesc aabb,
                                               struct QueryFilterDesc filter);

uint32_t query_intersect_aabb_rigid_bodies(const struct WorldHandle *world,
                                           struct AabbDesc aabb,
                                           struct QueryFilterDesc filter,
                                           RigidBodyHandleRaw *out_handles,
                                           uint32_t capacity);

double continuum_tetra_volume(struct FemTetrahedron tetra);

struct Bool continuum_tetra_shape_functions(struct FemTetrahedron tetra,
                                            struct Vec3 point,
                                            struct FemShapeFunctionReport *out_report);

struct Bool continuum_linear_elastic_constitutive_matrix(struct MaterialProperties material,
                                                         double *out_matrix,
                                                         uint32_t capacity,
                                                         struct FemConstitutiveReport *out_report);

struct Bool continuum_tetra_strain_displacement_matrix(struct FemTetrahedron tetra,
                                                       double *out_matrix,
                                                       uint32_t capacity,
                                                       double *out_volume);

struct Bool continuum_newmark_beta_solve(const double *mass_matrix,
                                         const double *damping_matrix,
                                         const double *stiffness_matrix,
                                         const double *displacement,
                                         const double *velocity,
                                         const double *acceleration,
                                         const double *external_force,
                                         uint32_t dof,
                                         struct NewmarkBetaParameters params,
                                         double *out_delta_displacement,
                                         double *out_next_displacement,
                                         double *out_next_velocity,
                                         double *out_next_acceleration,
                                         uint32_t capacity,
                                         struct NewmarkBetaReport *out_report);

struct Bool continuum_linear_tetra_element_stiffness(struct FemTetrahedron tetra,
                                                     struct MaterialProperties material,
                                                     double *out_stiffness,
                                                     uint32_t capacity,
                                                     double *out_volume);

struct Bool continuum_deformation_gradient(struct FemTetrahedron reference_tetra,
                                           struct FemTetrahedron deformed_tetra,
                                           double *out_matrix,
                                           uint32_t capacity);

struct Bool control_pid_step(double setpoint,
                             double measurement,
                             double dt,
                             struct PidGains gains,
                             struct PidState *state,
                             struct PidReport *out_report);

struct Bool control_state_space_step(const double *a_matrix,
                                     const double *b_matrix,
                                     const double *c_matrix,
                                     const double *d_matrix,
                                     const double *state,
                                     const double *input,
                                     uint32_t state_count,
                                     uint32_t input_count,
                                     uint32_t output_count,
                                     double *out_next_state,
                                     double *out_output,
                                     uint32_t state_capacity,
                                     uint32_t output_capacity,
                                     struct StateSpaceReport *out_report);

struct Bool control_mpc_solve_box_qp(const double *a_matrix,
                                     const double *b_matrix,
                                     const double *q_diag,
                                     const double *r_diag,
                                     const double *initial_state,
                                     const double *target_state,
                                     struct MpcConfig config,
                                     double *out_first_control,
                                     uint32_t control_capacity,
                                     struct MpcReport *out_report);

struct Bool control_lqr_like_stabilizing_input(const double *state,
                                               const double *gain_matrix,
                                               uint32_t state_count,
                                               uint32_t input_count,
                                               double control_min,
                                               double control_max,
                                               double *out_control,
                                               uint32_t capacity);

struct CharacterControllerHandle *character_controller_create(void);

void character_controller_destroy(struct CharacterControllerHandle *controller);

void character_controller_set_up(struct CharacterControllerHandle *controller, struct Vec3 up);

void character_controller_set_offset_absolute(struct CharacterControllerHandle *controller,
                                              double offset);

void character_controller_set_offset_relative(struct CharacterControllerHandle *controller,
                                              double offset);

void character_controller_set_slide(struct CharacterControllerHandle *controller,
                                    struct Bool slide);

void character_controller_set_autostep(struct CharacterControllerHandle *controller,
                                       struct Bool enabled,
                                       double max_height,
                                       double min_width,
                                       struct Bool include_dynamic_bodies);

void character_controller_set_snap_to_ground(struct CharacterControllerHandle *controller,
                                             struct Bool enabled,
                                             double distance);

void character_controller_set_slope_angles(struct CharacterControllerHandle *controller,
                                           double max_climb_angle,
                                           double min_slide_angle);

struct EffectiveCharacterMovement character_controller_move_shape(const struct WorldHandle *world,
                                                                  struct CharacterControllerHandle *controller,
                                                                  double dt,
                                                                  struct ShapeDesc shape_desc,
                                                                  struct Vec3 translation,
                                                                  struct Quat rotation,
                                                                  struct Vec3 desired_translation);

uint32_t character_controller_collision_count(const struct CharacterControllerHandle *controller);

FfiCharacterCollision character_controller_get_collision(const struct CharacterControllerHandle *controller,
                                                         uint32_t index);

struct Bool character_controller_solve_impulses(struct WorldHandle *world,
                                                struct CharacterControllerHandle *controller,
                                                double dt,
                                                struct ShapeDesc shape_desc,
                                                double character_mass);

struct CRbTreeHandle *crb_tree_create(void);

void crb_tree_destroy(struct CRbTreeHandle *tree);

void crb_tree_clear(struct CRbTreeHandle *tree);

uint32_t crb_tree_len(const struct CRbTreeHandle *tree);

struct Bool crb_tree_insert(struct CRbTreeHandle *tree, uint64_t id, struct AabbDesc aabb);

uint8_t crb_tree_insert_flag(struct CRbTreeHandle *tree, uint64_t id, struct AabbDesc aabb);

struct Bool crb_tree_update(struct CRbTreeHandle *tree, uint64_t id, struct AabbDesc aabb);

struct Bool crb_tree_remove(struct CRbTreeHandle *tree, uint64_t id);

uint32_t crb_tree_query_aabb_count(const struct CRbTreeHandle *tree, struct AabbDesc aabb);

uint32_t crb_tree_query_aabb(const struct CRbTreeHandle *tree,
                             struct AabbDesc aabb,
                             uint64_t *out_ids,
                             uint32_t capacity);

struct ColliderBuilderHandle *collider_builder_create_kdop(const double *points_xyz,
                                                           uint32_t point_count,
                                                           uint32_t preset);

struct ColliderBuilderHandle *collider_builder_create_fdh(const double *points_xyz,
                                                          uint32_t point_count,
                                                          const double *directions_xyz,
                                                          uint32_t direction_count);

struct Bool em_lorentz_force(double charge,
                             struct Vec3 velocity,
                             struct ElectromagneticField field,
                             double mass,
                             struct LorentzForceReport *out_report);

struct Bool em_magnetic_flux(struct Vec3 magnetic_field,
                             struct Vec3 area_normal,
                             double area,
                             struct MagneticFluxReport *out_report);

struct Bool em_faraday_induction(double previous_flux,
                                 double current_flux,
                                 double dt,
                                 double turns,
                                 double resistance,
                                 struct FaradayInductionReport *out_report);

struct Bool em_maxwell_point_update(struct ElectromagneticField field,
                                    struct Vec3 curl_electric,
                                    struct Vec3 curl_magnetic,
                                    struct Vec3 current_density,
                                    double charge_density,
                                    double divergence_electric,
                                    double divergence_magnetic,
                                    double permittivity,
                                    double permeability,
                                    double dt,
                                    struct MaxwellPointReport *out_report);

struct Bool em_fdtd_yee_update(const struct Vec3 *electric_fields,
                               const struct Vec3 *magnetic_fields,
                               const struct Vec3 *curl_electric,
                               const struct Vec3 *curl_magnetic,
                               uint32_t cell_count,
                               double permittivity,
                               double permeability,
                               double conductivity,
                               double dt,
                               struct Vec3 *out_electric_fields,
                               struct Vec3 *out_magnetic_fields,
                               uint32_t capacity,
                               struct FdtdYeeReport *out_report);

double em_vacuum_permittivity(void);

double em_vacuum_permeability(void);

uint32_t last_error_code(void);

const char *last_error_message(void);

void last_error_clear(void);

struct Bool world_set_coulomb_friction_law(struct WorldHandle *world,
                                           struct CoulombFrictionLaw law);

uint8_t world_set_coulomb_friction_law_flag(struct WorldHandle *world,
                                            struct CoulombFrictionLaw law);

void world_clear_coulomb_friction_law(struct WorldHandle *world);

struct Bool world_get_coulomb_friction_law(const struct WorldHandle *world,
                                           struct CoulombFrictionLaw *out_law);

struct Bool world_set_air_drag_law(struct WorldHandle *world, struct AirDragLaw law);

uint8_t world_set_air_drag_law_flag(struct WorldHandle *world, struct AirDragLaw law);

void world_clear_air_drag_law(struct WorldHandle *world);

struct Bool world_get_air_drag_law(const struct WorldHandle *world, struct AirDragLaw *out_law);

struct Bool world_set_external_force_law(struct WorldHandle *world, struct ExternalForceLaw law);

uint8_t world_set_external_force_law_flag(struct WorldHandle *world, struct ExternalForceLaw law);

void world_clear_external_force_law(struct WorldHandle *world);

struct Bool world_get_external_force_law(const struct WorldHandle *world,
                                         struct ExternalForceLaw *out_law);

struct Bool world_set_newton_gravity_law(struct WorldHandle *world, struct NewtonGravityLaw law);

uint8_t world_set_newton_gravity_law_flag(struct WorldHandle *world, struct NewtonGravityLaw law);

void world_clear_newton_gravity_law(struct WorldHandle *world);

struct Bool world_get_newton_gravity_law(const struct WorldHandle *world,
                                         struct NewtonGravityLaw *out_law);

struct Bool world_get_custom_physics_report(const struct WorldHandle *world,
                                            struct CustomPhysicsReport *out_report);

void world_clear_events(struct WorldHandle *world);

uint32_t world_collision_event_count(const struct WorldHandle *world);

struct CollisionEventRecord world_get_collision_event(const struct WorldHandle *world,
                                                      uint32_t index);

uint32_t world_get_collision_events(const struct WorldHandle *world,
                                    struct CollisionEventRecord *out_events,
                                    uint32_t capacity);

uint32_t world_contact_force_event_count(const struct WorldHandle *world);

struct ContactForceEventRecord world_get_contact_force_event(const struct WorldHandle *world,
                                                             uint32_t index);

uint32_t world_get_contact_force_events(const struct WorldHandle *world,
                                        struct ContactForceEventRecord *out_events,
                                        uint32_t capacity);

void world_set_contact_pair_filter_callback(struct WorldHandle *world,
                                            uintptr_t _callback,
                                            uintptr_t _user_data);

void world_set_intersection_pair_filter_callback(struct WorldHandle *world,
                                                 uintptr_t _callback,
                                                 uintptr_t _user_data);

void world_clear_contact_pair_filter_callback(struct WorldHandle *world);

void world_clear_intersection_pair_filter_callback(struct WorldHandle *world);

/**
 * Allocate a collision-event ring buffer of `capacity` records.
 * Events will be written here during `world_step` instead of (or in addition to)
 * the legacy Vec queue.  Java drains the ring buffer at its own pace.
 */
struct Bool world_init_collision_event_ring(struct WorldHandle *world, uint32_t capacity);

/**
 * Allocate a contact-force-event ring buffer.
 */
struct Bool world_init_contact_force_event_ring(struct WorldHandle *world, uint32_t capacity);

/**
 * Drain the collision-event ring buffer into `out_events`.
 * Returns the number of events drained.  This is the **only** FFI call needed
 * per frame after init — no more count-then-allocate-then-read cycles.
 */
uint32_t world_drain_collision_event_ring(const struct WorldHandle *world,
                                          struct CollisionEventRecord *out_events,
                                          uint32_t capacity);

/**
 * Drain the contact-force-event ring buffer.
 */
uint32_t world_drain_contact_force_event_ring(const struct WorldHandle *world,
                                              struct ContactForceEventRecord *out_events,
                                              uint32_t capacity);

/**
 * Get the current number of events in the collision ring buffer (cheap, no lock).
 */
uint32_t world_collision_event_ring_len(const struct WorldHandle *world);

/**
 * Get the current number of events in the contact-force ring buffer.
 */
uint32_t world_contact_force_event_ring_len(const struct WorldHandle *world);

/**
 * Get ring buffer statistics (capacity, occupancy, drops, wraps).
 */
struct Bool world_collision_event_ring_stats(const struct WorldHandle *world,
                                             struct EventRingBufferStats *out_stats);

struct Bool world_contact_force_event_ring_stats(const struct WorldHandle *world,
                                                 struct EventRingBufferStats *out_stats);

/**
 * Clear both ring buffers and reset drop counters.
 */
void world_clear_event_rings(struct WorldHandle *world);

/**
 * Register a collision-event callback.
 *
 * `callback` is a C function pointer (zero = unregister).
 * `user_data` is passed through unchanged to each invocation.
 * Returns an opaque handle for later unregistration.
 */
EventCallbackHandle world_register_collision_callback(struct WorldHandle *world,
                                                      uintptr_t callback,
                                                      uintptr_t user_data);

/**
 * Register a contact-force-event callback.
 */
EventCallbackHandle world_register_contact_force_callback(struct WorldHandle *world,
                                                          uintptr_t callback,
                                                          uintptr_t user_data);

/**
 * Unregister a previously registered callback by its handle.
 * Passing 0 or an invalid handle is a no-op.
 */
void world_unregister_callback(struct WorldHandle *world, EventCallbackHandle handle);

/**
 * Set the event dispatch mode.
 *
 * - `Poll` (0): legacy Vec queue only (default).
 * - `Callback` (1): registered callbacks only.
 * - `Both` (2): ring buffer + callbacks.
 */
struct Bool world_set_event_dispatch_mode(struct WorldHandle *world, uint32_t mode);

struct Bool fluid_estimate_aabb_forces(struct FluidVolume fluid,
                                       struct Vec3 body_center,
                                       struct Vec3 body_half_extents,
                                       double body_volume,
                                       struct Vec3 body_linvel,
                                       struct Vec3 body_angvel,
                                       struct FluidForceReport *out_report);

struct Bool fluid_apply_aabb_forces(struct WorldHandle *world,
                                    RigidBodyHandleRaw body_handle,
                                    struct FluidVolume fluid,
                                    struct Vec3 body_half_extents,
                                    double body_volume,
                                    struct Bool wake_up,
                                    struct FluidForceReport *out_report);

uint8_t fluid_apply_aabb_forces_flag(struct WorldHandle *world,
                                     RigidBodyHandleRaw body_handle,
                                     struct FluidVolume fluid,
                                     struct Vec3 body_half_extents,
                                     double body_volume,
                                     struct Bool wake_up,
                                     struct FluidForceReport *out_report);

struct Bool fluid_navier_stokes_simplified_step(struct Vec3 velocity,
                                                struct Vec3 advection,
                                                struct Vec3 pressure_gradient,
                                                struct Vec3 laplacian_velocity,
                                                struct Vec3 external_acceleration,
                                                double density,
                                                double kinematic_viscosity,
                                                double dt,
                                                struct NavierStokesReport *out_report);

double fluid_sph_poly6_kernel(double distance, double smoothing_radius);

struct Bool fluid_sph_spiky_gradient(struct Vec3 offset,
                                     double smoothing_radius,
                                     struct Vec3 *out_gradient);

double fluid_sph_viscosity_laplacian(double distance, double smoothing_radius);

struct Bool fluid_sph_estimate_density(struct Vec3 position,
                                       const struct SphParticle *particles,
                                       uint32_t particle_count,
                                       double smoothing_radius,
                                       double *out_density);

struct Bool fluid_sph_estimate_forces(struct SphParticle particle,
                                      const struct SphParticle *particles,
                                      uint32_t particle_count,
                                      double smoothing_radius,
                                      double gas_constant,
                                      double rest_density,
                                      double viscosity,
                                      double surface_tension,
                                      struct SphForceReport *out_report);

double fluid_bernoulli_pressure(double total_pressure,
                                double density,
                                double velocity,
                                double gravity,
                                double elevation);

struct Bool fluid_bernoulli_report(double pressure,
                                   double density,
                                   double velocity,
                                   double gravity,
                                   double elevation,
                                   struct BernoulliReport *out_report);

struct Bool fracture_stress_intensity_factor(double stress,
                                             double crack_length,
                                             double geometry_factor,
                                             double fracture_toughness,
                                             struct StressIntensityReport *out_report);

struct Bool fracture_griffith_criterion(double stress,
                                        double crack_length,
                                        struct FractureMaterial material,
                                        struct GriffithReport *out_report);

struct Bool fracture_miner_damage(const double *cycle_counts,
                                  const double *cycles_to_failure,
                                  uint32_t count,
                                  struct MinerDamageReport *out_report);

struct Bool fracture_sn_curve_life(double stress_amplitude,
                                   double coefficient,
                                   double exponent,
                                   double endurance_limit,
                                   struct SnCurveReport *out_report);

struct Bool fracture_energy_release(double strain_energy,
                                    double new_surface_area,
                                    double surface_energy,
                                    double kinetic_energy,
                                    struct FractureEnergyReport *out_report);

struct Bool fracture_mode_from_stress(double tensile_stress,
                                      double shear_stress,
                                      double compressive_stress,
                                      struct FractureModeReport *out_report);

struct Bool world_replace_body_with_fracture_fragments(struct WorldHandle *world,
                                                       RigidBodyHandleRaw source_body,
                                                       const struct FractureFragmentDesc *fragments,
                                                       uint32_t fragment_count,
                                                       struct Bool connect_fragments,
                                                       struct Bool remove_source,
                                                       RigidBodyHandleRaw *out_body_handles,
                                                       ImpulseJointHandleRaw *out_joint_handles,
                                                       uint32_t capacity,
                                                       struct FractureReplaceReport *out_report);

struct JointBuilderHandle *joint_builder_create(uint32_t joint_type,
                                                struct Vec3 axis_or_primary,
                                                double b,
                                                double c);

void joint_builder_destroy(struct JointBuilderHandle *builder);

void joint_builder_set_contacts_enabled(struct JointBuilderHandle *builder, struct Bool enabled);

void joint_builder_set_local_anchor1(struct JointBuilderHandle *builder, struct Vec3 anchor);

void joint_builder_set_local_anchor2(struct JointBuilderHandle *builder, struct Vec3 anchor);

void joint_builder_set_limits(struct JointBuilderHandle *builder,
                              uint32_t axis,
                              double min,
                              double max);

void joint_builder_set_motor_velocity(struct JointBuilderHandle *builder,
                                      uint32_t axis,
                                      double target_vel,
                                      double factor);

void joint_builder_set_motor_position(struct JointBuilderHandle *builder,
                                      uint32_t axis,
                                      double target_pos,
                                      double stiffness,
                                      double damping);

ImpulseJointHandleRaw world_insert_impulse_joint(struct WorldHandle *world,
                                                 RigidBodyHandleRaw body1,
                                                 RigidBodyHandleRaw body2,
                                                 struct JointBuilderHandle *builder,
                                                 struct Bool wake_up);

struct Bool world_remove_impulse_joint(struct WorldHandle *world,
                                       ImpulseJointHandleRaw handle,
                                       struct Bool wake_up);

double molecular_lennard_jones_potential(double distance, double epsilon, double sigma);

struct Bool molecular_lennard_jones_force(struct Vec3 displacement,
                                          double epsilon,
                                          double sigma,
                                          double softening,
                                          struct Vec3 *out_force);

double molecular_coulomb_potential(double distance,
                                   double charge_a,
                                   double charge_b,
                                   double coulomb_constant,
                                   double relative_permittivity);

struct Bool molecular_coulomb_force(struct Vec3 displacement,
                                    double charge_a,
                                    double charge_b,
                                    double coulomb_constant,
                                    double relative_permittivity,
                                    double softening,
                                    struct Vec3 *out_force);

struct Bool molecular_pair_interaction(struct MolecularParticle particle_a,
                                       struct MolecularParticle particle_b,
                                       struct MolecularForceLaw law,
                                       struct MolecularPairReport *out_report);

struct Bool molecular_apply_pair_forces(struct WorldHandle *world,
                                        RigidBodyHandleRaw body_a,
                                        RigidBodyHandleRaw body_b,
                                        struct MolecularParticle particle_a,
                                        struct MolecularParticle particle_b,
                                        struct MolecularForceLaw law,
                                        struct Bool wake_up,
                                        struct MolecularPairReport *out_report);

uint8_t molecular_apply_pair_forces_flag(struct WorldHandle *world,
                                         RigidBodyHandleRaw body_a,
                                         RigidBodyHandleRaw body_b,
                                         struct MolecularParticle particle_a,
                                         struct MolecularParticle particle_b,
                                         struct MolecularForceLaw law,
                                         struct Bool wake_up,
                                         struct MolecularPairReport *out_report);

double molecular_vacuum_coulomb_constant(void);

uint32_t neural_bounds_required_weight_count(uint32_t hidden_width, uint32_t hidden_layers);

struct ColliderBuilderHandle *collider_builder_create_neural_bounds(struct NeuralBoundsDesc desc,
                                                                    const double *weights,
                                                                    uint32_t weight_count);

uint32_t query_intersect_neural_bounds_count(const struct WorldHandle *world,
                                             struct NeuralBoundsDesc desc,
                                             const double *weights,
                                             uint32_t weight_count,
                                             struct QueryFilterDesc filter);

uint32_t query_intersect_neural_bounds_count_all(const struct WorldHandle *world,
                                                 struct NeuralBoundsDesc desc,
                                                 const double *weights,
                                                 uint32_t weight_count);

uint32_t query_intersect_neural_bounds(const struct WorldHandle *world,
                                       struct NeuralBoundsDesc desc,
                                       const double *weights,
                                       uint32_t weight_count,
                                       struct QueryFilterDesc filter,
                                       ColliderHandleRaw *out_handles,
                                       uint32_t capacity);

uint32_t query_intersect_neural_bounds_all(const struct WorldHandle *world,
                                           struct NeuralBoundsDesc desc,
                                           const double *weights,
                                           uint32_t weight_count,
                                           ColliderHandleRaw *out_handles,
                                           uint32_t capacity);

struct Bool physchem_catalyst_rate_multiplier(double base_rate,
                                              struct CatalystEffect catalyst,
                                              struct CatalystReport *out_report);

struct Bool physchem_gray_scott_reaction_terms(double u,
                                               double v,
                                               double laplacian_u,
                                               double laplacian_v,
                                               struct GrayScottParams params,
                                               struct CatalystEffect catalyst,
                                               struct GrayScottReactionReport *out_report);

struct Bool physchem_gray_scott_step_2d(const double *u_values,
                                        const double *v_values,
                                        uint32_t width,
                                        uint32_t height,
                                        struct GrayScottParams params,
                                        struct CatalystEffect catalyst,
                                        double dt,
                                        double *out_u_values,
                                        double *out_v_values,
                                        uint32_t capacity,
                                        struct ReactionDiffusionReport *out_report);

double physchem_reaction_diffusion_explicit(double concentration,
                                            double laplacian,
                                            double diffusion_coefficient,
                                            double reaction_rate,
                                            double source,
                                            double dt);

struct Bool physchem_concentration_buoyancy(double concentration,
                                            double reference_concentration,
                                            double reference_density,
                                            double expansion_coefficient,
                                            double volume,
                                            struct Vec3 gravity,
                                            struct ConcentrationBuoyancyReport *out_report);

/**
 * Compute Debye length, plasma frequency, and related plasma parameters.
 *
 *   λ_D = sqrt(ε₀ k_B T_e / (n_e e²))
 *   ω_pe = sqrt(n_e e² / (ε₀ m_e))
 *   ω_pi = sqrt(n_i Z² e² / (ε₀ m_i))
 *   N_D = (4π/3) n_e λ_D³
 *   v_th = sqrt(k_B T_e / m_e)
 *
 * `electron_density` — n_e (m⁻³)
 * `electron_temperature` — T_e (K)
 * `ion_density` — n_i (m⁻³, typically ≈ n_e)
 * `ion_mass` — m_i (kg, e.g. 1.672e-27 for protons)
 * `ion_charge_state` — Z (e.g. 1 for singly ionised)
 */
struct Bool pl_plasma_params(double electron_density,
                             double electron_temperature,
                             double ion_density,
                             double ion_mass,
                             double ion_charge_state,
                             struct PlasmaParamsReport *out_params);

/**
 * Compute the Debye length directly from density and temperature.
 */
double pl_debye_length(double density, double temperature);

/**
 * Compute the plasma frequency from density.
 */
double pl_plasma_frequency(double density);

/**
 * Advance a single particle by one time step using the Boris algorithm.
 *
 * The Boris pusher is a second-order accurate, symplectic integrator for
 * charged particle motion in electromagnetic fields:
 *
 *   1. Half-step acceleration from E-field
 *   2. Rotation by B-field (gyration)
 *   3. Half-step acceleration from E-field (completing the step)
 *
 * References:
 *   Birdsall & Langdon, "Plasma Physics via Computer Simulation"
 */
struct Bool pl_boris_push(struct PicParticle particle,
                          struct GridField field,
                          struct BorisPusherParams params,
                          struct PicParticle *out_particle);

/**
 * Interpolate the electromagnetic field from a grid cell to the particle
 * position using first-order (linear / area-weighted) interpolation.
 *
 * `grid` — pointer to a 3D array of `GridField` of size nx × ny × nz,
 * stored in row-major order (x-fastest, then y, then z).
 * `cell_size` — grid cell size (uniform in all directions).
 * `origin_x/y/z` — position of grid cell centre (0,0,0).
 *
 * Returns the interpolated field at the particle position.
 */
struct Bool pl_interpolate_field(const struct GridField *grid,
                                 uint32_t nx,
                                 uint32_t ny,
                                 uint32_t nz,
                                 double cell_size,
                                 double origin_x,
                                 double origin_y,
                                 double origin_z,
                                 double particle_x,
                                 double particle_y,
                                 double particle_z,
                                 struct GridField *out_field);

/**
 * Deposit a single particle's charge and current onto a grid cell using
 * first-order (Cloud-in-Cell) weighting.
 *
 * The charge density contribution is: ρ = q · w / V_cell
 * The current density contribution is: j = ρ · v
 *
 * `cell_volume` — volume of a single grid cell (m³).
 */
struct Bool pl_deposit_particle(struct PicParticle particle,
                                double cell_size,
                                double cell_volume,
                                struct ChargeDensityCell *out_density);

/**
 * Compute velocity moments of a distribution function from a set of
 * macroparticles. This is a reduced representation of the Vlasov equation:
 *
 *   n = Σ wⱼ
 *   u = (1/n) Σ wⱼ vⱼ
 *   T = (m/3n) Σ wⱼ |vⱼ − u|²    (isotropic temperature)
 *   q = (m/2) Σ wⱼ (vⱼ − u)² (vⱼ − u)   (heat flux)
 *
 * `particles` — pointer to array of `PicParticle`.
 * `count` — number of particles.
 * `out_moments` — computed moments.
 */
struct Bool pl_vlasov_moments(const struct PicParticle *particles,
                              uint32_t count,
                              struct VlasovMomentReport *out_moments);

/**
 * Solve Poisson's equation ∇²φ = −ρ/ε₀ on a 1D grid using a simple
 * tridiagonal (finite-difference) solver with Dirichlet boundary conditions
 * (φ = 0 at both ends).
 *
 * `rho` — charge density array (C/m³), length `n`.
 * `dx` — grid spacing (m).
 * `phi_out` — pre-allocated output array for potential φ (V).
 * `e_out` — pre-allocated output array for electric field E = −dφ/dx (V/m).
 *
 * Returns Bool::TRUE on success.
 */
struct Bool pl_poisson_solve_1d(const double *rho,
                                uint32_t n,
                                double dx,
                                double *phi_out,
                                double *e_out);

/**
 * Detect a magnetic X-point (reconnection site) in a 2D plane from the
 * magnetic field components Bx, By on a regular grid.
 *
 * An X-point is characterised by B = 0 (or very small) with a hyperbolic
 * null topology: Bx ∝ (x − x₀), By ∝ −(y − y₀) (or rotated).
 *
 * `bx_grid` — pointer to Bx array (T), size nx × ny, row-major.
 * `by_grid` — pointer to By array (T).
 * `nx`, `ny` — grid dimensions.
 * `cell_size` — uniform grid cell size (m).
 * `origin_x`, `origin_y` — position of grid cell (0,0) centre.
 * `threshold` — maximum |B| at a null point (T).
 *
 * Returns the first X-point found (if any).
 */
struct Bool pl_find_xpoint(const double *bx_grid,
                           const double *by_grid,
                           uint32_t nx,
                           uint32_t ny,
                           double cell_size,
                           double origin_x,
                           double origin_y,
                           double threshold,
                           struct MagneticXPoint *out_xpoint);

/**
 * Compute the Sweet–Parker reconnection rate estimate.
 *
 *   R = v_in / v_A = 1 / √S
 *
 * where S = μ₀ L v_A / η is the Lundquist number.
 *
 * `lundquist_number` — S = μ₀ L_A v_A / η (dimensionless).
 */
double pl_sweet_parker_rate(double lundquist_number);

/**
 * Compute the Petschek fast reconnection rate estimate.
 *
 *   R ≈ π / (4 ln S)
 *
 * `lundquist_number` — S (dimensionless).
 */
double pl_petschek_rate(double lundquist_number);

/**
 * Compute the Alfvén speed v_A = B / √(μ₀ n m_i).
 */
double pl_alfven_speed(double magnetic_field, double density, double ion_mass);

/**
 * Compute the Lundquist number S = μ₀ L v_A / η.
 */
double pl_lundquist_number(double length_scale, double alfven_speed, double resistivity);

/**
 * Compute a summary report for a PIC simulation step from an array of
 * particles and a field grid.
 *
 * `particles` — pointer to array of `PicParticle`.
 * `particle_count` — number of particles.
 * `grid` — pointer to array of `GridField`.
 * `grid_cells` — total number of grid cells.
 */
struct Bool pl_pic_step_report(const struct PicParticle *particles,
                               uint32_t particle_count,
                               const struct GridField *grid,
                               uint32_t grid_cells,
                               struct PicStepReport *out_report);

double quantum_reduced_planck_constant(void);

double quantum_wave_probability_density(struct QuantumWaveFunction wave);

struct Bool quantum_wave_normalize(struct QuantumWaveFunction wave,
                                   struct QuantumWaveFunction *out_wave);

double quantum_wkb_transmission(double action_integral, double reduced_planck);

struct Bool quantum_rectangular_barrier_tunneling(struct QuantumBarrier barrier,
                                                  struct QuantumTunnelingReport *out_report);

double quantum_rectangular_barrier_probability(struct QuantumBarrier barrier);

double quantum_zero_point_energy(double angular_frequency, double reduced_planck);

struct Bool quantum_harmonic_oscillator_report(double angular_frequency,
                                               double reduced_planck,
                                               struct QuantumOscillatorReport *out_report);

struct RayHit query_cast_ray(const struct WorldHandle *world,
                             struct Vec3 origin,
                             struct Vec3 direction,
                             double max_toi,
                             struct Bool solid,
                             struct QueryFilterDesc filter);

ColliderHandleRaw query_cast_ray_out(const struct WorldHandle *world,
                                     struct Vec3 origin,
                                     struct Vec3 direction,
                                     double max_toi,
                                     struct Bool solid,
                                     struct QueryFilterDesc filter,
                                     struct RayHit *out_hit);

uint32_t query_cast_rays(const struct WorldHandle *world,
                         const double *rays,
                         uint32_t ray_count,
                         double max_toi,
                         struct Bool solid,
                         struct QueryFilterDesc filter,
                         struct RayHit *out_hits,
                         uint32_t capacity);

struct PointProjection query_project_point(const struct WorldHandle *world,
                                           struct Vec3 point,
                                           double max_dist,
                                           struct Bool solid,
                                           struct QueryFilterDesc filter,
                                           ColliderHandleRaw *out_collider);

ColliderHandleRaw query_project_point_out(const struct WorldHandle *world,
                                          struct Vec3 point,
                                          double max_dist,
                                          struct Bool solid,
                                          struct QueryFilterDesc filter,
                                          ColliderHandleRaw *out_collider,
                                          struct PointProjection *out_projection);

uint32_t query_intersect_point_count(const struct WorldHandle *world,
                                     struct Vec3 point,
                                     struct QueryFilterDesc filter);

uint32_t query_intersect_aabb_count(const struct WorldHandle *world,
                                    struct AabbDesc aabb,
                                    struct QueryFilterDesc filter);

uint32_t query_intersect_aabb(const struct WorldHandle *world,
                              struct AabbDesc aabb,
                              struct QueryFilterDesc filter,
                              ColliderHandleRaw *out_handles,
                              uint32_t capacity);

uint32_t query_intersect_aabb_count_all(const struct WorldHandle *world, struct AabbDesc aabb);

uint32_t query_intersect_aabb_counts(const struct WorldHandle *world,
                                     const struct AabbDesc *aabbs,
                                     uint32_t query_count,
                                     struct QueryFilterDesc filter,
                                     uint32_t *out_counts,
                                     uint32_t capacity);

uint32_t query_intersect_obb_count(const struct WorldHandle *world,
                                   struct Obb obb,
                                   struct QueryFilterDesc filter);

uint32_t query_intersect_obb_count_all(const struct WorldHandle *world, struct Obb obb);

uint32_t query_intersect_obb_counts(const struct WorldHandle *world,
                                    const struct Obb *obbs,
                                    uint32_t query_count,
                                    struct QueryFilterDesc filter,
                                    uint32_t *out_counts,
                                    uint32_t capacity);

uint32_t query_intersect_obb(const struct WorldHandle *world,
                             struct Obb obb,
                             struct QueryFilterDesc filter,
                             ColliderHandleRaw *out_handles,
                             uint32_t capacity);

uint32_t query_intersect_obb_all(const struct WorldHandle *world,
                                 struct Obb obb,
                                 ColliderHandleRaw *out_handles,
                                 uint32_t capacity);

uint32_t query_intersect_sphere_count(const struct WorldHandle *world,
                                      struct Sphere sphere,
                                      struct QueryFilterDesc filter);

uint32_t query_intersect_sphere_count_all(const struct WorldHandle *world, struct Sphere sphere);

uint32_t query_intersect_sphere_counts(const struct WorldHandle *world,
                                       const struct Sphere *spheres,
                                       uint32_t query_count,
                                       struct QueryFilterDesc filter,
                                       uint32_t *out_counts,
                                       uint32_t capacity);

uint32_t query_intersect_sphere(const struct WorldHandle *world,
                                struct Sphere sphere,
                                struct QueryFilterDesc filter,
                                ColliderHandleRaw *out_handles,
                                uint32_t capacity);

uint32_t query_intersect_sphere_all(const struct WorldHandle *world,
                                    struct Sphere sphere,
                                    ColliderHandleRaw *out_handles,
                                    uint32_t capacity);

uint32_t query_intersect_aabb_rigid_body_count_all(const struct WorldHandle *world,
                                                   struct AabbDesc aabb);

uint32_t query_intersect_aabb_rigid_bodies_all(const struct WorldHandle *world,
                                               struct AabbDesc aabb,
                                               RigidBodyHandleRaw *out_handles,
                                               uint32_t capacity);

struct ShapeCastHit query_cast_shape(const struct WorldHandle *world,
                                     struct ShapeDesc shape_desc,
                                     struct Vec3 translation,
                                     struct Quat rotation,
                                     struct Vec3 velocity,
                                     struct ShapeCastOptionsDesc options,
                                     struct QueryFilterDesc filter);

ColliderHandleRaw query_cast_shape_out(const struct WorldHandle *world,
                                       struct ShapeDesc shape_desc,
                                       struct Vec3 translation,
                                       struct Quat rotation,
                                       struct Vec3 velocity,
                                       struct ShapeCastOptionsDesc options,
                                       struct QueryFilterDesc filter,
                                       struct ShapeCastHit *out_hit);

/**
 * Compute the Lorentz factor gamma = 1/sqrt(1 - v^2/c^2).
 */
struct Bool rel_lorentz_factor(double speed, double *out_gamma);

/**
 * Build the full 4x4 Lorentz boost matrix for a given velocity 3-vector.
 *
 * The matrix acts on column 4-vectors (ct, x, y, z)^T.
 */
struct Bool rel_lorentz_boost(struct Vec3 velocity, struct LorentzBoost *out_boost);

/**
 * Apply a Lorentz boost to a 4-vector (ct, x, y, z).
 */
struct Bool rel_transform_four_vector(struct LorentzBoost boost,
                                      double ct,
                                      double x,
                                      double y,
                                      double z,
                                      struct LorentzTransformedFrame *out_transformed);

/**
 * Relativistic velocity addition (3D general formula).
 *
 * w = (u + v_∥ + v_⊥/γ_u) / (1 + u·v/c²)
 */
struct Bool rel_velocity_addition(struct Vec3 u, struct Vec3 v, struct Vec3 *out_result);

/**
 * Rapidity = arctanh(v/c).
 */
double rel_rapidity(double speed);

/**
 * Beta (v/c) from Lorentz factor: beta = sqrt(1 - 1/gamma^2).
 */
double rel_beta_from_gamma(double gamma);

/**
 * Return the speed of light constant.
 */
double rel_speed_of_light(void);

/**
 * Compute the Schwarzschild radius rs = 2GM/c^2.
 */
double rel_schwarzschild_radius(double mass, double gravitational_constant);

/**
 * Compute the Schwarzschild metric coefficients at a given radius.
 */
struct Bool rel_schwarzschild_metric(double radius,
                                     double mass,
                                     double gravitational_constant,
                                     struct SchwarzschildMetric *out_metric);

/**
 * Einstein light deflection angle: delta_phi = 4GM/(b*c^2).
 *
 * Returns the deflection angle in radians. Returns ERR_UNSUPPORTED when the
 * impact parameter is close to the photon sphere (b < 2.6 * rs).
 */
double rel_light_deflection_angle(double impact_parameter,
                                  double mass,
                                  double gravitational_constant);

/**
 * Effective potential for Schwarzschild orbits (per unit mass m of the orbiting body).
 *
 * V_eff(r) = -GM/r + L^2/(2*r^2) - G*M*L^2/(c^2*r^3)
 *
 * The orbiting body's mass m and angular momentum L are parameters.
 */
struct Bool rel_effective_potential(double radius,
                                    double angular_momentum,
                                    double mass,
                                    double gravitational_constant,
                                    double *out_potential);

/**
 * Compute gravitational time dilation factors.
 *
 * Stationary factor: dtau/dt = sqrt(1 - rs/r)
 * Orbital factor (circular orbit): dtau/dt = sqrt(1 - 3*rs/(2*r))
 */
struct Bool rel_gravitational_time_dilation(double radius,
                                            double mass,
                                            double gravitational_constant,
                                            struct GravitationalTimeDilation *out_dilation);

/**
 * Lightweight gravitational time dilation: returns sqrt(1 - rs/r) directly.
 */
double rel_gravitational_time_dilation_simple(double radius, double schwarzschild_radius);

/**
 * Compute length contraction: L = L0 / gamma.
 */
struct Bool rel_length_contraction(double proper_length,
                                   double speed,
                                   struct LengthContraction *out_contraction);

/**
 * Compute relativistic particle properties.
 *
 * For zero mass (photon-like), speed must equal c and the particle has
 * no well-defined gamma from velocity alone — gamma and total energy
 * are returned as INFINITY, and momentum is set to a unit vector scaled
 * by INFINITY (direction only).
 */
struct Bool rel_particle_properties(double mass,
                                    struct Vec3 velocity,
                                    struct RelativisticParticle *out_particle);

/**
 * Compute the invariant (rest) mass from energy and momentum:
 *
 * m0 = sqrt(E^2/c^4 - p^2/c^2)
 *
 * Returns NAN for tachyonic states (E^2 < p^2 * c^2).
 */
double rel_invariant_mass(double energy, double px, double py, double pz);

struct RigidBodyBuilderHandle *rigid_body_builder_create(uint32_t status);

RigidBody *rigid_body_builder_build(struct RigidBodyBuilderHandle *builder);

void rigid_body_builder_destroy(struct RigidBodyBuilderHandle *builder);

void rigid_body_destroy_raw(RigidBody *rigid_body);

void rigid_body_builder_set_translation(struct RigidBodyBuilderHandle *builder,
                                        struct Vec3 translation);

void rigid_body_builder_set_rotation(struct RigidBodyBuilderHandle *builder,
                                     struct Vec3 rotation_axis_angle);

void rigid_body_builder_set_pose(struct RigidBodyBuilderHandle *builder,
                                 struct Vec3 translation,
                                 struct Quat rotation);

void rigid_body_builder_set_additional_mass_properties(struct RigidBodyBuilderHandle *builder,
                                                       struct Vec3 center,
                                                       double mass,
                                                       struct Vec3 inertia);

void rigid_body_builder_set_linvel(struct RigidBodyBuilderHandle *builder, struct Vec3 linvel);

void rigid_body_builder_set_angvel(struct RigidBodyBuilderHandle *builder, struct Vec3 angvel);

void rigid_body_builder_set_gravity_scale(struct RigidBodyBuilderHandle *builder,
                                          double gravity_scale);

void rigid_body_builder_set_linear_damping(struct RigidBodyBuilderHandle *builder,
                                           double linear_damping);

void rigid_body_builder_set_angular_damping(struct RigidBodyBuilderHandle *builder,
                                            double angular_damping);

void rigid_body_builder_set_can_sleep(struct RigidBodyBuilderHandle *builder,
                                      struct Bool can_sleep);

void rigid_body_builder_set_enabled_rotations(struct RigidBodyBuilderHandle *builder,
                                              struct Bool allow_x,
                                              struct Bool allow_y,
                                              struct Bool allow_z);

void rigid_body_builder_set_user_data(struct RigidBodyBuilderHandle *builder,
                                      uint64_t user_data_low,
                                      uint64_t user_data_high);

void rigid_body_builder_set_additional_mass(struct RigidBodyBuilderHandle *builder, double mass);

RigidBodyHandleRaw world_insert_rigid_body(struct WorldHandle *world, RigidBody *memory_handle);

struct Bool world_remove_rigid_body(struct WorldHandle *world,
                                    RigidBodyHandleRaw handle,
                                    struct Bool remove_attached_colliders);

RigidBody *world_copy_rigid_body(struct WorldHandle *world, RigidBodyHandleRaw handle);

uint8_t world_remove_rigid_body_flag(struct WorldHandle *world,
                                     RigidBodyHandleRaw handle,
                                     struct Bool remove_attached_colliders);

uint32_t rigid_body_get_status(const struct WorldHandle *world, RigidBodyHandleRaw handle);

struct Bool rigid_body_set_status(struct WorldHandle *world,
                                  RigidBodyHandleRaw handle,
                                  uint32_t status,
                                  struct Bool wake_up);

struct Vec3 rigid_body_get_translation(const struct WorldHandle *world, RigidBodyHandleRaw handle);

void rigid_body_get_translation_out(const struct WorldHandle *world,
                                    RigidBodyHandleRaw handle,
                                    struct Vec3 *out_translation);

struct Quat rigid_body_get_rotation(const struct WorldHandle *world, RigidBodyHandleRaw handle);

void rigid_body_get_rotation_out(const struct WorldHandle *world,
                                 RigidBodyHandleRaw handle,
                                 struct Quat *out_rotation);

struct Bool rigid_body_set_pose(struct WorldHandle *world,
                                RigidBodyHandleRaw handle,
                                struct Vec3 translation,
                                struct Quat rotation,
                                struct Bool wake_up);

struct Bool rigid_body_set_translation(struct WorldHandle *world,
                                       RigidBodyHandleRaw handle,
                                       struct Vec3 translation,
                                       struct Bool wake_up);

uint8_t rigid_body_set_translation_flag(struct WorldHandle *world,
                                        RigidBodyHandleRaw handle,
                                        struct Vec3 translation,
                                        struct Bool wake_up);

struct Bool rigid_body_set_rotation(struct WorldHandle *world,
                                    RigidBodyHandleRaw handle,
                                    struct Quat rotation,
                                    struct Bool wake_up);

uint8_t rigid_body_set_rotation_flag(struct WorldHandle *world,
                                     RigidBodyHandleRaw handle,
                                     struct Quat rotation,
                                     struct Bool wake_up);

uint8_t rigid_body_set_pose_flag(struct WorldHandle *world,
                                 RigidBodyHandleRaw handle,
                                 struct Vec3 translation,
                                 struct Quat rotation,
                                 struct Bool wake_up);

struct Vec3 rigid_body_get_linvel(const struct WorldHandle *world, RigidBodyHandleRaw handle);

void rigid_body_get_linvel_out(const struct WorldHandle *world,
                               RigidBodyHandleRaw handle,
                               struct Vec3 *out_linvel);

struct Bool rigid_body_set_linvel(struct WorldHandle *world,
                                  RigidBodyHandleRaw handle,
                                  struct Vec3 linvel,
                                  struct Bool wake_up);

uint8_t rigid_body_set_linvel_flag(struct WorldHandle *world,
                                   RigidBodyHandleRaw handle,
                                   struct Vec3 linvel,
                                   struct Bool wake_up);

struct Vec3 rigid_body_get_angvel(const struct WorldHandle *world, RigidBodyHandleRaw handle);

void rigid_body_get_angvel_out(const struct WorldHandle *world,
                               RigidBodyHandleRaw handle,
                               struct Vec3 *out_angvel);

struct Bool rigid_body_set_angvel(struct WorldHandle *world,
                                  RigidBodyHandleRaw handle,
                                  struct Vec3 angvel,
                                  struct Bool wake_up);

uint8_t rigid_body_set_angvel_flag(struct WorldHandle *world,
                                   RigidBodyHandleRaw handle,
                                   struct Vec3 angvel,
                                   struct Bool wake_up);

struct Bool rigid_body_add_force(struct WorldHandle *world,
                                 RigidBodyHandleRaw handle,
                                 struct Vec3 force,
                                 struct Bool wake_up);

uint8_t rigid_body_add_force_flag(struct WorldHandle *world,
                                  RigidBodyHandleRaw handle,
                                  struct Vec3 force,
                                  struct Bool wake_up);

struct Bool rigid_body_add_torque(struct WorldHandle *world,
                                  RigidBodyHandleRaw handle,
                                  struct Vec3 torque,
                                  struct Bool wake_up);

uint8_t rigid_body_add_torque_flag(struct WorldHandle *world,
                                   RigidBodyHandleRaw handle,
                                   struct Vec3 torque,
                                   struct Bool wake_up);

struct Bool rigid_body_apply_impulse(struct WorldHandle *world,
                                     RigidBodyHandleRaw handle,
                                     struct Vec3 impulse,
                                     struct Bool wake_up);

uint8_t rigid_body_apply_impulse_flag(struct WorldHandle *world,
                                      RigidBodyHandleRaw handle,
                                      struct Vec3 impulse,
                                      struct Bool wake_up);

struct Bool rigid_body_apply_torque_impulse(struct WorldHandle *world,
                                            RigidBodyHandleRaw handle,
                                            struct Vec3 torque_impulse,
                                            struct Bool wake_up);

uint8_t rigid_body_apply_torque_impulse_flag(struct WorldHandle *world,
                                             RigidBodyHandleRaw handle,
                                             struct Vec3 torque_impulse,
                                             struct Bool wake_up);

struct Bool rigid_body_enable_ccd(struct WorldHandle *world,
                                  RigidBodyHandleRaw handle,
                                  struct Bool enabled);

uint8_t rigid_body_enable_ccd_flag(struct WorldHandle *world,
                                   RigidBodyHandleRaw handle,
                                   struct Bool enabled);

struct Bool rigid_body_sleep(struct WorldHandle *world, RigidBodyHandleRaw handle);

uint8_t rigid_body_sleep_flag(struct WorldHandle *world, RigidBodyHandleRaw handle);

struct Bool rigid_body_wake_up(struct WorldHandle *world,
                               RigidBodyHandleRaw handle,
                               struct Bool strong);

uint8_t rigid_body_wake_up_flag(struct WorldHandle *world,
                                RigidBodyHandleRaw handle,
                                struct Bool strong);

struct Bool rigid_body_is_sleeping(const struct WorldHandle *world, RigidBodyHandleRaw handle);

uint8_t rigid_body_is_sleeping_flag(const struct WorldHandle *world, RigidBodyHandleRaw handle);

struct RTreeHandle *rtree_create(void);

void rtree_destroy(struct RTreeHandle *tree);

void rtree_clear(struct RTreeHandle *tree);

uint32_t rtree_len(const struct RTreeHandle *tree);

struct Bool rtree_insert(struct RTreeHandle *tree, uint64_t id, struct AabbDesc aabb);

struct Bool rtree_update(struct RTreeHandle *tree, uint64_t id, struct AabbDesc aabb);

struct Bool rtree_remove(struct RTreeHandle *tree, uint64_t id);

void rtree_rebuild(struct RTreeHandle *tree);

uint32_t rtree_query_aabb_count(struct RTreeHandle *tree, struct AabbDesc aabb);

uint32_t rtree_query_aabb(struct RTreeHandle *tree,
                          struct AabbDesc aabb,
                          uint64_t *out_ids,
                          uint32_t capacity);

struct Bool softbody_predict_positions(const struct Vec3 *positions,
                                       const struct Vec3 *velocities,
                                       const double *inverse_masses,
                                       uint32_t particle_count,
                                       struct Vec3 gravity,
                                       double damping,
                                       double dt,
                                       struct Vec3 *out_predicted_positions,
                                       uint32_t capacity,
                                       struct SoftBodyStepReport *out_report);

struct Bool softbody_mass_spring_forces(const struct Vec3 *positions,
                                        const struct Vec3 *velocities,
                                        uint32_t particle_count,
                                        const struct SoftSpring *springs,
                                        uint32_t spring_count,
                                        struct Vec3 *out_forces,
                                        uint32_t force_capacity,
                                        struct SoftBodyStepReport *out_report);

struct Bool softbody_solve_xpbd_distance_constraints(struct Vec3 *positions,
                                                     const double *inverse_masses,
                                                     uint32_t particle_count,
                                                     struct SoftDistanceConstraint *constraints,
                                                     uint32_t constraint_count,
                                                     double dt,
                                                     uint32_t iterations,
                                                     struct SoftBodyStepReport *out_report);

struct Bool softbody_solve_xpbd_bending_constraints(struct Vec3 *positions,
                                                    const double *inverse_masses,
                                                    uint32_t particle_count,
                                                    struct SoftBendingConstraint *constraints,
                                                    uint32_t constraint_count,
                                                    double dt,
                                                    uint32_t iterations,
                                                    struct SoftBodyStepReport *out_report);

struct Bool softbody_solve_sphere_collision_constraints(struct Vec3 *positions,
                                                        const double *inverse_masses,
                                                        uint32_t particle_count,
                                                        const struct SoftSphereCollision *spheres,
                                                        uint32_t sphere_count,
                                                        struct SoftBodyStepReport *out_report);

struct Bool softbody_solve_xpbd_volume_constraints(struct Vec3 *positions,
                                                   const double *inverse_masses,
                                                   uint32_t particle_count,
                                                   struct SoftVolumeConstraint *constraints,
                                                   uint32_t constraint_count,
                                                   double dt,
                                                   uint32_t iterations,
                                                   struct SoftBodyStepReport *out_report);

struct Bool softbody_update_velocities(const struct Vec3 *previous_positions,
                                       const struct Vec3 *current_positions,
                                       uint32_t particle_count,
                                       double dt,
                                       struct Vec3 *out_velocities,
                                       uint32_t capacity,
                                       struct SoftBodyStepReport *out_report);

double space_kepler_period(double mu, double semi_major_axis);

double space_kepler_semi_major_axis(double mu, double period);

struct Bool space_elements_to_state(struct OrbitalElements elements,
                                    double mu,
                                    struct StateVector *out_state);

struct Bool space_state_to_elements(struct StateVector state,
                                    double mu,
                                    struct OrbitalElements *out_elements);

struct Bool space_j2_acceleration(struct Vec3 position,
                                  double mu,
                                  double equatorial_radius,
                                  double j2,
                                  struct Vec3 *out_acceleration);

struct Bool space_apply_j2_force_to_body(struct WorldHandle *world,
                                         RigidBodyHandleRaw body_handle,
                                         double mu,
                                         double equatorial_radius,
                                         double j2,
                                         double mass,
                                         struct Bool wake_up,
                                         struct Vec3 *out_acceleration);

uint8_t space_apply_j2_force_to_body_flag(struct WorldHandle *world,
                                          RigidBodyHandleRaw body_handle,
                                          double mu,
                                          double equatorial_radius,
                                          double j2,
                                          double mass,
                                          struct Bool wake_up,
                                          struct Vec3 *out_acceleration);

struct Bool space_quaternion_derivative(struct Quat attitude,
                                        struct Vec3 angular_velocity,
                                        struct QuaternionDerivative *out_derivative);

struct Bool space_rigid_body_euler_derivative(struct Vec3 inertia_diag,
                                              struct Vec3 angular_velocity,
                                              struct Vec3 torque,
                                              struct RigidBodyEulerDerivative *out_derivative);

struct Bool space_cmg_exchange(struct Vec3 gimbal_axis,
                               struct Vec3 wheel_momentum,
                               double gimbal_rate,
                               struct CmgExchange *out_exchange);

struct Bool space_apply_cmg_torque_to_body(struct WorldHandle *world,
                                           RigidBodyHandleRaw body_handle,
                                           struct Vec3 gimbal_axis,
                                           struct Vec3 wheel_momentum,
                                           double gimbal_rate,
                                           struct Bool wake_up,
                                           struct CmgExchange *out_exchange);

uint8_t space_apply_cmg_torque_to_body_flag(struct WorldHandle *world,
                                            RigidBodyHandleRaw body_handle,
                                            struct Vec3 gimbal_axis,
                                            struct Vec3 wheel_momentum,
                                            double gimbal_rate,
                                            struct Bool wake_up,
                                            struct CmgExchange *out_exchange);

struct Bool space_cw_derivative(struct CwState state,
                                double mean_motion,
                                struct CwDerivative *out_derivative);

double space_lambert_time_elliptic(double mu,
                                   double semi_major_axis,
                                   double alpha,
                                   double beta,
                                   uint32_t revolutions);

struct Bool space_dh_transform(double theta,
                               double d,
                               double a,
                               double alpha,
                               struct DhTransform *out_transform);

double space_arm_first_joint_inverse(double wrist_x, double wrist_y);

double space_arm_third_joint_angle(double planar_radius,
                                   double vertical_offset,
                                   double link2,
                                   double link3,
                                   struct Bool elbow_up);

struct Bool space_manipulator_dynamics_diag(struct Vec3 mass_matrix_diag,
                                            struct Vec3 joint_acceleration,
                                            struct Vec3 coriolis,
                                            struct Vec3 gravity,
                                            struct ManipulatorDynamics *out_dynamics);

struct Bool space_solar_panel_power(double solar_flux,
                                    double area,
                                    double efficiency,
                                    double incidence_angle,
                                    double degradation,
                                    struct SolarPanelPower *out_power);

struct Bool space_thermal_balance(double absorbed_power,
                                  double internal_power,
                                  double emitted_area,
                                  double emissivity,
                                  struct ThermalBalance *out_balance);

struct Bool space_co2_mass_balance(double current_mass,
                                   double generation_rate,
                                   double removal_rate,
                                   double leakage_rate,
                                   double volume,
                                   double dt,
                                   struct Co2MassBalance *out_balance);

struct Bool space_friis_link(double transmit_power,
                             double transmit_gain,
                             double receive_gain,
                             double wavelength,
                             double range,
                             double system_loss,
                             struct FriisLink *out_link);

double space_friis_wavelength_from_frequency(double frequency);

double space_tsiolkovsky_delta_v(double specific_impulse,
                                 double standard_gravity,
                                 double initial_mass,
                                 double final_mass);

struct Bool space_hohmann_transfer(double mu,
                                   double radius1,
                                   double radius2,
                                   struct HohmannTransfer *out_transfer);

double space_atmospheric_density_scale_height(double reference_density,
                                              double altitude,
                                              double reference_altitude,
                                              double scale_height);

struct Bool space_atmospheric_drag_acceleration(struct Vec3 velocity,
                                                struct Vec3 atmosphere_velocity,
                                                double density,
                                                double drag_coefficient,
                                                double area,
                                                double mass,
                                                struct Vec3 *out_acceleration);

struct Bool space_apply_atmospheric_drag_to_body(struct WorldHandle *world,
                                                 RigidBodyHandleRaw body_handle,
                                                 struct Vec3 atmosphere_velocity,
                                                 double density,
                                                 double drag_coefficient,
                                                 double area,
                                                 double mass,
                                                 struct Bool wake_up,
                                                 struct Vec3 *out_acceleration);

uint8_t space_apply_atmospheric_drag_to_body_flag(struct WorldHandle *world,
                                                  RigidBodyHandleRaw body_handle,
                                                  struct Vec3 atmosphere_velocity,
                                                  double density,
                                                  double drag_coefficient,
                                                  double area,
                                                  double mass,
                                                  struct Bool wake_up,
                                                  struct Vec3 *out_acceleration);

struct Bool space_triad_attitude(struct Vec3 body_primary,
                                 struct Vec3 body_secondary,
                                 struct Vec3 reference_primary,
                                 struct Vec3 reference_secondary,
                                 struct Quat *out_attitude);

struct Bool space_ekf_predict_scalar(double state,
                                     double covariance,
                                     double nonlinear_delta,
                                     double jacobian,
                                     double process_noise,
                                     struct ScalarKalman *out_prediction);

double space_ekf_gain_scalar(double covariance,
                             double measurement_jacobian,
                             double measurement_noise);

struct Bool space_ekf_update_scalar(double predicted_state,
                                    double predicted_covariance,
                                    double measurement,
                                    double predicted_measurement,
                                    double kalman_gain,
                                    double measurement_jacobian,
                                    struct ScalarKalman *out_update);

struct Bool space_least_squares_attitude_two_vector(struct Vec3 body_primary,
                                                    struct Vec3 body_secondary,
                                                    struct Vec3 reference_primary,
                                                    struct Vec3 reference_secondary,
                                                    struct LeastSquaresAttitude *out_attitude);

struct Bool space_gnss_pseudorange(struct Vec3 receiver,
                                   struct Vec3 satellite,
                                   double receiver_clock_bias,
                                   double satellite_clock_bias,
                                   double ionosphere_delay,
                                   double troposphere_delay,
                                   struct GnssObservation *out_observation);

double space_gnss_double_difference_carrier_phase(double range_rover_sat_a,
                                                  double range_rover_sat_b,
                                                  double range_base_sat_a,
                                                  double range_base_sat_b,
                                                  double wavelength,
                                                  double ambiguity);

double space_structural_natural_frequency(double stiffness, double mass, double mode_factor);

struct Bool space_contact_force_hunt_crossley(double penetration,
                                              double penetration_rate,
                                              double stiffness,
                                              double damping,
                                              double exponent,
                                              struct ContactForceModel *out_force);

double space_radiation_absorbed_dose(double energy_joules, double mass_kg, double quality_factor);

double space_semi_major_axis_decay_rate(double semi_major_axis,
                                        double density,
                                        double drag_coefficient,
                                        double area,
                                        double mass,
                                        double mu);

double space_heat_pipe_thermal_resistance(double evaporator_resistance,
                                          double vapor_resistance,
                                          double condenser_resistance,
                                          double wick_resistance);

struct Bool space_battery_equivalent_circuit(double open_circuit_voltage,
                                             double current,
                                             double ohmic_resistance,
                                             double rc_voltage,
                                             double rc_resistance,
                                             double rc_capacitance,
                                             double capacity_coulombs,
                                             struct BatteryEquivalentCircuit *out_battery);

struct Bool space_hall_thruster_performance(double mass_flow_rate,
                                            double exhaust_velocity,
                                            double input_power,
                                            double standard_gravity,
                                            struct HallThrusterPerformance *out_performance);

struct Bool space_artificial_potential_guidance(struct Vec3 position,
                                                struct Vec3 target,
                                                struct Vec3 obstacle,
                                                double attractive_gain,
                                                double repulsive_gain,
                                                double influence_radius,
                                                struct Vec3 *out_command);

struct Bool space_debris_collision_probability(double miss_distance,
                                               double combined_radius,
                                               double sigma_radial,
                                               double sigma_intrack,
                                               struct CollisionProbability *out_probability);

struct Bool space_atomic_oxygen_erosion(double fluence,
                                        double erosion_yield,
                                        double area,
                                        double density,
                                        struct AtomicOxygenErosion *out_erosion);

struct Bool space_flexible_mode_derivative(double displacement,
                                           double velocity,
                                           double natural_frequency,
                                           double damping_ratio,
                                           double modal_force,
                                           double modal_mass,
                                           struct FlexibleModeDerivative *out_derivative);

struct Bool space_slosh_pendulum_derivative(double angle,
                                            double angular_rate,
                                            double length,
                                            double damping,
                                            double lateral_acceleration,
                                            double gravity,
                                            struct SloshPendulumDerivative *out_derivative);

struct Bool space_variational_two_body(struct Vec3 position,
                                       struct Vec3 velocity,
                                       double mu,
                                       struct VariationalState *out_derivative);

struct Bool space_single_phase_loop_heat_transfer(double mass_flow_rate,
                                                  double specific_heat,
                                                  double inlet_temperature,
                                                  double heat_input,
                                                  struct FluidLoopHeatTransfer *out_heat);

struct Bool space_radar_range_rate(struct Vec3 radar_position,
                                   struct Vec3 target_position,
                                   struct Vec3 radar_velocity,
                                   struct Vec3 target_velocity,
                                   struct RadarMeasurement *out_measurement);

struct Bool space_mass_properties_two_body(double mass1,
                                           struct Vec3 position1,
                                           struct Vec3 inertia1_diag,
                                           double mass2,
                                           struct Vec3 position2,
                                           struct Vec3 inertia2_diag,
                                           struct MassProperties *out_properties);

double space_docking_buffer_energy(double relative_speed,
                                   double reduced_mass,
                                   double stroke,
                                   double efficiency);

struct Bool space_bang_off_bang_profile(double angle,
                                        double max_acceleration,
                                        double max_rate,
                                        struct BangOffBangProfile *out_profile);

struct Bool space_solar_radiation_pressure_acceleration(struct Vec3 sun_direction,
                                                        double solar_flux,
                                                        double reflectivity,
                                                        double area,
                                                        double mass,
                                                        struct Vec3 *out_acceleration);

struct Bool space_apply_solar_radiation_pressure_to_body(struct WorldHandle *world,
                                                         RigidBodyHandleRaw body_handle,
                                                         struct Vec3 sun_direction,
                                                         double solar_flux,
                                                         double reflectivity,
                                                         double area,
                                                         double mass,
                                                         struct Bool wake_up,
                                                         struct Vec3 *out_acceleration);

uint8_t space_apply_solar_radiation_pressure_to_body_flag(struct WorldHandle *world,
                                                          RigidBodyHandleRaw body_handle,
                                                          struct Vec3 sun_direction,
                                                          double solar_flux,
                                                          double reflectivity,
                                                          double area,
                                                          double mass,
                                                          struct Bool wake_up,
                                                          struct Vec3 *out_acceleration);

struct Bool space_gravity_gradient_torque(struct Vec3 position,
                                          struct Vec3 inertia_diag,
                                          double mu,
                                          struct Vec3 *out_torque);

struct Bool space_apply_gravity_gradient_torque_to_body(struct WorldHandle *world,
                                                        RigidBodyHandleRaw body_handle,
                                                        struct Vec3 inertia_diag,
                                                        double mu,
                                                        struct Bool wake_up,
                                                        struct Vec3 *out_torque);

uint8_t space_apply_gravity_gradient_torque_to_body_flag(struct WorldHandle *world,
                                                         RigidBodyHandleRaw body_handle,
                                                         struct Vec3 inertia_diag,
                                                         double mu,
                                                         struct Bool wake_up,
                                                         struct Vec3 *out_torque);

struct Bool space_magnetic_torquer_dipole(struct Vec3 commanded_torque,
                                          struct Vec3 magnetic_field,
                                          double max_dipole,
                                          struct Vec3 *out_dipole);

struct Bool space_apply_magnetic_torquer_to_body(struct WorldHandle *world,
                                                 RigidBodyHandleRaw body_handle,
                                                 struct Vec3 commanded_torque,
                                                 struct Vec3 magnetic_field,
                                                 double max_dipole,
                                                 struct Bool wake_up,
                                                 struct Vec3 *out_dipole);

uint8_t space_apply_magnetic_torquer_to_body_flag(struct WorldHandle *world,
                                                  RigidBodyHandleRaw body_handle,
                                                  struct Vec3 commanded_torque,
                                                  struct Vec3 magnetic_field,
                                                  double max_dipole,
                                                  struct Bool wake_up,
                                                  struct Vec3 *out_dipole);

struct Bool space_cmg_robust_pseudoinverse_diag(struct Vec3 jacobian_diag,
                                                struct Vec3 desired_torque,
                                                double damping,
                                                struct CmgRobustInverse *out_inverse);

struct Bool space_sgp4_j2_secular_rates(double semi_major_axis,
                                        double eccentricity,
                                        double inclination,
                                        double mean_motion,
                                        double equatorial_radius,
                                        double j2,
                                        struct Sgp4SecularRates *out_rates);

double space_docking_glideslope_command(double range,
                                        double desired_slope,
                                        double closing_speed_limit);

double space_sagnac_phase_rate(double area, double angular_rate, double wavelength);

double space_solar_array_pd_torque(double angle_error, double rate_error, double kp, double kd);

struct Bool space_sabatier_methane_rate(double co2_molar_rate,
                                        double h2_molar_rate,
                                        double conversion,
                                        struct ChemicalReactionRate *out_rate);

struct Bool space_spe_oxygen_rate(double current,
                                  double cells,
                                  double faraday_efficiency,
                                  struct ChemicalReactionRate *out_rate);

struct Bool space_radiator_power(double area,
                                 double emissivity,
                                 double temperature,
                                 double sink_temperature,
                                 double absorbed_power,
                                 struct RadiatorPower *out_power);

double space_whipple_critical_projectile_diameter(double bumper_thickness,
                                                  double bumper_density,
                                                  double projectile_density,
                                                  double impact_velocity,
                                                  double standoff);

double space_surface_charging_current_balance(double photo_current,
                                              double secondary_current,
                                              double backscatter_current,
                                              double electron_current,
                                              double ion_current);

struct Bool space_airlock_depressurization(double pressure,
                                           double ambient_pressure,
                                           double volume,
                                           double conductance,
                                           double dt,
                                           struct AirlockDepressurization *out_state);

/**
 * Compute the velocity induced by a straight vortex segment at a field point
 * using the Biot–Savart law.
 *
 * v = (κ / 4π) * ∫ (dℓ × r̂) / |r|²
 *
 * For a straight segment from s₁ to s₂, the induced velocity at point p is
 * evaluated using the analytical formula involving the solid angle.
 */
struct Bool sf_biot_savart_velocity(struct VortexSegment segment,
                                    struct Vec3 field_point,
                                    struct BiotSavartVelocity *out_velocity);

/**
 * Compute the self-induced velocity of a vortex ring.
 *
 * For a circular vortex ring of radius R, the self-induced velocity is:
 *
 *   v_ring = (κ / 4πR) * [ln(8R/ξ) - 1/2]
 *
 * where κ = h/m is the circulation quantum and ξ is the healing length.
 */
struct Bool sf_vortex_ring_velocity(struct VortexRing ring, struct Vec3 *out_velocity);

/**
 * Return the circulation quantum constant κ₀ = h/m for ⁴He.
 */
double sf_circulation_quantum(void);

/**
 * Compute the circulation around a closed loop by summing Biot–Savart
 * contributions along a set of segments forming the loop.
 *
 * `segments` — pointer to an array of `VortexSegment`.
 * `segment_count` — number of segments.
 * `sample_point` — a point on the loop where the velocity is integrated.
 */
struct Bool sf_circulation_around_loop(const struct VortexSegment *segments,
                                       uint32_t segment_count,
                                       struct Vec3 sample_point,
                                       struct QuantisedCirculation *out_circulation);

/**
 * Estimate the quantum number n = ∮v·dℓ / (h/m) given a velocity field
 * around a loop approximated by N tangent velocity samples.
 *
 * `tangent_velocities` — pointer to an array of tangential velocity
 * components (m/s) equally spaced around the loop.
 * `loop_radius` — radius of the circular loop (m).
 * `sample_count` — number of samples.
 */
struct Bool sf_quantum_number_estimate(const double *tangent_velocities,
                                       double loop_radius,
                                       uint32_t sample_count,
                                       int32_t *out_quantum);

/**
 * Evaluate the Gross–Pitaevskii order parameter ψ = √n · exp(iφ) at a point,
 * returning amplitude, phase, and density.
 *
 * For a generic vortex line passing through `vortex_center` with direction
 * `vortex_axis`, the phase wraps by 2π around the line.
 */
struct Bool sf_gp_order_parameter(double x,
                                  double y,
                                  double z,
                                  struct Vec3 vortex_center,
                                  struct Vec3 vortex_axis,
                                  int32_t circulation_quantum,
                                  double healing_length,
                                  double background_density,
                                  struct GpOrderParameter *out_param);

/**
 * Compute the Gross–Pitaevskii energy density (per unit volume) terms:
 *
 *   ε_kin  = (ħ² / 2m) |∇ψ|²
 *   ε_int  = (g / 2) |ψ|⁴
 *   ε_trap = V_trap |ψ|²
 *
 * Simplified: uses a Thomas–Fermi approximation with a harmonic trapping
 * potential V_trap = ½ m ω² r².
 */
struct Bool sf_gp_energy_density(double density,
                                 double trapping_frequency,
                                 double mass,
                                 double coupling_constant,
                                 double radius_from_center,
                                 struct GpEnergyDensity *out_energy);

/**
 * Time-evolve the Gross–Pitaevskii order parameter at a single spatial point
 * using imaginary-time propagation (simple relaxation to ground state).
 *
 * The homogeneous GP equation in imaginary time τ = i·t gives:
 *   ∂ψ/∂τ = -(1/ħ) · (g|ψ|² - μ) · ψ
 *
 * For the real amplitude a = |ψ|:
 *   ∂a/∂τ = -(1/ħ) · (g a² - μ) · a
 *
 * This converges to the equilibrium a = √(μ/g).
 */
struct Bool sf_gp_amplitude_evolution(double amplitude,
                                      double density,
                                      struct GpTimeEvolutionParams params,
                                      double *out_next_amplitude);

/**
 * Detect and perform a vortex reconnection between two line segments if
 * they are closer than `reconnection_distance`.
 *
 * Reconnection model:
 *   1. Find the closest points between the two segments.
 *   2. If the minimum distance < reconnection_distance, reconnect by
 *      swapping endpoints: s1_start ↔ s2_start and s1_end ↔ s2_end.
 *   3. Return the new segments and energy dissipation estimate.
 */
struct Bool sf_vortex_reconnection(struct VortexSegment seg1,
                                   struct VortexSegment seg2,
                                   double reconnection_distance,
                                   double healing_length,
                                   struct VortexReconnectionReport *out_report);

/**
 * Compute statistics for a vortex filament tangle (array of segments).
 *
 * `segments` — pointer to array of `VortexSegment`.
 * `segment_count` — number of segments.
 * `box_volume` — volume of the bounding box containing the tangle (for line density).
 */
struct Bool sf_vortex_tangle_stats(const struct VortexSegment *segments,
                                   uint32_t segment_count,
                                   double box_volume,
                                   struct VortexTangleStats *out_stats);

/**
 * Sample the GP order parameter on a 2D grid cross-section (for visualisation).
 *
 * The grid lies in the plane perpendicular to `plane_axis`, centered at
 * `plane_center`, with `nx` × `ny` points covering extents `extent_x` × `extent_y`.
 *
 * `out_grid` — pre-allocated buffer of `GpGridPoint` of length `nx * ny`.
 */
uint32_t sf_gp_grid_sample(struct Vec3 plane_center,
                           struct Vec3 plane_axis,
                           uint32_t nx,
                           uint32_t ny,
                           double extent_x,
                           double extent_y,
                           struct Vec3 vortex_center,
                           struct Vec3 vortex_axis,
                           int32_t circulation_quantum,
                           double healing_length,
                           double background_density,
                           struct GpGridPoint *out_grid,
                           uint32_t out_len);

/**
 * Compute the healing length ξ = ħ / √(2mgn) given the coupling constant
 * and background density.
 */
double sf_healing_length(double coupling_constant, double mass, double background_density);

/**
 * Compute the speed of sound c = √(gn/m) for a superfluid.
 */
double sf_sound_speed(double coupling_constant, double mass, double background_density);

/**
 * Return the helium mass constant.
 */
double sf_helium_mass(void);

/**
 * Return the scattering length for ⁴He.
 */
double sf_helium_scattering_length(void);

struct Bool thermal_fourier_conduction(double hot_temperature,
                                       double cold_temperature,
                                       double conductivity,
                                       double area,
                                       double thickness,
                                       struct HeatConductionReport *out_report);

struct Bool thermal_phase_change(double temperature,
                                 double phase_temperature,
                                 double mass,
                                 double specific_heat,
                                 double latent_heat,
                                 double heat_input,
                                 struct PhaseChangeReport *out_report);

struct Bool thermal_phase_condition(double temperature,
                                    double solidus_temperature,
                                    double liquidus_temperature,
                                    struct PhaseChangeReport *out_report);

struct Bool thermal_stefan_boltzmann_radiation(double temperature,
                                               double ambient_temperature,
                                               double emissivity,
                                               double area,
                                               struct ThermalRadiationReport *out_report);

struct Bool thermal_fem_diffusion_step(const struct FemHeatNode *nodes,
                                       uint32_t node_count,
                                       const struct FemHeatEdge *edges,
                                       uint32_t edge_count,
                                       double dt,
                                       double *out_temperatures,
                                       uint32_t capacity,
                                       struct FemHeatDiffusionReport *out_report);

struct Bool thermal_stress_from_expansion(struct MaterialProperties material,
                                          double strain,
                                          double delta_temperature,
                                          struct ThermalStressReport *out_report);

struct Bool thermal_thermoelastic_stress_strain(struct MaterialProperties material,
                                                double strain_x,
                                                double strain_y,
                                                double strain_z,
                                                double delta_temperature,
                                                struct ThermoelasticReport *out_report);

struct Bool topology_simp_material(double density,
                                   struct TopologyOptimizationParams params,
                                   struct SimpMaterialReport *out_report);

double topology_simp_stiffness(double density,
                               double penalization,
                               double stiffness_min,
                               double stiffness_solid);

double topology_compliance_sensitivity(double density,
                                       double element_energy,
                                       struct TopologyOptimizationParams params);

struct Bool topology_oc_update(const double *densities,
                               const double *sensitivities,
                               uint32_t cell_count,
                               struct TopologyOptimizationParams params,
                               double *out_densities,
                               uint32_t capacity,
                               struct TopologyOptimizationReport *out_report);

struct Bool topology_density_filter_2d(const double *densities,
                                       uint32_t width,
                                       uint32_t height,
                                       double filter_radius,
                                       double *out_densities,
                                       uint32_t capacity);

struct Bool topology_density_to_voxels(const double *densities,
                                       uint32_t cell_count,
                                       double threshold,
                                       uint8_t *out_voxels,
                                       uint32_t capacity,
                                       struct DensityFieldStats *out_stats);

struct Bool topology_runtime_shape_density_step(const double *densities,
                                                const double *element_energies,
                                                uint32_t cell_count,
                                                struct TopologyOptimizationParams params,
                                                double *out_densities,
                                                uint32_t capacity,
                                                struct TopologyOptimizationReport *out_report);

struct Bool trajectory_estimate_forces(struct TrajectoryState state,
                                       struct TrajectoryEnvironment env,
                                       struct TrajectoryForceReport *out_report);

struct Bool trajectory_integrate_step(struct TrajectoryState state,
                                      struct TrajectoryEnvironment env,
                                      double dt,
                                      struct TrajectoryState *out_state,
                                      struct TrajectoryForceReport *out_report);

struct Bool trajectory_apply_forces_to_body(struct WorldHandle *world,
                                            RigidBodyHandleRaw body_handle,
                                            struct TrajectoryEnvironment env,
                                            struct Bool wake_up,
                                            struct TrajectoryForceReport *out_report);

uint8_t trajectory_apply_forces_to_body_flag(struct WorldHandle *world,
                                             RigidBodyHandleRaw body_handle,
                                             struct TrajectoryEnvironment env,
                                             struct Bool wake_up,
                                             struct TrajectoryForceReport *out_report);

struct Bool trajectory_glide_estimate(struct TrajectoryGlideState state,
                                      struct TrajectoryGlideEnvironment env,
                                      struct TrajectoryGlideReport *out_report);

struct Bool trajectory_glide_integrate_step(struct TrajectoryGlideState state,
                                            struct TrajectoryGlideEnvironment env,
                                            double dt,
                                            struct TrajectoryGlideState *out_state,
                                            struct TrajectoryGlideReport *out_report);

struct Bool transmission_gear_evaluate(double driver_angle,
                                       double driven_angle,
                                       double driver_angular_velocity,
                                       double driven_angular_velocity,
                                       struct GearConstraintDesc desc,
                                       struct GearConstraintReport *out_report);

double transmission_gear_target_angle(double driver_angle,
                                      double ratio,
                                      struct Bool opposite_direction,
                                      double phase);

struct Bool transmission_screw_evaluate(double screw_angle,
                                        double nut_translation,
                                        double screw_angular_velocity,
                                        double nut_linear_velocity,
                                        struct ScrewConstraintDesc desc,
                                        struct ScrewConstraintReport *out_report);

double transmission_screw_target_translation(double screw_angle,
                                             double lead,
                                             struct Bool right_handed,
                                             double phase);

struct Bool transmission_cycloidal_cam_evaluate(double cam_angle,
                                                double follower_displacement,
                                                double cam_angular_velocity,
                                                struct CamConstraintDesc desc,
                                                struct CamConstraintReport *out_report);

struct Bool transmission_archimedean_spiral_evaluate(double angle,
                                                     double radial_position,
                                                     double angular_velocity,
                                                     struct SpiralConstraintDesc desc,
                                                     struct SpiralConstraintReport *out_report);

double transmission_archimedean_spiral_radius(double angle,
                                              double initial_radius,
                                              double radial_pitch,
                                              double phase);

struct ColliderBuilderHandle *collider_builder_create_voxels(const uint8_t *voxels,
                                                             uint32_t size_x,
                                                             uint32_t size_y,
                                                             uint32_t size_z,
                                                             double voxel_size,
                                                             struct Vec3 origin,
                                                             struct VoxelColliderOptions options);

struct ColliderBuilderHandle *collider_builder_create_voxels_auto(const uint8_t *voxels,
                                                                  uint32_t size_x,
                                                                  uint32_t size_y,
                                                                  uint32_t size_z,
                                                                  double voxel_size,
                                                                  struct Vec3 origin,
                                                                  struct Bool dynamic_body);

struct VoxelBuildStats voxel_build_stats(const uint8_t *voxels,
                                         uint32_t size_x,
                                         uint32_t size_y,
                                         uint32_t size_z,
                                         double voxel_size,
                                         struct Vec3 origin,
                                         struct VoxelColliderOptions options);

struct VoxelBuildStats voxel_aabb_build_stats(struct AabbDesc aabb,
                                              double voxel_size,
                                              struct VoxelColliderOptions options);

struct VoxelBuildStats voxel_obb_build_stats(struct Obb obb,
                                             double voxel_size,
                                             struct VoxelColliderOptions options);

void voxel_aabb_build_stats_out(struct AabbDesc aabb,
                                double voxel_size,
                                struct VoxelColliderOptions options,
                                struct VoxelBuildStats *out_stats);

void voxel_obb_build_stats_out(struct Obb obb,
                               double voxel_size,
                               struct VoxelColliderOptions options,
                               struct VoxelBuildStats *out_stats);

struct ColliderBuilderHandle *collider_builder_create_voxel_aabb(struct AabbDesc aabb,
                                                                 double voxel_size,
                                                                 struct VoxelColliderOptions options);

struct ColliderBuilderHandle *collider_builder_create_voxel_aabb_auto(struct AabbDesc aabb,
                                                                      double voxel_size,
                                                                      struct Bool dynamic_body);

struct ColliderBuilderHandle *collider_builder_create_voxel_obb(struct Obb obb,
                                                                double voxel_size,
                                                                struct VoxelColliderOptions options);

struct ColliderBuilderHandle *collider_builder_create_voxel_obb_auto(struct Obb obb,
                                                                     double voxel_size,
                                                                     struct Bool dynamic_body);

uint32_t query_intersect_voxel_aabb(const struct WorldHandle *world,
                                    struct AabbDesc aabb,
                                    struct QueryFilterDesc filter,
                                    ColliderHandleRaw *out_handles,
                                    uint32_t capacity);

uint32_t query_intersect_voxel_aabb_count(const struct WorldHandle *world,
                                          struct AabbDesc aabb,
                                          struct QueryFilterDesc filter);

uint32_t query_intersect_voxel_obb(const struct WorldHandle *world,
                                   struct Obb obb,
                                   struct QueryFilterDesc filter,
                                   ColliderHandleRaw *out_handles,
                                   uint32_t capacity);

uint32_t query_intersect_voxel_obb_count(const struct WorldHandle *world,
                                         struct Obb obb,
                                         struct QueryFilterDesc filter);

RigidBodyHandleRaw world_insert_static_voxel_aabb(struct WorldHandle *world,
                                                  struct AabbDesc aabb,
                                                  double voxel_size,
                                                  struct VoxelColliderOptions options,
                                                  double friction,
                                                  double restitution);

RigidBodyHandleRaw world_insert_dynamic_voxel_obb(struct WorldHandle *world,
                                                  struct Obb obb,
                                                  double voxel_size,
                                                  struct VoxelColliderOptions options,
                                                  double density,
                                                  double friction,
                                                  double restitution);

/**
 * Compute wavenumber from wavelength: k = 2π / λ.
 */
double wo_wavenumber(double wavelength);

/**
 * Compute wavelength from wavenumber: λ = 2π / k.
 */
double wo_wavelength(double wavenumber);

/**
 * Compute the complex amplitude of a plane wave at position (x, y, z):
 *   E = A₀ · exp(i (k·r − ωt))
 * where k = (kx, ky, kz) and ωt is a global time phase offset.
 *
 * For a wave propagating along the z-axis: E = A₀ · exp(i (k·z − φ₀))
 */
struct Bool wo_plane_wave(struct PlaneWaveParams params,
                          double x,
                          double y,
                          double z,
                          double kx,
                          double ky,
                          double kz,
                          struct ComplexAmplitude *out_amplitude);

/**
 * Compute the complex amplitude of a spherical wave at an observation point.
 *
 *   E = A₀ · exp(i k r) / r
 *
 * where r is the distance from the source to the observation point.
 */
struct Bool wo_spherical_wave(double source_x,
                              double source_y,
                              double source_z,
                              double obs_x,
                              double obs_y,
                              double obs_z,
                              double wavenumber,
                              double amplitude,
                              struct SphericalWavePoint *out_wave);

/**
 * Compute the field at an observation point from N point sources
 * using the Huygens–Fresnel superposition integral.
 *
 *   E(P) = Σ_j A_j · exp(i k r_j) / r_j
 *
 * where r_j is the distance from source j to the observation point.
 */
struct Bool wo_huygens_fresnel(const struct PointSource *sources,
                               uint32_t source_count,
                               double obs_x,
                               double obs_y,
                               double obs_z,
                               double wavenumber,
                               struct ComplexAmplitude *out_amplitude);

/**
 * Compute the Fresnel diffraction field at a single observation point from a
 * rectangular aperture, using the Fresnel (paraxial) approximation.
 *
 * The Fresnel diffraction integral for a rectangular aperture:
 *
 *   E(x, y) ∝ ∫∫ A(ξ, η) · exp( i k / (2z) · [(x-ξ)² + (y-η)²] ) dξ dη
 *
 * This simplified version assumes uniform illumination (A = 1) over the
 * aperture and performs a numerical Riemann sum over `samples_x × samples_y`
 * sub-divisions of the aperture.
 */
struct Bool wo_fresnel_diffraction_point(struct ApertureDesc aperture,
                                         double obs_x,
                                         double obs_y,
                                         double obs_z,
                                         double wavenumber,
                                         uint32_t samples_x,
                                         uint32_t samples_y,
                                         struct DiffractionPoint *out_point);

/**
 * Compute the Fresnel–Kirchhoff diffraction integral at a single observation
 * point, including the obliquity (inclination) factor.
 *
 *   E(P) = (1 / iλ) ∫∫ A(ξ,η) · exp(i k r) / r · cosθ dξ dη
 *
 * where cosθ = z/r is the obliquity factor for normal incidence.
 */
struct Bool wo_kirchhoff_diffraction_point(struct ApertureDesc aperture,
                                           double obs_x,
                                           double obs_y,
                                           double obs_z,
                                           double wavenumber,
                                           uint32_t samples_x,
                                           uint32_t samples_y,
                                           struct KirchhoffDiffractionPoint *out_point);

/**
 * Compute the interference pattern from Young's double-slit experiment at a
 * single observation point on a distant screen.
 *
 * Slits are at (±d/2, 0) in the aperture plane, screen at distance D.
 * Single-slit envelope (width a) is included.
 *
 * Returns the normalised intensity:
 *   I = I₀ · cos²(π d x / λ D) · sinc²(π a x / λ D)
 */
struct Bool wo_young_slit_point(double slit_separation,
                                double slit_width,
                                double screen_distance,
                                double wavelength,
                                double obs_x,
                                double obs_y,
                                struct YoungSlitPoint *out_point);

/**
 * Compute the Young's interference pattern across a 1D array of points
 * (along the x-axis) and write intensities into a pre-allocated buffer.
 */
uint32_t wo_young_slit_pattern(double slit_separation,
                               double slit_width,
                               double screen_distance,
                               double wavelength,
                               double x_min,
                               double x_max,
                               uint32_t num_points,
                               double *out_intensities,
                               uint32_t out_len);

/**
 * Compute thin-film interference for a single layer.
 *
 * Optical path difference (normal incidence): OPD = 2 n_film t cos θ_t
 * where θ_t is the transmission angle (from Snell's law).
 *
 * Phase difference: δ = (2π/λ) · OPD + π (if half-wave loss occurs)
 *
 * Half-wave loss occurs when n_film > n_incident or n_film > n_substrate
 * (reflection off a higher-index medium).
 *
 * Interference intensity: I = I₀ · [1 + cos(δ)] / 2  (simplified)
 */
struct Bool wo_thin_film_interference(struct ThinFilmParams params,
                                      double wavelength,
                                      struct ThinFilmInterferenceReport *out_report);

/**
 * Compute thin-film interference for multiple wavelengths (rainbow spectrum).
 *
 * `wavelengths` — pointer to array of wavelengths (m).
 * `intensities_out` — pre-allocated buffer for output intensities.
 * `count` — number of wavelengths.
 *
 * Returns the number of intensities written.
 */
uint32_t wo_thin_film_spectrum(struct ThinFilmParams params,
                               const double *wavelengths,
                               double *intensities_out,
                               uint32_t count);

/**
 * Compute the radius of the n-th Fresnel zone for a point at distance D
 * from the aperture plane and wavelength λ.
 *
 *   r_n = √(n λ D)
 *
 * Also determines whether the zone contributes constructively.
 */
struct Bool wo_fresnel_zone(uint32_t zone_index,
                            double distance,
                            double wavelength,
                            struct FresnelZoneReport *out_zone);

/**
 * Compute the sum of contributions from the first N Fresnel zones
 * (simplified phasor sum).
 *
 * `num_zones` — number of zones to sum.
 * `out_intensity` — normalised intensity after summing N zones.
 */
struct Bool wo_fresnel_zone_sum(uint32_t num_zones,
                                double distance,
                                double wavelength,
                                double *out_intensity);

/**
 * Sample the Fresnel diffraction pattern on a regular 2D grid in the
 * observation plane.
 *
 * `nx` × `ny` points spanning `extent_x` × `extent_y` around the optical axis.
 * Results are written into `out_grid` (array of `DiffractionPoint`, capacity `out_len`).
 *
 * Returns the number of points written.
 */
uint32_t wo_fresnel_grid(struct ApertureDesc aperture,
                         double screen_distance,
                         double wavenumber,
                         uint32_t nx,
                         uint32_t ny,
                         double extent_x,
                         double extent_y,
                         uint32_t samples_x,
                         uint32_t samples_y,
                         struct DiffractionPoint *out_grid,
                         uint32_t out_len);

struct WorldHandle *world_create(struct Vec3 gravity);

void world_destroy(struct WorldHandle *world);

void world_step(struct WorldHandle *world, double delta_seconds);

struct Bool world_set_integration_parameters(struct WorldHandle *world,
                                             double dt,
                                             uint32_t solver_iterations,
                                             uint32_t ccd_substeps);

uint32_t world_get_integration_parameters(const struct WorldHandle *world,
                                          double *out_values,
                                          uint32_t capacity);

void world_set_gravity(struct WorldHandle *world, struct Vec3 gravity);

struct Vec3 world_get_gravity(const struct WorldHandle *world);

int32_t world_get_rigid_body_set_size(const struct WorldHandle *world);

int32_t world_get_collider_set_size(const struct WorldHandle *world);

void world_get_gravity_out(const struct WorldHandle *world, struct Vec3 *out_gravity);

uint32_t world_dynamic_body_snapshot_count(const struct WorldHandle *world);

uint32_t world_dynamic_body_snapshot(const struct WorldHandle *world,
                                     RigidBodyHandleRaw *out_handles,
                                     double *out_values,
                                     uint32_t capacity);

uint32_t world_body_snapshot_count(const struct WorldHandle *world);

uint32_t world_body_snapshot(const struct WorldHandle *world,
                             RigidBodyHandleRaw *out_handles,
                             double *out_values,
                             uint32_t capacity);

uint32_t world_update_body_poses(struct WorldHandle *world,
                                 const RigidBodyHandleRaw *handles,
                                 const double *values,
                                 uint32_t count,
                                 struct Bool wake_up);

uint32_t world_update_body_velocities(struct WorldHandle *world,
                                      const RigidBodyHandleRaw *handles,
                                      const double *values,
                                      uint32_t count,
                                      struct Bool wake_up);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* RIGID_BODY_H */
