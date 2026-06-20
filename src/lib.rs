#![allow(clippy::missing_safety_doc)]

use mimalloc::MiMalloc;

#[global_allocator]
static GLOBAL_ALLOCATOR: MiMalloc = MiMalloc;

mod abi;
mod helper;
mod rapier;

pub use rapier::ffi::{
    AabbDesc, AcousticContactDesc, AcousticExcitationReport, AcousticMaterial,
    AcousticResonanceReport, AcousticWaveReport, AeroForceReport, AeroSurface, AirDragLaw,
    AirlockDepressurization, AtomicOxygenErosion, BangOffBangProfile,
    BatteryEquivalentCircuit, BernoulliReport, BodyStatus, Bool, CRbTreeHandle, CamConstraintDesc,
    CamConstraintReport, Capsule, CatalystEffect, CatalystReport, CharacterCollision,
    CharacterControllerHandle, ChemicalReactionRate, CmgExchange, CmgRobustInverse, Co2MassBalance,
    ColliderBuilderHandle, ColliderHandleRaw, CollisionEventRecord, CollisionProbability,
    ConcentrationBuoyancyReport, ContactForceEventRecord, ContactForceModel, CoulombFrictionLaw,
    CustomPhysicsReport, CwDerivative, CwState, Cylinder, DensityFieldStats, DhTransform,
    EffectiveCharacterMovement, ElectromagneticField, Ellipsoid, ExternalForceLaw,
    FaradayInductionReport, FdtdYeeReport, FemConstitutiveReport, FemHeatDiffusionReport,
    FemHeatEdge, FemHeatNode, FemShapeFunctionReport, FemTetrahedron, FlexibleModeDerivative,
    FluidForceReport, FluidLoopHeatTransfer, FluidVolume, FractureEnergyReport,
    FractureFragmentDesc, FractureMaterial, FractureModeReport, FractureReplaceReport, FriisLink,
    GearConstraintDesc, GearConstraintReport, GnssObservation, GrayScottParams,
    GrayScottReactionReport, GriffithReport, HallThrusterPerformance, HeatConductionReport,
    HertzContactReport, HillMuscleDesc, HillMuscleReport, HillMuscleState, HohmannTransfer,
    ImpulseJointHandleRaw, InteractionGroupsDesc, JointAxisDesc, JointBuilderHandle, JointTypeDesc,
    KdopPreset, LeastSquaresAttitude, LorentzForceReport, MagneticFluxReport, ManipulatorDynamics,
    MassProperties, MaterialProperties, MaxwellPointReport, MinerDamageReport, ModalAnalysisReport,
    ModalSynthesisReport, MolecularForceLaw, MolecularPairReport, MolecularParticle, MpcConfig,
    MpcReport, NBodyForceReport, NBodyParticle, NBodySolverParams, NavierStokesReport,
    NeuralActivation, NeuralBoundsDesc, NewmarkBetaParameters, NewmarkBetaReport, Obb,
    OrbitalElements, OrbitalResonanceReport, PhaseChangeReport, PidGains, PidReport, PidState,
    PointProjection, Prism, QuantumBarrier, QuantumOscillatorReport, QuantumTunnelingReport,
    QuantumWaveFunction, Quat, QuaternionDerivative, QueryFilterDesc, RTreeHandle,
    RadarMeasurement, RadiatorPower, RayHit, ReactionDiffusionReport, RelativisticOrbitReport,
    RigidBodyBuilderHandle, RigidBodyEulerDerivative, RigidBodyHandleRaw, RocheLimitReport,
    ScalarKalman, ScrewConstraintDesc, ScrewConstraintReport, Sgp4SecularRates, ShapeCastHit,
    ShapeCastOptionsDesc, ShapeDesc, ShapeType, SimpMaterialReport, SkeletalConstraintReport,
    SkeletalJointLimit, SloshPendulumDerivative, SnCurveReport, SoftBendingConstraint,
    SoftBodyStepReport, SoftDistanceConstraint, SoftSphereCollision, SoftSpring,
    SoftVolumeConstraint, SolarPanelPower, SpatializedSample, SphForceReport, SphParticle, Sphere,
    SphericalShell, SpiralConstraintDesc, SpiralConstraintReport, Ssv, StateSpaceReport,
    StateVector, StressIntensityReport, StressStrainReport, StructuralModeReport, ThermalBalance,
    ThermalRadiationReport, ThermalStressReport, ThermoelasticReport, TopologyOptimizationParams,
    TopologyOptimizationReport, TrajectoryEnvironment, TrajectoryForceReport,
    TrajectoryGlideEnvironment, TrajectoryGlideReport, TrajectoryGlideState, TrajectoryState,
    VariationalState, Vec3, VoxelBuildStats, VoxelColliderMode, VoxelColliderOptions, WorldHandle,
};

#[cfg(feature = "anvilkit-bridge")]
pub use rapier::ffi::AnvilKitAppHandle;
