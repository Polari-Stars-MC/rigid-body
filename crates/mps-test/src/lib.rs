// mps-test - extracted integration tests for mps-core physics engine
// Each module mirrors a rapier submodule from mps-core

pub mod cosmos {
    pub mod arena;
    pub mod bodies;
    pub mod celestial_rotation;
    pub mod central_body_switch;
    pub mod convergence;
    pub mod cross_world;
    pub mod drag_terminal_velocity;
    pub mod drift_matrix;
    pub mod error;
    pub mod ffi;
    pub mod flight;
    pub mod gravity;
    pub mod hohmann_transfer_validation;
    pub mod integrator;
    pub mod long_orbit;
    pub mod orbit;
    pub mod orbit_diagnostics;
    pub mod perturbation;
    pub mod radio;
    pub mod world;
}

pub mod rapier {
    pub mod acoustics;
    pub mod acoustics_ffi;
    pub mod aerodynamics;
    pub mod anvilkit;
    pub mod articulation;
    pub mod astrocalc;
    pub mod astrophysics;
    pub mod balloon;
    pub mod batch;
    pub mod biomechanics;
    pub mod body_spatial_index;
    pub mod bounds;
    pub mod bridge;
    pub mod celestial_data;
    pub mod chaos;
    pub mod character_body;
    pub mod cloth;
    pub mod collider;
    pub mod collision_mode;
    pub mod compat;
    pub mod continuum;
    pub mod control_theory;
    pub mod controller;
    pub mod cosmology;
    pub mod crbtree;
    pub mod cross_validate;
    pub mod debug_recording;
    pub mod disasters;
    pub mod disciplines;
    pub mod domain;
    pub mod dop;
    pub mod electromagnetism;
    pub mod emag;
    pub mod error;
    pub mod events;
    pub mod ffi;
    pub mod fluid;
    pub mod fluid_sph;
    pub mod force_queue;
    pub mod force_queue_integration;
    pub mod forces;
    pub mod fracture;
    pub mod fracture_mesh;
    // 真实物理验证系列：摩擦/恢复系数能量学（库仑锥 + 反弹速度比 + 静摩擦保持）
    pub mod friction_restitution_energy;
    pub mod galactic_dynamics;
    pub mod granular;
    // 真实物理验证系列：颗粒休止角（摩擦单调性 + 对称/守恒）
    pub mod granular_repose;
    pub mod gravitational_models;
    pub mod hair;
    pub mod heliophysics;
    pub mod high_energy_astro;
    pub mod integrators;
    pub mod interaction;
    pub mod joints;
    pub mod material_mechanics;
    pub mod math;
    pub mod matmech;
    pub mod molecular;
    pub mod neural;
    pub mod nuclear;
    pub mod nucphys;
    pub mod parallel;
    pub mod performance_matrix;
    pub mod physchem;
    pub mod planetary_science;
    pub mod plasma;
    pub mod plasma_ffi;
    pub mod qphys;
    pub mod quantum;
    pub mod query;
    pub mod record_replay;
    pub mod registry;
    pub mod rel;
    pub mod relativity;
    pub mod rigid_body;
    pub mod rope;
    pub mod rope_knot;
    pub mod rotor;
    pub mod rtree;
    pub mod scientists;
    pub mod sensor;
    pub mod servo_body;
    pub mod shared_arena;
    pub mod sleep_wake;
    pub mod soft_body;
    pub mod soft_data;
    pub mod soft_upstream;
    pub mod softbody;
    pub mod spaceflight;
    pub mod stellar;
    pub mod superfluidity;
    pub mod terrain_gravity;
    pub mod thermo;
    pub mod thermodynamics;
    pub mod tire_model;
    pub mod topology;
    pub mod trajectory;
    pub mod transmission;
    pub mod vehicle;
    pub mod volcano;
    pub mod voronoi;
    pub mod voxel;
    pub mod wave_optics;
    pub mod world;

    // CI守门测试：跨 crate 模块镜像对齐 (OPTIMIZATION.md §8)。
    pub mod verify_module_mirror;
    // ABI 锁定测试：pin shared_arena constants (OPTIMIZATION.md §10)。
    pub mod arena_compat;
    // 跨 crate 错误码一致性 (OPTIMIZATION.md §1 可选加固)。
    pub mod error_consistency;
    // 跨 crate 版本锁定 (ARENA_VERSION ↔ ABI_VERSION, OPTIMIZATION.md §N6)。
    pub mod version_consistency;
    // mps-web metrics.rs ↔ source counts 同步 (OPTIMIZATION.md §N3)。
    pub mod verify_metrics_sync;
}

// mps-ffm 探针库契约守门（公式层 JNI/FFM 兼容性）——不属于 rapier 镜像树。
pub mod ffm_probe;

// 每个 extern "C" 必须套 ffi_guard/ffi_scalar（AGENTS.md 硬性规定）——
// 跨整个 rapier 树 + cosmos ffi 的横向审计，不属于镜像树。
pub mod verify_ffi_guard;
