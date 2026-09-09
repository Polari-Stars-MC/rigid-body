// mps-test - extracted integration tests for mps-core physics engine
// Each module mirrors a rapier submodule from mps-core

pub mod cosmos {
    pub mod arena;
    pub mod bodies;
    pub mod ffi;
    pub mod gravity;
    pub mod integrator;
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
    pub mod bounds;
    pub mod bridge;
    pub mod celestial_data;
    pub mod chaos;
    pub mod character_body;
    pub mod cloth;
    pub mod collider;
    pub mod collision_mode;
    pub mod continuum;
    pub mod control_theory;
    pub mod controller;
    pub mod cosmology;
    pub mod crbtree;
    pub mod cross_validate;
    pub mod disasters;
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
    pub mod galactic_dynamics;
    pub mod granular;
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
    pub mod physchem;
    pub mod planetary_science;
    pub mod plasma;
    pub mod plasma_ffi;
    pub mod qphys;
    pub mod quantum;
    pub mod query;
    pub mod registry;
    pub mod rel;
    pub mod relativity;
    pub mod rigid_body;
    pub mod rope;
    pub mod rope_knot;
    pub mod rtree;
    pub mod sensor;
    pub mod servo_body;
    pub mod shared_arena;
    pub mod soft_body;
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
