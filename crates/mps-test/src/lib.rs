// mps-test - extracted integration tests for mps-core physics engine
// Each module mirrors a rapier submodule from mps-core

pub mod rapier {
    pub mod acoustics;
    pub mod aerodynamics;
    pub mod anvilkit;
    pub mod astrophysics;
    pub mod biomechanics;
    pub mod bounds;
    pub mod bridge;
    pub mod celestial_data;
    pub mod chaos;
    pub mod collider;
    pub mod continuum;
    pub mod control_theory;
    pub mod controller;
    pub mod crbtree;
    pub mod dop;
    pub mod electromagnetism;
    pub mod events;
    pub mod ffi;
    pub mod fluid;
    pub mod forces;
    pub mod fracture;
    pub mod gravitational_models;
    pub mod integrators;
    pub mod interaction;
    pub mod joints;
    pub mod math;
    pub mod molecular;
    pub mod neural;
    pub mod physchem;
    pub mod plasma;
    pub mod quantum;
    pub mod query;
    pub mod relativity;
    pub mod rigid_body;
    pub mod rtree;
    pub mod shared_arena;
    pub mod softbody;
    pub mod spaceflight;
    pub mod superfluidity;
    pub mod terrain_gravity;
    pub mod thermodynamics;
    pub mod topology;
    pub mod trajectory;
    pub mod transmission;
    pub mod voxel;
    pub mod wave_optics;
    pub mod world;
}