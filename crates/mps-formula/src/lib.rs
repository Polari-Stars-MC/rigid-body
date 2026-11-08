// Every C ABI entry point validates raw pointers at the boundary (length/null
// checks via `crate::ffi`) and wraps its whole body in `error::ffi_guard`
// (catch_unwind → `ERR_INTERNAL` + the entry point's failure sentinel), so
// the safe-fn-raw-pointer lint is noise here — same pattern as mps-core.
// This crate cannot depend on mps-core/mps-cosmos, so the guard lives locally
// in `error.rs`, next to the shared thread-local error slot.
#![allow(clippy::not_unsafe_ptr_arg_deref)]

pub mod acoustics;
pub mod astrophysics;
pub mod biomechanics;
pub mod celestial_data;
pub mod chaos;
pub mod continuum;
pub mod control_theory;
pub mod cosmology;
pub mod disasters;
pub mod disciplines;
pub mod domain;
pub mod electromagnetism;
pub mod error;
pub use domain::FormulaError;
pub mod ffi;
pub mod fluid;
pub mod galactic_dynamics;
pub mod gravitational_models;
pub mod heliophysics;
pub mod high_energy_astro;
pub mod integrators;
pub mod material_mechanics;
pub mod math;
pub mod molecular;
pub mod nuclear;
pub mod physchem;
pub mod planetary_science;
pub mod plasma;
pub mod quantum;
pub mod relativity;
pub mod rotor;
pub mod scientists;
pub mod softbody;
pub mod spaceflight;
pub mod stellar;
pub mod superfluidity;
pub mod thermodynamics;
pub mod topology;
pub mod trajectory;
pub mod transmission;
pub mod volcano;
pub mod voronoi;
pub mod wave_optics;
