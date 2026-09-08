//! 旧域模块 —— 公式已迁移到 `crate::scientists::{leonhard_euler, daniel_bernoulli,
//! george_stokes, claude_louis_navier}::formulas::*` 中。本文件保留为 re-export 兼容层，
//! 现有调用路径 `mps_formula::fluid::<fn>` 解析到对应科学家公式定义。

pub use crate::scientists::daniel_bernoulli::formulas::bernoulli_pressure;
pub use crate::scientists::daniel_bernoulli::formulas::bernoulli_report;
pub use crate::scientists::daniel_bernoulli::formulas::darcy_friction_factor;
pub use crate::scientists::daniel_bernoulli::formulas::flow_regime;
pub use crate::scientists::daniel_bernoulli::formulas::re_n;
pub use crate::scientists::daniel_bernoulli::formulas::{
    bernoulli_pressure_checked, bernoulli_report_checked,
};
pub use crate::scientists::george_stokes::formulas::area_mach_ratio;
pub use crate::scientists::george_stokes::formulas::bingham_stress;
pub use crate::scientists::george_stokes::formulas::doublet_stream_function_2d;
pub use crate::scientists::george_stokes::formulas::epsilon_equation_source;
pub use crate::scientists::george_stokes::formulas::isentropic_density_ratio;
pub use crate::scientists::george_stokes::formulas::isentropic_pressure_ratio;
pub use crate::scientists::george_stokes::formulas::isentropic_temperature_ratio;
pub use crate::scientists::george_stokes::formulas::k_epsilon_constants;
pub use crate::scientists::george_stokes::formulas::k_epsilon_eddy_viscosity;
pub use crate::scientists::george_stokes::formulas::k_epsilon_production;
pub use crate::scientists::george_stokes::formulas::k_equation_source;
pub use crate::scientists::george_stokes::formulas::normal_shock_density_ratio;
pub use crate::scientists::george_stokes::formulas::normal_shock_downstream_mach;
pub use crate::scientists::george_stokes::formulas::normal_shock_pressure_ratio;
pub use crate::scientists::george_stokes::formulas::power_law_viscosity;
pub use crate::scientists::george_stokes::formulas::prandtl_meyer_angle;
pub use crate::scientists::george_stokes::formulas::source_potential_2d;
pub use crate::scientists::george_stokes::formulas::turbulent_length_scale;
pub use crate::scientists::george_stokes::formulas::turbulent_reynolds;
pub use crate::scientists::leonhard_euler::formulas::blasius_displacement_thickness;
pub use crate::scientists::leonhard_euler::formulas::blasius_momentum_thickness;
pub use crate::scientists::leonhard_euler::formulas::blasius_thickness;
pub use crate::scientists::leonhard_euler::formulas::compute_fluid_forces;
pub use crate::scientists::leonhard_euler::formulas::laminar_skin_friction;
pub use crate::scientists::leonhard_euler::formulas::navier_stokes_simplified_step;
pub use crate::scientists::leonhard_euler::formulas::sph_estimate_density;
pub use crate::scientists::leonhard_euler::formulas::sph_estimate_forces;
pub use crate::scientists::leonhard_euler::formulas::sph_poly6_kernel;
pub use crate::scientists::leonhard_euler::formulas::sph_spiky_gradient;
pub use crate::scientists::leonhard_euler::formulas::sph_viscosity_laplacian;
pub use crate::scientists::leonhard_euler::formulas::turbulent_skin_friction;
