//! 零引用 C ABI 的**有限性扫描**（ROADMAP「还开着」第 6 条）。
//!
//! ## 为什么需要这个文件
//!
//! `docs/ROADMAP.md` 第 6 条写得很直白：
//!
//! > **正确性没有下界。** 测试全绿只证明「没崩、没变」，不证明「算对了」……
//! > 缺的是解析解对照、守恒律断言，以及**既无 JNI 绑定也无测试引用**的入口的
//! > 基本有限性检查。
//!
//! 能力地图统计出 **143 个入口**既无 JNI 绑定也无测试引用。这份文件对其中
//! **86 个**（签名形状严格为 `Bool f(double…, double *out)`）做机械化扫描。
//!
//! ## 检查什么
//!
//! 对每个入口喂 8 组输入向量（正常值、跨 6 个数量级、零、负数、`NaN`、
//! `INFINITY`），断言：
//!
//! 1. **不 panic** —— `ffi_guard` 若失效会在这里炸
//! 2. **返回 `TRUE` 时输出必须有限** —— 这是核心。`NaN`/`Inf` 输入若被静默
//!    接受并产出非有限值，说明该入口缺少输入校验
//!
//! 返回 `FALSE` 是**合法**结果（输入被拒绝），不额外断言 —— 各入口的合法域不同，
//! 统一输入不可能全部命中。
//!
//! ## 这不是「算对了」的证明
//!
//! 有限性只是**下界中的下界**。它抓不到量纲错误、公式写反、系数漏乘。
//! 那些需要解析解对照（方法见 ADR-0025~0033 那一批）。
//!
//! ## 剩下的 57 个入口
//!
//! 需要 `WorldHandle`、`int32_t`/`struct Bool` 参数、或多个输出指针的入口签名
//! 不统一，无法表驱动。其中 `tyndall_*` 已有场景级测试；其余仍无覆盖，
//! 清单见 `docs/CAPABILITY_MAP.md`。

use mps_core::rapier::ffi::Bool;

type F0 = extern "C" fn(*mut f64) -> Bool;
type F1 = extern "C" fn(f64, *mut f64) -> Bool;
type F2 = extern "C" fn(f64, f64, *mut f64) -> Bool;
type F3 = extern "C" fn(f64, f64, f64, *mut f64) -> Bool;
type F4 = extern "C" fn(f64, f64, f64, f64, *mut f64) -> Bool;
type F5 = extern "C" fn(f64, f64, f64, f64, f64, *mut f64) -> Bool;

/// 输入向量池。前四组是「物理上可能合理」的量级，后四组是边界与非法值。
///
/// 只取前 `arity` 个分量，所以短的入口复用同一张表。
const VECTORS: &[[f64; 6]] = &[
    // 典型物理量级
    [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    // 一阶/二阶混合
    [0.5, 2.0, 1.0, 3.0, 1.0, 1.0],
    // 跨 6 个数量级
    [1e-3, 1e3, 1.0, 1e6, 0.5, 2.0],
    // 天文/微观量级
    [1.989e30, 6.96e8, 3.0e8, 1.5e11, 5.97e24, 6.37e6],
    // 全零（通常是非法域）
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    // 全负（通常是非法域）
    [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0],
    // NaN 输入
    [f64::NAN, 1.0, 1.0, 1.0, 1.0, 1.0],
    // 无穷输入
    [f64::INFINITY, 1.0, 1.0, 1.0, 1.0, 1.0],
];

/// 收集一条违规（不 panic，让一次运行报出全部问题）。
fn record(violations: &mut Vec<String>, name: &str, ok: Bool, out: f64, args: &[f64]) {
    if ok == Bool::TRUE && !out.is_finite() {
        violations.push(format!("{name} 返回 TRUE 但输出 {out}，输入 {args:?}"));
    }
}

/// 六个 arity 的扫描器共用的收尾断言。
fn finish(violations: Vec<String>) {
    assert!(
        violations.is_empty(),
        "发现 {} 个入口返回 TRUE 却给出非有限输出：\n{:#?}",
        violations.len(),
        violations
    );
}

#[test]
fn zero_arg_entries_are_finite() {
    let mut v = Vec::new();
    for (name, f) in A0 {
        let mut out = f64::NAN;
        let ok = f(&mut out);
        record(&mut v, name, ok, out, &[]);
    }
    finish(v);
}

#[test]
fn one_arg_entries_are_finite() {
    let mut v = Vec::new();
    for (name, f) in A1 {
        for a in VECTORS {
            let mut out = f64::NAN;
            let ok = f(a[0], &mut out);
            record(&mut v, name, ok, out, &a[..1]);
        }
    }
    finish(v);
}

#[test]
fn two_arg_entries_are_finite() {
    let mut v = Vec::new();
    for (name, f) in A2 {
        for a in VECTORS {
            let mut out = f64::NAN;
            let ok = f(a[0], a[1], &mut out);
            record(&mut v, name, ok, out, &a[..2]);
        }
    }
    finish(v);
}

#[test]
fn three_arg_entries_are_finite() {
    let mut v = Vec::new();
    for (name, f) in A3 {
        for a in VECTORS {
            let mut out = f64::NAN;
            let ok = f(a[0], a[1], a[2], &mut out);
            record(&mut v, name, ok, out, &a[..3]);
        }
    }
    finish(v);
}

#[test]
fn four_arg_entries_are_finite() {
    let mut v = Vec::new();
    for (name, f) in A4 {
        for a in VECTORS {
            let mut out = f64::NAN;
            let ok = f(a[0], a[1], a[2], a[3], &mut out);
            record(&mut v, name, ok, out, &a[..4]);
        }
    }
    finish(v);
}

#[test]
fn five_arg_entries_are_finite() {
    let mut v = Vec::new();
    for (name, f) in A5 {
        for a in VECTORS {
            let mut out = f64::NAN;
            let ok = f(a[0], a[1], a[2], a[3], a[4], &mut out);
            record(&mut v, name, ok, out, &a[..5]);
        }
    }
    finish(v);
}
/// 表必须非空，否则上面六个测试会「全绿但什么也没测」。
#[test]
fn tables_cover_every_drivable_entry() {
    let total = A0.len() + A1.len() + A2.len() + A3.len() + A4.len() + A5.len();
    assert_eq!(total, 86, "有限性表应覆盖 86 个可表驱动入口，实际 {total}");
}
const A0: &[(&str, F0)] = &[
    (
        "nuclear_dd_fusion_branch1_energy",
        mps_core::rapier::nucphys::nuclear_dd_fusion_branch1_energy,
    ),
    (
        "nuclear_dd_fusion_branch2_energy",
        mps_core::rapier::nucphys::nuclear_dd_fusion_branch2_energy,
    ),
    (
        "quantum_minimum_uncertainty_product",
        mps_core::rapier::qphys::quantum_minimum_uncertainty_product,
    ),
    (
        "quantum_variational_hydrogen_optimal_alpha",
        mps_core::rapier::qphys::quantum_variational_hydrogen_optimal_alpha,
    ),
];

const A1: &[(&str, F1)] = &[
    (
        "acoustics_maekawa_barrier_attenuation",
        mps_core::rapier::acoustics_ffi::acoustics_maekawa_barrier_attenuation,
    ),
    (
        "acoustics_thorp_absorption",
        mps_core::rapier::acoustics_ffi::acoustics_thorp_absorption,
    ),
    (
        "astrophysics_lane_emden_first_zero",
        mps_core::rapier::astrocalc::astrophysics_lane_emden_first_zero,
    ),
    (
        "astrophysics_main_sequence_lifetime",
        mps_core::rapier::astrocalc::astrophysics_main_sequence_lifetime,
    ),
    (
        "astrophysics_mass_radius_relation",
        mps_core::rapier::astrocalc::astrophysics_mass_radius_relation,
    ),
    (
        "electromagnetism_poynting_magnitude_plane_wave",
        mps_core::rapier::emag::electromagnetism_poynting_magnitude_plane_wave,
    ),
    (
        "electromagnetism_return_loss",
        mps_core::rapier::emag::electromagnetism_return_loss,
    ),
    (
        "nuclear_half_value_layer",
        mps_core::rapier::nucphys::nuclear_half_value_layer,
    ),
    (
        "nuclear_mean_lifetime",
        mps_core::rapier::nucphys::nuclear_mean_lifetime,
    ),
    (
        "quantum_angular_momentum_squared",
        mps_core::rapier::qphys::quantum_angular_momentum_squared,
    ),
    (
        "quantum_compton_wavelength_shift",
        mps_core::rapier::qphys::quantum_compton_wavelength_shift,
    ),
    (
        "quantum_photoelectric_threshold",
        mps_core::rapier::qphys::quantum_photoelectric_threshold,
    ),
    (
        "quantum_variational_hydrogen_energy",
        mps_core::rapier::qphys::quantum_variational_hydrogen_energy,
    ),
];

const A2: &[(&str, F2)] = &[
    (
        "acoustics_acoustic_impedance",
        mps_core::rapier::acoustics_ffi::acoustics_acoustic_impedance,
    ),
    (
        "acoustics_mass_law_tl",
        mps_core::rapier::acoustics_ffi::acoustics_mass_law_tl,
    ),
    (
        "astrophysics_binary_semi_major_axis",
        mps_core::rapier::astrocalc::astrophysics_binary_semi_major_axis,
    ),
    (
        "astrophysics_blackbody_spectral_radiance",
        mps_core::rapier::astrocalc::astrophysics_blackbody_spectral_radiance,
    ),
    (
        "astrophysics_eddington_luminosity",
        mps_core::rapier::astrocalc::astrophysics_eddington_luminosity,
    ),
    (
        "astrophysics_eddington_luminosity_solar",
        mps_core::rapier::astrocalc::astrophysics_eddington_luminosity_solar,
    ),
    (
        "astrophysics_mass_function",
        mps_core::rapier::astrocalc::astrophysics_mass_function,
    ),
    (
        "astrophysics_mass_luminosity_relation",
        mps_core::rapier::astrocalc::astrophysics_mass_luminosity_relation,
    ),
    (
        "astrophysics_nickel56_decay_luminosity",
        mps_core::rapier::astrocalc::astrophysics_nickel56_decay_luminosity,
    ),
    (
        "astrophysics_transit_depth",
        mps_core::rapier::astrocalc::astrophysics_transit_depth,
    ),
    (
        "electromagnetism_dipole_radiation_resistance",
        mps_core::rapier::emag::electromagnetism_dipole_radiation_resistance,
    ),
    (
        "electromagnetism_effective_aperture",
        mps_core::rapier::emag::electromagnetism_effective_aperture,
    ),
    (
        "electromagnetism_far_field_distance",
        mps_core::rapier::emag::electromagnetism_far_field_distance,
    ),
    (
        "electromagnetism_intrinsic_impedance",
        mps_core::rapier::emag::electromagnetism_intrinsic_impedance,
    ),
    (
        "electromagnetism_quarter_wave_transformer",
        mps_core::rapier::emag::electromagnetism_quarter_wave_transformer,
    ),
    (
        "electromagnetism_wavelength_in_medium",
        mps_core::rapier::emag::electromagnetism_wavelength_in_medium,
    ),
    (
        "nuclear_atomic_mass_approx",
        mps_core::rapier::nucphys::nuclear_atomic_mass_approx,
    ),
    (
        "nuclear_bethe_weizsaecker_binding_energy",
        mps_core::rapier::nucphys::nuclear_bethe_weizsaecker_binding_energy,
    ),
    (
        "nuclear_binding_energy_per_nucleon",
        mps_core::rapier::nucphys::nuclear_binding_energy_per_nucleon,
    ),
    (
        "nuclear_reaction_rate",
        mps_core::rapier::nucphys::nuclear_reaction_rate,
    ),
    (
        "nuclear_specific_activity",
        mps_core::rapier::nucphys::nuclear_specific_activity,
    ),
    (
        "quantum_compton_scattered_wavelength",
        mps_core::rapier::qphys::quantum_compton_scattered_wavelength,
    ),
    (
        "quantum_einstein_a_coefficient",
        mps_core::rapier::qphys::quantum_einstein_a_coefficient,
    ),
    (
        "quantum_fermi_golden_rule_linear",
        mps_core::rapier::qphys::quantum_fermi_golden_rule_linear,
    ),
    (
        "quantum_free_particle_energy",
        mps_core::rapier::qphys::quantum_free_particle_energy,
    ),
    (
        "quantum_photoelectric_max_kinetic",
        mps_core::rapier::qphys::quantum_photoelectric_max_kinetic,
    ),
    (
        "relativity_flat_universe_lookback_time",
        mps_core::rapier::rel::relativity_flat_universe_lookback_time,
    ),
    (
        "relativity_gw_frequency_derivative",
        mps_core::rapier::rel::relativity_gw_frequency_derivative,
    ),
    (
        "relativity_gw_inspiral_time_to_coalescence",
        mps_core::rapier::rel::relativity_gw_inspiral_time_to_coalescence,
    ),
    (
        "relativity_hawking_temperature",
        mps_core::rapier::rel::relativity_hawking_temperature,
    ),
    (
        "relativity_hubble_distance",
        mps_core::rapier::rel::relativity_hubble_distance,
    ),
    (
        "relativity_hubble_recession_velocity",
        mps_core::rapier::rel::relativity_hubble_recession_velocity,
    ),
    (
        "relativity_photon_sphere_radius",
        mps_core::rapier::rel::relativity_photon_sphere_radius,
    ),
    (
        "relativity_relativistic_aberration",
        mps_core::rapier::rel::relativity_relativistic_aberration,
    ),
    (
        "relativity_relativistic_doppler_beaming_factor",
        mps_core::rapier::rel::relativity_relativistic_doppler_beaming_factor,
    ),
    (
        "relativity_relativistic_doppler_transverse",
        mps_core::rapier::rel::relativity_relativistic_doppler_transverse,
    ),
    (
        "relativity_relativistic_energy_from_momentum",
        mps_core::rapier::rel::relativity_relativistic_energy_from_momentum,
    ),
    (
        "relativity_relativistic_momentum",
        mps_core::rapier::rel::relativity_relativistic_momentum,
    ),
    (
        "relativity_relativistic_total_energy",
        mps_core::rapier::rel::relativity_relativistic_total_energy,
    ),
    (
        "tyndall_mie_extinction_efficiency_exact",
        mps_core::rapier::emag::tyndall_mie_extinction_efficiency_exact,
    ),
    (
        "tyndall_mie_scattering_efficiency_best",
        mps_core::rapier::emag::tyndall_mie_scattering_efficiency_best,
    ),
    (
        "tyndall_mie_scattering_efficiency_exact",
        mps_core::rapier::emag::tyndall_mie_scattering_efficiency_exact,
    ),
];

const A3: &[(&str, F3)] = &[
    (
        "acoustics_eyring_rt60",
        mps_core::rapier::acoustics_ffi::acoustics_eyring_rt60,
    ),
    (
        "acoustics_sabine_rt60",
        mps_core::rapier::acoustics_ffi::acoustics_sabine_rt60,
    ),
    (
        "astrophysics_jeans_length",
        mps_core::rapier::astrocalc::astrophysics_jeans_length,
    ),
    (
        "astrophysics_jeans_mass",
        mps_core::rapier::astrocalc::astrophysics_jeans_mass,
    ),
    (
        "astrophysics_nfw_circular_velocity",
        mps_core::rapier::astrocalc::astrophysics_nfw_circular_velocity,
    ),
    (
        "astrophysics_nfw_density",
        mps_core::rapier::astrocalc::astrophysics_nfw_density,
    ),
    (
        "astrophysics_nfw_enclosed_mass",
        mps_core::rapier::astrocalc::astrophysics_nfw_enclosed_mass,
    ),
    (
        "electromagnetism_coaxial_cutoff_frequency",
        mps_core::rapier::emag::electromagnetism_coaxial_cutoff_frequency,
    ),
    (
        "electromagnetism_coaxial_impedance",
        mps_core::rapier::emag::electromagnetism_coaxial_impedance,
    ),
    (
        "electromagnetism_faraday_rotation",
        mps_core::rapier::emag::electromagnetism_faraday_rotation,
    ),
    (
        "electromagnetism_rayleigh_scattering_cross_section",
        mps_core::rapier::emag::electromagnetism_rayleigh_scattering_cross_section,
    ),
    (
        "electromagnetism_skin_depth",
        mps_core::rapier::emag::electromagnetism_skin_depth,
    ),
    (
        "quantum_rabi_oscillation_probability",
        mps_core::rapier::qphys::quantum_rabi_oscillation_probability,
    ),
    (
        "relativity_gravitational_redshift",
        mps_core::rapier::rel::relativity_gravitational_redshift,
    ),
    (
        "relativity_gw_strain_amplitude",
        mps_core::rapier::rel::relativity_gw_strain_amplitude,
    ),
    (
        "relativity_lense_thirring_angular_frequency",
        mps_core::rapier::rel::relativity_lense_thirring_angular_frequency,
    ),
    (
        "relativity_schwarzschild_effective_potential",
        mps_core::rapier::rel::relativity_schwarzschild_effective_potential,
    ),
    (
        "tyndall_single_scattering_albedo_complex",
        mps_core::rapier::emag::tyndall_single_scattering_albedo_complex,
    ),
];

const A4: &[(&str, F4)] = &[
    (
        "astrophysics_radial_velocity_semi_amplitude",
        mps_core::rapier::astrocalc::astrophysics_radial_velocity_semi_amplitude,
    ),
    (
        "astrophysics_ss73_disk_temperature",
        mps_core::rapier::astrocalc::astrophysics_ss73_disk_temperature,
    ),
    (
        "nuclear_four_factor_formula",
        mps_core::rapier::nucphys::nuclear_four_factor_formula,
    ),
    (
        "quantum_spin_orbit_energy",
        mps_core::rapier::qphys::quantum_spin_orbit_energy,
    ),
    (
        "relativity_einstein_radius",
        mps_core::rapier::rel::relativity_einstein_radius,
    ),
    (
        "relativity_gw_inspiral_snr",
        mps_core::rapier::rel::relativity_gw_inspiral_snr,
    ),
    (
        "relativity_kerr_ergosphere_radius",
        mps_core::rapier::rel::relativity_kerr_ergosphere_radius,
    ),
];

const A5: &[(&str, F5)] = &[
    (
        "electromagnetism_friis_power_received",
        mps_core::rapier::emag::electromagnetism_friis_power_received,
    ),
    (
        "relativity_kerr_frame_dragging_frequency",
        mps_core::rapier::rel::relativity_kerr_frame_dragging_frequency,
    ),
];
