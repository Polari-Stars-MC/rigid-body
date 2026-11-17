//! Electromagnetism C ABI — thin `#[unsafe(no_mangle)]` wrappers around the
//! pure `mps_formula::electromagnetism` scalar helpers (Poynting, wave
//! propagation, antennas, impedance matching, coax, scattering, Faraday).
//!
//! Scalar results reuse [`crate::rapier::ffi::ffi_scalar`] (null `out` →
//! `Bool::FALSE`, `None` result → `Bool::FALSE`, otherwise writes and returns
//! `Bool::TRUE`). `transmission_line_input_impedance` writes two `f64`
//! outputs (real, imag).
//!
//! No `WorldHandle` / Rapier state is touched — these are pure calculators.
//! The Vec3-returning helpers (Biot–Savart, Poynting vector) and the existing
//! `em_*` FFI already live in `mps_formula::electromagnetism` and are not
//! re-wrapped here.
//!
//! Rust module name is `emag`; the exported C symbols are prefixed
//! `electromagnetism_`.
//!
//! NOTE: functions are written out explicitly (no macro) because cbindgen does
//! not expand declarative macros, so macro-generated `pub extern "C" fn`
//! items are silently omitted from `rigid_body.h`.

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use crate::rapier::ffi::{Bool, ffi_scalar};
use mps_formula::electromagnetism::*;

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_poynting_magnitude_plane_wave(
    e_field_magnitude: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || poynting_magnitude_plane_wave(e_field_magnitude))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_phase_velocity(refractive_index: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || phase_velocity(refractive_index))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_wavelength_in_medium(
    frequency: f64,
    refractive_index: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || wavelength_in_medium(frequency, refractive_index))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_intrinsic_impedance(
    permeability: f64,
    permittivity: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || intrinsic_impedance(permeability, permittivity))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_skin_depth(
    frequency: f64,
    permeability: f64,
    conductivity: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || skin_depth(frequency, permeability, conductivity))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_vacuum_wavelength(frequency: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || vacuum_wavelength(frequency))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_wave_frequency(wavelength: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || wave_frequency(wavelength))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_dipole_radiation_resistance(
    dipole_length: f64,
    wavelength: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        dipole_radiation_resistance(dipole_length, wavelength)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_half_wave_dipole_directivity(out: *mut f64) -> Bool {
    ffi_scalar(out, || Some(half_wave_dipole_directivity()))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_effective_aperture(
    gain_linear: f64,
    wavelength: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || effective_aperture(gain_linear, wavelength))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_far_field_distance(
    antenna_size: f64,
    wavelength: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || far_field_distance(antenna_size, wavelength))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_friis_power_received(
    transmit_power: f64,
    tx_gain: f64,
    rx_gain: f64,
    wavelength: f64,
    range: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        friis_power_received(transmit_power, tx_gain, rx_gain, wavelength, range)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_reflection_coefficient(
    load_impedance: f64,
    characteristic_impedance: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        reflection_coefficient(load_impedance, characteristic_impedance)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_vswr(reflection_coeff: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || vswr(reflection_coeff))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_return_loss(reflection_coeff: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || return_loss(reflection_coeff))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_quarter_wave_transformer(
    z0: f64,
    z_load: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || quarter_wave_transformer(z0, z_load))
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_coaxial_impedance(
    inner_diameter: f64,
    outer_diameter: f64,
    relative_permittivity: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        coaxial_impedance(inner_diameter, outer_diameter, relative_permittivity)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_coaxial_cutoff_frequency(
    inner_diameter: f64,
    outer_diameter: f64,
    relative_permittivity: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        coaxial_cutoff_frequency(inner_diameter, outer_diameter, relative_permittivity)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_rayleigh_scattering_cross_section(
    refractive_index: f64,
    diameter: f64,
    wavelength: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        rayleigh_scattering_cross_section(refractive_index, diameter, wavelength)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_faraday_rotation(
    verdet_constant: f64,
    magnetic_field: f64,
    path_length: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        faraday_rotation(verdet_constant, magnetic_field, path_length)
    })
}

/// Transmission-line input impedance (lossless). Writes (real, imag) into
/// `out_real` / `out_imag`. Returns `Bool::FALSE` on invalid input or a null output.
#[unsafe(no_mangle)]
pub extern "C" fn electromagnetism_transmission_line_input_impedance(
    z0: f64,
    z_load_real: f64,
    z_load_imag: f64,
    phase_constant: f64,
    length: f64,
    out_real: *mut f64,
    out_imag: *mut f64,
) -> Bool {
    let a = ffi_scalar(out_real, || -> Option<f64> {
        transmission_line_input_impedance(z0, z_load_real, z_load_imag, phase_constant, length)
            .map(|(re, _)| re)
    });
    let b = ffi_scalar(out_imag, || -> Option<f64> {
        transmission_line_input_impedance(z0, z_load_real, z_load_imag, phase_constant, length)
            .map(|(_, im)| im)
    });
    if a == Bool::TRUE && b == Bool::TRUE {
        Bool::TRUE
    } else {
        Bool::FALSE
    }
}

// ── 丁达尔效应 / 体积散射（ADR-0018）────────────────────────────────────────
//
// 纯计算器，不碰 WorldHandle。公式在 `mps_formula::tyndall`，这里只做 C ABI
// 包装（与上文 em_* 同风格）。
//
// 这些入口给 Minecraft 侧渲染光柱用：宿主拿到散射系数、相函数值、沿视线的
// 累积亮度，自己决定怎么画。物理层不关心渲染。

#[unsafe(no_mangle)]
pub extern "C" fn tyndall_size_parameter(diameter: f64, wavelength: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::size_parameter(diameter, wavelength)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn tyndall_rayleigh_phase(cos_theta: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || mps_formula::tyndall::rayleigh_phase(cos_theta))
}

#[unsafe(no_mangle)]
pub extern "C" fn tyndall_henyey_greenstein_phase(cos_theta: f64, g: f64, out: *mut f64) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::henyey_greenstein_phase(cos_theta, g)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn tyndall_mie_scattering_efficiency(
    size_param: f64,
    refractive_index: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::mie_scattering_efficiency(size_param, refractive_index)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn tyndall_beer_lambert_transmittance(
    sigma_ext: f64,
    number_density: f64,
    path_length: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::beer_lambert_transmittance(sigma_ext, number_density, path_length)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn tyndall_optical_thickness(
    sigma_ext: f64,
    number_density: f64,
    path_length: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::optical_thickness(sigma_ext, number_density, path_length)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn tyndall_single_scattering_albedo(
    sigma_sca: f64,
    sigma_ext: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::single_scattering_albedo(sigma_sca, sigma_ext)
    })
}

/// 沿视线单次散射的累积亮度，见 `mps_formula::tyndall::inscatter_along_ray`。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_inscatter_along_ray(
    sigma_sca: f64,
    sigma_ext: f64,
    number_density: f64,
    phase: f64,
    light_intensity: f64,
    ray_length: f64,
    steps: u32,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::inscatter_along_ray(
            sigma_sca,
            sigma_ext,
            number_density,
            phase,
            light_intensity,
            ray_length,
            steps,
        )
    })
}

/// 丁达尔可见度评分（`0..=1`），渲染层可直接用它调制光柱亮度。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_visibility_score(
    sigma_sca: f64,
    sigma_ext: f64,
    number_density: f64,
    path_length: f64,
    g: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::tyndall_visibility(
            sigma_sca,
            sigma_ext,
            number_density,
            path_length,
            g,
        )
    })
}

// ── 丁达尔光谱版（ADR-0020）─────────────────────────────────────────────────
//
// 标量版本只能让光柱"变暗"，分通道版本能让它"变暖"—— 穿过厚烟雾后蓝光先被
// 散射掉，光柱偏红，与日落同源。
//
// 写 `out_r` / `out_g` / `out_b` 三个 `f64`，全部成功才写，避免半更新的状态。

#[unsafe(no_mangle)]
pub extern "C" fn tyndall_wavelength_to_rgb(
    wavelength: f64,
    out_r: *mut f64,
    out_g: *mut f64,
    out_b: *mut f64,
) -> Bool {
    if out_r.is_null() || out_g.is_null() || out_b.is_null() {
        set_error(ERR_NULL_POINTER, "tyndall_wavelength_to_rgb: out is null");
        return Bool::FALSE;
    }
    ffi_guard(
        Bool::FALSE,
        || match mps_formula::tyndall_spectral::wavelength_to_rgb(wavelength) {
            Some((r, g, b)) => {
                unsafe {
                    *out_r = r;
                    *out_g = g;
                    *out_b = b;
                }
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_wavelength_to_rgb: non-finite wavelength",
                );
                Bool::FALSE
            }
        },
    )
}

/// 由单波长参数构造三通道光学参数（米氏标度 `σ ∝ λ^(-n)`）。
///
/// `exponent` 取 `1.0~1.5` 做丁达尔光柱；取 `4.0` 是瑞利极限（强烈偏蓝）。
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn tyndall_spectral_optics_from_monochrome(
    sigma_sca: f64,
    sigma_ext: f64,
    mass: f64,
    wavelength_ref: f64,
    exponent: f64,
    out_sigma_sca: *mut f64,
    out_sigma_ext: *mut f64,
) -> Bool {
    if out_sigma_sca.is_null() || out_sigma_ext.is_null() {
        set_error(
            ERR_NULL_POINTER,
            "tyndall_spectral_optics_from_monochrome: out is null",
        );
        return Bool::FALSE;
    }
    ffi_guard(Bool::FALSE, || {
        match mps_formula::tyndall_spectral::SpectralOptics::from_monochrome(
            sigma_sca,
            sigma_ext,
            mass,
            wavelength_ref,
            exponent,
        ) {
            Some(o) => {
                // 按 RGB 顺序写 6 个值：sca_r, sca_g, sca_b, ext_r, ext_g, ext_b。
                unsafe {
                    *out_sigma_sca = o.sigma_sca_r;
                    *out_sigma_ext = o.sigma_ext_r;
                    *out_sigma_sca.add(1) = o.sigma_sca_g;
                    *out_sigma_ext.add(1) = o.sigma_ext_g;
                    *out_sigma_sca.add(2) = o.sigma_sca_b;
                    *out_sigma_ext.add(2) = o.sigma_ext_b;
                }
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_spectral_optics_from_monochrome: invalid arguments",
                );
                Bool::FALSE
            }
        }
    })
}

/// 均匀介质的分通道散射亮度。
///
/// 六个 `sigma_*` 分别是 R/G/B 的散射与消光截面；`light_*` 是三通道入射强度。
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn tyndall_inscatter_spectral(
    sigma_sca_r: f64,
    sigma_sca_g: f64,
    sigma_sca_b: f64,
    sigma_ext_r: f64,
    sigma_ext_g: f64,
    sigma_ext_b: f64,
    mass: f64,
    number_density: f64,
    phase: f64,
    light_r: f64,
    light_g: f64,
    light_b: f64,
    ray_length: f64,
    steps: u32,
    out_r: *mut f64,
    out_g: *mut f64,
    out_b: *mut f64,
) -> Bool {
    if out_r.is_null() || out_g.is_null() || out_b.is_null() {
        set_error(ERR_NULL_POINTER, "tyndall_inscatter_spectral: out is null");
        return Bool::FALSE;
    }
    ffi_guard(Bool::FALSE, || {
        let optics = mps_formula::tyndall_spectral::SpectralOptics {
            sigma_sca_r,
            sigma_sca_g,
            sigma_sca_b,
            sigma_ext_r,
            sigma_ext_g,
            sigma_ext_b,
            mass,
        };
        let light = mps_formula::tyndall_spectral::SpectralLuminance {
            r: light_r,
            g: light_g,
            b: light_b,
        };
        match mps_formula::tyndall_spectral::inscatter_spectral(
            optics,
            number_density,
            phase,
            light,
            ray_length,
            steps,
        ) {
            Some(v) => {
                unsafe {
                    *out_r = v.r;
                    *out_g = v.g;
                    *out_b = v.b;
                }
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_inscatter_spectral: invalid arguments",
                );
                Bool::FALSE
            }
        }
    })
}

/// 分通道透射率。
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn tyndall_spectral_transmittance(
    sigma_ext_r: f64,
    sigma_ext_g: f64,
    sigma_ext_b: f64,
    sigma_sca_r: f64,
    sigma_sca_g: f64,
    sigma_sca_b: f64,
    mass: f64,
    number_density: f64,
    path_length: f64,
    out_r: *mut f64,
    out_g: *mut f64,
    out_b: *mut f64,
) -> Bool {
    if out_r.is_null() || out_g.is_null() || out_b.is_null() {
        set_error(
            ERR_NULL_POINTER,
            "tyndall_spectral_transmittance: out is null",
        );
        return Bool::FALSE;
    }
    ffi_guard(Bool::FALSE, || {
        let optics = mps_formula::tyndall_spectral::SpectralOptics {
            sigma_sca_r,
            sigma_sca_g,
            sigma_sca_b,
            sigma_ext_r,
            sigma_ext_g,
            sigma_ext_b,
            mass,
        };
        match mps_formula::tyndall_spectral::spectral_transmittance(
            optics,
            number_density,
            path_length,
        ) {
            Some(v) => {
                unsafe {
                    *out_r = v.r;
                    *out_g = v.g;
                    *out_b = v.b;
                }
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_spectral_transmittance: invalid arguments",
                );
                Bool::FALSE
            }
        }
    })
}
/// 多次散射的沿视线累积亮度（ADR-0022）。
///
/// `orders = 1` 时与 `tyndall_inscatter_along_ray` 逐位相同；`orders > 1` 时
/// 按等比级数叠加高阶散射，修正厚介质的低估。
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn tyndall_inscatter_multiple(
    sigma_sca: f64,
    sigma_ext: f64,
    number_density: f64,
    phase: f64,
    light_intensity: f64,
    ray_length: f64,
    steps: u32,
    orders: u32,
    geometric_factor: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::inscatter_along_ray_multiple(
            sigma_sca,
            sigma_ext,
            number_density,
            phase,
            light_intensity,
            ray_length,
            steps,
            orders,
            geometric_factor,
        )
    })
}

/// 多次散射的阶数上限（C 侧探测用）。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_max_scatter_orders() -> u32 {
    ffi_guard(0, || mps_formula::tyndall::MAX_SCATTER_ORDERS)
}
/// 米氏散射效率的**精确解**（Riccati–Bessel 级数，ADR-0025/0026）。
///
/// 比 `tyndall_mie_scattering_efficiency`（三段渐近式）准确，代价是
/// `O(n_max)` 次复数运算。`x > 1` 时已验证；`x < 0.3` 的瑞利极限有约
/// 1.52 倍偏差（见 ADR-0025），需要定量精度时请用
/// `tyndall_mie_scattering_efficiency_best`。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_mie_scattering_efficiency_exact(
    size_param: f64,
    refractive_index: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::mie_scattering_efficiency_exact(size_param, refractive_index)
    })
}

/// 米氏消光效率的精确解（满足光学定理 `Q_ext == Q_sca`，无吸收时）。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_mie_extinction_efficiency_exact(
    size_param: f64,
    refractive_index: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::mie_extinction_efficiency_exact(size_param, refractive_index)
    })
}

/// 按尺寸参数自动选择米氏算法（`x < 0.3` 用渐近式，否则用精确解）。
///
/// **这是推荐入口**：它规避了精确解在瑞利极限的偏差，也避免了宿主自己
/// 判断该用哪个。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_mie_scattering_efficiency_best(
    size_param: f64,
    refractive_index: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::mie_scattering_efficiency_best(size_param, refractive_index)
    })
}
/// 复折射率下的米氏效率三元组 `(Q_ext, Q_sca, Q_abs)`（ADR-0027）。
///
/// `refractive_index_imag` 是吸收系数 `κ`：0 = 透明颗粒，>0 = 吸光颗粒。
/// 三个输出**全部成功才写**。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_mie_efficiencies_complex(
    size_param: f64,
    refractive_index_real: f64,
    refractive_index_imag: f64,
    out_ext: *mut f64,
    out_sca: *mut f64,
    out_abs: *mut f64,
) -> Bool {
    if out_ext.is_null() || out_sca.is_null() || out_abs.is_null() {
        set_error(
            ERR_NULL_POINTER,
            "tyndall_mie_efficiencies_complex: out is null",
        );
        return Bool::FALSE;
    }
    ffi_guard(
        Bool::FALSE,
        || match mps_formula::tyndall::mie_efficiencies_complex(
            size_param,
            refractive_index_real,
            refractive_index_imag,
        ) {
            Some((q_ext, q_sca, q_abs)) => {
                unsafe {
                    *out_ext = q_ext;
                    *out_sca = q_sca;
                    *out_abs = q_abs;
                }
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_mie_efficiencies_complex: invalid arguments",
                );
                Bool::FALSE
            }
        },
    )
}

/// 复折射率下的单散射反照率 `ω = Q_sca / Q_ext`（ADR-0027）。
///
/// 直接对应 [ADR-0020](../adr/0020-spectral-tyndall.md) 里 `ω` 的精确版：
/// `1` = 纯散射，`→ 0` = 强吸收。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_single_scattering_albedo_complex(
    size_param: f64,
    refractive_index_real: f64,
    refractive_index_imag: f64,
    out: *mut f64,
) -> Bool {
    ffi_scalar(out, || {
        mps_formula::tyndall::single_scattering_albedo_complex(
            size_param,
            refractive_index_real,
            refractive_index_imag,
        )
    })
}
/// 由精确米氏散射算出三通道光谱光学参数（ADR-0030）。
///
/// 输入是**材料参数**（直径、复折射率、密度），输出六通道截面。
/// 与 `tyndall_spectral_optics_from_monochrome` 的区别：那里需要手调波长指数，
/// 这里逐通道解米氏方程，自动包含米氏区的振荡结构。
///
/// 六个 `out_*` 指针**全部成功才写**。
///
/// # Safety
/// 六个输出指针必须可写。
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn tyndall_spectral_optics_from_mie(
    diameter: f64,
    refractive_index_real: f64,
    refractive_index_imag: f64,
    density: f64,
    out_sca_r: *mut f64,
    out_sca_g: *mut f64,
    out_sca_b: *mut f64,
    out_ext_r: *mut f64,
    out_ext_g: *mut f64,
    out_ext_b: *mut f64,
) -> Bool {
    if out_sca_r.is_null()
        || out_sca_g.is_null()
        || out_sca_b.is_null()
        || out_ext_r.is_null()
        || out_ext_g.is_null()
        || out_ext_b.is_null()
    {
        set_error(
            ERR_NULL_POINTER,
            "tyndall_spectral_optics_from_mie: out is null",
        );
        return Bool::FALSE;
    }
    ffi_guard(
        Bool::FALSE,
        || match mps_formula::tyndall_spectral::SpectralOptics::from_mie(
            diameter,
            refractive_index_real,
            refractive_index_imag,
            density,
        ) {
            Some(o) => {
                unsafe {
                    *out_sca_r = o.sigma_sca_r;
                    *out_sca_g = o.sigma_sca_g;
                    *out_sca_b = o.sigma_sca_b;
                    *out_ext_r = o.sigma_ext_r;
                    *out_ext_g = o.sigma_ext_g;
                    *out_ext_b = o.sigma_ext_b;
                }
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_spectral_optics_from_mie: invalid arguments (diameter > 0, \
                     n > 0, kappa >= 0, density > 0)",
                );
                Bool::FALSE
            }
        },
    )
}
/// CIE 1931 色匹配函数 `(x̄, ȳ, z̄)`（ADR-0032）。
///
/// 用 Wyman–Sloan–Shirley 解析拟合，误差约峰值 1%。`ȳ` 同时是光度函数 `V(λ)`。
/// 三个输出**全部成功才写**。
///
/// # Safety
/// 三个输出指针必须可写。
#[unsafe(no_mangle)]
pub extern "C" fn tyndall_cie_color_matching(
    wavelength_nm: f64,
    out_x: *mut f64,
    out_y: *mut f64,
    out_z: *mut f64,
) -> Bool {
    if out_x.is_null() || out_y.is_null() || out_z.is_null() {
        set_error(ERR_NULL_POINTER, "tyndall_cie_color_matching: out is null");
        return Bool::FALSE;
    }
    ffi_guard(
        Bool::FALSE,
        || match mps_formula::tyndall_spectral::cie_color_matching(wavelength_nm) {
            Some((x, y, z)) => {
                unsafe {
                    *out_x = x;
                    *out_y = y;
                    *out_z = z;
                }
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_cie_color_matching: non-finite wavelength",
                );
                Bool::FALSE
            }
        },
    )
}

/// **连续光谱的均匀介质散射 → 线性 sRGB**（ADR-0032）。
///
/// 这是三离散波长版本的物理升级版：逐波长解米氏方程，再用 CIE 1931 匹配函数
/// 梯形积分。米氏区的振荡结构会被正确积分进去。
///
/// 输出是**线性** sRGB，显示端还需 gamma 编码（`v^(1/2.2)` 近似）。
///
/// # Parameters
/// * `diameter` 颗粒直径（m）
/// * `refractive_index_real` / `refractive_index_imag` 复折射率（`κ = 0` 为透明）
/// * `number_density` 数密度（个/m³）
/// * `phase` 相函数值（丁达尔取前向 0.7~0.9 的 Henyey–Greenstein）
/// * `light_intensity` 入射光强度
/// * `ray_length` 视线长度（m）
/// * `steps` 单次散射积分的采样段数
/// * `samples` 光谱采样点数（推荐 15~31）
///
/// # Safety
/// `out_r` / `out_g` / `out_b` 必须可写。
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn tyndall_inscatter_rgb_uniform(
    diameter: f64,
    refractive_index_real: f64,
    refractive_index_imag: f64,
    number_density: f64,
    phase: f64,
    light_intensity: f64,
    ray_length: f64,
    steps: u32,
    samples: u32,
    out_r: *mut f64,
    out_g: *mut f64,
    out_b: *mut f64,
) -> Bool {
    if out_r.is_null() || out_g.is_null() || out_b.is_null() {
        set_error(
            ERR_NULL_POINTER,
            "tyndall_inscatter_rgb_uniform: out is null",
        );
        return Bool::FALSE;
    }
    ffi_guard(
        Bool::FALSE,
        || match mps_formula::tyndall_spectral::inscatter_rgb_uniform(
            diameter,
            refractive_index_real,
            refractive_index_imag,
            number_density,
            phase,
            light_intensity,
            ray_length,
            steps,
            samples,
        ) {
            Some(v) => {
                unsafe {
                    *out_r = v.r;
                    *out_g = v.g;
                    *out_b = v.b;
                }
                clear_error();
                Bool::TRUE
            }
            None => {
                set_error(
                    ERR_INVALID_ARGUMENT,
                    "tyndall_inscatter_rgb_uniform: invalid arguments",
                );
                Bool::FALSE
            }
        },
    )
}
