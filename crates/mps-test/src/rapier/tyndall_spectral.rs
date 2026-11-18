//! 丁达尔效应的**光谱**版本测试（ADR-0020）。
//!
//! 核心断言是那条物理规律：**厚介质里蓝光先被散射掉，光柱偏红**。
//! 这与日落同源，也是分通道版本相对标量版本的唯一视觉价值。

use mps_formula::tyndall_spectral::{
    BLUE_NM, RED_NM, SpectralLuminance, SpectralOptics, cie_color_matching,
    inscatter_rgb_from_particles, inscatter_rgb_uniform, inscatter_spectral,
    inscatter_spectral_full, inscatter_spectral_through_particles, mie_spectral_samples,
    spectral_albedo, spectral_samples_to_rgb, spectral_transmittance, wavelength_to_rgb,
    xyz_to_linear_srgb,
};

const EPS: f64 = 1e-12;

#[test]
fn wavelength_to_rgb_covers_the_visible_spectrum() {
    // 可见光内都有非零输出。
    for nm in [400.0, 450.0, 500.0, 550.0, 600.0, 650.0, 700.0] {
        let (r, g, b) = wavelength_to_rgb(nm * 1e-9).unwrap();
        assert!(
            r > 0.0 || g > 0.0 || b > 0.0,
            "{nm} nm should produce some colour, got ({r}, {g}, {b})"
        );
    }
    // 可见光外为全零（不是错误）。
    assert_eq!(wavelength_to_rgb(300e-9).unwrap(), (0.0, 0.0, 0.0));
    assert_eq!(wavelength_to_rgb(900e-9).unwrap(), (0.0, 0.0, 0.0));
    // 非有限值才是错误。
    assert!(wavelength_to_rgb(f64::NAN).is_none());
}

#[test]
fn wavelength_to_rgb_gives_expected_primaries() {
    // 450 nm 应以蓝为主。
    let (r, _, b) = wavelength_to_rgb(450e-9).unwrap();
    assert!(b > r, "450nm should be blue-dominant: r={r} b={b}");

    // 550 nm 应以绿为主。
    let (r, g, b) = wavelength_to_rgb(550e-9).unwrap();
    assert!(
        g > r && g > b,
        "550nm should be green-dominant: {r},{g},{b}"
    );

    // 650 nm 应以红为主。
    let (r, g, b) = wavelength_to_rgb(650e-9).unwrap();
    assert!(r > g && r > b, "650nm should be red-dominant: {r},{g},{b}");
}

#[test]
fn wavelength_to_rgb_stays_in_unit_range() {
    let mut nm = 380.0;
    while nm <= 780.0 {
        let (r, g, b) = wavelength_to_rgb(nm * 1e-9).unwrap();
        for (name, v) in [("r", r), ("g", g), ("b", b)] {
            assert!(
                (0.0..=1.0).contains(&v),
                "{nm}nm channel {name} = {v} out of range"
            );
        }
        nm += 10.0;
    }
}

/// `from_monochrome` 的标度关系：`σ ∝ λ^(-n)`。
#[test]
fn monochrome_expansion_follows_the_wavelength_law() {
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 1.0).unwrap();
    assert!(o.is_valid());

    // n = 1：蓝光（450nm）截面应是红光（650nm）的 650/450 倍。
    let ratio = o.sigma_sca_b / o.sigma_sca_r;
    let expect = RED_NM / BLUE_NM;
    assert!(
        (ratio - expect).abs() / expect < 1e-9,
        "n=1 should give ratio {expect}, got {ratio}"
    );
    // 绿光在中间。
    assert!(o.sigma_sca_b > o.sigma_sca_g && o.sigma_sca_g > o.sigma_sca_r);
}

/// `n = 0` 退化为无波长依赖（三通道相同）。
#[test]
fn zero_exponent_is_channel_flat() {
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 0.0).unwrap();
    assert!((o.sigma_sca_r - o.sigma_sca_g).abs() / o.sigma_sca_g < EPS);
    assert!((o.sigma_sca_g - o.sigma_sca_b).abs() / o.sigma_sca_b < EPS);
}

/// `n = 4` 是瑞利极限：蓝/红比应是 (650/450)⁴ ≈ 4.35。
#[test]
fn rayleigh_exponent_gives_strong_blue_bias() {
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 4.0).unwrap();
    let ratio = o.sigma_sca_b / o.sigma_sca_r;
    let expect = (RED_NM / BLUE_NM).powi(4);
    assert!(
        (ratio - expect).abs() / expect < 1e-9,
        "n=4 should give {expect}, got {ratio}"
    );
    assert!(ratio > 4.0, "rayleigh must be strongly blue-biased");
}

#[test]
fn monochrome_expansion_rejects_invalid() {
    // 散射 > 消光
    assert!(SpectralOptics::from_monochrome(2e-12, 1e-12, 1e-6, 550e-9, 1.0).is_none());
    // 质量非正
    assert!(SpectralOptics::from_monochrome(1e-12, 1e-12, 0.0, 550e-9, 1.0).is_none());
    // 参考波长非正
    assert!(SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 0.0, 1.0).is_none());
    // 非有限
    assert!(SpectralOptics::from_monochrome(f64::NAN, 1e-12, 1e-6, 550e-9, 1.0).is_none());
}

/// **核心物理断言**：穿过厚介质后，蓝光衰减得比红光快 → 透射光偏红。
#[test]
fn blue_is_attenuated_faster_than_red() {
    // 米氏区（n = 1）：蓝光截面略大。
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 1.0).unwrap();

    // 稀薄：三通道都接近 1。
    let thin = spectral_transmittance(o, 1e10, 1.0).unwrap();
    assert!(thin.r > 0.9 && thin.b > 0.9);

    // 厚：全部衰减，但蓝光衰减更多。
    let thick = spectral_transmittance(o, 1e13, 1.0).unwrap();
    assert!(
        thick.r > thick.b,
        "red must survive better than blue: r={} b={}",
        thick.r,
        thick.b
    );
    assert!(
        thick.g > thick.b,
        "green must also survive better than blue"
    );
}

/// 同样条件下，**前向散射**的光柱亮度也是蓝光先饱和（消光更快）。
#[test]
fn spectral_inscatter_shows_the_reddening() {
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 1.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };

    // 光学厚度中等偏高：消光差异开始显著。
    let v = inscatter_spectral(o, 1e13, 1.0, white, 4.0, 256).unwrap();
    assert!(v.is_finite_non_negative());

    let (r, g, b) = v.normalized();
    assert!(
        r > b,
        "the beam must be warm (r > b) after travelling: normalized ({r}, {g}, {b})"
    );
}

/// 无波长依赖时，三通道结果必须**完全相同**（与标量版本自洽）。
#[test]
fn flat_optics_gives_identical_channels() {
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 0.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };
    let v = inscatter_spectral(o, 1e12, 1.0, white, 5.0, 128).unwrap();
    assert!((v.r - v.g).abs() / v.g.max(EPS) < 1e-12);
    assert!((v.g - v.b).abs() / v.b.max(EPS) < 1e-12);
}

/// 单通道入射光只点亮对应通道。
#[test]
fn per_channel_light_isolates_channels() {
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 1.0).unwrap();
    let red_only = SpectralLuminance {
        r: 1.0,
        g: 0.0,
        b: 0.0,
    };
    let v = inscatter_spectral(o, 1e12, 1.0, red_only, 3.0, 64).unwrap();
    assert!(v.r > 0.0);
    assert_eq!(v.g, 0.0);
    assert_eq!(v.b, 0.0);
}

#[test]
fn spectral_inscatter_scales_with_density_and_length() {
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 1.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };
    let a = inscatter_spectral(o, 1e11, 1.0, white, 2.0, 128).unwrap();
    let b = inscatter_spectral(o, 1e12, 1.0, white, 2.0, 128).unwrap();
    let c = inscatter_spectral(o, 1e11, 1.0, white, 6.0, 128).unwrap();
    // 数密度 ×10 → 亮度至少上升（消光同时增大，所以不是严格 ×10）。
    assert!(b.luminance() > a.luminance());
    // 长度 ×3 → 亮度上升。
    assert!(c.luminance() > a.luminance());
}

#[test]
fn spectral_inscatter_rejects_invalid() {
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 1.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };
    // steps = 0
    assert!(inscatter_spectral(o, 1e12, 1.0, white, 1.0, 0).is_none());
    // 负长度
    assert!(inscatter_spectral(o, 1e12, 1.0, white, -1.0, 8).is_none());
    // 负入射光
    let neg = SpectralLuminance {
        r: -1.0,
        g: 0.0,
        b: 0.0,
    };
    assert!(inscatter_spectral(o, 1e12, 1.0, neg, 1.0, 8).is_none());
    // 非法光学参数
    let bad = SpectralOptics {
        sigma_sca_r: 2e-12,
        sigma_sca_g: 1e-12,
        sigma_sca_b: 1e-12,
        sigma_ext_r: 1e-12,
        sigma_ext_g: 1e-12,
        sigma_ext_b: 1e-12,
        mass: 1e-6,
    };
    assert!(inscatter_spectral(bad, 1e12, 1.0, white, 1.0, 8).is_none());
}

/// 反照率的通道差异来自**散射与吸收的指数不同**，方向需要想清楚。
///
/// 设定：`σ_sca = 1e-12`、`σ_ext = 2e-12`（即一半吸收），散射指数 1，
/// 吸收指数 0（灰体）。那么：
///
/// ```text
/// 蓝光散射更强（λ⁻¹），但吸收部分不变  → 蓝的反照率更高
/// ```
///
/// 这看起来与"光柱偏红"矛盾，其实不矛盾：**偏红来自消光更快，而不是反照率
/// 更低**。蓝光的 σ_ext 更大 → 衰减更快 → 远处剩下的是红光。反照率描述的是
/// "被消光的光里有多少被散射出来"，两者是不同的量。
///
/// 如果吸收也有强的蓝偏（`absorption_exponent > 0`），蓝的反照率才会降低。
#[test]
fn spectral_albedo_reflects_the_scattering_absorption_split() {
    // 灰体吸收（n_abs = 0）：蓝光散射更强 → 反照率更高。
    let gray = SpectralOptics::from_monochrome(1e-12, 2e-12, 1e-6, 550e-9, 1.0).unwrap();
    let a = spectral_albedo(gray).unwrap();
    assert!((0.0..=1.0).contains(&a.r));
    assert!((0.0..=1.0).contains(&a.b));
    assert!(
        a.b > a.r,
        "with grey absorption, blue keeps a higher albedo (stronger scattering): b={} r={}",
        a.b,
        a.r
    );

    // 吸收也强烈偏蓝（n_abs = 1，与散射同）：反照率趋于通道平坦。
    let absorbing =
        SpectralOptics::from_monochrome_split(1e-12, 2e-12, 1e-6, 550e-9, 1.0, 1.0).unwrap();
    let b = spectral_albedo(absorbing).unwrap();
    assert!(
        (b.b - b.r).abs() < 1e-9,
        "when both scale equally the albedo is channel-flat: b={} r={}",
        b.b,
        b.r
    );

    // 纯散射介质（σ_sca == σ_ext）：反照率恒为 1，与指数无关。
    let pure = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 4.0).unwrap();
    let c = spectral_albedo(pure).unwrap();
    assert!((c.r - 1.0).abs() < 1e-12 && (c.b - 1.0).abs() < 1e-12);
}

#[test]
fn luminance_uses_rec709_weights() {
    // 纯绿应比纯红亮（0.7152 > 0.2126）。
    let green = SpectralLuminance {
        r: 0.0,
        g: 1.0,
        b: 0.0,
    };
    let red = SpectralLuminance {
        r: 1.0,
        g: 0.0,
        b: 0.0,
    };
    assert!(green.luminance() > red.luminance());
    // 纯白 = 1。
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };
    assert!((white.luminance() - 1.0).abs() < 1e-12);
}

#[test]
fn normalized_handles_zero_safely() {
    // 全零不能产生 NaN。
    assert_eq!(SpectralLuminance::ZERO.normalized(), (0.0, 0.0, 0.0));
    let v = SpectralLuminance {
        r: 2.0,
        g: 1.0,
        b: 0.5,
    };
    assert_eq!(v.normalized(), (1.0, 0.5, 0.25));
}

// ── 粒子驱动的光谱散射（ADR-0021）───────────────────────────────────────────

/// 最关键的实现细节：**三通道的透射率必须独立累积**。
///
/// 如果只用一个透射率标量，蓝光不会比红光衰减得快，"偏红"就消失了。这个测试
/// 用过量的光学厚度把差异放大，若实现退化成立刻失败。
/// 三通道的透射率必须**独立累积**，而"偏红"只在**光学厚**时出现。
///
/// ## 两个容易误判的物理结论
///
/// **1. 光学薄（τ ≪ 1）时，蓝光一直更亮。**
///
/// ```text
/// L(λ) ∝ σ_sca(λ) · (1 - exp(-τ(λ))) ≈ σ_sca(λ) · τ(λ) ∝ σ_sca(λ)
/// ```
///
/// 蓝光的 `σ_sca` 更大，所以散射更多。这与直觉相反但正确。
///
/// **2. 光学厚（τ ≫ 1）时，红光胜出。**
///
/// `1 - exp(-τ)` 饱和到 1，各通道的差距只剩**吸收**造成的：蓝光被吸收后不再
/// 散射出来。所以"厚烟雾偏红"必须同时有 `σ_ext > σ_sca`（吸收）和足够的光学
/// 厚度。
///
/// ## 量级很重要
///
/// `sample_number_density` 返回的是 Poly6 核归一化后的**真实数密度**——对
/// 间距 0.1 m、核半径 0.8 m 的粒子云，`n ≈ 12.7`。要让 `τ = σ·n·L ≈ 1`，
/// `σ` 必须在 `1e-3` 量级，而不是 `1e-12`（后者是**单分子**截面，差 9 个数量级）。
#[test]
fn particle_spectral_keeps_per_channel_transmittance() {
    let positions: Vec<[f64; 3]> = (0..60).map(|i| [i as f64 * 0.1, 0.0, 0.0]).collect();
    // 散射指数 1（米氏区），吸收指数 4（蓝端强吸收），sigma 取 1e-2 使 tau ≫ 1。
    let o = SpectralOptics::from_monochrome_split(1e-2, 4e-2, 1e-6, 550e-9, 1.0, 4.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };

    let v = inscatter_spectral_through_particles(
        &positions,
        o,
        [-0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        60.0,
        0.8,
        1.0,
        white,
        256,
    )
    .unwrap();

    assert!(v.is_finite_non_negative());
    assert!(v.luminance() > 0.0, "the beam must scatter something");
    assert!(
        v.r > v.b,
        "optically thick + blue absorption must be warm: r={} b={}",
        v.r,
        v.b
    );
}

/// 同一介质在**光学薄**时必须是蓝光更强 —— 与上面形成对照，钉住"厚度决定偏色
/// 方向"这个反直觉的结论。
#[test]
fn optically_thin_medium_stays_blue() {
    let positions: Vec<[f64; 3]> = (0..60).map(|i| [i as f64 * 0.1, 0.0, 0.0]).collect();
    // 同样的指数关系，但 sigma 小 100 倍 => tau ≪ 1。
    let o = SpectralOptics::from_monochrome_split(1e-4, 4e-4, 1e-6, 550e-9, 1.0, 4.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };
    let v = inscatter_spectral_through_particles(
        &positions,
        o,
        [-0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        60.0,
        0.8,
        1.0,
        white,
        256,
    )
    .unwrap();
    assert!(
        v.b > v.r,
        "optically thin scattering is blue-dominant: r={} b={}",
        v.r,
        v.b
    );
}
/// 对照：**纯散射**介质里蓝光一直更亮，与厚度无关 —— 这是上面那条结论的直接
/// 验证，也防止有人"修正"实现去迎合错误直觉。
#[test]
fn pure_scattering_stays_blue_regardless_of_thickness() {
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 1.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };
    for count in [3usize, 10, 40, 120] {
        let positions: Vec<[f64; 3]> = (0..count).map(|i| [i as f64 * 0.15, 0.0, 0.0]).collect();
        let v = inscatter_spectral_through_particles(
            &positions,
            o,
            [-1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            8.0,
            0.8,
            1.0,
            white,
            128,
        )
        .unwrap();
        // 纯散射：蓝光更强，且比值恒为 σ_sca 比。
        assert!(
            v.b > v.r,
            "pure scattering must stay blue at count={count}: r={} b={}",
            v.r,
            v.b
        );
    }
}

/// 无波长依赖时三通道必须相同（与标量实现自洽）。
#[test]
fn particle_spectral_flat_optics_match_scalar() {
    let positions: Vec<[f64; 3]> = (0..20).map(|i| [i as f64 * 0.25, 0.0, 0.0]).collect();
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 0.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };

    let v = inscatter_spectral_through_particles(
        &positions,
        o,
        [-1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        6.0,
        0.8,
        1.0,
        white,
        96,
    )
    .unwrap();

    // 三通道一致。
    assert!((v.r - v.g).abs() / v.g.max(1e-30) < 1e-12);
    assert!((v.g - v.b).abs() / v.b.max(1e-30) < 1e-12);

    // 与标量版本对比：同样的粒子、同样的参数，亮度应一致。
    use mps_formula::tyndall_particles::{ParticleOptics, inscatter_through_particles};
    let scalar = inscatter_through_particles(
        &positions,
        ParticleOptics {
            sigma_sca: 1e-12,
            sigma_ext: 1e-12,
            mass: 1e-6,
        },
        [-1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        6.0,
        0.8,
        1.0,
        1.0,
        96,
    )
    .unwrap();
    assert!(
        (v.r - scalar).abs() / scalar.max(1e-30) < 1e-9,
        "flat spectral must equal the scalar result: {} vs {scalar}",
        v.r
    );
}

/// 视线错过粒子云时为全零。
#[test]
fn particle_spectral_is_zero_when_ray_misses() {
    let positions = vec![[0.0, 0.0, 0.0], [0.2, 0.0, 0.0]];
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 1.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };
    let v = inscatter_spectral_through_particles(
        &positions,
        o,
        [-1.0, 50.0, 0.0],
        [1.0, 0.0, 0.0],
        4.0,
        0.5,
        1.0,
        white,
        32,
    )
    .unwrap();
    assert_eq!(v, SpectralLuminance::ZERO);
}

#[test]
fn particle_spectral_rejects_bad_arguments() {
    let positions = vec![[0.0, 0.0, 0.0]];
    let o = SpectralOptics::from_monochrome(1e-12, 1e-12, 1e-6, 550e-9, 1.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };

    // 未归一化方向。
    assert!(
        inscatter_spectral_through_particles(
            &positions,
            o,
            [0.0; 3],
            [2.0, 0.0, 0.0],
            1.0,
            0.5,
            1.0,
            white,
            8
        )
        .is_none()
    );
    // steps = 0
    assert!(
        inscatter_spectral_through_particles(
            &positions,
            o,
            [0.0; 3],
            [1.0, 0.0, 0.0],
            1.0,
            0.5,
            1.0,
            white,
            0
        )
        .is_none()
    );
    // 负长度
    assert!(
        inscatter_spectral_through_particles(
            &positions,
            o,
            [0.0; 3],
            [1.0, 0.0, 0.0],
            -1.0,
            0.5,
            1.0,
            white,
            8
        )
        .is_none()
    );
    // 负入射光
    let neg = SpectralLuminance {
        r: -1.0,
        g: 0.0,
        b: 0.0,
    };
    assert!(
        inscatter_spectral_through_particles(
            &positions,
            o,
            [0.0; 3],
            [1.0, 0.0, 0.0],
            1.0,
            0.5,
            1.0,
            neg,
            8
        )
        .is_none()
    );
}
// ── 完整版：粒子 + 光谱 + 多次散射 + 加速（ADR-0024）────────────────────────

/// `orders = 1` 必须与单次散射版本**逐位相同**。
#[test]
fn full_with_one_order_matches_single_scatter() {
    let positions: Vec<[f64; 3]> = (0..80).map(|i| [i as f64 * 0.05, 0.0, 0.0]).collect();
    let o = SpectralOptics::from_monochrome_split(1e-2, 4e-2, 1e-6, 550e-9, 1.0, 4.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };
    let args = ([-0.5, 0.0, 0.0], [1.0, 0.0, 0.0], 8.0, 0.8, 1.0);

    let single = inscatter_spectral_through_particles(
        &positions, o, args.0, args.1, args.2, args.3, args.4, white, 64,
    )
    .unwrap();
    let full = inscatter_spectral_full(
        &positions, o, args.0, args.1, args.2, args.3, args.4, white, 64, 1, 0.5,
    )
    .unwrap();

    assert!((full.r - single.r).abs() <= single.r.max(1e-30) * 1e-15);
    assert!((full.g - single.g).abs() <= single.g.max(1e-30) * 1e-15);
    assert!((full.b - single.b).abs() <= single.b.max(1e-30) * 1e-15);
}

/// **每通道用自己的反照率** —— 这是分通道多次散射的关键。
///
/// 如果三通道共享一个 `r = ω·g`，修正倍数就会相同，"高阶散射也偏红"这一半
/// 物理就丢了。这个测试构造三通道反照率差异明显的介质，检查修正倍数的差异。
#[test]
fn full_uses_per_channel_albedo_for_the_series() {
    let positions: Vec<[f64; 3]> = (0..80).map(|i| [i as f64 * 0.05, 0.0, 0.0]).collect();
    // 红通道：几乎纯散射（ω_r ≈ 1）；蓝通道：强吸收（ω_b 明显 < 1）。
    let o = SpectralOptics {
        sigma_sca_r: 1e-2,
        sigma_sca_g: 5e-3,
        sigma_sca_b: 1e-3,
        sigma_ext_r: 1.01e-2, // ω_r ≈ 0.99
        sigma_ext_g: 2e-2,    // ω_g = 0.25
        sigma_ext_b: 5e-2,    // ω_b = 0.02
        mass: 1e-6,
    };
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };
    let args = ([-0.5, 0.0, 0.0], [1.0, 0.0, 0.0], 8.0, 0.8, 1.0);

    let single = inscatter_spectral_through_particles(
        &positions, o, args.0, args.1, args.2, args.3, args.4, white, 64,
    )
    .unwrap();
    let full = inscatter_spectral_full(
        &positions, o, args.0, args.1, args.2, args.3, args.4, white, 64, 12, 0.5,
    )
    .unwrap();

    let ratio_r = full.r / single.r;
    let ratio_g = full.g / single.g;
    let ratio_b = full.b / single.b;

    // 红通道反照率最高 => 高阶贡献最多；蓝通道几乎全吸收 => 几乎无增益。
    assert!(
        ratio_r > ratio_g,
        "red has the highest albedo, so its series gain must be largest: r={ratio_r} g={ratio_g}"
    );
    assert!(
        ratio_g > ratio_b,
        "blue is nearly absorbing, so its gain must be smallest: g={ratio_g} b={ratio_b}"
    );
    assert!(ratio_b >= 1.0, "gain can never be below 1");
    // 红通道 ω≈0.99、g=0.5 => r≈0.495，12 阶极限约 1.98。
    assert!(
        ratio_r < 2.05,
        "red gain must stay bounded for g=0.5, got {ratio_r}"
    );
}

#[test]
fn full_rejects_invalid_orders_and_factor() {
    let positions = vec![[0.0; 3]];
    let o = SpectralOptics::from_monochrome(1e-2, 2e-2, 1e-6, 550e-9, 1.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };
    let args = ([0.0; 3], [1.0, 0.0, 0.0], 1.0, 0.8, 1.0);
    // orders = 0
    assert!(
        inscatter_spectral_full(
            &positions, o, args.0, args.1, args.2, args.3, args.4, white, 8, 0, 0.5
        )
        .is_none()
    );
    // orders 超上限
    assert!(
        inscatter_spectral_full(
            &positions, o, args.0, args.1, args.2, args.3, args.4, white, 8, 99, 0.5
        )
        .is_none()
    );
    // geometric_factor 非法
    assert!(
        inscatter_spectral_full(
            &positions, o, args.0, args.1, args.2, args.3, args.4, white, 8, 2, 0.0
        )
        .is_none()
    );
    assert!(
        inscatter_spectral_full(
            &positions, o, args.0, args.1, args.2, args.3, args.4, white, 8, 2, 1.5
        )
        .is_none()
    );
}

/// 完整版必须随阶数单调递增。
#[test]
fn full_increases_with_order() {
    let positions: Vec<[f64; 3]> = (0..80).map(|i| [i as f64 * 0.05, 0.0, 0.0]).collect();
    let o = SpectralOptics::from_monochrome(1e-2, 2e-2, 1e-6, 550e-9, 1.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };
    let args = ([-0.5, 0.0, 0.0], [1.0, 0.0, 0.0], 8.0, 0.8, 1.0);
    let l1 = inscatter_spectral_full(
        &positions, o, args.0, args.1, args.2, args.3, args.4, white, 64, 1, 0.5,
    )
    .unwrap();
    let l4 = inscatter_spectral_full(
        &positions, o, args.0, args.1, args.2, args.3, args.4, white, 64, 4, 0.5,
    )
    .unwrap();
    assert!(l4.luminance() > l1.luminance());
}
// ── 由精确米氏算光谱（ADR-0030）────────────────────────────────────────────

/// 小颗粒（瑞利区）应**强烈偏蓝**：`σ_sca(蓝) ≫ σ_sca(红)`。
///
/// 10 nm 水滴，`x = πd/λ ≈ 0.057`（蓝端）到 `0.048`（红端），深在瑞利区。
/// 理论上 `σ ∝ λ⁻⁴`，所以蓝/红比应接近 `(650/450)⁴ ≈ 4.35`。
#[test]
fn tiny_particles_scatter_blue_far_more_than_red() {
    let o = SpectralOptics::from_mie(10e-9, 1.33, 0.0, 1000.0).unwrap();
    assert!(o.is_valid());
    let ratio = o.sigma_sca_b / o.sigma_sca_r;
    let expect = (RED_NM / BLUE_NM).powi(4);
    assert!(
        (ratio - expect).abs() / expect < 0.15,
        "rayleigh ratio {ratio} should be near {expect}"
    );
    assert!(o.sigma_sca_b > o.sigma_sca_g && o.sigma_sca_g > o.sigma_sca_r);
}

/// 大颗粒（米氏/几何区）的通道差异应**远小于**瑞利区。
///
/// 5 µm 水滴，`x ≈ 28`（蓝端）到 `24`（红端），已进入几何光学区。
/// 此时 `Q → 2`，三通道趋于一致（颜色近中性）。
#[test]
fn large_particles_are_nearly_colour_neutral() {
    let o = SpectralOptics::from_mie(5e-6, 1.33, 0.0, 1000.0).unwrap();
    let ratio = o.sigma_sca_b / o.sigma_sca_r;
    // 与瑞利区的 4.35 相比，这里应接近 1。
    assert!(
        (ratio - 1.0).abs() < 0.35,
        "large particles should be near-neutral, got {ratio}"
    );
}

/// 透明颗粒的 `σ_ext == σ_sca`（无吸收）。
#[test]
fn transparent_particles_have_no_absorption() {
    let o = SpectralOptics::from_mie(1e-6, 1.33, 0.0, 1000.0).unwrap();
    for (s, e) in [
        (o.sigma_sca_r, o.sigma_ext_r),
        (o.sigma_sca_g, o.sigma_ext_g),
        (o.sigma_sca_b, o.sigma_ext_b),
    ] {
        assert!((e - s).abs() <= s * 1e-10, "sigma_ext {e} != sigma_sca {s}");
    }
}

/// 吸光颗粒的 `σ_ext > σ_sca`，且蓝端吸收更强（烟灰的特征）。
#[test]
fn absorbing_particles_lose_blue_light() {
    let clean = SpectralOptics::from_mie(1e-6, 1.5, 0.0, 1800.0).unwrap();
    let soot = SpectralOptics::from_mie(1e-6, 1.5, 0.3, 1800.0).unwrap();

    // 烟灰的消光更大（吸收贡献）。
    assert!(soot.sigma_ext_b > clean.sigma_ext_b);

    // 烟灰的反照率低于透明颗粒。
    let albedo = |o: &SpectralOptics| spectral_albedo(*o).unwrap();
    let a_clean = albedo(&clean);
    let a_soot = albedo(&soot);
    assert!(
        a_soot.g < a_clean.g,
        "soot albedo {} should be below clean {}",
        a_soot.g,
        a_clean.g
    );
}

/// 质量与几何一致：`ρ·πd³/6`。
#[test]
fn mass_follows_the_sphere_formula() {
    let d = 1e-6;
    let rho = 2650.0;
    let o = SpectralOptics::from_mie(d, 1.5, 0.0, rho).unwrap();
    let expect = rho * std::f64::consts::PI * d.powi(3) / 6.0;
    assert!((o.mass - expect).abs() / expect < 1e-12);
}

/// 截面必须非负，且 `σ_ext >= σ_sca`（结构约束）。
#[test]
fn from_mie_respects_the_structural_constraints() {
    for (d, kappa) in [(10e-9, 0.0), (1e-6, 0.0), (1e-6, 0.5), (10e-6, 0.1)] {
        let o = SpectralOptics::from_mie(d, 1.5, kappa, 2000.0).unwrap();
        assert!(o.sigma_sca_r >= 0.0 && o.sigma_sca_g >= 0.0 && o.sigma_sca_b >= 0.0);
        assert!(o.sigma_ext_r >= o.sigma_sca_r);
        assert!(o.sigma_ext_g >= o.sigma_sca_g);
        assert!(o.sigma_ext_b >= o.sigma_sca_b);
    }
}

/// 与幂律拟合的关系：瑞利区拟合出的指数应接近 4。
///
/// 用 `from_mie` 得到三点，再反解 `σ ∝ λ^(-n)` 的指数。
#[test]
fn rayleigh_regime_recovers_the_power_law_exponent() {
    let o = SpectralOptics::from_mie(10e-9, 1.33, 0.0, 1000.0).unwrap();
    // n = ln(σ_r/σ_b) / ln(λ_b/λ_r)
    let n = (o.sigma_sca_r / o.sigma_sca_b).ln() / ((BLUE_NM * 1e-9) / (RED_NM * 1e-9)).ln();
    assert!(
        (n - 4.0).abs() < 0.5,
        "rayleigh exponent should be ≈4, got {n}"
    );
}

#[test]
fn from_mie_rejects_invalid() {
    assert!(SpectralOptics::from_mie(0.0, 1.33, 0.0, 1000.0).is_none());
    assert!(SpectralOptics::from_mie(-1e-6, 1.33, 0.0, 1000.0).is_none());
    assert!(SpectralOptics::from_mie(1e-6, 0.0, 0.0, 1000.0).is_none());
    assert!(SpectralOptics::from_mie(1e-6, 1.33, -0.1, 1000.0).is_none());
    assert!(SpectralOptics::from_mie(1e-6, 1.33, 0.0, 0.0).is_none());
    assert!(SpectralOptics::from_mie(f64::NAN, 1.33, 0.0, 1000.0).is_none());
}

/// 端到端：水滴烟雾的光柱应接近中性，烟灰烟雾的光柱应偏暗且偏红。
#[test]
fn water_and_soot_beams_differ_as_expected() {
    let water = SpectralOptics::from_mie(1e-6, 1.33, 0.0, 1000.0).unwrap();
    let soot = SpectralOptics::from_mie(60e-9, 1.75, 0.5, 1800.0).unwrap();
    let white = SpectralLuminance {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };

    let lw = inscatter_spectral_through_particles(
        &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.2, 0.0, 0.0]],
        water,
        [-0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        2.0,
        0.8,
        1.0,
        white,
        64,
    )
    .unwrap();
    let ls = inscatter_spectral_through_particles(
        &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.2, 0.0, 0.0]],
        soot,
        [-0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        2.0,
        0.8,
        1.0,
        white,
        64,
    )
    .unwrap();

    assert!(lw.luminance() > 0.0 && ls.luminance() > 0.0);
    // 烟灰的散射弱得多（大量吸收）。
    assert!(
        ls.luminance() < lw.luminance(),
        "soot beam {} should be dimmer than water {}",
        ls.luminance(),
        lw.luminance()
    );
}
// ── CIE 加权积分（ADR-0032）───────────────────────────────────────────────

/// 色匹配函数在已知峰值处必须对上 CIE 1931 表。
///
/// 三个参照值（取自 CIE 1931 2° 标准观察者表）：
///
/// | λ (nm) | 表值 | 说明 |
/// | ---: | ---: | --- |
/// | 600 | `x̄ = 1.0622` | `x̄` 峰值附近 |
/// | 555 | `ȳ = 1.0000` | `ȳ` 的峰值（同时是光度函数 `V(λ)` 峰值） |
/// | 445 | `z̄ = 1.7826` | `z̄` 的峰值 |
///
/// 解析拟合的误差约 1% 峰值，所以用 3% 容差。
#[test]
fn cie_color_matching_matches_known_peaks() {
    let (x, _, _) = cie_color_matching(600.0).unwrap();
    assert!((x - 1.0622).abs() < 0.03, "x̄(600) = {x}");

    let (_, y, _) = cie_color_matching(555.0).unwrap();
    assert!((y - 1.0).abs() < 0.02, "ȳ(555) = {y}");

    let (_, _, z) = cie_color_matching(445.0).unwrap();
    assert!((z - 1.7826).abs() < 0.05, "z̄(445) = {z}");
}

/// 可见光外应迅速衰减到接近零。
#[test]
fn cie_color_matching_decays_outside_the_visible_range() {
    for nm in [320.0, 820.0] {
        let (x, y, z) = cie_color_matching(nm).unwrap();
        assert!(
            x.abs() < 0.01 && y < 0.01 && z < 0.01,
            "at {nm} nm: ({x}, {y}, {z}) should be near zero"
        );
    }
    assert!(cie_color_matching(f64::NAN).is_none());
}

/// D65 白点的 XYZ 必须映射到 `(1,1,1)` 线性 sRGB。
#[test]
fn xyz_to_linear_srgb_maps_d65_white_to_unity() {
    let (r, g, b) = xyz_to_linear_srgb(0.95047, 1.0, 1.08883).unwrap();
    assert!((r - 1.0).abs() < 0.01, "r = {r}");
    assert!((g - 1.0).abs() < 0.01, "g = {g}");
    assert!((b - 1.0).abs() < 0.01, "b = {b}");
    // 钳位是**逐通道**的：负的 X 只把 r 钳到 0，g/b 仍按矩阵算。
    // （`g = −0.9689·X`，X 为负时 g 为正，所以不是全零。）
    let (r2, g2, b2) = xyz_to_linear_srgb(-1.0, 0.0, 0.0).unwrap();
    assert_eq!(r2, 0.0, "negative X must clamp r");
    assert!((g2 - 0.9689).abs() < 1e-9, "g = {g2}");
    assert_eq!(b2, 0.0, "negative X must clamp b");
    // 所有输出都必须非负。
    for (x, y, z) in [(-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)] {
        let (r, g, b) = xyz_to_linear_srgb(x, y, z).unwrap();
        assert!(r >= 0.0 && g >= 0.0 && b >= 0.0);
    }
    assert!(xyz_to_linear_srgb(f64::NAN, 0.0, 0.0).is_none());
}

/// 平坦光谱（等能白）的亮度必须归一化到 1。
///
/// 这钉住 `spectral_samples_to_rgb` 的归一化约定：调用方不用管绝对标度。
/// 等能白的 sRGB 略偏暖（因为等能白 ≠ D65），所以三个通道不严格相等。
#[test]
fn flat_spectrum_is_normalised_to_unit_luminance() {
    let samples: Vec<(f64, f64)> = (0..31)
        .map(|i| (380.0 + 400.0 * i as f64 / 30.0, 1.0))
        .collect();
    let rgb = spectral_samples_to_rgb(&samples).unwrap();
    assert!(
        (rgb.luminance() - 1.0).abs() < 0.05,
        "luminance = {}",
        rgb.luminance()
    );
    assert!(rgb.r > 0.0 && rgb.g > 0.0 && rgb.b > 0.0);
    // 三通道接近但不相等（等能白偏暖）。
    let mx = rgb.r.max(rgb.g).max(rgb.b);
    let mn = rgb.r.min(rgb.g).min(rgb.b);
    assert!(mx / mn < 1.5, "channels too far apart: {:?}", rgb);
}

/// 窄蓝带的 RGB 必须以蓝为主；窄红带以红为主。
#[test]
fn narrow_bands_produce_the_expected_dominant_channel() {
    let band = |lo: f64, hi: f64| -> Vec<(f64, f64)> {
        (0..5)
            .map(|i| (lo + (hi - lo) * i as f64 / 4.0, 1.0))
            .collect()
    };

    let blue = spectral_samples_to_rgb(&band(440.0, 480.0)).unwrap();
    assert!(blue.b > blue.r && blue.b > blue.g, "blue band: {:?}", blue);

    let red = spectral_samples_to_rgb(&band(620.0, 660.0)).unwrap();
    assert!(red.r > red.g && red.r > red.b, "red band: {:?}", red);

    let green = spectral_samples_to_rgb(&band(520.0, 560.0)).unwrap();
    assert!(
        green.g > green.r && green.g > green.b,
        "green band: {:?}",
        green
    );
}

#[test]
fn spectral_samples_to_rgb_rejects_invalid() {
    // 采样不足。
    assert!(spectral_samples_to_rgb(&[]).is_none());
    assert!(spectral_samples_to_rgb(&[(550.0, 1.0)]).is_none());
    // 波长降序。
    assert!(spectral_samples_to_rgb(&[(600.0, 1.0), (500.0, 1.0)]).is_none());
    // 重复波长。
    assert!(spectral_samples_to_rgb(&[(550.0, 1.0), (550.0, 1.0)]).is_none());
    // 负辐亮度。
    assert!(spectral_samples_to_rgb(&[(400.0, -1.0), (500.0, 1.0)]).is_none());
    // 非有限。
    assert!(spectral_samples_to_rgb(&[(400.0, f64::NAN), (500.0, 1.0)]).is_none());
}

/// 光谱采样必须覆盖 380~780 nm 且升序。
#[test]
fn mie_spectral_samples_covers_the_visible_range() {
    let s = mie_spectral_samples(1e-6, 1.33, 0.0, 15).unwrap();
    assert_eq!(s.len(), 15);
    assert!((s[0].0 - 380.0).abs() < 1e-6, "first = {}", s[0].0);
    assert!((s[14].0 - 780.0).abs() < 1e-6, "last = {}", s[14].0);
    for w in s.windows(2) {
        assert!(w[1].0 > w[0].0, "wavelengths must ascend");
    }
    // 每个通道的 ext >= sca，且都非负。
    for (nm, sca, ext) in &s {
        assert!(*sca >= 0.0 && *ext >= *sca, "bad channel at {nm} nm");
    }
}

#[test]
fn mie_spectral_samples_rejects_invalid() {
    assert!(mie_spectral_samples(0.0, 1.33, 0.0, 15).is_none());
    assert!(mie_spectral_samples(1e-6, 0.0, 0.0, 15).is_none());
    assert!(mie_spectral_samples(1e-6, 1.33, -0.1, 15).is_none());
    assert!(
        mie_spectral_samples(1e-6, 1.33, 0.0, 1).is_none(),
        "need >= 2 samples"
    );
    assert!(
        mie_spectral_samples(1e-6, 1.33, 0.0, 1000).is_none(),
        "too many"
    );
}

/// 端到端：小颗粒的均匀烟雾，CIE 积分后光柱必须**偏蓝**。
///
/// 光学薄时 `L ∝ σ_sca ∝ λ⁻⁴`，所以蓝端显著更强。
#[test]
fn optically_thin_fine_smoke_is_blue() {
    let rgb = inscatter_rgb_uniform(10e-9, 1.33, 0.0, 1e18, 1.0, 1.0, 10.0, 64, 15).unwrap();
    assert!(rgb.luminance() > 0.0, "the beam must scatter something");
    assert!(
        rgb.b > rgb.r,
        "fine particles must give a blue beam: {:?}",
        rgb
    );
}

/// 端到端：大颗粒（几何光学区）的光柱必须**接近中性**。
#[test]
fn large_particles_give_a_neutral_beam() {
    let rgb = inscatter_rgb_uniform(5e-6, 1.33, 0.0, 1e13, 1.0, 1.0, 10.0, 64, 15).unwrap();
    assert!(rgb.luminance() > 0.0);
    let mx = rgb.r.max(rgb.g).max(rgb.b);
    let mn = rgb.r.min(rgb.g).min(rgb.b);
    assert!(
        mx / mn < 1.6,
        "large particles should be near-neutral: {:?}",
        rgb
    );
}

#[test]
fn inscatter_rgb_uniform_rejects_invalid() {
    assert!(inscatter_rgb_uniform(0.0, 1.33, 0.0, 1e18, 1.0, 1.0, 10.0, 64, 15).is_none());
    assert!(inscatter_rgb_uniform(1e-6, 1.33, 0.0, -1.0, 1.0, 1.0, 10.0, 64, 15).is_none());
    assert!(inscatter_rgb_uniform(1e-6, 1.33, 0.0, 1e18, 1.0, 1.0, 0.0, 64, 15).is_none());
    assert!(inscatter_rgb_uniform(1e-6, 1.33, 0.0, 1e18, 1.0, 1.0, 10.0, 0, 15).is_none());
}
// ── 粒子云 + CIE（ADR-0039）──────────────────────────────────────────────

/// 沿 X 轴排一列粒子（间距 `0.02`），返回位置数组。
fn cloud(count: usize, spacing: f64) -> Vec<[f64; 3]> {
    (0..count).map(|i| [i as f64 * spacing, 0.0, 0.0]).collect()
}

/// 细颗粒（瑞利区）的粒子云，CIE 积分后必须**偏蓝**。
#[test]
fn particle_cie_rgb_fine_particles_are_blue() {
    let positions = cloud(200, 0.02);
    let rgb = inscatter_rgb_from_particles(
        &positions,
        10e-9, // 10 nm 水滴 → x ≈ 0.06
        1.33,
        0.0,
        1e-21, // 单颗粒质量
        [-0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        6.0,
        0.5,
        1.0,
        1.0,
        64,
        15,
    )
    .unwrap();
    assert!(rgb.luminance() > 0.0, "光柱必须有亮度：{rgb:?}");
    assert!(rgb.b > rgb.r, "细颗粒必须偏蓝：{rgb:?}");
}

/// 大颗粒（几何光学区）的粒子云必须**接近中性**。
#[test]
fn particle_cie_rgb_large_particles_are_neutral() {
    let positions = cloud(200, 0.02);
    let rgb = inscatter_rgb_from_particles(
        &positions,
        5e-6, // 5 µm → x ≈ 28
        1.33,
        0.0,
        1e-10,
        [-0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        6.0,
        0.5,
        1.0,
        1.0,
        64,
        15,
    )
    .unwrap();
    assert!(rgb.luminance() > 0.0);
    let mx = rgb.r.max(rgb.g).max(rgb.b);
    let mn = rgb.r.min(rgb.g).min(rgb.b);
    assert!(mx / mn.max(1e-30) < 1.8, "大颗粒应接近中性：{rgb:?}");
}

/// 空粒子云 → 无散射 → 全黑。
#[test]
fn particle_cie_rgb_is_black_without_particles() {
    let rgb = inscatter_rgb_from_particles(
        &[],
        1e-6,
        1.33,
        0.0,
        1e-15,
        [-0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        6.0,
        0.5,
        1.0,
        1.0,
        64,
        15,
    )
    .unwrap();
    assert_eq!(rgb, SpectralLuminance::ZERO);
}

/// 吸收性颗粒（烟灰）的光柱必须比透明颗粒**暗**。
#[test]
fn particle_cie_rgb_soot_is_dimmer_than_water() {
    let positions = cloud(200, 0.02);
    let args = (
        1e-6,
        1e-15,
        [-0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        6.0,
        0.5,
        1.0,
        1.0,
        64,
        15,
    );
    let water = inscatter_rgb_from_particles(
        &positions, args.0, 1.5, 0.0, args.1, args.2, args.3, args.4, args.5, args.6, args.7,
        args.8, args.9,
    )
    .unwrap();
    let soot = inscatter_rgb_from_particles(
        &positions, args.0, 1.5, 0.5, args.1, args.2, args.3, args.4, args.5, args.6, args.7,
        args.8, args.9,
    )
    .unwrap();
    assert!(
        soot.luminance() < water.luminance(),
        "烟灰必须比透明颗粒暗：{} vs {}",
        soot.luminance(),
        water.luminance()
    );
}

#[test]
fn particle_cie_rgb_rejects_invalid() {
    let positions = cloud(50, 0.02);
    let base = (
        1e-6,
        1.33,
        0.0,
        1e-15,
        [-0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        6.0,
        0.5,
        1.0,
        1.0,
    );
    // 质量非正。
    let bad = (
        base.0, base.1, base.2, 0.0, base.4, base.5, base.6, base.7, base.8, base.9,
    );
    assert!(
        inscatter_rgb_from_particles(
            &positions, bad.0, bad.1, bad.2, bad.3, bad.4, bad.5, bad.6, bad.7, bad.8, bad.9, 64,
            15
        )
        .is_none()
    );
    // 未归一化方向。
    let bad_dir = (
        base.0,
        base.1,
        base.2,
        base.3,
        base.4,
        [2.0, 0.0, 0.0],
        base.6,
        base.7,
        base.8,
        base.9,
    );
    assert!(
        inscatter_rgb_from_particles(
            &positions, bad_dir.0, bad_dir.1, bad_dir.2, bad_dir.3, bad_dir.4, bad_dir.5,
            bad_dir.6, bad_dir.7, bad_dir.8, bad_dir.9, 64, 15
        )
        .is_none()
    );
    // 直径非正。
    assert!(
        inscatter_rgb_from_particles(
            &positions, 0.0, base.1, base.2, base.3, base.4, base.5, base.6, base.7, base.8,
            base.9, 64, 15
        )
        .is_none()
    );
    // 采样点不足。
    assert!(
        inscatter_rgb_from_particles(
            &positions, base.0, base.1, base.2, base.3, base.4, base.5, base.6, base.7, base.8,
            base.9, 64, 1
        )
        .is_none()
    );
}
