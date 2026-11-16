//! 丁达尔效应的**光谱**版本测试（ADR-0020）。
//!
//! 核心断言是那条物理规律：**厚介质里蓝光先被散射掉，光柱偏红**。
//! 这与日落同源，也是分通道版本相对标量版本的唯一视觉价值。

use mps_formula::tyndall_spectral::{
    BLUE_NM, GREEN_NM, RED_NM, SpectralLuminance, SpectralOptics, inscatter_spectral,
    spectral_albedo, spectral_transmittance, wavelength_to_rgb,
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
