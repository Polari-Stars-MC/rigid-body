//! Precision celestial body parameters.
//!
//! Data sources:
//! - JPL DE441 planetary ephemerides (2021)
//! - EGM2008 Earth gravity model (degree 2190, coefficients trimmed to ≤8 for real-time)
//! - LP165P Moon gravity model (degree 165)
//! - IERS Conventions (2010) for fundamental constants
//! - IAU 2015 Resolution B3 for cartographic coordinates
//!
//! Each body exposes: GM, equatorial radius, flattening, rotation rate,
//! zonal harmonics J2–J6, and low-degree spherical-harmonic coefficients
//! for use with [`super::gravitational_models`].

use crate::ffi::{Bool, Vec3};

// ---------------------------------------------------------------------------
// Fundamental constants (CODATA 2018 / IERS 2010)
// ---------------------------------------------------------------------------

/// Newtonian gravitational constant (m³·kg⁻¹·s⁻²) — CODATA 2018
pub const G: f64 = 6.67430e-11;

/// Astronomical Unit (m) — IAU 2012 Resolution B2
pub const AU: f64 = 149_597_870_700.0;

/// Speed of light (m·s⁻¹) — exact (SI 1983)
pub const C: f64 = 299_792_458.0;

// ---------------------------------------------------------------------------
// CelestialBody struct
// ---------------------------------------------------------------------------

/// Precision parameters for a solar-system body.
///
/// `spherical_harmonics_c` and `_s` store lower-triangular packed
/// coefficients C̄ₙₘ and S̄ₙₘ for n=0..max_degree, in row-major order.
/// Index: `n*(n+1)/2 + m`.
#[derive(Clone, Debug)]
pub struct CelestialBody {
    /// Body name (e.g. "Earth")
    pub name: &'static str,
    /// Gravitational parameter GM (m³/s²)
    pub gm: f64,
    /// Equatorial radius (m)
    pub equatorial_radius: f64,
    /// Flattening f = (a-c)/a
    pub flattening: f64,
    /// Siderial rotation rate (rad/s) — affects centrifugal force
    pub rotation_rate: f64,
    /// J2 zonal harmonic (oblateness)
    pub j2: f64,
    /// J3 zonal harmonic (pear-shape)
    pub j3: f64,
    /// J4 zonal harmonic
    pub j4: f64,
    /// J5 zonal harmonic
    pub j5: f64,
    /// J6 zonal harmonic
    pub j6: f64,
    /// Maximum degree of stored spherical harmonics
    pub max_degree: u32,
    /// Packed C̄ₙₘ coefficients (normalized, no C00 term stored)
    pub c_coeffs: &'static [f64],
    /// Packed S̄ₙₘ coefficients (normalized)
    pub s_coeffs: &'static [f64],
    /// Reference radius for spherical harmonics (m)
    pub ref_radius: f64,
    /// Surface atmospheric density at reference altitude (kg/m³), 0 = none
    pub surface_density: f64,
    /// Atmospheric scale height (m), 0 = no atmosphere
    pub scale_height: f64,
    /// Solar radiation pressure coefficient (N/m² at 1 AU)
    /// Effective area × solar constant / c
    pub solar_pressure_constant: f64,
}

impl CelestialBody {
    /// Polar radius (m)
    pub fn polar_radius(&self) -> f64 {
        self.equatorial_radius * (1.0 - self.flattening)
    }

    /// Effective radius at latitude φ (radians from equator)
    pub fn radius_at_latitude(&self, latitude: f64) -> f64 {
        let a = self.equatorial_radius;
        let e2 = self.flattening * (2.0 - self.flattening); // e²
        let sin_lat = latitude.sin();
        a * (1.0 - e2 * sin_lat * sin_lat).sqrt()
    }

    /// Centrifugal potential at a given distance from rotation axis ρ
    /// V_cf = -½ ω² ρ²
    pub fn centrifugal_potential(&self, rho: f64) -> f64 {
        -0.5 * self.rotation_rate * self.rotation_rate * rho * rho
    }

    /// Centrifugal acceleration at a body-fixed position
    pub fn centrifugal_acceleration(&self, position: Vec3) -> Vec3 {
        let w2 = self.rotation_rate * self.rotation_rate;
        Vec3 {
            x: w2 * position.x,
            y: w2 * position.y,
            z: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Earth — EGM2008 coefficients to degree 8
// ---------------------------------------------------------------------------

/// EGM2008 C̄ₙₘ (normalized), degree 2–8, packed: index = n*(n+1)/2 + m
/// Full EGM2008 has degree 2190; this subset captures >99.9% of variance.
static EARTH_C: &[f64] = &[
    // n=2 (index 3..8): C̄₂₀ through S̄₂₂ has 3C + 2S = 5 entries, but C only has n+1 per row
    // Index: row-major packed: C20, C21, C22, C30, C31, C32, C33, ...
    // Row n has n+1 entries (m=0..n)

    // n=2 (m=0..2) — quadrupole
    -0.484165371736e-3, // C̄₂₀ — Earth's oblateness (largest term after GM/r)
    -0.186987635955e-9, // C̄₂₁ — offset of figure axis from rotation axis
    0.243914352398e-5,  // C̄₂₂ — equatorial ellipticity
    // n=3 (m=0..3) — pear-shape (southern hemisphere wider)
    0.957161207093e-6, // C̄₃₀
    0.203046370047e-5, // C̄₃₁
    0.904787894809e-6, // C̄₃₂
    0.721145563610e-6, // C̄₃₃
    // n=4 (m=0..4)
    0.539965866638e-6,  // C̄₄₀
    -0.536157389388e-6, // C̄₄₁
    0.350501623960e-6,  // C̄₄₂
    0.990856766672e-6,  // C̄₄₃
    -0.188560802735e-6, // C̄₄₄
    // n=5 (m=0..5)
    0.685323475630e-7,  // C̄₅₀
    -0.621961686860e-7, // C̄₅₁
    0.652560676220e-6,  // C̄₅₂
    -0.451961963230e-6, // C̄₅₃
    -0.295301647660e-6, // C̄₅₄
    0.174971983200e-6,  // C̄₅₅
    // n=6 (m=0..6)
    0.149837544770e-6,  // C̄₆₀
    -0.643534460108e-7, // C̄₆₁
    0.481701299910e-7,  // C̄₆₂
    0.571806550690e-7,  // C̄₆₃
    -0.860047924280e-7, // C̄₆₄
    -0.267104285480e-6, // C̄₆₅
    0.953019525160e-8,  // C̄₆₆
    // n=7 (m=0..7)
    0.905392187530e-7,  // C̄₇₀
    0.279463044790e-6,  // C̄₇₁
    0.330016563120e-6,  // C̄₇₂
    0.250431748030e-6,  // C̄₇₃
    -0.120410003560e-6, // C̄₇₄
    0.171010984840e-7,  // C̄₇₅
    -0.151646710310e-6, // C̄₇₆
    0.204962220200e-8,  // C̄₇₇
    // n=8 (m=0..8)
    0.496200040420e-7,  // C̄₈₀
    0.232885176830e-7,  // C̄₈₁
    0.802931486830e-7,  // C̄₈₂
    -0.192408232230e-7, // C̄₈₃
    -0.244186670760e-6, // C̄₈₄
    -0.255714750010e-7, // C̄₈₅
    -0.651060001540e-7, // C̄₈₆
    0.671610680030e-7,  // C̄₈₇
    0.124003863830e-6,  // C̄₈₈
];

/// EGM2008 S̄ₙₘ (normalized), degree 2–8, packed
static EARTH_S: &[f64] = &[
    // n=2 (m=0..2) — S₂₀=0 by definition
    0.0, // S̄₂₀=0
    // Row-major packed starting from n=2,m=0:
    // n=2: S20(missing), S21, S22
    // n=3: S30(missing), S31, S32, S33
    // etc.
    // We store 0.0 for the m=0 entries to keep indexing simple.

    // n=2 (1 entry stored: m=1, at index 0; n=2,m=0 is excluded from C coeffs too)

    // S̄₂₁
    0.119528012031e-8,
    // S̄₂₂
    -0.140016683654e-5,
    // n=3
    0.248131151178e-5,  // S̄₃₁
    -0.618954181980e-6, // S̄₃₂
    0.141424658150e-5,  // S̄₃₃
    // n=4
    -0.473567346518e-6, // S̄₄₁
    0.662480026275e-6,  // S̄₄₂
    -0.200956723174e-6, // S̄₄₃
    0.308842122930e-6,  // S̄₄₄
    // n=5
    -0.944023767490e-7, // S̄₅₁
    -0.323342465440e-6, // S̄₅₂
    -0.214858255130e-6, // S̄₅₃
    0.496604264150e-7,  // S̄₅₄
    -0.669250301080e-6, // S̄₅₅
    // n=6
    0.166949386352e-6,  // S̄₆₁
    -0.373828465810e-6, // S̄₆₂
    0.903833650960e-7,  // S̄₆₃
    0.471469239470e-6,  // S̄₆₄
    -0.536488432810e-6, // S̄₆₅
    0.237346636410e-6,  // S̄₆₆
    // n=7
    0.956639264400e-7,  // S̄₇₁
    0.928973017490e-7,  // S̄₇₂
    0.217046688950e-6,  // S̄₇₃
    -0.589855817090e-7, // S̄₇₄
    0.586048085220e-7,  // S̄₇₅
    0.165222862630e-6,  // S̄₇₆
    -0.993975314290e-7, // S̄₇₇
    // n=8
    0.586875657760e-7,  // S̄₈₁
    0.653133976450e-7,  // S̄₈₂
    -0.859303011180e-7, // S̄₈₃
    -0.708973069480e-7, // S̄₈₄
    0.893621588270e-7,  // S̄₈₅
    0.309678712820e-6,  // S̄₈₆
    0.748217893620e-7,  // S̄₈₇
    0.120334437960e-6,  // S̄₈₈
];

/// IERS 2010 / DE441
pub const EARTH_GM: f64 = 3.986004415e14;
pub const EARTH_EQ_RADIUS: f64 = 6_378_136.3; // m
pub const EARTH_POLAR_RADIUS: f64 = 6_356_751.9; // m
pub const EARTH_FLATTENING: f64 = 1.0 / 298.257_222_101;
pub const EARTH_ROTATION_RATE: f64 = 7.292_115_0e-5; // rad/s
pub const EARTH_J2: f64 = 0.001_082_626_683_55;
pub const EARTH_J3: f64 = -2.532_656_485_33e-6;
pub const EARTH_J4: f64 = -1.619_621_591_37e-6;
pub const EARTH_J5: f64 = -2.273_0e-7;
pub const EARTH_J6: f64 = 5.407e-7;

/// Reference surface atmospheric density for NRLMSISE-00 at 0km (kg/m³)
pub const EARTH_SURFACE_DENSITY: f64 = 1.225;
/// Atmospheric scale height near surface (m)
pub const EARTH_SCALE_HEIGHT: f64 = 8_500.0;

/// Solar constant / c (N/m² at 1 AU from Sun)
pub const SOLAR_PRESSURE_AT_1AU: f64 = 4.5605e-6; // N/m²

// ---------------------------------------------------------------------------
// Moon — LP165P primary coefficients
// ---------------------------------------------------------------------------

/// LP165P C̄ₙₘ degree 2–4, packed
static MOON_C: &[f64] = &[
    // n=2
    -0.908698e-4, // C̄₂₀
    0.970e-11,    // C̄₂₁
    0.346561e-4,  // C̄₂₂
    // n=3
    -0.142e-5, // C̄₃₀
    0.269e-4,  // C̄₃₁
    0.142e-4,  // C̄₃₂
    0.126e-4,  // C̄₃₃
    // n=4
    0.640e-5,  // C̄₄₀
    -0.587e-5, // C̄₄₁
    0.102e-5,  // C̄₄₂
    0.778e-5,  // C̄₄₃
    -0.431e-5, // C̄₄₄
];

static MOON_S: &[f64] = &[
    // n=2
    0.0, 0.279e-10,  // S̄₂₁
    -0.815e-12, // S̄₂₂
    // n=3
    0.0, 0.505e-5,  // S̄₃₁
    0.519e-5,  // S̄₃₂
    -0.287e-5, // S̄₃₃
    // n=4
    0.0, 0.158e-4,  // S̄₄₁
    -0.625e-6, // S̄₄₂
    0.125e-5,  // S̄₄₃
    0.107e-6,  // S̄₄₄
];

pub const MOON_GM: f64 = 4.902_800_118e12;
pub const MOON_EQ_RADIUS: f64 = 1_737_400.0;
pub const MOON_FLATTENING: f64 = 1.0 / 400.0; // approximately
pub const MOON_ROTATION_RATE: f64 = 2.661_699_5e-6; // rad/s
pub const MOON_J2: f64 = 2.033_0e-4;
pub const MOON_J3: f64 = -1.00e-5;

// ---------------------------------------------------------------------------
// Mars — Mars50c
// ---------------------------------------------------------------------------

static MARS_C: &[f64] = &[
    // n=2
    -0.874_576_407e-3, // C̄₂₀
    -0.20e-9,          // C̄₂₁
    -0.844_906_405e-4, // C̄₂₂
    // n=3
    0.114_979_620e-4, // C̄₃₀
    0.367_047_1e-5,   // C̄₃₁
    -0.168_783_4e-4,  // C̄₃₂
    0.395_614e-5,     // C̄₃₃
];

static MARS_S: &[f64] = &[
    0.0,
    0.37e-9,          // S̄₂₁
    0.483_091_169e-4, // S̄₂₂
    0.0,
    0.264_210_7e-5,  // S̄₃₁
    -0.142_325_8e-4, // S̄₃₂
    0.135_685e-5,    // S̄₃₃
];

pub const MARS_GM: f64 = 4.282_837_362e13;
pub const MARS_EQ_RADIUS: f64 = 3_396_190.0;
pub const MARS_FLATTENING: f64 = 1.0 / 169.77;
pub const MARS_ROTATION_RATE: f64 = 7.088_218_108e-5; // rad/s
pub const MARS_J2: f64 = 1.960_454e-3;
pub const MARS_J3: f64 = 3.145e-5;

// ---------------------------------------------------------------------------
// Sun
// ---------------------------------------------------------------------------

pub const SUN_GM: f64 = 1.327_124_400_419_393_8e20;
pub const SUN_EQ_RADIUS: f64 = 695_700_000.0;
pub const SUN_FLATTENING: f64 = 9.0e-6; // nearly spherical
pub const SUN_J2: f64 = 2.22e-7;

// ---------------------------------------------------------------------------
// Jupiter
// ---------------------------------------------------------------------------

pub const JUPITER_GM: f64 = 1.266_865_349_218_048e17;
pub const JUPITER_EQ_RADIUS: f64 = 71_492_000.0;
pub const JUPITER_FLATTENING: f64 = 1.0 / 15.41;
pub const JUPITER_ROTATION_RATE: f64 = 1.758_532e-4;
pub const JUPITER_J2: f64 = 1.473_6e-2;
pub const JUPITER_J4: f64 = -5.87e-4;

// ---------------------------------------------------------------------------
// Saturn
// ---------------------------------------------------------------------------

pub const SATURN_GM: f64 = 3.793_120_749_865_088e16;
pub const SATURN_EQ_RADIUS: f64 = 60_268_000.0;
pub const SATURN_FLATTENING: f64 = 1.0 / 10.21;
pub const SATURN_ROTATION_RATE: f64 = 1.637_88e-4;
pub const SATURN_J2: f64 = 1.629_1e-2;
pub const SATURN_J4: f64 = -9.15e-4;

// ---------------------------------------------------------------------------
// Pre-built body instances
// ---------------------------------------------------------------------------

/// Index into the built-in body table (for C FFI)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum CelestialBodyId {
    Sun = 0,
    Mercury = 1,
    Venus = 2,
    Earth = 3,
    Moon = 4,
    Mars = 5,
    Jupiter = 6,
    Saturn = 7,
    Uranus = 8,
    Neptune = 9,
}

/// Convert a raw `u32` body ID to a `CelestialBodyId`, returning `None` for
/// out-of-range values.  Used by FFI entry points instead of `transmute`.
pub fn celestial_body_id_from_u32(value: u32) -> Option<CelestialBodyId> {
    match value {
        0 => Some(CelestialBodyId::Sun),
        1 => Some(CelestialBodyId::Mercury),
        2 => Some(CelestialBodyId::Venus),
        3 => Some(CelestialBodyId::Earth),
        4 => Some(CelestialBodyId::Moon),
        5 => Some(CelestialBodyId::Mars),
        6 => Some(CelestialBodyId::Jupiter),
        7 => Some(CelestialBodyId::Saturn),
        8 => Some(CelestialBodyId::Uranus),
        9 => Some(CelestialBodyId::Neptune),
        _ => None,
    }
}

/// Get a built-in celestial body by ID.
pub fn get_celestial_body(id: CelestialBodyId) -> &'static CelestialBody {
    match id {
        CelestialBodyId::Sun => &SUN,
        CelestialBodyId::Mercury => &MERCURY,
        CelestialBodyId::Venus => &VENUS,
        CelestialBodyId::Earth => &EARTH,
        CelestialBodyId::Moon => &MOON,
        CelestialBodyId::Mars => &MARS,
        CelestialBodyId::Jupiter => &JUPITER,
        CelestialBodyId::Saturn => &SATURN,
        CelestialBodyId::Uranus => &URANUS,
        CelestialBodyId::Neptune => &NEPTUNE,
    }
}

// Pre-built instances
pub static SUN: CelestialBody = CelestialBody {
    name: "Sun",
    gm: SUN_GM,
    equatorial_radius: SUN_EQ_RADIUS,
    flattening: SUN_FLATTENING,
    rotation_rate: 2.865e-6,
    j2: SUN_J2,
    j3: 0.0,
    j4: 0.0,
    j5: 0.0,
    j6: 0.0,
    max_degree: 2,
    c_coeffs: &[],
    s_coeffs: &[],
    ref_radius: SUN_EQ_RADIUS,
    surface_density: 0.0,
    scale_height: 0.0,
    solar_pressure_constant: 0.0, // Sun doesn't receive solar radiation
};

pub static EARTH: CelestialBody = CelestialBody {
    name: "Earth",
    gm: EARTH_GM,
    equatorial_radius: EARTH_EQ_RADIUS,
    flattening: EARTH_FLATTENING,
    rotation_rate: EARTH_ROTATION_RATE,
    j2: EARTH_J2,
    j3: EARTH_J3,
    j4: EARTH_J4,
    j5: EARTH_J5,
    j6: EARTH_J6,
    max_degree: 8,
    c_coeffs: EARTH_C,
    s_coeffs: EARTH_S,
    ref_radius: EARTH_EQ_RADIUS,
    surface_density: EARTH_SURFACE_DENSITY,
    scale_height: EARTH_SCALE_HEIGHT,
    solar_pressure_constant: SOLAR_PRESSURE_AT_1AU,
};

pub static MOON: CelestialBody = CelestialBody {
    name: "Moon",
    gm: MOON_GM,
    equatorial_radius: MOON_EQ_RADIUS,
    flattening: MOON_FLATTENING,
    rotation_rate: MOON_ROTATION_RATE,
    j2: MOON_J2,
    j3: MOON_J3,
    j4: 0.0,
    j5: 0.0,
    j6: 0.0,
    max_degree: 4,
    c_coeffs: MOON_C,
    s_coeffs: MOON_S,
    ref_radius: MOON_EQ_RADIUS,
    surface_density: 0.0,
    scale_height: 0.0,
    solar_pressure_constant: SOLAR_PRESSURE_AT_1AU,
};

pub static MARS: CelestialBody = CelestialBody {
    name: "Mars",
    gm: MARS_GM,
    equatorial_radius: MARS_EQ_RADIUS,
    flattening: MARS_FLATTENING,
    rotation_rate: MARS_ROTATION_RATE,
    j2: MARS_J2,
    j3: MARS_J3,
    j4: 0.0,
    j5: 0.0,
    j6: 0.0,
    max_degree: 3,
    c_coeffs: MARS_C,
    s_coeffs: MARS_S,
    ref_radius: MARS_EQ_RADIUS,
    surface_density: 0.020, // Mars surface density ~0.02 kg/m³
    scale_height: 11_100.0,
    solar_pressure_constant: SOLAR_PRESSURE_AT_1AU,
};

pub static JUPITER: CelestialBody = CelestialBody {
    name: "Jupiter",
    gm: JUPITER_GM,
    equatorial_radius: JUPITER_EQ_RADIUS,
    flattening: JUPITER_FLATTENING,
    rotation_rate: JUPITER_ROTATION_RATE,
    j2: JUPITER_J2,
    j3: 0.0,
    j4: JUPITER_J4,
    j5: 0.0,
    j6: 0.0,
    max_degree: 4,
    c_coeffs: &[],
    s_coeffs: &[],
    ref_radius: JUPITER_EQ_RADIUS,
    surface_density: 0.0,
    scale_height: 0.0,
    solar_pressure_constant: SOLAR_PRESSURE_AT_1AU / (5.2 * 5.2),
};

pub static SATURN: CelestialBody = CelestialBody {
    name: "Saturn",
    gm: SATURN_GM,
    equatorial_radius: SATURN_EQ_RADIUS,
    flattening: SATURN_FLATTENING,
    rotation_rate: SATURN_ROTATION_RATE,
    j2: SATURN_J2,
    j3: 0.0,
    j4: SATURN_J4,
    j5: 0.0,
    j6: 0.0,
    max_degree: 4,
    c_coeffs: &[],
    s_coeffs: &[],
    ref_radius: SATURN_EQ_RADIUS,
    surface_density: 0.0,
    scale_height: 0.0,
    solar_pressure_constant: SOLAR_PRESSURE_AT_1AU / (9.5 * 9.5),
};

// Smaller planets with basic parameters
pub static MERCURY: CelestialBody = CelestialBody {
    name: "Mercury",
    gm: 2.203_178e13,
    equatorial_radius: 2_439_700.0,
    flattening: 0.0,
    rotation_rate: 1.240_0e-6,
    j2: 6.0e-5,
    j3: 0.0,
    j4: 0.0,
    j5: 0.0,
    j6: 0.0,
    max_degree: 2,
    c_coeffs: &[],
    s_coeffs: &[],
    ref_radius: 2_439_700.0,
    surface_density: 0.0,
    scale_height: 0.0,
    solar_pressure_constant: SOLAR_PRESSURE_AT_1AU / (0.387 * 0.387),
};

pub static VENUS: CelestialBody = CelestialBody {
    name: "Venus",
    gm: 3.248_585_920e14,
    equatorial_radius: 6_051_800.0,
    flattening: 0.0,
    rotation_rate: -2.992_5e-7,
    j2: 4.458e-6,
    j3: 0.0,
    j4: 0.0,
    j5: 0.0,
    j6: 0.0,
    max_degree: 2,
    c_coeffs: &[],
    s_coeffs: &[],
    ref_radius: 6_051_800.0,
    surface_density: 65.0,
    scale_height: 15_900.0,
    solar_pressure_constant: SOLAR_PRESSURE_AT_1AU / (0.723 * 0.723),
};

pub static URANUS: CelestialBody = CelestialBody {
    name: "Uranus",
    gm: 5.793_951_322e15,
    equatorial_radius: 25_559_000.0,
    flattening: 1.0 / 43.6,
    rotation_rate: -1.012_37e-4,
    j2: 3.343_43e-3,
    j3: 0.0,
    j4: -2.88e-4,
    j5: 0.0,
    j6: 0.0,
    max_degree: 4,
    c_coeffs: &[],
    s_coeffs: &[],
    ref_radius: 25_559_000.0,
    surface_density: 0.0,
    scale_height: 0.0,
    solar_pressure_constant: SOLAR_PRESSURE_AT_1AU / (19.2 * 19.2),
};

pub static NEPTUNE: CelestialBody = CelestialBody {
    name: "Neptune",
    gm: 6.835_099_97e15,
    equatorial_radius: 24_764_000.0,
    flattening: 1.0 / 58.5,
    rotation_rate: 1.083_4e-4,
    j2: 3.408e-3,
    j3: 0.0,
    j4: -3.34e-4,
    j5: 0.0,
    j6: 0.0,
    max_degree: 4,
    c_coeffs: &[],
    s_coeffs: &[],
    ref_radius: 24_764_000.0,
    surface_density: 0.0,
    scale_height: 0.0,
    solar_pressure_constant: SOLAR_PRESSURE_AT_1AU / (30.1 * 30.1),
};

// ---------------------------------------------------------------------------
// C FFI
// ---------------------------------------------------------------------------

/// Get body parameters by ID.  `body_id` maps to `CelestialBodyId`.
///
/// Returns the packed data:
///   out_gm            — gravitational parameter (m³/s²)
///   out_eq_radius     — equatorial radius (m)
///   out_flattening    — flattening f = (a-c)/a
///   out_rotation_rate — siderial rotation rate (rad/s)
///   out_j2_j6         — [j2, j3, j4, j5, j6] array of 5 f64s
///
/// `max_degree` is returned directly.
/// Returns 0 on success; sets error on invalid ID.
///
/// # Safety
///
/// Every `out_*` pointer may be null, in which case that value is not written;
/// each non-null scalar output must point to writable memory for one element
/// (`f64`, or `u32` for `out_max_degree`), and `out_j2_j6` must point to
/// writable memory for 5 `f64` elements. An invalid `body_id` fails with
/// `ERR_INVALID_ARGUMENT`. No ownership is transferred.
#[unsafe(no_mangle)]
pub extern "C" fn celestial_get_body(
    body_id: u32,
    out_gm: *mut f64,
    out_eq_radius: *mut f64,
    out_flattening: *mut f64,
    out_rotation_rate: *mut f64,
    out_j2_j6: *mut f64,
    out_max_degree: *mut u32,
    out_ref_radius: *mut f64,
    out_surface_density: *mut f64,
    out_scale_height: *mut f64,
) -> Bool {
    let id = match body_id {
        0 => CelestialBodyId::Sun,
        1 => CelestialBodyId::Mercury,
        2 => CelestialBodyId::Venus,
        3 => CelestialBodyId::Earth,
        4 => CelestialBodyId::Moon,
        5 => CelestialBodyId::Mars,
        6 => CelestialBodyId::Jupiter,
        7 => CelestialBodyId::Saturn,
        8 => CelestialBodyId::Uranus,
        9 => CelestialBodyId::Neptune,
        _ => {
            crate::error::set_error(
                crate::error::ERR_INVALID_ARGUMENT,
                "invalid celestial body ID",
            );
            return Bool::FALSE;
        }
    };

    let body = get_celestial_body(id);
    if let Some(p) = unsafe { out_gm.as_mut() } {
        *p = body.gm;
    }
    if let Some(p) = unsafe { out_eq_radius.as_mut() } {
        *p = body.equatorial_radius;
    }
    if let Some(p) = unsafe { out_flattening.as_mut() } {
        *p = body.flattening;
    }
    if let Some(p) = unsafe { out_rotation_rate.as_mut() } {
        *p = body.rotation_rate;
    }
    if let Some(p) = unsafe { out_j2_j6.as_mut() } {
        let arr = unsafe { std::slice::from_raw_parts_mut(p, 5) };
        arr[0] = body.j2;
        arr[1] = body.j3;
        arr[2] = body.j4;
        arr[3] = body.j5;
        arr[4] = body.j6;
    }
    if let Some(p) = unsafe { out_max_degree.as_mut() } {
        *p = body.max_degree;
    }
    if let Some(p) = unsafe { out_ref_radius.as_mut() } {
        *p = body.ref_radius;
    }
    if let Some(p) = unsafe { out_surface_density.as_mut() } {
        *p = body.surface_density;
    }
    if let Some(p) = unsafe { out_scale_height.as_mut() } {
        *p = body.scale_height;
    }

    crate::error::clear_error();
    Bool::TRUE
}

/// Retrieve packed spherical-harmonic coefficients for a body.
///
/// `c_coeffs_out` and `s_coeffs_out` must point to pre-allocated buffers of
/// size `len` (use `celestial_get_sh_coeff_count(body_id)` first).
/// Returns the number of coefficients actually written.
///
/// # Safety
///
/// `c_coeffs_out` and `s_coeffs_out` may each be null, in which case that
/// buffer is not written; each non-null buffer must point to writable memory
/// for `capacity` `f64` elements (only `min(capacity, coefficient count)`
/// elements are written). An invalid `body_id` returns 0. No ownership is
/// transferred.
#[unsafe(no_mangle)]
pub extern "C" fn celestial_get_sh_coeffs(
    body_id: u32,
    c_coeffs_out: *mut f64,
    s_coeffs_out: *mut f64,
    capacity: u32,
) -> u32 {
    let id = match celestial_body_id_from_u32(body_id) {
        Some(id) => id,
        None => return 0,
    };
    let body = get_celestial_body(id);
    let count = capacity.min(body.c_coeffs.len() as u32);
    if !c_coeffs_out.is_null() {
        let Some(dst) = crate::ffi::formula_result(unsafe {
            crate::ffi::checked_output_slice(c_coeffs_out, count as usize, "c_coeffs_out")
        }) else {
            return 0;
        };
        dst.copy_from_slice(&body.c_coeffs[..count as usize]);
    }
    if !s_coeffs_out.is_null() {
        let Some(dst) = crate::ffi::formula_result(unsafe {
            crate::ffi::checked_output_slice(s_coeffs_out, count as usize, "s_coeffs_out")
        }) else {
            return 0;
        };
        dst.copy_from_slice(&body.s_coeffs[..count as usize]);
    }
    count
}

/// Return the number of spherical-harmonic coefficients for a given body.
///
/// # Safety
///
/// No pointers are dereferenced; this function is safe to call with any
/// `body_id` (invalid IDs return 0).
#[unsafe(no_mangle)]
pub extern "C" fn celestial_get_sh_coeff_count(body_id: u32) -> u32 {
    let id = match celestial_body_id_from_u32(body_id) {
        Some(id) => id,
        None => return 0,
    };
    get_celestial_body(id).c_coeffs.len() as u32
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Pluto gravitational parameter GM (m3/s2) - derived from mass 1.303e22 kg x G.
pub const PLUTO_GM: f64 = 8.71e11;

// ---------------------------------------------------------------------------
// Natural satellites (regular moons of the major planets)
// ---------------------------------------------------------------------------

/// A natural satellite (moon) of a major planet.
///
/// Contains the precision data needed to treat the moon as a point-mass
/// gravity source or as an integrated body in a `CosmosWorld`. Values are
/// SI (m, m3/s2, s). `semi_major_axis` and `orbital_period` are measured
/// from the parent planet's centre.
#[derive(Clone, Copy, Debug)]
pub struct Moon {
    /// Moon name (e.g. "Titan").
    pub name: &'static str,
    /// Parent planet name (matches a `CelestialBody.name`, e.g. "Saturn").
    pub parent_planet: &'static str,
    /// Gravitational parameter GM (m3/s2).
    pub gm: f64,
    /// Mean radius (m).
    pub radius: f64,
    /// Semi-major axis of the orbit about the parent planet (m).
    pub semi_major_axis: f64,
    /// Orbital period (s); retrograde orbits use the absolute magnitude.
    pub orbital_period: f64,
}

/// Regular moons of the major planets (NASA Planetary Fact Sheet / JPL).
///
/// Covers the Sun-Earth-Mars-Jupiter-Saturn-Uranus-Neptune-Pluto satellite
/// systems: the Moon, the Martian pair, the four Galilean satellites, the
/// principal Saturnian/Uranian/Neptunian satellites, Triton, Nereid, and
/// Charon. Orbital elements are mean values; individual orbits carry
/// eccentricity/inclination perturbations not modelled here.
pub static MOONS: &[Moon] = &[
    Moon {
        name: "Moon",
        parent_planet: "Earth",
        gm: 4.902_8e12,
        radius: 1.737_4e6,
        semi_major_axis: 3.844_0e8,
        orbital_period: 2.360_59e6,
    },
    Moon {
        name: "Phobos",
        parent_planet: "Mars",
        gm: 7.11e5,
        radius: 1.11e4,
        semi_major_axis: 9.376e6,
        orbital_period: 2.755_4e4,
    },
    Moon {
        name: "Deimos",
        parent_planet: "Mars",
        gm: 9.8e4,
        radius: 6.2e3,
        semi_major_axis: 2.346_3e7,
        orbital_period: 1.090_7e5,
    },
    Moon {
        name: "Io",
        parent_planet: "Jupiter",
        gm: 5.959_9e12,
        radius: 1.821_6e6,
        semi_major_axis: 4.218e8,
        orbital_period: 1.528_4e5,
    },
    Moon {
        name: "Europa",
        parent_planet: "Jupiter",
        gm: 3.202_7e12,
        radius: 1.560_9e6,
        semi_major_axis: 6.711e8,
        orbital_period: 3.068_2e5,
    },
    Moon {
        name: "Ganymede",
        parent_planet: "Jupiter",
        gm: 9.887_8e12,
        radius: 2.631_2e6,
        semi_major_axis: 1.070_4e9,
        orbital_period: 6.181_5e5,
    },
    Moon {
        name: "Callisto",
        parent_planet: "Jupiter",
        gm: 7.179_2e12,
        radius: 2.410_3e6,
        semi_major_axis: 1.882_7e9,
        orbital_period: 1.442_2e6,
    },
    Moon {
        name: "Amalthea",
        parent_planet: "Jupiter",
        gm: 2.3e9,
        radius: 8.35e4,
        semi_major_axis: 1.812e8,
        orbital_period: 4.317e4,
    },
    Moon {
        name: "Titan",
        parent_planet: "Saturn",
        gm: 8.978_14e12,
        radius: 2.574_7e6,
        semi_major_axis: 1.221_87e9,
        orbital_period: 1.377_66e6,
    },
    Moon {
        name: "Enceladus",
        parent_planet: "Saturn",
        gm: 3.8e9,
        radius: 2.52e5,
        semi_major_axis: 2.379_48e8,
        orbital_period: 1.182_9e5,
    },
    Moon {
        name: "Tethys",
        parent_planet: "Saturn",
        gm: 4.12e10,
        radius: 5.315e5,
        semi_major_axis: 2.946_19e8,
        orbital_period: 1.631_2e5,
    },
    Moon {
        name: "Dione",
        parent_planet: "Saturn",
        gm: 7.29e10,
        radius: 5.61e5,
        semi_major_axis: 3.773_96e8,
        orbital_period: 2.364_3e5,
    },
    Moon {
        name: "Rhea",
        parent_planet: "Saturn",
        gm: 1.53e11,
        radius: 7.636e5,
        semi_major_axis: 5.271_08e8,
        orbital_period: 3.903_4e5,
    },
    Moon {
        name: "Iapetus",
        parent_planet: "Saturn",
        gm: 1.20e11,
        radius: 7.34e5,
        semi_major_axis: 3.560_82e9,
        orbital_period: 6.854_1e6,
    },
    Moon {
        name: "Mimas",
        parent_planet: "Saturn",
        gm: 2.5e9,
        radius: 1.98e5,
        semi_major_axis: 1.856e8,
        orbital_period: 8.180_1e4,
    },
    Moon {
        name: "Titania",
        parent_planet: "Uranus",
        gm: 3.43e10,
        radius: 7.89e5,
        semi_major_axis: 4.359_1e8,
        orbital_period: 7.523_1e5,
    },
    Moon {
        name: "Oberon",
        parent_planet: "Uranus",
        gm: 3.23e10,
        radius: 7.61e5,
        semi_major_axis: 5.835_2e8,
        orbital_period: 1.163_3e6,
    },
    Moon {
        name: "Umbriel",
        parent_planet: "Uranus",
        gm: 3.07e10,
        radius: 5.85e5,
        semi_major_axis: 2.663e8,
        orbital_period: 3.579_2e5,
    },
    Moon {
        name: "Ariel",
        parent_planet: "Uranus",
        gm: 1.42e10,
        radius: 5.79e5,
        semi_major_axis: 1.909e8,
        orbital_period: 2.176_3e5,
    },
    Moon {
        name: "Miranda",
        parent_planet: "Uranus",
        gm: 4.19e9,
        radius: 2.36e5,
        semi_major_axis: 1.293_9e8,
        orbital_period: 1.220_5e5,
    },
    Moon {
        name: "Triton",
        parent_planet: "Neptune",
        gm: 1.427_6e12,
        radius: 1.353_4e6,
        semi_major_axis: 3.547_59e8,
        orbital_period: 5.076_9e5,
    },
    Moon {
        name: "Nereid",
        parent_planet: "Neptune",
        gm: 2.0e10,
        radius: 1.7e5,
        semi_major_axis: 5.513_8e9,
        orbital_period: 3.110_6e7,
    },
    Moon {
        name: "Charon",
        parent_planet: "Pluto",
        gm: 1.04e11,
        radius: 6.06e5,
        semi_major_axis: 1.959_1e7,
        orbital_period: 5.518_3e5,
    },
    // Irregular-but-near-circular inner satellites (NASA Planetary Fact Sheet).
    Moon {
        name: "Thebe",
        parent_planet: "Jupiter",
        gm: 1.5e8,
        radius: 4.9e4,
        semi_major_axis: 2.21e8,
        orbital_period: 5.799e4,
    },
    Moon {
        name: "Metis",
        parent_planet: "Jupiter",
        gm: 1.0e8,
        radius: 3.0e4,
        semi_major_axis: 1.28e8,
        orbital_period: 2.556e4,
    },
    Moon {
        name: "Pan",
        parent_planet: "Saturn",
        gm: 1.0e8,
        radius: 1.5e4,
        semi_major_axis: 1.335e8,
        orbital_period: 4.976e4,
    },
    Moon {
        name: "Atlas",
        parent_planet: "Saturn",
        gm: 1.0e7,
        radius: 1.5e4,
        semi_major_axis: 1.375e8,
        orbital_period: 5.203e4,
    },
    Moon {
        name: "Prometheus",
        parent_planet: "Saturn",
        gm: 1.6e9,
        radius: 5.0e4,
        semi_major_axis: 1.394e8,
        orbital_period: 5.312e4,
    },
    Moon {
        name: "Janus",
        parent_planet: "Saturn",
        gm: 5.3e9,
        radius: 9.0e4,
        semi_major_axis: 1.514e8,
        orbital_period: 6.009e4,
    },
    Moon {
        name: "Puck",
        parent_planet: "Uranus",
        gm: 2.0e9,
        radius: 8.1e4,
        semi_major_axis: 8.6e7,
        orbital_period: 6.585e4,
    },
];

#[cfg(test)]
mod moon_tests {
    use super::*;

    /// Parent-planet GM lookup for the Kepler self-consistency check.
    fn planet_gm(name: &str) -> f64 {
        match name {
            "Sun" => SUN_GM,
            "Earth" => EARTH_GM,
            "Mars" => MARS_GM,
            "Jupiter" => JUPITER_GM,
            "Saturn" => SATURN_GM,
            "Uranus" => URANUS.gm,
            "Neptune" => NEPTUNE.gm,
            "Pluto" => PLUTO_GM,
            other => panic!("unknown parent planet in MOONS: {other}"),
        }
    }

    #[test]
    fn moons_table_populated_and_unique() {
        assert!(!MOONS.is_empty(), "MOONS must not be empty");
        let mut seen = std::collections::HashSet::new();
        for m in MOONS {
            assert!(seen.insert(m.name), "duplicate moon name: {}", m.name);
        }
    }

    #[test]
    fn moons_physical_fields_positive_and_orbit_sane() {
        for m in MOONS {
            assert!(m.gm > 0.0, "{} gm must be positive", m.name);
            assert!(m.radius > 0.0, "{} radius must be positive", m.name);
            assert!(m.semi_major_axis > m.radius, "{} orbit inside body", m.name);
            assert!(m.orbital_period > 0.0, "{} period must be positive", m.name);
        }
    }

    #[test]
    fn moons_satisfy_kepler_third_law() {
        for m in MOONS {
            let gm_total = planet_gm(m.parent_planet) + m.gm;
            let t_calc =
                (4.0 * std::f64::consts::PI * std::f64::consts::PI * m.semi_major_axis.powi(3)
                    / gm_total)
                    .sqrt();
            let rel = (t_calc - m.orbital_period).abs() / m.orbital_period;
            assert!(
                rel < 0.12,
                "{} violates Kepler: calc T={:.3e}s obs T={:.3e}s (rel {:.3})",
                m.name,
                t_calc,
                m.orbital_period,
                rel
            );
        }
    }
}
