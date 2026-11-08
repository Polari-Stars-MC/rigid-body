//! 科学家目录模块（每个科学家一个子文件，公式实现内迁）。
//!
//! 与 `disciplines` 模块通过 `field_id` 对齐；每位科学家的元数据与公式
//! 实现放在 `scientists/<id>.rs`（原域模块仅 `pub use` 重导出）。
//! 纯数据 + 纯函数，不依赖 Rapier / `WorldHandle`。

/// 一级学科标识类型（与 `disciplines` 模块一致）。
pub type FieldId = &'static str;

/// 科学家贡献记录。
#[derive(Clone, Copy, Debug)]
pub struct ScientistRecord {
    /// 稳定 `id`（小写，空格用下划线）。
    pub id: &'static str,
    /// 姓名（原文）。
    pub name: &'static str,
    /// 出生年份（公元前用负数；`None` 表示未知）。
    pub birth_year: Option<i32>,
    /// 逝世年份（`None` 表示在世或未知）。
    pub death_year: Option<i32>,
    /// 一级学科 `field_id`（与 `disciplines` 模块对齐）。
    pub field_id: FieldId,
    /// 国籍/地区。
    pub nationality: &'static str,
    /// 一句话贡献摘要。
    pub contribution: &'static str,
    /// 代表性常数或公式（无则空字符串）。
    pub key_constants: &'static str,
}

/// 各科学家的 `ScientistRecord` 常量（按 `id` 字典序重导出为 `SCIENTIST_<ID>`）。
pub use self::albert_einstein::SCIENTIST as SCIENTIST_ALBERT_EINSTEIN;
pub use self::aleksandr_lyapunov::SCIENTIST as SCIENTIST_ALEKSANDR_LYAPUNOV;
pub use self::andre_marie_ampere::SCIENTIST as SCIENTIST_ANDRE_MARIE_AMPERE;
pub use self::archimedes::SCIENTIST as SCIENTIST_ARCHIMEDES;
pub use self::arthur_eddington::SCIENTIST as SCIENTIST_ARTHUR_EDDINGTON;
pub use self::augustin_fresnel::SCIENTIST as SCIENTIST_AUGUSTIN_FRESNEL;
pub use self::bernhard_riemann::SCIENTIST as SCIENTIST_BERNHARD_RIEMANN;
pub use self::carl_friedrich_gauss::SCIENTIST as SCIENTIST_CARL_FRIEDRICH_GAUSS;
pub use self::charles_augustin_de_coulomb::SCIENTIST as SCIENTIST_CHARLES_AUGUSTIN_DE_COULOMB;
pub use self::christiaan_huygens::SCIENTIST as SCIENTIST_CHRISTIAAN_HUYGENS;
pub use self::claude_louis_navier::SCIENTIST as SCIENTIST_CLAUDE_LOUIS_NAVIER;
pub use self::daniel_bernoulli::SCIENTIST as SCIENTIST_DANIEL_BERNOULLI;
pub use self::edward_lorenz::SCIENTIST as SCIENTIST_EDWARD_LORENZ;
pub use self::edwin_hubble::SCIENTIST as SCIENTIST_EDWIN_HUBBLE;
pub use self::enrico_fermi::SCIENTIST as SCIENTIST_ENRICO_FERMI;
pub use self::ernest_rutherford::SCIENTIST as SCIENTIST_ERNEST_RUTHERFORD;
pub use self::ernst_chladni::SCIENTIST as SCIENTIST_ERNST_CHLADNI;
pub use self::ernst_mach::SCIENTIST as SCIENTIST_ERNST_MACH;
pub use self::erwin_schrodinger::SCIENTIST as SCIENTIST_ERWIN_SCHRODINGER;
pub use self::galileo_galilei::SCIENTIST as SCIENTIST_GALILEO_GALILEI;
pub use self::georg_ohm::SCIENTIST as SCIENTIST_GEORG_OHM;
pub use self::george_stokes::SCIENTIST as SCIENTIST_GEORGE_STOKES;
pub use self::gottfried_leibniz::SCIENTIST as SCIENTIST_GOTTFRIED_LEIBNIZ;
pub use self::hans_christian_orsted::SCIENTIST as SCIENTIST_HANS_CHRISTIAN_ORSTED;
pub use self::heinrich_hertz::SCIENTIST as SCIENTIST_HEINRICH_HERTZ;
pub use self::hendrik_lorentz::SCIENTIST as SCIENTIST_HENDRIK_LORENTZ;
pub use self::henri_poincare::SCIENTIST as SCIENTIST_HENRI_POINCARE;
pub use self::hermann_von_helmholtz::SCIENTIST as SCIENTIST_HERMANN_VON_HELMHOLTZ;
pub use self::hideki_yukawa::SCIENTIST as SCIENTIST_HIDEKI_YUKAWA;
pub use self::isaac_newton::SCIENTIST as SCIENTIST_ISAAC_NEWTON;
pub use self::james_chadwick::SCIENTIST as SCIENTIST_JAMES_CHADWICK;
pub use self::james_clerk_maxwell::SCIENTIST as SCIENTIST_JAMES_CLERK_MAXWELL;
pub use self::james_thomson::SCIENTIST as SCIENTIST_JAMES_THOMSON;
pub use self::james_watt::SCIENTIST as SCIENTIST_JAMES_WATT;
pub use self::jean_le_rond_dalembert::SCIENTIST as SCIENTIST_JEAN_LE_ROND_DALEMBERT;
pub use self::johannes_kepler::SCIENTIST as SCIENTIST_JOHANNES_KEPLER;
pub use self::joseph_fourier::SCIENTIST as SCIENTIST_JOSEPH_FOURIER;
pub use self::joseph_louis_lagrange::SCIENTIST as SCIENTIST_JOSEPH_LOUIS_LAGRANGE;
pub use self::karl_schwarzschild::SCIENTIST as SCIENTIST_KARL_SCHWARZSCHILD;
pub use self::leonhard_euler::SCIENTIST as SCIENTIST_LEONHARD_EULER;
pub use self::lev_landau::SCIENTIST as SCIENTIST_LEV_LANDAU;
pub use self::lord_kelvin::SCIENTIST as SCIENTIST_LORD_KELVIN;
pub use self::lord_rayleigh::SCIENTIST as SCIENTIST_LORD_RAYLEIGH;
pub use self::louis_de_broglie::SCIENTIST as SCIENTIST_LOUIS_DE_BROGLIE;
pub use self::ludwig_boltzmann::SCIENTIST as SCIENTIST_LUDWIG_BOLTZMANN;
pub use self::ludwig_prandtl::SCIENTIST as SCIENTIST_LUDWIG_PRANDTL;
pub use self::marie_curie::SCIENTIST as SCIENTIST_MARIE_CURIE;
pub use self::max_born::SCIENTIST as SCIENTIST_MAX_BORN;
pub use self::max_planck::SCIENTIST as SCIENTIST_MAX_PLANCK;
pub use self::michael_faraday::SCIENTIST as SCIENTIST_MICHAEL_FARADAY;
pub use self::mitchell_feigenbaum::SCIENTIST as SCIENTIST_MITCHELL_FEIGENBAUM;
pub use self::niels_bohr::SCIENTIST as SCIENTIST_NIELS_BOHR;
pub use self::osborne_reynolds::SCIENTIST as SCIENTIST_OSBORNE_REYNOLDS;
pub use self::paul_dirac::SCIENTIST as SCIENTIST_PAUL_DIRAC;
pub use self::peter_debye::SCIENTIST as SCIENTIST_PETER_DEBYE;
pub use self::pierre_simon_laplace::SCIENTIST as SCIENTIST_PIERRE_SIMON_LAPLACE;
pub use self::richard_feynman::SCIENTIST as SCIENTIST_RICHARD_FEYNMAN;
pub use self::rudolf_clausius::SCIENTIST as SCIENTIST_RUDOLF_CLAUSIUS;
pub use self::sadi_carnot::SCIENTIST as SCIENTIST_SADI_CARNOT;
pub use self::satyendra_nath_bose::SCIENTIST as SCIENTIST_SATYENDRA_NATH_BOSE;
pub use self::theodore_von_karman::SCIENTIST as SCIENTIST_THEODORE_VON_KARMAN;
pub use self::thomas_young::SCIENTIST as SCIENTIST_THOMAS_YOUNG;
pub use self::werner_heisenberg::SCIENTIST as SCIENTIST_WERNER_HEISENBERG;
pub use self::willard_gibbs::SCIENTIST as SCIENTIST_WILLARD_GIBBS;
pub use self::william_rowan_hamilton::SCIENTIST as SCIENTIST_WILLIAM_ROWAN_HAMILTON;
pub use self::wolfgang_pauli::SCIENTIST as SCIENTIST_WOLFGANG_PAULI;

/// 各科学家的子模块（元数据 + `formulas` 实现）。
#[path = "albert_einstein.rs"]
pub mod albert_einstein;
#[path = "aleksandr_lyapunov.rs"]
pub mod aleksandr_lyapunov;
#[path = "andre_marie_ampere.rs"]
pub mod andre_marie_ampere;
#[path = "archimedes.rs"]
pub mod archimedes;
#[path = "arthur_eddington.rs"]
pub mod arthur_eddington;
#[path = "augustin_fresnel.rs"]
pub mod augustin_fresnel;
#[path = "bernhard_riemann.rs"]
pub mod bernhard_riemann;
#[path = "carl_friedrich_gauss.rs"]
pub mod carl_friedrich_gauss;
#[path = "charles_augustin_de_coulomb.rs"]
pub mod charles_augustin_de_coulomb;
#[path = "chen_ning_yang.rs"]
pub mod chen_ning_yang;
#[path = "christiaan_huygens.rs"]
pub mod christiaan_huygens;
#[path = "claude_louis_navier.rs"]
pub mod claude_louis_navier;
#[path = "daniel_bernoulli.rs"]
pub mod daniel_bernoulli;
#[path = "edward_lorenz.rs"]
pub mod edward_lorenz;
#[path = "edwin_hubble.rs"]
pub mod edwin_hubble;
#[path = "enrico_fermi.rs"]
pub mod enrico_fermi;
#[path = "ernest_rutherford.rs"]
pub mod ernest_rutherford;
#[path = "ernst_chladni.rs"]
pub mod ernst_chladni;
#[path = "ernst_mach.rs"]
pub mod ernst_mach;
#[path = "erwin_schrodinger.rs"]
pub mod erwin_schrodinger;
#[path = "galileo_galilei.rs"]
pub mod galileo_galilei;
#[path = "georg_ohm.rs"]
pub mod georg_ohm;
#[path = "george_stokes.rs"]
pub mod george_stokes;
#[path = "gottfried_leibniz.rs"]
pub mod gottfried_leibniz;
#[path = "hans_christian_orsted.rs"]
pub mod hans_christian_orsted;
#[path = "heinrich_hertz.rs"]
pub mod heinrich_hertz;
#[path = "hendrik_lorentz.rs"]
pub mod hendrik_lorentz;
#[path = "henri_poincare.rs"]
pub mod henri_poincare;
#[path = "hermann_von_helmholtz.rs"]
pub mod hermann_von_helmholtz;
#[path = "hideki_yukawa.rs"]
pub mod hideki_yukawa;
#[path = "isaac_newton.rs"]
pub mod isaac_newton;
#[path = "james_chadwick.rs"]
pub mod james_chadwick;
#[path = "james_clerk_maxwell.rs"]
pub mod james_clerk_maxwell;
#[path = "james_thomson.rs"]
pub mod james_thomson;
#[path = "james_watt.rs"]
pub mod james_watt;
#[path = "jean_le_rond_dalembert.rs"]
pub mod jean_le_rond_dalembert;
#[path = "johannes_kepler.rs"]
pub mod johannes_kepler;
#[path = "joseph_fourier.rs"]
pub mod joseph_fourier;
#[path = "joseph_louis_lagrange.rs"]
pub mod joseph_louis_lagrange;
#[path = "karl_schwarzschild.rs"]
pub mod karl_schwarzschild;
#[path = "leonhard_euler.rs"]
pub mod leonhard_euler;
#[path = "lev_landau.rs"]
pub mod lev_landau;
#[path = "lord_kelvin.rs"]
pub mod lord_kelvin;
#[path = "lord_rayleigh.rs"]
pub mod lord_rayleigh;
#[path = "louis_de_broglie.rs"]
pub mod louis_de_broglie;
#[path = "ludwig_boltzmann.rs"]
pub mod ludwig_boltzmann;
#[path = "ludwig_prandtl.rs"]
pub mod ludwig_prandtl;
#[path = "marie_curie.rs"]
pub mod marie_curie;
#[path = "max_born.rs"]
pub mod max_born;
#[path = "max_planck.rs"]
pub mod max_planck;
#[path = "michael_faraday.rs"]
pub mod michael_faraday;
#[path = "mitchell_feigenbaum.rs"]
pub mod mitchell_feigenbaum;
#[path = "niels_bohr.rs"]
pub mod niels_bohr;
#[path = "osborne_reynolds.rs"]
pub mod osborne_reynolds;
#[path = "other.rs"]
pub mod other;
#[path = "paul_dirac.rs"]
pub mod paul_dirac;
#[path = "peter_debye.rs"]
pub mod peter_debye;
#[path = "pierre_simon_laplace.rs"]
pub mod pierre_simon_laplace;
#[path = "richard_feynman.rs"]
pub mod richard_feynman;
#[path = "rudolf_clausius.rs"]
pub mod rudolf_clausius;
#[path = "sadi_carnot.rs"]
pub mod sadi_carnot;
#[path = "satyendra_nath_bose.rs"]
pub mod satyendra_nath_bose;
#[path = "theodore_von_karman.rs"]
pub mod theodore_von_karman;
#[path = "thomas_young.rs"]
pub mod thomas_young;
#[path = "werner_heisenberg.rs"]
pub mod werner_heisenberg;
#[path = "willard_gibbs.rs"]
pub mod willard_gibbs;
#[path = "william_rowan_hamilton.rs"]
pub mod william_rowan_hamilton;
#[path = "wolfgang_pauli.rs"]
pub mod wolfgang_pauli;

/// 收录的科学家总数。
pub fn scientist_count() -> usize {
    66
}

/// 收录的所有科学家记录（按 `id` 字典序）。
pub static SCIENTISTS: &[ScientistRecord] = &[
    self::albert_einstein::SCIENTIST,
    self::edwin_hubble::SCIENTIST,
    self::aleksandr_lyapunov::SCIENTIST,
    self::andre_marie_ampere::SCIENTIST,
    self::archimedes::SCIENTIST,
    self::arthur_eddington::SCIENTIST,
    self::augustin_fresnel::SCIENTIST,
    self::bernhard_riemann::SCIENTIST,
    self::carl_friedrich_gauss::SCIENTIST,
    self::charles_augustin_de_coulomb::SCIENTIST,
    self::christiaan_huygens::SCIENTIST,
    self::claude_louis_navier::SCIENTIST,
    self::daniel_bernoulli::SCIENTIST,
    self::edward_lorenz::SCIENTIST,
    self::enrico_fermi::SCIENTIST,
    self::ernest_rutherford::SCIENTIST,
    self::ernst_chladni::SCIENTIST,
    self::ernst_mach::SCIENTIST,
    self::erwin_schrodinger::SCIENTIST,
    self::galileo_galilei::SCIENTIST,
    self::georg_ohm::SCIENTIST,
    self::gottfried_leibniz::SCIENTIST,
    self::george_stokes::SCIENTIST,
    self::hans_christian_orsted::SCIENTIST,
    self::heinrich_hertz::SCIENTIST,
    self::hendrik_lorentz::SCIENTIST,
    self::henri_poincare::SCIENTIST,
    self::hermann_von_helmholtz::SCIENTIST,
    self::hideki_yukawa::SCIENTIST,
    self::isaac_newton::SCIENTIST,
    self::james_chadwick::SCIENTIST,
    self::james_clerk_maxwell::SCIENTIST,
    self::james_thomson::SCIENTIST,
    self::james_watt::SCIENTIST,
    self::jean_le_rond_dalembert::SCIENTIST,
    self::johannes_kepler::SCIENTIST,
    self::karl_schwarzschild::SCIENTIST,
    self::joseph_fourier::SCIENTIST,
    self::joseph_louis_lagrange::SCIENTIST,
    self::leonhard_euler::SCIENTIST,
    self::lev_landau::SCIENTIST,
    self::lord_kelvin::SCIENTIST,
    self::lord_rayleigh::SCIENTIST,
    self::louis_de_broglie::SCIENTIST,
    self::ludwig_boltzmann::SCIENTIST,
    self::ludwig_prandtl::SCIENTIST,
    self::marie_curie::SCIENTIST,
    self::max_born::SCIENTIST,
    self::max_planck::SCIENTIST,
    self::mitchell_feigenbaum::SCIENTIST,
    self::michael_faraday::SCIENTIST,
    self::niels_bohr::SCIENTIST,
    self::osborne_reynolds::SCIENTIST,
    self::paul_dirac::SCIENTIST,
    self::peter_debye::SCIENTIST,
    self::pierre_simon_laplace::SCIENTIST,
    self::richard_feynman::SCIENTIST,
    self::rudolf_clausius::SCIENTIST,
    self::sadi_carnot::SCIENTIST,
    self::satyendra_nath_bose::SCIENTIST,
    self::werner_heisenberg::SCIENTIST,
    self::willard_gibbs::SCIENTIST,
    self::theodore_von_karman::SCIENTIST,
    self::thomas_young::SCIENTIST,
    self::william_rowan_hamilton::SCIENTIST,
    self::wolfgang_pauli::SCIENTIST,
];

/// 按 `id` 精确查找一位科学家。空/未知返回 `None`。
///
/// 纯函数：不读写线程局部错误槽，失败仅由 `None` 表达（domain.rs 策略）。
pub fn scientist_by_id(id: &str) -> Option<&'static ScientistRecord> {
    if id.is_empty() {
        return None;
    }
    SCIENTISTS.iter().find(|s| s.id == id)
}

/// 按一级 `field_id` 过滤，返回匹配科学家的切片。
///
/// 纯函数：不读写线程局部错误槽，空 `field_id` 返回空切片。
pub fn scientists_by_field(field_id: &str) -> Vec<&'static ScientistRecord> {
    if field_id.is_empty() {
        return Vec::new();
    }
    SCIENTISTS
        .iter()
        .filter(|s| s.field_id == field_id)
        .collect()
}

/// 指定学科下的科学家人数。
pub fn scientist_count_by_field(field_id: &str) -> usize {
    scientists_by_field(field_id).len()
}

/// 返回给定公式模块相关的科学家 `id` 列表。
pub fn scientists_for_module(module: &str) -> Vec<&'static str> {
    let fields: &[&str] = match module {
        "acoustics" | "wave_optics" => &["optics"],
        "aerodynamics" | "fluid" => &["fluid"],
        "astrophysics"
        | "cosmology"
        | "galactic_dynamics"
        | "high_energy_astro"
        | "stellar"
        | "planetary_science"
        | "heliophysics"
        | "gravitational_models" => &["astro"],
        "biomechanics" | "softbody" | "material_mechanics" => &["mechanics"],
        "chaos" | "control_theory" | "integrators" | "trajectory" | "topology" => &["mathphys"],
        "continuum" => &["fluid", "mechanics"],
        "electromagnetism" | "transmission" => &["electromagnetism"],
        "math" => &["mathphys"],
        "molecular" | "physchem" => &["condmat"],
        "nuclear" => &["nuclear"],
        "plasma" | "superfluidity" => &["condmat", "quantum_mechanics"],
        "quantum" => &["quantum_mechanics"],
        "relativity" => &["relativity"],
        "thermodynamics" => &["statistical"],
        _ => return Vec::new(),
    };
    let mut out: Vec<&str> = SCIENTISTS
        .iter()
        .filter(|s| fields.contains(&s.field_id))
        .map(|s| s.id)
        .collect();
    out.sort_unstable();
    out
}
