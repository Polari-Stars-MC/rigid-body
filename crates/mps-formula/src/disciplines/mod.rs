//! 学科公开目录模块（按一级学科分类）。
//!
//! 本模块收录 `mps-formula` 公式库覆盖的物理/应用数学一级学科，与
//! `scientists` 模块通过 `field_id` 对齐。每个学科对应 `disciplines/<id>.rs`
//! 一个子文件（存放该学科的 `DISCIPLINE` 常量）。纯数据模块，仅提供只读
//! 访问函数，不依赖 Rapier 或 `WorldHandle`，并经 `mps-core` 重导出供
//! Java/JNI 调用。

/// 一级学科记录。
#[derive(Clone, Copy, Debug)]
pub struct Discipline {
    /// 稳定标识（小写，例如 `"quantum_mechanics"`）。
    pub field_id: &'static str,
    /// 学科中文名（例如 `"量子力学"`）。
    pub name_zh: &'static str,
    /// 学科英文名（例如 `"Quantum Mechanics"`）。
    pub name_en: &'static str,
    /// 父学科 `field_id`（顶级学科填 `""`）。
    pub parent_id: &'static str,
    /// 一句话简介。
    pub summary: &'static str,
    /// 代表性常数/公式（无则空字符串）。
    pub key_symbols: &'static str,
}

/// 各学科的 `Discipline` 常量（按 `field_id` 字典序重导出为
/// `DISCIPLINE_<FIELD_ID Upper>`）。
pub use self::astro::DISCIPLINE as DISCIPLINE_ASTRO;
pub use self::condmat::DISCIPLINE as DISCIPLINE_CONDMAT;
pub use self::electromagnetism::DISCIPLINE as DISCIPLINE_ELECTROMAGNETISM;
pub use self::fluid::DISCIPLINE as DISCIPLINE_FLUID;
pub use self::mathphys::DISCIPLINE as DISCIPLINE_MATHPHYS;
pub use self::mechanics::DISCIPLINE as DISCIPLINE_MECHANICS;
pub use self::nuclear::DISCIPLINE as DISCIPLINE_NUCLEAR;
pub use self::optics::DISCIPLINE as DISCIPLINE_OPTICS;
pub use self::quantum_mechanics::DISCIPLINE as DISCIPLINE_QUANTUM_MECHANICS;
pub use self::relativity::DISCIPLINE as DISCIPLINE_RELATIVITY;
pub use self::statistical::DISCIPLINE as DISCIPLINE_STATISTICAL;

/// 各学科的子模块（每个学科一个 `.rs` 文件）。
#[path = "astro.rs"]
pub mod astro;
#[path = "condmat.rs"]
pub mod condmat;
#[path = "electromagnetism.rs"]
pub mod electromagnetism;
#[path = "fluid.rs"]
pub mod fluid;
#[path = "mathphys.rs"]
pub mod mathphys;
#[path = "mechanics.rs"]
pub mod mechanics;
#[path = "nuclear.rs"]
pub mod nuclear;
#[path = "optics.rs"]
pub mod optics;
#[path = "quantum_mechanics.rs"]
pub mod quantum_mechanics;
#[path = "relativity.rs"]
pub mod relativity;
#[path = "statistical.rs"]
pub mod statistical;

/// 收录的学科记录（按 `field_id` 字典序排列）。
pub static DISCIPLINES: &[Discipline] = &[
    self::astro::DISCIPLINE,
    self::condmat::DISCIPLINE,
    self::electromagnetism::DISCIPLINE,
    self::fluid::DISCIPLINE,
    self::mathphys::DISCIPLINE,
    self::mechanics::DISCIPLINE,
    self::nuclear::DISCIPLINE,
    self::optics::DISCIPLINE,
    self::quantum_mechanics::DISCIPLINE,
    self::relativity::DISCIPLINE,
    self::statistical::DISCIPLINE,
];

/// 顶级学科数量（全部为 `parent_id == ""`）。
pub fn discipline_count() -> usize {
    DISCIPLINES.len()
}

/// 按 `field_id` 精确查找一个学科。返回 `None` 当 `field_id` 为空或不存在。
///
/// 纯函数：不读写线程局部错误槽，失败仅由 `None` 表达（domain.rs 策略）。
pub fn discipline_by_id(field_id: &str) -> Option<&'static Discipline> {
    if field_id.is_empty() {
        return None;
    }
    DISCIPLINES.iter().find(|d| d.field_id == field_id)
}
