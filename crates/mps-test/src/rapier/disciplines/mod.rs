//! 学科目录模块镜像测试。
//!
//! 镜像 `mps-formula/src/disciplines`：验证目录数据、查询 API 与跨模块
//! `field_id` 对齐契约（与 `scientists` 模块一致）。
#[cfg(test)]
mod tests {
    use mps_formula::disciplines::*;

    #[test]
    fn quantum_mechanics_present() {
        let qm = discipline_by_id("quantum_mechanics").expect("should exist");
        assert_eq!(qm.name_zh, "量子力学");
        assert_eq!(qm.name_en, "Quantum Mechanics");
        assert!(qm.parent_id.is_empty());
    }

    #[test]
    fn electromagnetism_translated() {
        let em = discipline_by_id("electromagnetism").expect("should exist");
        assert_eq!(em.name_zh, "电磁力学");
        assert!(em.summary.contains("Maxwell"));
    }

    #[test]
    fn rejects_empty_field_id() {
        assert!(discipline_by_id("").is_none());
    }

    #[test]
    fn all_top_level_disciplines() {
        // 11 个一级学科，均为顶级（parent_id == ""）。
        assert_eq!(discipline_count(), 11);
        for d in DISCIPLINES {
            assert_eq!(d.parent_id, "");
        }
    }

    #[test]
    fn discipline_field_ids_are_unique() {
        let mut seen = std::collections::HashSet::new();
        for d in DISCIPLINES {
            assert!(
                seen.insert(d.field_id),
                "duplicate field_id: {}",
                d.field_id
            );
        }
    }
}
