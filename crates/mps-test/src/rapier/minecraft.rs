//! `mps_core::rapier::minecraft` 测试镜像（对应 `crates/mps-core/src/rapier/minecraft.rs`）。

#[cfg(test)]
mod tests {
    use mps_core::rapier::minecraft::{BlockMaterial, MAGIC, MaterialTable, read_header};

    #[test]
    fn magic_is_java_snapshot_marker() {
        // Java 侧按大端写入的 "TSNP"（0x5453_4E50）。
        assert_eq!(MAGIC.to_be_bytes(), *b"TSNP");
    }

    #[test]
    fn header_rejects_bad_magic() {
        let err = read_header(&[0u8; 16]).expect_err("bad magic must be rejected");
        assert!(err.contains("magic"), "unexpected error: {err}");
    }

    #[test]
    fn material_table_maps_by_index() {
        let table = MaterialTable::from_parts(
            vec![BlockMaterial {
                density: 2.5,
                friction: 0.6,
                restitution: 0.1,
            }],
            vec![42],
        );
        let material = table.get(0).expect("material 0");
        assert_eq!(material.density, 2.5);
        assert_eq!(material.friction, 0.6);
        assert!(table.get(1).is_none());
    }
}
