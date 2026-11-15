#[cfg(test)]
mod tests {
    use mps_formula::convex_decomposition::*;

    /// Build a closed box mesh (12 triangles) from its min/max corners.
    fn box_mesh(min: [f64; 3], max: [f64; 3]) -> (Vec<[f64; 3]>, Vec<Triangle>) {
        let p = vec![
            [min[0], min[1], min[2]], // 0
            [max[0], min[1], min[2]], // 1
            [max[0], max[1], min[2]], // 2
            [min[0], max[1], min[2]], // 3
            [min[0], min[1], max[2]], // 4
            [max[0], min[1], max[2]], // 5
            [max[0], max[1], max[2]], // 6
            [min[0], max[1], max[2]], // 7
        ];
        let t = vec![
            Triangle { a: 0, b: 2, c: 1 },
            Triangle { a: 0, b: 3, c: 2 },
            Triangle { a: 4, b: 5, c: 6 },
            Triangle { a: 4, b: 6, c: 7 },
            Triangle { a: 0, b: 1, c: 5 },
            Triangle { a: 0, b: 5, c: 4 },
            Triangle { a: 1, b: 2, c: 6 },
            Triangle { a: 1, b: 6, c: 5 },
            Triangle { a: 2, b: 3, c: 7 },
            Triangle { a: 2, b: 7, c: 6 },
            Triangle { a: 3, b: 0, c: 4 },
            Triangle { a: 3, b: 4, c: 7 },
        ];
        (p, t)
    }

    /// An L-shaped prism (concave): a 2x1x1 box unioned with a 1x2x1 box.
    /// Built as two boxes' worth of triangles, which is exactly the concave case
    /// a decomposition must split.
    fn l_shape_mesh() -> (Vec<[f64; 3]>, Vec<Triangle>) {
        let (mut p, mut t) = box_mesh([0.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
        let (p2, t2) = box_mesh([0.0, 1.0, 0.0], [1.0, 3.0, 1.0]);
        let base = p.len() as u32;
        p.extend(p2);
        for tri in t2 {
            t.push(Triangle {
                a: tri.a + base,
                b: tri.b + base,
                c: tri.c + base,
            });
        }
        (p, t)
    }

    // -----------------------------------------------------------------------
    // 输入校验
    // -----------------------------------------------------------------------

    #[test]
    fn rejects_empty_and_invalid_input() {
        let opts = DecomposeOptions::default();
        assert!(decompose(&[], &[], &opts).is_none());
        let (p, t) = box_mesh([0.0; 3], [1.0; 3]);
        assert!(decompose(&p, &[], &opts).is_none());
        // 索引越界
        let bad = vec![Triangle { a: 0, b: 1, c: 99 }];
        assert!(decompose(&p, &bad, &opts).is_none());
        // 退化三角形（两个索引相同）
        let degen = vec![Triangle { a: 0, b: 0, c: 1 }];
        assert!(decompose(&p, &degen, &opts).is_none());
        // 非有限坐标
        let mut nan_p = p.clone();
        nan_p[0][0] = f64::NAN;
        assert!(decompose(&nan_p, &t, &opts).is_none());
    }

    #[test]
    fn rejects_zero_depth_or_zero_parts() {
        let (p, t) = box_mesh([0.0; 3], [1.0; 3]);
        let opts = DecomposeOptions {
            max_depth: 0,
            ..DecomposeOptions::default()
        };
        assert!(decompose(&p, &t, &opts).is_none());
        let opts = DecomposeOptions {
            max_parts: 0,
            ..DecomposeOptions::default()
        };
        assert!(decompose(&p, &t, &opts).is_none());
    }

    // -----------------------------------------------------------------------
    // 凸输入：不应被拆
    // -----------------------------------------------------------------------

    #[test]
    fn convex_box_is_emitted_as_one_part() {
        let (p, t) = box_mesh([0.0; 3], [1.0; 3]);
        let parts = decompose(&p, &t, &DecomposeOptions::default()).unwrap();
        assert_eq!(parts.len(), 1, "a convex box must not be split");
        assert_eq!(parts[0].depth, 0);
        assert_eq!(parts[0].triangle_count, 12);
    }

    #[test]
    fn convex_box_volume_is_preserved() {
        let (p, t) = box_mesh([0.0; 3], [2.0, 3.0, 4.0]);
        let parts = decompose(&p, &t, &DecomposeOptions::default()).unwrap();
        let v = total_volume(&parts).unwrap();
        let expected = 2.0 * 3.0 * 4.0;
        assert!(
            (v - expected).abs() / expected < 1e-9,
            "volume {v} vs expected {expected}"
        );
    }

    // -----------------------------------------------------------------------
    // 凹输入：必须被拆成多块
    // -----------------------------------------------------------------------

    #[test]
    fn concave_l_shape_is_split_into_multiple_parts() {
        let (p, t) = l_shape_mesh();
        let parts = decompose(&p, &t, &DecomposeOptions::default()).unwrap();
        assert!(
            parts.len() >= 2,
            "a concave L-shape must be split, got {} part(s)",
            parts.len()
        );
        // 每一块都必须是真凸的（点数 >= 4 且体积可算）
        for (i, part) in parts.iter().enumerate() {
            assert!(
                part.points.len() >= 4,
                "part {i} has only {} points",
                part.points.len()
            );
        }
    }

    #[test]
    fn concave_l_shape_volume_is_approximately_preserved() {
        let (p, t) = l_shape_mesh();
        let parts = decompose(&p, &t, &DecomposeOptions::default()).unwrap();
        // L 形体由两块不重叠的盒子组成：2*1*1 + 1*2*1 = 4
        // 分解后的凸块体积之和应当接近这个值（允许切分产生的微小重叠）
        let v = total_volume(&parts).unwrap();
        assert!(
            (v - 4.0).abs() / 4.0 < 0.05,
            "decomposed volume {v} differs from the 4.0 input by more than 5%"
        );
    }

    #[test]
    fn max_depth_caps_the_part_count() {
        let (p, t) = l_shape_mesh();
        let opts = DecomposeOptions {
            max_depth: 1,
            ..DecomposeOptions::default()
        };
        let parts = decompose(&p, &t, &opts).unwrap();
        // 深度 1 最多 2 块
        assert!(
            parts.len() <= 2,
            "max_depth=1 produced {} parts",
            parts.len()
        );
    }

    #[test]
    fn max_parts_caps_the_output() {
        let (p, t) = l_shape_mesh();
        let opts = DecomposeOptions {
            max_parts: 2,
            ..DecomposeOptions::default()
        };
        let parts = decompose(&p, &t, &opts).unwrap();
        assert!(
            parts.len() <= 2,
            "max_parts=2 produced {} parts",
            parts.len()
        );
    }

    #[test]
    fn min_extent_stops_splitting_tiny_parts() {
        let (p, t) = l_shape_mesh();
        // 比整个模型还大：第一次就停止
        let opts = DecomposeOptions {
            min_extent: 100.0,
            ..DecomposeOptions::default()
        };
        let parts = decompose(&p, &t, &opts).unwrap();
        assert_eq!(
            parts.len(),
            1,
            "min_extent above the model must stop at once"
        );
    }

    // -----------------------------------------------------------------------
    // 确定性与幂等
    // -----------------------------------------------------------------------

    #[test]
    fn decomposition_is_deterministic() {
        let (p, t) = l_shape_mesh();
        let opts = DecomposeOptions::default();
        let a = decompose(&p, &t, &opts).unwrap();
        let b = decompose(&p, &t, &opts).unwrap();
        assert_eq!(a.len(), b.len());
        for (x, y) in a.iter().zip(b.iter()) {
            assert_eq!(x.points.len(), y.points.len());
            assert_eq!(x.depth, y.depth);
            assert_eq!(x.triangle_count, y.triangle_count);
        }
    }

    #[test]
    fn output_is_sorted_coarse_to_fine() {
        let (p, t) = l_shape_mesh();
        let parts = decompose(&p, &t, &DecomposeOptions::default()).unwrap();
        for w in parts.windows(2) {
            assert!(
                w[0].depth <= w[1].depth,
                "output not sorted by depth: {} then {}",
                w[0].depth,
                w[1].depth
            );
        }
    }

    // -----------------------------------------------------------------------
    // 报告辅助
    // -----------------------------------------------------------------------

    #[test]
    fn convex_part_fraction_is_one_for_a_box() {
        let (p, t) = box_mesh([0.0; 3], [1.0; 3]);
        let parts = decompose(&p, &t, &DecomposeOptions::default()).unwrap();
        assert!((convex_part_fraction(&parts) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn sphere_ratio_reference_is_pi_over_six() {
        let r = sphere_to_bbox_volume_ratio();
        assert!((r - std::f64::consts::PI / 6.0).abs() < 1e-15);
    }

    // -----------------------------------------------------------------------
    // 退化几何：不得死循环
    // -----------------------------------------------------------------------

    #[test]
    fn degenerate_coplanar_input_terminates() {
        // 全部点共面：没有可分的轴，必须原样返回而不是无限递归
        let p = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        let t = vec![Triangle { a: 0, b: 1, c: 2 }, Triangle { a: 0, b: 2, c: 3 }];
        let parts = decompose(&p, &t, &DecomposeOptions::default()).unwrap();
        assert!(!parts.is_empty());
    }
}
