#[cfg(test)]
mod tests {
    use mps_core::rapier::crbtree::*;
    use mps_core::rapier::ffi::*;

    fn aabb(min: f64, max: f64) -> AabbDesc {
        AabbDesc {
            mins: Vec3 {
                x: min,
                y: min,
                z: min,
            },
            maxs: Vec3 {
                x: max,
                y: max,
                z: max,
            },
        }
    }

    #[test]
    fn crb_tree_queries_intersections_in_id_order() {
        let tree = crb_tree_create();
        assert!(!tree.is_null());

        assert_eq!(crb_tree_insert(tree, 20, aabb(2.0, 3.0)), Bool::TRUE);
        assert_eq!(crb_tree_insert(tree, 10, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(crb_tree_insert(tree, 30, aabb(4.0, 5.0)), Bool::TRUE);

        assert_eq!(crb_tree_query_aabb_count(tree, aabb(0.5, 2.5)), 2);

        let mut ids = [0; 4];
        let written = crb_tree_query_aabb(tree, aabb(0.5, 2.5), ids.as_mut_ptr(), ids.len() as u32);
        assert_eq!(written, 2);
        assert_eq!(&ids[..2], &[10, 20]);

        crb_tree_destroy(tree);
    }

    #[test]
    fn crb_tree_update_remove_and_reject_invalid_bounds() {
        let tree = crb_tree_create();

        assert_eq!(crb_tree_insert(tree, 7, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(crb_tree_update(tree, 7, aabb(10.0, 11.0)), Bool::TRUE);
        assert_eq!(crb_tree_query_aabb_count(tree, aabb(0.0, 1.0)), 0);
        assert_eq!(crb_tree_query_aabb_count(tree, aabb(10.5, 10.6)), 1);
        assert_eq!(crb_tree_remove(tree, 7), Bool::TRUE);
        assert_eq!(crb_tree_remove(tree, 7), Bool::FALSE);
        assert_eq!(crb_tree_insert(tree, 0, aabb(0.0, 1.0)), Bool::FALSE);
        assert_eq!(
            crb_tree_insert(
                tree,
                1,
                AabbDesc {
                    mins: Vec3 {
                        x: 1.0,
                        y: 0.0,
                        z: 0.0
                    },
                    maxs: Vec3 {
                        x: 0.0,
                        y: 1.0,
                        z: 1.0
                    },
                }
            ),
            Bool::FALSE
        );

        crb_tree_destroy(tree);
    }
}



