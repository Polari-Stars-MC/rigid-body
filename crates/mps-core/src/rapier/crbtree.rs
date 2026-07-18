use std::collections::BTreeMap;

use crate::rapier::ffi::{
    AabbDesc, Bool, CRbTreeHandle, MAX_OUTPUT_CAPACITY, MAX_TREE_ENTRIES, Vec3,
};

#[derive(Clone, Copy, Debug)]
struct Aabb {
    mins: Vec3,
    maxs: Vec3,
}

impl Aabb {
    fn from_desc(desc: AabbDesc) -> Option<Self> {
        let mins = desc.mins;
        let maxs = desc.maxs;
        if !mins.x.is_finite()
            || !mins.y.is_finite()
            || !mins.z.is_finite()
            || !maxs.x.is_finite()
            || !maxs.y.is_finite()
            || !maxs.z.is_finite()
            || mins.x > maxs.x
            || mins.y > maxs.y
            || mins.z > maxs.z
        {
            return None;
        }

        Some(Self { mins, maxs })
    }

    fn intersects(self, other: Self) -> bool {
        self.mins.x <= other.maxs.x
            && self.maxs.x >= other.mins.x
            && self.mins.y <= other.maxs.y
            && self.maxs.y >= other.mins.y
            && self.mins.z <= other.maxs.z
            && self.maxs.z >= other.mins.z
    }
}

pub(crate) struct CRbTreeIndex {
    entries: BTreeMap<u64, Aabb>,
}

impl CRbTreeIndex {
    fn new() -> Self {
        Self {
            entries: BTreeMap::new(),
        }
    }

    fn insert(&mut self, id: u64, bounds: Aabb) -> bool {
        if id == 0 {
            return false;
        }
        if !self.entries.contains_key(&id) && self.entries.len() >= MAX_TREE_ENTRIES {
            return false;
        }
        self.entries.insert(id, bounds);
        true
    }

    fn query_count(&self, bounds: Aabb) -> u32 {
        self.entries
            .values()
            .filter(|entry| entry.intersects(bounds))
            .count()
            .min(u32::MAX as usize) as u32
    }

    fn query(&self, bounds: Aabb, out_ids: &mut [u64]) -> u32 {
        let mut written = 0usize;
        for (id, entry) in &self.entries {
            if written >= out_ids.len() {
                break;
            }
            if entry.intersects(bounds) {
                out_ids[written] = *id;
                written += 1;
            }
        }
        written as u32
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_create() -> *mut CRbTreeHandle {
    Box::into_raw(Box::new(CRbTreeHandle {
        inner: CRbTreeIndex::new(),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_destroy(tree: *mut CRbTreeHandle) {
    if tree.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(tree));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_clear(tree: *mut CRbTreeHandle) {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return;
    };
    tree.inner.entries.clear();
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_len(tree: *const CRbTreeHandle) -> u32 {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return 0;
    };
    tree.inner.entries.len().min(u32::MAX as usize) as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_insert(tree: *mut CRbTreeHandle, id: u64, aabb: AabbDesc) -> Bool {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return Bool::FALSE;
    };
    tree.inner.insert(id, bounds).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_insert_flag(tree: *mut CRbTreeHandle, id: u64, aabb: AabbDesc) -> u8 {
    crb_tree_insert(tree, id, aabb).0
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_update(tree: *mut CRbTreeHandle, id: u64, aabb: AabbDesc) -> Bool {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return Bool::FALSE;
    };
    if !tree.inner.entries.contains_key(&id) {
        return Bool::FALSE;
    }
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return Bool::FALSE;
    };
    tree.inner.insert(id, bounds).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_remove(tree: *mut CRbTreeHandle, id: u64) -> Bool {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return Bool::FALSE;
    };
    tree.inner.entries.remove(&id).is_some().into()
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_query_aabb_count(tree: *const CRbTreeHandle, aabb: AabbDesc) -> u32 {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return 0;
    };
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return 0;
    };
    tree.inner.query_count(bounds)
}

#[unsafe(no_mangle)]
pub extern "C" fn crb_tree_query_aabb(
    tree: *const CRbTreeHandle,
    aabb: AabbDesc,
    out_ids: *mut u64,
    capacity: u32,
) -> u32 {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return 0;
    };
    if out_ids.is_null() || capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
        return 0;
    }
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return 0;
    };

    let out = unsafe { std::slice::from_raw_parts_mut(out_ids, capacity as usize) };
    tree.inner.query(bounds, out)
}

#[cfg(test)]
mod tests {
    use super::*;

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
