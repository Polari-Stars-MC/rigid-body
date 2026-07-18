use crate::rapier::ffi::{
    AabbDesc, Bool, MAX_OUTPUT_CAPACITY, MAX_TREE_ENTRIES, RTreeHandle, Vec3,
};
use smallvec::SmallVec;

const MAX_CHILDREN: usize = 8;

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

    fn union(self, other: Self) -> Self {
        Self {
            mins: Vec3 {
                x: self.mins.x.min(other.mins.x),
                y: self.mins.y.min(other.mins.y),
                z: self.mins.z.min(other.mins.z),
            },
            maxs: Vec3 {
                x: self.maxs.x.max(other.maxs.x),
                y: self.maxs.y.max(other.maxs.y),
                z: self.maxs.z.max(other.maxs.z),
            },
        }
    }

    fn intersects(self, other: Self) -> bool {
        self.mins.x <= other.maxs.x
            && self.maxs.x >= other.mins.x
            && self.mins.y <= other.maxs.y
            && self.maxs.y >= other.mins.y
            && self.mins.z <= other.maxs.z
            && self.maxs.z >= other.mins.z
    }

    fn center_axis(self, axis: usize) -> f64 {
        match axis {
            0 => (self.mins.x + self.maxs.x) * 0.5,
            1 => (self.mins.y + self.maxs.y) * 0.5,
            _ => (self.mins.z + self.maxs.z) * 0.5,
        }
    }

    fn extent_axis(self, axis: usize) -> f64 {
        match axis {
            0 => self.maxs.x - self.mins.x,
            1 => self.maxs.y - self.mins.y,
            _ => self.maxs.z - self.mins.z,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Entry {
    id: u64,
    bounds: Aabb,
}

#[derive(Clone, Debug)]
enum NodeKind {
    Leaf(Box<SmallVec<[Entry; MAX_CHILDREN]>>),
    Branch(Vec<Node>),
}

#[derive(Clone, Debug)]
struct Node {
    bounds: Aabb,
    kind: NodeKind,
}

pub(crate) struct RTreeIndex {
    entries: Vec<Entry>,
    root: Option<Node>,
    dirty: bool,
}

impl RTreeIndex {
    fn new() -> Self {
        Self {
            entries: Vec::new(),
            root: None,
            dirty: false,
        }
    }

    fn clear(&mut self) {
        self.entries.clear();
        self.root = None;
        self.dirty = false;
    }

    fn insert(&mut self, id: u64, bounds: Aabb) -> bool {
        if id == 0 {
            return false;
        }

        if let Some(entry) = self.entries.iter_mut().find(|entry| entry.id == id) {
            entry.bounds = bounds;
        } else {
            if self.entries.len() >= MAX_TREE_ENTRIES {
                return false;
            }
            self.entries.push(Entry { id, bounds });
        }
        self.dirty = true;
        true
    }

    fn remove(&mut self, id: u64) -> bool {
        let Some(index) = self.entries.iter().position(|entry| entry.id == id) else {
            return false;
        };
        self.entries.swap_remove(index);
        self.dirty = true;
        true
    }

    fn rebuild_if_needed(&mut self) {
        if !self.dirty {
            return;
        }
        self.root = build_node(&mut self.entries);
        self.dirty = false;
    }

    fn query_count(&mut self, bounds: Aabb) -> u32 {
        self.rebuild_if_needed();
        let Some(root) = &self.root else {
            return 0;
        };
        count_node(root, bounds)
    }

    fn query(&mut self, bounds: Aabb, out_ids: &mut [u64]) -> u32 {
        self.rebuild_if_needed();
        let Some(root) = &self.root else {
            return 0;
        };
        let mut written = 0usize;
        query_node(root, bounds, out_ids, &mut written);
        written as u32
    }
}

fn entries_bounds(entries: &[Entry]) -> Option<Aabb> {
    let mut iter = entries.iter();
    let first = iter.next()?.bounds;
    Some(iter.fold(first, |acc, entry| acc.union(entry.bounds)))
}

fn nodes_bounds(nodes: &[Node]) -> Option<Aabb> {
    let mut iter = nodes.iter();
    let first = iter.next()?.bounds;
    Some(iter.fold(first, |acc, node| acc.union(node.bounds)))
}

fn longest_axis(bounds: Aabb) -> usize {
    let x = bounds.extent_axis(0);
    let y = bounds.extent_axis(1);
    let z = bounds.extent_axis(2);
    if x >= y && x >= z {
        0
    } else if y >= z {
        1
    } else {
        2
    }
}

fn build_node(entries: &mut [Entry]) -> Option<Node> {
    let bounds = entries_bounds(entries)?;
    if entries.len() <= MAX_CHILDREN {
        return Some(Node {
            bounds,
            kind: NodeKind::Leaf(Box::new(entries.iter().copied().collect())),
        });
    }

    let axis = longest_axis(bounds);
    entries.sort_unstable_by(|a, b| {
        a.bounds
            .center_axis(axis)
            .total_cmp(&b.bounds.center_axis(axis))
            .then_with(|| a.id.cmp(&b.id))
    });

    let child_count = entries.len().div_ceil(MAX_CHILDREN);
    let mut children = Vec::with_capacity(child_count);
    for chunk in entries.chunks_mut(MAX_CHILDREN) {
        if let Some(child) = build_node(chunk) {
            children.push(child);
        }
    }

    let bounds = nodes_bounds(&children)?;
    Some(Node {
        bounds,
        kind: NodeKind::Branch(children),
    })
}

fn count_node(node: &Node, bounds: Aabb) -> u32 {
    if !node.bounds.intersects(bounds) {
        return 0;
    }

    match &node.kind {
        NodeKind::Leaf(entries) => entries
            .iter()
            .filter(|entry| entry.bounds.intersects(bounds))
            .count() as u32,
        NodeKind::Branch(children) => children
            .iter()
            .map(|child| count_node(child, bounds))
            .sum::<u32>(),
    }
}

fn query_node(node: &Node, bounds: Aabb, out_ids: &mut [u64], written: &mut usize) {
    if *written >= out_ids.len() || !node.bounds.intersects(bounds) {
        return;
    }

    match &node.kind {
        NodeKind::Leaf(entries) => {
            for entry in entries.iter() {
                if *written >= out_ids.len() {
                    return;
                }
                if entry.bounds.intersects(bounds) {
                    out_ids[*written] = entry.id;
                    *written += 1;
                }
            }
        }
        NodeKind::Branch(children) => {
            for child in children {
                query_node(child, bounds, out_ids, written);
            }
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_create() -> *mut RTreeHandle {
    Box::into_raw(Box::new(RTreeHandle {
        inner: RTreeIndex::new(),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_destroy(tree: *mut RTreeHandle) {
    if tree.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(tree));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_clear(tree: *mut RTreeHandle) {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return;
    };
    tree.inner.clear();
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_len(tree: *const RTreeHandle) -> u32 {
    let Some(tree) = (unsafe { tree.as_ref() }) else {
        return 0;
    };
    tree.inner.entries.len().min(u32::MAX as usize) as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_insert(tree: *mut RTreeHandle, id: u64, aabb: AabbDesc) -> Bool {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return Bool::FALSE;
    };
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return Bool::FALSE;
    };
    tree.inner.insert(id, bounds).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_update(tree: *mut RTreeHandle, id: u64, aabb: AabbDesc) -> Bool {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return Bool::FALSE;
    };
    if !tree.inner.entries.iter().any(|entry| entry.id == id) {
        return Bool::FALSE;
    }
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return Bool::FALSE;
    };
    tree.inner.insert(id, bounds).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_remove(tree: *mut RTreeHandle, id: u64) -> Bool {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return Bool::FALSE;
    };
    tree.inner.remove(id).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_rebuild(tree: *mut RTreeHandle) {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return;
    };
    tree.inner.dirty = true;
    tree.inner.rebuild_if_needed();
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_query_aabb_count(tree: *mut RTreeHandle, aabb: AabbDesc) -> u32 {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
        return 0;
    };
    let Some(bounds) = Aabb::from_desc(aabb) else {
        return 0;
    };
    tree.inner.query_count(bounds)
}

#[unsafe(no_mangle)]
pub extern "C" fn rtree_query_aabb(
    tree: *mut RTreeHandle,
    aabb: AabbDesc,
    out_ids: *mut u64,
    capacity: u32,
) -> u32 {
    let Some(tree) = (unsafe { tree.as_mut() }) else {
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
    fn rtree_queries_intersections() {
        let tree = rtree_create();
        assert!(!tree.is_null());

        assert_eq!(rtree_insert(tree, 10, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(rtree_insert(tree, 20, aabb(2.0, 3.0)), Bool::TRUE);
        assert_eq!(rtree_insert(tree, 30, aabb(4.0, 5.0)), Bool::TRUE);

        assert_eq!(rtree_query_aabb_count(tree, aabb(0.5, 2.5)), 2);

        let mut ids = [0; 4];
        let written = rtree_query_aabb(tree, aabb(0.5, 2.5), ids.as_mut_ptr(), ids.len() as u32);
        assert_eq!(written, 2);
        assert_eq!(&ids[..2], &[10, 20]);

        rtree_destroy(tree);
    }

    #[test]
    fn rtree_update_and_remove() {
        let tree = rtree_create();

        assert_eq!(rtree_insert(tree, 7, aabb(0.0, 1.0)), Bool::TRUE);
        assert_eq!(rtree_update(tree, 7, aabb(10.0, 11.0)), Bool::TRUE);
        assert_eq!(rtree_query_aabb_count(tree, aabb(0.0, 1.0)), 0);
        assert_eq!(rtree_query_aabb_count(tree, aabb(10.5, 10.6)), 1);

        assert_eq!(rtree_remove(tree, 7), Bool::TRUE);
        assert_eq!(rtree_remove(tree, 7), Bool::FALSE);
        assert_eq!(rtree_len(tree), 0);

        rtree_destroy(tree);
    }

    #[test]
    fn rtree_rejects_invalid_bounds() {
        let tree = rtree_create();
        assert_eq!(
            rtree_insert(
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
        assert_eq!(rtree_insert(tree, 0, aabb(0.0, 1.0)), Bool::FALSE);
        rtree_destroy(tree);
    }
}
