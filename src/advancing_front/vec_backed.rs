use super::*;
use crate::shape::InnerTriangle;
use crate::{points::Points, shape::Point, triangles::TriangleId, PointId};

/// Advancing front, stores all advancing edges in a btree, this makes store compact
/// and easier to update
pub struct AdvancingFront {
    nodes: Vec<Entry>,
    /// In my local test, hit rate is about 40%
    access_cache: Option<(PointKey, usize)>,
    #[cfg(test)]
    pub miss_count: std::sync::atomic::AtomicU64,
    #[cfg(test)]
    pub hit_count: std::sync::atomic::AtomicU64,
}

struct Entry {
    key: PointKey,
    node: NodeInner,
}

impl Entry {
    fn new(key: PointKey, node: NodeInner) -> Self {
        Self { key, node }
    }

    fn point(&self) -> Point {
        self.key.point()
    }

    fn to_node<'a>(&self, index: usize, af: &'a AdvancingFront) -> NodeRef<'a> {
        NodeRef {
            point_id: self.node.point_id,
            point: self.point(),
            triangle: self.node.triangle.into_option(),
            index,
            advancing_front: af,
        }
    }
}

/// New type to wrap `Point` as Node's key
#[derive(Debug, Clone, Copy)]
struct PointKey(Point);

impl PartialEq for PointKey {
    fn eq(&self, other: &Self) -> bool {
        self.0.x.eq(&other.0.x) && self.0.y.eq(&other.0.y)
    }
}

impl Eq for PointKey {}

impl PartialOrd for PointKey {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        match self.0.x.partial_cmp(&other.0.x) {
            None | Some(Ordering::Equal) => self.0.y.partial_cmp(&other.0.y),
            x_order => {
                return x_order;
            }
        }
    }
}

impl Ord for PointKey {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap_or(Ordering::Equal)
    }
}

impl From<Point> for PointKey {
    fn from(value: Point) -> Self {
        Self(value)
    }
}

impl PointKey {
    /// clone the point
    fn point(&self) -> Point {
        self.0
    }
}

struct NodeInner {
    point_id: PointId,
    /// last node's triangle is None
    pub triangle: TriangleId,
}

impl AdvancingFront {
    /// Create a new advancing front with the initial triangle
    /// Triangle's point order: P0, P-1, P-2
    pub fn new(triangle: &InnerTriangle, triangle_id: TriangleId, points: &Points) -> Self {
        let mut nodes = Vec::<Entry>::with_capacity(32);

        let first_point = points
            .get_point(triangle.points[1])
            .expect("should not fail");
        let middle_point = points
            .get_point(triangle.points[0])
            .expect("should not fail");
        let tail_node = points
            .get_point(triangle.points[2])
            .expect("should not fail");

        nodes.push(Entry::new(
            first_point.into(),
            NodeInner {
                point_id: triangle.points[1],
                triangle: triangle_id,
            },
        ));
        nodes.push(Entry::new(
            middle_point.into(),
            NodeInner {
                point_id: triangle.points[0],
                triangle: triangle_id,
            },
        ));
        nodes.push(Entry::new(
            tail_node.into(),
            NodeInner {
                point_id: triangle.points[2],
                triangle: TriangleId::INVALID,
            },
        ));

        nodes.sort_unstable_by_key(|e| e.key);

        Self {
            nodes,
            access_cache: None,
            #[cfg(test)]
            hit_count: 0.into(),
            #[cfg(test)]
            miss_count: 0.into(),
        }
    }

    /// insert a new node for point and triangle
    /// or update the node pointing to new triangle
    pub fn insert(&mut self, point_id: PointId, point: Point, triangle_id: TriangleId) {
        debug_assert!(!triangle_id.invalid());
        let new_node = NodeInner {
            point_id,
            triangle: triangle_id,
        };
        let node_index = match self.search_by_key(&PointKey(point)) {
            Ok(idx) => {
                self.nodes[idx].node = new_node;
                idx
            }
            Err(idx) => {
                self.nodes.insert(idx, Entry::new(point.into(), new_node));
                idx
            }
        };
        self.access_cache = Some((PointKey(point), node_index));
    }

    /// insert a new node for point and triangle
    /// or update the node pointing to new triangle
    /// when call this method, need to ensure that index still points to the correct node
    pub(crate) unsafe fn update_and_delete_by_index(
        &mut self,
        update_index: usize,
        point_id: PointId,
        triangle_id: TriangleId,
        delete_index: usize,
    ) {
        debug_assert!(!triangle_id.invalid());

        // update first, update won't modify index, so later delete is still safe
        let entry = self.nodes.get_mut(update_index).unwrap();
        debug_assert!(entry.node.point_id == point_id, "point_id mismatch");
        entry.node.triangle = triangle_id;

        // then delete
        self.nodes.remove(delete_index);

        self.access_cache = None;
    }

    /// Get `n`th node
    pub fn nth(&self, n: usize) -> Option<NodeRef> {
        self.nodes.get(n).map(|entry| entry.to_node(n, self))
    }

    pub fn iter<'a>(&'a self) -> Box<dyn Iterator<Item = NodeRef> + 'a> {
        Box::new(
            self.nodes
                .iter()
                .enumerate()
                .map(|(idx, entry)| entry.to_node(idx, self)),
        )
    }

    /// locate the node containing point
    /// locate the node for `x`
    pub fn locate_node(&self, point: Point) -> Option<NodeRef> {
        let key = PointKey(point);
        let idx = match self.search_by_key(&key) {
            Err(idx) => idx - 1,
            Ok(idx) => idx,
        };
        // safety: idx is checked
        let entry = unsafe { self.nodes.get_unchecked(idx) };
        Some(entry.to_node(idx, self))
    }

    /// Get the node identified by `point`
    pub fn get_node(&self, point: Point) -> Option<NodeRef> {
        let index = self.search_by_key_with_cache(&PointKey(point)).ok()?;

        // safety: idx is checked
        Some(unsafe { self.nodes.get_unchecked(index) }.to_node(index, self))
    }

    /// Get the node identified by `point`
    pub fn get_node_with_cache(&mut self, point: Point) -> Option<NodeRef> {
        let index = self.search_by_key_with_cache(&PointKey(point)).ok()?;

        // update cache
        self.access_cache = Some((PointKey(point), index));

        // safety: idx is checked
        Some(unsafe { self.nodes.get_unchecked(index) }.to_node(index, self))
    }

    /// Get the node identified by `point`
    pub fn get_node_with_id(&self, node_id: NodeId) -> Option<NodeRef> {
        let index = self.resolve_index_for_id(node_id).ok()?;
        // safety: idx is checked
        Some(unsafe { self.nodes.get_unchecked(index) }.to_node(index, self))
    }

    /// update node's triangle
    pub fn update_triangle(&mut self, point: Point, triangle_id: TriangleId) {
        let idx = self.search_by_key_with_cache(&PointKey(point)).unwrap();
        self.nodes[idx].node.triangle = triangle_id;
    }

    /// Get next node of the node identified by `point`
    /// Note: even if the node is deleted, this also returns next node as if it is not deleted
    pub fn locate_next_node(&self, node_id: NodeId) -> Option<NodeRef> {
        let idx = match self.resolve_index_for_id(node_id) {
            Ok(idx) => idx + 1,
            Err(idx) => idx,
        };
        if idx < self.nodes.len() {
            // safety: idx checked above
            Some(unsafe { self.nodes.get_unchecked(idx) }.to_node(idx, self))
        } else {
            None
        }
    }

    /// Get next node of the node identified by `point`
    /// Note: even if the node is deleted, this also returns next node as if it is not deleted
    pub(super) fn next_node(&self, node: &NodeRef) -> Option<NodeRef> {
        let idx = node.index + 1;
        if idx < self.nodes.len() {
            Some(self.nodes[idx].to_node(idx, self))
        } else {
            None
        }
    }

    /// Get prev node of the node identified by `point`
    /// Note: even if the node is deleted, then this returns prev node as if it is not deleted
    pub fn locate_prev_node(&self, node_id: NodeId) -> Option<NodeRef> {
        let idx = match self.resolve_index_for_id(node_id) {
            Ok(idx) | Err(idx) if idx > 0 => idx - 1,
            _ => return None,
        };

        // safety: idx checked above
        Some(unsafe { self.nodes.get_unchecked(idx) }.to_node(idx, self))
    }

    /// Get prev node of the node identified by `point`
    /// Note: even if the node is deleted, then this returns prev node as if it is not deleted
    pub(super) fn prev_node(&self, node: &NodeRef) -> Option<NodeRef> {
        if node.index == 0 {
            return None;
        }

        let index = node.index - 1;
        // satefy: idx checked above
        Some(unsafe { self.nodes.get_unchecked(index) }.to_node(index, self))
    }

    fn search_by_key(&self, key: &PointKey) -> Result<usize, usize> {
        self.nodes.binary_search_by_key(key, |e| e.key)
    }

    fn search_by_key_with_cache(&self, key: &PointKey) -> Result<usize, usize> {
        match self.access_cache {
            Some((p, i)) => {
                let order = p.cmp(key);
                match order {
                    Ordering::Equal => {
                        #[cfg(test)]
                        self.hit_count
                            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        Ok(i)
                    }
                    Ordering::Greater => {
                        // cached key is larger, then we check one elem before
                        match self.nodes[i - 1].key.cmp(key) {
                            Ordering::Equal => {
                                #[cfg(test)]
                                self.hit_count
                                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                Ok(i - 1)
                            }
                            Ordering::Less => {
                                // the prev item is less than key
                                #[cfg(test)]
                                self.hit_count
                                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                Err(i)
                            }
                            _ => {
                                // never mind
                                #[cfg(test)]
                                self.miss_count
                                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);

                                self.nodes[..i].binary_search_by_key(key, |e| e.key)
                            }
                        }
                    }
                    Ordering::Less if i < self.nodes.len() => {
                        match self.nodes[i + 1].key.cmp(key) {
                            Ordering::Greater => {
                                // the prev item is less than key
                                #[cfg(test)]
                                self.hit_count
                                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                Err(i + 1)
                            }
                            _ => {
                                // never mind
                                #[cfg(test)]
                                self.miss_count
                                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);

                                self.nodes[i..]
                                    .binary_search_by_key(key, |e| e.key)
                                    .map(|idx| idx + i)
                                    .map_err(|idx| idx + i)
                            }
                        }
                    }
                    _ => {
                        #[cfg(test)]
                        self.miss_count
                            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);

                        self.nodes[i..]
                            .binary_search_by_key(key, |e| e.key)
                            .map(|idx| idx + i)
                            .map_err(|idx| idx + i)
                    }
                }
            }
            _ => {
                #[cfg(test)]
                self.miss_count
                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);

                self.search_by_key(key)
            }
        }
    }

    /// resolve node_id's latest index.
    /// Return Err(index to insert) when the node is deleted
    fn resolve_index_for_id(&self, node_id: NodeId) -> Result<usize, usize> {
        match self.nodes.get(node_id.index_hint) {
            Some(entry) if entry.node.point_id == node_id.point_id => {
                // index_hint match
                Ok(node_id.index_hint)
            }
            _ => {
                let idx = self.search_by_key(&PointKey(node_id.point))?;
                Ok(idx)
            }
        }
    }
}
