use super::*;
use crate::shape::InnerTriangle;
use crate::{points::Points, shape::Point, triangles::TriangleId, PointId};
use sweep_bptree::{BPlusTree, Cursor, NodeStoreVec};

type BPlusTreeAF = BPlusTree<NodeStoreVec<PointKey, NodeInner, 128, 129, 128>>;
pub type CursorAF = Cursor<PointKey>;

/// Advancing front, stores all advancing edges in a btree, this makes store compact
/// and easier to update
pub struct AdvancingFront {
    nodes: BPlusTreeAF,
}

/// New type to wrap `Point` as Node's key
#[derive(Debug, Clone, Copy)]
pub struct PointKey(Point);

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

#[derive(Debug, Clone, Copy)]
struct NodeInner {
    point_id: PointId,
    /// last node's triangle is None
    pub triangle: TriangleId,
}

impl AdvancingFront {
    /// Create a new advancing front with the initial triangle
    /// Triangle's point order: P0, P-1, P-2
    pub fn new(triangle: &InnerTriangle, triangle_id: TriangleId, points: &Points) -> Self {
        let mut nodes = BPlusTreeAF::new(NodeStoreVec::new());

        let first_point = points
            .get_point(triangle.points[1])
            .expect("should not fail");
        let middle_point = points
            .get_point(triangle.points[0])
            .expect("should not fail");
        let tail_node = points
            .get_point(triangle.points[2])
            .expect("should not fail");

        nodes.insert(
            first_point.into(),
            NodeInner {
                point_id: triangle.points[1],
                triangle: triangle_id,
            },
        );
        nodes.insert(
            middle_point.into(),
            NodeInner {
                point_id: triangle.points[0],
                triangle: triangle_id,
            },
        );
        nodes.insert(
            tail_node.into(),
            NodeInner {
                point_id: triangle.points[2],
                triangle: TriangleId::INVALID,
            },
        );

        Self { nodes }
    }

    /// insert a new node for point and triangle
    /// or update the node pointing to new triangle
    pub fn insert(&mut self, point_id: PointId, point: Point, triangle_id: TriangleId) {
        debug_assert!(!triangle_id.invalid());
        let new_node = NodeInner {
            point_id,
            triangle: triangle_id,
        };
        self.nodes.insert(point.into(), new_node);
    }

    /// insert a new node for point and triangle
    /// or update the node pointing to new triangle
    /// when call this method, need to ensure that index still points to the correct node
    pub(crate) fn update_and_delete_by_index(
        &mut self,
        point: Point,
        point_id: PointId,
        triangle_id: TriangleId,
        point_to_remove: Point,
    ) {
        debug_assert!(!triangle_id.invalid());

        // update first, update won't modify index, so later delete is still safe
        let node = self.nodes.get_mut(&point.into()).unwrap();
        debug_assert!(node.point_id == point_id, "point_id mismatch");
        node.triangle = triangle_id;

        // then delete
        self.nodes.remove(&point_to_remove.into());
    }

    /// Get `n`th node
    pub fn nth(&self, n: usize) -> Option<NodeRef> {
        self.nodes.iter().nth(n).map(|(k, v)| NodeRef {
            point_id: v.point_id,
            point: k.point(),
            triangle: v.triangle.into_option(),
            advancing_front: self,
            cursor: self.nodes.get_cursor(k).unwrap().0,
        })
    }

    pub fn iter<'a>(&'a self) -> Box<dyn Iterator<Item = NodeRef> + 'a> {
        Box::new(self.nodes.iter().map(|(k, v)| NodeRef {
            point_id: v.point_id,
            point: k.point(),
            triangle: v.triangle.into_option(),
            advancing_front: self,
            cursor: self.nodes.get_cursor(k).unwrap().0,
        }))
    }

    /// locate the node containing point
    /// locate the node for `x`
    pub fn locate_node(&self, point: Point) -> Option<NodeRef> {
        let key = PointKey(point);
        let (cursor, _kv) = self.nodes.get_cursor(&key)?;
        let (prev_cursor, prev_value) = cursor.prev_with_value(&self.nodes)?;
        Some(NodeRef {
            point_id: prev_value.point_id,
            point: prev_cursor.key().point(),
            triangle: prev_value.triangle.into_option(),
            cursor: prev_cursor,
            advancing_front: self,
        })
    }

    /// Get the node identified by `point`
    pub fn get_node(&self, point: Point) -> Option<NodeRef> {
        let key = PointKey::from(point);
        let (cursor, value) = self.nodes.get_cursor(&key)?;
        let (_k, v) = value?;

        Some(NodeRef {
            point_id: v.point_id,
            point,
            triangle: v.triangle.into_option(),
            advancing_front: self,
            cursor,
        })
    }

    /// Get the node identified by `point`
    pub fn get_node_with_cache(&mut self, point: Point) -> Option<NodeRef> {
        self.get_node(point)
    }

    /// Get the node identified by `point`
    pub fn get_node_with_id(&self, node_id: NodeId) -> Option<NodeRef> {
        let cursor = node_id.cursor;
        let cursor_value = cursor.value(&self.nodes)?;

        Some(NodeRef {
            point_id: cursor_value.point_id,
            point: cursor.key().point(),
            triangle: cursor_value.triangle.into_option(),
            cursor: cursor,
            advancing_front: self,
        })
    }

    /// update node's triangle
    pub fn update_triangle(&mut self, point: Point, triangle_id: TriangleId) {
        let node = self.nodes.get_mut(&PointKey(point)).unwrap();
        node.triangle = triangle_id;
    }

    /// Get next node of the node identified by `point`
    /// Note: even if the node is deleted, this also returns next node as if it is not deleted
    pub fn locate_next_node(&self, node_id: NodeId) -> Option<NodeRef> {
        let (next, next_value) = node_id.cursor.next_with_value(&self.nodes)?;
        Some(NodeRef {
            point_id: next_value.point_id,
            point: next.key().point(),
            triangle: next_value.triangle.into_option(),
            cursor: next,
            advancing_front: self,
        })
    }

    /// Get next node of the node identified by `point`
    /// Note: even if the node is deleted, this also returns next node as if it is not deleted
    pub(super) fn next_node(&self, node: &NodeRef) -> Option<NodeRef> {
        let (next, next_value) = node.cursor.next_with_value(&self.nodes)?;
        Some(NodeRef {
            point_id: next_value.point_id,
            point: next.key().point(),
            triangle: next_value.triangle.into_option(),
            cursor: next,
            advancing_front: self,
        })
    }

    /// Get prev node of the node identified by `point`
    /// Note: even if the node is deleted, then this returns prev node as if it is not deleted
    pub fn locate_prev_node(&self, node_id: NodeId) -> Option<NodeRef> {
        let (prev, prev_value) = node_id.cursor.prev_with_value(&self.nodes)?;
        Some(NodeRef {
            point_id: prev_value.point_id,
            point: prev.key().point(),
            triangle: prev_value.triangle.into_option(),
            cursor: prev,
            advancing_front: self,
        })
    }

    /// Get prev node of the node identified by `point`
    /// Note: even if the node is deleted, then this returns prev node as if it is not deleted
    pub(super) fn prev_node(&self, node: &NodeRef) -> Option<NodeRef> {
        // todo: NodeId is Copy, so mut actually is not useful
        let (prev, prev_value) = node.cursor.prev_with_value(&self.nodes)?;
        Some(NodeRef {
            point_id: prev_value.point_id,
            point: prev.key().point(),
            triangle: prev_value.triangle.into_option(),
            cursor: prev,
            advancing_front: self,
        })
    }
}
