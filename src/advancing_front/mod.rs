use std::cmp::Ordering;

use crate::{triangles::TriangleId, Point, PointId};

mod vec_backed;
pub use vec_backed::AdvancingFront;

/// A owned version of NodeId, this should be used when you need to pass NodeRef along with
/// AdvancingFront's mut reference.
#[derive(Clone, Copy)]
pub struct NodeId {
    point_id: PointId,
    point: Point,
    cursor: vec_backed::CursorAF,
}

impl NodeId {
    pub fn point_id(&self) -> PointId {
        self.point_id
    }

    pub fn point(&self) -> Point {
        self.point
    }
}

/// A borrowed version of Node, use this in most case, it ensures the underlying
/// `AdvancingFront` stays.
#[derive(Clone)]
pub struct NodeRef<'a> {
    point_id: PointId,
    point: Point,
    /// last node's triangle is None
    pub triangle: Option<TriangleId>,

    cursor: vec_backed::CursorAF,
    advancing_front: &'a AdvancingFront,
}

impl<'a> NodeRef<'a> {
    pub fn point(&self) -> Point {
        self.point
    }

    pub fn point_id(&self) -> PointId {
        self.point_id
    }

    pub fn next(&self) -> Option<NodeRef<'a>> {
        self.advancing_front.next_node(self)
    }

    pub fn prev(&self) -> Option<NodeRef> {
        self.advancing_front.prev_node(self)
    }

    /// convert to a owned version of NodeId
    pub fn get_node_id(&self) -> NodeId {
        NodeId {
            point_id: self.point_id,
            point: self.point,
            cursor: self.cursor,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        points::PointsBuilder,
        shape::{InnerTriangle, Point},
        triangles::TriangleStore,
    };

    #[test]
    fn test_advancing_front() {
        let mut triangles = TriangleStore::new();

        let mut points = PointsBuilder::default();
        let p_0 = points.add_steiner_point(Point::new(-1., 0.));
        let p_1 = points.add_steiner_point(Point::new(0., 3.));
        let p_2 = points.add_steiner_point(Point::new(1., 1.));
        let points = points.build();

        let triangle_id = triangles.insert(InnerTriangle::new(p_0, p_1, p_2));
        let triangle = triangles.get(triangle_id).unwrap();

        let advancing_front = AdvancingFront::new(triangle, triangle_id, &points);
        {
            let p = advancing_front.locate_node(Point::new(0., 10.)).unwrap();
            let point = p.point();
            assert_eq!(point.x, 0.0);
            assert_eq!(point.y, 3.0);

            let p = advancing_front
                .locate_node(Point::new(0.3, 10.))
                .unwrap()
                .get_node_id();
            let point = p.point();
            assert_eq!(point.x, 0.0);
            assert_eq!(point.y, 3.0);

            let prev_node = advancing_front.locate_prev_node(p).unwrap();
            assert_eq!(prev_node.point().x, -1.);

            let next_node = advancing_front.locate_next_node(p).unwrap();
            assert_eq!(next_node.point().x, 1.);
        }
    }
}
