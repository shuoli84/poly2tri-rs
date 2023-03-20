use std::cmp::Ordering;

use crate::{shape::Point, Float};

/// Type alias to the underlying type for PointId.
/// Despite of maximum number supported, type size also affect performance
/// PointId compare is in hot path, e.g, triangle neighbor check, find edge index etc
type NumType = u32;

/// new type for point id, currently is the index in context
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct PointId(pub(crate) NumType);

impl PointId {
    /// Get the inner value as usize
    pub fn as_usize(&self) -> usize {
        self.0 as usize
    }

    /// Get the inner value as u32
    pub fn as_u32(&self) -> u32 {
        self.0 as u32
    }

    /// helper method used in the crate when I know the `PointId` is valid in `Points`
    pub(crate) fn get(&self, points: &Points) -> Point {
        unsafe { points.get_point_uncheck(*self) }
    }
}

#[derive(Clone, Default)]
pub struct PointsBuilder {
    points: Vec<PointWithEdge>,
}

impl PointsBuilder {
    pub fn with_capacity(cap: usize) -> Self {
        Self {
            points: Vec::with_capacity(cap),
        }
    }

    /// Add a point
    pub fn add_steiner_point(&mut self, point: Point) -> PointId {
        let point_id = PointId(self.points.len() as NumType);
        self.points.push(PointWithEdge {
            point,
            edges: PointEdges::None,
        });
        point_id
    }

    pub(crate) fn get_point_mut(&mut self, point_id: PointId) -> Option<&mut PointWithEdge> {
        self.points.get_mut(point_id.as_usize())
    }

    pub fn build(self) -> Points {
        Points::new(self.points)
    }
}

#[derive(Clone, Copy)]
pub enum PointEdges {
    None,
    One(PointId),
    Two(PointId, PointId),
}

impl PointEdges {
    pub fn push(&mut self, point_id: PointId) {
        *self = match self {
            PointEdges::None => PointEdges::One(point_id),
            PointEdges::One(p0) => PointEdges::Two(*p0, point_id),
            PointEdges::Two(_, _) => panic!("one point only has two edges"),
        };
    }
}

#[derive(Clone, Copy)]
pub struct PointWithEdge {
    pub point: Point,
    pub edges: PointEdges,
}

impl Iterator for PointEdges {
    type Item = PointId;

    fn next(&mut self) -> Option<Self::Item> {
        let mut result = None;
        let new_self = match std::mem::replace(self, PointEdges::None) {
            PointEdges::None => PointEdges::None,
            PointEdges::One(p) => {
                result = Some(p);
                PointEdges::None
            }
            PointEdges::Two(p0, p1) => {
                result = Some(p0);
                PointEdges::One(p1)
            }
        };
        *self = new_self;
        result
    }
}

/// Point store
#[derive(Clone)]
pub struct Points {
    points: Vec<PointWithEdge>,
    y_sorted: Vec<PointId>,
    pub head: PointId,
    pub tail: PointId,
}

impl Points {
    pub fn new(mut points: Vec<PointWithEdge>) -> Self {
        let mut xmax = Float::MIN;
        let mut xmin = Float::MAX;
        let mut ymax = Float::MIN;
        let mut ymin = Float::MAX;

        let mut unsorted_points = points
            .iter()
            .enumerate()
            .map(|(idx, p)| {
                xmax = xmax.max(p.point.x);
                xmin = xmin.min(p.point.x);
                ymax = ymax.max(p.point.y);
                ymin = ymin.min(p.point.y);
                (PointId(idx as NumType), p.point)
            })
            .collect::<Vec<_>>();

        // sort by y
        unsorted_points.sort_by(|p1, p2| {
            let p1 = p1.1;
            let p2 = p2.1;

            if p1.y < p2.y {
                Ordering::Less
            } else if p1.y == p2.y {
                if p1.x < p2.x {
                    Ordering::Less
                } else {
                    Ordering::Greater
                }
            } else {
                Ordering::Greater
            }
        });
        let sorted_ids = unsorted_points
            .into_iter()
            .map(|(idx, _)| idx)
            .collect::<Vec<_>>();

        let (head, tail) = {
            let dx = (xmax - xmin) * 0.3;
            let dy = (ymax - ymin) * 0.3;

            let head = Point::new(xmin - dx, ymin - dy);
            let head_id = PointId(points.len() as NumType);
            points.push(PointWithEdge {
                point: head,
                edges: PointEdges::None,
            });

            let tail = Point::new(xmax + dx, ymin - dy);
            let tail_id = PointId(points.len() as NumType);
            points.push(PointWithEdge {
                point: tail,
                edges: PointEdges::None,
            });
            (head_id, tail_id)
        };

        Self {
            points,
            y_sorted: sorted_ids,
            head,
            tail,
        }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// get point for id
    pub fn get_point(&self, point_id: PointId) -> Option<Point> {
        self.points
            .get(point_id.as_usize())
            .map(|p| &p.point)
            .cloned()
    }

    /// get point for id
    pub unsafe fn get_point_uncheck(&self, point_id: PointId) -> Point {
        unsafe { self.points.get_unchecked(point_id.as_usize()).point }
    }

    pub fn iter_point_by_y<'a>(
        &'a self,
        order: usize,
    ) -> impl Iterator<Item = (PointId, Point, PointEdges)> + 'a {
        self.y_sorted.iter().skip(order).map(|id| {
            let point = unsafe { self.points.get_unchecked(id.as_usize()).clone() };
            (*id, point.point, point.edges)
        })
    }

    /// get point by y order
    /// Not including head, tail
    pub fn get_id_by_y(&self, order: usize) -> Option<PointId> {
        self.y_sorted.get(order).cloned()
    }

    /// iter all points
    pub fn iter(&self) -> impl Iterator<Item = (PointId, &Point, PointEdges)> {
        self.points
            .iter()
            .enumerate()
            .map(|(idx, p)| (PointId(idx as NumType), &p.point, p.edges))
    }

    /// iter all points without fake points
    pub fn iter_without_fake(&self) -> impl Iterator<Item = (PointId, &Point, PointEdges)> {
        // skip the last two points, they are fake point
        self.points.as_slice()[..self.points.len() - 2]
            .iter()
            .enumerate()
            .map(|(idx, p)| (PointId(idx as NumType), &p.point, p.edges))
    }
}
