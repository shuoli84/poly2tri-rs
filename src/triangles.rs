use crate::shape::InnerTriangle;

#[derive(Debug, Hash, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct TriangleId(usize);

impl TriangleId {
    pub const INVALID: TriangleId = TriangleId(usize::MAX);

    /// whether id is invalid
    pub fn invalid(&self) -> bool {
        self.0 == Self::INVALID.0
    }

    pub fn get<'a, 'b>(&'a self, triangles: &'b TriangleStore) -> &'b InnerTriangle {
        triangles.get_unchecked(*self)
    }

    pub fn try_get<'a, 'b>(&'a self, triangles: &'b TriangleStore) -> Option<&'b InnerTriangle> {
        triangles.get(*self)
    }

    pub fn as_usize(&self) -> usize {
        self.0
    }

    pub fn from_index(index: usize) -> Self {
        Self(index)
    }

    pub fn into_option(self) -> Option<Self> {
        if self.invalid() {
            None
        } else {
            Some(self)
        }
    }
}

/// Triangle store, store triangles and their neighborhood relations
// Note: For n vetexes, there will around n - 2 triangles, so space complexity is
//       O(n).
#[derive(Debug)]
pub struct TriangleStore {
    triangles: Vec<InnerTriangle>,
}

impl TriangleStore {
    pub fn new() -> Self {
        Self { triangles: vec![] }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            triangles: Vec::with_capacity(capacity),
        }
    }

    /// Returns number of triangles
    pub fn len(&self) -> usize {
        self.triangles.len()
    }

    /// insert a new triangle
    pub fn insert(&mut self, triangle: InnerTriangle) -> TriangleId {
        let id = TriangleId::from_index(self.triangles.len());
        self.triangles.push(triangle);
        id
    }

    pub fn get(&self, id: TriangleId) -> Option<&InnerTriangle> {
        if id == TriangleId::INVALID {
            return None;
        }
        unsafe { Some(self.triangles.get_unchecked(id.as_usize())) }
    }

    pub fn get_unchecked(&self, id: TriangleId) -> &InnerTriangle {
        debug_assert!(!id.invalid());
        unsafe { self.triangles.get_unchecked(id.as_usize()) }
    }

    pub fn get_mut(&mut self, id: TriangleId) -> Option<&mut InnerTriangle> {
        self.triangles.get_mut(id.as_usize())
    }

    pub(crate) unsafe fn get_mut_two(
        &mut self,
        id_0: TriangleId,
        id_1: TriangleId,
    ) -> (&mut InnerTriangle, &mut InnerTriangle) {
        assert!(
            id_0 != id_1
                && id_0.as_usize() < self.triangles.len()
                && id_1.as_usize() < self.triangles.len()
        );

        let slice: *mut InnerTriangle = self.triangles.as_mut_ptr();

        // satefy: asserted that id_0 != id_1 && id_0 < len && id_1 < len
        let ref_0 = unsafe { &mut *slice.add(id_0.as_usize()) };
        let ref_1 = unsafe { &mut *slice.add(id_1.as_usize()) };

        (ref_0, ref_1)
    }

    pub(crate) unsafe fn get_mut_six(
        &mut self,
        id_0: TriangleId,
        id_1: TriangleId,
        id_2: TriangleId,
        id_3: TriangleId,
        id_4: TriangleId,
        id_5: TriangleId,
    ) -> (
        &mut InnerTriangle,
        &mut InnerTriangle,
        Option<&mut InnerTriangle>,
        Option<&mut InnerTriangle>,
        Option<&mut InnerTriangle>,
        Option<&mut InnerTriangle>,
    ) {
        assert!(
            id_0 != id_1
                && id_0.as_usize() < self.triangles.len()
                && id_1.as_usize() < self.triangles.len()
        );

        let slice: *mut InnerTriangle = self.triangles.as_mut_ptr();

        // satefy: asserted that id_0 != id_1 && id_0 < len && id_1 < len
        let ref_0 = unsafe { &mut *slice.add(id_0.as_usize()) };
        let ref_1 = unsafe { &mut *slice.add(id_1.as_usize()) };
        let ref_2 = if !id_2.invalid() {
            Some(unsafe { &mut *slice.add(id_2.as_usize()) })
        } else {
            None
        };

        let ref_3 = if !id_3.invalid() {
            Some(unsafe { &mut *slice.add(id_3.as_usize()) })
        } else {
            None
        };

        let ref_4 = if !id_4.invalid() {
            Some(unsafe { &mut *slice.add(id_4.as_usize()) })
        } else {
            None
        };

        let ref_5 = if !id_5.invalid() {
            Some(unsafe { &mut *slice.add(id_5.as_usize()) })
        } else {
            None
        };

        (ref_0, ref_1, ref_2, ref_3, ref_4, ref_5)
    }

    pub fn get_mut_unchecked(&mut self, id: TriangleId) -> &mut InnerTriangle {
        unsafe { self.triangles.get_unchecked_mut(id.as_usize()) }
    }

    pub fn iter(&self) -> impl Iterator<Item = (TriangleId, &InnerTriangle)> {
        self.triangles
            .iter()
            .enumerate()
            .map(|(idx, t)| (TriangleId::from_index(idx), t))
    }

    /// mark two triangle as neighbor
    pub fn mark_neighbor(&mut self, left: TriangleId, right: TriangleId) {
        let (left_triangle, right_triangle) = unsafe { self.get_mut_two(left, right) };
        Self::mark_neighbor_for_two_mut(left, right, left_triangle, right_triangle)
    }

    /// mark two triangle as neighbor
    /// also clears delaunay flag for the common edge
    pub fn mark_neighbor_for_two_mut(
        left: TriangleId,
        right: TriangleId,
        left_triangle: &mut InnerTriangle,
        right_triangle: &mut InnerTriangle,
    ) {
        let Some((l_ei, r_ei)) = left_triangle.common_edge_index(&right_triangle) else {
            debug_assert!(false, "they are not neighbors");
            return;
        };

        let is_constrained_edge =
            left_triangle.is_constrained(l_ei) || right_triangle.is_constrained(r_ei);

        left_triangle.neighbors[l_ei] = right;
        left_triangle.set_constrained(l_ei, is_constrained_edge);
        left_triangle.set_delaunay(l_ei, false);

        right_triangle.neighbors[r_ei] = left;
        right_triangle.set_constrained(r_ei, is_constrained_edge);
        right_triangle.set_delaunay(r_ei, false);
    }
}

#[cfg(test)]
mod tests {
    use crate::{points::PointsBuilder, shape::Point};

    use super::*;

    #[test]
    fn test_triangles() {
        let mut triangles = TriangleStore::new();
        let mut points = PointsBuilder::default();

        let p0 = points.add_steiner_point(Point::new(0., 0.));
        let p1 = points.add_steiner_point(Point::new(2., 0.));
        let p2 = points.add_steiner_point(Point::new(1., 2.));
        let p3 = points.add_steiner_point(Point::new(4., 2.));

        let t1 = triangles.insert(InnerTriangle::new(p0, p1, p2));
        let t2 = triangles.insert(InnerTriangle::new(p2, p1, p3));

        triangles.mark_neighbor(t1, t2);
        {
            let t = triangles.get(t1).unwrap();
            assert_eq!(t.neighbors[0], t2);
            let t = triangles.get(t2).unwrap();
            assert_eq!(t.neighbors[2], t1);
        }
    }

    #[test]
    fn test_triangles_get_mut_two() {
        let mut triangles = TriangleStore::new();
        let mut points = PointsBuilder::default();

        let p0 = points.add_steiner_point(Point::new(0., 0.));
        let p1 = points.add_steiner_point(Point::new(2., 0.));
        let p2 = points.add_steiner_point(Point::new(1., 2.));
        let p3 = points.add_steiner_point(Point::new(4., 2.));

        let t1 = triangles.insert(InnerTriangle::new(p0, p1, p2));
        let t2 = triangles.insert(InnerTriangle::new(p1, p2, p3));

        let (t1, t2) = unsafe { triangles.get_mut_two(t1, t2) };
        assert_eq!(t1.points, [p0, p1, p2]);
        assert_eq!(t2.points, [p1, p2, p3]);
    }
}
