use crate::advancing_front::AdvancingFront;
use crate::points::{Points, PointsBuilder};
use crate::triangles::TriangleId;
use crate::triangles::TriangleStore;
use crate::{shape::*, PointDataSource, PointId, Sweeper, Triangle};

/// Observer for sweeper, used to monitor how sweeper works, quite useful
/// for visual debugging when things goes wrong. Check example's draw.
#[allow(unused_variables)]
pub trait Observer {
    /// A point_event processed
    fn enter_point_event(&mut self, point_id: PointId, context: &Sweeper) {}
    fn exit_point_event(&mut self, point_id: PointId, context: &Sweeper) {}

    /// An edge event processed
    fn edge_event(&mut self, edge: Edge, context: &Sweeper) {}

    /// Sweep process done
    fn sweep_done(&mut self, context: &Sweeper) {}

    /// The result finalized, holes, fake points etc cleaned.
    fn finalized(&mut self, context: &Sweeper) {}

    /// About to legalize for triangle
    #[inline]
    fn will_legalize(&mut self, triangle_id: TriangleId, context: &Sweeper) {}

    /// A single step inside one legalization process
    #[inline]
    fn legalize_step(&mut self, triangle_id: TriangleId, context: &Sweeper) {}

    /// A rotate happened
    #[inline]
    fn triangle_rotated(
        &mut self,
        triangle_id: TriangleId,
        opposite_triangle_id: TriangleId,
        context: &Sweeper,
    ) {
    }

    /// The triangle legalized
    #[inline]
    fn legalized(&mut self, triangel_id: TriangleId, context: &Sweeper) {}
}

/// Default dummy observer, blank impl, so all calls should be optimized out by compiler.
impl Observer for () {}

/// CDT Builder
///
/// PD(PointData), associated data for each point
///
/// # Example
/// ```rust
///    use poly2tri_rs::{Builder, Point};
///
///    let builder = Builder::new(vec![
///        Point::new(-10., -10.),
///        Point::new(810., -10.),
///        Point::new(810., 810.),
///        Point::new(-10., 810.),
///    ]).add_steiner_points(vec![
///        Point::new(50., 50.),
///    ]).add_hole(vec![
///        Point::new(400., 400.),
///        Point::new(600., 400.),
///        Point::new(600., 600.),
///        Point::new(400., 600.),
///    ]);
///    let cdt = builder.build();
///    let _triangles = cdt.triangulate();
/// ```
#[derive(Clone)]
pub struct Builder<PD = ()> {
    points_builder: PointsBuilder,
    data: Vec<PD>,
}

impl<PD> Builder<PD> {
    /// Create a new Builder with polyline
    /// There should be only one polyline, and multiple holes and steiner points supported
    pub fn new(polyline_source: impl PointDataSource<Data = PD>) -> Self {
        let (polyline, data) = polyline_source.points_and_data();

        let mut points_builder = PointsBuilder::with_capacity(polyline.len());
        parse_polyline(&polyline, &mut points_builder);

        Self {
            points_builder,
            data,
        }
    }

    /// Add a single sparse `Point`, there is no edge attached to it
    /// NOTE: if the point locates outside of polyline, then it has no
    /// effect on the final result
    pub fn add_steiner_point(mut self, point: Point, data: PD) -> Self {
        self.points_builder.add_steiner_point(point);
        self.data.push(data);
        self
    }

    /// Add multiple [`Point`], batch version for `Self::add_point`
    pub fn add_steiner_points(mut self, point_source: impl PointDataSource<Data = PD>) -> Self {
        let (points, data) = point_source.points_and_data();
        self.points_builder.add_steiner_points(points);
        self.data.extend(data);
        self
    }

    /// Add a hole defined by polyline.
    pub fn add_hole(mut self, polyline: impl PointDataSource<Data = PD>) -> Self {
        let (polyline, data) = polyline.points_and_data();
        parse_polyline(&polyline, &mut self.points_builder);
        self.data.extend(data);

        self
    }

    /// Add holes
    pub fn add_holes(
        mut self,
        holes: impl IntoIterator<Item = impl PointDataSource<Data = PD>>,
    ) -> Self {
        for polyline in holes.into_iter() {
            self = self.add_hole(polyline);
        }
        self
    }

    /// build the sweeper
    pub fn build(self) -> CDT<PD> {
        let points = self.points_builder.build();
        CDT {
            points,
            data: self.data,
        }
    }
}

/// Main interface, user should grab a new Sweeper by [`SweeperBuilder::build`]
#[derive(Clone)]
pub struct CDT<PD> {
    points: Points,
    data: Vec<PD>,
}

/// The result of triangulate
pub struct Triangles<PD> {
    /// points store, it includes all points, including ones in hole
    points: Points,
    /// point data
    data: Vec<PD>,
    /// including all triangles, including ones in hole
    triangles: TriangleStore,
    /// final result `TriangleId`s
    result: Vec<TriangleId>,

    /// iterator next cursor
    next: usize,
}

impl<PD> Triangles<PD> {
    /// iter all points. Note: not all points is in valid triangle, if the point
    /// is outside or in hole, then it still reachable here.
    pub fn points(&self) -> &[Point] {
        self.points.points_without_fake()
    }

    /// get slice for Point Data
    pub fn data(&self) -> &[PD] {
        &self.data
    }

    /// Get indices. Each three indices construct an triangle
    /// Point's order is the same as input, also you can iter point
    /// with `iter_point` to construct a new point buffer.
    pub fn triangle_list_indices(&self) -> Vec<u32> {
        let mut result = Vec::with_capacity(self.result.len() * 3);

        for triangle_id in self.result.iter() {
            let t = triangle_id.get(&self.triangles);
            result.extend_from_slice(&[
                t.points[0].as_u32(),
                t.points[1].as_u32(),
                t.points[2].as_u32(),
            ]);
        }

        result
    }
}

impl<PD> Iterator for Triangles<PD> {
    type Item = Triangle;

    fn next(&mut self) -> Option<Self::Item> {
        if self.next < self.result.len() {
            let index = self.next;
            self.next += 1;

            // safety: just checked index less than len
            let tri_id = unsafe { self.result.get_unchecked(index) };
            let triangle = tri_id.get(&self.triangles);

            return Some(Triangle {
                points: [
                    triangle.points[0].get(&self.points),
                    triangle.points[1].get(&self.points),
                    triangle.points[2].get(&self.points),
                ],
            });
        } else {
            None
        }
    }
}

impl<PD> CDT<PD> {
    /// Run trianglate with dummy observer
    pub fn triangulate(self) -> Triangles<PD> {
        self.triangulate_with_observer(&mut ())
    }

    /// Run triangulate with observer
    pub fn triangulate_with_observer(self, observer: &mut dyn Observer) -> Triangles<PD> {
        let mut triangles = TriangleStore::with_capacity(self.points.len() * 3);

        let initial_triangle = triangles.insert(InnerTriangle::new(
            self.points.get_id_by_y(0).unwrap(),
            self.points.head,
            self.points.tail,
        ));

        // create the advancing front with initial triangle
        let mut advancing_front = AdvancingFront::new(
            triangles.get(initial_triangle).unwrap(),
            initial_triangle,
            &self.points,
        );

        let mut context = Sweeper::new(&self.points, &mut triangles, &mut advancing_front);

        Sweeper::sweep_points(&mut context, observer);
        observer.sweep_done(&context);

        Sweeper::finalize_polygon(&mut context);
        observer.finalized(&context);

        // take result out of context
        let result = context.result;

        Triangles {
            points: self.points,
            data: self.data,
            triangles,
            result,

            next: 0,
        }
    }
}

fn parse_polyline(polyline: &[Point], points: &mut PointsBuilder) {
    // todo: batch fill points
    let mut point_iter = polyline
        .iter()
        .map(|p| (points.add_steiner_point(*p), p))
        .collect::<Vec<_>>()
        .into_iter();

    if let Some(first_point) = point_iter.next() {
        let mut last_point = first_point;
        loop {
            match point_iter.next() {
                Some(p2) => {
                    let edge = Edge::new(last_point, p2);
                    points.get_edges_mut(edge.q).unwrap().push(edge.p);
                    last_point = p2;
                }
                None => {
                    let edge = Edge::new(last_point, first_point);
                    points.get_edges_mut(edge.q).unwrap().push(edge.p);
                    break;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::io::{Read, Write};

    use rand::Rng;

    use crate::Float;

    use super::*;

    #[derive(Default)]
    struct UnitTestObservor {
        hit_count: u64,
        mis_count: u64,
        rotate_count: u64,
        legalize_step_count: u64,
        legalize_count: u64,
    }

    impl UnitTestObservor {
        pub fn hit_rate(&self) -> f64 {
            self.hit_count as f64 / (self.hit_count + self.mis_count) as f64
        }
    }

    impl Observer for UnitTestObservor {
        fn legalized(&mut self, _triangel_id: TriangleId, _context: &Sweeper) {
            self.legalize_count += 1;
        }

        fn legalize_step(&mut self, _triangle_id: TriangleId, _context: &Sweeper) {
            self.legalize_step_count += 1;
        }

        fn triangle_rotated(
            &mut self,
            _triangle_id: TriangleId,
            _opposite_triangle_id: TriangleId,
            _context: &Sweeper,
        ) {
            self.rotate_count += 1;
        }

        fn finalized(&mut self, context: &Sweeper) {
            println!(
                "points: {} triangle: {}",
                context.points.len(),
                context.triangles.len()
            );
            println!(
                "legalize: {} steps: {} rotate: {}",
                self.legalize_count, self.legalize_step_count, self.rotate_count
            );
        }
    }

    #[test]
    fn test_bird() {
        let file_path = "test_data/bird.dat";
        let points = try_load_from_file(file_path).unwrap();

        let mut cache_hit = UnitTestObservor::default();
        let sweeper = Builder::new(points).build();
        let triangles = sweeper
            .triangulate_with_observer(&mut cache_hit)
            .collect::<Vec<_>>();
        assert_eq!(triangles.len(), 273);
        assert!(cache_hit.rotate_count == 272);
    }

    #[test]
    fn test_triangles_indices() {
        let file_path = "test_data/bird.dat";
        let points = try_load_from_file(file_path).unwrap();
        let points_len = points.len();

        let sweeper = Builder::new(points).build();
        let triangles = sweeper.triangulate();
        let indices = triangles.triangle_list_indices();

        assert_eq!(indices.len(), 273 * 3);
        assert_eq!(triangles.points().len(), points_len);
    }

    #[test]
    fn test_nazca_heron() {
        let file_path = "test_data/nazca_heron.dat";
        let points = try_load_from_file(file_path).unwrap();

        let sweeper = Builder::new(points).build();
        let mut cache_hit = UnitTestObservor::default();
        let triangles = sweeper
            .triangulate_with_observer(&mut cache_hit)
            .collect::<Vec<_>>();
        assert_eq!(triangles.len(), 1034);
        assert!(cache_hit.rotate_count == 671);
    }

    #[test]
    fn test_rand() {
        let test_path = "test_data/latest_test_data";
        let points = match try_load_from_file(test_path) {
            None => {
                let mut points = Vec::<Point>::new();
                for _ in 0..100 {
                    let x: Float = rand::thread_rng().gen_range(0.0..800.);
                    let y: Float = rand::thread_rng().gen_range(0.0..800.);
                    points.push(Point::new(x, y));
                }
                save_to_file(&points, test_path);
                points
            }
            Some(points) => points,
        };

        let sweeper = Builder::new(vec![
            Point::new(-10., -10.),
            Point::new(810., -10.),
            Point::new(810., 810.),
            Point::new(-10., 810.),
        ])
        .add_steiner_points(points)
        .add_hole(vec![
            Point::new(400., 400.),
            Point::new(600., 400.),
            Point::new(600., 600.),
            Point::new(400., 600.),
        ])
        .build();
        let _ = sweeper.triangulate();

        delete_file(test_path);
    }

    fn try_load_from_file(path: &str) -> Option<Vec<Point>> {
        let mut f = std::fs::File::options().read(true).open(path).ok()?;
        let mut value = "".to_string();
        f.read_to_string(&mut value).unwrap();
        let mut points = vec![];
        for line in value.lines() {
            let mut iter = line.split_whitespace();
            let x = iter.next().unwrap();
            let y = iter.next().unwrap();

            let x = x.parse::<Float>().unwrap();
            let y = y.parse::<Float>().unwrap();
            points.push(Point::new(x, y));
        }

        Some(points)
    }

    fn save_to_file(points: &[Point], path: &str) {
        use std::fmt::Write;

        let mut f = std::fs::File::options()
            .write(true)
            .create_new(true)
            .open(path)
            .unwrap();

        let mut value = "".to_string();
        for p in points {
            writeln!(value, "{} {}", p.x, p.y).unwrap();
        }

        f.write_all(value.as_bytes()).unwrap();
    }

    fn delete_file(path: &str) {
        std::fs::remove_file(path).unwrap();
    }
}
