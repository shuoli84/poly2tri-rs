mod advancing_front;
mod context;
pub mod loader;
mod points;
mod shape;
mod sweeper;
mod triangles;
mod utils;
pub use sweeper::{Observer, Sweeper, SweeperBuilder};

/// exported to enable observer
pub use context::Context;
pub use points::PointId;
pub use shape::{Edge, Point};
pub use triangles::TriangleId;

#[cfg(feature = "f32")]
pub type Float = f32;

#[cfg(not(feature = "f32"))]
pub type Float = f64;

pub trait PointDataSource {
    type Data;

    fn points_and_data(self) -> (Vec<Point>, Vec<Self::Data>);
}

impl PointDataSource for Vec<Point> {
    type Data = ();

    fn points_and_data(self) -> (Vec<Point>, Vec<Self::Data>) {
        let data = vec![(); self.len()];
        (self, data)
    }
}

impl<D> PointDataSource for (Vec<Point>, Vec<D>) {
    type Data = D;

    fn points_and_data(self) -> (Vec<Point>, Vec<Self::Data>) {
        (self.0, self.1)
    }
}

pub trait AsPoint {
    type Data;

    fn point(&self) -> Point;

    fn data(self) -> Self::Data;
}

impl AsPoint for Point {
    type Data = ();

    #[inline]
    fn point(&self) -> Point {
        *self
    }

    #[inline]
    fn data(self) -> Self::Data {
        ()
    }
}

impl<T> AsPoint for (Point, T) {
    type Data = T;

    #[inline]
    fn point(&self) -> Point {
        self.0
    }

    #[inline]
    fn data(self) -> Self::Data {
        self.1
    }
}

impl<T> AsPoint for ([Float; 2], T) {
    type Data = T;

    #[inline]
    fn point(&self) -> Point {
        Point::new(self.0[0], self.0[1])
    }

    #[inline]
    fn data(self) -> Self::Data {
        self.1
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Triangle {
    pub points: [Point; 3],
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_customized_point() {
        let sweeper = SweeperBuilder::<i32>::new((
            vec![Point::new(0., 1.), Point::new(0., 2.), Point::new(0., 3.)],
            vec![30, 31, 32],
        ))
        .add_steiner_point(Point::new(1., 2.), 33)
        .add_holes(vec![(
            vec![Point::new(3.0, 4.0), Point::new(3.0, 5.0)],
            vec![34, 35],
        )])
        .build();

        let triangles = sweeper.triangulate();

        assert_eq!(triangles.data().to_vec(), vec![30, 31, 32, 33, 34, 35]);
    }
}
