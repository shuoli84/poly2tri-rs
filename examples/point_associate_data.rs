use poly2tri_rs::{Builder, Point};

fn main() {
    // user data type i32
    let sweeper = Builder::<i32>::new((
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

    let points = triangles.points();
    let data = triangles.data();
    assert_eq!(points.len(), data.len());
    assert_eq!(data, &[30, 31, 32, 33, 34, 35]);
}
