use poly2tri_rs::{Builder, Float, Point};
use rand::Rng;
mod utils;

fn main() {
    let mut points = Vec::<Point>::new();
    for _ in 0..100 {
        let x: Float = rand::thread_rng().gen_range(0.0..800.);
        let y: Float = rand::thread_rng().gen_range(0.0..800.);
        points.push(Point::new(x, y));
    }

    let builder = Builder::new(vec![
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
    ]);
    let sweeper = builder.build();

    let triangles = sweeper.triangulate();
    utils::draw_svg(triangles, "square_with_hole.svg".into());
}
