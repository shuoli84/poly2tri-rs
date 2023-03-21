use criterion::{criterion_group, criterion_main, Criterion};
use poly2tri_rs::{Builder, Float, Point};

fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("bench_100", |b| {
        let points = parse_points(include_str!("../test_data/random_100"));
        b.iter(|| {
            let sweeper = Builder::new(vec![
                Point::new(-10., -10.),
                Point::new(810., -10.),
                Point::new(810., 810.),
                Point::new(-10., 810.),
            ])
            .add_steiner_points(points.clone())
            .add_hole(vec![
                Point::new(400., 400.),
                Point::new(600., 400.),
                Point::new(600., 600.),
                Point::new(400., 600.),
            ])
            .build();

            let _result = sweeper.triangulate();
        })
    });

    c.bench_function("bench_bird", |b| {
        let points = parse_points(include_str!("../test_data/bird.dat"));
        b.iter(|| {
            let sweeper = Builder::new(points.clone()).build();
            let _result = sweeper.triangulate();
        })
    });

    c.bench_function("bench_nazca_heron", |b| {
        let points = parse_points(include_str!("../test_data/nazca_heron.dat"));
        b.iter(|| {
            let sweeper = Builder::new(points.clone()).build();
            let _result = sweeper.triangulate();
        })
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

fn parse_points(serialized: &str) -> Vec<Point> {
    let mut points = vec![];
    for line in serialized.lines() {
        let mut iter = line.split_whitespace();
        let x = iter.next().unwrap();
        let y = iter.next().unwrap();
        let x = x.parse::<Float>().unwrap();
        let y = y.parse::<Float>().unwrap();
        points.push(Point::new(x, y));
    }
    points
}
