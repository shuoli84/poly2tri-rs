use std::io::{Read, Write};

use poly2tri_rs::{Point, SweeperBuilder};
use rand::Rng;

fn main() {
    test_forever_rand();
}

fn test_forever_rand() {
    let mut idx = 0;
    loop {
        idx += 1;
        println!("run {idx}");
        test_rand();
    }
}

fn test_rand() {
    // attach_debugger();
    let file_path = "test_data/lastest_test_data";
    let points = if let Some(points) = try_load_from_file(file_path) {
        points
    } else {
        let mut points = Vec::<Point>::new();
        for _ in 0..100 {
            let x: f64 = rand::thread_rng().gen_range(0.0..800.);
            let y: f64 = rand::thread_rng().gen_range(0.0..800.);
            points.push(Point::new(x, y));
        }
        save_to_file(&points, file_path);
        points
    };
    let sweeper = SweeperBuilder::new(vec![
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
    delete_file(file_path);
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
        let x = x.parse::<f64>().unwrap();
        let y = y.parse::<f64>().unwrap();
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
