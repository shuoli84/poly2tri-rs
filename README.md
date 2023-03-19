# poly2tri-rs

An idiomatic and fast(not just because the language) rust porting for `poly2tri` cpp project. It calculates CDT (Constrained Delaunay Triangulation) on a polygon.

# Install

Add following to cargo.toml
```toml
[dependencies]
poly2tri-rs = "0.1"
```

Or

```bash
cargo add poly2tri-rs
```

# Features

* Multiple hole and steiner points
* Fast
* Test coverage, there are both stable and random tests(prop test). Actually discovered and fixed several bugs from cpp lib.

## Performance
Introduced a bunch of perf optimizations to meet the cpp version, without
breaking the type system.

Timing at the time of writing on my m1 mbp:

|  example     |  poly2tri-rs  |  poly2tri (cpp) |  point count |
| --------     |  ------------ |  -------------- |  ----------- |
|  bird        |   0.08ms      |    0.17ms       |  275         |
|  debug       |   0.06ms       |    0.14ms       |  200         |
|  nazca_heron |   0.4ms      |    0.55ms       |  1036        |
|  nazca_monkey |  0.52ms      |    0.76ms       |  1204        |

# Limitations

* Only one polyline supported. For multiple polyline, need to preprocess them.
* Duplicate point not supported. (yet)

# Examples

## A square with a hole

![Picture](https://raw.githubusercontent.com/shuoli84/assets/main/square_with_hole.svg)

```bash
cargo run --example square && open square_with_hole.svg
```

Sample code

``` rust
fn example() {
    // randomly generate steinier points (point with no edge)
    let mut points = Vec::<Point>::new();
    for _ in 0..100 {
        let x: f64 = rand::thread_rng().gen_range(0.0..800.);
        let y: f64 = rand::thread_rng().gen_range(0.0..800.);
        points.push(Point::new(x, y));
    }

    // outer square
    let builder = SweeperBuilder::new(vec![
        Point::new(-10., -10.),
        Point::new(810., -10.),
        Point::new(810., 810.),
        Point::new(-10., 810.),
    ])
    .add_steiner_points(points)
    // square hole
    .add_hole(vec![
        Point::new(400., 400.),
        Point::new(600., 400.),
        Point::new(600., 600.),
        Point::new(400., 600.),
    ]);

    // consume builder and grab a sweeper
    let sweeper = builder.build();

    // triangulate with cdt
    let triangles = sweeper.triangulate();

    // draw
    draw(triangles, "square_with_hole.svg".into());
}
```

## Draw test data from poly2tri's testbed

```bash
# clone repos
git clone git@github.com:jhasse/poly2tri.git
git clone git@github.com:shuoli84/poly2tri-rs.git

cd poly2tri-rs
cargo run --example draw --release -- --path ../poly2tri/testbed/data/funny.dat --output funny.svg 

# open the svg
open funny.svg
```