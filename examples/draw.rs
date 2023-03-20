/// This example is like a visual debugger, it can draw each step
use clap::Parser;
use poly2tri_rs::{
    loader::{Loader, PlainFileLoader},
    Context, Edge, Float, Observer, Point, Sweeper, TriangleId,
};
use utils::draw_svg;
mod utils;

/// Simple program to greet a person
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Name of the person to greet
    #[arg(short, long)]
    path: Option<std::path::PathBuf>,

    #[arg(short, long)]
    output: Option<std::path::PathBuf>,

    #[arg(long)]
    point: bool,

    #[arg(long)]
    edge: bool,

    #[arg(long, default_value = "false")]
    test: bool,

    #[arg(long, default_value = "false")]
    debug: bool,

    #[arg(long, default_value = "1")]
    bench_count: usize,

    #[arg(long, default_value = "1000")]
    frame_count: usize,
}

fn main() {
    let args = Args::parse();

    let sweeper_builder = {
        let mut file_loader = PlainFileLoader::default();
        file_loader
            .load(args.path.as_ref().unwrap().as_os_str().to_str().unwrap())
            .unwrap()
    };

    if let Some(output_path) = args.output {
        // draw result instead of debug

        draw_svg(sweeper_builder.build().triangulate(), output_path);

        return;
    }

    if args.bench_count == 1 {
        let mut observer = DrawObserver::new(&args);
        let _result = sweeper_builder
            .clone()
            .build()
            .triangulate_with_observer(&mut observer);
        observer.save();

        // measure time with dummy observer
        let start = std::time::Instant::now();
        let count = 1000;
        for _ in 0..count {
            let _ = sweeper_builder.clone().build().triangulate();
        }
        let end = std::time::Instant::now();
        let duration = end.duration_since(start) / count;
        println!("{:?} per draw", duration);
    } else {
        for _ in 0..args.bench_count {
            let _ = sweeper_builder.clone().build().triangulate();
        }
    }
}

struct DrawObserver {
    messages: Vec<String>,
    debug: bool,
    // whether show debug info, like point_id, locations, triangle, messages
    point: bool,
    edge: bool,

    frame_count: usize,

    legalizing: Option<TriangleId>,

    // whether all process done
    finalized: bool,

    /// how many legalize steps triggered
    legalized_step_count: u64,
    point_count: u64,
    edge_count: u64,
    rotate_count: u64,

    /// svgs
    frames: Vec<String>,
    frame_messages: Vec<Vec<String>>,
}

impl DrawObserver {
    fn new(args: &Args) -> Self {
        Self {
            debug: args.debug,
            point: args.point,
            edge: args.edge,
            messages: Default::default(),
            legalizing: Default::default(),
            finalized: false,
            frames: vec![],
            frame_messages: vec![],
            legalized_step_count: 0,
            point_count: 0,
            edge_count: 0,
            rotate_count: 0,
            frame_count: args.frame_count,
        }
    }

    fn save(&self) {
        use askama::Template;

        #[derive(Template)] // this will generate the code...
        #[template(path = "draw_template.html")] // using the template in this path, relative
                                                 // to the `templates` dir in the crate root

        struct DrawTemplate<'a> {
            // the name of the struct can be anything
            frames: &'a [String],
            frame_messages: &'a [Vec<String>],
            legalized_count: u64,
            point_count: u64,
            edge_count: u64,
            rotate_count: u64,
        }

        let html_content = DrawTemplate {
            frames: self.frames.as_slice(),
            frame_messages: &self.frame_messages,
            legalized_count: self.legalized_step_count,
            point_count: self.point_count,
            edge_count: self.edge_count,
            rotate_count: self.rotate_count,
        }
        .render()
        .unwrap();

        std::fs::write("draw.html", html_content).unwrap();
    }
}

impl Observer for DrawObserver {
    fn exit_point_event(&mut self, point_id: poly2tri_rs::PointId, context: &Context) {
        self.point_count += 1;
        if !self.point {
            return;
        }
        let point = context.points.get_point(point_id).unwrap();
        self.messages
            .push(format!("point event: {point_id:?} {point:?}"));
        self.draw(context);
    }

    fn edge_event(&mut self, edge: Edge, context: &Context) {
        self.edge_count += 1;
        if !self.edge {
            return;
        }
        self.messages.push(format!(
            "edge_event: p:{} q:{}",
            edge.p.as_usize(),
            edge.q.as_usize(),
        ));

        self.draw(context);
    }

    fn will_legalize(&mut self, triangle_id: TriangleId, _context: &Context) {
        self.legalizing = Some(triangle_id);
    }

    fn legalize_step(&mut self, _triangle_id: TriangleId, _context: &Context) {
        // do nothing for now
        self.legalized_step_count += 1;
    }

    fn triangle_rotated(
        &mut self,
        _triangle_id: TriangleId,
        _opposite_triangle_id: TriangleId,
        _context: &Context,
    ) {
        self.rotate_count += 1;
    }

    fn legalized(&mut self, _triangle_id: TriangleId, _context: &Context) {
        self.legalizing = None;
    }

    fn sweep_done(&mut self, context: &Context) {
        self.messages.push("sweep done".into());
        self.draw(context);
    }

    fn finalized(&mut self, context: &Context) {
        self.messages.push("finalized".into());
        self.finalized = true;
        self.draw(context);
    }
}

impl DrawObserver {
    fn draw(&mut self, context: &Context) {
        if self.frames.len() >= self.frame_count {
            return;
        }

        use svg::Document;
        use svg::Node;

        #[derive(Debug, Clone, Copy)]
        struct MapRect {
            x: Float,
            y: Float,
            w: Float,
            h: Float,
        }

        // map rect with y flipped, svg's coordinate with origin at left-top
        #[derive(Debug)]
        struct Map {
            from: MapRect,
            to: MapRect,
        }

        impl Map {
            fn map_point(&self, x: Float, y: Float) -> (Float, Float) {
                let x = (x - self.from.x) / self.from.w * self.to.w + self.to.x;
                let y = self.to.h - (y - self.from.y) / self.from.h * self.to.h + self.to.y;
                (x, y)
            }
        }

        let mut min_x = Float::MAX;
        let mut max_x = Float::MIN;
        let mut min_y = Float::MAX;
        let mut max_y = Float::MIN;
        for p in context.points.iter().map(|(_, p, _)| p) {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }

        let w = max_x - min_x;
        let space = w * 0.05; // give some space

        let from = MapRect {
            x: min_x - space,
            y: min_y - space,
            w: max_x - min_x + 2. * space,
            h: max_y - min_y + 2. * space,
        };

        let to = if from.w <= 100. {
            MapRect {
                x: 0.,
                y: 0.,
                w: 800.,
                h: 800.,
            }
        } else {
            from
        };
        let map = Map { from, to };

        let mut doc = Document::new()
            .set("viewBox", (to.x, to.y, to.w, to.h))
            .set("style", "background-color: #F5F5F5");

        if !self.finalized {
            let point_r = from.w / 200.;

            for (id, point, edges) in context.points.iter() {
                let (x, y) = map.map_point(point.x, point.y);

                if self.debug {
                    doc.append(text(
                        format!("({}) ({:.2}, {:.2})", id.as_usize(), point.x, point.y),
                        (x, y),
                    ));
                }

                if self.point {
                    doc.append(circle((x, y), point_r, "red", "clear"));
                }

                for p_id in edges {
                    let p_point = context.points.get_point(p_id).unwrap();
                    let p = map.map_point(p_point.x, p_point.y);
                    let q = map.map_point(point.x, point.y);

                    doc.append(line(p, q, "black"));
                }
            }

            for (id, t) in context.triangles.iter() {
                let p0 = context.points.get_point(t.points[0]).unwrap();
                let p1 = context.points.get_point(t.points[1]).unwrap();
                let p2 = context.points.get_point(t.points[2]).unwrap();

                let p0 = map.map_point(p0.x, p0.y);
                let p1 = map.map_point(p1.x, p1.y);
                let p2 = map.map_point(p2.x, p2.y);

                doc.append(triangle(p0, p1, p2, "blue", "clear"));

                let center = ((p0.0 + p1.0 + p2.0) / 3., (p0.1 + p1.1 + p2.1) / 3.);

                let point_percent = 0.5;
                let center_percent = 1. - point_percent;

                if self.debug {
                    let p0_drifted = (
                        center.0 * center_percent + p0.0 * point_percent,
                        center.1 * center_percent + p0.1 * point_percent,
                    );
                    let p1_drifted = (
                        center.0 * center_percent + p1.0 * point_percent,
                        center.1 * center_percent + p1.1 * point_percent,
                    );
                    let p2_drifted = (
                        center.0 * center_percent + p2.0 * point_percent,
                        center.1 * center_percent + p2.1 * point_percent,
                    );

                    let color_for_idx = |idx: usize| {
                        let color = if t.is_constrained(idx) {
                            "yellow"
                        } else {
                            "gray"
                        };
                        let color = if t.neighbors[idx].invalid() {
                            "red"
                        } else {
                            color
                        };
                        let color = if t.is_delaunay(idx) { "black" } else { color };
                        color
                    };

                    doc.append(line(p0_drifted, p1_drifted, color_for_idx(2)));
                    doc.append(line(p1_drifted, p2_drifted, color_for_idx(0)));
                    doc.append(line(p2_drifted, p0_drifted, color_for_idx(1)));

                    doc.append(text(
                        format!("{}", id.as_usize()),
                        ((p0.0 + p1.0 + p2.0) / 3., (p0.1 + p1.1 + p2.1) / 3.),
                    ));
                }
            }

            if self.debug {
                for node in context.advancing_front.iter() {
                    if let Some(t) = node.triangle {
                        let t = context.triangles.get(t).unwrap();

                        let p0 = context.points.get_point(t.points[0]).unwrap();
                        let p1 = context.points.get_point(t.points[1]).unwrap();
                        let p2 = context.points.get_point(t.points[2]).unwrap();

                        let p0 = map.map_point(p0.x, p0.y);
                        let p1 = map.map_point(p1.x, p1.y);
                        let p2 = map.map_point(p2.x, p2.y);

                        doc.append(line(p0, p1, "red"));
                        doc.append(line(p1, p2, "red"));
                        doc.append(line(p2, p0, "red"));
                    }
                }
            }
        }

        let border_color = if context.result.iter().any(|t| {
            let t = context.triangles.get(*t).unwrap();

            let p0 = context.points.get_point(t.points[0]).unwrap();
            let p1 = context.points.get_point(t.points[1]).unwrap();
            let p2 = context.points.get_point(t.points[2]).unwrap();

            let p0 = map.map_point(p0.x, p0.y);
            let p1 = map.map_point(p1.x, p1.y);
            let p2 = map.map_point(p2.x, p2.y);

            distance(p0, p1) < 1. || distance(p0, p2) < 1. || distance(p1, p2) < 1.
        }) {
            "clear"
        } else {
            "white"
        };

        for t in &context.result {
            let t = context.triangles.get(*t).unwrap();

            let p0 = context.points.get_point(t.points[0]).unwrap();
            let p1 = context.points.get_point(t.points[1]).unwrap();
            let p2 = context.points.get_point(t.points[2]).unwrap();

            let p0 = map.map_point(p0.x, p0.y);
            let p1 = map.map_point(p1.x, p1.y);
            let p2 = map.map_point(p2.x, p2.y);

            doc.append(triangle(p0, p1, p2, border_color, "blue"));
        }

        let mut draw_illegal_triangle = |tid: TriangleId, fill_color: &str, border_color: &str| {
            let t = tid.get(&context.triangles);
            let p0 = context.points.get_point(t.points[0]).unwrap();
            let p1 = context.points.get_point(t.points[1]).unwrap();
            let p2 = context.points.get_point(t.points[2]).unwrap();

            {
                let p0 = map.map_point(p0.x, p0.y);
                let p1 = map.map_point(p1.x, p1.y);
                let p2 = map.map_point(p2.x, p2.y);

                doc.append(triangle(p0, p1, p2, fill_color, border_color));
            }
        };

        let illegal_pairs = Sweeper::<()>::illegal_triangles(context);
        for (from_tid, to_tid) in illegal_pairs {
            let from_t = from_tid.get(&context.triangles);
            let to_t = to_tid.get(&context.triangles);

            let (from_i, to_i) = from_t.common_edge_index(to_t).unwrap();

            let p = from_t.points[from_i];
            let (illegal, det) = in_circle(
                context.points.get_point(p).unwrap(),
                context.points.get_point(from_t.point_ccw(p)).unwrap(),
                context.points.get_point(from_t.point_cw(p)).unwrap(),
                context.points.get_point(to_t.points[to_i]).unwrap(),
            );

            self.messages.push(format!("illegal: {illegal} det: {det}"));

            draw_illegal_triangle(from_tid, "red", "red");
            draw_illegal_triangle(to_tid, "yellow", "red");
        }

        self.frames.push(doc.to_string());
        self.frame_messages.push(std::mem::take(&mut self.messages));
    }
}

fn line(p: (Float, Float), q: (Float, Float), color: &str) -> svg::node::element::Line {
    svg::node::element::Line::new()
        .set("class", "edge")
        .set("stroke", to_color(color))
        .set("x1", p.0)
        .set("y1", p.1)
        .set("x2", q.0)
        .set("y2", q.1)
}

fn text(content: impl Into<String>, p: (Float, Float)) -> svg::node::element::Text {
    svg::node::element::Text::new()
        .add(svg::node::Text::new(content))
        .set("x", p.0)
        .set("y", p.1)
}

fn triangle(
    p0: (Float, Float),
    p1: (Float, Float),
    p2: (Float, Float),
    border_color: &str,
    fill_color: &str,
) -> svg::node::element::Path {
    let data = svg::node::element::path::Data::new()
        .move_to(p0)
        .line_to(p1)
        .line_to(p2)
        .close();

    svg::node::element::Path::new()
        .set("d", data)
        .set("stroke", to_color(border_color))
        .set("fill", to_color(fill_color))
}

fn distance(l: (Float, Float), r: (Float, Float)) -> Float {
    ((r.0 - l.0) * (r.0 - l.0) + (r.1 - l.1) * (r.1 - l.1)).sqrt()
}

fn circle(
    c: (Float, Float),
    r: Float,
    stroke_color: &str,
    fill_color: &str,
) -> svg::node::element::Circle {
    svg::node::element::Circle::new()
        .set("cx", c.0)
        .set("cy", c.1)
        .set("r", r)
        .set("stroke-color", to_color(stroke_color))
        .set("stroke-width", 1)
        .set("fill-color", to_color(fill_color))
}

fn to_color(name: &str) -> String {
    match name {
        "blue" => "#29B6F6",
        "yellow" => "#FFA726",
        "red" => "#EF5350",
        "black" => "#3E2723",
        "gray" => "#616161",
        "clear" => "#00000000",
        _ => name,
    }
    .into()
}

/// both check and returns the det value
fn in_circle(pa: Point, pb: Point, pc: Point, pd: Point) -> (bool, Float) {
    let adx = pa.x - pd.x;
    let ady = pa.y - pd.y;
    let bdx = pb.x - pd.x;
    let bdy = pb.y - pd.y;

    let adxbdy = adx * bdy;
    let bdxady = bdx * ady;
    let oabd = adxbdy - bdxady;

    if oabd <= 0. {
        return (false, 0.);
    }

    let cdx = pc.x - pd.x;
    let cdy = pc.y - pd.y;

    let cdxady = cdx * ady;
    let adxcdy = adx * cdy;
    let ocad = cdxady - adxcdy;

    if ocad <= 0. {
        return (false, 0.);
    }

    let bdxcdy = bdx * cdy;
    let cdxbdy = cdx * bdy;

    let alift = adx * adx + ady * ady;
    let blift = bdx * bdx + bdy * bdy;
    let clift = cdx * cdx + cdy * cdy;

    let det = alift * (bdxcdy - cdxbdy) + blift * ocad + clift * oabd;

    (det > 0., det)
}
