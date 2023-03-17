use poly2tri_rs::Triangle;

pub fn draw_svg(triangles: impl Iterator<Item = Triangle>, path: std::path::PathBuf) {
    use svg::Document;
    use svg::Node;

    // calculate view box
    let mut x_min = f64::MAX;
    let mut x_max = f64::MIN;
    let mut y_min = f64::MAX;
    let mut y_max = f64::MIN;

    let mut doc = Document::new().set("style", "background-color: #F5F5F5");

    for t in triangles {
        let data = svg::node::element::path::Data::new()
            .move_to((t.points[0].x, t.points[0].y))
            .line_to((t.points[1].x, t.points[1].y))
            .line_to((t.points[2].x, t.points[2].y))
            .close();

        doc.append(
            svg::node::element::Path::new()
                .set("d", data)
                .set("stroke", "white")
                .set("fill", "#29B6F6"),
        );

        for i in 0..3 {
            x_min = x_min.min(t.points[i].x);
            x_max = x_max.max(t.points[i].x);
            y_min = y_min.min(t.points[i].y);
            y_max = y_max.max(t.points[i].y);
        }
    }

    let doc = doc.set("viewBox", (x_min, y_min, x_max - x_min, y_max - y_min));
    svg::save(path, &doc).unwrap();
}
