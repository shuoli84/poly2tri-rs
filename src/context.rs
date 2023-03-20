use crate::{
    advancing_front::{AdvancingFront, NodeId, NodeRef},
    points::Points,
    shape::InnerTriangle,
    triangles::TriangleStore,
    utils::{in_circle, in_scan_area, orient_2d, Angle, Orientation},
    Edge, Float, Observer, Point, PointId, TriangleId,
};

pub struct Context<'a> {
    pub points: &'a Points,
    pub triangles: &'a mut TriangleStore,
    pub advancing_front: &'a mut AdvancingFront,
    pub result: Vec<TriangleId>,

    // reusable legalize task queue to reduce alloc overhead
    pub(crate) legalize_task_queue: Vec<TriangleId>,
    // reusable legalize remap triangle ids to reduce alloc overhead
    pub(crate) legalize_remap_tids: Vec<TriangleId>,
    // reusable legalize triangle id queue
    pub(crate) triangle_id_queue: Vec<TriangleId>,
}

impl<'a> Context<'a> {
    pub fn new(
        points: &'a Points,
        triangles: &'a mut TriangleStore,
        advancing_front: &'a mut AdvancingFront,
    ) -> Self {
        Self {
            points,
            triangles,
            advancing_front,
            result: Vec::with_capacity(points.len()),

            legalize_task_queue: Vec::with_capacity(32),
            legalize_remap_tids: Vec::with_capacity(32),
            triangle_id_queue: Vec::with_capacity(32),
        }
    }
}

impl Context<'_> {
    pub(crate) fn sweep_points(context: &mut Context, observer: &mut dyn Observer) {
        for (point_id, point, edges) in context.points.iter_point_by_y(1) {
            observer.enter_point_event(point_id, context);
            Self::point_event(point_id, point, context, observer);
            observer.exit_point_event(point_id, context);

            for p in edges {
                let edge = Edge { p, q: point_id };
                Self::edge_event(edge, point, context, observer);

                observer.edge_event(edge, context);
            }

            debug_assert!(Self::verify_triangles(context));
        }
    }

    pub(crate) fn finalize_polygon(context: &mut Context) -> Option<()> {
        // get an internal triangle to start with
        // the first node is head, artificial point, so skip
        let node = context.advancing_front.nth(1)?;

        let mut t = node.triangle?;

        loop {
            if let Some(tri) = context.triangles.get(t) {
                if !tri.constrained_edge_cw(node.point_id()) {
                    t = tri.neighbor_ccw(node.point_id());
                } else {
                    break;
                }
            } else {
                // no valid triangle found, just break
                break;
            }
        }

        if !t.invalid() {
            Self::clean_mesh(t, context);
        }

        Some(())
    }

    fn clean_mesh(triangle_id: TriangleId, context: &mut Context) -> Option<()> {
        // id and from, it should not trigger from again
        let mut triangles = Vec::<(TriangleId, TriangleId)>::with_capacity(context.points.len());
        triangles.push((triangle_id, TriangleId::INVALID));

        while let Some((t, from)) = triangles.pop() {
            if t.invalid() {
                continue;
            }

            let tri = context.triangles.get_mut(t).unwrap();

            if !tri.interior {
                tri.interior = true;
                context.result.push(t);

                for i in 0..3 {
                    if !tri.is_constrained(i) {
                        let new_t = tri.neighbors[i];
                        if new_t != from {
                            triangles.push((new_t, t));
                        }
                    }
                }
            }
        }

        Some(())
    }
}

// first need to visualize each step, then trouble shoot
// print detailed steps, like what changes this going to address.

/// Point event related methods
impl Context<'_> {
    fn point_event(
        point_id: PointId,
        point: Point,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) {
        let node = context.advancing_front.locate_node(point).unwrap();
        let node_id = node.get_node_id();
        let next_node = node.next().unwrap();
        let node_point = node.point();

        let triangle = context.triangles.insert(InnerTriangle::new(
            point_id,
            node.point_id(),
            next_node.point_id(),
        ));
        let node_triangle = node.triangle.unwrap();
        context.triangles.mark_neighbor(node_triangle, triangle);
        context.advancing_front.insert(point_id, point, triangle);

        Self::legalize(triangle, context, observer);

        // in middle case, the node's x should be less than point'x
        // in left case, they are same.
        if point.x <= node_point.x + Float::EPSILON {
            Self::fill_one(node_id, context, observer);
        }

        Self::fill_advancing_front(point, context, observer);
    }

    /// helper function to check wether triangle is legal
    fn is_legalize(triangle_id: TriangleId, context: &Context) -> [TriangleId; 3] {
        let mut result = [TriangleId::INVALID; 3];
        for point_idx in 0..3 {
            let triangle = context.triangles.get_unchecked(triangle_id);
            let opposite_triangle_id = triangle.neighbors[point_idx];
            let Some(opposite_triangle) = context.triangles.get(opposite_triangle_id) else {
                continue;
            };

            let p = triangle.points[point_idx];
            let op = opposite_triangle.opposite_point(&triangle, p);
            let oi = opposite_triangle.point_index(op).unwrap();

            if opposite_triangle.is_constrained(oi) {
                continue;
            }

            let inside = unsafe {
                in_circle(
                    context.points.get_point_uncheck(p),
                    context.points.get_point_uncheck(triangle.point_ccw(p)),
                    context.points.get_point_uncheck(triangle.point_cw(p)),
                    context.points.get_point_uncheck(op),
                )
            };

            if inside {
                result[point_idx] = opposite_triangle_id;
            }
        }

        result
    }

    /// legalize the triangle
    fn legalize(triangle_id: TriangleId, context: &mut Context, observer: &mut dyn Observer) {
        observer.will_legalize(triangle_id, context);

        // keeps record of all touched triangles, after legalize finished
        // need to remap all to the advancing front
        let mut legalized_triangles = std::mem::take(&mut context.legalize_remap_tids);

        // record the task and who triggered it
        let mut task_queue = std::mem::take(&mut context.legalize_task_queue);
        task_queue.push(triangle_id);
        legalized_triangles.push(triangle_id);

        while let Some(triangle_id) = task_queue.pop() {
            for point_idx in 0..3 {
                let triangle = triangle_id.get(&context.triangles);
                // skip legalize for constrained_edge
                if triangle.is_constrained(point_idx) || triangle.is_delaunay(point_idx) {
                    continue;
                }

                let opposite_triangle_id = triangle.neighbors[point_idx];
                if opposite_triangle_id.invalid() {
                    continue;
                };
                let opposite_triangle = opposite_triangle_id.get(&context.triangles);

                let p = triangle.points[point_idx];
                let op = opposite_triangle.opposite_point(&triangle, p);

                let illegal = in_circle(
                    p.get(&context.points),
                    triangle.point_ccw(p).get(&context.points),
                    triangle.point_cw(p).get(&context.points),
                    op.get(&context.points),
                );
                if illegal {
                    observer.triangle_rotated(triangle_id, opposite_triangle_id, context);
                    // rotate shared edge one vertex cw to legalize it
                    let need_remap = Self::rotate_triangle_pair(
                        triangle_id,
                        p,
                        opposite_triangle_id,
                        op,
                        context.triangles,
                    );

                    // set the delaunay flag for the edge we just fixed
                    {
                        let (t, ot) = unsafe {
                            context
                                .triangles
                                .get_mut_two(triangle_id, opposite_triangle_id)
                        };

                        let (t_idx, ot_idx) = t.common_edge_index(ot).unwrap();
                        t.set_delaunay(t_idx, true);
                        ot.set_delaunay(ot_idx, true);
                    }

                    task_queue.push(triangle_id);
                    task_queue.push(opposite_triangle_id);

                    if need_remap {
                        legalized_triangles.push(triangle_id);
                        legalized_triangles.push(opposite_triangle_id);
                    }
                    break;
                } else {
                    // though we can set delaunay edge to prevent future recalulate
                    // it turns out slower, it means the recalculation is not many
                }
            }

            observer.legalize_step(triangle_id, context);
        }

        for triangle_id in legalized_triangles.drain(..) {
            Self::map_triangle_to_nodes(triangle_id, context);
        }

        {
            // give back the task queue
            context.legalize_task_queue = task_queue;
            context.legalize_remap_tids = legalized_triangles;
        }

        observer.legalized(triangle_id, context);
    }

    /// Rotate the triangle pair, returns two flag indicate (t, ot) whether candidate for af remap
    fn rotate_triangle_pair(
        t_id: TriangleId,
        p: PointId,
        ot_id: TriangleId,
        op: PointId,
        triangles: &mut TriangleStore,
    ) -> bool {
        let (t, ot) = unsafe { triangles.get_mut_two(t_id, ot_id) };

        let n1 = t.neighbor_ccw(p);
        let n2 = t.neighbor_cw(p);
        let n3 = ot.neighbor_ccw(op);
        let n4 = ot.neighbor_cw(op);

        let ea1 = t.edge_attr_ccw(p);
        let ea2 = t.edge_attr_cw(p);
        let ea3 = ot.edge_attr_ccw(op);
        let ea4 = ot.edge_attr_cw(op);

        // rotate shared edge one vertex cw to legalize it
        t.rotate_cw(p, op);
        ot.rotate_cw(op, p);

        t.set_edge_attr_cw(p, ea2);
        t.set_edge_attr_ccw(op, ea3);
        ot.set_edge_attr_ccw(p, ea1);
        ot.set_edge_attr_cw(op, ea4);

        t.clear_neighbors();
        ot.clear_neighbors();

        TriangleStore::mark_neighbor_for_two_mut(t_id, ot_id, t, ot);

        let (t, ot, t_n1, t_n2, t_n3, t_n4) =
            unsafe { triangles.get_mut_six(t_id, ot_id, n1, n2, n3, n4) };

        if let Some(t_n2) = t_n2 {
            TriangleStore::mark_neighbor_for_two_mut(t_id, n2, t, t_n2);
        }
        if let Some(t_n3) = t_n3 {
            TriangleStore::mark_neighbor_for_two_mut(t_id, n3, t, t_n3);
        }
        if let Some(t_n1) = t_n1 {
            TriangleStore::mark_neighbor_for_two_mut(ot_id, n1, ot, t_n1);
        }
        if let Some(t_n4) = t_n4 {
            TriangleStore::mark_neighbor_for_two_mut(ot_id, n4, ot, t_n4);
        }

        n1.invalid() || n2.invalid() || n3.invalid() || n4.invalid()
    }

    /// update advancing front node's triangle
    fn map_triangle_to_nodes(triangle_id: TriangleId, context: &mut Context) {
        let triangle = triangle_id.get(&context.triangles);
        for i in 0..3 {
            if triangle.neighbors[i].invalid() {
                let point = unsafe {
                    context
                        .points
                        .get_point_uncheck(triangle.point_cw(triangle.points[i]))
                };
                context.advancing_front.update_triangle(point, triangle_id);
            }
        }
    }

    /// fill the node with one triangle.
    /// Note: The moment it filled, advancing_front is modified.
    /// if the node is covered by another triangle, then it is deleted from advancing_front.
    /// all following advancing front lookup is affected.
    /// Returns the new next's point info.
    fn fill_one(
        node: NodeId,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) -> Option<FillOne> {
        let node = context.advancing_front.get_node_with_id(node).unwrap();
        let prev_node = node.prev()?;
        let next_node = node.next()?;

        // After fill and legalize, next's node won't change. So we save a version here
        // why: Most of time, after fill, external code needs to query the new
        //      next/prev and check whether more work needs to do. The checking logic
        //      only requires point info.
        let fill_one_result = FillOne {
            prev: prev_node.get_node_id(),
            next: next_node.get_node_id(),
        };

        let new_triangle = context.triangles.insert(InnerTriangle::new(
            prev_node.point_id(),
            node.point_id(),
            next_node.point_id(),
        ));

        context
            .triangles
            .mark_neighbor(new_triangle, prev_node.triangle.unwrap());
        context
            .triangles
            .mark_neighbor(new_triangle, node.triangle.unwrap());

        // update prev_node's triangle to newly created and delete the node.
        // node is covered by new triangle.
        // safety: prev_node and node is valid till this point, advanceing_front can not changed
        //       under the hood, so the index is still valid
        unsafe {
            context.advancing_front.update_and_delete_by_index(
                prev_node.index(),
                prev_node.point_id(),
                new_triangle,
                node.index(),
            )
        };

        // legalize works on existing triangles, no new triangle will be created
        // that ganrentees next point won't change
        Self::legalize(new_triangle, context, observer);

        Some(fill_one_result)
    }

    fn fill_advancing_front(node_point: Point, context: &mut Context, observer: &mut dyn Observer) {
        let node_id = context
            .advancing_front
            .get_node(node_point)
            .unwrap()
            .get_node_id();

        {
            // fill right holes
            let mut node_id = node_id.clone();
            while let Some(next_node) = context.advancing_front.locate_next_node(node_id) {
                if next_node.next().is_some() {
                    // if HoleAngle exceeds 90 degrees then break
                    if Self::should_fill(&next_node) {
                        break;
                    }
                    let next_node_id = next_node.get_node_id();

                    node_id = match Self::fill_one(next_node_id, context, observer) {
                        Some(fill_one) => fill_one.next,
                        None => next_node_id,
                    };
                } else {
                    break;
                }
            }
        }

        {
            // fill left holes
            let mut node_id = node_id.clone();

            while let Some(prev_node) = context.advancing_front.locate_prev_node(node_id) {
                if prev_node.prev().is_some() {
                    // if HoleAngle exceeds 90 degrees then break
                    if Self::should_fill(&prev_node) {
                        break;
                    }

                    node_id = prev_node.get_node_id();
                    Self::fill_one(node_id, context, observer);
                } else {
                    break;
                }
            }
        }

        // fill right basins
        if Self::basin_angle_satisfy(node_id, context) {
            Self::fill_basin(node_id, context, observer);
        }
    }

    fn should_fill(node: &NodeRef) -> bool {
        let next = node.next().unwrap();
        let prev_node = node.prev().unwrap();

        let angle = crate::utils::Angle::new(node.point(), next.point(), prev_node.point());

        if angle.is_negative() {
            // negative means next -> node -> prev is cw, then it is not a hole
            return true;
        } else if !angle.exceeds_90_degree() {
            // we only fill holes with angle less than PI/2
            // otherwise we generate many bad shaped triangles
            // that needs rotated later
            return false;
        }

        if let Some(next_next) = next.next() {
            if Angle::new(node.point(), next_next.point(), prev_node.point())
                .between_0_to_90_degree()
            {
                return false;
            }
        }

        if let Some(prev_prev) = prev_node.prev() {
            if Angle::new(node.point(), next.point(), prev_prev.point()).between_0_to_90_degree() {
                return false;
            }
        }

        true
    }
}

struct FillOne {
    prev: NodeId,
    next: NodeId,
}

#[derive(Debug)]
struct ConstrainedEdge {
    constrained_edge: Edge,
    p: Point,
    q: Point,
    /// Whether the constrained edge is "right" edge, p.x larger than q.x
    right: bool,
}

impl ConstrainedEdge {
    fn p_id(&self) -> PointId {
        self.constrained_edge.p
    }

    fn q_id(&self) -> PointId {
        self.constrained_edge.q
    }

    fn with_q(&self, q: PointId, context: &Context) -> Self {
        let q_point = q.get(&context.points);
        Self {
            constrained_edge: Edge {
                p: self.constrained_edge.p,
                q,
            },
            p: self.p,
            q: q_point,
            right: self.p.x > q_point.x,
        }
    }
}

/// EdgeEvent related methods
impl Context<'_> {
    fn edge_event(edge: Edge, q: Point, context: &mut Context, observer: &mut dyn Observer) {
        let p = edge.p.get(&context.points);

        let constrain_edge = ConstrainedEdge {
            constrained_edge: edge,
            p,
            q,
            right: p.x > q.x,
        };

        {
            // check and fill
            let node = context.advancing_front.get_node_with_cache(q).unwrap();

            let triangle_id = node.triangle.unwrap();
            let node_id = node.get_node_id();
            if Self::try_mark_edge_for_triangle(edge.p, edge.q, triangle_id, context) {
                // the edge is already an edge of the triangle, return
                return;
            }

            // for now we will do all needed filling
            Self::fill_edge_event(&constrain_edge, node_id, context, observer);
        }

        // node's triangle may changed, get the latest
        let triangle = context
            .advancing_front
            .get_node_with_cache(q)
            .unwrap()
            .triangle
            .unwrap();

        // this triangle crosses constraint so let's flippin start!
        let mut triangle_ids = std::mem::take(&mut context.triangle_id_queue);
        Self::edge_event_process(
            edge.p,
            edge.q,
            &constrain_edge,
            triangle,
            edge.q,
            &mut triangle_ids,
            context,
        );

        for triangle_id in triangle_ids.drain(..) {
            Self::legalize(triangle_id, context, observer);
        }
        context.triangle_id_queue = triangle_ids;
    }

    /// try mark edge for triangle if the constrained edge already is a edge
    /// returns `true` if yes, otherwise `false`
    fn try_mark_edge_for_triangle(
        p: PointId,
        q: PointId,
        t_id: TriangleId,
        context: &mut Context,
    ) -> bool {
        let triangle = context.triangles.get_mut_unchecked(t_id);
        let Some(index) = triangle.edge_index(p, q) else { return false; };

        triangle.set_constrained(index, true);
        let neighbor_t_id = triangle.neighbors[index];
        if !neighbor_t_id.invalid() {
            let ot = context.triangles.get_mut_unchecked(neighbor_t_id);
            let index = ot.neighbor_index(t_id);
            ot.set_constrained(index, true);
        }

        true
    }

    fn fill_edge_event(
        edge: &ConstrainedEdge,
        node_id: NodeId,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) {
        if edge.right {
            Self::fill_right_above_edge_event(edge, node_id, context, observer);
        } else {
            Self::fill_left_above_edge_event(edge, node_id, context, observer);
        }
    }

    fn fill_right_above_edge_event(
        edge: &ConstrainedEdge,
        node_id: NodeId,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) {
        let mut node_id = node_id.clone();
        while let Some(next_node) = context.advancing_front.locate_next_node(node_id) {
            if next_node.point().x >= edge.p.x {
                break;
            }

            // check if next node is below the edge
            if orient_2d(edge.q, next_node.point(), edge.p).is_ccw() {
                Self::fill_right_below_edge_event(edge, node_id, context, observer);
            } else {
                // try next node
                node_id = next_node.get_node_id();
            }
        }
    }

    fn fill_right_below_edge_event(
        edge: &ConstrainedEdge,
        node_id: NodeId,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) -> Option<()> {
        if node_id.point().x >= edge.p.x {
            return None;
        }

        let node = context.advancing_front.get_node_with_id(node_id).unwrap();

        let next_node = node.next().unwrap();
        let next_next_node = next_node.next().unwrap();

        if orient_2d(node.point(), next_node.point(), next_next_node.point()).is_ccw() {
            // concave
            Self::fill_right_concave_edge_event(edge, node_id, context, observer);
        } else {
            // convex
            Self::fill_right_convex_edge_event(edge, node_id, context, observer)?;

            // retry this one
            Self::fill_right_below_edge_event(edge, node_id, context, observer);
        }

        Some(())
    }

    /// recursively fill concave nodes
    fn fill_right_concave_edge_event(
        edge: &ConstrainedEdge,
        node_id: NodeId,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) {
        let next_id = {
            let next_node = context.advancing_front.locate_next_node(node_id).unwrap();
            let next_id = next_node.get_node_id();
            match Self::fill_one(next_id, context, observer) {
                None => {
                    // nothing changed
                    next_id
                }
                Some(fill_one) => fill_one.next,
            }
        };

        if next_id.point_id() != edge.p_id() {
            // next above or below edge?
            if orient_2d(edge.q, next_id.point(), edge.p).is_ccw() {
                let next_next_node = context.advancing_front.locate_next_node(next_id).unwrap();

                //  below
                if orient_2d(node_id.point(), next_id.point(), next_next_node.point()).is_ccw() {
                    // next is concave
                    Self::fill_right_concave_edge_event(edge, node_id, context, observer);
                } else {
                    // next is convex
                }
            }
        }
    }

    // if nothing changed, returns None
    #[must_use]
    fn fill_right_convex_edge_event(
        edge: &ConstrainedEdge,
        node_id: NodeId,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) -> Option<()> {
        let next_node = context.advancing_front.locate_next_node(node_id).unwrap();
        let next_next_node = next_node.next().unwrap();
        let next_next_next_node = next_next_node.next()?;
        // next concave or convex?
        if orient_2d(
            next_node.point(),
            next_next_node.point(),
            next_next_next_node.point(),
        )
        .is_ccw()
        {
            // concave
            Self::fill_right_concave_edge_event(edge, node_id, context, observer);
        } else {
            // convex
            // next above or below edge?
            if orient_2d(edge.q, next_next_node.point(), edge.p).is_ccw() {
                // Below
                Self::fill_right_convex_edge_event(
                    edge,
                    next_node.get_node_id(),
                    context,
                    observer,
                )?;
            } else {
                // Above
            }
        }

        Some(())
    }

    fn fill_left_above_edge_event(
        edge: &ConstrainedEdge,
        node_id: NodeId,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) {
        let mut node_id = node_id.clone();
        while let Some(prev_node) = context.advancing_front.locate_prev_node(node_id) {
            // check if next node is below the edge
            if prev_node.point().x <= edge.p.x {
                break;
            }

            if orient_2d(edge.q, prev_node.point(), edge.p).is_cw() {
                Self::fill_left_below_edge_event(edge, node_id, context, observer);
            } else {
                node_id = prev_node.get_node_id();
            }
        }
    }

    fn fill_left_below_edge_event(
        edge: &ConstrainedEdge,
        node_id: NodeId,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) -> Option<()> {
        if node_id.point().x > edge.p.x {
            let prev_node = context.advancing_front.locate_prev_node(node_id).unwrap();
            let prev_prev_node = prev_node.prev().unwrap();
            if orient_2d(node_id.point(), prev_node.point(), prev_prev_node.point()).is_cw() {
                Self::fill_left_concave_edge_event(edge, node_id, context, observer);
            } else {
                // convex
                Self::fill_left_convex_edge_event(edge, node_id, context, observer)?;

                // retry this one
                Self::fill_left_below_edge_event(edge, node_id, context, observer);
            }
            Some(())
        } else {
            None
        }
    }

    // if nothing changed, returns None
    #[must_use]
    fn fill_left_convex_edge_event(
        edge: &ConstrainedEdge,
        node_id: NodeId,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) -> Option<()> {
        // next concave or convex?
        let prev_node = context.advancing_front.locate_prev_node(node_id).unwrap();
        let prev_prev_node = prev_node.prev().unwrap();
        let prev_prev_prev_node = prev_prev_node.prev()?;

        if orient_2d(
            prev_node.point(),
            prev_prev_node.point(),
            prev_prev_prev_node.point(),
        )
        .is_cw()
        {
            // concave
            Self::fill_left_concave_edge_event(edge, prev_node.get_node_id(), context, observer);
        } else {
            // convex
            // next above or below edge?
            if orient_2d(edge.q, prev_prev_node.point(), edge.p).is_cw() {
                // below
                Self::fill_left_convex_edge_event(
                    edge,
                    prev_node.get_node_id(),
                    context,
                    observer,
                )?;
            } else {
                // above
            }
        }
        Some(())
    }

    fn fill_left_concave_edge_event(
        edge: &ConstrainedEdge,
        node_id: NodeId,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) {
        let prev_node = context.advancing_front.locate_prev_node(node_id).unwrap();

        let prev_node_id = prev_node.get_node_id();

        let prev_node_id = match Self::fill_one(prev_node_id, context, observer) {
            Some(fill_one) => fill_one.prev,
            None => prev_node_id,
        };

        if prev_node_id.point_id() != edge.p_id() {
            // next above or below edge?
            if orient_2d(edge.q, prev_node_id.point(), edge.p).is_cw() {
                let prev_node = context
                    .advancing_front
                    .get_node_with_id(prev_node_id)
                    .unwrap();
                // below
                let prev_prev_node = prev_node.prev().unwrap();
                if orient_2d(node_id.point(), prev_node.point(), prev_prev_node.point()).is_cw() {
                    // next is concave
                    Self::fill_left_concave_edge_event(edge, node_id, context, observer);
                } else {
                    // next is convex
                }
            }
        }
    }

    fn edge_event_process(
        ep: PointId,
        eq: PointId,
        constrain_edge: &ConstrainedEdge,
        triangle_id: TriangleId,
        p: PointId,
        triangle_ids: &mut Vec<TriangleId>,
        context: &mut Context,
    ) {
        assert!(!triangle_id.invalid());

        if Self::try_mark_edge_for_triangle(ep, eq, triangle_id, context) {
            return;
        }

        let triangle = context.triangles.get_mut_unchecked(triangle_id);
        let p1 = triangle.point_ccw(p);
        let o1 = orient_2d(
            eq.get(&context.points),
            p1.get(&context.points),
            ep.get(&context.points),
        );

        if o1.is_collinear() {
            if let Some(edge_index) = triangle.edge_index(eq, p1) {
                triangle.set_constrained(edge_index, true);

                let neighbor_across_t = triangle.neighbor_across(p);
                Self::edge_event_process(
                    ep,
                    p1,
                    &constrain_edge.with_q(p1, context),
                    neighbor_across_t,
                    p1,
                    triangle_ids,
                    context,
                );
                return;
            } else {
                panic!("EdgeEvent - collinear points not supported")
            }
        }

        let p2 = triangle.point_cw(p);
        let o2 = orient_2d(
            eq.get(&context.points),
            p2.get(&context.points),
            ep.get(&context.points),
        );
        if o2.is_collinear() {
            if let Some(edge_index) = triangle.edge_index(eq, p2) {
                triangle.set_constrained(edge_index, true);

                let neighbor_across_t = triangle.neighbor_across(p);
                Self::edge_event_process(
                    ep,
                    p2,
                    &constrain_edge.with_q(p2, context),
                    neighbor_across_t,
                    p2,
                    triangle_ids,
                    context,
                );

                return;
            } else {
                panic!("collinear points not supported");
            }
        }

        if o1 == o2 {
            // need to decide if we are rotating cw or ccw to get to a triangle
            // that will cross edge
            let triangle_id = if o1.is_cw() {
                triangle.neighbor_ccw(p)
            } else {
                triangle.neighbor_cw(p)
            };

            Self::edge_event_process(
                ep,
                eq,
                constrain_edge,
                triangle_id,
                p,
                triangle_ids,
                context,
            );
        } else {
            Self::flip_edge_event(
                ep,
                eq,
                constrain_edge,
                triangle_id,
                p,
                triangle_ids,
                context,
            );
        }
    }
}

/// flip edge related methods
impl Context<'_> {
    fn flip_edge_event(
        ep: PointId,
        eq: PointId,
        edge: &ConstrainedEdge,
        triangle_id: TriangleId,
        p: PointId,
        legalize_queue: &mut Vec<TriangleId>,
        context: &mut Context,
    ) {
        let t = triangle_id.get(&context.triangles);
        let ot_id = t.neighbor_across(p);
        let ot = ot_id.get(&context.triangles);
        let op = ot.opposite_point(t, p);

        if in_scan_area(
            p.get(&context.points),
            t.point_ccw(p).get(&context.points),
            t.point_cw(p).get(&context.points),
            op.get(&context.points),
        ) {
            // lets rotate shared edge one vertex cw
            if Self::rotate_triangle_pair(triangle_id, p, ot_id, op, &mut context.triangles) {
                Self::map_triangle_to_nodes(triangle_id, context);
                Self::map_triangle_to_nodes(ot_id, context);
            }
            // legalize later
            legalize_queue.extend([triangle_id, ot_id]);

            if p == eq && op == ep {
                if eq == edge.q_id() && ep == edge.p_id() {
                    context
                        .triangles
                        .get_mut_unchecked(triangle_id)
                        .set_constrained_for_edge(ep, eq);

                    context
                        .triangles
                        .get_mut_unchecked(ot_id)
                        .set_constrained_for_edge(ep, eq);
                }
            } else {
                let o = orient_2d(
                    eq.get(&context.points),
                    op.get(&context.points),
                    ep.get(&context.points),
                );

                let t = Self::next_flip_triangle(o, triangle_id, ot_id, legalize_queue);
                Self::flip_edge_event(ep, eq, edge, t, p, legalize_queue, context);
            }
        } else {
            let new_p = Self::next_flip_point(ep, eq, ot_id, op, context);
            Self::flip_scan_edge_event(
                ep,
                eq,
                edge,
                triangle_id,
                ot_id,
                new_p,
                legalize_queue,
                context,
            );
            Self::edge_event_process(ep, eq, edge, triangle_id, p, legalize_queue, context);
        }
    }

    fn next_flip_triangle(
        o: Orientation,
        t: TriangleId,
        ot: TriangleId,
        triangle_ids: &mut Vec<TriangleId>,
    ) -> TriangleId {
        if o.is_ccw() {
            // ot is not crossing edge after flip
            triangle_ids.push(ot);
            t
        } else {
            // t is not crossing edge after flip
            triangle_ids.push(t);
            ot
        }
    }

    fn next_flip_point(
        ep: PointId,
        eq: PointId,
        ot: TriangleId,
        op: PointId,
        context: &mut Context,
    ) -> PointId {
        let o2d = orient_2d(
            eq.get(&context.points),
            op.get(&context.points),
            ep.get(&context.points),
        );

        let ot = context.triangles.get_unchecked(ot);
        match o2d {
            Orientation::CW => {
                // right
                ot.point_ccw(op)
            }
            Orientation::CCW => {
                // left
                ot.point_cw(op)
            }
            Orientation::Collinear => {
                panic!("Opposing point on constrained edge");
            }
        }
    }

    fn flip_scan_edge_event(
        ep: PointId,
        eq: PointId,
        edge: &ConstrainedEdge,
        flip_triangle_id: TriangleId,
        t_id: TriangleId,
        p: PointId,
        triangle_ids: &mut Vec<TriangleId>,
        context: &mut Context,
    ) {
        let t = t_id.get(&context.triangles);
        let ot = t.neighbor_across(p);
        if ot.invalid() {
            panic!("flip_scan_edge_event - null neighbor across");
        }

        let op = ot.get(&context.triangles).opposite_point(t, p);
        let flip_triangle = flip_triangle_id.get(&context.triangles);
        let p1 = flip_triangle.point_ccw(eq);
        let p2 = flip_triangle.point_cw(eq);

        if in_scan_area(
            eq.get(&context.points),
            p1.get(&context.points),
            p2.get(&context.points),
            op.get(&context.points),
        ) {
            // flip with new edge op -> eq
            Self::flip_edge_event(eq, op, edge, ot, op, triangle_ids, context);

            // original comment:
            // TODO: Actually I just figured out that it should be possible to
            //       improve this by getting the next ot and op before the the above
            //       flip and continue the flipScanEdgeEvent here
            // set new ot and op here and loop back to inScanArea test
            // also need to set a new flip_triangle first
            // Turns out at first glance that this is somewhat complicated
            // so it will have to wait.
        } else {
            let new_p = Self::next_flip_point(ep, eq, ot, op, context);
            Self::flip_scan_edge_event(
                ep,
                eq,
                edge,
                flip_triangle_id,
                ot,
                new_p,
                triangle_ids,
                context,
            );
        }
    }
}

#[derive(Debug)]
struct Basin {
    left: Point,
    right: Point,
    width: Float,
    left_higher: bool,
}

impl Basin {
    pub fn is_shallow(&self, point: Point) -> bool {
        let height = if self.left_higher {
            self.left.y - point.y
        } else {
            self.right.y - point.y
        };

        self.width > height
    }

    pub fn completed(&self, point: Point) -> bool {
        if point.x >= self.right.x || point.x <= self.left.x {
            return true;
        }

        self.is_shallow(point)
    }
}

/// Basin related methods
impl Context<'_> {
    fn basin_angle_satisfy(node_id: NodeId, context: &Context) -> bool {
        const TAN_3_4_PI: Float = -1.;
        let Some(next) = context.advancing_front.locate_next_node(node_id) else { return false };
        let Some(next_next) = next.next() else { return false };

        let ax = node_id.point().x - next_next.point().x;
        let ay = node_id.point().y - next_next.point().y;
        // the basin angle is (1/2pi, pi), so as long as tan value is less than 3/4 pi's, then its angle is less than 3/4 pi

        // ay / ax < tan(3/4 * PI)
        if ax > 0. {
            ay < TAN_3_4_PI * ax
        } else {
            ay > TAN_3_4_PI * ax
        }
    }

    /// basin is like a bowl, we first identify it's left, bottom, right node.
    /// then fill it
    fn fill_basin(
        node_point: NodeId,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) -> Option<()> {
        let next_node = context.advancing_front.locate_next_node(node_point)?;
        let next_next_node = next_node.next()?;

        // find the left
        let left: NodeRef<'_>;
        if orient_2d(
            node_point.point(),
            next_node.point(),
            next_next_node.point(),
        )
        .is_ccw()
        {
            left = next_next_node;
        } else {
            left = next_node;
        }

        // find the bottom
        let mut bottom = left.clone();
        while let Some(next_node) = bottom.next() {
            if bottom.point().y >= next_node.point().y {
                bottom = next_node;
            } else {
                break;
            }
        }

        // no valid basin
        if bottom.point_id().eq(&left.point_id()) {
            return None;
        }

        // find the right
        let mut right: NodeRef = bottom.clone();
        while let Some(next_node) = right.next() {
            if right.point().y < next_node.point().y {
                right = next_node;
            } else {
                break;
            }
        }
        if right.point_id() == bottom.point_id() {
            // no valid basin
            return None;
        }

        let width = right.point().x - left.point().x;
        let left_higher: bool = left.point().y > right.point().y;

        Self::fill_basin_req(
            bottom.get_node_id(),
            &Basin {
                left: left.point(),
                right: right.point(),
                width,
                left_higher,
            },
            context,
            observer,
        );

        Some(())
    }

    fn fill_basin_req(
        node: NodeId,
        basin: &Basin,
        context: &mut Context,
        observer: &mut dyn Observer,
    ) -> Option<()> {
        if basin.completed(node.point()) {
            return None;
        }

        let fill_one = Self::fill_one(node, context, observer).expect("already in basin");
        let prev = fill_one.prev;
        let next = fill_one.next;

        if prev.point().eq(&basin.left) && next.point().eq(&basin.right) {
            return Some(());
        }

        let new_node = if prev.point().eq(&basin.left) {
            let next = context.advancing_front.get_node_with_id(next).unwrap();
            let next_next = next.next().unwrap();
            if orient_2d(node.point(), next.point(), next_next.point()).is_cw() {
                return None;
            }

            next.get_node_id()
        } else if next.point().eq(&basin.right) {
            let prev = context.advancing_front.get_node_with_id(prev).unwrap();
            let prev_prev = prev.prev()?;
            if orient_2d(node.point(), prev.point(), prev_prev.point()).is_ccw() {
                return None;
            }

            prev.get_node_id()
        } else {
            // continue with the neighbor node with lowest Y value
            if prev.point().y < next.point().y {
                prev
            } else {
                next
            }
        };

        Self::fill_basin_req(new_node, basin, context, observer)
    }
}

impl Context<'_> {
    pub fn verify_triangles(context: &Context) -> bool {
        Self::illegal_triangles(context).is_empty()
    }

    /// verify all triangles stored in context are legal
    #[allow(unused)]
    pub fn illegal_triangles(context: &Context) -> Vec<(TriangleId, TriangleId)> {
        let triangle_ids = context
            .triangles
            .iter()
            .map(|(t_id, _)| t_id)
            .collect::<Vec<_>>();

        let mut result = Vec::<(TriangleId, TriangleId)>::new();

        for t_id in triangle_ids {
            for illegal_neighbor in &Self::is_legalize(t_id, context) {
                if !illegal_neighbor.invalid() {
                    result.push((t_id, *illegal_neighbor));
                }
            }
        }

        result
    }
}
