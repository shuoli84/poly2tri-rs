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

#[derive(Debug, Clone, Copy)]
pub struct Triangle {
    pub points: [Point; 3],
}
