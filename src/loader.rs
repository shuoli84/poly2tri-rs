use crate::{Float, Point, SweeperBuilder};

#[derive(thiserror::Error, Debug)]
pub enum LoaderError {
    #[error("IO error")]
    Io(#[from] std::io::Error),
    #[error("Inner error")]
    Inner(#[from] Box<dyn std::error::Error>),
}

/// Loader loads source to a [`Sweeper`].
/// e.g: PlainFileLoader load from file path with same format defined by original 'poly2tri' project
pub trait Loader {
    fn load(&mut self, source: &str) -> Result<SweeperBuilder, LoaderError>;
}

/// Loaders can load data from file
#[derive(Default)]
pub struct PlainFileLoader {}

#[derive(Default)]
enum ParseState {
    #[default]
    Polygon,
    Hole,
    Steiner,
}

impl Loader for PlainFileLoader {
    fn load(&mut self, path: &str) -> Result<SweeperBuilder, LoaderError> {
        let mut f = std::fs::File::options().read(true).open(path)?;
        let mut value = "".to_string();
        std::io::Read::read_to_string(&mut f, &mut value).unwrap();

        let mut state = ParseState::default();
        let mut polygon = vec![];
        let mut holes = Vec::<Vec<Point>>::new();
        let mut steiner_points = Vec::<Point>::new();

        for line in value.lines() {
            if line.eq("HOLE") {
                state = ParseState::Hole;
                holes.push(vec![]);
                continue;
            } else if line.eq("STEINER") {
                state = ParseState::Steiner;
                continue;
            }
            let Some(point) = parse_point(line)? else {
                continue
            };

            match state {
                ParseState::Polygon => {
                    polygon.push(point);
                }
                ParseState::Hole => {
                    let current_hole = holes.last_mut().unwrap();
                    current_hole.push(point);
                }
                ParseState::Steiner => {
                    steiner_points.push(point);
                }
            }
        }

        Ok(SweeperBuilder::new(polygon)
            .add_holes(holes)
            .add_steiner_points(steiner_points))
    }
}

fn parse_point(line: &str) -> Result<Option<Point>, LoaderError> {
    if line.is_empty() {
        return Ok(None);
    }
    let mut iter = line.split_whitespace();
    let x = iter.next().unwrap();
    let y = iter.next().unwrap();
    let x = x.parse::<Float>().unwrap();
    let y = y.parse::<Float>().unwrap();

    Ok(Some(Point::new(x, y)))
}
