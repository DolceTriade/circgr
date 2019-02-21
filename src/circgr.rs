use std::collections::HashMap;
use std::vec::Vec;

#[derive(Debug, PartialEq, Eq, Hash, Clone)]
pub enum Direction {
    Up,
    Down,
    Left,
    Right,
    UpLeft,
    UpRight,
    DownLeft,
    DownRight,
    In,
    Out,
    Clockwise,
    CounterClockwise,
    MAX_DIRECTIONS,
}

#[derive(Default, Builder, Debug, Clone, PartialEq)]
#[builder(setter(into))]
pub struct Point {
    x: f64,
    y: f64,
    timestamp: u32,
}

#[derive(Default, Builder, Debug, Clone)]
#[builder(setter(into))]
pub struct ResultantVector {
    angle: f64,
    magnitude: f64,
    dispersion: f64,
}

#[derive(Default, Builder, Debug, Clone)]
#[builder(setter(into))]
pub struct DirectionalEvents {
    observations: HashMap<Direction, Vec<f64>>,
    resultants: HashMap<Direction, ResultantVector>,
}

#[derive(Default, Builder, Debug, Clone)]
#[builder(setter(into))]
pub struct TemporalEvents {
    start_time: u32,
    end_time: u32,
    duration: u32,
    resultants: HashMap<Direction, ResultantVector>,
}

#[derive(Default, Builder, Debug, Clone)]
#[builder(setter(into))]
pub struct Trace {
    observations: Vec<f64>,
    resultant: ResultantVector,
}

#[derive(Default, Debug, Clone)]
pub struct Gesture {
    traces: HashMap<u32, Trace>,
    anchors: HashMap<u32, Point>,
    centroid: Point,
    resultant: ResultantVector,
    temporal_events: TemporalEvents,
    directional_events: DirectionalEvents,
}

impl Gesture {
    fn new(raw_traces: &HashMap<u32, Vec<Point>>, sample_resolution: u32) -> Self {
        build_gesture(&raw_traces, sample_resolution)
    }
}

fn build_gesture(raw_traces: &HashMap<u32, Vec<Point>>, sample_resolution: u32) -> Gesture {
    let mut gesture = Gesture::default();


    for (id, points) in raw_traces {
        let info = preprocess_trace(&points);
        let interval = info.path_length / sample_resolution as f64;
        if interval < 0.5_f64 {
            gesture.anchors.insert(*id, info.centroid.clone());
            continue;
        }
    }

    if gesture.anchors.is_empty() {
    }

    gesture
}

fn distance(a: &Point, b: &Point) -> f64 {
    ((a.x - b.x).powf(2.0) + (a.y - b.y).powf(2.0)).sqrt()
}

#[derive(Default, Debug, Clone)]
struct TraceInfo {
    path_length: f64,
    centroid: Point,
    start_time: u32,
    end_time: u32,
}

fn preprocess_trace(points: &[Point]) -> TraceInfo {
    let mut info = TraceInfo::default();
    info.start_time = points[0].timestamp;
    info.end_time = points[0].timestamp;
    info.centroid = points[0].clone();

    for i in 1..points.len() {
        let prev = &points[i - 1];
        let p = &points[i];

        info.path_length += distance(p, prev);
        info.centroid.x += p.x;
        info.centroid.y += p.y;

        if p.timestamp < info.start_time {
            info.start_time = p.timestamp;
        }

        if p.timestamp > info.end_time {
            info.end_time = p.timestamp;
        }
    }

    info.centroid.x = info.centroid.x / points.len() as f64;
    info.centroid.y = info.centroid.x / points.len() as f64;

    info
}

fn compute_centroid(points: &[Point]) -> Point {
    let mut centroid = Point::default();
    for p in points {
        centroid.x += p.x;
        centroid.y += p.y;
    }
    centroid.x = centroid.x / points.len() as f64;
    centroid.y = centroid.y / points.len() as f64;

    centroid
}