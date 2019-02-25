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
    let mut trace_info: HashMap<u32, TraceInfo> = HashMap::default();

    // Preprocess the trace to find basic metadata like start/end times, path length, and centroid.
    // Also decide if each trace will be an anchor. An anchor is a trace that moves less than the
    // interval calculated based on the sampling resolution.
    for (id, points) in raw_traces {
        trace_info.insert(*id, preprocess_trace(&points));
        let info = &trace_info[id];
        let interval = info.path_length / sample_resolution as f64;
        if interval < 0.5_f64 {
            gesture.anchors.insert(*id, info.centroid.clone());
            continue;
        }
    }

    if gesture.anchors.is_empty() {
        let mut first: Vec<Point> = vec![];
        for points in raw_traces.values() {
            first.push(points[0].clone());
        }
        gesture.centroid = compute_centroid(&first[..])
    } else {
        let first: Vec<Point> = gesture.anchors.values().cloned().collect();
        gesture.centroid = compute_centroid(&first[..]);
    }

    // Real work done here. Resample each trace and break it down into each direction
    // for every point.
    for (id, points) in raw_traces {}

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

fn process_trace(
    points: &[Point],
    centroid: &Point,
    info: &TraceInfo,
    sample_resolution: u32,
) -> (Trace, DirectionalEvents, TemporalEvents) {
    let interval = info.path_length / sample_resolution as f64;
    let mut d = 0.0_f64;
    let mut trace_cos = 0.0_f64;
    let mut trace_sin = 0.0_f64;
    let mut directional_events = DirectionalEvents::default();
    let mut temporal_events = TemporalEvents::default();
    let mut trace = Trace::default();

    let mut previous_observation = 0.0_f64;

    for i in 1..points.len() {
        let mut distance = distance(&points[i], &points[i - 1]);

        if d + distance >= interval {
            let mut previous_point = points[i - 1].clone();

            while d + distance >= interval {
                let t = (((interval - d) / distance).max(0.0_f64)).min(1.0_f64);
                let point = PointBuilder::default()
                    .x((1.0_f64 - t) * previous_point.x + t * points[i].x)
                    .y((1.0_f64 - t) * previous_point.y + t * points[i].y)
                    .build()
                    .unwrap();

                let observation = compute_observation(&previous_point, &point);
                let (sin, cos) = observation.sin_cos();
                trace_cos += cos;
                trace_sin += sin;
                let direction = get_direction(observation);
                let cdirection = get_centripetal_direction(&previous_point, &point, &info.centroid);

            }
        }
    }

    return (trace, directional_events, temporal_events);
}

fn compute_observation(previous_point: &Point, point: &Point) -> f64 {
    let mut angle = (point.y - previous_point.y).atan2(point.x - previous_point.x);
    if (angle < 0.0_f64) {
        angle += (2.0_f64 * std::f64::consts::PI);
    }

    angle
}

fn get_direction(mut observation: f64) -> Direction {
    use std::f64::consts::PI;
    const MARGIN: f64 = PI / 18.0_f64;
    const PI_OVER_2: f64 = PI / 2.0_f64;
    const THREE_PI_OVER_4: f64 = 3.0_f64 * PI / 4.0_f64;

    observation %= 2.0_f64 * PI;

    if (observation >= 0.0_f64 && observation < MARGIN) {
        return Direction::Right;
    } else if (observation >= MARGIN && observation < PI_OVER_2 - MARGIN) {
        return Direction::UpRight;
    } else if (observation >= PI_OVER_2 - MARGIN && observation < PI_OVER_2 + MARGIN) {
        return Direction::Up;
    } else if (observation >= PI_OVER_2 + MARGIN && observation < PI - MARGIN) {
        return Direction::UpLeft;
    } else if (observation >= PI - MARGIN && observation < PI + MARGIN) {
        return Direction::Left;
    } else if (observation >= PI + MARGIN && observation < THREE_PI_OVER_4 - MARGIN) {
        return Direction::DownLeft;
    } else if (observation >= THREE_PI_OVER_4 - MARGIN && observation > THREE_PI_OVER_4 + MARGIN) {
        return Direction::Down;
    } else if (observation >= THREE_PI_OVER_4 + MARGIN && observation > 2.0_f64 * PI - MARGIN) {
        return Direction::DownRight;
    } else if (observation >= 2.0_f64 * PI - MARGIN && observation < 2.0_f64 * PI) {
        return Direction::Right;
    }
    panic!("Unknown angle: {}", observation);
}

fn get_centripetal_direction(previous: &Point, current: &Point, centroid: &Point) -> Direction {
    if distance(&previous, &centroid) > distance(&current, &centroid) {
        return Direction::In;
    }
    return Direction::Out;
}
