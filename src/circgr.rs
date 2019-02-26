use std::collections::HashMap;
use std::f64::consts::PI;
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
}

#[derive(Default, Builder, Debug, Clone, PartialEq)]
#[builder(setter(into))]
pub struct Point {
    x: f64,
    y: f64,
    timestamp: u64,
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
    start_time: u64,
    end_time: u64,
    duration: u64,
    observations: HashMap<Direction, Vec<f64>>,
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
    let mut dir_cos_map = HashMap::new();
    let mut dir_sin_map = HashMap::new();
    let mut directional_events = HashMap::new();
    let mut temporal_events = HashMap::new();
    for (id, points) in raw_traces {
        gesture.traces.insert(
            *id,
            process_trace(
                &points[..],
                &gesture.centroid,
                &trace_info[id],
                sample_resolution,
                &mut dir_cos_map,
                &mut dir_sin_map,
                &mut directional_events,
                &mut temporal_events,
            ),
        );
    }

    let mut resultants = HashMap::new();
    for direction in dir_cos_map.keys() {
        resultants.insert(
            direction.clone(),
            calculate_resultant(
                dir_sin_map[direction],
                dir_cos_map[direction],
                directional_events[direction].len(),
            ),
        );
    }

    let mut gc = 0.0_f64;
    let mut gs = 0.0_f64;
    for trace in gesture.traces.values() {
        gc += trace.resultant.angle.cos();
        gs += trace.resultant.angle.sin();
    }

    gesture.resultant = calculate_resultant(gs, gc, gesture.anchors.len() + gesture.traces.len());
    gesture.directional_events = DirectionalEvents {
        observations: directional_events,
        resultants: resultants,
    };

    gesture
}

fn distance(a: &Point, b: &Point) -> f64 {
    ((a.x - b.x).powf(2.0) + (a.y - b.y).powf(2.0)).sqrt()
}

#[derive(Default, Debug, Clone)]
struct TraceInfo {
    path_length: f64,
    centroid: Point,
    start_time: u64,
    end_time: u64,
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
    dir_cos_map: &mut HashMap<Direction, f64>,
    dir_sin_map: &mut HashMap<Direction, f64>,
    directional_events: &mut HashMap<Direction, Vec<f64>>,
    temporal_events: &mut HashMap<Direction, Vec<f64>>,
) -> Trace {
    let interval = info.path_length / sample_resolution as f64;
    let mut d = 0.0_f64;
    let mut trace_cos = 0.0_f64;
    let mut trace_sin = 0.0_f64;
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
                    .timestamp(points[i].timestamp)
                    .build()
                    .unwrap();

                let observation = compute_observation(&previous_point, &point);
                trace.observations.push(observation.clone());
                let (sin, cos) = observation.sin_cos();
                trace_cos += cos;
                trace_sin += sin;
                let direction = get_direction(observation.clone());
                let cdirection = get_centripetal_direction(&previous_point, &point, &info.centroid);
                *dir_cos_map.entry(direction.clone()).or_insert(0.0_f64) += cos;
                *dir_sin_map.entry(direction.clone()).or_insert(0.0_f64) += sin;
                *dir_cos_map.entry(cdirection.clone()).or_insert(0.0_f64) += cos;
                *dir_sin_map.entry(cdirection.clone()).or_insert(0.0_f64) += sin;
                directional_events
                    .entry(direction.clone())
                    .or_insert(Vec::new())
                    .push(observation.clone());
                directional_events
                    .entry(cdirection.clone())
                    .or_insert(Vec::new())
                    .push(observation.clone());

                let temporal_observation =
                    compute_temporal_observation(point.timestamp, info.start_time, info.end_time);
                temporal_events
                    .entry(direction)
                    .or_insert(Vec::new())
                    .push(temporal_observation.clone());
                temporal_events
                    .entry(cdirection)
                    .or_insert(Vec::new())
                    .push(temporal_observation.clone());

                if previous_observation > 0.0_f64 {
                    let clock_direction =
                        get_rotational_direction(previous_observation, observation);
                    *dir_cos_map
                        .entry(clock_direction.clone())
                        .or_insert(0.0_f64) += cos;
                    *dir_sin_map
                        .entry(clock_direction.clone())
                        .or_insert(0.0_f64) += sin;
                    directional_events
                        .entry(clock_direction.clone())
                        .or_insert(Vec::new())
                        .push(observation);
                    temporal_events
                        .entry(clock_direction)
                        .or_insert(Vec::new())
                        .push(temporal_observation);
                }

                previous_observation = observation;

                // Update partial length
                distance = d + distance - interval;
                d = 0.0_f64;
                previous_point = point;
            }
        } else {
            d += distance;
        }
    }

    trace.resultant = calculate_resultant(trace_sin, trace_cos, trace.observations.len());

    return trace;
}

fn compute_observation(previous_point: &Point, point: &Point) -> f64 {
    let mut angle = (point.y - previous_point.y).atan2(point.x - previous_point.x);
    if (angle < 0.0_f64) {
        angle += (2.0_f64 * PI);
    }

    angle
}

fn get_direction(mut observation: f64) -> Direction {
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
    } else if (observation >= THREE_PI_OVER_4 + MARGIN && observation > (2.0_f64 * PI) - MARGIN) {
        return Direction::DownRight;
    } else if (observation >= (2.0_f64 * PI) - MARGIN && observation < 2.0_f64 * PI) {
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

fn compute_temporal_observation(time: u64, start_time: u64, end_time: u64) -> f64 {
    if time > end_time || time < start_time {
        panic!(
            "Time not in range: {} < {} < {}",
            start_time, time, end_time
        );
    }
    return ((time - start_time) / (end_time - start_time)) as f64 * (2.0_f64 * PI);
}

fn get_rotational_direction(previous: f64, current: f64) -> Direction {
    let max_ccw = previous + PI;

    if max_ccw < (2.0_f64 * PI) - 1.0_f64.to_radians() {
        if previous <= current && current <= max_ccw {
            return Direction::CounterClockwise;
        } else {
            return Direction::Clockwise;
        }
    } else {
        let split = (2.0_f64 * PI) - previous;
        let adjusted = max_ccw - (2.0_f64 * PI);
        if (previous <= current && current <= 2.0_f64 * PI)
            || (0.0_f64 <= current && current <= adjusted)
        {
            return Direction::CounterClockwise;
        } else {
            return Direction::Clockwise;
        }
    }
}

fn calculate_resultant(sin: f64, cos: f64, len: usize) -> ResultantVector {
    let point = PointBuilder::default().x(cos).y(sin).build().unwrap();
    let angle = compute_observation(&Point::default(), &point);
    let magnitude = distance(&Point::default(), &point);
    let dispersion = len as f64 - magnitude;

    ResultantVector {
        angle: angle,
        magnitude: magnitude,
        dispersion: dispersion,
    }
}
