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
}

#[derive(Default, Builder, Debug, Clone)]
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

pub struct Gesture {
    traces: HashMap<u32, Trace>,
    anchors: HashMap<u32, Point>,
    centroid: Point,
    resultant: ResultantVector,
    temporal_events: TemporalEvents,
    directional_events: DirectionalEvents,
}
