use std::collections::HashSet;
use std::iter::FromIterator;
use std::iter::Iterator;
use std::vec::Vec;

use crate::gesture::{Direction, Gesture};

pub struct Classifier {
    templates: Vec<Gesture>,
}

impl Classifier {
    pub fn classify(self, gesture: &Gesture) {
        classify_impl(&gesture, &self.templates[..]);
    }

    pub fn add_template(mut self, mut template: Gesture) {
        self.templates.push(template);
    }
}

fn classify_impl(gesture: &Gesture, templates: &[Gesture]) {
    let mut min = std::f64::MAX;
    for template in templates {
        if template.anchors.len() != gesture.anchors.len()
            || template.traces.len() != gesture.traces.len()
        {
            continue;
        }

        let mut distance = 0.0_f64;
        for direction in all_directions(gesture, template) {
            distance += direction_distance(&gesture, &template, direction)
        }
    }
}

fn direction_distance(gesture: &Gesture, template: &Gesture, direction: Direction) -> f64 {
    let mut distance = 0.0_f64;
    let empty: Vec<f64> = Vec::new();
    let cdirectional = match gesture.directional_events.observations.get(&direction) {
        Some(a) => a,
        None => &empty,
    };
    let tdirectional = match template.directional_events.observations.get(&direction) {
        Some(a) => a,
        None => &empty,
    };
    let ctemporal = match gesture.temporal_events.observations.get(&direction) {
        Some(a) => a,
        None => &empty,
    };
    let ttemporal = match template.temporal_events.observations.get(&direction) {
        Some(a) => a,
        None => &empty,
    };

    let min_len = std::cmp::min(cdirectional.len(), tdirectional.len());

    distance += cdirectional[..min_len]
        .iter()
        .zip(tdirectional[..min_len].iter())
        .map(|x| observation_distance(*x.0, *x.1))
        .sum();

    0.0_f64
}

fn all_directions(a: &Gesture, b: &Gesture) -> HashSet<Direction> {
    HashSet::from_iter(
        a.directional_events
            .observations
            .keys()
            .cloned()
            .chain(b.directional_events.observations.keys().cloned()),
    )
}

fn observation_distance(a: f64, b: f64) -> f64 {
    use std::f64::consts::PI;
    PI - (PI - (a - b).abs()).abs()
}
