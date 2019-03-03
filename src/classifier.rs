use crate::gesture::{Direction, Gesture};
use itertools::izip;
use std::collections::HashSet;
use std::f64::consts::PI;
use std::iter::FromIterator;
use std::iter::Iterator;
use std::vec::Vec;

pub struct Classifier {
    templates: Vec<Gesture>,
}

impl Classifier {
    pub fn new() -> Classifier {
        Classifier {
            templates: Vec::new(),
        }
    }

    pub fn classify(&self, gesture: &Gesture) -> Option<(String, f64)> {
        classify_impl(&gesture, &self.templates[..])
    }

    pub fn classify_dumb(&self, gesture: &Gesture) -> Option<(String, f64)> {
        classify_dumb_impl(&gesture, &self.templates[..])
    }

    pub fn add_template(&mut self, template: Gesture) {
        self.templates.push(template);
    }
}

fn classify_impl(gesture: &Gesture, templates: &[Gesture]) -> Option<(String, f64)> {
    let mut min = std::f64::MAX;
    let mut name = "".to_string();
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

        if min > distance {
            name = template.name.clone();
            min = distance;
        }
    }

    if name.is_empty() {
        return None;
    }

    return Some((name, min));
}

fn direction_distance(gesture: &Gesture, template: &Gesture, direction: Direction) -> f64 {
    let mut distance = 0.0_f64;
    let empty: Vec<f64> = Vec::new();
    let cresultant = match gesture.directional_events.resultants.get(&direction) {
        Some(a) => a.angle,
        None => 0.0_f64,
    };
    let tresultant = match template.directional_events.resultants.get(&direction) {
        Some(a) => a.angle,
        None => 0.0_f64,
    };
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

    let sum: f64 = izip!(
        &cdirectional[..min_len],
        &tdirectional[..min_len],
        &ctemporal[..min_len],
        &ttemporal[..min_len]
    )
    .map(|x| -> f64 { observation_distance(*x.0, *x.1) + observation_distance(*x.2, *x.3) })
    .sum();

    distance += sum;

    distance += match cdirectional.len() {
        _ if cdirectional.len() == min_len => {
            &tdirectional[min_len..]
                .iter()
                .map(|x| -> f64 { observation_distance(*x, cresultant) })
                .sum()
                + (tdirectional.len() - min_len) as f64 * PI
        }
        _ => {
            &cdirectional[min_len..]
                .iter()
                .map(|x| -> f64 { observation_distance(*x, tresultant) })
                .sum()
                + (cdirectional.len() - min_len) as f64 * PI
        }
    };

    return distance;
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
    PI - (PI - (a - b).abs()).abs()
}

fn classify_dumb_impl(gesture: &Gesture, templates: &[Gesture]) -> Option<(String, f64)> {
    let mut min = std::f64::MAX;
    let mut name = "".to_string();
    let co = sorted_observations(&gesture);
    for template in templates {
        if template.anchors.len() != gesture.anchors.len()
            || template.traces.len() != gesture.traces.len()
        {
            continue;
        }

        let mut distance = 0.0_f64;
        let to = sorted_observations(template);
        let min_len = co.len().min(to.len());
        let sum: f64 = izip!(&co[..min_len], &to[..min_len])
            .map(|x| -> f64 { observation_distance(*x.0, *x.1) })
            .sum();
        distance += sum;
        distance += match co.len() {
            _ if co.len() == min_len => &to[min_len..].iter().sum() + 0.0_f64,
            _ => &co[min_len..].iter().sum() + 0.0_f64,
        };
        if min > distance {
            name = template.name.clone();
            min = distance;
        }
    }

    if name.is_empty() {
        return None;
    }

    return Some((name, min));
}

fn sorted_observations(gesture: &Gesture) -> Vec<f64> {
    let mut v: Vec<f64> = gesture
        .traces
        .values()
        .flat_map(|s| s.observations.iter())
        .cloned()
        .collect();
    v.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
    v
}
