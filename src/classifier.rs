use crate::gesture::{Direction, Gesture, Trace};
use std::collections::HashSet;
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

    pub fn classify(&self, gesture: &Gesture) -> Option<(String, usize)> {
        classify_impl(&gesture, &self.templates[..])
    }

    pub fn add_template(&mut self, template: Gesture) {
        self.templates.push(template);
    }
}

fn classify_impl(gesture: &Gesture, templates: &[Gesture]) -> Option<(String, usize)> {
    let mut min = std::usize::MAX;
    let mut name = "".to_string();
    for template in templates {
        if template.anchors.len() != gesture.anchors.len()
            || template.traces.len() != gesture.traces.len()
        {
            continue;
        }

        let mut distance = 0_usize;
        for trace_pair in pair_traces(&gesture, &template) {
            distance += trace_pair.2;
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

fn all_directions(a: &Trace, b: &Trace) -> HashSet<Direction> {
    HashSet::from_iter(
        a.directional_events
            .observations
            .keys()
            .cloned()
            .chain(b.directional_events.observations.keys().cloned()),
    )
}

fn pair_traces<'a>(gesture: &'a Gesture, template: &'a Gesture) -> Vec<(&'a Trace, &'a Trace, usize)> {
    let mut ret = Vec::new();
    let mut paired = HashSet::new();
    for trace in gesture.traces.values() {
        let mut min = std::usize::MAX;
        let mut best = trace;
        let mut id = 0;
        for ttrace in &template.traces {
            if paired.contains(ttrace.0) {
                continue;
            }
            let score = trace_similiarity(&trace, &ttrace.1);
            println!("Trace similarity: {}", &score);
            if score < min {
                min = score;
                best = &ttrace.1;
                id = *ttrace.0;
            }
        }
        paired.insert(id);
        println!("Pairing {:#?} {:#?} => {}", print_directions(&trace), print_directions(&best), &min);
        ret.push((trace, best, min));
    }
    ret
}

fn trace_similiarity(a: &Trace, b: &Trace) -> usize {
    let mut distance = 0;
    let empty = Vec::new();
    for direction in all_directions(a, b) {
        let a_len = a
            .directional_events
            .observations
            .get(&direction)
            .unwrap_or(&empty)
            .len();
        let b_len = b
            .directional_events
            .observations
            .get(&direction)
            .unwrap_or(&empty)
            .len();
        distance += a_len.max(b_len) - a_len.min(b_len);
    }
    distance
}

fn print_directions(trace: &Trace) -> String {
    let mut s = String::new();
    for kv in &trace.directional_events.observations {
        s.push_str(&format!("{:?}={} ", kv.0, kv.1.len()));
    }
    s
}
