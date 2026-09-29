//! VOR: the radial a beacon's two 30 Hz tones put the receiver on, and the
//! station's Morse identifier.

use common::Pulse;
use common::Unit;
use common::packet::{Entity, Fact, Id, Proto, Quantity, Reading};

pub const TAG: [u8; 2] = *b"VR";

pub const IDENT_LEN: usize = 4;

pub fn encode(radial_deg: f32, reference_hz: f32, ident: Option<&str>) -> Vec<u8> {
    let mut v = TAG.to_vec();
    v.extend(((radial_deg.rem_euclid(360.0) * 100.0).round() as u16 % 36_000).to_be_bytes());
    v.extend((reference_hz.round().clamp(0.0, 65_535.0) as u16).to_be_bytes());
    let mut id = [0u8; IDENT_LEN];
    for (d, c) in id.iter_mut().zip(ident.unwrap_or("").bytes()) {
        *d = c;
    }
    v.extend(id);
    v
}

pub fn read(bytes: &[u8]) -> Option<Proto> {
    if bytes.len() != TAG.len() + 4 + IDENT_LEN || bytes[..2] != TAG {
        return None;
    }
    let radial = f64::from(u16::from_be_bytes([bytes[2], bytes[3]])) / 100.0;
    if radial >= 360.0 {
        return None;
    }
    let ident: String = bytes[6..].iter().take_while(|c| **c != 0).map(|c| *c as char).collect();
    let mut p = Proto::new("vor", "radial").saying(Fact::Sensed(Reading {
        quantity: Quantity::Radial,
        value: radial,
        unit: Unit::Degree,
    }));
    if !ident.is_empty() {
        p = p.by(Entity::new("vor", Id::Call(ident)));
    }
    Some(p)
}

pub fn radial(bytes: &[u8]) -> Option<f32> {
    (bytes.len() > 4 && bytes[..2] == TAG)
        .then(|| f32::from(u16::from_be_bytes([bytes[2], bytes[3]])) / 100.0)
}

pub struct Ident {
    block_us: u32,
    recent: Vec<f32>,
    history: Vec<f32>,
    on: bool,
    pending: u32,
    run: u32,
    pulses: Vec<Pulse>,
}

pub const IDENT_REST_US: u32 = 1_500_000;

const SMOOTH_BLOCKS: usize = 4;

const HISTORY_BLOCKS: usize = 1_500;

const DEBOUNCE_BLOCKS: u32 = 5;

impl Ident {
    pub fn new(block_s: f64) -> Self {
        Self {
            block_us: (block_s * 1e6) as u32,
            recent: Vec::new(),
            history: Vec::new(),
            on: false,
            pending: 0,
            run: 0,
            pulses: Vec::new(),
        }
    }

    pub fn push(&mut self, level: f32) -> Option<String> {
        self.recent.push(level);
        if self.recent.len() > SMOOTH_BLOCKS {
            self.recent.remove(0);
        }
        let level = self.recent.iter().sum::<f32>() / self.recent.len() as f32;
        self.history.push(level);
        if self.history.len() > HISTORY_BLOCKS {
            self.history.remove(0);
        }
        let mut sorted = self.history.clone();
        sorted.sort_by(f32::total_cmp);
        let floor = sorted[sorted.len() / 4];
        let peak = sorted[sorted.len() * 98 / 100];
        let keyed = sorted.len() >= 50 && peak > 2.5 * floor;
        let raw = keyed && level > (peak + floor) / 2.0;
        self.pending = if raw != self.on { self.pending + 1 } else { 0 };
        self.run += self.block_us;
        let mut read = None;
        if self.pending >= DEBOUNCE_BLOCKS {
            let back = self.pending * self.block_us;
            self.pending = 0;
            let held = self.run.saturating_sub(back);
            match raw {
                true => {
                    if let Some(last) = self.pulses.last_mut() {
                        last.gap = held;
                    }
                }
                false => self.pulses.push(Pulse { mark: held, gap: 0 }),
            }
            self.on = raw;
            self.run = back;
        }
        if !self.on && self.run >= IDENT_REST_US && !self.pulses.is_empty() {
            if let Some(last) = self.pulses.last_mut() {
                last.gap = self.run;
            }
            let text = crate::morse::decode(&self.pulses);
            self.pulses.clear();
            let text = text.trim();
            if (2..=IDENT_LEN).contains(&text.len()) && text.chars().all(|c| c.is_ascii_uppercase())
            {
                read = Some(text.to_string());
            }
        }
        read
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_radial_and_an_ident_round_trip() {
        let p = read(&encode(253.47, 480.0, Some("BIG"))).expect("a row");
        assert_eq!((p.id, p.kind), ("vor", "radial"));
        assert_eq!(p.subject.map(|e| e.id.to_string()).as_deref(), Some("BIG"));
        assert_eq!(radial(&encode(253.47, 480.0, None)), Some(253.47));
        assert!(read(b"VR").is_none());
    }

    #[test]
    fn a_keyed_ident_is_read() {
        let mut id = Ident::new(0.01);
        let mut got = None;
        let pulses = crate::morse::encode("OCK", 7.0);
        for _ in 0..200 {
            got = got.or(id.push(0.001));
        }
        for p in &pulses {
            for _ in 0..p.mark / 10_000 {
                got = got.or(id.push(0.1));
            }
            for _ in 0..(p.gap / 10_000).min(300) {
                got = got.or(id.push(0.001));
            }
        }
        for _ in 0..300 {
            got = got.or(id.push(0.001));
        }
        assert_eq!(got.as_deref(), Some("OCK"));
    }
}
