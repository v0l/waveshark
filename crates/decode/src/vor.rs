//! VOR: the radial a beacon's two 30 Hz tones put the receiver on, and the
//! station's Morse identifier.

use common::Pulse;
use common::Unit;
use common::packet::{Entity, Fact, Id, Proto, Quantity, Reading};

pub const TAG: [u8; 2] = *b"VR";

pub const IDENT_LEN: usize = 4;

const NO_RADIAL: u16 = u16::MAX;

pub fn encode(radial_deg: Option<f32>, reference_hz: f32, ident: Option<&str>) -> Vec<u8> {
    let mut v = TAG.to_vec();
    let radial =
        radial_deg.map_or(NO_RADIAL, |r| (r.rem_euclid(360.0) * 100.0).round() as u16 % 36_000);
    v.extend(radial.to_be_bytes());
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
    let raw = u16::from_be_bytes([bytes[2], bytes[3]]);
    let radial = (raw < 36_000).then(|| f64::from(raw) / 100.0);
    let ident: String = bytes[6..].iter().take_while(|c| **c != 0).map(|c| *c as char).collect();
    if radial.is_none() && ident.is_empty() {
        return None;
    }
    let mut p = Proto::new("vor", if radial.is_some() { "radial" } else { "ident" });
    if let Some(r) = radial {
        p = p.saying(Fact::Sensed(Reading {
            quantity: Quantity::Radial,
            value: r,
            unit: Unit::Degree,
        }));
    }
    if !ident.is_empty() {
        p = p.by(Entity::new("vor", Id::Call(ident)));
    }
    Some(p)
}

pub fn radial(bytes: &[u8]) -> Option<f32> {
    let raw = u16::from_be_bytes(bytes.get(2..4)?.try_into().ok()?);
    (bytes[..2] == TAG && raw < 36_000).then(|| f32::from(raw) / 100.0)
}

pub struct Ident {
    block_us: u32,
    recent: Vec<f32>,
    history: Vec<f32>,
    on: bool,
    pending: u32,
    run: u32,
    pulses: Vec<Pulse>,
    partial: bool,
}

pub const IDENT_REST_US: u32 = 1_000_000;

const SMOOTH_BLOCKS: usize = 6;

const IDENT_THRESHOLD: f32 = 0.35;

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
            partial: false,
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
        let raw = keyed && level > floor + IDENT_THRESHOLD * (peak - floor);
        self.pending = if raw != self.on { self.pending + 1 } else { 0 };
        self.run += self.block_us;
        let mut read = None;
        if self.pending >= DEBOUNCE_BLOCKS {
            let back = self.pending * self.block_us;
            self.pending = 0;
            let held = self.run.saturating_sub(back);
            match raw {
                true => match self.pulses.last_mut() {
                    Some(last) => last.gap = held,
                    None => self.partial = held < IDENT_REST_US,
                },
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
            let whole = !std::mem::take(&mut self.partial);
            if whole
                && (2..=IDENT_LEN).contains(&text.len())
                && text.chars().all(|c| c.is_ascii_uppercase())
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
        let p = read(&encode(Some(253.47), 480.0, Some("BIG"))).expect("a row");
        assert_eq!((p.id, p.kind), ("vor", "radial"));
        assert_eq!(p.subject.map(|e| e.id.to_string()).as_deref(), Some("BIG"));
        assert_eq!(radial(&encode(Some(253.47), 480.0, None)), Some(253.47));
        let p = read(&encode(None, 0.0, Some("BIG"))).expect("an identifier on its own");
        assert_eq!(p.kind, "ident");
        assert!(p.facts.is_empty());
        assert!(read(&encode(None, 0.0, None)).is_none());
        assert!(read(b"VR").is_none());
    }

    #[test]
    fn an_ident_heard_from_the_middle_is_not_read() {
        let mut id = Ident::new(0.01);
        let mut got = None;
        let pulses = crate::morse::encode("BIG", 7.0);
        for _ in 0..60 {
            got = got.or(id.push(0.001));
        }
        for p in pulses.iter().skip(2) {
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
        assert_eq!(got, None);
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
