//! Long wave time signals: MSF on 60 kHz and DCF77 on 77.5 kHz, a minute's
//! date and time keyed one bit a second into the carrier.

use common::Unit;
use common::packet::{Entity, Fact, Proto, Quantity, Reading};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Station {
    Msf,
    Dcf77,
    Tdf,
}

impl Station {
    pub fn name(self) -> &'static str {
        match self {
            Self::Msf => "MSF",
            Self::Dcf77 => "DCF77",
            Self::Tdf => "TDF",
        }
    }

    pub fn id(self) -> &'static str {
        match self {
            Self::Msf => "msf",
            Self::Dcf77 => "dcf77",
            Self::Tdf => "tdf",
        }
    }

    fn code(self) -> u8 {
        match self {
            Self::Msf => 1,
            Self::Dcf77 => 2,
            Self::Tdf => 3,
        }
    }

    fn of(code: u8) -> Option<Self> {
        match code {
            1 => Some(Self::Msf),
            2 => Some(Self::Dcf77),
            3 => Some(Self::Tdf),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Minute {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub utc_offset_min: i16,
}

impl Minute {
    pub fn unix(&self) -> i64 {
        let days = days_from_civil(i64::from(self.year), self.month, self.day);
        days * 86_400 + i64::from(self.hour) * 3_600 + i64::from(self.minute) * 60
            - i64::from(self.utc_offset_min) * 60
    }
}

fn days_from_civil(y: i64, m: u8, d: u8) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let m = i64::from(m);
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + i64::from(d) - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

pub const TAG: [u8; 2] = *b"TS";

pub fn encode(station: Station, m: &Minute) -> Vec<u8> {
    let mut v = TAG.to_vec();
    v.push(station.code());
    v.extend(m.unix().to_be_bytes());
    v
}

pub fn read(bytes: &[u8]) -> Option<Proto> {
    if bytes.len() != 11 || bytes[..2] != TAG {
        return None;
    }
    let station = Station::of(bytes[2])?;
    let unix = i64::from_be_bytes(bytes[3..11].try_into().ok()?);
    Some(Proto::new(station.id(), "time").by(Entity::call("time", station.name())).saying(
        Fact::Sensed(Reading { quantity: Quantity::Clock, value: unix as f64, unit: Unit::Second }),
    ))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Second {
    pub low: [bool; 5],
    pub gap_before_ms: u32,
}

impl Second {
    pub fn keyed(off_ms: u32, second_off: bool, gap_before_ms: u32) -> Self {
        let low = std::array::from_fn(|k| {
            let at = k as u32 * 100 + 50;
            at < off_ms || (second_off && (200..300).contains(&at))
        });
        Self { low, gap_before_ms }
    }
}

pub struct Slicer {
    block_ms: u32,
    history: Vec<f32>,
    on: bool,
    since_start_ms: u32,
    pending: u32,
    started: bool,
    gap_before_ms: u32,
    levels: Vec<f32>,
    threshold: f32,
}

const HISTORY_BLOCKS: usize = 300;

const DEBOUNCE_BLOCKS: u32 = 3;

const SECOND_START_MS: u32 = 700;

impl Slicer {
    pub fn new(block_s: f64) -> Self {
        Self {
            block_ms: (block_s * 1000.0).round() as u32,
            history: Vec::new(),
            on: true,
            since_start_ms: 0,
            pending: 0,
            started: false,
            gap_before_ms: 0,
            levels: Vec::new(),
            threshold: 0.0,
        }
    }

    fn second(&self) -> Second {
        let per = (100 / self.block_ms.max(1)) as usize;
        let low = std::array::from_fn(|k| {
            let from = k * per + per / 5;
            let to = (k + 1) * per - per / 5;
            let slots = self.levels.get(from..to).unwrap_or(&[]);
            !slots.is_empty() && slots.iter().sum::<f32>() / (slots.len() as f32) < self.threshold
        });
        Second { low, gap_before_ms: self.gap_before_ms }
    }

    pub fn push(&mut self, level: f32) -> Option<Second> {
        self.history.push(level);
        if self.history.len() > HISTORY_BLOCKS {
            self.history.remove(0);
        }
        let mut sorted = self.history.clone();
        sorted.sort_by(f32::total_cmp);
        let lo = sorted[sorted.len() / 20];
        let hi = sorted[sorted.len() / 2];
        let keyed = hi > 1.5 * lo;
        self.threshold = (hi + lo) / 2.0;
        let raw = !keyed || level > self.threshold;
        self.pending = if raw != self.on { self.pending + 1 } else { 0 };
        self.since_start_ms += self.block_ms;
        if self.started && self.levels.len() < 60 {
            self.levels.push(level);
        }
        let mut done = None;
        if self.pending >= DEBOUNCE_BLOCKS {
            let back = self.pending * self.block_ms;
            self.pending = 0;
            self.on = raw;
            let edge_ms = self.since_start_ms - back;
            if !raw && (edge_ms >= SECOND_START_MS || !self.started) {
                if self.started {
                    done = Some(self.second());
                }
                self.started = true;
                self.gap_before_ms = edge_ms;
                self.since_start_ms = back;
                let keep = (back / self.block_ms) as usize;
                let tail = self.history.len().saturating_sub(keep);
                self.levels = self.history[tail..].to_vec();
            }
        }
        done
    }
}

fn bcd(bits: &[bool], weights: &[u32]) -> u32 {
    bits.iter().zip(weights).filter(|(b, _)| **b).map(|(_, w)| w).sum()
}

fn even(bits: &[bool]) -> bool {
    bits.iter().filter(|b| **b).count() % 2 == 0
}

pub struct Msf {
    a: Vec<bool>,
    b: Vec<bool>,
    open: bool,
}

impl Default for Msf {
    fn default() -> Self {
        Self::new()
    }
}

impl Msf {
    pub fn new() -> Self {
        Self { a: Vec::new(), b: Vec::new(), open: false }
    }

    pub fn push(&mut self, s: Second) -> Option<Minute> {
        if s.low[3] && s.low[4] {
            let read = (self.open && self.a.len() >= 59).then(|| self.minute()).flatten();
            self.a = vec![false];
            self.b = vec![false];
            self.open = true;
            return read;
        }
        if !self.open {
            return None;
        }
        self.a.push(s.low[1]);
        self.b.push(s.low[2]);
        None
    }

    fn minute(&self) -> Option<Minute> {
        let (a, b) = (&self.a, &self.b);
        let odd = |bits: &[bool], p: bool| {
            (bits.iter().filter(|x| **x).count() + usize::from(p)) % 2 == 1
        };
        if !(odd(&a[17..25], b[54])
            && odd(&a[25..36], b[55])
            && odd(&a[36..39], b[56])
            && odd(&a[39..52], b[57]))
        {
            return None;
        }
        let m = Minute {
            year: 2000 + bcd(&a[17..25], &[80, 40, 20, 10, 8, 4, 2, 1]) as u16,
            month: bcd(&a[25..30], &[10, 8, 4, 2, 1]) as u8,
            day: bcd(&a[30..36], &[20, 10, 8, 4, 2, 1]) as u8,
            hour: bcd(&a[39..45], &[20, 10, 8, 4, 2, 1]) as u8,
            minute: bcd(&a[45..52], &[40, 20, 10, 8, 4, 2, 1]) as u8,
            utc_offset_min: if b[58] { 60 } else { 0 },
        };
        valid(&m).then_some(m)
    }
}

pub struct Dcf77 {
    bits: Vec<bool>,
    open: bool,
}

impl Default for Dcf77 {
    fn default() -> Self {
        Self::new()
    }
}

impl Dcf77 {
    pub fn new() -> Self {
        Self { bits: Vec::new(), open: false }
    }

    pub fn push(&mut self, s: Second) -> Option<Minute> {
        if s.gap_before_ms >= 1_500 {
            let read = (self.open && self.bits.len() == 59).then(|| self.minute()).flatten();
            self.bits = vec![s.low[1]];
            self.open = true;
            return read;
        }
        if self.open {
            self.bits.push(s.low[1]);
        }
        None
    }

    fn minute(&self) -> Option<Minute> {
        let b = &self.bits;
        if b[0] || !b[20] {
            return None;
        }
        let offset = match (b[17], b[18]) {
            (true, false) => 120,
            (false, true) => 60,
            _ => return None,
        };
        central_european(b, offset)
    }
}

fn central_european(b: &[bool], offset: i16) -> Option<Minute> {
    if !even(&b[21..29]) || !even(&b[29..36]) || !even(&b[36..59]) {
        return None;
    }
    let m = Minute {
        year: 2000 + bcd(&b[50..58], &[1, 2, 4, 8, 10, 20, 40, 80]) as u16,
        month: bcd(&b[45..50], &[1, 2, 4, 8, 10]) as u8,
        day: bcd(&b[36..42], &[1, 2, 4, 8, 10, 20]) as u8,
        hour: bcd(&b[29..35], &[1, 2, 4, 8, 10, 20]) as u8,
        minute: bcd(&b[21..28], &[1, 2, 4, 8, 10, 20, 40]) as u8,
        utc_offset_min: offset,
    };
    valid(&m).then_some(m)
}

pub struct Tdf {
    phases: Vec<f32>,
    last: Option<f32>,
    unwrapped: f32,
    active: Vec<bool>,
    hist: [f32; SLOTS],
    blocks: u64,
}

const SLOTS: usize = 100;

const KEEP_BLOCKS: usize = 61 * SLOTS;

const MARK: [bool; 10] = [true, true, true, false, false, true, true, true, false, false];

const SWING_RAD: f32 = 0.4;

const MARK_MATCH: usize = 8;

impl Default for Tdf {
    fn default() -> Self {
        Self::new()
    }
}

impl Tdf {
    pub fn new() -> Self {
        Self {
            phases: Vec::new(),
            last: None,
            unwrapped: 0.0,
            active: Vec::new(),
            hist: [0.0; SLOTS],
            blocks: 0,
        }
    }

    fn marked(&self, from_back: usize) -> Option<bool> {
        let at = self.active.len().checked_sub(from_back)?;
        let window = self.active.get(at..at + MARK.len())?;
        Some(window.iter().zip(MARK).filter(|(a, m)| **a == *m).count() >= MARK_MATCH)
    }

    fn onset(&self, p: usize) -> f32 {
        MARK.iter()
            .enumerate()
            .map(|(k, m)| {
                let h = self.hist[(p + k) % SLOTS];
                if *m { h } else { -h }
            })
            .sum()
    }

    pub fn push(&mut self, phase: f32) -> Option<Minute> {
        let step = self.last.map_or(0.0, |l| {
            (phase - l + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU)
                - std::f32::consts::PI
        });
        self.last = Some(phase);
        self.unwrapped += step;
        self.phases.push(self.unwrapped);
        if self.phases.len() > SLOTS {
            self.phases.remove(0);
        }
        let mut sorted = self.phases.clone();
        sorted.sort_by(f32::total_cmp);
        let middle = sorted[sorted.len() / 2];
        let on = (self.unwrapped - middle).abs() > SWING_RAD;
        self.active.push(on);
        if self.active.len() > KEEP_BLOCKS {
            self.active.remove(0);
        }
        let slot = (self.blocks % SLOTS as u64) as usize;
        self.blocks += 1;
        for h in self.hist.iter_mut() {
            *h *= 0.999;
        }
        self.hist[slot] += f32::from(u8::from(on));
        if self.blocks < 5 * SLOTS as u64 {
            return None;
        }
        let start = (0..SLOTS).max_by(|a, b| self.onset(*a).total_cmp(&self.onset(*b)))?;
        if (slot + SLOTS - start) % SLOTS != 2 * MARK.len() - 1 {
            return None;
        }
        let back = 2 * MARK.len();
        if self.marked(back)? {
            return None;
        }
        let mut bits = Vec::with_capacity(59);
        for k in 0..59 {
            let from = back + SLOTS * (59 - k);
            if !self.marked(from)? {
                return None;
            }
            bits.push(self.marked(from - MARK.len())?);
        }
        central_european(&bits, if bits[17] { 120 } else { 60 })
    }
}

fn valid(m: &Minute) -> bool {
    (1..=12).contains(&m.month) && (1..=31).contains(&m.day) && m.hour < 24 && m.minute < 60
}

#[cfg(test)]
mod tests {
    use super::*;

    fn msf_minute(m: &Minute) -> Vec<(u32, bool)> {
        let bcd_bits = |v: u32, weights: &[u32]| {
            let mut v = v;
            weights
                .iter()
                .map(|w| {
                    let on = v >= *w;
                    if on {
                        v -= w;
                    }
                    on
                })
                .collect::<Vec<bool>>()
        };
        let mut a = [false; 60];
        let mut b = [false; 60];
        a[17..25]
            .copy_from_slice(&bcd_bits(u32::from(m.year - 2000), &[80, 40, 20, 10, 8, 4, 2, 1]));
        a[25..30].copy_from_slice(&bcd_bits(u32::from(m.month), &[10, 8, 4, 2, 1]));
        a[30..36].copy_from_slice(&bcd_bits(u32::from(m.day), &[20, 10, 8, 4, 2, 1]));
        a[39..45].copy_from_slice(&bcd_bits(u32::from(m.hour), &[20, 10, 8, 4, 2, 1]));
        a[45..52].copy_from_slice(&bcd_bits(u32::from(m.minute), &[40, 20, 10, 8, 4, 2, 1]));
        for (k, r) in [(54, 17..25), (55, 25..36), (56, 36..39), (57, 39..52)] {
            b[k] = a[r].iter().filter(|x| **x).count() % 2 == 0;
        }
        let mut out = vec![(500, false)];
        for s in 1..60 {
            let off = match (a[s], b[s]) {
                (true, true) => 300,
                (true, false) => 200,
                _ => 100,
            };
            out.push((off, b[s] && !a[s]));
        }
        out
    }

    #[test]
    fn a_keyed_msf_minute_is_read() {
        let want =
            Minute { year: 2021, month: 11, day: 28, hour: 19, minute: 59, utc_offset_min: 0 };
        let mut msf = Msf::new();
        let mut got = None;
        for _ in 0..2 {
            for (off, second) in msf_minute(&want) {
                let s = Second::keyed(off, second, 1000);
                got = got.or(msf.push(s));
            }
        }
        assert_eq!(got, Some(want));
        assert_eq!(common::packet::clock_label(want.unix()), "2021-11-28 19:59 UTC");
        let p = read(&encode(Station::Msf, &want)).expect("a row");
        assert_eq!((p.id, p.kind), ("msf", "time"));
    }
}
