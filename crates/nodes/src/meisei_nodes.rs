//! Meisei iMS-100 radiosondes as a stage: a source's stream in, gathered
//! records out.
//!
//! 2400 chips a second, biphase coded, so the data rate is 1200 bits a
//! second and each half-frame of 300 bits takes a quarter of a second. The
//! waveform is [`dsp::fsk::BitSync`] at the chip rate, the frame is
//! [`decode::meisei`], and this is the wire between them: find one of the two
//! half-frame headers in the chips, take the coding off, read the six
//! codewords, and hand the half to the gatherer that pairs them.
//!
//! The coding carries no polarity of its own: a bit is whether the two chips
//! of a pair are the same, which does not change when the receiver turns the
//! signal over. Only the header search has to look both ways, and only to
//! find the pairs.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::meisei;
use dsp::fsk::BitSync;
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// Chips a second. Two to a bit.
pub const BAUD: f64 = 2_400.0;

/// The channel an iMS-100 is tuned to.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// What the signal occupies.
pub const OCCUPIED_HZ: f64 = 12_000.0;

/// The meteorological aids band.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// Chips in one half-frame.
const HALF_CHIPS: usize = meisei::HALF_BITS * 2;

/// Chips held while looking for a header: three half-frames.
const MAX_CHIPS: usize = HALF_CHIPS * 3;

/// Chips of the header allowed to be wrong. The BCH code behind it refuses
/// what a false header would produce, so slack here costs nothing.
const HEADER_SLACK: u32 = 6;

pub struct MeiseiNode {
    sync: Option<BitSync>,
    meter: crate::FrameMeter,
    gather: meisei::Gather,
    chips: Vec<bool>,
    /// Chips already searched and known not to start a header.
    scanned: usize,
    halves: u64,
    records: u64,
}

impl Default for MeiseiNode {
    fn default() -> Self {
        Self::new()
    }
}

impl MeiseiNode {
    pub fn new() -> Self {
        Self {
            sync: None,
            meter: crate::FrameMeter::new(1.0, 0, 0.6),
            gather: meisei::Gather::new(),
            chips: Vec::new(),
            scanned: 0,
            halves: 0,
            records: 0,
        }
    }

    /// Half-frames whose codewords all came through the BCH.
    pub fn halves(&self) -> u64 {
        self.halves
    }

    /// Records gathered, which is what reaches the bus.
    pub fn records(&self) -> u64 {
        self.records
    }

    fn search(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let headers = header_chips();
        let mut at = self.scanned;
        while at + 48 <= self.chips.len() {
            if !headers.iter().any(|h| matches(&self.chips, at, h)) {
                at += 1;
                continue;
            }
            if at + HALF_CHIPS > self.chips.len() {
                self.scanned = at;
                return out;
            }
            let bits = biphase(&self.chips[at..at + HALF_CHIPS]);
            match meisei::read(&bits) {
                Some(half) => {
                    self.halves += 1;
                    if let Some(record) = self.gather.take(&half) {
                        self.records += 1;
                        out.push(record);
                    }
                    self.chips.drain(..at + HALF_CHIPS);
                    at = 0;
                    self.scanned = 0;
                }
                None => at += 1,
            }
        }
        self.scanned = at;
        out
    }
}

/// Both headers, each as chips, each way up: four patterns, because a header
/// is what fixes the chip pairing and the pairing is what the coding needs.
fn header_chips() -> Vec<Vec<bool>> {
    let mut out = Vec::with_capacity(4);
    for header in [meisei::HEADER_A, meisei::HEADER_B] {
        let mut chips = Vec::with_capacity(48);
        // Biphase: the level turns over at every bit, and again in the
        // middle of a zero.
        let mut level = false;
        for k in (0..24).rev() {
            level = !level;
            chips.push(level);
            if header >> k & 1 == 0 {
                level = !level;
            }
            chips.push(level);
        }
        out.push(chips.iter().map(|c| !c).collect());
        out.push(chips);
    }
    out
}

fn matches(chips: &[bool], at: usize, header: &[bool]) -> bool {
    let mut wrong = 0;
    for (k, &want) in header.iter().enumerate() {
        wrong += u32::from(chips[at + k] != want);
        if wrong > HEADER_SLACK {
            return false;
        }
    }
    true
}

/// Biphase chips back to bits: a pair that does not turn over is a one.
/// Nothing here depends on which way up the signal arrived.
fn biphase(chips: &[bool]) -> Vec<bool> {
    chips.chunks(2).map(|c| c.len() == 2 && c[0] == c[1]).collect()
}

impl Simple for MeiseiNode {
    fn name(&self) -> &str {
        "ims100"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("ims100 reads complex baseband"));
        }
        let s = BitSync::with_bandwidth(i.spec.rate, BAUD, OCCUPIED_HZ);
        if !s.usable() {
            return Err(common::Error::other(format!(
                "ims100 needs at least {} S/s for its {BAUD} chips a second",
                4.0 * BAUD
            )));
        }
        self.sync = Some(s);
        self.meter = crate::FrameMeter::new(i.spec.rate, i.spec.center.0, 1.0);
        let mut out = i.spec.with_kind(PortKind::Frames);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(i.spec.rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let (Some(iq), Some(s)) = (i.as_iq(), self.sync.as_mut()) else {
            return Ok(());
        };
        self.meter.feed(iq);
        s.process(iq, &mut self.chips);
        for record in self.search() {
            o.frames_mut().push(self.meter.frame(record));
        }
        if self.chips.len() > MAX_CHIPS {
            let drop = self.chips.len() - MAX_CHIPS;
            self.chips.drain(..drop);
            self.scanned = self.scanned.saturating_sub(drop);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        self.chips.clear();
        self.scanned = 0;
        self.gather = meisei::Gather::new();
        if let Some(s) = &mut self.sync {
            s.reset();
        }
    }
}

/// The year the receiver is running in, for the decade the sonde leaves off.
fn this_year() -> u16 {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    use chrono::Datelike;
    chrono::DateTime::from_timestamp(secs as i64, 0).map(|t| t.year() as u16).unwrap_or(1970)
}

/// What the protocols node makes of a gathered record.
pub fn meisei_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let r = meisei::parse(bytes)?;
    let year = meisei::year_near(r.year_digit, this_year());
    let (h, m, s) = r.utc;
    let mut fields: Vec<(String, common::Value)> = vec![
        ("model".into(), common::Value::Text("iMS-100".into())),
        ("frame".into(), common::Value::Int(r.counter as i64)),
        (
            "utc".into(),
            common::Value::Text(format!(
                "{year:04}-{:02}-{:02} {h:02}:{m:02}:{s:06.3}",
                r.month, r.day
            )),
        ),
    ];
    if !r.serial.is_empty() {
        fields.push(("serial".into(), common::Value::Text(r.serial.clone())));
    }
    if r.has_position() {
        fields.push(("altitude_m".into(), common::Value::Float(r.altitude_m)));
        fields.push(("climb_ms".into(), common::Value::Float(r.climb_ms)));
        fields.push(("speed_kt".into(), common::Value::Float(r.speed_kt)));
        fields.push(("course_deg".into(), common::Value::Float(r.course_deg)));
    }

    let mut d = Decoded::bytes("ims100", center, 0.0, bytes.to_vec())
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(true))
        .with_text(r.summary())
        .with_detail(format!("iMS-100, frame {}", r.counter))
        .with_fields(fields);
    if !r.serial.is_empty() {
        d = d.by(common::Identity::new("meisei", r.serial.clone()).made_by("Meisei"));
    }
    if r.has_position() {
        d = d
            .reporting(common::ReportDetail::Sonde {
                altitude_m: r.altitude_m,
                climb_ms: r.climb_ms,
                // The standard frame carries no battery voltage.
                battery_v: f32::NAN,
                satellites: 0,
                descending: r.climb_ms < -1.0,
                sensors: None,
            })
            .at_position(common::Position {
                lat: r.lat_deg,
                lon: r.lon_deg,
                altitude_m: Some(r.altitude_m),
                speed_kt: Some(r.speed_kt),
                course_deg: Some(r.course_deg),
            });
    }
    Some(d)
}

pub struct Meisei;

impl Protocol for Meisei {
    fn id(&self) -> &'static str {
        "ims100"
    }
    fn label(&self) -> &'static str {
        "ims100"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["meisei", "ims-100", "rs-11g"]
    }
    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }
    fn default_hz(&self) -> f64 {
        403_000_000.0
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 4.0 * BAUD,
            feed_rate_hz: 48_000.0,
            span_wide: false,
            families: &[],
        }
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) || bytes.len() != meisei::RECORD {
            return None;
        }
        Some(meisei_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("ims100")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "ims100",
    summary: "Meisei iMS-100 radiosonde frames, 2400 baud biphase FSK at 400 to 406 MHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(MeiseiNode::new()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bits as chips: the level turns over at every bit, and again in the
    /// middle of a zero.
    fn keyed(bits: &[bool]) -> Vec<bool> {
        let mut chips = Vec::with_capacity(bits.len() * 2);
        let mut level = false;
        for &bit in bits {
            level = !level;
            chips.push(level);
            if !bit {
                level = !level;
            }
            chips.push(level);
        }
        chips
    }

    /// One half-frame's bits, built the way the decoder's own test does.
    fn half(kind: meisei::Half, words: &[u16; meisei::WORDS]) -> Vec<bool> {
        const GEN: u64 = 0b1_0101_0011_1001;
        let header = match kind {
            meisei::Half::First => meisei::HEADER_A,
            meisei::Half::Second => meisei::HEADER_B,
        };
        let mut bits: Vec<bool> = (0..24).map(|i| header >> (23 - i) & 1 != 0).collect();
        for block in 0..6 {
            let mut data: Vec<bool> = Vec::with_capacity(34);
            for w in 0..2 {
                let value = words[block * 2 + w];
                data.extend((0..16).map(|i| value >> (15 - i) & 1 != 0));
                data.push(value.count_ones().is_multiple_of(2));
            }
            let mut acc = 0u64;
            for bit in &data {
                acc = acc << 1 | u64::from(*bit);
                if acc >> 12 & 1 != 0 {
                    acc ^= GEN;
                }
            }
            for _ in 0..12 {
                acc <<= 1;
                if acc >> 12 & 1 != 0 {
                    acc ^= GEN;
                }
            }
            bits.extend(data);
            bits.extend((0..12).map(|i| acc >> (11 - i) & 1 != 0));
        }
        bits
    }

    fn a_flight() -> ([u16; meisei::WORDS], [u16; meisei::WORDS]) {
        let mut a = [0u16; meisei::WORDS];
        a[0] = 1_746;
        a[10] = 20_500;
        a[11] = 5 << 8 | 42;
        let mut b = [0u16; meisei::WORDS];
        b[0] = 13 * 1000 + 3 * 10 + 5;
        let (lat, lon) = (53_210_000u32, 5_000_000u32);
        b[1] = (lat >> 16) as u16;
        b[2] = lat as u16;
        b[3] = (lon >> 16) as u16;
        b[4] = lon as u16;
        let alt = 471_222u32;
        b[5] = (alt >> 8) as u16;
        b[6] = (alt as u16) << 8;
        b[9] = 20_000;
        b[10] = 1_749;
        b[11] = a[10]
            .wrapping_add(a[11])
            .wrapping_add(b[..11].iter().fold(0u16, |s, w| s.wrapping_add(*w)));
        (a, b)
    }

    /// The whole chain this file is, on samples, and both ways up: the chip
    /// clock, the header search, the biphase decode, the BCH and the pairing
    /// of two half-frames into one fix.
    #[test]
    fn a_keyed_pair_is_read_off_the_samples() {
        for inverted in [false, true] {
            let rate = 48_000.0;
            let (a, b) = a_flight();
            let mut wire: Vec<bool> = (0..200).map(|i| i % 2 == 0).collect();
            wire.extend(keyed(&half(meisei::Half::First, &a)));
            wire.extend(keyed(&half(meisei::Half::Second, &b)));
            wire.extend((0..40).map(|i| i % 2 == 0));
            let iq = dsp::fsk::modulate(
                &wire.iter().map(|c| c != &inverted).collect::<Vec<_>>(),
                rate,
                BAUD,
                1_200.0,
                0.5,
            );

            let mut n = MeiseiNode::new();
            n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
            let mut got = Vec::new();
            for block in iq.chunks(2048) {
                let mut chips = Vec::new();
                n.sync.as_mut().unwrap().process(block, &mut chips);
                n.chips.extend(chips);
                got.extend(n.search());
            }
            assert_eq!(got.len(), 1, "{} records, inverted {inverted}", got.len());
            assert_eq!(n.halves(), 2, "{} half-frames read", n.halves());

            let d = meisei_decoded(&got[0], common::Hz(403_000_000)).expect("a decode");
            assert_eq!(d.field("model").map(|v| v.to_string()).as_deref(), Some("iMS-100"));
            let p = d.position.expect("a position");
            assert!((p.lat - 53.35).abs() < 1e-6, "{}", p.lat);
            assert!((p.lon - 5.0).abs() < 1e-6, "{}", p.lon);
            assert!((p.altitude_m.unwrap() - 4_712.22).abs() < 0.01, "{:?}", p.altitude_m);
        }
    }

    /// Twenty seconds of noise produces no records.
    #[test]
    fn noise_produces_no_records() {
        let rate = 48_000.0;
        let mut seed = 0x0bad_c0de_dead_10ccu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<common::C32> =
            (0..rate as usize * 20).map(|_| common::C32::new(rng(), rng())).collect();
        let mut n = MeiseiNode::new();
        n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
        let mut records = 0;
        for block in iq.chunks(4096) {
            let mut chips = Vec::new();
            n.sync.as_mut().unwrap().process(block, &mut chips);
            n.chips.extend(chips);
            records += n.search().len();
            if n.chips.len() > MAX_CHIPS {
                let drop = n.chips.len() - MAX_CHIPS;
                n.chips.drain(..drop);
                n.scanned = n.scanned.saturating_sub(drop);
            }
        }
        assert_eq!(records, 0, "{records} records out of twenty seconds of noise");
    }
}
