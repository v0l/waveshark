//! Meteo-Radiy MRZ radiosondes as a stage: a source's stream in, gathered
//! records out.
//!
//! 2400 chips a second, Manchester coded, so the data rate is 1200 bits a
//! second. The frame is found by its own first three bytes rather than by a
//! preamble: an MRZ leads with a run of `0xAA`, which is the same alternating
//! pattern as its preamble, so `AA BF 35` is what says where the frame
//! starts and a preamble correlation says only that one is coming.
//!
//! The frame is [`decode::mrz`], the waveform [`dsp::fsk::BitSync`], and the
//! layout is from zilog80's `rs1729/RS`, `demod/mod/mp3h1mod.c`.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::mrz;
use dsp::fsk::BitSync;
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// Chips a second. Two to a bit.
pub const BAUD: f64 = 2_400.0;

/// The channel an MRZ is tuned to.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// What the signal occupies.
pub const OCCUPIED_HZ: f64 = 12_000.0;

/// The meteorological aids band.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// Chips in the longest frame.
const FRAME_CHIPS: usize = mrz::FRAME_ECEF * 8 * 2;

/// Chips held while looking for a frame: three of them.
const MAX_CHIPS: usize = FRAME_CHIPS * 3;

/// Chips of the three sync bytes allowed to be wrong. The CRC behind them
/// refuses what a false sync would produce.
const SYNC_SLACK: u32 = 4;

pub struct MrzNode {
    sync: Option<BitSync>,
    meter: crate::FrameMeter,
    gather: mrz::Gather,
    chips: Vec<bool>,
    scanned: usize,
    frames: u64,
}

impl Default for MrzNode {
    fn default() -> Self {
        Self::new()
    }
}

impl MrzNode {
    pub fn new() -> Self {
        Self {
            sync: None,
            meter: crate::FrameMeter::new(1.0, 0, 0.6),
            gather: mrz::Gather::new(),
            chips: Vec::new(),
            scanned: 0,
            frames: 0,
        }
    }

    /// Frames whose check held.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    fn search(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let sync = sync_chips();
        let mut at = self.scanned;
        while at + sync[0].len() <= self.chips.len() {
            let Some(inverted) = sync.iter().position(|s| matches(&self.chips, at, s)) else {
                at += 1;
                continue;
            };
            if at + FRAME_CHIPS > self.chips.len() {
                self.scanned = at;
                return out;
            }
            let frame = manchester(&self.chips[at..at + FRAME_CHIPS], inverted == 1);
            match self.gather.take(&frame) {
                Some(record) => {
                    self.frames += 1;
                    let used = at + record.len().saturating_sub(12) * 16;
                    out.push(record);
                    self.chips.drain(..used.min(self.chips.len()));
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

/// The three sync bytes as chips, each way up. Manchester: a one is a fall
/// and a zero a rise, or the other way about when the receiver has the
/// signal over.
fn sync_chips() -> [Vec<bool>; 2] {
    let upright: Vec<bool> = mrz::SYNC
        .iter()
        .flat_map(|b| (0..8).rev().map(move |k| b >> k & 1 != 0))
        .flat_map(|bit| [bit, !bit])
        .collect();
    let inverted = upright.iter().map(|c| !c).collect();
    [upright, inverted]
}

fn matches(chips: &[bool], at: usize, want: &[bool]) -> bool {
    let mut wrong = 0;
    for (k, &w) in want.iter().enumerate() {
        wrong += u32::from(chips[at + k] != w);
        if wrong > SYNC_SLACK {
            return false;
        }
    }
    true
}

/// Manchester chips back to bytes, most significant bit first.
fn manchester(chips: &[bool], inverted: bool) -> Vec<u8> {
    let bits: Vec<bool> = chips.chunks(2).map(|c| (c[0] && !c[c.len() - 1]) != inverted).collect();
    bits.chunks(8).map(|b| b.iter().fold(0u8, |v, bit| v << 1 | u8::from(*bit))).collect()
}

impl Simple for MrzNode {
    fn name(&self) -> &str {
        "mrz"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("mrz reads complex baseband"));
        }
        let s = BitSync::with_bandwidth(i.spec.rate, BAUD, OCCUPIED_HZ);
        if !s.usable() {
            return Err(common::Error::other(format!(
                "mrz needs at least {} S/s for its {BAUD} chips a second",
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
        self.gather = mrz::Gather::new();
        if let Some(s) = &mut self.sync {
            s.reset();
        }
    }
}

/// What the protocols node makes of a gathered record.
pub fn mrz_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let r = mrz::parse(bytes)?;
    let (h, m, s) = r.utc;
    let mut fields: Vec<(String, common::Value)> = vec![
        ("model".into(), common::Value::Text("MRZ".into())),
        ("utc".into(), common::Value::Text(format!("{h:02}:{m:02}:{s:02}"))),
        ("altitude_m".into(), common::Value::Float(r.altitude_m)),
        ("climb_ms".into(), common::Value::Float(r.climb_ms)),
        ("speed_kt".into(), common::Value::Float(r.speed_kt)),
        ("course_deg".into(), common::Value::Float(r.course_deg)),
    ];
    if !r.serial.is_empty() {
        fields.push(("serial".into(), common::Value::Text(r.serial.clone())));
    }
    if r.satellites > 0 {
        fields.push(("satellites".into(), common::Value::Int(r.satellites as i64)));
    }
    if let Some((y, mo, d)) = r.date {
        fields.push(("date".into(), common::Value::Text(format!("{y:04}-{mo:02}-{d:02}"))));
    }

    let mut d = Decoded::bytes("mrz", center, 0.0, bytes.to_vec())
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(true))
        .with_text(r.summary())
        .with_detail(format!("MRZ, {h:02}:{m:02}:{s:02} UTC"))
        .with_fields(fields);
    if !r.serial.is_empty() {
        d = d.by(common::Identity::new("mrz", r.serial.clone()).made_by("Meteo-Radiy"));
    }
    if r.has_position() {
        d = d
            .reporting(common::ReportDetail::Sonde {
                altitude_m: r.altitude_m,
                climb_ms: r.climb_ms,
                // The frame carries sensor counts rather than a battery
                // voltage; not-a-number is how a track says unread.
                battery_v: f32::NAN,
                satellites: r.satellites,
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

pub struct Mrz;

impl Protocol for Mrz {
    fn id(&self) -> &'static str {
        "mrz"
    }
    fn label(&self) -> &'static str {
        "mrz"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["mp3h1", "meteo-radiy"]
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
        if !(BAND.0..BAND.1).contains(&hz)
            || !matches!(bytes.len(), mrz::RECORD_ECEF | mrz::RECORD_LATLON)
        {
            return None;
        }
        Some(mrz_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("mrz")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "mrz",
    summary: "Meteo-Radiy MRZ radiosonde frames, 2400 baud Manchester FSK at 400 to 406 MHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(MrzNode::new()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bytes as chips: a one is a fall, a zero a rise.
    fn keyed(frame: &[u8]) -> Vec<bool> {
        frame
            .iter()
            .flat_map(|b| (0..8).rev().map(move |k| b >> k & 1 != 0))
            .flat_map(|bit| [bit, !bit])
            .collect()
    }

    fn a_frame() -> Vec<u8> {
        const A: f64 = 6_378_137.0;
        const E2: f64 = 6.694_379_990_141_32e-3;
        let (lat, lon, alt) = (53.35f64, -5.0f64, 4_712.22f64);
        let (sla, cla) = lat.to_radians().sin_cos();
        let (slo, clo) = lon.to_radians().sin_cos();
        let n = A / (1.0 - E2 * sla * sla).sqrt();
        let cm = |v: f64| ((v * 100.0).round() as i32).to_le_bytes();

        let mut f = vec![0u8; mrz::FRAME_ECEF];
        f[..3].copy_from_slice(&mrz::SYNC);
        f[3] = 0x81;
        f[4..7].copy_from_slice(&[5, 42, 20]);
        f[8..12].copy_from_slice(&cm((n + alt) * cla * clo));
        f[12..16].copy_from_slice(&cm((n + alt) * cla * slo));
        f[16..20].copy_from_slice(&cm((n * (1.0 - E2) + alt) * sla));
        f[26] = 11;
        let cs = decode::bits::crc16le(&f[3..48], 0x8005, 0xFFFF).to_le_bytes();
        f[48..50].copy_from_slice(&cs);
        f
    }

    /// The whole chain this file is, on samples and both ways up: the chip
    /// clock, the sync search, the Manchester decode and the frame's own
    /// check.
    #[test]
    fn a_keyed_frame_is_read_off_the_samples() {
        for inverted in [false, true] {
            let rate = 48_000.0;
            let frame = a_frame();
            // A run of 0xAA in front, which is what an MRZ leads with and
            // what the frame's own first byte looks like.
            let mut wire: Vec<bool> = keyed(&[0xAA; 8]);
            wire.extend(keyed(&frame));
            wire.extend((0..40).map(|i| i % 2 == 0));
            let iq = dsp::fsk::modulate(
                &wire.iter().map(|c| c != &inverted).collect::<Vec<_>>(),
                rate,
                BAUD,
                1_200.0,
                0.5,
            );

            let mut n = MrzNode::new();
            n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
            let mut got = Vec::new();
            for block in iq.chunks(2048) {
                let mut chips = Vec::new();
                n.sync.as_mut().unwrap().process(block, &mut chips);
                n.chips.extend(chips);
                got.extend(n.search());
            }
            assert_eq!(got.len(), 1, "{} records, inverted {inverted}", got.len());
            assert_eq!(got[0][..mrz::FRAME_ECEF], frame[..], "not the bytes that were keyed");

            let d = mrz_decoded(&got[0], common::Hz(403_000_000)).expect("a decode");
            let p = d.position.expect("a position");
            assert!((p.lat - 53.35).abs() < 1e-6, "{}", p.lat);
            assert!((p.lon + 5.0).abs() < 1e-6, "{}", p.lon);
            assert!((p.altitude_m.unwrap() - 4_712.22).abs() < 0.05, "{:?}", p.altitude_m);
        }
    }

    /// Twenty seconds of noise produces no frames.
    #[test]
    fn noise_produces_no_frames() {
        let rate = 48_000.0;
        let mut seed = 0x5eed_1234_9876_fedcu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<common::C32> =
            (0..rate as usize * 20).map(|_| common::C32::new(rng(), rng())).collect();
        let mut n = MrzNode::new();
        n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
        let mut frames = 0;
        for block in iq.chunks(4096) {
            let mut chips = Vec::new();
            n.sync.as_mut().unwrap().process(block, &mut chips);
            n.chips.extend(chips);
            frames += n.search().len();
            if n.chips.len() > MAX_CHIPS {
                let drop = n.chips.len() - MAX_CHIPS;
                n.chips.drain(..drop);
                n.scanned = n.scanned.saturating_sub(drop);
            }
        }
        assert_eq!(frames, 0, "{frames} frames out of twenty seconds of noise");
    }
}
