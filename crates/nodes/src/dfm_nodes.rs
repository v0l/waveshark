//! Graw DFM radiosondes as a stage: a source's stream in, gathered records
//! out.
//!
//! A DFM keys 2500 chips a second, Manchester coded, so the data rate is
//! 1250 bits a second and a 280-bit frame takes 224 ms of air. The waveform
//! is [`dsp::fsk::BitSync`] at the chip rate, the frame is
//! [`decode::dfm`], and this is the wire between them: find the header in
//! the chips, take the Manchester coding off, read the frame, and hand it to
//! the gatherer that turns nine of them into one position.
//!
//! Polarity is not fixed. A DFM-06 and a DFM-09 key the same coding the
//! other way up, and which way a receiver sees it also depends on the
//! receiver, so the header is looked for both ways round and whichever
//! matches says how to read the rest.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::dfm;
use dsp::fsk::BitSync;
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// Chips a second. Two chips to a bit, so the sonde sends 1250 bits a
/// second.
pub const BAUD: f64 = 2_500.0;

/// The channel a DFM is tuned to. The meteorological band is stepped in
/// 10 kHz, and the sonde occupies most of a 12.5 kHz channel.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// What the signal occupies, which is the intermediate filter zilog80's
/// decoder defaults to.
pub const OCCUPIED_HZ: f64 = 12_000.0;

/// The meteorological aids band, the same one the RS41 is launched into.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// Chips in one frame.
const FRAME_CHIPS: usize = dfm::FRAME_BITS * 2;

/// Chips held while looking for a header: three frames, so a frame
/// straddling two blocks is never lost and a quiet channel cannot grow.
const MAX_CHIPS: usize = FRAME_CHIPS * 3;

/// Chips of the header allowed to be wrong. The header is 32 chips and
/// every nibble behind it is protected, so a false header costs one Hamming
/// failure and nothing else.
const HEADER_SLACK: u32 = 4;

pub struct DfmNode {
    sync: Option<BitSync>,
    meter: crate::FrameMeter,
    gather: dfm::Gather,
    chips: Vec<bool>,
    /// Chips already searched and known not to start a header.
    scanned: usize,
    frames: u64,
    records: u64,
}

impl Default for DfmNode {
    fn default() -> Self {
        Self::new()
    }
}

impl DfmNode {
    pub fn new() -> Self {
        Self {
            sync: None,
            meter: crate::FrameMeter::new(1.0, 0, 0.6),
            gather: dfm::Gather::new(),
            chips: Vec::new(),
            scanned: 0,
            frames: 0,
            records: 0,
        }
    }

    /// Frames whose every nibble came through the Hamming code.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Records gathered, which is what reaches the bus.
    pub fn records(&self) -> u64 {
        self.records
    }

    /// Look for headers in the chips held, returning every record a frame
    /// behind one completed.
    fn search(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let header = header_chips();
        let mut at = self.scanned;
        while at + header.len() <= self.chips.len() {
            let Some(inverted) = matches(&self.chips, at, &header) else {
                at += 1;
                continue;
            };
            if at + FRAME_CHIPS > self.chips.len() {
                // Not enough of the frame has arrived. Waiting here rather
                // than walking past it is what keeps a frame that straddles
                // two blocks.
                self.scanned = at;
                return out;
            }
            let bits = manchester(&self.chips[at..at + FRAME_CHIPS], inverted);
            match dfm::read(&bits) {
                Some(frame) => {
                    self.frames += 1;
                    if let Some(record) = self.gather.take(&frame) {
                        self.records += 1;
                        out.push(record);
                    }
                    self.chips.drain(..at + FRAME_CHIPS);
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

/// Whether the header sits at `at`, and which way up it is. `Some(true)`
/// where the chips are inverted, which is how a DFM-06 and a DFM-09 differ.
fn matches(chips: &[bool], at: usize, header: &[bool]) -> Option<bool> {
    let mut wrong = [0u32; 2];
    for (k, &want) in header.iter().enumerate() {
        let got = chips[at + k];
        wrong[(got == want) as usize] += 1;
        if wrong[0] > HEADER_SLACK && wrong[1] > HEADER_SLACK {
            return None;
        }
    }
    match (wrong[0] <= HEADER_SLACK, wrong[1] <= HEADER_SLACK) {
        (true, _) => Some(false),
        (_, true) => Some(true),
        _ => None,
    }
}

/// The header as it arrives: every bit as two chips, `01` for a one.
fn header_chips() -> Vec<bool> {
    (0..16)
        .flat_map(|k| {
            let bit = dfm::HEADER >> (15 - k) & 1 != 0;
            [!bit, bit]
        })
        .collect()
}

/// Manchester chips back to bits. The second chip of each pair is the bit,
/// and a pair that is not a transition is left to the Hamming code: half a
/// wrong pair is one wrong bit, which is what that code is for.
fn manchester(chips: &[bool], inverted: bool) -> Vec<bool> {
    chips.chunks(2).map(|c| c[c.len() - 1] != inverted).collect()
}

impl Simple for DfmNode {
    fn name(&self) -> &str {
        "dfm"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("dfm reads complex baseband"));
        }
        let s = BitSync::with_bandwidth(i.spec.rate, BAUD, OCCUPIED_HZ);
        if !s.usable() {
            return Err(common::Error::other(format!(
                "dfm needs at least {} S/s for its {BAUD} chips a second",
                4.0 * BAUD
            )));
        }
        self.sync = Some(s);
        self.meter = crate::FrameMeter::new(i.spec.rate, i.spec.center.0, 2.5);
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
        self.gather = dfm::Gather::new();
        if let Some(s) = &mut self.sync {
            s.reset();
        }
    }
}

/// What the protocols node makes of a gathered record.
pub fn dfm_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let r = dfm::parse(bytes)?;
    let mut fields: Vec<(String, common::Value)> = vec![
        ("model".into(), common::Value::Text(r.model.label().into())),
        ("frame".into(), common::Value::Int(r.frame_no as i64)),
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
    if r.satellites > 0 {
        fields.push(("satellites".into(), common::Value::Int(r.satellites as i64)));
    }
    if let Some((y, mo, d, h, mi)) = r.date {
        fields.push((
            "utc".into(),
            common::Value::Text(format!("{y:04}-{mo:02}-{d:02} {h:02}:{mi:02}")),
        ));
    }

    // The serial is what a chaser follows and what SondeHub files a flight
    // under; a sonde that has not sent both halves of it yet is still a
    // sonde, so it is reported without an identity rather than under a
    // made-up one.
    let mut d = Decoded::bytes("dfm", center, 0.0, bytes.to_vec())
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(true))
        .with_text(r.summary())
        .with_detail(format!("{}, frame {}", r.model.label(), r.frame_no))
        .with_fields(fields);
    if !r.serial.is_empty() {
        d = d.by(common::Identity::new("graw", r.serial.clone()).made_by("Graw"));
    }
    if r.has_position() {
        d = d
            .reporting(common::ReportDetail::Sonde {
                altitude_m: r.altitude_m,
                climb_ms: r.climb_ms,
                // A DFM sends no battery voltage; not-a-number is how a
                // sonde track says a reading has not been read.
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

pub struct Dfm;

impl Protocol for Dfm {
    fn id(&self) -> &'static str {
        "dfm"
    }
    fn label(&self) -> &'static str {
        "dfm"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["graw", "dfm09", "dfm17"]
    }
    /// The meteorological aids allocation, as for the RS41: 2500 baud FSK in
    /// a 12.5 kHz channel is a common enough shape, and outside this band
    /// none of it is a sonde.
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
        if !(BAND.0..BAND.1).contains(&hz) || bytes.len() != dfm::RECORD {
            return None;
        }
        Some(dfm_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("dfm")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "dfm",
    summary: "Graw DFM radiosonde frames, 2500 baud Manchester FSK at 400 to 406 MHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(DfmNode::new()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame's chips: the header, then every bit of the frame as a
    /// transition.
    fn chips(bits: &[bool]) -> Vec<bool> {
        bits.iter().flat_map(|&b| [!b, b]).collect()
    }

    /// One data block's nibbles, Hamming coded and interleaved, as the frame
    /// carries them. The same arrangement the decoder undoes.
    fn put(bits: &mut [bool], at: usize, nibs: &[u8]) {
        let rows: [u8; 4] = [0b0111_0000, 0b1011_0000, 0b1101_0000, 0b1110_0000];
        let l = nibs.len();
        for (i, nib) in nibs.iter().enumerate() {
            let mut code = nib << 4;
            for (r, row) in rows.iter().enumerate() {
                code |= ((row & code).count_ones() as u8 & 1) << (3 - r);
            }
            for j in 0..8 {
                bits[at + l * j + i] = code >> (7 - j) & 1 != 0;
            }
        }
    }

    /// A frame carrying two data blocks and one configuration channel.
    fn frame(conf: [u8; 7], a: [u8; 13], b: [u8; 13]) -> Vec<bool> {
        let mut bits = vec![false; dfm::FRAME_BITS];
        for (k, bit) in bits.iter_mut().take(16).enumerate() {
            *bit = dfm::HEADER >> (15 - k) & 1 != 0;
        }
        put(&mut bits, 16, &conf);
        put(&mut bits, 16 + 56, &a);
        put(&mut bits, 16 + 160, &b);
        bits
    }

    fn block(id: u8, payload: u64) -> [u8; 13] {
        let mut nibs = [0u8; 13];
        for (i, nib) in nibs.iter_mut().enumerate().take(12) {
            *nib = (payload >> (4 * (11 - i))) as u8 & 0xF;
        }
        nibs[12] = id;
        nibs
    }

    fn a_flight() -> Vec<bool> {
        let lat = 533_500_000u32 as u64;
        let lon = (-50_000_000i32) as u32 as u64;
        let alt = 471_222u64;
        let blocks = [
            block(0, 2 << 24 | 0x71 << 16),
            block(1, 40_000),
            block(2, lat << 16 | 900),
            block(3, lon << 16 | 20_000),
            block(4, alt << 16 | 500),
            block(5, 0),
            block(6, 0),
            block(7, 0),
            block(8, 0x7E9 << 36 | 3 << 32 | 13 << 27 | 5 << 22 | 42 << 16 | 11 << 8),
        ];
        // Channel 0xB with both halves of the serial, which names it a
        // DFM-17.
        let conf = |half: u8, v: u16| {
            let mut c = [0u8; 7];
            c[0] = 0xB;
            c[1] = 0xC;
            for (i, n) in (0..4).map(|i| (v >> (4 * (3 - i))) as u8 & 0xF).enumerate() {
                c[2 + i] = n;
            }
            c[6] = half;
            c
        };
        let mut bits = Vec::new();
        for (i, pair) in blocks.chunks(2).enumerate() {
            let b = pair.get(1).copied().unwrap_or(pair[0]);
            let c = match i {
                0 => conf(0, 0x0163),
                _ => conf(1, 0x4A21),
            };
            bits.extend(frame(c, pair[0], b));
        }
        bits
    }

    /// The whole chain this file is, on samples: the chip clock, the header
    /// search, the Manchester decode, and the gathering that turns nine
    /// blocks into one fix.
    #[test]
    fn a_keyed_flight_is_read_off_the_samples() {
        for inverted in [false, true] {
            let rate = 48_000.0;
            let mut wire: Vec<bool> = (0..200).map(|i| i % 2 == 0).collect();
            wire.extend(chips(&a_flight()).into_iter().map(|c| c != inverted));
            wire.extend((0..40).map(|i| i % 2 == 0));
            let iq = dsp::fsk::modulate(&wire, rate, BAUD, 2_400.0, 0.5);

            let mut n = DfmNode::new();
            n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
            let mut got = Vec::new();
            for block in iq.chunks(2048) {
                let mut chips = Vec::new();
                n.sync.as_mut().unwrap().process(block, &mut chips);
                n.chips.extend(chips);
                got.extend(n.search());
            }
            assert_eq!(got.len(), 1, "{} records, inverted {inverted}", got.len());
            assert_eq!(n.frames(), 5, "{} frames read", n.frames());

            let d = dfm_decoded(&got[0], common::Hz(403_000_000)).expect("a decode");
            assert_eq!(d.field("model").map(|v| v.to_string()).as_deref(), Some("DFM-17"));
            assert_eq!(
                d.field("serial").map(|v| v.to_string()),
                Some(format!("{}", 0x0163_4A21u32))
            );
            let p = d.position.expect("a position");
            assert!((p.lat - 53.35).abs() < 1e-6, "{}", p.lat);
            assert!((p.lon + 5.0).abs() < 1e-6, "{}", p.lon);
            assert!((p.altitude_m.unwrap() - 4_712.22).abs() < 0.01, "{:?}", p.altitude_m);
        }
    }

    /// Twenty seconds of noise produces no records. The header is 32 chips
    /// with four of slack, so candidates turn up regularly and the Hamming
    /// code is what refuses them.
    #[test]
    fn noise_produces_no_records() {
        let rate = 48_000.0;
        let mut seed = 0x2468_ace0_1357_9bdfu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<common::C32> =
            (0..rate as usize * 20).map(|_| common::C32::new(rng(), rng())).collect();
        let mut n = DfmNode::new();
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
