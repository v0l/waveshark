//! Meteomodem M10 and M20 radiosondes as a stage: a source's stream in,
//! frames out.
//!
//! Both key 9600-odd chips a second with a transition in every chip pair, so
//! the data rate is half of that and a frame of a hundred bytes takes a
//! sixth of a second. The two differ by 15 baud and by everything above the
//! waveform, so one node reads both and [`decode::m10`] says which it was.
//!
//! The coding is differential rather than Manchester: a bit is whether this
//! chip pair went the same way as the one before it, which is why nothing
//! here has to know which way up the receiver put the signal. What the sync
//! header does is find the frame, not the polarity.
//!
//! The waveform and the header are from zilog80's `rs1729/RS`,
//! `demod/mod/m10m20mod.c`.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::m10;
use dsp::fsk::BitSync;
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// Chips a second. An M10 keys 9615 and an M20 9600, which is a sixth of a
/// chip apart over a whole frame and well inside what the clock recovery
/// follows, so both are read at the one rate.
pub const BAUD: f64 = 9_615.0;

/// The channel a Meteomodem sonde is tuned to.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

/// What the signal occupies.
pub const OCCUPIED_HZ: f64 = 20_000.0;

/// The meteorological aids band.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// The sync header, as chips. Not a byte of the frame: the frame's own
/// length and type follow it, and this is what says where they start.
const SYNC: [bool; 32] = {
    let raw = *b"10011001100110010100110010011001";
    let mut out = [false; 32];
    let mut i = 0;
    while i < 32 {
        out[i] = raw[i] == b'1';
        i += 1;
    }
    out
};

/// Chips of the sync allowed to be wrong. There is no error correction in
/// either sonde, so a false sync costs one checksum and nothing else.
const SYNC_SLACK: u32 = 4;

/// Chips held while looking for a sync: two of the longest frames and their
/// headers.
const MAX_CHIPS: usize = (m10::MAX_FRAME + 2) * 8 * 2 * 2;

pub struct M10Node {
    sync: Option<BitSync>,
    meter: crate::FrameMeter,
    chips: Vec<bool>,
    /// Chips already searched and known not to start a sync.
    scanned: usize,
    frames: u64,
}

impl Default for M10Node {
    fn default() -> Self {
        Self::new()
    }
}

impl M10Node {
    pub fn new() -> Self {
        Self {
            sync: None,
            meter: crate::FrameMeter::new(1.0, 0, 0.6),
            chips: Vec::new(),
            scanned: 0,
            frames: 0,
        }
    }

    /// Frames whose checksum held.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Look for syncs in the chips held, returning every frame behind one.
    fn search(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let mut at = self.scanned;
        while at + SYNC.len() <= self.chips.len() {
            if !synced(&self.chips, at) {
                at += 1;
                continue;
            }
            match self.read_frame(at + SYNC.len(), self.chips[at + SYNC.len() - 1]) {
                Some(Some(frame)) => {
                    self.frames += 1;
                    let used = at + SYNC.len() + frame.len() * 16;
                    out.push(frame);
                    self.chips.drain(..used.min(self.chips.len()));
                    at = 0;
                    self.scanned = 0;
                }
                // A sync whose frame has not all arrived: wait here, so a
                // frame split across two blocks is not walked past.
                Some(None) => {
                    self.scanned = at;
                    return out;
                }
                None => at += 1,
            }
        }
        self.scanned = at;
        out
    }

    /// The frame starting at chip `at`. `None` where those chips are not a
    /// frame, `Some(None)` where not enough of them have arrived.
    ///
    /// The length is in the frame's first byte, so two bytes are read to
    /// find out how many more to read.
    ///
    /// `seed` is the pair the first bit is measured against, which is the
    /// last pair of the sync header: the sonde's differential encoder ran
    /// through the header without stopping, and taking the reference from
    /// there is what makes the whole frame read the same either way up.
    /// Where that gives no frame the other reference is tried, since it can
    /// only change the first bit of the length byte and trying it costs one
    /// checksum.
    fn read_frame(&self, at: usize, seed: bool) -> Option<Option<Vec<u8>>> {
        let mut short = false;
        for seed in [seed, !seed] {
            let Some(head) = bytes(&self.chips, at, 2, seed) else {
                short = true;
                continue;
            };
            let Some(len) = m10::declared_len(&head) else { continue };
            let Some(frame) = bytes(&self.chips, at, len + 1, seed) else {
                short = true;
                continue;
            };
            if m10::check_ok(&frame) {
                return Some(Some(frame));
            }
        }
        match short {
            true => Some(None),
            false => None,
        }
    }
}

/// Whether the sync sits at `at`, either way up. Which way is not worth
/// keeping: the frame behind it is differentially coded, so it reads the
/// same whichever way the receiver put it.
fn synced(chips: &[bool], at: usize) -> bool {
    let mut wrong = [0u32; 2];
    for (k, &want) in SYNC.iter().enumerate() {
        wrong[(chips[at + k] == want) as usize] += 1;
    }
    wrong[0] <= SYNC_SLACK || wrong[1] <= SYNC_SLACK
}

/// `count` bytes of frame from chip `at`, or `None` where the chips have not
/// all arrived.
///
/// Two chips make a bit and the bit is whether the pair went the same way as
/// the pair before it, the first pair being measured against a fall. Bits
/// are most significant first within a byte.
fn bytes(chips: &[bool], at: usize, count: usize, seed: bool) -> Option<Vec<u8>> {
    if at + count * 16 > chips.len() {
        return None;
    }
    let mut out = vec![0u8; count];
    let mut last = seed;
    for i in 0..count * 8 {
        let pair = chips[at + 2 * i + 1];
        out[i / 8] = out[i / 8] << 1 | u8::from(pair == last);
        last = pair;
    }
    Some(out)
}

impl Simple for M10Node {
    fn name(&self) -> &str {
        "m10"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("m10 reads complex baseband"));
        }
        let s = BitSync::with_bandwidth(i.spec.rate, BAUD, OCCUPIED_HZ);
        if !s.usable() {
            return Err(common::Error::other(format!(
                "m10 needs at least {} S/s for its {BAUD} chips a second",
                4.0 * BAUD
            )));
        }
        self.sync = Some(s);
        self.meter = crate::FrameMeter::new(i.spec.rate, i.spec.center.0, 0.5);
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
        for frame in self.search() {
            o.frames_mut().push(self.meter.frame(frame));
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
        if let Some(s) = &mut self.sync {
            s.reset();
        }
    }
}

/// What the protocols node makes of a Meteomodem frame.
pub fn m10_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let r = m10::parse(bytes)?;
    let mut fields: Vec<(String, common::Value)> = vec![
        ("model".into(), common::Value::Text(r.model.label().into())),
        ("serial".into(), common::Value::Text(r.serial.clone())),
        ("counter".into(), common::Value::Int(r.counter as i64)),
    ];
    if r.has_position() {
        fields.push(("altitude_m".into(), common::Value::Float(r.altitude_m)));
        fields.push(("climb_ms".into(), common::Value::Float(r.climb_ms)));
        fields.push(("speed_kt".into(), common::Value::Float(r.speed_kt)));
        fields.push(("course_deg".into(), common::Value::Float(r.course_deg)));
    }
    if r.satellites > 0 {
        fields.push(("satellites".into(), common::Value::Int(r.satellites as i64)));
    }
    if r.gps_week > 0 {
        fields.push(("gps_week".into(), common::Value::Int(r.gps_week as i64)));
    }
    if let Some((y, mo, d, h, mi, s)) = r.utc {
        fields.push((
            "utc".into(),
            common::Value::Text(format!("{y:04}-{mo:02}-{d:02} {h:02}:{mi:02}:{s:04.1}")),
        ));
    }

    let mut d = Decoded::bytes("m10", center, 0.0, bytes.to_vec())
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(true))
        .with_text(r.summary())
        .with_detail(format!("{}, counter {}", r.model.label(), r.counter))
        .with_fields(fields)
        .by(common::Identity::new("meteomodem", r.serial.clone()).made_by("Meteomodem"));
    if r.has_position() {
        d = d
            .reporting(common::ReportDetail::Sonde {
                altitude_m: r.altitude_m,
                climb_ms: r.climb_ms,
                // Neither sonde sends its battery voltage in the standard
                // part of the frame; not-a-number is how a sonde track says
                // a reading has not been read.
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

pub struct M10;

impl Protocol for M10 {
    fn id(&self) -> &'static str {
        "m10"
    }
    fn label(&self) -> &'static str {
        "m10"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["m20", "meteomodem"]
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
            feed_rate_hz: 96_000.0,
            span_wide: false,
            families: &[],
        }
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    /// The band is shared with the other sondes, so a frame is claimed on
    /// its own length byte, its type byte and its checksum rather than on
    /// where it was heard.
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) || m10::frame_len(bytes).is_none() {
            return None;
        }
        Some(m10_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("m10")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "m10",
    summary: "Meteomodem M10 and M20 radiosonde frames, 9600 baud FSK at 400 to 406 MHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(M10Node::new()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame on the air: the sync header, then every bit as a pair of
    /// chips that either repeats the last pair or turns it over.
    fn keyed(frame: &[u8]) -> Vec<bool> {
        let mut chips: Vec<bool> = SYNC.to_vec();
        // The encoder runs on through the header, so the first bit is
        // measured against the header's last pair.
        let mut last = *SYNC.last().unwrap();
        for byte in frame {
            for k in (0..8).rev() {
                let bit = byte >> k & 1 != 0;
                // The bit says whether this pair goes the same way as the
                // one before it, so a one repeats and a zero turns over.
                let pair = last == bit;
                chips.extend([!pair, pair]);
                last = pair;
            }
        }
        chips
    }

    fn an_m10() -> Vec<u8> {
        const B60B60: f64 = (1u32 << 30) as f64 / 90.0;
        let mut f = vec![0u8; 0x65];
        f[0] = 0x64;
        f[1] = 0x9F;
        let put = |f: &mut Vec<u8>, at: usize, b: &[u8]| f[at..at + b.len()].copy_from_slice(b);
        put(&mut f, 0x04, &(1_800i16).to_be_bytes());
        put(&mut f, 0x08, &(1_000i16).to_be_bytes());
        put(&mut f, 0x0A, &(452_540_000u32).to_be_bytes());
        put(&mut f, 0x0E, &((53.35 * B60B60) as i32).to_be_bytes());
        put(&mut f, 0x12, &((-5.0 * B60B60) as i32).to_be_bytes());
        put(&mut f, 0x16, &(4_712_220i32).to_be_bytes());
        put(&mut f, 0x1E, &[11]);
        put(&mut f, 0x20, &(2_357u16).to_be_bytes());
        put(&mut f, 0x5D, &[0x03, 0x00, 0x2A, 0x21, 0x4A]);
        put(&mut f, 0x62, &[57]);
        let cs = decode::m10::check(&f[..0x63]).to_be_bytes();
        put(&mut f, 0x63, &cs);
        f
    }

    /// The whole chain this file is, on samples: the chip clock, the sync
    /// search, the differential decode and the checksum. Run both ways up,
    /// because a differentially coded frame does not care and this proves
    /// it.
    #[test]
    fn a_keyed_frame_is_read_off_the_samples() {
        for inverted in [false, true] {
            let rate = 96_000.0;
            let frame = an_m10();
            let mut wire: Vec<bool> = (0..200).map(|i| i % 2 == 0).collect();
            wire.extend(keyed(&frame).into_iter().map(|c| c != inverted));
            wire.extend((0..40).map(|i| i % 2 == 0));
            let iq = dsp::fsk::modulate(&wire, rate, BAUD, 4_800.0, 0.5);

            let mut n = M10Node::new();
            n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
            let mut got = Vec::new();
            for block in iq.chunks(2048) {
                let mut chips = Vec::new();
                n.sync.as_mut().unwrap().process(block, &mut chips);
                n.chips.extend(chips);
                got.extend(n.search());
            }
            assert_eq!(got.len(), 1, "{} frames, inverted {inverted}", got.len());
            assert_eq!(got[0], frame, "the bytes are not the ones that were keyed");

            let d = m10_decoded(&got[0], common::Hz(403_000_000)).expect("a decode");
            assert_eq!(d.field("model").map(|v| v.to_string()).as_deref(), Some("M10"));
            let p = d.position.expect("a position");
            assert!((p.lat - 53.35).abs() < 1e-6, "{}", p.lat);
            assert!((p.lon + 5.0).abs() < 1e-6, "{}", p.lon);
            assert!((p.altitude_m.unwrap() - 4_712.22).abs() < 0.01, "{:?}", p.altitude_m);
        }
    }

    /// Twenty seconds of noise produces no frames. The sync is 32 chips with
    /// four of slack, so candidates turn up regularly and the checksum is
    /// what refuses them.
    #[test]
    fn noise_produces_no_frames() {
        let rate = 96_000.0;
        let mut seed = 0x1357_9bdf_2468_ace0u64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<common::C32> =
            (0..rate as usize * 20).map(|_| common::C32::new(rng(), rng())).collect();
        let mut n = M10Node::new();
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
