//! COSPAS-SARSAT 406 MHz distress beacons as a stage: a channel in,
//! messages out.
//!
//! The waveform is [`dsp::biphase`] and the message is [`decode::epirb`];
//! what is here is the wiring between them. A burst is 160 ms of unmodulated
//! carrier and then 112 or 144 bits at 400 baud, so the demodulator has a
//! quarter of a second of carrier to settle its reference on before the
//! preamble arrives, and the whole transmission is over in half a second.
//!
//! The preamble is matched on the chips rather than on bits, which is what
//! says where a bit starts and which way up the modulation was. Both frame
//! synchronisation patterns are searched: a beacon under test keys the
//! complemented one so the satellites ignore it, and a receiver on the
//! ground should say what it heard either way.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::epirb::{self, Beacon, Coding, Identity, Mode};
use decode::framing;
use dsp::biphase::{BiphaseDemod, CHIPS_PER_BIT};
use dsp::{FirDecim, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// The 406 MHz distress band. Nothing else may transmit in it, and beacons
/// sit on channels 3 kHz apart across the lower half of it.
pub const BAND: (f64, f64) = (406_000_000.0, 406_100_000.0);

/// The channel to offer when somebody places one by hand.
pub const DEFAULT_HZ: f64 = 406_025_000.0;

/// How much of the band one decoder reads. Wide enough to hold a beacon
/// keyed anywhere near the channel it was set to, and to leave the carrier
/// tracker something to find.
pub const CHANNEL_WIDTH_HZ: f64 = 20_000.0;

/// The rate the chips are recovered at: twelve samples a chip, which is what
/// the zero-crossing clock wants and no more.
const WORK_HZ: f64 = 9_600.0;

/// The rate to ask the receiver for, which decimates to [`WORK_HZ`] by four.
const FEED_HZ: f64 = 38_400.0;

pub const BAUD: f64 = 400.0;

/// Chips of the preamble that may disagree and still count as a match.
///
/// The preamble is 24 bits, so 48 chips. Measured on synthesised bursts in
/// noise: at 8 allowed, every burst is read down to the 7.8 dB the chips
/// themselves survive, and ten minutes of noise produces no message at all,
/// because the two codes refuse what the preamble let through.
const SYNC_TOLERANCE: usize = 5;

/// Bits of the preamble the search matches on: the frame synchronisation
/// pattern and the tail of the bit synchronisation run.
const SYNC_BITS: usize = 16;

/// How far the phase must swing, in radians, averaged over the chips the
/// preamble was matched on, before the match counts as a transmission.
///
/// A beacon keys 1.1 radians either side of the carrier. Measured as the
/// mean size of a chip: 0.95 radians on the keyed message, 0.00 to 0.11 on
/// the unmodulated carrier in front of it, and 0.37 on noise alone, where
/// the phase is uniform and a chip averages what the integration leaves. So
/// this separates a message from both, and without it four beacons come out
/// of ten minutes of noise: a pattern match on noise is rare but ten
/// minutes is a million chances at it.
const MIN_SWING_RAD: f32 = 0.6;

/// A beacon message, less the preamble, in chips.
const SHORT_CHIPS: usize = (epirb::SHORT_BITS - 24) * CHIPS_PER_BIT;
const LONG_CHIPS: usize = (epirb::LONG_BITS - 24) * CHIPS_PER_BIT;

pub struct EpirbNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    demod: BiphaseDemod,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    chips: Vec<f32>,
    /// The chips of the preamble just matched, and what it said.
    reading: Option<Reading>,
    /// The last chips seen, for the preamble search.
    window: Vec<f32>,
    meter: crate::FrameMeter,
    frames: u64,
}

/// A message being read off the chips that followed a preamble.
struct Reading {
    mode: Mode,
    inverted: bool,
    chips: Vec<f32>,
}

impl Default for EpirbNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl EpirbNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(WORK_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            demod: BiphaseDemod::new(WORK_HZ, BAUD),
            mixed: Vec::new(),
            narrow: Vec::new(),
            chips: Vec::new(),
            reading: None,
            window: Vec::new(),
            meter: crate::FrameMeter::new(WORK_HZ, channel_hz as u64, 2.0),
            frames: 0,
        }
    }

    /// Messages that checked.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Feed one chip, and hand back the message where it completed one.
    ///
    /// The bits of the preamble are known rather than read, so what is
    /// emitted is the whole transmission from bit one, which is 14 bytes for
    /// a short message and 18 for a long one.
    fn feed(&mut self, chip: f32) -> Option<Vec<u8>> {
        if let Some(mut reading) = self.reading.take() {
            reading.chips.push(chip);
            // The format flag is the first bit after the preamble, so how
            // much is still to come is known two chips in.
            let want = match reading.chips.first().zip(reading.chips.get(1)) {
                Some((a, b)) => match (a - b > 0.0) != reading.inverted {
                    true => LONG_CHIPS,
                    false => SHORT_CHIPS,
                },
                None => LONG_CHIPS,
            };
            if reading.chips.len() < want {
                self.reading = Some(reading);
                return None;
            }
            let mut bits = reading.mode.preamble();
            let read = framing::biphase_l_bits(&reading.chips, reading.inverted);
            bits.extend((0..read.len()).filter_map(|i| read.get(i)));
            let bytes: Vec<u8> = bits
                .chunks(8)
                .map(|b| b.iter().fold(0u8, |acc, v| acc << 1 | u8::from(*v)))
                .collect();
            epirb::parse(&bytes)?;
            self.frames += 1;
            return Some(bytes);
        }

        // One chip more than the preamble, because the decision is taken a
        // chip late: a preamble that opens with a run of identical bits
        // reads almost as well one chip early and upside down, and only the
        // two scores side by side tell them apart.
        let preamble = SYNC_BITS * CHIPS_PER_BIT;
        self.window.push(chip);
        if self.window.len() > preamble + 1 {
            self.window.remove(0);
        }
        if self.window.len() <= preamble {
            return None;
        }
        let swing = self.window[..preamble].iter().map(|c| c.abs()).sum::<f32>() / preamble as f32;
        if swing < MIN_SWING_RAD {
            return None;
        }
        for mode in [Mode::Distress, Mode::SelfTest] {
            let all = mode.preamble();
            let bits = &all[all.len() - SYNC_BITS..];
            let here = framing::biphase_l_match(&self.window[..preamble], bits);
            let next = framing::biphase_l_match(&self.window[1..], bits);
            let Some((wrong, inverted)) = here else { continue };
            if wrong > SYNC_TOLERANCE || next.is_some_and(|(w, _)| w < wrong) {
                continue;
            }
            // The preamble ended a chip ago, so the message starts with the
            // chip that has just arrived.
            let mut chips = Vec::with_capacity(LONG_CHIPS);
            chips.push(chip);
            self.reading = Some(Reading { mode, inverted, chips });
            self.window.clear();
            break;
        }
        None
    }
}

impl Simple for EpirbNode {
    fn name(&self) -> &str {
        "epirb"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("epirb reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        self.channel_hz = center;
        let factor = (rate / WORK_HZ).round().max(1.0) as usize;
        let work = rate / factor as f64;
        self.mixer = Mixer::new(0.0, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.demod = BiphaseDemod::new(work, BAUD);
        self.meter = crate::FrameMeter::new(work, self.channel_hz as u64, 2.0);
        self.reading = None;
        self.window.clear();

        let mut out = i.spec.with_kind(PortKind::Frames);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);

        let mut chips = std::mem::take(&mut self.chips);
        chips.clear();
        self.demod.process(&self.narrow, &mut chips);
        for chip in &chips {
            if let Some(message) = self.feed(*chip) {
                o.frames_mut().push(self.meter.frame(message));
            }
        }
        self.chips = chips;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.demod.reset();
        self.reading = None;
        self.window.clear();
        self.meter.reset();
    }
}

/// What the receiver makes of a beacon message.
pub fn epirb_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let b = epirb::parse(bytes)?;
    let mut fields: Vec<(String, common::Value)> = vec![
        ("hex_id".into(), common::Value::Text(b.hex_id.clone())),
        ("country".into(), common::Value::Int(i64::from(b.country))),
        ("protocol".into(), common::Value::Text(b.coding.label().into())),
        ("beacon".into(), common::Value::Text(b.kind().into())),
        (
            "mode".into(),
            common::Value::Text(
                match b.mode {
                    Mode::Distress => "distress",
                    Mode::SelfTest => "self test",
                }
                .into(),
            ),
        ),
    ];
    match b.identity {
        Identity::Mmsi { last_six, beacon } => {
            fields.push(("mmsi_last_six".into(), common::Value::Int(i64::from(last_six))));
            fields.push(("beacon_number".into(), common::Value::Int(i64::from(beacon))));
        }
        Identity::AircraftAddress(a) => {
            fields.push(("aircraft_address".into(), common::Value::Text(format!("{a:06X}"))));
        }
        Identity::Serial { certificate, serial } => {
            fields.push(("certificate".into(), common::Value::Int(i64::from(certificate))));
            fields.push(("serial".into(), common::Value::Int(i64::from(serial))));
        }
        Identity::Unknown => {}
    }
    if let Some(homing) = b.homing_121_5 {
        fields.push(("homing_121_5".into(), common::Value::Bool(homing)));
    }
    if b.corrected > 0 {
        fields.push(("corrected_bits".into(), common::Value::Int(i64::from(b.corrected))));
    }

    let mut d = Decoded::bytes("epirb", center, 0.0, bytes.to_vec())
        .with_modulation(common::Modulation::Psk2)
        .with_crc(Some(true))
        .with_text(b.summary())
        .with_detail(detail(&b))
        .with_fields(fields)
        .by(common::Identity::new("epirb", b.hex_id.clone()).named(b.kind()));
    if let Some((lat, lon)) = b.position {
        d = d.at_position(common::Position {
            lat,
            lon,
            altitude_m: None,
            speed_kt: None,
            course_deg: None,
        });
    }
    Some(d)
}

fn detail(b: &Beacon) -> String {
    let coding = match b.coding {
        Coding::User(_) => "user protocol",
        Coding::Location(_) => "location protocol",
    };
    format!("{}, {coding}, country {}", b.coding.label(), b.country)
}

pub struct Epirb;

impl Protocol for Epirb {
    fn id(&self) -> &'static str {
        "epirb"
    }
    fn label(&self) -> &'static str {
        "epirb"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["cospas-sarsat", "sarsat", "plb", "elt", "406"]
    }
    /// The whole allocation rather than one channel: beacons are assigned
    /// channels 3 kHz apart across it, and nothing else may transmit there,
    /// so anything found in the band is worth pointing this at.
    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: WORK_HZ,
            feed_rate_hz: FEED_HZ,
            span_wide: false,
            families: &[],
        }
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) {
            return None;
        }
        Some(epirb_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("epirb")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "epirb",
    summary: "COSPAS-SARSAT 406 MHz distress beacons: EPIRBs, PLBs and ELTs",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(EpirbNode::default()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::C32;

    /// The message an EPIRB with an MMSI transmits, as bytes off the air.
    fn a_message() -> Vec<u8> {
        let num = |value: u64, width: usize| -> Vec<bool> {
            (0..width).rev().map(|i| value >> i & 1 != 0).collect()
        };
        let mut pdf1 = vec![true, false];
        pdf1.extend(num(232, 10));
        pdf1.extend(num(0b0010, 4));
        pdf1.extend(num(123_456, 20));
        pdf1.extend(num(1, 4));
        pdf1.push(false);
        pdf1.extend(num(53 * 4 + 1, 9));
        pdf1.push(true);
        pdf1.extend(num(10 * 4 + 1, 10));
        let mut pdf2 = vec![true, true, false, true, true, false];
        pdf2.push(true);
        pdf2.extend(num(6, 5));
        pdf2.extend(num(9, 4));
        pdf2.push(false);
        pdf2.extend(num(3, 5));
        pdf2.extend(num(7, 4));
        keyed_bytes(&pdf1, &pdf2)
    }

    fn keyed_bytes(pdf1: &[bool], pdf2: &[bool]) -> Vec<u8> {
        use decode::bits::{BCH_63_51_GEN, BCH_127_106_GEN, bch_parity};
        let mut bits = Mode::Distress.preamble();
        bits.extend_from_slice(pdf1);
        let p = bch_parity(pdf1, BCH_127_106_GEN, 21);
        bits.extend((0..21).rev().map(|i| p >> i & 1 != 0));
        bits.extend_from_slice(pdf2);
        let p = bch_parity(pdf2, BCH_63_51_GEN, 12);
        bits.extend((0..12).rev().map(|i| p >> i & 1 != 0));
        bits.chunks(8).map(|b| b.iter().fold(0u8, |acc, v| acc << 1 | u8::from(*v))).collect()
    }

    fn bits_of(bytes: &[u8]) -> Vec<bool> {
        bytes.iter().flat_map(|b| (0..8).map(move |i| b >> (7 - i) & 1 != 0)).collect()
    }

    /// A burst as a beacon sends it: 160 ms of unmodulated carrier, then the
    /// message keyed biphase-L at 1.1 radians, then silence.
    fn a_burst(bytes: &[u8], rate: f64, offset_hz: f64, quiet_s: f64) -> Vec<C32> {
        let carrier = |n: usize, from: usize| -> Vec<C32> {
            (from..from + n)
                .map(|i| {
                    C32::from_polar(
                        1.0,
                        (std::f64::consts::TAU * offset_hz * i as f64 / rate) as f32,
                    )
                })
                .collect()
        };
        let mut out = carrier((rate * 0.16) as usize, 0);
        out.extend(dsp::biphase::key_biphase(&bits_of(bytes), rate, BAUD, 1.1, offset_hz, 150e-6));
        out.extend(std::iter::repeat_n(C32::new(0.0, 0.0), (rate * quiet_s) as usize));
        out
    }

    fn noisy(iq: &[C32], amplitude: f32, seed: u64) -> Vec<C32> {
        let mut s = seed;
        let mut next = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 40) as f32 / 8_388_608.0 - 1.0
        };
        iq.iter().map(|x| x + C32::new(next() * amplitude, next() * amplitude)).collect()
    }

    fn read(stream: &[C32], rate: f64) -> Vec<Vec<u8>> {
        let mut n = EpirbNode::new(DEFAULT_HZ);
        n.demod = BiphaseDemod::new(rate, BAUD);
        let mut got = Vec::new();
        for block in stream.chunks(1024) {
            let mut chips = Vec::new();
            n.demod.process(block, &mut chips);
            for chip in chips {
                if let Some(message) = n.feed(chip) {
                    got.push(message);
                }
            }
        }
        got
    }

    /// The whole of this file on a keyed burst: the carrier, the chips, the
    /// preamble matched on them, the message and the fields.
    #[test]
    fn a_keyed_burst_is_read_off_the_carrier() {
        let rate = 9_600.0;
        let air = a_message();
        let got = read(&a_burst(&air, rate, 700.0, 0.1), rate);
        assert_eq!(got.len(), 1, "{} messages", got.len());
        assert_eq!(got[0], air, "the bytes are not the ones that were keyed");

        let d = epirb_decoded(&got[0], common::Hz(406_025_000)).expect("a decode");
        assert_eq!(d.field("hex_id").map(|v| v.to_string()).as_deref(), Some("1D043C4802FFBFF"));
        assert_eq!(d.field("mmsi_last_six").map(|v| v.to_string()).as_deref(), Some("123456"));
        assert_eq!(d.field("beacon").map(|v| v.to_string()).as_deref(), Some("EPIRB"));
        assert_eq!(d.field("country").map(|v| v.to_string()).as_deref(), Some("232"));
        let p = d.position.expect("a position");
        assert!((p.lat - 53.36).abs() < 1e-6, "{}", p.lat);
        assert!((p.lon + 10.192_222).abs() < 1e-5, "{}", p.lon);
    }

    /// A beacon under test keys the complemented frame synchronisation so
    /// the satellites ignore it. A receiver on the ground says what it
    /// heard, and says which it was.
    #[test]
    fn a_self_test_burst_is_read_and_labelled() {
        let rate = 9_600.0;
        let mut air = a_message();
        // The self-test pattern is the last eight bits of the normal one
        // complemented, which is bits 17 to 24 of the message.
        // Bits 17 to 24 are the whole of the third byte.
        air[2] ^= 0xff;
        let got = read(&a_burst(&air, rate, 0.0, 0.1), rate);
        assert_eq!(got.len(), 1, "{} messages", got.len());
        let d = epirb_decoded(&got[0], common::Hz(406_025_000)).expect("a decode");
        assert_eq!(d.field("mode").map(|v| v.to_string()).as_deref(), Some("self test"));
        assert_eq!(d.field("hex_id").map(|v| v.to_string()).as_deref(), Some("1D043C4802FFBFF"));
    }

    /// How many of `n` bursts come back unchanged with noise of `amp` added
    /// to each component of every sample.
    fn read_bursts(n: usize, amp: f32, seed: u64) -> usize {
        let rate = 9_600.0;
        let air = a_message();
        let mut stream = Vec::new();
        for _ in 0..n {
            stream.extend(a_burst(&air, rate, 300.0, 0.05));
        }
        let stream = noisy(&stream, amp, seed);
        read(&stream, rate).iter().filter(|m| **m == air).count()
    }

    /// Twenty bursts in noise, which is the whole of this file twenty times
    /// over. Every burst is read at 7.8 dB of signal to noise, which is what
    /// the chips themselves were measured to survive, and 19 of 20 at
    /// 6.2 dB, where the last one is lost in the demodulator rather than in
    /// the codes.
    #[test]
    fn a_beacon_repeating_in_noise_is_read_every_time() {
        let seed = 0x51ee_beef_1234_5678;
        assert_eq!(read_bursts(20, 0.5, seed), 20, "at 7.8 dB");
        assert_eq!(read_bursts(20, 0.6, seed), 19, "at 6.2 dB");
    }

    /// Ten minutes of noise produces no beacon. The preamble will match
    /// something eventually; the two codes are what refuse it.
    #[test]
    fn noise_produces_no_beacons() {
        let rate = 9_600.0;
        let quiet: Vec<C32> = vec![C32::new(0.0, 0.0); (rate * 600.0) as usize];
        let stream = noisy(&quiet, 1.0, 0xdead_beef_cafe_f00d);
        let got = read(&stream, rate);
        assert_eq!(got.len(), 0, "{} beacons out of ten minutes of noise", got.len());
    }
}
