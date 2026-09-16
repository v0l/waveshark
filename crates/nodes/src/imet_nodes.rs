//! InterMet iMet radiosondes as a stage: a source's stream in,
//! transmissions out.
//!
//! Two layers, like APRS and for the same reason: the channel is narrowband
//! FM, and the data is in the audio as Bell 202 tones. What sits above the
//! tones is not HDLC but an ordinary asynchronous line, eight bits with a
//! start and a stop, so the symbols come from [`dsp::afsk::AfskBits`] and
//! this assembles characters from them.
//!
//! A transmission is a run of packets sent back to back and then silence, so
//! the line going idle is what ends it. What reaches the bus is that run,
//! packets and all, because [`decode::imet`] needs the position packet and
//! the weather packet together to name the sonde.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::imet;
use dsp::afsk::{AfskBits, AfskConfig, Symbol};
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// The meteorological aids band, where every sonde is.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// The channel an iMet occupies. Wider than the other sondes' because this
/// one is voice-bandwidth FM with tones in it rather than keyed data.
pub const CHANNEL_WIDTH_HZ: f64 = 16_000.0;

/// Audio rate the discriminator output is decimated to, as for APRS: well
/// above the 2200 Hz upper tone.
const AUDIO_HZ: f64 = 48_000.0;

/// Peak deviation an iMet keys.
const DEVIATION_HZ: f64 = 3_000.0;

/// Idle symbols that end a transmission. A stop bit is one symbol of mark
/// and the next character follows it immediately, so three in a row is the
/// line resting rather than a gap inside a packet.
const IDLE_SYMBOLS: usize = 3;

/// The longest run worth holding: a second's packets with room to spare.
const MAX_BYTES: usize = 256;

pub struct ImetNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    bits: AfskBits,
    line: Uart,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    symbols: Vec<Symbol>,
    /// The characters of the transmission being assembled.
    run: Vec<u8>,
    meter: crate::FrameMeter,
    frames: u64,
}

impl Default for ImetNode {
    fn default() -> Self {
        Self::new(403_000_000.0)
    }
}

impl ImetNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            bits: AfskBits::new(AUDIO_HZ, AfskConfig::default()),
            line: Uart::default(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            symbols: Vec::new(),
            run: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 2.0),
            frames: 0,
        }
    }

    /// Transmissions that held at least one packet.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Feed one symbol, and hand back the transmission where it ended one.
    fn feed(&mut self, sym: Symbol) -> Option<Vec<u8>> {
        match self.line.push(sym) {
            Read::Byte(b) => {
                // Bytes before the first `0x01` are the tail of something
                // missed or noise the slicer clocked, and a packet cannot
                // start anywhere else.
                if self.run.is_empty() && b != imet::SOH {
                    return None;
                }
                self.run.push(b);
                if self.run.len() > MAX_BYTES {
                    self.run.clear();
                }
                None
            }
            Read::Idle => self.take_run(),
            Read::Nothing => None,
        }
    }

    fn take_run(&mut self) -> Option<Vec<u8>> {
        if self.run.is_empty() {
            return None;
        }
        let run = std::mem::take(&mut self.run);
        // Held to what actually checked: a transmission is bytes off an
        // asynchronous line, and everything after a failed check is framing
        // that has slipped.
        let report = imet::parse(&run)?;
        if report.packets == 0 {
            return None;
        }
        self.frames += 1;
        Some(run)
    }
}

/// What reading one symbol produced.
enum Read {
    Byte(u8),
    /// The line has been resting long enough to end a transmission.
    Idle,
    Nothing,
}

/// An asynchronous line: a start bit, eight data bits least significant
/// first, and a stop bit.
///
/// Its own piece rather than part of the node because nothing about it is
/// this sonde's: it is how a serial port has worked since teleprinters, and
/// the next protocol that speaks one can take it.
#[derive(Default)]
struct Uart {
    /// Bits of the character so far, or `None` between characters.
    partial: Option<(u8, u32)>,
    idle: usize,
}

impl Uart {
    fn push(&mut self, sym: Symbol) -> Read {
        if sym.quiet {
            self.partial = None;
            self.idle += 1;
            return match self.idle == IDLE_SYMBOLS {
                true => Read::Idle,
                false => Read::Nothing,
            };
        }
        match &mut self.partial {
            // A mark between characters is the line resting.
            None if sym.mark => {
                self.idle += 1;
                match self.idle == IDLE_SYMBOLS {
                    true => Read::Idle,
                    false => Read::Nothing,
                }
            }
            // A space between characters is a start bit.
            None => {
                self.idle = 0;
                self.partial = Some((0, 0));
                Read::Nothing
            }
            Some((byte, have)) => {
                if *have < 8 {
                    *byte |= u8::from(sym.mark) << *have;
                    *have += 1;
                    return Read::Nothing;
                }
                // The stop bit. A space here is a framing slip, and the
                // character it would have made is not a character.
                let (byte, ok) = (*byte, sym.mark);
                self.partial = None;
                match ok {
                    true => Read::Byte(byte),
                    false => Read::Nothing,
                }
            }
        }
    }
}

impl Simple for ImetNode {
    fn name(&self) -> &str {
        "imet"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("imet reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        // Placed on its own channel, which for a sonde is wherever the
        // detector found one: the stream it is handed is that channel, so
        // the middle of the stream is the middle of the signal.
        self.channel_hz = center;
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(0.0, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        self.bits = AfskBits::new(audio_rate, AfskConfig::default());
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 2.0);

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
        self.audio.clear();
        self.fm.process(&self.narrow, &mut self.audio);
        self.meter.feed(&self.narrow);

        let audio = std::mem::take(&mut self.audio);
        let mut symbols = std::mem::take(&mut self.symbols);
        symbols.clear();
        self.bits.process(&audio, &mut symbols);
        for sym in &symbols {
            if let Some(run) = self.feed(*sym) {
                o.frames_mut().push(self.meter.frame(run));
            }
        }
        self.audio = audio;
        self.symbols = symbols;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.bits.reset();
        self.line = Uart::default();
        self.run.clear();
        self.meter.reset();
    }
}

/// What the protocols node makes of a transmission.
pub fn imet_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let r = imet::parse(bytes)?;
    let serial = r.name(center.as_f64());
    let mut fields: Vec<(String, common::Value)> =
        vec![("packet".into(), common::Value::Int(r.counter as i64))];
    if !serial.is_empty() {
        fields.push(("serial".into(), common::Value::Text(serial.clone())));
    }
    if r.has_position() {
        fields.push(("altitude_m".into(), common::Value::Float(r.altitude_m)));
        fields.push(("satellites".into(), common::Value::Int(r.satellites as i64)));
    }
    if r.speed_kt > 0.0 {
        fields.push(("speed_kt".into(), common::Value::Float(r.speed_kt)));
        fields.push(("course_deg".into(), common::Value::Float(r.course_deg)));
        fields.push(("climb_ms".into(), common::Value::Float(r.climb_ms)));
    }
    if let Some(v) = r.pressure_mbar {
        fields.push(("pressure_mbar".into(), common::Value::Float(v)));
    }
    if let Some(v) = r.temperature_c {
        fields.push(("temperature_c".into(), common::Value::Float(v)));
    }
    if let Some(v) = r.humidity_pct {
        fields.push(("humidity_pct".into(), common::Value::Float(v)));
    }
    if let Some(v) = r.battery_v {
        fields.push(("battery_v".into(), common::Value::Float(v)));
    }
    if let Some((h, m, s)) = r.utc {
        fields.push(("utc".into(), common::Value::Text(format!("{h:02}:{m:02}:{s:02}"))));
    }

    let mut d = Decoded::bytes("imet", center, 0.0, bytes.to_vec())
        .with_modulation(common::Modulation::Afsk)
        .with_crc(Some(true))
        .with_text(r.summary())
        .with_detail(format!("{} packets, counter {}", r.packets, r.counter))
        .with_fields(fields);
    if !serial.is_empty() {
        d = d.by(common::Identity::new("imet", serial.clone()).made_by("InterMet"));
    }
    if r.has_position() {
        d = d
            .reporting(common::ReportDetail::Sonde {
                altitude_m: r.altitude_m,
                climb_ms: r.climb_ms,
                battery_v: r.battery_v.unwrap_or(f64::NAN) as f32,
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

pub struct Imet;

impl Protocol for Imet {
    fn id(&self) -> &'static str {
        "imet"
    }
    fn label(&self) -> &'static str {
        "imet"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["imet4", "intermet"]
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
            min_rate_hz: AUDIO_HZ,
            feed_rate_hz: AUDIO_HZ,
            span_wide: false,
            families: &[],
        }
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    /// Claimed on the packets themselves: the band holds every make of
    /// sonde, and an iMet transmission is a run of packets that check.
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) || imet::packet_len(bytes).is_none() {
            return None;
        }
        Some(imet_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("imet")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "imet",
    summary: "InterMet iMet radiosonde packets, 1200 baud AFSK at 400 to 406 MHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(ImetNode::default()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bytes as an asynchronous line sends them: a start bit, eight data
    /// bits least significant first, a stop bit, and marks either side.
    fn keyed(bytes: &[u8], rate: f64) -> Vec<f32> {
        let mut levels: Vec<bool> = vec![true; 40];
        for b in bytes {
            levels.push(false);
            levels.extend((0..8).map(|k| b >> k & 1 != 0));
            levels.push(true);
        }
        levels.extend(std::iter::repeat_n(true, 40));
        tones(&levels, rate)
    }

    /// Levels as Bell 202 audio: the mark tone for a one, the space tone
    /// for a zero, with the phase carried across so there is no click at a
    /// symbol boundary.
    fn tones(levels: &[bool], rate: f64) -> Vec<f32> {
        let sps = rate / dsp::afsk::BAUD;
        let mut out = Vec::new();
        let mut phase = 0.0f64;
        for (i, &level) in levels.iter().enumerate() {
            let f = match level {
                true => dsp::afsk::MARK_HZ,
                false => dsp::afsk::SPACE_HZ,
            };
            while (out.len() as f64) < (i + 1) as f64 * sps {
                phase += std::f64::consts::TAU * f / rate;
                out.push(0.5 * phase.sin() as f32);
            }
        }
        out
    }

    fn a_transmission() -> Vec<u8> {
        let crc = |p: &[u8]| decode::bits::crc16(p, 0x1021, 0x0000).to_be_bytes();
        let mut gps = vec![imet::SOH, 0x02];
        gps.extend((53.35f32).to_le_bytes());
        gps.extend((-5.0f32).to_le_bytes());
        gps.extend((9_712u16).to_le_bytes());
        gps.push(11);
        gps.extend([5, 42, 20]);
        let c = crc(&gps);
        gps.extend(c);

        let mut ptu = vec![imet::SOH, 0x01];
        ptu.extend((1_745u16).to_le_bytes());
        ptu.extend(&55_321u32.to_le_bytes()[..3]);
        ptu.extend((-2_150i16).to_le_bytes());
        ptu.extend((1_275u16).to_le_bytes());
        ptu.push(41);
        let c = crc(&ptu);
        ptu.extend(c);

        gps.extend(ptu);
        gps
    }

    /// The whole chain this file is, on audio: the tones, the bit clock, the
    /// asynchronous framing, the run of packets and the name the sonde gets
    /// from the two of them together.
    #[test]
    fn a_keyed_transmission_is_read_off_the_audio() {
        let rate = 48_000.0;
        let frame = a_transmission();
        let audio = keyed(&frame, rate);

        let mut n =
            ImetNode { bits: AfskBits::new(rate, AfskConfig::default()), ..ImetNode::default() };
        let mut got: Vec<Vec<u8>> = Vec::new();
        for block in audio.chunks(1024) {
            let mut symbols = Vec::new();
            n.bits.process(block, &mut symbols);
            for sym in symbols {
                if let Some(run) = n.feed(sym) {
                    got.push(run);
                }
            }
        }
        assert_eq!(got.len(), 1, "{} transmissions", got.len());
        assert_eq!(got[0], frame, "the bytes are not the ones that were keyed");

        let d = imet_decoded(&got[0], common::Hz(403_000_000)).expect("a decode");
        assert_eq!(d.field("serial").map(|v| v.to_string()).as_deref(), Some("iMet-0513-4030"));
        assert_eq!(d.field("temperature_c").map(|v| v.to_string()).as_deref(), Some("-21.5"));
        let p = d.position.expect("a position");
        assert!((p.lat - 53.35).abs() < 1e-5, "{}", p.lat);
        assert!((p.lon + 5.0).abs() < 1e-5, "{}", p.lon);
        assert_eq!(p.altitude_m, Some(4_712.0));
    }

    /// Twenty seconds of noise produces no transmissions. The line will
    /// frame characters out of anything; the packet checks are what refuse
    /// them.
    #[test]
    fn noise_produces_no_transmissions() {
        let rate = 48_000.0;
        let mut seed = 0xfeed_face_dead_beefu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let audio: Vec<f32> = (0..rate as usize * 20).map(|_| rng()).collect();
        let mut n =
            ImetNode { bits: AfskBits::new(rate, AfskConfig::default()), ..ImetNode::default() };
        let mut runs = 0;
        for block in audio.chunks(4096) {
            let mut symbols = Vec::new();
            n.bits.process(block, &mut symbols);
            for sym in symbols {
                runs += n.feed(sym).is_some() as usize;
            }
        }
        assert_eq!(runs, 0, "{runs} transmissions out of twenty seconds of noise");
    }
}
