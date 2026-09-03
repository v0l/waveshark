//! POCSAG as a graph node.
//!
//! The same shape as APRS and for the same reason: the channel is ordinary
//! narrowband FM, so the node mixes the pager channel down, filters it,
//! discriminates it, and hands the audio to `dsp::pocsag`, which does the bit
//! recovery, the sync search and the error correction. The message tables are
//! `decode::pocsag`. Neither of those knows about pipelines.
//!
//! What reaches the bus is a transmission's codewords, every one of which
//! passed BCH(31,21) or was corrected by it. Where APRS carries an AX.25
//! frame that passed a check sequence, this carries a run of codewords that
//! passed theirs.

use common::Result;
use decode::pocsag::{self, Body};
use dsp::pocsag::{PocsagConfig, PocsagDemod, Transmission, DEVIATION_HZ};
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::event::{media, Decoded};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// A common UK and European paging channel, and only the default the node is
/// built with before the scanner table tells it where to listen.
pub const DEFAULT_HZ: f64 = 153_350_000.0;

/// The channel a POCSAG transmitter occupies: 4.5 kHz deviation at up to 2400
/// bits per second is about 12.5 kHz by Carson, and the allocations are
/// 12.5 or 25 kHz.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// Audio rate the discriminator output is decimated to. A whole number of
/// samples per bit at every rate POCSAG uses: 75, 32 and 16.
const AUDIO_HZ: f64 = 38_400.0;

pub struct PocsagNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    demod: PocsagDemod,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    sends: Vec<Transmission>,
    accepted: u64,
}

impl Default for PocsagNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl PocsagNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            demod: PocsagDemod::new(AUDIO_HZ, PocsagConfig::default()),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            sends: Vec::new(),
            accepted: 0,
        }
    }

    /// Transmissions accepted since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for PocsagNode {
    fn name(&self) -> &str {
        "pocsag"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("pocsag reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("pocsag needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        // Built for the rate it will actually be handed rather than for a
        // nominal 38.4 kHz, since the decimation factor has to be an integer
        // and the span decides what that leaves.
        self.demod = PocsagDemod::new(audio_rate, PocsagConfig::default());

        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
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

        self.sends.clear();
        let audio = std::mem::take(&mut self.audio);
        self.demod.process(&audio, &mut self.sends);
        self.audio = audio;

        let out = o.frames_mut();
        for t in &self.sends {
            self.accepted += 1;
            out.push(t.to_bytes());
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.demod.reset();
    }
}

/// The decodes a transmission's codewords become: one per page.
///
/// A transmission carries a transmitter's whole queue, so it is several pages
/// to several pagers, and each is a row of its own. What they share is the
/// bytes they came out of, which travel with each so that a log holds the
/// evidence rather than a rendering of it.
pub fn pocsag_decoded(bytes: &[u8], center: common::Hz) -> Vec<Decoded> {
    use common::Value;
    let codewords = Transmission::codewords_from_bytes(bytes);
    pocsag::parse(&codewords)
        .into_iter()
        .map(|m| {
            let mut fields: Vec<(String, Value)> = vec![
                ("address".into(), Value::Int(i64::from(m.address))),
                ("function".into(), Value::Int(i64::from(m.function))),
            ];
            let (protocol, text) = match &m.body {
                Body::Tone => ("POCSAG-Tone", None),
                Body::Numeric(s) => ("POCSAG-Numeric", Some(s.clone())),
                Body::Alpha(s) => ("POCSAG-Alpha", Some(s.clone())),
            };
            if let Some(t) = &text {
                fields.push(("message".into(), Value::Text(t.clone())));
            }
            let detail = match &text {
                Some(t) => format!("address={} {t}", m.address),
                None => format!("address={} tone only", m.address),
            };
            let mut d = Decoded::bytes(protocol, center, 0.0, bytes.to_vec())
                .with_detail(detail)
                .with_fields(fields)
                .with_modulation("FSK")
                // Every codeword read here either verified against
                // BCH(31,21) or was corrected by it, which is a real
                // integrity check rather than a plausibility argument.
                .with_crc(Some(true));
            if let Some(t) = text {
                d = d.with_media(media::TEXT).with_text(t);
            }
            d
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_span_without_its_channel() {
        let mut n = PocsagNode::default();
        assert!(n.negotiate(&spec(2_400_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(2_400_000.0, 160_000_000.0)).is_err());
        assert!(n.negotiate(&spec(25_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(10_000.0, DEFAULT_HZ)).is_err());
    }

    /// The whole path on synthetic RF: an FM carrier keyed with POCSAG bits,
    /// into the node, out as an addressed page.
    ///
    /// The point is that the three layers agree. Each is tested alone and
    /// each could be self-consistently wrong; only running them together
    /// shows that the codewords the framer packs are the ones the message
    /// tables read, in the frame positions the address depends on.
    #[test]
    fn a_modulated_transmission_becomes_a_page() {
        let (rate, center) = (2_400_000.0, DEFAULT_HZ);
        let contents = pocsag::encode(1_234_568, 3, &Body::Alpha("MOVE TO CHANNEL 2".into()));
        let bits = dsp::pocsag::encode_bits(&contents);

        // Keyed FSK at 1200 baud: the bit rate is not announced anywhere in
        // the signal, so the node has to find it.
        let sps = (rate / 1200.0) as usize;
        let mut iq = Vec::with_capacity(bits.len() * sps);
        let mut phase = 0.0f64;
        for &b in &bits {
            let f = if b { -DEVIATION_HZ } else { DEVIATION_HZ };
            for _ in 0..sps {
                phase += std::f64::consts::TAU * f / rate;
                iq.push(common::C32::new(phase.cos() as f32, phase.sin() as f32));
            }
        }

        let mut node = PocsagNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames: Vec<Vec<u8>> = Vec::new();
        // A pager transmission has no closing flag: it ends when the signal
        // does, or when the batch that should have followed is not there. So
        // the trailing silence is not decoration, it is the thing that closes
        // the transmission, and it has to be long enough to be heard as
        // silence through the channel filter.
        let quiet = vec![common::C32::new(0.0, 0.0); 400_000];
        for block in [&quiet[..], &iq[..], &quiet[..]] {
            let input = Payload::Iq(block.to_vec());
            let mut out = Payload::Frames(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Frames(f) = out {
                frames.extend(f);
            }
        }

        assert_eq!(frames.len(), 1, "expected one transmission off the air");
        let decodes = pocsag_decoded(&frames[0], Hz(center as u64));
        assert_eq!(decodes.len(), 1);
        assert_eq!(decodes[0].protocol, "POCSAG-Alpha");
        assert_eq!(decodes[0].text.as_deref(), Some("MOVE TO CHANNEL 2"));
        assert_eq!(decodes[0].media_type, media::TEXT);
        assert_eq!(decodes[0].crc_ok, Some(true));
        let get = |k: &str| decodes[0].fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("address"), Some(common::Value::Int(1_234_568)));
    }

    /// A transmitter empties its queue in one go, so one transmission is
    /// several pages to several pagers and each is a row of its own.
    #[test]
    fn one_transmission_becomes_a_row_for_each_page() {
        let mut contents = pocsag::encode(1_000_001, 3, &Body::Alpha("FIRST".into()));
        contents.extend(pocsag::encode(2_000_002, 0, &Body::Numeric("112".into())));
        let words: Vec<u32> = contents.into_iter().map(dsp::pocsag::encode_codeword).collect();
        let t = Transmission { codewords: words, baud: 1200, corrected: 0, lost: 0 };

        let decodes = pocsag_decoded(&t.to_bytes(), Hz(DEFAULT_HZ as u64));
        assert_eq!(decodes.len(), 2);
        assert_eq!(decodes[0].protocol, "POCSAG-Alpha");
        assert_eq!(decodes[1].protocol, "POCSAG-Numeric");
        assert_eq!(decodes[1].text.as_deref(), Some("112"));
    }
}
