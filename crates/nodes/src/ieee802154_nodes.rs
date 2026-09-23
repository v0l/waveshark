//! IEEE 802.15.4 at 2450 MHz as a graph node.
//!
//! Wiring only, like `ble_nodes`: the spreading, the synchronisation and the
//! frame check are `dsp::oqpsk`, the header walk is `decode::ieee802154`, and
//! neither knows about pipelines or about the other.
//!
//! The link under Zigbee, Thread and Matter, which is why nothing here is
//! named for any of them. What the decoder gives is the MAC: who addressed
//! whom, in which personal area network, and how often. Above that the three
//! of them encrypt, so a payload is a length and not a sentence.
//!
//! # Sixteen channels and a span that holds three
//!
//! The channels are five megahertz apart across 80 MHz, and no tuner here
//! samples that wide: 20 MS/s covers three of them. So the node reads
//! whichever channels the span reaches, and reports a frame at its own
//! channel's centre rather than at the tuner's. Where a span covers more than
//! one the port carries the band, because a frame cannot then be placed by
//! the port alone.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape, Stickiness};
use common::Result;
pub use decode::ieee802154::CHANNEL_WIDTH_HZ;
pub use decode::ieee802154::channel_of;
pub use decode::ieee802154::read;
use dsp::oqpsk::{
    OQPSK_2450, OqpskConfig, OqpskDetector, OqpskFrame, channel_2450_hz, channels_2450,
};
use identify::Signal;
pub use identify::ieee802154::Ieee802154;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// Where a receiver tunes when it cannot say which channel a frame came from:
/// the middle of the band the sixteen channels are spread across.
pub const BAND_CENTER_HZ: f64 = 2_442_500_000.0;

pub struct Ieee802154Node {
    rate: f64,
    cfg: OqpskConfig,
    det: OqpskDetector,
    meter: crate::FrameMeter,
    frames: Vec<OqpskFrame>,
    accepted: u64,
}

impl Default for Ieee802154Node {
    fn default() -> Self {
        Self::new(OqpskConfig::default())
    }
}

impl Ieee802154Node {
    pub fn new(cfg: OqpskConfig) -> Self {
        let hz = channel_2450_hz(11).unwrap();
        Self {
            rate: 8_000_000.0,
            cfg,
            // Replaced at negotiation, when the real rate and centre are known.
            det: OqpskDetector::new(8_000_000.0, hz, OQPSK_2450, &channels_2450(), cfg),
            // Five milliseconds at 8 MS/s: the longest PPDU is 4.3 ms, so a
            // frame's own samples are in there without a ring the size of the
            // span.
            meter: crate::FrameMeter::new(8_000_000.0, hz as u64, 0.005)
                .keyed_as(common::Modulation::Oqpsk),
            frames: Vec::new(),
            accepted: 0,
        }
    }

    /// Frames that passed their check since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for Ieee802154Node {
    fn name(&self) -> &str {
        "ieee802154"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("802.15.4 reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        // Two megasamples is the modulation's own width; the chip boundaries
        // are found in the samples, which takes a few of them per chip.
        if rate < 6_000_000.0 {
            return Err(common::Error::other("802.15.4 needs at least 6 MS/s"));
        }
        let det = OqpskDetector::new(rate, center, OQPSK_2450, &channels_2450(), self.cfg);
        let covered = det.channels();
        let hz = match covered.as_slice() {
            [] => {
                return Err(common::Error::other(
                    "802.15.4 needs a channel of the 2450 MHz plan (11 to 26) inside the span",
                ));
            }
            [one] => channel_2450_hz(*one).unwrap(),
            _ => BAND_CENTER_HZ,
        };
        self.det = det;
        self.meter =
            crate::FrameMeter::new(rate, hz as u64, 0.005).keyed_as(common::Modulation::Oqpsk);
        self.rate = rate;
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.meter.feed(iq);
        self.frames.clear();
        self.det.process(iq, &mut self.frames);
        let out = o.packets_mut();
        for f in &self.frames {
            self.accepted += 1;
            // The detector measured this burst against the floor either side
            // of it; what it does not keep is the samples, and the channel a
            // span holding several cannot get from the port.
            let hz = channel_2450_hz(f.channel).unwrap_or(BAND_CENTER_HZ) as u64;
            let mut pkt =
                crate::measured(hz, CHANNEL_WIDTH_HZ as u32, f.psdu.clone(), f.rssi_dbfs, f.snr_db)
                    .keyed(common::packet::Keying::configured(common::Modulation::Oqpsk));
            // The header, the payload and both check bytes at 250 kbit/s,
            // with the synchronisation header and room either side for the
            // ramp.
            let len = ((f.psdu.len() + 14) * 8) as f64 * 4e-6 * self.rate;
            pkt.carrier.iq = self.meter.iq_at(f.start_sample, len as usize);
            out.push(pkt);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        self.det.reset();
    }
}

impl Protocol for Ieee802154 {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::InBursts
    }

    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
    }
    fn aliases(&self) -> &'static [&'static str] {
        Signal::aliases(self)
    }
    fn placement(&self) -> Placement {
        Signal::placement(self)
    }
    fn shape(&self) -> Shape {
        Signal::shape(self)
    }
    fn default_hz(&self) -> f64 {
        Signal::default_hz(self)
    }

    /// The three things carried on it, because that is what a person types.

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: CHANNEL_WIDTH_HZ as u64 }
    }
    /// A channel of the 2450 MHz plan is a frequency nothing else here
    /// transmits a frame from. Channel 26 is 2480 MHz, where nothing else
    /// sits either; the Bluetooth advertising channel of that name is at
    /// 2480 MHz too, so the claim is narrower than its own channel.
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        channel_of(p.center_hz() as f64)?;
        Some(read(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    /// It cuts its own channels out of the span, for the reason `ble` does:
    /// a bank channel is [`dsp::source::BANK_CHANNEL_HZ`] wide at twice that
    /// rate, and this needs two megahertz at six.

    /// Nothing kept, as the other 2.4 GHz span-wide fronts keep nothing.
    ///
    /// A latched span-wide front end closes its band to the detector from the
    /// moment the span reaches it, and a 2450 plan channel is two megahertz
    /// of air every other transmitter in the band also uses. Measured on the
    /// 61.44 MS/s capture of a busy 2.4 GHz band, latching channel 16 costs
    /// the ExpressLRS handset hopping through it four of its 107 frames, for
    /// a channel this front end read nothing on. Wi-Fi declines the same
    /// trade for the same reason.
    fn stickiness(&self) -> Stickiness {
        Stickiness::Forget
    }
    /// Channel 11, which is where a Zigbee coordinator starts unless it was
    /// told otherwise.

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.0} 802.15.4", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        let label = match channel_of(hz) {
            Some(ch) => format!("154 ch{ch}"),
            None => "802.15.4".into(),
        };
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label }]
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("ieee802154")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "ieee802154",
    summary: "One 802.15.4 channel at 2450 MHz: O-QPSK, the spreading code and the frame check",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(Ieee802154Node::default()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The channel a layer states it was working.
    fn channel(d: &common::packet::Proto) -> Option<common::packet::Channel> {
        d.facts.iter().find_map(|f| match f {
            common::packet::Fact::Channel(c) => Some(c.clone()),
            _ => None,
        })
    }

    use common::Hz;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_span_with_no_channel_in_it() {
        let mut n = Ieee802154Node::default();
        assert!(n.negotiate(&spec(8_000_000.0, 2_426_000_000.0)).is_ok());
        // In the ISM band, but below channel 11 and so on nothing: the
        // channels are five megahertz apart, so a span this wide anywhere
        // between 11 and 26 covers one.
        assert!(n.negotiate(&spec(8_000_000.0, 2_399_000_000.0)).is_err());
        // Wide enough, wrong band.
        assert!(n.negotiate(&spec(8_000_000.0, 868_000_000.0)).is_err());
        // On channel 15, too slow to find the chip boundaries.
        assert!(n.negotiate(&spec(4_000_000.0, 2_425_000_000.0)).is_err());
    }

    /// Frames are tagged with the channel they arrived on, not with the
    /// tuner's centre: 2426 is where this receiver was parked and no
    /// 802.15.4 frame was ever sent there.
    #[test]
    fn frames_are_tagged_with_the_channel_they_arrived_on() {
        let mut n = Ieee802154Node::default();
        let out = n.negotiate(&spec(8_000_000.0, 2_426_000_000.0)).unwrap();
        assert_eq!(out.kind, PortKind::Packets);
        assert_eq!(out.center, Hz(2_425_000_000));
        assert_eq!(channel_of(out.center.as_f64()), Some(15));
        // A span reaching several cannot place a frame by its port, so it
        // carries the band instead.
        let wide = n.negotiate(&spec(20_000_000.0, 2_425_000_000.0)).unwrap();
        assert_eq!(wide.center, Hz(BAND_CENTER_HZ as u64));
    }

    /// A data frame becomes a row naming both ends and the network they are
    /// in, on the channel it was heard on.
    #[test]
    fn a_frame_becomes_a_row_naming_both_ends() {
        let mpdu = [0x61, 0x88, 0x2b, 0x34, 0x12, 0x01, 0x00, 0x00, 0x00, 0xaa, 0xbb];
        let d = read(&mpdu, Hz(2_425_000_000)).expect("a decode");
        assert_eq!((d.id, d.kind), ("ieee802154", "data"));
        // The network is part of who the source is, since a short address
        // means nothing outside its own PAN.
        assert_eq!(d.parties(), (Some("0x1234/0x0000"), Some("0x0001")));
        let ch = channel(&d).expect("a channel");
        assert_eq!(ch.plan, common::ChannelPlan::Ieee802154);
        assert_eq!(ch.heard, 15);
        // Nothing said: a clear MAC header is not a promise about the
        // Zigbee payload above it.
        assert_eq!(ch.secrecy, common::Secrecy::Unsaid);
    }

    /// A secured frame says what protects it, which is the statement the
    /// channel view reads. A level below four authenticates the payload and
    /// leaves it readable, so only a level above says the traffic is shut.
    #[test]
    fn a_secured_frame_names_what_protects_it() {
        let mut mpdu = vec![0x69, 0x88, 0x2b, 0x34, 0x12, 0x01, 0x00, 0x00, 0x00];
        mpdu.extend_from_slice(&[0x0d, 0x01, 0x00, 0x00, 0x00, 0x01]);
        mpdu.extend_from_slice(&[0xaa, 0xbb, 0xcc, 0xde, 0xad, 0xbe, 0xef]);
        let d = read(&mpdu, Hz(2_425_000_000)).expect("a decode");
        let ch = channel(&d).expect("a channel");
        assert_eq!(ch.secrecy, common::Secrecy::Encrypted(Some("802.15.4 MAC".into())));

        mpdu[9] = 0x09;
        let d = read(&mpdu, Hz(2_425_000_000)).expect("a decode");
        assert_eq!(channel(&d).expect("a channel").secrecy, common::Secrecy::Unsaid);
    }

    /// An extended address is the device's EUI-64, so the row carries the
    /// manufacturer it was assigned to.
    #[test]
    fn an_extended_source_carries_its_manufacturer() {
        let mut mpdu = vec![0x41, 0xc8, 0x07, 0x34, 0x12, 0x01, 0x00];
        mpdu.extend_from_slice(&[0x44, 0x33, 0x22, 0x11, 0x00, 0x4b, 0x12, 0x00]);
        let d = read(&mpdu, Hz(2_405_000_000)).expect("a decode");
        let who = d.subject.expect("an identity");
        assert_eq!(who.id.to_string(), "00:12:4B:00:11:22:33:44");
        assert_eq!(who.vendor.as_deref(), Some("00124B"));
    }

    /// Bytes that are not a MAC frame produce no row, so the bus offers them
    /// to whatever else claims the frequency.
    #[test]
    fn bytes_that_are_not_a_frame_produce_no_row() {
        assert!(read(&[0x61], Hz(2_405_000_000)).is_none());
    }

    /// The whole path: a beacon request keyed on channel 15, through the
    /// node, onto the bus and back out as a row. The registry is asked which
    /// protocol the frame belongs to rather than this test knowing, because
    /// that walk is where a frame gets lost.
    #[test]
    fn a_keyed_frame_comes_back_off_the_bus_as_a_row() {
        let rate = 8_000_000.0;
        let center = 2_426_000_000.0;
        let mut mpdu = vec![0x03, 0x08, 0x4f, 0xff, 0xff, 0xff, 0xff, 0x07];
        let crc = dsp::oqpsk::fcs(&mpdu);
        mpdu.push(crc as u8);
        mpdu.push((crc >> 8) as u8);
        let chips = dsp::oqpsk::encode_ppdu(&OQPSK_2450, &mpdu);

        // O-QPSK with half-sine chips: chip k is a half sine two chips long
        // starting k chips in, on the in-phase arm when k is even.
        let sps = rate / OQPSK_2450.chip_rate;
        let n_samples = ((chips.len() + 2) as f64 * sps) as usize;
        let mut iq = vec![common::C32::new(0.0, 0.0); n_samples];
        for (k, &c) in chips.iter().enumerate() {
            let sign = if c { 1.0 } else { -1.0 };
            for j in 0..(2.0 * sps) as usize {
                let at = (k as f64 * sps) as usize + j;
                if at >= n_samples {
                    break;
                }
                let v = (sign * (std::f64::consts::PI * j as f64 / (2.0 * sps)).sin()) as f32;
                if k % 2 == 0 {
                    iq[at].re += v;
                } else {
                    iq[at].im += v;
                }
            }
        }
        let mut phase = 0.0f64;
        for s in iq.iter_mut() {
            phase += std::f64::consts::TAU * (2_425_000_000.0 - center) / rate;
            *s *= common::C32::new(phase.cos() as f32, phase.sin() as f32);
        }

        let mut node = Ieee802154Node::default();
        let port = spec(rate, center);
        let out = node.negotiate(&port).unwrap();
        assert_eq!(out.center, Hz(2_425_000_000));

        let ins = [port];
        let tags = Vec::new();
        let quiet = vec![common::C32::new(0.0, 0.0); 40_000];
        let mut frames: Vec<common::packet::Packet> = Vec::new();
        for block in [&quiet[..], &iq[..], &quiet[..]] {
            let input = Payload::Iq(block.to_vec());
            let mut got = Payload::Packets(Vec::new());
            let mut events = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut got, &mut ctx).unwrap();
            if let Payload::Packets(f) = got {
                frames.extend(f);
            }
        }
        assert_eq!(frames.len(), 1, "expected one frame off the air");
        assert_eq!(frames[0].bytes(), &mpdu[..mpdu.len() - 2]);
        assert_eq!(frames[0].carrier.center_hz, 2_425_000_000);
        assert!(frames[0].carrier.rssi_dbfs.is_finite() && frames[0].carrier.snr_db.is_finite());
        assert_eq!(node.accepted(), 1);

        // And the registry's walk gives it to this protocol rather than to
        // whatever else claims the frequency.
        let rows = crate::protocol::frame_readers()
            .iter()
            .find_map(|p| p.stated(&frames[0]))
            .expect("a protocol claimed it");
        assert_eq!(rows.len(), 1);
        assert_eq!((rows[0].id, rows[0].kind), ("ieee802154", "command"));
        assert_eq!(channel(&rows[0]).map(|c| c.heard), Some(15));
    }
}
