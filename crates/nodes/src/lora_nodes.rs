//! LoRa as a graph node: a source's stream in, decoded frames out.
//!
//! The demodulator in `dsp::lora` reads a whole packet from a block of
//! samples, because that is what a dechirp wants: a symbol is only a tone
//! once a full symbol of it has arrived, and the preamble that fixes the
//! clock is twenty of them. A stream arrives in blocks of whatever the radio
//! felt like handing over, so this node holds a window of samples and asks
//! the demodulator about it, keeping what it has when the answer is a packet
//! the window cut in half.
//!
//! Two things it has to work out for itself. The bandwidth comes from the
//! source: LoRa occupies its whole channel and the detector measured that,
//! so the nearest of the three bandwidths anyone uses is the one to
//! resample to. The spreading factor is found by trying: dechirping at the
//! wrong one gives no peak at all, so the six of them are six cheap
//! questions, asked once and then remembered for as long as the source keeps
//! answering the same way.
//!
//! What reaches the bus is the payload as the transmitter sent it, with the
//! header's own checksum and the payload CRC both checked, behind the
//! parameters it was read at. A LoRa payload is bytes and nothing else: it
//! carries no address, no protocol and no frequency, and it can arrive on
//! any of three continents' ISM bands, so what identifies it downstream is
//! the envelope the front end writes in front of it. Behind a Meshtastic
//! sync word the first sixteen bytes of the payload are read as its packet
//! header, which is as far as anyone without the channel key gets.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::lora::KNOWN_SYNC;
pub use decode::lora::read;
use decode::lora::{self};
pub use dsp::lora::{ChirpReader, Found, HOLD_SECONDS, OUTSIDE_RATIO};
use identify::Signal;
pub use identify::lora::ALL_BANDWIDTHS_HZ;
pub use identify::lora::Lora;
pub use identify::lora::{BANDWIDTHS_2G4_HZ, BANDWIDTHS_HZ, is_2g4};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// Widest channel, which is what decides whether a source is one at all.
pub const CHANNEL_WIDTH_HZ: f64 = 812_500.0;

/// A source narrower than this fraction of a bandwidth is not that channel.
/// LoRa fills its channel by construction, so a signal well inside one is
/// something else sitting in the same place. Seven tenths, so the neighbours
/// an octave apart do not overlap: the widest a 62.5 kHz channel may
/// measure (1.4 of it) is the narrowest a 125 kHz one may.
const FILL: f64 = 0.7;

pub struct LoraNode {
    /// Channel bandwidth in hertz, or zero to take it from the source.
    bandwidth_hz: f64,
    /// Spreading factor to read, or zero to find it.
    sf: u8,
    reader: ChirpReader,
    decoded: u64,
}

impl Default for LoraNode {
    fn default() -> Self {
        Self::new(0.0)
    }
}

impl LoraNode {
    pub fn new(bandwidth_hz: f64) -> Self {
        Self { bandwidth_hz, sf: 0, reader: ChirpReader::new(), decoded: 0 }
    }

    /// Frames whose header checksum passed since the node was built.
    pub fn read(&self) -> u64 {
        self.decoded
    }

    /// The spreading factor the node settled on, once it has.
    pub fn spreading_factor(&self) -> Option<u8> {
        self.reader.locked_sf()
    }

    fn bandwidth(&self) -> f64 {
        self.reader.bandwidth()
    }
}

/// The nearest standard bandwidth to a measured width, when the width is
/// close enough to one to mean it.
pub fn bandwidth_for(width_hz: f64) -> Option<f64> {
    BANDWIDTHS_HZ.iter().copied().find(|bw| width_hz >= bw * FILL && width_hz <= bw * 1.4)
}

/// Every standard bandwidth a measured width could be, nearest first.
///
/// A width is a measurement of a strong signal's skirts as much as of its
/// channel: a 62.5 kHz MeshCore packet at 59 dB measured 110 kHz and read
/// as the 125 kHz channel, and dechirped nothing. Where the measurement
/// sits between two channels both are offered, and whichever reads the
/// packet is the one kept.
pub fn bandwidths_for(width_hz: f64) -> Vec<f64> {
    fit(&BANDWIDTHS_HZ, width_hz)
}

/// The same, for a source at `center_hz`: the sub-gigahertz bandwidths or
/// the one 2.4 GHz uses.
pub fn bandwidths_for_at(center_hz: f64, width_hz: f64) -> Vec<f64> {
    if is_2g4(center_hz) {
        fit(&BANDWIDTHS_2G4_HZ, width_hz)
    } else {
        fit(&BANDWIDTHS_HZ, width_hz)
    }
}

fn fit(bandwidths: &[f64], width_hz: f64) -> Vec<f64> {
    let mut out: Vec<f64> = bandwidths
        .iter()
        .copied()
        .filter(|bw| width_hz >= bw * FILL && width_hz <= bw * 2.0)
        .collect();
    out.sort_by(|a, b| (a - width_hz).abs().total_cmp(&(b - width_hz).abs()));
    out
}

impl Simple for LoraNode {
    fn name(&self) -> &str {
        "lora"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("lora reads complex baseband"));
        }
        let rate = i.spec.rate;
        let bw = if self.bandwidth_hz > 0.0 {
            self.bandwidth_hz
        } else {
            let width = if i.spec.bandwidth > 0.0 { i.spec.bandwidth } else { rate };
            bandwidth_for(width).ok_or_else(|| {
                common::Error::other(format!(
                    "lora: {width:.0} Hz is not one of its channels, which fill \
                     125, 250 or 500 kHz"
                ))
            })?
        };
        // Every LoRa transmitter on 2.4 GHz is an SX128x, and that family
        // sends with I and Q swapped against the SX127x convention, so a
        // demodulator built for 868 MHz finds nothing there at all. The band
        // decides it, because nothing else on 2.4 GHz chirps this way and a
        // node that tried both ways up would cost twice as much on every
        // source to read a transmitter that does not exist.
        let center_hz = i.spec.center.as_f64();
        let inverted = is_2g4(center_hz);
        let sfs: Vec<u8> = match (self.sf, inverted) {
            // The SX128x rates use SF5 to SF8, and scanning 9 to 12 there
            // would be scanning for something no chip in the band sends.
            (0, true) => (5..=8u8).collect(),
            (0, false) => dsp::lora::SCANNED_SPREADING_FACTORS.collect(),
            (sf, _) => vec![sf],
        };
        self.reader.design(rate, center_hz, bw, sfs, inverted)?;

        // Packets rather than frames, for the same reason M17 sends
        // packets: what the front end knows about a transmission is more
        // than its bytes, and a packet has somewhere to put the width it was
        // heard through and how strong it was.
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.bandwidth = bw;
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.reader.feed(iq);
        while let Some(Found { packet, samples, rssi_dbfs, snr_db }) = self.reader.next() {
            let bw = self.bandwidth();
            let ldro = dsp::lora::ldro_default(packet.sf, bw);
            match lora::decode(&packet.symbols, packet.sf, ldro) {
                // A frame whose payload CRC did not check, or that has none
                // to check, stands on its eight-bit header checksum alone,
                // which noise passes one time in 256. The networks that send
                // without a CRC are known by their sync word; a two byte
                // frame under an unknown one, read off the intermodulation
                // product of a strong burst, is not a frame.
                Ok(frame)
                    if frame.crc_ok != Some(true)
                        && (frame.header.has_crc || !KNOWN_SYNC.contains(&packet.sync_word)) =>
                {
                    c.warn(format!(
                        "SF{} over {:.0} kHz: {} bytes without a CRC that checks, \
                         sync {:#04x}, refused",
                        packet.sf,
                        bw / 1e3,
                        frame.payload.len(),
                        packet.sync_word
                    ));
                }
                Ok(frame) => {
                    self.decoded += 1;
                    let mut f = crate::measured(
                        self.reader.center_hz() as u64,
                        bw as u32,
                        frame.to_bytes(packet.sf, bw, packet.sync_word),
                        rssi_dbfs,
                        snr_db,
                    );
                    f.carrier.iq = Some(std::sync::Arc::new(common::IqBurst {
                        rate: self.reader.sample_rate(),
                        center_hz: self.reader.center_hz() as u64,
                        samples,
                    }));
                    // How it was keyed, which for LoRa is the spreading
                    // factor and the width the chirp sweeps: a row cannot
                    // say which channel plan a mesh is on without them.
                    let keying = common::packet::Keying::configured(common::Modulation::Chirp).of(
                        common::packet::KeyingParams {
                            bandwidth_hz: bw as f32,
                            spreading: Some(packet.sf),
                            ..Default::default()
                        },
                    );
                    // The payload CRC the header asked for, checked by the
                    // reader before it called this a frame.
                    o.packets_mut()
                        .push(f.keyed(keying).checked(common::packet::Integrity::Passed));
                }
                Err(e) => {
                    c.warn(format!(
                        "SF{} over {:.0} kHz: {} symbols and {e:?}",
                        packet.sf,
                        bw / 1e3,
                        packet.symbols.len()
                    ));
                }
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.reader.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float(BANDWIDTH_HZ, self.bandwidth_hz, 0.0..=500_000.0)
                .label("Channel bandwidth, 0 to measure it"),
            Param::float(SF, self.sf as f64, 0.0..=12.0).label("Spreading factor, 0 to find it"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            BANDWIDTH_HZ => self.bandwidth_hz = v.as_f64().unwrap_or(0.0).max(0.0),
            SF => {
                let sf = v.as_f64().unwrap_or(0.0) as u8;
                self.sf =
                    if sf == 0 || dsp::lora::SPREADING_FACTORS.contains(&sf) { sf } else { 0 };
                self.reader.unlock();
            }
            _ => return Err(common::Error::other(format!("lora: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

impl Protocol for Lora {
    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
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

    /// The same chirp is legal at 433, 868 and 915 MHz and none of those
    /// bands is only LoRa, so the claim is the front end's tag plus a
    /// spreading factor, a bandwidth and a coding rate that LoRa defines.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        read(bytes).map(|d| vec![d])
    }

    /// The Meshtastic EU_868 slot, which is where a LoRa packet heard in
    /// Europe most often is.

    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Packets]
    }
    fn accepts_width(&self, hz: f64, source_width_hz: f64) -> bool {
        !bandwidths_for_at(hz, source_width_hz).is_empty()
    }
    fn widths_for(&self, hz: f64, source_width_hz: f64) -> Vec<f64> {
        bandwidths_for_at(hz, source_width_hz)
    }
    /// Two channels an octave apart share a sweep rate two spreading
    /// factors apart, so a 250 kHz packet also reads, as something, through
    /// a 125 kHz demodulator seeing half of it. The narrower one cannot
    /// read a chirp of the wider, so when both read the wider is the
    /// channel.
    fn resolve_widths(&self, heard: &mut Vec<f64>) {
        let widest = heard.iter().copied().fold(0.0, f64::max);
        heard.retain(|w| *w >= widest);
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(BANDWIDTH_HZ, at.width_hz)]
    }
}

/// The setting names this stage reads.
const BANDWIDTH_HZ: &str = "bandwidth_hz";
const SF: &str = "sf";

pub const DESC: StageDesc = StageDesc {
    name: "lora",
    summary: "LoRa chirp spread spectrum: dechirp, then the frame \
              behind it, at any spreading factor over 125 to 500 kHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let mut n = LoraNode::new(s.f64_or(BANDWIDTH_HZ, 0.0));
    // Nought is "find it", and so is anything the demodulator cannot run at.
    let sf = s.f64_or(SF, 0.0) as u8;
    if dsp::lora::SPREADING_FACTORS.contains(&sf) {
        Simple::set_param(&mut n, SF, ParamValue::Float(sf as f64))?;
    }
    Ok(Box::new(n))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    fn spec(rate: f64, bandwidth: f64) -> PortSpec {
        let mut s = StreamSpec::iq(rate, Hz(869_525_000));
        s.bandwidth = bandwidth;
        PortSpec { spec: s, latency: 0 }
    }

    /// A source on 2.4 GHz is an SX128x, so the node reads it the other way
    /// up and looks only at the spreading factors that family uses. Getting
    /// this wrong is not a degradation: an ExpressLRS handset sending a
    /// hundred packets a second reads as an empty band.
    #[test]
    fn a_source_on_2g4_is_read_the_way_an_sx128x_transmits() {
        assert!(is_2g4(2_440_400_000.0));
        assert!(!is_2g4(869_525_000.0));

        let mut n = LoraNode::new(812_500.0);
        let mut s = StreamSpec::iq(4_000_000.0, Hz(2_440_400_000));
        s.bandwidth = 812_500.0;
        n.negotiate(&PortSpec { spec: s, latency: 0 }).expect("a 2.4 GHz source");
        assert_eq!(n.reader.demods().len(), 4, "SF5 to SF8, which is what the band uses");
        assert!(n.reader.demods().iter().all(|d| d.inverted()));

        let mut n = LoraNode::new(250_000.0);
        n.negotiate(&spec(2_000_000.0, 250_000.0)).expect("an 868 MHz source");
        assert!(n.reader.demods().iter().all(|d| !d.inverted()));
    }

    #[test]
    fn a_source_is_matched_to_the_channel_it_fills() {
        assert_eq!(bandwidth_for(250_000.0), Some(250_000.0));
        assert_eq!(bandwidth_for(210_000.0), Some(250_000.0));
        assert_eq!(bandwidth_for(120_000.0), Some(125_000.0));
        assert_eq!(bandwidth_for(480_000.0), Some(500_000.0));
        // A narrow carrier sitting in a LoRa channel is not a LoRa channel.
        assert_eq!(bandwidth_for(25_000.0), None);
        assert_eq!(bandwidth_for(800_000.0), None);
        // A strong signal measures wide: both neighbours are offered,
        // nearest first, and the one that reads it is kept.
        assert_eq!(bandwidths_for(110_000.0), vec![125_000.0, 62_500.0]);
        assert_eq!(bandwidths_for(40_000.0), Vec::<f64>::new());
        assert_eq!(bandwidths_for(60_000.0), vec![62_500.0]);
    }

    #[test]
    fn the_node_refuses_a_stream_too_slow_for_its_channel() {
        let mut n = LoraNode::default();
        assert!(n.negotiate(&spec(2_000_000.0, 250_000.0)).is_ok());
        let mut n = LoraNode::default();
        assert!(n.negotiate(&spec(300_000.0, 250_000.0)).is_err(), "under two samples a chip");
        let mut n = LoraNode::default();
        assert!(n.negotiate(&spec(2_000_000.0, 25_000.0)).is_err(), "not a LoRa channel");
    }
}
