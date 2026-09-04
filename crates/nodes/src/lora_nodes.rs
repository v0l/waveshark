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

use common::{Result, C32};
use decode::lora::{self, Received};
use dsp::lora::{Demod, OVERSAMPLE};
use dsp::FirDecim;
use pipeline::event::{Decoded, Event};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// The bandwidths LoRa is used at in practice. The standard defines nine,
/// down to 7.8 kHz, but a receiver that offers all of them has to guess
/// between neighbours a few kilohertz apart on a measurement worth rather
/// less than that.
pub const BANDWIDTHS_HZ: [f64; 3] = [125_000.0, 250_000.0, 500_000.0];

/// Widest channel, which is what decides whether a source is one at all.
pub const CHANNEL_WIDTH_HZ: f64 = 500_000.0;

/// A source narrower than this fraction of a bandwidth is not that channel.
/// LoRa fills its channel by construction, so a signal well inside one is
/// something else sitting in the same place.
const FILL: f64 = 0.55;

/// Seconds of samples held while waiting for a packet to finish.
///
/// The longest packet LoRa can send is a 255 byte payload at SF12 over 125
/// kHz, which is a little over six seconds. Holding that at two samples a
/// chip costs 12 MB per source at the widest bandwidth, which is why the cap
/// is on time rather than on symbols: at 500 kHz the same six seconds is the
/// same buffer and a far longer packet than anyone sends.
const HOLD_SECONDS: f64 = 6.5;

pub struct LoraNode {
    /// Channel bandwidth in hertz, or zero to take it from the source.
    bandwidth_hz: f64,
    /// Spreading factor to read, or zero to find it.
    sf: u8,
    /// The one that has been answering, so the search is not repeated on
    /// every window of a source that already said what it is.
    locked_sf: Option<u8>,
    decim: Option<FirDecim>,
    demods: Vec<Demod>,
    /// Samples at [`OVERSAMPLE`] per chip, waiting to be read.
    held: Vec<C32>,
    /// Most samples held before the oldest are dropped.
    hold: usize,
    decoded: u64,
    center_hz: f64,
}

impl Default for LoraNode {
    fn default() -> Self {
        Self::new(0.0)
    }
}

impl LoraNode {
    pub fn new(bandwidth_hz: f64) -> Self {
        Self {
            bandwidth_hz,
            sf: 0,
            locked_sf: None,
            decim: None,
            demods: Vec::new(),
            held: Vec::new(),
            hold: 0,
            decoded: 0,
            center_hz: 0.0,
        }
    }

    /// Frames whose header checksum passed since the node was built.
    pub fn decoded(&self) -> u64 {
        self.decoded
    }

    /// The spreading factor the node settled on, once it has.
    pub fn spreading_factor(&self) -> Option<u8> {
        self.locked_sf
    }

    fn bandwidth(&self) -> f64 {
        self.bandwidth_hz
    }
}

/// The nearest standard bandwidth to a measured width, when the width is
/// close enough to one to mean it.
pub fn bandwidth_for(width_hz: f64) -> Option<f64> {
    BANDWIDTHS_HZ
        .iter()
        .copied()
        .find(|bw| width_hz >= bw * FILL && width_hz <= bw * 1.4)
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

        // Two samples a chip, and by a whole number: a fractional resampler
        // in front of a dechirp buys nothing, because the demodulator
        // measures its own timing anyway and a rate a few percent off is a
        // sample rate offset it already tracks.
        let want = bw * OVERSAMPLE as f64;
        let factor = (rate / want).round().max(1.0) as usize;
        let got = rate / factor as f64;
        if got < want * 0.9 {
            return Err(common::Error::other(format!(
                "lora needs {want:.0} S/s for a {bw:.0} Hz channel and this \
                 stream decimates to {got:.0}"
            )));
        }
        self.bandwidth_hz = bw;
        self.center_hz = i.spec.center.as_f64();
        self.decim = Some(FirDecim::design_hz(rate, factor, bw / 2.0, 60.0));
        self.demods = match self.sf {
            0 => dsp::lora::SPREADING_FACTORS
                .map(|sf| Demod::new(dsp::lora::Config { sf, ..Default::default() }))
                .collect(),
            sf => vec![Demod::new(dsp::lora::Config { sf, ..Default::default() })],
        };
        self.held.clear();
        self.hold = (got * HOLD_SECONDS) as usize;

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
        let (Some(iq), Some(decim)) = (i.as_iq(), self.decim.as_mut()) else { return Ok(()) };
        decim.process(iq, &mut self.held);

        loop {
            // The one that has worked before is asked first, both because it
            // is usually right and because a wrong spreading factor can find
            // a preamble in another one's payload.
            let order: Vec<usize> = match self.locked_sf {
                Some(sf) => {
                    let at = self.demods.iter().position(|d| d.spreading_factor() == sf);
                    at.into_iter().chain((0..self.demods.len()).filter(|k| Some(*k) != at)).collect()
                }
                None => (0..self.demods.len()).collect(),
            };

            let mut found = None;
            for k in order {
                let Some(p) = self.demods[k].detect(&self.held, 0) else { continue };
                if !p.complete {
                    // The window ends inside a transmission. Keeping the
                    // samples and asking again is the whole point of holding
                    // them; decoding now would report a truncated packet as
                    // a CRC failure, which is a worse answer than silence.
                    return Ok(());
                }
                found = Some(p);
                break;
            }
            let Some(packet) = found else {
                // Nothing here. Keep a packet's worth in case one is
                // arriving, and drop the rest so a quiet source does not
                // grow a buffer for as long as it stays open.
                if self.held.len() > self.hold {
                    let drop = self.held.len() - self.hold;
                    self.held.drain(..drop);
                }
                return Ok(());
            };

            let ldro = dsp::lora::ldro_default(packet.sf, self.bandwidth());
            match lora::decode(&packet.symbols, packet.sf, ldro) {
                Ok(frame) => {
                    self.locked_sf = Some(packet.sf);
                    self.decoded += 1;
                    let bw = self.bandwidth();
                    o.packets_mut().push(common::Packet {
                        at_us: now_us(),
                        center_hz: self.center_hz as u64,
                        bandwidth_hz: bw as u32,
                        // A dechirp measures how far its peak stood over the
                        // rest of the transform, which is a processing gain
                        // rather than a channel SNR. The source this came
                        // from was measured, and that is the level the row
                        // gets; inventing one here would be worse.
                        rssi_dbfs: f32::NAN,
                        snr_db: f32::NAN,
                        modulation: Some("CSS"),
                        body: common::PacketBody::Frame(frame.to_bytes(
                            packet.sf,
                            bw,
                            packet.sync_word,
                        )),
                        iq: None,
                        audio: None,
                        measure: None,
                    });
                }
                Err(e) => {
                    c.emit(Event::Warning {
                        stage: "lora".into(),
                        message: format!(
                            "SF{} over {:.0} kHz: {} symbols and {e:?}",
                            packet.sf,
                            self.bandwidth() / 1e3,
                            packet.symbols.len()
                        ),
                    });
                }
            }
            let end = packet.end.min(self.held.len());
            self.held.drain(..end);
            if self.held.len() < self.demods[0].symbol_len() * 4 {
                return Ok(());
            }
        }
    }

    fn reset(&mut self) {
        self.held.clear();
        self.locked_sf = None;
        if let Some(d) = &mut self.decim {
            d.reset();
        }
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("bandwidth_hz", self.bandwidth_hz, 0.0..=500_000.0)
                .label("Channel bandwidth, 0 to measure it"),
            Param::float("sf", self.sf as f64, 0.0..=12.0)
                .label("Spreading factor, 0 to find it"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "bandwidth_hz" => self.bandwidth_hz = v.as_f64().unwrap_or(0.0).max(0.0),
            "sf" => {
                let sf = v.as_f64().unwrap_or(0.0) as u8;
                self.sf = if sf == 0 || dsp::lora::SPREADING_FACTORS.contains(&sf) { sf } else { 0 };
                self.locked_sf = None;
            }
            _ => return Err(common::Error::other(format!("lora: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

/// One frame off the bus as a row: what the radio parameters were, what the
/// header said, and whose packet it is where that can be read.
///
/// Recognised the way an M17 transmission is, by its shape rather than by
/// its frequency, because a chirp arrives wherever somebody put it: 433, 868
/// and 915 MHz are all in use and none of them is only LoRa.
pub fn lora_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let r = Received::parse(bytes)?;
    let cr = format!("4/{}", 4 + r.coding_rate);
    let mut fields: Vec<(String, Value)> = vec![
        ("spreading_factor".into(), Value::Int(i64::from(r.sf))),
        ("bandwidth_hz".into(), Value::Float(r.bandwidth_hz)),
        ("coding_rate".into(), Value::Text(cr.clone())),
        ("sync_word".into(), Value::Text(format!("0x{:02x}", r.sync_word))),
        ("payload_len".into(), Value::Int(r.payload.len() as i64)),
    ];

    let mesh = r.meshtastic();
    if let Some(m) = &mesh {
        fields.extend([
            ("source".into(), Value::Text(format!("{:08x}", m.source))),
            (
                "destination".into(),
                Value::Text(if m.is_broadcast() {
                    "broadcast".into()
                } else {
                    format!("{:08x}", m.destination)
                }),
            ),
            ("packet_id".into(), Value::Text(format!("{:08x}", m.packet_id))),
            ("hops".into(), Value::Text(format!("{}/{}", m.hop_limit, m.hop_start))),
            ("channel_hash".into(), Value::Int(i64::from(m.channel_hash))),
        ]);
    }

    let shape = format!("SF{} BW{:.0}k {cr}", r.sf, r.bandwidth_hz / 1e3);
    let detail = match &mesh {
        Some(m) => format!(
            "{shape}, {:08x} to {}, {} of {} hops left",
            m.source,
            if m.is_broadcast() { "everyone".into() } else { format!("{:08x}", m.destination) },
            m.hop_limit,
            m.hop_start,
        ),
        None => format!(
            "{shape}, {} byte payload, sync 0x{:02x}",
            r.payload.len(),
            r.sync_word
        ),
    };

    Some(
        Decoded::bytes(
            if mesh.is_some() { "Meshtastic" } else { "LoRa" },
            center,
            0.0,
            r.payload.clone(),
        )
        .with_modulation("CSS")
        .with_bandwidth(r.bandwidth_hz)
        .with_crc(r.crc_ok)
        .with_detail(detail)
        .with_fields(fields),
    )
}

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
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

    #[test]
    fn a_source_is_matched_to_the_channel_it_fills() {
        assert_eq!(bandwidth_for(250_000.0), Some(250_000.0));
        assert_eq!(bandwidth_for(210_000.0), Some(250_000.0));
        assert_eq!(bandwidth_for(120_000.0), Some(125_000.0));
        assert_eq!(bandwidth_for(480_000.0), Some(500_000.0));
        // A narrow carrier sitting in a LoRa channel is not a LoRa channel.
        assert_eq!(bandwidth_for(25_000.0), None);
        assert_eq!(bandwidth_for(800_000.0), None);
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
