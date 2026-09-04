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
use decode::lorawan;
use decode::meshtastic;
use dsp::lora::{Demod, OVERSAMPLE};
use dsp::FirDecim;
use pipeline::event::{Decoded, Event};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// The bandwidths LoRa is used at in practice. The standard defines nine,
/// down to 7.8 kHz, but a receiver that offers all of them has to guess
/// between neighbours a few kilohertz apart on a measurement worth rather
/// less than that. 62.5 kHz is MeshCore's European preset (869.618 MHz,
/// SF8), which arrived at 61 dB and was refused as no channel at all.
pub const BANDWIDTHS_HZ: [f64; 4] = [62_500.0, 125_000.0, 250_000.0, 500_000.0];

/// Widest channel, which is what decides whether a source is one at all.
pub const CHANNEL_WIDTH_HZ: f64 = 500_000.0;

/// A source narrower than this fraction of a bandwidth is not that channel.
/// LoRa fills its channel by construction, so a signal well inside one is
/// something else sitting in the same place. Seven tenths, so the neighbours
/// an octave apart do not overlap: the widest a 62.5 kHz channel may
/// measure (1.4 of it) is the narrowest a 125 kHz one may.
const FILL: f64 = 0.7;

/// Sync words of the networks whose frames are believed without a payload
/// CRC: LoRaWAN public, private (MeshCore among them) and Meshtastic.
const KNOWN_SYNC: [u8; 3] = [0x34, 0x12, lora::MESHTASTIC_SYNC];

/// Power arriving over power in the channel past which a packet read is
/// taken to be the alias of a wider channel's. A packet that fills its
/// channel reads near one; one that fills twice the width reads two.
const OUTSIDE_RATIO: f32 = 1.5;

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
    /// Input samples the decimator produces per output sample wanted, so the
    /// stream reaching the demodulator is exactly [`OVERSAMPLE`] per chip. The
    /// decimator can only divide by a whole number, and a common SDR rate
    /// (2.048 MS/s) does not divide to 500 kS/s, so what it leaves (512 kS/s)
    /// is resampled the last 2.4% here. A whole-number-only chain drifts the
    /// symbol boundary across an SF11 packet and the header checksum fails,
    /// which read as "no LoRa here" on air even though the preamble locked.
    resample_step: f64,
    /// Fractional read position into `pending`, and the tail carried between
    /// blocks.
    resample_pos: f64,
    pending: Vec<C32>,
    demods: Vec<Demod>,
    /// How far into `held` each demodulator has already found nothing.
    ///
    /// Without it every block scanned the whole of `held` again for every
    /// spreading factor, so a source open for a second cost a second of
    /// dechirping per block, six times over, and a strong signal's image
    /// held open for that long took the whole receiver under real time.
    scanned: Vec<usize>,
    /// Samples `held` must reach before a packet found still in progress
    /// is read again. Every block otherwise re-read the whole of it, from
    /// its preamble to the buffer's end, and a 640 ms packet cost the last
    /// of its blocks tens of milliseconds each. Eight symbols later is soon
    /// enough to notice that it ended.
    retry_at: usize,
    /// Samples at [`OVERSAMPLE`] per chip, waiting to be read.
    held: Vec<C32>,
    /// Most samples held before the oldest are dropped.
    hold: usize,
    decoded: u64,
    center_hz: f64,
    /// Power of the stream arriving and of the channel cut out of it, each
    /// smoothed over the last few blocks. Their ratio says whether what is
    /// transmitting fits the channel: a chirp twice the channel's width
    /// dechirps, at two spreading factors up, as a packet in the narrower
    /// one, with half its energy left outside. That packet is an alias of
    /// the wider channel's and is refused here by the energy that is not in
    /// the channel; the wider channel's own demodulator reads the real one.
    in_pow: f32,
    chan_pow: f32,
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
            resample_step: 1.0,
            resample_pos: 0.0,
            pending: Vec::new(),
            demods: Vec::new(),
            scanned: Vec::new(),
            retry_at: 0,
            held: Vec::new(),
            hold: 0,
            decoded: 0,
            center_hz: 0.0,
            in_pow: 0.0,
            chan_pow: 0.0,
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

/// Every standard bandwidth a measured width could be, nearest first.
///
/// A width is a measurement of a strong signal's skirts as much as of its
/// channel: a 62.5 kHz MeshCore packet at 59 dB measured 110 kHz and read
/// as the 125 kHz channel, and dechirped nothing. Where the measurement
/// sits between two channels both are offered, and whichever reads the
/// packet is the one kept.
pub fn bandwidths_for(width_hz: f64) -> Vec<f64> {
    let mut out: Vec<f64> = BANDWIDTHS_HZ
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

        // Two samples a chip, exactly. The decimator divides by a whole
        // number and lands near the target; the last few percent is a
        // fractional resample here. It is not optional: the demodulator's
        // symbol length is a whole number of samples at OVERSAMPLE per chip,
        // and a rate 2.4% off (512 kS/s from a 2.048 MS/s SDR against the
        // 500 kS/s a 250 kHz channel wants) drifts the symbol boundary far
        // enough across an SF11 packet that the header checksum fails.
        let want = bw * OVERSAMPLE as f64;
        // The stream has to carry two samples a chip before any resample: a
        // resample can retime samples that exist, not invent ones a rate
        // below the target never had.
        if rate < want {
            return Err(common::Error::other(format!(
                "lora needs {want:.0} S/s for a {bw:.0} Hz channel and this \
                 stream is only {rate:.0}"
            )));
        }
        let factor = (rate / want).round().max(1.0) as usize;
        let got = rate / factor as f64;
        if got < want {
            // The decimator must not land below the wanted rate, or the
            // resample would have to invent samples: pick the factor that
            // leaves it at or above `want`.
            let factor = factor.saturating_sub(1).max(1);
            self.resample_step = (rate / factor as f64) / want;
            self.decim = Some(FirDecim::design_hz(rate, factor, bw / 2.0, 60.0));
        } else {
            self.resample_step = got / want;
            self.decim = Some(FirDecim::design_hz(rate, factor, bw / 2.0, 60.0));
        }
        self.resample_pos = 0.0;
        self.pending.clear();
        self.bandwidth_hz = bw;
        self.center_hz = i.spec.center.as_f64();
        self.demods = match self.sf {
            0 => dsp::lora::SCANNED_SPREADING_FACTORS
                .map(|sf| Demod::new(dsp::lora::Config::for_sf(sf)))
                .collect(),
            sf => vec![Demod::new(dsp::lora::Config::for_sf(sf))],
        };
        self.scanned = vec![0; self.demods.len()];
        self.held.clear();
        self.hold = (want * HOLD_SECONDS) as usize;

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
        // Decimate to near the target, then resample the last few percent to
        // exactly OVERSAMPLE per chip. `pending` holds the decimator output
        // with the fractional read position carried between blocks.
        let before = self.pending.len();
        decim.process(iq, &mut self.pending);
        if !iq.is_empty() && self.pending.len() > before {
            let mean = |s: &[C32]| s.iter().map(|c| c.norm_sqr()).sum::<f32>() / s.len() as f32;
            let (a, b) = (mean(iq), mean(&self.pending[before..]));
            self.in_pow += (a - self.in_pow) * 0.2;
            self.chan_pow += (b - self.chan_pow) * 0.2;
        }
        let step = self.resample_step;
        while (self.resample_pos as usize) + 1 < self.pending.len() {
            let idx = self.resample_pos as usize;
            let frac = (self.resample_pos - idx as f64) as f32;
            self.held.push(self.pending[idx] * (1.0 - frac) + self.pending[idx + 1] * frac);
            self.resample_pos += step;
        }
        // Drop consumed pending samples, keeping the one the position still
        // sits inside so the next block continues the phase.
        let consumed = self.resample_pos as usize;
        if consumed > 0 && consumed <= self.pending.len() {
            self.pending.drain(..consumed);
            self.resample_pos -= consumed as f64;
        }

        if self.held.len() < self.retry_at {
            return Ok(());
        }
        self.retry_at = 0;
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
                let Some(p) = self.demods[k].detect(&self.held, self.scanned[k]) else {
                    self.scanned[k] = self.demods[k].resume();
                    continue;
                };
                if !p.complete {
                    // The window ends inside a transmission. Keeping the
                    // samples and asking again is the whole point of holding
                    // them; decoding now would report a truncated packet as
                    // a CRC failure, which is a worse answer than silence.
                    self.retry_at = self.held.len() + 8 * self.demods[k].symbol_len();
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
                    for s in &mut self.scanned {
                        *s = s.saturating_sub(drop);
                    }
                }
                return Ok(());
            };

            // More than twice the channel's power arriving than is in the
            // channel: what was read is the middle of something wider.
            if self.in_pow > self.chan_pow * OUTSIDE_RATIO {
                let end = packet.end.min(self.held.len());
                self.held.drain(..end);
                self.scanned.fill(0);
                if self.held.len() < self.demods[0].symbol_len() * 4 {
                    return Ok(());
                }
                continue;
            }
            let ldro = dsp::lora::ldro_default(packet.sf, self.bandwidth());
            match lora::decode(&packet.symbols, packet.sf, ldro) {
                // A frame whose payload CRC did not check, or that has none
                // to check, stands on its eight-bit header checksum alone,
                // which noise passes one time in 256. The networks that send
                // without a CRC are known by their sync word; a two byte
                // frame under an unknown one, read off the intermodulation
                // product of a strong burst, is not a frame.
                Ok(frame)
                    if frame.crc_ok != Some(true)
                        && !(!frame.header.has_crc && KNOWN_SYNC.contains(&packet.sync_word)) =>
                {
                    c.emit(Event::Warning {
                        stage: "lora".into(),
                        message: format!(
                            "SF{} over {:.0} kHz: {} bytes without a CRC that checks, sync {:#04x}, refused",
                            packet.sf,
                            self.bandwidth() / 1e3,
                            frame.payload.len(),
                            packet.sync_word
                        ),
                    });
                }
                Ok(frame) => {
                    self.locked_sf = Some(packet.sf);
                    self.decoded += 1;
                    let bw = self.bandwidth();
                    // The packet's own samples, at two a chip, and the level
                    // they stood at against the channel just before the
                    // preamble. A dechirp's peak over its transform is a
                    // processing gain, not a channel SNR, so the level is
                    // measured on the samples themselves.
                    let end = packet.end.min(self.held.len());
                    let samples = self.held[packet.start..end].to_vec();
                    let power = |s: &[C32]| {
                        s.iter().map(|c| c.norm_sqr()).sum::<f32>() / s.len().max(1) as f32
                    };
                    let sig = power(&samples);
                    let sym = self.demods[0].symbol_len();
                    let before = &self.held[packet.start.saturating_sub(2 * sym)..packet.start];
                    let (rssi_dbfs, snr_db) = if before.len() >= sym / 2 && sig > 0.0 {
                        let noise = power(before).max(1e-20);
                        (10.0 * sig.log10(), 10.0 * ((sig - noise).max(noise * 0.01) / noise).log10())
                    } else {
                        (10.0 * sig.max(1e-20).log10(), f32::NAN)
                    };
                    let rate = bw * OVERSAMPLE as f64;
                    o.packets_mut().push(common::Packet {
                        at_us: now_us(),
                        center_hz: self.center_hz as u64,
                        bandwidth_hz: bw as u32,
                        rssi_dbfs,
                        snr_db,
                        modulation: Some("CSS"),
                        body: common::PacketBody::Frame(frame.to_bytes(
                            packet.sf,
                            bw,
                            packet.sync_word,
                        )),
                        iq: Some(std::sync::Arc::new(common::IqBurst {
                            rate,
                            center_hz: self.center_hz as u64,
                            samples,
                        })),
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
            self.scanned.fill(0);
            if self.held.len() < self.demods[0].symbol_len() * 4 {
                return Ok(());
            }
        }
    }

    fn reset(&mut self) {
        self.held.clear();
        self.scanned.fill(0);
        self.retry_at = 0;
        self.pending.clear();
        self.resample_pos = 0.0;
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
        if let Some(name) = m.well_known_channel() {
            fields.push(("channel".into(), Value::Text(name.into())));
        }
    }

    // What the packet says, where the public default channel key opens it.
    let body = r.meshtastic_message();
    if let Some(d) = &body {
        fields.push(("port".into(), Value::Text(d.port().into())));
        if d.data.reply_id != 0 {
            fields.push(("reply_to".into(), Value::Text(format!("{:08x}", d.data.reply_id))));
        }
        match &d.message {
            meshtastic::Message::Text(t) => {
                fields.push(("text".into(), Value::Text(t.clone())));
            }
            meshtastic::Message::Position(p) => {
                if let (Some(lat), Some(lon)) = (p.latitude, p.longitude) {
                    fields.push(("latitude".into(), Value::Float(lat)));
                    fields.push(("longitude".into(), Value::Float(lon)));
                }
                if let Some(a) = p.altitude {
                    fields.push(("altitude_m".into(), Value::Int(i64::from(a))));
                }
                if let Some(s) = p.sats_in_view {
                    fields.push(("satellites".into(), Value::Int(i64::from(s))));
                }
                // Worth showing: a low precision is the sender deliberately
                // blurring where it is, not a poor fix.
                if let Some(b) = p.precision_bits {
                    fields.push(("precision_bits".into(), Value::Int(i64::from(b))));
                }
            }
            meshtastic::Message::NodeInfo(u) => {
                fields.push(("name".into(), Value::Text(u.long_name.clone())));
                fields.push(("short_name".into(), Value::Text(u.short_name.clone())));
                if u.is_licensed {
                    fields.push(("licensed".into(), Value::Bool(true)));
                }
            }
            meshtastic::Message::Telemetry(t) => {
                if let Some(b) = t.battery_level {
                    fields.push(("battery".into(), Value::Int(i64::from(b))));
                }
                if let Some(v) = t.voltage {
                    fields.push(("voltage".into(), Value::Float(f64::from(v))));
                }
                if let Some(c) = t.channel_utilization {
                    fields.push(("channel_util".into(), Value::Float(f64::from(c))));
                }
                if let Some(c) = t.temperature {
                    fields.push(("temperature".into(), Value::Float(f64::from(c))));
                }
                if let Some(u) = t.uptime_seconds {
                    fields.push(("uptime_s".into(), Value::Int(i64::from(u))));
                }
            }
            meshtastic::Message::Opaque => {}
        }
    }

    // MeshCore keeps its routing in the clear, so the shape of the packet
    // reads whether or not its payload does; an advert is the whole node.
    let core = r.meshcore();
    if let Some(p) = &core {
        fields.push(("type".into(), Value::Text(p.payload_type.name().into())));
        fields.push(("route".into(), Value::Text(p.route.name().into())));
        fields.push(("hops".into(), Value::Int(p.hops() as i64)));
        // MeshCore has no sync word of its own, so say plainly whether
        // anything past the header agreed this is one.
        fields.push(("verified".into(), Value::Bool(p.corroborated())));
        if p.payload_type.is_encrypted() {
            fields.push(("encrypted".into(), Value::Bool(true)));
        }
        if let Some(a) = p.advert() {
            fields.push(("node".into(), Value::Text(a.node_type.name().into())));
            fields.push(("node_hash".into(), Value::Text(format!("{:02x}", a.hash()))));
            if let Some(n) = &a.name {
                fields.push(("name".into(), Value::Text(n.clone())));
            }
            if let (Some(lat), Some(lon)) = (a.latitude, a.longitude) {
                fields.push(("latitude".into(), Value::Float(lat)));
                fields.push(("longitude".into(), Value::Float(lon)));
            }
        }
        if let Some(m) = p.public_message() {
            fields.push(("channel".into(), Value::Text("Public (default key)".into())));
            // The text travels as `sender: message`, and the name in front of
            // it is part of the plaintext rather than a protocol field. A
            // group message carries no signature, so anyone holding the
            // channel key can write any name there; `text` is what was sent,
            // and `from` is only what it claims to be.
            //
            // Named `from` and not `sender` because that is what the message
            // view reads, and a field named anything else is a message with
            // nobody's name on it. See `DecodeRecord::to_message`.
            let (sender, body) = m.sender_and_body();
            if let Some(s) = sender {
                fields.push(("from".into(), Value::Text(s.to_string())));
            }
            fields.push(("text".into(), Value::Text(body.to_string())));
        }
    }

    // LoRaWAN: the keys are per device and not published, so this is the
    // metadata around a payload that stays shut. A join request is the
    // exception and names the device outright.
    let wan = r.lorawan();
    if let Some(f) = &wan {
        fields.push(("type".into(), Value::Text(f.mtype.name().into())));
        match &f.body {
            lorawan::Body::Join(j) => {
                fields.push(("dev_eui".into(), Value::Text(lorawan::format_eui(j.dev_eui))));
                fields.push(("join_eui".into(), Value::Text(lorawan::format_eui(j.join_eui))));
                fields.push(("dev_nonce".into(), Value::Int(i64::from(j.dev_nonce))));
            }
            lorawan::Body::Data(d) => {
                fields.push(("dev_addr".into(), Value::Text(format!("{:08x}", d.dev_addr))));
                fields.push(("frame_counter".into(), Value::Int(i64::from(d.f_cnt))));
                if let Some(p) = d.f_port {
                    fields.push(("port".into(), Value::Int(i64::from(p))));
                }
                fields.push(("payload_len".into(), Value::Int(d.payload_len as i64)));
                if d.adr {
                    fields.push(("adr".into(), Value::Bool(true)));
                }
                if d.ack {
                    fields.push(("ack".into(), Value::Bool(true)));
                }
                if d.f_pending {
                    fields.push(("pending".into(), Value::Bool(true)));
                }
                if d.f_opts_len > 0 {
                    fields.push(("mac_bytes".into(), Value::Int(i64::from(d.f_opts_len))));
                }
                // The payload is enciphered under a session key that is not
                // public, so say so rather than leaving it to be inferred.
                if d.payload_len > 0 {
                    fields.push(("encrypted".into(), Value::Bool(true)));
                }
            }
            lorawan::Body::JoinAccept | lorawan::Body::Opaque => {}
        }
    }

    let shape = format!("SF{} BW{:.0}k {cr}", r.sf, r.bandwidth_hz / 1e3);
    let detail = match &mesh {
        Some(m) => {
            let chan = match m.well_known_channel() {
                Some(name) => format!(" on {name}"),
                None => format!(" on channel #{:02x}", m.channel_hash),
            };
            // What it says comes first past the routing, since that is what
            // a reader is looking for; the radio shape stays in front because
            // it is what tells two networks apart.
            let says = match body.as_ref().map(|d| (&d.message, d.port())) {
                Some((meshtastic::Message::Text(t), _)) => format!(", \"{t}\""),
                Some((meshtastic::Message::Position(p), _)) => {
                    match (p.latitude, p.longitude) {
                        (Some(lat), Some(lon)) => format!(", at {lat:.5}, {lon:.5}"),
                        _ => ", position".to_string(),
                    }
                }
                Some((meshtastic::Message::NodeInfo(u), _)) => {
                    format!(", is {} ({})", u.long_name, u.short_name)
                }
                Some((meshtastic::Message::Telemetry(t), _)) => match t.battery_level {
                    Some(b) => format!(", telemetry, battery {b}%"),
                    None => ", telemetry".to_string(),
                },
                Some((meshtastic::Message::Opaque, port)) => format!(", {port}"),
                None => String::new(),
            };
            format!(
                "{shape}, {:08x} to {}, {} of {} hops left{chan}{says}",
                m.source,
                if m.is_broadcast() { "everyone".into() } else { format!("{:08x}", m.destination) },
                m.hop_limit,
                m.hop_start,
            )
        }
        None => match &core {
            Some(p) => {
                let mut s = format!("{shape}, {} {}", p.payload_type.name(), p.route.name());
                if p.hops() > 0 {
                    s.push_str(&format!(", {} hops", p.hops()));
                }
                if let Some(a) = p.advert() {
                    match &a.name {
                        Some(n) => s.push_str(&format!(", \"{n}\" ({})", a.node_type.name())),
                        None => s.push_str(&format!(", {}", a.node_type.name())),
                    }
                    if let (Some(lat), Some(lon)) = (a.latitude, a.longitude) {
                        s.push_str(&format!(" at {lat:.5}, {lon:.5}"));
                    }
                } else if let Some(m) = p.public_message() {
                    let (sender, body) = m.sender_and_body();
                    match sender {
                        Some(who) => s.push_str(&format!(", {who}: \"{body}\"")),
                        None => s.push_str(&format!(", \"{body}\"")),
                    }
                } else {
                    // Nothing but the header said this was MeshCore, and a
                    // packet from another network on the same sync word can
                    // say that much. Do not let it read as a certainty.
                    s.push_str(", header only");
                }
                s
            }
            None => match &wan {
                Some(f) => {
                    let mut s = format!("{shape}, {}", f.mtype.name());
                    match &f.body {
                        lorawan::Body::Join(j) => s.push_str(&format!(
                            ", device {} joining {}",
                            lorawan::format_eui(j.dev_eui),
                            lorawan::format_eui(j.join_eui)
                        )),
                        lorawan::Body::Data(d) => {
                            s.push_str(&format!(
                                ", {:08x} frame {}",
                                d.dev_addr, d.f_cnt
                            ));
                            match d.f_port {
                                Some(0) => s.push_str(", mac commands"),
                                Some(p) => s.push_str(&format!(", port {p}")),
                                None => {}
                            }
                            if d.payload_len > 0 {
                                s.push_str(&format!(", {} bytes sealed", d.payload_len));
                            }
                        }
                        lorawan::Body::JoinAccept => s.push_str(", sealed"),
                        lorawan::Body::Opaque => {}
                    }
                    s
                }
                None => format!(
                    "{shape}, {} byte payload, sync 0x{:02x}",
                    r.payload.len(),
                    r.sync_word
                ),
            },
        },
    };

    let protocol = if mesh.is_some() {
        "Meshtastic"
    } else if core.is_some() {
        "MeshCore"
    } else if wan.is_some() {
        "LoRaWAN"
    } else {
        "LoRa"
    };

    Some(
        Decoded::bytes(
            protocol,
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
