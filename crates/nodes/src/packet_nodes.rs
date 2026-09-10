//! Decoding, as a consumer of the packet bus.
//!
//! Every front end puts what it produced on the bus, and this reads it: a
//! burst of timings goes through the protocol tables, a frame of bytes goes
//! to whichever registered protocol claims it. Both come out as decodes,
//! which is what a packet list, a chart or an alert wants.
//!
//! It runs here rather than inside each channel's chain, where it used to,
//! because there is one of it. A decoder per channel meant the same protocol
//! tables were consulted in a hundred places, decodes reached the rest of the
//! program through whatever collected them, and a burst that arrived by some
//! other route (a log being replayed, a future front end) got no decoding at
//! all. Decoding is cheap integer work on a burst that has already been found:
//! the expensive per-sample DSP stays parallel in the banks, and this sees a
//! few packets a second.

use common::{Packet, PacketBody, Result};
use decode::Protocols;
use pipeline::event::{Decoded, Event};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

use crate::decode_nodes::{decoded_event, unmatched_event};
use pipeline::registry::{Category, Settings, StageDesc};

pub struct PacketDecodeNode {
    protocols: Protocols,
    /// Report every protocol that claims a packet, rather than only the first.
    report_all: bool,
    /// Report bursts no protocol claimed, with the coding inferred from their
    /// timings.
    report_unknown: bool,
    /// What decoded in the last block, for a host that wants this node's
    /// output rather than every event the graph produced.
    hits: Vec<Decoded>,
    /// Where each packet's decodes sit in `hits`, one span per packet of the
    /// last batch, so the conclusions can be handed back to the packets they
    /// came from.
    spans: Vec<(usize, usize)>,
}

impl Default for PacketDecodeNode {
    fn default() -> Self {
        Self::new(Protocols::all())
    }
}

impl PacketDecodeNode {
    pub fn new(protocols: Protocols) -> Self {
        Self {
            protocols,
            report_all: true,
            report_unknown: true,
            hits: Vec::new(),
            spans: Vec::new(),
        }
    }

    /// Decode a batch of packets and write the conclusions onto them, which
    /// is the whole of what this node does to a packet.
    ///
    /// Public because the bus is not the only source of packets: a directory
    /// rebuilt from the packet log has to reach the same conclusions as the
    /// receiver did when the packets were live, and two implementations of
    /// "what protocol is this" would drift apart the first time one was
    /// fixed. A replay annotates and then reads `Packet::decodes`, exactly as
    /// a view wired to the bus does.
    pub fn annotate(&mut self, packets: &mut [Packet]) {
        self.decode_all(packets);
        for (p, hits) in packets.iter_mut().zip(self.per_packet()) {
            // Replaced only where there is something to replace it with, so
            // annotating twice is still idempotent. A conclusion the
            // protocols cannot reach from the bytes is the front end's, and
            // overwriting it lost the only copy: an analogue voice channel
            // carries an empty frame with its speech beside it, so every over
            // arrived here as a packet nothing could read and left as a
            // packet saying nothing. No call row, nothing to subscribe to,
            // and a channel marked as voice that could not be heard.
            if !hits.is_empty() {
                p.decodes = hits.to_vec();
            }
        }
    }

    fn decode_all(&mut self, packets: &[Packet]) {
        self.hits.clear();
        self.spans.clear();
        for p in packets {
            let from = self.hits.len();
            match &p.body {
                PacketBody::Pulses(pkg) => self.decode_burst(p, pkg, keying_of(p)),
                PacketBody::Frame(f) => self.decode_frame(p, &f.bytes),
            }
            // Where this packet's decodes are in `hits`, so they can be put
            // back on the packet they came from without matching on anything.
            self.spans.push((from, self.hits.len()));
        }
    }

    /// What each packet of the last batch decoded to, in the same order.
    fn per_packet(&self) -> impl Iterator<Item = &[Decoded]> {
        self.spans.iter().map(|(a, b)| &self.hits[*a..*b])
    }

    fn decode_burst(&mut self, p: &Packet, pkg: &common::Package, modulation: common::Modulation) {
        decode_burst_into(
            &self.protocols,
            Options { report_all: self.report_all, report_unknown: self.report_unknown },
            p,
            pkg,
            modulation,
            &mut self.hits,
        );
    }

    fn decode_frame(&mut self, p: &Packet, bytes: &[u8]) {
        decode_frame_into(p, bytes, &mut self.hits);
    }
}

/// What to report about a burst: every protocol that claimed it or only the
/// first, and whether a burst nothing claimed is reported at all. A
/// preference rather than a fact about the packet, which is why it is the
/// node's parameters and travels with the call rather than being decided
/// below.
#[derive(Clone, Copy, Debug)]
struct Options {
    report_all: bool,
    report_unknown: bool,
}

/// Which keying a burst arrived under, which is not something the protocols
/// can tell and belongs in the packet list's own column: a device that exists
/// in both an OOK and an FSK variant decodes the same either way.
///
/// Measured where a classifier saw the burst. The fallback is the channel
/// width the packet arrived through, which is only ever a guess: the wide
/// tier carries plenty of on-off keyed sensors, and this used to label every
/// one of them FSK.
fn keying_of(p: &Packet) -> common::Modulation {
    p.modulation().unwrap_or(match p.measure.as_ref() {
        Some(m) => m.modulation,
        None if p.bandwidth_hz > 60_000 => common::Modulation::Fsk2,
        None => common::Modulation::Ook,
    })
}

fn decode_burst_into(
    protocols: &Protocols,
    opts: Options,
    p: &Packet,
    pkg: &common::Package,
    modulation: common::Modulation,
    hits: &mut Vec<Decoded>,
) {
    let center = common::Hz(p.center_hz());
    let mut matched = false;
    // A protocol that fails is not reported. A CRC failure in particular
    // is a protocol saying "those were my timings but the reception was
    // not good enough", which is worth knowing while tuning a chain and
    // is noise in a packet list.
    for (_, res) in protocols.diagnose(pkg) {
        if let Ok(report) = res {
            matched = true;
            hits.push(decoded_event(&report, pkg, center, modulation));
            if !opts.report_all {
                break;
            }
        }
    }
    if !matched && opts.report_unknown {
        // The keying column shows what the classifier measured where it
        // is more specific than the front end that read the burst: a
        // chirp or a carrier that no front end reads, or a burst it
        // could not name at all.
        let label = match p.measure.as_ref().map(|m| m.modulation) {
            Some(l) if l != common::Modulation::Unknown => l,
            _ => modulation,
        };
        hits.push(unmatched_event(pkg, center, label, p.measure.as_ref()));
    }
}

/// A frame from a demodulator that produces bytes.
///
/// Every protocol is offered it in turn, most specific claim first
/// (`protocol::frame_readers`), and the first to claim it reads it. Mode S
/// and AIS both arrive here as bytes with nothing to distinguish them, so the
/// packet's own centre frequency does it. That is not a tag somebody
/// attached: where a frame was received is evidence the packet already
/// carries, and a 162 MHz frame is not a Mode S frame no matter what its bits
/// would parse as.
///
/// Parsing again here rather than carrying the demodulator's own parse on
/// the bus is deliberate: what travels is the evidence, and every consumer
/// draws its own conclusions from it.
fn decode_frame_into(p: &Packet, bytes: &[u8], hits: &mut Vec<Decoded>) {
    for proto in crate::protocol::frame_readers() {
        if let Some(rows) = proto.read_frame(p, bytes) {
            hits.extend(rows);
            return;
        }
    }
}

impl Simple for PacketDecodeNode {
    fn name(&self) -> &str {
        PROTOCOLS.name
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Packets {
            return Err(common::Error::other("packet_decode reads the packet bus"));
        }
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        // Annotated and passed on, rather than consumed: everything that
        // wants to know what a packet was reads it from the packet, and this
        // is the one stage that decides. A view wired here sees the same
        // conclusions the log's replay would reach.
        let mut packets: Vec<Packet> = i.as_packets().unwrap_or(&[]).to_vec();
        self.annotate(&mut packets);
        for d in &self.hits {
            c.emit(Event::Decoded(d.clone()));
        }
        o.packets_mut().extend(packets);
        Ok(())
    }
    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool("report_all", self.report_all).label("Report every matching protocol"),
            Param::bool("report_unknown", self.report_unknown).label("Report unrecognised bursts"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "report_all" => self.report_all = v.as_bool().unwrap_or(true),
            "report_unknown" => self.report_unknown = v.as_bool().unwrap_or(true),
            _ => {
                return Err(common::Error::other(format!(
                    "packet_decode: unknown parameter {name:?}"
                )))
            }
        }
        Ok(())
    }
}

/// One row per burst, however many channels heard it.
///
/// Channels overlap by design: a two times oversampled channelizer hands
/// adjacent channels each other's transition band, so a transmitter sitting
/// anywhere near an edge is genuinely present in two of them, and its
/// sidebands reach further still. Each of those channels runs its own
/// detector, reads a mangled copy of the same burst, and reports it. Measured
/// on a synthetic FSK packet, the channel holding the signal read it correctly
/// as 46 bits at 110 us a symbol while its neighbour reported 139 bits at
/// 36 us: not a second device, just the same one seen through a filter skirt.
/// Running two banks over the same air makes this certain rather than likely.
///
/// A node rather than a pass the packet list makes for itself, because the
/// map, the device database and the feeds read the same bus: with the copies
/// dropped here, every consumer sees the rows the packet list shows instead
/// of each deciding for itself what a duplicate is.
#[derive(Default)]
pub struct DedupeNode {
    /// Bursts already reported, for long enough to recognise the same one
    /// arriving again from another channel.
    ///
    /// Deduping within a block is not enough. Reads from the radio are short,
    /// about seven milliseconds at 2.3 MS/s, and a burst that starts near the
    /// end of one is finished by the detectors in the next, so the copies from
    /// neighbouring channels straddle the boundary. Measured on live 868 MHz
    /// traffic, one transmission appeared as four rows 31 kHz apart.
    recent: Vec<Heard>,
}

/// A burst that has already been reported.
#[derive(Clone, Copy, Debug)]
struct Heard {
    at: std::time::Instant,
    freq: f64,
    channel_hz: f64,
    modulation: common::Modulation,
    /// Whether a protocol claimed it.
    known: bool,
}

/// One decode as the dedupe reads it: where it was heard, how wide the
/// channel it came through was, how it was keyed, how strong it was, and
/// whether a protocol claimed it.
#[derive(Clone, Copy, Debug)]
struct Seen {
    freq: f64,
    channel_hz: f64,
    modulation: common::Modulation,
    rssi_dbfs: f32,
    known: bool,
}

impl Seen {
    fn of(p: &Packet, d: &Decoded) -> Self {
        Self {
            freq: d.center.as_f64(),
            // The width the packet was heard through, as the front end that
            // produced it declared.
            channel_hz: f64::from(p.bandwidth_hz),
            modulation: d.modulation.unwrap_or(common::Modulation::Unknown),
            rssi_dbfs: p.rssi_dbfs(),
            known: d.protocol != crate::decode_nodes::UNKNOWN,
        }
    }

    fn heard_at(&self, at: std::time::Instant) -> Heard {
        Heard {
            at,
            freq: self.freq,
            channel_hz: self.channel_hz,
            modulation: self.modulation,
            known: self.known,
        }
    }
}

/// How long a burst stays in that memory.
///
/// A block is roughly a tenth of a second, and a burst that starts near the
/// end of one is finished by the detectors in the next, so its copies from
/// neighbouring channels straddle the boundary and a per-block comparison
/// misses half of them. Measured on live 868 MHz traffic, one transmission
/// appeared as four rows 31 kHz apart across two blocks.
///
/// Long enough to cover that, short enough that a device repeating its packet
/// two or three times a second still gets a row per repeat.
const DEDUPE_WINDOW: std::time::Duration = std::time::Duration::from_millis(300);

/// Whether a new report is the same burst as one already reported.
///
/// Same channel through the same front end is a second transmission, which on
/// a device that repeats its packet is exactly what should be logged. Same
/// channel through the other front end is one burst read twice. Anything else
/// near enough in frequency is one signal seen through a filter skirt: two and
/// a half channels either side, taken from the wider of the two reports
/// because that is the one whose skirts reach furthest.
fn same_burst(kept: &Heard, new: &Seen) -> bool {
    // A real decode is never a copy of a guess. The front end names what it
    // measured about every burst, including the ones it read nothing from,
    // and a measurement of noise a few kilohertz off a sensor a moment
    // before it keyed up must not stand in for the sensor's packet.
    if new.known && !kept.known {
        return false;
    }
    let d = (kept.freq - new.freq).abs();
    if d < 1.0 && (kept.channel_hz - new.channel_hz).abs() < 1.0 {
        return kept.modulation != new.modulation;
    }
    d <= 2.5 * kept.channel_hz.max(new.channel_hz)
}

/// Which of one block's decodes are the first report of their burst.
///
/// The strongest report of a burst wins, and a real decode beats an unknown
/// however loud, because a protocol that matched its own CRC is better
/// evidence than a stronger guess.
fn first_reports(block: &[Seen], at: std::time::Instant) -> Vec<bool> {
    let mut order: Vec<usize> = (0..block.len()).collect();
    order.sort_by(|&a, &b| {
        let key = |s: &Seen| (s.known, s.rssi_dbfs);
        let (ka, kb) = (key(&block[a]), key(&block[b]));
        kb.0.cmp(&ka.0).then(kb.1.total_cmp(&ka.1))
    });
    let mut keep = vec![false; block.len()];
    let mut kept: Vec<Heard> = Vec::new();
    for i in order {
        if kept.iter().any(|k| same_burst(k, &block[i])) {
            continue;
        }
        keep[i] = true;
        kept.push(block[i].heard_at(at));
    }
    keep
}

impl DedupeNode {
    /// Whether a burst is new, remembering it if so.
    fn accept(&mut self, s: &Seen, now: std::time::Instant) -> bool {
        self.recent.retain(|k| now.saturating_duration_since(k.at) < DEDUPE_WINDOW);
        if self.recent.iter().any(|k| same_burst(k, s)) {
            return false;
        }
        self.recent.push(s.heard_at(now));
        true
    }
}

impl Simple for DedupeNode {
    fn name(&self) -> &str {
        "dedupe"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Packets {
            return Err(common::Error::other("dedupe reads the packet bus"));
        }
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let packets = i.as_packets().unwrap_or(&[]);
        // Wall clock rather than the stream's: the window is about how a
        // burst falls across the blocks a radio delivers, and a replay is
        // driven at whatever speed the machine manages.
        let now = std::time::Instant::now();
        let seen: Vec<Seen> =
            packets.iter().flat_map(|p| p.decodes.iter().map(|d| Seen::of(p, d))).collect();
        let first = first_reports(&seen, now);
        let mut k = 0;
        for p in packets {
            // A packet nothing decoded, from a front end that puts what it
            // heard on the bus either way, has nothing to compare and passes
            // through untouched.
            if p.decodes.is_empty() {
                o.packets_mut().push(p.clone());
                continue;
            }
            let mut kept = p.clone();
            let from = k;
            kept.decodes = p
                .decodes
                .iter()
                .enumerate()
                .filter(|(n, _)| first[from + n] && self.accept(&seen[from + n], now))
                .map(|(_, d)| d.clone())
                .collect();
            k += p.decodes.len();
            // Every reading of this burst was a copy of one already reported,
            // so the packet itself is that copy.
            if !kept.decodes.is_empty() {
                o.packets_mut().push(kept);
            }
        }
        Ok(())
    }

    /// Every channel covers a different frequency after a retune, so nothing
    /// already reported can be the same burst as anything arriving.
    fn reset(&mut self) {
        self.recent.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decode_nodes::UNKNOWN;
    use common::{Hz, Pulse};

    fn spec() -> PortSpec {
        let mut s = StreamSpec::iq(0.0, Hz::mhz(433)).with_kind(PortKind::Packets);
        s.bandwidth = 31_250.0;
        PortSpec { spec: s, latency: 0 }
    }

    fn run(node: &mut PacketDecodeNode, packets: Vec<Packet>) -> Vec<Decoded> {
        let ins = [spec()];
        let mut events = Vec::new();
        let tags = Vec::new();
        let mut new_tags = Vec::new();
        let mut out = Payload::Packets(Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Simple::process(node, &Payload::Packets(packets), &mut out, &mut ctx).unwrap();
        out.as_packets().unwrap_or(&[]).iter().flat_map(|p| p.decodes.clone()).collect()
    }

    fn burst(center_hz: u64, bandwidth_hz: u32, pulses: Vec<Pulse>) -> Packet {
        Packet::of_pulses(
            0,
            bandwidth_hz,
            common::Package {
                pulses,
                snr_db: 22.0,
                rssi_dbfs: -20.0,
                start_sample: 0,
                center_hz,
                modulation: None,
            },
        )
    }

    #[test]
    fn a_burst_nothing_claims_is_still_reported() {
        // The whole reason to sweep a band: an unknown device is what a
        // scanner should surface, and silence looks the same as a broken
        // chain.
        let mut n = PacketDecodeNode::default();
        let pulses: Vec<Pulse> = (0..24).map(|_| Pulse { mark: 500, gap: 1500 }).collect();
        let hits = run(&mut n, vec![burst(433_920_000, 31_250, pulses)]);
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].protocol, "unknown");
        assert_eq!(hits[0].center, Hz(433_920_000));
    }

    #[test]
    fn a_frame_on_the_bus_decodes_as_an_aircraft() {
        // The other kind of packet: a demodulator that produces bytes rather
        // than timings, decoded by the same node so that one consumer sees
        // every packet the receiver heard.
        let mut n = PacketDecodeNode::default();
        let bytes: Vec<u8> = (0..14)
            .map(|i| {
                u8::from_str_radix(&"8D4840D6202CC371C32CE0576098"[i * 2..i * 2 + 2], 16).unwrap()
            })
            .collect();
        let hits = run(
            &mut n,
            vec![Packet::of_frame(
                0,
                2_000_000,
                common::Frame::measured(bytes, -18.0, 12.0).at(1_090_000_000),
            )],
        );
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].protocol, "ADSB-Identification");
        assert!(hits[0].detail.as_deref().unwrap_or_default().contains("KLM1023"));
    }

    /// The samples a front end attached to its frame stay with the decoded
    /// packet.
    ///
    /// A frame carries its own samples, because the demodulator is what knows
    /// which of them the frame was read from; a burst carries them on the
    /// packet. Every row is built from `Packet::samples`, which finds either,
    /// and annotating a packet must not lose the evidence the row is made of.
    #[test]
    fn the_samples_a_frame_carries_stay_with_the_packet() {
        let mut n = PacketDecodeNode::default();
        let bytes: Vec<u8> = (0..14)
            .map(|i| {
                u8::from_str_radix(&"8D4840D6202CC371C32CE0576098"[i * 2..i * 2 + 2], 16).unwrap()
            })
            .collect();
        let mut frame = common::Frame::measured(bytes, -18.0, 12.0).at(1_090_000_000);
        frame.iq = Some(std::sync::Arc::new(common::IqBurst {
            rate: 2_400_000.0,
            center_hz: 1_090_000_000,
            samples: vec![common::C32::new(0.5, -0.5); 32],
        }));
        let mut packets = vec![Packet::of_frame(0, 2_000_000, frame)];
        n.annotate(&mut packets);
        assert_eq!(packets[0].decodes.len(), 1, "the frame should have decoded");
        let iq = packets[0].samples().expect("the packet kept the frame's samples");
        assert_eq!(iq.samples.len(), 32);
    }

    #[test]
    fn the_channel_width_says_which_front_end_heard_it() {
        // The packet list has a column for it, and the protocols cannot say:
        // plenty of devices exist in both an OOK and an FSK variant.
        let mut n = PacketDecodeNode::default();
        let pulses: Vec<Pulse> = (0..24).map(|_| Pulse { mark: 500, gap: 1500 }).collect();
        let ook = run(&mut n, vec![burst(433_920_000, 31_250, pulses.clone())]);
        assert_eq!(ook[0].modulation, Some(common::Modulation::Ook));
        let fsk = run(&mut n, vec![burst(868_300_000, 125_000, pulses)]);
        assert_eq!(fsk[0].modulation, Some(common::Modulation::Fsk2));
    }

    /// A pager transmission arrives as bytes like a Mode S frame does, and
    /// only where it was received says which it is. It also becomes several
    /// rows rather than one, because a transmitter sends its whole queue in
    /// one go.
    #[test]
    fn a_pager_transmission_becomes_a_row_for_each_page() {
        let bytes = pocsag_frame();
        let mut n = PacketDecodeNode::default();
        let hits = run(
            &mut n,
            vec![Packet::of_frame(
                0,
                12_500,
                common::Frame::measured(bytes, -18.0, 12.0).at(439_987_500),
            )],
        );
        assert_eq!(hits.len(), 2);
        assert_eq!(hits[0].protocol, "POCSAG-Alpha");
        assert_eq!(hits[0].text.as_deref(), Some("ON CALL"));
        assert_eq!(hits[1].protocol, "POCSAG-Numeric");
    }

    /// Bytes a pager transmission travels on the bus as: two pages, as a
    /// transmitter that empties its queue sends them.
    fn pocsag_frame() -> Vec<u8> {
        use decode::pocsag::Body;
        let mut contents = decode::pocsag::encode(1_000_001, 3, &Body::Alpha("ON CALL".into()));
        contents.extend(decode::pocsag::encode(2_000_002, 0, &Body::Numeric("999".into())));
        contents.into_iter().flat_map(|c| dsp::pocsag::encode_codeword(c).to_be_bytes()).collect()
    }

    /// The most specific claim reads a frame, and anything else that could
    /// read the same bytes has a weaker one.
    ///
    /// `decode_frame_into` offers a frame to each protocol in turn and the
    /// first claim wins, so being asked in the wrong order hides a decode:
    /// a pager's codeword parser really does find pages in the bytes of an
    /// M17 packet received in the VHF paging allocation. The frequencies here
    /// are the ones where two protocols share spectrum: 144 to 146 MHz sits
    /// inside that allocation, and the 420 to 430 MHz TETRA downlinks inside
    /// the UHF one.
    #[test]
    fn the_most_specific_claim_reads_a_frame() {
        let mode_s: Vec<u8> = (0..14)
            .map(|i| {
                u8::from_str_radix(&"8D4840D6202CC371C32CE0576098"[i * 2..i * 2 + 2], 16).unwrap()
            })
            .collect();
        let m17 =
            decode::m17::Event::Packet { lsf: None, data: vec![0x05, b'h', b'i', 0] }.to_bytes();
        let tetra = decode::tetra::Event::Sync(decode::tetra::SyncPdu {
            system_code: 0,
            colour: 5,
            timeslot: 1,
            frame: 3,
            multiframe: 7,
            sharing_mode: 0,
            mcc: 206,
            mnc: 1,
            service_level: 0,
            late_entry: false,
        })
        .to_bytes();
        let cases: Vec<(&str, u64, Vec<u8>)> = vec![
            ("mode_s", 1_090_000_000, mode_s),
            ("pocsag", 439_987_500, pocsag_frame()),
            ("m17", 144_800_000, m17.clone()),
            ("m17", 439_000_000, m17),
            ("tetra", 392_000_000, tetra.clone()),
            ("tetra", 425_000_000, tetra),
        ];
        for (id, hz, bytes) in cases {
            let p = Packet::of_frame(
                0,
                12_500,
                common::Frame::measured(bytes.clone(), -18.0, 12.0).at(hz),
            );
            let winner = crate::protocol::by_id(id).expect("a registered protocol");
            let rows = winner.read_frame(&p, &bytes).expect("its own frame");
            assert!(!rows.is_empty(), "{id} read nothing at {hz} Hz");
            for other in crate::protocol::all().iter().filter(|o| o.id() != id) {
                if other.read_frame(&p, &bytes).is_some_and(|r| !r.is_empty()) {
                    assert!(
                        other.frame_claim() > winner.frame_claim(),
                        "{} claims a {id} frame at {hz} Hz on an equal or better claim",
                        other.id()
                    );
                }
            }
            let mut hits = Vec::new();
            decode_frame_into(&p, &bytes, &mut hits);
            let read: Vec<&str> = hits.iter().map(|d| d.protocol).collect();
            let want: Vec<&str> = rows.iter().map(|d| d.protocol).collect();
            assert_eq!(read, want, "what the walk read at {hz} Hz");
        }
    }

    #[test]
    fn a_frame_that_is_not_mode_s_is_dropped_rather_than_guessed_at() {
        let mut n = PacketDecodeNode::default();
        let hits = run(
            &mut n,
            vec![Packet::of_frame(
                0,
                2_000_000,
                common::Frame::measured(vec![0xff; 5], -18.0, 12.0).at(1_090_000_000),
            )],
        );
        assert!(hits.is_empty());
    }

    /// A channel the wide bank splits the span into.
    const WIDE_HZ: f64 = 125_000.0;
    /// A channel the narrow bank splits it into, where an OOK sensor is read.
    const OOK_HZ: f64 = 31_250.0;

    /// One burst as it reaches the dedupe: a packet the protocols have
    /// already annotated, at the width the front end heard it through.
    fn heard(freq: f64, protocol: &'static str, rssi: f32) -> Packet {
        let mut p = Packet::of_pulses(
            0,
            WIDE_HZ as u32,
            common::Package {
                pulses: Vec::new(),
                snr_db: 20.0,
                rssi_dbfs: rssi,
                start_sample: 0,
                center_hz: freq as u64,
                modulation: None,
            },
        );
        p.decodes = vec![Decoded::bytes(protocol, Hz(freq as u64), 0.0, vec![1, 2, 3])
            .with_modulation(common::Modulation::Fsk2)];
        p
    }

    /// What one block of packets comes out of the node as, in rows.
    fn deduped(node: &mut DedupeNode, packets: Vec<Packet>) -> Vec<Decoded> {
        let ins = [spec()];
        let mut events = Vec::new();
        let tags = Vec::new();
        let mut new_tags = Vec::new();
        let mut out = Payload::Packets(Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Simple::process(node, &Payload::Packets(packets), &mut out, &mut ctx).unwrap();
        out.as_packets().unwrap_or(&[]).iter().flat_map(|p| p.decodes.clone()).collect()
    }

    #[test]
    fn the_same_burst_seen_by_two_channels_is_reported_once() {
        // Channels overlap, so a strong transmitter is genuinely present in
        // its neighbours, where the detectors read a mangled copy of it. The
        // loudest reading wins and the skirts are dropped.
        let kept = deduped(
            &mut DedupeNode::default(),
            vec![
                heard(868_100_000.0, UNKNOWN, -54.0),
                heard(868_100_000.0 + WIDE_HZ, UNKNOWN, -38.0),
                heard(868_100_000.0 - WIDE_HZ, UNKNOWN, -61.0),
            ],
        );
        assert_eq!(kept.len(), 1, "kept {kept:#?}");
        assert_eq!(kept[0].center, Hz(868_100_000 + WIDE_HZ as u64), "the strongest wins");
    }

    #[test]
    fn a_real_decode_beats_a_louder_guess() {
        let kept = deduped(
            &mut DedupeNode::default(),
            vec![
                heard(868_100_000.0, UNKNOWN, -20.0),
                heard(868_100_000.0 + WIDE_HZ, "Fineoffset-WHx080", -44.0),
            ],
        );
        assert_eq!(kept.len(), 1);
        assert_eq!(kept[0].protocol, "Fineoffset-WHx080", "a CRC beats a stronger guess");
    }

    #[test]
    fn two_devices_far_apart_are_both_kept() {
        let kept = deduped(
            &mut DedupeNode::default(),
            vec![heard(868_100_000.0, UNKNOWN, -40.0), heard(869_000_000.0, UNKNOWN, -50.0)],
        );
        assert_eq!(kept.len(), 2);
    }

    #[test]
    fn a_device_that_repeats_its_packet_is_logged_every_time() {
        // Two bursts on one channel through one front end are two
        // transmissions, not one seen twice, and a sensor that sends its
        // reading three times should show three rows.
        let kept = deduped(
            &mut DedupeNode::default(),
            vec![heard(868_100_000.0, UNKNOWN, -40.0), heard(868_100_000.0, UNKNOWN, -41.0)],
        );
        assert_eq!(kept.len(), 2);
    }

    #[test]
    fn one_burst_read_by_both_front_ends_is_logged_once() {
        // The OOK and FSK branches see the same channel, so a burst can be
        // decoded by one and guessed at by the other. That is one packet.
        let mut ook = heard(868_100_000.0, "Fineoffset-WHx080", -44.0);
        ook.decodes[0].modulation = Some(common::Modulation::Ook);
        let kept =
            deduped(&mut DedupeNode::default(), vec![heard(868_100_000.0, UNKNOWN, -30.0), ook]);
        assert_eq!(kept.len(), 1);
        assert_eq!(kept[0].protocol, "Fineoffset-WHx080");
    }

    /// A burst nothing decoded at all still crosses the node: what a front
    /// end put on the bus is evidence whether or not a protocol claimed it.
    #[test]
    fn a_packet_with_no_decodes_passes_through() {
        let mut p = heard(868_100_000.0, UNKNOWN, -30.0);
        p.decodes.clear();
        let ins = [spec()];
        let mut events = Vec::new();
        let tags = Vec::new();
        let mut new_tags = Vec::new();
        let mut out = Payload::Packets(Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Simple::process(&mut DedupeNode::default(), &Payload::Packets(vec![p]), &mut out, &mut ctx)
            .unwrap();
        assert_eq!(out.as_packets().unwrap_or(&[]).len(), 1);
    }

    fn ook_at(freq: f64) -> Seen {
        Seen {
            freq,
            channel_hz: OOK_HZ,
            modulation: common::Modulation::Ook,
            rssi_dbfs: -30.0,
            known: false,
        }
    }

    #[test]
    fn a_burst_split_across_two_blocks_is_still_reported_once() {
        // Observed on live 868 MHz traffic: one transmission arrived as four
        // rows 31 kHz apart, because reads from the radio are milliseconds
        // long and each was deduped alone.
        let mut sc = DedupeNode::default();
        let t0 = std::time::Instant::now();
        let block = std::time::Duration::from_millis(7);
        let mut kept = 0;
        for (n, freq) in [868_362_300.0, 868_393_400.0, 868_331_100.0].iter().enumerate() {
            let at = t0 + block * n as u32;
            if sc.accept(&ook_at(*freq), at) {
                kept += 1;
            }
        }
        assert_eq!(kept, 1, "one burst logged as {kept} rows");
    }

    #[test]
    fn a_device_repeating_on_its_own_channel_is_logged_every_time() {
        // Same channel through the same front end is a second transmission,
        // not a second reading of the first, and a sensor that sends its
        // packet three times should show three rows.
        let mut sc = DedupeNode::default();
        let t0 = std::time::Instant::now();
        for n in 0..3u32 {
            let at = t0 + std::time::Duration::from_millis(60) * n;
            assert!(sc.accept(&ook_at(868_362_300.0), at), "repeat {n} was swallowed");
        }
    }

    #[test]
    fn a_neighbour_is_only_a_duplicate_while_the_burst_is_recent() {
        let mut sc = DedupeNode::default();
        let t0 = std::time::Instant::now();
        assert!(sc.accept(&ook_at(868_362_300.0), t0));

        let soon = t0 + std::time::Duration::from_millis(50);
        assert!(!sc.accept(&ook_at(868_393_400.0), soon), "a skirt slipped through");

        // Long enough later and it is a different burst that happens to be
        // next door, which is the whole reason the memory expires.
        let later = t0 + DEDUPE_WINDOW + std::time::Duration::from_millis(10);
        assert!(sc.accept(&ook_at(868_393_400.0), later), "the memory never expired");
    }

    /// A retune puts every channel on a different frequency, so what was
    /// reported before it cannot be the same burst as anything after.
    #[test]
    fn a_retune_forgets_what_was_reported() {
        let mut sc = DedupeNode::default();
        let t0 = std::time::Instant::now();
        assert!(sc.accept(&ook_at(868_362_300.0), t0));
        Simple::reset(&mut sc);
        assert!(sc.accept(&ook_at(868_362_300.0), t0), "the memory survived a retune");
    }

    #[test]
    fn the_dedupe_memory_is_shorter_than_a_repeating_device() {
        // Long enough to cover a block boundary, short enough that a sensor
        // repeating its packet two or three times a second still gets a row
        // per repeat.
        assert!(DEDUPE_WINDOW >= std::time::Duration::from_millis(250));
        assert!(DEDUPE_WINDOW <= std::time::Duration::from_millis(400));
    }
}

pub const PROTOCOLS: StageDesc = StageDesc {
    name: "protocols",
    summary: "Run every known protocol over everything on the bus, once",
    category: Category::Decode,
    feeds_bus: false,
};

pub fn build_protocols(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(PacketDecodeNode::default()))
}

pub const DEDUPE: StageDesc = StageDesc {
    name: "dedupe",
    summary: "One row per burst: drop the copies the neighbouring \
              channels and the other front end read of it",
    category: Category::Decode,
    feeds_bus: false,
};

pub fn build_dedupe(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(DedupeNode::default()))
}
