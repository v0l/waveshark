//! Decoding, as a consumer of the packet bus.
//!
//! Every front end puts what it produced on the bus, and this reads it: a
//! burst of timings goes through the protocol tables, a frame goes through
//! the Mode S parser. Both come out as decodes, which is what a packet list,
//! a chart or an alert wants.
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
use decode::{adsb, Protocols};
use pipeline::event::{Decoded, Event};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

use crate::decode_nodes::{decoded_event, unmatched_event};

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
}

impl Default for PacketDecodeNode {
    fn default() -> Self {
        Self::new(Protocols::all())
    }
}

impl PacketDecodeNode {
    pub fn new(protocols: Protocols) -> Self {
        Self { protocols, report_all: true, report_unknown: true, hits: Vec::new() }
    }

    pub fn hits(&self) -> &[Decoded] {
        &self.hits
    }

    fn decode_burst(&mut self, p: &Packet, pkg: &common::Package, modulation: &'static str) {
        let center = common::Hz(p.center_hz);
        let mut matched = false;
        // A protocol that fails is not reported. A CRC failure in particular
        // is a protocol saying "those were my timings but the reception was
        // not good enough", which is worth knowing while tuning a chain and
        // is noise in a packet list.
        for (_, res) in self.protocols.diagnose(pkg) {
            if let Ok(report) = res {
                matched = true;
                self.hits.push(
                    decoded_event(&report, pkg, center, modulation)
                        .with_bandwidth(p.bandwidth_hz as f64)
                        .with_iq(p.iq.clone()),
                );
                if !self.report_all {
                    break;
                }
            }
        }
        if !matched && self.report_unknown {
            // The keying column shows what the classifier measured where it
            // is more specific than the front end that read the burst: a
            // chirp or a carrier that no front end reads, or a burst it
            // could not name at all.
            let label = match p.measure.as_ref().map(|m| m.modulation) {
                Some(l) if l != "unknown" => l,
                _ => modulation,
            };
            self.hits.push(
                unmatched_event(pkg, center, label, p.measure.as_ref())
                    .with_bandwidth(p.bandwidth_hz as f64)
                    .with_iq(p.iq.clone()),
            );
        }
    }

    /// A frame from a demodulator that produces bytes.
    ///
    /// Mode S and AIS both arrive here as bytes with nothing to distinguish
    /// them, so the packet's own centre frequency does it. That is not a tag
    /// somebody attached: where a frame was received is evidence the packet
    /// already carries, and a 162 MHz frame is not a Mode S frame no matter
    /// what its bits would parse as.
    ///
    /// Parsing again here rather than carrying the demodulator's own parse on
    /// the bus is deliberate: what travels is the evidence, and every consumer
    /// draws its own conclusions from it.
    fn decode_frame(&mut self, p: &Packet, bytes: &[u8]) {
        let center = common::Hz(p.center_hz);
        // M17 is the one protocol here that its own frequency cannot
        // identify: it runs wherever an amateur puts it, which includes the
        // 2 m channels APRS uses and the 70 cm ones near the pager bands. So
        // it is recognised by shape instead, and tried first because that
        // shape is the most specific claim any of these make: a tagged event
        // of an exact length, carrying a link setup frame whose CRC checks.
        if let Some(d) = crate::m17_nodes::m17_decoded(bytes, center) {
            // The speech travelled with the packet rather than in its bytes,
            // and this is the row it belongs to.
            self.hits.push(d.with_audio(p.audio.clone()));
            return;
        }
        // A TETRA broadcast identifies itself twice over: it arrives from a
        // downlink band, and its bytes are a tagged PDU that had to pass the
        // standard's own CRC to exist at all.
        if dsp::tetra::is_downlink_band(p.center_hz as f64) {
            if let Some(d) = crate::tetra_nodes::tetra_decoded(bytes, center) {
                self.hits.push(d);
            }
            return;
        }
        if dsp::ais::is_ais_band(p.center_hz as f64) {
            let Ok(frame) = decode::ais::parse(bytes) else { return };
            self.hits.push(crate::ais_nodes::ais_decoded(&frame, bytes, center));
            return;
        }
        // The 2 m packet channels. Wherever the scanner put APRS, a frame
        // from it arrives tagged with that channel rather than 1090.
        //
        // Tested before the pager bands because 144 to 146 MHz is inside the
        // VHF paging allocation. Two protocols really do share that spectrum,
        // and the narrower window is the more specific claim.
        if dsp::afsk::is_packet_band(p.center_hz as f64) {
            let Ok(frame) = decode::ax25::parse(bytes) else { return };
            self.hits.push(crate::aprs_nodes::aprs_decoded(&frame, bytes, center));
            return;
        }
        // A pager transmission is codewords rather than one frame, so it can
        // become several rows: a transmitter empties its queue in one go.
        if dsp::pocsag::is_pager_band(p.center_hz as f64) {
            self.hits.extend(crate::pocsag_nodes::pocsag_decoded(bytes, center));
            return;
        }
        // Meters: the 868.95 MHz uplink and the older 868.3 MHz mode.
        if dsp::wmbus::is_wmbus_band(p.center_hz as f64) {
            if let Some(d) = crate::wmbus_nodes::wmbus_decoded(bytes, center) {
                self.hits.push(d);
            }
            return;
        }
        let Ok(frame) = adsb::parse(bytes) else { return };
        let mut d = crate::modes_nodes::adsb_decoded(&frame, bytes, center);
        // A local demodulator reports no level for a frame it has already
        // accepted, but a Beast feed carries one, and dropping it would make
        // a remote receiver's frames look weaker than nothing.
        if p.rssi_dbfs.is_finite() {
            d = d.with_level(p.rssi_dbfs, p.snr_db);
        }
        self.hits.push(d);
    }
}

impl Simple for PacketDecodeNode {
    fn name(&self) -> &str {
        "packet_decode"
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

    fn process(&mut self, i: &Payload, _o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        self.hits.clear();
        let packets: Vec<Packet> = i.as_packets().unwrap_or(&[]).to_vec();
        for p in &packets {
            match &p.body {
                PacketBody::Pulses(_) => {
                    let Some(pkg) = p.package() else { continue };
                    // Which keying a burst arrived under is not something the
                    // protocols can tell, and it belongs in the packet list's
                    // own column: a device that exists in both an OOK and an
                    // FSK variant decodes the same either way.
                    //
                    // Measured where a classifier saw the burst. The fallback
                    // is the channel width the packet arrived through, which
                    // is only ever a guess: the wide tier carries plenty of
                    // on-off keyed sensors, and this used to label every one
                    // of them FSK.
                    let modulation = p.modulation.unwrap_or(match p.measure.as_ref() {
                        Some(m) => m.modulation,
                        None if p.bandwidth_hz > 60_000 => "FSK",
                        None => "OOK",
                    });
                    self.decode_burst(p, &pkg, modulation);
                }
                PacketBody::Frame(bytes) => self.decode_frame(p, bytes),
            }
        }
        for d in &self.hits {
            c.emit(Event::Decoded(d.clone()));
        }
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool("report_all", self.report_all).label("Report every matching protocol"),
            Param::bool("report_unknown", self.report_unknown)
                .label("Report unrecognised bursts"),
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

#[cfg(test)]
mod tests {
    use super::*;
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
        node.hits().to_vec()
    }

    fn burst(center_hz: u64, bandwidth_hz: u32, pulses: Vec<Pulse>) -> Packet {
        Packet {
            at_us: 0,
            center_hz,
            bandwidth_hz,
            rssi_dbfs: -20.0,
            snr_db: 22.0,
            modulation: None,
            body: PacketBody::Pulses(pulses),
            measure: None,
            iq: None,
            audio: None,
        }
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
            vec![Packet {
                at_us: 0,
                center_hz: 1_090_000_000,
                bandwidth_hz: 2_000_000,
                rssi_dbfs: f32::NAN,
                snr_db: f32::NAN,
                modulation: None,
                body: PacketBody::Frame(bytes),
                measure: None,
                iq: None,
                audio: None,
            }],
        );
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].protocol, "ADSB-Identification");
        assert!(hits[0].detail.as_deref().unwrap_or_default().contains("KLM1023"));
    }

    #[test]
    fn the_channel_width_says_which_front_end_heard_it() {
        // The packet list has a column for it, and the protocols cannot say:
        // plenty of devices exist in both an OOK and an FSK variant.
        let mut n = PacketDecodeNode::default();
        let pulses: Vec<Pulse> = (0..24).map(|_| Pulse { mark: 500, gap: 1500 }).collect();
        let ook = run(&mut n, vec![burst(433_920_000, 31_250, pulses.clone())]);
        assert_eq!(ook[0].modulation, Some("OOK"));
        let fsk = run(&mut n, vec![burst(868_300_000, 125_000, pulses)]);
        assert_eq!(fsk[0].modulation, Some("FSK"));
    }

    /// A pager transmission arrives as bytes like a Mode S frame does, and
    /// only where it was received says which it is. It also becomes several
    /// rows rather than one, because a transmitter sends its whole queue in
    /// one go.
    #[test]
    fn a_pager_transmission_becomes_a_row_for_each_page() {
        use decode::pocsag::Body;
        let mut contents = decode::pocsag::encode(1_000_001, 3, &Body::Alpha("ON CALL".into()));
        contents.extend(decode::pocsag::encode(2_000_002, 0, &Body::Numeric("999".into())));
        let bytes: Vec<u8> = contents
            .into_iter()
            .flat_map(|c| dsp::pocsag::encode_codeword(c).to_be_bytes())
            .collect();

        let mut n = PacketDecodeNode::default();
        let hits = run(
            &mut n,
            vec![Packet {
                at_us: 0,
                center_hz: 439_987_500,
                bandwidth_hz: 12_500,
                rssi_dbfs: f32::NAN,
                snr_db: f32::NAN,
                modulation: None,
                body: PacketBody::Frame(bytes),
                measure: None,
                iq: None,
                audio: None,
            }],
        );
        assert_eq!(hits.len(), 2);
        assert_eq!(hits[0].protocol, "POCSAG-Alpha");
        assert_eq!(hits[0].text.as_deref(), Some("ON CALL"));
        assert_eq!(hits[1].protocol, "POCSAG-Numeric");
    }

    #[test]
    fn a_frame_that_is_not_mode_s_is_dropped_rather_than_guessed_at() {
        let mut n = PacketDecodeNode::default();
        let hits = run(
            &mut n,
            vec![Packet {
                at_us: 0,
                center_hz: 1_090_000_000,
                bandwidth_hz: 2_000_000,
                rssi_dbfs: f32::NAN,
                snr_db: f32::NAN,
                modulation: None,
                body: PacketBody::Frame(vec![0xff; 5]),
                measure: None,
                iq: None,
                audio: None,
            }],
        );
        assert!(hits.is_empty());
    }
}
