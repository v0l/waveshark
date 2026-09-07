//! A GSM carrier as a graph node.
//!
//! The wiring only. The tone search, the burst demodulator and the channel
//! coding are `dsp::gsm`, and the row a decode becomes is at the bottom of
//! this file; neither knows about pipelines.
//!
//! One node watches one carrier, named by its own frequency the way the APRS
//! and pager front ends name theirs, because that is what a GSM beacon is:
//! cells are 200 kHz apart and a receiver reads one of them at a time.
//! Watching a whole band means placing one of these per carrier, which is the
//! scanner table's job and not a loop hidden in here.
//!
//! The channel is absolute rather than an offset from the span. An offset
//! saved in a patch means a different carrier as soon as the dial moves, and
//! a front end that quietly follows the tuning is one that says it heard a
//! cell where there is none.

use common::Result;
use dsp::gsm::{self, sch, GsmConfig, Hit, SchDetector};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// What one carrier occupies, and the width a burst was heard through.
pub const CHANNEL_WIDTH_HZ: f64 = gsm::CHANNEL_SPACING_HZ;

/// Where to look when nothing says otherwise: the middle of the E-GSM 900
/// downlink, which is the band most likely to hold a beacon in Europe. There
/// is no frequency worth compiling in beyond that, so the scanner table
/// carries the channel and this is only what an unconfigured node opens on.
pub const DEFAULT_HZ: f64 = 947_400_000.0;

pub struct GsmNode {
    cfg: GsmConfig,
    /// Which carrier to watch, as a frequency.
    channel_hz: f64,
    /// The channel number this carrier is, so a grant naming it can be told
    /// from one naming another carrier the receiver is not listening to.
    arfcn: u16,
    det: SchDetector,
    meter: crate::FrameMeter,
    hits: Vec<Hit>,
    accepted: u64,
}

impl Default for GsmNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ, GsmConfig::default())
    }
}

impl GsmNode {
    pub fn new(channel_hz: f64, cfg: GsmConfig) -> Self {
        // Replaced at negotiation, when the real rate and centre are known.
        let rate = 2_400_000.0;
        Self {
            cfg,
            channel_hz,
            arfcn: gsm::arfcn(channel_hz).unwrap_or(u16::MAX),
            det: SchDetector::new(rate, channel_hz, channel_hz, cfg),
            meter: crate::FrameMeter::new(rate, channel_hz as u64, 0.25),
            hits: Vec::new(),
            accepted: 0,
        }
    }

    /// Bursts and blocks that passed their check since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for GsmNode {
    fn name(&self) -> &str {
        "gsm"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("gsm reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if !SchDetector::rate_is_enough(rate) {
            return Err(common::Error::other(
                "gsm needs at least three samples a symbol, so 813 kS/s",
            ));
        }
        // The carrier and its skirts, not just its centre: a channel sitting
        // on the edge of the span is one being read through the anti-alias
        // filter, which is a channel that decodes nothing.
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("gsm needs its carrier inside the span"));
        }
        self.arfcn = gsm::arfcn(self.channel_hz).unwrap_or(u16::MAX);
        self.det = SchDetector::new(rate, center, self.channel_hz, self.cfg);
        self.meter =
            crate::FrameMeter::new(self.det.channel_rate(), self.channel_hz as u64, 0.25);

        // Frames rather than bytes: two bursts written into one buffer cannot
        // be told apart afterwards.
        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.hits.clear();
        self.det.process(iq, &mut self.hits);
        // The channel this cut out, not the span it came from: what a burst
        // was heard at is the level of one carrier.
        self.meter.feed(self.det.channel());
        let out = o.frames_mut();
        for hit in &self.hits {
            // Two kinds of evidence off one carrier: the synchronisation
            // burst's 25 bit field, and the 23 byte blocks the broadcast and
            // common control channels carry. Both are bytes on the bus and
            // the length is what tells them apart, which is the same job the
            // band does for everything else on it.
            let (bytes, start, len) = match hit {
                Hit::Sync(s) => {
                    let Some(b) = sch::pack(&s.sch) else { continue };
                    (b.to_vec(), s.start_sample, s.samples)
                }
                Hit::Block(b) => {
                    // A block off the beacon's own timeslot may be an
                    // assignment, and an assignment says where the next part
                    // of the transaction happens. Following it is the only
                    // way to see a channel a cell hands out: nothing on the
                    // air says one is in use.
                    if b.timeslot == 0 {
                        if let Some(g) = decode::gsm::parse(&b.bytes).and_then(|m| m.grant) {
                            if g.arfcn == Some(self.arfcn) && g.timeslot != 0 {
                                self.det.follow(g.timeslot);
                            }
                        }
                    }
                    (b.bytes.to_vec(), b.start_sample, b.samples)
                }
            };
            self.accepted += 1;
            // The burst's own samples rather than the quarter second of
            // channel it arrived in, and without emptying the ring: a
            // synchronisation burst and the block it announced can both come
            // out of one block of samples.
            out.push(self.meter.frame_at(bytes, start, len).at(self.channel_hz as u64));
        }
        Ok(())
    }

    /// One 200 kHz carrier, which is what makes this a front end something
    /// can place rather than one that has to be named here: the auto node
    /// asks a source's width and puts this on the ones that match.
    fn channels(&self) -> &'static [f64] {
        &[CHANNEL_WIDTH_HZ]
    }

    fn reset(&mut self) {
        self.det.reset();
        self.meter.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float("channel_hz", self.channel_hz, 100e6..=2_000e6)
            .unit("Hz")
            .label("Which carrier to watch")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "channel_hz" => self.channel_hz = v.as_f64().unwrap_or(DEFAULT_HZ),
            _ => return Err(common::Error::other(format!("gsm: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

/// The row a burst or a block becomes.
///
/// Takes the bytes off the bus rather than a parsed message, for the same
/// reason the AIS one does: what travelled is the field the cell
/// transmitted, and a consumer reads it for itself. Which of the two kinds
/// this is comes from the length, because that is what the front end put on
/// the bus: four bytes is the synchronisation field, 23 is a control channel
/// block.
pub fn gsm_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    match bytes.len() {
        4 => sync_decoded(bytes, center),
        gsm::bcch::BLOCK_BYTES => block_decoded(bytes, center),
        _ => None,
    }
}

/// The channel number, where the frequency names one.
fn arfcn_field(center: common::Hz, fields: &mut Vec<(String, common::Value)>) -> Option<u16> {
    let n = gsm::arfcn(center.as_f64())?;
    fields.push(("arfcn".into(), common::Value::Int(i64::from(n))));
    Some(n)
}

fn sync_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let sch = sch::unpack(bytes)?;
    let mut fields: Vec<(String, Value)> = vec![
        ("bsic".into(), Value::Int(i64::from(sch.bsic()))),
        ("ncc".into(), Value::Int(i64::from(sch.ncc))),
        ("bcc".into(), Value::Int(i64::from(sch.bcc))),
        ("frame".into(), Value::Int(i64::from(sch.frame_number))),
    ];
    let arfcn = arfcn_field(center, &mut fields);

    // The cell, as it is written down: the colour code as two octal digits,
    // which is how a base station is configured and how a survey names it.
    let cell = match arfcn {
        Some(n) => format!("ARFCN {n} BSIC {}{}", sch.ncc, sch.bcc),
        None => format!("BSIC {}{}", sch.ncc, sch.bcc),
    };
    let detail = format!("{cell} frame {}", sch.frame_number);
    Some(
        Decoded::bytes("GSM-SCH", center, 0.0, bytes.to_vec())
            .with_link(pipeline::event::Link::beacon(pipeline::event::Party::unit(cell)))
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation("GMSK")
            // The ten parity bits held in the demodulator, which is a real
            // check and the only reason this burst exists rather than a
            // Viterbi decoder's best guess at noise.
            .with_crc(Some(true)),
    )
}

fn block_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    // Two kinds of block share one length. A broadcast block addresses
    // nobody and starts with a pseudo length; a block off a channel the cell
    // assigned has a link layer in front of it. The broadcast reading is
    // tried first because it is the more specific claim: it requires the
    // radio resource discriminator in a fixed place.
    let msg = decode::gsm::parse(bytes).or_else(|| decode::gsm::parse_dedicated(bytes))?;
    let mut fields: Vec<(String, Value)> = vec![
        ("message".into(), Value::Text(msg.name.into())),
        ("message_type".into(), Value::Int(i64::from(msg.type_id))),
    ];
    let arfcn = arfcn_field(center, &mut fields);
    if let Some(id) = msg.cell_id {
        fields.push(("cell_id".into(), Value::Int(i64::from(id))));
    }
    // The neighbours a cell tells phones to measure are where the rest of
    // the network is: a scan that reads one cell has been handed the channel
    // numbers of the others. Its own allocation is a different list and says
    // where its traffic hops, so the two are not run together.
    let list = msg.channels.iter().map(|n| n.to_string()).collect::<Vec<_>>().join(",");
    let list_name = if msg.channels_are_neighbours { "neighbours" } else { "allocation" };
    if !msg.channels.is_empty() {
        fields.push((list_name.into(), Value::Text(list.clone())));
    }
    if !msg.pages.is_empty() {
        let who =
            msg.pages.iter().map(|p| p.to_string()).collect::<Vec<_>>().join(" ");
        fields.push(("paging".into(), Value::Text(who)));
        // A network pages by temporary identity, which it reallocates. One
        // that pages by permanent identity has given that up, and a row that
        // does not separate the two hides it.
        let permanent = msg
            .pages
            .iter()
            .filter(|p| !matches!(p, decode::gsm::Identity::Tmsi(_)))
            .count();
        if permanent > 0 {
            fields.push(("paged_by_identity".into(), Value::Int(permanent as i64)));
        }
    }
    if let Some(id) = &msg.identity {
        fields.push(("phone".into(), Value::Text(id.to_string())));
    }
    if let Some((power, ta)) = msg.sacch {
        fields.push(("ordered_power".into(), Value::Int(i64::from(power))));
        fields.push(("timing_advance".into(), Value::Int(i64::from(ta))));
        fields.push(("range_m".into(), Value::Int(i64::from(ta) * 554)));
    }
    if msg.sapi != 0 {
        fields.push(("sapi".into(), Value::Int(i64::from(msg.sapi))));
    }
    if let Some(g) = msg.grant {
        fields.push(("channel_type".into(), Value::Text(g.kind.into())));
        fields.push(("timeslot".into(), Value::Int(i64::from(g.timeslot))));
        fields.push(("tsc".into(), Value::Int(i64::from(g.tsc))));
        if let Some(n) = g.arfcn {
            fields.push(("granted_arfcn".into(), Value::Int(i64::from(n))));
        }
        if let Some((maio, hsn)) = g.hopping {
            fields.push(("hopping".into(), Value::Text(format!("MAIO {maio} HSN {hsn}"))));
        }
        // The timing advance is how long the phone's burst took to arrive,
        // so it is a range to it and worth a field of its own.
        fields.push(("timing_advance".into(), Value::Int(i64::from(g.timing_advance))));
        fields.push(("range_m".into(), Value::Int(i64::from(g.distance_m()))));
    }
    if let Some(lai) = msg.lai {
        fields.push(("mcc".into(), Value::Int(i64::from(lai.mcc))));
        fields.push(("mnc".into(), Value::Int(i64::from(lai.mnc))));
        fields.push(("lac".into(), Value::Int(i64::from(lai.lac))));
        fields.push(("plmn".into(), Value::Text(lai.to_string())));
    }

    // A cell that names itself is a party worth tracking across sightings;
    // one that does not is still a message from whatever carrier this is.
    let mut detail = msg.name.to_string();
    let mut party = None;
    if let Some(lai) = msg.lai {
        detail.push_str(&format!(" {lai} LAC {}", lai.lac));
        if let Some(id) = msg.cell_id {
            detail.push_str(&format!(" CI {id}"));
            // Operator, area and cell: the identity a cell is known by
            // everywhere, so two receivers in different places agree about
            // which one they heard and a survey can accumulate it.
            let cell = format!("{lai}-{}-{id}", lai.lac);
            fields.push(("cell".into(), Value::Text(cell.clone())));
            party = Some(cell);
        }
    }
    if !msg.channels.is_empty() {
        detail.push_str(&format!(" {list_name} {list}"));
    }
    for p in &msg.pages {
        detail.push_str(&format!(" {p}"));
    }
    if let Some(id) = &msg.identity {
        detail.push_str(&format!(" {id}"));
    }
    if let Some((_, ta)) = msg.sacch {
        detail.push_str(&format!(" {} m away", u32::from(ta) * 554));
    }
    if let Some(g) = msg.grant {
        detail.push_str(&format!(" {} sub {} TS {}", g.kind, g.subchannel, g.timeslot));
        match (g.arfcn, g.hopping) {
            (Some(n), _) => detail.push_str(&format!(" ARFCN {n}")),
            (_, Some((maio, hsn))) => detail.push_str(&format!(" MAIO {maio} HSN {hsn}")),
            _ => {}
        }
        detail.push_str(&format!(" {} m away", g.distance_m()));
    }
    if party.is_none() {
        party = arfcn.map(|n| format!("ARFCN {n}"));
    }

    let protocol = match msg.name {
        n if n.starts_with("SI") => "GSM-SI",
        n if n.starts_with("Paging") || n.starts_with("Imm") => "GSM-CCCH",
        // A channel the cell assigned: what happens on it is a transaction
        // with one phone rather than something broadcast.
        _ => "GSM-SDCCH",
    };
    let mut d = Decoded::bytes(protocol, center, 0.0, bytes.to_vec())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation("GMSK")
        // Forty bits of Fire code held over the block before it left the
        // demodulator.
        .with_crc(Some(true));
    if let Some(p) = party {
        d = d.with_link(pipeline::event::Link::beacon(pipeline::event::Party::unit(p)));
    }
    Some(d)
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Hz, C32};
    use dsp::gsm::Sch;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_span_it_cannot_read() {
        let mut n = GsmNode::default();
        assert!(n.negotiate(&spec(2_400_000.0, DEFAULT_HZ)).is_ok());
        // Under three samples a symbol there is nothing to interpolate.
        assert!(n.negotiate(&spec(400_000.0, DEFAULT_HZ)).is_err());
        // A carrier outside the span is not a carrier.
        n.set_param("channel_hz", ParamValue::Float(DEFAULT_HZ + 1_400_000.0)).unwrap();
        assert!(n.negotiate(&spec(2_400_000.0, DEFAULT_HZ)).is_err());
    }

    #[test]
    fn the_node_outputs_frames_tagged_with_the_carrier() {
        let mut n = GsmNode::new(948_000_000.0, GsmConfig::default());
        let out = n.negotiate(&spec(2_400_000.0, 947_400_000.0)).unwrap();
        assert_eq!(out.kind, PortKind::Frames);
        assert_eq!(out.center, Hz(948_000_000));
        assert_eq!(out.bandwidth, CHANNEL_WIDTH_HZ);
    }

    /// A cell becomes a row saying which cell it is.
    #[test]
    fn a_burst_becomes_a_row_naming_the_cell() {
        use common::Value;
        let sch = Sch { ncc: 5, bcc: 3, frame_number: 51 * 26 * 42 + 21 };
        let bytes = sch::pack(&sch).unwrap();
        let d = gsm_decoded(&bytes, Hz(947_400_000)).expect("a row");
        assert_eq!(d.protocol, "GSM-SCH");
        assert_eq!(d.crc_ok, Some(true));
        let get = |k: &str| d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("bsic"), Some(Value::Int(0x2B)));
        assert_eq!(get("frame"), Some(Value::Int(i64::from(sch.frame_number))));
        assert_eq!(get("arfcn"), Some(Value::Int(62)), "947.4 MHz is channel 62");
        assert!(d.detail.as_deref().unwrap().contains("BSIC 53"));
    }

    /// The whole path on synthetic RF: a beacon into the node, a cell out of
    /// the row it produces.
    ///
    /// The point is that the three layers agree. The demodulator's soft bits,
    /// the channel coding's field order and the row's reading of the packed
    /// bytes are each tested alone and each could be self-consistently wrong;
    /// only running them together shows that what the node puts on the bus is
    /// what a consumer gets back.
    #[test]
    fn a_modulated_beacon_becomes_a_cell_on_the_bus() {
        let want = Sch { ncc: 2, bcc: 6, frame_number: 51 * 26 * 9 + 31 };
        let (rate, center) = (2_400_000.0, 947_400_000.0);
        let iq = beacon(&want, rate);

        let mut node = GsmNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames: Vec<common::Frame> = Vec::new();
        for block in iq.chunks(8192) {
            let input = Payload::Iq(block.to_vec());
            let mut out = Payload::Frames(Vec::new());
            let mut events = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Frames(f) = out {
                frames.extend(f);
            }
        }

        assert_eq!(frames.len(), 2, "expected both bursts off the air");
        let f = &frames[0];
        // Every packet carries what it was heard at, measured on the channel
        // rather than on the span.
        assert!(f.rssi_dbfs.is_finite(), "level {}", f.rssi_dbfs);
        assert!(f.snr_db.is_finite() && f.snr_db > 0.0, "snr {}", f.snr_db);
        assert!(f.iq.is_some(), "a burst with no samples behind it");

        let d = gsm_decoded(&f.bytes, Hz(f.center_hz)).expect("a row");
        assert_eq!(d.detail.as_deref(), Some("ARFCN 62 BSIC 26 frame 11965"));
    }

    /// The block a cell broadcasts becomes a row naming the operator, the
    /// location area and the cell.
    #[test]
    fn a_broadcast_block_becomes_a_row_naming_the_operator() {
        use common::Value;
        let mut block = [0x2Bu8; 23];
        block[..8].copy_from_slice(&[0x49, 0x06, 0x1B, 0x12, 0x34, 0x62, 0xF2, 0x10]);
        let d = gsm_decoded(&block, Hz(947_400_000)).expect("a row");
        assert_eq!(d.protocol, "GSM-SI");
        assert_eq!(d.crc_ok, Some(true));
        let get = |k: &str| d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("message"), Some(Value::Text("SI3".into())));
        assert_eq!(get("cell_id"), Some(Value::Int(0x1234)));
        assert_eq!(get("plmn"), Some(Value::Text("262-01".into())));
        assert_eq!(d.detail.as_deref(), Some("SI3 262-01 LAC 11051 CI 4660"));
    }

    /// A block off a channel the cell assigned becomes its own kind of row:
    /// a transaction with one phone rather than something broadcast.
    #[test]
    fn a_dedicated_block_becomes_a_row_naming_the_phone() {
        use common::Value;
        let mut b = vec![0x01, 0x03, 15 << 2, 0x05, 0x08, 0x70];
        b.extend_from_slice(&[0x00, 0xF1, 0x10, 0x00, 0x01, 0x33]);
        b.extend_from_slice(&[0x05, 0xF4, 0xAA, 0xBB, 0xCC, 0xDD]);
        b.resize(23, 0x2B);
        let d = gsm_decoded(&b, Hz(947_400_000)).expect("a row");
        assert_eq!(d.protocol, "GSM-SDCCH");
        let get = |k: &str| d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("message"), Some(Value::Text("LocationUpdatingRequest".into())));
        assert_eq!(get("phone"), Some(Value::Text("TMSI AABBCCDD".into())));
        assert!(d.detail.as_deref().unwrap().contains("001-01"));
    }

    /// A filler frame is not a row. A cell with nothing to say fills its
    /// blocks with 0x2B, and every one of those passes the Fire code.
    #[test]
    fn padding_is_not_a_row() {
        assert!(gsm_decoded(&[0x2Bu8; 23], Hz(947_400_000)).is_none());
    }

    /// Two beacons ten frames apart, which is what the control multiframe
    /// holds: a frequency correction burst and the synchronisation burst one
    /// frame after it, twice. Two rather than one because a synchronisation
    /// burst is reported only once a second agrees with it about the time.
    fn beacon(sch: &Sch, rate: f64) -> Vec<C32> {
        let sps = 8;
        let work = gsm::SYMBOL_RATE * sps as f64;
        let lead = 200.0;
        let total = ((lead * 2.0 + 12.0 * gsm::FRAME_SYMBOLS) * sps as f64) as usize;
        let mut base = vec![C32::new(0.0, 0.0); total];
        let mut place = |at: f64, wave: &[C32]| {
            let at = (at * sps as f64) as usize;
            base[at..at + wave.len()].copy_from_slice(wave);
        };
        for n in 0..2u32 {
            let at = lead + 10.0 * f64::from(n) * gsm::FRAME_SYMBOLS;
            let this = Sch { frame_number: sch.frame_number + 10 * n, ..*sch };
            place(at, &gsm::modulate(&[0u8; gsm::BURST_BITS], sps));
            place(
                at + gsm::FRAME_SYMBOLS,
                &gsm::modulate(&gsm::sch_burst_bits(&this).unwrap(), sps),
            );
        }

        let ratio = work / rate;
        let n = (base.len() as f64 / ratio) as usize - 1;
        let mut seed = 0x9E37_79B9u32;
        let mut rand = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            (seed as f32 / u32::MAX as f32) - 0.5
        };
        // A little noise, so the floor the level is measured against is a
        // floor rather than a divide by zero.
        (0..n).map(|i| base[(i as f64 * ratio) as usize] + C32::new(rand(), rand()) * 0.05).collect()
    }
}
