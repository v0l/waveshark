//! UAT at 978 MHz as a graph node.
//!
//! The wiring only: the waveform is `dsp::fsk`'s sync-word detector, the
//! frames and the FIS-B products are `decode::uat`, and neither knows about
//! the other or about pipelines.
//!
//! One node rather than two, for the reason Mode S is one: a frame the
//! demodulator believes blanks the air it occupied, and the only thing that
//! can tell a sync word off noise from a real one is the Reed-Solomon
//! decode. So the acceptance test runs inside the search, as the closure the
//! detector is given, and a rejected match blanks nothing.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::uat::{self, Frame as UatFrame};
use dsp::fsk::{SyncBurst, SyncDetector, SyncPattern};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// What the signal occupies: 1.04 Mbit/s keyed 312.5 kHz either side, which
/// is about 1.4 MHz by Carson's rule, plus room for a tuner's error.
pub const CHANNEL_WIDTH_HZ: f64 = 2_000_000.0;

/// How many sync bits may be wrong and the word still be this one.
///
/// Four of 36. The cost of raising it is candidates the Reed-Solomon decode
/// then throws away: measured on synthesised noise, four allowed gives 4
/// candidates in 4.8 million samples and none of them corrects.
const MAX_SYNC_ERRORS: u32 = 4;

/// Which of the detector's two patterns matched.
const ADSB: usize = 0;
const UPLINK: usize = 1;

pub struct UatNode {
    det: SyncDetector,
    meter: crate::FrameMeter,
    bursts: Vec<SyncBurst>,
    accepted: u64,
}

impl Default for UatNode {
    fn default() -> Self {
        Self::new()
    }
}

fn patterns() -> Vec<SyncPattern> {
    vec![
        SyncPattern {
            word: uat::ADSB_SYNC,
            bits: uat::SYNC_BITS,
            // The long form always: a basic message is the first 30 bytes of
            // it, and which one it was is the payload type code's to say.
            payload_bits: uat::LONG_BYTES * 8,
            max_errors: MAX_SYNC_ERRORS,
        },
        SyncPattern {
            word: uat::UPLINK_SYNC,
            bits: uat::SYNC_BITS,
            payload_bits: uat::UPLINK_BYTES * 8,
            max_errors: MAX_SYNC_ERRORS,
        },
    ]
}

impl UatNode {
    pub fn new() -> Self {
        Self {
            // Replaced at negotiation, when the real rate is known.
            det: SyncDetector::new(2_400_000.0, uat::BAUD, patterns()),
            // An uplink frame is 4.4 ms of air; ten milliseconds holds one
            // whole with room either side.
            meter: crate::FrameMeter::new(2_400_000.0, uat::CHANNEL_HZ as u64, 0.01),
            bursts: Vec::new(),
            accepted: 0,
        }
    }

    /// Frames that corrected since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

/// The bits of a burst as bytes, most significant bit first, which is the
/// order UAT keys them in.
fn pack(bits: &[bool]) -> Vec<u8> {
    bits.chunks(8)
        .map(|c| c.iter().enumerate().fold(0u8, |b, (i, &v)| b | (u8::from(v) << (7 - i))))
        .collect()
}

/// The payload a burst carries once its code has corrected it, or nothing
/// where the sync word was noise.
fn correct(b: &SyncBurst) -> Option<uat::Corrected> {
    let bytes = pack(&b.bits);
    match b.pattern {
        ADSB => uat::correct_adsb(&bytes),
        UPLINK => uat::correct_uplink(&bytes),
        _ => None,
    }
}

impl Simple for UatNode {
    fn name(&self) -> &str {
        "uat"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("uat reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if rate < 2.0 * uat::BAUD {
            return Err(common::Error::other(
                "uat needs 2.084 MS/s or more: its bits are 0.96 us wide",
            ));
        }
        // The channel and its skirts have to be inside the span. Read
        // through the anti-alias filter's edge it is silence.
        if (center - uat::CHANNEL_HZ).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("uat needs 978 MHz inside the span"));
        }
        self.det = SyncDetector::new(rate, uat::BAUD, patterns());
        self.meter = crate::FrameMeter::new(rate, uat::CHANNEL_HZ as u64, 0.01);
        // Frames rather than bytes: an 18-byte basic message and a 34-byte
        // long one written into one buffer cannot be told apart afterwards,
        // and the length is what says which it was.
        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(uat::CHANNEL_HZ as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.meter.feed(iq);
        self.bursts.clear();
        // The code is the acceptance test, so only a frame that corrected
        // blanks the air behind it.
        self.det.process_valid(iq, &mut self.bursts, &|b: &SyncBurst| correct(b).is_some());
        let out = o.frames_mut();
        for b in &self.bursts {
            let Some(c) = correct(b) else { continue };
            self.accepted += 1;
            let snr = self.meter.snr_db_at(b.at_sample, b.len_samples);
            out.push(self.meter.frame_measured(c.data, b.at_sample, b.len_samples, snr));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        self.det.reset();
    }
}

/// The rows a corrected UAT payload becomes: one for an aircraft, and for a
/// ground station one for the station and one per product it sent.
pub fn uat_decoded(frame: &UatFrame, bytes: &[u8], center: common::Hz) -> Vec<Decoded> {
    match frame {
        UatFrame::Adsb(a) => vec![adsb_decoded(a, bytes, center)],
        UatFrame::Uplink(u) => uplink_decoded(u, bytes, center),
    }
}

fn round1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

fn round5(v: f64) -> f64 {
    (v * 100_000.0).round() / 100_000.0
}

fn adsb_decoded(a: &uat::Adsb, bytes: &[u8], center: common::Hz) -> Decoded {
    use common::Value;
    let mut fields: Vec<(String, Value)> = Vec::new();
    let address = format!("{:06x}", a.address);
    fields.push(("address".into(), Value::Text(address.clone())));
    fields.push(("address_type".into(), Value::Text(a.qualifier.name().into())));

    let mut position = None;
    let mut altitude_ft = None;
    let mut ground_speed_kt = None;
    let mut track_deg = None;
    let mut vertical_rate_fpm = None;
    if let Some(sv) = &a.state {
        if let Some((lat, lon)) = sv.position {
            fields.push(("lat".into(), Value::Float(round5(lat))));
            fields.push(("lon".into(), Value::Float(round5(lon))));
        }
        if let Some(alt) = sv.altitude_ft {
            altitude_ft = Some(alt);
            fields.push(("altitude_ft".into(), Value::Int(i64::from(alt))));
        }
        if let Some(src) = sv.altitude_source {
            fields.push(("altitude_source".into(), Value::Text(src.name().into())));
        }
        if let Some(v) = sv.ground_speed_kt {
            ground_speed_kt = Some(v);
            fields.push(("ground_speed_kt".into(), Value::Float(round1(v))));
        }
        if let (Some(d), Some(k)) = (sv.track_deg, sv.track_kind) {
            track_deg = Some(d);
            fields.push((k.name().replace(' ', "_"), Value::Float(round1(d))));
        }
        if let Some(v) = sv.vertical_rate_fpm {
            vertical_rate_fpm = Some(v);
            fields.push(("vertical_rate_fpm".into(), Value::Int(i64::from(v))));
        }
        fields.push(("nic".into(), Value::Int(i64::from(sv.nic))));
        position = sv.position.map(|(lat, lon)| common::Position {
            lat,
            lon,
            altitude_m: sv.altitude_ft.map(|ft| f64::from(ft) * 0.3048),
            speed_kt: sv.ground_speed_kt,
            course_deg: sv.track_deg,
        });
    }

    let mut name = None;
    if let Some(ms) = &a.status {
        if let Some(cs) = &ms.callsign {
            let key = if ms.callsign_is_squawk { "squawk" } else { "callsign" };
            if !ms.callsign_is_squawk {
                name = Some(cs.clone());
            }
            fields.push((key.into(), Value::Text(cs.clone())));
        }
        fields.push(("emitter".into(), Value::Text(ms.emitter.name().into())));
        if ms.emergency != uat::Emergency::None {
            fields.push(("emergency".into(), Value::Text(ms.emergency.name().into())));
        }
        if ms.ident_active {
            fields.push(("ident".into(), Value::Bool(true)));
        }
    }
    if let Some(alt) = a.secondary_altitude_ft {
        fields.push(("secondary_altitude_ft".into(), Value::Int(i64::from(alt))));
    }

    // Named for what the frame says, not for its type code: a frame with a
    // position is a position report whichever of the eleven forms carried it.
    let protocol = match (position.is_some(), a.qualifier) {
        (_, uat::AddressQualifier::IcaoTisb | uat::AddressQualifier::TisbTrackFile) => "UAT-TISB",
        (true, _) => "UAT-Position",
        (false, _) => "UAT-Status",
    };
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let mut d = Decoded::bytes(protocol, center, 0.0, bytes.to_vec())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::Fsk2)
        // Every frame here corrected under its Reed-Solomon code, which is a
        // real integrity check.
        .with_crc(Some(true))
        .reporting(common::ReportDetail::Aircraft {
            altitude_ft,
            ground_speed_kt,
            track_deg,
            vertical_rate_fpm,
            squawk: None,
            wind: None,
            temp_c: None,
            // UAT sends the position itself: nothing to pair up across
            // frames the way 1090 MHz needs.
            cpr: None,
        });
    d.position = position;
    // A track file number is the ground station's bookkeeping and not an
    // address, so it names nobody.
    if a.qualifier.is_icao() || a.qualifier == uat::AddressQualifier::Vehicle {
        d.link = Some(pipeline::event::Link::beacon(pipeline::event::Party::unit(address.clone())));
        let mut who = common::Identity::new("uat", address);
        who.name = name;
        d.identity = Some(who);
    }
    d
}

fn uplink_decoded(u: &uat::Uplink, bytes: &[u8], center: common::Hz) -> Vec<Decoded> {
    use common::Value;
    let mut fields: Vec<(String, Value)> = Vec::new();
    if let Some((lat, lon)) = u.position {
        fields.push(("lat".into(), Value::Float(round5(lat))));
        fields.push(("lon".into(), Value::Float(round5(lon))));
    }
    fields.push(("position_valid".into(), Value::Bool(u.position_valid)));
    fields.push(("slot".into(), Value::Int(i64::from(u.slot_id))));
    fields.push(("tisb_site".into(), Value::Int(i64::from(u.tisb_site_id))));
    fields.push(("frames".into(), Value::Int(u.frames.len() as i64)));
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let station = format!("gs{:02}", u.tisb_site_id);
    let mut d = Decoded::bytes("UAT-Uplink", center, 0.0, bytes.to_vec())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(true))
        .reporting(common::ReportDetail::Station { aid: false });
    // The station vouches for its own position, so a doubtful one is not
    // plotted.
    if u.position_valid {
        d.position =
            u.position.map(|(lat, lon)| common::Position { lat, lon, ..Default::default() });
    }
    d.identity = Some(common::Identity::new("uat-gs", station));
    let mut out = vec![d];
    out.extend(u.frames.iter().filter_map(|f| product_decoded(f, center)));
    out
}

fn product_decoded(f: &uat::InfoFrame, center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let fisb = f.fisb.as_ref()?;
    let mut fields: Vec<(String, Value)> = vec![
        ("product".into(), Value::Text(uat::product_name(fisb.product_id).into())),
        ("product_id".into(), Value::Int(i64::from(fisb.product_id))),
        ("format".into(), Value::Text(fisb.format.name().into())),
        (
            "issued".into(),
            Value::Text(match (fisb.month_day, fisb.seconds) {
                (Some((m, day)), Some(s)) => {
                    format!("{m:02}-{day:02} {:02}:{:02}:{s:02}", fisb.hours, fisb.minutes)
                }
                (Some((m, day)), None) => {
                    format!("{m:02}-{day:02} {:02}:{:02}", fisb.hours, fisb.minutes)
                }
                (None, Some(s)) => format!("{:02}:{:02}:{s:02}", fisb.hours, fisb.minutes),
                (None, None) => format!("{:02}:{:02}", fisb.hours, fisb.minutes),
            }),
        ),
        ("bytes".into(), Value::Int(fisb.data.len() as i64)),
    ];
    let text = fisb.text.as_ref().map(|t| t.trim_end().to_string()).filter(|t| !t.is_empty());
    if let Some(t) = &text {
        fields.push(("text".into(), Value::Text(t.clone())));
    }
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let mut d = Decoded::bytes("FISB", center, 0.0, fisb.data.clone())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(true));
    if let Some(t) = text {
        d.media_type = common::media::TEXT;
        d.text = Some(t);
        // A weather report a machine composed and broadcast to everybody in
        // range. Nobody wrote it and it is addressed to nobody, so it
        // belongs in the packet list and not in the messages.
        d.written = false;
    }
    Some(d)
}

/// UAT as the auto node and the tables know it: the 978 MHz channel, read
/// off the span because a frame is over before a detector could open a
/// source on it.
pub struct Uat;

impl Protocol for Uat {
    fn id(&self) -> &'static str {
        "uat"
    }
    fn label(&self) -> &'static str {
        "uat"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["uat978", "adsb978", "fisb"]
    }
    /// An aircraft sends its own position, and a ground station its site.
    fn reports_position(&self) -> bool {
        true
    }
    fn placement(&self) -> Placement {
        Placement::Channels(vec![uat::CHANNEL_HZ])
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 2_000_000 }
    }
    /// A UAT payload and any other frame are both bytes; where it was
    /// received is what tells them apart, and then its length says which of
    /// the three forms it is.
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        if !uat::is_uat_band(p.center_hz() as f64) {
            return None;
        }
        let center = common::Hz(p.center_hz());
        Some(uat::parse(bytes).map(|f| uat_decoded(&f, bytes, center)).unwrap_or_default())
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            // Two samples a bit is the floor the correlator reads at.
            min_rate_hz: 2.0 * uat::BAUD,
            feed_rate_hz: 2_400_000.0,
            span_wide: true,
            families: &[],
        }
    }
    fn stage_label(&self, _hz: f64) -> String {
        "978 UAT".into()
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "UAT".into() }]
    }
    /// Every sample gets a discriminator and two correlations, so what it is
    /// handed is what it costs: 2.4 MS/s is plenty for a 1 Mbit/s link and
    /// the 20 MS/s a receiver may be running is eight times the work for the
    /// same frames.
    fn narrow_span(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("uat")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "uat",
    summary: "978 MHz UAT: sync word search, FSK bits and Reed-Solomon",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(UatNode::default()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_rate_its_bits_cannot_survive() {
        let mut n = UatNode::default();
        assert!(n.negotiate(&spec(2_000_000.0, uat::CHANNEL_HZ)).is_err());
        assert!(n.negotiate(&spec(2_400_000.0, uat::CHANNEL_HZ)).is_ok());
        // Tuned elsewhere: the channel is not in the span.
        assert!(n.negotiate(&spec(2_400_000.0, 1_090_000_000.0)).is_err());
    }

    /// Bits keyed at the UAT rate and deviation, sync word first.
    fn on_air(sync: u64, payload: &[u8]) -> Vec<C32> {
        let mut bits: Vec<bool> = (0..uat::SYNC_BITS).rev().map(|k| (sync >> k) & 1 == 1).collect();
        for b in payload {
            for k in (0..8).rev() {
                bits.push((b >> k) & 1 == 1);
            }
        }
        dsp::fsk::modulate(&bits, 2_400_000.0, uat::BAUD, uat::DEVIATION_HZ, 1.0)
    }

    fn noise(n: usize, seed: u64, amp: f32) -> Vec<C32> {
        let mut s = seed;
        let mut rng = move || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((s >> 33) as f32 / (1u64 << 30) as f32 - 1.0) * amp
        };
        (0..n).map(|_| C32::new(rng(), rng())).collect()
    }

    /// Run blocks through the node and collect what reached the bus.
    fn run(node: &mut UatNode, blocks: &[Vec<C32>]) -> Vec<common::Frame> {
        let ins = [spec(2_400_000.0, uat::CHANNEL_HZ)];
        let tags = Vec::new();
        let mut frames = Vec::new();
        for block in blocks {
            let input = Payload::Iq(block.clone());
            let mut out = Payload::Frames(Vec::new());
            let mut events = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Frames(f) = out {
                frames.extend(f);
            }
        }
        frames
    }

    /// A long codeword keyed on the air, read back as the aircraft that sent
    /// it. The point is that the three layers agree about bit order: each
    /// could be self-consistently wrong alone.
    #[test]
    fn a_keyed_frame_becomes_an_aircraft_at_the_right_place() {
        let payload = long_payload();
        let mut coded = payload.clone();
        coded.extend_from_slice(
            &decode::rs::ReedSolomon::new(8, 0x187, 120, 1, 14, 207).encode(&payload),
        );
        let mut node = UatNode::default();
        node.negotiate(&spec(2_400_000.0, uat::CHANNEL_HZ)).unwrap();
        let frames = run(
            &mut node,
            &[noise(16_384, 1, 0.02), on_air(uat::ADSB_SYNC, &coded), noise(16_384, 2, 0.02)],
        );

        assert_eq!(frames.len(), 1, "one frame off the air");
        assert_eq!(frames[0].bytes, payload, "the 34 bytes the aircraft sent");
        assert!(frames[0].rssi_dbfs.is_finite() && frames[0].snr_db.is_finite());
        assert!(frames[0].iq.is_some(), "the samples it was read from");
        assert_eq!(node.accepted(), 1);

        let rows = Uat
            .read_frame(&packet(&frames[0]), &frames[0].bytes)
            .expect("a frame from the UAT channel");
        assert_eq!(rows.len(), 1);
        let d = &rows[0];
        assert_eq!(d.protocol, "UAT-Position");
        assert_eq!(d.crc_ok, Some(true));
        let get = |k: &str| d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("address"), Some(common::Value::Text("a0dead".into())));
        assert_eq!(get("altitude_ft"), Some(common::Value::Int(9_500)));
        assert_eq!(get("callsign"), Some(common::Value::Text("N172SP".into())));
        let p = d.position.as_ref().expect("a position");
        assert!((p.lat - 40.0).abs() < 0.001 && (p.lon + 105.0).abs() < 0.001);
        assert_eq!(d.identity.as_ref().and_then(|i| i.name.clone()), Some("N172SP".into()));
    }

    /// Minutes of noise, and nothing reaches the bus: a sync word that
    /// matches noise has no codeword behind it.
    #[test]
    fn noise_decodes_to_nothing() {
        let mut node = UatNode::default();
        node.negotiate(&spec(2_400_000.0, uat::CHANNEL_HZ)).unwrap();
        // Two minutes of air at 2.4 MS/s, in blocks of a hundredth of a
        // second.
        let blocks: Vec<Vec<C32>> = (0..600).map(|i| noise(24_000, 1000 + i, 0.3)).collect();
        let frames = run(&mut node, &blocks);
        assert_eq!(frames.len(), 0, "{} frames out of noise", frames.len());
        assert_eq!(node.accepted(), 0);
    }

    /// One wrong byte in the air is inside the code, and the row that comes
    /// out is the one the aircraft sent.
    #[test]
    fn a_frame_with_a_broken_byte_still_corrects() {
        let payload = long_payload();
        let mut coded = payload.clone();
        coded.extend_from_slice(
            &decode::rs::ReedSolomon::new(8, 0x187, 120, 1, 14, 207).encode(&payload),
        );
        coded[5] ^= 0xff;
        coded[20] ^= 0x0f;
        let mut node = UatNode::default();
        node.negotiate(&spec(2_400_000.0, uat::CHANNEL_HZ)).unwrap();
        let frames = run(
            &mut node,
            &[noise(16_384, 3, 0.02), on_air(uat::ADSB_SYNC, &coded), noise(16_384, 4, 0.02)],
        );
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].bytes, payload);
    }

    /// A ground station's uplink: 552 bytes of interleaved codewords, read
    /// back as the station and the one product it sent.
    #[test]
    fn a_keyed_uplink_becomes_a_station_and_a_product() {
        let mut data = vec![0u8; uat::UPLINK_DATA_BYTES];
        let lat = (39.861_67f64 * 16_777_216.0 / 360.0).round() as u32;
        let lon = ((360.0f64 - 104.673) * 16_777_216.0 / 360.0).round() as u32;
        data[0] = (lat >> 15) as u8;
        data[1] = (lat >> 7) as u8;
        data[2] = ((lat << 1) as u8) | ((lon >> 23) as u8 & 1);
        data[3] = (lon >> 15) as u8;
        data[4] = (lon >> 7) as u8;
        data[5] = ((lon << 1) as u8) | 1;
        data[6] = 0x80 | 0x20 | 9;
        data[7] = 2 << 4;
        // Product 51, national NEXRAD: a picture, listed rather than drawn.
        let body = [(51u16 >> 6) as u8, ((51 & 0x3f) << 2) as u8, 16 << 2, 53 << 2, 0xde, 0xad];
        data[8] = (body.len() >> 1) as u8;
        data[9] = (body.len() << 7) as u8;
        data[10..10 + body.len()].copy_from_slice(&body);

        let rs = decode::rs::ReedSolomon::new(8, 0x187, 120, 1, 20, 163);
        let mut coded = vec![0u8; uat::UPLINK_BYTES];
        for block in 0..uat::UPLINK_BLOCKS {
            let from = block * uat::UPLINK_BLOCK_DATA_BYTES;
            let chunk = &data[from..from + uat::UPLINK_BLOCK_DATA_BYTES];
            let mut word = chunk.to_vec();
            word.extend_from_slice(&rs.encode(chunk));
            for (i, b) in word.iter().enumerate() {
                coded[i * uat::UPLINK_BLOCKS + block] = *b;
            }
        }

        let mut node = UatNode::default();
        node.negotiate(&spec(2_400_000.0, uat::CHANNEL_HZ)).unwrap();
        let frames = run(
            &mut node,
            &[noise(16_384, 5, 0.02), on_air(uat::UPLINK_SYNC, &coded), noise(16_384, 6, 0.02)],
        );
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].bytes.len(), uat::UPLINK_DATA_BYTES);

        let rows = Uat.read_frame(&packet(&frames[0]), &frames[0].bytes).expect("an uplink");
        assert_eq!(rows.len(), 2, "the station, and the product it sent");
        assert_eq!(rows[0].protocol, "UAT-Uplink");
        let site = rows[0].position.as_ref().expect("a site position");
        assert!((site.lat - 39.861_67).abs() < 0.001 && (site.lon + 104.673).abs() < 0.001);
        assert_eq!(rows[1].protocol, "FISB");
        let get = |k: &str| rows[1].fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("product"), Some(common::Value::Text("national NEXRAD".into())));
        assert_eq!(get("product_id"), Some(common::Value::Int(51)));
        assert!(!rows[1].written, "a machine composed it and addressed it to nobody");
    }

    fn packet(f: &common::Frame) -> common::Packet {
        common::Packet::of_frame(0, 2_000_000, f.clone())
    }

    /// The aircraft of the decode crate's own test: 40 N, 105 W at 9500
    /// feet, calling itself N172SP.
    fn long_payload() -> Vec<u8> {
        let mut f = vec![0u8; uat::LONG_DATA_BYTES];
        f[0] = 1 << 3;
        f[1] = 0xa0;
        f[2] = 0xde;
        f[3] = 0xad;
        let lat = (40.0f64 * 16_777_216.0 / 360.0).round() as u32;
        let lon = ((360.0f64 - 105.0) * 16_777_216.0 / 360.0).round() as u32;
        f[4] = (lat >> 15) as u8;
        f[5] = (lat >> 7) as u8;
        f[6] = ((lat << 1) as u8) | ((lon >> 23) as u8 & 1);
        f[7] = (lon >> 15) as u8;
        f[8] = (lon >> 7) as u8;
        f[9] = (lon << 1) as u8;
        let alt = ((9_500 + 1000) / 25 + 1) as u16;
        f[10] = (alt >> 4) as u8;
        f[11] = ((alt << 4) as u8) | 8;
        let (ns, ew) = (101u32, 101u32);
        f[12] = (ns >> 6) as u8;
        f[13] = ((ns << 2) as u8) | (ew >> 9) as u8;
        f[14] = (ew >> 1) as u8;
        f[15] = (ew << 7) as u8;
        let vv = 640 / 64 + 1;
        f[15] |= (vv >> 4) as u8;
        f[16] = (vv << 4) as u8;
        // "N172SP", three characters to every two bytes, base 40, the first
        // of them the emitter category.
        for (at, v) in [
            (17, 1u16 * 1600 + 23 * 40 + 1),
            (19, 7 * 1600 + 2 * 40 + 28),
            (21, 25 * 1600 + 36 * 40 + 36),
        ] {
            f[at] = (v >> 8) as u8;
            f[at + 1] = v as u8;
        }
        f[26] = 0x02;
        f
    }
}
