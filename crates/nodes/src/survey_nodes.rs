//! The device database, as a consumer of the packet bus.
//!
//! A survey is a different question from a packet list. The list asks what
//! was transmitted; this asks what is out there, once per thing, with the
//! places it was heard from. Everything it knows comes off the bus, so it
//! behaves the same on live packets and on a day of the packet log replayed
//! through the same graph.
//!
//! # Identity is what a decode already says
//!
//! Nothing new is parsed here. Every protocol that can name a transmitter
//! already puts that name in its decoded fields: `icao` for an aircraft,
//! `mmsi` for a vessel, `address` for a BLE advertiser or a pager, `from` for
//! an amateur callsign or a DMR radio, `id` for the ISM sensors. So the table
//! below is a list of which field carries the identity for which protocol,
//! and adding a protocol to the survey is adding a row to it.
//!
//! That is deliberately not "any field called id". A field name means what
//! its decoder meant by it, and treating a sensor's rolling four-bit channel
//! number as an identity would invent a new device every time a battery was
//! changed. What goes in the database is a field a decoder chose as the
//! transmitter's own identifier.
//!
//! # Where the receiver was
//!
//! A sighting carries the position of the *receiver*, not of the device, and
//! the difference matters enough to be worth repeating wherever this is read.
//! A survey driving past a beacon records a line of positions along a road
//! and the level at each; the beacon is somewhere near the strongest of them
//! and this does not claim to know where.
//!
//! The position comes through [`SurveyNode::set_station`] and is the station
//! position the whole receiver works from, whether a GPS supplied it or an
//! operator typed it in. A survey recorded only where a GPS said it was, so a
//! fixed installation with its position entered by hand wrote every sighting
//! blind.

use common::{Packet, Result};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use survey::{Db, Report, Sighting};

/// Which decoded field carries the transmitter's identity, per protocol
/// prefix, and what to call that identity space in the database.
///
/// Matched on the start of the protocol name because several decoders report
/// a family: `APRS-Position` and `APRS-Status` are one radio, and `AIS-Static`
/// and `AIS-Position` are one vessel.
/// What a decode says about who transmitted it.
///
/// The decoder's own answer, and there is no other: every protocol that
/// names a transmitter says so on the decode. `None` for a decode that identifies nothing: an
/// unclaimed burst, a frame whose protocol has no notion of a transmitter.
/// Those are real receptions and they belong in the packet log, which has
/// them; they are not devices.
pub fn identity(d: &Decoded) -> Option<(String, String)> {
    let who = d.identity.as_ref()?;
    Some((who.space.clone(), who.id.clone()))
}

/// A name a device gave for itself, where its decode carries one.
pub(crate) fn name_of(d: &Decoded) -> Option<String> {
    if let Some(n) = d.identity.as_ref().and_then(|w| w.name.clone()) {
        return Some(n);
    }
    for key in ["name", "callsign", "node_name"] {
        if let Some((_, v)) = d.fields.iter().find(|(k, _)| k == key) {
            let s = v.to_string();
            if !s.is_empty() {
                return Some(s);
            }
        }
    }
    None
}

/// Who made it, where the decode says so.
pub(crate) fn vendor_of(d: &Decoded) -> Option<String> {
    if let Some(v) = d.identity.as_ref().and_then(|w| w.vendor.clone()) {
        return Some(v);
    }
    d.fields
        .iter()
        .find(|(k, _)| k == "vendor" || k == "operator" || k == "manufacturer")
        .map(|(_, v)| v.to_string())
        .filter(|s| !s.is_empty())
}

/// The device database on the bus.
pub struct SurveyNode {
    db: Option<Db>,
    /// Where the receiver is. `None` until somebody says, which is a sighting
    /// with no position rather than no sighting: what was heard is still
    /// evidence.
    station: Option<gps::Fix>,
    heard: u64,
    failures: u64,
}

impl Default for SurveyNode {
    fn default() -> Self {
        Self::new(None)
    }
}

impl SurveyNode {
    /// A survey writing to `db`, or one that records nothing when there is
    /// none. A node with no database still sits in the graph: turning the
    /// survey on is opening a file, not rebuilding the receiver.
    pub fn new(db: Option<Db>) -> Self {
        Self {
            db,
            station: None,
            heard: 0,
            failures: 0,
        }
    }

    pub fn set_db(&mut self, db: Option<Db>) {
        self.db = db;
    }

    pub fn is_recording(&self) -> bool {
        self.db.is_some()
    }

    pub fn db(&self) -> Option<&Db> {
        self.db.as_ref()
    }

    /// Where the receiver is now, as the station position, carrying the
    /// quality fields when a fix supplied it.
    pub fn set_station(&mut self, at: Option<gps::Fix>) {
        self.station = at;
    }

    pub fn station(&self) -> Option<gps::Fix> {
        self.station
    }

    /// Receptions attributed to a device since the node was built, and writes
    /// the database refused.
    pub fn heard(&self) -> u64 {
        self.heard
    }

    pub fn failures(&self) -> u64 {
        self.failures
    }

    fn sighting(&self, p: &Packet, d: &Decoded) -> Sighting {
        let fix = self.station;
        Sighting {
            at_us: p.at_us,
            lat: fix.map(|f| f.lat),
            lon: fix.map(|f| f.lon),
            alt_m: fix.and_then(|f| f.alt_m),
            accuracy_m: fix.and_then(|f| f.accuracy_m()),
            rssi_dbfs: d.rssi_dbfs.or(p.rssi_dbfs().is_finite().then_some(p.rssi_dbfs())),
            snr_db: d.snr_db.or(p.snr_db().is_finite().then_some(p.snr_db())),
            center_hz: d.center.0,
        }
    }
}

impl Simple for SurveyNode {
    fn name(&self) -> &str {
        "survey"
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Packets {
            return Err(common::Error::other("survey reads the packet bus"));
        }
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        if self.db.is_none() {
            return Ok(());
        }
        // What the packet already decoded to, rather than a second run of
        // the same tables: this node used to decode every packet again, so a
        // decoder fix reached the packet list and left the device database
        // reporting last week's conclusions.
        //
        // One burst can decode as more than one protocol, and a survey wants
        // the transmitter rather than the ambiguity: the first decode that
        // names one is taken and the rest of that packet is left to the
        // packet list, which does report all of them.
        for p in i.as_packets().unwrap_or(&[]) {
            for d in p.decodes.iter() {
                let Some((protocol, ident)) = identity(d) else { continue };
                let report = Report {
                    protocol,
                    ident,
                    name: name_of(d),
                    vendor: vendor_of(&d),
                    sighting: self.sighting(p, d),
                };
                self.heard += 1;
                if let Some(db) = self.db.as_mut() {
                    if db.record(&report).is_err() {
                        // A survey that cannot write is a survey that stops
                        // recording, not a receiver that stops receiving. The
                        // count is what the interface shows.
                        self.failures += 1;
                    }
                }
                break;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Hz, Value};

    fn decoded(protocol: &'static str, fields: &[(&str, &str)]) -> Decoded {
        Decoded::bytes(protocol, Hz(2_426_000_000), 0.0, vec![]).with_fields(
            fields.iter().map(|(k, v)| ((*k).to_string(), Value::Text((*v).to_string()))).collect(),
        )
    }

    /// The identity is the decoder's own statement now, whatever the
    /// protocol: a table of "which field holds the identity for which
    /// protocol name" is a table that goes stale the day a decoder is added.
    #[test]
    fn each_protocol_gives_up_the_identity_its_decoder_named() {
        let named = |protocol: &'static str, space: &str, id: &str| {
            Decoded::bytes(protocol, Hz(2_426_000_000), 0.0, vec![])
                .by(common::Identity::new(space, id))
        };
        let cases = [
            (named("BLE-Adv", "ble", "6C:70:CB:EF:72:4D"), "ble", "6C:70:CB:EF:72:4D"),
            (named("ADS-B-Position", "adsb", "4ca1fb"), "adsb", "4ca1fb"),
            (named("AIS-Position", "ais", "235009802"), "ais", "235009802"),
            (named("APRS-Position", "aprs", "EI2ABC-9"), "aprs", "EI2ABC-9"),
            (named("POCSAG-Alpha", "pocsag", "1234568"), "pocsag", "1234568"),
        ];
        for (d, space, ident) in cases {
            assert_eq!(identity(&d), Some((space.into(), ident.into())), "{}", d.protocol);
        }
        // A decode that names nobody is a reception, not a device.
        assert_eq!(identity(&decoded("unknown", &[("baud", "1500")])), None);
    }

    /// A sensor's id is eight bits chosen when the batteries go in, so it is
    /// only an identity together with the model that read it. Built through
    /// the decoder's own report, since that is what decides it now.
    #[test]
    fn an_ism_sensor_is_identified_by_its_model_and_id_together() {
        let report = |model| {
            let r = decode::Report::new(model).int("id", 163);
            crate::decode_nodes::decoded_event(
                &r,
                &common::Package::default(),
                Hz(433_920_000),
                "OOK",
            )
        };
        let a = report("Acurite-Tower");
        let b = report("Nexus-TH");
        assert_ne!(identity(&a), identity(&b), "two makes sharing an id are two devices");
        assert_eq!(identity(&a), Some(("ism:Acurite-Tower".into(), "163".into())));
    }

    /// A burst nothing claimed is a reception, not a device.
    #[test]
    fn a_decode_that_names_nobody_is_not_a_device() {
        assert_eq!(identity(&decoded("unknown", &[("coding", "PWM")])), None);
        assert_eq!(identity(&decoded("BLE-Adv", &[])), None, "a protocol without its field");
    }

    /// What a pager said is not what the pager is called.
    #[test]
    fn a_page_is_not_a_name() {
        let d = decoded("POCSAG-Alpha", &[("address", "1234568"), ("message", "CALL BASE")]);
        assert_eq!(name_of(&d), None);
        let ble = decoded("BLE-Adv", &[("address", "AA:BB"), ("name", "EVCS")]);
        assert_eq!(name_of(&ble).as_deref(), Some("EVCS"));
    }

    fn packet(bytes: Vec<u8>, center_hz: u64) -> Packet {
        Packet::of_frame(
            1_000_000,
            2_000_000,
            common::Frame::measured(bytes, -46.0, 20.0).at(center_hz),
        )
    }

    /// Through the protocols first, as the graph runs it: the survey reads
    /// what a packet decoded to, so a test that hands it bare packets is
    /// testing nothing the receiver does.
    fn annotated(mut packets: Vec<Packet>) -> Vec<Packet> {
        let mut d = crate::PacketDecodeNode::default();
        d.annotate(&mut packets);
        packets
    }

    fn run(node: &mut SurveyNode, packets: Vec<Packet>) {
        let packets = annotated(packets);
        let mut s = pipeline::StreamSpec::iq(0.0, Hz(2_426_000_000));
        s.kind = PortKind::Packets;
        let ins = [PortSpec { spec: s, latency: 0 }];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut out = Payload::Packets(Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        node.process(&Payload::Packets(packets), &mut out, &mut ctx).unwrap();
    }

    /// A real advertising PDU off the bus becomes a device with the position
    /// the receiver was at when it heard it.
    #[test]
    fn a_ble_advertisement_becomes_a_device_at_the_receivers_position() {
        let mut node = SurveyNode::new(Some(Db::in_memory().unwrap()));
        node.set_station(Some(gps::Fix {
            lat: 53.6369,
            lon: -6.6528,
            hdop: Some(0.9),
            ..Default::default()
        }));
        // A Samsung monitor's ADV_IND, dewhitened and CRC checked.
        let pdu = vec![
            0x00, 0x11, 0x3a, 0xf5, 0x0a, 0xcd, 0x31, 0xe8, 0x02, 0x01, 0x06, 0x07, 0xff, 0xe1,
            0x02, 0x10, 0x00, 0x26, 0xc0,
        ];
        run(&mut node, vec![packet(pdu, 2_426_000_000)]);
        let db = node.db().expect("a database");
        let rows = db.devices(survey::Query::default()).unwrap();
        assert_eq!(rows.len(), 1, "expected one device, got {rows:?}");
        assert_eq!(rows[0].protocol, "ble");
        assert_eq!(rows[0].ident, "E8:31:CD:0A:F5:3A");
        let s = &db.sightings(rows[0].id).unwrap()[0];
        assert_eq!((s.lat, s.lon), (Some(53.6369), Some(-6.6528)));
        assert_eq!(s.rssi_dbfs, Some(-46.0));
        assert_eq!(s.accuracy_m, Some(4.5), "HDOP times the receiver's own error");
    }

    /// A cell becomes a device, by the identity it is known by everywhere
    /// rather than by the channel it was heard on: a network moves a cell to
    /// another carrier without it becoming a different cell.
    #[test]
    fn a_broadcast_block_becomes_a_cell_in_the_survey() {
        let mut node = SurveyNode::new(Some(Db::in_memory().unwrap()));
        // A system information type 3 for the test network 001-01, location
        // area 1, cell 1.
        let mut block = vec![0x49, 0x06, 0x1B, 0x00, 0x01, 0x00, 0xF1, 0x10, 0x00, 0x01];
        block.resize(23, 0x2B);
        run(&mut node, vec![packet(block, 947_400_000)]);
        let db = node.db().expect("a database");
        let rows = db.devices(survey::Query::default()).unwrap();
        assert_eq!(rows.len(), 1, "expected one cell, got {rows:?}");
        assert_eq!(rows[0].protocol, "gsm");
        assert_eq!(rows[0].ident, "001-01-1-1");
    }

    /// Indoors, or before the first lock, there is no position. What was
    /// heard is still recorded.
    #[test]
    fn without_a_fix_a_sighting_is_still_recorded() {
        let mut node = SurveyNode::new(Some(Db::in_memory().unwrap()));
        let pdu = vec![
            0x00, 0x11, 0x3a, 0xf5, 0x0a, 0xcd, 0x31, 0xe8, 0x02, 0x01, 0x06, 0x07, 0xff, 0xe1,
            0x02, 0x10, 0x00, 0x26, 0xc0,
        ];
        run(&mut node, vec![packet(pdu, 2_426_000_000)]);
        let db = node.db().unwrap();
        assert_eq!(db.counts().unwrap(), (1, 1));
        assert_eq!(db.sightings(1).unwrap()[0].lat, None);
    }

    /// With no database the node is inert, and the graph is the same graph.
    #[test]
    fn a_survey_with_nowhere_to_write_records_nothing() {
        let mut node = SurveyNode::default();
        run(&mut node, vec![packet(vec![0x00, 0x11, 0x3a], 2_426_000_000)]);
        assert!(!node.is_recording());
        assert_eq!(node.heard(), 0);
    }
}
