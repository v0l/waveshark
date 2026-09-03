//! A TETRA downlink carrier as a graph node.
//!
//! The same shape as the M17 front end: complex baseband in, packets out.
//! `dsp::tetra` demodulates the pi/4-DQPSK, walks the slots and runs the
//! channel coding; `decode::tetra` reads the PDUs. What reaches the bus is
//! the cell's identity rather than every burst: a base station repeats its
//! SYNC PDU seventeen times a second for years, and seventeen identical rows
//! a second is a log nobody can read. A row is emitted when what the cell
//! says changes, which for a healthy carrier is once; and for every call
//! control PDU, which is what the call list is built from. On a network
//! that enciphers the air interface there are no call control PDUs to read,
//! only MAC headers naming who is addressed, and those are reported once per
//! address every couple of seconds: enough to say a group is busy, not so
//! many that the list is a scroll of one group.

use common::Result;
use decode::tetra::{Address, CallPdu, Event, RESOURCE, TRAFFIC, TRAFFIC_END};
use std::collections::HashMap;
use dsp::tetra::{Block, Burst, TetraConfig, TetraDemod, TetraRx, OCCUPIED_HZ};
use dsp::{FirDecim, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// The raster TETRA carriers sit on.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

/// The least stream rate worth building the demodulator for. The source
/// extractor's floor of 25 kS/s clears it; the occupied signal only just
/// fits there, and the demodulator's tests show it still reads.
pub const MIN_RATE_HZ: f64 = OCCUPIED_HZ;

/// Rate the demodulator likes to run at: four samples a symbol.
const DEMOD_HZ: f64 = 72_000.0;

/// Slots between rows for one address that keeps being addressed with
/// nothing readable: about two seconds, at 85/6 ms a slot.
const RESOURCE_EVERY_SLOTS: u64 = 140;

/// Slots a traffic channel's marker may go unseen before its traffic is
/// taken to have ended: a frame is four slots and the access assign field
/// for one timeslot comes once a frame, so this is three frames of it
/// missing, or the demodulator losing lock for that long.
const TRAFFIC_HANG_SLOTS: u64 = 12;

/// Seconds a slot is: 255 symbols at 18 kbaud.
const SLOT_S: f64 = 255.0 / 18_000.0;

/// Frames a marker has to be seen on before its traffic is reported: one
/// misread field must not open a call.
const TRAFFIC_CONFIRM: u32 = 2;

/// Traffic running on one timeslot, as the access assign field shows it.
struct Traffic {
    marker: u8,
    since: u64,
    last: u64,
    /// Frames the marker has been seen on, and whether that was enough to
    /// have been reported.
    frames: u32,
    reported: bool,
}

pub struct TetraNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    demod: TetraDemod,
    rx: TetraRx,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    bursts: Vec<Burst>,
    blocks: Vec<Block>,
    /// The last identity and broadcast reported, so a repeat is not a row.
    last_sync: Option<Vec<u8>>,
    last_sysinfo: Option<Vec<u8>>,
    last_network: Option<Vec<u8>>,
    /// The encryption mode the cell's signalling last carried, which is
    /// what its traffic carries too.
    last_aie: u8,
    /// When each address was last reported as a bare resource, by slot.
    resource_seen: HashMap<u32, u64>,
    /// The cell's frequency band and offset from its SYSINFO, which is what
    /// a carrier number in an allocation or a neighbour is relative to.
    cell_band: Option<(u8, u8)>,
    /// Which party each usage marker was given to, from the addresses that
    /// carried one.
    markers: HashMap<u8, u32>,
    /// Traffic on each timeslot right now, by timeslot number.
    traffic: HashMap<u8, Traffic>,
    /// The slot counter as of the last burst, for reaping traffic whose
    /// marker stopped appearing.
    slot_now: u64,
    accepted: u64,
}

impl Default for TetraNode {
    fn default() -> Self {
        Self::new(390_000_000.0)
    }
}

impl TetraNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(DEMOD_HZ, 1, OCCUPIED_HZ / 2.0, 60.0),
            demod: TetraDemod::new(DEMOD_HZ, TetraConfig::default()),
            rx: TetraRx::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            bursts: Vec::new(),
            blocks: Vec::new(),
            last_sync: None,
            last_sysinfo: None,
            last_network: None,
            last_aie: 0,
            resource_seen: HashMap::new(),
            cell_band: None,
            markers: HashMap::new(),
            traffic: HashMap::new(),
            slot_now: 0,
            accepted: 0,
        }
    }

    pub fn channel_hz(&self) -> f64 {
        self.channel_hz
    }

    /// Rows reported since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    /// The cell as the lower MAC currently believes it, once one SYNC PDU
    /// has decoded.
    pub fn cell(&self) -> Option<dsp::tetra::Cell> {
        self.rx.cell
    }

    /// Whether a call PDU is news. Call control always is; a resource whose
    /// SDU could not be read is, for an enciphered address, once every
    /// [`RESOURCE_EVERY_SLOTS`], and for anything else never: a clear PDU of
    /// another protocol is not a call.
    fn worth_a_row(&mut self, c: &CallPdu, slot: u64) -> bool {
        if c.pdu != RESOURCE {
            return true;
        }
        if c.aie == 0 {
            return false;
        }
        // All ones is the address of nobody, which the idle filler carries.
        let Some(ssi) = c.address.ssi().filter(|s| *s != 0 && *s != 0xff_ffff) else { return false };
        // An allocation is news whenever it comes: it says where the
        // traffic is going.
        if c.alloc.is_none()
            && self.resource_seen.get(&ssi).is_some_and(|last| slot.saturating_sub(*last) < RESOURCE_EVERY_SLOTS)
        {
            return false;
        }
        self.resource_seen.insert(ssi, slot);
        true
    }

    /// A traffic event for a marker, addressed to the party it was given to
    /// when that is known and to the marker itself otherwise.
    fn traffic_event(&self, pdu: u8, tn: u8, marker: u8, seconds: f32, slot: u64) -> Event {
        let address = match self.markers.get(&marker) {
            Some(ssi) => Address::Ssi(*ssi),
            None => Address::UsageMarker(marker),
        };
        let time = self.rx.time_at(slot).map(|mut t| {
            t.tn = tn;
            t
        });
        Event::Call(CallPdu {
            pdu,
            address,
            // The traffic itself is enciphered whenever its signalling is;
            // a cell that encrypts encrypts everything.
            aie: self.last_aie,
            e2e: None,
            call_id: None,
            from: None,
            group: None,
            time,
            alloc: None,
            marker: Some(marker),
            seconds,
            text: None,
        })
    }

    /// What one slot's access assign field says about the traffic on it,
    /// turned into events when that changes.
    fn follow_traffic(&mut self, a: &decode::tetra::AachPdu, slot: u64, out: &mut Vec<Event>) {
        let Some(t) = a.time else { return };
        let tn = t.tn;
        match (a.traffic_marker(), self.traffic.get_mut(&tn)) {
            (Some(m), Some(run)) if run.marker == m => {
                run.last = slot;
                run.frames += 1;
                if !run.reported && run.frames >= TRAFFIC_CONFIRM {
                    run.reported = true;
                    let (since, marker) = (run.since, run.marker);
                    out.push(self.traffic_event(TRAFFIC, tn, marker, 0.0, since));
                }
            }
            (Some(m), _) => {
                self.end_traffic(tn, slot, out);
                self.traffic.insert(
                    tn,
                    Traffic { marker: m, since: slot, last: slot, frames: 1, reported: false },
                );
            }
            (None, Some(_)) if a.dl_usage.is_some() => self.end_traffic(tn, slot, out),
            _ => {}
        }
    }

    /// Close the traffic on a timeslot, reporting it if it was ever
    /// reported as started.
    fn end_traffic(&mut self, tn: u8, slot: u64, out: &mut Vec<Event>) {
        let Some(run) = self.traffic.remove(&tn) else { return };
        if !run.reported {
            return;
        }
        let secs = (run.last.saturating_sub(run.since) as f64 * SLOT_S) as f32;
        out.push(self.traffic_event(TRAFFIC_END, tn, run.marker, secs, slot));
    }

    /// Traffic whose marker has not been seen for a while has ended, lock
    /// lost or not.
    fn reap_traffic(&mut self, out: &mut Vec<Event>) {
        let stale: Vec<u8> = self
            .traffic
            .iter()
            .filter(|(_, r)| self.slot_now.saturating_sub(r.last) > TRAFFIC_HANG_SLOTS)
            .map(|(tn, _)| *tn)
            .collect();
        for tn in stale {
            let last = self.traffic[&tn].last;
            self.end_traffic(tn, last, out);
        }
    }
}

impl Node for TetraNode {
    fn name(&self) -> &str {
        "tetra"
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        1
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("tetra reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if rate < MIN_RATE_HZ {
            return Err(common::Error::other("tetra needs the width of its carrier"));
        }
        if (self.channel_hz - center).abs() > (rate - OCCUPIED_HZ) / 2.0 {
            return Err(common::Error::other("tetra needs its carrier inside the span"));
        }
        let factor = (rate / DEMOD_HZ).round().max(1.0) as usize;
        let demod_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, OCCUPIED_HZ / 2.0, 60.0);
        self.demod = TetraDemod::new(demod_rate, TetraConfig::default());
        self.rx = TetraRx::new();

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        out.rate = 0.0;
        Ok(vec![out])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        _c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(iq) = inputs[0].as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);

        self.bursts.clear();
        let narrow = std::mem::take(&mut self.narrow);
        let mut bursts = std::mem::take(&mut self.bursts);
        self.demod.process(&narrow, &mut bursts);
        self.narrow = narrow;

        self.blocks.clear();
        let mut blocks = std::mem::take(&mut self.blocks);
        for b in &bursts {
            self.rx.push(b, &mut blocks);
        }
        self.bursts = bursts;

        let at_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0);
        let out = outputs[0].packets_mut();
        // Every block's event, then the traffic the access assign fields
        // describe, as events of the node's own.
        let mut events: Vec<(Event, u64)> = Vec::new();
        for block in &blocks {
            self.slot_now = self.slot_now.max(block.slot);
            let Some(event) = Event::from_block(block) else { continue };
            match event {
                Event::Aach(a) => {
                    let mut made = Vec::new();
                    self.follow_traffic(&a, block.slot, &mut made);
                    events.extend(made.into_iter().map(|e| (e, block.slot)));
                }
                Event::Sysinfo(si) => {
                    self.cell_band = Some((si.freq_band, si.freq_offset));
                    events.push((Event::Sysinfo(si), block.slot));
                }
                Event::Call(mut c) => {
                    if let (Some(m), Some(ssi)) = (c.marker, c.address.ssi()) {
                        self.markers.insert(m, ssi);
                    }
                    if c.aie != 0 {
                        self.last_aie = c.aie;
                    }
                    if let (Some(a), Some(band)) = (c.alloc.as_mut(), self.cell_band) {
                        a.band.get_or_insert(band);
                    }
                    events.push((Event::Call(c), block.slot));
                }
                Event::Network(mut n) => {
                    if let Some(band) = self.cell_band {
                        for nb in &mut n.neighbours {
                            nb.band.get_or_insert(band);
                        }
                    }
                    events.push((Event::Network(n), block.slot));
                }
                other => events.push((other, block.slot)),
            }
        }
        let mut reaped = Vec::new();
        self.reap_traffic(&mut reaped);
        events.extend(reaped.into_iter().map(|e| (e, self.slot_now)));

        for (event, slot) in events {
            let bytes = event.to_bytes();
            // The repeat test ignores the cell's clock: a SYNC PDU differs
            // every frame by its frame number alone, and that is not news.
            let (seen, key) = match &event {
                Event::Sync(_) => (&mut self.last_sync, Event::identity_key(&bytes)),
                Event::Sysinfo(_) => (&mut self.last_sysinfo, Event::identity_key(&bytes)),
                Event::Network(_) => (&mut self.last_network, Event::identity_key(&bytes)),
                Event::Call(c) if matches!(c.pdu, TRAFFIC | TRAFFIC_END) => (&mut None, None),
                Event::Call(c) => {
                    if !self.worth_a_row(c, slot) {
                        continue;
                    }
                    (&mut None, None)
                }
                Event::Aach(_) => continue,
            };
            if let Some(key) = key {
                if seen.as_deref() == Some(&key[..]) {
                    continue;
                }
                *seen = Some(key);
            }
            self.accepted += 1;
            out.push(common::Packet {
                at_us,
                center_hz: self.channel_hz as u64,
                bandwidth_hz: CHANNEL_WIDTH_HZ as u32,
                rssi_dbfs: f32::NAN,
                snr_db: f32::NAN,
                modulation: Some("pi/4-DQPSK"),
                body: common::PacketBody::Frame(bytes),
                iq: None,
                audio: None,
                measure: None,
            });
        }
        self.blocks = blocks;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.demod.reset();
        self.rx = TetraRx::new();
        self.last_sync = None;
        self.last_sysinfo = None;
        self.last_network = None;
        self.resource_seen.clear();
        self.markers.clear();
        self.traffic.clear();
    }
}

/// The row a TETRA broadcast becomes.
pub fn tetra_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let event = Event::parse(bytes)?;
    let mut fields: Vec<(String, Value)> = Vec::new();
    let protocol = match &event {
        Event::Sync(s) => {
            fields.push(("mcc".into(), Value::Int(s.mcc.into())));
            fields.push(("mnc".into(), Value::Int(s.mnc.into())));
            fields.push(("colour".into(), Value::Int(s.colour.into())));
            fields.push(("frame".into(), Value::Int(s.frame.into())));
            fields.push(("multiframe".into(), Value::Int(s.multiframe.into())));
            if s.sharing_mode != 0 {
                fields.push(("sharing".into(), Value::Int(s.sharing_mode.into())));
            }
            "TETRA-Sync"
        }
        Event::Sysinfo(s) => {
            fields.push(("carrier_hz".into(), Value::Float(s.downlink_hz())));
            fields.push(("la".into(), Value::Int(s.la.into())));
            fields.push(("subscriber_class".into(), Value::Int(s.subscriber_class.into())));
            fields.push(("service_details".into(), Value::Int(s.bs_service_details.into())));
            "TETRA-Sysinfo"
        }
        // Named the way the call list reads a decode: `to` and `from` are
        // the parties, `call_type` says group or private where the PDU
        // said, `encryption` is what protects the traffic, `seconds` is
        // how long an over ran and `live` that it is still running.
        Event::Call(c) => {
            fields.push(("pdu".into(), Value::Text(c.name().into())));
            match c.address {
                Address::Ssi(s) | Address::Ussi(s) => {
                    fields.push(("to".into(), Value::Text(s.to_string())));
                }
                Address::UsageMarker(m) => {
                    fields.push(("to".into(), Value::Text(format!("marker {m}"))));
                }
                Address::Smi(s) => fields.push(("smi".into(), Value::Int(s.into()))),
                Address::EventLabel(e) => fields.push(("event_label".into(), Value::Int(e.into()))),
            }
            if let Some(f) = c.from {
                fields.push(("from".into(), Value::Text(f.to_string())));
            }
            if let Some(id) = c.call_id {
                fields.push(("call_id".into(), Value::Int(id.into())));
            }
            if let Some(g) = c.group {
                fields.push(("call_type".into(), Value::Text(if g { "group" } else { "private" }.into())));
            }
            fields.push(("encryption".into(), Value::Text(c.encryption())));
            if let Some(m) = c.marker {
                fields.push(("marker".into(), Value::Int(m.into())));
            }
            if let Some(a) = c.alloc {
                if let Some(band) = a.band {
                    fields.push(("traffic_hz".into(), Value::Float(a.hz(band))));
                }
                fields.push(("timeslot".into(), Value::Int(a.timeslot.into())));
            }
            // Traffic is on the timeslot it was seen on; signalling names
            // the timeslot it was heard on separately, below, since that
            // is the control channel and not the call's.
            if matches!(c.pdu, TRAFFIC | TRAFFIC_END) {
                if let Some(t) = c.time {
                    fields.push(("timeslot".into(), Value::Int(t.tn.into())));
                }
            }
            if c.pdu == TRAFFIC {
                fields.push(("live".into(), Value::Bool(true)));
            }
            if c.pdu == TRAFFIC_END {
                fields.push(("seconds".into(), Value::Float(f64::from(c.seconds))));
            }
            if let Some(text) = &c.text {
                fields.push(("text".into(), Value::Text(text.clone())));
            }
            if let Some(t) = c.time {
                fields.push(("slot".into(), Value::Int(t.tn.into())));
                fields.push(("frame".into(), Value::Int(t.frame.into())));
            }
            if c.text.is_some() {
                "TETRA-SDS"
            } else {
                "TETRA-Call"
            }
        }
        Event::Network(n) => {
            fields.push(("neighbours".into(), Value::Int(n.neighbours.len() as i64)));
            for nb in &n.neighbours {
                let hz = nb.band.map(|b| nb.hz(b));
                let mut s = match hz {
                    Some(hz) => format!("cell {} at {:.4} MHz", nb.cell_id, hz / 1e6),
                    None => format!("cell {} carrier {}", nb.cell_id, nb.carrier),
                };
                if let Some(la) = nb.la {
                    s.push_str(&format!(" LA {la}"));
                }
                fields.push((format!("cell_{}", nb.cell_id), Value::Text(s)));
            }
            "TETRA-Network"
        }
        Event::Aach(_) => return None,
    };
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    Some(
        Decoded::bytes(protocol, center, 0.0, bytes.to_vec())
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation("pi/4-DQPSK")
            // Every block behind an event passed the CRC the standard puts
            // on it; a burst that failed never became a block.
            .with_crc(Some(true)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use dsp::tetra::{coding, synth, SLOT_BITS};

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_stream_without_its_carrier() {
        let mut n = TetraNode::new(390_000_000.0);
        assert!(n.negotiate(&[spec(2_400_000.0, 390_000_000.0)]).is_ok());
        assert!(n.negotiate(&[spec(2_400_000.0, 420_000_000.0)]).is_err());
        // The extractor's floor rate holds the occupied signal, barely.
        assert!(n.negotiate(&[spec(25_000.0, 390_000_000.0)]).is_ok());
        assert!(n.negotiate(&[spec(20_000.0, 390_000_000.0)]).is_err());
    }

    fn put(bits: &mut [u8], at: usize, n: usize, v: u32) {
        for i in 0..n {
            bits[at + i] = ((v >> (n - 1 - i)) & 1) as u8;
        }
    }

    /// A downlink whose SYNC PDU says Ireland, decoded off synthetic RF
    /// through the whole node: mixer, decimator, demodulator, lower MAC,
    /// upper MAC, one row.
    #[test]
    fn a_cell_becomes_one_row_not_seventeen_a_second() {
        let (rate, hz) = (300_000.0, 390_000_000.0);
        let mut pdu = vec![0u8; 60];
        put(&mut pdu, 4, 6, 7);
        put(&mut pdu, 12, 5, 3);
        put(&mut pdu, 31, 10, 272);
        put(&mut pdu, 41, 14, 91);
        let sb1 = coding::encode_block(&coding::BLK_BSCH, coding::SCRAMB_INIT, &pdu);
        let scramb = coding::scramb_init(272, 91, 7);

        // A SYSINFO broadcast in block 2: carrier 3600 in band 3 with a
        // +6.25 kHz offset is 390.00625 MHz.
        let mut si = vec![0u8; 124];
        put(&mut si, 0, 2, 0b10);
        put(&mut si, 4, 12, 3_600);
        put(&mut si, 16, 4, 3);
        put(&mut si, 20, 2, 1);
        put(&mut si, 82, 14, 4321);
        let bkn2 = coding::encode_block(&coding::BLK_HALF, scramb, &si);
        let burst = synth::sync_burst(&sb1, &[0; 30], &bkn2);

        let mut bits = Vec::new();
        for _ in 0..40 {
            bits.extend_from_slice(&burst);
        }
        assert_eq!(bits.len() % SLOT_BITS, 0);
        let iq = synth::modulate(&bits, rate, 750.0);

        let mut node = TetraNode::new(hz);
        node.negotiate(&[spec(rate, hz)]).unwrap();
        let ins = [spec(rate, hz)];
        let tags = Vec::new();
        let mut rows = Vec::new();
        for chunk in iq.chunks(16_384) {
            let input = Payload::Iq(chunk.to_vec());
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&[&input], std::slice::from_mut(&mut out), &mut ctx).unwrap();
            if let Payload::Packets(ps) = out {
                for p in ps {
                    if let common::PacketBody::Frame(b) = p.body {
                        rows.push(tetra_decoded(&b, Hz(hz as u64)).unwrap());
                    }
                }
            }
        }

        // Forty repeats of the same broadcast are two rows: one identity,
        // one system broadcast.
        assert_eq!(rows.len(), 2, "{rows:?}");
        let sync = rows.iter().find(|r| r.protocol == "TETRA-Sync").expect("no sync row");
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone())
        };
        assert_eq!(get(sync, "mcc"), Some(common::Value::Int(272)));
        assert_eq!(get(sync, "mnc"), Some(common::Value::Int(91)));
        assert_eq!(get(sync, "colour"), Some(common::Value::Int(7)));
        let si = rows.iter().find(|r| r.protocol == "TETRA-Sysinfo").expect("no sysinfo row");
        assert_eq!(get(si, "carrier_hz"), Some(common::Value::Float(390_006_250.0)));
        assert_eq!(get(si, "la"), Some(common::Value::Int(4321)));
        assert_eq!(node.cell().map(|c| (c.mcc, c.mnc)), Some((272, 91)));
    }

    /// The access assign field of a burst, coded and scrambled as the cell
    /// sends it.
    fn aach(scramb: u32, header: u32, field1: u32, field2: u32) -> [u8; 30] {
        let info = ((header << 12) | (field1 << 6) | field2) as u16;
        let word = coding::rm3014_encode(info);
        let mut bb = [0u8; 30];
        for (i, b) in bb.iter_mut().enumerate() {
            *b = ((word >> (29 - i)) & 1) as u8;
        }
        coding::scramble(scramb, &mut bb);
        bb
    }

    /// Traffic on one timeslot, seen only through the access assign field,
    /// becomes a start row and an end row with the airtime between them:
    /// what a call on an encrypting network leaves readable.
    #[test]
    fn traffic_on_a_slot_is_a_call_with_its_airtime() {
        let (rate, hz) = (300_000.0, 390_000_000.0);
        let scramb = coding::scramb_init(272, 91, 7);
        let mut bits = Vec::new();
        let frames = 40u32;
        for frame in 1..=frames {
            // Slot 1: the sync burst, saying which frame this is.
            let mut pdu = vec![0u8; 60];
            put(&mut pdu, 4, 6, 7);
            put(&mut pdu, 10, 2, 0);
            put(&mut pdu, 12, 5, frame % 18 + 1);
            put(&mut pdu, 17, 6, frame / 18 + 1);
            put(&mut pdu, 31, 10, 272);
            put(&mut pdu, 41, 14, 91);
            let sb1 = coding::encode_block(&coding::BLK_BSCH, coding::SCRAMB_INIT, &pdu);
            let bkn2 = coding::encode_block(&coding::BLK_HALF, scramb, &vec![0u8; 124]);
            bits.extend_from_slice(&synth::sync_burst(&sb1, &aach(scramb, 0, 10, 10), &bkn2));
            // Slots 2 to 4: normal bursts; slot 2 carries usage marker 23
            // for the first thirty frames, then falls idle.
            let full = coding::encode_block(&coding::BLK_FULL, scramb, &vec![0u8; 268]);
            for tn in 2..=4 {
                let bb = if tn == 2 && frame <= 30 {
                    aach(scramb, 3, 23, 0)
                } else {
                    aach(scramb, 3, 0, 0)
                };
                bits.extend_from_slice(&synth::normal_burst(&full[..216], &bb, &full[216..], false));
            }
        }
        let iq = synth::modulate(&bits, rate, 750.0);

        let mut node = TetraNode::new(hz);
        node.negotiate(&[spec(rate, hz)]).unwrap();
        let ins = [spec(rate, hz)];
        let tags = Vec::new();
        let mut rows = Vec::new();
        for chunk in iq.chunks(16_384) {
            let input = Payload::Iq(chunk.to_vec());
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&[&input], std::slice::from_mut(&mut out), &mut ctx).unwrap();
            if let Payload::Packets(ps) = out {
                for p in ps {
                    if let common::PacketBody::Frame(b) = p.body {
                        rows.push(tetra_decoded(&b, Hz(hz as u64)).unwrap());
                    }
                }
            }
        }
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string())
        };
        let traffic: Vec<&Decoded> = rows.iter().filter(|r| r.protocol == "TETRA-Call").collect();
        let names: Vec<String> = traffic.iter().filter_map(|r| get(r, "pdu")).collect();
        assert_eq!(names, ["TRAFFIC", "TRAFFIC END"], "{rows:?}");
        assert_eq!(get(traffic[0], "to").as_deref(), Some("marker 23"));
        assert_eq!(get(traffic[0], "timeslot").as_deref(), Some("2"));
        assert_eq!(get(traffic[0], "live").as_deref(), Some("true"));
        // Twenty-nine frames of four slots between the first and the last
        // frame the marker was seen on.
        let secs: f64 = get(traffic[1], "seconds").unwrap().parse().unwrap();
        let want = 29.0 * 4.0 * 255.0 / 18_000.0;
        assert!((secs - want).abs() < 0.2, "{secs} s of traffic, wanted about {want:.2}");
    }
}
