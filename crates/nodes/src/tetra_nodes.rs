//! A TETRA downlink carrier as a graph node.
//!
//! The same shape as the M17 front end: complex baseband in, packets out.
//! `dsp::tetra` demodulates the pi/4-DQPSK, walks the slots and runs the
//! channel coding; `decode::tetra` reads the PDUs. What reaches the bus is
//! the cell's identity rather than every burst: a base station repeats its
//! SYNC PDU seventeen times a second for years, and seventeen identical rows
//! a second is a log nobody can read. A row is emitted when what the cell
//! says changes, which for a healthy carrier is once.

use common::Result;
use decode::tetra::Event;
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
        for block in &blocks {
            let Some(event) = Event::from_block(block) else { continue };
            let bytes = event.to_bytes();
            // The repeat test ignores the cell's clock: a SYNC PDU differs
            // every frame by its frame number alone, and that is not news.
            let (seen, key) = match event {
                Event::Sync(mut s) => {
                    s.timeslot = 0;
                    s.frame = 0;
                    s.multiframe = 0;
                    (&mut self.last_sync, Event::Sync(s).to_bytes())
                }
                Event::Sysinfo(_) => (&mut self.last_sysinfo, bytes.clone()),
            };
            if seen.as_deref() == Some(&key[..]) {
                continue;
            }
            *seen = Some(key);
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
}
