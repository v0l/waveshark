use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape, Stickiness};
use common::Result;
use dsp::lte::scan::Scan;
use dsp::lte::{Heard, bands};
use identify::Signal;
pub use identify::lte::{DEFAULT_HZ, Lte, MIN_RATE_HZ, NARROWEST_HZ, WIDEST_HZ, WIDTHS_HZ};
use pipeline::Request;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub struct LteNode {
    seed: Option<f64>,
    scan: Option<Scan>,
    heard: Vec<(f64, Heard)>,
    claimed: Vec<(f64, f64)>,
    locked: bool,
    cells: std::collections::BTreeMap<u16, (f32, f32)>,
}

impl LteNode {
    pub fn new(seed: Option<f64>) -> Self {
        Self {
            seed: seed.map(bands::on_raster),
            scan: None,
            heard: Vec::new(),
            claimed: Vec::new(),
            locked: false,
            cells: Default::default(),
        }
    }
}

impl Simple for LteNode {
    fn name(&self) -> &str {
        "lte"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("lte reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        let Some(scan) = Scan::new(rate, center, self.seed) else {
            return Err(common::Error::other(
                "lte needs 1.92 MS/s for its synchronisation signals",
            ));
        };
        self.scan = Some(scan);
        Ok(i.spec.with_kind(PortKind::Packets))
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let (Payload::Iq(iq), Some(scan)) = (i, self.scan.as_mut()) else { return Ok(()) };
        self.heard.clear();
        scan.push(iq, &mut self.heard);
        for (channel, h) in &self.heard {
            if let Some(rx) = scan.receiver_mut(*channel) {
                identify::lte::follow(rx, h);
            }
        }
        let held = scan.held();
        if held != self.claimed {
            if self.claimed.iter().any(|b| !held.contains(b)) {
                c.request(Request::Release);
                self.claimed.clear();
            }
            for &(lo_hz, hi_hz) in held.iter().filter(|b| !self.claimed.contains(b)) {
                c.request(Request::Claim { lo_hz, hi_hz });
            }
            self.claimed = held;
        }
        let out = o.packets_mut();
        for (channel, h) in self.heard.drain(..) {
            let (pci, offset_hz, width_hz, snr_db, rssi_dbfs, rsrp, rsrq, rate) = match &h {
                Heard::Mib {
                    pci,
                    offset_hz,
                    width_hz,
                    snr_db,
                    rssi_dbfs,
                    rsrp_dbfs,
                    rsrq_db,
                    rate,
                    ..
                }
                | Heard::SystemInformation {
                    pci,
                    offset_hz,
                    width_hz,
                    snr_db,
                    rssi_dbfs,
                    rsrp_dbfs,
                    rsrq_db,
                    rate,
                    ..
                } => {
                    (*pci, *offset_hz, *width_hz, *snr_db, *rssi_dbfs, *rsrp_dbfs, *rsrq_db, *rate)
                }
            };
            self.locked = true;
            self.cells.insert(pci, (rsrp, rsrq));
            let carrier = bands::on_raster(channel + offset_hz) as u64;
            let bytes = identify::lte::bytes_of(&h);
            let samples = match h {
                Heard::Mib { samples, .. } | Heard::SystemInformation { samples, .. } => samples,
            };
            let mut p = crate::measured(carrier, width_hz as u32, bytes, rssi_dbfs, snr_db)
                .keyed(common::packet::Keying::configured(common::Modulation::Ofdm));
            p.carrier.iq =
                Some(std::sync::Arc::new(common::IqBurst { rate, center_hz: carrier, samples }));
            out.push(p.checked(common::packet::Integrity::Passed));
        }
        Ok(())
    }

    fn acquisition(&self) -> Option<pipeline::Acquisition> {
        Some(if self.locked {
            pipeline::Acquisition::Locked
        } else {
            pipeline::Acquisition::Searching
        })
    }

    fn readings(&self) -> Vec<(String, String)> {
        self.cells
            .iter()
            .map(|(pci, (rsrp, rsrq))| {
                (format!("PCI {pci}"), format!("RSRP {rsrp:.1} dBFS, RSRQ {rsrq:.1} dB"))
            })
            .collect()
    }

    fn reset(&mut self) {
        if let Some(scan) = self.scan.as_mut() {
            scan.reset();
        }
        self.heard.clear();
        self.locked = false;
        self.cells.clear();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float(CHANNEL_HZ, self.seed.unwrap_or(0.0), 0.0..=6_000e6)
                .unit("Hz")
                .label("The carrier to look at first, or 0 to find them all"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            CHANNEL_HZ => self.seed = v.as_f64().filter(|hz| *hz > 0.0).map(bands::on_raster),
            _ => return Err(common::Error::other(format!("lte: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

fn earfcn(hz: f64) -> Option<u32> {
    bands::containing(hz)?.earfcn(hz)
}

impl Protocol for Lte {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::Continuously
    }

    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
    }
    fn aliases(&self) -> &'static [&'static str] {
        Signal::aliases(self)
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

    fn accepts_width(&self, _hz: f64, source_width_hz: f64) -> bool {
        (NARROWEST_HZ..=WIDEST_HZ * 1.25).contains(&source_width_hz)
    }

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn keys(&self) -> Option<common::Modulation> {
        Some(common::Modulation::Ofdm)
    }

    fn stickiness(&self) -> Stickiness {
        Stickiness::Claim
    }

    fn claims_exactly(&self) -> bool {
        true
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        decode::lte::read(p.bytes())
    }
    fn dedupe_key(&self, p: &common::packet::Packet) -> Option<Vec<u8>> {
        decode::lte::repeat_key(p.bytes())
    }

    fn stage_label(&self, hz: f64) -> String {
        match earfcn(hz) {
            Some(n) => format!("EARFCN {n}"),
            None => format!("{:.1} LTE", hz / 1e6),
        }
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        let label = earfcn(hz).map_or("LTE".into(), |n| format!("LTE {n}"));
        vec![Mark { hz: bands::on_raster(hz), width_hz: WIDTHS_HZ[0], label }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, bands::on_raster(at.center_hz))]
    }
}

const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "lte",
    summary: "One LTE downlink carrier: the cells on it, their MIB and SIB1",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(LteNode::new(Some(s.f64_or(CHANNEL_HZ, 0.0)).filter(|hz| *hz > 0.0))))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(rate: f64, hz: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, common::Hz(hz as u64)), latency: 0 }
    }

    #[test]
    fn the_node_reads_any_span_fast_enough_for_the_synchronisation_signals() {
        let mut n = LteNode::new(Some(762.96e6));
        assert_eq!(n.seed, Some(763e6), "on the 100 kHz raster");
        assert!(
            n.negotiate(&spec(1.5e6, 763e6)).is_err(),
            "too slow for the synchronisation signals"
        );
        let out = n.negotiate(&spec(12e6, 762e6)).expect("a span holding the carrier");
        assert_eq!((out.kind, out.center.as_f64()), (PortKind::Packets, 762e6));
        assert!(LteNode::new(None).negotiate(&spec(20e6, 806e6)).is_ok(), "no carrier named");
    }

    #[test]
    fn the_capture_carrier_is_labelled_by_its_earfcn() {
        assert_eq!(Lte.stage_label(763e6), "EARFCN 9260");
        assert_eq!(Lte.stage_label(600e6), "600.0 LTE");
    }
}
