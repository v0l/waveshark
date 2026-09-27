use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape, Stickiness};
use common::Result;
use identify::Signal;
pub use identify::walksnail::{CENTERS_HZ, DEFAULT_HZ, MIN_RATE_HZ, WIDTH_HZ, Walksnail};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

pub struct WalksnailNode {
    span: Option<dsp::artosyn::Span>,
    reports: Vec<dsp::artosyn::Report>,
}

impl Default for WalksnailNode {
    fn default() -> Self {
        Self::new()
    }
}

impl WalksnailNode {
    pub fn new() -> Self {
        Self { span: None, reports: Vec::new() }
    }
}

impl Simple for WalksnailNode {
    fn name(&self) -> &'static str {
        "walksnail"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Iq {
            return Err(common::Error::other("walksnail reads complex baseband"));
        }
        let (rate, center) = (input.spec.rate, input.spec.center.as_f64());
        if rate + 1.0 < MIN_RATE_HZ {
            return Err(common::Error::other(
                "walksnail needs 20 MS/s: the channel is 1729 carriers 10.9 kHz apart",
            ));
        }
        let Some(span) = dsp::artosyn::Span::new(rate, center, &CENTERS_HZ) else {
            return Err(common::Error::other("walksnail needs a whole 20 MHz channel in the span"));
        };
        let hz = match span.channels().as_slice() {
            [one] => *one,
            _ => center,
        };
        self.span = Some(span);
        let mut out = input.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(hz as u64);
        out.bandwidth = WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, input: &Payload, output: &mut Payload, _ctx: &mut NodeCtx) -> Result<()> {
        let (Payload::Iq(iq), Some(span)) = (input, self.span.as_mut()) else {
            return Ok(());
        };
        self.reports.clear();
        span.process(iq, &mut self.reports);
        let out = output.packets_mut();
        for r in self.reports.drain(..) {
            let bytes = decode::walksnail::wrap(&decode::walksnail::link(&r));
            let mut p =
                crate::measured(r.center_hz as u64, WIDTH_HZ as u32, bytes, r.rssi_dbfs, r.snr_db)
                    .keyed(common::packet::Keying::configured(common::Modulation::Ofdm));
            p.carrier.iq = Some(std::sync::Arc::new(common::IqBurst {
                rate: r.rate,
                center_hz: r.center_hz as u64,
                samples: r.samples,
            }));
            out.push(p);
        }
        Ok(())
    }

    fn acquisition(&self) -> Option<pipeline::Acquisition> {
        Some(match self.span.as_ref().is_some_and(|s| s.locked()) {
            true => pipeline::Acquisition::Locked,
            false => pipeline::Acquisition::Searching,
        })
    }

    fn readings(&self) -> Vec<(String, String)> {
        let Some(span) = self.span.as_ref() else { return Vec::new() };
        let mut out = Vec::new();
        for r in span.latest() {
            let l = decode::walksnail::link(r);
            let at = format!("{:.0} MHz", r.center_hz / 1e6);
            let rate = l
                .frames_per_second()
                .map_or(format!("{} frames", l.frames), |f| format!("{f:.1} frames/s"));
            out.push((at, rate));
            if let Some(c) = l.constellation {
                out.push(("cells".into(), decode::walksnail::constellation_label(c).into()));
            }
            if let Some(mer) = l.mer_db {
                out.push(("MER".into(), format!("{mer:.1} dB")));
            }
            if let Some(b) = l.balance_db {
                out.push(("antennas".into(), format!("{b:+.1} dB")));
            }
            if let Some(c) = l.counter {
                out.push(("frame".into(), c.to_string()));
            }
            if l.missed > 0 {
                out.push(("missed".into(), l.missed.to_string()));
            }
            out.push(("offset".into(), format!("{:+.1} kHz", r.offset_hz / 1e3)));
        }
        out
    }

    fn reset(&mut self) {
        if let Some(s) = self.span.as_mut() {
            s.reset();
        }
        self.reports.clear();
    }
}

impl Protocol for Walksnail {
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
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        decode::walksnail::read(p.bytes()).map(|d| vec![d])
    }
    fn stickiness(&self) -> Stickiness {
        Stickiness::Forget
    }
    fn wakes_on(&self) -> crate::protocol::Wake {
        crate::protocol::Wake::Detected { hold_s: 1.0 }
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.0} WALKSNAIL", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: WIDTH_HZ, label: "WALKSNAIL".into() }]
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("walksnail")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "walksnail",
    summary: "Walksnail Avatar: the 20 MHz OFDM downlink a video transmitter sends, measured without its payload",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(WalksnailNode::new()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(rate: f64, hz: u64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, common::Hz(hz)), latency: 0 }
    }

    #[test]
    fn the_node_wants_a_whole_channel_at_20_ms_per_second() {
        let mut n = WalksnailNode::new();
        assert!(n.negotiate(&spec(10e6, 5_805_000_000)).is_err(), "too slow for the channel");
        assert!(n.negotiate(&spec(20e6, 5_812_000_000)).is_err(), "the channel hangs off the span");
        let out = n.negotiate(&spec(20e6, 5_805_000_000)).expect("one channel");
        assert_eq!(out.center.as_f64(), 5_805e6);
        assert_eq!(n.span.as_ref().map(|s| s.channels()), Some(vec![5_805e6]));
    }

    #[test]
    fn a_hackrf_span_holds_a_channel_only_within_half_a_megahertz_of_its_centre() {
        let mut n = WalksnailNode::new();
        let out =
            n.negotiate(&spec(20e6, 5_805_500_000)).expect("0.5 MHz off still holds 18.9 MHz");
        assert_eq!(out.center.as_f64(), 5_805e6);
        assert!(
            n.negotiate(&spec(20e6, 5_805_600_000)).is_err(),
            "0.6 MHz off cuts the outer carriers"
        );
    }
}
