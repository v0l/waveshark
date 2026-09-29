use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::timesignal::{self, Station};
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::timesignal::{AUDIO_HZ, CHANNEL_WIDTH_HZ, DCF77, MSF, Reader, TDF, TimeSignal};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub struct TimeNode {
    station: Station,
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    reader: Reader,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    minutes: Vec<timesignal::Minute>,
    meter: crate::FrameMeter,
    read: u64,
}

impl TimeNode {
    pub fn new(station: Station, channel_hz: f64) -> Self {
        Self {
            station,
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            reader: Reader::new(station, AUDIO_HZ),
            mixed: Vec::new(),
            narrow: Vec::new(),
            minutes: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 60.0)
                .keyed_as(keying(station)),
            read: 0,
        }
    }
}

impl Simple for TimeNode {
    fn name(&self) -> &str {
        self.station.id()
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("a time signal reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("the time signal is outside the span"));
        }
        let factor = (rate / AUDIO_HZ).floor().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.reader = Reader::new(self.station, audio_rate);
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 60.0)
            .keyed_as(keying(self.station));
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);
        self.minutes.clear();
        self.reader.push(&self.narrow, &mut self.minutes);
        self.read += self.minutes.len() as u64;
        let frames: Vec<Vec<u8>> =
            self.minutes.iter().map(|m| timesignal::encode(self.station, m)).collect();
        let read = self.meter.packets(frames, common::packet::now_us());
        o.packets_mut()
            .extend(read.into_iter().map(|p| p.checked(common::packet::Integrity::Passed)));
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.meter.reset();
    }

    fn acquisition(&self) -> Option<pipeline::Acquisition> {
        Some(match self.read {
            0 => pipeline::Acquisition::Searching,
            _ => pipeline::Acquisition::Locked,
        })
    }
}

impl Protocol for TimeSignal {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::Continuously
    }

    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
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
    fn keys(&self) -> Option<common::Modulation> {
        Some(keying(self.0))
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        timesignal::read(p.bytes()).filter(|d| d.id == self.0.id()).map(|d| vec![d])
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.1} kHz {}", hz / 1e3, self.0.name())
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: self.0.name().into() }]
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        let desc = match self.0 {
            Station::Msf => MSF_DESC,
            Station::Dcf77 => DCF77_DESC,
            Station::Tdf => TDF_DESC,
        };
        vec![NodeSpec::new(desc.name).f(CHANNEL_HZ, self.hz())]
    }
}

const CHANNEL_HZ: &str = "channel_hz";

fn keying(station: Station) -> common::Modulation {
    match station {
        Station::Msf | Station::Dcf77 => common::Modulation::Ask,
        Station::Tdf => common::Modulation::Psk2,
    }
}

pub const MSF_DESC: StageDesc = StageDesc {
    name: "msf",
    summary: "MSF on 60 kHz: the minute's date and time, one bit a second",
    category: Category::Decode,
    feeds_bus: true,
};

pub const DCF77_DESC: StageDesc = StageDesc {
    name: "dcf77",
    summary: "DCF77 on 77.5 kHz: the minute's date and time, one bit a second",
    category: Category::Decode,
    feeds_bus: true,
};

pub const TDF_DESC: StageDesc = StageDesc {
    name: "tdf",
    summary: "TDF on 162 kHz: the minute's date and time, phase keyed one bit a second",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build_tdf(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(TimeNode::new(Station::Tdf, s.f64_or(CHANNEL_HZ, TDF.hz()))))
}

pub fn build_msf(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(TimeNode::new(Station::Msf, s.f64_or(CHANNEL_HZ, MSF.hz()))))
}

pub fn build_dcf77(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(TimeNode::new(Station::Dcf77, s.f64_or(CHANNEL_HZ, DCF77.hz()))))
}
