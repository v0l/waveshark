use crate::NodeSpec;
use crate::broadcast::{Broadcast, SERVICE, SOUND_RATE_HZ, Want};
use crate::protocol::{Placed, Placement, Protocol, Shape};
use common::{C32, Result};
use decode::dvbs2::{self as dvbs2dec, Fec, Outcome, Transport};
use decode::dvbt::TsPacket;
use dsp::dc::SpurCancel;
use dsp::dvbs2::acquire::estimate_within;
use dsp::dvbs2::acquire::spurs;
use dsp::dvbs2::{Config, Constellation, Dvbs2 as Phy, Framed, Header};
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::dvbs2::Dvbs2;
pub use identify::dvbs2::{DEFAULT_HZ, MIN_RATE_HZ, ROLLOFF, WIDTH_HZ};
use pipeline::node::{NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

pub const SYSTEM: &str = "DVB-S2";
const CHANNEL_HZ: &str = "channel_hz";
const SYMBOL_RATE: &str = "symbol_rate";
const WIDTH: &str = "width_hz";
const QUEUE: usize = 16;
const WAIT: std::time::Duration = std::time::Duration::from_millis(200);
const LOOK: usize = 1 << 18;
const MARGIN: f64 = 1.3;
const SPUR_WIDTH_HZ: f64 = 3_000.0;
const REORDER: usize = 64;
const QUIET_S: f64 = 0.5;

#[derive(Clone, Copy, Debug, Default)]
struct Heard {
    locked: bool,
    header: Option<Header>,
    mer_db: Option<f32>,
    faded: u64,
    offset_hz: f64,
    symbol_rate: Option<f64>,
}

struct Front {
    rate: f64,
    within: f64,
    mixer: Mixer,
    spurs: Option<Vec<SpurCancel>>,
    recentre: Mixer,
    centred: Vec<C32>,
    decim: Option<FirDecim>,
    symbol_rate: Option<f64>,
    looking: Vec<C32>,
    phy: Option<Phy>,
    offset_hz: f64,
    mixed: Vec<C32>,
    narrow: Vec<C32>,
}

impl Front {
    fn new(rate: f64, shift: f64, symbol_rate: Option<f64>, within: f64) -> Front {
        Front {
            rate,
            within,
            mixer: Mixer::new(shift, rate),
            spurs: None,
            recentre: Mixer::new(0.0, rate),
            centred: Vec::new(),
            decim: None,
            symbol_rate,
            looking: Vec::new(),
            phy: None,
            offset_hz: 0.0,
            mixed: Vec::new(),
            narrow: Vec::new(),
        }
    }

    fn push(&mut self, iq: &[C32], out: &mut Vec<Framed>) {
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        for s in self.spurs.iter_mut().flatten() {
            s.process(&mut self.mixed);
        }
        if self.phy.is_none() {
            self.looking.extend_from_slice(&self.mixed);
            if self.looking.len() < LOOK {
                return;
            }
            if self.spurs.is_none() {
                let mut found: Vec<SpurCancel> = spurs(&self.looking, self.rate)
                    .into_iter()
                    .map(|hz| SpurCancel::new(hz, self.rate, SPUR_WIDTH_HZ))
                    .collect();
                for s in &mut found {
                    s.process(&mut self.looking);
                }
                self.spurs = Some(found);
            }
            let found = estimate_within(&self.looking, self.rate, self.within);
            let symbol_rate = match (self.symbol_rate, found) {
                (Some(rs), _) => rs,
                (None, Some(c)) => c.symbol_rate,
                (None, None) => {
                    self.looking.clear();
                    return;
                }
            };
            let factor =
                (self.rate / (symbol_rate * (1.0 + ROLLOFF) * MARGIN)).floor().max(1.0) as usize;
            self.offset_hz = found.filter(|_| factor > 1).map_or(0.0, |c| c.offset_hz);
            self.recentre = Mixer::new(-self.offset_hz, self.rate);
            self.mixed = std::mem::take(&mut self.looking);
            self.decim = (factor > 1).then(|| {
                FirDecim::design_hz(
                    self.rate,
                    factor,
                    symbol_rate * (1.0 + ROLLOFF) / 2.0 * 1.05,
                    60.0,
                )
            });
            self.symbol_rate = Some(symbol_rate);
            self.phy = Some(Phy::new(Config {
                rate_hz: self.rate / factor as f64,
                symbol_rate,
                rolloff: ROLLOFF,
                gold: 0,
            }));
        }
        let Some(phy) = &mut self.phy else { return };
        self.centred.clear();
        self.recentre.process(&self.mixed, &mut self.centred);
        match &mut self.decim {
            Some(d) => {
                self.narrow.clear();
                d.process(&self.centred, &mut self.narrow);
                phy.push(&self.narrow, out);
            }
            None => phy.push(&self.centred, out),
        }
    }

    fn heard(&self) -> Heard {
        let Some(phy) = &self.phy else {
            return Heard { symbol_rate: self.symbol_rate, ..Heard::default() };
        };
        Heard {
            locked: phy.locked(),
            header: phy.heard(),
            mer_db: phy.mer_db(),
            faded: phy.faded(),
            offset_hz: self.offset_hz + phy.offset_hz(),
            symbol_rate: Some(phy.symbol_rate()),
        }
    }
}

struct Offloaded {
    iq: Option<crossbeam_channel::Sender<Vec<C32>>>,
    results: crossbeam_channel::Receiver<(u64, Outcome)>,
    pending: BTreeMap<u64, Outcome>,
    next: u64,
    heard: Arc<Mutex<Heard>>,
    dropped: u64,
    threads: Vec<std::thread::JoinHandle<()>>,
}

fn workers() -> usize {
    std::thread::available_parallelism().map_or(2, |n| n.get() / 2).clamp(1, 6)
}

impl Offloaded {
    fn new(rate: f64, shift: f64, symbol_rate: Option<f64>, within: f64) -> Offloaded {
        let (iq, work) = crossbeam_channel::bounded::<Vec<C32>>(QUEUE);
        let count = workers();
        let (to_fec, fec_in) = crossbeam_channel::bounded::<(u64, Framed)>(4 * count);
        let (done, results) = crossbeam_channel::unbounded::<(u64, Outcome)>();
        let heard = Arc::new(Mutex::new(Heard::default()));
        let mut threads = Vec::new();
        for n in 0..count {
            let (fec_in, done) = (fec_in.clone(), done.clone());
            let t = std::thread::Builder::new().name(format!("dvbs2-fec-{n}")).spawn(move || {
                let mut fec = Fec::new();
                while let Ok((seq, frame)) = fec_in.recv() {
                    let read = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        fec.decode(&frame.demodulate())
                    }));
                    let outcome = read.unwrap_or_else(|_| {
                        fec = Fec::new();
                        Outcome::Unsupported
                    });
                    if done.send((seq, outcome)).is_err() {
                        return;
                    }
                }
            });
            threads.extend(t.ok());
        }
        let mine = heard.clone();
        let phy = std::thread::Builder::new().name("dvbs2".into()).spawn(move || {
            let mut front = Front::new(rate, shift, symbol_rate, within);
            let mut frames = Vec::new();
            let mut seq = 0u64;
            while let Ok(block) = work.recv() {
                frames.clear();
                front.push(&block, &mut frames);
                for f in frames.drain(..) {
                    match to_fec.send_timeout((seq, f), WAIT) {
                        Ok(()) => {}
                        Err(crossbeam_channel::SendTimeoutError::Timeout(_)) => {
                            let _ = done.send((seq, Outcome::Shed));
                        }
                        Err(crossbeam_channel::SendTimeoutError::Disconnected(_)) => return,
                    }
                    seq += 1;
                }
                *mine.lock().unwrap_or_else(|e| e.into_inner()) = front.heard();
            }
        });
        threads.extend(phy.ok());
        Offloaded {
            iq: Some(iq),
            results,
            pending: BTreeMap::new(),
            next: 0,
            heard,
            dropped: 0,
            threads,
        }
    }

    fn push(&mut self, iq: &[C32]) {
        let Some(tx) = &self.iq else { return };
        if tx.send_timeout(iq.to_vec(), WAIT).is_err() {
            self.dropped += 1;
        }
    }

    fn take(&mut self, transport: &mut Transport, out: &mut Vec<TsPacket>) -> usize {
        while let Ok((seq, o)) = self.results.try_recv() {
            self.pending.insert(seq, o);
        }
        let mut taken = 0;
        loop {
            match self.pending.remove(&self.next) {
                Some(o) => transport.take(o, out),
                None if self.pending.len() > REORDER => transport.take(Outcome::Shed, out),
                None => return taken,
            }
            self.next += 1;
            taken += 1;
        }
    }

    fn finish(&mut self, transport: &mut Transport, out: &mut Vec<TsPacket>) {
        self.iq = None;
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
        self.take(transport, out);
    }

    fn heard(&self) -> Heard {
        *self.heard.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl Drop for Offloaded {
    fn drop(&mut self) {
        self.iq = None;
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

pub struct Dvbs2Node {
    channel_hz: f64,
    symbol_rate: Option<f64>,
    width_hz: Option<f64>,
    rate: f64,
    shift: f64,
    rx: Option<Offloaded>,
    transport: Transport,
    tv: Broadcast,
    packets: Vec<TsPacket>,
    told: Option<Header>,
    quiet_s: f64,
}

impl Dvbs2Node {
    pub fn new(channel_hz: f64, symbol_rate: Option<f64>) -> Self {
        Self {
            channel_hz,
            symbol_rate,
            width_hz: None,
            rate: 0.0,
            shift: 0.0,
            rx: None,
            transport: Transport::default(),
            tv: Broadcast::new(SYSTEM, channel_hz),
            packets: Vec::new(),
            told: None,
            quiet_s: f64::INFINITY,
        }
    }

    pub fn want(&mut self, want: Want) {
        self.tv.want(want);
    }

    pub fn within(&mut self, width_hz: f64) {
        self.width_hz = (width_hz > 0.0).then_some(width_hz);
    }

    pub fn wanted(&self) -> &Want {
        self.tv.wanted()
    }

    pub fn watching(&self) -> Option<u16> {
        self.tv.watching()
    }

    pub fn services(&self) -> &[decode::mpegts::Service] {
        self.tv.services()
    }

    pub fn broadcast(&self) -> &Broadcast {
        &self.tv
    }

    pub fn stats(&self) -> dvbs2dec::Stats {
        self.transport.stats()
    }

    pub fn header(&self) -> Option<Header> {
        self.heard().header
    }

    pub fn symbol_rate(&self) -> Option<f64> {
        self.heard().symbol_rate
    }

    fn heard(&self) -> Heard {
        self.rx.as_ref().map(Offloaded::heard).unwrap_or_default()
    }

    pub fn flush(&mut self, out: &mut Vec<common::VideoFrame>) -> crate::broadcast::Tail {
        let mut packets = Vec::new();
        if let Some(rx) = &mut self.rx {
            rx.finish(&mut self.transport, &mut packets);
        }
        self.tv.flush(&packets, out)
    }

    fn start(&mut self) {
        self.rx = (self.rate > 0.0).then(|| {
            let within = self.width_hz.map_or(self.rate, |w| w.min(self.rate)) / 2.0;
            Offloaded::new(self.rate, self.shift, self.symbol_rate, within)
        });
        self.transport = Transport::default();
        self.told = None;
        self.quiet_s = f64::INFINITY;
    }
}

fn keying(header: Header) -> common::Modulation {
    match header.modcod.constellation {
        Constellation::Qpsk => common::Modulation::Psk4,
        Constellation::Psk8 => common::Modulation::Psk8,
        Constellation::Apsk16 | Constellation::Apsk32 => common::Modulation::Apsk,
    }
}

impl pipeline::node::Node for Dvbs2Node {
    fn name(&self) -> &str {
        "dvbs2"
    }

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        3
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("dvbs2 reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if rate < MIN_RATE_HZ {
            return Err(common::Error::other("dvbs2 needs 2 MS/s of span"));
        }
        if (self.channel_hz - center).abs() > rate / 2.0 {
            return Err(common::Error::other("dvbs2 needs its carrier inside the span"));
        }
        self.rate = rate;
        self.shift = center - self.channel_hz;
        self.start();
        self.tv.retune();

        let mut out = i.spec.with_kind(PortKind::Bytes);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = WIDTH_HZ.min(rate);
        let mut video = out.with_kind(PortKind::Video);
        video.rate = 0.0;
        let mut sound = out.with_kind(PortKind::Real);
        sound.rate = SOUND_RATE_HZ;
        sound.channels = 1;
        sound.bandwidth = 0.0;
        Ok(vec![out, video, sound])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(iq) = inputs[0].as_iq() else { return Ok(()) };
        let Some(rx) = &mut self.rx else { return Ok(()) };
        rx.push(iq);
        self.packets.clear();
        let mut packets = std::mem::take(&mut self.packets);
        let before = self.transport.stats();
        rx.take(&mut self.transport, &mut packets);
        self.quiet_s = match self.transport.stats().packets > before.packets {
            true => 0.0,
            false => self.quiet_s + c.block_seconds,
        };
        self.tv.push(&packets, outputs[0].bytes_mut());
        self.packets = packets;
        let (_, rest) = outputs.split_at_mut(1);
        let (video, sound) = rest.split_at_mut(1);
        self.tv.play(c.block_seconds, &mut video[0], &mut sound[0]);

        let heard = self.heard();
        let mer = heard.mer_db.unwrap_or(0.0);
        let width = heard.symbol_rate.map_or(WIDTH_HZ, |rs| rs * (1.0 + ROLLOFF));
        if let Some(header) = heard.header
            && self.told != Some(header)
        {
            self.told = Some(header);
            let carrier = crate::locked(self.channel_hz as u64, width as u32, iq, mer);
            c.emit(pipeline::event::Event::Decoded(
                common::packet::Packet::heard(carrier)
                    .keyed(common::packet::Keying::configured(keying(header)))
                    .decoded(dvbs2dec::carrier_read()),
            ));
        }
        for id in self.tv.fresh_services() {
            if let Some(d) = dvbs2dec::service_read(self.tv.mux(), id) {
                let carrier = crate::locked(self.channel_hz as u64, width as u32, iq, mer);
                let keyed = heard.header.map_or(common::Modulation::Unknown, keying);
                c.emit(pipeline::event::Event::Decoded(
                    common::packet::Packet::heard(carrier)
                        .keyed(common::packet::Keying::configured(keyed))
                        .decoded(d),
                ));
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.start();
        self.tv.reset();
    }

    fn params(&self) -> Vec<pipeline::param::Param> {
        vec![
            self.tv.param(),
            pipeline::param::Param::float(SYMBOL_RATE, self.symbol_rate.unwrap_or(0.0), 0.0..=60e6)
                .label("Symbol rate")
                .unit("sym/s"),
        ]
    }

    fn acquisition(&self) -> Option<pipeline::Acquisition> {
        let heard = self.heard();
        Some(match (heard.locked, self.quiet_s < QUIET_S) {
            (false, _) => pipeline::Acquisition::Searching,
            (true, true) => pipeline::Acquisition::Locked,
            (true, false) => pipeline::Acquisition::Acquiring,
        })
    }

    fn readings(&self) -> Vec<(String, String)> {
        let heard = self.heard();
        let stats = self.transport.stats();
        let mut out = Vec::new();
        if let Some(h) = heard.header {
            out.push(("carrier".into(), h.label()));
        }
        if let Some(rs) = heard.symbol_rate {
            out.push(("symbol rate".into(), format!("{:.3} Msym/s", rs / 1e6)));
        }
        if let Some(mer) = heard.mer_db.filter(|_| heard.locked) {
            out.push(("MER".into(), format!("{mer:.1} dB")));
            out.push(("offset".into(), format!("{:+.0} kHz", heard.offset_hz / 1e3)));
        }
        if let Some(bb) = self.transport.baseband()
            && let Some(alpha) = bb.rolloff.alpha()
        {
            out.push(("roll-off".into(), format!("{alpha:.2}")));
        }
        if stats.frames > 0 {
            out.push(("frames".into(), stats.frames.to_string()));
            out.push(("failed".into(), (stats.ldpc_failed + stats.bch_failed).to_string()));
            out.push(("packets".into(), stats.packets.to_string()));
        }
        if heard.faded > 0 {
            out.push(("too weak".into(), heard.faded.to_string()));
        }
        if stats.shed > 0 {
            out.push(("frames shed".into(), stats.shed.to_string()));
        }
        if let Some(rx) = &self.rx
            && rx.dropped > 0
        {
            out.push(("blocks lost".into(), rx.dropped.to_string()));
        }
        out
    }

    fn set_param(&mut self, name: &str, v: pipeline::ParamValue) -> Result<()> {
        match name {
            SERVICE => self.tv.set_service("dvbs2", v),
            SYMBOL_RATE => {
                let rs = v
                    .as_f64()
                    .ok_or_else(|| common::Error::other("dvbs2: a symbol rate is a number"))?;
                let rs = (rs > 0.0).then_some(rs);
                if rs != self.symbol_rate {
                    self.symbol_rate = rs;
                    self.start();
                }
                Ok(())
            }
            _ => Err(common::Error::other(format!("dvbs2: unknown parameter {name:?}"))),
        }
    }
}

impl Protocol for Dvbs2 {
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

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.1} DVB-S2", hz / 1e6)
    }
    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Bytes, PortKind::Video, PortKind::Real]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz).f(WIDTH, at.width_hz)]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "dvbs2",
    summary: "One DVB-S2 carrier: its frames, LDPC and BCH, its transport stream",
    category: Category::Decode,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let rs = s.f64_or(SYMBOL_RATE, 0.0);
    let mut node = Dvbs2Node::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ), (rs > 0.0).then_some(rs));
    node.want(Want::from_settings(s));
    node.within(s.f64_or(WIDTH, 0.0));
    Ok(Box::new(node))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use decode::dvbs2::{BbHeader, Rolloff, Stream};
    use dsp::dvbs2::{FecFrame, ModCod, tx};
    use dsp::resample::Rational;
    use pipeline::node::Node;

    fn packet(n: u32) -> [u8; 188] {
        let mut p = [0u8; 188];
        p[0] = 0x47;
        p[1] = 0x01;
        p[2] = 0x23;
        p[3] = 0x10 | (n % 16) as u8;
        for (i, b) in p[4..].iter_mut().enumerate() {
            *b = (i as u32 ^ n) as u8;
        }
        p
    }

    fn on_air(frames: usize, es_n0_db: f32, offset_hz: f64) -> (Vec<[u8; 188]>, Vec<C32>) {
        let header = Header {
            modcod: ModCod::from_index(14).unwrap(),
            frame: FecFrame::Normal,
            pilots: true,
        };
        let kbch = decode::bits::bch::Bch::dvbs2(header.frame, header.modcod.rate).unwrap().k();
        let dfl = (kbch - 80) / 8;
        let (mut stream, mut sent, mut crc, mut n) = (Vec::new(), Vec::new(), 0u8, 0);
        while stream.len() < dfl * frames + 188 {
            let p = packet(n);
            n += 1;
            let mut up = p;
            up[0] = crc;
            crc = decode::bits::crc8(&p[1..], 0xD5, 0);
            stream.extend_from_slice(&up);
            sent.push(p);
        }
        let mut symbols = Vec::new();
        for f in 0..frames {
            let start = f * dfl;
            let bb = BbHeader {
                stream: Stream::Transport,
                single_stream: true,
                constant_coding: true,
                issy: false,
                null_deletion: false,
                rolloff: Rolloff::R25,
                isi: 0,
                upl: 1504,
                dfl: (dfl * 8) as u16,
                sync: 0x47,
                syncd: ((188 - start % 188) % 188 * 8) as u16,
            };
            let word = decode::dvbs2::encode(header, &bb, &stream[start..start + dfl]);
            symbols.extend(tx::frame(header, &word, 0));
        }
        let shaped = tx::shape(&symbols, 4, ROLLOFF);
        let mut resample = Rational::approx(57e6, 20e6, 4096);
        let mut span = Vec::new();
        resample.process(&shaped, &mut span);
        let power = span.iter().map(|x| x.norm_sqr()).sum::<f32>() / span.len() as f32;
        let sigma = (power * (20e6 / 14.25e6) as f32 / 10f32.powf(es_n0_db / 10.0)).sqrt();
        let mut state = 0x5EED_u64;
        let mut uniform = || {
            state = state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            ((state >> 40) as f32 + 0.5) / (1u64 << 24) as f32
        };
        let iq = span
            .iter()
            .enumerate()
            .map(|(k, x)| {
                let p = 2.0 * std::f64::consts::PI * offset_hz * k as f64 / 20e6;
                let r = (-uniform().ln()).sqrt() * sigma;
                let t = 2.0 * std::f32::consts::PI * uniform();
                x * C32::new(p.cos() as f32, p.sin() as f32) + C32::new(r * t.cos(), r * t.sin())
            })
            .collect();
        (sent, iq)
    }

    fn run(
        node: &mut Dvbs2Node,
        iq: &[C32],
        centre: f64,
    ) -> (Vec<u8>, Vec<pipeline::event::Event>) {
        let spec = PortSpec { spec: StreamSpec::iq(20e6, Hz(centre as u64)), latency: 0 };
        node.negotiate(&[spec]).expect("a carrier in the span");
        let mut stream = Vec::new();
        let mut events = Vec::new();
        let settle = (0..400).map(|_| &[][..]);
        for (n, block) in iq.chunks(65_536).chain(settle).enumerate() {
            if block.is_empty() {
                if n > 0 && node.heard().header.is_some() && node.told.is_some() {
                    break;
                }
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
            let payload = Payload::Iq(block.to_vec());
            let mut out = [
                Payload::empty_of(PortKind::Bytes),
                Payload::empty_of(PortKind::Video),
                Payload::empty_of(PortKind::Real),
            ];
            let ins = [spec];
            let (tags, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            Node::process(node, &[&payload], &mut out, &mut ctx).expect("the stage runs");
            stream.extend_from_slice(out[0].as_bytes().unwrap_or(&[]));
        }
        stream.extend_from_slice(&node.flush(&mut Vec::new()).bytes);
        (stream, events)
    }

    fn reading(node: &Dvbs2Node, caption: &str) -> Option<String> {
        Node::readings(node).into_iter().find(|(c, _)| c == caption).map(|(_, v)| v)
    }

    #[test]
    fn a_carrier_off_the_channel_is_found_timed_and_read_to_its_packets() {
        let (sent, iq) = on_air(24, 14.0, 2.3e6);
        let mut node = Dvbs2Node::new(1_433_100_000.0, None);
        let (stream, events) = run(&mut node, &iq, 1_431_100_000.0);
        assert_eq!(stream.len() % 188, 0);
        let got: Vec<&[u8]> = stream.chunks_exact(188).collect();
        let first = sent.iter().position(|p| p[..] == *got[0]).expect("the first packet was sent");
        assert!(
            got.iter().zip(&sent[first..]).all(|(g, s)| *g == &s[..]),
            "packets out of order or changed"
        );
        let stats = node.stats();
        assert_eq!(stats.ldpc_failed + stats.bch_failed + stats.shed, 0, "{stats:?}");
        assert_eq!((got.len(), first), (739, 0), "{stats:?}");
        assert_eq!(reading(&node, "carrier").as_deref(), Some("8PSK 3/4, normal frames, pilots"));
        assert_eq!(reading(&node, "symbol rate").as_deref(), Some("14.250 Msym/s"));
        assert_eq!(reading(&node, "roll-off").as_deref(), Some("0.25"));
        let told = events
            .iter()
            .filter(|e| matches!(
                e,
                pipeline::event::Event::Decoded(p) if p.innermost().is_some_and(|l| l.id == "dvbs2")
            ))
            .count();
        assert_eq!(told, 1, "the carrier is announced once, and the packets carry no tables");
    }

    #[test]
    fn a_hackrf_centre_spur_seven_db_down_inside_the_carrier_is_taken_out() {
        let (sent, mut iq) = on_air(24, 12.0, 2.3e6);
        let power = iq.iter().map(|x| x.norm_sqr()).sum::<f32>() / iq.len() as f32;
        let offset = C32::new(1.0, -0.6) * (0.2 * power / 1.36).sqrt();
        iq.iter_mut().for_each(|x| *x += offset);
        let mut node = Dvbs2Node::new(1_433_100_000.0, None);
        let (stream, _) = run(&mut node, &iq, 1_431_100_000.0);
        let got: Vec<&[u8]> = stream.chunks_exact(188).collect();
        let first = sent.iter().position(|p| p[..] == *got[0]).expect("the first packet was sent");
        let stats = node.stats();
        assert_eq!(stats.ldpc_failed + stats.bch_failed, 0, "{stats:?}");
        assert_eq!((got.len(), first), (739, 0), "{stats:?}");
    }

    #[test]
    fn a_frame_that_never_comes_back_from_the_decoders_does_not_hold_up_the_rest() {
        let (done, results) = crossbeam_channel::unbounded();
        let mut rx = Offloaded {
            iq: None,
            results,
            pending: BTreeMap::new(),
            next: 0,
            heard: Arc::new(Mutex::new(Heard::default())),
            dropped: 0,
            threads: Vec::new(),
        };
        let mut transport = Transport::default();
        let mut out = Vec::new();
        for seq in 1..=REORDER as u64 {
            done.send((seq, Outcome::HeaderFailed)).unwrap();
        }
        assert_eq!(rx.take(&mut transport, &mut out), 0, "waiting for frame 0 while it may come");
        done.send((REORDER as u64 + 1, Outcome::HeaderFailed)).unwrap();
        assert_eq!(rx.take(&mut transport, &mut out), REORDER + 2);
        let stats = transport.stats();
        assert_eq!((stats.shed, stats.header_failed), (1, REORDER as u64 + 1));
    }
}
