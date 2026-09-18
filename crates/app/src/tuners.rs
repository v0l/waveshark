//! Radios the receiver is not listening to, served to the network.
//!
//! The head's own stage publishes the span this receiver is on. This one takes
//! a radio nobody here is using, opens it, and publishes it as another stream
//! on the same port, so a machine with three dongles in it is three tuners a
//! subscriber can pick from rather than one.
//!
//! The dial of such a tuner is offered without asking, which the receiver's
//! own never is: there is no local screen showing it, so a remote tune moves
//! nothing anybody here is watching.
//!
//! The stage lives in the application rather than in `nodes`, because opening
//! a radio means naming the drivers and a node may not.
//!
//! # Why the samples come off a thread
//!
//! `RxStream::read` blocks until the radio has a buffer, and the graph's block
//! is the receiver's own radio's. Two radios at two rates cannot be clocked by
//! each other, so the served one gets a thread that reads it, pushes what it
//! read, and answers whatever a subscriber asked of its dial. The node holds
//! the thread and what it is doing, and the graph's tick only ever starts it.

use common::device::{Device, DriverKind, GainMode};
use common::{Hz, Result, SampleFormat, Sps};
use iqstream::{Setting, SettingKind, SettingValue};
use pipeline::SettingsExt;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

/// Which radio to serve: its position in the receiver list, or a piece of its
/// label.
pub const RADIO: &str = "radio";
/// Where the server listens, as `host:port`, shared with any other stage
/// naming the same address.
pub const ADDRESS: &str = "address";
/// What the tuner is called on the wire. Its label unless something said
/// otherwise.
pub const STREAM: &str = "stream";
/// Samples per second to run it at.
pub const RATE: &str = "rate";
/// Where to park it, in hertz. A subscriber may move it from there.
pub const CENTER_HZ: &str = "center_hz";

/// What a served radio runs at when nothing said.
///
/// The rate a dongle is happiest at and what nearly everything reading one
/// over the network expects. A radio that cannot reach it is given its own
/// fastest instead.
const DEFAULT_RATE: Sps = Sps(2_400_000);

pub struct ServedTunerNode {
    radio: String,
    address: String,
    stream: Option<String>,
    rate: Option<Sps>,
    center: Option<Hz>,
    serving: Option<Serving>,
    /// Why there is nothing being served, for the chain view. Kept so a radio
    /// somebody else has open is not reopened every block, and cleared when
    /// the next attempt is due.
    fault: Option<String>,
    /// When the radio may be tried again. A dongle plugged in after the
    /// receiver started is worth serving, and one somebody else had open is
    /// worth serving when they let it go.
    next_try: Option<std::time::Instant>,
}

/// How long a radio that would not open is left alone.
///
/// Enumerating the USB bus is not free and a fault is usually somebody else
/// holding the dongle, which they hold for minutes rather than milliseconds.
const RETRY_EVERY: std::time::Duration = std::time::Duration::from_secs(5);

/// One radio, open and being read.
struct Serving {
    label: String,
    tuner: Arc<iqstream::Stream>,
    server: Arc<iqstream::Server>,
    stop: Arc<AtomicBool>,
    blocks: Arc<AtomicU64>,
    /// Set by the thread when the radio stops delivering: unplugged, taken,
    /// or ended by the driver. The node then takes the stream off the server
    /// rather than leaving subscribers on one nothing pushes to.
    ended: Arc<Mutex<Option<String>>>,
}

impl Drop for Serving {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        // Not joined: the thread is inside a blocking read and cannot be woken
        // from here. It sees the flag when the radio next delivers, closes the
        // device and ends.
    }
}

impl ServedTunerNode {
    pub fn new(radio: &str, address: &str) -> Self {
        Self {
            radio: radio.into(),
            address: address.into(),
            stream: None,
            rate: None,
            center: None,
            serving: None,
            fault: None,
            next_try: None,
        }
    }

    pub fn at_rate(mut self, rate: Option<Sps>) -> Self {
        self.rate = rate;
        self
    }

    pub fn at(mut self, center: Option<Hz>) -> Self {
        self.center = center;
        self
    }

    pub fn called(mut self, name: Option<String>) -> Self {
        self.stream = name;
        self
    }

    /// Open the radio and start reading it, or say why not and try again in
    /// a while.
    fn start(&mut self) {
        // A radio that stopped delivering takes its stream with it, so a
        // subscriber is dropped rather than left on a stream nothing pushes
        // to, and the next attempt opens a fresh one.
        if let Some(s) = &self.serving
            && let Some(why) = s.ended.lock().ok().and_then(|e| e.clone())
        {
            tracing::warn!("iqstream_tuner: {}: {why}", s.label);
            s.server.remove_stream(s.tuner.id());
            self.serving = None;
            self.fault = Some(why);
            self.next_try = Some(std::time::Instant::now() + RETRY_EVERY);
        }
        if self.serving.is_some() {
            return;
        }
        match self.next_try {
            Some(at) if std::time::Instant::now() < at => return,
            _ => {}
        }
        match self.open() {
            Ok(s) => {
                self.serving = Some(s);
                self.fault = None;
                self.next_try = None;
            }
            Err(e) => {
                // Said once per attempt rather than once per block, which at
                // a hundred blocks a second would be a log nobody can read.
                tracing::warn!("iqstream_tuner: {}: {e}", self.radio);
                self.fault = Some(e.to_string());
                self.next_try = Some(std::time::Instant::now() + RETRY_EVERY);
            }
        }
    }

    fn open(&mut self) -> Result<Serving> {
        let entry = pick(&self.radio).ok_or_else(|| {
            common::Error::other(format!("no radio matching {:?} is attached", self.radio))
        })?;
        let mut dev = crate::devices::open(&entry)?;

        let rate = self.rate.unwrap_or_else(|| DEFAULT_RATE.min(*entry.rates.end()));
        dev.set_rate(rate)?;
        if let Some(c) = self.center {
            dev.set_dial(c)?;
        }
        // Whatever the driver will pick for itself: a served dongle has no
        // operator here to turn a gain up, and one left at zero reads nothing.
        for stage in dev.info().gain_stages.clone() {
            let _ = dev.set_gain(&stage.name, GainMode::Auto);
        }

        let addr = self
            .address
            .parse()
            .map_err(|_| common::Error::other(format!("{:?} is not an address", self.address)))?;
        let server = nodes::iqstream_nodes::server(addr)?;
        let reach = dev.info().ranges.iter().fold(None::<(u64, u64)>, |acc, r| {
            let (lo, hi) = (r.range.start().0, r.range.end().0);
            Some(acc.map_or((lo, hi), |(a, b)| (a.min(lo), b.max(hi))))
        });
        let label = entry.label.clone();
        let tuner = server.stream_named(iqstream::StreamConfig {
            name: self.stream.clone().unwrap_or_else(|| label.clone()),
            center_hz: dev.center().0,
            sample_rate: dev.rate().0 as u32,
            gain_db: None,
            settings: settings_of(dev.as_ref()),
            // Always, and this is the difference from the receiver's own span:
            // nothing here is listening to this radio, so moving it disturbs
            // nobody.
            tunable: true,
            tune_range_hz: reach,
        });

        let stop = Arc::new(AtomicBool::new(false));
        let blocks = Arc::new(AtomicU64::new(0));
        let ended: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
        let pumping = (tuner.clone(), stop.clone(), blocks.clone(), ended.clone());
        std::thread::Builder::new()
            .name("iqstream-tuner".into())
            .spawn(move || {
                let (tuner, stop, blocks, ended) = pumping;
                let why = match pump(dev, &tuner, &stop, &blocks) {
                    Ok(()) => return,
                    Err(e) => e.to_string(),
                };
                if let Ok(mut held) = ended.lock() {
                    *held = Some(why);
                }
            })
            .map_err(|e| common::Error::other(format!("spawn tuner thread: {e}")))?;

        Ok(Serving { label, tuner, server, stop, blocks, ended })
    }
}

/// What the radio is set to, in the terms the protocol carries.
///
/// Read back off the driver rather than remembered from what was asked for: a
/// dongle snaps a gain to its nearest step, and a switch another program left
/// on is on whatever this stage did.
pub fn settings_of(dev: &dyn Device) -> Vec<Setting> {
    let info = dev.info();
    let mut out = Vec::new();
    let modes: Vec<(String, GainMode)> = dev.gains();
    for stage in &info.gain_stages {
        let mode = modes.iter().find(|(n, _)| *n == stage.name).map(|(_, m)| *m);
        out.push(Setting {
            name: stage.name.clone(),
            label: stage.label.clone(),
            kind: SettingKind::Gain,
            value: match mode {
                Some(GainMode::Manual(db)) => SettingValue::Gain(db),
                // A stage the driver will not read back is reported as the
                // driver's own choice, which is what it was left at.
                _ => SettingValue::Auto,
            },
            options: Vec::new(),
            range_db: Some((*stage.range.start(), *stage.range.end())),
        });
    }
    for t in dev.toggles() {
        out.push(Setting {
            name: t.name,
            label: t.label,
            kind: SettingKind::Switch,
            value: SettingValue::Switch(t.on),
            options: Vec::new(),
            range_db: None,
        });
    }
    for c in dev.choices() {
        out.push(Setting {
            name: c.name,
            label: c.label,
            kind: SettingKind::Choice,
            value: SettingValue::Choice(c.selected),
            options: c.options,
            range_db: None,
        });
    }
    out
}

/// Carry out one request, on whichever of the driver's controls it names.
fn apply(dev: &mut dyn Device, ask: &iqstream::Ask) -> Result<()> {
    match &ask.value {
        SettingValue::Auto => dev.set_gain(&ask.name, GainMode::Auto),
        SettingValue::Gain(db) => dev.set_gain(&ask.name, GainMode::Manual(*db)),
        SettingValue::Switch(on) => dev.set_toggle(&ask.name, *on),
        SettingValue::Choice(v) => dev.set_choice(&ask.name, v),
    }
}

/// How often the radio is asked what it is set to.
///
/// A block is tens of milliseconds and reading a gain back crosses USB, so
/// this is the gap between a switch being thrown at the far end and the
/// readers being told, against a control transfer a block.
const SETTINGS_EVERY: std::time::Duration = std::time::Duration::from_millis(500);

/// Read the radio, hand it out, and move it where a subscriber asked.
fn pump(
    mut dev: Box<dyn Device>,
    tuner: &Arc<iqstream::Stream>,
    stop: &Arc<AtomicBool>,
    blocks: &Arc<AtomicU64>,
) -> Result<()> {
    let mut rx = dev.start_rx()?;
    let mut uc8 = Vec::new();
    let mut asked_settings = std::time::Instant::now();
    while !stop.load(Ordering::SeqCst) {
        let buf = match rx.read() {
            Ok(b) => b,
            Err(e) => {
                rx.stop();
                return Err(e);
            }
        };
        SampleFormat::Cu8.encode(&buf.samples, &mut uc8);
        tuner.push(&uc8);
        blocks.fetch_add(1, Ordering::Relaxed);

        // A remote tune is answered here and nowhere else, because this radio
        // is not in the graph: where it lands is read back off the device
        // rather than assumed, since a dongle steps in units of its own.
        if let Some(t) = tuner.wanted() {
            match dev.set_dial(Hz(t.center_hz)) {
                Ok(()) => tuner.retuned(dev.center().0),
                Err(e) => tracing::debug!("iqstream_tuner: {e}"),
            }
        }

        // What a subscriber asked this radio to be set to. Applied and then
        // read back rather than assumed: a dongle snaps a gain to its own
        // step, and a switch it has not got is refused by the driver.
        for ask in tuner.asked() {
            if let Err(e) = apply(dev.as_mut(), &ask) {
                tracing::debug!("iqstream_tuner: {}: {e}", ask.name);
            }
            asked_settings = std::time::Instant::now() - SETTINGS_EVERY;
        }

        // And what it is set to, which moves without anything here asking:
        // a driver's own AGC, or another program with the same dongle open.
        // Only a set that differs is announced, so this is a comparison a
        // block and a message only when something moved.
        if asked_settings.elapsed() >= SETTINGS_EVERY {
            asked_settings = std::time::Instant::now();
            tuner.set_settings(settings_of(dev.as_ref()));
            tuner.set_sample_rate(dev.rate().0 as u32);
        }
    }
    rx.stop();
    Ok(())
}

/// The attached radio a setting names: a position in the list, or a piece of
/// its label.
///
/// Only real hardware, because the point is a radio this receiver is not
/// using: a capture, a network stream or a stitched pair is not a tuner
/// anybody can be handed.
fn pick(radio: &str) -> Option<crate::devices::Entry> {
    let attached: Vec<crate::devices::Entry> = crate::devices::list()
        .into_iter()
        .filter(|e| matches!(e.kind, DriverKind::RtlSdr | DriverKind::HackRf | DriverKind::LimeSdr))
        .collect();
    let wanted = radio.trim();
    if let Ok(i) = wanted.parse::<usize>() {
        return attached.into_iter().nth(i);
    }
    let lower = wanted.to_lowercase();
    attached.into_iter().find(|e| e.label.to_lowercase().contains(&lower))
}

impl Simple for ServedTunerNode {
    fn name(&self) -> &str {
        DESC.name
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn readings(&self) -> Vec<(String, String)> {
        let mut v = vec![("radio".into(), self.radio.clone())];
        match (&self.serving, &self.fault) {
            (Some(s), _) => {
                v.push(("stream".into(), s.tuner.name().to_string()));
                v.push(("tuner".into(), s.label.clone()));
                v.push(("at".into(), format!("{:.4} MHz", s.tuner.center_hz() as f64 / 1e6)));
                v.push(("readers".into(), s.tuner.subscribers().to_string()));
                v.push(("blocks".into(), s.blocks.load(Ordering::Relaxed).to_string()));
            }
            (None, Some(e)) => {
                v.push(("fault".into(), e.clone()));
                if let Some(at) = self.next_try {
                    let left = at.saturating_duration_since(std::time::Instant::now());
                    v.push(("retry in".into(), format!("{:.0} s", left.as_secs_f64())));
                }
            }
            (None, None) => v.push(("state".into(), "not open yet".into())),
        }
        v
    }

    /// Takes the receiver's own stream and passes it on untouched: this stage
    /// reads a different radio, and is in the graph to be seen, tapped and
    /// switched off like anything else.
    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("iqstream_tuner needs IQ"));
        }
        Ok(i.spec)
    }

    fn process(&mut self, _i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        self.start();
        Ok(())
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "iqstream_tuner",
    summary: "Serve a radio this receiver is not listening to as its own \
              stream, on the same port as the span",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let default = format!("0.0.0.0:{}", nodes::iqstream_nodes::DEFAULT_PORT);
    let rate = s.f64_or(RATE, 0.0);
    let center = s.f64_or(CENTER_HZ, 0.0);
    let name = s.get(STREAM).and_then(|v| v.as_str()).filter(|n| !n.is_empty()).map(String::from);
    Ok(Box::new(
        ServedTunerNode::new(s.str_or(RADIO, ""), s.str_or(ADDRESS, &default))
            .at_rate((rate > 0.0).then_some(Sps(rate as u64)))
            .at((center > 0.0).then_some(Hz(center as u64)))
            .called(name),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::node::Node;

    /// The stage takes IQ and nothing else, the same as the span's own
    /// server: it sits on the head to be clocked by it.
    #[test]
    fn the_stage_sits_on_an_iq_stream() {
        let mut n = ServedTunerNode::new("0", "127.0.0.1:0");
        let iq = PortSpec { spec: StreamSpec::iq(2_400_000.0, Hz::mhz(433)), latency: 0 };
        assert!(Simple::negotiate(&mut n, &iq).is_ok());
        let mut audio = StreamSpec::iq(48_000.0, Hz(0));
        audio.kind = PortKind::Real;
        assert!(Simple::negotiate(&mut n, &PortSpec { spec: audio, latency: 0 }).is_err());
    }

    /// A radio that is not attached is a fault on the stage rather than a
    /// receiver that will not start, and it is not tried again this block:
    /// reopening a missing dongle every block would be an enumeration of the
    /// USB bus a hundred times a second.
    #[test]
    fn a_radio_that_is_not_there_is_reported_and_tried_again_later() {
        let mut n = ServedTunerNode::new("no such dongle", "127.0.0.1:0");
        for _ in 0..3 {
            n.start();
        }
        let readings = Node::readings(&n);
        let fault = readings.iter().find(|(k, _)| k == "fault").map(|(_, v)| v.clone());
        assert!(fault.is_some_and(|f| f.contains("no such dongle")), "{readings:?}");
        assert!(n.serving.is_none());

        // But it is due again, because a dongle plugged in after the receiver
        // started is worth serving.
        let due = n.next_try.expect("another attempt");
        assert!(due > std::time::Instant::now());
        assert!(due <= std::time::Instant::now() + RETRY_EVERY);
        assert!(
            readings.iter().any(|(k, _)| k == "retry in"),
            "the chain view says when: {readings:?}"
        );
    }

    /// The stream takes the radio's name unless the patch said otherwise,
    /// because that is what a subscriber picking a tuner reads.
    #[test]
    fn a_stream_is_named_for_its_radio_unless_told() {
        let n = ServedTunerNode::new("0", "127.0.0.1:0");
        assert_eq!(n.stream, None);
        let n = n.called(Some("loft".into()));
        assert_eq!(n.stream.as_deref(), Some("loft"));
    }

    /// A radio describes itself through one field: its gain stages, its
    /// switches and its antenna port, in the terms the protocol carries, so a
    /// reader on the other side of the country knows what its samples were
    /// heard at.
    #[test]
    fn a_radios_gains_switches_and_ports_become_settings() {
        let dev = Bench::default();
        let s = settings_of(&dev);
        assert_eq!(s.len(), 4, "two gains, a switch and a port: {s:?}");

        assert_eq!(s[0].name, "lna");
        assert_eq!(s[0].label, "LNA");
        assert_eq!(s[0].kind, SettingKind::Gain);
        assert_eq!(s[0].value, SettingValue::Gain(24.0));
        assert_eq!(s[0].range_db, Some((0.0, 40.0)));
        // A stage the driver picks for itself reads as such rather than as a
        // number nothing is set to.
        assert_eq!(s[1].value, SettingValue::Auto);

        assert_eq!(s[2].kind, SettingKind::Switch);
        assert_eq!(s[2].value, SettingValue::Switch(true));
        assert_eq!(s[3].kind, SettingKind::Choice);
        assert_eq!(s[3].value, SettingValue::Choice("LNAW".into()));
        assert_eq!(s[3].options, vec!["LNAH", "LNAL", "LNAW"]);
    }

    /// A radio with nothing to say about itself says nothing, rather than a
    /// list of settings that are all the driver's defaults.
    #[test]
    fn a_radio_that_offers_nothing_lists_nothing() {
        let mut dev = Bench::default();
        dev.info.gain_stages.clear();
        dev.toggles.clear();
        dev.choices.clear();
        assert!(settings_of(&dev).is_empty());
    }

    /// A radio to read settings off, with one stage set by hand, one left to
    /// the driver, a bias tee and an antenna port.
    struct Bench {
        info: common::device::DeviceInfo,
        gains: Vec<(String, GainMode)>,
        toggles: Vec<common::device::Toggle>,
        choices: Vec<common::device::Choice>,
        tuning: common::Tuning,
    }

    impl Default for Bench {
        fn default() -> Self {
            let stage = |name: &str, label: &str, hi: f32| common::device::GainStage {
                name: name.into(),
                label: label.into(),
                range: 0.0..=hi,
                values: Vec::new(),
                step: 0.0,
                auto: true,
            };
            Bench {
                info: common::device::DeviceInfo {
                    kind: DriverKind::RtlSdr,
                    id: "bench".into(),
                    label: "Bench".into(),
                    tuner: "none".into(),
                    ranges: vec![common::device::TunerRange {
                        label: "rx",
                        range: Hz(24_000_000)..=Hz(1_766_000_000),
                    }],
                    rates: Vec::new(),
                    rate_range: Sps(225_000)..=Sps(2_400_000),
                    gain_stages: vec![stage("lna", "LNA", 40.0), stage("vga", "VGA", 60.0)],
                    native_format: SampleFormat::Cu8,
                    usable_bandwidth_ratio: 0.8,
                    tunable: true,
                    tx: None,
                },
                gains: vec![("lna".into(), GainMode::Manual(24.0)), ("vga".into(), GainMode::Auto)],
                toggles: vec![common::device::Toggle {
                    name: "bias_t".into(),
                    label: "Bias tee".into(),
                    help: "feeds the mast head amplifier".into(),
                    on: true,
                }],
                choices: vec![common::device::Choice {
                    name: "antenna".into(),
                    label: "Antenna".into(),
                    help: "which socket the cable is in".into(),
                    options: vec!["LNAH".into(), "LNAL".into(), "LNAW".into()],
                    selected: "LNAW".into(),
                }],
                tuning: common::Tuning::default(),
            }
        }
    }

    impl Device for Bench {
        fn info(&self) -> &common::device::DeviceInfo {
            &self.info
        }
        fn set_center(&mut self, _f: Hz) -> Result<()> {
            Ok(())
        }
        fn center(&self) -> Hz {
            Hz::mhz(433)
        }
        fn tuning(&self) -> &common::Tuning {
            &self.tuning
        }
        fn tuning_mut(&mut self) -> &mut common::Tuning {
            &mut self.tuning
        }
        fn set_rate(&mut self, _r: Sps) -> Result<()> {
            Ok(())
        }
        fn rate(&self) -> Sps {
            Sps(2_400_000)
        }
        fn set_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
            match self.gains.iter_mut().find(|(n, _)| n == stage) {
                Some((_, held)) => {
                    *held = mode;
                    Ok(())
                }
                None => Err(common::Error::other(format!("no {stage} gain"))),
            }
        }
        fn set_toggle(&mut self, name: &str, on: bool) -> Result<()> {
            match self.toggles.iter_mut().find(|t| t.name == name) {
                Some(t) => {
                    t.on = on;
                    Ok(())
                }
                None => Err(common::Error::other(format!("no {name} switch"))),
            }
        }
        fn set_choice(&mut self, name: &str, value: &str) -> Result<()> {
            match self.choices.iter_mut().find(|c| c.name == name) {
                Some(c) => {
                    c.selected = value.to_string();
                    Ok(())
                }
                None => Err(common::Error::other(format!("no {name} to pick"))),
            }
        }
        fn gains(&self) -> Vec<(String, GainMode)> {
            self.gains.clone()
        }
        fn toggles(&self) -> Vec<common::device::Toggle> {
            self.toggles.clone()
        }
        fn choices(&self) -> Vec<common::device::Choice> {
            self.choices.clone()
        }
        fn start_rx(&mut self) -> Result<Box<dyn common::device::RxStream>> {
            Err(common::Error::other("not a real radio"))
        }
    }

    /// A request off the wire lands on whichever of the driver's controls it
    /// names: a gain stage, a switch or an antenna port, and the same name
    /// does not mean two of them.
    #[test]
    fn a_request_reaches_the_control_it_names() {
        let mut dev = Bench::default();
        apply(&mut dev, &iqstream::Ask { name: "lna".into(), value: SettingValue::Gain(31.0) })
            .unwrap();
        apply(&mut dev, &iqstream::Ask { name: "vga".into(), value: SettingValue::Auto }).unwrap();
        apply(
            &mut dev,
            &iqstream::Ask { name: "bias_t".into(), value: SettingValue::Switch(false) },
        )
        .unwrap();
        apply(
            &mut dev,
            &iqstream::Ask { name: "antenna".into(), value: SettingValue::Choice("LNAL".into()) },
        )
        .unwrap();

        let s = settings_of(&dev);
        assert_eq!(s[0].value, SettingValue::Gain(31.0));
        assert_eq!(s[1].value, SettingValue::Auto);
        assert_eq!(s[2].value, SettingValue::Switch(false));
        assert_eq!(s[3].value, SettingValue::Choice("LNAL".into()));

        // What the driver will not do is a fault the caller logs, not a
        // stream that ends.
        assert!(
            apply(&mut dev, &iqstream::Ask { name: "if".into(), value: SettingValue::Gain(10.0) })
                .is_err()
        );
    }
}
