//! Background RF thread: owns the device, publishes spectrum frames, and
//! demodulates whichever channel is selected for audio.

use crate::chain::Plan;
use audio::AudioPlayer;
use common::{C32, GainMode, Hz, Sps};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use std::sync::{
    Arc,
    atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering},
};

mod cmd;
mod device;
mod publish;
mod replay;
mod status;
#[cfg(test)]
pub(crate) mod tests;
mod thread;
mod tx;

pub use cmd::*;
use device::*;
pub use replay::*;
pub use status::*;
use thread::*;
pub use tx::*;

pub struct Radio {
    pub cmd: Sender<Cmd>,
    pub frames: Receiver<Frame>,
    /// Packets decoded anywhere in the span, in the order they were found.
    pub decodes: Receiver<Vec<crate::row::Reception>>,
    pub status: Arc<Status>,
    handle: Option<std::thread::JoinHandle<()>>,
}

/// How many devices have been opened since the process started.
///
/// A claim taken on a radio nobody named is invisible from the outside, so
/// the count is kept where the claim is taken and a test can read it back.
static OPENED: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

impl Radio {
    /// Devices opened so far, counting each start of a radio thread.
    pub fn opened() -> usize {
        OPENED.load(Ordering::Relaxed)
    }

    /// Start streaming from an RTL-SDR. `repaint` is called on every frame so
    /// the UI wakes without polling.
    #[allow(clippy::too_many_arguments)]
    pub fn start(
        entry: crate::devices::Entry,
        center: Hz,
        rate: Sps,
        // The local oscillator of whatever is on the cable, which the dial
        // reads above the tuner. Zero for an aerial.
        offset: f64,
        fft: usize,
        repaint: impl Fn() + Send + 'static,
    ) -> Self {
        let (cmd_tx, cmd_rx) = bounded(64);
        // Depth 2: the UI only ever draws the newest spectrum, so queuing more
        // just adds latency between the radio and what is on screen.
        let (frame_tx, frame_rx) = bounded(2);
        // Deeper than the spectrum queue, and for the opposite reason: a
        // dropped spectrum frame is replaced 30 times a second, but a dropped
        // decode is a packet that will not come again.
        let (dec_tx, dec_rx) = bounded(64);
        let status = Arc::new(Status::default());
        let st = status.clone();
        OPENED.fetch_add(1, Ordering::Relaxed);

        let handle = std::thread::Builder::new()
            .name("radio".into())
            .spawn(move || {
                if let Err(e) =
                    run(entry, center, rate, offset, fft, cmd_rx, frame_tx, dec_tx, &st, repaint)
                {
                    *st.error.lock() = Some(e.to_string());
                }
                st.running.store(false, Ordering::Relaxed);
            })
            .expect("spawn radio thread");

        Self { cmd: cmd_tx, frames: frame_rx, decodes: dec_rx, status, handle: Some(handle) }
    }

    /// The whole receiver on a radio somebody else opened.
    ///
    /// For a test: `sources::FileRadio` hears a capture and keeps what it
    /// transmits, so everything from a command arriving to a sample reaching
    /// the antenna runs exactly as it does on a HackRF. Nothing above this is
    /// test-only, which is the point.
    #[cfg(test)]
    pub fn on_device(dev: Box<dyn common::Device>, center: Hz, rate: Sps, fft: usize) -> Self {
        Self::on_device_hearing(dev, center, rate, fft, None)
    }

    /// The same, with the microphone handed in rather than opened: a test
    /// radio hears the speech it was given and never the room.
    #[cfg(test)]
    pub fn on_device_hearing(
        dev: Box<dyn common::Device>,
        center: Hz,
        rate: Sps,
        fft: usize,
        mic: Option<Arc<dyn audio::AudioSource>>,
    ) -> Self {
        let (cmd_tx, cmd_rx) = bounded(64);
        let (frame_tx, frame_rx) = bounded(2);
        let (dec_tx, dec_rx) = bounded(64);
        let status = Arc::new(Status::default());
        let st = status.clone();
        let handle = std::thread::Builder::new()
            .name("radio".into())
            .spawn(move || {
                let built = RadioThread::with_device(
                    dev,
                    None,
                    center,
                    rate,
                    0.0,
                    fft,
                    cmd_rx,
                    frame_tx,
                    dec_tx,
                    &st,
                    || {},
                );
                let ran = match built {
                    Ok(mut t) => {
                        if let Some(src) = mic {
                            t.hand_microphone(src);
                        }
                        t.run()
                    }
                    Err(e) => Err(e),
                };
                if let Err(e) = ran {
                    *st.error.lock() = Some(e.to_string());
                }
                st.running.store(false, Ordering::Relaxed);
            })
            .expect("spawn radio thread");
        Self { cmd: cmd_tx, frames: frame_rx, decodes: dec_rx, status, handle: Some(handle) }
    }

    pub fn send(&self, c: Cmd) {
        let _ = self.cmd.try_send(c);
    }
}

/// How long a radio is given to stop before it is abandoned.
///
/// A USB call that never returns is not a thing this process can cancel, so
/// the choice is between waiting for it and leaving it. Waiting means the
/// window is frozen with nothing on screen to say why, which is how a radio
/// that stopped responding took the whole interface with it; leaving it means
/// a thread and a USB claim are held until the process exits, and the
/// operator can carry on, change device, or close the window.
const STOP_GRACE: std::time::Duration = std::time::Duration::from_millis(1500);

impl Drop for Radio {
    fn drop(&mut self) {
        self.send(Cmd::Stop);
        let Some(h) = self.handle.take() else { return };
        // Joined on another thread, so this one can give up on it. The
        // handle is moved in, so abandoning it leaks a thread rather than
        // leaving a dangling join.
        let (tx, rx) = bounded::<()>(1);
        let waiter = std::thread::Builder::new().name("radio-stop".into()).spawn(move || {
            let _ = h.join();
            let _ = tx.send(());
        });
        if waiter.is_err() {
            return;
        }
        if rx.recv_timeout(STOP_GRACE).is_err() {
            tracing::warn!(
                "the radio did not stop within {:?}; abandoning its thread and USB claim",
                STOP_GRACE
            );
        }
    }
}

/// One listening channel on its own, for tests and benchmarks.
///
/// A thin holder around a [`crate::chain::Receiver`] carrying a single
/// channel, so that measuring a chain measures the chain the receiver builds.
/// It used to construct its own copy of the audio branch, which drifted:
/// whatever was true of a filter here was not necessarily true of the one the
/// radio ran.
#[cfg_attr(not(test), allow(dead_code))]
pub struct Audio {
    rx: crate::chain::Receiver,
    pcm: Vec<f32>,
}

#[cfg_attr(not(test), allow(dead_code))]
impl Audio {
    pub fn new(offset: f64, rate: f64, mode: Demod, _target: f64) -> Self {
        Self::of(Self::spec(offset, mode), rate)
    }

    fn spec(offset: f64, mode: Demod) -> ChannelSpec {
        ChannelSpec {
            id: 1,
            label: String::new(),
            offset_hz: offset,
            mode: ChanMode::Audio(mode),
            bandwidth_hz: None,
            audio_low_hz: None,
            squelch_db: None,
            voice: false,
            reads: None,
            agc: true,
            blanker: None,
            denoise: false,
            denoise_db: dsp::denoise::DEFAULT_DEPTH_DB,
            notch: false,
            tx: None,
            tone: None,
        }
    }

    fn of(spec: ChannelSpec, rate: f64) -> Self {
        let plan = Plan {
            center: Hz(0),
            rate,
            zoom: 1,
            dc_block: false,
            refresh_hz: 30.0,
            smoothing: crate::chain::DEFAULT_SMOOTHING,
            trace: dsp::spectrum::Detector::Average,
            wf_detector: dsp::spectrum::Detector::Peak,
            fft: 1024,
            channels: vec![spec],
            fronts: Vec::new(),
            edits: Default::default(),
            record: false,
            capture: false,
            heat: Default::default(),
            capture_dir: crate::chain::default_capture_dir(),
            capture_format: common::SampleFormat::Cu8,
            capture_arm: Default::default(),
            log: false,
            calls: None,
            transcribe: false,
            transcribe_model: String::new(),
            transcribe_device: String::new(),
            feeds: Vec::new(),
            iqstream: None,
            iqstream_tuners: Vec::new(),
            kiss: None,
            seams: Vec::new(),
            tx: None,
            tx_capture: None,
            rds: None,
            scan: Default::default(),
            settings: Default::default(),
        };
        let rx = crate::chain::Receiver::build(&plan, Default::default()).expect("audio chain");
        Self { rx, pcm: Vec::new() }
    }

    fn chan(&self) -> &crate::chain::Chan {
        &self.rx.channels()[0]
    }

    pub fn cost(&self) -> String {
        self.chan().detail.clone()
    }

    pub fn latency_ms(&self) -> f64 {
        self.rx.latency_ms(0)
    }

    /// How much gain the AGC is applying, or zero in a mode without one.
    pub fn agc_gain_db(&self) -> f32 {
        self.chan().agc_gain_db
    }

    /// What the squelch measured on the last block, in dB.
    pub fn squelch_db(&self) -> f32 {
        self.chan().squelch_db
    }

    pub fn audio_rate(&self) -> f64 {
        self.chan().audio_rate
    }

    pub fn topology(&self) -> pipeline::graph::Topology {
        self.rx.topology()
    }

    pub fn process(&mut self, input: &[C32], gain: f32) -> &[f32] {
        self.pcm.clear();
        if self.rx.process(input).is_err() {
            return &self.pcm;
        }
        self.pcm.extend(self.rx.channel_audio(0).iter().map(|v| v * gain));
        &self.pcm
    }
}

#[allow(clippy::too_many_arguments)]
fn run(
    entry: crate::devices::Entry,
    center: Hz,
    rate: Sps,
    offset: f64,
    fft: usize,
    cmd: Receiver<Cmd>,
    frames: Sender<Frame>,
    decodes: Sender<Vec<crate::row::Reception>>,
    status: &Status,
    repaint: impl Fn(),
) -> anyhow::Result<()> {
    RadioThread::open(entry, center, rate, offset, fft, cmd, frames, decodes, status, repaint)?
        .run()
}

/// Tell the strip what the nodes hold, when it differs from what it was
/// last told.
///
/// A fader or a squelch set through the chain view lands on the node, and
/// the strip is what the operator reads, so it has to follow. A squelch or
/// a gain control is also a plan value, which the next rebuild draws from,
/// so the plan follows too; a level is not, since the fader keeps it. A
/// revision moves only when something changed, so what the strip sends
/// itself does not come back to it.
fn pull_levels(rx: &crate::chain::Receiver, plan: &mut Plan, status: &Status) {
    let (audio, chans) = rx.levels();
    let was = status.levels();
    for c in &chans {
        if let Some(have) = plan.channels.iter_mut().find(|h| h.id == c.id) {
            have.label = c.label.clone();
            have.squelch_db = c.squelch_db;
            have.agc = c.agc;
            have.blanker = c.blanker;
            have.denoise = c.denoise;
            have.denoise_db = c.denoise_db;
            have.notch = c.notch;
        }
    }
    if audio != was.audio || chans != was.channels {
        status.set_levels(audio, chans);
    }
}

/// Every front end the span covers, or none of them.
///
/// `decode_on` is the operator's own switch: turning decoding off stops the
/// front ends being built at all, which is the expensive thing the receiver
/// does. It does not change which of them belong here.
fn fronts_here(
    scanners: &crate::scanners::Scanners,
    plan: &Plan,
    decode_on: bool,
) -> Vec<crate::scanners::FrontAt> {
    if !decode_on {
        return Vec::new();
    }
    scanners.fronts(crate::scanners::Span::new(plan.center.as_f64(), plan.eff_rate()))
}

/// Publish the chain the receiver is running, for the chain view.
///
/// There is one graph and it holds everything, so this is no longer a choice
/// between chains: what is drawn is what runs.
fn publish_chain(status: &Status, rx: &crate::chain::Receiver) {
    // Both chains as one: the receiver's, and the transmitter's from the
    // thread that runs it, with its ids moved out of the way.
    status.set_chain(
        Some(crate::transmit::merged(&rx.topology(), rx.tx_topology().as_ref())),
        rx.latency_ms(0),
    );
    *status.waiting.lock() = rx.waiting.clone();
}

/// A plan that only scans, for tests about the shape of the receiver.
#[cfg(test)]
fn plan_at(rate: f64, center: Hz) -> Plan {
    Plan {
        center,
        rate,
        iqstream: None,
        iqstream_tuners: Vec::new(),
        zoom: 1,
        dc_block: false,
        refresh_hz: 30.0,
        smoothing: crate::chain::DEFAULT_SMOOTHING,
        trace: dsp::spectrum::Detector::Average,
        wf_detector: dsp::spectrum::Detector::Peak,
        fft: 1024,
        channels: Vec::new(),
        fronts: vec![crate::scanners::FrontAt {
            front: crate::scanners::Front::Banks(crate::scanners::DEFAULT_WIDTHS.to_vec()),
            // The whole span: these tests are about the shape of the
            // receiver, not about which band a block covers.
            band: (0.0, f64::INFINITY),
        }],
        scan: Default::default(),
        heat: Default::default(),
        edits: Default::default(),
        record: false,
        capture: false,
        capture_dir: crate::chain::default_capture_dir(),
        capture_format: common::SampleFormat::Cu8,
        capture_arm: Default::default(),
        log: false,
        calls: None,
        transcribe: false,
        transcribe_model: String::new(),
        transcribe_device: String::new(),
        feeds: Vec::new(),
        kiss: None,
        seams: Vec::new(),
        tx: None,
        tx_capture: None,
        rds: None,
        settings: Default::default(),
    }
}
