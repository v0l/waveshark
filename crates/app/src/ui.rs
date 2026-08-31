//! Instrument front panel: readout, spectrum, waterfall, channel strips.

use crate::bands;
use crate::dial::Dial;
use crate::radio::{
    ChannelSpec, ChannelState, Cmd, DecodeRecord, Demod, Frame, Radio, StationInfo,
};
use crate::theme::{self, legend, value};
use crate::waterfall::Waterfall;
use crate::wheel::Wheel;
use common::{GainMode, Hz, Sps};
use egui::containers::{CentralPanel, Panel};
use egui::{Align2, Color32, FontFamily, FontId, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};

pub struct App {
    radio: Option<Radio>,
    err: Option<String>,

    center: f64,
    rate: f64,

    db: Vec<f32>,
    wf: Waterfall,
    floor: f32,
    ceil: f32,
    auto_scale: bool,

    dial: Dial,
    /// Centre the waterfall history currently corresponds to, so a retune can
    /// slide it instead of throwing it away.
    wf_center: f64,
    /// Centre frequency of the spectrum currently held in `db`, which lags the
    /// requested centre while a retune is pending.
    db_center: f64,
    wf_pending: Vec<f32>,
    wf_last: Option<std::time::Instant>,
    rows_per_sec: f32,
    refresh: f32,
    fft_size: usize,
    scrub: Wheel,
    open: Option<Settings>,
    smoothing: f32,
    wf_top_offset: f32,
    wf_rows: usize,
    channels: Vec<Channel>,
    /// The channel whose chain the signal chain view shows.
    listening: Option<usize>,
    volume: f32,
    next_id: u32,
    fft: usize,
    devices: Vec<crate::devices::Entry>,
    device: Option<crate::devices::Entry>,
    spans: Vec<crate::devices::Span>,
    /// Software decimation currently applied, 1 for none.
    zoom: usize,
    /// Run for this many seconds, report CPU used, then quit.
    pub soak: Option<f32>,
    /// Save a PNG to this path once the radio has settled, then quit.
    pub shot: Option<String>,
    /// Where bursts are being written and how much may be written, when
    /// recording.
    record_dir: Option<(std::path::PathBuf, Option<u64>)>,
    shot_at: Option<std::time::Instant>,
    shot_sent: bool,
    /// Remove the direct-conversion centre spur. On by default: it is an
    /// artefact of the receiver, not something being received.
    dc_block: bool,
    view: View,
    /// Shape of the running chain, republished by the radio thread whenever it
    /// rebuilds one. Cloned rather than shared so drawing never blocks the
    /// thread that has to keep draining USB.
    chain_topo: Option<pipeline::graph::Topology>,
    chain_latency: f64,
    /// Packets decoded anywhere in the span, oldest first.
    decodes: Vec<Logged>,
    /// Number given to the next packet.
    next_packet: u64,
    /// Packet whose bytes are shown in the dump.
    selected: Option<u64>,
    /// Show bursts no protocol claimed.
    show_unknown: bool,
    /// Decoding every channel is on by default and can be turned off; it is
    /// the most expensive thing the app does.
    decode_on: bool,
    /// Whether the packet log is showing.
    log_open: bool,
    /// Share of the scope pane given to the spectrum, the rest going to the
    /// waterfall. Dragged rather than fixed: which of the two matters depends
    /// entirely on what is being looked for.
    plot_frac: f32,
    /// The split between spectrum and waterfall is being dragged.
    splitting: bool,
    /// Channel whose marker is being dragged in the spectrum.
    drag_ch: Option<usize>,
    /// Shared per-digit readout for the channel strip. Only one channel can be
    /// under the pointer, so one is enough.
    chan_dial: crate::dial::Dial,
    /// Settings as last written to disk, and when. Compared against the live
    /// ones each frame rather than tracked with a dirty flag, because every
    /// control that changes one would otherwise have to remember to set it.
    /// Every packet, appended to disk as it arrives. On unless the receiver
    /// has nowhere to write, which is the case in tests.
    /// Where the packet log is being written, for the status line. The log
    /// itself lives in the graph, on the radio thread.
    packet_log: Option<std::path::PathBuf>,
    /// Aircraft, folded together from the ADS-B packets in the same stream.
    flights: crate::flights::Flights,
    /// Where the receiver is, when it has been told.
    location: Option<(f64, f64)>,
    saved: crate::session::Session,
    saved_at: Option<std::time::Instant>,
    /// Gain stages and switches restored from the session, waiting for the
    /// radio to report its controls so they can be applied to it.
    pending_radio: Option<crate::session::Session>,
}

/// Which settings panel is open. Each pane owns its own, because spectrum and
/// waterfall settings are unrelated and lumping them together makes both
/// harder to find.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Settings {
    Spectrum,
    Waterfall,
    /// The radio's own controls: gain stages, its switches, and the
    /// corrections applied to what comes off it.
    Radio,
}

/// A decode, as shown in the packet log.
pub struct Logged {
    /// Position in the capture, counted from the first packet and never
    /// reused, so a row keeps its number as the list scrolls.
    id: u64,
    rec: DecodeRecord,
}

pub struct Channel {
    /// Stable for the life of the channel, so the radio thread can keep its
    /// chain when a different channel is removed.
    id: u64,
    freq: f64,
    demod: Demod,
    label: String,
    /// Whether this channel is being demodulated into the mix.
    on: bool,
    /// Its own level in the mix, before the master volume.
    volume: f32,
    muted: bool,
    /// Where the squelch opens. None means the mode's own default, which is
    /// what an operator who has never touched the control should get.
    squelch_db: Option<f32>,
    agc: bool,
}

const FFTS: [usize; 6] = [512, 1024, 2048, 4096, 8192, 16384];
/// Spectrum refresh rates in frames per second.
const REFRESH: [(&str, f32); 4] = [("10", 10.0), ("20", 20.0), ("30", 30.0), ("60", 60.0)];
/// Waterfall scroll rates in rows per second.
const SPEEDS: [(&str, f32); 5] = [
    ("5", 5.0),
    ("10", 10.0),
    ("20", 20.0),
    ("40", 40.0),
    ("80", 80.0),
];

/// What the main pane shows.
#[derive(Clone, Copy, PartialEq, Eq)]
enum View {
    Spectrum,
    Chain,
    Flights,
}

impl View {
    fn label(self) -> &'static str {
        match self {
            View::Spectrum => "Spectrum",
            View::Chain => "Signal chain",
            View::Flights => "Flights",
        }
    }
}

/// Packets kept in the log. About a screenful of scrollback at any plausible
/// reading speed, and bounded memory on a band that never goes quiet.
const DECODE_LOG_MAX: usize = 500;

/// Colour of a packet whose integrity check passed.
const CRC_OK: Color32 = Color32::from_rgb(0x6F, 0xD1, 0x8A);

/// Share of the scope pane the spectrum gets by default.
const DEFAULT_PLOT_FRAC: f32 = 0.34;
/// Range the split can be dragged to. Neither pane may be squeezed to nothing:
/// a two pixel waterfall is not a smaller waterfall, it is a broken one.
const PLOT_FRAC_RANGE: std::ops::RangeInclusive<f32> = 0.12..=0.85;
/// Height of the drag handle between the two, in pixels.
const SPLIT_GRIP_H: f32 = 7.0;

/// How near the pointer must be to a channel marker to grab it, in pixels.
///
/// Must exceed egui's drag threshold, or the pointer leaves the marker before
/// the drag is reported and the grab is never seen.
const GRAB_PX: f64 = 10.0;

/// Rate limits per driver, used to build the span list before the device is
/// opened. The driver reports the same numbers through `DeviceInfo`.
fn device_rates(e: &crate::devices::Entry) -> std::ops::RangeInclusive<Sps> {
    match e.kind {
        common::DriverKind::HackRf => Sps(2_000_000)..=Sps(20_000_000),
        _ => Sps(225_000)..=Sps(2_400_000),
    }
}

impl Default for App {
    fn default() -> Self {
        Self {
            record_dir: None,
            radio: None,
            err: None,
            center: crate::session::DEFAULT_CENTER,
            rate: 2_304_000.0,
            db: Vec::new(),
            wf: Waterfall::new(512),
            floor: -90.0,
            ceil: -20.0,
            auto_scale: true,
            dial: Dial::new(),
            wf_center: crate::session::DEFAULT_CENTER,
            db_center: crate::session::DEFAULT_CENTER,
            wf_pending: Vec::new(),
            wf_last: None,
            rows_per_sec: 20.0,
            refresh: 30.0,
            fft_size: 2048,
            scrub: Wheel::default(),
            open: None,
            smoothing: 0.35,
            wf_top_offset: 5.0,
            wf_rows: 512,
            channels: Vec::new(),
            listening: None,
            volume: 0.5,
            next_id: 1,
            fft: 2048,
            devices: Vec::new(),
            device: None,
            spans: Vec::new(),
            zoom: 1,
            soak: None,
            shot: None,
            decodes: Vec::new(),
            next_packet: 1,
            selected: None,
            show_unknown: true,
            decode_on: true,
            log_open: true,
            plot_frac: DEFAULT_PLOT_FRAC,
            splitting: false,
            drag_ch: None,
            chan_dial: crate::dial::Dial::new(),
            shot_at: None,
            shot_sent: false,
            dc_block: true,
            view: View::Spectrum,
            chain_topo: None,
            chain_latency: 0.0,
            packet_log: None,
            flights: crate::flights::Flights::new(),
            location: None,
            saved: crate::session::Session::default(),
            saved_at: None,
            pending_radio: None,
        }
    }
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        theme::install(&cc.egui_ctx);
        let devices = crate::devices::list();
        let s = crate::session::Session::load();
        // The saved radio may not be plugged in any more, in which case the
        // rest of the session still applies to whatever is.
        let device = s
            .device
            .as_deref()
            .and_then(|want| devices.iter().find(|d| d.label == want).cloned())
            .or_else(|| devices.first().cloned());
        let mut app = Self {
            devices,
            device,
            packet_log: crate::packetlog::PacketLog::default_dir(),
            flights: {
                let mut f = crate::flights::Flights::new();
                if let Some((lat, lon)) = s.location {
                    f.set_reference(lat, lon);
                }
                f
            },
            center: s.center,
            wf_center: s.center,
            db_center: s.center,
            // The file holds the device's own rate; the app works in the
            // effective one, which zoom divides.
            rate: s.rate / s.zoom.max(1) as f64,
            zoom: s.zoom,
            fft: s.fft,
            fft_size: s.fft,
            dc_block: s.dc_block,
            decode_on: s.decode_on,
            location: s.location,
            volume: s.volume,
            saved: s.clone(),
            pending_radio: Some(s),
            ..Default::default()
        };
        app.connect(&cc.egui_ctx);
        app
    }

    /// The live settings, in the form they are stored in.
    fn session(&self) -> crate::session::Session {
        let radio = self.radio.as_ref().map(|r| r.status.radio());
        crate::session::Session {
            device: self.device.as_ref().map(|d| d.label.clone()),
            center: self.center,
            rate: self.rate * self.zoom.max(1) as f64,
            zoom: self.zoom,
            fft: self.fft,
            // Read back from the driver rather than from what was asked for,
            // so the file holds the gain the hardware actually took.
            gains: radio
                .as_ref()
                .map(|r| r.stages.iter().map(|(s, m)| (s.name.clone(), *m)).collect())
                .unwrap_or_else(|| self.saved.gains.clone()),
            toggles: radio
                .as_ref()
                .map(|r| r.toggles.iter().map(|t| (t.name.clone(), t.on)).collect())
                .unwrap_or_else(|| self.saved.toggles.clone()),
            ppm: radio.as_ref().map(|r| r.ppm).unwrap_or(self.saved.ppm),
            location: self.location,
            dc_block: self.dc_block,
            decode_on: self.decode_on,
            volume: self.volume,
        }
    }

    /// Write the session out when it has changed and settled.
    ///
    /// Debounced because dragging the dial changes the centre on every frame,
    /// and a file written sixty times a second to record a frequency nobody
    /// stopped on is a lot of writes for no information.
    fn save_session(&mut self) {
        let now = self.session();
        if now == self.saved {
            return;
        }
        let due = self.saved_at.is_none_or(|t| t.elapsed().as_secs_f32() >= 2.0);
        if !due {
            return;
        }
        now.save();
        self.saved = now;
        self.saved_at = Some(std::time::Instant::now());
    }

    /// Push restored gain stages and switches at the radio once it is up.
    ///
    /// Deferred rather than sent with the tuning: the stage names come from
    /// the driver, and until it has reported them there is nothing to check a
    /// saved name against.
    fn restore_radio_settings(&mut self) {
        let Some(want) = self.pending_radio.clone() else { return };
        let Some(radio) = self.radio.as_ref() else { return };
        let controls = radio.status.radio();
        if controls.stages.is_empty() && controls.toggles.is_empty() {
            return;
        }
        self.pending_radio = None;
        for (name, mode) in &want.gains {
            if controls.stages.iter().any(|(s, _)| &s.name == name) {
                self.send(Cmd::GainStage(name.clone(), *mode));
            }
        }
        for (name, on) in &want.toggles {
            if controls.toggles.iter().any(|t| &t.name == name) {
                self.send(Cmd::Toggle(name.clone(), *on));
            }
        }
        if want.ppm != 0.0 {
            self.send(Cmd::Ppm(want.ppm));
        }
        if !want.dc_block {
            self.send(Cmd::DcBlock(false));
        }
        if !want.decode_on {
            self.send(Cmd::Decode(false));
        }
    }

    /// Tell the tracker where the receiver is, so a single position frame
    /// resolves instead of waiting for a matching pair.
    pub fn set_location(&mut self, lat: f64, lon: f64) {
        self.location = Some((lat, lon));
        self.flights.set_reference(lat, lon);
    }

    /// Turn the packet log off, or point it somewhere other than the default.
    pub fn set_packet_log(&mut self, off: bool, dir: Option<std::path::PathBuf>) {
        self.packet_log = if off { None } else { dir.or_else(crate::packetlog::PacketLog::default_dir) };
        let dir = self.packet_log.clone();
        self.send(Cmd::PacketLog(dir));
    }

    /// Record every burst that decodes into a directory of captures.
    ///
    /// Held rather than sent once: choosing a device, or changing the span,
    /// starts a new radio thread, and recording that quietly stopped when the
    /// UI reconnected would be worse than not recording at all.
    pub fn record_to(&mut self, dir: std::path::PathBuf, budget_mb: Option<u64>) {
        self.record_dir = Some((dir.clone(), budget_mb));
        self.send(Cmd::Record(Some((dir, budget_mb))));
    }

    /// Pick the span closest to `hz`, narrowing in software if the radio
    /// cannot sample that slowly.
    pub fn set_span(&mut self, hz: f64) {
        let Some(sp) = self
            .spans
            .iter()
            .min_by(|a, b| (a.effective() - hz).abs().total_cmp(&(b.effective() - hz).abs()))
            .cloned()
        else {
            return;
        };
        self.send(Cmd::Rate(common::Sps(sp.rate as u64)));
        self.send(Cmd::Zoom(sp.zoom));
        self.rate = sp.effective();
        self.zoom = sp.zoom;
        self.reset_waterfall();
        self.retune_listener();
    }

    /// Start tuned to a station and listening to it.
    ///
    /// Useful for screenshots and for checking a change against real RF
    /// without a dozen clicks first.
    pub fn tune_to(&mut self, mhz: f64, demod: Demod) {
        let freq = mhz * 1e6;
        // The first channel sets where the receiver points; later ones are
        // added around it, since a second call moving the centre would drag
        // the first channel to the edge of the span or out of it.
        if self.channels.is_empty() {
            self.center = freq;
            self.send(Cmd::Center(common::Hz(freq as u64)));
        }
        self.channels.push(Channel {
            id: self.next_id as u64,
            freq,
            demod,
            label: format!("{mhz:.1}"),
            on: true,
            volume: 0.8,
            muted: false,
            squelch_db: None,
            agc: true,
        });
        self.next_id += 1;
        self.listen(self.channels.len() - 1);
    }

    fn connect(&mut self, ctx: &egui::Context) {
        // Dropping the old Radio stops its thread and releases the USB claim
        // before the next one tries to take it.
        self.radio = None;
        self.err = None;
        let Some(entry) = self.device.clone() else {
            self.err = Some("no radio found. plug one in, then press RESCAN.".into());
            return;
        };
        self.spans = crate::devices::spans_with_zoom(&device_rates(&entry));
        if !self.spans.iter().any(|s| (s.effective() - self.rate).abs() < 1.0) {
            self.rate = self.spans.last().map(|s| s.effective()).unwrap_or(self.rate);
            self.zoom = 1;
        }
        let c = ctx.clone();
        self.radio = Some(Radio::start(
            entry,
            Hz(self.center as u64),
            // The radio samples at the full rate and the zoom narrows it in
            // software, so it is started at the rate before that division.
            Sps((self.rate * self.zoom.max(1) as f64).round() as u64),
            self.fft,
            move || c.request_repaint(),
        ));
        if self.zoom > 1 {
            self.send(Cmd::Zoom(self.zoom));
        }
        if let Some(r) = self.record_dir.clone() {
            self.send(Cmd::Record(Some(r)));
        }
        // The log is a node in the graph, so a new radio thread means a new
        // graph and it has to be told where to write again.
        if let Some(d) = self.packet_log.clone() {
            self.send(Cmd::PacketLog(Some(d)));
        }
        // Whatever the radio was set to last time has to be pushed at it
        // again: a new thread means a freshly opened device at its defaults.
        self.pending_radio = Some(self.saved.clone());
        self.reset_waterfall();
    }

    /// Release the radio without quitting.
    ///
    /// Dropping it stops the thread and gives up the USB claim; a stale
    /// process holding that claim is why a second program fails to open the
    /// device at all.
    fn stop(&mut self) {
        self.radio = None;
        self.listening = None;
        self.err = None;
    }

    fn select_device(&mut self, ctx: &egui::Context, e: crate::devices::Entry) {
        if self.device.as_ref() == Some(&e) {
            return;
        }
        self.device = Some(e);
        self.listening = None;
        self.connect(ctx);
    }

    fn send(&self, c: Cmd) {
        if let Some(r) = &self.radio {
            r.send(c);
        }
    }

    fn drain(&mut self) {
        let Some(radio) = &self.radio else { return };
        if let Some(e) = radio.status.error.lock().take() {
            self.err = Some(e);
        }
        let mut latest: Option<Frame> = None;
        while let Ok(f) = radio.frames.try_recv() {
            latest = Some(f);
        }
        self.chain_topo = radio.status.chain();
        self.chain_latency = radio.status.chain_latency();
        let mut batches = Vec::new();
        while let Ok(batch) = radio.decodes.try_recv() {
            batches.push(batch);
        }
        for b in batches {
            self.log_decodes(b);
        }
        if let Some(f) = latest {
            // The requested centre is not overwritten by the frame's. Retunes
            // are spaced out because each blocks the radio thread, so frames
            // arrive from the old frequency for a while after a drag moves the
            // view. Adopting their centre would drag the view back under the
            // pointer every time one landed.
            self.db_center = f.center;
            self.rate = f.rate;
            if self.auto_scale {
                self.rescale(&f.db);
            }
            self.slide_waterfall(f.center, f.db.len());

            // Hold the peak between rows rather than sampling one frame in N,
            // or a short burst lands between rows and is never drawn.
            if self.wf_pending.len() != f.db.len() {
                self.wf_pending = f.db.clone();
            } else {
                for (a, b) in self.wf_pending.iter_mut().zip(&f.db) {
                    *a = a.max(*b);
                }
            }
            let due = self
                .wf_last
                .map(|t| t.elapsed().as_secs_f32() >= 1.0 / self.rows_per_sec)
                .unwrap_or(true);
            if due {
                // The waterfall tops out below the trace's ceiling: the plot
                // wants headroom so peaks are not clipped flat, the colour
                // ramp wants the opposite or its hottest colours go unused.
                let pending = std::mem::take(&mut self.wf_pending);
                    self.wf.push(&pending, self.floor, self.ceil - self.wf_top_offset);
                self.wf_pending = pending;
                self.wf_pending.fill(f32::MIN);
                self.wf_last = Some(std::time::Instant::now());
            }
            self.db = f.db;
        }
    }

    /// Add decoded packets to the on-screen list, oldest first.
    ///
    /// Nothing is written here. The packet log is a node in the graph and
    /// stores what the demodulators produced, which is a better record than
    /// this list: these are conclusions, and they are bounded.
    fn log_decodes(&mut self, batch: Vec<DecodeRecord>) {
        for rec in batch {
            self.flights.update(&rec);
            let id = self.next_packet;
            self.next_packet += 1;
            self.decodes.push(Logged { id, rec });
        }
        // A busy band produces packets faster than anyone reads them, and an
        // unbounded log is a slow memory leak with a scrollbar.
        if self.decodes.len() > DECODE_LOG_MAX {
            let drop = self.decodes.len() - DECODE_LOG_MAX;
            self.decodes.drain(..drop);
            // A selection that has aged out of the list must not leave the
            // dump showing bytes with no row above them.
            if self.selected.is_some_and(|id| !self.decodes.iter().any(|l| l.id == id)) {
                self.selected = None;
            }
        }
    }

    /// Slide the waterfall to match a new centre frequency.
    fn slide_waterfall(&mut self, center: f64, bins: usize) {
        if bins == 0 {
            return;
        }
        // Aligned to where the rows' data actually is, not to where the view
        // has been moved to, or the history smears as the two drift apart.
        let hz_per_bin = self.rate / bins as f64;
        let d = ((center - self.wf_center) / hz_per_bin).round();
        if d != 0.0 {
            self.wf.shift(d as i32);
            self.wf_center += d * hz_per_bin;
        }
    }

    /// Track percentiles, not extremes: one strong carrier would otherwise
    /// flatten everything else in the span.
    fn rescale(&mut self, db: &[f32]) {
        let mut v: Vec<f32> = db.iter().copied().filter(|x| x.is_finite()).collect();
        if v.is_empty() {
            return;
        }
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let pct = |p: f32| v[((v.len() - 1) as f32 * p) as usize];
        let (lo, hi) = (pct(0.10) - 6.0, pct(0.999) + 3.0);
        self.floor += (lo - self.floor) * 0.05;
        self.ceil += (hi.max(lo + 20.0) - self.ceil) * 0.05;
    }

    /// Frequency under the pointer, snapped to the band's channel plan while
    /// shift is held.
    ///
    /// Snapping is opt-in rather than always on because most of the spectrum
    /// has no legal raster, and a band that does still carries signals off it.
    fn hz_at_snapped(&self, rect: &Rect, x: f32, ui: &egui::Ui) -> f64 {
        let hz = self.hz_at(rect, x);
        if ui.input(|i| i.modifiers.shift) {
            bands::snap(hz)
        } else {
            hz
        }
    }

    fn hz_at(&self, rect: &Rect, x: f32) -> f64 {
        let t = ((x - rect.left()) / rect.width()).clamp(0.0, 1.0) as f64;
        self.center - self.rate / 2.0 + t * self.rate
    }

    /// Index of the channel marker within grabbing distance of `x`.
    ///
    /// Tolerance is in pixels, not Hz: the marker is a line on screen and the
    /// pointer is aiming at that line, so how close a grab counts as a hit must
    /// not change with the span.
    fn channel_at(&self, rect: &Rect, x: f32) -> Option<usize> {
        let tol = GRAB_PX * self.rate / rect.width().max(1.0) as f64;
        let hz = self.hz_at(rect, x);
        self.channels
            .iter()
            .enumerate()
            .filter(|(_, c)| (c.freq - hz).abs() < tol)
            .min_by(|a, b| {
                (a.1.freq - hz).abs().partial_cmp(&(b.1.freq - hz).abs()).unwrap()
            })
            .map(|(i, _)| i)
    }

    fn x_of(&self, rect: &Rect, hz: f64) -> f32 {
        let t = (hz - (self.center - self.rate / 2.0)) / self.rate;
        rect.left() + (t as f32) * rect.width()
    }

    fn retune(&mut self, hz: f64) {
        self.center = hz.clamp(24e6, 1766e6);
        self.send(Cmd::Center(Hz(self.center as u64)));
        self.retune_listener();
    }

    /// The span or bin count changed, so old rows no longer line up.
    fn reset_waterfall(&mut self) {
        self.wf.clear();
        self.wf_center = self.center;
        self.wf_pending.clear();
    }

    fn add_channel(&mut self, freq: f64) {
        let id = self.next_id;
        self.next_id += 1;
        self.channels.push(Channel {
            id: id as u64,
            freq,
            demod: bands::demod_at(freq),
            label: format!("CH{id}"),
            on: true,
            volume: 0.8,
            muted: false,
            squelch_db: None,
            agc: true,
        });
        self.listening = Some(self.channels.len() - 1);
        self.send_channels();
    }

    /// Hand the radio the whole channel list.
    ///
    /// The whole list rather than an edit, because the radio thread is the
    /// one that knows which chains it already has: sending it the state it
    /// should be in leaves no way for the two to disagree, and it keeps the
    /// chains of channels that did not change.
    fn send_channels(&mut self) {
        let specs = self.channel_specs();
        self.send(Cmd::Channels(specs));
    }

    fn channel_specs(&self) -> Vec<ChannelSpec> {
        let center = self.center;
        self.channels
            .iter()
            .filter(|c| c.on)
            .map(|c| ChannelSpec {
                id: c.id,
                offset_hz: c.freq - center,
                demod: c.demod,
                volume: c.volume,
                muted: c.muted,
                squelch_db: c.squelch_db,
                agc: c.agc,
            })
            .collect()
    }

    fn listen(&mut self, idx: usize) {
        if let Some(ch) = self.channels.get_mut(idx) {
            ch.on = true;
        }
        self.listening = Some(idx);
        self.send_channels();
    }

    fn retune_listener(&mut self) {
        if self.listening.is_some_and(|i| i >= self.channels.len()) {
            self.listening = None;
        }
        self.send_channels();
    }
}

/// A line of explanation under a control.
///
/// Added through `Label` with wrapping asked for explicitly: inside a modal
/// the surrounding layout justifies text, which spreads a wrapped sentence
/// across the full width and leaves holes in the middle of it.
fn hint(ui: &mut egui::Ui, text: &str) {
    ui.add(egui::Label::new(egui::RichText::new(text).small().color(theme::LEGEND)).wrap());
}

/// A squelch control that shows what it is deciding against.
///
/// A threshold with no meter beside it is a number to guess at: the operator
/// cannot tell whether 9 dB is one above the noise or ten below the station.
/// The bar is what the squelch is measuring right now, the marker is where it
/// opens, and dragging moves the marker.
fn squelch_meter(
    ui: &mut egui::Ui,
    lo: f32,
    hi: f32,
    measured: f32,
    threshold: &mut f32,
    open: bool,
) -> bool {
    let (rect, resp) =
        ui.allocate_exact_size(egui::vec2(120.0, 12.0), egui::Sense::click_and_drag());
    let at = |v: f32| {
        let t = ((v - lo) / (hi - lo)).clamp(0.0, 1.0);
        rect.left() + t * rect.width()
    };
    let p = ui.painter();
    p.rect_filled(rect, 2.0, theme::PANEL);
    let fill = egui::Rect::from_min_max(rect.min, egui::pos2(at(measured), rect.max.y));
    // Coloured by the decision rather than by the level, so a glance says
    // whether audio is getting through without reading the numbers.
    p.rect_filled(fill, 2.0, if open { theme::TRACE } else { theme::LEGEND });
    let x = at(*threshold);
    p.line_segment(
        [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
        egui::Stroke::new(1.5, theme::VALUE),
    );

    let mut changed = false;
    if let Some(pos) = resp.interact_pointer_pos() {
        if resp.dragged() || resp.clicked() {
            let t = ((pos.x - rect.left()) / rect.width()).clamp(0.0, 1.0);
            *threshold = lo + t * (hi - lo);
            changed = true;
        }
    }
    changed
}

/// A labelled settings row: legend on the left, control on the right, so the
/// modal reads as a column of settings rather than a wall of widgets.
fn row(ui: &mut egui::Ui, label: &str, add: impl FnOnce(&mut egui::Ui)) {
    ui.horizontal(|ui| {
        ui.add_sized([90.0, 18.0], egui::Label::new(legend(label)));
        add(ui);
    });
}

/// Resolution bandwidth, which is what the bin count actually buys you.
fn bin_hint(rate: f64, bins: usize) -> String {
    let hz = rate / bins as f64;
    if hz >= 1000.0 {
        format!("{:.1} kHz per bin", hz / 1e3)
    } else {
        format!("{hz:.0} Hz per bin")
    }
}

/// Settings affordance in a pane corner.
fn cog_rect(pane: &Rect) -> Rect {
    let s = 18.0;
    Rect::from_min_size(Pos2::new(pane.right() - s - 6.0, pane.top() + 6.0), Vec2::splat(s))
}

fn cog(p: &egui::Painter, r: &Rect, hot: bool) {
    let col = if hot { theme::READOUT } else { Color32::from_rgb(0x6A, 0x72, 0x7C) };
    let c = r.center();
    let rad = r.width() * 0.30;
    for i in 0..6 {
        let a = std::f32::consts::TAU * i as f32 / 6.0;
        let (s, co) = a.sin_cos();
        p.line_segment(
            [
                Pos2::new(c.x + co * rad * 0.95, c.y + s * rad * 0.95),
                Pos2::new(c.x + co * rad * 1.55, c.y + s * rad * 1.55),
            ],
            Stroke::new(1.6, col),
        );
    }
    p.circle_stroke(c, rad, Stroke::new(1.6, col));
}

fn fmt_hz(hz: f64) -> String {
    if hz.abs() >= 1e6 {
        format!("{:.4} MHz", hz / 1e6)
    } else if hz.abs() >= 1e3 {
        format!("{:.1} kHz", hz / 1e3)
    } else {
        format!("{hz:.0} Hz")
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _f: &mut eframe::Frame) {
        let _f = tracing::info_span!("frame").entered();
        {
            let _s = tracing::info_span!("drain").entered();
            self.drain();
        }
        self.screenshot(ui.ctx());
        self.soak_check(ui.ctx());
        {
            let _s = tracing::info_span!("head").entered();
            self.head(ui);
        }
        {
            let _s = tracing::info_span!("strip").entered();
            self.strip(ui);
        }
        {
            let _s = tracing::info_span!("log").entered();
            self.decode_log(ui);
        }
        {
            let _s = tracing::info_span!("scope").entered();
            CentralPanel::default()
                .frame(egui::Frame::NONE.fill(theme::CHASSIS))
                .show(ui, |ui| match self.view {
                    View::Spectrum => self.scope(ui),
                    View::Chain => self.chain(ui),
                    View::Flights => self.flights_view(ui),
                });
        }
        self.settings_modal(ui.ctx());
        self.restore_radio_settings();
        self.save_session();
    }

    fn on_exit(&mut self) {
        // The periodic save is debounced, so a change made in the last couple
        // of seconds before quitting is still only in memory.
        self.saved_at = None;
        self.save_session();
    }
}

impl App {
    /// Self-measured CPU, because a GUI process is awkward to sample from a
    /// shell and the number that matters is over a steady-state window.
    fn soak_check(&mut self, ctx: &egui::Context) {
        let Some(secs) = self.soak else { return };
        if self.shot_sent {
            return;
        }
        // Deliberately does not request repaints: the point is to measure how
        // often the app redraws on its own.
        let t0 = *self.shot_at.get_or_insert_with(std::time::Instant::now);
        let el = t0.elapsed().as_secs_f32();
        if el < secs {
            return;
        }
        let cpu = std::fs::read_to_string("/proc/self/stat")
            .ok()
            .and_then(|s| {
                // Fields are offset by the comm field, which can contain
                // spaces and parentheses, so start after the last ')'.
                let tail = &s[s.rfind(')')? + 1..];
                let f: Vec<&str> = tail.split_whitespace().collect();
                let u: f64 = f.get(11)?.parse().ok()?;
                let k: f64 = f.get(12)?.parse().ok()?;
                Some((u + k) / 100.0)
            })
            .unwrap_or(0.0);
        println!(
            "ran {el:.1}s, used {cpu:.2}s CPU = {:.0}% of one core",
            cpu / el as f64 * 100.0
        );
        crate::prof::report(std::time::Duration::from_secs_f32(el));
        self.shot_sent = true;
        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
    }

    fn screenshot(&mut self, ctx: &egui::Context) {
        let Some(path) = self.shot.clone() else { return };
        ctx.request_repaint();
        let t0 = *self.shot_at.get_or_insert_with(std::time::Instant::now);
        // Wait for the tuner to lock and the waterfall to fill; a screenshot
        // taken before that reviews an empty screen, not the design.
        if !self.shot_sent && t0.elapsed().as_secs_f32() > 6.0 {
            if self.channels.is_empty() {
                self.add_channel(95.8e6);
                self.add_channel(95.35e6);
            }
            self.shot_sent = true;
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(Default::default()));
        }
        let img = ctx.input(|i| {
            i.events.iter().find_map(|e| match e {
                egui::Event::Screenshot { image, .. } => Some(image.clone()),
                _ => None,
            })
        });
        if let Some(img) = img {
            let (w, h) = (img.width() as u32, img.height() as u32);
            let buf: Vec<u8> = img.pixels.iter().flat_map(|p| [p.r(), p.g(), p.b(), p.a()]).collect();
            if let Some(b) = image::RgbaImage::from_raw(w, h, buf) {
                let _ = b.save(&path);
                println!("wrote {path} ({w}x{h})");
            }
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }

    /// The readout and the controls that set it.
    fn head(&mut self, ui: &mut egui::Ui) {
        Panel::top("head")
            .frame(
                egui::Frame::NONE
                    .fill(theme::PANEL)
                    .inner_margin(egui::Margin::symmetric(14, 10)),
            )
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    let out = self.dial.show(ui, self.center, 34.0);
                    if out.changed {
                        self.retune(out.hz);
                    }

                    ui.add_space(16.0);
                    ui.vertical(|ui| {
                        ui.add_space(2.0);
                        ui.label(legend("band"));
                        ui.label(
                            value(bands::name_at(self.center))
                                .color(theme::TRACE)
                                .size(14.0),
                        );
                    });

                    ui.add_space(18.0);
                    self.divider(ui);
                    ui.add_space(18.0);

                    ui.vertical(|ui| {
                        ui.label(legend("radio"));
                        let cur = self
                            .device
                            .as_ref()
                            .map(|d| d.label.clone())
                            .unwrap_or_else(|| "none".into());
                        let mut pick = None;
                        let mut rescan = false;
                        egui::ComboBox::from_id_salt("device")
                            .selected_text(cur)
                            .width(190.0)
                            .show_ui(ui, |ui| {
                                for d in &self.devices {
                                    let on = self.device.as_ref() == Some(d);
                                    if ui.selectable_label(on, &d.label).clicked() {
                                        pick = Some(d.clone());
                                    }
                                }
                                ui.separator();
                                if ui.selectable_label(false, "Rescan").clicked() {
                                    rescan = true;
                                }
                            });
                        if rescan {
                            self.devices = crate::devices::list();
                            if self.device.is_none() {
                                self.device = self.devices.first().cloned();
                                let c = ui.ctx().clone();
                                self.connect(&c);
                            }
                        }
                        if let Some(d) = pick {
                            let c = ui.ctx().clone();
                            self.select_device(&c, d);
                        }

                        ui.add_space(4.0);
                        ui.horizontal(|ui| {
                            // Stopping releases the USB claim, which is the
                            // only way to hand the radio to another program
                            // without quitting.
                            let on = self.radio.is_some();
                            if ui
                                .add_enabled(!on, egui::Button::new("START"))
                                .clicked()
                            {
                                let c = ui.ctx().clone();
                                self.connect(&c);
                            }
                            if ui.add_enabled(on, egui::Button::new("STOP")).clicked() {
                                self.stop();
                            }
                            if ui.add_enabled(on, egui::Button::new("GAIN")).clicked() {
                                self.open = Some(Settings::Radio);
                            }
                        });
                    });

                    ui.add_space(18.0);
                    self.divider(ui);
                    ui.add_space(18.0);

                    ui.vertical(|ui| {
                        ui.label(legend("bandwidth"));
                        let cur = self
                            .spans
                            .iter()
                            .find(|s| (self.rate - s.effective()).abs() < 1.0)
                            .map(|s| s.label.clone())
                            .unwrap_or_else(|| "custom".into());
                        let mut pick = None;
                        egui::ComboBox::from_id_salt("span")
                            .selected_text(cur)
                            .width(96.0)
                            .show_ui(ui, |ui| {
                                for sp in &self.spans {
                                    let on = (self.rate - sp.effective()).abs() < 1.0
                                        && self.zoom == sp.zoom;
                                    let text = if sp.zoom > 1 {
                                        format!("{}  /{}", sp.label, sp.zoom)
                                    } else {
                                        sp.label.clone()
                                    };
                                    if ui.selectable_label(on, text).clicked() && !on {
                                        pick = Some(sp.clone());
                                    }
                                }
                            });
                        if let Some(sp) = pick {
                            // Rate first: the radio rebuilds everything on a
                            // rate change, and a zoom sent before it would be
                            // applied to a chain about to be replaced.
                            self.send(Cmd::Rate(Sps(sp.rate as u64)));
                            self.send(Cmd::Zoom(sp.zoom));
                            self.rate = sp.effective();
                            self.zoom = sp.zoom;
                            self.reset_waterfall();
                            self.retune_listener();
                        }
                    });

                    ui.add_space(18.0);
                    self.divider(ui);
                    ui.add_space(18.0);

                    ui.vertical(|ui| {
                        ui.label(legend("view"));
                        let mut v = self.view;
                        egui::ComboBox::from_id_salt("view")
                            .selected_text(v.label())
                            .width(120.0)
                            .show_ui(ui, |ui| {
                                for opt in [View::Spectrum, View::Chain, View::Flights] {
                                    ui.selectable_value(&mut v, opt, opt.label());
                                }
                            });
                        self.view = v;
                    });

                    ui.add_space(18.0);
                    self.divider(ui);
                    ui.add_space(18.0);

                    ui.vertical(|ui| {
                        ui.label(legend("decode"));
                        ui.horizontal(|ui| {
                            // Decoding the whole span at once is the expensive
                            // thing the app does, so it is a switch rather
                            // than something buried in a settings modal.
                            if ui.selectable_label(self.decode_on, "ALL").clicked() {
                                self.decode_on = !self.decode_on;
                                self.send(Cmd::Decode(self.decode_on));
                            }
                            if ui.selectable_label(self.log_open, "LOG").clicked() {
                                self.log_open = !self.log_open;
                            }
                        });
                    });

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        self.lamps(ui);
                    });
                });

            if let Some(e) = &self.err {
                    ui.add_space(4.0);
                    ui.label(
                        egui::RichText::new(e)
                            .color(theme::FAULT)
                            .font(FontId::proportional(12.0)),
                    );
                }
            });
    }

    fn settings_modal(&mut self, ctx: &egui::Context) {
        let Some(which) = self.open else { return };
        let title = match which {
            Settings::Spectrum => "Spectrum",
            Settings::Waterfall => "Waterfall",
            Settings::Radio => "Radio",
        };
        let r = egui::containers::Modal::new(egui::Id::new(title))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(if which == Settings::Radio { 420.0 } else { 320.0 });
                ui.label(legend(title));
                ui.add_space(10.0);
                match which {
                    Settings::Spectrum => self.spectrum_settings(ui),
                    Settings::Waterfall => self.waterfall_settings(ui),
                    Settings::Radio => self.radio_settings(ui),
                }
                ui.add_space(12.0);
                ui.separator();
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("CLOSE").clicked() {
                            self.open = None;
                        }
                    });
                });
            });
        if r.should_close() {
            self.open = None;
        }
    }

    /// Everything the radio itself can be set to.
    ///
    /// Separate from the spectrum and waterfall settings because it is a
    /// different kind of thing: those change what you see, these change what
    /// the receiver does, and getting them wrong costs sensitivity or
    /// intermodulation rather than a prettier display.
    fn radio_settings(&mut self, ui: &mut egui::Ui) {
        let Some(radio) = self.radio.as_ref() else {
            ui.label(legend("no radio running"));
            return;
        };
        let controls = radio.status.radio();
        if controls.stages.is_empty() && controls.toggles.is_empty() {
            ui.label(legend("this device has no adjustable stages"));
            return;
        }

        for (stage, mode) in &controls.stages {
            let auto = *mode == GainMode::Auto;
            let mut db = match mode {
                GainMode::Auto => *stage.range.start(),
                GainMode::Manual(v) => *v,
            };
            ui.horizontal(|ui| {
                ui.label(legend(&stage.label));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if stage.auto {
                        let mut on = auto;
                        if ui.checkbox(&mut on, "Auto").changed() {
                            self.send(Cmd::GainStage(
                                stage.name.clone(),
                                if on { GainMode::Auto } else { GainMode::Manual(db) },
                            ));
                        }
                    }
                    // Under AUTO the number is the hardware's business and
                    // showing a stale one invites the operator to believe it.
                    let text =
                        if auto { "auto".to_string() } else { format!("{db:.1} dB") };
                    ui.label(value(text).size(11.0));
                });
            });
            let lo = *stage.range.start();
            let hi = *stage.range.end();
            // Snapped as it is dragged, because the hardware does it anyway:
            // a slider that glides between values the tuner cannot reach shows
            // a number the receiver is not using.
            let slider = egui::Slider::new(&mut db, lo..=hi).show_value(false);
            if ui.add_enabled(!auto, slider).changed() {
                let want = stage.quantise(db);
                self.send(Cmd::GainStage(stage.name.clone(), GainMode::Manual(want)));
            }
            if !stage.values.is_empty() {
                hint(ui, &format!("{} steps, {lo:.0} to {hi:.0} dB", stage.values.len()));
            } else if stage.step > 0.0 {
                hint(ui, &format!("{:.0} dB steps, {lo:.0} to {hi:.0} dB", stage.step));
            }
            ui.add_space(10.0);
        }

        if !controls.toggles.is_empty() {
            ui.separator();
            ui.add_space(6.0);
            for t in &controls.toggles {
                let mut on = t.on;
                if ui.checkbox(&mut on, &t.label).changed() {
                    self.send(Cmd::Toggle(t.name.clone(), on));
                }
                hint(ui, &t.help);
                ui.add_space(8.0);
            }
        }

        ui.separator();
        ui.add_space(6.0);
        row(ui, "Correction", |ui| {
            let mut ppm = controls.ppm;
            if ui
                .add(egui::DragValue::new(&mut ppm).speed(0.5).range(-200.0..=200.0).suffix(" ppm"))
                .changed()
            {
                self.send(Cmd::Ppm(ppm));
            }
        });
        ui.label(
            egui::RichText::new(
                "The reference oscillator is a few tens of parts per million out on a cheap dongle, which is a kilohertz or two at 145 MHz and rather more higher up. Tune a known carrier and correct until it sits on its nominal frequency.",
            )
            .small()
            .color(theme::LEGEND),
        );
        ui.add_space(10.0);

        let mut dc = self.dc_block;
        if ui.checkbox(&mut dc, "Remove the DC spur").changed() {
            self.dc_block = dc;
            self.send(Cmd::DcBlock(dc));
        }
        ui.label(
            egui::RichText::new(
                "A direct conversion receiver leaks its own local oscillator into the middle of the span, where it looks exactly like a carrier on the frequency you are tuned to. This measures the offset and subtracts it.",
            )
            .small()
            .color(theme::LEGEND),
        );
    }

    fn spectrum_settings(&mut self, ui: &mut egui::Ui) {
        row(ui, "FFT bins", |ui| {
            let mut n = self.fft_size;
            egui::ComboBox::from_id_salt("fft")
                .selected_text(n.to_string())
                .width(120.0)
                .show_ui(ui, |ui| {
                    for v in FFTS {
                        ui.selectable_value(&mut n, v, v.to_string());
                    }
                });
            if n != self.fft_size {
                self.fft_size = n;
                self.send(Cmd::Fft(n));
                self.reset_waterfall();
            }
        });
        ui.label(
            egui::RichText::new(bin_hint(self.rate, self.fft_size))
                .small()
                .color(theme::LEGEND),
        );
        ui.add_space(8.0);

        row(ui, "Refresh", |ui| {
            let mut v = self.refresh;
            egui::ComboBox::from_id_salt("fps")
                .selected_text(format!("{} fps", v as i32))
                .width(120.0)
                .show_ui(ui, |ui| {
                    for (n, f) in REFRESH {
                        ui.selectable_value(&mut v, f, format!("{n} fps"));
                    }
                });
            if (v - self.refresh).abs() > 0.01 {
                self.refresh = v;
                self.send(Cmd::Refresh(v));
            }
        });
        ui.add_space(8.0);

        row(ui, "Averaging", |ui| {
            if ui
                .add(egui::Slider::new(&mut self.smoothing, 0.02..=1.0).show_value(false))
                .changed()
            {
                self.send(Cmd::Smoothing(self.smoothing));
            }
            ui.label(value(if self.smoothing > 0.95 {
                "off".to_string()
            } else {
                format!("{:.0}%", (1.0 - self.smoothing) * 100.0)
            }));
        });
        ui.add_space(8.0);

        row(ui, "Centre spur", |ui| {
            if ui.checkbox(&mut self.dc_block, "Remove").changed() {
                self.send(Cmd::DcBlock(self.dc_block));
            }
            ui.label(
                egui::RichText::new("LO leakage at the tuned frequency")
                    .color(theme::LEGEND)
                    .size(10.0),
            );
        });
        ui.add_space(8.0);
        self.scale_settings(ui);
    }

    fn waterfall_settings(&mut self, ui: &mut egui::Ui) {
        row(ui, "Scroll rate", |ui| {
            let mut v = self.rows_per_sec;
            egui::ComboBox::from_id_salt("rows")
                .selected_text(format!("{} rows/s", v as i32))
                .width(130.0)
                .show_ui(ui, |ui| {
                    for (n, f) in SPEEDS {
                        ui.selectable_value(&mut v, f, format!("{n} rows/s"));
                    }
                });
            self.rows_per_sec = v;
        });
        ui.add_space(8.0);

        row(ui, "History", |ui| {
            let mut n = self.wf_rows;
            egui::ComboBox::from_id_salt("hist")
                .selected_text(format!("{n} rows"))
                .width(130.0)
                .show_ui(ui, |ui| {
                    for v in [256usize, 512, 1024, 2048] {
                        ui.selectable_value(&mut n, v, format!("{v} rows"));
                    }
                });
            if n != self.wf_rows {
                self.wf_rows = n;
                self.wf.set_height(n);
            }
        });
        ui.label(
            egui::RichText::new(format!(
                "{:.0} s of history at {:.0} rows/s",
                self.wf.height() as f32 / self.rows_per_sec,
                self.rows_per_sec
            ))
            .small()
            .color(theme::LEGEND),
        );
        ui.add_space(8.0);

        row(ui, "Contrast", |ui| {
            ui.add(egui::Slider::new(&mut self.wf_top_offset, 0.0..=20.0).show_value(false));
            ui.label(value(format!("{:.0} dB", self.wf_top_offset)));
        });
        ui.label(
            egui::RichText::new("How far below the trace ceiling the hottest colour sits.")
                .small()
                .color(theme::LEGEND),
        );
        ui.add_space(8.0);
        self.scale_settings(ui);
    }

    fn scale_settings(&mut self, ui: &mut egui::Ui) {
        row(ui, "Scale", |ui| {
            ui.checkbox(&mut self.auto_scale, "Auto");
        });
        ui.add_enabled_ui(!self.auto_scale, |ui| {
            row(ui, "Floor", |ui| {
                ui.add(egui::Slider::new(&mut self.floor, -140.0..=0.0).suffix(" dB"));
            });
            row(ui, "Ceiling", |ui| {
                ui.add(egui::Slider::new(&mut self.ceil, -140.0..=20.0).suffix(" dB"));
            });
        });
    }

    fn divider(&self, ui: &mut egui::Ui) {
        let h = 40.0;
        let (rect, _) = ui.allocate_exact_size(Vec2::new(1.0, h), Sense::hover());
        ui.painter().line_segment(
            [
                Pos2::new(rect.center().x, rect.top()),
                Pos2::new(rect.center().x, rect.bottom()),
            ],
            Stroke::new(1.0, theme::ETCH),
        );
    }

    /// Status lamps. Dark is good; an unlit lamp means nothing is wrong.
    fn lamps(&self, ui: &mut egui::Ui) {
        let Some(r) = &self.radio else { return };
        use std::sync::atomic::Ordering;
        let dropped = r.status.dropped.load(Ordering::Relaxed);
        let running = r.status.running.load(Ordering::Relaxed);

        ui.vertical(|ui| {
            ui.add_space(4.0);
            lamp(ui, "drops", dropped > 0, theme::FAULT, &format!("{dropped}"));
            lamp(ui, "rx", running, theme::TRACE, if running { "on" } else { "off" });
        });
    }

    /// Gain and squelch, for the modes that have them.
    ///
    /// Worth a line of its own because on a weak signal these two are the
    /// difference between a band that is dead and a receiver that is muted,
    /// and without them both look and sound identical.
    fn channel_audio(ui: &mut egui::Ui, ch: &mut Channel, st: ChannelState) -> bool {
        let (gain_db, open, measured) = (st.agc_gain_db, st.squelch_open, st.squelch_db);
        let mut changed = false;
        ui.add_space(4.0);
        if ch.demod != Demod::Wfm {
            ui.horizontal(|ui| {
                ui.label(legend("agc"));
                if ui.selectable_label(ch.agc, if ch.agc { "ON" } else { "OFF" }).clicked() {
                    ch.agc = !ch.agc;
                    changed = true;
                }
                if ch.agc {
                    ui.label(value(format!("{gain_db:+.0} dB")).size(10.0));
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if !open {
                        ui.label(value("MUTED").size(10.0).color(theme::LEGEND));
                    }
                });
            });
        }
        if let Some(default) = ch.demod.default_squelch_db() {
            let (lo, hi, ratio) = ch.demod.squelch_range();
            let mut db = ch.squelch_db.unwrap_or(default);
            ui.horizontal(|ui| {
                ui.label(legend("sql"));
                if squelch_meter(ui, lo, hi, measured, &mut db, open) {
                    ch.squelch_db = Some(db);
                    changed = true;
                }
                // At the bottom of its range the squelch passes everything,
                // and saying so is more use than printing the number that
                // happens to be there.
                let text = if db <= lo + 0.5 {
                    "off".to_string()
                } else {
                    format!("{db:.0}{}", if ratio { "" } else { " dBFS" })
                };
                ui.label(value(text).size(10.0));
            });
            // The reading the threshold is being set against. Without it the
            // control is a number to guess at, and the right number differs
            // by mode and moves with the RF gain.
            ui.horizontal(|ui| {
                ui.add_space(28.0);
                hint(ui, &format!("now {measured:.0} dB"));
            });
        }
        changed
    }

    /// What the radio is hearing on the channel being listened to.
    ///
    /// This belongs inside the channel rather than beside the list: a station
    /// name is a property of one tuned frequency, and with several channels
    /// configured a panel-level readout gives no clue which one it describes.
    fn channel_rds(ui: &mut egui::Ui, st: &StationInfo, blend: f32) {
        if st.is_empty() && blend <= 0.01 {
            return;
        }
        ui.add_space(6.0);
        ui.separator();
        ui.horizontal(|ui| {
            ui.label(legend("rds"));
            if let Some(pi) = st.pi {
                ui.label(legend(&format!("PI {pi:04X}")));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // Stereo belongs here too: it is a property of this station,
                // and it fades with the blend because the audio does.
                let t = blend.clamp(0.0, 1.0);
                if t > 0.01 {
                    let c = theme::TRACE.gamma_multiply(0.35 + 0.65 * t);
                    ui.label(value(if t > 0.99 { "STEREO" } else { "BLEND" }).size(10.0).color(c));
                }
            });
        });
        if let Some(n) = &st.name {
            // Cyan, not amber: this is what the radio heard, not something the
            // operator set.
            ui.label(value(n).size(15.0).color(theme::TRACE));
        }
        if let Some(p) = st.pty {
            ui.label(legend(p));
        }
        if let Some(rt) = &st.radiotext {
            ui.add_space(2.0);
            // Radiotext is up to 64 characters and the strip is narrow, so let
            // it wrap rather than truncating a song title mid-word.
            ui.label(egui::RichText::new(rt).color(theme::LEGEND).size(11.0));
        }
    }

    fn strip(&mut self, ui: &mut egui::Ui) {
        Panel::right("channels")
            .default_size(285.0)
            .frame(
                egui::Frame::NONE
                    .fill(theme::PANEL)
                    .inner_margin(egui::Margin::symmetric(12, 10)),
            )
            .show(ui, |ui| {
                ui.label(legend("channels"));
                ui.add_space(6.0);

                // The master, which every channel's own level runs into.
                ui.horizontal(|ui| {
                    ui.label(legend("master"));
                    if ui
                        .add(egui::Slider::new(&mut self.volume, 0.0..=1.0).show_value(false))
                        .changed()
                    {
                        self.send(Cmd::Volume(self.volume));
                    }
                    let all_muted = !self.channels.is_empty()
                        && self.channels.iter().all(|c| c.muted || !c.on);
                    if ui.selectable_label(all_muted, "MUTE").clicked() {
                        for c in &mut self.channels {
                            c.muted = !all_muted;
                        }
                        self.send_channels();
                    }
                });

                ui.add_space(8.0);

                if self.channels.is_empty() {
                    ui.label(
                        egui::RichText::new("Click the spectrum to tune a channel.")
                            .color(theme::LEGEND)
                            .size(12.0),
                    );
                }

                let states: Vec<ChannelState> =
                    self.radio.as_ref().map(|r| r.status.channel_states()).unwrap_or_default();
                let mut remove = None;
                let mut tune = None;
                for (i, ch) in self.channels.iter_mut().enumerate() {
                    let active = self.listening == Some(i);
                    egui::Frame::NONE
                        .fill(if active { Color32::from_rgb(0x2A, 0x2E, 0x36) } else { theme::WELL })
                        .stroke(Stroke::new(1.0, if active { theme::READOUT } else { theme::ETCH }))
                        .corner_radius(2.0)
                        .inner_margin(egui::Margin::same(8))
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                // A lit bar marks the channel you are hearing.
                                let (r, _) = ui.allocate_exact_size(Vec2::new(3.0, 16.0), Sense::hover());
                                ui.painter().rect_filled(
                                    r,
                                    1.0,
                                    if active { theme::READOUT } else { theme::ETCH },
                                );
                                ui.add(
                                    egui::TextEdit::singleline(&mut ch.label)
                                        .desired_width(90.0)
                                        .frame(egui::Frame::NONE),
                                );
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        if ui.small_button("REMOVE").clicked() {
                                            remove = Some(i);
                                        }
                                    },
                                );
                            });
                            // Per-digit, like the main tuner: the wheel over a
                            // digit steps that decade, so tuning is repeatable
                            // rather than depending on pointer speed.
                            let d = self.chan_dial.compact(ui, ch.freq, 23.0);
                            if d.changed {
                                ch.freq = d.hz;
                                tune = Some(i);
                            }
                            ui.label(legend(bands::name_at(ch.freq)));
                            ui.add_space(4.0);
                            // Two rows: broadcast modes, then the ones an
                            // amateur band needs. Six across is narrower than
                            // the panel gets on a laptop.
                            ui.horizontal(|ui| {
                                for m in [Demod::Wfm, Demod::Nfm, Demod::Am] {
                                    if ui.selectable_label(ch.demod == m, m.label()).clicked() {
                                        ch.demod = m;
                                        tune = Some(i);
                                    }
                                }
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        // Every channel can be on at once and
                                        // they mix, so this is a per channel
                                        // switch rather than a choice of one.
                                        let text = if ch.on { "ON" } else { "OFF" };
                                        if ui.selectable_label(ch.on, text).clicked() {
                                            ch.on = !ch.on;
                                            tune = Some(i);
                                        }
                                    },
                                );
                            });
                            ui.horizontal(|ui| {
                                for m in [Demod::Usb, Demod::Lsb, Demod::Cw] {
                                    if ui.selectable_label(ch.demod == m, m.label()).clicked() {
                                        ch.demod = m;
                                        tune = Some(i);
                                    }
                                }
                            });
                            if ch.on {
                                // Its own level, which runs into the master.
                                ui.add_space(4.0);
                                ui.horizontal(|ui| {
                                    ui.label(legend("vol"));
                                    if ui
                                        .add(
                                            egui::Slider::new(&mut ch.volume, 0.0..=1.0)
                                                .show_value(false),
                                        )
                                        .changed()
                                    {
                                        tune = Some(i);
                                    }
                                    if ui.selectable_label(ch.muted, "M").clicked() {
                                        ch.muted = !ch.muted;
                                        tune = Some(i);
                                    }
                                });
                                let st = states.iter().find(|s| s.id == ch.id).copied();
                                if ch.demod == Demod::Wfm {
                                    // Each channel's own RDS, not the first
                                    // channel's: two WFM channels are usually
                                    // two different stations.
                                    let station = self
                                        .radio
                                        .as_ref()
                                        .and_then(|r| r.status.station_for(ch.id));
                                    if let Some(station) = station {
                                        let blend = st.map(|s| s.stereo_blend).unwrap_or(0.0);
                                        Self::channel_rds(ui, &station, blend);
                                    }
                                }
                                if let Some(st) = st {
                                    if Self::channel_audio(ui, ch, st) {
                                        tune = Some(i);
                                    }
                                }
                            }
                        });
                    ui.add_space(6.0);
                }

                if let Some(i) = remove {
                    self.channels.remove(i);
                    match self.listening {
                        Some(l) if l == i => self.listening = None,
                        Some(l) if l > i => self.listening = Some(l - 1),
                        _ => {}
                    }
                    self.retune_listener();
                }
                if tune.is_some() {
                    if let Some(i) = tune {
                        self.listening = Some(i);
                    }
                    self.send_channels();
                }

            });
    }

    /// Open on the chain view, for screenshots and for starting where the
    /// operator left off.
    /// Open the radio's own controls, for a screenshot or a quick check.
    pub fn show_radio_settings(&mut self) {
        self.open = Some(Settings::Radio);
    }

    pub fn show_chain(&mut self) {
        self.view = View::Chain;
    }

    pub fn show_flights(&mut self) {
        self.view = View::Flights;
    }

    /// Point the receiver at a frequency without opening a channel on it.
    ///
    /// Distinct from [`Self::tune_to`], which also starts demodulating: ADS-B
    /// and the band scanners want the dial moved and nothing listening, since
    /// there is no audio to be had at 1090 MHz.
    pub fn set_center(&mut self, mhz: f64) {
        self.center = mhz * 1e6;
        self.send(Cmd::Center(Hz(self.center as u64)));
        self.reset_waterfall();
    }

    /// The signal chain the listening channel is running.
    fn chain(&mut self, ui: &mut egui::Ui) {
        let Some(topo) = self.chain_topo.clone() else {
            ui.centered_and_justified(|ui| {
                ui.label(
                    egui::RichText::new("The radio is stopped, so no chain is running.")
                        .color(theme::LEGEND),
                );
            });
            return;
        };
        egui::ScrollArea::vertical().show(ui, |ui| {
            crate::chainview::draw(ui, &topo, self.chain_latency);
        });
    }

    fn scope(&mut self, ui: &mut egui::Ui) {
        let full = ui.available_rect_before_wrap();
        let ribbon_h = 16.0;
        let usable = (full.height() - ribbon_h - SPLIT_GRIP_H).max(1.0);
        let plot_h = usable * self.plot_frac.clamp(*PLOT_FRAC_RANGE.start(), *PLOT_FRAC_RANGE.end());
        let plot = Rect::from_min_max(full.min, Pos2::new(full.right(), full.top() + plot_h));
        let ribbon = Rect::from_min_max(
            Pos2::new(full.left(), plot.bottom()),
            Pos2::new(full.right(), plot.bottom() + ribbon_h),
        );
        let grip = Rect::from_min_max(
            Pos2::new(full.left(), ribbon.bottom()),
            Pos2::new(full.right(), ribbon.bottom() + SPLIT_GRIP_H),
        );
        let fall = Rect::from_min_max(Pos2::new(full.left(), grip.bottom()), full.max);

        let resp = ui.allocate_rect(full, Sense::click_and_drag());
        let p = ui.painter_at(full).to_owned();
        let plot_cog = cog_rect(&plot);
        let fall_cog = cog_rect(&fall);
        p.rect_filled(plot, 0.0, theme::WELL);

        self.grid(&p, &plot);
        {
            let _s = tracing::info_span!("trace").entered();
            self.trace(&p, &plot);
        }
        self.ribbon(&p, &ribbon);

        p.rect_filled(fall, 0.0, theme::CHASSIS);
        {
            let _wf = tracing::info_span!("wf_texture").entered();
            self.wf.draw(ui.ctx(), &p, fall);
        }

        self.markers(&p, &full);

        let hover = resp.hover_pos();
        let grip_hot = self.splitting || hover.is_some_and(|h| grip.contains(h));
        split_grip(&p, &grip, grip_hot);
        let plot_hot = hover.is_some_and(|h| plot_cog.contains(h));
        let fall_hot = hover.is_some_and(|h| fall_cog.contains(h));

        // Reaching for the cog is not reading the spectrum, so the crosshair
        // and its readout get out of the way rather than sitting under the
        // pointer while it is over a button.
        cog(&p, &plot_cog, plot_hot);
        cog(&p, &fall_cog, fall_hot);

        if !plot_hot && !fall_hot {
            let shift = ui.input(|i| i.modifiers.shift);
            self.cursor(&p, &full, &resp, shift);
        }

        if resp.clicked() && self.drag_ch.is_none() {
            if let Some(pos) = resp.interact_pointer_pos() {
                // Cogs sit inside the pane, so they get first refusal on a
                // click; otherwise opening settings would also drop a channel.
                if plot_cog.contains(pos) {
                    self.open = Some(Settings::Spectrum);
                } else if fall_cog.contains(pos) {
                    self.open = Some(Settings::Waterfall);
                } else if grip.contains(pos) {
                    // Dropping a channel on the divider is never what was
                    // meant; a double click there restores the default split.
                } else {
                    // Hit testing uses the true position; only the frequency a
                    // new channel lands on is snapped, so shift-clicking an
                    // existing channel still selects it.
                    match self.channel_at(&full, pos.x) {
                        Some(i) => self.listen(i),
                        None => self.add_channel(self.hz_at_snapped(&full, pos.x, ui)),
                    }
                }
            }
        }
        // Grabbing a channel marker moves that channel; grabbing anywhere else
        // pans the view. Deciding once when the drag starts means the gesture
        // cannot change meaning halfway through as the pointer moves off the
        // line it grabbed.
        if resp.drag_started() {
            // Test where the button went down, not where the pointer is now.
            // egui only reports a drag once the pointer has passed its drag
            // threshold, which is about the same distance as the grab
            // tolerance, so by this point the pointer has already left the
            // marker it grabbed and every drag looked like a pan.
            let origin = ui
                .input(|i| i.pointer.press_origin())
                .or_else(|| resp.interact_pointer_pos());
            self.splitting = origin.is_some_and(|pos| grip.contains(pos));
            self.drag_ch = origin.and_then(|pos| {
                if plot_cog.contains(pos) || fall_cog.contains(pos) || grip.contains(pos) {
                    return None;
                }
                self.channel_at(&full, pos.x)
            });
        }
        if resp.dragged() && self.splitting {
            if let Some(pos) = resp.interact_pointer_pos() {
                // Follow the pointer rather than accumulating deltas, so the
                // divider cannot drift away from the cursor over a long drag.
                let f = (pos.y - full.top() - SPLIT_GRIP_H / 2.0) / usable;
                self.plot_frac =
                    f.clamp(*PLOT_FRAC_RANGE.start(), *PLOT_FRAC_RANGE.end());
            }
        } else if resp.dragged() {
            match self.drag_ch {
                Some(i) if i < self.channels.len() => {
                    if let Some(pos) = resp.interact_pointer_pos() {
                        // Follow the pointer rather than accumulating deltas,
                        // so the marker cannot drift away from the cursor.
                        self.channels[i].freq = self.hz_at_snapped(&full, pos.x, ui);
                        if self.listening == Some(i) {
                            self.listen(i);
                        }
                    }
                }
                _ => {
                    let dx = resp.drag_delta().x as f64;
                    if dx.abs() > 0.0 {
                        self.retune(self.center - dx * self.rate / full.width() as f64);
                    }
                }
            }
        }
        if resp.drag_stopped() {
            self.drag_ch = None;
            self.splitting = false;
        }

        if resp.double_clicked() && hover.is_some_and(|h| grip.contains(h)) {
            self.plot_frac = DEFAULT_PLOT_FRAC;
        }

        // A marker under the pointer is draggable, so say so.
        if grip_hot {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeVertical);
        } else if self.drag_ch.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
        } else if let Some(h) = hover {
            if !plot_hot && !fall_hot && self.channel_at(&full, h.x).is_some() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
            }
        }

        // Wheel over the pane scrubs the centre frequency. A notch moves a
        // twentieth of the span, so the gesture means the same thing at every
        // zoom level.
        if resp.hovered() && !plot_hot && !fall_hot {
            let n = self.scrub.notches(ui);
            if n != 0 {
                self.retune(self.center - n as f64 * self.rate / 20.0);
            }
        }
    }

    fn grid(&self, p: &egui::Painter, plot: &Rect) {
        for i in 0..=10 {
            let x = plot.left() + plot.width() * i as f32 / 10.0;
            p.line_segment(
                [Pos2::new(x, plot.top()), Pos2::new(x, plot.bottom())],
                Stroke::new(1.0, Color32::from_rgb(0x24, 0x28, 0x2E)),
            );
        }
        // Amplitude graticule, labelled in dBFS so the numbers mean something.
        for i in 1..4 {
            let y = plot.top() + plot.height() * i as f32 / 4.0;
            p.line_segment(
                [Pos2::new(plot.left(), y), Pos2::new(plot.right(), y)],
                Stroke::new(1.0, Color32::from_rgb(0x22, 0x26, 0x2B)),
            );
            let db = self.ceil - (self.ceil - self.floor) * i as f32 / 4.0;
            p.text(
                Pos2::new(plot.right() - 4.0, y - 1.0),
                Align2::RIGHT_BOTTOM,
                format!("{db:.0}"),
                FontId::new(9.0, FontFamily::Name(theme::LEGEND_FONT.into())),
                Color32::from_rgb(0x4A, 0x51, 0x5A),
            );
        }
    }

    /// Which bins of the held spectrum belong under screen column `c`.
    ///
    /// `None` where the column falls outside the data, which happens while a
    /// retune is pending and the view has moved past what has been received.
    fn column_bins(
        &self,
        plot: &Rect,
        c: usize,
        _cols: usize,
        n: usize,
    ) -> Option<(usize, usize)> {
        let lo = self.db_center - self.rate / 2.0;
        let bin = |f: f64| ((f - lo) / self.rate * n as f64).floor();
        let a = bin(self.hz_at(plot, plot.left() + c as f32));
        let b = bin(self.hz_at(plot, plot.left() + c as f32 + 1.0));
        if a < 0.0 || a >= n as f64 {
            return None;
        }
        let a = a as usize;
        Some((a, (b.clamp(0.0, n as f64) as usize).max(a + 1).min(n)))
    }

    fn trace(&self, p: &egui::Painter, plot: &Rect) {
        if self.db.is_empty() {
            return;
        }
        let span = (self.ceil - self.floor).max(1.0);
        let n = self.db.len();
        let cols = plot.width().max(1.0) as usize;
        // Columns are placed by frequency rather than by bin index. While a
        // retune is pending the held spectrum belongs to a different centre,
        // and drawing it stretched across the pane would put every signal at
        // the wrong frequency. Positioning it by its own centre slides it under
        // the drag instead, which is where its data really is.
        let mut pts = Vec::with_capacity(cols);
        for c in 0..cols {
            let Some((a, b)) = self.column_bins(plot, c, cols, n) else { continue };
            // Max, not mean: averaging hides the narrow carriers that matter.
            let v = self.db[a..b].iter().copied().fold(f32::MIN, f32::max);
            let t = ((v - self.floor) / span).clamp(0.0, 1.0);
            pts.push(Pos2::new(plot.left() + c as f32, plot.bottom() - t * plot.height()));
        }
        if pts.len() < 2 {
            return;
        }
        // Fill under the trace so occupied spectrum reads as mass. Built as a
        // quad strip: a spectrum outline is wildly concave, and asking for a
        // convex polygon fill turns it into fan-shaped wedges.
        let mut mesh = egui::Mesh::default();
        let fill = Color32::from_rgba_unmultiplied(0x5C, 0xD0, 0xE8, 26);
        for w in pts.windows(2) {
            let i = mesh.vertices.len() as u32;
            for v in [
                w[0],
                w[1],
                Pos2::new(w[1].x, plot.bottom()),
                Pos2::new(w[0].x, plot.bottom()),
            ] {
                mesh.colored_vertex(v, fill);
            }
            mesh.add_triangle(i, i + 1, i + 2);
            mesh.add_triangle(i, i + 2, i + 3);
        }
        p.add(egui::Shape::mesh(mesh));
        p.add(egui::Shape::line(pts, Stroke::new(1.2, theme::TRACE)));
    }

    fn ribbon(&self, p: &egui::Painter, r: &Rect) {
        p.rect_filled(*r, 0.0, theme::CHASSIS);
        let (lo, hi) = (self.center - self.rate / 2.0, self.center + self.rate / 2.0);
        for b in bands::in_span(lo, hi) {
            let x0 = self.x_of(r, b.lo.max(lo)).max(r.left());
            let x1 = self.x_of(r, b.hi.min(hi)).min(r.right());
            if x1 - x0 < 1.0 {
                continue;
            }
            let cell = Rect::from_min_max(Pos2::new(x0, r.top() + 2.0), Pos2::new(x1, r.bottom() - 2.0));
            p.rect_filled(cell, 1.0, b.color);
            if x1 - x0 > 60.0 {
                p.text(
                    cell.center(),
                    Align2::CENTER_CENTER,
                    b.name,
                    FontId::new(9.0, FontFamily::Name(theme::LEGEND_FONT.into())),
                    Color32::from_rgb(0xE8, 0xEC, 0xF0),
                );
            }
        }
    }

    /// Aircraft, assembled from the ADS-B packets in the same stream the log
    /// shows. A view over the packet bus, not a second receiver.
    fn flights_view(&mut self, ui: &mut egui::Ui) {
        let now = std::time::Instant::now();
        let active = self.flights.active(now);
        // The pane runs to the window edge, and a table that starts there is
        // unreadable.
        let margin = egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 8));
        margin.show(ui, |ui| self.flights_table(ui, &active, now));
    }

    fn flights_table(
        &self,
        ui: &mut egui::Ui,
        active: &[&crate::flights::Aircraft],
        now: std::time::Instant,
    ) {
        ui.horizontal(|ui| {
            ui.label(legend("aircraft"));
            ui.label(value(active.len().to_string()).size(12.0));
            if active.is_empty() {
                ui.add_space(10.0);
                ui.label(legend("tune to 1090 MHz with a span of 2 MS/s or more"));
            }
        });
        ui.add_space(6.0);

        const COLS: [(&str, f32); 8] = [
            ("callsign", 90.0),
            ("icao", 70.0),
            ("altitude", 80.0),
            ("speed", 70.0),
            ("track", 60.0),
            ("climb", 80.0),
            ("position", 170.0),
            ("msgs", 56.0),
        ];
        // Wide enough that the last column is not clipped when the channel
        // strip is open, and scrolled sideways rather than squeezed when the
        // window is narrower than that.
        let width: f32 = COLS.iter().map(|(_, w)| w).sum::<f32>() + 60.0;
        egui::ScrollArea::horizontal().auto_shrink([false, false]).show(ui, |ui| {
        ui.set_min_width(width);
        let (rect, _) = ui.allocate_exact_size(Vec2::new(width, Self::ROW_H), Sense::hover());
        let p = ui.painter_at(rect);
        let mut x = rect.left();
        for (name, w) in COLS {
            Self::cell(&p, rect, x, w, name, theme::LEGEND);
            x += w;
        }
        Self::cell(&p, rect, x, rect.right() - x, "age", theme::LEGEND);
        p.line_segment(
            [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
            Stroke::new(1.0, theme::ETCH),
        );

        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            for (n, a) in active.iter().enumerate() {
                let (rect, _) =
                    ui.allocate_exact_size(Vec2::new(width, Self::ROW_H), Sense::hover());
                if !ui.is_rect_visible(rect) {
                    continue;
                }
                let p = ui.painter_at(rect);
                if n % 2 == 1 {
                    p.rect_filled(rect, 0.0, Color32::from_rgb(0x24, 0x27, 0x2D));
                }
                let dash = "-".to_string();
                let text = [
                    (a.callsign.clone().unwrap_or_else(|| dash.clone()), theme::TRACE),
                    (format!("{:06x}", a.icao), theme::VALUE),
                    (
                        a.altitude_ft.map(|v| format!("{v} ft")).unwrap_or_else(|| dash.clone()),
                        theme::VALUE,
                    ),
                    (
                        a.ground_speed_kt
                            .map(|v| format!("{v:.0} kt"))
                            .unwrap_or_else(|| dash.clone()),
                        theme::VALUE,
                    ),
                    (
                        a.track_deg.map(|v| format!("{v:.0}")).unwrap_or_else(|| dash.clone()),
                        theme::LEGEND,
                    ),
                    (
                        a.vertical_rate_fpm
                            .map(|v| format!("{v:+} fpm"))
                            .unwrap_or_else(|| dash.clone()),
                        // Climb and descent are worth telling apart at a
                        // glance; level flight is not worth colouring at all.
                        match a.vertical_rate_fpm {
                            Some(v) if v > 128 => CRC_OK,
                            Some(v) if v < -128 => theme::READOUT,
                            _ => theme::LEGEND,
                        },
                    ),
                    (
                        a.position
                            .map(|(lat, lon)| format!("{lat:.4}, {lon:.4}"))
                            .unwrap_or_else(|| dash.clone()),
                        theme::TRACE,
                    ),
                    (a.messages.to_string(), theme::LEGEND),
                ];
                let mut x = rect.left();
                for ((t, c), (_, w)) in text.iter().zip(COLS) {
                    Self::cell(&p, rect, x, w, t, *c);
                    x += w;
                }
                let age = a.age(now).as_secs();
                Self::cell(&p, rect, x, rect.right() - x, &format!("{age}s"), theme::LEGEND);
            }
        });
        });
    }

    /// The packet log: everything decoded anywhere in the span.
    fn decode_log(&mut self, ui: &mut egui::Ui) {
        if !self.log_open {
            return;
        }
        Panel::bottom("decodes")
            .default_size(230.0)
            // Drag the top edge to resize. A band that is busy wants a tall
            // list; one that is quiet wants the waterfall back.
            .resizable(true)
            .min_size(64.0)
            .max_size(720.0)
            .show_separator_line(true)
            .frame(
                egui::Frame::NONE
                    .fill(theme::PANEL)
                    .inner_margin(egui::Margin::symmetric(12, 8)),
            )
            .show(ui, |ui| {
                self.log_header(ui);
                ui.add_space(4.0);
                let selected = self
                    .selected
                    .and_then(|id| self.decodes.iter().find(|l| l.id == id))
                    .map(|l| l.rec.clone());
                let dump_h = if selected.is_some() { 116.0 } else { 0.0 };
                let list_h = (ui.available_height() - dump_h).max(24.0);
                // Two nested scroll areas so the headings stay above the rows
                // vertically but travel with them sideways, which is the only
                // arrangement where a narrow window can still reach the last
                // column and the headings never leave the top.
                // Both areas are given an explicit height. Without it the
                // content asks for as much room as it has rows, the panel
                // grows to match, and the headings are pushed off the top of
                // the window they are supposed to be pinned to.
                egui::ScrollArea::horizontal()
                    .auto_shrink([false, false])
                    .max_height(list_h)
                    .show(ui, |ui| {
                    let w = ui.available_width().max(Self::table_width());
                    ui.set_min_width(w);
                    if !self.decodes.is_empty() {
                        self.log_header_row(ui, w);
                    }
                    egui::ScrollArea::vertical()
                        .auto_shrink([false, false])
                        .max_height((list_h - Self::ROW_H).max(16.0))
                        .stick_to_bottom(true)
                        .id_salt("packet_rows")
                        .show(ui, |ui| self.log_rows(ui, w));
                });
                if let Some(rec) = selected {
                    ui.separator();
                    packet_detail(ui, &rec);
                }
            });
    }

    fn log_header(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(legend("packets"));
            ui.label(value(self.decodes.len().to_string()).size(12.0));
            if let Some(r) = &self.radio {
                use std::sync::atomic::Ordering;
                let narrow = r.status.scan_channels.load(Ordering::Relaxed);
                let wide = r.status.scan_channels_wide.load(Ordering::Relaxed);
                let total = r.status.decoded.load(Ordering::Relaxed);
                let aircraft = r.status.aircraft.load(Ordering::Relaxed);
                ui.add_space(10.0);
                ui.label(legend(&if r.status.modes_on.load(Ordering::Relaxed) {
                    format!("1090 MHz mode s, {aircraft} aircraft, {total} frames")
                } else if narrow > 0 {
                    format!("{narrow} ook + {wide} fsk channels, {total} seen")
                } else {
                    "decoding off".into()
                }));
            }
            let logged = self
                .radio
                .as_ref()
                .map(|r| r.status.logged.load(std::sync::atomic::Ordering::Relaxed))
                .unwrap_or(0);
            if logged > 0 {
                ui.add_space(10.0);
                ui.label(legend(&format!("{logged} saved")))
                    .on_hover_text(match &self.packet_log {
                        Some(d) => format!("appended to {}", d.display()),
                        None => "appended to the packet log".into(),
                    });
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("CLEAR").clicked() {
                    self.decodes.clear();
                    self.selected = None;
                }
                // Unknown bursts are the point of scanning a band, but on a
                // noisy one they crowd out the decodes, so they can be hidden
                // here without turning the reporting off upstream.
                if ui.selectable_label(self.show_unknown, "UNKNOWN").clicked() {
                    self.show_unknown = !self.show_unknown;
                }
            });
        });
    }

    /// Column headings and their widths in pixels. Fixed rather than sized to
    /// the content: a table whose columns resize as packets arrive is a table
    /// that moves under the pointer, and the last column absorbs the slack.
    const COLS: [(&'static str, f32); 8] = [
        ("no", 40.0),
        ("time", 64.0),
        ("frequency", 96.0),
        ("mod", 34.0),
        ("rssi", 48.0),
        ("snr", 44.0),
        ("protocol", 140.0),
        ("len", 38.0),
    ];
    const ROW_H: f32 = 16.0;

    /// Width the table needs before the info column starts being squeezed.
    fn table_width() -> f32 {
        Self::COLS.iter().map(|(_, w)| w).sum::<f32>() + 340.0
    }

    /// The heading strip, above the rows and outside their vertical scroll, so
    /// it cannot scroll away from what it labels.
    fn log_header_row(&self, ui: &mut egui::Ui, w: f32) {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(w, Self::ROW_H), Sense::hover());
        let p = ui.painter_at(rect);
        let mut x = rect.left();
        for (name, cw) in Self::COLS {
            Self::cell(&p, rect, x, cw, name, theme::LEGEND);
            x += cw;
        }
        Self::cell(&p, rect, x, rect.right() - x, "info", theme::LEGEND);
        p.line_segment(
            [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
            Stroke::new(1.0, theme::ETCH),
        );
    }

    /// One cell of text, clipped to its column so a long field cannot push the
    /// ones after it sideways.
    fn cell(p: &egui::Painter, row: Rect, x: f32, w: f32, text: &str, col: Color32) {
        let r = Rect::from_min_max(Pos2::new(x, row.top()), Pos2::new(x + w - 6.0, row.bottom()));
        p.with_clip_rect(r.intersect(p.clip_rect())).text(
            Pos2::new(r.left(), r.center().y),
            Align2::LEFT_CENTER,
            text,
            FontId::new(11.0, FontFamily::Name(theme::READOUT_FONT.into())),
            col,
        );
    }

    fn log_rows(&mut self, ui: &mut egui::Ui, width: f32) {
        if self.decodes.is_empty() {
            ui.label(legend(if self.decode_on {
                "listening to every channel in the span"
            } else {
                "decoding is off"
            }));
            return;
        }
        let t0 = self.decodes.first().map(|l| l.rec.at);
        let mut clicked = None;

        for (n, log) in self.decodes.iter().enumerate() {
            let rec = &log.rec;
            if !self.show_unknown && !rec.is_known() {
                continue;
            }
            // Every row is the same height and every column the same width, so
            // nothing reflows as packets arrive or the pointer moves over
            // them. The whole row is one hit target, painted rather than built
            // from widgets, which is also what keeps a five hundred row list
            // cheap to draw.
            let (rect, resp) =
                ui.allocate_exact_size(Vec2::new(width, Self::ROW_H), Sense::click());
            if !ui.is_rect_visible(rect) {
                continue;
            }
            let on = self.selected == Some(log.id);
            let p = ui.painter_at(rect);
            if on {
                p.rect_filled(rect, 0.0, theme::ETCH);
            } else if resp.hovered() {
                p.rect_filled(rect, 0.0, Color32::from_rgb(0x2A, 0x2E, 0x35));
            } else if n % 2 == 1 {
                p.rect_filled(rect, 0.0, Color32::from_rgb(0x24, 0x27, 0x2D));
            }
            if resp.clicked() {
                clicked = Some(log.id);
            }
            if resp.hovered() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
            }

            let col = row_color(rec);
            // Seconds since the first packet in the list, the way a capture is
            // timed rather than a wall clock, so two transmissions can be
            // compared without arithmetic.
            let secs = t0
                .map(|t0| rec.at.saturating_duration_since(t0).as_secs_f64())
                .unwrap_or(0.0);
            let text = [
                (format!("{:>4}", log.id), col),
                (format!("{secs:8.3}"), theme::LEGEND),
                (fmt_hz(rec.freq), theme::TRACE),
                (rec.modulation.to_string(), theme::LEGEND),
                (fmt_db(rec.rssi_dbfs), level_color(rec.rssi_dbfs)),
                (fmt_db(rec.snr_db), theme::LEGEND),
                (rec.model.clone(), col),
                (format!("{:>4}", rec.bytes.len()), theme::LEGEND),
            ];
            let mut x = rect.left();
            for ((t, c), (_, cw)) in text.iter().zip(Self::COLS) {
                Self::cell(&p, rect, x, cw, t, *c);
                x += cw;
            }
            Self::cell(&p, rect, x, rect.right() - x, &rec.detail, theme::VALUE);
        }

        if let Some(id) = clicked {
            // Clicking the selected packet again closes the dump.
            self.selected = (self.selected != Some(id)).then_some(id);
        }
    }

    fn markers(&self, p: &egui::Painter, full: &Rect) {
        let (lo, hi) = (self.center - self.rate / 2.0, self.center + self.rate / 2.0);
        for (i, ch) in self.channels.iter().enumerate() {
            if ch.freq < lo || ch.freq > hi {
                continue;
            }
            let x = self.x_of(full, ch.freq);
            let active = self.listening == Some(i);
            let col = if active { theme::READOUT } else { Color32::from_rgb(0x6E, 0x7A, 0x88) };

            // Show what the demodulator actually takes in, not just where it
            // is centred: an NFM channel and a WFM channel at the same spot
            // are wildly different slices of spectrum.
            let half = ch.demod.bandwidth() / 2.0;
            let (bx0, bx1) = (self.x_of(full, ch.freq - half), self.x_of(full, ch.freq + half));
            if bx1 - bx0 >= 1.0 {
                let band = Rect::from_min_max(
                    Pos2::new(bx0.max(full.left()), full.top()),
                    Pos2::new(bx1.min(full.right()), full.bottom()),
                );
                p.rect_filled(
                    band,
                    0.0,
                    Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), if active { 34 } else { 18 }),
                );
                for ex in [bx0, bx1] {
                    if full.x_range().contains(ex) {
                        p.line_segment(
                            [Pos2::new(ex, full.top()), Pos2::new(ex, full.bottom())],
                            Stroke::new(1.0, Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), 120)),
                        );
                    }
                }
            }
            p.line_segment(
                [Pos2::new(x, full.top()), Pos2::new(x, full.bottom())],
                Stroke::new(if active { 1.5 } else { 1.0 }, col),
            );
            // Flag the label off the line so it never sits on the trace.
            let t = p.layout_no_wrap(
                ch.label.clone(),
                FontId::new(10.0, FontFamily::Name(theme::LEGEND_FONT.into())),
                Color32::BLACK,
            );
            let flag = Rect::from_min_size(
                Pos2::new(x + 1.0, full.top() + 2.0),
                Vec2::new(t.size().x + 8.0, t.size().y + 4.0),
            );
            p.rect_filled(flag, 1.0, col);
            p.galley(Pos2::new(flag.left() + 4.0, flag.top() + 2.0), t, Color32::BLACK);
        }
    }

    fn cursor(&self, p: &egui::Painter, full: &Rect, resp: &egui::Response, shift: bool) {
        let Some(pos) = resp.hover_pos() else { return };
        let raw = self.hz_at(full, pos.x);
        let raster = bands::raster_at(raw);
        // With shift held the readout shows where a channel would actually
        // land, not where the pointer is, so the snap is visible before
        // committing to it.
        let hz = if shift { bands::snap(raw) } else { raw };
        p.line_segment(
            [Pos2::new(pos.x, full.top()), Pos2::new(pos.x, full.bottom())],
            Stroke::new(1.0, Color32::from_rgb(0x55, 0x5E, 0x69)),
        );
        if shift {
            // Mark the snapped frequency when it differs from the pointer.
            let sx = self.x_of(full, hz);
            if (sx - pos.x).abs() > 1.0 && full.x_range().contains(sx) {
                p.line_segment(
                    [Pos2::new(sx, full.top()), Pos2::new(sx, full.bottom())],
                    Stroke::new(1.0, theme::READOUT),
                );
            }
        }
        let text = match (shift, raster) {
            (true, Some(r)) => {
                format!("{} {} snap {}", fmt_hz(hz), bands::name_at(hz), fmt_hz(r.step))
            }
            (true, None) => format!("{} {} no channel plan", fmt_hz(hz), bands::name_at(hz)),
            _ => match raster {
                // Advertise the gesture only where it would do something.
                Some(_) => format!("{} {} shift to snap", fmt_hz(hz), bands::name_at(hz)),
                None => format!("{} {}", fmt_hz(hz), bands::name_at(hz)),
            },
        };
        let g = p.layout_no_wrap(
            text,
            FontId::new(11.0, FontFamily::Name(theme::READOUT_FONT.into())),
            theme::VALUE,
        );
        let left = (pos.x + 8.0).min(full.right() - g.size().x - 10.0);
        let box_r = Rect::from_min_size(
            Pos2::new(left - 5.0, full.top() + 5.0),
            g.size() + Vec2::new(10.0, 6.0),
        );
        p.rect(box_r, 2.0, theme::WELL, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);
        p.galley(Pos2::new(left, full.top() + 8.0), g, theme::VALUE);
    }
}

/// The handle between the spectrum and the waterfall.
///
/// Drawn as a short bar rather than a full-width line: a line reads as a
/// border, and a border is not something anyone tries to drag.
fn split_grip(p: &egui::Painter, r: &Rect, hot: bool) {
    p.rect_filled(*r, 0.0, theme::CHASSIS);
    let col = if hot { theme::READOUT } else { theme::ETCH };
    let w = 46.0;
    let y = r.center().y;
    let x0 = r.center().x - w / 2.0;
    for dy in [-2.0f32, 1.0] {
        p.line_segment(
            [Pos2::new(x0, y + dy), Pos2::new(x0 + w, y + dy)],
            Stroke::new(1.0, col),
        );
    }
}

/// A level in dB, or blank when the decoder did not measure one. Blank rather
/// than a zero: a missing measurement and a strong signal must not look alike.
fn fmt_db(v: f32) -> String {
    if v.is_finite() {
        format!("{v:6.1}")
    } else {
        " -".into()
    }
}

/// Amber when a packet is loud enough to be clipping the front end, which is
/// worth seeing: a decode can fail from too much gain as easily as too little.
fn level_color(rssi_dbfs: f32) -> Color32 {
    if rssi_dbfs > -3.0 {
        theme::FAULT
    } else if rssi_dbfs > -12.0 {
        theme::READOUT
    } else {
        theme::LEGEND
    }
}

/// Fixed-width text, so columns of numbers line up and a hex dump reads as one.
fn mono(text: &str, col: Color32) -> egui::RichText {
    egui::RichText::new(text)
        .font(FontId::new(11.0, FontFamily::Name(theme::READOUT_FONT.into())))
        .color(col)
}

/// Green for a verified packet, amber for one with no check to verify, red for
/// a failed one, grey for a burst nothing claimed. The same colours are used
/// on the waterfall.
fn row_color(rec: &DecodeRecord) -> Color32 {
    if !rec.is_known() {
        return theme::LEGEND;
    }
    match rec.crc {
        Some(true) => CRC_OK,
        Some(false) => theme::FAULT,
        None => theme::READOUT,
    }
}

/// What a selected packet holds: its fields, then its bytes.
///
/// The fields come first because they are the answer; the bytes are there for
/// when the answer is wrong, or when the protocol is unknown and the bytes are
/// all there is. Both are also what a view widget would consume: a map reads
/// the fields, an image pane reads the bytes and the media type.
fn packet_detail(ui: &mut egui::Ui, rec: &DecodeRecord) {
    if !rec.fields.is_empty() {
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 14.0;
            for (k, v) in &rec.fields {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 4.0;
                    ui.label(legend(k));
                    ui.label(mono(&v.to_string(), theme::VALUE));
                });
            }
        });
        ui.add_space(4.0);
    }
    hex_dump(ui, &rec.bytes);
}

/// Offset, hex, and printable ASCII, sixteen bytes to the line.
///
/// The bytes are what a protocol is worked out from, so they are shown as they
/// are rather than summarised. For an unknown burst these are the bits sliced
/// under a guessed coding, which is a guess about the framing and not about
/// the reception.
fn hex_dump(ui: &mut egui::Ui, bytes: &[u8]) {
    if bytes.is_empty() {
        ui.label(legend("no bits could be read from this burst"));
        return;
    }
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .id_salt("hex")
        .show(ui, |ui| {
            for (i, row) in bytes.chunks(16).enumerate() {
                let hex: String = row
                    .iter()
                    .enumerate()
                    .map(|(k, b)| if k == 7 { format!("{b:02x}  ") } else { format!("{b:02x} ") })
                    .collect();
                let ascii: String = row
                    .iter()
                    .map(|b| if b.is_ascii_graphic() || *b == b' ' { *b as char } else { '.' })
                    .collect();
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 8.0;
                    ui.label(mono(&format!("{:04x}", i * 16), theme::LEGEND));
                    ui.label(mono(&format!("{hex:<49}"), theme::VALUE));
                    ui.label(mono(&ascii, theme::TRACE));
                });
            }
        });
}

fn lamp(ui: &mut egui::Ui, label: &str, lit: bool, col: Color32, text: &str) {
    ui.horizontal(|ui| {
        let (r, _) = ui.allocate_exact_size(Vec2::new(7.0, 7.0), Sense::hover());
        let c = if lit { col } else { Color32::from_rgb(0x2C, 0x31, 0x38) };
        ui.painter().circle_filled(r.center(), 3.5, c);
        if lit {
            // A faint halo reads as a lit lamp rather than a painted dot.
            ui.painter()
                .circle_filled(r.center(), 6.0, Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), 30));
        }
        ui.label(legend(label));
        ui.label(value(text).size(11.0).color(if lit { col } else { theme::LEGEND }));
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn app() -> App {
        App {
            center: 100_000_000.0,
            rate: 2_000_000.0,
            // The waterfall holds history from where the radio actually is,
            // which after a settled tune is the same place.
            wf_center: 100_000_000.0,
            db_center: 100_000_000.0,
            ..Default::default()
        }
    }

    fn channel(app: &mut App, offset: f64, on: bool, volume: f32) {
        let freq = app.center + offset;
        let id = app.next_id as u64;
        app.next_id += 1;
        app.channels.push(Channel {
            id,
            freq,
            demod: Demod::Nfm,
            label: format!("CH{id}"),
            on,
            volume,
            muted: false,
            squelch_db: None,
            agc: true,
        });
    }

    #[test]
    fn every_channel_that_is_on_goes_to_the_mixer() {
        // The point of the mixer: several channels at once, each with its own
        // level, not one at a time.
        let mut a = app();
        channel(&mut a, 100_000.0, true, 0.8);
        channel(&mut a, -250_000.0, true, 0.3);
        channel(&mut a, 400_000.0, false, 1.0);

        let specs = a.channel_specs();
        assert_eq!(specs.len(), 2, "a channel that is off should not be demodulated");
        assert_eq!(specs[0].offset_hz, 100_000.0);
        assert_eq!(specs[1].offset_hz, -250_000.0);
        assert_eq!(specs[1].volume, 0.3, "each channel keeps its own level");
    }

    #[test]
    fn a_channel_keeps_its_identity_when_a_neighbour_is_removed() {
        // Chains are matched by id on the radio thread. If these were
        // positions, removing the first channel would silently hand its
        // running chain to the second.
        let mut a = app();
        channel(&mut a, 100_000.0, true, 1.0);
        channel(&mut a, 200_000.0, true, 1.0);
        let second = a.channel_specs()[1].id;
        a.channels.remove(0);
        assert_eq!(a.channel_specs()[0].id, second);
    }

    #[test]
    fn muting_everything_leaves_the_channels_running() {
        // Mute is a level, not a teardown: unmuting should not have to wait
        // for chains to be rebuilt and AGCs to settle again.
        let mut a = app();
        channel(&mut a, 100_000.0, true, 1.0);
        for c in &mut a.channels {
            c.muted = true;
        }
        let specs = a.channel_specs();
        assert_eq!(specs.len(), 1);
        assert!(specs[0].muted);
    }

    fn rect() -> Rect {
        Rect::from_min_size(Pos2::new(0.0, 0.0), Vec2::new(1000.0, 400.0))
    }


    fn record(freq: f64, crc: Option<bool>) -> DecodeRecord {
        DecodeRecord {
            at: std::time::Instant::now(),
            freq,
            model: "Fineoffset-WHx080".into(),
            channel_hz: 31_250.0,
            modulation: "OOK",
            detail: "temperature_c=16.2 humidity_pct=89".into(),
            fields: vec![
                ("temperature_c".into(), common::Value::Float(16.2)),
                ("humidity_pct".into(), common::Value::Int(89)),
            ],
            media_type: pipeline::event::media::BYTES,
            rssi_dbfs: -18.0,
            snr_db: 21.5,
            bytes: vec![0xab, 0xcd],
            crc,
        }
    }

    #[test]
    fn the_scope_split_is_adjustable_and_bounded() {
        let mut a = app();
        assert_eq!(a.plot_frac, DEFAULT_PLOT_FRAC);
        // Dragging past either end clamps rather than collapsing a pane: a
        // two pixel waterfall is not a smaller waterfall, it is a broken one.
        for want in [0.0f32, 1.0, 0.6] {
            a.plot_frac = want.clamp(*PLOT_FRAC_RANGE.start(), *PLOT_FRAC_RANGE.end());
            assert!(PLOT_FRAC_RANGE.contains(&a.plot_frac), "{want} left {}", a.plot_frac);
        }
        assert!(*PLOT_FRAC_RANGE.start() > 0.0 && *PLOT_FRAC_RANGE.end() < 1.0);
    }

    #[test]
    fn the_split_moves_the_boundary_the_way_the_pointer_went() {
        // The mapping the drag uses: pointer y within the pane becomes the
        // spectrum's share of it.
        let full = Rect::from_min_size(Pos2::new(0.0, 100.0), Vec2::new(1000.0, 800.0));
        let usable = full.height() - 16.0 - SPLIT_GRIP_H;
        let frac_at = |y: f32| ((y - full.top() - SPLIT_GRIP_H / 2.0) / usable)
            .clamp(*PLOT_FRAC_RANGE.start(), *PLOT_FRAC_RANGE.end());

        let up = frac_at(300.0);
        let down = frac_at(700.0);
        assert!(down > up, "dragging down must grow the spectrum");
        // A quarter of the way down the pane is about a quarter of the split.
        assert!((frac_at(full.top() + usable * 0.25) - 0.25).abs() < 0.02);
    }


    #[test]
    fn logged_packets_are_numbered_in_arrival_order() {
        let mut a = app();
        a.log_decodes(vec![record(a.center, None), record(a.center, None)]);
        assert_eq!(a.decodes[0].id, 1);
        assert_eq!(a.decodes[1].id, 2);
    }

    #[test]
    fn hiding_unknowns_does_not_discard_them() {
        // The filter is a view, not a policy: turning it back on must show the
        // bursts that arrived while it was off.
        let mut a = app();
        let mut unknown = record(a.center, None);
        unknown.model = "unknown".into();
        a.log_decodes(vec![unknown, record(a.center, Some(true))]);
        a.show_unknown = false;
        assert_eq!(a.decodes.len(), 2, "hiding must not drop anything");
        assert_eq!(a.decodes.iter().filter(|l| !l.rec.is_known()).count(), 1);
    }

    #[test]
    fn the_packet_log_is_bounded() {
        let mut a = app();
        for i in 0..(DECODE_LOG_MAX + 120) {
            a.log_decodes(vec![record(100_000_000.0 + i as f64, Some(true))]);
        }
        assert_eq!(a.decodes.len(), DECODE_LOG_MAX);
        // The oldest are the ones dropped, so the newest packet is still there.
        let newest = 100_000_000.0 + (DECODE_LOG_MAX + 119) as f64;
        assert_eq!(a.decodes.last().unwrap().rec.freq, newest);
        // Numbers keep counting past what the list holds, so a row keeps the
        // number it was given.
        assert_eq!(a.decodes.last().unwrap().id, (DECODE_LOG_MAX + 120) as u64);
    }

    #[test]
    fn frequency_mapping_round_trips() {
        let a = app();
        let r = rect();
        for hz in [99_000_000.0, 100_000_000.0, 100_750_000.0] {
            let back = a.hz_at(&r, a.x_of(&r, hz));
            assert!((back - hz).abs() < 1.0, "{hz} came back as {back}");
        }
    }

    fn with_channels(freqs: &[f64]) -> App {
        let mut a = app();
        a.center = 95_000_000.0;
        a.rate = 2_400_000.0;
        for f in freqs {
            a.channels.push(Channel {
                id: 1,
                freq: *f,
                demod: Demod::Wfm,
                label: "t".into(),
                on: true,
                volume: 0.8,
                muted: false,
                squelch_db: None,
                agc: true,
            });
        }
        a
    }

    #[test]
    fn a_marker_can_be_grabbed_from_further_than_the_drag_threshold() {
        // egui only reports a drag once the pointer has moved about 6 px, and
        // the hit test runs at that moment. A tolerance at or under the
        // threshold means the pointer has always already left the marker, and
        // every attempt to drag a channel pans the view instead.
        assert!(GRAB_PX > 6.0);
    }

    #[test]
    fn grabbing_is_a_fixed_distance_on_screen_at_any_span() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        for rate in [250_000.0, 2_400_000.0, 20_000_000.0] {
            let mut a = with_channels(&[95_000_000.0]);
            a.rate = rate;
            let x = a.x_of(&rect, 95_000_000.0);
            assert_eq!(a.channel_at(&rect, x), Some(0), "rate {rate}");
            // Just inside the grab distance, and just outside it.
            assert_eq!(a.channel_at(&rect, x + (GRAB_PX as f32) * 0.8), Some(0), "rate {rate}");
            assert_eq!(a.channel_at(&rect, x + (GRAB_PX as f32) * 1.5), None, "rate {rate}");
        }
    }

    #[test]
    fn the_nearest_marker_wins_when_two_are_close() {
        // Taking the first match would grab whichever was added earlier rather
        // than the one being pointed at.
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        let a = with_channels(&[95_000_000.0, 95_009_000.0]);
        let x = a.x_of(&rect, 95_009_000.0);
        assert_eq!(a.channel_at(&rect, x), Some(1));
        let x = a.x_of(&rect, 95_000_000.0);
        assert_eq!(a.channel_at(&rect, x), Some(0));
    }

    #[test]
    fn empty_space_grabs_nothing() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        let a = with_channels(&[95_000_000.0]);
        assert_eq!(a.channel_at(&rect, a.x_of(&rect, 94_500_000.0)), None);
    }

    #[test]
    fn the_trace_covers_the_whole_pane_when_nothing_is_pending() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        let mut a = app();
        a.center = 95_000_000.0;
        a.db_center = 95_000_000.0;
        a.rate = 2_400_000.0;
        assert_eq!(a.column_bins(&rect, 0, 1000, 2048).map(|x| x.0), Some(0));
        assert_eq!(a.column_bins(&rect, 999, 1000, 2048).map(|x| x.0), Some(2045));
    }

    #[test]
    fn a_pending_retune_slides_the_trace_instead_of_stretching_it() {
        // The held spectrum belongs to the old centre. Drawing it across the
        // whole pane would put every signal at the wrong frequency; it has to
        // move with the drag, because that is where its data is.
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        let mut a = app();
        a.rate = 2_400_000.0;
        a.db_center = 95_000_000.0;
        // View dragged a quarter span right, data not yet caught up.
        a.center = 95_600_000.0;
        // A quarter of a 2.4 MHz span is 512 bins of 2048, so the left of the
        // pane now shows what was a quarter of the way in.
        assert_eq!(a.column_bins(&rect, 0, 1000, 2048).map(|x| x.0), Some(512));
        // And the right quarter has no data at all yet.
        assert_eq!(a.column_bins(&rect, 900, 1000, 2048), None);
    }

    #[test]
    fn dragging_the_other_way_leaves_the_left_empty() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        let mut a = app();
        a.rate = 2_400_000.0;
        a.db_center = 95_000_000.0;
        a.center = 94_400_000.0;
        assert_eq!(a.column_bins(&rect, 0, 1000, 2048), None);
        assert_eq!(a.column_bins(&rect, 999, 1000, 2048).map(|x| x.0), Some(1533));
    }

    #[test]
    fn the_edges_are_the_ends_of_the_span() {
        let a = app();
        let r = rect();
        assert!((a.hz_at(&r, r.left()) - 99_000_000.0).abs() < 1.0);
        assert!((a.hz_at(&r, r.right()) - 101_000_000.0).abs() < 1.0);
    }

    #[test]
    fn clicks_outside_the_pane_clamp_to_the_span() {
        let a = app();
        let r = rect();
        assert!((a.hz_at(&r, -500.0) - 99_000_000.0).abs() < 1.0);
        assert!((a.hz_at(&r, 5000.0) - 101_000_000.0).abs() < 1.0);
    }

    #[test]
    fn new_channels_take_the_mode_of_their_band() {
        let mut a = app();
        a.add_channel(95.8e6);
        a.add_channel(124.0e6);
        assert_eq!(a.channels[0].demod, Demod::Wfm);
        assert_eq!(a.channels[1].demod, Demod::Am);
    }

    #[test]
    fn auto_scale_ignores_a_single_strong_carrier() {
        let mut a = app();
        a.floor = -90.0;
        a.ceil = -20.0;
        let mut db = vec![-95.0f32; 1024];
        db[500] = 0.0;
        for _ in 0..200 {
            a.rescale(&db);
        }
        assert!(a.floor < -95.0, "floor tracked the carrier: {}", a.floor);
        assert!(a.floor > -110.0, "floor ran away: {}", a.floor);
    }

    #[test]
    fn retuning_stays_inside_what_the_tuner_can_reach() {
        let mut a = app();
        a.retune(1.0);
        assert_eq!(a.center, 24e6);
        a.retune(9e9);
        assert_eq!(a.center, 1766e6);
    }

    #[test]
    fn fmt_hz_scales_units() {
        assert_eq!(fmt_hz(95_800_000.0), "95.8000 MHz");
        assert_eq!(fmt_hz(12_500.0), "12.5 kHz");
        assert_eq!(fmt_hz(400.0), "400 Hz");
    }
}
