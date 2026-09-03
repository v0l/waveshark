//! What each view remembers, kept apart from every other view.
//!
//! One struct per pane rather than one struct for the application. A pane can
//! then be handed exactly what it draws, which is what makes it a widget that
//! can be moved, tested or shown twice; and a field's owner is decided by
//! which pane it belongs to rather than by which file happened to add it.
//!
//! These are the parts of the interface that survive a frame. Anything a pane
//! works out again each frame stays a local.

use super::{Channel, Logged, MapView};
use crate::dial::Dial;
use crate::waterfall::Waterfall;
use crate::wheel::Wheel;
use std::time::Instant;

/// The spectrum and the waterfall: what is being drawn, and how.
pub(super) struct ScopeState {
    /// The latest spectrum, and the centre it was taken at, which lags the
    /// requested centre while a retune is pending.
    pub db: Vec<f32>,
    pub db_center: f64,
    pub wf: Waterfall,
    /// Centre the waterfall history corresponds to, so a retune can slide it
    /// instead of throwing it away.
    pub wf_center: f64,
    /// Frames held back until the next waterfall row is due, and where they
    /// were tuned, so a retune starts a fresh row rather than mixing two
    /// spans into one.
    pub wf_pending: Vec<f32>,
    pub wf_pending_center: f64,
    pub wf_last: Option<Instant>,
    pub wf_rows: usize,
    pub wf_top_offset: f32,
    pub rows_per_sec: f32,
    pub refresh: f32,
    pub smoothing: f32,
    pub floor: f32,
    pub ceil: f32,
    pub auto_scale: bool,
    /// Bins asked for, and bins the running spectrum actually has.
    pub fft: usize,
    pub fft_size: usize,
    /// Share of the pane given to the spectrum, the rest going to the
    /// waterfall. Dragged rather than fixed: which of the two matters depends
    /// entirely on what is being looked for.
    pub plot_frac: f32,
    pub splitting: bool,
    /// Channel whose marker is being dragged.
    pub drag_ch: Option<usize>,
    pub scrub: Wheel,
    /// Spectrum stages the operator added, from the last frame. Each covers
    /// whatever was wired into it rather than the span.
    pub extra: Vec<crate::radio::Spectrum>,
}

impl Default for ScopeState {
    fn default() -> Self {
        Self {
            db: Vec::new(),
            db_center: crate::session::DEFAULT_CENTER,
            wf: Waterfall::new(512),
            wf_center: crate::session::DEFAULT_CENTER,
            wf_pending: Vec::new(),
            wf_pending_center: 0.0,
            wf_last: None,
            wf_rows: 512,
            wf_top_offset: 5.0,
            rows_per_sec: 20.0,
            refresh: 30.0,
            smoothing: 0.35,
            floor: -90.0,
            ceil: -20.0,
            auto_scale: true,
            fft: 2048,
            fft_size: 2048,
            plot_frac: super::DEFAULT_PLOT_FRAC,
            splitting: false,
            drag_ch: None,
            scrub: Wheel::default(),
            extra: Vec::new(),
        }
    }
}

impl ScopeState {
    /// Take the view settings from a saved session.
    pub fn restore(&mut self, v: &crate::session::ViewPrefs, fft: usize) {
        self.rows_per_sec = v.rows_per_sec;
        self.wf_rows = v.wf_rows;
        self.wf = Waterfall::new(v.wf_rows);
        self.wf_top_offset = v.wf_top_offset;
        self.auto_scale = v.auto_scale;
        self.floor = v.floor;
        self.ceil = v.ceil;
        self.refresh = v.refresh;
        self.smoothing = v.smoothing;
        self.fft = fft;
        self.fft_size = fft;
    }

    /// The view settings, in the form they are stored in.
    pub fn prefs(&self) -> crate::session::ViewPrefs {
        crate::session::ViewPrefs {
            rows_per_sec: self.rows_per_sec,
            wf_rows: self.wf_rows,
            wf_top_offset: self.wf_top_offset,
            auto_scale: self.auto_scale,
            floor: self.floor,
            ceil: self.ceil,
            refresh: self.refresh,
            smoothing: self.smoothing,
        }
    }
}

/// The signal chain view: what the receiver is running, and what the operator
/// has drawn.
#[derive(Default)]
pub(super) struct ChainState {
    /// Shape of the running chain, republished by the radio thread whenever
    /// it rebuilds one. Cloned rather than shared so drawing never blocks the
    /// thread that has to keep draining USB.
    pub topo: Option<pipeline::graph::Topology>,
    pub latency: f64,
    /// The stage whose settings are showing, by node id.
    pub sel: Option<usize>,
    /// Manual mode and where the stages have been dragged to.
    pub edit: crate::chainview::Edit,
    /// The graph the operator has drawn, when manual mode is on.
    pub patch: crate::patch::Patch,
    /// Which revision the radio thread last published, so an edit it refused
    /// can be noticed and taken back.
    pub patch_rev: u64,
    /// The last patch handed to the radio thread. What comes back matches it
    /// when the edit built, and is the previous graph when it did not.
    pub patch_sent: Option<crate::patch::Patch>,
    /// The graph as last drawn by hand, which is not the one running: in
    /// automatic mode the receiver derives its own, and this is what taking
    /// it over goes back to.
    pub drawn: Option<crate::patch::Patch>,
    /// Where the stages were when the graph was last written out, so that
    /// dragging one is saved without writing the file on every frame.
    pub places: crate::patch::Places,
    pub saved_at: Option<Instant>,
    /// The operator's own stage that is selected, by patch id.
    pub pick: Option<u64>,
    /// The wire that is selected, named by the input it lands on.
    pub wire: Option<(u64, usize)>,
    /// Graphs as they were before each edit, and the ones undone since.
    /// Snapshots rather than a list of operations: a patch is small, and an
    /// operation log has to be kept correct against every future edit while a
    /// snapshot is right by construction.
    pub undo: Vec<crate::patch::Patch>,
    pub redo: Vec<crate::patch::Patch>,
}

/// The packet log and its inspector.
pub(super) struct LogState {
    /// Packets decoded anywhere in the span, oldest first.
    pub decodes: Vec<Logged>,
    /// Number given to the next packet.
    pub next_packet: u64,
    /// Packet whose bytes are shown in the dump.
    pub selected: Option<u64>,
    /// Height of the inspector inside the log window, dragged by its top
    /// edge. Held here rather than in a panel's memory so it is exactly this
    /// for every packet, whatever the packet holds.
    pub inspector_h: f32,
    /// Show bursts no protocol claimed.
    pub show_unknown: bool,
    pub open: bool,
    /// Print every packet to standard output as well as listing it, timed
    /// from when the window opened.
    pub print: bool,
    pub print_since: Instant,
    /// Where the log is being written, for the status line. The log itself
    /// lives in the graph, on the radio thread.
    pub path: Option<std::path::PathBuf>,
}

impl Default for LogState {
    fn default() -> Self {
        Self {
            decodes: Vec::new(),
            next_packet: 1,
            selected: None,
            inspector_h: 116.0 + super::BURST_VIEW_H + 24.0,
            show_unknown: true,
            open: true,
            print: false,
            print_since: Instant::now(),
            path: None,
        }
    }
}

/// The map: where it is looking, what is under it, and what is on it.
pub(super) struct MapState {
    pub view: MapView,
    pub tiles: crate::map::Tiles,
    /// Tracks, folded together from whatever on the bus reports a position:
    /// aircraft from ADS-B, vessels and marks from AIS.
    pub tracks: Vec<crate::tracks::Track>,
}

impl Default for MapState {
    fn default() -> Self {
        Self { view: MapView::default(), tiles: crate::map::Tiles::new(), tracks: Vec::new() }
    }
}

/// The call list and what it has subscribed to.
pub(super) struct CallsState {
    /// Who has been talking to whom, folded together from every decode that
    /// names a destination.
    pub list: crate::calls::Calls,
    /// What the call bus is subscribed to, as the interface holds it. The
    /// radio thread is sent the whole set whenever it changes.
    pub subs: Vec<crate::callbus::Subscription>,
    /// Groups switched off by hand, so one that was turned off does not
    /// subscribe itself again the next time somebody transmits on it.
    pub optout: Vec<crate::callbus::Rule>,
}

impl Default for CallsState {
    fn default() -> Self {
        Self { list: crate::calls::Calls::new(), subs: Vec::new(), optout: Vec::new() }
    }
}

/// The channel strip: every level that reaches the speaker.
pub(super) struct AudioState {
    pub channels: Vec<Channel>,
    /// The channel whose chain the signal chain view shows.
    pub listening: Option<usize>,
    pub volume: f32,
    pub next_id: u32,
    /// Shared per-digit readout for the strip. Only one channel can be under
    /// the pointer, so one is enough.
    pub dial: Dial,
    /// Level, mute and gain control for all call audio.
    pub call_volume: f32,
    pub call_muted: bool,
    pub call_agc: bool,
}

impl Default for AudioState {
    fn default() -> Self {
        Self {
            channels: Vec::new(),
            listening: None,
            volume: 0.5,
            next_id: 1,
            dial: Dial::new(),
            call_volume: 0.8,
            call_muted: false,
            call_agc: true,
        }
    }
}
