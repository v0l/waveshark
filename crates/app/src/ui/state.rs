//! What each view remembers, kept apart from every other view.
//!
//! One struct per pane rather than one struct for the application. A pane can
//! then be handed exactly what it draws, which is what makes it a widget that
//! can be moved, tested or shown twice; and a field's owner is decided by
//! which pane it belongs to rather than by which file happened to add it.
//!
//! These are the parts of the interface that survive a frame. Anything a pane
//! works out again each frame stays a local.

use crate::radio::{Cmd, DecodeRecord, Demod};
use crate::dial::Dial;
use crate::waterfall::Waterfall;
use crate::wheel::Wheel;
use std::time::Instant;

/// A decode, as shown in the packet log.
pub struct Logged {
    /// Position in the capture, counted from the first packet and never
    /// reused, so a row keeps its number as the list scrolls.
    pub(super) id: u64,
    pub(super) rec: DecodeRecord,
}

pub struct Channel {
    /// Stable for the life of the channel, so the radio thread can keep its
    /// chain when a different channel is removed.
    pub(super) id: u64,
    pub(super) freq: f64,
    pub(super) demod: Demod,
    pub(super) label: String,
    /// Whether this channel is being demodulated into the mix.
    pub(super) on: bool,
    /// Its own level in the mix, before the master volume.
    pub(super) volume: f32,
    pub(super) muted: bool,
    /// Where the squelch opens. None means the mode's own default, which is
    /// what an operator who has never touched the control should get.
    pub(super) squelch_db: Option<f32>,
    pub(super) agc: bool,
}

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
    /// The graph as it is running, with the operator's edits in it: what the
    /// view draws in manual mode, and what an edit is made to.
    pub patch: crate::patch::Patch,
    /// The graph as the receiver drew it before the edits. What `patch`
    /// differs from it by is what the operator changed, and that is what is
    /// sent, saved and put back on top of the next graph the receiver draws.
    pub base: crate::patch::Patch,
    /// Which revision the radio thread last published, so an edit it refused
    /// can be noticed and taken back.
    pub patch_rev: u64,
    /// The last patch handed to the radio thread. What comes back matches it
    /// when the edit built, and is the previous graph when it did not.
    pub patch_sent: Option<crate::patch::Patch>,
    /// What the operator has changed, as last sent to the receiver and as
    /// saved on disk. Applied whether or not manual mode is on: the mode is
    /// a lock on editing, not a different receiver.
    pub edits: crate::patch::Edits,
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

impl ChainState {
    /// Change the graph, keeping what it was so the change can be taken back.
    pub fn edit(&mut self, cmds: &mut Vec<Cmd>, f: impl FnOnce(&mut crate::patch::Patch)) {
        let before = self.patch.clone();
        f(&mut self.patch);
        if self.patch == before {
            return;
        }
        self.undo.push(before);
        // Undoing and then drawing something else abandons what was undone,
        // which is what makes redo mean anything: a branch nobody can reach
        // is a trap rather than a history.
        self.redo.clear();
        // A hundred edits is more than anybody backs out of in one sitting
        // and small enough to keep in hand: a patch is a few dozen stages.
        if self.undo.len() > 100 {
            self.undo.remove(0);
        }
        self.send_patch(cmds);
    }

    pub fn undo(&mut self, cmds: &mut Vec<Cmd>) {
        if let Some(was) = self.undo.pop() {
            self.redo.push(std::mem::replace(&mut self.patch, was));
            self.wire = None;
            self.send_patch(cmds);
        }
    }

    pub fn redo(&mut self, cmds: &mut Vec<Cmd>) {
        if let Some(next) = self.redo.pop() {
            self.undo.push(std::mem::replace(&mut self.patch, next));
            self.wire = None;
            self.send_patch(cmds);
        }
    }

    /// Hand the patch to the radio thread, remembering what was sent so that
    /// one handed back after a refusal can be told apart from an echo.
    pub fn send_patch(&mut self, cmds: &mut Vec<Cmd>) {
        self.edits = crate::patch::Edits::diff(&self.patch, &self.base);
        self.patch_sent = Some(self.patch.clone());
        cmds.push(Cmd::Edits(self.edits.clone()));
        self.save_patch();
    }

    /// Write the edits out, with where the stages were put.
    pub fn save_patch(&mut self) {
        self.places =
            self.edit.pos.iter().map(|(k, p)| (*k, (p.x, p.y))).collect();
        self.edits.save(&self.places);
        self.saved_at = Some(std::time::Instant::now());
    }

    /// Write it out again when a stage has been moved and the pointer has
    /// settled. Dragging changes a position on every frame, and a file
    /// written sixty times a second to record where a box ended up is a lot
    /// of writes for one arrangement.
    pub fn save_places(&mut self) {
        if !self.edit.manual {
            return;
        }
        let now: crate::patch::Places =
            self.edit.pos.iter().map(|(k, p)| (*k, (p.x, p.y))).collect();
        if now == self.places {
            return;
        }
        let due = self.saved_at.is_none_or(|t| t.elapsed().as_secs_f32() >= 2.0);
        if due {
            self.save_patch();
        }
    }

    /// Unlock the graph for editing, or lock it again.
    ///
    /// Nothing about what runs changes with it: the edits already made stay
    /// on the graph either way, and the graph keeps following the dial
    /// either way. Locking it puts the stages back where the automatic
    /// layout has them, since dragging them about was the point of
    /// unlocking.
    pub fn set_manual(&mut self, on: bool, cmds: &mut Vec<Cmd>) {
        self.edit.manual = on;
        if !on {
            self.edit.arrange();
            self.pick = None;
            self.wire = None;
        }
        cmds.push(Cmd::Manual(on));
    }
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
/// The call list and what it has subscribed to.
pub(super) struct CallsState {
    /// Who has been talking to whom, folded together from every decode that
    /// names a destination.
    pub list: crate::calls::Calls,
    /// What the call bus is subscribed to, as the interface holds it. The
    /// radio thread is sent the whole set whenever it changes.
    pub subs: Vec<crate::audiobus::Subscription>,
    /// Groups switched off by hand, so one that was turned off does not
    /// subscribe itself again the next time somebody transmits on it.
    pub optout: Vec<crate::audiobus::Rule>,
}

impl CallsState {
    /// Subscribe to any group not heard of before, unless it was switched off
    /// by hand.
    ///
    /// Every group is listened to until it is turned off. A scanner that
    /// hears nothing until it is configured is a scanner nobody hears
    /// anything on, and the box on the row is how it is turned off.
    pub fn subscribe_new(&mut self, calls: &[crate::calls::Call], cmds: &mut Vec<crate::radio::Cmd>) {
        let mut added = false;
        for c in calls {
            let rule = crate::audiobus::Rule::Group(c.to.clone());
            if self.optout.contains(&rule) || self.subs.iter().any(|s| s.rule == rule) {
                continue;
            }
            self.subs.push(crate::audiobus::Subscription::new(rule));
            added = true;
        }
        if added {
            cmds.push(crate::radio::Cmd::CallSubs(self.subs.clone()));
        }
    }

    /// Subscribe to a rule, or drop it if it is already there.
    pub fn toggle(&mut self, rule: crate::audiobus::Rule, cmds: &mut Vec<crate::radio::Cmd>) {
        match self.subs.iter().position(|s| s.rule == rule) {
            Some(i) => {
                self.subs.remove(i);
                self.optout.push(rule);
            }
            None => {
                self.optout.retain(|r| r != &rule);
                self.subs.push(crate::audiobus::Subscription::new(rule));
            }
        }
        cmds.push(crate::radio::Cmd::CallSubs(self.subs.clone()));
    }
}

impl Default for CallsState {
    fn default() -> Self {
        Self { list: crate::calls::Calls::new(), subs: Vec::new(), optout: Vec::new() }
    }
}

/// The message view: what was written over the air.
#[derive(Default)]
pub(super) struct MessagesState {
    /// Every decode that carried text, newest last.
    pub list: crate::messages::Messages,
    /// What the operator typed in the filter box. Kept here rather than in
    /// the pane so it survives a look at the spectrum and back.
    pub filter: String,
}

/// The key manager: the keys known, and what the operator is typing.
pub(super) struct KeysState {
    /// Keys stored on disk, loaded at startup and written when one changes.
    pub store: crate::keystore::KeyStore,
    /// The hex the operator is typing, per cell tag, before it is applied.
    pub typing: std::collections::HashMap<String, String>,
}

impl Default for KeysState {
    fn default() -> Self {
        Self { store: crate::keystore::KeyStore::load(), typing: std::collections::HashMap::new() }
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
    /// Which publication of the levels was last taken from the radio.
    pub levels_rev: u64,
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
            levels_rev: 0,
        }
    }
}
