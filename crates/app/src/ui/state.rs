//! What each view remembers, kept apart from every other view.
//!
//! One struct per pane rather than one struct for the application. A pane can
//! then be handed exactly what it draws, which is what makes it a widget that
//! can be moved, tested or shown twice; and a field's owner is decided by
//! which pane it belongs to rather than by which file happened to add it.
//!
//! These are the parts of the interface that survive a frame. Anything a pane
//! works out again each frame stays a local.

use crate::dial::Dial;
use crate::radio::{ChanMode, Cmd};
use crate::row::Reception;
use crate::waterfall::Waterfall;
use crate::wheel::Wheel;
use std::time::Instant;

/// A decode, as shown in the packet log.
pub struct Logged {
    /// Position in the capture, counted from the first packet and never
    /// reused, so a row keeps its number as the list scrolls.
    pub(super) id: u64,
    pub(super) rec: Reception,
}

pub struct Channel {
    /// Stable for the life of the channel, so the radio thread can keep its
    /// chain when a different channel is removed.
    pub(super) id: u64,
    pub(super) freq: f64,
    /// What it does with that frequency: play it, or decode it.
    pub(super) mode: ChanMode,
    /// The channel's width, or None for whatever the mode asks for.
    pub(super) bandwidth_hz: Option<f64>,
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
    /// Treat what is heard here as speech: calls on the bus, a row in the
    /// call list, and a transcript where a model is installed.
    pub(super) voice: bool,
    /// What this channel transmits when it is keyed, or `None` for a channel
    /// that only listens. Every channel starts that way.
    pub(super) tx: Option<crate::radio::TxSpec>,
    /// Whether a satellite pass is tuning this channel. Its dial belongs to
    /// the pass: a frequency typed in here would be overwritten within the
    /// second, which is a control that appears to do nothing.
    pub(super) doppler: bool,
}

impl Channel {
    /// The width this channel is really built at, which is what the marker on
    /// the spectrum has to be drawn from as well.
    pub(super) fn bandwidth(&self) -> f64 {
        self.bandwidth_hz.filter(|b| *b >= 100.0).unwrap_or_else(|| self.mode.bandwidth())
    }
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
    /// What the trace and the waterfall each take out of a frame.
    pub trace: dsp::spectrum::Detector,
    pub wf_detector: dsp::spectrum::Detector,
    pub floor: f32,
    pub ceil: f32,
    /// The window the waterfall's colours run between while the scale
    /// follows the signal, worked out from its own readings: its detector
    /// is not the trace's, and a peak waterfall coloured against an average
    /// trace's window comes out at the top of the ramp everywhere.
    pub wf_floor: f32,
    pub wf_ceil: f32,
    pub auto_scale: bool,
    /// The colours a row is drawn in, which an export is offered as its
    /// default: what is on screen is what somebody means by "this one".
    pub ramp: crate::heatmap::Ramp,
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
    /// How the converter was driven, from the last frame, and how many
    /// frames in a row it has been starved or clipping: the warning waits
    /// for a run so a single strong burst does not flash it.
    pub adc: nodes::AdcHealth,
    pub adc_bad_frames: u32,
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
            trace: dsp::spectrum::Detector::Average,
            wf_detector: dsp::spectrum::Detector::Peak,
            floor: -90.0,
            ceil: -20.0,
            wf_floor: -90.0,
            wf_ceil: -20.0,
            auto_scale: true,
            ramp: crate::heatmap::Ramp::Chassis,
            fft: 2048,
            fft_size: 2048,
            plot_frac: super::DEFAULT_PLOT_FRAC,
            splitting: false,
            drag_ch: None,
            scrub: Wheel::default(),
            extra: Vec::new(),
            adc: nodes::AdcHealth::default(),
            adc_bad_frames: 0,
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
        self.ramp = v.ramp;
        self.wf.set_ramp(v.ramp);
        self.floor = v.floor;
        self.ceil = v.ceil;
        self.refresh = v.refresh;
        self.smoothing = v.smoothing;
        self.trace = v.trace;
        self.wf_detector = v.wf_detector;
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
            ramp: self.ramp,
            floor: self.floor,
            ceil: self.ceil,
            refresh: self.refresh,
            smoothing: self.smoothing,
            trace: self.trace,
            wf_detector: self.wf_detector,
        }
    }
}

/// Which direction the chain view is drawing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) enum ChainSide {
    #[default]
    Rx,
    Tx,
}

impl ChainSide {
    pub fn label(self) -> &'static str {
        match self {
            Self::Rx => "RX",
            Self::Tx => "TX",
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
    /// What each scope stage is seeing, by node id, refreshed with the
    /// spectrum.
    pub scopes: Vec<(usize, nodes::ScopeFrame)>,
    /// The stage whose settings are showing, by node id.
    pub sel: Option<usize>,
    /// Which half of the receiver the pane is drawing.
    ///
    /// The two directions are separate chains that meet only at the radio, so
    /// the view switches between them rather than showing both at once: the
    /// transmit chain is four stages in a graph that on a wide span holds
    /// forty, and drawn together the one you are looking at is behind the
    /// banks.
    pub side: ChainSide,
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
    /// Whether the edits read off disk have been seen in a running graph.
    ///
    /// The receiver publishes its first graph before it has been handed the
    /// edits, and what is read off that is an empty set. Adopted, it replaced
    /// what had just been loaded and wrote an empty file over it: every
    /// setting changed by parameter, the transcriber's switch among them, was
    /// lost at the next start and had to be found again.
    pub edits_landed: bool,
    /// Edits changed and not yet written. A fader drag changes them on
    /// every frame, and a file written sixty times a second to record where
    /// a level ended up is a lot of writes for one level.
    pub edits_dirty: bool,
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
        self.edits =
            crate::patch::Edits::diff(&self.patch, &self.base, crate::chain::operator_owns);
        self.patch_sent = Some(self.patch.clone());
        cmds.push(Cmd::Edits(self.edits.clone()));
        self.save_patch();
    }

    /// Take what the running graph says the edits are.
    ///
    /// A setting changed by parameter, from the chain inspector or the
    /// transcript card, is an edit the receiver made on the interface's
    /// behalf: it comes back as the running graph and has to be written out
    /// like one drawn by hand, or the model picked is the model until the
    /// program is restarted.
    ///
    /// Not before what was read off disk has been seen running, though. The
    /// receiver publishes its first graph before it has been handed the
    /// edits, and what is read off that is an empty set: adopted, it wrote an
    /// empty file over the one just loaded, and the transcriber's switch had
    /// to be found again at every start.
    pub fn take_edits_from_the_running_graph(&mut self) {
        let edits = crate::patch::Edits::diff(&self.patch, &self.base, crate::chain::operator_owns);
        self.edits_landed |= !edits.is_empty() || self.edits.is_empty();
        if edits != self.edits && self.edits_landed {
            self.edits = edits;
            self.edits_dirty = true;
        }
        self.flush_edits(false);
    }

    /// Write the edits out once they have settled, or now.
    pub fn flush_edits(&mut self, now: bool) {
        if !self.edits_dirty {
            return;
        }
        let due = now || self.saved_at.is_none_or(|t| t.elapsed().as_secs_f32() >= 1.0);
        if due {
            self.edits_dirty = false;
            self.save_patch();
        }
    }

    /// Write the edits out, with where the stages were put.
    pub fn save_patch(&mut self) {
        self.places = self.edit.pos.iter().map(|(k, p)| (*k, (p.x, p.y))).collect();
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
    /// What the time column counts from: when the receiver started. Fixed
    /// rather than read off the head of the list, which is bounded and drops
    /// its oldest rows, moving every row's time whenever it did.
    pub origin: Instant,
    /// Packet whose bytes are shown in the dump.
    pub selected: Option<u64>,
    /// Height of the inspector inside the log window, dragged by its top
    /// edge. Held here rather than in a panel's memory so it is exactly this
    /// for every packet, whatever the packet holds.
    pub inspector_h: f32,
    pub open: bool,
    /// Print every packet to standard output as well as listing it, timed
    /// from when the window opened.
    pub print: bool,
    pub print_since: Instant,
    /// The packet the signal identification modal is open on, if it is.
    pub sigid: Option<Reception>,
    /// The `.sub` file being written out of a packet, if one is.
    pub sub_save: SubSave,
}

/// Writing one packet out as a Flipper `.sub` file.
///
/// The text is built before the dialog opens, so what is saved is the packet
/// the operator was looking at rather than whichever row the list has
/// scrolled to by the time they choose a name.
#[derive(Default)]
pub(super) struct SubSave {
    going: Option<(String, poll_promise::Promise<Option<std::path::PathBuf>>)>,
    /// The last file written or the reason none was, for the line under the
    /// button. Kept until the next save, so a path stays readable.
    pub said: Option<String>,
}

impl SubSave {
    /// Ask where to write `text`, offering `stem` as the name.
    pub fn ask(&mut self, ctx: &egui::Context, text: String, stem: String) {
        if self.going.is_some() {
            return;
        }
        let start = crate::chain::default_sub_dir();
        let _ = std::fs::create_dir_all(&start);
        let ctx = ctx.clone();
        self.going = Some((
            text,
            poll_promise::Promise::spawn_thread("save sub", move || {
                let picked = rfd::FileDialog::new()
                    .set_title("Write this burst as a Flipper .sub file")
                    .set_directory(&start)
                    .set_file_name(format!("{stem}.sub"))
                    .add_filter("Flipper SubGhz", &["sub"])
                    .save_file();
                // Nothing is drawing while the dialog is up, so the frame
                // that reads this has to be asked for.
                ctx.request_repaint();
                picked
            }),
        ));
    }

    /// Write the file, once the dialog has said where.
    pub fn poll(&mut self) {
        if self.going.as_ref().is_none_or(|(_, p)| p.ready().is_none()) {
            return;
        }
        let Some((text, promise)) = self.going.take() else {
            return;
        };
        let Some(path) = promise.block_and_take() else {
            return;
        };
        // The dialog's own name is taken as given except for the suffix: a
        // file without it is one the Flipper will not list.
        let path = match path.extension() {
            Some(e) if e.eq_ignore_ascii_case("sub") => path,
            _ => path.with_extension("sub"),
        };
        self.said = Some(match std::fs::write(&path, text) {
            Ok(()) => format!("wrote {}", path.display()),
            Err(e) => format!("could not write {}: {e}", path.display()),
        });
    }
}

/// What a search for a modem on the serial ports found, or is still looking
/// for.
///
/// The results are offered and never applied: a probe opens a port, and
/// deciding which GPS the station reads is the operator's, so this holds
/// candidates until one is picked.
pub struct ModemScan {
    pub done: std::sync::mpsc::Receiver<Vec<gps::Found>>,
    pub found: Option<Vec<gps::Found>>,
}

/// The device database, as the interface holds it.
///
/// The survey itself is a node on the radio thread; what lives here is where
/// it writes, where the position comes from, and what the pane is showing.
pub struct SurveyState {
    /// The row the pane is expanded on, which is the device whose sightings
    /// are drawn on the map.
    pub selected: Option<i64>,
    /// Rows as the radio thread last published them.
    pub rows: Vec<survey::Device>,
    /// Sightings of the selected device, fetched when the selection changes
    /// rather than every frame: a device heard all afternoon has thousands.
    pub trail: Vec<survey::Sighting>,
    /// Where those sightings put the device, when they can say: refitted
    /// whenever the trail is.
    pub estimate: Option<survey::Estimate>,
    /// Free text the list is filtered by: an address, a name, a protocol.
    pub filter: String,
    /// A read-only handle on the same file the radio thread is writing, which
    /// is how the pane draws a survey without the rows travelling through the
    /// status block every frame.
    pub db: Option<survey::Db>,
    /// When the rows were last read, so a pane open on a busy band is not a
    /// query per frame.
    pub refreshed: Option<Instant>,
    /// The GPS source being typed in settings, while it is being typed. Kept
    /// apart from the live one so a half-written port does not restart the
    /// reader on every keystroke.
    pub gps_edit: Option<String>,
    /// A search of the serial ports for a sub-ghz-modem, which takes a couple
    /// of seconds per port and so runs off the frame.
    pub gps_scan: Option<ModemScan>,
    /// The feed to wigle.net: who it uploads as, and what it has sent.
    pub wigle: WigleState,
    /// The feed to beacondb.net: whether its dialog is up, and what it has
    /// sent.
    pub beacondb: BeaconDbState,
    /// The feed into Home Assistant: the broker, and what it has published.
    pub homeassistant: HomeAssistantState,
}

/// The beaconDB feed, as the interface holds it.
///
/// No account, so there is nothing to type: what lives here is the switch
/// and what the radio thread last reported.
#[derive(Default)]
pub struct BeaconDbState {
    pub open: bool,
    pub status: Option<nodes::BeaconDbStatus>,
}

/// The feed into the house, as the interface holds it.
///
/// The broker as it is being typed, kept apart from the live one so a
/// half-written hostname does not reconnect on every keystroke, and what the
/// node last said it was doing.
pub struct HomeAssistantState {
    pub open: bool,
    pub on: bool,
    pub host: String,
    pub port: String,
    pub username: String,
    pub password: String,
    pub prefix: String,
    pub topic: String,
    /// Identity spaces worth publishing, as typed: `ism,wmbus` is a house's
    /// own sensors without the street's handsets.
    pub spaces: String,
    /// Whether what people say and write goes with the readings.
    pub buses: bool,
    pub status: Option<nodes::HomeAssistantStatus>,
}

impl Default for HomeAssistantState {
    /// Calls and messages on: a receiver pointed at a house is pointed at it
    /// for what it hears, and the sensors are the part somebody filters.
    fn default() -> Self {
        Self {
            open: false,
            on: false,
            host: String::new(),
            port: String::new(),
            username: String::new(),
            password: String::new(),
            prefix: String::new(),
            topic: String::new(),
            spaces: String::new(),
            buses: true,
            status: None,
        }
    }
}

/// The pass table, as the interface holds it. Everything in it is derived
/// from the elements and the station, so nothing here is worth saving except
/// what the operator chose.
pub struct SatsState {
    pub group: &'static datasets::tle::Group,
    /// The least a pass has to reach to be listed.
    pub min_el_deg: f64,
    /// The catalogue number the table has selected, which is also what the
    /// map draws a path for.
    pub selected: Option<u64>,
    /// Which transmitter of a satellite the operator picked, by SatNOGS's
    /// identifier for it. A satellite has many: the ISS has fifty rows and
    /// forty-one of them are live, so quoting one and offering no way to
    /// change it hides most of what is up there. Empty means whatever
    /// `Transmitters::best` chooses.
    pub downlink: std::collections::HashMap<u64, String>,
    /// The channel that is following a satellite down, if one is.
    pub tracking: Option<Tracking>,
}

/// A listening channel tied to a satellite's downlink.
///
/// The correction is not a setting that can be applied once: a low pass
/// moves several kilohertz in a minute, so a channel that is tuned when the
/// operator clicks is off the transmission before the satellite is overhead.
/// What is held is what it takes to keep tuning it: which satellite, what it
/// transmits on, and which channel to move.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tracking {
    pub norad: u64,
    /// As transmitted, before any shift.
    pub downlink_hz: f64,
    /// The channel being moved, by its own id rather than by position: the
    /// strip is reordered when channels are added and removed.
    pub channel: u64,
    /// Where it was last put, so a shift smaller than the receiver can act
    /// on does not become a command every frame.
    pub tuned_hz: f64,
}

impl Default for SatsState {
    fn default() -> Self {
        // The amateur group is the one this receiver can actually hear.
        Self {
            group: &datasets::tle::AMATEUR,
            min_el_deg: 10.0,
            selected: None,
            downlink: std::collections::HashMap::new(),
            tracking: None,
        }
    }
}

/// The wardriving feed, as the interface holds it.
///
/// The account is the operator's and the uploading is the node's; what lives
/// here is what has been typed and what the radio thread last reported.
#[derive(Default)]
pub struct WigleState {
    pub open: bool,
    /// The API name from wigle.net/account, which is not the login name.
    pub name: String,
    pub token: String,
    /// Whether wigle.net may licence the rows on commercially. Off unless the
    /// operator says otherwise: they are the ones giving the data away.
    pub donate: bool,
    /// Whether the feed is running, as the interface asked for it.
    pub on: bool,
    /// What the node last said it was doing.
    pub status: Option<nodes::WigleStatus>,
}

impl Default for SurveyState {
    fn default() -> Self {
        Self {
            selected: None,
            rows: Vec::new(),
            trail: Vec::new(),
            estimate: None,
            filter: String::new(),
            db: None,
            refreshed: None,
            gps_edit: None,
            gps_scan: None,
            wigle: WigleState::default(),
            beacondb: BeaconDbState::default(),
            homeassistant: HomeAssistantState::default(),
        }
    }
}

impl Default for LogState {
    fn default() -> Self {
        Self {
            decodes: Vec::new(),
            next_packet: 1,
            origin: Instant::now(),
            selected: None,
            sigid: None,
            sub_save: SubSave::default(),
            inspector_h: 116.0 + super::BURST_VIEW_H + 24.0,
            open: true,
            print: false,
            print_since: Instant::now(),
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
    pub subs: Vec<crate::mix::calls::Subscription>,
    /// Groups switched off by hand, so one that was turned off does not
    /// subscribe itself again the next time somebody transmits on it.
    pub optout: Vec<crate::mix::calls::Rule>,
    /// The recordings table under the live list: what the folder holds, and
    /// whether it is open at all. Headers only, so a week of a busy
    /// talkgroup lists in a few milliseconds; the audio is read when a row
    /// is played.
    pub recordings: Vec<crate::calllog::Entry>,
    /// When the folder was last walked, so the table refreshes itself
    /// without doing so on every frame.
    pub read_at: Option<Instant>,
    /// What the recordings table is narrowed to: every word of it has to
    /// appear somewhere in a row for that row to be listed.
    pub filter: String,
    /// What the last export or timeline did, shown under the filter until
    /// something else is asked for.
    pub log_note: String,
    /// The conversation drawn against the clock, over whatever the filter
    /// leaves.
    pub timeline: super::timeline::TimelineState,
    /// A save dialog in flight, and what it will have written when it
    /// answers. Off the painting thread: a dialog that blocks the frame is a
    /// window the compositor calls unresponsive.
    pub saving: Option<poll_promise::Promise<String>>,
    pub log_open: bool,
    /// Where the divider between the live list and the recordings sits.
    pub log_frac: f32,
    pub log_splitting: bool,
}

/// The most the recordings table holds. Beyond a few thousand rows nobody is
/// reading a list, and the folder is minutes of walking rather than
/// milliseconds.
pub(super) const RECORDINGS_MAX: usize = 2_000;

/// How often the folder is walked while the table is open. Long enough that
/// a receiver recording every over is not reading its own folder back
/// continuously, short enough that an over appears while somebody is still
/// looking for it.
pub(super) const RECORDINGS_EVERY: std::time::Duration = std::time::Duration::from_secs(3);

impl CallsState {
    /// Walk the folder again if it is time to, or if something asked.
    /// The recordings the filter leaves, newest first.
    ///
    /// Every word of the filter has to appear somewhere in the row, so
    /// "pmr5 101" is that caller on that talkgroup and the order of the words
    /// does not matter. A row is matched against what it shows: the time, the
    /// system, the frequency, the group and the caller.
    pub fn filtered(&self) -> Vec<crate::calllog::Entry> {
        let want = self.filter.to_lowercase();
        let terms: Vec<&str> = want.split_whitespace().collect();
        if terms.is_empty() {
            return self.recordings.clone();
        }
        self.recordings
            .iter()
            .filter(|e| {
                let c = &e.call;
                let row = format!(
                    "{} {} {:.4} {} {}",
                    crate::segments::when(c.at_us).format("%Y-%m-%d %H:%M:%S"),
                    c.system,
                    c.channel_hz as f64 / 1e6,
                    c.to.as_deref().unwrap_or(""),
                    c.from.as_deref().unwrap_or(""),
                )
                .to_lowercase();
                terms.iter().all(|t| row.contains(t))
            })
            .cloned()
            .collect()
    }

    pub fn read_recordings(&mut self, dir: &std::path::Path, force: bool) {
        let due = self.read_at.is_none_or(|at| at.elapsed() >= RECORDINGS_EVERY);
        if !force && !due {
            return;
        }
        self.recordings = crate::calllog::browse(dir, RECORDINGS_MAX);
        self.read_at = Some(Instant::now());
    }
}

impl CallsState {
    /// Subscribe to any group not heard of before, unless it was switched off
    /// by hand.
    ///
    /// Every group is listened to until it is turned off. A scanner that
    /// hears nothing until it is configured is a scanner nobody hears
    /// anything on, and the box on the row is how it is turned off.
    pub fn subscribe_new(
        &mut self,
        calls: &[crate::calls::Call],
        cmds: &mut Vec<crate::radio::Cmd>,
    ) {
        let mut added = false;
        for c in calls {
            let rule = crate::mix::calls::Rule::Group(c.to.clone());
            if self.optout.contains(&rule) || self.subs.iter().any(|s| s.rule == rule) {
                continue;
            }
            self.subs.push(crate::mix::calls::Subscription::new(rule));
            added = true;
        }
        if added {
            cmds.push(crate::radio::Cmd::CallSubs(self.subs.clone()));
        }
    }

    /// Subscribe to a rule, or drop it if it is already there.
    pub fn toggle(&mut self, rule: crate::mix::calls::Rule, cmds: &mut Vec<crate::radio::Cmd>) {
        match self.subs.iter().position(|s| s.rule == rule) {
            Some(i) => {
                self.subs.remove(i);
                self.optout.push(rule);
            }
            None => {
                self.optout.retain(|r| r != &rule);
                self.subs.push(crate::mix::calls::Subscription::new(rule));
            }
        }
        cmds.push(crate::radio::Cmd::CallSubs(self.subs.clone()));
    }
}

impl Default for CallsState {
    fn default() -> Self {
        Self {
            list: crate::calls::Calls::new(),
            subs: Vec::new(),
            optout: Vec::new(),
            recordings: Vec::new(),
            read_at: None,
            filter: String::new(),
            log_note: String::new(),
            timeline: Default::default(),
            saving: None,
            log_open: false,
            log_frac: 0.6,
            log_splitting: false,
        }
    }
}

impl MessagesState {
    /// The recent past off the disk, oldest first, so the view is not empty
    /// after a restart. The file is the record; this is the last couple of
    /// days of it.
    pub fn loaded() -> Self {
        let mut s = Self::default();
        let dir = crate::messagelog::messages_dir();
        for m in crate::messagelog::recent(&dir, crate::messagelog::LOAD_DAYS) {
            let at = m.last;
            s.list.push(m, at);
        }
        s
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

/// The transcript view: what was said, as the model read it.
///
/// `log` is a copy of the program's one transcript (`transcripts::log`),
/// taken whenever its sequence number moves, so the pane draws from
/// something it owns rather than holding the shared lock while it draws.
#[derive(Default)]
pub(super) struct TranscriptState {
    pub log: crate::transcripts::TranscriptLog,
    /// The conversation being read on its own, by its key, or `None` for
    /// everything the receiver heard. Set by the call list's own button.
    pub only: Option<common::ConversationKey>,
    /// What the operator typed in the filter box.
    pub filter: String,
    /// Whether the model has been asked to load, so the button says so once
    /// rather than every frame.
    pub asked: bool,
    /// The transcript's sequence number the copy was taken at.
    pub seq: u64,
}

/// The data links view: who is talking to whom, and which link is being
/// followed.
#[derive(Default)]
pub(super) struct LinksState {
    pub list: crate::links::Links,
    /// What the operator typed in the filter box.
    pub filter: String,
    /// The link being followed, by its title, so the choice survives the
    /// directory being rebuilt every frame.
    pub chosen: Option<String>,
    /// Why the last load from the log failed, when it did.
    pub error: Option<String>,
}

/// The control view: the handsets heard, and where their sticks are.
#[derive(Default)]
pub(super) struct ControlState {
    pub list: crate::control::Controls,
}

/// The key manager: the keys known, and what the operator is typing. The
/// store exists only with the `tea` feature (there is no key material to keep
/// without it); the view itself is always present as an encryption monitor.
pub(super) struct KeysState {
    /// Keys stored on disk, loaded at startup and written when one changes.
    /// The TETRA entries are read only with the `tea` feature; the channel
    /// keys of the mesh protocols are in force in every build.
    pub store: crate::keystore::KeyStore,
    /// The hex the operator is typing, per cell tag, before it is applied.
    #[cfg_attr(not(feature = "tea"), allow(dead_code))]
    pub typing: std::collections::HashMap<String, String>,
    /// The channel key being entered: protocol, name, key.
    pub new_system: decode::channel_keys::System,
    pub new_name: String,
    pub new_key: String,
    /// A Meshtastic node to read channels from, and the read in flight.
    pub node_host: String,
    pub node_fetch: Option<poll_promise::Promise<Result<Vec<crate::meshnode::Channel>, String>>>,
    /// What the last read said: how many came in, or what went wrong.
    pub node_result: Option<Result<String, String>>,
}

impl Default for KeysState {
    fn default() -> Self {
        let store = crate::keystore::KeyStore::load();
        store.publish();
        Self {
            store,
            typing: std::collections::HashMap::new(),
            new_system: decode::channel_keys::System::Meshtastic,
            new_name: String::new(),
            new_key: String::new(),
            node_host: String::new(),
            node_fetch: None,
            node_result: None,
        }
    }
}

/// The channel strip: every level that reaches the speaker.
pub(super) struct AudioState {
    pub channels: Vec<Channel>,
    /// The channel whose chain the signal chain view shows.
    pub listening: Option<usize>,
    pub volume: f32,
    /// Whether the bus passes anything at all: the master mute.
    pub muted: bool,
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
    /// The channel being keyed. Held here rather than read back off the key,
    /// because the key moves when the panel relaids itself and a transmission
    /// must not.
    pub keying: Keying,
    /// The `.sub` file the SUB transmit source plays, as parsed.
    pub sub_pick: SubPick,
    /// The capture the IQ transmit source replays.
    pub capture_pick: CapturePick,
}

/// What the transmit key is doing: which channel it is keying, and whether it
/// is being held or was latched with a right click.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Keying {
    pub at: Option<u64>,
    /// Latched, so letting the pointer go leaves the carrier up.
    pub latched: bool,
    /// What is being typed into a data mode's fields, by parameter name,
    /// until it is sent. The running stage holds what was sent, and a field
    /// that read it back every frame could not be typed into: the first
    /// keystroke would be replaced by the stage's own answer.
    pub fields: std::collections::HashMap<String, String>,
}

/// The `.sub` file loaded for the SUB transmit source, as the interface
/// parsed it. One at a time: the radio has one transmitter and the file is
/// what a channel keys, not a library.
#[derive(Default)]
pub(super) struct SubPick {
    /// The loaded file, held so the strip can show what it keys.
    pub file: Option<crate::radio::SubFile>,
    /// Why the last file would not play, said once.
    pub fault: Option<String>,
    picking: Option<poll_promise::Promise<Option<std::path::PathBuf>>>,
}

impl SubPick {
    /// Ask for a `.sub` file.
    pub fn ask(&mut self, ctx: &egui::Context) {
        if self.picking.is_some() {
            return;
        }
        let start = std::env::var_os("HOME")
            .map(std::path::PathBuf::from)
            .map(|h| h.join("Downloads"))
            .filter(|d| d.is_dir())
            .or_else(|| std::env::var_os("HOME").map(std::path::PathBuf::from))
            .unwrap_or_default();
        let ctx = ctx.clone();
        self.picking = Some(poll_promise::Promise::spawn_thread("open sub", move || {
            let picked = rfd::FileDialog::new()
                .set_title("Choose a Flipper .sub file")
                .set_directory(&start)
                .add_filter("Flipper SubGhz", &["sub"])
                .pick_file();
            // Nothing is drawing while the dialog is up, so the frame that
            // reads this has to be asked for.
            ctx.request_repaint();
            picked
        }));
    }

    /// Take the file the dialog came back with, once it has.
    pub fn poll(&mut self, cmds: &mut Vec<crate::radio::Cmd>) {
        if self.picking.as_ref().is_none_or(|p| p.ready().is_none()) {
            return;
        }
        let Some(path) = self.picking.take().and_then(|p| p.block_and_take()) else {
            return;
        };
        match crate::radio::SubFile::open(&path) {
            Ok(f) => {
                self.fault = None;
                // Both copies: the radio keys it, and the strip says what it
                // is keying. Without the second the strip went on offering
                // to choose a file after one had been chosen.
                self.file = Some(f.clone());
                cmds.push(crate::radio::Cmd::SubFile(Some(f)));
            }
            Err(e) => self.fault = Some(format!("{}: {e}", path.display())),
        }
    }
}

/// The capture loaded for the IQ transmit source.
///
/// The same shape as [`SubPick`] and for the same reason: the radio plays it
/// and the strip has to say what is about to go on the air. What is held here
/// is what the name resolved to, because a capture whose name does not carry
/// its rate cannot be replayed at all and saying so at the dialog is the only
/// place an operator can do anything about it.
#[derive(Default)]
pub(super) struct CapturePick {
    pub file: Option<crate::radio::TxCapture>,
    pub fault: Option<String>,
    picking: Option<poll_promise::Promise<Option<std::path::PathBuf>>>,
}

impl CapturePick {
    pub fn ask(&mut self, ctx: &egui::Context) {
        if self.picking.is_some() {
            return;
        }
        let start = crate::chain::default_capture_dir();
        let _ = std::fs::create_dir_all(&start);
        let ctx = ctx.clone();
        self.picking = Some(poll_promise::Promise::spawn_thread("open tx capture", move || {
            let picked = rfd::FileDialog::new()
                .set_title("Choose a capture to transmit")
                .set_directory(&start)
                .add_filter("IQ captures", &["cu8", "cs8", "cs16", "cf32", "data", "sigmf-data"])
                .pick_file();
            ctx.request_repaint();
            picked
        }));
    }

    pub fn poll(&mut self, cmds: &mut Vec<crate::radio::Cmd>) {
        if self.picking.as_ref().is_none_or(|p| p.ready().is_none()) {
            return;
        }
        let Some(path) = self.picking.take().and_then(|p| p.block_and_take()) else {
            return;
        };
        match crate::radio::TxCapture::open(&path) {
            Some(c) => {
                self.fault = None;
                self.file = Some(c.clone());
                cmds.push(crate::radio::Cmd::TxCapture(Some(c)));
            }
            None => {
                self.fault = Some(format!(
                    "{}: cannot tell its sample rate and format. Name it like \
                     <what>_<centre>_<rate>.<format>, e.g. bench_433.92M_250k.cu8",
                    path.display()
                ))
            }
        }
    }
}

impl Default for AudioState {
    fn default() -> Self {
        Self {
            channels: Vec::new(),
            listening: None,
            volume: 0.5,
            muted: false,
            next_id: 1,
            dial: Dial::new(),
            call_volume: 0.8,
            call_muted: false,
            call_agc: true,
            levels_rev: 0,
            keying: Keying::default(),
            sub_pick: SubPick::default(),
            capture_pick: CapturePick::default(),
        }
    }
}

/// A file being chosen for a stage's setting.
///
/// One at a time, wherever it was asked for: the chain view's inspector and
/// the channel strip both open the same dialog for the same stage, and what
/// comes back is a command like any other setting change. The dialog runs on
/// a thread of its own, because a window that stops painting while it is up
/// is a window the compositor puts a "not responding" notice over.
#[derive(Default)]
pub(super) struct FilePick {
    going: Option<(usize, String, poll_promise::Promise<Option<std::path::PathBuf>>)>,
}

impl FilePick {
    /// Ask for a file for `param` on the stage with this node id.
    pub fn ask(&mut self, ctx: &egui::Context, node: usize, param: &str, title: &str) {
        if self.going.is_some() {
            return;
        }
        // Where videos usually are on this machine, falling back to
        // wherever the receiver was started from.
        let start = std::env::var_os("HOME")
            .map(std::path::PathBuf::from)
            .map(|h| h.join("Videos"))
            .filter(|d| d.is_dir())
            .or_else(|| std::env::var_os("HOME").map(std::path::PathBuf::from))
            .unwrap_or_default();
        let (ctx, title) = (ctx.clone(), title.to_string());
        self.going = Some((
            node,
            param.to_string(),
            poll_promise::Promise::spawn_thread("open file", move || {
                let picked = rfd::FileDialog::new()
                    .set_title(&title)
                    .set_directory(&start)
                    .add_filter(
                        "Video and transport streams",
                        &["ts", "m2ts", "mpg", "mpeg", "mp4", "mkv", "mov", "avi", "webm", "m4v"],
                    )
                    .add_filter("Anything", &["*"])
                    .pick_file();
                // Nothing is drawing while the dialog is up, so the frame
                // that reads this has to be asked for.
                ctx.request_repaint();
                picked
            }),
        ));
    }

    /// Whether a dialog is up, so a second button press does nothing.
    pub fn busy(&self) -> bool {
        self.going.is_some()
    }

    /// Send what the dialog came back with, once it has.
    pub fn poll(&mut self, cmds: &mut Vec<crate::radio::Cmd>) {
        if self.going.as_ref().is_none_or(|(.., p)| p.ready().is_none()) {
            return;
        }
        let Some((node, param, promise)) = self.going.take() else {
            return;
        };
        let Some(path) = promise.block_and_take() else {
            return;
        };
        cmds.push(crate::radio::Cmd::NodeParam(
            node,
            param,
            pipeline::param::ParamValue::Text(path.display().to_string()),
        ));
    }
}

/// What a frequency list dialog came back with: channels to merge into the
/// bank, or a sentence saying why there are none.
pub(super) enum ListIo {
    Read(Box<crate::memory::formats::Read>, crate::memory::formats::Format),
    Said(String),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chain::derived;
    use pipeline::ParamValue as V;

    /// The graph the receiver draws before it has been handed anything.
    /// A graph with one stage carrying one setting the operator owns.
    ///
    /// The spectrum's, not the transcriber's: the transcriber's switch, its
    /// weights and its device are in the saved record rather than in the
    /// graph, so none of them is an edit any more.
    fn drawn() -> crate::patch::Patch {
        let mut p = crate::patch::Patch::default();
        let mut s = pipeline::registry::Settings::new();
        s.insert("smoothing".into(), V::Float(0.5));
        p.add_derived(derived::SPECTRUM, "spectrum", s);
        p
    }

    /// What was saved is not thrown away by the first graph the receiver
    /// publishes.
    ///
    /// The receiver comes up, draws a graph and publishes it before the
    /// interface has handed it the edits read off disk. Reading that graph
    /// gives an empty set, and adopting it wrote an empty file over the one
    /// just loaded: every setting changed by parameter had to be found again
    /// at the next start.
    #[test]
    fn a_saved_edit_survives_the_graph_published_before_it_lands() {
        let mut c = ChainState { edits: crate::patch::Edits::default(), ..Default::default() };
        // As `App::new` leaves it: read off disk, not yet seen running.
        c.edits.settings.push((derived::SPECTRUM, "smoothing".into(), V::Float(0.8)));
        assert!(!c.edits_landed);

        // The first publish: the receiver's own graph, without them.
        c.base = drawn();
        c.patch = drawn();
        c.take_edits_from_the_running_graph();
        assert_eq!(c.edits.settings.len(), 1, "the switch was thrown away before it was applied");

        // The next one, once the receiver has them: the same edits, still
        // there, and now known to have landed.
        let mut running = drawn();
        running
            .stage_mut(derived::SPECTRUM)
            .unwrap()
            .settings
            .insert("smoothing".into(), V::Float(0.8));
        c.patch = running;
        c.take_edits_from_the_running_graph();
        assert!(c.edits_landed);
        assert_eq!(c.edits.settings.len(), 1);

        // And now the operator switching it off is taken, because what it
        // reads is the graph they are actually looking at.
        c.patch = drawn();
        c.take_edits_from_the_running_graph();
        assert!(c.edits.settings.is_empty(), "the setting cannot be put back again");
    }

    /// A receiver with nothing saved adopts what it reads straight away.
    #[test]
    fn with_nothing_saved_the_first_graph_is_taken_as_it_is() {
        let mut c = ChainState::default();
        assert!(c.edits.is_empty());
        c.base = drawn();
        c.patch = drawn();
        c.take_edits_from_the_running_graph();
        assert!(c.edits_landed, "there was nothing to wait for");
    }
}
