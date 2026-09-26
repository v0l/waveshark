//! Draws the signal chain that is actually running.
//!
//! Read from the built graph, so it cannot describe a chain the radio is not
//! using. A diagram maintained alongside the code would be worth less than
//! nothing: wrong documentation is believed.
//!
//! The receiver is one graph and that graph branches: the spectrum, the
//! recorder, the channel banks and every channel being listened to all read
//! the same samples. So this draws a tree rather than a column, with a node's
//! depth set by the longest path that reaches it, and edges drawn back to
//! whichever node actually produced each input.

use egui::{Color32, FontFamily, FontId, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};
use egui_bench::text::Line;
use egui_bench::theme;
use pipeline::cost::Cost;
use pipeline::graph::Topology;
use pipeline::port::{PortKind, StreamSpec};
use std::collections::HashMap;

const BOX_W: f32 = 190.0;
const BOX_H: f32 = 46.0;
const GAP: f32 = 34.0;
/// Space between one column of stages and the next. Wide enough for the wire
/// label that sits in it: the rate a link carries is the most useful thing on
/// the screen and it used to be printed over the boxes on either side.
const COL_GAP: f32 = 120.0;
/// A composite node's inner chain, drawn smaller beneath it.
const INNER_H: f32 = 26.0;
const INNER_GAP: f32 = 8.0;
/// One line of a node's own cost breakdown, drawn under its box.
const PHASE_H: f32 = 13.0;
/// A scope is drawn as its readings: a wider, taller box with the meter, the
/// spectrum and the spectrogram inside it, in the graph where the wire is.
const SCOPE_W: f32 = 250.0;
const SCOPE_H: f32 = 210.0;

fn box_size(node: &pipeline::graph::TopoNode) -> Vec2 {
    if node.kind == "scope" { Vec2::new(SCOPE_W, SCOPE_H) } else { Vec2::new(BOX_W, BOX_H) }
}

/// Whether a stage can be dragged to another size.
fn resizable(node: &pipeline::graph::TopoNode) -> bool {
    node.kind == "scope"
}

/// Side of the square in a resizable box's corner that takes the drag.
const CORNER: f32 = 12.0;

/// What a stage costs, as text: its 95th percentile call and, where the
/// stage runs on samples, the share of real time that is.
///
/// The share is the number that matters. A millisecond means nothing until
/// it is set against the block it had to fit in, and it is the share that
/// says whether this stage is the reason the sparkline is red.
fn cost_label(c: &Cost) -> Option<String> {
    if c.calls == 0 {
        return None;
    }
    let t = if c.p95_us >= 10_000 {
        format!("{:.1} ms", c.p95_us as f32 / 1000.0)
    } else if c.p95_us >= 1000 {
        format!("{:.2} ms", c.p95_us as f32 / 1000.0)
    } else {
        format!("{} us", c.p95_us)
    };
    Some(match c.load() {
        Some(l) if l >= 0.1 => format!("{t}  {:.0}%", l * 100.0),
        Some(l) => format!("{t}  {:.1}%", l * 100.0),
        None => t,
    })
}

/// Legend under half real time, amber past it, fault red at and over one:
/// the same reading the sparkline gives for the whole graph, per stage.
fn cost_colour(c: &Cost) -> Color32 {
    match c.load() {
        Some(l) if l >= 1.0 => theme::FAULT,
        Some(l) if l >= 0.5 => theme::READOUT,
        _ => theme::LEGEND,
    }
}

fn kind_label(k: PortKind) -> &'static str {
    match k {
        PortKind::Iq => "iq",
        PortKind::Real => "real",
        PortKind::Bytes => "bytes",
        PortKind::Pulses => "pulses",
        PortKind::Timings => "timings",
        PortKind::Soft => "soft",
        PortKind::Packets => "packets",
        PortKind::Sources => "sources",
        PortKind::Voice => "voice",
        PortKind::Video => "video",
        PortKind::Spectrum => "spectrum",
    }
}

/// Rate as a human reads it, in the unit that suits the magnitude, or `None`
/// where the stream has no rate to report.
///
/// A packet bus carries events rather than samples: nothing on it arrives at a
/// fixed rate, and its spec says so by leaving the rate at zero. Printing that
/// as `0 S/s` claimed the wire was dead while packets were going down it. What
/// those wires get instead is [`flow_label`], measured by the graph. A source
/// port is the other case, carrying many streams each at its own rate,
/// where one number would have to be wrong about all but one of them.
fn rate_label(s: &StreamSpec) -> Option<String> {
    if matches!(s.kind, PortKind::Packets | PortKind::Sources) {
        return None;
    }
    let r = s.frame_rate();
    if !r.is_finite() || r <= 0.0 {
        return None;
    }
    let base = if r >= 1e6 {
        format!("{:.3} MS/s", r / 1e6)
    } else if r >= 10e3 {
        format!("{:.1} kS/s", r / 1e3)
    } else {
        format!("{r:.0} S/s")
    };
    Some(if s.channels > 1 { format!("{base} x{}", s.channels) } else { base })
}

/// What the graph measured on a wire that has no rate of its own, in items per
/// second.
///
/// Two significant figures at the bottom of the range, because the interesting
/// question about a bus carrying one message every few seconds is whether it is
/// carrying anything at all.
fn flow_label(rate: f32) -> String {
    if !rate.is_finite() || rate >= 100.0 {
        return format!("{rate:.0}/s");
    }
    if rate >= 10.0 {
        format!("{rate:.1}/s")
    } else if rate >= 0.05 {
        format!("{rate:.2}/s")
    } else {
        // Below this the figure is what is left in the smoothing rather than a
        // rate: one message every twenty seconds is better said as idle.
        "idle".to_string()
    }
}

/// What a wire carries, and how fast: its declared sample rate where it has
/// one, otherwise what the graph counted going down it.
fn wire_label(s: &StreamSpec, measured: Option<f32>) -> String {
    let kind = kind_label(s.kind);
    match (rate_label(s), measured) {
        (Some(r), _) => format!("{kind}  {r}"),
        (None, Some(r)) => format!("{kind}  {}", flow_label(r)),
        (None, None) => kind.to_string(),
    }
}

/// Where a node sits: which row, and which column of the whole drawing.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Place {
    depth: usize,
    col: usize,
}

/// Rows and columns for a graph that branches.
///
/// Depth is the longest path from the source rather than the shortest, so a
/// node never sits above something it consumes.
///
/// Columns are inherited: a node takes its parent's column if it is the first
/// thing that parent feeds, and otherwise starts a new one. That keeps each
/// branch in a straight vertical trunk, which matters because the interesting
/// question about this graph is which stages belong to the same path. Packing
/// each row independently reads as a pile of unrelated boxes.
fn layout(topo: &Topology) -> Vec<Place> {
    let n = topo.nodes.len();
    let index = |id| topo.nodes.iter().position(|x| x.id == id);
    let mut depth = vec![0usize; n];
    let mut col = vec![usize::MAX; n];
    let mut taken = vec![false; n];
    let mut next_col = 0usize;

    // Nodes arrive in execution order, so one forward pass settles both: a
    // producer is always visited before anything it feeds.
    for i in 0..n {
        let parents: Vec<usize> = topo.nodes[i]
            .inputs
            .iter()
            .filter_map(|(slot, _)| topo.producer(*slot))
            .filter_map(|p| index(p.id))
            .collect();
        depth[i] = parents.iter().map(|&j| depth[j] + 1).max().unwrap_or(0);
        col[i] = match parents.iter().find(|&&j| !taken[j]) {
            Some(&j) => {
                taken[j] = true;
                col[j]
            }
            None => {
                let c = next_col;
                next_col += 1;
                c
            }
        };
    }
    (0..n).map(|i| Place { depth: depth[i], col: col[i] }).collect()
}

/// How tall a lane is, allowing for any composite node's inner chain.
fn lane_height(topo: &Topology, places: &[Place], lane: usize) -> f32 {
    let inner = topo
        .nodes
        .iter()
        .zip(places)
        .filter(|(_, p)| p.col == lane)
        .map(|(n, _)| n.inner.iter().map(|t| t.nodes.len()).sum::<usize>())
        .max()
        .unwrap_or(0);
    let phases = topo
        .nodes
        .iter()
        .zip(places)
        .filter(|(_, p)| p.col == lane)
        .map(|(n, _)| n.phases.len())
        .max()
        .unwrap_or(0);
    let extra = if inner == 0 { 0.0 } else { INNER_GAP + inner as f32 * (INNER_H + INNER_GAP) };
    let phase_extra = if phases == 0 { 0.0 } else { INNER_GAP + phases as f32 * PHASE_H };
    let tallest = topo
        .nodes
        .iter()
        .zip(places)
        .filter(|(_, p)| p.col == lane)
        .map(|(n, _)| box_size(n).y)
        .fold(BOX_H, f32::max);
    tallest + GAP + extra + phase_extra
}

/// Where the operator has put the stages, when they have moved any.
///
/// Positions are held here rather than worked out from the graph because in
/// manual mode they are no longer a function of it: the point of moving a box
/// is that it stays where it was put, including across the rebuild that a
/// parameter change causes.
///
/// Kept relative to the drawing's top left, so the arrangement survives the
/// pane being resized or scrolled.
pub struct Edit {
    /// The operator owns the shape of the graph, so boxes can be dragged and
    /// wires drawn between ports.
    pub manual: bool,
    /// Centre of each stage relative to the drawing's top left, by the key
    /// that survives a rebuild.
    pub pos: HashMap<u64, Pos2>,
    /// Size of each stage the operator has resized, by the same key. Only a
    /// scope offers a corner to drag; the rest are the size their text needs.
    pub size: HashMap<u64, Vec2>,
    /// Where each box was last drawn, in screen coordinates, and where the
    /// source box was. Kept so that anything outside the drawing can point at
    /// a stage without repeating the layout arithmetic.
    pub drawn: HashMap<u64, Rect>,
    pub drawn_src: Rect,
    origin: Pos2,
    view: Rect,
    /// What the pointer took hold of, if anything.
    drag: Option<Drag>,
}

pub struct Carried(pub String);

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Attach {
    Nothing,
    Feeds((u64, usize)),
    Reads(crate::patch::Source),
    Splice(crate::patch::Source, (u64, usize)),
}

#[derive(Clone, Copy)]
struct Ghost {
    id: u64,
    r: Rect,
    ins: usize,
    outs: usize,
}

/// What a drag in progress is doing.
#[derive(Clone, Copy)]
enum Drag {
    /// Moving a box: which one, and where in it the pointer took hold.
    Node(u64, Vec2),
    /// Resizing a box from its bottom right corner.
    Resize(u64),
    /// Drawing a wire, which can be started at either end: from an output
    /// looking for something to feed, or from an input looking for something
    /// to read. Both are how people reach for a connection, and a view that
    /// only accepts one of them feels broken to whoever reached the other
    /// way.
    Wire { from: Option<crate::patch::Source>, to: Option<(u64, usize)>, at: Pos2 },
    /// Moving the view. A drag that started on nothing is a drag of the
    /// canvas: in manual mode the pane takes every drag so that a stage can
    /// be moved, and without this the graph could not be panned at all.
    Pan,
}

/// How a node is recognised between one rebuild and the next.
///
/// A stage the operator drew carries its patch id as a tag; everything the
/// receiver builds for itself has none and is keyed by its position, from
/// below the block the receiver's own stages are named in. Overlapping that
/// block put the DC block and the spectrum on the same key, and two boxes
/// with one position are drawn on top of each other with every wire in the
/// graph converging on the pile.
fn keys(topo: &Topology) -> Vec<u64> {
    let mut seen: HashMap<(String, String), u32> = HashMap::new();
    topo.nodes
        .iter()
        .map(|n| match n.tag {
            Some(t) => t,
            None => {
                let nth = seen.entry((n.label.clone(), n.kind.clone())).or_insert(0);
                *nth += 1;
                stable_key(&n.label, &n.kind, *nth)
            }
        })
        .collect()
}

/// A name for a node the receiver built for itself.
///
/// From what it is rather than where it is. A `NodeId` is a position, and
/// every patch edit renumbers them, so keying by id meant the whole automatic
/// chain jumped to a fresh layout each time a stage was added: the view
/// looked broken because it was redrawing the same graph under new names.
fn stable_key(label: &str, kind: &str, nth: u32) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    label.hash(&mut h);
    kind.hash(&mut h);
    nth.hash(&mut h);
    // Clear of the patch's own ids, which count up from one, and of the
    // block the receiver's stages are named in.
    1_000_000 + h.finish() % (crate::patch::builtin::FIRST - 2_000_000)
}

impl Default for Edit {
    fn default() -> Self {
        Self {
            manual: false,
            pos: HashMap::new(),
            size: HashMap::new(),
            drawn: HashMap::new(),
            drawn_src: Rect::NOTHING,
            origin: Pos2::ZERO,
            view: Rect::NOTHING,
            drag: None,
        }
    }
}

impl Edit {
    /// Forget the arrangement, so the automatic layout draws it again.
    pub fn arrange(&mut self) {
        self.pos.clear();
        self.size.clear();
        self.drag = None;
    }

    /// Whether anything has been moved by hand.
    pub fn moved(&self) -> bool {
        !self.pos.is_empty()
    }

    pub fn arrange_keeping(&mut self, keep: impl Fn(u64) -> bool) {
        self.pos.retain(|k, _| keep(*k));
        self.size.retain(|k, _| keep(*k));
        self.drag = None;
    }

    pub fn free_spot(&self) -> Pos2 {
        self.free_spot_among(&[])
    }

    fn free_spot_among(&self, also: &[Rect]) -> Pos2 {
        let view = if self.view.is_positive() {
            self.view
        } else {
            Rect::from_min_size(self.origin, Vec2::new(BOX_W * 3.0, BOX_H * 6.0))
        };
        let start = view.center();
        let step = BOX_H + GAP / 2.0;
        let taken = |c: Pos2| {
            let r = Rect::from_center_size(c, Vec2::new(BOX_W, BOX_H)).expand(GAP / 4.0);
            self.drawn.values().chain(also).any(|d| d.intersects(r))
        };
        let spot = (0..24)
            .map(|i| {
                let n = (i as f32 + 1.0) / 2.0;
                let dir = if i % 2 == 0 { 1.0 } else { -1.0 };
                start + Vec2::new(0.0, dir * n.floor() * step)
            })
            .find(|c| !taken(*c))
            .unwrap_or(start);
        spot - self.origin.to_vec2()
    }
}

/// What the operator did to the chain view this frame.
#[derive(Default)]
pub struct Interaction {
    /// The node now selected, or `None` if the click was on empty space.
    pub selected: Option<usize>,
    /// A parameter that was changed: node id, name, new value.
    pub changed: Option<(usize, String, pipeline::param::ParamValue)>,
    /// A wire drawn: where it comes from, and the stage and input port it
    /// was dropped on.
    pub link: Option<(crate::patch::Source, u64, usize)>,
    /// A wire pulled off an input port.
    pub unlink: Option<(u64, usize)>,
    /// Every wire pulled off an output port.
    pub unlink_out: Option<crate::patch::Source>,
    /// The operator's own stage that was last clicked, by patch id. Kept
    /// apart from `selected` because a stage waiting to be wired up is not in
    /// the running graph and so has no node to inspect.
    pub picked: Option<u64>,
    /// A wire that was clicked, named by the end that is unique: an input
    /// takes one producer, so the stage and port it lands on says which wire
    /// is meant.
    pub wire: Option<(u64, usize)>,
    /// How far the canvas was dragged this frame.
    pub pan: Vec2,
    pub dropped: Option<(String, Pos2, Attach)>,
    pub blank: bool,
    pub bounds: Option<Rect>,
}

/// The selected node's settings, as controls.
///
/// Every stage already describes its own knobs as data, so this renders a
/// decoder it has never heard of. That is the point of the parameter
/// description existing at all: without it, each stage would need its own
/// panel written by hand and the ones nobody wrote would be unreachable.
pub const SOURCE: usize = pipeline::graph::GRAPH_INPUT.node.0;

pub fn inspector(
    ui: &mut egui::Ui,
    topo: &Topology,
    selected: usize,
    browse: &mut Option<(usize, String)>,
    off: &mut Option<(u64, bool)>,
) -> Option<(usize, String, pipeline::param::ParamValue)> {
    if selected == SOURCE {
        source_inspector(ui, topo, off);
        return None;
    }
    let node = topo.nodes.iter().find(|n| n.id.0 == selected)?;

    Line::new().legend(&node.label).elided(ui);
    small(ui, &node.kind, theme::LEGEND);
    ui.add_space(6.0);
    for (slot, spec) in &node.inputs {
        small(ui, &format!("in  {}", wire_label(spec, topo.rate_of(*slot))), theme::LEGEND);
    }
    if !node.sink {
        for (slot, spec) in &node.outputs {
            small(ui, &format!("out {}", wire_label(spec, topo.rate_of(*slot))), theme::TRACE);
        }
    }
    if !node.readings.is_empty() {
        ui.add_space(6.0);
        for (what, value) in &node.readings {
            Line::new().legend(what).gap(6.0).value(value.clone()).size(11.0).elided(ui);
        }
    }
    if let Some(text) = cost_label(&node.cost) {
        ui.add_space(6.0);
        small(ui, &format!("p95 {text}"), cost_colour(&node.cost));
        for (name, c) in &node.phases {
            if let Some(t) = cost_label(c) {
                small(ui, &format!("  {name}  {t}"), cost_colour(c));
            }
        }
    }
    ui.add_space(12.0);

    if let Some(tag) = node.tag {
        let mut on = !node.off;
        let help = "Off stops this stage and every stage that only it feeds. Kept across restarts.";
        if egui_bench::form::switch(ui, "run", &mut on, "this stage", help) {
            *off = Some((tag, !on));
        }
        if node.idle && !node.off {
            egui_bench::text::hint(ui, "idle: nothing feeding it is running");
        }
        ui.add_space(6.0);
    }

    if node.params.is_empty() {
        Line::new().note("This stage has nothing to set.").show(ui);
        return None;
    }

    let mut out = None;
    for prm in &node.params {
        match param_control(ui, node.id.0 as u64, prm, &prm.value, true) {
            Control::Set(v, _) => out = Some((node.id.0, prm.name.clone(), v)),
            Control::Browse => *browse = Some((node.id.0, prm.name.clone())),
            Control::Nothing => {}
        }
    }
    out
}

pub fn waiting_inspector(
    ui: &mut egui::Ui,
    stage: &crate::patch::Stage,
    waiting: Option<&crate::chain::Waiting>,
) -> Option<(String, pipeline::param::ParamValue)> {
    Line::new().legend(&stage.kind).elided(ui);
    small(ui, "not connected", theme::LEGEND);
    ui.add_space(6.0);
    let Some(w) = waiting else {
        Line::new().note("Its settings show once the receiver has built it.").wrapped(ui);
        return None;
    };
    small(ui, &format!("{} in  {} out", w.inputs, w.outputs), theme::LEGEND);
    ui.add_space(12.0);
    if w.params.is_empty() {
        Line::new().note("This stage has nothing to set.").show(ui);
        return None;
    }
    let mut out = None;
    for prm in &w.params {
        let shown = stage.settings.get(&prm.name).unwrap_or(&prm.value);
        if let Control::Set(v, true) = param_control(ui, stage.id, prm, shown, false) {
            out = Some((prm.name.clone(), v));
        }
    }
    out
}

fn small(ui: &mut egui::Ui, text: &str, colour: Color32) {
    Line::new().value(text).size(10.0).tint(colour).elided(ui);
}

enum Control {
    Nothing,
    Set(pipeline::param::ParamValue, bool),
    Browse,
}

fn param_control(
    ui: &mut egui::Ui,
    salt: u64,
    prm: &pipeline::param::Param,
    shown: &pipeline::param::ParamValue,
    can_browse: bool,
) -> Control {
    use pipeline::param::{ParamRange, ParamValue};
    let name = if prm.label.is_empty() { prm.name.clone() } else { prm.label.clone() };
    let key = ui.id().with(("draft", salt, &prm.name));
    let mut out = Control::Nothing;
    let rebuilds = "Changing this rebuilds the chain.";
    if !matches!(shown, ParamValue::Bool(_)) {
        Line::new().legend(&name).elided(ui);
    }
    match (shown, &prm.range) {
        (ParamValue::Float(v), ParamRange::Float { range, log }) => {
            let mut x = ui.data(|d| d.get_temp::<f64>(key)).unwrap_or(*v);
            let r = ui
                .add(egui::Slider::new(&mut x, range.clone()).suffix(unit(prm)).logarithmic(*log));
            out = slid(ui, key, &r, x, ParamValue::Float);
        }
        (ParamValue::Int(v), ParamRange::Int { range }) => {
            let mut x = ui.data(|d| d.get_temp::<i64>(key)).unwrap_or(*v);
            let r = ui.add(egui::Slider::new(&mut x, range.clone()).suffix(unit(prm)));
            out = slid(ui, key, &r, x, ParamValue::Int);
        }
        (ParamValue::Bool(v), _) => {
            let mut x = *v;
            let help = if prm.affects_rate { rebuilds } else { name.as_str() };
            if egui_bench::form::switch(ui, "", &mut x, &name, help) {
                out = Control::Set(ParamValue::Bool(x), true);
            }
        }
        (ParamValue::Choice(i), ParamRange::Choices(choices)) => {
            let mut pick = *i;
            let options = choices.iter().cloned().enumerate();
            if egui_bench::form::choice(ui, (key, "choice"), &mut pick, options) {
                out = Control::Set(ParamValue::Choice(pick), true);
            }
        }
        (ParamValue::Text(s), _) => {
            let mut t = ui.data(|d| d.get_temp::<String>(key)).unwrap_or_else(|| s.clone());
            // A path gets a dialog beside the box. Typing one in still
            // works, and is the only way to reach a file on a machine
            // with no file manager behind the dialog.
            let file = can_browse && prm.name == "path";
            let mut browse = false;
            let r = ui
                .horizontal(|ui| {
                    egui_bench::form::field_then(
                        ui,
                        &mut t,
                        "",
                        if file { 30.0 } else { 0.0 },
                        |ui| {
                            if file && ui.button("…").on_hover_text("Choose a file").clicked() {
                                browse = true;
                            }
                        },
                    )
                })
                .inner;
            if r.has_focus() {
                ui.data_mut(|d| d.insert_temp(key, t.clone()));
            } else {
                ui.data_mut(|d| d.remove::<String>(key));
            }
            if r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                out = Control::Set(ParamValue::Text(t), true);
            } else if browse {
                out = Control::Browse;
            }
        }
        // A value whose range says something else about it: show the
        // number rather than a control that would write the wrong type.
        (v, _) => {
            Line::new().value(format!("{v:?}")).size(11.0).show(ui);
        }
    }
    if prm.affects_rate && !matches!(shown, ParamValue::Bool(_)) {
        egui_bench::text::hint(ui, "changing this rebuilds the chain");
    }
    ui.add_space(8.0);
    out
}

fn slid<T: Clone + Send + Sync + 'static>(
    ui: &egui::Ui,
    key: egui::Id,
    r: &egui::Response,
    x: T,
    wrap: fn(T) -> pipeline::param::ParamValue,
) -> Control {
    if r.dragged() {
        ui.data_mut(|d| d.insert_temp(key, x.clone()));
    } else {
        ui.data_mut(|d| d.remove::<T>(key));
    }
    if r.drag_stopped() {
        return Control::Set(wrap(x), true);
    }
    if r.changed() {
        return Control::Set(wrap(x), !r.dragged());
    }
    Control::Nothing
}

fn source_inspector(ui: &mut egui::Ui, topo: &Topology, off: &mut Option<(u64, bool)>) {
    Line::new().legend("Source").show(ui);
    small(ui, "device", theme::LEGEND);
    ui.add_space(6.0);
    small(ui, &format!("out {}", wire_label(&topo.input, topo.rate_of(0))), theme::TRACE);
    ui.add_space(12.0);
    let mut on = !topo.input_off;
    let help = "Off stops every receive stage: the spectrum, the channels and the decoders. \
                The transmitter keeps running. Kept across restarts.";
    if egui_bench::form::switch(ui, "receive", &mut on, "from the radio", help) {
        *off = Some((crate::patch::builtin::SPAN, !on));
    }
}

fn unit(p: &pipeline::param::Param) -> String {
    if p.unit.is_empty() { String::new() } else { format!(" {}", p.unit) }
}

#[allow(clippy::too_many_arguments)]
pub fn draw(
    ui: &mut egui::Ui,
    topo: &Topology,
    latency_ms: f64,
    selected: Option<usize>,
    edit: &mut Edit,
    patch: Option<&crate::patch::Patch>,
    wire: Option<(u64, usize)>,
    picked_ghost: Option<u64>,
    elsewhere: &[u64],
    scopes: &[(usize, nodes::ScopeFrame)],
    waiting: &[crate::chain::Waiting],
) -> Interaction {
    let places = layout(topo);
    // Depth runs left to right and a branch takes a lane of its own down the
    // screen, which is the way every flowgraph editor draws a graph.
    let columns = places.iter().map(|p| p.depth + 1).max().unwrap_or(0);
    let lanes = places.iter().map(|p| p.col + 1).max().unwrap_or(1).max(1);

    let box_w = BOX_W;
    // A lane is as tall as the tallest thing in it, which for a bank is its
    // own inner chain drawn underneath it.
    let lane_h: Vec<f32> = (0..lanes).map(|l| lane_height(topo, &places, l)).collect();
    let height = (lane_h.iter().sum::<f32>() + 40.0).max(ui.available_height());
    let width = ((columns + 1) as f32 * (box_w + COL_GAP) + 24.0).max(ui.available_width());

    let (rect, resp) = ui.allocate_exact_size(
        Vec2::new(width, height.max(ui.available_height())),
        if edit.manual { Sense::click_and_drag() } else { Sense::click() },
    );
    let p = ui.painter_at(rect);
    let mut act = Interaction { selected, ..Default::default() };
    edit.origin = rect.min;
    edit.view = ui.clip_rect().intersect(rect);
    let pointer = resp.hover_pos();

    p.text(
        Pos2::new(rect.left() + 12.0, rect.top() + 4.0),
        egui::Align2::LEFT_TOP,
        format!("{} stages   {latency_ms:.1} ms through the chain", topo.nodes.len()),
        theme::figure(10.0),
        theme::LEGEND,
    );

    // The source on the left, then a column per depth.
    let top = rect.top() + 24.0;
    let left = rect.left() + 12.0;
    let src_auto = Pos2::new(left + box_w / 2.0, top + BOX_H / 2.0);
    // The span is a box like any other in manual mode: it can be dragged, and
    // it is where every wire that reads the receiver's own samples starts.
    let src_centre = if edit.manual {
        rect.min
            + edit
                .pos
                .entry(crate::patch::builtin::SPAN)
                .or_insert(src_auto - rect.min.to_vec2())
                .to_vec2()
    } else {
        src_auto
    };
    let src = Rect::from_center_size(src_centre, Vec2::new(box_w, BOX_H));
    stage(&p, src, "Source", if topo.input_off { "off" } else { "device" }, true, None);
    if selected == Some(SOURCE) || pointer.is_some_and(|q| src.contains(q)) {
        p.rect_stroke(
            src.expand(1.0),
            4.0,
            Stroke::new(if selected == Some(SOURCE) { 1.5 } else { 1.0 }, theme::READOUT),
            StrokeKind::Outside,
        );
    }

    let mut lane_top = vec![0.0f32; lanes];
    let mut y = top;
    for (l, h) in lane_h.iter().enumerate() {
        lane_top[l] = y;
        y += h;
    }

    // Places first, then edges, then the boxes on top of them. Drawn the
    // other way round, an edge that has to cross a lane is painted over the
    // stage it crosses, and a bank's inner chain ends up with a wire through
    // the middle of it.
    let node_keys = keys(topo);
    let mut rects: Vec<Rect> = Vec::with_capacity(topo.nodes.len());
    for ((i, node), pl) in topo.nodes.iter().enumerate().zip(&places) {
        let size = match edit.size.get(&node_keys[i]) {
            Some(s) if resizable(node) => *s,
            _ => box_size(node),
        };
        let x = left + (pl.depth + 1) as f32 * (box_w + COL_GAP) + box_w / 2.0;
        let auto = Pos2::new(x, lane_top[pl.col] + size.y / 2.0);
        // Entering manual mode pins every stage where the automatic layout
        // had just put it. Without that, moving one box would let the rest
        // reflow around the gap it left, which reads as the graph rearranging
        // itself in response to being touched.
        let centre = if edit.manual {
            rect.min + edit.pos.entry(node_keys[i]).or_insert(auto - rect.min.to_vec2()).to_vec2()
        } else {
            auto
        };
        rects.push(Rect::from_center_size(centre, size));
    }

    // Stages that have been added but are not running: a stage whose inputs
    // are not all fed is left out of the built graph, so without these the
    // one thing you cannot do is wire up a stage you just added.
    let mut ghosts: Vec<Ghost> = Vec::new();
    if edit.manual {
        if let Some(patch) = patch {
            // Not running here and not running in the other half either.
            // The view draws one direction at a time, and a receive stage
            // is not a stage waiting to be wired just because the transmit
            // half is showing: every one of them appeared as a ghost under
            // the source the moment manual mode was entered on that view.
            for st in patch.stages().iter().filter(|s| {
                !topo.nodes.iter().any(|t| t.tag == Some(s.id)) && !elsewhere.contains(&s.id)
            }) {
                let at = match edit.pos.get(&st.id) {
                    Some(at) => *at,
                    None => {
                        let placed: Vec<Rect> =
                            rects.iter().copied().chain(ghosts.iter().map(|g| g.r)).collect();
                        let seed = edit.free_spot_among(&placed);
                        edit.pos.insert(st.id, seed);
                        seed
                    }
                };
                let desc = waiting.iter().find(|w| w.id == st.id);
                ghosts.push(Ghost {
                    id: st.id,
                    r: Rect::from_center_size(rect.min + at.to_vec2(), Vec2::new(box_w, BOX_H)),
                    ins: desc.map_or(1, |w| w.inputs),
                    outs: desc.map_or(1, |w| w.outputs),
                });
            }
        }
    }

    if edit.manual {
        // A stage that is neither running nor waiting is no longer anywhere.
        edit.pos.retain(|k, _| {
            *k == crate::patch::builtin::SPAN
                || node_keys.contains(k)
                || ghosts.iter().any(|g| g.id == *k)
        });
        let press = ui.input(|i| i.pointer.press_origin());
        interact(
            &resp, press, topo, &node_keys, &rects, &ghosts, src, rect.min, edit, &mut act, patch,
        );
        // Dragging empty canvas moves the view. The pane owns every drag in
        // manual mode so that a stage can be moved, which leaves the scroll
        // area nothing to scroll with, so the drag is handed back to it here.
        if act.pan != Vec2::ZERO {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
        }
        for i in 0..topo.nodes.len() {
            if let Some(p) = edit.pos.get(&node_keys[i]) {
                rects[i] = Rect::from_center_size(rect.min + p.to_vec2(), rects[i].size());
            }
        }
        for g in ghosts.iter_mut() {
            if let Some(p) = edit.pos.get(&g.id) {
                g.r = Rect::from_center_size(rect.min + p.to_vec2(), g.r.size());
            }
        }
    }

    edit.drawn_src = src;
    edit.drawn.clear();
    edit.drawn.insert(crate::patch::builtin::SPAN, src);
    for i in 0..topo.nodes.len() {
        edit.drawn.insert(node_keys[i], rects[i]);
    }
    for g in &ghosts {
        edit.drawn.insert(g.id, g.r);
    }
    act.bounds = Some(edit.drawn.values().fold(src, |all, r| all.union(*r)));

    for (i, node) in topo.nodes.iter().enumerate() {
        for (k, (slot, spec)) in node.inputs.iter().enumerate() {
            let to = port(rects[i], k, node.inputs.len(), Side::In);
            // Which of the producer's outputs this is, so two streams out of
            // one stage leave it at two different points rather than crossing
            // inside the box.
            let from = match topo.producer(*slot) {
                Some(prod) => {
                    let j = topo.nodes.iter().position(|x| x.id == prod.id);
                    let out = prod.outputs.iter().position(|(s, _)| s == slot).unwrap_or(0);
                    match j {
                        Some(j) => port(rects[j], out, prod.outputs.len(), Side::Out),
                        None => port(src, 0, 1, Side::Out),
                    }
                }
                // No producer means it reads the graph input directly.
                None => port(src, 0, 1, Side::Out),
            };
            // Labelled once per node, and not at all on a node that gathers
            // many inputs: a bus with six wires into it would stack six
            // identical labels on the same three pixels.
            let label = node.inputs.len() <= 2 && slot == &node.inputs[0].0;
            // A wire can be taken hold of anywhere along it, not only at the
            // port it lands on: reaching for the line you can see is what
            // anybody tries first.
            let mine = edit.manual && node.tag.is_some();
            // The pointer where it is, or where it was pressed: the frame a
            // click lands in is the frame the button came up, and by then
            // egui no longer reports a hover position.
            let at = pointer.or_else(|| resp.interact_pointer_pos());
            let on = mine && at.is_some_and(|q| near_wire(from, to, q));
            let chosen = mine && node.tag.map(|t| (t, k)) == wire;
            edge(&p, from, to, spec, topo.rate_of(*slot), label, on || chosen);
            if on && resp.clicked() {
                act.wire = node.tag.map(|t| (t, k));
            }
        }
    }

    // Wires into a stage that is not running yet. The graph cannot show these
    // because it does not contain them, and they are most of what the
    // operator is looking at while wiring something up.
    if let Some(patch) = patch.filter(|_| edit.manual) {
        for g in &ghosts {
            for l in patch.links().iter().filter(|l| l.to.0 == g.id) {
                let from = wire_start(topo, &rects, &ghosts, src, l.from);
                loose(&p, from, port(g.r, l.to.1, g.ins.max(1), Side::In));
            }
        }
    }

    // The wire being drawn, with its loose end on the pointer. Drawn like any
    // other so that what it will look like once dropped is what is on screen
    // while deciding where to drop it.
    if let Some(Drag::Wire { from, to, at }) = edit.drag {
        let anchor = match (from, to) {
            (Some(from), _) => wire_start(topo, &rects, &ghosts, src, from),
            // Pulled off an input that had nothing on it: the wire hangs from
            // the port it is looking for a source for.
            (None, Some((tag, k))) => in_port_of(topo, &rects, &ghosts, tag, k).unwrap_or(at),
            (None, None) => at,
        };
        loose(&p, anchor, at);
        // What it would attach to, marked while the pointer is over it. A
        // wire that quietly does nothing when let go is the whole reason
        // this view felt broken.
        // Mirrors what letting go would do: a wire held by its input end is
        // looking for something to read, and one drawn out of an output is
        // looking for something to feed.
        let onto = match to {
            Some(_) => match output_at(topo, &rects, &ghosts, src, at)
                .or_else(|| body_output(topo, &rects, &ghosts, src, at))
            {
                Some(crate::patch::Source::Stage(tag, _)) => Some(tag),
                Some(crate::patch::Source::Span) => None,
                None if from.is_some() => input_at(topo, &rects, &ghosts, at)
                    .or_else(|| body_input(topo, &rects, &ghosts, at))
                    .map(|(tag, _)| tag),
                None => None,
            },
            None => input_at(topo, &rects, &ghosts, at)
                .or_else(|| body_input(topo, &rects, &ghosts, at))
                .map(|(tag, _)| tag),
        };
        if let Some(r) = onto.and_then(|tag| edit.drawn.get(&tag)) {
            target_ring(&p, *r);
        }
    }

    for g in &ghosts {
        let r = &g.r;
        let kind = patch.and_then(|p| p.stage(g.id)).map(|s| s.kind.clone()).unwrap_or_default();
        let picked = picked_ghost == Some(g.id);
        p.rect(
            *r,
            3.0,
            theme::WELL,
            Stroke::new(if picked { 2.0 } else { 1.0 }, theme::READOUT),
            StrokeKind::Inside,
        );
        p.text(
            Pos2::new(r.center().x, r.top() + 9.0),
            egui::Align2::CENTER_TOP,
            &kind,
            FontId::new(12.0, FontFamily::Proportional),
            theme::VALUE,
        );
        p.text(
            Pos2::new(r.center().x, r.top() + 26.0),
            egui::Align2::CENTER_TOP,
            "not connected",
            theme::figure(10.0),
            theme::LEGEND,
        );
        for k in 0..g.ins {
            knob(&p, port(*r, k, g.ins, Side::In), true, pointer);
        }
        for k in 0..g.outs {
            knob(&p, port(*r, k, g.outs, Side::Out), true, pointer);
        }
        if pointer.is_some_and(|q| r.contains(q)) && resp.clicked() {
            act.picked = Some(g.id);
        }
    }

    for (i, node) in topo.nodes.iter().enumerate() {
        let r = rects[i];
        let x = r.center().x;
        let hot = pointer.is_some_and(|q| r.contains(q));
        let on = selected == Some(node.id.0);
        let kind = match (node.off, node.idle) {
            (true, _) => "off",
            (false, true) => "idle",
            (false, false) => node.kind.as_str(),
        };
        stage(&p, r, &node.label, kind, node.idle, Some(&node.cost).filter(|_| !node.idle));
        if node.kind == "scope" {
            let frame = scopes.iter().find(|(id, _)| *id == node.id.0).map(|(_, f)| f);
            paint_scope(ui.ctx(), &p, r, node.id.0, frame);
            if edit.manual {
                // The grip: two short diagonals in the corner, the mark
                // every resizable window has carried.
                let c =
                    Rect::from_min_max(Pos2::new(r.right() - CORNER, r.bottom() - CORNER), r.max);
                let hot_c = pointer.is_some_and(|q| c.contains(q));
                let col = if hot_c { theme::READOUT } else { theme::LEGEND };
                for k in [4.0f32, 8.0] {
                    p.line_segment(
                        [
                            Pos2::new(r.right() - 2.0, r.bottom() - k - 2.0),
                            Pos2::new(r.right() - k - 2.0, r.bottom() - 2.0),
                        ],
                        Stroke::new(1.0, col),
                    );
                }
            }
        }
        if on || hot {
            p.rect_stroke(
                r.expand(1.0),
                4.0,
                Stroke::new(if on { 1.5 } else { 1.0 }, theme::READOUT),
                StrokeKind::Outside,
            );
        }
        if hot && resp.clicked() && edit.drag.is_none() {
            // Clicking the selected stage again closes the inspector, so the
            // panel is not a thing you have to hunt for a way out of.
            act.selected = if on { None } else { Some(node.id.0) };
            act.picked = node.tag;
        }
        // The ports themselves, so where a wire may be attached is visible
        // rather than inferred from where the ones already there happen to
        // land. A port that can be dragged is lit; one on a stage the
        // receiver built for itself is not, because its wiring is not the
        // patch's to change and a port that looks live but refuses every drag
        // is worse than one that never invited the attempt.
        let live = edit.manual && node.tag.is_some();
        for k in 0..node.inputs.len() {
            knob(&p, port(r, k, node.inputs.len(), Side::In), live, pointer);
        }
        if !node.sink {
            for k in 0..node.outputs.len() {
                knob(&p, port(r, k, node.outputs.len(), Side::Out), live, pointer);
            }
        }
        if !node.params.is_empty() {
            // A dot for a stage that has settings, so which boxes are worth
            // clicking is visible without clicking all of them.
            p.circle_filled(
                Pos2::new(r.right() - 7.0, r.top() + 7.0),
                2.0,
                if on { theme::READOUT } else { theme::LEGEND },
            );
        }

        // Where this node's own time goes, when it can say.
        let below = if node.phases.is_empty() { r.bottom() } else { phases(&p, r, &node.phases) };

        // A composite draws what it runs inside itself, so a bank is not an
        // opaque box with several hundred decoders hidden in it, and a node
        // holding a graph per front end is not one box for all of them.
        let mut iy = below + INNER_GAP;
        for (g, inner) in node.inner.iter().enumerate() {
            for (k, sub) in inner.nodes.iter().enumerate() {
                let ir = Rect::from_center_size(
                    Pos2::new(x, iy + INNER_H / 2.0),
                    Vec2::new(box_w - 22.0, INNER_H),
                );
                p.rect(ir, 3.0, theme::WELL, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);
                // The count belongs on the first row of the first chain: it
                // says how many times that chain runs, which is a bank's
                // hundreds of channels all running the one drawn.
                let text = if g == 0 && k == 0 && node.inner_count > 1 {
                    format!("{}  x{}", sub.label, node.inner_count)
                } else {
                    sub.label.clone()
                };
                p.text(
                    ir.center(),
                    egui::Align2::CENTER_CENTER,
                    text,
                    FontId::new(10.0, FontFamily::Proportional),
                    theme::LEGEND,
                );
                iy += INNER_H + INNER_GAP;
            }
        }

        // A leaf that carries a stream says where it goes. A sink writes no
        // buffer at all, so labelling one with a rate would invent an output
        // that does not exist.
        let feeds_anything = topo.nodes.iter().any(|other| {
            other.inputs.iter().any(|(s, _)| node.outputs.iter().any(|(o, _)| o == s))
        });
        if !feeds_anything && !node.sink {
            if let Some((slot, spec)) = node.outputs.first() {
                p.text(
                    Pos2::new(r.right() + 8.0, r.center().y),
                    egui::Align2::LEFT_CENTER,
                    format!("out  {}", wire_label(spec, topo.rate_of(*slot))),
                    theme::figure(10.0),
                    theme::TRACE,
                );
            }
        }
    }

    // A click that hit no box clears the selection, which is the only way out
    // of the inspector that does not need a button to be found.
    let on_source = pointer.is_some_and(|q| src.contains(q));
    if resp.clicked() && on_source && edit.drag.is_none() {
        act.selected = if selected == Some(SOURCE) { None } else { Some(SOURCE) };
    } else if resp.clicked() && !rects.iter().any(|r| pointer.is_some_and(|q| r.contains(q))) {
        act.selected = None;
        act.blank = !ghosts.iter().any(|g| pointer.is_some_and(|q| g.r.contains(q)));
    }

    let dropping = pointer.or_else(|| ui.ctx().pointer_latest_pos());
    if let (Some(carried), Some(q)) = (resp.dnd_hover_payload::<Carried>(), dropping) {
        let onto = attach_at(topo, &rects, &ghosts, src, patch, q);
        let target = match onto {
            Attach::Feeds((tag, _)) | Attach::Splice(_, (tag, _)) => edit.drawn.get(&tag),
            Attach::Reads(crate::patch::Source::Stage(tag, _)) => edit.drawn.get(&tag),
            Attach::Reads(crate::patch::Source::Span) => Some(&src),
            Attach::Nothing => None,
        };
        if let Some(r) = target {
            target_ring(&p, *r);
        }
        let r = Rect::from_center_size(q, Vec2::new(box_w, BOX_H));
        p.rect_stroke(r, 3.0, Stroke::new(1.0, theme::READOUT), StrokeKind::Inside);
        p.text(
            r.center(),
            egui::Align2::CENTER_CENTER,
            &carried.0,
            FontId::new(12.0, FontFamily::Proportional),
            theme::READOUT,
        );
        if let Some(carried) = resp.dnd_release_payload::<Carried>() {
            act.dropped = Some((carried.0.clone(), q - rect.min.to_vec2(), onto));
        }
    }
    act
}

fn attach_at(
    topo: &Topology,
    rects: &[Rect],
    ghosts: &[Ghost],
    src: Rect,
    patch: Option<&crate::patch::Patch>,
    q: Pos2,
) -> Attach {
    if let Some(to) = input_at(topo, rects, ghosts, q) {
        return Attach::Feeds(to);
    }
    if let Some(from) = output_at(topo, rects, ghosts, src, q) {
        return Attach::Reads(from);
    }
    match wire_at(topo, rects, src, patch, q) {
        Some((from, to)) => Attach::Splice(from, to),
        None => Attach::Nothing,
    }
}

fn wire_at(
    topo: &Topology,
    rects: &[Rect],
    src: Rect,
    patch: Option<&crate::patch::Patch>,
    q: Pos2,
) -> Option<(crate::patch::Source, (u64, usize))> {
    use crate::patch::Source;
    for (i, node) in topo.nodes.iter().enumerate() {
        let Some(tag) = node.tag else { continue };
        for (k, (slot, _)) in node.inputs.iter().enumerate() {
            let to = port(rects[i], k, node.inputs.len(), Side::In);
            let from = match topo.producer(*slot) {
                Some(prod) => match topo.nodes.iter().position(|x| x.id == prod.id) {
                    Some(j) => {
                        let out = prod.outputs.iter().position(|(s, _)| s == slot).unwrap_or(0);
                        port(rects[j], out, prod.outputs.len(), Side::Out)
                    }
                    None => port(src, 0, 1, Side::Out),
                },
                None => port(src, 0, 1, Side::Out),
            };
            if !near_wire(from, to, q) {
                continue;
            }
            let source = match patch.and_then(|p| p.feeding((tag, k))) {
                Some(s) => s,
                None if crate::patch::builtin::is(tag) => Source::Span,
                None => continue,
            };
            return Some((source, (tag, k)));
        }
    }
    None
}

fn in_port_of(
    topo: &Topology,
    rects: &[Rect],
    ghosts: &[Ghost],
    tag: u64,
    k: usize,
) -> Option<Pos2> {
    if let Some(i) = topo.nodes.iter().position(|n| n.tag == Some(tag)) {
        return Some(port(rects[i], k, topo.nodes[i].inputs.len().max(1), Side::In));
    }
    ghosts.iter().find(|g| g.id == tag).map(|g| port(g.r, k, g.ins.max(1), Side::In))
}

/// How near a port the pointer has to be for the drag to be about that port
/// rather than about the box it sits on.
const PORT_GRAB: f32 = 9.0;

/// Move a stage, draw a wire, or pull one off.
///
/// The whole drawing is one response rather than a widget per box, so what a
/// drag is about has to be decided when it starts and then remembered:
/// hit-testing again each frame would hand the drag to whatever the pointer
/// happened to pass over.
#[allow(clippy::too_many_arguments)]
fn interact(
    resp: &egui::Response,
    press: Option<Pos2>,
    topo: &Topology,
    node_keys: &[u64],
    rects: &[Rect],
    ghosts: &[Ghost],
    src: Rect,
    origin: Pos2,
    edit: &mut Edit,
    act: &mut Interaction,
    patch: Option<&crate::patch::Patch>,
) {
    use crate::patch::Source;

    // Right-click takes wires off a port: an input loses the one wire it can
    // have, an output loses all of them. A stage with an input port hanging
    // is left out of the built graph rather than refused, so this is an edit
    // like any other rather than a way to break the receiver.
    if resp.secondary_clicked() {
        if let Some(q) = resp.interact_pointer_pos() {
            if let Some((tag, port)) = input_at(topo, rects, ghosts, q) {
                act.unlink = Some((tag, port));
            } else if let Some(from) = output_at(topo, rects, ghosts, src, q) {
                act.unlink_out = Some(from);
            }
        }
    }

    // Where the button went down, not where the pointer is now. A drag is
    // only reported once it has moved past egui's threshold, which is further
    // than a port is wide: hit-testing the current position meant every drag
    // that started on a port was read as a drag of the box behind it.
    if resp.drag_started() {
        edit.drag = press.or_else(|| resp.interact_pointer_pos()).map(|q| {
            // Ports first: they sit on the edge of a box, so testing the box
            // first would mean a wire could never be started at all.
            if let Some(from) = output_at(topo, rects, ghosts, src, q) {
                return Drag::Wire { from: Some(from), to: None, at: q };
            }
            // Taking hold of a wire where it lands, which is how a connection
            // is moved rather than deleted and drawn again.
            if let Some((tag, port)) = input_at(topo, rects, ghosts, q) {
                let from = match patch.and_then(|p| p.feeding((tag, port))) {
                    Some(from) => {
                        act.unlink = Some((tag, port));
                        Some(from)
                    }
                    // The receiver's own stages read the span unless they
                    // have been told otherwise, and that wire is on screen
                    // even though no link says so. Grabbing it has to work,
                    // or the one connection everybody tries to edit first is
                    // the one that cannot be.
                    None if crate::patch::builtin::is(tag) => Some(Source::Span),
                    None => None,
                };
                return Drag::Wire { from, to: Some((tag, port)), at: q };
            }
            if let Some(g) = ghosts.iter().find(|g| g.r.contains(q)) {
                return Drag::Node(g.id, g.r.center() - q);
            }
            if src.contains(q) {
                return Drag::Node(crate::patch::builtin::SPAN, src.center() - q);
            }
            if let Some(i) = rects.iter().position(|r| {
                let c =
                    Rect::from_min_max(Pos2::new(r.right() - CORNER, r.bottom() - CORNER), r.max);
                c.contains(q)
            }) {
                if resizable(&topo.nodes[i]) {
                    return Drag::Resize(node_keys[i]);
                }
            }
            match rects.iter().position(|r| r.contains(q)) {
                Some(i) => Drag::Node(node_keys[i], rects[i].center() - q),
                None => Drag::Pan,
            }
        });
    }

    if let (Some(drag), Some(q)) = (edit.drag, resp.interact_pointer_pos()) {
        match drag {
            Drag::Node(k, grab) if resp.dragged() => {
                edit.pos.insert(k, q + grab - origin.to_vec2());
            }
            Drag::Resize(k) if resp.dragged() => {
                // The top left stays put; the centre moves with the size.
                if let Some(r) = edit.drawn.get(&k).copied() {
                    let min = Vec2::new(SCOPE_W, 120.0);
                    let size = (q - r.min).max(min);
                    let centre = r.min + size / 2.0;
                    edit.size.insert(k, size);
                    edit.pos.insert(k, centre - origin.to_vec2());
                }
            }
            Drag::Wire { from, to, .. } if resp.dragged() => {
                edit.drag = Some(Drag::Wire { from, to, at: q })
            }
            Drag::Pan if resp.dragged() => act.pan = resp.drag_delta(),
            _ => {}
        }
        // Where the wire lands decides what it means: on an input, whatever
        // it came from now feeds that input; on an output, that output now
        // feeds whichever input the wire was pulled off. Anywhere else and
        // the wire is abandoned, because an edit that half happens is worse
        // than one that visibly did not.
        if resp.drag_stopped() {
            if let Drag::Wire { from, to, .. } = drag {
                let on_in =
                    input_at(topo, rects, ghosts, q).or_else(|| body_input(topo, rects, ghosts, q));
                let on_out = output_at(topo, rects, ghosts, src, q)
                    .or_else(|| body_output(topo, rects, ghosts, src, q));
                match to {
                    // Pulled off an input: whatever it is dropped on is what
                    // that input reads now. Dropping it on another input
                    // instead moves the wire across, which is what a wire
                    // held by its end looks like it should do.
                    Some(to) => match (on_out, from, on_in) {
                        (Some(from), _, _) => act.link = Some((from, to.0, to.1)),
                        (None, Some(from), Some(landed)) => {
                            act.link = Some((from, landed.0, landed.1))
                        }
                        _ => {}
                    },
                    // Drawn out of an output: it has to land on an input.
                    None => {
                        if let (Some(from), Some(landed)) = (from, on_in) {
                            act.link = Some((from, landed.0, landed.1));
                        }
                    }
                }
            }
        }
    }

    if resp.drag_stopped() {
        edit.drag = None;
    }
}

fn near(p: Pos2, q: Pos2) -> bool {
    p.distance(q) <= PORT_GRAB
}

/// One port, lit if a wire can be dragged from or to it, and larger under the
/// pointer so that what will be grabbed is known before the drag starts.
fn knob(p: &egui::Painter, at: Pos2, live: bool, pointer: Option<Pos2>) {
    let hot = live && pointer.is_some_and(|q| near(at, q));
    let col = if live { theme::READOUT } else { theme::ETCH };
    p.circle_filled(at, if hot { 4.5 } else { 2.5 }, col);
    if hot {
        p.circle_stroke(at, 7.0, Stroke::new(1.0, theme::READOUT));
    }
}

/// The operator's stage and input port under a point, if there is one.
///
/// Only a stage the operator drew: the head of the chain, the spectrum and
/// the listening channels are the receiver's own wiring, and a wire dropped
/// on one of them would describe a graph the patch cannot express.
fn input_at(topo: &Topology, rects: &[Rect], ghosts: &[Ghost], q: Pos2) -> Option<(u64, usize)> {
    for (i, node) in topo.nodes.iter().enumerate() {
        let Some(tag) = node.tag else { continue };
        let n = node.inputs.len().max(1);
        for k in 0..n {
            if near(port(rects[i], k, n, Side::In), q) {
                return Some((tag, k));
            }
        }
    }
    ghosts.iter().find_map(|g| {
        (0..g.ins).find(|k| near(port(g.r, *k, g.ins, Side::In), q)).map(|k| (g.id, k))
    })
}

/// The output port under a point, as something a wire can start at or land
/// on: a stage the operator drew, or the span itself.
fn output_at(
    topo: &Topology,
    rects: &[Rect],
    ghosts: &[Ghost],
    src: Rect,
    q: Pos2,
) -> Option<crate::patch::Source> {
    use crate::patch::Source;
    if near(port(src, 0, 1, Side::Out), q) || src.contains(q) {
        return Some(Source::Span);
    }
    for (i, node) in topo.nodes.iter().enumerate() {
        let Some(tag) = node.tag else { continue };
        if node.sink || crate::patch::builtin::is(tag) {
            continue;
        }
        for k in 0..node.outputs.len() {
            if near(port(rects[i], k, node.outputs.len(), Side::Out), q) {
                return Some(Source::Stage(tag, k));
            }
        }
    }
    ghosts.iter().find_map(|g| {
        (0..g.outs)
            .find(|k| near(port(g.r, *k, g.outs, Side::Out), q))
            .map(|k| Source::Stage(g.id, k))
    })
}

/// The stage whose box a point is inside, as something a wire can read.
fn body_output(
    topo: &Topology,
    rects: &[Rect],
    ghosts: &[Ghost],
    src: Rect,
    q: Pos2,
) -> Option<crate::patch::Source> {
    use crate::patch::Source;
    if src.contains(q) {
        return Some(Source::Span);
    }
    for (i, node) in topo.nodes.iter().enumerate() {
        let Some(tag) = node.tag else { continue };
        if !node.sink && !crate::patch::builtin::is(tag) && rects[i].contains(q) {
            return Some(Source::Stage(tag, 0));
        }
    }
    ghosts.iter().find(|g| g.outs > 0 && g.r.contains(q)).map(|g| Source::Stage(g.id, 0))
}

/// The operator's stage whose box a point is inside, and the input port
/// nearest to where the wire was let go.
fn body_input(topo: &Topology, rects: &[Rect], ghosts: &[Ghost], q: Pos2) -> Option<(u64, usize)> {
    for (i, node) in topo.nodes.iter().enumerate() {
        let Some(tag) = node.tag else { continue };
        if rects[i].contains(q) {
            let n = node.inputs.len().max(1);
            let k = (0..n)
                .min_by(|a, b| {
                    let d = |k: &usize| port(rects[i], *k, n, Side::In).distance(q);
                    d(a).total_cmp(&d(b))
                })
                .unwrap_or(0);
            return Some((tag, k));
        }
    }
    let g = ghosts.iter().find(|g| g.ins > 0 && g.r.contains(q))?;
    let k = (0..g.ins)
        .min_by(|a, b| {
            let d = |k: &usize| port(g.r, *k, g.ins, Side::In).distance(q);
            d(a).total_cmp(&d(b))
        })
        .unwrap_or(0);
    Some((g.id, k))
}

/// Where a wire being drawn starts on screen.
fn wire_start(
    topo: &Topology,
    rects: &[Rect],
    ghosts: &[Ghost],
    src: Rect,
    from: crate::patch::Source,
) -> Pos2 {
    let tag = match from {
        crate::patch::Source::Span => return port(src, 0, 1, Side::Out),
        crate::patch::Source::Stage(tag, _) => tag,
    };
    let k = match from {
        crate::patch::Source::Stage(_, k) => k,
        crate::patch::Source::Span => 0,
    };
    if let Some(i) = topo.nodes.iter().position(|n| n.tag == Some(tag)) {
        let n = &topo.nodes[i];
        return port(rects[i], k, n.outputs.len().max(1), Side::Out);
    }
    ghosts
        .iter()
        .find(|g| g.id == tag)
        .map(|g| port(g.r, k, g.outs.max(1), Side::Out))
        .unwrap_or(src.center_bottom())
}

/// Whether a point is on a wire, near enough to have meant it.
///
/// The curve is sampled rather than solved: a cubic is not worth inverting to
/// answer a question about a pointer that is a few pixels wide.
fn near_wire(from: Pos2, to: Pos2, q: Pos2) -> bool {
    let reach = ((to.x - from.x).abs() * 0.5).clamp(26.0, 90.0)
        + if to.x < from.x { (from.x - to.x) * 0.5 } else { 0.0 };
    let (c1, c2) = (Pos2::new(from.x + reach, from.y), Pos2::new(to.x - reach, to.y));
    (0..=16).any(|i| {
        let t = i as f32 / 16.0;
        let u = 1.0 - t;
        let at = Pos2::new(
            u * u * u * from.x + 3.0 * u * u * t * c1.x + 3.0 * u * t * t * c2.x + t * t * t * to.x,
            u * u * u * from.y + 3.0 * u * u * t * c1.y + 3.0 * u * t * t * c2.y + t * t * t * to.y,
        );
        at.distance(q) <= 6.0
    })
}

/// A ring round whatever a wire would attach to if it were let go now.
fn target_ring(p: &egui::Painter, at: Rect) {
    p.rect_stroke(at.expand(2.0), 4.0, Stroke::new(1.5, theme::READOUT), StrokeKind::Outside);
}

/// The wire that is being drawn but has not landed anywhere yet.
fn loose(p: &egui::Painter, from: Pos2, to: Pos2) {
    let stroke = Stroke::new(1.5, theme::READOUT);
    let reach = ((to.x - from.x).abs() * 0.5).clamp(26.0, 90.0);
    p.add(egui::epaint::CubicBezierShape::from_points_stroke(
        [from, Pos2::new(from.x + reach, from.y), Pos2::new(to.x - reach, to.y), to],
        false,
        Color32::TRANSPARENT,
        stroke,
    ));
    p.circle_filled(to, 3.0, theme::READOUT);
}

/// A scope's readings inside its box, under the label line: the meter, the
/// spectrum auto-ranged to what it is seeing, and the spectrogram.
fn paint_scope(
    ctx: &egui::Context,
    p: &egui::Painter,
    r: Rect,
    id: usize,
    frame: Option<&nodes::ScopeFrame>,
) {
    let mono = theme::figure(9.0);
    let inner = Rect::from_min_max(
        Pos2::new(r.left() + 8.0, r.top() + 40.0),
        Pos2::new(r.right() - 8.0, r.bottom() - 8.0),
    );
    let Some(f) = frame else {
        p.text(
            inner.center(),
            egui::Align2::CENTER_CENTER,
            "nothing on the wire yet",
            mono,
            theme::LEGEND,
        );
        return;
    };
    let db = |v: f32| 20.0 * v.max(1e-6).log10();

    // Level, on the same strip every fader carries, the held peak as a mark.
    let vu =
        Rect::from_min_size(inner.min, Vec2::new(inner.width() - 78.0, egui_bench::meter::VU_H));
    egui_bench::meter::vu(p, vu, f.rms * 2f32.sqrt());
    let x = vu.left() + f.peak_hold.clamp(0.0, 1.0).sqrt() * vu.width();
    p.line_segment(
        [Pos2::new(x, vu.top()), Pos2::new(x, vu.bottom())],
        Stroke::new(1.0, theme::VALUE),
    );
    p.text(
        Pos2::new(inner.right(), vu.center().y),
        egui::Align2::RIGHT_CENTER,
        format!("{:+.0}/{:+.0} dB", db(f.rms), db(f.peak)),
        mono.clone(),
        theme::VALUE,
    );
    if f.spectrum.is_empty() {
        return;
    }

    let (lo_hz, hi_hz) = if f.real {
        (0.0, f.rate / 2.0)
    } else {
        (f.center_hz - f.rate / 2.0, f.center_hz + f.rate / 2.0)
    };
    let fmt = |hz: f64| {
        if hz.abs() >= 1e6 {
            format!("{:.3}M", hz / 1e6)
        } else if hz.abs() >= 1e3 {
            format!("{:.1}k", hz / 1e3)
        } else {
            format!("{hz:.0}")
        }
    };
    let (mut floor, mut top) = (f32::INFINITY, f32::NEG_INFINITY);
    for row in f.history.iter().chain(std::iter::once(&f.spectrum)) {
        for &v in row {
            if v.is_finite() {
                floor = floor.min(v);
                top = top.max(v);
            }
        }
    }
    // Ninety decibels under the loudest thing seen: what a display can
    // show, and enough that silence reads as black rather than as the shape
    // of the noise it is scaled against.
    if !top.is_finite() {
        top = nodes::scope_nodes::FLOOR_DB + 90.0;
    }
    floor = floor.max(top - 90.0);
    if top - floor < 10.0 {
        top = floor + 10.0;
    }

    // Spectrum in the upper part, spectrogram below it.
    let body_top = vu.bottom() + 6.0;
    let spec_h = ((inner.bottom() - body_top) * 0.4).max(30.0);
    let sr = Rect::from_min_max(
        Pos2::new(inner.left(), body_top),
        Pos2::new(inner.right(), body_top + spec_h),
    );
    p.rect_filled(sr, 2.0, theme::WELL);
    let n = f.spectrum.len();
    let pts: Vec<Pos2> = f
        .spectrum
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            let x = sr.left() + (i as f32 + 0.5) / n as f32 * sr.width();
            let t = ((v - floor) / (top - floor)).clamp(0.0, 1.0);
            Pos2::new(x, sr.bottom() - 1.0 - t * (sr.height() - 2.0))
        })
        .collect();
    p.add(egui::Shape::line(pts, Stroke::new(1.0, theme::TRACE)));
    // Range at the top left, one number over the other; the frequency ends
    // in the bottom corners where they belong.
    p.text(
        Pos2::new(sr.left() + 3.0, sr.top() + 2.0),
        egui::Align2::LEFT_TOP,
        format!("{top:.0} dB"),
        mono.clone(),
        theme::LEGEND,
    );
    p.text(
        Pos2::new(sr.left() + 3.0, sr.top() + 13.0),
        egui::Align2::LEFT_TOP,
        format!("{floor:.0}"),
        mono.clone(),
        theme::LEGEND,
    );
    p.text(
        Pos2::new(sr.left() + 3.0, sr.bottom() - 2.0),
        egui::Align2::LEFT_BOTTOM,
        fmt(lo_hz),
        mono.clone(),
        theme::LEGEND,
    );
    p.text(
        Pos2::new(sr.right() - 3.0, sr.bottom() - 2.0),
        egui::Align2::RIGHT_BOTTOM,
        fmt(hi_hz),
        mono.clone(),
        theme::LEGEND,
    );

    if f.history.is_empty() {
        return;
    }
    let fr = Rect::from_min_max(Pos2::new(inner.left(), sr.bottom() + 4.0), inner.max);
    if fr.height() < 8.0 {
        return;
    }
    let rows = f.history.len();
    let cols = n.min(256);
    let step = (n / cols).max(1);
    let mut img = egui::ColorImage::new([cols, rows], vec![Color32::BLACK; cols * rows]);
    for (y, row) in f.history.iter().enumerate() {
        for x in 0..cols {
            let v = row[x * step..((x + 1) * step).min(n)]
                .iter()
                .copied()
                .fold(f32::NEG_INFINITY, f32::max);
            let t = ((v - floor) / (top - floor)).clamp(0.0, 1.0);
            img.pixels[y * cols + x] = crate::waterfall::colormap(t);
        }
    }
    let tex = ctx.load_texture(format!("scope-{id}"), img, egui::TextureOptions::LINEAR);
    p.image(tex.id(), fr, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
}

fn stage(p: &egui::Painter, r: Rect, label: &str, kind: &str, recessed: bool, cost: Option<&Cost>) {
    let fill = if recessed { theme::WELL } else { theme::PANEL };
    p.rect(r, 3.0, fill, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);
    p.text(
        Pos2::new(r.center().x, r.top() + 9.0),
        egui::Align2::CENTER_TOP,
        label,
        FontId::new(12.0, FontFamily::Proportional),
        if kind == "off" || kind == "idle" { theme::LEGEND } else { theme::VALUE },
    );
    let mono = theme::figure(10.0);
    // The kind on the left and the cost on the right of one line, so a box
    // that is costing something says so without growing.
    let cost_text = cost.and_then(cost_label);
    if let (Some(text), Some(c)) = (cost_text, cost) {
        p.text(
            Pos2::new(r.left() + 8.0, r.top() + 26.0),
            egui::Align2::LEFT_TOP,
            kind,
            mono.clone(),
            theme::LEGEND,
        );
        p.text(
            Pos2::new(r.right() - 8.0, r.top() + 26.0),
            egui::Align2::RIGHT_TOP,
            text,
            mono,
            cost_colour(c),
        );
    } else {
        p.text(
            Pos2::new(r.center().x, r.top() + 26.0),
            egui::Align2::CENTER_TOP,
            kind,
            mono,
            theme::LEGEND,
        );
    }
}

/// A node's own breakdown, one line per phase, under its box.
fn phases(p: &egui::Painter, r: Rect, phases: &[(String, Cost)]) -> f32 {
    let mono = theme::figure(9.0);
    let mut y = r.bottom() + INNER_GAP;
    for (name, c) in phases {
        let Some(text) = cost_label(c) else { continue };
        p.text(
            Pos2::new(r.left() + 10.0, y),
            egui::Align2::LEFT_TOP,
            name,
            mono.clone(),
            theme::LEGEND,
        );
        p.text(
            Pos2::new(r.right() - 10.0, y),
            egui::Align2::RIGHT_TOP,
            text,
            mono.clone(),
            cost_colour(c),
        );
        y += PHASE_H;
    }
    y
}

/// Which edge of a box a port sits on.
#[derive(Clone, Copy, PartialEq)]
enum Side {
    In,
    Out,
}

/// Where one port sits: inputs down the left edge, outputs down the right,
/// spread evenly so a stage with several of either shows which is which.
///
/// Signal flows left to right here, the way it does in every flowgraph editor
/// anybody has used. It used to run downwards, which came from this being a
/// diagram to read rather than a graph to edit: a wire arriving at the top of
/// a box says nothing about which end of it is the input.
fn port(r: Rect, i: usize, n: usize, side: Side) -> Pos2 {
    let n = n.max(1);
    let step = r.height() / (n + 1) as f32;
    let x = if side == Side::In { r.left() } else { r.right() };
    Pos2::new(x, r.top() + step * (i + 1) as f32)
}

/// A labelled wire carrying what the link actually contains.
///
/// Curved rather than routed around corners. Once stages can be dragged
/// anywhere, an orthogonal route has no good answer for a wire that runs
/// backwards: it doubles back through the boxes it is trying to avoid. A
/// curve leaving the producer to the right and arriving at the consumer from
/// the left says the same thing about direction and stays readable wherever
/// the two ends are.
fn edge(
    p: &egui::Painter,
    from: Pos2,
    to: Pos2,
    spec: &StreamSpec,
    measured: Option<f32>,
    label: bool,
    lit: bool,
) {
    let col = if lit { theme::READOUT } else { Color32::from_rgb(0x4A, 0x55, 0x60) };
    let stroke = Stroke::new(if lit { 2.0 } else { 1.0 }, col);
    // Enough slack that the curve leaves and arrives horizontally, and more
    // of it when the ends are far apart or the wire runs back up the graph.
    let reach = ((to.x - from.x).abs() * 0.5).clamp(26.0, 90.0)
        + if to.x < from.x { (from.x - to.x) * 0.5 } else { 0.0 };
    let pts = [from, Pos2::new(from.x + reach, from.y), Pos2::new(to.x - reach, to.y), to];
    p.add(egui::epaint::CubicBezierShape::from_points_stroke(
        pts,
        false,
        Color32::TRANSPARENT,
        stroke,
    ));
    for d in [-4.0, 4.0] {
        p.line_segment([Pos2::new(to.x - 6.0, to.y + d), to], stroke);
    }
    if label {
        // Halfway along the wire, which is the middle of the gap between two
        // boxes: printed at either end it landed on top of one of them.
        let mid = Pos2::new(
            (from.x + 3.0 * pts[1].x + 3.0 * pts[2].x + to.x) / 8.0,
            (from.y + 3.0 * pts[1].y + 3.0 * pts[2].y + to.y) / 8.0,
        );
        p.text(
            Pos2::new(mid.x, mid.y - 3.0),
            egui::Align2::CENTER_BOTTOM,
            wire_label(spec, measured),
            theme::figure(10.0),
            theme::LEGEND,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use pipeline::graph::{NodeId, TopoNode};

    #[test]
    fn a_stereo_stream_says_so_in_its_label() {
        let s = StreamSpec::iq(96_000.0, Hz::mhz(95))
            .with_kind(PortKind::Real)
            .with_rate(48_000.0)
            .with_channels(2);
        // The port rate is 96 kHz but each ear runs at 48 kHz. Showing the port
        // rate would read as an audio chain running twice as fast as it is.
        assert_eq!(rate_label(&s).as_deref(), Some("48.0 kS/s x2"));
    }

    #[test]
    fn a_packet_wire_does_not_claim_a_sample_rate() {
        // Packets are events. The bus used to be labelled "packets 0 S/s",
        // which reads as a dead wire while decodes are going down it.
        let s = StreamSpec::iq(0.0, Hz::mhz(1090)).with_kind(PortKind::Packets);
        assert_eq!(rate_label(&s), None);
        assert_eq!(wire_label(&s, None), "packets");
        // Same for a port carrying one stream per transmitter found, where a
        // single rate would be wrong about all but one of them.
        let src = StreamSpec::iq(2_400_000.0, Hz::mhz(433)).with_kind(PortKind::Sources);
        assert_eq!(wire_label(&src, None), "sources");
    }

    #[test]
    fn a_packet_wire_says_what_the_graph_counted_on_it() {
        let s = StreamSpec::iq(0.0, Hz::mhz(1090)).with_kind(PortKind::Packets);
        assert_eq!(wire_label(&s, Some(3.4)), "packets  3.40/s");
        assert_eq!(wire_label(&s, Some(128.0)), "packets  128/s");
        // A bus nothing is using says so, rather than showing a number left
        // over in the smoothing.
        assert_eq!(wire_label(&s, Some(0.0)), "packets  idle");
        assert_eq!(wire_label(&s, Some(0.01)), "packets  idle");
        // A stream with a real sample rate keeps it: that number is exact, and
        // a measured one would only wobble around it.
        let iq = StreamSpec::iq(2_400_000.0, Hz::mhz(433));
        assert_eq!(wire_label(&iq, Some(2_399_888.0)), "iq  2.400 MS/s");
    }

    #[test]
    fn rates_are_shown_in_a_unit_that_suits_them() {
        let at = |r: f64| rate_label(&StreamSpec::iq(r, Hz::mhz(95))).unwrap_or_default();
        assert_eq!(at(2_304_000.0), "2.304 MS/s");
        assert_eq!(at(48_000.0), "48.0 kS/s");
        // A symbol rate is not helped by being rounded to 1.2 kS/s, which is
        // why the threshold is 10 kS/s rather than 1.
        assert_eq!(at(1187.5), "1188 S/s");
        assert_eq!(at(4_750.0), "4750 S/s");
    }

    fn node(id: usize, label: &str, ins: &[usize], out: usize) -> TopoNode {
        let s = StreamSpec::iq(2_400_000.0, Hz::mhz(433));
        TopoNode {
            id: NodeId(id),
            tag: None,
            label: label.into(),
            kind: label.into(),
            latency: 0,
            inputs: ins.iter().map(|i| (*i, s)).collect(),
            outputs: vec![(out, s)],
            inner: Vec::new(),
            inner_count: 1,
            sink: false,
            params: Vec::new(),
            cost: Default::default(),
            phases: Vec::new(),
            readings: Vec::new(),
            off: false,
            idle: false,
        }
    }

    /// Slot 0 is the graph input; a node's output slot is its own.
    fn branchy() -> Topology {
        Topology {
            input: StreamSpec::iq(2_400_000.0, Hz::mhz(433)),
            input_off: false,
            nodes: vec![
                node(0, "DC block", &[0], 1),
                node(1, "Spectrum", &[1], 2),
                node(2, "31 kHz bank", &[1], 3),
                node(3, "Mixer", &[1], 4),
                node(4, "Demod", &[4], 5),
            ],
            output_slot: 5,
            rates: Vec::new(),
        }
    }

    #[test]
    fn branches_off_one_node_share_a_row() {
        // The receiver's whole shape: the spectrum, the banks and every
        // channel read the same samples, so drawing them in a column would
        // say one feeds the next, which is not true of any of them.
        let p = layout(&branchy());
        assert_eq!(p[0].depth, 0, "the DC block reads the source");
        assert_eq!((p[1].depth, p[2].depth, p[3].depth), (1, 1, 1));
        assert_ne!(p[1].col, p[2].col, "branches do not share a column");
        assert_ne!(p[2].col, p[3].col);
    }

    #[test]
    fn a_stage_sits_below_what_feeds_it_in_the_same_trunk() {
        let p = layout(&branchy());
        // The demodulator is fed by the mixer, so it belongs a row further
        // down and directly underneath it.
        assert!(p[4].depth > p[3].depth);
        assert_eq!(p[4].col, p[3].col, "a chain should read as a straight line");
    }

    /// A view driven by pointer events, the way the operator drives it.
    struct Harness {
        ctx: egui::Context,
        topo: Topology,
        edit: Edit,
        patch: crate::patch::Patch,
        waiting: Vec<crate::chain::Waiting>,
        at: Pos2,
    }

    impl Harness {
        fn new(topo: Topology, patch: crate::patch::Patch) -> Self {
            let ctx = egui::Context::default();
            crate::ui::install(&ctx);
            Self {
                ctx,
                topo,
                edit: Edit { manual: true, ..Default::default() },
                patch,
                waiting: Vec::new(),
                at: Pos2::ZERO,
            }
        }

        /// One frame, with the pointer where it was left and any events.
        fn frame(&mut self, events: Vec<egui::Event>) -> Interaction {
            let input = egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(900.0, 700.0))),
                events,
                ..Default::default()
            };
            let mut act = Interaction::default();
            let topo = self.topo.clone();
            let patch = self.patch.clone();
            let waiting = self.waiting.clone();
            let edit = &mut self.edit;
            let out = &mut act;
            let _ = self.ctx.run_ui(input, |ui| {
                *out =
                    draw(ui, &topo, 0.0, None, edit, Some(&patch), None, None, &[], &[], &waiting);
            });
            act
        }

        fn press(&mut self, at: Pos2) -> Interaction {
            self.at = at;
            self.frame(vec![
                egui::Event::PointerMoved(at),
                egui::Event::PointerButton {
                    pos: at,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: Default::default(),
                },
            ])
        }

        fn move_to(&mut self, at: Pos2) -> Interaction {
            self.at = at;
            self.frame(vec![egui::Event::PointerMoved(at)])
        }

        fn release(&mut self, at: Pos2) -> Interaction {
            self.at = at;
            self.frame(vec![
                egui::Event::PointerMoved(at),
                egui::Event::PointerButton {
                    pos: at,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: Default::default(),
                },
            ])
        }

        /// Where a box's ports are on screen, by the key it is drawn under.
        fn out_port(&self, k: u64) -> Pos2 {
            port(self.edit.drawn[&k], 0, 1, Side::Out)
        }

        fn in_port(&self, k: u64) -> Pos2 {
            port(self.edit.drawn[&k], 0, 1, Side::In)
        }

        fn source_port(&self) -> Pos2 {
            port(self.edit.drawn_src, 0, 1, Side::Out)
        }

        fn drop_carried(&mut self, kind: &str, at: Pos2) -> Interaction {
            self.move_to(at);
            egui::DragAndDrop::set_payload(&self.ctx, Carried(kind.into()));
            self.move_to(at);
            self.release(at)
        }
    }

    /// A topology with stages the operator drew, so there is something with
    /// live ports to aim at. The second stands in for a stage further down,
    /// such as the spectrum: everything in the graph is a patch stage now.
    fn with_patch_stage() -> (Topology, crate::patch::Patch, u64) {
        let (topo, patch, id, _) = with_two_stages();
        (topo, patch, id)
    }

    fn with_two_stages() -> (Topology, crate::patch::Patch, u64, u64) {
        let mut patch = crate::patch::Patch::default();
        let id = patch.add("envelope");
        let sink = patch.add("spectrum");
        let mut topo = branchy();
        topo.nodes[1].tag = Some(id);
        topo.nodes[2].tag = Some(sink);
        (topo, patch, id, sink)
    }

    #[test]
    fn a_click_on_the_source_selects_it_so_receiving_can_be_switched_off() {
        let (topo, patch, _) = with_patch_stage();
        let mut h = Harness::new(topo, patch);
        h.frame(vec![]);
        let src = h.edit.drawn_src.center();
        h.press(src);
        let act = h.release(src);
        assert_eq!(act.selected, Some(SOURCE));
    }

    #[test]
    fn a_wire_dragged_from_the_source_onto_a_stage_connects_it() {
        // The whole point of manual mode. Tested through pointer events
        // rather than by calling the handler, because everything that has
        // gone wrong here was in the gesture rather than in the patch: a
        // drag the scroll area swallowed, a port nothing could hit.
        let (topo, patch, id) = with_patch_stage();
        let mut h = Harness::new(topo, patch);
        h.frame(vec![]);
        let src_port = h.source_port();
        let target = h.in_port(id);
        h.press(src_port);
        h.move_to(src_port + Vec2::new(0.0, 20.0));
        assert!(h.edit.drag.is_some(), "a drag from a port has to start");
        h.move_to(target);
        let act = h.release(target);
        assert_eq!(
            act.link.map(|(_, to, port)| (to, port)),
            Some((id, 0)),
            "the wire should land on the stage's input"
        );
    }

    #[test]
    fn dragging_empty_canvas_moves_the_view() {
        // In manual mode the pane takes every drag so a stage can be moved,
        // which left nothing to pan the graph with: on a chain wider than the
        // window, the stages off the right edge could not be reached at all.
        let (topo, patch, id) = with_patch_stage();
        let mut h = Harness::new(topo, patch);
        h.frame(vec![]);
        // Somewhere no box is: below everything the layout drew.
        let empty = Pos2::new(
            h.edit.drawn_src.left() + 4.0,
            h.edit.drawn.values().fold(0.0f32, |a, r| a.max(r.bottom())) + 60.0,
        );
        h.press(empty);
        let act = h.move_to(empty + Vec2::new(-40.0, -25.0));
        assert!(act.pan != Vec2::ZERO, "an empty-canvas drag has to pan the view");
        // And it must not have moved a stage while doing it.
        let before = h.edit.pos.get(&id).copied();
        h.move_to(empty + Vec2::new(-80.0, -50.0));
        assert_eq!(h.edit.pos.get(&id).copied(), before, "panning moved a stage");
    }

    #[test]
    fn adding_a_stage_does_not_move_the_stages_already_there() {
        // Every patch edit rebuilds the receiver and renumbers its nodes. A
        // view that keyed positions by that number redrew the whole automatic
        // chain in a new place each time a stage was added, which is the
        // single thing that made this feel broken.
        let (topo, patch, _) = with_patch_stage();
        let mut h = Harness::new(topo, patch);
        h.frame(vec![]);
        let before: Vec<(u64, Pos2)> = h.edit.drawn.iter().map(|(k, r)| (*k, r.center())).collect();

        // The same graph with another stage in it, as a rebuild would hand
        // it over: one more node, and every id after it shifted.
        let extra = h.patch.add("mixer");
        let mut grown = h.topo.clone();
        let mut node = grown.nodes[0].clone();
        node.id = pipeline::graph::NodeId(99);
        node.tag = Some(extra);
        node.label = "Mixer".into();
        grown.nodes.insert(0, node);
        for (i, n) in grown.nodes.iter_mut().enumerate() {
            if n.tag.is_none() {
                n.id = pipeline::graph::NodeId(i);
            }
        }
        h.topo = grown;
        h.frame(vec![]);
        for (k, was) in before {
            let now = h.edit.drawn.get(&k).map(|r| r.center());
            assert_eq!(now, Some(was), "a stage that was already there has moved");
        }
    }

    #[test]
    fn no_two_boxes_land_on_the_same_position() {
        // The receiver's own stages are keyed by position and the operator's
        // by patch id. When those two schemes met in the middle, the DC block
        // was drawn underneath the spectrum and every wire in the graph
        // appeared to converge on one box.
        let (topo, patch, _, _) = with_two_stages();
        let mut h = Harness::new(topo, patch);
        h.frame(vec![]);
        let mut seen: Vec<Pos2> = Vec::new();
        for r in h.edit.drawn.values() {
            assert!(
                !seen.iter().any(|p| p.distance(r.center()) < 1.0),
                "two stages are drawn in the same place"
            );
            seen.push(r.center());
        }
        // The span's own box counts: it is dragged and wired like any other.
        assert_eq!(seen.len(), h.topo.nodes.len() + 1);
    }

    #[test]
    fn a_wire_is_hit_along_its_length_and_not_only_at_its_ends() {
        // Reaching for the line you can see is what anybody tries first, and
        // a port is a few pixels across. The curve is sampled rather than
        // solved, so what matters is that the sampling is fine enough to
        // catch a pointer anywhere along a long wire.
        let from = Pos2::new(100.0, 100.0);
        let to = Pos2::new(400.0, 260.0);
        assert!(near_wire(from, to, from), "at the producer");
        assert!(near_wire(from, to, to), "at the consumer");
        assert!(
            near_wire(from, to, Pos2::new(250.0, 180.0)),
            "and halfway along, which is where the line is easiest to hit"
        );
        assert!(!near_wire(from, to, Pos2::new(250.0, 40.0)), "well clear of it");
    }

    #[test]
    fn a_wire_comes_away_in_the_hand_when_grabbed_at_its_input() {
        // Reaching for an existing connection is the first thing anyone does.
        // The wire has to leave the port it landed on and follow the pointer
        // from its own source, or moving a connection means deleting it and
        // drawing it again from memory.
        use crate::patch::Source;
        let (topo, mut patch, _, sink) = with_two_stages();
        patch.connect(Source::Span, (sink, 0));
        let mut h = Harness::new(topo, patch);
        h.frame(vec![]);
        let at = h.in_port(sink);
        h.press(at);
        let act = h.move_to(at + Vec2::new(0.0, 25.0));
        assert!(
            matches!(
                h.edit.drag,
                Some(Drag::Wire { from: Some(Source::Span), to: Some((_, 0)), .. })
            ),
            "the wire should be in hand, still attached to what it came from"
        );
        let _ = act;
    }

    #[test]
    fn the_spectrums_wire_can_be_pulled_onto_a_stages_output() {
        // Putting a stage in front of the spectrum, reached for from the
        // spectrum's end: drag its wire away and drop it on what should feed
        // it now. The other direction works too, and which one somebody
        // reaches for is not something a view gets to decide.
        use crate::patch::Source;
        let (topo, patch, id, sink) = with_two_stages();
        let mut h = Harness::new(topo, patch);
        h.frame(vec![]);
        let at = h.in_port(sink);
        let onto = h.out_port(id);
        h.press(at);
        h.move_to(at + Vec2::new(0.0, -20.0));
        let act = h.release(onto);
        assert_eq!(
            act.link,
            Some((Source::Stage(id, 0), sink, 0)),
            "the spectrum should end up reading the stage"
        );
    }

    #[test]
    fn a_wire_lands_on_the_spectrum_when_dropped_on_its_box() {
        // Ports are a few pixels across. Dropping on the stage means the
        // stage, or the gesture is a test of aim rather than of intent.
        let (topo, patch, id, sink) = with_two_stages();
        let mut h = Harness::new(topo, patch);
        h.frame(vec![]);
        let from = h.out_port(id);
        let onto = h.edit.drawn[&sink].center();
        h.press(from);
        h.move_to(from + Vec2::new(0.0, 20.0));
        let act = h.release(onto);
        assert_eq!(act.link, Some((crate::patch::Source::Stage(id, 0), sink, 0)),);
    }

    #[test]
    fn manual_mode_pins_every_stage_where_the_layout_had_it() {
        // Otherwise moving one box lets the rest reflow into the space it
        // left, and the graph appears to rearrange itself because a stage was
        // touched.
        let topo = branchy();
        let mut edit = Edit { manual: true, ..Default::default() };
        let ctx = egui::Context::default();
        crate::ui::install(&ctx);
        let frame = |edit: &mut Edit| {
            let _ = ctx.run_ui(Default::default(), |ui| {
                draw(ui, &topo, 0.0, None, edit, None, None, None, &[], &[], &[]);
            });
        };
        frame(&mut edit);
        assert_eq!(edit.pos.len(), topo.nodes.len() + 1, "every stage and the span itself");
        assert!(edit.moved());

        // A stage moved by hand stays put across the next frame, which is
        // what makes the arrangement worth anything: a parameter change
        // rebuilds the graph and redraws this several times a second.
        let put = Pos2::new(11.0, 500.0);
        let first = keys(&topo)[0];
        edit.pos.insert(first, put);
        frame(&mut edit);
        assert_eq!(edit.pos[&first], put);

        edit.arrange();
        assert!(!edit.moved(), "arranging hands the layout back to the graph");
    }

    #[test]
    fn a_composite_makes_room_for_what_it_runs() {
        let mut t = branchy();
        let inner = Topology {
            input: t.input,
            input_off: false,
            nodes: vec![node(0, "Envelope", &[0], 1), node(1, "OOK pulses", &[1], 2)],
            output_slot: 2,
            rates: Vec::new(),
        };
        t.nodes[2].inner = vec![inner];
        t.nodes[2].inner_count = 74;
        let places = layout(&t);
        let lane = places[2].col;
        let with = lane_height(&t, &places, lane);
        t.nodes[2].inner = Vec::new();
        let without = lane_height(&t, &places, lane);
        assert!(with > without, "a bank's channel chain has to fit somewhere");
    }

    #[test]
    fn a_stage_dropped_from_the_palette_lands_where_it_was_let_go() {
        let (topo, patch, _) = with_patch_stage();
        let mut h = Harness::new(topo, patch);
        h.frame(vec![]);
        let empty = Pos2::new(
            h.edit.drawn_src.center().x,
            h.edit.drawn.values().fold(0.0f32, |a, r| a.max(r.bottom())) + 60.0,
        );
        let act = h.drop_carried("mixer", empty);
        assert_eq!(
            act.dropped,
            Some(("mixer".into(), empty - h.edit.origin.to_vec2(), Attach::Nothing))
        );
    }

    #[test]
    fn a_stage_dropped_on_an_input_feeds_it_and_on_an_output_reads_it() {
        use crate::patch::Source;
        let (topo, patch, id) = with_patch_stage();
        let mut h = Harness::new(topo, patch);
        h.frame(vec![]);
        let at = h.in_port(id);
        let act = h.drop_carried("mixer", at);
        assert_eq!(act.dropped.map(|d| d.2), Some(Attach::Feeds((id, 0))));
        let at = h.out_port(id);
        let act = h.drop_carried("mixer", at);
        assert_eq!(act.dropped.map(|d| d.2), Some(Attach::Reads(Source::Stage(id, 0))));
        let at = h.source_port();
        let act = h.drop_carried("mixer", at);
        assert_eq!(act.dropped.map(|d| d.2), Some(Attach::Reads(Source::Span)));
    }

    #[test]
    fn a_stage_dropped_on_a_wire_is_spliced_into_it() {
        use crate::patch::Source;
        let (topo, mut patch, _, sink) = with_two_stages();
        patch.connect(Source::Span, (sink, 0));
        let mut h = Harness::new(topo, patch);
        h.frame(vec![]);
        let from = h.out_port(keys(&h.topo)[0]);
        let to = h.in_port(sink);
        let middle = from + (to - from) / 2.0;
        let act = h.drop_carried("mixer", middle);
        assert_eq!(act.dropped.map(|d| d.2), Some(Attach::Splice(Source::Span, (sink, 0))));
    }

    #[test]
    fn a_waiting_stage_offers_the_ports_the_receiver_built_it_with() {
        use crate::patch::Source;
        let (topo, mut patch, _) = with_patch_stage();
        let rds = patch.add("rds_tx");
        let feed = patch.add("feed");
        let mut h = Harness::new(topo, patch);
        h.waiting = vec![
            crate::chain::Waiting { id: rds, inputs: 2, outputs: 1, params: Vec::new() },
            crate::chain::Waiting { id: feed, inputs: 0, outputs: 1, params: Vec::new() },
        ];
        h.frame(vec![]);
        let second = port(h.edit.drawn[&rds], 1, 2, Side::In);
        let src = h.source_port();
        h.press(src);
        h.move_to(src + Vec2::new(0.0, 20.0));
        h.move_to(second);
        let act = h.release(second);
        assert_eq!(act.link, Some((Source::Span, rds, 1)), "the second input of two");

        let r = h.edit.drawn[&feed];
        let act = h.drop_carried("mixer", port(r, 0, 1, Side::In));
        assert_eq!(
            act.dropped.map(|d| d.2),
            Some(Attach::Nothing),
            "a stage with no inputs has no input port to land on"
        );
    }

    #[test]
    fn stages_waiting_to_be_wired_are_placed_in_view_and_clear_of_each_other() {
        let (topo, mut patch, _) = with_patch_stage();
        let added: Vec<u64> = (0..3).map(|_| patch.add("mixer")).collect();
        let mut h = Harness::new(topo, patch);
        h.frame(vec![]);
        h.frame(vec![]);
        for id in &added {
            let r = h.edit.drawn[id];
            assert!(h.edit.view.contains(r.center()), "stage {id} at {r:?} is out of view");
            let overlaps = h.edit.drawn.iter().filter(|(k, o)| *k != id && o.intersects(r)).count();
            assert_eq!(overlaps, 0, "stage {id} is drawn on top of another");
        }
    }

    #[test]
    fn locking_keeps_where_the_operators_stages_were_put() {
        let mut edit = Edit::default();
        let mine = 7;
        let receivers = crate::patch::builtin::SPAN;
        edit.pos.insert(mine, Pos2::new(40.0, 50.0));
        edit.pos.insert(receivers, Pos2::new(10.0, 10.0));
        edit.arrange_keeping(|k| k == mine);
        assert_eq!(edit.pos.len(), 1);
        assert_eq!(edit.pos[&mine], Pos2::new(40.0, 50.0));
    }
}
