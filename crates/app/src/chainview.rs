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

use crate::theme;
use egui::{Color32, FontFamily, FontId, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};
use pipeline::graph::Topology;
use pipeline::port::{PortKind, StreamSpec};

const BOX_W: f32 = 190.0;
/// Narrower than this and a label stops being readable, so the view scrolls
/// sideways instead of shrinking further.
const MIN_BOX_W: f32 = 118.0;
const BOX_H: f32 = 46.0;
const GAP: f32 = 34.0;
const COL_GAP: f32 = 16.0;
/// A composite node's inner chain, drawn smaller beneath it.
const INNER_H: f32 = 26.0;
const INNER_GAP: f32 = 8.0;

fn kind_label(k: PortKind) -> &'static str {
    match k {
        PortKind::Iq => "iq",
        PortKind::Real => "real",
        PortKind::Bytes => "bytes",
        PortKind::Pulses => "pulses",
        PortKind::Soft => "soft",
        PortKind::Frames => "frames",
        PortKind::Packets => "packets",
    }
}

/// Rate as a human reads it, in the unit that suits the magnitude.
fn rate_label(s: &StreamSpec) -> String {
    let r = s.frame_rate();
    let base = if r >= 1e6 {
        format!("{:.3} MS/s", r / 1e6)
    } else if r >= 10e3 {
        format!("{:.1} kS/s", r / 1e3)
    } else {
        format!("{r:.0} S/s")
    };
    if s.channels > 1 {
        format!("{base} x{}", s.channels)
    } else {
        base
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

/// How tall a row is, allowing for any composite node's inner chain.
fn row_height(topo: &Topology, places: &[Place], depth: usize) -> f32 {
    let inner = topo
        .nodes
        .iter()
        .zip(places)
        .filter(|(_, p)| p.depth == depth)
        .map(|(n, _)| n.inner.as_ref().map(|t| t.nodes.len()).unwrap_or(0))
        .max()
        .unwrap_or(0);
    let extra = if inner == 0 {
        0.0
    } else {
        INNER_GAP + inner as f32 * (INNER_H + INNER_GAP)
    };
    BOX_H + GAP + extra
}

/// What the operator did to the chain view this frame.
#[derive(Default)]
pub struct Interaction {
    /// The node now selected, or `None` if the click was on empty space.
    pub selected: Option<usize>,
    /// A parameter that was changed: node id, name, new value.
    pub changed: Option<(usize, String, pipeline::param::ParamValue)>,
}

/// The selected node's settings, as controls.
///
/// Every stage already describes its own knobs as data, so this renders a
/// decoder it has never heard of. That is the point of the parameter
/// description existing at all: without it, each stage would need its own
/// panel written by hand and the ones nobody wrote would be unreachable.
pub fn inspector(
    ui: &mut egui::Ui,
    topo: &Topology,
    selected: usize,
) -> Option<(usize, String, pipeline::param::ParamValue)> {
    use pipeline::param::{ParamRange, ParamValue};
    let node = topo.nodes.iter().find(|n| n.id.0 == selected)?;
    let mut out = None;

    ui.label(theme::legend(&node.label));
    ui.label(
        egui::RichText::new(&node.kind)
            .font(FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into())))
            .color(theme::LEGEND),
    );
    ui.add_space(6.0);
    for (slot, spec) in &node.inputs {
        ui.label(
            egui::RichText::new(format!("in  {}  {}", kind_label(spec.kind), rate_label(spec)))
                .font(FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into())))
                .color(theme::LEGEND),
        );
        let _ = slot;
    }
    if !node.sink {
        for (_, spec) in &node.outputs {
            ui.label(
                egui::RichText::new(format!(
                    "out {}  {}",
                    kind_label(spec.kind),
                    rate_label(spec)
                ))
                .font(FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into())))
                .color(theme::TRACE),
            );
        }
    }
    ui.add_space(10.0);
    ui.separator();
    ui.add_space(6.0);

    if node.params.is_empty() {
        ui.label(
            egui::RichText::new("This stage has nothing to set.")
                .size(11.0)
                .color(theme::LEGEND),
        );
        return None;
    }

    for prm in &node.params {
        let name = if prm.label.is_empty() { prm.name.clone() } else { prm.label.clone() };
        ui.label(theme::legend(&name));
        match (&prm.value, &prm.range) {
            (ParamValue::Float(v), ParamRange::Float { range, log }) => {
                let mut x = *v;
                let mut w = egui::Slider::new(&mut x, range.clone()).suffix(unit(prm));
                if *log {
                    w = w.logarithmic(true);
                }
                if ui.add(w).changed() {
                    out = Some((node.id.0, prm.name.clone(), ParamValue::Float(x)));
                }
            }
            (ParamValue::Int(v), ParamRange::Int { range }) => {
                let mut x = *v;
                if ui.add(egui::Slider::new(&mut x, range.clone()).suffix(unit(prm))).changed() {
                    out = Some((node.id.0, prm.name.clone(), ParamValue::Int(x)));
                }
            }
            (ParamValue::Bool(v), _) => {
                let mut x = *v;
                if ui.checkbox(&mut x, "").changed() {
                    out = Some((node.id.0, prm.name.clone(), ParamValue::Bool(x)));
                }
            }
            (ParamValue::Choice(i), ParamRange::Choices(choices)) => {
                let mut pick = *i;
                egui::ComboBox::from_id_salt((node.id.0, &prm.name))
                    .selected_text(choices.get(pick).cloned().unwrap_or_default())
                    .width(ui.available_width())
                    .show_ui(ui, |ui| {
                        for (k, c) in choices.iter().enumerate() {
                            ui.selectable_value(&mut pick, k, c);
                        }
                    });
                if pick != *i {
                    out = Some((node.id.0, prm.name.clone(), ParamValue::Choice(pick)));
                }
            }
            (ParamValue::Text(s), _) => {
                let mut t = s.clone();
                let r = ui.add(egui::TextEdit::singleline(&mut t).desired_width(f32::INFINITY));
                if r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    out = Some((node.id.0, prm.name.clone(), ParamValue::Text(t)));
                }
            }
            // A value whose range says something else about it: show the
            // number rather than a control that would write the wrong type.
            (v, _) => {
                ui.label(
                    egui::RichText::new(format!("{v:?}"))
                        .size(11.0)
                        .color(theme::VALUE),
                );
            }
        }
        if prm.affects_rate {
            ui.label(
                egui::RichText::new("changing this rebuilds the chain")
                    .size(10.0)
                    .color(theme::LEGEND),
            );
        }
        ui.add_space(8.0);
    }
    out
}

fn unit(p: &pipeline::param::Param) -> String {
    if p.unit.is_empty() { String::new() } else { format!(" {}", p.unit) }
}

pub fn draw(
    ui: &mut egui::Ui,
    topo: &Topology,
    latency_ms: f64,
    selected: Option<usize>,
) -> Interaction {
    let places = layout(topo);
    let rows = places.iter().map(|p| p.depth + 1).max().unwrap_or(0);
    let widest = places.iter().map(|p| p.col + 1).max().unwrap_or(1).max(1);

    let heights: Vec<f32> = (0..rows).map(|d| row_height(topo, &places, d)).collect();
    let height = BOX_H + GAP + heights.iter().sum::<f32>() + 24.0;
    // Boxes shrink to fit a branchy graph, down to the point where the labels
    // stop being readable; past that the view scrolls sideways.
    let avail = ui.available_width();
    let box_w = ((avail - 24.0 - (widest - 1) as f32 * COL_GAP) / widest as f32)
        .clamp(MIN_BOX_W, BOX_W);
    let width = (widest as f32 * (box_w + COL_GAP) + 24.0).max(avail);

    let (rect, resp) = ui.allocate_exact_size(
        Vec2::new(width, height.max(ui.available_height())),
        Sense::click(),
    );
    let p = ui.painter_at(rect);
    let cx = rect.center().x;
    let mut act = Interaction { selected, changed: None };
    let pointer = resp.hover_pos();

    p.text(
        Pos2::new(rect.left() + 12.0, rect.top() + 4.0),
        egui::Align2::LEFT_TOP,
        format!("{} stages   {latency_ms:.1} ms through the chain", topo.nodes.len()),
        FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into())),
        theme::LEGEND,
    );

    // The source, then a row per depth.
    let top = rect.top() + 16.0;
    let widest_span = widest as f32 * box_w + (widest.saturating_sub(1)) as f32 * COL_GAP;
    let src_x = cx - widest_span / 2.0 + box_w / 2.0;
    let src = Rect::from_center_size(Pos2::new(src_x, top + BOX_H / 2.0), Vec2::new(box_w, BOX_H));
    stage(&p, src, "Source", "device", true);

    let mut row_top = vec![0.0f32; rows];
    let mut y = top + BOX_H + GAP;
    for (d, h) in heights.iter().enumerate() {
        row_top[d] = y;
        y += h;
    }

    // Places first, then edges, then the boxes on top of them. Drawn the
    // other way round, an edge that has to cross a column is painted over the
    // stage it crosses, and a bank's inner chain ends up with a wire through
    // the middle of it.
    let span = widest as f32 * box_w + (widest.saturating_sub(1)) as f32 * COL_GAP;
    let mut rects: Vec<Rect> = Vec::with_capacity(topo.nodes.len());
    for (_, pl) in topo.nodes.iter().zip(&places) {
        let x = cx - span / 2.0 + pl.col as f32 * (box_w + COL_GAP) + box_w / 2.0;
        rects.push(Rect::from_center_size(
            Pos2::new(x, row_top[pl.depth] + BOX_H / 2.0),
            Vec2::new(box_w, BOX_H),
        ));
    }

    for (i, node) in topo.nodes.iter().enumerate() {
        let to = rects[i];
        for (slot, spec) in &node.inputs {
            let from = match topo.producer(*slot) {
                Some(prod) => topo
                    .nodes
                    .iter()
                    .position(|x| x.id == prod.id)
                    .map(|j| rects[j])
                    .unwrap_or(src),
                // No producer means it reads the graph input directly.
                None => src,
            };
            // Labelled once per node, and not at all on a node that gathers
            // many inputs: a bus with six wires into it would stack six
            // identical labels on the same three pixels.
            let label = node.inputs.len() <= 2 && slot == &node.inputs[0].0;
            edge(&p, from.center_bottom(), to.center_top(), spec, label);
        }
    }

    for (i, node) in topo.nodes.iter().enumerate() {
        let r = rects[i];
        let x = r.center().x;
        let hot = pointer.is_some_and(|q| r.contains(q));
        let on = selected == Some(node.id.0);
        stage(&p, r, &node.label, &node.kind, false);
        if on || hot {
            p.rect_stroke(
                r.expand(1.0),
                4.0,
                Stroke::new(if on { 1.5 } else { 1.0 }, theme::READOUT),
                StrokeKind::Outside,
            );
        }
        if hot && resp.clicked() {
            // Clicking the selected stage again closes the inspector, so the
            // panel is not a thing you have to hunt for a way out of.
            act.selected = if on { None } else { Some(node.id.0) };
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

        // A composite draws what it runs inside itself, so a bank is not an
        // opaque box with several hundred decoders hidden in it.
        if let Some(inner) = &node.inner {
            let mut iy = r.bottom() + INNER_GAP;
            for (k, sub) in inner.nodes.iter().enumerate() {
                let ir = Rect::from_center_size(
                    Pos2::new(x, iy + INNER_H / 2.0),
                    Vec2::new(box_w - 22.0, INNER_H),
                );
                p.rect(ir, 3.0, theme::WELL, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);
                let text = if k == 0 && node.inner_count > 1 {
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
            if let Some((_, spec)) = node.outputs.first() {
                let below = r.bottom()
                    + node
                        .inner
                        .as_ref()
                        .map(|t| INNER_GAP + t.nodes.len() as f32 * (INNER_H + INNER_GAP))
                        .unwrap_or(0.0);
                p.text(
                    Pos2::new(x, below + 6.0),
                    egui::Align2::CENTER_TOP,
                    format!("out  {}  {}", kind_label(spec.kind), rate_label(spec)),
                    FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into())),
                    theme::TRACE,
                );
            }
        }
    }

    // A click that hit no box clears the selection, which is the only way out
    // of the inspector that does not need a button to be found.
    if resp.clicked() && !rects.iter().any(|r| pointer.is_some_and(|q| r.contains(q))) {
        act.selected = None;
    }
    act
}

fn stage(p: &egui::Painter, r: Rect, label: &str, kind: &str, source: bool) {
    let fill = if source { theme::WELL } else { theme::PANEL };
    p.rect(r, 3.0, fill, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);
    p.text(
        Pos2::new(r.center().x, r.top() + 9.0),
        egui::Align2::CENTER_TOP,
        label,
        FontId::new(12.0, FontFamily::Proportional),
        theme::VALUE,
    );
    p.text(
        Pos2::new(r.center().x, r.top() + 26.0),
        egui::Align2::CENTER_TOP,
        kind,
        FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into())),
        theme::LEGEND,
    );
}

/// A labelled arrow carrying what the link actually contains.
fn edge(p: &egui::Painter, from: Pos2, to: Pos2, spec: &StreamSpec, label: bool) {
    let col = Color32::from_rgb(0x4A, 0x55, 0x60);
    // Down out of the producer, across, then down into the consumer, so a
    // branch reads as a branch rather than a diagonal crossing other boxes.
    let mid = (from.y + to.y) / 2.0;
    let a = Pos2::new(from.x, mid);
    let b = Pos2::new(to.x, mid);
    p.line_segment([from, a], Stroke::new(1.0, col));
    if (from.x - to.x).abs() > 0.5 {
        p.line_segment([a, b], Stroke::new(1.0, col));
    }
    p.line_segment([b, to], Stroke::new(1.0, col));
    for d in [-4.0, 4.0] {
        p.line_segment([Pos2::new(to.x + d, to.y - 6.0), to], Stroke::new(1.0, col));
    }
    if label {
        // Above the stage it feeds and centred on it, so the label stays
        // inside that stage's own column. Off to the right it drifted over
        // whatever the next column was drawing.
        p.text(
            Pos2::new(to.x, to.y - 7.0),
            egui::Align2::CENTER_BOTTOM,
            format!("{}  {}", kind_label(spec.kind), rate_label(spec)),
            FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into())),
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
        assert_eq!(rate_label(&s), "48.0 kS/s x2");
    }

    #[test]
    fn rates_are_shown_in_a_unit_that_suits_them() {
        let at = |r: f64| rate_label(&StreamSpec::iq(r, Hz::mhz(95)));
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
            label: label.into(),
            kind: label.into(),
            latency: 0,
            inputs: ins.iter().map(|i| (*i, s)).collect(),
            outputs: vec![(out, s)],
            inner: None,
            inner_count: 1,
            sink: false,
            params: Vec::new(),
        }
    }

    /// Slot 0 is the graph input; a node's output slot is its own.
    fn branchy() -> Topology {
        Topology {
            input: StreamSpec::iq(2_400_000.0, Hz::mhz(433)),
            nodes: vec![
                node(0, "DC block", &[0], 1),
                node(1, "Spectrum", &[1], 2),
                node(2, "31 kHz bank", &[1], 3),
                node(3, "Mixer", &[1], 4),
                node(4, "Demod", &[4], 5),
            ],
            output_slot: 5,
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

    #[test]
    fn a_composite_makes_room_for_what_it_runs() {
        let mut t = branchy();
        let inner = Topology {
            input: t.input,
            nodes: vec![node(0, "Envelope", &[0], 1), node(1, "OOK pulses", &[1], 2)],
            output_slot: 2,
        };
        t.nodes[2].inner = Some(Box::new(inner));
        t.nodes[2].inner_count = 74;
        let places = layout(&t);
        let with = row_height(&t, &places, 1);
        t.nodes[2].inner = None;
        let without = row_height(&t, &places, 1);
        assert!(with > without, "a bank's channel chain has to fit somewhere");
    }
}
