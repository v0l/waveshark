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

pub fn draw(ui: &mut egui::Ui, topo: &Topology, latency_ms: f64) {
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

    let (rect, _) =
        ui.allocate_exact_size(Vec2::new(width, height.max(ui.available_height())), Sense::hover());
    let p = ui.painter_at(rect);
    let cx = rect.center().x;

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

    let mut rects: Vec<Rect> = Vec::with_capacity(topo.nodes.len());
    let span = widest as f32 * box_w + (widest.saturating_sub(1)) as f32 * COL_GAP;
    for (node, pl) in topo.nodes.iter().zip(&places) {
        let x = cx - span / 2.0 + pl.col as f32 * (box_w + COL_GAP) + box_w / 2.0;
        let r = Rect::from_center_size(
            Pos2::new(x, row_top[pl.depth] + BOX_H / 2.0),
            Vec2::new(box_w, BOX_H),
        );
        rects.push(r);
        stage(&p, r, &node.label, &node.kind, false);

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
    }

    // Edges last, so they sit under nothing and above the background.
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
            edge(&p, from.center_bottom(), to.center_top(), spec);
        }
        // A leaf that carries a stream says where it goes. A sink writes no
        // buffer at all, so labelling one with a rate would invent an output
        // that does not exist.
        let feeds_anything = topo.nodes.iter().any(|other| {
            other.inputs.iter().any(|(s, _)| node.outputs.iter().any(|(o, _)| o == s))
        });
        if !feeds_anything && !node.sink {
            if let Some((_, spec)) = node.outputs.first() {
                let below = to.bottom()
                    + node.inner.as_ref().map(|t| {
                        INNER_GAP + t.nodes.len() as f32 * (INNER_H + INNER_GAP)
                    }).unwrap_or(0.0);
                p.text(
                    Pos2::new(to.center().x, below + 6.0),
                    egui::Align2::CENTER_TOP,
                    format!("out  {}  {}", kind_label(spec.kind), rate_label(spec)),
                    FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into())),
                    theme::TRACE,
                );
            }
        }
    }
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
fn edge(p: &egui::Painter, from: Pos2, to: Pos2, spec: &StreamSpec) {
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
    p.text(
        Pos2::new(to.x + 8.0, (mid + to.y) / 2.0),
        egui::Align2::LEFT_CENTER,
        format!("{}  {}", kind_label(spec.kind), rate_label(spec)),
        FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into())),
        theme::LEGEND,
    );
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
        }
    }

    /// Slot 0 is the graph input; a node's output slot is its own.
    fn branchy() -> Topology {
        Topology {
            input: StreamSpec::iq(2_400_000.0, Hz::mhz(433)),
            nodes: vec![
                node(0, "DC block", &[0], 1),
                node(1, "Spectrum", &[1], 2),
                node(2, "OOK bank", &[1], 3),
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
