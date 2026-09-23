use egui::{Color32, Pos2, Rect, Response, Ui, Vec2};
use egui_bench::theme;
use egui_bench::trace::Trace;

/// Full scale of the speed trace, in octaves either side of real time: the
/// top of the well is 8x and the bottom is an eighth.
const SPEED_OCTAVES: f32 = 3.0;

/// Below this the margin is thin enough to be worth saying so before a block
/// is actually late.
const SPEED_TIGHT: f32 = 1.5;

/// How fast the graph is running against real time, drawn as a trace over a
/// line at 1x.
///
/// A lamp lived here and said too little: green meant "nothing has been
/// dropped yet", which is the same colour whether the host has ten times the
/// headroom it needs or is a hair from falling over. What an operator about to
/// add a channel wants is the margin, and the margin is only readable against
/// real time, so the trace is drawn against a 1x rule. Touching that rule is
/// the warning; crossing it is the fault, and the dropped count that used to
/// be the whole reading is behind the hover.
pub fn speed_trace(ui: &mut Ui, size: Vec2, running: bool, dropped: u64, hist: &[f32]) -> Response {
    let now = hist.last().copied().unwrap_or(0.0);
    let worst = hist.iter().copied().fold(f32::INFINITY, f32::min);
    let col = if !running || dropped > 0 || worst < 1.0 {
        theme::FAULT
    } else if worst < SPEED_TIGHT {
        theme::READOUT
    } else {
        theme::OK
    };

    let resp = ui.add(Trace::new(hist).size(size).ratio(1.0, SPEED_OCTAVES).tint(col));
    resp.on_hover_text(if !running {
        "Stopped. The device is free for another program.".to_string()
    } else if hist.is_empty() {
        "Receiving. No block has been timed yet.".to_string()
    } else if dropped == 0 {
        format!(
            "Running at {now:.1}x real time, worst {worst:.1}x of the last {} blocks. \
             No samples dropped.",
            hist.len()
        )
    } else {
        format!(
            "Running at {now:.1}x real time, worst {worst:.1}x, and {} samples were dropped: \
             the host is not keeping up with this span.",
            super::burst::thousands(dropped)
        )
    })
}

/// Resolution bandwidth, which is what the bin count actually buys you.
pub fn bin_hint(rate: f64, bins: usize) -> String {
    let hz = rate / bins as f64;
    if hz >= 1000.0 {
        format!("{:.1} kHz per bin", hz / 1e3)
    } else {
        format!("{hz:.0} Hz per bin")
    }
}

/// Settings affordance in a pane corner.
pub fn cog_rect(pane: &Rect) -> Rect {
    let s = 18.0;
    Rect::from_min_size(Pos2::new(pane.right() - s - 6.0, pane.top() + 6.0), Vec2::splat(s))
}

pub fn cog(p: &egui::Painter, r: &Rect, hot: bool) {
    let col = if hot { theme::READOUT } else { Color32::from_rgb(0x6A, 0x72, 0x7C) };
    crate::icons::Icon::Setup.paint(p, *r, col);
}
