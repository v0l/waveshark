//! The recordings as a timeline: clips on a clock with the dead air taken
//! out, played from wherever the pointer is.
//!
//! The table below answers "what was recorded"; this answers "what happened".
//! A conversation is overs separated by silence, and a list of rows hides
//! exactly the thing worth seeing: who answered quickly, where the pauses
//! were, which minute was busy.
//!
//! # The axis is not the clock
//!
//! A talkgroup's day is four minutes of speech in twenty-four hours, and
//! drawn against real time that is a few hairlines at one end of the pane. So
//! anything longer than [`BREAK_AFTER_S`] with nobody transmitting is not
//! drawn at all: it becomes a marker saying how long it was, and the clips
//! either side of it sit next to each other. What is left is an axis measured
//! in speech rather than in hours, and it is the same axis the playback runs
//! on, so a break costs [`BREAK_S`] to listen to and [`BREAK_S`] of width to
//! look at and the cursor stays where the eye is.
//!
//! Three things can be done to it, and they are the three an editor offers:
//! click to play from there, drag to select a stretch, and write that stretch
//! out.

use super::*;
use crate::calllog::Entry;
use std::collections::HashMap;
use std::path::PathBuf;

/// How tall the clip lane is, the ruler above it and the overview under it.
const LANE_H: f32 = 84.0;
const RULER_H: f32 = 18.0;
const OVERVIEW_H: f32 = 12.0;

/// Quiet longer than this is a marker rather than a stretch of lane, and is
/// not played through either.
///
/// Three seconds: that is about as long as a pause between two overs of the
/// same exchange, and it is also as long as anybody will sit through. Beyond
/// it the silence says nothing the marker does not say better.
pub const BREAK_AFTER_S: f64 = 3.0;

/// What a break is worth, in the seconds the axis is measured in.
///
/// It is both the width of the marker and the silence playback puts there, so
/// a minute of listening is a minute of lane wherever the cursor is.
pub const BREAK_S: f64 = 0.8;

/// The most one press plays, from wherever it started.
const PLAY_MAX_S: f64 = 300.0;

/// Clips whose envelope is decoded in one frame, so a folder holding a week
/// of a repeater fills the lane over a few frames rather than stalling one.
const DECODE_PER_FRAME: usize = 6;

/// Peaks kept for one clip.
const PEAKS: usize = 256;

/// The room a clock label needs before the next one is worth drawing.
const CLOCK_GAP: f32 = 110.0;

/// The width a break marker needs before its length is written on it.
const BREAK_LABEL_W: f32 = 22.0;

#[derive(Default)]
pub(super) struct TimelineState {
    pub open: bool,
    /// The window, in axis seconds: where it starts and how wide it is.
    pub view: Option<(f64, f64)>,
    /// The stretch somebody dragged out, and the anchor of a drag in flight,
    /// both in axis seconds.
    pub sel: Option<(f64, f64)>,
    pub drag: Option<f64>,
    /// Whether the drag in flight began on the clock, which pans rather than
    /// selects for as long as it lasts.
    ruler_drag: bool,
    /// Where the last press started playing and how long it queued, so the
    /// cursor can be drawn against what the player has left.
    pub playing: Option<(f64, f64)>,
    /// One clip's envelope, keyed by the file and the offset in it.
    peaks: HashMap<(PathBuf, u64), Vec<f32>>,
}

impl TimelineState {
    /// Zoom about the middle of what is on screen, which is what the
    /// buttons do and what keeps the thing being looked at in view.
    pub fn zoom(&mut self, factor: f64, total: f64) {
        let Some((from, span)) = self.view else { return };
        let middle = from + span / 2.0;
        let span = (span * factor).clamp(0.2, total.max(1.0));
        self.view = Some(bounded(middle - span / 2.0, span, total));
    }

    /// Fit the view to all of it, which is what opening it does and what FIT
    /// does afterwards.
    pub fn fit(&mut self, entries: &[Entry]) {
        let map = Map::of(entries);
        self.view = Some((0.0, map.total().max(1.0)));
        self.sel = None;
        self.playing = None;
    }
}

/// A window held inside the axis: nothing is drawn before the first over or
/// after the last, so panning cannot run off into empty lane and zooming out
/// stops at the whole of it.
fn bounded(from: f64, span: f64, total: f64) -> (f64, f64) {
    let span = span.min(total.max(0.2));
    (from.clamp(0.0, (total - span).max(0.0)), span)
}

/// Where each stretch of recorded air sits on the axis.
///
/// A run of overs with nothing longer than [`BREAK_AFTER_S`] between them is
/// one stretch, drawn at real speed; between two stretches is a break, drawn
/// as a marker and worth [`BREAK_S`] on the axis whatever it was on the
/// clock.
pub(super) struct Map {
    runs: Vec<Run>,
}

struct Run {
    from_us: u64,
    to_us: u64,
    /// Where the run starts on the axis, in seconds.
    at_s: f64,
    /// The quiet before it, in seconds, or zero for the first run.
    break_s: f64,
}

impl Map {
    pub fn of(entries: &[Entry]) -> Self {
        let mut order: Vec<&Entry> = entries.iter().collect();
        order.sort_by_key(|e| e.call.at_us);
        let mut runs: Vec<Run> = Vec::new();
        let mut at_s = 0.0;
        for e in order {
            let (from, to) = (e.call.at_us, e.call.end_us().max(e.call.at_us + 1));
            match runs.last_mut() {
                Some(last) if from.saturating_sub(last.to_us) as f64 / 1e6 <= BREAK_AFTER_S => {
                    last.to_us = last.to_us.max(to);
                }
                _ => {
                    let quiet = match runs.last() {
                        Some(l) => {
                            at_s = l.at_s + (l.to_us - l.from_us) as f64 / 1e6 + BREAK_S;
                            from.saturating_sub(l.to_us) as f64 / 1e6
                        }
                        None => 0.0,
                    };
                    runs.push(Run { from_us: from, to_us: to, at_s, break_s: quiet });
                }
            }
        }
        Self { runs }
    }

    /// How long the whole axis is, in seconds.
    pub fn total(&self) -> f64 {
        self.runs.last().map(|r| r.at_s + (r.to_us - r.from_us) as f64 / 1e6).unwrap_or(0.0)
    }

    /// Where a moment sits on the axis, and nothing for one inside a break.
    fn at(&self, us: u64) -> Option<f64> {
        let r = self.runs.iter().find(|r| us >= r.from_us && us <= r.to_us)?;
        Some(r.at_s + (us - r.from_us) as f64 / 1e6)
    }

    /// The real stretches of air a span of the axis covers, in order.
    pub fn ranges(&self, from_s: f64, to_s: f64) -> Vec<(u64, u64)> {
        let mut out = Vec::new();
        for r in &self.runs {
            let len = (r.to_us - r.from_us) as f64 / 1e6;
            let (a, b) = (r.at_s, r.at_s + len);
            if b < from_s || a > to_s {
                continue;
            }
            let start = r.from_us + ((from_s.max(a) - a) * 1e6) as u64;
            let end = r.from_us + ((to_s.min(b) - a) * 1e6) as u64;
            if end > start {
                out.push((start, end));
            }
        }
        out
    }
}

/// What the lane wants done that it cannot do itself.
pub(super) enum Act {
    /// Play these stretches of air, in order, from this point on the axis.
    ///
    /// The point comes back with them because how long the audio turns out
    /// to be is what draws the cursor, and only the caller that builds it
    /// knows that: a press near the end of a conversation asks for five
    /// minutes and gets whatever is left.
    Play {
        at: f64,
        ranges: Vec<(u64, u64)>,
    },
    /// Write them out, through a save dialog.
    Export(Vec<(u64, u64)>),
    Close,
}

/// Draw the lane over `entries`, and say what was asked of it.
pub(super) fn show(
    ui: &mut egui::Ui,
    st: &mut TimelineState,
    entries: &[Entry],
    left_s: f32,
) -> Option<Act> {
    let map = Map::of(entries);
    if st.view.is_none() {
        st.fit(entries);
    }
    let (from, span) = st.view.map(|(f, s)| (f, s.max(0.05)))?;
    let mut press: Option<Press> = None;

    // A card like every other panel: the legend and what it is for in the
    // header, what can be done to it on the right of that, and the lane in
    // the body. The header only records which button was pressed, because
    // the body holds the state and two closures cannot both have it.
    let sel = st.sel;
    let card = panel::card(
        ui,
        Some(theme::TRACE),
        |ui| {
            let mut line = Line::new()
                .legend("timeline")
                .note("clips against the clock, with the dead air taken out");
            if let Some((a, b)) = sel {
                line = line.gap(10.0).set(format!("{:.0} s picked", b - a)).size(11.0);
            }
            line.elided(ui);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.small_button("CLOSE").clicked() {
                    press = Some(Press::Close);
                }
                ui.add_space(6.0);
                if ui.small_button("FIT").on_hover_text("Show all of it").clicked() {
                    press = Some(Press::Fit);
                }
                // Zoom as buttons as well as on the wheel: a wheel over a
                // pane that also scrolls is a guess about which one was
                // meant, and these are not a guess.
                if ui.small_button("\u{2212}").on_hover_text("Further out").clicked() {
                    press = Some(Press::Out);
                }
                if ui.small_button("+").on_hover_text("Closer in").clicked() {
                    press = Some(Press::In);
                }
                ui.add_space(6.0);
                if ui
                    .add_enabled(sel.is_some(), egui::Button::new("EXPORT").small())
                    .on_hover_text("Write the selected stretch out as one Opus file")
                    .clicked()
                {
                    press = Some(Press::Export);
                }
                if ui
                    .add_enabled(sel.is_some(), egui::Button::new("PLAY").small())
                    .on_hover_text("Play the selected stretch")
                    .clicked()
                {
                    press = Some(Press::Play);
                }
            });
        },
        |ui| {
            let mut act: Option<Act> = None;
            let width = ui.available_width();
            ui.add_space(2.0);
            let (rect, resp) = ui.allocate_exact_size(
                Vec2::new(width.max(120.0), RULER_H + LANE_H),
                Sense::click_and_drag(),
            );
            let ruler =
                Rect::from_min_max(rect.left_top(), Pos2::new(rect.right(), rect.top() + RULER_H));
            let lane =
                Rect::from_min_max(Pos2::new(rect.left(), ruler.bottom()), rect.right_bottom());
            // The clock is a handle on the lane rather than a target of its own:
            // a press on it drags the whole view, which is what the strip of
            // times along the top of an editor does. Which it was is remembered
            // for as long as the drag lasts, or letting the pointer wander down
            // into the lane would turn a pan into a selection halfway through.
            if resp.drag_started() {
                st.ruler_drag = resp.interact_pointer_pos().is_some_and(|pos| ruler.contains(pos));
            }
            if resp.drag_stopped() {
                st.ruler_drag = false;
            }
            let ruler_drag = st.ruler_drag && resp.dragged();
            let p = ui.painter_at(rect);
            p.rect_filled(lane, 2.0, theme::WELL);
            p.rect_filled(ruler, 0.0, theme::CHASSIS);

            let at_x = |s: f64| -> f32 { lane.left() + ((s - from) / span) as f32 * lane.width() };
            let at_s = |x: f32| -> f64 {
                from + ((x - lane.left()) / lane.width()).clamp(0.0, 1.0) as f64 * span
            };

            // The selection under the clips, so a clip inside it still reads as a
            // clip rather than as a tinted block.
            if let Some((a, b)) = st.sel {
                let band = Rect::from_min_max(
                    Pos2::new(at_x(a), lane.top()),
                    Pos2::new(at_x(b).max(at_x(a) + 1.0), lane.bottom()),
                );
                p.rect_filled(band, 0.0, theme::READOUT.gamma_multiply(0.18));
                p.rect_stroke(
                    band,
                    0.0,
                    Stroke::new(1.0, theme::READOUT),
                    egui::StrokeKind::Inside,
                );
            }

            // The runs, each with the time it starts at, and the break before it
            // saying how long nobody transmitted. Both labels are skipped where
            // there is no room for them: a hundred runs in a pane's width is a smear
            // if every one is written on, and the rule on the marker is its width
            // rather than a count, so zooming in brings them back.
            let mut labelled = f32::NEG_INFINITY;
            for r in &map.runs {
                let x0 = at_x(r.at_s);
                let x1 = at_x(r.at_s + (r.to_us - r.from_us) as f64 / 1e6);
                if r.break_s > 0.0 {
                    draw_break(&p, lane, at_x(r.at_s - BREAK_S), x0, r.break_s);
                }
                // A run is one recording, not a row of boxes: the band runs from
                // the first over to the last with a zero line through it, and a
                // pause between two overs inside it reads as the silence it was
                // rather than as a hole in the picture.
                if x1 > lane.left() && x0 < lane.right() {
                    let band = Rect::from_min_max(
                        Pos2::new(x0.max(lane.left()), lane.top() + 6.0),
                        Pos2::new(x1.min(lane.right()).max(x0 + 1.0), lane.bottom() - 6.0),
                    );
                    p.rect_filled(band, 1.0, theme::PANEL);
                    p.line_segment(
                        [
                            Pos2::new(band.left(), band.center().y),
                            Pos2::new(band.right(), band.center().y),
                        ],
                        Stroke::new(1.0, theme::TRACE.gamma_multiply(0.35)),
                    );
                }
                if x0 < lane.left() || x0 > lane.right() {
                    continue;
                }
                if x0 - labelled < CLOCK_GAP {
                    continue;
                }
                // The rule is drawn with the label rather than at every run: at a
                // hundred runs to the pane it is the rules, not the clips, that the
                // eye reads as the picture.
                p.line_segment(
                    [Pos2::new(x0, ruler.bottom() - 5.0), Pos2::new(x0, lane.bottom())],
                    Stroke::new(1.0, theme::ETCH),
                );
                labelled = x0;
                p.text(
                    Pos2::new(x0 + 2.0, ruler.top() + 1.0),
                    egui::Align2::LEFT_TOP,
                    crate::segments::when(r.from_us).format("%d %b %H:%M:%S").to_string(),
                    theme::figure(10.0),
                    theme::LEGEND,
                );
            }

            let mut decoded = 0;
            for e in entries {
                let (Some(a), Some(b)) = (map.at(e.call.at_us), map.at(e.call.end_us())) else {
                    continue;
                };
                let (x0, x1) = (at_x(a), at_x(b));
                if x1 < lane.left() || x0 > lane.right() {
                    continue;
                }
                let clip = Rect::from_min_max(
                    Pos2::new(x0, lane.top() + 6.0),
                    Pos2::new(x1.max(x0 + 1.0), lane.bottom() - 6.0),
                );
                let key = (e.file.clone(), e.at);
                if !st.peaks.contains_key(&key) && decoded < DECODE_PER_FRAME && clip.width() > 2.0
                {
                    st.peaks.insert(key.clone(), envelope(e));
                    decoded += 1;
                }
                match st.peaks.get(&key) {
                    Some(peaks) if clip.width() > 2.0 && !peaks.is_empty() => {
                        draw_envelope(&p, clip, peaks)
                    }
                    // Nothing decoded yet, or a clip too narrow to draw one in: the
                    // block itself still says a transmission was here.
                    _ => {
                        p.rect_filled(
                            clip.shrink2(Vec2::new(0.0, clip.height() * 0.3)),
                            1.0,
                            theme::TRACE,
                        );
                    }
                }
            }

            // The cursor: where the player has got to. Playback and the axis are
            // measured in the same seconds, so this is the start plus what has been
            // played.
            if let Some((started, total)) = st.playing
                && left_s > 0.0
            {
                let at = started + (total - left_s as f64).max(0.0);
                // Follow it when it runs off the edge, so a zoomed-in lane does not
                // have to be dragged along behind the playback.
                if at < from || at > from + span {
                    st.view = Some(bounded(at - span / 2.0, span, map.total()));
                }
                let x = at_x(at);
                if lane.x_range().contains(x) {
                    // Amber, two pixels, with a head on the ruler and a wash either
                    // side: a hairline over a lane of cyan clips cannot be found,
                    // and the whole point of it is to be findable while it moves.
                    p.rect_filled(
                        Rect::from_min_max(
                            Pos2::new(x - 3.0, lane.top()),
                            Pos2::new(x + 3.0, lane.bottom()),
                        ),
                        0.0,
                        theme::READOUT.gamma_multiply(0.20),
                    );
                    p.line_segment(
                        [Pos2::new(x, ruler.top() + 6.0), Pos2::new(x, lane.bottom())],
                        Stroke::new(2.0, theme::READOUT),
                    );
                    p.add(egui::Shape::convex_polygon(
                        vec![
                            Pos2::new(x - 5.0, ruler.top() + 2.0),
                            Pos2::new(x + 5.0, ruler.top() + 2.0),
                            Pos2::new(x, ruler.top() + 9.0),
                        ],
                        theme::READOUT,
                        Stroke::NONE,
                    ));
                }
            }

            // Zoom about the pointer, which is what every editor does and what makes
            // a long conversation navigable without a scrollbar.
            if let Some(pos) = resp.hover_pos() {
                // Wheel events rather than the scroll delta: the delta carries
                // whatever a scroll area near the pointer is still easing, and the
                // lane would zoom itself while nobody was touching it.
                let scroll: f32 = ui.input(|i| {
                    i.events
                        .iter()
                        .filter_map(|e| match e {
                            egui::Event::MouseWheel { delta, .. } => Some(delta.y),
                            _ => None,
                        })
                        .sum()
                });
                if scroll.abs() > 0.1 {
                    let anchor = at_s(pos.x);
                    let factor = (1.0 - scroll as f64 * 0.15).clamp(0.2, 5.0);
                    let new_span = (span * factor).clamp(0.2, map.total().max(1.0));
                    st.view = Some(bounded(
                        anchor - (anchor - from) / span * new_span,
                        new_span,
                        map.total(),
                    ));
                }
            }

            // Dragging with the left button picks a stretch; dragging with any
            // other, or anywhere on the ruler, carries the lane along under the
            // pointer. Both on the same surface, because the thing being pointed
            // at is the same thing.
            let panning = resp.dragged_by(egui::PointerButton::Secondary)
                || resp.dragged_by(egui::PointerButton::Middle)
                || ruler_drag;
            if panning {
                let by = resp.drag_delta().x as f64 / lane.width() as f64 * span;
                st.view = Some(bounded(from - by, span, map.total()));
                st.drag = None;
                ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
            }
            if !panning
                && resp.drag_started_by(egui::PointerButton::Primary)
                && let Some(pos) = resp.interact_pointer_pos()
            {
                st.drag = Some(at_s(pos.x));
            }
            if !panning
                && resp.dragged_by(egui::PointerButton::Primary)
                && let (Some(anchor), Some(pos)) = (st.drag, resp.interact_pointer_pos())
            {
                let now = at_s(pos.x);
                st.sel = Some((anchor.min(now), anchor.max(now)));
            }
            if resp.drag_stopped() {
                st.drag = None;
                // A drag that went nowhere is a click, and a click plays from there
                // rather than selecting nothing.
                if let Some((a, b)) = st.sel.filter(|(a, b)| b - a < span / 200.0) {
                    let _ = b;
                    st.sel = None;
                    act = Some(Act::Play { at: a, ranges: map.ranges(a, a + PLAY_MAX_S) });
                }
            }
            if resp.clicked()
                && let Some(pos) = resp.interact_pointer_pos()
            {
                let at = at_s(pos.x);
                st.sel = None;
                act = Some(Act::Play { at, ranges: map.ranges(at, at + PLAY_MAX_S) });
            }
            if resp.hovered() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::Text);
            }

            overview(ui, st, &map, (from, span));

            ui.add_space(2.0);
            hint(
                ui,
                "Click to play from there, drag to select a stretch, drag the clock or the right \
             button to pan, scroll or the buttons to zoom. Quiet longer than three seconds is a \
             marker saying how long it lasted, and is not played.",
            );
            act
        },
    );
    let mut act = card.inner;
    match (press, sel) {
        (Some(Press::Close), _) => act = Some(Act::Close),
        (Some(Press::Fit), _) => st.fit(entries),
        (Some(Press::In), _) => st.zoom(0.5, map.total()),
        (Some(Press::Out), _) => st.zoom(2.0, map.total()),
        (Some(Press::Export), Some((a, b))) => act = Some(Act::Export(map.ranges(a, b))),
        (Some(Press::Play), Some((a, b))) => {
            act = Some(Act::Play { at: a, ranges: map.ranges(a, b) })
        }
        _ => {}
    }
    act
}

/// A button in the card's header, applied once the body has had the state.
#[derive(Clone, Copy)]
enum Press {
    Close,
    Fit,
    In,
    Out,
    Export,
    Play,
}

/// The whole conversation in a strip under the lane, with the window on it.
///
/// A zoomed-in lane has no edges to tell you where in the afternoon you are,
/// and a scrollbar would say how far along without saying where the talking
/// was. This says both: every run is a tick, the window is a box, and
/// dragging the box is how the lane is panned.
fn overview(ui: &mut egui::Ui, st: &mut TimelineState, map: &Map, view: (f64, f64)) {
    let (from, span) = view;
    let total = map.total().max(1.0);
    let (rect, resp) = ui.allocate_exact_size(
        Vec2::new(ui.available_width() - 24.0, OVERVIEW_H),
        Sense::click_and_drag(),
    );
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 1.0, theme::WELL);
    let at_x = |s: f64| rect.left() + (s / total).clamp(0.0, 1.0) as f32 * rect.width();
    for r in &map.runs {
        let x0 = at_x(r.at_s);
        let x1 = at_x(r.at_s + (r.to_us - r.from_us) as f64 / 1e6);
        p.rect_filled(
            Rect::from_min_max(
                Pos2::new(x0, rect.top() + 2.0),
                Pos2::new(x1.max(x0 + 1.0), rect.bottom() - 2.0),
            ),
            0.0,
            theme::TRACE.gamma_multiply(0.7),
        );
    }
    let window = Rect::from_min_max(
        Pos2::new(at_x(from), rect.top()),
        Pos2::new(at_x(from + span).max(at_x(from) + 2.0), rect.bottom()),
    );
    p.rect_filled(window, 0.0, theme::READOUT.gamma_multiply(0.18));
    p.rect_stroke(window, 0.0, Stroke::new(1.0, theme::READOUT), egui::StrokeKind::Inside);

    // A press anywhere on the strip centres the window there, and dragging
    // carries it: the same gesture whichever was meant.
    if (resp.clicked() || resp.dragged())
        && let Some(pos) = resp.interact_pointer_pos()
    {
        let middle = ((pos.x - rect.left()) / rect.width()).clamp(0.0, 1.0) as f64 * total;
        st.view = Some(bounded(middle - span / 2.0, span, total));
    }
    if resp.hovered() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
    }
}

/// The marker that stands for a stretch of empty band: a hatched gap with how
/// long it was written in it.
fn draw_break(p: &egui::Painter, lane: Rect, x0: f32, x1: f32, seconds: f64) {
    let (x0, x1) = (x0.max(lane.left()), x1.min(lane.right()));
    if x1 <= x0 {
        return;
    }
    // Never thinner than a couple of pixels: a break is the one thing on
    // this lane that says time was taken out, and one it is too narrow to
    // see is a lane that lies about being continuous.
    let band =
        Rect::from_min_max(Pos2::new(x0, lane.top()), Pos2::new(x1.max(x0 + 3.0), lane.bottom()));
    p.rect_filled(band, 0.0, theme::CHASSIS);
    // A cut rather than a thing that was recorded: a rule through the middle
    // where the lane would have been, and the hatching only where there is
    // room for it to read as hatching rather than as noise.
    p.line_segment(
        [Pos2::new(band.left(), band.center().y), Pos2::new(band.right(), band.center().y)],
        Stroke::new(1.0, theme::ETCH),
    );
    if band.width() >= 8.0 {
        for k in 0..2 {
            let x = band.left() + band.width() * (k as f32 + 0.5) / 2.0;
            p.line_segment(
                [Pos2::new(x - 3.0, band.bottom() - 6.0), Pos2::new(x + 3.0, band.top() + 6.0)],
                Stroke::new(1.0, theme::ETCH),
            );
        }
    }
    if band.width() >= BREAK_LABEL_W {
        p.text(
            band.center_top() + Vec2::new(0.0, 2.0),
            egui::Align2::CENTER_TOP,
            span_label(seconds),
            theme::legend_font(9.0),
            theme::LEGEND,
        );
    }
}

/// How long a quiet stretch was, as somebody would say it.
fn span_label(seconds: f64) -> String {
    match seconds {
        s if s < 90.0 => format!("{s:.0}s"),
        s if s < 5_400.0 => format!("{:.0}m", s / 60.0),
        s if s < 172_800.0 => format!("{:.0}h", s / 3_600.0),
        s => format!("{:.0}d", s / 86_400.0),
    }
}

/// A clip's envelope, drawn as a bar per column about the middle.
fn draw_envelope(p: &egui::Painter, clip: Rect, peaks: &[f32]) {
    let mid = clip.center().y;
    let half = clip.height() / 2.0 - 1.0;
    let cols = (clip.width().floor() as usize).clamp(1, peaks.len().max(1));
    for c in 0..cols {
        let lo = c * peaks.len() / cols;
        let hi = ((c + 1) * peaks.len() / cols).max(lo + 1).min(peaks.len());
        let v = peaks[lo..hi].iter().fold(0.0f32, |a, s| a.max(*s)).clamp(0.0, 1.0);
        let x = clip.left() + c as f32 + 0.5;
        let h = (v * half).max(0.5);
        p.line_segment(
            [Pos2::new(x, mid - h), Pos2::new(x, mid + h)],
            Stroke::new(1.0, theme::TRACE),
        );
    }
}

/// One over decoded and reduced to what a lane can draw.
///
/// Normalised to the loudest sample in the over, because what is being read
/// off this is where the speech was rather than how loud the transmitter
/// happened to be. The level column in the table is the other question.
fn envelope(e: &Entry) -> Vec<f32> {
    let Some(speech) = crate::calllog::speech_of(e) else {
        return Vec::new();
    };
    if speech.pcm.is_empty() {
        return Vec::new();
    }
    let step = speech.pcm.len().div_ceil(PEAKS).max(1);
    let peaks: Vec<f32> =
        speech.pcm.chunks(step).map(|c| c.iter().fold(0.0f32, |a, s| a.max(s.abs()))).collect();
    let loudest = peaks.iter().fold(0.0f32, |a, v| a.max(*v));
    match loudest > 1e-4 {
        true => peaks.iter().map(|v| v / loudest).collect(),
        false => peaks,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(at_us: u64, ms: u32) -> Entry {
        Entry {
            call: crate::calllog::Call {
                at_us,
                channel_hz: 446_050_000,
                duration_ms: ms,
                peak: 0.5,
                system: "Audio".into(),
                from: None,
                to: Some("PMR5".into()),
                frames: Vec::new(),
            },
            file: PathBuf::from("day.wscal"),
            at: at_us,
            len: 0,
        }
    }

    /// Two overs a few seconds apart are one run at real speed; an hour of
    /// nothing between them is a break worth [`BREAK_S`], whatever the hour
    /// was.
    #[test]
    fn dead_air_costs_a_marker_rather_than_an_hour_of_lane() {
        let entries =
            [entry(10_000_000, 2_000), entry(15_000_000, 3_000), entry(3_615_000_000, 1_000)];
        let map = Map::of(&entries);
        assert_eq!(map.runs.len(), 2, "the five second pause split a run");
        // Eight seconds of the first run, then the break, then one second.
        assert!((map.total() - (8.0 + BREAK_S + 1.0)).abs() < 0.01, "{}", map.total());
        // The hour is on the marker, and it is the hour it was.
        // Measured from the end of the last over to the start of the next,
        // which is what an operator watching the band saw.
        assert!((map.runs[1].break_s - 3_597.0).abs() < 1.0, "{}", map.runs[1].break_s);
        assert_eq!(span_label(map.runs[1].break_s), "60m");

        // A moment in the second run lands after the break, not an hour
        // along an axis that does not have one.
        let at = map.at(3_615_500_000).expect("the second run is on the axis");
        assert!((at - (8.0 + BREAK_S + 0.5)).abs() < 0.01, "{at}");

        // Selecting across the break gives the two stretches of real air it
        // covers, and not the hour between them.
        let ranges = map.ranges(0.0, map.total());
        assert_eq!(ranges.len(), 2);
        assert_eq!(ranges[0].0, 10_000_000);
        assert_eq!(ranges[1].1, 3_616_000_000);
    }

    /// The axis starts at zero and a lone over still has a lane worth
    /// dragging in.
    #[test]
    fn one_over_is_its_own_run() {
        let map = Map::of(&[entry(10_000_000, 200)]);
        assert_eq!(map.runs.len(), 1);
        assert!((map.total() - 0.2).abs() < 0.01, "{}", map.total());
        assert_eq!(map.at(10_000_000), Some(0.0));
    }
}
