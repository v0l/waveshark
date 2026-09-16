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

/// How tall the clip lane is, and the ruler above it.
const LANE_H: f32 = 84.0;
const RULER_H: f32 = 18.0;

/// Quiet longer than this is a marker rather than a stretch of lane.
///
/// Ten seconds: shorter than that is somebody thinking before they answer,
/// which is part of the conversation, and longer is the band being empty.
pub const BREAK_AFTER_S: f64 = 10.0;

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
    /// Where the last press started playing and how long it queued, so the
    /// cursor can be drawn against what the player has left.
    pub playing: Option<(f64, f64)>,
    /// One clip's envelope, keyed by the file and the offset in it.
    peaks: HashMap<(PathBuf, u64), Vec<f32>>,
}

impl TimelineState {
    /// Fit the view to all of it, which is what opening it does and what FIT
    /// does afterwards.
    pub fn fit(&mut self, entries: &[Entry]) {
        let map = Map::of(entries);
        self.view = Some((0.0, map.total().max(1.0)));
        self.sel = None;
        self.playing = None;
    }
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
    /// Play these stretches of air, in order.
    Play(Vec<(u64, u64)>),
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
    let mut act = None;

    ui.horizontal(|ui| {
        ui.add_space(12.0);
        let breaks = entries.len().saturating_sub(map.runs.len());
        let mut line = theme::Line::new()
            .legend("timeline")
            .value(format!("{} overs", entries.len()))
            .size(11.0);
        if breaks > 0 {
            line = line.gap(12.0).value(format!("{} runs", map.runs.len())).size(11.0);
        }
        if let Some((a, b)) = st.sel {
            line = line.gap(12.0).set(format!("{:.0} s picked", b - a)).size(11.0);
        }
        line.show(ui);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.add_space(12.0);
            if ui.small_button("CLOSE").clicked() {
                act = Some(Act::Close);
            }
            ui.add_space(8.0);
            if ui.small_button("FIT").on_hover_text("Show all of it").clicked() {
                st.fit(entries);
            }
            ui.add_space(8.0);
            let sel = st.sel;
            if ui
                .add_enabled(sel.is_some(), egui::Button::new("EXPORT").small())
                .on_hover_text("Write the selected stretch out as one Opus file")
                .clicked()
                && let Some((a, b)) = sel
            {
                act = Some(Act::Export(map.ranges(a, b)));
            }
            ui.add_space(8.0);
            if ui
                .add_enabled(sel.is_some(), egui::Button::new("PLAY").small())
                .on_hover_text("Play the selected stretch")
                .clicked()
                && let Some((a, b)) = sel
            {
                st.playing = Some((a, b - a));
                act = Some(Act::Play(map.ranges(a, b)));
            }
        });
    });

    let width = ui.available_width() - 24.0;
    ui.add_space(2.0);
    let (rect, resp) = ui.allocate_exact_size(
        Vec2::new(width.max(120.0), RULER_H + LANE_H),
        Sense::click_and_drag(),
    );
    let ruler = Rect::from_min_max(rect.left_top(), Pos2::new(rect.right(), rect.top() + RULER_H));
    let lane = Rect::from_min_max(Pos2::new(rect.left(), ruler.bottom()), rect.right_bottom());
    let p = ui.painter_at(rect);
    p.rect_filled(lane, 2.0, theme::WELL);
    p.rect_filled(ruler, 0.0, theme::CHASSIS);

    let at_x = |s: f64| -> f32 { lane.left() + ((s - from) / span) as f32 * lane.width() };
    let at_s =
        |x: f32| -> f64 { from + ((x - lane.left()) / lane.width()).clamp(0.0, 1.0) as f64 * span };

    // The selection under the clips, so a clip inside it still reads as a
    // clip rather than as a tinted block.
    if let Some((a, b)) = st.sel {
        let band = Rect::from_min_max(
            Pos2::new(at_x(a), lane.top()),
            Pos2::new(at_x(b).max(at_x(a) + 1.0), lane.bottom()),
        );
        p.rect_filled(band, 0.0, theme::READOUT.gamma_multiply(0.18));
        p.rect_stroke(band, 0.0, Stroke::new(1.0, theme::READOUT), egui::StrokeKind::Inside);
    }

    // The runs, each with the time it starts at, and the break before it
    // saying how long nobody transmitted. Both labels are skipped where
    // there is no room for them: a hundred runs in a pane's width is a smear
    // if every one is written on, and the rule on the marker is its width
    // rather than a count, so zooming in brings them back.
    let mut labelled = f32::NEG_INFINITY;
    for r in &map.runs {
        let x0 = at_x(r.at_s);
        if r.break_s > 0.0 {
            draw_break(&p, lane, at_x(r.at_s - BREAK_S), x0, r.break_s);
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
            egui::FontId::new(10.0, egui::FontFamily::Name(theme::READOUT_FONT.into())),
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
        // Half a pixel off each side, so two overs a second apart read as
        // two clips rather than as one block.
        let clip = Rect::from_min_max(
            Pos2::new(x0 + 0.5, lane.top() + 6.0),
            Pos2::new((x1 - 0.5).max(x0 + 1.5), lane.bottom() - 6.0),
        );
        p.rect_filled(clip, 1.0, theme::PANEL);
        let key = (e.file.clone(), e.at);
        if !st.peaks.contains_key(&key) && decoded < DECODE_PER_FRAME && clip.width() > 2.0 {
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
                p.rect_filled(clip.shrink2(Vec2::new(0.0, clip.height() * 0.3)), 1.0, theme::TRACE);
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
        let x = at_x(at);
        if lane.x_range().contains(x) {
            p.line_segment(
                [Pos2::new(x, lane.top()), Pos2::new(x, lane.bottom())],
                Stroke::new(1.0, theme::OK),
            );
        }
    }

    // Zoom about the pointer, which is what every editor does and what makes
    // a long conversation navigable without a scrollbar.
    if let Some(pos) = resp.hover_pos() {
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll.abs() > 0.1 {
            let anchor = at_s(pos.x);
            let factor = (1.0 - scroll as f64 * 0.002).clamp(0.2, 5.0);
            let new_span = (span * factor).clamp(0.2, map.total().max(1.0) * 1.5);
            st.view = Some((anchor - (anchor - from) / span * new_span, new_span));
        }
    }

    if resp.drag_started()
        && let Some(pos) = resp.interact_pointer_pos()
    {
        st.drag = Some(at_s(pos.x));
    }
    if resp.dragged()
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
            st.playing = Some((a, PLAY_MAX_S));
            act = Some(Act::Play(map.ranges(a, a + PLAY_MAX_S)));
        }
    }
    if resp.clicked()
        && let Some(pos) = resp.interact_pointer_pos()
    {
        let at = at_s(pos.x);
        st.sel = None;
        st.playing = Some((at, PLAY_MAX_S));
        act = Some(Act::Play(map.ranges(at, at + PLAY_MAX_S)));
    }
    if resp.hovered() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::Text);
    }

    ui.add_space(2.0);
    hint(
        ui,
        "Click to play from there, drag to select a stretch, scroll to zoom. Quiet longer than \
         ten seconds is a marker saying how long it lasted, not lane. A row in the table below \
         opens its own conversation.",
    );
    act
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
    p.rect_stroke(
        band,
        0.0,
        Stroke::new(1.0, theme::READOUT.gamma_multiply(0.35)),
        egui::StrokeKind::Inside,
    );
    // Two hatched strokes rather than a fill, so it reads as the axis being
    // cut rather than as something that was recorded.
    for k in 0..3 {
        let x = band.left() + band.width() * (k as f32 + 0.5) / 3.0;
        p.line_segment(
            [Pos2::new(x - 3.0, band.bottom() - 4.0), Pos2::new(x + 3.0, band.top() + 4.0)],
            Stroke::new(1.0, theme::ETCH),
        );
    }
    if band.width() >= BREAK_LABEL_W {
        p.text(
            band.center_top() + Vec2::new(0.0, 2.0),
            egui::Align2::CENTER_TOP,
            span_label(seconds),
            egui::FontId::new(9.0, egui::FontFamily::Name(theme::LEGEND_FONT.into())),
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
