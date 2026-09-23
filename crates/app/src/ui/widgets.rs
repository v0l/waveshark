//! The controls the panes are built out of.
//!
//! Each is an [`egui::Widget`]: it borrows the one value it edits and knows
//! nothing about the receiver. That is what makes them usable anywhere a
//! level or a threshold has to be shown, and it is what keeps a pane's own
//! code about the pane rather than about drawing rectangles.
//!
//! The pattern throughout is `ui.add(Thing::new(&mut value, reading))`, so
//! these compose with `add_sized`, `add_enabled` and the rest of egui's
//! layout without any of it being re-implemented here.

use crate::theme::{self, legend};
use egui::{Color32, Pos2, Rect, Response, Sense, Stroke, Ui, Vec2, Widget};

/// Height of a level bar. Short enough to sit on a row with a slider without
/// changing the row's height.
pub const VU_H: f32 = 6.0;

/// Height of a fader, whose track is its own meter.
pub const FADER_H: f32 = 14.0;

/// Half the fader handle's width, and how far the travel is inset from each
/// end so the handle stays inside the track at either extreme.
const GRIP: f32 = 3.0;

/// Where a level bar stops being green.
///
/// Approaching clip, and clipping. The mix is hard limited at full scale, so
/// the red region is where the receiver is discarding what it was given
/// rather than reproducing it.
const WARN: f32 = 0.70;
const PEAK: f32 = 0.90;

const GREEN: Color32 = Color32::from_rgb(0x6F, 0xD1, 0x8A);
const AMBER: Color32 = Color32::from_rgb(0xE8, 0xB0, 0x3E);

/// A level meter, painted into a rectangle already laid out.
///
/// A painter rather than a widget, like [`cell`]: whoever draws the row owns
/// the rectangle, and a fader's track is this same meter drawn inside it.
///
/// The scale is not linear in amplitude. Speech spends most of its time well
/// below full scale, and a linear bar leaves that as a stub near the left end
/// where no movement is readable. The square root spreads the quiet half of
/// the range across most of the bar, which is where the useful reading is.
pub fn vu(p: &egui::Painter, r: Rect, peak: f32) {
    // An empty meter still has to read as a meter. Drawn as a well with
    // an engraved edge and its two region marks, so a silent channel
    // looks silent rather than looking like a control that failed to
    // appear.
    p.rect_filled(r, 1.0, theme::WELL);
    p.rect_stroke(r, 1.0, Stroke::new(1.0, theme::ETCH), egui::StrokeKind::Inside);

    let at = |v: f32| r.left() + v.clamp(0.0, 1.0).sqrt() * r.width();
    for (v, c) in [(WARN, AMBER), (PEAK, theme::FAULT)] {
        let x = at(v);
        p.line_segment(
            [Pos2::new(x, r.top() + 1.0), Pos2::new(x, r.bottom() - 1.0)],
            Stroke::new(1.0, c.gamma_multiply(0.45)),
        );
    }

    let peak = peak.clamp(0.0, 1.0);
    if peak <= 0.001 {
        return;
    }
    // Filled in three pieces so the bar carries its own colour where it
    // reaches: the reading is the colour as much as the length.
    let end = at(peak);
    let mut x = r.left();
    for (limit, colour) in [(WARN, GREEN), (PEAK, AMBER), (1.0, theme::FAULT)] {
        let stop = at(limit).min(end);
        if stop > x {
            p.rect_filled(
                Rect::from_min_max(
                    Pos2::new(x, r.top() + 1.0),
                    Pos2::new(stop.max(x + 1.0), r.bottom() - 1.0),
                ),
                0.0,
                colour,
            );
        }
        x = stop;
        if x >= end {
            break;
        }
    }
}

/// A volume control whose track is its own meter.
///
/// One strip rather than a slider with a bar beneath it: what you set and
/// what that is producing are read in the same glance, and the handle sits at
/// the point on the scale the level is being measured against.
pub struct Fader<'a> {
    value: &'a mut f32,
    peak: f32,
    width: f32,
}

impl<'a> Fader<'a> {
    pub fn new(value: &'a mut f32, peak: f32) -> Self {
        Self { value, peak, width: 130.0 }
    }

    pub fn width(mut self, w: f32) -> Self {
        self.width = w;
        self
    }
}

impl Widget for Fader<'_> {
    fn ui(self, ui: &mut Ui) -> Response {
        let w = self.width.min(ui.available_width()).max(40.0);
        let (rect, mut resp) =
            ui.allocate_exact_size(Vec2::new(w, FADER_H), Sense::click_and_drag());
        let (lo, hi) = (rect.left() + GRIP, rect.right() - GRIP);

        if resp.dragged() || resp.clicked() {
            if let Some(p) = ui.ctx().pointer_interact_pos() {
                let t = ((p.x - lo) / (hi - lo)).clamp(0.0, 1.0);
                if (t - *self.value).abs() > 1e-4 {
                    *self.value = t;
                    resp.mark_changed();
                }
            }
        }
        if !ui.is_rect_visible(rect) {
            return resp;
        }

        let p = ui.painter();
        vu(p, Rect::from_center_size(rect.center(), Vec2::new(rect.width(), VU_H)), self.peak);

        // Amber, because the handle is the one part of this the operator set,
        // and outlined so it stays legible crossing a lit bar of any colour.
        let x = lo + self.value.clamp(0.0, 1.0) * (hi - lo);
        let handle = Rect::from_center_size(
            Pos2::new(x, rect.center().y),
            Vec2::new(GRIP * 2.0, rect.height()),
        );
        p.rect_filled(handle, 1.0, theme::CHASSIS);
        p.rect_filled(
            handle.shrink(1.0),
            1.0,
            if resp.hovered() || resp.dragged() { theme::VALUE } else { theme::READOUT },
        );
        resp
    }
}

/// A squelch control that shows what it is deciding against.
///
/// A threshold with no meter beside it is a number to guess at: the operator
/// cannot tell whether 9 dB is one above the noise or ten below the station.
/// The bar is what the squelch is measuring right now, the marker is where it
/// opens, and dragging moves the marker.
pub struct Squelch<'a> {
    threshold: &'a mut f32,
    range: (f32, f32),
    measured: f32,
    open: bool,
}

impl<'a> Squelch<'a> {
    pub fn new(threshold: &'a mut f32, lo: f32, hi: f32, measured: f32, open: bool) -> Self {
        Self { threshold, range: (lo, hi), measured, open }
    }
}

impl Widget for Squelch<'_> {
    fn ui(self, ui: &mut Ui) -> Response {
        // The same strip as the volume fader: a well that is the meter, and
        // an amber handle on it that is what the operator set. Drawn
        // differently it read as a different kind of control, and it is not.
        let (lo, hi) = self.range;
        let w = 130.0f32.min(ui.available_width()).max(40.0);
        let (rect, mut resp) =
            ui.allocate_exact_size(Vec2::new(w, FADER_H), Sense::click_and_drag());
        let (x0, x1) = (rect.left() + GRIP, rect.right() - GRIP);
        let frac = |v: f32| ((v - lo) / (hi - lo)).clamp(0.0, 1.0);
        let at = |v: f32| x0 + frac(v) * (x1 - x0);

        if resp.dragged() || resp.clicked() {
            if let Some(p) = ui.ctx().pointer_interact_pos() {
                let t = ((p.x - x0) / (x1 - x0)).clamp(0.0, 1.0);
                let v = lo + t * (hi - lo);
                if (v - *self.threshold).abs() > 1e-3 {
                    *self.threshold = v;
                    resp.mark_changed();
                }
            }
        }
        if !ui.is_rect_visible(rect) {
            return resp;
        }

        let p = ui.painter();
        let well = Rect::from_center_size(rect.center(), Vec2::new(rect.width(), VU_H));
        p.rect_filled(well, 1.0, theme::WELL);
        p.rect_stroke(well, 1.0, Stroke::new(1.0, theme::ETCH), egui::StrokeKind::Inside);
        // The bar is what the squelch is measuring now, coloured by what it
        // decided: a glance says whether audio is getting through.
        let end = at(self.measured);
        if end > well.left() + 1.0 {
            p.rect_filled(
                Rect::from_min_max(
                    Pos2::new(well.left() + 1.0, well.top() + 1.0),
                    Pos2::new(end, well.bottom() - 1.0),
                ),
                0.0,
                if self.open { theme::TRACE } else { theme::LEGEND },
            );
        }
        let x = at(*self.threshold);
        let handle = Rect::from_center_size(
            Pos2::new(x, rect.center().y),
            Vec2::new(GRIP * 2.0, rect.height()),
        );
        p.rect_filled(handle, 1.0, theme::CHASSIS);
        p.rect_filled(
            handle.shrink(1.0),
            1.0,
            if resp.hovered() || resp.dragged() { theme::VALUE } else { theme::READOUT },
        );
        resp
    }
}

/// Height of a row in the painted tables.
///
/// The call list, the packet log and the track list are painted rather than
/// filled with widgets: a table of a thousand rows is a thousand allocations
/// a frame otherwise, and every one of them is a rectangle and some text.
pub const ROW_H: f32 = 16.0;

/// One cell of text, clipped to its column so a long field cannot push the
/// ones after it sideways.
pub fn cell(p: &egui::Painter, row: Rect, x: f32, w: f32, text: &str, col: Color32) {
    let r = Rect::from_min_max(Pos2::new(x, row.top()), Pos2::new(x + w - 6.0, row.bottom()));
    p.with_clip_rect(r.intersect(p.clip_rect())).text(
        Pos2::new(r.left(), r.center().y),
        egui::Align2::LEFT_CENTER,
        text,
        egui::FontId::new(11.0, egui::FontFamily::Name(theme::READOUT_FONT.into())),
        col,
    );
}

/// Height of the rail down the left of a card, and the inset of a card's
/// contents from its edge.
const RAIL_W: f32 = 3.0;

/// A block of related lines: a captioned header on its own ground, and the
/// content under it.
///
/// The panes that list what the receiver heard were rows of text on a
/// striped background, which is readable while every row is one line and
/// stops being readable the moment one of them wraps: nothing says where a
/// message ends and the next begins except a shade of grey. A card says it
/// with an edge. The header is recessed into the chassis the way a legend
/// plate is, the body sits proud of it, and the rail down the left is where
/// a card carries its state: amber for what the operator set, cyan for what
/// the radio heard, nothing at all for a card that is only telling you
/// something.
pub fn card<R>(
    ui: &mut Ui,
    rail: Option<Color32>,
    header: impl FnOnce(&mut Ui),
    body: impl FnOnce(&mut Ui) -> R,
) -> egui::InnerResponse<R> {
    let outer =
        egui::Frame::NONE.fill(theme::PANEL).stroke(Stroke::new(1.0, theme::ETCH)).corner_radius(2);
    let framed = outer.show(ui, |ui| {
        // The header and the body are one surface split by a rule, so no
        // spacing may creep in between them.
        ui.spacing_mut().item_spacing.y = 0.0;
        let head = egui::Frame::NONE.fill(theme::WELL).inner_margin(egui::Margin {
            left: 10,
            right: 10,
            top: 4,
            bottom: 4,
        });
        let h = head.show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| header(ui));
        });
        let r = h.response.rect;
        ui.painter().line_segment(
            [Pos2::new(r.left(), r.bottom()), Pos2::new(r.right(), r.bottom())],
            Stroke::new(1.0, theme::ETCH),
        );
        egui::Frame::NONE
            .inner_margin(egui::Margin { left: 10, right: 10, top: 6, bottom: 8 })
            .show(ui, |ui| {
                ui.spacing_mut().item_spacing.y = 4.0;
                ui.set_width(ui.available_width());
                body(ui)
            })
            .inner
    });
    if let Some(c) = rail {
        let r = framed.response.rect;
        ui.painter().rect_filled(
            Rect::from_min_max(r.left_top(), Pos2::new(r.left() + RAIL_W, r.bottom())),
            0.0,
            c,
        );
    }
    framed
}

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

    let (rect, resp) = ui.allocate_exact_size(size, Sense::hover());
    let p = ui.painter();
    p.rect_filled(rect, 1.0, theme::WELL);
    p.rect_stroke(rect, 1.0, Stroke::new(1.0, theme::ETCH), egui::StrokeKind::Inside);

    // Ratios, so 2x above the line has to look like half speed below it; on a
    // linear axis everything slow is squashed into the bottom pixel.
    let plot = rect.shrink(2.0);
    let y = |v: f32| {
        let t = (v.max(0.03).log2() / SPEED_OCTAVES).clamp(-1.0, 1.0);
        plot.center().y - t * plot.height() / 2.0
    };
    let one = y(1.0);
    for x in (0..plot.width() as i32).step_by(4) {
        let x = plot.left() + x as f32;
        p.line_segment(
            [Pos2::new(x, one), Pos2::new((x + 2.0).min(plot.right()), one)],
            Stroke::new(1.0, theme::LEGEND.gamma_multiply(0.7)),
        );
    }

    if hist.len() > 1 {
        let step = plot.width() / (hist.len() - 1) as f32;
        let pts: Vec<Pos2> = hist
            .iter()
            .enumerate()
            .map(|(i, v)| Pos2::new(plot.left() + i as f32 * step, y(*v)))
            .collect();
        p.add(egui::Shape::line(pts, Stroke::new(1.0, col)));
    }

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

/// A line of explanation under a control.
///
/// A `theme::Line` like everything else that puts words on the screen, so a
/// hint beside a reading sits on the same baseline as the reading. It is
/// wrapped rather than shown, because the surrounding layout justifies text
/// inside a modal, which spreads a wrapped sentence across the full width and
/// leaves holes in the middle of it.
pub fn hint(ui: &mut egui::Ui, text: &str) {
    theme::Line::new().note(text).size(10.0).wrapped(ui);
}

/// A "?" carrying, on hover, the explanation that would otherwise sit under a
/// control as a line of prose.
///
/// A settings modal reads as a column of settings only while the settings are
/// what is on it. Paragraphs between them push the next control off the screen
/// and are read once, so the explanation is kept where somebody who wants it
/// will look and out of the way of somebody who does not.
pub fn help(ui: &mut egui::Ui, text: &str) -> Response {
    let (rect, r) = ui.allocate_exact_size(Vec2::splat(14.0), Sense::hover());
    let col = if r.hovered() { theme::READOUT } else { theme::LEGEND };
    if ui.is_rect_visible(rect) {
        let p = ui.painter();
        p.circle_stroke(rect.center(), 6.0, Stroke::new(1.0, col));
        p.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "?",
            egui::FontId::new(10.0, egui::FontFamily::Name(theme::LEGEND_FONT.into())),
            col,
        );
    }
    r.on_hover_text(text)
}

/// The heading every modal wears, so one dialog does not announce itself in a
/// different voice from the next.
pub fn modal_title(ui: &mut egui::Ui, text: &str) {
    ui.label(legend(text));
    ui.add_space(10.0);
}

/// The tabs a modal holding more than one subject wears, under its title.
///
/// A dialog is a column of cards, and a column long enough to scroll is a
/// dialog nobody reads to the end of. Tabs split the subjects it holds
/// without splitting it into two places to look.
///
/// Silkscreened legends over an etched rule, the chosen one marked in amber
/// where the rule breaks for it: the panel's own grammar, where amber is
/// what the operator set. Not a row of keys, because a key is pressed and
/// does something and a tab is where you are.
pub fn tabs<T: PartialEq + Copy>(ui: &mut Ui, current: &mut T, options: &[(T, &str)]) -> bool {
    const TAB_H: f32 = 20.0;
    const MARK_H: f32 = 2.0;
    const GAP: f32 = 20.0;

    let width = ui.available_width();
    let (strip, _) = ui.allocate_exact_size(Vec2::new(width, TAB_H + MARK_H), Sense::hover());
    let mut changed = false;
    let mut marks = Vec::with_capacity(options.len());
    let mut x = strip.left();
    for (i, (value, label)) in options.iter().enumerate() {
        let galley = ui.painter().layout_job(theme::legend_job(label));
        let w = galley.size().x;
        let hit = Rect::from_min_size(Pos2::new(x, strip.top()), Vec2::new(w, TAB_H));
        let r = ui.interact(hit, ui.id().with(("tab", i)), Sense::click());
        let on = *current == *value;
        if r.clicked() && !on {
            *current = *value;
            changed = true;
        }
        let colour = match (on, r.hovered()) {
            (true, _) => theme::VALUE,
            (false, true) => theme::VALUE,
            (false, false) => theme::LEGEND,
        };
        let at = Pos2::new(x, strip.top() + (TAB_H - galley.size().y) * 0.5);
        ui.painter().galley(at, galley, colour);
        marks.push((x, w, on));
        x += w + GAP;
    }
    let rule = Rect::from_min_size(
        Pos2::new(strip.left(), strip.bottom() - MARK_H),
        Vec2::new(width, 1.0),
    );
    ui.painter().rect_filled(rule, 0.0, theme::ETCH);
    for (x, w, on) in marks {
        if on {
            let mark =
                Rect::from_min_size(Pos2::new(x, strip.bottom() - MARK_H), Vec2::new(w, MARK_H));
            ui.painter().rect_filled(mark, 0.0, theme::READOUT);
        }
    }
    ui.add_space(10.0);
    changed
}

/// Width of the legend column in a settings modal, and where what it labels
/// begins.
pub const LABEL_W: f32 = 90.0;
const LABEL_GAP: f32 = 8.0;

/// A labelled settings row: legend on the left, control on the right, so the
/// modal reads as a column of settings rather than a wall of widgets.
pub fn row(ui: &mut egui::Ui, label: &str, add: impl FnOnce(&mut egui::Ui)) {
    ui.horizontal(|ui| {
        ui.add_sized([LABEL_W, 18.0], egui::Label::new(legend(label)));
        add(ui);
    });
}

/// A settings row with its explanation on a "?" at the end of the row.
///
/// The control keeps the column [`row`] puts it in, and the icon lands where
/// every other icon in the modal is, which is what makes them read as one
/// affordance rather than as decoration on particular settings.
pub fn row_help(ui: &mut egui::Ui, label: &str, text: &str, add: impl FnOnce(&mut egui::Ui)) {
    ui.horizontal(|ui| {
        ui.add_sized([LABEL_W, 18.0], egui::Label::new(legend(label)));
        // The icon is taken off the right before the control is drawn, or a
        // slider that fills the row leaves nothing for it to sit in.
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            help(ui, text);
            ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), add);
        });
    });
}

/// A settings row that reads a value back rather than setting one.
///
/// Not [`row`] with a label in it: two labels in a row are two galleys, and
/// egui centres them against each other, so a legend beside its value sits a
/// little low. One line puts them on the same baseline and still lands the
/// value in the column.
pub fn reading(ui: &mut egui::Ui, label: &str, text: impl Into<String>) {
    theme::Line::new()
        .legend(label)
        .column(ui, LABEL_W + LABEL_GAP)
        .value(text)
        .size(11.0)
        .show(ui);
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

/// A text field set into the panel the way a readout is: a well with an
/// etched edge, the text in the readout face, the hint in the legend's
/// grey. Fills the row it is given.
///
/// The stock text edit is a grey box in a proportional face, which is what
/// every other program's is, and on a chassis whose every reading is amber
/// tabular figures in a recess it is the one thing on screen that looks
/// pasted on.
pub fn field(ui: &mut Ui, text: &mut String, hint: &str) -> Response {
    field_of(ui, text, hint, false, 1, f32::INFINITY)
}

/// A [`field`] with controls after it on the same row. `reserve` is the
/// width kept for them: a field that fills the row and then has a button
/// added pushes the button off the edge, and what a button will measure is
/// not known until it is drawn.
pub fn field_then(
    ui: &mut Ui,
    text: &mut String,
    hint: &str,
    reserve: f32,
    after: impl FnOnce(&mut Ui),
) -> Response {
    let w = (ui.available_width() - reserve).max(60.0);
    let r = field_of(ui, text, hint, false, 1, w);
    after(ui);
    r
}

/// A [`field`] whose contents are not shown: a key.
pub fn secret(ui: &mut Ui, text: &mut String) -> Response {
    field_of(ui, text, "", true, 1, f32::INFINITY)
}

/// A [`field`] of several lines, for prose.
pub fn prose(ui: &mut Ui, text: &mut String, hint: &str, rows: usize) -> Response {
    field_of(ui, text, hint, false, rows, f32::INFINITY)
}

/// The well is `width` across, or the row when that is infinite.
fn field_of(
    ui: &mut Ui,
    text: &mut String,
    hint: &str,
    secret: bool,
    rows: usize,
    width: f32,
) -> Response {
    let font = egui::FontId::new(12.5, egui::FontFamily::Name(theme::READOUT_FONT.into()));
    let hint =
        egui::RichText::new(hint).font(font.clone()).color(theme::LEGEND.gamma_multiply(0.6));
    egui::Frame::NONE
        .fill(theme::WELL)
        .stroke(Stroke::new(1.0, theme::ETCH))
        .corner_radius(2)
        .inner_margin(egui::Margin { left: 6, right: 6, top: 3, bottom: 3 })
        .show(ui, |ui| {
            let mut edit = if rows > 1 {
                egui::TextEdit::multiline(text).desired_rows(rows)
            } else {
                egui::TextEdit::singleline(text)
            };
            edit = edit
                .frame(egui::Frame::NONE)
                .font(font)
                .text_color(theme::VALUE)
                .hint_text(hint)
                .password(secret)
                .desired_width(if width.is_finite() { width - 12.0 } else { width });
            ui.add(edit)
        })
        .inner
}

/// A status lamp and what it says: lit green for a thing that will work,
/// red for one that will not and why.
///
/// A settings dialog can only be judged by closing it and trying; this is
/// the trying, done as the fields are typed, so the reason the agent cannot
/// answer is on the same screen as the field that fixes it.
pub fn lamp(ui: &mut Ui, ok: bool, text: &str) {
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(10.0, 18.0), Sense::hover());
        let c = rect.center();
        let col = if ok { theme::OK } else { theme::FAULT };
        ui.painter().circle_filled(c, 3.0, col);
        ui.painter().circle_stroke(c, 4.5, Stroke::new(1.0, col.gamma_multiply(0.4)));
        theme::Line::new().value(text).tint(col).size(11.0).wrapped(ui);
    });
}

/// How far a download has got: a bar in a recess, with the figures on it.
///
/// A length the far end declared draws a bar; one it did not draws a stripe
/// that sweeps, because a bar at a guessed position is a worse answer than
/// no bar. The numbers are on the row either way, since a bar says roughly
/// and a person waiting on 400 MB wants exactly.
pub fn progress(ui: &mut Ui, done: u64, total: Option<u64>) {
    let ctx = ui.ctx().clone();
    let h = 10.0;
    let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), h), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 1.0, theme::WELL);
    match total.filter(|t| *t > 0) {
        Some(total) => {
            let f = (done as f32 / total as f32).clamp(0.0, 1.0);
            let mut bar = rect;
            bar.set_width(rect.width() * f);
            p.rect_filled(bar, 1.0, theme::TRACE);
        }
        None => {
            // Nothing to be a fraction of, so it moves rather than fills:
            // a fifth of the width, sweeping once every two seconds.
            let t = ctx.input(|i| i.time) as f32 % 2.0 / 2.0;
            let w = rect.width() * 0.2;
            let x = rect.left() + (rect.width() + w) * t - w;
            let mut bar = egui::Rect::from_min_size(Pos2::new(x, rect.top()), Vec2::new(w, h));
            bar = bar.intersect(rect);
            p.rect_filled(bar, 1.0, theme::TRACE);
            ctx.request_repaint();
        }
    }
    let said = match total.filter(|t| *t > 0) {
        Some(t) => format!(
            "{} of {} ({:.0}%)",
            crate::data::fmt_bytes(done),
            crate::data::fmt_bytes(t),
            done as f64 / t as f64 * 100.0
        ),
        None => format!("{} so far", crate::data::fmt_bytes(done)),
    };
    theme::Line::new().legend("downloading").value(said).size(11.0).show(ui);
}

/// A card whose header is a legend and, on the right, one line saying what
/// the card is for: the sentence that would otherwise sit under the title
/// as a paragraph nobody reads twice.
pub fn section<R>(
    ui: &mut Ui,
    label: &str,
    note: &str,
    body: impl FnOnce(&mut Ui) -> R,
) -> egui::InnerResponse<R> {
    card(
        ui,
        None,
        |ui| {
            theme::Line::new().legend(label).show(ui);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                theme::Line::new().note(note).size(10.5).elided(ui);
            });
        },
        body,
    )
}

/// A switch in a settings row: the legend in the column every other row
/// keeps, the box and what it does in the control column, the "?" at the
/// end. Returns whether it was thrown.
pub fn switch(ui: &mut Ui, label: &str, on: &mut bool, text: &str, help: &str) -> bool {
    let mut changed = false;
    row_help(ui, label, help, |ui| {
        changed = ui.checkbox(on, text).changed();
    });
    changed
}

/// A choice from a closed list, in the control column, as wide as the
/// field beside it would be. Shows the label of whatever is picked.
pub fn choice<T: PartialEq + Clone>(
    ui: &mut Ui,
    id: impl std::hash::Hash + std::fmt::Debug,
    picked: &mut T,
    options: impl IntoIterator<Item = (T, String)>,
) -> bool {
    let options: Vec<(T, String)> = options.into_iter().collect();
    let shown = options.iter().find(|(v, _)| v == picked).map(|(_, l)| l.clone());
    let mut changed = false;
    egui::ComboBox::from_id_salt(id)
        .selected_text(shown.unwrap_or_default())
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            for (v, label) in options {
                let on = *picked == v;
                if ui.selectable_label(on, label).clicked() && !on {
                    *picked = v;
                    changed = true;
                }
            }
        });
    changed
}

/// The bottom of every modal: a rule, then the buttons against the right
/// edge, the one that closes it outermost.
pub fn footer(ui: &mut Ui, buttons: impl FnOnce(&mut Ui)) {
    ui.add_space(10.0);
    let r = ui.available_rect_before_wrap();
    ui.painter().line_segment(
        [Pos2::new(r.left(), r.top()), Pos2::new(r.right(), r.top())],
        Stroke::new(1.0, theme::ETCH),
    );
    ui.add_space(8.0);
    ui.horizontal(|ui| {
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), buttons);
    });
}
