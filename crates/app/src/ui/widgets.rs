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

/// A level meter.
///
/// The scale is not linear in amplitude. Speech spends most of its time well
/// below full scale, and a linear bar leaves that as a stub near the left end
/// where no movement is readable. The square root spreads the quiet half of
/// the range across most of the bar, which is where the useful reading is.
pub struct Vu {
    peak: f32,
    width: f32,
}

impl Vu {
    pub fn new(peak: f32) -> Self {
        Self { peak, width: 120.0 }
    }

    pub fn width(mut self, w: f32) -> Self {
        self.width = w;
        self
    }

    /// Paint a meter into a rectangle already laid out, for the tables that
    /// paint their own rows rather than filling them with widgets.
    pub fn paint(p: &egui::Painter, r: Rect, peak: f32) {
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
}

impl Widget for Vu {
    fn ui(self, ui: &mut Ui) -> Response {
        let w = self.width.min(ui.available_width()).max(24.0);
        let (r, resp) = ui.allocate_exact_size(Vec2::new(w, VU_H), Sense::hover());
        if ui.is_rect_visible(r) {
            Vu::paint(ui.painter(), r, self.peak);
        }
        resp
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
        Vu::paint(
            p,
            Rect::from_center_size(rect.center(), Vec2::new(rect.width(), VU_H)),
            self.peak,
        );

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

/// A section legend with its explanation on a "?" at the end of the row.
///
/// Against the right edge rather than against the label, so the icons form a
/// column instead of a ragged edge following the length of each word.
pub fn legend_help(ui: &mut egui::Ui, label: &str, text: &str) {
    ui.horizontal(|ui| {
        ui.label(legend(label));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            help(ui, text);
        });
    });
}

/// A checkbox with its explanation on a "?" at the end of the row.
pub fn check_help(ui: &mut egui::Ui, on: &mut bool, label: &str, text: &str) -> Response {
    ui.horizontal(|ui| {
        let r = ui.checkbox(on, label);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            help(ui, text);
        });
        r
    })
    .inner
}

/// The heading every modal wears, so one dialog does not announce itself in a
/// different voice from the next.
pub fn modal_title(ui: &mut egui::Ui, text: &str) {
    ui.label(legend(text));
    ui.add_space(10.0);
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
