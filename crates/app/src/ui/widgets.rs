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

use crate::theme;
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
        Vu::paint(p, Rect::from_center_size(rect.center(), Vec2::new(rect.width(), VU_H)), self.peak);

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
        let (lo, hi) = self.range;
        let (rect, mut resp) =
            ui.allocate_exact_size(Vec2::new(120.0, 12.0), Sense::click_and_drag());
        let at = |v: f32| rect.left() + ((v - lo) / (hi - lo)).clamp(0.0, 1.0) * rect.width();

        if let Some(pos) = resp.interact_pointer_pos() {
            if resp.dragged() || resp.clicked() {
                let t = ((pos.x - rect.left()) / rect.width()).clamp(0.0, 1.0);
                *self.threshold = lo + t * (hi - lo);
                resp.mark_changed();
            }
        }

        let p = ui.painter();
        p.rect_filled(rect, 2.0, theme::PANEL);
        // Coloured by the decision rather than by the level, so a glance says
        // whether audio is getting through without reading the numbers.
        p.rect_filled(
            Rect::from_min_max(rect.min, Pos2::new(at(self.measured), rect.max.y)),
            2.0,
            if self.open { theme::TRACE } else { theme::LEGEND },
        );
        let x = at(*self.threshold);
        p.line_segment(
            [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
            Stroke::new(1.5, theme::VALUE),
        );
        resp
    }
}
