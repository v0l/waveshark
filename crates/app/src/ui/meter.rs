//! The level meter that goes beside a volume control.
//!
//! A fader says what was asked for; only a meter says what is arriving. The
//! two belong together, so every level in the receiver has one: the master,
//! the call bus, each channel strip, and each row of the call list.
//!
//! The scale is not linear in amplitude. Speech spends most of its time well
//! below full scale, and a linear bar leaves that as a stub near the left end
//! where no movement is readable. The square root spreads the quiet half of
//! the range across most of the bar, which is where the useful reading is.

use crate::theme;
use egui::{Color32, Pos2, Rect, Response, Sense, Stroke, Ui, Vec2};

/// Height of the bar. Short enough to sit on the same row as a slider without
/// changing the row's height.
pub const H: f32 = 6.0;

/// Where the bar stops being green.
///
/// Approaching clip, and clipping. The mix is hard limited at full scale, so
/// the red region is the part of the range where the receiver is discarding
/// what it was given rather than reproducing it.
const WARN: f32 = 0.70;
const PEAK: f32 = 0.90;

const GREEN: Color32 = Color32::from_rgb(0x6F, 0xD1, 0x8A);
const AMBER: Color32 = Color32::from_rgb(0xE8, 0xB0, 0x3E);

/// Height of the combined control: the meter is the fader's track, so the
/// level and the setting share one strip of panel instead of two.
pub const FADER_H: f32 = 14.0;

/// Half the handle's width, and how far the travel is inset from each end so
/// the handle stays inside the track at either extreme.
const GRIP: f32 = 3.0;

/// Paint a meter into a rectangle already laid out.
///
/// Used by the call list, which paints its whole table rather than filling it
/// with widgets.
pub fn paint(p: &egui::Painter, r: Rect, peak: f32) {
    // An empty meter still has to read as a meter. Drawn as a well with an
    // engraved edge and its two region marks, so a silent channel looks
    // silent rather than looking like a control that failed to appear.
    p.rect_filled(r, 1.0, theme::WELL);
    p.rect_stroke(r, 1.0, Stroke::new(1.0, theme::ETCH), egui::StrokeKind::Inside);

    let at = |v: f32| r.left() + scale(v) * r.width();
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
/// One strip rather than a slider with a bar beneath it: what you set and what
/// that is producing are read in the same glance, and the handle sits at the
/// point on the scale the level is being measured against.
pub fn fader(ui: &mut Ui, value: &mut f32, peak: f32, width: f32) -> Response {
    let w = width.min(ui.available_width()).max(40.0);
    let (rect, mut resp) = ui.allocate_exact_size(Vec2::new(w, FADER_H), Sense::click_and_drag());
    let (lo, hi) = (rect.left() + GRIP, rect.right() - GRIP);

    if resp.dragged() || resp.clicked() {
        if let Some(p) = ui.ctx().pointer_interact_pos() {
            let t = ((p.x - lo) / (hi - lo)).clamp(0.0, 1.0);
            if (t - *value).abs() > 1e-4 {
                *value = t;
                resp.mark_changed();
            }
        }
    }
    if !ui.is_rect_visible(rect) {
        return resp;
    }

    let p = ui.painter();
    paint(p, Rect::from_center_size(rect.center(), Vec2::new(rect.width(), H)), peak);

    // Amber, because the handle is the one part of this the operator set, and
    // outlined so it stays legible crossing a lit bar of any colour.
    let x = lo + value.clamp(0.0, 1.0) * (hi - lo);
    let handle =
        Rect::from_center_size(Pos2::new(x, rect.center().y), Vec2::new(GRIP * 2.0, rect.height()));
    p.rect_filled(handle, 1.0, theme::CHASSIS);
    p.rect_filled(
        handle.shrink(1.0),
        1.0,
        if resp.hovered() || resp.dragged() { theme::VALUE } else { theme::READOUT },
    );
    resp
}

fn scale(v: f32) -> f32 {
    v.clamp(0.0, 1.0).sqrt()
}
