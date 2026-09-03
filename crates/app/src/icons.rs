//! Line-drawn icons for the top bar.
//!
//! Drawn with the painter rather than set from an icon font. A font is a
//! second asset to ship, a second thing to fall back from when a glyph is
//! missing, and it renders at whatever weight the font was designed for; these
//! are a dozen strokes each and match the panel's own line weight because they
//! use it.
//!
//! Every icon carries its label as hover text. An icon alone is a rebus, and
//! the label is what makes the first use of the app possible.

use crate::theme;
use egui::{Color32, Pos2, Rect, Response, Sense, Stroke, Ui, Vec2};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Icon {
    /// Start the radio.
    Play,
    Stop,
    /// The radio's own controls: gain, switches, corrections.
    Sliders,
    /// Application setup.
    Setup,
    /// Decode everything in the span.
    Decode,
    /// The packet log.
    Log,
    /// Audio on, and audio muted. Two icons rather than one lit differently:
    /// a mute control has to say which state it is in from across the desk,
    /// and colour alone does not carry that.
    Sound,
    Mute,
    /// Write the raw span to a file.
    Capture,
}

/// Side of the clickable square, in points.
///
/// Sized against the controls beside it rather than against the glyph: an
/// icon the size of a full stop is a smaller target than the text button it
/// replaced, which is a worse control however clean it looks.
pub const SIZE: f32 = 28.0;
/// Fraction of the square the glyph is inset by.
///
/// Measured against the screen rather than chosen: at the first inset the
/// drawing area was eleven points across, and a five-transition waveform in
/// eleven points is a filled rectangle.
const INSET: f32 = 0.2;

impl Icon {
    /// Draw the glyph inside `r`. Public so the panes can settle their corner
    /// affordance with the same shape the top bar uses: two drawings of the
    /// same idea is one of them being wrong.
    pub fn paint(self, p: &egui::Painter, r: Rect, col: Color32) {
        // Everything is drawn inside a box inset from the hit area, so
        // adjacent icons do not appear to touch.
        let b = r.shrink(r.width() * INSET);
        // Proportional so the whole set can be resized from `SIZE` alone. A
        // fixed weight makes a larger icon look hollow and a smaller one
        // makes it a blob.
        let sw = (b.width() * 0.115).max(1.5);
        let s = Stroke::new(sw, col);
        let c = b.center();
        match self {
            Icon::Play => {
                p.add(egui::Shape::convex_polygon(
                    vec![
                        Pos2::new(b.left() + b.width() * 0.1, b.top()),
                        Pos2::new(b.right(), c.y),
                        Pos2::new(b.left() + b.width() * 0.1, b.bottom()),
                    ],
                    col,
                    Stroke::NONE,
                ));
            }
            Icon::Stop => {
                p.rect_filled(b.shrink(b.width() * 0.06), 1.0, col);
            }
            Icon::Sliders => {
                // Two rails with a knob on each, the knobs at different
                // positions so it reads as a mixer rather than a list.
                for (i, at) in [0.62f32, 0.34].into_iter().enumerate() {
                    let y = b.top() + b.height() * (0.3 + 0.4 * i as f32);
                    p.line_segment([Pos2::new(b.left(), y), Pos2::new(b.right(), y)], s);
                    let x = b.left() + b.width() * at;
                    p.line_segment(
                        [Pos2::new(x, y - b.height() * 0.2), Pos2::new(x, y + b.height() * 0.2)],
                        Stroke::new(sw * 1.7, col),
                    );
                }
            }
            Icon::Setup => {
                // A hex nut, not a cogwheel.
                //
                // The cog is the default answer and it does not survive being
                // drawn small: six teeth on a ring at fourteen points is a
                // fuzzy circle, which is what it looked like on screen. A
                // slotted screw was the next try and reads as a no-entry sign,
                // because a bar across a ring is that sign. A hexagon has a
                // silhouette nothing else in this set shares, it holds its
                // shape down to a dozen points, and it belongs to the same
                // machined-panel world as the rest of the instrument.
                let rad = b.width() * 0.5;
                let pts: Vec<Pos2> = (0..6)
                    .map(|i| {
                        let a = (60.0 * i as f32 + 90.0).to_radians();
                        let (sn, cs) = a.sin_cos();
                        Pos2::new(c.x + cs * rad, c.y + sn * rad)
                    })
                    .collect();
                p.add(egui::Shape::closed_line(pts, s));
                p.circle_filled(c, rad * 0.22, col);
            }
            Icon::Decode => {
                // Signals standing in a span, which is what decoding the whole
                // span is about. A waveform was the first idea and it does not
                // survive being fourteen points wide: the transitions close up
                // and it reads as a solid block.
                let bar = Stroke::new(sw * 1.45, col);
                for (at, h) in [(0.08f32, 0.55f32), (0.5, 1.0), (0.92, 0.75)] {
                    let x = b.left() + b.width() * at;
                    p.line_segment(
                        [Pos2::new(x, b.bottom()), Pos2::new(x, b.bottom() - b.height() * h)],
                        bar,
                    );
                }
            }
            Icon::Sound | Icon::Mute => {
                // A speaker: a box and a cone. Drawn filled rather than
                // stroked because at fourteen points an outlined cone closes
                // up into a blob, and this shape has to be recognisable at
                // the size the strip uses.
                let w = b.width();
                let h = b.height();
                let body = Rect::from_min_max(
                    Pos2::new(b.left(), c.y - h * 0.18),
                    Pos2::new(b.left() + w * 0.3, c.y + h * 0.18),
                );
                p.rect_filled(body, 1.0, col);
                p.add(egui::Shape::convex_polygon(
                    vec![
                        Pos2::new(b.left() + w * 0.28, c.y - h * 0.18),
                        Pos2::new(b.left() + w * 0.6, b.top()),
                        Pos2::new(b.left() + w * 0.6, b.bottom()),
                        Pos2::new(b.left() + w * 0.28, c.y + h * 0.18),
                    ],
                    col,
                    Stroke::NONE,
                ));
                if self == Icon::Sound {
                    // Two arcs for sound coming out of it, as short strokes
                    // rather than curves: a curve this small is a smudge.
                    for (i, at) in [0.72f32, 0.9].into_iter().enumerate() {
                        let x = b.left() + w * at;
                        let dy = h * (0.16 + 0.12 * i as f32);
                        p.line_segment([Pos2::new(x, c.y - dy), Pos2::new(x, c.y + dy)], s);
                    }
                } else {
                    // The slash, which is what says muted at a glance.
                    p.line_segment(
                        [
                            Pos2::new(b.left() + w * 0.66, c.y - h * 0.3),
                            Pos2::new(b.right(), c.y + h * 0.3),
                        ],
                        Stroke::new(sw * 1.1, col),
                    );
                }
            }
            Icon::Capture => {
                // The recording dot every tape machine has had, with a ring
                // around it so an off state is still a shape rather than a
                // dim smudge.
                let rad = b.width() * 0.46;
                p.circle_stroke(c, rad, s);
                p.circle_filled(c, rad * 0.45, col);
            }
            Icon::Log => {
                // Rows with a mark against each, which is what the log is.
                for i in 0..3 {
                    let y = b.top() + b.height() * (0.15 + 0.35 * i as f32);
                    p.line_segment(
                        [
                            Pos2::new(b.left(), y),
                            Pos2::new(b.left() + b.width() * 0.18, y),
                        ],
                        s,
                    );
                    p.line_segment(
                        [Pos2::new(b.left() + b.width() * 0.34, y), Pos2::new(b.right(), y)],
                        s,
                    );
                }
            }
        }
    }
}

/// Colour for an icon in a given state.
///
/// Separated from the drawing so the choice can be checked without a painter,
/// and because getting it wrong is the failure that matters: an icon that
/// looks the same on and off is a switch with no readout.
pub fn tint(enabled: bool, selected: bool, hovered: bool) -> Color32 {
    if !enabled {
        theme::ETCH
    } else if selected || hovered {
        // Amber for both. It is the panel's one accent, and a control that
        // lights up white on hover and amber when on belongs to two different
        // instruments. The filled well behind a selected icon is what tells
        // the two states apart.
        theme::READOUT
    } else {
        theme::LEGEND
    }
}

/// An icon that behaves like a button, labelled by hover text.
pub fn icon_button(ui: &mut Ui, icon: Icon, tip: &str, enabled: bool, selected: bool) -> Response {
    let (rect, mut resp) = ui.allocate_exact_size(
        Vec2::splat(SIZE),
        if enabled { Sense::click() } else { Sense::hover() },
    );
    let hovered = resp.hovered();
    if ui.is_rect_visible(rect) {
        let p = ui.painter();
        if selected || (hovered && enabled) {
            p.rect_filled(rect, 3.0, if selected { theme::WELL } else { theme::ETCH });
        }
        icon.paint(p, rect, tint(enabled, selected, hovered));
    }
    resp = resp.on_hover_text(tip);
    if enabled {
        // The pointer has to say the thing is pressable; the icon alone does
        // not, having no border to read as a button.
        resp.clone().on_hover_cursor(egui::CursorIcon::PointingHand);
    }
    resp
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_disabled_icon_cannot_be_confused_with_an_active_one() {
        assert_eq!(tint(false, false, false), theme::ETCH);
        assert_eq!(tint(false, true, true), theme::ETCH, "disabled wins over every other state");
    }

    #[test]
    fn a_switch_that_is_on_reads_as_on() {
        // Amber is the panel's one accent, the colour the tuned frequency is
        // set in, and nothing that is merely available uses it.
        assert_eq!(tint(true, true, false), theme::READOUT);
        assert_ne!(tint(true, true, false), tint(true, false, false));
    }

    #[test]
    fn hovering_an_available_control_changes_it() {
        assert_ne!(tint(true, false, true), tint(true, false, false));
    }
}
