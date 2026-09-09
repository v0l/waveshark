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
    /// Key the transmitter: a mast with waves off it.
    Transmit,
    /// The dataset cache: somebody else's files kept on this machine.
    Data,
    /// Open the page a dataset comes from, in a browser.
    Link,
    /// The views, one glyph each. They are tabs rather than a list, so each
    /// one has to be told apart from the other nine at 22 points: no two of
    /// them share a silhouette.
    Spectrum,
    Chain,
    Map,
    Calls,
    Messages,
    Video,
    Links,
    Devices,
    Satellite,
    Key,
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
                    // A cross where the sound would have come out, rather
                    // than a slash laid over the speaker. The slash version
                    // crosses the cone, and at fourteen points the two fills
                    // merge into one blob that reads as neither.
                    let (x0, x1) = (b.left() + w * 0.66, b.right());
                    let d = h * 0.17;
                    let x = Stroke::new(sw * 1.1, col);
                    p.line_segment([Pos2::new(x0, c.y - d), Pos2::new(x1, c.y + d)], x);
                    p.line_segment([Pos2::new(x0, c.y + d), Pos2::new(x1, c.y - d)], x);
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
            Icon::Transmit => {
                // A mast, a dot at its tip, and two arcs either side of the
                // tip: the mark on every PTT the trade has made.
                let tip = Pos2::new(c.x, b.top() + b.height() * 0.28);
                p.line_segment([tip, Pos2::new(c.x, b.bottom())], s);
                p.circle_filled(tip, sw * 0.9, col);
                for (k, rad) in [(0.28f32, 1.0f32), (0.46, 1.0)] {
                    let r = b.width() * k;
                    for side in [-1.0f32, 1.0] {
                        let pts: Vec<Pos2> = (0..=8)
                            .map(|i| {
                                let a = -0.9 + 1.8 * i as f32 / 8.0;
                                Pos2::new(tip.x + side * r * a.cos() * rad, tip.y - r * a.sin())
                            })
                            .collect();
                        p.add(egui::Shape::line(pts, s));
                    }
                }
            }
            Icon::Data => {
                // The stacked cylinder every database has been drawn as
                // since tape reels: a top ellipse, two sides, and two more
                // ellipses under it for the stack.
                let (rx, ry) = (b.width() * 0.42, b.height() * 0.14);
                let (top, bot) = (b.top() + ry + sw * 0.5, b.bottom() - ry - sw * 0.5);
                let ring = |y: f32| -> Vec<Pos2> {
                    (0..=28)
                        .map(|i| {
                            let a = std::f32::consts::TAU * i as f32 / 28.0;
                            Pos2::new(c.x + rx * a.cos(), y + ry * a.sin())
                        })
                        .collect()
                };
                p.add(egui::Shape::line(ring(top), s));
                for side in [-1.0f32, 1.0] {
                    let x = c.x + side * rx;
                    p.line_segment([Pos2::new(x, top), Pos2::new(x, bot)], s);
                }
                // Only the front halves of the lower rims: a whole ellipse
                // there reads as a second cylinder rather than a shelf.
                for k in [0.5f32, 1.0] {
                    let y = top + (bot - top) * k;
                    let pts: Vec<Pos2> = (0..=14)
                        .map(|i| {
                            let a = std::f32::consts::PI * i as f32 / 14.0;
                            Pos2::new(c.x + rx * a.cos(), y + ry * a.sin())
                        })
                        .collect();
                    p.add(egui::Shape::line(pts, s));
                }
            }
            Icon::Link => {
                // Two rounded links of a chain on a diagonal, the shape a
                // browser has meant by a link since it meant anything.
                let d = b.width() * 0.16;
                let len = b.width() * 0.30;
                for side in [-1.0f32, 1.0] {
                    let mid = Pos2::new(c.x + side * d, c.y - side * d);
                    let dir = Vec2::new(0.62, -0.62);
                    let a = mid - dir * len * 0.5;
                    let z = mid + dir * len * 0.5;
                    p.line_segment([a, z], s);
                    p.circle_stroke(z, sw * 0.9, s);
                }
                // The bar between them, which is what makes it a chain
                // rather than two ticks.
                p.line_segment(
                    [
                        Pos2::new(c.x - d * 0.7, c.y + d * 0.7),
                        Pos2::new(c.x + d * 0.7, c.y - d * 0.7),
                    ],
                    s,
                );
            }
            Icon::Spectrum => {
                // A trace with one signal standing out of the noise, which
                // is what the pane shows. Distinct from `Decode`'s three
                // bars because the two sit in the same bar.
                let n = 16;
                let pts: Vec<Pos2> = (0..=n)
                    .map(|i| {
                        let t = i as f32 / n as f32;
                        let x = b.left() + b.width() * t;
                        // A narrow peak at 0.55, on a floor that wobbles.
                        let d = (t - 0.55) / 0.09;
                        let peak = (-d * d).exp();
                        let floor = 0.12 * ((t * 27.0).sin() * 0.5 + 0.5);
                        let y = b.bottom() - b.height() * (0.1 + floor + 0.8 * peak);
                        Pos2::new(x, y)
                    })
                    .collect();
                p.add(egui::Shape::line(pts, Stroke::new(sw * 0.9, col)));
            }
            Icon::Chain => {
                // Two stages and a wire between them: the graph, drawn as
                // the chain view draws it.
                let h = b.height() * 0.34;
                let w = b.width() * 0.34;
                let left = Rect::from_min_size(
                    Pos2::new(b.left(), b.top() + b.height() * 0.08),
                    Vec2::new(w, h),
                );
                let right = Rect::from_min_size(
                    Pos2::new(b.right() - w, b.bottom() - h - b.height() * 0.08),
                    Vec2::new(w, h),
                );
                p.rect_stroke(left, 1.0, s, egui::StrokeKind::Middle);
                p.rect_stroke(right, 1.0, s, egui::StrokeKind::Middle);
                p.line_segment([left.right_center(), Pos2::new(right.left(), left.center().y)], s);
                p.line_segment(
                    [Pos2::new(right.left(), left.center().y), right.left_center()],
                    s,
                );
            }
            Icon::Map => {
                // The pin every map has dropped since maps were on screens.
                let head = Pos2::new(c.x, b.top() + b.height() * 0.34);
                let rad = b.width() * 0.27;
                p.circle_stroke(head, rad, s);
                for side in [-1.0f32, 1.0] {
                    p.line_segment(
                        [
                            Pos2::new(head.x + side * rad * 0.86, head.y + rad * 0.5),
                            Pos2::new(c.x, b.bottom()),
                        ],
                        s,
                    );
                }
            }
            Icon::Calls => {
                // A microphone: who is talking, not what is coming out of
                // the speaker, which is what `Sound` already means.
                let w = b.width() * 0.34;
                let cap = Rect::from_min_size(
                    Pos2::new(c.x - w * 0.5, b.top()),
                    Vec2::new(w, b.height() * 0.52),
                );
                p.rect_stroke(cap, w * 0.5, s, egui::StrokeKind::Middle);
                let cradle = b.height() * 0.28;
                let pts: Vec<Pos2> = (0..=10)
                    .map(|i| {
                        let a = std::f32::consts::PI * i as f32 / 10.0;
                        Pos2::new(c.x + cradle * a.cos(), cap.bottom() + cradle * a.sin() * 0.8)
                    })
                    .collect();
                p.add(egui::Shape::line(pts, s));
                p.line_segment([Pos2::new(c.x, cap.bottom() + cradle * 0.8), Pos2::new(c.x, b.bottom())], s);
            }
            Icon::Messages => {
                // A bubble with a tail. Lines inside it would close up at
                // this size, so the shape carries it alone.
                let body = Rect::from_min_max(
                    Pos2::new(b.left(), b.top() + b.height() * 0.08),
                    Pos2::new(b.right(), b.bottom() - b.height() * 0.3),
                );
                p.rect_stroke(body, b.width() * 0.18, s, egui::StrokeKind::Middle);
                p.add(egui::Shape::line(
                    vec![
                        Pos2::new(body.left() + body.width() * 0.24, body.bottom()),
                        Pos2::new(body.left() + body.width() * 0.18, b.bottom()),
                        Pos2::new(body.left() + body.width() * 0.52, body.bottom()),
                    ],
                    s,
                ));
            }
            Icon::Video => {
                // A screen on a stand. A film frame with sprocket holes is
                // the other convention and it fills in at this size.
                let screen = Rect::from_min_max(
                    Pos2::new(b.left(), b.top() + b.height() * 0.06),
                    Pos2::new(b.right(), b.bottom() - b.height() * 0.34),
                );
                p.rect_stroke(screen, 1.0, s, egui::StrokeKind::Middle);
                p.line_segment([Pos2::new(c.x, screen.bottom()), Pos2::new(c.x, b.bottom())], s);
                p.line_segment(
                    [
                        Pos2::new(b.left() + b.width() * 0.24, b.bottom()),
                        Pos2::new(b.right() - b.width() * 0.24, b.bottom()),
                    ],
                    s,
                );
            }
            Icon::Links => {
                // Two ends and the traffic between them: who is talking to
                // whom. The boxes of `Chain` are stages; these are parties.
                let rad = b.width() * 0.16;
                let (l, r) = (
                    Pos2::new(b.left() + rad, b.top() + rad),
                    Pos2::new(b.right() - rad, b.bottom() - rad),
                );
                p.circle_stroke(l, rad, s);
                p.circle_stroke(r, rad, s);
                let dir = (r - l).normalized();
                let (a, z) = (l + dir * rad * 1.4, r - dir * rad * 1.4);
                p.line_segment([a, z], s);
                // One arrowhead, so the line reads as a direction rather
                // than a rod.
                let back = -dir * b.width() * 0.16;
                let n = Vec2::new(-dir.y, dir.x) * b.width() * 0.1;
                p.line_segment([z, z + back + n], s);
                p.line_segment([z, z + back - n], s);
            }
            Icon::Devices => {
                // A handset: a body with a stub antenna, which is what the
                // survey is a list of.
                let w = b.width() * 0.52;
                let body = Rect::from_min_max(
                    Pos2::new(c.x - w * 0.5, b.top() + b.height() * 0.3),
                    Pos2::new(c.x + w * 0.5, b.bottom()),
                );
                p.rect_stroke(body, b.width() * 0.1, s, egui::StrokeKind::Middle);
                let ant = Pos2::new(body.right() - w * 0.22, body.top());
                p.line_segment([ant, Pos2::new(ant.x + b.width() * 0.12, b.top())], s);
                p.line_segment(
                    [
                        Pos2::new(body.left() + w * 0.22, body.center().y),
                        Pos2::new(body.right() - w * 0.22, body.center().y),
                    ],
                    s,
                );
            }
            Icon::Satellite => {
                // A dish looking up, with the pass it is following above it.
                // Two shapes that were tried first and do not survive 22
                // points: a spacecraft between two panels closes into a
                // dumbbell, and a tilted orbit ring around a dot reads as an
                // eye.
                let rim = b.width() * 0.46;
                let pivot = Pos2::new(c.x - b.width() * 0.06, c.y + b.height() * 0.2);
                // The dish, as an arc open towards the upper right.
                let dish: Vec<Pos2> = (0..=12)
                    .map(|i| {
                        let a = -1.75 + 1.9 * i as f32 / 12.0;
                        Pos2::new(pivot.x + rim * a.cos(), pivot.y + rim * a.sin())
                    })
                    .collect();
                p.add(egui::Shape::line(dish.clone(), s));
                p.line_segment([dish[0], dish[dish.len() - 1]], Stroke::new(sw * 0.8, col));
                // The mount, and the feed the dish points at.
                p.line_segment([pivot, Pos2::new(pivot.x, b.bottom())], s);
                p.line_segment(
                    [
                        Pos2::new(pivot.x - b.width() * 0.17, b.bottom()),
                        Pos2::new(pivot.x + b.width() * 0.17, b.bottom()),
                    ],
                    s,
                );
                p.circle_filled(
                    Pos2::new(b.right() - b.width() * 0.08, b.top() + b.height() * 0.08),
                    sw * 1.4,
                    col,
                );
            }
            Icon::Key => {
                // A key: a bow, a shaft and two teeth.
                let rad = b.width() * 0.22;
                let bow = Pos2::new(b.left() + rad, c.y);
                p.circle_stroke(bow, rad, s);
                p.line_segment([Pos2::new(bow.x + rad, c.y), Pos2::new(b.right(), c.y)], s);
                for at in [0.72f32, 0.92] {
                    let x = b.left() + b.width() * at;
                    p.line_segment([Pos2::new(x, c.y), Pos2::new(x, c.y + b.height() * 0.22)], s);
                }
            }
            Icon::Log => {
                // Rows with a mark against each, which is what the log is.
                for i in 0..3 {
                    let y = b.top() + b.height() * (0.15 + 0.35 * i as f32);
                    p.line_segment(
                        [Pos2::new(b.left(), y), Pos2::new(b.left() + b.width() * 0.18, y)],
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
    icon_button_sized(ui, icon, tip, enabled, selected, SIZE)
}

/// The same, at a size that fits a row of controls rather than the toolbar.
pub fn icon_button_sized(
    ui: &mut Ui,
    icon: Icon,
    tip: &str,
    enabled: bool,
    selected: bool,
    size: f32,
) -> Response {
    let (rect, mut resp) = ui.allocate_exact_size(
        Vec2::splat(size),
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

/// A tab in the view strip: an icon button with a dot in its corner when the
/// view behind it has something to show.
///
/// The dot is the whole point of a strip over a dropdown. Ten tabs are only
/// worth their width if the operator can see, without opening any of them,
/// which ones are holding traffic.
pub fn icon_tab(
    ui: &mut Ui,
    icon: Icon,
    tip: &str,
    selected: bool,
    live: bool,
    size: f32,
) -> Response {
    let (rect, mut resp) = ui.allocate_exact_size(Vec2::splat(size), Sense::click());
    let hovered = resp.hovered();
    if ui.is_rect_visible(rect) {
        let p = ui.painter();
        // The strip is already a well, so the selected button cannot be a
        // second well: it would be the same colour as its own background.
        // A raised face and a bar under it is what a tab has always been.
        if selected {
            p.rect_filled(rect, 3.0, theme::PANEL);
            let bar = Rect::from_min_max(
                Pos2::new(rect.left() + 2.0, rect.bottom() - 2.0),
                Pos2::new(rect.right() - 2.0, rect.bottom() - 0.5),
            );
            p.rect_filled(bar, 1.0, theme::READOUT);
        } else if hovered {
            p.rect_filled(rect, 3.0, theme::ETCH);
        }
        icon.paint(p, rect.translate(Vec2::new(0.0, -1.0)), tint(true, selected, hovered));
        // The dot is the whole point of a strip over a dropdown: it says
        // which views are holding traffic without opening any of them. Not
        // drawn on the open tab, where the traffic is already on screen.
        if live && !selected {
            let at = Pos2::new(rect.right() - size * 0.17, rect.top() + size * 0.17);
            p.circle_filled(at, (size * 0.09).max(1.5), theme::TRACE);
        }
    }
    resp = resp.on_hover_text(tip);
    resp.clone().on_hover_cursor(egui::CursorIcon::PointingHand);
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
