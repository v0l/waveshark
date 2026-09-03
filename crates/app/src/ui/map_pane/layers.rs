//! What the receiver draws on the map.
//!
//! One [`Layer`] per body of data, each built fresh around what it draws
//! from. The map holds the switches and the camera; these hold the knowledge
//! of what an airport or a track looks like, which is why they are here and
//! not in the widget.

use super::super::mapview::{Canvas, Layer};
use super::*;

/// Distance rings around the receiver.
///
/// Centred on the antenna, not on the view: they say how far away something
/// is from where you are listening, which does not change when the map is
/// dragged.
pub(super) struct RingLayer {
    pub home: Option<(f64, f64)>,
}

impl Layer for RingLayer {
    fn key(&self) -> &'static str {
        "rings"
    }

    fn label(&self) -> &'static str {
        "RINGS"
    }

    fn draw(&mut self, c: &Canvas) {
        let Some((lat, lon)) = self.home else { return };
        let at = c.at(lat, lon);
        // Sized at the antenna, not at the view: the rings are around a
        // place, and that place does not move when the map is dragged.
        let nm_px = c.nm_px_at(lat);
        // Rings at a round distance that fits the window, rather than a
        // fraction of a zoom: 25 nm is 25 nm at every scale.
        let span = f64::from(c.rect.width().min(c.rect.height())) / 2.0 / nm_px;
        let step = [1.0, 2.0, 5.0, 10.0, 25.0, 50.0, 100.0, 200.0, 500.0]
            .into_iter()
            .find(|s| s * 2.0 >= span)
            .unwrap_or(1000.0);
        for k in 1..=3 {
            let r = (step * f64::from(k) * nm_px) as f32;
            c.p.circle_stroke(at, r, Stroke::new(1.0, theme::READOUT.gamma_multiply(0.30)));
            c.p.text(
                Pos2::new(at.x + 4.0, at.y - r),
                Align2::LEFT_CENTER,
                format!("{:.0} nm", step * f64::from(k)),
                Canvas::font(),
                theme::READOUT.gamma_multiply(0.55),
            );
        }
    }
}

/// Where the receiver is. Not switchable: the station stays on the map
/// whatever is drawn around it, and a map of what you can hear with no mark
/// for where you are hearing it is a map of nothing in particular.
pub(super) struct StationLayer {
    pub home: Option<(f64, f64)>,
}

impl Layer for StationLayer {
    fn key(&self) -> &'static str {
        "station"
    }

    fn switchable(&self) -> bool {
        false
    }

    fn draw(&mut self, c: &Canvas) {
        let Some((lat, lon)) = self.home else { return };
        let at = c.at(lat, lon);
        c.p.circle_stroke(at, 5.0, Stroke::new(1.5, theme::READOUT));
        c.p.circle_filled(at, 1.5, theme::READOUT);
    }
}

/// Airports, in the map's amber so a fixed facility is not mistaken for the
/// cyan of something in the air, with the frequency card for whichever one is
/// hovered.
#[derive(Default)]
pub(super) struct AirportLayer {
    /// The markers drawn this frame, kept so the pointer can be hit-tested
    /// against them once every layer has drawn.
    shown: Vec<(Pos2, &'static datasets::airports::Airport)>,
}

impl Layer for AirportLayer {
    fn key(&self) -> &'static str {
        "airports"
    }

    fn label(&self) -> &'static str {
        "AIRPORTS"
    }

    fn draw(&mut self, c: &Canvas) {
        self.shown = draw_airports(c);
    }

    /// The card is drawn after every layer, so it sits over the tiles and the
    /// aircraft instead of vanishing behind them.
    fn over(&mut self, c: &Canvas) {
        if c.zoom() < crate::data::SHOW_ZOOM {
            return;
        }
        if let Some(pos) = c.hover() {
            if let Some((at, a)) = hovered_airport(&self.shown, pos) {
                airport_card(&c.p, c.rect, at, a);
            }
        }
    }

    fn status(&self) -> Option<String> {
        (!self.shown.is_empty()).then(|| format!("{} airports", self.shown.len()))
    }
}

/// Aircraft, vessels and stations, with the trail each one came along.
pub(super) struct TrackLayer<'a> {
    pub active: &'a [&'a crate::tracks::Track],
    pub now: std::time::Instant,
}

impl Layer for TrackLayer<'_> {
    fn key(&self) -> &'static str {
        "tracks"
    }

    fn label(&self) -> &'static str {
        "TRACKS"
    }

    fn draw(&mut self, c: &Canvas) {
        for a in self.active {
            let Some((lat, lon)) = a.position else { continue };
            // Faded by age against its own kind's memory: a minute of silence
            // means an aircraft is gone and means nothing at all for a vessel,
            // so fading both on the same clock would grey out half the
            // shipping while it was still there.
            let stale = a.kind().forget().as_secs_f32();
            let fade = 1.0 - (a.age(self.now).as_secs_f32() / stale).clamp(0.0, 0.75);
            // Drawn in segments, brightening towards the track: a line of one
            // colour says nothing about which end of it is now, and over map
            // tiles a thin one is lost in the roads.
            if a.trail.len() > 1 {
                let pts: Vec<Pos2> = a.trail.iter().map(|(la, lo)| c.at(*la, *lo)).collect();
                let n = pts.len() as f32;
                for (k, seg) in pts.windows(2).enumerate() {
                    let along = (k as f32 + 1.0) / n;
                    c.p.line_segment(
                        [seg[0], seg[1]],
                        Stroke::new(
                            2.5,
                            theme::TRACE.gamma_multiply((0.25 + 0.65 * along) * fade),
                        ),
                    );
                }
            }
            let at = c.at(lat, lon);
            let col = theme::TRACE.gamma_multiply(fade);
            // An unconfirmed position came from one ADS-B frame read against
            // the receiver, which is right for anything in ordinary range and
            // a whole zone out beyond it. Drawn hollow so it does not claim
            // more than it knows. Nothing else can be unconfirmed: an AIS
            // position is absolute.
            if a.confirmed {
                track_mark(&c.p, at, a.kind(), a.course_deg, col);
            } else {
                c.p.circle_stroke(at, 3.5, Stroke::new(1.0, col.gamma_multiply(0.7)));
            }
            let label = a.label.clone().unwrap_or_else(|| a.id.text());
            c.label(Pos2::new(at.x + 9.0, at.y - 5.0), &label, theme::VALUE, fade);
            // The second line is whatever that kind is measured by: an
            // aircraft by its altitude, a vessel by its speed. A station is
            // fixed and has neither.
            let under = match a.kind() {
                crate::tracks::Kind::Aircraft => a.altitude_ft().map(|ft| format!("{ft} ft")),
                crate::tracks::Kind::Vessel | crate::tracks::Kind::Vehicle => {
                    a.speed_kt.filter(|v| *v > 0.0).map(|kt| format!("{kt:.0} kt"))
                }
                crate::tracks::Kind::Station => None,
            };
            if let Some(t) = under {
                c.label(Pos2::new(at.x + 9.0, at.y + 5.0), &t, theme::LEGEND, fade);
            }
        }
    }

    fn status(&self) -> Option<String> {
        let n = self.active.iter().filter(|a| a.position.is_some()).count();
        Some(format!("{n} plotted"))
    }
}

/// Airport markers and their ident labels.
///
/// Returns the on-screen markers, so the pointer can be hit-tested against
/// them for the frequency card. Airports appear only once the map is zoomed
/// in past [`crate::data::SHOW_ZOOM`]; at the default wide view a marker is a
/// blob under the traffic, and the range rings already say where the
/// interesting things are.
fn draw_airports(c: &Canvas) -> Vec<(Pos2, &'static datasets::airports::Airport)> {
    if c.zoom() < crate::data::SHOW_ZOOM {
        return Vec::new();
    }
    // Cull to the window, plus room for a label hanging over an edge.
    let near = c.rect.expand(30.0);
    let mut shown: Vec<(&'static datasets::airports::Airport, Pos2)> = crate::data::airports()
        .iter()
        .filter_map(|a| {
            let at = c.at(a.lat, a.lon);
            near.contains(at).then_some((a, at))
        })
        .collect();
    for (a, at) in &shown {
        let (r, bright) = match a.kind {
            datasets::airports::Kind::Large => (4.5, 1.0),
            datasets::airports::Kind::Medium => (3.5, 0.85),
            datasets::airports::Kind::Small => (2.6, 0.7),
        };
        let col = theme::READOUT.gamma_multiply(bright);
        c.p.circle_filled(*at, r, col);
        c.p.circle_stroke(*at, r + 1.0, Stroke::new(1.0, col.gamma_multiply(0.55)));
    }
    // Ident labels appear as the map zooms in, large airports first, and
    // drop where they would cover one already drawn: a city full of
    // strips must not become a wall of text. Larger first so a big field
    // wins its label against smaller neighbours.
    shown.sort_by_key(|(a, _)| match a.kind {
        datasets::airports::Kind::Large => 0,
        datasets::airports::Kind::Medium => 1,
        datasets::airports::Kind::Small => 2,
    });
    let mut labels: Vec<Rect> = Vec::new();
    for (a, at) in &shown {
        let at_zoom = match a.kind {
            datasets::airports::Kind::Large => 9.0,
            datasets::airports::Kind::Medium => 10.0,
            datasets::airports::Kind::Small => 11.0,
        };
        if c.zoom() < at_zoom {
            continue;
        }
        let at = Pos2::new(at.x + 8.0, at.y - 5.0);
        let r = c.label_rect(at, &a.ident, theme::VALUE, 1.0);
        if labels.iter().any(|l| l.intersects(r.expand(3.0))) {
            continue;
        }
        labels.push(r);
        c.label(at, &a.ident, theme::VALUE, 1.0);
    }
    shown.into_iter().map(|(a, at)| (at, a)).collect()
}

/// The airport nearest the pointer within a marker's grabbing distance, if
/// any, with where its marker sits so the card can be anchored to it. The
/// card belongs on a hover, so the threshold is a small screen distance
/// rather than a whole map.
fn hovered_airport<'a>(
    shown: &[(Pos2, &'a datasets::airports::Airport)],
    pos: Pos2,
) -> Option<(Pos2, &'a datasets::airports::Airport)> {
    const PX: f32 = 12.0;
    let mut best: Option<(f32, Pos2, &'a datasets::airports::Airport)> = None;
    for (at, a) in shown {
        let d = at.distance(pos);
        if d <= PX && best.is_none_or(|(b, _, _)| d < b) {
            best = Some((d, *at, a));
        }
    }
    best.map(|(_, at, a)| (at, a))
}

/// The frequency card shown when an airport is hovered: name, code,
/// elevation and the air traffic frequencies, primary ones first.
///
/// Drawn by hand rather than as an egui tooltip so it stays in the map's
/// language and is clipped with the pane, and so it does not depend on the
/// tooltip API changing under us.
fn airport_card(p: &egui::Painter, rect: Rect, anchor: Pos2, a: &datasets::airports::Airport) {
    // At most this many frequency rows before the card says how many it
    // is not showing. A field with thirty listed frequencies would
    // otherwise cover the map it is annotating.
    const MAX_ROWS: usize = 10;
    let (pad, sep, rule_gap) = (8.0, 4.0, 6.0);
    let font = |sz: f32| FontId::new(sz, FontFamily::Name(theme::READOUT_FONT.into()));

    // The name wraps rather than setting the card's width: "Charles de
    // Gaulle International Airport" is wider than anything else on the
    // card and would drag the whole thing across the map.
    let text_max = (f64::from(rect.width()) - 2.0 * f64::from(pad) - 8.0).clamp(80.0, 240.0) as f32;
    let name = p.layout(a.name.clone(), font(13.0), theme::VALUE, text_max);
    let class = match a.kind {
        datasets::airports::Kind::Large => "LARGE",
        datasets::airports::Kind::Medium => "MEDIUM",
        datasets::airports::Kind::Small => "SMALL",
    };
    let mut meta = format!("{class} AIRPORT");
    if let Some(el) = a.elev_ft {
        meta.push_str(&format!("   {el} FT"));
    }
    let meta = p.layout_no_wrap(meta, font(9.0), theme::LEGEND);
    let ident = p.layout_no_wrap(a.ident.clone(), font(11.0), theme::READOUT);

    // Every row below the rule, built once and then both measured and
    // drawn from this list. A row that is drawn without being measured is
    // a row that hangs off the bottom of the card.
    let mut rows: Vec<(std::sync::Arc<egui::Galley>, Color32)> = Vec::new();
    if a.freqs.is_empty() {
        let g = p.layout_no_wrap(
            "no published frequencies".to_string(),
            font(10.0),
            theme::LEGEND,
        );
        rows.push((g, theme::LEGEND));
    } else {
        for f in a.freqs.iter().take(MAX_ROWS) {
            let label = if f.kind == datasets::airports::FreqKind::Other {
                f.desc.as_str()
            } else {
                f.kind.label()
            };
            // The role is padded to a fixed width so the numbers line up
            // down the column, and truncated rather than let a long
            // description push the frequency off the card.
            let label: String = label.chars().take(16).collect();
            let g = p.layout_no_wrap(
                format!("{label:<16}{}", datasets::airports::fmt_mhz(f.mhz)),
                font(11.0),
                theme::VALUE,
            );
            rows.push((g, theme::VALUE));
        }
        if a.freqs.len() > MAX_ROWS {
            let g = p.layout_no_wrap(
                format!("+{} more", a.freqs.len() - MAX_ROWS),
                font(10.0),
                theme::LEGEND,
            );
            rows.push((g, theme::LEGEND));
        }
    }

    let head = [
        (name, theme::VALUE),
        (meta, theme::LEGEND),
        (ident, theme::READOUT),
    ];
    let head_sizes: Vec<Vec2> = head.iter().map(|(g, _)| g.size()).collect();
    let row_sizes: Vec<Vec2> = rows.iter().map(|(g, _)| g.size()).collect();
    let l = card_layout(&head_sizes, &row_sizes, pad, sep, rule_gap);

    // Beside the marker, or to its left when that would run off the right
    // edge, and clamped so the card stays on the map. The upper bound is
    // held above the lower one because a card wider than the pane would
    // otherwise clamp with a reversed range.
    let mut at = anchor + Vec2::new(14.0, -8.0);
    if anchor.x + 14.0 + l.size.x > rect.right() - 4.0 {
        at.x = anchor.x - 14.0 - l.size.x;
    }
    let max_x = (rect.right() - 4.0 - l.size.x).max(rect.left() + 4.0);
    let max_y = (rect.bottom() - 4.0 - l.size.y).max(rect.top() + 4.0);
    at.x = at.x.clamp(rect.left() + 4.0, max_x);
    at.y = at.y.clamp(rect.top() + 4.0, max_y);
    let card = Rect::from_min_size(at, l.size);
    p.rect_filled(card, 3.0, theme::PANEL);
    p.rect_stroke(card, 3.0, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);

    // Drawn entirely from the measured layout, so a line cannot be placed
    // somewhere the card was never sized for.
    let x = card.left() + l.text_x;
    for ((g, col), y) in head.iter().chain(rows.iter()).zip(&l.ys) {
        p.galley(Pos2::new(x, card.top() + y), g.clone(), *col);
    }
    let rule_y = card.top() + l.rule_y;
    p.line_segment(
        [Pos2::new(x, rule_y), Pos2::new(card.right() - pad, rule_y)],
        Stroke::new(1.0, theme::ETCH),
    );
}

/// A mark pointing where the track is going, shaped by what it is.
///
/// The shapes have to be tellable apart at a glance and at a few pixels,
/// because a busy estuary puts aircraft and shipping on the same screen.
/// An aircraft is a swept arrowhead, a vessel a longer hull with a bow,
/// and a station a fixed diamond that does not point anywhere because it
/// is not going anywhere.
fn track_mark(
    p: &egui::Painter,
    at: Pos2,
    kind: crate::tracks::Kind,
    course_deg: Option<f64>,
    col: Color32,
) {
    use crate::tracks::Kind;
    if kind == Kind::Station {
        let d = 4.0;
        p.add(egui::Shape::convex_polygon(
            vec![
                Pos2::new(at.x, at.y - d),
                Pos2::new(at.x + d, at.y),
                Pos2::new(at.x, at.y + d),
                Pos2::new(at.x - d, at.y),
            ],
            Color32::TRANSPARENT,
            Stroke::new(1.5, col),
        ));
        return;
    }
    let Some(track) = course_deg else {
        p.circle_filled(at, 3.0, col);
        return;
    };
    let t = (track as f32).to_radians();
    let (s, c) = (t.sin(), t.cos());
    // Course is clockwise from north, and north is up, so a point ahead of
    // the track is (sin, -cos) in screen coordinates.
    let rot = |x: f32, y: f32| Pos2::new(at.x + x * c + y * s, at.y + x * s - y * c);
    let shape = match kind {
        Kind::Aircraft => {
            vec![rot(0.0, 6.0), rot(-4.0, -4.0), rot(0.0, -1.5), rot(4.0, -4.0)]
        }
        // Longer and narrower, with a squared stern: a hull rather than a
        // wing.
        Kind::Vessel => vec![
            rot(0.0, 7.0),
            rot(-2.5, 2.0),
            rot(-2.5, -5.0),
            rot(2.5, -5.0),
            rot(2.5, 2.0),
        ],
        // Short and blunt, which is neither of the other two at a glance.
        _ => vec![rot(0.0, 4.5), rot(-3.0, 1.0), rot(-3.0, -3.0), rot(3.0, -3.0), rot(3.0, 1.0)],
    };
    p.add(egui::Shape::convex_polygon(shape, col, Stroke::NONE));
}


/// Where each line of the airport card sits, and how big the card has to be
/// to hold them all.
struct CardLayout {
    size: Vec2,
    /// Top of each line from the card's top edge: the head lines, then the
    /// rows below the rule, in the order they are drawn.
    ys: Vec<f32>,
    rule_y: f32,
    text_x: f32,
}

/// Lay the card out from the measured size of every line it will draw.
///
/// Pure arithmetic, and separate from the drawing, because the two ways this
/// went wrong were both a line drawn that the size had not accounted for: the
/// width left no room for the margin the text was drawn at, and the "+N more"
/// row was painted below a card measured without it. Measuring and drawing
/// from one list is what stops that, and it can be checked without a font.
fn card_layout(head: &[Vec2], rows: &[Vec2], pad: f32, sep: f32, rule_gap: f32) -> CardLayout {
    let text_w = head
        .iter()
        .chain(rows)
        .map(|s| s.x)
        .fold(0.0f32, f32::max);
    let mut ys = Vec::with_capacity(head.len() + rows.len());
    let mut y = pad;
    for (i, s) in head.iter().enumerate() {
        if i > 0 {
            y += sep;
        }
        ys.push(y);
        y += s.y;
    }
    y += rule_gap;
    let rule_y = y;
    y += 1.0 + rule_gap;
    for (i, s) in rows.iter().enumerate() {
        if i > 0 {
            y += sep;
        }
        ys.push(y);
        y += s.y;
    }
    CardLayout {
        size: Vec2::new(text_w + pad * 2.0, y + pad),
        ys,
        rule_y,
        text_x: pad,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The airport card must be big enough for every line it draws.
    ///
    /// It twice was not: the width was the widest line with no room for the
    /// margin the text is drawn at, so every row ran over the right edge, and
    /// the "+N more" row was drawn below a card measured without it. Both are
    /// a line outside the box, so that is what this checks.
    #[test]
    fn the_airport_card_holds_every_line_it_draws() {
        let (pad, sep, rule_gap) = (8.0, 4.0, 6.0);
        let head = [
            Vec2::new(180.0, 15.0),
            Vec2::new(90.0, 11.0),
            Vec2::new(40.0, 13.0),
        ];
        // Eleven rows: ten frequencies and the "+N more" that follows them.
        let rows: Vec<Vec2> = (0..11).map(|_| Vec2::new(150.0, 13.0)).collect();
        let l = card_layout(&head, &rows, pad, sep, rule_gap);

        for (i, s) in head.iter().chain(rows.iter()).enumerate() {
            let right = l.text_x + s.x;
            assert!(
                right <= l.size.x - pad + 1e-3,
                "line {i} ends at {right}, past the {} the card is wide",
                l.size.x
            );
            let bottom = l.ys[i] + s.y;
            assert!(
                bottom <= l.size.y - pad + 1e-3,
                "line {i} ends at {bottom}, past the {} the card is tall",
                l.size.y
            );
        }
        // The rule sits between the head and the rows, not on top of either.
        assert!(l.rule_y > l.ys[head.len() - 1]);
        assert!(l.rule_y < l.ys[head.len()]);
    }
}
