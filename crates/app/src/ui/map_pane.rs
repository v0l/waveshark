//! The map: tiles, aircraft and vessels from the tracker, airports, and the
//! table beside them.

use super::state::MapState;
use super::*;

/// The map, over where it is looking and what is on it.
pub(super) struct Map<'a> {
    pub st: &'a mut MapState,
    /// Where the receiver is, when it has been told.
    pub home: Option<(f64, f64)>,
    /// The position being typed, while it is being typed. Kept apart from the
    /// real one so a half-finished latitude does not move the map.
    pub edit: &'a mut Option<String>,
}

impl Map<'_> {
    /// Everything the tracker in the graph is holding: aircraft from ADS-B,
    /// vessels and navigation marks from AIS.
    ///
    /// Read from the receiver rather than assembled here: the tracker is a
    /// node fed by the bus, so it sees every frame rather than the ones still
    /// in the on-screen packet list.
    ///
    /// Returns a position the operator dropped or typed, for the caller to
    /// tell the receiver about.
    pub(super) fn show(self, ui: &mut egui::Ui) -> Option<(f64, f64)> {
        let now = std::time::Instant::now();
        // The pane runs to the window edge, and a table that starts there is
        // unreadable.
        let margin = egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 8));
        self.st.tiles.poll(ui.ctx());
        let mut view = self.st.view;
        let mut place = None;
        let mut edit = self.edit.take();
        {
            let tiles = &mut self.st.tiles;
            let active: Vec<&crate::tracks::Track> = self.st.tracks.iter().collect();
            let home = self.home;
            let body = |ui: &mut egui::Ui| {
                place = Self::station_row(ui, home, &mut edit);
                ui.add_space(6.0);
                // Half the pane each, roughly: the map is the view worth
                // having and the table is what you read once something on it
                // is interesting.
                let h = (ui.available_height() * 0.55).clamp(160.0, 1200.0);
                let (v, dropped) = Self::map_pane(ui, tiles, &active, now, home, view, h);
                view = v;
                place = place.or(dropped);
                ui.add_space(10.0);
                Self::track_rows(ui, &active, now);
            };
            margin.show(ui, body);
        }
        self.st.view = view;
        *self.edit = edit;
        if place.is_some() {
            *self.edit = None;
        }
        place
    }

    /// The station position, shown and editable.
    ///
    /// Worth a control rather than only a command line flag: it is what makes
    /// a single position frame resolve instead of waiting for a matching
    /// pair, and it is the point the range rings are drawn around. Anything
    /// within a couple of hundred miles of the truth does the job.
    pub(super) fn station_row(
        ui: &mut egui::Ui,
        home: Option<(f64, f64)>,
        edit: &mut Option<String>,
    ) -> Option<(f64, f64)> {
        let mut set = None;
        ui.horizontal(|ui| {
            ui.label(legend("station"));
            let text = edit.get_or_insert_with(|| match home {
                Some((lat, lon)) => format!("{lat:.4}, {lon:.4}"),
                None => String::new(),
            });
            let r = ui.add(
                egui::TextEdit::singleline(text)
                    .desired_width(150.0)
                    .hint_text("lat, lon")
                    .font(FontId::new(12.0, FontFamily::Name(theme::READOUT_FONT.into()))),
            );
            let typed = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if typed || ui.small_button("SET").clicked() {
                if let Ok(p) = crate::parse_location(text) {
                    set = Some(p);
                }
            }
            ui.add_space(8.0);
            ui.label(legend(if home.is_some() {
                "right-click the map to move it"
            } else {
                "type it, or right-click the map"
            }));
        });
        set
    }

    /// Where the tracks are, on OpenStreetMap tiles.
    ///
    /// The tiles come from our own fetcher rather than a map crate: slippy
    /// tiles are a URL template and a Mercator projection, and what a map
    /// widget adds on top is a way to draw things over them, which is the
    /// part this view has to write anyway.
    ///
    /// Returns the view to use next frame, since dragging and scrolling
    /// change it, and a position if the station was dropped somewhere.
    fn map_pane(
        ui: &mut egui::Ui,
        tiles: &mut crate::map::Tiles,
        active: &[&crate::tracks::Track],
        now: std::time::Instant,
        home: Option<(f64, f64)>,
        view: MapView,
        height: f32,
    ) -> (MapView, Option<(f64, f64)>) {
        let w = ui.available_width();
        let (rect, resp) = ui.allocate_exact_size(Vec2::new(w, height), Sense::click_and_drag());
        let p = ui.painter_at(rect);
        p.rect_filled(rect, 2.0, theme::WELL);
        let font = FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into()));

        // Centre on the receiver, or on what it can hear, until someone drags
        // the map somewhere else. After that it stays where it was put: a map
        // that recentres itself is a map you cannot read.
        let mut view = view;
        if view.center.is_none() {
            view.center = home.or_else(|| mean_position(active));
        }
        let Some((clat, clon)) = view.center else {
            p.rect_stroke(rect, 2.0, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);
            p.text(rect.center(), Align2::CENTER_CENTER, "no positions yet", font, theme::LEGEND);
            return (view, None);
        };

        let mid = rect.center();
        let mut center = (clat, clon);
        let offset = |pos: Pos2| (f64::from(pos.x - mid.x), f64::from(pos.y - mid.y));

        if resp.dragged() {
            let d = resp.drag_delta();
            center = crate::map::screen_to_ll(
                center,
                view.zoom,
                (f64::from(-d.x), f64::from(-d.y)),
            );
        }
        if let (true, Some(pos)) = (resp.hovered(), resp.hover_pos()) {
            let d = ui.input(|i| i.smooth_scroll_delta.y);
            if d != 0.0 {
                let next = (view.zoom + f64::from(d) * 0.004).clamp(2.0, 19.0);
                center = crate::map::anchored_zoom(center, view.zoom, next, offset(pos));
                view.zoom = next;
            }
        }
        view.center = Some(center);

        let (z, scale) = (crate::map::level(view.zoom), crate::map::tile_scale(view.zoom));
        let (cx, cy) = crate::map::project(center.0, center.1, z);
        let to_screen = |lat: f64, lon: f64| {
            let (x, y) = crate::map::ll_to_screen(center, view.zoom, (lat, lon));
            Pos2::new(mid.x + x as f32, mid.y + y as f32)
        };

        let clip = p.with_clip_rect(rect);
        Self::draw_tiles(&clip, tiles, rect, mid, (cx, cy), z, scale);

        // Range rings are centred on the receiver, not on the view. They say
        // how far away something is from the antenna, which does not change
        // when the map is dragged.
        let m_px = crate::map::resolution(center.0, z) * crate::map::TILE_PX / scale;
        let nm_px = 1852.0 / m_px;
        if let Some((lat, lon)) = home {
            let at = to_screen(lat, lon);
            // Rings at a round distance that fits the window, rather than a
            // fraction of a zoom: 25 nm is 25 nm at every scale.
            let span = f64::from(rect.width().min(rect.height())) / 2.0 / nm_px;
            let step = [1.0, 2.0, 5.0, 10.0, 25.0, 50.0, 100.0, 200.0, 500.0]
                .into_iter()
                .find(|s| s * 2.0 >= span)
                .unwrap_or(1000.0);
            for k in 1..=3 {
                let r = (step * f64::from(k) * nm_px) as f32;
                clip.circle_stroke(at, r, Stroke::new(1.0, theme::READOUT.gamma_multiply(0.30)));
                clip.text(
                    Pos2::new(at.x + 4.0, at.y - r),
                    Align2::LEFT_CENTER,
                    format!("{:.0} nm", step * f64::from(k)),
                    font.clone(),
                    theme::READOUT.gamma_multiply(0.55),
                );
            }
            clip.circle_stroke(at, 5.0, Stroke::new(1.5, theme::READOUT));
            clip.circle_filled(at, 1.5, theme::READOUT);
        }

        // Airports are fixed things under the traffic: drawn behind it so the
        // cyan of what is moving stays the brightest thing on screen.
        let airports_shown = Self::draw_airports(&clip, rect, view.zoom, to_screen);

        for a in active {
            let Some((lat, lon)) = a.position else { continue };
            // Faded by age against its own kind's memory: a minute of silence
            // means an aircraft is gone and means nothing at all for a vessel,
            // so fading both on the same clock would grey out half the
            // shipping while it was still there.
            let stale = a.kind().forget().as_secs_f32();
            let fade = 1.0 - (a.age(now).as_secs_f32() / stale).clamp(0.0, 0.75);
            // Drawn in segments, brightening towards the track: a line of one
            // colour says nothing about which end of it is now, and over map
            // tiles a thin one is lost in the roads.
            if a.trail.len() > 1 {
                let pts: Vec<Pos2> = a.trail.iter().map(|(la, lo)| to_screen(*la, *lo)).collect();
                let n = pts.len() as f32;
                for (k, seg) in pts.windows(2).enumerate() {
                    let along = (k as f32 + 1.0) / n;
                    clip.line_segment(
                        [seg[0], seg[1]],
                        Stroke::new(
                            2.5,
                            theme::TRACE.gamma_multiply((0.25 + 0.65 * along) * fade),
                        ),
                    );
                }
            }
            let at = to_screen(lat, lon);
            let col = theme::TRACE.gamma_multiply(fade);
            // An unconfirmed position came from one ADS-B frame read against
            // the receiver, which is right for anything in ordinary range and
            // a whole zone out beyond it. Drawn hollow so it does not claim
            // more than it knows. Nothing else can be unconfirmed: an AIS
            // position is absolute.
            if a.confirmed {
                Self::track_mark(&clip, at, a.kind(), a.course_deg, col);
            } else {
                clip.circle_stroke(at, 3.5, Stroke::new(1.0, col.gamma_multiply(0.7)));
            }
            let label = a.label.clone().unwrap_or_else(|| a.id.text());
            Self::map_label(&clip, Pos2::new(at.x + 9.0, at.y - 5.0), &label, theme::VALUE, fade);
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
                Self::map_label(&clip, Pos2::new(at.x + 9.0, at.y + 5.0), &t, theme::LEGEND, fade);
            }
        }

        // Hover over an airport to read its frequencies. The card is drawn on
        // the map painter after everything else, so it sits over the tiles and
        // the aircraft instead of vanishing behind them.
        if view.zoom >= crate::data::SHOW_ZOOM && !resp.dragged() {
            if let Some(pos) = resp.hover_pos() {
                if let Some((at, a)) = Self::hovered_airport(&airports_shown, pos) {
                    Self::airport_card(&clip, rect, at, a);
                }
            }
        }

        let shown = active.iter().filter(|a| a.position.is_some()).count();
        let mut status = format!(
            "{shown} plotted    {:.1} nm/cm    z{:.1}    drag to pan, scroll to zoom",
            nm_px.recip() * 37.8,
            view.zoom
        );
        if !airports_shown.is_empty() {
            status.push_str(&format!("    {} airports", airports_shown.len()));
        }
        Self::map_label(
            &clip,
            Pos2::new(rect.left() + 8.0, rect.top() + 10.0),
            &status,
            theme::LEGEND,
            1.0,
        );
        // Required by the tile usage policy, and by the licence the map data
        // is under.
        Self::map_label(
            &clip,
            Pos2::new(rect.right() - 150.0, rect.bottom() - 10.0),
            "(c) OpenStreetMap contributors",
            theme::LEGEND,
            1.0,
        );

        // A map with no tiles under it still shows tracks, and would quietly
        // look like empty sky and empty sea. Say what failed instead.
        if let Some((err, n)) = tiles.error() {
            let short: String = err.chars().take(110).collect();
            clip.rect_filled(
                Rect::from_min_max(
                    Pos2::new(rect.left(), rect.bottom() - 30.0),
                    Pos2::new(rect.right(), rect.bottom()),
                ),
                0.0,
                theme::WELL,
            );
            Self::map_label(
                &clip,
                Pos2::new(rect.left() + 8.0, rect.bottom() - 20.0),
                &format!("{n} tile(s) failed, map is not showing terrain"),
                theme::FAULT,
                1.0,
            );
            Self::map_label(
                &clip,
                Pos2::new(rect.left() + 8.0, rect.bottom() - 8.0),
                &short,
                theme::LEGEND,
                1.0,
            );
        }
        p.rect_stroke(rect, 2.0, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);

        // Right-click puts the station where the pointer is. A receiver knows
        // where it is on a map long before it knows its coordinates.
        let mut place = None;
        if resp.secondary_clicked() {
            if let Some(pos) = resp.interact_pointer_pos() {
                place = Some(crate::map::screen_to_ll(center, view.zoom, offset(pos)));
            }
        }
        (view, place)
    }

    /// Every tile the view touches, asked for as it is drawn.
    fn draw_tiles(
        p: &egui::Painter,
        tiles: &mut crate::map::Tiles,
        rect: Rect,
        mid: Pos2,
        center: (f64, f64),
        z: u8,
        scale: f64,
    ) {
        let (cx, cy) = center;
        let half_w = f64::from(rect.width()) / 2.0 / scale;
        let half_h = f64::from(rect.height()) / 2.0 / scale;
        let n = 1i64 << z;
        let (x0, x1) = ((cx - half_w).floor() as i64, (cx + half_w).floor() as i64);
        let (y0, y1) = ((cy - half_h).floor() as i64, (cy + half_h).floor() as i64);
        for ty in y0..=y1 {
            // The world does not wrap north to south, so a tile above the
            // pole is nothing rather than a tile from the other end.
            if ty < 0 || ty >= n {
                continue;
            }
            for tx in x0..=x1 {
                // Longitude does wrap, so panning past the date line shows
                // the far side of the world rather than a hole.
                let wrapped = tx.rem_euclid(n);
                let id = crate::map::TileId { z, x: wrapped as u32, y: ty as u32 };
                let Some(tex) = tiles.get(id) else { continue };
                let min = Pos2::new(
                    mid.x + ((tx as f64 - cx) * scale) as f32,
                    mid.y + ((ty as f64 - cy) * scale) as f32,
                );
                let at = Rect::from_min_size(min, Vec2::splat(scale as f32));
                p.image(
                    tex.id(),
                    at,
                    Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
                    // Held back so the tracks over it stay the brightest
                    // thing on screen, and so it sits in the panel's palette
                    // rather than glowing white in a dark interface.
                    Color32::from_gray(150),
                );
            }
        }
    }

    /// The backing box a map label would occupy, before it is drawn. Kept
    /// separate from [`Self::map_label`] so airport labels can refuse to draw
    /// where another one already is, instead of covering it.
    fn map_label_rect(p: &egui::Painter, at: Pos2, text: &str, col: Color32, fade: f32) -> Rect {
        let font = FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into()));
        let g = p.layout_no_wrap(text.to_string(), font, col.gamma_multiply(fade));
        Rect::from_min_size(at - Vec2::new(2.0, g.size().y / 2.0), g.size() + Vec2::new(4.0, 0.0))
    }

    /// Text with a dark backing, since map tiles are busy and unbacked labels
    /// vanish over a town.
    fn map_label(p: &egui::Painter, at: Pos2, text: &str, col: Color32, fade: f32) -> Rect {
        let r = Self::map_label_rect(p, at, text, col, fade);
        p.rect_filled(r, 2.0, Color32::from_black_alpha((190.0 * fade) as u8));
        let font = FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into()));
        let g = p.layout_no_wrap(text.to_string(), font, col.gamma_multiply(fade));
        p.galley(Pos2::new(at.x, at.y - g.size().y / 2.0), g, col);
        r
    }

    /// Airport markers and their ident labels, in the map's amber so a fixed
    /// facility is not mistaken for the cyan of something in the air.
    ///
    /// Returns the on-screen markers, so the caller can hit-test the pointer
    /// against them for the frequency tooltip. Airports appear only once the
    /// map is zoomed in past [`crate::data::SHOW_ZOOM`]; at the default
    /// wide view a marker is a blob under the traffic, and the range rings
    /// already say where the interesting things are.
    fn draw_airports(
        clip: &egui::Painter,
        rect: Rect,
        zoom: f64,
        to_screen: impl Fn(f64, f64) -> Pos2,
    ) -> Vec<(Pos2, &'static datasets::airports::Airport)> {
        if zoom < crate::data::SHOW_ZOOM {
            return Vec::new();
        }
        // Cull to the window, plus room for a label hanging over an edge.
        let near = rect.expand(30.0);
        let mut shown: Vec<(&'static datasets::airports::Airport, Pos2)> = crate::data::airports()
            .iter()
            .filter_map(|a| {
                let at = to_screen(a.lat, a.lon);
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
            clip.circle_filled(*at, r, col);
            clip.circle_stroke(*at, r + 1.0, Stroke::new(1.0, col.gamma_multiply(0.55)));
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
            if zoom < at_zoom {
                continue;
            }
            let at = Pos2::new(at.x + 8.0, at.y - 5.0);
            let r = Self::map_label_rect(clip, at, &a.ident, theme::VALUE, 1.0);
            if labels.iter().any(|l| l.intersects(r.expand(3.0))) {
                continue;
            }
            labels.push(r);
            Self::map_label(clip, at, &a.ident, theme::VALUE, 1.0);
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

    fn track_rows(
        ui: &mut egui::Ui,
        active: &[&crate::tracks::Track],
        now: std::time::Instant,
    ) {
        use crate::tracks::Kind;
        let count = |k: Kind| active.iter().filter(|t| t.kind() == k).count();
        ui.horizontal(|ui| {
            ui.label(legend("tracks"));
            ui.label(value(active.len().to_string()).size(12.0));
            // Broken down by kind, because "14 tracks" on a coast says nothing
            // about whether the aircraft or the shipping is being heard.
            for (k, name) in
                [
                    (Kind::Aircraft, "aircraft"),
                    (Kind::Vessel, "vessels"),
                    (Kind::Vehicle, "vehicles"),
                    (Kind::Station, "stations"),
                ]
            {
                let n = count(k);
                if n > 0 {
                    ui.add_space(8.0);
                    ui.label(legend(name));
                    ui.label(value(n.to_string()).size(12.0));
                }
            }
            if active.is_empty() {
                ui.add_space(10.0);
                ui.label(legend(
                    "tune to 1090 for aircraft, 162 for shipping, 144.8 for APRS",
                ));
            }
        });
        ui.add_space(6.0);

        // One table for both, so the columns are what the two have in common
        // and the one column that differs is named for what it holds. An
        // altitude column full of dashes beside every vessel is worse than a
        // column that says "altitude / status".
        const COLS: [(&str, f32); 10] = [
            ("name", 100.0),
            ("id", 80.0),
            ("kind", 62.0),
            ("alt / status", 96.0),
            ("speed", 70.0),
            ("course", 60.0),
            ("position", 170.0),
            // Both come from Mode S replies to a radar rather than from any
            // broadcast, so they fill in for aircraft under interrogation and
            // stay blank for everything else, vessels included. Blank rather
            // than a dash: nothing is missing from a ship that has no squawk.
            ("squawk", 64.0),
            ("wind / temp", 130.0),
            ("msgs", 56.0),
        ];
        // Wide enough that the last column is not clipped when the channel
        // strip is open, and scrolled sideways rather than squeezed when the
        // window is narrower than that.
        let width: f32 = COLS.iter().map(|(_, w)| w).sum::<f32>() + 60.0;
        egui::ScrollArea::horizontal().auto_shrink([false, false]).show(ui, |ui| {
        ui.set_min_width(width);
        let (rect, _) = ui.allocate_exact_size(Vec2::new(width, widgets::ROW_H), Sense::hover());
        let p = ui.painter_at(rect);
        let mut x = rect.left();
        for (name, w) in COLS {
            widgets::cell(&p, rect, x, w, name, theme::LEGEND);
            x += w;
        }
        widgets::cell(&p, rect, x, rect.right() - x, "age", theme::LEGEND);
        p.line_segment(
            [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
            Stroke::new(1.0, theme::ETCH),
        );

        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            for (n, a) in active.iter().enumerate() {
                let (rect, _) =
                    ui.allocate_exact_size(Vec2::new(width, widgets::ROW_H), Sense::hover());
                if !ui.is_rect_visible(rect) {
                    continue;
                }
                let p = ui.painter_at(rect);
                if n % 2 == 1 {
                    p.rect_filled(rect, 0.0, Color32::from_rgb(0x24, 0x27, 0x2D));
                }
                let dash = "-".to_string();
                // The one column that differs by kind: an aircraft is placed
                // vertically by its altitude, a vessel by what it is doing.
                // What a radar's interrogations got out of it, which no
                // broadcast carries: the code the crew set, and the weather
                // the aircraft is flying through.
                let (squawk, weather) = match &a.detail {
                    crate::tracks::Detail::Aircraft { squawk, wind, temp_c, .. } => (
                        squawk.map(|s| format!("{s:04}")).unwrap_or_default(),
                        match (wind, temp_c) {
                            (Some((kt, deg)), Some(t)) => {
                                format!("{deg:.0}/{kt:.0} kt  {t:.0} C")
                            }
                            (Some((kt, deg)), None) => format!("{deg:.0}/{kt:.0} kt"),
                            (None, Some(t)) => format!("{t:.0} C"),
                            (None, None) => String::new(),
                        },
                    ),
                    _ => (String::new(), String::new()),
                };
                let (state, state_col) = match &a.detail {
                    crate::tracks::Detail::Aircraft {
                        altitude_ft, vertical_rate_fpm, ..
                    } => (
                        altitude_ft.map(|v| format!("{v} ft")).unwrap_or_else(|| dash.clone()),
                        // Climb and descent are worth telling apart at a
                        // glance; level flight is not worth colouring at all.
                        match vertical_rate_fpm {
                            Some(v) if *v > 128 => CRC_OK,
                            Some(v) if *v < -128 => theme::READOUT,
                            _ => theme::VALUE,
                        },
                    ),
                    crate::tracks::Detail::Vessel { nav_status, ship_type, .. } => (
                        nav_status
                            .or(*ship_type)
                            .map(str::to_string)
                            .unwrap_or_else(|| dash.clone()),
                        theme::LEGEND,
                    ),
                    crate::tracks::Detail::Station { aid } => (
                        if *aid { "navigation mark".into() } else { "shore station".into() },
                        theme::LEGEND,
                    ),
                    // An APRS station says what it is in a comment more often
                    // than in any field, so that is what the column shows.
                    crate::tracks::Detail::Aprs { comment, altitude_ft, .. } => (
                        comment
                            .clone()
                            .or_else(|| altitude_ft.map(|v| format!("{v} ft")))
                            .unwrap_or_else(|| dash.clone()),
                        theme::LEGEND,
                    ),
                };
                let kind = match a.kind() {
                    Kind::Aircraft => "air",
                    Kind::Vessel => "sea",
                    Kind::Vehicle => "land",
                    Kind::Station => "fixed",
                };
                let text = [
                    (a.label.clone().unwrap_or_else(|| dash.clone()), theme::TRACE),
                    (a.id.text(), theme::VALUE),
                    (kind.to_string(), theme::LEGEND),
                    (state, state_col),
                    (
                        a.speed_kt.map(|v| format!("{v:.0} kt")).unwrap_or_else(|| dash.clone()),
                        theme::VALUE,
                    ),
                    (
                        a.course_deg.map(|v| format!("{v:.0}")).unwrap_or_else(|| dash.clone()),
                        theme::LEGEND,
                    ),
                    (
                        a.position
                            .map(|(lat, lon)| format!("{lat:.4}, {lon:.4}"))
                            .unwrap_or_else(|| dash.clone()),
                        theme::TRACE,
                    ),
                    (squawk, theme::VALUE),
                    (weather, theme::TRACE),
                    (a.messages.to_string(), theme::LEGEND),
                ];
                let mut x = rect.left();
                for ((t, c), (_, w)) in text.iter().zip(COLS) {
                    widgets::cell(&p, rect, x, w, t, *c);
                    x += w;
                }
                let age = a.age(now).as_secs();
                widgets::cell(&p, rect, x, rect.right() - x, &format!("{age}s"), theme::LEGEND);
            }
        });
        });
    }
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
