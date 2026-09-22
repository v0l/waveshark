//! An interactive slippy map, and the surface things are drawn on.
//!
//! In the widget layer rather than the pane layer: it holds tiles, a camera
//! and a set of layers, and knows nothing about the receiver, aircraft or
//! airports. What is drawn on it comes in as [`Layer`] implementations, so a
//! second view wanting a map gets one by handing over a different list rather
//! than by copying this file.

use super::*;

/// Where the map is looking. `center` is `None` until something has been
/// drawn, so the first thing heard decides where the map opens rather than
/// the map opening on the ocean.
#[derive(Clone, Copy)]
pub(super) struct Camera {
    pub center: Option<(f64, f64)>,
    /// Continuous, not a tile level: the tile level is where the pictures
    /// come from, and rounding the camera to it would make most scroll
    /// notches do nothing.
    pub zoom: f64,
}

impl Default for Camera {
    fn default() -> Self {
        Self { center: None, zoom: DEFAULT_MAP_ZOOM }
    }
}

/// Something drawn over the tiles.
///
/// A layer is built fresh each frame around whatever it draws from, so it
/// borrows the caller's data rather than keeping a copy the map would have to
/// be told to update.
pub(super) trait Layer {
    /// What this layer is saved and switched by. Stable across releases; the
    /// label is not.
    fn key(&self) -> &'static str;

    /// What the switch says. Empty for a layer that has no switch.
    fn label(&self) -> &'static str {
        ""
    }

    /// Whether the operator can turn it off. A layer that answers false is
    /// always drawn and gets no switch.
    fn switchable(&self) -> bool {
        true
    }

    fn draw(&mut self, c: &Canvas);

    /// Drawn after every layer, for a hover card that has to sit over the
    /// things around what it describes.
    fn over(&mut self, _c: &Canvas) {}

    /// A phrase for the status line, such as how many of something is on
    /// screen.
    fn status(&self) -> Option<String> {
        None
    }

    /// Whose data this draws, for the credit in the corner. Empty for a
    /// layer drawing what this receiver measured, which is nobody else's,
    /// and more than one where a layer draws more than one publisher's.
    ///
    /// Answered by the layer rather than listed by the map: a layer added
    /// later that draws somebody's data credits them by saying so here, and
    /// the credit appears exactly while that data is on screen.
    fn credits(&self) -> Vec<crate::data::Credit> {
        Vec::new()
    }
}

/// Which layers are drawn, by key.
///
/// Held as the ones switched off, so a layer added later is on for everybody
/// rather than off for anybody with a saved session. Keys rather than an enum
/// because the map does not know what layers exist: the view handing them
/// over does.
#[derive(Clone, Default)]
pub(super) struct LayerSet {
    off: Vec<String>,
    /// Every key this set has been shown, so what is saved can be written
    /// without the layers themselves, which exist only during a frame.
    known: Vec<String>,
}

impl LayerSet {
    pub fn on(&self, key: &str) -> bool {
        !self.off.iter().any(|k| k == key)
    }

    pub fn set(&mut self, key: &str, on: bool) {
        self.note(key);
        self.off.retain(|k| k != key);
        if !on {
            self.off.push(key.to_string());
        }
    }

    fn note(&mut self, key: &str) {
        if !self.known.iter().any(|k| k == key) {
            self.known.push(key.to_string());
        }
    }

    /// Every layer named and its state, in the form the session file holds.
    /// All of them, not just the ones switched off: the file is read by hand,
    /// and a switch nobody can see in it is a switch nobody knows exists.
    pub fn saved(&self) -> Vec<(String, bool)> {
        self.known.iter().map(|k| (k.clone(), self.on(k))).collect()
    }

    pub fn restore(&mut self, saved: &[(String, bool)]) {
        for (key, on) in saved {
            self.set(key, *on);
        }
    }
}

/// The map, and what it remembers between frames.
#[derive(Default)]
pub(super) struct MapView {
    pub camera: Camera,
    pub layers: LayerSet,
    tiles: crate::map::Tiles,
}

/// One row of the layer list, laid out before anything is drawn.
struct Switch {
    at: Rect,
    label: &'static str,
    on: bool,
    hover: bool,
}

/// Height of a row in the layer list, and the padding inside its plate.
const SWITCH_ROW: f32 = 17.0;
const SWITCH_PAD: f32 = 6.0;
/// How far the control sits in from the corner of the map.
const SWITCH_INSET: f32 = 8.0;

/// Where the layer control sits: the button, and the panel under it.
///
/// Free of the map so the geometry can be tested. A panel that opens off the
/// bottom of a short map is a set of layers nobody can reach, so it rises to
/// stay inside the tiles.
fn switch_rects(map: Rect, button_w: f32, panel_w: f32, rows: usize) -> (Rect, Rect) {
    let button = Rect::from_min_size(
        Pos2::new(map.right() - SWITCH_INSET - button_w, map.top() + SWITCH_INSET),
        Vec2::new(button_w, SWITCH_ROW + 3.0),
    );
    let h = rows as f32 * SWITCH_ROW + SWITCH_PAD * 2.0;
    let top = (button.bottom() + 4.0).min((map.bottom() - SWITCH_INSET - h).max(map.top() + 4.0));
    let panel = Rect::from_min_size(
        Pos2::new(map.right() - SWITCH_INSET - panel_w, top),
        Vec2::new(panel_w, h),
    );
    (button, panel)
}

/// The `i`th row of an open layer panel.
fn switch_row(panel: Rect, i: usize) -> Rect {
    Rect::from_min_size(
        Pos2::new(panel.left(), panel.top() + SWITCH_PAD + i as f32 * SWITCH_ROW),
        Vec2::new(panel.width(), SWITCH_ROW),
    )
}

/// What a frame of the map did that the caller may care about.
pub(super) struct Drawn {
    /// Where a right-click landed, in degrees.
    pub picked: Option<(f64, f64)>,
}

impl MapView {
    /// Tiles fetched since the last frame, which have to be uploaded on the
    /// main thread. Called once a frame whether or not the map is on screen,
    /// so a map that is not being looked at still finishes what it started.
    pub fn poll(&mut self, ctx: &egui::Context) {
        self.tiles.poll(ctx);
    }

    /// Draw the map and everything switched on, in the order given.
    ///
    /// The tiles come from our own fetcher rather than a map crate: slippy
    /// tiles are a URL template and a Mercator projection, and what a map
    /// widget adds on top is a way to draw things over them, which is the
    /// part this file has to write anyway.
    ///
    /// `fallback` is where to open when nothing has set the camera yet.
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        height: f32,
        fallback: Option<(f64, f64)>,
        rt: &tokio::runtime::Handle,
        layers: &mut [&mut dyn Layer],
    ) -> Drawn {
        let w = ui.available_width();
        let (rect, resp) = ui.allocate_exact_size(Vec2::new(w, height), Sense::click_and_drag());
        let p = ui.painter_at(rect);
        p.rect_filled(rect, 2.0, theme::WELL);
        let font = FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into()));

        // Centre on what the caller suggests until someone drags the map
        // somewhere else. After that it stays where it was put: a map that
        // recentres itself is a map you cannot read.
        if self.camera.center.is_none() {
            self.camera.center = fallback;
        }
        let Some((clat, clon)) = self.camera.center else {
            p.rect_stroke(rect, 2.0, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);
            p.text(rect.center(), Align2::CENTER_CENTER, "no positions yet", font, theme::LEGEND);
            return Drawn { picked: None };
        };

        let mid = rect.center();
        let mut center = (clat, clon);
        let offset = |pos: Pos2| (f64::from(pos.x - mid.x), f64::from(pos.y - mid.y));

        if resp.dragged() {
            let d = resp.drag_delta();
            center = crate::map::screen_to_ll(
                center,
                self.camera.zoom,
                (f64::from(-d.x), f64::from(-d.y)),
            );
        }
        if let (true, Some(pos)) = (resp.hovered(), resp.hover_pos()) {
            let d = ui.input(|i| i.smooth_scroll_delta.y);
            if d != 0.0 {
                let next = (self.camera.zoom + f64::from(d) * 0.004).clamp(2.0, 19.0);
                center = crate::map::anchored_zoom(center, self.camera.zoom, next, offset(pos));
                self.camera.zoom = next;
            }
        }
        self.camera.center = Some(center);

        let zoom = self.camera.zoom;
        let (z, scale) = (crate::map::level(zoom), crate::map::tile_scale(zoom));
        let (cx, cy) = crate::map::project(center.0, center.1, z);
        let clip = p.with_clip_rect(rect);
        Self::draw_tiles(&clip, &mut self.tiles, rect, (cx, cy), z, scale, rt);

        // The layer control, over the tiles in the corner rather than on a
        // row above them: the row cost the map its height and read as a
        // toolbar, and every mapping application puts its layers here.
        // Placed before the layers draw so the pointer over it is the
        // control's and not the map's.
        let legend_font = FontId::new(10.0, FontFamily::Name(theme::LEGEND_FONT.into()));
        let width =
            |s: &str| p.layout_no_wrap(s.to_string(), legend_font.clone(), theme::LEGEND).size().x;
        let named: Vec<(&'static str, &'static str)> =
            layers.iter().filter(|l| l.switchable()).map(|l| (l.key(), l.label())).collect();
        for (key, _) in &named {
            self.layers.note(key);
        }
        let button_w = width("LAYERS") + 18.0;
        let widest = named.iter().map(|(_, l)| width(l)).fold(0.0, f32::max);
        let (button, panel) =
            switch_rects(rect, button_w, (widest + 34.0).max(button_w), named.len());
        // Open while the pointer is on it, shut the moment it leaves: a
        // panel held open by a click is one more thing to put away, and it
        // covers the corner of the map until somebody does.
        let reach = button.union(panel);
        let open = ui.input(|i| i.pointer.hover_pos()).is_some_and(|p| reach.contains(p));
        let mut switches = Vec::new();
        if open {
            for (i, (key, label)) in named.iter().enumerate() {
                let at = switch_row(panel, i);
                let r = ui.interact(at, ui.id().with(("map-layer", *key)), Sense::click());
                if r.clicked() {
                    let on = self.layers.on(key);
                    self.layers.set(key, !on);
                }
                switches.push(Switch { at, label, on: self.layers.on(key), hover: r.hovered() });
            }
        }
        if switches.iter().any(|s| s.hover) {
            ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
        }
        // A click on the control is not a click on the map: without this a
        // layer under the panel is picked through it, and a right-click moves
        // the station to wherever the switch happens to be.
        let taken = move |pos: &Pos2| button.contains(*pos) || (open && panel.contains(*pos));

        let canvas = Canvas {
            p: clip,
            rect,
            mid,
            center,
            zoom,
            nm_px: crate::map::nm_px(center.0, zoom),
            // Nothing is hovered while the map is being dragged: the pointer
            // is moving the world, not pointing at it.
            hover: resp.hover_pos().filter(|p| !resp.dragged() && !taken(p)),
            // A press that did not turn into a drag. A layer that draws
            // things worth picking reads this; the map itself has no use for
            // a left click, which is why one is free to mean "that one".
            click: resp
                .clicked()
                .then(|| resp.interact_pointer_pos())
                .flatten()
                .filter(|p| !taken(p)),
        };

        let mut status = format!(
            "{:.1} nm/cm    z{zoom:.1}    drag to pan, scroll to zoom",
            canvas.nm_px.recip() * 37.8
        );
        let mut on: Vec<&mut &mut dyn Layer> =
            layers.iter_mut().filter(|l| !l.switchable() || self.layers.on(l.key())).collect();
        for l in on.iter_mut() {
            l.draw(&canvas);
        }
        for l in on.iter_mut() {
            l.over(&canvas);
            if let Some(s) = l.status() {
                status.push_str("    ");
                status.push_str(&s);
            }
        }

        canvas.label(Pos2::new(rect.left() + 8.0, rect.top() + 10.0), &status, theme::LEGEND, 1.0);
        Self::draw_switches(&canvas.p, &legend_font, button, panel, &switches);

        // The tiles are somebody's, and so is anything a layer drew over
        // them. Both licences ask to be named where the map is seen.
        let mut credits = vec![crate::data::Credit {
            name: "OpenStreetMap",
            licence: "ODbL",
            url: "https://www.openstreetmap.org/copyright",
        }];
        for c in on.iter().flat_map(|l| l.credits()) {
            if !credits.contains(&c) {
                credits.push(c);
            }
        }
        let above_error = self.tiles.error().is_some();
        Self::draw_credits(ui, &canvas.p, rect, &credits, above_error);

        // A map with no tiles under it still shows what is drawn over them,
        // and would quietly look like empty sky and empty sea. Say what
        // failed instead.
        if let Some((err, n)) = self.tiles.error() {
            let short: String = err.chars().take(110).collect();
            canvas.p.rect_filled(
                Rect::from_min_max(
                    Pos2::new(rect.left(), rect.bottom() - 30.0),
                    Pos2::new(rect.right(), rect.bottom()),
                ),
                0.0,
                theme::WELL,
            );
            canvas.label(
                Pos2::new(rect.left() + 8.0, rect.bottom() - 20.0),
                &format!("{n} tile(s) failed, map is not showing terrain"),
                theme::FAULT,
                1.0,
            );
            canvas.label(
                Pos2::new(rect.left() + 8.0, rect.bottom() - 8.0),
                &short,
                theme::LEGEND,
                1.0,
            );
        }
        p.rect_stroke(rect, 2.0, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);

        let picked = resp
            .secondary_clicked()
            .then(|| resp.interact_pointer_pos())
            .flatten()
            .filter(|p| !taken(p))
            .map(|pos| crate::map::screen_to_ll(center, zoom, offset(pos)));
        Drawn { picked }
    }

    /// The layer control: one button shut, the list of layers open.
    ///
    /// Drawn last of everything over the tiles, so a track passing under it
    /// does not come out on top of the switch being reached for.
    fn draw_switches(
        p: &egui::Painter,
        font: &FontId,
        button: Rect,
        panel: Rect,
        switches: &[Switch],
    ) {
        let plate = |r: Rect| {
            p.rect_filled(r, 3.0, Color32::from_black_alpha(205));
            p.rect_stroke(r, 3.0, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);
        };
        plate(button);
        let lit = match switches.is_empty() {
            true => theme::VALUE,
            false => theme::READOUT,
        };
        p.text(button.center(), Align2::CENTER_CENTER, "LAYERS", font.clone(), lit);
        if switches.is_empty() {
            return;
        }
        plate(panel);
        for s in switches {
            let box_at = Rect::from_center_size(
                Pos2::new(panel.left() + SWITCH_PAD + 5.0, s.at.center().y),
                Vec2::splat(9.0),
            );
            match s.on {
                true => {
                    p.rect_filled(box_at, 2.0, theme::READOUT);
                }
                false => {
                    p.rect_stroke(box_at, 2.0, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);
                }
            }
            let col = match (s.on, s.hover) {
                (_, true) => theme::READOUT,
                (true, false) => theme::VALUE,
                (false, false) => theme::LEGEND,
            };
            p.text(
                Pos2::new(box_at.right() + 7.0, s.at.center().y),
                Align2::LEFT_CENTER,
                s.label,
                font.clone(),
                col,
            );
        }
    }

    /// Who the map belongs to, along the bottom right.
    ///
    /// One plate rather than a line of loose text: tiles are busy, and this
    /// is a licence condition rather than decoration, so it has to stay
    /// readable over a city. Each name is the publisher's page.
    fn draw_credits(
        ui: &mut egui::Ui,
        p: &egui::Painter,
        rect: Rect,
        credits: &[crate::data::Credit],
        raised: bool,
    ) {
        let name_font = FontId::new(10.0, FontFamily::Name(theme::LEGEND_FONT.into()));
        let small = FontId::new(9.0, FontFamily::Name(theme::LEGEND_FONT.into()));
        // Laid out first and drawn second, because the plate has to be the
        // size of the text and the text sits inside the plate.
        let mut pieces: Vec<(std::sync::Arc<egui::Galley>, Option<&'static str>)> = Vec::new();
        for (i, c) in credits.iter().enumerate() {
            if i > 0 {
                pieces.push((p.layout_no_wrap("·".into(), small.clone(), theme::ETCH), None));
            }
            pieces.push((
                p.layout_no_wrap(c.name.into(), name_font.clone(), theme::VALUE),
                Some(c.url),
            ));
            pieces.push((p.layout_no_wrap(c.licence.into(), small.clone(), theme::LEGEND), None));
        }
        let gap = 5.0;
        let w: f32 =
            pieces.iter().map(|(g, _)| g.size().x).sum::<f32>() + gap * (pieces.len() - 1) as f32;
        let h = pieces.iter().map(|(g, _)| g.size().y).fold(0.0, f32::max);
        let pad = Vec2::new(7.0, 4.0);
        let bottom = rect.bottom() - if raised { 38.0 } else { 8.0 };
        let plate = Rect::from_min_max(
            Pos2::new(rect.right() - 8.0 - w - pad.x * 2.0, bottom - h - pad.y * 2.0),
            Pos2::new(rect.right() - 8.0, bottom),
        );
        p.rect_filled(plate, 3.0, Color32::from_black_alpha(205));
        p.rect_stroke(plate, 3.0, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);

        let mut x = plate.left() + pad.x;
        for (g, link) in pieces {
            let at = Pos2::new(x, plate.center().y - g.size().y / 2.0);
            let size = g.size();
            if let Some(url) = link {
                let hit = Rect::from_min_size(at, size).expand2(Vec2::new(2.0, 3.0));
                let r = ui.interact(hit, ui.id().with(("map-credit", url)), Sense::click());
                if r.hovered() {
                    ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
                    let y = at.y + size.y - 1.0;
                    p.line_segment(
                        [Pos2::new(at.x, y), Pos2::new(at.x + size.x, y)],
                        Stroke::new(1.0, theme::READOUT),
                    );
                }
                if r.clicked() {
                    ui.ctx().open_url(egui::OpenUrl::new_tab(url));
                }
                p.galley(at, g, if r.hovered() { theme::READOUT } else { theme::VALUE });
            } else {
                p.galley(at, g, theme::LEGEND);
            }
            x += size.x + gap;
        }
    }

    /// Every tile the view touches, asked for as it is drawn.
    fn draw_tiles(
        p: &egui::Painter,
        tiles: &mut crate::map::Tiles,
        rect: Rect,
        center: (f64, f64),
        z: u8,
        scale: f64,
        rt: &tokio::runtime::Handle,
    ) {
        let (cx, cy) = center;
        let mid = rect.center();
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
                let Some(tex) = tiles.get(id, rt) else {
                    continue;
                };
                let min = Pos2::new(
                    mid.x + ((tx as f64 - cx) * scale) as f32,
                    mid.y + ((ty as f64 - cy) * scale) as f32,
                );
                let at = Rect::from_min_size(min, Vec2::splat(scale as f32));
                p.image(
                    tex.id(),
                    at,
                    Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
                    // Held back so what is drawn over it stays the brightest
                    // thing on screen, and so it sits in the panel's palette
                    // rather than glowing white in a dark interface.
                    Color32::from_gray(150),
                );
            }
        }
    }
}

/// The map surface a layer draws on: where things are, and the few marks that
/// every layer needs to make the same way.
pub(super) struct Canvas {
    /// Clipped to the map, so a layer cannot draw over the table under it.
    pub p: egui::Painter,
    pub rect: Rect,
    mid: Pos2,
    center: (f64, f64),
    zoom: f64,
    nm_px: f64,
    hover: Option<Pos2>,
    click: Option<Pos2>,
}

impl Canvas {
    /// Where a position falls on screen.
    pub fn at(&self, lat: f64, lon: f64) -> Pos2 {
        let (x, y) = crate::map::ll_to_screen(self.center, self.zoom, (lat, self.near(lon)));
        Pos2::new(self.mid.x + x as f32, self.mid.y + y as f32)
    }

    /// The same longitude, written as the copy of the world nearest the
    /// view.
    ///
    /// The projection is absolute, so 170 east of a view centred on Ireland
    /// lands a whole world to the right rather than just off the left edge,
    /// where the tiles for it are drawn. Anything short, an airport or a
    /// mast, is off screen either way; a line drawn between two points that
    /// disagree about which copy they are on is a stroke across the map,
    /// which is what a satellite's ground track looked like.
    fn near(&self, lon: f64) -> f64 {
        nearest_copy(lon, self.center.1)
    }

    pub fn zoom(&self) -> f64 {
        self.zoom
    }

    /// Screen pixels per nautical mile at a latitude, for anything drawn at
    /// a real distance from a real place. Mercator stretches with latitude,
    /// so a ring sized at the view's centre changes size as the map is
    /// dragged north and south while the place it is around does not move.
    pub fn nm_px_at(&self, lat: f64) -> f64 {
        crate::map::nm_px(lat, self.zoom)
    }

    /// Where the pointer is, or `None` while the map is being dragged.
    pub fn hover(&self) -> Option<Pos2> {
        self.hover
    }

    /// Where the map was clicked this frame, for a layer whose marks can be
    /// picked. Panning is a drag and does not report one.
    pub fn click(&self) -> Option<Pos2> {
        self.click
    }

    /// How wide the whole world is on screen, in pixels.
    ///
    /// What a layer joining two positions with a line needs in order to tell
    /// a move from a wrap: at a zoom where the world is narrower than the
    /// pane, a jump across the date line is shorter than the pane and a test
    /// against the pane's width lets it through.
    pub fn world_px(&self) -> f32 {
        (crate::map::tile_scale(self.zoom) * f64::from(1u32 << crate::map::level(self.zoom))) as f32
    }

    /// The backing box a label would occupy, before it is drawn. Separate
    /// from [`Self::label`] so a layer can refuse to draw where another label
    /// already is, instead of covering it.
    pub fn label_rect(&self, at: Pos2, text: &str, col: Color32, fade: f32) -> Rect {
        let g = self.p.layout_no_wrap(text.to_string(), Self::font(), col.gamma_multiply(fade));
        Rect::from_min_size(at - Vec2::new(2.0, g.size().y / 2.0), g.size() + Vec2::new(4.0, 0.0))
    }

    /// Text with a dark backing, since map tiles are busy and unbacked labels
    /// vanish over a town.
    pub fn label(&self, at: Pos2, text: &str, col: Color32, fade: f32) -> Rect {
        let r = self.label_rect(at, text, col, fade);
        self.p.rect_filled(r, 2.0, Color32::from_black_alpha((190.0 * fade) as u8));
        let g = self.p.layout_no_wrap(text.to_string(), Self::font(), col.gamma_multiply(fade));
        self.p.galley(Pos2::new(at.x, at.y - g.size().y / 2.0), g, col);
        r
    }

    pub fn font() -> FontId {
        FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into()))
    }
}

/// A longitude written as the copy of the world nearest a centre. Free of
/// the canvas so it can be tested without one.
fn nearest_copy(lon: f64, centre: f64) -> f64 {
    lon - 360.0 * ((lon - centre) / 360.0).round()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// What broke the ground tracks: two points either side of the date line
    /// projected a whole world apart, and the line between them crossed the
    /// map.
    #[test]
    fn a_longitude_is_drawn_on_the_copy_of_the_world_nearest_the_view() {
        // Nothing moves what is already near the centre.
        assert!((nearest_copy(-6.0, -6.26) + 6.0).abs() < 1e-9);
        // A point just past the date line, seen from Ireland, is off the
        // left edge rather than a world to the right.
        assert!((nearest_copy(179.0, -6.26) - -181.0).abs() < 1e-9);
        assert!((nearest_copy(-179.0, -6.26) - -179.0).abs() < 1e-9);
        // And the same the other way round, from a view near the date line.
        assert!((nearest_copy(-179.0, 178.0) - 181.0).abs() < 1e-9);
        // A step across the line is a step, not a stroke across the world.
        let a = nearest_copy(179.5, 179.0);
        let b = nearest_copy(-179.5, 179.0);
        assert!((a - b).abs() < 1.5, "{a} to {b}");
    }

    /// What the map pane hands over: eight layers, of which the station is
    /// always drawn and gets no switch.
    const LAYERS: usize = 7;

    #[test]
    fn the_open_layer_panel_stays_on_the_map() {
        let map = Rect::from_min_size(Pos2::new(40.0, 60.0), Vec2::new(900.0, 520.0));
        let (button, panel) = switch_rects(map, 60.0, 130.0, LAYERS);
        assert!(map.contains_rect(button), "{button:?} left {map:?}");
        assert!(map.contains_rect(panel), "{panel:?} left {map:?}");
        // Under the button, not over it, when there is room.
        assert!(panel.top() >= button.bottom(), "{} over {}", panel.top(), button.bottom());
        // One row each, all of them inside the plate.
        for i in 0..LAYERS {
            let row = switch_row(panel, i);
            assert!(panel.contains_rect(row), "row {i} at {row:?} left {panel:?}");
        }
        assert_eq!(switch_row(panel, 1).top() - switch_row(panel, 0).top(), SWITCH_ROW);
        // What the pointer has to be inside for the list to be open: the
        // button, the list, and the gap between them, so crossing from one
        // to the other does not shut it.
        let reach = button.union(panel);
        assert!(reach.contains(button.center()));
        assert!(reach.contains(switch_row(panel, LAYERS - 1).center()));
        assert!(reach.contains(Pos2::new(button.center().x, button.bottom() + 2.0)));
        assert!(!reach.contains(map.center()), "the map itself is not the control");
    }

    /// The table takes more than half the pane, and the map under a narrow
    /// window is shorter than eight rows of switches: the panel rises rather
    /// than opening off the bottom edge, where nothing can be reached.
    #[test]
    fn a_panel_taller_than_a_short_map_rises_to_stay_on_it() {
        let map = Rect::from_min_size(Pos2::new(0.0, 0.0), Vec2::new(600.0, 140.0));
        let (button, panel) = switch_rects(map, 60.0, 130.0, LAYERS);
        assert!(map.contains_rect(panel), "{panel:?} left {map:?}");
        assert!(panel.top() < button.bottom(), "a 140 px map has room it has not");
        assert_eq!(switch_row(panel, LAYERS - 1).bottom(), panel.bottom() - SWITCH_PAD);
    }

    #[test]
    fn a_layer_switched_off_is_the_only_one_saved_off() {
        let mut set = LayerSet::default();
        for key in ["rings", "cells", "airports", "tracks"] {
            set.note(key);
        }
        assert_eq!(set.saved().len(), 4);
        assert_eq!(set.saved().iter().filter(|(_, on)| *on).count(), 4);
        set.set("cells", false);
        assert_eq!(set.saved().iter().filter(|(_, on)| !*on).count(), 1);
        assert!(!set.on("cells"));
        assert!(set.on("tracks"));

        let mut second = LayerSet::default();
        second.restore(&set.saved());
        assert_eq!(second.saved(), set.saved());
        second.set("cells", true);
        assert_eq!(second.saved().iter().filter(|(_, on)| *on).count(), 4);
    }

    /// Zoomed out, the whole world is narrower than the pane, so a line that
    /// jumps a world is shorter than the pane: a wrap test measured against
    /// the pane lets every one of them through, which is what put horizontal
    /// strokes across the map.
    #[test]
    fn a_wrap_is_measured_against_the_world_and_not_against_the_pane() {
        let world =
            |zoom: f64| crate::map::tile_scale(zoom) * f64::from(1u32 << crate::map::level(zoom));
        assert!(world(2.0) < 1200.0, "{} px", world(2.0));
        // And it doubles with every level, so the test has to scale too.
        assert!((world(3.0) / world(2.0) - 2.0).abs() < 1e-9);
    }
}
