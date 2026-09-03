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

    /// The switch per layer that has one.
    ///
    /// Beside the map rather than in a settings panel: which layer is worth
    /// seeing changes with what is being watched, and a switch two panes away
    /// is one nobody flicks.
    pub fn switches(&mut self, ui: &mut egui::Ui, layers: &[&mut dyn Layer]) {
        ui.horizontal(|ui| {
            ui.label(legend("layers"));
            for l in layers.iter().filter(|l| l.switchable()) {
                self.layers.note(l.key());
                let on = self.layers.on(l.key());
                if ui.selectable_label(on, l.label()).clicked() {
                    self.layers.set(l.key(), !on);
                }
            }
        });
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

        let canvas = Canvas {
            p: clip,
            rect,
            mid,
            center,
            zoom,
            nm_px: crate::map::nm_px(center.0, zoom),
            // Nothing is hovered while the map is being dragged: the pointer
            // is moving the world, not pointing at it.
            hover: resp.hover_pos().filter(|_| !resp.dragged()),
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
        // Required by the tile usage policy, and by the licence the map data
        // is under.
        canvas.label(
            Pos2::new(rect.right() - 150.0, rect.bottom() - 10.0),
            "(c) OpenStreetMap contributors",
            theme::LEGEND,
            1.0,
        );

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
            canvas.label(Pos2::new(rect.left() + 8.0, rect.bottom() - 8.0), &short, theme::LEGEND, 1.0);
        }
        p.rect_stroke(rect, 2.0, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);

        let picked = resp
            .secondary_clicked()
            .then(|| resp.interact_pointer_pos())
            .flatten()
            .map(|pos| crate::map::screen_to_ll(center, zoom, offset(pos)));
        Drawn { picked }
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
                let Some(tex) = tiles.get(id, rt) else { continue };
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
}

impl Canvas {
    /// Where a position falls on screen.
    pub fn at(&self, lat: f64, lon: f64) -> Pos2 {
        let (x, y) = crate::map::ll_to_screen(self.center, self.zoom, (lat, lon));
        Pos2::new(self.mid.x + x as f32, self.mid.y + y as f32)
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
