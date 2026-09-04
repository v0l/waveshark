//! The map pane: where the receiver is, what layers are drawn, and the table
//! of tracks beside them.
//!
//! The map itself is [`super::mapview`], which knows nothing about the
//! receiver. Everything here that is receiver-specific is a [`Layer`] handed
//! to it: the range rings, the station, the airports, the tracks.

mod layers;

use super::mapview::{Layer, MapView};
use super::*;
use layers::{AirportLayer, RingLayer, StationLayer, TrackLayer};

/// What the map pane remembers. Its own, and reachable from no other view:
/// the camera and the tiles belong to the map widget, and the tracks are the
/// pane's copy of what the tracker in the graph is holding.
#[derive(Default)]
pub(super) struct MapState {
    pub map: MapView,
    /// Tracks, folded together from whatever on the bus reports a position:
    /// aircraft from ADS-B, vessels and marks from AIS.
    pub tracks: Vec<crate::tracks::Track>,
}

/// The map, over where it is looking and what is on it.
pub(super) struct Map<'a> {
    pub st: &'a mut MapState,
    /// Where the receiver is, when it has been told.
    pub home: Option<(f64, f64)>,
    /// The position being typed, while it is being typed. Kept apart from the
    /// real one so a half-finished latitude does not move the map.
    pub edit: &'a mut Option<String>,
    /// Where tile fetches are run. The application owns the runtime; the pane
    /// is handed a handle for the frame.
    pub rt: tokio::runtime::Handle,
}

impl Map<'_> {
    /// Everything the tracker in the graph is holding, on tiles, and the
    /// table under them.
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
        self.st.map.poll(ui.ctx());
        let mut place = None;
        let mut edit = self.edit.take();
        {
            let map = &mut self.st.map;
            let active: Vec<&crate::tracks::Track> = self.st.tracks.iter().collect();
            let home = self.home;
            let rt = &self.rt;
            let body = |ui: &mut egui::Ui| {
                place = Self::station_row(ui, home, &mut edit);
                ui.add_space(4.0);

                // Built fresh each frame around what they draw from, so a
                // layer borrows the live data instead of the map holding a
                // copy somebody has to remember to update. Order is what the
                // map draws in: fixed things under moving ones, so the cyan
                // of what is in the air stays the brightest thing on screen.
                let mut rings = RingLayer { home };
                let mut airports = AirportLayer::default();
                let mut station = StationLayer { home };
                let mut tracks = TrackLayer { active: &active, now };
                let mut layers: [&mut dyn Layer; 4] =
                    [&mut rings, &mut airports, &mut station, &mut tracks];

                map.switches(ui, &layers);
                ui.add_space(6.0);
                // Half the pane each, roughly: the map is the view worth
                // having and the table is what you read once something on it
                // is interesting.
                let h = (ui.available_height() * 0.55).clamp(160.0, 1200.0);
                let fallback = home.or_else(|| mean_position(&active));
                let drawn = map.show(ui, h, fallback, rt, &mut layers);
                place = place.or(drawn.picked);
                ui.add_space(10.0);
                Self::track_rows(ui, &active, now);
            };
            margin.show(ui, body);
        }
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
    fn track_rows(
        ui: &mut egui::Ui,
        active: &[&crate::tracks::Track],
        now: std::time::Instant,
    ) {
        use crate::tracks::Kind;
        let count = |k: Kind| active.iter().filter(|t| t.kind() == k).count();
        ui.horizontal(|ui| {
            let mut line =
                theme::Line::new().legend("tracks").value(active.len().to_string()).size(12.0);
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
                    line = line.gap(16.0).legend(name).value(n.to_string()).size(12.0);
                }
            }
            if active.is_empty() {
                line = line
                    .gap(16.0)
                    .legend("tune to 1090 for aircraft, 162 for shipping, 144.8 for APRS");
            }
            line.show(ui);
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
                    crate::tracks::Detail::Mesh { short_name, altitude_m, battery_pct, .. } => {
                        let mut parts = Vec::new();
                        if let Some(s) = short_name {
                            parts.push(s.clone());
                        }
                        if let Some(b) = battery_pct {
                            parts.push(if *b > 100 { "on power".into() } else { format!("{b}%") });
                        }
                        if let Some(a) = altitude_m {
                            parts.push(format!("{a} m"));
                        }
                        (if parts.is_empty() { dash.clone() } else { parts.join(", ") }, theme::LEGEND)
                    }
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
