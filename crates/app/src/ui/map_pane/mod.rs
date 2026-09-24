//! The map pane: where the receiver is, what layers are drawn, and the table
//! of tracks beside them.
//!
//! The map itself is [`super::mapview`], which knows nothing about the
//! receiver. Everything here that is receiver-specific is a [`Layer`] handed
//! to it: the range rings, the station, the airports, the tracks.

mod layers;
mod silhouette;

use super::mapview::{Layer, MapView};
use super::*;
use layers::{
    AirportLayer, CellLayer, DispatchLayer, RingLayer, SatLayer, SightingLayer, SondeLayer,
    StationLayer, TrackLayer,
};

/// What the map pane remembers. Its own, and reachable from no other view:
/// the camera and the tiles belong to the map widget, and the tracks are the
/// pane's copy of what the tracker in the graph is holding.
/// Share of the pane the map gets by default. The map is the view worth
/// having; the table is what you read once something on it is interesting.
const DEFAULT_MAP_FRAC: f32 = 0.55;
/// How far the divider can be dragged. Neither half may be squeezed to
/// nothing: a two pixel map is not a smaller map, it is a broken one.
const MAP_FRAC_RANGE: std::ops::RangeInclusive<f32> = 0.15..=0.9;

pub(super) struct MapState {
    pub map: MapView,
    /// Where the divider between the map and the table sits.
    pub map_frac: f32,
    /// The satellite clicked on the map this frame, for the application to
    /// select. Held here rather than returned because the layer that names
    /// it lives inside the draw and the caller reads state, not layers.
    pub sat_hit: Option<u64>,
    /// Whether the divider is being dragged, so it stays lit and keeps the
    /// drag even when the pointer runs ahead of it.
    splitting: bool,
    /// Tracks, folded together from whatever on the bus reports a position:
    /// aircraft from ADS-B, vessels and marks from AIS.
    pub tracks: Vec<crate::tracks::Track>,
}

impl Default for MapState {
    fn default() -> Self {
        Self {
            map: MapView::default(),
            map_frac: DEFAULT_MAP_FRAC,
            sat_hit: None,
            splitting: false,
            tracks: Vec::new(),
        }
    }
}

/// One device's sightings, as the map is handed them.
pub(super) struct Trail<'a> {
    pub points: &'a [survey::Sighting],
    pub ident: Option<&'a str>,
    /// Where the sightings put the device, when they can say.
    pub estimate: Option<survey::Estimate>,
}

/// The map, over where it is looking and what is on it.
pub(super) struct Map<'a> {
    pub st: &'a mut MapState,
    /// Where the receiver is, when it has been told.
    pub home: Option<(f64, f64)>,
    /// How far out that is, in metres, when the fix that set it said.
    pub accuracy_m: Option<f64>,
    /// The sightings of the device selected in the device list, if one is.
    pub trail: Trail<'a>,
    /// Devices the survey holds, for the layers that draw what was heard
    /// rather than what somebody published.
    pub heard: &'a [survey::Device],
    pub messages: &'a crate::messages::Messages,
    /// The satellite the pass table has selected, drawn whether or not it is
    /// above the horizon, and the group it was selected from.
    pub sat: Option<u64>,
    pub sat_group: &'static datasets::tle::Group,
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
        let mut sat_hit = None;
        let mut split = self.st.map_frac;
        let splitting = &mut self.st.splitting;
        {
            let map = &mut self.st.map;
            let active: Vec<&crate::tracks::Track> = self.st.tracks.iter().collect();
            let home = self.home;
            let accuracy_m = self.accuracy_m;
            let rt = &self.rt;
            let selected_sat = self.sat;
            let sat_group = self.sat_group;
            let body = |ui: &mut egui::Ui| {
                // Built fresh each frame around what they draw from, so a
                // layer borrows the live data instead of the map holding a
                // copy somebody has to remember to update. Order is what the
                // map draws in: fixed things under moving ones, so the cyan
                // of what is in the air stays the brightest thing on screen.
                let mut rings = RingLayer { home };
                let mut airports = AirportLayer::default();
                let mut cells = CellLayer::new(self.heard);
                let mut sondes = SondeLayer::default();
                let mut station = StationLayer { home, accuracy_m };
                let mut tracks = TrackLayer { active: &active, now, named_airframes: false };
                let mut sats = SatLayer::new(
                    crate::sats::sky(sat_group),
                    sat_group,
                    selected_sat,
                    home,
                    crate::sats::now_s(),
                );
                let mut sightings = SightingLayer {
                    trail: self.trail.points,
                    ident: self.trail.ident,
                    estimate: self.trail.estimate,
                };
                let mut dispatch = DispatchLayer::new(self.messages.recent());
                let mut layers: [&mut dyn Layer; 9] = [
                    &mut rings,
                    &mut cells,
                    &mut airports,
                    &mut sondes,
                    &mut station,
                    &mut sightings,
                    &mut dispatch,
                    &mut sats,
                    &mut tracks,
                ];

                // The table is the tracks layer read as text, so it goes
                // with it: switching the layer off and leaving half the pane
                // to a table of what is no longer drawn is a pane arguing
                // with itself.
                let listing = map.layers.on("tracks");
                let top = ui.cursor().top();
                let usable = match listing {
                    true => (ui.available_height() - SPLIT_GRIP_H).max(200.0),
                    false => ui.available_height().max(200.0),
                };
                let frac = match listing {
                    true => split.clamp(*MAP_FRAC_RANGE.start(), *MAP_FRAC_RANGE.end()),
                    false => 1.0,
                };
                let fallback = home.or_else(|| mean_position(&active));
                let drawn = map.show(ui, usable * frac, fallback, rt, &mut layers);
                place = drawn.picked;
                // A satellite clicked on the map is the same selection the
                // pass table makes, so picking one here shows its track and
                // its footprint and highlights its card.
                sat_hit = sats.hit;
                if listing {
                    split = Self::divider(ui, top, usable, split, splitting);
                    ui.add_space(4.0);
                    Self::track_rows(ui, &active, now);
                }
            };
            margin.show(ui, body);
        }
        self.st.map_frac = split;
        self.st.sat_hit = sat_hit;
        place
    }

    /// The handle between the map and the table, and the drag that moves it.
    fn divider(ui: &mut egui::Ui, top: f32, usable: f32, frac: f32, splitting: &mut bool) -> f32 {
        split_divider(ui, top, usable, frac, splitting, MAP_FRAC_RANGE, DEFAULT_MAP_FRAC)
    }

    fn track_rows(ui: &mut egui::Ui, active: &[&crate::tracks::Track], now: std::time::Instant) {
        use crate::tracks::Kind;
        let count = |k: Kind| active.iter().filter(|t| t.kind() == k).count();
        ui.horizontal(|ui| {
            let mut line = Line::new().legend("tracks").value(active.len().to_string()).size(12.0);
            // Broken down by kind, because "14 tracks" on a coast says nothing
            // about whether the aircraft or the shipping is being heard.
            for (k, name) in [
                (Kind::Aircraft, "aircraft"),
                (Kind::Vessel, "vessels"),
                (Kind::Vehicle, "vehicles"),
                (Kind::Station, "stations"),
            ] {
                let n = count(k);
                if n > 0 {
                    line = line.gap(16.0).legend(name).value(n.to_string()).size(12.0);
                }
            }
            line.show(ui);
        });
        ui.add_space(6.0);

        // One table for both, so the columns are what the two have in common
        // and the one column that differs is named for what it holds. An
        // altitude column full of dashes beside every vessel is worse than a
        // column that says "altitude / status".
        const COLS: [(&str, f32); 13] = [
            ("name", 100.0),
            ("id", 100.0),
            ("registry", 110.0),
            ("system", 76.0),
            ("kind", 62.0),
            ("status", 110.0),
            ("speed", 70.0),
            ("course", 60.0),
            ("position", 170.0),
            // Beside the position, since it is the third coordinate of it.
            ("alt", 64.0),
            // The squawk comes from Mode S replies to a radar rather than
            // from any broadcast, so it fills in for aircraft under
            // interrogation and stays blank for everything else, vessels
            // included. Blank rather than a dash: nothing is missing from a
            // ship that has no squawk. The weather is whatever the thing
            // measured where it is: an aircraft's wind and air temperature
            // out of its registers, a mesh node's sensor readings.
            ("squawk", 64.0),
            ("weather", 150.0),
            ("msgs", 56.0),
        ];
        // Wide enough that the last column is not clipped when the channel
        // strip is open, and scrolled sideways rather than squeezed when the
        // window is narrower than that.
        let width: f32 = COLS.iter().map(|(_, w)| w).sum::<f32>() + 60.0;
        let fleet = active
            .iter()
            .any(|t| matches!(t.id, crate::tracks::TrackId::Icao(_)))
            .then(crate::data::aircraft)
            .flatten();
        egui::ScrollArea::horizontal().auto_shrink([false, false]).show(ui, |ui| {
            ui.set_min_width(width);
            let (rect, _) = ui.allocate_exact_size(Vec2::new(width, table::ROW_H), Sense::hover());
            let p = ui.painter_at(rect);
            let mut x = rect.left();
            for (name, w) in COLS {
                table::cell(&p, rect, x, w, name, theme::LEGEND);
                x += w;
            }
            table::cell(&p, rect, x, rect.right() - x, "age", theme::LEGEND);
            p.line_segment(
                [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
                Stroke::new(1.0, theme::ETCH),
            );

            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                for (n, a) in active.iter().enumerate() {
                    let (rect, _) =
                        ui.allocate_exact_size(Vec2::new(width, table::ROW_H), Sense::hover());
                    if !ui.is_rect_visible(rect) {
                        continue;
                    }
                    let registry = (
                        a.airframe(fleet.as_deref()).map(|p| p.summary()).unwrap_or_default(),
                        theme::VALUE,
                    );
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
                        crate::tracks::Detail::Mesh {
                            temperature_c,
                            humidity_pct,
                            pressure_hpa,
                            ..
                        } => {
                            let mut parts = Vec::new();
                            if let Some(t) = temperature_c {
                                parts.push(format!("{t:.1} C"));
                            }
                            if let Some(h) = humidity_pct {
                                parts.push(format!("{h:.0}%"));
                            }
                            if let Some(p) = pressure_hpa {
                                parts.push(format!("{p:.0} hPa"));
                            }
                            (String::new(), parts.join("  "))
                        }
                        _ => (String::new(), String::new()),
                    };
                    let alt = match &a.detail {
                        crate::tracks::Detail::Aircraft { altitude_ft, .. }
                        | crate::tracks::Detail::Aprs { altitude_ft, .. } => {
                            altitude_ft.map(|v| format!("{v} ft"))
                        }
                        crate::tracks::Detail::Mesh { altitude_m, .. } => {
                            altitude_m.map(|v| format!("{v} m"))
                        }
                        crate::tracks::Detail::Sonde { altitude_m, .. } => {
                            Some(format!("{altitude_m:.0} m"))
                        }
                        // Whatever the thing turns out to be, a height it
                        // stated is a height: a drone's altitude arrives
                        // before anything has said it is an aircraft.
                        _ => a.altitude_m.map(|m| format!("{m:.0} m")),
                    }
                    .unwrap_or_else(|| dash.clone());
                    let (state, state_col) = match &a.alert {
                        // Anything that said something is wrong says it
                        // here, whatever sort of thing it is: a beacon's
                        // distress outranks how fast it is climbing.
                        Some((severity, what)) => (
                            what.clone(),
                            match severity {
                                common::packet::Severity::Immediate => theme::FAULT,
                                common::packet::Severity::Warning => theme::READOUT,
                                common::packet::Severity::Advisory => theme::VALUE,
                            },
                        ),
                        None => state_of(a, &dash),
                    };
                    let kind = match a.kind() {
                        Kind::Aircraft => "air",
                        Kind::Vessel => "sea",
                        Kind::Vehicle => "land",
                        Kind::Station => "fixed",
                        Kind::Sonde => "balloon",
                        Kind::Transmitter => "heard",
                    };
                    let text = [
                        (a.label.clone().unwrap_or_else(|| dash.clone()), theme::TRACE),
                        (a.id.text(), theme::VALUE),
                        registry,
                        (a.id.system().to_string(), theme::LEGEND),
                        (kind.to_string(), theme::LEGEND),
                        (state, state_col),
                        (
                            a.speed_kt
                                .map(|v| format!("{v:.0} kt"))
                                .unwrap_or_else(|| dash.clone()),
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
                        (alt, theme::VALUE),
                        (squawk, theme::VALUE),
                        (weather, theme::TRACE),
                        (a.messages.to_string(), theme::LEGEND),
                    ];
                    let mut x = rect.left();
                    let mut id_end = x;
                    for ((t, c), (name, w)) in text.iter().zip(COLS) {
                        table::cell(&p, rect, x, w, t, *c);
                        if name == "id" {
                            let drawn = p.layout_no_wrap(t.to_string(), theme::figure(11.0), *c);
                            id_end = x + drawn.size().x.min(w - 6.0);
                        }
                        x += w;
                    }
                    let age = a.age(now).as_secs();
                    table::cell(&p, rect, x, rect.right() - x, &format!("{age}s"), theme::LEGEND);
                    if let Some(url) = a.vesselfinder() {
                        let at = Rect::from_min_size(
                            Pos2::new(id_end + 4.0, rect.top()),
                            Vec2::splat(table::ROW_H),
                        );
                        let id = egui::Id::new(("vesselfinder", &a.id));
                        let tip = "Open on VesselFinder";
                        if crate::icons::icon_button_at(ui, at, id, crate::icons::Icon::Link, tip)
                            .clicked()
                        {
                            ui.ctx().open_url(egui::OpenUrl::new_tab(url));
                        }
                    }
                }
            });
        });
    }
}

/// What a row says the thing is doing, and the colour it reads in.
///
/// One arm per kind of track, because what is worth knowing at a glance
/// differs: an aircraft's climb, a vessel's navigation status, a mesh node's
/// battery. An alert overrides all of it, and that is decided by the caller.
fn state_of(a: &crate::tracks::Track, dash: &str) -> (String, Color32) {
    let dash = dash.to_string();
    match &a.detail {
        crate::tracks::Detail::Aircraft { .. } => {
            let vertical_rate_fpm = &a.vertical_rate_fpm();
            (
                match vertical_rate_fpm {
                    Some(v) if *v > 128 => format!("climbing {v} fpm"),
                    Some(v) if *v < -128 => format!("descending {} fpm", -v),
                    Some(_) => "level".to_string(),
                    None => dash.clone(),
                },
                // Climb and descent are worth telling apart at a
                // glance; level flight is not worth colouring at all.
                match vertical_rate_fpm {
                    Some(v) if *v > 128 => CRC_OK,
                    Some(v) if *v < -128 => theme::READOUT,
                    _ => theme::VALUE,
                },
            )
        }
        crate::tracks::Detail::Vessel { nav_status, ship_type, .. } => (
            nav_status.or(*ship_type).map(str::to_string).unwrap_or_else(|| dash.clone()),
            theme::LEGEND,
        ),
        crate::tracks::Detail::Station { aid } => {
            (if *aid { "navigation mark".into() } else { "shore station".into() }, theme::LEGEND)
        }
        // An APRS station says what it is in a comment more often
        // than in any field, so that is what the column shows.
        crate::tracks::Detail::Aprs { comment, .. } => {
            (comment.clone().unwrap_or_else(|| dash.clone()), theme::LEGEND)
        }
        // What a sonde watcher wants at a glance: whether it
        // is still going up, and how fast.
        crate::tracks::Detail::Sonde { climb_ms, descending, temperature_c, .. } => (
            match temperature_c {
                // The reading the balloon was sent up for,
                // beside the only other thing worth seeing at
                // a glance: which way it is going.
                Some(t) => format!(
                    "{} {:.1} m/s, {t:.1} C",
                    if *descending { "descending" } else { "climbing" },
                    climb_ms.abs()
                ),
                None => format!(
                    "{} {:.1} m/s",
                    if *descending { "descending" } else { "climbing" },
                    climb_ms.abs()
                ),
            },
            if *descending { theme::READOUT } else { CRC_OK },
        ),
        crate::tracks::Detail::Mesh { short_name, battery_pct, .. } => {
            let mut parts = Vec::new();
            if let Some(s) = short_name {
                parts.push(s.clone());
            }
            if let Some(b) = battery_pct {
                parts.push(if *b > 100 { "on power".into() } else { format!("{b}%") });
            }
            (if parts.is_empty() { dash.clone() } else { parts.join(", ") }, theme::LEGEND)
        }
        crate::tracks::Detail::MeshCore { role, .. } => (role.to_string(), theme::LEGEND),
        // The protocol said where it was and not what it is,
        // and the packet list is where its fields are.
        crate::tracks::Detail::Device => (dash.clone(), theme::LEGEND),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tracks::{Detail, Track, TrackId};

    const ID_COLUMN: std::ops::Range<f32> = 100.0..200.0;

    fn clicks_that_open(track: &Track, xs: std::ops::Range<f32>) -> Vec<(Pos2, String)> {
        let ctx = egui::Context::default();
        crate::ui::install(&ctx);
        let now = std::time::Instant::now();
        let frame = |events: Vec<egui::Event>| {
            let input = egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1600.0, 400.0))),
                events,
                ..Default::default()
            };
            ctx.run_ui(input, |ui| Map::track_rows(ui, &[track], now))
                .platform_output
                .commands
                .into_iter()
                .filter_map(|c| match c {
                    egui::OutputCommand::OpenUrl(u) => Some(u.url),
                    _ => None,
                })
                .collect::<Vec<_>>()
        };
        let button = |pos: Pos2, pressed: bool| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: Default::default(),
        };
        let mut opened = Vec::new();
        for row in 0..40 {
            let mut x = xs.start;
            while x < xs.end {
                let at = Pos2::new(x, row as f32 * table::ROW_H / 2.0);
                let mut urls = frame(vec![egui::Event::PointerMoved(at), button(at, true)]);
                urls.extend(frame(vec![egui::Event::PointerMoved(at), button(at, false)]));
                opened.extend(urls.into_iter().map(|u| (at, u)));
                x += 2.0;
            }
        }
        opened
    }

    fn vessel(mmsi: u32) -> Track {
        let detail = Detail::Vessel {
            heading_deg: None,
            nav_status: None,
            ship_type: None,
            destination: None,
            class_b: false,
        };
        Track::new(TrackId::Mmsi(mmsi), detail, std::time::Instant::now())
    }

    #[test]
    fn the_vesselfinder_link_sits_after_the_mmsi_in_the_id_column() {
        let opened = clicks_that_open(&vessel(244650878), 0.0..1600.0);
        assert!(!opened.is_empty(), "nothing on the row opened VesselFinder");
        for (at, url) in &opened {
            assert_eq!(url, "https://www.vesselfinder.com/vessels/details/244650878");
            assert!(ID_COLUMN.contains(&at.x), "opened from {at:?}, outside the id column");
            assert!(at.x > ID_COLUMN.start + 40.0, "opened from {at:?}, on the MMSI itself");
        }
        let xs: Vec<f32> = opened.iter().map(|(at, _)| at.x).collect();
        let (lo, hi) = xs.iter().fold((f32::MAX, f32::MIN), |(l, h), x| (l.min(*x), h.max(*x)));
        assert!(
            hi - lo <= table::ROW_H + 10.0,
            "ceiling: one icon and egui's 5 px pick radius each side, got {lo}..{hi}"
        );
        let mut seen = std::collections::HashSet::new();
        assert!(
            opened.iter().all(|(at, _)| seen.insert((at.x as i32, at.y as i32))),
            "a click opened twice"
        );
    }

    #[test]
    fn an_aircraft_row_has_no_link() {
        let plane =
            Track::new(TrackId::Icao(0x4ca068), Detail::new_aircraft(), std::time::Instant::now());
        assert!(clicks_that_open(&plane, ID_COLUMN).is_empty());
    }
}
