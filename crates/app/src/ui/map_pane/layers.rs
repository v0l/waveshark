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
        "RANGE RINGS"
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
    /// How far out the position may be, in metres, when a fix said so. Drawn
    /// as the circle the receiver is somewhere inside, which is the honest
    /// shape of a GPS position: a mark alone claims a metre it does not have,
    /// and a bad fix looks exactly like a good one.
    pub accuracy_m: Option<f64>,
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
        // Under the mark, and only once it is larger than the mark: a
        // five metre circle at 50 nm to the screen is a ring inside the dot,
        // which reads as a decoration rather than as an error bar.
        if let Some(m) = self.accuracy_m {
            let r = (m / 1852.0 * c.nm_px_at(lat)) as f32;
            if r > 6.0 {
                c.p.circle_filled(at, r, theme::READOUT.gamma_multiply(0.08));
                c.p.circle_stroke(at, r, Stroke::new(1.0, theme::READOUT.gamma_multiply(0.35)));
            }
        }
        c.p.circle_stroke(at, 5.0, Stroke::new(1.5, theme::READOUT));
        c.p.circle_filled(at, 1.5, theme::READOUT);
    }
}

/// Where one device was heard from, and how well, and where that puts it.
///
/// Every point is a place the *receiver* stood when it heard the device, so
/// what is drawn is a trail along a road with a level at each point, not a
/// pin on the transmitter. The brightest point is the closest approach,
/// which is where to start walking. Where the levels along the drive can
/// say more, `survey::locate` says where the transmitter probably is, and
/// that is drawn as a mark inside the region the levels fit about as well:
/// a drive along one road gives a region that reaches across the road,
/// because levels carry no bearing and cannot say which side.
pub(super) struct SightingLayer<'a> {
    pub trail: &'a [survey::Sighting],
    /// The device the trail belongs to, for the status line.
    pub ident: Option<&'a str>,
    pub estimate: Option<survey::Estimate>,
}

impl Layer for SightingLayer<'_> {
    fn key(&self) -> &'static str {
        "sightings"
    }

    fn label(&self) -> &'static str {
        "SIGHTINGS"
    }

    fn draw(&mut self, c: &Canvas) {
        let points: Vec<(Pos2, Option<f32>)> =
            self.trail.iter().filter_map(|s| Some((c.at(s.lat?, s.lon?), s.rssi_dbfs))).collect();
        if points.is_empty() {
            return;
        }
        // The levels present, so the scale is the drive's own range rather
        // than an absolute one: a survey in a city and one in a field have
        // different floors and both want the strongest point to stand out.
        let levels: Vec<f32> = points.iter().filter_map(|(_, r)| *r).collect();
        let lo = levels.iter().cloned().fold(f32::INFINITY, f32::min);
        let hi = levels.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        let span = (hi - lo).max(1.0);
        for (at, rssi) in &points {
            // A sighting with no level still happened, and is drawn at the
            // dimmest end rather than dropped.
            let strength = rssi.map(|r| ((r - lo) / span).clamp(0.0, 1.0)).unwrap_or(0.0);
            let alpha = 0.25 + 0.75 * strength;
            c.p.circle_filled(*at, 2.0 + 3.0 * strength, theme::TRACE.gamma_multiply(alpha));
        }
        // The strongest point marked, because that is the one a person is
        // looking for: it is where to start walking.
        if let Some((at, _)) = points
            .iter()
            .zip(self.trail.iter())
            .filter(|(_, s)| s.rssi_dbfs.is_some())
            .max_by(|a, b| a.1.rssi_dbfs.unwrap().total_cmp(&b.1.rssi_dbfs.unwrap()))
            .map(|(p, _)| p)
        {
            c.p.circle_stroke(*at, 8.0, Stroke::new(1.5, theme::TRACE));
        }
        // Where the levels put it: the region first, then the point, drawn
        // in the readout colour rather than the trail's so a conclusion is
        // not mistaken for a measurement.
        if let Some(est) = &self.estimate {
            let at = c.at(est.lat, est.lon);
            let r = (est.radius_m / 1852.0 * c.nm_px_at(est.lat)) as f32;
            c.p.circle_filled(at, r.max(4.0), theme::READOUT.gamma_multiply(0.10));
            c.p.circle_stroke(at, r.max(4.0), Stroke::new(1.0, theme::READOUT.gamma_multiply(0.6)));
            let arm = 7.0;
            c.p.line_segment(
                [at - Vec2::new(arm, 0.0), at + Vec2::new(arm, 0.0)],
                Stroke::new(1.5, theme::READOUT),
            );
            c.p.line_segment(
                [at - Vec2::new(0.0, arm), at + Vec2::new(0.0, arm)],
                Stroke::new(1.5, theme::READOUT),
            );
        }
    }

    fn status(&self) -> Option<String> {
        let n = self.trail.iter().filter(|s| s.lat.is_some()).count();
        if n == 0 {
            return None;
        }
        let mut s = match self.ident {
            Some(id) => format!("{id}: {n} sightings"),
            None => format!("{n} sightings"),
        };
        match &self.estimate {
            Some(e) => s.push_str(&format!(
                ", likely within {} of {:.5}, {:.5} (fit {:.1} dB)",
                if e.radius_m >= 1000.0 {
                    format!("{:.1} km", e.radius_m / 1000.0)
                } else {
                    format!("{:.0} m", e.radius_m)
                },
                e.lat,
                e.lon,
                e.residual_db
            )),
            None => s.push_str(", too few places to locate it from"),
        }
        Some(s)
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

    /// Only while its markers are on screen. A credit for data that is not
    /// being shown is noise in the corner, and the licence asks for a credit
    /// where the data is used rather than everywhere it might be.
    fn credits(&self) -> Vec<crate::data::Credit> {
        match self.shown.is_empty() {
            true => Vec::new(),
            false => vec![crate::data::Which::Airports.credit()],
        }
    }
}

/// Where weather balloons go up from, as SondeHub's crowd-sourced list of
/// upper-air stations has it.
///
/// The only transmitter on this map whose next appearance is published: a
/// station launches at a filed hour, so a site is drawn with the time until
/// its next flight. Amber like the airports, because a launch site is a
/// fixed facility rather than something in the air, and hollow where the
/// site flies nothing this receiver can read.
#[derive(Default)]
pub(super) struct SondeLayer {
    /// The sites held this frame. Kept as a handle rather than as borrowed
    /// rows, because a refresh replaces the list and a hover reads it after
    /// every layer has drawn.
    sites: Option<std::sync::Arc<Vec<datasets::sondehub::Site>>>,
    /// Where each drawn site landed on screen, as an index into `sites`.
    shown: Vec<(Pos2, usize)>,
}

impl Layer for SondeLayer {
    fn key(&self) -> &'static str {
        "sondes"
    }

    fn label(&self) -> &'static str {
        "SONDES"
    }

    fn draw(&mut self, c: &Canvas) {
        if c.zoom() < SITE_ZOOM {
            self.shown.clear();
            return;
        }
        // Asking is what downloads it, so the file is fetched when somebody
        // switches this on rather than at every start.
        self.sites = crate::data::launch_sites();
        let Some(sites) = self.sites.clone() else {
            self.shown.clear();
            return;
        };
        let near = c.rect.expand(20.0);
        self.shown = sites
            .iter()
            .enumerate()
            .filter_map(|(i, s)| {
                let at = c.at(s.lat, s.lon);
                near.contains(at).then_some((at, i))
            })
            .collect();
        let readable = datasets::sondehub::Model::Rs41;
        for (at, i) in &self.shown {
            let s = &sites[*i];
            let col = theme::READOUT.gamma_multiply(0.85);
            if s.flies(readable) {
                c.p.circle_filled(*at, 3.0, col);
            } else {
                c.p.circle_stroke(*at, 3.0, Stroke::new(1.0, col.gamma_multiply(0.6)));
            }
            // The balloon over the mark, so a site reads as a launch rather
            // than as another airfield.
            c.p.line_segment(
                [Pos2::new(at.x, at.y - 4.0), Pos2::new(at.x, at.y - 8.0)],
                Stroke::new(1.0, col.gamma_multiply(0.5)),
            );
            c.p.circle_stroke(Pos2::new(at.x, at.y - 10.0), 2.5, Stroke::new(1.0, col));
        }
    }

    fn over(&mut self, c: &Canvas) {
        if c.zoom() < SITE_ZOOM {
            return;
        }
        let Some(sites) = self.sites.clone() else { return };
        let Some(pos) = c.hover() else { return };
        if let Some((at, i)) = nearest(&self.shown, pos, 12.0) {
            site_card(&c.p, c.rect, at, &sites[i]);
        }
    }

    fn status(&self) -> Option<String> {
        let now = crate::data::utc_now();
        let sites = self.sites.clone().unwrap_or_default();
        let next = self
            .shown
            .iter()
            .filter_map(|(_, i)| sites.get(*i))
            .filter_map(|s| s.next_launch(now).map(|(d, _)| (d, s.name.as_str())))
            .min_by_key(|(d, _)| *d);
        let sites = |n: usize| match n {
            1 => "1 sonde site".to_string(),
            n => format!("{n} sonde sites"),
        };
        match (self.shown.len(), next) {
            (0, _) => None,
            (n, None) => Some(sites(n)),
            (n, Some((d, name))) => Some(format!("{}, next {name} in {}", sites(n), until(d))),
        }
    }

    fn credits(&self) -> Vec<crate::data::Credit> {
        match self.shown.is_empty() {
            true => Vec::new(),
            false => vec![crate::data::Which::LaunchSites.credit()],
        }
    }
}

/// Below this the sites are not drawn. Lower than the airports' threshold
/// because there are 900 upper-air stations in the world against tens of
/// thousands of airfields: at a country's width they are a scatter of marks
/// rather than a wall, and a balloon two hundred kilometres away is still
/// one this receiver will hear.
const SITE_ZOOM: f64 = 6.0;

/// A duration as a person reads a countdown: hours and minutes up to a day,
/// days and hours beyond it.
fn until(d: std::time::Duration) -> String {
    let mins = d.as_secs() / 60;
    match (mins / 60, mins % 60) {
        (0, m) => format!("{m} min"),
        (h, m) if h < 24 => format!("{h} h {m:02} min"),
        (h, _) => format!("{} d {} h", h / 24, h % 24),
    }
}

/// The marker nearest the pointer within `px`, for a hover card.
fn nearest(shown: &[(Pos2, usize)], pos: Pos2, px: f32) -> Option<(Pos2, usize)> {
    let mut best: Option<(f32, Pos2, usize)> = None;
    for (at, i) in shown {
        let d = at.distance(pos);
        if d <= px && best.is_none_or(|(b, _, _)| d < b) {
            best = Some((d, *at, *i));
        }
    }
    best.map(|(_, at, i)| (at, i))
}

/// What a hovered launch site says: the station, what it flies, when it
/// flies it, and anything the contributors noted.
fn site_card(p: &egui::Painter, rect: Rect, anchor: Pos2, s: &datasets::sondehub::Site) {
    const MAX_ROWS: usize = 8;
    let (pad, sep, rule_gap) = (8.0, 4.0, 6.0);
    let font = |sz: f32| theme::figure(sz);
    let text_max = (f64::from(rect.width()) - 2.0 * f64::from(pad) - 8.0).clamp(80.0, 260.0) as f32;

    let name = p.layout(s.name.clone(), font(13.0), theme::VALUE, text_max);
    let mut meta = format!("SONDE SITE   {} M", s.alt_m.round());
    if let Some(b) = s.burst_m {
        meta.push_str(&format!("   BURST {:.0} KM", b / 1000.0));
    }
    let meta = p.layout_no_wrap(meta, font(9.0), theme::LEGEND);
    let station = p.layout_no_wrap(format!("WMO {}", s.station), font(11.0), theme::READOUT);

    let mut rows: Vec<(std::sync::Arc<egui::Galley>, Color32)> = Vec::new();
    let mut row = |text: String, col: Color32, size: f32| {
        rows.push((p.layout(text, font(size), col, text_max), col));
    };
    let sondes: Vec<String> = s
        .sondes
        .iter()
        .map(|t| match t.hz {
            Some(hz) => format!("{} {:.3} MHz", t.label(), hz / 1e6),
            None => t.label(),
        })
        .collect();
    match sondes.is_empty() {
        true => row("instrument not filed".into(), theme::LEGEND, 10.0),
        false => row(sondes.join(", "), theme::VALUE, 11.0),
    }

    let now = crate::data::utc_now();
    match s.next_launch(now) {
        Some((d, l)) => row(
            format!("next in {}, {:02}:{:02} UTC", until(d), l.hour, l.minute),
            theme::TRACE,
            11.0,
        ),
        None => row("schedule not known".into(), theme::LEGEND, 10.0),
    }
    for l in s.schedule.iter().take(MAX_ROWS) {
        row(format!("{:<10} {:02}:{:02} UTC", l.day.label(), l.hour, l.minute), theme::VALUE, 11.0);
    }
    if s.schedule.len() > MAX_ROWS {
        row(format!("+{} more", s.schedule.len() - MAX_ROWS), theme::LEGEND, 10.0);
    }
    if !s.notes.is_empty() {
        row(s.notes.clone(), theme::LEGEND, 10.0);
    }

    let head = [(name, theme::VALUE), (meta, theme::LEGEND), (station, theme::READOUT)];
    let head_sizes: Vec<Vec2> = head.iter().map(|(g, _)| g.size()).collect();
    let row_sizes: Vec<Vec2> = rows.iter().map(|(g, _)| g.size()).collect();
    let l = card_layout(&head_sizes, &row_sizes, pad, sep, rule_gap);

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

/// Cells from the OpenCelliD export, at the position the crowd averaged for
/// each and with the radius that position is good to.
///
/// Somebody else's claim about where a mast is, so it is drawn dimmer than
/// anything this receiver measured and the radius is drawn rather than
/// hidden: half these positions come from a phone in a car going past, and a
/// dot alone would claim a street the export does not know.
#[derive(Default)]
pub(super) struct CellLayer<'a> {
    /// Cells this receiver decoded, from the survey. The export is what the
    /// crowd knows; these are what was heard, and the ones the export has no
    /// row for are what beaconDB is asked about.
    pub heard: &'a [survey::Device],
    shown: Vec<(Pos2, datasets::cells::Cell)>,
    /// Cells the export does not have, placed where beaconDB says a receiver
    /// hearing them is, with the metres that is good to.
    guessed: Vec<(Pos2, survey::beacondb::Cell, f64)>,
    /// Why nothing was drawn, for the status line. A layer switched on and
    /// silent is indistinguishable from a country with no masts in it.
    quiet: Option<&'static str>,
}

/// Below this the export is a wall of dots over a whole city. Higher than
/// the airports' threshold because a country has a hundred airports and tens
/// of thousands of cells.
const CELL_ZOOM: f64 = 12.0;

impl<'a> CellLayer<'a> {
    /// Around the cells the survey holds, which is what the beaconDB lookup
    /// has anything to say about.
    pub fn new(heard: &'a [survey::Device]) -> Self {
        Self { heard, ..Default::default() }
    }

    /// Cells this receiver heard that the export has no row for, placed
    /// where beaconDB says.
    ///
    /// Drawn as the accuracy it came with rather than as a mast: beaconDB
    /// answers where a receiver seeing this cell probably is, which with one
    /// cell and nothing else is that cell's own estimated position and worth
    /// no more precision than the circle around it.
    fn draw_heard(&mut self, c: &Canvas, near: Rect, export: Option<&datasets::cells::Cells>) {
        if !crate::beacondb::lookup_on() {
            return;
        }
        for d in self.heard.iter().filter(|d| d.protocol == "gsm") {
            let Some(cell) = survey::beacondb::cell(&d.ident) else {
                continue;
            };
            let known = export.is_some_and(|e| {
                e.get(cell.mcc, &cell.mnc.to_string(), cell.lac, cell.cid).is_some()
            });
            if known {
                continue;
            }
            let Some(crate::beacondb::Answer::At { lat, lon, accuracy_m }) =
                crate::beacondb::position(cell)
            else {
                continue;
            };
            let at = c.at(lat, lon);
            if !near.contains(at) {
                continue;
            }
            let r = (accuracy_m / 1852.0 * c.nm_px_at(lat)) as f32;
            if r > 3.0 {
                c.p.circle_stroke(at, r, Stroke::new(1.0, theme::TRACE.gamma_multiply(0.18)));
            }
            // A cross, not a dot: this is somebody's estimate of a place, and
            // it must not read as a mast the export has a position for.
            let arm = 3.5;
            let dim = theme::TRACE.gamma_multiply(0.7);
            c.p.line_segment(
                [Pos2::new(at.x - arm, at.y), Pos2::new(at.x + arm, at.y)],
                Stroke::new(1.0, dim),
            );
            c.p.line_segment(
                [Pos2::new(at.x, at.y - arm), Pos2::new(at.x, at.y + arm)],
                Stroke::new(1.0, dim),
            );
            self.guessed.push((at, cell, accuracy_m));
        }
    }
}

impl Layer for CellLayer<'_> {
    fn key(&self) -> &'static str {
        "cells"
    }

    fn label(&self) -> &'static str {
        "CELL TOWERS"
    }

    fn draw(&mut self, c: &Canvas) {
        self.shown.clear();
        self.guessed.clear();
        self.quiet = None;
        if c.zoom() < CELL_ZOOM {
            self.quiet = Some("zoom in");
            return;
        }
        let near = c.rect.expand(30.0);
        // Asking is what starts the download, so the layer being switched on
        // is what fetches the export rather than the receiver fetching it in
        // case somebody looks.
        let cells = crate::data::cell_towers();
        if let Some(cells) = &cells {
            // A linear pass over the country's rows: they are sorted by
            // identity rather than by position, and at this zoom the window
            // is a few streets, so anything spatial would be an index built
            // for a filter that already costs less than the draw.
            for cell in cells.iter() {
                let at = c.at(cell.lat, cell.lon);
                if !near.contains(at) {
                    continue;
                }
                let r = (f64::from(cell.range_m) / 1852.0 * c.nm_px_at(cell.lat)) as f32;
                if r > 3.0 {
                    c.p.circle_stroke(at, r, Stroke::new(1.0, theme::OK.gamma_multiply(0.22)));
                }
                c.p.circle_filled(at, 2.5, theme::OK.gamma_multiply(0.75));
                self.shown.push((at, cell.clone()));
            }
        }
        self.draw_heard(c, near, cells.as_deref());
        if cells.is_none() && self.guessed.is_empty() {
            self.quiet = Some(crate::data::Which::CellTowers.blocked().unwrap_or("not downloaded"));
        }
    }

    fn over(&mut self, c: &Canvas) {
        let Some(pos) = c.hover() else { return };
        if let Some((at, cell, accuracy_m)) = nearest_guess(&self.guessed, pos) {
            let who = network_name(cell.mcc, &cell.mnc.to_string());
            let line = format!(
                "{}-{} LAC {} CI {} {} beaconDB estimate ±{:.0} m",
                cell.mcc, cell.mnc, cell.lac, cell.cid, who, accuracy_m
            );
            c.label(Pos2::new(at.x + 8.0, at.y - 6.0), &line, theme::TRACE, 1.0);
            return;
        }
        let Some((at, cell)) = nearest_cell(&self.shown, pos) else {
            return;
        };
        // The network's name where the operator table has landed, and the
        // codes either way: a beacon gives numbers, and a card that shows
        // only a brand cannot be matched against what was decoded.
        let who = network_name(cell.mcc, &cell.mnc);
        let line = format!(
            "{} {}-{} LAC {} CI {} {} ±{} m, {} reports",
            cell.radio, cell.mcc, cell.mnc, cell.area, cell.cell, who, cell.range_m, cell.samples
        );
        c.label(Pos2::new(at.x + 8.0, at.y - 6.0), &line, theme::VALUE, 1.0);
    }

    fn status(&self) -> Option<String> {
        if let Some(why) = self.quiet {
            return Some(format!("cells: {why}"));
        }
        match (self.shown.len(), self.guessed.len()) {
            (0, 0) => None,
            (n, 0) => Some(format!("{n} cells")),
            (0, g) => Some(format!("{g} cells from beaconDB")),
            (n, g) => Some(format!("{n} cells, {g} from beaconDB")),
        }
    }

    /// Asked for in writing by OpenCelliD: a visible credit and a link, for
    /// as long as their masts are on the screen, and only then. beaconDB is
    /// named on the same terms, while one of its estimates is drawn.
    fn credits(&self) -> Vec<crate::data::Credit> {
        let mut out = Vec::new();
        if !self.shown.is_empty() {
            out.push(crate::data::Which::CellTowers.credit());
        }
        if !self.guessed.is_empty() {
            out.push(crate::beacondb::CREDIT);
        }
        out
    }
}

/// What subscribers call the network an MCC and MNC belong to, or a phrase
/// saying nobody knows: a card showing only numbers is a card that cannot be
/// read, and one showing only a brand cannot be matched against a decode.
fn network_name(mcc: u16, mnc: &str) -> String {
    crate::data::cell_operators()
        .and_then(|ops| ops.get(mcc, mnc).map(|o| o.brand.clone()))
        .filter(|b| !b.is_empty())
        .unwrap_or_else(|| "unknown network".into())
}

/// The beaconDB estimate under the pointer, within a marker's grabbing
/// distance.
fn nearest_guess(
    shown: &[(Pos2, survey::beacondb::Cell, f64)],
    pos: Pos2,
) -> Option<(Pos2, survey::beacondb::Cell, f64)> {
    const PX: f32 = 10.0;
    shown
        .iter()
        .map(|(at, cell, acc)| (at.distance(pos), at, cell, acc))
        .filter(|(d, ..)| *d <= PX)
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, at, cell, acc)| (*at, *cell, *acc))
}

/// The cell under the pointer, within a marker's grabbing distance.
fn nearest_cell(
    shown: &[(Pos2, datasets::cells::Cell)],
    pos: Pos2,
) -> Option<(Pos2, &datasets::cells::Cell)> {
    const PX: f32 = 10.0;
    shown
        .iter()
        .map(|(at, cell)| (at.distance(pos), at, cell))
        .filter(|(d, _, _)| *d <= PX)
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, at, cell)| (*at, cell))
}

/// Aircraft, vessels and stations, with the trail each one came along.
pub(super) struct TrackLayer<'a> {
    pub active: &'a [&'a crate::tracks::Track],
    pub now: std::time::Instant,
    pub named_airframes: bool,
}

impl Layer for TrackLayer<'_> {
    fn key(&self) -> &'static str {
        "tracks"
    }

    fn label(&self) -> &'static str {
        "TRACKS"
    }

    fn draw(&mut self, c: &Canvas) {
        let fleet = self
            .active
            .iter()
            .any(|a| matches!(a.id, crate::tracks::TrackId::Icao(_)))
            .then(crate::data::aircraft)
            .flatten();
        self.named_airframes = false;
        for a in self.active {
            let Some((lat, lon)) = a.position else {
                continue;
            };
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
                        Stroke::new(2.5, theme::TRACE.gamma_multiply((0.25 + 0.65 * along) * fade)),
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
            let airframe = a.airframe(fleet.as_deref());
            self.named_airframes |= airframe.is_some();
            let class = airframe.and_then(|p| p.class);
            let reach = if a.confirmed {
                track_mark(&c.p, at, a.kind(), a.course_deg, col, class)
            } else {
                c.p.circle_stroke(at, 3.5, Stroke::new(1.0, col.gamma_multiply(0.7)));
                0.0
            };
            let text_x = at.x + (reach + 3.0).max(9.0);
            let label = a
                .label
                .clone()
                .or_else(|| airframe.map(|p| p.registration.to_string()).filter(|r| !r.is_empty()))
                .unwrap_or_else(|| a.id.text());
            c.label(Pos2::new(text_x, at.y - 5.0), &label, theme::VALUE, fade);
            // The second line is whatever that kind is measured by: an
            // aircraft by its altitude, a vessel by its speed. A station is
            // fixed and has neither.
            let under = match a.kind() {
                crate::tracks::Kind::Aircraft => a.altitude_ft().map(|ft| format!("{ft} ft")),
                // Metres, as a sonde reports and as the people who chase them
                // talk: a balloon at 35 km is not "114,829 ft" to anybody.
                crate::tracks::Kind::Sonde => match a.detail {
                    crate::tracks::Detail::Sonde { altitude_m, climb_ms, .. } => {
                        Some(format!("{altitude_m:.0} m  {climb_ms:+.1} m/s"))
                    }
                    _ => None,
                },
                crate::tracks::Kind::Vessel | crate::tracks::Kind::Vehicle => {
                    a.speed_kt.filter(|v| *v > 0.0).map(|kt| format!("{kt:.0} kt"))
                }
                // Nothing is known about it beyond where it was, and a
                // speed is only there if the message carried one.
                crate::tracks::Kind::Transmitter => {
                    a.speed_kt.filter(|v| *v > 0.0).map(|kt| format!("{kt:.0} kt"))
                }
                crate::tracks::Kind::Station => None,
            };
            if let Some(t) = under {
                c.label(Pos2::new(text_x, at.y + 5.0), &t, theme::LEGEND, fade);
            }
        }
    }

    fn status(&self) -> Option<String> {
        let n = self.active.iter().filter(|a| a.position.is_some()).count();
        Some(format!("{n} plotted"))
    }

    fn credits(&self) -> Vec<crate::data::Credit> {
        match self.named_airframes {
            true => vec![crate::data::Which::Aircraft.credit()],
            false => Vec::new(),
        }
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
    let font = |sz: f32| theme::figure(sz);

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
        let g = p.layout_no_wrap("no published frequencies".to_string(), font(10.0), theme::LEGEND);
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

    let head = [(name, theme::VALUE), (meta, theme::LEGEND), (ident, theme::READOUT)];
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
    class: Option<datasets::aircraft::Class>,
) -> f32 {
    use crate::tracks::Kind;
    if let (Kind::Aircraft, Some(class)) = (kind, class) {
        let balloon = class == datasets::aircraft::Class::Balloon;
        if let Some(course) = course_deg.or(balloon.then_some(0.0)) {
            super::silhouette::draw(p, at, class, course, col);
            return super::silhouette::span(class) / 2.0;
        }
    }
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
        return 0.0;
    }
    let Some(track) = course_deg else {
        p.circle_filled(at, 3.0, col);
        return 0.0;
    };
    let t = (track as f32).to_radians();
    let (s, c) = (t.sin(), t.cos());
    // Course is clockwise from north, and north is up, so a point ahead of
    // the track is (sin, -cos) in screen coordinates.
    let rot = |x: f32, y: f32| Pos2::new(at.x + x * c + y * s, at.y + x * s - y * c);
    let shape = match kind {
        // A balloon and what hangs under it: a circle above the fix, drawn
        // whichever way the wind is taking it, since a sonde has no heading
        // of its own.
        Kind::Sonde => {
            p.circle_filled(Pos2::new(at.x, at.y - 4.0), 3.5, col);
            p.line_segment(
                [Pos2::new(at.x, at.y - 1.0), Pos2::new(at.x, at.y + 4.0)],
                Stroke::new(1.0, col),
            );
            return 0.0;
        }
        Kind::Aircraft => {
            vec![rot(0.0, 6.0), rot(-4.0, -4.0), rot(0.0, -1.5), rot(4.0, -4.0)]
        }
        // Longer and narrower, with a squared stern: a hull rather than a
        // wing.
        Kind::Vessel => {
            vec![rot(0.0, 7.0), rot(-2.5, 2.0), rot(-2.5, -5.0), rot(2.5, -5.0), rot(2.5, 2.0)]
        }
        // Short and blunt, which is neither of the other two at a glance.
        _ => vec![rot(0.0, 4.5), rot(-3.0, 1.0), rot(-3.0, -3.0), rot(3.0, -3.0), rot(3.0, 1.0)],
    };
    p.add(egui::Shape::convex_polygon(shape, col, Stroke::NONE));
    0.0
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
    let text_w = head.iter().chain(rows).map(|s| s.x).fold(0.0f32, f32::max);
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
    CardLayout { size: Vec2::new(text_w + pad * 2.0, y + pad), ys, rule_y, text_x: pad }
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
        let head = [Vec2::new(180.0, 15.0), Vec2::new(90.0, 11.0), Vec2::new(40.0, 13.0)];
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

/// Satellites: the path each one is on, and where it is now.
///
/// Drawn from the elements rather than from anything heard, so unlike a
/// track it is a prediction and is drawn as one: a thin line for where the
/// sub-point runs, brighter ahead of the satellite than behind it, and a
/// ring at the place it is over at this instant.
///
/// Only what is being watched. Propagating a hundred objects a frame is
/// cheap, but a hundred ground tracks drawn over each other is a map you
/// cannot read, so the layer draws the selected satellite and whatever is
/// above the horizon from here.
pub(super) struct SatLayer {
    pub sky: Option<std::sync::Arc<crate::sats::Sky>>,
    /// Which group they came from, so the credit names the right row.
    pub group: &'static datasets::tle::Group,
    /// The one the pass table has selected, drawn whether or not it is up.
    pub selected: Option<u64>,
    pub home: Option<(f64, f64)>,
    pub now_s: i64,
    /// Where each drawn satellite ended up, for the hover card.
    shown: Vec<(Pos2, String, f64, f64)>,
    /// Catalogue numbers as drawn, beside `shown`, so a click can name what
    /// it landed on.
    ids: Vec<u64>,
    /// What was clicked this frame, for the pane to select.
    pub hit: Option<u64>,
}

impl SatLayer {
    pub fn new(
        sky: Option<std::sync::Arc<crate::sats::Sky>>,
        group: &'static datasets::tle::Group,
        selected: Option<u64>,
        home: Option<(f64, f64)>,
        now_s: i64,
    ) -> Self {
        Self { sky, group, selected, home, now_s, shown: Vec::new(), ids: Vec::new(), hit: None }
    }
}

/// A minute a sample draws a low orbit to within a few kilometres of itself,
/// which is finer than any zoom that fits a whole orbit on screen.
const TRACK_STEP_S: i64 = 60;

impl Layer for SatLayer {
    fn key(&self) -> &'static str {
        "satellites"
    }

    fn label(&self) -> &'static str {
        "SATELLITES"
    }

    fn draw(&mut self, c: &Canvas) {
        self.shown.clear();
        self.ids.clear();
        self.hit = None;
        let Some(sky) = self.sky.clone() else { return };
        let station = self.home.map(|(lat, lon)| orbit::Station::new(lat, lon));
        for sat in sky.sats() {
            let up = station.and_then(|s| sat.look(s, self.now_s)).is_some_and(|l| l.el_deg > 0.0);
            let picked = self.selected == Some(sat.norad);
            if !up && !picked {
                continue;
            }
            let Some((lat, lon, alt_km)) = sat.subpoint(self.now_s) else {
                continue;
            };
            // One orbit, half of it behind and half ahead, so the path says
            // where it came from as well as where it is going.
            let period = sat.period_s() as i64;
            let steps = (period / TRACK_STEP_S).clamp(8, 240) as usize;
            let start = self.now_s - period / 2;
            let track = sat.ground_track(start, TRACK_STEP_S, steps + 1);
            let bright = if picked { theme::READOUT } else { theme::TRACE };
            // Whether any of this satellite ends up on screen. A pass over
            // the other side of the world is above the horizon and drawn
            // nowhere, and counting it as drawn credited CelesTrak on a map
            // showing none of their data.
            let mut visible = false;
            let mut prev: Option<Pos2> = None;
            for (t, plat, plon) in track {
                let at = c.at(plat, plon);
                visible |= c.rect.contains(at);
                if let Some(last) = prev {
                    // A minute of flight is a short step on any map. One
                    // that lands half a world away is the far edge of it
                    // rather than a move, and drawing it would put a stroke
                    // across everything. Measured against the world and not
                    // against the pane: zoomed out, the whole world is
                    // narrower than the pane and every wrap would pass.
                    if (at.x - last.x).abs() < c.world_px() / 2.0 {
                        let ahead = t >= self.now_s;
                        let fade = if ahead { 0.55 } else { 0.22 };
                        c.p.line_segment([last, at], Stroke::new(1.0, bright.gamma_multiply(fade)));
                    }
                }
                prev = Some(at);
            }
            let at = c.at(lat, lon);
            // The circle of ground that can see it, which is the question a
            // map is being asked: whether the station at the other end of a
            // contact is inside it. Only for the one that was picked: half a
            // dozen circles two thousand kilometres across is a map of
            // circles. Drawn from the sub-point, so it is a circle on the
            // ground and not on the projection, at the scale of its centre.
            if picked {
                let r = (orbit::footprint_km(alt_km) / 1.852 * c.nm_px_at(lat)) as f32;
                if r > 4.0 {
                    // Filled as well as outlined. A hairline two thousand
                    // kilometres across is a circle you have to look for,
                    // and the answer it carries is which side of it a place
                    // is on, which a wash says at a glance and a line does
                    // not. Faint enough that the coastline under it still
                    // reads.
                    c.p.circle_filled(at, r, bright.gamma_multiply(0.06));
                    c.p.circle_stroke(at, r, Stroke::new(1.0, bright.gamma_multiply(0.45)));
                    // A footprint wide enough to cover the view is this
                    // satellite's data on screen even when the satellite
                    // itself is on the other side of the world, and it has
                    // to be credited like anything else that is drawn.
                    visible |= c.rect.distance_to_pos(at) <= r;
                }
            }
            c.p.circle_stroke(at, 4.0, Stroke::new(1.5, bright));
            c.p.circle_filled(at, 1.5, bright);
            if !(visible || c.rect.contains(at)) {
                continue;
            }
            self.shown.push((at, sat.name.clone(), lat, lon));
            self.ids.push(sat.norad);
        }
    }

    fn over(&mut self, c: &Canvas) {
        // Picked by the same reach as the hover card, so what a click
        // selects is what the pointer was naming.
        if let Some(pos) = c.click() {
            self.hit = self
                .shown
                .iter()
                .zip(&self.ids)
                .map(|((at, ..), id)| (at.distance(pos), *id))
                .filter(|(d, _)| *d <= 12.0)
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .map(|(_, id)| id);
        }
        let Some(pos) = c.hover() else { return };
        let Some((at, name, lat, lon)) = self
            .shown
            .iter()
            .map(|(at, name, lat, lon)| (at.distance(pos), *at, name.clone(), *lat, *lon))
            .filter(|(d, ..)| *d <= 12.0)
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, at, name, lat, lon)| (at, name, lat, lon))
        else {
            return;
        };
        let line = match self.home.map(|(hlat, hlon)| orbit::Station::new(hlat, hlon)) {
            Some(s) => match self.sky.as_ref().and_then(|sky| {
                sky.sats().iter().find(|x| x.name == name).and_then(|x| x.look(s, self.now_s))
            }) {
                Some(l) => format!(
                    "{name}  {:.2}, {:.2}  az {:.0} el {:.0}  {:.0} km",
                    lat, lon, l.az_deg, l.el_deg, l.range_km
                ),
                None => format!("{name}  {lat:.2}, {lon:.2}"),
            },
            None => format!("{name}  {lat:.2}, {lon:.2}"),
        };
        c.label(Pos2::new(at.x + 8.0, at.y - 6.0), &line, theme::VALUE, 1.0);
    }

    fn status(&self) -> Option<String> {
        (!self.shown.is_empty()).then(|| match self.shown.len() {
            1 => "1 satellite".into(),
            n => format!("{n} satellites"),
        })
    }

    fn credits(&self) -> Vec<crate::data::Credit> {
        match self.shown.is_empty() {
            true => Vec::new(),
            false => vec![crate::data::Which::Satellites(self.group).credit()],
        }
    }
}

const DISPATCH_WINDOW_S: f64 = 2.0 * 3600.0;
const DISPATCH_CARD_PAGES: usize = 3;

pub(super) struct DispatchLayer<'a> {
    pub messages: Vec<&'a crate::messages::Message>,
    shown: Vec<(Pos2, Vec<usize>)>,
    locating: usize,
}

impl<'a> DispatchLayer<'a> {
    pub fn new(messages: Vec<&'a crate::messages::Message>) -> Self {
        Self { messages, shown: Vec::new(), locating: 0 }
    }
}

fn dispatch_age_s(m: &crate::messages::Message) -> f64 {
    crate::messages::now_us().saturating_sub(m.at_us) as f64 / 1e6
}

impl Layer for DispatchLayer<'_> {
    fn key(&self) -> &'static str {
        "dispatch"
    }

    fn label(&self) -> &'static str {
        "DISPATCH"
    }

    fn draw(&mut self, c: &Canvas) {
        self.shown.clear();
        self.locating = 0;
        let ctx = c.p.ctx().clone();
        let mut placed: Vec<(datasets::geocode::Place, Vec<usize>)> = Vec::new();
        for (i, m) in self.messages.iter().enumerate() {
            let Some(dest) = m.destination.as_deref() else { continue };
            if dispatch_age_s(m) > DISPATCH_WINDOW_S {
                continue;
            }
            let Some(place) = crate::places::place_of(&ctx, dest) else {
                self.locating += 1;
                continue;
            };
            match placed.iter_mut().find(|(p, _)| p.label == place.label) {
                Some((_, pages)) => pages.push(i),
                None => placed.push((place, vec![i])),
            }
        }
        let near = c.rect.expand(30.0);
        for (place, pages) in placed {
            let at = c.at(place.lat, place.lon);
            if !near.contains(at) {
                continue;
            }
            let newest =
                pages.iter().map(|i| dispatch_age_s(self.messages[*i])).fold(f64::MAX, f64::min);
            let fade = (1.0 - newest / DISPATCH_WINDOW_S).clamp(0.25, 1.0) as f32;
            let col = theme::TRACE.gamma_multiply(fade);
            let r = (place.precision.radius_m() / 1852.0 * c.nm_px_at(place.lat)) as f32;
            if r > 6.0 {
                c.p.circle_stroke(at, r, Stroke::new(1.0, col.gamma_multiply(0.35)));
            }
            c.p.circle_filled(at, 4.0, col);
            c.p.circle_stroke(at, 6.5, Stroke::new(1.0, col.gamma_multiply(0.6)));
            self.shown.push((at, pages));
        }
    }

    fn over(&mut self, c: &Canvas) {
        let Some(pos) = c.hover() else { return };
        let hit = self
            .shown
            .iter()
            .map(|(at, pages)| (at.distance(pos), *at, pages))
            .filter(|(d, _, _)| *d <= 10.0)
            .min_by(|a, b| a.0.total_cmp(&b.0));
        let Some((_, at, pages)) = hit else { return };
        let mut y = at.y - 6.0;
        let mut newest: Vec<&crate::messages::Message> =
            pages.iter().map(|i| self.messages[*i]).collect();
        newest.sort_by_key(|m| std::cmp::Reverse(m.at_us));
        if let Some(dest) = newest.first().and_then(|m| m.destination.as_deref()) {
            y += c.label(Pos2::new(at.x + 10.0, y), dest, theme::VALUE, 1.0).height() + 2.0;
        }
        for m in newest.iter().take(DISPATCH_CARD_PAGES) {
            let line = format!("{}  {}", m.when(), m.text);
            y += c.label(Pos2::new(at.x + 10.0, y), &line, theme::TRACE, 1.0).height() + 2.0;
        }
        if newest.len() > DISPATCH_CARD_PAGES {
            let more = format!("+{} more", newest.len() - DISPATCH_CARD_PAGES);
            c.label(Pos2::new(at.x + 10.0, y), &more, theme::LEGEND, 1.0);
        }
    }

    fn status(&self) -> Option<String> {
        let pins = self.shown.len();
        match (pins, self.locating) {
            (0, 0) => None,
            (n, 0) => Some(format!("{n} dispatches")),
            (0, k) => Some(format!("locating {k} dispatches")),
            (n, k) => Some(format!("{n} dispatches, locating {k}")),
        }
    }

    fn credits(&self) -> Vec<crate::data::Credit> {
        if self.shown.is_empty() {
            return Vec::new();
        }
        let g = crate::places::geocoder();
        vec![crate::data::Credit { name: g.name(), licence: g.terms(), url: g.page() }]
    }
}
