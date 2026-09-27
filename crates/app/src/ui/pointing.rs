use super::sats_pane::{self, Downlink};
use super::state::Pointing;
use super::*;
use egui_bench::form::footer;
use egui_bench::panel::section;
use egui_bench::readout::readout;

const WIDTH: f32 = 560.0;
const PASS_EVERY_S: i64 = 30;
const PASS_LOOKBACK_S: i64 = 3_600;
const PASS_WINDOW_S: i64 = 2 * 86_400;
const STALE_DAYS: f64 = 7.0;
const NARROWEST_HZ: f64 = 5_000.0;
const WIDEST_HZ: f64 = 200_000.0;

pub(super) struct Heard {
    pub label: String,
    pub hz: f64,
    pub level_db: Option<f32>,
    pub mer_db: Option<f32>,
    pub following: bool,
}

pub(super) struct Span<'a> {
    pub db: &'a [f32],
    pub center_hz: f64,
    pub rate_hz: f64,
}

pub(super) enum Out {
    Listen(Downlink),
    Stop,
    ShowOnMap,
}

pub(super) struct PointingModal<'a> {
    pub st: &'a mut Pointing,
    pub sat: &'a orbit::Sat,
    pub station: orbit::Station,
    pub now: i64,
    pub down: Option<datasets::satnogs::Transmitter>,
    pub following: bool,
    pub span: Span<'a>,
    pub heard: Vec<Heard>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Best {
    Mer,
    AboveFloor,
}

impl Best {
    fn name(self) -> &'static str {
        match self {
            Best::Mer => "MER",
            Best::AboveFloor => "above the floor",
        }
    }
}

impl PointingModal<'_> {
    pub fn show(self, ctx: &egui::Context) -> (bool, Vec<Out>) {
        let PointingModal { st, sat, station, now, down, following, span, heard } = self;
        let mut out = Vec::new();
        let mut close = false;
        let look = sat.look(station, now);
        let fixed = sat.stationary();
        let pass = if fixed { None } else { pass_of(st, sat, station, now) };
        let arriving = down.as_ref().and_then(|d| d.downlink_hz).map(|hz| hz as f64);
        let shifted = look.zip(arriving).map(|(l, hz)| l.doppler_hz(hz));
        let on_downlink = shifted.and_then(|hz| {
            let half = down.as_ref().and_then(|d| d.baud).map_or(NARROWEST_HZ, |b| b * 0.7);
            level_above_floor(&span, hz, half.clamp(NARROWEST_HZ / 2.0, WIDEST_HZ / 2.0))
        });
        let mer = heard
            .iter()
            .filter_map(|h| h.mer_db)
            .fold(None, |m: Option<f32>, v| Some(m.map_or(v, |m| m.max(v))));
        let reading = match (mer, on_downlink) {
            (Some(m), _) => Some((m, Best::Mer)),
            (None, Some(a)) => Some((a, Best::AboveFloor)),
            _ => None,
        };
        if let Some((v, kind)) = reading
            && st.best.is_none_or(|(b, k)| k != kind.name() || v > b)
        {
            st.best = Some((v, kind.name()));
        }
        let up = look.is_some_and(|l| l.el_deg > 0.0);
        let r = egui::containers::Modal::new(egui::Id::new("sat-pointing"))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(WIDTH);
                modal_title(ui, &sat.name);
                let w = ui.available_width();
                egui::ScrollArea::vertical()
                    .auto_shrink([false, true])
                    .max_height(super::settings::share_of_screen(ui, 0.7, 320.0, 760.0))
                    .show(ui, |ui| {
                        ui.set_max_width(w);
                        point_card(ui, sat, station, look, pass.as_ref(), fixed, now);
                        ui.add_space(6.0);
                        if fixed {
                            slot_card(ui, sat, station, look);
                        } else {
                            pass_card(ui, sat, station, look, pass.as_ref(), now);
                        }
                        ui.add_space(6.0);
                        if let Some(d) = down.as_ref() {
                            link_card(ui, d, look, sat.look(station, now + 1), shifted);
                            ui.add_space(6.0);
                        }
                        let listenable = down.as_ref().is_some_and(|d| d.downlink_hz.is_some());
                        let told = Advice { listenable, following, fixed };
                        if listenable || fixed || !heard.is_empty() {
                            receiver_card(ui, &heard, on_downlink, shifted, &span, st.best, told);
                            ui.add_space(6.0);
                        }
                        elements_card(ui, sat, now);
                    });
                footer(ui, |ui| {
                    if ui.button(crate::i18n::t("ui.close")).clicked() {
                        close = true;
                    }
                    let link =
                        down.as_ref().and_then(|d| sats_pane::downlink(sat.norad, &sat.name, d));
                    match following {
                        true => {
                            if ui.button("STOP").clicked() {
                                out.push(Out::Stop);
                            }
                        }
                        false => {
                            let can = link.is_some() && up;
                            let b = ui.add_enabled(can, egui::Button::new("LISTEN"));
                            let b = match (link.is_some(), up) {
                                (false, _) => b.on_disabled_hover_text("no downlink to listen to"),
                                (true, false) => b.on_disabled_hover_text("not above the horizon"),
                                (true, true) => b,
                            };
                            if b.clicked()
                                && let Some(l) = link
                            {
                                out.push(Out::Listen(l));
                            }
                        }
                    }
                    if ui.button("SHOW ON MAP").clicked() {
                        out.push(Out::ShowOnMap);
                    }
                    if st.best.is_some() && ui.button("RESET BEST").clicked() {
                        st.best = None;
                    }
                });
            });
        if r.should_close() {
            close = true;
        }
        ctx.request_repaint_after(std::time::Duration::from_millis(250));
        (close, out)
    }
}

fn point_card(
    ui: &mut egui::Ui,
    sat: &orbit::Sat,
    station: orbit::Station,
    look: Option<orbit::Look>,
    pass: Option<&orbit::Pass>,
    fixed: bool,
    now: i64,
) {
    section(ui, "point", &format!("#{} from {}", sat.norad, place(station)), |ui| {
        let Some(l) = look else {
            panel::status(ui, false, "the elements do not propagate to now");
            return;
        };
        let up = l.el_deg > 0.0;
        let tint = if up { theme::TRACE } else { theme::LEGEND };
        let mut big = vec![
            ("azimuth", format!("{:.1}\u{b0} {}", l.az_deg, compass(l.az_deg)), tint),
            ("elevation", format!("{:.1}\u{b0}", l.el_deg), tint),
        ];
        let skew = fixed.then(|| l.skew_deg(station));
        if let Some(d) = skew {
            big.push(("skew", sats_pane::skew(d), tint));
        }
        ui.horizontal_top(|ui| {
            ui.spacing_mut().item_spacing.x = 18.0;
            for (label, v, tint) in &big {
                readout(ui, label, v.clone(), *tint);
            }
            if let Some(d) = skew {
                help(ui, &sats_pane::skew_help(d));
            }
        });
        ui.add_space(4.0);
        reading(ui, "range", format!("{:.0} km", l.range_km));
        reading(
            ui,
            if l.range_rate_kms < 0.0 { "closing" } else { "opening" },
            format!("{:.3} km/s", l.range_rate_kms.abs()),
        );
        reading(ui, "delay", format!("{:.1} ms", l.delay_ms()));
        reading(
            ui,
            "overhead",
            format!("{} at {:.0} km", place(orbit::Station::new(l.lat_deg, l.lon_deg)), l.alt_km),
        );
        let (ok, said) = horizon(l, pass, fixed, now);
        panel::status(ui, ok, &said);
    });
}

fn horizon(l: orbit::Look, pass: Option<&orbit::Pass>, fixed: bool, now: i64) -> (bool, String) {
    match (l.el_deg > 0.0, fixed, pass) {
        (true, true, _) => (true, "above the horizon, and stays there".into()),
        (true, false, Some(p)) => (
            true,
            format!(
                "above the horizon, sets {} at {:.0}\u{b0} {}",
                crate::sats::in_when(p.set_s - now),
                p.set_az_deg,
                compass(p.set_az_deg)
            ),
        ),
        (true, false, None) => (true, "above the horizon".into()),
        (false, true, _) => (false, "below the horizon, and stays there from here".into()),
        (false, false, Some(p)) => (
            false,
            format!(
                "below the horizon, rises {} at {:.0}\u{b0} {}",
                crate::sats::in_when(p.rise_s - now),
                p.rise_az_deg,
                compass(p.rise_az_deg)
            ),
        ),
        (false, false, None) => (false, "below the horizon, no pass in the next two days".into()),
    }
}

fn pass_card(
    ui: &mut egui::Ui,
    sat: &orbit::Sat,
    station: orbit::Station,
    look: Option<orbit::Look>,
    pass: Option<&orbit::Pass>,
    now: i64,
) {
    let note = match pass {
        Some(p) if p.rise_s <= now => "the one in progress, times in UTC",
        Some(_) => "the next one, times in UTC",
        None => "none in the next two days",
    };
    section(ui, "pass", note, |ui| {
        let Some(p) = pass else { return };
        let at = |t: i64| sat.look(station, t);
        ui.columns(2, |col| {
            let ui = &mut col[0];
            let when_az = |t: i64, az: f64| {
                format!("{}, {:.0}\u{b0} {}", crate::sats::utc_hms(t), az, compass(az))
            };
            reading(ui, "rises", when_az(p.rise_s, p.rise_az_deg));
            let peak_az = at(p.peak_s).map_or(String::new(), |l| {
                format!(" at {:.0}\u{b0} {}", l.az_deg, compass(l.az_deg))
            });
            reading(
                ui,
                "peaks",
                format!("{}, {:.0}\u{b0}{peak_az}", crate::sats::utc_hms(p.peak_s), p.max_el_deg),
            );
            reading(ui, "sets", when_az(p.set_s, p.set_az_deg));
            reading(ui, "lasts", format!("{}m {:02}s", p.duration_s() / 60, p.duration_s() % 60));
            let arc = sat.arc(station, p.rise_s, p.set_s, sats_pane::ARC_POINTS);
            sats_pane::sky_plot(&mut col[1], &arc, &[], &[], look.as_ref());
        });
    });
}

fn slot_card(
    ui: &mut egui::Ui,
    sat: &orbit::Sat,
    station: orbit::Station,
    look: Option<orbit::Look>,
) {
    section(ui, "slot", "fixed in the sky from here", |ui| {
        ui.columns(2, |col| {
            let ui = &mut col[0];
            if let Some(l) = look {
                reading(ui, "slot", sats_pane::slot(l.lon_deg));
                reading(ui, "footprint", format!("{:.0} km", l.footprint_km()));
            }
            reading(ui, "inclined", format!("{:.2}\u{b0}", sat.inclination_deg));
            let drift = sat.drift_deg_per_day();
            reading(
                ui,
                "drifts",
                format!(
                    "{:.2}\u{b0} {} a day",
                    drift.abs(),
                    if drift > 0.0 { "east" } else { "west" }
                ),
            );
            let belt = orbit::clarke_belt(station, sats_pane::BELT_POINTS);
            sats_pane::sky_plot(&mut col[1], &[], &belt, &[], look.as_ref());
        });
    });
}

fn link_card(
    ui: &mut egui::Ui,
    d: &datasets::satnogs::Transmitter,
    look: Option<orbit::Look>,
    next: Option<orbit::Look>,
    shifted: Option<f64>,
) {
    section(ui, "link", &d.mode, |ui| {
        reading(ui, "downlink", d.label());
        if let (Some(hz), Some(s), Some(l)) = (d.downlink_hz, shifted, look) {
            let hz = hz as f64;
            reading(ui, "arrives", format!("{:.4} MHz, {:+.0} Hz", s / 1e6, s - hz));
            if let Some(n) = next {
                let drift = n.doppler_hz(hz) - l.doppler_hz(hz);
                reading(ui, "moving", format!("{drift:+.1} Hz a second"));
            }
            reading(ui, "free space", format!("{:.1} dB", l.path_loss_db(hz)));
        }
        if let (Some(u), Some(l)) = (d.uplink_hz, look) {
            let u = u as f64;
            reading(
                ui,
                "send on",
                format!("{:.4} MHz to land on {:.4}", send_on(u, &l) / 1e6, u / 1e6),
            );
        } else if let Some(up) = d.uplink_label() {
            reading(ui, "uplink", up);
        }
    });
}

fn receiver_card(
    ui: &mut egui::Ui,
    heard: &[Heard],
    on_downlink: Option<f32>,
    shifted: Option<f64>,
    span: &Span,
    best: Option<(f32, &'static str)>,
    told: Advice,
) {
    section(ui, "receiver", "peak the antenna on these", |ui| {
        let at_downlink = match (shifted, on_downlink) {
            (Some(_), Some(db)) => format!("{db:.1} dB above the floor"),
            (Some(hz), None) if !span.db.is_empty() => format!(
                "outside the span, {:.3} MHz is {:.3} to {:.3}",
                hz / 1e6,
                (span.center_hz - span.rate_hz / 2.0) / 1e6,
                (span.center_hz + span.rate_hz / 2.0) / 1e6
            ),
            (Some(_), None) => "no spectrum yet".into(),
            (None, _) => String::new(),
        };
        if shifted.is_some() {
            reading(ui, "level", at_downlink);
        }
        for h in heard {
            let mut said = Vec::new();
            if let Some(db) = h.level_db {
                said.push(format!("{db:.1} dB"));
            }
            if let Some(m) = h.mer_db {
                said.push(format!("MER {m:.1} dB"));
            }
            if said.is_empty() {
                said.push("nothing measured yet".into());
            }
            let who = format!("{} {:.3}", h.label, h.hz / 1e6);
            let tint = if h.following { theme::TRACE } else { theme::VALUE };
            Line::new().legend(&who).hanging(
                ui,
                egui_bench::readout::LABEL_W + 8.0,
                Line::new().value(said.join(", ")).tint(tint).size(11.0),
            );
        }
        if let Some((v, kind)) = best {
            Line::new().legend("best").hanging(
                ui,
                egui_bench::readout::LABEL_W + 8.0,
                Line::new().value(format!("{v:.1} dB {kind}")).tint(theme::READOUT).size(11.0),
            );
        }
        let mer = heard.iter().filter(|h| h.mer_db.is_some()).count();
        let (ok, said) = match (told.following, told.listenable, mer) {
            (true, _, _) => (true, "listening on the downlink, following the Doppler".to_string()),
            (false, true, _) => {
                (false, "not listening, LISTEN puts a channel on the downlink".into())
            }
            (false, false, _) if !told.fixed => {
                (true, "no downlink listed, measured on the channels above".into())
            }
            (false, false, 0) => (
                false,
                "no downlink listed, tune a DVB-S2 channel to a transponder and peak on its MER"
                    .into(),
            ),
            (false, false, 1) => (true, "reading MER off one DVB-S2 channel".into()),
            (false, false, n) => (true, format!("reading MER off {n} DVB-S2 channels")),
        };
        panel::status(ui, ok, &said);
    });
}

#[derive(Clone, Copy)]
struct Advice {
    listenable: bool,
    following: bool,
    fixed: bool,
}

fn elements_card(ui: &mut egui::Ui, sat: &orbit::Sat, now: i64) {
    section(ui, "elements", "what the prediction is fitted to", |ui| {
        let epoch = crate::segments::when(sat.epoch_s.max(0) as u64 * 1_000_000);
        reading(ui, "epoch", epoch.format("%Y-%m-%d %H:%M UTC").to_string());
        let age = sat.age_days(now);
        reading(ui, "age", format!("{age:.1} days"));
        reading(ui, "period", format!("{:.1} min", sat.period_s() / 60.0));
        match age > STALE_DAYS {
            true => panel::status(ui, false, "stale, refresh the elements in Data"),
            false => panel::status(ui, true, "fresh enough to point by"),
        }
    });
}

fn pass_of(
    st: &mut Pointing,
    sat: &orbit::Sat,
    station: orbit::Station,
    now: i64,
) -> Option<orbit::Pass> {
    let fresh =
        st.pass.is_some_and(|(at, p)| now - at < PASS_EVERY_S && p.is_none_or(|p| p.set_s >= now));
    if !fresh {
        st.pass = Some((now, current_or_next(sat, station, now)));
    }
    st.pass.and_then(|(_, p)| p)
}

fn current_or_next(sat: &orbit::Sat, station: orbit::Station, now: i64) -> Option<orbit::Pass> {
    sat.passes(station, now - PASS_LOOKBACK_S, PASS_LOOKBACK_S + PASS_WINDOW_S, 0.0)
        .into_iter()
        .find(|p| p.set_s >= now)
}

fn place(s: orbit::Station) -> String {
    format!(
        "{:.2}\u{b0}{} {:.2}\u{b0}{}",
        s.lat_deg.abs(),
        if s.lat_deg < 0.0 { "S" } else { "N" },
        s.lon_deg.abs(),
        if s.lon_deg < 0.0 { "W" } else { "E" }
    )
}

pub(super) fn compass(az_deg: f64) -> &'static str {
    const POINTS: [&str; 16] = [
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW",
        "NW", "NNW",
    ];
    let i = ((az_deg.rem_euclid(360.0) + 11.25) / 22.5) as usize % 16;
    POINTS[i]
}

pub(super) fn send_on(uplink_hz: f64, l: &orbit::Look) -> f64 {
    uplink_hz * uplink_hz / l.doppler_hz(uplink_hz)
}

pub(super) fn level_above_floor(span: &Span, hz: f64, half_width_hz: f64) -> Option<f32> {
    let n = span.db.len();
    if n < 3 || span.rate_hz <= 0.0 {
        return None;
    }
    let low = span.center_hz - span.rate_hz / 2.0;
    let bin = |f: f64| (f - low) / span.rate_hz * n as f64;
    let (a, b) = (bin(hz - half_width_hz), bin(hz + half_width_hz));
    if b < 0.0 || a >= n as f64 {
        return None;
    }
    let (a, b) = (a.floor().max(0.0) as usize, (b.ceil() as usize).min(n - 1));
    let peak = span.db[a..=b].iter().copied().fold(f32::NEG_INFINITY, f32::max);
    let mut sorted: Vec<f32> = span.db.iter().copied().filter(|v| v.is_finite()).collect();
    if sorted.is_empty() || !peak.is_finite() {
        return None;
    }
    sorted.sort_by(f32::total_cmp);
    Some(peak - sorted[sorted.len() / 2])
}

pub(super) fn mer_of(readings: &[(String, String)]) -> Option<f32> {
    let (_, v) = readings.iter().find(|(k, _)| k == "MER")?;
    v.trim_end_matches("dB").trim().parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    const ISS: (&str, &str) = (
        "1 25544U 98067A   24015.52815972  .00016717  00000-0  30074-3 0  9990",
        "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.49514029431344",
    );

    fn iss() -> orbit::Sat {
        orbit::Sat::from_lines("ISS (ZARYA)", ISS.0, ISS.1).expect("elements")
    }

    fn dublin() -> orbit::Station {
        orbit::Station::new(53.35, -6.26)
    }

    #[test]
    fn an_azimuth_is_named_by_the_nearest_of_sixteen_points() {
        let named: Vec<&str> =
            [0.0, 11.2, 11.3, 90.0, 180.0, 247.5, 348.7, 348.8, 359.9, -45.0, 405.0]
                .into_iter()
                .map(compass)
                .collect();
        assert_eq!(named, ["N", "N", "NNE", "E", "S", "WSW", "NNW", "N", "N", "NW", "NE"]);
    }

    #[test]
    fn a_downlink_is_measured_against_the_median_of_the_span() {
        let mut db = vec![-100.0f32; 1_000];
        for v in &mut db[898..=902] {
            *v = -70.0;
        }
        db[10] = -20.0;
        let span = Span { db: &db, center_hz: 145.8e6, rate_hz: 1e6 };
        assert_eq!(level_above_floor(&span, 146.2e6, 2_500.0), Some(30.0));
        assert_eq!(level_above_floor(&span, 145.31e6, 500.0), Some(80.0));
        assert_eq!(level_above_floor(&span, 145.5e6, 2_500.0), Some(0.0));
        assert_eq!(level_above_floor(&span, 146.4e6, 2_500.0), None, "above the span");
        assert_eq!(level_above_floor(&span, 145.2e6, 2_500.0), None, "below the span");
        let empty = Span { db: &[], center_hz: 145.8e6, rate_hz: 1e6 };
        assert_eq!(level_above_floor(&empty, 145.8e6, 2_500.0), None);
    }

    #[test]
    fn mer_is_read_off_a_dvbs2_channel_readings() {
        let said = |pairs: &[(&str, &str)]| -> Vec<(String, String)> {
            pairs.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect()
        };
        let locked = said(&[("carrier", "QPSK 3/4"), ("MER", "9.1 dB"), ("offset", "+12 kHz")]);
        assert_eq!(mer_of(&locked), Some(9.1));
        assert_eq!(mer_of(&said(&[("carrier", "QPSK 3/4")])), None);
        assert_eq!(mer_of(&said(&[("MER", "garbled")])), None);
    }

    #[test]
    fn an_uplink_is_sent_off_frequency_so_it_lands_on_the_one_listed() {
        let at = iss().epoch_s;
        let here = dublin();
        let sat = iss();
        let (closing, opening) = (0..86_400)
            .step_by(10)
            .filter_map(|s| sat.look(here, at + s))
            .filter(|l| l.el_deg > 10.0)
            .fold((None, None), |(c, o): (Option<orbit::Look>, Option<orbit::Look>), l| {
                match l.range_rate_kms < -4.0 {
                    true => (c.or(Some(l)), o),
                    false if l.range_rate_kms > 4.0 => (c, o.or(Some(l))),
                    false => (c, o),
                }
            });
        let (closing, opening) = (closing.expect("an approach"), opening.expect("a retreat"));
        let u = 145.99e6;
        for l in [closing, opening] {
            let sent = send_on(u, &l);
            assert!((l.doppler_hz(sent) - u).abs() < 1e-3, "{sent} lands {}", l.doppler_hz(sent));
        }
        assert!(send_on(u, &closing) < u, "an approaching satellite hears it higher");
        assert!(send_on(u, &opening) > u, "a receding one hears it lower");
    }

    #[test]
    fn the_pass_shown_is_the_one_in_progress_or_else_the_next() {
        let sat = iss();
        let here = dublin();
        let from = sat.epoch_s;
        let passes = sat.passes(here, from, 86_400, 0.0);
        assert_eq!(passes.len(), 5, "passes over Dublin in the first day of these elements");
        let (second, third) = (passes[1], passes[2]);
        let same = |p: Option<orbit::Pass>, of: orbit::Pass| {
            p.is_some_and(|p| {
                [(p.rise_s, of.rise_s), (p.peak_s, of.peak_s), (p.set_s, of.set_s)]
                    .iter()
                    .all(|(a, b)| (a - b).abs() <= 1)
            })
        };
        let within = "within the one second the search refines to";
        assert!(same(current_or_next(&sat, here, second.peak_s), second), "at its peak, {within}");
        assert!(same(current_or_next(&sat, here, second.rise_s - 60), second), "before it");
        assert!(same(current_or_next(&sat, here, second.set_s + 1), third), "after it");
        let mut st = Pointing::new(sat.norad, &datasets::tle::AMATEUR);
        assert!(same(pass_of(&mut st, &sat, here, second.peak_s), second));
        assert!(
            same(pass_of(&mut st, &sat, here, second.set_s + 1), third),
            "a pass that has set is searched again rather than held"
        );
    }
}
