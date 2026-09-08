//! Passes: what is overhead now, and what is coming.
//!
//! A prediction rather than a reception, which is the thing to keep in mind
//! reading it. Nothing here has been heard: it is arithmetic on elements
//! somebody else fitted, and how old those are decides how much of it to
//! believe, so the age is on the screen rather than buried in the dataset
//! pane.
//!
//! The row an operator wants is not "where is it" but "when, how high, and
//! which way do I point". So the table is ordered by rise time, a satellite
//! already up is at the top with its live look angles, and the doppler for
//! the group's usual downlink is beside it, because that is the number that
//! decides where to tune.

use super::state::SatsState;
use super::*;

pub(super) struct Sats<'a> {
    pub st: &'a mut SatsState,
    /// Where the receiver is. Without it there are no passes: a pass is over
    /// somewhere.
    pub home: Option<(f64, f64)>,
}

/// One satellite's downlink, as everything downstream needs it: which
/// satellite, what it transmits on, and what it is. Carried whole rather
/// than as a frequency, because a channel wants to be named after the thing
/// it is listening to and a chain wants to be built for the mode it is in.
#[derive(Clone, Debug)]
pub(super) struct Downlink {
    pub norad: u64,
    /// The satellite, as CelesTrak names it.
    pub sat: String,
    /// Hertz as transmitted, before any shift.
    pub hz: f64,
    /// The mode, as SatNOGS names it: `FM`, `USB`, `BPSK`, `AFSK`.
    pub mode: String,
    /// What the transmitter is: `Mode V/U FM voice`.
    pub what: String,
}

/// What the pane asks the application to do.
pub(super) enum Action {
    /// Look at the selected satellite on the map.
    ShowOnMap,
    /// Listen on this downlink, following it down as the pass moves. There
    /// is no listen-without-following: a downlink tuned once is off the
    /// transmission within the minute.
    Track(Downlink),
    /// Stop listening.
    Untrack,
}

impl Sats<'_> {
    pub(super) fn show(self, ui: &mut egui::Ui) -> Vec<Action> {
        let mut acts = Vec::new();
        let now = crate::sats::now_s();
        ui.add_space(8.0);

        let Some((lat, lon)) = self.home else {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| {
                hint(
                    ui,
                    "No station position. A pass is over somewhere: set one in Setup, or \
                     right-click the map, and this fills in.",
                );
            });
            return acts;
        };
        let station = orbit::Station::new(lat, lon);

        ui.horizontal(|ui| {
            ui.add_space(12.0);
            theme::Line::new().legend("group").show(ui);
            egui::ComboBox::from_id_salt("sat-group")
                .selected_text(self.st.group.name)
                .width(140.0)
                .show_ui(ui, |ui| {
                    for g in datasets::tle::GROUPS.iter().copied() {
                        ui.selectable_value(&mut self.st.group, g, g.name);
                    }
                });
            ui.add_space(12.0);
            theme::Line::new().legend("above").show(ui);
            // The threshold is what makes the table readable: everything in
            // the group grazes the horizon at some point in a day, and those
            // passes are not workable.
            egui::ComboBox::from_id_salt("sat-min-el")
                .selected_text(format!("{:.0}\u{b0}", self.st.min_el_deg))
                .width(70.0)
                .show_ui(ui, |ui| {
                    for e in [0.0, 5.0, 10.0, 20.0, 30.0, 45.0] {
                        ui.selectable_value(&mut self.st.min_el_deg, e, format!("{e:.0}\u{b0}"));
                    }
                });
            ui.add_space(12.0);
            if ui.button(legend("SHOW ON MAP")).clicked() {
                acts.push(Action::ShowOnMap);
            }
        });
        ui.add_space(6.0);

        let sky = crate::sats::sky(self.st.group);
        // Asking is what downloads it. A pass list without frequencies is
        // still a pass list, so this being absent changes a card rather than
        // emptying the pane.
        let tx = crate::data::transmitters();
        let passes = crate::sats::passes(self.st.group, station, now, self.st.min_el_deg);
        // A search takes a moment and a table that is empty while it runs
        // reads as "nothing is coming", which is the opposite of the truth.
        let heading = match (&passes, sky.as_ref()) {
            (_, None) => "downloading elements".to_string(),
            (None, Some(s)) => format!("searching {} objects", s.sats().len()),
            (Some(p), Some(s)) => format!(
                "{} passes over the next day, from {} objects{}",
                p.len(),
                s.sats().len(),
                if crate::sats::computing() { ", refreshing" } else { "" }
            ),
        };
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            theme::Line::new().legend("passes").value(heading).size(11.0).show(ui);
        });
        ui.add_space(6.0);

        // Elements go stale in days. Say how stale the worst of them is,
        // because a table drawn from a fortnight-old set is fiction and
        // nothing else on this screen would show it.
        if let Some(s) = sky.as_ref() {
            if let Some(oldest) =
                s.sats().iter().map(|x| x.age_days(now)).fold(None, |m: Option<f64>, a| {
                    Some(m.map_or(a, |m| m.max(a)))
                })
            {
                let stale = oldest > 7.0;
                ui.horizontal(|ui| {
                    ui.add_space(12.0);
                    let mut line = theme::Line::new()
                        .legend("oldest elements")
                        .value(format!("{oldest:.1} days past epoch"))
                        .size(12.0);
                    if stale {
                        line = line
                            .tint(theme::FAULT)
                            .note("refresh them, a set this old is a guess")
                            .tint(theme::FAULT);
                    }
                    line.show(ui);
                });
            }
        }
        ui.add_space(4.0);

        let Some(passes) = passes else {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(500));
            return acts;
        };
        let mut selected = self.st.selected;
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
                for u in passes.iter().take(200) {
                    let up = u.pass.rise_s <= now && now <= u.pass.set_s;
                    let picked = selected == Some(u.norad);
                    // Live angles for what is up, the plan for what is not:
                    // an operator turning an antenna now wants the first,
                    // one setting an alarm wants the second.
                    let live = up
                        .then(|| {
                            sky.as_ref()
                                .and_then(|s| s.get(u.norad))
                                .and_then(|s| s.look(station, now))
                        })
                        .flatten();
                    let down = tx.as_ref().and_then(|t| t.best(u.norad)).cloned();
                    // The sky plot is only drawn for the picked card: it is
                    // the shape of one pass, and thirty of them stacked is a
                    // page of circles nobody reads.
                    let arc = picked
                        .then(|| {
                            sky.as_ref().and_then(|s| s.get(u.norad)).map(|s| {
                                s.arc(station, u.pass.rise_s, u.pass.set_s, ARC_POINTS)
                            })
                        })
                        .flatten()
                        .unwrap_or_default();
                    let tracking = self.st.tracking.map(|t| t.norad) == Some(u.norad);
                    let out = pass_card(
                        ui,
                        u,
                        live.as_ref(),
                        down.as_ref(),
                        &arc,
                        now,
                        picked,
                        tracking,
                    );
                    if out.response.clicked() {
                        selected = (!picked).then_some(u.norad);
                    }
                    let link = down.as_ref().and_then(|d| {
                        Some(Downlink {
                            norad: u.norad,
                            sat: u.name.clone(),
                            hz: d.downlink_hz? as f64,
                            mode: d.mode.clone(),
                            what: d.description.clone(),
                        })
                    });
                    if out.inner.track {
                        match (tracking, &link) {
                            (true, _) => acts.push(Action::Untrack),
                            (false, Some(link)) => acts.push(Action::Track(link.clone())),
                            (false, None) => {}
                        }
                    }
                    ui.add_space(4.0);
                }
                if passes.is_empty() {
                    ui.add_space(24.0);
                    ui.vertical_centered(|ui| {
                        hint(
                            ui,
                            "Nothing in this group rises above the threshold in the next day. \
                             Lower it, or pick another group.",
                        );
                    });
                }
            });
        });
        self.st.selected = selected;
        // A countdown that does not count down is a clock nobody trusts.
        ui.ctx().request_repaint_after(std::time::Duration::from_millis(500));
        acts
    }
}

/// One pass, as a card.
///
/// The rail is where the card says what it is: amber for the one the
/// operator picked, cyan for a pass that is happening now, nothing for one
/// that is merely coming. Everything on it is predicted rather than heard,
/// which is why a pass in progress gets the cyan rather than every row
/// getting it.
/// What the buttons on a card were asked to do this frame.
struct Pressed {
    track: bool,
    /// Where the buttons ended up. The card is clickable as a whole, and
    /// that interaction is added after them, so egui's hit test would hand
    /// it every press including the ones aimed at an icon: the row selected
    /// and nothing else happened. The card's own click is skipped where the
    /// pointer is over a button instead.
    buttons: Rect,
}

#[allow(clippy::too_many_arguments)]
fn pass_card(
    ui: &mut egui::Ui,
    u: &crate::sats::Upcoming,
    live: Option<&orbit::Look>,
    down: Option<&datasets::satnogs::Transmitter>,
    arc: &[orbit::Look],
    now: i64,
    picked: bool,
    tracking: bool,
) -> egui::InnerResponse<Pressed> {
    let rail = match (picked, live.is_some()) {
        (true, _) => Some(theme::READOUT),
        (false, true) => Some(theme::TRACE),
        _ => None,
    };
    let mut pressed = Pressed { track: false, buttons: Rect::NOTHING };
    let can_tune = live.is_some() && down.and_then(|d| d.downlink_hz).is_some();
    let inner = widgets::card(
        ui,
        rail,
        |ui| {
            theme::Line::new()
                .legend(&format!("#{}", u.norad))
                .value(u.name.clone())
                .tint(theme::VALUE)
                .size(12.0)
                .show(ui);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // Two affordances rather than a sentence telling somebody to
                // right-click: listen once where it is arriving now, and
                // follow it down, which is the one that matters because a
                // low pass drifts out of a narrow channel in under a minute.
                // One control, because there is only one useful thing to do
                // with a downlink: listen to it while something keeps it
                // tuned. A listen that did not correct would be a button
                // whose result stops working while you watch it.
                use crate::icons::{icon_button_sized, Icon};
                let listen = icon_button_sized(
                    ui,
                    Icon::Sound,
                    match tracking {
                        true => "stop listening",
                        false => "listen, following the Doppler down",
                    },
                    can_tune,
                    tracking,
                    20.0,
                );
                pressed.track = listen.clicked();
                pressed.buttons = listen.rect;
                let when = match live.is_some() {
                    true => format!("up now, sets {}", crate::sats::in_when(u.pass.set_s - now)),
                    false => crate::sats::in_when(u.pass.rise_s - now),
                };
                theme::Line::new().legend(&when).show(ui);
            });
        },
        |ui| {
            theme::Line::new()
                .legend("rises")
                .value(crate::sats::utc_hms(u.pass.rise_s))
                .size(12.0)
                .gap(14.0)
                .legend("peak")
                .value(format!("{:.0}\u{b0}", u.pass.max_el_deg))
                .size(12.0)
                .gap(14.0)
                .legend("for")
                .value(format!(
                    "{}m {:02}s",
                    u.pass.duration_s() / 60,
                    u.pass.duration_s() % 60
                ))
                .size(12.0)
                .gap(14.0)
                .legend("az")
                .value(format!("{:.0}\u{b0} to {:.0}\u{b0}", u.pass.rise_az_deg, u.pass.set_az_deg))
                .size(12.0)
                .show(ui);
            // What it transmits on, whether or not it is up: a card for a
            // pass in two hours is worth reading for the frequency.
            if let Some(d) = down {
                theme::Line::new()
                    .legend("downlink")
                    .value(d.label())
                    .size(12.0)
                    .tint(if d.alive { theme::VALUE } else { theme::LEGEND })
                    .show(ui);
            }
            if !arc.is_empty() {
                sky_plot(ui, arc, live);
            }
            let Some(l) = live else { return };
            theme::Line::new()
                .legend("now")
                .value(format!("az {:.0}\u{b0} el {:.0}\u{b0}", l.az_deg, l.el_deg))
                .tint(theme::TRACE)
                .size(12.0)
                .gap(14.0)
                .legend("range")
                .value(format!("{:.0} km", l.range_km))
                .size(12.0)
                .gap(14.0)
                .legend("closing")
                .value(format!("{:.2} km/s", -l.range_rate_kms))
                .size(12.0)
                .show(ui);
            // The number that decides where to tune, said as a frequency
            // rather than left as a bare offset. Off the satellite's own
            // downlink where SatNOGS has one, because a shift quoted on a
            // frequency it does not use is a number that cannot be used.
            // What the geometry does to the link. Spreading only, which is
            // why it is called free space and not "the loss": no
            // atmosphere, no antennas, no polarisation. It is worth showing
            // because it changes by ten dB across a pass, which is what
            // says a fade was the pass and not the receiver.
            let hz = down.and_then(|d| d.downlink_hz);
            let mut link = theme::Line::new()
                .legend("free space")
                .value(match hz {
                    Some(hz) => format!("{:.1} dB", l.path_loss_db(hz as f64)),
                    None => "\u{2014}".into(),
                })
                .size(12.0)
                .gap(14.0)
                .legend("delay")
                .value(format!("{:.1} ms", l.delay_ms()))
                .size(12.0)
                .gap(14.0)
                .legend("footprint");
            link = link.value(format!("{:.0} km", l.footprint_km())).size(12.0);
            link.show(ui);
            let Some(hz) = hz else { return };
            let shifted = l.doppler_hz(hz as f64);
            theme::Line::new()
                .legend("arrives at")
                .value(format!("{:.4} MHz", shifted / 1e6))
                .tint(theme::READOUT)
                .size(12.0)
                .gap(14.0)
                .legend("doppler")
                .value(format!("{:+.0} Hz", shifted - hz as f64))
                .size(12.0)
                .show(ui);
        },
    );
    // The card is clickable underneath, but not where a button is: an
    // interaction added over the icons takes every press, so the card claims
    // the pointer only where no icon is under it.
    let id = ui.id().with(u.norad).with(u.pass.rise_s);
    let over_button = ui
        .ctx()
        .pointer_interact_pos()
        .is_some_and(|p| pressed.buttons.expand(2.0).contains(p));
    let hit = match over_button {
        true => ui.interact(Rect::NOTHING, id, Sense::hover()),
        false => ui.interact(inner.response.rect, id, Sense::click()),
    };
    egui::InnerResponse::new(pressed, hit)
}



/// How finely a pass is sampled for the sky plot. Fifty points across a pass
/// of a few minutes is a curve rather than a polygon at any size this is
/// drawn at.
const ARC_POINTS: usize = 50;

/// Size of the sky plot, in points.
const SKY_D: f32 = 150.0;

/// The pass drawn as a track across the sky.
///
/// The view an operator actually points an antenna by: north at the top,
/// east to the right, the horizon at the rim and the zenith at the centre,
/// which is the convention every satellite program uses and the one a
/// rotator's own display shows. A row of azimuths cannot say whether a pass
/// goes behind the house; this can.
fn sky_plot(ui: &mut egui::Ui, arc: &[orbit::Look], live: Option<&orbit::Look>) {
    let (rect, _) = ui.allocate_exact_size(Vec2::splat(SKY_D), Sense::hover());
    let p = ui.painter_at(rect);
    let mid = rect.center();
    let r = SKY_D / 2.0 - 10.0;
    // Elevation is the radius, ninety degrees at the middle. Linear in
    // elevation rather than in zenith distance, which is what makes a low
    // pass look low.
    let at = |l: &orbit::Look| {
        let rho = r * (1.0 - (l.el_deg.max(0.0) / 90.0)) as f32;
        let az = (l.az_deg * std::f64::consts::PI / 180.0) as f32;
        Pos2::new(mid.x + rho * az.sin(), mid.y - rho * az.cos())
    };
    for (ring, el) in [(r, 0.0), (r * 2.0 / 3.0, 30.0), (r / 3.0, 60.0)] {
        let dim = if el == 0.0 { theme::ETCH } else { theme::ETCH.gamma_multiply(0.6) };
        p.circle_stroke(mid, ring, Stroke::new(1.0, dim));
    }
    let font = FontId::new(9.0, FontFamily::Name(theme::LEGEND_FONT.into()));
    for (name, dx, dy) in [("N", 0.0, -1.0), ("E", 1.0, 0.0), ("S", 0.0, 1.0), ("W", -1.0, 0.0)] {
        p.text(
            Pos2::new(mid.x + dx * (r + 6.0), mid.y + dy * (r + 6.0)),
            Align2::CENTER_CENTER,
            name,
            font.clone(),
            theme::LEGEND,
        );
    }
    for w in arc.windows(2) {
        p.line_segment([at(&w[0]), at(&w[1])], Stroke::new(1.5, theme::READOUT));
    }
    // Where it comes up and where it goes down, which is the pair of numbers
    // the plot exists to make obvious.
    if let (Some(first), Some(last)) = (arc.first(), arc.last()) {
        p.circle_filled(at(first), 2.5, theme::OK);
        p.circle_stroke(at(last), 3.0, Stroke::new(1.0, theme::LEGEND));
    }
    if let Some(l) = live.filter(|l| l.el_deg > 0.0) {
        let now = at(l);
        p.circle_stroke(now, 4.0, Stroke::new(1.5, theme::TRACE));
        p.circle_filled(now, 1.5, theme::TRACE);
    }
}
