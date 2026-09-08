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
    /// The mode as SatNOGS spells it, for the name the strip shows.
    pub mode: String,
    /// What that mode is, which is what picks the chain.
    pub kind: datasets::satnogs::Mode,
    /// Symbols a second where it is digital, which for LoRa is the
    /// bandwidth the chirp is spread over.
    pub baud: Option<f64>,
    /// What the transmitter is: `Mode V/U FM voice`.
    pub what: String,
}

/// One of a satellite's transmitters as everything downstream wants it, or
/// `None` where it has no downlink to listen to.
fn downlink(norad: u64, sat: &str, t: &datasets::satnogs::Transmitter) -> Option<Downlink> {
    Some(Downlink {
        norad,
        sat: sat.to_string(),
        hz: t.downlink_hz? as f64,
        mode: t.mode.clone(),
        kind: t.kind,
        baud: t.baud,
        what: t.description.clone(),
    })
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
            if let Some(oldest) = s
                .sats()
                .iter()
                .map(|x| x.age_days(now))
                .fold(None, |m: Option<f64>, a| Some(m.map_or(a, |m| m.max(a))))
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
                    // What this satellite transmits on, and which of them
                    // the operator picked. A choice made once stands until
                    // it is changed: the ISS has forty-one live transmitters
                    // and a card that reset to the lowest frequency every
                    // frame would be unusable.
                    let live_tx: Vec<datasets::satnogs::Transmitter> =
                        tx.as_ref().map(|t| t.live(u.norad).cloned().collect()).unwrap_or_default();
                    let chosen = self.st.downlink.get(&u.norad).cloned();
                    let down = tx
                        .as_ref()
                        .and_then(|t| {
                            chosen
                                .as_deref()
                                .and_then(|uuid| t.by_uuid(u.norad, uuid))
                                .or_else(|| t.best(u.norad))
                        })
                        .cloned();
                    // The sky plot is only drawn for the picked card: it is
                    // the shape of one pass, and thirty of them stacked is a
                    // page of circles nobody reads.
                    let arc = picked
                        .then(|| {
                            sky.as_ref()
                                .and_then(|s| s.get(u.norad))
                                .map(|s| s.arc(station, u.pass.rise_s, u.pass.set_s, ARC_POINTS))
                        })
                        .flatten()
                        .unwrap_or_default();
                    let tracking = self.st.tracking.map(|t| t.norad) == Some(u.norad);
                    let out = pass_card(
                        ui,
                        u,
                        live.as_ref(),
                        down.as_ref(),
                        &live_tx,
                        &arc,
                        now,
                        picked,
                        tracking,
                    );
                    if out.response.clicked() {
                        selected = (!picked).then_some(u.norad);
                    }
                    // Listening is asked for on the row of the transmitter
                    // it is about, so the satellite it belongs to is picked
                    // in the same press: the operator pointed at a
                    // downlink, not at a satellite.
                    match out.inner.listen {
                        Some(Listen::Stop) => acts.push(Action::Untrack),
                        Some(Listen::Start(uuid)) => {
                            let link = tx
                                .as_ref()
                                .and_then(|t| t.by_uuid(u.norad, &uuid))
                                .and_then(|d| downlink(u.norad, &u.name, d));
                            if let Some(link) = link {
                                self.st.downlink.insert(u.norad, uuid);
                                acts.push(Action::Track(link));
                            }
                        }
                        None => {}
                    }
                    // Picking another transmitter while one is being
                    // followed moves the channel to it rather than waiting
                    // to be asked twice: the operator asked to listen to
                    // this satellite, and has now said on what.
                    if let Some(uuid) = out.inner.pick {
                        let moved = tx
                            .as_ref()
                            .and_then(|t| t.by_uuid(u.norad, &uuid))
                            .and_then(|d| downlink(u.norad, &u.name, d));
                        self.st.downlink.insert(u.norad, uuid);
                        if let (true, Some(m)) = (tracking, moved) {
                            acts.push(Action::Track(m));
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
    /// The listen icon pressed on one of the transmitter rows.
    listen: Option<Listen>,
    /// The transmitter picked this frame, by its SatNOGS identifier.
    pick: Option<String>,
    /// Where the transmitter table ended up. The card is clickable as a
    /// whole and that interaction is added after its contents, so egui's hit
    /// test hands it every press including the ones aimed at a row. That is
    /// why no downlink in the table could be clicked: the card claimed them
    /// all and merely selected itself. The card's own click is skipped where
    /// the pointer is over the table.
    table: Rect,
}

/// A press on a row's listen icon.
enum Listen {
    /// Follow this transmitter, by its SatNOGS identifier.
    Start(String),
    /// Stop following the one that is being followed.
    Stop,
}

#[allow(clippy::too_many_arguments)]
fn pass_card(
    ui: &mut egui::Ui,
    u: &crate::sats::Upcoming,
    live: Option<&orbit::Look>,
    down: Option<&datasets::satnogs::Transmitter>,
    live_tx: &[datasets::satnogs::Transmitter],
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
    let mut pressed = Pressed { listen: None, pick: None, table: Rect::NOTHING };
    // Which row is the one being listened to: the followed downlink is
    // always the chosen one, since picking another while tracking moves the
    // channel to it.
    let tracked = tracking.then(|| down.map(|d| d.uuid.as_str())).flatten();
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
                let when = match live.is_some() {
                    true => format!("up now, sets {}", crate::sats::in_when(u.pass.set_s - now)),
                    false => crate::sats::in_when(u.pass.rise_s - now),
                };
                theme::Line::new().legend(&when).show(ui);
                // The card says it is being listened to, since the control
                // that says so is now down in the table and the table is
                // only drawn for the picked card.
                if tracking {
                    theme::Line::new().legend("listening").tint(theme::READOUT).show(ui);
                }
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
                .value(format!("{}m {:02}s", u.pass.duration_s() / 60, u.pass.duration_s() % 60))
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
                // What to transmit on to be heard through it, where it is
                // not a beacon. Shown even though this receiver may not be
                // able to key it: knowing a repeater's input is half of
                // knowing what the downlink is carrying.
                if let Some(up) = d.uplink_label() {
                    theme::Line::new().legend("uplink").value(up).size(12.0).show(ui);
                }
            }
            // What the satellite is for, before any list of what it is on:
            // an operator deciding whether to wait for a pass wants "VHF
            // voice, SSTV, APRS", not forty rows of free text.
            let chans = datasets::satnogs::channels(live_tx.iter());
            if chans.len() > 1 {
                let names: Vec<String> = chans
                    .iter()
                    .map(|(n, c)| match c {
                        1 => n.clone(),
                        _ => format!("{n} \u{d7}{c}"),
                    })
                    .collect();
                theme::Line::new()
                    .legend("channels")
                    .value(names.join("  \u{b7}  "))
                    .size(12.0)
                    .show(ui);
            }
            // The table and the plot are the two halves of the same
            // question, which is what to tune and where to point, so they
            // sit beside each other and take half the card each rather than
            // pushing one another off the screen.
            let table = picked && !live_tx.is_empty();
            if table || !arc.is_empty() {
                ui.columns(2, |col| {
                    if table {
                        let out = transmitter_table(&mut col[0], live_tx, down, tracked, live);
                        pressed.pick = out.pick;
                        pressed.listen = out.listen;
                        pressed.table = out.rect;
                    }
                    if !arc.is_empty() {
                        sky_plot(&mut col[1], arc, live);
                    }
                });
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
        },
    );
    // The card is clickable underneath, but not where a button is: an
    // interaction added over the icons takes every press, so the card claims
    // the pointer only where no icon is under it.
    let id = ui.id().with(u.norad).with(u.pass.rise_s);
    let over_button =
        ui.ctx().pointer_interact_pos().is_some_and(|p| pressed.table.expand(2.0).contains(p));
    let hit = match over_button {
        true => ui.interact(Rect::NOTHING, id, Sense::hover()),
        false => ui.interact(inner.response.rect, id, Sense::click()),
    };
    egui::InnerResponse::new(pressed, hit)
}

/// What the transmitter table was asked to do this frame, with its rect so
/// the card underneath can leave those presses alone.
struct Table {
    pick: Option<String>,
    listen: Option<Listen>,
    rect: Rect,
}

/// Every live transmitter of the selected satellite as a table, each row
/// carrying the control that listens to it.
///
/// Drawn only for the picked card, because a satellite can have dozens: the
/// ISS has forty-one live transmitters, from suit radios at 121 MHz to the
/// CCSDS downlink at 2.2 GHz, and the pane used to quote one of them chosen
/// by a rule that meant nothing to an operator. Columns rather than one
/// sentence a row, because the question is nearly always a comparison
/// between rows: which of these is on two metres, which is FM, which has an
/// uplink.
///
/// The listen icon is on the row rather than on the card because the card
/// has no one downlink to speak for: the ISS has forty-one live
/// transmitters, and a single button on the header listened to whichever of
/// them a rule had chosen. There is only one useful thing to do with a
/// downlink, which is to listen to it while something keeps it tuned, so
/// one control a row and no separate follow: a listen that did not correct
/// stops working while you watch it, a low pass drifting out of a narrow
/// channel in under a minute.
fn transmitter_table(
    ui: &mut egui::Ui,
    live_tx: &[datasets::satnogs::Transmitter],
    chosen: Option<&datasets::satnogs::Transmitter>,
    tracked: Option<&str>,
    // Where the satellite is now, if it is up: the shift is a property of
    // the transmitter and the geometry together, so it belongs on the row
    // rather than on one line under the card quoting whichever transmitter
    // was chosen. Listening to a satellite that is not up would tune the
    // receiver away from the band for a pass that is hours off.
    live: Option<&orbit::Look>,
) -> Table {
    if live_tx.is_empty() {
        return Table { pick: None, listen: None, rect: Rect::NOTHING };
    }
    let mut picked = None;
    let mut listen = None;
    ui.add_space(2.0);
    theme::Line::new()
        .legend("transmitters")
        .value(format!("{} live", live_tx.len()))
        .size(11.0)
        .show(ui);
    let w = ui.available_width();
    // The table has half a card, not all of it, so the columns compress to
    // what is there rather than running off the edge and being clipped.
    let full: f32 = TX_COLS.iter().map(|(_, c)| c).sum::<f32>() + DESC_W;
    let scale = ((w - LISTEN_W) / full).min(1.0);
    // The heading sits outside the scroll so it cannot scroll away from what
    // it labels.
    let (head, _) = ui.allocate_exact_size(Vec2::new(w, widgets::ROW_H), Sense::hover());
    let p = ui.painter_at(head);
    let mut x = head.left() + LISTEN_W;
    for (name, cw) in TX_COLS {
        widgets::cell(&p, head, x, cw * scale, name, theme::LEGEND);
        x += cw * scale;
    }
    widgets::cell(&p, head, x, (head.right() - x).max(0.0), "description", theme::LEGEND);
    p.line_segment(
        [Pos2::new(head.left(), head.bottom()), Pos2::new(head.right(), head.bottom())],
        Stroke::new(1.0, theme::ETCH),
    );
    // Tall tables scroll rather than pushing the next pass off the screen.
    let out = egui::ScrollArea::vertical()
        .id_salt(("sat-tx", live_tx.first().map(|t| t.norad)))
        .max_height(TX_LIST_H)
        .auto_shrink([false, true])
        .show(ui, |ui| {
            for (n, t) in live_tx.iter().enumerate() {
                let is = chosen.is_some_and(|c| c.uuid == t.uuid);
                // A row with nothing coming down cannot be listened to, so
                // it is shown and not offered: an operator asking what a
                // satellite uses still wants to see it.
                let can = t.downlink_hz.is_some();
                let sense = match can {
                    true => Sense::click(),
                    false => Sense::hover(),
                };
                let (rect, resp) = ui.allocate_exact_size(Vec2::new(w, widgets::ROW_H), sense);
                if !ui.is_rect_visible(rect) {
                    continue;
                }
                let on = tracked == Some(t.uuid.as_str());
                let can_listen = can && (live.is_some() || on);
                let icon = Rect::from_center_size(
                    Pos2::new(rect.left() + LISTEN_W / 2.0, rect.center().y),
                    Vec2::splat(widgets::ROW_H - 2.0),
                );
                let over_icon = can_listen && resp.hover_pos().is_some_and(|p| icon.contains(p));
                let p = ui.painter_at(rect);
                if is {
                    p.rect_filled(rect, 0.0, theme::ETCH);
                } else if resp.hovered() {
                    p.rect_filled(rect, 0.0, Color32::from_rgb(0x2A, 0x2E, 0x35));
                } else if n % 2 == 1 {
                    p.rect_filled(rect, 0.0, Color32::from_rgb(0x24, 0x27, 0x2D));
                }
                if resp.hovered() {
                    ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
                }
                if resp.clicked() {
                    match (over_icon, on) {
                        (true, true) => listen = Some(Listen::Stop),
                        (true, false) => listen = Some(Listen::Start(t.uuid.clone())),
                        (false, _) if !is => picked = Some(t.uuid.clone()),
                        (false, _) => {}
                    }
                }
                if can {
                    if on || over_icon {
                        p.rect_filled(icon, 3.0, if on { theme::WELL } else { theme::ETCH });
                    }
                    crate::icons::Icon::Sound.paint(
                        &p,
                        icon,
                        crate::icons::tint(can_listen, on, over_icon),
                    );
                }
                let dim = match (is, can) {
                    (true, _) => theme::READOUT,
                    (false, true) => theme::VALUE,
                    (false, false) => theme::LEGEND,
                };
                let quiet = match is {
                    true => theme::READOUT,
                    false => theme::LEGEND,
                };
                // The uplink is a column rather than a tooltip: two of the
                // ISS's rows are the same 145.800 downlink and differ only
                // in which crew uplink they are, so a table without it has
                // duplicate rows.
                let shifted =
                    live.zip(t.downlink_hz).map(|(l, hz)| (l.doppler_hz(hz as f64), hz as f64));
                let cells = [
                    (
                        match t.downlink_hz {
                            Some(hz) => format!("{:.4}", hz as f64 / 1e6),
                            None => "\u{2014}".into(),
                        },
                        dim,
                    ),
                    (
                        match shifted {
                            Some((s, _)) => format!("{:.4}", s / 1e6),
                            None => String::new(),
                        },
                        match is {
                            true => theme::READOUT,
                            false => theme::TRACE,
                        },
                    ),
                    (
                        match shifted {
                            Some((s, hz)) => format!("{:+.0}", s - hz),
                            None => String::new(),
                        },
                        quiet,
                    ),
                    (t.mode.clone(), quiet),
                    (
                        match t.baud {
                            Some(b) if b > 0.0 => format!("{b:.0}"),
                            _ => String::new(),
                        },
                        quiet,
                    ),
                    (
                        match t.uplink_hz {
                            Some(hz) => format!("{:.4}", hz as f64 / 1e6),
                            None => String::new(),
                        },
                        quiet,
                    ),
                    (t.channel(), quiet),
                ];
                let mut x = rect.left() + LISTEN_W;
                for ((text, col), (_, cw)) in cells.iter().zip(TX_COLS) {
                    widgets::cell(&p, rect, x, cw * scale, text, *col);
                    x += cw * scale;
                }
                let what = match (t.description.is_empty(), t.invert) {
                    (true, _) => String::new(),
                    (false, true) => format!("{} (inverting)", t.description),
                    (false, false) => t.description.clone(),
                };
                widgets::cell(&p, rect, x, (rect.right() - x).max(0.0), &what, quiet);
                match (can, over_icon, on) {
                    (false, _, _) => {
                        resp.on_hover_text("transmit only, nothing to listen to");
                    }
                    (true, _, _) if !can_listen => {}
                    (true, true, true) => {
                        resp.on_hover_text("stop listening");
                    }
                    (true, true, false) => {
                        resp.on_hover_text("listen, following the Doppler down");
                    }
                    (true, false, _) => {}
                }
            }
        });
    Table { pick: picked, listen, rect: head.union(out.inner_rect) }
}

/// Width of the listen column, left of the frequencies so the icons line up
/// down the edge of the table.
const LISTEN_W: f32 = 20.0;

/// What the description column is worth when the table has room for it. It
/// is the fill column, so this only decides how much the rest give up.
const DESC_W: f32 = 120.0;

/// Columns of the transmitter table and their widths. Fixed rather than
/// sized to the content, so the frequencies line up down the column and
/// nothing moves under the pointer as a satellite is picked.
const TX_COLS: [(&str, f32); 7] = [
    ("downlink", 74.0),
    ("arrives", 74.0),
    ("doppler", 60.0),
    ("mode", 56.0),
    ("baud", 46.0),
    ("uplink", 74.0),
    ("channel", 96.0),
];

/// How much of the transmitter table is shown before it scrolls. Six rows,
/// which fits every satellite that has a handful and keeps the ISS from
/// filling the pane with one card.
const TX_LIST_H: f32 = 96.0;

/// How finely a pass is sampled for the sky plot. Fifty points across a pass
/// of a few minutes is a curve rather than a polygon at any size this is
/// drawn at.
const ARC_POINTS: usize = 50;

/// Size of the sky plot, in points: the least it is drawn at, and the most.
const SKY_D: f32 = 150.0;
const SKY_MAX_D: f32 = 240.0;

/// The pass drawn as a track across the sky.
///
/// The view an operator actually points an antenna by: north at the top,
/// east to the right, the horizon at the rim and the zenith at the centre,
/// which is the convention every satellite program uses and the one a
/// rotator's own display shows. A row of azimuths cannot say whether a pass
/// goes behind the house; this can.
fn sky_plot(ui: &mut egui::Ui, arc: &[orbit::Look], live: Option<&orbit::Look>) {
    // Square, and as big as its half of the card allows up to a size where
    // more pixels say nothing more about a pass, sitting in the middle of
    // the half rather than against the table.
    let w = ui.available_width();
    let d = w.clamp(SKY_D, SKY_MAX_D);
    let (band, _) = ui.allocate_exact_size(Vec2::new(w.max(d), d), Sense::hover());
    let rect = Rect::from_center_size(band.center(), Vec2::splat(d));
    let p = ui.painter_at(rect);
    let mid = rect.center();
    let r = d / 2.0 - 10.0;
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
