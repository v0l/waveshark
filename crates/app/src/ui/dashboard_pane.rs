//! The dashboard: what the receiver can do, and what it is doing.
//!
//! Two halves, and the split is between two different questions. Quick start
//! answers "what is this for", which somebody asks once; the status cards
//! answer "what is it doing now", which somebody asks all day. The first is
//! always there because a receiver with nothing tuned has nothing else to
//! show, and the second appears only while a radio is running, since every
//! reading on it would otherwise be a zero pretending to be a measurement.
//!
//! It is a view over what the receiver already publishes and nothing else: no
//! reading here is taken anywhere but from `radio::Status` and the counts the
//! other views keep. Nothing on this pane may be the only place a number is
//! computed, or the dashboard becomes a second opinion about the receiver.

use super::widgets::{card, hint, speed_trace};
use super::*;
use std::sync::atomic::Ordering::Relaxed;

/// Which colour a card's rail carries, which is the theme's own rule: amber
/// for what the operator sets, cyan for what the radio heard.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Rail {
    Set,
    Heard,
}

impl Rail {
    fn colour(self) -> Color32 {
        match self {
            Rail::Set => theme::READOUT,
            Rail::Heard => theme::TRACE,
        }
    }
}

/// Where a quick start item takes you.
enum Goes {
    View(View),
    Settings(Settings),
}

/// One thing the receiver can do, and the one click that starts doing it.
struct Quick {
    /// What part of the receiver this is, in the words the interface uses for
    /// it elsewhere.
    place: &'static str,
    title: &'static str,
    note: &'static str,
    rail: Rail,
    goes: Goes,
}

/// The six worth a card. Everything else is a line in the list under them:
/// a quick start that lists everything is a manual, and nobody reads the
/// eleventh item on one.
const CARDS: [Quick; 6] = [
    Quick {
        place: "spectrum",
        title: "Turn the dial and look",
        note: "Drag the span, scroll to zoom, click a signal to put a channel on it.",
        rail: Rail::Set,
        goes: Goes::View(View::Spectrum),
    },
    Quick {
        place: "scanner table",
        title: "Let it watch the whole span",
        note: "Give a band to a decoder once and every burst in it is read while you do \
               something else.",
        rail: Rail::Set,
        goes: Goes::Settings(Settings::Scanners),
    },
    Quick {
        place: "map",
        title: "See who is out there",
        note: "Aircraft, vessels, APRS stations and mesh nodes, on tiles, with range rings from \
               your antenna.",
        rail: Rail::Heard,
        goes: Goes::View(View::Map),
    },
    Quick {
        place: "calls",
        title: "Hear a conversation",
        note: "DMR, TETRA and M17 voice as rows you can click to listen, with what was said \
               beside it.",
        rail: Rail::Heard,
        goes: Goes::View(View::Calls),
    },
    Quick {
        place: "messages",
        title: "Read text sent over the air",
        note: "Pagers, TETRA, APRS and mesh, in one column, repeats folded into one line.",
        rail: Rail::Heard,
        goes: Goes::View(View::Messages),
    },
    Quick {
        place: "devices",
        title: "Survey while you drive",
        note: "One row per transmitter rather than per burst, and a position fitted from the \
               levels it was heard at.",
        rail: Rail::Heard,
        goes: Goes::View(View::Devices),
    },
];

/// The rest, as one line each.
const MORE: [Quick; 5] = [
    Quick {
        place: "signal chain",
        title: "Build the receiver by hand",
        note: "Unlock the graph and wire a demodulator yourself.",
        rail: Rail::Set,
        goes: Goes::View(View::Chain),
    },
    Quick {
        place: "satellites",
        title: "Catch a pass",
        note: "When it rises, where to point, and the Doppler on the downlink.",
        rail: Rail::Heard,
        goes: Goes::View(View::Satellites),
    },
    Quick {
        place: "radio settings",
        title: "Record the band",
        note: "Raw IQ of the whole span, or just the bursts in it.",
        rail: Rail::Set,
        goes: Goes::Settings(Settings::Radio),
    },
    Quick {
        place: "packet log settings",
        title: "Bring in another receiver",
        note: "Beast or AVR over TCP, joining the same packet bus as the local front ends.",
        rail: Rail::Set,
        goes: Goes::Settings(Settings::PacketLog),
    },
    Quick {
        place: "memory",
        title: "Save a channel you will come back to",
        note: "Its mode, width and squelch, in a group.",
        rail: Rail::Set,
        goes: Goes::Settings(Settings::Memory),
    },
];

/// What the other views are holding, so this one can say how much without
/// keeping a second copy of any of it.
pub(super) struct Counts {
    pub tracks: usize,
    pub calls: usize,
    pub messages: usize,
    pub links: usize,
    pub fix: bool,
}

pub(super) struct Dashboard<'a> {
    pub radio: Option<&'a Radio>,
    pub device: Option<&'a str>,
    pub center: f64,
    pub rate: f64,
    pub zoom: usize,
    pub decode_on: bool,
    pub counts: Counts,
}

pub(super) enum Action {
    Open(View),
    Panel(Settings),
    /// Start the radio that is already chosen, which is the one thing a cold
    /// receiver needs and the reason this pane exists at all.
    Start,
    Hide,
}

impl Dashboard<'_> {
    pub(super) fn show(self, ui: &mut egui::Ui) -> Vec<Action> {
        let mut acts = Vec::new();
        let running = self.radio.is_some_and(|r| r.status.running.load(Relaxed));

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            let mut line = theme::Line::new().legend("dashboard").gap(10.0);
            line = match (running, self.device) {
                (true, Some(d)) => line.legend(d).tint(theme::OK),
                (true, None) => line.legend("running").tint(theme::OK),
                (false, _) => line.legend("radio stopped"),
            };
            line.size(11.0).show(ui);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(12.0);
                // Quiet, and at the far end: hiding the dashboard is done
                // once and never again, so it may not sit where the eye goes
                // for the readings.
                let hit = ui
                    .add(egui::Button::new(legend("hide dashboard")).frame(false))
                    .on_hover_text("Stop showing this view. Settings, App turns it back on.");
                if hit.clicked() {
                    acts.push(Action::Hide);
                }
            });
        });
        ui.add_space(6.0);

        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
                ui.spacing_mut().item_spacing.y = 8.0;
                if running {
                    self.status(ui);
                } else {
                    Self::cold(ui, self.device, &mut acts);
                }

                ui.add_space(10.0);
                Self::rule(ui, "quick start");
                ui.add_space(2.0);

                // Three across on a window that has the room for it, two when
                // it does not: a card whose note wraps to four lines is a card
                // nobody finishes reading.
                let cols = if ui.available_width() > 900.0 { 3 } else { 2 };
                for row in CARDS.chunks(cols) {
                    ui.columns(cols, |c| {
                        for (i, q) in row.iter().enumerate() {
                            if let Some(a) = Self::quick_card(&mut c[i], q) {
                                acts.push(a);
                            }
                        }
                    });
                }

                ui.add_space(6.0);
                for q in &MORE {
                    if let Some(a) = Self::quick_row(ui, q) {
                        acts.push(a);
                    }
                }
                ui.add_space(12.0);
            });
        });
        acts
    }

    /// What the receiver is doing, once there is a receiver doing it.
    fn status(&self, ui: &mut egui::Ui) {
        let Some(r) = self.radio else { return };
        let s = &r.status;
        let wide = ui.available_width() > 900.0;

        // Three across when there is room, otherwise stacked in one column,
        // which `columns(1)` gives without a second layout being written.
        let col = |k: usize| if wide { k } else { 0 };
        ui.columns(if wide { 3 } else { 1 }, |c| {
            card(
                &mut c[col(0)],
                Some(Rail::Set.colour()),
                |ui| {
                    ui.label(legend("tuned"));
                },
                |ui| {
                    theme::Line::new().set(fmt_hz(self.center)).size(20.0).show(ui);
                    let span = format!("{:.3} MS/s", self.rate * self.zoom.max(1) as f64 / 1e6);
                    theme::Line::new()
                        .legend("span")
                        .column(ui, 74.0)
                        .set(span)
                        .size(12.0)
                        .show(ui);
                    if self.zoom > 1 {
                        theme::Line::new()
                            .legend("zoom")
                            .column(ui, 74.0)
                            .set(format!("/{}", self.zoom))
                            .size(12.0)
                            .show(ui);
                    }
                    theme::Line::new()
                        .legend("decoding")
                        .column(ui, 74.0)
                        .set(if self.decode_on { "on" } else { "off" })
                        .size(12.0)
                        .show(ui);
                },
            );

            let hist = s.speed_history();
            let now = hist.last().copied().unwrap_or(0.0);
            card(
                &mut c[col(1)],
                None,
                |ui| {
                    ui.label(legend("real time"));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(value(format!("{now:.2}x")).size(12.0));
                    });
                },
                |ui| {
                    let w = ui.available_width();
                    speed_trace(ui, Vec2::new(w, 44.0), true, s.dropped.load(Relaxed), &hist);
                    // The number that says whether another channel will fit,
                    // and the delay a listener hears, are the two things this
                    // trace is read for.
                    hint(
                        ui,
                        &format!(
                            "{:.0} ms through the chain. Above the line is headroom.",
                            s.chain_latency()
                        ),
                    );
                },
            );

            let banks = s.scan_channels.load(Relaxed) + s.scan_channels_wide.load(Relaxed);
            card(
                &mut c[col(2)],
                Some(Rail::Heard.colour()),
                |ui| {
                    ui.label(legend("decoded"));
                },
                |ui| {
                    theme::Line::new()
                        .heard(burst::thousands(s.decoded.load(Relaxed)))
                        .size(20.0)
                        .gap(6.0)
                        .legend("packets")
                        .show(ui);
                    theme::Line::new()
                        .legend("channels")
                        .column(ui, 84.0)
                        .heard(burst::thousands(banks))
                        .size(12.0)
                        .show(ui);
                    theme::Line::new()
                        .legend("logged")
                        .column(ui, 84.0)
                        .heard(burst::thousands(s.logged.load(Relaxed)))
                        .size(12.0)
                        .show(ui);
                    // A column of zeros is not the answer to "why is nothing
                    // decoding", and this is the usual reason for it.
                    if !self.decode_on {
                        hint(ui, "Decoding is off, so the span is not being read.");
                    }
                },
            );
        });

        ui.columns(if wide { 3 } else { 1 }, |c| {
            let heard = [
                ("tracks", self.counts.tracks as u64),
                ("calls", self.counts.calls as u64),
                ("messages", self.counts.messages as u64),
                ("links", self.counts.links as u64),
                ("devices", s.survey_devices.load(Relaxed)),
            ];
            card(
                &mut c[0],
                Some(Rail::Heard.colour()),
                |ui| {
                    ui.label(legend("what has been heard"));
                },
                |ui| {
                    for (name, n) in heard {
                        theme::Line::new()
                            .legend(name)
                            .column(ui, 90.0)
                            .heard(burst::thousands(n))
                            .size(12.0)
                            .show(ui);
                    }
                },
            );

            // What is transmitting at this moment, which is the one reading
            // here that is gone a second later and so the one worth a list
            // rather than a count.
            let mut open = s.sources.lock().clone();
            open.sort_by(|a, b| b.source.snr_db.total_cmp(&a.source.snr_db));
            card(
                &mut c[col(1)],
                Some(Rail::Heard.colour()),
                |ui| {
                    ui.label(legend("on the air now"));
                },
                |ui| {
                    if open.is_empty() {
                        hint(ui, "Nothing is transmitting in the span.");
                    }
                    for src in open.iter().take(5) {
                        theme::Line::new()
                            .heard(fmt_hz(src.source.center_hz))
                            .size(12.0)
                            .column(ui, 96.0)
                            .legend(src.source.locked_to.unwrap_or("open"))
                            .column(ui, 176.0)
                            .heard(format!("{:.0} dB", src.source.snr_db))
                            .size(12.0)
                            .show(ui);
                    }
                    if open.len() > 5 {
                        hint(ui, &format!("and {} more", open.len() - 5));
                    }
                },
            );

            let dropped = s.dropped.load(Relaxed);
            let backlog = s.audio_backlog.load(Relaxed);
            card(
                &mut c[col(2)],
                None,
                |ui| {
                    ui.label(legend("health"));
                },
                |ui| {
                    Self::lamp_row(
                        ui,
                        dropped == 0,
                        &match dropped {
                            0 => "No samples dropped".to_string(),
                            n => format!("{} samples dropped", burst::thousands(n)),
                        },
                    );
                    Self::lamp_row(
                        ui,
                        backlog < 4,
                        &format!("Audio bus {backlog} transfers behind"),
                    );
                    let err = s.error.lock().clone();
                    match err {
                        Some(e) => Self::lamp_row(ui, false, &e),
                        None => Self::lamp_row(ui, true, "No fault reported"),
                    }
                    Self::lamp_row(
                        ui,
                        self.counts.fix,
                        match self.counts.fix {
                            true => "GPS fix, and the station is following it",
                            false => "No GPS fix. The station is where it was set",
                        },
                    );
                },
            );
        });
    }

    /// A lamp and what it is a lamp for. Green or amber rather than green or
    /// red: none of these is a fault that stops the receiver, and a red lamp
    /// for a missing GPS would say otherwise.
    fn lamp_row(ui: &mut egui::Ui, ok: bool, text: &str) {
        ui.horizontal(|ui| {
            let (rect, _) = ui.allocate_exact_size(Vec2::splat(9.0), Sense::hover());
            let col = if ok { theme::OK } else { theme::READOUT };
            ui.painter().circle_filled(rect.center(), 4.0, col);
            ui.label(egui::RichText::new(text).size(12.0).color(theme::VALUE));
        });
    }

    /// What the pane says with no radio running: the one thing to do next.
    fn cold(ui: &mut egui::Ui, device: Option<&str>, acts: &mut Vec<Action>) {
        card(
            ui,
            None,
            |ui| {
                ui.label(legend("nothing is being received"));
            },
            |ui| {
                ui.label(
                    egui::RichText::new("Start a radio and the readings appear here.").size(14.0),
                );
                hint(
                    ui,
                    "The receiver opens on 433.92 MHz with the scanner table already watching \
                     the span, so sensors, remotes and beacons reach the packet list without \
                     anything else being set.",
                );
                ui.add_space(4.0);
                ui.horizontal(|ui| match device {
                    Some(d) => {
                        if ui.button(value(format!("Start {d}")).size(13.0)).clicked() {
                            acts.push(Action::Start);
                        }
                    }
                    None => {
                        hint(
                            ui,
                            "No radio found. Plug one in, or add one on the network from \
                                  the device box in the bar above.",
                        );
                    }
                });
            },
        );
    }

    /// One quick start item as a card the whole of which is the target.
    fn quick_card(ui: &mut egui::Ui, q: &Quick) -> Option<Action> {
        let framed = card(
            ui,
            Some(q.rail.colour().gamma_multiply(0.55)),
            |ui| {
                ui.label(legend(q.place));
            },
            |ui| {
                ui.label(egui::RichText::new(q.title).size(14.0).color(theme::VALUE));
                hint(ui, q.note);
            },
        );
        let rect = framed.response.rect;
        let hit = ui.interact(rect, ui.auto_id_with(q.title), Sense::click());
        if hit.hovered() {
            ui.painter().rect_stroke(
                rect,
                2.0,
                Stroke::new(1.0, q.rail.colour()),
                egui::StrokeKind::Inside,
            );
            ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
        }
        hit.clicked().then(|| q.action())
    }

    /// One quick start item as a row: the same thing, for the ones that do not
    /// earn a card.
    fn quick_row(ui: &mut egui::Ui, q: &Quick) -> Option<Action> {
        let row = ui.horizontal(|ui| {
            // Proportional rather than the readout face: this is a sentence,
            // and the mono face is for readings.
            theme::Line::new()
                .note(q.title)
                .tint(theme::VALUE)
                .size(13.0)
                .gap(10.0)
                .note(q.note)
                .gap(14.0)
                .legend(q.place)
                .show(ui);
        });
        let rect = Rect::from_min_max(
            row.response.rect.left_top(),
            Pos2::new(ui.max_rect().right(), row.response.rect.bottom()),
        );
        let hit = ui.interact(rect, ui.auto_id_with(q.title), Sense::click());
        if hit.hovered() {
            // Underlined rather than filled: a panel painted over the row
            // after it is drawn covers the words it is highlighting.
            ui.painter().line_segment(
                [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
                Stroke::new(1.0, q.rail.colour()),
            );
            ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
        }
        hit.clicked().then(|| q.action())
    }

    /// A section legend with a rule running off the end of it.
    fn rule(ui: &mut egui::Ui, name: &str) {
        ui.horizontal(|ui| {
            ui.label(legend(name));
            let (rect, _) =
                ui.allocate_exact_size(Vec2::new(ui.available_width(), 8.0), Sense::hover());
            ui.painter().line_segment(
                [Pos2::new(rect.left(), rect.center().y), Pos2::new(rect.right(), rect.center().y)],
                Stroke::new(1.0, theme::ETCH),
            );
        });
    }
}

impl Quick {
    fn action(&self) -> Action {
        match self.goes {
            Goes::View(v) => Action::Open(v),
            Goes::Settings(s) => Action::Panel(s),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every item goes somewhere, and no two go to the same place: a quick
    /// start with the map on it twice is a list nobody curated.
    #[test]
    fn each_quick_start_item_has_its_own_destination() {
        let all: Vec<&Quick> = CARDS.iter().chain(MORE.iter()).collect();
        for (i, a) in all.iter().enumerate() {
            for b in &all[i + 1..] {
                let same = match (&a.goes, &b.goes) {
                    (Goes::View(x), Goes::View(y)) => x == y,
                    (Goes::Settings(x), Goes::Settings(y)) => x == y,
                    _ => false,
                };
                assert!(!same, "{} and {} lead to the same place", a.title, b.title);
            }
        }
    }

    /// The dashboard may not offer itself, which is a tab that does nothing.
    #[test]
    fn nothing_points_at_the_dashboard() {
        for q in CARDS.iter().chain(MORE.iter()) {
            assert!(
                !matches!(q.goes, Goes::View(View::Dashboard)),
                "{} points at the dashboard",
                q.title
            );
        }
    }
}
