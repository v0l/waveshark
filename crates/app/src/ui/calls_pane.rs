//! The call list: who is on the air, on which channel, and for how long.
//!
//! A row per conversation rather than per transmission. What a scanner
//! operator wants from this pane is the state of the band at a glance, so a
//! group that has been busy for a minute is one line that changes rather than
//! forty lines that scroll.

use super::*;
use crate::calls::Call;
use std::sync::atomic::Ordering;

/// Columns, and how wide each is.
///
/// The channel is a column rather than a suffix on the group because it is the
/// actionable one: clicking a row tunes there.
const COLS: [(&str, f32); 7] = [
    ("system", 70.0),
    ("channel", 100.0),
    ("group / party", 190.0),
    ("caller", 120.0),
    ("airtime", 78.0),
    ("overs", 56.0),
    ("last", 60.0),
];

impl App {
    pub(super) fn call_view(&mut self, ui: &mut egui::Ui) {
        let now = std::time::Instant::now();
        // Cloned rather than borrowed: the header below has a button that
        // clears the table, and a row that tunes the receiver, so the pane
        // needs the rest of `self` while it is drawing the list.
        let calls: Vec<Call> = self.calls.active(now).into_iter().cloned().collect();
        let mut clear = false;

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            ui.label(legend("calls"));
            let live = calls.iter().filter(|c| c.live(now)).count();
            ui.label(value(format!("{} heard", calls.len())).size(11.0));
            if live > 0 {
                ui.label(
                    egui::RichText::new(format!("{live} on air"))
                        .color(CRC_OK)
                        .size(11.0),
                );
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(12.0);
                clear = ui.small_button("CLEAR").clicked();
            });
        });
        ui.add_space(6.0);
        if clear {
            self.calls.clear();
            return;
        }

        if calls.is_empty() {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| {
                hint(
                    ui,
                    "Nothing has called yet. A call is any decode that names who it is for, \
                     which today means M17; a trunked system joins this list by naming its \
                     fields the same way.",
                );
            });
            return;
        }

        let width: f32 = COLS.iter().map(|(_, w)| w).sum::<f32>() + 40.0;
        let mut tune_to = None;
        egui::ScrollArea::horizontal().auto_shrink([false, false]).show(ui, |ui| {
            ui.set_min_width(width);
            let (rect, _) =
                ui.allocate_exact_size(Vec2::new(width, Self::ROW_H), Sense::hover());
            let p = ui.painter_at(rect);
            let mut x = rect.left() + 12.0;
            for (name, w) in COLS {
                Self::cell(&p, rect, x, w, name, theme::LEGEND);
                x += w;
            }
            p.line_segment(
                [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
                Stroke::new(1.0, theme::ETCH),
            );

            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                for (n, c) in calls.iter().enumerate() {
                    let (rect, resp) = ui.allocate_exact_size(
                        Vec2::new(width, Self::ROW_H),
                        Sense::click(),
                    );
                    if !ui.is_rect_visible(rect) {
                        continue;
                    }
                    let p = ui.painter_at(rect);
                    if n % 2 == 1 {
                        p.rect_filled(rect, 0.0, Color32::from_rgb(0x24, 0x27, 0x2D));
                    }
                    let live = c.live(now);
                    if live {
                        // A live call is marked at the edge rather than by
                        // colouring the row: the row's colours already say
                        // whether the traffic is worth listening to.
                        p.rect_filled(
                            Rect::from_min_max(
                                rect.left_top(),
                                Pos2::new(rect.left() + 3.0, rect.bottom()),
                            ),
                            0.0,
                            CRC_OK,
                        );
                    }
                    if resp.clicked() {
                        tune_to = Some(c.channel_hz);
                    }
                    let cells = row_cells(c, now, live);
                    let mut x = rect.left() + 12.0;
                    for ((text, col), (_, w)) in cells.iter().zip(COLS) {
                        Self::cell(&p, rect, x, w, text, *col);
                        x += w;
                    }
                }
            });
        });

        // Clicking a call puts the dial on its channel, which is the only
        // thing anybody wants to do with a row that says somebody is talking.
        if let Some(hz) = tune_to {
            self.set_center(hz / 1e6);
        }
    }
}

impl App {
    /// The strip that listens to voice as it decodes.
    ///
    /// Beside the channel strips because it is the same kind of control and
    /// the same output, and apart from them because a call is not a channel:
    /// nothing here is tuned or demodulated by the strip, and it is the front
    /// end watching the channel that decides when there is anything to hear.
    pub(super) fn call_strip(&mut self, ui: &mut egui::Ui) {
        let now = std::time::Instant::now();
        let live = self.calls.active(now).into_iter().find(|c| c.live(now)).cloned();
        // Shown once there is something to listen to, and kept while it is
        // switched on, so turning it on does not make the panel jump about.
        if live.is_none() && !self.call_listen {
            return;
        }
        let playing =
            self.radio.as_ref().is_some_and(|r| r.status.replaying.load(Ordering::Relaxed));

        egui::Frame::NONE
            .fill(theme::PANEL)
            .stroke(Stroke::new(
                if self.call_listen { 1.5 } else { 1.0 },
                if self.call_listen { theme::READOUT } else { theme::ETCH },
            ))
            .corner_radius(2.0)
            .inner_margin(egui::Margin::same(8))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    let (r, _) = ui.allocate_exact_size(Vec2::new(3.0, 16.0), Sense::hover());
                    ui.painter().rect_filled(
                        r,
                        1.0,
                        match &live {
                            Some(_) if self.call_listen => theme::READOUT,
                            Some(_) => CRC_OK,
                            None => theme::ETCH,
                        },
                    );
                    ui.label(legend("calls"));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.selectable_label(self.call_listen, "LISTEN").clicked() {
                            self.call_listen = !self.call_listen;
                            self.send(Cmd::CallAudio {
                                on: self.call_listen,
                                volume: self.call_volume,
                            });
                        }
                    });
                });

                // What is being heard, or the last thing that was: a strip
                // that empties between overs says nothing about the
                // conversation it is following.
                match &live {
                    Some(c) => {
                        ui.label(value(c.title()).size(12.0));
                        ui.label(
                            legend(&format!("{} {:.4} MHz", c.system, c.channel_hz / 1e6)),
                        );
                    }
                    None => {
                        ui.label(legend(if playing { "replaying" } else { "nothing on air" }));
                    }
                }

                ui.horizontal(|ui| {
                    ui.label(legend("vol"));
                    if ui
                        .add(egui::Slider::new(&mut self.call_volume, 0.0..=1.0).show_value(false))
                        .changed()
                        && self.call_listen
                    {
                        self.send(Cmd::CallAudio {
                            on: true,
                            volume: self.call_volume,
                        });
                    }
                });
            });
        ui.add_space(8.0);
    }
}

/// One row's text and colours.
fn row_cells(c: &Call, now: std::time::Instant, live: bool) -> Vec<(String, Color32)> {
    let party = if c.group { theme::TRACE } else { theme::READOUT };
    let airtime = if c.seconds > 0.0 {
        format!("{:.1} s", c.seconds)
    } else {
        // Not every system says how long a transmission was. A dash is
        // honest; a zero would read as a call with no audio in it.
        "-".to_string()
    };
    vec![
        (c.system.clone(), theme::LEGEND),
        (format!("{:.4} MHz", c.channel_hz / 1e6), theme::VALUE),
        (
            if c.encrypted { format!("{}  ENC", c.to) } else { c.to.clone() },
            if c.encrypted { theme::FAULT } else { party },
        ),
        (c.from.clone().unwrap_or_else(|| "-".into()), theme::VALUE),
        (airtime, theme::VALUE),
        (c.overs.to_string(), theme::LEGEND),
        (
            if live { "now".to_string() } else { format!("{}s", c.age(now).as_secs()) },
            if live { CRC_OK } else { theme::LEGEND },
        ),
    ]
}
