//! The call list: who is on the air, on which channel, and what to listen to.
//!
//! A row per conversation rather than per transmission, and two checkboxes on
//! each: one subscribes to the group, one to whoever is talking. That is the
//! whole of the call bus's configuration. Anything more elaborate would be a
//! rules editor for a decision an operator makes by pointing at the row.

use super::*;
use crate::callbus::{Rule, Subscription};
use crate::calls::Call;
use std::sync::atomic::Ordering;

/// Columns, and how wide each is.
///
/// The two subscription boxes come first, because that is what this pane is
/// for: the rest of the row is what tells you whether to tick them.
const COLS: [(&str, f32); 9] = [
    ("grp", 34.0),
    ("who", 34.0),
    ("system", 60.0),
    ("channel", 100.0),
    ("group / party", 180.0),
    ("caller", 110.0),
    ("airtime", 74.0),
    ("overs", 50.0),
    ("last", 56.0),
];

/// Width of the subscription pane beside the list.
const SIDE_W: f32 = 230.0;

impl App {
    pub(super) fn call_view(&mut self, ui: &mut egui::Ui) {
        let now = std::time::Instant::now();
        let calls: Vec<Call> = self.calls.active(now).into_iter().cloned().collect();

        Panel::right("call-subs")
            .default_size(SIDE_W)
            .frame(
                egui::Frame::NONE
                    .fill(theme::PANEL)
                    .inner_margin(egui::Margin::symmetric(12, 10)),
            )
            .show_inside(ui, |ui| self.call_subs_pane(ui));

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            ui.label(legend("calls"));
            let live = calls.iter().filter(|c| c.live(now)).count();
            ui.label(value(format!("{} heard", calls.len())).size(11.0));
            if live > 0 {
                ui.label(egui::RichText::new(format!("{live} on air")).color(CRC_OK).size(11.0));
            }
        });
        ui.add_space(6.0);

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

        let width: f32 = COLS.iter().map(|(_, w)| w).sum::<f32>() + 24.0;
        let mut tune_to = None;
        let mut toggled: Vec<Rule> = Vec::new();
        egui::ScrollArea::horizontal().auto_shrink([false, false]).show(ui, |ui| {
            ui.set_min_width(width);
            let (rect, _) = ui.allocate_exact_size(Vec2::new(width, Self::ROW_H), Sense::hover());
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

            let subs = self.call_subs.clone();
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                for (n, c) in calls.iter().enumerate() {
                    let h = Self::ROW_H.max(20.0);
                    let (rect, resp) =
                        ui.allocate_exact_size(Vec2::new(width, h), Sense::click());
                    if !ui.is_rect_visible(rect) {
                        continue;
                    }
                    let p = ui.painter_at(rect);
                    if n % 2 == 1 {
                        p.rect_filled(rect, 0.0, Color32::from_rgb(0x24, 0x27, 0x2D));
                    }
                    let live = c.live(now);
                    if live {
                        p.rect_filled(
                            Rect::from_min_max(
                                rect.left_top(),
                                Pos2::new(rect.left() + 3.0, rect.bottom()),
                            ),
                            0.0,
                            CRC_OK,
                        );
                    }

                    // The two boxes. Drawn as widgets rather than painted,
                    // so they behave like every other checkbox in the app.
                    let group_rule = Rule::Group(c.to.clone());
                    let caller_rule = c.from.clone().map(Rule::Caller);
                    let mut on_group = subs.iter().any(|s| s.rule == group_rule);
                    let mut on_caller = caller_rule
                        .as_ref()
                        .is_some_and(|r| subs.iter().any(|s| &s.rule == r));
                    let box_at = |i: usize| {
                        let x: f32 = rect.left() + 12.0 + COLS[..i].iter().map(|(_, w)| w).sum::<f32>();
                        Rect::from_min_size(Pos2::new(x, rect.top() + 2.0), Vec2::new(28.0, h - 4.0))
                    };
                    let mut sub = ui.new_child(egui::UiBuilder::new().max_rect(box_at(0)));
                    if sub.checkbox(&mut on_group, "").changed() {
                        toggled.push(group_rule);
                    }
                    if let Some(r) = caller_rule {
                        let mut sub = ui.new_child(egui::UiBuilder::new().max_rect(box_at(1)));
                        if sub.checkbox(&mut on_caller, "").changed() {
                            toggled.push(r);
                        }
                    }

                    if resp.clicked() {
                        tune_to = Some(c.channel_hz);
                    }
                    let cells = row_cells(c, now, live);
                    let mut x = rect.left() + 12.0 + COLS[..2].iter().map(|(_, w)| w).sum::<f32>();
                    for ((text, col), (_, w)) in cells.iter().zip(&COLS[2..]) {
                        Self::cell(&p, rect, x, *w, text, *col);
                        x += w;
                    }
                }
            });
        });

        for rule in toggled {
            self.toggle_call_sub(rule);
        }
        // Clicking a row puts the dial on its channel, which is the only
        // other thing anybody wants to do with a call.
        if let Some(hz) = tune_to {
            self.set_center(hz / 1e6);
        }
    }

    /// The subscription pane: everything the call bus is listening to.
    fn call_subs_pane(&mut self, ui: &mut egui::Ui) {
        ui.label(legend("listening to"));
        ui.add_space(6.0);

        let heard = self
            .radio
            .as_ref()
            .and_then(|r| r.status.call_heard.lock().clone())
            .filter(|_| !self.call_subs.is_empty());
        let replaying =
            self.radio.as_ref().is_some_and(|r| r.status.replaying.load(Ordering::Relaxed));

        if self.call_subs.is_empty() {
            hint(
                ui,
                "Nothing. Tick a box beside a group or a caller in the list and their \
                 audio is mixed into the master output as it decodes.",
            );
        }

        let mut remove = None;
        let mut changed = false;
        for (i, s) in self.call_subs.iter_mut().enumerate() {
            egui::Frame::NONE
                .fill(theme::PANEL)
                .stroke(Stroke::new(1.0, if s.muted { theme::ETCH } else { theme::READOUT }))
                .corner_radius(2.0)
                .inner_margin(egui::Margin::same(6))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(value(s.rule.label()).size(11.0));
                        ui.with_layout(
                            egui::Layout::right_to_left(egui::Align::Center),
                            |ui| {
                                if ui.small_button("X").clicked() {
                                    remove = Some(i);
                                }
                                if ui.selectable_label(s.muted, "M").clicked() {
                                    s.muted = !s.muted;
                                    changed = true;
                                }
                            },
                        );
                    });
                    if ui
                        .add(egui::Slider::new(&mut s.volume, 0.0..=1.0).show_value(false))
                        .changed()
                    {
                        changed = true;
                    }
                });
            ui.add_space(6.0);
        }
        if let Some(i) = remove {
            self.call_subs.remove(i);
            changed = true;
        }

        ui.add_space(4.0);
        let everything = Rule::Everything;
        let mut all = self.call_subs.iter().any(|s| s.rule == everything);
        if ui.checkbox(&mut all, "Everything").changed() {
            self.toggle_call_sub(everything);
        }
        hint(ui, "Every call every front end decodes, whoever it is for.");

        ui.add_space(10.0);
        ui.separator();
        ui.add_space(6.0);
        ui.label(legend("on air"));
        match &heard {
            Some(h) => ui.label(value(h.clone()).size(12.0)),
            None => ui.label(legend("nothing")),
        };

        // What the bus actually put into the mix. An empty meter with a call
        // on air means the subscription is wrong; a full one with silence
        // from the speaker means the fault is past this point.
        let peak = self.radio.as_ref().map(|r| r.status.call_peak()).unwrap_or(0.0);
        ui.add_space(6.0);
        let (r, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 8.0), Sense::hover());
        ui.painter().rect_filled(r, 1.0, theme::WELL);
        if peak > 0.0 {
            let w = (peak.clamp(0.0, 1.0) * r.width()).max(2.0);
            ui.painter().rect_filled(
                Rect::from_min_size(r.min, Vec2::new(w, r.height())),
                1.0,
                if peak > 0.98 { theme::FAULT } else { CRC_OK },
            );
        }
        ui.label(legend(&format!("bus level {:.0}%", peak * 100.0)));
        if replaying {
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("replaying").color(CRC_OK).size(11.0));
                if ui.small_button("STOP").clicked() {
                    self.send(Cmd::StopPlay);
                }
            });
        }

        if changed {
            self.send(Cmd::CallSubs(self.call_subs.clone()));
        }
    }

    /// Subscribe to a rule, or drop it if it is already there.
    fn toggle_call_sub(&mut self, rule: Rule) {
        match self.call_subs.iter().position(|s| s.rule == rule) {
            Some(i) => {
                self.call_subs.remove(i);
            }
            None => self.call_subs.push(Subscription::new(rule)),
        }
        self.send(Cmd::CallSubs(self.call_subs.clone()));
    }
}

/// One row's text and colours, from the system column onwards.
fn row_cells(c: &Call, now: std::time::Instant, live: bool) -> Vec<(String, Color32)> {
    let party = if c.group { theme::TRACE } else { theme::READOUT };
    let airtime =
        if c.seconds > 0.0 { format!("{:.1} s", c.seconds) } else { "-".to_string() };
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
