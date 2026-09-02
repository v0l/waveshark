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
const COLS: [(&str, f32); 10] = [
    ("grp", 34.0),
    ("who", 34.0),
    ("system", 60.0),
    ("channel", 100.0),
    ("group / party", 180.0),
    ("caller", 110.0),
    ("level", 70.0),
    ("airtime", 74.0),
    ("overs", 50.0),
    ("last", 56.0),
];

impl App {
    pub(super) fn call_view(&mut self, ui: &mut egui::Ui) {
        let now = std::time::Instant::now();
        let calls: Vec<Call> = self.calls.active(now).into_iter().cloned().collect();
        // Every group is listened to unless it was turned off. A scanner that
        // hears nothing until it is configured is a scanner nobody hears
        // anything on, and the box on the row is how it is turned off.
        self.subscribe_new_groups(&calls);
        let levels = self
            .radio
            .as_ref()
            .map(|r| r.status.call_levels())
            .unwrap_or_default();

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
                    for (i, ((text, col), (_, w))) in cells.iter().zip(&COLS[2..]).enumerate() {
                        // The level column is a meter rather than a number:
                        // what it answers is whether this call is reaching
                        // the speaker, and a bar answers that at a glance.
                        if i == LEVEL_COL {
                            let key = crate::callbus::CallBus::key_of(&c.system, c.channel_hz);
                            let peak = levels
                                .iter()
                                .find(|(k, _)| *k == key)
                                .map(|(_, v)| *v)
                                .unwrap_or(0.0);
                            meter(&p, rect, x, *w, peak);
                        } else {
                            Self::cell(&p, rect, x, *w, text, *col);
                        }
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

    /// Subscribe to any group not heard of before, unless it was switched
    /// off by hand.
    ///
    /// The opt-outs are remembered separately, so a group turned off does not
    /// come back the next time somebody transmits on it.
    fn subscribe_new_groups(&mut self, calls: &[Call]) {
        let mut added = false;
        for c in calls {
            let rule = Rule::Group(c.to.clone());
            if self.call_optout.contains(&rule) || self.call_subs.iter().any(|s| s.rule == rule) {
                continue;
            }
            self.call_subs.push(Subscription::new(rule));
            added = true;
        }
        if added {
            self.send(Cmd::CallSubs(self.call_subs.clone()));
        }
    }

    /// Subscribe to a rule, or drop it if it is already there.
    fn toggle_call_sub(&mut self, rule: Rule) {
        match self.call_subs.iter().position(|s| s.rule == rule) {
            Some(i) => {
                self.call_subs.remove(i);
                self.call_optout.push(rule);
            }
            None => {
                self.call_optout.retain(|r| r != &rule);
                self.call_subs.push(Subscription::new(rule));
            }
        }
        self.send(Cmd::CallSubs(self.call_subs.clone()));
    }
}

/// Which of the columns after the checkboxes is the meter.
const LEVEL_COL: usize = 4;

/// The level bar in a row, drawn in the cell rather than as a widget: the
/// whole table is painted, and a bar is two rectangles.
fn meter(p: &egui::Painter, row: Rect, x: f32, w: f32, peak: f32) {
    let h = 6.0;
    let r = Rect::from_min_size(
        Pos2::new(x, row.center().y - h / 2.0),
        Vec2::new(w - 10.0, h),
    );
    p.rect_filled(r, 1.0, theme::WELL);
    if peak > 0.001 {
        let filled = (peak.clamp(0.0, 1.0).sqrt() * r.width()).max(2.0);
        p.rect_filled(
            Rect::from_min_size(r.min, Vec2::new(filled, r.height())),
            1.0,
            if peak > 0.98 { theme::FAULT } else { CRC_OK },
        );
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
        // The meter is painted over this one; the text is what a row without
        // a level would have shown.
        (String::new(), theme::VALUE),
        (airtime, theme::VALUE),
        (c.overs.to_string(), theme::LEGEND),
        (
            if live { "now".to_string() } else { format!("{}s", c.age(now).as_secs()) },
            if live { CRC_OK } else { theme::LEGEND },
        ),
    ]
}
