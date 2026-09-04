//! The call list: who is on the air, on which channel, and what to listen to.
//!
//! A row per conversation rather than per transmission, and two checkboxes on
//! each: one subscribes to the group, one to whoever is talking. That is the
//! whole of the call bus's configuration. Anything more elaborate would be a
//! rules editor for a decision an operator makes by pointing at the row.

use super::state::{AudioState, CallsState};
use super::*;
use crate::audiobus::Rule;
use crate::calls::Call;

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

/// What the list wants done that it cannot do itself.
pub(super) enum Action {
    /// Tune the dial to the channel a call is on.
    Tune(f64),
    /// Throw the list away.
    Clear,
}

/// The call list, over what it lists and what it has subscribed to.
pub(super) struct CallList<'a> {
    pub st: &'a mut CallsState,
    /// The call bus's own level, which is mixed here rather than in the
    /// channel strip: it is not a channel anybody tuned, and beside the
    /// master it read as a control over everything the receiver plays.
    pub audio: &'a mut AudioState,
    pub radio: Option<&'a Radio>,
    /// Where the pane puts what it wants the receiver to do.
    pub cmds: &'a mut Vec<Cmd>,
}

impl CallList<'_> {
    /// Draw the list, and say what a click asked for.
    pub(super) fn show(mut self, ui: &mut egui::Ui) -> Option<Action> {
        let now = std::time::Instant::now();
        let calls: Vec<Call> = self.st.list.active(now).into_iter().cloned().collect();
        self.st.subscribe_new(&calls, self.cmds);
        let levels = self.radio.map(|r| r.status.call_levels()).unwrap_or_default();
        let mut act = None;

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            let live = calls.iter().filter(|c| c.live(now)).count();
            let mut head =
                theme::Line::new().legend("calls").value(format!("{} heard", calls.len())).size(11.0);
            if live > 0 {
                head = head.value(format!("{live} on air")).tint(CRC_OK).size(11.0);
            }
            head.show(ui);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(12.0);
                if ui.add_enabled(!calls.is_empty(), egui::Button::new("Clear calls")).clicked() {
                    act = Some(Action::Clear);
                }
            });
        });
        ui.add_space(6.0);

        self.mixer(ui);
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
            return act;
        }

        let width: f32 = COLS.iter().map(|(_, w)| w).sum::<f32>() + 24.0;
        let mut tune_to = None;
        let mut toggled: Vec<Rule> = Vec::new();
        egui::ScrollArea::horizontal().auto_shrink([false, false]).show(ui, |ui| {
            ui.set_min_width(width);
            let (rect, _) = ui.allocate_exact_size(Vec2::new(width, widgets::ROW_H), Sense::hover());
            let p = ui.painter_at(rect);
            let mut x = rect.left() + 12.0;
            for (name, w) in COLS {
                widgets::cell(&p, rect, x, w, name, theme::LEGEND);
                x += w;
            }
            p.line_segment(
                [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
                Stroke::new(1.0, theme::ETCH),
            );

            let subs = self.st.subs.clone();
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                for (n, c) in calls.iter().enumerate() {
                    let h = widgets::ROW_H.max(20.0);
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
                            let key = crate::audiobus::AudioBus::key_of(&c.system, c.channel_hz);
                            let peak = levels
                                .iter()
                                .find(|(k, _)| *k == key)
                                .map(|(_, v)| *v)
                                .unwrap_or(0.0);
                            let r = Rect::from_min_size(
                                Pos2::new(x, rect.center().y - super::widgets::VU_H / 2.0),
                                Vec2::new(w - 10.0, super::widgets::VU_H),
                            );
                            Vu::paint(&p, r, peak);
                        } else {
                            widgets::cell(&p, rect, x, *w, text, *col);
                        }
                        x += w;
                    }
                }
            });
        });

        for rule in toggled {
            self.st.toggle(rule, self.cmds);
        }
        // Clicking a row puts the dial on its channel, which is the only
        // other thing anybody wants to do with a call.
        act.or(tune_to.map(Action::Tune))
    }

    /// The call bus: one level for every call the front ends decode, its
    /// mute, and the gain control that rides it.
    ///
    /// Here rather than beside the master fader because that is what it
    /// governs. Sat under the master it looked like a control over the whole
    /// output, and the gain switch under it looked like a limiter on the
    /// speaker rather than one on incoming speech.
    fn mixer(&mut self, ui: &mut egui::Ui) {
        let level = self.radio.map(|r| r.status.call_level()).unwrap_or(0.0);
        let gain_db = self.radio.map(|r| r.status.call_gain_db()).unwrap_or(0.0);
        egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
            widgets::card(
                ui,
                Some(theme::READOUT),
                |ui| {
                    theme::Line::new()
                        .legend("call audio")
                        .note("every call the front ends decode, mixed into the speaker")
                        .show(ui);
                },
                |ui| {
                    ui.horizontal(|ui| {
                        theme::Line::new().legend("level").show(ui);
                        let mut changed = ui
                            .add(Fader::new(&mut self.audio.call_volume, level).width(VU_W))
                            .changed();
                        if crate::icons::icon_button(
                            ui,
                            if self.audio.call_muted {
                                crate::icons::Icon::Mute
                            } else {
                                crate::icons::Icon::Sound
                            },
                            "Mute call audio",
                            true,
                            self.audio.call_muted,
                        )
                        .clicked()
                        {
                            self.audio.call_muted = !self.audio.call_muted;
                            changed = true;
                        }
                        if changed {
                            self.cmds.push(Cmd::CallVolume {
                                volume: self.audio.call_volume,
                                muted: self.audio.call_muted,
                            });
                        }
                        ui.add_space(12.0);
                        // A call arrives at whatever level the transmitting
                        // radio's microphone was set to, which is not
                        // something a listener can fix at the far end.
                        if ui
                            .checkbox(&mut self.audio.call_agc, "Even out the levels")
                            .changed()
                        {
                            self.cmds.push(Cmd::CallAgc(self.audio.call_agc));
                        }
                        if self.audio.call_agc && gain_db.abs() > 0.1 {
                            theme::Line::new().set(format!("{gain_db:+.0} dB")).size(11.0).show(ui);
                        }
                    });
                },
            );
        });
    }
}

/// Which of the columns after the checkboxes is the meter.
const LEVEL_COL: usize = 4;

/// One row's text and colours, from the system column onwards.
fn row_cells(c: &Call, now: std::time::Instant, live: bool) -> Vec<(String, Color32)> {
    let party = if c.group { theme::TRACE } else { theme::READOUT };
    let airtime =
        if c.seconds > 0.0 { format!("{:.1} s", c.seconds) } else { "-".to_string() };
    vec![
        (c.system.clone(), theme::LEGEND),
        (format!("{:.4} MHz", c.channel_hz / 1e6), theme::VALUE),
        // Enciphered traffic is marked with what protects it, so a key
        // that undoes it later has a name to change.
        (
            match &c.cipher {
                Some(how) => format!("{}  {how}", c.to),
                None if c.encrypted => format!("{}  ENC", c.to),
                None => c.to.clone(),
            },
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
