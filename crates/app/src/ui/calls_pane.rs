//! The call list: who is on the air, on which channel, and what to listen to.
//!
//! A row per conversation rather than per transmission, and two checkboxes on
//! each: one subscribes to the group, one to whoever is talking. That is the
//! whole of the call bus's configuration. Anything more elaborate would be a
//! rules editor for a decision an operator makes by pointing at the row.
//!
//! Under it, on a divider, is what the recorder kept: a row per over off the
//! disk, playable. The live list answers who is on the air and the recordings
//! answer who was, which is the same split the map makes between what it
//! draws and the track table beside it. Until this the call log could only be
//! read by running the program again with `--replay`.

use super::state::{AudioState, CallsState};
use super::*;
use crate::calls::Call;
use crate::chain::derived;
use crate::mix::calls::Rule;
use crate::transcripts::TranscriptLog;
use pipeline::param::ParamValue;

/// Columns, and how wide each is.
///
/// The two subscription boxes come first, because that is what this pane is
/// for: the rest of the row is what tells you whether to tick them.
const COLS: [(&str, f32); 14] = [
    ("grp", 34.0),
    ("who", 34.0),
    ("system", 60.0),
    ("channel", 100.0),
    ("group / party", 180.0),
    ("caller", 110.0),
    // What an analogue channel has instead of a talkgroup: the coded
    // squelch its users are set to.
    ("code", 66.0),
    ("codec", 100.0),
    ("level", 70.0),
    ("airtime", 74.0),
    ("overs", 50.0),
    ("last", 56.0),
    ("said", 320.0),
    ("log", 46.0),
];

/// The recordings table's columns.
///
/// Play comes first for the reason the subscription boxes do on the live
/// list: it is what the table is for, and the rest of the row is what tells
/// you whether to press it. Behind it the columns are the live list's in the
/// same order, so the eye reads the two tables the same way.
const LOG_COLS: [(&str, f32); 9] = [
    ("play", 48.0),
    ("save", 48.0),
    ("when", 150.0),
    ("system", 60.0),
    ("channel", 100.0),
    ("group / party", 160.0),
    ("caller", 100.0),
    ("length", 64.0),
    ("peak", 56.0),
];

/// Share of the pane the live list keeps when the recordings are open, and
/// how far the divider can be dragged. Neither half may be squeezed to
/// nothing.
const DEFAULT_LOG_FRAC: f32 = 0.6;

/// Silence between two overs played as one conversation.
///
/// Long enough to hear as a break between speakers and short enough that an
/// afternoon of a quiet talkgroup is still worth sitting through: the pauses
/// as they happened would be the afternoon.
const TIMELINE_GAP_S: f64 = 0.4;

/// The most one conversation plays for, newest kept.
///
/// Ten minutes is a long listen and 115 MB of samples by the time the player
/// has resampled it to the output rate. A longer one is a narrower filter.
const TIMELINE_MAX_S: f64 = 600.0;
const LOG_FRAC_RANGE: std::ops::RangeInclusive<f32> = 0.15..=0.85;

/// What the list wants done that it cannot do itself.
pub(super) enum Action {
    /// Tune the dial to the channel a call is on.
    Tune(f64),
    /// Throw the list away.
    Clear,
    /// Open the recorder's settings.
    Open(super::Settings),
    /// Read everything the model heard on one conversation, in the
    /// transcript view.
    Transcript(common::ConversationKey),
}

/// The call list, over what it lists and what it has subscribed to.
pub(super) struct CallList<'a> {
    pub st: &'a mut CallsState,
    /// The call bus's own level, which is mixed here rather than in the
    /// channel strip: it is not a channel anybody tuned, and beside the
    /// master it read as a control over everything the receiver plays.
    pub audio: &'a mut AudioState,
    pub radio: Option<&'a Radio>,
    /// What the transcriber has read, so a row can offer a way into it. Only
    /// where there is something to show: a button that opens an empty pane is
    /// a button that lies about what was heard.
    pub said: &'a TranscriptLog,
    /// Where the pane puts what it wants the receiver to do.
    pub cmds: &'a mut Vec<Cmd>,
    /// The record, which is where the recorder's switch lives: the stage is
    /// in the graph and the graph only exists while a radio is running, and
    /// a switch somebody expects to stay on cannot be kept somewhere that
    /// goes away when they stop the receiver.
    pub settings: &'a crate::session::Settings,
}

impl CallList<'_> {
    /// Draw the list, and say what a click asked for.
    pub(super) fn show(mut self, ui: &mut egui::Ui) -> Option<Action> {
        let now = std::time::Instant::now();
        let calls: Vec<Call> = self.st.list.active(now).into_iter().cloned().collect();
        let levels = self.radio.map(|r| r.status.call_levels()).unwrap_or_default();
        let mut act = None;

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            let live = calls.iter().filter(|c| c.live(now)).count();
            let mut head =
                Line::new().legend("calls").value(format!("{} heard", calls.len())).size(11.0);
            if live > 0 {
                head = head.value(format!("{live} on air")).tint(CRC_OK).size(11.0);
            }
            head.show(ui);
            let rec = self.radio.and_then(|r| r.status.recorder.lock().clone());
            let open = &mut self.st.log_open;
            let mut opened = false;
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(12.0);
                if ui.add_enabled(!calls.is_empty(), egui::Button::new("Clear calls")).clicked() {
                    act = Some(Action::Clear);
                }
                ui.add_space(12.0);
                if ui
                    .button("SETTINGS")
                    .on_hover_text("What is recorded, and where it is written")
                    .clicked()
                {
                    act = Some(Action::Open(super::Settings::Calls));
                }
                ui.add_space(12.0);
                // The way into what was recorded. Off by default: the pane's
                // question is who is on the air, and the disk is the second
                // question.
                if ui
                    .selectable_label(*open, "Recordings")
                    .on_hover_text("Every over the recorder kept, playable")
                    .clicked()
                {
                    *open = !*open;
                    opened = *open;
                }
                {
                    ui.add_space(12.0);
                    let mut on = self.settings.read(|s| s.calls_on);
                    let where_to = rec
                        .as_ref()
                        .map(|r| r.dir.clone())
                        .unwrap_or_else(|| crate::calllog::calls_dir().display().to_string());
                    if ui
                        .checkbox(&mut on, "Record")
                        .on_hover_text(format!(
                            "Keep every over as Opus in {where_to}, about 2 kB a second of speech"
                        ))
                        .changed()
                    {
                        self.settings.edit(|s| s.calls_on = on);
                    }
                    // What is on the disk, so a watch left running overnight
                    // can be judged without leaving the pane. A recorder at
                    // its limit says so: that state looks like a quiet band
                    // and is not one.
                    if let Some(rec) = &rec
                        && (rec.on || rec.bytes > 0)
                    {
                        let (text, tint) = match rec.full {
                            true => ("folder full".to_string(), theme::FAULT),
                            false => (
                                format!(
                                    "{} recorded, {}",
                                    rec.calls,
                                    super::human_bytes(rec.bytes)
                                ),
                                theme::LEGEND,
                            ),
                        };
                        Line::new().value(text).tint(tint).size(11.0).show(ui);
                    }
                }
            });
            if opened {
                self.st.read_at = None;
            }
        });
        ui.add_space(6.0);

        self.mixer(ui);
        ui.add_space(6.0);

        // Split like the map over its tracks: the live list above, what is
        // on the disk below, and a grip between them for whichever half is
        // being read.
        let open = self.st.log_open;
        let top = ui.cursor().top();
        let usable = match open {
            true => (ui.available_height() - SPLIT_GRIP_H).max(200.0),
            false => ui.available_height().max(120.0),
        };
        let frac = match open {
            true => self.st.log_frac.clamp(*LOG_FRAC_RANGE.start(), *LOG_FRAC_RANGE.end()),
            false => 1.0,
        };
        let mut live = None;
        ui.allocate_ui(Vec2::new(ui.available_width(), usable * frac), |ui| {
            live = self.live_rows(ui, &calls, now, &levels);
        });
        if open {
            let mut splitting = self.st.log_splitting;
            self.st.log_frac = split_divider(
                ui,
                top,
                usable,
                self.st.log_frac,
                &mut splitting,
                LOG_FRAC_RANGE,
                DEFAULT_LOG_FRAC,
            );
            self.st.log_splitting = splitting;
            ui.add_space(4.0);
            self.recordings(ui);
        }
        live.or(act)
    }

    /// The live list itself: a row per conversation.
    fn live_rows(
        &mut self,
        ui: &mut egui::Ui,
        calls: &[Call],
        now: std::time::Instant,
        levels: &[(common::ConversationKey, f32)],
    ) -> Option<Action> {
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
            return None;
        }

        let width: f32 = COLS.iter().map(|(_, w)| w).sum::<f32>() + 24.0;
        let mut tune_to = None;
        let mut read: Option<common::ConversationKey> = None;
        let mut toggled: Vec<Rule> = Vec::new();
        egui::ScrollArea::horizontal().auto_shrink([false, false]).show(ui, |ui| {
            ui.set_min_width(width);
            let (rect, _) = ui.allocate_exact_size(Vec2::new(width, table::ROW_H), Sense::hover());
            let p = ui.painter_at(rect);
            let mut x = rect.left() + 12.0;
            for (name, w) in COLS {
                table::cell(&p, rect, x, w, name, theme::LEGEND);
                x += w;
            }
            p.line_segment(
                [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
                Stroke::new(1.0, theme::ETCH),
            );

            let subs = self.st.subs.clone();
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                for (n, c) in calls.iter().enumerate() {
                    let h = table::ROW_H.max(20.0);
                    let (rect, resp) = ui.allocate_exact_size(Vec2::new(width, h), Sense::click());
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
                    let mut on_caller =
                        caller_rule.as_ref().is_some_and(|r| subs.iter().any(|s| &s.rule == r));
                    let box_at = |i: usize| {
                        let x: f32 =
                            rect.left() + 12.0 + COLS[..i].iter().map(|(_, w)| w).sum::<f32>();
                        Rect::from_min_size(
                            Pos2::new(x, rect.top() + 2.0),
                            Vec2::new(28.0, h - 4.0),
                        )
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

                    // The way into the transcript, on the rows that have one.
                    // A call the model read nothing on gets no button rather
                    // than a button onto an empty pane.
                    let key = c.key();
                    if self.said.has(&key) {
                        let x: f32 = rect.left()
                            + 12.0
                            + COLS[..COLS.len() - 1].iter().map(|(_, w)| w).sum::<f32>();
                        let at = Rect::from_min_size(
                            Pos2::new(x, rect.top() + 1.0),
                            Vec2::new(COLS[COLS.len() - 1].1 - 6.0, h - 2.0),
                        );
                        let mut sub = ui.new_child(egui::UiBuilder::new().max_rect(at));
                        // The button is laid out inside a cell narrower than
                        // the word plus its padding, and a wrapping button
                        // breaks "read" across two lines and spills out of
                        // the row. The column is the width; the label is not
                        // negotiable.
                        sub.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                        if sub
                            .small_button("read")
                            .on_hover_text("Everything the model read on this conversation")
                            .clicked()
                        {
                            read = Some(key);
                        }
                    }

                    if resp.clicked() {
                        tune_to = Some(c.channel_hz);
                    }
                    let cells = row_cells(c, now, live);
                    let mut x = rect.left() + 12.0 + COLS[..2].iter().map(|(_, w)| w).sum::<f32>();
                    for (i, ((text, col), (_, w))) in cells.iter().zip(&COLS[2..]).enumerate() {
                        // The level column is a meter rather than a number:
                        // what it answers is how loud this call is arriving,
                        // and a bar answers that at a glance. The level is
                        // the one the bus measured off the transmission, not
                        // the one after the faders, so a call nobody has
                        // subscribed to still shows that somebody is talking.
                        if i == LEVEL_COL {
                            let key = c.key().meter();
                            let peak = levels
                                .iter()
                                .find(|(k, _)| *k == key)
                                .map(|(_, v)| *v)
                                .unwrap_or(0.0);
                            let r = Rect::from_min_size(
                                Pos2::new(x, rect.center().y - meter::VU_H / 2.0),
                                Vec2::new(w - 10.0, meter::VU_H),
                            );
                            meter::vu(&p, r, peak);
                        } else {
                            table::cell(&p, rect, x, *w, text, *col);
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
        read.map(Action::Transcript).or(tune_to.map(Action::Tune))
    }

    /// What the recorder kept: a row per over off the disk, playable.
    ///
    /// Headers only, walked from the folder every few seconds: the audio of
    /// one over is a megabyte and is read when somebody plays it. The folder
    /// is the running recorder's where there is one, so a receiver told to
    /// write somewhere else lists what it is writing rather than the default.
    fn recordings(&mut self, ui: &mut egui::Ui) {
        let rec = self.radio.and_then(|r| r.status.recorder.lock().clone());
        let dir = rec
            .as_ref()
            .map(|r| std::path::PathBuf::from(&r.dir))
            .unwrap_or_else(crate::calllog::calls_dir);
        self.st.read_recordings(&dir, false);
        // What a dialog came back with, once it has: the thread that opened
        // it did the writing, so this is only the line to show.
        if self.st.saving.as_ref().is_some_and(|p| p.ready().is_some())
            && let Some(note) = self.st.saving.take().map(|p| p.block_and_take())
            && !note.is_empty()
        {
            self.st.log_note = note;
        }

        // Filtered once a frame: the table, the count, what PLAY ALL joins
        // and what EXPORT writes are all the same set, or a listener hears
        // one conversation and saves another.
        let shown = self.st.filtered();
        let seconds: f64 = shown.iter().map(|e| e.call.seconds()).sum();
        let held = self.st.recordings.len();
        let count = match shown.len() == held {
            true => format!("{held} overs"),
            false => format!("{} of {held} overs", shown.len()),
        };
        // A card, like every other panel: what it holds in the header, what
        // can be done to it on the right of that, the filter in the body and
        // what the last press did on the rail at the foot.
        let mut pressed: Option<Press> = None;
        let saving = self.st.saving.is_some();
        let timeline_open = self.st.timeline.open;
        let empty = shown.is_empty();
        egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
            panel::card(
                ui,
                Some(theme::TRACE),
                |ui| {
                    Line::new()
                        .legend("recorded")
                        .value(count)
                        .size(11.0)
                        .gap(12.0)
                        .value(format!("{seconds:.0} s"))
                        .size(11.0)
                        .elided(ui);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        // The folder is on the button rather than on a row:
                        // a path is longer than the pane.
                        if ui
                            .small_button("REFRESH")
                            .on_hover_text(dir.display().to_string())
                            .clicked()
                        {
                            pressed = Some(Press::Refresh);
                        }
                        ui.add_space(6.0);
                        // A folder rather than a file, because what is
                        // listed is many overs; one of them is the row's own
                        // button.
                        if ui
                            .add_enabled(!empty && !saving, egui::Button::new("EXPORT").small())
                            .on_hover_text("Write every over listed into a folder, as Opus")
                            .clicked()
                        {
                            pressed = Some(Press::Export);
                        }
                        ui.add_space(6.0);
                        // The picture of what is listed, which is the way
                        // into playing from a moment rather than from an
                        // over.
                        if ui
                            .selectable_label(timeline_open, "TIMELINE")
                            .on_hover_text("Draw what is listed against the clock")
                            .clicked()
                        {
                            pressed = Some(Press::Timeline);
                        }
                        ui.add_space(6.0);
                        // The overs back to back rather than the air as it
                        // was: filter to a group and this is what was said
                        // on it, with the waiting taken out.
                        if ui
                            .add_enabled(!empty, egui::Button::new("PLAY ALL").small())
                            .on_hover_text(
                                "Play what is listed as one conversation, oldest first, up to \
                                 the newest ten minutes of it",
                            )
                            .clicked()
                        {
                            pressed = Some(Press::PlayAll);
                        }
                    });
                },
                |ui| {
                    form::row_help(
                        ui,
                        "filter",
                        "Every word has to be somewhere on the row, in any order: a talkgroup \
                         and a caller narrows to that caller on that group.",
                        |ui| {
                            let mut clear = false;
                            form::field_then(
                                ui,
                                &mut self.st.filter,
                                "talkgroup, caller, system or frequency",
                                70.0,
                                |ui| clear = ui.small_button("CLEAR").clicked(),
                            );
                            if clear {
                                self.st.filter.clear();
                            }
                        },
                    );
                    if !self.st.log_note.is_empty() {
                        let note = self.st.log_note.clone();
                        // Green for what was written or played, red for what
                        // would not: the lamp is the answer to the last
                        // press and the only place it is reported.
                        let ok = !note.contains("not") && !note.contains("nothing");
                        panel::status(ui, ok, &note);
                    }
                },
            );
        });
        ui.add_space(6.0);
        match pressed {
            None => {}
            Some(Press::Refresh) => self.st.read_recordings(&dir, true),
            Some(Press::Timeline) => {
                self.st.timeline.open = !self.st.timeline.open;
                self.st.timeline.fit(&shown);
            }
            Some(Press::Export) => self.export_all(&shown, &dir, ui.ctx()),
            Some(Press::PlayAll) => self.play_all(&shown),
        }

        if self.st.timeline.open && !shown.is_empty() {
            let left = self.radio.map(|r| r.status.replay_left_s()).unwrap_or(0.0);
            let st = &mut self.st;
            let act = egui::Frame::NONE
                .inner_margin(egui::Margin::symmetric(12, 0))
                .show(ui, |ui| super::timeline::show(ui, &mut st.timeline, &shown, left))
                .inner;
            self.timeline_act(act, &shown, ui.ctx());
            ui.add_space(6.0);
        }

        if self.st.recordings.is_empty() {
            ui.add_space(12.0);
            ui.vertical_centered(|ui| {
                hint(
                    ui,
                    "Nothing has been recorded. Switch Record on and every over heard, on a \
                     voice channel or a voice front end, is kept as Opus.",
                );
            });
            return;
        }

        if shown.is_empty() {
            ui.add_space(12.0);
            ui.vertical_centered(|ui| {
                hint(ui, "Nothing recorded matches the filter.");
            });
            return;
        }

        let width: f32 = LOG_COLS.iter().map(|(_, w)| w).sum::<f32>() + 24.0;
        let mut play = None;
        let mut save = None;
        let mut open = None;
        egui::ScrollArea::horizontal().id_salt("recordings").auto_shrink([false, false]).show(
            ui,
            |ui| {
                ui.set_min_width(width);
                let (rect, _) =
                    ui.allocate_exact_size(Vec2::new(width, table::ROW_H), Sense::hover());
                let p = ui.painter_at(rect);
                let mut x = rect.left() + 12.0;
                for (name, w) in LOG_COLS {
                    table::cell(&p, rect, x, w, name, theme::LEGEND);
                    x += w;
                }
                p.line_segment(
                    [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
                    Stroke::new(1.0, theme::ETCH),
                );
                egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                    for (n, e) in shown.iter().enumerate() {
                        let h = table::ROW_H.max(20.0);
                        let (rect, resp) =
                            ui.allocate_exact_size(Vec2::new(width, h), Sense::click());
                        if !ui.is_rect_visible(rect) {
                            continue;
                        }
                        let p = ui.painter_at(rect);
                        if n % 2 == 1 {
                            p.rect_filled(rect, 0.0, Color32::from_rgb(0x24, 0x27, 0x2D));
                        }
                        // The row is the way into its own conversation: the
                        // filter narrows to that channel and talkgroup and
                        // the timeline draws it.
                        if resp.clicked() {
                            open = Some(e.call.clone());
                        }
                        if resp.hovered() {
                            ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
                        }
                        let buttons = LOG_COLS[0].1 + LOG_COLS[1].1;
                        let mut x = rect.left() + 12.0 + buttons;
                        for ((text, col), (_, w)) in log_cells(&e.call).iter().zip(&LOG_COLS[2..]) {
                            table::cell(&p, rect, x, *w, text, *col);
                            x += w;
                        }
                        let button_at = |i: usize| {
                            let x: f32 = rect.left()
                                + 12.0
                                + LOG_COLS[..i].iter().map(|(_, w)| w).sum::<f32>();
                            Rect::from_min_size(
                                Pos2::new(x, rect.top() + 1.0),
                                Vec2::new(LOG_COLS[i].1 - 6.0, h - 2.0),
                            )
                        };
                        let mut sub = ui.new_child(egui::UiBuilder::new().max_rect(button_at(0)));
                        sub.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                        if sub
                            .small_button("play")
                            .on_hover_text("Play this over through the speaker")
                            .clicked()
                        {
                            play = Some((*e).clone());
                        }
                        let mut sub = ui.new_child(egui::UiBuilder::new().max_rect(button_at(1)));
                        sub.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                        if sub
                            .small_button("save")
                            .on_hover_text("Write this over out as a WAV")
                            .clicked()
                        {
                            save = Some((*e).clone());
                        }
                    }
                });
            },
        );

        // Decoded here rather than on the radio thread: a second of Opus is
        // a fraction of a millisecond, and the thread moving blocks has no
        // business opening files.
        if let Some(e) = play {
            match crate::calllog::speech_of(&e) {
                Some(s) => self.cmds.push(Cmd::Play(std::sync::Arc::new(s))),
                None => {
                    self.st.log_note = format!("{} did not read back", e.file.display());
                }
            }
        }
        if let Some(e) = save {
            self.save_one(&e, &dir, ui.ctx());
        }
    }

    /// Every over listed, into a folder somebody picks, as the Opus it was
    /// recorded as.
    fn export_all(
        &mut self,
        shown: &[crate::calllog::Entry],
        dir: &std::path::Path,
        ctx: &egui::Context,
    ) {
        if self.st.saving.is_some() {
            return;
        }
        let (entries, start, ctx) = (shown.to_vec(), dir.to_path_buf(), ctx.clone());
        self.st.saving = Some(poll_promise::Promise::spawn_thread("export calls", move || {
            let picked = rfd::FileDialog::new()
                .set_title("Export the recordings listed")
                .set_directory(&start)
                .pick_folder();
            ctx.request_repaint();
            let Some(into) = picked else { return String::new() };
            match crate::calllog::export(&entries, &into) {
                Ok((done, 0)) => format!("{done} written to {}", into.display()),
                Ok((done, bad)) => {
                    format!("{done} written to {}, {bad} would not read back", into.display())
                }
                Err(e) => format!("{}: {e}", into.display()),
            }
        }));
    }

    /// The overs listed, back to back, with the waiting between them cut to
    /// a breath.
    fn play_all(&mut self, shown: &[crate::calllog::Entry]) {
        match crate::calllog::timeline(shown, TIMELINE_GAP_S, TIMELINE_MAX_S) {
            Some(s) => {
                let all: f64 = shown.iter().map(|e| e.call.seconds()).sum();
                self.st.log_note = match all > s.seconds() + 1.0 {
                    true => format!(
                        "playing the newest {:.0} s of {:.0} s; narrow the filter for the rest",
                        s.seconds(),
                        all
                    ),
                    false => format!("{} overs, {:.0} s", shown.len(), s.seconds()),
                };
                self.cmds.push(Cmd::Play(std::sync::Arc::new(s)));
            }
            None => self.st.log_note = "none of those overs would decode".into(),
        }
    }

    /// What the timeline asked for: play a stretch, write one out, or close.
    ///
    /// Play and export are the same stretch of the same conversation, taken
    /// at packet granularity: what is heard is what is written.
    fn timeline_act(
        &mut self,
        act: Option<super::timeline::Act>,
        shown: &[crate::calllog::Entry],
        ctx: &egui::Context,
    ) {
        match act {
            None => {}
            Some(super::timeline::Act::Close) => self.st.timeline.open = false,
            Some(super::timeline::Act::Play { at, ranges }) => {
                let from = ranges.first().map(|(a, _)| *a).unwrap_or_default();
                match crate::calllog::audio(shown, &ranges, super::timeline::BREAK_S) {
                    Some(s) => {
                        // The cursor is drawn from how long this turned out
                        // to be, not from how much was asked for: a press
                        // near the end of a conversation asks for five
                        // minutes and gets the twenty seconds that are left.
                        self.st.timeline.playing = Some((at, s.seconds()));
                        self.st.log_note = format!(
                            "playing {} from {}",
                            fmt_span(s.seconds()),
                            crate::segments::when(from).format("%H:%M:%S")
                        );
                        self.cmds.push(Cmd::Play(std::sync::Arc::new(s)));
                    }
                    None => {
                        self.st.timeline.playing = None;
                        self.st.log_note = "nothing was recorded there".into();
                    }
                }
            }
            Some(super::timeline::Act::Export(ranges)) => {
                if self.st.saving.is_some() {
                    return;
                }
                let from = ranges.first().map(|(a, _)| *a).unwrap_or_default();
                let to = ranges.last().map(|(_, b)| *b).unwrap_or_default();
                let packets = crate::calllog::sections(shown, &ranges, super::timeline::BREAK_S);
                if packets.is_empty() {
                    self.st.log_note = "nothing was recorded in that stretch".into();
                    return;
                }
                let name = format!(
                    "{}_{}.opus",
                    crate::segments::when(from).format("%Y%m%d-%H%M%S"),
                    fmt_span((to - from) as f64 / 1e6).replace(' ', "")
                );
                let start = crate::calllog::calls_dir();
                let ctx = ctx.clone();
                self.st.saving =
                    Some(poll_promise::Promise::spawn_thread("export section", move || {
                        let picked = rfd::FileDialog::new()
                            .set_title("Export this stretch of the conversation")
                            .set_directory(&start)
                            .set_file_name(&name)
                            .add_filter("Opus", &["opus"])
                            .save_file();
                        ctx.request_repaint();
                        let Some(path) = picked else { return String::new() };
                        match crate::oggopus::write(
                            &path,
                            &packets,
                            crate::calllog::RATE as u32,
                            crate::calllog::FRAME,
                        ) {
                            Ok(()) => format!("{} written", path.display()),
                            Err(e) => format!("{}: {e}", path.display()),
                        }
                    }));
            }
        }
    }

    /// One over, through a save dialog, as the Opus it was stored as.
    fn save_one(&mut self, e: &crate::calllog::Entry, dir: &std::path::Path, ctx: &egui::Context) {
        if self.st.saving.is_some() {
            return;
        }
        let Some(call) = crate::calllog::call_of(e) else {
            self.st.log_note = format!("{} did not read back", e.file.display());
            return;
        };
        let name = format!("{}.opus", crate::calllog::stem(&call, None));
        let (start, ctx) = (dir.to_path_buf(), ctx.clone());
        self.st.saving = Some(poll_promise::Promise::spawn_thread("export over", move || {
            let picked = rfd::FileDialog::new()
                .set_title("Save this over")
                .set_directory(&start)
                .set_file_name(&name)
                .add_filter("Opus", &["opus"])
                .save_file();
            ctx.request_repaint();
            let Some(path) = picked else { return String::new() };
            match crate::oggopus::write(
                &path,
                &call.frames,
                crate::calllog::RATE as u32,
                crate::calllog::FRAME,
            ) {
                Ok(()) => format!("{} written", path.display()),
                Err(e) => format!("{}: {e}", path.display()),
            }
        }));
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
            panel::card(
                ui,
                Some(theme::READOUT),
                |ui| {
                    Line::new()
                        .legend("call audio")
                        .note("every call the front ends decode, mixed into the speaker")
                        .show(ui);
                },
                |ui| {
                    ui.horizontal(|ui| {
                        Line::new().legend("level").show(ui);
                        if ui
                            .add(Fader::new(&mut self.audio.call_volume, level).width(VU_W))
                            .changed()
                        {
                            self.cmds.push(Cmd::StageParam(
                                derived::CALLS,
                                "vol".into(),
                                ParamValue::Float(self.audio.call_volume as f64),
                            ));
                        }
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
                            self.cmds.push(Cmd::StageParam(
                                derived::CALLS,
                                "mute".into(),
                                ParamValue::Bool(self.audio.call_muted),
                            ));
                        }
                        ui.add_space(12.0);
                        // A call arrives at whatever level the transmitting
                        // radio's microphone was set to, which is not
                        // something a listener can fix at the far end.
                        if ui
                            .checkbox(&mut self.audio.call_agc, "AGC")
                            .on_hover_text("Even out the level between one call and the next")
                            .changed()
                        {
                            self.cmds.push(Cmd::StageParam(
                                derived::CALLS,
                                "agc".into(),
                                ParamValue::Bool(self.audio.call_agc),
                            ));
                        }
                        if self.audio.call_agc && gain_db.abs() > 0.1 {
                            Line::new().set(format!("{gain_db:+.0} dB")).size(11.0).show(ui);
                        }
                    });
                },
            );
        });
    }
}

/// A button in the recordings card's header, applied once the body has had
/// the state it needs.
#[derive(Clone, Copy)]
enum Press {
    Refresh,
    Export,
    Timeline,
    PlayAll,
}

/// A length as somebody would say it: seconds under a minute, minutes above.
fn fmt_span(seconds: f64) -> String {
    match seconds < 60.0 {
        true => format!("{seconds:.0} s"),
        false => format!("{:.0} min", seconds / 60.0),
    }
}

/// One recorded over's row, up to but not including the play button.
fn log_cells(c: &crate::calllog::Call) -> Vec<(String, Color32)> {
    // UTC, as the log stores it and as `--replay` prints it: an over found
    // here has to be findable in the file by the same timestamp.
    let when = crate::segments::when(c.at_us).format("%Y-%m-%d  %H:%M:%S").to_string();
    vec![
        (when, theme::VALUE),
        (c.system.clone(), theme::LEGEND),
        (format!("{:.4} MHz", c.channel_hz as f64 / 1e6), theme::VALUE),
        (c.to.clone().unwrap_or_else(|| "-".into()), theme::TRACE),
        (c.from.clone().unwrap_or_else(|| "-".into()), theme::VALUE),
        (format!("{:.1} s", c.seconds()), theme::VALUE),
        // What the channel was doing, not what the recording is at: a tuned
        // analogue channel with its gain up reaches many times full scale.
        (format!("{:.2}", c.peak), theme::LEGEND),
    ]
}

/// Which of the columns after the checkboxes is the meter.
///
/// Found by name rather than counted by hand: a column inserted in front of
/// it moved the meter onto the wrong one, which paints a bar over a reading
/// and leaves the rest of the row a column out.
const LEVEL_COL: usize = level_col();

const fn level_col() -> usize {
    const fn same(a: &str, b: &str) -> bool {
        let (a, b) = (a.as_bytes(), b.as_bytes());
        if a.len() != b.len() {
            return false;
        }
        let mut i = 0;
        while i < a.len() {
            if a[i] != b[i] {
                return false;
            }
            i += 1;
        }
        true
    }
    let mut i = 2;
    while i < COLS.len() {
        if same(COLS[i].0, "level") {
            return i - 2;
        }
        i += 1;
    }
    panic!("the calls table has no level column")
}

/// One row's text and colours, from the system column onwards.
fn row_cells(c: &Call, now: std::time::Instant, live: bool) -> Vec<(String, Color32)> {
    let party = if c.group { theme::TRACE } else { theme::READOUT };
    let airtime = if c.seconds > 0.0 { format!("{:.1} s", c.seconds) } else { "-".to_string() };
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
        (c.code.clone().unwrap_or_else(|| "-".into()), theme::LEGEND),
        (c.codec.unwrap_or("-").to_string(), theme::LEGEND),
        // The meter is painted over this one; the text is what a row without
        // a level would have shown.
        (String::new(), theme::VALUE),
        (airtime, theme::VALUE),
        (c.overs.to_string(), theme::LEGEND),
        (
            if live { "now".to_string() } else { format!("{}s", c.age(now).as_secs()) },
            if live { CRC_OK } else { theme::LEGEND },
        ),
        (c.transcript.clone().unwrap_or_default(), theme::READOUT),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The filter narrows by anything on the row, in any order, and a word
    /// that matches nothing leaves nothing.
    #[test]
    fn the_filter_matches_every_word_against_the_row() {
        let call = |from: &str, to: &str, hz: u64, at_us: u64| crate::calllog::Entry {
            call: crate::calllog::Call {
                at_us,
                channel_hz: hz,
                duration_ms: 1_000,
                peak: 0.5,
                system: "M17".into(),
                from: Some(from.into()),
                to: Some(to.into()),
                frames: Vec::new(),
            },
            file: std::path::PathBuf::from("day.wscal"),
            at: 0,
            len: 0,
        };
        let mut st = CallsState::default();
        st.recordings =
            vec![call("M0ABC", "ALL", 434_000_000, 2), call("M0XYZ", "GB7XX", 430_512_500, 1)];
        assert_eq!(st.filtered().len(), 2, "an empty filter hides nothing");

        st.filter = "m0abc".into();
        assert_eq!(st.filtered().len(), 1);
        assert_eq!(st.filtered()[0].call.from.as_deref(), Some("M0ABC"));

        // Two words, neither in the order the row writes them: the caller and
        // the frequency he was on.
        st.filter = "430.5125 m0xyz".into();
        assert_eq!(st.filtered().len(), 1);
        assert_eq!(st.filtered()[0].call.to.as_deref(), Some("GB7XX"));

        st.filter = "gb7xx m0abc".into();
        assert!(st.filtered().is_empty(), "two words that are on different rows matched one");
    }

    /// A row has exactly one cell per header, and the meter is on the level
    /// column.
    ///
    /// The two are counted in different places, so a column added to one and
    /// not the other silently shifts every cell after it: the code column
    /// did exactly that, painting the level meter over the codec.
    #[test]
    fn every_column_has_a_cell_behind_it() {
        let now = std::time::Instant::now();
        let call = Call {
            system: "Audio".into(),
            channel_hz: 446_050_000.0,
            to: "PMR5".into(),
            from: Some("101".into()),
            group: false,
            encrypted: false,
            cipher: None,
            codec: None,
            code: Some("D023".into()),
            first: now,
            last: now,
            overs: 1,
            seconds: 1.0,
            transcript: None,
            heard_s: Default::default(),
            by_bus: true,
        };
        // Two checkboxes at the front and the log button at the back are
        // drawn rather than written, so the text cells are what is left.
        let cells = row_cells(&call, now, true);
        assert_eq!(
            cells.len(),
            COLS.len() - 3,
            "{} cells against {} text headers",
            cells.len(),
            COLS.len() - 3
        );
        assert_eq!(COLS[LEVEL_COL + 2].0, "level");
        // The code the group is using is on the row, where the column says.
        let at = COLS[2..].iter().position(|(n, _)| *n == "code").expect("a code column");
        assert_eq!(cells[at].0, "D023");
    }
}
