//! What was said, as the model on the audio bus read it.
//!
//! The transcriber has been running since long before there was anywhere to
//! read it: the call list showed the last line of an over in a column 320
//! points wide and everything else was thrown away on screen while the node
//! held an afternoon of it. This is that log, drawn.
//!
//! Two things it shows that a list of lines would not. A conversation is a
//! key, so the pane can be opened on one and nothing else, which is what the
//! call list's button does. And the model itself is a piece of equipment
//! with a state: which one, where its files are, whether they are there at
//! all, what it is running on, and how fast it read the last window. Without
//! that, a model that was never downloaded, one that failed to load, one too
//! slow to keep up and a quiet band all look the same, which is to say they
//! all look like an empty pane.

use super::state::TranscriptState;
use super::*;
use crate::transcripts::{Engine, ModelState, Utterance, LIVE};

/// The transcript, over what was said and what read it.
pub(super) struct Transcript<'a> {
    pub st: &'a mut TranscriptState,
    /// The transcriber in the running graph, or `None` where there is none:
    /// no radio, or a build made without speech to text.
    pub engine: Option<Engine>,
    pub cmds: &'a mut Vec<Cmd>,
}

/// What the pane wants done that it cannot do itself.
pub(super) enum Action {
    /// Throw the transcript away.
    Clear,
}

impl Transcript<'_> {
    pub(super) fn show(mut self, ui: &mut egui::Ui) -> Option<Action> {
        let now = std::time::Instant::now();
        let mut act = None;

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            theme::Line::new()
                .legend("transcript")
                .value(format!("{} lines", self.st.log.len()))
                .size(11.0)
                .show(ui);
            if !self.st.log.is_empty() {
                ui.add_space(12.0);
                ui.add(
                    egui::TextEdit::singleline(&mut self.st.filter)
                        .hint_text("filter")
                        .desired_width(160.0),
                );
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(12.0);
                if ui
                    .add_enabled(!self.st.log.is_empty(), egui::Button::new("Clear transcript"))
                    .clicked()
                {
                    act = Some(Action::Clear);
                }
            });
        });
        ui.add_space(6.0);

        self.model_card(ui);
        ui.add_space(6.0);

        // The conversation being read on its own, with the way back out of
        // it. Drawn whether or not it has any lines: a call whose speech the
        // model read nothing from is a fact worth seeing, and dropping the
        // filter silently would show somebody else's words instead.
        if let Some(key) = self.st.only.clone() {
            egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
                ui.horizontal(|ui| {
                    theme::Line::new().legend("conversation").set(who(&key)).show(ui);
                    ui.add_space(8.0);
                    if ui.button("Everything heard").clicked() {
                        self.st.only = None;
                    }
                });
            });
            ui.add_space(4.0);
        }

        let needle = self.st.filter.to_lowercase();
        let lines: Vec<&Utterance> = match &self.st.only {
            Some(key) => self.st.log.of(key).iter().collect(),
            None => self.st.log.recent(usize::MAX),
        };
        let shown: Vec<&Utterance> = lines
            .into_iter()
            .filter(|u| {
                needle.is_empty()
                    || u.text.to_lowercase().contains(&needle)
                    || who(&u.key).to_lowercase().contains(&needle)
            })
            .collect();

        if shown.is_empty() {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| {
                hint(ui, self.nothing_yet());
            });
            return act;
        }

        // Newest at the bottom and stuck there, which is how a conversation
        // reads: the line being spoken now is where the eye already is, and
        // a partial that grows is the same row getting taller.
        let wall = std::time::SystemTime::now();
        let width = ui.available_width().max(COLS.iter().map(|(_, w)| w).sum::<f32>() + 300.0);
        let text_w = width - 24.0 - COLS.iter().map(|(_, w)| w).sum::<f32>();
        let (rect, _) = ui.allocate_exact_size(Vec2::new(width, widgets::ROW_H), Sense::hover());
        {
            let p = ui.painter_at(rect);
            let mut x = rect.left() + 12.0;
            for (name, w) in COLS {
                widgets::cell(&p, rect, x, w, name, theme::LEGEND);
                x += w;
            }
            widgets::cell(&p, rect, x, text_w, "text", theme::LEGEND);
            p.line_segment(
                [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
                Stroke::new(1.0, theme::ETCH),
            );
        }
        egui::ScrollArea::vertical().stick_to_bottom(true).auto_shrink([false, false]).show(
            ui,
            |ui| {
                for (n, u) in shown.iter().enumerate() {
                    row(ui, u, n, now, wall, width, text_w);
                }
                ui.add_space(8.0);
            },
        );
        act
    }

    /// What to say when there is nothing to read, which is four different
    /// situations and only one of them is "nobody is talking".
    fn nothing_yet(&self) -> &'static str {
        if !self.st.filter.is_empty() {
            return "Nothing said matches that.";
        }
        match self.engine.as_ref().map(|e| (&e.health.state, e.enabled)) {
            None => {
                "There is no transcriber in the graph. Either the radio is not running, or \
                 this build was made without the stt feature, which is what compiles the \
                 Whisper decoder in."
            }
            Some((ModelState::Failed(_), _)) => {
                "The model could not be loaded; what went wrong is above."
            }
            Some((_, false)) => "Transcription is switched off.",
            Some((ModelState::Cold, _)) => {
                "Nothing has been transcribed yet. The model loads on the first speech worth \
                 reading, or now, from the button above."
            }
            Some(_) => {
                "The model is loaded and nobody has said anything on a channel the receiver \
                 is decoding as speech. A channel has to be marked as voice for its audio to \
                 reach here."
            }
        }
    }

    /// The equipment: which model, where, on what, and what it has done.
    fn model_card(&mut self, ui: &mut egui::Ui) {
        let Some(e) = self.engine.clone() else {
            egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
                widgets::card(
                    ui,
                    Some(theme::LEGEND),
                    |ui| {
                        theme::Line::new().legend("model").set("none").tint(theme::LEGEND).show(ui);
                    },
                    |ui| {
                        hint(
                            ui,
                            "No transcriber is running. It is a node on the audio bus tap, so \
                             it appears when the radio does.",
                        );
                    },
                );
            });
            return;
        };
        let rail = match e.health.state {
            ModelState::Ready => theme::TRACE,
            ModelState::Failed(_) => theme::FAULT,
            _ => theme::READOUT,
        };
        // The header and the body of a card are drawn by two closures at
        // once, so what they ask for is collected rather than pushed.
        let asked = self.st.asked;
        let want: std::cell::RefCell<(Option<bool>, bool, Option<String>, Option<String>)> =
            std::cell::RefCell::new((None, false, None, None));
        egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
            widgets::card(
                ui,
                Some(rail),
                |ui| {
                    // The model by its short name. What is on disc is what
                    // runs, and where a directory holds something other
                    // than the pick, the files line below says so.
                    let mut head = theme::Line::new().legend("model").set(e.label.clone());
                    head = head.legend("state").value(e.health.state.label()).tint(rail);
                    if !e.health.device.is_empty() {
                        head = head.legend("on").value(&e.health.device);
                    }
                    head.size(11.0).show(ui);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let mut on = e.enabled;
                        if ui
                            .checkbox(&mut on, "Transcribe")
                            .on_hover_text("Read what is heard on every channel marked as voice")
                            .changed()
                        {
                            want.borrow_mut().0 = Some(on);
                        }
                    });
                },
                |ui| {
                    // The pick, and where it runs, as two settings rows so
                    // the legends make a column and the boxes make another.
                    // Sent as ids rather than positions, so what the patch
                    // records survives the list growing.
                    let small = |t: &str| egui::RichText::new(t).size(11.0);
                    widgets::row(ui, "model", |ui| {
                        egui::ComboBox::from_id_salt("stt-model")
                            .selected_text(small(&e.label))
                            .width(300.0)
                            .show_ui(ui, |ui| {
                                for m in &e.models {
                                    let mut text = m.label.clone();
                                    if m.present {
                                        text.push_str("  (on disc)");
                                    } else if m.bytes > 0 {
                                        text.push_str(&format!(
                                            "  ({})",
                                            super::human_bytes(m.bytes)
                                        ));
                                    }
                                    if ui.selectable_label(m.id == e.model, small(&text)).clicked()
                                    {
                                        want.borrow_mut().2 = Some(m.id.clone());
                                    }
                                }
                            });
                    });
                    widgets::row(ui, "run on", |ui| {
                        let now = e
                            .devices
                            .iter()
                            .find(|(id, _)| *id == e.device_choice)
                            .map(|(_, l)| l.clone())
                            .unwrap_or_else(|| e.device_choice.clone());
                        egui::ComboBox::from_id_salt("stt-device")
                            .selected_text(small(&now))
                            .width(300.0)
                            .show_ui(ui, |ui| {
                                for (id, label) in &e.devices {
                                    if ui
                                        .selectable_label(*id == e.device_choice, small(label))
                                        .clicked()
                                    {
                                        want.borrow_mut().3 = Some(id.clone());
                                    }
                                }
                            });
                    });
                    // What is on disc, not what would be fetched: a
                    // directory filled by hand holds whatever was put there,
                    // and that is what runs. The path is on hover, where
                    // somebody checking the files can read it and nobody
                    // else has to.
                    let mut l = theme::Line::new();
                    l = if e.health.present {
                        l.legend("on disc")
                            .value(super::human_bytes(e.health.bytes))
                            .value(&e.health.weights)
                            .value(&e.health.flavour)
                    } else {
                        l.legend("on disc").value("nothing yet").tint(theme::READOUT)
                    };
                    let where_ = if e.dir.is_empty() { "unset".to_string() } else { e.dir.clone() };
                    l.size(11.0).show(ui).on_hover_text(where_);
                    // A download is minutes of nothing otherwise: the
                    // smallest model is 74 MB and the largest a few
                    // gigabytes, and a card that says only "downloading"
                    // reads the same as one that has hung.
                    if matches!(e.health.state, ModelState::Fetching) {
                        let f = &e.health.fetch;
                        let mut l = theme::Line::new().legend("fetching");
                        l = if f.file.is_empty() {
                            l.value("asking the hub")
                        } else {
                            l.value(&f.file)
                        };
                        if f.total > 0 {
                            l = l.value(format!(
                                "{} of {}",
                                super::human_bytes(f.done),
                                super::human_bytes(f.total)
                            ));
                        } else if f.done > 0 {
                            l = l.value(super::human_bytes(f.done));
                        }
                        if f.files > 1 {
                            l = l.legend("file").value(format!("{} of {}", f.files_done + 1, f.files));
                        }
                        l.size(11.0).show(ui);
                        if let Some(x) = f.fraction() {
                            let (r, _) = ui.allocate_exact_size(
                                egui::vec2(ui.available_width().min(320.0), 6.0),
                                egui::Sense::hover(),
                            );
                            let p = ui.painter();
                            p.rect_filled(r, 1.0, theme::WELL);
                            let mut done = r;
                            done.set_width(r.width() * x.clamp(0.0, 1.0));
                            p.rect_filled(done, 1.0, theme::TRACE);
                        }
                    }
                    ui.horizontal(|ui| {
                        let mut l = theme::Line::new().legend("read").value(e.health.reads.to_string());
                        if let Some(x) = e.speed() {
                            // Against real time, because that is the number
                            // that decides whether the receiver keeps up:
                            // below one, speech arrives faster than it can
                            // be read and the partials fall behind.
                            l = l
                                .legend("last")
                                .value(format!("{:.1} s in {} ms", e.health.last_audio_s, e.health.last_ms))
                                .legend("speed")
                                .value(format!("{x:.1}x real time"))
                                .tint(if x < 1.0 { theme::FAULT } else { theme::VALUE });
                        }
                        if e.speakers > 0 {
                            l = l
                                .legend("holding")
                                .value(format!("{:.1} s on {}", e.holding_s, e.speakers))
                                .tint(theme::TRACE);
                        }
                        if e.busy {
                            l = l.legend("reading");
                        }
                        l.size(11.0).show(ui);
                    });
                    if let ModelState::Failed(why) = &e.health.state {
                        theme::Line::new().words(why).tint(theme::FAULT).wrapped(ui);
                    }
                    if !e.health.note.is_empty() {
                        theme::Line::new().words(&e.health.note).tint(theme::READOUT).wrapped(ui);
                    }
                    // Loading it by hand is the only way to find out whether
                    // transcription works on this machine without waiting
                    // for somebody to key up: the model is fetched and
                    // loaded by the first speech worth reading, and that may
                    // be an hour away.
                    let cold = matches!(e.health.state, ModelState::Cold | ModelState::Failed(_));
                    if cold {
                        ui.horizontal(|ui| {
                            let label = if e.health.present {
                                "Load the model now"
                            } else {
                                "Download and load the model"
                            };
                            if ui.add_enabled(!asked, egui::Button::new(label)).clicked() {
                                want.borrow_mut().1 = true;
                            }
                            if !e.health.present {
                                let size = e
                                    .models
                                    .iter()
                                    .find(|m| m.id == e.model)
                                    .filter(|m| m.bytes > 0)
                                    .map(|m| super::human_bytes(m.bytes));
                                hint(
                                    ui,
                                    &match size {
                                        Some(s) => {
                                            format!("{s} from {}, over the network, once.", e.repo)
                                        }
                                        None => {
                                            format!("From {}, over the network, once.", e.repo)
                                        }
                                    },
                                );
                            }
                        });
                    }
                },
            );
        });
        let (enabled, load, model, device) = want.into_inner();
        if let Some(id) = model {
            self.cmds.push(Cmd::NodeParam(
                e.node,
                "model".into(),
                pipeline::param::ParamValue::Text(id),
            ));
        }
        if let Some(id) = device {
            self.cmds.push(Cmd::NodeParam(
                e.node,
                "device".into(),
                pipeline::param::ParamValue::Text(id),
            ));
        }
        if let Some(on) = enabled {
            self.cmds.push(Cmd::NodeParam(
                e.node,
                "enabled".into(),
                pipeline::param::ParamValue::Bool(on),
            ));
        }
        if load {
            self.st.asked = true;
            self.cmds.push(Cmd::NodeParam(
                e.node,
                "load".into(),
                pipeline::param::ParamValue::Bool(true),
            ));
        }
        // The button is offered again once the model has left the cold or
        // failed state and come back to it, which is what a changed model
        // directory or a second failure looks like.
        if !matches!(e.health.state, ModelState::Cold | ModelState::Failed(_)) {
            self.st.asked = false;
        }
    }
}

/// Columns before the text, and how wide each is. The text takes the rest.
const COLS: [(&str, f32); 4] =
    [("time", 76.0), ("freq", 96.0), ("speaker", 120.0), ("group / chan", 120.0)];

/// One row of the log: when, where, who, to whom, and the words. The words
/// wrap and the row grows to hold them, since a transmission is a sentence
/// or three and a clipped sentence is not a transcript.
fn row(
    ui: &mut egui::Ui,
    u: &Utterance,
    n: usize,
    now: std::time::Instant,
    wall: std::time::SystemTime,
    width: f32,
    text_w: f32,
) {
    let age = now.saturating_duration_since(u.at);
    // A row still being spoken is dim and marked, because it will be
    // replaced: reading a partial as final is how a half sentence gets
    // written down as what somebody said.
    let live = !u.settled && age < LIVE;
    // The model's own verdict on itself: below about -1.0 mean log
    // probability, or a high chance the window was not speech at all. The
    // words are shown either way, because a doubtful reading of a fading
    // handheld is worth more than a blank row, but a reader is told.
    let unsure = !u.credible || u.confidence < -1.0;
    let tint = if live {
        theme::READOUT
    } else if unsure {
        theme::FAULT
    } else {
        theme::VALUE
    };
    let mut text = u.text.clone();
    if live {
        text.push_str(" ...");
    } else if unsure {
        text.push_str("  (unsure)");
    }
    let font = egui::FontId::new(11.0, egui::FontFamily::Name(theme::READOUT_FONT.into()));
    let galley = ui.painter().layout(text, font, tint, text_w - 6.0);
    let h = (galley.size().y + 4.0).max(widgets::ROW_H);
    let (rect, _) = ui.allocate_exact_size(Vec2::new(width, h), Sense::hover());
    if !ui.is_rect_visible(rect) {
        return;
    }
    let p = ui.painter_at(rect);
    if n % 2 == 1 {
        p.rect_filled(rect, 0.0, Color32::from_rgb(0x24, 0x27, 0x2D));
    }
    if live {
        p.rect_filled(
            Rect::from_min_max(rect.left_top(), Pos2::new(rect.left() + 3.0, rect.bottom())),
            0.0,
            theme::READOUT,
        );
    }
    // Wall time from the receiver's clock: the utterance is stamped with an
    // Instant so it can be aged, and the difference is what puts it on a
    // clock somebody can read.
    let when = wall
        .checked_sub(age)
        .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
        .map(|d| crate::sats::utc_hms(d.as_secs() as i64))
        .unwrap_or_default();
    let freq = if u.key.channel_hz > 0 {
        format!("{:.4}", u.key.channel_hz as f64 / 1e6)
    } else {
        String::new()
    };
    let speaker = u.key.from.clone().unwrap_or_default();
    let chan = match (&u.key.to, u.key.system.as_str()) {
        (Some(c), _) => c.clone(),
        (None, "") => String::new(),
        (None, system) => system.to_string(),
    };
    // Cells are drawn on the first line of the row, which is the row's
    // top rather than its middle when the text has wrapped.
    let line = Rect::from_min_size(rect.min, Vec2::new(rect.width(), widgets::ROW_H));
    let mut x = rect.left() + 12.0;
    let cells = [
        (when, theme::LEGEND),
        (freq, theme::VALUE),
        (speaker, theme::VALUE),
        (chan, theme::VALUE),
    ];
    for ((_, w), (t, col)) in COLS.iter().zip(cells) {
        widgets::cell(&p, line, x, *w, &t, col);
        x += w;
    }
    p.galley(Pos2::new(x, rect.top() + 2.0), galley, tint);
}

/// A conversation key, as a person reads it.
pub(super) fn who(key: &common::ConversationKey) -> String {
    let mut out = format!("{}  {:.4} MHz", key.system, key.channel_hz as f64 / 1e6);
    if let Some(c) = &key.to {
        out.push_str(&format!("  {c}"));
    }
    if let Some(from) = &key.from {
        out.push_str(&format!("  < {from}"));
    }
    out
}
