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
use crate::transcripts::{Engine, ModelState, Speaker, Utterance, LIVE};

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
        // a partial that grows is the same line getting longer.
        egui::ScrollArea::vertical().stick_to_bottom(true).auto_shrink([false, false]).show(
            ui,
            |ui| {
                ui.spacing_mut().item_spacing.y = 4.0;
                egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
                    let mut last: Option<&str> = None;
                    for u in &shown {
                        // The speaker is written once above a run of lines
                        // from the same conversation. Repeated over every
                        // line it is most of the pane, and what somebody is
                        // reading here is the words.
                        let same = last == Some(u.key.as_str());
                        line(ui, u, now, !same);
                        last = Some(u.key.as_str());
                    }
                });
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
        match self.engine.as_ref().map(|e| (&e.state, e.enabled)) {
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
        let rail = match e.state {
            ModelState::Ready => theme::TRACE,
            ModelState::Failed(_) => theme::FAULT,
            _ => theme::READOUT,
        };
        // The header and the body of a card are drawn by two closures at
        // once, so what they ask for is collected rather than pushed.
        let asked = self.st.asked;
        let want: std::cell::RefCell<(Option<bool>, bool)> = std::cell::RefCell::new((None, false));
        egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
            widgets::card(
                ui,
                Some(rail),
                |ui| {
                    // What is on disc is what runs, so that is what is
                    // named. The repository is only where files would be
                    // fetched from, and naming it while a different model
                    // sits in the directory is the card telling a lie about
                    // the one thing it exists to say.
                    let mut head = theme::Line::new().legend("model").set(match e.present {
                        true => local_name(&e.dir),
                        false => e.repo.clone(),
                    });
                    head = head.legend("state").value(e.state.label()).tint(rail);
                    if !e.device.is_empty() {
                        head = head.legend("on").value(&e.device);
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
                    // Wrapped, because the path is as long as somebody's
                    // home directory makes it and a clipped path is a path
                    // that cannot be checked.
                    let where_ = if e.dir.is_empty() { "unset" } else { e.dir.as_str() };
                    let mut l = theme::Line::new().legend("files").set(where_);
                    l = if e.present {
                        // What is on disc, not what would be fetched. A
                        // directory filled by an earlier run, or by hand,
                        // holds a different model from the one named above,
                        // and the one on disc is the one that runs.
                        l.legend("on disc")
                            .value(super::human_bytes(e.bytes))
                            .value(&e.weights)
                            .value(&e.flavour)
                    } else {
                        l.legend("to fetch")
                            .set(&e.repo)
                            .legend("on disc")
                            .value("nothing")
                            .tint(theme::READOUT)
                    };
                    l.size(11.0).wrapped(ui);
                    ui.horizontal(|ui| {
                        let mut l = theme::Line::new().legend("read").value(e.reads.to_string());
                        if let Some(x) = e.speed() {
                            // Against real time, because that is the number
                            // that decides whether the receiver keeps up:
                            // below one, speech arrives faster than it can
                            // be read and the partials fall behind.
                            l = l
                                .legend("last")
                                .value(format!("{:.1} s in {} ms", e.last_audio_s, e.last_ms))
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
                    if let ModelState::Failed(why) = &e.state {
                        theme::Line::new().words(why).tint(theme::FAULT).wrapped(ui);
                    }
                    // What the last window came back as, whether or not it
                    // became a line. A card saying "read 8" over an empty
                    // pane is a receiver that looks broken; the same card
                    // saying the eight reads came back empty and the model
                    // heard no speech in them is a receiver being handed
                    // silence, which is a different problem in a different
                    // place.
                    if e.reads > 0 {
                        let (text, tint) = match (e.last_text.is_empty(), e.last_speech) {
                            (true, _) => ("(nothing)".to_string(), theme::LEGEND),
                            (false, true) => (e.last_text.clone(), theme::VALUE),
                            (false, false) => (e.last_text.clone(), theme::FAULT),
                        };
                        let mut l = theme::Line::new().legend("last read").words(text).tint(tint);
                        if !e.last_speech {
                            l = l.legend("no speech").tint(theme::FAULT);
                        }
                        l.size(11.0).wrapped(ui);
                    }
                    // Loading it by hand is the only way to find out whether
                    // transcription works on this machine without waiting
                    // for somebody to key up: the model is fetched and
                    // loaded by the first speech worth reading, and that may
                    // be an hour away.
                    let cold = matches!(e.state, ModelState::Cold | ModelState::Failed(_));
                    if cold {
                        ui.horizontal(|ui| {
                            let label = if e.present {
                                "Load the model now"
                            } else {
                                "Download and load the model"
                            };
                            if ui.add_enabled(!asked, egui::Button::new(label)).clicked() {
                                want.borrow_mut().1 = true;
                            }
                            if !e.present {
                                hint(ui, "Tens of megabytes, over the network, once.");
                            }
                        });
                    }
                },
            );
        });
        let (enabled, load) = want.into_inner();
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
        if !matches!(e.state, ModelState::Cold | ModelState::Failed(_)) {
            self.st.asked = false;
        }
    }
}

/// One line of the log: who said it when the speaker changed, the words, and
/// how well they were read.
fn line(ui: &mut egui::Ui, u: &Utterance, now: std::time::Instant, name: bool) {
    if name {
        ui.add_space(4.0);
        theme::Line::new().legend("from").set(who(&u.key)).size(11.0).show(ui);
    }
    let age = now.saturating_duration_since(u.at);
    // A line still being spoken is dim and marked, because it will be
    // replaced: reading a partial as final is how a half sentence gets
    // written down as what somebody said.
    let live = !u.settled && age < LIVE;
    let mut l = theme::Line::new().legend(&when(age)).words(&u.text).tint(if live {
        theme::READOUT
    } else {
        theme::VALUE
    });
    if live {
        l = l.legend("...");
    }
    // The model's own verdict on itself: below about -1.0 mean log
    // probability, or a high chance the window was not speech at all. The
    // words are shown either way, because a doubtful reading of a fading
    // handheld is worth more than a blank pane, but a reader is told rather
    // than left to trust it.
    if !u.credible || u.confidence < -1.0 {
        l = l.legend("unsure").tint(theme::FAULT);
    }
    l.wrapped(ui);
}

/// What the model in a directory is called, which is the directory's own
/// name: the files carry no name of their own and the repository they came
/// from is not recorded beside them.
fn local_name(dir: &str) -> String {
    std::path::Path::new(dir)
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| dir.to_string())
}

/// A conversation key, as a person reads it.
pub(super) fn who(key: &str) -> String {
    let Some(s) = Speaker::parse(key) else {
        return key.to_string();
    };
    let mut out = format!("{}  {:.4} MHz", s.proto, s.freq_hz as f64 / 1e6);
    if let Some(c) = &s.channel {
        out.push_str(&format!("  {c}"));
    }
    if let Some(from) = &s.speaker {
        out.push_str(&format!("  < {from}"));
    }
    out
}

/// How long ago, short enough for the head of a line.
fn when(d: std::time::Duration) -> String {
    let s = d.as_secs();
    match s {
        0..=59 => format!("{s}s"),
        60..=3599 => format!("{}m", s / 60),
        _ => format!("{}h", s / 3600),
    }
}
