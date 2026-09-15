//! The Agent view: what a model was asked, and what it did about it.
//!
//! A tool call is drawn as a row of its own rather than folded into the
//! model's prose, because what the receiver did is the part an operator has
//! to be able to check: a model that says it tuned to 145.5 and a model that
//! tuned to 145.5 look the same in a sentence.

use super::*;
use crate::agent::channel::{AgentChannel, State};
use crate::agent::chat::{Chat, Turn};

/// The question box: two rows of text, and the margins around them.
const ASK_H: f32 = 76.0;

/// What to print about the speech model, or `None` when there is nothing
/// worth saying: a model that is loaded and idle is not news.
fn speech_line(h: &crate::transcripts::Health) -> Option<(String, egui::Color32)> {
    use crate::transcripts::ModelState;
    match &h.state {
        ModelState::Cold => Some(("not loaded yet".into(), theme::LEGEND)),
        ModelState::Fetching => {
            let what = match h.fetch.fraction() {
                Some(f) => format!("downloading {:.0}%", f * 100.0),
                None => "downloading".to_string(),
            };
            let of = match h.fetch.files {
                0 => String::new(),
                n => format!(" ({} of {n})", h.fetch.files_done + 1),
            };
            Some((format!("{what}{of}"), theme::READOUT))
        }
        ModelState::Loading => Some(("loading".into(), theme::READOUT)),
        ModelState::Failed(e) => Some((e.clone(), theme::FAULT)),
        // Once it has spoken, how long the last reply took to make. A model
        // slower than the over it is answering is the fault that otherwise
        // shows only as an agent that never seems to reply.
        ModelState::Ready if h.reads > 0 => {
            Some((format!("{} in {:.1}s", h.device, h.last_ms as f64 / 1000.0), theme::LEGEND))
        }
        ModelState::Ready => None,
    }
}

/// What the pane wants done that it cannot do itself.
pub(super) enum Action {
    Ask(String),
    Clear,
    Interrupt,
    /// Send an answer over the air again, by its place in the log.
    SayAgain(usize),
    /// Open the settings that say which model this is.
    Settings,
}

pub(super) struct AgentView<'a> {
    pub chat: &'a mut Chat,
    /// What the speech model is doing, which the state alone cannot say: a
    /// download, a load and a card generating all look like "making speech".
    pub voice: crate::transcripts::Health,
    /// Why the agent cannot answer over the air, when it cannot.
    pub air_fault: Option<&'static str>,
    /// The name it answers to.
    pub wake: String,
    /// Whether a radio is running, which is most of what a model can do
    /// anything about.
    pub running: bool,
    /// The agent on the air, when a channel has been given to it.
    pub air: &'a AgentChannel,
}

impl AgentView<'_> {
    pub(super) fn show(mut self, ui: &mut egui::Ui) -> Option<Action> {
        let mut act = None;
        let busy = self.chat.busy();
        let model = self.chat.config.model.clone();
        let fault = self.chat.config.fault();

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            theme::Line::new()
                .legend("model")
                .set(match model.is_empty() {
                    true => "none".to_string(),
                    false => model,
                })
                .size(11.0)
                .show(ui);
            ui.add_space(12.0);
            theme::Line::new()
                .legend("radio")
                .value(if self.running { "running" } else { "stopped" })
                .size(11.0)
                .show(ui);
            if let Some(id) = self.air.on {
                ui.add_space(12.0);
                let (word, tint) = match (self.air.state, self.air_fault) {
                    // Listening and unable to answer is not listening. This
                    // is the state a wake word nobody set leaves it in, and
                    // it looked identical to working.
                    (State::Listening, Some(f)) => (f.to_string(), theme::FAULT),
                    (State::Listening, None) => ("listening".into(), theme::LEGEND),
                    (State::OnAir, _) => ("on air".into(), theme::FAULT),
                    (s, _) => (s.label().into(), theme::READOUT),
                };
                theme::Line::new()
                    .legend(&format!("channel {id}"))
                    .value(word)
                    .size(11.0)
                    .tint(tint)
                    .show(ui);
                // The name it answers to, which has to survive the speech
                // model: an operator saying it and getting nothing needs to
                // see what the receiver is listening for.
                if !self.wake.trim().is_empty() {
                    ui.add_space(12.0);
                    theme::Line::new().legend("name").set(self.wake.trim()).size(11.0).show(ui);
                }
            }
            // The speech model, which is the slow half of an answer and the
            // one with gigabytes to fetch. Only drawn once a channel has been
            // given to the agent, since nothing else here speaks.
            if self.air.on.is_some()
                && let Some((word, tint)) = speech_line(&self.voice)
            {
                ui.add_space(12.0);
                theme::Line::new().legend("voice").value(word).size(11.0).tint(tint).show(ui);
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(12.0);
                if ui.button("Model").clicked() {
                    act = Some(Action::Settings);
                }
                if ui.add_enabled(!self.chat.turns.is_empty(), egui::Button::new("Clear")).clicked()
                {
                    act = Some(Action::Clear);
                }
                if busy && ui.button("Stop").clicked() {
                    act = Some(Action::Interrupt);
                }
            });
        });
        // The last over on the channel and what became of it. Most overs are
        // not for the agent, and an operator who has just spoken needs to see
        // that it was read and how it was read before they can tell whether
        // the name got through.
        if self.air.on.is_some()
            && let Some(h) = &self.air.last
        {
            ui.horizontal(|ui| {
                ui.add_space(12.0);
                let note = match h.passed {
                    None => "answering".to_string(),
                    Some(p) => p.label().to_string(),
                };
                theme::Line::new()
                    .legend("heard")
                    .heard(h.text.clone())
                    .size(11.0)
                    .gap(10.0)
                    .legend(&note)
                    .size(11.0)
                    .elided(ui);
            });
        }
        ui.add_space(6.0);

        if let Some(why) = fault {
            egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
                widgets::card(
                    ui,
                    Some(theme::FAULT),
                    |ui| {
                        theme::Line::new().legend("agent").set(why).tint(theme::FAULT).show(ui);
                    },
                    |ui| {
                        hint(
                            ui,
                            "Model takes the address of anything speaking the OpenAI chat \
                             completions API, the model to ask for, and a key where the \
                             server wants one.",
                        );
                    },
                );
            });
            ui.add_space(6.0);
        }

        // The question is drawn before the conversation is laid out, so the
        // box stays at the foot of the pane while the scroll above it grows.
        let ask = self.ask_box(ui, busy);
        if ask.is_some() {
            act = ask;
        }

        egui::ScrollArea::vertical().stick_to_bottom(true).auto_shrink([false, false]).show(
            ui,
            |ui| {
                egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 4)).show(ui, |ui| {
                    if self.chat.turns.is_empty() {
                        ui.add_space(24.0);
                        ui.vertical_centered(|ui| {
                            hint(
                                ui,
                                "Ask for something. The model has the whole receiver: what it \
                                 tunes, opens or switches on happens in this window, and what \
                                 it reads is what the panes are showing.",
                            );
                        });
                    }
                    for turn in &self.chat.turns {
                        draw(ui, turn);
                        ui.add_space(6.0);
                    }
                    // What was said over the air, under what was typed: one
                    // agent, two ways in, and an operator needs to see what
                    // it has been telling people on the channel.
                    let now = std::time::Instant::now();
                    let quiet = !self.air.state.busy();
                    for (nth, x) in self.air.log.iter().enumerate() {
                        let ago = now.duration_since(x.at).as_secs();
                        theme::Line::new()
                            .legend("heard")
                            .value(match ago {
                                0..=59 => format!("{ago}s ago"),
                                _ => format!("{}m ago", ago / 60),
                            })
                            .size(11.0)
                            .show(ui);
                        theme::Line::new().heard(x.heard.clone()).wrapped(ui);
                        match &x.said {
                            Ok(said) => {
                                // "Say again" is the commonest thing anybody
                                // says on a channel, and the answer is
                                // already made: this sends the same over
                                // rather than asking for another.
                                ui.horizontal(|ui| {
                                    theme::Line::new().legend("said").size(11.0).show(ui);
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            let can = quiet && x.can_repeat();
                                            let b = egui::Button::new("SAY AGAIN").small();
                                            if ui
                                                .add_enabled(can, b)
                                                .on_hover_text(
                                                    "Send this answer over the air again, \
                                                     the same words and the same voice",
                                                )
                                                .on_disabled_hover_text(match x.can_repeat() {
                                                    true => "it is answering something else",
                                                    false => "this one never went out",
                                                })
                                                .clicked()
                                            {
                                                act = Some(Action::SayAgain(nth));
                                            }
                                        },
                                    );
                                });
                                theme::Line::new().words(said.clone()).wrapped(ui);
                            }
                            Err(e) => {
                                theme::Line::new()
                                    .value(e.clone())
                                    .size(11.0)
                                    .tint(theme::FAULT)
                                    .wrapped(ui);
                            }
                        }
                        ui.add_space(6.0);
                    }
                    if busy {
                        ui.horizontal(|ui| {
                            ui.add(egui::Spinner::new().size(12.0));
                            ui.add_space(6.0);
                            theme::Line::new().legend("thinking").size(11.0).show(ui);
                        });
                    }
                });
            },
        );
        act
    }

    /// The box at the foot, and the two ways of sending what is in it.
    fn ask_box(&mut self, ui: &mut egui::Ui, busy: bool) -> Option<Action> {
        let mut act = None;
        let draft = &mut self.chat.draft;
        // Two rows of text and the button beside them. Not resizable: what
        // grows here is the conversation above it.
        Panel::bottom("ask")
            .exact_size(ASK_H)
            .resizable(false)
            .frame(
                egui::Frame::NONE.fill(theme::PANEL).inner_margin(egui::Margin::symmetric(12, 8)),
            )
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    // The box takes what the button leaves. Laid out this way
                    // round because a right-to-left layout inside a row takes
                    // the whole width with it and the box then has none.
                    const SEND_W: f32 = 64.0;
                    let box_w = (ui.available_width() - SEND_W - 8.0).max(120.0);
                    let entry = ui.add_sized(
                        egui::Vec2::new(box_w, ASK_H - 16.0),
                        egui::TextEdit::multiline(draft)
                            .hint_text("What is on 446 MHz?")
                            .desired_rows(2),
                    );
                    let send = ui
                        .add_enabled(
                            !busy,
                            egui::Button::new("Send").min_size(egui::Vec2::new(SEND_W, 28.0)),
                        )
                        .clicked();
                    // Enter sends and shift-enter breaks the line, which is
                    // what every chat box does and what a hand expects.
                    let entered = entry.has_focus()
                        && ui.input(|i| i.key_pressed(egui::Key::Enter) && !i.modifiers.shift);
                    if !busy && (send || entered) {
                        let text = draft.trim().to_string();
                        // The newline that sent it is already in the box.
                        draft.clear();
                        if !text.is_empty() {
                            act = Some(Action::Ask(text));
                        }
                    }
                });
            });
        act
    }
}

/// One turn, drawn by what it is.
fn draw(ui: &mut egui::Ui, turn: &Turn) {
    match turn {
        Turn::You(text) => {
            theme::Line::new().legend("you").size(11.0).show(ui);
            theme::Line::new().set(text.clone()).wrapped(ui);
        }
        Turn::Said(text) => {
            // Named, like the operator's own lines: a conversation where only
            // one side is labelled is one where the reader has to work out
            // who is talking from the typeface.
            theme::Line::new().legend("agent").size(11.0).show(ui);
            // The thing on this pane to read, so it is the brightest and the
            // only prose: the note face, which is proportional, at the size a
            // reading is set in rather than the size a hint is.
            theme::Line::new().note(text.clone()).size(13.0).tint(theme::VALUE).wrapped(ui);
        }
        Turn::Did { name, args, answer } => {
            // A tool call is a margin note about how the answer was arrived
            // at. It was drawn in the caption face lit up, which put the
            // machinery above the words on the page.
            let tint = match answer {
                Some(Err(_)) => theme::FAULT,
                _ => theme::LEGEND,
            };
            let mark = match answer {
                None => "running",
                Some(Ok(_)) => "",
                Some(Err(_)) => "refused",
            };
            ui.horizontal(|ui| {
                // Indented off the prose: this is the margin, not the page.
                ui.add_space(10.0);
                let mut line = theme::Line::new().legend(name).size(10.5).tint(tint);
                if !args.is_empty() && args != "{}" {
                    // Arguments as they were written, not shouted: a
                    // frequency in caps is a frequency misread.
                    line = line.gap(6.0).note(short(args)).size(10.5).tint(tint);
                }
                if !mark.is_empty() {
                    line = line.gap(6.0).legend(mark).size(10.5).tint(tint);
                }
                line.elided(ui);
            });
            if let Some(Err(e)) = answer {
                ui.horizontal(|ui| {
                    ui.add_space(10.0);
                    theme::Line::new().note(e.clone()).size(11.5).tint(theme::FAULT).wrapped(ui);
                });
            }
        }
        Turn::Fault(e) => {
            theme::Line::new().value(e.clone()).tint(theme::FAULT).wrapped(ui);
        }
    }
}

/// Arguments as one short line. A model that passes a whole patch would
/// otherwise push every reading off the row.
fn short(args: &str) -> String {
    let one: String = args.split_whitespace().collect::<Vec<_>>().join(" ");
    let one = one.trim_matches(|c| c == '{' || c == '}');
    match one.chars().count() > 80 {
        true => format!("{}…", one.chars().take(80).collect::<String>()),
        false => one.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn long_arguments_are_cut_and_short_ones_are_not() {
        assert_eq!(short("{\"mhz\": 145.5}"), "\"mhz\": 145.5");
        let long = format!("{{\"text\":\"{}\"}}", "x".repeat(200));
        assert!(short(&long).ends_with('…'));
        assert!(short(&long).chars().count() <= 81);
    }
}
