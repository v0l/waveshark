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
///
/// A fault is one word here and the whole message on hover: the message
/// is a server's JSON and as long as it likes, and on this line it pushed
/// the buttons off the edge of the window.
fn speech_line(h: &crate::transcripts::Health) -> Option<(String, egui::Color32, Option<String>)> {
    use crate::transcripts::ModelState;
    let plain = |w: &str, c| Some((w.to_string(), c, None));
    match &h.state {
        ModelState::Cold => plain("not loaded yet", theme::LEGEND),
        ModelState::Fetching => {
            let what = match h.fetch.fraction() {
                Some(f) => format!("downloading {:.0}%", f * 100.0),
                None => "downloading".to_string(),
            };
            let of = match h.fetch.files {
                0 => String::new(),
                n => format!(" ({} of {n})", h.fetch.files_done + 1),
            };
            Some((format!("{what}{of}"), theme::READOUT, None))
        }
        ModelState::Loading => plain("loading", theme::READOUT),
        ModelState::Failed(e) => Some(("failed".into(), theme::FAULT, Some(e.clone()))),
        // Once it has spoken, how long the last reply took to make. A model
        // slower than the over it is answering is the fault that otherwise
        // shows only as an agent that never seems to reply.
        ModelState::Ready if h.reads > 0 => Some((
            format!("{} in {:.1}s", h.device, h.last_ms as f64 / 1000.0),
            theme::LEGEND,
            None,
        )),
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
        // The buttons take their width off the right first, so nothing the
        // status line says can push them off the edge of the window.
        ui.horizontal(|ui| {
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
                ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                    self.status_line(ui, model.clone());
                });
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
                let mut line = Line::new().legend("heard");
                // Who said it, where the radio said so: an analogue PTT-ID
                // or a decoded call's caller. The model is told the same.
                if let Some(from) = &h.from {
                    line = line.value(from.clone()).size(11.0);
                }
                line.measured(h.text.clone())
                    .size(11.0)
                    .gap(10.0)
                    .legend(&note)
                    .size(11.0)
                    .elided(ui);
            });
        }
        // Whether the next over has to say the name. An operator who has just
        // been answered can carry on talking, and nothing else on the screen
        // says for how long.
        if self.air.on.is_some()
            && let Some(left) = self.air.following(&self.chat.config, std::time::Instant::now())
        {
            ui.horizontal(|ui| {
                ui.add_space(12.0);
                Line::new()
                    .legend("open")
                    .value(format!("no name needed for {left:.0} s"))
                    .size(11.0)
                    .show(ui);
            });
        }
        ui.add_space(6.0);

        if let Some(why) = fault {
            egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
                panel::card(
                    ui,
                    Some(theme::FAULT),
                    |ui| {
                        Line::new().legend("agent").set(why).tint(theme::FAULT).show(ui);
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
                    // it has been telling people on the channel. One card
                    // per over, in the shape a call has everywhere else:
                    // what was heard, what was answered, or why not.
                    let now = std::time::Instant::now();
                    let quiet = !self.air.state.busy();
                    if !self.air.log.is_empty() && !self.chat.turns.is_empty() {
                        ui.add_space(6.0);
                        Line::new().legend("on the air").size(11.0).show(ui);
                        ui.add_space(4.0);
                    }
                    for (nth, x) in self.air.log.iter().enumerate() {
                        let ago = now.duration_since(x.at).as_secs();
                        let when = match ago {
                            0..=59 => format!("{ago}s ago"),
                            _ => format!("{}m ago", ago / 60),
                        };
                        let rail = match &x.said {
                            Ok(_) => Some(theme::TRACE),
                            Err(_) => Some(theme::FAULT),
                        };
                        panel::card(
                            ui,
                            rail,
                            |ui| {
                                Line::new().legend("heard").value(when).size(11.0).show(ui);
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        // "Say again" is the commonest thing
                                        // anybody says on a channel, and the
                                        // answer is already made: this sends
                                        // the same over rather than asking
                                        // for another.
                                        let can = quiet && x.can_repeat();
                                        let b = egui::Button::new("SAY AGAIN").small();
                                        if ui
                                            .add_enabled(can, b)
                                            .on_hover_text(
                                                "Send this answer over the air again, the \
                                                 same words and the same voice",
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
                            },
                            |ui| {
                                Line::new().measured(x.heard.clone()).wrapped(ui);
                                match &x.said {
                                    Ok(said) => {
                                        ui.add_space(2.0);
                                        Line::new().legend("said").size(11.0).show(ui);
                                        Line::new()
                                            .note(said.clone())
                                            .size(13.0)
                                            .tint(theme::VALUE)
                                            .wrapped(ui);
                                    }
                                    // One line of the fault, the whole of it
                                    // on hover: a server's error is a page
                                    // of JSON and this is a log, not a
                                    // debugger.
                                    Err(e) => {
                                        ui.add_space(2.0);
                                        ui.horizontal(|ui| {
                                            let (rect, _) = ui.allocate_exact_size(
                                                egui::Vec2::new(10.0, 18.0),
                                                egui::Sense::hover(),
                                            );
                                            let c = rect.center();
                                            ui.painter().circle_filled(c, 3.0, theme::FAULT);
                                            Line::new()
                                                .value(short_fault(e))
                                                .tint(theme::FAULT)
                                                .size(11.0)
                                                .elided(ui)
                                                .on_hover_text(e);
                                        });
                                    }
                                }
                            },
                        );
                        ui.add_space(6.0);
                    }
                    if busy {
                        ui.horizontal(|ui| {
                            ui.add(egui::Spinner::new().size(12.0));
                            ui.add_space(6.0);
                            Line::new().legend("thinking").size(11.0).show(ui);
                        });
                    }
                });
            },
        );
        act
    }

    /// Where the agent is: the model, the radio, the channel it answers on,
    /// its name, and the state of its voice.
    fn status_line(&self, ui: &mut egui::Ui, model: String) {
        ui.add_space(12.0);
        Line::new()
            .legend("model")
            .set(match model.is_empty() {
                true => "none".to_string(),
                false => model,
            })
            .size(11.0)
            .show(ui);
        ui.add_space(12.0);
        Line::new()
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
            Line::new().legend(&format!("channel {id}")).value(word).size(11.0).tint(tint).show(ui);
            // The name it answers to, which has to survive the speech
            // model: an operator saying it and getting nothing needs to
            // see what the receiver is listening for.
            if !self.wake.trim().is_empty() {
                ui.add_space(12.0);
                Line::new().legend("name").set(self.wake.trim()).size(11.0).show(ui);
            }
        }
        // The speech model, which is the slow half of an answer and the
        // one with gigabytes to fetch. Only drawn once a channel has been
        // given to the agent, since nothing else here speaks.
        if self.air.on.is_some()
            && let Some((word, tint, detail)) = speech_line(&self.voice)
        {
            ui.add_space(12.0);
            let r = Line::new().legend("voice").value(word).size(11.0).tint(tint).elided(ui);
            if let Some(d) = detail {
                r.on_hover_text(d);
            }
        }
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
            Line::new().legend("you").size(11.0).show(ui);
            Line::new().set(text.clone()).wrapped(ui);
        }
        Turn::Said(text) => {
            // Named, like the operator's own lines: a conversation where only
            // one side is labelled is one where the reader has to work out
            // who is talking from the typeface.
            Line::new().legend("agent").size(11.0).show(ui);
            // The thing on this pane to read, so it is the brightest and the
            // only prose: the note face, which is proportional, at the size a
            // reading is set in rather than the size a hint is.
            Line::new().note(text.clone()).size(13.0).tint(theme::VALUE).wrapped(ui);
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
                let mut line = Line::new().legend(name).size(10.5).tint(tint);
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
                    Line::new().note(e.clone()).size(11.5).tint(theme::FAULT).wrapped(ui);
                });
            }
        }
        Turn::Fault(e) => {
            Line::new().value(e.clone()).tint(theme::FAULT).wrapped(ui);
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

/// The first sentence of a fault, for a line in a log. A gateway's answer
/// wraps a provider's answer wraps a JSON body, and the first clause is
/// the one that says what happened.
fn short_fault(e: &str) -> String {
    let first = e.lines().next().unwrap_or(e).trim();
    let cut = first.find(": {").map(|i| &first[..i]).unwrap_or(first);
    match cut.char_indices().nth(120) {
        Some((i, _)) => format!("{}…", &cut[..i]),
        None => cut.to_string(),
    }
}
