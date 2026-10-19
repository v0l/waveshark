//! What the Agent view shows beside the conversation: the agent on the air.
//!
//! The conversation itself, its input box and its tool rows are drawn by
//! `egui_bench::agent`. What stays here is what only a radio has: whether the
//! receiver is running, the channel the agent answers on, its name, the state
//! of its voice, and one card for every over it has answered.

use super::*;
use crate::agent::channel::{AgentChannel, State};

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

pub(super) fn status(
    ui: &mut egui::Ui,
    running: bool,
    air: &AgentChannel,
    voice: &crate::transcripts::Health,
    air_fault: Option<&'static str>,
    wake: &str,
) {
    ui.horizontal_wrapped(|ui| {
        Line::new()
            .legend("radio")
            .value(if running { "running" } else { "stopped" })
            .size(11.0)
            .show(ui);
        if let Some(id) = air.on {
            ui.add_space(12.0);
            let (word, tint) = match (air.state, air_fault) {
                (State::Listening, Some(f)) => (f.to_string(), theme::FAULT),
                (State::Listening, None) => ("listening".into(), theme::LEGEND),
                (State::OnAir, _) => ("on air".into(), theme::FAULT),
                (s, _) => (s.label().into(), theme::READOUT),
            };
            Line::new().legend(&format!("channel {id}")).value(word).size(11.0).tint(tint).show(ui);
            if !wake.trim().is_empty() {
                ui.add_space(12.0);
                Line::new().legend("name").set(wake.trim()).size(11.0).show(ui);
            }
        }
        if air.on.is_some()
            && let Some((word, tint, detail)) = speech_line(voice)
        {
            ui.add_space(12.0);
            let r = Line::new().legend("voice").value(word).size(11.0).tint(tint).elided(ui);
            if let Some(d) = detail {
                r.on_hover_text(d);
            }
        }
    });
}

pub(super) fn heard(ui: &mut egui::Ui, air: &AgentChannel, config: &crate::agent::config::Config) {
    if air.on.is_some()
        && let Some(h) = &air.last
    {
        ui.horizontal(|ui| {
            let note = match h.passed {
                None => "answering".to_string(),
                Some(p) => p.label().to_string(),
            };
            let mut line = Line::new().legend("heard");
            if let Some(from) = &h.from {
                line = line.value(from.clone()).size(11.0);
            }
            line.measured(h.text.clone()).size(11.0).gap(10.0).legend(&note).size(11.0).elided(ui);
        });
    }
    if air.on.is_some()
        && let Some(left) = air.following(config, common::time::Instant::now())
    {
        ui.horizontal(|ui| {
            Line::new()
                .legend("open")
                .value(format!("no name needed for {left:.0} s"))
                .size(11.0)
                .show(ui);
        });
    }
}

pub(super) fn on_the_air(ui: &mut egui::Ui, air: &AgentChannel) -> Option<usize> {
    let mut again = None;
    let now = common::time::Instant::now();
    let quiet = !air.state.busy();
    if air.log.is_empty() {
        return None;
    }
    ui.add_space(6.0);
    Line::new().legend("on the air").size(11.0).show(ui);
    ui.add_space(4.0);
    for (nth, x) in air.log.iter().enumerate() {
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
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let can = quiet && x.can_repeat();
                    let b = egui::Button::new("SAY AGAIN").small();
                    if ui
                        .add_enabled(can, b)
                        .on_hover_text(
                            "Send this answer over the air again, the same words and the same voice",
                        )
                        .on_disabled_hover_text(match x.can_repeat() {
                            true => "it is answering something else",
                            false => "this one never went out",
                        })
                        .clicked()
                    {
                        again = Some(nth);
                    }
                });
            },
            |ui| {
                Line::new().measured(x.heard.clone()).wrapped(ui);
                match &x.said {
                    Ok(said) => {
                        ui.add_space(2.0);
                        Line::new().legend("said").size(11.0).show(ui);
                        Line::new().note(said.clone()).size(13.0).tint(theme::VALUE).wrapped(ui);
                    }
                    Err(e) => {
                        ui.add_space(2.0);
                        ui.horizontal(|ui| {
                            let (rect, _) = ui.allocate_exact_size(
                                egui::Vec2::new(10.0, 18.0),
                                egui::Sense::hover(),
                            );
                            ui.painter().circle_filled(rect.center(), 3.0, theme::FAULT);
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
    again
}

#[cfg(not(feature = "agent"))]
pub(super) fn unavailable(ui: &mut egui::Ui) {
    Line::new().legend("agent").size(12.0).show(ui);
    ui.add_space(6.0);
    hint(
        ui,
        "The chat is part of the desktop app. This build was made without it, and the browser \
         cannot run it. The agent on the air still answers when a channel is given to it.",
    );
    ui.add_space(10.0);
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
