//! The settings dialogs, and the one that creates a remote radio.
//!
//! Each is a leaf: it reads and writes the receiver's state and draws nothing
//! anybody else depends on, which is what makes them separable from the panes.

use super::*;
use crate::agent::config::{Reading, Speech};
use crate::ui::widgets::{
    card, choice, field, field_then, footer, lamp, prose, secret, section, switch,
};

/// What the lamp says for a voice made here: the speaker, and whether its
/// files are on disc or have still to be fetched.
#[cfg(feature = "tts")]
fn local_voice_lamp(c: &crate::agent::config::Config) -> String {
    let name = match c.voice_local.trim() {
        "" => tts::DEFAULT_VOICE,
        n => n,
    };
    let dir = match c.voice_dir.trim() {
        "" => tts::default_dir(),
        d => std::path::PathBuf::from(d),
    };
    match tts::Files::in_dir(&dir, name) {
        Ok(f) => format!("{}, {:.0} MB on disc", tts::label_of(name), f.bytes() as f64 / 1e6),
        Err(_) => format!("{}, 330 MB to fetch on the first over", tts::label_of(name)),
    }
}

#[cfg(not(feature = "tts"))]
fn local_voice_lamp(_: &crate::agent::config::Config) -> String {
    "this build has no speech model".to_string()
}

/// Where the speech model's files go, for the hint beside the field.
fn voice_dir_hint() -> String {
    #[cfg(feature = "tts")]
    {
        tts::default_dir().display().to_string()
    }
    #[cfg(not(feature = "tts"))]
    {
        "this build has no speech model".to_string()
    }
}

/// Every speaker that can be picked: the catalogue with the publisher's
/// grade, and a mark against the ones already fetched. A closed list for the
/// same reason the transcriber has one, and English only, because the
/// dictionary that turns words into phonemes here is English.
#[cfg(feature = "tts")]
fn voices(dir: &str) -> Vec<(String, String)> {
    let root = match dir.trim() {
        "" => tts::default_dir(),
        d => std::path::PathBuf::from(d),
    };
    let here = tts::installed(&root);
    let mut out: Vec<(String, String)> = tts::VOICES
        .iter()
        .map(|v| {
            let mark = match here.iter().any(|h| h == v.id) {
                true => " (on disc)",
                false => "",
            };
            (v.id.to_string(), format!("{}{mark}", tts::label_of(v.id)))
        })
        .collect();
    for id in here {
        if tts::voice(&id).is_none() {
            out.push((id.clone(), format!("{id} (on disc)")));
        }
    }
    out
}

impl App {
    pub(super) fn settings_modal(&mut self, ctx: &egui::Context) {
        let Some(which) = self.open else { return };
        let title = match which {
            Settings::Spectrum => "Spectrum",
            Settings::Waterfall => "Waterfall",
            Settings::Radio => "Radio",
            Settings::PacketLog => "Packet log",
            Settings::Scanners => "Scanners",
            Settings::Memory => "Memory bank",
            Settings::Data => crate::i18n::t("settings.data"),
            Settings::Agent => "Agent",
            Settings::App => crate::i18n::t("settings.title"),
        };
        let r = egui::containers::Modal::new(egui::Id::new(title))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(match which {
                    Settings::Scanners | Settings::Memory | Settings::Data => 560.0,
                    _ => 520.0,
                });
                modal_title(ui, title);
                match which {
                    Settings::Spectrum => self.scope_settings(ui, true),
                    Settings::Waterfall => self.scope_settings(ui, false),
                    Settings::Radio => self.radio_settings(ui),
                    Settings::PacketLog => self.packet_log_settings(ui),
                    Settings::Scanners => self.scanner_settings(ui),
                    Settings::Memory => self.memory_pane(ui),
                    Settings::Data => self.data_settings(ui),
                    Settings::Agent => self.agent_settings(ui),
                    Settings::App => self.app_settings(ui),
                }
                footer(ui, |ui| {
                    if ui.button(crate::i18n::t("ui.close")).clicked() {
                        self.open = None;
                    }
                });
            });
        if r.should_close() {
            self.open = None;
        }
    }

    /// The scanner table: which front end runs on which frequency.
    ///
    /// A block per scanner rather than a text box over the file. The file is
    /// still the format of record and is still worth hand-editing, but the
    /// question this pane answers is "why is nothing decoding here", and the
    /// answer is a frequency compared against a list of ranges. That is a
    /// thing to show, not a thing to make somebody read.
    /// The memory bank: what was saved, by group, and a way back to it.
    fn memory_pane(&mut self, ui: &mut egui::Ui) {
        let mut recall: Option<crate::memory::Saved> = None;
        let mut remove: Option<usize> = None;
        let groups: Vec<String> = self.memory.groups().iter().map(|g| g.to_string()).collect();
        if groups.is_empty() {
            section(ui, "channels", "", |ui| {
                lamp(ui, true, "nothing saved yet: SAVE on a strip channel puts it here");
            });
        }
        // The scroll area is told the modal's width: inside it the width
        // on offer is the screen's, and a card that fills it fills that.
        let w = ui.available_width();
        egui::ScrollArea::vertical().max_height(460.0).show(ui, |ui| {
            ui.set_max_width(w);
            for g in &groups {
                let rows: Vec<(usize, crate::memory::Saved)> =
                    self.memory.in_group(g).map(|(i, s)| (i, s.clone())).collect();
                section(ui, g, &format!("{} saved", rows.len()), |ui| {
                    for (i, s) in rows {
                        ui.horizontal(|ui| {
                            let reach = (s.freq - self.center).abs() <= self.rate / 2.0;
                            let mut line = theme::Line::new()
                                .set(format!("{:.4}", s.freq / 1e6))
                                .gap(10.0)
                                .legend(&s.mode.label());
                            if let Some(bw) = s.bandwidth_hz {
                                line = line
                                    .gap(10.0)
                                    .value(format!("{} kHz", crate::scanners::num(bw / 1e3)));
                            }
                            if !s.label.is_empty() {
                                line = line.gap(12.0).words(&s.label);
                            }
                            line.show(ui);
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    if ui
                                        .small_button("×")
                                        .on_hover_text("forget this channel")
                                        .clicked()
                                    {
                                        remove = Some(i);
                                    }
                                    let tip = if reach {
                                        "put this channel on the strip"
                                    } else {
                                        "tune to this channel and put it on the strip"
                                    };
                                    if ui.small_button("RECALL").on_hover_text(tip).clicked() {
                                        recall = Some(s.clone());
                                    }
                                },
                            );
                        });
                    }
                });
                ui.add_space(6.0);
            }
        });
        if let Some(i) = remove {
            self.memory.remove(i);
            let _ = self.memory.save();
        }
        if let Some(s) = recall {
            self.recall(&s);
            self.open = None;
        }
        if let Some(p) = crate::memory::Memory::path() {
            theme::Line::new().note(p.display().to_string()).size(10.0).elided(ui);
        }
    }

    /// The scope's own panels, and whatever they asked for afterwards.
    fn scope_settings(&mut self, ui: &mut egui::Ui, spectrum: bool) {
        let mut pane = scope_settings::ScopeSettings {
            st: &mut self.scope,
            dc_block: &mut self.dc_block,
            rate: self.rate,
            cmds: &mut self.cmds,
            acts: Vec::new(),
        };
        if spectrum {
            pane.spectrum(ui);
        } else {
            pane.waterfall(ui);
        }
        let acts = pane.acts;
        for a in acts {
            match a {
                scope_settings::Action::ResetWaterfall => self.reset_waterfall(),
            }
        }
    }

    fn scanner_settings(&mut self, ui: &mut egui::Ui) {
        let (center, rate) = (self.center, self.rate);
        // Taken out of `self` so the closures below can borrow the rest of
        // it, and put back at the end.
        let mut rows = self
            .scanner_edit
            .take()
            .unwrap_or_else(|| self.scanners.list.iter().map(ScannerRow::from_scanner).collect());

        let live: Vec<crate::scanners::Scanner> =
            rows.iter().filter_map(ScannerRow::to_scanner).collect();
        let table = crate::scanners::Scanners { list: live, version: crate::scanners::VERSION };
        let active: Vec<String> =
            table.active(center, rate).into_iter().map(|s| s.name.clone()).collect();

        // What the table does here, before the table: the question this
        // pane answers is "why is nothing decoding here".
        section(ui, "here", "what the table runs on the span in front of you", |ui| {
            theme::Line::new()
                .legend("tuned to")
                .value(format!("{:.4} MHz", center / 1e6))
                .size(12.0)
                .gap(16.0)
                .legend("span")
                .value(format!("{:.0} kHz", rate / 1e3))
                .size(12.0)
                .show(ui);
            match active.is_empty() {
                false => lamp(ui, true, &format!("running {}", active.join(", "))),
                true => lamp(
                    ui,
                    false,
                    "no block covers this frequency and span: add one, or widen a range",
                ),
            }
        });
        ui.add_space(8.0);

        let mut remove = None;
        let mut tune_to = None;
        let w = ui.available_width();
        egui::ScrollArea::vertical().max_height(360.0).id_salt("scanrows").show(ui, |ui| {
            ui.set_max_width(w);
            for (i, r) in rows.iter_mut().enumerate() {
                let on = active.iter().any(|n| n == &r.name);
                let here = r.regions.is_empty() || r.regions.contains(&crate::bands::plan());
                // The rail says what the span is doing with the block: cyan
                // for one running here, red for another region's, nothing
                // for one the span does not cover.
                let rail = match (on, here, r.enabled) {
                    (true, _, _) => Some(theme::TRACE),
                    (_, false, _) => Some(theme::FAULT),
                    (_, _, false) => Some(theme::ETCH),
                    _ => None,
                };
                let bad = r.to_scanner().is_none();
                let current_banks = r.banks_with_current_widths();
                let middle = (r.lo_mhz + r.hi_mhz) / 2.0;
                // The header and the body each take their own fields, since
                // both are drawn from one row at once.
                let ScannerRow {
                    name,
                    lo_mhz,
                    hi_mhz,
                    span_khz,
                    margin_khz,
                    front,
                    channels,
                    widths,
                    regions,
                    enabled,
                } = r;
                card(
                    ui,
                    rail,
                    |ui| {
                        // The switch that keeps a block in the table without
                        // running it, so turning auto off after pinning a
                        // few channels does not throw it away.
                        ui.checkbox(enabled, "").on_hover_text(if *enabled {
                            "running: click to switch off"
                        } else {
                            "off: click to run"
                        });
                        ui.add(
                            egui::TextEdit::singleline(name)
                                .frame(egui::Frame::NONE)
                                .font(egui::FontId::new(
                                    11.5,
                                    egui::FontFamily::Name(theme::LEGEND_FONT.into()),
                                ))
                                .text_color(theme::VALUE)
                                .desired_width(120.0)
                                .hint_text("name"),
                        );
                        // A block that is somebody else's allocation says so,
                        // and says it in the red it is not running in.
                        if !regions.is_empty() {
                            let names: Vec<&str> = regions.iter().map(|p| p.id()).collect();
                            let mut line = theme::Line::new().legend("region").value(names.join(", "));
                            if !here {
                                line = line.tint(theme::FAULT);
                            }
                            line.size(11.0).show(ui).on_hover_text(match here {
                                true => "this block is for the region in the settings",
                                false => "another region's allocation, so it does not run here. \
                                          Drop the region line in the scanners file to run it anyway",
                            });
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("REMOVE").clicked() {
                                remove = Some(i);
                            }
                            if ui.add_enabled(!on, egui::Button::new("TUNE")).clicked() {
                                tune_to = Some(middle);
                            }
                        });
                    },
                    |ui| {
                        row(ui, "front", |ui| {
                            egui::ComboBox::from_id_salt(("front", i))
                                .selected_text(front.label())
                                .width(120.0)
                                .show_ui(ui, |ui| {
                                    for f in crate::scanners::Front::all() {
                                        let label = f.label();
                                        // Keep the widths already typed
                                        // when switching back to banks.
                                        let pick = if matches!(f, crate::scanners::Front::Banks(_)) {
                                            current_banks.clone()
                                        } else {
                                            f
                                        };
                                        if ui
                                            .selectable_label(front.key() == pick.key(), label)
                                            .clicked()
                                        {
                                            *front = pick;
                                        }
                                    }
                                });
                        });
                        row(ui, "range", |ui| {
                            mhz_field(ui, lo_mhz);
                            theme::Line::new().legend("to").show(ui);
                            mhz_field(ui, hi_mhz);
                            theme::Line::new().legend("MHz").show(ui);
                            ui.add_space(8.0);
                            theme::Line::new().legend("span").show(ui);
                            ui.add(
                                egui::DragValue::new(span_khz)
                                    .speed(10.0)
                                    .range(1.0..=20_000.0)
                                    .suffix(" kHz"),
                            );
                        });
                        match front {
                            // A bank front end is defined by its channel
                            // widths; everything else by the channels that
                            // have to be inside the span.
                            crate::scanners::Front::Banks(_) => {
                                row(ui, "widths", |ui| {
                                    field_then(ui, widths, "31.25, 125", 40.0, |ui| {
                                        theme::Line::new().legend("kHz").show(ui);
                                    });
                                });
                            }
                            _ => {
                                row(ui, "channels", |ui| {
                                    // Not an example of a value: a hint
                                    // that looks like data reads as data
                                    // on a row that needs none.
                                    field_then(ui, channels, "none needed", 190.0, |ui| {
                                        theme::Line::new().legend("MHz").show(ui);
                                        ui.add_space(6.0);
                                        theme::Line::new().legend("margin").show(ui);
                                        ui.add(
                                            egui::DragValue::new(margin_khz)
                                                .speed(1.0)
                                                .range(0.0..=1000.0)
                                                .suffix(" kHz"),
                                        );
                                    });
                                });
                            }
                        }
                        if bad {
                            lamp(ui, false, "needs a name and a range that goes upwards");
                        }
                    },
                );
                ui.add_space(6.0);
            }
        });

        if let Some(i) = remove {
            rows.remove(i);
        }
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            if ui.button("ADD").clicked() {
                // Starts on the frequency being looked at, since wanting a
                // scanner here is why the pane is open.
                rows.push(ScannerRow::new_at(center, rate));
            }
            if ui.button("DEFAULTS").clicked() {
                rows = crate::scanners::Scanners::default()
                    .list
                    .iter()
                    .map(ScannerRow::from_scanner)
                    .collect();
            }
            let dirty = table != self.scanners;
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // Saving writes the file and hands the table to the radio
                // thread, which rebuilds: a change to what runs on this
                // frequency has to take effect without a retune. Saved
                // either way, since the table is configuration. It only
                // reaches the graph when the graph is the table's to build.
                if ui.add_enabled(dirty, egui::Button::new("SAVE")).clicked() {
                    let _ = table.save();
                    self.scanners = table.clone();
                    self.send(Cmd::Scanners(table.clone()));
                }
                if self.chain.edit.manual {
                    theme::Line::new().note(crate::i18n::t("ui.manual_locked")).size(11.0).show(ui);
                }
                if ui.add_enabled(dirty, egui::Button::new("REVERT")).clicked() {
                    rows = self.scanners.list.iter().map(ScannerRow::from_scanner).collect();
                }
                if dirty {
                    theme::Line::new().set("unsaved").size(11.0).show(ui);
                }
            });
        });
        if let Some(p) = crate::scanners::Scanners::path() {
            theme::Line::new().note(p.display().to_string()).size(10.0).elided(ui);
        }

        self.scanner_edit = Some(rows);
        if let Some(mhz) = tune_to {
            self.retune(mhz * 1e6);
        }
    }

    /// The packet log, and everything else that feeds the bus.
    ///
    /// Feeds live here rather than beside the tuner because that is what they
    /// are: another front end putting packets on the same bus, whose frames
    /// reach the packet list, the log and the flight list exactly like the
    /// ones this receiver demodulated itself.
    /// Which model the Agent view talks to.
    ///
    /// Written out as it is changed rather than on a save button: the file is
    /// four lines and a receiver that forgets an endpoint because nobody
    /// found the button is a receiver with no agent.
    /// The agent: the model it thinks with, what it answers to on the air,
    /// and where its voice comes from. Three cards, and under each a lamp
    /// saying whether that part will work as it is set, because the only
    /// other way to find out is to close the dialog and try.
    fn agent_settings(&mut self, ui: &mut egui::Ui) {
        use crate::agent::served;
        let before = self.chat.config.clone();
        let c = &mut self.chat.config;
        // What the servers offer, asked once per address and again on the
        // button: a model id and a voice name are strings only the server
        // can check.
        served::fetch(&c.url, &c.key, false);
        if c.speech == Speech::Server {
            let key = if c.voice_key.trim().is_empty() { &c.key } else { &c.voice_key };
            served::fetch(&c.voice_url, key, false);
        }
        let chat = served::served(&c.url);
        // The listing arrives on a thread of its own, so the dialog has to
        // come back and look.
        if matches!(chat, Some(served::Served { state: served::State::Fetching })) {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(300));
        }

        section(ui, "model", "any server speaking the OpenAI chat API with tool calls", |ui| {
            row_help(
                ui,
                "server",
                "The base address, without /chat/completions. A server on this machine \
                     needs no key and sends nothing anywhere.",
                |ui| {
                    let mut ask = false;
                    field_then(ui, &mut c.url, crate::agent::config::DEFAULT_URL, 60.0, |ui| {
                        ask = ui.small_button("ASK").on_hover_text("List what it serves").clicked();
                    });
                    if ask {
                        served::fetch(&c.url, &c.key, true);
                    }
                },
            );
            row_help(ui, "model", "As the server names it.", |ui| {
                let offered = chat.as_ref().map(|s| s.chat_models()).unwrap_or_default();
                pick_or_type(ui, "chat-model", &mut c.model, offered, "qwen3:8b");
            });
            row_help(ui, "key", "Sent as a bearer token. Leave empty for a local server.", |ui| {
                secret(ui, &mut c.key);
            });
            row_help(
                ui,
                "tool calls",
                "How many calls one question may take before the model is stopped. The \
                     model is given the whole receiver, and one in a loop would drive it all \
                     night.",
                |ui| {
                    let mut steps = c.steps as u32;
                    if ui.add(egui::DragValue::new(&mut steps).range(1..=100)).changed() {
                        c.steps = steps as usize;
                    }
                },
            );
            row_help(
                ui,
                "brief",
                "Anything the model should know that the receiver cannot tell it: your \
                     callsign, what not to key up, who it is talking to.",
                |ui| {
                    prose(ui, &mut c.brief, "Standing instructions", 3);
                },
            );
            ui.add_space(4.0);
            let at = format!("{} at {}", c.model.trim(), host_of(&c.url));
            match (c.fault(), chat.as_ref().map(|s| &s.state)) {
                (Some(why), _) => lamp(ui, false, why),
                (None, Some(served::State::Failed(e))) => {
                    lamp(ui, false, &format!("{}: {e}", host_of(&c.url)))
                }
                (None, Some(served::State::Fetching)) => {
                    lamp(ui, true, &format!("asking {} what it serves", host_of(&c.url)))
                }
                (None, Some(served::State::Ready(m))) => {
                    match m.iter().any(|x| x.id == c.model.trim()) {
                        true => lamp(ui, true, &at),
                        false => lamp(ui, false, &format!("{at}, which it does not list")),
                    }
                }
                (None, None) => lamp(ui, true, &at),
            }
        });
        ui.add_space(8.0);

        section(ui, "on the air", "a channel set to AGENT on the strip, marked as voice", |ui| {
            row_help(
                ui,
                "name",
                "What it answers to. An over that does not start with this is heard and \
                 ignored, unless it is already in a conversation. Empty means it never keys.",
                |ui| {
                    field(ui, &mut c.wake, "shark");
                },
            );
            row_help(
                ui,
                "hang",
                "Seconds the channel must be quiet before it keys. It never keys while the \
                 squelch is open.",
                |ui| {
                    ui.add(egui::DragValue::new(&mut c.hang_s).speed(0.1).range(0.0..=30.0));
                    theme::Line::new().legend("s").show(ui);
                },
            );
            row_help(
                ui,
                "follow",
                "Seconds after its own over that it keeps answering without being named, so a \
                 conversation does not need the name every time. Zero wants the name on every \
                 over, which is what to set on a busy channel.",
                |ui| {
                    ui.add(egui::DragValue::new(&mut c.follow_s).speed(1.0).range(0.0..=600.0));
                    theme::Line::new().legend("s").show(ui);
                },
            );
            ui.add_space(4.0);
            match c.wake.trim() {
                "" => lamp(ui, false, "no name: it will listen and never answer"),
                name => {
                    let follow = match c.follow_s > 0.0 {
                        true => format!(", then anything for {:.0} s", c.follow_s),
                        false => String::new(),
                    };
                    lamp(
                        ui,
                        true,
                        &format!(
                            "answers to {name}, {:.1} s after the channel clears{follow}",
                            c.hang_s
                        ),
                    )
                }
            }
        });
        ui.add_space(8.0);

        section(ui, "voice", "how what it says is turned into speech", |ui| {
            row_help(
                ui,
                "from",
                "A model on this machine needs nothing else running and about 3.5 GB of \
                 weights, fetched the first time it speaks, and wants the card. The model's \
                 server needs an /audio/speech beside its chat, as OpenAI and OpenRouter \
                 have. A speech server is one of its own, with its own address and key.",
                |ui| {
                    for how in Speech::ALL {
                        if ui.selectable_label(c.speech == how, how.label()).clicked() {
                            c.speech = how;
                        }
                    }
                },
            );
            match c.speech {
                Speech::Local => Self::local_voice_rows(ui, c),
                Speech::Chat => {
                    let models = chat.as_ref().map(|s| s.speech_models()).unwrap_or_default();
                    let voices =
                        chat.as_ref().map(|s| s.voices_of(&c.voice_model)).unwrap_or_default();
                    row_help(
                        ui,
                        "model",
                        "As the model's server names it: tts-1 on OpenAI, \
                         hexgrad/kokoro-82m on OpenRouter.",
                        |ui| {
                            pick_or_type(ui, "speech-model", &mut c.voice_model, models, "tts-1");
                        },
                    );
                    row_help(ui, "voice", "As that model names them.", |ui| {
                        pick_or_type(ui, "speech-voice", &mut c.voice, voices, "alloy");
                    });
                }
                Speech::Server => {
                    let own = served::served(&c.voice_url);
                    let models = own.as_ref().map(|s| s.speech_models()).unwrap_or_default();
                    let voices =
                        own.as_ref().map(|s| s.voices_of(&c.voice_model)).unwrap_or_default();
                    row_help(
                        ui,
                        "server",
                        "An OpenAI-compatible /v1/audio/speech. Often not the same server as \
                         the model.",
                        |ui| {
                            let mut ask = false;
                            field_then(
                                ui,
                                &mut c.voice_url,
                                "https://api.openai.com/v1",
                                60.0,
                                |ui| {
                                    ask = ui
                                        .small_button("ASK")
                                        .on_hover_text("List what it serves")
                                        .clicked();
                                },
                            );
                            if ask {
                                let key = match c.voice_key.trim() {
                                    "" => c.key.clone(),
                                    k => k.to_string(),
                                };
                                served::fetch(&c.voice_url, &key, true);
                            }
                        },
                    );
                    row_help(ui, "model", "As that server names it.", |ui| {
                        pick_or_type(ui, "own-speech-model", &mut c.voice_model, models, "tts-1");
                    });
                    row_help(ui, "voice", "As that model names them.", |ui| {
                        pick_or_type(ui, "own-speech-voice", &mut c.voice, voices, "alloy");
                    });
                    row_help(ui, "key", "Leave empty to use the model's key.", |ui| {
                        secret(ui, &mut c.voice_key);
                    });
                }
            }
            ui.add_space(4.0);
            match c.voice_fault() {
                None => {
                    let from = match c.speech {
                        Speech::Local => local_voice_lamp(c),
                        Speech::Chat => format!("{} at {}", c.voice_model.trim(), host_of(&c.url)),
                        Speech::Server => {
                            format!("{} at {}", c.voice_model.trim(), host_of(&c.voice_url))
                        }
                    };
                    // What the server lists, when it lists anything: a
                    // voice it does not have is a 400 at the first over.
                    let listing = match c.speech {
                        Speech::Chat => served::served(&c.url),
                        Speech::Server => served::served(&c.voice_url),
                        Speech::Local => None,
                    };
                    let voices = listing.map(|l| l.voices_of(&c.voice_model)).unwrap_or_default();
                    if !voices.is_empty() && !voices.iter().any(|v| v == c.voice.trim()) {
                        lamp(ui, false, &format!("{from}, which has no voice {}", c.voice.trim()));
                    } else {
                        lamp(ui, true, &from);
                    }
                }
                Some(why) => lamp(ui, false, why),
            }
        });

        ui.add_space(8.0);

        section(ui, "reading", "how speech heard on the air is read back into words", |ui| {
            row_help(
                ui,
                "on",
                "A model on this machine reads everything locally and wants the card and the \
                 weights, chosen on the Transcript pane. The model's server, or one of its \
                 own, reads it on an /audio/transcriptions instead, which is what a machine \
                 with no card should use. Audio leaves this machine either way it is not local.",
                |ui| {
                    for how in Reading::ALL {
                        if ui.selectable_label(c.reading == how, how.label()).clicked() {
                            c.reading = how;
                        }
                    }
                },
            );
            match c.reading {
                Reading::Local => {
                    hint(ui, "Which model and which device are on the Transcript pane.");
                }
                Reading::Chat => {
                    let models = chat.as_ref().map(|s| s.reading_models()).unwrap_or_default();
                    row_help(
                        ui,
                        "model",
                        "As the model's server names it: whisper-1 on OpenAI.",
                        |ui| {
                            pick_or_type(ui, "read-model", &mut c.read_model, models, "whisper-1");
                        },
                    );
                }
                Reading::Server => {
                    let own = served::served(&c.read_url);
                    let models = own.as_ref().map(|s| s.reading_models()).unwrap_or_default();
                    row_help(
                        ui,
                        "server",
                        "An OpenAI-compatible /v1/audio/transcriptions: a hosted one, or a \
                         whisper.cpp or faster-whisper server on the network.",
                        |ui| {
                            let mut ask = false;
                            field_then(
                                ui,
                                &mut c.read_url,
                                "http://127.0.0.1:9000/v1",
                                60.0,
                                |ui| {
                                    ask = ui
                                        .small_button("ASK")
                                        .on_hover_text("List what it serves")
                                        .clicked();
                                },
                            );
                            if ask {
                                let key = match c.read_key.trim() {
                                    "" => c.key.clone(),
                                    k => k.to_string(),
                                };
                                served::fetch(&c.read_url, &key, true);
                            }
                        },
                    );
                    row_help(ui, "model", "As that server names it.", |ui| {
                        pick_or_type(ui, "own-read-model", &mut c.read_model, models, "whisper-1");
                    });
                    row_help(ui, "key", "Leave empty to use the model's key.", |ui| {
                        secret(ui, &mut c.read_key);
                    });
                }
            }
            ui.add_space(4.0);
            match (c.reading, c.reading_fault()) {
                (_, Some(why)) => lamp(ui, false, why),
                (Reading::Local, _) => lamp(ui, true, "a model here, on the Transcript pane"),
                (Reading::Chat, _) => {
                    lamp(ui, true, &format!("{} at {}", c.read_model.trim(), host_of(&c.url)))
                }
                (Reading::Server, _) => {
                    lamp(ui, true, &format!("{} at {}", c.read_model.trim(), host_of(&c.read_url)))
                }
            }
        });

        if *c != before {
            let _ = c.save();
            // The transcriber is a stage in a graph that knows nothing about
            // the agent, and it reads where this says.
            crate::agent::config::publish_reading(c);
        }
    }

    /// The rows for a voice made on this machine: which speaker, where it
    /// runs, and where its files are.
    fn local_voice_rows(ui: &mut egui::Ui, c: &mut crate::agent::config::Config) {
        #[cfg(feature = "tts")]
        {
            let chosen = match c.voice_local.trim() {
                "" => tts::DEFAULT_VOICE.to_string(),
                name => name.to_string(),
            };
            row_help(
                ui,
                "speaker",
                "The letter after the grade is the publisher's, from how much and how good \
                 the audio behind each voice was. They are the same arithmetic and they do \
                 not sound alike.",
                |ui| {
                    egui::ComboBox::from_id_salt("voice_local")
                        .selected_text(tts::label_of(&chosen))
                        .width(300.0)
                        .show_ui(ui, |ui| {
                            for (id, label) in voices(&c.voice_dir) {
                                let on = chosen == id;
                                if ui.selectable_label(on, label).clicked() {
                                    c.voice_local = id;
                                }
                            }
                        });
                },
            );
            hint(ui, "Kokoro, 82 million parameters, ahead of real time on a processor.");
            row_help(
                ui,
                "run on",
                "Auto takes the fastest that will hold the weights and falls back to the CPU, \
                 saying so. A card picked by name fails rather than falling back.",
                |ui| {
                    let want = tts::DeviceChoice::parse(&c.voice_device).id();
                    egui::ComboBox::from_id_salt("voice_device")
                        .selected_text(
                            tts::devices()
                                .into_iter()
                                .find(|(id, _)| *id == want)
                                .map(|(_, l)| l)
                                .unwrap_or_else(|| want.clone()),
                        )
                        .width(300.0)
                        .show_ui(ui, |ui| {
                            for (id, label) in tts::devices() {
                                if ui.selectable_label(want == id, label).clicked() {
                                    c.voice_device = id;
                                }
                            }
                        });
                },
            );
        }
        row_help(
            ui,
            "files",
            "The model, the voices and the pronunciation dictionary. Empty for the usual \
             place.",
            |ui| {
                field(ui, &mut c.voice_dir, &voice_dir_hint());
            },
        );
    }

    fn packet_log_settings(&mut self, ui: &mut egui::Ui) {
        let (logged, bytes, full) = match &self.radio {
            Some(r) => {
                use std::sync::atomic::Ordering;
                (
                    r.status.logged.load(Ordering::Relaxed),
                    r.status.log_bytes.load(Ordering::Relaxed),
                    r.status.log_full.load(Ordering::Relaxed),
                )
            }
            None => (0, 0, false),
        };

        section(ui, "packet log", "every burst the front ends read, a day per file", |ui| {
            // Off until it is asked for, and remembered once it is: writing
            // every burst a receiver hears onto somebody's disc is a decision
            // for them to make.
            let mut on = self.log.path.is_some();
            let log_help = "Timings and frames as demodulated, replayable.";
            if switch(ui, "write", &mut on, "every packet to disk", log_help) {
                let dir = if on {
                    self.log_dir.clone().or_else(crate::packetlog::PacketLog::default_dir)
                } else {
                    None
                };
                self.log.path = dir.clone();
                self.send(Cmd::PacketLog(dir));
            }
            // What the list shows, rather than what the receiver does. An
            // unrecognised burst is still reported, logged and replayable
            // with this off; it is only kept out of the table.
            let mut unknown = self.log.show_unknown;
            let unknown_help = "Bursts that decoded to no known protocol. They are the point \
                                of scanning an unfamiliar band, and on a noisy one they bury \
                                the decodes.";
            if switch(ui, "list", &mut unknown, "unrecognised bursts too", unknown_help) {
                self.log.show_unknown = unknown;
            }
            row_help(ui, "folder", "Where the files go. Enter or SET applies it.", |ui| {
                let mut set = false;
                let r = field_then(ui, &mut self.log_dir_edit, "where the files go", 44.0, |ui| {
                    set = ui.small_button("SET").clicked();
                });
                let typed = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                if typed || set {
                    let dir = std::path::PathBuf::from(self.log_dir_edit.trim());
                    if !self.log_dir_edit.trim().is_empty() {
                        self.log_dir = Some(dir.clone());
                        self.log.path = Some(dir.clone());
                        self.send(Cmd::PacketLog(Some(dir)));
                    }
                }
            });
            let cap_help = "What the whole folder may take. The oldest days are deleted to \
                            keep it under, so the log rolls rather than stopping.";
            row_help(ui, "limit", cap_help, |ui| {
                let mut cap = self.log_cap_mb;
                let opts = [Some(512u64), Some(2048), Some(8192), Some(32_768), None];
                if choice(ui, "log_cap", &mut cap, opts.map(|o| (o, size_label(o)))) {
                    self.log_cap_mb = cap;
                    self.send(Cmd::PacketLogCap(cap.map(|mb| mb << 20)));
                }
            });
            ui.add_space(4.0);
            reading(ui, "folder holds", human_bytes(bytes));
            reading(ui, "this session", format!("{logged} packets"));
            if full {
                lamp(ui, false, "stopped: today's file is over the limit on its own");
            } else if on {
                lamp(ui, true, "writing");
            }
        });
        ui.add_space(8.0);

        // The call log, beside the packet log: the same decision about the
        // same disc, for speech rather than packets. Off until asked for.
        let rec = self.radio.as_ref().and_then(|r| r.status.recorder.lock().clone());
        section(ui, "calls", "every over heard, kept as Opus", |ui| {
            let Some(rec) = rec else {
                lamp(ui, false, "no receiver running");
                return;
            };
            let mut on = rec.on;
            let help = "Every transmission on a voice channel or a voice front end, as it \
                        was heard, before any fader: about 2 kB a second of speech. The \
                        Recordings table in the Calls view plays them back.";
            if switch(ui, "record", &mut on, "every over", help) {
                self.cmds.push(Cmd::StageParam(
                    crate::chain::derived::CALL_LOG,
                    "enabled".into(),
                    pipeline::param::ParamValue::Bool(on),
                ));
            }
            reading(ui, "folder", rec.dir.clone());
            reading(ui, "folder holds", human_bytes(rec.bytes));
            reading(ui, "this session", format!("{} calls", rec.calls));
            match (rec.full, rec.on, rec.recording) {
                (true, ..) => lamp(ui, false, "stopped: the folder is at its limit"),
                (_, true, 0) => lamp(ui, true, "recording, nobody talking"),
                (_, true, n) => {
                    lamp(ui, true, &format!("recording {n} over{}", if n == 1 { "" } else { "s" }))
                }
                (_, false, _) => lamp(ui, false, "off: nothing is kept"),
            }
        });
        ui.add_space(8.0);

        let status = self.radio.as_ref().map(|r| r.status.feeds.lock().clone()).unwrap_or_default();
        let mut remove = None;
        section(ui, "feeds", "packets from another receiver, over TCP", |ui| {
            for (i, f) in self.feeds.iter().enumerate() {
                let live = status.iter().find(|s| s.spec == *f);
                let (ok, said) = match live {
                    Some(s) if s.connected => (true, format!("{} frames", s.frames)),
                    Some(s) => (false, s.error.clone().unwrap_or_else(|| "not connected".into())),
                    None => (false, "no receiver running".into()),
                };
                ui.horizontal(|ui| {
                    theme::Line::new().value(f.address()).size(12.0).legend(f.kind.name).show(ui);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.small_button("REMOVE").clicked() {
                            remove = Some(i);
                        }
                    });
                });
                // A feed that is down says why. The alternative is a dark
                // lamp and a guess about whether it is the network, the
                // port, or a receiver somebody turned off.
                lamp(ui, ok, &said);
                ui.add_space(4.0);
            }
            row_help(ui, "add", "A host, or host:port, and what it speaks.", |ui| {
                let mut add = false;
                let mut kind = self.feed_kind;
                field_then(ui, &mut self.feed_host, "host, or host:port", 150.0, |ui| {
                    egui::ComboBox::from_id_salt("feed_kind")
                        .selected_text(kind.name)
                        .width(90.0)
                        .show_ui(ui, |ui| {
                            for k in nodes::FEED_KINDS {
                                if ui.selectable_label(kind.name == k.name, k.name).clicked() {
                                    kind = k;
                                }
                            }
                        });
                    add = ui.button("ADD").clicked();
                });
                self.feed_kind = kind;
                if add {
                    match parse_feed(&self.feed_host, self.feed_kind) {
                        Some(spec) if !self.feeds.contains(&spec) => {
                            self.feeds.push(spec);
                            self.send(Cmd::Feeds(self.feeds.clone()));
                            self.feed_host.clear();
                        }
                        Some(_) => self.err = Some("that feed is already attached".into()),
                        None => self.err = Some("expected host or host:port".into()),
                    }
                }
            });
        });
        if let Some(i) = remove {
            self.feeds.remove(i);
            self.send(Cmd::Feeds(self.feeds.clone()));
        }
    }

    /// Everything the radio itself can be set to.
    ///
    /// Where this receiver is, rather than what it is doing.
    ///
    /// One pane for the settings that are true of the installation and not of
    /// the session: they survive changing radio, they are asked once, and
    /// none of them belong under a cog on the spectrum.
    fn app_settings(&mut self, ui: &mut egui::Ui) {
        let t = crate::i18n::t;

        section(ui, "here", "the language, the country and the band plan the dial names", |ui| {
            row_help(ui, t("settings.language"), t("settings.language.help"), |ui| {
                let mut lang = crate::i18n::language();
                let opts = crate::i18n::Language::ALL.iter().map(|l| (*l, l.label().to_string()));
                if choice(ui, "app-language", &mut lang, opts) {
                    crate::i18n::set_language(lang);
                }
            });
            row_help(ui, t("settings.country"), t("settings.country.help"), |ui| {
                let mut code = self.country.clone();
                let opts = crate::locale::COUNTRIES
                    .iter()
                    .map(|c| (c.code.to_string(), c.name.to_string()));
                if choice(ui, "app-country", &mut code, opts)
                    && let Some(c) = crate::locale::by_code(&code)
                {
                    self.country = c.code.to_string();
                    // The cell export is fetched per country, so the
                    // dataset pane has to hear about this to know which
                    // one it would fetch.
                    crate::data::set_country(&self.country);
                    // A country decides the plan the first time and then
                    // stops having an opinion, so choosing one after
                    // overriding the plan puts the override back rather
                    // than leaving a mismatch nobody asked for.
                    crate::bands::set_plan(c.plan);
                    // The map has to open somewhere. A capital city is
                    // wrong by a couple of hundred miles, which is close
                    // enough to draw with and is replaced the moment a
                    // real position is typed in.
                    if self.location.is_none() {
                        self.set_location(c.centre.0, c.centre.1);
                        self.station_edit = None;
                    }
                }
            });
            row_help(ui, t("settings.band_plan"), t("settings.band_plan.help"), |ui| {
                let mut plan = crate::bands::plan();
                let opts = crate::bands::Plan::ALL.iter().map(|p| (*p, p.label().to_string()));
                if choice(ui, "app-band-plan", &mut plan, opts) {
                    crate::bands::set_plan(plan);
                }
            });
            // The plan is abstract until it is applied to the frequency in
            // front of you, and this is the one line that makes the choice
            // concrete.
            reading(
                ui,
                "so",
                format!(
                    "{} here is {}",
                    fmt_hz(self.center),
                    crate::bands::name_at_in(crate::bands::plan(), self.center)
                ),
            );
        });
        ui.add_space(8.0);

        // Sound devices. Here rather than with the radio's controls because
        // they are not the radio: which speaker the mix comes out of and
        // which microphone a keyed channel transmits from are properties of
        // this machine.
        section(ui, "sound", "this machine's speaker and microphone", |ui| {
            row_help(ui, "speaker", "Where the mix, the calls and any replay come out.", |ui| {
                let mut out = self.audio_out.clone();
                if device_combo(ui, "app-audio-out", &mut out, audio::AudioPlayer::devices()) {
                    self.audio_out = out;
                    self.send_audio();
                }
            });
            let mic = "What a keyed channel transmits. Held open while a channel is set to \
                       MIC, so the meter moves before you key.";
            row_help(ui, "microphone", mic, |ui| {
                let mut input = self.audio_in.clone();
                if device_combo(ui, "app-audio-in", &mut input, audio::AudioCapture::devices()) {
                    self.audio_in = input;
                    self.send_audio();
                }
            });
        });
        ui.add_space(8.0);

        section(ui, "opens on", "the first view when the window comes up", |ui| {
            let mut on = self.dashboard;
            let help = "Quick start and receiver status, as the first view. Off takes its \
                        tab away and opens the receiver on the spectrum.";
            if switch(ui, "dashboard", &mut on, "show it first", help) {
                match on {
                    true => self.dashboard = true,
                    false => self.hide_dashboard(),
                }
            }
        });
        ui.add_space(8.0);

        section(ui, t("settings.position"), "where the aerial is, for ranges and the map", |ui| {
            row_help(ui, "station", t("settings.position.help"), |ui| {
                let mut edit = self.station_edit.take();
                let text = edit.get_or_insert_with(|| match self.location {
                    Some((lat, lon)) => format!("{lat:.4}, {lon:.4}"),
                    None => String::new(),
                });
                let mut pressed = false;
                let r = field_then(ui, text, "lat, lon", 44.0, |ui| {
                    pressed = ui.small_button("SET").clicked();
                });
                let typed = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                let mut set = None;
                if (typed || pressed)
                    && let Ok(p) = crate::parse_location(text)
                {
                    set = Some(p);
                }
                self.station_edit = edit;
                if let Some((lat, lon)) = set {
                    self.set_location(lat, lon);
                    self.station_edit = None;
                }
            });
            reading(
                ui,
                "or",
                if self.location.is_some() {
                    "right-click the map to move it"
                } else {
                    "right-click the map"
                },
            );
            self.gps_rows(ui);
        });
        ui.add_space(8.0);

        Self::version_settings(ui);
    }

    /// What this build is, and what the newest published release is.
    ///
    /// Nothing is downloaded here. The answer an operator wants is whether
    /// the binary they are running is the current one, and where to get the
    /// one that is; the archive's name is shown because a release carries one
    /// per platform and picking the wrong one is the usual mistake.
    fn version_settings(ui: &mut egui::Ui) {
        let t = crate::i18n::t;
        let state = crate::update::state();
        let busy = matches!(state, crate::update::State::Checking);
        section(ui, t("settings.version"), "what this build is, and the newest release", |ui| {
            ui.horizontal(|ui| {
                theme::Line::new()
                    .legend("running")
                    .value(crate::update::running())
                    .size(13.0)
                    .gap(18.0)
                    .legend("build")
                    .value(crate::update::platform())
                    .size(13.0)
                    .show(ui);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let label = if busy { "CHECKING" } else { t("ui.check") };
                    if ui.add_enabled(!busy, egui::Button::new(legend(label))).clicked() {
                        crate::update::check_now();
                    }
                });
            });
            match &state {
                crate::update::State::Unchecked => lamp(ui, true, "not checked yet"),
                crate::update::State::Checking => {
                    lamp(ui, true, "asking GitHub for the latest release");
                }
                crate::update::State::Current(r) => {
                    lamp(ui, true, &format!("up to date, latest release is {}", r.version));
                }
                crate::update::State::Newer(r) => {
                    lamp(ui, true, &format!("{} is available", r.version));
                    match &r.asset {
                        Some(a) => reading(
                            ui,
                            "archive",
                            format!("{} ({})", a.name, crate::data::fmt_bytes(a.bytes)),
                        ),
                        None => reading(
                            ui,
                            "archive",
                            format!("none for {} in that release", crate::update::platform()),
                        ),
                    }
                    if !r.page.is_empty() && ui.button(legend("OPEN THE RELEASE")).clicked() {
                        ui.ctx().open_url(egui::OpenUrl::new_tab(r.page.clone()));
                    }
                }
                crate::update::State::Failed(e) => lamp(ui, false, e),
            }
        });
        // The check runs on a thread of its own, so without this the answer
        // sits unshown until the pointer moves.
        if busy {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(200));
        }
    }

    /// Where the position comes from, and what the survey does with it.
    ///
    /// Under the station position rather than in a card of its own, because
    /// a GPS is not a feature of the device database: it is the other way of
    /// answering the question the row above asks, and a receiver that is
    /// moving should say so where somebody would go to type a position by
    /// hand.
    fn gps_rows(&mut self, ui: &mut egui::Ui) {
        let help = "A fix moves the station position, which is the position everything \
                    else works from. The reader always runs and looks for a gpsd on this \
                    machine, so nothing need be set here unless the receiver is a serial \
                    port or a daemon elsewhere: a device path such as /dev/ttyACM0, or \
                    gpsd:host.";
        let mut set: Option<Option<gps::Transport>> = None;
        row_help(ui, "gps", help, |ui| {
            let text = self.survey.gps_edit.get_or_insert_with(|| {
                self.survey.gps.as_ref().map(|t| t.to_string()).unwrap_or_default()
            });
            let (mut pressed, mut auto) = (false, false);
            let named = self.survey.gps.is_some();
            let reserve = if named { 92.0 } else { 44.0 };
            let r = field_then(ui, text, gps::Transport::LOCAL_GPSD, reserve, |ui| {
                pressed = ui.small_button("SET").clicked();
                if named {
                    auto = ui.small_button("AUTO").clicked();
                }
            });
            let typed = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if typed || pressed {
                // An empty box is the local gpsd rather than nothing: there
                // is no off, since a receiver with no fix simply leaves the
                // station where it was put.
                set = Some(gps::Transport::parse(text));
            }
            if auto {
                text.clear();
                set = Some(None);
            }
        });
        if let Some(t) = set {
            self.set_gps(t);
            self.survey.gps_edit = None;
        }
        // What the link is doing, which is three different states an operator
        // has to be able to tell apart: nothing listening on the other end, a
        // receiver talking but with no sky, and a real fix.
        let (ok, line) = match (crate::station::connected(), crate::station::fix()) {
            (_, Some(f)) => {
                let sats = f.sats.map(|n| format!(", {n} satellites")).unwrap_or_default();
                let how = f.accuracy_m().map(|m| format!(", ±{m:.0} m")).unwrap_or_default();
                (
                    true,
                    format!(
                        "{:.5}, {:.5}{sats}{how}, {} fixes",
                        f.lat,
                        f.lon,
                        crate::station::fixes()
                    ),
                )
            }
            // Waiting says nothing on its own: an antenna indoors and an
            // antenna unplugged look the same for the first minute, and the
            // satellite counts tell them apart.
            (true, None) => match crate::station::sky() {
                Some(s) => (
                    true,
                    format!("connected, no fix yet: {} of {} satellites used", s.used, s.seen),
                ),
                None => (true, "connected, waiting for a fix".into()),
            },
            // Nothing answering is only a fault when somebody named a
            // receiver; the local gpsd is looked for whether or not one is
            // there.
            (false, None) => (
                self.survey.gps.is_none(),
                "no gps answering: the station is where it was set".into(),
            ),
        };
        lamp(ui, ok, &line);
        // The reader is not the radio's, so this pane keeps its own clock:
        // without it a fix arriving while nothing else is moving would sit
        // unshown until the pointer did.
        ui.ctx().request_repaint_after(std::time::Duration::from_millis(500));
    }

    /// What is in the dataset cache, and the buttons that go and ask.
    ///
    /// A window of its own rather than a block inside Setup. The airports,
    /// repeaters, host files and registries are somebody else's files kept on
    /// this machine, there are a dozen of them and there will be more, and
    /// the questions an operator has about each are how old the copy is, how
    /// much disc it is using, and whether the last attempt to update it
    /// worked. That is a list, and a list does not fit under the three
    /// settings Setup is actually about.
    fn data_settings(&mut self, ui: &mut egui::Ui) {
        let t = crate::i18n::t;
        let rows = crate::data::status();
        let busy = rows.iter().any(|r| r.busy);
        let w = ui.available_width();
        egui::ScrollArea::vertical().max_height(460.0).show(ui, |ui| {
            ui.set_max_width(w);
            for r in &rows {
                // The rail says whether the copy is here: cyan for a
                // dataset on disc, nothing for one never fetched, red for
                // one whose last fetch failed.
                let rail = match (&r.error, r.bytes > 0) {
                    (Some(_), _) => Some(theme::FAULT),
                    (None, true) => Some(theme::TRACE),
                    (None, false) => None,
                };
                card(
                    ui,
                    rail,
                    |ui| {
                        theme::Line::new()
                            .legend(&r.which.label())
                            .note(r.which.publisher())
                            .size(10.5)
                            .show(ui);
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            // The way to the publisher's own page, which is
                            // where the terms, the credit and the contact
                            // are. Never the fetch URL: the cell export's
                            // carries the operator's token.
                            if crate::icons::icon_button_sized(
                                ui,
                                crate::icons::Icon::Link,
                                r.which.page(),
                                true,
                                false,
                                20.0,
                            )
                            .clicked()
                            {
                                ui.ctx()
                                    .open_url(egui::OpenUrl::new_tab(r.which.page().to_string()));
                            }
                            // Disabled rather than hidden while it works: a
                            // button that vanishes under the pointer is a
                            // button that gets pressed twice.
                            let label = if r.busy { "CHECKING" } else { t("ui.refresh") };
                            let can = !r.busy && r.blocked.is_none();
                            if ui
                                .add_enabled(can, egui::Button::new(legend(label)))
                                .on_hover_text(r.which.about())
                                .clicked()
                            {
                                crate::data::refresh(r.which);
                            }
                        });
                    },
                    |ui| {
                        theme::Line::new()
                            .legend("held")
                            .value(match r.rows {
                                Some(n) => format!("{n} rows"),
                                // Cached but not parsed is the ordinary
                                // state for the registries, which are read
                                // the first time something asks them a
                                // question.
                                None if r.bytes > 0 => "on disc".into(),
                                None => "not downloaded".into(),
                            })
                            .size(12.0)
                            .gap(18.0)
                            .legend("size")
                            .value(crate::data::fmt_bytes(r.bytes))
                            .size(12.0)
                            .gap(18.0)
                            .legend("checked")
                            .value(match r.checked_ago {
                                Some(s) => crate::data::fmt_ago(s),
                                None => "never".into(),
                            })
                            .size(12.0)
                            .show(ui);
                        // The terms on the row, not in a document nobody
                        // opens. OpenCelliD asks in writing for a visible
                        // credit and a link, and a receiver that draws its
                        // masts while saying nothing is not complying.
                        hint(ui, r.which.terms());
                        if let Some(e) = &r.error {
                            lamp(ui, false, e);
                        }
                        if let Some(b) = r.blocked {
                            hint(ui, b);
                        }
                        for (i, k) in r.which.keys().iter().enumerate() {
                            self.key_field(ui, r.which, i, *k);
                        }
                    },
                );
                ui.add_space(6.0);
            }
        });
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            if ui.add_enabled(!busy, egui::Button::new(legend(t("ui.refresh_all")))).clicked() {
                // A dataset that cannot be fetched is skipped rather than
                // failed: refresh all is a convenience, not a demand for a
                // token.
                for w in crate::data::Which::all().iter().filter(|w| w.blocked().is_none()) {
                    crate::data::refresh(*w);
                }
            }
            help(ui, t("settings.data.help"));
            if let Some(dir) = crate::data::cache_dir() {
                theme::Line::new().note(dir.display().to_string()).size(10.0).elided(ui);
            }
        });
        // A check runs on its own thread and finishes without an event, so
        // the pane has to come back and look, or a finished download stays
        // reading CHECKING until the pointer moves.
        if busy {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(400));
        }
    }

    /// The credential a dataset needs, on that dataset's row.
    ///
    /// On the row rather than in a field under the list, and not in Setup at
    /// all: a token is not a preference, it is the one thing standing
    /// between that row and a download, and the link beside it goes to the
    /// page that hands one out.
    fn key_field(
        &mut self,
        ui: &mut egui::Ui,
        which: crate::data::Which,
        index: usize,
        k: crate::data::Key,
    ) {
        let Some(slot) = self.key_slot(which, index) else {
            return;
        };
        let before = slot.clone();
        row_help(ui, k.label, k.help, |ui| {
            if k.secret {
                secret(ui, slot);
            } else {
                field(ui, slot, k.hint);
            }
        });
        if *slot != before {
            let value = slot.clone();
            which.set_key(index, &value);
        }
    }

    /// Where this pane holds the credential for a dataset, so what is typed
    /// is what the session saves.
    fn key_slot(&mut self, which: crate::data::Which, index: usize) -> Option<&mut String> {
        match (which, index) {
            (crate::data::Which::CellTowers, 0) => Some(&mut self.opencellid_token),
            (crate::data::Which::Satellites(g), 0) if g.needs_login() => {
                Some(&mut self.spacetrack_identity)
            }
            (crate::data::Which::Satellites(g), _) if g.needs_login() => {
                Some(&mut self.spacetrack_password)
            }
            _ => None,
        }
    }

    /// Create a radio that is not on this machine.
    ///
    /// Nothing on the bus reveals a receiver on the network, so a remote radio
    /// is made rather than found: the address is the device. It is created
    /// where every other radio is chosen, because from the dial's point of
    /// view that is all it is.
    /// The wigle.net feed: the account it uploads as, and what it has done.
    ///
    /// Its own dialog rather than a row in settings because it is the one
    /// place in the program that holds a credential, and because what an
    /// operator wants when they open it is not a switch but an answer: how
    /// much is waiting, how much has gone, and what the last refusal said.
    pub(super) fn wigle_modal(&mut self, ctx: &egui::Context) {
        if !self.survey.wigle.open {
            return;
        }
        let mut close = false;
        let mut apply = false;
        let r = egui::containers::Modal::new(egui::Id::new("wigle"))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(520.0);
                modal_title(ui, "Feed wigle.net");
                let w = &mut self.survey.wigle;
                section(ui, "account", "from wigle.net/account, beside the token", |ui| {
                    row_help(ui, "API name", "It is not the name you log in with.", |ui| {
                        field(ui, &mut w.name, "AID00000000000000000000000000000");
                    });
                    row_help(
                        ui,
                        "API token",
                        "Stored in the session file in plain text, so treat it as a \
                         password that lives on this machine.",
                        |ui| {
                            secret(ui, &mut w.token);
                        },
                    );
                    let complete = !w.name.trim().is_empty() && !w.token.trim().is_empty();
                    match complete {
                        true => lamp(ui, true, &format!("uploading as {}", w.name.trim())),
                        false => lamp(ui, false, "no account: nothing can be uploaded"),
                    }
                });
                ui.add_space(8.0);
                section(ui, "upload", "Bluetooth devices and cells heard with a position", |ui| {
                    let why = "Written as WiGLE CSV and uploaded. Everything else the survey \
                               records stays on this machine: the format has no type for an \
                               aircraft or a pager, and a row filed under the wrong one cannot \
                               be taken back. A drive with no coverage sends when it gets home.";
                    if switch(ui, "upload", &mut w.on, "while receiving", why) {
                        apply = true;
                    }
                    let donate = "Lets wigle.net licence what you upload commercially. Off \
                                  unless you say otherwise: they are your observations to \
                                  give away.";
                    if switch(ui, "commercial", &mut w.donate, "allow commercial use", donate) {
                        apply = true;
                    }
                    // What it is actually doing. Three numbers and the last
                    // refusal, which is everything an operator can act on.
                    match w.status.as_ref() {
                        Some(st) => {
                            reading(
                                ui,
                                "waiting",
                                format!("{} rows in {} files", st.queued_rows, st.queued_files),
                            );
                            reading(
                                ui,
                                "uploaded",
                                format!("{} rows in {} files", st.sent_rows, st.sent_files),
                            );
                            if let Some(t) = &st.transaction {
                                reading(ui, "last", t);
                            }
                            reading(ui, "spool", st.spool.display().to_string());
                            if let Some(e) = &st.error {
                                lamp(ui, false, e);
                            }
                        }
                        None => lamp(ui, false, "no receiver running, so nothing is collected"),
                    }
                });
                footer(ui, |ui| {
                    if ui.button(crate::i18n::t("ui.close")).clicked() {
                        close = true;
                    }
                    if ui.button("APPLY").clicked() {
                        apply = true;
                    }
                });
            });
        if r.should_close() {
            close = true;
        }
        if apply {
            self.apply_wigle();
        }
        if close {
            self.survey.wigle.open = false;
        }
    }

    /// The beacondb.net feed: the switch, and what it has sent.
    ///
    /// Beside the WiGLE dialog rather than inside it because they are
    /// different bargains. beaconDB takes no account and puts what it
    /// collects into the public domain, so there is nothing to type and
    /// nothing to log in to; what there is instead is a decision, which is
    /// why this asks rather than defaulting to on.
    pub(super) fn beacondb_modal(&mut self, ctx: &egui::Context) {
        if !self.survey.beacondb.open {
            return;
        }
        let mut close = false;
        let mut apply = false;
        let r = egui::containers::Modal::new(egui::Id::new("beacondb"))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(520.0);
                modal_title(ui, "Feed beacondb.net");
                let b = &mut self.survey.beacondb;
                section(ui, "submit", "crowd-sourced, no account, published as collected", |ui| {
                    let why = "Bluetooth devices and cells heard with a position are spooled \
                               to disc and submitted when there is a network. Everything else \
                               the survey records stays here: there is no beacon type for an \
                               aircraft or a pager. What is submitted is where this receiver \
                               was when it heard something, so a drive is a track of where you \
                               have been. Levels are not sent: this receiver measures dBFS and \
                               the field means dBm.";
                    if switch(ui, "submit", &mut b.on, "while receiving", why) {
                        apply = true;
                    }
                    let ask = "Draws a position for a cell you have decoded that the \
                               OpenCelliD export has no row for, as a cross with the accuracy \
                               beaconDB gives it. Asking tells beaconDB which cells this \
                               receiver has heard, which is why it is separate from submitting.";
                    if switch(ui, "look up", &mut b.lookup, "where a heard cell is", ask) {
                        apply = true;
                    }
                    match b.status.as_ref() {
                        Some(st) => {
                            reading(
                                ui,
                                "waiting",
                                format!(
                                    "{} observations in {} files",
                                    st.queued_items, st.queued_files
                                ),
                            );
                            reading(
                                ui,
                                "submitted",
                                format!(
                                    "{} observations in {} files",
                                    st.sent_items, st.sent_files
                                ),
                            );
                            reading(ui, "spool", st.spool.display().to_string());
                            if let Some(e) = &st.error {
                                lamp(ui, false, e);
                            }
                        }
                        None => lamp(ui, false, "no receiver running, so nothing is collected"),
                    }
                });
                footer(ui, |ui| {
                    if ui.button(crate::i18n::t("ui.close")).clicked() {
                        close = true;
                    }
                    if ui.button("APPLY").clicked() {
                        apply = true;
                    }
                });
            });
        if r.should_close() {
            close = true;
        }
        if apply {
            self.apply_beacondb();
        }
        if close {
            self.survey.beacondb.open = false;
        }
    }

    /// Where every device heard goes into the house.
    ///
    /// One address and a switch. Everything else has a working default,
    /// because a person setting this up has already configured a broker once
    /// in Home Assistant and should not have to do it twice.
    pub(super) fn homeassistant_modal(&mut self, ctx: &egui::Context) {
        if !self.survey.homeassistant.open {
            return;
        }
        let mut close = false;
        let mut apply = false;
        let r = egui::containers::Modal::new(egui::Id::new("homeassistant"))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(520.0);
                modal_title(ui, "Publish to Home Assistant");
                let ha = &mut self.survey.homeassistant;
                section(ui, "broker", "the MQTT broker Home Assistant is already using", |ui| {
                    row_help(
                        ui,
                        "host",
                        "The host running Mosquitto, or whatever Home Assistant's MQTT \
                         integration is pointed at. What goes out is what your neighbours \
                         are transmitting as well as what you are, so point it at a broker \
                         on your own network.",
                        |ui| {
                            field_then(ui, &mut ha.host, "homeassistant.local", 78.0, |ui| {
                                ui.add_sized([70.0, 22.0], |ui: &mut egui::Ui| {
                                    field(ui, &mut ha.port, "1883")
                                });
                            });
                        },
                    );
                    row_help(
                        ui,
                        "user",
                        "Blank where the broker allows anonymous clients.",
                        |ui| {
                            field(ui, &mut ha.username, "");
                        },
                    );
                    row_help(
                        ui,
                        "password",
                        "Kept in the session file in plain text, so treat it as a password \
                         that lives on this machine.",
                        |ui| {
                            secret(ui, &mut ha.password);
                        },
                    );
                });
                ui.add_space(8.0);
                section(
                    ui,
                    "publish",
                    "every transmitter the decoders can name, as a device",
                    |ui| {
                        row_help(
                            ui,
                            "discovery",
                            "What Home Assistant listens under. Blank means homeassistant, which \
                         is what it uses unless somebody changed it.",
                            |ui| {
                                field(ui, &mut ha.prefix, "homeassistant");
                            },
                        );
                        row_help(
                            ui,
                            "topic",
                            "What this receiver's own topics live under. Blank means waveshark.",
                            |ui| {
                                field(ui, &mut ha.topic, "waveshark");
                            },
                        );
                        row_help(
                            ui,
                            "kinds",
                            "Which kinds of transmitter are worth a permanent device, comma \
                         separated. ism,wmbus is a house's own sensors and meters. Blank or \
                         all is everything, which on a Bluetooth band means the handsets \
                         walking past: those rotate their address every quarter of an hour, \
                         each is a message every ten seconds, and Home Assistant keeps every \
                         one it is told about.",
                            |ui| {
                                field(ui, &mut ha.spaces, "ism,wmbus");
                            },
                        );
                        let on_help = "While this is on, every named transmitter heard is \
                                   announced once and then reported at most every few \
                                   seconds. A city centre holds thousands of Bluetooth \
                                   addresses, so the node stops at a couple of hundred \
                                   devices.";
                        if switch(ui, "publish", &mut ha.on, "while receiving", on_help) {
                            apply = true;
                        }
                        let buses_help = "A call bus and a message bus beside the sensors: an \
                                          event to trigger on when somebody keys up or writes, \
                                          a lamp while the channel is busy, and who was heard \
                                          last. Off publishes the meters and keeps the traffic \
                                          off the dashboard.";
                        if switch(
                            ui,
                            "traffic",
                            &mut ha.buses,
                            "calls and messages too",
                            buses_help,
                        ) {
                            apply = true;
                        }
                        match ha.status.as_ref() {
                            Some(st) if st.configured => {
                                let said = format!(
                                    "{} {}, {} devices, {} published",
                                    if st.connected { "connected to" } else { "connecting to" },
                                    st.host,
                                    st.devices,
                                    st.published
                                );
                                lamp(ui, st.connected, &said);
                                if st.dropped > 0 {
                                    reading(
                                        ui,
                                        "dropped",
                                        format!(
                                            "{} readings: the broker was not keeping up",
                                            st.dropped
                                        ),
                                    );
                                }
                                if let Some(e) = &st.error {
                                    lamp(ui, false, e);
                                }
                            }
                            Some(_) => lamp(ui, false, "nothing is being published"),
                            None => lamp(ui, false, "no receiver running, so nothing is published"),
                        }
                    },
                );
                footer(ui, |ui| {
                    if ui.button(crate::i18n::t("ui.close")).clicked() {
                        close = true;
                    }
                    if ui.button("APPLY").clicked() {
                        apply = true;
                    }
                });
            });
        if r.should_close() {
            close = true;
        }
        if apply {
            self.apply_homeassistant();
        }
        if close {
            self.survey.homeassistant.open = false;
        }
    }

    pub(super) fn remote_modal(&mut self, ctx: &egui::Context) {
        let Some(mut edit) = self.remote.take() else {
            return;
        };
        let (mut close, mut add) = (false, false);
        let r = egui::containers::Modal::new(egui::Id::new("add-remote"))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(520.0);
                modal_title(ui, "Add remote radio");
                section(ui, "radio", "a receiver on the network, reached by address", |ui| {
                    row_help(ui, "protocol", edit.kind.help(), |ui| {
                        let opts = RemoteKind::ALL.iter().map(|k| (*k, k.label().to_string()));
                        choice(ui, "remote-kind", &mut edit.kind, opts);
                    });
                    let mut focus = None;
                    row_help(ui, "address", edit.kind.help(), |ui| {
                        let f = field(ui, &mut edit.host, edit.kind.placeholder());
                        if f.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                            add = true;
                        }
                        focus = Some(f);
                    });
                    row_help(
                        ui,
                        "name",
                        "What the radio list calls it. An address says which machine and \
                         nothing about which aerial.",
                        |ui| {
                            let name = field(ui, &mut edit.label, "loft dongle");
                            if name.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                                add = true;
                            }
                        },
                    );
                    // Focused so the address can be typed straight away, but
                    // only while nothing else holds it: taking it back every
                    // frame would fight the buttons below.
                    if let Some(f) = focus
                        && ui.memory(|m| m.focused().is_none())
                    {
                        f.request_focus();
                    }
                    match &edit.err {
                        Some(e) => lamp(ui, false, e),
                        None if edit.host.trim().is_empty() => lamp(ui, false, "no address"),
                        None => lamp(
                            ui,
                            true,
                            &format!("{} at {}", edit.kind.label(), edit.host.trim()),
                        ),
                    }
                });
                footer(ui, |ui| {
                    if ui.button("ADD").clicked() {
                        add = true;
                    }
                    if ui.button(crate::i18n::t("ui.close")).clicked() {
                        close = true;
                    }
                });
            });
        if r.should_close() {
            close = true;
        }
        if add {
            match self.add_remote(ctx, &edit) {
                Ok(()) => close = true,
                Err(e) => edit.err = Some(e),
            }
        }
        if !close {
            self.remote = Some(edit);
        }
    }

    /// Register the server, list it, and tune to it.
    ///
    /// The server is asked what it is streaming before it is kept, because a
    /// remote radio that does not answer is an entry in a list with nothing
    /// behind it, and the operator finds out at the point of adding rather
    /// than later when the spectrum stays empty.
    fn add_remote(
        &mut self,
        ctx: &egui::Context,
        edit: &RemoteEdit,
    ) -> std::result::Result<(), String> {
        match edit.kind {
            RemoteKind::IqStream => {
                iqnet::probe(&edit.host).map_err(|e| e.to_string())?;
            }
        }
        let addr = crate::devices::add_stream(&edit.host, &edit.label)
            .ok_or_else(|| "expected host or host:port".to_string())?;
        self.devices = crate::devices::list();
        let found = self
            .devices
            .iter()
            .find(|d| d.addr.as_deref() == Some(addr.as_str()))
            .cloned()
            .ok_or_else(|| format!("{addr} did not answer"))?;
        self.select_device(ctx, found);
        Ok(())
    }

    /// Ask for a capture and make it the receiver.
    ///
    /// The file dialog is the whole interface: which recording to replay is a
    /// choice somebody makes once, and a folder scanned for candidates would
    /// list every capture ever written next to the radios that are actually
    /// plugged in. The filename has to carry the sample rate and the format,
    /// because a guessed rate rescales every pulse width downstream and the
    /// receiver then decodes nothing for a reason nobody can see.
    ///
    /// The dialog runs on a thread of its own. Asking for it on the one that
    /// paints stops the receiver painting for as long as it is open, and a
    /// window that does not paint is a window the compositor puts a "not
    /// responding" dialog over.
    pub(super) fn open_capture(&mut self, ctx: &egui::Context) {
        if self.picking.is_some() {
            return;
        }
        let start = crate::chain::default_capture_dir();
        let _ = std::fs::create_dir_all(&start);
        let ctx = ctx.clone();
        self.picking = Some(poll_promise::Promise::spawn_thread("open capture", move || {
            let picked = rfd::FileDialog::new()
                .set_title("Replay a capture")
                .set_directory(&start)
                .add_filter("IQ captures", &["cu8", "cs8", "cs16", "cf32", "data", "sigmf-data"])
                .pick_file();
            // Nothing is drawing while the dialog is up, so the frame that
            // reads this has to be asked for.
            ctx.request_repaint();
            picked
        }));
    }

    /// Take the capture the dialog came back with, once it has.
    pub(super) fn poll_capture(&mut self, ctx: &egui::Context) {
        if self.picking.as_ref().is_none_or(|p| p.ready().is_none()) {
            return;
        }
        let Some(path) = self.picking.take().and_then(|p| p.block_and_take()) else {
            return;
        };
        let Some(c) = crate::devices::add_capture(path.clone()) else {
            self.err = Some(format!(
                "{}: cannot tell its sample rate and format. Name it like \
                 <what>_<centre>_<rate>.<format>, e.g. bench_433.92M_250k.cu8",
                path.display()
            ));
            self.err_at = Some(std::time::Instant::now());
            return;
        };
        self.devices = crate::devices::list();
        if let Some(e) = self.devices.iter().find(|d| d.path.as_deref() == Some(c.path.as_path())) {
            let e = e.clone();
            self.select_device(ctx, e);
        }
    }

    /// Build the device list again, keeping the chosen radio if it is still
    /// there. Connects only when nothing was chosen, so a rescan cannot pull
    /// the receiver off the radio it is running.
    pub(super) fn rescan(&mut self, ctx: &egui::Context) {
        self.devices = crate::devices::list();
        if self.device.as_ref().is_some_and(|c| !self.devices.iter().any(|d| d.label == c.label)) {
            self.device = None;
            self.radio = None;
        }
        if self.device.is_none() {
            self.device = self.devices.first().cloned();
            if self.device.is_some() {
                self.connect(ctx);
            }
        }
    }

    /// Separate from the spectrum and waterfall settings because it is a
    /// different kind of thing: those change what you see, these change what
    /// the receiver does, and getting them wrong costs sensitivity or
    /// intermodulation rather than a prettier display.
    fn radio_settings(&mut self, ui: &mut egui::Ui) {
        let Some(radio) = self.radio.as_ref() else {
            section(ui, "radio", "", |ui| lamp(ui, false, "no radio running"));
            return;
        };
        let controls = radio.status.radio();
        // Every control here writes the one record and applies it at the end,
        // which is the same route a restore and a reset take: there is no
        // second way to set the radio that could disagree with the first.
        let mut changed = false;

        section(ui, "gain", "each stage of the front end, in order from the aerial", |ui| {
            if controls.stages.is_empty() {
                lamp(ui, true, "this device has no adjustable stages");
            }
            for (stage, mode) in &controls.stages {
                let auto = *mode == GainMode::Auto;
                let mut db = match mode {
                    GainMode::Auto => *stage.range.start(),
                    GainMode::Manual(v) => *v,
                };
                let lo = *stage.range.start();
                let hi = *stage.range.end();
                let steps = if !stage.values.is_empty() {
                    format!("{} steps, {lo:.0} to {hi:.0} dB", stage.values.len())
                } else if stage.step > 0.0 {
                    format!("{:.0} dB steps, {lo:.0} to {hi:.0} dB", stage.step)
                } else {
                    format!("{lo:.0} to {hi:.0} dB")
                };
                // The short name in the column, the long one behind the "?"
                // with the steps: "LNA (sets noise figure)" does not fit a
                // legend and does not need to.
                row_help(ui, &stage.name, &format!("{}. {steps}.", stage.label), |ui| {
                    // Snapped as it is dragged, because the hardware does it
                    // anyway: a slider that glides between values the tuner
                    // cannot reach shows a number the receiver is not using.
                    wide_slider(ui);
                    let slider = egui::Slider::new(&mut db, lo..=hi).show_value(false);
                    if ui.add_enabled(!auto, slider).changed() {
                        let want = stage.quantise(db);
                        self.radio_settings.set_gain(&stage.name, GainMode::Manual(want));
                        changed = true;
                    }
                    // Under AUTO the number is the hardware's business and
                    // showing a stale one invites the operator to believe it.
                    let text = if auto { "auto".to_string() } else { format!("{db:.1} dB") };
                    theme::Line::new().set(text).size(11.0).show(ui);
                    if stage.auto {
                        let mut on = auto;
                        if ui.checkbox(&mut on, "auto").changed() {
                            let mode = if on { GainMode::Auto } else { GainMode::Manual(db) };
                            self.radio_settings.set_gain(&stage.name, mode);
                            changed = true;
                        }
                    }
                });
            }
            // The transmit gain, which is one number for the radio: a
            // channel's own trim is added to it when that channel is keyed.
            // With the stages because it is one, and last because it is the
            // one that radiates.
            if let Some(stage) = controls.tx_stages.iter().find(|s| s.name == "txvga") {
                let mut db = self.radio_settings.tx_gain_db;
                let (lo, hi) = (*stage.range.start(), *stage.range.end());
                row_help(
                    ui,
                    "transmit",
                    "What every keyed channel transmits at, before its own trim. Start at \
                     the bottom and into a dummy load.",
                    |ui| {
                        wide_slider(ui);
                        if ui.add(egui::Slider::new(&mut db, lo..=hi).show_value(false)).changed() {
                            self.radio_settings.tx_gain_db = stage.quantise(db);
                            changed = true;
                        }
                        theme::Line::new().set(format!("{db:.0} dB")).size(11.0).show(ui);
                    },
                );
            }
            for c in &controls.choices {
                row_help(ui, &c.label, &c.help, |ui| {
                    let mut picked = c.selected.clone();
                    let opts = c.options.iter().map(|o| (o.clone(), o.clone()));
                    if choice(ui, ("radio-choice", &c.name), &mut picked, opts) {
                        self.radio_settings.set_choice(&c.name, &picked);
                        changed = true;
                    }
                });
            }
            for t in &controls.toggles {
                let mut on = t.on;
                if switch(ui, &t.label, &mut on, "", &t.help) {
                    self.radio_settings.set_toggle(&t.name, on);
                    changed = true;
                }
            }
        });
        ui.add_space(8.0);

        section(ui, "tuning", "what this radio is off by, saved against it", |ui| {
            let ppm_help = "The reference oscillator is a few tens of parts per million out \
                            on a cheap dongle, which is a kilohertz or two at 145 MHz and \
                            rather more higher up. Tune a known carrier and correct until it \
                            sits on its nominal frequency.";
            row_help(ui, "correction", ppm_help, |ui| {
                let mut ppm = self.radio_settings.ppm;
                let drag = egui::DragValue::new(&mut ppm).speed(0.5).range(-200.0..=200.0);
                if ui.add(drag.suffix(" ppm")).changed() {
                    self.set_ppm(ppm);
                    changed = true;
                }
            });
            let offset_help = "Added to the tuner's frequency to get the dial's, for a \
                               converter on the cable. 9750 for a satellite LNB on its low \
                               band, -125 for an HF upconverter.";
            row_help(ui, "offset", offset_help, |ui| {
                let mut mhz = self.radio_settings.offset / 1e6;
                let drag = egui::DragValue::new(&mut mhz)
                    .speed(1.0)
                    .range(-10_000.0..=100_000.0)
                    .max_decimals(6);
                if ui.add(drag.suffix(" MHz")).changed() {
                    self.set_offset(mhz * 1e6);
                    changed = true;
                }
            });
            let mut dc = self.dc_block;
            let dc_help = "A direct conversion receiver leaks its own local oscillator into \
                           the middle of the span, where it looks exactly like a carrier on \
                           the frequency you are tuned to. This measures the offset and \
                           subtracts it.";
            if switch(ui, "centre spur", &mut dc, "remove", dc_help) {
                self.dc_block = dc;
                self.send(Cmd::DcBlock(dc));
            }
        });
        ui.add_space(8.0);

        self.raw_capture(ui);

        if changed {
            self.apply_radio_settings();
        }
    }

    /// Writing the span to disk exactly as it arrives.
    ///
    /// Here rather than with the packet log, where it used to be. Both write
    /// to disk and that is all they have in common: the log holds what the
    /// demodulators made of a burst, and this holds the samples themselves,
    /// for the transmission nothing made anything of. Two capture switches on
    /// one panel meant picking the wrong one and finding out an hour later.
    fn raw_capture(&mut self, ui: &mut egui::Ui) {
        let (cap_on, cap_bytes, cap_folder, cap_full, cap_file) = match &self.radio {
            Some(r) => {
                use std::sync::atomic::Ordering;
                (
                    r.status.capture_on.load(Ordering::Relaxed),
                    r.status.capture_bytes.load(Ordering::Relaxed),
                    r.status.capture_folder.load(Ordering::Relaxed),
                    r.status.capture_full.load(Ordering::Relaxed),
                    r.status.capture_file.lock().clone(),
                )
            }
            None => (false, 0, 0, false, None),
        };
        section(ui, "raw capture", "the whole span to one file, as it arrives", |ui| {
            let mut on = cap_on;
            let why = "The recording to make when the receiver shows a transmission and \
                       reads nothing from it: replaying the file puts the same samples \
                       through the same graph, so a decoder can be changed and tried again.";
            if switch(ui, "capture", &mut on, "the raw span", why) {
                self.set_capture(on);
            }
            let cap_help = "What the whole folder may take. Nothing here is deleted: a \
                            capture is evidence of a signal that may not come again, so \
                            writing stops instead.";
            row_help(ui, "limit", cap_help, |ui| {
                let mut cap = self.capture_cap_mb;
                let opts = [Some(1024u64), Some(4096), Some(16_384), Some(65_536), None];
                if choice(ui, "capture_cap", &mut cap, opts.map(|o| (o, size_label(o)))) {
                    self.capture_cap_mb = cap;
                    self.send(Cmd::CaptureCap(cap.map(|mb| mb << 20).unwrap_or(0)));
                }
            });
            // Where the files are, and a way into it. A capture is made to
            // be replayed, trimmed or sent somewhere, all of which happen
            // outside this program, and a path that can only be read off the
            // screen and typed again is a path nobody uses.
            let dir = crate::chain::default_capture_dir();
            row(ui, "folder", |ui| {
                if ui.small_button("OPEN").on_hover_text(dir.display().to_string()).clicked() {
                    // Created first: the folder does not exist until the
                    // first capture is written, and a file manager handed a
                    // missing path either opens nothing or opens somewhere
                    // else.
                    let _ = std::fs::create_dir_all(&dir);
                    ui.ctx().open_url(egui::OpenUrl::new_tab(file_url(&dir)));
                }
                theme::Line::new().value(dir.display().to_string()).size(11.0).elided(ui);
            });
            ui.add_space(4.0);
            // The folder first: it is the number the limit above is about,
            // and showing only the file being written made a folder of two
            // gigabytes read as seventy megabytes.
            reading(ui, "folder holds", human_bytes(cap_folder));
            if cap_bytes > 0 {
                reading(ui, "this file", human_bytes(cap_bytes));
            }
            if let Some(f) = &cap_file {
                reading(ui, "file", f);
            }
            if cap_full {
                lamp(ui, false, "stopped: the folder is at its limit");
            } else if cap_on {
                lamp(ui, true, "writing");
            }
        });
    }
}

/// A path as a `file:` URL, which is what the browser handler opening it
/// expects. Only the characters that would end the path are escaped: a home
/// directory with a space in it is common enough to matter, and a full
/// encoder for one link is a dependency for nothing.
fn file_url(path: &std::path::Path) -> String {
    let mut s = String::from("file://");
    for c in path.display().to_string().chars() {
        match c {
            ' ' => s.push_str("%20"),
            '%' => s.push_str("%25"),
            '#' => s.push_str("%23"),
            '?' => s.push_str("%3F"),
            _ => s.push(c),
        }
    }
    s
}

/// A folder limit as it is offered and shown, in whichever unit reads.
fn size_label(mb: Option<u64>) -> String {
    match mb {
        Some(mb) if mb >= 1024 => format!("{} GB", mb / 1024),
        Some(mb) => format!("{mb} MB"),
        None => "no limit".into(),
    }
}

/// A radio reached over the network, as the dialog that creates one asks for
/// it.
///
/// The protocol is a choice rather than an assumption: rtl_tcp and airspy's
/// own network server are the same shape of thing, and the dialog is where
/// they will be offered.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RemoteKind {
    IqStream,
}

impl RemoteKind {
    pub const ALL: &'static [RemoteKind] = &[RemoteKind::IqStream];

    fn label(self) -> &'static str {
        match self {
            Self::IqStream => "iqstream",
        }
    }

    fn help(self) -> &'static str {
        match self {
            Self::IqStream => {
                "One tuner shared with many readers, so a dongle already feeding a decoder \
                 elsewhere can still be listened to here. The frequency and the span belong \
                 to whoever owns that tuner and cannot be changed from this end."
            }
        }
    }

    fn placeholder(self) -> &'static str {
        match self {
            Self::IqStream => "host, or host:port (1234)",
        }
    }
}

#[derive(Clone)]
pub struct RemoteEdit {
    kind: RemoteKind,
    host: String,
    /// What to call it in the radio list. Optional, and worth having: an
    /// address says which machine and nothing about which aerial.
    label: String,
    /// Why the last attempt was refused, kept beside the field it belongs to
    /// rather than in the status line under the dial.
    err: Option<String>,
}

impl Default for RemoteEdit {
    fn default() -> Self {
        Self { kind: RemoteKind::IqStream, host: String::new(), label: String::new(), err: None }
    }
}

/// A picker over the sound devices a host reports, with the system default at
/// the top as an empty name.
///
/// The default is worth having as a choice rather than as an absence: a host
/// that gains a device follows the default, and an operator who picked one
/// deliberately should keep it. Returns whether the selection changed.
fn device_combo(ui: &mut egui::Ui, id: &str, current: &mut String, names: Vec<String>) -> bool {
    let mut changed = false;
    let shown = if current.is_empty() { "System default".to_string() } else { current.clone() };
    egui::ComboBox::from_id_salt(id).selected_text(shown).width(ui.available_width()).show_ui(
        ui,
        |ui| {
            if ui.selectable_label(current.is_empty(), "System default").clicked()
                && !current.is_empty()
            {
                current.clear();
                changed = true;
            }
            for n in names {
                let on = *current == n;
                if ui.selectable_label(on, &n).clicked() && !on {
                    *current = n;
                    changed = true;
                }
            }
        },
    );
    changed
}

/// The host of an address, which is what somebody reading a status line
/// wants to know about a server: "api.openai.com", not the whole URL.
fn host_of(url: &str) -> String {
    let rest = url.trim().split("://").nth(1).unwrap_or(url.trim());
    let host = rest.split('/').next().unwrap_or(rest);
    match host {
        "" => "nowhere".into(),
        h => h.to_string(),
    }
}

/// A slider that takes the control column rather than egui's hundred
/// points, leaving room for the reading and the "?" after it.
fn wide_slider(ui: &mut egui::Ui) {
    ui.spacing_mut().slider_width = (ui.available_width() - 120.0).max(80.0);
}

/// A field that becomes a picker once the server has said what it offers.
///
/// What is typed stays typed: a model the listing has not caught up with,
/// or a server that lists nothing, is still reachable by name, which is
/// why the picker keeps whatever is set even when it is not on the list.
fn pick_or_type(ui: &mut egui::Ui, id: &str, value: &mut String, offered: Vec<String>, hint: &str) {
    if offered.is_empty() {
        field(ui, value, hint);
        return;
    }
    let mut options: Vec<(String, String)> =
        offered.iter().map(|o| (o.clone(), o.clone())).collect();
    if !value.trim().is_empty() && !offered.iter().any(|o| o == value.trim()) {
        options.insert(0, (value.clone(), format!("{} (not listed)", value.trim())));
    }
    if value.trim().is_empty() {
        options.insert(0, (String::new(), "choose".into()));
    }
    choice(ui, id, value, options);
}
