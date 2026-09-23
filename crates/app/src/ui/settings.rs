//! The settings dialogs, and the one that creates a remote radio.
//!
//! Each is a leaf: it reads and writes the receiver's state and draws nothing
//! anybody else depends on, which is what makes them separable from the panes.

use super::*;
use crate::agent::config::{Reading, Speech};
use crate::ui::widgets::{
    card, choice, field, field_then, footer, hint, lamp, prose, reading, row, row_help, secret,
    section, switch, tabs,
};

/// Ask every USB serial port whether a sub-ghz-modem is on it.
///
/// Off the frame, because a probe waits two seconds for a board that resets
/// when its port is opened and there can be half a dozen ports.
fn scan_for_modems() -> crate::ui::state::ModemScan {
    let (tx, done) = std::sync::mpsc::channel();
    std::thread::Builder::new()
        .name("modem-scan".into())
        .spawn(move || {
            let _ = tx.send(gps::modem::discover());
        })
        .ok();
    crate::ui::state::ModemScan { done, found: None }
}

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

/// A scroll area's height as a share of the screen, so a modal with two
/// growing lists in it still fits a laptop panel.
fn share_of_screen(ui: &egui::Ui, share: f32, least: f32, most: f32) -> f32 {
    (ui.ctx().input(|i| i.content_rect().height()) * share).clamp(least, most)
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
            Settings::BandWalk => "Band walk",
            Settings::Memory => "Memory bank",
            Settings::Data => crate::i18n::t("settings.data"),
            Settings::Calls => "Calls",
            Settings::Agent => "Agent",
            Settings::App => crate::i18n::t("settings.title"),
        };
        let r = egui::containers::Modal::new(egui::Id::new(title))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(match which {
                    Settings::Scanners | Settings::BandWalk | Settings::Memory | Settings::Data => {
                        560.0
                    }
                    _ => 520.0,
                });
                modal_title(ui, title);
                match which {
                    Settings::Spectrum => self.scope_settings(ui, true),
                    Settings::Waterfall => self.scope_settings(ui, false),
                    Settings::Radio => self.radio_settings(ui),
                    Settings::PacketLog => self.packet_log_settings(ui),
                    Settings::Scanners => self.scanner_settings(ui),
                    Settings::BandWalk => self.band_walk(ui),
                    Settings::Memory => self.memory_pane(ui),
                    Settings::Data => self.data_settings(ui),
                    Settings::Calls => self.calls_settings(ui),
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
                            if let Some(tone) = s.tone {
                                line = line.gap(10.0).set(tone.label());
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
        section(ui, "lists", "read a list somebody else wrote, or take this one away", |ui| {
            ui.horizontal(|ui| {
                let busy = self.memory_io.is_some();
                if ui
                    .add_enabled(!busy, egui::Button::new("IMPORT"))
                    .on_hover_text(
                        "a Chirp or plain CSV, a PortaPack Freqman .TXT, an SDR# \
                             frequencies.xml, or a waveshark channels file",
                    )
                    .clicked()
                {
                    self.import_channels(ui.ctx());
                }
                if ui
                    .add_enabled(!busy && !self.memory.list.is_empty(), egui::Button::new("EXPORT"))
                    .on_hover_text("the whole bank as a Chirp CSV")
                    .clicked()
                {
                    self.export_channels(ui.ctx());
                }
            });
            if !self.memory_note.is_empty() {
                let ok = !self.memory_note.starts_with("nothing");
                lamp(ui, ok, &self.memory_note.clone());
            }
        });
        if let Some(p) = crate::memory::Memory::path() {
            theme::Line::new().note(p.display().to_string()).size(10.0).elided(ui);
        }
    }

    /// Read a frequency list, whoever wrote it. The group a channel goes into
    /// is the one its own list named, or the file's name where it named none.
    fn import_channels(&mut self, ctx: &egui::Context) {
        if self.memory_io.is_some() {
            return;
        }
        let ctx = ctx.clone();
        self.memory_io = Some(poll_promise::Promise::spawn_thread("import channels", move || {
            let picked = rfd::FileDialog::new()
                .set_title("Import a frequency list")
                .add_filter("Frequency lists", &["csv", "txt", "TXT", "xml", "channels"])
                .add_filter("Anything", &["*"])
                .pick_file();
            ctx.request_repaint();
            let Some(path) = picked else { return state::ListIo::Said(String::new()) };
            let text = match std::fs::read_to_string(&path) {
                Ok(t) => t,
                Err(e) => return state::ListIo::Said(format!("{}: {e}", path.display())),
            };
            let stem =
                path.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
            let (format, read) = crate::memory::formats::read(&text, &stem);
            state::ListIo::Read(Box::new(read), format)
        }));
    }

    /// The bank as a Chirp CSV, which is the export a handheld's programming
    /// software will take.
    fn export_channels(&mut self, ctx: &egui::Context) {
        if self.memory_io.is_some() {
            return;
        }
        let text = crate::memory::formats::write_csv(&self.memory);
        let count = self.memory.list.len();
        let ctx = ctx.clone();
        self.memory_io = Some(poll_promise::Promise::spawn_thread("export channels", move || {
            let picked = rfd::FileDialog::new()
                .set_title("Export the memory bank")
                .set_file_name(crate::memory::formats::export_name())
                .add_filter("CSV", &["csv"])
                .save_file();
            ctx.request_repaint();
            let Some(path) = picked else { return state::ListIo::Said(String::new()) };
            state::ListIo::Said(match std::fs::write(&path, text) {
                Ok(()) => format!("{count} channels written to {}", path.display()),
                Err(e) => format!("nothing written: {}: {e}", path.display()),
            })
        }));
    }

    /// Take the list the dialog came back with, once it has.
    pub(super) fn poll_memory_io(&mut self) {
        if self.memory_io.as_ref().is_none_or(|p| p.ready().is_none()) {
            return;
        }
        let Some(io) = self.memory_io.take().map(|p| p.block_and_take()) else {
            return;
        };
        match io {
            state::ListIo::Said(s) => self.memory_note = s,
            state::ListIo::Read(read, format) => {
                let note = read.note(format);
                let added = self.memory.merge(read.list);
                self.memory_note = match added {
                    0 => format!("nothing new: {note}"),
                    n => format!("{note}, {n} new"),
                };
                let _ = self.memory.save();
            }
        }
    }

    /// The scope's own panels, and whatever they asked for afterwards.
    fn scope_settings(&mut self, ui: &mut egui::Ui, spectrum: bool) {
        let mut pane = scope_settings::ScopeSettings {
            st: &mut self.scope,
            settings: self.settings.clone(),
            rate: self.rate,
            heat: self.radio.as_ref().and_then(|r| r.status.heatmap.lock().clone()),
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
                // Coloured and scaled as the waterfall is showing it, because
                // what somebody means by "export this" is what they can see.
                scope_settings::Action::ExportHeatmap => self.send(Cmd::ExportHeatmap {
                    ramp: self.scope.ramp,
                    floor: self.scope.floor,
                    ceil: self.scope.ceil - self.scope.wf_top_offset,
                }),
            }
        }
    }

    /// The walk over a band: where the dial goes when it is let off the span,
    /// and what it heard on the way.
    fn band_walk(&mut self, ui: &mut egui::Ui) {
        let status = self.radio.as_ref().and_then(|r| r.status.band_scan.lock().clone());
        let (mut on, mut lo, mut hi, mut step, mut dwell, mut hold) = self.setting(|s| {
            (s.scan_on, s.scan_lo_mhz, s.scan_hi_mhz, s.scan_step_khz, s.scan_dwell_s, s.scan_hold)
        });
        let (mut locks, mut sparse, mut linger) =
            self.setting(|s| (s.scan_locks, s.scan_sparse, s.scan_linger_s));
        let was = (on, lo, hi, step, dwell, hold, locks, sparse, linger);
        let mut acts: Vec<Cmd> = Vec::new();
        let mut tune_to = None;

        section(ui, "the dial", "step it past the span until something answers", |ui| {
            row_help(
                ui,
                "band",
                "The edges the dial walks between. Each step tunes a span, so a band wider \
                 than the span takes several, and the walk starts again at the bottom when \
                 it reaches the top.",
                |ui| {
                    ui.add(
                        egui::DragValue::new(&mut lo).speed(0.1).range(0.0..=6000.0).suffix(" MHz"),
                    );
                    theme::Line::new().legend("to").size(11.0).show(ui);
                    ui.add(
                        egui::DragValue::new(&mut hi).speed(0.1).range(0.0..=6000.0).suffix(" MHz"),
                    );
                },
            );
            row_help(
                ui,
                "step",
                "How far the dial moves each time. Zero steps by the span the radio is \
                 sampling, which covers the band without leaving anything untuned.",
                |ui| {
                    ui.add(
                        egui::DragValue::new(&mut step)
                            .speed(10.0)
                            .range(0.0..=100_000.0)
                            .suffix(" kHz"),
                    );
                },
            );
            row_help(
                ui,
                "dwell",
                "How long each step is listened to before the next one. A burst nobody \
                 transmitted during the dwell is a step that heard nothing.",
                |ui| {
                    ui.add(
                        egui::DragValue::new(&mut dwell).speed(0.1).range(0.1..=60.0).suffix(" s"),
                    );
                },
            );
            row_help(
                ui,
                "lock",
                "How many packets a step has to carry before the walk calls it busy. \
                 Counted together they can arrive any time during the dwell, which survives \
                 bad reception; counted in a row a gap starts the count again, which is \
                 faster and drops a signal heard in pieces.",
                |ui| {
                    ui.add(
                        egui::DragValue::new(&mut locks)
                            .speed(0.1)
                            .range(1.0..=16.0)
                            .fixed_decimals(0)
                            .prefix("heard "),
                    );
                    choice(
                        ui,
                        "scan_lock",
                        &mut sparse,
                        [(true, "together".to_string()), (false, "one after another".to_string())],
                    );
                },
            );
            row_help(
                ui,
                "linger",
                "How long a logging walk stays on a step it heard something on. Zero notes \
                 it and moves on, which maps a band fastest. A positive number stays that \
                 long. A negative one stays until that many seconds pass with nothing \
                 further, each burst starting the count again, which is the only setting \
                 that follows a conversation.",
                |ui| {
                    ui.add(
                        egui::DragValue::new(&mut linger)
                            .speed(0.1)
                            .range(-60.0..=60.0)
                            .suffix(" s"),
                    );
                },
            );
            row_help(
                ui,
                "on a hit",
                "Whether the walk stays on a step that turned something up, so it can be \
                 listened to, or writes it down and carries on.",
                |ui| {
                    choice(
                        ui,
                        "scan_on_hit",
                        &mut hold,
                        [
                            (true, "hold there".to_string()),
                            (false, "log it and move on".to_string()),
                        ],
                    );
                },
            );
            switch(
                ui,
                "walk",
                &mut on,
                "step the dial",
                "The dial moves on its own while this is on, so whatever you were listening \
                 to is left behind.",
            );
            match &status {
                Some(st) if st.running => {
                    let at = st
                        .center_hz
                        .map(|c| format!("{:.4} MHz", c / 1e6))
                        .unwrap_or_else(|| "starting".into());
                    match st.holding {
                        true => {
                            lamp(ui, true, &format!("held at {at}"));
                            if ui.button("RESUME").clicked() {
                                acts.push(Cmd::StageParam(
                                    crate::chain::derived::SCAN,
                                    "holding".into(),
                                    pipeline::ParamValue::Bool(false),
                                ));
                            }
                        }
                        false => lamp(
                            ui,
                            true,
                            &format!("at {at}, {} steps taken, {} to a pass", st.steps, st.stops),
                        ),
                    }
                }
                Some(_) => lamp(ui, false, "the dial stays where you put it"),
                None => lamp(ui, false, "no receiver running, so nothing is walking"),
            }
        });

        if let Some(st) = status.as_ref().filter(|st| !st.found.is_empty()) {
            ui.add_space(6.0);
            let w = ui.available_width();
            let tall = share_of_screen(ui, 0.45, 160.0, 560.0);
            egui::ScrollArea::vertical().max_height(tall).id_salt("walkhits").show(ui, |ui| {
                ui.set_max_width(w);
                for f in &st.found {
                    let mhz = f.center_hz as f64 / 1e6;
                    card(
                        ui,
                        Some(theme::TRACE),
                        |ui| {
                            theme::Line::new()
                                .value(format!("{mhz:.4} MHz"))
                                .size(12.0)
                                .gap(12.0)
                                .heard(f.protocol.clone().unwrap_or_else(|| "unclaimed".into()))
                                .size(11.0)
                                .show(ui);
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    if ui.button("IGNORE").clicked() {
                                        let mut list: Vec<String> =
                                            st.ignore.iter().map(nodes::Key::label).collect();
                                        list.push(f.key.label());
                                        acts.push(Cmd::StageParam(
                                            crate::chain::derived::SCAN,
                                            "ignore".into(),
                                            pipeline::ParamValue::Text(list.join(",")),
                                        ));
                                    }
                                    if ui.button("TUNE").clicked() {
                                        tune_to = Some(mhz);
                                    }
                                },
                            );
                        },
                        |ui| {
                            theme::Line::new()
                                .legend("heard")
                                .value(f.heard.to_string())
                                .size(11.0)
                                .gap(16.0)
                                .legend("snr")
                                .value(format!("{:.0} dB", f.snr_db))
                                .size(11.0)
                                .gap(16.0)
                                .legend("as")
                                .value(f.key.label())
                                .size(11.0)
                                .show(ui);
                        },
                    );
                    ui.add_space(4.0);
                }
            });
        }

        if (on, lo, hi, step, dwell, hold, locks, sparse, linger) != was {
            self.settings.edit(|s| {
                s.scan_on = on;
                s.scan_lo_mhz = lo;
                s.scan_hi_mhz = hi;
                s.scan_step_khz = step;
                s.scan_dwell_s = dwell;
                s.scan_hold = hold;
                s.scan_locks = locks;
                s.scan_sparse = sparse;
                s.scan_linger_s = linger;
            });
        }
        for c in acts {
            self.send(c);
        }
        if let Some(mhz) = tune_to {
            self.retune(mhz * 1e6);
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
        let active: Vec<String> = table
            .active(crate::scanners::Span::whole(center, rate))
            .into_iter()
            .map(|s| s.name.clone())
            .collect();

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
        // The walk is its own dialog: it has a form and a list of finds, and
        // both of them beside the table left nothing room enough to read.
        let walk = self.radio.as_ref().and_then(|r| r.status.band_scan.lock().clone());
        let mut open_walk = false;
        section(ui, "band walk", "step the dial past the span until something answers", |ui| {
            ui.horizontal(|ui| {
                match walk.as_ref().filter(|st| st.running) {
                    Some(st) => {
                        let at = st
                            .center_hz
                            .map(|c| format!("{:.4} MHz", c / 1e6))
                            .unwrap_or_else(|| "starting".into());
                        let what = match st.holding {
                            true => format!("held at {at}"),
                            false => format!("at {at}, {} found", st.found.len()),
                        };
                        lamp(ui, true, &what);
                    }
                    None => lamp(ui, false, "the dial stays where you put it"),
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    open_walk = ui.button("WALK").clicked();
                });
            });
        });
        if open_walk {
            self.scanner_edit = Some(rows);
            self.open = Some(Settings::BandWalk);
            return;
        }
        ui.add_space(8.0);

        let mut remove = None;
        let mut tune_to = None;
        let w = ui.available_width();
        let tall = share_of_screen(ui, 0.34, 200.0, 420.0);
        egui::ScrollArea::vertical().max_height(tall).id_salt("scanrows").show(ui, |ui| {
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
            let mut on = self.setting(|s| s.packet_log_on);
            let log_help = "Timings and frames as demodulated, replayable.";
            if switch(ui, "write", &mut on, "every packet to disk", log_help) {
                self.settings.edit(|s| s.packet_log_on = on);
            }
            // What the list shows, rather than what the receiver does. An
            // unrecognised burst is still reported, logged and replayable
            // with this off; it is only kept out of the table.
            let mut unknown = self.setting(|s| s.list_unknown);
            let unknown_help = "Bursts that decoded to no known protocol. They are the point \
                                of scanning an unfamiliar band, and on a noisy one they bury \
                                the decodes.";
            if switch(ui, "list", &mut unknown, "unrecognised bursts too", unknown_help) {
                self.settings.edit(|s| s.list_unknown = unknown);
            }
            row_help(ui, "folder", "Where the files go. Enter or SET applies it.", |ui| {
                let mut set = false;
                let r = field_then(ui, &mut self.log_dir_edit, "where the files go", 44.0, |ui| {
                    set = ui.small_button("SET").clicked();
                });
                let typed = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                if (typed || set) && !self.log_dir_edit.trim().is_empty() {
                    let dir = self.log_dir_edit.trim().to_string();
                    self.settings.edit(|s| s.log_dir = dir);
                }
            });
            let cap_help = "What the whole folder may take. The oldest days are deleted to \
                            keep it under, so the log rolls rather than stopping.";
            row_help(ui, "limit", cap_help, |ui| {
                let mut cap = self.setting(|s| s.log_cap_mb);
                let opts = [Some(512u64), Some(2048), Some(8192), Some(32_768), None];
                if choice(ui, "log_cap", &mut cap, opts.map(|o| (o, size_label(o)))) {
                    self.settings.edit(|s| s.log_cap_mb = cap);
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

        let status = self.radio.as_ref().map(|r| r.status.feeds.lock().clone()).unwrap_or_default();
        let mut remove = None;
        let feeds = self.setting(|s| s.feeds.clone());
        section(ui, "feeds", "packets from another receiver, over TCP", |ui| {
            for (i, f) in feeds.iter().enumerate() {
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
                        Some(spec) if !feeds.contains(&spec) => {
                            self.settings.edit(|s| s.feeds.push(spec));
                            self.feed_host.clear();
                        }
                        Some(_) => self.err = Some("that feed is already attached".into()),
                        None => self.err = Some("expected host or host:port".into()),
                    }
                }
            });
        });
        if let Some(i) = remove {
            self.settings.edit(|s| {
                s.feeds.remove(i);
            });
        }
    }

    /// What is kept of the speech heard, beside the list that plays it back.
    ///
    /// Its own dialog rather than a card under the packet log: a recording is
    /// a decision about the same disc, but nobody looking for what happened
    /// to an over goes to the packet list to find it.
    fn calls_settings(&mut self, ui: &mut egui::Ui) {
        let rec = self.radio.as_ref().and_then(|r| r.status.recorder.lock().clone());
        section(ui, "calls", "every over heard, kept as Opus", |ui| {
            let mut on = self.setting(|s| s.calls_on);
            let help = "Every transmission on a voice channel or a voice front end, as it \
                        was heard, before any fader: about 2 kB a second of speech. The \
                        Recordings table in the Calls view plays them back.";
            if switch(ui, "record", &mut on, "every over", help) {
                self.settings.edit(|s| s.calls_on = on);
            }
            row_help(ui, "folder", "Where the overs go. Enter or SET applies it.", |ui| {
                let mut set = false;
                let text = self
                    .calls_dir_edit
                    .get_or_insert_with(|| crate::calllog::calls_dir().display().to_string());
                let r = field_then(ui, text, "where the overs go", 44.0, |ui| {
                    set = ui.small_button("SET").clicked();
                });
                let typed = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                if (typed || set) && !text.trim().is_empty() {
                    let dir = text.trim().to_string();
                    self.settings.edit(|s| s.calls_dir = dir);
                }
            });
            let Some(rec) = rec else {
                lamp(ui, false, "no receiver running, so nothing is being kept");
                return;
            };
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
        self.reading_section(ui);
    }

    /// Speech into words, beside the recording it is read from.
    ///
    /// The model, what it runs on and how it is getting on are on the
    /// Transcript view, where the words are; this is the switch and the two
    /// choices, so somebody setting up recording does not have to go and
    /// find them.
    fn reading_section(&mut self, ui: &mut egui::Ui) {
        let engine = self.radio.as_ref().and_then(|r| r.status.transcriber.lock().clone());
        section(ui, "reading", "speech heard on the air, read back as words", |ui| {
            let mut on = self.setting(|s| s.transcribe_on);
            let help = "Every over the recorder keeps is read by a model here, and the \
                        words appear on the Transcript view and against the call.";
            if switch(ui, "read", &mut on, "what was said", help) {
                self.settings.edit(|s| s.transcribe_on = on);
            }
            let Some(e) = engine else {
                lamp(ui, false, "no receiver running, so the model is not loaded");
                return;
            };
            row_help(ui, "model", "Bigger reads better and slower.", |ui| {
                let mut id = e.model.clone();
                let opts = e.models.iter().map(|m| (m.id.clone(), m.label.clone()));
                if choice(ui, "calls-read-model", &mut id, opts) {
                    self.settings.edit(|s| s.transcribe_model = id.clone());
                }
            });
            row_help(ui, "run on", "Auto takes the fastest that will hold the weights.", |ui| {
                let mut device = e.device_choice.clone();
                let opts = e.devices.iter().cloned();
                if choice(ui, "calls-read-device", &mut device, opts) {
                    self.settings.edit(|s| s.transcribe_device = device.clone());
                }
            });
            match (&e.reading_on, on) {
                (Some(where_), _) => lamp(ui, true, where_),
                (None, true) => lamp(ui, true, &format!("{} on {}", e.label, e.device_choice)),
                (None, false) => lamp(ui, false, "off: nothing is read"),
            }
        });
    }

    /// The span served to the network, which is this receiver seen as a
    /// tuner by another one.
    ///
    /// Beside the TNC because it is the same decision: who on this network
    /// may read what this radio hears. Off until it is asked for, since it
    /// puts the whole span on the wire.
    fn iqstream_section(&mut self, ui: &mut egui::Ui) {
        let addr = self.setting(|s| s.iqstream_address());
        let server = addr.and_then(nodes::iqstream_nodes::running);
        section(ui, "iq server", "the span to another receiver, over IQStream", |ui| {
            let mut on = self.setting(|s| s.iqstream_on);
            let help = "Serves the samples this receiver is reading to anything speaking \
                        IQStream, which is how another WaveShark adds this radio as a \
                        remote tuner. The whole span goes out, so it costs bandwidth.";
            if switch(ui, "serve", &mut on, "the span from this machine", help) {
                self.settings.edit(|s| s.iqstream_on = on);
            }
            row_help(
                ui,
                "listen",
                "A port, or host:port. A port alone is every interface.",
                |ui| {
                    let mut text = self.setting(|s| s.iqstream_addr.clone());
                    if field(ui, &mut text, "1234, or 0.0.0.0:1234").changed() {
                        self.settings.edit(|s| s.iqstream_addr = text.clone());
                    }
                },
            );
            let mut tunable = self.setting(|s| s.iqstream_tunable);
            let tune_help = "There is one tuner, so a subscriber moving the dial moves it \
                             here too, and whatever is being listened to on this screen \
                             goes with it.";
            if switch(ui, "retuning", &mut tunable, "a subscriber may move the dial", tune_help) {
                self.settings.edit(|s| s.iqstream_tunable = tunable);
            }
            if let Some(server) = &server {
                let streams = server.streams();
                let subscribers: usize = streams.iter().map(|s| s.subscribers()).sum();
                reading(ui, "subscribers", subscribers.to_string());
                reading(ui, "streams", streams.len().to_string());
                if let Some(sent) = streams.iter().map(|s| s.blocks_sent()).max() {
                    reading(ui, "sent", format!("{sent} blocks"));
                }
            }
            match (on, addr, server.as_ref()) {
                (false, _, _) => lamp(ui, false, "off: nothing is served"),
                (true, None, _) => lamp(ui, false, "not a port or a host:port"),
                (true, _, Some(s)) => lamp(ui, true, &format!("listening on {}", s.addr())),
                (true, Some(a), None) => lamp(ui, false, &format!("{a} is not being served yet")),
            }
        });
    }

    /// The KISS TNC: where it listens, and what is connected to it.
    ///
    /// Off until it is asked for, since it opens a listening port and a
    /// client on it can key the transmitter.
    fn kiss_section(&mut self, ui: &mut egui::Ui) {
        let addr = self.setting(|s| s.kiss_address());
        let tnc = addr.and_then(nodes::kiss_nodes::running);
        section(ui, "tnc", "AX.25 to packet software here, over KISS", |ui| {
            let mut on = self.setting(|s| s.kiss_on);
            let help = "Serves every AX.25 frame heard on the packet band to anything \
                        speaking KISS, and keys the transmitter with what a client sends \
                        back. Direwolf, Xastir, APRSIS32 and pat all speak it.";
            if switch(ui, "serve", &mut on, "a KISS TNC on this machine", help) {
                self.settings.edit(|s| s.kiss_on = on);
            }
            row_help(ui, "listen", "A port, or host:port. A port alone is loopback.", |ui| {
                let mut text = self.setting(|s| s.kiss_addr.clone());
                if field(ui, &mut text, "8001, or 0.0.0.0:8001").changed() {
                    self.settings.edit(|s| s.kiss_addr = text.clone());
                }
            });
            if let Some(tnc) = &tnc {
                reading(ui, "clients", tnc.connected().to_string());
                reading(ui, "to clients", format!("{} frames", tnc.sent()));
                reading(ui, "from clients", format!("{} frames", tnc.received()));
                // Frames a client sent that no keyed channel took. Shown
                // rather than counted silently: a client talking into a
                // receiver with no transmit channel looks like a TNC that
                // works until nothing goes out.
                if tnc.dropped() > 0 {
                    reading(ui, "dropped", format!("{} frames", tnc.dropped()));
                }
            }
            match (on, addr, tnc.as_ref()) {
                (false, _, _) => lamp(ui, false, "off: nothing is served"),
                (true, None, _) => lamp(ui, false, "not a port or a host:port"),
                (true, _, Some(t)) => match (t.error(), t.bound()) {
                    (Some(e), _) => lamp(ui, false, &e),
                    (None, Some(b)) => lamp(ui, true, &format!("listening on {b}")),
                    (None, None) => lamp(ui, false, "not listening"),
                },
                (true, Some(a), None) => lamp(ui, false, &format!("{a} is not being served yet")),
            }
        });
    }

    /// Everything the radio itself can be set to.
    ///
    /// Where this receiver is, rather than what it is doing.
    ///
    /// One pane for the settings that are true of the installation and not of
    /// the session: they survive changing radio, they are asked once, and
    /// none of them belong under a cog on the spectrum.
    fn app_settings(&mut self, ui: &mut egui::Ui) {
        let mut tab = self.setup_tab;
        if tabs(ui, &mut tab, &[(SetupTab::General, "GENERAL"), (SetupTab::Network, "NETWORK")]) {
            self.setup_tab = tab;
        }
        match tab {
            SetupTab::General => self.setup_general(ui),
            SetupTab::Network => self.setup_network(ui),
        }
    }

    /// What this installation is: where it is, what it sounds through, and
    /// what it opens on.
    fn setup_general(&mut self, ui: &mut egui::Ui) {
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
                let mut code = self.setting(|s| s.country.clone());
                let opts = crate::locale::COUNTRIES
                    .iter()
                    .map(|c| (c.code.to_string(), c.name.to_string()));
                if choice(ui, "app-country", &mut code, opts)
                    && let Some(c) = crate::locale::by_code(&code)
                {
                    self.settings.edit(|s| s.country = c.code.to_string());
                    // A country decides the plan the first time and then
                    // stops having an opinion, so choosing one after
                    // overriding the plan puts the override back rather
                    // than leaving a mismatch nobody asked for.
                    crate::bands::set_plan(c.plan);
                    // The map has to open somewhere. A capital city is
                    // wrong by a couple of hundred miles, which is close
                    // enough to draw with and is replaced the moment a
                    // real position is typed in.
                    if self.setting(|s| s.location).is_none() {
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
                let mut out = self.setting(|s| s.audio_out.clone());
                if device_combo(ui, "app-audio-out", &mut out, audio::AudioPlayer::devices()) {
                    self.settings.edit(|s| s.audio_out = out);
                }
            });
            let mic = "What a keyed channel transmits. Held open while a channel is set to \
                       MIC, so the meter moves before you key.";
            row_help(ui, "microphone", mic, |ui| {
                let mut input = self.setting(|s| s.audio_in.clone());
                if device_combo(ui, "app-audio-in", &mut input, audio::AudioCapture::devices()) {
                    self.settings.edit(|s| s.audio_in = input);
                }
            });
        });
        ui.add_space(8.0);

        section(ui, "opens on", "the first view when the window comes up", |ui| {
            let mut on = self.setting(|s| s.dashboard);
            let help = "Quick start and receiver status, as the first view. Off takes its \
                        tab away and opens the receiver on the spectrum.";
            if switch(ui, "dashboard", &mut on, "show it first", help) {
                match on {
                    true => self.settings.edit(|s| s.dashboard = true),
                    false => self.hide_dashboard(),
                }
            }
        });
        ui.add_space(8.0);

        section(ui, t("settings.position"), "where the aerial is, for ranges and the map", |ui| {
            row_help(ui, "station", t("settings.position.help"), |ui| {
                let mut edit = self.station_edit.take();
                let here = self.setting(|s| s.location);
                let text = edit.get_or_insert_with(|| match here {
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
                if self.setting(|s| s.location.is_some()) {
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

    /// What this receiver hands to other machines.
    ///
    /// A listening socket is an installation's decision rather than the
    /// packet list's, and all of them are the same decision: who, on this
    /// network, may read what this radio hears.
    fn setup_network(&mut self, ui: &mut egui::Ui) {
        self.kiss_section(ui);
        ui.add_space(8.0);
        self.iqstream_section(ui);
    }

    /// What this build is, what the newest published release is, and the one
    /// button that fetches it.
    ///
    /// The asset's name is shown because a release carries one per platform
    /// and picking the wrong one is the usual mistake. Installing is the
    /// system's job: this downloads the package and opens it, then closes the
    /// window, because an installer cannot replace a program that is running.
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
            let features = crate::update::features();
            reading(
                ui,
                "features",
                if features.is_empty() { "none".to_string() } else { features.join(" ") },
            );
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
                        Some(a) => {
                            reading(
                                ui,
                                "download",
                                format!("{} ({})", a.name, crate::data::fmt_bytes(a.bytes)),
                            );
                            Self::install_row(ui, a);
                        }
                        None => reading(
                            ui,
                            "download",
                            format!("nothing for {} in that release", crate::update::platform()),
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
        if busy || matches!(crate::update::install_state(), crate::update::Install::Fetching { .. })
        {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(200));
        }
    }

    /// The button that fetches the new version, and how far it has got.
    fn install_row(ui: &mut egui::Ui, asset: &crate::update::Asset) {
        use crate::update::{Install, Kind};
        let installer = asset.kind == Kind::Installer;
        match crate::update::install_state() {
            Install::Idle => {
                let label = if installer { "DOWNLOAD AND INSTALL" } else { "DOWNLOAD" };
                if ui.button(legend(label)).clicked() {
                    crate::update::install(asset.clone());
                }
            }
            Install::Fetching { got, total } => {
                let share = if total > 0 { got as f32 / total as f32 } else { 0.0 };
                ui.add(egui::ProgressBar::new(share).desired_height(6.0));
                reading(
                    ui,
                    "fetched",
                    format!("{} of {}", crate::data::fmt_bytes(got), crate::data::fmt_bytes(total)),
                );
            }
            Install::Launched(path) => {
                // Windows locks the file of a running program and macOS
                // refuses to replace a running bundle, so on those the last
                // thing this window does is close. A Linux package manager
                // replaces /usr/bin/waveshark under a running one without
                // complaint, and closing the receiver to install would be
                // rude rather than necessary.
                let must_close = installer && (cfg!(windows) || cfg!(target_os = "macos"));
                if must_close {
                    lamp(ui, true, "the installer is open; WaveShark is closing");
                    ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
                } else if installer {
                    lamp(ui, true, "handed to the system installer");
                } else {
                    lamp(ui, true, &format!("saved to {}", path.display()));
                }
            }
            Install::Failed(e) => lamp(ui, false, &e),
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
                    port or a daemon elsewhere: a device path such as /dev/ttyACM0, \
                    gpsd:host, or modem:/dev/ttyACM0 for a sub-ghz-modem. DETECT asks \
                    every USB serial port whether a modem is on it.";
        let mut set: Option<Option<gps::Transport>> = None;
        let saved = self.setting(|s| s.gps.clone());
        row_help(ui, "gps", help, |ui| {
            let text = self.survey.gps_edit.get_or_insert_with(|| saved.clone());
            let (mut pressed, mut auto) = (false, false);
            let named = !saved.is_empty();
            let scanning = self
                .survey
                .gps_scan
                .as_ref()
                .is_some_and(|s: &crate::ui::state::ModemScan| s.found.is_none());
            let reserve = if named { 178.0 } else { 130.0 };
            let r = field_then(ui, text, gps::Transport::LOCAL_GPSD, reserve, |ui| {
                pressed = ui.small_button("SET").clicked();
                if named {
                    auto = ui.small_button("AUTO").clicked();
                }
                if ui
                    .add_enabled(!scanning, egui::Button::new("DETECT").small())
                    .on_hover_text(
                        "Opens each USB serial port in turn and asks for a modem's INFO. \
                         A port another program is reading cannot be opened, so close \
                         anything holding one first.",
                    )
                    .clicked()
                {
                    self.survey.gps_scan = Some(scan_for_modems());
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
        self.modem_rows(ui);
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
            (false, None) => {
                (saved.is_empty(), "no gps answering: the station is where it was set".into())
            }
        };
        lamp(ui, ok, &line);
        // The reader is not the radio's, so this pane keeps its own clock:
        // without it a fix arriving while nothing else is moving would sit
        // unshown until the pointer did.
        ui.ctx().request_repaint_after(std::time::Duration::from_millis(500));
    }

    /// What DETECT found, offered rather than applied.
    ///
    /// A modem answering on a port is not a decision: the operator may be
    /// running a real gpsd over a better receiver, and the board with no GNSS
    /// on it is listed too so that finding it is not mistaken for finding
    /// nothing. USE is what changes the station's source.
    fn modem_rows(&mut self, ui: &mut egui::Ui) {
        let Some(scan) = self.survey.gps_scan.as_mut() else {
            return;
        };
        if scan.found.is_none() {
            if let Ok(found) = scan.done.try_recv() {
                scan.found = Some(found);
            }
        }
        let Some(found) = scan.found.clone() else {
            hint(ui, "asking the serial ports for a modem");
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(250));
            return;
        };
        if found.is_empty() {
            hint(ui, "no modem answered on any usb serial port");
            return;
        }
        let mut pick = None;
        for f in &found {
            let path = match &f.transport {
                gps::Transport::Modem { path, .. } => path.clone(),
                other => other.to_string(),
            };
            let usable = f.info.gps.has_receiver();
            row(ui, "modem", |ui| {
                if ui
                    .add_enabled(usable, egui::Button::new("USE").small())
                    .on_disabled_hover_text("this board has no GNSS receiver on it")
                    .clicked()
                {
                    pick = Some(f.transport.clone());
                }
                let line = format!("{path}, {}", f.info.summary());
                theme::Line::new().value(line).show(ui);
            });
        }
        if let Some(t) = pick {
            self.survey.gps_edit = Some(t.to_string());
            self.set_gps(Some(t));
            self.survey.gps_scan = None;
        }
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
                        // While it runs, how far: the cell export is 85 MB
                        // and a script repository two gigabytes, and a
                        // button reading CHECKING for four minutes is
                        // indistinguishable from one that has hung.
                        if r.busy && (r.progress.running || r.progress.done > 0) {
                            widgets::progress(ui, r.progress.done, r.progress.total);
                        }
                        if let Some(e) = &r.error {
                            lamp(ui, false, e);
                        }
                        // What the descriptions read as, not what landed:
                        // a file that fails its own vectors is on disc and
                        // running nothing.
                        if r.which == crate::data::Which::Repo(&datasets::git::PROTOCOLS)
                            && let Some(got) = crate::protocols::last()
                        {
                            match got.refused.first() {
                                None => lamp(
                                    ui,
                                    true,
                                    &format!("{} descriptions installed", got.names.len()),
                                ),
                                Some((path, why)) => lamp(ui, false, &format!("{path}: {why}")),
                            }
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
            // Often enough that a bar moves rather than steps: a download
            // is the one thing in this pane that changes while nobody
            // touches anything.
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(100));
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
        let Some(before) = self.setting(|s| key_of(s, which, index).cloned()) else {
            return;
        };
        let mut text = before.clone();
        row_help(ui, k.label, k.help, |ui| {
            if k.secret {
                secret(ui, &mut text);
            } else {
                field(ui, &mut text, k.hint);
            }
        });
        if text != before {
            self.settings.edit(|s| {
                if let Some(slot) = key_slot(s, which, index) {
                    *slot = text.clone();
                }
            });
            which.set_key(index, &text);
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
        // Two switches and no text, so each goes into the record as it is
        // clicked: there is nothing here to type wrongly on the way.
        let (mut on, mut lookup) = self.setting(|s| (s.beacondb_on, s.beacondb_lookup));
        let (was_on, was_lookup) = (on, lookup);
        let r = egui::containers::Modal::new(egui::Id::new("beacondb"))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(520.0);
                modal_title(ui, "Feed beacondb.net");
                let b = &self.survey.beacondb;
                section(ui, "submit", "crowd-sourced, no account, published as collected", |ui| {
                    let why = "Bluetooth devices and cells heard with a position are spooled \
                               to disc and submitted when there is a network. Everything else \
                               the survey records stays here: there is no beacon type for an \
                               aircraft or a pager. What is submitted is where this receiver \
                               was when it heard something, so a drive is a track of where you \
                               have been. Levels are not sent: this receiver measures dBFS and \
                               the field means dBm.";
                    switch(ui, "submit", &mut on, "while receiving", why);
                    let ask = "Draws a position for a cell you have decoded that the \
                               OpenCelliD export has no row for, as a cross with the accuracy \
                               beaconDB gives it. Asking tells beaconDB which cells this \
                               receiver has heard, which is why it is separate from submitting.";
                    switch(ui, "look up", &mut lookup, "where a heard cell is", ask);
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
                });
            });
        if r.should_close() {
            close = true;
        }
        if (on, lookup) != (was_on, was_lookup) {
            self.settings.edit(|s| {
                s.beacondb_on = on;
                s.beacondb_lookup = lookup;
            });
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
                                if st.offline > 0 {
                                    reading(
                                        ui,
                                        "missed",
                                        format!("{} readings: nothing was connected", st.offline),
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

    /// The one place a thing on the network is brought in.
    ///
    /// Two different things arrive over a socket and an operator has no way to
    /// tell them apart from an address: a radio hands over samples and the
    /// whole receiver runs on them here, where a packet feed hands over frames
    /// somebody else has already demodulated, which only the packet bus sees.
    /// So the first row is that choice, and the dialog routes it to the radio
    /// list or to the feeds.
    pub(super) fn remote_modal(&mut self, ctx: &egui::Context) {
        let Some(mut edit) = self.remote.take() else {
            return;
        };
        let (mut close, mut add, mut find) = (false, false, false);
        let mut link = None;
        let r = egui::containers::Modal::new(egui::Id::new("add-remote"))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(520.0);
                modal_title(ui, "Add over the network");
                section(ui, "source", "what is at the other end of the socket", |ui| {
                    row_help(ui, "brings", Over::HELP, |ui| {
                        let opts = Over::ALL.iter().map(|o| (*o, o.label().to_string()));
                        choice(ui, "remote-over", &mut edit.over, opts);
                    });
                    match edit.over {
                        Over::Samples => {
                            row_help(ui, "protocol", edit.proto.help(), |ui| {
                                let opts =
                                    remote::Proto::ALL.iter().map(|p| (*p, p.name().to_string()));
                                choice(ui, "remote-proto", &mut edit.proto, opts);
                            });
                            row_help(ui, "serves it", edit.proto.help(), |ui| {
                                if server_row(ui, edit.proto.server()) {
                                    link = Some(edit.proto.url().to_string());
                                }
                            });
                            if edit.proto == remote::Proto::SpyServer {
                                row_help(ui, "public", FIND_HELP, |ui| {
                                    if listed_row(ui) {
                                        find = true;
                                    }
                                });
                            }
                        }
                        Over::Frames => {
                            row_help(ui, "format", FEED_HELP, |ui| {
                                let opts = nodes::FEED_KINDS
                                    .iter()
                                    .map(|k| (k.name, k.name.to_string()))
                                    .collect::<Vec<_>>();
                                let mut name = edit.feed.name;
                                if choice(ui, "remote-feed", &mut name, opts)
                                    && let Some(k) = nodes::feed_kind(name)
                                {
                                    edit.feed = k;
                                }
                            });
                            row_help(ui, "serves it", FEED_HELP, |ui| {
                                if server_row(ui, edit.feed.server) {
                                    link = Some(edit.feed.url.to_string());
                                }
                            });
                        }
                    }
                    let mut focus = None;
                    let placeholder = match edit.over {
                        Over::Samples => edit.proto.placeholder().to_string(),
                        Over::Frames => {
                            format!("host, or host:port ({})", edit.feed.default_port)
                        }
                    };
                    row_help(
                        ui,
                        "address",
                        "Where it is. A bare host gets the usual port.",
                        |ui| {
                            let f = field(ui, &mut edit.host, &placeholder);
                            if f.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                                add = true;
                            }
                            focus = Some(f);
                        },
                    );
                    if edit.over == Over::Samples {
                        row_help(
                            ui,
                            "name",
                            "What the radio list calls it. An address says which machine and \
                             nothing about which aerial.",
                            |ui| {
                                let name = field(ui, &mut edit.label, "loft dongle");
                                if name.lost_focus()
                                    && ui.input(|i| i.key_pressed(egui::Key::Enter))
                                {
                                    add = true;
                                }
                            },
                        );
                    }
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
                        None => {
                            let what = match edit.over {
                                Over::Samples => edit.proto.name(),
                                Over::Frames => edit.feed.name,
                            };
                            lamp(ui, true, &format!("{what} at {}", edit.host.trim()))
                        }
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
        if let Some(url) = link {
            ctx.open_url(egui::OpenUrl::new_tab(url));
        }
        if find {
            self.find = Some(FindEdit::open());
        }
        if r.should_close() && self.find.is_none() {
            close = true;
        }
        if add {
            let done = match edit.over {
                Over::Samples => self.add_remote(ctx, &mut edit),
                Over::Frames => self.add_feed(&edit),
            };
            match done {
                Ok(()) => close = true,
                Err(e) => edit.err = Some(e),
            }
        }
        if !close {
            self.remote = Some(edit);
        }
    }

    pub(super) fn find_modal(&mut self, ctx: &egui::Context) {
        let Some(mut edit) = self.find.take() else {
            return;
        };
        let which = crate::data::Which::SpyServers;
        let servers = crate::data::spyservers();
        let (hz, bad_hz) = edit.hz();
        let filter =
            datasets::spyserver::Filter { hz, full_control: edit.full_control, free: edit.free };
        let kept: Vec<&datasets::spyserver::Server> =
            servers.iter().flat_map(|v| v.iter()).filter(|s| filter.keeps(s)).collect();
        let (mut close, mut tune) = (false, None);
        let r = egui::containers::Modal::new(egui::Id::new("find-spyserver"))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(560.0);
                modal_title(ui, "Public SpyServers");
                section(ui, "filter", "which of the listed servers to show", |ui| {
                    row_help(ui, "tunes", TUNES_HELP, |ui| {
                        field(ui, &mut edit.mhz, "any frequency, or MHz such as 145.8");
                    });
                    switch(
                        ui,
                        "control",
                        &mut edit.full_control,
                        "only servers whose dial may be moved",
                        CONTROL_HELP,
                    );
                    switch(
                        ui,
                        "free",
                        &mut edit.free,
                        "only servers with a listener slot free",
                        FREE_HELP,
                    );
                    match (&edit.err, &bad_hz, &servers) {
                        (Some(e), _, _) => lamp(ui, false, e),
                        (None, Some(e), _) => lamp(ui, false, e),
                        (None, None, Some(v)) => {
                            lamp(ui, true, &format!("{} of {} servers", kept.len(), v.len()))
                        }
                        (None, None, None) => match crate::data::failed(which) {
                            Some(e) => lamp(ui, false, &e),
                            None => lamp(ui, false, "reading the directory"),
                        },
                    }
                });
                ui.add_space(6.0);
                let w = ui.available_width();
                egui::ScrollArea::vertical()
                    .max_height(share_of_screen(ui, 0.55, 240.0, 520.0))
                    .show(ui, |ui| {
                        ui.set_max_width(w);
                        for s in &kept {
                            if server_card(ui, s) {
                                tune = Some((*s).clone());
                            }
                            ui.add_space(6.0);
                        }
                    });
                footer(ui, |ui| {
                    if ui.button(crate::i18n::t("ui.close")).clicked() {
                        close = true;
                    }
                    let label = match crate::data::busy(which) {
                        true => "CHECKING",
                        false => crate::i18n::t("ui.refresh"),
                    };
                    if ui.add_enabled(!crate::data::busy(which), egui::Button::new(label)).clicked()
                    {
                        crate::data::refresh(which);
                    }
                });
            });
        if r.should_close() {
            close = true;
        }
        if let Some(s) = tune {
            let mut remote = RemoteEdit::spyserver();
            remote.host = s.addr();
            remote.label = s.description.clone();
            match self.add_remote(ctx, &mut remote) {
                Ok(()) => {
                    self.remote = None;
                    close = true;
                }
                Err(e) => edit.err = Some(format!("{}: {e}", s.addr())),
            }
        }
        if !close {
            self.find = Some(edit);
        }
    }

    /// Ask what a capture holds, for a file whose name does not say.
    ///
    /// Refusing was right where a rate would be guessed, since a wrong rate
    /// rescales every pulse width, and wrong where the operator knows the
    /// rate and has nowhere to type it.
    pub(super) fn capture_modal(&mut self, ctx: &egui::Context) {
        let Some(mut edit) = self.capture_edit.take() else {
            return;
        };
        let (mut close, mut open) = (false, false);
        let r = egui::containers::Modal::new(egui::Id::new("describe-capture"))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(520.0);
                modal_title(ui, "What is in this capture");
                let name = edit
                    .path
                    .file_name()
                    .map(|n| n.to_string_lossy().into_owned())
                    .unwrap_or_else(|| edit.path.display().to_string());
                section(ui, "capture", "the name does not say, so say it here", |ui| {
                    reading(ui, "file", name);
                    row_help(
                        ui,
                        "rate",
                        "Samples per second, as the recording was made. k and M are \
                         understood. A rate set wrong rescales every pulse width.",
                        |ui| {
                            let f = field(ui, &mut edit.rate, "250k");
                            if f.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                                open = true;
                            }
                        },
                    );
                    row_help(
                        ui,
                        "centre",
                        "Where the radio was tuned, in MHz. Empty for a recording at \
                         baseband, which puts the dial at zero.",
                        |ui| {
                            field(ui, &mut edit.center, "433.92");
                        },
                    );
                    row_help(
                        ui,
                        "samples",
                        "How a sample is written: unsigned bytes from an RTL-SDR, signed \
                         bytes from a HackRF, 16-bit or float from most recorders.",
                        |ui| {
                            let opts = common::SampleFormat::ALL
                                .iter()
                                .map(|f| (*f, format!("{} ({})", f.extension(), f.label())));
                            choice(ui, "capture-format", &mut edit.format, opts);
                        },
                    );
                    match edit.resolve() {
                        Err(e) => lamp(ui, false, &e),
                        Ok(c) => lamp(
                            ui,
                            true,
                            &format!(
                                "{:.0} S/s, {}, {:.1} s",
                                c.rate.as_f64(),
                                match c.center {
                                    Some(h) => format!("{:.3} MHz", h.as_f64() / 1e6),
                                    None => "baseband".to_string(),
                                },
                                c.seconds
                            ),
                        ),
                    }
                });
                footer(ui, |ui| {
                    if ui.button(if edit.to_air { "SEND" } else { "OPEN" }).clicked() {
                        open = true;
                    }
                    if ui.button(crate::i18n::t("ui.close")).clicked() {
                        close = true;
                    }
                });
            });
        if r.should_close() {
            close = true;
        }
        if open {
            match edit.resolve() {
                Err(_) => {}
                Ok(c) => {
                    close = true;
                    match edit.to_air {
                        true => {
                            let tx =
                                crate::radio::TxCapture::new(&c.path, c.rate, c.center, c.format);
                            self.audio.capture_pick.file = tx.clone();
                            self.cmds.push(crate::radio::Cmd::TxCapture(tx));
                        }
                        false => {
                            crate::devices::add_capture(c.path.clone(), c.rate, c.center, c.format);
                            self.devices = crate::devices::list();
                            if let Some(e) = self
                                .devices
                                .iter()
                                .find(|d| d.path.as_deref() == Some(c.path.as_path()))
                                .cloned()
                            {
                                self.select_device(ctx, e);
                            }
                        }
                    }
                }
            }
        }
        if !close {
            self.capture_edit = Some(edit);
        }
    }

    /// Register the server, list it, and tune to it.
    ///
    /// The server is asked what it is streaming before it is kept, because a
    /// remote radio that does not answer is an entry in a list with nothing
    /// behind it, and the operator finds out at the point of adding rather
    /// than later when the spectrum stays empty.
    ///
    /// What answered decides the protocol, whatever was picked: iqstreamd and
    /// rtl_tcp both listen on 1234, so an address alone cannot say which is
    /// there and the picker is a guess until something replies.
    fn add_remote(
        &mut self,
        ctx: &egui::Context,
        edit: &mut RemoteEdit,
    ) -> std::result::Result<(), String> {
        let found = match edit.proto.probe(&edit.host) {
            Ok(p) => p,
            Err(chosen) => match remote::identify(&edit.host) {
                Ok(other) => {
                    edit.proto = other.proto;
                    other
                }
                Err(_) => return Err(chosen.to_string()),
            },
        };
        let addr = crate::devices::add_stream(found.proto, &edit.host, &edit.label)
            .ok_or_else(|| "expected host or host:port".to_string())?;
        self.devices = crate::devices::list();
        let found = self
            .devices
            .iter()
            .find(|d| d.addr.as_deref().is_some_and(|a| crate::devices::same_server(a, &addr)))
            .cloned()
            .ok_or_else(|| format!("{addr} did not answer"))?;
        self.select_device(ctx, found);
        Ok(())
    }

    /// Attach the feed, which is a setting rather than a radio: the receiver
    /// keeps running on whatever it is tuned to and the frames join the bus.
    fn add_feed(&mut self, edit: &RemoteEdit) -> std::result::Result<(), String> {
        let spec = super::parse_feed(&edit.host, edit.feed)
            .ok_or_else(|| "expected host or host:port".to_string())?;
        if self.setting(|s| s.feeds.clone()).contains(&spec) {
            return Err("that feed is already attached".into());
        }
        self.settings.edit(|s| s.feeds.push(spec));
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
        // A recording from another program is named for what it holds, not
        // in the rtl_433 convention, so the common case of somebody else's
        // file is a card asking what it is rather than a refusal.
        let meta = crate::devices::describe_capture(&path);
        let Some(c) = meta
            .rate
            .zip(meta.format)
            .and_then(|(r, f)| crate::devices::add_capture(path.clone(), r, meta.center, f))
        else {
            self.capture_edit = Some(CaptureEdit::new(path, false));
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
            if let Some(stage) = controls.tx_stages.iter().find(|s| !s.is_switch()) {
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
            for n in &controls.numbers {
                row_help(ui, &n.label, &n.help, |ui| {
                    let mut v = n.value;
                    let drag = egui::DragValue::new(&mut v)
                        .speed(n.step.max(1.0))
                        .range(n.range.clone())
                        .suffix(format!(" {}", n.unit));
                    if ui.add(drag).changed() {
                        self.radio_settings.set_number(&n.name, n.quantise(v));
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
            let mut dc = self.setting(|s| s.dc_block);
            let dc_help = "A direct conversion receiver leaks its own local oscillator into \
                           the middle of the span, where it looks exactly like a carrier on \
                           the frequency you are tuned to. This measures the offset and \
                           subtracts it.";
            if switch(ui, "centre spur", &mut dc, "remove", dc_help) {
                self.settings.edit(|s| s.dc_block = dc);
            }
        });
        ui.add_space(8.0);

        self.rds_station(ui);
        ui.add_space(8.0);

        self.raw_capture(ui);
        ui.add_space(8.0);
        self.trim_capture(ui);

        if changed {
            self.apply_radio_settings();
        }
    }

    /// What a keyed WFM channel says about itself on the 57 kHz subcarrier.
    ///
    /// With the radio rather than on the strip because it is one station for
    /// the receiver however many channels are set to WFM, and because a name
    /// and a message are typed once and left alone.
    fn rds_station(&mut self, ui: &mut egui::Ui) {
        section(ui, "rds", "what a keyed WFM channel calls itself", |ui| {
            let mut on = self.setting(|s| s.rds_on);
            let why = "A broadcast station names itself on a subcarrier above the \
                       programme, and a receiver shows that name instead of the \
                       frequency. Off transmits the programme alone.";
            if switch(ui, "identify", &mut on, "as a station", why) {
                self.settings.edit(|s| s.rds_on = on);
            }
            let pi_help = "The programme identification code, four hex digits, which is \
                           how a receiver tells two transmitters of one programme apart. \
                           The first digit is the country.";
            row_help(ui, "pi code", pi_help, |ui| {
                let mut text = self.setting(|s| s.rds_pi.clone());
                if field(ui, &mut text, "C479").changed() {
                    self.settings.edit(|s| s.rds_pi = text.clone());
                }
            });
            row_help(ui, "station", "Eight characters, which is all the standard carries.", |ui| {
                let mut text = self.setting(|s| s.rds_name.clone());
                if field(ui, &mut text, "WAVESHRK").changed() {
                    self.settings.edit(|s| s.rds_name = text.clone());
                }
            });
            let text_help = "The message under the name, up to 64 characters. Every four \
                             characters is another group, so a long one takes longer to \
                             arrive and the name is repeated less often.";
            row_help(ui, "radiotext", text_help, |ui| {
                let mut text = self.setting(|s| s.rds_text.clone());
                if field(ui, &mut text, "nothing").changed() {
                    self.settings.edit(|s| s.rds_text = text.clone());
                }
            });
            let station = self.setting(|s| s.rds());
            let pi = self.setting(|s| s.rds_pi.clone());
            match (on, station) {
                (false, _) => lamp(ui, true, "the programme alone, with no data on it"),
                (true, Some(s)) => lamp(
                    ui,
                    true,
                    &format!("{:04X} \"{}\" on a WFM channel", s.pi, s.label().trim_end()),
                ),
                (true, None) => {
                    lamp(ui, false, &format!("{pi:?} is not four hex digits, so nothing is sent"))
                }
            }
        });
    }

    /// Cutting the capture being replayed down to what is in it.
    ///
    /// Only on a file, because there is nothing to cut on a radio. The same
    /// cut the `iq_clipper` example makes, which was the only way to reach it
    /// and is findable by nobody: a recording is made here, played here, and
    /// is trimmed here too. The output is a second file beside the original,
    /// named so the receiver list reads its centre and rate back.
    fn trim_capture(&mut self, ui: &mut egui::Ui) {
        let Some(path) = self
            .device
            .as_ref()
            .filter(|d| d.kind == common::device::DriverKind::File)
            .and_then(|d| d.path.clone())
        else {
            return;
        };
        if self.trimming.as_ref().is_some_and(|p| p.ready().is_some()) {
            let done = self.trimming.take().expect("a cut that is ready").block_and_take();
            if let Ok(made) = &done {
                crate::devices::add_named_capture(made.path.clone());
                self.devices = crate::devices::list();
            }
            self.trim.said = Some(done);
        }
        let meta = crate::devices::described(&path);
        let running = self.trimming.is_some();
        let mut cut = false;
        card(
            ui,
            None,
            |ui| {
                theme::Line::new().legend("trim").show(ui);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    cut = ui.add_enabled(!running, egui::Button::new("TRIM")).clicked();
                    theme::Line::new()
                        .note("a shorter capture beside this one")
                        .size(10.5)
                        .elided(ui);
                });
            },
            |ui| {
                let keep_help = "The transmissions keeps what is over the noise and drops \
                                 the silence between, which is what makes a capture small \
                                 enough to keep. A window keeps a stretch of the recording \
                                 whatever is in it.";
                row_help(ui, "keep", keep_help, |ui| {
                    choice(
                        ui,
                        "trim_keep",
                        &mut self.trim.bursts,
                        [(true, "the transmissions".to_string()), (false, "a window".to_string())],
                    );
                });
                if self.trim.bursts {
                    let margin_help = "Quiet kept either side of a transmission. A detector \
                                       takes its noise floor from the band next to a burst, \
                                       so a file cut flush to the edges reads worse than the \
                                       one it came from: the BLE capture reads five of \
                                       eleven packets at 2 ms and six at 4 ms.";
                    row_help(ui, "margin", margin_help, |ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.trim.margin_ms)
                                .speed(0.5)
                                .range(0.0..=1000.0)
                                .suffix(" ms either side"),
                        );
                    });
                    let over_help = "How far over the noise floor a burst has to be. The \
                                     floor is a low percentile of the whole file, so a \
                                     recording that is mostly transmission has a high one.";
                    row_help(ui, "over", over_help, |ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.trim.threshold_db)
                                .speed(0.5)
                                .range(1.0..=60.0)
                                .suffix(" dB"),
                        );
                    });
                } else {
                    row_help(ui, "window", "Where the window starts and how long it runs.", |ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.trim.skip_s)
                                .speed(0.1)
                                .range(0.0..=100_000.0)
                                .max_decimals(3)
                                .suffix(" s in"),
                        );
                        ui.add(
                            egui::DragValue::new(&mut self.trim.seconds)
                                .speed(0.1)
                                .range(0.001..=100_000.0)
                                .max_decimals(3)
                                .suffix(" s long"),
                        );
                    });
                }
                let name_help = "Added to the name of the new file, which keeps the centre \
                                 and the rate of the original so it replays.";
                row_help(ui, "called", name_help, |ui| {
                    field(ui, &mut self.trim.tag, "clip");
                });
                match (meta.center, meta.rate.zip(meta.format)) {
                    (Some(c), Some((r, f))) => {
                        let name = sources::clip::output_name(&path, c, r, f, self.trim.tag.trim());
                        reading(
                            ui,
                            "writes",
                            name.file_name().and_then(|s| s.to_str()).unwrap_or("").to_string(),
                        );
                    }
                    _ => {
                        lamp(ui, false, "this capture has no centre and rate to write");
                    }
                }
                if running {
                    lamp(ui, true, "cutting");
                } else if let Some(said) = &self.trim.said {
                    match said {
                        Ok(c) => lamp(
                            ui,
                            true,
                            &format!(
                                "{} span(s), {:.3} s kept, {:.1}% of the recording",
                                c.spans.len(),
                                c.seconds(meta.rate.unwrap_or(common::Sps(1))),
                                100.0 * c.share()
                            ),
                        ),
                        Err(e) => lamp(ui, false, e),
                    }
                }
            },
        );
        if !cut {
            return;
        }
        let (Some(center), Some((rate, format))) = (meta.center, meta.rate.zip(meta.format)) else {
            self.trim.said = Some(Err("this capture has no centre and rate to write".into()));
            return;
        };
        let out = sources::clip::free_name(sources::clip::output_name(
            &path,
            center,
            rate,
            format,
            self.trim.tag.trim(),
        ));
        let cut = match self.trim.bursts {
            true => sources::Cut::Bursts {
                skip_s: 0.0,
                how: sources::clip::Bursts {
                    margin_ms: self.trim.margin_ms,
                    threshold_db: self.trim.threshold_db,
                    ..Default::default()
                },
            },
            false => sources::Cut::Window { skip_s: self.trim.skip_s, seconds: self.trim.seconds },
        };
        self.trim.said = None;
        let ctx = ui.ctx().clone();
        // Off the frame: a gigabyte read, measured and written back is
        // seconds of work, and a window that does not paint is a window the
        // compositor puts a "not responding" dialog over.
        self.trimming = Some(poll_promise::Promise::spawn_thread("trim capture", move || {
            let r = sources::clip_file_as(&path, &out, &cut, meta).map_err(|e| e.to_string());
            ctx.request_repaint();
            r
        }));
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
        let (armed, bursts, level_db, threshold_db) = match &self.radio {
            Some(r) => {
                use std::sync::atomic::Ordering;
                (
                    r.status.capture_armed.load(Ordering::Relaxed),
                    r.status.capture_bursts.load(Ordering::Relaxed),
                    f32::from_bits(r.status.capture_level_db.load(Ordering::Relaxed)),
                    f32::from_bits(r.status.capture_threshold_db.load(Ordering::Relaxed)),
                )
            }
            None => (false, 0, f32::NEG_INFINITY, f32::NEG_INFINITY),
        };
        section(ui, "raw capture", "the whole span to one file, as it arrives", |ui| {
            // What was asked for rather than what the node reports: a radio
            // that is not running has no capture node to ask, and a switch
            // that reads off one goes dark the moment the source stops.
            let mut on = self.setting(|s| s.capture_on);
            let why = "The recording to make when the receiver shows a transmission and \
                       reads nothing from it: replaying the file puts the same samples \
                       through the same graph, so a decoder can be changed and tried again.";
            if switch(ui, "capture", &mut on, "the raw span", why) {
                self.set_capture(on);
            }
            let mut arm = self.setting(|s| s.capture_arm);
            let was = arm;
            let trigger_help = "On the switch, the file is everything from the moment it \
                                goes on. Armed on energy, the receiver waits and writes a \
                                file per burst, with the signal from before the trigger in \
                                front of it: that is how to catch something that happens \
                                twice a night without recording the night.";
            row_help(ui, "start on", trigger_help, |ui| {
                choice(
                    ui,
                    "capture_trigger",
                    &mut arm.trigger,
                    [
                        (nodes::capture_nodes::Trigger::Switch, "the switch".to_string()),
                        (nodes::capture_nodes::Trigger::Energy, "energy".to_string()),
                    ],
                );
            });
            if arm.trigger == nodes::capture_nodes::Trigger::Energy {
                let ref_help = "Above the floor follows the band as it gets busier and \
                                survives a gain change. In dBFS is the number to set when \
                                the floor itself is what moved.";
                row_help(ui, "threshold", ref_help, |ui| {
                    choice(
                        ui,
                        "capture_reference",
                        &mut arm.reference,
                        [
                            (nodes::capture_nodes::Reference::Floor, "over the floor".to_string()),
                            (nodes::capture_nodes::Reference::Absolute, "in dBFS".to_string()),
                        ],
                    );
                });
                let db_help = "How loud the span has to get before a file is opened. The \
                               detector opens a channel at 8 dB over the floor, so much \
                               above 10 dB is a capture that misses what the receiver \
                               heard.";
                row_help(ui, "at", db_help, |ui| {
                    let mut db = arm.threshold_db as f64;
                    if ui
                        .add(
                            egui::DragValue::new(&mut db)
                                .speed(0.5)
                                .range(-120.0..=60.0)
                                .suffix(" dB"),
                        )
                        .changed()
                    {
                        arm.threshold_db = db as f32;
                    }
                });
                let band_help = "What the level is measured over. The whole span only \
                                 trips on a signal about as wide as it is: a 12.5 kHz \
                                 transmission 20 dB over the noise lifts a 2.4 MHz span \
                                 by 1.8 dB and never opens a file. Set the width of the \
                                 signal and where it sits from the middle of the span, \
                                 and the trigger measures that instead.";
                row_help(ui, "measure", band_help, |ui| {
                    let (mut width, mut offset) =
                        (arm.band_hz as f64 / 1e3, arm.band_offset_hz as f64 / 1e3);
                    if ui
                        .add(
                            egui::DragValue::new(&mut width)
                                .speed(1.0)
                                .range(0.0..=100_000.0)
                                .suffix(" kHz wide"),
                        )
                        .changed()
                    {
                        arm.band_hz = (width * 1e3) as f32;
                    }
                    if ui
                        .add(
                            egui::DragValue::new(&mut offset)
                                .speed(1.0)
                                .range(-50_000.0..=50_000.0)
                                .suffix(" kHz off"),
                        )
                        .changed()
                    {
                        arm.band_offset_hz = (offset * 1e3) as f32;
                    }
                });
                if arm.band_hz <= 0.0 {
                    hint(ui, "zero is the whole span, which a narrow signal cannot lift");
                }
                let window_help = "How much of the signal before the trigger goes in the \
                                   file, and how long the span may stay quiet before it \
                                   is closed. A short pre-roll loses the head of the \
                                   burst, which is where the sync word is.";
                row_help(ui, "window", window_help, |ui| {
                    let (mut pre, mut hang) = (arm.pre_ms as f64, arm.hang_ms as f64);
                    if ui
                        .add(
                            egui::DragValue::new(&mut pre)
                                .speed(10.0)
                                .range(0.0..=5_000.0)
                                .suffix(" ms before"),
                        )
                        .changed()
                    {
                        arm.pre_ms = pre as f32;
                    }
                    if ui
                        .add(
                            egui::DragValue::new(&mut hang)
                                .speed(10.0)
                                .range(0.0..=30_000.0)
                                .suffix(" ms after"),
                        )
                        .changed()
                    {
                        arm.hang_ms = hang as f32;
                    }
                });
            }
            if arm != was {
                self.set_capture_arm(arm);
            }
            let cap_help = "What the whole folder may take. Nothing here is deleted: a \
                            capture is evidence of a signal that may not come again, so \
                            writing stops instead.";
            row_help(ui, "limit", cap_help, |ui| {
                let mut cap = self.setting(|s| s.capture_cap_mb);
                let opts = [Some(1024u64), Some(4096), Some(16_384), Some(65_536), None];
                if choice(ui, "capture_cap", &mut cap, opts.map(|o| (o, size_label(o)))) {
                    self.settings.edit(|s| s.capture_cap_mb = cap);
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
            if arm.trigger == nodes::capture_nodes::Trigger::Energy && cap_on {
                reading(ui, "files", bursts.to_string());
            }
            if cap_full {
                lamp(ui, false, "stopped: the folder is at its limit");
            } else if armed {
                // What the setting comes to right now, which is the only way
                // to tell a threshold nothing will ever reach from one the
                // noise crosses: both look the same as a number in a box.
                match threshold_db.is_finite() {
                    true => lamp(
                        ui,
                        true,
                        &format!(
                            "armed at {threshold_db:.0} dBFS, {} at {:.0} dBFS",
                            match arm.band_hz > 0.0 {
                                true => "band",
                                false => "span",
                            },
                            level_db.max(-199.0)
                        ),
                    ),
                    false => lamp(ui, false, "waiting for a floor to measure the threshold from"),
                }
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

/// What comes over the socket, which is the first thing the dialog asks.
///
/// Both answers are an address in a list and neither says which it is, so the
/// difference is stated here rather than left to the operator: samples make
/// this receiver run the whole chain, frames are somebody else's decode and
/// reach the packet bus alone.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Over {
    Samples,
    Frames,
}

impl Over {
    const ALL: &'static [Over] = &[Over::Samples, Over::Frames];

    const HELP: &'static str = "Samples are a radio: the span, the spectrum, the decoders and \
                                the audio all run here, on what its tuner hears. Frames are a \
                                feed: another receiver has already demodulated them and only \
                                the packet list, the map and the log see them.";

    fn label(self) -> &'static str {
        match self {
            Self::Samples => "samples, a radio",
            Self::Frames => "frames, a packet feed",
        }
    }
}

const FEED_HELP: &str = "The wire format the far end writes. Beast carries a signal \
                                 level with every frame and AVR carries none.";

/// The program to install at the far end, with a button to its page.
///
/// A protocol name is not enough to act on: SpyServer, rtl_tcp and readsb are
/// names of programs before they are names of formats, and an operator
/// reading one has nowhere to go.
fn server_row(ui: &mut egui::Ui, server: &str) -> bool {
    let mut open = false;
    ui.horizontal(|ui| {
        theme::Line::new().value(server).size(13.0).show(ui);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            open = ui.small_button("PAGE").clicked();
        });
    });
    open
}

const FIND_HELP: &str = "Servers their owners have listed in the Airspy directory, open to \
     anybody. TUNE on one adds it to the radio list like an address typed here.";
const TUNES_HELP: &str = "Keep only servers whose radio reaches this frequency. Empty for all.";
const CONTROL_HELP: &str = "A server granting control lets the dial go anywhere in its range. \
     One that does not lets it move only inside the span it is already on.";
const FREE_HELP: &str = "A server takes a fixed number of listeners and turns the next away.";

fn listed_row(ui: &mut egui::Ui) -> bool {
    let mut open = false;
    ui.horizontal(|ui| {
        let said = match crate::data::spyservers() {
            Some(v) => format!("{} in the Airspy directory", v.len()),
            None => "the Airspy directory".to_string(),
        };
        theme::Line::new().value(said).size(13.0).show(ui);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            open = ui.small_button("FIND").clicked();
        });
    });
    open
}

fn bare_mhz(hz: u64) -> String {
    let s = format!("{:.3}", hz as f64 / 1e6);
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

fn server_card(ui: &mut egui::Ui, s: &datasets::spyserver::Server) -> bool {
    let mut tune = false;
    let rail = s.has_slot().then_some(theme::TRACE);
    let mhz = |hz: u64| format!("{:.3}", hz as f64 / 1e6);
    card(
        ui,
        rail,
        |ui| {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                tune = ui.button("TUNE").clicked();
                ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                    theme::Line::new()
                        .value(&s.description)
                        .size(12.0)
                        .gap(12.0)
                        .note(&s.device)
                        .size(10.5)
                        .elided(ui);
                });
            });
        },
        |ui| {
            theme::Line::new()
                .legend("range")
                .value(format!("{}-{} MHz", bare_mhz(s.min_hz), bare_mhz(s.max_hz)))
                .size(12.0)
                .gap(18.0)
                .legend("on")
                .heard(mhz(s.center_hz))
                .size(12.0)
                .gap(18.0)
                .legend("span")
                .value(format!("{} kHz", s.bandwidth_hz / 1000))
                .size(12.0)
                .gap(18.0)
                .legend("users")
                .value(format!("{} of {}", s.clients, s.max_clients))
                .size(12.0)
                .show(ui);
            let dial = match s.full_control {
                true => "dial free".to_string(),
                false => "dial fixed to its span".to_string(),
            };
            let session = match s.session_limit {
                Some(secs) => format!(", {} min a session", secs.div_ceil(60)),
                None => String::new(),
            };
            let antenna = match s.antenna.is_empty() {
                true => String::new(),
                false => format!(", {}", s.antenna),
            };
            hint(ui, &format!("{}, {dial}{session}{antenna}", s.addr()));
        },
    );
    tune
}

/// A capture whose name does not say what it holds, while the card asking is
/// open.
///
/// One card for both halves: the receiver list replays a capture and the
/// strip's IQ source transmits one, and both read the same names off the same
/// disc.
#[derive(Clone)]
pub struct CaptureEdit {
    path: std::path::PathBuf,
    /// Whether it is being sent rather than replayed.
    to_air: bool,
    /// Megahertz, empty for a baseband recording.
    center: String,
    /// Samples per second, k and M understood.
    rate: String,
    format: common::SampleFormat,
}

impl CaptureEdit {
    /// Whatever the name did carry is filled in, because a capture named
    /// `sdrsharp_20240110_433920kHz_IQ.wav` says its centre and not its rate
    /// and retyping the half that was there is work nobody should do.
    pub fn new(path: std::path::PathBuf, to_air: bool) -> Self {
        let meta = crate::devices::describe_capture(&path);
        Self {
            center: meta.center.map(|c| format!("{:.6}", c.as_f64() / 1e6)).unwrap_or_default(),
            rate: meta.rate.map(|r| r.0.to_string()).unwrap_or_default(),
            format: meta.format.unwrap_or(common::SampleFormat::Cu8),
            path,
            to_air,
        }
    }

    /// What the fields describe, or why they do not describe a capture yet.
    fn resolve(&self) -> std::result::Result<crate::devices::Capture, String> {
        let rate = sources::parse_si(self.rate.trim())
            .filter(|r| *r >= 1.0)
            .ok_or_else(|| "no sample rate, and a guessed one decodes nothing".to_string())?;
        let center = match self.center.trim() {
            "" => None,
            t => Some(common::Hz(
                (t.parse::<f64>().map_err(|_| format!("{t} is not a frequency in MHz"))? * 1e6)
                    as u64,
            )),
        };
        let c = crate::devices::Capture {
            path: self.path.clone(),
            rate: common::Sps(rate as u64),
            center,
            format: self.format,
            seconds: 0.0,
        };
        let len = std::fs::metadata(&c.path)
            .ok()
            .filter(|m| m.is_file())
            .map(|m| m.len())
            .ok_or_else(|| format!("{} is not a file", c.path.display()))?;
        let samples = len / c.format.bytes_per_sample() as u64;
        if samples == 0 {
            return Err(format!("{} holds no whole samples in that format", c.path.display()));
        }
        Ok(crate::devices::Capture { seconds: samples as f64 / c.rate.as_f64(), ..c })
    }
}

#[derive(Clone)]
pub struct RemoteEdit {
    over: Over,
    proto: remote::Proto,
    feed: &'static nodes::FeedKind,
    host: String,
    /// What to call it in the radio list. Optional, and worth having: an
    /// address says which machine and nothing about which aerial.
    label: String,
    /// Why the last attempt was refused, kept beside the field it belongs to
    /// rather than in the status line under the dial.
    err: Option<String>,
}

impl RemoteEdit {
    pub fn spyserver() -> Self {
        Self { proto: remote::Proto::SpyServer, ..Self::default() }
    }
}

#[derive(Clone, Default)]
pub struct FindEdit {
    mhz: String,
    full_control: bool,
    free: bool,
    err: Option<String>,
}

impl FindEdit {
    pub fn open() -> Self {
        crate::data::check(crate::data::Which::SpyServers);
        Self { free: true, ..Self::default() }
    }

    fn hz(&self) -> (Option<u64>, Option<String>) {
        match self.mhz.trim() {
            "" => (None, None),
            t => match t.parse::<f64>() {
                Ok(m) if m >= 0.0 => (Some((m * 1e6).round() as u64), None),
                _ => (None, Some(format!("{t} is not a frequency in MHz"))),
            },
        }
    }
}

impl Default for RemoteEdit {
    fn default() -> Self {
        Self {
            over: Over::Samples,
            proto: remote::Proto::IqStream,
            feed: nodes::FEED_KINDS[0],
            host: String::new(),
            label: String::new(),
            err: None,
        }
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

/// Where the record keeps the credential a dataset needs, so what is typed
/// into the row is what is saved and what fetches it.
fn key_slot(
    s: &mut crate::session::Session,
    which: crate::data::Which,
    index: usize,
) -> Option<&mut String> {
    match (which, index) {
        (crate::data::Which::CellTowers, 0) => Some(&mut s.opencellid_token),
        (crate::data::Which::Satellites(g), 0) if g.needs_login() => {
            Some(&mut s.spacetrack_identity)
        }
        (crate::data::Which::Satellites(g), _) if g.needs_login() => {
            Some(&mut s.spacetrack_password)
        }
        _ => None,
    }
}

fn key_of(s: &crate::session::Session, which: crate::data::Which, index: usize) -> Option<&String> {
    match (which, index) {
        (crate::data::Which::CellTowers, 0) => Some(&s.opencellid_token),
        (crate::data::Which::Satellites(g), 0) if g.needs_login() => Some(&s.spacetrack_identity),
        (crate::data::Which::Satellites(g), _) if g.needs_login() => Some(&s.spacetrack_password),
        _ => None,
    }
}

/// How the capture on the dial is to be cut.
#[derive(Clone, Debug)]
pub struct TrimEdit {
    /// The transmissions, or a window of the recording.
    bursts: bool,
    margin_ms: f64,
    threshold_db: f64,
    skip_s: f64,
    seconds: f64,
    /// Added to the new file's name, so a folder of cuts says which is which.
    tag: String,
    /// What the last cut came to, kept beside the card that asked for it.
    said: Option<std::result::Result<sources::Clipped, String>>,
}

impl Default for TrimEdit {
    fn default() -> Self {
        Self {
            bursts: true,
            margin_ms: 2.0,
            threshold_db: 6.0,
            skip_s: 0.0,
            seconds: 5.0,
            tag: "clip".into(),
            said: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wrote(name: &str, bytes: usize) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join("sr_capture_card");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(name);
        std::fs::write(&path, vec![0u8; bytes]).unwrap();
        path
    }

    /// The card is filled in from the name, because a capture from another
    /// program usually says half of it: retyping the half that was there is
    /// work nobody should do.
    #[test]
    fn a_half_named_capture_arrives_with_what_it_did_say_filled_in() {
        // 433.92 MHz and 32-bit float from the name, no rate.
        let path = wrote("sdrsharp_433.92M_IQ.cf32", 8 * 250_000);
        let e = CaptureEdit::new(path.clone(), false);
        assert_eq!(e.center, "433.920000");
        assert_eq!(e.rate, "");
        assert_eq!(e.format, common::SampleFormat::Cf32);
        assert!(e.resolve().is_err(), "a card with no rate cannot be accepted");

        let told = CaptureEdit { rate: "250k".into(), ..e };
        let c = told.resolve().expect("a described capture");
        assert_eq!(c.rate, common::Sps(250_000));
        assert_eq!(c.center, Some(common::Hz(433_920_000)));
        assert_eq!(c.format, common::SampleFormat::Cf32);
        assert!((c.seconds - 1.0).abs() < 0.01, "{} s", c.seconds);

        // A rate typed in full, and a centre left empty, which is a
        // recording at baseband rather than a refusal.
        let plain = CaptureEdit {
            rate: "2048000".into(),
            center: String::new(),
            ..CaptureEdit::new(path.clone(), true)
        };
        let c = plain.resolve().expect("a baseband capture");
        assert_eq!(c.rate, common::Sps(2_048_000));
        assert_eq!(c.center, None);

        // A file too short to hold one sample is refused, because the fault
        // is the format rather than the rate and nothing downstream would
        // say so.
        let stub = wrote("stub.iq", 2);
        let short = CaptureEdit { rate: "250k".into(), ..CaptureEdit::new(stub, false) };
        let short = CaptureEdit { format: common::SampleFormat::Cf32, ..short };
        let err = short.resolve().unwrap_err();
        assert!(err.contains("no whole samples"), "unhelpful: {err}");
    }

    /// The same card serves the transmit side, which reads the same names off
    /// the same disc.
    #[test]
    fn a_described_capture_becomes_something_the_transmitter_can_send() {
        let path = wrote("recording.iq", 2 * 250_000);
        let e = CaptureEdit { rate: "250k".into(), ..CaptureEdit::new(path.clone(), true) };
        assert!(e.to_air);
        let c = e.resolve().expect("a described capture");
        let tx = crate::radio::TxCapture::new(&c.path, c.rate, c.center, c.format)
            .expect("a capture to send");
        assert_eq!(tx.rate, common::Sps(250_000));
        assert_eq!(tx.format, common::SampleFormat::Cu8);
        assert!((tx.seconds - 1.0).abs() < 0.01, "{} s", tx.seconds);
        assert_eq!(tx.label(), "recording.iq");
        assert_eq!(crate::radio::TxCapture::open(&path), None, "the name says no rate");
    }
}
