//! The settings dialogs, and the one that creates a remote radio.
//!
//! Each is a leaf: it reads and writes the receiver's state and draws nothing
//! anybody else depends on, which is what makes them separable from the panes.

use super::*;

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
            Settings::App => crate::i18n::t("settings.title"),
        };
        let r = egui::containers::Modal::new(egui::Id::new(title))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(match which {
                    Settings::Radio | Settings::PacketLog => 420.0,
                    Settings::App | Settings::Data => 520.0,
                    Settings::Scanners | Settings::Memory => 560.0,
                    _ => 320.0,
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
                    Settings::App => self.app_settings(ui),
                }
                ui.add_space(12.0);
                ui.separator();
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button(crate::i18n::t("ui.close")).clicked() {
                            self.open = None;
                        }
                    });
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
        if self.memory.list.is_empty() {
            hint(ui, "Nothing saved yet. SAVE on a strip channel puts it here.");
        }
        let mut recall: Option<crate::memory::Saved> = None;
        let mut remove: Option<usize> = None;
        let groups: Vec<String> = self.memory.groups().iter().map(|g| g.to_string()).collect();
        egui::ScrollArea::vertical().max_height(420.0).show(ui, |ui| {
            for g in &groups {
                egui::CollapsingHeader::new(legend(g))
                    .id_salt(("memory", g))
                    .default_open(true)
                    .show(ui, |ui| {
                        let rows: Vec<(usize, crate::memory::Saved)> =
                            self.memory.in_group(g).map(|(i, s)| (i, s.clone())).collect();
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
            ui.add_space(4.0);
            hint(ui, &p.display().to_string());
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

        ui.horizontal(|ui| {
            let mut line = theme::Line::new()
                .legend("tuned to")
                .value(format!("{:.4} MHz", center / 1e6))
                .size(12.0)
                .gap(16.0)
                .legend("span")
                .value(format!("{:.0} kHz", rate / 1e3))
                .size(12.0)
                .gap(16.0)
                .legend("running");
            line = match active.is_empty() {
                false => line.heard(active.join(", ")),
                true => line.value("nothing").tint(theme::FAULT),
            };
            line.show(ui);
        });
        if active.is_empty() {
            hint(ui, "No block covers this frequency and span, so nothing is decoded here. Add one, or widen a range.");
        }
        ui.add_space(8.0);

        let mut remove = None;
        let mut tune_to = None;
        egui::ScrollArea::vertical().max_height(360.0).id_salt("scanrows").show(ui, |ui| {
            for (i, r) in rows.iter_mut().enumerate() {
                let on = active.iter().any(|n| n == &r.name);
                // Running blocks are framed, so which of them the span covers
                // is visible without reading every range.
                let frame = egui::Frame::NONE
                    .fill(if on { theme::WELL } else { theme::CHASSIS })
                    .stroke(Stroke::new(1.0, if on { theme::TRACE } else { theme::ETCH }))
                    .inner_margin(egui::Margin::symmetric(8, 6))
                    .corner_radius(2);
                frame.show(ui, |ui| {
                    ui.horizontal(|ui| {
                        // The switch that keeps a block in the table without
                        // running it, so turning auto off after pinning a few
                        // channels does not throw it away.
                        ui.checkbox(&mut r.enabled, "").on_hover_text(if r.enabled {
                            "running: click to switch off"
                        } else {
                            "off: click to run"
                        });
                        ui.add(
                            egui::TextEdit::singleline(&mut r.name)
                                .desired_width(112.0)
                                .hint_text("name"),
                        );
                        ui.add_space(4.0);
                        ui.label(legend("front"));
                        egui::ComboBox::from_id_salt(("front", i))
                            .selected_text(r.front.label())
                            .width(84.0)
                            .show_ui(ui, |ui| {
                                for f in crate::scanners::Front::all() {
                                    let label = f.label();
                                    // Keep the widths already typed when
                                    // switching back to banks.
                                    let pick = if matches!(f, crate::scanners::Front::Banks(_)) {
                                        r.banks_with_current_widths()
                                    } else {
                                        f
                                    };
                                    if ui
                                        .selectable_label(r.front.key() == pick.key(), label)
                                        .clicked()
                                    {
                                        r.front = pick;
                                    }
                                }
                            });
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("REMOVE").clicked() {
                                remove = Some(i);
                            }
                            if ui.add_enabled(!on, egui::Button::new("TUNE")).clicked() {
                                tune_to = Some((r.lo_mhz + r.hi_mhz) / 2.0);
                            }
                        });
                    });
                    ui.horizontal(|ui| {
                        ui.label(legend("range"));
                        mhz_field(ui, &mut r.lo_mhz);
                        ui.label(legend("to"));
                        mhz_field(ui, &mut r.hi_mhz);
                        ui.label(legend("MHz"));
                        ui.add_space(8.0);
                        ui.label(legend("span"));
                        ui.add(
                            egui::DragValue::new(&mut r.span_khz)
                                .speed(10.0)
                                .range(1.0..=20_000.0)
                                .suffix(" kHz"),
                        );
                    });
                    ui.horizontal(|ui| {
                        match &mut r.front {
                            // A bank front end is defined by its channel
                            // widths; everything else by the channels that
                            // have to be inside the span.
                            crate::scanners::Front::Banks(_) => {
                                ui.label(legend("widths"));
                                ui.add(
                                    egui::TextEdit::singleline(&mut r.widths)
                                        .desired_width(180.0)
                                        .hint_text("31.25, 125 kHz"),
                                );
                                ui.label(legend("kHz"));
                            }
                            _ => {
                                ui.label(legend("channels"));
                                ui.add(
                                    egui::TextEdit::singleline(&mut r.channels)
                                        .desired_width(180.0)
                                        // Not an example of a value: a hint
                                        // that looks like data reads as data
                                        // on a row that needs none.
                                        .hint_text("none needed"),
                                );
                                ui.label(legend("MHz"));
                                ui.add_space(6.0);
                                ui.label(legend("margin"));
                                ui.add(
                                    egui::DragValue::new(&mut r.margin_khz)
                                        .speed(1.0)
                                        .range(0.0..=1000.0)
                                        .suffix(" kHz"),
                                );
                            }
                        }
                    });
                    if r.to_scanner().is_none() {
                        ui.label(
                            egui::RichText::new("needs a name and a range that goes upwards")
                                .color(theme::FAULT)
                                .size(10.0),
                        );
                    }
                });
                ui.add_space(4.0);
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
                // frequency has to take effect without a retune.
                // Saved either way, since the table is configuration. It only
                // reaches the graph when the graph is the table's to build.
                if ui.add_enabled(dirty, egui::Button::new("SAVE")).clicked() {
                    let _ = table.save();
                    self.scanners = table.clone();
                    self.send(Cmd::Scanners(table.clone()));
                }
                if self.chain.edit.manual {
                    ui.label(
                        egui::RichText::new(crate::i18n::t("ui.manual_locked"))
                            .color(theme::LEGEND)
                            .size(11.0),
                    );
                }
                if ui.add_enabled(dirty, egui::Button::new("REVERT")).clicked() {
                    rows = self.scanners.list.iter().map(ScannerRow::from_scanner).collect();
                }
                if dirty {
                    ui.label(egui::RichText::new("unsaved").color(theme::READOUT).size(11.0));
                }
            });
        });
        if let Some(p) = crate::scanners::Scanners::path() {
            ui.add_space(4.0);
            hint(ui, &p.display().to_string());
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

        // Off until it is asked for, and remembered once it is: writing
        // every burst a receiver hears onto somebody's disc is a decision
        // for them to make. What is settable besides the switch is where it
        // goes and how large it may get.
        let mut on = self.log.path.is_some();
        let log_help = "Timings and frames as demodulated, a day per file, replayable.";
        if check_help(ui, &mut on, "Write every packet to disk", log_help).changed() {
            let dir = if on {
                self.log_dir.clone().or_else(crate::packetlog::PacketLog::default_dir)
            } else {
                None
            };
            self.log.path = dir.clone();
            self.send(Cmd::PacketLog(dir));
        }
        ui.add_space(8.0);

        // What the list shows, rather than what the receiver does. An
        // unrecognised burst is still reported, logged and replayable with
        // this off; it is only kept out of the table.
        let mut unknown = self.log.show_unknown;
        let unknown_help = "Bursts that decoded to no known protocol. They are the point of \
                            scanning an unfamiliar band, and on a noisy one they bury the \
                            decodes.";
        if check_help(ui, &mut unknown, "Show unrecognised bursts", unknown_help).changed() {
            self.log.show_unknown = unknown;
        }
        ui.add_space(8.0);

        row(ui, "directory", |ui| {
            let r = ui.add(
                egui::TextEdit::singleline(&mut self.log_dir_edit)
                    .desired_width(240.0)
                    .hint_text("where the files go"),
            );
            let typed = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if typed || ui.small_button("SET").clicked() {
                let dir = std::path::PathBuf::from(self.log_dir_edit.trim());
                if !self.log_dir_edit.trim().is_empty() {
                    self.log_dir = Some(dir.clone());
                    self.log.path = Some(dir.clone());
                    self.send(Cmd::PacketLog(Some(dir)));
                }
            }
        });

        let cap_help = "What the whole folder may take. The oldest days are deleted to keep it \
                        under, so the log rolls rather than stopping.";
        row_help(ui, "folder limit", cap_help, |ui| {
            let mut cap = self.log_cap_mb;
            egui::ComboBox::from_id_salt("log_cap")
                .selected_text(size_label(cap))
                .width(160.0)
                .show_ui(ui, |ui| {
                    for opt in [Some(512u64), Some(2048), Some(8192), Some(32_768), None] {
                        ui.selectable_value(&mut cap, opt, size_label(opt));
                    }
                });
            if cap != self.log_cap_mb {
                self.log_cap_mb = cap;
                self.send(Cmd::PacketLogCap(cap.map(|mb| mb << 20)));
            }
        });
        ui.add_space(10.0);

        reading(ui, "folder holds", human_bytes(bytes));
        reading(ui, "this session", format!("{logged} packets"));
        if full {
            ui.add(
                egui::Label::new(
                    egui::RichText::new(
                        "The log has stopped: today's file is over the limit on its own. \
                         Raise it here to start again.",
                    )
                    .small()
                    .color(theme::FAULT),
                )
                .wrap(),
            );
        }

        ui.add_space(12.0);
        ui.separator();
        ui.add_space(6.0);
        legend_help(ui, "feeds", "Packets from another receiver, over TCP.");
        ui.add_space(8.0);

        let status = self.radio.as_ref().map(|r| r.status.feeds.lock().clone()).unwrap_or_default();
        let mut remove = None;
        for (i, f) in self.feeds.iter().enumerate() {
            let live = status.iter().find(|s| s.spec == *f);
            ui.horizontal(|ui| {
                let (r, _) = ui.allocate_exact_size(Vec2::new(3.0, 16.0), Sense::hover());
                ui.painter().rect_filled(
                    r,
                    1.0,
                    match live {
                        Some(s) if s.connected => CRC_OK,
                        Some(_) => theme::FAULT,
                        None => theme::ETCH,
                    },
                );
                theme::Line::new().value(f.address()).size(11.0).legend(f.kind.name).show(ui);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button("REMOVE").clicked() {
                        remove = Some(i);
                    }
                    if let Some(s) = live {
                        theme::Line::new().legend(&format!("{} frames", s.frames)).show(ui);
                    }
                });
            });
            // A feed that is down says why. The alternative is a dark lamp and
            // a guess about whether it is the network, the port, or a receiver
            // somebody turned off.
            if let Some(e) = live.and_then(|s| s.error.clone()) {
                ui.add(egui::Label::new(egui::RichText::new(e).small().color(theme::FAULT)).wrap());
            }
            ui.add_space(6.0);
        }
        if let Some(i) = remove {
            self.feeds.remove(i);
            self.send(Cmd::Feeds(self.feeds.clone()));
        }

        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.add(
                egui::TextEdit::singleline(&mut self.feed_host)
                    .desired_width(170.0)
                    .hint_text("host, or host:port"),
            );
            egui::ComboBox::from_id_salt("feed_kind")
                .selected_text(self.feed_kind.name)
                .width(90.0)
                .show_ui(ui, |ui| {
                    for k in nodes::FEED_KINDS {
                        let on = self.feed_kind.name == k.name;
                        if ui.selectable_label(on, k.name).clicked() {
                            self.feed_kind = k;
                        }
                    }
                });
            if ui.button("ADD").clicked() {
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

        legend_help(ui, t("settings.language"), t("settings.language.help"));
        let mut lang = crate::i18n::language();
        egui::ComboBox::from_id_salt("app-language")
            .selected_text(lang.label())
            .width(ui.available_width())
            .show_ui(ui, |ui| {
                for l in crate::i18n::Language::ALL {
                    ui.selectable_value(&mut lang, l, l.label());
                }
            });
        crate::i18n::set_language(lang);
        ui.add_space(10.0);

        legend_help(ui, t("settings.country"), t("settings.country.help"));
        let current = crate::locale::by_code(&self.country);
        let mut pick: Option<&'static crate::locale::Country> = None;
        egui::ComboBox::from_id_salt("app-country")
            .selected_text(current.map(|c| c.name).unwrap_or("—"))
            .width(ui.available_width())
            .show_ui(ui, |ui| {
                for c in crate::locale::COUNTRIES {
                    let on = current.is_some_and(|s| s.code == c.code);
                    if ui.selectable_label(on, c.name).clicked() && !on {
                        pick = Some(c);
                    }
                }
            });
        if let Some(c) = pick {
            self.country = c.code.to_string();
            // The cell export is fetched per country, so the dataset pane
            // has to hear about this to know which one it would fetch.
            crate::data::set_country(&self.country);
            // A country decides the plan the first time and then stops having
            // an opinion, so choosing one after overriding the plan puts the
            // override back rather than leaving a mismatch nobody asked for.
            crate::bands::set_plan(c.plan);
            // The map has to open somewhere. A capital city is wrong by a
            // couple of hundred miles, which is close enough to draw with and
            // is replaced the moment a real position is typed in.
            if self.location.is_none() {
                self.set_location(c.centre.0, c.centre.1);
                self.station_edit = None;
            }
        }
        ui.add_space(10.0);

        // Sound devices. Here rather than with the radio's controls because
        // they are not the radio: which speaker the mix comes out of and
        // which microphone a keyed channel transmits from are properties of
        // this machine.
        legend_help(ui, "Speaker", "Where the mix, the calls and any replay come out.");
        let mut out = self.audio_out.clone();
        if device_combo(ui, "app-audio-out", &mut out, audio::AudioPlayer::devices()) {
            self.audio_out = out;
            self.send_audio();
        }
        ui.add_space(10.0);

        let mic = "What a keyed channel transmits. Held open while a channel is set to MIC, so \
                   the meter moves before you key.";
        legend_help(ui, "Microphone", mic);
        let mut input = self.audio_in.clone();
        if device_combo(ui, "app-audio-in", &mut input, audio::AudioCapture::devices()) {
            self.audio_in = input;
            self.send_audio();
        }
        ui.add_space(10.0);

        // Above the band plan rather than under the version: it is a question
        // about the interface, like the language and the sound devices, and
        // somebody looking for it has just come from the dashboard.
        legend_help(
            ui,
            "Dashboard",
            "Quick start and receiver status, as the first view. Off takes its tab away and \
             opens the receiver on the spectrum.",
        );
        let mut on = self.dashboard;
        if ui.checkbox(&mut on, "Open on the dashboard").changed() {
            match on {
                true => self.dashboard = true,
                false => self.hide_dashboard(),
            }
        }
        ui.add_space(10.0);

        legend_help(ui, t("settings.band_plan"), t("settings.band_plan.help"));
        let mut plan = crate::bands::plan();
        egui::ComboBox::from_id_salt("app-band-plan")
            .selected_text(plan.label())
            .width(ui.available_width())
            .show_ui(ui, |ui| {
                for p in crate::bands::Plan::ALL {
                    ui.selectable_value(&mut plan, p, p.label());
                }
            });
        crate::bands::set_plan(plan);
        ui.add_space(4.0);
        // The plan is abstract until it is applied to the frequency in front
        // of you, and this is the one line that makes the choice concrete.
        hint(
            ui,
            &format!(
                "{} here is {}",
                fmt_hz(self.center),
                crate::bands::name_at_in(plan, self.center)
            ),
        );
        ui.add_space(10.0);

        ui.separator();
        ui.add_space(6.0);
        legend_help(ui, t("settings.position"), t("settings.position.help"));
        let mut edit = self.station_edit.take();
        let set = map_pane::Map::station_row(ui, self.location, &mut edit);
        self.station_edit = edit;
        if let Some((lat, lon)) = set {
            self.set_location(lat, lon);
            self.station_edit = None;
        }
        ui.add_space(10.0);

        self.gps_settings(ui);

        ui.separator();
        ui.add_space(6.0);
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
        legend_help(ui, t("settings.version"), t("settings.version.help"));
        ui.add_space(4.0);

        let state = crate::update::state();
        let busy = matches!(state, crate::update::State::Checking);
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
            crate::update::State::Unchecked => hint(ui, "Not checked yet."),
            crate::update::State::Checking => {
                hint(ui, "Asking GitHub for the latest release.");
            }
            crate::update::State::Current(r) => {
                hint(ui, &format!("Up to date. Latest release is {}.", r.version));
            }
            crate::update::State::Newer(r) => {
                theme::Line::new()
                    .legend("available")
                    .value(&r.version)
                    .tint(theme::OK)
                    .size(13.0)
                    .show(ui);
                match &r.asset {
                    Some(a) => {
                        hint(ui, &format!("{} ({})", a.name, crate::data::fmt_bytes(a.bytes)))
                    }
                    None => hint(
                        ui,
                        &format!("No {} archive in that release.", crate::update::platform()),
                    ),
                }
                if !r.page.is_empty() && ui.button(legend("OPEN THE RELEASE")).clicked() {
                    ui.ctx().open_url(egui::OpenUrl::new_tab(r.page.clone()));
                }
            }
            crate::update::State::Failed(e) => {
                ui.label(egui::RichText::new(e).small().color(theme::FAULT));
            }
        }
        // The check runs on a thread of its own, so without this the answer
        // sits unshown until the pointer moves.
        if busy {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(200));
        }
    }

    /// Where the position comes from, and what the survey does with it.
    ///
    /// Under the station position rather than in a pane of its own, because
    /// a GPS is not a feature of the device database: it is the other way of
    /// answering the question the box above asks, and a receiver that is
    /// moving should say so where somebody would go to type a position by
    /// hand.
    fn gps_settings(&mut self, ui: &mut egui::Ui) {
        legend_help(
            ui,
            "GPS",
            "A fix moves the station position, which is the position everything else works \
             from. The reader always runs and looks for a gpsd on this machine, so nothing \
             need be set here unless the receiver is a serial port or a daemon elsewhere: a \
             device path such as /dev/ttyACM0, or gpsd:host.",
        );
        let mut set: Option<Option<gps::Transport>> = None;
        ui.horizontal(|ui| {
            let text = self.survey.gps_edit.get_or_insert_with(|| {
                self.survey.gps.as_ref().map(|t| t.to_string()).unwrap_or_default()
            });
            let r = ui.add(
                egui::TextEdit::singleline(text)
                    .desired_width(190.0)
                    .hint_text(gps::Transport::LOCAL_GPSD)
                    .font(FontId::new(12.0, FontFamily::Name(theme::READOUT_FONT.into()))),
            );
            let typed = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if typed || ui.small_button("SET").clicked() {
                // An empty box is the local gpsd rather than nothing: there
                // is no off, since a receiver with no fix simply leaves the
                // station where it was put.
                set = Some(gps::Transport::parse(text));
            }
            if self.survey.gps.is_some() && ui.small_button("AUTO").clicked() {
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
        let line = match (crate::station::connected(), crate::station::fix()) {
            (_, Some(f)) => {
                let sats = f.sats.map(|n| format!(", {n} satellites")).unwrap_or_default();
                let how = f.accuracy_m().map(|m| format!(", ±{m:.0} m")).unwrap_or_default();
                format!("{:.5}, {:.5}{sats}{how}, {} fixes", f.lat, f.lon, crate::station::fixes())
            }
            // Waiting says nothing on its own: an antenna indoors and an
            // antenna unplugged look the same for the first minute, and the
            // satellite counts tell them apart.
            (true, None) => match crate::station::sky() {
                Some(s) => {
                    format!("connected, no fix yet: {} of {} satellites used", s.used, s.seen)
                }
                None => "connected, waiting for a fix".into(),
            },
            (false, None) => {
                "nothing answering: the station position is whatever is set above".to_string()
            }
        };
        hint(ui, &line);
        // The reader is not the radio's, so this pane keeps its own clock:
        // without it a fix arriving while nothing else is moving would sit
        // unshown until the pointer did.
        ui.ctx().request_repaint_after(std::time::Duration::from_millis(500));
        ui.add_space(6.0);

        // The survey is what a position is for, and the switch belongs beside
        // it rather than three panes away.
        let mut on = self.survey.path.is_some();
        let survey_help = "One row per transmitter heard, with the places it was heard from. \
                           The packet log keeps the transmissions; this keeps the transmitters.";
        if check_help(ui, &mut on, "Record a device database", survey_help).changed() {
            self.set_survey(!on, None);
        }
        if let Some(p) = self.survey.path.as_ref() {
            hint(ui, &p.display().to_string());
        }
        ui.add_space(10.0);
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
        hint(ui, t("settings.data.help"));
        ui.add_space(6.0);

        let rows = crate::data::status();
        let busy = rows.iter().any(|r| r.busy);
        egui::ScrollArea::vertical().max_height(420.0).show(ui, |ui| {
            for r in &rows {
                let frame = egui::Frame::NONE
                    .fill(theme::WELL)
                    .stroke(Stroke::new(1.0, theme::ETCH))
                    .inner_margin(egui::Margin::symmetric(8, 6))
                    .corner_radius(2);
                frame.show(ui, |ui| {
                    ui.horizontal(|ui| {
                        theme::Line::new()
                            .value(r.which.label())
                            .size(13.0)
                            .note(r.which.publisher())
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
                            // button that vanishes under the pointer is a button
                            // that gets pressed twice.
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
                    });
                    theme::Line::new()
                        .legend("held")
                        .value(match r.rows {
                            Some(n) => format!("{n} rows"),
                            // Cached but not parsed is the ordinary state for the
                            // registries, which are read the first time something
                            // asks them a question.
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
                    // The terms on the row, not in a document nobody opens.
                    // OpenCelliD asks in writing for a visible credit and a
                    // link, and a receiver that draws its masts while saying
                    // nothing is not complying with that.
                    hint(ui, r.which.terms());
                    if let Some(e) = &r.error {
                        ui.label(egui::RichText::new(e).small().color(theme::FAULT));
                    }
                    if let Some(b) = r.blocked {
                        hint(ui, b);
                    }
                    for (i, k) in r.which.keys().iter().enumerate() {
                        self.key_field(ui, r.which, i, *k);
                    }
                });
                ui.add_space(4.0);
            }
        });

        ui.add_space(6.0);

        ui.horizontal(|ui| {
            if ui.add_enabled(!busy, egui::Button::new(legend(t("ui.refresh_all")))).clicked() {
                // A dataset that cannot be fetched is skipped rather than
                // failed: refresh all is a convenience, not a demand for a
                // token.
                for w in crate::data::Which::all().iter().filter(|w| w.blocked().is_none()) {
                    crate::data::refresh(*w);
                }
            }
            if let Some(dir) = crate::data::cache_dir() {
                ui.label(
                    egui::RichText::new(dir.display().to_string()).small().color(theme::LEGEND),
                );
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
        ui.horizontal(|ui| {
            theme::Line::new().legend(k.label).show(ui);
            ui.add(
                egui::TextEdit::singleline(slot)
                    .desired_width(ui.available_width())
                    .password(k.secret)
                    .hint_text(k.hint),
            )
            .on_hover_text(k.help);
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
                ui.set_width(440.0);
                modal_title(ui, "Feed wigle.net");
                hint(
                    ui,
                    "Bluetooth devices and cells heard with a position are written as WiGLE \
                     CSV and uploaded. Everything else the survey records stays on this \
                     machine: the format has no type for an aircraft or a pager, and a row \
                     filed under the wrong one cannot be taken back.",
                );
                ui.add_space(10.0);

                legend_help(
                    ui,
                    "API name",
                    "From wigle.net/account, where it is shown beside the token. It is not \
                     the name you log in with.",
                );
                ui.add(
                    egui::TextEdit::singleline(&mut self.survey.wigle.name)
                        .desired_width(ui.available_width())
                        .hint_text("AID00000000000000000000000000000"),
                );
                ui.add_space(8.0);

                legend_help(
                    ui,
                    "API token",
                    "The token from the same page. It is stored in the session file in plain \
                     text, so treat it as a password that lives on this machine.",
                );
                ui.add(
                    egui::TextEdit::singleline(&mut self.survey.wigle.token)
                        .password(true)
                        .desired_width(ui.available_width()),
                );
                ui.add_space(10.0);

                let donate_help = "Lets wigle.net licence what you upload commercially. Off \
                                   unless you say otherwise: they are your observations to \
                                   give away.";
                if check_help(
                    ui,
                    &mut self.survey.wigle.donate,
                    "Allow commercial use",
                    donate_help,
                )
                .changed()
                {
                    apply = true;
                }
                let on_help = "While this is on, every Bluetooth device and cell heard with a \
                               position is spooled to disc and uploaded when there is a \
                               network. A drive with no coverage sends when it gets home.";
                if check_help(ui, &mut self.survey.wigle.on, "Upload while receiving", on_help)
                    .changed()
                {
                    apply = true;
                }
                ui.add_space(10.0);

                // What it is actually doing. Three numbers and the last
                // refusal, which is everything an operator can act on.
                match self.survey.wigle.status.as_ref() {
                    Some(s) => {
                        theme::Line::new()
                            .legend("waiting")
                            .value(format!("{} rows in {} files", s.queued_rows, s.queued_files))
                            .legend("uploaded")
                            .value(format!("{} rows in {} files", s.sent_rows, s.sent_files))
                            .size(11.0)
                            .show(ui);
                        if let Some(t) = &s.transaction {
                            hint(ui, &format!("last transaction {t}"));
                        }
                        hint(ui, &format!("spool {}", s.spool.display()));
                        if let Some(e) = &s.error {
                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new(e).small().color(theme::FAULT),
                                )
                                .wrap(),
                            );
                        }
                    }
                    None => hint(ui, "no receiver running, so nothing is being collected"),
                }

                ui.add_space(12.0);
                ui.separator();
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button(crate::i18n::t("ui.close")).clicked() {
                            close = true;
                        }
                        if ui.button("APPLY").clicked() {
                            apply = true;
                        }
                    });
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
                ui.set_width(440.0);
                modal_title(ui, "Feed beacondb.net");
                hint(
                    ui,
                    "Bluetooth devices and cells heard with a position are submitted to \
                     beaconDB, which is crowd-sourced, needs no account, and publishes what \
                     it collects. Everything else the survey records stays on this machine: \
                     there is no beacon type for an aircraft or a pager.",
                );
                ui.add_space(6.0);
                hint(
                    ui,
                    "What is submitted is where this receiver was when it heard something, so \
                     a drive is a track of where you have been. Levels are not sent: this \
                     receiver measures dBFS and the field means dBm.",
                );
                ui.add_space(10.0);

                let on_help = "While this is on, every Bluetooth device and cell heard with a \
                               position is spooled to disc and submitted when there is a \
                               network. A drive with no coverage sends when it gets home.";
                if check_help(ui, &mut self.survey.beacondb.on, "Submit while receiving", on_help)
                    .changed()
                {
                    apply = true;
                }
                let ask_help = "Draws a position for a cell you have decoded that the \
                                OpenCelliD export has no row for, as a cross with the \
                                accuracy beaconDB gives it. Asking tells beaconDB which \
                                cells this receiver has heard, which is why it is separate \
                                from submitting.";
                if check_help(
                    ui,
                    &mut self.survey.beacondb.lookup,
                    "Ask where a heard cell is",
                    ask_help,
                )
                .changed()
                {
                    apply = true;
                }
                ui.add_space(10.0);

                match self.survey.beacondb.status.as_ref() {
                    Some(s) => {
                        theme::Line::new()
                            .legend("waiting")
                            .value(format!(
                                "{} observations in {} files",
                                s.queued_items, s.queued_files
                            ))
                            .legend("submitted")
                            .value(format!(
                                "{} observations in {} files",
                                s.sent_items, s.sent_files
                            ))
                            .size(11.0)
                            .show(ui);
                        hint(ui, &format!("spool {}", s.spool.display()));
                        if let Some(e) = &s.error {
                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new(e).small().color(theme::FAULT),
                                )
                                .wrap(),
                            );
                        }
                    }
                    None => hint(ui, "no receiver running, so nothing is being collected"),
                }

                ui.add_space(12.0);
                ui.separator();
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button(crate::i18n::t("ui.close")).clicked() {
                            close = true;
                        }
                        if ui.button("APPLY").clicked() {
                            apply = true;
                        }
                    });
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
                ui.set_width(440.0);
                modal_title(ui, "Publish to Home Assistant");
                hint(
                    ui,
                    "Every transmitter the decoders can name becomes a device in Home \
                     Assistant over MQTT discovery, and every number they recover becomes an \
                     entity under it: a weather station's temperature, a tyre sensor's \
                     pressure, and the level each was heard at.",
                );
                ui.add_space(6.0);
                hint(
                    ui,
                    "It publishes to the broker Home Assistant is already using, in plain \
                     MQTT. What goes out is what your neighbours are transmitting as well as \
                     what you are, so point it at a broker on your own network.",
                );
                ui.add_space(10.0);

                let ha = &mut self.survey.homeassistant;
                legend_help(
                    ui,
                    "broker",
                    "The host running Mosquitto, or whatever Home Assistant's MQTT \
                     integration is pointed at.",
                );
                ui.horizontal(|ui| {
                    ui.add(
                        egui::TextEdit::singleline(&mut ha.host)
                            .desired_width(ui.available_width() - 90.0)
                            .hint_text("homeassistant.local"),
                    );
                    ui.add(
                        egui::TextEdit::singleline(&mut ha.port)
                            .desired_width(70.0)
                            .hint_text("1883"),
                    );
                });
                ui.add_space(8.0);

                legend_help(ui, "user", "Blank where the broker allows anonymous clients.");
                ui.add(
                    egui::TextEdit::singleline(&mut ha.username)
                        .desired_width(ui.available_width()),
                );
                ui.add_space(6.0);
                legend_help(
                    ui,
                    "password",
                    "Kept in the session file in plain text, so treat it as a password that \
                     lives on this machine.",
                );
                ui.add(
                    egui::TextEdit::singleline(&mut ha.password)
                        .password(true)
                        .desired_width(ui.available_width()),
                );
                ui.add_space(8.0);

                legend_help(
                    ui,
                    "discovery prefix",
                    "What Home Assistant listens under. Blank means homeassistant, which is \
                     what it uses unless somebody changed it.",
                );
                ui.add(
                    egui::TextEdit::singleline(&mut ha.prefix)
                        .desired_width(ui.available_width())
                        .hint_text("homeassistant"),
                );
                ui.add_space(6.0);
                legend_help(
                    ui,
                    "topic",
                    "What this receiver's own topics live under. Blank means waveshark.",
                );
                ui.add(
                    egui::TextEdit::singleline(&mut ha.topic)
                        .desired_width(ui.available_width())
                        .hint_text("waveshark"),
                );
                ui.add_space(8.0);

                legend_help(
                    ui,
                    "publish",
                    "Which kinds of transmitter are worth a permanent device, comma \
                     separated. Blank is everything, which on a Bluetooth band means the \
                     handsets walking past: those rotate their address every quarter of an \
                     hour, and Home Assistant keeps every one it is told about. ism,wmbus \
                     is a house's own sensors and meters.",
                );
                ui.add(
                    egui::TextEdit::singleline(&mut ha.spaces)
                        .desired_width(ui.available_width())
                        .hint_text("everything"),
                );
                ui.add_space(10.0);

                let on_help = "While this is on, every named transmitter heard is announced \
                               once and then reported at most every few seconds. A city \
                               centre holds thousands of Bluetooth addresses, so the node \
                               stops at a couple of hundred devices.";
                if check_help(ui, &mut ha.on, "Publish while receiving", on_help).changed() {
                    apply = true;
                }
                ui.add_space(10.0);

                match self.survey.homeassistant.status.as_ref() {
                    Some(s) if s.configured => {
                        theme::Line::new()
                            .legend(if s.connected { "connected" } else { "connecting" })
                            .value(s.host.clone())
                            .legend("devices")
                            .value(s.devices.to_string())
                            .legend("published")
                            .value(s.published.to_string())
                            .size(11.0)
                            .show(ui);
                        if s.dropped > 0 {
                            hint(
                                ui,
                                &format!(
                                    "{} readings dropped: the broker was not keeping up",
                                    s.dropped
                                ),
                            );
                        }
                        if let Some(e) = &s.error {
                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new(e).small().color(theme::FAULT),
                                )
                                .wrap(),
                            );
                        }
                    }
                    Some(_) => hint(ui, "nothing is being published"),
                    None => hint(ui, "no receiver running, so nothing is being published"),
                }

                ui.add_space(12.0);
                ui.separator();
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button(crate::i18n::t("ui.close")).clicked() {
                            close = true;
                        }
                        if ui.button("APPLY").clicked() {
                            apply = true;
                        }
                    });
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
                ui.set_width(420.0);
                modal_title(ui, "Add remote radio");

                legend_help(ui, "protocol", edit.kind.help());
                egui::ComboBox::from_id_salt("remote-kind")
                    .selected_text(edit.kind.label())
                    .width(ui.available_width())
                    .show_ui(ui, |ui| {
                        for k in RemoteKind::ALL {
                            ui.selectable_value(&mut edit.kind, *k, k.label());
                        }
                    });
                ui.add_space(10.0);

                ui.label(legend("address"));
                let field = ui.add(
                    egui::TextEdit::singleline(&mut edit.host)
                        .desired_width(ui.available_width())
                        .hint_text(edit.kind.placeholder()),
                );
                ui.add_space(10.0);

                legend_help(
                    ui,
                    "name",
                    "What the radio list calls it. An address says which machine and nothing \
                     about which aerial.",
                );
                let name = ui.add(
                    egui::TextEdit::singleline(&mut edit.label)
                        .desired_width(ui.available_width())
                        .hint_text("loft dongle"),
                );
                if name.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    add = true;
                }
                // Focused so the address can be typed straight away, but only
                // while nothing else holds it: taking it back every frame
                // would fight the buttons below.
                if ui.memory(|m| m.focused().is_none()) {
                    field.request_focus();
                }
                if field.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    add = true;
                }
                if let Some(e) = &edit.err {
                    ui.add_space(6.0);
                    ui.add(
                        egui::Label::new(egui::RichText::new(e).small().color(theme::FAULT)).wrap(),
                    );
                }

                ui.add_space(12.0);
                ui.separator();
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("ADD").clicked() {
                            add = true;
                        }
                        if ui.button(crate::i18n::t("ui.close")).clicked() {
                            close = true;
                        }
                    });
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
    pub(super) fn open_capture(&mut self, ctx: &egui::Context) {
        let start = crate::chain::default_capture_dir();
        let _ = std::fs::create_dir_all(&start);
        let Some(path) = rfd::FileDialog::new()
            .set_title("Replay a capture")
            .set_directory(&start)
            .add_filter("IQ captures", &["cu8", "cs8", "cs16", "cf32", "data", "sigmf-data"])
            .pick_file()
        else {
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
            ui.label(legend("no radio running"));
            return;
        };
        let controls = radio.status.radio();
        if controls.stages.is_empty() && controls.toggles.is_empty() && controls.choices.is_empty()
        {
            ui.label(legend("this device has no adjustable stages"));
        }
        // Every control here writes the one record and applies it at the end,
        // which is the same route a restore and a reset take: there is no
        // second way to set the radio that could disagree with the first.
        let mut changed = false;

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
            ui.horizontal(|ui| {
                ui.label(legend(&stage.label));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    help(ui, &steps);
                    if stage.auto {
                        let mut on = auto;
                        if ui.checkbox(&mut on, "Auto").changed() {
                            let mode = if on { GainMode::Auto } else { GainMode::Manual(db) };
                            self.radio_settings.set_gain(&stage.name, mode);
                            changed = true;
                        }
                    }
                    // Under AUTO the number is the hardware's business and
                    // showing a stale one invites the operator to believe it.
                    let text = if auto { "auto".to_string() } else { format!("{db:.1} dB") };
                    ui.label(value(text).size(11.0));
                });
            });
            // Snapped as it is dragged, because the hardware does it anyway:
            // a slider that glides between values the tuner cannot reach shows
            // a number the receiver is not using.
            let slider = egui::Slider::new(&mut db, lo..=hi).show_value(false);
            if ui.add_enabled(!auto, slider).changed() {
                let want = stage.quantise(db);
                self.radio_settings.set_gain(&stage.name, GainMode::Manual(want));
                changed = true;
            }
            ui.add_space(10.0);
        }

        // The transmit gain, which is one number for the radio: a channel's
        // own trim is added to it when that channel is keyed. Separate from
        // the stages above because none of those are in circuit while
        // transmitting, and because this one radiates.
        if let Some(stage) = controls.tx_stages.iter().find(|s| s.name == "txvga") {
            ui.separator();
            ui.add_space(6.0);
            let mut db = self.radio_settings.tx_gain_db;
            ui.horizontal(|ui| {
                ui.label(legend("Transmit gain"));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    help(
                        ui,
                        "What every keyed channel transmits at, before its own trim. Start at \
                         the bottom and into a dummy load.",
                    );
                    ui.label(value(format!("{db:.0} dB")).size(11.0));
                });
            });
            let (lo, hi) = (*stage.range.start(), *stage.range.end());
            if ui.add(egui::Slider::new(&mut db, lo..=hi).show_value(false)).changed() {
                self.radio_settings.tx_gain_db = stage.quantise(db);
                changed = true;
            }
            ui.add_space(10.0);
        }

        if !controls.choices.is_empty() {
            ui.separator();
            ui.add_space(6.0);
            for c in &controls.choices {
                legend_help(ui, &c.label, &c.help);
                let mut picked = c.selected.clone();
                egui::ComboBox::from_id_salt(format!("radio-choice-{}", c.name))
                    .selected_text(&picked)
                    .width(ui.available_width())
                    .show_ui(ui, |ui| {
                        for opt in &c.options {
                            ui.selectable_value(&mut picked, opt.clone(), opt);
                        }
                    });
                if picked != c.selected {
                    self.radio_settings.set_choice(&c.name, &picked);
                    changed = true;
                }
                ui.add_space(8.0);
            }
        }

        if !controls.toggles.is_empty() {
            ui.separator();
            ui.add_space(6.0);
            for t in &controls.toggles {
                let mut on = t.on;
                if check_help(ui, &mut on, &t.label, &t.help).changed() {
                    self.radio_settings.set_toggle(&t.name, on);
                    changed = true;
                }
                ui.add_space(8.0);
            }
        }

        ui.separator();
        ui.add_space(6.0);
        let ppm_help = "The reference oscillator is a few tens of parts per million out on a \
                        cheap dongle, which is a kilohertz or two at 145 MHz and rather more \
                        higher up. Tune a known carrier and correct until it sits on its \
                        nominal frequency. Saved against this radio, so each one keeps its \
                        own figure.";
        row_help(ui, "Correction", ppm_help, |ui| {
            let mut ppm = self.radio_settings.ppm;
            if ui
                .add(egui::DragValue::new(&mut ppm).speed(0.5).range(-200.0..=200.0).suffix(" ppm"))
                .changed()
            {
                self.set_ppm(ppm);
                changed = true;
            }
        });
        ui.add_space(10.0);

        let mut dc = self.dc_block;
        let dc_help = "A direct conversion receiver leaks its own local oscillator into the \
                       middle of the span, where it looks exactly like a carrier on the \
                       frequency you are tuned to. This measures the offset and subtracts it.";
        if check_help(ui, &mut dc, "Remove the DC spur", dc_help).changed() {
            self.dc_block = dc;
            self.send(Cmd::DcBlock(dc));
        }

        ui.add_space(12.0);
        ui.separator();
        ui.add_space(6.0);
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
        legend_help(
            ui,
            "raw capture",
            "The whole span to one file, as it arrives. This is the recording to \
             make when the receiver shows a transmission and reads nothing from \
             it: replaying the file puts the same samples through the same \
             graph, so a decoder can be changed and tried again.",
        );
        ui.add_space(8.0);

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
        let mut on = cap_on;
        if ui.checkbox(&mut on, "Capture the raw span").changed() {
            self.set_capture(on);
        }
        ui.add_space(6.0);

        let cap_help = "What the whole folder may take. Nothing here is deleted: a capture is \
                        evidence of a signal that may not come again, so writing stops instead.";
        row_help(ui, "folder limit", cap_help, |ui| {
            let mut cap = self.capture_cap_mb;
            egui::ComboBox::from_id_salt("capture_cap")
                .selected_text(size_label(cap))
                .width(160.0)
                .show_ui(ui, |ui| {
                    for opt in [Some(1024u64), Some(4096), Some(16_384), Some(65_536), None] {
                        ui.selectable_value(&mut cap, opt, size_label(opt));
                    }
                });
            if cap != self.capture_cap_mb {
                self.capture_cap_mb = cap;
                self.send(Cmd::CaptureCap(cap.map(|mb| mb << 20).unwrap_or(0)));
            }
        });
        ui.add_space(8.0);

        // Where the files are, and a way into it. A capture is made to be
        // replayed, trimmed or sent somewhere, all of which happen outside
        // this program, and a path that can only be read off the screen and
        // typed again is a path nobody uses.
        let dir = crate::chain::default_capture_dir();
        row(ui, "folder", |ui| {
            if ui.small_button("OPEN").on_hover_text(dir.display().to_string()).clicked() {
                // Created first: the folder does not exist until the first
                // capture is written, and a file manager handed a missing
                // path either opens nothing or opens somewhere else.
                let _ = std::fs::create_dir_all(&dir);
                ui.ctx().open_url(egui::OpenUrl::new_tab(file_url(&dir)));
            }
            ui.add(egui::Label::new(value(dir.display().to_string()).size(11.0)).truncate());
        });
        ui.add_space(4.0);

        // The folder first: it is the number the limit above is about, and
        // showing only the file being written made a folder of two gigabytes
        // read as seventy megabytes.
        reading(ui, "folder holds", human_bytes(cap_folder));
        if cap_bytes > 0 {
            reading(ui, "this file", human_bytes(cap_bytes));
        }
        if let Some(f) = &cap_file {
            row(ui, "file", |ui| {
                ui.add(egui::Label::new(value(f).size(11.0)).wrap());
            });
        }
        if cap_full {
            ui.add(
                egui::Label::new(
                    egui::RichText::new(
                        "The capture stopped: the folder is at its limit. Raise it, or \
                         move the captures somewhere else.",
                    )
                    .small()
                    .color(theme::FAULT),
                )
                .wrap(),
            );
        }
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
