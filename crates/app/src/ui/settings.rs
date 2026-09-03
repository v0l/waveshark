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
            Settings::App => crate::i18n::t("settings.title"),
        };
        let r = egui::containers::Modal::new(egui::Id::new(title))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(match which {
                    Settings::Radio | Settings::PacketLog => 420.0,
                    Settings::App => 520.0,
                    Settings::Scanners => 560.0,
                    _ => 320.0,
                });
                modal_title(ui, title);
                match which {
                    Settings::Spectrum => self.spectrum_settings(ui),
                    Settings::Waterfall => self.waterfall_settings(ui),
                    Settings::Radio => self.radio_settings(ui),
                    Settings::PacketLog => self.packet_log_settings(ui),
                    Settings::Scanners => self.scanner_settings(ui),
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
        let table = crate::scanners::Scanners { list: live };
        let active: Vec<String> =
            table.active(center, rate).into_iter().map(|s| s.name.clone()).collect();

        ui.horizontal(|ui| {
            ui.label(legend("tuned to"));
            ui.label(value(format!("{:.4} MHz", center / 1e6)).size(12.0));
            ui.add_space(8.0);
            ui.label(legend("span"));
            ui.label(value(format!("{:.0} kHz", rate / 1e3)).size(12.0));
            ui.add_space(8.0);
            ui.label(legend("running"));
            match active.is_empty() {
                false => ui.label(
                    egui::RichText::new(active.join(", ")).color(theme::TRACE).size(13.0),
                ),
                true => ui.label(egui::RichText::new("nothing").color(theme::FAULT).size(13.0)),
            };
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
                        ui.add(
                            egui::TextEdit::singleline(&mut r.name)
                                .desired_width(120.0)
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
                if self.chain_edit.manual {
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

        // The log is on by default and stays on: the transmission worth
        // having is always the one before somebody thought to press record.
        // What is settable is where it goes and how large it may get.
        let mut on = self.packet_log.is_some();
        if ui.checkbox(&mut on, "Write every packet to disk").changed() {
            let dir = if on {
                self.log_dir
                    .clone()
                    .or_else(crate::packetlog::PacketLog::default_dir)
            } else {
                None
            };
            self.packet_log = dir.clone();
            self.send(Cmd::PacketLog(dir));
        }
        hint(ui, "Timings and frames as demodulated, a day per file, replayable.");
        ui.add_space(8.0);

        // What the list shows, rather than what the receiver does. An
        // unrecognised burst is still reported, logged and replayable with
        // this off; it is only kept out of the table.
        let mut unknown = self.show_unknown;
        if ui.checkbox(&mut unknown, "Show unrecognised bursts").changed() {
            self.show_unknown = unknown;
        }
        hint(
            ui,
            "Bursts that decoded to no known protocol. They are the point of scanning an unfamiliar band, and on a noisy one they bury the decodes.",
        );
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
                    self.packet_log = Some(dir.clone());
                    self.send(Cmd::PacketLog(Some(dir)));
                }
            }
        });

        row(ui, "size limit", |ui| {
            let mut cap = self.log_cap_mb;
            egui::ComboBox::from_id_salt("log_cap")
                .selected_text(match cap {
                    Some(mb) => format!("{mb} MB per day"),
                    None => "no limit".into(),
                })
                .width(160.0)
                .show_ui(ui, |ui| {
                    for opt in [Some(128u64), Some(512), Some(2048), Some(8192), None] {
                        let label = match opt {
                            Some(mb) => format!("{mb} MB per day"),
                            None => "no limit".into(),
                        };
                        ui.selectable_value(&mut cap, opt, label);
                    }
                });
            if cap != self.log_cap_mb {
                self.log_cap_mb = cap;
                self.send(Cmd::PacketLogCap(cap.map(|mb| mb << 20)));
            }
        });
        hint(ui, "A runaway guard, not a budget.");
        ui.add_space(10.0);

        row(ui, "today", |ui| {
            ui.label(value(format!("{} in {logged} packets", human_bytes(bytes))).size(11.0));
        });
        if full {
            ui.add(
                egui::Label::new(
                    egui::RichText::new(
                        "The log has stopped: the day's file reached the limit. \
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
        ui.label(legend("feeds"));
        hint(ui, "Packets from another receiver, over TCP.");
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
                ui.label(value(f.address()).size(11.0));
                ui.label(legend(f.kind.name));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button("REMOVE").clicked() {
                        remove = Some(i);
                    }
                    if let Some(s) = live {
                        ui.label(legend(&format!("{} frames", s.frames)));
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

        ui.label(legend(t("settings.language")));
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
        hint(ui, t("settings.language.help"));
        ui.add_space(10.0);

        ui.label(legend(t("settings.country")));
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
        hint(ui, t("settings.country.help"));
        ui.add_space(10.0);

        ui.label(legend(t("settings.band_plan")));
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
        hint(ui, t("settings.band_plan.help"));
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
        ui.label(legend(t("settings.position")));
        let mut edit = self.station_edit.take();
        let set = Self::station_row(ui, self.location, &mut edit);
        self.station_edit = edit;
        if let Some((lat, lon)) = set {
            self.set_location(lat, lon);
            self.station_edit = None;
        }
        hint(ui, t("settings.position.help"));
        ui.add_space(10.0);

        ui.separator();
        ui.add_space(6.0);
        Self::data_settings(ui);
    }

    /// What is in the dataset cache, and the button that goes and asks.
    ///
    /// The airports, repeaters and ID registries are somebody else's files
    /// kept on this machine, so the questions an operator has about them are
    /// how old the copy is, how much disc it is using, and whether the last
    /// attempt to update it worked. Those are the three columns.
    fn data_settings(ui: &mut egui::Ui) {
        let t = crate::i18n::t;
        ui.label(legend(t("settings.data")));
        ui.add_space(4.0);

        let rows = crate::data::status();
        let busy = rows.iter().any(|r| r.busy);
        for r in &rows {
            let frame = egui::Frame::NONE
                .fill(theme::WELL)
                .stroke(Stroke::new(1.0, theme::ETCH))
                .inner_margin(egui::Margin::symmetric(8, 6))
                .corner_radius(2);
            frame.show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(value(r.which.label()).size(13.0));
                    ui.label(egui::RichText::new(r.which.publisher()).small().color(theme::LEGEND));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        // Disabled rather than hidden while it works: a
                        // button that vanishes under the pointer is a button
                        // that gets pressed twice.
                        let label = if r.busy { "CHECKING" } else { t("ui.refresh") };
                        if ui
                            .add_enabled(!r.busy, egui::Button::new(legend(label)))
                            .on_hover_text(r.which.about())
                            .clicked()
                        {
                            crate::data::refresh(r.which);
                        }
                    });
                });
                ui.horizontal(|ui| {
                    ui.label(legend("held"));
                    ui.label(match r.rows {
                        Some(n) => value(format!("{n} rows")).size(12.0),
                        // Cached but not parsed is the ordinary state for the
                        // registries, which are read the first time something
                        // asks them a question.
                        None if r.bytes > 0 => value("on disc").size(12.0),
                        None => value("not downloaded").size(12.0),
                    });
                    ui.add_space(10.0);
                    ui.label(legend("size"));
                    ui.label(value(crate::data::fmt_bytes(r.bytes)).size(12.0));
                    ui.add_space(10.0);
                    ui.label(legend("checked"));
                    ui.label(match r.checked_ago {
                        Some(s) => value(crate::data::fmt_ago(s)).size(12.0),
                        None => value("never").size(12.0),
                    });
                });
                if let Some(e) = &r.error {
                    ui.label(egui::RichText::new(e).small().color(theme::FAULT));
                }
            });
            ui.add_space(4.0);
        }

        ui.horizontal(|ui| {
            if ui.add_enabled(!busy, egui::Button::new(legend(t("ui.refresh_all")))).clicked() {
                for w in crate::data::Which::ALL {
                    crate::data::refresh(w);
                }
            }
            if let Some(dir) = crate::data::cache_dir() {
                ui.label(
                    egui::RichText::new(dir.display().to_string()).small().color(theme::LEGEND),
                );
            }
        });
        hint(ui, t("settings.data.help"));
        // A check runs on its own thread and finishes without an event, so
        // the pane has to come back and look, or a finished download stays
        // reading CHECKING until the pointer moves.
        if busy {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(400));
        }
    }

    /// Create a radio that is not on this machine.
    ///
    /// Nothing on the bus reveals a receiver on the network, so a remote radio
    /// is made rather than found: the address is the device. It is created
    /// where every other radio is chosen, because from the dial's point of
    /// view that is all it is.
    pub(super) fn remote_modal(&mut self, ctx: &egui::Context) {
        let Some(mut edit) = self.remote.take() else { return };
        let (mut close, mut add) = (false, false);
        let r = egui::containers::Modal::new(egui::Id::new("add-remote"))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(420.0);
                modal_title(ui, "Add remote radio");

                ui.label(legend("protocol"));
                egui::ComboBox::from_id_salt("remote-kind")
                    .selected_text(edit.kind.label())
                    .width(ui.available_width())
                    .show_ui(ui, |ui| {
                        for k in RemoteKind::ALL {
                            ui.selectable_value(&mut edit.kind, *k, k.label());
                        }
                    });
                hint(ui, edit.kind.help());
                ui.add_space(10.0);

                ui.label(legend("address"));
                let field = ui.add(
                    egui::TextEdit::singleline(&mut edit.host)
                        .desired_width(ui.available_width())
                        .hint_text(edit.kind.placeholder()),
                );
                ui.add_space(10.0);

                ui.label(legend("name"));
                let name = ui.add(
                    egui::TextEdit::singleline(&mut edit.label)
                        .desired_width(ui.available_width())
                        .hint_text("loft dongle"),
                );
                if name.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    add = true;
                }
                hint(
                    ui,
                    "What the radio list calls it. An address says which machine and nothing \
                     about which aerial.",
                );
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
                        egui::Label::new(egui::RichText::new(e).small().color(theme::FAULT))
                            .wrap(),
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
            return;
        }

        for (stage, mode) in &controls.stages {
            let auto = *mode == GainMode::Auto;
            let mut db = match mode {
                GainMode::Auto => *stage.range.start(),
                GainMode::Manual(v) => *v,
            };
            ui.horizontal(|ui| {
                ui.label(legend(&stage.label));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if stage.auto {
                        let mut on = auto;
                        if ui.checkbox(&mut on, "Auto").changed() {
                            self.send(Cmd::GainStage(
                                stage.name.clone(),
                                if on { GainMode::Auto } else { GainMode::Manual(db) },
                            ));
                        }
                    }
                    // Under AUTO the number is the hardware's business and
                    // showing a stale one invites the operator to believe it.
                    let text =
                        if auto { "auto".to_string() } else { format!("{db:.1} dB") };
                    ui.label(value(text).size(11.0));
                });
            });
            let lo = *stage.range.start();
            let hi = *stage.range.end();
            // Snapped as it is dragged, because the hardware does it anyway:
            // a slider that glides between values the tuner cannot reach shows
            // a number the receiver is not using.
            let slider = egui::Slider::new(&mut db, lo..=hi).show_value(false);
            if ui.add_enabled(!auto, slider).changed() {
                let want = stage.quantise(db);
                self.send(Cmd::GainStage(stage.name.clone(), GainMode::Manual(want)));
            }
            if !stage.values.is_empty() {
                hint(ui, &format!("{} steps, {lo:.0} to {hi:.0} dB", stage.values.len()));
            } else if stage.step > 0.0 {
                hint(ui, &format!("{:.0} dB steps, {lo:.0} to {hi:.0} dB", stage.step));
            }
            ui.add_space(10.0);
        }

        if !controls.choices.is_empty() {
            ui.separator();
            ui.add_space(6.0);
            for c in &controls.choices {
                ui.label(legend(&c.label));
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
                    self.send(Cmd::Choice(c.name.clone(), picked));
                }
                hint(ui, &c.help);
                ui.add_space(8.0);
            }
        }

        if !controls.toggles.is_empty() {
            ui.separator();
            ui.add_space(6.0);
            for t in &controls.toggles {
                let mut on = t.on;
                if ui.checkbox(&mut on, &t.label).changed() {
                    self.send(Cmd::Toggle(t.name.clone(), on));
                }
                hint(ui, &t.help);
                ui.add_space(8.0);
            }
        }

        ui.separator();
        ui.add_space(6.0);
        row(ui, "Correction", |ui| {
            let mut ppm = controls.ppm;
            if ui
                .add(egui::DragValue::new(&mut ppm).speed(0.5).range(-200.0..=200.0).suffix(" ppm"))
                .changed()
            {
                self.send(Cmd::Ppm(ppm));
            }
        });
        ui.label(
            egui::RichText::new(
                "The reference oscillator is a few tens of parts per million out on a cheap dongle, which is a kilohertz or two at 145 MHz and rather more higher up. Tune a known carrier and correct until it sits on its nominal frequency.",
            )
            .small()
            .color(theme::LEGEND),
        );
        ui.add_space(10.0);

        let mut dc = self.dc_block;
        if ui.checkbox(&mut dc, "Remove the DC spur").changed() {
            self.dc_block = dc;
            self.send(Cmd::DcBlock(dc));
        }
        ui.label(
            egui::RichText::new(
                "A direct conversion receiver leaks its own local oscillator into the middle of the span, where it looks exactly like a carrier on the frequency you are tuned to. This measures the offset and subtracts it.",
            )
            .small()
            .color(theme::LEGEND),
        );
    }

    fn spectrum_settings(&mut self, ui: &mut egui::Ui) {
        row(ui, "FFT bins", |ui| {
            let mut n = self.fft_size;
            egui::ComboBox::from_id_salt("fft")
                .selected_text(n.to_string())
                .width(120.0)
                .show_ui(ui, |ui| {
                    for v in FFTS {
                        ui.selectable_value(&mut n, v, v.to_string());
                    }
                });
            if n != self.fft_size {
                self.fft_size = n;
                // The same value the session saves and the radio starts with,
                // so a chosen FFT size survives a restart rather than only
                // living in the running spectrum.
                self.fft = n;
                self.send(Cmd::Fft(n));
                self.reset_waterfall();
            }
        });
        ui.label(
            egui::RichText::new(bin_hint(self.rate, self.fft_size))
                .small()
                .color(theme::LEGEND),
        );
        ui.add_space(8.0);

        row(ui, "Refresh", |ui| {
            let mut v = self.refresh;
            egui::ComboBox::from_id_salt("fps")
                .selected_text(format!("{} fps", v as i32))
                .width(120.0)
                .show_ui(ui, |ui| {
                    for (n, f) in REFRESH {
                        ui.selectable_value(&mut v, f, format!("{n} fps"));
                    }
                });
            if (v - self.refresh).abs() > 0.01 {
                self.refresh = v;
                self.send(Cmd::Refresh(v));
            }
        });
        ui.add_space(8.0);

        row(ui, "Averaging", |ui| {
            if ui
                .add(egui::Slider::new(&mut self.smoothing, 0.02..=1.0).show_value(false))
                .changed()
            {
                self.send(Cmd::Smoothing(self.smoothing));
            }
            ui.label(value(if self.smoothing > 0.95 {
                "off".to_string()
            } else {
                format!("{:.0}%", (1.0 - self.smoothing) * 100.0)
            }));
        });
        ui.add_space(8.0);

        row(ui, "Centre spur", |ui| {
            if ui.checkbox(&mut self.dc_block, "Remove").changed() {
                self.send(Cmd::DcBlock(self.dc_block));
            }
            ui.label(
                egui::RichText::new("LO leakage at the tuned frequency")
                    .color(theme::LEGEND)
                    .size(10.0),
            );
        });
        ui.add_space(8.0);
        self.scale_settings(ui);
    }

    fn waterfall_settings(&mut self, ui: &mut egui::Ui) {
        row(ui, "Scroll rate", |ui| {
            let mut v = self.rows_per_sec;
            egui::ComboBox::from_id_salt("rows")
                .selected_text(format!("{} rows/s", v as i32))
                .width(130.0)
                .show_ui(ui, |ui| {
                    for (n, f) in SPEEDS {
                        ui.selectable_value(&mut v, f, format!("{n} rows/s"));
                    }
                });
            self.rows_per_sec = v;
        });
        ui.add_space(8.0);

        row(ui, "History", |ui| {
            let mut n = self.wf_rows;
            egui::ComboBox::from_id_salt("hist")
                .selected_text(format!("{n} rows"))
                .width(130.0)
                .show_ui(ui, |ui| {
                    for v in [256usize, 512, 1024, 2048] {
                        ui.selectable_value(&mut n, v, format!("{v} rows"));
                    }
                });
            if n != self.wf_rows {
                self.wf_rows = n;
                self.wf.set_height(n);
            }
        });
        ui.label(
            egui::RichText::new(format!(
                "{:.0} s of history at {:.0} rows/s",
                self.wf.height() as f32 / self.rows_per_sec,
                self.rows_per_sec
            ))
            .small()
            .color(theme::LEGEND),
        );
        ui.add_space(8.0);

        row(ui, "Contrast", |ui| {
            ui.add(egui::Slider::new(&mut self.wf_top_offset, 0.0..=20.0).show_value(false));
            ui.label(value(format!("{:.0} dB", self.wf_top_offset)));
        });
        ui.label(
            egui::RichText::new("How far below the trace ceiling the hottest colour sits.")
                .small()
                .color(theme::LEGEND),
        );
        ui.add_space(8.0);
        self.scale_settings(ui);
    }

    fn scale_settings(&mut self, ui: &mut egui::Ui) {
        row(ui, "Scale", |ui| {
            ui.checkbox(&mut self.auto_scale, "Auto");
        });
        ui.add_enabled_ui(!self.auto_scale, |ui| {
            row(ui, "Floor", |ui| {
                ui.add(egui::Slider::new(&mut self.floor, -140.0..=0.0).suffix(" dB"));
            });
            row(ui, "Ceiling", |ui| {
                ui.add(egui::Slider::new(&mut self.ceil, -140.0..=20.0).suffix(" dB"));
            });
        });
    }
}
