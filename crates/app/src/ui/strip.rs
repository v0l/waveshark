//! The channel strip: every level that reaches the speaker, and the controls
//! that belong to one channel rather than to the receiver.

use super::state::AudioState;
use super::*;
use crate::audiobus::AudioBusNode;
use pipeline::param::ParamValue;

/// What the strip wants done that it cannot do itself.
pub(super) enum Action {
    /// The channel list changed, so the radio needs the whole of it.
    Channels,
}

/// The strip, over the levels it sets.
pub(super) struct Strip<'a> {
    pub st: &'a mut AudioState,
    pub radio: Option<&'a Radio>,
    pub acts: Vec<Action>,
    pub cmds: &'a mut Vec<Cmd>,
}

impl Strip<'_> {
    /// Gain and squelch, for the modes that have them.
    ///
    /// Worth a line of its own because on a weak signal these two are the
    /// difference between a band that is dead and a receiver that is muted,
    /// and without them both look and sound identical.
    fn channel_audio(ui: &mut egui::Ui, ch: &mut Channel, st: ChannelState) -> bool {
        let (gain_db, open, measured) = (st.agc_gain_db, st.squelch_open, st.squelch_db);
        let mut changed = false;
        ui.add_space(4.0);
        if ch.demod != Demod::Wfm {
            ui.horizontal(|ui| {
                ui.label(legend("agc"));
                if ui.selectable_label(ch.agc, if ch.agc { "ON" } else { "OFF" }).clicked() {
                    ch.agc = !ch.agc;
                    changed = true;
                }
                if ch.agc {
                    ui.label(value(format!("{gain_db:+.0} dB")).size(10.0));
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if !open {
                        ui.label(value("MUTED").size(10.0).color(theme::LEGEND));
                    }
                });
            });
        }
        if let Some(default) = ch.demod.default_squelch_db() {
            let (lo, hi, ratio) = ch.demod.squelch_range();
            let mut db = ch.squelch_db.unwrap_or(default);
            ui.horizontal(|ui| {
                ui.label(legend("sql"));
                if ui.add(Squelch::new(&mut db, lo, hi, measured, open)).changed() {
                    ch.squelch_db = Some(db);
                    changed = true;
                }
                // At the bottom of its range the squelch passes everything,
                // and saying so is more use than printing the number that
                // happens to be there.
                let text = if db <= lo + 0.5 {
                    "off".to_string()
                } else {
                    format!("{db:.0}{}", if ratio { "" } else { " dBFS" })
                };
                ui.label(value(text).size(10.0));
            });
            // The reading the threshold is being set against. Without it the
            // control is a number to guess at, and the right number differs
            // by mode and moves with the RF gain.
            ui.horizontal(|ui| {
                ui.add_space(28.0);
                hint(ui, &format!("now {measured:.0} dB"));
            });
        }
        changed
    }

    /// What the radio is hearing on the channel being listened to.
    ///
    /// This belongs inside the channel rather than beside the list: a station
    /// name is a property of one tuned frequency, and with several channels
    /// configured a panel-level readout gives no clue which one it describes.
    fn channel_rds(ui: &mut egui::Ui, st: &StationInfo, blend: f32) {
        if st.is_empty() && blend <= 0.01 {
            return;
        }
        ui.add_space(6.0);
        ui.separator();
        ui.horizontal(|ui| {
            ui.label(legend("rds"));
            if let Some(pi) = st.pi {
                ui.label(legend(&format!("PI {pi:04X}")));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // Stereo belongs here too: it is a property of this station,
                // and it fades with the blend because the audio does.
                let t = blend.clamp(0.0, 1.0);
                if t > 0.01 {
                    let c = theme::TRACE.gamma_multiply(0.35 + 0.65 * t);
                    ui.label(value(if t > 0.99 { "STEREO" } else { "BLEND" }).size(10.0).color(c));
                }
            });
        });
        if let Some(n) = &st.name {
            // Cyan, not amber: this is what the radio heard, not something the
            // operator set.
            ui.label(value(n).size(15.0).color(theme::TRACE));
        }
        if let Some(p) = st.pty {
            ui.label(legend(p));
        }
        if let Some(rt) = &st.radiotext {
            ui.add_space(2.0);
            // Radiotext is up to 64 characters and the strip is narrow, so let
            // it wrap rather than truncating a song title mid-word.
            ui.label(egui::RichText::new(rt).color(theme::LEGEND).size(11.0));
        }
    }

    /// Draw the strip, and collect what it wants done.
    pub(super) fn show(mut self, ui: &mut egui::Ui) -> Vec<Action> {
        Panel::right("channels")
            .default_size(285.0)
            .frame(
                egui::Frame::NONE
                    .fill(theme::PANEL)
                    .inner_margin(egui::Margin::symmetric(12, 10)),
            )
            .show(ui, |ui| {
                ui.label(legend("channels"));
                ui.add_space(6.0);

                // The master, which every channel's own level runs into.
                let out_level = self.radio.map(|r| r.status.out_level()).unwrap_or(0.0);
                let call_level = self.radio.map(|r| r.status.call_level()).unwrap_or(0.0);
                ui.horizontal(|ui| {
                    ui.label(legend("master"));
                    if ui.add(Fader::new(&mut self.st.volume, out_level).width(VU_W)).changed() {
                        self.cmds.push(Cmd::Volume(self.st.volume));
                    }
                    let all_muted = !self.st.channels.is_empty()
                        && self.st.channels.iter().all(|c| c.muted || !c.on);
                    if crate::icons::icon_button(
                        ui,
                        if all_muted { crate::icons::Icon::Mute } else { crate::icons::Icon::Sound },
                        "Mute every channel",
                        true,
                        all_muted,
                    )
                    .clicked()
                    {
                        for c in &mut self.st.channels {
                            c.muted = !all_muted;
                        }
                        self.acts.push(Action::Channels);
                    }
                });

                // Call audio has one level for the lot, beside the master:
                // it is not a channel anybody tuned, it is whatever the front
                // ends decode, and mixing it belongs where every other level
                // in the receiver is set.
                ui.horizontal(|ui| {
                    ui.label(legend("calls"));
                    let mut changed = ui
                        .add(Fader::new(&mut self.st.call_volume, call_level).width(VU_W))
                        .changed();
                    if crate::icons::icon_button(
                        ui,
                        if self.st.call_muted {
                            crate::icons::Icon::Mute
                        } else {
                            crate::icons::Icon::Sound
                        },
                        "Mute call audio",
                        true,
                        self.st.call_muted,
                    )
                    .clicked()
                    {
                        self.st.call_muted = !self.st.call_muted;
                        changed = true;
                    }
                    if changed {
                        self.cmds.push(Cmd::CallVolume {
                            volume: self.st.call_volume,
                            muted: self.st.call_muted,
                        });
                    }
                });
                // The gain control, with what it is doing beside it: a call
                // arrives at whatever level the transmitting radio's
                // microphone was set to, which is not something a listener
                // can fix at the far end.
                ui.horizontal(|ui| {
                    ui.add_space(4.0);
                    if ui.checkbox(&mut self.st.call_agc, "AGC").changed() {
                        self.cmds.push(Cmd::CallAgc(self.st.call_agc));
                    }
                    let db = self
                        .radio
                        .map(|r| r.status.call_gain_db())
                        .unwrap_or(0.0);
                    if self.st.call_agc && db.abs() > 0.1 {
                        ui.label(value(format!("{db:+.0} dB")).size(11.0));
                    }
                });

                ui.add_space(8.0);

                if self.st.channels.is_empty() {
                    ui.label(
                        egui::RichText::new("Click the spectrum to tune a channel.")
                            .color(theme::LEGEND)
                            .size(12.0),
                    );
                }

                let states: Vec<ChannelState> =
                    self.radio.map(|r| r.status.channel_states()).unwrap_or_default();
                let mut remove = None;
                let mut tune = None;
                for (i, ch) in self.st.channels.iter_mut().enumerate() {
                    let active = self.st.listening == Some(i);
                    // Both strips take the panel fill. The selected one used a
                    // lighter wash, which was the exact colour of a slider's
                    // handle and trough, so the volume control disappeared
                    // into the strip it sat on. Selection is carried by the
                    // amber edge and the lit bar instead, which is how it is
                    // marked on a mixing desk: a lamp, not a change of paint.
                    egui::Frame::NONE
                        .fill(theme::PANEL)
                        .stroke(Stroke::new(
                            if active { 1.5 } else { 1.0 },
                            if active { theme::READOUT } else { theme::ETCH },
                        ))
                        .corner_radius(2.0)
                        .inner_margin(egui::Margin::same(8))
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                // A lit bar marks the channel you are hearing.
                                let (r, _) = ui.allocate_exact_size(Vec2::new(3.0, 16.0), Sense::hover());
                                ui.painter().rect_filled(
                                    r,
                                    1.0,
                                    if active { theme::READOUT } else { theme::ETCH },
                                );
                                ui.add(
                                    egui::TextEdit::singleline(&mut ch.label)
                                        .desired_width(90.0)
                                        .frame(egui::Frame::NONE),
                                );
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        if ui.small_button("REMOVE").clicked() {
                                            remove = Some(i);
                                        }
                                    },
                                );
                            });
                            // Per-digit, like the main tuner: the wheel over a
                            // digit steps that decade, so tuning is repeatable
                            // rather than depending on pointer speed.
                            let d = self.st.dial.compact(ui, ch.freq, 23.0);
                            if d.changed {
                                ch.freq = d.hz;
                                tune = Some(i);
                            }
                            ui.label(legend(bands::name_at(ch.freq)));
                            ui.add_space(4.0);
                            // Two rows: broadcast modes, then the ones an
                            // amateur band needs. Six across is narrower than
                            // the panel gets on a laptop.
                            ui.horizontal(|ui| {
                                for m in [Demod::Wfm, Demod::Nfm, Demod::Am] {
                                    if ui.selectable_label(ch.demod == m, m.label()).clicked() {
                                        ch.demod = m;
                                        tune = Some(i);
                                    }
                                }
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        // Every channel can be on at once and
                                        // they mix, so this is a per channel
                                        // switch rather than a choice of one.
                                        let text = if ch.on { "ON" } else { "OFF" };
                                        if ui.selectable_label(ch.on, text).clicked() {
                                            ch.on = !ch.on;
                                            tune = Some(i);
                                        }
                                    },
                                );
                            });
                            ui.horizontal(|ui| {
                                for m in [Demod::Usb, Demod::Lsb, Demod::Cw] {
                                    if ui.selectable_label(ch.demod == m, m.label()).clicked() {
                                        ch.demod = m;
                                        tune = Some(i);
                                    }
                                }
                            });
                            if ch.on {
                                // Its own level, which runs into the master,
                                // read against what it is contributing.
                                let st = states.iter().find(|s| s.id == ch.id).copied();
                                ui.add_space(4.0);
                                ui.horizontal(|ui| {
                                    ui.label(legend("vol"));
                                    let level = st.map(|s| s.level).unwrap_or(0.0);
                                    if ui.add(Fader::new(&mut ch.volume, level).width(VU_W)).changed() {
                                        tune = Some(i);
                                    }
                                    if ui.selectable_label(ch.muted, "M").clicked() {
                                        ch.muted = !ch.muted;
                                        tune = Some(i);
                                    }
                                });
                                if ch.demod == Demod::Wfm {
                                    // Each channel's own RDS, not the first
                                    // channel's: two WFM channels are usually
                                    // two different stations.
                                    let station =
                                        self.radio.and_then(|r| r.status.station_for(ch.id));
                                    if let Some(station) = station {
                                        let blend = st.map(|s| s.stereo_blend).unwrap_or(0.0);
                                        Self::channel_rds(ui, &station, blend);
                                    }
                                }
                                if let Some(st) = st {
                                    if Self::channel_audio(ui, ch, st) {
                                        tune = Some(i);
                                    }
                                }
                            }
                        });
                    ui.add_space(6.0);
                }

                // Chains the operator drew and wired into the bus are strips
                // too: nobody tuned them, so there is no dial or mode to
                // show, but each has a level and a meter like everything
                // else that reaches the speaker. Set by the same route the
                // chain view uses, since the level is the bus's parameter.
                let (bus, strips) =
                    self.radio.map(|r| r.status.strips()).unwrap_or((None, Vec::new()));
                if let Some(bus) = bus {
                    for s in strips.iter().filter(|s| s.channel.is_none() && !s.voice) {
                        egui::Frame::NONE
                            .fill(theme::PANEL)
                            .stroke(Stroke::new(1.0, theme::ETCH))
                            .corner_radius(2.0)
                            .inner_margin(egui::Margin::same(8))
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    let (r, _) = ui
                                        .allocate_exact_size(Vec2::new(3.0, 16.0), Sense::hover());
                                    ui.painter().rect_filled(r, 1.0, theme::ETCH);
                                    ui.label(value(s.label.clone()).size(12.0));
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            ui.label(legend("chain"));
                                        },
                                    );
                                });
                                ui.add_space(4.0);
                                ui.horizontal(|ui| {
                                    ui.label(legend("vol"));
                                    let mut v = s.volume;
                                    if ui.add(Fader::new(&mut v, s.level).width(VU_W)).changed() {
                                        self.cmds.push(Cmd::NodeParam(
                                            bus,
                                            AudioBusNode::param_of(s.port, "vol"),
                                            ParamValue::Float(v as f64),
                                        ));
                                    }
                                    if ui.selectable_label(s.muted, "M").clicked() {
                                        self.cmds.push(Cmd::NodeParam(
                                            bus,
                                            AudioBusNode::param_of(s.port, "mute"),
                                            ParamValue::Bool(!s.muted),
                                        ));
                                    }
                                });
                            });
                        ui.add_space(6.0);
                    }
                }

                if let Some(i) = remove {
                    self.st.channels.remove(i);
                    match self.st.listening {
                        Some(l) if l == i => self.st.listening = None,
                        Some(l) if l > i => self.st.listening = Some(l - 1),
                        _ => {}
                    }
                    self.acts.push(Action::Channels);
                }
                if tune.is_some() {
                    if let Some(i) = tune {
                        self.st.listening = Some(i);
                    }
                    self.acts.push(Action::Channels);
                }

            });
        self.acts
    }
}
