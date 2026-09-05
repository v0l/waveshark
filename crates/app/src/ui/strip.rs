//! The channel strip: every level that reaches the speaker, and the controls
//! that belong to one channel rather than to the receiver.

use crate::radio::TxSource;
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
        // A decode channel has neither: its front end sets its own levels and
        // decides for itself whether a burst is a transmission.
        let Some(demod) = ch.mode.demod() else { return false };
        ui.add_space(4.0);
        if demod != Demod::Wfm {
            ui.horizontal(|ui| {
                theme::Line::new().legend("agc").show(ui);
                if ui.selectable_label(ch.agc, if ch.agc { "ON" } else { "OFF" }).clicked() {
                    ch.agc = !ch.agc;
                    changed = true;
                }
                if ch.agc {
                    theme::Line::new().value(format!("{gain_db:+.0} dB")).size(11.0).show(ui);
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if !open {
                        theme::Line::new().legend("muted").show(ui);
                    }
                });
            });
        }
        if let Some(default) = demod.default_squelch_db() {
            let (lo, hi, ratio) = demod.squelch_range();
            let mut db = ch.squelch_db.unwrap_or(default);
            ui.horizontal(|ui| {
                theme::Line::new().legend("sql").show(ui);
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
                theme::Line::new().value(text).size(11.0).show(ui);
            });
            // The reading the threshold is being set against. Without it the
            // control is a number to guess at, and the right number differs
            // by mode and moves with the RF gain.
            ui.horizontal(|ui| {
                ui.add_space(28.0);
                theme::Line::new().note(format!("now {measured:.0} dB")).show(ui);
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
            let mut head = theme::Line::new().legend("rds");
            if let Some(pi) = st.pi {
                head = head.legend(&format!("PI {pi:04X}"));
            }
            head.show(ui);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // Stereo belongs here too: it is a property of this station,
                // and it fades with the blend because the audio does.
                let t = blend.clamp(0.0, 1.0);
                if t > 0.01 {
                    let c = theme::TRACE.gamma_multiply(0.35 + 0.65 * t);
                    theme::Line::new()
                        .value(if t > 0.99 { "stereo" } else { "blend" })
                        .tint(c)
                        .size(11.0)
                        .show(ui);
                }
            });
        });
        if let Some(n) = &st.name {
            // Cyan, not amber: this is what the radio heard, not something the
            // operator set.
            theme::Line::new().heard(n).size(15.0).show(ui);
        }
        if let Some(p) = st.pty {
            theme::Line::new().legend(p).show(ui);
        }
        if let Some(rt) = &st.radiotext {
            ui.add_space(2.0);
            // Radiotext is up to 64 characters and the strip is narrow, so let
            // it wrap rather than truncating a song title mid-word.
            theme::Line::new().note(rt).wrapped(ui);
        }
    }

    /// The transmit half of a channel.
    ///
    /// Drawn on every channel of a radio that can transmit, and on none of a
    /// radio that cannot: a key that always fails is worse than no key, since
    /// the operator learns to press it.
    ///
    /// There is no mode here. A channel is one frequency and one mode, and it
    /// transmits in the mode it receives: a radio that listens in NFM and
    /// keys up in AM cannot be worked by whoever is on the other end.
    fn channel_tx(
        ui: &mut egui::Ui,
        ch: &mut Channel,
        keyed: Option<u64>,
        mic: (f32, f32),
        cmds: &mut Vec<Cmd>,
    ) -> bool {
        let mut changed = false;
        let mode = crate::radio::tx_mode_for(&ch.mode);
        let tx = ch.tx.get_or_insert_with(crate::radio::TxSpec::default);

        ui.add_space(6.0);
        ui.separator();
        ui.horizontal(|ui| {
            theme::Line::new().legend("tx").show(ui);
            match mode {
                Some(m) => {
                    theme::Line::new().value(m.label()).size(12.0).show(ui);
                }
                // Said rather than hidden: a mode with no modulator behind it
                // is a gap in this receiver, not a property of the channel.
                None => {
                    theme::Line::new().note("no transmitter for this mode").show(ui);
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                theme::Line::new()
                    .value(format!("{:.4} MHz", (ch.freq + tx.shift_hz) / 1e6))
                    .size(11.0)
                    .show(ui);
            });
        });

        ui.horizontal(|ui| {
            theme::Line::new().legend("src").show(ui);
            for src in [TxSource::Mic, TxSource::Tone] {
                if ui.selectable_label(tx.source == src, src.label()).clicked() {
                    tx.source = src;
                    changed = true;
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                theme::Line::new().legend("shift").show(ui);
                let mut khz = tx.shift_hz / 1e3;
                if ui
                    .add(
                        egui::DragValue::new(&mut khz)
                            .speed(0.1)
                            .range(-10_000.0..=10_000.0)
                            .suffix(" kHz"),
                    )
                    .changed()
                {
                    tx.shift_hz = khz * 1e3;
                    changed = true;
                }
            });
        });

        match tx.source {
            TxSource::Mic => {
                // The microphone's own fader and meter, read the way the
                // channel's audio is: the level beside the control that sets
                // it, so an operator can see they are being heard.
                ui.horizontal(|ui| {
                    theme::Line::new().legend("mic").show(ui);
                    let mut g = tx.mic_gain / 4.0;
                    if ui.add(Fader::new(&mut g, mic.0).width(VU_W)).changed() {
                        tx.mic_gain = (g * 4.0).clamp(0.0, 4.0);
                        changed = true;
                    }
                    theme::Line::new().value(format!("{:.1}x", tx.mic_gain)).size(11.0).show(ui);
                });
                ui.horizontal(|ui| {
                    theme::Line::new().legend("agc").show(ui);
                    let on = tx.mic_agc;
                    if ui.selectable_label(on, if on { "ON" } else { "OFF" }).clicked() {
                        tx.mic_agc = !on;
                        changed = true;
                    }
                    if on {
                        theme::Line::new().value(format!("{:+.0} dB", mic.1)).size(11.0).show(ui);
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        theme::Line::new().note("mic open").show(ui);
                    });
                });
            }
            TxSource::Tone => {
                ui.horizontal(|ui| {
                    theme::Line::new().legend("tone").show(ui);
                    let mut hz = tx.tone_hz;
                    if ui
                        .add(
                            egui::DragValue::new(&mut hz)
                                .speed(10.0)
                                .range(100.0..=5_000.0)
                                .suffix(" Hz"),
                        )
                        .changed()
                    {
                        tx.tone_hz = hz;
                        changed = true;
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        theme::Line::new().legend("trim").show(ui);
                        let mut db = tx.trim_db;
                        if ui
                            .add(
                                egui::DragValue::new(&mut db)
                                    .speed(0.5)
                                    .range(0.0..=20.0)
                                    .suffix(" dB"),
                            )
                            .changed()
                        {
                            tx.trim_db = db;
                            changed = true;
                        }
                    });
                });
            }
        }

        ui.add_space(4.0);
        let keyed_here = keyed == Some(ch.id);
        let can_key = mode.is_some();
        let key = ui.add_enabled(
            can_key,
            egui::Button::new(
                egui::RichText::new(if keyed_here { "ON AIR" } else { "KEY" })
                    .size(15.0)
                    .color(if keyed_here { theme::PANEL } else { theme::READOUT }),
            )
            .fill(if keyed_here { theme::READOUT } else { theme::PANEL })
            .min_size(Vec2::new(VU_W, 26.0))
            // Dragging, not clicking. A button that only senses clicks
            // reports the press and then stops tracking the pointer, so a key
            // held down came back up on its own after a frame or two: the
            // carrier lasted as long as it took the queue to drain.
            .sense(Sense::click_and_drag()),
        );
        // Held, not toggled: released, lost focus and the pointer leaving all
        // drop the carrier.
        if can_key && key.is_pointer_button_down_on() && !keyed_here {
            cmds.push(Cmd::Key(Some(ch.id)));
        } else if keyed_here && !key.is_pointer_button_down_on() {
            cmds.push(Cmd::Key(None));
        }
        changed
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
                // Call audio has a level of its own and it is set on the
                // calls page: it is not a channel anybody tuned, and a fader
                // for it under the master read as a second master.
                let out_level = self.radio.map(|r| r.status.out_level()).unwrap_or(0.0);
                ui.horizontal(|ui| {
                    theme::Line::new().legend("master").show(ui);
                    if ui.add(Fader::new(&mut self.st.volume, out_level).width(VU_W)).changed() {
                        self.cmds
                            .push(Cmd::Volume { volume: self.st.volume, muted: self.st.muted });
                    }
                    // The master mute is the bus's own, not a sweep over the
                    // channel mutes: muting every channel left calls, replays
                    // and any chain drawn by hand still coming out of the
                    // speaker, which is not what a master mute means.
                    if crate::icons::icon_button(
                        ui,
                        if self.st.muted {
                            crate::icons::Icon::Mute
                        } else {
                            crate::icons::Icon::Sound
                        },
                        "Mute everything",
                        true,
                        self.st.muted,
                    )
                    .clicked()
                    {
                        self.st.muted = !self.st.muted;
                        self.cmds
                            .push(Cmd::Volume { volume: self.st.volume, muted: self.st.muted });
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
                // What the radio can do, asked of the radio: an RTL-SDR gets
                // no transmit controls at all rather than controls that fail.
                let can_tx = self
                    .radio
                    .map(|r| r.status.can_transmit.load(std::sync::atomic::Ordering::Relaxed))
                    .unwrap_or(false);
                // What the microphone is hearing, and what the levelling is
                // adding, for the meter beside the key.
                let mic = self
                    .radio
                    .map(|r| {
                        (
                            f32::from_bits(
                                r.status.mic_level.load(std::sync::atomic::Ordering::Relaxed),
                            ),
                            f32::from_bits(
                                r.status.mic_gain_db.load(std::sync::atomic::Ordering::Relaxed),
                            ),
                        )
                    })
                    .unwrap_or((0.0, 0.0));
                let keyed = self
                    .radio
                    .map(|r| r.status.keyed.load(std::sync::atomic::Ordering::Relaxed))
                    .filter(|id| *id != 0);
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
                            theme::Line::new().legend(bands::name_at(ch.freq)).show(ui);
                            ui.add_space(4.0);
                            // Two rows: broadcast modes, then the ones an
                            // amateur band needs. Six across is narrower than
                            // the panel gets on a laptop.
                            ui.horizontal(|ui| {
                                for m in [Demod::Wfm, Demod::Nfm, Demod::Am] {
                                    let on = ch.mode == ChanMode::Audio(m);
                                    if ui.selectable_label(on, m.label()).clicked() {
                                        ch.mode = ChanMode::Audio(m);
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
                                    let on = ch.mode == ChanMode::Audio(m);
                                    if ui.selectable_label(on, m.label()).clicked() {
                                        ch.mode = ChanMode::Audio(m);
                                        tune = Some(i);
                                    }
                                }
                            });
                            // The front ends that read one channel, asked of
                            // the registry rather than listed here. Picking
                            // one runs that decoder on this frequency alone,
                            // which is what makes a single channel readable
                            // with the scanner switched off.
                            ui.horizontal_wrapped(|ui| {
                                for (kind, width) in crate::chain::channel_fronts() {
                                    let on = ch.mode == ChanMode::Decode(kind.to_string());
                                    let r = ui.selectable_label(on, crate::chain::front_label(kind));
                                    if r.clicked() {
                                        ch.mode = ChanMode::Decode(kind.to_string());
                                        tune = Some(i);
                                    }
                                    r.on_hover_text(format!(
                                        "decode this frequency as {} in a {:.1} kHz channel",
                                        crate::chain::front_label(kind),
                                        width / 1e3,
                                    ));
                                }
                            });
                            // Above everything that comes and goes. The RDS
                            // readout appears the moment a station is
                            // identified and disappears when it is lost, and
                            // a key drawn under it moves out from under the
                            // pointer mid-transmission: on WFM the carrier
                            // dropped every time a station name arrived.
                            if can_tx && Self::channel_tx(ui, ch, keyed, mic, self.cmds) {
                                tune = Some(i);
                            }
                            if ch.on {
                                // Its own level, which runs into the master,
                                // read against what it is contributing.
                                let st = states.iter().find(|s| s.id == ch.id).copied();
                                ui.add_space(4.0);
                                ui.horizontal(|ui| {
                                    theme::Line::new().legend("vol").show(ui);
                                    let level = st.map(|s| s.level).unwrap_or(0.0);
                                    if ui.add(Fader::new(&mut ch.volume, level).width(VU_W)).changed() {
                                        tune = Some(i);
                                    }
                                    if ui.selectable_label(ch.muted, "M").clicked() {
                                        ch.muted = !ch.muted;
                                        tune = Some(i);
                                    }
                                });
                                if ch.mode == ChanMode::Audio(Demod::Wfm) {
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
                                    theme::Line::new().value(s.label.clone()).size(12.0).show(ui);
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            theme::Line::new().legend("chain").show(ui);
                                        },
                                    );
                                });
                                ui.add_space(4.0);
                                ui.horizontal(|ui| {
                                    theme::Line::new().legend("vol").show(ui);
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
