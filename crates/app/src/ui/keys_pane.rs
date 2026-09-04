//! The key manager: the enciphered channels heard, and the keys held.
//!
//! A row per channel a front end has identified, showing what it is, the
//! encryption in force, and where any key search has got to. This much is a
//! useful encryption monitor on its own, and compiles in every build.
//!
//! With the `tea` feature, the row also shows the key in force and its
//! provenance, offers a box to enter one by hand, persists it, and shows a
//! recovered key. Without the feature there is no cipher or key store to
//! back that, so those controls are absent and the view is read-only.
//!
//! TETRA is the one enciphered mode read today, so the identity a row shows
//! is a cell; the view is written to hold any enciphered channel a front end
//! reports, so a mode added later joins it without this file changing.

use super::state::KeysState;
use super::*;
use nodes::tetra_nodes::{KeyStatus, Recovery};

/// The key manager, over the live cell status and the stored keys.
pub(super) struct Keys<'a> {
    pub st: &'a mut KeysState,
    pub radio: Option<&'a Radio>,
    #[cfg_attr(not(feature = "tea"), allow(dead_code))]
    pub cmds: &'a mut Vec<Cmd>,
}

impl Keys<'_> {
    pub(super) fn show(mut self, ui: &mut egui::Ui) {
        let live: Vec<KeyStatus> = self.radio.map(|r| r.status.tetra_keys()).unwrap_or_default();

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            let enc = live.iter().filter(|s| s.aie != 0).count();
            let n = live.len();
            let chan = if n == 1 { "channel" } else { "channels" };
            theme::Line::new()
                .legend("encryption keys")
                .value(format!("{n} {chan}, {enc} enciphered"))
                .size(11.0)
                .show(ui);
        });
        ui.add_space(6.0);

        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 6.0;
            egui::Frame::NONE
                .inner_margin(egui::Margin::symmetric(12, 0))
                .show(ui, |ui| {
                    self.channel_keys(ui);
                    ui.add_space(10.0);
                    if live.is_empty() {
                        ui.add_space(12.0);
                        ui.vertical_centered(|ui| {
                            hint(
                                ui,
                                "No enciphered cell heard yet. Tune one and it appears here with its \
                                 encryption mode and, where the build can, its key.",
                            );
                        });
                    }
                    for s in live.iter() {
                        self.card(ui, s);
                    }
                });
            ui.add_space(8.0);
        });
    }

    /// The channel keys of the mesh protocols: the ones held, and a row to
    /// add one. A Meshtastic channel is its name and its PSK, the name
    /// being part of what the packets hash; a MeshCore channel is a PSK and
    /// the name is only what to call it here. A DMR entry is a talkgroup and
    /// its privacy key, held for the decoder that will read it.
    fn channel_keys(&mut self, ui: &mut egui::Ui) {
        use decode::channel_keys::{ChannelKey, System};
        let held: Vec<ChannelKey> = self.st.store.channels().to_vec();
        let mut forget: Option<(System, String)> = None;
        // Amber rail: these are the operator's own settings, like a key
        // typed for a cell.
        widgets::card(
            ui,
            (!held.is_empty()).then_some(theme::READOUT),
            |ui| {
                theme::Line::new()
                    .legend("channel keys")
                    .value(format!("{} held", held.len()))
                    .show(ui);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    theme::Line::new()
                        .value("default and public channels are always read")
                        .tint(theme::LEGEND)
                        .size(11.0)
                        .show(ui);
                });
            },
            |ui| {
                for c in &held {
                    ui.horizontal(|ui| {
                        theme::Line::new()
                            .legend(c.system.as_str())
                            .value(&c.name)
                            .size(11.0)
                            .show(ui);
                        theme::Line::new()
                            .value(decode::channel_keys::hex(&c.key))
                            .tint(theme::LEGEND)
                            .size(11.0)
                            .show(ui);
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("forget").clicked() {
                                forget = Some((c.system, c.name.clone()));
                            }
                        });
                    });
                }
                if !held.is_empty() {
                    ui.add_space(4.0);
                }
                ui.horizontal(|ui| {
                    egui::ComboBox::from_id_salt("new_channel_system")
                        .selected_text(self.st.new_system.as_str())
                        .width(100.0)
                        .show_ui(ui, |ui| {
                            for s in [System::Meshtastic, System::MeshCore, System::Dmr] {
                                ui.selectable_value(&mut self.st.new_system, s, s.as_str());
                            }
                        });
                    ui.add(
                        egui::TextEdit::singleline(&mut self.st.new_name)
                            .hint_text(match self.st.new_system {
                                System::Dmr => "talkgroup",
                                _ => "channel name",
                            })
                            .desired_width(120.0),
                    );
                    ui.add(
                        egui::TextEdit::singleline(&mut self.st.new_key)
                            .hint_text("key: hex or base64")
                            .desired_width(240.0)
                            .font(egui::FontId::monospace(12.0)),
                    );
                    let key = decode::channel_keys::parse_key(&self.st.new_key);
                    let ok = key.is_some() && !self.st.new_name.trim().is_empty();
                    if ui.add_enabled(ok, egui::Button::new("Add")).clicked() {
                        if let Some(key) = key {
                            self.st.store.insert_channel(ChannelKey {
                                system: self.st.new_system,
                                name: self.st.new_name.trim().to_string(),
                                key,
                            });
                            let _ = self.st.store.save();
                            self.st.store.publish();
                            self.st.new_name.clear();
                            self.st.new_key.clear();
                        }
                    }
                });
            },
        );
        if let Some((system, name)) = forget {
            self.st.store.remove_channel(system, &name);
            let _ = self.st.store.save();
            self.st.store.publish();
        }
    }

    /// One channel: what it is in the header, what is known about its keying
    /// underneath.
    ///
    /// This was a single row that ran off the right of the pane, with the
    /// frequency, the cell, the cipher, the search, two text boxes and three
    /// buttons on one line. Which of them belonged to which channel was
    /// anybody's guess once a second cell appeared.
    fn card(&mut self, ui: &mut egui::Ui, s: &KeyStatus) {
        // Amber where a key is in force, because a channel that is being
        // read is the one worth spotting from across the pane.
        let rail = match (s.aie == 0, s.has_key) {
            (true, _) => None,
            (false, true) => Some(theme::READOUT),
            (false, false) => Some(theme::FAULT),
        };
        widgets::card(
            ui,
            rail,
            |ui| {
                theme::Line::new()
                    .value(format!("{:.4} MHz", s.channel_hz / 1e6))
                    .legend(&format!("{}/{}/cc{}", s.mcc, s.mnc, s.colour))
                    .show(ui);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let cipher = if s.aie == 0 { "clear".into() } else { format!("AIE-{}", s.aie) };
                    theme::Line::new()
                        .value(cipher)
                        .tint(if s.aie == 0 { theme::LEGEND } else { theme::READOUT })
                        .size(11.0)
                        .show(ui);
                });
            },
            |ui| {
                let mut line = theme::Line::new()
                    .legend("key")
                    .value(if s.has_key { "in force" } else { "none held" })
                    .tint(if s.has_key { theme::VALUE } else { theme::LEGEND })
                    .size(11.0);
                #[cfg(feature = "tea")]
                {
                    line = line.value(self.provenance(s)).tint(theme::LEGEND).size(11.0);
                }
                if s.reuse_pairs > 0 {
                    line = line.legend(&format!("{} reuse", s.reuse_pairs));
                }
                line.show(ui);

                // The key search, shown as it happens rather than only when
                // it lands. Always drawn: without `tea` the phase is Idle, so
                // nothing shows, which is correct.
                if !s.has_key {
                    if let Some((text, colour)) = recovery_label(s.recovery) {
                        theme::Line::new()
                            .legend("search")
                            .value(text)
                            .tint(colour)
                            .size(11.0)
                            .wrapped(ui);
                    }
                }

                // The key material: entry, provenance, persistence. Only with
                // the cipher and store the `tea` feature brings.
                #[cfg(feature = "tea")]
                self.keying(ui, s);
            },
        );
    }

    /// The key-material controls for one row: provenance from the store, a
    /// box to enter a key by hand, persistence of a recovered one, and a
    /// forget button. Present only with the `tea` feature.
    #[cfg(feature = "tea")]
    fn keying(&mut self, ui: &mut egui::Ui, s: &KeyStatus) {
        use crate::keystore::{CellId, Origin};
        let cell = CellId { mcc: s.mcc, mnc: s.mnc, colour: s.colour };
        let stored = self.st.store.get(cell).map(|e| e.origin);

        // Manual entry for an enciphered cell with no key.
        if s.aie != 0 && !s.has_key {
            ui.horizontal(|ui| {
                theme::Line::new().legend("key").show(ui);
                let buf = self.st.typing.entry(cell.tag_key()).or_default();
                ui.add(
                    egui::TextEdit::singleline(buf)
                        .hint_text("hex: 8 digits for TEA1, 20 for TEA2")
                        .desired_width(240.0)
                        .font(egui::FontId::monospace(12.0)),
                );
                let parsed = crate::keystore::parse_typed_key(buf);
                if ui.add_enabled(parsed.is_some(), egui::Button::new("Set key")).clicked() {
                    if let Some(key) = parsed {
                        self.cmds.push(Cmd::TetraKey { colour: cell.colour, key });
                        self.st.store.insert(cell, key, Origin::Manual);
                        let _ = self.st.store.save();
                        self.st.typing.remove(&cell.tag_key());
                    }
                }
            });
        }

        // Identity secret entry: a 16-hex-digit TA61 `c` that de-anonymises
        // the encrypted identities on this cell, independent of the voice
        // key and working on TEA2/3. Session-only; not persisted yet.
        if s.aie != 0 {
            ui.horizontal(|ui| {
                theme::Line::new().legend("identity").show(ui);
                let key = format!("{}#id", cell.tag_key());
                let buf = self.st.typing.entry(key.clone()).or_default();
                ui.add(
                    egui::TextEdit::singleline(buf)
                        .hint_text("secret: 16 hex digits")
                        .desired_width(240.0)
                        .font(egui::FontId::monospace(12.0)),
                );
                let secret = parse_id_secret(buf);
                if ui.add_enabled(secret.is_some(), egui::Button::new("Set secret")).clicked() {
                    if let Some(c) = secret {
                        self.cmds.push(Cmd::TetraIdSecret { colour: cell.colour, c });
                        self.st.typing.remove(&key);
                    }
                }
            });
        }

        if stored.is_some() {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("Forget this key").clicked() {
                    self.st.store.remove(cell);
                    let _ = self.st.store.save();
                }
            });
        }
    }

    /// Where the key in force came from, for the line that says there is one.
    ///
    /// A recovered key is one broken this session: persisting it would need
    /// the key value threaded up through the status snapshot, which it is
    /// not yet, so it is in force until the receiver stops.
    #[cfg(feature = "tea")]
    fn provenance(&self, s: &KeyStatus) -> &'static str {
        use crate::keystore::{CellId, Origin};
        if !s.has_key {
            return "";
        }
        let cell = CellId { mcc: s.mcc, mnc: s.mnc, colour: s.colour };
        match self.st.store.get(cell).map(|e| e.origin) {
            Some(Origin::Manual) => "entered here",
            Some(Origin::Recovered) => "recovered, saved",
            None => "recovered this session",
        }
    }
}

/// Parse a 16-hex-digit TA61 identity secret into its 8 bytes.
#[cfg(feature = "tea")]
fn parse_id_secret(s: &str) -> Option<[u8; 8]> {
    let s = s.trim();
    if s.len() != 16 || !s.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    let mut out = [0u8; 8];
    for (i, o) in out.iter_mut().enumerate() {
        *o = u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).ok()?;
    }
    Some(out)
}

/// A short line for where the key search is, and the colour to draw it. None
/// when idle, so a channel not being worked shows nothing.
fn recovery_label(r: Recovery) -> Option<(String, Color32)> {
    match r {
        Recovery::Idle => None,
        Recovery::Gathering { have, need, messages } => {
            Some((format!("gathering {have}/{need} ({messages} msgs)"), theme::LEGEND))
        }
        Recovery::Searching { gpu } => {
            Some((format!("searching ({})", if gpu { "GPU" } else { "CPU" }), theme::READOUT))
        }
        Recovery::Exhausted { dropped } => {
            Some((format!("not TEA1 ({dropped} tried)"), theme::LEGEND))
        }
        // The air never names the cipher, so a ruled-out TEA1 only says the
        // cipher is one of the others. TEA1 is the sole reduced-key cipher
        // this can break: TEA2/3/5/6 are full-length, and TEA4/7 are reduced
        // but different algorithms, not implemented and (TEA7) 56-bit. So a
        // key must be entered by hand here whatever it turns out to be.
        Recovery::NotTea1 => {
            Some(("not TEA1 (unbreakable here): enter a key".into(), theme::READOUT))
        }
    }
}
