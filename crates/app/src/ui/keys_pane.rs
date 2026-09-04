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
            ui.label(legend("encryption keys"));
            let enc = live.iter().filter(|s| s.aie != 0).count();
            let n = live.len();
            let chan = if n == 1 { "channel" } else { "channels" };
            ui.label(value(format!("{n} {chan}, {enc} enciphered")).size(11.0));
        });
        ui.add_space(6.0);

        if live.is_empty() {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| {
                hint(
                    ui,
                    "No enciphered channel heard yet. Tune one and it appears here with its \
                     encryption mode and, where the build can, its key.",
                );
            });
            return;
        }

        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            for (n, s) in live.iter().enumerate() {
                self.row(ui, s, n % 2 == 1);
                ui.add_space(2.0);
            }
        });
    }

    fn row(&mut self, ui: &mut egui::Ui, s: &KeyStatus, striped: bool) {
        let frame = egui::Frame::NONE
            .inner_margin(egui::Margin::symmetric(12, 6))
            .fill(if striped { Color32::from_rgb(0x24, 0x27, 0x2D) } else { Color32::TRANSPARENT });
        frame.show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(value(format!("{:.4} MHz", s.channel_hz / 1e6)).size(11.0));
                ui.label(legend(&format!("{}/{}/cc{}", s.mcc, s.mnc, s.colour)).size(11.0));
                ui.label(
                    egui::RichText::new(if s.aie == 0 {
                        "clear".into()
                    } else {
                        format!("AIE-{}", s.aie)
                    })
                    .color(if s.aie == 0 { theme::LEGEND } else { theme::READOUT })
                    .size(11.0),
                );

                ui.label(
                    egui::RichText::new(if s.has_key { "key: set" } else { "no key" })
                        .color(if s.has_key { theme::VALUE } else { theme::LEGEND })
                        .size(11.0),
                );

                if s.reuse_pairs > 0 {
                    ui.label(legend(&format!("{} reuse", s.reuse_pairs)).size(11.0));
                }

                // The key search, shown as it happens rather than only when
                // it lands. Always drawn: without `tea` the phase is Idle, so
                // nothing shows, which is correct.
                if !s.has_key {
                    if let Some((text, colour)) = recovery_label(s.recovery) {
                        ui.label(egui::RichText::new(text).color(colour).size(11.0));
                    }
                }

                // The key material: entry, provenance, persistence. Only with
                // the cipher and store the `tea` feature brings.
                #[cfg(feature = "tea")]
                self.keying(ui, s);
            });
        });
    }

    /// The key-material controls for one row: provenance from the store, a
    /// box to enter a key by hand, persistence of a recovered one, and a
    /// forget button. Present only with the `tea` feature.
    #[cfg(feature = "tea")]
    fn keying(&mut self, ui: &mut egui::Ui, s: &KeyStatus) {
        use crate::keystore::{CellId, Origin};
        let cell = CellId { mcc: s.mcc, mnc: s.mnc, colour: s.colour };
        let stored = self.st.store.get(cell).map(|e| e.origin);

        // Refine the "key: set" label with where the key came from.
        if let Some(origin) = stored {
            let word = match origin {
                Origin::Manual => "manual",
                Origin::Recovered => "recovered",
            };
            ui.label(egui::RichText::new(word).color(theme::LEGEND).size(11.0));
        }

        // A recovered key (in force but not in the store) is shown as such;
        // persisting it across restarts would need the key value threaded up
        // through the status snapshot, which it is not yet. It stays in force
        // for the session regardless.
        if s.has_key && stored.is_none() {
            ui.label(egui::RichText::new("recovered").color(theme::LEGEND).size(11.0));
        }

        // Manual entry for an enciphered cell with no key.
        if s.aie != 0 && !s.has_key {
            let buf = self.st.typing.entry(cell.tag_key()).or_default();
            ui.add(
                egui::TextEdit::singleline(buf)
                    .hint_text("hex key: 8 for TEA1, 20 for TEA2")
                    .desired_width(220.0)
                    .font(egui::FontId::monospace(12.0)),
            );
            let parsed = crate::keystore::parse_typed_key(buf);
            if ui.add_enabled(parsed.is_some(), egui::Button::new("set")).clicked() {
                if let Some(key) = parsed {
                    self.cmds.push(Cmd::TetraKey { colour: cell.colour, key });
                    self.st.store.insert(cell, key, Origin::Manual);
                    let _ = self.st.store.save();
                    self.st.typing.remove(&cell.tag_key());
                }
            }
        }

        // Identity secret entry: a 16-hex-digit TA61 `c` that de-anonymises
        // the encrypted identities on this cell, independent of the voice
        // key and working on TEA2/3. Session-only; not persisted yet.
        if s.aie != 0 {
            let key = format!("{}#id", cell.tag_key());
            let buf = self.st.typing.entry(key.clone()).or_default();
            ui.add(
                egui::TextEdit::singleline(buf)
                    .hint_text("identity secret: 16 hex")
                    .desired_width(180.0)
                    .font(egui::FontId::monospace(12.0)),
            );
            let secret = parse_id_secret(buf);
            if ui.add_enabled(secret.is_some(), egui::Button::new("id")).clicked() {
                if let Some(c) = secret {
                    self.cmds.push(Cmd::TetraIdSecret { colour: cell.colour, c });
                    self.st.typing.remove(&key);
                }
            }
        }

        if stored.is_some() {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("forget").clicked() {
                    self.st.store.remove(cell);
                    let _ = self.st.store.save();
                }
            });
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
