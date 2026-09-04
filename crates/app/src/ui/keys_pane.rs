//! The key manager: the enciphered channels heard, and the keys held.
//!
//! A row per channel whose traffic is enciphered, showing what it is, the
//! encryption in force, and the key: none, one an operator typed, or one the
//! receiver recovered from the air. A channel with no key gets a box to type
//! one into; entering it installs the key on the front ends and writes it to
//! disk, so it is there the next time the network is heard. A recovered key
//! appears in the same list, marked as recovered, and is persisted the same
//! way. While a recovery runs, the row shows where it is: gathering the
//! retransmissions it needs, searching, or spent.
//!
//! TETRA is the one enciphered mode read today, so the identity a row shows
//! is a cell; the view is written to hold any enciphered channel a front end
//! reports, and reads as one, so a mode added later joins it without this
//! file changing.

use super::state::KeysState;
use super::*;
use crate::keystore::{CellId, Origin};
use decode::tea::Key;
use nodes::tetra_nodes::{KeyStatus, Recovery};

/// The key manager, over the live cell status and the stored keys.
pub(super) struct Keys<'a> {
    pub st: &'a mut KeysState,
    pub radio: Option<&'a Radio>,
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
                    "No enciphered channel heard yet. Tune one and it appears here; a channel \
                     with no key gets a box to enter one, and a key the receiver recovers from \
                     traffic shows up on its own.",
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
        let cell = CellId { mcc: s.mcc, mnc: s.mnc, colour: s.colour };
        let frame = egui::Frame::NONE
            .inner_margin(egui::Margin::symmetric(12, 6))
            .fill(if striped { Color32::from_rgb(0x24, 0x27, 0x2D) } else { Color32::TRANSPARENT });
        frame.show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(value(format!("{:.4} MHz", s.channel_hz / 1e6)).size(11.0));
                ui.label(legend(&format!("{}/{}/cc{}", s.mcc, s.mnc, s.colour)).size(11.0));
                ui.label(
                    egui::RichText::new(if s.aie == 0 { "clear".into() } else { format!("AIE-{}", s.aie) })
                        .color(if s.aie == 0 { theme::LEGEND } else { theme::READOUT })
                        .size(11.0),
                );

                // What key is in force, from the store (which the live status
                // also carries, but the store knows the provenance).
                let stored = self.st.store.get(cell).map(|e| e.origin);
                let label = match (s.key.is_some(), stored) {
                    (true, Some(Origin::Manual)) => "key: manual",
                    (true, Some(Origin::Recovered)) => "key: recovered",
                    (true, None) => "key: set",
                    (false, _) => "no key",
                };
                ui.label(
                    egui::RichText::new(label)
                        .color(if s.key.is_some() { theme::VALUE } else { theme::LEGEND })
                        .size(11.0),
                );

                if s.reuse_pairs > 0 {
                    ui.label(legend(&format!("{} reuse", s.reuse_pairs)).size(11.0));
                }

                // The key search, shown as it happens rather than only when
                // it lands: what a key with no manual entry is doing.
                if s.key.is_none() {
                    if let Some((text, colour)) = recovery_label(s.recovery) {
                        ui.label(egui::RichText::new(text).color(colour).size(11.0));
                    }
                }

                // Manual entry for an enciphered cell with no key.
                if s.aie != 0 && s.key.is_none() {
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
                            self.apply(cell, key, Origin::Manual);
                            self.st.typing.remove(&cell.tag_key());
                        }
                    }
                }

                // A key the receiver recovered but has not yet been saved:
                // persist it so it survives a restart.
                if let (Some(key), None) = (s.key, stored) {
                    self.persist(cell, key, Origin::Recovered);
                }

                if self.st.store.get(cell).is_some() {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("forget").clicked() {
                            self.st.store.remove(cell);
                            let _ = self.st.store.save();
                        }
                    });
                }
            });
        });
    }

    /// Install a key on the running graph and persist it.
    fn apply(&mut self, cell: CellId, key: Key, origin: Origin) {
        self.cmds.push(Cmd::TetraKey { colour: cell.colour, key });
        self.persist(cell, key, origin);
    }

    /// Record a key in the store and write it out.
    fn persist(&mut self, cell: CellId, key: Key, origin: Origin) {
        self.st.store.insert(cell, key, origin);
        let _ = self.st.store.save();
    }
}

/// A short line for where the key search is, and the colour to draw it. None
/// when idle, so a channel not being worked shows nothing.
fn recovery_label(r: Recovery) -> Option<(String, Color32)> {
    match r {
        Recovery::Idle => None,
        Recovery::Gathering { have, need, messages } => Some((
            format!("gathering {have}/{need} ({messages} msgs)"),
            theme::LEGEND,
        )),
        Recovery::Searching { gpu } => {
            Some((format!("searching ({})", if gpu { "GPU" } else { "CPU" }), theme::READOUT))
        }
        Recovery::Exhausted { dropped } => {
            Some((format!("no TEA1 key ({dropped} tried)"), theme::LEGEND))
        }
        Recovery::NotTea1 => Some(("not TEA1 (TEA2/3): key must be entered".into(), theme::READOUT)),
    }
}
