//! The message view: what was written, rather than who was talking.
//!
//! Text is the one thing on this receiver that is not a number, so it is not
//! drawn as a table cell clipped to a column. Each message is a block: a
//! header line saying where it came from and when, and the words underneath
//! at full width, wrapped. A long page stays readable and a short one takes
//! one line.

use super::state::MessagesState;
use super::*;
use crate::messages::Message;

/// The message list, over what it lists.
pub(super) struct Msgs<'a> {
    pub st: &'a mut MessagesState,
}

impl Msgs<'_> {
    /// Draw the list, and say which channel a click asked to tune to.
    pub(super) fn show(self, ui: &mut egui::Ui) -> Option<f64> {
        let now = std::time::Instant::now();
        let msgs: Vec<Message> = self.st.list.recent().into_iter().cloned().collect();

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            ui.label(legend("messages"));
            ui.label(value(format!("{} received", msgs.len())).size(11.0));
            if !self.st.list.is_empty() {
                let filter = &mut self.st.filter;
                ui.add_space(12.0);
                ui.add(
                    egui::TextEdit::singleline(filter)
                        .hint_text("filter")
                        .desired_width(160.0),
                );
                if !filter.is_empty() && ui.button("clear").clicked() {
                    filter.clear();
                }
            }
        });
        ui.add_space(6.0);

        if self.st.list.is_empty() {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| {
                hint(
                    ui,
                    "Nothing has been written yet. Any decode carrying a text or message \
                     field lands here: a TETRA short data message, an M17 SMS packet, an \
                     APRS message, a pager page.",
                );
            });
            return None;
        }

        let needle = self.st.filter.to_lowercase();
        let shown: Vec<&Message> = msgs
            .iter()
            .filter(|m| {
                needle.is_empty()
                    || m.text.to_lowercase().contains(&needle)
                    || m.title().to_lowercase().contains(&needle)
                    || m.system.to_lowercase().contains(&needle)
            })
            .collect();

        let mut tune_to = None;
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            for (n, m) in shown.iter().enumerate() {
                if n > 0 {
                    ui.add_space(2.0);
                }
                let resp = message_block(ui, m, now, n % 2 == 1);
                if resp.clicked() {
                    tune_to = Some(m.channel_hz);
                }
            }
            ui.add_space(8.0);
        });
        // The same gesture as the call list: clicking a row puts the dial on
        // the channel it was heard on.
        tune_to
    }
}

/// One message: header line, then the words.
fn message_block(
    ui: &mut egui::Ui,
    m: &Message,
    now: std::time::Instant,
    striped: bool,
) -> egui::Response {
    let frame = egui::Frame::NONE
        .inner_margin(egui::Margin::symmetric(12, 6))
        .fill(if striped { Color32::from_rgb(0x24, 0x27, 0x2D) } else { Color32::TRANSPARENT });
    let inner = frame.show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.label(legend(&m.system).size(11.0));
            ui.label(value(format!("{:.4} MHz", m.channel_hz / 1e6)).size(11.0));
            let title = m.title();
            if !title.is_empty() {
                ui.label(egui::RichText::new(title).color(theme::READOUT).size(11.0));
            }
            // Heard more than once means retransmitted, not sent twice: the
            // count is here so a row that says 3 is not read as three pages.
            if m.heard > 1 {
                ui.label(legend(&format!("x{}", m.heard)).size(11.0));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(legend(&age(m.age(now))).size(11.0));
            });
        });
        ui.label(
            egui::RichText::new(&m.text)
                .color(theme::VALUE)
                .family(FontFamily::Name(theme::READOUT_FONT.into()))
                .size(13.0),
        );
    });
    ui.interact(inner.response.rect, ui.id().with(m.first).with(&m.text), Sense::click())
}

/// How long ago, short enough for the corner of a row.
fn age(d: std::time::Duration) -> String {
    let s = d.as_secs();
    match s {
        0..=59 => format!("{s}s"),
        60..=3599 => format!("{}m", s / 60),
        _ => format!("{}h", s / 3600),
    }
}
