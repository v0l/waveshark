//! The message view: what was written, rather than who was talking.
//!
//! Text is the one thing on this receiver that is not a number, so it is not
//! drawn as a table cell clipped to a column. Each message is a card: a
//! header saying where it came from and when, and the words underneath at
//! full width, wrapped. A long page stays readable and a short one takes one
//! line. Striping was doing that job and could not: a message that wraps to
//! four lines has no edge, and two grey rows next to each other read as one
//! long message.

use super::state::MessagesState;
use super::*;
use crate::messages::Message;

/// The message list, over what it lists.
pub(super) struct Msgs<'a> {
    pub st: &'a mut MessagesState,
}

/// What the list wants done that it cannot do itself.
pub(super) enum Action {
    /// Throw the list away.
    Clear,
}

impl Msgs<'_> {
    /// Draw the list, and say what a click asked for.
    pub(super) fn show(self, ui: &mut egui::Ui) -> Option<Action> {
        let now = std::time::Instant::now();
        let msgs: Vec<Message> = self.st.list.recent().into_iter().cloned().collect();
        let mut act = None;

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            // Read back from the file and heard just now are counted apart.
            // The list is loaded at start so an overnight watch is readable
            // in the morning, and a page from a band the receiver is no
            // longer pointed at reads as traffic arriving now unless the
            // header says otherwise.
            let logged = msgs.iter().filter(|m| m.logged).count();
            let live = msgs.len() - logged;
            let mut head = theme::Line::new().legend("messages");
            head = match (live, logged) {
                (n, 0) => head.value(format!("{n} received")),
                (0, k) => head.value(format!("{k} from the log")),
                (n, k) => head.value(format!("{n} received, {k} from the log")),
            };
            head.size(11.0).show(ui);
            if !self.st.list.is_empty() {
                let filter = &mut self.st.filter;
                ui.add_space(12.0);
                ui.add(egui::TextEdit::singleline(filter).hint_text("filter").desired_width(160.0));
                if !filter.is_empty() && ui.button("Clear filter").clicked() {
                    filter.clear();
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.add_space(12.0);
                    if ui.button("Clear messages").clicked() {
                        act = Some(Action::Clear);
                    }
                });
            }
        });
        ui.add_space(6.0);

        if self.st.list.is_empty() {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| {
                hint(
                    ui,
                    "Nothing has been written yet. A decode the decoder says somebody wrote \
                     lands here: a TETRA short data message, an M17 SMS packet, an APRS \
                     message, a pager page. A station's radiotext and an aircraft's \
                     datalink are text and are not messages, so they stay in the packet \
                     list.",
                );
                hint(
                    ui,
                    &format!(
                        "They are kept as they arrive, a file a day in {}, so an overnight \
                         watch is readable in the morning.",
                        crate::messagelog::messages_dir().display()
                    ),
                );
            });
            return act;
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

        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 6.0;
            // The cards keep their own margin off the edge of the pane, so
            // the rule down the left of one is not against the window frame.
            egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
                for m in &shown {
                    // Drawn and not clickable. A message is a thing that
                    // was said, and the frequency it arrived on is a
                    // detail of how it got here: retuning the receiver
                    // because somebody read a page is the last thing
                    // this view should do, and it took the receiver off
                    // whatever it was watching.
                    message_card(ui, m, now);
                }
            });
            ui.add_space(8.0);
        });
        act
    }
}

/// One message: a header saying where it came from, then the words.
fn message_card(ui: &mut egui::Ui, m: &Message, now: std::time::Instant) -> egui::Response {
    let inner = widgets::card(
        ui,
        // Cyan for what the receiver heard, and no rail for what it read
        // back off the file: the traffic on the screen now is what is on
        // the air now.
        if m.logged { None } else { Some(theme::TRACE) },
        |ui| {
            let mut head = theme::Line::new()
                .legend(&m.system)
                .value(format!("{:.4} MHz", m.channel_hz / 1e6))
                .size(11.0);
            let title = m.title();
            if !title.is_empty() {
                head = head.value(title).tint(theme::READOUT).size(11.0);
            }
            // Heard more than once means retransmitted, not sent twice: the
            // count is here so a card that says 3 is not read as three pages.
            if m.heard > 1 {
                head = head.legend(&format!("x{}", m.heard));
            }
            head.show(ui);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // A message off the file says when it was heard rather than
                // how long ago, and says that it came off the file: the
                // receiver was somewhere else then, possibly on another
                // band, and an age alone reads as something arriving now.
                // Every message carries the clock time it was heard at, so
                // one off the file says that rather than an age: the
                // receiver was somewhere else then, possibly on another
                // band, and an age alone reads as something arriving now.
                match m.logged {
                    true => theme::Line::new().legend("logged").value(m.when()).size(11.0).show(ui),
                    false => theme::Line::new().legend(&age(m.age(now))).show(ui),
                };
            });
        },
        |ui| {
            theme::Line::new().words(&m.text).wrapped(ui);
        },
    );
    // The same gesture as the call list: clicking a card puts the dial on
    // the channel it was heard on.
    ui.interact(inner.response.rect, ui.id().with(m.first).with(&m.text), Sense::click())
}

/// How long ago, short enough for the corner of a header.
fn age(d: std::time::Duration) -> String {
    let s = d.as_secs();
    match s {
        0..=59 => format!("{s}s"),
        60..=3599 => format!("{}m", s / 60),
        _ => format!("{}h", s / 3600),
    }
}
