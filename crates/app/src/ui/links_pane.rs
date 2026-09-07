//! The data links view: who is talking to whom, and what passed between them.
//!
//! Two panes in one, the way a conversation list and a follow window are two
//! panes in Wireshark. The directory is a table of links, most recently heard
//! first; picking one opens its packets underneath, in the order they
//! arrived, with what each carried in the clear.
//!
//! Nothing here knows a protocol. A row exists because a decode named an end,
//! so a decoder added later appears in this view the day it names its fields
//! the way the rest do. See `crate::links`.

use super::state::LinksState;
use super::*;
use crate::links::{Link, Moment};

pub(super) struct LinksView<'a> {
    pub st: &'a mut LinksState,
}

pub(super) enum Action {
    /// Tune the dial to the channel a link was heard on.
    Tune(f64),
    /// Throw the directory away.
    Clear,
    /// Read the packet log back into the directory, so it holds what was
    /// heard before the receiver was started as well as since.
    LoadLog,
}

impl LinksView<'_> {
    pub(super) fn show(self, ui: &mut egui::Ui) -> Option<Action> {
        let now = std::time::Instant::now();
        let links: Vec<Link> = self.st.list.active(now).into_iter().cloned().collect();
        let mut act = None;

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            let live = links.iter().filter(|l| l.live(now)).count();
            theme::Line::new()
                .legend("links")
                .value(format!("{} seen", links.len()))
                .size(11.0)
                .show(ui);
            if live > 0 {
                theme::Line::new()
                    .legend("live")
                    .value(live.to_string())
                    .tint(theme::OK)
                    .size(11.0)
                    .show(ui);
            }
            if !links.is_empty() {
                let filter = &mut self.st.filter;
                ui.add_space(12.0);
                ui.add(
                    egui::TextEdit::singleline(filter).hint_text("filter").desired_width(180.0),
                );
                if !filter.is_empty() && ui.button("Clear filter").clicked() {
                    filter.clear();
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.add_space(12.0);
                    if ui.button("Clear links").clicked() {
                        act = Some(Action::Clear);
                    }
                    if ui.button("Load from log").clicked() {
                        act = Some(Action::LoadLog);
                    }
                });
            }
        });
        if let Some(e) = &self.st.error {
            ui.horizontal(|ui| {
                ui.add_space(12.0);
                theme::Line::new().legend(e).tint(theme::FAULT).size(11.0).show(ui);
            });
        }
        ui.add_space(6.0);

        if links.is_empty() {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| {
                hint(
                    ui,
                    "Nothing has named itself yet. Any decode carrying a from or to field \
                     lands here: an advertiser, a meter, a pager capcode, a radio calling \
                     a talkgroup. Pick a link to follow what passed between its ends.",
                );
                ui.add_space(8.0);
                if ui.button("Load from log").clicked() {
                    act = Some(Action::LoadLog);
                }
            });
            return act;
        }

        let needle = self.st.filter.to_lowercase();
        let shown: Vec<&Link> = links
            .iter()
            .filter(|l| needle.is_empty() || l.title().to_lowercase().contains(&needle))
            .collect();

        // The chosen link, if it is still in the directory: a link that aged
        // out while its packets were on screen closes the follow view rather
        // than showing a list that can no longer grow.
        let chosen: Option<&Link> =
            self.st.chosen.as_ref().and_then(|t| links.iter().find(|l| &l.title() == t));

        let height = ui.available_height();
        let table = if chosen.is_some() { height * 0.45 } else { height };
        egui::ScrollArea::vertical()
            .id_salt("links-directory")
            .max_height(table)
            .auto_shrink([false, false])
            .show(ui, |ui| {
                egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
                    for l in &shown {
                        let picked = self.st.chosen.as_deref() == Some(l.title().as_str());
                        let r = link_row(ui, l, now, picked);
                        if r.clicked() {
                            self.st.chosen =
                                if picked { None } else { Some(l.title()) };
                        }
                        if r.double_clicked() {
                            act = Some(Action::Tune(l.channel_hz));
                        }
                    }
                });
                ui.add_space(8.0);
            });

        if let Some(l) = chosen {
            ui.separator();
            ui.horizontal(|ui| {
                ui.add_space(12.0);
                theme::Line::new()
                    .legend("following")
                    .value(l.title())
                    .tint(theme::READOUT)
                    .size(11.0)
                    .value(format!("{} packets", l.packets))
                    .value(format!("{} B", l.bytes))
                    .show(ui);
            });
            ui.add_space(4.0);
            egui::ScrollArea::vertical()
                .id_salt("links-follow")
                .stick_to_bottom(true)
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(
                        ui,
                        |ui| {
                            let first = l.moments.front().map(|m| m.at);
                            for m in &l.moments {
                                moment_row(ui, m, first);
                            }
                        },
                    );
                    ui.add_space(8.0);
                });
        }
        act
    }
}

/// One link in the directory.
fn link_row(ui: &mut egui::Ui, l: &Link, now: std::time::Instant, picked: bool) -> egui::Response {
    let tint = if l.live(now) { theme::OK } else { theme::TRACE };
    let inner = widgets::card(
        ui,
        Some(if picked { theme::READOUT } else { tint }),
        |ui| {
            theme::Line::new()
                .legend(&l.system)
                .value(crate::links::end_label(&l.from).to_string())
                .tint(theme::READOUT)
                .size(11.0)
                .legend("->")
                .value(crate::links::end_label(&l.to).to_string())
                .size(11.0)
                .show(ui);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                theme::Line::new().legend(&age(l.age(now))).show(ui);
            });
        },
        |ui| {
            let mut line = theme::Line::new()
                .legend("packets")
                .value(l.packets.to_string())
                .size(11.0)
                .legend("bytes")
                .value(l.bytes.to_string())
                .size(11.0)
                .legend("on")
                .value(format!("{:.4} MHz", l.channel_hz / 1e6))
                .size(11.0);
            if l.last_rssi_dbfs.is_finite() {
                line = line.legend("rssi").value(format!("{:.0} dBFS", l.last_rssi_dbfs)).size(11.0);
            }
            // A link whose frames fail their checks is not a link, and the
            // row says so rather than letting the count speak for it.
            if l.crc_failures > 0 {
                line = line
                    .legend("bad crc")
                    .value(l.crc_failures.to_string())
                    .tint(theme::FAULT)
                    .size(11.0);
            }
            line.show(ui);
        },
    );
    ui.interact(inner.response.rect, ui.id().with(l.title()), Sense::click())
}

/// One packet inside a followed link.
fn moment_row(ui: &mut egui::Ui, m: &Moment, first: Option<std::time::Instant>) {
    let t = first.map(|f| m.at.saturating_duration_since(f).as_secs_f64()).unwrap_or(0.0);
    ui.horizontal_wrapped(|ui| {
        theme::Line::new()
            .legend(&format!("{t:>8.3}"))
            .value(m.protocol.clone())
            .size(11.0)
            .show(ui);
        if m.rssi_dbfs.is_finite() {
            theme::Line::new().legend(&format!("{:.0} dBFS", m.rssi_dbfs)).size(11.0).show(ui);
        }
        if m.crc == Some(false) {
            theme::Line::new().legend("crc failed").tint(theme::FAULT).size(11.0).show(ui);
        }
        theme::Line::new().legend(&format!("{} B", m.bytes)).size(11.0).show(ui);
    });
    // What it said, where it said anything: this is the point of following a
    // link, so it gets a line of its own at full width rather than a column.
    if let Some(t) = &m.text {
        ui.horizontal(|ui| {
            ui.add_space(16.0);
            theme::Line::new().words(t).wrapped(ui);
        });
    } else if !m.detail.is_empty() {
        ui.horizontal(|ui| {
            ui.add_space(16.0);
            theme::Line::new().legend(&m.detail).size(10.0).show(ui);
        });
    }
}

fn age(d: std::time::Duration) -> String {
    let s = d.as_secs();
    match s {
        0..=59 => format!("{s}s"),
        60..=3599 => format!("{}m", s / 60),
        _ => format!("{}h", s / 3600),
    }
}
