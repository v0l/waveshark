//! The control view: where the sticks are on every handset in earshot.
//!
//! One card per transmitter rather than one row per frame. A control link
//! sends tens of frames a second and its channels only move by a few
//! microseconds between them, so the packet list is the wrong instrument
//! entirely: what a person wants is a bar per channel that moves, and the
//! knowledge that the link is still up.
//!
//! The bars are painted rather than built from widgets, for the reason the
//! other tables are: sixteen channels on a handful of links is hundreds of
//! rectangles a frame otherwise. Everything with words in it goes through
//! `theme::Line` and `widgets::cell` as usual.

use super::*;
use crate::control::{Control, RANGE_US};

pub(super) struct ControlView<'a> {
    pub st: &'a mut crate::ui::state::ControlState,
}

impl ControlView<'_> {
    pub(super) fn show(self, ui: &mut egui::Ui) {
        let now = std::time::Instant::now();
        let links: Vec<Control> = self.st.list.active(now).into_iter().cloned().collect();

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            let live = links.iter().filter(|c| c.live(now)).count();
            theme::Line::new()
                .legend("control links")
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
        });
        ui.add_space(6.0);

        if links.is_empty() {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| {
                hint(
                    ui,
                    "No handset has been heard. A model control link lands here as soon as \
                     its decoder reads the sticks: ExpressLRS on 2.4 GHz is the one the \
                     receiver places for itself, on a chirp the classifier has named. The \
                     bars are servo pulse widths, so 1500 us is a centred stick.",
                );
            });
            return;
        }

        egui::ScrollArea::vertical().id_salt("control-links").auto_shrink([false, false]).show(
            ui,
            |ui| {
                egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
                    for c in &links {
                        link_card(ui, c, now);
                    }
                });
                ui.add_space(8.0);
            },
        );
    }
}

/// One handset: what it is, how it is being heard, and its channels.
fn link_card(ui: &mut egui::Ui, c: &Control, now: std::time::Instant) {
    let live = c.live(now);
    widgets::card(
        ui,
        Some(if live { theme::OK } else { theme::TRACE }),
        |ui| {
            theme::Line::new()
                .legend(&c.system)
                .value(c.id.clone())
                .tint(theme::READOUT)
                .size(11.0)
                .show(ui);
            // Armed is the one thing on here worth seeing from across a
            // room, so it is a word in the fault colour rather than a flag
            // among the numbers. It is what the handset is asking for, not
            // what the aircraft did, and the caption says so.
            if c.armed == Some(true) {
                theme::Line::new().legend("armed").tint(theme::FAULT).size(11.0).show(ui);
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                theme::Line::new().legend(&age(c.age(now))).show(ui);
            });
        },
        |ui| {
            let mut line = theme::Line::new()
                .legend("frames")
                .value(c.frames.to_string())
                .size(11.0)
                .legend("channels")
                .value(c.carried().to_string())
                .size(11.0)
                .legend("last on")
                .value(format!("{:.4} MHz", c.channel_hz / 1e6))
                .size(11.0);
            if let Some(rate) = c.frame_rate() {
                line = line.legend("rate").value(format!("{rate:.0} Hz")).size(11.0);
            }
            if c.last_rssi_dbfs.is_finite() {
                line =
                    line.legend("rssi").value(format!("{:.0} dBFS", c.last_rssi_dbfs)).size(11.0);
            }
            if let Some(mw) = c.uplink_power_mw {
                line = line.legend("uplink").value(format!("{mw} mW")).size(11.0);
            }
            line.show(ui);
            ui.add_space(4.0);
            channels(ui, c, now);
        },
    );
}

/// The channels, in two columns of eight.
///
/// Every channel the link has ever carried is drawn, including the ones this
/// frame did not: a stick that stops being sent has not moved to zero, and a
/// gap in the grid would make a sixteen channel model look like an eight
/// channel one every other frame.
fn channels(ui: &mut egui::Ui, c: &Control, now: std::time::Instant) {
    let carried = c.channels.iter().rposition(Option::is_some).map_or(0, |i| i + 1);
    if carried == 0 {
        return;
    }
    let rows = carried.div_ceil(2);
    let width = ui.available_width();
    let col = (width / 2.0).max(120.0);
    let (rect, _) = ui.allocate_exact_size(
        egui::Vec2::new(width, rows as f32 * widgets::ROW_H),
        egui::Sense::hover(),
    );
    if !ui.is_rect_visible(rect) {
        return;
    }
    let p = ui.painter();
    for i in 0..carried {
        let (r, k) = (i % rows, i / rows);
        let top = rect.top() + r as f32 * widgets::ROW_H;
        let left = rect.left() + k as f32 * col;
        // A gutter, or the microseconds of the left column read as part of
        // the caption of the right one.
        let row = egui::Rect::from_min_max(
            egui::Pos2::new(left, top),
            egui::Pos2::new(left + col - 18.0, top + widgets::ROW_H),
        );
        channel_row(p, row, i, c.channels[i], c.channel_at[i], now);
    }
}

/// One channel: its number, a bar, and the width in microseconds.
fn channel_row(
    p: &egui::Painter,
    row: egui::Rect,
    index: usize,
    us: Option<u16>,
    at: Option<std::time::Instant>,
    now: std::time::Instant,
) {
    let label_w = 34.0;
    let value_w = 56.0;
    widgets::cell(p, row, row.left(), label_w, &format!("ch{}", index + 1), theme::LEGEND);

    let bar = egui::Rect::from_min_max(
        egui::Pos2::new(row.left() + label_w, row.top() + 3.0),
        egui::Pos2::new(row.right() - value_w, row.bottom() - 3.0),
    );
    if bar.width() < 8.0 {
        return;
    }
    p.rect_filled(bar, 1.0, theme::WELL);
    p.rect_stroke(bar, 1.0, egui::Stroke::new(1.0, theme::ETCH), egui::StrokeKind::Inside);
    // The centre mark, since a stick's rest position is the reading a person
    // checks first and a bar without it is a length with nothing to compare.
    let (lo, hi) = (f32::from(RANGE_US.0), f32::from(RANGE_US.1));
    let at_us = |v: f32| bar.left() + ((v - lo) / (hi - lo)).clamp(0.0, 1.0) * bar.width();
    let mid = at_us(1500.0);
    p.line_segment(
        [egui::Pos2::new(mid, bar.top()), egui::Pos2::new(mid, bar.bottom())],
        egui::Stroke::new(1.0, theme::ETCH),
    );

    let Some(us) = us else {
        widgets::cell(p, row, row.right() - value_w, value_w, "-", theme::LEGEND);
        return;
    };
    // A channel the link has stopped sending keeps its last position and is
    // drawn dimmed, which is the honest reading: this is where it was, and
    // nothing has said otherwise since.
    let stale = at.is_none_or(|t| now.saturating_duration_since(t) > crate::control::LIVE);
    let colour = if stale { theme::READOUT_DIM } else { theme::READOUT };
    let x = at_us(f32::from(us));
    let fill = egui::Rect::from_min_max(
        egui::Pos2::new(mid.min(x), bar.top() + 1.0),
        egui::Pos2::new(mid.max(x).max(mid.min(x) + 1.0), bar.bottom() - 1.0),
    );
    p.rect_filled(fill, 0.0, colour.gamma_multiply(0.55));
    p.line_segment(
        [egui::Pos2::new(x, bar.top()), egui::Pos2::new(x, bar.bottom())],
        egui::Stroke::new(2.0, colour),
    );
    widgets::cell(
        p,
        row,
        row.right() - value_w + 6.0,
        value_w,
        &format!("{us:>4} us"),
        if stale { theme::LEGEND } else { theme::VALUE },
    );
}

fn age(d: std::time::Duration) -> String {
    let s = d.as_secs();
    match s {
        0 => format!("{} ms", d.as_millis()),
        1..=59 => format!("{s}s"),
        60..=3599 => format!("{}m", s / 60),
        _ => format!("{}h", s / 3600),
    }
}
