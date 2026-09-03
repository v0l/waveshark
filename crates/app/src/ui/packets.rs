//! The packet log and its inspector.

use super::state::LogState;
use super::*;

/// What the log wants done that it cannot do itself.
pub(super) enum Action {
    /// Turn decoding of the whole span on or off.
    Decode(bool),
    /// Open one of the settings panels the header carries a button for.
    Open(Settings),
}

/// The log, over the packets it lists.
pub(super) struct Log<'a> {
    pub st: &'a mut LogState,
    pub radio: Option<&'a Radio>,
    pub scanners: &'a crate::scanners::Scanners,
    pub center: f64,
    pub rate: f64,
    pub decode_on: bool,
    /// Whether the receiver is running the operator's own graph, in which
    /// case the decode switch is not the pane's to throw.
    pub cmds: &'a mut Vec<Cmd>,
    pub acts: Vec<Action>,
}

impl Log<'_> {
    /// The packet log: everything decoded anywhere in the span.
    pub(super) fn show(mut self, ui: &mut egui::Ui) -> Vec<Action> {
        if !self.st.open {
            return self.acts;
        }
        Panel::bottom("decodes")
            .default_size(230.0)
            // Drag the top edge to resize. A band that is busy wants a tall
            // list; one that is quiet wants the waterfall back.
            .resizable(true)
            .min_size(64.0)
            .max_size(720.0)
            .show_separator_line(true)
            .frame(
                egui::Frame::NONE
                    .fill(theme::PANEL)
                    .inner_margin(egui::Margin::symmetric(12, 8)),
            )
            .show(ui, |ui| {
                self.log_header(ui);
                ui.add_space(4.0);
                let selected = self
                    .st
                    .selected
                    .and_then(|id| self.st.decodes.iter().find(|l| l.id == id))
                    .map(|l| l.rec.clone());
                // The inspector lives inside this window and takes its room
                // from the list, so the window itself stays the size it was
                // dragged to. Its height is one number held by the app, not
                // read from what the packet holds, so a packet with no bytes
                // or no samples gets the same height as one with both and
                // nothing jumps when moving between rows.
                let avail = ui.available_height();
                // The list, the drag handle and the inspector body are three
                // widgets in a column, so two gaps of item spacing sit between
                // them. Spending the whole height as though they did not made
                // the content taller than the panel by those two gaps, and
                // what went over the edge was the toolbar at the top: the row
                // with the decode switch and the frame count was drawn half
                // outside the panel's clip rect.
                let gap = ui.spacing().item_spacing.y;
                let (inspect_h, gaps) = if selected.is_some() {
                    self.st.inspector_h = self.st.inspector_h.clamp(INSPECTOR_MIN_H, inspector_max(avail, gap));
                    (self.st.inspector_h, gap * 2.0)
                } else {
                    (0.0, 0.0)
                };
                let list_h = (avail - inspect_h - gaps).max(24.0);
                // Two nested scroll areas so the headings stay above the rows
                // vertically but travel with them sideways, which is the only
                // arrangement where a narrow window can still reach the last
                // column and the headings never leave the top.
                // Both areas are given an explicit height. Without it the
                // content asks for as much room as it has rows, the panel
                // grows to match, and the headings are pushed off the top of
                // the window they are supposed to be pinned to.
                egui::ScrollArea::horizontal()
                    .auto_shrink([false, false])
                    .max_height(list_h)
                    .show(ui, |ui| {
                    let w = ui.available_width().max(Self::table_width());
                    ui.set_min_width(w);
                    if !self.st.decodes.is_empty() {
                        self.log_header_row(ui, w);
                    }
                    egui::ScrollArea::vertical()
                        .auto_shrink([false, false])
                        .max_height((list_h - widgets::ROW_H).max(16.0))
                        .stick_to_bottom(true)
                        .id_salt("packet_rows")
                        .show(ui, |ui| self.log_rows(ui, w));
                });
                if let Some(rec) = &selected {
                    self.inspector(ui, rec, inspect_h, avail);
                }
            });
        self.acts
    }

    /// The packet inspector under the list: a drag handle, then the burst
    /// and the bytes in exactly `height` pixels.
    fn inspector(&mut self, ui: &mut egui::Ui, rec: &DecodeRecord, height: f32, avail: f32) {
        let w = ui.available_width();
        // The handle: a thin strip that drags the divider. Dragging up makes
        // the inspector taller and the list shorter; the window is unmoved.
        let (hrect, hresp) = ui.allocate_exact_size(Vec2::new(w, HANDLE_H), Sense::drag());
        if hresp.dragged() {
            let gap = ui.spacing().item_spacing.y;
            self.st.inspector_h = (self.st.inspector_h - hresp.drag_delta().y)
                .clamp(INSPECTOR_MIN_H, inspector_max(avail, gap));
        }
        if hresp.hovered() || hresp.dragged() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeVertical);
        }
        let y = hrect.center().y;
        ui.painter().line_segment(
            [Pos2::new(hrect.left(), y), Pos2::new(hrect.right(), y)],
            Stroke::new(1.0, if hresp.hovered() || hresp.dragged() { theme::READOUT } else { theme::ETCH }),
        );
        // The body, in the rest of the height regardless of what it holds.
        let body_h = (height - HANDLE_H).max(8.0);
        let (rect, _) = ui.allocate_exact_size(Vec2::new(w, body_h), Sense::hover());
        let mut child = ui.new_child(egui::UiBuilder::new().max_rect(rect).layout(*ui.layout()));
        child.set_clip_rect(rect);
        if packet_detail(&mut child, rec) {
            if let Some(a) = rec.audio.clone() {
                self.cmds.push(Cmd::Play(a));
            }
        }
    }

    fn log_header(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            // The switch that produces everything below it. Decoding the whole
            // span at once is the expensive thing this app does, so it stays a
            // one-click switch rather than a line in a settings modal, and it
            // sits beside the legend that reports what it did.
            if crate::icons::icon_button(
                ui,
                crate::icons::Icon::Decode,
                crate::i18n::t("ui.decode_all"),
                true,
                self.decode_on,
            )
            .clicked()
            {
                self.acts.push(Action::Decode(!self.decode_on));
            }
            ui.add_space(6.0);
            // No row count here. The list keeps the last 500 and drops the
            // rest, so the number stops meaning anything the moment a band
            // gets busy, which is exactly when it would be looked at. The
            // frame total below is a real total and is worth printing.
            if let Some(r) = self.radio {
                use std::sync::atomic::Ordering;
                let narrow = r.status.scan_channels.load(Ordering::Relaxed);
                let wide = r.status.scan_channels_wide.load(Ordering::Relaxed);
                let total = r.status.decoded.load(Ordering::Relaxed);
                let aircraft = r.status.aircraft.load(Ordering::Relaxed);
                ui.add_space(10.0);
                // Several front ends can run on one span now, so this names
                // all of them rather than the first that happens to be on.
                let mut running: Vec<String> = Vec::new();
                if r.status.modes_on.load(Ordering::Relaxed) {
                    running.push("mode s".into());
                }
                if r.status.ais_on.load(Ordering::Relaxed) {
                    running.push("ais".into());
                }
                if r.status.aprs_on.load(Ordering::Relaxed) {
                    running.push("aprs".into());
                }
                if r.status.pocsag_on.load(Ordering::Relaxed) {
                    running.push("pocsag".into());
                }
                if r.status.m17_on.load(Ordering::Relaxed) {
                    running.push("m17".into());
                }
                if narrow > 0 || wide > 0 {
                    running.push(format!("{narrow} ook + {wide} fsk channels"));
                }
                if r.status.sources_on.load(Ordering::Relaxed) {
                    let live = r.status.sources.lock().iter().filter(|e| e.live).count();
                    running.push(if live == 0 {
                        "auto".into()
                    } else {
                        format!("auto, {live} sources")
                    });
                }
                let tracking = r.status.modes_on.load(Ordering::Relaxed)
                    || r.status.ais_on.load(Ordering::Relaxed)
                    || r.status.aprs_on.load(Ordering::Relaxed);
                ui.label(legend(&if running.is_empty() {
                    "decoding off".to_string()
                } else if tracking {
                    format!("{}, {aircraft} tracks, {total} frames", running.join(" + "))
                } else {
                    format!("{}, {total} frames", running.join(" + "))
                }));
            }
            let logged = self
                .radio
                .as_ref()
                .map(|r| r.status.logged.load(std::sync::atomic::Ordering::Relaxed))
                .unwrap_or(0);
            if logged > 0 {
                ui.add_space(10.0);
                ui.label(legend(&format!("{logged} saved")))
                    .on_hover_text(match &self.st.path {
                        Some(d) => format!("appended to {}", d.display()),
                        None => "appended to the packet log".into(),
                    });
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("CLEAR").clicked() {
                    self.st.decodes.clear();
                    self.st.selected = None;
                }
                if ui.button("SETTINGS").clicked() {
                    self.acts.push(Action::Open(Settings::PacketLog));
                }

                // Which front end runs on which frequency. Named rather than
                // drawn: it sits in a row of named buttons with room for a
                // word, and no glyph says "the table deciding what decodes
                // where" without being learned first.
                if ui.button("SCANNERS").clicked() {
                    self.acts.push(Action::Open(Settings::Scanners));
                }
            });
        });
    }

    /// Column headings and their widths in pixels. Fixed rather than sized to
    /// the content: a table whose columns resize as packets arrive is a table
    /// that moves under the pointer, and the last column absorbs the slack.
    const COLS: [(&'static str, f32); 8] = [
        ("no", 40.0),
        ("time", 64.0),
        ("frequency", 96.0),
        ("mod", 70.0),
        ("rssi", 48.0),
        ("snr", 44.0),
        ("protocol", 140.0),
        ("len", 38.0),
    ];

    /// Width the table needs before the info column starts being squeezed.
    fn table_width() -> f32 {
        Self::COLS.iter().map(|(_, w)| w).sum::<f32>() + 340.0
    }

    /// The heading strip, above the rows and outside their vertical scroll, so
    /// it cannot scroll away from what it labels.
    fn log_header_row(&self, ui: &mut egui::Ui, w: f32) {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(w, widgets::ROW_H), Sense::hover());
        let p = ui.painter_at(rect);
        let mut x = rect.left();
        for (name, cw) in Self::COLS {
            widgets::cell(&p, rect, x, cw, name, theme::LEGEND);
            x += cw;
        }
        widgets::cell(&p, rect, x, rect.right() - x, "info", theme::LEGEND);
        p.line_segment(
            [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
            Stroke::new(1.0, theme::ETCH),
        );
    }

    fn log_rows(&mut self, ui: &mut egui::Ui, width: f32) {
        if self.st.decodes.is_empty() {
            // What is actually running here, rather than a claim about
            // sweeping the span that has not been true since the front end
            // became a table lookup.
            let running = self.scanners.active(self.center, self.rate);
            let waiting = match (self.decode_on, running.as_slice()) {
                (false, _) => "decoding is off".to_string(),
                (true, []) => {
                    "no scanner covers this span: press SCAN to add one".to_string()
                }
                (true, blocks) => {
                    let names: Vec<&str> = blocks.iter().map(|s| s.name.as_str()).collect();
                    format!("{} running, nothing heard yet", names.join(", "))
                }
            };
            ui.label(legend(&waiting));
            return;
        }
        let t0 = self.st.decodes.first().map(|l| l.rec.at);
        let mut clicked = None;

        // Striping counts the rows actually drawn, not their place in the
        // list: with unknowns hidden the drawn rows are not contiguous in
        // it, and striping on the list index made the shading of a row jump
        // as the hidden rows above it scrolled past.
        let mut shown = 0usize;
        for log in self.st.decodes.iter() {
            let rec = &log.rec;
            if !self.st.show_unknown && !rec.is_known() {
                continue;
            }
            let n = shown;
            shown += 1;
            // Every row is the same height and every column the same width, so
            // nothing reflows as packets arrive or the pointer moves over
            // them. The whole row is one hit target, painted rather than built
            // from widgets, which is also what keeps a five hundred row list
            // cheap to draw.
            let (rect, resp) =
                ui.allocate_exact_size(Vec2::new(width, widgets::ROW_H), Sense::click());
            if !ui.is_rect_visible(rect) {
                continue;
            }
            let on = self.st.selected == Some(log.id);
            let p = ui.painter_at(rect);
            if on {
                p.rect_filled(rect, 0.0, theme::ETCH);
            } else if resp.hovered() {
                p.rect_filled(rect, 0.0, Color32::from_rgb(0x2A, 0x2E, 0x35));
            } else if n % 2 == 1 {
                p.rect_filled(rect, 0.0, Color32::from_rgb(0x24, 0x27, 0x2D));
            }
            if resp.clicked() {
                clicked = Some(log.id);
            }
            if resp.hovered() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
            }

            let col = row_color(rec);
            // Seconds since the first packet in the list, the way a capture is
            // timed rather than a wall clock, so two transmissions can be
            // compared without arithmetic.
            let secs = t0
                .map(|t0| rec.at.saturating_duration_since(t0).as_secs_f64())
                .unwrap_or(0.0);
            let text = [
                (format!("{:>4}", log.id), col),
                (format!("{secs:8.3}"), theme::LEGEND),
                (fmt_hz(rec.freq), theme::TRACE),
                (rec.modulation.to_string(), theme::LEGEND),
                (fmt_db(rec.rssi_dbfs), level_color(rec.rssi_dbfs)),
                (fmt_db(rec.snr_db), theme::LEGEND),
                (rec.model.clone(), col),
                (format!("{:>4}", rec.bytes.len()), theme::LEGEND),
            ];
            let mut x = rect.left();
            for ((t, c), (_, cw)) in text.iter().zip(Self::COLS) {
                widgets::cell(&p, rect, x, cw, t, *c);
                x += cw;
            }
            widgets::cell(&p, rect, x, rect.right() - x, &rec.detail, theme::VALUE);
        }

        if let Some(id) = clicked {
            // Clicking the selected packet again closes the dump.
            self.st.selected = (self.st.selected != Some(id)).then_some(id);
        }
    }
}
